"""No hardware: dispatch completion must retain all native command checks."""
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from blupe_controller.runtime.YAM_control.i2rt_bimanual_adapter import (
    AdapterState, I2RTBimanualAdapter, TrajectoryInterrupted,
)


class Robot:
    def __init__(self):
        self.position = np.zeros(7)
        self.commands = []
        self._server_thread = SimpleNamespace(is_alive=lambda: True)
        self.motor_chain = SimpleNamespace(running=True, read_states=lambda: [
            SimpleNamespace(id=i, error_code='0x1') for i in range(1, 8)])

    def get_observations(self):
        return dict(joint_pos=self.position[:6].copy(), gripper_pos=self.position[6:].copy())

    def command_joint_pos(self, value):
        self.commands.append(np.asarray(value).copy())


def setup():
    robots, disabled = [Robot(), Robot()], []
    pending = iter(robots)
    adapter = I2RTBimanualAdapter(left_channel='can0', right_channel='can1',
        left_gripper_limits=(6.3, 1.15), right_gripper_limits=(6.4, 1.14),
        robot_factory=lambda **kwargs: next(pending), robot_disabler=disabled.append,
        channel_ready=lambda channel: True)
    adapter.launch()
    for robot in robots: robot.commands.clear()
    return adapter, robots, disabled


def test_dispatch_completion_reports_measured_lag_without_settling_claim():
    adapter, robots, disabled = setup()
    checked = []
    target = (.1,)*12
    snap = adapter.execute((target,), cadence_hz=1000, wait_for_settle=False,
        validate_waypoint=lambda i, previous, point: checked.append(point) or point)
    assert snap.left.joints_rad == (0.,)*6
    assert adapter.last_command['joints'] == target
    assert adapter.state == AdapterState.STOPPED
    assert checked == [target] and not disabled
    assert all(len(robot.commands) == 1 for robot in robots)


def test_cancel_after_dispatch_still_interrupts_before_completion():
    adapter, robots, disabled = setup()
    cancel = threading.Event()
    with pytest.raises(TrajectoryInterrupted):
        adapter.execute(((.1,)*12,), cadence_hz=1000, wait_for_settle=False,
            cancel_event=cancel, on_waypoint=lambda i: cancel.set())
    assert adapter.state == AdapterState.STOPPED and not disabled


def test_post_dispatch_motor_fault_is_not_hidden_by_no_settle_mode():
    adapter, robots, disabled = setup()
    def fault(i): robots[1].motor_chain.running = False
    with pytest.raises(RuntimeError, match='motor chain is not running'):
        adapter.execute(((.1,)*12,), cadence_hz=1000, wait_for_settle=False, on_waypoint=fault)
    assert adapter.state == AdapterState.FAULT and len(disabled) == 2


def test_runtime_validator_rejection_never_reaches_a_motor():
    adapter, robots, disabled = setup()
    def reject(*args): raise ValueError('guard rejected')
    with pytest.raises(ValueError, match='guard rejected'):
        adapter.execute(((.1,)*12,), cadence_hz=1000, wait_for_settle=False,
                        validate_waypoint=reject)
    assert all(not robot.commands for robot in robots)


def test_existing_callers_still_request_measured_settling_by_default():
    import inspect
    assert inspect.signature(I2RTBimanualAdapter.execute).parameters['wait_for_settle'].default is True


def test_no_settle_is_an_explicit_boolean():
    adapter, robots, _ = setup()
    with pytest.raises(ValueError, match='boolean'):
        adapter.execute(((.1,)*12,), cadence_hz=1000, wait_for_settle='false')
    assert all(not robot.commands for robot in robots)
