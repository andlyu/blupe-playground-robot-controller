"""In-memory i2rt-shaped driver. No CAN imports, sockets, or hardware fallback.

Models two producer threads, delayed feedback, bus ownership and motor-off ACKs.
This is a contract simulator, not an emulator of the vendor CAN protocol.
"""
import threading
import time

from types import SimpleNamespace
import numpy as np
from YAM_control.motor_shutdown import disable_motorchain


class SimulatedBus:
    def __init__(self):
        self.owners = {}
        self.positions = {}
        self.robots = []
        self.events = []
        self.freeze_feedback = False
        self.raised_elbow_bias_rad = 0.0
        self.joint_bias_rad = {}  # per-channel six-joint tracking error for regressions
        self.fail_off_ids = set()
        self.shutdown_gate = threading.Event()
        self.shutdown_gate.set()
        self.lock = threading.RLock()

    def factory(self, *, channel, **kwargs):
        with self.lock:
            if channel in self.owners:
                raise RuntimeError(f'duplicate driver owns {channel}')
            robot = SimulatedRobot(self, channel)
            self.owners[channel] = robot
            self.robots.append(robot)
            return robot

    def cleanup(self):
        self.shutdown_gate.set()
        self.fail_off_ids.clear()
        for robot in self.robots:
            if not robot.motor_chain.closed:
                result = disable_motorchain(robot)
                if not result['ok']:
                    raise RuntimeError(result)


class SimulatedChain:
    def __init__(self, robot):
        self.robot = robot
        self.running = True
        self.closed = False
        self.enabled = set(range(1, 8))
        self._command = robot.target.copy()
        self._last_runtime_feedback = {}
        self.motor_interface = self
        self._control_thread = threading.Thread(target=self._loop, name='sim-CAN', daemon=True)
        self._control_thread.start()

    def __len__(self):
        return 7

    def _loop(self):
        while self.running:
            now = time.monotonic()
            for motor_id in range(1, 8):
                self.robot._yam_command_timing.begin(motor_id, now)
                self._last_runtime_feedback[motor_id] = {'monotonic': now, 'code': '0x1'}
            with self.robot.lock:
                # Feedback follows commands only while the CAN worker runs.
                if not self.robot.bus.freeze_feedback:
                    target = self._command.copy()
                    bias = self.robot.bus.joint_bias_rad.get(self.robot.channel)
                    if bias is not None:
                        target[:6] += bias
                    if target[2] > 1.2:
                        target[2] += self.robot.bus.raised_elbow_bias_rad
                    self.robot.q[:] += np.clip(target - self.robot.q, -.02, .02)
            threading.Event().wait(.002)

    def read_states(self):
        return [SimpleNamespace(id=i, error_code='0x1' if i in self.enabled else '0x0') for i in range(1,8)]

    def stop_thread(self, timeout=2.):
        if not self.robot.bus.shutdown_gate.wait(timeout):
            raise RuntimeError('simulated shutdown timeout')
        self.running = False
        self._control_thread.join(timeout)
        if self._control_thread.is_alive():
            raise RuntimeError('CAN worker still alive')
        self.robot.bus.events.append((self.robot.channel, 'can_join'))

    def motor_off(self, mid):
        if self._control_thread.is_alive() or self.robot._server_thread.is_alive():
            raise RuntimeError('motor_off raced a producer thread')
        if mid in self.robot.bus.fail_off_ids:
            raise RuntimeError(f'missing motor-off acknowledgement: {mid}')
        self.enabled.discard(mid)
        self.robot.bus.events.append((self.robot.channel, 'off', mid))

    def close(self):
        if self.enabled or self._control_thread.is_alive():
            raise RuntimeError('closing an enabled or busy bus')
        self.closed = True
        with self.robot.bus.lock:
            self.robot.bus.owners.pop(self.robot.channel, None)
        self.robot.bus.events.append((self.robot.channel, 'close'))


class SimulatedRobot:
    chain_class = SimulatedChain
    def __init__(self, bus, channel):
        self.bus, self.channel = bus, channel
        self.lock = threading.RLock()
        self.q = bus.positions.setdefault(channel, np.zeros(7))
        self.target = self.q.copy()
        self.commands = []
        self.command_times = []
        self._stop_event = threading.Event()
        from YAM_control.control_timing import CommandTiming
        self._yam_command_timing = CommandTiming()
        self._server_thread = threading.Thread(target=self._control, name='sim-controller', daemon=True)
        self.motor_chain = self.chain_class(self)
        self._server_thread.start()

    def _control(self):
        while not self._stop_event.wait(.002):
            with self.lock:
                self.motor_chain._command[:] = self.target
        self.bus.events.append((self.channel, 'robot_join'))

    def get_observations(self):
        with self.lock:
            return dict(joint_pos=self.q[:6].copy(), gripper_pos=self.q[6:].copy())

    def command_joint_pos(self, value):
        if self._stop_event.is_set() or self.motor_chain.closed:
            raise RuntimeError('command sent to stopped driver')
        with self.lock:
            self.target[:] = value
            self.commands.append(self.target.copy())
            self.command_times.append(time.monotonic())
