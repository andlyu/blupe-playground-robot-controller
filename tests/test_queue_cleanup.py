"""Queue-aware cleanup with simulated arms, real bounded pose motion, no network."""
import threading
import time
from unittest.mock import Mock, patch
import pytest
from blupe_controller.cloud import CloudBridge
from blupe_controller.operator import Operator
from test_pose_motion import Driver


def wait_for(predicate, timeout=5):
    end = time.monotonic()+timeout
    while not predicate() and time.monotonic()<end: time.sleep(.01)
    assert predicate()


@pytest.fixture
def setup():
    driver = Driver()
    driver.enable = Mock(side_effect=lambda: setattr(driver, 'mode', 'active'))
    driver.disable = Mock(side_effect=lambda: setattr(driver, 'mode', 'readonly'))
    config = dict(hardware='so101', robot_id='test', api='https://example.com',
                  token_file='/unused', cameras={}, settings={})
    operator = Operator(driver, config)
    operator.poses = Mock()
    operator.poses.get.side_effect = lambda name: dict(joints_deg=[3. if name=='home' else 0.]*5, gripper=0.)
    with patch('blupe_controller.cloud.SessionApiSimClient'):
        bridge = CloudBridge(driver, config, operator.poses)
    operator.cloud = bridge
    bridge.return_home = operator.return_home_for_queue
    bridge.return_zero = operator.return_zero_for_queue
    bridge.auto_queue = True
    bridge.authorize = Mock()
    bridge.queued_work = Mock(return_value=False)
    yield bridge, operator, driver
    operator.action(dict(action='hold'))


def stop(b, reason):
    b.lease = dict(session_id='s', episode_id='e', lease_id='l')
    b._stop_run({**b.lease, 'reason':reason})


@pytest.mark.parametrize('reason', ['user_requested','policy_complete','session_timeout','policy_runtime_timeout'])
@pytest.mark.parametrize('queued', [False, True])
def test_stop_parks_or_homes_without_disarming_queue(setup, reason, queued):
    b, o, d = setup
    b.queued_work.return_value = queued
    stop(b, reason)
    wait_for(lambda: not b.returning_home)
    assert b.auto_queue and b.lease is None
    if queued:
        b.authorize.assert_called_once()
        d.disable.assert_not_called()
        assert d.q == [3.]*5
    else:
        d.disable.assert_called_once()
        b.authorize.assert_not_called()
        assert b.parked and d.mode == 'readonly' and d.q == [0.]*5


def test_later_queue_item_enables_and_homes_before_ready(setup):
    b, o, d = setup
    stop(b, 'policy_complete')
    wait_for(lambda: b.parked)
    b.queued_work.return_value = True
    wait_for(lambda: b.authorize.called)
    d.enable.assert_called_once()
    assert d.q == [3.]*5 and d.mode == 'active' and not b.parked


def test_pause_while_parked_prevents_wakeup(setup):
    b, o, d = setup
    stop(b, 'policy_complete')
    wait_for(lambda: b.parked)
    o.action(dict(action='cloud_pause'))
    b.queued_work.return_value = True
    time.sleep(2.1)
    d.enable.assert_not_called()
    b.authorize.assert_not_called()
    assert not b.auto_queue


def test_pause_during_zero_does_not_disable_or_resume(setup):
    b, o, d = setup
    d.q = [20.]*5
    stop(b, 'user_requested')
    wait_for(lambda: bool(d.sent))
    o.action(dict(action='hold'))
    wait_for(lambda: o.motion is None)
    d.disable.assert_not_called()
    b.authorize.assert_not_called()
    assert not b.auto_queue


def test_queue_network_failure_parks_without_accepting_work(setup):
    b, o, d = setup
    b.queued_work.side_effect = OSError('offline')
    stop(b, 'policy_complete')
    wait_for(lambda: b.parked)
    d.disable.assert_called_once()
    b.authorize.assert_not_called()
    assert b.auto_queue


def test_torque_off_failure_blocks_future_runs(setup):
    b, o, d = setup
    d.disable.side_effect = ValueError('torque still on')
    stop(b, 'policy_complete')
    wait_for(lambda: not b.returning_home)
    assert not b.auto_queue and not b.parked and 'torque still on' in b.error
    b.authorize.assert_not_called()


def test_fault_stop_does_not_park(setup):
    b, o, d = setup
    stop(b, 'hardware_fault')
    assert not b.auto_queue and not b.returning_home
    assert d.sent == []
    d.disable.assert_not_called()
