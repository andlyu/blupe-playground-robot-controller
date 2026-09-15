"""Enable from captured Zero using real queue cleanup and bounded pose movement."""
import time
from types import SimpleNamespace
from unittest.mock import Mock, patch
import pytest
from blupe_controller.cloud import CloudBridge
from blupe_controller.operator import Operator
from test_pose_motion import Driver
from test_queue_cleanup import wait_for


@pytest.fixture(params=['so101', 'bimanual_so101'])
def setup(request):
    count = 10 if request.param == 'bimanual_so101' else 5
    driver = Driver()
    driver.q = [0.]*count
    driver.g = [0., 0.] if count == 10 else 0.
    driver.joint_tolerance_deg = .2
    driver._validate_target = Mock()
    original_state = driver.state
    driver.state = lambda: {**original_state(), 'error':''}
    driver.mode = 'readonly'
    driver.enable = Mock(side_effect=lambda: setattr(driver, 'mode', 'active'))
    driver.disable = Mock(side_effect=lambda: setattr(driver, 'mode', 'readonly'))
    config = dict(hardware=request.param, robot_id='test', api='https://example.com',
                  token_file='/unused', cameras={}, settings={})
    operator = Operator(driver, config)
    poses = {'zero':dict(joints_deg=[0.]*count, gripper=driver.g),
             'home':dict(joints_deg=[10.]*count, gripper=driver.g)}
    operator.poses = SimpleNamespace(poses=poses, get=lambda name: poses[name])
    with patch('blupe_controller.cloud.SessionApiSimClient'):
        bridge = CloudBridge(driver, config, operator.poses)
    operator.cloud = bridge
    bridge.return_home = operator.return_home_for_queue
    bridge.return_zero = operator.return_zero_for_queue
    bridge.connected = True
    bridge.queued_work = Mock(return_value=False)
    camera = Mock(); camera.__enter__ = Mock(return_value=camera); camera.__exit__ = Mock()
    camera.read.return_value = b'{"ok":true}'
    with patch('blupe_controller.cloud.urlopen', return_value=camera):
        yield bridge, operator, driver
    operator.action(dict(action='hold'))


def test_zero_waits_without_motor_writes_then_homes_before_admission(setup):
    b, o, d = setup
    o.action(dict(action='auto_queue', enabled=True))
    wait_for(lambda: b.queued_work.called)
    assert b.auto_queue and b.parked and not b.ready
    assert b.api_station_status_payload()['mode'] == 'STOPPED_READY'
    assert d.sent == []
    d.enable.assert_not_called(); d.disable.assert_not_called()
    b.client.request_ready.assert_not_called()
    b.queued_work.return_value = True
    wait_for(lambda: b.ready, timeout=6)
    assert b.api_station_status_payload()['mode'] == 'READY'
    assert b.near(d.state(), o.poses.get('home')['joints_deg'], d.g)
    d.enable.assert_called_once()
    b.client.request_ready.assert_called_once()


def test_pause_after_enabling_zero_prevents_later_motion(setup):
    b, o, d = setup
    o.action(dict(action='auto_queue', enabled=True))
    o.action(dict(action='cloud_pause'))
    b.queued_work.return_value = True
    time.sleep(2.1)
    assert not b.auto_queue and not b.ready
    d.enable.assert_not_called(); assert d.sent == []


def test_active_zero_parks_and_disables_before_waiting(setup):
    b, o, d = setup
    d.mode = 'active'
    o.action(dict(action='auto_queue', enabled=True))
    wait_for(lambda: b.parked)
    d.disable.assert_called_once()
    assert not b.ready and d.mode == 'readonly'
    b.client.request_ready.assert_not_called()


def test_zero_rejects_offline_busy_missing_home_and_wrong_pose(setup):
    b, o, d = setup
    b.connected = False
    with pytest.raises(ValueError): o.action(dict(action='auto_queue', enabled=True))
    b.connected = True; b.pending = True
    with pytest.raises(ValueError): o.action(dict(action='auto_queue', enabled=True))
    b.pending = False; home = o.poses.poses.pop('home')
    with pytest.raises(KeyError): o.action(dict(action='auto_queue', enabled=True))
    o.poses.poses['home'] = home
    d.q[-1] = 5. # Only the second arm leaves Zero in the bimanual case.
    with pytest.raises(ValueError): o.action(dict(action='auto_queue', enabled=True))
    assert not b.auto_queue and not b.ready
    d.enable.assert_not_called(); assert d.sent == []
