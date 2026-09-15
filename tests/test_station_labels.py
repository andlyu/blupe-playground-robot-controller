"""Public status describes both SO101 variants without changing admission gates."""
from types import SimpleNamespace
from unittest.mock import Mock, patch
import pytest
from blupe_controller.cloud import CloudBridge


@pytest.fixture(params=['so101', 'bimanual_so101'])
def bridge(request):
    count = 10 if request.param == 'bimanual_so101' else 5
    gripper = [.5, .5] if count == 10 else .5
    state = dict(joints_deg=[0.]*count, gripper=gripper, mode='active', error='')
    driver = Mock()
    driver.joint_tolerance_deg = 5.
    driver.state.return_value = state
    poses = SimpleNamespace(poses={'home': dict(joints_deg=[0.]*count, gripper=gripper)})
    config = dict(hardware=request.param, robot_id='test', api='https://example.com',
                  token_file='/unused', cameras={}, settings={})
    with patch('blupe_controller.cloud.SessionApiSimClient'):
        result = CloudBridge(driver, config, poses)
    result.connected = result.auto_queue = result.ready = True
    yield result
    result.pause()


def test_ready_requires_home_auto_queue_and_idle(bridge):
    b = bridge
    assert b.api_station_status_payload()['mode'] == 'READY'
    assert b.api_station_status_payload()['queue_ready'] is True
    for name, blocked in [('auto_queue', False), ('connected', False), ('pending', True),
                          ('returning_home', True), ('lease', {'session_id':'s'})]:
        original = getattr(b, name)
        setattr(b, name, blocked)
        assert b.api_station_status_payload()['queue_ready'] is False
        setattr(b, name, original)
    # Either arm leaving Home must invalidate bimanual readiness.
    b.driver.state()['joints_deg'][-1] = 20.
    assert not b.api_station_status_payload()['homed']
    assert not b.api_station_status_payload()['queue_ready']


def test_manual_authorization_remains_separate_from_public_auto_readiness(bridge):
    bridge.auto_queue = False
    assert bridge.api_station_status_payload()['mode'] == 'STOPPED'
    assert bridge.ready is True  # Publishing status never revokes manual admission.
    bridge.client.request_ready.assert_not_called()
    bridge.driver.move.assert_not_called()


@pytest.mark.parametrize('waiting, expected', [(True, 'MOVING_HOME'), (False, 'PARKING_ZERO')])
def test_cleanup_publishes_actual_destination_and_clears_on_pause(bridge, waiting, expected):
    b = bridge
    b.ready = False
    b.returning_home = True
    b.queued_work = Mock(return_value=waiting)
    def moving(generation):
        status = b.api_station_status_payload()
        assert status['mode'] == expected
        assert not status['settled'] and not status['queue_ready']
        b.pause()
    b.return_home = b.return_zero = moving
    b._next_auto_task(b.generation)
    assert b.cleanup_phase is None
    assert b.api_station_status_payload()['mode'] == 'STOPPED'


def test_parked_auto_queue_is_ready_to_wait_but_not_to_admit_a_run(bridge):
    b = bridge
    b.parked = True
    b.ready = False
    b.driver.state()['mode'] = 'readonly'
    assert b.api_station_status_payload()['mode'] == 'STOPPED_READY'
    assert not b.api_station_status_payload()['queue_ready']
    for field, blocked in [('auto_queue', False), ('connected', False), ('parked', False),
                           ('pending', True), ('returning_home', True), ('ready', True)]:
        original = getattr(b, field)
        setattr(b, field, blocked)
        assert b.api_station_status_payload()['mode'] != 'STOPPED_READY'
        setattr(b, field, original)
    b.driver.state()['error'] = 'motor error'
    assert b.api_station_status_payload()['mode'] != 'STOPPED_READY'
    b.driver.state()['error'] = ''
    b.lease = {'session_id': 'active'}
    assert b.api_station_status_payload()['mode'] == 'EXECUTING'
    b.lease = None
    b.cleanup_phase = 'MOVING_HOME'
    assert b.api_station_status_payload()['mode'] == 'MOVING_HOME'
    b.driver.state()['mode'] = 'fault'
    status = b.api_station_status_payload()
    assert status['mode'] == 'FAULT' and not status['safety']['ok']
