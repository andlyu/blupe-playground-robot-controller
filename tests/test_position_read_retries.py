"""Exercise controller policy and the installed LeRobot packet retry loop; no IO."""
from unittest.mock import Mock, patch, call
import pytest
from blupe_controller.so101 import retry_position_reads, NAMES
import test_so101


def test_policy_only_changes_position_reads_and_preserves_arguments():
    bus=Mock();original=bus.sync_read;write=bus.sync_write
    retry_position_reads(bus)
    bus.sync_read('Present_Position', ['wrist_roll'], normalize=False)
    original.assert_called_once_with('Present_Position',['wrist_roll'],normalize=False,num_retry=0)
    bus.sync_read('Torque_Enable',normalize=False,num_retry=0)
    assert original.call_args.kwargs=={'normalize':False,'num_retry':0}
    assert bus.sync_write is write


@pytest.fixture(autouse=True)
def retry_sleep():
    with patch('blupe_controller.so101.time.sleep') as sleep:
        yield sleep


@pytest.fixture
def fixture():
    case=test_so101.LeRobotTests();case.setUp()
    try:yield case
    finally:case.tearDown()


def packet_bus(case, results):
    # Use the real SDK retry implementation against a fake packet reader.
    motors=pytest.importorskip('lerobot.motors.motors_bus')
    bus=case.robot.bus
    bus._is_comm_success.side_effect=lambda status:status==0
    bus.sync_reader.txRxPacket.side_effect=results
    bus.sync_reader.getData.return_value=0
    bus.packet_handler.getTxRxResult.return_value='No status packet'
    def read(name, *, num_retry=0):
        values,_=motors.MotorsBus._sync_read(bus,56,2,list(range(1,7)),num_retry=num_retry,
                                           err_msg=f'Failed after {num_retry+1} tries.')
        return {name:values[i+1] for i,name in enumerate(NAMES)}
    bus.sync_read=read
    case.robot.get_observation.side_effect=lambda:{name+'.pos':v for name,v in bus.sync_read('Present_Position').items()}
    return bus


@pytest.mark.parametrize('misses',[1,2,3])
def test_recovers_without_fault_after_up_to_three_missed_packets(fixture,misses):
    bus=packet_bus(fixture,[-1]*misses+[0])
    fixture.driver.connect()
    assert fixture.driver.mode=='readonly' and fixture.driver.error==''
    assert bus.sync_reader.txRxPacket.call_count==misses+1
    fixture.robot.send_action.assert_not_called()
    bus.enable_torque.assert_not_called()


def test_four_failures_exhaust_budget_and_latch_fault(fixture):
    fixture.driver.connect()
    bus=packet_bus(fixture,[-1]*4)
    retry_position_reads(bus)
    with pytest.raises(ConnectionError,match='4 tries'):
        fixture.driver.state()
    assert fixture.driver.mode=='fault'
    assert bus.sync_reader.txRxPacket.call_count==4
    bus.sync_reader.getData.assert_not_called()
    # A later good read must not clear the latched safety fault.
    bus.sync_reader.txRxPacket.side_effect=[0]
    assert fixture.driver.state()['mode']=='fault'
    fixture.robot.send_action.assert_not_called()


def test_feedback_retry_does_not_repeat_the_motion_write(fixture):
    fixture.driver.connect();fixture.driver.enable()
    bus=packet_bus(fixture,[-1,-1,-1,0]);retry_position_reads(bus)
    fixture.driver.move([1]*5,.5)
    fixture.robot.send_action.assert_called_once()
    assert bus.sync_reader.txRxPacket.call_count==4
    assert fixture.driver.mode=='active'


def test_pre_command_read_uses_same_budget(fixture):
    fixture.driver.connect();fixture.driver.enable()
    bus=packet_bus(fixture,[-1,-1,-1,0,0]);retry_position_reads(bus)
    def send(action):
        # Same pre-write Present_Position call as LeRobot's max_relative_target gate.
        bus.sync_read('Present_Position')
        bus.sync_write('Goal_Position',action)
        return action
    fixture.robot.send_action.side_effect=send
    fixture.driver.move([1]*5,.5)
    assert bus.sync_reader.txRxPacket.call_count==5
    assert fixture.robot.send_action.call_count==1
    assert bus.sync_write.call_count==2  # enable preload, then one motion write


def test_write_failure_is_not_retried(fixture):
    fixture.driver.connect();fixture.driver.enable()
    fixture.robot.send_action.side_effect=ConnectionError('write failed')
    with pytest.raises(ConnectionError,match='write failed'):
        fixture.driver.move([1]*5,.5)
    fixture.robot.send_action.assert_called_once()
    assert fixture.driver.mode=='fault'


def test_invalid_feedback_fails_immediately(fixture):
    fixture.driver.connect()
    fixture.robot.get_observation.reset_mock()
    fixture.robot.get_observation.return_value={**{name+'.pos':0 for name in NAMES}, 'shoulder_pan.pos':float('nan')}
    with pytest.raises(ValueError,match='finite'):fixture.driver.state()
    fixture.robot.get_observation.assert_called_once()
    assert fixture.driver.mode=='fault'


def test_exhausted_pre_command_read_does_not_write_a_target(fixture):
    fixture.driver.connect();fixture.driver.enable()
    bus=packet_bus(fixture,[-1]*4);retry_position_reads(bus)
    bus.sync_write.reset_mock()
    def send(action):
        bus.sync_read('Present_Position')
        bus.sync_write('Goal_Position',action)
        return action
    fixture.robot.send_action.side_effect=send
    with pytest.raises(ConnectionError):fixture.driver.move([1]*5,.5)
    assert bus.sync_reader.txRxPacket.call_count==4
    bus.sync_write.assert_not_called()
    assert fixture.driver.mode=='fault'


@pytest.mark.parametrize('failures',[0,1,2,3,4])
def test_delay_occurs_only_between_failed_attempts(failures,retry_sleep):
    bus=Mock();original=bus.sync_read;events=[]
    def read(*args,**kwargs):
        events.append('read')
        if events.count('read')<=failures:raise ConnectionError('no response')
        return {'gripper':.5}
    original.side_effect=read
    retry_sleep.side_effect=lambda seconds:events.append(('sleep',seconds))
    retry_position_reads(bus)
    if failures==4:
        with pytest.raises(ConnectionError,match='4 tries'):bus.sync_read('Present_Position')
    else:
        assert bus.sync_read('Present_Position')=={'gripper':.5}
    attempts=min(failures+1,4)
    assert events==['read']+sum(([('sleep',.5),'read'] for _ in range(attempts-1)),[])
    assert retry_sleep.call_count==min(failures,3)
    assert all(c.kwargs['num_retry']==0 for c in original.call_args_list)


def test_other_errors_fail_without_retry_or_delay(retry_sleep):
    bus=Mock();original=bus.sync_read;original.side_effect=ValueError('invalid value')
    retry_position_reads(bus)
    with pytest.raises(ValueError):bus.sync_read('Present_Position')
    original.assert_called_once();retry_sleep.assert_not_called()
