"""Local run deadlines with a simulated driver; no hardware or network."""
import asyncio
import threading
import time
from unittest.mock import Mock, patch

import pytest

from blupe_controller.cloud import CloudBridge
from blupe_controller.runtime.YAM_control.session_api_sim_client import SessionApiSimClient
from test_cloud import Driver

IDS = dict(session_id='session', episode_id='ep_test', lease_id='lease')

@pytest.fixture
def bridge():
    driver = Driver()
    poses = Mock(); poses.poses = {}
    config = dict(robot_id='robot-abecb4cd868ab24b', hardware='so101',
                  api='https://example.com', token_file='/unused', cameras={'front':0}, settings={})
    with patch('blupe_controller.cloud.SessionApiSimClient'):
        b = CloudBridge(driver, config, poses)
    b.driver.hold = Mock(wraps=driver.hold)
    b.return_home = Mock()
    b.authorize = Mock()
    yield b
    b.pause()


def prepare(b, duration, ids=IDS):
    b.ready = True
    return b.api_prepare_session({**ids, 'run_duration_s':duration})


def command(ids=IDS):
    return {**ids, 'step_id':0, 'command_id':'one', 'left_joints_deg':[1]*5,
            'right_joints_deg':[], 'left_gripper':.5}


def wait_expired(b):
    end = time.monotonic()+2
    while (b.lease is not None or b.returning_home) and time.monotonic()<end:
        time.sleep(.005)
    assert b.lease is None


def messages(b):
    return [call.args[0] for call in b.client.send.call_args_list]


def test_watchdog_expires_while_model_thinks_without_network_callbacks(bridge):
    b = bridge
    prepare(b, .04)
    recorder = Mock(); b.recorder = recorder; b.auto_queue = True
    wait_expired(b)
    b.driver.hold.assert_not_called()
    b.return_home.assert_called_once()
    recorder.finish.assert_called_once_with('policy_runtime_timeout')
    assert b.last_stop_reason == 'policy_runtime_timeout'
    assert not b.pending and not b.ready and b.auto_queue
    assert b.driver.moves == []
    assert any(m['type']=='safety_abort' and m['code']=='policy_runtime_timeout' for m in messages(b))


def test_expiry_during_trajectory_stops_dispatch_and_aborts(bridge):
    b = bridge
    prepare(b, .15)
    payload = {**IDS, 'trajectory_id':'trajectory'}
    with b.lock:
        b.launch(payload, [([i]*5, .5) for i in range(6)], True)
    wait_expired(b)
    moves = len(b.driver.moves)
    time.sleep(.35)
    assert 0 < moves < 6 and len(b.driver.moves)==moves
    b.driver.hold.assert_not_called()
    assert any(m.get('trajectory_id')=='trajectory' and m.get('status')=='aborted'
               and m.get('code')=='policy_runtime_timeout' for m in messages(b))
    assert not any(m.get('status')=='completed' for m in messages(b))


def test_deadline_also_expires_during_settle(bridge):
    b = bridge
    prepare(b, .04)
    b.api_handle_joint_command(command())
    wait_expired(b)
    assert len(b.driver.moves)==1
    assert any(m.get('command_id')=='one' and m.get('reason')=='policy_runtime_timeout' for m in messages(b))
    assert not any(m.get('status')=='executed' for m in messages(b))


def test_dispatch_checks_deadline_even_if_watchdog_has_not_run(bridge):
    b = bridge
    with patch.object(b,'watch_deadline'), patch('blupe_controller.cloud.time.monotonic',return_value=100.):
        prepare(b, 180)
        assert b.run_deadline==280.
    with patch('blupe_controller.cloud.time.monotonic',return_value=280.), patch('blupe_controller.cloud.time.time',return_value=0.):
        result=b.api_handle_joint_command(command())
    assert result[0]['reason']=='policy_runtime_timeout'
    assert b.driver.moves==[]
    b.driver.hold.assert_not_called()


def test_expired_worker_cannot_move_even_before_watchdog_runs(bridge):
    b=bridge
    with patch.object(b,'watch_deadline'), patch('blupe_controller.cloud.time.monotonic',return_value=10.):
        prepare(b, 180)
    generation=b.generation
    with patch('blupe_controller.cloud.time.monotonic',return_value=190.):
        b.execute(command(),[([1]*5,.5)],False,generation)
    assert not b.driver.moves
    b.driver.hold.assert_not_called()


def test_late_commands_and_subsequent_session_are_isolated(bridge):
    b=bridge
    prepare(b,.03); old_generation=b.generation;old_deadline=b.run_deadline
    wait_expired(b)
    assert b.api_handle_joint_command(command())[0]['reason']=='policy_runtime_timeout'
    assert b.api_handle_joint_trajectory({**IDS,'trajectory_id':'late'})[0]['code']=='policy_runtime_timeout'
    new_ids=dict(session_id='new',episode_id='ep_new',lease_id='newlease')
    prepare(b,180,new_ids)
    assert b.run_deadline > old_deadline
    # Delayed old watchdog, old worker, and old completion cannot stop or move the new session.
    b.watch_deadline(old_generation,time.monotonic()-1,threading.Event())
    b.execute(command(),[([2]*5,.5)],False,old_generation)
    b.api_handle_stop({**IDS,'reason':'policy_complete'})
    rejected=b.api_handle_joint_command(command())[0]
    assert rejected['session_id']==IDS['session_id']
    assert b.lease==new_ids and b.driver.moves==[]
    assert b.api_handle_joint_command(command(new_ids))==[]
    end=time.monotonic()+1
    while b.pending and time.monotonic()<end:time.sleep(.01)
    assert b.driver.moves==[[1]*5] and b.lease==new_ids
    b.driver.hold.assert_not_called()


@pytest.mark.parametrize('duration',[None,True,0,-1,float('inf'),float('nan'),'180'])
def test_missing_or_invalid_duration_fails_closed(bridge,duration):
    assert prepare(bridge,duration) is None
    assert bridge.lease is None and not bridge.ready
    assert bridge.driver.moves==[]
    assert messages(bridge)[-1]['code']=='invalid_run_duration'


def test_timeout_wins_over_simultaneous_policy_complete(bridge):
    b=bridge
    with patch.object(b,'watch_deadline'),patch('blupe_controller.cloud.time.monotonic',return_value=10.):
        prepare(b,180)
    b.auto_queue=True
    with patch('blupe_controller.cloud.time.monotonic',return_value=190.):
        b.api_handle_stop({**IDS,'reason':'policy_complete'})
    wait_expired(b)
    assert b.last_stop_reason=='policy_runtime_timeout'
    b.return_home.assert_called_once()
    assert b.auto_queue


def test_hold_failure_still_revokes_records_and_reports(bridge):
    b=bridge
    with patch.object(b,'watch_deadline'):
        prepare(b,180)
    b.recorder=Mock();recorder=b.recorder
    b.return_home=None
    b.driver.hold.side_effect=OSError('servo unavailable')
    with b.lock:
        b.run_deadline=time.monotonic()-1
        assert b.expire_run()
    assert b.lease is None and not b.pending
    recorder.finish.assert_called_once_with('policy_runtime_timeout')
    assert messages(b)[-1]['code']=='policy_runtime_timeout'


def test_transport_passes_handoff_duration_unchanged(bridge):
    bridge.ready=True
    client=SessionApiSimClient(bridge,'wss://example.com','test',None)
    payload={'type':'prepare_session',**IDS,'run_duration_s':180}
    asyncio.run(client._handle_message(payload))
    assert bridge.run_duration_s==180
    assert 179 < bridge.run_deadline-time.monotonic() <= 180


def test_expired_session_cannot_restart_its_clock(bridge):
    prepare(bridge,.02); wait_expired(bridge)
    assert prepare(bridge,180,{**IDS,'lease_id':'replacement'}) is None
    assert bridge.lease is None


def test_timeout_manifest_is_finalized(tmp_path,bridge):
    from blupe_controller.recording import EpisodeRecorder
    b=bridge; b.config['settings']['recording_root']=str(tmp_path)
    with patch.object(EpisodeRecorder,'frame',return_value=('front',(b'\xff\xd8test',time.monotonic(),1))):
        prepare(b,.15)
        recorder=b.recorder
        wait_expired(b);recorder.thread.join(2)
    import json
    meta=json.loads((recorder.path/'manifest.json').read_text())
    assert meta['status']=='finalized' and meta['rows']>0
    assert meta['outcome']=='policy_runtime_timeout' and meta['run_duration_s']==.15
    b.api_connection_changed(False,'offline')
    assert b.last_stop_reason=='policy_runtime_timeout'
