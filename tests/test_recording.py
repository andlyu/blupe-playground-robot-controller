import json
import time
import math
from unittest.mock import Mock, patch
from blupe_controller.recording import EpisodeRecorder
from blupe_controller.cloud import CloudBridge


def test_recording_native_shape_freshness_and_finalization(tmp_path):
    config = {'robot_id':'robot-maker','hardware':'bimanual_so101',
              'cameras':{'overhead':6,'side':3,'left':4,'right':0},'settings':{}}
    measured = {'joints_deg':[30.]*10,'gripper':[.1,.2]}
    command = {'joints':[.6]*10,'grippers':[.2,.3],'monotonic':time.monotonic()}
    recorder = EpisodeRecorder(tmp_path,config,'ep_test','Move block',[f'j{i}' for i in range(10)],
                               lambda:(measured,command,time.monotonic()))
    recorder.frame = lambda role, device: (role,(b'\xff\xd8test',time.monotonic(),1))
    recorder.start()
    deadline = time.monotonic()+2
    while recorder.meta['rows'] < 2 and time.monotonic() < deadline: time.sleep(.01)
    recorder.finish('user_requested');recorder.thread.join(2)
    meta = json.loads((recorder.path/'manifest.json').read_text())
    rows = [json.loads(line) for line in (recorder.path/'samples.jsonl').read_text().splitlines()]
    assert meta['status']=='finalized' and meta['robot_id']=='robot-maker'
    assert meta['outcome']=='user_requested' and len(rows)>=2
    assert len(rows[0]['measured_joints'])==10 and rows[0]['measured_joints'][0]==math.radians(30)
    assert rows[0]['training_valid'] and rows[0]['action_joints']==[.6]*10
    assert all(role+'_image_path' in rows[0] for role in config['cameras'])
    stale = recorder.row({role:(b'\xff\xd8test',time.monotonic()-2,1) for role in config['cameras']},
                         measured,command,time.monotonic()-2,time.monotonic())
    assert not stale['training_valid']


def test_cloud_recording_starts_only_on_assignment_and_finishes_on_disconnect(tmp_path):
    driver=Mock(joint_names=['j']*5)
    driver.state.return_value={'mode':'active','joints_deg':[0]*5,'gripper':.2}
    config={'robot_id':'robot-maker','hardware':'so101','api':'https://example.com',
            'token_file':'/tmp/unused','cameras':{'front':0},'settings':{'recording_root':str(tmp_path)}}
    poses=Mock();poses.poses={}
    with patch('blupe_controller.cloud.SessionApiSimClient'), patch('blupe_controller.recording.EpisodeRecorder') as factory:
        bridge=CloudBridge(driver,config,poses)
        ids={'session_id':'s','episode_id':'ep_test','lease_id':'l','task':'Move block'}
        assert bridge.api_prepare_session(ids) is None
        factory.assert_not_called()
        bridge.ready=True;bridge.api_prepare_session(ids)
        factory.return_value.start.assert_called_once()
        snapshot=factory.call_args.args[-1]()
        assert snapshot[0]['joints_deg']==[0]*5
        bridge.api_connection_changed(False)
        factory.return_value.finish.assert_called_once_with('connection_lost')
        driver.hold.assert_called_once()
