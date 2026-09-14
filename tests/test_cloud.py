import time
import io
import unittest
from unittest.mock import Mock, patch
from blupe_controller.cloud import CloudBridge
from blupe_controller.so101 import SO101Driver


class Driver:
    _action = staticmethod(SO101Driver._action)
    def __init__(self):
        self.joints=[0]*5;self.gripper=.5;self.moves=[];self.held=False
    def state(self):return {'joints_deg':self.joints,'gripper':self.gripper,'mode':'active','error':''}
    def _validate_target(self,j,g):
        if not isinstance(j,list) or len(j)!=5 or any(abs(v)>90 for v in j):raise ValueError('invalid target')
    def move(self,j,g):
        self.joints=j;self.gripper=g;self.moves.append(j)
        return {**self.state(),'sent_action':self._action(j,g)}
    def hold(self):self.held=True


class CloudTests(unittest.TestCase):
    def setUp(self):
        self.driver=Driver()
        self.poses=Mock();self.poses.poses={}
        config={'robot_id':'test','api':'https://example.com','token_file':'/tmp/credential','cameras':{'front':0},'settings':{}}
        with patch('blupe_controller.cloud.SessionApiSimClient'):
            self.bridge=CloudBridge(self.driver,config,self.poses)
        self.ids={'session_id':'session','episode_id':'episode','lease_id':'lease'}

    def prepare(self):
        self.bridge.connected=True
        self.poses.get.return_value={'joints_deg':[0]*5,'gripper':.5}
        self.bridge.ready=True
        with patch('blupe_controller.cloud.urlopen',return_value=io.BytesIO(b'{"ok":true}')):
            return self.bridge.api_prepare_session(self.ids)

    def test_never_accepts_session_until_authorized(self):
        self.assertIsNone(self.bridge.api_prepare_session(self.ids))
        self.assertEqual(self.driver.moves,[])
        first=self.prepare()
        self.assertEqual(first['right_joints_deg'],[])
        self.assertEqual(len(first['left_joints_deg']),5)

    def test_joint_target_execution_and_observation(self):
        self.prepare()
        command={**self.ids,'step_id':0,'command_id':'one','left_joints_deg':[1]*5,'right_joints_deg':[],'left_gripper':.5}
        self.assertEqual(self.bridge.api_handle_joint_command(command),[])
        deadline=time.monotonic()+2
        while self.bridge.pending and time.monotonic()<deadline:time.sleep(.01)
        self.assertFalse(self.bridge.pending)
        self.assertEqual(self.driver.moves,[[1]*5])
        messages=[c.args[0] for c in self.bridge.client.send.call_args_list]
        self.assertTrue(any(m.get('status')=='executed' for m in messages))
        self.assertTrue(any(m.get('type')=='observation' and m['step_id']==1 for m in messages))
        rejected=self.bridge.api_handle_joint_command(command)
        self.assertEqual(rejected[0]['status'],'rejected')
        self.assertEqual(len(self.driver.moves),1)

    def test_bad_trajectory_is_rejected_before_any_move(self):
        self.prepare()
        payload={**self.ids,'schema_version':1,'type':'joint_trajectory','trajectory_id':'traj','cadence_hz':10,'dispatched_at':time.time(),
                 'waypoints':[{'step_id':0,'left_joints_deg':[0]*5,'right_joints_deg':[]},
                              {'step_id':1,'left_joints_deg':[999]*5,'right_joints_deg':[]}]}
        self.assertEqual(self.bridge.api_handle_joint_trajectory(payload)[0]['status'],'rejected')
        self.assertEqual(self.driver.moves,[])

    def test_disconnect_holds_and_revokes_session(self):
        self.prepare()
        self.bridge.api_connection_changed(False,'disconnected')
        self.assertTrue(self.driver.held)
        self.assertIsNone(self.bridge.lease)
        self.assertFalse(self.bridge.ready)

    def test_autoqueue_success_returns_home_then_rearms(self):
        self.prepare()
        self.bridge.auto_queue=True
        self.bridge.on_session_complete=Mock()
        self.driver.joints=[20]*5
        self.bridge.api_handle_stop({**self.ids,'reason':'policy_complete'})
        self.bridge.on_session_complete.assert_called_once()
        self.assertTrue(self.bridge.returning_home)
        self.assertIsNone(self.bridge.lease)
        self.assertFalse(self.bridge.ready)
        self.assertFalse(self.driver.held)
        self.driver.joints=[0]*5
        with patch('blupe_controller.cloud.urlopen',return_value=io.BytesIO(b'{"ok":true}')):
            self.bridge.finish_return_home()
        self.assertTrue(self.bridge.ready)
        self.assertTrue(self.bridge.auto_queue)
        self.bridge.client.request_ready.assert_called_once()

    def test_autoqueue_does_not_rearm_on_stop_fault_or_stale_lease(self):
        for reason in ('user_requested','session_timeout','lease_expired','unknown'):
            with self.subTest(reason=reason):
                self.prepare();self.bridge.auto_queue=True
                self.bridge.on_session_complete=Mock()
                self.bridge.api_handle_stop({**self.ids,'reason':reason})
                self.assertFalse(self.bridge.auto_queue)
                self.assertFalse(self.bridge.ready)
                self.bridge.on_session_complete.assert_not_called()
        self.prepare();self.bridge.auto_queue=True
        self.bridge.api_handle_stop({**self.ids,'lease_id':'old','reason':'policy_complete'})
        self.assertEqual(self.bridge.lease,self.ids)

    def test_autoqueue_pause_during_return_never_rearms(self):
        self.prepare();self.bridge.auto_queue=True
        self.bridge.on_session_complete=Mock()
        self.bridge.api_handle_stop({**self.ids,'reason':'policy_complete'})
        self.bridge.pause()
        self.bridge.finish_return_home()
        self.assertFalse(self.bridge.auto_queue)
        self.assertFalse(self.bridge.ready)
        self.bridge.client.request_ready.assert_not_called()

    def test_autoqueue_camera_failure_after_return_holds(self):
        self.prepare();self.bridge.auto_queue=True
        self.bridge.on_session_complete=Mock()
        self.bridge.api_handle_stop({**self.ids,'reason':'policy_complete'})
        with patch('blupe_controller.cloud.urlopen',return_value=io.BytesIO(b'{"ok":false}')):
            self.bridge.finish_return_home()
        self.assertFalse(self.bridge.auto_queue)
        self.assertFalse(self.bridge.ready)
        self.assertTrue(self.driver.held)

    def test_assignment_rechecks_home_and_camera(self):
        self.prepare();self.bridge.lease=None;self.bridge.ready=True
        self.bridge.auto_queue=True
        self.driver.joints=[20]*5
        self.assertIsNone(self.bridge.api_prepare_session(self.ids))
        self.assertFalse(self.bridge.auto_queue)
        self.assertTrue(self.driver.held)


def test_home_and_trajectory_share_so101_tolerance():
    state = {'joints_deg':[1.9]*5, 'gripper':.1}
    assert CloudBridge.near(state,[0.]*5,.1)
    assert CloudBridge.near(state,[0.]*5,.1,joint_tolerance_deg=5.)
    state['joints_deg'][0]=5.1
    assert not CloudBridge.near(state,[0.]*5,.1,joint_tolerance_deg=5.)
