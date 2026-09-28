import sys
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/blupe_controller/runtime'))
from YAM_control.first_call_wander import FirstCallWander, TrajectoryInterrupted

START=(0.,)*12
PATH=((.01,)*12,(.02,)*12)

def snap(q=START, gripper=1.):
    return NS(left=NS(joints_rad=tuple(q[:6]),gripper=gripper),right=NS(joints_rad=tuple(q[6:]),gripper=gripper))

class Adapter:
    def __init__(self):
        self.q=START;self.calls=[];self.started=threading.Event();self.gripper=1.;self.bad_return=False
    def snapshot(self):return snap(self.q,self.gripper)
    def execute(self,path,**kw):
        self.calls.append((path,kw))
        if len(self.calls)==1:
            for i,p in enumerate(path):
                kw['validate_waypoint'](i,self.q,p);self.q=p;kw['on_waypoint'](i)
            self.started.set()
            kw['cancel_event'].wait(2)
            raise TrajectoryInterrupted('cancelled')
        for i,p in enumerate(path):
            kw['validate_waypoint'](i,self.q,p);self.q=p
        return snap((.2,)*12 if self.bad_return else self.q)

class Guard:
    def __init__(self):self.calls=[];self.fail=False
    def approve(self,start,path,**kw):
        if self.fail:raise ValueError('collision')
        self.calls.append((start,path));return NS(waypoints=path)

class Tests(unittest.TestCase):
    def setUp(self):
        self.a=Adapter();self.g=Guard();self.live=True
        self.m=FirstCallWander(self.a,self.g,NS(max_joint_velocity_rad_s=.35),'unused',
            lambda identity:self.live,planner=lambda *args:(PATH,.4))
        self.identity=('session','episode','lease')
    def test_first_only_return_barrier_and_unchanged_grippers(self):
        self.m.start(self.identity);self.assertTrue(self.a.started.wait(1))
        self.assertTrue(self.m.blocks_motion)
        result=self.m.finish(self.identity)
        self.assertEqual(result['state'],'returned');self.assertEqual(self.a.q,START)
        self.assertFalse(self.m.blocks_motion)
        self.m.start(self.identity);self.assertEqual(len(self.a.calls),2)
        for path,kw in self.a.calls:self.assertEqual(kw['gripper_waypoints'],((1.,1.),)*len(path))
    def test_preflight_failure_never_moves(self):
        def reject(*args):raise ValueError('unsafe path')
        self.m.planner=reject;self.m.start(self.identity);self.m.done.wait(1)
        self.assertEqual(self.m.status()['state'],'skipped');self.assertEqual(self.a.calls,[])
    def test_closed_grippers_skip_without_opening(self):
        self.a.gripper=.5;self.m.start(self.identity);self.m.done.wait(1)
        self.assertEqual(self.m.status()['reason'],'grippers_not_open');self.assertEqual(self.a.calls,[])
    def test_stop_revokes_return_and_blocks_late_policy(self):
        self.m.start(self.identity);self.assertTrue(self.a.started.wait(1))
        self.live=False;self.m.abort();self.m.done.wait(1)
        self.assertEqual(len(self.a.calls),1);self.assertTrue(self.m.blocks_motion)
        with self.assertRaises(RuntimeError):self.m.finish(self.identity)
    def test_return_collision_does_not_dispatch(self):
        self.m.start(self.identity);self.assertTrue(self.a.started.wait(1));self.g.fail=True
        with self.assertRaises(RuntimeError):self.m.finish(self.identity)
        self.assertEqual(len(self.a.calls),1);self.assertTrue(self.m.blocks_motion)
    def test_wrong_lease_and_unsettled_return_rejected(self):
        self.m.start(self.identity);self.assertTrue(self.a.started.wait(1))
        with self.assertRaises(ValueError):self.m.finish(('other',)*3)
        self.a.bad_return=True
        with self.assertRaises(RuntimeError):self.m.finish(self.identity)
        self.assertTrue(self.m.blocks_motion)
    def test_ready_during_planning_skips(self):
        entered,release=threading.Event(),threading.Event()
        def plan(*args):entered.set();release.wait(1);return PATH,.4
        self.m.planner=plan;self.m.start(self.identity);self.assertTrue(entered.wait(1))
        self.m.finish_event.set();release.set();self.m.done.wait(1)
        self.assertEqual(self.m.status()['state'],'skipped');self.assertEqual(self.a.calls,[])

if __name__=='__main__':unittest.main()

class IntegrationTests(unittest.TestCase):
    def test_installer_gates_targets_and_other_motion(self):
        import os,time
        from unittest.mock import patch
        from YAM_control.first_call_wander_integration import install
        class Parent:
            def __init__(self):
                self.lock=threading.RLock();self._physical=Adapter();self._safety_guardrails=Guard()
                self._safety_config=NS(max_joint_velocity_rad_s=.35)
                self.mode='API_ACTIVE';self.api_authorized=True;self.api_last_step=-1
                self.api_session_id='s';self.api_episode_id='e';self.api_lease_id='l';self._hardware_pending=None
            def _api_raw_trajectory_result_locked(self,payload,status,code):return dict(status=status,code=code)
            def api_handle_joint_trajectory(self,payload):return ['accepted']
        hardware=NS(PhysicalOperator=Parent,base=NS(MODEL_PATH='unused',OperatorSimulator=Parent))
        install(hardware);op=hardware.PhysicalOperator()
        p=dict(schema_version=1,executor='yam_first_call',operation='start',payload={},
               session_id='s',episode_id='e',lease_id='l',expires_at=time.time()+120)
        with patch.dict(os.environ,{'YAM_FIRST_CALL_WANDER':'1'}):
            op._physical.gripper=.5
            op.api_handle_execution(p);op.first_call_wander.done.wait(1)
            self.assertEqual(op.api_handle_execution(dict(p,operation='finish'))['state'],'skipped')
            for update in ({'lease_id':'bad'},{'payload':{'target':[0]*12}},{'expires_at':0}):
                with self.assertRaises(ValueError):op.api_handle_execution(dict(p,**update))
        op.first_call_wander.state='failed'
        self.assertEqual(op.api_handle_joint_trajectory({})[0]['code'],'first_call_not_settled')
