import time
import unittest
from blupe_controller.operator import Operator

class Driver:
    def __init__(self, stuck=False):
        self.q=[0.]*5; self.g=0.; self.mode='active'; self.sent=[]; self.stuck=stuck
    def _validate_target(self,q,g):
        if len(q)!=5: raise ValueError('Invalid target')
    def state(self):return dict(joints_deg=list(self.q),gripper=self.g,mode=self.mode)
    def move(self,q,g):
        self.sent.append((list(q),g))
        if not self.stuck:self.q=list(q);self.g=g
    def hold(self):self.mode='hold';return self.state()

class MotionTests(unittest.TestCase):
    def operator(self,stuck=False):return Operator(Driver(stuck),{'robot_id':'test','cameras':{}})
    def wait(self,o,limit=5):
        end=time.monotonic()+limit
        while o.motion is not None and time.monotonic()<end:time.sleep(.02)
        self.assertIsNone(o.motion)
    def test_one_request_reaches_pose_with_bounded_steps(self):
        o=self.operator();o.start_pose({'joints_deg':[3.]*5,'gripper':.03});self.wait(o)
        self.assertEqual(o.driver.q,[3.]*5)
        previous=[0.]*5
        for q,g in o.driver.sent:
            self.assertLessEqual(max(abs(a-b) for a,b in zip(q,previous)),1)
            previous=q
        self.assertEqual(o.motion_error,'')
    def test_stop_prevents_subsequent_commands(self):
        o=self.operator();o.start_pose({'joints_deg':[90.]*5,'gripper':.5})
        time.sleep(.15)
        with self.assertRaises(ValueError):o.action({'action':'enable'})
        o.action({'action':'hold'});count=len(o.driver.sent);self.wait(o)
        self.assertEqual(len(o.driver.sent),count)
        self.assertLess(o.driver.q[0],90)
    def test_stuck_feedback_stops(self):
        o=self.operator(True);o.start_pose({'joints_deg':[10.]*5,'gripper':.1});self.wait(o)
        self.assertIn('progress',o.motion_error);self.assertEqual(o.driver.mode,'hold')

    def test_tracking_deadband_does_not_pin_target(self):
        o=self.operator()
        o.driver.joint_tolerance_deg = .1  # Exercise tracking rather than early settling.
        def delayed_move(q,g):
            o.driver.sent.append((list(q),g))
            if max(abs(a-b) for a,b in zip(q,o.driver.q)) >= 1.5:
                o.driver.q=list(q);o.driver.g=g
        o.driver.move=delayed_move
        o.start_pose({'joints_deg':[4.]*5,'gripper':.04});self.wait(o)
        self.assertEqual(o.driver.q,[4.]*5)
        self.assertEqual(o.motion_error,'')

    def test_stuck_target_lookahead_is_bounded(self):
        o=self.operator(True)
        o.start_pose({'joints_deg':[90.]*5,'gripper':.9});self.wait(o)
        self.assertTrue(o.driver.sent)
        self.assertLessEqual(max(max(q) for q,g in o.driver.sent),5.)
        self.assertLessEqual(max(g for q,g in o.driver.sent),.05)
