import json
from pathlib import Path
import tempfile
import unittest
from blupe_controller.poses import PoseStore


class PoseTests(unittest.TestCase):
    def test_persist_both_poses_and_check_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            cal=root/'cal.json';cal.write_text('{}')
            config={'robot_id':'test','settings':{'lerobot_config_file':str(root/'robot.json'),'calibration_file':str(cal)}}
            store=PoseStore(config)
            zero={'mode':'readonly','joints_deg':[1,2,3,4,5],'gripper':.2}
            store.capture('zero',zero)
            store.capture('home',{**zero,'joints_deg':[5,4,3,2,1]})
            restored=PoseStore(config)
            self.assertEqual(restored.get('zero')['joints_deg'],[1,2,3,4,5])
            self.assertEqual(restored.get('home')['joints_deg'],[5,4,3,2,1])
            self.assertEqual(store.path.stat().st_mode&0o777,0o600)
            before=store.path.read_bytes()
            with self.assertRaises(ValueError):store.capture('zero',{**zero,'mode':'fault'})
            self.assertEqual(before,store.path.read_bytes())
            cal.write_text('{"changed":true}')
            with self.assertRaises(ValueError):PoseStore(config)

    def test_invalid_pose_not_persisted(self):
        store=PoseStore({'robot_id':'test'})
        for values in ([1,2], [float('nan')]*5):
            with self.assertRaises(ValueError):store.capture('zero',{'mode':'readonly','joints_deg':values,'gripper':.5})
        self.assertEqual(store.poses,{})
