"""Named joint poses; recording does not alter calibration or command motors."""
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time


class PoseStore:
    def __init__(self, config):
        settings = config.get('settings', {})
        source = settings.get('lerobot_config_file')
        self.path = Path(settings['poses_file']) if settings.get('poses_file') else (
            Path(source).parent / (config['robot_id'] + '.poses.json') if source else None)
        calibration = settings.get('calibration_file')
        self.identity = {'robot_id': config['robot_id'], 'calibration_sha256':
            hashlib.sha256(Path(calibration).read_bytes()).hexdigest() if calibration else None}
        self.poses = {}
        if self.path and self.path.exists():
            saved = json.loads(self.path.read_text())
            if saved.get('identity') != self.identity:
                raise ValueError('Saved poses belong to a different robot or calibration; record new poses')
            self.poses = saved['poses']
            for name, pose in self.poses.items():
                self.validate(name, pose)

    @staticmethod
    def validate(name, pose):
        if name not in ('zero', 'home'):
            raise ValueError('Pose must be zero or home')
        joints = pose.get('joints_deg')
        gripper = pose.get('gripper')
        if not isinstance(joints, list) or len(joints) != 5 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in [*joints, gripper]):
            raise ValueError('Pose requires five finite joint angles and a gripper value')
        if not 0 <= gripper <= 1:
            raise ValueError('Invalid gripper position')

    def capture(self, name, state):
        if state.get('mode') not in ('readonly','hold','active'):
            raise ValueError('Cannot record a pose from faulted feedback')
        pose = {'joints_deg':list(state['joints_deg']), 'gripper':state['gripper'], 'recorded_at':time.time()}
        self.validate(name, pose)
        updated = {**self.poses, name:pose}
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            fd, temporary = tempfile.mkstemp(dir=self.path.parent, prefix='.poses-')
            try:
                with os.fdopen(fd, 'w') as out:
                    json.dump({'version':1, 'identity':self.identity, 'poses':updated}, out, indent=2)
                    out.write('\n')
                    out.flush()
                    os.fsync(out.fileno())
                os.replace(temporary, self.path)
            finally:
                if os.path.exists(temporary):
                    os.unlink(temporary)
        self.poses = updated
        return pose

    def get(self, name):
        if name not in self.poses:
            raise ValueError(f'Capture {name} first; no default pose is assumed')
        return self.poses[name]
