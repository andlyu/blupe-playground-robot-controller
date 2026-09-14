"""Episode capture from cached feedback and the existing camera relay, never motor IO."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import re
import threading
import time
from urllib.request import urlopen


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, allow_nan=False) + '\n')
    temporary.replace(path)


class EpisodeRecorder:
    def __init__(self, root, config, episode_id, task, joint_names, snapshot):
        if not re.fullmatch(r'ep_[A-Za-z0-9_-]+', episode_id):
            raise ValueError('Invalid recording episode ID')
        self.path = Path(root) / episode_id
        self.snapshot = snapshot
        self.origin = 'http://127.0.0.1:' + str(config['settings'].get('camera_port', 8089))
        self.cameras = dict(config['cameras'])
        self.stop_event = threading.Event()
        self.meta = dict(schema_version=1, episode_id=episode_id, robot_id=config['robot_id'],
                         hardware=config['hardware'], task=task, fps=10, simulated=False,
                         joint_names=list(joint_names), joint_units='radians',
                         gripper_units='normalized_0_closed_1_open', camera_devices=self.cameras,
                         started_at=time.time(), started_monotonic=time.monotonic(),
                         status='recording', outcome=None, rows=0, valid_rows=0,
                         capture_errors=0, missed_ticks=0, owner_pid=os.getpid())
        self.thread = threading.Thread(target=self._run, daemon=True, name='robot-episode-recorder')

    def start(self):
        self.thread.start()

    def finish(self, outcome):
        if not self.stop_event.is_set():
            self.meta.update(outcome=outcome, ended_at=time.time(), ended_monotonic=time.monotonic())
            self.stop_event.set()

    def frame(self, role, device):
        with urlopen(f'{self.origin}/{device}/snapshot.jpg', timeout=.5) as response:
            jpeg = response.read(4 * 1024 * 1024 + 1)
            stamp = float(response.headers['X-Capture-Monotonic'])
            sequence = int(response.headers['X-Frame-Sequence'])
        if not jpeg.startswith(b'\xff\xd8') or len(jpeg) > 4 * 1024 * 1024 or not math.isfinite(stamp):
            raise ValueError('Invalid timestamped JPEG')
        return role, (jpeg, stamp, sequence)

    def _run(self):
        try:
            self.path.mkdir(parents=True, exist_ok=False)
            (self.path / 'images').mkdir()
            atomic_json(self.path / 'manifest.json', self.meta)
            with ThreadPoolExecutor(max_workers=len(self.cameras)) as pool, (self.path / 'samples.jsonl').open('w') as stream:
                deadline = time.monotonic()
                while not self.stop_event.is_set():
                    try:
                        futures = [pool.submit(self.frame, role, device) for role, device in self.cameras.items()]
                        frames = dict(f.result() for f in futures)
                        state, command, observed = self.snapshot()
                        now = time.monotonic()
                        if self.stop_event.is_set():
                            break
                        row = self.row(frames, state, command, observed, now)
                        stream.write(json.dumps(row, allow_nan=False) + '\n')
                        stream.flush()
                        self.meta['rows'] += 1
                        self.meta['valid_rows'] += int(row['training_valid'])
                    except Exception as error:
                        self.meta['capture_errors'] += 1
                        self.meta['last_error_type'] = type(error).__name__
                    deadline += .1
                    now = time.monotonic()
                    if deadline < now:
                        missed = int((now-deadline)/.1)+1
                        self.meta['missed_ticks'] += missed
                        deadline += missed*.1
                    self.stop_event.wait(max(0, deadline-time.monotonic()))
            self.meta.update(status='finalized' if self.meta['rows'] else 'recording_failed',
                             samples_sha256=hashlib.sha256((self.path/'samples.jsonl').read_bytes()).hexdigest())
            atomic_json(self.path/'manifest.json', self.meta)
        except Exception as error:
            self.meta.update(status='recording_failed', last_error_type=type(error).__name__)

    def row(self, frames, state, command, observed, now):
        joints = [math.radians(v) for v in state['joints_deg']]
        grippers = state['gripper'] if isinstance(state['gripper'], list) else [state['gripper']]
        if len(joints) != len(self.meta['joint_names']) or not all(math.isfinite(v) for v in joints+grippers):
            raise ValueError('Invalid recorded feedback')
        start = self.meta['started_monotonic']
        row = dict(episode_id=self.meta['episode_id'], frame_index=self.meta['rows'], timestamp=now-start,
                   measured_joints=joints, measured_grippers=grippers,
                   action_joints=command['joints'] if command else [0.]*len(joints),
                   action_grippers=command['grippers'] if command else [0.]*len(grippers),
                   action_valid=command is not None, action_timestamp=command['monotonic']-start if command else None,
                   feedback_age_s=now-observed)
        valid = command is not None and 0 <= now-observed <= .15
        stamps = []
        for role, (jpeg, stamp, sequence) in frames.items():
            relative = f'images/{role}-{self.meta["rows"]:06d}.jpg'
            (self.path/relative).write_bytes(jpeg)
            row.update({role+'_image_path':relative, role+'_camera_timestamp':stamp-start,
                        role+'_frame_age_s':now-stamp, role+'_frame_sequence':sequence})
            stamps.append(stamp)
            valid = valid and 0 <= now-stamp <= .15
        row['camera_skew_s'] = max(stamps)-min(stamps)
        row['training_valid'] = bool(valid and row['camera_skew_s'] <= .15)
        return row
