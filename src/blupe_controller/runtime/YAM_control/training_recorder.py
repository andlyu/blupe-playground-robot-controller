"""Model-independent 10 Hz training recorder. Disk/camera I/O never runs in control callbacks."""
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

CAMERAS = {'left': '10', 'right': '4', 'top': '16'}
JOINT_NAMES = [f'{arm}_joint_{i}' for arm in ('left', 'right') for i in range(6)]


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, allow_nan=False, indent=2) + '\n')
    tmp.replace(path)


class RelayFrames:
    def __init__(self, origin='http://127.0.0.1:8089'):
        self.origin = origin.rstrip('/')
        self.pool = ThreadPoolExecutor(max_workers=3)
        # The viewing-only side camera must never delay the training cameras.
        self.observer = None
        self.observer_stop = threading.Event()
        self.observer_thread = threading.Thread(target=self._observer, daemon=True)
        self.observer_thread.start()

    def _observer(self):
        while not self.observer_stop.is_set():
            try:
                _, frame = self.one('observer', '18', origin='http://127.0.0.1:8090')
                self.observer = frame
            except Exception:
                self.observer = None
            self.observer_stop.wait(.1)

    def one(self, name, device, *, origin=None):
        with urlopen(f'{origin or self.origin}/{device}/snapshot.jpg', timeout=.5) as r:
            body = r.read(4 * 1024 * 1024 + 1)
            stamp = float(r.headers['X-Capture-Monotonic'])
            sequence = int(r.headers['X-Frame-Sequence'])
        if len(body) > 4 * 1024 * 1024 or not body.startswith(b'\xff\xd8') or not math.isfinite(stamp):
            raise ValueError('Invalid timestamped JPEG')
        return name, {'jpeg': body, 'monotonic': stamp, 'sequence': sequence}

    def capture(self):
        futures = [self.pool.submit(self.one, k, v) for k, v in CAMERAS.items()]
        frames = dict(f.result() for f in futures)
        observer = self.observer
        if observer and 0 <= time.monotonic() - observer['monotonic'] <= .15:
            frames['observer'] = observer
        return frames

    def close(self):
        self.observer_stop.set()
        self.pool.shutdown(wait=False, cancel_futures=True)


class EpisodeRecorder:
    def __init__(self, root, episode_id, snapshot, *, frames=None, simulated=False, task=None):
        if not re.fullmatch(r'ep_[A-Za-z0-9_-]+', episode_id):
            raise ValueError('Invalid episode ID')
        self.path = Path(root) / episode_id
        self.snapshot = snapshot
        self.frames = frames or RelayFrames()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.meta = {'schema_version': 1, 'episode_id': episode_id, 'fps': 10,
                     'started_at': time.time(), 'started_monotonic': time.monotonic(),
                     'status': 'recording', 'outcome': None, 'simulated': simulated,
                     'owner_pid': os.getpid(), 'task': task,
                     'rows': 0, 'valid_rows': 0, 'capture_errors': 0, 'missed_ticks': 0,
                     'joint_names': JOINT_NAMES, 'joint_units': 'radians',
                     'gripper_units': 'normalized_0_closed_1_open',
                     'camera_devices': dict(CAMERAS), 'max_alignment_error_s': .15}
        self.thread = threading.Thread(target=self._run, name='yam-training-record', daemon=True)

    def start(self):
        self.thread.start()

    def finish(self, outcome):
        with self.lock:
            if not self.stop_event.is_set():
                self.meta['outcome'] = outcome
                self.meta['ended_at'] = time.time()
                self.meta['ended_monotonic'] = time.monotonic()
                self.stop_event.set()

    def status(self):
        with self.lock:
            return dict(self.meta)

    def _run(self):
        try:
            self.path.mkdir(parents=True, exist_ok=False)
            (self.path / 'images').mkdir()
            atomic_json(self.path / 'manifest.json', self.meta)
            deadline = time.monotonic()
            with (self.path / 'samples.jsonl').open('w') as stream:
                while not self.stop_event.is_set():
                    try:
                        frames = self.frames.capture()
                        state = self.snapshot()
                        stamp = time.monotonic()
                        if self.stop_event.is_set():
                            break
                        if not set(CAMERAS).issubset(frames) or set(frames) - set(CAMERAS) - {'observer'}:
                            raise ValueError('Three camera views required')
                        row = self._row(frames, state, stamp)
                        stream.write(json.dumps(row, allow_nan=False) + '\n')
                        stream.flush()
                        with self.lock:
                            self.meta['rows'] += 1
                            self.meta['valid_rows'] += int(row['training_valid'])
                    except Exception as exc:
                        with self.lock:
                            self.meta['capture_errors'] += 1
                            self.meta['last_error_type'] = type(exc).__name__
                    deadline += .1
                    now = time.monotonic()
                    if deadline < now:
                        missed = int((now - deadline) / .1) + 1
                        with self.lock:
                            self.meta['missed_ticks'] += missed
                        deadline += missed * .1
                    self.stop_event.wait(max(0, deadline - time.monotonic()))
            with self.lock:
                self.meta['status'] = 'finalized' if self.meta['rows'] else 'recording_failed'
                self.meta['outcome'] = self.meta['outcome'] or 'recorder_stopped'
                self.meta['samples_sha256'] = hashlib.sha256((self.path / 'samples.jsonl').read_bytes()).hexdigest()
                atomic_json(self.path / 'manifest.json', self.meta)
        except Exception as exc:
            with self.lock:
                self.meta.update(status='recording_failed', last_error_type=type(exc).__name__)
        finally:
            close = getattr(self.frames, 'close', None)
            if close:
                close()

    def _row(self, frames, state, stamp):
        measured, command = state
        q = list(measured.left.joints_rad) + list(measured.right.joints_rad)
        g = [measured.left.gripper, measured.right.gripper]
        if len(q) != 12 or not all(math.isfinite(x) for x in q + g):
            raise ValueError('Invalid measured state')
        row = {'episode_id': self.meta['episode_id'], 'frame_index': self.meta['rows'],
               'timestamp': stamp - self.meta['started_monotonic'],
               'measured_joints': q, 'measured_grippers': g,
               'action_joints': list(command['joints']) if command else [0.] * 12,
               'action_grippers': list(command['grippers']) if command else [0., 0.],
               'action_valid': command is not None,
               'action_timestamp': command['monotonic'] - self.meta['started_monotonic'] if command else None}
        # Driver-reported effort and velocity, aligned with the measured pose.
        # Unavailable feedback remains null; it must never look like zero torque.
        for field, attribute in [('measured_joint_effort', 'joint_effort'),
                                 ('measured_joint_velocity', 'joint_velocity')]:
            left = getattr(measured.left, attribute, None)
            right = getattr(measured.right, attribute, None)
            row[field] = (list(left) if left is not None else [None]*6) + (list(right) if right is not None else [None]*6)
        row['measured_gripper_effort'] = [getattr(arm, 'gripper_effort', None) for arm in (measured.left, measured.right)]
        row['measured_gripper_velocity'] = [getattr(arm, 'gripper_velocity', None) for arm in (measured.left, measured.right)]
        valid = command is not None
        capture_times = []
        for name, frame in frames.items():
            age = stamp - frame['monotonic']
            if name in CAMERAS:
                capture_times.append(frame['monotonic'])
                valid = valid and 0 <= age <= .15
            relative = f'images/{name}-{self.meta["rows"]:06d}.jpg'
            (self.path / relative).write_bytes(frame['jpeg'])
            row[name + '_image_path'] = relative
            row[name + '_camera_timestamp'] = frame['monotonic'] - self.meta['started_monotonic']
            row[name + '_frame_age_s'] = age
            row[name + '_frame_sequence'] = frame['sequence']
        row['camera_skew_s'] = max(capture_times) - min(capture_times)
        row['training_valid'] = bool(valid)
        return row
