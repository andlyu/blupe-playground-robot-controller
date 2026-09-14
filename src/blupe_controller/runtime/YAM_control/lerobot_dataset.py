"""LeRobot v2.1 export of finalized YAM recordings; no camera or motor ownership.

The visualizer fetches meta/info.json and then templated per-episode Parquet and
one MP4 per camera. The raw archive provides none of those, so this writes the
canonical layout beside it instead of replacing it.

Timing: LeRobot's frame clock is regular, so `timestamp` is frame_index/fps and
the encoded video carries exactly one frame per recorded row. Real acquisition
time survives verbatim in `source_timestamp`, and `frame_gap` marks a row whose
real interval exceeded 1.5 nominal periods, i.e. the recorder missed a tick.
Nothing is resampled: no joint value or camera frame is invented to fill a gap.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

from YAM_control.training_recorder import CAMERAS, atomic_json

CODEBASE_VERSION = 'v2.1'
CHUNKS_SIZE = 1000
CAMERA_ORDER = ('left', 'top', 'right')
ROBOT_TYPE = 'yam_bimanual'
DEFAULT_TASK = 'YAM bimanual policy run'
# Bimanual order matching the official v2.1 example: each arm's joints are
# followed by that arm's gripper. The recorder stores all joints, then grippers.
MOTORS = ([f'left_joint_{i}' for i in range(6)] + ['left_gripper']
          + [f'right_joint_{i}' for i in range(6)] + ['right_gripper'])
DATA_PATH = 'data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet'
VIDEO_PATH = 'videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4'
REGISTRY = 'meta/blupe_source_episodes.jsonl'
TIMING = {
    'frame_clock': 'timestamp = frame_index / fps',
    'source_timestamp': 'recorder elapsed seconds at capture, never resampled',
    'frame_gap': 'true when the real interval before this row exceeded 1.5 / fps',
    'note': 'Missed ticks compress real time in playback; no frame is interpolated.',
}
SOURCE_FLOATS = ['source_timestamp', 'action_timestamp', 'camera_skew_s',
                 *(n + suffix for n in CAMERA_ORDER
                   for suffix in ('_frame_age_s', '_camera_timestamp'))]
SOURCE_INTS = [n + '_frame_sequence' for n in CAMERA_ORDER]


@dataclass(frozen=True)
class RobotLayout:
    cameras: tuple
    motors: tuple
    joints_per_arm: int
    arms: int
    robot_type: str

    @property
    def source_floats(self):
        return ['source_timestamp', 'action_timestamp', 'camera_skew_s',
                *(n + suffix for n in self.cameras for suffix in ('_frame_age_s', '_camera_timestamp'))]

    @property
    def source_ints(self):
        return [n + '_frame_sequence' for n in self.cameras]


YAM_LAYOUT = RobotLayout(CAMERA_ORDER, tuple(MOTORS), 6, 2, ROBOT_TYPE)


def robot_layout(meta):
    hardware = meta.get('hardware')
    if hardware not in ('so101', 'bimanual_so101'):
        raise ValueError('Unsupported robot layout')
    arms = 1 if hardware == 'so101' else 2
    joints = meta['joint_names']
    if len(joints) != arms * 5 or len(set(joints)) != len(joints):
        raise ValueError('Unexpected SO101 joint names')
    cameras = tuple(meta['camera_devices'])
    if not cameras or any(not name.isidentifier() for name in cameras):
        raise ValueError('Invalid camera roles')
    motors = []
    for arm in range(arms):
        motors.extend(joints[arm*5:(arm+1)*5])
        motors.append(('left_gripper' if arm == 0 else 'right_gripper') if arms == 2 else 'gripper')
    return RobotLayout(cameras, tuple(motors), 5, arms, hardware)


def source_fingerprint(episode, meta, rows, layout=YAM_LAYOUT):
    digest = hashlib.sha256(json.dumps(meta, sort_keys=True).encode())
    for row in rows:
        for name in layout.cameras:
            digest.update(image_path(episode, row, name).read_bytes())
    return digest.hexdigest()


def video_key(name):
    return f'observation.images.{name}'


def state_vector(joints, grippers, layout=YAM_LAYOUT):
    """Interleave recorder [left 0-5, right 0-5] + [left, right] grippers."""
    if len(joints) != layout.joints_per_arm * layout.arms or len(grippers) != layout.arms:
        raise ValueError('Recording does not match robot joint/gripper layout')
    values = []
    for arm in range(layout.arms):
        values.extend(joints[arm*layout.joints_per_arm:(arm+1)*layout.joints_per_arm])
        values.append(grippers[arm])
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Non-finite joint value')
    return [float(x) for x in values]


def load_episode(episode, layout=YAM_LAYOUT):
    """Read a finalized physical recording, refusing edited or simulated ones."""
    episode = Path(episode)
    meta = json.loads((episode / 'manifest.json').read_text())
    if meta['status'] != 'finalized' or meta.get('simulated'):
        raise ValueError('Only finalized physical episodes may be published')
    raw = (episode / 'samples.jsonl').read_bytes()
    if hashlib.sha256(raw).hexdigest() != meta['samples_sha256']:
        raise ValueError('Recording changed after finalization')
    rows = [json.loads(line) for line in raw.splitlines()]
    if not rows or len(rows) != meta['rows']:
        raise ValueError('Invalid finalized sample count')
    if set(layout.cameras) != set(meta.get('camera_devices', CAMERAS)):
        raise ValueError('Camera set changed without updating the LeRobot export')
    return meta, rows


def image_path(episode, row, name):
    path = (Path(episode) / row[name + '_image_path']).resolve()
    if path.parent != (Path(episode) / 'images').resolve():
        raise ValueError('Image path escapes recording')
    return path


def episode_columns(meta, rows, *, episode_index, task_index, index_offset, layout=YAM_LAYOUT):
    """Per-frame columns for one episode, in canonical LeRobot v2.1 order."""
    fps = meta['fps']
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('Invalid recording FPS')
    gap_threshold = 1.5 / fps
    previous = None
    columns = []
    for frame_index, row in enumerate(rows):
        source = row['timestamp']
        if not math.isfinite(source) or source < 0 or (previous is not None and source <= previous):
            raise ValueError('Invalid source timeline')
        column = {
            'observation.state': state_vector(row['measured_joints'], row['measured_grippers'], layout=layout),
            'action': state_vector(row['action_joints'], row['action_grippers'], layout=layout),
            'timestamp': frame_index / fps,
            'frame_index': frame_index,
            'episode_index': episode_index,
            'index': index_offset + frame_index,
            'task_index': task_index,
            'source_timestamp': source,
            'action_timestamp': row['action_timestamp'] if row['action_timestamp'] is not None else 0.,
            'frame_gap': previous is not None and source - previous > gap_threshold,
            'training_valid': bool(row['training_valid']),
            'action_valid': bool(row['action_valid']),
            'camera_skew_s': row['camera_skew_s'],
        }
        for name in layout.cameras:
            column[name + '_frame_age_s'] = row[name + '_frame_age_s']
            column[name + '_camera_timestamp'] = row[name + '_camera_timestamp']
            column[name + '_frame_sequence'] = row[name + '_frame_sequence']
        columns.append(column)
        previous = source
    return columns


def build_features(fps, camera_shapes, layout=YAM_LAYOUT):
    """info.json feature block: videos live in MP4s, everything else in Parquet."""
    motors = {'motors': list(layout.motors)}
    features = {
        'observation.state': {'dtype': 'float32', 'shape': [len(layout.motors)], 'names': motors},
        'action': {'dtype': 'float32', 'shape': [len(layout.motors)], 'names': motors},
    }
    for name in layout.cameras:
        features[video_key(name)] = {
            'dtype': 'video', 'shape': list(camera_shapes[name]),
            'names': ['height', 'width', 'channel'],
            'video_info': {'video.fps': float(fps), 'video.codec': 'h264',
                           'video.pix_fmt': 'yuv420p', 'video.is_depth_map': False,
                           'has_audio': False},
        }
    features['timestamp'] = {'dtype': 'float32', 'shape': [1], 'names': None}
    for name in layout.source_floats:
        features[name] = {'dtype': 'float64', 'shape': [1], 'names': None}
    for name in ['frame_index', 'episode_index', 'index', 'task_index', *layout.source_ints]:
        features[name] = {'dtype': 'int64', 'shape': [1], 'names': None}
    for name in ['frame_gap', 'training_valid', 'action_valid']:
        features[name] = {'dtype': 'bool', 'shape': [1], 'names': None}
    return features


def write_parquet(columns, destination, layout=YAM_LAYOUT):
    from datasets import Dataset, Features, Sequence, Value
    import pyarrow
    pyarrow.set_cpu_count(1)
    pyarrow.set_io_thread_count(1)
    features = {
        'observation.state': Sequence(Value('float32'), length=len(layout.motors)),
        'action': Sequence(Value('float32'), length=len(layout.motors)),
    }
    features['timestamp'] = Value('float32')
    for name in layout.source_floats:
        features[name] = Value('float64')
    for name in ['frame_index', 'episode_index', 'index', 'task_index', *layout.source_ints]:
        features[name] = Value('int64')
    for name in ['frame_gap', 'training_valid', 'action_valid']:
        features[name] = Value('bool')
    destination.parent.mkdir(parents=True, exist_ok=True)
    Dataset.from_list(columns, features=Features(features)).to_parquet(str(destination), batch_size=128)


def encode_camera_video(episode, rows, name, output, fps):
    """One MP4 per camera, one encoded frame per recorded row."""
    from PIL import Image
    paths = [image_path(episode, row, name) for row in rows]
    with Image.open(paths[0]) as first:
        width, height = first.size
    if width % 2 or height % 2:
        raise ValueError('Camera frames must have even dimensions for yuv420p')
    encoder = os.environ.get('YAM_FFMPEG') or shutil.which('ffmpeg')
    if not encoder:
        raise RuntimeError('FFmpeg is required for LeRobot video export')
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_suffix('.partial.mp4')
    # See docs/refs/ffmpeg: explicit RGB24 geometry/rate, H.264 yuv420p +faststart.
    command = [encoder, '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pixel_format', 'rgb24',
               '-video_size', f'{width}x{height}', '-framerate', str(fps), '-i', '-', '-an',
               '-c:v', 'libx264', '-threads', '1', '-preset', 'veryfast', '-crf', '23',
               '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(partial)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    try:
        for path in paths:
            with Image.open(path) as source:
                frame = source.convert('RGB')
            if frame.size != (width, height):
                raise ValueError('Camera frame size changed mid-episode')
            process.stdin.write(frame.tobytes())
        process.stdin.close()
        if process.wait(timeout=600) != 0:
            raise RuntimeError('Video encoder failed')
        partial.replace(output)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        if process.stdin and not process.stdin.closed:
            process.stdin.close()
        partial.unlink(missing_ok=True)
    return (height, width, 3)


def _estimate_num_samples(length, min_samples=100, max_samples=10_000, power=.75):
    if length < min_samples:
        min_samples = length
    return max(min_samples, min(int(length ** power), max_samples))


def _sample_indices(length):
    import numpy as np
    return np.round(np.linspace(0, length - 1, _estimate_num_samples(length))).astype(int).tolist()


def _image_stats(paths):
    """Channel-wise (3,1,1) statistics over evenly sampled frames, normalized to [0,1]."""
    import numpy as np
    from PIL import Image
    sampled = None
    indices = _sample_indices(len(paths))
    for position, index in enumerate(indices):
        with Image.open(paths[index]) as source:
            frame = np.asarray(source.convert('RGB'), dtype=np.uint8).transpose(2, 0, 1)
        if max(frame.shape[1:]) >= 300:
            step = int(frame.shape[2] / 150) if frame.shape[2] > frame.shape[1] else int(frame.shape[1] / 150)
            frame = frame[:, ::step, ::step]
        if sampled is None:
            sampled = np.empty((len(indices), *frame.shape), dtype=np.uint8)
        sampled[position] = frame
    stats = {key: value / 255. for key, value in
             {'min': np.min(sampled, axis=(0, 2, 3), keepdims=True),
              'max': np.max(sampled, axis=(0, 2, 3), keepdims=True),
              'mean': np.mean(sampled, axis=(0, 2, 3), keepdims=True),
              'std': np.std(sampled, axis=(0, 2, 3), keepdims=True)}.items()}
    stats = {key: np.squeeze(value, axis=0) for key, value in stats.items()}
    stats['count'] = np.array([len(indices)])
    return stats


def episode_stats(episode, rows, columns, layout=YAM_LAYOUT):
    """Per-episode statistics in the official v2.1 shape."""
    import numpy as np
    stats = {}
    for name in columns[0]:
        values = np.array([column[name] for column in columns])
        if values.ndim == 1:
            values = values.reshape(-1, 1)
        stats[name] = {'min': np.min(values, axis=0), 'max': np.max(values, axis=0),
                       'mean': np.mean(values, axis=0), 'std': np.std(values, axis=0),
                       'count': np.array([len(columns)])}
    for name in layout.cameras:
        stats[video_key(name)] = _image_stats([image_path(episode, row, name) for row in rows])
    return stats


def aggregate_stats(per_episode):
    """Weighted mean/variance across episodes, matching the official aggregation."""
    import numpy as np
    aggregated = {}
    for key in per_episode[0]:
        entries = [stats[key] for stats in per_episode if key in stats]
        means = np.stack([np.asarray(entry['mean'], dtype=np.float64) for entry in entries])
        variances = np.stack([np.asarray(entry['std'], dtype=np.float64) ** 2 for entry in entries])
        counts = np.stack([np.asarray(entry['count'], dtype=np.float64) for entry in entries])
        total = counts.sum(axis=0)
        while counts.ndim < means.ndim:
            counts = np.expand_dims(counts, axis=-1)
        mean = (means * counts).sum(axis=0) / total
        variance = ((variances + (means - mean) ** 2) * counts).sum(axis=0) / total
        aggregated[key] = {
            'min': np.min(np.stack([np.asarray(entry['min']) for entry in entries]), axis=0),
            'max': np.max(np.stack([np.asarray(entry['max']) for entry in entries]), axis=0),
            'mean': mean, 'std': np.sqrt(variance), 'count': total.astype(np.int64),
        }
    return aggregated


def _plain(value):
    import numpy as np
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    return value


def read_jsonl(path):
    path = Path(path)
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def write_jsonl(path, records):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(''.join(json.dumps(record, allow_nan=False) + '\n' for record in records))
    tmp.replace(path)


def append_episode(root, episode, *, task=None, layout=YAM_LAYOUT):
    """Add one finalized recording to a local v2.1 root; idempotent per episode ID.

    Only meta/ has to exist beforehand, so the publisher can build an upload
    payload holding just the new episode plus refreshed metadata.
    """
    root, episode = Path(root), Path(episode)
    meta, rows = load_episode(episode, layout=layout)
    fingerprint = source_fingerprint(episode, meta, rows, layout=layout)
    registry = read_jsonl(root / REGISTRY)
    for entry in registry:
        if entry['episode_id'] == meta['episode_id']:
            if entry['source_sha256'] != fingerprint:
                raise ValueError('Episode ID already exists with different data')
            return entry
    episode_index = len(registry)
    chunk = episode_index // CHUNKS_SIZE
    episodes = read_jsonl(root / 'meta/episodes.jsonl')
    if len(episodes) != episode_index:
        raise ValueError('Episode metadata is out of sync with the source registry')

    text = (task or meta.get('task') or DEFAULT_TASK).strip() or DEFAULT_TASK
    tasks = read_jsonl(root / 'meta/tasks.jsonl')
    known = {entry['task']: entry['task_index'] for entry in tasks}
    if text not in known:
        known[text] = len(tasks)
        tasks.append({'task_index': known[text], 'task': text})
    columns = episode_columns(meta, rows, episode_index=episode_index, task_index=known[text],
                              index_offset=sum(entry['length'] for entry in episodes), layout=layout)

    # Data before metadata: a crash can leave an unreferenced file, never a
    # metadata entry pointing at one that does not exist.
    shapes = {}
    for name in layout.cameras:
        target = root / VIDEO_PATH.format(episode_chunk=chunk, video_key=video_key(name),
                                          episode_index=episode_index)
        shapes[name] = encode_camera_video(episode, rows, name, target, meta['fps'])
    features = build_features(meta['fps'], shapes, layout=layout)
    info_path = root / 'meta/info.json'
    if info_path.exists():
        previous_info = json.loads(info_path.read_text())
        if previous_info['fps'] != meta['fps'] or previous_info['features'] != features:
            raise ValueError('Recording FPS or camera schema differs from existing dataset')
    write_parquet(columns, root / DATA_PATH.format(episode_chunk=chunk, episode_index=episode_index), layout=layout)

    stats = episode_stats(episode, rows, columns, layout=layout)
    per_episode = read_jsonl(root / 'meta/episodes_stats.jsonl')
    per_episode.append({'episode_index': episode_index, 'stats': _plain(stats)})
    episodes.append({'episode_index': episode_index, 'tasks': [text], 'length': len(rows)})
    entry = {'episode_index': episode_index, 'episode_id': meta['episode_id'],
             'source_sha256': fingerprint,
             'samples_sha256': meta['samples_sha256'], 'outcome': meta['outcome'],
             'recorded_at': meta['started_at'], 'rows': meta['rows'],
             'valid_rows': meta['valid_rows'], 'missed_ticks': meta.get('missed_ticks', 0),
             'capture_errors': meta.get('capture_errors', 0)}
    registry.append(entry)

    write_jsonl(root / 'meta/tasks.jsonl', tasks)
    write_jsonl(root / 'meta/episodes.jsonl', episodes)
    write_jsonl(root / 'meta/episodes_stats.jsonl', per_episode)
    write_jsonl(root / REGISTRY, registry)
    atomic_json(root / 'meta/stats.json',
                _plain(aggregate_stats([record['stats'] for record in per_episode])))
    total_frames = sum(record['length'] for record in episodes)
    atomic_json(root / 'meta/info.json', {
        'codebase_version': CODEBASE_VERSION, 'robot_type': layout.robot_type,
        'total_episodes': len(episodes), 'total_frames': total_frames,
        'total_tasks': len(tasks), 'total_videos': len(episodes) * len(layout.cameras),
        'total_chunks': chunk + 1, 'chunks_size': CHUNKS_SIZE, 'fps': meta['fps'],
        'splits': {'train': f'0:{len(episodes)}'}, 'data_path': DATA_PATH,
        'video_path': VIDEO_PATH, 'features': features,
        'blupe_timing': TIMING,
    })
    return entry


def episode_files(episode_index, layout=YAM_LAYOUT):
    """Repository paths written for one episode, for upload verification."""
    chunk = episode_index // CHUNKS_SIZE
    return [DATA_PATH.format(episode_chunk=chunk, episode_index=episode_index),
            *(VIDEO_PATH.format(episode_chunk=chunk, video_key=video_key(name),
                                episode_index=episode_index) for name in layout.cameras)]
