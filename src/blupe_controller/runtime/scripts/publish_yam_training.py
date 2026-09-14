#!/usr/bin/env python3
"""Durable single-writer public YAM dataset publisher, separate from motion control."""
import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path
import tempfile
import time

from YAM_control.training_recorder import CAMERAS, atomic_json

REPO = 'andlyu/Public-YAM-runs'
LEROBOT_META = ['meta/info.json', 'meta/episodes.jsonl', 'meta/tasks.jsonl',
                'meta/episodes_stats.jsonl', 'meta/stats.json',
                'meta/blupe_source_episodes.jsonl']


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def export_episode(episode, destination):
    from datasets import Dataset, Features, Image, Sequence, Value
    import pyarrow
    pyarrow.set_cpu_count(1)
    pyarrow.set_io_thread_count(1)
    episode, destination = Path(episode), Path(destination)
    meta = json.loads((episode / 'manifest.json').read_text())
    if meta['status'] != 'finalized' or meta.get('simulated'):
        raise ValueError('Only finalized physical episodes may be published')
    cameras = meta.get('camera_devices', CAMERAS)
    joint_count = len(meta.get('joint_names', [])) or 12
    gripper_count = 1 if meta.get('hardware') == 'so101' else 2
    raw = (episode / 'samples.jsonl').read_bytes()
    if hashlib.sha256(raw).hexdigest() != meta['samples_sha256']:
        raise ValueError('Recording changed after finalization')
    features = {k: Value(t) for k, t in {
        'episode_id': 'string', 'frame_index': 'int64', 'timestamp': 'float64',
        'action_valid': 'bool', 'action_timestamp': 'float64', 'camera_skew_s': 'float64',
        'training_valid': 'bool', 'outcome': 'string', 'task': 'string',
    }.items()}
    for k, length in [('measured_joints', joint_count), ('measured_grippers', gripper_count),
                      ('action_joints', joint_count), ('action_grippers', gripper_count)]:
        features[k] = Sequence(Value('float64'), length=length)
    for key, length in [('measured_joint_effort',joint_count), ('measured_joint_velocity',joint_count),
                        ('measured_gripper_effort',gripper_count), ('measured_gripper_velocity',gripper_count)]:
        features[key] = Sequence(Value('float64'), length=length)
    for name in cameras:
        features[name + '_image'] = Image()
        for key in ['camera_timestamp', 'frame_age_s']:
            features[name + '_' + key] = Value('float64')
        features[name + '_frame_sequence'] = Value('int64')
    import pyarrow.parquet as pq
    (destination / 'data').mkdir(parents=True)
    (destination / 'episodes').mkdir()
    parquet = destination / 'data' / (meta['episode_id'] + '.parquet')
    rows = []
    count = 0
    image_hashes = hashlib.sha256()
    # Bounded image batches; see docs/refs/huggingface-hub/parquet-batch-writer.md.
    with pq.ParquetWriter(str(parquet), Features(features).arrow_schema) as writer:
        for line in raw.splitlines():
            sample = json.loads(line)
            # Older finalized recordings predate optional motor telemetry.
            row = {key: sample.get(key) for key in features}
            for key, width in [('measured_joint_effort', joint_count), ('measured_joint_velocity', joint_count),
                               ('measured_gripper_effort', gripper_count), ('measured_gripper_velocity', gripper_count)]:
                if row[key] is None:
                    row[key] = [None] * width
            row['outcome'] = meta['outcome']
            row['task'] = meta.get('task')
            for name in cameras:
                image_path = (episode / sample[name + '_image_path']).resolve()
                if image_path.parent != (episode / 'images').resolve():
                    raise ValueError('Image path escapes recording')
                body = image_path.read_bytes()
                from PIL import Image as PILImage
                import io
                with PILImage.open(io.BytesIO(body)) as image:
                    image.verify()
                image_hashes.update(body)
                row[name + '_image'] = {'bytes': body, 'path': None}
            rows.append(row)
            count += 1
            if len(rows) >= 16:
                writer.write_table(Dataset.from_list(rows, features=Features(features)).data.table)
                rows.clear()
        if rows:
            writer.write_table(Dataset.from_list(rows, features=Features(features)).data.table)
    if not count or count != meta['rows']:
        raise ValueError('Invalid finalized sample count')
    from YAM_control.training_video import RENDER_VERSION, render_video
    video = episode / 'video.mp4'
    video_info = episode / 'video.json'
    saved_video = json.loads(video_info.read_text()) if video_info.exists() else {}
    if (not video.exists() or saved_video.get('render_version') != RENDER_VERSION
            or saved_video.get('samples_sha256') != meta['samples_sha256']
            or saved_video.get('sha256') != hashlib.sha256(video.read_bytes()).hexdigest()):
        saved_video = render_video(episode, video)
        saved_video.update(samples_sha256=meta['samples_sha256'],
                           sha256=hashlib.sha256(video.read_bytes()).hexdigest())
        atomic_json(video_info, saved_video)
    (destination / 'videos').mkdir()
    shutil.copyfile(video, destination / 'videos' / (meta['episode_id'] + '.mp4'))
    preview = episode / 'preview.mp4'
    preview_info = episode / 'preview.json'
    saved_preview = json.loads(preview_info.read_text()) if preview_info.exists() else {}
    viewing_info = episode / 'viewing.json'
    viewing_video = episode / 'viewing.mp4'
    if viewing_info.exists() and viewing_video.exists():
        viewing = json.loads(viewing_info.read_text())
        if (viewing.get('render_version') == RENDER_VERSION and viewing.get('samples_sha256') == meta['samples_sha256'] and
                viewing.get('sha256') == hashlib.sha256(viewing_video.read_bytes()).hexdigest()):
            shutil.copyfile(viewing_video, preview)
            atomic_json(preview_info, viewing)
            saved_preview = viewing
    if (not preview.exists() or saved_preview.get('render_version') != RENDER_VERSION
            or saved_preview.get('samples_sha256') != meta['samples_sha256']
            or saved_preview.get('sha256') != hashlib.sha256(preview.read_bytes()).hexdigest()):
        saved_preview = render_video(episode, preview, preview=True)
        saved_preview.update(samples_sha256=meta['samples_sha256'],
                             sha256=hashlib.sha256(preview.read_bytes()).hexdigest())
        atomic_json(preview_info, saved_preview)
    (destination / 'previews').mkdir()
    shutil.copyfile(preview, destination / 'previews' / (meta['episode_id'] + '.mp4'))
    manifest = dict(meta)
    manifest.pop('owner_pid', None)
    manifest.pop('started_monotonic', None)
    manifest.pop('ended_monotonic', None)
    manifest.update(video=saved_video, preview=saved_preview,
                    parquet_sha256=file_sha256(parquet),
                    images_sha256=image_hashes.hexdigest(), dataset_title='Public YAM runs')
    atomic_json(destination / 'episodes' / (meta['episode_id'] + '.json'), manifest)
    return manifest


def recover_orphan(episode, meta):
    if meta.get('status') != 'recording' or not isinstance(meta.get('owner_pid'), int):
        return meta
    try:
        os.kill(meta['owner_pid'], 0)
        return meta
    except ProcessLookupError:
        pass
    except PermissionError:
        return meta
    samples = episode / 'samples.jsonl'
    rows = []
    corrupt = 0
    for line in samples.read_bytes().splitlines() if samples.exists() else []:
        try:
            row = json.loads(line)
            if not all((episode / row[n + '_image_path']).is_file() for n in meta.get('camera_devices', CAMERAS)):
                raise ValueError('Missing image')
            rows.append(row)
        except (ValueError, KeyError):
            corrupt += 1
    raw = ''.join(json.dumps(row) + '\n' for row in rows).encode()
    samples.write_bytes(raw)
    meta.update(status='finalized' if rows else 'recording_failed', outcome='recorder_process_interrupted',
                rows=len(rows), valid_rows=sum(bool(r['training_valid']) for r in rows),
                recovered_dropped_rows=corrupt, ended_at=None,
                samples_sha256=hashlib.sha256(raw).hexdigest())
    atomic_json(episode / 'manifest.json', meta)
    return meta


def publish_once(root, api=None, *, repo=REPO):
    from huggingface_hub import HfApi, hf_hub_download
    root = Path(root)
    api = api or HfApi()
    info = api.repo_info(repo_id=repo, repo_type='dataset')
    if info.private:
        raise ValueError('Configured dataset must be public')
    results = []
    for path in sorted(root.glob('ep_*/manifest.json')):
        episode = path.parent
        meta = recover_orphan(episode, json.loads(path.read_text()))
        state_file = episode / 'upload.json'
        state = json.loads(state_file.read_text()) if state_file.exists() else {}
        if (meta.get('status') != 'finalized' or meta.get('simulated')
                or (state.get('status') == 'uploaded' and state.get('lerobot_version') == 'v2.1')):
            continue
        # Viewer publication is a separate small commit; do not start the large
        # export for new recordings until that worker has handled the episode.
        since = float(os.environ.get('YAM_VIEWING_FIRST_SINCE', '0'))
        if since and meta.get('started_at', 0) >= since:
            receipt = episode / 'viewing-upload.json'
            viewing = json.loads(receipt.read_text()) if receipt.exists() else {}
            if (viewing.get('render_version') == RENDER_VERSION and viewing.get('samples_sha256') != meta.get('samples_sha256') or
                    viewing.get('status') not in ('uploaded', 'arm_check')):
                continue
        if state.get('retry_at', 0) > time.time():
            continue
        attempts = state.get('attempts', 0) + 1
        atomic_json(state_file, {'status': 'uploading', 'attempts': attempts})
        try:
            with tempfile.TemporaryDirectory(prefix='yam-export-', dir=root) as temp:
                from YAM_control.lerobot_dataset import (append_episode, episode_files, REGISTRY,
                                                          read_jsonl, write_jsonl,
                                                          source_fingerprint)
                # Pin every read to one revision, then compare-and-swap the commit.
                # See HF upload_folder(parent_commit) and saved v2.1 references.
                revision = api.repo_info(repo_id=repo, repo_type='dataset').sha
                has_registry = api.file_exists(repo, REGISTRY, repo_type='dataset', revision=revision)
                if has_registry:
                    for filename in LEROBOT_META:
                        source = hf_hub_download(repo, filename, repo_type='dataset', revision=revision)
                        target = Path(temp) / filename
                        target.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copyfile(source, target)
                elif api.file_exists(repo, 'meta/info.json', repo_type='dataset', revision=revision):
                    raise ValueError('Existing LeRobot dataset has no source registry')
                registry = read_jsonl(Path(temp) / REGISTRY)
                existing = {entry['episode_id']: entry for entry in registry}
                eid = meta['episode_id']
                remote_manifest = f'episodes/{eid}.json'
                # Stable paths give restart-safe idempotency, including a lost
                # commit reply. Verify a previous commit before uploading again.
                identical = False
                registry_repaired = False
                if api.file_exists(repo, remote_manifest, repo_type='dataset', revision=revision):
                    # Verify source provenance without re-encoding the existing archive.
                    # Parquet library versions can change output bytes for identical rows.
                    saved = json.loads(Path(hf_hub_download(repo, remote_manifest, repo_type='dataset', revision=revision)).read_text())
                    from YAM_control.lerobot_dataset import load_episode, image_path
                    checked_meta, rows = load_episode(episode)
                    image_hash = hashlib.sha256()
                    for row in rows:
                        for name in CAMERAS:
                            image_hash.update(image_path(episode, row, name).read_bytes())
                    if (any(saved.get(key) != checked_meta.get(key) for key in
                            ('episode_id', 'samples_sha256', 'rows', 'fps', 'task', 'outcome'))
                            or saved.get('images_sha256') != image_hash.hexdigest()):
                        raise ValueError('Episode ID already exists with different data')
                    old_entry = existing.get(eid)
                    if old_entry:
                        expected = {
                            'episode_id': eid, 'source_sha256': source_fingerprint(episode, checked_meta, rows),
                            'samples_sha256': checked_meta['samples_sha256'], 'outcome': checked_meta['outcome'],
                            'recorded_at': checked_meta['started_at'], 'rows': checked_meta['rows'],
                            'valid_rows': checked_meta['valid_rows'],
                            'missed_ticks': checked_meta.get('missed_ticks', 0),
                            'capture_errors': checked_meta.get('capture_errors', 0),
                        }
                        # Repair only hashes from the earlier bulk converter. Any
                        # semantic disagreement still blocks publication.
                        if any(old_entry.get(key) != value for key, value in expected.items()
                               if key not in ('source_sha256', 'samples_sha256')):
                            raise ValueError('LeRobot registry disagrees with source metadata')
                        if any(old_entry.get(key) != expected[key]
                               for key in ('source_sha256', 'samples_sha256')):
                            old_entry.update(expected)
                            write_jsonl(Path(temp) / REGISTRY, registry)
                            registry_repaired = True
                    manifest = saved
                    identical = True
                else:
                    manifest = export_episode(episode, temp)
                entry = append_episode(temp, episode)
                required = [remote_manifest, f'data/{eid}.parquet', f'videos/{eid}.mp4',
                            *episode_files(entry['episode_index']), *LEROBOT_META]
                if manifest.get('preview'):
                    required.append(f'previews/{eid}.mp4')
                if eid in existing:
                    if not identical or not all(api.file_exists(repo, name, repo_type='dataset', revision=revision)
                                                for name in required):
                        raise RuntimeError('Committed episode is incomplete')
                    if registry_repaired:
                        commit = api.upload_folder(repo_id=repo, repo_type='dataset', folder_path=temp,
                                                   parent_commit=revision,
                                                   commit_message=f'Repair LeRobot provenance for {eid}')
                        revision = commit.oid
                else:
                    # Preserve robot-specific configs added by other publishers.
                    card = (hf_hub_download(repo, 'README.md', repo_type='dataset', revision=revision)
                            if api.file_exists(repo, 'README.md', repo_type='dataset', revision=revision)
                            else Path(__file__).resolve().parents[1] / 'docs/PUBLIC-YAM-DATASET-CARD.md')
                    shutil.copyfile(card, Path(temp) / 'README.md')
                    commit = api.upload_folder(repo_id=repo, repo_type='dataset', folder_path=temp,
                                               parent_commit=revision,
                                               commit_message=f'Add LeRobot YAM episode {eid}')
                    revision = commit.oid
                saved = json.loads(Path(hf_hub_download(repo, remote_manifest, repo_type='dataset', revision=revision)).read_text())
                remote_entries = read_jsonl(hf_hub_download(repo, REGISTRY, repo_type='dataset', revision=revision))
                if (saved != manifest
                        or entry not in remote_entries
                        or not all(api.file_exists(repo, name, repo_type='dataset', revision=revision)
                                   for name in required)):
                    raise RuntimeError('Remote episode verification failed')
                state = {'status': 'uploaded', 'attempts': attempts, 'commit': revision,
                         'lerobot_version': 'v2.1', 'lerobot_episode_index': entry['episode_index'],
                         'url': f'https://huggingface.co/datasets/{repo}', 'uploaded_at': time.time(),
                         'video_url': f'https://huggingface.co/datasets/{repo}/resolve/{revision}/videos/{eid}.mp4?download=true'}
        except Exception as exc:
            state = {'status': 'retrying', 'attempts': attempts, 'error_type': type(exc).__name__,
                     'retry_at': time.time() + min(300, 10 * 2 ** min(attempts-1, 5))}
        atomic_json(state_file, state)
        results.append({'episode_id': meta['episode_id'], **state})
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--repo', default=REPO)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()
    args.root.mkdir(parents=True, exist_ok=True)
    import fcntl
    with (args.root / '.publisher.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        while True:
            try:
                for item in publish_once(args.root, repo=args.repo):
                    print('[dataset] ' + json.dumps(item), flush=True)
            except Exception as exc:
                print('[dataset] publisher_error=' + type(exc).__name__, flush=True)
            if args.once:
                break
            time.sleep(5)


if __name__ == '__main__':
    main()
