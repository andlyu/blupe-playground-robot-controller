import hashlib
import json
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
import pyarrow.parquet as pq

from YAM_control import lerobot_dataset as lr
from scripts.publish_yam_training import publish_once


def fixture(root, eid='ep_first', task='pick', times=(0., .1, .4)):
    episode = Path(root) / eid
    (episode / 'images').mkdir(parents=True)
    rows = []
    for i, timestamp in enumerate(times):
        row = dict(episode_id=eid, frame_index=i, timestamp=timestamp,
                   measured_joints=list(range(12)), measured_grippers=[.3, .8],
                   action_joints=[x + .1 for x in range(12)], action_grippers=[.4, .9],
                   action_timestamp=None if i == 0 else timestamp - .01,
                   action_valid=i != 0, training_valid=i != 0, camera_skew_s=.002)
        for n, name in enumerate(lr.CAMERA_ORDER):
            filename = f'images/{name}-{i}.jpg'
            Image.new('RGB', (16, 12), (60 * (n+1), 20 * i, 0)).save(episode / filename)
            row.update({name + '_image_path': filename, name + '_frame_age_s': .01,
                        name + '_camera_timestamp': timestamp - .01,
                        name + '_frame_sequence': i})
        rows.append(row)
    raw = ''.join(json.dumps(r) + '\n' for r in rows).encode()
    (episode / 'samples.jsonl').write_bytes(raw)
    meta = dict(episode_id=eid, status='finalized', simulated=False, rows=len(rows),
                valid_rows=len(rows)-1, fps=10, task=task, outcome='session_api_stop',
                started_at='2026-09-06T00:00:00Z',
                samples_sha256=hashlib.sha256(raw).hexdigest())
    (episode / 'manifest.json').write_text(json.dumps(meta))
    return episode


class FakeHub:
    def __init__(self, root):
        self.root = Path(root)
        self.uploads = 0
        self.sha = 'initial'
        self.lose_reply = False
        self.conflict = False

    def repo_info(self, **kw):
        return SimpleNamespace(private=False, sha=self.sha)

    def file_exists(self, repo, path, **kw):
        return (self.root / path).exists()

    def download(self, repo, path, **kw):
        return str(self.root / path)

    def upload_folder(self, **kw):
        if kw['parent_commit'] != self.sha or self.conflict:
            raise RuntimeError('Concurrent commit')
        shutil.copytree(kw['folder_path'], self.root, dirs_exist_ok=True)
        self.uploads += 1
        self.sha = f'commit-{self.uploads}'
        if self.lose_reply:
            self.lose_reply = False
            raise ConnectionError('Lost commit reply')
        return SimpleNamespace(oid=self.sha)


class LeRobotTests(unittest.TestCase):
    def test_vectors_timing_video_alignment_stats_and_multiple_episodes(self):
        import cv2
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as out:
            ep = fixture(root)
            entry = lr.append_episode(out, ep)
            self.assertEqual(lr.append_episode(out, ep), entry)
            table = pq.read_table(Path(out) / lr.episode_files(0)[0]).to_pylist()
            np.testing.assert_allclose(table[0]['observation.state'], [0,1,2,3,4,5,.3,6,7,8,9,10,11,.8])
            np.testing.assert_allclose(table[0]['action'], [.1,1.1,2.1,3.1,4.1,5.1,.4,6.1,7.1,8.1,9.1,10.1,11.1,.9])
            np.testing.assert_allclose([r['timestamp'] for r in table], [0,.1,.2])
            self.assertEqual([r['source_timestamp'] for r in table], [0,.1,.4])
            self.assertEqual([r['frame_gap'] for r in table], [False,False,True])
            self.assertFalse(table[0]['action_valid'])
            for n, video in enumerate(lr.episode_files(0)[1:]):
                capture = cv2.VideoCapture(str(Path(out) / video))
                try:
                    self.assertEqual(capture.get(cv2.CAP_PROP_FRAME_COUNT), 3)
                    self.assertEqual(capture.get(cv2.CAP_PROP_FPS), 10)
                    for i in range(3):
                        ok, frame = capture.read()
                        self.assertTrue(ok)
                        self.assertAlmostEqual(float(frame[:,:,2].mean()), 60*(n+1), delta=8)
                        self.assertAlmostEqual(float(frame[:,:,1].mean()), 20*i, delta=8)
                finally:
                    capture.release()
            lr.append_episode(out, fixture(root, 'ep_second', 'place', (0.,.1)))
            info = json.loads((Path(out) / 'meta/info.json').read_text())
            self.assertEqual((info['total_episodes'], info['total_frames'], info['total_tasks'], info['total_videos']), (2,5,2,6))
            second = pq.read_table(Path(out) / lr.episode_files(1)[0]).to_pylist()
            self.assertEqual([r['index'] for r in second], [3,4])
            self.assertEqual(second[0]['task_index'], 1)
            stats = json.loads((Path(out) / 'meta/stats.json').read_text())
            self.assertEqual(stats['index']['count'], [5])
            np.testing.assert_allclose(stats['index']['mean'], [2])
            np.testing.assert_allclose(stats['index']['std'], [np.std(range(5))])
            self.assertEqual(np.array(stats['observation.images.left']['mean']).shape, (3,1,1))
            Image.new('RGB', (16,12), 'white').save(ep / 'images/left-0.jpg')
            with self.assertRaisesRegex(ValueError, 'different data'):
                lr.append_episode(out, ep)

    def test_lost_reply_backfill_and_restart(self):
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as remote:
            ep = fixture(root)
            hub = FakeHub(remote)
            hub.lose_reply = True
            with patch('huggingface_hub.hf_hub_download', hub.download):
                self.assertEqual(publish_once(root, hub)[0]['status'], 'retrying')
                (ep / 'upload.json').write_text('{}')
                result = publish_once(root, hub)[0]
                self.assertEqual(result['status'], 'uploaded', result)
                self.assertEqual(hub.uploads, 1)
                self.assertEqual(publish_once(root, hub), [])
                # Simulate legacy-only remote and an old successful upload marker.
                shutil.rmtree(Path(remote) / 'meta')
                (ep / 'upload.json').write_text('{"status":"uploaded"}')
                archive = Path(remote) / 'data/ep_first.parquet'
                original = archive.read_bytes()
                result = publish_once(root, hub)[0]
                self.assertEqual(result['status'], 'uploaded', result)
                self.assertEqual(result['lerobot_episode_index'], 0)
                self.assertEqual(archive.read_bytes(), original)
                fixture(root, 'ep_second', 'place')
                result = publish_once(root, hub)[0]
                self.assertEqual(result['lerobot_episode_index'], 1)
                self.assertEqual(len(lr.read_jsonl(Path(remote) / lr.REGISTRY)), 2)

    def test_conflict_does_not_publish_metadata_or_mark_uploaded(self):
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as remote:
            ep = fixture(root)
            hub = FakeHub(remote)
            hub.conflict = True
            with patch('huggingface_hub.hf_hub_download', hub.download):
                self.assertEqual(publish_once(root, hub)[0]['status'], 'retrying')
                self.assertFalse((Path(remote) / lr.REGISTRY).exists())
                hub.conflict = False
                (ep / 'upload.json').write_text('{}')
                self.assertEqual(publish_once(root, hub)[0]['status'], 'uploaded')

    def test_reject_bad_timing_simulation_and_schema_change(self):
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as out:
            ep = fixture(root)
            lr.append_episode(out, ep)
            other = fixture(root, 'ep_second')
            path = other / 'manifest.json'
            meta = json.loads(path.read_text())
            meta['fps'] = 20
            path.write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError, 'schema'):
                lr.append_episode(out, other)
            self.assertEqual(len(lr.read_jsonl(Path(out) / lr.REGISTRY)), 1)
            meta['simulated'] = True
            path.write_text(json.dumps(meta))
            with self.assertRaisesRegex(ValueError, 'physical'):
                lr.append_episode(out, other)
            bad = fixture(root, 'ep_bad', times=(0., .2, .1))
            with self.assertRaisesRegex(ValueError, 'timeline'):
                lr.append_episode(out, bad)

    def test_backfill_does_not_reencode_archive_or_overwrite_legacy_bytes(self):
        from scripts.publish_yam_training import export_episode
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as remote:
            ep = fixture(root)
            export_episode(ep, remote)
            (ep / 'upload.json').write_text('{"status":"uploaded"}')
            original = {name: (Path(remote)/name).read_bytes() for name in
                        ('data/ep_first.parquet', 'episodes/ep_first.json', 'videos/ep_first.mp4')}
            hub = FakeHub(remote)
            with patch('huggingface_hub.hf_hub_download', hub.download), patch(
                    'scripts.publish_yam_training.export_episode', side_effect=AssertionError('Archive regenerated')):
                result = publish_once(root, hub)[0]
            self.assertEqual(result['status'], 'uploaded', result)
            for name, body in original.items():
                self.assertEqual((Path(remote)/name).read_bytes(), body)

    def test_backfill_rejects_changed_camera_bytes_before_upload(self):
        from scripts.publish_yam_training import export_episode
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as remote:
            ep = fixture(root)
            export_episode(ep, remote)
            Image.new('RGB', (16,12), 'white').save(ep / 'images/left-0.jpg')
            hub = FakeHub(remote)
            with patch('huggingface_hub.hf_hub_download', hub.download):
                result = publish_once(root, hub)[0]
            self.assertEqual(result['status'], 'retrying')
            self.assertEqual(hub.uploads, 0)
            self.assertFalse((Path(remote)/lr.REGISTRY).exists())

    def test_backfill_repairs_hash_only_registry_damage(self):
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as remote:
            ep = fixture(root)
            hub = FakeHub(remote)
            with patch('huggingface_hub.hf_hub_download', hub.download):
                first = publish_once(root, hub)[0]
                self.assertEqual(first['status'], 'uploaded', first)
                registry = lr.read_jsonl(Path(remote) / lr.REGISTRY)
                registry[0]['source_sha256'] = 'bad-source-hash'
                registry[0]['samples_sha256'] = 'bad-samples-hash'
                lr.write_jsonl(Path(remote) / lr.REGISTRY, registry)
                (ep / 'upload.json').write_text('{}')
                with patch('scripts.publish_yam_training.export_episode',
                           side_effect=AssertionError('Archive regenerated')):
                    repaired = publish_once(root, hub)[0]
            self.assertEqual(repaired['status'], 'uploaded', repaired)
            self.assertEqual(hub.uploads, 2)
            fixed = lr.read_jsonl(Path(remote) / lr.REGISTRY)[0]
            meta, rows = lr.load_episode(ep)
            self.assertEqual(fixed['source_sha256'], lr.source_fingerprint(ep, meta, rows))
            self.assertEqual(fixed['samples_sha256'], meta['samples_sha256'])
