import io, json, tempfile, time, unittest
from pathlib import Path
from unittest.mock import patch
from PIL import Image
from YAM_control.training_recorder import EpisodeRecorder, CAMERAS
from scripts.publish_yam_training import export_episode


class Frames:
    def __init__(self, stale=False):
        self.stale = stale
        self.n = 0
    def capture(self):
        self.n += 1
        frames = {}
        for i, name in enumerate(CAMERAS):
            b = io.BytesIO()
            Image.new('RGB', (16, 12), ((i+1)*60, 0, 0)).save(b, format='JPEG')
            frames[name] = {'jpeg': b.getvalue(), 'monotonic': time.monotonic()-(1 if self.stale else .01), 'sequence': self.n}
        return frames

class TrainingExportVideoTests(unittest.TestCase):

    def fixture(self, root, stale=False, episode_id='ep_fixture'):
        from YAM_control.i2rt_bimanual_adapter import BimanualSnapshot, ArmSnapshot
        state = BimanualSnapshot(ArmSnapshot(tuple(range(6)), .3), ArmSnapshot(tuple(range(6,12)), .8))
        r = EpisodeRecorder(root, episode_id, lambda: (state, {'joints': tuple(x+.1 for x in range(12)), 'grippers': (.4,.9), 'monotonic': time.monotonic()}), frames=Frames(stale))
        r.start()
        time.sleep(.35)
        r.finish('session_timeout')
        r.thread.join(3)
        self.assertEqual(r.meta['status'], 'finalized', r.meta)
        return r

    def test_export_rebuilds_old_preview_and_rejects_old_viewing_cache(self):
        import hashlib
        from YAM_control.training_video import RENDER_VERSION, render_video
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as out:
            recorder = self.fixture(root)
            export_episode(recorder.path, Path(out)/'first')
            old = json.loads((recorder.path/'preview.json').read_text())
            old['render_version'] = RENDER_VERSION - 1
            (recorder.path/'preview.json').write_text(json.dumps(old))
            old['sha256'] = hashlib.sha256(b'old viewing placeholders').hexdigest()
            (recorder.path/'viewing.json').write_text(json.dumps(old))
            (recorder.path/'viewing.mp4').write_bytes(b'old viewing placeholders')
            with patch('YAM_control.training_video.render_video', wraps=render_video) as render:
                manifest = export_episode(recorder.path, Path(out)/'second')
                self.assertEqual(render.call_count, 1)
                self.assertTrue(render.call_args.kwargs['preview'])
            self.assertEqual(manifest['preview']['render_version'], RENDER_VERSION)
            self.assertNotEqual((recorder.path/'preview.mp4').read_bytes(), b'old viewing placeholders')
