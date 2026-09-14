import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from YAM_control.training_video import RENDER_VERSION, motion_frames, render_video
from YAM_control.training_recorder import EpisodeRecorder


class ViewingPreviewTests(unittest.TestCase):
    def test_backfill_only_adds_preview_and_manifest_then_skips(self):
        import hashlib
        import shutil
        from unittest.mock import patch
        from scripts.publish_yam_previews import publish
        with tempfile.TemporaryDirectory() as root, tempfile.TemporaryDirectory() as remote:
            episode = Path(root)/'ep_test'; episode.mkdir()
            samples = b'{}\n'; (episode/'samples.jsonl').write_bytes(samples)
            meta = {'episode_id':'ep_test', 'status':'finalized', 'samples_sha256':hashlib.sha256(samples).hexdigest()}
            (episode/'manifest.json').write_text(json.dumps(meta))
            (Path(remote)/'episodes').mkdir()
            (Path(remote)/'episodes/ep_test.json').write_text(json.dumps(meta))
            class Hub:
                uploads = 0
                def repo_info(self, **kw): return SimpleNamespace(private=False, sha='before')
                def file_exists(self, repo, path, **kw): return (Path(remote)/path).exists()
                def upload_folder(self, **kw):
                    self.uploads += 1
                    self.paths = sorted(str(p.relative_to(kw['folder_path'])) for p in Path(kw['folder_path']).rglob('*') if p.is_file())
                    assert kw['parent_commit'] == 'before'
                    shutil.copytree(kw['folder_path'], remote, dirs_exist_ok=True)
                    return SimpleNamespace(oid='after')
            hub = Hub()
            def render(source, destination, **kw):
                self.assertTrue(kw['preview']); Path(destination).write_bytes(b'preview'); return {'speed':10, 'render_version':RENDER_VERSION}
            with patch('scripts.publish_yam_previews.render_video', render):
                args = dict(api=hub, download=lambda repo, path, **kw: str(Path(remote)/path))
                self.assertEqual(publish(root,'public',**args), ['ep_test'])
                self.assertEqual(publish(root,'public',**args), [])
                manifest = Path(remote)/'episodes/ep_test.json'
                old = json.loads(manifest.read_text())
                old['preview']['render_version'] = RENDER_VERSION - 1
                manifest.write_text(json.dumps(old))
                self.assertEqual(publish(root,'public',**args), ['ep_test'])
                self.assertEqual(publish(root,'public',**args), [])
            self.assertEqual(hub.paths, ['episodes/ep_test.json','previews/ep_test.mp4'])
            self.assertEqual(hub.uploads,2)

    def rows(self):
        return [dict(timestamp=n/10, measured_joints=[max(0, min(n-30, 10))*.01]*12,
                     measured_grippers=[0, 0]) for n in range(80)]

    def test_motion_and_gripper_kept_idle_cut_gaps_retained(self):
        rows = self.rows()
        keep = motion_frames(rows, 80)
        self.assertTrue(set(range(30, 41)).issubset(keep))
        self.assertNotIn(15, keep)
        self.assertNotIn(60, keep)
        rows = [r for r in rows if not 4.5 < r['timestamp'] < 7]
        self.assertIn(60, motion_frames(rows, 80))
        rows = self.rows()
        for n, r in enumerate(rows):
            r['measured_joints'] = [0]*12
            r['measured_grippers'] = [0 if n < 40 else .5, 0]
        self.assertIn(40, motion_frames(rows, 80))

    def test_side_preview_geometry_and_missing_side_frame(self):
        from PIL import Image
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root/'images').mkdir()
            rows = self.rows()
            for camera, color in [('top', 'red'), ('observer', 'green'), ('left', 'blue'), ('right', 'yellow')]:
                Image.new('RGB', (16, 12), color).save(root/'images'/f'{camera}.jpg')
                for row in rows:
                    row[camera+'_image_path'] = f'images/{camera}.jpg'
            del rows[35]['observer_image_path']
            (root/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (root/'manifest.json').write_text(json.dumps({'status': 'finalized'}))
            output = root/'preview.mp4'
            info = render_video(root, output, preview=True)
            self.assertEqual(info['camera_order'], ['top','observer','left','right'])
            self.assertGreater(info['idle_removed_s'], 3)
            probe = json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-of','json',str(output)]))
            self.assertEqual((probe['streams'][0]['width'], probe['streams'][0]['height']), (1280,768))
            subprocess.run(['ffmpeg','-v','error','-i',str(output),'-f','null','-'],check=True)

    def test_recording_gaps_hold_past_image_but_long_outage_stays_visible(self):
        from PIL import Image
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root/'images').mkdir()
            Image.new('RGB', (640,360), 'red').save(root/'images/red.jpg')
            Image.new('RGB', (640,360), 'green').save(root/'images/green.jpg')
            rows = [dict(timestamp=t, **{n+'_image_path': 'images/'+color+'.jpg' for n in ('left','top','right')})
                    for t,color in [(0,'red'),(.3,'red'),(4.5,'green')]]
            (root/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (root/'manifest.json').write_text(json.dumps({'status':'finalized'}))
            output = root/'test.mp4'; info = render_video(root,output)
            raw = subprocess.check_output(['ffmpeg','-v','error','-i',str(output),'-vf','crop=2:2:320:180',
                                           '-f','rawvideo','-pix_fmt','rgb24','-'])
            pixels = [raw[i:i+3] for i in range(0,len(raw),12)]
            self.assertGreater(pixels[2][0],200)  # 200 ms gap is red, not blank.
            self.assertLess(max(pixels[40]),60)  # Long gap remains explicitly missing.
            self.assertGreater(pixels[45][1],80)  # New green frame only at its timestamp.
            self.assertGreater(info['held_camera_ticks'],0)

    def test_preview_omits_long_camera_gaps_without_future_images(self):
        from PIL import Image
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); (root/'images').mkdir()
            Image.new('RGB',(640,360),'red').save(root/'images/red.jpg')
            Image.new('RGB',(640,360),'green').save(root/'images/green.jpg')
            rows=[dict(timestamp=t, measured_joints=[t]*12, measured_grippers=[0,0],
                       **{n+'_image_path':'images/'+color+'.jpg' for n in ('top','observer','left','right')})
                  for t,color in [(.1,'red'),(.3,'red'),(12,'green')]]
            (root/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (root/'manifest.json').write_text(json.dumps({'status':'finalized'}))
            output=root/'preview.mp4'; info=render_video(root,output,preview=True)
            raw=subprocess.check_output(['ffmpeg','-v','error','-i',str(output),'-vf','crop=2:2:320:180',
                                         '-f','rawvideo','-pix_fmt','rgb24','-'])
            pixels=[raw[i:i+3] for i in range(0,len(raw),12)]
            self.assertTrue(all(max(p)>80 for p in pixels))
            self.assertTrue(all(p[0]>200 for p in pixels[:-1]))
            self.assertGreater(pixels[-1][1],80)
            self.assertEqual(info['missing_camera_ticks'],0)
            self.assertGreater(info['unavailable_ticks_removed'],0)

    def test_preview_omits_single_camera_dropout_including_frame_age(self):
        from PIL import Image
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root/'images').mkdir()
            Image.new('RGB', (640, 360), 'red').save(root/'images/red.jpg')
            rows = []
            for n in range(61):
                row = dict(timestamp=n/10, measured_joints=[n*.01]*12,
                           measured_grippers=[0, 0],
                           **{name+'_image_path':'images/red.jpg'
                              for name in ('top','observer','left','right')})
                # The sample is current, but its right-camera image is stale.
                if 10 <= n < 50:
                    row['right_frame_age_s'] = 4
                rows.append(row)
            (root/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (root/'manifest.json').write_text(json.dumps({'status':'finalized'}))
            output = root/'preview.mp4'
            info = render_video(root, output, preview=True)
            for x, y in [(320,180), (960,180), (320,564), (960,564)]:
                raw = subprocess.check_output(['ffmpeg','-v','error','-i',str(output),
                    '-vf',f'crop=2:2:{x}:{y}','-f','rawvideo','-pix_fmt','rgb24','-'])
                self.assertTrue(raw)
                self.assertTrue(all(raw[i] > 200 for i in range(0,len(raw),12)))
            self.assertEqual(info['missing_camera_ticks'], 0)
            self.assertEqual(info['render_version'], RENDER_VERSION)
            self.assertGreater(info['unavailable_ticks_removed'], 0)

    def test_video_ends_on_last_picture_despite_late_episode_stop(self):
        from PIL import Image
        import subprocess
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root/'images').mkdir()
            Image.new('RGB',(640,360),'red').save(root/'images/red.jpg')
            Image.new('RGB',(640,360),'green').save(root/'images/green.jpg')
            rows = [dict(timestamp=i/10, measured_joints=[i*.01]*12, measured_grippers=[0,0],
                         **{n+'_image_path': 'images/'+('green' if i==9 else 'red')+'.jpg'
                            for n in ('top','observer','left','right')}) for i in range(10)]
            (root/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            (root/'manifest.json').write_text(json.dumps({'status':'finalized','started_monotonic':0,'ended_monotonic':8}))
            for preview in (False,True):
                output=root/'clip.mp4';info=render_video(root,output,preview=preview)
                raw=subprocess.check_output(['ffmpeg','-v','error','-i',str(output),'-vf','crop=2:2:320:180',
                                             '-f','rawvideo','-pix_fmt','rgb24','-'])
                self.assertGreater(raw[-11],80)  # Last picture is green, never blank.
                self.assertLess(info['duration_s'],2)
                self.assertGreater(info['trimmed_tail_s'],6)

    def test_optional_side_does_not_change_training_validity(self):
        from PIL import Image
        import io
        image = io.BytesIO(); Image.new('RGB',(16,12)).save(image,format='JPEG')
        with tempfile.TemporaryDirectory() as directory:
            state = SimpleNamespace(left=SimpleNamespace(joints_rad=[0]*6,gripper=0), right=SimpleNamespace(joints_rad=[0]*6,gripper=0))
            recorder = EpisodeRecorder(directory,'ep_test',None,frames=object())
            (recorder.path/'images').mkdir(parents=True)
            frames = {name: {'jpeg':image.getvalue(),'monotonic':100,'sequence':1} for name in ('top','left','right')}
            frames['observer'] = dict(frames['top'], monotonic=0)
            row = recorder._row(frames,(state,dict(joints=[0]*12,grippers=[0,0],monotonic=100)),100.05)
            self.assertEqual(row['measured_joint_effort'], [None]*12)
            self.assertEqual(row['measured_gripper_effort'], [None]*2)
            state.left.joint_effort = tuple(range(6))
            state.right.joint_effort = tuple(range(6,12))
            row = recorder._row(frames,(state,dict(joints=[0]*12,grippers=[0,0],monotonic=100)),100.05)
            self.assertEqual(row['measured_joint_effort'], list(range(12)))
            self.assertTrue(row['training_valid'])
            self.assertEqual(row['camera_skew_s'],0)
