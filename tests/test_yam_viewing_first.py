import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from scripts.publish_yam_viewing import publish_episode, INDEX
from YAM_control.training_video import RENDER_VERSION


def test_preview_publishes_without_training_files_and_rebases(tmp_path):
    episode=tmp_path/'ep_test';episode.mkdir()
    raw=b'{"timestamp":0}\n'
    (episode/'samples.jsonl').write_bytes(raw)
    meta=dict(episode_id='ep_test',status='finalized',started_at=1,task='Move block',outcome='user_requested',rows=1,samples_sha256=hashlib.sha256(raw).hexdigest())
    (episode/'manifest.json').write_text(json.dumps(meta))
    remote={}; commits=[]
    class Hub:
        def repo_info(self,**kw):return SimpleNamespace(private=False,sha=str(len(commits)))
        def file_exists(self,repo,path,**kw):return path in remote
        def create_commit(self,**kw):
            commits.append(kw)
            if len(commits)==1:
                error=RuntimeError('changed');error.response=SimpleNamespace(status_code=412);raise error
            for op in kw['operations']:remote[op.path_in_repo]=Path(op.path_or_fileobj).read_bytes()
            return SimpleNamespace(oid='ok')
    def render(ep,path,**kw):
        assert kw['preview'];Path(path).write_bytes(b'video');return dict(render_version=RENDER_VERSION,speed=10,camera_order=['top','observer','left','right'],duration_s=2)
    with patch('scripts.publish_yam_viewing.render_video',render),patch('scripts.publish_yam_viewing.time.sleep'):
        publish_episode(episode,'repo',Hub(),None)
        publish_episode(episode,'repo',Hub(),None)
    assert set(remote)=={INDEX,'viewing/ep_test.mp4'}
    assert len(commits)==2
    assert json.loads(remote[INDEX])['task']=='Move block'
    assert json.loads((episode/'viewing-upload.json').read_text())['status']=='uploaded'


def test_old_renderer_receipt_and_cached_video_are_rebuilt_once(tmp_path):
    episode = tmp_path / 'ep_test'; episode.mkdir()
    raw = b'{"timestamp":0}\n'
    digest = hashlib.sha256(raw).hexdigest()
    (episode / 'samples.jsonl').write_bytes(raw)
    (episode / 'manifest.json').write_text(json.dumps(dict(
        episode_id='ep_test', status='finalized', samples_sha256=digest)))
    (episode / 'viewing.mp4').write_bytes(b'old dark placeholders')
    (episode / 'viewing.json').write_text(json.dumps(dict(
        render_version=RENDER_VERSION - 1, samples_sha256=digest,
        sha256=hashlib.sha256(b'old dark placeholders').hexdigest())))
    (episode / 'viewing-upload.json').write_text(json.dumps(dict(
        status='uploaded', samples_sha256=digest, render_version=RENDER_VERSION - 1)))
    remote = {}; renders = []
    class Hub:
        def repo_info(self, **kw): return SimpleNamespace(private=False, sha='before')
        def file_exists(self, repo, path, **kw): return path in remote
        def create_commit(self, **kw):
            for op in kw['operations']:
                remote[op.path_in_repo] = Path(op.path_or_fileobj).read_bytes()
            return SimpleNamespace(oid='after')
    def render(ep, path, **kw):
        assert kw['preview']
        renders.append(ep)
        Path(path).write_bytes(b'new continuous preview')
        return dict(render_version=RENDER_VERSION, missing_camera_ticks=0)
    with patch('scripts.publish_yam_viewing.render_video', render):
        publish_episode(episode, 'repo', Hub(), None)
        publish_episode(episode, 'repo', Hub(), None)
    assert len(renders) == 1
    assert remote['viewing/ep_test.mp4'] == b'new continuous preview'
    assert json.loads(remote[INDEX])['preview']['render_version'] == RENDER_VERSION
    assert json.loads((episode / 'viewing-upload.json').read_text())['render_version'] == RENDER_VERSION


def test_keep_published_does_not_rebuild_legacy_receipt(tmp_path):
    episode = tmp_path/'ep_test'; episode.mkdir()
    meta = dict(episode_id='ep_test', status='finalized', samples_sha256='unchanged')
    (episode/'manifest.json').write_text(json.dumps(meta))
    receipt = dict(status='uploaded', samples_sha256='unchanged')
    (episode/'viewing-upload.json').write_text(json.dumps(receipt))
    # No samples/video files needed: this must return without rendering or remote IO.
    with patch('scripts.publish_yam_viewing.render_video') as render:
        publish_episode(episode, 'repo', None, None, keep_published=True)
    render.assert_not_called()
    assert json.loads((episode/'viewing-upload.json').read_text()) == receipt
