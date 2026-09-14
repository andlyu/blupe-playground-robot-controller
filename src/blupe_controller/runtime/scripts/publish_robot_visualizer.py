"""Publish a robot's LeRobot playback mirror, preserving the shared raw archive."""
import json
from pathlib import Path
import shutil
import tempfile
import time

from YAM_control import lerobot_dataset as lr
from YAM_control.training_recorder import atomic_json
from scripts.publish_yam_training import LEROBOT_META


def publish_visualizer(episode, repo, api, download):
    episode = Path(episode)
    meta = json.loads((episode/'manifest.json').read_text())
    layout = lr.robot_layout(meta)
    # Full physical-source validation also covers retries with an existing remote entry.
    _, rows = lr.load_episode(episode, layout=layout)
    fingerprint = lr.source_fingerprint(episode, meta, rows, layout=layout)
    for attempt in range(8):
        info = api.repo_info(repo_id=repo, repo_type='dataset')
        if info.private:
            raise ValueError('Visualizer mirror must be public')
        with tempfile.TemporaryDirectory(prefix='visualizer-', dir=episode.parent) as temp:
            stage = Path(temp)
            if api.file_exists(repo, lr.REGISTRY, repo_type='dataset', revision=info.sha):
                for name in LEROBOT_META:
                    src = download(repo, name, repo_type='dataset', revision=info.sha)
                    dst = stage/name; dst.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(src, dst)
            elif api.file_exists(repo, 'meta/info.json', repo_type='dataset', revision=info.sha):
                raise ValueError('Visualizer has no source registry')
            entry = next((e for e in lr.read_jsonl(stage/lr.REGISTRY) if e['episode_id']==meta['episode_id']), None)
            if entry and entry['source_sha256'] != fingerprint:
                raise ValueError('Visualizer source identity mismatch')
            if not entry:
                entry = lr.append_episode(stage, episode, layout=layout)
                (stage/'README.md').write_text('---\ntags:\n- robotics\n- lerobot\nconfigs:\n- config_name: default\n  data_files:\n  - split: train\n    path: data/chunk-*/*.parquet\n---\n\n# '+layout.robot_type+' run visualizer\n\nLeRobot v2.1 playback mirror for `'+meta['robot_id']+'`.\n\nOriginal images and telemetry remain in [the shared archive](https://huggingface.co/datasets/andlyu/Public-YAM-runs). Failed runs are retained; these are not all successful demonstrations.\n')
                try:
                    commit = api.upload_folder(repo_id=repo,repo_type='dataset',folder_path=str(stage),parent_commit=info.sha,
                                               commit_message='Visualize '+meta['episode_id'])
                    revision = commit.oid
                except Exception as error:
                    if getattr(getattr(error,'response',None),'status_code',None) not in (409,412) or attempt==7:
                        raise
                    time.sleep(min(2**attempt,15)); continue
            else:
                revision = info.sha
            for name in [*LEROBOT_META,*lr.episode_files(entry['episode_index'],layout=layout)]:
                if not api.file_exists(repo,name,repo_type='dataset',revision=revision):
                    raise RuntimeError('Incomplete visualizer upload')
            receipt = dict(repo=repo,commit=revision,episode_index=entry['episode_index'],
                           samples_sha256=meta['samples_sha256'],uploaded_at=time.time())
            atomic_json(episode/'visualizer-upload.json',receipt)
            return receipt
