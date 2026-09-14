"""Backfill viewing previews from retained recordings, preserving original archives.

Run beside the dataset publisher with the same HF configuration. Compare-and-swap
commits make concurrent publication safe; interrupted runs resume by manifest hash.
"""
import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import time

from YAM_control.training_recorder import atomic_json
from YAM_control.training_video import RENDER_VERSION, render_video


def publish(root, repo, *, api=None, download=None):
    from huggingface_hub import HfApi, hf_hub_download
    api = api or HfApi()
    download = download or hf_hub_download
    results = []
    paths = sorted(Path(root).glob('ep_*/manifest.json'), key=lambda p: json.loads(p.read_text()).get('started_at', 0), reverse=True)
    for path in paths:
        episode = path.parent
        meta = json.loads(path.read_text())
        if meta.get('status') != 'finalized' or meta.get('simulated'):
            continue
        eid = meta['episode_id']
        info = api.repo_info(repo_id=repo, repo_type='dataset')
        if info.private:
            raise ValueError('Dataset must be public')
        revision = info.sha
        manifest_path = f'episodes/{eid}.json'
        if not api.file_exists(repo, manifest_path, repo_type='dataset', revision=revision):
            continue  # The regular publisher owns new recordings.
        remote = json.loads(Path(download(repo, manifest_path, repo_type='dataset', revision=revision)).read_text())
        digest = hashlib.sha256((episode/'samples.jsonl').read_bytes()).hexdigest()
        if digest != meta['samples_sha256'] or digest != remote['samples_sha256']:
            raise ValueError(f'Recording hash mismatch: {eid}')
        if (remote.get('preview', {}).get('samples_sha256') == digest
                and remote.get('preview', {}).get('render_version') == RENDER_VERSION
                and api.file_exists(repo, f'previews/{eid}.mp4', repo_type='dataset', revision=revision)):
            continue
        with tempfile.TemporaryDirectory(prefix='yam-preview-') as temp:
            target = Path(temp)
            (target/'previews').mkdir(); (target/'episodes').mkdir()
            video = target/'previews'/f'{eid}.mp4'
            preview = render_video(episode, video, preview=True)
            preview.update(samples_sha256=digest, sha256=hashlib.sha256(video.read_bytes()).hexdigest())
            for attempt in range(8):
                # The normal uploader can commit while a preview is encoding.
                # Rebase metadata only after verifying the same source recording.
                revision = api.repo_info(repo_id=repo, repo_type='dataset').sha
                remote = json.loads(Path(download(repo, manifest_path, repo_type='dataset', revision=revision)).read_text())
                if remote['samples_sha256'] != digest:
                    raise ValueError(f'Remote recording changed: {eid}')
                remote['preview'] = preview
                atomic_json(target/manifest_path, remote)
                try:
                    commit = api.upload_folder(repo_id=repo, repo_type='dataset', folder_path=temp,
                                               parent_commit=revision, commit_message=f'Add viewing preview for {eid}')
                    break
                except Exception as exc:
                    if getattr(getattr(exc, 'response', None), 'status_code', None) != 409 or attempt == 7:
                        raise
                    time.sleep(min(2 ** attempt, 15))
            saved = json.loads(Path(download(repo, manifest_path, repo_type='dataset', revision=commit.oid)).read_text())
            if saved != remote or not api.file_exists(repo, f'previews/{eid}.mp4', repo_type='dataset', revision=commit.oid):
                raise RuntimeError('Preview publication could not be verified')
            results.append(eid)
            print(f'Published viewing preview: {eid}', flush=True)
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True)
    parser.add_argument('--repo', default='andlyu/Public-YAM-runs')
    args = parser.parse_args()
    publish(args.root, args.repo)
