"""Publish small viewing previews independently of the training archive."""
import argparse
import hashlib
import json
import re
import tempfile
import time
from pathlib import Path
from YAM_control.training_recorder import atomic_json
from YAM_control.training_video import RENDER_VERSION, render_video
from YAM_control.motion_classification import classify_motion

INDEX = 'meta/blupe_viewing.jsonl'

def publish_episode(episode, repo, api, download, *, keep_published=False):
    episode = Path(episode)
    meta = json.loads((episode/'manifest.json').read_text())
    if meta.get('status') != 'finalized' or meta.get('simulated'):
        return
    eid = meta['episode_id']
    if not re.fullmatch(r'ep_[A-Za-z0-9_-]+', eid):
        raise ValueError('Invalid episode ID')
    receipt = episode/'viewing-upload.json'
    saved = json.loads(receipt.read_text()) if receipt.exists() else {}
    if saved.get('samples_sha256') == meta['samples_sha256'] and (
        saved.get('status') == 'arm_check' or
        (saved.get('status') == 'uploaded' and
         (keep_published or saved.get('render_version') == RENDER_VERSION))
    ):
        return
    raw = (episode/'samples.jsonl').read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != meta['samples_sha256']:
        raise ValueError('Finalized recording changed')
    motion = classify_motion([json.loads(line) for line in raw.splitlines()])
    if motion['kind'] == 'arm_check':
        atomic_json(receipt, {'samples_sha256':digest, 'status':'arm_check'})
        return
    # Separate filenames prevent collisions with the legacy training publisher.
    video = episode/'viewing.mp4'
    info_path = episode/'viewing.json'
    preview = json.loads(info_path.read_text()) if info_path.exists() else {}
    if (not video.exists() or preview.get('render_version') != RENDER_VERSION
            or preview.get('samples_sha256') != digest
            or preview.get('sha256') != hashlib.sha256(video.read_bytes()).hexdigest()):
        preview = render_video(episode, video, preview=True)
        preview.update(samples_sha256=digest, sha256=hashlib.sha256(video.read_bytes()).hexdigest())
        atomic_json(info_path, preview)
    entry = {key:meta.get(key) for key in ('episode_id','started_at','task','outcome','rows','samples_sha256')}
    entry.update(preview=preview)
    with tempfile.TemporaryDirectory(prefix='yam-viewing-index-') as tmp:
        target = Path(tmp)
        for attempt in range(8):
            repo_info = api.repo_info(repo_id=repo, repo_type='dataset')
            if repo_info.private:
                raise ValueError('Dataset must be public')
            revision = repo_info.sha
            entries = {}
            if api.file_exists(repo, INDEX, repo_type='dataset', revision=revision):
                entries = {r['episode_id']:r for r in (json.loads(line) for line in Path(download(repo,INDEX,repo_type='dataset',revision=revision)).read_text().splitlines() if line)}
            entries[eid] = entry
            index = target/'index.jsonl'
            index.write_text(''.join(json.dumps(r)+'\n' for r in entries.values()))
            from huggingface_hub import CommitOperationAdd
            try:
                commit = api.create_commit(repo_id=repo, repo_type='dataset', parent_commit=revision,
                    commit_message=f'Publish viewing video for {eid}', operations=[
                        CommitOperationAdd(path_in_repo=f'viewing/{eid}.mp4',path_or_fileobj=str(video)),
                        CommitOperationAdd(path_in_repo=INDEX,path_or_fileobj=str(index))])
                break
            except Exception as exc:
                if getattr(getattr(exc,'response',None),'status_code',None) not in (409,412) or attempt == 7:
                    raise
                time.sleep(min(2**attempt,15))
        if not api.file_exists(repo,f'viewing/{eid}.mp4',repo_type='dataset',revision=commit.oid):
            raise RuntimeError('Viewing upload verification failed')
    atomic_json(receipt, {'samples_sha256':digest,'status':'uploaded','render_version':preview['render_version'],
                          'commit':commit.oid,'uploaded_at':time.time()})
    print('[viewing] published '+eid,flush=True)

def main():
    import fcntl
    from huggingface_hub import HfApi, hf_hub_download
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--repo',default='andlyu/Public-YAM-runs')
    parser.add_argument('--since',type=float,default=0)
    parser.add_argument('--keep-published', action='store_true',
                        help='Leave uploaded episodes unchanged across renderer upgrades')
    args=parser.parse_args()
    with (args.root/'.viewing-publisher.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        while True:
            paths=sorted(args.root.glob('ep_*/manifest.json'),key=lambda p:json.loads(p.read_text()).get('started_at',0),reverse=True)
            for path in paths:
                if json.loads(path.read_text()).get('started_at',0)<args.since:
                    continue
                try: publish_episode(path.parent,args.repo,HfApi(),hf_hub_download,
                                     keep_published=args.keep_published)
                except Exception as exc: print('[viewing] '+path.parent.name+' error='+type(exc).__name__,flush=True)
            time.sleep(5)

if __name__=='__main__':main()
