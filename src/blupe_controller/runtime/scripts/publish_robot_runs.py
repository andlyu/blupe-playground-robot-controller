"""Publish non-YAM physical recordings into the shared archive with native schemas.

YAM's existing LeRobot root remains unchanged. Each other robot has its own
Parquet data section; viewing and episode metadata share the existing catalog.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import tempfile
import time

from YAM_control.training_recorder import atomic_json
from scripts.publish_yam_training import export_episode, recover_orphan
from scripts.publish_yam_viewing import publish_episode, INDEX


def publish_training(episode, repo, api, download):
    episode = Path(episode)
    meta = json.loads((episode/'manifest.json').read_text())
    if meta.get('status') != 'finalized' or meta.get('simulated'):
        return
    rid, eid = meta.get('robot_id', ''), meta['episode_id']
    if (not re.fullmatch(r'robot-[A-Za-z0-9_-]+', rid) or
            not re.fullmatch(r'ep_[A-Za-z0-9_-]+', eid)):
        raise ValueError('Robot and episode IDs required')
    digest = hashlib.sha256((episode/'samples.jsonl').read_bytes()).hexdigest()
    if digest != meta['samples_sha256']:
        raise ValueError('Finalized recording changed')
    receipt = episode/'robot-upload.json'
    if receipt.exists():
        saved = json.loads(receipt.read_text())
        if saved.get('samples_sha256') == digest and saved.get('repo') == repo:
            return
    prefix = f'robots/{rid}'
    with tempfile.TemporaryDirectory(prefix='robot-export-', dir=episode.parent) as temporary:
        stage = Path(temporary)
        manifest = export_episode(episode, stage)
        data_dir = stage/prefix/'data'
        data_dir.parent.mkdir(parents=True)
        shutil.move(str(stage/'data'), str(data_dir))
        manifest.update(dataset_title='Public robot runs', parquet_path=f'{prefix}/data/{eid}.parquet')
        atomic_json(stage/'episodes'/f'{eid}.json', manifest)
        schema = {k:meta[k] for k in ('robot_id','hardware','joint_names','joint_units','gripper_units','fps')}
        schema.update(camera_order=list(meta['camera_devices']), format='embedded-image-parquet', data_glob=f'{prefix}/data/*.parquet')
        atomic_json(stage/prefix/'schema.json', schema)
        for attempt in range(8):
            info = api.repo_info(repo_id=repo, repo_type='dataset')
            if info.private:
                raise ValueError('Dataset must be public')
            revision = info.sha
            # A robot's native schema must never silently change between runs.
            schema_path = f'{prefix}/schema.json'
            if api.file_exists(repo, schema_path, repo_type='dataset', revision=revision):
                previous = json.loads(Path(download(repo, schema_path, repo_type='dataset', revision=revision)).read_text())
                if previous != schema:
                    raise ValueError('Robot dataset schema changed; migration required')
            remote_manifest = f'episodes/{eid}.json'
            if api.file_exists(repo, remote_manifest, repo_type='dataset', revision=revision):
                saved = json.loads(Path(download(repo, remote_manifest, repo_type='dataset', revision=revision)).read_text())
                if any(saved.get(k) != manifest.get(k) for k in ('robot_id','samples_sha256','images_sha256','task','outcome')):
                    raise ValueError('Episode ID already belongs to different data')
            import yaml
            card = (Path(download(repo,'README.md',repo_type='dataset',revision=revision)).read_text()
                    if api.file_exists(repo,'README.md',repo_type='dataset',revision=revision)
                    else (Path(__file__).resolve().parents[1]/'docs/PUBLIC-YAM-DATASET-CARD.md').read_text())
            if not card.startswith('---\n') or '\n---\n' not in card[4:]:
                raise ValueError('Dataset card has no YAML metadata')
            header, body = card[4:].split('\n---\n',1)
            metadata = yaml.safe_load(header)
            configs = metadata.setdefault('configs',[])
            wanted = {'config_name':rid,'data_files':[{'split':'train','path':f'{prefix}/data/*.parquet'}]}
            existing = next((item for item in configs if item['config_name']==rid),None)
            if existing is not None and existing != wanted:
                raise ValueError('Dataset config already has a different definition')
            if existing is None:
                configs.append(wanted)
            (stage/'README.md').write_text('---\n'+yaml.safe_dump(metadata,sort_keys=False)+'---\n'+body)
            entries = {}
            if api.file_exists(repo, INDEX, repo_type='dataset', revision=revision):
                entries = {r['episode_id']:r for r in (json.loads(line) for line in
                           Path(download(repo,INDEX,repo_type='dataset',revision=revision)).read_text().splitlines() if line)}
            entries[eid] = {**{k:meta.get(k) for k in ('episode_id','robot_id','hardware','started_at','task','outcome','rows','samples_sha256')},
                            'preview':manifest['preview'], 'original_available':True,
                            'parquet_path':manifest['parquet_path']}
            # Viewing URL remains valid regardless of whether the smaller publisher ran first.
            (stage/'viewing').mkdir(exist_ok=True)
            shutil.copyfile(stage/'previews'/f'{eid}.mp4', stage/'viewing'/f'{eid}.mp4')
            (stage/'meta').mkdir(exist_ok=True)
            (stage/INDEX).write_text(''.join(json.dumps(row)+'\n' for row in entries.values()))
            try:
                commit = api.upload_folder(repo_id=repo, repo_type='dataset', folder_path=str(stage),
                    parent_commit=revision, commit_message=f'Archive {rid} episode {eid}')
                break
            except Exception as error:
                if getattr(getattr(error,'response',None),'status_code',None) not in (409,412) or attempt == 7:
                    raise
                time.sleep(min(2**attempt,15))
        required = [remote_manifest, manifest['parquet_path'], f'videos/{eid}.mp4', f'viewing/{eid}.mp4', schema_path, INDEX]
        if not all(api.file_exists(repo,p,repo_type='dataset',revision=commit.oid) for p in required):
            raise RuntimeError('Archive upload verification failed')
        atomic_json(receipt, dict(repo=repo, samples_sha256=digest, commit=commit.oid, uploaded_at=time.time()))


def main():
    import fcntl
    from huggingface_hub import HfApi, hf_hub_download
    parser = argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--repo',default='andlyu/Public-YAM-runs')
    parser.add_argument('--once',action='store_true')
    args = parser.parse_args()
    args.root.mkdir(parents=True,exist_ok=True)
    with (args.root/'.robot-publisher.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        while True:
            for path in sorted(args.root.glob('ep_*/manifest.json')):
                try:
                    recover_orphan(path.parent,json.loads(path.read_text()))
                    publish_episode(path.parent,args.repo,HfApi(),hf_hub_download)
                    publish_training(path.parent,args.repo,HfApi(),hf_hub_download)
                except Exception as error:
                    print(f'[robot-publisher] {path.parent.name} error={type(error).__name__}',flush=True)
            if args.once:
                return
            time.sleep(10)


if __name__ == '__main__':
    main()
