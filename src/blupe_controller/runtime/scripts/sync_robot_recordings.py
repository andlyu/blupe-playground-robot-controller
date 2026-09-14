"""Pull finalized robot episodes through restricted rsync, then archive them.

This process owns no motors or cameras. Keep dataset credentials on this host.
"""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

from YAM_control.training_recorder import atomic_json
from scripts.publish_yam_viewing import publish_episode
from scripts.publish_robot_runs import publish_training


def sync_once(root, host, ssh, repo, api, download, visualizer_repo=None):
    root = Path(root)
    manifests = root/'manifests'
    manifests.mkdir(parents=True, exist_ok=True)
    subprocess.run(['rsync','-rt','--no-links','-e',ssh,'--include=/ep_*/',
                    '--include=/ep_*/manifest.json','--exclude=*',host+':./',str(manifests)+'/'],
                   check=True, capture_output=True, timeout=60)
    for path in sorted(manifests.glob('ep_*/manifest.json')):
        meta = json.loads(path.read_text())
        eid = path.parent.name
        if meta.get('status') != 'finalized' or meta.get('simulated'):
            continue
        if not re.fullmatch(r'ep_[A-Za-z0-9_-]+',eid) or meta.get('episode_id') != eid:
            raise ValueError('Recording identity mismatch')
        receipt = path.parent/'published.json'
        visualizer_receipt = path.parent/'visualizer-published.json'
        if receipt.exists():
            saved = json.loads(receipt.read_text())
            if saved.get('repo') == repo and saved.get('samples_sha256') == meta['samples_sha256']:
                if not visualizer_repo or (visualizer_receipt.exists() and json.loads(visualizer_receipt.read_text()).get('repo') == visualizer_repo):
                    continue
        # Process one episode at a time and release its cloud copy after upload.
        episode = root/'working'/eid
        episode.mkdir(parents=True,exist_ok=True)
        subprocess.run(['rsync','-rt','--no-links','-e',ssh,host+':'+eid+'/',str(episode)+'/'],
                       check=True,capture_output=True,timeout=600)
        if json.loads((episode/'manifest.json').read_text()) != meta:
            raise ValueError('Recording changed during transfer')
        publish_episode(episode,repo,api,download)
        publish_training(episode,repo,api,download)
        uploaded = json.loads((episode/'robot-upload.json').read_text())
        atomic_json(receipt,uploaded)
        if visualizer_repo:
            from scripts.publish_robot_visualizer import publish_visualizer
            result = publish_visualizer(episode,visualizer_repo,api,download)
            atomic_json(visualizer_receipt,result)
        shutil.rmtree(episode)
        print('[robot-sync] published '+eid,flush=True)


def main():
    import fcntl
    from huggingface_hub import HfApi, hf_hub_download
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--host',required=True)
    parser.add_argument('--ssh',required=True)
    parser.add_argument('--repo',default='andlyu/Public-YAM-runs')
    parser.add_argument('--visualizer-repo')
    args=parser.parse_args()
    args.root.mkdir(parents=True,exist_ok=True)
    with (args.root/'.sync.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        while True:
            try:
                sync_once(args.root,args.host,args.ssh,args.repo,HfApi(),hf_hub_download,args.visualizer_repo)
            except Exception as error:
                print('[robot-sync] error='+type(error).__name__,flush=True)
            time.sleep(10)


if __name__ == '__main__':
    main()
