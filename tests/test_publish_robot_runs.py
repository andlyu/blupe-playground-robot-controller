"""Real JPEG/Parquet/MP4 export plus a conflict-and-retry Hub simulation."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from PIL import Image
from scripts.publish_robot_runs import publish_training
from scripts.publish_yam_viewing import publish_episode, INDEX


def test_native_bimanual_export_and_publication(tmp_path):
    episode=tmp_path/'ep_maker';(episode/'images').mkdir(parents=True)
    cameras={'overhead':6,'side':3,'left':4,'right':0}
    rows=[]
    for i in range(35):
        row=dict(episode_id='ep_maker',frame_index=i,timestamp=i*.1,measured_joints=[i*.01]*10,
                 measured_grippers=[.1,.2],action_joints=[.1]*10,action_grippers=[.1,.2],
                 action_valid=True,action_timestamp=0,camera_skew_s=.01,training_valid=True)
        for role in cameras:
            relative=f'images/{role}-{i}.jpg';Image.new('RGB',(32,24),'red').save(episode/relative)
            row.update({role+'_image_path':relative,role+'_camera_timestamp':i*.1,
                        role+'_frame_age_s':.01,role+'_frame_sequence':i})
        rows.append(row)
    raw=''.join(json.dumps(row)+'\n' for row in rows).encode();(episode/'samples.jsonl').write_bytes(raw)
    meta=dict(status='finalized',episode_id='ep_maker',robot_id='robot-maker',hardware='bimanual_so101',
              fps=10,rows=35,valid_rows=35,started_at=1,task='Move block',outcome='user_requested',
              joint_names=[f'{s}_{j}' for s in ('left','right') for j in range(5)],joint_units='radians',
              gripper_units='normalized_0_closed_1_open',camera_devices=cameras,
              samples_sha256=hashlib.sha256(raw).hexdigest())
    (episode/'manifest.json').write_text(json.dumps(meta))
    remote={INDEX:b'{"episode_id":"ep_yam","started_at":0,"task":"YAM run","preview":{}}\n'}
    commits=[]
    class Hub:
        def repo_info(self,**kw):return SimpleNamespace(private=False,sha=str(len(commits)))
        def file_exists(self,repo,path,**kw):return path in remote
        def create_commit(self,**kw):
            for op in kw['operations']:remote[op.path_in_repo]=Path(op.path_or_fileobj).read_bytes()
            return SimpleNamespace(oid='preview')
        def upload_folder(self,**kw):
            commits.append(kw)
            if len(commits)==1:
                remote[INDEX]+=b'{"episode_id":"ep_concurrent","started_at":1,"preview":{}}\n'
                error=RuntimeError();error.response=SimpleNamespace(status_code=412);raise error
            stage=Path(kw['folder_path'])
            remote.update({str(p.relative_to(stage)):p.read_bytes() for p in stage.rglob('*') if p.is_file()})
            return SimpleNamespace(oid='archive')
    def download(repo,path,**kwargs):
        target=tmp_path/'download'/path;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(remote[path]);return str(target)
    with patch('scripts.publish_robot_runs.time.sleep'):
        publish_episode(episode,'repo',Hub(),download)
        publish_training(episode,'repo',Hub(),download)
        publish_training(episode,'repo',Hub(),download)
    assert len(commits)==2
    entries={row['episode_id']:row for row in map(json.loads,remote[INDEX].splitlines())}
    assert set(entries)=={'ep_yam','ep_concurrent','ep_maker'}
    assert entries['ep_maker']['robot_id']=='robot-maker'
    assert entries['ep_maker']['preview']['camera_order']==list(cameras)
    assert entries['ep_maker']['original_available'] is True
    import yaml
    card = yaml.safe_load(remote['README.md'].decode().split('---')[1])
    assert {c['config_name'] for c in card['configs']} == {'default','lerobot','robot-maker'}
    assert 'meta/info.json' not in remote  # Existing YAM LeRobot schema untouched.
    import pyarrow.parquet as pq
    data=tmp_path/'saved.parquet';data.write_bytes(remote['robots/robot-maker/data/ep_maker.parquet'])
    table=pq.read_table(data)
    assert len(table.to_pylist()[0]['measured_joints'])==10
    assert all(role+'_image' in table.column_names for role in cameras)
    import cv2
    video=cv2.VideoCapture(str(episode/'viewing.mp4'))
    ok,frame=video.read();video.release()
    assert ok and frame.shape[:2]==(768,1280)


def test_cloud_sync_only_publishes_finalized_and_releases_verified_copy(tmp_path):
    from scripts.sync_robot_recordings import sync_once
    root=tmp_path/'mirror';(root/'manifests/ep_ready').mkdir(parents=True)
    (root/'manifests/ep_live').mkdir()
    meta={'status':'finalized','episode_id':'ep_ready','samples_sha256':'digest'}
    (root/'manifests/ep_ready/manifest.json').write_text(json.dumps(meta))
    (root/'manifests/ep_live/manifest.json').write_text(json.dumps({'status':'recording'}))
    transfers=[]
    def rsync(command,**kw):
        transfers.append(command)
        if command[-2]=='maker@host:ep_ready/':
            (Path(command[-1])/'manifest.json').write_text(json.dumps(meta))
    def archive(episode,*args):
        (episode/'robot-upload.json').write_text(json.dumps({'repo':'repo','samples_sha256':'digest','commit':'verified'}))
    with patch('scripts.sync_robot_recordings.subprocess.run',rsync), patch('scripts.sync_robot_recordings.publish_episode') as preview, patch('scripts.sync_robot_recordings.publish_training',archive):
        sync_once(root,'maker@host','ssh','repo',None,None)
        sync_once(root,'maker@host','ssh','repo',None,None)
    assert preview.call_count==1
    assert sum(command[-2]=='maker@host:ep_ready/' for command in transfers)==1
    assert not (root/'working/ep_ready').exists()
    assert json.loads((root/'manifests/ep_ready/published.json').read_text())['commit']=='verified'
