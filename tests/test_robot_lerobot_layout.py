import hashlib
import json
import shutil
import pyarrow.parquet as pq
import pytest
from YAM_control import lerobot_dataset as lr
from test_yam_lerobot_dataset import fixture

@pytest.mark.parametrize('arms',[1,2])
def test_so101_visualizer_preserves_native_layout(tmp_path,arms):
    episode=fixture(tmp_path)
    meta=json.loads((episode/'manifest.json').read_text())
    names=[f'arm{a}_joint{j}' for a in range(arms) for j in range(5)]
    cameras=('front',) if arms==1 else ('overhead','side','left','right')
    meta.update(hardware='so101' if arms==1 else 'bimanual_so101',robot_id='robot-test',joint_names=names,camera_devices={n:i for i,n in enumerate(cameras)})
    rows=[json.loads(line) for line in (episode/'samples.jsonl').read_text().splitlines()]
    for row in rows:
        row['measured_joints']=list(range(arms*5));row['action_joints']=[x+.1 for x in range(arms*5)]
        row['measured_grippers']=[.3,.8][:arms];row['action_grippers']=[.4,.9][:arms]
        for name in cameras:
            for suffix in ('_image_path','_frame_age_s','_camera_timestamp','_frame_sequence'):
                row[name+suffix]=row['top'+suffix]
    raw=''.join(json.dumps(r)+'\n' for r in rows).encode();(episode/'samples.jsonl').write_bytes(raw)
    meta['samples_sha256']=hashlib.sha256(raw).hexdigest();(episode/'manifest.json').write_text(json.dumps(meta))
    layout=lr.robot_layout(meta);out=tmp_path/'export'
    entry=lr.append_episode(out,episode,layout=layout)
    info=json.loads((out/'meta/info.json').read_text())
    assert info['robot_type']==meta['hardware']
    assert info['total_videos']==len(cameras)
    assert info['features']['observation.state']['shape']==[arms*6]
    data=pq.read_table(out/lr.episode_files(0,layout=layout)[0]).to_pylist()
    assert data[0]['observation.state'][:5]==[0,1,2,3,4]
    assert data[0]['observation.state'][5]==pytest.approx(.3)
    assert lr.append_episode(out,episode,layout=layout)==entry
    assert len(lr.MOTORS)==14 and lr.CAMERA_ORDER==('left','top','right')
