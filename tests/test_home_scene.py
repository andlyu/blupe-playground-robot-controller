import base64
import json
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/blupe_controller/runtime'))
from YAM_control.home_scene import HomeScene, home_duration
from YAM_control.auto_queue import AutoQueue
from YAM_control.i2rt_bimanual_adapter import AdapterState
from YAM_control.ruckig_waypath import plan_joint_trajectory


class Operator:
    def __init__(self):
        self.home=(0.,)*12;self._safety_config='limits-v1'
        self.api_lease_id=None;self._hardware_pending=None
        self.grip=1.;self.joints=(0.,)*12;self.messages=[]
        self.api_home_scene_supported=True
        self.api_client=NS(send=self.messages.append)
        self._physical=NS(state=AdapterState.STOPPED,fault=None,snapshot=lambda:NS(
            left=NS(joints_rad=self.joints[:6],gripper=self.grip),
            right=NS(joints_rad=self.joints[6:],gripper=self.grip)))
        self.queue=[dict(session_id='s',home_planning={'profile':'a'*64})]
    def status(self,**kwargs):
        return dict(mode=self.mode if hasattr(self,'mode') else 'READY',homed=True,settled=True,
                    feedback_available=True,safety={'controller_ok':True})
    def queue_snapshot(self):return {'entries':self.queue}


def source(name,device):
    return name,dict(jpeg=b'\xff\xd8view\xff\xd9',monotonic=time.monotonic(),sequence=1)


def scene(op,tmp_path):
    return HomeScene(op,dict(robot_id='yam-1',cameras={'left':10,'top':16,'right':4}),tmp_path/'home.json')


def test_coherent_capture_saved_then_offered_only_once(tmp_path):
    op=Operator();cache=scene(op,tmp_path)
    with patch('YAM_control.home_scene.RelayFrames.one',side_effect=lambda _,name,device:source(name,device)):
        cache.capture(threading.Event())
    saved=json.loads(cache.path.read_text())
    assert saved['observation']['left_gripper']==1 and set(saved['frames'])=={'left','top','right'}
    assert all(base64.b64decode(f['jpeg_base64']).startswith(b'\xff\xd8') for f in saved['frames'].values())
    cache.start();assert op.messages[0]['phase']=='offer' and cache.scene is None
    cache.start();assert len(op.messages)==1
    assert scene(op,tmp_path).scene is None # Never reuse across process restarts.


@pytest.mark.parametrize('bad',['gripper','stale','skew','drift','cancel'])
def test_unverifiable_capture_never_publishes(tmp_path,bad):
    op=Operator();cache=scene(op,tmp_path);cancel=threading.Event()
    if bad=='gripper':op.grip=.5
    if bad=='cancel':cancel.set()
    def capture(_,name,device):
        key,f=source(name,device)
        if bad=='stale':f['monotonic']-=1
        if bad=='skew' and name=='top':f['monotonic']-=.2
        if bad=='drift':op.joints=(.1,)*12
        return key,f
    with patch('YAM_control.home_scene.RelayFrames.one',side_effect=capture):
        if bad=='cancel':cache.capture(cancel)
        else:
            with pytest.raises(ValueError):cache.capture(cancel)
    assert cache.scene is None and not cache.path.exists()


def test_pacing_uses_measured_mean_and_calibration_with_cold_default():
    assert home_duration(None,8) is None
    assert home_duration(float('nan'),8) is None
    assert home_duration(15,8,1)==6
    assert home_duration(2,8,1)==0
    assert home_duration(100,8,1)==30
    normal=plan_joint_trajectory(((0.,)*12,(.1,)*12))
    paced=plan_joint_trajectory(((0.,)*12,(.1,)*12),minimum_duration_s=8)
    assert paced.duration_s>=8 and paced.duration_s>=normal.duration_s
    assert paced.max_velocity_rad_s<=normal.max_velocity_rad_s+1e-8
    np.testing.assert_allclose(paced.positions[-1],normal.positions[-1])
    for value in (-1,31,float('nan')):
        with pytest.raises(ValueError):plan_joint_trajectory(((0.,)*12,(.1,)*12),minimum_duration_s=value)


def test_custom_home_wrapper_keeps_owner_and_scopes_timing_hint(tmp_path):
    from YAM_control.home_scene import install
    calls = []
    class Custom(Operator):
        def _move_home(self, *, cancel_event=None):
            calls.append((cancel_event, self._home_scene_target_duration))
            return 'custom home'
    hardware = NS(PhysicalOperator=Custom, base=NS(OperatorSimulator=Custom))
    config = dict(robot_id='yam-1', cameras={'left':10,'top':16,'right':4})
    install(hardware, config, tmp_path/'scene.json')
    cls = hardware.PhysicalOperator
    install(hardware, config)
    assert hardware.PhysicalOperator is cls
    op = cls(); cancel = threading.Event()
    assert op._move_home(cancel_event=cancel, target_duration_s=9) == 'custom home'
    assert calls == [(cancel, 9)] and op._home_scene_target_duration is None
    assert op.status()['home_scene']['version'] == 1
    extra = tmp_path/'camera_owner.py'; extra.write_text('first')
    config['home_scene_runtime_paths'] = [str(extra)]
    first = op.home_scene.fingerprint()
    extra.write_text('second')
    assert op.home_scene.fingerprint() != first


def test_launcher_installs_after_existing_customizations(tmp_path, monkeypatch):
    import runpy
    from types import ModuleType
    runtime = Path(__file__).resolve().parents[1]/'src/blupe_controller/runtime'
    launcher = runpy.run_path(str(runtime/'scripts/with_home_scene.py'))
    entry = tmp_path/'existing.py'; entry.write_text('# existing launcher')
    config = tmp_path/'config.json'; config.write_text('{"robot_id":"yam-1"}')
    monkeypatch.setenv('BLUPE_CONTROLLER_ROOT',str(tmp_path))
    monkeypatch.setenv('BLUPE_HOME_SCENE_ENTRYPOINT',str(entry))
    monkeypatch.setenv('BLUPE_HOME_SCENE_CONFIG',str(config))
    calls=[]
    hardware=NS(PhysicalOperator='original',base=NS(main=lambda:calls.append('started')))
    package=ModuleType('scripts');package.yam_operator_hardware_web=hardware
    scene_module=ModuleType('YAM_control.home_scene')
    scene_module.install=lambda hw,cfg,path:calls.append((hw.PhysicalOperator,cfg['robot_id'],path.name))
    monkeypatch.setitem(sys.modules,'scripts',package)
    monkeypatch.setitem(sys.modules,'YAM_control.home_scene',scene_module)
    def launch(path,run_name):
        assert path == str(entry) and run_name == '__main__'
        hardware.PhysicalOperator='custom'
        hardware.base.main()
    monkeypatch.setattr(runpy,'run_path',launch)
    monkeypatch.setattr(sys,'argv',['launcher','--unchanged'])
    launcher['main']()
    assert calls == [('custom','yam-1','home-scene.json'),'started']
    assert sys.argv == [str(entry),'--unchanged']


def lifecycle(tmp_path):
    op=Operator();op.mode='STOPPED';op.lock=threading.RLock();op._physical_lock=threading.RLock()
    op.stop_event=threading.Event();op.api_authorized=False;op.controller_ok=True
    op.api_connected=True;op._policy_deadline=None;op.cleanup_state={'running':False}
    op.calls=[];op.auto_queue=AutoQueue(op);op.home_scene=scene(op,tmp_path)
    def home(**kwargs):op.calls.append('home');op.mode='READY';return 'home'
    def park(**kwargs):op.calls.append('park');op.mode='DISABLED';op._physical.state=AdapterState.DISABLED;op.auto_queue.mark_parked();return 'park'
    op._move_home=home;op._park_zero_and_stop=park
    op._hold_physical=lambda reason:op.calls.append('hold')
    op.home_scene.capture=lambda cancel:op.calls.append('capture')
    return op


def test_normal_completion_captures_then_parks_even_with_queue_waiting(tmp_path):
    op=lifecycle(tmp_path);op.auto_queue.enabled=True
    op.auto_queue.policy_completed(capture_home=True);op.auto_queue.tick()
    assert op.calls==['home','capture','park'] and op.auto_queue.parked


def test_abort_adds_no_capture_or_extra_home(tmp_path):
    op=lifecycle(tmp_path);op.queue=[]
    op.auto_queue.policy_completed(capture_home=False);op.auto_queue.tick()
    assert op.calls==['park']


def test_capture_failure_still_parks_and_cancellation_prevents_capture(tmp_path):
    op=lifecycle(tmp_path)
    def fail(cancel):raise OSError('camera unavailable')
    op.home_scene.capture=fail
    op.auto_queue.policy_completed(capture_home=True);op.auto_queue.tick()
    assert op.calls==['home','park']
    op=lifecycle(tmp_path)
    def stop(**kw):op.calls.append('home');kw['cancel_event'].set()
    op._move_home=stop
    op.auto_queue.policy_completed(capture_home=True);op.auto_queue.tick()
    assert op.calls==['home','hold']


def test_relay_preserves_source_epoch_and_pixels_together(monkeypatch):
    import io
    from unittest.mock import Mock
    from YAM_control import camera_relay
    cam=camera_relay.Cam.__new__(camera_relay.Cam)
    cam.lock=threading.Lock();cam.jpeg=b'\xff\xd8frame\xff\xd9';cam.last_frame_mono=time.monotonic()
    cam.last_frame_wall=time.time()-.1;cam.frame_count=7;cam.error='';cam.stale_timeout_s=2
    monkeypatch.setitem(camera_relay.CAMS,'test',cam)
    handler=camera_relay.Handler.__new__(camera_relay.Handler)
    headers={};handler.path='/test/snapshot.jpg';handler.send_response=Mock();handler.send_error=Mock()
    handler.send_header=lambda k,v:headers.update({k:v});handler.end_headers=Mock();handler.wfile=io.BytesIO()
    handler.do_GET()
    assert headers['X-Captured-At']==str(cam.last_frame_wall)
    assert headers['X-Capture-Monotonic']==str(cam.last_frame_mono)
    assert handler.wfile.getvalue()==cam.jpeg
    handler.path='/test';handler.connection=Mock();handler.client_address=('127.0.0.1',1)
    class FirstFrame(io.BytesIO):
        def write(self,b):
            result=super().write(b)
            if b==cam.jpeg+b'\r\n':raise BrokenPipeError()
            return result
    handler.wfile=FirstFrame()
    monkeypatch.setattr(camera_relay.select,'select',lambda *args:([],[handler.connection],[]))
    handler.do_GET()
    assert f'X-Captured-At: {cam.last_frame_wall}\r\n'.encode() in handler.wfile.getvalue()
    assert cam.jpeg in handler.wfile.getvalue()
