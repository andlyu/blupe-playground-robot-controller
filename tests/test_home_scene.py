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
from YAM_control.home_scene import HomeScene
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
    with patch('YAM_control.home_scene.capture_frame',side_effect=lambda name,device:source(name,device)):
        cache.capture(threading.Event())
    saved=json.loads(cache.path.read_text())
    assert saved['observation']['left_gripper']==1 and set(saved['frames'])=={'left','top','right'}
    assert all(base64.b64decode(f['jpeg_base64']).startswith(b'\xff\xd8') for f in saved['frames'].values())
    cache.start();assert op.messages[0]['phase']=='offer' and cache.scene is None
    cache.start();assert len(op.messages)==1
    assert scene(op,tmp_path).scene is None # Never reuse across process restarts.


def test_home_capture_reads_raw_relay_frames_without_recording_client_state(tmp_path, monkeypatch):
    import io
    from YAM_control import home_scene, training_recorder
    calls = []
    jpeg = b'\xff\xd8raw-model-camera-pixels\xff\xd9'
    def recording_frame(self, *args, **kwargs):
        # Installed recording clients require constructor-owned connection state
        # and resize their JPEGs. Home evidence must not borrow this method.
        return self._local.connections
    monkeypatch.setattr(training_recorder.RelayFrames, 'one', recording_frame)
    def request(url, *, timeout):
        calls.append((url, timeout))
        result = io.BytesIO(jpeg)
        result.headers = {'X-Capture-Monotonic': str(time.monotonic()), 'X-Frame-Sequence': '7'}
        return result
    monkeypatch.setattr(home_scene, 'urlopen', request, raising=False)
    cache = scene(Operator(), tmp_path)
    cache.capture(threading.Event())
    assert {url for url, timeout in calls} == {
        f'http://127.0.0.1:8089/{device}/snapshot.jpg' for device in (10,16,4)}
    assert all(timeout == .5 for url, timeout in calls)
    assert all(base64.b64decode(frame['jpeg_base64']) == jpeg for frame in cache.scene['frames'].values())
    assert cache.path.exists()


@pytest.mark.parametrize('bad',['gripper','stale','skew','drift','cancel'])
def test_unverifiable_capture_never_publishes(tmp_path,bad):
    op=Operator();cache=scene(op,tmp_path);cancel=threading.Event()
    if bad=='gripper':op.grip=.5
    if bad=='cancel':cancel.set()
    def capture(name,device):
        key,f=source(name,device)
        if bad=='stale':f['monotonic']-=1
        if bad=='skew' and name=='top':f['monotonic']-=.2
        if bad=='drift':op.joints=(.1,)*12
        return key,f
    with patch('YAM_control.home_scene.capture_frame',side_effect=capture):
        if bad=='cancel':cache.capture(cancel)
        else:
            with pytest.raises(ValueError):cache.capture(cancel)
    assert cache.scene is None and not cache.path.exists()
    assert bool(cache.capture_error) is (bad != 'cancel')


class CaptureClock:
    def __init__(self):
        self.now = 10.
        self.stopped = False
        self.cancel_on_wait = False
    def is_set(self):
        return self.stopped
    def wait(self, seconds):
        self.now += seconds
        self.stopped = self.stopped or self.cancel_on_wait
        return self.stopped


def recapture_fixture(tmp_path, monkeypatch, failure, *, persistent=False):
    from YAM_control import home_scene
    clock = CaptureClock()
    monkeypatch.setattr(home_scene, 'time', NS(monotonic=lambda:clock.now, time=lambda:1700000000.))
    cache = scene(Operator(), tmp_path)
    measured = []
    frames = []
    def measure():
        measured.append(clock.now)
        attempt = (len(measured)+1)//2
        assert cache.scene is None and not cache.path.exists()
        failing = attempt == 1 or persistent
        if failing and failure == 'bracket' and len(measured)%2:
            clock.now += 1.01
        if failing and failure == 'expired' and not len(measured)%2:
            clock.now += .3
        return [attempt*.001]*12, [1.,1.]
    def frame(name, device):
        attempt = (len(measured)+1)//2
        stamp = clock.now
        if attempt == 1 or persistent:
            if failure == 'stale': stamp -= 1
            elif failure == 'skew' and name == 'top': stamp -= .16
            elif failure == 'future': stamp += 1
        frames.append((attempt, name))
        return name, dict(jpeg=b'\xff\xd8'+str(attempt).encode(), monotonic=stamp, sequence=attempt)
    monkeypatch.setattr(cache, '_measured', measure)
    monkeypatch.setattr(home_scene, 'capture_frame', frame)
    return cache, clock, measured, frames


@pytest.mark.parametrize('failure', ['stale', 'skew', 'expired', 'bracket', 'future'])
def test_freshness_failure_recaptures_all_frames_in_new_encoder_bracket(tmp_path, monkeypatch, failure):
    cache, clock, measured, frames = recapture_fixture(tmp_path, monkeypatch, failure)
    cache.capture(clock)
    saved = json.loads(cache.path.read_text())
    assert len(measured) == 4  # Both encoder checks run again for the new frame set.
    assert {name for attempt,name in frames if attempt == 2} == {'left','top','right'}
    assert all(base64.b64decode(f['jpeg_base64']) == b'\xff\xd82' for f in saved['frames'].values())
    assert all(f['sequence'] == 2 for f in saved['frames'].values())
    assert saved['observation']['left_joints_deg'] == pytest.approx([math.degrees(.002)]*6)
    assert cache.capture_error is None


def test_persistent_freshness_failure_is_bounded_and_never_saved(tmp_path, monkeypatch):
    cache, clock, measured, frames = recapture_fixture(tmp_path, monkeypatch, 'expired', persistent=True)
    with pytest.raises(ValueError, match='freshness'):
        cache.capture(clock)
    assert len(measured) == 6
    assert len(frames) == 9
    assert cache.scene is None and not cache.path.exists()
    assert 'freshness' in cache.capture_error


def test_cancel_during_recapture_wait_does_not_read_or_publish_again(tmp_path, monkeypatch):
    cache, clock, measured, frames = recapture_fixture(tmp_path, monkeypatch, 'expired')
    original = cache._measured
    def measure():
        result = original()
        if len(measured) == 2:
            clock.cancel_on_wait = True
        return result
    monkeypatch.setattr(cache, '_measured', measure)
    cache.capture(clock)
    assert len(frames) == 3 and len(measured) == 2
    assert cache.scene is None and not cache.path.exists()


def test_pose_drift_is_terminal_even_when_frames_are_also_expired(tmp_path, monkeypatch):
    cache, clock, measured, frames = recapture_fixture(tmp_path, monkeypatch, 'expired')
    original = cache._measured
    def measure():
        q,g = original()
        return ([value+.1 for value in q] if len(measured)%2 == 0 else q),g
    monkeypatch.setattr(cache, '_measured', measure)
    with pytest.raises(ValueError, match='pose changed'):
        cache.capture(clock)
    assert len(measured) == 2 and len(frames) == 3
    assert cache.scene is None and not cache.path.exists()


@pytest.mark.parametrize('unsafe', ['pose', 'lease', 'fault'])
def test_recapture_rechecks_home_and_stops_on_changed_safety_state(tmp_path, monkeypatch, unsafe):
    cache, clock, measured, frames = recapture_fixture(tmp_path, monkeypatch, 'expired')
    original = cache._measured
    def measure():
        if len(measured) == 2:
            if unsafe == 'pose': cache.op.grip = .5
            elif unsafe == 'lease': cache.op.api_lease_id = 'new-lease'
            elif unsafe == 'fault': cache.op.status = lambda **kw: {'safety': {'controller_ok':False}}
            return HomeScene._measured(cache)
        return original()
    monkeypatch.setattr(cache, '_measured', measure)
    with pytest.raises(ValueError, match='Home'):
        cache.capture(clock)
    assert len(frames) == 3 and len(measured) == 2
    assert cache.scene is None and not cache.path.exists()


def accepted_cache(op, cache):
    fingerprint = cache.fingerprint()
    cache.offer = dict(session_id='s', scene=dict(config_sha256=fingerprint, captured_at=time.time()))
    cache.timing = dict(started_monotonic=time.monotonic()-8)
    op.queue[0]['home_planning'].update(config_sha256=fingerprint, sample_count=0)


@pytest.mark.parametrize('calibration_s,mean_s', [(0,None),(8,None),(20,2),(8,100)])
def test_accepted_cache_gets_ten_second_home_without_timing_history(tmp_path, calibration_s, mean_s):
    op=Operator();cache=scene(op,tmp_path);accepted_cache(op,cache)
    cache.timing['started_monotonic']=time.monotonic()-calibration_s
    op.queue[0]['home_planning']['mean_first_plan_s']=mean_s
    assert cache.duration()==10.0
    assert cache.timing['using_cache'] is True
    assert cache.timing['target_home_s']==10.0


@pytest.mark.parametrize('case', ['cold','not_accepted','queue_changed','empty_queue','ineligible',
    'configuration_changed','scene_changed','stale','future','invalidated','api_unsupported'])
def test_no_usable_cache_preserves_normal_home_timing(tmp_path, case):
    op=Operator();cache=scene(op,tmp_path);accepted_cache(op,cache)
    hint=op.queue[0]['home_planning']
    hint.update(mean_first_plan_s=30,sample_count=10)
    if case=='cold':cache.offer=None
    elif case=='not_accepted':hint['config_sha256']=None
    elif case=='queue_changed':op.queue[0]['session_id']='different'
    elif case=='empty_queue':op.queue=[]
    elif case=='ineligible':hint.pop('profile')
    elif case=='configuration_changed':op.home=(.1,)*12
    elif case=='scene_changed':cache.offer['scene']['config_sha256']='different'
    elif case=='stale':cache.offer['scene']['captured_at']=time.time()-121
    elif case=='future':cache.offer['scene']['captured_at']=time.time()+10
    elif case=='invalidated':cache.invalidate()
    elif case=='api_unsupported':op.api_home_scene_supported=False
    assert cache.duration() is None
    assert cache.timing['using_cache'] is False


def test_pacing_only_slows_safe_trajectory_and_preserves_endpoint():
    normal=plan_joint_trajectory(((0.,)*12,(.1,)*12))
    paced=plan_joint_trajectory(((0.,)*12,(.1,)*12),minimum_duration_s=10)
    assert paced.duration_s>=10 and paced.duration_s>=normal.duration_s
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


@pytest.mark.parametrize('diagnostics_supported', [False, True])
def test_capture_supports_packaged_and_custom_status_signatures(tmp_path, diagnostics_supported):
    from YAM_control.home_scene import install
    calls = []
    class Packaged(Operator):
        def status(self):
            calls.append(None)
            return Operator.status(self)
    class Custom(Operator):
        def status(self, *, include_diagnostics=True):
            calls.append(include_diagnostics)
            return Operator.status(self)
    parent = Custom if diagnostics_supported else Packaged
    hardware = NS(PhysicalOperator=parent, base=NS(OperatorSimulator=parent))
    install(hardware, dict(robot_id='yam-1', cameras={'left':10,'top':16,'right':4}), tmp_path/'scene.json')
    op = hardware.PhysicalOperator()
    with patch('YAM_control.home_scene.capture_frame', side_effect=lambda name,device:source(name,device)):
        op.home_scene.capture(threading.Event())
    assert op.home_scene.scene is not None and op.home_scene.path.exists()
    assert calls == ([False, False] if diagnostics_supported else [None, None])


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


def stopped_operator(tmp_path):
    # Exercise the production stop handler with the real queue state machine,
    # without importing the hardware server or opening any motor connection.
    import ast
    from types import MethodType
    path=Path(__file__).resolve().parents[1]/'src/blupe_controller/runtime/scripts/yam_operator_hardware_web.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='PhysicalOperator')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='api_handle_stop')
    scope={'threading':threading};exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
    op=lifecycle(tmp_path);op.api_session_id='session';op.api_lease_id='lease';op.api_authorized=True
    op.api_handle_stop=MethodType(scope['api_handle_stop'],op)
    def hold(reason):
        op.calls.append(('hold',reason));op.api_session_id=None;op.api_lease_id=None;op.api_authorized=False
    op._hold_physical=hold
    op._finish_training_recording=lambda reason:op.calls.append(('recording_end',reason))
    op._clear_api_lease_locked=lambda:hold('clear_faulted_lease')
    return op


@pytest.mark.parametrize('reason',['user_requested','policy_complete'])
@pytest.mark.parametrize('auto_queue_enabled',[False,True])
@pytest.mark.parametrize('waiting',[False,True])
def test_normal_stop_captures_before_parking_even_when_queue_paused(tmp_path,reason,auto_queue_enabled,waiting):
    op=stopped_operator(tmp_path);op.auto_queue.enabled=auto_queue_enabled
    if not waiting:op.queue=[]
    op.api_handle_stop(dict(session_id='session',lease_id='lease',reason=reason))
    assert op.api_lease_id is None and not op.api_authorized
    op.auto_queue.tick()
    assert op.calls==[('hold',reason),'home','capture','park']
    assert op.auto_queue.parked


@pytest.mark.parametrize('reason',['operator_stop','browser_stop_or_forget','browser_disconnect','session_timeout','api_error'])
def test_abort_reasons_do_not_add_home_capture(tmp_path,reason):
    op=stopped_operator(tmp_path);op.auto_queue.enabled=True;op.queue=[]
    op.api_handle_stop(dict(session_id='session',lease_id='lease',reason=reason))
    op.auto_queue.tick()
    assert op.calls==[('hold',reason),'park']


@pytest.mark.parametrize('case',['stale_session','stale_lease','fault','cancelled_cleanup'])
def test_normal_stop_never_bypasses_lease_fault_or_cancellation(tmp_path,case):
    op=stopped_operator(tmp_path);payload=dict(session_id='session',lease_id='lease',reason='user_requested')
    if case=='stale_session':payload['session_id']='old'
    elif case=='stale_lease':payload['lease_id']='old'
    elif case=='fault':op.mode='FAULT';op._physical.fault='motor fault'
    op.api_handle_stop(payload)
    if case=='cancelled_cleanup':op.stop_event.set()
    op.auto_queue.tick()
    assert 'home' not in op.calls and 'capture' not in op.calls
