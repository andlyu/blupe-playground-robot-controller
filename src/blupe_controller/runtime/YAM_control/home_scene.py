"""Home-scene evidence and startup pacing. Motor ownership stays in AutoQueue."""
from concurrent.futures import ThreadPoolExecutor
import base64
import hashlib
import inspect
import json
import math
import logging
from pathlib import Path
import time
import uuid
from urllib.request import urlopen

from YAM_control.training_recorder import atomic_json


CACHED_HOME_DURATION_S = 10.0
MAX_CAPTURE_ATTEMPTS = 3


class CaptureFreshnessError(ValueError):
    """Retryable camera timing rejection; pose and safety failures are terminal."""


def capture_frame(name, device):
    # Home planning compares model-camera pixels. Recording clients may resize
    # frames and require private connection state, so read the raw relay here.
    with urlopen(f'http://127.0.0.1:8089/{device}/snapshot.jpg', timeout=.5) as response:
        jpeg = response.read(4 * 1024 * 1024 + 1)
        stamp = float(response.headers['X-Capture-Monotonic'])
        sequence = int(response.headers['X-Frame-Sequence'])
    if len(jpeg) > 4 * 1024 * 1024 or not jpeg.startswith(b'\xff\xd8') or not math.isfinite(stamp):
        raise ValueError('Invalid timestamped Home JPEG')
    return name, dict(jpeg=jpeg, monotonic=stamp, sequence=sequence)


class HomeScene:
    def __init__(self, operator, config, path=None):
        self.op = operator
        self.config = config
        self.path = Path(path) if path else None
        # Saved disk artifacts are evidence, not a reusable authority after restart.
        self.boot_id = uuid.uuid4().hex
        self.scene = None
        self.offer = None
        self.timing = {}
        self.settling_samples = []
        self.settling_config = None
        self.last_home = {}
        self.capture_error = None

    def fingerprint(self):
        config = self.config
        calibration = config.get('yam_calibration_file')
        fields = dict(robot_id=config['robot_id'], cameras=config['cameras'],
                      home=list(self.op.home), safety=repr(self.op._safety_config),
                      calibration=hashlib.sha256(Path(calibration).read_bytes()).hexdigest() if calibration else None)
        # Cover runtime geometry and motion code, not just a friendly profile name.
        root = Path(__file__).resolve().parents[1]
        files = {Path(__file__), root/'scripts/yam_operator_hardware_web.py',
                 root/'YAM_control/hardware_safety.py'}
        # Custom launchers retain their installed planner and camera owner.
        # Fingerprint their real source as well as the packaged runtime.
        for cls in type(self.op).__mro__:
            try:
                source = inspect.getsourcefile(cls)
            except TypeError:
                source = None
            if source:
                files.add(Path(source))
        for name in ('YAM_control/ruckig_waypath.py', 'YAM_control/camera_relay.py',
                     'scripts/publish_yam_cameras.py'):
            if (root/name).is_file():
                files.add(root/name)
        files.update(Path(p) for p in config.get('home_scene_runtime_paths', []))
        fields['runtime'] = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
        return hashlib.sha256(json.dumps(fields, sort_keys=True).encode()).hexdigest()

    def invalidate(self):
        self.scene = None
        self.offer = None

    def _measured(self):
        status = self.op.status(**getattr(self.op, '_home_scene_status_kwargs', {}))
        if (not status.get('homed') or not status.get('settled')
                or not status.get('feedback_available', False)
                or not status.get('safety', {}).get('controller_ok')
                or self.op.api_lease_id is not None or self.op._hardware_pending is not None):
            raise ValueError('Home is not confirmed idle')
        snap = self.op._physical.snapshot()
        joints = list(snap.left.joints_rad+snap.right.joints_rad)
        grips = [snap.left.gripper,snap.right.gripper]
        if (len(joints) != 12 or any(type(v) not in (int,float) or not math.isfinite(v) for v in joints+grips)
                or any(abs(g-1.) > .02 for g in grips)):
            raise ValueError('Home grippers or joints are not verified')
        return joints,grips

    def capture(self, cancel):
        self.capture_error = None
        self.invalidate()
        try:
            for attempt in range(MAX_CAPTURE_ATTEMPTS):
                if cancel.is_set():
                    return
                try:
                    return self._capture(cancel)
                except CaptureFreshnessError as exc:
                    if attempt == MAX_CAPTURE_ATTEMPTS-1:
                        raise
                    logging.getLogger(__name__).info(
                        '[home-scene] recapturing after freshness rejection (%s/%s): %s',
                        attempt+1, MAX_CAPTURE_ATTEMPTS, exc)
                    if cancel.wait(.05):
                        return
        except Exception as exc:
            self.capture_error = f'{type(exc).__name__}: {exc}'[:200]
            raise

    def _capture(self, cancel):
        self.invalidate()
        start = time.monotonic()
        before_q,before_g = self._measured()
        if cancel.is_set():
            return
        with ThreadPoolExecutor(max_workers=3) as pool:
            for _ in range(4):
                if cancel.wait(.05):
                    return
                jobs = [pool.submit(capture_frame, name, str(device))
                        for name,device in self.config['cameras'].items()]
                frames = dict(job.result() for job in jobs)
                if all(f['monotonic'] >= start for f in frames.values()):
                    break
        q,g = self._measured()
        end, wall = time.monotonic(),time.time()
        if cancel.is_set():
            return
        if set(frames) != {'left','top','right'}:
            raise ValueError('Home capture camera roles are incomplete')
        joint_drift = max(abs(a-b) for a,b in zip(q,before_q))
        gripper_drift = max(abs(a-b) for a,b in zip(g,before_g))
        # Pose changes must fail even if the same frame set is also too old.
        if joint_drift > math.radians(.25) or gripper_drift > .01:
            raise ValueError(f'Home capture pose changed: joints={math.degrees(joint_drift):.3f}deg '
                             f'grippers={gripper_drift:.4f}')
        stamps = [f['monotonic'] for f in frames.values()]
        if (max(stamps)-min(stamps) > .15 or end-start > 1.0
                or any(not start <= t <= end or end-t > .25 for t in stamps)):
            raise CaptureFreshnessError(
                f'Home capture freshness failed: bracket={end-start:.3f}s '
                f'skew={max(stamps)-min(stamps):.3f}s age={end-min(stamps):.3f}s '
                f'outside_bracket={any(not start <= t <= end for t in stamps)}')
        scene = dict(version=1, scene_id=uuid.uuid4().hex, robot_id=self.config['robot_id'],
            config_sha256=self.fingerprint(), boot_id=self.boot_id, captured_at=wall,
            camera_map=dict(self.config['cameras']), encoder_bracket_s=end-start,
            observation=dict(jetson_id=self.config['robot_id'], source='hardware', homed=True, settled=True,
                observed_at=wall, left_joints_deg=[math.degrees(v) for v in q[:6]],
                right_joints_deg=[math.degrees(v) for v in q[6:]], left_gripper=g[0],right_gripper=g[1]),
            frames={name:dict(jpeg_base64=base64.b64encode(f['jpeg']).decode(),
                sha256=hashlib.sha256(f['jpeg']).hexdigest(), captured_at=wall-(end-f['monotonic']),
                sequence=f['sequence']) for name,f in frames.items()})
        if self.path:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            atomic_json(self.path,scene)
            self.path.chmod(0o600)
        if not cancel.is_set():
            self.scene = scene

    def start(self):
        self.offer = None
        self.timing = {'started_monotonic':time.monotonic()}
        if not getattr(self.op, 'api_home_scene_supported', False):
            return
        queue = self.op.queue_snapshot().get('entries', [])
        if not queue or not (queue[0].get('home_planning') or {}).get('profile'):
            return
        scene = self.scene
        if not scene or not 0 <= time.time()-scene['captured_at'] <= 120 or scene['config_sha256'] != self.fingerprint():
            self.scene = None
            return
        self.offer = {'session_id':queue[0]['session_id'], 'scene':scene}
        self.op.api_client.send(dict(schema_version=1,type='home_scene',phase='offer',**self.offer))
        # A scene can be offered once. An aborted startup cannot silently reuse it.
        self.scene = None

    def duration(self):
        self._settling_configuration()
        calibration_s = time.monotonic()-self.timing['started_monotonic']
        queue = self.op.queue_snapshot().get('entries', [])
        hint = (queue[0].get('home_planning') or {}) if queue else {}
        offered_scene = (self.offer or {}).get('scene') or {}
        captured_at = offered_scene.get('captured_at')
        using_cache = bool(
            getattr(self.op, 'api_home_scene_supported', False)
            and self.offer and queue and queue[0]['session_id'] == self.offer['session_id']
            and hint.get('profile')
            and hint.get('config_sha256') == offered_scene.get('config_sha256') == self.settling_config
            and type(captured_at) in (int, float) and 0 <= time.time()-captured_at <= 120)
        # Pace the Home motion itself, independently of calibration or inference
        # history. The planner still keeps any longer duration its limits require.
        duration = CACHED_HOME_DURATION_S if using_cache else None
        settling = sum(self.settling_samples)/len(self.settling_samples) if self.settling_samples else 1.0
        self.timing.update(calibration_s=calibration_s, using_cache=using_cache,
                           mean_first_plan_s=hint.get('mean_first_plan_s'),
                           sample_count=hint.get('sample_count',0), target_home_s=duration,
                           settling_allowance_s=settling, settling_basis='measured' if self.settling_samples else 'provisional default')
        return duration

    def _settling_configuration(self):
        config = self.fingerprint()
        if config != self.settling_config:
            self.settling_samples.clear()
            self.settling_config = config

    def home_finished(self, elapsed_s, commanded_s):
        self._settling_configuration()
        self.last_home = dict(actual_home_s=elapsed_s, commanded_home_s=commanded_s)
        # Includes feedback settling and controller overhead, measured on this rig.
        self.settling_samples.append(max(0.,elapsed_s-commanded_s))
        del self.settling_samples[:-20]

    def ready(self, payload):
        self.timing["home_ready_monotonic"] = time.monotonic()
        if not getattr(self.op, 'api_home_scene_supported', False):
            return
        scene = (self.offer or {}).get('scene')
        matched = (scene and self.offer['session_id'] == payload['session_id']
                   and scene['config_sha256'] == self.fingerprint())
        self.op.api_client.send(dict(schema_version=1,type='home_scene',phase='ready',
            **{k:payload[k] for k in ('session_id','episode_id','lease_id')},
            scene_id=scene['scene_id'] if matched else None,
            config_sha256=self.fingerprint(),boot_id=self.boot_id))
        self.offer = None


def install(hardware, config, path=None):
    parent = hardware.PhysicalOperator
    if getattr(parent, '_home_scene_installed', False):
        return
    class HomeSceneOperator(parent):
        _home_scene_installed = True
        # Custom stations can omit expensive diagnostics; the packaged
        # controller exposes status() without that optional keyword.
        _home_scene_status_kwargs = ({'include_diagnostics': False}
            if 'include_diagnostics' in inspect.signature(parent.status).parameters else {})
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.home_scene = HomeScene(self,config,path)
        def status(self, *args, **kwargs):
            result = super().status(*args, **kwargs)
            scene = getattr(self, 'home_scene', None)
            result['home_scene'] = dict(version=1,
                api_supported=bool(getattr(self, 'api_home_scene_supported', False)),
                cached=bool(scene and scene.scene), offered=bool(scene and scene.offer),
                capture_error=scene.capture_error if scene else None)
            return result
        def _move_home(self, *, cancel_event=None, target_duration_s=None):
            # Older custom subclasses forward only cancel_event. The base
            # planner reads this scoped hint without bypassing that subclass.
            self._home_scene_target_duration = target_duration_s
            try:
                return super()._move_home(cancel_event=cancel_event)
            finally:
                self._home_scene_target_duration = None
        def api_prepare_session(self,payload):
            result = super().api_prepare_session(payload)
            if result is not None:
                try:
                    self.home_scene.ready(payload)
                except Exception as exc:
                    logging.getLogger(__name__).info("[home-scene] ready unavailable: %s", type(exc).__name__)
            return result
        def action(self,action,*args,**kwargs):
            if hasattr(self,'home_scene'):
                self.home_scene.invalidate()
            return super().action(action,*args,**kwargs)
        def _hold_physical(self,reason):
            if hasattr(self,'home_scene'):
                self.home_scene.invalidate()
            return super()._hold_physical(reason)
        def _stop_physical(self,reason):
            if reason != 'operator_park_zero' and hasattr(self,'home_scene'):
                self.home_scene.invalidate()
            return super()._stop_physical(reason)
        def api_connection_changed(self,connected,error=None):
            if not connected and hasattr(self,'home_scene'):
                self.home_scene.invalidate()
            return super().api_connection_changed(connected,error)
    hardware.PhysicalOperator = HomeSceneOperator
    if hardware.base.OperatorSimulator is parent:
        hardware.base.OperatorSimulator = HomeSceneOperator
