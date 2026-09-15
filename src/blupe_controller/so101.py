"""Thin BluPe adapter for LeRobot's Python SO101 implementation.

LeRobot owns protocol handling, normalization and relative-target clipping.
No native worker, interpolation thread or independent watchdog is used.
"""
from functools import wraps
import json
import math
from pathlib import Path
import threading

NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


POSITION_READ_RETRIES = 3


def retry_position_reads(bus):
    """Use the SDK's packet retries for feedback and pre-command safety reads.

    This is per robot bus; writes and other registers keep their original policy.
    Failed packets never supply cached positions or re-send a motion command.
    """
    original = bus.sync_read

    @wraps(original)
    def sync_read(data_name, *args, **kwargs):
        if data_name == 'Present_Position':
            kwargs['num_retry'] = POSITION_READ_RETRIES
        return original(data_name, *args, **kwargs)

    bus.sync_read = sync_read


def calibration(path):
    data = json.loads(Path(path).read_text())
    if set(data) != set(NAMES):
        raise ValueError('SO101 requires five named joints and one gripper')
    ids = set()
    for name in NAMES:
        row = data[name]
        for key in ('id', 'drive_mode', 'homing_offset', 'range_min', 'range_max'):
            if type(row.get(key)) is not int:
                raise ValueError(f'{name}: invalid {key}')
        if not (1 <= row['id'] <= 252 and row['id'] not in ids and
                0 <= row['range_min'] < row['range_max'] <= 4095 and
                abs(row['homing_offset']) <= 2047 and row['drive_mode'] in (0, 1)):
            raise ValueError(f'{name}: invalid calibration')
        ids.add(row['id'])
    return data


def validate_profile(config):
    settings = config.get('settings', {})
    port = settings.get('serial_port', '')
    if not isinstance(port, str) or not port.startswith('/dev/'):
        raise ValueError('SO101 requires an absolute /dev/ serial_port')
    calibration(settings.get('calibration_file', ''))
    cameras = config.get('cameras', {})
    if not cameras or len(cameras) > 32 or any(not isinstance(k, str) or not k.isidentifier() or
            type(v) is not int or v < 0 for k, v in cameras.items()) or len(set(cameras.values())) != len(cameras):
        raise ValueError('Provide distinct camera indices with named roles')
    port = settings.get('camera_port', 8089)
    if type(port) is not int or not 1024 <= port <= 65535:
        raise ValueError('Invalid camera port')
    for key in ('operator_port',):
        value = settings.get(key, 8096)
        if type(value) is not int or not 1024 <= value <= 65535 or value == port:
            raise ValueError('Operator and camera ports must be valid and distinct')
    return config



def relative_limit(value):
    if value is None:
        return None
    def number(v):
        if type(v) not in (int,float) or not math.isfinite(v) or v<=0:
            raise ValueError('max_relative_target must contain positive finite numbers')
        return float(v)
    if isinstance(value,dict):
        if set(value)!=set(NAMES):
            raise ValueError('max_relative_target must name all six motors')
        return {k:number(v) for k,v in value.items()}
    return number(value)


def make_robot(config):
    from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
    from lerobot.robots.so_follower.so_follower import SOFollower
    settings = config['settings']
    calibration_path = Path(settings['calibration_file'])
    # Camera capture runs in the existing relay, using the same source config.
    robot_config = SO101FollowerConfig(
        port=settings['serial_port'], id=calibration_path.stem,
        calibration_dir=calibration_path.parent, use_degrees=True, cameras={},
        max_relative_target=relative_limit(settings.get('max_relative_target')),
        disable_torque_on_disconnect=settings.get('disable_torque_on_disconnect', False))
    return SOFollower(robot_config)


class SO101Driver:
    joint_names = NAMES[:5]

    def __init__(self, config):
        from .lerobot_config import resolve
        self.config = validate_profile(resolve(config))
        self.cal = calibration(self.config['settings']['calibration_file'])
        self.robot = None
        self.mode = 'readonly'
        self.error = ''
        self.enabled_once = False
        self.lock = threading.RLock()

    def connect(self):
        if self.robot is not None:
            raise ValueError('Already connected')
        self.robot = make_robot(self.config)
        retry_position_reads(self.robot.bus)
        try:
            # SOFollower.connect() also configures registers and toggles torque.
            # Use its bus's read-only connection for an already configured arm.
            self.robot.bus.connect()
            if not self.robot.is_calibrated:
                raise ValueError('LeRobot calibration does not match the servos')
            for name in NAMES:
                if self.robot.bus.read('Operating_Mode', name, normalize=False) != 0:
                    raise ValueError('Servo is not in position mode')
            self.state()
            return self
        except Exception:
            self.close()
            raise

    def state(self, wait=0):
        with self.lock:
            if self.robot is None:
                raise ValueError('Not connected')
            try:
                observation = self.robot.get_observation()
                joints = [float(observation[name+'.pos']) for name in self.joint_names]
                gripper = float(observation['gripper.pos']) / 100
                self._validate_target(joints, gripper)
            except Exception as error:
                self.mode, self.error = 'fault', f'{type(error).__name__}: {error}'
                raise
            return {'mode':self.mode, 'error':self.error, 'joint_names':list(self.joint_names),
                    'joints_deg':joints, 'gripper':gripper}

    def _validate_target(self, joints_deg, gripper):
        if not isinstance(joints_deg, (list, tuple)) or len(joints_deg) != 5:
            raise ValueError('SO101 requires five joint angles')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in [*joints_deg, gripper]):
            raise ValueError('Targets must be finite numbers')
        if not 0 <= gripper <= 1:
            raise ValueError('Gripper must be between zero and one')
        for angle, name in zip(joints_deg, self.joint_names):
            c = self.cal[name]
            raw = angle * 4095 / 360 + (c['range_min'] + c['range_max']) / 2
            if not c['range_min'] <= raw <= c['range_max']:
                raise ValueError(f'{name}: target outside calibrated limits')

    @staticmethod
    def _action(joints, gripper):
        return {**{name+'.pos':v for name,v in zip(NAMES[:5], joints)}, 'gripper.pos':gripper*100}

    def enable(self):
        with self.lock:
            if self.mode == 'fault':
                raise ValueError('Restart after resolving the fault')
            state = self.state()
            # Preload the measured pose before enabling torque.
            self.robot.bus.sync_write('Goal_Position', {
                key.removesuffix('.pos'):value for key,value in self._action(state['joints_deg'],state['gripper']).items()})
            self.enabled_once = True
            try:
                self.robot.bus.enable_torque()
            except Exception as error:
                self.mode, self.error = 'fault', f'{type(error).__name__}: {error}'
                raise
            self.mode = 'active'
            return {**state, 'mode':self.mode}

    def disable(self):
        with self.lock:
            try:
                self.robot.bus.disable_torque()
                for name in NAMES:
                    if self.robot.bus.read('Torque_Enable', name, normalize=False) != 0:
                        raise ValueError(f'{name}: torque-off verification failed')
            except Exception as error:
                self.mode, self.error = 'fault', f'{type(error).__name__}: {error}'
                raise
            self.mode = 'readonly'
            return self.state()

    def move(self, joints_deg, gripper):
        with self.lock:
            self._validate_target(joints_deg, gripper)
            if self.mode != 'active':
                raise ValueError('Enable before moving')
            try:
                sent = self.robot.send_action(self._action(joints_deg, gripper))
                result = self.state()
                # LeRobot may clip via max_relative_target: report actual sent values.
                result['sent_action'] = sent
                return result
            except Exception as error:
                self.mode, self.error = 'fault', f'{type(error).__name__}: {error}'
                raise

    def hold(self):
        with self.lock:
            state = self.state()
            if self.enabled_once:
                self.robot.bus.sync_write('Goal_Position', {
                    key.removesuffix('.pos'):value for key,value in self._action(state['joints_deg'],state['gripper']).items()})
            if self.mode != 'fault':
                self.mode = 'hold' if self.enabled_once else 'readonly'
            return {**state, 'mode':self.mode}

    def close(self):
        with self.lock:
            if self.robot is not None:
                try:
                    if self.robot.bus.is_connected:
                        self.robot.bus.disconnect(disable_torque=self.enabled_once and
                            self.config['settings'].get('disable_torque_on_disconnect',False))
                finally:
                    self.robot = None


def console(config, probe=False):
    driver = SO101Driver(config).connect()
    try:
        print(json.dumps(driver.state(), indent=2))
        return 0
    finally:
        driver.close()
