"""MakerArm SDK adapter. CAN and watchdogs live in a separate process.

Pinned SDK: b30d05a23d72e8c155a8e00f807aba8e8c705f68. Uses the SDK's
private protocol, not the LeRobot branch's MIT wire protocol. No zero writes.
"""
import math
import multiprocessing as mp
import threading
import time

JOINTS = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_yaw','wrist_roll')


def validate_profile(config):
    s = config.get('settings', {})
    if s.get('backend', 'socketcan') not in ('socketcan','slcan'):
        raise ValueError('MakerArm backend must be socketcan or slcan')
    if not isinstance(s.get('channel'), str) or not s['channel']:
        raise ValueError('Provide MakerArm CAN channel/serial port')
    endpoints = s.get('gripper_endpoints_rad', [])
    if len(endpoints) != 2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in endpoints) or endpoints[0] == endpoints[1]:
        raise ValueError('Provide measured gripper_endpoints_rad [closed, open] in SDK coordinates')
    speed = s.get('max_velocity', .1)
    if type(speed) not in (int,float) or not math.isfinite(speed) or not 0 < speed <= .1:
        raise ValueError('Initial MakerArm max_velocity must be in (0, 0.1] rad/s')
    for key, default in [('operator_port',8096),('camera_port',8089)]:
        if type(s.get(key,default)) is not int or not 1024 <= s.get(key,default) <= 65535:
            raise ValueError('Invalid local port')
    if s.get('operator_port',8096) == s.get('camera_port',8089):
        raise ValueError('Camera and operator ports must differ')
    cameras = config.get('cameras', {})
    if not cameras or any(not n.isidentifier() or type(v) is not int or v < 0 for n,v in cameras.items()) or len(set(cameras.values())) != len(cameras):
        raise ValueError('Provide distinct named camera indices')
    return config


def _worker(pipe, settings):
    from importlib.resources import files
    from maker_arm.arm import Arm, ArmState
    path = str(files('maker_arm.profiles').joinpath('maker_arm_v1.yaml'))
    backend = settings.get('backend', 'socketcan')
    options = {'channel':settings['channel']} if backend == 'socketcan' else {'port':settings['channel']}
    arm = Arm.from_yaml(path, backend=backend, **options)
    arm.config.max_velocity = settings.get('max_velocity', .1)
    if backend == 'slcan':
        arm.config.control_rate_hz = 25
    mode = 'readonly'
    last = time.monotonic()
    try:
        arm.connect()
        closed, opened = settings['gripper_endpoints_rad']
        limits = [[math.degrees(j.lo),math.degrees(j.hi)] for j in arm.config.joints[:6]]
        if not all(arm.config.joints[6].lo <= g <= arm.config.joints[6].hi for g in (closed,opened)):
            raise ValueError('Gripper endpoints exceed SDK profile limits')
        pipe.send({'limits':limits})
        while True:
            if time.monotonic()-last > 1 and mode == 'active':
                arm.hold_current_position()
                mode = 'hold'
            if not pipe.poll(.05):
                continue
            command, args = pipe.recv()
            last = time.monotonic()
            try:
                if command == 'close':
                    if arm.state is ArmState.ENABLED:
                        arm.hold_current_position()
                        pipe.send({'holding':True})
                        mode = 'hold'
                        continue
                    arm.disconnect()
                    pipe.send({})
                    return
                if command == 'release':
                    arm.disconnect()
                    pipe.send({})
                    return
                if command == 'enable':
                    if arm.state is not ArmState.ENABLED:
                        arm.enable()
                    mode = 'active'
                elif command == 'hold':
                    arm.hold_current_position()
                    mode = 'hold'
                elif command == 'move':
                    if mode != 'active':
                        raise ValueError('Explicit enable required')
                    joints, grip = args
                    if not arm.set_joint_targets([*map(math.radians,joints), closed+grip*(opened-closed)]):
                        raise ValueError('SDK rejected joint targets')
                if arm.state is not ArmState.ENABLED:
                    if not all(arm.refresh(wait=True)):
                        raise ValueError('Missing fresh MakerArm feedback')
                if arm.state is ArmState.FAULT or any(m.feedback_age > arm.config.feedback_timeout for m in arm.motors):
                    raise ValueError(arm.fault_reason or 'Stale MakerArm feedback')
                q = arm.get_joint_positions()
                if len(q) != 7 or not all(math.isfinite(v) for v in q):
                    raise ValueError('Nonfinite MakerArm feedback')
                pipe.send(dict(mode=mode, error='', joint_names=list(JOINTS), joints_deg=list(map(math.degrees,q[:6])), gripper=(q[6]-closed)/(opened-closed)))
            except Exception as exc:
                arm.hold_current_position()
                mode = 'fault'
                pipe.send({'error':str(exc)})
    except (EOFError, BrokenPipeError, ConnectionResetError):
        if arm.state is ArmState.ENABLED:
            arm.hold_current_position()
            while True:
                time.sleep(1)
    except Exception as exc:
        pipe.send({'error':str(exc)})
    finally:
        arm.disconnect()


class MakerArmDriver:
    joint_names = JOINTS
    joint_tolerance_deg = 1.0
    gripper_action_scale = 1
    def __init__(self, config):
        self.config = validate_profile(config)
        self.lock = threading.RLock()
        self.process = None

    def connect(self):
        ctx = mp.get_context('spawn')
        self.pipe, child = ctx.Pipe()
        self.process = ctx.Process(target=_worker, args=(child,self.config['settings']), daemon=False)
        self.process.start()
        child.close()
        if not self.pipe.poll(15):
            raise ValueError('MakerArm SDK connection timed out')
        reply = self.pipe.recv()
        if reply.get('error'):
            raise ValueError(reply['error'])
        self.limits = reply['limits']
        return self

    def _request(self, command, args=None):
        with self.lock:
            self.pipe.send((command,args))
            if not self.pipe.poll(15 if command == 'enable' else 3):
                self.pipe.close()
                raise ValueError('MakerArm worker timed out; connection closed, worker holds the arm')
            reply = self.pipe.recv()
            if reply.get('error'):
                raise ValueError(reply['error'])
            return reply

    def state(self, wait=0):
        return self._request('state')

    def enable(self):
        return self._request('enable')

    def hold(self):
        return self._request('hold')

    def _validate_target(self, joints, grip):
        if not isinstance(joints, (list, tuple)) or len(joints) != 6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in [*joints,grip]):
            raise ValueError('MakerArm requires six finite joint angles and gripper')
        if not 0 <= grip <= 1 or any(not lo <= v <= hi for v,(lo,hi) in zip(joints,self.limits)):
            raise ValueError('MakerArm target exceeds SDK profile limits')

    def _action(self, joints, grip):
        return {**{n+'.pos':v for n,v in zip(JOINTS,joints)},'gripper.pos':grip}

    def move(self, joints_deg, gripper):
        self._validate_target(joints_deg,gripper)
        reply = self._request('move',(joints_deg,gripper))
        return {**reply,'sent_action':self._action(joints_deg,gripper)}

    def close(self):
        if self.process and self.process.is_alive():
            result = self._request('close')
            if result.get('holding'):
                # Match the SDK's intentional-release workflow. Headless services must
                # not silently drop torque on shutdown.
                while input('Support MakerArm and payload. Type RELEASE to release torque: ') != 'RELEASE':
                    pass
                self._request('release')
            self.pipe.close()
            self.process.join(timeout=3)
