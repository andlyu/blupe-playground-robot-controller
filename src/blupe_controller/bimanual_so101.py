"""Bimanual SO101 bring-up without calibration or motor writes.

Feetech protocol/register references: docs/refs/feetech/tables.py, SDK 1.0.0.
This monitor deliberately cannot enable torque, record calibrated poses or move.
Owner-provided autocalibration will be integrated separately.
"""
import threading
from pathlib import Path


def validate_profile(config):
    settings = config.get('settings', {})
    arms = settings.get('arms', {})
    if not isinstance(arms, dict) or len(arms) != 2:
        raise ValueError('Provide the two arm serial ports in settings.arms')
    if any(not name.isidentifier() or not isinstance(port,str) or not port.startswith('/dev/') for name,port in arms.items()):
        raise ValueError('Arm names and absolute /dev/ serial ports are required')
    if len({str(Path(p).resolve()) for p in arms.values()}) != 2:
        raise ValueError('Each arm must use its own serial port')
    if settings.get('cloud_enabled') and not settings.get('calibrations'):
        raise ValueError('Bimanual bring-up is read-only; cloud execution is not available yet')
    cameras = config.get('cameras', {})
    if not cameras or any(not n.isidentifier() or type(d) is not int or d < 0 for n,d in cameras.items()) or len(set(cameras.values())) != len(cameras):
        raise ValueError('Provide distinct named camera indices')
    ports = [settings.get('operator_port',8096),settings.get('camera_port',8089)]
    if any(type(p) is not int or not 1024 <= p <= 65535 for p in ports) or len(set(ports)) != 2:
        raise ValueError('Provide distinct valid operator and camera ports')
    return config


class BimanualSO101Monitor:
    joint_names = ()

    def __init__(self, config):
        self.config = validate_profile(config)
        self.ports = {}
        self.lock = threading.RLock()

    def connect(self):
        from scservo_sdk import PortHandler, PacketHandler
        self.packet = PacketHandler(0)
        if Path('/tmp/blupe-so101-calibrating').exists():
            return self
        try:
            for name,device in self.config['settings']['arms'].items():
                port = PortHandler(device)
                self.ports[name] = port
                if not port.openPort() or not port.setBaudRate(1_000_000):
                    raise ValueError(f'{name}: could not open serial port')
                for motor in range(1,7):
                    model,result,error = self.packet.ping(port,motor)
                    if result or error or model != 777:
                        raise ValueError(f'{name} motor {motor}: expected STS3215 feedback, got model={model}, result={result}, error={error}')
            self.state()
            return self
        except Exception:
            self.close()
            raise

    def state(self, wait=0):
        with self.lock:
            if Path('/tmp/blupe-so101-calibrating').exists():
                self.close()
                return dict(mode='readonly',error='',joint_names=[],joints_deg=[],gripper=0,calibration_ready=False,arms=[],setup_message='Calibration in progress in a separate task. Serial ports released by this monitor.')
            if not self.ports:
                self.connect()
            arms = []
            for name,port in self.ports.items():
                motors = []
                for motor in range(1,7):
                    position,result,error = self.packet.read2ByteTxRx(port,motor,56)
                    torque,tr,te = self.packet.read1ByteTxRx(port,motor,40)
                    if result or error or tr or te:
                        raise ValueError(f'{name} motor {motor}: feedback unavailable ({result}/{error}, {tr}/{te})')
                    motors.append(dict(id=motor,position_raw=position,torque_enabled=bool(torque)))
                arms.append(dict(name=name,port=self.config['settings']['arms'][name],motors=motors))
            return dict(mode='readonly',error='',joint_names=[],joints_deg=[],gripper=0,
                        calibration_ready=False,arms=arms,
                        setup_message='Both arms connected. Calibration deferred; monitoring only. No motor commands are enabled.')

    def _validate_target(self, *args):
        raise ValueError('Calibration is deferred. This bimanual controller is monitoring only.')

    enable = move = _validate_target

    def hold(self):
        return {**self.state(), 'message':'Monitoring only; this controller has not enabled or commanded either arm.'}

    def close(self):
        with self.lock:
            for port in self.ports.values():
                port.closePort()
            self.ports.clear()


class BimanualSO101Driver:
    """Two calibrated LeRobot drivers; both targets validated before either write."""
    joint_names = tuple(f'{side}_{name}' for side in ('left', 'right') for name in
                        ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'))
    joint_counts = (5, 5)

    def __init__(self, config):
        from .so101 import SO101Driver
        self.config = validate_profile(config)
        settings = config['settings']
        mapping = settings.get('arm_mapping', {})
        if set(mapping) != {'left', 'right'} or set(mapping.values()) != set(settings['arms']):
            raise ValueError('Map left and right to the two arm names')
        self.drivers = []
        self.lock = threading.RLock()
        for side in ('left', 'right'):
            arm = mapping[side]
            child = {**config, 'hardware':'so101', 'settings':{**settings,
                'serial_port':settings['arms'][arm], 'calibration_file':settings['calibrations'][arm]}}
            self.drivers.append(SO101Driver(child))

    def connect(self):
        try:
            for driver in self.drivers:
                driver.connect()
            return self
        except Exception:
            self.close()
            raise

    def state(self, wait=0):
        with self.lock:
            states = [d.state() for d in self.drivers]
            modes = [s['mode'] for s in states]
            mode = 'fault' if 'fault' in modes else ('active' if modes == ['active','active'] else 'readonly')
            return dict(mode=mode, error='; '.join(s['error'] for s in states if s['error']),
                        joint_names=list(self.joint_names),
                        joints_deg=states[0]['joints_deg']+states[1]['joints_deg'],
                        gripper=[s['gripper'] for s in states], calibration_ready=True)

    def _validate_target(self, joints, grippers):
        if not isinstance(joints, (list,tuple)) or len(joints)!=10 or not isinstance(grippers,(list,tuple)) or len(grippers)!=2:
            raise ValueError('Bimanual SO101 requires ten joint angles and two grippers')
        for i,d in enumerate(self.drivers):
            d._validate_target(joints[i*5:(i+1)*5], grippers[i])

    def _action(self, joints, grippers):
        return {f'{side}_{key}':value for i,side in enumerate(('left','right'))
                for key,value in self.drivers[i]._action(joints[i*5:(i+1)*5],grippers[i]).items()}

    def enable(self):
        with self.lock:
            self.state()
            try:
                for d in self.drivers: d.enable()
            except Exception:
                self.hold()
                raise
            return self.state()

    def move(self, joints, grippers):
        with self.lock:
            self._validate_target(joints,grippers)
            if self.state()['mode']!='active': raise ValueError('Enable both arms before moving')
            results=[]
            try:
                for i,d in enumerate(self.drivers): results.append(d.move(joints[i*5:(i+1)*5],grippers[i]))
            except Exception:
                self.hold()
                raise
            return {**self.state(), 'sent_action':{f'{side}_{k}':v for side,result in zip(('left','right'),results) for k,v in result['sent_action'].items()}}

    def hold(self):
        with self.lock:
            errors=[]
            for d in self.drivers:
                try: d.hold()
                except Exception as exc: errors.append(str(exc))
            if errors: raise ValueError('; '.join(errors))
            return self.state()

    def close(self):
        for d in self.drivers: d.close()
