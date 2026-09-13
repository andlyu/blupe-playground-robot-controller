"""SO101 profile and subprocess driver. Importing this module never opens hardware."""
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time

NAMES = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')


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


def build():
    source = Path(__file__).with_name('native') / 'so101.cpp'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
    cache = Path.home() / '.cache/blupe-controller/native'
    cache.mkdir(parents=True, exist_ok=True, mode=0o700)
    binary = cache / ('so101-' + digest)
    if not binary.exists():
        compiler = shutil.which('clang++') or shutil.which('g++')
        if not compiler:
            raise ValueError('Install a C++17 compiler (on macOS: xcode-select --install)')
        with tempfile.TemporaryDirectory(dir=cache) as temp:
            built = Path(temp) / 'so101'
            subprocess.run([compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-pthread', str(source), '-o', str(built)], check=True)
            built.replace(binary)
    return binary


class SO101Driver:
    """Native ownership of serial I/O; angles in degrees, gripper in [0, 1].

    connect() is read-only. enable() is explicit. hold()/close() retain the last
    bounded target; they do not release torque or certify an emergency stop.
    """
    joint_names = NAMES[:5]

    def __init__(self, config):
        validate_profile(config)
        self.config = config
        self.cal = calibration(config['settings']['calibration_file'])
        self.proc = None
        self.condition = threading.Condition()
        self.write_lock = threading.Lock()
        self.latest = None
        self.received = 0
        self.seq = 0
        self.done = threading.Event()

    def connect(self):
        if self.proc:
            raise ValueError('Driver already connected')
        binary = build()
        self.temp = tempfile.TemporaryDirectory(prefix='blupe-so101-')
        profile = Path(self.temp.name) / 'profile'
        profile.write_text('\n'.join(' '.join(str(self.cal[n][k]) for k in
            ('id', 'range_min', 'range_max', 'homing_offset')) for n in NAMES))
        self.proc = subprocess.Popen([str(binary), self.config['settings']['serial_port'], str(profile)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        os.set_blocking(self.proc.stdin.fileno(), False)
        threading.Thread(target=self._read, daemon=True).start()
        try:
            initial = self.state(wait=3)
            if initial['mode'] == 'fault':
                raise ValueError(initial['error'])
        except Exception:
            self.close()
            raise
        threading.Thread(target=self._heartbeat, daemon=True).start()
        return self

    def _read(self):
        for line in self.proc.stdout:
            try:
                state = json.loads(line)
            except ValueError:
                continue
            with self.condition:
                self.latest = state
                self.received = time.monotonic()
                self.condition.notify_all()
        with self.condition:
            self.condition.notify_all()

    def _heartbeat(self):
        while not self.done.wait(0.1):
            try:
                self._send('ping', wait=False)
            except (OSError, ValueError):
                return

    def state(self, wait=0):
        with self.condition:
            self.condition.wait_for(lambda: self.latest is not None or self.proc.poll() is not None, timeout=wait)
            if self.proc.poll() is not None:
                raise ValueError('Native driver exited: ' + self.proc.stderr.read().strip())
            if self.latest is None or time.monotonic() - self.received > 0.5:
                raise ValueError('No fresh native feedback')
            result = dict(self.latest)
        raw = result['raw']
        result['joint_names'] = list(self.joint_names)
        result['joints_deg'] = [(raw[i] - (self.cal[n]['range_min'] + self.cal[n]['range_max']) / 2) * 360 / 4095
                                for i, n in enumerate(NAMES[:5])]
        g = self.cal['gripper']
        fraction = (raw[5] - g['range_min']) / (g['range_max'] - g['range_min'])
        result['gripper'] = 1 - fraction if g['drive_mode'] else fraction
        return result

    def _send(self, text, wait=True):
        with self.write_lock:
            if self.proc is None or self.proc.poll() is not None:
                raise ValueError('Native driver is not running')
            self.seq += 1
            seq = self.seq
            message = f'{seq} {int(time.time() * 1000) + 500} {text}\n'.encode()
            if len(message) > 512 or os.write(self.proc.stdin.fileno(), message) != len(message):
                raise ValueError('Native command pipe unavailable')
        if wait:
            with self.condition:
                if not self.condition.wait_for(lambda: self.latest and self.latest['seq'] >= seq, timeout=1):
                    raise ValueError('Native command timed out')
            state = self.state()
            if state['mode'] == 'fault':
                raise ValueError(state['error'])
            return state

    def enable(self):
        return self._send('enable')

    def hold(self):
        return self._send('hold')

    def move(self, joints_deg, gripper):
        if len(joints_deg) != 5:
            raise ValueError('SO101 requires five joint angles')
        values = [*joints_deg, gripper]
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError('Targets must be finite numbers')
        if not 0 <= gripper <= 1:
            raise ValueError('Gripper must be between zero and one')
        raw = []
        for angle, name in zip(joints_deg, NAMES[:5]):
            c = self.cal[name]
            value = angle * 4095 / 360 + (c['range_min'] + c['range_max']) / 2
            if not c['range_min'] <= value <= c['range_max']:
                raise ValueError(f'{name}: target outside calibrated limits')
            raw.append(int(value))
        c = self.cal['gripper']
        fraction = 1 - gripper if c['drive_mode'] else gripper
        raw.append(int(c['range_min'] + fraction * (c['range_max'] - c['range_min'])))
        return self._send('target ' + ' '.join(map(str, raw)))

    def close(self):
        self.done.set()
        if self.proc:
            try:
                self._send('close', wait=False)
                self.proc.wait(timeout=1)
            except (OSError, ValueError, subprocess.TimeoutExpired):
                self.proc.terminate()
                try:
                    self.proc.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait()
            for stream in (self.proc.stdin, self.proc.stdout, self.proc.stderr):
                stream.close()
            self.temp.cleanup()


def console(config, probe=False):
    driver = SO101Driver(config).connect()
    try:
        print(json.dumps(driver.state(), indent=2))
        if probe:
            return 0
        print('Read-only. Commands: state, enable, move <five degrees> <gripper 0..1>, hold, quit.')
        while True:
            try:
                parts = input('so101> ').split()
                if not parts:
                    continue
                if parts == ['quit']:
                    break
                if parts == ['state']:
                    result = driver.state()
                elif parts == ['enable']:
                    result = driver.enable()
                elif parts == ['hold']:
                    result = driver.hold()
                elif parts[0] == 'move' and len(parts) == 7:
                    result = driver.move(list(map(float, parts[1:6])), float(parts[6]))
                else:
                    raise ValueError('Unknown command')
                print(json.dumps(result))
            except ValueError as error:
                print(error)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        driver.close()
