"""Setup/diagnostics never import hardware drivers or command a robot."""
import argparse
import concurrent.futures
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
from urllib.parse import urlsplit

RUNTIME = Path(__file__).resolve().parent / 'runtime'
DEFAULT_CONFIG = Path.home() / '.config/blupe-controller/config.json'


def validate(config):
    if config.get('version') != 1 or config.get('hardware') not in ('yam', 'so101', 'makerarm', 'bimanual_so101'):
        raise ValueError('Supported profiles: yam, so101, makerarm, bimanual_so101.')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', config.get('robot_id', '')):
        raise ValueError('Invalid robot ID')
    if config['hardware'] == 'bimanual_so101':
        from .bimanual_so101 import validate_profile
        return validate_profile(config)
    origin = urlsplit(config.get('api', ''))
    if origin.scheme != 'https' or not origin.hostname or origin.username or origin.password or origin.path not in ('', '/') or origin.query or origin.fragment:
        raise ValueError('Use an HTTPS API origin with no credentials, path or query')
    if not Path(config.get('token_file', '')).is_absolute():
        raise ValueError('Use an absolute credential-file path')
    if config['hardware'] == 'makerarm':
        from .makerarm import validate_profile
        return validate_profile(config)
    if config['hardware'] == 'so101':
        from .so101 import validate_profile
        from .lerobot_config import resolve
        return validate_profile(resolve(config))
    cameras = config.get('cameras', {})
    if set(cameras) != {'left', 'top', 'right'} or any(type(v) is not int or v < 0 for v in cameras.values()) or len(set(cameras.values())) != 3:
        raise ValueError('YAM requires three distinct camera device numbers: left, top, right')
    return config


def load(path):
    return validate(json.loads(path.read_text()))


def setup(args):
    if args.hardware == 'so101' and args.lerobot_config:
        if args.serial_port or args.calibration or args.camera or args.cameras:
            raise ValueError('Use the LeRobot file alone for hardware configuration')
        cameras = {}
        settings = {'lerobot_config_file':str(args.lerobot_config.resolve()), 'camera_port':args.camera_port,
                    'operator_port':args.operator_port,'operator_hostname':args.operator_hostname}
    elif args.hardware == 'so101':
        if not args.serial_port or not args.calibration or not args.camera:
            raise ValueError('SO101 requires --serial-port, --calibration, and --camera ROLE=INDEX')
        try:
            cameras = dict((k, int(v)) for k,v in (value.split('=', 1) for value in args.camera))
        except ValueError:
            raise ValueError('Use --camera ROLE=INDEX')
        settings = {'serial_port': args.serial_port, 'calibration_file': str(args.calibration.resolve()), 'camera_port': args.camera_port, 'operator_port':args.operator_port, 'operator_hostname':args.operator_hostname}
    else:
        if not args.cameras:
            raise ValueError('YAM requires --cameras LEFT TOP RIGHT')
        cameras = dict(zip(('left', 'top', 'right'), args.cameras))
        settings = {}
    config = validate({'version': 1, 'hardware': args.hardware, 'robot_id': args.robot_id,
        'api': args.api.rstrip('/'), 'token_file': str(args.token_file.resolve()),
        'cameras': cameras, 'settings': settings})
    if settings.get('lerobot_config_file'):
        config['settings'] = settings
        config.pop('cameras', None)
    args.config.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    # Never silently overwrite a configured or running controller.
    fd = os.open(args.config, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as file:
        json.dump(config, file, indent=2)
        file.write('\n')
    print(f'Configuration saved: {args.config}. No motors or services started.')


def doctor(config):
    if config['hardware'] == 'makerarm':
        checks = {name: importlib.util.find_spec(module) is not None for name,module in [('maker_arm','maker_arm'),('opencv','cv2'),('websockets','websockets')]}
        for name, ok in checks.items():
            print(f'{name}: {"OK" if ok else "MISSING"}')
        return all(checks.values())
    if config['hardware'] == 'so101':
        checks = {'serial_device': Path(config['settings']['serial_port']).exists(),
                  'lerobot': importlib.util.find_spec('lerobot') is not None,
                  'opencv': importlib.util.find_spec('cv2') is not None}
        for name, ok in checks.items():
            print(f'{"OK" if ok else "MISSING"} {name}')
        print('No devices opened. Use probe for read-only servo feedback; cameras for capture.')
        return all(checks.values())
    checks = {'linux': sys.platform == 'linux', 'architecture': platform.machine() in {'aarch64','arm64','x86_64'},
        'bundled_model': (RUNTIME / 'assets/yam_bimanual/scene.xml').is_file(),
        'bundled_mesh': (RUNTIME / 'assets/yam/assets/base.stl').is_file()}
    for module in ('numpy','mujoco','ruckig','cv2','websockets','cryptography','i2rt'):
        checks[module] = importlib.util.find_spec(module) is not None
    token = Path(config['token_file'])
    checks['credential_file'] = token.is_file() and token.stat().st_mode & 0o077 == 0 and bool(token.read_text().strip())
    checks['can_interfaces'] = all((Path('/sys/class/net') / c).exists() for c in ('can0','can1'))
    for role, device in config['cameras'].items():
        checks['camera_' + role] = Path(f'/dev/video{device}').exists()
    for name, ok in checks.items():
        print(f'{"OK" if ok else "MISSING"} {name}')
    print('Read-only checks; motor behavior and driver compatibility are not certified by this check.')
    return all(checks.values())


def runtime_path():
    # Preserve the existing worker's private subprocess/module layout.
    sys.path.insert(0, str(RUNTIME))


def run(config):
    from .backends import run as run_backend
    return run_backend(config)


def cameras(config):
    runtime_path()
    from YAM_control import camera_relay
    sys.argv = ['blupe-controller', '--devices', *map(str, config['cameras'].values()), '--host', '127.0.0.1', '--port', str(config.get('settings', {}).get('camera_port', 8089))]
    if config.get('settings', {}).get('camera_settings'):
        sys.argv += ['--device-settings', json.dumps(config['settings']['camera_settings'])]
    camera_relay.main()


def publish(config):
    runtime_path()
    from scripts.publish_yam_cameras import publish as upload
    args = argparse.Namespace(camera_origin=f'http://127.0.0.1:{config.get("settings", {}).get("camera_port", 8089)}', api=config['api'],
        jetson_id=config['robot_id'], token_file=config['token_file'])
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(config['cameras'])) as pool:
        futures = [pool.submit(upload, args, role, device) for role, device in config['cameras'].items()]
        for future in futures:
            future.result()


def connect_operator(args):
    """Open only the assigned loopback operator port, using pinned SSH host trust."""
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.-]{0,252}', args.host):
        raise ValueError('Invalid tunnel host')
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]{0,63}', args.user):
        raise ValueError('Invalid tunnel account')
    if not 1024 <= args.remote_port <= 65535 or not 1024 <= args.local_port <= 65535:
        raise ValueError('Invalid assigned operator port')
    for path in (args.identity, args.known_hosts):
        if not path.is_file(): raise ValueError('Tunnel key and administrator-verified known-hosts file are required')
    if args.identity.stat().st_mode & 0o077:
        raise ValueError('Tunnel private key must be readable only by its owner')
    return subprocess.call(['ssh', '-N', '-T', '-o', 'BatchMode=yes', '-o', 'IdentitiesOnly=yes',
        '-o', 'StrictHostKeyChecking=yes', '-o', 'UserKnownHostsFile=' + str(args.known_hosts.resolve()),
        '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3',
        '-o', 'ConnectTimeout=10', '-i', str(args.identity.resolve()),
        '-R', f'127.0.0.1:{args.remote_port}:127.0.0.1:{args.local_port}', args.user + '@' + args.host])


def main(argv=None):
    parser = argparse.ArgumentParser(description='BluPe Playground robot controller')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    commands = parser.add_subparsers(dest='command', required=True)
    s = commands.add_parser('setup', help='Save robot identity and camera configuration; does not move motors')
    s.add_argument('--robot-id', required=True)
    s.add_argument('--api', required=True)
    s.add_argument('--token-file', type=Path, required=True)
    s.add_argument('--hardware', default='yam')
    s.add_argument('--cameras', type=int, nargs=3, metavar=('LEFT','TOP','RIGHT'))
    s.add_argument('--lerobot-config', type=Path, help='Reference a LeRobot SO101 robot JSON configuration')
    s.add_argument('--serial-port')
    s.add_argument('--calibration', type=Path)
    s.add_argument('--camera', action='append', help='SO101 camera ROLE=INDEX; repeat for multiple cameras')
    s.add_argument('--camera-port', type=int, default=8089)
    s.add_argument('--operator-port', type=int, default=8096)
    s.add_argument('--operator-hostname', default='localhost', help='Allowed hosted operator hostname when tunneled')
    record = commands.add_parser('record-pose', help='Record current SO101 joints without moving or recalibrating')
    record.add_argument('name', choices=('zero','home'))
    commands.add_parser('probe', help='Read SO101 positions without enabling torque or writing registers')
    commands.add_parser('doctor', help='Check dependencies and device paths without opening hardware')
    commands.add_parser('run', help='Start hardware console; motion requires explicit enable')
    commands.add_parser('cameras', help='Capture cameras on loopback port 8089')
    commands.add_parser('publish-cameras', help='Upload fresh snapshots through the existing robot API')
    tunnel = commands.add_parser('connect-operator', help='Connect the local operator console through an administrator-provisioned SSH tunnel')
    tunnel.add_argument('--host', default='100.61.149.60')
    tunnel.add_argument('--user', required=True)
    tunnel.add_argument('--remote-port', type=int, required=True)
    tunnel.add_argument('--local-port', type=int, default=8096)
    tunnel.add_argument('--identity', type=Path, required=True)
    tunnel.add_argument('--known-hosts', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'connect-operator': return connect_operator(args)
        if args.command == 'setup':
            setup(args); return 0
        config = load(args.config)
        if args.command == 'record-pose':
            if config['hardware'] == 'makerarm':
                from .makerarm import MakerArmDriver as PoseDriver
            elif config['hardware'] == 'so101':
                from .so101 import SO101Driver as PoseDriver
            else:
                raise ValueError('Pose recording supports SO101 and MakerArm')
            from .poses import PoseStore
            store = PoseStore(config)
            if store.path is None: raise ValueError('Use --lerobot-config or configure settings.poses_file for persistent recording')
            driver = PoseDriver(config).connect()
            try:
                pose = store.capture(args.name, driver.state())
                print(json.dumps({'name':args.name, 'path':str(store.path), **pose}, indent=2))
                return 0
            finally:
                driver.close()
        if args.command == 'probe':
            if config['hardware'] != 'so101': raise ValueError('probe currently supports SO101 only')
            from .so101 import console
            return console(config, probe=True)
        if args.command == 'doctor': return 0 if doctor(config) else 1
        return {'run':run, 'cameras':cameras, 'publish-cameras':publish}[args.command](config)
    except (ValueError, OSError, RuntimeError) as error:
        parser.exit(2, f'{error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
