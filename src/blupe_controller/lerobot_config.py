"""Read LeRobot configuration without importing or starting its hardware runtime.

Reference: docs/refs/feetech/{robot,follower,camera}_config.py (LeRobot 0.5.1).
Only the explicitly supported configuration subset is accepted.
"""
import json
from pathlib import Path
import re


def resolve(config):
    reference = config.get('settings', {}).get('lerobot_config_file')
    if not reference:
        return config
    path = Path(reference)
    robot = json.loads(path.read_text())
    if not isinstance(robot, dict):
        raise ValueError('LeRobot configuration must be an object')
    # Accept a standalone RobotConfig or the robot section of a recorded run.
    robot = robot.get('robot', robot)
    if not isinstance(robot, dict) or robot.get('type') != 'so101_follower':
        raise ValueError('Expected a LeRobot so101_follower configuration')
    supported = {'type','id','calibration_dir','port','use_degrees','disable_torque_on_disconnect','max_relative_target','cameras'}
    if set(robot) - supported:
        raise ValueError('Unsupported LeRobot robot fields: ' + ', '.join(sorted(set(robot)-supported)))
    if robot.get('use_degrees', True) is not True:
        raise ValueError('Set LeRobot use_degrees=true; BluPe joint targets use degrees')
    local_id = robot.get('id', '')
    if not isinstance(local_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]+',local_id):
        raise ValueError('Provide the LeRobot calibration id')
    directory = robot.get('calibration_dir')
    if not isinstance(directory, str) or not directory:
        raise ValueError('Provide an explicit LeRobot calibration_dir')
    directory = Path(directory).expanduser()
    if not directory.is_absolute():
        directory = path.parent / directory
    cameras, capture = {}, {}
    definitions = robot.get('cameras')
    if not isinstance(definitions, dict) or not definitions:
        raise ValueError('Provide named LeRobot OpenCV cameras')
    for role, camera in definitions.items():
        if not isinstance(camera, dict) or camera.get('type') != 'opencv':
            raise ValueError('Only LeRobot OpenCV cameras are supported')
        allowed = {'type','index_or_path','width','height','fps','color_mode','rotation','warmup_s','fourcc','backend'}
        if set(camera)-allowed:
            raise ValueError('Unsupported camera fields')
        if camera.get('rotation',0) != 0 or camera.get('backend',0) != 0 or camera.get('fourcc') not in (None,'MJPG') or camera.get('warmup_s',1) != 1 or camera.get('color_mode','RGB') not in ('RGB','BGR'):
            raise ValueError('Only default rotation/backend/warmup and MJPG capture are supported')
        device = camera.get('index_or_path')
        if type(device) is not int or device < 0:
            raise ValueError('Camera index_or_path must be a nonnegative device index')
        options = {key:camera.get(key) for key in ('width','height','fps')}
        for key, high in [('width',8192),('height',8192),('fps',120)]:
            if type(options[key]) is not int or not 1 <= options[key] <= high:
                raise ValueError(f'Camera {role} requires a valid integer {key}')
        cameras[role] = device
        capture[str(device)] = options
    # Derived values only exist in memory; reread the source for every invocation.
    result = {**config, 'cameras':cameras, 'settings':{**config['settings'],
        'serial_port':robot.get('port'), 'calibration_file':str(directory / (local_id+'.json')),
        'camera_settings':capture, 'max_relative_target':robot.get('max_relative_target'),
        'disable_torque_on_disconnect':robot.get('disable_torque_on_disconnect',True)}}
    return result
