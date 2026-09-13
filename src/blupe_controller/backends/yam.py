"""YAM runtime adapter preserving the native worker and existing safety stack."""
import json
import os
from pathlib import Path
import sys

def run(config):
    from ..cli import doctor, runtime_path, DEFAULT_CONFIG
    if sys.platform != 'linux':
        raise ValueError('YAM hardware control requires Linux')
    if not doctor(config):
        raise ValueError('Resolve missing prerequisites before starting the controller')
    runtime_path()
    os.environ["BLUPE_CAMERA_ROLES"] = json.dumps({k:str(v) for k,v in config["cameras"].items()})
    from scripts import yam_operator_hardware_web as hardware
    state = DEFAULT_CONFIG.parent / 'state'
    state.mkdir(parents=True, exist_ok=True, mode=0o700)
    os.environ['YAM_AUTO_QUEUE_STATE'] = str(state / 'auto-queue.json')
    # Start paused even if a previous run left a park receipt. Setup/installation
    # must never replay an automatic-start authorization.
    if Path(os.environ['YAM_AUTO_QUEUE_STATE']).exists():
        saved = json.loads(Path(os.environ['YAM_AUTO_QUEUE_STATE']).read_text())
        saved['enabled'] = False
        Path(os.environ['YAM_AUTO_QUEUE_STATE']).write_text(json.dumps(saved))
    sys.argv = ['blupe-controller', '--host', '127.0.0.1', '--port', '8096',
        '--jetson-id', config['robot_id'], '--jetson-token-file', config['token_file'],
        '--session-api-base', config['api'], '--session-api-websocket', config['api'].replace('https://','wss://',1) + '/v1/jetsons/connect',
        '--jetson-camera-base', 'http://127.0.0.1:8089', '--camera-reference-base', 'http://127.0.0.1:8089']
    return hardware.main()
