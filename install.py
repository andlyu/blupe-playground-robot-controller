#!/usr/bin/env python3
"""Install a versioned controller environment; never start services or hardware."""
import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', type=Path, default=Path.home()/'.local/share/blupe-controller/0.1.0a2')
    parser.add_argument('--driver-path', type=Path, help='Optional local checkout of your validated, patched i2rt driver')
    args = parser.parse_args()
    if sys.platform != 'linux' or not (3, 10) <= sys.version_info[:2] < (3, 13):
        parser.error('Use Linux with Python 3.10–3.12')
    prefix = args.prefix.resolve()
    if any(c in str(prefix) for c in '\n\r%"'):
        parser.error('Installation path contains unsupported service-file characters')
    if prefix.exists():
        parser.error('Destination already exists. Choose a new version directory; active installations are never overwritten.')
    prefix.mkdir(parents=True, mode=0o700)
    subprocess.run([sys.executable, '-m', 'venv', str(prefix/'venv')], check=True)
    python = str(prefix/'venv/bin/python')
    wheel = list((Path(__file__).resolve().parent/'dist').glob('*.whl'))
    if len(wheel) != 1:
        parser.error('Expected one release wheel in dist/. Download the release bundle or build it first.')
    subprocess.run([python, '-m', 'pip', 'install', str(wheel[0])+'[yam]'], check=True)
    if args.driver_path:
        subprocess.run([python, '-m', 'pip', 'install', str(args.driver_path.resolve())], check=True)
    executable = prefix/'venv/bin/blupe-controller'
    units = prefix/'systemd'
    units.mkdir()
    for name, command in [('controller','run'),('cameras','cameras'),('camera-publisher','publish-cameras')]:
        (units/f'blupe-{name}.service').write_text(f'''[Unit]
Description=BluPe {name}
After=network-online.target

[Service]
Type=simple
Environment=MUJOCO_GL=egl
ExecStart="{executable}" {command}
# Never restart a process that may own motors automatically.
Restart=no
UMask=0077

[Install]
WantedBy=default.target
''')
    print(f'Installed: {executable}\nService templates: {units}\nRun setup, then doctor. No service or motor has been started.')


if __name__ == '__main__': main()
