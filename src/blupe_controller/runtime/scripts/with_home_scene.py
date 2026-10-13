#!/usr/bin/env python3
"""Install Home evidence after an existing station's controller customizations."""
import json
import os
from pathlib import Path
import runpy
import sys


def main():
    root = Path(os.environ['BLUPE_CONTROLLER_ROOT']).resolve(strict=True)
    entry = Path(os.environ['BLUPE_HOME_SCENE_ENTRYPOINT']).resolve(strict=True)
    config_path = Path(os.environ['BLUPE_HOME_SCENE_CONFIG']).resolve(strict=True)
    config = json.loads(config_path.read_text())
    sys.path.insert(0, str(root))
    import YAM_control
    public_runtime = Path(__file__).resolve().parents[1]
    # Nested launchers prepend sys.path, which recalculates namespace paths.
    # Freeze the installed-first search order so the public extension survives.
    YAM_control.__path__ = list(YAM_control.__path__)
    YAM_control.__path__.append(str(public_runtime / 'YAM_control'))
    from scripts import yam_operator_hardware_web as hardware
    original_main = hardware.base.main

    def start():
        from YAM_control.home_scene import install
        install(hardware, config, config_path.with_name('home-scene.json'))
        return original_main()

    hardware.base.main = start
    sys.argv[0] = str(entry)
    runpy.run_path(str(entry), run_name='__main__')


if __name__ == '__main__':
    main()
