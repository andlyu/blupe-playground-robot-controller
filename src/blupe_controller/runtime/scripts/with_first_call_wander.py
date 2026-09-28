#!/usr/bin/env python3
"""Load the public YAM extension around an existing custom controller entry point.

No controller source is copied or replaced. Set BLUPE_CONTROLLER_ROOT and
BLUPE_YAM_ENTRYPOINT; all command-line flags are forwarded unchanged.
"""
import os
from pathlib import Path
import runpy
import sys


def main():
    root = Path(os.environ['BLUPE_CONTROLLER_ROOT']).resolve()
    entry = Path(os.environ['BLUPE_YAM_ENTRYPOINT']).resolve(strict=True)
    public_runtime = Path(__file__).resolve().parents[1]
    # Keep the rig's installed driver, mapping, calibration and planner modules.
    sys.path.insert(0, str(root))
    import YAM_control
    YAM_control.__path__.append(str(public_runtime/'YAM_control'))
    from scripts import yam_operator_hardware_web as hardware
    from YAM_control.first_call_wander_integration import install
    original_main = hardware.base.main
    def start():
        # The custom entry has now finished installing its own operator subclass.
        install(hardware)
        return original_main()
    hardware.base.main = start
    sys.argv[0] = str(entry)
    runpy.run_path(str(entry), run_name='__main__')


if __name__ == '__main__':
    main()
