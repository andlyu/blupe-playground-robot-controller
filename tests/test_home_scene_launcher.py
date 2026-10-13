import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class NestedLauncherTest(unittest.TestCase):
    def test_namespace_extension_survives_nested_launcher(self):
        source = Path(__file__).resolve().parents[1] / 'src/blupe_controller/runtime/scripts/with_home_scene.py'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            installed = root / 'installed'
            public = root / 'public'
            for base in (installed, public):
                (base / 'YAM_control').mkdir(parents=True)
                (base / 'scripts').mkdir()
            (installed / 'scripts/__init__.py').write_text('')
            (installed / 'scripts/yam_operator_hardware_web.py').write_text(
                "from types import SimpleNamespace\nbase = SimpleNamespace(main=lambda: print('started'))\nPhysicalOperator = 'installed'\n")
            (public / 'YAM_control/home_scene.py').write_text(
                "def install(hardware, config, path):\n    assert hardware.PhysicalOperator == 'custom'\n    print('home-scene-installed')\n")
            (public / 'YAM_control/first_call_wander_integration.py').write_text(
                "def install(hardware):\n    hardware.PhysicalOperator = 'custom'\n")
            launcher = public / 'scripts/with_home_scene.py'
            launcher.write_text(source.read_text())
            entry = public / 'scripts/existing.py'
            entry.write_text(
                "import os, sys\nfrom pathlib import Path\nsys.path.insert(0, os.environ['BLUPE_CONTROLLER_ROOT'])\n"
                "import YAM_control\nYAM_control.__path__.append(str(Path(__file__).resolve().parents[1] / 'YAM_control'))\n"
                "from YAM_control.first_call_wander_integration import install\n"
                "from scripts import yam_operator_hardware_web as hardware\ninstall(hardware)\nhardware.base.main()\n")
            config = root / 'config.json'
            config.write_text('{"robot_id":"test"}')
            env = dict(os.environ, BLUPE_CONTROLLER_ROOT=str(installed),
                       BLUPE_HOME_SCENE_ENTRYPOINT=str(entry), BLUPE_HOME_SCENE_CONFIG=str(config))
            result = subprocess.run([sys.executable, str(launcher)], env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.splitlines(), ['home-scene-installed', 'started'])
