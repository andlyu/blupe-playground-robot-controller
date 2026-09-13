# BluPe Playground robot controller

The local controller for BluPe Playground: YAM control, camera capture, and the
connection to the cloud Session API. This repository is separate from the cloud
API and dashboards.

**Developer alpha, not a certified turnkey Jetson installer.** Setup and wheel
installation are tested without hardware. Linux/Jetson hardware installation,
physical operation, and the external patched i2rt dependency still require
validation on the target machine. Installation never starts motors or services.

## Download and install

Download the `.tar.gz` bundle from [Releases](https://github.com/andlyu/blupe-playground-robot-controller/releases).
Extract it, then on Linux with Python 3.10–3.12 and venv/pip available:

```sh
python3 install.py --driver-path /path/to/your/validated/i2rt
```

The installer creates `~/.local/share/blupe-controller/0.1.0a1/venv`, installs the
bundled wheel and pinned Python runtime dependencies, and writes systemd user
service templates. Internet access is needed for dependencies. i2rt is not bundled
or silently downloaded: use the tested driver checkout for your robot, including
its motor shutdown/recovery patches. This repo does not certify stock i2rt as an
interchangeable replacement. CAN interfaces and camera permissions must already
be configured by the owner. One controller computer per robot setup in this alpha.

## Connect your robot

Your administrator registers the robot/controller in the existing cloud API and
provides its robot ID and device credential. Save that credential in a private
file, readable only by you. Operator dashboard assignment and the outbound tunnel
are currently provisioned separately. **Pairing codes are not implemented yet.**

```sh
CONTROLLER="$HOME/.local/share/blupe-controller/0.1.0a1/venv/bin/blupe-controller"
"$CONTROLLER" setup --robot-id YOUR_ROBOT_ID --api https://YOUR_SESSION_API \
  --token-file "$HOME/.config/blupe-controller/device-token" --cameras 10 16 4
"$CONTROLLER" doctor
```

The three camera numbers above are examples in left/top/right order; use your
actual devices. Configuration is stored privately at
`~/.config/blupe-controller/config.json`. YAM has no additional guided settings
in this release; it uses the extracted bimanual profile (`can0` and `can1`).
SO101 calibration and custom driver setup are future extensions and are rejected,
not presented as working options.

After diagnostics pass, run in separate terminals:

```sh
"$CONTROLLER" cameras
"$CONTROLLER" publish-cameras
"$CONTROLLER" run
```

Open `http://127.0.0.1:8096` locally. Arms remain disabled until an operator launches
them. Automatic queue startup starts paused. Camera capture is loopback-only;
fresh snapshots upload to robot-scoped API routes. The hosted low-latency composite
video pipeline and operator tunnel are not packaged in this first alpha.

**Do not connect a second hardware setup to the shared production queue yet:**
robot-specific cloud dispatch is a separate platform change. Test against an
isolated API until that change is deployed.

Systemd templates are generated but not installed/enabled. Choose those only after
manual target validation. There is no auto-update or automatic service restart.
Keep the previous version for rollback; stop/park hardware before switching.

## Development

```sh
python -m pip install -e .
python -m unittest discover -s tests
python -m pip wheel --no-deps . -w dist
```

`blupe-controller setup` and `doctor` do not open CAN devices. The wheel contains
models and the isolated motor worker. `SOURCE-MANIFEST.json` records the extracted
files from blupe-evals; modifications in this repo remove private LAN defaults,
parameterize camera roles, and provide standalone packaging. No robot credentials,
recordings, or evaluation datasets are included.

See `THIRD-PARTY-NOTICES.md` for model and dependency attribution. This alpha makes
no license grant for first-party code; third-party notices retain their original
terms. A first-party license can be chosen before a stable release.
