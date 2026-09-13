# BluPe Playground robot controller

## What the controller does

The controller is the software that runs on the computer connected to your robot,
such as a Jetson, desktop, or laptop. It connects the physical arm and its cameras
to BluPe Playground so people can request tasks and watch the robot perform them.

It receives commands from the cloud API, moves the robot to requested joint
positions, and sends back its current positions and execution status. It also
captures and publishes camera images, enforces local safety limits and stop
behavior, and provides controls for the robot’s operator.

The cloud API manages requests and assigns each session to the correct robot.
The controller handles the actual hardware: servo communication, calibration,
motion execution, and cameras. Adapting it to another arm means implementing
those hardware-specific parts.

## Current implementation: YAM

**The current implementation is for a bimanual YAM setup on Linux/Jetson.** It is
a developer alpha, not a universal robot controller. Selecting another robot type
in the dashboard does not add its driver to this package. To use your own arm,
adapt the hardware-specific parts below; keep the cloud connection and session
protocol.

Setup and wheel installation are tested without hardware. Physical operation and
the external patched i2rt dependency still require validation on the target machine.
Installation never starts motors or services.

## Adapting the controller to your arm

The implementation below is the work required to support another arm; selecting
a robot type in the dashboard does not implement it automatically.

### 1. Adapt the controller to the robot

**A. Joint control through C++.** Implement your arm’s driver so it can read joint
positions and move to requested joint targets. Define servo IDs, joint order,
units, gripper conversion, calibration, and serial/CAN configuration. Continuous
servo communication must run in an independent **C++ process**, so a Python
interpreter pause cannot interrupt the communication loop. A C++ extension called
synchronously from Python alone does not meet this requirement. Python can handle
setup, UI, and cloud messages; send bounded, timestamped commands to the C++
process and receive feedback over IPC. The native process must detect stale
commands and apply the arm’s defined safe behavior. Verify communication timing
under Python load; using C++ alone does not guarantee real-time scheduling.

Start from the current [YAM adapter](src/blupe_controller/runtime/YAM_control/i2rt_bimanual_adapter.py)
and [motor worker](src/blupe_controller/runtime/YAM_control/motor_worker.py), but
replace their hardware-specific implementation. The alpha’s worker is a separate
Python process; it does **not yet meet this C++ requirement** for the new adapter.
For SO101, map its five arm joints and separate gripper to the Feetech interface.
Keep the local calibration ID separate from the cloud robot ID.

**B. Safety precautions.** Implement calibrated position, velocity, and per-command
movement limits; validate all targets before execution. Define safe enable,
home/rest, hold, stop, and torque-off behavior for the actual arm. Enforce limits
and command/connection watchdogs in the native controller so they still work if
Python or the cloud stops responding. Provide an independent stop path and test
fault handling. Replace the YAM two-arm/12-joint assumptions in
[hardware_safety.py](src/blupe_controller/runtime/YAM_control/hardware_safety.py);
do not reuse YAM home poses or CAN shutdown commands on another robot.

**C. Cameras.** Configure camera names, devices, resolution, and frame rate for
your setup. Keep camera capture/encoding outside the motor communication process.
Replace the alpha’s mandatory left/top/right camera mapping with your arm’s layout.
Verify camera identity and capture timestamps; a repeated old image is not a live
stream.

### 2. Connect the controller to the platform

**A. Operator access.** Run the robot-specific operator panel on a configurable
local port and connect it through its assigned authenticated tunnel. The alpha
uses local port 8096; its separate YAM hard-off service uses 8098. Other arms need
appropriate controls and a stop implementation. An administrator currently
provisions dedicated remote loopback ports and the tunnel account. Keep local
control ports bound to loopback rather than exposing them directly to the internet.

**B. Cloud API connection.** Connect outbound over HTTPS/WSS on port 443 using the
robot ID and controller credential from the operator dashboard. This connection
needs no inbound port or SSH tunnel. Reuse the
[WebSocket transport](src/blupe_controller/runtime/YAM_control/session_api_sim_client.py)
and implement its preparation, trajectory, feedback, heartbeat, and stop callbacks.
Use `/v1/robots/<robot_id>/queue` and include `robot_id` in session creation; the
alpha’s unqualified `/v1/queue` reader still defaults to `yam-1`. The API accepts
variable-length `left_joints_deg` and `right_joints_deg` arrays and separate 0–1
gripper values. A single arm can use the left array and an empty right array.

**C. Camera streaming.** Configure the local camera service (port 8089 in the alpha)
and publish each feed under the correct robot ID. The package includes fresh JPEG
uploads, but not the hosted low-latency video publisher. Integrate the video
publisher/receiver and route that robot’s streams into its operator/user views.
Verify the complete path for correct camera labels, freshness, synchronization
where needed, reconnect behavior, and visible stream-loss reporting.

Configure dependencies and startup for the controller computer in
[cli.py](src/blupe_controller/cli.py), [pyproject.toml](pyproject.toml), and
[install.py](install.py). The alpha is Linux/YAM-only and generates systemd units;
a Mac-connected arm also needs a macOS installation/startup path.

Validate with simulated hardware, then verify calibration and feedback before
physical motion tests. Test target execution, safety limits, Python stalls,
connection loss, stop behavior, and camera streaming before enabling queued runs.
The cloud API already supports per-robot queues; SO101 and other non-YAM controller
profiles are not implemented in this release.

## Download and install

Download the `.tar.gz` bundle from [Releases](https://github.com/andlyu/blupe-playground-robot-controller/releases).
Extract it, then on Linux with Python 3.10–3.12 and venv/pip available:

```sh
python3 install.py --driver-path /path/to/your/validated/i2rt
```

The installer creates `~/.local/share/blupe-controller/0.1.0a2/venv`, installs the
bundled wheel and pinned Python runtime dependencies, and writes systemd user
service templates. Internet access is needed for dependencies. i2rt is not bundled
or silently downloaded: use the tested driver checkout for your robot, including
its motor shutdown/recovery patches. This repo does not certify stock i2rt as an
interchangeable replacement. CAN interfaces and camera permissions must already
be configured by the owner. One controller computer per robot setup in this alpha.

## Connect your robot

Create an account in the [operator dashboard](https://operator-blupe-yam.100-61-149-60.sslip.io/),
then select **Create/setup a robot** and download its private settings. Use the
provided robot ID and save its controller credential in a private token file.
The settings download is a reference; this alpha does not import it automatically.
Operator tunnel provisioning remains an administrator step.

```sh
CONTROLLER="$HOME/.local/share/blupe-controller/0.1.0a2/venv/bin/blupe-controller"
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
video pipeline is not packaged in this alpha; see below for the operator tunnel command.

**Before enabling queued runs for a new robot, update this alpha’s queue reader
and any session submissions to use that robot’s ID.** Cloud routing is deployed;
the alpha’s unqualified queue reader still defaults to `yam-1`.

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

## Operator dashboard connection

Sign into the [operator dashboard](https://operator-blupe-yam.100-61-149-60.sslip.io/)
with your operator username/password. Your administrator
must assign your robot to your account and reserve a dedicated loopback tunnel
port. The controller uses a separate SSH key, never your dashboard password.

With the controller running locally, start the provisioned tunnel:

```sh
"$CONTROLLER" connect-operator --user YOUR_TUNNEL_ACCOUNT \
  --remote-port YOUR_ASSIGNED_PORT --identity /path/to/private-key \
  --known-hosts /path/to/administrator-verified-known-hosts
```

The SSH server must restrict this key to the assigned loopback forwarding port;
use a dedicated account with no shell access. Host verification is mandatory.
This command forwards the operator console on local port 8096; the existing
independent hard-off service/tunnel on 8098 remains separately provisioned.
It neither creates a cloud account nor changes the cloud dashboard deployment.
