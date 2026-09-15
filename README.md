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

## Supported implementations

**YAM:** the existing Linux/Jetson controller, cloud session bridge, and bimanual
operator panel. The released `v0.1.0-alpha.2` bundle remains YAM-only.

**SO101 developer integration (source checkout):** LeRobot's Python SO101 robot
and configuration, a local operator panel, and named camera capture/publishing.
There is no C++ worker for SO101. The cloud bridge supports robot-specific joint
commands and trajectories, with explicit operator queue handoff. Joint trajectories have been exercised on the development SO101; cameras use
fresh snapshots and local MJPEG. Each new robot still needs its own calibration
and attended hardware checks.
See [SO101 setup](docs/SO101.md).

**MakerMods MakerArm developer integration:** SDK control in a separate process,
six-joint poses, and the shared operator/cloud bridge. Hardware validation and a
verified motor-to-URDF mapping are still required. See [Jetson setup](docs/MAKERARM.md).

**Bimanual SO101 developer integration:** Two calibrated LeRobot arms, independent grippers, shared cameras, and optional auto-queue with return-home between completed tasks. See [bimanual setup](docs/BIMANUAL-SO101.md).

Installation never starts motors or services.

YAM saved-video processing and Past runs upload workers are included in the
source checkout. See [video processing](docs/VIDEO-PROCESSING.md) for setup and
`--keep-published` deployments that leave existing videos unchanged.

## Code layout

One command, `blupe-controller --config <profile.json> run`, selects the runtime
from the profile's `hardware` field (`yam`, `so101`, `bimanual_so101`, or `makerarm`). Imports are lazy, so SO101
does not load YAM hardware dependencies and YAM does not load LeRobot.

- `backends/`: runtime selection and robot-specific startup.
- `so101.py`: LeRobot driver; `operator.py` and `cloud.py`: its operator/session loop.
- `runtime/`: existing YAM native-worker, safety, and session implementation.
- `templates/operator.html`: shared console layout with hardware-specific bindings.
- `tolerances.py`: SO101's 5° completion/home tolerance; calibration still bounds targets.
- `lerobot_config.py`, camera relay/publisher, and CLI: setup and camera integration.

YAM and SO101 intentionally retain different motor execution paths. The current
`RobotDriver` protocol describes the single-arm SO101 interface; YAM has not been
forced into that interface. A new robot needs a backend and a validated hardware
adapter; it must not silently inherit another robot's safety or home behavior.
Astra prompts and Cartesian IK belong to the Playground runner, which sends this
controller joint waypoints. Private credentials and this Mac's calibration stay
outside source control.

## Adapting the controller to your arm

The implementation below is the work required to support another arm; selecting
a robot type in the dashboard does not implement it automatically.

### 1. Adapt the controller to the robot

**A. Joint control.** Use the arm's existing robot implementation and configuration
where possible. SO101 uses LeRobot's Python `SOFollower` and `SO101FollowerConfig`;
LeRobot handles Feetech communication and calibration. BluPe maps five joint angles
in degrees plus a 0–1 gripper value to LeRobot actions. No custom C++ driver is
required for this arm. Use independent native communication where the hardware
requires uninterrupted host commands, as with the YAM MIT-mode path.

**B. Safety precautions.** Implement calibrated position, velocity, and per-command
movement limits; validate all targets before execution. Define safe enable,
home/rest, hold, stop, and torque-off behavior for the actual arm. For controllers with independent native workers, enforce watchdogs there.
SO101 software checks run in Python and stop running if Python stalls; with torque
enabled, its servos may continue to the last target and hold it. Provide an independent stop path and test
fault handling. Replace the YAM two-arm/12-joint assumptions in
[hardware_safety.py](src/blupe_controller/runtime/YAM_control/hardware_safety.py);
do not reuse YAM home poses or CAN shutdown commands on another robot.

**C. Cameras.** Configure camera names, devices, resolution, and frame rate for
your setup. Keep camera capture/encoding outside the motor communication process.
The SO101 setup accepts any named camera roles; YAM retains left/top/right.
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
[install.py](install.py). The release installer is Linux/YAM-only and generates systemd units.
SO101 uses the source installation and foreground startup described below.

Validate with simulated hardware, then verify calibration and feedback before
physical motion tests. Test target execution, safety limits, Python stalls,
connection loss, stop behavior, and camera streaming before enabling queued runs.
The cloud API already supports per-robot queues. The SO101 hardware profile and cloud bridge use five joint targets and an empty
right-arm array, with explicit operator queue authorization.

## Download and install YAM

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
For SO101, use the separate [source setup instructions](docs/SO101.md).

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

### Saved pose examples

- [SO101 zero/home targets and Andrew’s calibration](examples/so101/README.md)
- [YAM zero/home targets](examples/yam/README.md)

### SO101 auto-queue (single or bimanual)

On macOS or Linux, choose **Enable Auto-queue** from enabled Home or saved Zero, including Zero with torque off. At Zero the arm waits for work, then enables and moves Home before accepting it. Visitor Stop, time expiry, and completion keep auto-queue enabled. If work is queued, the arms return to saved Home before accepting it. Otherwise they move to saved Zero and verify torque off; auto-queue checks for new work every two seconds, then enables and returns Home before accepting it. Operator Hold/Pause, disconnect, and faults cancel auto-queue. Both saved poses are required. It is off after startup; launching the controller never enables motion automatically. Update an existing checkout with `git pull` and restart its controller process.

## SO101 recording and uploads

Single and bimanual SO101 can record cloud sessions and publish native data,
Past runs videos and LeRobot visualizer exports. See the [Mac setup and cloud
publisher guide](docs/SO101-RECORDING.md). Recording and transfer must be explicitly
configured; pulling main alone does not enable uploads.

### Local SO101 run deadline

The controller requires a finite, positive `run_duration_s` in the Session API
`prepare_session` handoff. A monotonic watchdog starts when it accepts the session,
including time spent waiting for the model. At expiry it revokes the lease,
cancels command execution, and follows the queue-aware Home/Zero cleanup above.
Late commands are rejected. An already-enabled auto-queue stays enabled;
cleanup never enables an auto-queue that was off.

The controller reports `policy_runtime_timeout` through the existing safety-abort
protocol (the API classifies it as `timed_out`) and finalizes the recording with
that outcome. Status exposes `run_duration_s`, `run_remaining_s`, and
`last_stop_reason`; handoff logs show the duration actually received. Missing or
invalid durations fail closed. The local watchdog does not depend on cloud
connectivity, but Python scheduling and an in-flight hardware IO call can delay
command revocation; this is not a hard real-time or hardware emergency-stop mechanism.
