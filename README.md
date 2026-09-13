# BluPe Playground robot controller

The local controller connects a robot and its cameras to BluPe Playground’s cloud
API and operator dashboard.

**The current implementation is for a bimanual YAM setup on Linux/Jetson.** It is
a developer alpha, not a universal robot controller. Selecting another robot type
in the dashboard does not add its driver to this package. To use your own arm,
adapt the hardware-specific parts below; keep the cloud connection and session
protocol.

Setup and wheel installation are tested without hardware. Physical operation and
the external patched i2rt dependency still require validation on the target machine.
Installation never starts motors or services.

## Adapting the controller to your arm

Start from this repository and add a hardware profile for your robot. This requires
code changes as well as configuration; the current CLI accepts only `yam`.

1. **Connect your servos.** Implement an adapter that connects through your arm’s
   USB serial, CAN, or other interface, reads joint/gripper positions, executes
   trajectories, and stops execution. Replace the YAM/i2rt implementation for
   your profile; use your manufacturer’s SDK or an existing compatible driver.
   C++ changes are only needed if that driver requires them. Start with
   [i2rt_bimanual_adapter.py](src/blupe_controller/runtime/YAM_control/i2rt_bimanual_adapter.py)
   and its [isolated motor worker](src/blupe_controller/runtime/YAM_control/motor_worker.py).
2. **Define calibration and motion behavior.** Specify servo IDs/order, joint
   units, gripper conversion, limits, and home/rest positions. Implement explicit
   enable, hold/stop, disconnect, and torque-off behavior for your arm. Replace
   the YAM model and two-arm/12-joint assumptions in
   [hardware_safety.py](src/blupe_controller/runtime/YAM_control/hardware_safety.py)
   and the local operator controls. Do not reuse YAM home poses or CAN shutdown
   commands on another robot. For SO101, this means five arm joints plus a gripper,
   its saved calibration, and a Feetech serial driver.
3. **Configure images and ports.** Map camera names to the correct devices and
   choose local camera/operator ports. The current setup requires three cameras
   named left/top/right and uses ports 8089/8096; make these configurable for your
   profile in [cli.py](src/blupe_controller/cli.py). The cloud API accepts variable
   camera names, but the packaged UI and publisher still need the matching changes.
   JPEG uploads are included; the hosted low-latency video publisher is not.
4. **Map your arm to the cloud protocol.** Implement the callbacks used by
   [SessionApiSimClient](src/blupe_controller/runtime/YAM_control/session_api_sim_client.py):
   preparation, trajectory execution, feedback, heartbeat, and stop. Despite its
   historical name, this transport is also used by the hardware controller. The
   API accepts variable-length joint arrays; its current wire names remain
   `left_joints_deg` and `right_joints_deg`, with grippers separate. A single arm
   can use the left array and an empty right array. API grippers use 0–1; convert
   your driver’s units explicitly. Always read `/v1/robots/<robot_id>/queue` and
   include `robot_id` when creating sessions. The alpha’s
   [queue reader](src/blupe_controller/runtime/scripts/yam_operator_sim_web.py)
   still uses `/v1/queue`, which defaults to `yam-1`.
5. **Support your controller computer.** Add your profile’s dependencies, device
   checks, and startup path in [cli.py](src/blupe_controller/cli.py),
   [pyproject.toml](pyproject.toml), and [install.py](install.py). The installer
   currently requires Linux and generates systemd units. A Mac-connected arm needs
   a macOS installation/startup path and serial/camera checks instead of Linux CAN
   checks. Update the local and hosted operator panel for your arm’s controls.
6. **Validate, then connect.** Test the adapter with simulated hardware first,
   then validate calibration and feedback before enabling physical commands.
   Test motion limits, stop/disconnect behavior, camera freshness, and routing to
   your own robot queue. Create your robot in the operator dashboard and use its
   generated robot ID and controller credential. Keep that cloud ID separate from
   any local calibration ID. An administrator still provisions the operator tunnel.

The cloud API already supports per-robot queues and routing. You do not need to
host another API for your arm; you do need a compatible controller adapter and
operator panel. SO101 and other non-YAM profiles are not implemented in this release.

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
