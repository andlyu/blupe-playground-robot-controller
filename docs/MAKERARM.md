# MakerArm on a fresh Jetson

This backend uses the MakerArm SDK private protocol and its production profile.
The LeRobot `robot/makermods-maker-arm` branch uses a different MIT wire protocol
and folded-pose zero. Do not run both or switch motor protocol automatically.
Have the owner confirm which protocol and zero the arm currently uses first.

## Install

Inspect `cat /etc/os-release`, `cat /etc/nv_tegra_release`, `python3 --version`,
`ip link`, and connected USB devices. Use Python 3.10–3.12. On Ubuntu:

```sh
sudo apt-get update
sudo apt-get install -y git python3-venv python3-dev can-utils libgl1 libglib2.0-0

git clone https://github.com/makermods-robotics/maker-arm-sdk.git
cd maker-arm-sdk
git checkout b30d05a23d72e8c155a8e00f807aba8e8c705f68
python3 -m venv .venv
. .venv/bin/activate
pip install -e .
cd ..
git clone https://github.com/andlyu/blupe-playground-robot-controller.git
pip install -e './blupe-playground-robot-controller[makerarm]'
maker-arm doctor
```

Use a controller checkout containing the MakerArm integration. These local changes
must be published before a fresh public clone includes them.

For native SocketCAN, configure the existing adapter at 1 Mbps as described in
SDK docs/linux.md. For a serial SLCAN adapter, use its documented setup script or
choose backend `slcan` and the actual serial port. Do not assume `/dev/ttyACM0`.
Do not execute `zero`, `check`, or `teleop` during read-only setup.

## Private configuration

Write a version-1 controller JSON with hardware `makerarm`, the actual registered
`robot_id`, HTTPS `api`, absolute `token_file`, and named `cameras` (e.g. front:0).
Settings:

- `backend`: socketcan or slcan; `channel`: actual CAN interface or serial port.
- `gripper_endpoints_rad`: [closed, open], measured SDK actuator coordinates.
  Required; no jaw-width conversion or motor endpoints are guessed.
- `max_velocity`: initially 0.1 rad/s or lower.
- `operator_port`: 8096; `camera_port`: 8089 (distinct local ports).
- `poses_file`: absolute private path for saved zero/home poses.
- `cloud_enabled`: false until local setup and registration are verified.
- `operator_hostname`: approved hostname if using the operator tunnel.

Run `blupe-controller --config /absolute/config.json doctor`, then `run`.
Connect is read-only. Explicit Launch Arm enables torque. Capture home/zero with `blupe-controller --config /absolute/config.json record-pose home`
(or `zero`), while the controller is stopped so there is only one bus owner,
after positioning the arm; recording does not change calibration.

Use the existing `cameras` command for previews and `publish-cameras` after the
owner has approved publishing. API traffic is outbound HTTPS/WSS. Remote operator
access requires a separate approved tunnel to the loopback operator port; camera
preview is also loopback. Do not open CAN or broad inbound ports to the Internet.

## Process behavior

The SDK's CAN loop runs in a separate process from the web/cloud controller.
After one second without parent requests it holds measured position. Missing CAN
feedback retains the SDK's fault/watchdog behavior; motor watchdogs may release
torque. This is not a hard real-time or power-loss holding guarantee.

On normal shutdown an enabled arm holds and asks for RELEASE before disconnecting.
Support the arm and payload first. On parent death the worker holds and remains
alive; do not launch another bus owner. Support/isolate the arm before terminating
an orphan worker. Automatic startup is intentionally not configured until actual
hardware validation and the owner's service/shutdown procedure are established.

## Runner and deployment

In blupe-remote-yam configure hardware `makerarm`, joint_counts [6,0], camera names,
and a verified SDK-joint-to-URDF mapping. The vendor explicitly says physical zero
alignment is unverified; identity mapping is not a physical calibration.

The operator portal and registry must include makerarm support and this robot's
existing tunnel ports. Test home, hold, queue handoff, and small trajectories with
the owner before making it available. No physical acceptance test has been run here.
