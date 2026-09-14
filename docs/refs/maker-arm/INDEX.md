# MakerArm authoritative references

Fetched 2026-09-13.

- SDK: https://github.com/makermods-robotics/maker-arm-sdk
  commit `b30d05a23d72e8c155a8e00f807aba8e8c705f68`, Apache-2.0.
  URDF.md is `urdf/maker_arm/README.md`; SAFETY.md is `docs/safety.md`.
  `models/makerarm/robot.urdf` is the unmodified upstream URDF. This runner reads
  joint transforms only; it neither loads meshes nor performs collision checks.
- LeRobot: https://github.com/makermods-robotics/lerobot/tree/robot/makermods-maker-arm
  commit `2115f638667c6b1625527cd4284a400e8efad093`, Apache-2.0.
  LEROBOT.mdx is `docs/source/maker.mdx`.

## Use-case map

- Grasp frame, axes, limitations: URDF.md, original robot.urdf.
- Torque/watchdog behavior: SAFETY.md, SDK `maker_arm/arm.py`.
- MIT protocol and resting-pose zero: LEROBOT.mdx and
  `src/lerobot/robots/maker_follower/{maker_follower,config_maker_follower}.py`.

## Integration findings

The SDK and LeRobot branch are NOT interchangeable at the bus/zero boundary.
The branch requires a motor protocol switch and connect() enables torque.
The BluPe backend deliberately uses SDK Arm.connect() (read-only), its packaged
model profile, and a separate process for the SDK's CAN control thread. Never
run both programs on the same bus or switch protocol automatically.

URDF physical zero alignment is unverified upstream. Require a measured mapping
`q_model = signs * radians(controller_degrees) + offsets_rad`. Do not substitute
identity mapping for missing calibration. Native base_link is Y-up; the viewer's
90-degree display rotation is not part of the model. Gripper normalization is
actuator travel, not jaw width; the SDK says its motor-to-gap relation is uncalibrated.
