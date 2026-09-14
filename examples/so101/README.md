# SO101 zero and home targets

## Andrew’s calibration

[`dice_06.json`](dice_06.json) is the LeRobot calibration used by Andrew’s single SO101 follower (`id: dice_06`). It includes the six motors’ IDs, drive modes, homing offsets, and calibrated raw position ranges.

For that arm, use this directory as LeRobot’s `calibration_dir` with `id: dice_06`, or point the controller’s `settings.calibration_file` to this file. Other SO101 arms need their own calibration; this is not a universal default. Adding the file does not change calibration on connected hardware.

## Saved poses

`poses.json` contains the zero and home poses recorded on Andrew’s SO101 follower. Arm targets are in LeRobot degrees (`use_degrees=True`); gripper values are normalized from 0 (closed) to 1 (open).

These are reusable target examples, not universal SO101 defaults. Check the receiving arm’s calibrated joint conventions and limits before using them. For a bimanual setup, validate each arm separately.

This file intentionally omits the source robot ID and calibration fingerprint. It is not a drop-in replacement for a controller `.poses.json` file: the current pose store requires the receiving robot’s identity and calibration fingerprint. Import support is still needed; copying this file alone does not configure the controller.
