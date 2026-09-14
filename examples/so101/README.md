# SO101 zero and home targets

`poses.json` contains the zero and home poses recorded on Andrew’s SO101 follower. Arm targets are in LeRobot degrees (`use_degrees=True`); gripper values are normalized from 0 (closed) to 1 (open).

These are reusable target examples, not universal SO101 defaults. Check the receiving arm’s calibrated joint conventions and limits before using them. For a bimanual setup, validate each arm separately.

This file intentionally omits the source robot ID and calibration fingerprint. It is not a drop-in replacement for a controller `.poses.json` file: the current pose store requires the receiving robot’s identity and calibration fingerprint. Import support is still needed; copying this file alone does not configure the controller.
