# YAM zero and home targets

`poses.json` documents the existing dual-YAM operator targets. Each arm has six joint targets in driver order, in **radians**. Gripper values use the normalized driver scale; `1.0` is fully open. `null` means no gripper target is specified.

- **Home:** copied from `src/blupe_controller/runtime/config/yam_operator_hardware_pose.json`, including fully open grippers.
- **Zero:** six zero joint angles per arm, matching `_park_zero_and_stop` in `src/blupe_controller/runtime/scripts/yam_operator_hardware_web.py`. That operation supplies no explicit gripper target and disables torque after parking.

This is a reference target file, not a new controller configuration format. The controller continues to use its existing home configuration and park implementation. Validate joint conventions and limits for the receiving hardware before motion.
