# YAM zero and home targets

`poses.json` documents the existing dual-YAM operator targets. Each arm has six joint targets in driver order, in **radians**. Gripper values use the normalized driver scale; `1.0` is fully open. `null` means no gripper target is specified.

- **Home:** copied from `src/blupe_controller/runtime/config/yam_operator_hardware_pose.json`, including fully open grippers.
- **Zero:** six zero joint angles per arm, matching `_park_zero_and_stop` in `src/blupe_controller/runtime/scripts/yam_operator_hardware_web.py`. That operation supplies no explicit gripper target and disables torque after parking.

This is a reference target file, not a new controller configuration format. The controller continues to use its existing home configuration and park implementation. Validate joint conventions and limits for the receiving hardware before motion.

For cached startup planning, the station requests a **10-second Home trajectory**
when its fresh scene offer has been accepted for the same eligible queued run
and the configuration still matches. This is the Home movement itself; launch
and gripper calibration happen first. Existing velocity/acceleration/jerk limits
may require longer. No timing history is needed. Without a usable accepted cache,
Home keeps its normal safe timing with no added delay. The playground Stop button and normal policy completion first revoke the run,
then return Home, capture the scene, and park at Zero with torque disabled. This
cleanup also runs when automatic queue starts are paused. Operator emergency
Stop, faults, disconnect, credential clearing and timeout do not request a Home
capture.
