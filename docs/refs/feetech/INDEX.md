# LeRobot SO101 references

Source: https://github.com/huggingface/lerobot, installed version 0.5.1 (Apache-2.0),
captured 2026-09-13. Original copyright notices retained.

- `robot_config.py`, `follower_config.py`: RobotConfig and SO101FollowerConfig fields.
- `camera_config.py`: OpenCVCameraConfig fields.
- `follower.py`: standard robot, action and observation methods, connect side effects.
- `normalization.py`, `tables.py`: degree/gripper normalization and calibration reference.

BluPe uses LeRobot's Python implementation. Servo protocol code is not duplicated.
A read-only probe uses `robot.bus.connect()` because `robot.connect()` also changes
motor configuration and torque. Calibration must already match. Camera capture
runs in the existing relay with settings derived from the same robot config.
