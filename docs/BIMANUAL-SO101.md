# Bimanual SO101

1. **Hardware:** Set `hardware` to `bimanual_so101`. In `settings.arms`, name the two serial ports; `settings.arm_mapping` maps `left` and `right` to those names. Configure named cameras independently.
2. **Calibration:** `settings.calibrations` maps each arm name to its own absolute LeRobot calibration-file path. The controller loads both calibrations and checks both joint targets before writing either arm. Without calibration it runs a read-only monitor. The deployed Jetson uses LeRobot 0.6.2; the existing `so101` install extra pins 0.5.1, so reproduce and verify the runtime appropriate to your hardware.
3. **Poses and queue:** Use a `settings.poses_file` with ten arm joint angles and two gripper values, bound to the robot and calibration identity. Auto-queue is off at startup. Enable both arms, reach home, and enable auto-queue in the Operator panel. Visitor Stop, time expiry, and completion preserve auto-queue: return Home for waiting work, otherwise move to saved Zero and verify torque off. Later queued work enables both arms and returns Home before acceptance. Operator Hold/Pause, disconnect, and faults cancel auto-queue.
4. **Limits:** Each arm retains its calibrated joint limits. Cartesian IK/workspace limits belong to the Playground runner. This controller does not yet check collisions between the two arms.

Run the profile with `blupe-controller --config /path/to/profile.json run`.
The cloud connection uses that profile’s robot ID and controller credential; remote Operator access uses a tunnel originating on the robot computer.
