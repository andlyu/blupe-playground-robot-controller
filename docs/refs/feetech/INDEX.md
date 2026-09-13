# SO101 protocol references

Captured from this Mac’s installed packages on 2026-09-13; LeRobot 0.5.1 (Apache-2.0) and feetech-servo-sdk 1.0.0 (Unlicense); original copyright notices retained. LeRobot source: https://github.com/huggingface/lerobot. Feetech SDK: https://github.com/Adam-Software/FEETECH-Servo-Python-SDK. These snapshots are reference material, not runtime dependencies.

- Register addresses/model numbers: `tables.py` (STS3215 model 777, protocol endianness 0).
- Packet framing/checksum/read/write: `packet.py`, `constants.py`.
- Calibration and units: `normalization.py`, `_normalize` / `_unnormalize`.
- Motor IDs, gripper setup and connect side effects: `follower.py`.

The native driver reads calibrated positions from register 56. Homing offsets are already applied by the servo; compare register 31 with the calibration instead of adding offsets again. Body joints use degrees about the calibrated midpoint, with 4095 ticks per revolution; the gripper uses a normalized fraction of its calibrated range. `SOFollower.connect()` configures motors and toggles torque, so it is unsuitable for a read-only probe.

Only the STS3215 protocol is implemented. No homing/calibration writes occur. Native limits and a heartbeat timeout remain active if Python stalls. Hardware force limits and emergency power removal still require commissioning.
