"""SO101 completion tolerances shared by home, queue admission and trajectories.

These describe accepted measured error, not changes to calibration or target limits.
YAM's independent runtime retains its existing hardware-specific tolerances.
"""
SO101_JOINT_TOLERANCE_DEG = 5.0
SO101_GRIPPER_TOLERANCE = 0.02

def near_pose(state, joints, gripper, joint_tolerance_deg=SO101_JOINT_TOLERANCE_DEG):
    return (len(state['joints_deg']) == len(joints) and bool(joints)
            and max(abs(a-b) for a,b in zip(state['joints_deg'], joints)) <= joint_tolerance_deg
            and abs(state['gripper']-gripper) < SO101_GRIPPER_TOLERANCE)
