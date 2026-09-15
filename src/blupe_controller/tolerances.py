"""SO101 completion tolerances shared by home, queue admission and trajectories.

These describe accepted measured error, not changes to calibration or target limits.
YAM's independent runtime retains its existing hardware-specific tolerances.
"""
SO101_JOINT_TOLERANCE_DEG = 5.0
SO101_GRIPPER_TOLERANCE = 0.05

def near_pose(state, joints, gripper, joint_tolerance_deg=SO101_JOINT_TOLERANCE_DEG):
    return (len(state['joints_deg']) == len(joints) and bool(joints)
            and max(abs(a-b) for a,b in zip(state['joints_deg'], joints)) <= joint_tolerance_deg
            and gripper_error(state['gripper'],gripper) < SO101_GRIPPER_TOLERANCE)


def gripper_values(value):
    return list(value) if isinstance(value,(list,tuple)) else [value]

def gripper_error(a,b):
    a,b=gripper_values(a),gripper_values(b)
    return max(abs(x-y) for x,y in zip(a,b)) if len(a)==len(b) else float('inf')

def gripper_step(target, previous, limit):
    result=[b+max(-limit,min(limit,a-b)) for a,b in zip(gripper_values(target),gripper_values(previous))]
    return result if isinstance(target,(list,tuple)) else result[0]
