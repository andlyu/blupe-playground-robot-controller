"""Allowlisted public diagnostics from private controller exceptions."""
import math
import re


def gripper_fault_message(error):
    number = r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?'
    match = re.fullmatch(
        rf'(?:RuntimeError: )?(left|right) driver returned invalid normalized gripper feedback: '
        rf'values=\[({number})\], shape=\(1,\); expected one finite value in '
        rf'\[({number}), ({number})\]', error or '')
    if not match:
        return None
    arm, reading, lower, upper = match.groups()
    values = tuple(map(float, (reading, lower, upper)))
    if not all(math.isfinite(value) for value in values):
        return None
    return (f'{arm.capitalize()} gripper feedback {values[0]:.8g} is outside the accepted range '
            f'[{values[1]:g}, {values[2]:g}]. Robot unavailable; operator attention needed.')
