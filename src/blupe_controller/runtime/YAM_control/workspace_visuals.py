"""Render the hardware TCP workspace as non-colliding boundary planes."""

from pathlib import Path

import mujoco
import numpy as np

from YAM_control.hardware_safety import HardwareSafetyConfig


SAFETY_CONFIG = Path(__file__).resolve().parents[1] / "config/yam_hardware_safety.json"


def workspace_bounds(config, arm):
    """Match BimanualHardwareGuard._tip_inside_workspace in the base frame."""
    x, y, z = config.rest_tip_m[arm]
    w = config.workspace
    lower = np.array([x - w.backward_m,
                      y - (w.inward_m if arm == "left" else w.outward_m),
                      max(z - w.down_m, config.table_z_m + w.minimum_table_clearance_m)])
    upper = np.array([x + w.forward_m,
                      y + (w.outward_m if arm == "left" else w.inward_m),
                      z + w.up_m])
    return lower, upper


def add_workspace_planes(scene, config):
    """Call after update_scene; decorations never enter the physics model.

    Uses mjv_initGeom as in docs/refs/mujoco/python.rst and the renderer's
    scene lifecycle in docs/refs/mujoco/renderer-3.11.0.py. Thin boxes make
    finite, double-sided planes, unlike MuJoCo's infinite ground planes.
    """
    if scene.ngeom + 36 > scene.maxgeom:
        raise ValueError("MuJoCo scene needs room for 36 workspace decorations")

    def box(position, size, color):
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_BOX, size,
                           position, np.eye(3).ravel(), np.asarray(color, dtype=np.float32))
        geom.category = mujoco.mjtCatBit.mjCAT_DECOR
        scene.ngeom += 1

    for arm, color in (("left", (0.12, 0.65, 1.0)), ("right", (1.0, 0.48, 0.12))):
        lower, upper = workspace_bounds(config, arm)
        center, half = (lower + upper) / 2, (upper - lower) / 2
        for axis in range(3):
            for boundary in (lower[axis], upper[axis]):
                position, size = center.copy(), half.copy()
                position[axis], size[axis] = boundary, 0.0005
                box(position, size, (*color, 0.09))
            # Four edge lines parallel to this axis outline the planes.
            others = [i for i in range(3) if i != axis]
            for a in (lower[others[0]], upper[others[0]]):
                for b in (lower[others[1]], upper[others[1]]):
                    position, size = center.copy(), np.full(3, 0.0012)
                    position[others] = [a, b]
                    size[axis] = half[axis]
                    box(position, size, (*color, 0.65))
