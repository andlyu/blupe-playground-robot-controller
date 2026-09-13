"""Local jerk-limited YAM waypath planning with MuJoCo preflight.

Ruckig API follows docs/refs/ruckig (upstream v0.15.3). This module never imports i2rt,
opens CAN, or commands hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np


JOINT_COUNT = 12
DEFAULT_CADENCE_HZ = 20.0
DEFAULT_MAX_VELOCITY_RAD_S = 0.25
DEFAULT_MAX_ACCELERATION_RAD_S2 = 0.50
DEFAULT_MAX_JERK_RAD_S3 = 1.0
MAX_CARTESIAN_DEVIATION_M = 0.025


class WaypathPlanningError(ValueError):
    pass


@dataclass(frozen=True)
class JointTrajectoryPlan:
    positions: tuple[tuple[float, ...], ...]
    velocities: tuple[tuple[float, ...], ...]
    accelerations: tuple[tuple[float, ...], ...]
    segment_indices: tuple[int, ...]
    cadence_hz: float
    duration_s: float
    max_velocity_rad_s: float
    max_acceleration_rad_s2: float
    max_jerk_rad_s3: float


@dataclass(frozen=True)
class PlannedWaypath:
    arm: str
    frame: str
    step_m: float
    cartesian_offsets_m: tuple[tuple[float, float, float], ...]
    joint_keyframes: tuple[tuple[float, ...], ...]
    trajectory: JointTrajectoryPlan
    tip_samples_m: tuple[tuple[float, float, float], ...]
    max_cartesian_deviation_m: float

    def summary(self) -> dict[str, object]:
        return {
            "arm": self.arm,
            "frame": self.frame,
            "step_m": self.step_m,
            "keyframes": len(self.joint_keyframes),
            "samples": len(self.trajectory.positions),
            "cadence_hz": self.trajectory.cadence_hz,
            "duration_s": self.trajectory.duration_s,
            "max_velocity_rad_s": self.trajectory.max_velocity_rad_s,
            "max_acceleration_rad_s2": self.trajectory.max_acceleration_rad_s2,
            "max_jerk_rad_s3": self.trajectory.max_jerk_rad_s3,
            "max_cartesian_deviation_m": self.max_cartesian_deviation_m,
        }


def box_loop_offsets(step_m: float) -> tuple[tuple[float, float, float], ...]:
    """Forward, right, up, back, left, down in the bimanual base frame."""
    step = float(step_m)
    if not math.isfinite(step) or not 0.01 <= step <= 0.10:
        raise WaypathPlanningError("step_m must be between 0.01 and 0.10")
    return (
        (0.0, 0.0, 0.0),
        (step, 0.0, 0.0),
        (step, -step, 0.0),
        (step, -step, step),
        (0.0, -step, step),
        (0.0, 0.0, step),
        (0.0, 0.0, 0.0),
    )


def collision_enabled_model(model_path: str | Path) -> mujoco.MjModel:
    """Load a dedicated model whose group-0 robot and floor geoms can contact."""
    model = mujoco.MjModel.from_xml_path(str(model_path))
    enabled = model.geom_group == 0
    model.geom_contype[enabled] = 1
    model.geom_conaffinity[enabled] = 1
    return model


def plan_joint_trajectory(
    keyframes: Sequence[Sequence[float]],
    *,
    cadence_hz: float = DEFAULT_CADENCE_HZ,
    max_velocity_rad_s: float = DEFAULT_MAX_VELOCITY_RAD_S,
    max_acceleration_rad_s2: float = DEFAULT_MAX_ACCELERATION_RAD_S2,
    max_jerk_rad_s3: float = DEFAULT_MAX_JERK_RAD_S3,
) -> JointTrajectoryPlan:
    """Generate local state-to-state segments, stopping smoothly at each keyframe."""
    from ruckig import DurationDiscretization, InputParameter, Result, Ruckig, Trajectory

    points = np.asarray(keyframes, dtype=np.float64)
    if points.ndim != 2 or len(points) < 2 or not np.all(np.isfinite(points)):
        raise WaypathPlanningError("keyframes must contain at least two finite joint vectors")
    dofs = int(points.shape[1])
    cadence = _positive(cadence_hz, "cadence_hz")
    velocity_limit = _positive(max_velocity_rad_s, "max_velocity_rad_s")
    acceleration_limit = _positive(max_acceleration_rad_s2, "max_acceleration_rad_s2")
    jerk_limit = _positive(max_jerk_rad_s3, "max_jerk_rad_s3")
    period = 1.0 / cadence
    zeros = [0.0] * dofs
    positions: list[tuple[float, ...]] = []
    velocities: list[tuple[float, ...]] = []
    accelerations: list[tuple[float, ...]] = []
    segment_indices: list[int] = []
    total_duration = 0.0

    for segment_index, (start, target) in enumerate(zip(points[:-1], points[1:])):
        if np.array_equal(start, target):
            continue
        inp = InputParameter(dofs)
        inp.current_position = start.tolist()
        inp.current_velocity = zeros
        inp.current_acceleration = zeros
        inp.target_position = target.tolist()
        inp.target_velocity = zeros
        inp.target_acceleration = zeros
        inp.max_velocity = [velocity_limit] * dofs
        inp.max_acceleration = [acceleration_limit] * dofs
        inp.max_jerk = [jerk_limit] * dofs
        inp.duration_discretization = DurationDiscretization.Discrete
        otg = Ruckig(dofs, period)
        otg.validate_input(inp, True, True)
        trajectory = Trajectory(dofs)
        result = otg.calculate(inp, trajectory)
        if result not in {Result.Working, Result.Finished}:
            raise WaypathPlanningError(
                f"Ruckig failed for segment {segment_index}: {result}"
            )
        ticks = max(1, int(round(trajectory.duration / period)))
        if abs(ticks * period - trajectory.duration) > 1e-8:
            raise WaypathPlanningError("Ruckig returned a non-discrete trajectory duration")
        for tick in range(1, ticks + 1):
            position, velocity, acceleration = trajectory.at_time(tick * period)
            positions.append(tuple(float(value) for value in position))
            velocities.append(tuple(float(value) for value in velocity))
            accelerations.append(tuple(float(value) for value in acceleration))
            segment_indices.append(segment_index)
        positions[-1] = tuple(float(value) for value in target)
        total_duration += trajectory.duration

    if not positions:
        raise WaypathPlanningError("waypath contains no motion")
    velocity_peak = float(np.max(np.abs(np.asarray(velocities))))
    acceleration_array = np.asarray(accelerations)
    acceleration_with_start = np.vstack((np.zeros((1, dofs)), acceleration_array))
    jerk_peak = float(np.max(np.abs(np.diff(acceleration_with_start, axis=0) / period)))
    if velocity_peak > velocity_limit + 1e-8:
        raise WaypathPlanningError("sampled velocity exceeds configured limit")
    if float(np.max(np.abs(acceleration_array))) > acceleration_limit + 1e-8:
        raise WaypathPlanningError("sampled acceleration exceeds configured limit")
    if jerk_peak > jerk_limit + 1e-6:
        raise WaypathPlanningError("sampled jerk exceeds configured limit")
    return JointTrajectoryPlan(
        positions=tuple(positions),
        velocities=tuple(velocities),
        accelerations=tuple(accelerations),
        segment_indices=tuple(segment_indices),
        cadence_hz=cadence,
        duration_s=total_duration,
        max_velocity_rad_s=velocity_peak,
        max_acceleration_rad_s2=float(np.max(np.abs(acceleration_array))),
        max_jerk_rad_s3=jerk_peak,
    )


class MuJoCoWaypathPlanner:
    def __init__(
        self,
        model_path: str | Path,
        joint_lower_rad: Sequence[float],
        joint_upper_rad: Sequence[float],
    ) -> None:
        self.model = collision_enabled_model(model_path)
        self.data = mujoco.MjData(self.model)
        self.lower = np.asarray(joint_lower_rad, dtype=np.float64)
        self.upper = np.asarray(joint_upper_rad, dtype=np.float64)
        if self.lower.shape != (JOINT_COUNT,) or self.upper.shape != (JOINT_COUNT,):
            raise WaypathPlanningError("bimanual planner requires 12 joint limits")

    def plan_box_loop(
        self,
        home_joints_rad: Sequence[float],
        *,
        arm: str,
        step_m: float = 0.10,
        cadence_hz: float = DEFAULT_CADENCE_HZ,
    ) -> PlannedWaypath:
        if arm not in {"left", "right"}:
            raise WaypathPlanningError("arm must be left or right")
        home = np.asarray(home_joints_rad, dtype=np.float64)
        if home.shape != (JOINT_COUNT,) or not np.all(np.isfinite(home)):
            raise WaypathPlanningError("home_joints_rad must contain 12 finite values")
        offsets = box_loop_offsets(step_m)
        body_name = f"{arm}_grasp"
        active = slice(0, 6) if arm == "left" else slice(6, 12)
        self._set_joints(home)
        home_position = np.asarray(self.data.body(body_name).xpos, dtype=np.float64).copy()
        home_rotation = np.asarray(self.data.body(body_name).xmat, dtype=np.float64).reshape(3, 3).copy()
        keyframes = [home.copy()]
        seed = home.copy()
        for offset in offsets[1:-1]:
            seed = self._solve_pose(
                seed,
                body_name,
                active,
                home_position + np.asarray(offset),
                home_rotation,
            )
            keyframes.append(seed.copy())
        keyframes.append(home.copy())
        trajectory = plan_joint_trajectory(keyframes, cadence_hz=cadence_hz)
        tips: list[tuple[float, float, float]] = []
        max_deviation = 0.0
        targets = tuple(home_position + np.asarray(offset) for offset in offsets)
        for joints, segment_index in zip(trajectory.positions, trajectory.segment_indices):
            self._set_joints(joints)
            if self.data.ncon:
                raise WaypathPlanningError(
                    f"MuJoCo predicted {self.data.ncon} contact(s) in segment {segment_index}"
                )
            tip = np.asarray(self.data.body(body_name).xpos, dtype=np.float64).copy()
            tips.append(tuple(float(value) for value in tip))
            max_deviation = max(
                max_deviation,
                _point_segment_distance(tip, targets[segment_index], targets[segment_index + 1]),
            )
        if max_deviation > MAX_CARTESIAN_DEVIATION_M:
            raise WaypathPlanningError(
                f"joint interpolation bows {max_deviation:.4f} m from the Cartesian path"
            )
        return PlannedWaypath(
            arm=arm,
            frame="yam_bimanual/base_link",
            step_m=float(step_m),
            cartesian_offsets_m=offsets,
            joint_keyframes=tuple(tuple(float(value) for value in point) for point in keyframes),
            trajectory=trajectory,
            tip_samples_m=tuple(tips),
            max_cartesian_deviation_m=max_deviation,
        )

    def _set_joints(self, joints: Sequence[float]) -> None:
        self.data.qpos[:JOINT_COUNT] = joints
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _solve_pose(
        self,
        seed: np.ndarray,
        body_name: str,
        active: slice,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
    ) -> np.ndarray:
        joints = seed.copy()
        body_id = self.model.body(body_name).id
        active_columns = np.arange(self.model.nv)[active]
        for _ in range(400):
            self._set_joints(joints)
            body = self.data.body(body_name)
            position_error = target_position - np.asarray(body.xpos)
            current_rotation = np.asarray(body.xmat).reshape(3, 3)
            rotation_delta = target_rotation @ current_rotation.T
            rotation_error = 0.5 * np.array(
                [
                    rotation_delta[2, 1] - rotation_delta[1, 2],
                    rotation_delta[0, 2] - rotation_delta[2, 0],
                    rotation_delta[1, 0] - rotation_delta[0, 1],
                ]
            )
            if np.linalg.norm(position_error) <= 0.001 and np.linalg.norm(rotation_error) <= 0.01:
                return joints
            jac_position = np.zeros((3, self.model.nv))
            jac_rotation = np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(
                self.model, self.data, jac_position, jac_rotation, body_id
            )
            jacobian = np.vstack(
                (jac_position[:, active_columns], 0.35 * jac_rotation[:, active_columns])
            )
            error = np.concatenate((position_error, 0.35 * rotation_error))
            damping = 0.02
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + damping * damping * np.eye(6), error
            )
            magnitude = float(np.max(np.abs(delta)))
            if magnitude > 0.04:
                delta *= 0.04 / magnitude
            joints[active] = np.clip(
                joints[active] + delta,
                self.lower[active] + 1e-4,
                self.upper[active] - 1e-4,
            )
        self._set_joints(joints)
        residual = float(
            np.linalg.norm(target_position - np.asarray(self.data.body(body_name).xpos))
        )
        raise WaypathPlanningError(f"IK did not converge; position residual={residual:.4f} m")


def _point_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    span = end - start
    denominator = float(span @ span)
    if denominator == 0.0:
        return float(np.linalg.norm(point - start))
    alpha = float(np.clip(((point - start) @ span) / denominator, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + alpha * span)))


def _positive(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise WaypathPlanningError(f"{name} must be positive and finite")
    return result
