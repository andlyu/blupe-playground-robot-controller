"""Fail-closed bimanual trajectory validation for the physical YAM gateway.

This module does not import i2rt or command hardware. Callers provide canonical
forward kinematics and swept-collision functions, making the boundary safe to
exercise with mocks before it is connected to RoboCurve.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Callable, Mapping, Sequence


JOINT_COUNT = 12
ARM_NAMES = ("left", "right")
JointVector = tuple[float, ...]
TipPosition = tuple[float, float, float]
ForwardKinematics = Callable[[JointVector], Mapping[str, Sequence[float]]]
CollisionCheck = Callable[[JointVector], bool]


class SafetyViolation(ValueError):
    """A trajectory failed a mandatory hardware-side safety check."""

    def __init__(self, code: str, *, waypoint: int | None = None) -> None:
        self.code = code
        self.waypoint = waypoint
        suffix = "" if waypoint is None else f" at waypoint {waypoint}"
        super().__init__(f"{code}{suffix}")


@dataclass(frozen=True)
class WorkspaceLimits:
    forward_m: float
    backward_m: float
    outward_m: float
    inward_m: float
    up_m: float
    down_m: float
    minimum_table_clearance_m: float


@dataclass(frozen=True)
class HardwareSafetyConfig:
    rest_tip_m: Mapping[str, TipPosition]
    table_z_m: float
    workspace: WorkspaceLimits
    joint_lower_rad: JointVector
    joint_upper_rad: JointVector
    start_limit_tolerance_rad: float
    max_adjacent_delta_rad: float
    max_joint_velocity_rad_s: float
    sweep_resolution_rad: float
    max_waypoints: int
    collision_required: bool = True

    @classmethod
    def load(cls, path: str | Path) -> "HardwareSafetyConfig":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("schema_version") != 1:
            raise ValueError("hardware safety config requires schema_version=1")
        workspace = payload["workspace_m"]
        cfg = cls(
            rest_tip_m={
                arm: _tip(payload["rest_tip_m"][arm], f"rest_tip_m.{arm}")
                for arm in ARM_NAMES
            },
            table_z_m=_number(payload["table_z_m"], "table_z_m"),
            workspace=WorkspaceLimits(
                forward_m=_positive(workspace["forward"], "workspace.forward"),
                backward_m=_positive(workspace["backward"], "workspace.backward"),
                outward_m=_positive(workspace["outward"], "workspace.outward"),
                inward_m=_positive(workspace["inward"], "workspace.inward"),
                up_m=_positive(workspace["up"], "workspace.up"),
                down_m=_positive(workspace["down"], "workspace.down"),
                minimum_table_clearance_m=_positive(
                    workspace["minimum_table_clearance"],
                    "workspace.minimum_table_clearance",
                ),
            ),
            joint_lower_rad=_joints(payload["joint_lower_rad"], "joint_lower_rad"),
            joint_upper_rad=_joints(payload["joint_upper_rad"], "joint_upper_rad"),
            start_limit_tolerance_rad=_positive(
                payload["start_limit_tolerance_rad"], "start_limit_tolerance_rad"
            ),
            max_adjacent_delta_rad=_positive(
                payload["max_adjacent_delta_rad"], "max_adjacent_delta_rad"
            ),
            max_joint_velocity_rad_s=_positive(
                payload["max_joint_velocity_rad_s"], "max_joint_velocity_rad_s"
            ),
            sweep_resolution_rad=_positive(
                payload["sweep_resolution_rad"], "sweep_resolution_rad"
            ),
            max_waypoints=int(payload["max_waypoints"]),
            collision_required=payload.get("collision_required") is True,
        )
        if cfg.max_waypoints <= 0:
            raise ValueError("max_waypoints must be positive")
        if any(lo >= hi for lo, hi in zip(cfg.joint_lower_rad, cfg.joint_upper_rad)):
            raise ValueError("each joint lower bound must be below its upper bound")
        return cfg


@dataclass(frozen=True)
class ApprovedTrajectory:
    """Immutable output produced only after the complete trajectory passes."""

    waypoints: tuple[JointVector, ...]
    cadence_hz: float
    swept_samples: int


class BimanualHardwareGuard:
    def __init__(
        self,
        config: HardwareSafetyConfig,
        *,
        forward_kinematics: ForwardKinematics | None,
        collision_check: CollisionCheck | None,
    ) -> None:
        self._config = config
        self._fk = forward_kinematics
        self._collision = collision_check

    def approve(
        self,
        start_joints_rad: Sequence[float],
        waypoints_rad: Sequence[Sequence[float]],
        *,
        cadence_hz: float,
        target_limit_tolerance_rad: float = 0.0,
    ) -> ApprovedTrajectory:
        """Validate all points and swept samples before returning any command."""
        cfg = self._config
        cadence = _positive(cadence_hz, "cadence_hz")
        target_tolerance = float(target_limit_tolerance_rad)
        if not math.isfinite(target_tolerance) or target_tolerance < 0.0:
            raise ValueError("target_limit_tolerance_rad must be finite and non-negative")
        start = _joints(start_joints_rad, "start_joints_rad")
        waypoints = tuple(
            _joints(point, f"waypoints_rad[{index}]")
            for index, point in enumerate(waypoints_rad)
        )
        if not waypoints:
            raise SafetyViolation("empty_trajectory")
        if len(waypoints) > cfg.max_waypoints:
            raise SafetyViolation("trajectory_too_long")
        if self._fk is None:
            raise SafetyViolation("forward_kinematics_unavailable")
        if cfg.collision_required and self._collision is None:
            raise SafetyViolation("collision_checker_unavailable")

        for joint, (value, lower, upper) in enumerate(
            zip(start, cfg.joint_lower_rad, cfg.joint_upper_rad)
        ):
            tolerance = cfg.start_limit_tolerance_rad
            if value < lower - tolerance or value > upper + tolerance:
                raise SafetyViolation(f"start_joint_{joint}_outside_limits")

        swept_samples = 0
        previous = start
        for waypoint_index, target in enumerate(waypoints):
            self._validate_target(target, waypoint_index, target_tolerance)
            max_delta = max(abs(after - before) for before, after in zip(previous, target))
            if max_delta > cfg.max_adjacent_delta_rad + 1e-12:
                raise SafetyViolation("adjacent_delta_limit", waypoint=waypoint_index)
            if max_delta * cadence > cfg.max_joint_velocity_rad_s + 1e-12:
                raise SafetyViolation("joint_velocity_limit", waypoint=waypoint_index)

            subdivisions = max(1, math.ceil(max_delta / cfg.sweep_resolution_rad))
            for sample_index in range(1, subdivisions + 1):
                alpha = sample_index / subdivisions
                sample = tuple(
                    before + (after - before) * alpha
                    for before, after in zip(previous, target)
                )
                self._validate_workspace(sample, waypoint_index)
                self._validate_collision(sample, waypoint_index)
                swept_samples += 1
            previous = target

        return ApprovedTrajectory(waypoints, cadence, swept_samples)

    def _validate_target(
        self, target: JointVector, waypoint: int, tolerance: float
    ) -> None:
        for joint, (value, lower, upper) in enumerate(
            zip(target, self._config.joint_lower_rad, self._config.joint_upper_rad)
        ):
            if value < lower - tolerance or value > upper + tolerance:
                raise SafetyViolation(f"joint_{joint}_outside_limits", waypoint=waypoint)

    def _validate_workspace(self, joints: JointVector, waypoint: int) -> None:
        assert self._fk is not None
        try:
            positions = self._fk(joints)
            tips = {arm: _tip(positions[arm], f"fk.{arm}") for arm in ARM_NAMES}
        except Exception as exc:
            raise SafetyViolation("forward_kinematics_failed", waypoint=waypoint) from exc
        for arm, tip in tips.items():
            if not self._tip_inside_workspace(arm, tip):
                raise SafetyViolation(f"{arm}_tip_outside_workspace", waypoint=waypoint)

    def _tip_inside_workspace(self, arm: str, tip: TipPosition) -> bool:
        cfg = self._config
        limits = cfg.workspace
        rest_x, rest_y, rest_z = cfg.rest_tip_m[arm]
        x, y, z = tip
        if not rest_x - limits.backward_m <= x <= rest_x + limits.forward_m:
            return False
        if arm == "left":
            lateral_ok = rest_y - limits.inward_m <= y <= rest_y + limits.outward_m
        else:
            lateral_ok = rest_y - limits.outward_m <= y <= rest_y + limits.inward_m
        minimum_z = max(
            rest_z - limits.down_m,
            cfg.table_z_m + limits.minimum_table_clearance_m,
        )
        return lateral_ok and minimum_z <= z <= rest_z + limits.up_m

    def _validate_collision(self, joints: JointVector, waypoint: int) -> None:
        if self._collision is None:
            return
        try:
            clear = self._collision(joints)
        except Exception as exc:
            raise SafetyViolation("collision_checker_failed", waypoint=waypoint) from exc
        if clear is not True:
            raise SafetyViolation("predicted_collision", waypoint=waypoint)


def _number(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _positive(value: object, name: str) -> float:
    number = _number(value, name)
    if number <= 0:
        raise ValueError(f"{name} must be positive")
    return number


def _joints(values: Sequence[float], name: str) -> JointVector:
    if len(values) != JOINT_COUNT:
        raise ValueError(f"{name} must contain {JOINT_COUNT} joints")
    return tuple(_number(value, f"{name}[{index}]") for index, value in enumerate(values))


def _tip(values: Sequence[float], name: str) -> TipPosition:
    if len(values) != 3:
        raise ValueError(f"{name} must contain xyz")
    return tuple(_number(value, f"{name}[{index}]") for index, value in enumerate(values))  # type: ignore[return-value]
