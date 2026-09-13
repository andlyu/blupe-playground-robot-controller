"""Versioned, fail-closed parsing for Jetson-local bimanual trajectories."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import re
import time
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = 1
MESSAGE_TYPE = "joint_trajectory"
JOINTS_PER_ARM = 6
DEFAULT_CADENCE_HZ = 10.0
MAX_WAYPOINTS = 300
MAX_DISPATCH_AGE_S = 10.0
MAX_FUTURE_SKEW_S = 2.0
MAX_GRIPPER_DELTA_PER_STEP = 0.3
_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")
_TOP_LEVEL_FIELDS = frozenset(
    {
        "schema_version",
        "type",
        "session_id",
        "episode_id",
        "lease_id",
        "trajectory_id",
        "dispatched_at",
        "cadence_hz",
        "waypoints",
    }
)
_WAYPOINT_FIELDS = frozenset(
    {
        "step_id",
        "left_joints_deg",
        "right_joints_deg",
        "left_gripper",
        "right_gripper",
    }
)


class TrajectorySchemaError(ValueError):
    """A trajectory message is malformed, stale, conflicting, or out of sequence."""

    def __init__(
        self,
        code: str,
        *,
        waypoint: int | None = None,
        step_id: int | None = None,
    ) -> None:
        self.code = code
        self.waypoint = waypoint
        self.step_id = step_id
        detail = "" if waypoint is None else f" at waypoint {waypoint}"
        super().__init__(f"{code}{detail}")


@dataclass(frozen=True)
class TrajectoryWaypoint:
    step_id: int
    left_joints_deg: tuple[float, ...]
    right_joints_deg: tuple[float, ...]
    left_gripper: float | None
    right_gripper: float | None

    @property
    def joints_rad(self) -> tuple[float, ...]:
        return tuple(math.radians(value) for value in self.left_joints_deg + self.right_joints_deg)


@dataclass(frozen=True)
class JointTrajectory:
    session_id: str
    episode_id: str
    lease_id: str
    trajectory_id: str
    dispatched_at: float
    cadence_hz: float
    waypoints: tuple[TrajectoryWaypoint, ...]

    @property
    def first_step_id(self) -> int:
        return self.waypoints[0].step_id

    @property
    def last_step_id(self) -> int:
        return self.waypoints[-1].step_id

    @property
    def fingerprint(self) -> str:
        canonical = {
            "session_id": self.session_id,
            "episode_id": self.episode_id,
            "lease_id": self.lease_id,
            "cadence_hz": self.cadence_hz,
            "waypoints": [
                {
                    "step_id": point.step_id,
                    "left_joints_deg": point.left_joints_deg,
                    "right_joints_deg": point.right_joints_deg,
                    "left_gripper": point.left_gripper,
                    "right_gripper": point.right_gripper,
                }
                for point in self.waypoints
            ],
        }
        encoded = json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode("ascii")
        return hashlib.sha256(encoded).hexdigest()


def parse_joint_trajectory(
    payload: Mapping[str, Any],
    *,
    expected_session_id: str,
    expected_episode_id: str,
    expected_lease_id: str,
    expected_step_id: int,
    now_s: float | None = None,
    max_waypoints: int = MAX_WAYPOINTS,
    required_cadence_hz: float = DEFAULT_CADENCE_HZ,
    enforce_freshness: bool = True,
) -> JointTrajectory:
    """Parse one exact v1 payload; no target is returned until every point is valid."""
    if not isinstance(payload, Mapping):
        raise TrajectorySchemaError("invalid_trajectory_payload")
    fields = set(payload)
    if fields != _TOP_LEVEL_FIELDS:
        raise TrajectorySchemaError(
            "missing_trajectory_fields" if _TOP_LEVEL_FIELDS - fields else "unexpected_trajectory_fields"
        )
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise TrajectorySchemaError("unsupported_trajectory_schema")
    if payload.get("type") != MESSAGE_TYPE:
        raise TrajectorySchemaError("invalid_trajectory_type")
    expected_ids = {
        "session_id": expected_session_id,
        "episode_id": expected_episode_id,
        "lease_id": expected_lease_id,
    }
    if any(payload.get(name) != expected for name, expected in expected_ids.items()):
        raise TrajectorySchemaError("lease_mismatch")
    trajectory_id = payload.get("trajectory_id")
    if not isinstance(trajectory_id, str) or _ID.fullmatch(trajectory_id) is None:
        raise TrajectorySchemaError("invalid_trajectory_id")
    dispatched_at = _number(payload.get("dispatched_at"), "invalid_dispatched_at")
    if enforce_freshness:
        current = time.time() if now_s is None else _number(now_s, "invalid_gateway_time")
        if dispatched_at < current - MAX_DISPATCH_AGE_S:
            raise TrajectorySchemaError("stale_trajectory")
        if dispatched_at > current + MAX_FUTURE_SKEW_S:
            raise TrajectorySchemaError("future_trajectory")
    cadence_hz = _number(payload.get("cadence_hz"), "invalid_trajectory_cadence")
    if not math.isclose(cadence_hz, required_cadence_hz, rel_tol=0.0, abs_tol=1e-9):
        raise TrajectorySchemaError("unsupported_trajectory_cadence")
    raw_waypoints = payload.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise TrajectorySchemaError("empty_trajectory")
    if len(raw_waypoints) > max_waypoints:
        raise TrajectorySchemaError("trajectory_too_long")

    waypoints: list[TrajectoryWaypoint] = []
    for index, raw in enumerate(raw_waypoints):
        if not isinstance(raw, Mapping):
            raise TrajectorySchemaError("invalid_waypoint", waypoint=index)
        fields = set(raw)
        required = {"step_id", "left_joints_deg", "right_joints_deg"}
        if not required.issubset(fields):
            raise TrajectorySchemaError("missing_waypoint_fields", waypoint=index)
        if fields - _WAYPOINT_FIELDS:
            raise TrajectorySchemaError("unexpected_waypoint_fields", waypoint=index)
        step_id = raw.get("step_id")
        wanted_step = expected_step_id + index
        if isinstance(step_id, bool) or not isinstance(step_id, int) or step_id != wanted_step:
            raise TrajectorySchemaError(
                "noncontiguous_step_id",
                waypoint=index,
                step_id=step_id if isinstance(step_id, int) and not isinstance(step_id, bool) else None,
            )
        waypoints.append(
            TrajectoryWaypoint(
                step_id=step_id,
                left_joints_deg=_joint_array(raw.get("left_joints_deg"), "left", index),
                right_joints_deg=_joint_array(raw.get("right_joints_deg"), "right", index),
                left_gripper=_optional_gripper(raw, "left_gripper", index),
                right_gripper=_optional_gripper(raw, "right_gripper", index),
            )
        )
    return JointTrajectory(
        session_id=expected_session_id,
        episode_id=expected_episode_id,
        lease_id=expected_lease_id,
        trajectory_id=trajectory_id,
        dispatched_at=dispatched_at,
        cadence_hz=cadence_hz,
        waypoints=tuple(waypoints),
    )


def resolve_gripper_waypoints(
    trajectory: JointTrajectory,
    *,
    left_start: float,
    right_start: float,
    max_delta_per_step: float = MAX_GRIPPER_DELTA_PER_STEP,
) -> tuple[tuple[float, float], ...]:
    """Carry omitted grippers forward and reject a fast close/open before motion."""
    left = _gripper(left_start, "invalid_left_gripper_start")
    right = _gripper(right_start, "invalid_right_gripper_start")
    limit = _number(max_delta_per_step, "invalid_gripper_delta_limit")
    if limit <= 0.0:
        raise TrajectorySchemaError("invalid_gripper_delta_limit")
    resolved: list[tuple[float, float]] = []
    for index, waypoint in enumerate(trajectory.waypoints):
        next_left = left if waypoint.left_gripper is None else waypoint.left_gripper
        next_right = right if waypoint.right_gripper is None else waypoint.right_gripper
        if abs(next_left - left) > limit + 1e-12:
            raise TrajectorySchemaError(
                "left_gripper_delta_limit", waypoint=index, step_id=waypoint.step_id
            )
        if abs(next_right - right) > limit + 1e-12:
            raise TrajectorySchemaError(
                "right_gripper_delta_limit", waypoint=index, step_id=waypoint.step_id
            )
        left, right = next_left, next_right
        resolved.append((left, right))
    return tuple(resolved)


def _joint_array(value: Any, arm: str, waypoint: int) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != JOINTS_PER_ARM:
        raise TrajectorySchemaError(f"invalid_{arm}_joints", waypoint=waypoint)
    try:
        return tuple(_number(item, f"invalid_{arm}_joints") for item in value)
    except TrajectorySchemaError as exc:
        raise TrajectorySchemaError(exc.code, waypoint=waypoint) from exc


def _optional_gripper(raw: Mapping[str, Any], name: str, waypoint: int) -> float | None:
    if name not in raw:
        return None
    try:
        return _gripper(raw[name], f"invalid_{name}")
    except TrajectorySchemaError as exc:
        raise TrajectorySchemaError(exc.code, waypoint=waypoint) from exc


def _gripper(value: Any, code: str) -> float:
    result = _number(value, code)
    if not 0.0 <= result <= 1.0:
        raise TrajectorySchemaError(code)
    return result


def _number(value: Any, code: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TrajectorySchemaError(code)
    result = float(value)
    if not math.isfinite(result):
        raise TrajectorySchemaError(code)
    return result
