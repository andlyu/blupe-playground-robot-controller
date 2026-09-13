#!/usr/bin/env python3
"""Physical-mode operator console with lazy, button-owned i2rt startup.

Home and policy actions intentionally remain unavailable until the physical
trajectory executor is connected to the hardware safety guard.
"""

from __future__ import annotations

import logging
import math
import os
import threading
import time
from pathlib import Path

import mujoco
import numpy as np

from scripts import yam_operator_sim_web as base
from YAM_control.auto_queue import AutoQueue
from YAM_control.shared_feedback import FeedbackUnavailable
from YAM_control.position_fetch import fetch_position, PositionFetchBusy, PositionFetchCancelled
from YAM_control.isolated_adapter import IsolatedAdapter
from YAM_control.hardware_safety import BimanualHardwareGuard, HardwareSafetyConfig, SafetyViolation
from YAM_control.i2rt_bimanual_adapter import (
    AdapterState,
    TrajectoryInterrupted,
    I2RTBimanualAdapter,
    TrajectorySettleTimeout,
    ServoFaultHolding,
    WaypointSafetyRejected,
)
from YAM_control.ruckig_waypath import (
    DEFAULT_MAX_ACCELERATION_RAD_S2,
    DEFAULT_MAX_JERK_RAD_S3,
    MuJoCoWaypathPlanner,
    PlannedWaypath,
    collision_enabled_model,
    plan_joint_trajectory,
)
from YAM_control.joint_trajectory import (
    TrajectorySchemaError,
    parse_joint_trajectory,
    resolve_gripper_waypoints,
)


LEFT_GRIPPER_LIMITS = (6.300923920134496, 1.1532928832839549)
RIGHT_GRIPPER_LIMITS = (6.420424200808728, 1.1495765621423661)
SAFETY_CONFIG = Path(__file__).resolve().parents[1] / "config" / "yam_hardware_safety.json"
HARDWARE_POSE_CONFIG = (
    Path(__file__).resolve().parents[1] / "config" / "yam_operator_hardware_pose.json"
)
LOCAL_CADENCE_HZ = 10.0
WAYPATH_CADENCE_HZ = 20.0
HOME_WAYPOINT_LIMIT_TOLERANCE_RAD = 0.001
LOGGER = logging.getLogger(__name__)
NON_TERMINAL_GUARDRAIL_CODES = {
    "command_endpoint_delta_limit",
    "adjacent_delta_limit",
    "joint_velocity_limit",
    "left_tip_outside_workspace",
    "right_tip_outside_workspace",
    "predicted_collision",
}


def policy_ready(status):
    safety = status.get('safety') or {}
    api = status.get('api') or {}
    return bool(status.get('feedback_available', True) and status.get('mode') in {'READY', 'STOPPED'}
                and status.get('homed') and status.get('settled')
                and safety.get('controller_ok') and safety.get('position_limits')
                and safety.get('drives_enabled') is True
                and not status.get('adapter_fault')
                and api.get('connected') and not api.get('authorized')
                and not api.get('session_id'))

class PhysicalOperator(base.OperatorSimulator):
    startup_title = "YAM physical operator: http://{host}:{port}"
    startup_safety = "PHYSICAL ACTUATION: operator-owned i2rt; Safety Guardrails required"

    def __init__(self, pose_config_path=HARDWARE_POSE_CONFIG, *, adapter=None) -> None:
        super().__init__(pose_config_path)
        self._physical = adapter if adapter is not None else IsolatedAdapter(
            left_channel="can0",
            right_channel="can1",
            left_gripper_limits=LEFT_GRIPPER_LIMITS,
            right_gripper_limits=RIGHT_GRIPPER_LIMITS,
            calibrate_grippers_on_launch=True,
        )
        self._physical_lock = threading.RLock()
        self.policy_settle_tolerance = 0.1
        self.last_run_failure = None
        self._safety_config = HardwareSafetyConfig.load(SAFETY_CONFIG)
        self._safety_model = collision_enabled_model(base.MODEL_PATH)
        self._safety_data = mujoco.MjData(self._safety_model)
        self._safety_guardrails = BimanualHardwareGuard(
            self._safety_config,
            forward_kinematics=self._forward_kinematics,
            collision_check=self._collision_clear,
        )
        self._hardware_pending = None
        self._policy_deadline = None
        self._training_recorder = None
        self._training_error = None
        self._waypath_planner = MuJoCoWaypathPlanner(
            base.MODEL_PATH,
            self._safety_config.joint_lower_rad,
            self._safety_config.joint_upper_rad,
        )
        self._waypath_plan: PlannedWaypath | None = None
        self._waypath_approved = None
        self._waypath_state = "empty"
        self._waypath_error: str | None = None
        self._waypath_preview_started = 0.0
        self._physical_event = "Physical arms disabled; press Launch Arms to initialize"
        self.auto_queue = AutoQueue(self, os.environ.get('YAM_AUTO_QUEUE_STATE'))

    def start(self):
        super().start()
        self.auto_queue.start()

    def _loop(self) -> None:
        """Mirror physical feedback for visualization without owning control state."""
        renderer = None
        period = 0.08
        try:
            renderer = mujoco.Renderer(self.model, height=480, width=720)
            self._log("hardware_monitor", render_period_s=period)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth = 135
            camera.elevation = -22
            camera.distance = 2.0
            camera.lookat[:] = [0.25, 0.0, 0.30]
            while not self.stop_event.is_set():
                started = time.monotonic()
                state = self._physical.state
                if state in {AdapterState.STOPPED, AdapterState.EXECUTING}:
                    try:
                        snapshot = self._physical.snapshot()
                        joints = np.asarray(
                            snapshot.left.joints_rad + snapshot.right.joints_rad,
                            dtype=np.float64,
                        )
                        display_joints = np.clip(joints, self.lower, self.upper)
                        with self.lock:
                            self.data.qpos[:base.N_JOINTS] = display_joints
                            self.data.qvel[:] = 0.0
                            self.data.ctrl[:base.N_JOINTS] = display_joints
                            self.target = display_joints.copy()
                    except Exception as exc:
                        LOGGER.warning("physical visualization feedback unavailable: %s", exc)
                elif self._waypath_plan is not None and self._waypath_state in {"previewed", "validated"}:
                    samples = self._waypath_plan.trajectory.positions
                    elapsed = max(0.0, time.monotonic() - self._waypath_preview_started)
                    preview_index = int(elapsed * self._waypath_plan.trajectory.cadence_hz) % len(samples)
                    preview_joints = np.asarray(samples[preview_index], dtype=np.float64)
                    with self.lock:
                        self.data.qpos[:base.N_JOINTS] = preview_joints
                        self.data.qvel[:] = 0.0
                        self.data.ctrl[:base.N_JOINTS] = preview_joints
                with self.lock:
                    mujoco.mj_forward(self.model, self.data)
                    renderer.update_scene(self.data, camera=camera)
                    rgb = renderer.render()
                    ok, encoded = base.cv2.imencode(
                        ".jpg",
                        base.cv2.cvtColor(rgb, base.cv2.COLOR_RGB2BGR),
                        [base.cv2.IMWRITE_JPEG_QUALITY, 85],
                    )
                    if ok:
                        self.latest_jpeg = encoded.tobytes()
                self.stop_event.wait(max(0.0, period - (time.monotonic() - started)))
        except Exception as exc:
            LOGGER.exception("physical visualization loop failed")
            with self.lock:
                self.render_error = str(exc)
        finally:
            if renderer is not None:
                renderer.close()

    def action(self, action: str) -> str:
        if action == 'auto_queue_on':
            return self.auto_queue.enable()
        if action == 'auto_queue_off':
            return self.auto_queue.pause()
        if action == "stop":
            self.auto_queue.pause('Emergency Stop; automatic queue paused', clear_park=True)
            self._stop_physical("operator_stop")
            return self._physical_event
        if self.auto_queue.snapshot()['running']:
            return 'Automatic startup in progress; use Pause auto queue or Emergency Stop'
        if action in {'launch', 'home', 'park', 'run_policy'}:
            self.auto_queue.pause('Manual control; automatic queue paused')
        with self._physical_lock:
            if action in {'home','park'} and getattr(self._physical, 'link_fault', None):
                try:
                    self._physical.recover_operator_link()
                except Exception as exc:
                    with self.lock:
                        self.event = f'{action.title()} blocked: {exc}'
                        self._physical_event = self.event
                    return self.event
                with self.lock:
                    self.mode = 'STOPPED'
                    self.controller_ok = True
                    self.drives_enabled = True
                    self.launched = True
                    self.policy_phase = 'holding'
            if action == "launch":
                if self._physical.state in {AdapterState.EXECUTING, AdapterState.INITIALIZING}:
                    return "Launch blocked: wait for active playback or initialization to finish"
                try:
                    return self._launch_physical()
                except Exception as exc:
                    fault = self._physical.fault or f"{type(exc).__name__}: {exc}"
                    with self.lock:
                        self.mode = "FAULT"
                        self.command_source = "operator/launch"
                        self.policy_phase = "launch_failed"
                        self.launched = False
                        self.drives_enabled = None
                        self.controller_ok = False
                        self.api_authorized = False
                    self._physical_event = f"Launch failed closed: {fault}"
                    self.event = self._physical_event
                    LOGGER.exception("physical arm launch failed closed")
                    return self._physical_event
            if action == "test_left_wrist_42":
                return self._test_left_wrist_42()
            if action == "home":
                return self._move_home()
            if action == "park":
                return self._park_zero_and_stop()
            if action == "run_policy":
                current = self.status()
                if not policy_ready(current):
                    return "Run Policy blocked: arms must be healthy, settled at Home, and free of an active session"
                with self.lock:
                    self.data.qpos[:base.N_JOINTS] = current['joints']
                    self.mode = "READY"
                return super().action(action)
            return f"Unknown action: {action}"

    def waypath_action(self, payload) -> str:
        action = str(payload.get("action", ""))
        if action == "hold":
            self._waypath_state = "holding"
            self._hold_physical("operator_waypath_hold")
            self._waypath_state = "held"
            return self._physical_event
        if action == "torque_off":
            return self.action("stop")
        with self._physical_lock:
            if action == "preview":
                arm = str(payload.get("arm", ""))
                frame = str(payload.get("frame", "yam_bimanual/base_link"))
                if frame != "yam_bimanual/base_link":
                    raise ValueError("only yam_bimanual/base_link is supported")
                step_m = float(payload.get("step_m", 0.10))
                plan = self._waypath_planner.plan_box_loop(
                    self.home,
                    arm=arm,
                    step_m=step_m,
                    cadence_hz=WAYPATH_CADENCE_HZ,
                )
                if len(plan.trajectory.positions) > self._safety_config.max_waypoints:
                    raise ValueError("generated waypath exceeds the hardware waypoint limit")
                with self.lock:
                    self._waypath_plan = plan
                    self._waypath_approved = None
                    self._waypath_state = "previewed"
                    self._waypath_error = None
                    self._waypath_preview_started = time.monotonic()
                    self.event = (
                        f"MuJoCo preview ready: {arm} arm, {len(plan.trajectory.positions)} "
                        f"samples, {plan.trajectory.duration_s:.2f} s"
                    )
                return self.event
            if action == "validate":
                if self._waypath_plan is None:
                    raise ValueError("Preview must succeed before Validate")
                approved = self._safety_guardrails.approve(
                    self.home,
                    self._waypath_plan.trajectory.positions,
                    cadence_hz=self._waypath_plan.trajectory.cadence_hz,
                )
                with self.lock:
                    self._waypath_approved = approved
                    self._waypath_state = "validated"
                    self._waypath_error = None
                    self.event = (
                        f"Waypath validated: {len(approved.waypoints)} samples and "
                        f"{approved.swept_samples} swept collision checks"
                    )
                return self.event
            if action == "execute":
                return self._start_waypath_execution()
        raise ValueError(f"unknown waypath action: {action}")

    def _start_waypath_execution(self) -> str:
        queue = self.auto_queue.snapshot()
        if queue["enabled"] or queue["running"]:
            raise ValueError("Pause automatic queue before executing a manual waypath")
        if self._waypath_plan is None or self._waypath_approved is None:
            raise ValueError("Preview and Validate must succeed before Execute")
        if self._physical.state != AdapterState.STOPPED:
            raise ValueError("Execute requires initialized, stopped physical arms")
        with self.lock:
            if (self.api_authorized or self.api_session_id is not None
                    or self.api_lease_id is not None or self._hardware_pending is not None):
                raise ValueError("Execute is blocked while Session API owns motion")
            if self._waypath_state == "executing":
                raise ValueError("waypath execution is already active")
        start = self._physical_joints()
        home_error = float(np.max(np.abs(start - self.home)))
        if home_error > self.settle_tolerance:
            raise ValueError(
                f"Move Home before Execute; maximum joint error is {home_error:.4f} rad"
            )
        approved = self._safety_guardrails.approve(
            start,
            self._waypath_plan.trajectory.positions,
            cadence_hz=self._waypath_plan.trajectory.cadence_hz,
        )
        self._waypath_approved = approved
        self._waypath_state = "executing"
        self.mode = "WAYPATH_EXECUTING"
        self.command_source = "operator/waypath"
        self.policy_phase = "waypath_executing"
        self.event = "Executing validated local Ruckig waypath"
        threading.Thread(
            target=self._execute_waypath,
            daemon=True,
            name="yam-ruckig-waypath",
        ).start()
        return self.event

    def _execute_waypath(self) -> None:
        plan = self._waypath_plan
        approved = self._waypath_approved
        assert plan is not None and approved is not None

        def validate_waypoint(index, previous, waypoint):
            checked = self._safety_guardrails.approve(
                previous,
                (waypoint,),
                cadence_hz=plan.trajectory.cadence_hz,
            )
            return checked.waypoints[0]

        try:
            snapshot = self._physical.execute(
                approved.waypoints,
                cadence_hz=plan.trajectory.cadence_hz,
                validate_waypoint=validate_waypoint,
                settle_tolerance_rad=self.settle_tolerance,
                disable_on_settle_timeout=False,
            )
            joints = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
            with self.lock:
                self.data.qpos[:base.N_JOINTS] = joints
                self.data.qvel[:] = 0.0
                self.data.ctrl[:base.N_JOINTS] = joints
                self.target = joints.copy()
                self._waypath_state = "completed"
                self._waypath_error = None
                self.mode = "READY"
                self.command_source = "operator/waypath"
                self.policy_phase = "waypath_complete"
                self.event = "Ruckig waypath completed and both arms settled at Home"
                self._physical_event = self.event
        except TrajectoryInterrupted as exc:
            with self.lock:
                self._waypath_state = "held"
                self._waypath_error = str(exc)
                self.mode = "STOPPED"
                self.policy_phase = "holding"
                self.event = f"Waypath held: {exc}"
                self._physical_event = self.event
        except Exception as exc:
            try:
                self._physical.hold()
            except Exception:
                LOGGER.exception("waypath failure could not retain position hold")
            with self.lock:
                self._waypath_state = "failed"
                self._waypath_error = f"{type(exc).__name__}: {exc}"
                healthy = self._physical.state == AdapterState.STOPPED
                self.mode = "STOPPED" if healthy else "FAULT"
                self.controller_ok = healthy
                self.policy_phase = "waypath_failed"
                self.event = f"Waypath failed closed: {self._waypath_error}"
                self._physical_event = self.event
            LOGGER.exception("local Ruckig waypath failed")

    def _launch_physical(self, *, cancel_event=None) -> str:
        self.auto_queue.invalidate_park()
        snapshot = self._physical.launch(cancel_event=cancel_event)
        joints = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
        display_joints = np.clip(joints, self.lower, self.upper)
        limit_discrepancy = float(np.max(np.abs(display_joints - joints)))
        if limit_discrepancy > 0.005:
            self._physical.stop()
            raise RuntimeError(
                "physical feedback exceeds modeled joint limits by "
                f"{limit_discrepancy:.6f} rad"
            )
        with self.lock:
            self.data.qpos[:base.N_JOINTS] = display_joints
            self.data.qvel[:] = 0.0
            self.data.ctrl[:base.N_JOINTS] = display_joints
            self.target = display_joints.copy()
            self.mode = "STOPPED"
            self.command_source = "operator"
            self.policy_phase = "idle"
            self.launched = True
            self.drives_enabled = True
            self.controller_ok = True
            # Verified communication recovery also applies to manual/Home holds.
            self._physical.recovery_allowed = True
        self._physical_event = (
            "Both physical arms initialized; holding observed pose; "
            "Safety Guardrails active"
        )
        return self._physical_event

    def _authorize_automatic_policy(self):
        if self._physical.state != AdapterState.STOPPED:
            return 'Run Policy blocked: physical arms must be stopped'
        return super().action('run_policy')

    def start_queue_cleanup(self, session_ids):
        if self.auto_queue.snapshot()['running']:
            raise ValueError('Wait for automatic startup to finish before queue cleanup')
        self.auto_queue.pause('Queue cleanup; automatic queue paused')
        return super().start_queue_cleanup(session_ids)

    def api_transport_resume(self):
        # snapshot() checks independently refreshed motor status and feedback;
        # a latched servo fault must never be cleared by a network reconnect.
        self._physical.snapshot()
        super().api_transport_resume()

    def api_connection_changed(self, connected, error=None):
        with self.lock:
            active_motion_authority = bool(
                not connected
                and (self.api_lease_id is not None or self._hardware_pending is not None)
            )
        super().api_connection_changed(connected, error)
        if active_motion_authority:
            self._hold_physical("session_api_disconnect")

    def api_handle_joint_command(self, payload):
        with self.lock:
            reason = None
            if self.mode != "API_ACTIVE" or self.api_lease_id is None:
                reason = "operator_not_authorized"
            elif any(payload.get(name) != expected for name, expected in (
                ("session_id", self.api_session_id),
                ("episode_id", self.api_episode_id),
                ("lease_id", self.api_lease_id),
            )):
                reason = "lease_mismatch"
            step_id = payload.get("step_id")
            command_id = payload.get("command_id")
            if reason is None and (
                isinstance(step_id, bool) or not isinstance(step_id, int) or step_id <= self.api_last_step
            ):
                reason = "stale_or_duplicate_step"
            if reason is None and (not isinstance(command_id, str) or not command_id):
                reason = "missing_command_id"
            arms = (payload.get("left_joints_deg"), payload.get("right_joints_deg"))
            if reason is None and not all(
                isinstance(arm, list) and len(arm) == 6 and all(
                    not isinstance(value, bool)
                    and isinstance(value, (int, float))
                    and np.isfinite(value)
                    for value in arm
                ) for arm in arms
            ):
                reason = "bimanual_command_requires_two_six_joint_arrays"
            if reason is None and self._hardware_pending is not None:
                reason = "command_in_flight"
            if reason is not None:
                self.event = f"API command rejected by Safety Guardrails: {reason}"
                return [self._api_action_result_locked(payload, "rejected", reason)]

        try:
            start = self._physical_joints()
            target = np.deg2rad(np.asarray(arms[0] + arms[1], dtype=np.float64))
            if np.max(np.abs(target - start)) > 0.06 + 1e-12:
                raise SafetyViolation("command_endpoint_delta_limit", waypoint=0)
            waypoints = self._bounded_ramp(start, target)
            approved = self._safety_guardrails.approve(
                start,
                waypoints,
                cadence_hz=LOCAL_CADENCE_HZ,
                target_limit_tolerance_rad=HOME_WAYPOINT_LIMIT_TOLERANCE_RAD,
            )
        except SafetyViolation as exc:
            reason = exc.code
            with self.lock:
                self.event = f"API command rejected by Safety Guardrails: {reason}"
                rejected = self._api_action_result_locked(payload, "rejected", reason)
                target_limit = reason.startswith("joint_") and reason.endswith("_outside_limits")
                if reason in NON_TERMINAL_GUARDRAIL_CODES or target_limit:
                    return [rejected]
                self.mode = "FAULT"
                self.controller_ok = False
                self.api_authorized = False
                aborted = self._api_envelope_locked(
                    "safety_abort",
                    code="safety_guardrails_fault",
                    message="Required hardware safety validation is unavailable",
                    step_id=payload.get("step_id"),
                    observed_at=time.time(),
                    details={"component": "safety_guardrails", "violation_code": reason},
                )
            self._physical.stop()
            return [rejected, aborted]
        except Exception as exc:
            reason = f"safety_guardrails_unavailable:{type(exc).__name__}"
            with self.lock:
                self.event = f"FAULT: {reason}"
                self.mode = "FAULT"
                self.controller_ok = False
                self.api_authorized = False
                rejected = self._api_action_result_locked(payload, "rejected", reason)
                aborted = self._api_envelope_locked(
                    "safety_abort",
                    code="safety_guardrails_unavailable",
                    message="Safety Guardrails could not validate the physical command",
                    step_id=payload.get("step_id"),
                    observed_at=time.time(),
                    details={"component": "safety_guardrails", "error_type": type(exc).__name__},
                )
            self._physical.stop()
            return [rejected, aborted]

        with self.lock:
            if self.mode != "API_ACTIVE" or self.api_lease_id != payload.get("lease_id"):
                return [self._api_action_result_locked(payload, "rejected", "authority_revoked")]
            self.api_last_step = step_id
            self._hardware_pending = {
                "cancel_event": threading.Event(),
                "session_id": self.api_session_id,
                "episode_id": self.api_episode_id,
                "lease_id": self.api_lease_id,
                "step_id": step_id,
                "command_id": command_id,
            }
            self.policy_phase = f"executing_step_{step_id}"
            self.event = f"API command step {step_id} accepted by Safety Guardrails"
        threading.Thread(
            target=self._execute_api_command,
            args=(approved,),
            daemon=True,
            name=f"yam-hardware-step-{step_id}",
        ).start()
        return []

    def api_handle_joint_trajectory(self, payload):
        """Accept a structural envelope, then safety-check each 10 Hz waypoint."""
        with self.lock:
            if self.mode != "API_ACTIVE" or self.api_lease_id is None:
                return [self._api_raw_trajectory_result_locked(payload, "rejected", "operator_not_authorized")]
            trajectory_id = payload.get("trajectory_id")
            existing = (
                self.api_trajectory_records.get(trajectory_id)
                if isinstance(trajectory_id, str)
                else None
            )
            expected_step = (
                existing["trajectory"].first_step_id
                if existing is not None
                else self.api_last_step + 1
            )
            try:
                trajectory = parse_joint_trajectory(
                    payload,
                    expected_session_id=self.api_session_id,
                    expected_episode_id=self.api_episode_id,
                    expected_lease_id=self.api_lease_id,
                    expected_step_id=expected_step,
                    max_waypoints=self._safety_config.max_waypoints,
                    required_cadence_hz=LOCAL_CADENCE_HZ,
                    enforce_freshness=existing is None,
                )
            except TrajectorySchemaError as exc:
                self.event = f"API trajectory rejected by Safety Guardrails: {exc.code}"
                return [self._api_raw_trajectory_result_locked(payload, "rejected", exc.code)]
            if existing is not None:
                if existing["fingerprint"] != trajectory.fingerprint:
                    return [self._api_trajectory_result_locked(
                        trajectory,
                        "rejected",
                        code="trajectory_id_conflict",
                        message="trajectory_id was reused with different content",
                        details={},
                    )]
                return [self._api_trajectory_record_result_locked(existing)]
            if self._hardware_pending is not None:
                return [self._api_trajectory_result_locked(
                    trajectory,
                    "rejected",
                    code="command_in_flight",
                    message="Another command is already active",
                    details={},
                )]

        def still_authorized():
            with self.lock:
                return self.mode == "API_ACTIVE" and self.api_lease_id == payload.get("lease_id")

        try:
            snapshot = fetch_position(self._physical.snapshot, still_authorized,
                on_retry=lambda attempt: LOGGER.warning(
                    "[position-fetch] worker busy; retry=%d/3 delay_ms=500 trajectory=%s",
                    attempt, trajectory.trajectory_id))
            start = np.asarray(
                snapshot.left.joints_rad + snapshot.right.joints_rad,
                dtype=np.float64,
            )
            grippers = resolve_gripper_waypoints(
                trajectory,
                left_start=snapshot.left.gripper,
                right_start=snapshot.right.gripper,
            )
            # Reject an unsafe later waypoint before accepting any of the packet.
            # Runtime validation remains in the adapter dispatch loop as well.
            try:
                self._safety_guardrails.approve(
                    start,
                    tuple(point.joints_rad for point in trajectory.waypoints),
                    cadence_hz=trajectory.cadence_hz,
                    target_limit_tolerance_rad=HOME_WAYPOINT_LIMIT_TOLERANCE_RAD,
                )
            except SafetyViolation as exc:
                raise TrajectorySchemaError(exc.code, waypoint=exc.waypoint) from exc
        except PositionFetchCancelled:
            with self.lock:
                return [self._api_trajectory_result_locked(
                    trajectory, "rejected", code="authority_revoked",
                    message="Authority revoked while fetching joint positions", details={})]
        except TrajectorySchemaError as exc:
            reason = exc.code
            with self.lock:
                self.event = f"API trajectory rejected by Safety Guardrails: {reason}"
                rejected = self._api_trajectory_result_locked(
                    trajectory,
                    "rejected",
                    step_id=getattr(exc, "step_id", None),
                    code=reason,
                    message="Trajectory rejected before motion by Safety Guardrails",
                    details={"waypoint": exc.waypoint} if exc.waypoint is not None else {},
                )
            return [rejected]
        except Exception as exc:
            busy = isinstance(exc, PositionFetchBusy)
            code = "position_fetch_busy" if busy else "safety_guardrails_unavailable"
            message = str(exc) if busy else "Safety Guardrails could not validate the physical trajectory"
            reason = message if busy else f"{code}:{type(exc).__name__}"
            with self.lock:
                self.event = f"FAULT: {reason}"
                self.mode = "FAULT"
                self.controller_ok = False
                self.api_authorized = False
                rejected = self._api_trajectory_result_locked(
                    trajectory,
                    "rejected",
                    code=code,
                    message=message,
                    details={"component": "position_fetch" if busy else "safety_guardrails", "error_type": type(exc).__name__},
                )
                aborted = self._api_envelope_locked(
                    "safety_abort",
                    code=code,
                    message=message,
                    step_id=None,
                    observed_at=time.time(),
                    details={"component": "position_fetch" if busy else "safety_guardrails", "error_type": type(exc).__name__},
                )
            self._physical.stop()
            return [rejected, aborted]

        with self.lock:
            if self.mode != "API_ACTIVE" or self.api_lease_id != payload.get("lease_id"):
                return [self._api_trajectory_result_locked(
                    trajectory, "rejected", code="authority_revoked",
                    message="Authority revoked during trajectory validation", details={},
                )]
            record = {
                "trajectory": trajectory,
                "fingerprint": trajectory.fingerprint,
                "status": "accepted",
                "step_id": None,
                "code": None,
                "message": None,
                "details": None,
                "progress_count": 0,
            }
            self.api_trajectory_records[trajectory.trajectory_id] = record
            while len(self.api_trajectory_records) > 32:
                self.api_trajectory_records.pop(next(iter(self.api_trajectory_records)))
            self.api_last_step = trajectory.last_step_id
            self._hardware_pending = {
                "cancel_event": threading.Event(),
                "session_id": self.api_session_id,
                "episode_id": self.api_episode_id,
                "lease_id": self.api_lease_id,
                "trajectory": trajectory,
                "grippers": grippers,
                "record": record,
            }
            self.policy_phase = "trajectory_buffered"
            self.event = (
                f"API trajectory {trajectory.trajectory_id} buffered; each of "
                f"{len(trajectory.waypoints)} points checked and sent at {trajectory.cadence_hz:g} Hz"
            )
            accepted = self._api_trajectory_record_result_locked(record)
            client = self.api_client
        if client is not None:
            client.send(accepted)
            responses = []
        else:
            responses = [accepted]
        threading.Thread(
            target=self._execute_api_trajectory,
            args=(trajectory, trajectory.trajectory_id),
            daemon=True,
            name=f"yam-hardware-trajectory-{trajectory.trajectory_id}",
        ).start()
        return responses

    def api_prepare_session(self, payload):
        duration = payload.get("run_duration_s", 300)
        if type(duration) is not int or not 60 <= duration <= 600:
            raise ValueError("run_duration_s must be an integer from 60 to 600")
        with self._physical_lock:
            with self.auto_queue.lock:
                if self.auto_queue.ready_deadline is not None and self.auto_queue.cancel.is_set():
                    return None
                observation = super().api_prepare_session(payload)
                if observation is not None:
                    self.auto_queue.ready_deadline = None
            if observation is not None:
                self._physical.recovery_allowed = True
                self._start_training_recording(payload['episode_id'], payload.get('task'))
                with self.lock:
                    token = object()
                    self._policy_duration_s = duration
                    self._policy_deadline = (token, time.monotonic() + duration)
                threading.Thread(target=self._policy_watchdog, args=(token,),
                                 daemon=True, name="yam-policy-deadline").start()
            return observation

    def _start_training_recording(self, episode_id, task=None):
        root = os.environ.get('YAM_DATASET_ROOT')
        if not root:
            return
        try:
            from YAM_control.training_recorder import EpisodeRecorder
            if self._training_recorder:
                self._training_recorder.finish('superseded')
            def snapshot():
                return self._physical.snapshot(), self._physical.last_command
            self._training_recorder = EpisodeRecorder(root, episode_id, snapshot, simulated=getattr(self, 'training_simulated', False), task=task)
            self._training_recorder.start()
            self._training_error = None
        except Exception as exc:
            self._training_error = type(exc).__name__
            LOGGER.warning('Training recorder unavailable: %s', self._training_error)

    def _finish_training_recording(self, reason, diagnostic_error=None):
        recorder = self._training_recorder
        if recorder:
            if not recorder.stop_event.is_set():
                try:
                    trace = self._physical.diagnostic_trace()
                    trace.update(episode_id=recorder.meta['episode_id'], reason=reason, error=diagnostic_error)
                    def save_trace():
                        try:
                            from YAM_control.training_recorder import atomic_json
                            recorder.path.mkdir(parents=True,exist_ok=True)
                            atomic_json(recorder.path/'failure-summary.json',
                                        {'error': diagnostic_error, 'reason': reason})
                            atomic_json(recorder.path/'motor-trace.json',trace)
                        except Exception:
                            LOGGER.exception('[diagnostics] unable to save motor trace')
                    threading.Thread(target=save_trace,daemon=True,name='yam-save-motor-trace').start()
                except Exception:
                    LOGGER.exception('[diagnostics] unable to capture motor trace')
            recorder.finish(reason)

    def _policy_watchdog(self, token):
        # Independent of model requests, browser rendering and cloud connectivity.
        while not self.stop_event.is_set():
            with self.lock:
                deadline = self._policy_deadline
                if deadline is None or deadline[0] is not token:
                    return
                remaining = deadline[1] - time.monotonic()
            if remaining <= 0:
                self._expire_policy_runtime(token)
                return
            self.stop_event.wait(min(.2, remaining))

    def _expire_policy_runtime(self, token):
        with self._physical_lock:
            with self.lock:
                if self._policy_deadline is None or self._policy_deadline[0] is not token:
                    return
                self._policy_deadline = None
                notification = self._api_envelope_locked(
                    "safety_abort", code="policy_runtime_timeout",
                    message="Run duration limit reached; parking at zero and disabling torque",
                    step_id=max(0, self.api_last_step), observed_at=time.time(),
                    details={"timeout_s": getattr(self, "_policy_duration_s", 300), "owner": "jetson"},
                ) if self.api_lease_id is not None else None
            self._hold_physical("session_timeout")
            # Reporting is best-effort; physical cleanup never waits on AWS.
            if notification is not None and self.api_client is not None:
                try:
                    self.api_client.send(notification)
                except Exception:
                    LOGGER.warning("policy timeout notification unavailable")
            self._park_after_policy_timeout()

    def api_handle_stop(self, payload):
        with self.lock:
            if payload.get("session_id") != self.api_session_id or payload.get("lease_id") != self.api_lease_id:
                return
            deadline = self._policy_deadline
            faulted = self.mode == "FAULT" or self._physical.fault is not None
        if faulted:
            # A terminal network message cannot clear a latched hardware fault.
            self.auto_queue.pause('Controller fault; automatic queue paused', clear_park=True)
            self._finish_training_recording("physical_execution_failure")
            with self.lock:
                self._policy_deadline = None
                self.api_authorized = False
                self._clear_api_lease_locked()
            return
        if payload.get("reason") == "session_timeout" and deadline is not None:
            # Server expiry and the local watchdog share one guarded cleanup.
            threading.Thread(target=self._expire_policy_runtime, args=(deadline[0],),
                             daemon=True, name="yam-policy-timeout-park").start()
        else:
            # Preserve the recorded terminal reason before revoking the lease.
            self._hold_physical(payload.get("reason") or "session_api_stop")
            if self.auto_queue.enabled or payload.get("reason") == "policy_complete":
                with self.lock:
                    self._policy_deadline = None
                self.auto_queue.policy_completed()

    def _park_after_policy_timeout(self):
        with self._physical_lock:
            # hold() revokes dispatch immediately, but the executor exits on its
            # next cancellation check. Do not race its EXECUTING -> STOPPED handoff.
            deadline = time.monotonic() + 3.0
            while self._physical.state == AdapterState.EXECUTING and time.monotonic() < deadline:
                time.sleep(0.01)
            result = self._park_zero_and_stop()
            with self.lock:
                self.event = f"Policy runtime ended: selected duration limit reached. {result}"
                self._physical_event = self.event
                self._log("policy_runtime_timeout", timeout_s=getattr(self, "_policy_duration_s", 300), mode=self.mode,
                          torque_off_verified=self._physical.torque_off_verified)

    def _record_run_failure(self, exc, step_id):
        self._finish_training_recording("physical_execution_failure", {"error_type":type(exc).__name__, "message":str(exc), "step_id":step_id})
        # Retain the failure across hold/park and later status messages.
        self.last_run_failure = {
            "timestamp": time.time(), "episode_id": self.api_episode_id,
            "step_id": step_id, "error_type": type(exc).__name__,
            "message": str(exc),
        }
        self.event = f"Run failed: {type(exc).__name__}: {exc}"
        self._physical_event = self.event
        LOGGER.error("%s", self.event)

    def _execute_api_command(self, approved):
        with self.lock:
            pending = self._hardware_pending
            if pending is None:
                return
            cancel_event = pending["cancel_event"]
        try:
            snapshot = self._physical.execute(
                approved.waypoints,
                cadence_hz=approved.cadence_hz,
                settle_tolerance_rad=self.policy_settle_tolerance,
                disable_on_settle_timeout=False,
                cancel_event=cancel_event,
            )
            joints = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
            with self.lock:
                completed = self._hardware_pending
                if completed is None:
                    return
                self.data.qpos[:base.N_JOINTS] = joints
                self.data.qvel[:] = 0.0
                self.data.ctrl[:base.N_JOINTS] = joints
                self.target = joints.copy()
                self._hardware_pending = None
                self.api_observation_step += 1
                self.policy_phase = "waiting_for_command"
                self.event = f"API command step {completed['step_id']} executed and physically settled"
                if self.api_client is not None:
                    self.api_client.send(self._api_action_result_locked(completed, "executed", "settled"))
                    self.api_client.send(self._api_observation_locked())
        except TrajectoryInterrupted:
            return
        except Exception as exc:
            with self.lock:
                pending = self._hardware_pending
                self._hardware_pending = None
                self.mode = "FAULT"
                self.controller_ok = False
                self._record_run_failure(exc, pending.get("step_id") if pending else None)
                if pending is not None and self.api_client is not None:
                    self.api_client.send(self._api_envelope_locked(
                        "safety_abort",
                        code="physical_execution_failure",
                        message=str(exc) if isinstance(exc, (TrajectorySettleTimeout, ServoFaultHolding)) else "Physical arm execution failed and motion was stopped",
                        step_id=pending.get("step_id"),
                        observed_at=time.time(),
                        details={"component": "i2rt_executor", "error_type": type(exc).__name__},
                    ))

            self._recover_joint_settle_fault(exc)

    def _execute_api_trajectory(self, trajectory, trajectory_id):
        def progress(index):
            with self.lock:
                pending = self._hardware_pending
                if pending is None or pending["trajectory"].trajectory_id != trajectory_id:
                    return
                trajectory = pending["trajectory"]
                point = trajectory.waypoints[index]
                record = pending["record"]
                record["progress_count"] = index + 1
                self.policy_phase = f"trajectory_step_{point.step_id}"
                client = self.api_client
                message = self._api_trajectory_progress_locked(
                    trajectory, point.step_id, index + 1
                )
            if client is not None:
                client.send(message)

        try:
            with self.lock:
                pending = self._hardware_pending
                if pending is None or pending["trajectory"].trajectory_id != trajectory_id:
                    return
                grippers = pending["grippers"]
                cancel_event = pending["cancel_event"]

            def validate_waypoint(index, previous, waypoint):
                try:
                    approved = self._safety_guardrails.approve(
                        np.asarray(previous, dtype=np.float64),
                        (waypoint,),
                        cadence_hz=trajectory.cadence_hz,
                        target_limit_tolerance_rad=HOME_WAYPOINT_LIMIT_TOLERANCE_RAD,
                    )
                except SafetyViolation as exc:
                    target_limit = exc.code.startswith("joint_") and exc.code.endswith("_outside_limits")
                    if exc.code in NON_TERMINAL_GUARDRAIL_CODES or target_limit:
                        raise WaypointSafetyRejected(index, exc.code) from exc
                    raise
                return tuple(approved.waypoints[0])

            snapshot = self._physical.execute(
                tuple(point.joints_rad for point in trajectory.waypoints),
                cadence_hz=trajectory.cadence_hz,
                gripper_waypoints=grippers,
                on_waypoint=progress,
                validate_waypoint=validate_waypoint,
                settle_tolerance_rad=self.policy_settle_tolerance,
                disable_on_settle_timeout=False,
                cancel_event=cancel_event,
            )
            joints = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
            with self.lock:
                completed = self._hardware_pending
                if completed is None or completed["trajectory"].trajectory_id != trajectory_id:
                    return
                trajectory = completed["trajectory"]
                record = completed["record"]
                self.data.qpos[:base.N_JOINTS] = joints
                self.data.qvel[:] = 0.0
                self.data.ctrl[:base.N_JOINTS] = joints
                self.target = joints.copy()
                self._hardware_pending = None
                self.api_observation_step = trajectory.last_step_id + 1
                self.policy_phase = "waiting_for_command"
                self.event = (
                    f"API trajectory {trajectory_id} executed once and physically settled"
                )
                record["status"] = "completed"
                client = self.api_client
                result = self._api_trajectory_record_result_locked(record)
                observation = self._api_observation_locked()
            if client is not None:
                client.send(observation)
                client.send(result)
        except WaypointSafetyRejected as exc:
            self._physical.diagnostic_event("safety_rejected", code=exc.code, waypoint=exc.waypoint)
            with self.lock:
                pending = self._hardware_pending
                if pending is None or pending["trajectory"].trajectory_id != trajectory_id:
                    return
                trajectory = pending["trajectory"]
                record = pending["record"]
                point = trajectory.waypoints[exc.waypoint]
                record["status"] = "aborted"
                record["step_id"] = point.step_id
                record["code"] = exc.code
                record["message"] = "Waypoint rejected before driver dispatch by Safety Guardrails"
                record["details"] = {"waypoint": exc.waypoint}
                self._hardware_pending = None
                self.policy_phase = "waiting_for_command"
                self.event = (
                    f"API trajectory stopped before unsafe step {point.step_id}: {exc.code}"
                )
                client = self.api_client
                result = self._api_trajectory_record_result_locked(record)
            if client is not None:
                client.send(result)
        except TrajectoryInterrupted:
            return
        except Exception as exc:
            if isinstance(exc, TrajectorySettleTimeout):
                try:
                    self._return_settle_to_model(trajectory_id, exc)
                    return
                except TrajectoryInterrupted:
                    return  # Stop, lease cancellation and the runtime watchdog win.
                except Exception as hold_error:
                    exc = hold_error  # Unhealthy/unverified holds stay terminal.
            with self.lock:
                pending = self._hardware_pending
                if pending is None or pending["trajectory"].trajectory_id != trajectory_id:
                    return
                trajectory = pending["trajectory"]
                record = pending["record"]
                failed_step = (
                    trajectory.first_step_id + record["progress_count"]
                    if record["progress_count"] < len(trajectory.waypoints)
                    else trajectory.last_step_id
                )
                record["status"] = "aborted"
                record["step_id"] = failed_step
                record["code"] = "physical_execution_failure"
                record["message"] = str(exc) if isinstance(exc, (TrajectorySettleTimeout, ServoFaultHolding)) else "Physical trajectory execution failed and motion was stopped"
                record["details"] = {
                    "component": "i2rt_executor",
                    "error_type": type(exc).__name__,
                }
                self._hardware_pending = None
                self.mode = "FAULT"
                self.controller_ok = False
                self.api_authorized = False
                self._record_run_failure(exc, failed_step)
                client = self.api_client
                result = self._api_trajectory_record_result_locked(record)
                aborted = self._api_envelope_locked(
                    "safety_abort",
                    code="physical_execution_failure",
                    message=record["message"],
                    step_id=failed_step,
                    observed_at=time.time(),
                    details={"component": "i2rt_executor", "error_type": type(exc).__name__},
                )
            try:
                self._physical.hold()
            except Exception:
                LOGGER.exception("physical trajectory failure could not verify position hold")
            if client is not None:
                client.send(result)
                client.send(aborted)
            self._recover_joint_settle_fault(exc)

    def _return_settle_to_model(self, trajectory_id, error):
        with self.lock:
            pending = self._hardware_pending
            if (pending is None or pending['trajectory'].trajectory_id != trajectory_id
                    or pending['cancel_event'].is_set() or not self.api_authorized):
                raise TrajectoryInterrupted('Settling recovery no longer owns the run')
            trajectory = pending['trajectory']
            record = pending['record']
            if record['progress_count'] != len(trajectory.waypoints):
                raise RuntimeError('Settling recovery requires all waypoint progress')
            snapshot = self._physical.hold_measured(cancel_event=pending['cancel_event'])
            joints = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
            self.data.qpos[:base.N_JOINTS] = joints
            self.data.qvel[:] = 0.0
            self.data.ctrl[:base.N_JOINTS] = joints
            self.target = joints.copy()
            record.update(status='aborted', step_id=trajectory.last_step_id,
                          code='joint_settle_timeout', message=str(error),
                          details={'component':'i2rt_executor', 'error_type':'TrajectorySettleTimeout',
                                   'recoverable':True, 'hold':'measured_position'})
            self._hardware_pending = None
            self.api_observation_step = trajectory.last_step_id + 1
            self.policy_phase = 'waiting_for_command'
            self.event = 'Joint target missed; holding measured position and asking the model to revise'
            client = self.api_client
            result = self._api_trajectory_record_result_locked(record)
            observation = self._api_observation_locked()
        self._physical.diagnostic_event('joint_settle_replan', message=str(error))
        if client is not None:
            # Abort releases the packet in the API; only then publish the new cursor.
            client.send(result)
            client.send(observation)

    def _test_left_wrist_42(self):
        import json
        with self.lock:
            if self.mode != 'READY' or self.api_lease_id is not None or self.api_authorized:
                return 'Wrist test blocked: move Home first, with no active session'
        if self._physical.state != AdapterState.STOPPED:
            return 'Wrist test blocked: driver not stopped'
        start = self._physical_joints()
        target = start.copy()
        target[5] = math.radians(42.1)
        waypoints = self._bounded_ramp(start, target)
        approved = self._safety_guardrails.approve(start, waypoints, cadence_hz=LOCAL_CADENCE_HZ)
        path = Path(__file__).resolve().parents[1] / 'outputs' / ('wrist-test-'+str(int(time.time()))+'.jsonl')
        path.parent.mkdir(parents=True, exist_ok=True)
        done = threading.Event()
        def record():
            with path.open('w') as f:
                while not done.is_set():
                    try:
                        snap = self._physical.snapshot().left
                        f.write(json.dumps({'at':time.time(),'joints':snap.joints_rad,
                            'effort':snap.joint_effort,'velocity':snap.joint_velocity,'target_joint_5':float(target[5])})+'\n')
                        f.flush()
                    except Exception as error:
                        f.write(json.dumps({'error':type(error).__name__})+'\n')
                    done.wait(.1)
        recorder = threading.Thread(target=record,daemon=True)
        recorder.start()
        with self.lock:
            self.mode='WRIST_TEST'
        try:
            snapshot = self._physical.execute(approved.waypoints,cadence_hz=approved.cadence_hz,
                settle_tolerance_rad=self.policy_settle_tolerance,disable_on_settle_timeout=False)
            with self.lock:
                self.mode='STOPPED'
            message = f'Wrist test reached {math.degrees(snapshot.left.joints_rad[5]):.1f} degrees; target 42.1 degrees. Telemetry: {path.name}'
        except Exception as exc:
            with self.lock:
                self.mode='FAULT'
                self._record_run_failure(exc,None)
            message = f'Wrist test stopped: {exc}. Telemetry: {path.name}'
        finally:
            done.set();recorder.join(timeout=2)
        self.event = self._physical_event = message
        return message

    def _report_idle_servo_fault(self, exc):
        # A fault while Astra is thinking has no trajectory executor to report it.
        with self.lock:
            if self._hardware_pending is not None:
                return
            manual = self.api_lease_id is None
        if manual:
            self._mark_servo_fault_holding(exc)
            return
        with self.lock:
            if self.api_lease_id is None or self._hardware_pending is not None:
                return
            self._physical.recovery_allowed = False
            self._record_run_failure(exc, None)
            client = self.api_client
            aborted = self._api_envelope_locked(
                "safety_abort", code="physical_execution_failure", message=str(exc),
                step_id=None, observed_at=time.time(),
                details={"component":"i2rt_feedback", "error_type":type(exc).__name__})
            self.api_authorized = False
            self._policy_deadline = None
            self._clear_api_lease_locked()
        self._mark_servo_fault_holding(exc)
        if client is not None:
            client.send(aborted)

    def _mark_servo_fault_holding(self, exc):
        self.auto_queue.pause("Servo fault; operator recovery required", clear_park=True)
        with self.lock:
            self.mode = "FAULT"
            self.controller_ok = False
            self.event = str(exc) + "; healthy servos holding; operator recovery required"
            self._physical_event = self.event

    def _recover_joint_settle_fault(self, exc):
        if not isinstance(exc, TrajectorySettleTimeout):
            return
        with self._physical_lock:
            with self.lock:
                if self.mode != 'FAULT' or self._physical.state != AdapterState.STOPPED:
                    return
                cancel = threading.Event()
                self._fault_recovery_cancel = cancel
            self.auto_queue.pause('Joint mismatch; returning to zero, queue paused', clear_park=True)
            try:
                # No new lease may survive the failed trajectory. Readiness stays off.
                self._hold_physical('joint_settle_recovery')
                if cancel.is_set():
                    return
                if self._physical.state != AdapterState.STOPPED:
                    self._stop_physical('fault_recovery_unhealthy')
                    return
                self._park_zero_and_stop(cancel_event=cancel)
            except TrajectoryInterrupted:
                return
            except Exception:
                LOGGER.exception('Joint mismatch recovery failed')
                self._stop_physical('fault_recovery_failed')
            finally:
                with self.lock:
                    self._fault_recovery_cancel = None

    def _park_zero_and_stop(self, *, cancel_event=None):
        with self.lock:
            if self.mode not in {"STOPPED", "READY"} or self.api_lease_id is not None:
                return (
                    "Park at Zero blocked: arms must be initialized and idle; "
                    "use Emergency Stop if motion is active"
                )
        if self._physical.state != AdapterState.STOPPED:
            return (
                "Park at Zero blocked: arms must be initialized and idle; "
                "use Emergency Stop if motion is active"
            )

        start = self._physical_joints()
        target = np.zeros(base.N_JOINTS, dtype=np.float64)
        legal_start = np.clip(
            start,
            self._safety_config.joint_lower_rad,
            self._safety_config.joint_upper_rad,
        )
        waypoints = self._bounded_ramp(legal_start, target)
        if not np.array_equal(start, legal_start):
            waypoints = (tuple(float(value) for value in legal_start), *waypoints)
        try:
            approved = self._safety_guardrails.approve(
                start,
                waypoints,
                cadence_hz=LOCAL_CADENCE_HZ,
                target_limit_tolerance_rad=HOME_WAYPOINT_LIMIT_TOLERANCE_RAD,
            )
        except Exception as exc:
            message = f"Park at Zero rejected before motion: {type(exc).__name__}: {exc}"
            LOGGER.error(message, exc_info=True)
            self._stop_physical("park_validation_failure")
            with self.lock:
                self.event = f"{message}; {self._physical_event}"
                self._physical_event = self.event
            return self.event

        try:
            with self.lock:
                self.mode = "PARKING_ZERO"
                self.command_source = "operator"
                self.policy_phase = "parking_at_zero"
                self.event = "Parking both physical arms at joint zero"
                self._physical_event = self.event
            snapshot = self._physical.execute(
                approved.waypoints,
                cadence_hz=approved.cadence_hz,
                settle_tolerance_rad=self.settle_tolerance,
                cancel_event=cancel_event,
            )
            joints = np.asarray(
                snapshot.left.joints_rad + snapshot.right.joints_rad,
                dtype=np.float64,
            )
            with self.lock:
                self.data.qpos[:base.N_JOINTS] = np.clip(joints, self.lower, self.upper)
                self.data.qvel[:] = 0.0
                self.data.ctrl[:base.N_JOINTS] = self.data.qpos[:base.N_JOINTS]
                self.target = self.data.qpos[:base.N_JOINTS].copy()
        except TrajectoryInterrupted:
            raise
        except Exception as exc:
            message = f"Park at Zero failed: {type(exc).__name__}: {exc}"
            LOGGER.error(message, exc_info=True)
            if isinstance(exc, ServoFaultHolding):
                self._mark_servo_fault_holding(exc)
            else:
                self._stop_physical("park_execution_failure")
            with self.lock:
                self.event = f"{message}; {self._physical_event}"
                self._physical_event = self.event
            return self.event

        self._stop_physical("operator_park_zero")
        # The receipt is written only after settled zero feedback AND verified off.
        # Stop cancellation must not be overwritten by a late park completion.
        with self.auto_queue.lock:
            if (self.mode == 'DISABLED' and self._physical.torque_off_verified
                    and (cancel_event is None or not cancel_event.is_set())
                    and np.max(np.abs(joints)) <= self.settle_tolerance):
                self.auto_queue.mark_parked()
        with self.lock:
            if self.mode == "DISABLED":
                self.policy_phase = "parked"
                self.event = "Both physical arms parked at joint zero and torque-disabled"
            self._physical_event = self.event
        return self.event

    def _move_home(self, *, cancel_event=None):
        if self._physical.state != AdapterState.STOPPED:
            return "Move Home blocked: launch both physical arms first"
        initial = self._physical.snapshot()
        start = np.asarray(initial.left.joints_rad + initial.right.joints_rad)
        # Match RoboCurve inspect-robots-yam's _ramp_to/_send behavior: a
        # measured calibration offset may seed the ramp, but every commanded
        # point is brought inside the model limits. The explicit first point
        # below makes that correction visible to our swept-path validator.
        legal_start = np.clip(
            start,
            self._safety_config.joint_lower_rad,
            self._safety_config.joint_upper_rad,
        )
        waypoints = self._bounded_ramp(legal_start, self.home)
        if not np.array_equal(start, legal_start):
            waypoints = (
                tuple(float(value) for value in legal_start),
                *waypoints,
            )
        # Normalized driver position 1.0 is fully open. Pad short/already-home
        # paths so opening never exceeds the adapter's 0.1-per-step limit.
        gripper_start = np.asarray((initial.left.gripper, initial.right.gripper), dtype=float)
        opening_steps = max(1, math.ceil(float(np.max(np.abs(1.0 - gripper_start))) / 0.1))
        waypoints += (waypoints[-1],) * max(0, opening_steps - len(waypoints))
        gripper_waypoints = tuple(
            tuple(float(value) for value in gripper_start + (1.0 - gripper_start) * (index / len(waypoints)))
            for index in range(1, len(waypoints) + 1)
        )
        try:
            approved = self._safety_guardrails.approve(
                start,
                waypoints,
                cadence_hz=LOCAL_CADENCE_HZ,
                target_limit_tolerance_rad=HOME_WAYPOINT_LIMIT_TOLERANCE_RAD,
            )
        except Exception as exc:
            message = f"Move Home rejected before motion: {type(exc).__name__}: {exc}"
            LOGGER.error(message, exc_info=True)
            with self.lock:
                self.mode = "STOPPED"
                self.command_source = "operator"
                self.policy_phase = "home_rejected"
                self.event = message
                self._physical_event = message
            return message

        try:
            with self.lock:
                self.mode = "MOVING_HOME"
                self.command_source = "operator"
                self.policy_phase = "moving_home"
                self.event = "Moving both physical arms to Home and opening both grippers"
                self._physical_event = self.event
            snapshot = self._physical.execute(
                approved.waypoints,
                cadence_hz=approved.cadence_hz,
                gripper_waypoints=gripper_waypoints,
                settle_tolerance_rad=self.settle_tolerance,
                disable_on_settle_timeout=False,
                cancel_event=cancel_event,
            )
        except TrajectoryInterrupted:
            raise
        except TrajectorySettleTimeout as exc:
            if cancel_event is not None and cancel_event.is_set():
                raise TrajectoryInterrupted('Home cancelled before measured hold') from exc
            try:
                snapshot = self._physical.hold_measured(cancel_event=cancel_event)
            except TrajectoryInterrupted:
                raise
            except Exception as hold_error:
                self._mark_servo_fault_holding(hold_error)
                return self.event
            joints = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
            with self.lock:
                self.data.qpos[:base.N_JOINTS] = joints
                self.data.qvel[:] = 0.0
                self.data.ctrl[:base.N_JOINTS] = joints
                self.target = joints.copy()
                self.mode = "STOPPED"
                self.command_source = "home_settle_recovery"
                self.policy_phase = "holding"
                self.event = f"Home target missed; holding measured position: {exc}"
                self._physical_event = self.event
            return self.event
        except Exception as exc:
            message = f"Move Home execution failed: {type(exc).__name__}: {exc}"
            LOGGER.error(message, exc_info=True)
            with self.lock:
                self.event = message
                self._physical_event = message
            if isinstance(exc, ServoFaultHolding):
                self._mark_servo_fault_holding(exc)
            else:
                self._stop_physical("home_execution_failure")
            with self.lock:
                self.event = message
                self._physical_event = message
            return message
        joints = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
        with self.lock:
            self.data.qpos[:base.N_JOINTS] = joints
            self.data.qvel[:] = 0.0
            self.data.ctrl[:base.N_JOINTS] = joints
            self.target = joints.copy()
            self.mode = "READY"
            self.policy_phase = "home_settled"
            self.event = "Both physical arms settled at Home; Safety Guardrails active"
            self._physical_event = self.event
        return self.event

    def _hold_physical(self, reason):
        self._physical.recovery_allowed = False
        self._finish_training_recording(reason)
        with self.lock:
            if self._hardware_pending is not None:
                self._hardware_pending["cancel_event"].set()
            self._clear_api_lease_locked()
            self.api_authorized = False
        hold_error = None
        try:
            self._physical.hold()
        except Exception as exc:
            hold_error = f"{type(exc).__name__}: {exc}"
            LOGGER.error("physical position hold failed: %s", hold_error)
        with self.lock:
            self._clear_api_lease_locked()
            self._hardware_pending = None
            self.api_authorized = False
            state = self._physical.state
            healthy_hold = hold_error is None and state in {
                AdapterState.STOPPED,
                AdapterState.EXECUTING,
            }
            self.mode = "STOPPED" if healthy_hold else "FAULT"
            self.launched = healthy_hold
            self.drives_enabled = True if healthy_hold else None
            self.controller_ok = healthy_hold
            self.policy_phase = "holding" if healthy_hold else "hold_failed"
            self.command_source = reason
            self._physical_event = (
                f"Trajectory stopped; both arms holding last target: {reason}"
                if healthy_hold
                else f"FAULT: position hold failed: {hold_error}"
            )
            self.event = self._physical_event

    def _stop_physical(self, reason):
        self._physical.recovery_allowed = False
        if reason != "operator_park_zero":
            cancel = getattr(self, "_fault_recovery_cancel", None)
            if cancel is not None:
                cancel.set()
        if reason != 'operator_park_zero':
            self.auto_queue.pause(f'{reason}; automatic queue paused', clear_park=True)
        self._finish_training_recording(reason)
        with self.lock:
            self._policy_deadline = None
        disable_error = None
        try:
            self._physical.stop()
        except Exception as exc:
            disable_error = f"{type(exc).__name__}: {exc}"
            LOGGER.error("physical torque disable unverified: %s", disable_error)
        with self.lock:
            self._clear_api_lease_locked()
            self._hardware_pending = None
            self.api_authorized = False
            self.mode = "FAULT" if disable_error else "DISABLED"
            self.launched = False
            self.drives_enabled = False
            self.policy_phase = "disabled"
            self.command_source = reason
            self._physical_event = (
                f"TORQUE-OFF UNVERIFIED: {disable_error}"
                if disable_error
                else f"Both physical arms torque-disabled: {reason}"
            )
            self.event = self._physical_event

    def _physical_joints(self):
        snapshot = self._physical.snapshot()
        return np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)

    def _api_observation_locked(self):
        snapshot = self._physical.snapshot()
        joints_rad = np.asarray(
            snapshot.left.joints_rad + snapshot.right.joints_rad,
            dtype=np.float64,
        )
        settled = bool(
            self._physical.state == AdapterState.STOPPED
            and self._hardware_pending is None
        )
        homed = bool(
            settled
            and np.max(np.abs(joints_rad - self.home)) <= self.settle_tolerance
        )
        joints_deg = np.rad2deg(joints_rad)
        return self._api_envelope_locked(
            "observation",
            step_id=self.api_observation_step,
            observed_at=time.time(),
            homed=homed,
            settled=settled,
            left_joints_deg=joints_deg[:6].tolist(),
            right_joints_deg=joints_deg[6:].tolist(),
            left_gripper=snapshot.left.gripper,
            right_gripper=snapshot.right.gripper,
            images=self._camera_references_locked(),
        )

    def _bounded_ramp(self, start, target):
        # Home/Park still need a measured hold (and gripper handling) at target.
        if np.array_equal(start, target):
            return (tuple(float(value) for value in target),)
        return plan_joint_trajectory(
            (start, target),
            cadence_hz=LOCAL_CADENCE_HZ,
            max_velocity_rad_s=self._safety_config.max_joint_velocity_rad_s,
            max_acceleration_rad_s2=DEFAULT_MAX_ACCELERATION_RAD_S2,
            max_jerk_rad_s3=DEFAULT_MAX_JERK_RAD_S3,
        ).positions

    def _set_safety_pose(self, joints):
        self._safety_data.qpos[:base.N_JOINTS] = joints
        self._safety_data.qvel[:] = 0.0
        mujoco.mj_forward(self._safety_model, self._safety_data)

    def _forward_kinematics(self, joints):
        with self._physical_lock:
            self._set_safety_pose(joints)
            return {
                name.split("_", 1)[0]: tuple(float(value) for value in self._safety_data.body(name).xpos)
                for name in base.END_EFFECTORS
            }

    def _collision_clear(self, joints):
        with self._physical_lock:
            self._set_safety_pose(joints)
            return self._safety_data.ncon == 0

    def status(self):
        status = super().status()
        operator_mode = status["mode"]
        state = self._physical.state
        if state == AdapterState.FAULT and (self._physical.recovery_status or {}).get('state') == 'failed':
            self._report_idle_servo_fault(ServoFaultHolding(self._physical.fault or 'Motor recovery failed'))
        joints = status["joints"]
        left_gripper = None
        right_gripper = None
        feedback_unavailable = False
        if state in {AdapterState.STOPPED, AdapterState.EXECUTING}:
            try:
                snapshot = self._physical.snapshot()
                joints = list(snapshot.left.joints_rad + snapshot.right.joints_rad)
                left_gripper = snapshot.left.gripper
                right_gripper = snapshot.right.gripper
            except Exception as exc:
                if isinstance(exc, FeedbackUnavailable) or (type(exc) is RuntimeError and str(exc) == 'Motor operation capacity exceeded'):
                    feedback_unavailable = True
                else:
                    if isinstance(exc, ServoFaultHolding):
                        self._report_idle_servo_fault(exc)
                    state = self._physical.state
                    if state != AdapterState.DISABLED:
                        self._physical_event = (
                            f"Physical feedback fault: {type(exc).__name__}: {exc}"
                        )
                        state = AdapterState.FAULT
        status.update(
            {
                "backend": "physical_i2rt",
                "mode": (
                    operator_mode
                    if state in {AdapterState.STOPPED, AdapterState.EXECUTING}
                    else state.value
                ),
                "event": "Joint feedback temporarily unavailable; waiting for fresh telemetry" if feedback_unavailable else self._physical_event,
                "feedback_available": not feedback_unavailable,
                "joints": joints,
                "left_gripper": left_gripper,
                "right_gripper": right_gripper,
                "adapter_fault": self._physical.fault,
                "last_run_failure": self.last_run_failure,
                "recovery": self._physical.recovery_status,
                "policy_settle_tolerance_rad": self.policy_settle_tolerance,
                "launched": state in {AdapterState.STOPPED, AdapterState.EXECUTING},
                "homed": bool(
                    state == AdapterState.STOPPED
                    and not feedback_unavailable
                    and self._hardware_pending is None
                    and np.max(np.abs(np.asarray(joints) - self.home))
                    <= self.settle_tolerance
                ),
                "settled": bool(
                    state == AdapterState.STOPPED
                    and not feedback_unavailable
                    and self._hardware_pending is None
                ),
                "rested": state == AdapterState.DISABLED and self.auto_queue.snapshot()['parked'],
                "policy_phase": status["policy_phase"],
                "command_source": status["command_source"],
                "waypath": {
                    "state": self._waypath_state,
                    "validated": self._waypath_approved is not None,
                    "error": self._waypath_error,
                    "plan": self._waypath_plan.summary() if self._waypath_plan else None,
                },
            }
        )
        status['dataset'] = self._training_recorder.status() if self._training_recorder else {
            'status': 'idle' if os.environ.get('YAM_DATASET_ROOT') else 'disabled',
            'error': self._training_error,
        }
        if self._training_recorder:
            import json
            upload_path = self._training_recorder.path / 'upload.json'
            try:
                status['dataset']['upload'] = json.loads(upload_path.read_text())
            except (OSError, ValueError):
                status['dataset']['upload'] = {'status': 'pending' if status['dataset']['status'] == 'finalized' else 'waiting'}
        status["safety"].update(
            {
                "drives_enabled": (
                    True
                    if state in {AdapterState.STOPPED, AdapterState.EXECUTING}
                    else False if self._physical.torque_off_verified else None
                ),
                "torque_off_verified": self._physical.torque_off_verified,
                "controller_ok": state != AdapterState.FAULT,
                "physical_trajectory_enabled": True,
            }
        )
        if (self._physical.recovery_status or {}).get('state') == 'paused':
            status['event'] = 'Run paused: recovering motor communication'
        status['auto_queue'] = self.auto_queue.snapshot()
        status['control_timing'] = self._physical.timing_status()
        status['can_run_policy'] = policy_ready(status)
        return status

    def api_station_status_payload(self):
        status = self.status()
        state = self._physical.state
        position_ok = bool(status["safety"]["position_limits"])
        initialized = state in {AdapterState.STOPPED, AdapterState.EXECUTING}
        safety_ok = initialized and position_ok and status["safety"]["controller_ok"] and status.get("feedback_available", True)
        reason = None
        if state == AdapterState.FAULT:
            reason = "physical_controller_fault"
            # Publish only reconstructed numeric diagnostics, never raw exceptions.
            from YAM_control.public_fault import gripper_fault_message
            reason = gripper_fault_message(status.get("adapter_fault")) or reason
        elif not initialized:
            reason = "hardware_not_initialized"
        elif not position_ok:
            reason = "joint_limit"
        elif not status.get("feedback_available", True):
            reason = "feedback_temporarily_unavailable"
        joints = np.rad2deg(np.asarray(status["joints"], dtype=np.float64))
        return {
            "schema_version": 1,
            "type": "station_status",
            "source": "hardware",
            "mode": status["mode"],
            "queue_ready": bool(status["auto_queue"]["enabled"] and status["safety"]["controller_ok"]
                and not status["api"]["authorized"] and not status["api"].get("session_id")
                and ((status["mode"] == "DISABLED" and status["auto_queue"]["parked"])
                     or (status["mode"] in {"READY", "STOPPED"} and status["homed"] and status["settled"] and safety_ok))),
            "observed_at": time.time(),
            "homed": bool(status["homed"] and safety_ok),
            "settled": bool(status["settled"] and safety_ok),
            "left_joints_deg": joints[:6].tolist(),
            "right_joints_deg": joints[6:].tolist(),
            "left_gripper": status.get("left_gripper"),
            "right_gripper": status.get("right_gripper"),
            "images": self._camera_references_locked(),
            "safety": {
                "ok": safety_ok,
                "estop_engaged": False,
                "reason": reason,
            },
        }

    def stop(self) -> None:
        self.stop_event.set()
        self.auto_queue.pause('Operator shutting down')
        self._finish_training_recording("operator_shutdown")
        self._physical.stop()
        if isinstance(self._physical, IsolatedAdapter):
            self._physical.close()
        super().stop()


HARDWARE_PAGE = (
    base.PAGE.replace("Bimanual YAM Operator Console / SIM", "Bimanual YAM Operator Console / HARDWARE")
    .replace("LOCAL / BIMANUAL MUJOCO / NO HARDWARE", "LOCAL OPERATOR / PHYSICAL BIMANUAL YAM")
    .replace("Live MuJoCo Bimanual YAM", "Physical Bimanual YAM")
    .replace(
        """<button class="stop" onclick="act('stop')">Stop, Rest & Disable Both</button>""",
        """<button id="park" class="park" onclick="act('park')">Park at Zero &amp; Stop</button><button class="stop" onclick="act('stop')">Emergency / Full Torque Off</button>""",
    )
    .replace(
        "</head>",
        """<style>.controls .park{background:var(--amber)}.controls .stop{grid-column:auto;min-height:66px;font-size:15px}.controls .run{grid-column:1/-1}.waypath{margin:18px 0;padding:16px;border:2px solid var(--ink);background:#d8e8dc}.waypath h3{margin:0 0 10px}.waypath-row{display:flex;gap:10px;flex-wrap:wrap;align-items:end}.waypath label{font:11px ui-monospace,monospace}.waypath select,.waypath input{display:block;margin-top:4px;padding:9px;border:1px solid var(--ink);background:#fff}.waypath-actions{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px}.waypath-actions button{min-height:48px}.waypath-actions .hold{background:var(--amber)}.waypath-note{font:11px/1.4 ui-monospace,monospace;margin-top:10px}</style></head>""",
    )
    .replace(
        "flag('drives',s.safety.drives_enabled,'ENABLED','DISABLED');",
        "if(s.safety.drives_enabled===null){const e=document.getElementById('drives');e.textContent='UNVERIFIED';e.className='bad'}else flag('drives',s.safety.drives_enabled,'ENABLED','DISABLED');",
    )
    .replace(
        "STOP rejects API commands immediately, returns both arms to captured Rest under limits, then disables command acceptance.",
        "Emergency Stop torque-disables immediately without commanding motion. Park at Zero runs a guarded ramp, settles, then disables. All physical commands pass Safety Guardrails before i2rt.",
    )
    .replace("Safety gate", "Safety Guardrails")
    .replace(
        '<div class="controls">',
        """<section class="waypath"><h3>Jerk-limited Waypath</h3><div class="waypath-row"><label>ARM<select id="waypath-arm"><option value="left">Left</option><option value="right">Right</option></select></label><label>FRAME<select id="waypath-frame"><option value="yam_bimanual/base_link">Robot base</option></select></label><label>STEP (CM)<input id="waypath-step" type="number" min="1" max="10" step="1" value="10"></label></div><div class="waypath-actions"><button onclick="waypath('preview')">Preview</button><button onclick="waypath('validate')">Validate</button><button onclick="waypath('execute')">Execute</button><button class="hold" onclick="waypath('hold')">Hold</button></div><div id="waypath-result" class="waypath-note">Order: Preview → Validate → Launch Arms → Move Home → Execute. Forward is base +X; right is base -Y. Emergency torque-off is separate below.</div></section><section id="independent-hard-off" style="margin:18px 0">
  <div style="display:flex;align-items:center;justify-content:space-between;gap:16px;margin-bottom:10px">
    <strong>Independent Hard Torque Off</strong>
    <a href="http://127.0.0.1:8098/" target="_blank" rel="noopener" style="color:inherit">Open standalone</a>
  </div>
  <iframe src="http://127.0.0.1:8098/" title="Disable torque on both arms" sandbox="allow-forms allow-scripts allow-same-origin" referrerpolicy="no-referrer" style="display:block;width:100%;height:640px;border:1px solid rgba(255,255,255,.18);border-radius:12px;background:#eee8d9"></iframe>
</section><div class="controls">""",
        1,
    )
    .replace(
        "</body>",
        """<script>async function waypath(action){const out=document.getElementById('waypath-result');try{const payload={action,arm:document.getElementById('waypath-arm').value,frame:document.getElementById('waypath-frame').value,step_m:Number(document.getElementById('waypath-step').value)/100};const r=await fetch('/api/waypath',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(payload)});const d=await r.json();out.textContent=d.message||d.error||`Waypath action failed (${r.status})`}catch(e){out.textContent=`Waypath request failed: ${e.message}`}}</script></body>""",
    )
)
HARDWARE_PAGE = HARDWARE_PAGE.replace('<h2>Safety Guardrails</h2>', '<h2>Safety Guardrails</h2><section id="last-run-failure" role="alert" hidden><h3>Last run failure</h3><p id="last-run-failure-text" style="white-space:pre-wrap;overflow-wrap:anywhere"></p></section>')
HARDWARE_PAGE = HARDWARE_PAGE.replace("const m=document.getElementById('mode');", "const failure=s.last_run_failure;document.getElementById('last-run-failure').hidden=!failure;if(failure)document.getElementById('last-run-failure-text').textContent=new Date(failure.timestamp*1000).toLocaleString()+' · Step '+failure.step_id+' · '+failure.error_type+'\\n'+failure.message;const m=document.getElementById('mode');")


HARDWARE_PAGE = HARDWARE_PAGE.replace('<h2>Safety Guardrails</h2>', '<h2>Safety Guardrails</h2><p><a href="https://huggingface.co/datasets/andlyu/Public-YAM-runs/tree/main" target="_blank" rel="noopener">Public-YAM-runs</a></p><p id="dataset-status">Dataset recorder: loading</p>')
HARDWARE_PAGE = HARDWARE_PAGE.replace("const m=document.getElementById('mode');", "document.getElementById('dataset-status').textContent='Dataset: '+(s.dataset?.status||'disabled')+' · '+(s.dataset?.rows||0)+' samples · upload '+(s.dataset?.upload?.status||'waiting');const m=document.getElementById('mode');")

HARDWARE_PAGE = HARDWARE_PAGE.replace('<div class="queue-bank">', '''<div class="queue-bank">
<h3>Automatic queue</h3><p id="auto-queue-status" role="status">Loading…</p>
<p>When parked, launch both arms, settle at Home, then run the next policy through the Session API.</p>
<div class="queue-actions"><button id="auto-queue-on" onclick="act('auto_queue_on')">Enable auto queue</button>
<button id="auto-queue-off" onclick="act('auto_queue_off')">Pause auto queue</button></div>
<p>Emergency Stop pauses automatic starts. After a fault or reboot, park again before enabling.</p>
</div><div class="queue-bank">''', 1)
HARDWARE_PAGE = HARDWARE_PAGE.replace("const m=document.getElementById('mode');", """
const aq=s.auto_queue||{};
document.getElementById('auto-queue-status').textContent=(aq.enabled?'ON · ':'PAUSED · ')+(aq.message||'');
document.getElementById('auto-queue-on').disabled=!!(aq.enabled||aq.running);
document.getElementById('auto-queue-off').disabled=!aq.enabled;
document.getElementById('home').disabled=!!aq.running;
const m=document.getElementById('mode');""")

# Prominent torque status is independent of the high-level run mode.
HARDWARE_PAGE = HARDWARE_PAGE.replace('</header>', '''</header>
<div id="torque-indicator" role="status" aria-live="polite" style="margin:12px 18px;padding:14px 18px;border:2px solid currentColor;font:bold 18px ui-monospace,monospace;background:#fff3cf;color:#745000">TORQUE UNKNOWN · waiting for robot status</div>''', 1)
HARDWARE_PAGE = HARDWARE_PAGE.replace('</head>', '''<script>
let torqueStatusUpdated = 0;
function updateTorqueIndicator(safety) {
  const e = document.getElementById('torque-indicator');
  if (!e) return;
  torqueStatusUpdated = Date.now();
  if (safety?.drives_enabled === true) {
    e.textContent = 'TORQUE ENABLED · motors powered'; e.style.color = '#a5261c'; e.style.background = '#ffe4df';
  } else if (safety?.drives_enabled === false && safety?.torque_off_verified === true) {
    e.textContent = 'TORQUE OFF · verified'; e.style.color = '#21644f'; e.style.background = '#e2f3e9';
  } else {
    e.textContent = 'TORQUE UNKNOWN · off state not verified'; e.style.color = '#745000'; e.style.background = '#fff3cf';
  }
}
setInterval(() => {
  if (torqueStatusUpdated && Date.now() - torqueStatusUpdated > 3000) {
    updateTorqueIndicator(null);
    document.getElementById('torque-indicator').textContent = 'TORQUE UNKNOWN · robot status disconnected or stale';
    torqueStatusUpdated = 0;
  }
}, 500);
</script></head>''', 1)
HARDWARE_PAGE = HARDWARE_PAGE.replace("const m=document.getElementById('mode');", "updateTorqueIndicator(s.safety);const m=document.getElementById('mode');", 1)

HARDWARE_PAGE = HARDWARE_PAGE.replace('</header>', '<p id="fault-summary" role="alert" style="margin:12px 18px;white-space:pre-wrap;color:#a5261c;font-weight:bold" hidden></p></header>', 1)
HARDWARE_PAGE = HARDWARE_PAGE.replace("const m=document.getElementById('mode');", "const faultSummary=document.getElementById('fault-summary');faultSummary.hidden=!s.last_run_failure;if(s.last_run_failure)faultSummary.textContent='Last fault: '+s.last_run_failure.message;const m=document.getElementById('mode');", 1)

HARDWARE_PAGE = HARDWARE_PAGE.replace("document.getElementById('run').disabled=s.mode!=='READY'||!s.api.connected;", "document.getElementById('run').disabled=!s.can_run_policy;")

from YAM_control.error_history_ui import add_error_history
HARDWARE_PAGE = add_error_history(HARDWARE_PAGE)
from YAM_control.control_timing_ui import add_control_timing
HARDWARE_PAGE = add_control_timing(HARDWARE_PAGE)

def main() -> int:
    """Launch hardware UI without changing simulation imports."""
    base.PAGE = HARDWARE_PAGE
    base.POSE_CONFIG_PATH = HARDWARE_POSE_CONFIG
    base.OperatorSimulator = PhysicalOperator
    return base.main()


if __name__ == "__main__":
    raise SystemExit(main())
