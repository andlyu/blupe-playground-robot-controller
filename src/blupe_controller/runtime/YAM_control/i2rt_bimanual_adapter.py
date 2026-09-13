"""Lazy, operator-owned i2rt lifecycle for a physical bimanual YAM.

Importing and constructing this adapter is inert. Motor drivers are created only
by ``launch()``, which is intended to be called by an explicit local operator
action. Approved trajectories use the same i2rt ``command_joint_pos`` primitive
as RoboCurve and are paced locally rather than by network round trips.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import logging
from pathlib import Path
import threading
import time
from typing import Any, Callable

import numpy as np


LOGGER = logging.getLogger(__name__)
# Allow calibrated endpoint feedback drift; commanded grippers stay in [0, 1].
GRIPPER_FEEDBACK_TOLERANCE = 0.02
RobotFactory = Callable[..., Any]
RobotDisabler = Callable[[Any], Any]
ChannelReady = Callable[[str], bool]
WaypointCallback = Callable[[int], None]
WaypointValidator = Callable[
    [int, tuple[float, ...], tuple[float, ...]],
    tuple[float, ...],
]


class AdapterState(str, Enum):
    DISABLED = "DISABLED"
    STOPPING = "STOPPING"
    INITIALIZING = "INITIALIZING"
    STOPPED = "STOPPED"
    EXECUTING = "EXECUTING"
    FAULT = "FAULT"


class TrajectoryInterrupted(RuntimeError):
    """Playback canceled while healthy drivers retain their last target."""


class TrajectorySettleTimeout(RuntimeError):
    """The motors stayed healthy but did not reach the final target in time."""


class ServoFaultHolding(RuntimeError):
    """Servo fault latched while the CAN worker preserves healthy holds."""


class WaypointSafetyRejected(RuntimeError):
    """A just-in-time safety check rejected a waypoint before driver dispatch."""

    def __init__(self, waypoint: int, code: str) -> None:
        self.waypoint = waypoint
        self.code = code
        super().__init__(f"{code} at waypoint {waypoint}")


@dataclass(frozen=True)
class ArmSnapshot:
    joints_rad: tuple[float, ...]
    gripper: float | None
    joint_effort: tuple[float, ...] | None = None
    joint_velocity: tuple[float, ...] | None = None
    gripper_effort: float | None = None
    gripper_velocity: float | None = None


@dataclass(frozen=True)
class BimanualSnapshot:
    left: ArmSnapshot
    right: ArmSnapshot


class I2RTBimanualAdapter:
    """Initialize both arms atomically and disable both on any uncertainty."""

    def __init__(
        self,
        *,
        left_channel: str = "can0",
        right_channel: str = "can1",
        left_gripper_limits: tuple[float, float],
        right_gripper_limits: tuple[float, float],
        calibrate_grippers_on_launch: bool = False,
        robot_factory: RobotFactory | None = None,
        robot_disabler: RobotDisabler | None = None,
        channel_ready: ChannelReady | None = None,
    ) -> None:
        self.left_channel = left_channel
        self.right_channel = right_channel
        self.left_gripper_limits = _limits(left_gripper_limits, "left")
        self.right_gripper_limits = _limits(right_gripper_limits, "right")
        self.calibrate_grippers_on_launch = calibrate_grippers_on_launch
        self._robot_factory = robot_factory
        self._robot_disabler = robot_disabler
        self._channel_ready = channel_ready or _channel_is_up
        self._robots: dict[str, Any] = {}
        self.torque_off_verified = False
        self._state = AdapterState.DISABLED
        self._fault: str | None = None
        self._lock = threading.RLock()
        self._stop_requested = threading.Event()
        self._last_command = None
        self.recovery_status = None
        self.recovery_allowed = False
        self._recovery_lock = threading.RLock()

    def diagnostic_event(self, event, **details):
        for robot in tuple(self._robots.values()):
            trace = getattr(robot, '_yam_motor_trace', None)
            if trace is not None:
                trace.add({'monotonic':time.monotonic(), 'event':event, **details})

    def diagnostic_trace(self):
        from YAM_control.motor_diagnostics import snapshot
        with self._lock:
            robots = dict(self._robots)
            requested = dict(self._last_command) if self._last_command else None
        return {'schema_version':1, 'captured_at':time.time(), 'captured_monotonic':time.monotonic(),
                'requested_joint_command':requested,
                'arms':{arm:snapshot(robot) for arm,robot in robots.items()}}

    def timing_status(self):
        from YAM_control.control_timing import arm_timing
        return {'isolated': False, 'status_age_ms': 0, 'available': True,
                'arms': {arm: arm_timing(robot) for arm, robot in tuple(self._robots.items())}}

    @property
    def last_command(self):
        with self._lock:
            return dict(self._last_command) if self._last_command else None

    @property
    def state(self) -> AdapterState:
        with self._lock:
            return self._state

    @property
    def fault(self) -> str | None:
        with self._lock:
            return self._fault

    def launch(self, *, cancel_event: threading.Event | None = None) -> BimanualSnapshot:
        """Initialize both drivers once and hold their observed positions."""
        with self._lock:
            def check_cancelled():
                if cancel_event is not None and cancel_event.is_set():
                    raise TrajectoryInterrupted("arm initialization cancelled")

            check_cancelled()
            if self._state == AdapterState.STOPPED:
                try:
                    for arm, channel in (("left", self.left_channel), ("right", self.right_channel)):
                        _verify_robot_health(self._robots[arm], arm, channel)
                except Exception as exc:
                    self._state = AdapterState.FAULT
                    self._fault = f"relaunch health check failed: {type(exc).__name__}: {exc}"
                    raise
                return self._snapshot_locked()
            if self._state == AdapterState.EXECUTING:
                raise RuntimeError("playback is still active; wait for Hold to finish before Launch")
            if self._state == AdapterState.STOPPING:
                raise RuntimeError("torque shutdown is still in progress; wait before Launch")
            if self._state == AdapterState.INITIALIZING:
                raise RuntimeError("arm initialization is already in progress")
            if self._state == AdapterState.FAULT:
                if self._robots:
                    raise RuntimeError(
                        "Driver recovery required: support both arms, use Independent Hard Torque Off, "
                        "verify both arms acknowledge motor-off, then restart the operator console. "
                        "Launch cannot replace drivers whose cleanup is unverified."
                    )
                # A failed initialization whose cleanup succeeded owns no drivers.
                # An explicit Launch may safely retry the usual initialization.
            self.torque_off_verified = False
            self._state = AdapterState.INITIALIZING
            self._fault = None
            self._stop_requested.clear()
            created: list[tuple[str, Any]] = []
            try:
                for channel in (self.left_channel, self.right_channel):
                    if not self._channel_ready(channel):
                        raise RuntimeError(f"CAN channel is not UP: {channel}")
                factory = self._robot_factory or _default_robot_factory()
                left = factory(
                    channel=self.left_channel,
                    zero_gravity_mode=False,
                    gripper_limits_override=(None if self.calibrate_grippers_on_launch
                                             else np.asarray(self.left_gripper_limits)),
                )
                from YAM_control.motor_diagnostics import attach
                created.append(("left", left))
                if self._robot_factory is None:
                    _set_left_wrist_kp(left, 20.0)
                attach(left)
                check_cancelled()
                left_position = _position(left, "left")
                right = factory(
                    channel=self.right_channel,
                    zero_gravity_mode=False,
                    gripper_limits_override=(None if self.calibrate_grippers_on_launch
                                             else np.asarray(self.right_gripper_limits)),
                )
                created.append(("right", right))
                attach(right)
                check_cancelled()
                right_position = _position(right, "right")

                # The first command is exactly the measured state: no jump.
                left.command_joint_pos(left_position.copy())
                right.command_joint_pos(right_position.copy())
                deadline = time.monotonic() + 0.5
                while True:
                    check_cancelled()
                    _verify_robot_health(left, "left", self.left_channel)
                    _verify_robot_health(right, "right", self.right_channel)
                    if time.monotonic() >= deadline:
                        break
                    time.sleep(0.1)
                self._robots = {"left": left, "right": right}
                self._state = AdapterState.STOPPED
                LOGGER.info("physical YAM drivers initialized and holding current pose")
                return self._snapshot_locked()
            except Exception as exc:
                failures, retained = self._disable_pairs(reversed(created))
                self._robots = retained
                self.torque_off_verified = bool(created) and not failures and not retained
                self._state = AdapterState.FAULT
                self._fault = f"{type(exc).__name__}: {exc}"
                if failures:
                    self._fault += "; torque disable unverified: " + "; ".join(failures)
                LOGGER.error("physical YAM initialization failed closed: %s", self._fault)
                raise

    def execute(
        self,
        waypoints_rad: tuple[tuple[float, ...], ...],
        *,
        cadence_hz: float,
        gripper_waypoints: tuple[tuple[float, float], ...] | None = None,
        max_gripper_delta_per_step: float = 0.3,
        on_waypoint: WaypointCallback | None = None,
        validate_waypoint: WaypointValidator | None = None,
        settle_tolerance_rad: float = 0.02,
        settle_timeout_s: float = 3.0,
        disable_on_settle_timeout: bool = True,
        cancel_event: threading.Event | None = None,
    ) -> BimanualSnapshot:
        """Check and send each waypoint locally at the requested cadence."""
        if (
            cadence_hz <= 0
            or settle_tolerance_rad <= 0
            or settle_timeout_s <= 0
            or max_gripper_delta_per_step <= 0
        ):
            raise ValueError("cadence and settle settings must be positive")
        validated_waypoints = tuple(_approved_waypoint(point) for point in waypoints_rad)
        if not validated_waypoints:
            raise ValueError("approved trajectory is empty")
        validated_grippers = _approved_grippers(gripper_waypoints, len(validated_waypoints))
        with self._recovery_lock, self._lock:
            if cancel_event is not None and cancel_event.is_set():
                raise TrajectoryInterrupted("queued playback canceled before execution")
            if self._state != AdapterState.STOPPED:
                raise RuntimeError("physical arms are not ready for trajectory execution")
            left = self._robots["left"]
            right = self._robots["right"]
            self._state = AdapterState.EXECUTING
            self._stop_requested.clear()
        period = 1.0 / cadence_hz
        try:
            if any(getattr(robot.motor_chain, 'runtime_fault', None) for robot in (left, right)):
                initial = _snapshot_robots(left, right)
                self._recover_before_motion(
                    initial.left.joints_rad + initial.right.joints_rad,
                    (initial.left.gripper, initial.right.gripper),
                    (lambda measured, goal: validate_waypoint(0, measured, goal)) if validate_waypoint else None,
                    cancel_event)
            left_position = _verify_robot_health(left, "left", self.left_channel)
            right_position = _verify_robot_health(right, "right", self.right_channel)
            previous_target = tuple(left_position[:6]) + tuple(right_position[:6])
            left_gripper = float(left_position[6])
            right_gripper = float(right_position[6])
            if validated_grippers is None:
                resolved_grippers = ((left_gripper, right_gripper),) * len(validated_waypoints)
            else:
                previous = (left_gripper, right_gripper)
                for index, pair in enumerate(validated_grippers):
                    if max(abs(pair[arm] - previous[arm]) for arm in (0, 1)) > (
                        max_gripper_delta_per_step + 1e-12
                    ):
                        raise ValueError(f"gripper delta limit at waypoint {index}")
                    previous = pair
                resolved_grippers = validated_grippers
            next_deadline = time.monotonic()
            for index, waypoint in enumerate(validated_waypoints):
                if self._stop_requested.is_set() or (cancel_event is not None and cancel_event.is_set()):
                    raise TrajectoryInterrupted("trajectory interrupted; retaining last target")
                link_guard = getattr(self, '_operator_link_guard', None)
                if link_guard and link_guard(previous_target, (left_gripper, right_gripper),
                        (lambda measured, goal: validate_waypoint(index, measured, goal)) if validate_waypoint else None,
                        cancel_event):
                    next_deadline = time.monotonic()
                if validate_waypoint is not None:
                    waypoint = _approved_waypoint(
                        validate_waypoint(index, previous_target, waypoint)
                    )
                target = np.asarray(waypoint, dtype=np.float64)
                if self._recover_before_motion(
                    previous_target, (left_gripper, right_gripper),
                    (lambda measured, goal: validate_waypoint(index, measured, goal)) if validate_waypoint else None,
                    cancel_event,
                ):
                    next_deadline = time.monotonic()  # No catch-up burst after a pause.
                    if validate_waypoint is not None:
                        validate_waypoint(index, previous_target, waypoint)
                # A slow validator can itself trigger a pause. Recheck after
                # ALL callbacks so it cannot dispatch past a newly raised gate.
                if link_guard and link_guard(previous_target, (left_gripper, right_gripper),
                        (lambda measured, goal: validate_waypoint(index, measured, waypoint)) if validate_waypoint else None,
                        cancel_event):
                    next_deadline = time.monotonic()
                left_gripper, right_gripper = resolved_grippers[index]
                left_command = np.concatenate((target[:6], [left_gripper]))
                right_command = np.concatenate((target[6:], [right_gripper]))
                with self._lock:
                    if self._stop_requested.is_set() or (cancel_event is not None and cancel_event.is_set()):
                        raise TrajectoryInterrupted("dispatch canceled; retaining last target")
                    left.command_joint_pos(left_command)
                    right.command_joint_pos(right_command)
                    self._last_command = {"joints": tuple(waypoint),
                                          "grippers": (left_gripper, right_gripper),
                                          "monotonic": time.monotonic()}
                previous_target = waypoint
                if on_waypoint is not None:
                    on_waypoint(index)
                next_deadline += period
                if self._stop_requested.wait(max(0.0, next_deadline - time.monotonic())):
                    raise TrajectoryInterrupted("trajectory interrupted; retaining last target")

            target = np.asarray(validated_waypoints[-1], dtype=np.float64)
            settled_since: float | None = None
            deadline = time.monotonic() + settle_timeout_s
            while time.monotonic() < deadline:
                if self._stop_requested.is_set() or (cancel_event is not None and cancel_event.is_set()):
                    raise TrajectoryInterrupted("settle interrupted; retaining last target")
                pause_started = time.monotonic()
                if self._recover_before_motion(
                    tuple(target), (left_gripper, right_gripper),
                    (lambda measured, goal: validate_waypoint(len(validated_waypoints)-1, measured, goal)) if validate_waypoint else None,
                    cancel_event,
                ):
                    deadline += time.monotonic() - pause_started
                    settled_since = None
                snapshot = _snapshot_robots(left, right)
                measured = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
                if np.max(np.abs(measured - target)) <= settle_tolerance_rad:
                    settled_since = settled_since or time.monotonic()
                    if time.monotonic() - settled_since >= 0.35:
                        with self._lock:
                            self._state = AdapterState.STOPPED
                        return snapshot
                else:
                    settled_since = None
                self._stop_requested.wait(0.02)
            residuals = np.abs(measured - target)
            worst_index = int(np.argmax(residuals))
            arm = "left" if worst_index < 6 else "right"
            joint = worst_index if worst_index < 6 else worst_index - 6
            raise TrajectorySettleTimeout(
                f"Arm did not reach its target within {settle_timeout_s:g}s: "
                f"{arm}_joint_{joint} residual_rad={residuals[worst_index]:.6f} "
                f"measured_rad={measured[worst_index]:.6f} "
                f"target_rad={target[worst_index]:.6f}; "
                f"allowed_rad={settle_tolerance_rad:.3f}. Motion stopped. "
                "Check for contact or obstruction before continuing."
            )
        except ServoFaultHolding as exc:
            self._retain_servo_fault_hold(exc)
            raise
        except TrajectoryInterrupted:
            with self._lock:
                if self._state == AdapterState.EXECUTING:
                    self._state = AdapterState.STOPPED
            raise
        except WaypointSafetyRejected:
            with self._lock:
                if self._state != AdapterState.DISABLED:
                    self._state = AdapterState.STOPPED
            raise
        except TrajectorySettleTimeout as exc:
            if disable_on_settle_timeout:
                LOGGER.error("physical trajectory settle failure; disabling: %s", exc)
                with self._lock:
                    robots = [(arm, self._robots.pop(arm, None)) for arm in ("right", "left")]
                    self._fault = f"{type(exc).__name__}: {exc}"
                    self._state = AdapterState.FAULT
                failures, retained = self._disable_pairs(robots)
                with self._lock:
                    self._robots.update(retained)
                    if failures:
                        self._fault += "; torque disable unverified: " + "; ".join(failures)
            else:
                LOGGER.warning("physical trajectory did not settle; holding for recovery: %s", exc)
                with self._lock:
                    self._state = AdapterState.STOPPED
            raise
        except Exception as exc:
            LOGGER.error(
                "physical trajectory failed before disable: %s: %s",
                type(exc).__name__,
                exc,
                exc_info=True,
            )
            with self._lock:
                robots = [(arm, self._robots.pop(arm, None)) for arm in ("right", "left")]
                operator_stopped = self._stop_requested.is_set() and self._state in {AdapterState.DISABLED, AdapterState.STOPPING}
                if not operator_stopped:
                    self._fault = f"{type(exc).__name__}: {exc}"
                    self._state = AdapterState.FAULT
            failures, retained = self._disable_pairs(robots)
            with self._lock:
                self._robots.update(retained)
                if failures:
                    self._state = AdapterState.FAULT
                    self._fault = (self._fault or "execution failed") + "; torque disable unverified: " + "; ".join(failures)
            raise

    def _recover_before_motion(self, resume_target, resume_grippers, validate_resume, cancel_event, *, idle=False):
        with self._recovery_lock:
            guard = getattr(self, '_operator_link_guard', None)
            resumed = guard(resume_target, resume_grippers, validate_resume, cancel_event) if guard and not idle else False
            return self._recover_communication(resume_target, resume_grippers, validate_resume, cancel_event, idle=idle) or resumed

    def _recover_communication(self, resume_target, resume_grippers, validate_resume, cancel_event, *, idle=False):
        """Pause both arms until the driver verifies a communication recovery."""
        pairs = [(arm, self._robots[arm], channel) for arm, channel in
                 (("left", self.left_channel), ("right", self.right_channel))]
        faults = [(arm, robot.motor_chain) for arm, robot, _ in pairs
                  if getattr(robot.motor_chain, 'runtime_fault', None)]
        if not faults:
            for arm, robot, channel in pairs:
                _verify_runtime_health(robot, arm, channel)
            return False
        chains = [robot.motor_chain for _, robot, _ in pairs]
        for arm, chain in faults:
            if not hasattr(chain, 'recovery_status') or chain.recovery_status() == 'failed':
                self.recovery_status = {'state':'failed', 'reason':f'{arm} servo fault: {chain.runtime_fault}'}
                raise ServoFaultHolding(self.recovery_status['reason'])
        if any(not hasattr(chain, 'pause_for_peer_recovery') for chain in chains):
            self.recovery_status = {'state':'failed', 'reason':'Bimanual recovery support unavailable'}
            raise ServoFaultHolding(self.recovery_status['reason'])
        started = time.monotonic()
        self.recovery_status = {'state':'paused', 'reason':str(faults[0][1].runtime_fault)}
        self.diagnostic_event('communication_recovery_paused', reason=self.recovery_status['reason'])
        for chain in chains:
            chain.pause_for_peer_recovery()
        # The driver caps communication recovery at 1.5 s from fault feedback.
        # Allow its result plus a fresh complete feedback cycle to be published.
        # The independent run-duration watchdog and Stop still take precedence.
        deadline = started + 2.0
        try:
            while True:
                if self._stop_requested.is_set() or (cancel_event and cancel_event.is_set()):
                    raise TrajectoryInterrupted('Recovery cancelled; no resume')
                for arm, robot, channel in pairs:
                    if not robot._server_thread.is_alive() or not robot.motor_chain.running:
                        raise ServoFaultHolding(f'{arm} controller stopped during recovery')
                states = [chain.recovery_status(started) for chain in chains]
                if 'failed' in states:
                    reason = '; '.join(str(chain.runtime_fault) for chain in chains if chain.runtime_fault)
                    raise ServoFaultHolding('Communication recovery failed: ' + reason)
                if all(state == 'recovered' for state in states):
                    break
                if time.monotonic() >= deadline:
                    raise ServoFaultHolding('Communication recovery did not produce fresh healthy feedback')
                self._stop_requested.wait(.005)
            snap = _snapshot_robots(pairs[0][1], pairs[1][1])
            measured = tuple(snap.left.joints_rad + snap.right.joints_rad)
            grips = (snap.left.gripper, snap.right.gripper)
            # Validate outside CAN locks so collision checks never starve motors.
            if max(abs(a-b) for a,b in zip(measured,resume_target)) > .06 + 1e-12:
                raise ServoFaultHolding('Recovery pose moved too far to resume safely')
            if max(abs(a-b) for a,b in zip(grips,resume_grippers)) > .3 + 1e-12:
                raise ServoFaultHolding('Recovery gripper position changed too far to resume safely')
            if validate_resume is not None:
                validate_resume(measured, resume_target)
            from contextlib import ExitStack
            with self._lock, ExitStack() as locks:
                for chain in chains:
                    locks.enter_context(chain.command_lock)
                expected = AdapterState.STOPPED if idle else AdapterState.EXECUTING
                if self._state != expected or (idle and not self.recovery_allowed) or self._stop_requested.is_set() or (cancel_event and cancel_event.is_set()):
                    raise TrajectoryInterrupted('Recovery cancelled before resume')
                if any(chain.recovery_status(started) != 'recovered' for chain in chains):
                    raise ServoFaultHolding('Motor status changed before resume')
                pairs[0][1].command_joint_pos(np.asarray(tuple(resume_target[:6]) + (resume_grippers[0],)))
                pairs[1][1].command_joint_pos(np.asarray(tuple(resume_target[6:]) + (resume_grippers[1],)))
                for chain in chains:
                    chain.release_recovery_hold(started)
                self.recovery_status = {'state':'resumed'}
                self.diagnostic_event('communication_recovery_resumed')
            return True
        except TrajectoryInterrupted:
            self.recovery_status = {'state':'cancelled'}
            raise
        except Exception as exc:
            self.recovery_status = {'state':'failed', 'reason':str(exc)}
            if isinstance(exc, ServoFaultHolding):
                raise
            raise ServoFaultHolding(f'Recovery resume validation failed: {exc}') from exc

    def snapshot(self) -> BimanualSnapshot:
        with self._lock:
            if self._state not in {AdapterState.STOPPED, AdapterState.EXECUTING}:
                raise RuntimeError("physical arms are not initialized")
            left = self._robots["left"]
            right = self._robots["right"]
        try:
            if self._state == AdapterState.STOPPED and self.recovery_allowed:
                with self._recovery_lock:
                    if self._state == AdapterState.STOPPED:
                        snap = _snapshot_robots(left, right)
                        self._recover_before_motion(
                            snap.left.joints_rad + snap.right.joints_rad,
                            (snap.left.gripper, snap.right.gripper), None, None, idle=True)
            for arm, robot, channel in (("left", left, self.left_channel), ("right", right, self.right_channel)):
                chain = robot.motor_chain
                recovering = (self._state == AdapterState.EXECUTING
                              and getattr(chain, 'runtime_fault', None)
                              and hasattr(chain, 'recovery_status'))
                if not recovering:
                    _verify_runtime_health(robot, arm, channel)
            return _snapshot_robots(left, right)
        except ServoFaultHolding as exc:
            self._retain_servo_fault_hold(exc)
            raise
        except Exception as exc:
            with self._lock:
                if self._state in {AdapterState.STOPPED, AdapterState.EXECUTING}:
                    self._fault = f"{type(exc).__name__}: {exc}"
                    self._state = AdapterState.FAULT
                    self._stop_requested.set()
            LOGGER.error("physical adapter feedback failed closed: %s", self._fault)
            raise

    def _retain_servo_fault_hold(self, exc: Exception) -> None:
        """Cancel both producers' motion without cutting healthy servos' torque."""
        with self._lock:
            self._stop_requested.set()
            if self._state in {AdapterState.DISABLED, AdapterState.STOPPING}:
                return  # An explicit torque-off racing this report takes precedence.
            self._state = AdapterState.FAULT
            self._fault = str(exc)
            for arm, robot in self._robots.items():
                try:
                    robot.motor_chain.request_fault_hold("bimanual motion canceled after servo fault")
                except Exception as hold_error:
                    self._fault += f"; {arm} hold unverified: {type(hold_error).__name__}"
            self._fault += "; motion latched stopped; operator recovery required"
        LOGGER.error("[servo-recovery] %s", self._fault)

    def hold_measured(self, *, cancel_event=None):
        """Replace a missed target with measured holds; never clear motor faults."""
        with self._lock:
            if self._state != AdapterState.STOPPED or self._stop_requested.is_set() or (cancel_event and cancel_event.is_set()):
                raise TrajectoryInterrupted("Measured hold cancelled")
            positions = []
            for arm, channel in (("left", self.left_channel), ("right", self.right_channel)):
                robot = self._robots[arm]
                positions.append(_verify_robot_health(robot, arm, channel))
                feedback = getattr(robot.motor_chain, '_last_runtime_feedback', None)
                if feedback is not None or self._robot_factory is None:
                    now = time.monotonic()
                    if not isinstance(feedback, dict) or any(
                        motor not in feedback or feedback[motor].get('code') != '0x1'
                        or not 0 <= now - float(feedback[motor].get('monotonic', 0)) <= .25
                        for motor in range(1, 8)
                    ):
                        raise ServoFaultHolding(f'{arm} measured hold requires fresh normal motor feedback')
            # Validate both sides before commanding either; Stop shares this lock.
            for arm, position in zip(("left", "right"), positions):
                self._robots[arm].command_joint_pos(position.copy())
            return _snapshot_robots(self._robots['left'], self._robots['right'])

    def hold(self) -> None:
        """Cancel playback atomically with dispatch; retain healthy driver targets."""
        with self._lock:
            self._stop_requested.set()
            if self._state == AdapterState.DISABLED:
                return
            if self._state not in {AdapterState.STOPPED, AdapterState.EXECUTING}:
                raise RuntimeError(self._fault or "physical adapter cannot hold")
            try:
                for arm, channel in (("left", self.left_channel), ("right", self.right_channel)):
                    _verify_runtime_health(self._robots[arm], arm, channel)
            except Exception as exc:
                self._state = AdapterState.FAULT
                self._fault = f"hold health verification failed: {type(exc).__name__}: {exc}"
                raise
        LOGGER.info("physical playback canceled; healthy drivers retain last targets")

    def stop(self) -> None:
        """Immediately disable both motor chains and fail if acknowledgement is incomplete."""
        self._stop_requested.set()
        with self._lock:
            if self._state == AdapterState.STOPPING:
                raise RuntimeError("torque shutdown is already in progress")
            if not self._robots and not self.torque_off_verified:
                raise RuntimeError(self._fault or "no owned drivers; torque-off is unverified")
            if self._state == AdapterState.FAULT and not self._robots:
                raise RuntimeError(self._fault or "motor state unverified; independent recovery required")
            robots = [
                (arm, self._robots.pop(arm, None))
                for arm in ("right", "left")
            ]
            self.torque_off_verified = False
            self._state = AdapterState.STOPPING
            self._fault = None
        failures, retained = self._disable_pairs(robots)
        if failures:
            message = "torque disable unverified: " + "; ".join(failures)
            with self._lock:
                self._robots.update(retained)
                self._state = AdapterState.FAULT
                self._fault = message
            raise RuntimeError(message)
        with self._lock:
            self._state = AdapterState.DISABLED
            self.torque_off_verified = True
        LOGGER.info("physical YAM drivers disabled")

    def _snapshot_locked(self) -> BimanualSnapshot:
        return BimanualSnapshot(
            left=_arm_snapshot(_position(self._robots["left"], "left")),
            right=_arm_snapshot(_position(self._robots["right"], "right")),
        )

    def _disable(self, robot: Any) -> str | None:
        disabler = self._robot_disabler or _default_robot_disabler
        try:
            result = disabler(robot)
            if isinstance(result, dict) and result.get("ok") is False:
                failed_ids = [
                    error.get("motor_id")
                    for error in result.get("errors", [])
                    if error.get("motor_id") is not None
                ]
                raise RuntimeError(f"motor-off acknowledgement failed for ids {failed_ids}")
            return None
        except Exception as exc:
            LOGGER.error("motor disable failed: %s", exc)
            return f"{type(exc).__name__}: {exc}"

    def _disable_pairs(self, robots) -> tuple[list[str], dict[str, Any]]:
        failures: list[str] = []
        retained: dict[str, Any] = {}
        for arm, robot in robots:
            if robot is None:
                continue
            error = self._disable(robot)
            if error is not None:
                failures.append(f"{arm}: {error}")
                retained[arm] = robot
        return failures, retained


def _default_robot_factory() -> RobotFactory:
    from i2rt.robots.get_robot import get_yam_robot

    return get_yam_robot


def _default_robot_disabler(robot: Any) -> None:
    from YAM_control.yam_real_serve import disable_motorchain

    return disable_motorchain(robot)


def _channel_is_up(channel: str) -> bool:
    flags_path = Path("/sys/class/net") / channel / "flags"
    try:
        flags = int(flags_path.read_text(encoding="ascii").strip(), 16)
    except (OSError, ValueError):
        return False
    return bool(flags & 0x1)


def _limits(values: tuple[float, float], arm: str) -> tuple[float, float]:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (2,) or not np.all(np.isfinite(array)) or array[0] == array[1]:
        raise ValueError(f"{arm} gripper limits must contain two distinct finite values")
    return float(array[0]), float(array[1])


def _approved_waypoint(values: tuple[float, ...]) -> tuple[float, ...]:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (12,) or not np.all(np.isfinite(array)):
        raise ValueError("approved trajectory contained invalid joints")
    return tuple(float(value) for value in array)


def _approved_grippers(
    values: tuple[tuple[float, float], ...] | None,
    waypoint_count: int,
) -> tuple[tuple[float, float], ...] | None:
    if values is None:
        return None
    if len(values) != waypoint_count:
        raise ValueError("gripper trajectory length does not match joints")
    result = []
    for index, pair in enumerate(values):
        array = np.asarray(pair, dtype=np.float64)
        if array.shape != (2,) or not np.all(np.isfinite(array)) or np.any(array < 0) or np.any(array > 1):
            raise ValueError(f"approved trajectory contained invalid grippers at waypoint {index}")
        result.append((float(array[0]), float(array[1])))
    return tuple(result)


def _position(robot: Any, arm: str, observation=None) -> np.ndarray:
    if observation is None:
        observation = robot.get_observations()
    joints = np.asarray(observation.get("joint_pos"), dtype=np.float64)
    gripper = np.asarray(observation.get("gripper_pos"), dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise RuntimeError(f"{arm} driver returned invalid arm joint feedback")
    if (
        gripper.shape != (1,)
        or not np.all(np.isfinite(gripper))
        or not -GRIPPER_FEEDBACK_TOLERANCE <= float(gripper[0]) <= 1.0 + GRIPPER_FEEDBACK_TOLERANCE
    ):
        raise RuntimeError(
            f"{arm} driver returned invalid normalized gripper feedback: "
            f"values={gripper.tolist()!r}, shape={gripper.shape}; expected one finite value in "
            f"[{-GRIPPER_FEEDBACK_TOLERANCE}, {1.0 + GRIPPER_FEEDBACK_TOLERANCE}]"
        )
    return np.concatenate((joints, np.clip(gripper, 0.0, 1.0)))


def _arm_snapshot(position: np.ndarray) -> ArmSnapshot:
    return ArmSnapshot(
        joints_rad=tuple(float(value) for value in position[:6]),
        gripper=float(position[6]),
    )


def _snapshot_robots(left: Any, right: Any) -> BimanualSnapshot:
    def read(robot, arm):
        observation = robot.get_observations()
        position = _position(robot, arm, observation)
        def optional(key, count):
            value = observation.get(key)
            if value is None:
                return None
            try:
                data = np.asarray(value, dtype=float)
                if data.shape == (count,) and np.all(np.isfinite(data)):
                    return tuple(float(v) for v in data)
            except (TypeError, ValueError):
                pass
            return None
        effort, velocity = optional('gripper_eff', 1), optional('gripper_vel', 1)
        return ArmSnapshot(tuple(float(v) for v in position[:6]), float(position[6]),
                           optional('joint_eff', 6), optional('joint_vel', 6),
                           effort[0] if effort else None, velocity[0] if velocity else None)
    return BimanualSnapshot(left=read(left, 'left'), right=read(right, 'right'))


def _verify_runtime_health(robot: Any, arm: str, channel: str) -> None:
    control_thread = getattr(robot, "_server_thread", None)
    if control_thread is None or not control_thread.is_alive():
        raise RuntimeError(f"{arm} i2rt control thread is not alive on {channel}")
    chain = getattr(robot, "motor_chain", None)
    if chain is None or not bool(getattr(chain, "running", False)):
        raise RuntimeError(f"{arm} motor chain is not running on {channel}")
    fault = getattr(chain, "runtime_fault", None)
    if fault:
        raise ServoFaultHolding(f"{arm} servo fault on {channel}: {fault}")


def _verify_robot_health(robot: Any, arm: str, channel: str) -> np.ndarray:
    _verify_runtime_health(robot, arm, channel)
    chain = robot.motor_chain
    states = chain.read_states()
    if len(states) != 7:
        raise RuntimeError(f"{arm} expected 7 motors on {channel}, got {len(states)}")
    ids = tuple(int(state.id) for state in states)
    if ids != tuple(range(1, 8)):
        raise RuntimeError(f"{arm} motor assignment mismatch on {channel}: ids={ids}")
    errors = tuple(
        (int(state.id), str(state.error_code))
        for state in states
        if str(state.error_code) != "0x1"
    )
    if errors:
        raise RuntimeError(f"{arm} motor faults on {channel}: {errors}")
    return _position(robot, arm)


def _set_left_wrist_kp(robot, value):
    """User-requested wrist-only gain; preserve all other gains and damping."""
    kp = np.asarray(robot._kp, dtype=float).copy()
    kd = np.asarray(robot._kd, dtype=float).copy()
    if kp.shape != kd.shape or kp.ndim != 1 or len(kp) < 6:
        raise RuntimeError('Unexpected driver gains; wrist gain was not applied')
    kp[5] = value
    robot.update_kp_kd(kp, kd)
