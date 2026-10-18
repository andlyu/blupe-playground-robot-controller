"""Return to the selected zero pose, verify arrival, then disable torque.

Rejected/unsettled parks retain healthy targets. Explicit Stop and native motor
health shutdowns remain independent of this ordered workflow.
"""
import logging
import os
import numpy as np
from YAM_control.i2rt_bimanual_adapter import (
    AdapterState, TrajectoryInterrupted, TrajectorySettleTimeout, ServoFaultHolding,
)
from YAM_control.hardware_safety import (
    ApprovedTrajectory, BimanualHardwareGuard, SafetyViolation, _joints, _positive,
)
JOINT_COUNT = 12
LOCAL_CADENCE_HZ = 10.0
HOME_WAYPOINT_LIMIT_TOLERANCE_RAD = 0.001
LOGGER = logging.getLogger(__name__)


class MotorGuardedPark(BimanualHardwareGuard):
    """The selected motor-guard profile, without Cartesian/collision queries."""
    def approve(self, start_joints_rad, waypoints_rad, *, cadence_hz,
                target_limit_tolerance_rad=0.):
        cfg = self._config
        cadence = _positive(cadence_hz, 'cadence_hz')
        if cadence != LOCAL_CADENCE_HZ:
            raise ValueError('Park cadence must remain10Hz')
        previous = _joints(start_joints_rad, 'start_joints_rad')
        points = tuple(_joints(p, f'waypoints_rad[{i}]') for i,p in enumerate(waypoints_rad))
        if not points:
            raise SafetyViolation('empty_trajectory')
        if len(points) > cfg.max_waypoints:
            raise SafetyViolation('trajectory_too_long')
        for i,(value,lower,upper) in enumerate(zip(previous,cfg.joint_lower_rad,cfg.joint_upper_rad)):
            if value < lower-cfg.start_limit_tolerance_rad or value > upper+cfg.start_limit_tolerance_rad:
                raise SafetyViolation(f'start_joint_{i}_outside_limits')
        for i,target in enumerate(points):
            self._validate_target(target,i,target_limit_tolerance_rad)
            delta = max(abs(a-b) for a,b in zip(previous,target))
            if delta > cfg.max_adjacent_delta_rad+1e-12:
                raise SafetyViolation('adjacent_delta_limit',waypoint=i)
            if delta*cadence > cfg.max_joint_velocity_rad_s+1e-12:
                raise SafetyViolation('joint_velocity_limit',waypoint=i)
            previous = target
        return ApprovedTrajectory(points,cadence,0)


def park_zero_and_stop(self, *, cancel_event=None, hold=False):
    policy = os.environ.get('YAM_PARK_EXECUTION_POLICY', 'validated')
    if policy not in ('validated', 'controller-faults'):
        raise ValueError('Unknown Park execution policy')
    guard = self._safety_guardrails
    if policy == 'controller-faults':
        # Use the same explicitly selected native motor-guard profile as the
        # attended reset. Operator Park must not silently re-enable geometry.
        guard = MotorGuardedPark(guard._config, forward_kinematics=None,
                                collision_check=None)
    if cancel_event is not None and cancel_event.is_set():
        return 'Park at Zero interrupted; check controller state'
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
    target = np.asarray(getattr(self, 'park_pose', np.zeros(JOINT_COUNT)), dtype=np.float64)
    legal_start = np.clip(
        start,
        self._safety_config.joint_lower_rad,
        self._safety_config.joint_upper_rad,
    )
    waypoints = self._bounded_ramp(legal_start, target)
    # Preserve the calibrated endpoint exactly: subtraction/addition can put a
    # boundary target a few floating-point units outside unchanged motor limits.
    waypoints = (*waypoints[:-1], tuple(float(value) for value in target))
    if not np.array_equal(start, legal_start):
        waypoints = (tuple(float(value) for value in legal_start), *waypoints)
    try:
        approved = guard.approve(
            start,
            waypoints,
            cadence_hz=LOCAL_CADENCE_HZ,
            target_limit_tolerance_rad=HOME_WAYPOINT_LIMIT_TOLERANCE_RAD,
        )
    except Exception as exc:
        message = f"Park at Zero rejected before motion: {type(exc).__name__}: {exc}"
        LOGGER.error(message, exc_info=True)
        with self.lock:
            self.event = f"{message}; torque-off not requested"
            self._physical_event = self.event
        return self.event

    try:
        with self.lock:
            # Approval can outlive a concurrent Stop. Reuse the admission state
            # before changing the mode or dispatching the approved trajectory.
            if (self.mode not in {'STOPPED', 'READY'}
                    or self.api_lease_id is not None
                    or self._physical.state != AdapterState.STOPPED
                    or (cancel_event is not None and cancel_event.is_set())):
                return 'Park at Zero interrupted; check controller state'
            self.mode = "PARKING_ZERO"
            self.command_source = "operator"
            self.policy_phase = "parking_at_zero"
            self.event = "Parking both physical arms at joint zero"
            if policy == 'controller-faults':
                self.event += "; motor guards active, geometry checks off"
            self._physical_event = self.event
        snapshot = self._physical.execute(
            approved.waypoints,
            cadence_hz=approved.cadence_hz,
            settle_tolerance_rad=self.settle_tolerance,
            disable_on_settle_timeout=False,
            cancel_event=cancel_event,
        )
        joints = np.asarray(
            snapshot.left.joints_rad + snapshot.right.joints_rad,
            dtype=np.float64,
        )
        with self.lock:
            self.data.qpos[:JOINT_COUNT] = np.clip(joints, self.lower, self.upper)
            self.data.qvel[:] = 0.0
            self.data.ctrl[:JOINT_COUNT] = self.data.qpos[:JOINT_COUNT]
            self.target = self.data.qpos[:JOINT_COUNT].copy()
    except TrajectoryInterrupted:
        raise
    except TrajectorySettleTimeout as exc:
        if (self._physical.state != AdapterState.STOPPED
                or (cancel_event is not None and cancel_event.is_set())):
            return 'Park at Zero interrupted; check controller state'
        self._hold_physical('park_settle_timeout')
        with self.lock:
            self.event = f"Park at Zero failed to settle; torque-off not requested: {exc}; {self._physical_event}"
            self._physical_event = self.event
        return self.event
    except Exception as exc:
        message = f"Park at Zero failed: {type(exc).__name__}: {exc}"
        LOGGER.error(message, exc_info=True)
        if isinstance(exc, ServoFaultHolding):
            self._mark_servo_fault_holding(exc)
        elif self._physical.state in {AdapterState.STOPPED, AdapterState.EXECUTING}:
            self._hold_physical('park_execution_failure')
        # Native driver failures keep their existing shutdown/fault handling;
        # the Zero workflow never adds a second early torque-off request.
        with self.lock:
            self.event = f"{message}; {self._physical_event}"
            self._physical_event = self.event
        return self.event

    # Verify arrival BEFORE releasing torque; Stop/cancel beats late completion.
    with self.lock:
        if (self.mode != 'PARKING_ZERO' or self._physical.state != AdapterState.STOPPED
                or (cancel_event is not None and cancel_event.is_set())):
            return 'Zero hold interrupted; check controller state' if hold else 'Park at Zero interrupted; check controller state'
    if not np.isfinite(joints).all() or np.max(np.abs(joints-target)) > self.settle_tolerance:
        self._hold_physical('park_arrival_unverified')
        with self.lock:
            self.event = f'Park at Zero failed: arrival unverified; torque-off not requested; {self._physical_event}'
            self._physical_event = self.event
        return self.event

    if hold:
        # execute() has verified settling and retains its last zero target.
        # Never replace a concurrent Stop/fault with a successful hold state.
        with self.lock:
            if (self.mode != "PARKING_ZERO"
                    or self._physical.state != AdapterState.STOPPED
                    or (cancel_event is not None and cancel_event.is_set())):
                return "Zero hold interrupted; check controller state"
            self.mode = "STOPPED"
            self.command_source = "operator_zero_hold"
            self.policy_phase = "holding_zero"
            self.target = target.copy()
            self.data.ctrl[:JOINT_COUNT] = target
            self.event = "Both arms holding commanded encoder zero with torque enabled"
            self._physical_event = self.event
            return self.event

    self._stop_physical("operator_park_zero")
    # The receipt is written only after settled zero feedback AND verified off.
    # Stop cancellation must not be overwritten by a late park completion.
    with self.auto_queue.lock:
        if (self.mode == 'DISABLED' and self._physical.torque_off_verified
                and (cancel_event is None or not cancel_event.is_set())
                and np.max(np.abs(joints-target)) <= self.settle_tolerance):
            self.auto_queue.mark_parked()
    with self.lock:
        if self.mode == "DISABLED":
            self.policy_phase = "parked"
            self.event = "Both physical arms parked at joint zero and torque-disabled"
        self._physical_event = self.event
    return self.event

