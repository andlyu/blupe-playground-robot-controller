"""Lease-owned cube poses through Session API; no local-pose reservation.

The operator selects the cube profile. The existing adapter remains the only
motor owner. Stop revokes the lease and cancels the same dispatch event.
"""
from copy import deepcopy
from functools import lru_cache
import hashlib
import inspect
import os
from types import SimpleNamespace
import textwrap
import threading
import time
import uuid

import numpy as np

from YAM_control.i2rt_bimanual_adapter import (
    AdapterState, ServoFaultHolding, TrajectoryInterrupted, TrajectorySettleTimeout,
)
from YAM_control.zero_control import MotorGuardedPark

PROTOCOL = 'cube-session-pose-v1'
TIMING_PROFILE = 'cube-linear-0.1-0.8-v1'
RAMP_SHA256 = '32cc35028293d78e08e0dcaa9abbcd00017b1c941d8fb91697dd1676be746466'
DELEGATOR_SHA256 = '8e9e1fac9c83589d413c94436e91a3c81857ce6e22b4c5c71b8a1eebc11ed400'


@lru_cache(maxsize=16)
def qualified_ramp(operator_class):
    for cls in operator_class.__mro__:
        function = cls.__dict__.get('_bounded_ramp')
        if function is None:
            continue
        try:
            digest = hashlib.sha256(textwrap.dedent(inspect.getsource(function)).strip().encode()).hexdigest()
        except (OSError, TypeError):
            return None
        if digest == RAMP_SHA256 and function.__globals__.get('LOCAL_CADENCE_HZ') == 10.:
            return function
        if digest != DELEGATOR_SHA256:
            break
    return None


def points_for(op, start, target, floor=None):
    if floor is None:
        return op._bounded_ramp(start, target)
    if type(floor) not in (int, float) or floor not in (.1, .8):
        raise ValueError('Unqualified cube duration floor')
    function = qualified_ramp(type(op))
    if function is not None:
        return function(SimpleNamespace(ramp_secs=floor), start, target)
    raise ValueError('Installed ramp differs from the qualified cube timing helper')


class CubeSessionExecutor:
    def __init__(self, op):
        self.op = op
        self.lock = threading.RLock()
        self.plan = None
        self.result = dict(state='empty')
        self.cancel = threading.Event()
        self.finished = threading.Event()
        self.finished.set()
        self.active = False
        self.stopped_identity = None

    @property
    def enabled(self):
        return os.environ.get('YAM_CUBE_RESET_SESSION_ENABLED') == '1'

    @property
    def policy(self):
        return os.environ.get('YAM_CUBE_RESET_EXECUTION_POLICY', 'validated')

    def status(self):
        # Operator status can be called with op.lock held. Do not acquire the
        # admission lock here: request admission takes that lock before op.lock.
        identity = (self.op.api_session_id, self.op.api_episode_id, self.op.api_lease_id)
        current = self.plan is not None and self.plan['identity'] == identity
        return dict(deepcopy(self.result), protocol=PROTOCOL, enabled=self.enabled,
            active=self.active, cancelled=bool(current and self.cancel.is_set()),
            lease_id=self.op.api_lease_id,
            completion_wait_supported=True, execution_policy=self.policy,
            controller_fault=getattr(self.op._physical, 'fault', None),
            cube_timing_profile=TIMING_PROFILE if qualified_ramp(type(self.op)) else None,
            motor_owner='session_api')

    def abort(self):
        # Latch the lease before parent Hold/Stop can wait for op.lock. A
        # concurrent preview must not replace its cancellation Event and lose Stop.
        if self.op.api_lease_id is not None:
            self.stopped_identity = (self.op.api_session_id, self.op.api_episode_id, self.op.api_lease_id)
        self.cancel.set()

    def authorized(self, identity):
        op = self.op
        return (self.enabled and op.mode == 'API_ACTIVE' and op.api_authorized
            and identity == (op.api_session_id, op.api_episode_id, op.api_lease_id)
            and identity != self.stopped_identity
            and not getattr(op._physical, 'fault', None))

    def check(self, identity):
        if not self.authorized(identity):
            raise TrajectoryInterrupted('Reset session authority revoked')

    def guard(self):
        if self.policy == 'validated':
            return self.op._safety_guardrails
        if self.policy == 'controller-faults':
            return MotorGuardedPark(self.op._safety_guardrails._config,
                                    forward_kinematics=None, collision_check=None)
        raise ValueError('Unknown cube session execution policy')

    def request(self, envelope):
        identity = tuple(envelope.get(k) for k in ('session_id', 'episode_id', 'lease_id'))
        expires = envelope.get('expires_at')
        if (envelope.get('schema_version') != 1 or not all(isinstance(v, str) and v for v in identity)
                or type(expires) not in (int, float) or not time.time() < expires <= time.time()+121):
            raise ValueError('Malformed or expired reset session request')
        operation, payload = envelope.get('operation'), envelope.get('payload')
        if not isinstance(payload, dict):
            raise ValueError('Reset execution payload must be an object')
        if operation in ('status', 'wait'):
            with self.lock, self.op.lock:
                self.check(identity)
                if payload.get('plan_id') is not None and (
                        self.plan is None or self.plan['id'] != payload['plan_id']
                        or self.plan['identity'] != identity):
                    raise ValueError('Reset completion does not match this lease/plan')
            if operation == 'wait':
                seconds = payload.get('timeout_s', 2.)
                if type(seconds) not in (int, float) or not 0 < seconds <= 5:
                    raise ValueError('Completion wait must be at most five seconds')
                self.finished.wait(seconds)
            with self.lock, self.op.lock:
                self.check(identity)
                return self.status()
        if operation == 'preview':
            return self.preview(identity, payload)
        with self.lock, self.op.lock:
            self.check(identity)
            if operation != 'execute':
                raise ValueError('Unsupported cube session operation')
            plan = self.plan
            if (plan is None or payload.get('plan_id') != plan['id'] or plan['used']
                    or identity != plan['identity'] or self.active
                    or self.op._hardware_pending is not None or self.cancel.is_set()):
                raise ValueError('A matching unused reset plan is required; execution is never retried')
            plan['used'] = True
            self.active = True
            self.finished.clear()
            plan['pending'] = dict(cancel_event=self.cancel, session_id=identity[0],
                episode_id=identity[1], lease_id=identity[2], cube_reset_plan=plan['id'])
            self.op._hardware_pending = plan['pending']
            self.result['state'] = 'preparing'
            threading.Thread(target=self.run, args=(plan,), daemon=True,
                             name='yam-session-cube-pose').start()
            return self.status()

    def preview(self, identity, payload):
        with self.lock:
            with self.op.lock:
                self.check(identity)
                if self.active or self.op._hardware_pending is not None:
                    raise ValueError('Another session command is active')
            if payload.get('execution_policy') != self.policy:
                raise ValueError('Requested reset profile differs from operator selection')
            q, g = np.asarray(payload.get('joints_rad'), float), np.asarray(payload.get('grippers'), float)
            if q.shape != (12,) or not np.isfinite(q).all():
                raise ValueError('Expected twelve finite encoder radians')
            if g.shape != (2,) or not np.isfinite(g).all() or np.any((g < 0) | (g > 1)):
                raise ValueError('Expected two normalized grippers')
            # Feedback/planning must not hold op.lock: Stop can revoke this
            # lease immediately, even if a snapshot or geometry query is slow.
            snapshot = self.op._physical.snapshot()
            start = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
            floor = payload.get('duration_floor_s')
            points = tuple(points_for(self.op, start, q, floor))
            g0 = np.asarray([snapshot.left.gripper, snapshot.right.gripper])
            if not np.isfinite(g0).all() or np.any((g0 < 0) | (g0 > 1)):
                raise ValueError('Fresh normalized gripper feedback is required')
            count = max(len(points), int(np.ceil(np.max(np.abs(g-g0))/.1)), 1)
            points += (points[-1],)*(count-len(points))
            self.guard().approve(start, points, cadence_hz=10.)
            with self.op.lock:
                self.check(identity)
                if self.op._hardware_pending is not None:
                    raise ValueError('Another session command became active during preview')
                self.cancel = threading.Event()
                self.plan = dict(id=uuid.uuid4().hex, identity=identity, target=q,
                    grippers=g, floor=floor, used=False)
                self.result = dict(state='previewed', plan_id=self.plan['id'],
                    duration_s=len(points)/10., duration_floor_s=floor,
                    geometry_checks=self.policy != 'controller-faults')
                return self.status()

    def run(self, plan):
        op = self.op
        try:
            self.check(plan['identity'])
            initial = op._physical.snapshot()
            start = np.asarray(initial.left.joints_rad + initial.right.joints_rad)
            points = tuple(points_for(op, start, plan['target'], plan['floor']))
            g0 = np.array([initial.left.gripper, initial.right.gripper])
            count = max(len(points), int(np.ceil(np.max(np.abs(plan['grippers']-g0))/.1)), 1)
            points += (points[-1],)*(count-len(points))
            grippers = tuple(tuple(g0+(plan['grippers']-g0)*(i/count)) for i in range(1, count+1))
            guard = self.guard()
            approved = guard.approve(start, points, cadence_hz=10.)
            def validate(index, previous, waypoint):
                self.check(plan['identity'])
                if self.cancel.is_set():
                    raise TrajectoryInterrupted('Reset session cancelled')
                return guard.approve(previous, (waypoint,), cadence_hz=10.).waypoints[0]
            with op.lock:
                self.check(plan['identity'])
                if self.cancel.is_set() or op._hardware_pending is not plan['pending']:
                    raise TrajectoryInterrupted('Reset session cancelled before dispatch')
                op.policy_phase = 'cube_reset_executing'
                op.event = op._physical_event = 'Session API executing Reset bin: '+self.policy
                self.result['state'] = 'executing'
                op.auto_queue.invalidate_park()
            warning = None
            try:
                snapshot = op._physical.execute(approved.waypoints, cadence_hz=10.,
                    gripper_waypoints=grippers, validate_waypoint=validate,
                    settle_tolerance_rad=op.settle_tolerance,
                    disable_on_settle_timeout=False, cancel_event=self.cancel)
            except TrajectorySettleTimeout as exc:
                if self.policy != 'controller-faults' or op._physical.state != AdapterState.STOPPED or op._physical.fault:
                    raise
                snapshot, warning = op._physical.snapshot(), str(exc)
            measured = np.asarray(snapshot.left.joints_rad + snapshot.right.joints_rad)
            with self.lock, op.lock:
                self.check(plan['identity'])
                if self.cancel.is_set() or op._hardware_pending is not plan['pending']:
                    raise TrajectoryInterrupted('Reset session cancelled after dispatch')
                op.data.qpos[:12] = measured
                op.data.qvel[:] = 0.
                op.data.ctrl[:12] = plan['target']
                op.target = plan['target'].copy()
                op._hardware_pending = None
                op.policy_phase = 'waiting_for_command'
                error = float(np.max(np.abs(measured-plan['target'])))
                self.result.update(state='holding', measured_rad=measured.tolist(),
                    max_error_rad=error, reached_target=error <= op.settle_tolerance,
                    warning=warning, duration_s=count/10., waypoints=count)
        except Exception as exc:
            with self.lock, op.lock:
                self.result.update(state='interrupted' if self.cancel.is_set() or not self.authorized(plan['identity']) else 'failed', error=str(exc))
                if op._hardware_pending is plan['pending']:
                    op._hardware_pending = None
                    if isinstance(exc, ServoFaultHolding):
                        op._mark_servo_fault_holding(exc)
                    elif self.authorized(plan['identity']):
                        op._hold_physical('cube_session_failure')
        finally:
            with self.lock:
                self.active = False
                self.finished.set()


def install(hardware):
    parent = hardware.PhysicalOperator
    if getattr(parent, '_cube_session_installed', False):
        return
    class CubeSessionOperator(parent):
        _cube_session_installed = True
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.cube_reset_session = CubeSessionExecutor(self)
        def status(self, *args, **kwargs):
            result = super().status(*args, **kwargs)
            executor = getattr(self, 'cube_reset_session', None)
            if executor is not None:
                result['cube_reset_session'] = executor.status()
            return result
        def api_handle_execution(self, payload):
            if payload.get('executor') == 'yam_cube_reset':
                return self.cube_reset_session.request(payload)
            handler = getattr(super(), 'api_handle_execution', None)
            if handler is None:
                raise ValueError('Unsupported executor')
            return handler(payload)
        def _hold_physical(self, reason):
            executor = getattr(self, 'cube_reset_session', None)
            if executor is not None:
                executor.abort()
            return super()._hold_physical(reason)
        def _stop_physical(self, reason):
            executor = getattr(self, 'cube_reset_session', None)
            if executor is not None:
                executor.abort()
            return super()._stop_physical(reason)
    hardware.PhysicalOperator = CubeSessionOperator
    if getattr(hardware.base, 'OperatorSimulator', None) is parent:
        hardware.base.OperatorSimulator = CubeSessionOperator
