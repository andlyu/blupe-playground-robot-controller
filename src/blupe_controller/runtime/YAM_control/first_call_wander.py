"""Controller-owned first-inference motion; no policy waypoints or gripper changes.

The caller must hold an active lease. A start is consumed once per lease, even
when planning is skipped. Finish requests return only after measured settling.
"""
from __future__ import annotations

import threading
import time
import numpy as np

from YAM_control.hardware_safety import SafetyViolation
from YAM_control.i2rt_bimanual_adapter import TrajectoryInterrupted
from YAM_control.ruckig_waypath import MuJoCoWaypathPlanner, plan_joint_trajectory, WaypathPlanningError

CADENCE = 10.0
WAIT_SECONDS = 10.0
SETTLE_TOLERANCE = 0.02


def plan_wander(model_path, config, guard, start):
    """Choose the largest complete, bounded round trip that fits ten seconds.

    IK preserves both initial tool orientations. Smaller candidates are planned
    independently; no unsafe packet is clipped or sent to the driver.
    """
    planner = MuJoCoWaypathPlanner(model_path, config.joint_lower_rad, config.joint_upper_rad)
    start = np.asarray(start, dtype=float)
    planner._set_joints(start)
    poses = [(planner.data.body(side+'_grasp').xpos.copy(),
              planner.data.body(side+'_grasp').xmat.reshape(3, 3).copy())
             for side in ('left', 'right')]
    reason = 'no_feasible_path'
    for scale in (1., .8, .6, .4, .25, .15, .08):
        try:
            seed = start.copy()
            keys = [start.copy()]
            # +x is forward in the shared bimanual base frame. Keep the
            # sideways component small, with the right arm following the left.
            for offsets in (((.24, .025, .08), (.025, -.005, .01)),
                            ((.44, .04, .12), (.44, -.04, .12))):
                for arm, side in enumerate(('left', 'right')):
                    position, rotation = poses[arm]
                    seed = planner._solve_pose(seed, side+'_grasp', slice(arm*6, arm*6+6),
                                               position + scale*np.asarray(offsets[arm]), rotation)
                keys.append(seed.copy())
            path = plan_joint_trajectory(keys, cadence_hz=CADENCE,
                max_velocity_rad_s=min(.25, config.max_joint_velocity_rad_s))
            if path.duration_s > WAIT_SECONDS:
                reason = 'outbound_duration_exceeds_ten_seconds'
                continue
            # Stretch a safe short path over the full waiting window instead of
            # reaching the endpoint quickly and sitting still.
            samples = np.vstack((start, np.asarray(path.positions)))
            old_times = np.linspace(0., WAIT_SECONDS, len(samples))
            new_times = np.arange(1, round(WAIT_SECONDS*CADENCE)+1)/CADENCE
            outward = tuple(tuple(float(np.interp(t,old_times,samples[:,j])) for j in range(12))
                            for t in new_times)
            reverse = tuple(reversed((tuple(start),) + outward[:-1]))
            guard.approve(start, outward + reverse, cadence_hz=CADENCE)
            return outward, scale
        except (SafetyViolation, WaypathPlanningError, ValueError) as exc:
            reason = str(exc)
    raise ValueError('Wandering skipped: ' + reason)


class FirstCallWander:
    def __init__(self, adapter, guard, config, model_path, authorized, planner=plan_wander):
        self.adapter, self.guard, self.config = adapter, guard, config
        self.model_path, self.authorized, self.planner = model_path, authorized, planner
        self.lock = threading.RLock()
        self.identity = None
        self.state = 'idle'
        self.reason = None
        self.scale = None
        self.finish_event = threading.Event()
        self.abort_event = threading.Event()
        self.done = threading.Event()
        self.done.set()

    def status(self):
        with self.lock:
            return dict(state=self.state, reason=self.reason, distance_scale=self.scale,
                        settled=self.state in {'returned', 'skipped'})

    @property
    def blocks_motion(self):
        with self.lock:
            return self.state not in {'idle', 'returned', 'skipped'}

    def start(self, identity):
        with self.lock:
            if identity == self.identity:
                return self.status()
            if not self.done.is_set():
                raise ValueError('Previous wandering phase is still active')
            if not self.authorized(identity):
                raise ValueError('First-call wandering requires the active unused lease')
            self.identity, self.state, self.reason, self.scale = identity, 'planning', None, None
            self.finish_event.clear(); self.abort_event.clear(); self.done.clear()
            threading.Thread(target=self._run, args=(identity,), daemon=True,
                             name='yam-first-call-wander').start()
            return self.status()

    def finish(self, identity, timeout=35.):
        with self.lock:
            if identity != self.identity:
                raise ValueError('Wandering lease mismatch')
            self.finish_event.set()
        if not self.done.wait(timeout):
            # Never infer completion or resend a timed-out movement.
            raise RuntimeError('Wandering return has not been verified; policy motion blocked')
        result = self.status()
        if not result['settled']:
            raise RuntimeError('Wandering did not return: '+str(result['reason'] or result['state']))
        return result

    def abort(self):
        self.abort_event.set()
        self.finish_event.set()

    def _check_authority(self, identity):
        if self.abort_event.is_set() or not self.authorized(identity):
            raise TrajectoryInterrupted('Wandering authority revoked; no automatic return')

    def _run(self, identity):
        timer = None
        moved = False
        try:
            snapshot = self.adapter.snapshot()
            start = tuple(snapshot.left.joints_rad + snapshot.right.joints_rad)
            grippers = (snapshot.left.gripper, snapshot.right.gripper)
            # Closed/partly closed fingers may hold an object. Never open them.
            if min(grippers) < .95:
                with self.lock:
                    self.state, self.reason = 'skipped', 'grippers_not_open'
                return
            began = time.monotonic()
            outward, scale = self.planner(self.model_path, self.config, self.guard, start)
            self._check_authority(identity)
            if self.finish_event.is_set() or time.monotonic()-began >= WAIT_SECONDS:
                with self.lock:
                    self.state, self.reason = 'skipped', 'model_ready_before_motion'
                return
            with self.lock:
                self.state, self.scale = 'wandering', scale
            timer = threading.Timer(max(0., WAIT_SECONDS-(time.monotonic()-began)), self.finish_event.set)
            timer.daemon = True; timer.start()
            progress = [-1]
            def validate(index, previous, target):
                self._check_authority(identity)
                return self.guard.approve(previous, (target,), cadence_hz=CADENCE).waypoints[0]
            moved = True
            try:
                self.adapter.execute(outward, cadence_hz=CADENCE,
                    gripper_waypoints=(grippers,)*len(outward),
                    on_waypoint=lambda index: progress.__setitem__(0, index),
                    validate_waypoint=validate, cancel_event=self.finish_event,
                    settle_tolerance_rad=SETTLE_TOLERANCE, disable_on_settle_timeout=False)
                self.finish_event.wait(max(0., WAIT_SECONDS-(time.monotonic()-began)))
            except TrajectoryInterrupted:
                if not self.finish_event.is_set():
                    raise
            self._check_authority(identity)
            with self.lock:
                self.state = 'returning'
            snapshot = self.adapter.snapshot()
            measured = tuple(snapshot.left.joints_rad + snapshot.right.joints_rad)
            # Reconnect measured pose to the last reached path point, then retrace.
            # Validate the actual return, including the reconnect, before dispatch.
            last = outward[progress[0]] if progress[0] >= 0 else start
            connector = ()
            if max(abs(a-b) for a,b in zip(measured,last)) > 1e-6:
                connector = plan_joint_trajectory((measured,last), cadence_hz=CADENCE,
                    max_velocity_rad_s=min(.25,self.config.max_joint_velocity_rad_s)).positions
            reverse = tuple(reversed((start,)+outward[:max(0,progress[0])]))
            returning = tuple(connector) + reverse
            self.guard.approve(measured, returning, cadence_hz=CADENCE)
            result = self.adapter.execute(returning, cadence_hz=CADENCE,
                gripper_waypoints=(grippers,)*len(returning), validate_waypoint=validate,
                cancel_event=self.abort_event, settle_tolerance_rad=SETTLE_TOLERANCE,
                disable_on_settle_timeout=False)
            self._check_authority(identity)
            measured = result.left.joints_rad + result.right.joints_rad
            if max(abs(a-b) for a,b in zip(measured,start)) > SETTLE_TOLERANCE:
                raise RuntimeError('Wandering return pose did not settle')
            with self.lock:
                self.state = 'returned'
        except Exception as exc:
            with self.lock:
                # A preflight failure is a harmless skipped animation. After any
                # dispatch, uncertainty blocks all policy motion for this lease.
                self.state = 'failed' if moved or self.abort_event.is_set() else 'skipped'
                self.reason = str(exc)
        finally:
            if timer:
                timer.cancel()
            self.done.set()
