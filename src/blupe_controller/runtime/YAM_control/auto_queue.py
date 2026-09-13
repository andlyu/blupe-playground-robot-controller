"""Operator-enabled FIFO startup; only a verified park permits another launch."""
from __future__ import annotations

import json
import logging
from pathlib import Path
import threading
import time

from YAM_control.i2rt_bimanual_adapter import AdapterState, TrajectoryInterrupted

LOGGER = logging.getLogger(__name__)


class AutoQueue:
    def __init__(self, operator, state_path=None):
        self.operator = operator
        self.lock = threading.RLock()
        self.state_path = Path(state_path) if state_path else None
        try:
            self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        except OSError:
            self.boot_id = None
        self.enabled = False
        self.parked = False
        self.running = False
        self.completion_pending = False
        self.ready_deadline = None
        self.cancel = threading.Event()
        self.thread = None
        self.message = 'Paused'
        if self.state_path:
            try:
                saved = json.loads(self.state_path.read_text())
                # A reboot or interrupted startup requires another operator park.
                if self.boot_id and saved.get('boot_id') == self.boot_id:
                    self.parked = saved.get('parked') is True
                    self.enabled = saved.get('enabled') is True and self.parked
                self.message = 'Waiting for queue' if self.enabled else 'Paused'
            except (OSError, ValueError, AttributeError):
                pass

    def _save(self):
        if self.state_path:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.state_path.with_suffix('.tmp')
            temporary.write_text(json.dumps({
                'boot_id': self.boot_id, 'enabled': self.enabled, 'parked': self.parked,
            }))
            temporary.replace(self.state_path)

    def snapshot(self):
        with self.lock:
            return dict(enabled=self.enabled, running=self.running,
                        parked=self.parked, message=self.message)

    def _message(self, message):
        with self.lock:
            if self.message != message:
                LOGGER.info('[auto-queue] %s', message)
                self.message = message

    def enable(self):
        with self.lock:
            if self.running:
                return 'Automatic startup is still finishing; wait before enabling'
            if not self.parked and not self.enabled:
                return f'Enable blocked: use Park at Zero first. {self.message}'
            self.enabled = True
            try:
                self._save()
            except OSError:
                self.enabled = False
                raise
            self._message('Enabled; waiting for a parked station and queued runner')
        return self.message

    def pause(self, reason='Paused by operator', *, clear_park=False):
        # No motor lock: stop must interrupt an in-progress launch/Home immediately.
        with self.lock:
            self.enabled = False
            self.completion_pending = False
            self.cancel.set()
            if clear_park:
                self.parked = False
            self._message(reason)
            try:
                self._save()
            except OSError:
                LOGGER.exception('[auto-queue] Could not persist pause')
        op = self.operator
        if op is not None:
            with op.lock:
                if self.ready_deadline is not None and op.api_lease_id is None:
                    op.api_authorized = False
                    if op.mode == 'API_WAITING':
                        op.mode = 'READY'
                        op.policy_phase = 'automatic_handoff_paused'
                    self.ready_deadline = None
        return self.message

    def mark_parked(self):
        with self.lock:
            self.parked = True
            self._save()

    def invalidate_park(self):
        with self.lock:
            self.parked = False
            # Persist before enabling drivers, so a crash cannot replay a park receipt.
            self._save()

    def start(self):
        if self.thread is None:
            self.thread = threading.Thread(target=self._loop, daemon=True, name='yam-auto-queue')
            self.thread.start()

    def _loop(self):
        while not self.operator.stop_event.is_set():
            try:
                self.tick()
            except Exception as exc:
                self.pause(f'Automatic queue paused: {type(exc).__name__}')
                LOGGER.exception('[auto-queue] Startup failed closed')
            self.operator.stop_event.wait(1.0)

    def _eligible(self):
        op = self.operator
        with op.lock:
            return (
                op.mode == 'DISABLED' and op._physical.state == AdapterState.DISABLED
                and op._physical.fault is None and op.controller_ok and op.api_connected
                and op.api_lease_id is None and not op.api_authorized
                and op._hardware_pending is None and op._policy_deadline is None
                and not op.cleanup_state['running'] and not op.stop_event.is_set()
            )

    def policy_completed(self):
        with self.lock:
            # Completion cleanup must run even when automatic starts are paused.
            self.completion_pending = True
            self.ready_deadline = None
            self._message('Run ended; checking the next queued run')

    def _completion_eligible(self):
        op = self.operator
        with op.lock:
            return (self.completion_pending and op.mode == 'STOPPED'
                    and op._physical.state == AdapterState.STOPPED
                    and op._physical.fault is None and op.controller_ok
                    and op.api_lease_id is None and not op.api_authorized
                    and op._hardware_pending is None and op._policy_deadline is None
                    and not op.cleanup_state['running'] and not op.stop_event.is_set())

    def _queue_waiting(self):
        queue = self.operator.queue_snapshot()
        if queue.get('error'):
            raise RuntimeError('Session API queue unavailable')
        return bool(queue['entries'])

    def tick(self):
        op = self.operator
        with self.lock:
            if (not self.enabled and not self.completion_pending) or self.running:
                return
            parked = self.parked
            ready_deadline = self.ready_deadline
        if op._physical.state == AdapterState.FAULT or op.mode == 'FAULT':
            self.pause('Controller fault; automatic queue paused', clear_park=True)
            return
        if ready_deadline is not None:
            try:
                cleared = not self._queue_waiting()
            except RuntimeError:
                cleared = False
            if not cleared and time.monotonic() < ready_deadline:
                return
            # Serialize with assignment: a late prepare cannot acquire a lease
            # after readiness is withdrawn, or during the guarded park.
            if not op._physical_lock.acquire(blocking=False):
                return
            try:
                with self.lock:
                    if self.ready_deadline != ready_deadline or not self.enabled:
                        return
                    with op.lock:
                        if op.api_lease_id is not None:
                            return
                        op.api_authorized = False
                        op.mode = 'READY'
                        self.ready_deadline = None
                    self.running = True
                    cancel = self.cancel
                try:
                    self._return_to_rest(cancel, 'Queue cleared' if cleared else 'Session API handoff timed out')
                finally:
                    with self.lock:
                        self.running = False
                if not cleared:
                    self.pause('Session API handoff timed out; parked, automatic queue paused')
            finally:
                op._physical_lock.release()
            return
        continuation = self._completion_eligible()
        if not continuation and (not parked or not self._eligible()):
            with op.lock:
                if op.api_lease_id is not None:
                    message = 'Policy running through Session API'
                elif op._policy_deadline is not None:
                    message = 'Session ended; holding until the runtime watchdog parks at zero'
                elif not parked:
                    message = 'Waiting for Park at Zero before the next automatic launch'
                else:
                    message = 'Waiting for Session API connection and an idle station'
            self._message(message)
            return
        # Read failures while idle do not touch the robot; retry the next poll.
        try:
            waiting = self._queue_waiting()
        except RuntimeError:
            if not continuation:
                self._message('Waiting for Session API queue connection')
                return
            waiting = False  # A cloud outage must not prevent local parking.
        if not waiting and not continuation:
            self._message('Waiting for queue')
            return
        if not op._physical_lock.acquire(blocking=False):
            return
        try:
            with self.lock:
                if (not self.enabled and not continuation) or self.running or not (self._completion_eligible() if continuation else self.parked and self._eligible()):
                    return
                self.running = True
                self.cancel = threading.Event()
                cancel = self.cancel
            try:
                self.completion_pending = False
                self.invalidate_park()
                if continuation:
                    try:
                        waiting = self.enabled and op.api_connected and self._queue_waiting()
                    except RuntimeError:
                        waiting = False
                    if not waiting:
                        self._return_to_rest(cancel, 'Run ended; no available queued run')
                        return
                    self._message('Run ended; preparing the next queued run')
                else:
                    self._message('Queue detected; launching both arms')
                    op._launch_physical(cancel_event=cancel)
                self._check(cancel)
                # Torque-off arms may settle away from zero. Launch holds fresh
                # feedback; _move_home validates the complete path from that
                # measured pose with the same guardrails as manual Home.
                # Cancellation while drivers initialized must not cause Home motion.
                if not self._queue_waiting():
                    self._return_to_rest(cancel, 'Queue cleared before Home')
                    return
                self._message('Moving both arms to Home')
                result = op._move_home(cancel_event=cancel)
                self._check(cancel)
                status = op.status()
                if status['mode'] != 'READY' or not status['homed']:
                    raise RuntimeError(result)
                # Re-read after Home: never leave readiness armed for an empty queue.
                if not self._queue_waiting():
                    self._return_to_rest(cancel, 'Queue cleared while moving Home')
                    return
                self._check(cancel)
                # The same lock protects stop's cancellation and the readiness grant.
                with self.lock:
                    self._check(cancel)
                    result = op._authorize_automatic_policy()
                    if not result.startswith('API handoff authorized'):
                        raise RuntimeError(result)
                    self.ready_deadline = time.monotonic() + 30.0
                    self._message('Home confirmed; policy handed to Session API')
            except TrajectoryInterrupted:
                # Emergency Stop owns torque-off; a simple auto-queue pause holds.
                if op._physical.state in {AdapterState.STOPPED, AdapterState.EXECUTING}:
                    op._hold_physical('automatic_start_cancelled')
                self._message('Automatic startup cancelled')
            except Exception as exc:
                self.pause(f'Automatic queue paused: {exc}', clear_park=True)
                if op._physical.state in {AdapterState.STOPPED, AdapterState.EXECUTING}:
                    op._hold_physical('automatic_start_failed')
                LOGGER.exception('[auto-queue] Automatic startup failed')
            finally:
                with self.lock:
                    self.running = False
        finally:
            op._physical_lock.release()

    def _check(self, cancel):
        if cancel.is_set() or self.operator.stop_event.is_set():
            raise TrajectoryInterrupted('Automatic startup cancelled')
        if not self.operator.api_connected:
            raise RuntimeError('Session API disconnected during startup')

    def _return_to_rest(self, cancel, reason):
        if cancel.is_set() or self.operator.stop_event.is_set():
            raise TrajectoryInterrupted('Automatic parking cancelled')
        result = self.operator._park_zero_and_stop(cancel_event=cancel)
        if cancel.is_set() or self.operator.stop_event.is_set():
            raise TrajectoryInterrupted('Automatic parking cancelled')
        if not self.parked:
            raise RuntimeError(result)
        self._message(f'{reason}; parked, waiting for queue')
