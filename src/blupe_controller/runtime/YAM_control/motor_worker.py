"""Minimal motor owner. No operator, rendering, camera, history or recorder imports."""
from __future__ import annotations
import itertools
import logging
import os
import queue
import sys
import threading
import time
from multiprocessing.connection import Connection
# Install before hardware imports so their basicConfig cannot add a synchronous sink.
_LOG_BUFFER = None
if __name__ == '__main__':
    from YAM_control.nonblocking_logging import install
    _LOG_BUFFER = install()

from YAM_control.isolated_adapter import Transport, LINK_TIMEOUT, error_record, raise_remote
from YAM_control.i2rt_bimanual_adapter import I2RTBimanualAdapter, AdapterState, TrajectoryInterrupted

LOGGER = logging.getLogger(__name__)


class Worker:
    def __init__(self, transport):
        self.transport = transport
        self.adapter = None
        self.last_heartbeat = time.monotonic()
        self.link_fault = None
        self.active = {}
        self.active_methods = {}
        self.heartbeat_stable_since = None
        from YAM_control.operator_link_pause import OperatorLinkPause
        self.link_pause = OperatorLinkPause(self)
        self.callbacks = {}
        self.lock = threading.Lock()
        self.slots = threading.BoundedSemaphore(4)
        self.urgent_slots = threading.BoundedSemaphore(2)
        self.callback_ids = itertools.count(1)
        self.closed = threading.Event()
        self.owner_lock = None

    def status(self):
        a = self.adapter
        # Plain small state reads: never wait behind launch/stop's adapter lock.
        return {'monotonic': time.monotonic(), 'state': 'FAULT' if self.link_fault else a._state.value,
                'fault': self.link_fault or a._fault, 'link_fault': self.link_fault, 'torque_off_verified': a.torque_off_verified,
                'recovery_status': a.recovery_status, 'recovery_allowed': a.recovery_allowed,
                'last_command': a._last_command, 'timing': a.timing_status()['arms'],
                'logging': _LOG_BUFFER.stats() if _LOG_BUFFER else None}

    def latch_link_fault(self, detail=None):
        if self.link_fault or self.adapter is None:
            return
        a = self.adapter
        # Set cancellation without waiting for a motion/initialization lock.
        a._stop_requested.set()
        a.recovery_allowed = False
        with self.lock:
            for cancel in self.active.values():
                cancel.set()
        if a._robots or a._state == AdapterState.INITIALIZING:
            self.link_fault = 'Operator link stalled; motion canceled; existing motor holds retained'
            if detail:
                self.link_fault += f' ({detail})'
            LOGGER.error('[motor-control] %s; cause=%s heartbeat_age_ms=%.1f transport_failed=%s',
                         self.link_fault, detail or getattr(self.transport, 'failure_reason', None) or 'unspecified',
                         (time.monotonic()-self.last_heartbeat)*1000, self.transport.failed.is_set())

    def monitor(self):
        next_log = 0
        while not self.closed.wait(.1):
            if self.transport.failed.is_set():
                self.latch_link_fault('transport failed: '+str(self.transport.failure_reason))
            elif time.monotonic()-self.last_heartbeat > LINK_TIMEOUT and self.adapter is not None:
                if self.adapter._robots and self.adapter._state in {AdapterState.STOPPED, AdapterState.EXECUTING}:
                    self.link_pause.request('operator heartbeat delayed')
                else:
                    self.latch_link_fault('heartbeat expired')
            self.link_pause.poll()
            if self.adapter is not None and not self.transport.failed.is_set():
                try:
                    status = self.status()
                    self.transport.send({'kind': 'status', 'status': status})
                    if time.monotonic() >= next_log:
                        for arm, timing in status['timing'].items():
                            LOGGER.info('[motor-control] arm=%s command_age_ms=%s feedback_age_ms=%s max_gap_ms=%s updates_over_50ms=%s updates_over_100ms=%s',
                                        arm, timing.get('command_age_ms'), timing.get('feedback_age_ms'),
                                        timing.get('max_gap_ms'), timing.get('updates_over_50ms'), timing.get('updates_over_100ms'))
                        next_log = time.monotonic()+5
                except (RuntimeError, ValueError):
                    self.latch_link_fault()

    def callback(self, request_id, name, cancel, *args):
        callback_id = next(self.callback_ids)
        inbox = queue.Queue(maxsize=1)
        with self.lock:
            self.callbacks[callback_id] = inbox
        try:
            self.transport.send({'kind': 'callback', 'id': request_id, 'callback_id': callback_id,
                                 'name': name, 'args': args})
            deadline = time.monotonic() + LINK_TIMEOUT
            while True:
                if cancel.is_set() or self.adapter._stop_requested.is_set():
                    raise TrajectoryInterrupted('Operator callback canceled; retaining last target')
                if time.monotonic() >= deadline:
                    self.link_pause.request('operator callback delayed: '+name)
                    if self.link_fault or time.monotonic()-deadline >= self.link_pause.MAX_PAUSE_S:
                        self.latch_link_fault('operator callback did not recover within 10 seconds: '+name)
                        raise TrajectoryInterrupted('Operator callback expired; no resume')
                try:
                    result = inbox.get(timeout=.02)
                    if 'error' in result:
                        raise_remote(result['error'])
                    return result.get('value')
                except queue.Empty:
                    pass
        finally:
            with self.lock:
                self.callbacks.pop(callback_id, None)

    def configure(self, config, simulated, lock_path):
        if self.adapter is not None:
            raise RuntimeError('Motor worker already configured')
        native_library = config.pop('native_sim_library', None)
        native_can_library = config.pop('native_can_library', None) or os.environ.get('YAM_NATIVE_CAN_LIBRARY')
        if native_can_library and simulated:
            raise ValueError('Native CAN requires physical mode')
        if native_library and not simulated:
            raise ValueError('Native prototype is simulation-only')
        if not simulated:
            import fcntl
            self.owner_lock = open(lock_path or f'/tmp/yam-motor-worker-{os.getuid()}.lock', 'a')
            fcntl.flock(self.owner_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if simulated:
            from YAM_control.simulated_driver import SimulatedBus
            from YAM_control.motor_shutdown import disable_motorchain
            if native_library:
                from YAM_control.native_simulated_driver import NativeSimulatedBus
                self.sim_bus = NativeSimulatedBus(native_library)
            else:
                self.sim_bus = SimulatedBus()
            config.update(robot_factory=self.sim_bus.factory, robot_disabler=disable_motorchain,
                          channel_ready=lambda _: True)
        if native_can_library:
            from YAM_control.native_can_driver import robot_factory
            config['robot_factory'] = robot_factory(native_can_library)
        self.adapter = I2RTBimanualAdapter(**config)
        self.adapter._operator_link_guard = self.link_pause.guard
        self.last_heartbeat = time.monotonic()

    def recover_operator_link(self, request_id, cancel):
        """Explicit manual recovery only; no servo reset, enable, or replay."""
        a = self.adapter
        def link_ready():
            now = time.monotonic()
            if (cancel.is_set() or self.transport.failed.is_set()
                    or now-self.last_heartbeat > .3
                    or self.heartbeat_stable_since is None
                    or now-self.heartbeat_stable_since < 1.):
                raise TrajectoryInterrupted('Operator link recovery requires one second of fresh heartbeats')
        link_ready()
        if not self.link_fault:
            return False
        with self.lock:
            if self.callbacks or any(mid != request_id and self.active_methods.get(mid) in
                    {'launch','execute','stop','hold_measured','recover_operator_link'} for mid in self.active):
                raise TrajectoryInterrupted('Wait for canceled motor operations to finish before link recovery')
        with a._lock:
            if a._state != AdapterState.STOPPED or a._fault or set(a._robots) != {'left','right'}:
                raise RuntimeError('Operator link recovery requires both arms holding without a motor fault')
            link_ready()
            # hold_measured validates BOTH arms, including fresh normal feedback,
            # before replacing targets. Never clear a native/motor fault latch.
            a._stop_requested.clear()
            try:
                result = a.hold_measured(cancel_event=cancel)
                link_ready()
            except Exception:
                a._stop_requested.set()
                raise
            self.link_fault = None
            from YAM_control.operator_link_pause import OperatorLinkPause
            self.link_pause = OperatorLinkPause(self)
            a._operator_link_guard = self.link_pause.guard
            a.recovery_allowed = False  # A new manual action owns continuation.
        LOGGER.info('[motor-control] Operator link recovered after verified measured hold')
        return result

    def run_call(self, message, cancel, reserved):
        method, request_id = message['method'], message['id']
        reply = {'kind': 'result', 'id': request_id}
        try:
            args, kwargs = message['args'], message['kwargs']
            if method == 'configure':
                self.configure(*args, **kwargs)
            elif self.adapter is None:
                raise RuntimeError('Motor worker not configured')
            elif method == 'close':
                if self.adapter._robots or self.adapter._state not in {AdapterState.DISABLED, AdapterState.FAULT}:
                    raise RuntimeError('Cannot close motor worker before verified driver shutdown')
                self.closed.set()
            elif method == 'recover_operator_link':
                reply['value'] = self.recover_operator_link(request_id, cancel)
            elif method == 'set_recovery_allowed':
                if args[0] and self.link_fault:
                    raise TrajectoryInterrupted(self.link_fault)
                self.adapter.recovery_allowed = bool(args[0])
            elif method == 'trace_head':
                from YAM_control.motor_diagnostics import snapshot
                reply['value'] = {'schema_version': 1, 'captured_at': time.time(),
                    'captured_monotonic': time.monotonic(), 'requested_joint_command': self.adapter.last_command,
                    'arms': {arm: snapshot(robot, include_trace=False)
                             for arm, robot in tuple(self.adapter._robots.items())}}
            elif method == 'trace_rows':
                arm, after, end = args
                trace = getattr(self.adapter._robots.get(arm), '_yam_motor_trace', None)
                reply['value'] = trace.page(after, end) if trace else []
            elif method in {'launch', 'execute', 'snapshot', 'hold', 'stop', 'hold_measured', 'diagnostic_event'}:
                if method in {'launch', 'execute', 'hold_measured'}:
                    if self.link_fault or self.transport.failed.is_set():
                        raise TrajectoryInterrupted(self.link_fault)
                    if time.monotonic()-message['sent'] > LINK_TIMEOUT:
                        raise TrajectoryInterrupted('Expired motor request rejected')
                    kwargs['cancel_event'] = cancel
                for name in message['callbacks']:
                    kwargs[name] = lambda *a, name=name: self.callback(request_id, name, cancel, *a)
                reply['value'] = getattr(self.adapter, method)(*args, **kwargs)
                if method == 'launch' or (method == 'stop' and self.adapter.torque_off_verified):
                    from YAM_control.operator_link_pause import OperatorLinkPause
                    self.link_pause = OperatorLinkPause(self)
                    self.adapter._operator_link_guard = self.link_pause.guard
                if method == 'stop' and self.adapter.torque_off_verified:
                    self.link_fault = None
            else:
                raise ValueError('Unsupported motor operation')
        except Exception as exc:
            reply['error'] = error_record(exc)
            if method == 'configure':
                self.closed.set()
        finally:
            with self.lock:
                self.active.pop(request_id, None)
                self.active_methods.pop(request_id, None)
            if reserved:
                self.slots.release()
            else:
                self.urgent_slots.release()
        if self.adapter is not None:
            reply['status'] = self.status()
        try:
            self.transport.send(reply)
        except (RuntimeError, ValueError):
            self.latch_link_fault()

    def publish_feedback(self):
        # One reader for UI, recorder, station and validation. Never consumes RPC
        # slots or blocks the CAN loop; snapshots retain adapter health checks.
        while not self.closed.is_set():
            a = self.adapter
            if a is not None and a._state in {AdapterState.STOPPED, AdapterState.EXECUTING}:
                packet = {'kind': 'feedback', 'monotonic': time.monotonic()}
                try:
                    packet['value'] = a.snapshot()
                except Exception as exc:
                    packet['error'] = error_record(exc)
                if not self.transport.failed.is_set():
                    self.transport.send_latest(packet)
            self.closed.wait(.02)

    def run(self):
        threading.Thread(target=self.monitor, daemon=True, name='motor-link-monitor').start()
        threading.Thread(target=self.publish_feedback, daemon=True, name='motor-feedback').start()
        while not self.closed.is_set():
            try:
                if not self.transport.reader.poll(.1):
                    continue
                message = self.transport.receive()
            except (OSError, EOFError, ValueError):
                self.transport.failed.set()
                self.latch_link_fault()
                # Do not drop torque just because the operator process died.
                if self.adapter is None or not self.adapter._robots:
                    return
                self.closed.wait()
                return
            kind = message['kind']
            if kind == 'heartbeat':
                # A buffered heartbeat is not proof the operator is responsive.
                if 0 <= time.monotonic()-message['sent'] <= .5:
                    now = time.monotonic()
                    if self.heartbeat_stable_since is None or now-self.last_heartbeat > .5:
                        self.heartbeat_stable_since = now
                    self.last_heartbeat = now
            elif kind == 'cancel':
                with self.lock:
                    cancel = self.active.get(message['id'])
                    if cancel:
                        cancel.set()
            elif kind == 'callback_result':
                with self.lock:
                    inbox = self.callbacks.get(message['callback_id'])
                    if inbox and inbox.empty():
                        inbox.put_nowait(message)
            elif kind == 'call':
                # Hold/Stop cancellation is not queued behind trajectory callbacks.
                urgent = message['method'] in {'hold', 'stop'}
                if urgent and self.adapter:
                    self.adapter._stop_requested.set()
                    with self.lock:
                        for cancel in self.active.values():
                            cancel.set()
                reserved = not urgent
                capacity = self.slots if reserved else self.urgent_slots
                if not capacity.acquire(blocking=False):
                    self.transport.send({'kind': 'result', 'id': message['id'],
                        'error': error_record(RuntimeError('Motor operation capacity exceeded'))})
                    continue
                cancel = threading.Event()
                with self.lock:
                    self.active[message['id']] = cancel
                    self.active_methods[message['id']] = message['method']
                threading.Thread(target=self.run_call, args=(message, cancel, reserved), daemon=True,
                                 name='motor-operation').start()
        # Allow the final close acknowledgement to leave the writer queue.
        deadline = time.monotonic() + 1
        while not self.transport.out.empty() and time.monotonic() < deadline:
            time.sleep(.01)


if __name__ == '__main__':
    worker = Worker(Transport(Connection(int(sys.argv[1]), readable=True, writable=False),
                              Connection(int(sys.argv[2]), readable=False, writable=True)))
    worker.run()
