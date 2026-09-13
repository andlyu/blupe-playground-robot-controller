"""Private process boundary for physical control (no HTTP/rendering in worker).

CPython 3.11 subprocess pass_fds + Connection: docs/refs/python-process.
Pickle is confined to inherited anonymous pipes between our own two processes;
there is no listening socket or externally supplied pickle. One writer per pipe.
"""
from __future__ import annotations
import itertools
import os
import pickle
import queue
import subprocess
import sys
import threading
import time
from multiprocessing.connection import Connection
from pathlib import Path
from YAM_control.shared_feedback import SharedFeedback, FeedbackUnavailable
from YAM_control.i2rt_bimanual_adapter import (
    AdapterState, TrajectoryInterrupted, TrajectorySettleTimeout,
    ServoFaultHolding, WaypointSafetyRejected,
)

MAX_MESSAGE = 256 * 1024
LINK_TIMEOUT = 1.0


def error_record(exc):
    return {'type': type(exc).__name__, 'message': str(exc),
            'code': getattr(exc, 'code', None), 'waypoint': getattr(exc, 'waypoint', None)}


def raise_remote(error):
    if error['type'] == 'WaypointSafetyRejected':
        raise WaypointSafetyRejected(error['waypoint'], error['code'])
    cls = {'TrajectoryInterrupted': TrajectoryInterrupted,
           'TrajectorySettleTimeout': TrajectorySettleTimeout,
           'ServoFaultHolding': ServoFaultHolding, 'ValueError': ValueError,
           'RuntimeError': RuntimeError}.get(error['type'], RuntimeError)
    raise cls(error['message'])


class Transport:
    """Bounded outgoing queue: a paused reader never blocks motor control."""
    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer
        self.out = queue.Queue(maxsize=32)
        self.failed = threading.Event()
        self._latest_lock = threading.Lock()
        self._latest = {}
        self.failure_reason = None
        threading.Thread(target=self._write, daemon=True, name='motor-ipc-write').start()

    def fail(self, reason):
        if self.failure_reason is None:
            self.failure_reason = reason
        self.failed.set()

    def send(self, message):
        if message.get('kind') in {'status', 'heartbeat'}:
            self.send_latest(message)
            return
        if self.failed.is_set():
            raise RuntimeError('Motor process link unavailable')
        data = pickle.dumps(message, protocol=4)
        if len(data) > MAX_MESSAGE:
            raise ValueError('Motor IPC message exceeds bounded payload limit')
        try:
            self.out.put_nowait(data)
        except queue.Full:
            self.fail(f"reliable_queue_full: kind={message.get('kind')} queued={self.out.qsize()}")
            raise RuntimeError('Motor process link backpressure')

    def send_latest(self, message):
        # Telemetry is replaceable; it must never fill the reliable RPC queue.
        if self.failed.is_set():
            raise RuntimeError('Motor process link unavailable')
        kind = message.get('kind')
        if kind not in {'feedback', 'status', 'heartbeat'}:
            raise ValueError('Only replaceable telemetry may be coalesced')
        data = pickle.dumps(message, protocol=4)
        if len(data) > MAX_MESSAGE:
            raise ValueError('Motor telemetry exceeds bounded payload limit')
        with self._latest_lock:
            self._latest[kind] = data

    def _write(self):
        try:
            while not self.failed.is_set():
                try:
                    data = self.out.get(timeout=.01)
                except queue.Empty:
                    with self._latest_lock:
                        key = next(iter(self._latest), None)
                        data = self._latest.pop(key) if key is not None else None
                    if data is None:
                        continue
                self.writer.send_bytes(data)
        except (OSError, EOFError) as exc:
            self.fail(f"write_failed: {type(exc).__name__} errno={getattr(exc, 'errno', None)}")

    def receive(self):
        try:
            return pickle.loads(self.reader.recv_bytes(MAX_MESSAGE))
        except (OSError, EOFError, ValueError) as exc:
            self.fail(f"read_failed: {type(exc).__name__} errno={getattr(exc, 'errno', None)}")
            raise


class IsolatedAdapter:
    def __init__(self, *, simulated=False, lock_path=None, **config):
        self._feedback = SharedFeedback()
        self._ids = itertools.count(1)
        self._pending = {}
        self._pending_lock = threading.Lock()
        self._status = {'state': 'DISABLED', 'fault': None, 'torque_off_verified': False,
                        'recovery_allowed': False, 'recovery_status': None, 'last_command': None,
                        'timing': {}, 'monotonic': time.monotonic()}
        self._closed = False
        to_child_read, to_child_write = os.pipe()
        to_parent_read, to_parent_write = os.pipe()
        root = str(Path(__file__).resolve().parents[1])
        env = dict(os.environ)
        env['PYTHONPATH'] = root + os.pathsep + env.get('PYTHONPATH', '')
        # Exec a minimal module, never fork the multi-threaded operator runtime.
        try:
            self.process = subprocess.Popen(
                [sys.executable, '-m', 'YAM_control.motor_worker',
                 str(to_child_read), str(to_parent_write)],
                pass_fds=(to_child_read, to_parent_write), env=env, cwd=root)
        finally:
            os.close(to_child_read)
            os.close(to_parent_write)
        self._transport = Transport(Connection(to_parent_read, readable=True, writable=False),
                                    Connection(to_child_write, readable=False, writable=True))
        threading.Thread(target=self._read, daemon=True, name='motor-ipc-read').start()
        try:
            self._call('configure', config, simulated=simulated, lock_path=lock_path, timeout=10)
        except Exception:
            self._closed = True
            self._transport.writer.close()
            self._transport.reader.close()
            # Configuration cannot initialize motors; the worker exits on error/EOF.
            self.process.wait(timeout=5)
            raise
        threading.Thread(target=self._heartbeat, daemon=True, name='motor-ipc-heartbeat').start()

    def _heartbeat(self):
        while not self._closed and self.process.poll() is None:
            try:
                self._transport.send({'kind': 'heartbeat', 'sent': time.monotonic()})
            except RuntimeError:
                return
            time.sleep(.1)

    def _read(self):
        try:
            while not self._closed:
                message = self._transport.receive()
                if message['kind'] == 'feedback':
                    self._feedback.publish(message)
                    continue
                if 'status' in message and message['status']['monotonic'] >= self._status['monotonic']:
                    self._status = message['status']
                if message['kind'] == 'status':
                    continue
                with self._pending_lock:
                    pending = self._pending.get(message.get('id'))
                if pending is None:
                    continue
                if message['kind'] == 'callback':
                    # Callbacks run on the caller's thread, preserving its context.
                    pending.put(message)
                else:
                    pending.put(message)
        except (OSError, EOFError, ValueError):
            self._transport.failed.set()

    def _call(self, method, *args, timeout=15, cancel_event=None, **kwargs):
        request_id = next(self._ids)
        callbacks = {k: kwargs.pop(k) for k in ('on_waypoint', 'validate_waypoint') if kwargs.get(k) is not None}
        for key in ('on_waypoint', 'validate_waypoint'):
            kwargs.pop(key, None)
        if cancel_event is not None and cancel_event.is_set():
            raise TrajectoryInterrupted('Operation canceled before motor dispatch')
        pending = queue.Queue()
        with self._pending_lock:
            self._pending[request_id] = pending
        try:
            self._transport.send({'kind': 'call', 'id': request_id, 'method': method,
                                  'args': args, 'kwargs': kwargs, 'callbacks': list(callbacks),
                                  'sent': time.monotonic()})
            deadline = time.monotonic() + timeout
            canceled = False
            while True:
                if cancel_event is not None and cancel_event.is_set() and not canceled:
                    self._transport.send({'kind': 'cancel', 'id': request_id})
                    canceled = True
                if self.process.poll() is not None or self._transport.failed.is_set():
                    raise ServoFaultHolding('Motor process unavailable; torque state unverified')
                if time.monotonic() >= deadline:
                    self._transport.send({'kind': 'cancel', 'id': request_id})
                    raise TrajectoryInterrupted(f'Motor operation {method} timed out; cancellation requested')
                try:
                    reply = pending.get(timeout=.02)
                except queue.Empty:
                    continue
                if reply['kind'] == 'callback':
                    response = {'kind': 'callback_result', 'id': request_id, 'callback_id': reply['callback_id']}
                    try:
                        if canceled:
                            raise TrajectoryInterrupted('Callback canceled')
                        response['value'] = callbacks[reply['name']](*reply['args'])
                    except Exception as exc:
                        response['error'] = error_record(exc)
                    self._transport.send(response)
                    continue
                if 'error' in reply:
                    raise_remote(reply['error'])
                return reply.get('value')
        finally:
            with self._pending_lock:
                self._pending.pop(request_id, None)

    @property
    def state(self):
        if self.process.poll() is not None or self._transport.failed.is_set() or time.monotonic()-self._status['monotonic'] > LINK_TIMEOUT:
            return AdapterState.FAULT
        return AdapterState(self._status['state'])

    @property
    def fault(self):
        if self.state == AdapterState.FAULT and not self._status['fault']:
            return 'Motor process status unavailable or stale; torque state unverified'
        return self._status['fault']

    @property
    def torque_off_verified(self):
        return (self.process.poll() is None and not self._transport.failed.is_set()
                and time.monotonic()-self._status['monotonic'] <= LINK_TIMEOUT
                and self._status['torque_off_verified'])

    @property
    def last_command(self): return self._status['last_command']
    @property
    def recovery_status(self): return self._status['recovery_status']
    @property
    def recovery_allowed(self): return self._status['recovery_allowed']
    @recovery_allowed.setter
    def recovery_allowed(self, value): self._call('set_recovery_allowed', bool(value))

    @property
    def link_fault(self): return self._status.get('link_fault')

    def recover_operator_link(self): return self._call('recover_operator_link')

    def launch(self, **kw): return self._call('launch', timeout=120, **kw)
    def snapshot(self):
        if self.process.poll() is not None or self._transport.failed.is_set() or self._status['state'] == 'FAULT':
            raise ServoFaultHolding(self.fault or 'Motor controller fault')
        # Status and feedback are independently coalesced. After a parent pause,
        # wait for fresh status first, then a NEW pose; arrival order must not
        # turn a healthy link into a stale-status fault or admit an old pose.
        deadline = time.monotonic() + .5
        while time.monotonic()-self._status['monotonic'] > LINK_TIMEOUT:
            if self.process.poll() is not None or self._transport.failed.is_set() or time.monotonic() >= deadline:
                raise ServoFaultHolding(self.fault or 'Motor process status unavailable')
            time.sleep(.005)
        packet = self._feedback.read()
        if self.state == AdapterState.FAULT:
            raise ServoFaultHolding(self.fault or 'Motor controller fault')
        if 'error' in packet:
            raise_remote(packet['error'])
        if self.state not in {AdapterState.STOPPED, AdapterState.EXECUTING}:
            raise FeedbackUnavailable('Physical arms are not initialized')
        return packet['value']
    def hold(self): return self._call('hold')
    def stop(self): return self._call('stop', timeout=30)
    def hold_measured(self, **kw): return self._call('hold_measured', **kw)
    def execute(self, waypoints_rad, **kw):
        timeout = len(waypoints_rad)/max(float(kw.get('cadence_hz', 1)), .01) + float(kw.get('settle_timeout_s', 3)) + 30
        return self._call('execute', waypoints_rad, timeout=timeout, **kw)
    def diagnostic_event(self, event, **details):
        return self._call('diagnostic_event', event, **details)
    def timing_status(self):
        return {'isolated': True, 'pid': self.process.pid,
                'status_age_ms': max(0, time.monotonic()-self._status['monotonic'])*1000,
                'available': self.process.poll() is None and not self._transport.failed.is_set(),
                'transport_failure': self._transport.failure_reason,
                'arms': self._status['timing'], 'logging': self._status.get('logging')}
    def diagnostic_trace(self):
        # Export small chunks, never serialize a multi-MB trace in the controller.
        head = self._call('trace_head')
        for arm, data in head['arms'].items():
            trace = data.get('trace')
            if trace is None:
                continue
            after, end = 0, trace['end_sequence']
            rows = []
            for _ in range(65):
                chunk = self._call('trace_rows', arm, after, end)
                if not chunk:
                    break
                rows.extend(chunk)
                after = chunk[-1]['sequence']
            trace['rows'] = rows
        return head
    def close(self):
        self._call('close')  # Worker refuses while any drivers remain owned.
        self._closed = True
        self.process.wait(timeout=5)
        self._transport.reader.close()
        self._transport.writer.close()
