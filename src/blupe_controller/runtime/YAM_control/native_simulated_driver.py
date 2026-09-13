"""Opt-in C++ simulated motors behind the i2rt-shaped interface. Never hardware.

ctypes.CDLL releases the Python GIL while calling native functions. Native arm
threads keep updating even when Python cannot run; existing gateway/trajectory
checks remain above this test backend. Library is explicitly supplied by caller.
"""
import ctypes as C
import threading
import time
from types import SimpleNamespace
import numpy as np
from YAM_control.simulated_driver import SimulatedBus, SimulatedRobot


class Snapshot(C.Structure):
    _fields_ = [('positions', C.c_double*7), ('feedback_age_s', C.c_double*7),
                ('max_gap_ms', C.c_double), ('cycles', C.c_uint64),
                ('codes', C.c_int*7), ('phases', C.c_int*7), ('resets', C.c_int*7),
                ('running', C.c_int), ('held', C.c_int), ('faulted', C.c_int)]


def library(path):
    lib = C.CDLL(str(path))
    lib.yam_sim_snapshot_size.argtypes = []
    lib.yam_sim_snapshot_size.restype = C.c_size_t
    if lib.yam_sim_snapshot_size() != C.sizeof(Snapshot):
        raise RuntimeError('Native snapshot ABI mismatch')
    signatures = {
        'yam_sim_create': ([], C.c_void_p),
        'yam_sim_command': ([C.c_void_p, C.POINTER(C.c_double)], C.c_int),
        'yam_sim_snapshot': ([C.c_void_p, C.POINTER(Snapshot)], C.c_int),
        'yam_sim_inject': ([C.c_void_p, C.c_int, C.c_int], C.c_int),
        'yam_sim_stop': ([C.c_void_p], C.c_int),
        'yam_sim_hold': ([C.c_void_p], C.c_int),
        'yam_sim_release': ([C.c_void_p], C.c_int),
        'yam_sim_destroy': ([C.c_void_p], None),
    }
    for name, (args, result) in signatures.items():
        fn = getattr(lib, name); fn.argtypes = args; fn.restype = result
    return lib


class NativeSimulatedBus(SimulatedBus):
    def __init__(self, path):
        super().__init__()
        self.library = library(path)

    def factory(self, *, channel, **kwargs):
        with self.lock:
            if channel in self.owners:
                raise RuntimeError('duplicate native simulated owner')
            robot = NativeRobot(self, channel)
            self.owners[channel] = robot
            self.robots.append(robot)
            return robot


class NativeThread:
    def __init__(self, chain): self.chain = chain
    def is_alive(self): return bool(self.chain.sample().running) if not self.chain.closed else False
    def join(self, timeout=None): self.chain.stop_thread(timeout or 2.)


class NativeTiming:
    def __init__(self, chain): self.chain = chain
    def report(self, feedback, now=None):
        s = self.chain.sample()
        age = max(s.feedback_age_s)*1000
        return dict(command_age_ms=None, feedback_age_ms=age,
                    max_gap_ms=s.max_gap_ms, complete=True,
                    native_simulated=True, cycles=s.cycles)


class NativeChain:
    def __init__(self, robot):
        self.robot = robot
        self.lib = robot.bus.library
        self.handle = self.lib.yam_sim_create()
        if not self.handle: raise RuntimeError('native simulated arm creation failed')
        self.closed = False
        self.enabled = set(range(1,8))
        self.motor_interface = self
        self._control_thread = NativeThread(self)
        self._last_runtime_feedback = {}
        self.command_lock = threading.RLock()
        self._api_lock = threading.RLock()
        robot._yam_command_timing = NativeTiming(self)

    def __len__(self): return 7
    @property
    def running(self): return self._control_thread.is_alive()
    @property
    def runtime_fault(self):
        if self.closed: return None
        s = self.sample()
        return 'native simulated feedback fault or command watchdog' if s.faulted or any(p != 0 for p in s.phases) else None

    def sample(self):
        with self._api_lock:
            if self.closed: raise RuntimeError('native simulated arm closed')
            s = Snapshot()
            if self.lib.yam_sim_snapshot(self.handle, C.byref(s)) != 0: raise RuntimeError('native snapshot failed')
            now = time.monotonic()
            self._last_runtime_feedback = {i+1: dict(monotonic=now-s.feedback_age_s[i], code=hex(s.codes[i])) for i in range(7)}
            return s

    def command(self, value):
        a = (C.c_double*7)(*value)
        with self._api_lock:
            if self.closed: raise RuntimeError('native simulated arm closed')
            result = self.lib.yam_sim_command(self.handle, a)
        if result == -1: raise ValueError('native target invalid')
        # A latched fault rejects motion; the adapter reports it from runtime_fault.
        return result == 0

    def inject(self, motor_id, code):
        with self._api_lock:
            if self.closed: raise RuntimeError('native simulated arm closed')
            if self.lib.yam_sim_inject(self.handle, motor_id, code): raise ValueError('invalid injection')

    def read_states(self):
        s = self.sample()
        return [SimpleNamespace(id=i+1,error_code=hex(s.codes[i]) if i+1 in self.enabled else '0x0') for i in range(7)]

    def pause_for_peer_recovery(self):
        with self._api_lock:
            if self.closed or self.lib.yam_sim_hold(self.handle): raise RuntimeError('native hold failed')

    def recovery_status(self, fresh_since=0.):
        s = self.sample()
        if s.faulted or 9 in s.phases: return 'failed'
        if any(p not in (0,8) for p in s.phases): return 'pending'
        if any(f['monotonic'] < fresh_since or f['code'] != '0x1' for f in self._last_runtime_feedback.values()): return 'pending'
        return 'recovered'

    def release_recovery_hold(self, fresh_since):
        with self._api_lock:
            if self.closed or self.recovery_status(fresh_since) != 'recovered': raise RuntimeError('native recovery unverified')
            if self.lib.yam_sim_release(self.handle): raise RuntimeError('native release rejected')

    def stop_thread(self, timeout=2.):
        if not self.robot.bus.shutdown_gate.wait(timeout): raise RuntimeError('simulated shutdown timeout')
        with self._api_lock:
            if self.closed: return
            if self.lib.yam_sim_stop(self.handle): raise RuntimeError('native stop failed')
        self.robot.bus.events.append((self.robot.channel,'can_join'))

    def motor_off(self, mid):
        if self.running or self.robot._server_thread.is_alive(): raise RuntimeError('motor_off raced producer')
        if mid in self.robot.bus.fail_off_ids: raise RuntimeError('missing motor-off acknowledgement')
        self.enabled.discard(mid)
        self.robot.bus.events.append((self.robot.channel,'off',mid))

    def close(self):
        with self._api_lock:
            if self.closed:return
            if self.enabled or self.running:raise RuntimeError('closing enabled or busy simulated bus')
            self.lib.yam_sim_destroy(self.handle)
            self.closed=True
        with self.robot.bus.lock:self.robot.bus.owners.pop(self.robot.channel,None)
        self.robot.bus.events.append((self.robot.channel,'close'))


class NativeRobot(SimulatedRobot):
    chain_class = NativeChain
    def _control(self):
        while not self._stop_event.wait(.002):
            with self.lock: target = self.target.copy()
            self.motor_chain.command(target)
        self.bus.events.append((self.channel,'robot_join'))

    def get_observations(self):
        s = self.motor_chain.sample()
        return dict(joint_pos=np.array(s.positions[:6]),gripper_pos=np.array(s.positions[6:]))
