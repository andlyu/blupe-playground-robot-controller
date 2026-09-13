"""Opt-in native SocketCAN runtime; I2RT retains bring-up and verified shutdown.

The native owner duplicates I2RT's socket after initialization, replacing its
Python CAN loop. No independent reader/sender may use that socket until joined.
"""
import ctypes as C
import logging
import threading
import time
import numpy as np

class Limits(C.Structure):
    _fields_ = [('lo', C.c_double*5), ('hi', C.c_double*5)]
class Target(C.Structure):
    _fields_ = [('value', C.c_double*5)]
class Feedback(C.Structure):
    _fields_ = [(k,C.c_double) for k in ('pos','vel','torque','stamp','command_stamp')] + [(k,C.c_int) for k in ('code','mos','rotor')]
class Snapshot(C.Structure):
    _fields_ = [('feedback',Feedback*7),('hold',Target*7),('max_gap_ms',C.c_double),('max_reply_ms',C.c_double),('max_gap_60s_ms',C.c_double)] + [(k,C.c_uint64) for k in ('cycles','over50','over100','timeouts','over50_60s','over100_60s')] + [(k,C.c_int*7) for k in ('phases','resets','missing')] + [(k,C.c_int) for k in ('running','held','terminal','fault_motor','fault_code')] + [('mailbox_gaps',C.c_uint64),('mailbox_gap_ms',C.c_double),('mailbox_gap_started',C.c_double),('terminal_reason',C.c_int),('terminal_mailbox_age_ms',C.c_double)]


def native_fault_message(s, explicit_reason=None):
    if s.terminal_reason == 1:
        return (f'native CAN worker: command mailbox timeout: no fresh Python command for '
                f'{s.terminal_mailbox_age_ms / 1000:.3f}s (limit 5.000s); measured-position hold latched')
    if s.terminal_reason == 2:
        return 'native CAN worker: explicit fault hold requested: ' + (explicit_reason or 'reason not supplied')
    if s.terminal_reason == 4:
        return 'native CAN worker: socket send/receive error'
    code=(hex(s.fault_code) if s.fault_code>=0 else
          'missing reply after retry budget' if s.fault_code==-1 else 'socket send/receive error')
    return f'native CAN motor {s.fault_motor or "worker"}: {code}'


def library(path):
    lib=C.CDLL(str(path))
    signatures={
        'snapshot_size':([],C.c_size_t),
        'create':([C.c_int,C.POINTER(Limits),C.POINTER(Target),C.POINTER(Feedback)],C.c_void_p),
        'command':([C.c_void_p,C.POINTER(Target)],C.c_int),
        'snapshot':([C.c_void_p,C.POINTER(Snapshot)],C.c_int),
        'hold':([C.c_void_p,C.c_int],C.c_int),
        'release':([C.c_void_p,C.c_double],C.c_int),
        'stop':([C.c_void_p],C.c_int),'destroy':([C.c_void_p],None)}
    for name,(args,result) in signatures.items():
        fn=getattr(lib,'yam_can_'+name);fn.argtypes=args;fn.restype=result
    if lib.yam_can_snapshot_size()!=C.sizeof(Snapshot):raise RuntimeError('Native CAN ABI mismatch')
    return lib

class NativeTiming:
    def __init__(self,chain):self.chain=chain
    def report(self,feedback,now=None):
        s=self.chain._native_sample();now=time.monotonic()
        return dict(native_can=True,command_age_ms=(now-min(f.command_stamp for f in s.feedback))*1000,
                    feedback_age_ms=(now-min(f.stamp for f in s.feedback))*1000,
                    max_gap_ms=s.max_gap_ms,max_gap_60s_ms=s.max_gap_60s_ms,max_transaction_ms=s.max_reply_ms,
                    updates_over_50ms_60s=s.over50_60s,updates_over_100ms_60s=s.over100_60s,
                    updates_over_50ms=s.over50,updates_over_100ms=s.over100,
                    transaction_errors=s.timeouts,complete=all(f.command_stamp>0 for f in s.feedback))


def robot_factory(path):
    """Called once in the isolated worker, BEFORE either physical arm is created."""
    lib=library(path)
    from i2rt.motor_drivers.dm_driver import DMChainCanInterface, MotorCmd
    from i2rt.motor_drivers.utils import MotorType,FeedbackFrameInfo,MotorErrorCode
    from i2rt.robots import get_robot

    class NativeChain(DMChainCanInterface):
        def __init__(self,motor_list,motor_offset,motor_direction,channel='can0',**kwargs):
            if [m[0] for m in motor_list]!=list(range(1,8)) or channel not in ('can0','can1'):
                raise ValueError('Native CAN requires seven motors on can0/can1')
            if kwargs.get('get_same_bus_device_driver') or kwargs.get('use_buffered_reader'):
                raise ValueError('Native CAN does not support a second CAN reader')
            if kwargs.get('control_mode','MIT')!='MIT':raise ValueError('Native CAN requires MIT mode')
            mode=kwargs.get('receive_mode')
            if mode and any(mode.get_receive_id(i)!=i+16 for i in range(1,8)):
                raise ValueError('Native CAN requires p16 feedback IDs')
            if len(motor_offset)!=7 or len(motor_direction)!=7 or not np.isfinite(motor_offset).all() or not all(d in (-1,1) for d in motor_direction):
                raise ValueError('Invalid native offsets/directions')
            self._native_limits=(Limits*7)()
            for i,(_,kind) in enumerate(motor_list):
                c=MotorType.get_motor_constants(kind)
                for j,key in enumerate(('POSITION','VELOCITY','KP','KD','TORQUE')):
                    self._native_limits[i].lo[j]=getattr(c,key+'_MIN');self._native_limits[i].hi[j]=getattr(c,key+'_MAX')
            self._native_handle=None;self._native_api_lock=threading.RLock()
            self._native_stop=threading.Event();self._native_last_event=None
            self._native_mailbox_gaps=0;self._native_channel=channel
            self._native_explicit_reason=None
            super().__init__(motor_list,motor_offset,motor_direction,channel,**kwargs)

        def _native_targets(self):
            a=(Target*7)()
            for i,c in enumerate(self.commands):
                a[i].value[:]=(self._joint_position_sim_to_real_idx(c.pos,i),c.vel*self.motor_direction[i],c.kp,c.kd,c.torque*self.motor_direction[i])
            return a

        def start_thread(self):
            if self.start_thread_flag:return
            # Initialization completed. Drain its residual replies before handing
            # over the exact socket; no notifier or Python CAN thread is running.
            self.motor_interface._drain_bus(timeout_s=.01)
            sock=self.motor_interface.bus.socket
            f=(Feedback*7)()
            for i,fb in enumerate(self.state):
                f[i]=Feedback(fb.position,fb.velocity,fb.torque,time.monotonic(),0,int(fb.error_code,16),int(fb.temperature_mos),int(fb.temperature_rotor))
            self._native_handle=lib.yam_can_create(sock.fileno(),self._native_limits,self._native_targets(),f)
            if not self._native_handle:raise RuntimeError('Native CAN startup failed')
            self._control_thread=threading.Thread(target=self._native_bridge,name='native-CAN-mailbox',daemon=True)
            self.start_thread_flag=True;self._control_thread.start()

        def _native_sample(self):
            with self._native_api_lock:
                if not self._native_handle:raise RuntimeError('Native CAN is closed')
                s=Snapshot()
                if lib.yam_can_snapshot(self._native_handle,C.byref(s)):raise RuntimeError('Native CAN snapshot failed')
                return s

        def _native_publish(self,s):
            if s.mailbox_gaps != self._native_mailbox_gaps:
                logging.getLogger(__name__).warning(
                    "[native-mailbox] channel=%s gaps=%d gap_ms=%.1f started_monotonic=%.6f timeout_ms=5000",
                    self._native_channel,s.mailbox_gaps,s.mailbox_gap_ms,s.mailbox_gap_started)
                self.recovery_events.append(dict(event="native_mailbox_gap",monotonic=s.mailbox_gap_started,
                    gap_ms=s.mailbox_gap_ms,count=s.mailbox_gaps))
                self._native_mailbox_gaps=s.mailbox_gaps
            states=[FeedbackFrameInfo(id=i+1,error_code=hex(f.code),error_message=str(MotorErrorCode.get_error_message(f.code)),position=f.pos,velocity=f.vel,torque=f.torque,temperature_mos=f.mos,temperature_rotor=f.rotor) for i,f in enumerate(s.feedback)]
            with self.state_lock:
                self.state=states;self._update_absolute_positions(states)
            self._last_runtime_feedback={i+1:dict(monotonic=f.stamp,code=hex(f.code),position_rad=f.pos) for i,f in enumerate(s.feedback)}
            self._max_runtime_gap_ms=s.max_gap_ms
            if s.terminal or s.fault_motor:
                self.runtime_fault=native_fault_message(s,self._native_explicit_reason)
            event=(s.terminal_reason,s.terminal,s.fault_motor,s.fault_code,tuple(s.phases),tuple(s.resets))
            if event!=self._native_last_event:
                self.recovery_events.append(dict(event='native_runtime',monotonic=time.monotonic(),motor_id=s.fault_motor,code=s.fault_code,phases=list(s.phases),attempts=list(s.resets),terminal=bool(s.terminal),terminal_reason=s.terminal_reason,mailbox_age_ms=s.terminal_mailbox_age_ms,reason=self.runtime_fault))
                self._native_last_event=event
            if s.held:
                self._hold_commands=[MotorCmd(pos=self._joint_position_real_to_sim_idx(t.value[0],i),vel=0,kp=t.value[2],kd=t.value[3],torque=t.value[4]*self.motor_direction[i]) for i,t in enumerate(s.hold)]

        def _native_bridge(self):
            try:
                while not self._native_stop.is_set():
                    with self.command_lock:
                        with self._native_api_lock:
                            if lib.yam_can_command(self._native_handle,self._native_targets())<0:raise RuntimeError('Invalid native motor command')
                        self._native_publish(self._native_sample())
                    self._native_stop.wait(.002)
            except Exception as exc:
                self._native_explicit_reason=f'Native mailbox bridge failed: {type(exc).__name__}: {exc}'
                self.runtime_fault='native CAN worker: explicit fault hold requested: '+self._native_explicit_reason
                with self._native_api_lock:lib.yam_can_hold(self._native_handle,1)
                # CAN thread remains alive holding; explicit shutdown still joins it.

        def request_fault_hold(self,reason,feedback=None):
            with self.command_lock,self._native_api_lock:
                if self._native_explicit_reason is None:self._native_explicit_reason=str(reason)
                lib.yam_can_hold(self._native_handle,1)
                s=self._native_sample()
                self.runtime_fault=native_fault_message(s,self._native_explicit_reason) if s.terminal else str(reason)

        def pause_for_peer_recovery(self):
            with self.command_lock,self._native_api_lock:lib.yam_can_hold(self._native_handle,0)

        def recovery_status(self,fresh_since=0.):
            s=self._native_sample()
            if s.terminal or 9 in s.phases:return 'failed'
            if any(p not in (0,8) for p in s.phases) or any(s.missing):return 'pending'
            if any(f.code!=1 or f.stamp<fresh_since or time.monotonic()-f.stamp>.1 for f in s.feedback):return 'pending'
            return 'recovered'

        def release_recovery_hold(self,fresh_since):
            with self.command_lock:
                s=self._native_sample()
                if self.recovery_status(fresh_since)!='recovered':raise RuntimeError('Native recovery unverified')
                if not s.held:return
                self._native_publish(s)
                self.commands=[MotorCmd(**vars(c)) for c in self._hold_commands]
                with self._native_api_lock:
                    if lib.yam_can_release(self._native_handle,fresh_since):raise RuntimeError('Native release rejected')
                deadline=time.monotonic()+.1
                while self._native_sample().held:
                    if time.monotonic()>deadline:raise RuntimeError('Native hold release not acknowledged')
                    time.sleep(.001)
                self._hold_commands=None;self.runtime_fault=None

        def stop_thread(self,timeout=2.):
            self._native_stop.set()
            thread=getattr(self,'_control_thread',None)
            if thread:
                thread.join(timeout)
                if thread.is_alive():raise RuntimeError('Native mailbox did not join')
            with self._native_api_lock:
                if self._native_handle:lib.yam_can_stop(self._native_handle)
            self.running=False

        def close(self):
            self.stop_thread()
            with self._native_api_lock:
                if self._native_handle:lib.yam_can_destroy(self._native_handle);self._native_handle=None
            self.motor_interface.close()

    # Only the isolated worker imports this factory. Patch the constructor used
    # by get_yam_robot, leaving its calibration/gravity/limits code intact.
    get_robot.DMChainCanInterface=NativeChain
    def factory(**kwargs):
        robot=get_robot.get_yam_robot(**kwargs)
        if kwargs.get('channel')=='can0':
            from YAM_control.i2rt_bimanual_adapter import _set_left_wrist_kp
            _set_left_wrist_kp(robot,20.0)
        robot._yam_command_timing=NativeTiming(robot.motor_chain)
        return robot
    return factory
