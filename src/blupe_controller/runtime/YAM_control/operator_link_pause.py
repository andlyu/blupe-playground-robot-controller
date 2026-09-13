"""Bounded pause/resume of an owned trajectory after temporary Python IPC stalls."""
import logging
import threading
import time
from YAM_control.i2rt_bimanual_adapter import TrajectoryInterrupted, ServoFaultHolding, _verify_robot_health


LOGGER=logging.getLogger(__name__)

class OperatorLinkPause:
    MAX_PAUSE_S = 10.
    MAX_RECOVERIES = 3
    def __init__(self, worker):
        self.worker=worker
        self.lock=threading.RLock()
        self.started=None
        self.reason=None
        self.count=0
        self.reference=None
        self.created=time.monotonic()

    def ready(self):
        w=self.worker;now=time.monotonic()
        return (not w.transport.failed.is_set() and now-w.last_heartbeat<=.3
                and w.heartbeat_stable_since is not None and now-w.heartbeat_stable_since>=1.)

    def request(self, reason):
        with self.lock:
            if self.started is not None:return
            w=self.worker
            if self.count>=self.MAX_RECOVERIES:
                w.latch_link_fault('automatic link recovery budget exhausted');return
            self.started=time.monotonic();self.reason=reason;self.count+=1
            command=w.adapter._last_command
            self.reference=(tuple(command['joints']),tuple(command['grippers'])) if command and command.get('monotonic',0)>=self.created else None
            w.adapter.recovery_status={'state':'paused','reason':reason,'attempt':self.count}
            LOGGER.warning('[operator-link] paused reason=%s attempt=%s heartbeat_age_ms=%.1f',reason,self.count,(time.monotonic()-w.last_heartbeat)*1000)

    def poll(self):
        with self.lock:
            if self.started is not None and not self.ready() and time.monotonic()-self.started>self.MAX_PAUSE_S:
                self.worker.latch_link_fault('operator link did not recover within 10 seconds')

    def guard(self, target, grips, validate, cancel, _retry=0):
        with self.lock:
            if self.started is None:return False
            started=self.started;reference=self.reference or (target,grips)
        w=self.worker;a=w.adapter
        while True:
            if a._stop_requested.is_set() or (cancel and cancel.is_set()) or w.link_fault:
                raise TrajectoryInterrupted('Link pause canceled; no trajectory resume')
            if w.transport.failed.is_set():
                w.latch_link_fault('operator transport failed during pause')
                raise TrajectoryInterrupted(w.link_fault)
            if self.ready():break
            if time.monotonic()-started>self.MAX_PAUSE_S:
                w.latch_link_fault('operator link did not recover within 10 seconds')
                raise TrajectoryInterrupted(w.link_fault)
            a._stop_requested.wait(.01)
        measured=self.verified_pose(reference)
        if validate is not None:validate(measured,target)
        # Stop, run expiry, or another link interruption during validation wins.
        if a._stop_requested.is_set() or (cancel and cancel.is_set()) or w.link_fault:
            raise TrajectoryInterrupted('Link recovery canceled before resume')
        if not self.ready():
            if _retry>=2:
                w.latch_link_fault('operator link remained unstable during resume validation')
                raise TrajectoryInterrupted(w.link_fault)
            return self.guard(target,grips,validate,cancel,_retry+1)
        self.verified_pose(reference)
        with self.lock:
            self.started=None;self.reason=None;self.reference=None
        a.recovery_status={'state':'resumed','reason':'operator link recovered','attempt':self.count}
        LOGGER.info('[operator-link] resumed attempt=%s pause_s=%.3f',self.count,time.monotonic()-started)
        return True

    def verified_pose(self, reference):
        a=self.worker.adapter
        positions=[]
        for arm,channel in [('left',a.left_channel),('right',a.right_channel)]:
            robot=a._robots[arm]
            positions.append(_verify_robot_health(robot,arm,channel))
            feedback=getattr(robot.motor_chain,'_last_runtime_feedback',{})
            now=time.monotonic()
            if any(feedback.get(i,{}).get('code')!='0x1' or
                   not 0<=now-feedback.get(i,{}).get('monotonic',0)<=.1 for i in range(1,8)):
                raise ServoFaultHolding(f'{arm} link resume requires fresh normal motor feedback')
        measured=tuple(positions[0][:6])+tuple(positions[1][:6])
        measured_grips=(float(positions[0][6]),float(positions[1][6]))
        if max(abs(x-y) for x,y in zip(measured,reference[0]))>.06:
            raise ServoFaultHolding('Link recovery pose changed too far to resume safely')
        if max(abs(x-y) for x,y in zip(measured_grips,reference[1]))>.3:
            raise ServoFaultHolding('Link recovery gripper changed too far to resume safely')
        return measured
