"""Bounded driver-boundary trace. No disk I/O, extra CAN reads, or command changes.

Schema/source: docs/refs/i2rt/diagnostic-driver-source.txt. Motor command positions
are in motor space; requested targets are in joint space. Effort is driver-native
motor feedback, not an external force measurement. Feed-forward torque is NOT
net torque: onboard position/velocity gains also contribute.
"""
from collections import deque
import math
import threading
import time


def clean(value):
    if value is None:
        return None
    if isinstance(value, (str, bool)):
        return value
    try:
        return [clean(v) for v in value]
    except TypeError:
        try:
            number = float(value)
            return number if math.isfinite(number) else None
        except (TypeError, ValueError):
            return None



class MotorTrace:
    def __init__(self):
        self.rows = deque(maxlen=4096)
        self.lock = threading.Lock()
        self.sequence = 0
        self.dropped = 0

    def add(self, row):
        # The control loop never waits for the reader.
        if not self.lock.acquire(blocking=False):
            self.dropped += 1
            return
        try:
            self.sequence += 1
            row['sequence'] = self.sequence
            self.rows.append(row)
        finally:
            self.lock.release()

    def snapshot(self):
        cutoff = time.monotonic()-10
        with self.lock:
            rows = [r for r in self.rows if r['monotonic'] >= cutoff]
        return {'rows':rows, 'dropped':self.dropped, 'capacity':4096,
                'retained_s':rows[-1]['monotonic']-rows[0]['monotonic'] if len(rows)>1 else 0}

    def page(self, after, end):
        with self.lock:
            rows = []
            for row in self.rows:
                if after < row['sequence'] <= end:
                    rows.append(row)
                    if len(rows) == 64:
                        break
            return rows

    def head(self):
        return {'dropped': self.dropped, 'capacity': 4096, 'end_sequence': self.sequence}


def attach(robot):
    from YAM_control.control_timing import attach_timing
    attach_timing(robot)
    chain = getattr(robot, 'motor_chain', None)
    if chain is None or not callable(getattr(chain, 'set_commands', None)):
        return None
    if hasattr(robot, '_yam_motor_trace'):
        return robot._yam_motor_trace
    trace = MotorTrace()
    original = chain.set_commands
    def traced(*args, **kwargs):
        started = time.monotonic()
        # Capture input values before the downstream driver can mutate arrays.
        try:
            command = {k:clean(kwargs.get(k)) for k in ('pos','vel','kp','kd')}
            command['feedforward_torque'] = clean(args[0] if args else kwargs.get('torques'))
            command['latest_requested_joint_command'] = getattr(robot, '_yam_requested_command', None)
        except Exception:
            command = None
        try:
            result = original(*args, **kwargs)
        except Exception as exc:
            try:trace.add({'monotonic':started,'elapsed_s':time.monotonic()-started,
                           'command':command,'error_type':type(exc).__name__})
            except Exception:pass
            raise
        try:
            feedback = [{k:clean(getattr(m,k,None)) for k in
                         ('id','pos','vel','eff','error_code','temp_mos','temp_rotor','timestamp')} for m in result]
            trace.add({'monotonic':started,'elapsed_s':time.monotonic()-started,
                       'command':command,'feedback':feedback})
        except Exception:
            trace.dropped += 1
        return result
    request_original = robot.command_joint_pos
    def requested(position):
        try:
            robot._yam_requested_command = {'monotonic':time.monotonic(), 'position':clean(position)}
        except Exception:
            pass
        return request_original(position)
    robot.command_joint_pos = requested
    chain.set_commands = traced
    robot._yam_motor_trace = trace
    return trace


def snapshot(robot, *, include_trace=True):
    from YAM_control.control_timing import arm_timing
    trace = getattr(robot, '_yam_motor_trace', None)
    chain = getattr(robot, 'motor_chain', None)
    return {'trace':(trace.snapshot() if include_trace else trace.head()) if trace else None,
            'control_timing': arm_timing(robot),
            'runtime_fault':getattr(chain, 'runtime_fault', None),
            'servo_recovery_events':list(getattr(chain, 'recovery_events', ())),
            'max_can_command_gap_ms':getattr(chain, '_max_runtime_gap_ms', None),
            'last_can_feedback':dict(getattr(chain, '_last_runtime_feedback', {})),
            'feedforward_clip':clean(getattr(robot,'_clip_motor_torque',None)),
            'feedforward_clip_unbounded':getattr(robot,'_clip_motor_torque',None) == math.inf,
            'joint_limits':clean(getattr(robot,'_joint_limits',None)),
            'control_thread_alive':getattr(robot,'_server_thread',None).is_alive() if getattr(robot,'_server_thread',None) else None}
