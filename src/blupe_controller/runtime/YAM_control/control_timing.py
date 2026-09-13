"""Small in-memory command timing; diagnostic thresholds, not motor watchdogs."""
from collections import deque
import time

WARNING_MS = 50.0
CRITICAL_MS = 100.0


class CommandTiming:
    def __init__(self):
        self.last = {}
        self.maximum = 0.0
        self.over_warning = 0
        self.over_critical = 0
        self.last_breach = None
        self.reply_maximum = 0.0
        self.reply_errors = 0
        self.bins = deque(maxlen=61)

    def begin(self, motor_id, now=None):
        now = time.monotonic() if now is None else now
        previous = self.last.get(motor_id)
        self.last[motor_id] = now
        if previous is None:
            return
        gap = max(0, (now-previous)*1000)
        self.maximum = max(self.maximum, gap)
        warning, critical = int(gap > WARNING_MS), int(gap > CRITICAL_MS)
        self.over_warning += warning
        self.over_critical += critical
        if warning:
            self.last_breach = now
        bucket = int(now)
        if not self.bins or self.bins[-1][0] != bucket:
            self.bins.append((bucket, gap, warning, critical))
        else:
            b = self.bins[-1]
            self.bins[-1] = (bucket, max(b[1], gap), b[2]+warning, b[3]+critical)

    def report(self, feedback, now=None):
        now = time.monotonic() if now is None else now
        stamps = [self.last.get(i) for i in range(1, 8)]
        replies = [feedback.get(i, {}).get('monotonic') for i in range(1, 8)]
        age = lambda values: max(0, (now-min(values))*1000) if all(v is not None for v in values) else None
        # A tuple snapshot avoids deque mutation during iteration by the CAN thread.
        recent = [b for b in tuple(self.bins) if now-b[0] < 60]
        current, feedback_age = age(stamps), age(replies)
        return {'command_age_ms': current, 'feedback_age_ms': feedback_age,
                'max_gap_ms': self.maximum, 'max_gap_60s_ms': max([b[1] for b in recent], default=0),
                'updates_over_50ms': self.over_warning, 'updates_over_100ms': self.over_critical,
                'updates_over_50ms_60s': sum(b[2] for b in recent),
                'updates_over_100ms_60s': sum(b[3] for b in recent),
                'last_breach_age_s': now-self.last_breach if self.last_breach is not None else None,
                'max_transaction_ms': self.reply_maximum, 'transaction_errors': self.reply_errors,
                'warning_ms': WARNING_MS, 'critical_ms': CRITICAL_MS,
                'complete': current is not None and feedback_age is not None}


def attach_timing(robot):
    chain = getattr(robot, 'motor_chain', None)
    interface = getattr(chain, 'motor_interface', None)
    original = getattr(interface, 'set_control', None)
    if not callable(original) or hasattr(robot, '_yam_command_timing'):
        return
    timing = CommandTiming()
    def measured(motor_id, *args, **kwargs):
        started = time.monotonic()
        timing.begin(motor_id, started)
        try:
            return original(motor_id, *args, **kwargs)
        except Exception:
            timing.reply_errors += 1
            raise
        finally:
            timing.reply_maximum = max(timing.reply_maximum, (time.monotonic()-started)*1000)
    interface.set_control = measured
    robot._yam_command_timing = timing


def arm_timing(robot):
    timing = getattr(robot, '_yam_command_timing', None)
    chain = getattr(robot, 'motor_chain', None)
    result = timing.report(getattr(chain, '_last_runtime_feedback', {})) if timing else {'complete': False}
    thread = getattr(robot, '_server_thread', None)
    result.update(control_thread_alive=bool(thread and thread.is_alive()),
                  can_worker_running=bool(getattr(chain, 'running', False)),
                  runtime_fault=getattr(chain, 'runtime_fault', None))
    return result
