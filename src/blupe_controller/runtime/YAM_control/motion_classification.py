"""Conservative arm-check detection from measured feedback, never task text.

This identifies repeated symmetric raise/lower motion with open, inactive grippers.
Other motion (including uncertain/incomplete checks) remains visible in history.
It is not a task-success evaluator.
"""
import math

VERSION = 2


def classify_motion(rows):
    def result(kind, reason, **metrics):
        return dict(version=VERSION, kind=kind, reason=reason, **metrics)
    if len(rows) < 50:
        return result('unknown', 'insufficient_feedback')
    try:
        times = [float(r['timestamp']) for r in rows]
        joints = [list(map(float, r['measured_joints'])) for r in rows]
        grips = [list(map(float, r['measured_grippers'])) for r in rows]
        if any(len(q) != 12 for q in joints) or any(len(g) != 2 for g in grips):
            raise ValueError()
        if not all(math.isfinite(v) for v in times + [v for q in joints+grips for v in q]):
            raise ValueError()
    except (KeyError, TypeError, ValueError):
        return result('unknown', 'invalid_feedback')
    gaps = [b-a for a, b in zip(times, times[1:])]
    duration = times[-1]-times[0]
    if duration < 8 or min(gaps) <= 0 or len(rows)/duration < 6:
        return result('unknown', 'incomplete_feedback')
    if min(v for g in grips for v in g) < .85 or any(max(v)-min(v) > .08 for v in zip(*grips)):
        return result('other', 'gripper_activity_or_grasp')
    columns = list(zip(*joints))
    ranges = [max(v)-min(v) for v in columns]
    if any(ranges[i] > .08 for i in (0, 4, 5, 6, 10, 11)):
        return result('other', 'motion_outside_raise_lower_plane')
    if any(ranges[i] < minimum for i, minimum in [(1,.12),(2,.35),(3,.2),(7,.12),(8,.35),(9,.2)]):
        return result('unknown', 'insufficient_bimanual_motion')
    # The coordinated shoulder/elbow/wrist excursion of the raise/lower path.
    for offset in (0, 6):
        if not (.25 <= ranges[offset+1]/ranges[offset+2] <= .4 and
                .55 <= ranges[offset+3]/ranges[offset+2] <= .8):
            return result('other', 'different_joint_path')
    signals = []
    for i in (2, 8):
        low = min(columns[i])
        signals.append([(v-low)/ranges[i] for v in columns[i]])
    symmetry = sum(abs(a-b) for a,b in zip(*signals))/len(rows)
    if symmetry > .12:
        return result('other', 'asymmetric_motion')
    cycles = []
    for signal in signals:
        # Hysteresis rejects jitter near turning points; require full excursions.
        low_seen, high_seen, count = False, False, 0
        observed_peak, returns = False, 0
        for index, value in enumerate(signal):
            if index and gaps[index-1] > .5:
                low_seen = high_seen = observed_peak = False  # Never count across a recording gap.
            if value <= .2:
                if observed_peak:
                    returns += 1
                    observed_peak = False
                if high_seen:
                    count += 1
                    high_seen = False
                low_seen = True
            elif value >= .8:
                observed_peak = True
                if low_seen:
                    high_seen = True
                    low_seen = False
        cycles.append(count)
        if count < 1 or returns < 2 or signal[0] > .25:
            return result('unknown', 'no_repeated_complete_return')
    return result('arm_check', 'repeated_symmetric_raise_lower_open_grippers',
                  cycles=min(cycles), max_feedback_gap_s=round(max(gaps),3), symmetry_error=round(symmetry,4), duration_s=round(duration,3))
