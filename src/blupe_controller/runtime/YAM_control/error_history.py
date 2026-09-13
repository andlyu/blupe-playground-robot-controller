"""Read-only error catalogue and affected-run counts from durable episode records.

Counts are runs with evidence, not log lines, retry attempts, or motor incidents.
No robot access. Full scans run in a separate interpreter, never in the HTTP process.
"""
import json
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

# Shared with the operator portal and generated documentation.
CATALOG_PATH = Path(__file__).resolve().parents[1] / "deploy/operator/error-catalog.json"
CATALOG = {key: (row["name"], row["meaning"], row["handling"])
           for key, row in json.loads(CATALOG_PATH.read_text()).items()}


def classify(message, outcome):
    value = str(message).lower()
    if 'gripper delta' in value or 'gripper_delta' in value:
        return 'gripper_delta'
    if '0xd' in value or 'loss communication' in value or 'loss of communication' in value:
        return 'communication_loss'
    if 'did not reach' in value or 'trajectorysettletimeout' in value:
        return 'joint_timeout'
    if 'reply' in value or 'transport' in value:
        return 'feedback_timeout'
    if 'servofaultholding' in value or 'motorerror' in value:
        return 'servo_fault'
    if outcome in {'session_timeout', 'policy_runtime_timeout'}:
        return 'time_limit'
    if outcome == 'physical_execution_failure':
        return 'execution'
    return None


def read_object(path):
    # Never touch samples/images or load unbounded files in the HTTP worker.
    with path.open('rb') as stream:
        raw = stream.read(16 * 1024 * 1024 + 1)
    if len(raw) > 16 * 1024 * 1024:
        raise ValueError('Record too large')
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError('Expected object')
    return value


def summarize(root, now=None):
    now = time.time() if now is None else now
    rows = {key: {'id': key, 'name': entry[0], 'meaning': entry[1], 'handling': entry[2],
                  'counts': {'24h': 0, '7d': 0, 'all': 0}, 'recent': []} for key, entry in CATALOG.items()}
    result = {'available': bool(root and Path(root).is_dir()), 'generated_at': now,
              'recorded_runs': 0, 'unreadable_records': 0, 'earliest_run': None,
              'count_unit': 'affected runs', 'issues': []}
    if not result['available']:
        return result
    for path in sorted(Path(root).glob('ep_*/manifest.json')):
        try:
            meta = read_object(path)
            stamp = float(meta.get('ended_at') or meta['started_at'])
            if not math.isfinite(stamp):
                raise ValueError('Invalid time')
            if meta.get('simulated'):
                continue
        except (OSError, ValueError, TypeError, KeyError):
            result['unreadable_records'] += 1
            continue
        result['recorded_runs'] += 1
        result['earliest_run'] = min(result['earliest_run'] or stamp, stamp)
        trace = {}
        trace_path = path.with_name('failure-summary.json')
        if not trace_path.exists():
            trace_path = path.with_name('motor-trace.json')
        if meta.get('outcome') == 'physical_execution_failure' and trace_path.exists():
            try:
                trace = read_object(trace_path)
            except (OSError, ValueError, TypeError):
                result['unreadable_records'] += 1
        error = trace.get('error') or {}
        message = ' '.join(str(error.get(k) or '') for k in ('error_type', 'message')) if isinstance(error, dict) else str(error)
        outcome = meta.get('outcome')
        found = {}
        category = classify(message, outcome)
        if category:
            found[category] = message.strip() or str(outcome)
        if meta.get('capture_errors') or meta.get('status') == 'recording_failed':
            found['recording'] = f"{meta.get('capture_errors', 0)} capture errors; {meta.get('last_error_type') or meta.get('status')}"
        for key, evidence in found.items():
            row = rows[key]
            row['counts']['all'] += 1
            for label, seconds in [('24h', 86400), ('7d', 604800)]:
                row['counts'][label] += int(0 <= now - stamp <= seconds)
            row['recent'].append({'episode_id': path.parent.name, 'timestamp': stamp,
                                  'evidence': evidence[:2000], 'outcome': outcome or 'not recorded'})
    for row in rows.values():
        row['recent'] = sorted(row['recent'], key=lambda r: r['timestamp'], reverse=True)[:10]
    result['issues'] = sorted(rows.values(), key=lambda r: (-r['counts']['24h'], -r['counts']['all'], r['name']))
    return result


_cache = {}
_lock = threading.Lock()


def _refresh(root, entry):
    try:
        result = subprocess.run([sys.executable, '-m', 'YAM_control.error_history', root],
                                cwd=str(CATALOG_PATH.parents[2]), capture_output=True,
                                check=True, timeout=30)
        if len(result.stdout) > 256 * 1024:
            raise ValueError('History summary too large')
        value = json.loads(result.stdout)
        with _lock:
            entry.update(value=value, updated=time.monotonic(), error=None)
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        with _lock:
            entry['error'] = type(exc).__name__
    finally:
        with _lock:
            entry['pending'] = False


def cached_summary(root):
    root = str(root or '')
    with _lock:
        entry = _cache.setdefault(root, dict(value=None, updated=0, attempted=-60, pending=False, error=None))
        now = time.monotonic()
        if not entry['pending'] and now-entry['attempted'] > 30:
            entry.update(pending=True, attempted=now)
            threading.Thread(target=_refresh, args=(root, entry), daemon=True,
                             name='error-history-process').start()
        result = dict(entry['value'] or {'available': False, 'issues': [], 'recorded_runs': 0})
        result.update(refreshing=entry['pending'], refresh_error=entry['error'],
                      stale=entry['value'] is None or now-entry['updated'] > 60)
        return result


if __name__ == '__main__':
    # Low-priority diagnostics cannot compete at the motor process's priority.
    if hasattr(os, 'nice'):
        try:
            os.nice(10)
        except OSError:
            pass  # Some development sandboxes disallow even lower priority.
    print(json.dumps(summarize(sys.argv[1] or None)), flush=True)
