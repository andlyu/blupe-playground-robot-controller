"""Real message dispatch and bounded pose cleanup with simulated SO101 arms."""
import asyncio
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

from blupe_controller.runtime.YAM_control.session_api_sim_client import SessionApiSimClient
from test_auto_queue_zero import setup
from test_queue_cleanup import wait_for


ENDED_SESSION = {'code': 'invalid_session_state', 'message': 'session is not active', 'status': 409}
IDS = {'session_id': 'stopped-session', 'episode_id': 'episode', 'lease_id': 'lease'}


def dispatch(bridge, payload):
    client = SessionApiSimClient(bridge, '', 'test', Path('/unused'))
    asyncio.run(client._handle_message(payload))


def prepare(bridge, driver, ids=IDS):
    driver.mode = 'active'
    driver.q = list(bridge.poses.get('home')['joints_deg'])
    bridge.auto_queue = True
    bridge.ready = True
    dispatch(bridge, {'type': 'prepare_session', **ids, 'run_duration_s': 180})
    assert bridge.lease == ids


@pytest.mark.parametrize('error_first', [False, True])
@pytest.mark.parametrize('queued', [False, True])
def test_stop_and_ended_session_reply_complete_cleanup(setup, queued, error_first):
    b, o, d = setup
    prepare(b, d)
    d.q = [6.] * len(d.q)
    b.queued_work.return_value = queued
    error = {'error': ENDED_SESSION}
    if error_first:
        dispatch(b, error)
        assert b.lease == IDS and b.auto_queue
    dispatch(b, {'type': 'stop_session', **IDS, 'reason': 'user_requested'})
    dispatch(b, error)
    wait_for(lambda: b.ready if queued else b.parked)
    assert b.auto_queue and b.lease is None and not b.returning_home
    assert b.last_stop_reason == 'user_requested' and b.error == ''
    assert json.loads(b.status()['api_error']) == ENDED_SESSION
    assert o.motion_error == ''
    if queued:
        assert d.mode == 'active' and b.near(d.state(), o.poses.get('home')['joints_deg'], d.g)
        b.client.request_ready.assert_called_once()
        d.disable.assert_not_called()
    else:
        assert d.mode == 'readonly' and b.near(d.state(), o.poses.get('zero')['joints_deg'], d.g)
        d.disable.assert_called_once()
        b.client.request_ready.assert_not_called()
        # A repeated late rejection must not disarm the parked queue watcher.
        dispatch(b, error)
        b.queued_work.return_value = True
        wait_for(lambda: b.ready, timeout=6)
        d.enable.assert_called_once()
        assert b.near(d.state(), o.poses.get('home')['joints_deg'], d.g)
    # Nor may an old reply or old Stop cancel the next visitor's live lease.
    next_ids = {key: value + '-next' for key, value in IDS.items()}
    prepare(b, d, next_ids)
    generation = b.generation
    dispatch(b, error)
    dispatch(b, {'type': 'stop_session', **IDS, 'reason': 'user_requested'})
    assert b.lease == next_ids and b.auto_queue and b.generation == generation


@pytest.mark.parametrize('error', [
    {'code': 'invalid_session_state', 'message': 'trajectory progress does not match the active trajectory', 'status': 409},
    {'code': 'not_authorized', 'message': 'lease has expired', 'status': 403},
    {'code': 'invalid_session_state', 'message': 'session is not active', 'status': 403},
    {'code': 'not_found', 'message': 'unknown session', 'status': 404},
    'malformed error',
    None,
])
def test_other_api_errors_still_pause(setup, error):
    b, o, d = setup
    prepare(b, d)
    dispatch(b, {'error': error})
    assert not b.auto_queue and b.lease is None and not b.returning_home
    assert d.mode == 'hold' and b.last_stop_reason == 'api_error'
    assert d.sent == []


@pytest.mark.parametrize('interrupt', ['cloud_pause', 'hardware_fault'])
def test_late_reply_cannot_undo_pause_or_fault(setup, interrupt):
    b, o, d = setup
    prepare(b, d)
    if interrupt == 'cloud_pause':
        o.action({'action': 'cloud_pause'})
    else:
        state = d.state
        d.state = lambda: {**state(), 'mode': 'fault', 'error': 'motor fault'}
        d.hold = Mock()
        dispatch(b, {'type': 'stop_session', **IDS, 'reason': 'user_requested'})
    generation = b.generation
    dispatch(b, {'error': ENDED_SESSION})
    assert b.generation == generation and not b.auto_queue and not b.returning_home
    assert b.lease is None and d.sent == []


def test_invalid_json_api_error_still_pauses(setup):
    b, o, d = setup
    prepare(b, d)
    b.api_error_received('not json')
    assert b.last_stop_reason == 'api_error' and not b.auto_queue and b.lease is None
