"""Native reset executor admission/cancellation using an inert motor adapter."""
from dataclasses import replace
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock
import asyncio

import numpy as np
import pytest

from YAM_control.cube_session_executor import CubeSessionExecutor
from YAM_control.hardware_safety import BimanualHardwareGuard, HardwareSafetyConfig
from YAM_control.i2rt_bimanual_adapter import AdapterState, TrajectoryInterrupted
from YAM_control.session_api_sim_client import SessionApiSimClient


def operator():
    cfg = HardwareSafetyConfig.load(Path(__file__).resolve().parents[1]/
        'src/blupe_controller/runtime/config/yam_hardware_safety.json')
    cfg = replace(cfg, joint_lower_rad=(-1.,)*12, joint_upper_rad=(1.,)*12,
        max_adjacent_delta_rad=.3, max_joint_velocity_rad_s=2.)
    op = SimpleNamespace(lock=threading.RLock(), _physical_lock=threading.RLock(),
        mode='API_ACTIVE', api_authorized=True, api_session_id='session',
        api_episode_id='episode', api_lease_id='lease', _hardware_pending=None,
        settle_tolerance=.02, data=SimpleNamespace(qpos=np.zeros(12),qvel=np.zeros(12),ctrl=np.zeros(12)),
        auto_queue=SimpleNamespace(invalidate_park=Mock()),
        _safety_guardrails=BimanualHardwareGuard(cfg,
            forward_kinematics=Mock(side_effect=AssertionError('geometry queried')),
            collision_check=Mock(side_effect=AssertionError('geometry queried'))),
        _bounded_ramp=lambda start,target: (tuple(start),tuple(target)))
    def snap(q=None):
        q = np.zeros(12) if q is None else q
        return SimpleNamespace(left=SimpleNamespace(joints_rad=tuple(q[:6]),gripper=.5),
                               right=SimpleNamespace(joints_rad=tuple(q[6:]),gripper=.5))
    op._physical=SimpleNamespace(snapshot=Mock(side_effect=lambda:snap()),
        state=AdapterState.STOPPED,fault=None)
    def execute(points,**kwargs):
        assert kwargs['disable_on_settle_timeout'] is False
        for i,q in enumerate(points):
            kwargs['validate_waypoint'](i,np.zeros(12),q)
        return snap(points[-1])
    op._physical.execute=Mock(side_effect=execute)
    op._hold_physical=Mock();op._mark_servo_fault_holding=Mock()
    return op


def request(operation, **payload):
    return dict(schema_version=1,session_id='session',episode_id='episode',lease_id='lease',
        expires_at=time.time()+120,operation=operation,payload=payload)


@pytest.fixture(autouse=True)
def selected(monkeypatch):
    monkeypatch.setenv('YAM_CUBE_RESET_SESSION_ENABLED','1')
    monkeypatch.setenv('YAM_CUBE_RESET_EXECUTION_POLICY','controller-faults')


def preview(executor, value=.05):
    return executor.request(request('preview',joints_rad=[value]*12,grippers=[.5,.5],
        execution_policy='controller-faults'))


def test_native_session_keeps_api_ownership_and_motor_guards():
    op=operator();x=CubeSessionExecutor(op);p=preview(x)
    assert p['execution_policy']=='controller-faults' and not p['geometry_checks']
    x.request(request('execute',plan_id=p['plan_id']))
    assert x.finished.wait(2)
    assert x.status()['state']=='holding' and x.status()['reached_target']
    assert op.mode=='API_ACTIVE' and op.api_lease_id=='lease'
    assert op._hardware_pending is None
    op._physical.execute.assert_called_once();op._hold_physical.assert_not_called()
    op._safety_guardrails._fk.assert_not_called()
    with pytest.raises(ValueError,match='unused'):
        x.request(request('execute',plan_id=p['plan_id']))
    assert op._physical.execute.call_count==1


@pytest.mark.parametrize('change',[{'lease_id':'wrong'},{'episode_id':'wrong'},
    {'session_id':'wrong'},{'expires_at':time.time()-1}])
def test_wrong_or_expired_authority_never_dispatches(change):
    op=operator();x=CubeSessionExecutor(op)
    with pytest.raises((ValueError,TrajectoryInterrupted)):
        x.request(dict(request('preview',joints_rad=[.05]*12,grippers=[.5,.5],
            execution_policy='controller-faults'),**change))
    op._physical.execute.assert_not_called()


@pytest.mark.parametrize('value,reason',[(2.,'outside_limits'),(.5,'adjacent_delta_limit'),(.25,'joint_velocity_limit')])
def test_selected_profile_retains_bounds_and_motion_limits(value,reason):
    op=operator();x=CubeSessionExecutor(op)
    with pytest.raises(ValueError,match=reason):preview(x,value)
    op._physical.execute.assert_not_called()


def test_disabled_executor_or_another_command_cannot_admit(monkeypatch):
    op=operator();x=CubeSessionExecutor(op)
    monkeypatch.setenv('YAM_CUBE_RESET_SESSION_ENABLED','0')
    with pytest.raises(TrajectoryInterrupted):preview(x)
    monkeypatch.setenv('YAM_CUBE_RESET_SESSION_ENABLED','1');op._hardware_pending={}
    with pytest.raises(ValueError,match='Another session command'):preview(x)
    op._physical.execute.assert_not_called()


def test_stop_before_dispatch_and_late_completion_cannot_restore_authority():
    op=operator();x=CubeSessionExecutor(op);p=preview(x)
    started,release=threading.Event(),threading.Event()
    def execute(points,**kwargs):
        started.set();assert release.wait(2)
        return op._physical.snapshot()
    op._physical.execute.side_effect=execute
    x.request(request('execute',plan_id=p['plan_id']));assert started.wait(2)
    x.abort();op.mode='DISABLED';op.api_lease_id=None;op._hardware_pending=None
    release.set();assert x.finished.wait(2)
    assert x.status()['state']=='interrupted' and op.mode=='DISABLED'
    assert op.api_lease_id is None and op._hardware_pending is None
    op._hold_physical.assert_not_called()


def test_cancelled_previous_lease_does_not_cancel_next_session():
    op=operator();x=CubeSessionExecutor(op);preview(x);x.abort()
    op.api_lease_id='next-lease'
    assert x.status()['cancelled'] is False
    p=request('preview',joints_rad=[.05]*12,grippers=[.5,.5],execution_policy='controller-faults')
    p['lease_id']='next-lease'
    assert x.request(p)['cancelled'] is False


def test_stop_during_preview_cannot_be_erased_by_a_new_cancel_event():
    op=operator();x=CubeSessionExecutor(op);snapshot=op._physical.snapshot()
    def stopped_snapshot():
        x.abort()  # Parent Stop has not acquired op.lock or cleared lease yet.
        return snapshot
    op._physical.snapshot.side_effect=stopped_snapshot
    with pytest.raises(TrajectoryInterrupted):preview(x)
    assert x.cancel.is_set() and x.plan is None
    op._physical.execute.assert_not_called()


def test_slow_preview_does_not_block_native_stop_lock():
    op=operator();x=CubeSessionExecutor(op);snapshot=op._physical.snapshot()
    started,release=threading.Event(),threading.Event();errors=[]
    def slow_snapshot():
        started.set();assert release.wait(2);return snapshot
    op._physical.snapshot.side_effect=slow_snapshot
    def plan():
        try:preview(x)
        except TrajectoryInterrupted:errors.append('cancelled')
    thread=threading.Thread(target=plan);thread.start();assert started.wait(2)
    assert op.lock.acquire(blocking=False)
    try:x.abort();op.api_authorized=False
    finally:op.lock.release()
    release.set();thread.join(2)
    assert not thread.is_alive() and errors==['cancelled']
    op._physical.execute.assert_not_called()


def test_default_validated_profile_keeps_geometry(monkeypatch):
    op=operator();x=CubeSessionExecutor(op)
    monkeypatch.setenv('YAM_CUBE_RESET_EXECUTION_POLICY','validated')
    with pytest.raises(ValueError,match='forward_kinematics_failed'):
        x.request(request('preview',joints_rad=[.05]*12,grippers=[.5,.5],execution_policy='validated'))
    op._physical.execute.assert_not_called()
    op._safety_guardrails._fk.assert_called_once()


def test_session_transport_stop_does_not_wait_for_executor_completion():
    started,release,stopped=threading.Event(),threading.Event(),threading.Event()
    class Bridge:
        def api_handle_execution(self,payload):
            started.set();assert release.wait(2);return {'state':'holding'}
        def api_handle_stop(self,payload):stopped.set()
    async def check():
        client=SessionApiSimClient(Bridge(),'wss://inert','yam-1',None)
        await client._handle_message({'type':'execution_request'})
        assert await asyncio.to_thread(started.wait,1)
        await client._handle_message({'type':'stop_session'})
        assert stopped.is_set()
        release.set();await asyncio.gather(*client.execution_tasks)
        assert client.outbound.get_nowait()['status']=='completed'
    asyncio.run(check())


def replay_operator(*, failed=False, gate=None):
    op=operator();x=CubeSessionExecutor(op)
    primary=SimpleNamespace(finished=threading.Event(),request_id=None,
        bridge=SimpleNamespace(error=None),load=Mock(return_value=(np.full((3,14),.05),
            dict(episode='physical-123',return_home=False))))
    primary.finished.set();cancel=threading.Event()
    engine=SimpleNamespace(guard=x.guard(),state=dict(running=False),thread=None)
    def begin(actions,**kwargs):
        engine.state=dict(running=True,phase='preflight',total=len(actions),completed=0)
        def run():
            try:
                if gate is not None: assert gate.wait(2)
                engine.guard.approve(np.zeros(12),(np.full(12,.05),),cadence_hz=10.)
                engine.state.update(phase='failed' if failed or cancel.is_set() else 'complete',
                    error='failure' if failed else None,completed=len(actions),finish_action='held')
            except Exception:
                engine.state.update(phase='stopped',error='Stopped')
            finally:engine.state['running']=False;primary.finished.set()
        engine.thread=threading.Thread(target=run);engine.thread.start()
    engine._begin=begin;primary.engine=engine;primary.cancel=cancel.set
    primary.status=lambda:dict(engine.state,request_id=primary.request_id,
        motor_owner='primary_controller',controller_fault=None)
    op.primary_replay=primary
    return op,x,primary,cancel


def preview_replay(x):
    return x.request(request('preview',motion_kind='recorded_replay',
        execution_policy='controller-faults',replay_request_id='a'*32,
        recipe=dict(episode='physical-123',return_home=False)))


def test_saved_replay_uses_existing_loader_engine_and_keeps_session_owner():
    op,x,p,cancel=replay_operator();plan=preview_replay(x)
    p.load.assert_called_once_with('physical-123',True,dict(episode='physical-123',return_home=False))
    original=p.engine.guard
    x.request(request('execute',plan_id=plan['plan_id']));assert x.finished.wait(2)
    assert x.status()['state']=='holding'
    assert x.status()['replay_result']['request_id']=='a'*32
    assert op.mode=='API_ACTIVE' and op.api_lease_id=='lease'
    assert p.engine.guard is original and p.finished.is_set()
    op._physical.execute.assert_not_called()  # Existing Replay engine owns dispatch.
    with pytest.raises(ValueError):x.request(request('execute',plan_id=plan['plan_id']))


def test_failed_replay_cannot_report_successful_handoff():
    op,x,p,cancel=replay_operator(failed=True);plan=preview_replay(x)
    x.request(request('execute',plan_id=plan['plan_id']));assert x.finished.wait(2)
    assert x.status()['state']=='failed' and 'replay_result' not in x.status()
    op._hold_physical.assert_called_once()


def test_stop_during_replay_cancels_engine_and_late_finish_cannot_restore_lease():
    gate=threading.Event();op,x,p,cancel=replay_operator(gate=gate);plan=preview_replay(x)
    x.request(request('execute',plan_id=plan['plan_id']))
    x.abort();assert cancel.is_set()
    op.mode='DISABLED';op.api_lease_id=None;op._hardware_pending=None
    gate.set();assert x.finished.wait(2)
    assert x.status()['state']=='interrupted' and op.mode=='DISABLED'
    assert op._hardware_pending is None and op.api_lease_id is None


def test_stop_during_replay_load_prevents_start():
    op,x,p,cancel=replay_operator();load=p.load.side_effect
    def stopped(*args):x.abort();return p.load.return_value
    p.load.side_effect=stopped
    with pytest.raises(TrajectoryInterrupted):preview_replay(x)
    assert p.engine.thread is None


def test_camera_clear_home_uses_existing_native_home_without_owner_transition():
    op=operator();op.home=np.full(12,.03);op.park_pose=np.full(12,-.2)
    x=CubeSessionExecutor(op)
    plan=x.request(request('preview',motion_kind='camera_clear_home',execution_policy='controller-faults'))
    x.request(request('execute',plan_id=plan['plan_id']));assert x.finished.wait(2)
    assert np.allclose(op._physical.execute.call_args.args[0][-1],op.home)
    assert x.status()['reached_target'] and op.mode=='API_ACTIVE'
