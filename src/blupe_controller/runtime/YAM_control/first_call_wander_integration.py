"""Install the optional YAM first-call executor into an existing operator process.

Also supports deployments that wrap PhysicalOperator with a private planner.
The session transport remains the sole authenticated remote entry point.
"""
import asyncio
import os
import time

from YAM_control.first_call_wander import FirstCallWander


def install(hardware):
    if getattr(hardware.PhysicalOperator, '_first_call_wander_installed', False):
        return
    parent = hardware.PhysicalOperator

    class WanderingOperator(parent):
        _first_call_wander_installed = True

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.first_call_wander = FirstCallWander(self._physical, self._safety_guardrails,
                self._safety_config, hardware.base.MODEL_PATH, self._wander_authorized)

        def status(self, *args, **kwargs):
            result = super().status(*args, **kwargs)
            result['first_call_wander'] = self.first_call_wander.status()
            result['first_call_wander']['enabled'] = os.environ.get('YAM_FIRST_CALL_WANDER') == '1'
            return result

        def _wander_authorized(self, identity):
            with self.lock:
                return (self.mode == 'API_ACTIVE' and self.api_authorized
                    and identity == (self.api_session_id, self.api_episode_id, self.api_lease_id)
                    and self.api_last_step < 0 and self._hardware_pending is None)

        def api_handle_execution(self, payload):
            if payload.get('executor') != 'yam_first_call':
                handler = getattr(super(), 'api_handle_execution', None)
                if handler is None:
                    raise ValueError('Unsupported executor')
                return handler(payload)
            identity = tuple(payload.get(k) for k in ('session_id','episode_id','lease_id'))
            if (payload.get('schema_version') != 1 or payload.get('payload') != {}
                    or not all(isinstance(v,str) and v for v in identity)
                    or not isinstance(payload.get('expires_at'), (float,int))
                    or not time.time() < payload['expires_at'] <= time.time()+121):
                raise ValueError('Invalid or expired first-call request')
            operation = payload.get('operation')
            if operation not in {'start','finish','status'}:
                raise ValueError('Unsupported first-call operation')
            with self.lock:
                if not self._wander_authorized(identity):
                    raise ValueError('First-call lease is no longer authorized')
                if os.environ.get('YAM_FIRST_CALL_WANDER') != '1':
                    return dict(state='skipped',reason='not_enabled',settled=True)
                if operation == 'start':
                    return self.first_call_wander.start(identity)
                if identity != self.first_call_wander.identity:
                    raise ValueError('First-call start was not acknowledged')
                if operation == 'status':
                    return self.first_call_wander.status()
            return self.first_call_wander.finish(identity)

        def api_handle_joint_trajectory(self, payload):
            with self.lock:
                if self.first_call_wander.blocks_motion:
                    return [self._api_raw_trajectory_result_locked(payload, 'rejected', 'first_call_not_settled')]
                return super().api_handle_joint_trajectory(payload)

        def api_handle_joint_command(self, payload):
            with self.lock:
                if self.first_call_wander.blocks_motion:
                    raise ValueError('First-call return has not settled')
                return super().api_handle_joint_command(payload)

        def action(self, action, *args, **kwargs):
            if action != 'stop' and self.first_call_wander.blocks_motion:
                return 'First-call wandering is active; Stop is available'
            return super().action(action, *args, **kwargs)

        def _hold_physical(self, reason):
            self.first_call_wander.abort()
            return super()._hold_physical(reason)

        def _stop_physical(self, reason):
            self.first_call_wander.abort()
            return super()._stop_physical(reason)

        def api_prepare_session(self, payload):
            self.first_call_wander.abort()
            if not self.first_call_wander.done.wait(5):
                raise ValueError('Previous first-call motion has not stopped')
            return super().api_prepare_session(payload)

    hardware.PhysicalOperator = WanderingOperator
    if getattr(hardware.base, 'OperatorSimulator', None) is parent:
        hardware.base.OperatorSimulator = WanderingOperator

    from YAM_control.session_api_sim_client import SessionApiSimClient
    if getattr(SessionApiSimClient, '_first_call_wander_installed', False):
        return
    original = SessionApiSimClient._handle_message

    async def handle(self, payload):
        if payload.get('type') != 'execution_request' or payload.get('executor') != 'yam_first_call':
            return await original(self, payload)
        async def execute():
            response = {k:payload.get(k) for k in ('session_id','episode_id','lease_id','request_id')}
            response.update(schema_version=1,type='execution_result')
            try:
                result = await asyncio.to_thread(self.bridge.api_handle_execution,payload)
                response.update(status='completed',result=result)
            except Exception as exc:
                response.update(status='failed',error=str(exc)[:1000])
            self.send(response)
        # Finish may wait for settling; it must not block Stop or heartbeat traffic.
        tasks = getattr(self, '_wander_requests', None)
        if tasks is None:
            tasks = self._wander_requests = set()
        if len(tasks) >= 4:
            raise ValueError('Too many first-call requests')
        task = asyncio.create_task(execute())
        tasks.add(task); task.add_done_callback(tasks.discard)

    SessionApiSimClient._handle_message = handle
    SessionApiSimClient._first_call_wander_installed = True
