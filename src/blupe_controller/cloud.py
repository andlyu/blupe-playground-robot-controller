"""Robot-scoped cloud bridge. Operator-authorized automatic queue handoffs."""
from .tolerances import near_pose
import json
import copy
import math
import logging
from pathlib import Path
import threading
import time
from urllib.request import urlopen

from .runtime.YAM_control.session_api_sim_client import SessionApiSimClient
from .runtime.YAM_control.joint_trajectory import parse_joint_trajectory


class CloudBridge:
    def __init__(self, driver, config, poses):
        self.driver, self.config, self.poses = driver, config, poses
        self.near = lambda state, joints, gripper: near_pose(state, joints, gripper, getattr(driver, 'joint_tolerance_deg', 5.0))
        self.lock = threading.RLock()
        self.connected = False
        self.error = ''
        self.api_error = ''
        self.ready = False
        self.auto_queue = False
        self.return_home = None
        self.return_zero = None
        self.parked = False
        self.returning_home = False
        self.cleanup_phase = None
        self.lease = None
        self.step = 0
        self.recorder = None
        self.recording_error = ""
        self.recorded_command = None
        self.recorded_snapshot = (None, None, 0.)
        self.pending = False
        self.active_command = None
        self.run_deadline = None
        self.run_duration_s = None
        self.deadline_cancel = None
        self.timed_out_lease = None
        self.last_stop_reason = ''
        self.generation = 0
        self.client = SessionApiSimClient(self, config['api'].replace('https://','wss://',1)+'/v1/jetsons/connect',
                                         config['robot_id'], Path(config['token_file']))

    def start(self):
        self.client.start()

    def close(self):
        self.pause()
        self.client.stop()

    def cache_recording_state(self, state):
        self.recorded_snapshot = (copy.deepcopy(state), self.recorded_command, time.monotonic())

    def finish_recording(self, outcome):
        if self.recorder is not None:
            self.recorder.finish(outcome)
            self.recorder = None
        self.recorded_command = None

    def pause(self, reason="operator_paused"):
        with self.lock:
            if self.ready or self.lease is not None or self.auto_queue or not self.last_stop_reason:
                self.last_stop_reason = reason
            self.cancel_deadline()
            self.finish_recording(reason)
            self.active_command = None
            owned = self.ready or self.lease is not None or self.auto_queue or self.returning_home
            self.auto_queue = False
            self.returning_home = False
            self.cleanup_phase = None
            self.parked = False
            self.ready = False
            self.generation += 1
            self.pending = False
            self.lease = None
            if owned:
                self.driver.hold()

    def cancel_deadline(self):
        if self.deadline_cancel is not None:
            self.deadline_cancel.set()
        self.deadline_cancel = None
        self.run_deadline = None

    def watch_deadline(self, generation, deadline, cancel):
        # A local watchdog: no model reply, heartbeat, or network IO is needed.
        while not cancel.wait(max(0., deadline-time.monotonic())):
            with self.lock:
                if generation != self.generation or self.lease is None:
                    return
                if self.expire_run():
                    return

    def expire_run(self):
        # Caller holds the bridge lock; command dispatch uses this same gate.
        if self.lease is None or self.run_deadline is None or time.monotonic() < self.run_deadline:
            return False
        reason = 'policy_runtime_timeout'
        lease = dict(self.lease)
        duration = self.run_duration_s
        active = self.active_command
        self.timed_out_lease = lease
        self.error = reason
        try:
            # Revoke the expired session before the bounded Home cleanup.
            self._stop_run({**lease, 'reason': reason})
        except Exception as error:
            self.error = f'{reason}: hold failed: {error}'
            logging.getLogger(__name__).exception('Runtime timeout Hold failed for %s', lease['session_id'])
        logging.getLogger(__name__).warning('SO101 runtime timeout robot=%s session=%s run_duration_s=%s',
                                            self.config['robot_id'], lease['session_id'], duration)
        if active is not None:
            payload, trajectory = active
            result = (dict(type='trajectory_result', trajectory_id=payload['trajectory_id'],
                           status='aborted', reported_at=time.time(), code=reason) if trajectory else
                      dict(type='action_result', step_id=payload['step_id'], command_id=payload['command_id'],
                           status='rejected', reason=reason))
            self.client.send({'schema_version':1, **lease, **result})
        self.client.send({'schema_version':1, 'type':'safety_abort', **lease, 'code':reason,
                          'message':'Controller local run duration expired; ' + ('returning Home' if self.returning_home else 'Hold requested'),
                          'observed_at':time.time(), 'details':{'run_duration_s':duration}})
        return True

    def rejection(self, payload, kind, **fields):
        # A late command must never be attributed to the subsequent live lease.
        return {'schema_version':1, 'type':kind,
                **{k:payload.get(k) for k in ('session_id','episode_id','lease_id')}, **fields}

    def authorize(self):
        with self.lock:
            if not self.connected or self.lease or self.ready or self.returning_home:
                raise ValueError('Cloud must be connected and idle')
            state = self.driver.state()
            home = self.poses.get('home')
            if state['mode'] != 'active' or not self.near(state, home['joints_deg'], home['gripper']):
                raise ValueError('Enable the arm at its captured home before accepting a task')
            port = self.config['settings'].get('camera_port',8089)
            with urlopen(f'http://127.0.0.1:{port}/status',timeout=2) as response:
                if not json.load(response).get('ok'):
                    raise ValueError('Camera must be fresh before accepting a task')
            self.error = ''
            self.ready = True
            self.client.request_ready()
        return {'queue_ready':True}

    def set_auto_queue(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('auto_queue must be boolean')
        with self.lock:
            if not enabled:
                self.pause()
            else:
                if self.config.get('hardware') not in ('so101', 'bimanual_so101'):
                    raise ValueError('Auto-queue is supported for SO101 controllers')
                if self.auto_queue: return self.status()
                state = self.driver.state()
                zero = self.poses.get('zero') if self.return_zero is not None else None
                at_zero = bool(zero and self.near(state, zero['joints_deg'], zero['gripper']))
                if at_zero and state['mode'] in ('active', 'readonly'):
                    if not self.connected or self.lease or self.ready or self.returning_home or self.pending:
                        raise ValueError('Cloud must be connected and idle')
                    if state.get('error'):
                        raise ValueError('Resolve the controller fault before enabling auto-queue')
                    if self.return_home is None:
                        raise ValueError('Home movement must be configured before enabling auto-queue')
                    home = self.poses.get('home')
                    self.driver._validate_target(home['joints_deg'], home['gripper'])
                    # Zero arms may stay torque-off until work arrives. Admission
                    # remains disarmed until measured Home and camera checks pass.
                    self.error = ''
                    self.auto_queue = True
                    self.parked = state['mode'] == 'readonly'
                    self.returning_home = not self.parked
                    self.generation += 1
                    threading.Thread(target=self._next_auto_task, args=(self.generation,), daemon=True).start()
                    return self.status()
                if self.return_zero is not None:
                    self.poses.get('zero')
                self.authorize()
                self.auto_queue = True
            return self.status()

    def queued_work(self):
        with urlopen(f'{self.config["api"]}/v1/robots/{self.config["robot_id"]}/queue', timeout=3) as response:
            snapshot = json.load(response)
        if not isinstance(snapshot.get('entries'), list):
            raise ValueError('Queue snapshot has no entries list')
        return bool(snapshot['entries'])

    def _next_auto_task(self, generation):
        try:
            # Queue HTTP and operator motion never run while holding the cloud lock.
            # Unknown queue state parks safely and retries; it must not start a run.
            waiting = False
            if self.return_zero is not None and self.auto_queue:
                try:
                    waiting = self.queued_work()
                except Exception as error:
                    with self.lock:
                        if generation != self.generation: return
                        self.error = f'Queue lookup failed: {error}'
            if self.return_zero is not None and not waiting:
                with self.lock:
                    if generation != self.generation: return
                    already_parked = self.parked
                    self.cleanup_phase = None if already_parked else 'PARKING_ZERO'
                if not already_parked:
                    self.return_zero(generation)
                with self.lock:
                    if generation != self.generation: return
                    self.returning_home = False
                    self.cleanup_phase = None
                    self.parked = True
                while True:
                    time.sleep(2)
                    with self.lock:
                        if generation != self.generation or not self.auto_queue: return
                    try:
                        waiting = self.queued_work()
                    except Exception as error:
                        with self.lock:
                            if generation == self.generation:
                                self.error = f'Queue lookup failed: {error}'
                        continue
                    if waiting: break
                with self.lock:
                    if generation != self.generation or not self.auto_queue: return
                    self.returning_home = True
            with self.lock:
                if generation != self.generation: return
                self.returning_home = True
                self.cleanup_phase = 'MOVING_HOME'
            self.return_home(generation)
            with self.lock:
                if generation == self.generation:
                    self.returning_home = False
                    self.cleanup_phase = None
                    self.parked = False
                    if self.auto_queue:
                        self.authorize()
        except Exception as error:
            with self.lock:
                if generation == self.generation:
                    self.error = str(error)
                    self.pause()

    def status(self):
        with self.lock:
            return {'connected':self.connected,'queue_ready':self.ready,'session_active':self.lease is not None,
                    'command_active':self.pending,'error':self.error,'auto_queue':self.auto_queue,
                    'api_error':self.api_error,
                    'run_duration_s':self.run_duration_s,
                    'run_remaining_s':max(0.,self.run_deadline-time.monotonic()) if self.run_deadline is not None else None,
                    'last_stop_reason':self.last_stop_reason,
                    'returning_home':self.returning_home, 'parked':self.parked,
                    'recording':dict(self.recorder.meta) if self.recorder else None, 'recording_error':self.recording_error}

    def api_connection_changed(self, connected, error=None):
        with self.lock:
            self.connected = connected
            self.error = error or ''
            if not connected:
                self.pause("connection_lost")

    def api_error_received(self, error):
        with self.lock:
            self.api_error = error
            try:
                rejection = json.loads(error)
            except (TypeError, ValueError):
                rejection = None
            if (isinstance(rejection, dict)
                    and rejection.get('code') == 'invalid_session_state'
                    and rejection.get('message') == 'session is not active'
                    and rejection.get('status') == 409):
                # In-flight reports can be rejected before OR after stop_session.
                # This response has no lease identity; only the matching Stop
                # message may revoke a lease, never a late, uncorrelated reply.
                # Preserve local Home/Zero cleanup and a subsequent run's lease.
                return
            self.error = error
            self.pause("api_error")

    def envelope(self, kind, **fields):
        return {'schema_version':1,'type':kind,**(self.lease or {}),**fields}

    def image_fields(self, state):
        self.cache_recording_state(state)
        return {'observed_at':time.time(),'left_joints_deg':state['joints_deg'][:5] if self.config.get('hardware')=='bimanual_so101' else state['joints_deg'],
                'right_joints_deg':state['joints_deg'][5:] if self.config.get('hardware')=='bimanual_so101' else [],
                'left_gripper':state['gripper'][0] if isinstance(state['gripper'],list) else state['gripper'],
                'right_gripper':state['gripper'][1] if isinstance(state['gripper'],list) else None,
                'images':{role:{'url':f'{self.config["api"]}/v1/robots/{self.config["robot_id"]}/cameras/{role}.jpg'}
                          for role in self.config['cameras']}}

    near = staticmethod(near_pose)

    def observation(self):
        state = self.driver.state()
        home = self.poses.poses.get('home')
        return self.envelope('observation',step_id=self.step,settled=not self.pending,
            homed=bool(home and self.near(state,home['joints_deg'],home['gripper'])),**self.image_fields(state))

    def api_prepare_session(self, payload):
        with self.lock:
            if not self.ready or self.lease or self.driver.state()['mode'] != 'active':
                return None
            lease = {key:payload.get(key) for key in ('session_id','episode_id','lease_id')}
            if not all(isinstance(v,str) and v for v in lease.values()):
                return None
            if self.timed_out_lease and lease['session_id'] == self.timed_out_lease['session_id']:
                return None
            duration = payload.get('run_duration_s')
            logging.getLogger(__name__).warning('SO101 handoff robot=%s session=%s run_duration_s=%r',
                                                self.config['robot_id'], lease['session_id'], duration)
            if type(duration) not in (int, float) or not math.isfinite(duration) or duration <= 0:
                self.error = 'invalid_run_duration'
                self.pause('invalid_run_duration')
                self.client.send({'schema_version':1, 'type':'safety_abort', **lease,
                                  'code':'invalid_run_duration', 'message':'A finite positive run_duration_s is required'})
                return None
            self.lease = lease
            self.run_duration_s = float(duration)
            self.run_deadline = time.monotonic()+duration
            self.deadline_cancel = threading.Event()
            self.last_stop_reason = ''
            self.error = ''
            threading.Thread(target=self.watch_deadline,
                             args=(self.generation,self.run_deadline,self.deadline_cancel),
                             daemon=True, name='so101-run-deadline').start()
            self.ready = False
            self.step = 0
            observation = self.observation()
            root = self.config['settings'].get('recording_root')
            if root:
                try:
                    from .recording import EpisodeRecorder
                    self.recorder = EpisodeRecorder(root, self.config, lease['episode_id'], payload.get('task'),
                                                    self.driver.joint_names, lambda: self.recorded_snapshot)
                    self.recorder.meta['run_duration_s'] = self.run_duration_s
                    self.recorder.start()
                    self.recording_error = ''
                except Exception as error:
                    self.recording_error = type(error).__name__
                    self.recorder = None
            return observation

    def check(self, payload):
        self.expire_run()
        if self.timed_out_lease and all(payload.get(k)==v for k,v in self.timed_out_lease.items()):
            raise ValueError('policy_runtime_timeout')
        if not self.lease or any(payload.get(k)!=v for k,v in self.lease.items()):
            raise ValueError('lease_mismatch')
        if self.pending or self.driver.state()['mode'] != 'active':
            raise ValueError('controller_not_ready')

    def targets(self, points):
        gripper = self.driver.state()['gripper']
        targets = []
        for point in points:
            if self.config.get('hardware')=='bimanual_so101':
                gripper = [point.get(side+'_gripper') if point.get(side+'_gripper') is not None else gripper[i] for i,side in enumerate(('left','right'))]
                joints = point.get('left_joints_deg',[]) + point.get('right_joints_deg',[])
                if len(point.get('left_joints_deg',[]))!=5 or len(point.get('right_joints_deg',[]))!=5: raise ValueError('Expected five joints per arm')
                self.driver._validate_target(joints,gripper)
                targets.append((joints,gripper))
                continue
            if point.get('right_joints_deg') != [] or point.get('right_gripper') is not None:
                raise ValueError('Single-arm controller requires an empty right arm')
            if point.get('left_gripper') is not None:
                gripper = point['left_gripper']
            joints = point.get('left_joints_deg')
            self.driver._validate_target(joints, gripper)
            targets.append((joints,gripper))
        return targets

    def api_handle_joint_command(self, payload):
        with self.lock:
            try:
                self.check(payload)
                if type(payload.get('step_id')) is not int or payload['step_id']!=self.step or not payload.get('command_id'):
                    raise ValueError('stale_or_invalid_command')
                targets = self.targets([payload])
            except ValueError as error:
                return [self.rejection(payload,'action_result',step_id=payload.get('step_id'),command_id=payload.get('command_id'),status='rejected',reason=str(error))]
            self.launch(payload,targets,False)
            return []

    def api_handle_joint_trajectory(self, payload):
        with self.lock:
            try:
                self.check(payload)
                trajectory = parse_joint_trajectory(payload, expected_session_id=self.lease['session_id'],
                    expected_episode_id=self.lease['episode_id'],expected_lease_id=self.lease['lease_id'],
                    expected_step_id=self.step,joint_counts=getattr(self.driver,'joint_counts',(len(self.driver.joint_names),0)))
                targets = self.targets(payload['waypoints'])
            except ValueError as error:
                return [self.rejection(payload,'trajectory_result',trajectory_id=payload.get('trajectory_id'),status='rejected',reported_at=time.time(),code=str(error))]
            # Send accepted before the worker can send progress/completion.
            self.client.send(self.envelope('trajectory_result',trajectory_id=trajectory.trajectory_id,status='accepted',reported_at=time.time()))
            self.launch(payload,targets,True)
            return []

    def launch(self, payload, targets, trajectory):
        self.pending = True
        self.active_command = (dict(payload), trajectory)
        generation = self.generation
        threading.Thread(target=self.execute,args=(payload,targets,trajectory,generation),daemon=True).start()

    def execute(self, payload, targets, trajectory, generation):
        start = time.monotonic()
        try:
            for index,(joints,gripper) in enumerate(targets):
                while time.monotonic()<start+index*.1:
                    time.sleep(.01)
                with self.lock:
                    if generation!=self.generation or self.expire_run():
                        return
                    if time.monotonic()>start+index*.1+.5:
                        raise ValueError('trajectory_dispatch_stalled')
                    result = self.driver.move(joints,gripper)
                    sent = result.get('sent_action',{})
                    requested = self.driver._action(joints,gripper)
                    if any(abs(sent.get(k,float('inf'))-v)>1e-6 for k,v in requested.items()):
                        raise ValueError('LeRobot clipped target; send smaller joint steps')
                    self.recorded_command = {'joints':[math.radians(v) for v in joints],
                                             'grippers':list(gripper) if isinstance(gripper,list) else [gripper],
                                             'monotonic':time.monotonic()}
                    self.cache_recording_state(result)
                    if trajectory:
                        self.client.send(self.envelope('trajectory_progress',trajectory_id=payload['trajectory_id'],step_id=self.step+index,executed_at=time.time()))
            deadline = time.monotonic()+10
            settled = None
            while True:
                with self.lock:
                    if generation!=self.generation or self.expire_run():
                        return
                    state = self.driver.state()
                    self.cache_recording_state(state)
                    now = time.monotonic()
                    if self.near(state,joints,gripper):
                        settled = now if settled is None else settled
                        if now-settled>=.2:
                            break
                    else:
                        settled = None
                    if now>deadline:
                        raise ValueError('joint_settle_timeout')
                time.sleep(.05)
            with self.lock:
                if generation!=self.generation or self.expire_run():
                    return
                self.step += len(targets)
                self.pending = False
                self.active_command = None
                self.client.send(self.observation())
                if trajectory:
                    self.client.send(self.envelope('trajectory_result',trajectory_id=payload['trajectory_id'],status='completed',reported_at=time.time()))
                else:
                    self.client.send(self.envelope('action_result',step_id=payload['step_id'],command_id=payload['command_id'],status='executed',reason='settled'))
        except Exception as error:
            with self.lock:
                if generation!=self.generation or self.expire_run():
                    return
                self.error = str(error)
                if trajectory:
                    self.client.send(self.envelope('trajectory_result',trajectory_id=payload['trajectory_id'],status='aborted',reported_at=time.time(),code='controller_error',message=str(error)))
                else:
                    self.client.send(self.envelope('action_result',step_id=payload['step_id'],command_id=payload['command_id'],status='rejected',reason=str(error)))
                self.pause("controller_error")

    def api_handle_stop(self, payload):
        with self.lock:
            if self.expire_run():
                return
            self._stop_run(payload)

    def _stop_run(self, payload):
        with self.lock:
            if self.lease and all(payload.get(k)==v for k,v in self.lease.items()):
                self.last_stop_reason = payload.get('reason') or 'session_stopped'
                self.active_command = None
                self.cancel_deadline()
                self.finish_recording(payload.get('reason') or 'session_stopped')
                reason = payload.get('reason')
                normal_stop = reason in ('policy_complete', 'user_requested', 'session_timeout', 'policy_runtime_timeout')
                state = self.driver.state()
                if (normal_stop and self.config.get('hardware') in ('so101', 'bimanual_so101')
                        and self.return_home and state['mode'] == 'active' and not state.get('error')):
                    # Revoke old waypoints under the same lock used by execute().
                    self.pending = False
                    self.returning_home = True
                    self.cleanup_phase = 'PREPARING'
                    self.lease = None
                    self.ready = False
                    self.generation += 1
                    generation = self.generation
                    threading.Thread(target=self._next_auto_task,args=(generation,),daemon=True).start()
                else:
                    self.pause(payload.get('reason') or 'session_stopped')

    def api_heartbeat_payload(self):
        with self.lock:
            return self.envelope('heartbeat') if self.lease else None

    def api_next_observation_payload(self):
        return None

    def api_station_status_payload(self):
        with self.lock:
            state = self.driver.state()
            home = self.poses.poses.get('home')
            homed = bool(home and self.near(state,home['joints_deg'],home['gripper']))
            # Public readiness describes automatic runs. self.ready remains the
            # separate authorization gate, including manually authorized sessions.
            queue_ready = bool(self.connected and self.auto_queue and self.ready and homed
                               and state['mode'] == 'active' and not state['error']
                               and not self.pending and not self.returning_home and not self.lease)
            mode = ('FAULT' if state['mode'] == 'fault' else self.cleanup_phase
                    or ('EXECUTING' if self.lease else 'READY' if queue_ready else 'STOPPED'))
            if self.config.get('hardware') not in ('so101', 'bimanual_so101'):
                mode, queue_ready = state['mode'], self.ready
            return {'schema_version':1,'type':'station_status','source':'hardware','mode':mode,
                    'queue_ready':queue_ready,'settled':not self.pending and not self.returning_home,
                    'homed':homed,
                    **self.image_fields(state),'safety':{'ok':state['mode']!='fault','estop_engaged':False,'reason':state['error'] or None}}
