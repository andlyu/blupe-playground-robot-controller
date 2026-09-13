"""Robot-scoped cloud bridge. Each queue handoff requires explicit operator action."""
from .tolerances import near_pose
import json
from pathlib import Path
import threading
import time
from urllib.request import urlopen

from .runtime.YAM_control.session_api_sim_client import SessionApiSimClient
from .runtime.YAM_control.joint_trajectory import parse_joint_trajectory


class CloudBridge:
    def __init__(self, driver, config, poses):
        self.driver, self.config, self.poses = driver, config, poses
        self.lock = threading.RLock()
        self.connected = False
        self.error = ''
        self.ready = False
        self.lease = None
        self.step = 0
        self.pending = False
        self.generation = 0
        self.client = SessionApiSimClient(self, config['api'].replace('https://','wss://',1)+'/v1/jetsons/connect',
                                         config['robot_id'], Path(config['token_file']))

    def start(self):
        self.client.start()

    def close(self):
        self.pause()
        self.client.stop()

    def pause(self):
        with self.lock:
            owned = self.ready or self.lease is not None
            self.ready = False
            self.generation += 1
            self.pending = False
            self.lease = None
            if owned:
                self.driver.hold()

    def authorize(self):
        with self.lock:
            if not self.connected or self.lease or self.ready:
                raise ValueError('Cloud must be connected and idle')
            state = self.driver.state()
            home = self.poses.get('home')
            if state['mode'] != 'active' or not self.near(state, home['joints_deg'], home['gripper']):
                raise ValueError('Enable the arm at its captured home before accepting a task')
            port = self.config['settings'].get('camera_port',8089)
            with urlopen(f'http://127.0.0.1:{port}/status',timeout=2) as response:
                if not json.load(response).get('ok'):
                    raise ValueError('Camera must be fresh before accepting a task')
            self.ready = True
            self.client.request_ready()
        return {'queue_ready':True}

    def status(self):
        with self.lock:
            return {'connected':self.connected,'queue_ready':self.ready,'session_active':self.lease is not None,
                    'command_active':self.pending,'error':self.error}

    def api_connection_changed(self, connected, error=None):
        with self.lock:
            self.connected = connected
            self.error = error or ''
            if not connected:
                self.pause()

    def api_error_received(self, error):
        with self.lock:
            self.error = error
            self.pause()

    def envelope(self, kind, **fields):
        return {'schema_version':1,'type':kind,**(self.lease or {}),**fields}

    def image_fields(self, state):
        return {'observed_at':time.time(),'left_joints_deg':state['joints_deg'],'right_joints_deg':[],
                'left_gripper':state['gripper'],'right_gripper':None,
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
            self.lease = lease
            self.ready = False
            self.step = 0
            return self.observation()

    def check(self, payload):
        if not self.lease or any(payload.get(k)!=v for k,v in self.lease.items()):
            raise ValueError('lease_mismatch')
        if self.pending or self.driver.state()['mode'] != 'active':
            raise ValueError('controller_not_ready')

    def targets(self, points):
        gripper = self.driver.state()['gripper']
        targets = []
        for point in points:
            if point.get('right_joints_deg') != [] or point.get('right_gripper') is not None:
                raise ValueError('SO101 requires an empty right arm')
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
                return [self.envelope('action_result',step_id=payload.get('step_id'),command_id=payload.get('command_id'),status='rejected',reason=str(error))]
            self.launch(payload,targets,False)
            return []

    def api_handle_joint_trajectory(self, payload):
        with self.lock:
            try:
                self.check(payload)
                trajectory = parse_joint_trajectory(payload, expected_session_id=self.lease['session_id'],
                    expected_episode_id=self.lease['episode_id'],expected_lease_id=self.lease['lease_id'],
                    expected_step_id=self.step,joint_counts=(5,0))
                targets = self.targets(payload['waypoints'])
            except ValueError as error:
                return [self.envelope('trajectory_result',trajectory_id=payload.get('trajectory_id'),status='rejected',reported_at=time.time(),code=str(error))]
            # Send accepted before the worker can send progress/completion.
            self.client.send(self.envelope('trajectory_result',trajectory_id=trajectory.trajectory_id,status='accepted',reported_at=time.time()))
            self.launch(payload,targets,True)
            return []

    def launch(self, payload, targets, trajectory):
        self.pending = True
        generation = self.generation
        threading.Thread(target=self.execute,args=(payload,targets,trajectory,generation),daemon=True).start()

    def execute(self, payload, targets, trajectory, generation):
        start = time.monotonic()
        try:
            for index,(joints,gripper) in enumerate(targets):
                while time.monotonic()<start+index*.1:
                    time.sleep(.01)
                with self.lock:
                    if generation!=self.generation:
                        return
                    if time.monotonic()>start+index*.1+.5:
                        raise ValueError('trajectory_dispatch_stalled')
                    result = self.driver.move(joints,gripper)
                    sent = result.get('sent_action',{})
                    requested = self.driver._action(joints,gripper)
                    if any(abs(sent.get(k,float('inf'))-v)>1e-6 for k,v in requested.items()):
                        raise ValueError('LeRobot clipped target; send smaller joint steps')
                    if trajectory:
                        self.client.send(self.envelope('trajectory_progress',trajectory_id=payload['trajectory_id'],step_id=self.step+index,executed_at=time.time()))
            deadline = time.monotonic()+10
            settled = None
            while True:
                with self.lock:
                    if generation!=self.generation:
                        return
                    state = self.driver.state()
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
                if generation!=self.generation:
                    return
                self.step += len(targets)
                self.pending = False
                self.client.send(self.observation())
                if trajectory:
                    self.client.send(self.envelope('trajectory_result',trajectory_id=payload['trajectory_id'],status='completed',reported_at=time.time()))
                else:
                    self.client.send(self.envelope('action_result',step_id=payload['step_id'],command_id=payload['command_id'],status='executed',reason='settled'))
        except Exception as error:
            with self.lock:
                if generation!=self.generation:
                    return
                self.error = str(error)
                if trajectory:
                    self.client.send(self.envelope('trajectory_result',trajectory_id=payload['trajectory_id'],status='aborted',reported_at=time.time(),code='controller_error',message=str(error)))
                else:
                    self.client.send(self.envelope('action_result',step_id=payload['step_id'],command_id=payload['command_id'],status='rejected',reason=str(error)))
                self.pause()

    def api_handle_stop(self, payload):
        with self.lock:
            if self.lease and all(payload.get(k)==v for k,v in self.lease.items()):
                self.pause()

    def api_heartbeat_payload(self):
        with self.lock:
            return self.envelope('heartbeat') if self.lease else None

    def api_next_observation_payload(self):
        return None

    def api_station_status_payload(self):
        with self.lock:
            state = self.driver.state()
            home = self.poses.poses.get('home')
            return {'schema_version':1,'type':'station_status','source':'hardware','mode':state['mode'],
                    'queue_ready':self.ready,'settled':not self.pending,
                    'homed':bool(home and self.near(state,home['joints_deg'],home['gripper'])),
                    **self.image_fields(state),'safety':{'ok':state['mode']!='fault','estop_engaged':False,'reason':state['error'] or None}}
