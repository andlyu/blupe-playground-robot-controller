"""Small hardware-independent local panel. No automatic enable or queue handoff."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import secrets
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen

from .driver import RobotDriver
from .tolerances import near_pose
from .poses import PoseStore

from .operator_page import PAGE


class Operator:
    def __init__(self, driver: RobotDriver, config):
        self.driver = driver
        self.config = config
        self.lock = threading.Lock()
        self.poses = PoseStore(config)
        self.cloud = None
        self.ticket = secrets.token_urlsafe(32)
        self.motion = None
        self.motion_error = ''

    def state(self):
        state = self.driver.state()
        def at_pose(name):
            pose = self.poses.poses.get(name)
            return bool(pose and near_pose(state, pose['joints_deg'], pose['gripper']))
        return {**state, 'at_home':at_pose('home'), 'at_zero':at_pose('zero'),
                'robot_id':self.config['robot_id'],
                'manual_motion':self.motion is not None, 'manual_motion_error':self.motion_error,
                'cameras':list(self.config['cameras']), 'home_captured':'home' in self.poses.poses,
                'zero_captured':'zero' in self.poses.poses, 'saved_poses':self.poses.poses,
                'cloud_execution':self.cloud.status() if self.cloud else 'not connected'}

    def start_pose(self, pose):
        target = list(pose['joints_deg'])
        grip = pose['gripper']
        self.driver._validate_target(target, grip)
        initial = self.driver.state()
        if initial['mode'] != 'active':
            raise ValueError('Enable before moving')
        cancel = threading.Event()
        self.motion = cancel
        self.motion_error = ''

        def run():
            start = last_progress = time.monotonic()
            best = float('inf')
            settled = None
            commanded = list(initial['joints_deg'])
            commanded_grip = initial['gripper']
            try:
                while not cancel.wait(0.1):
                    with self.lock:
                        if cancel.is_set():
                            return
                        state = self.driver.state()
                        if state['mode'] != 'active':
                            raise ValueError('Movement interrupted by controller state')
                        current = state['joints_deg']
                        error = max([abs(a-b) for a,b in zip(target,current)] + [abs(grip-state['gripper'])*100])
                        now = time.monotonic()
                        if near_pose(state, target, grip):
                            settled = settled or now
                            if now-settled >= 0.3:
                                return
                        else:
                            settled = None
                        if error < best-0.1:
                            best, last_progress = error, now
                        if now-start > 60 or now-last_progress > 3:
                            raise ValueError('Saved pose movement timed out or stopped making progress')
                        # Ramp the prior command, not measured feedback. A servo
                        # tracking error must not pin every next target at the same
                        # position. Bound lookahead; LeRobot may clip it further.
                        step = lambda a,b,limit: b+max(-limit,min(limit,a-b))
                        commanded = [step(step(a,b,1.0),c,5.0)
                                     for a,b,c in zip(target,commanded,current)]
                        commanded_grip = step(step(grip,commanded_grip,0.01),state['gripper'],0.05)
                        result = self.driver.move(commanded, commanded_grip)
                        sent = (result or {}).get('sent_action')
                        if sent:
                            commanded = [sent[name+'.pos'] for name in self.driver.joint_names]
                            commanded_grip = sent['gripper.pos']/100

            except Exception as error:
                with self.lock:
                    self.motion_error = str(error)
                    try:
                        self.driver.hold()
                    except Exception:
                        pass
            finally:
                with self.lock:
                    if self.motion is cancel:
                        self.motion = None
        threading.Thread(target=run, daemon=True).start()
        return {'manual_motion':True}

    def action(self, payload):
        with self.lock:
            action = payload.get('action')
            if action == 'hold' and self.motion is not None:
                self.motion.set()
            elif self.motion is not None:
                raise ValueError('Stop the current pose movement first')
            if action == 'cloud_ready':
                if not self.cloud: raise ValueError('Cloud is not configured')
                return self.cloud.authorize()
            if action == 'cloud_pause':
                if self.cloud: self.cloud.pause()
                return {'queue_ready':False}
            if self.cloud and (self.cloud.ready or self.cloud.lease):
                if action != 'hold': raise ValueError('Pause cloud control before using manual controls')
                self.cloud.pause()
            if action == 'enable':
                return self.driver.enable()
            if action == 'hold':
                return self.driver.hold()
            if action in ('capture_home', 'capture_zero'):
                name = action.removeprefix('capture_')
                return {name:self.poses.capture(name, self.driver.state())}
            if action in ('home', 'zero'):
                pose = self.poses.get(action)
                return self.start_pose(pose)
            if action == 'move':
                if not isinstance(payload.get('joints_deg'), list):
                    raise ValueError('Provide joints_deg array')
                return self.driver.move(payload['joints_deg'], payload.get('gripper'))
            raise ValueError('Unknown operator action')


def serve(driver, config):
    operator = Operator(driver, config)
    port = config.get('settings', {}).get('operator_port', 8096)
    hosts = {'127.0.0.1', 'localhost', config.get('settings', {}).get('operator_hostname', 'localhost')}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def allowed(self):
            return urlsplit('http://' + self.headers.get('Host', '')).hostname in hosts

        def respond(self, status, data, content_type='application/json'):
            body = json.dumps(data).encode() if content_type == 'application/json' else data
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Options', 'SAMEORIGIN')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if not self.allowed():
                return self.respond(403, {'error':'Host not allowed'})
            path = urlsplit(self.path).path
            try:
                if path == '/':
                    return self.respond(200, PAGE.replace('__TICKET__', json.dumps(operator.ticket)).encode(), 'text/html; charset=utf-8')
                if path == '/api/queue':
                    with urlopen(config['api'] + '/v1/robots/' + config['robot_id'] + '/queue', timeout=5) as response:
                        data = json.loads(response.read(1000000))
                    return self.respond(200, data)
                if path == '/api/status':
                    return self.respond(200, operator.state())
                if path.startswith('/api/camera/'):
                    role = path.rsplit('/',1)[-1]
                    if role not in config['cameras']:
                        return self.respond(404, {'error':'Unknown camera'})
                    camera_port = config.get('settings', {}).get('camera_port', 8089)
                    device = config['cameras'][role]
                    with urlopen(f'http://127.0.0.1:{camera_port}/{device}/snapshot.jpg', timeout=1) as response:
                        body = response.read(4*1024*1024+1)
                        if len(body)>4*1024*1024:
                            raise ValueError('Camera frame too large')
                    return self.respond(200, body, 'image/jpeg')
                return self.respond(404, {'error':'Not found'})
            except (ValueError, OSError) as error:
                return self.respond(503, {'error':str(error)})

        def do_POST(self):
            if not self.allowed() or not hmac.compare_digest(self.headers.get('X-Blupe-Control',''), operator.ticket):
                return self.respond(403, {'error':'Control token required'})
            origin = self.headers.get('Origin')
            if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                return self.respond(403, {'error':'Origin mismatch'})
            if self.path != '/api/action':
                return self.respond(404, {'error':'Not found'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size')
                self.connection.settimeout(2)
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError('Expected object')
                self.respond(200, operator.action(payload))
            except (ValueError, OSError) as error:
                self.respond(400, {'error':str(error)})

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    if config.get('settings', {}).get('cloud_enabled'):
        operator.cloud = CloudBridge(driver, config, operator.poses)
        operator.cloud.start()
    print(f'SO101 operator: http://127.0.0.1:{port}/ — read-only until explicitly enabled', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        with operator.lock:
            if operator.motion is not None:
                operator.motion.set()
                driver.hold()
        if operator.cloud: operator.cloud.close()
        server.server_close()
