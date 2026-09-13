"""Small hardware-independent local panel. No automatic enable or queue handoff."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import secrets
import threading
from urllib.parse import urlsplit
from urllib.request import urlopen

from .driver import RobotDriver
from .poses import PoseStore

PAGE = '''<!doctype html><html><head><meta charset="utf-8"><title>BluPe operator</title>
<style>body{font:16px system-ui;max-width:900px;margin:40px auto;padding:20px;background:#f5f6f8;color:#172036}button,input{font:inherit;padding:10px;margin:4px}img{max-width:420px}pre{white-space:pre-wrap}</style></head>
<body><h1>BluPe robot operator</h1><p id="identity"></p><p>Starts read-only. Enable holds the current position. Hold stops target changes and retains torque.</p>
<button onclick="action('enable')">Enable</button><button onclick="action('hold')">Hold</button>
<button onclick="action('cloud_ready')">Accept next queued task</button><button onclick="action('cloud_pause')">Pause queue</button>
<button onclick="action('capture_zero')">Capture zero</button><button onclick="action('zero')">Move zero</button>
<button onclick="action('capture_home')">Capture home</button><button onclick="action('home')">Move home</button>
<p>Joint targets in degrees, in the displayed joint order. Gripper: 0–1.</p>
<input id="joints" size="38" placeholder="0, 0, 0, 0, 0"><input id="gripper" type="number" min="0" max="1" step="0.01" value="0.5">
<button onclick="action('move')">Move</button><p id="message"></p><pre id="state"></pre><div id="cameras"></div>
<script>
const ticket=__TICKET__;
async function action(action){try{const body={action};if(action==='move'){body.joints_deg=document.getElementById('joints').value.split(',').map(Number);body.gripper=Number(document.getElementById('gripper').value)}const r=await fetch('api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Blupe-Control':ticket},body:JSON.stringify(body)});document.getElementById('message').textContent=JSON.stringify(await r.json());}catch(e){document.getElementById('message').textContent=String(e)}}
async function update(){try{const r=await fetch('api/status');const s=await r.json();document.getElementById('state').textContent=JSON.stringify(s,null,2);document.getElementById('identity').textContent=s.robot_id;
if(!document.getElementById('cameras').children.length)for(const role of s.cameras||[]){const div=document.createElement('div');const label=document.createElement('p');label.textContent=role;const img=document.createElement('img');img.dataset.role=role;img.alt=role+' camera';div.append(label,img);document.getElementById('cameras').append(div)}
for(const img of document.querySelectorAll('img[data-role]'))img.src='api/camera/'+encodeURIComponent(img.dataset.role)+'?t='+Date.now();
}catch(e){document.getElementById('state').textContent='Controller unavailable: '+e}setTimeout(update,500)}update();
</script></body></html>'''


class Operator:
    def __init__(self, driver: RobotDriver, config):
        self.driver = driver
        self.config = config
        self.lock = threading.Lock()
        self.poses = PoseStore(config)
        self.cloud = None
        self.ticket = secrets.token_urlsafe(32)

    def state(self):
        return {**self.driver.state(), 'robot_id':self.config['robot_id'],
                'cameras':list(self.config['cameras']), 'home_captured':'home' in self.poses.poses,
                'zero_captured':'zero' in self.poses.poses, 'saved_poses':self.poses.poses,
                'cloud_execution':self.cloud.status() if self.cloud else 'not connected'}

    def action(self, payload):
        with self.lock:
            action = payload.get('action')
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
                return self.driver.move(pose['joints_deg'], pose['gripper'])
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
        from .cloud import CloudBridge
        operator.cloud = CloudBridge(driver, config, operator.poses)
        operator.cloud.start()
    print(f'SO101 operator: http://127.0.0.1:{port}/ — read-only until explicitly enabled', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if operator.cloud: operator.cloud.close()
        server.server_close()
