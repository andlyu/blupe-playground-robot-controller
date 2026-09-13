"""Local browser operator console backed by the YAM MuJoCo twin.

This process has no i2rt, CAN, RoboCurve, Jetson, or AWS credentials. The policy
mode generates a small deterministic command stream to exercise operator handoff.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.error import URLError
from urllib.request import urlopen

import cv2
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from YAM_control.workspace_visuals import SAFETY_CONFIG, HardwareSafetyConfig, add_workspace_planes
from YAM_control.session_api_sim_client import SessionApiSimClient
from YAM_control.joint_trajectory import (
    JointTrajectory,
    TrajectorySchemaError,
    parse_joint_trajectory,
)

MODEL_PATH = ROOT / "assets/yam_bimanual/scene.xml"
POSE_CONFIG_PATH = ROOT / "config/yam_operator_sim_pose.json"
N_JOINTS = 12
END_EFFECTORS = ("left_grasp", "right_grasp")
MAX_VELOCITY = 0.35
HOME_TOLERANCE = 0.015
JOINT_SENSOR_TOLERANCE = 0.002
MAX_COMMAND_DELTA = 0.2
API_SETTLE_TIME = 0.2
# Verified against wrist motion and serials: left ...071768=/10,
# right ...071268=/4. Keep published observations and UI labels identical.
import os
JETSON_CAMERA_ROLES = json.loads(os.environ.get("BLUPE_CAMERA_ROLES", '{"left":"10","top":"16","right":"4"}'))
JETSON_CAMERA_IDS = frozenset(JETSON_CAMERA_ROLES.values())


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Bimanual YAM Operator Console / SIM</title>
<style>
:root{--ink:#18201d;--paper:#eee9dc;--amber:#efaa22;--red:#d83b2d;--green:#2b8065;--line:#a9a294}
*{box-sizing:border-box}body{margin:0;color:var(--ink);background:var(--paper);font-family:Georgia,'Times New Roman',serif;background-image:linear-gradient(#0000000b 1px,transparent 1px),linear-gradient(90deg,#0000000b 1px,transparent 1px);background-size:24px 24px}
header{display:flex;justify-content:space-between;align-items:end;padding:22px 26px 14px;border-bottom:3px solid var(--ink);background:#f5f0e4e8}h1{font-size:clamp(24px,4vw,48px);margin:0;letter-spacing:-.04em}.tag{font:700 12px ui-monospace,monospace;letter-spacing:.12em;background:var(--amber);padding:7px 10px;border:2px solid var(--ink)}
main{display:grid;grid-template-columns:minmax(300px,1.4fr) minmax(300px,1fr);gap:18px;padding:18px;max-width:1300px;margin:auto}.panel{background:#f7f2e7;border:2px solid var(--ink);box-shadow:5px 5px 0 var(--ink);padding:18px}.panel h2{font:800 12px ui-monospace,monospace;letter-spacing:.15em;margin:0 0 14px;text-transform:uppercase}
.mode{font-size:clamp(38px,6vw,76px);line-height:.9;margin:4px 0 18px;letter-spacing:-.06em}.mode.policy{color:var(--green)}.mode.fault{color:var(--red)}
#arm{display:block;width:100%;height:360px;object-fit:cover;background:#d9e1d8;border:2px solid var(--ink)}.controls{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:16px}button{min-height:66px;border:2px solid var(--ink);background:#fffaf0;color:var(--ink);font:800 15px ui-monospace,monospace;text-transform:uppercase;cursor:pointer;box-shadow:3px 3px 0 var(--ink)}button:active{transform:translate(3px,3px);box-shadow:none}button:disabled{opacity:.35;cursor:not-allowed}.stop{grid-column:1/-1;background:var(--red);color:white;font-size:24px;min-height:90px}.run{background:var(--green);color:white}
.checks{display:grid;gap:8px}.check{display:flex;justify-content:space-between;border-bottom:1px solid var(--line);padding:8px 0;font:700 13px ui-monospace,monospace}.ok{color:var(--green)}.bad{color:var(--red)}.joints{display:grid;gap:8px;margin-top:16px}.joint{display:grid;grid-template-columns:34px 1fr 72px;gap:8px;align-items:center;font:12px ui-monospace,monospace}.track{height:10px;border:1px solid var(--ink);background:#ddd}.fill{height:100%;background:var(--amber);width:50%}.log{height:104px;overflow:auto;background:var(--ink);color:#dbe8df;padding:10px;font:11px/1.5 ui-monospace,monospace;margin-top:16px}.foot{font:11px ui-monospace,monospace;margin-top:12px;color:#555}
@media(max-width:780px){main{grid-template-columns:1fr;padding:10px}header{padding:16px}#arm{height:280px}}
</style>
<style>
.camera-bank{margin-top:18px;padding-top:16px;border-top:1px solid var(--line)}
.camera-bank h3{margin:0 0 10px;font:600 12px/1.2 ui-monospace,monospace;letter-spacing:.12em;color:var(--muted);text-transform:uppercase}
.camera-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}
.camera-feed{margin:0;border:1px solid var(--line);background:#090b0b;overflow:hidden}
.camera-feed img{display:block;width:100%;aspect-ratio:16/9;object-fit:cover}
.camera-feed figcaption{padding:7px 9px;font:600 11px/1.2 ui-monospace,monospace;color:var(--text);letter-spacing:.05em}
.camera-live{color:var(--ok)}
.queue-bank{margin-top:18px;padding-top:16px;border-top:1px solid var(--line)}
.queue-head{display:flex;align-items:center;justify-content:space-between;gap:10px}.queue-head h3{margin:0;font:800 12px ui-monospace,monospace;letter-spacing:.12em;text-transform:uppercase}.queue-count{font:800 12px ui-monospace,monospace;background:var(--amber);border:1px solid var(--ink);padding:4px 7px}
.queue-list{margin:10px 0 0;padding:0;list-style:none;max-height:220px;overflow:auto;border:1px solid var(--line);background:#eee8da}.queue-item{display:grid;grid-template-columns:34px 1fr auto;gap:8px;padding:8px;border-bottom:1px solid var(--line);font:11px ui-monospace,monospace}.queue-item:last-child{border-bottom:0}.queue-id{overflow:hidden;text-overflow:ellipsis}.queue-state{text-transform:uppercase}.queue-empty{padding:16px;font:12px ui-monospace,monospace;color:#666}.queue-actions{display:grid;grid-template-columns:1fr;gap:8px;margin-top:10px}.queue-actions button{min-height:48px;background:#fff3cf}.queue-note{margin-top:8px;font:10px/1.4 ui-monospace,monospace;color:#555}
@media(max-width:720px){.camera-grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<header><div><div class="tag">LOCAL / BIMANUAL MUJOCO / NO HARDWARE</div><h1>Bimanual YAM Operator Console</h1></div><div id="clock"></div></header>
<main>
<section class="panel"><h2>Live MuJoCo Bimanual YAM</h2><div id="mode" class="mode">DISABLED</div><img id="arm" src="/api/frame.jpg" alt="Live rendered bimanual YAM simulation"><div class="controls"><button class="stop" onclick="act('stop')">Stop, Rest & Disable Both</button><button onclick="act('launch')">Launch Arms</button><button id="home" onclick="act('home')">Move Both Home</button><button id="run" class="run" onclick="act('run_policy')">Run Policy via Session API</button></div><div class="camera-bank"><h3>Physical Jetson Cameras · Observation Only</h3><div class="camera-grid"><figure class="camera-feed"><img id="cam-left" alt="Live left RGB view from Jetson camera __LEFT_CAMERA_ID__"><figcaption><span class="camera-live">● LIVE</span> · LEFT VIEW / __LEFT_CAMERA_ID__</figcaption></figure><figure class="camera-feed"><img id="cam-top" alt="Live top RGB view from Jetson camera __TOP_CAMERA_ID__"><figcaption><span class="camera-live">● LIVE</span> · TOP VIEW / __TOP_CAMERA_ID__</figcaption></figure><figure class="camera-feed"><img id="cam-right" alt="Live right RGB view from Jetson camera __RIGHT_CAMERA_ID__"><figcaption><span class="camera-live">● LIVE</span> · RIGHT VIEW / __RIGHT_CAMERA_ID__</figcaption></figure></div></div></section>
<aside class="panel"><h2>Safety gate</h2><div class="checks"><div class="check"><span>SESSION API</span><span id="api"></span></div><div class="check"><span>COMMAND SOURCE</span><span id="source"></span></div><div class="check"><span>POLICY PHASE</span><span id="phase"></span></div><div class="check"><span>POLICY RUNTIME LIMIT</span><span>3 MIN · PARK ZERO &amp; TORQUE OFF</span></div><div class="check"><span>VERTICAL TRAVEL</span><span id="travel"></span></div><div class="check"><span>POSITION LIMITS</span><span id="position"></span></div><div class="check"><span>VELOCITY CAP</span><span class="ok">0.35 RAD/S</span></div><div class="check"><span>SIM CONTACTS</span><span id="contacts"></span></div><div class="check"><span>HOMED</span><span id="homed"></span></div><div class="check"><span>RESTED</span><span id="rested"></span></div><div class="check"><span>DRIVES</span><span id="drives"></span></div><div class="check"><span>CONTROLLER</span><span id="controller"></span></div></div><div class="queue-bank"><div class="queue-head"><h3>Session API FIFO</h3><span id="queue-count" class="queue-count">--</span></div><ul id="queue-list" class="queue-list"><li class="queue-empty">Loading queue...</li></ul><div class="queue-actions"><button id="drain" onclick="drainVisibleQueue()" disabled>Drain visible test queue</button></div><div id="queue-note" class="queue-note">Read-only queue view. Cleanup uses audited operator Stop/Rest/Home/ready and rejects unexpected assignments.</div></div><div id="joints" class="joints"></div><div id="log" class="log"></div><div class="foot">STOP rejects API commands immediately, returns both arms to captured Rest under limits, then disables command acceptance. Physical E-STOP remains immediate.</div></aside>
</main>
<script>
const log=document.getElementById('log');let prior='',queueSnapshot={entries:[],current_session_id:null,cleanup:{running:false}};
const cameraBase=(location.hostname==='127.0.0.1'||location.hostname==='localhost')?'http://127.0.0.1:18089':`${location.protocol}//${location.hostname}:8089`;
document.getElementById('cam-left').src=cameraBase+'/__LEFT_CAMERA_ID__';document.getElementById('cam-top').src=cameraBase+'/__TOP_CAMERA_ID__';document.getElementById('cam-right').src=cameraBase+'/__RIGHT_CAMERA_ID__';
async function act(action){try{const r=await fetch('/api/action',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({action})});const d=await r.json();const message=d.message||d.error||`Action failed (${r.status})`;log.innerHTML=`${new Date().toLocaleTimeString()} ${esc(message)}<br>`+log.innerHTML}catch(e){log.innerHTML=`${new Date().toLocaleTimeString()} Action request failed: ${esc(e.message)}<br>`+log.innerHTML}}
function flag(id,ok,good='CLEAR',bad='BLOCKED'){const e=document.getElementById(id);e.textContent=ok?good:bad;e.className=ok?'ok':'bad'}
function joints(q,lo,hi){document.getElementById('joints').innerHTML=q.map((v,i)=>`<div class="joint"><b>J${i+1}</b><div class="track"><div class="fill" style="width:${Math.max(0,Math.min(100,(v-lo[i])/(hi[i]-lo[i])*100))}%"></div></div><span>${v.toFixed(3)} rad</span></div>`).join('')}
function esc(v){return String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function queueIds(){const active=queueSnapshot.current_session_id;return [active,...queueSnapshot.entries.map(e=>e.session_id)].filter((v,i,a)=>v&&a.indexOf(v)===i)}
function renderQueue(){const entries=queueSnapshot.entries||[],active=queueSnapshot.current_session_id,cleanup=queueSnapshot.cleanup||{};const rows=[];if(active)rows.push({session_id:active,status:'active on station'});for(const entry of entries){if(entry.session_id!==active)rows.push(entry)}document.getElementById('queue-count').textContent=rows.length;document.getElementById('queue-list').innerHTML=rows.length?rows.map((e,i)=>`<li class="queue-item"><b>${i+1}</b><span class="queue-id" title="${esc(e.session_id)}">${esc(e.session_id)}</span><span class="queue-state">${esc(e.status||e.state||'queued')}</span></li>`).join(''):`<li class="queue-empty">Queue empty</li>`;const drain=document.getElementById('drain');drain.disabled=!rows.length||cleanup.running;document.getElementById('queue-note').textContent=cleanup.running?`Cleanup ${cleanup.processed||0}/${cleanup.total||0}: ${cleanup.message||'working'}`:cleanup.error?`Cleanup halted: ${cleanup.error}`:'Read-only queue view. Cleanup uses audited operator Stop/Home/ready and rejects unexpected assignments.'}
async function queueTick(){try{queueSnapshot=await(await fetch('/api/queue')).json();renderQueue()}catch(e){document.getElementById('queue-note').textContent='Queue unavailable: '+e.message}setTimeout(queueTick,1000)}
async function drainVisibleQueue(){const ids=queueIds();if(!ids.length)return;const preview=ids.slice(0,3).join('\n')+(ids.length>3?`\n... and ${ids.length-3} more`:'');if(!confirm(`Drain ${ids.length} visible test session(s) in FIFO order?\n\n${preview}\n\nThis sends audited operator Stop, Home, then ready. No action commands.`))return;const r=await fetch('/api/queue-cleanup',{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify({session_ids:ids})});const d=await r.json();log.innerHTML=`${new Date().toLocaleTimeString()} ${esc(d.message||d.error)}<br>`+log.innerHTML}
async function tick(){try{const s=await(await fetch('/api/status')).json();const m=document.getElementById('mode');m.textContent=s.mode;m.className='mode '+(s.mode==='API_ACTIVE'?'policy':s.mode==='FAULT'?'fault':'');const apiLabel=!s.api.connected?'DISCONNECTED':s.api.session_id?'CONNECTED / ACTIVE':s.api.authorized?'CONNECTED / WAITING':'CONNECTED / IDLE';flag('api',s.api.connected,apiLabel,'DISCONNECTED');document.getElementById('api').title=s.api.error||'';document.getElementById('source').textContent=s.command_source;document.getElementById('phase').textContent=s.policy_phase.toUpperCase();document.getElementById('travel').textContent='L '+(s.vertical_travel_m[0]*100).toFixed(1)+' / R '+(s.vertical_travel_m[1]*100).toFixed(1)+' CM';flag('position',s.safety.position_limits);flag('contacts',s.safety.contact_count===0,String(s.safety.contact_count)+' CLEAR',String(s.safety.contact_count)+' ACTIVE');flag('homed',s.homed,'YES','NO');flag('rested',s.rested,'YES','NO');flag('drives',s.safety.drives_enabled,'ENABLED','DISABLED');flag('controller',s.safety.controller_ok,'HEALTHY','FAULT');document.getElementById('controller').title=s.adapter_fault||'';document.getElementById('run').disabled=s.mode!=='READY'||!s.api.connected;joints(s.joints,s.lower,s.upper);document.getElementById('arm').src='/api/frame.jpg?t='+Date.now();if(s.event!==prior){prior=s.event;log.innerHTML=`${new Date().toLocaleTimeString()} ${esc(s.event)}<br>`+log.innerHTML}}catch(e){flag('controller',false)}document.getElementById('clock').textContent=new Date().toLocaleTimeString();setTimeout(tick,100)}tick();
queueTick();
</script></body></html>"""


for _camera_role, _camera_id in JETSON_CAMERA_ROLES.items():
    PAGE = PAGE.replace(f"__{_camera_role.upper()}_CAMERA_ID__", _camera_id)


class OperatorSimulator:
    def __init__(self, pose_config_path: Path = POSE_CONFIG_PATH) -> None:
        self.workspace_visual_config = HardwareSafetyConfig.load(SAFETY_CONFIG)
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        home_id = self.model.key("home").id
        mujoco.mj_resetDataKeyframe(self.model, self.data, home_id)
        self.lower = self.model.jnt_range[:N_JOINTS, 0].copy()
        self.upper = self.model.jnt_range[:N_JOINTS, 1].copy()
        self.rest = np.clip(
            self.data.qpos[:N_JOINTS].copy(), self.lower, self.upper
        )
        pose_config = json.loads(pose_config_path.read_text())
        packed_home = np.asarray(pose_config["home_pose"], dtype=np.float64)
        if packed_home.shape != (14,) or not np.all(np.isfinite(packed_home)):
            raise ValueError("operator home_pose must contain 14 finite values")
        self.home = np.concatenate((packed_home[:6], packed_home[7:13]))
        if np.any(self.home < self.lower - JOINT_SENSOR_TOLERANCE) or np.any(
            self.home > self.upper + JOINT_SENSOR_TOLERANCE
        ):
            raise ValueError("operator home_pose is outside MuJoCo joint bounds")
        self.home = np.clip(self.home, self.lower, self.upper)
        if pose_config.get("rest_pose") is not None:
            packed_rest = np.asarray(pose_config["rest_pose"], dtype=np.float64)
            if packed_rest.shape != (14,) or not np.all(np.isfinite(packed_rest)):
                raise ValueError("operator rest_pose must contain 14 finite values")
            self.rest = np.concatenate((packed_rest[:6], packed_rest[7:13]))
        self.ramp_secs = float(pose_config.get("ramp_secs", 3.0))
        self.settle_tolerance = float(
            pose_config.get("settle_tolerance_rad", HOME_TOLERANCE)
        )
        if self.ramp_secs <= 0 or self.settle_tolerance <= 0:
            raise ValueError("operator ramp and settle values must be positive")
        self.data.qpos[:N_JOINTS] = self.rest
        self.data.qvel[:] = 0.0
        self.data.ctrl[:N_JOINTS] = self.rest
        mujoco.mj_forward(self.model, self.data)
        self.target = self.rest.copy()
        self.ramp_start = self.rest.copy()
        self.ramp_goal: np.ndarray | None = None
        self.ramp_started = 0.0
        self.mode = "DISABLED"
        self.command_source = "none"
        self.event = "simulator initialized at captured Rest; command acceptance disabled"
        self.controller_generation = 1
        self.controller_ok = True
        self.launched = False
        self.drives_enabled = False
        self.policy_started = 0.0
        self.policy_phase = "idle"
        self.policy_start_joints = self.home.copy()
        self.policy_start_ee = np.stack(
            [self.data.body(name).xpos.copy() for name in END_EFFECTORS]
        )
        self.policy_settled_since: float | None = None
        self.api_client: SessionApiSimClient | None = None
        self.api_connected = False
        self.api_authorized = False
        self.api_error: str | None = None
        self.api_session_id: str | None = None
        self.api_episode_id: str | None = None
        self.api_lease_id: str | None = None
        self.api_last_step = -1
        self.api_observation_step = -1
        self.api_pending: dict[str, Any] | None = None
        self.api_pending_settled_since: float | None = None
        self.api_trajectory: dict[str, Any] | None = None
        self.api_trajectory_records: dict[str, dict[str, Any]] = {}
        self.public_base = "http://127.0.0.1:8096"
        self.session_api_base = "https://yam-session-api.n5hthc3gj4cqy.us-east-1.cs.amazonlightsail.com"
        self.camera_reference_base = "http://127.0.0.1:8089"
        self.cleanup_state: dict[str, Any] = {
            "running": False, "processed": 0, "total": 0,
            "message": "idle", "error": None,
        }
        self.latest_jpeg: bytes | None = None
        self.render_error: str | None = None
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def attach_api_client(self, client: SessionApiSimClient) -> None:
        self.api_client = client

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=2.0)

    def action(self, action: str) -> str:
        with self.lock:
            if action == "stop":
                if not self.launched and self._rested():
                    return "already at captured Rest and disabled; no change"
                if self.api_client is not None and self.api_lease_id is not None:
                    self.api_client.send(self._api_envelope_locked(
                        "safety_abort", reason="operator_stop"
                    ))
                self._clear_api_lease_locked()
                self.api_authorized = False
                self.mode = "STOPPING_REST"
                self.command_source = "operator/stop"
                self._start_ramp_locked(self.rest)
                self.policy_phase = "returning_to_rest_to_disable"
                self.drives_enabled = True
                self.event = "STOP: policy rejected; returning both arms to captured Rest before disable"
            elif action == "launch":
                if self.launched:
                    return "already launched; no change"
                self.mode = "STOPPED"
                self.command_source = "operator"
                self.target = self._bounded_current_pose()
                self.ramp_goal = None
                self.controller_generation += 1
                self.controller_ok = True
                self.launched = True
                self.drives_enabled = True
                self.policy_phase = "idle"
                self.event = f"controller launched (generation {self.controller_generation}); arms enabled and holding"
            elif action == "home":
                if not self.launched:
                    return "MOVE HOME rejected: launch arms first"
                self.mode = "HOMING"
                self.command_source = "operator/home"
                self._start_ramp_locked(self.home)
                self.drives_enabled = True
                self.policy_phase = "homing"
                self.event = "operator authorized safety-limited move home"
            elif action == "run_policy":
                if not self.launched or self.mode != "READY" or not self._homed():
                    return "RUN POLICY rejected: move home and reach READY first"
                if not self.api_connected or self.api_client is None:
                    return "RUN POLICY rejected: Session API is disconnected"
                self.mode = "API_WAITING"
                self.command_source = "YAM Session API"
                self.drives_enabled = True
                self.api_authorized = True
                self.policy_phase = "waiting_for_fifo_lease"
                self.policy_start_joints = self._bounded_current_pose()
                self.policy_start_ee = np.stack(
                    [self.data.body(name).xpos.copy() for name in END_EFFECTORS]
                )
                self.api_client.request_ready()
                self.event = "API handoff authorized; waiting for FIFO runner lease"
            else:
                return f"unknown action: {action}"
            return self.event

    def status(self) -> dict[str, Any]:
        with self.lock:
            q = self.data.qpos[:N_JOINTS].copy()
            ee = np.stack([self.data.body(name).xpos.copy() for name in END_EFFECTORS])
            return {
                "mode": self.mode,
                "command_source": self.command_source,
                "event": self.event,
                "joints": q.tolist(),
                "lower": self.lower.tolist(),
                "upper": self.upper.tolist(),
                "homed": self._homed(),
                "rested": self._rested(),
                "launched": self.launched,
                "home_target": self.home.tolist(),
                "rest_target": self.rest.tolist(),
                "ramp_secs": self.ramp_secs,
                "policy_phase": self.policy_phase,
                "vertical_travel_m": (ee[:, 2] - self.policy_start_ee[:, 2]).tolist(),
                "api": {
                    "connected": self.api_connected,
                    "authorized": self.api_authorized,
                    "session_id": self.api_session_id,
                    "episode_id": self.api_episode_id,
                    "error": self.api_error,
                },
                "safety": {
                    "position_limits": bool(
                        np.all(q >= self.lower - JOINT_SENSOR_TOLERANCE)
                        and np.all(q <= self.upper + JOINT_SENSOR_TOLERANCE)
                    ),
                    "contact_count": int(self.data.ncon),
                    "controller_ok": self.controller_ok,
                    "drives_enabled": self.drives_enabled,
                },
            }

    def queue_snapshot(self) -> dict[str, Any]:
        try:
            with urlopen(f"{self.session_api_base}/v1/queue", timeout=5.0) as response:
                payload = json.load(response)
            entries = payload.get("entries", [])
            if not isinstance(entries, list):
                raise ValueError("invalid queue entries")
        except (OSError, URLError, ValueError, json.JSONDecodeError) as exc:
            entries = []
            error = str(exc)
        else:
            error = None
        with self.lock:
            current_session_id = self.api_session_id
            cleanup = dict(self.cleanup_state)
        return {
            "entries": entries,
            "current_session_id": current_session_id,
            "cleanup": cleanup,
            "error": error,
        }

    def start_queue_cleanup(self, session_ids: Any) -> str:
        valid = (
            isinstance(session_ids, list)
            and 0 < len(session_ids) <= 100
            and all(isinstance(value, str) and value.startswith("sess_") for value in session_ids)
            and len(session_ids) == len(set(session_ids))
        )
        if not valid:
            raise ValueError("session_ids must be 1-100 unique session IDs")
        with self.lock:
            if self.cleanup_state["running"]:
                raise ValueError("queue cleanup is already running")
            self.cleanup_state = {
                "running": True, "processed": 0, "total": len(session_ids),
                "message": "starting", "error": None,
            }
        threading.Thread(
            target=self._queue_cleanup_loop,
            args=(list(session_ids),),
            daemon=True,
            name="operator-queue-cleanup",
        ).start()
        return f"authorized cleanup started for {len(session_ids)} visible sessions"

    def _queue_cleanup_loop(self, session_ids: list[str]) -> None:
        try:
            for index, expected in enumerate(session_ids):
                with self.lock:
                    assigned = self.api_session_id
                    self.cleanup_state["message"] = f"waiting for {expected}"
                if assigned is None:
                    self._cleanup_ready()
                    assigned = self._wait_for_assignment(15.0)
                if assigned != expected:
                    raise RuntimeError(
                        f"unexpected assignment {assigned}; expected {expected}"
                    )
                self.action("stop")
                self._wait_for_disabled(20.0)
                with self.lock:
                    self.cleanup_state["processed"] = index + 1
                    self.cleanup_state["message"] = f"stopped {expected}"
            with self.lock:
                self.cleanup_state.update(
                    running=False, message="complete; Rest/DISABLED", error=None
                )
        except Exception as exc:
            self._log("queue_cleanup_halted", error=str(exc))
            with self.lock:
                self.cleanup_state.update(
                    running=False, message="halted", error=str(exc)
                )

    def _cleanup_ready(self) -> None:
        with self.lock:
            mode = self.mode
            assigned = self.api_session_id
        if assigned is not None:
            return
        if mode != "DISABLED":
            self.action("stop")
            self._wait_for_disabled(20.0)
        self.action("launch")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            with self.lock:
                if self.launched:
                    break
            time.sleep(0.1)
        else:
            raise RuntimeError("simulation launch timed out")
        self.action("home")
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            with self.lock:
                if self.mode == "READY" and self._homed():
                    break
            time.sleep(0.1)
        else:
            raise RuntimeError("simulation Home timed out")
        message = self.action("run_policy")
        if not message.startswith("API handoff authorized"):
            raise RuntimeError(message)

    def _wait_for_assignment(self, timeout: float) -> str:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.api_session_id is not None:
                    return self.api_session_id
            time.sleep(0.1)
        raise RuntimeError("assignment timed out")

    def _wait_for_disabled(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                complete = (
                    self.mode == "DISABLED" and self._rested()
                    and self.api_session_id is None and not self.drives_enabled
                )
            if complete:
                return
            time.sleep(0.1)
        raise RuntimeError("Stop/Rest/DISABLED timed out")

    def api_transport_paused(self, error):
        # Keep the lease and step counters. No new commands arrive while the
        # transport is down; an already validated bounded command may settle.
        with self.lock:
            self.api_connected = False
            self.api_error = error
            self.event = 'Connection interrupted; waiting to resume this run'
            self._log('api_reconnecting', session_id=self.api_session_id)

    def api_transport_resume(self):
        with self.lock:
            if not self.controller_ok or not self.api_authorized or self.mode not in {'API_ACTIVE', 'API_WAITING'}:
                raise RuntimeError('Robot control is not healthy enough to resume')

    def api_connection_changed(self, connected: bool, error: str | None = None) -> None:
        with self.lock:
            self.api_connected = connected
            self.api_error = error
            self._log("api_connection", connected=connected, error=error)
            reannounce_ready = bool(
                connected
                and self.api_authorized
                and self.mode == "API_WAITING"
                and self.api_client is not None
            )
            if not connected and self.api_lease_id is not None:
                self.target = self._bounded_current_pose()
                self.api_authorized = False
                self._clear_api_lease_locked()
                self.mode = "FAULT"
                self.command_source = "operator"
                self.policy_phase = "api_disconnected_hold"
                self.controller_ok = False
                self.event = "FAULT: Session API disconnected; both arms held"
            elif not connected and self.api_authorized and self.mode == "API_WAITING":
                self.event = "Session API disconnected before lease; waiting to re-announce readiness"
            if reannounce_ready:
                self.api_client.request_ready()
                self.event = "Session API reconnected; re-announced readiness"

    def api_error_received(self, error: str) -> None:
        """Record a server rejection without misreporting transport loss."""
        with self.lock:
            self.api_error = error
            self._log("api_error", error=error)

    def api_prepare_session(self, payload: dict[str, Any]) -> dict[str, Any] | None:
        with self.lock:
            if not self.api_authorized or self.mode != "API_WAITING":
                self.event = "API lease ignored: operator has not authorized handoff"
                return None
            fields = [payload.get(name) for name in ("session_id", "episode_id", "lease_id")]
            if not all(isinstance(value, str) and value for value in fields):
                self.event = "API lease rejected: malformed identifiers"
                return None
            self.api_session_id, self.api_episode_id, self.api_lease_id = fields
            self.api_last_step = -1
            self.api_observation_step = 0
            self.api_pending = None
            self.api_trajectory = None
            self.mode = "API_ACTIVE"
            self.command_source = "YAM Session API"
            self.policy_phase = "waiting_for_command"
            self.event = f"API lease active: {self.api_episode_id}; waiting for bimanual command"
            observation = self._api_observation_locked()
            self._log_observation("prepare_observation", observation)
            return observation

    def api_handle_joint_command(self, payload: dict[str, Any]) -> list[dict[str, Any]]:
        with self.lock:
            self._log(
                "joint_command_received",
                session_id=payload.get("session_id"),
                episode_id=payload.get("episode_id"),
                step_id=payload.get("step_id"),
            )
            reason: str | None = None
            if self.mode != "API_ACTIVE" or self.api_lease_id is None:
                reason = "operator_not_authorized"
            elif any(payload.get(name) != expected for name, expected in (
                ("session_id", self.api_session_id),
                ("episode_id", self.api_episode_id),
                ("lease_id", self.api_lease_id),
            )):
                reason = "lease_mismatch"
            step_id = payload.get("step_id")
            command_id = payload.get("command_id")
            if reason is None and (
                isinstance(step_id, bool) or not isinstance(step_id, int) or step_id <= self.api_last_step
            ):
                reason = "stale_or_duplicate_step"
            if reason is None and (not isinstance(command_id, str) or not command_id):
                reason = "missing_command_id"
            left = payload.get("left_joints_deg")
            right = payload.get("right_joints_deg")
            arms_valid = all(
                isinstance(arm, list)
                and len(arm) == 6
                and all(
                    not isinstance(value, bool)
                    and isinstance(value, (int, float))
                    and np.isfinite(value)
                    for value in arm
                )
                for arm in (left, right)
            )
            if reason is None and not arms_valid:
                reason = "bimanual_command_requires_two_six_joint_arrays"
            target = None
            if reason is None:
                target = np.deg2rad(np.asarray(left + right, dtype=np.float64))
                if np.any(target < self.lower) or np.any(target > self.upper):
                    reason = "joint_position_limit"
                elif np.max(np.abs(target - self.data.qpos[:N_JOINTS])) > MAX_COMMAND_DELTA:
                    reason = "joint_delta_limit"
                elif self.data.ncon:
                    reason = "active_contact"
                elif self.api_pending is not None:
                    reason = "command_in_flight"
            if reason is not None:
                self.event = f"API command rejected: {reason}"
                self._log(
                    "joint_command_rejected",
                    session_id=payload.get("session_id"),
                    episode_id=payload.get("episode_id"),
                    step_id=payload.get("step_id"),
                    reason=reason,
                )
                return [self._api_action_result_locked(payload, "rejected", reason)]
            assert target is not None
            self.target = target
            self.api_last_step = step_id
            self.api_pending = {
                "session_id": self.api_session_id,
                "episode_id": self.api_episode_id,
                "lease_id": self.api_lease_id,
                "step_id": step_id,
                "command_id": command_id,
            }
            self.api_pending_settled_since = None
            self.policy_phase = f"executing_step_{step_id}"
            self.event = f"API command step {step_id} accepted by safety gate"
            self._log(
                "joint_command_accepted",
                session_id=self.api_session_id,
                episode_id=self.api_episode_id,
                step_id=step_id,
            )
            return []

    def api_handle_joint_trajectory(self, payload: dict[str, Any]) -> list[dict[str, Any]]:
        """Atomically validate and buffer one complete locally paced trajectory."""
        with self.lock:
            if self.mode != "API_ACTIVE" or self.api_lease_id is None:
                return [self._api_raw_trajectory_result_locked(payload, "rejected", "operator_not_authorized")]
            trajectory_id = payload.get("trajectory_id")
            existing = (
                self.api_trajectory_records.get(trajectory_id)
                if isinstance(trajectory_id, str)
                else None
            )
            expected_step = (
                existing["trajectory"].first_step_id
                if existing is not None
                else self.api_last_step + 1
            )
            try:
                trajectory = parse_joint_trajectory(
                    payload,
                    expected_session_id=self.api_session_id,
                    expected_episode_id=self.api_episode_id,
                    expected_lease_id=self.api_lease_id,
                    expected_step_id=expected_step,
                    enforce_freshness=existing is None,
                )
            except TrajectorySchemaError as exc:
                self.event = f"API trajectory rejected: {exc.code}"
                return [self._api_raw_trajectory_result_locked(payload, "rejected", exc.code)]
            if existing is not None:
                if existing["fingerprint"] != trajectory.fingerprint:
                    return [self._api_trajectory_result_locked(
                        trajectory,
                        "rejected",
                        code="trajectory_id_conflict",
                        message="trajectory_id was reused with different content",
                        details={},
                    )]
                return [self._api_trajectory_record_result_locked(existing)]
            if self.api_pending is not None or self.api_trajectory is not None:
                return [self._api_trajectory_result_locked(
                    trajectory,
                    "rejected",
                    code="command_in_flight",
                    message="Another command is already active",
                    details={},
                )]
            try:
                targets = self._validate_sim_trajectory_locked(trajectory)
            except TrajectorySchemaError as exc:
                self.event = f"API trajectory rejected: {exc.code}"
                return [self._api_trajectory_result_locked(trajectory, "rejected", exc.code)]
            record = {
                "trajectory": trajectory,
                "fingerprint": trajectory.fingerprint,
                "status": "accepted",
                "step_id": None,
                "code": None,
                "message": None,
                "details": None,
                "progress_count": 0,
            }
            self.api_trajectory_records[trajectory.trajectory_id] = record
            while len(self.api_trajectory_records) > 32:
                self.api_trajectory_records.pop(next(iter(self.api_trajectory_records)))
            self.api_last_step = trajectory.last_step_id
            self.api_trajectory = {
                "trajectory": trajectory,
                "targets": targets,
                "index": 0,
                "next_at": time.monotonic(),
                "settled_since": None,
                "record": record,
            }
            self.policy_phase = "trajectory_buffered"
            self.event = (
                f"API trajectory {trajectory.trajectory_id} atomically validated; "
                f"{len(targets)} points queued at {trajectory.cadence_hz:g} Hz"
            )
            return [self._api_trajectory_record_result_locked(record)]

    def api_handle_stop(self, payload: dict[str, Any]) -> None:
        with self.lock:
            if payload.get("session_id") != self.api_session_id or payload.get("lease_id") != self.api_lease_id:
                return
            self._log(
                "stop_session",
                session_id=self.api_session_id,
                episode_id=self.api_episode_id,
                reason=payload.get("reason"),
            )
            self._clear_api_lease_locked()
            self.api_authorized = False
            self.mode = "STOPPING_REST"
            self.command_source = "Session API stop"
            self.policy_phase = "returning_to_rest_to_disable"
            self._start_ramp_locked(self.rest)
            self.event = "Session API stop: returning both arms to captured Rest before disable"

    def api_heartbeat_payload(self) -> dict[str, Any] | None:
        with self.lock:
            if self.api_lease_id is None:
                return None
            return self._api_envelope_locked("heartbeat")

    def api_next_observation_payload(self) -> dict[str, Any] | None:
        # Episode observations are command-correlated, never timer-driven. The
        # prepare handler emits observation 0, then the settle path emits exactly
        # one observation after each executed command. Heartbeats remain periodic.
        return None

    def api_station_status_payload(self) -> dict[str, Any]:
        with self.lock:
            joints = np.rad2deg(self.data.qpos[:N_JOINTS])
            in_bounds = bool(
                np.all(self.data.qpos[:N_JOINTS] >= self.lower - JOINT_SENSOR_TOLERANCE)
                and np.all(self.data.qpos[:N_JOINTS] <= self.upper + JOINT_SENSOR_TOLERANCE)
            )
            safety_ok = self.controller_ok and in_bounds and self.data.ncon == 0
            reason = None
            if not self.controller_ok:
                reason = "controller_fault"
            elif not in_bounds:
                reason = "joint_limit"
            elif self.data.ncon:
                reason = "active_contact"
            return {
                "schema_version": 1,
                "type": "station_status",
                "source": "simulation",
                "mode": self.mode,
                "observed_at": time.time(),
                "homed": self._homed(),
                "settled": bool(
                    self.ramp_goal is None
                    and self.api_pending is None
                    and self.api_trajectory is None
                ),
                "left_joints_deg": joints[:6].tolist(),
                "right_joints_deg": joints[6:].tolist(),
                "left_gripper": None,
                "right_gripper": None,
                "images": self._camera_references_locked(),
                "safety": {
                    "ok": safety_ok,
                    "estop_engaged": False,
                    "reason": reason,
                },
            }

    def _api_envelope_locked(self, message_type: str, **extra: Any) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "type": message_type,
            "session_id": self.api_session_id,
            "episode_id": self.api_episode_id,
            "lease_id": self.api_lease_id,
            **extra,
        }

    def _api_observation_locked(self) -> dict[str, Any]:
        joints = np.rad2deg(self.data.qpos[:N_JOINTS])
        return self._api_envelope_locked(
            "observation",
            step_id=self.api_observation_step,
            observed_at=time.time(),
            homed=self._homed(),
            settled=bool(
                self.ramp_goal is None
                and self.api_pending is None
                and self.api_trajectory is None
            ),
            left_joints_deg=joints[:6].tolist(),
            right_joints_deg=joints[6:].tolist(),
            left_gripper=None,
            right_gripper=None,
            images=self._camera_references_locked(),
        )

    @staticmethod
    def _log(event: str, **fields: Any) -> None:
        print(
            "[sim-gateway] " + json.dumps(
                {"at": time.time(), "event": event, **fields},
                separators=(",", ":"),
            ),
            flush=True,
        )

    def _log_observation(self, event: str, observation: dict[str, Any]) -> None:
        self._log(
            event,
            session_id=observation.get("session_id"),
            episode_id=observation.get("episode_id"),
            step_id=observation.get("step_id"),
            observed_at=observation.get("observed_at"),
            left_joints_deg=observation.get("left_joints_deg"),
            right_joints_deg=observation.get("right_joints_deg"),
            left_gripper=observation.get("left_gripper"),
            right_gripper=observation.get("right_gripper"),
        )

    def _camera_references_locked(self) -> dict[str, dict[str, str]]:
        return {
            role: {"url": f"{self.camera_reference_base}/{device}"}
            for role, device in JETSON_CAMERA_ROLES.items()
        }

    def _api_action_result_locked(
        self, payload: dict[str, Any], status: str, reason: str
    ) -> dict[str, Any]:
        return self._api_envelope_locked(
            "action_result",
            step_id=payload.get("step_id"),
            command_id=payload.get("command_id"),
            status=status,
            reason=reason,
        )

    def _api_trajectory_result_locked(
        self,
        trajectory: JointTrajectory,
        status: str,
        *,
        step_id: int | None = None,
        code: str | None = None,
        message: str | None = None,
        details: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        result = self._api_envelope_locked(
            "trajectory_result",
            trajectory_id=trajectory.trajectory_id,
            status=status,
            reported_at=time.time(),
        )
        optional = {
            "step_id": step_id,
            "code": code,
            "message": message,
            "details": details,
        }
        result.update({name: value for name, value in optional.items() if value is not None})
        return result

    def _api_trajectory_record_result_locked(self, record: dict[str, Any]) -> dict[str, Any]:
        return self._api_trajectory_result_locked(
            record["trajectory"],
            record["status"],
            step_id=record["step_id"],
            code=record["code"],
            message=record["message"],
            details=record["details"],
        )

    def _api_raw_trajectory_result_locked(
        self, payload: dict[str, Any], status: str, reason: str
    ) -> dict[str, Any]:
        return self._api_envelope_locked(
            "trajectory_result",
            trajectory_id=payload.get("trajectory_id"),
            status=status,
            reported_at=time.time(),
            code=reason,
            message="Trajectory rejected before motion",
            details={},
        )

    def _api_trajectory_progress_locked(
        self, trajectory: JointTrajectory, step_id: int, executed_steps: int
    ) -> dict[str, Any]:
        return self._api_envelope_locked(
            "trajectory_progress",
            trajectory_id=trajectory.trajectory_id,
            step_id=step_id,
            executed_at=time.time(),
        )

    def _validate_sim_trajectory_locked(
        self, trajectory: JointTrajectory
    ) -> tuple[tuple[float, ...], ...]:
        targets = tuple(point.joints_rad for point in trajectory.waypoints)
        previous = tuple(float(value) for value in self.data.qpos[:N_JOINTS])
        scratch = mujoco.MjData(self.model)
        for index, target in enumerate(targets):
            vector = np.asarray(target, dtype=np.float64)
            if np.any(vector < self.lower) or np.any(vector > self.upper):
                raise TrajectorySchemaError("joint_position_limit", waypoint=index)
            delta = float(np.max(np.abs(vector - np.asarray(previous))))
            if delta * trajectory.cadence_hz > MAX_VELOCITY + 1e-12:
                raise TrajectorySchemaError("joint_velocity_limit", waypoint=index)
            subdivisions = max(1, math.ceil(delta / 0.01))
            for sample_index in range(1, subdivisions + 1):
                alpha = sample_index / subdivisions
                sample = np.asarray(previous) + (vector - np.asarray(previous)) * alpha
                scratch.qpos[:] = self.data.qpos
                scratch.qpos[:N_JOINTS] = sample
                scratch.qvel[:] = 0.0
                mujoco.mj_forward(self.model, scratch)
                if scratch.ncon:
                    raise TrajectorySchemaError("predicted_collision", waypoint=index)
            previous = target
        return targets

    def _clear_api_lease_locked(self) -> None:
        self.api_session_id = None
        self.api_episode_id = None
        self.api_lease_id = None
        self.api_pending = None
        self.api_pending_settled_since = None
        self.api_trajectory = None

    def _homed(self) -> bool:
        return bool(
            np.max(np.abs(self.data.qpos[:N_JOINTS] - self.home))
            <= self.settle_tolerance
        )

    def _rested(self) -> bool:
        return bool(
            np.max(np.abs(self.data.qpos[:N_JOINTS] - self.rest))
            <= self.settle_tolerance
        )

    def _start_ramp_locked(self, goal: np.ndarray) -> None:
        self.ramp_start = self._bounded_current_pose()
        self.ramp_goal = goal.copy()
        self.ramp_started = time.monotonic()
        self.target = self.ramp_start.copy()

    def _advance_ramp_locked(self, now: float) -> None:
        if self.ramp_goal is None:
            return
        alpha = min(1.0, max(0.0, (now - self.ramp_started) / self.ramp_secs))
        self.target = self.ramp_start + alpha * (self.ramp_goal - self.ramp_start)
        if alpha >= 1.0:
            self.target = self.ramp_goal.copy()
            self.ramp_goal = None

    def _complete_pose_transition_locked(self) -> None:
        if self.mode == "HOMING" and self.ramp_goal is None and self._homed():
            self.mode = "READY"
            self.command_source = "none"
            self.policy_phase = "idle"
            self.event = "both homes verified; ready for policy handoff"
        elif self.mode == "STOPPING_REST" and self.ramp_goal is None and self._rested():
            self.mode = "DISABLED"
            self.command_source = "none"
            self.policy_phase = "disabled"
            self.launched = False
            self.drives_enabled = False
            self.event = "STOP COMPLETE: both arms at captured Rest; commands disabled"

    def _bounded_current_pose(self) -> np.ndarray:
        return np.clip(self.data.qpos[:N_JOINTS], self.lower, self.upper).copy()

    def _loop(self) -> None:
        period = 0.01
        physics_substeps = max(1, int(round(period / float(self.model.opt.timestep))))
        next_frame = 0.0
        renderer = None
        try:
            renderer = mujoco.Renderer(self.model, height=480, width=720)
            self._log(
                "simulation_clock",
                control_period_s=period,
                physics_timestep_s=float(self.model.opt.timestep),
                physics_substeps=physics_substeps,
            )
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth = 135
            camera.elevation = -22
            camera.distance = 2.0
            camera.lookat[:] = [0.25, 0.0, 0.30]
            while not self.stop_event.is_set():
                started = time.monotonic()
                with self.lock:
                    if self.mode == "POLICY_ACTIVE":
                        elapsed = started - self.policy_started
                        current_ee = np.stack(
                            [self.data.body(name).xpos.copy() for name in END_EFFECTORS]
                        )
                        if elapsed > 30.0:
                            self.mode = "FAULT"
                            self.command_source = "operator"
                            self.policy_phase = "timed_out"
                            self.target = self._bounded_current_pose()
                            self.event = "FAULT: fake policy timed out; both arms held"
                        elif self.policy_phase == "moving_up_30cm":
                            target_ee = self.policy_start_ee.copy()
                            target_ee[:, 2] += 0.30
                            self._step_cartesian_pair(target_ee, period)
                            if np.max(np.linalg.norm(current_ee - target_ee, axis=1)) < 0.012:
                                self.policy_phase = "returning_to_start"
                                self.event = "both 30 cm apexes verified; returning to start"
                        elif self.policy_phase == "returning_to_start":
                            self._step_cartesian_pair(self.policy_start_ee, period)
                            if np.max(
                                np.linalg.norm(current_ee - self.policy_start_ee, axis=1)
                            ) < 0.012:
                                self.policy_phase = "settling_at_start"
                                self.target = self.policy_start_joints.copy()
                                self.event = "both start positions reached; verifying settle"
                        elif self.policy_phase == "settling_at_start":
                            self.target = self.policy_start_joints.copy()
                            joints_close = np.max(
                                np.abs(self.data.qpos[:N_JOINTS] - self.policy_start_joints)
                            ) < HOME_TOLERANCE
                            velocity_low = np.max(np.abs(self.data.qvel[:N_JOINTS])) < 0.03
                            if joints_close and velocity_low:
                                if self.policy_settled_since is None:
                                    self.policy_settled_since = started
                                elif started - self.policy_settled_since >= 0.35:
                                    self.mode = "READY"
                                    self.command_source = "none"
                                    self.policy_phase = "completed"
                                    self.event = "COMPLETED: both arms returned and settled"
                            else:
                                self.policy_settled_since = None

                    if not np.all(np.isfinite(self.target)):
                        self.mode = "FAULT"
                        self.command_source = "operator"
                        self.policy_phase = "faulted"
                        self.controller_ok = False
                        self.target = self._bounded_current_pose()
                        self.event = "FAULT: non-finite target rejected"
                    elif np.any(self.target < self.lower) or np.any(self.target > self.upper):
                        self.mode = "FAULT"
                        self.command_source = "operator"
                        self.policy_phase = "faulted"
                        self.controller_ok = False
                        self.target = self._bounded_current_pose()
                        self.event = "FAULT: joint-position limit rejected"

                    self._advance_ramp_locked(started)

                    if self.mode == "API_ACTIVE" and self.api_trajectory is not None:
                        pending_trajectory = self.api_trajectory
                        trajectory = pending_trajectory["trajectory"]
                        index = pending_trajectory["index"]
                        if index < len(trajectory.waypoints) and started >= pending_trajectory["next_at"]:
                            self.target = np.asarray(
                                pending_trajectory["targets"][index], dtype=np.float64
                            )
                            point = trajectory.waypoints[index]
                            pending_trajectory["index"] = index + 1
                            pending_trajectory["next_at"] = started + 1.0 / trajectory.cadence_hz
                            record = pending_trajectory["record"]
                            record["progress_count"] = index + 1
                            self.policy_phase = f"trajectory_step_{point.step_id}"
                            if self.api_client is not None:
                                self.api_client.send(
                                    self._api_trajectory_progress_locked(
                                        trajectory, point.step_id, index + 1
                                    )
                                )

                    current_ctrl = self.data.ctrl[:N_JOINTS].copy()
                    max_step = MAX_VELOCITY * period
                    self.data.ctrl[:N_JOINTS] = current_ctrl + np.clip(
                        self.target - current_ctrl, -max_step, max_step
                    )
                    for _ in range(physics_substeps):
                        mujoco.mj_step(self.model, self.data)
                    if self.mode == "API_ACTIVE" and self.api_pending is not None:
                        joints_close = np.max(
                            np.abs(self.data.qpos[:N_JOINTS] - self.target)
                        ) < HOME_TOLERANCE
                        velocity_low = np.max(np.abs(self.data.qvel[:N_JOINTS])) < 0.03
                        if joints_close and velocity_low:
                            if self.api_pending_settled_since is None:
                                self.api_pending_settled_since = started
                            elif started - self.api_pending_settled_since >= API_SETTLE_TIME:
                                completed = self.api_pending
                                self.api_pending = None
                                self.api_pending_settled_since = None
                                self.api_observation_step += 1
                                self.policy_phase = "waiting_for_command"
                                self.event = f"API command step {completed['step_id']} executed once and settled"
                                if self.api_client is not None:
                                    observation = self._api_observation_locked()
                                    self._log_observation("command_settled", observation)
                                    self.api_client.send(self._api_action_result_locked(
                                        completed, "executed", "settled"
                                    ))
                                    self.api_client.send(observation)
                        else:
                            self.api_pending_settled_since = None
                    elif self.mode == "API_ACTIVE" and self.api_trajectory is not None:
                        pending_trajectory = self.api_trajectory
                        trajectory = pending_trajectory["trajectory"]
                        if pending_trajectory["index"] >= len(trajectory.waypoints):
                            joints_close = np.max(
                                np.abs(self.data.qpos[:N_JOINTS] - self.target)
                            ) < HOME_TOLERANCE
                            velocity_low = np.max(np.abs(self.data.qvel[:N_JOINTS])) < 0.03
                            if joints_close and velocity_low:
                                if pending_trajectory["settled_since"] is None:
                                    pending_trajectory["settled_since"] = started
                                elif started - pending_trajectory["settled_since"] >= API_SETTLE_TIME:
                                    record = pending_trajectory["record"]
                                    record["status"] = "completed"
                                    self.api_trajectory = None
                                    self.api_observation_step = trajectory.last_step_id + 1
                                    self.policy_phase = "waiting_for_command"
                                    self.event = (
                                        f"API trajectory {trajectory.trajectory_id} executed "
                                        "once and settled"
                                    )
                                    if self.api_client is not None:
                                        observation = self._api_observation_locked()
                                        self._log_observation("trajectory_settled", observation)
                                        self.api_client.send(observation)
                                        self.api_client.send(
                                            self._api_trajectory_record_result_locked(record)
                                        )
                            else:
                                pending_trajectory["settled_since"] = None
                    self._complete_pose_transition_locked()
                    if started >= next_frame:
                        renderer.update_scene(self.data, camera=camera)
                        rgb = renderer.render()
                        ok, encoded = cv2.imencode(
                            ".jpg",
                            cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                            [cv2.IMWRITE_JPEG_QUALITY, 85],
                        )
                        if ok:
                            self.latest_jpeg = encoded.tobytes()
                        next_frame = started + 0.08
                self.stop_event.wait(max(0.0, period - (time.monotonic() - started)))
        except Exception as exc:
            with self.lock:
                self._log("controller_exception", error=str(exc))
                self.render_error = str(exc)
                self.mode = "FAULT"
                self.controller_ok = False
                self.event = f"FAULT: MuJoCo renderer failed: {exc}"
        finally:
            if renderer is not None:
                renderer.close()

    def _step_cartesian_pair(self, targets: np.ndarray, period: float) -> None:
        """Position-only damped least-squares IK for both fake runner commands."""
        jacobian = np.zeros((len(END_EFFECTORS) * 3, self.model.nv))
        cartesian_velocity = np.zeros(len(END_EFFECTORS) * 3)
        for index, name in enumerate(END_EFFECTORS):
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            linear = np.zeros((3, self.model.nv))
            rotational = np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(self.model, self.data, linear, rotational, body_id)
            jacobian[index * 3 : index * 3 + 3] = linear
            error = targets[index] - self.data.body(name).xpos
            cartesian_velocity[index * 3 : index * 3 + 3] = np.clip(
                2.2 * error, -0.12, 0.12
            )
        arm_jacobian = jacobian[:, :N_JOINTS]
        damping = 2e-3
        solve = arm_jacobian @ arm_jacobian.T + damping * np.eye(len(END_EFFECTORS) * 3)
        joint_velocity = arm_jacobian.T @ np.linalg.solve(solve, cartesian_velocity)
        joint_velocity = np.clip(joint_velocity, -MAX_VELOCITY, MAX_VELOCITY)
        self.target = np.clip(
            self.data.ctrl[:N_JOINTS] + joint_velocity * period,
            self.lower,
            self.upper,
        )


class Handler(BaseHTTPRequestHandler):
    simulator: OperatorSimulator
    jetson_camera_base: str

    def do_GET(self) -> None:
        if self.path == "/":
            body = PAGE.encode("utf-8")
            self._send(200, "text/html; charset=utf-8", body)
        elif self.path.startswith("/api/frame.jpg"):
            with self.simulator.lock:
                frame = self.simulator.latest_jpeg
            if frame is None:
                self._send(503, "text/plain", b"render starting")
            else:
                self._send(200, "image/jpeg", frame)
        elif self.path == "/api/status":
            self._json(200, self.simulator.status())
        elif self.path == "/api/error-history":
            import os
            from YAM_control.error_history import cached_summary
            self._json(200, cached_summary(os.environ.get('YAM_DATASET_ROOT')))
        elif self.path == "/api/queue":
            self._json(200, self.simulator.queue_snapshot())
        elif self.path.startswith("/api/jetson-camera/"):
            self._proxy_jetson_camera()
        else:
            self._json(404, {"error": "not found"})

    def _proxy_jetson_camera(self) -> None:
        camera_id = self.path.rsplit("/", 1)[-1]
        if camera_id not in JETSON_CAMERA_IDS:
            self._json(404, {"error": "unknown camera"})
            return
        headers_sent = False
        try:
            with urlopen(f"{self.jetson_camera_base}/{camera_id}", timeout=5.0) as upstream:
                content_type = upstream.headers.get(
                    "content-type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_response(200)
                self.send_header("content-type", content_type)
                self.send_header("cache-control", "no-store")
                self.end_headers()
                headers_sent = True
                read_available = getattr(upstream, "read1", upstream.read)
                while chunk := read_available(64 * 1024):
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return
        except (OSError, URLError) as exc:
            if not headers_sent:
                self._json(502, {"error": f"Jetson camera {camera_id} unavailable: {exc}"})

    def do_POST(self) -> None:
        if self.path not in {"/api/action", "/api/queue-cleanup", "/api/waypath"}:
            self._json(404, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("content-length", "0"))
            payload = json.loads(self.rfile.read(length) or b"{}")
            if self.path == "/api/action":
                message = self.simulator.action(str(payload.get("action", "")))
            elif self.path == "/api/waypath":
                handler = getattr(self.simulator, "waypath_action", None)
                if handler is None:
                    raise ValueError("waypath controls are unavailable for this backend")
                message = handler(payload)
            else:
                message = self.simulator.start_queue_cleanup(payload.get("session_ids"))
            self._json(200, {"ok": True, "message": message})
        except (ValueError, json.JSONDecodeError) as exc:
            self._json(400, {"ok": False, "error": str(exc)})
        except Exception as exc:
            LOGGER.exception("operator action failed")
            self._json(
                409,
                {"ok": False, "error": f"{type(exc).__name__}: {exc}"},
            )

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _json(self, status: int, payload: Any) -> None:
        self._send(status, "application/json", json.dumps(payload).encode("utf-8"))

    def _send(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("content-type", content_type)
        self.send_header("content-length", str(len(body)))
        self.send_header("cache-control", "no-store")
        self.end_headers()
        self.wfile.write(body)


class OperatorHTTPServer(ThreadingHTTPServer):
    daemon_threads = True


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the local YAM operator simulation UI")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8096)
    parser.add_argument(
        "--jetson-camera-base", default="http://127.0.0.1:8089"
    )
    parser.add_argument(
        "--camera-reference-base", default="http://127.0.0.1:8089"
    )
    parser.add_argument(
        "--session-api-websocket",
        default="wss://yam-session-api.n5hthc3gj4cqy.us-east-1.cs.amazonlightsail.com/v1/jetsons/connect",
    )
    parser.add_argument(
        "--session-api-base",
        default="https://yam-session-api.n5hthc3gj4cqy.us-east-1.cs.amazonlightsail.com",
    )
    parser.add_argument("--jetson-id", default="yam-1")
    parser.add_argument(
        "--jetson-token-file", type=Path, default=Path("/tmp/yam_session_api_jetson_token")
    )
    parser.add_argument(
        "--pose-config", type=Path, default=POSE_CONFIG_PATH
    )
    args = parser.parse_args()
    simulator = OperatorSimulator(args.pose_config)
    simulator.public_base = f"http://127.0.0.1:{args.port}"
    simulator.session_api_base = args.session_api_base.rstrip("/")
    simulator.camera_reference_base = args.camera_reference_base.rstrip("/")
    api_client = SessionApiSimClient(
        simulator,
        args.session_api_websocket,
        args.jetson_id,
        args.jetson_token_file,
    )
    simulator.attach_api_client(api_client)
    Handler.simulator = simulator
    Handler.jetson_camera_base = args.jetson_camera_base.rstrip("/")
    server = OperatorHTTPServer((args.host, args.port), Handler)
    simulator.start()
    api_client.start()
    title = getattr(simulator, "startup_title", "YAM operator simulator: http://{host}:{port}")
    print(title.format(host=args.host, port=args.port), flush=True)
    print(
        getattr(
            simulator,
            "startup_safety",
            "SIM ACTUATION ONLY: Session API and Jetson camera connections enabled; no CAN/RoboCurve",
        ),
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        api_client.stop()
        simulator.stop()


if __name__ == "__main__":
    main()
