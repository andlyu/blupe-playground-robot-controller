"""Standard YAM console template with SO101 capability bindings, no hardware imports."""
from pathlib import Path
import re


def standard_page():
    page = (Path(__file__).parent / 'templates' / 'operator.html').read_text()
    page = page.replace('Bimanual YAM Operator Console / SIM','SO101 Operator Console')
    page = page.replace('Bimanual YAM Operator Console','SO101 Operator Console')
    page = page.replace('LOCAL / BIMANUAL MUJOCO / NO HARDWARE','OPERATOR / PHYSICAL SO101')
    page = page.replace('Live MuJoCo Bimanual YAM','Physical SO101')
    page = page.replace('Safety gate','Safety Guardrails')
    page = re.sub(r'<img id="arm"[^>]*>', '<img id="arm" alt="Live robot camera">', page)
    controls = '''<div class="controls"><button id="hold" class="stop" onclick="act('hold')">Stop &amp; Hold</button>
<button id="launch" onclick="act('enable')">Launch Arm</button><button id="home" onclick="act('home')">Move Home</button>
<button id="zero" onclick="act('zero')">Move Zero</button><button id="run" class="run" onclick="act('cloud_ready')">Run via Session API</button>
</div><div class="camera-bank"><h3>Robot Cameras</h3><div id="camera-grid" class="camera-grid"></div><div id="camera-status" class="queue-note">Connecting camera…</div></div>'''
    page,count = re.subn(r'<div class="controls">.*?</section>', controls+'</section>',page,count=1)
    assert count==1
    checks = ''.join(f'<div class="check"><span>{label}</span><span id="{key}">—</span></div>' for key,label in [
        ('api','SESSION API'),('source','COMMAND SOURCE'),('phase','POLICY PHASE'),('position','POSITION LIMITS'),
        ('homed','HOME POSITION'),('zero-state','ZERO POSITION'),('commands','COMMAND ACCEPTANCE'),('controller','CONTROLLER')])
    page = re.sub(r'<div class="checks">.*?<div class="queue-bank">','<div class="checks">'+checks+'</div><div class="queue-bank">',page,count=1)
    page = re.sub(r'<div class="queue-actions">.*?<div id="joints"', '''<div class="queue-actions"><button id="auto-queue" onclick="act('auto_queue',{enabled:this.dataset.enabled!=='true'})">Enable Auto-queue</button><button onclick="act('cloud_pause')">Pause Queue</button></div>
<div id="queue-note" class="queue-note">Enable auto-queue at home to run queued tasks in sequence. Stop or Pause Queue disables it.</div></div><div id="joints"''',page,count=1)
    page = re.sub(r'<div class="foot">.*?</div>', '''<details class="pose-settings"><summary>Robot setup &amp; manual targets</summary>
<div class="controls"><button onclick="act('capture_zero')">Capture Zero</button><button onclick="act('capture_home')">Capture Home</button></div>
<p>Five joint angles in degrees; gripper from 0 to 1.</p><label>Joint targets <input id="targets" placeholder="0, 0, 0, 0, 0"></label>
<label>Gripper <input id="gripper" type="number" min="0" max="1" step="0.01" value="0.5"></label><button onclick="manualMove()">Move joints</button></details>
<div class="foot">Stop holds the current measured pose and pauses cloud control. It does not release torque. LeRobot software checks pause if Python stalls.</div>''',page,count=1)
    page = page[:page.index('<script>')] + SCRIPT + '\n</body></html>'
    page = page.replace('</head>','''<style>.joint{grid-template-columns:130px 1fr 80px}.camera-grid{grid-template-columns:repeat(auto-fit,minmax(160px,1fr))}.camera-feed figcaption{color:#eee}.pose-settings{margin-top:18px;font:12px ui-monospace,monospace}.pose-settings label{display:block;margin:8px 0}.pose-settings input{padding:8px;width:100%;font:inherit;border:1px solid var(--ink)}.pose-settings button{min-height:44px;font-size:12px}.pose-settings summary{cursor:pointer}#arm{object-fit:contain}</style></head>''')
    return page


SCRIPT = r'''<script>
const ticket=__TICKET__;
const log=document.getElementById('log');let prior='',cameraRoles=[];
const el=id=>document.getElementById(id);
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function note(message){log.innerHTML=`${new Date().toLocaleTimeString()} ${esc(message)}<br>`+log.innerHTML;}
async function act(action,extra={}){try{const r=await fetch('api/action',{method:'POST',headers:{'Content-Type':'application/json','X-Blupe-Control':ticket},body:JSON.stringify({action,...extra})});const d=await r.json();note(d.error||d.message||(r.ok?action.replaceAll('_',' '):`Request failed (${r.status})`));}catch(e){note(e.message)}}
function manualMove(){act('move',{joints_deg:el('targets').value.split(',').map(Number),gripper:Number(el('gripper').value)})}
function flag(id,ok,yes='CLEAR',no='BLOCKED'){el(id).textContent=ok?yes:no;el(id).className=ok?'ok':'bad'}
function cameras(roles){if(JSON.stringify(roles)!==JSON.stringify(cameraRoles)){cameraRoles=roles;el('camera-grid').innerHTML=roles.slice(1).map(role=>`<figure class="camera-feed"><img data-role="${esc(role)}" alt="${esc(role)} camera"><figcaption>${esc(role)}</figcaption></figure>`).join('')}
if(roles.length){el('arm').onload=()=>{el('camera-status').textContent=roles[0]+' · updated '+new Date().toLocaleTimeString()};el('arm').onerror=()=>{el('camera-status').textContent='Camera unavailable';el('arm').removeAttribute('src')};el('arm').src='api/camera/'+encodeURIComponent(roles[0])+'?t='+Date.now()}
for(const image of document.querySelectorAll('img[data-role]'))image.src='api/camera/'+encodeURIComponent(image.dataset.role)+'?t='+Date.now()}
async function tick(){try{const r=await fetch('api/status');if(!r.ok)throw Error('Controller unavailable');const s=await r.json();const cloud=typeof s.cloud_execution==='object'?s.cloud_execution:{};const active=s.mode==='active';const busy=cloud.session_active||cloud.queue_ready||s.manual_motion;
el('mode').textContent=s.mode==='readonly'?'DISABLED':s.mode==='active'?(s.manual_motion?'MOVING':busy?'API ACTIVE':'READY'):s.mode.toUpperCase();el('mode').className='mode '+(s.mode==='fault'?'fault':busy?'policy':'');
flag('api',cloud.connected,cloud.session_active?'CONNECTED / ACTIVE':'CONNECTED / IDLE','DISCONNECTED');el('source').textContent=(cloud.session_active||cloud.queue_ready)?'SESSION API':'OPERATOR';el('phase').textContent=s.manual_motion?'MOVING TO POSE':s.manual_motion_error?s.manual_motion_error:cloud.command_active?'EXECUTING':cloud.queue_ready?'WAITING FOR TASK':'PAUSED';
flag('position',s.mode!=='fault');flag('controller',s.mode!=='fault','HEALTHY','FAULT');flag('commands',active,'ENABLED','PAUSED');
const aq=el('auto-queue');aq.hidden=false;aq.dataset.enabled=String(!!cloud.auto_queue);aq.textContent=cloud.auto_queue?'Disable Auto-queue':'Enable Auto-queue';aq.disabled=!cloud.auto_queue&&(!active||busy||!cloud.connected||!s.at_home);
const home=s.saved_poses?.home,zero=s.saved_poses?.zero;flag('homed',s.at_home===true, 'AT HOME',home?'CAPTURED':'NOT CAPTURED');flag('zero-state',s.at_zero===true,'AT ZERO',zero?'CAPTURED':'NOT CAPTURED');
el('launch').disabled=busy||s.mode==='fault'||active;el('home').disabled=!active||busy||!home;el('zero').disabled=!active||busy||!zero;el('run').disabled=!active||busy||!cloud.connected||s.at_home!==true;
el('joints').innerHTML=s.joints_deg.map((v,i)=>`<div class="joint"><b>${esc(s.joint_names[i])}</b><div class="track"><div class="fill" style="width:${Math.max(0,Math.min(100,(v+180)/360*100))}%"></div></div><span>${v.toFixed(1)}°</span></div>`).join('')+ (Array.isArray(s.gripper)?s.gripper:[s.gripper]).map((g,i)=>`<div class="joint"><b>gripper ${i+1}</b><div class="track"><div class="fill" style="width:${g*100}%"></div></div><span>${(g*100).toFixed(1)}%</span></div>`).join('');
if(Array.isArray(s.gripper)){document.querySelector('details').style.display='none';}
if(s.calibration_ready===false){
el('mode').textContent='MONITORING';el('phase').textContent=s.setup_message;
flag('position',false,'VERIFIED','CALIBRATION PENDING');
for(const id of ['launch','home','zero','run','hold'])el(id).disabled=true;
document.querySelector('.pose-settings').hidden=true;
el('joints').innerHTML=(s.arms||[]).map(arm=>`<section><h3>${esc(arm.name.replaceAll('_',' '))}</h3><p>${esc(arm.port)}</p>${arm.motors.map(m=>`<div class="joint"><b>Servo ${m.id}</b><span>${m.position_raw} ticks</span><span>${m.torque_enabled?'Torque ON':'Torque OFF'}</span></div>`).join('')}</section>`).join('');
document.querySelector('.foot').textContent='Read-only monitor. Servo readings are raw encoder ticks, not calibrated angles. No motor writes or calibration changes.';
}
const event=s.error||cloud.error||'';if(event&&event!==prior)note(event);prior=event;cameras(s.cameras||[]);
}catch(e){flag('controller',false,'HEALTHY','UNAVAILABLE')}el('clock').textContent=new Date().toLocaleTimeString();setTimeout(tick,500)}
async function queueTick(){try{const r=await fetch('api/queue');if(!r.ok)throw Error('Queue unavailable');const q=await r.json();let entries=q.entries||[];if(q.current_session_id&&!entries.some(e=>e.session_id===q.current_session_id))entries=[{session_id:q.current_session_id,status:'active'},...entries];el('queue-count').textContent=entries.length;el('queue-list').innerHTML=entries.length?entries.map((e,i)=>`<li class="queue-item"><b>${i+1}</b><span class="queue-id">${esc(e.session_id)}</span><span class="queue-state">${esc(e.status||e.state||'queued')}</span></li>`).join(''):'<li class="queue-empty">Queue empty</li>';el('queue-note').textContent=q.message||'Enable auto-queue at home to run queued tasks in sequence. Stop or Pause Queue disables it.';}catch(e){el('queue-note').textContent=e.message}setTimeout(queueTick,2000)}tick();queueTick();
</script>'''

PAGE = standard_page()
