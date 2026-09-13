"""Operator timing display. No control commands or reset buttons."""
PANEL = '''<section class="panel" style="grid-column:1/-1" id="control-timing">
<h2>Motor communication</h2><p id="control-timing-summary" role="status">Waiting for controller timing…</p>
<div id="control-timing-arms"></div>
<p>Per-arm command dispatch gaps. Amber: over 50 ms. Red: over 100 ms, stale feedback or controller fault. These are diagnostic alerts, not firmware watchdog settings. Peak and counts reset when arm drivers are launched.</p>
</section>'''
SCRIPT = '''<script>
let controlTimingReceived=0;
function updateControlTiming(timing,mode){
const summary=document.getElementById('control-timing-summary'), arms=document.getElementById('control-timing-arms');
controlTimingReceived=Date.now();arms.replaceChildren();
if(!timing || !timing.available || timing.status_age_ms>1000){summary.textContent='UNKNOWN · controller timing unavailable or stale';summary.style.color='#a5261c';return;}
const entries=Object.entries(timing.arms||{});
summary.textContent=(timing.isolated?'Dedicated motor process':'Shared process · control isolation unavailable')+(entries.length?' · telemetry '+Math.round(timing.status_age_ms)+' ms old':' · arms inactive');
summary.style.color=timing.isolated?'#24633b':'#805c25';
const ms=v=>Number.isFinite(v)?v.toFixed(1)+' ms':'unknown';
for(const [arm,t] of entries){
const bad=!!t.runtime_fault || !t.control_thread_alive || !t.can_worker_running;
const unknown=!t.complete;
const red=bad || (t.command_age_ms>100 || t.feedback_age_ms>100 || t.max_gap_60s_ms>100);
const amber=unknown || (t.command_age_ms>50 || t.feedback_age_ms>50 || t.max_gap_60s_ms>50);
const line=document.createElement('p');line.style.color=red?'#a5261c':amber?'#805c25':'#24633b';
line.textContent=arm.toUpperCase()+' · '+(bad?'FAULT':unknown?'UNKNOWN':red?'DELAY':amber?'WARNING':'HEALTHY')+
' · Command age '+ms(t.command_age_ms)+' · Feedback age '+ms(t.feedback_age_ms)+
' · Worst gap: '+ms(t.max_gap_60s_ms)+' / 60s, '+ms(t.max_gap_ms)+' / launch'+
' · Delayed motor updates >50 / >100 ms: '+(t.updates_over_50ms??'?')+' / '+(t.updates_over_100ms??'?')+
' · Transaction errors '+(t.transaction_errors??'?');
arms.append(line);
}
}
setInterval(()=>{if(controlTimingReceived && Date.now()-controlTimingReceived>2500){
const summary=document.getElementById('control-timing-summary');summary.textContent='UNKNOWN · console status disconnected or stale';summary.style.color='#a5261c';
document.getElementById('control-timing-arms').replaceChildren();controlTimingReceived=0;
}},500);
</script>'''


def add_control_timing(page):
    return (page.replace('</main>', PANEL+'</main>', 1)
            .replace('</head>', SCRIPT+'</head>', 1)
            .replace("const m=document.getElementById('mode');",
                     "updateControlTiming(s.control_timing,s.mode);const m=document.getElementById('mode');", 1))
