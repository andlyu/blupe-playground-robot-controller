"""Operator-only read-only error view; no control actions."""
PANEL = '''<section class="panel" style="grid-column:1/-1" id="error-handling">
<h2>Robot failures</h2><p>What happened, and what happens next.</p>
<button type="button" id="errors-refresh">Refresh</button>
<p id="errors-coverage" role="status">Loading recorded history…</p>
<details><summary>What the counts include</summary>
<p>Counts are affected runs, not retries. One run can have several error types. Saved history excludes startup faults, successful recoveries without a terminal error, and model errors recorded only by the hosted runner. Zero means no matching saved evidence.</p></details>
<div id="errors-list" style="max-height:650px;overflow:auto"></div></section>'''
SCRIPT = '''<script>
(()=>{
const status=document.getElementById('errors-coverage'), list=document.getElementById('errors-list'), button=document.getElementById('errors-refresh');
const element=(tag,text)=>{const node=document.createElement(tag);node.textContent=text;return node;};
async function refresh(){button.disabled=true;try{
const response=await fetch('/api/error-history');if(!response.ok)throw Error('History unavailable');const data=await response.json();
if(!data.available)throw Error(data.refreshing ? 'Loading recorded history…' : 'Recorded history is unavailable');
status.textContent=`${data.stale ? 'STALE · ' : ''}${data.refresh_error ? 'Refresh failed · ' : ''}${data.recorded_runs} recorded runs · Updated ${new Date(data.generated_at*1000).toLocaleTimeString()}${data.unreadable_records ? ` · ${data.unreadable_records} records could not be read` : ''}`;
const opened=new Set([...list.querySelectorAll('details[open]')].map(node=>node.dataset.issue));
list.replaceChildren();for(const issue of data.issues){const card=document.createElement('article');card.style.cssText='border-bottom:1px solid #aaa;padding:16px 0';
card.append(element('h3',issue.name),element('p',`${issue.counts['24h']} affected ${issue.counts['24h'] === 1 ? 'run' : 'runs'} in the last 24 hours`));
for(const [label,text] of [['What happened: ',issue.meaning],['What happens next: ',issue.handling]]){
const paragraph=document.createElement('p');paragraph.append(element('strong',label),document.createTextNode(text));card.append(paragraph);}
const history=document.createElement('details');history.dataset.issue=issue.id;history.open=opened.has(issue.id);
history.append(element('summary','History & error details'),element('p',`Last 7 days: ${issue.counts['7d']} runs · All recorded: ${issue.counts.all} runs`));
if(!issue.recent.length)history.append(element('p','No matching saved evidence.'));
for(const run of issue.recent){history.append(element('p',`${new Date(run.timestamp*1000).toLocaleString()} · ${run.episode_id} · ${run.outcome}`));const evidence=element('pre',run.evidence);evidence.style.cssText='white-space:pre-wrap;overflow-wrap:anywhere';history.append(evidence);}card.append(history);list.append(card);}
}catch(error){status.textContent=error.message;list.replaceChildren();}finally{button.disabled=false;}}
button.addEventListener('click',refresh);refresh();setInterval(refresh,5000);
})();
</script>'''


def add_error_history(page):
    return page.replace('</main>', PANEL + '</main>', 1).replace('</body>', SCRIPT + '</body>', 1)
