// "Let Gemini drive" half of the Gemini panel (markup in demo.js) + call trace. POST /api/agent starts the closed loop in
// src/gemini_agent.py; every Gemini call arrives as a 'gemini_call' log event and becomes a card here:
// the image it saw, the sensor text it read, the function it chose + its reason, latency, tokens, quota.
const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

const GOALS = {
  pill_bottle: 'Unscrew the cap of the pill bottle and lift it off.',
  pill_tube: 'Unscrew the cap of the pill tube and lift it off.',
  eye_dropper: 'Dispense exactly 2 drops from the eye-drop bottle.',
};

const css = `
#tr-list { list-style: none; padding: 0; margin: 8px 0 0; max-height: 520px; overflow-y: auto; }
#tr-list li { border: 1px solid var(--line); border-radius: 6px; padding: 6px; margin-bottom: 6px; background: var(--panel); font-size: 12px; }
#tr-list .call { display: grid; grid-template-columns: 96px 1fr; gap: 8px; }
#tr-list img { width: 96px; border-radius: 4px; border: 1px solid var(--line); }
#tr-list .fn { font-family: ui-monospace, Consolas, monospace; color: var(--accent); font-weight: 600; }
#tr-list .meta { color: var(--muted); font-size: 11px; }
#tr-list .reason { margin: 2px 0; }
#tr-list pre { white-space: pre-wrap; font-size: 10.5px; margin: 4px 0 0; color: var(--muted); }
#tr-list .res-ok { color: var(--good); } #tr-list .res-no { color: var(--bad); }
#tr-list .fin { border-color: var(--good); } #tr-list .fin.no { border-color: var(--bad); }
`;

const MAX_CALLS = 8;
const msg = (t) => { if ($('dm-msg')) { $('dm-msg').textContent = t; $('dm-msg').className = 'muted'; } };

function build() {
  if (!$('tr-goal')) return;      // panel markup comes from demo.js
  const st = document.createElement('style'); st.textContent = css; document.head.append(st);
  $('tr-goal').value = GOALS.pill_bottle;
  $('tr-go').onclick = start;
  $('tr-clear').onclick = () => { $('tr-list').innerHTML = ''; };
}

let goalTouched = false, lastObject = null, ctlState = null;
async function start() {
  const src = document.querySelector('#dm-src button.on')?.dataset.src || 'sim';
  const body = { goal: $('tr-goal').value.trim(), max_steps: MAX_CALLS, source: src };
  if (!body.goal) return;
  try {
    const r = await fetch('/api/agent', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const j = await r.json();
    msg(j.msg ?? '');
  } catch (e) { msg('server unreachable'); }
}

function card(html, cls = '') {
  const li = document.createElement('li');
  if (cls) li.className = cls;
  li.innerHTML = html;
  $('tr-list').prepend(li);
  while ($('tr-list').children.length > 40) $('tr-list').lastChild.remove();
  return li;
}

const fmtArgs = (a) => Object.entries(a || {}).map(([k, v]) => `${k}=${typeof v === 'number' ? +v.toFixed(2) : v}`).join(', ');
let lastCall = null;

function onLog(e) {
  if (e.kind === 'status') { ctlState = e.state; $('tr-go').disabled = e.state !== 'idle'; return; }
  if (e.kind === 'agent_start') {
    card(`<b>Goal</b>: ${esc(e.goal)} <span class="meta">· up to ${e.max_steps} calls · image: ${esc(e.source)}</span>`);
  } else if (e.kind === 'gemini_call') {
    if (e.error && !e.function) { card(`<span class="res-no">Gemini call ${e.step}: ${esc(e.error)}</span>`); return; }
    lastCall = card(`<div class="call">
        ${e.image ? `<img src="${e.image}" alt="frame sent">` : '<div class="meta">no image</div>'}
        <div><div><span class="meta">#${e.step}</span> <span class="fn">${esc(e.function)}(${esc(fmtArgs(e.args))})</span></div>
          <div class="reason">${esc(e.reason)}</div>
          <div class="meta">${e.latency_s} s · ${e.tokens_in ?? '?'} in / ${e.tokens_out ?? '?'} out tokens</div>
          <div class="res meta">running…</div>
          <details><summary class="meta">sensor text Gemini read</summary><pre>${esc(e.observation)}</pre></details></div></div>`);
  } else if (e.kind === 'skill' && e.by === 'gemini' && lastCall) {
    const r = e.result || {}, ok = r.ok !== false;
    const bits = [r.blocked && `blocked: ${r.blocked}`, r.error, r.contact !== undefined && (r.contact ? `contact ${r.hold_ma} mA` : 'no contact'), `${e.duration_s} s`].filter(Boolean);
    lastCall.querySelector('.res').innerHTML = `<span class="${ok ? 'res-ok' : 'res-no'}">${ok ? 'done' : 'failed'}</span> · ${esc(bits.join(' · '))}`;
  } else if (e.kind === 'agent_finish') {
    card(`<b class="${e.success ? 'res-ok' : 'res-no'}">${e.success ? 'Finished' : 'Gave up'}</b> after ${e.steps} calls: ${esc(e.summary)}`,
      'fin' + (e.success ? '' : ' no'));
  } else if (e.kind === 'agent_done' && e.ok === false && e.reason) {
    card(`<span class="res-no">Agent stopped: ${esc(e.reason)}</span>`);
  }
}

function onState(s) {   // follow the sim's object with a matching default goal until the user edits it
  const obj = s.sim?.object;
  if (!obj || obj === lastObject) return;
  lastObject = obj;
  if (!goalTouched && GOALS[obj]) $('tr-goal').value = GOALS[obj];
}

function init() {
  build();
  if (!$('tr-goal')) return;
  $('tr-goal').addEventListener('input', () => { goalTouched = true; });
  window.twin.onLog.push(onLog);
  window.twin.onState.push(onState);
  fetch('/api/status').then((r) => r.ok ? r.json() : null).then((s) => {
    if (!s) { $('tr-go').disabled = true; msg('no controller (start main.py --sim)'); return; }
    ctlState = s.state; $('tr-go').disabled = s.state !== 'idle';
  }).catch(() => {});
}

if (window.twin) init(); else addEventListener('twin-ready', init, { once: true });
