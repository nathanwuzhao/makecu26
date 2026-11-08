// Demo add-ons for the twin page (needs window.twin from app.js):
//  1. One "Gemini" panel, two modes sharing the goal box + image source:
//     "Plan it": one frame -> POST /api/vision -> object, cap state and a skill plan; "Run plan" -> /api/plan.
//     "Let Gemini drive": closed loop (POST /api/agent), wired + rendered by trace.js into #tr-list.
//  2. "Innovation tour": camera moves + callouts + DOF arrows over the 3D view, explaining each axis.
//     "Labels" shows the arrows/callouts without the tour. Esc stays the page's E-stop (not handled here).
// Degrades quietly when the server has no controller / no /api/vision / no /api/sim.

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// fetch JSON without throwing: {ok, code, j}
async function call(method, path, body) {
  try {
    const r = await fetch(path, method === 'GET' ? {} : {
      method, headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body ?? {}) });
    let j = null;
    try { j = await r.json(); } catch {}
    return { ok: r.ok, code: r.status, j };
  } catch (e) { return { ok: false, code: 0, j: { msg: 'server unreachable' } }; }
}

// ================= 1. Vision -> Gemini panel =================

const box = $('demo');
box.innerHTML = `
  <div class="dm-head"><b>Gemini</b>
    <span class="seg" id="dm-src" title="What Gemini sees">
      <button data-src="sim" class="on" title="The simulated top webcam (matches the simulated robot)">Sim cam</button>
      <button data-src="camera" title="The real gripper webcam">Real cam</button>
    </span></div>
  <textarea id="tr-goal" rows="2" title="What you want done. Used by both buttons."></textarea>
  <div class="dm-row">
    <button id="dm-analyze" title="Gemini looks at ONE frame, names the object and writes a skill plan. You review it, then press Run plan.">
      Plan it <small>1 look → plan you run</small></button>
    <button id="tr-go" class="go" title="Gemini picks one skill at a time, sees the camera + sensors again after each, and stops when the goal is met.">
      Let Gemini drive <small>closed loop, ≤ 8 calls</small></button>
  </div>
  <div id="dm-msg" class="muted"></div>
  <div id="dm-out" hidden>
    <div class="dm-top">
      <img id="dm-thumb" alt="sent frame">
      <div class="dm-sum">
        <div><span id="dm-obj" class="dm-obj"></span> <span id="dm-cap" class="pill"></span></div>
        <div class="dm-conf"><div id="dm-conf-fill"></div></div>
        <div id="dm-conf-txt" class="muted"></div>
        <div id="dm-task" class="muted"></div>
      </div>
    </div>
    <div id="dm-obs" class="dm-text"></div>
    <ol id="dm-plan"></ol>
    <div id="dm-dropped" class="muted"></div>
    <div id="dm-why" class="dm-text muted"></div>
    <div id="dm-safety" class="dm-safety"></div>
    <div class="dm-row"><button id="dm-run" class="go" disabled title="Run the plan's skills in order (stops at the first blocked/failed step)">Run plan</button></div>
  </div>
  <ol id="tr-list"></ol>
  <div class="dm-row dm-tourbar">
    <button id="dm-tour" title="Guided walk-through of the mechanism">▶ Innovation tour</button>
    <label><input type="checkbox" id="dm-labels"> Labels</label>
    <button id="tr-clear" title="Clear the Gemini call cards">Clear trace</button>
    <span id="dm-budget" class="muted"></span>
  </div>`;

let source = 'sim', ctlState = null, hasCamera = true, analysis = null, busy = false, planRunning = false;

$('dm-src').onclick = (e) => {
  const b = e.target.closest('button');
  if (!b || b.disabled) return;
  source = b.dataset.src;
  for (const x of $('dm-src').children) x.classList.toggle('on', x === b);
};
const msg = (t, cls = '') => { $('dm-msg').textContent = t; $('dm-msg').className = 'muted ' + cls; };

function showBudget(b) {
  if (!b) return;
  const used = b.used_today ?? b.gemini_used_today, cap = b.daily_cap ?? b.gemini_daily_cap;
  if (false && cap) $('dm-budget').textContent = ''   // quota label removed from the UI on request
    + (b.can_call === false && b.why ? ` · ${b.why}` : '');
}

function refreshButtons() {
  const idle = ctlState === 'idle', moving = ['running', 'stopping', 'estopped'].includes(ctlState);
  $('dm-analyze').disabled = busy || moving;      // /api/vision also works without a controller
  if ($('tr-go')) $('tr-go').disabled = busy || !idle;
  $('dm-run').disabled = busy || !idle || !analysis?.plan?.length;
  const real = $('dm-src').querySelector('[data-src=camera]');
  real.disabled = !hasCamera;
  real.title = hasCamera ? 'Server grabs a frame from the real webcam' : 'no camera (server started with --no-camera)';
  if (!hasCamera && source === 'camera') $('dm-src').querySelector('[data-src=sim]').click();
}
function onStatus(st) {
  if (!st) return;
  ctlState = st.state ?? ctlState;
  if (st.has_camera !== undefined) hasCamera = st.has_camera;
  showBudget(st);
  if (ctlState === 'idle') planRunning = false;
  refreshButtons();
}
async function pollStatus() {
  const r = await call('GET', '/api/status');
  if (r.ok) onStatus(r.j); else { ctlState = 'no controller'; refreshButtons(); }
}
pollStatus();
setInterval(pollStatus, 2000);

$('dm-analyze').onclick = async () => {
  busy = true; refreshButtons();
  let image = null;
  try {
    if (source === 'sim') {
      if (!window.twin?.model) { msg('3D model not loaded yet', 'bad'); return; }
      image = await window.twin.topCamJPEG(640, 480);
    }
    msg(source === 'sim' ? 'sending the simulated top-cam frame to Gemini…' : 'server is grabbing a webcam frame for Gemini…');
    const r = await call('POST', '/api/vision', { image, source, goal: $('tr-goal').value.trim() });
    showBudget(r.j?.budget);
    if (r.code === 404) return msg('vision endpoint not available on this server (/api/vision)', 'bad');
    if (r.code === 409) return msg('busy: ' + (r.j?.msg ?? 'controller not idle'), 'bad');
    if (!r.j?.analysis) return msg(r.j?.msg ?? `no analysis (HTTP ${r.code})`, 'bad');
    msg(r.j.msg ?? 'ok', 'good');
    showAnalysis(r.j.analysis, image ?? r.j.image ?? null);
  } finally { busy = false; refreshButtons(); }
};

function showAnalysis(a, image) {
  analysis = a;
  $('dm-out').hidden = false;
  $('dm-thumb').hidden = !image;
  if (image) $('dm-thumb').src = image;
  $('dm-obj').textContent = String(a.object ?? '?').replace(/_/g, ' ');
  $('dm-cap').textContent = 'cap: ' + (a.cap_state ?? '?');
  $('dm-cap').className = 'pill ' + ({ off: 'good', loose: 'warn', on: '' }[a.cap_state] ?? '');
  const c = Math.max(0, Math.min(1, Number(a.confidence) || 0));
  $('dm-conf-fill').style.width = (c * 100) + '%';
  $('dm-conf-fill').style.background = c > 0.7 ? 'var(--good)' : c > 0.4 ? 'var(--warn)' : 'var(--bad)';
  $('dm-conf-txt').textContent = `confidence ${Math.round(c * 100)}%` + (a.model ? ` · ${a.model}` : '') + (a.latency_s ? ` · ${a.latency_s} s` : '');
  $('dm-task').textContent = a.recommended_task && a.recommended_task !== 'none' ? 'task: ' + a.recommended_task.replace(/_/g, ' ') : '';
  const obs = [].concat(a.observations ?? []).join(' ');
  $('dm-obs').innerHTML = (obs ? `<b>Sees:</b> ${esc(obs)}` : '') + (a.grasp_notes ? `<br><b>Grasp:</b> ${esc(a.grasp_notes)}` : '');
  $('dm-plan').innerHTML = (a.plan ?? []).map((s, i) => {
    const ps = Object.entries(s.params ?? {}).map(([k, v]) => `${k}=${+(+v).toFixed(2)}`).join(' ');
    return `<li data-i="${i}" title="${esc(s.why ?? '')}"><span class="n">${i + 1}</span>${esc(s.name)}${ps ? ` <small>${esc(ps)}</small>` : ''}</li>`;
  }).join('') || '<li class="muted">no plan</li>';
  $('dm-dropped').textContent = a.dropped?.length ? 'dropped by validation: ' + a.dropped.join('; ') : '';
  $('dm-why').textContent = a.reasoning ? 'Why: ' + a.reasoning : '';
  $('dm-safety').textContent = a.safety ? '⚠ ' + a.safety : '';
  $('dm-safety').hidden = !a.safety;
  refreshButtons();
}

$('dm-run').onclick = async () => {
  if (!analysis?.plan?.length) return;
  const steps = analysis.plan.map((s) => ({ name: s.name, params: s.params ?? {} }));
  busy = true; refreshButtons(); markPlan(null);
  const r = await call('POST', '/api/plan', { steps });
  busy = false;
  if (r.code === 404) msg('plan endpoint not available on this server (/api/plan)', 'bad');
  else if (!r.ok) msg('plan refused: ' + (r.j?.msg ?? 'HTTP ' + r.code), 'bad');
  else { msg(r.j?.msg ?? 'plan started', 'good'); planRunning = true; markPlan(0, 'run'); }   // plan_start log does the same
  if (r.j?.status) onStatus(r.j.status); else refreshButtons();
};

// highlight plan chips: i = current step index (null clears)
function markPlan(i, cls = 'run') {
  const lis = [...$('dm-plan').querySelectorAll('li[data-i]')];
  if (i == null) { lis.forEach((l) => l.classList.remove('run', 'done', 'fail')); return; }
  lis.forEach((l, k) => {
    if (k < i) { l.classList.remove('run', 'fail'); l.classList.add('done'); }
    else if (k === i) { l.classList.remove('run', 'done', 'fail'); l.classList.add(cls); }
  });
}

function onLog(e) {
  if (e.kind === 'status') return onStatus(e);
  if (e.kind === 'plan_step') {     // controller.run_plan: i (1-based), n, name, blocked | result
    const i = (e.i ?? 1) - 1, failed = !!e.blocked || e.result?.ok === false;
    planRunning = true;
    markPlan(i, failed ? 'fail' : 'done');
    if (!failed) markPlan(i + 1, 'run');
    msg(`plan step ${i + 1}/${e.n ?? '?'} ${e.name ?? ''}: ${failed ? (e.blocked || e.result?.problem || 'failed') : 'done'}`, failed ? 'bad' : '');
  } else if (e.kind === 'plan_start') {
    planRunning = true; markPlan(null); markPlan(0, 'run');
  } else if (e.kind === 'plan') {
    planRunning = false;
    $('dm-plan').querySelectorAll('li.run').forEach((l) => l.classList.remove('run'));
    msg(`plan ${e.ok ? 'finished' : 'stopped'}: ${e.outcome ?? ''} (${e.completed ?? 0}/${e.steps?.length ?? '?'} steps)`, e.ok ? 'good' : 'bad');
  }
}

// ================= 2. Innovation tour + labels =================

function initTour(twin) {
  const { THREE, camera, controls, root, renderer } = twin;
  const stage = $('stage');
  const mm = (x, y, z) => new THREE.Vector3(x / 1000, y / 1000, z / 1000);   // CAD mm -> root-local metres
  const toWorld = (v) => root.localToWorld(v.clone());

  // ---- overlay layer: callouts + caption card ----
  const layer = document.createElement('div');
  layer.id = 'dm-layer';
  stage.append(layer);
  const card = document.createElement('div');
  card.id = 'dm-card'; card.hidden = true;
  stage.append(card);

  // ---- DOF arrows (CAD frame, metres) ----
  const arrows = new THREE.Group();     // layer 1: main view only, never in the top-cam frame sent to Gemini
  arrows.visible = false;
  camera.layers.enable(1);
  root.add(arrows);
  const COL = { squeeze: 0x2f6fde, twist: 0xe07a1f, lift: 0x17924f };
  function arrow(name, from, dir, len, col) {
    const a = new THREE.ArrowHelper(dir.clone().normalize(), from, len, col, len * 0.35, len * 0.22);
    a.userData.step = name;
    a.traverse((o) => { if (o.material) { o.material.depthTest = false; o.material.transparent = true; } o.renderOrder = 10; });
    arrows.add(a);
    return a;
  }
  // (a) bottom squeeze (servo 0): both bottom fingers close along Y
  arrow('a', mm(131, -48, 0), mm(0, 1, 0), 0.016, COL.squeeze);
  arrow('a', mm(131, 48, 0), mm(0, -1, 0), 0.016, COL.squeeze);
  // (b) top squeeze (servo 5)
  arrow('b', mm(131, -48, 32), mm(0, 1, 0), 0.016, COL.squeeze);
  arrow('b', mm(131, 48, 32), mm(0, -1, 0), 0.016, COL.squeeze);
  // (c) twist: right top tip forward (+X, servo 4), left top tip back (-X, servo 6) -> CCW about +Z
  arrow('c', mm(70, -30, 52), mm(1, 0, 0), 0.03, COL.twist);
  arrow('c', mm(100, 30, 52), mm(-1, 0, 0), 0.03, COL.twist);
  {   // curved arrow around the cap axis, counter-clockwise seen from above (= loosening)
    const r = 0.019, c = mm(85, 0, 56), pts = [];
    for (let k = 0; k <= 32; k++) { const t = -Math.PI / 2 + k / 32 * Math.PI * 1.4; pts.push(new THREE.Vector3(c.x + r * Math.cos(t), c.y + r * Math.sin(t), c.z)); }
    const curve = new THREE.CatmullRomCurve3(pts);
    const tube = new THREE.Mesh(new THREE.TubeGeometry(curve, 48, 0.0009, 6), new THREE.MeshBasicMaterial({ color: COL.twist, depthTest: false, transparent: true }));
    const tip = pts[pts.length - 1], tan = curve.getTangent(1);
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.0025, 0.006, 12), tube.material);
    cone.position.copy(tip); cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), tan);
    for (const o of [tube, cone]) { o.renderOrder = 10; o.userData.step = 'c'; arrows.add(o); }
  }
  // (d) lift (servo 2): whole top layer along +Z
  arrow('d', mm(20, -50, 30), mm(0, 0, 1), 0.026, COL.lift);

  // the FlexiTac pad heatmap itself is drawn by objects.js; here we only point at it
  function cadBox(node) {   // node's current AABB in the CAD frame (root is an axis-aligned rotation)
    const wb = new THREE.Box3().setFromObject(node);
    return new THREE.Box3().setFromPoints([wb.min, wb.max].map((p) => root.worldToLocal(p.clone())));
  }
  const tip3 = twin.findNode(twin.model, 'finger tip:3');

  // ---- callouts: HTML labels pinned to a node (live bbox centre) or a fixed CAD point ----
  // fixed CAD points (mm) next to the arrows, so labels don't pile up on the big parent nodes
  const C = [
    { step: 'a', at: [131, -50, -4], text: 'Bottom squeeze · servo 0 (Y)' },
    { step: 'b', at: [131, -50, 34], text: 'Top squeeze · servo 5 (Y)' },
    { step: 'c', at: [100, -30, 52], text: 'Top R slide · servo 4 → +X' },
    { step: 'c', at: [70, 30, 52], text: 'Top L slide · servo 6 → −X' },
    { step: 'c', at: [85, 0, 60], text: 'cap turns CCW (loosen)' },
    { step: 'd', at: [20, -50, 56], text: 'Lift · servo 2 (Z, 24 mm)' },
    { step: 'e', pad: true, text: 'FlexiTac 16×32 tactile pad' },
    { step: 'e', at: [131, 48, 0], text: 'INA219 current = grip force' },
    { step: 'f', cam: true, text: 'Top webcam → Gemini' },
  ];
  for (const c of C) {
    c.el = document.createElement('div');
    c.el.className = 'dm-callout';
    c.el.textContent = c.text;
    layer.append(c.el);
  }
  const tmp = new THREE.Vector3();
  function anchor(c) {
    if (c.cam) return tmp.copy(twin.topCam.position);
    if (c.pad) {   // live: inner (+Y) face of the top-right tip, wherever the servos put it
      if (!tip3) return null;
      const b = cadBox(tip3), m = b.getCenter(tmp);
      return root.localToWorld(m.set(m.x, b.max.y, m.z));
    }
    return root.localToWorld(tmp.copy(mm(...c.at)));
  }

  // ---- what is visible: 'all' (Labels toggle), a tour step letter, or null ----
  let show = null;
  function setShow(s) {
    show = s;
    arrows.visible = !!s;
    arrows.children.forEach((o) => { o.visible = s === 'all' || o.userData.step === s; o.traverse((x) => x.layers.set(1)); });
  }
  twin.onFrame.push(() => {
    const w = stage.clientWidth, h = stage.clientHeight;
    for (const c of C) {
      const on = show === 'all' || show === c.step;
      const p = on && anchor(c);
      if (!p) { c.el.style.display = 'none'; continue; }
      p.project(camera);
      if (p.z > 1 || Math.abs(p.x) > 1.1 || Math.abs(p.y) > 1.1) { c.el.style.display = 'none'; continue; }
      c.el.style.display = '';
      c.el.style.transform = `translate(${(p.x + 1) / 2 * w}px, ${(1 - p.y) / 2 * h}px)`;
    }
    tween?.(performance.now());
  });

  // ---- smooth camera moves ----
  let tween = null;
  function flyTo(posMM, tgtMM, zoom = 1.6, ms = 1300) {   // zoom: scale the viewpoint's distance
    const p0 = camera.position.clone(), t0 = controls.target.clone();
    const tg = mm(...tgtMM), p1 = toWorld(mm(...posMM).sub(tg).multiplyScalar(zoom).add(tg)), t1 = toWorld(tg);
    const start = performance.now();
    tween = (now) => {
      const k = Math.min(1, (now - start) / ms), e = k < 0.5 ? 4 * k * k * k : 1 - Math.pow(-2 * k + 2, 3) / 2;
      camera.position.lerpVectors(p0, p1, e);
      controls.target.lerpVectors(t0, t1, e);
      if (k >= 1) tween = null;
    };
  }

  // ---- skills: run only those that exist and aren't blocked; wait until the controller is idle ----
  let tourToken = 0;
  async function runSkills(names, token) {
    for (const name of names) {
      if (token !== tourToken) return;
      const r = await call('GET', '/api/skills');
      if (!r.ok || !Array.isArray(r.j)) return;
      const s = r.j.find((x) => x.name === name);
      if (!s || s.blocked) continue;               // missing / not possible right now: skip quietly
      const st = await call('GET', '/api/status');
      if (st.j?.state !== 'idle') return;
      await twin.api('skill', { name, params: {} });
      for (let k = 0; k < 120 && token === tourToken; k++) {      // up to ~30 s per skill
        await sleep(250);
        const q = await call('GET', '/api/status');
        if (!q.ok || q.j?.state !== 'running') break;
      }
    }
  }

  // ---- tour steps ----
  const STEPS = [
    { key: 'a', title: 'Bottom squeeze holds the bottle', servos: 'servo 0 · Y axis',
      body: 'One rack-and-pinion closes both bottom fingers symmetrically on the bottle body. The INA219 current on this servo tells us when we touch and how hard we hold.',
      cam: [250, -160, 90], tgt: [85, 0, 5], skills: ['home', 'grip_bottle'] },
    { key: 'b', title: 'Top squeeze grips the cap', servos: 'servo 5 · Y axis',
      body: 'A second, independent squeeze one layer up closes on the cap. The bottle and the cap are held separately, so they can move relative to each other.',
      cam: [250, -150, 150], tgt: [85, 0, 32], skills: ['grip_cap'] },
    { key: 'c', title: 'The twist: linear in, rotation out', servos: 'servos 4 + 6 · opposite X',
      body: 'The two top finger tips slide in opposite directions, like rubbing a pencil between your palms. A cap of radius r turns d/r radians for a slide of d. No wrist joint and no rotary actuator: every motion in the hand is the same cheap rack and pinion.',
      cam: [150, -70, 270], tgt: [85, 0, 40], skills: ['twist'] },
    { key: 'd', title: 'Lift axis', servos: 'servo 2 · Z axis',
      body: 'The whole top layer rises up to 24 mm. That lifts a loosened cap clear to check it is off, and sets how high the top fingers sit on the object.',
      cam: [70, -320, 70], tgt: [45, 0, 30], skills: ['lift_test', 'lower'] },
    { key: 'e', title: 'Feeling the grip', servos: 'FlexiTac 16×32 + 2× INA219',
      body: 'A FlexiTac pressure pad (512 taxels) sits under the grip tape on the top-right finger, so the hand sees contact area, slip and torque on the cap. Squeeze servos are force-controlled from their current draw: close until contact, then hold a target current.',
      cam: [95, 75, 150], tgt: [82, -22, 32], zoom: 2.4, skills: [] },
    { key: 'f', title: 'Camera → Gemini picks skills', servos: 'top webcam · vision planner',
      body: 'A webcam on the backplate looks down at the work area. One frame goes to Gemini, which names the object, judges the cap state and returns a short plan made only of named, validated skills. It never sends raw servo angles. Try "Plan it" or "Let Gemini drive" in the side panel.',
      cam: [-140, -110, 280], tgt: [85, 0, 10], skills: [] },
    { key: 'g', title: 'More than pill bottles', servos: 'same hand, new skills',
      body: 'Pill tube / child-proof bottle: squeeze + twist + lift. Eye dropper: held inverted, the top squeeze presses out metered drops. Same 7 linear axes and sensors.',
      cam: [320, -260, 210], tgt: [60, 0, 40], zoom: 1.3, skills: [], objects: true },
  ];
  let idx = -1, saved = null;

  async function renderCard() {
    const s = STEPS[idx];
    card.hidden = false;
    card.innerHTML = `<div class="dm-card-head"><span class="muted">${idx + 1} / ${STEPS.length} · ${esc(s.servos)}</span>
        <button class="x" title="Exit tour">✕</button></div>
      <h3>${esc(s.title)}</h3><p>${esc(s.body)}</p><div class="dm-objs"></div>
      <div class="dm-card-nav"><button class="prev" ${idx ? '' : 'disabled'}>◀ Prev</button>
        <button class="next go">${idx === STEPS.length - 1 ? 'Finish' : 'Next ▶'}</button></div>`;
    card.querySelector('.x').onclick = endTour;
    card.querySelector('.prev').onclick = () => go(idx - 1);
    card.querySelector('.next').onclick = () => (idx === STEPS.length - 1 ? endTour() : go(idx + 1));
    if (s.objects) {   // sim object switcher, if the server runs SimRobot
      const r = await call('GET', '/api/sim');
      const objs = r.ok ? (r.j?.objects ?? []) : [];
      const list = Array.isArray(objs) ? objs : Object.keys(objs);
      const wrap = card.querySelector('.dm-objs');
      for (const o of list) {
        const name = typeof o === 'string' ? o : o.name, label = typeof o === 'string' ? o.replace(/_/g, ' ') : (o.label ?? o.name);
        const b = document.createElement('button');
        b.textContent = label;
        b.onclick = async () => {
          const q = await call('POST', '/api/sim/object', { name });
          b.title = q.j?.msg ?? '';
          wrap.querySelectorAll('button').forEach((x) => x.classList.toggle('on', x === b && q.ok));
        };
        wrap.append(b);
      }
    }
  }
  function go(i) {
    idx = Math.max(0, Math.min(STEPS.length - 1, i));
    const s = STEPS[idx], token = ++tourToken;
    setShow(s.key);
    flyTo(s.cam, s.tgt, s.zoom);
    renderCard();
    if (s.skills.length) sleep(900).then(() => runSkills(s.skills, token));
  }
  function startTour() {
    saved = { p: camera.position.clone(), t: controls.target.clone() };
    $('dm-labels').checked = false;
    go(0);
    $('dm-tour').textContent = '■ End tour';
  }
  function endTour() {
    tourToken++; idx = -1; card.hidden = true;
    setShow($('dm-labels').checked ? 'all' : null);
    if (saved) {
      const p0 = camera.position.clone(), t0 = controls.target.clone(), start = performance.now(), sv = saved;
      tween = (now) => {
        const k = Math.min(1, (now - start) / 1000), e = 1 - Math.pow(1 - k, 3);
        camera.position.lerpVectors(p0, sv.p, e); controls.target.lerpVectors(t0, sv.t, e);
        if (k >= 1) tween = null;
      };
    }
    $('dm-tour').textContent = '▶ Innovation tour';
  }
  $('dm-tour').disabled = false;
  $('dm-tour').onclick = () => (idx < 0 ? startTour() : endTour());
  $('dm-labels').disabled = false;
  $('dm-labels').onchange = () => { if (idx < 0) setShow($('dm-labels').checked ? 'all' : null); };
  window.twinDemo = { go, startTour, endTour, setShow };   // handy from the console
  const q = new URLSearchParams(location.search);       // ?tour=3 opens the tour at step 3, ?labels shows labels
  if (q.has('labels')) { $('dm-labels').checked = true; setShow('all'); }
  if (q.get('tour')) { startTour(); go(+q.get('tour') - 1); }
}

$('dm-tour').disabled = $('dm-labels').disabled = true;   // until the 3D model is in
function boot() {
  const twin = window.twin;
  twin.onLog.push(onLog);
  if (twin.model) initTour(twin); else addEventListener('twin-ready', () => initTour(twin), { once: true });
}
if (window.twin) boot();
else { const t = setInterval(() => { if (window.twin) { clearInterval(t); boot(); } }, 100); }
