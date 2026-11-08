// Digital twin: loads the CAD (assets/gripper.glb), moves parts from live servo angles
// (kinematics.json), and shows currents, tactile and agent events streamed by src/twin_server.py.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const SERVO_SPEED = 500;          // deg/s for display smoothing (SG90 ~600 deg/s unloaded)
const OVERRIDES_KEY = 'twin-overrides-v1';

let kinFile, kin;                 // kinematics as in the file / with local overrides applied
let joints = [];                  // {servo, node, base, perMeter}
let tints = [];                   // {ch, mats: [{mat, base}]}
let target = [0, 0, 0, 0, 0, 0, 0];
let shown = [0, 0, 0, 0, 0, 0, 0];
let latest = null;
let limp = [true, true, true, true, true, true, true];
// mm per (calibrated) degree per servo, from calibration.json: after calibrate.py, 0-180 deg spans
// that servo's own HOME..FAR pulse range, i.e. span_us of the raw 500-2400 us (= 180 raw deg) window.
const travel = {};
const RAW_US_PER_DEG = 1900 / 180;
async function loadTravel() {
  try {
    const cal = await (await fetch('/api/calibration')).json();
    for (const [i, c] of Object.entries(cal.servos || {})) {
      const rawDeg = Math.abs(c.far_us - c.home_us) / RAW_US_PER_DEG;
      travel[i] = rawDeg * kin.mm_per_deg / 180;
    }
  } catch {}
}
const hist = [];                  // [t, i0, i1]
// ?pose=a0,a1,...,a6 freezes the twin at fixed angles (checking axes without hardware)
const POSE = new URLSearchParams(location.search).get('pose')?.split(',').map(Number) ?? null;
if (POSE) { target = POSE.slice(0, 7); shown = [...target]; }

// ---------- kinematics + overrides ----------

function loadOverrides() {
  try { return JSON.parse(localStorage.getItem(OVERRIDES_KEY) || '{}'); } catch { return {}; }
}
function saveOverrides(o) {
  try { localStorage.setItem(OVERRIDES_KEY, JSON.stringify(o)); } catch {}
}
function applyOverrides() {
  kin = structuredClone(kinFile);
  for (const [s, v] of Object.entries(loadOverrides())) if (kin.servos[s]) Object.assign(kin.servos[s], v);
}

// ---------- three.js scene ----------

const renderer = new THREE.WebGLRenderer({ canvas: $('view'), antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1216);
const camera = new THREE.PerspectiveCamera(35, 1, 0.001, 20);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 2.2);
sun.position.set(0.4, 0.8, 0.6);
scene.add(sun);
const grid = new THREE.GridHelper(0.4, 20, 0x2a3039, 0x1d2229);
scene.add(grid);

const root = new THREE.Group();
root.rotation.x = -Math.PI / 2;   // CAD is Z-up, three.js is Y-up
scene.add(root);

function resize() {
  const c = renderer.domElement, w = c.clientWidth, h = c.clientHeight;
  if (c.width !== Math.floor(w * renderer.getPixelRatio()) || c.height !== Math.floor(h * renderer.getPixelRatio())) {
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
}

const sanitize = (n) => n.replace(/\s/g, '_').replace(/[\[\]\.:\/]/g, '');
function findNode(model, name) {
  let hit = null;
  model.traverse((o) => { if (!hit && (o.userData.name === name || o.name === sanitize(name))) hit = o; });
  return hit;
}

function buildJoints(model) {
  joints = [];
  root.updateMatrixWorld(true);
  for (const [s, cfg] of Object.entries(kin.servos)) {
    for (const [name, axis] of cfg.parts) {
      const node = findNode(model, name);
      if (!node) { console.warn('twin: part not found', name); continue; }
      // CAD-frame axis -> world -> the node's parent frame, per metre of travel
      const wo = node.getWorldPosition(new THREE.Vector3());
      const dir = new THREE.Vector3(...axis).normalize().applyQuaternion(root.quaternion);
      const p0 = node.parent.worldToLocal(wo.clone());
      const p1 = node.parent.worldToLocal(wo.clone().addScaledVector(dir, 1));
      joints.push({ servo: +s, node, base: node.position.clone(), perMeter: p1.sub(p0) });
    }
  }
}

function buildTints(model) {
  for (const [ch, names] of Object.entries(kin.tint_by_current || {})) {
    const mats = [];
    for (const name of names) {
      const node = findNode(model, name);
      node?.traverse((o) => {
        if (o.isMesh) {
          o.material = o.material.clone();      // don't recolor shared materials
          mats.push({ mat: o.material, base: o.material.color.clone() });
        }
      });
    }
    tints.push({ ch: +ch, mats });
  }
}

function frameModel(model) {
  const box = new THREE.Box3().setFromObject(root);
  const c = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3()).length();
  grid.position.y = box.min.y;
  controls.target.copy(c);
  camera.position.copy(c).add(new THREE.Vector3(size * 1.15, size * 0.7, size * 1.25));
  camera.near = size / 100; camera.far = size * 20; camera.updateProjectionMatrix();
}

function applyPose() {
  for (const j of joints) {
    const cfg = kin.servos[j.servo];
    const mm = cfg.sign * (shown[j.servo] - cfg.zero_deg) * (travel[j.servo] ?? kin.mm_per_deg);
    j.node.position.copy(j.base).addScaledVector(j.perMeter, mm / 1000);
  }
}

const RED = new THREE.Color(0xff3b30);
function applyTints() {
  if (!latest) return;
  for (const t of tints) {
    const ma = latest.current_ma[t.ch];
    const k = ma == null ? 0 : Math.max(0, Math.min(1, (ma - 15) / 300));   // ~idle -> 0, hard squeeze -> 1
    for (const m of t.mats) m.mat.color.copy(m.base).lerp(RED, k);
  }
}

let lastFrame = performance.now();
function animate(now) {
  const dt = Math.min(0.1, (now - lastFrame) / 1000); lastFrame = now;
  for (let i = 0; i < 7; i++) {
    const d = target[i] - shown[i], step = SERVO_SPEED * dt;
    shown[i] += Math.abs(d) <= step ? d : Math.sign(d) * step;
  }
  resize();
  applyPose();
  applyTints();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

// ---------- side panels ----------

function buildServoRows() {
  $('servos').innerHTML = '';
  for (let i = 0; i < 7; i++) {
    const row = document.createElement('div');
    row.className = 'servo';
    row.innerHTML = `<span class="idx">${i}</span><div class="bar"><div class="fill" id="f${i}"></div>
      <span class="lbl">${kin.servos[i]?.name ?? ''}</span></div><span class="val" id="v${i}">—</span>`;
    $('servos').append(row);
  }
}
function updateServoRows() {
  for (let i = 0; i < 7; i++) {
    const row = $('f' + i).closest('.servo');
    row.classList.toggle('limp', limp[i] && !POSE);
    $('f' + i).style.width = (target[i] / 180 * 100) + '%';
    $('v' + i).textContent = limp[i] && !POSE ? 'limp' : target[i] + '°';
  }
}

function drawPlot() {
  const c = $('plot'), g = c.getContext('2d');
  const w = c.width = c.clientWidth * devicePixelRatio, h = c.height = 120 * devicePixelRatio;
  g.clearRect(0, 0, w, h);
  if (hist.length < 2) return;
  const tEnd = hist[hist.length - 1][0], span = 10;
  let max = 200;
  for (const [, a, b] of hist) max = Math.max(max, a ?? 0, b ?? 0);
  g.lineWidth = 1.5 * devicePixelRatio;
  const css = getComputedStyle(document.documentElement);
  [[1, css.getPropertyValue('--i0')], [2, css.getPropertyValue('--i1')]].forEach(([k, col]) => {
    g.strokeStyle = col.trim(); g.beginPath();
    let started = false;
    for (const r of hist) {
      if (r[k] == null) continue;
      const x = w - (tEnd - r[0]) / span * w, y = h - r[k] / max * (h - 4) - 2;
      started ? g.lineTo(x, y) : (g.moveTo(x, y), started = true);
    }
    g.stroke();
  });
  g.fillStyle = '#8b95a3'; g.font = `${10 * devicePixelRatio}px system-ui`;
  g.fillText(`${Math.round(max)} mA`, 4, 12 * devicePixelRatio);
  const last = hist[hist.length - 1];
  g.fillText(`i0 ${last[1]?.toFixed(1) ?? '—'}  i1 ${last[2]?.toFixed(1) ?? '—'}`, w - 130 * devicePixelRatio, 12 * devicePixelRatio);
}

function drawHeat(hex) {
  const c = $('heat'), g = c.getContext('2d');
  const cw = c.width / 32, ch = c.height / 16;
  for (let k = 0; k < 512; k++) {
    const v = parseInt(hex.substr(k * 2, 2), 16) / 255;
    g.fillStyle = `hsl(${240 - v * 240}, 85%, ${12 + v * 48}%)`;
    g.fillRect((k % 32) * cw, Math.floor(k / 32) * ch, cw + 0.5, ch + 0.5);
  }
}

function fmtEvent(e) {
  const k = `<span class="k">${e.kind}</span>`;
  switch (e.kind) {
    case 'decision': return `${k}${e.action} · ${e.cap_state} · ${e.reasoning ?? ''}`;
    case 'lift_test':
      if (e.result) return `${k}cycle ${e.cycle}: ${e.result}`;
      return `${k}<span class="${e.cap_off ? 'ok' : 'no'}">${e.cap_off ? 'CAP OFF' : 'still on'}</span> (${Math.round((e.confidence ?? 0) * 100)}%) ${e.observation ?? ''}`;
    case 'cycle': {
      const s = e.summary || {}, g = s.grip || {}, d = s.grip_during_twist || {};
      return `${k}#${e.cycle} ${s.ok ? 'ok' : '<span class="no">' + (s.problem || 'fail') + '</span>'} · grip ${g.hold_ma ?? '—'} mA @${g.angle ?? '—'}° · twist ${d.min_ma ?? '—'}–${d.max_ma ?? '—'} mA · ${s.duration_s ?? '—'} s`;
    }
    case 'grip_bottle': return `${k}${e.contact ? 'holding' : '<span class="no">missed</span>'} ${e.hold_ma ?? ''} mA @${e.angle}°`;
    case 'stop': return `${k}${e.reason}${e.observation ? ' · ' + e.observation : ''}`;
    case 'estop': return `<span class="k no">E-STOP</span>${e.result}`;
    case 'run_start': return `${k}${e.gemini ? 'Gemini ' + e.model : 'rules (no API)'} · up to ${e.cycles} cycles`;
    case 'reset': return `${k}${e.note}`;
    default: {
      const { t, kind, ...rest } = e;
      return `${k}${JSON.stringify(rest).slice(0, 160)}`;
    }
  }
}
function addLog(e) {
  const li = document.createElement('li');
  li.innerHTML = `<small>${new Date(e.t * 1000).toLocaleTimeString()}</small> ${fmtEvent(e)}`;
  $('log').prepend(li);
  while ($('log').children.length > 60) $('log').lastChild.remove();
}

function buildCalib() {
  const rows = $('calib-rows');
  rows.innerHTML = '';
  for (let i = 0; i < 7; i++) {
    const cfg = kin.servos[i];
    const r = document.createElement('div');
    r.className = 'crow';
    r.innerHTML = `<span>${i} ${cfg.name}</span><button data-s="${i}" class="sign">${cfg.sign > 0 ? '+' : '−'}</button>
      <input type="number" step="1" min="0" max="180" value="${cfg.zero_deg}" data-s="${i}" class="zero" title="zero">`;
    rows.append(r);
  }
  rows.querySelectorAll('.sign').forEach((b) => b.onclick = () => {
    const s = b.dataset.s, o = loadOverrides();
    o[s] = { ...(o[s] || {}), sign: -kin.servos[s].sign };
    saveOverrides(o); applyOverrides(); buildCalib();
  });
  rows.querySelectorAll('.zero').forEach((inp) => inp.onchange = () => {
    const s = inp.dataset.s, o = loadOverrides();
    o[s] = { ...(o[s] || {}), zero_deg: Number(inp.value) || 0 };
    saveOverrides(o); applyOverrides();
  });
}
$('copy').onclick = async () => {
  const text = JSON.stringify(kin, null, 1);
  try { await navigator.clipboard.writeText(text); $('copy').textContent = 'Copied'; }
  catch { console.log(text); $('copy').textContent = 'See console'; }
  setTimeout(() => $('copy').textContent = 'Copy JSON', 1500);
};
$('reset').onclick = () => { saveOverrides({}); applyOverrides(); buildCalib(); };

// ---------- live data ----------

let count = 0, rateT = performance.now();
function connect() {
  const es = new EventSource('/state');
  es.onopen = () => { $('conn').textContent = 'live'; $('conn').className = 'pill good'; };
  es.onerror = () => { $('conn').textContent = 'disconnected'; $('conn').className = 'pill bad'; };
  es.addEventListener('state', (ev) => {
    latest = JSON.parse(ev.data);
    if (!POSE) latest.angles.forEach((a, i) => { limp[i] = a < 0; if (a >= 0) target[i] = a; });
    hist.push([latest.t, latest.current_ma[0], latest.current_ma[1]]);
    while (hist.length && latest.t - hist[0][0] > 10) hist.shift();
    count++;
  });
  es.addEventListener('log', (ev) => {
    const e = JSON.parse(ev.data);
    if (e.kind === 'status') showStatus(e); else addLog(e);
  });
}

setInterval(() => {   // panels at 10 Hz; the 3D view runs at display rate
  if (!latest) return;
  updateServoRows();
  drawPlot();
  drawHeat(latest.tactile);
  $('tact').textContent = latest.ok ? 'tactile ok' : 'tactile stale';
  $('tact').className = 'pill ' + (latest.ok ? 'good' : 'bad');
  const now = performance.now();
  if (now - rateT > 1000) { $('hz').textContent = Math.round(count * 1000 / (now - rateT)) + ' Hz'; count = 0; rateT = now; }
}, 100);

// webcam overlay only if the server has a camera
const cam = $('cam');
cam.onload = () => { $('cam-wrap').hidden = false; };
cam.src = '/cam.mjpg';

// ---------- run control (main.py's Controller via /api/*) ----------

let ctlState = null;
function showStatus(st) {
  ctlState = st.state;
  const pill = $('run-state');
  pill.textContent = { idle: 'ready', running: 'running', stopping: 'stopping…', estopped: 'E-STOPPED' }[st.state] ?? st.state;
  pill.className = 'pill ' + ({ running: 'run', stopping: 'stopping', estopped: 'estop', idle: 'good' }[st.state] ?? '');
  const has = st.state && st.state !== 'no controller';
  $('start').disabled = !has || st.state !== 'idle';
  $('stop').disabled = !has || st.state !== 'running';
  $('estop').disabled = !has;
  $('reset-estop').hidden = st.state !== 'estopped';
  if (st.gemini_daily_cap) $('budget').textContent = `(${st.gemini_used_today}/${st.gemini_daily_cap} today)`;
  if (st.has_camera === false) { $('opt-gemini').checked = false; $('opt-gemini').disabled = true; }
  $('run-info').textContent = st.state === 'running' ? `${st.gemini ? 'Gemini' : 'rules'} · ${st.cycles} cycles max` : '';
}
async function api(action, body = {}) {
  try {
    const r = await fetch('/api/' + action, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const j = await r.json();
    $('ctl-msg').textContent = j.msg ?? '';
    if (j.status) showStatus(j.status);
    return j;
  } catch (e) {
    $('ctl-msg').textContent = 'server unreachable: ' + e;
  }
}
$('start').onclick = () => api('start', { gemini: $('opt-gemini').checked, cycles: Number($('opt-cycles').value) || 20 });
$('stop').onclick = () => api('stop');
$('estop').onclick = () => api('estop');
$('reset-estop').onclick = () => api('reset');
addEventListener('keydown', (e) => { if (e.key === 'Escape') { e.preventDefault(); api('estop'); } });
fetch('/api/status').then((r) => r.ok ? r.json() : { state: 'no controller' }).then(showStatus)
  .catch(() => showStatus({ state: 'no controller' }));

// ---------- boot ----------

kinFile = await (await fetch('kinematics.json')).json();
applyOverrides();
await loadTravel();
buildServoRows();
buildCalib();
connect();
new GLTFLoader().load('assets/gripper.glb', (gltf) => {
  root.add(gltf.scene);
  buildJoints(gltf.scene);
  buildTints(gltf.scene);
  frameModel(gltf.scene);
}, undefined, (err) => console.error('twin: failed to load model', err));
requestAnimationFrame(animate);
