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
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xeef1f5);
const camera = new THREE.PerspectiveCamera(35, 1, 0.001, 20);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
// studio lighting: soft sky/ground fill + warm key (casts shadow) + cool fill + rim
scene.add(new THREE.HemisphereLight(0xffffff, 0xb8c0cc, 1.4));
const sun = new THREE.DirectionalLight(0xfff6ea, 2.6);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.bias = -0.0005;
sun.shadow.radius = 4;
scene.add(sun, sun.target);
const fill = new THREE.DirectionalLight(0xdfe8ff, 0.9);
fill.position.set(-0.6, 0.3, -0.2);
scene.add(fill);
const rim = new THREE.DirectionalLight(0xffffff, 0.8);
rim.position.set(-0.2, 0.5, -0.8);
scene.add(rim);
const ground = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), new THREE.ShadowMaterial({ opacity: 0.18 }));
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(0.4, 20, 0xc3cad4, 0xdde2e8);
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

// The STEP export leaves every material at glTF's default metallic = 1, which renders near-black
// without an environment map (the "murky" look). Make them plastic: PLA prints + servos.
function fixMaterials(model) {
  const cache = new Map();
  model.traverse((o) => {
    if (!o.isMesh) return;
    o.castShadow = o.receiveShadow = true;
    const src = o.material;
    if (!cache.has(src)) {
      const m = src.clone();
      m.metalness = 0.0;
      m.roughness = 0.6;
      m.polygonOffset = true; m.polygonOffsetFactor = 1; m.polygonOffsetUnits = 1;   // outlines win the depth tie
      const hsl = {}; m.color.getHSL(hsl);   // lift pure black so shading still reads
      if (hsl.l < 0.06) m.color.setHSL(hsl.h, hsl.s, 0.06);
      cache.set(src, m);
    }
    o.material = cache.get(src);
    addOutline(o);
  });
}

// Thin outline on each part's hard edges (faces meeting at > 30 deg), as a child of the mesh so it
// moves with it. One near-black line for every part: lit surfaces are brighter than their base
// colour, so a per-part "slightly darker" line disappears. Meshes use polygonOffset so lines win depth.
const outlineMat = new THREE.LineBasicMaterial({ color: 0x10151c, transparent: true, opacity: 0.55, toneMapped: false });
function addOutline(mesh) {
  const lines = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 30), outlineMat);
  lines.raycast = () => {};
  mesh.add(lines);
}

function frameModel(model) {
  const box = new THREE.Box3().setFromObject(root);
  // sim objects (objects.js) hang below the fingers down to CAD z -118 (insulin pen): keep them above the floor
  root.updateMatrixWorld(true);
  box.expandByPoint(root.localToWorld(new THREE.Vector3(0.085, 0, -0.125)));
  const c = box.getCenter(new THREE.Vector3()), size = box.getSize(new THREE.Vector3()).length();
  grid.position.y = ground.position.y = box.min.y - 0.0005;
  // shadow camera tight around the model, so the shadow stays sharp
  const r = size * 0.75, sc = sun.shadow.camera;
  sc.left = sc.bottom = -r; sc.right = sc.top = r; sc.near = 0.001; sc.far = size * 6;
  sun.target.position.copy(c);
  sun.position.copy(c).add(new THREE.Vector3(0.35, 0.8, 0.45).normalize().multiplyScalar(size * 2));
  sc.updateProjectionMatrix();
  controls.target.copy(c);
  // mostly from the finger tips (+X), a little to the right and above: you see into the gripper at the object
  camera.position.copy(c).add(new THREE.Vector3(size * 1.45, size * 0.45, size * 0.8));
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

// ---------- simulated top webcam ----------
// Mounted at the top of the tall backplate, looking down at the work area between the squeeze
// fingers. Rendered into the inset at the bottom-left of the 3D view (#topcam-wrap).
const TOPCAM = {
  plate: 'Backplate + Wiring & Camera Mount:1',
  look_at: ['Bottom Left:1', 'Bottom Right:1', 'Top Left:1', 'Top Right:1'],   // centre of these
  vfov: 45,                 // deg, typical USB webcam (~60 deg horizontal at 4:3)
  forward_mm: 15,           // in front of the plate face, toward the work area
  up_mm: 10,                // above the plate's top edge
};
const topCam = new THREE.PerspectiveCamera(TOPCAM.vfov, 4 / 3, 0.005, 5);
let topCamReady = false;
function placeTopCam(model) {
  const plate = findNode(model, TOPCAM.plate);
  if (!plate) { console.warn('twin: no backplate for top camera'); return; }
  root.updateMatrixWorld(true);
  const pb = new THREE.Box3().setFromObject(plate);
  const work = new THREE.Box3();
  for (const n of TOPCAM.look_at) { const o = findNode(model, n); if (o) work.union(new THREE.Box3().setFromObject(o)); }
  if (work.isEmpty()) return;
  const target = work.getCenter(new THREE.Vector3());
  const pc = pb.getCenter(new THREE.Vector3());
  const toward = new THREE.Vector3(target.x - pc.x, 0, target.z - pc.z).normalize();   // horizontal
  const pbs = pb.getSize(new THREE.Vector3());
  const half = Math.abs(toward.x) > Math.abs(toward.z) ? pbs.x / 2 : pbs.z / 2;       // to the plate face
  topCam.position.set(pc.x, pb.max.y + TOPCAM.up_mm / 1000, pc.z)
    .addScaledVector(toward, half + TOPCAM.forward_mm / 1000);
  topCam.lookAt(target);
  // small dark block where the webcam would be
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.02, 0.02), new THREE.MeshStandardMaterial({ color: 0x222831, roughness: 0.5 }));
  body.position.copy(topCam.position); body.quaternion.copy(topCam.quaternion);
  body.translateZ(0.012);                 // just behind the lens, so it's outside its own view
  scene.add(body);
  topCamReady = true;
}
function renderTopCam() {
  const wrap = $('topcam-wrap');
  if (!topCamReady || wrap.hidden) return;
  const c = renderer.domElement, r = wrap.getBoundingClientRect(), cr = c.getBoundingClientRect();
  const x = r.left - cr.left + 1, w = r.width - 2, h = r.height - 2, y = cr.bottom - r.bottom + 1;
  topCam.aspect = w / h; topCam.updateProjectionMatrix();
  renderer.setScissorTest(true);
  renderer.setScissor(x, y, w, h); renderer.setViewport(x, y, w, h);
  renderer.render(scene, topCam);
  renderer.setScissorTest(false);
  renderer.setViewport(0, 0, c.clientWidth, c.clientHeight);
}

// ---------- shared hook for add-on modules (web/objects.js, web/demo.js, ...) ----------
// Modules import nothing from app.js; they use window.twin. Wait for the 'twin-ready' window
// event (fired once the gripper GLB is loaded and placed) before touching twin.model.
//   twin.root        CAD-frame group (Z-up, metres; rotated -90 deg about X). Add CAD-frame objects here.
//   twin.topCam      simulated top webcam;  twin.topCamJPEG(w, h, q) -> Promise<dataURL>
//   twin.onFrame / onState / onLog   arrays of callbacks: (dt, now) / (latest SSE state) / (log event)
//   twin.api(action, body)            POST /api/<action>, updates the run-control panel
const twin = window.twin = {
  THREE, scene, root, renderer, camera, controls, topCam, findNode,
  get kin() { return kin; }, get latest() { return latest; }, get shown() { return shown; },
  model: null, onFrame: [], onState: [], onLog: [],
  api: (action, body) => api(action, body),
  topCamJPEG(w = 640, h = 480, q = 0.85) {
    const rt = new THREE.WebGLRenderTarget(w, h, { colorSpace: THREE.SRGBColorSpace });
    const prevAspect = topCam.aspect;
    topCam.aspect = w / h; topCam.updateProjectionMatrix();
    renderer.setRenderTarget(rt); renderer.render(scene, topCam); renderer.setRenderTarget(null);
    topCam.aspect = prevAspect; topCam.updateProjectionMatrix();
    const px = new Uint8Array(w * h * 4);
    renderer.readRenderTargetPixels(rt, 0, 0, w, h, px);
    rt.dispose();
    const c = document.createElement('canvas'); c.width = w; c.height = h;
    const g = c.getContext('2d'), img = g.createImageData(w, h);
    for (let y = 0; y < h; y++) img.data.set(px.subarray((h - 1 - y) * w * 4, (h - y) * w * 4), y * w * 4);  // flip
    g.putImageData(img, 0, 0);
    return Promise.resolve(c.toDataURL('image/jpeg', q));
  },
};

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
  for (const f of twin.onFrame) { try { f(dt, now); } catch (e) { console.error(e); } }
  renderTopCam();
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
  const w = c.width = c.clientWidth * devicePixelRatio, h = c.height = (c.clientHeight || 120) * devicePixelRatio;
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
  g.fillStyle = css.getPropertyValue('--muted').trim(); g.font = `${10 * devicePixelRatio}px system-ui`;
  g.fillText(`${Math.round(max)} mA`, 4, 12 * devicePixelRatio);
  const last = hist[hist.length - 1];
  g.fillText(`i0 ${last[1]?.toFixed(1) ?? '—'}  i1 ${last[2]?.toFixed(1) ?? '—'}`, w - 130 * devicePixelRatio, 12 * devicePixelRatio);
}

// Tactile: show pressure ABOVE the pad's resting reading, not raw values. Each cell keeps a
// baseline that follows its reading down at once and up slowly (~20 s), so stuck/noisy cells and
// drift read as 0 and only a real press lights up. "Tare" re-learns it (nothing touching the pad).
const TACT_DEADBAND = 12;          // counts above baseline that are still noise
const TACT_FULL = 80;              // counts above baseline shown as full colour
const TACT_UP = 0.002;             // baseline rise per frame (~25 Hz -> ~20 s)
let tactBase = null, tactTareFrames = 0;
function tactileTare() { tactBase = null; tactTareFrames = 0; }
function drawHeat(hex) {
  const c = $('heat'), g = c.getContext('2d');
  const cw = c.width / 32, ch = c.height / 16;
  const raw = new Float32Array(512);
  for (let k = 0; k < 512; k++) raw[k] = parseInt(hex.substr(k * 2, 2), 16) || 0;
  if (!tactBase) tactBase = Float32Array.from(raw);
  let touched = 0, peak = 0;
  g.fillStyle = '#f7f9fb'; g.fillRect(0, 0, c.width, c.height);
  for (let k = 0; k < 512; k++) {
    if (tactTareFrames < 10) tactBase[k] = Math.max(tactBase[k], raw[k]);   // first ~1 s: learn rest
    else if (raw[k] < tactBase[k]) tactBase[k] = raw[k];
    else tactBase[k] += (raw[k] - tactBase[k]) * TACT_UP;
    const d = raw[k] - tactBase[k] - TACT_DEADBAND;
    if (d <= 0) continue;
    const v = Math.min(1, d / TACT_FULL);
    touched++; peak = Math.max(peak, v);
    g.fillStyle = `hsla(${30 - v * 30}, 95%, ${60 - v * 15}%, ${0.25 + 0.75 * v})`;
    g.fillRect((k % 32) * cw, Math.floor(k / 32) * ch, cw + 0.5, ch + 0.5);
  }
  tactTareFrames++;
  g.strokeStyle = '#e3e7ec'; g.lineWidth = 1;   // faint grid every 4 cells so the pad shape reads
  for (let x = 0; x <= 32; x += 4) { g.beginPath(); g.moveTo(x * cw, 0); g.lineTo(x * cw, c.height); g.stroke(); }
  for (let y = 0; y <= 16; y += 4) { g.beginPath(); g.moveTo(0, y * ch); g.lineTo(c.width, y * ch); g.stroke(); }
  $('tact-info').textContent = touched ? `${touched} cells, peak ${Math.round(peak * 100)}%` : 'no contact';
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
    case 'gemini_call': return e.function ? `<span class="k">gemini #${e.step}</span>${e.function} · ${e.reason ?? ''}` : `<span class="k no">gemini</span>${e.error ?? ''}`;
    case 'agent_start': return `${k}${e.goal}`;
    case 'agent_finish': return `${k}<span class="${e.success ? 'ok' : 'no'}">${e.success ? 'success' : 'gave up'}</span> · ${e.summary ?? ''}`;
    case 'agent_done': return `${k}${e.ok ? 'ok' : (e.reason ?? e.summary ?? 'not done')}`;
    case 'frame_request': return '';
    case 'skill_start': return `${k}${e.label}${Object.keys(e.params || {}).length ? ' ' + JSON.stringify(e.params) : ''}`;
    case 'skill': {
      const r = e.result || {};
      const bits = [];
      if (r.contact !== undefined) bits.push(r.contact ? `contact @${r.contact_angle}° · ${r.hold_ma} mA${r.regulated ? '' : ' (not on target)'}` : 'closed fully, no contact');
      if (r.grip_during_twist) bits.push(`grip ${r.grip_during_twist.min_ma}–${r.grip_during_twist.max_ma} mA`);
      if (r.problem) bits.push(r.problem);
      return `${k}${e.label} <span class="${r.ok === false ? 'no' : 'ok'}">${r.ok === false ? 'failed' : 'done'}</span> · ${e.duration_s} s${bits.length ? ' · ' + bits.join(' · ') : ''}`;
    }
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
    const cm = latest.sim?.expected_ma || latest.current_ma;   // sim: calculated expected current (no spikes/noise)
    hist.push([latest.t, cm[0], cm[1]]);
    while (hist.length && latest.t - hist[0][0] > 10) hist.shift();
    count++;
    for (const f of twin.onState) { try { f(latest); } catch (e) { console.error(e); } }
  });
  es.addEventListener('log', (ev) => {
    const e = JSON.parse(ev.data);
    for (const f of twin.onLog) { try { f(e); } catch (err) { console.error(err); } }
    if (e.kind === 'frame_request') {        // server (Gemini agent) wants the simulated top-cam image
      twin.topCamJPEG(640, 480, 0.85).then((image) => fetch('/api/frame', { method: 'POST',
        headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ id: e.id, image }) })).catch(() => {});
      return;
    }
    if (e.kind === 'frame_request') return;
    if (e.kind === 'status') showStatus(e); else { addLog(e); if (e.kind === 'skill' || e.kind === 'reset') loadSkills(); }
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
  if (st.has_camera === false) { $('opt-gemini').checked = false; $('opt-gemini').disabled = true; }
  $('run-info').textContent = st.state !== 'running' ? ''
    : st.skill ? `skill: ${st.label}` : `${st.gemini ? 'Gemini' : 'rules'} · ${st.cycles} cycles max`;
  loadSkills();
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
// ---------- skills panel ----------
let skillsBusy = false;
async function loadSkills() {
  if (skillsBusy) return;
  skillsBusy = true;
  try {
    const r = await fetch('/api/skills');
    if (r.ok) renderSkills(await r.json());
  } catch (e) { /* server gone: leave as is */ }
  skillsBusy = false;
}
function renderSkills(list) {
  const box = $('skills'), idle = ctlState === 'idle';
  box.innerHTML = '';
  const params = new Map();
  let group = null, row = null;
  for (const s of list) {
    if (s.group !== 'manual' && s.name !== 'home') continue;   // demo: Home (sim) + finger strokes t / y / x / c only
    if (s.group !== group) {
      group = s.group;
      row = document.createElement('div');
      row.className = 'skill-row';
      row.innerHTML = `<span class="skill-group">${group}</span>`;
      box.append(row);
    }
    const b = document.createElement('button');
    b.textContent = s.label;
    b.disabled = !idle || !!s.blocked;
    b.title = s.desc + ` (servos ${s.servos.join(', ')})` + (s.blocked ? `

Blocked: ${s.blocked}` : '');
    if (s.blocked) b.classList.add('blocked');
    b.onclick = () => {
      const p = {};
      for (const prm of s.params) p[prm.name] = Number($('p-' + prm.name).value);
      api('skill', { name: s.name, params: p });
    };
    row.append(b);
    for (const prm of s.params) params.set(prm.name, prm);
  }
  const pb = $('skill-params');
  for (const prm of params.values()) {   // one shared input per parameter name, keeps its value
    if ($('p-' + prm.name)) continue;
    const l = document.createElement('label');
    l.title = prm.desc;
    l.innerHTML = `${prm.name} <input type="number" id="p-${prm.name}" min="${prm.lo}" max="${prm.hi}" step="0.1" value="${prm.default}">`;
    pb.append(l);
  }
}

$('start').onclick = () => api('start', { gemini: $('opt-gemini').checked, cycles: Number($('opt-cycles').value) || 20 });
$('stop').onclick = () => api('stop');
$('estop').onclick = () => api('estop');
$('reset-estop').onclick = () => api('reset');
addEventListener('keydown', (e) => { if (e.key === 'Escape') { e.preventDefault(); api('estop'); } });
// t / y: top-finger strokes (same keys as tools/record.py); x / c: same for the bottom fingers. Not while typing.
addEventListener('keydown', (e) => {
  if (e.repeat || e.ctrlKey || e.metaKey || e.altKey || e.target.closest?.('input, textarea, select')) return;
  const name = { t: 'slide_rf', y: 'slide_rb', x: 'bslide_rf', c: 'bslide_rb' }[e.key.toLowerCase()];
  if (!name) return;
  e.preventDefault();
  if (ctlState !== 'idle') { $('ctl-msg').textContent = `${e.key}: busy (${ctlState}), press Stop first`; return; }
  api('skill', { name, params: {} });
});
fetch('/api/status').then((r) => r.ok ? r.json() : { state: 'no controller' }).then(showStatus)
  .catch(() => showStatus({ state: 'no controller' }));

// ---------- boot ----------

kinFile = await (await fetch('kinematics.json')).json();
applyOverrides();
await loadTravel();
buildServoRows();
buildCalib();
$('tare').onclick = tactileTare;
$('topcam-toggle').onclick = () => { $('topcam-wrap').hidden = !$('topcam-wrap').hidden; };
connect();
new GLTFLoader().load('assets/gripper.glb', (gltf) => {
  fixMaterials(gltf.scene);
  root.add(gltf.scene);
  buildJoints(gltf.scene);
  buildTints(gltf.scene);
  frameModel(gltf.scene);
  placeTopCam(gltf.scene);
  twin.model = gltf.scene;
  window.dispatchEvent(new CustomEvent('twin-ready', { detail: twin }));
}, undefined, (err) => console.error('twin: failed to load model', err));
requestAnimationFrame(animate);
