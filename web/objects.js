// Objects + sim animation + tactile-on-finger for the digital twin.
// - Builds the demo objects from objects.json under twin.root (CAD frame, mm -> m): GLB or a
//   procedural pill bottle. Only the selected one is shown.
// - #objects-ui: object selector (POST /api/sim/object) + live readouts from latest.sim (SimRobot).
// - Animates cap / dial / plunger / dropper body from latest.sim, eased. Static if there's no sim.
// - Paints the FlexiTac heatmap on the inner face of the top right finger tip ("finger tip:3").
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const TACT_NODE = 'finger tip:3';   // FlexiTac lives here, under grip tape, inner face (+Y in CAD frame)
const TACT_INSET_MM = 0.3;          // in front of the face so it isn't z-fought / hidden
const TACT_MARGIN = 0.9;            // pad covers this fraction of the tip face (X and Z)
const EASE = 8;                     // 1/s, exponential easing rate for every animated value
const DEFAULT_PITCH_MM = 3;         // pill cap thread pitch if objects.json doesn't say

const $ = (id) => document.getElementById(id);
let T, THREE;                        // window.twin, its THREE
let cfg = null;                      // objects.json
const objs = window.twinObjects = {};  // name -> { def, holder, parts: {role: rig}, anim } (exposed for debugging)
let selected = null;                 // object name shown
let localOnly = false;               // server has no sim -> selection is view-only

// ---------- styles (one <style>, reuses style.css vars) ----------
const css = document.createElement('style');
css.textContent = `
#objects-ui { margin-top: 12px; border-top: 1px solid var(--line); padding-top: 8px; }
#objects-ui .ob-head { display: flex; justify-content: space-between; align-items: baseline; }
#objects-ui h2 { margin: 0 0 6px; }
#objects-ui .seg { display: flex; border: 1px solid var(--line); border-radius: 6px; overflow: hidden; }
#objects-ui .seg button:disabled { opacity: .45; cursor: not-allowed; }
#objects-ui .seg button { flex: 1; border: 0; border-radius: 0; border-right: 1px solid var(--line); padding: 6px 4px; font-size: 12px; }
#objects-ui .seg button:last-child { border-right: 0; }
#objects-ui .seg button.on { background: var(--accent); color: #fff; }
#objects-ui .ro { display: grid; grid-template-columns: auto 1fr; gap: 2px 10px; margin-top: 8px; font-size: 12px;
  font-variant-numeric: tabular-nums; }
#objects-ui .ro > span:nth-child(odd) { color: var(--muted); }
#objects-ui .badge { display: inline-block; padding: 0 6px; border-radius: 999px; border: 1px solid var(--line); margin-right: 4px; }
#objects-ui .badge.on { color: var(--good); border-color: var(--good); }
#objects-ui .badge.big { color: #fff; background: var(--good); border-color: var(--good); font-weight: 600; }
#objects-ui .badge.warn { color: #fff; background: var(--bad); border-color: var(--bad); font-weight: 600; }
#objects-ui .note { color: var(--muted); font-size: 12px; margin-top: 4px; min-height: 1em; }
#objects-ui label.pad { font-size: 12px; color: var(--muted); }
`;
document.head.append(css);

// ---------- shared materials / helpers ----------
let outlineMat;
function matte(m) {          // same look as app.js fixMaterials
  m.metalness = 0; m.roughness = m.roughness ?? 0.6;
  m.polygonOffset = true; m.polygonOffsetFactor = 1; m.polygonOffsetUnits = 1;
  return m;
}
function outline(root) {     // thin near-black hard-edge lines, children of each mesh
  root.traverse((o) => {
    if (!o.isMesh || o.userData.noOutline) return;
    const l = new THREE.LineSegments(new THREE.EdgesGeometry(o.geometry, 30), outlineMat);
    l.raycast = () => {};
    o.add(l);
  });
}
const mm = (v) => v / 1000;
const ease = (cur, tgt, dt) => cur + (tgt - cur) * (1 - Math.exp(-EASE * dt));

// ---------- procedural pill bottle (object frame: Z up, axis at origin, mm) ----------
function cylZ(r, h, mat, seg = 48) {      // cylinder along +Z, base at z=0
  const g = new THREE.CylinderGeometry(r, r, h, seg);
  g.rotateX(Math.PI / 2); g.translate(0, 0, h / 2);
  return new THREE.Mesh(g, mat);
}
function buildPillBottle(p) {
  // objects.json params: body spans neck_z-body_h..neck_z (neck = its top neck_h), cap neck_z..neck_z+cap_h
  const P = { body_d: 34, body_h: 75, neck_d: 30, neck_h: 4, neck_z: 17, cap_d: 38, cap_h: 18, cap_ridges: 24,
    label_z: null, body_color: '#d9822b', cap_color: '#f2f2f2', label_color: '#ffffff', ...p };
  const S = 0.001;                        // build in mm, scale the group
  const g = new THREE.Group(); g.scale.setScalar(S);
  const amber = matte(new THREE.MeshStandardMaterial({ color: P.body_color, transparent: true, opacity: 0.78, roughness: 0.45 }));
  const white = matte(new THREE.MeshStandardMaterial({ color: P.cap_color, roughness: 0.6 }));
  const paper = matte(new THREE.MeshStandardMaterial({ color: P.label_color, roughness: 0.8 }));
  const dark = matte(new THREE.MeshStandardMaterial({ color: 0x1d232b, roughness: 0.6 }));
  const red = matte(new THREE.MeshStandardMaterial({ color: 0xd63a30, roughness: 0.6 }));
  // body: z0 .. shoulder (neck_z - neck_h), then the threaded neck up to neck_z (+3 hidden in the cap)
  const body = new THREE.Group();
  const z0 = P.neck_z - P.body_h, bh = P.body_h - P.neck_h;
  const b = cylZ(P.body_d / 2, bh, amber); b.position.z = z0; body.add(b);
  const [l0, l1] = P.label_z || [z0 + bh * 0.2, z0 + bh * 0.7];
  const lbl = cylZ(P.body_d / 2 + 0.3, l1 - l0, paper); lbl.position.z = l0; body.add(lbl);
  const band = cylZ(P.body_d / 2 + 0.4, 3, red); band.position.z = l1 - 8; body.add(band);
  const neck = cylZ(P.neck_d / 2, P.neck_h + 3, amber); neck.position.z = P.neck_z - P.neck_h; body.add(neck);
  const nm = new THREE.Mesh(new THREE.BoxGeometry(1.6, 1.2, P.neck_h + 3), dark);   // neck marker on +X
  nm.position.set(P.neck_d / 2, 0, P.neck_z - P.neck_h + (P.neck_h + 3) / 2); body.add(nm);
  const bm = new THREE.Mesh(new THREE.BoxGeometry(1.6, 1.2, 10), dark);            // shoulder marker (lines up with cap's)
  bm.position.set(P.body_d / 2 + 0.3, 0, P.neck_z - P.neck_h - 6); body.add(bm);
  g.add(body);
  // cap: own group so it can turn/rise about the object axis; its origin is on the axis
  const cap = new THREE.Group(); cap.position.z = P.neck_z;
  cap.add(cylZ(P.cap_d / 2, P.cap_h, white, 64));
  for (let i = 0; i < P.cap_ridges; i++) {    // grip ridges
    const a = (i / P.cap_ridges) * Math.PI * 2;
    const r = new THREE.Mesh(new THREE.BoxGeometry(1.4, 1.0, P.cap_h - 3), white);
    r.position.set(Math.cos(a) * (P.cap_d / 2 + 0.3), Math.sin(a) * (P.cap_d / 2 + 0.3), 1.5 + (P.cap_h - 3) / 2);
    r.rotation.z = a; r.userData.noOutline = true;
    cap.add(r);
  }
  const cm = new THREE.Mesh(new THREE.BoxGeometry(2.2, 2.2, P.cap_h + 0.4), dark);   // side stripe on +X
  cm.position.set(P.cap_d / 2 + 0.6, 0, P.cap_h / 2); cap.add(cm);
  const tm = new THREE.Mesh(new THREE.BoxGeometry(P.cap_d / 2, 2.2, 0.6), dark);       // top stripe, centre -> +X
  tm.position.set(P.cap_d / 4, 0, P.cap_h + 0.3); cap.add(tm);
  g.add(cap);
  outline(g);
  return { group: g, nodes: { cap, body } };
}

// ---------- object loading ----------
async function fetchConfig() {
  for (let i = 0; i < 20; i++) {          // another agent may still be writing it
    try {
      const r = await fetch('objects.json', { cache: 'no-store' });
      if (r.ok) return await r.json();
    } catch {}
    await new Promise((res) => setTimeout(res, 1500));
  }
  return null;
}

function placeGroup(group, tf = {}) {
  const p = tf.position_mm || cfg.work_center_mm || [85, 0, 0], r = tf.rotation_deg || [0, 0, 0];
  const holder = new THREE.Group();       // holder: CAD transform; group inside keeps its own scale (mm build)
  holder.position.set(mm(p[0]), mm(p[1]), mm(p[2]));
  holder.rotation.set(...r.map((d) => THREE.MathUtils.degToRad(d)));
  holder.scale.setScalar(tf.scale ?? 1);
  holder.add(group);
  holder.visible = false;
  T.root.add(holder);
  return holder;
}

// Record each animated part's rest pose + the object axis (CAD +Z through the transform origin)
// in the part's parent frame, so turning/lifting works whatever the GLB node hierarchy is.
function rigPart(holder, node) {
  T.root.updateMatrixWorld(true);
  const ax0 = holder.position.clone(), ax1 = ax0.clone().add(new THREE.Vector3(0, 0, 1));   // root frame, metres
  const w0 = T.root.localToWorld(ax0.clone()), w1 = T.root.localToWorld(ax1.clone());
  const p0 = node.parent.worldToLocal(w0.clone()), p1 = node.parent.worldToLocal(w1.clone());
  const perMeter = p1.clone().sub(p0);
  // local centre of the part's geometry (pivot for squash)
  const box = new THREE.Box3().setFromObject(node);
  const centre = box.isEmpty() ? new THREE.Vector3() : node.worldToLocal(box.getCenter(new THREE.Vector3()));
  // which of the node's local axes is closest to CAD Y (squeeze direction)
  const yW = new THREE.Vector3(0, 1, 0).applyQuaternion(T.root.getWorldQuaternion(new THREE.Quaternion()));
  const nq = node.getWorldQuaternion(new THREE.Quaternion());
  const dots = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)]
    .map((v) => Math.abs(v.applyQuaternion(nq).dot(yW)));
  const sqAxis = dots.indexOf(Math.max(...dots));
  return { node, base: node.position.clone(), baseQ: node.quaternion.clone(), baseS: node.scale.clone(),
    p0, dir: perMeter.clone().normalize(), perMeter, centre, sqAxis };
}

// pose a rigged part: turn `ang` rad about the object axis, slide `lift` m along it, squash k along CAD Y
let _q, _v;   // scratch, made in init (THREE comes from window.twin)
function posePart(r, ang = 0, lift = 0, squash = 0) {
  _q.setFromAxisAngle(r.dir, ang);
  r.node.position.copy(r.base).sub(r.p0).applyQuaternion(_q).add(r.p0).addScaledVector(r.perMeter, lift);
  r.node.quaternion.copy(_q).multiply(r.baseQ);
  r.node.scale.copy(r.baseS);
  if (squash) {
    const s = ['x', 'y', 'z'][r.sqAxis];
    r.node.scale[s] *= 1 - squash;
    // keep the geometry centre in place: pos += R * (S0 - S) * c
    _v.set(0, 0, 0); _v[s] = r.centre[s] * r.baseS[s] * squash;
    r.node.position.add(_v.applyQuaternion(r.node.quaternion));
  }
}

async function buildObjects() {
  const loader = new GLTFLoader();
  for (const [name, def] of Object.entries(cfg.objects || {})) {
    try {
      let group, nodes = {};
      if (def.procedural === 'pill_bottle' || (!def.glb && name === 'pill_bottle')) {
        ({ group, nodes } = buildPillBottle(def.params || {}));
      } else if (def.glb) {
        const gltf = await loader.loadAsync(def.glb);
        group = gltf.scene;
        group.traverse((o) => { if (o.isMesh) { o.material = matte(o.material.clone()); o.castShadow = true; } });
        outline(group);
        for (const [role, n] of Object.entries(def.parts || {})) if (n) nodes[role] = T.findNode(group, n);
      } else continue;
      const holder = placeGroup(group, def.transform);
      const parts = {};
      for (const [role, node] of Object.entries(nodes)) {
        if (node) parts[role] = rigPart(holder, node);
        else console.warn('objects: part not found', name, role);
      }
      objs[name] = { def, holder, parts, anim: {}, drops: null, dropList: [] };
    } catch (e) { console.error('objects: failed to build', name, e); }
  }
}

function show(name) {
  if (!objs[name]) return;
  selected = name;
  for (const [n, o] of Object.entries(objs)) o.holder.visible = n === name;
  document.querySelectorAll('#objects-ui .seg button').forEach((b) => b.classList.toggle('on', b.dataset.name === name));
}

// ---------- UI ----------
function buildUI() {
  const box = $('objects-ui');
  if (!box) return;
  const names = Object.keys(objs);
  box.innerHTML = `<div class="ob-head"><h2>Object</h2>
      <label class="pad" title="Tactile heatmap on the top right finger"><input type="checkbox" id="ob-pad" checked> pad</label></div>
    <div class="seg">${names.map((n) => `<button data-name="${n}">${objs[n].def.label || n}</button>`).join('')}</div>
    <div class="ro" id="ob-ro"></div><div class="note" id="ob-note"></div>`;
  box.querySelectorAll('.seg button').forEach((b) => b.onclick = () => choose(b.dataset.name));
  const lock = (state) => box.querySelectorAll('.seg button').forEach((b) => {   // switch objects only when idle
    b.disabled = !localOnly && state != null && state !== 'idle';
    b.title = b.disabled ? 'stop the running task first' : ''; });
  T.onLog.push((e) => { if (e.kind === 'status') lock(e.state); });
  fetch('/api/status').then((r) => r.ok ? r.json() : null).then((s) => s && lock(s.state)).catch(() => {});
  $('ob-pad').onchange = (e) => { if (pad) pad.group.visible = e.target.checked; };
}
async function choose(name) {
  if (localOnly) { show(name); $('ob-note').textContent = 'view only (no sim on the server)'; return; }
  try {
    const r = await fetch('/api/sim/object', { method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }) });
    const j = await r.json().catch(() => ({}));
    if (r.status === 404) { localOnly = true; show(name); $('ob-note').textContent = 'view only (start with --sim to simulate)'; }
    else if (!r.ok || j.ok === false) $('ob-note').textContent = j.msg || `can't switch now (${r.status})`;
    else show(name);
  } catch { $('ob-note').textContent = 'view only (server unreachable)'; }
}

const yes = (v) => `<span class="badge ${v ? 'on' : ''}">${v ? 'yes' : 'no'}</span>`;
function updateUI() {
  const ro = $('ob-ro');
  if (!ro) return;
  const s = T.latest?.sim;
  if (!s) { ro.innerHTML = '<span>sim</span><span>— (real / mock robot: object static)</span>'; return; }
  const def = objs[s.object]?.def || {}, sim = def.sim || {};
  const rows = [['task', s.task ?? def.task ?? '—'],
    ['contact', `bottom ${yes(s.contact?.bottom)} top ${yes(s.contact?.top)}`],
    ['squeeze', `${(s.squeeze_mm?.bottom ?? 0).toFixed(1)} / ${(s.squeeze_mm?.top ?? 0).toFixed(1)} mm`]];
  if (s.task === 'unscrew' || s.object === 'pill_bottle') {
    const turns = (s.cap_turn_deg ?? 0) / 360, need = sim.turns_to_open;
    rows.push(['cap', `${Math.round(s.cap_turn_deg ?? 0)}° · ${turns.toFixed(2)}${need ? ' / ' + need : ''} turns ${s.cap_off ? '<span class="badge big">CAP OFF</span>' : ''}`]);
    if (s.cap_lift_mm) rows.push(['cap lift', `${s.cap_lift_mm.toFixed(1)} mm`]);
  }
  if (s.task === 'dispense' || s.object === 'eye_dropper') rows.push(['drops', `${s.drops ?? 0}`]);
  if (s.task === 'dial_and_inject' || s.object === 'insulin_pen')
    rows.push(['dose', `${s.dose_units ?? 0} u`], ['plunger', `${(s.plunger_mm ?? 0).toFixed(1)} mm`]);
  rows.push(['halves', `${(s.halves_mm ?? 0).toFixed(1)} mm`]);
  if (s.slip) rows.push(['grip', '<span class="badge warn">SLIP</span>']);
  ro.innerHTML = rows.map(([k, v]) => `<span>${k}</span><span>${v}</span>`).join('');
  if (!$('ob-note').dataset.keep) $('ob-note').textContent = s.note || '';
}

// ---------- animation from latest.sim ----------
function animate(dt) {
  const s = T.latest?.sim;
  if (s?.object && s.object !== selected && objs[s.object]) show(s.object);
  const o = objs[selected];
  if (!o) return;
  const a = o.anim, sim = o.def.sim || {}, P = o.parts;
  if (!s || s.object !== selected) {     // no sim (or sim on another object): stay at rest
    for (const r of Object.values(P)) posePart(r);
    return;
  }
  if (o.def.task === 'unscrew' && P.cap) {
    const pitch = sim.thread_pitch_mm ?? o.def.params?.thread_pitch_mm ?? DEFAULT_PITCH_MM;
    const deg = s.cap_turn_deg ?? 0;
    // SimRobot's cap_lift_mm already includes the thread rise (cap_thread_mm) + lift with the halves
    const lift = s.cap_thread_mm !== undefined ? s.cap_lift_mm ?? 0
      : pitch * Math.min(deg / 360, sim.turns_to_open ?? Infinity) + (s.cap_lift_mm ?? 0);
    a.cap = ease(a.cap ?? 0, deg, dt); a.lift = ease(a.lift ?? 0, lift, dt);
    posePart(P.cap, THREE.MathUtils.degToRad(a.cap), mm(a.lift));
  }
  if (o.def.task === 'dispense') {
    const D = o.def.grip?.top_d_mm || 30;
    a.sq = ease(a.sq ?? 0, Math.min(0.12, 0.6 * (s.squeeze_mm?.top ?? 0) / D), dt);   // visible but slight
    if (P.body) posePart(P.body, 0, 0, a.sq);
    if (o.drops == null) o.drops = s.drops ?? 0;
    if ((s.drops ?? 0) > o.drops) spawnDrop(o);
    o.drops = s.drops ?? 0;
  }
  if (o.def.task === 'dial_and_inject') {
    // dialing N units turns the dial N*360/units_per_turn and extends dial + button up by
    // N*plunger_mm_per_unit; plunger_mm (pushed in) takes that back. SimRobot's dial_deg / knob_out_mm win.
    const upt = sim.units_per_turn || 20, ppu = sim.plunger_mm_per_unit ?? 0.25;
    const ext = s.knob_out_mm ?? Math.max(0, (s.dose_units ?? 0) * ppu - (s.plunger_mm ?? 0));
    a.dial = ease(a.dial ?? 0, s.dial_deg ?? (s.dose_units ?? 0) * 360 / upt, dt);
    a.ext = ease(a.ext ?? 0, ext, dt);
    if (P.dial) posePart(P.dial, THREE.MathUtils.degToRad(a.dial), mm(a.ext));
    if (P.plunger && P.plunger !== P.dial) posePart(P.plunger, THREE.MathUtils.degToRad(a.dial), mm(a.ext));
  }
  // falling drops (root frame, -Z), gone after ~0.7 s
  for (const d of o.dropList) {
    d.t += dt; d.v += 9.81 * dt * 0.25; d.mesh.position.z -= d.v * dt;
    d.mesh.scale.setScalar(Math.min(1, d.t * 6));
  }
  o.dropList = o.dropList.filter((d) => { if (d.t < 0.7) return true; T.root.remove(d.mesh); return false; });
}
const dropMat = () => new THREE.MeshStandardMaterial({ color: 0x5fb0ff, transparent: true, opacity: 0.85, roughness: 0.15, metalness: 0 });
function spawnDrop(o) {
  // drip from the lowest point of the nozzle (or the whole object): held inverted, nozzle down
  const org = o.def.sim?.drop_origin_mm;
  const m = new THREE.Mesh(new THREE.SphereGeometry(mm(1.6), 16, 12), dropMat());
  T.root.add(m);
  o.dropList.push({ mesh: m, t: 0, v: 0.02 }); o.spawned = (o.spawned || 0) + 1;
  if (org) { m.position.set(mm(org[0]), mm(org[1]), mm(org[2] - 1.5)); return; }
  T.root.updateMatrixWorld(true);
  const src = o.parts.nozzle?.node || o.holder;
  const wb = new THREE.Box3().setFromObject(src);
  const c = T.root.worldToLocal(wb.getCenter(new THREE.Vector3()));
  let zmin = Infinity;   // lowest corner in root (CAD) frame
  for (const x of [wb.min.x, wb.max.x]) for (const y of [wb.min.y, wb.max.y]) for (const z of [wb.min.z, wb.max.z])
    zmin = Math.min(zmin, T.root.worldToLocal(new THREE.Vector3(x, y, z)).z);
  m.position.set(c.x, c.y, zmin - mm(1.5));
}

// ---------- tactile pad on the finger ----------
let pad = null;
const TACT_DEADBAND = 12, TACT_FULL = 80, TACT_UP = 0.002;   // same as app.js drawHeat
function buildPad() {
  const node = T.findNode(T.model, TACT_NODE);
  if (!node) { console.warn('objects: no', TACT_NODE, 'for the tactile pad'); return; }
  T.root.updateMatrixWorld(true);
  // tip's box in the CAD (root) frame
  const wb = new THREE.Box3().setFromObject(node), box = new THREE.Box3();
  for (const x of [wb.min.x, wb.max.x]) for (const y of [wb.min.y, wb.max.y]) for (const z of [wb.min.z, wb.max.z])
    box.expandByPoint(T.root.worldToLocal(new THREE.Vector3(x, y, z)));
  const size = box.getSize(new THREE.Vector3()), c = box.getCenter(new THREE.Vector3());
  // Same pad geometry as src/sim_robot.py: 32 cols x 2 mm from CAD x = 40.85 (finger at its CAD pose),
  // 16 rows x 2 mm over the full height of the top layer; row 0 at the top.
  const PAD_X0_MM = 40.85, PITCH_MM = 2.0;
  const w = mm(32 * PITCH_MM), h = size.z;
  c.x = mm(PAD_X0_MM + 16 * PITCH_MM);
  const canvas = document.createElement('canvas'); canvas.width = 256; canvas.height = 128;
  const tex = new THREE.CanvasTexture(canvas); tex.colorSpace = THREE.SRGBColorSpace;
  tex.magFilter = THREE.NearestFilter;
  const mat = new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false, side: THREE.DoubleSide,
    toneMapped: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
  const geo = new THREE.PlaneGeometry(w, h);
  const plane = new THREE.Mesh(geo, mat);
  plane.raycast = () => {};
  const edge = new THREE.LineSegments(new THREE.EdgesGeometry(geo), new THREE.LineBasicMaterial({ color: 0x8a939e, toneMapped: false }));
  plane.add(edge);
  // PlaneGeometry: normal +Z, U +X, V +Y. Rotate -90 deg about X: normal -> +Y, U -> +X, V -> -Z
  // (so canvas top = low Z: rows are drawn bottom-up below to keep row 0 at the top of the finger).
  const group = new THREE.Group();
  group.add(plane);
  group.rotation.x = -Math.PI / 2;
  group.position.set(c.x, box.max.y + mm(TACT_INSET_MM), c.z);   // inner face of the right tip (max Y)
  group.renderOrder = 2;
  T.root.add(group);
  T.root.updateMatrixWorld(true);
  node.attach(group);                     // keep world pose, now moves with the finger
  pad = { group, canvas, tex, base: null, frames: 0, t: 0 };
}
function drawPad(dt) {
  if (!pad || !pad.group.visible) return;
  pad.t += dt; if (pad.t < 0.1) return; pad.t = 0;   // 10 Hz like the panel
  const hex = T.latest?.tactile;
  if (!hex || hex.length < 1024) return;
  const raw = new Float32Array(512);
  for (let k = 0; k < 512; k++) raw[k] = parseInt(hex.substr(k * 2, 2), 16) || 0;
  if (!pad.base) pad.base = Float32Array.from(raw);
  const g = pad.canvas.getContext('2d'), cw = pad.canvas.width / 32, ch = pad.canvas.height / 16;
  g.clearRect(0, 0, pad.canvas.width, pad.canvas.height);
  g.fillStyle = 'rgba(120,130,140,0.12)'; g.fillRect(0, 0, pad.canvas.width, pad.canvas.height);   // faint pad area
  for (let k = 0; k < 512; k++) {
    if (pad.frames < 10) pad.base[k] = Math.max(pad.base[k], raw[k]);   // first ~1 s: learn rest
    else if (raw[k] < pad.base[k]) pad.base[k] = raw[k];
    else pad.base[k] += (raw[k] - pad.base[k]) * TACT_UP;
    const d = raw[k] - pad.base[k] - TACT_DEADBAND;
    if (d <= 0) continue;
    const v = Math.min(1, d / TACT_FULL);
    g.fillStyle = `hsla(${40 - v * 40}, 95%, ${58 - v * 15}%, ${0.35 + 0.65 * v})`;
    g.fillRect((k % 32) * cw, (15 - Math.floor(k / 32)) * ch, cw + 0.5, ch + 0.5);
  }
  pad.frames++;
  pad.tex.needsUpdate = true;
}

// ---------- ?fakesim=<name>: synthetic looping sim + tactile press (preview without SimRobot) ----------
const FAKE = new URLSearchParams(location.search).get('fakesim');
let fakeT = 0;
function fakeSim(dt) {
  fakeT = (fakeT + dt) % 12;
  const k = fakeT / 12, name = FAKE in objs ? FAKE : 'pill_bottle';
  const s = { object: name, task: objs[name]?.def.task, contact: { bottom: true, top: k > 0.05 },
    squeeze_mm: { bottom: 1, top: k > 0.05 ? 1.5 : 0 }, halves_mm: 0, slip: false, note: 'fakesim preview' };
  if (name === 'pill_bottle') Object.assign(s, { cap_turn_deg: Math.min(540, k * 800), cap_off: k * 800 >= 540,
    cap_lift_mm: Math.max(0, k * 800 - 540) / 20 });
  if (name === 'eye_dropper') Object.assign(s, { drops: Math.floor(k * 6), squeeze_mm: { bottom: 1, top: 2 + 2 * Math.sin(k * 40) } });
  if (name === 'insulin_pen') Object.assign(s, { dose_units: Math.min(10, Math.floor(k * 20)), plunger_mm: Math.max(0, (k - 0.6) * 6.25) });
  const tac = new Uint8Array(512).fill(10);
  if (s.contact.top) for (let r = 4; r < 12; r++) for (let c = 10; c < 22; c++)
    tac[r * 32 + c] = 10 + Math.max(0, 90 - 12 * Math.hypot(r - 8, (c - 16) / 1.5));
  return { sim: s, tactile: Array.from(tac, (v) => v.toString(16).padStart(2, '0')).join('') };
}

// ---------- start ----------
async function init(twin) {
  if (T) return;
  T = twin; THREE = twin.THREE; _q = new THREE.Quaternion(); _v = new THREE.Vector3();
  if (FAKE) {   // wrap twin.latest so every reader here sees the synthetic sim
    let fl = null;
    T = Object.create(twin, { latest: { get: () => fl ? { ...(twin.latest || {}), ...fl } : twin.latest } });
    twin.onFrame.push((dt) => { fl = fakeSim(dt); });
  }
  outlineMat = new THREE.LineBasicMaterial({ color: 0x10151c, transparent: true, opacity: 0.55, toneMapped: false });
  buildPad();
  T.onFrame.push((dt) => drawPad(dt));
  cfg = await fetchConfig();
  if (!cfg) { console.warn('objects: no objects.json'); if ($('objects-ui')) $('objects-ui').innerHTML = '<h2>Object</h2><div class="note">objects.json not found</div>'; return; }
  await buildObjects();
  buildUI();
  try { const r = await fetch('/api/sim'); if (r.status === 404) localOnly = true; } catch { localOnly = true; }
  show(T.latest?.sim?.object && objs[T.latest.sim.object] ? T.latest.sim.object : Object.keys(objs)[0]);
  T.onFrame.push((dt) => animate(dt));
  setInterval(updateUI, 200);
  // ?objview=x,y,z (mm, CAD frame): debug camera aimed at the work area, for screenshots
  const ov = new URLSearchParams(location.search).get('objview');
  if (ov) {
    const [x, y, z] = ov.split(',').map(Number), tgt = T.root.localToWorld(new THREE.Vector3(0.085, 0, 0.01));
    T.camera.position.copy(T.root.localToWorld(new THREE.Vector3(mm(x), mm(y), mm(z))));
    T.controls.target.copy(tgt); T.controls.update();
  }
  console.log('objects: ready', Object.keys(objs).join(','), pad ? 'pad ok' : 'no pad');
}
if (window.twin?.model) init(window.twin);
else window.addEventListener('twin-ready', (e) => init(e.detail));
