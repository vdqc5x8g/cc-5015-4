import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';

// ------------------------------------------------------------------ state
const state = {
  text: '',
  doc: null,
  selectedRepairs: new Set(),
  canvasWidth: 0,
  canvasHeight: 0,
  exaggeration: 1,
  activeTab: 'samples',
};

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0f15);

const renderer = new THREE.WebGLRenderer({
  canvas: document.getElementById('view'),
  antialias: true,
});
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(1, 1, false);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1e7);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.screenSpacePanning = false;

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const key = new THREE.DirectionalLight(0xffffff, 1.15);
key.position.set(0.6, 1.0, 0.45);
scene.add(key);
const fill = new THREE.DirectionalLight(0x88aaff, 0.35);
fill.position.set(-0.7, 0.4, -0.5);
scene.add(fill);

// Root transform: data is (x, y, elevation). Rotating -90° about X maps
// (x, y, z) -> (x, z, -y), so elevation becomes the vertical (Y-up) axis.
// Vertical exaggeration scales the data z axis only; stored coordinates and
// exports keep real units.
const root = new THREE.Group();
root.rotation.x = -Math.PI / 2;
root.scale.set(1, 1, 1);
scene.add(root);

// Separate groups so toggles are cheap.
const groups = {
  surface: new THREE.Group(),
  wire: new THREE.Group(),
  contours: new THREE.Group(),
  labels: new THREE.Group(),
  controls: new THREE.Group(),
  constraints: new THREE.Group(),
  issues: new THREE.Group(),
  ref: new THREE.Group(),
  holes: new THREE.Group(),
  original: new THREE.Group(),
};
for (const g of Object.values(groups))
  root.add(g);

const labelEls = [];
let labelsVisible = true;

// ------------------------------------------------------------- formatting
const fmt = (v, d = 2) => {
  if (!Number.isFinite(v)) return '—';
  const abs = Math.abs(v);
  if (abs !== 0 && (abs >= 1e5 || abs < 1e-3)) return v.toExponential(2);
  return v.toFixed(d);
};
const fmtCount = (n) => n.toLocaleString('en-US');
const el = (tag, cls, text) => {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
};
function kv(parent, key, value, cls = '') {
  parent.appendChild(el('div', 'k', key));
  parent.appendChild(el('div', 'v ' + cls, value));
}

// ------------------------------------------------------------- geometry math
function toRing(list) {
  if (!list || list.length < 2) return list || [];
  const first = list[0], last = list[list.length - 1];
  if (Math.hypot(first[0] - last[0], first[1] - last[1]) < 1e-12)
    return list.slice(0, -1);
  return list;
}

// ------------------------------------------------------------- scene clearing
function clearGroup(group) {
  while (group.children.length) {
    const child = group.children.pop();
    child.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      if (o.material) {
        if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose());
        else o.material.dispose();
      }
    });
    if (child.element && child.element.parentNode)
      child.element.parentNode.removeChild(child.element);
  }
}

function clearScene() {
  for (const g of Object.values(groups)) clearGroup(g);
  for (const div of labelEls)
    if (div.parentNode) div.parentNode.removeChild(div);
  labelEls.length = 0;
}

// ------------------------------------------------------------- camera framing
function frameBounds(box, azEl) {  if (box.isEmpty()) return;
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 1e-6);
  const dist = maxDim * 1.8;
  const dir = new THREE.Vector3(0.62, 0.72, 0.9).normalize();
  if (azEl) dir.setFromSphericalCoords(1, azEl.phi, azEl.theta);
  camera.position.copy(center).addScaledVector(dir, dist);
  camera.near = Math.max(dist / 1000, 1e-4);
  camera.far = dist * 100;
  camera.updateProjectionMatrix();
  controls.target.copy(center);
  controls.update();
}

// ------------------------------------------------------------- mesh building
function buildSurface(doc) {
  const m = doc.mesh;
  if (!m || !m.ok || !m.positions) return;

  const pos = m.positions;
  const idx = m.indices || [];
  const unsupported = m.unsupported || [];
  const n = pos.length / 3;

  const geometry = new THREE.BufferGeometry();
  const verts = new Float32Array(pos.length);
  verts.set(pos);
  geometry.setAttribute('position', new THREE.BufferAttribute(verts, 3));
  geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(idx), 1));

  // Per-vertex elevation colour: deep blue low -> orange high.
  const zs = new Float32Array(n);
  let zmin = Infinity, zmax = -Infinity;
  for (let i = 0; i < n; i++) {
    const z = pos[3 * i + 2];
    zs[i] = z;
    if (z < zmin) zmin = z;
    if (z > zmax) zmax = z;
  }
  const colors = new Float32Array(n * 3);
  const low = new THREE.Color(0x1e3a5f);
  const mid = new THREE.Color(0x2f7f6f);
  const high = new THREE.Color(0xc56a3f);
  const tmp = new THREE.Color();
  const span = Math.max(zmax - zmin, 1e-9);
  for (let i = 0; i < n; i++) {
    const t = (zs[i] - zmin) / span;
    if (t < 0.5) tmp.copy(low).lerp(mid, t * 2);
    else tmp.copy(mid).lerp(high, (t - 0.5) * 2);
    colors[3 * i] = tmp.r;
    colors[3 * i + 1] = tmp.g;
    colors[3 * i + 2] = tmp.b;
  }
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geometry.computeVertexNormals();

  const material = new THREE.MeshStandardMaterial({
    vertexColors: true,
    roughness: 0.82,
    metalness: 0.02,
    flatShading: false,
    side: THREE.DoubleSide,
    transparent: false,
    polygonOffset: true,
    polygonOffsetFactor: 1,
    polygonOffsetUnits: 1,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.userData.kind = 'surface';
  mesh.userData.triangles = idx.length / 3;
  groups.surface.add(mesh);

  // Wireframe: derive merged unique triangle edges so shared edges draw once.
  const edgeMap = new Map();
  const addEdge = (a, b) => {
    const k = a < b ? `${a}_${b}` : `${b}_${a}`;
    if (!edgeMap.has(k)) edgeMap.set(k, [a, b]);
  };
  for (let t = 0; t < idx.length; t += 3) {
    addEdge(idx[t], idx[t + 1]);
    addEdge(idx[t + 1], idx[t + 2]);
    addEdge(idx[t + 2], idx[t]);
  }
  const wirePos = new Float32Array(edgeMap.size * 2 * 3);
  let w = 0;
  for (const [a, b] of edgeMap.values()) {
    wirePos[w++] = pos[3 * a]; wirePos[w++] = pos[3 * a + 1]; wirePos[w++] = pos[3 * a + 2];
    wirePos[w++] = pos[3 * b]; wirePos[w++] = pos[3 * b + 1]; wirePos[w++] = pos[3 * b + 2];
  }
  const wireGeo = new THREE.BufferGeometry();
  wireGeo.setAttribute('position', new THREE.BufferAttribute(wirePos, 3));
  const wire = new THREE.LineSegments(
    wireGeo,
    new THREE.LineBasicMaterial({ color: 0x10202c, transparent: true, opacity: 0.45 }),
  );
  groups.wire.add(wire);

  // Constrained edges: real edges from the triangulation, lifted slightly.
  const con = m.constrained_edges_indices || [];
  const kinds = m.constrained_kinds || [];
  if (con.length) {
    const byKind = new Map();
    for (let i = 0; i < con.length / 2; i++) {
      const kind = kinds[i] || 'contour';
      const a = con[2 * i], b = con[2 * i + 1];
      if (!byKind.has(kind)) byKind.set(kind, []);
      byKind.get(kind).push(a, b);
    }
    const palette = {
      boundary: 0x5fa8ff,
      hole: 0xb07cff,
      contour: 0xffd166,
      fault: 0xff6b6b,
    };
    for (const [kind, list] of byKind) {
      const arr = new Float32Array(list.length * 3);
      for (let i = 0; i < list.length; i++) {
        const v = list[i];
        arr[3 * i] = pos[3 * v];
        arr[3 * i + 1] = pos[3 * v + 1];
        arr[3 * i + 2] = pos[3 * v + 2] + 0.0008 * Math.max(1, span / 50);
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(arr, 3));
      const line = new THREE.LineSegments(
        g,
        new THREE.LineBasicMaterial({ color: palette[kind] ?? 0xffd166 }),
      );
      groups.constraints.add(line);
    }
  }

  // Unsupported triangles: any vertex outside the observation support radius.
  // Drawn as a translucent amber overlay so it does not compete with the
  // elevation colour ramp.
  if (unsupported.some((u) => u)) {
    const over = [];
    for (let t = 0; t < idx.length; t += 3) {
      const a = idx[t], b = idx[t + 1], c = idx[t + 2];
      if (unsupported[a] || unsupported[b] || unsupported[c])
        over.push(a, b, c);
    }
    if (over.length) {
      const g = new THREE.BufferGeometry();
      const p = new Float32Array(over.length * 3);
      for (let i = 0; i < over.length; i++) {
        p[3 * i] = pos[3 * over[i]];
        p[3 * i + 1] = pos[3 * over[i] + 1];
        p[3 * i + 2] = pos[3 * over[i] + 2];
      }
      g.setAttribute('position', new THREE.BufferAttribute(p, 3));
      const overlay = new THREE.Mesh(
        g,
        new THREE.MeshBasicMaterial({
          color: 0xe08b3c, transparent: true, opacity: 0.34,
          depthWrite: false, side: THREE.DoubleSide,
        }),
      );
      overlay.renderOrder = 2;
      overlay.userData.kind = 'unsupported-overlay';
      groups.surface.add(overlay);
    }
  }

  // Store bounds for framing and label placement.
  mesh.userData.bounds = { zmin, zmax };
}

// ------------------------------------------------------------- surface sampling
// Drape overlays onto the TIN: look up the interpolated surface elevation at a
// plan position and offset slightly along +Z so the line/render test is stable.
function makeHeightSampler(doc) {
  const m = doc.mesh;
  if (!m || !m.ok || !m.positions || !m.indices || !m.indices.length) return null;
  const pos = m.positions;
  const idx = m.indices;
  const triCount = idx.length / 3;

  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < pos.length; i += 3) {
    const x = pos[i], y = pos[i + 1];
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  }
  const nx = Math.max(1, Math.min(96, Math.round(Math.sqrt(triCount))));
  const ny = nx;
  const dx = (maxX - minX) / nx || 1;
  const dy = (maxY - minY) / ny || 1;
  const buckets = new Array(nx * ny);
  const cellOf = (x, y) => {
    const cx = Math.min(nx - 1, Math.max(0, Math.floor((x - minX) / dx)));
    const cy = Math.min(ny - 1, Math.max(0, Math.floor((y - minY) / dy)));
    return cy * nx + cx;
  };
  for (let t = 0; t < triCount; t++) {
    const a = idx[3 * t], b = idx[3 * t + 1], c = idx[3 * t + 2];
    const xs = [pos[3 * a], pos[3 * b], pos[3 * c]];
    const ys = [pos[3 * a + 1], pos[3 * b + 1], pos[3 * c + 1]];
    const x0 = Math.min(nx - 1, Math.max(0, Math.floor((Math.min(...xs) - minX) / dx)));
    const x1 = Math.min(nx - 1, Math.max(0, Math.floor((Math.max(...xs) - minX) / dx)));
    const y0 = Math.min(ny - 1, Math.max(0, Math.floor((Math.min(...ys) - minY) / dy)));
    const y1 = Math.min(ny - 1, Math.max(0, Math.floor((Math.max(...ys) - minY) / dy)));
    for (let cy = y0; cy <= y1; cy++)
      for (let cx = x0; cx <= x1; cx++) {
        const cell = cy * nx + cx;
        if (!buckets[cell]) buckets[cell] = [];
        buckets[cell].push(t);
      }
  }

  const pointInTri = (px, py, ax, ay, bx, by, cx, cy) => {
    const d1 = (px - bx) * (ay - by) - (ax - bx) * (py - by);
    const d2 = (px - cx) * (by - cy) - (bx - cx) * (py - cy);
    const d3 = (px - ax) * (cy - ay) - (cx - ax) * (py - ay);
    return (d1 >= 0 && d2 >= 0 && d3 >= 0) || (d1 <= 0 && d2 <= 0 && d3 <= 0);
  };

  return (x, y) => {
    const cell = cellOf(x, y);
    const list = buckets[cell];
    if (!list) return null;
    for (const t of list) {
      const a = idx[3 * t], b = idx[3 * t + 1], c = idx[3 * t + 2];
      const ax = pos[3 * a], ay = pos[3 * a + 1], az = pos[3 * a + 2];
      const bx = pos[3 * b], by = pos[3 * b + 1], bz = pos[3 * b + 2];
      const cx = pos[3 * c], cy = pos[3 * c + 1], cz = pos[3 * c + 2];
      if (!pointInTri(x, y, ax, ay, bx, by, cx, cy)) continue;
      const det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
      if (Math.abs(det) < 1e-20) continue;
      const l1 = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) / det;
      const l2 = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) / det;
      const l3 = 1 - l1 - l2;
      return l1 * az + l2 * bz + l3 * cz;
    }
    return null;
  };
}

// ------------------------------------------------------------- input overlays
function buildInputOverlays(doc) {
  const interp = doc.mesh?.ok ? doc.mesh.interpolation : null;
  const minZ = interp ? interp.min_elevation : 0;
  const maxZ = interp ? interp.max_elevation : 0;
  const heightAt = makeHeightSampler(doc);
  const fallback = heightAt ? null : minZ;
  const lift = Math.max((maxZ - minZ) * 0.0018, 1e-4);
  const at = (x, y) => {
    const z = heightAt ? heightAt(x, y) : null;
    return (z ?? fallback ?? minZ) + lift * 2;
  };
  const dz = Math.max((maxZ - minZ) * 0.004, 1e-3);
  // With repairs applied the effective (derived) geometry is what the mesh was
  // built from; the untouched original is drawn dashed for comparison.
  const eff = doc.has_repairs && doc.derived ? doc.derived : doc.original;
  const showOriginalOutline = doc.has_repairs && doc.derived;

  const makeLine = (points, closed, color, opacity, dashed = false, parent = groups.contours) => {
    if (!points || points.length < 2) return;
    const pts = points.map((p) => new THREE.Vector3(p[0], p[1], at(p[0], p[1])));
    if (closed && pts.length > 2) pts.push(pts[0].clone());
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    let mat;
    if (dashed) {
      mat = new THREE.LineDashedMaterial({ color, dashSize: dz * 30, gapSize: dz * 18, opacity });
      const line = new THREE.Line(geo, mat);
      line.computeLineDistances();
      parent.add(line);
      return;
    }
    mat = new THREE.LineBasicMaterial({ color, opacity });
    parent.add(new THREE.Line(geo, mat));
  };

  // Reference plane / grid at the data minimum.
  const box = new THREE.Box3();
  const all = [];
  for (const p of doc.original.boundary || []) all.push(p);
  for (const h of doc.original.holes || []) for (const p of h) all.push(p);
  for (const c of doc.original.contours || []) for (const p of c.points) all.push(p);
  for (const p of all) box.expandByPoint(new THREE.Vector3(p[0], p[1], 0));
  const span = box.isEmpty()
    ? 1
    : Math.max(box.getSize(new THREE.Vector3()).x, box.getSize(new THREE.Vector3()).y, 1e-6);
  if (!box.isEmpty()) {
    const center = box.getCenter(new THREE.Vector3());
    const gridSize = span * 1.25;
    const div = 20;
    const grid = new THREE.GridHelper(gridSize, div, 0x24313f, 0x18222d);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(center.x, center.y, minZ - dz * 6);
    grid.material.transparent = true;
    grid.material.opacity = 0.4;
    groups.ref.add(grid);
  }

  const contourColors = [0xffd166, 0xff9f4a, 0x9be07a, 0x6fd3e0, 0xb79cff, 0xff7fb0];
  (eff.contours || []).forEach((c, i) => {
    const color = contourColors[i % contourColors.length];
    makeLine(c.points, c.closed, color, 1.0, false, groups.contours);
    // Elevation label at a representative point of every contour.
    const pts = c.points;
    if (!pts.length) return;
    const atPt = pts[Math.floor(pts.length / 2)];
    addLabel(`z=${fmt(c.elevation, 2)}${c.derived_from >= 0 ? ' *' : ''}`,
      atPt[0], atPt[1], at(atPt[0], atPt[1]), 'contour-label', color);
  });

  // Original (pre-repair) contours, dashed grey, so the audit trail is visible.
  if (showOriginalOutline) {
    for (const c of doc.original.contours || []) {
      if ((doc.derived.contours || []).some((d) => d.id === c.id)) continue;
      makeLine(c.points, c.closed, 0x6e7c8e, 1.0, true, groups.original);
    }
  }

  // Boundary and holes drawn explicitly.
  makeLine(toRing(eff.boundary || []), true, 0x5fa8ff, 1.0, false, groups.contours);
  for (const h of eff.holes || [])
    makeLine(toRing(h), true, 0xb07cff, 1.0, false, groups.holes);

  // Faults / breaklines: not contours, drawn in red and labelled.
  for (const f of eff.faults || []) {
    makeLine(f.points, false, 0xff6b6b, 1.0, false, groups.contours);
    const p = f.points[0];
    addLabel(`breakline ${f.name || f.id}`, p[0], p[1], at(p[0], p[1]), 'fault-label', 0xff6b6b);
  }

  // Control points: small markers at their supplied elevation.
  for (const cp of eff.controls || []) {
    const x = cp.x ?? cp.position?.[0];
    const y = cp.y ?? cp.position?.[1];
    const z = cp.elevation;
    const geo = new THREE.SphereGeometry(Math.max(span * 0.0035, 1e-3), 12, 8);
    const mat = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const sphere = new THREE.Mesh(geo, mat);
    sphere.position.set(x, y, z);
    groups.controls.add(sphere);
    addLabel(`${cp.name || 'cp'}: ${fmt(z, 2)}`, x, y, z + dz * 5, 'control-label', 0xffffff);
  }
}

function addLabel(text, x, y, z, cls, color) {
  const div = el('div', 'label ' + (cls || ''), text);
  if (color) div.style.color = '#' + new THREE.Color(color).getHexString();
  div.style.position = 'absolute';
  div.style.transform = 'translate(-50%,-50%)';
  div.style.pointerEvents = 'none';
  div.style.fontSize = '10.5px';
  div.style.fontFamily = 'ui-monospace, monospace';
  div.style.textShadow = '0 0 4px #000, 0 1px 2px rgba(0,0,0,.9)';
  div.style.whiteSpace = 'nowrap';
  div.style.background = 'rgba(8,12,17,.62)';
  div.style.padding = '0 3px';
  div.style.borderRadius = '3px';
  div.dataset.x = x;
  div.dataset.y = y;
  div.dataset.z = z;
  document.getElementById('view').parentNode.appendChild(div);
  labelEls.push(div);
}

// ------------------------------------------------------------- issue markers
function buildIssueMarkers(doc) {
  const issues = (doc.validation?.issues || []).filter(
    (i) => (i.positions || []).length && (state.issueFilter ? true : i.severity !== 'info'));
  const base = doc.mesh?.ok ? doc.mesh.interpolation.max_elevation : 0;
  const dz = Math.max(Math.abs(base) * 0.01, 1);
  for (const issue of issues) {
    for (const p of issue.positions) {
      const color = issue.severity === 'error' ? 0xf85149 : issue.severity === 'warning' ? 0xd29922 : 0x4fa3ff;
      const geo = new THREE.SphereGeometry(Math.max(dz * 0.4, 0.4), 10, 8);
      const mat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 });
      const marker = new THREE.Mesh(geo, mat);
      marker.position.set(p[0], p[1], base + dz);
      marker.userData.issue = issue;
      groups.issues.add(marker);
    }
  }
}

// ------------------------------------------------------------- render loop
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const tooltip = document.getElementById('tooltip');
let pointerInside = false;
let hoverTimer = 0;

function updateLabels() {
  const canvas = renderer.domElement;
  const rect = canvas.getBoundingClientRect();
  const v = new THREE.Vector3();
  for (const div of labelEls) {
    v.set(+div.dataset.x, +div.dataset.y, +div.dataset.z);
    root.localToWorld(v);
    v.project(camera);
    const x = (v.x * 0.5 + 0.5) * rect.width;
    const y = (-v.y * 0.5 + 0.5) * rect.height;
    const visible = labelsVisible && v.z < 1;
    div.style.display = visible ? 'block' : 'none';
    div.style.left = x + 'px';
    div.style.top = y + 'px';
  }
}

function onPointerMove(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  pointerInside = true;
  pointer.clientX = event.clientX;
  pointer.clientY = event.clientY;
}

renderer.domElement.addEventListener('pointermove', onPointerMove);
renderer.domElement.addEventListener('pointerleave', () => {
  pointerInside = false;
  tooltip.hidden = true;
});

function pick() {
  if (!pointerInside) return;
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects([...groups.surface.children, ...groups.issues.children], false);
  if (!hits.length) {
    tooltip.hidden = true;
    return;
  }
  const hit = hits[0];
  const issue = hit.object.userData.issue;
  if (issue) {
    tooltip.textContent = `${issue.severity.toUpperCase()} · ${issue.code}: ${issue.message}`;
    tooltip.style.maxWidth = '420px';
    tooltip.style.whiteSpace = 'normal';
  } else {
    // Surface: report the interpolated elevation and provenance at the hit.
    const point = hit.point;
    const ex = state.exaggeration || 1;
    tooltip.textContent =
      `x ${fmt(point.x, 2)}  y ${fmt(-point.z, 2)}  z ${fmt(point.y / ex, 2)}`;
    tooltip.style.whiteSpace = 'nowrap';
    tooltip.style.maxWidth = 'none';
  }
  tooltip.hidden = false;
  tooltip.style.left = pointer.clientX + 14 + 'px';
  tooltip.style.top = pointer.clientY + 12 + 'px';
}

function resize() {
  const parent = renderer.domElement.parentElement;
  const w = parent.clientWidth;
  const h = parent.clientHeight;
  if (w === state.canvasWidth && h === state.canvasHeight) return;
  state.canvasWidth = w;
  state.canvasHeight = h;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}

function animate() {
  resize();
  controls.update();
  updateLabels();
  if (++hoverTimer % 3 === 0) pick();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

// ------------------------------------------------------------- panels
function renderMeta() {
  const panel = document.getElementById('metaPanel');
  panel.innerHTML = '';
  const d = state.doc;
  if (!d) return;
  const meta = d.original.meta;
  kv(panel, 'dataset', d.original.name || '—');
  kv(panel, 'XY unit', `${meta.xy_unit} (${fmt(meta.xy_meters_per_unit, 4)} m per unit)`);
  kv(panel, 'elevation unit', `${meta.z_unit} (${fmt(meta.z_meters_per_unit, 4)} m per unit)`);
  kv(panel, 'elevation datum', meta.z_datum || 'not declared', meta.z_datum ? '' : 'warn');
  if (meta.coordinate_note) kv(panel, 'note', meta.coordinate_note);
  kv(panel, 'boundary', `${(d.original.boundary || []).length} vertices`);
  kv(panel, 'holes', `${(d.original.holes || []).length}`);
  kv(panel, 'contours', `${(d.original.contours || []).length} lines`);
  kv(panel, 'control points', `${(d.original.controls || []).length}`);
  kv(panel, 'breaklines', `${(d.original.faults || []).length}`);
}

function renderResult() {
  const panel = document.getElementById('resultPanel');
  const timing = document.getElementById('timingPanel');
  const badge = document.getElementById('meshBadge');
  panel.innerHTML = '';
  timing.innerHTML = '';
  const d = state.doc;
  if (!d) {
    badge.textContent = 'no mesh';
    return;
  }
  const m = d.mesh;
  if (!m || !m.ok) {
    badge.textContent = 'build failed';
    kv(panel, 'status', m?.error || d.error || 'no mesh', 'err');
    return;
  }
  badge.textContent =
    `${fmtCount(m.triangle_count)} faces · ${fmtCount(m.vertex_count)} vertices`;
  kv(panel, 'status', 'mesh built');
  kv(panel, 'faces', fmtCount(m.triangle_count));
  kv(panel, 'vertices', fmtCount(m.vertex_count));
  kv(panel, 'constrained edges', fmtCount(m.edge_count));
  kv(panel, 'holes (unfilled)', `${(m.holes || []).length}`);
  kv(panel, 'elevation range',
    `${fmt(m.interpolation.min_elevation, 2)} … ${fmt(m.interpolation.max_elevation, 2)} ${d.original.meta.z_unit}`);
  kv(panel, 'surface area', `${fmt(m.interpolation.total_area_m2, 1)} m²`);
  kv(panel, 'unsupported area',
    `${fmt(m.interpolation.unsupported_area_m2, 1)} m² ` +
    `(${fmt(100 * m.interpolation.unsupported_area_m2 / Math.max(m.interpolation.total_area_m2, 1e-9), 1)}%)`,
    m.interpolation.unsupported_area_m2 > 1e-9 ? 'warn' : '');
  kv(panel, 'output coordinate system', 'original XY + elevation (Z-up)');

  const pill = (label, value) => {
    const div = el('div', 'pill');
    const b = el('b', null, value);
    div.appendChild(b);
    div.appendChild(el('span', null, label));
    timing.appendChild(div);
  };
  pill('triangulation', fmt(m.triangulation_ms, 2) + ' ms');
  pill('elevation', fmt(m.elevation_ms, 2) + ' ms');
  pill('mesh total', fmt(m.total_ms, 2) + ' ms');
  pill('pipeline total', fmt(d.total_ms, 2) + ' ms');
}

function renderInterpolation() {
  const method = document.getElementById('methodPanel');
  const coverage = document.getElementById('coveragePanel');
  const sources = document.getElementById('sourcesPanel');
  method.innerHTML = '';
  coverage.textContent = '';
  sources.innerHTML = '';
  const d = state.doc;
  if (!d || !d.mesh?.ok) return;
  const is = d.mesh.interpolation;
  method.textContent = is.method;
  coverage.textContent = is.coverage_note;
  kv(sources, 'observed', fmtCount(is.observed_vertices));
  kv(sources, 'along constraint', fmtCount(is.edge_interpolated_vertices));
  kv(sources, 'harmonic (TIN interior)', fmtCount(is.face_interpolated_vertices));
  kv(sources, 'boundary / hole ring', fmtCount(is.boundary_interpolated_vertices));
  kv(sources, 'unobserved component', fmtCount(is.unobserved_vertices),
    is.unobserved_vertices ? 'err' : '');
  kv(sources, 'support radius', `${fmt(is.support_radius_m, 2)} m`);
  kv(sources, 'max distance to observation', `${fmt(is.max_distance_to_observation_m, 2)} m`);
}

function renderIssues() {
  const list = document.getElementById('issueList');
  const count = document.getElementById('issueCount');
  list.innerHTML = '';
  const d = state.doc;
  if (!d) return;
  const issues = d.validation?.issues || [];
  count.textContent = `${d.validation.errors}E ${d.validation.warnings}W`;
  if (!issues.length) {
    list.appendChild(el('div', 'empty', 'No issues found in the original data.'));
    return;
  }
  for (const issue of issues) {
    const div = el('div', 'issue ' + issue.severity);
    div.appendChild(el('div', 'code', issue.code +
      (issue.entity_id >= 0 ? ` · ${issue.entity_type} ${issue.entity_id}` : '')));
    div.appendChild(el('div', 'msg', issue.message));
    if (issue.positions && issue.positions.length)
      div.appendChild(el('div', 'pos', 'at ' + issue.positions
        .map((p) => `(${fmt(p[0], 2)}, ${fmt(p[1], 2)})`).join(' · ')));
    div.addEventListener('click', () => focusPosition(issue.positions?.[0]));
    list.appendChild(div);
  }
}

function focusPosition(p) {
  if (!p) return;
  controls.target.set(p[0], p[1], 0);
  controls.update();
}

function renderRepairs() {
  const wrap = document.getElementById('repairPanelWrap');
  const list = document.getElementById('repairList');
  list.innerHTML = '';
  const d = state.doc;
  const candidates = d?.repair_candidates || [];
  wrap.hidden = candidates.length === 0;
  if (!candidates.length) return;
  for (const c of candidates) {
    const row = el('label', 'repair');
    const check = document.createElement('input');
    check.type = 'checkbox';
    check.checked = state.selectedRepairs.has(c.id);
    check.addEventListener('change', () => {
      if (check.checked) state.selectedRepairs.add(c.id);
      else state.selectedRepairs.delete(c.id);
    });
    const body = el('div');
    body.appendChild(el('div', 'lbl', c.label + (c.recommended ? '  (recommended)' : '')));
    body.appendChild(el('div', 'det', c.detail || ''));
    body.appendChild(el('div', 'id', c.id));
    row.appendChild(check);
    row.appendChild(body);
    list.appendChild(row);
  }
}

function renderTolerance() {
  const panel = document.getElementById('tolerancePanel');
  const note = document.getElementById('toleranceNote');
  panel.innerHTML = '';
  const d = state.doc;
  if (!d || !d.tolerance) return;
  const t = d.tolerance;
  kv(panel, 'local origin', `(${fmt(t.origin_x, 3)}, ${fmt(t.origin_y, 3)})`);
  kv(panel, 'normalisation', `extent ${fmt(t.extent_m, 2)} m → 1`);
  kv(panel, 'max |coordinate|', `${fmt(t.max_abs_coordinate, 2)} ${t.xy_unit}`);
  kv(panel, 'snap tolerance', `${fmt(t.snap_tolerance_m, 4)} m`);
  kv(panel, 'elevation equality', `${fmt(t.z_tolerance, 3)} ${t.z_unit}`);
  kv(panel, 'short segment', `${fmt(t.short_segment_m, 3)} m`);
  kv(panel, 'P05 point spacing', `${fmt(t.spacing_p05_m, 3)} m`);
  kv(panel, 'precision floor', `${fmt(t.precision_floor_m, 3)} m`);
  if (t.precision_limited) kv(panel, 'precision limited', 'yes — data cannot resolve the requested tolerance', 'warn');
  note.textContent = t.explanation;
}

function renderExportNote() {
  const note = document.getElementById('exportNote');
  const repairs = state.selectedRepairs.size;
  note.textContent = repairs
    ? `Exports apply ${repairs} confirmed repair(s); the original document is still preserved in the pipeline JSON.`
    : 'Exports contain the real triangulated surface in the declared units.';
}

function renderAll() {
  renderMeta();
  renderResult();
  renderInterpolation();
  renderIssues();
  renderRepairs();
  renderTolerance();
  renderExportNote();
  renderAssetPanels();
  renderCurveEditor();
  document.getElementById('sourceText').value = state.text;
}

// ------------------------------------------------------------- scene assembly
function rebuildScene() {
  clearScene();
  const d = state.doc;
  if (!d) return;
  buildInputOverlays(d);
  if (d.mesh?.ok) buildSurface(d);
  buildIssueMarkers(d);
  applyToggles();

  // Frame the data in original coordinates (group maps to Y-up).
  const box = new THREE.Box3();
  for (const p of d.original.boundary || []) box.expandByPoint(new THREE.Vector3(p[0], p[1], 0));
  for (const c of d.original.contours || [])
    for (const p of c.points) box.expandByPoint(new THREE.Vector3(p[0], p[1], 0));
  if (d.mesh?.ok) {
    box.expandByPoint(new THREE.Vector3(0, 0, d.mesh.interpolation.min_elevation));
    box.expandByPoint(new THREE.Vector3(0, 0, d.mesh.interpolation.max_elevation));
  }
  if (!box.isEmpty()) {
    // Root maps (x, y, z) -> (x, z * ex, -y) via rotation about X.
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const ex = state.exaggeration;
    const shifted = new THREE.Box3(
      new THREE.Vector3(center.x - size.x / 2, (center.z - size.z / 2) * ex, -(center.y + size.y / 2)),
      new THREE.Vector3(center.x + size.x / 2, (center.z + size.z / 2) * ex, -(center.y - size.y / 2)),
    );
    frameBounds(shifted);
  }
}

function applyToggles() {
  const t = (id) => document.getElementById(id).checked;
  groups.surface.visible = t('showSurface');
  groups.wire.visible = t('showWire');
  groups.contours.visible = t('showContours');
  groups.controls.visible = t('showControls');
  groups.constraints.visible = t('showConstraints');
  groups.issues.visible = t('showIssues');
  groups.ref.visible = t('showRef');
  groups.holes.visible = t('showHoles');
  groups.original.visible = t('showOriginal');
  labelsVisible = t('showLabels');
}

for (const id of ['showSurface', 'showWire', 'showContours', 'showLabels',
  'showControls', 'showConstraints', 'showIssues', 'showRef', 'showHoles', 'showOriginal'])
  document.getElementById(id).addEventListener('change', applyToggles);

document.getElementById('exaggeration').addEventListener('input', (event) => {
  state.exaggeration = parseFloat(event.target.value);
  document.getElementById('exaggerationValue').textContent = state.exaggeration.toFixed(1);
  // Vertical exaggeration scales the data z axis only.
  root.scale.set(1, 1, state.exaggeration);
});

// ------------------------------------------------------------- server calls
async function loadSamples() {
  const select = document.getElementById('sampleSelect');
  try {
    const res = await fetch('/api/samples');
    const names = await res.json();
    select.innerHTML = '';
    for (const name of names) {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name.replace(/\.json$/, '');
      select.appendChild(opt);
    }
    const preferred = names.find((n) => n.includes('closed_hill')) || names[0];
    if (preferred) select.value = preferred;
  } catch (e) {
    console.error(e);
  }
}

async function loadSample(name) {
  const res = await fetch('/api/sample/' + encodeURIComponent(name));
  if (!res.ok) {
    setStatus('cannot load sample', 'err');
    return;
  }
  state.text = await res.text();
  state.selectedRepairs.clear();
  await build();
}

async function build() {
  if (!state.text.trim()) {
    setStatus('no input', 'err');
    return;
  }
  setStatus('building…', 'busy');
  const button = document.getElementById('runBtn');
  button.disabled = true;
  try {
    const res = await fetch('/api/run', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        input_text: state.text,
        repairs: [...state.selectedRepairs],
      }),
    });
    const doc = await res.json();
    state.doc = doc;
    rebuildScene();
    renderAll();
    if (doc.error) setStatus(doc.error, 'err');
    else if (doc.mesh?.ok) setStatus('mesh ready', '');
    else setStatus('build failed', 'err');
  } catch (e) {
    console.error(e);
    setStatus('request failed', 'err');
  } finally {
    button.disabled = false;
  }
}

function setStatus(text, cls) {
  const s = document.getElementById('status');
  s.textContent = text;
  s.className = 'status ' + (cls || '');
}

async function exportFormat(format) {
  setStatus('exporting ' + format + '…', 'busy');
  try {
    const res = await fetch('/api/export', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        input_text: state.text,
        repairs: [...state.selectedRepairs],
        format,
      }),
    });
    if (!res.ok) {
      const err = await res.json().catch(() => ({}));
      setStatus(err.error || 'export failed', 'err');
      return;
    }
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const base = (state.doc?.original?.name || 'terrain').replace(/[^\w.-]+/g, '_');
    const ext = format === 'pipeline' ? 'json' : format;
    a.href = url;
    a.download = `${base}_${format === 'json' ? 'mesh' : format}.${ext}`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 2000);
    setStatus('exported ' + format, '');
  } catch (e) {
    console.error(e);
    setStatus('export failed', 'err');
  }
}

// ------------------------------------------------------------- terrain assets
// An asset is a generated contour document plus its provenance and verification
// record. Loading it puts the kernels' own input into the viewport; editing a
// contour's elevation rewrites that document and posts it to /api/run, so the
// rebuild always goes through the same constrained-TIN kernel as the CLI.
const assetState = {
  index: null,
  current: null,      // { id, assetText, entry, verification }
  baseline: '',       // asset document text at load time
  editText: '',       // current text after user edits
  editable: [],       // parsed contour rows for the editor
  loading: false,
};

async function loadAssetIndex() {
  const select = document.getElementById('assetSelect');
  try {
    const res = await fetch('/api/assets');
    if (!res.ok) throw new Error('assets index unavailable');
    assetState.index = await res.json();
    select.innerHTML = '';
    for (const entry of assetState.index.assets || []) {
      const opt = document.createElement('option');
      opt.value = entry.id;
      opt.textContent = entry.title || entry.id;
      select.appendChild(opt);
    }
    if (assetState.index.assets?.length) {
      select.value = assetState.index.assets[0].id;
    }
  } catch (e) {
    console.error(e);
    const opt = document.createElement('option');
    opt.textContent = 'no assets (run scripts/make_terrain_assets.py)';
    select.appendChild(opt);
  }
}

async function loadAsset(id) {
  if (!id) return;
  assetState.loading = true;
  setStatus('loading asset ' + id + '…', 'busy');
  try {
    const res = await fetch('/api/asset/' + encodeURIComponent(id));
    if (!res.ok) {
      const err = await res.json().catch(() => ({}));
      throw new Error(err.error || 'asset not found');
    }
    const reply = await res.json();
    assetState.current = {
      id,
      assetText: reply.asset_text,
      entry: reply.index_entry || {},
      verification: reply.verification || {},
    };
    assetState.baseline = reply.asset_text;
    assetState.editText = reply.asset_text;
    state.text = reply.asset_text;
    state.selectedRepairs.clear();
    parseEditableContours();
    await build();
    renderAssetPanels();
    if (state.doc?.ok === false || state.doc?.mesh?.ok === false) {
      showEditError(state.doc?.error || state.doc?.mesh?.error || 'asset did not build');
    } else {
      showEditError('');
    }
    setStatus('asset ' + id + ' loaded', '');
  } catch (e) {
    console.error(e);
    setStatus(String(e.message || e), 'err');
  } finally {
    assetState.loading = false;
  }
}

function parseEditableContours() {
  try {
    const doc = JSON.parse(assetState.editText);
    assetState.editable = (doc.contours || []).map((c) => ({
      id: c.id,
      name: c.name,
      closed: !!c.closed,
      elevation: c.elevation,
      originalElevation: c.elevation,
      dx: 0,
      dy: 0,
    }));
  } catch (e) {
    assetState.editable = [];
  }
}

function renderAssetPanels() {
  const provenance = document.getElementById('assetProvenance');
  const verifyPanel = document.getElementById('verifyPanel');
  const verifyNote = document.getElementById('verifyNote');
  const badge = document.getElementById('assetBadge');
  const verifyBadge = document.getElementById('verifyBadge');
  provenance.innerHTML = '';
  verifyPanel.innerHTML = '';
  verifyNote.textContent = '';
  const current = assetState.current;
  if (!current) {
    badge.textContent = '—';
    verifyBadge.textContent = '—';
    verifyBadge.className = 'count';
    return;
  }
  const entry = current.entry || {};
  badge.textContent = entry.id || current.id;
  const provDoc = (() => { try { return JSON.parse(current.assetText).provenance || {}; } catch { return {}; } })();
  kv(provenance, 'recipe', entry.recipe || provDoc.recipe || '—');
  kv(provenance, 'heightfield', provDoc.heightfield || '—');
  kv(provenance, 'generator', provDoc.generator || '—');
  kv(provenance, 'recipe hash', (entry.recipe_sha256 || '').slice(0, 12) + '…');
  kv(provenance, 'contours hash', (entry.contours_sha256 || '').slice(0, 12) + '…');
  kv(provenance, 'cases', (entry.cases || provDoc.cases || []).join(', ') || '—');
  kv(provenance, 'levels', (provDoc.levels_m || []).join(', ') || '—');

  const v = current.verification || {};
  if (!v.asset) {
    verifyBadge.textContent = 'n/a';
    verifyBadge.className = 'count';
    kv(verifyPanel, 'status', 'no verification record (run scripts/verify_terrain_assets.py)', 'warn');
    return;
  }
  const ok = v.ok === true;
  verifyBadge.textContent = ok ? 'pass' : 'FAIL';
  verifyBadge.className = 'count';
  const err = v.elevation_error || {};
  const con = v.constraints || {};
  kv(verifyPanel, 'status', ok ? 'verified' : (v.error || 'failed'), ok ? 'ok' : 'err');
  kv(verifyPanel, 'elevation error', `${fmt(err.worst_m, 6)} m (limit ${fmt(err.allowed_m, 4)} m)`, 'ok');
  kv(verifyPanel, 'constrained edges', `${con.total ?? 0} present, ${con.missing ?? 0} missing`,
    con.missing ? 'err' : 'ok');
  const kinds = Object.entries(con.by_kind || {}).map(([k, n]) => `${k} ${n}`).join(' · ');
  if (kinds) kv(verifyPanel, 'by kind', kinds);
  kv(verifyPanel, 'triangles', fmtCount(v.mesh?.triangles || 0));
  verifyNote.textContent = ok
    ? 'Elevation error is measured against the parsed heightfield; constrained edges are audited against the TIN edge set.'
    : (v.error || 'verification failed');
}

function renderCurveEditor() {
  const wrap = document.getElementById('curvePanel');
  const list = document.getElementById('curveList');
  const count = document.getElementById('curveCount');
  if (!assetState.current || state.activeTab !== 'assets') {
    wrap.hidden = true;
    return;
  }
  wrap.hidden = false;
  list.innerHTML = '';
  count.textContent = `${assetState.editable.length}`;
  const palette = ['#ffd166', '#ff9f4a', '#9be07a', '#6fd3e0', '#b79cff', '#ff7fb0'];
  assetState.editable.forEach((row, index) => {
    const div = el('div', 'curve-row');
    div.style.setProperty('--curve-color', palette[index % palette.length]);
    if (row.elevation !== row.originalElevation || row.dx || row.dy) div.classList.add('changed');
    const left = el('div');
    left.appendChild(el('div', 'curve-name', row.name || `contour ${row.id}`));
    const offsets = row.dx || row.dy ? ` · shift ${row.dx ? row.dx.toFixed(1) : '0'},${row.dy ? row.dy.toFixed(1) : '0'} m` : '';
    left.appendChild(el('div', 'curve-kind',
      `${row.closed ? 'closed' : 'open'} · z ${row.originalElevation}` +
      `${row.elevation !== row.originalElevation ? ' → ' + row.elevation : ''}${offsets}`));
    const fields = el('div', 'curve-fields');
    const zInput = document.createElement('input');
    zInput.type = 'number';
    zInput.step = 'any';
    zInput.title = 'elevation';
    zInput.value = row.elevation;
    zInput.addEventListener('change', () => {
      const value = parseFloat(zInput.value);
      if (!Number.isFinite(value)) {
        showEditError('elevation must be a number');
        return;
      }
      assetState.editable[index].elevation = value;
      renderCurveEditor();
    });
    const dxInput = document.createElement('input');
    dxInput.type = 'number';
    dxInput.step = 'any';
    dxInput.title = 'plan shift X (m)';
    dxInput.value = row.dx;
    dxInput.addEventListener('change', () => {
      assetState.editable[index].dx = parseFloat(dxInput.value) || 0;
      renderCurveEditor();
    });
    const dyInput = document.createElement('input');
    dyInput.type = 'number';
    dyInput.step = 'any';
    dyInput.title = 'plan shift Y (m)';
    dyInput.value = row.dy;
    dyInput.addEventListener('change', () => {
      assetState.editable[index].dy = parseFloat(dyInput.value) || 0;
      renderCurveEditor();
    });
    fields.appendChild(zInput);
    fields.appendChild(dxInput);
    fields.appendChild(dyInput);
    div.appendChild(left);
    div.appendChild(fields);
    list.appendChild(div);
  });
}

function applyCurveEdits() {
  if (!assetState.current) return;
  let doc;
  try {
    doc = JSON.parse(assetState.baseline);
  } catch (e) {
    showEditError('cannot parse the asset document');
    return;
  }
  const edits = [];
  for (const row of assetState.editable) {
    const contour = (doc.contours || []).find((c) => c.id === row.id);
    if (!contour) continue;
    if (row.elevation !== row.originalElevation) {
      contour.elevation = row.elevation;
      contour.name = `z=${row.elevation}`;
      edits.push({ contour_id: row.id, kind: 'elevation', from: row.originalElevation, to: row.elevation });
    }
    if (row.dx || row.dy) {
      // Translate the curve in plan; whether the move is legal is the
      // kernel's call (leaving the boundary and crossings are validated).
      contour.points = contour.points.map((p) => [p[0] + row.dx, p[1] + row.dy]);
      edits.push({ contour_id: row.id, kind: 'offset', dx: row.dx, dy: row.dy });
    }
  }
  doc.provenance = doc.provenance || {};
  doc.provenance.user_edits = edits;
  assetState.editText = JSON.stringify(doc, null, 2);
  state.text = assetState.editText;
}

function showEditError(message) {
  const box = document.getElementById('editError');
  if (!message) {
    box.hidden = true;
    box.textContent = '';
    return;
  }
  box.hidden = false;
  box.textContent = message;
}

function resetCurves() {
  if (!assetState.current) return;
  assetState.editText = assetState.baseline;
  state.text = assetState.baseline;
  parseEditableContours();
  renderCurveEditor();
  showEditError('');
  renderAssetPanels();
}

async function rebuildFromEdits() {
  if (!assetState.current) return;
  const changed = assetState.editable.some(
    (row) => row.elevation !== row.originalElevation || row.dx || row.dy);
  if (!changed) {
    showEditError('');
    setStatus('no edits to rebuild', '');
    return;
  }
  applyCurveEdits();
  setStatus('rebuilding from edited contours…', 'busy');
  // Remember the last good result. build() replaces state.doc even when the
  // kernel rejects the document; for an interactive edit we keep the standing
  // mesh instead of clearing the viewport, and surface the rejection verbatim.
  // The rejected text stays in the source box so the user can see and fix it.
  const previousDoc = state.doc;
  await build();
  if (state.doc?.mesh?.ok) {
    showEditError('');
    setStatus('rebuilt · ' + fmtCount(state.doc.mesh.triangle_count) + ' faces', '');
  } else {
    const message = state.doc?.error || state.doc?.mesh?.error || 'rebuild failed';
    state.doc = previousDoc;
    rebuildScene();
    renderAll();
    document.getElementById('sourceText').value = assetState.editText;
    showEditError(message
      + '\nA contour edit the kernel rejects is not applied; the previous mesh stays on screen.');
    setStatus('rebuild rejected by kernel', 'err');
  }
  renderAssetPanels();
}

function setActiveTab(tab) {
  state.activeTab = tab;
  for (const button of document.querySelectorAll('.tab')) {
    const active = button.dataset.tab === tab;
    button.classList.toggle('active', active);
    button.setAttribute('aria-selected', active ? 'true' : 'false');
  }
  document.querySelector('.asset-panel').hidden = tab !== 'assets';
  document.getElementById('verifyPanel').closest('.panel').hidden = tab !== 'assets';
  document.getElementById('curvePanel').hidden = tab !== 'assets' || !assetState.current;
}

// ------------------------------------------------------------- wiring
document.getElementById('runBtn').addEventListener('click', build);
document.getElementById('reloadSample').addEventListener('click', () => {
  const name = document.getElementById('sampleSelect').value;
  if (name) loadSample(name);
});
document.getElementById('loadSource').addEventListener('click', () => {
  state.text = document.getElementById('sourceText').value;
  state.selectedRepairs.clear();
  build();
});
document.getElementById('loadAsset').addEventListener('click', () => {
  const id = document.getElementById('assetSelect').value;
  if (id) loadAsset(id);
});
document.getElementById('rebuildFromEdits').addEventListener('click', rebuildFromEdits);
document.getElementById('resetCurves').addEventListener('click', resetCurves);
for (const button of document.querySelectorAll('.tab'))
  button.addEventListener('click', () => setActiveTab(button.dataset.tab));
for (const button of document.querySelectorAll('[data-format]'))
  button.addEventListener('click', () => exportFormat(button.dataset.format));
document.getElementById('exportPipeline').addEventListener('click', () => exportFormat('pipeline'));

// ------------------------------------------------------------- boot
(async function boot() {
  setActiveTab('samples');
  document.querySelector('.asset-panel').hidden = true;
  await loadSamples();
  const name = document.getElementById('sampleSelect').value;
  if (name) await loadSample(name);
  await loadAssetIndex();
  animate();
})();

window.__terrain = {
  state, build, exportFormat, groups, camera, controls,
  assetState, loadAsset, rebuildFromEdits, resetCurves, loadAssetIndex,
};
