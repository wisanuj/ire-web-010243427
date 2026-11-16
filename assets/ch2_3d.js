/* ch2_3d.js — the 3-D scenes of chapter 2: frames, rotation matrices, mappings, operators, compound and inverse transforms,
   transform equations and the representations of orientation. CH2.S3.name(state, {dark}) returns a scene table (scene3d.js).
   The pages draw the tables live with WebGL (CH2.view, bottom of this file); IRE_Slides/build/coppelia/gen_scenes_ch2.mjs renders the
   same tables in CoppeliaSim for the slides. Colours are fixed for the course: X red, Y green, Z blue; the reference frame is grey.
   Needs robot.js, ch2.js, scene3d.js. World: metres, z up. */
(function (global) {
'use strict';
const RB = global.RB, S3 = global.S3, CH2 = global.CH2, NUM = global.NUM, C = S3.C, { sin, cos, PI, abs, max, min, acos, hypot } = Math, rad = RB.rad, v = S3.v;
const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
const { bar, cyl, sph, box, arrow, tube, dashed, floor, lab, camAt, scene, shadow, mesh, rj, pedestal, grip } = S3;
const SC = CH2.S3 = {}, I = RB.I3(), O0 = [0, 0, 0], AX = ['X', 'Y', 'Z'], AK = ['x', 'y', 'z'];
const view = (tgt, dist, az, el, fov = 28) => camAt(tgt, dist, az, el, fov);
/* the camera of an orbitable scene: st.az / st.el when the page has turned the view, else the scene's own; the angles used are kept in st._az, st._el */
/* the reader's own turning and zooming (S3.look: st.vaz, st.vel, st.vzoom) is added here, so that a fitted camera fits the view actually shown */
const lookAt = (st, az, el) => { const u = S3.look(st); return { a: (st.az === undefined ? az : st.az) + u.az, e: max(-20, min(89, (st.el === undefined ? el : st.el) + u.el)), z: u.zoom }; };
const cam = (st, tgt, dist, az, el) => { const { a, e, z } = lookAt(st, az, el); st._az = a; st._el = e; const c = view(st.tgt || tgt, (st.dist || dist) / z, a, e); c.user = true; return c; };
/* the same, but framed on the points pts (S3.fitCam): for arms the reader moves anywhere with sliders */
const camFit = (st, pts, az, el, w, h, o) => { const { a, e, z } = lookAt(st, az, el); st._az = a; st._el = e; const f = S3.fitCam(st, pts, a, e, 28, w, h, o), c = z === 1 ? f : S3.camAt(f.tgt, f.dist / z, a, e, 28); c.user = true; return c; };
const grey = s => s.dark ? [0.66, 0.7, 0.8] : [0.4, 0.45, 0.54];
const axc = (s, j) => [C.x, C.y, s.pal.z][j];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const PART = [0.6, 0.68, 0.8];

/* ---------------- building blocks ---------------- */
/* a frame at O with rotation R (columns = its unit axes in world coordinates). op: name (label subscript), ref (grey reference frame),
   L arrow length, r radius, tag ({A} written at the origin), hl (axis drawn thicker), al (ghost), lower (x, y, z), lab[j] (label tweaks) */
const frame = CH2.frame = (s, R, O, op = {}) => { const L = op.L || 0.42, r = op.r || 0.0068, g = op.ref ? grey(s) : null;
  for (let j = 0; j < 3; j++) { if (op.only && !op.only.includes(j)) continue; const d = RB.col(R, j), e = v(O, d, L), c = op.mono || g || axc(s, j);
    arrow(s, O, e, c, op.hl === j ? r * 1.7 : r, op.al);
    if (op.name !== undefined && op.labels !== false && !(op.al < 0.6)) lab(s, v(O, d, L + (op.lo || 0.065)), op.lower ? AK[j] : AX[j], { sub: op.name, m: 1, c: op.ref ? 'mut' : AK[j], s: op.ls || 34, a: 'c', dy: 12, ...((op.lab && op.lab[j]) || {}) }); }
  sph(s, O, r * 2.3, g || s.pal.ink, op.al);
  if (op.tag) lab(s, O, '{' + op.name + '}', { c: op.ref ? 'mut' : 'ink', s: op.ts || 32, a: 'r', dx: -16, dy: 38, ...(op.tagAt || {}) }); };
/* a vector drawn as an arrow, labelled at a fraction `at` of its length */
const vec = (s, a, b, c, op = {}) => { if (hypot(...sub(b, a)) < 1e-6) return; arrow(s, a, b, c, op.r || 0.0075, op.al);
  if (op.t) lab(s, v(a, sub(b, a), op.at === undefined ? 0.55 : op.at), op.t, { m: 1, s: op.s || 36, c: op.key, sub: op.sub, pre: op.pre, post: op.post, dx: op.dx || 0, dy: op.dy || 0, a: op.a || 'c' }); };
/* a dashed arrow (a transform that is asked for, or a ghost) */
const dvec = (s, a, b, c, op = {}) => { const d = sub(b, a), n = hypot(...d); if (n < 1e-6) return; const h = min(0.06, n * 0.3); dashed(s, a, v(a, d, 1 - h / n), c, op.r || 0.005, 15); arrow(s, v(a, d, 1 - h / n - 0.002), b, c, op.r || 0.005, op.al);
  if (op.t) lab(s, v(a, d, op.at === undefined ? 0.5 : op.at), op.t, { m: 1, s: op.s || 36, c: op.key, sub: op.sub, pre: op.pre, post: op.post, dx: op.dx || 0, dy: op.dy || 0, a: op.a || 'c' }); };
/* an arc of radius r about O from direction u to direction w (the angle between two axes); returns the middle of the arc */
const angArc = CH2.angArc = (s, O, u, w, r, c, tr = 0.0045, head = true) => { u = RB.unit(u); w = RB.unit(w); const th = acos(NUM.clamp(RB.dot(u, w), -1, 1)); if (th < 0.03) return null;
  const n = RB.unit(RB.cross(u, w)), b = RB.cross(n, u), N = max(8, Math.ceil(th * 22)), at = a => [O[0] + r * (u[0] * cos(a) + b[0] * sin(a)), O[1] + r * (u[1] * cos(a) + b[1] * sin(a)), O[2] + r * (u[2] * cos(a) + b[2] * sin(a))];
  tube(s, Array.from({ length: N + 1 }, (_, k) => at(th * k / N)), c, tr, head); return at(th / 2); };
/* a turn about an axis k (through O), drawn as an arrowed arc near the axis: shows the positive sense of a rotation */
const turn = (s, O, k, r, c, a0 = 0.3, a1 = 5.4, tr = 0.0042) => tube(s, RB.ring(O, k, r, a0, a1, 30), c, tr, true);
/* a block attached to a frame (the "body" whose orientation the frame describes) */
const part = (s, R, O, sz = [0.3, 0.2, 0.12], c = PART, al = 0.45) => { const Y = RB.col(R, 1), Z = RB.col(R, 2); box(s, v(O, Z, -sz[2] / 2), v(O, Z, sz[2] / 2), sz[0], sz[1], c, al, Y); };
/* a light grid on the X–Y plane of the reference frame, in the units of a numeric example (k metres per unit) */
const grid = (s, k, x0, x1, y0, y1, step = 1, nums) => { const c = s.dark ? [0.3, 0.35, 0.48] : [0.8, 0.83, 0.88];
  for (let x = Math.ceil(x0 / step) * step; x <= x1 + 1e-9; x += step) cyl(s, [x * k, y0 * k, -0.002], [x * k, y1 * k, -0.002], 0.0018, c);
  for (let y = Math.ceil(y0 / step) * step; y <= y1 + 1e-9; y += step) cyl(s, [x0 * k, y * k, -0.002], [x1 * k, y * k, -0.002], 0.0018, c);
  if (nums) { nums.x.forEach(x => lab(s, [x * k, 0, 0], String(x), { c: 'mut', s: 24, a: 'c', dy: 30, b: false })); nums.y.forEach(y => lab(s, [0, y * k, 0], String(y), { c: 'mut', s: 24, a: 'r', dx: -12, dy: 8, b: false })); } };
const fmtP = (p, d = 3) => `(${RB.u(p[0], d)}, ${RB.u(p[1], d)}${p.length > 2 && abs(p[2]) > 1e-9 ? ', ' + RB.u(p[2], d) : ''})`;

CH2.H = { frame, vec, dvec, angArc, turn, part, grid, view, cam, camFit, grey, axc };   // shared with the scenes of later chapters (ch3_3d.js …)

/* ================= page 2.1: positions, orientations, frames ================= */
/* a point and its position vector ᴬP = (p_x, p_y, p_z) */
SC.point = (st = {}, o = {}) => { const p = st.p || [0.42, 0.56, 0.42], s = scene(st.name || 'point', 1200, 1000, cam(st, [0.2, 0.25, 0.2], 3.1, 28, 22), o); floor(s, 0.95, [0.2, 0.25]);
  frame(s, I, O0, { name: 'A', L: 0.78, tag: true }); const q = [p[0], p[1], 0];
  dashed(s, p, q); dashed(s, q, [p[0], 0, 0]); dashed(s, q, [0, p[1], 0]);
  vec(s, O0, p, C.pk, { key: 'pk', t: 'P', pre: 'A', at: 0.5, dx: -26, dy: -6, a: 'r' }); sph(s, p, 0.024, C.pk);
  lab(s, [p[0], 0, 0], 'p', { sub: 'x', m: 1, c: 'x', s: 34, a: 'c', dy: 44 }); lab(s, [0, p[1], 0], 'p', { sub: 'y', m: 1, c: 'y', s: 34, a: 'c', dy: 44 }); lab(s, [p[0], p[1], p[2] * 0.72], 'p', { sub: 'z', m: 1, c: 'z', s: 34, dx: 16, dy: 10 });
  lab(s, p, st.num ? fmtP(st.num, 2) : 'P', { m: !st.num, c: 'pk', s: st.num ? 28 : 36, dx: 18, dy: -12 }); return s; };

/* frame {B} turned relative to {A}, same origin. st: R or (al, be, ga); body: a block on {B}; proj: j draws the projections of axis j of {B}
   onto the axes of {A} (its column of the rotation matrix); arcZ: the angle θ between X_A, X_B and Y_A, Y_B (a turn about Z) */
SC.orient = (st = {}, o = {}) => { const R = st.R || RB.Rzyx(st.al || 0, st.be || 0, st.ga || 0), s = scene(st.name || 'orient', st.w || 1200, st.h || 1000, cam(st, [0, 0, 0.1], 3.1, 28, 22), o); floor(s, 0.95);
  if (st.body) part(s, R, O0, [0.34, 0.22, 0.12]);
  frame(s, I, O0, { name: 'A', ref: true, L: 0.66, r: 0.0055, ls: 32 });
  frame(s, R, O0, { name: st.nb === undefined ? 'B' : st.nb, L: 0.5, hl: st.proj >= 0 ? st.proj : undefined, lab: st.labB });
  if (st.proj >= 0) { const j = st.proj, e = RB.sc(RB.col(R, j), 0.5), c = axc(s, j);
    for (let i = 0; i < 3; i++) { if (abs(e[i]) < 0.012) continue; const f = [0, 0, 0]; f[i] = e[i]; dashed(s, e, f, c, 0.003, 13); sph(s, f, 0.014, c); } }
  if (st.arcZ) { const r = 0.3; [0, 1].forEach(j => { const m = angArc(s, O0, RB.col(I, j), RB.col(R, j), r, s.pal.ink); if (m) lab(s, m, 'θ', { m: 1, s: 38, a: 'c', dx: j ? 18 : 0, dy: j ? 6 : 34 }); }); }
  return s; };

/* the three turns about the principal axes, side by side */
SC.rot3 = (st = {}, o = {}) => { const th = st.th === undefined ? rad(40) : st.th, c3 = view([0, 0, 0.12], 7.4, 28, 18), s = scene('rot3', 1800, 660, c3, o), A = S3.axes(c3), right = [-A.x[0], -A.x[1], 0];
  [RB.Rx, RB.Ry, RB.Rz].forEach((f, i) => { const P = v(O0, right, (i - 1) * 1.3), R = f(th), k = RB.col(I, i), a = RB.col(I, (i + 1) % 3); floor(s, 0.55, [P[0], P[1]]);
    frame(s, I, P, { name: 'A', ref: true, L: 0.55, r: 0.005, ls: 28 }); frame(s, R, P, { name: 'B', L: 0.44, ls: 28, lab: i === 0 ? [{ dy: 34 }] : undefined });
    const m = angArc(s, P, a, RB.col(R, (i + 1) % 3), 0.27, s.pal.ink); if (m) lab(s, m, 'θ', { m: 1, s: 32, a: 'c', dx: 16, dy: -4 });
    lab(s, v(P, [0, 0, -1], 0.02), 'R', { sub: AX[i], post: '(θ)', m: 1, s: 44, a: 'c', dy: 150 }); }); return s; };

/* the description of a frame: {B} = {ᴬ_B R, ᴬP_BORG}; body: the block {B} is attached to; pts3: three marked points of the body */
SC.desc = (st = {}, o = {}) => { const R = st.R || RB.Rzyx(rad(35), rad(-12), rad(18)), p = st.p || [-0.05, 0.85, 0.32], s = scene(st.name || 'desc', 1300, 1000, cam(st, [0.02, 0.45, 0.22], 3.6, 28, 24), o); floor(s, 1.2, [0, 0.45]);
  frame(s, I, O0, { name: 'A', ref: true, L: 0.5, tag: true });
  if (st.body !== false) part(s, R, p, [0.4, 0.26, 0.14]);
  frame(s, R, p, { name: 'B', L: 0.36, tag: true, tagAt: { dx: -24, dy: 46 } });
  vec(s, O0, p, C.am, { key: 'am', t: 'P', pre: 'A', sub: 'BORG', at: 0.5, dx: 40, dy: 34, a: 'l' });
  if (st.pts3) { const X = RB.col(R, 0), Y = RB.col(R, 1), Z = RB.col(R, 2), corner = (a, b, c) => v(v(v(p, X, a * 0.2), Y, b * 0.13), Z, c * 0.07);
    [[1, 1, 1], [-1, 1, 1], [1, -1, 1]].forEach((q, i) => { const P = corner(...q); sph(s, P, 0.018, C.pk); lab(s, P, 'P', { sub: String(i + 1), m: 1, c: 'pk', s: 32, dx: i === 1 ? -14 : 14, dy: -12, a: i === 1 ? 'r' : 'l' }); }); }
  return s; };

/* ================= page 2.2: mappings ================= */
/* one point P seen from {A} and from {B}. st: R and p (where {B} is, in units), bp (ᴮP in units), k (metres per unit), mode 'trans' | 'rot' | 'gen',
   grid [x0, x1, y0, y1, step] (numeric examples), num (write the coordinates of P) */
SC.map = (st = {}, o = {}) => { const k = st.k || 1, R = st.R || I, p = st.p || [0, 0, 0], bp = st.bp || [0.2, 0.3, 0.2], ap = RB.add(RB.mv(R, bp), p), K = x => RB.sc(x, k), B = K(p), P = K(ap), mode = st.mode || 'gen';
  const s = scene(st.name || 'map', st.w || 1300, st.h || 1000, cam(st, st.tgt0 || [0.25, 0.3, 0.15], st.dist0 || 3.4, 28, 24), o); floor(s, st.fr || 1.2, [B[0] / 2 + 0.1, B[1] / 2 + 0.15]);
  if (st.grid) { const [x0, x1, y0, y1, step] = st.grid; grid(s, k, x0, x1, y0, y1, step, st.nums); }
  const only = st.xy ? [0, 1] : undefined; frame(s, I, O0, { name: 'A', ref: true, L: st.LA || 0.5, tag: true, ls: st.ls, only });
  frame(s, R, B, { name: 'B', L: st.LB || 0.38, tag: mode !== 'rot', ls: st.ls, lab: st.labB, tagAt: st.tagB, only });
  sph(s, P, 0.02, C.pk);
  if (mode === 'rot') { vec(s, O0, P, C.pk, { key: 'pk', t: 'P', at: 0.62, dx: 24, dy: -8, a: 'l' });
    if (st.proj !== false) { for (let i = 0; i < 2; i++) { const f = [0, 0, 0]; f[i] = P[i]; if (abs(P[i]) > 0.01) dashed(s, P, f, grey(s), 0.003, 13); }
      [0, 1, 2].forEach(j => { const d = RB.col(R, j), t = RB.dot(P, d); if (abs(t) > 0.01 && abs(t) < hypot(...P) - 0.005) dashed(s, P, RB.sc(d, t), axc(s, j), 0.003, 13); }); } }
  else { vec(s, O0, B, C.am, { key: 'am', t: 'P', pre: 'A', sub: 'BORG', at: 0.5, ...(st.lborg || { dx: 30, dy: 40, a: 'l' }) });
    vec(s, B, P, C.p, { key: 'p', t: 'P', pre: 'B', at: 0.55, ...(st.lbp || { dx: 26, dy: -6, a: 'l' }) });
    vec(s, O0, P, C.pk, { key: 'pk', t: 'P', pre: 'A', at: 0.5, ...(st.lap || { dx: -26, dy: -6, a: 'r' }) }); }
  if (st.num) lab(s, P, st.num === true ? fmtP(ap) : st.num, { c: 'pk', s: 28, dx: 22, dy: -16 });
  return s; };

/* ================= page 2.3: operators, compound and inverse transforms, transform equations ================= */
/* a point moved inside one frame: kind 'rot' (turn by th about Z), 'trans' (shift by q) or 'gen' (turn, then shift) */
/* st.u: progress of the animation: 0…1 for 'rot' and 'trans'; 0…2 for 'gen' (0…1 the turn, 1…2 the shift) */
SC.oper = (st = {}, o = {}) => { const k = st.k || 1, kind = st.kind || 'gen', th = st.th === undefined ? rad(30) : st.th, q = st.q || [0, 0, 0], p1 = st.p1 || [0.3, 0.45, 0.2], K = x => RB.sc(x, k), u0 = st.u === undefined ? (kind === 'gen' ? 2 : 1) : st.u;
  const ur = kind === 'gen' ? min(1, u0) : u0, u = kind === 'gen' ? max(0, u0 - 1) : u0, R = st.R || RB.Rz(th * (kind === 'gen' ? ur : 1));
  const pr = kind === 'trans' ? p1 : RB.mv(R, p1), p2 = kind === 'rot' ? pr : RB.add(pr, q), P1 = K(p1), Pr = K(pr), P2 = K(p2);
  const s = scene(st.name || 'oper' + kind, st.w || 1300, st.h || 1000, cam(st, st.tgt0 || [0.25, 0.3, 0.12], st.dist0 || 3.4, 28, 26), o); floor(s, 1.2, [0.25, 0.3]);
  if (st.grid) { const [x0, x1, y0, y1, step] = st.grid; grid(s, k, x0, x1, y0, y1, step, st.nums); }
  frame(s, I, O0, { name: 'A', L: st.LA || 0.5, tag: true, only: st.xy ? [0, 1] : undefined });
  vec(s, O0, P1, C.pk, { key: 'pk', t: 'P', pre: 'A', sub: '1', at: 0.55, ...(st.l1 || { dx: -24, dy: -6, a: 'r' }) }); sph(s, P1, 0.02, C.pk);
  if (kind !== 'trans') { const ax = st.axis || [0, 0, 1], c0 = v(O0, ax, RB.dot(P1, ax)), r = hypot(...sub(P1, c0)), pts = [];
    if (r > 0.02 && th * (kind === 'gen' ? ur : u) !== 0) { const a = RB.unit(sub(P1, c0)), b = RB.cross(ax, a), ang = th * (kind === 'gen' ? ur : u); for (let i = 0; i <= 30; i++) { const t = ang * i / 30; pts.push(v(v(c0, a, r * cos(t)), b, r * sin(t))); } tube(s, pts, s.pal.ink, 0.0042, true); lab(s, pts[15], 'θ', { m: 1, s: 38, dx: 16, dy: -8 }); }
    if (kind === 'gen' && ur > 0.02) dvec(s, O0, Pr, s.pal.dash, { r: 0.0042 }); }
  if (kind === 'rot') { const Pu = K(RB.mv(RB.Rz(th * u), p1)); vec(s, O0, Pu, C.p, { key: 'p', t: 'P', pre: 'A', sub: '2', at: 0.55, ...(st.l2 || { dx: 26, dy: -6, a: 'l' }) }); sph(s, Pu, 0.02, C.p); }
  else { const Pa = kind === 'trans' ? P1 : Pr, Q = K(q), Pe = v(Pa, Q, u);
    if (kind === 'trans') vec(s, O0, Q, C.o, { key: 'o', t: 'Q', pre: 'A', at: 0.55, dx: 0, dy: 40 });
    if (kind === 'trans' || ur > 0.999) vec(s, Pa, Pe, C.o, { key: 'o', t: kind === 'trans' ? '' : 'Q', pre: 'A', at: 0.5, dx: 0, dy: -18 });
    if (u > 0.98) { vec(s, O0, P2, C.p, { key: 'p', t: 'P', pre: 'A', sub: '2', at: 0.62, ...(st.l2 || { dx: -26, dy: -8, a: 'r' }) }); sph(s, P2, 0.02, C.p); } }
  if (st.num) lab(s, P2, fmtP(p2), { c: 'p', s: 28, dx: 22, dy: -16 });
  return s; };

/* mapping and operator side by side: the same rotation matrix, two meanings */
SC.mapVsOp = (st = {}, o = {}) => { const th = st.th === undefined ? rad(35) : st.th, c3 = view([0, 0.12, 0.16], 5.5, 28, 22), s = scene('mapVsOp', 1800, 820, c3, o), A = S3.axes(c3), right = [-A.x[0], -A.x[1], 0], R = RB.Rz(th);
  const L0 = v(O0, right, -0.8), R0 = v(O0, right, 0.8), p = [0.05, 0.42, 0.18]; floor(s, 0.6, [L0[0], L0[1]]); floor(s, 0.6, [R0[0], R0[1]]);
  frame(s, I, L0, { name: 'A', ref: true, L: 0.55, r: 0.0055, ls: 28 }); frame(s, R, L0, { name: 'B', L: 0.45, ls: 28 }); const PL = RB.add(L0, RB.mv(R, p)); vec(s, L0, PL, C.pk, { key: 'pk', t: 'P', at: 0.6, dx: 22, a: 'l' }); sph(s, PL, 0.018, C.pk);
  const m = angArc(s, L0, [1, 0, 0], RB.col(R, 0), 0.25, s.pal.ink); if (m) lab(s, m, 'θ', { m: 1, s: 32, a: 'c', dy: 34 });
 
  frame(s, I, R0, { name: 'A', L: 0.55, r: 0.006, ls: 28 }); const P1 = RB.add(R0, p), P2 = RB.add(R0, RB.mv(R, p)); vec(s, R0, P1, C.pk, { key: 'pk', t: 'P', sub: '1', at: 0.6, dx: 24, dy: 10, a: 'l' }); vec(s, R0, P2, C.p, { key: 'p', t: 'P', sub: '2', at: 0.62, dx: -22, a: 'r' }); sph(s, P1, 0.018, C.pk); sph(s, P2, 0.018, C.p);
  const c0 = [R0[0], R0[1], R0[2] + p[2]], a = RB.unit(sub(P1, c0)), rr = hypot(...sub(P1, c0)), pts = []; for (let i = 0; i <= 24; i++) { const t = th * i / 24; pts.push(v(v(c0, a, rr * cos(t)), RB.cross([0, 0, 1], a), rr * sin(t))); } tube(s, pts, s.pal.ink, 0.004, true); lab(s, pts[12], 'θ', { m: 1, s: 32, dx: 6, dy: -14, a: 'c' });
  return s; };

/* three frames in a chain: ᴬ_B T, ᴮ_C T and their product ᴬ_C T. st.TAB, st.TBC in metres; inv: also the arrow ᴮP_AORG of the inverse */
SC.chain3 = (st = {}, o = {}) => { const TAB = st.TAB || RB.T(RB.Rz(rad(35)), [-0.05, 0.62, 0.18]), TBC = st.TBC, s = scene(st.name || 'chain3', st.w || 1300, st.h || 1000, cam(st, [0, 0.55, 0.28], 3.9, 28, 24), o); floor(s, 1.3, [0, 0.55]);
  const B = RB.Tp(TAB), only = st.xy ? [0, 1] : undefined; frame(s, I, O0, { name: 'A', ref: true, L: st.LA || 0.45, tag: true, only }); frame(s, RB.TR(TAB), B, { name: 'B', L: st.LB || 0.34, tag: true, tagAt: st.tagB || { dx: -16, dy: -18 }, only });
  if (!st.inv) vec(s, O0, B, C.am, { key: 'am', t: 'P', pre: 'A', sub: 'BORG', at: 0.5, ...(st.lab1 || { dx: 0, dy: 46, a: 'c' }) });
  if (TBC) { const TAC = RB.Tm(TAB, TBC), Cc = RB.Tp(TAC); frame(s, RB.TR(TAC), Cc, { name: 'C', L: 0.3, tag: true, tagAt: { dx: 20, dy: 44, a: 'l' } });
    vec(s, B, Cc, C.gr, { key: 'gr', t: 'P', pre: 'B', sub: 'CORG', at: 0.5, ...(st.lab2 || { dx: 30, dy: -10, a: 'l' }) });
    if (st.direct !== false) dvec(s, O0, Cc, C.pk, { key: 'pk', t: 'P', pre: 'A', sub: 'CORG', at: 0.3, ...(st.lab3 || { dx: -10, dy: -22, a: 'r' }) }); }
  if (st.inv) { const n = RB.unit(RB.cross(B, [0, 0, 1])), off = 0.03; vec(s, v(O0, n, off), v(B, n, off), C.am, { key: 'am', t: 'P', pre: 'A', sub: 'BORG', at: 0.45, dx: 30, dy: 44, a: 'l' }); vec(s, v(B, n, -off), v(O0, n, -off), C.b, { key: 'b', t: 'P', pre: 'B', sub: 'AORG', at: 0.45, dx: -26, dy: -14, a: 'r' }); }
  return s; };

/* homework 2.1: the wedge and its three frames (1 unit = u metres) */
SC.hw21 = (st = {}, o = {}) => { const u = 0.55, D = 0.3, s = scene('hw21', 1300, 1000, cam(st, [0.12, 0.28, 0.24], 3.3, 205, 18), o), E = CH2.EX.hw21, edge = s.dark ? [0.62, 0.66, 0.78] : [0.45, 0.5, 0.58]; floor(s, 0.9, [0.15, 0.28]);
  const P = [[0, 0, 0], [0, u, 0], [0, 0, u], [D, 0, 0], [D, u, 0], [D, 0, u]];
  mesh(s, [[P[0], P[1], P[4], P[3]], [P[0], P[3], P[5], P[2]], [P[1], P[2], P[5], P[4]]], C.b, 0.12); mesh(s, [[P[0], P[1], P[2], P[2]], [P[3], P[4], P[5], P[5]]], C.b, 0.08);
  [[0, 1], [1, 2], [2, 0], [0, 3], [2, 5], [3, 5], [1, 4, 1], [3, 4, 1], [4, 5, 1]].forEach(([a, b, h]) => h ? dashed(s, P[a], P[b], edge, 0.0026, 15) : cyl(s, P[a], P[b], 0.0028, edge));
  frame(s, I, P[0], { name: '0', lower: true, L: 0.17, r: 0.0055, ls: 30, lab: [{ dx: 10 }, { dy: 38 }, { dx: 22 }] });
  frame(s, RB.TR(E.T01), P[2], { name: '1', lower: true, L: 0.17, r: 0.0055, ls: 30, lab: [{ dx: 22, dy: 6 }, {}, { dy: 34 }] });
  frame(s, RB.TR(E.T02), P[1], { name: '2', lower: true, L: 0.17, r: 0.0055, ls: 30, lab: [{ dx: 4, dy: 38 }, { dx: -16 }, {}] });
  lab(s, [0, u * 0.5, 0], '1', { s: 32, a: 'c', dy: -12 }); lab(s, [0, 0, u * 0.5], '1', { s: 32, dx: -16, dy: 10, a: 'r' }); lab(s, [0, u / 2, u / 2], '√2', { s: 32, dx: 14, dy: -14 });
  const m = angArc(s, P[1], [0, -1, 0], [0, -1, 1], 0.13, s.pal.ink, 0.0035, false); if (m) lab(s, m, '45°', { s: 28, dx: -6, dy: -14, a: 'r' }); return s; };

/* the transform equation, shown as a loop of frames (Craig 2.6): ᵁ_D T = ᵁ_A T ᴬ_D T = ᵁ_B T ᴮ_C T ᶜ_D T */
SC.teq = (st = {}, o = {}) => { const s = scene('teq', 1400, 950, cam(st, [0.62, 0.55, 0.42], 4.4, 28, 22), o), unk = st.unknown || ''; floor(s, 1.4, [0.6, 0.5]);
  const F = { U: [I, [0, 0, 0]], A: [RB.Rz(rad(30)), [1.05, -0.05, 0.35]], B: [RB.mm(RB.Rx(rad(-20)), RB.Rz(rad(-40))), [0.15, 1.0, 0.3]], C: [RB.Ry(rad(25)), [0.95, 1.15, 0.85]], D: [RB.Rzyx(rad(60), rad(20), rad(10)), [1.35, 0.45, 1.0]] };
  Object.entries(F).forEach(([n, [R, p]]) => frame(s, R, p, { name: n, L: 0.2, r: 0.0055, labels: false, tag: true, ref: n === 'U', tagAt: { dx: -14, dy: -16 } }));
  [['U', 'A', C.am, 'am', { dy: 40 }], ['A', 'D', C.o, 'o', { dx: 30, a: 'l' }], ['U', 'B', C.gr, 'gr', { dx: -26, a: 'r' }], ['B', 'C', C.p, 'p', { dy: -18 }], ['C', 'D', C.b, 'b', { dx: 26, dy: -10, a: 'l' }]].forEach(([a, b, c, key, lo]) => {
    const f = unk === a + b ? dvec : vec; f(s, F[a][1], F[b][1], c, { key, t: 'T', pre: a, sub: b, s: 38, at: 0.5, r: 0.0065, ...lo }); }); return s; };

/* the arm reaching for a bolt (Craig 2.6): base {B}, tool {T}, station {S}, goal {G}. st.p: tool position (metres, default ᴮP_TORG of the example), st.yaw: turn of the tool about the vertical */
SC.bolt = (st = {}, o = {}) => { const E = CH2.EX.bolt, p = st.p || RB.Tp(E.BT), RT = RB.mm(RB.Rz(st.yaw || 0), RB.Rx(PI)), BS = E.BS, BG = E.BG, RS = RB.TR(BS), S = RB.Tp(BS), G = RB.Tp(BG);
  const s = scene(st.name || 'bolt', 1400, 1000, cam(st, [0.42, -0.05, 0.38], 4.2, -62, 22), o); floor(s, 1.35, [0.4, -0.1]);
  /* the table: its top corner is {S}; it spans 0.6 m along X_S and 0.45 m along Y_S */
  const tp = (x, y, z = 0) => RB.add(S, RB.mv(RS, [x, y, z])), tc = s.dark ? [0.36, 0.4, 0.5] : [0.78, 0.8, 0.84];
  box(s, tp(0.3, 0.225, -0.035), tp(0.3, 0.225, 0), 0.6, 0.45, tc, 1, RB.col(RS, 1)); [[0.04, 0.04], [0.56, 0.04], [0.04, 0.41], [0.56, 0.41]].forEach(([x, y]) => { const a = tp(x, y, -0.035); cyl(s, [a[0], a[1], 0], a, 0.016, tc); shadow(s, [a[0], a[1], 0], a, 0.016); }); shadow(s, tp(0.02, 0.225, -0.02), tp(0.58, 0.225, -0.02), 0.2);
  /* the bolt standing on the table at {G} */
  cyl(s, G, v(G, [0, 0, 1], 0.07), 0.011, [0.55, 0.58, 0.62]); cyl(s, v(G, [0, 0, 1], 0.07), v(G, [0, 0, 1], 0.092), 0.021, [0.45, 0.48, 0.52]);
  /* the arm: the RRR arm of chapter 1 with a tool pointing down */
  const q = CH2.armTo(p); s.reach = !!q; pedestal(s, [0, 0], 0.06, 0.1);
  if (q) { const K = RB.ARR.RRR.kin(q).pts, w = v(p, [0, 0, 1], CH2.TOOL); K.slice(0, -1).forEach((a, i) => bar(s, a, K[i + 1], i <= 0 ? 0.036 : 0.03, s.L[i]));
    rj(s, [0, 0, 0.225], [0, 0, 1], { r: 0.05, h: 0.05 }); const s1 = [-sin(q[0]), cos(q[0]), 0]; rj(s, K[1], s1, { r: 0.05, h: 0.05 }); rj(s, K[2], s1, { r: 0.045, h: 0.045 });
    bar(s, w, v(p, [0, 0, 1], 0.075), 0.02, s.L[3]); grip(s, v(p, [0, 0, 1], 0.075), [0, 0, -1], RB.col(RT, 1), { w: 0.034, L: 0.07 }); }
  else { sph(s, p, 0.03, C.rd); }
  if (st.std) { /* Craig 3.8: the standard frames — base {B}, station {S}, wrist {W}, tool {T}, goal {G} — and "where is the tool?" */
    const w = v(p, [0, 0, 1], CH2.TOOL), Lf = 0.17; frame(s, I, O0, { name: 'B', L: 0.3, ref: true, tag: true, labels: false, tagAt: { dx: -20, dy: 40 } });
    frame(s, RT, w, { name: 'W', L: Lf, tag: true, labels: false, tagAt: { dx: -18, dy: -12 } }); frame(s, RT, p, { name: 'T', L: Lf, tag: true, labels: false, tagAt: { dx: 22, dy: 12, a: 'l' } });
    frame(s, RS, S, { name: 'S', L: Lf, tag: true, labels: false, tagAt: { dx: 26, dy: 36, a: 'l' } }); frame(s, RB.TR(BG), G, { name: 'G', L: Lf * 0.85, tag: true, labels: false, tagAt: { dx: 22, dy: -8, a: 'l' } });
    vec(s, O0, w, C.o, { key: 'o', t: 'T', pre: 'B', sub: 'W', s: 34, at: 0.5, dx: -24, dy: -10, a: 'r', r: 0.006 }); vec(s, [0, 0, 0.004], S, C.am, { key: 'am', t: 'T', pre: 'B', sub: 'S', s: 34, at: 0.55, dy: 44, a: 'c', r: 0.006 });
    vec(s, S, G, C.gr, { key: 'gr', t: 'T', pre: 'S', sub: 'G', s: 34, at: 0.5, dx: 20, dy: 40, a: 'l', r: 0.006 }); dvec(s, S, p, C.pk, { key: 'pk', t: 'T', pre: 'S', sub: 'T', post: ' = ?', s: 34, at: 0.55, dx: 24, dy: -6, a: 'l', r: 0.0055 });
    return s; }
  const L = st.short ? 0.16 : 0.22; frame(s, I, O0, { name: 'B', L: 0.3, ref: true, tag: true, labels: false, tagAt: { dx: -20, dy: 40 } });
  frame(s, RT, p, { name: 'T', L, tag: true, labels: false, tagAt: { dx: -18, dy: -10 } }); frame(s, RS, S, { name: 'S', L, tag: true, labels: false, tagAt: { dx: 26, dy: 36, a: 'l' } }); frame(s, RB.TR(BG), G, { name: 'G', L: L * 0.85, tag: true, labels: false, tagAt: { dx: 22, dy: -8, a: 'l' } });
  if (st.arrows !== false) { vec(s, O0, p, C.o, { key: 'o', t: 'T', pre: 'B', sub: 'T', s: 36, at: 0.5, dx: -24, dy: -10, a: 'r', r: 0.006 }); vec(s, [0, 0, 0.004], S, C.am, { key: 'am', t: 'T', pre: 'B', sub: 'S', s: 36, at: 0.55, dx: 0, dy: 44, a: 'c', r: 0.006 });
    vec(s, S, G, C.gr, { key: 'gr', t: 'T', pre: 'S', sub: 'G', s: 36, at: 0.5, dx: 20, dy: 40, a: 'l', r: 0.006 }); dvec(s, p, G, C.pk, { key: 'pk', t: 'T', pre: 'T', sub: 'G', post: ' = ?', s: 36, at: 0.45, dx: 26, dy: -4, a: 'l', r: 0.0055 }); }
  return s; };

/* ================= page 2.4: representations of orientation ================= */
/* where frame {B} is after progress u (0…3) through the three turns of X–Y–Z fixed angles (kind 'fixed': γ about X_A, β about Y_A, α about Z_A)
   or Z–Y–X Euler angles (kind 'euler': α about Z_B, β about Y_B, γ about X_B). Returns the matrix, the step (0, 1, 2), the axis of that step
   in world coordinates, its angle, and the matrix at the start of the step */
CH2.seqAt = (kind, al, be, ga, u) => { const k = min(2, Math.floor(max(0, u))), f = NUM.clamp(u - k, 0, 1);
  if (kind === 'fixed') { const st = [I, RB.Rx(ga), RB.mm(RB.Ry(be), RB.Rx(ga))], tr = [RB.Rx(ga * f), RB.Ry(be * f), RB.Rz(al * f)];
    return { R: RB.mm(tr[k], st[k]), R0: st[k], k, f, axis: RB.col(I, k), ang: [ga, be, al][k], an: ['γ', 'β', 'α'][k], ax: AX[k] + '_A' }; }
  const st = [I, RB.Rz(al), RB.mm(RB.Rz(al), RB.Ry(be))], tr = [RB.Rz(al * f), RB.Ry(be * f), RB.Rx(ga * f)];
  return { R: RB.mm(st[k], tr[k]), R0: st[k], k, f, axis: RB.col(st[k], 2 - k), ang: [al, be, ga][k], an: ['α', 'β', 'γ'][k], ax: AX[2 - k] + '_B' }; };
const seqPanel = (s, P, kind, an, u, op = {}) => { const r = CH2.seqAt(kind, an[0], an[1], an[2], u), ax = r.axis, c = axc(s, kind === 'fixed' ? r.k : 2 - r.k);
  frame(s, I, P, { name: 'A', ref: true, L: 0.55, r: 0.005, ls: op.ls || 30, labels: op.labelsA !== false });
  if (u > 0.001) { if (r.f > 0.001 || op.ghost) frame(s, r.R0, P, { L: 0.44, r: 0.0045, al: 0.28 });
    cyl(s, v(P, ax, -0.62), v(P, ax, 0.72), 0.0032, c, 0.55); turn(s, v(P, ax, 0.6), ax, 0.1, c, 0.3, 0.3 + max(0.6, min(5.4, abs(r.ang) * r.f * 2.2)) * Math.sign(r.ang || 1), 0.0055); }
  frame(s, r.R, P, { name: 'B', L: 0.44, ls: op.ls || 30, lab: op.labB });
  return r; };
SC.seq = (st = {}, o = {}) => { const an = [st.al, st.be, st.ga], s = scene(st.name || 'seq', st.w || 1100, st.h || 1000, cam(st, [0, 0, 0.1], 3.2, 28, 20), o); floor(s, 0.9);
  if (st.target) frame(s, st.target, O0, { L: 0.5, r: 0.011, al: 0.3 });   // an orientation to match (problem 1 of page 2.4)
  s.r = seqPanel(s, O0, st.kind || 'fixed', an, st.u === undefined ? 3 : st.u, { ghost: st.ghost }); return s; };
/* any list of turns, played step by step: st.steps = [[axis, degrees, 'fixed' | 'euler'], …] (CH2.seqR), st.k = progress 0…n (fractions animate a step) */
SC.steps = (st = {}, o = {}) => { const L = st.steps, n = L.length, k = NUM.clamp(st.k || 0, 0, n), s = scene(st.name || 'steps', 1100, 1000, cam(st, [0, 0, 0.1], 3.2, 28, 20), o); floor(s, 0.9);
  frame(s, I, O0, { name: 'A', ref: true, L: 0.6, r: 0.005, ls: 30 });
  if (k <= 0) { frame(s, I, O0, { name: 'B', L: 0.46, ls: 30 }); return s; }
  const i = Math.min(n, Math.ceil(k)) - 1, f = k - i, [a, d, kind] = L[i], j = 'xyz'.indexOf(a), R0 = CH2.seqR(L.slice(0, i)), Rs = [RB.Rx, RB.Ry, RB.Rz][j](rad(d) * f), R = kind === 'fixed' ? RB.mm(Rs, R0) : RB.mm(R0, Rs), ax = kind === 'fixed' ? RB.col(I, j) : RB.col(R0, j), c = axc(s, j);
  frame(s, R0, O0, { L: 0.46, r: 0.0045, al: 0.28 }); cyl(s, v(O0, ax, -0.66), v(O0, ax, 0.76), 0.0032, c, 0.55); turn(s, v(O0, ax, 0.62), ax, 0.1, c, 0.3, 0.3 + max(0.6, min(5.4, abs(rad(d)) * f * 2.2)) * Math.sign(d || 1), 0.0055);
  lab(s, v(O0, ax, 0.8), (kind === 'fixed' ? T('แกนตรึง ', 'fixed ') : T('แกนเคลื่อน ', 'moving ')) + a, { c: AK[j], s: 28, a: 'c', dy: -8 });
  frame(s, R, O0, { name: 'B', L: 0.46, ls: 30 }); s.R = R; return s; };
/* four panels for the slides: the start and the frame after each of the three turns */
SC.seqStrip = (st = {}, o = {}) => { const kind = st.kind || 'fixed', an = [st.al, st.be, st.ga], c3 = view([0, 0, 0.2], 9.2, 28, 17), s = scene(st.name || 'seq_' + kind, 2000, 640, c3, o), A = S3.axes(c3), right = [-A.x[0], -A.x[1], 0];
  const names = kind === 'fixed' ? [['γ', 'X', 'A'], ['β', 'Y', 'A'], ['α', 'Z', 'A']] : [['α', 'Z', 'B'], ['β', 'Y', 'B'], ['γ', 'X', 'B']], vals = kind === 'fixed' ? [st.ga, st.be, st.al] : [st.al, st.be, st.ga];
  for (let i = 0; i < 4; i++) { const P = v(O0, right, (i - 1.5) * 1.18); floor(s, 0.6, [P[0], P[1]]); seqPanel(s, P, kind, an, i, { ghost: true, ls: 28, labelsA: i === 0 });
    lab(s, P, i ? `${i}.  ${names[i - 1][0]} = ${Math.round(RB.deg(vals[i - 1]))}°` : T('เริ่ม: {B} ทับ {A}', 'start: {B} on {A}'), { a: 'c', dy: 120, s: 30 });
    if (i) lab(s, P, T('รอบแกน ', 'about '), { a: 'r', dy: 160, s: 26, c: 'mut', b: false, dx: 6 }), lab(s, P, names[i - 1][1], { sub: names[i - 1][2], m: 1, a: 'l', dy: 160, s: 30, dx: 10, c: AK['XYZ'.indexOf(names[i - 1][1])] }); }
  return s; };

/* rotation about an axis K by θ (Craig 2.8); st.p: a point the axis passes through (units, k metres per unit), st.u: share of the turn shown */
SC.axis = (st = {}, o = {}) => { const kk = RB.unit(st.k || [0.707, 0.707, 0]), th = st.th === undefined ? rad(30) : st.th, u = st.u === undefined ? 1 : st.u, k = st.sc || 0.15, P = RB.sc(st.p || [0, 0, 0], k), R = RB.Rk(kk, th * u), Ob = RB.add(P, RB.mv(R, RB.sc(P, -1)));
  const s = scene(st.name || 'axis', st.w || 1200, st.h || 1000, cam(st, st.tgt0 || [0.12, 0.15, 0.2], st.dist0 || 3.3, -24, 22), o); floor(s, 1.0, [0.12, 0.15]);
  frame(s, I, O0, { name: 'A', ref: true, L: 0.55, r: 0.0055, tag: true, ls: 32 });
  cyl(s, v(P, kk, -0.45), v(P, kk, 0.3), 0.003, C.p, 0.6); vec(s, P, v(P, kk, 0.62), C.p, { key: 'p', t: 'K', pre: 'A', at: 1.08, dx: 4, dy: 8, a: 'c', r: 0.0075 });
  turn(s, v(P, kk, 0.48), kk, 0.07, C.p, 0.3, 0.3 + max(0.5, 5.2 * u), 0.004);
  if (hypot(...P) > 1e-6) { vec(s, O0, P, C.pk, { key: 'pk', t: 'P', pre: 'A', at: 0.5, dx: -22, dy: 6, a: 'r', r: 0.006 }); sph(s, P, 0.018, C.pk);
    const c0 = v(P, kk, RB.dot(sub(O0, P), kk)), a = sub(O0, c0), r = hypot(...a); if (r > 0.02 && u > 0.01) { const ua = RB.unit(a), b = RB.cross(kk, ua), pts = []; for (let i = 0; i <= 24; i++) { const t = th * u * i / 24; pts.push(v(v(c0, ua, r * cos(t)), b, r * sin(t))); } tube(s, pts, s.pal.ink, 0.004, true); } }
  frame(s, R, Ob, { name: 'B', L: 0.44, tag: hypot(...P) > 1e-6, tagAt: { dx: 22, dy: 40, a: 'l' }, lab: st.labB });
  if (hypot(...P) < 1e-6) { const m = angArc(s, O0, [1, 0, 0], RB.col(R, 0), 0.32, s.pal.ink, 0.004); if (m) lab(s, m, 'θ', { m: 1, s: 36, a: 'c', dy: 34 }); }
  return s; };

/* several frames that share their orientation data but not their origin: {1} in grey, {2} and {3} beside it, each with a faint copy of {1} */
SC.multi = (st = {}, o = {}) => { const c3 = view([0, 0, 0.12], 6.6, 28, 20), s = scene(st.name || 'multi', 1600, 800, c3, o), A = S3.axes(c3), right = [-A.x[0], -A.x[1], 0], F = st.F || [['1', I], ['2', CH2.EX.hw22.R12], ['3', CH2.EX.hw22.R13]];
  F.forEach(([n, R], i) => { const P = v(O0, right, (i - (F.length - 1) / 2) * 1.2); floor(s, 0.6, [P[0], P[1]]); if (i) frame(s, I, P, { L: 0.5, r: 0.004, al: 0.3, ref: true }); frame(s, R, P, { name: n, L: 0.45, ref: !i, tag: true, ls: 30, tagAt: { a: 'c', dx: 0, dy: 120 } }); });
  return s; };

/* ================= live views for the pages (browser only) =================
   CH2.view(name, ctx, W, H, state, palette): draws the scene with gl3d.js (or flat without WebGL), then its labels in 2-D on top.
   The camera of the last frame is kept in state._v for picking. */
if (typeof document === 'undefined') return;
const G = RB.G, KEY = { ink: 'ink', mut: 'muted', o: 'l1', b: 'l2', p: 'l3', x: 'x', y: 'y', z: 'z', pk: 'sel', gr: 'good', am: 'warn', rd: 'bad' };
/* a label with Craig's leading superscript (ᴬP), a subscript and trailing text */
/* a label with Craig's leading superscript (ᴬP), a subscript and trailing text. lw: its width; label: draws it with the left end of its baseline at (x, y) */
const fonts = size => { const fs = Math.round(size * 0.66); return { fM: `italic 600 ${size}px ${RB.SERIF}`, fS: `600 ${fs}px ${RB.SERIF}`, fP: `italic 600 ${fs}px ${RB.SERIF}`, fT: b => `${b ? 700 : 500} ${size}px ${RB.FONT}` }; };
const lw = (c, L, size) => { const f = fonts(size), W = (t, ff) => { if (!t) return 0; c.font = ff; return c.measureText(t).width; }; c.save();
  const w = L.m ? W(L.pre, f.fP) + (L.pre ? 1 : 0) + W(L.t, f.fM) + W(L.sub, f.fS) + (L.sub ? 1 : 0) + W(L.post, f.fM) : W(L.t, f.fT(L.b)); c.restore(); return w; };
const label = (c, L, x0, y, size, col, bg) => { const f = fonts(size); c.save(); c.textAlign = 'left'; c.textBaseline = 'alphabetic';
  const put = (t, ff, px, py, hw = 4) => { c.font = ff; c.lineWidth = hw; c.strokeStyle = bg; c.lineJoin = 'round'; c.strokeText(t, px, py); c.fillStyle = col; c.fillText(t, px, py); };
  if (!L.m) { put(L.t, f.fT(L.b), x0, y, 3.5); c.restore(); return; }
  const W = (t, ff) => { if (!t) return 0; c.font = ff; return c.measureText(t).width; }, wp = W(L.pre, f.fP) + (L.pre ? 1 : 0), wm = W(L.t, f.fM), ws = W(L.sub, f.fS) + (L.sub ? 1 : 0);
  if (L.pre) put(L.pre, f.fP, x0, y - size * 0.4); put(L.t, f.fM, x0 + wp, y); if (L.sub) put(L.sub, f.fS, x0 + wp + wm + 0.5, y + size * 0.24); if (L.post) put(L.post, f.fM, x0 + wp + wm + ws, y); c.restore(); };
/* the picture, then its labels: each label is moved off the picture and off the other labels when it has to be (S3.place), with a leader line back */
const paint = CH2.paint = (c, W, H, s, P, st, res) => { if (global.R3 && global.R3.ok()) global.R3.draw(c, W, H, s); else if (global.R3) global.R3.flat(c, W, H, s);
  const k = W / s.ow, items = s.labels.map(L => { const q = S3.proj(s.cam, W, H, L.p), size = max(11, L.s * 1.3 * k), w = lw(c, L, size), x = q[0] + L.dx * k;
    return { L, size, w, x: L.a === 'c' ? x - w / 2 : L.a === 'r' ? x - w : x, y: q[1] + L.dy * k, ax: q[0], ay: q[1] }; });
  S3.place(c, W, H, items, P.bg, st ? (st._lm || (st._lm = [])) : null, res);
  items.forEach(it => { if (it.lead) S3.leader(c, it, P.muted); }); items.forEach(it => label(c, it.L, it.X, it.Y, it.size, P[KEY[it.L.c]] || P.ink, P.bg)); };
/* scenes of any chapter: CH2.S3, then CH3.S3, CH4.S3 … */
const sceneFn = name => SC[name] || ['CH3', 'CH4', 'CH5', 'CH6', 'CH7', 'CH8', 'CH9'].map(k => global[k] && global[k].S3 && global[k].S3[name]).find(Boolean);
/* the faint 0.1 m grid (S3.floorGrid) goes into every scene measured in metres: the arms of chapters 3 and 4 and the bolt; chapter 2's other
   pictures are drawn to no scale (or carry their own numbered grid). A planar arm (every frame at one height) gets it in its own plane */
const gridOn = (name, s) => { if (SC[name] && name !== 'bolt') return false; if (s.F && s.gridZ === undefined) { const z = s.F.map(T => T[2][3]); if (z.every(x => Math.abs(x - z[0]) < 1e-6) && s.F.every(T => Math.abs(T[2][2]) > 0.999)) s.gridZ = z[0]; } return true; };
CH2.gridNote = (c, W, P) => G.text(c, T('ตาราง 0.1 m', 'grid 0.1 m'), W - 10, 17, { color: P.muted, size: 11.5, align: 'right', halo: P.bg });
CH2.view = (name, c, W, H, st, P) => { const s = sceneFn(name)(st, { dark: P === RB.DARK }); if (st.ov) st.ov(s, st);   // the open step of the page's solution (S3.OV)
  s.cam = S3.userCam(s.cam, st);   // scenes with a fixed camera: the reader's turning and zooming go on top
  const g = gridOn(name, s); if (g) S3.floorGrid(s);
  st._v = { cam: s.cam, W, H }; paint(c, W, H, s, P, st, g ? [[W - 120, 0, W, 26]] : null); if (g) CH2.gridNote(c, W, P); return s; };
/* the point of the horizontal plane z = z0 under the pointer */
CH2.pick = (st, p, z0 = 0) => st._v ? S3.unproject(st._v.cam, st._v.W, st._v.H, p.x, p.y, z0) : null;
/* drag in a canvas to turn the view (st.az, st.el); o.grab(p) may take the drag for something else (return true) */
CH2.orbit = (cv, st, o) => S3.orbit(cv, st, o || {});   // drag turns, pinch / Ctrl+wheel / buttons zoom (scene3d.js); o.grab(p) may take the drag
/* one simulation on a page: a canvas drawn every frame with scene `name` of state st; o: aspect, maxH, orbit (default on), drag (see CH2.orbit), tick(dt), after(c, W, H, P, scene) */
CH2.sim = (cv, st, name, o = {}) => { if (typeof cv === 'string') cv = document.getElementById(cv); const out = { s: null, cv };
  RB.stage(cv, o.aspect || 0.8, (c, W, H, dt, P) => { if (o.tick) o.tick(dt); out.s = CH2.view(typeof name === 'function' ? name() : name, c, W, H, st, P); if (o.after) o.after(c, W, H, P, out.s); }, { maxH: o.maxH || 520 });
  CH2.orbit(cv, st, o.drag || {}); return out; };   // every view can be turned and zoomed
CH2.pillAt = (c, t, col, y = 18) => G.pill(c, t, 10, y, { fill: RB.rgba(col, 0.16), stroke: col, color: col, size: 12.5 });

/* "try it first" for a whole matrix: o = { rows, cols, label (html), ans: matrix, tol, check: M → hint for a known mistake, onOk } */
const parseNum = t => { let x = String(t).trim().replace(/−/g, '-').replace(/,/g, '.').replace(/(^|[^.a-z])sqrt\s*\(/gi, '$1Math.sqrt(').replace(/√\s*\(?([0-9.]+)\)?/g, 'Math.sqrt($1)');
  if (!x || !/^[-+*/().0-9\sMathsqrt]*$/.test(x)) return NaN; try { const v = Function('"use strict";return (' + x + ')')(); return typeof v === 'number' ? v : NaN; } catch (e) { return NaN; } };
CH2.parseNum = parseNum;
CH2.matAsk = (el, o) => { const n = o.rows || 3, m = o.cols || 3, tol = o.tol || 0.006; el.classList.add('matask');
  el.innerHTML = `<span class="asklab">${T('ลองตอบก่อน:', 'Try first:')}</span> <span class="mlab">${o.label || ''}</span><span class="matin" style="grid-template-columns:repeat(${m},auto)">${Array.from({ length: n * m }, (_, i) => `<input class="mi" data-i="${i}" inputmode="text" autocomplete="off" aria-label="row ${Math.floor(i / m) + 1} column ${i % m + 1}">`).join('')}</span>` +
    ` <button type="button" class="btn on">${T('ตรวจ', 'Check')}</button> <span class="askres"></span>`;
  if (global.MC && MC.renderMath) MC.renderMath(el.querySelector('.mlab'));   // the label may hold TeX: the page rendered its math before this box existed
  const ins = [...el.querySelectorAll('input')], res = el.querySelector('.askres');
  const run = () => { const v = ins.map(x => parseNum(x.value)); if (v.some(x => !isFinite(x))) { res.innerHTML = T('ใส่ตัวเลขให้ครบทุกช่อง (ใช้ 0.866 หรือ √3/2 ก็ได้)', 'Fill in every box (0.866 or √3/2 both work)'); return; }
    const M = Array.from({ length: n }, (_, i) => v.slice(i * m, i * m + m)); let all = true; ins.forEach((x, i) => { const ok = abs(v[i] - o.ans[Math.floor(i / m)][i % m]) <= tol; x.classList.toggle('ok', ok); x.classList.toggle('no', !ok); if (!ok) all = false; });
    if (all) { res.innerHTML = `<span class="pill good">✓</span> ${T('ถูกต้อง!', 'Correct!')}`; if (o.onOk) o.onOk(); return; }
    const why = o.check ? o.check(M) : ''; res.innerHTML = `<span class="pill bad">✗</span> ${why || T('ยังไม่ใช่ ช่องสีแดงยังผิด ลองอีกครั้ง หรือเปิดเฉลยทีละขั้น', 'Not yet: the red boxes are wrong. Try again, or open the solution step by step')}`; if (global.MC && MC.renderMath) MC.renderMath(res); };
  el.querySelector('button').addEventListener('click', run); ins.forEach(x => x.addEventListener('keydown', e => { if (e.key === 'Enter') run(); }));
  return { reset() { ins.forEach(x => { x.value = ''; x.classList.remove('ok', 'no'); }); res.innerHTML = ''; } }; };
})(typeof window !== 'undefined' ? window : globalThis);
