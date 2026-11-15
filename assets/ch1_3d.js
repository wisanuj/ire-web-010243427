/* ch1_3d.js — the 3-D scenes of chapter 1. CH1.S3.name(state, {dark}) returns a scene table (see scene3d.js);
   the same functions feed the live WebGL views of the pages (CH1.view.name, bottom of this file) and, through
   IRE_Slides/build/coppelia/gen_scenes_ch1.mjs, the slide figures rendered by CoppeliaSim. Needs robot.js, ch1.js, scene3d.js. */
(function (global) {
'use strict';
const RB = global.RB, S3 = global.S3, CH1 = global.CH1, C = S3.C, { sin, cos, PI, max, min, abs, floor: flr, round } = Math, rad = RB.rad, v = S3.v, un = RB.unit;
const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
const { bar, cyl, sph, box, arrow, tube, dashed, floor, arc, arcH, arcV, rj, pj, grip, pedestal, grid, annulus, circle, cells, harm, lab, camAt, scene, cached, shadow } = S3;
const Y = [0, -1, 0], SC = CH1.S3 = {};
const vcam = (cx, cz, dist) => camAt([cx, 0, cz], dist * 1.08, -76, 10, 26), topcam = R => camAt([0, 0, 0], R * 4.5, -90, 58, 28);
/* a chain of R and P joints standing in the vertical x–z plane, same geometry as CH1.draw.chain */
function vchain(s, types, q, o = {}) { const h0 = o.h0 || 0.2, spec = CH1.chainSpec(types), k = RB.chain(spec, q), P = k.pts.map(p => [p[0], 0, p[1] + h0]);
  pedestal(s, [0, 0], h0 - 0.03); spec.forEach((j, i) => bar(s, P[i], P[i + 1], j.type === 'R' ? 0.03 : 0.017, s.L[i + 1]));
  spec.forEach((j, i) => { const a = k.ang[i], d = [cos(a), 0, sin(a)]; if (j.type === 'R') rj(s, P[i], Y, { r: 0.052, h: 0.06 }); else pj(s, v(P[i], d, 0.085), d, { up: [0, 1, 0] }); });
  const a = k.th; grip(s, P[P.length - 1], [cos(a), 0, sin(a)], [-sin(a), 0, cos(a)]); return { P, k }; }
/* the points the picture of a chain must hold: the base, every joint, the end of the gripper */
CH1.chainPts = (types, q, h0 = 0.2) => { const k = RB.chain(CH1.chainSpec(types), q), P = k.pts.map(p => [p[0], 0, p[1] + h0]), e = P[P.length - 1]; return [[0, 0, 0], [0.15, 0, 0], [-0.15, 0, 0], ...P, [e[0] + 0.1 * cos(k.th), 0, e[2] + 0.1 * sin(k.th)]]; };
/* st.xyz (the pages turn it on; the slide figures leave it off): x, y, z of the end of the arm from the base, with the base's axes (S3.xyz) */
const xyz = (s, st, O, tip, o = {}) => { if (st.xyz) s.tip = S3.xyz(s, O, tip, { head: T('ปลายแขน เทียบฐาน', 'end of arm, from the base'), axes: 0.24, axIdx: [0, 1], ...o }); };   // the planar arms: only x and y drawn (z points at a camera that looks down)

/* ---- page 1.1 ---- */
/* page view: the whole scene (base, every reachable part position, the camera on its post) below the page's sense–think–act boxes */
SC.loop = (st, o = {}) => { const s = scene('loop', 1400, 950, o.fig ? camAt([0.45, 0.45, 0.22], 4.1, -100, 38, 28) : st.view ? camAt(...st.view, 28) : camAt([0.42, 0.34, 0.3], 4.6, -98, 50, 28), o),
    { l1, l2 } = CH1.LOOP, part = [st.tx, st.ty], aim = st.mx != null ? [st.mx, st.my] : part; floor(s, 1.6, [0.35, 0.35]);
  if (st.qd) harm(s, [l1, l2], st.qd, { al: 0.3, cols: [s.L[3], s.L[3]], grip: false });
  const arm = harm(s, [l1, l2], st.q, { open: st.grip ? 0.2 : 1 }); xyz(s, st, arm.pts[0], arm.pts[2]);
  box(s, [part[0], part[1], 0], [part[0], part[1], 0.07], 0.07, 0.07, C.am); shadow(s, [part[0], part[1], 0.03], [part[0], part[1], 0.031], 0.045);
  const cp = [0.98, 0.82, 0.5], dir = un([aim[0] - cp[0], aim[1] - cp[1], 0.05 - cp[2]]); cyl(s, [cp[0], cp[1], 0], [cp[0], cp[1], cp[2]], 0.018, s.pal.base); shadow(s, [cp[0], cp[1], 0], cp, 0.018);
  box(s, v(cp, dir, -0.06), v(cp, dir, 0.08), 0.085, 0.07, s.pal.sleeve); cyl(s, v(cp, dir, 0.08), v(cp, dir, 0.12), 0.03, [0.05, 0.06, 0.09]);
  if (st.sense !== false) dashed(s, v(cp, dir, 0.13), [aim[0], aim[1], 0.08], s.L[2], 0.004, 17);
  if (st.mx != null && !o.fig) tube(s, RB.ring([st.mx, st.my, 0.09], [0, 0, 1], 0.07, 0, 2 * PI, 24), s.L[2], 0.005);   // where the camera measured the part
  if (o.fig) { lab(s, cp, '1  SENSE', { c: 'b', dx: -40, dy: -70, a: 'r', s: 34 }); lab(s, cp, 'camera finds the part', { c: 'mut', dx: -40, dy: -32, a: 'r', s: 24, b: false });
    const f = RB.fk2(l1, l2, st.qd[0], st.qd[1]), g = RB.fk2(l1, l2, st.q[0], st.q[1]); lab(s, [f.x1 / 2, f.y1 / 2, 0.11], '2  THINK', { c: 'p', dx: -40, dy: -40, a: 'r', s: 34 }); lab(s, [f.x1 / 2, f.y1 / 2, 0.11], 'joint angles are computed', { c: 'mut', dx: -40, dy: -2, a: 'r', s: 24, b: false });
    lab(s, [g.x1 / 2, g.y1 / 2, 0.11], '3  ACT', { c: 'o', dy: 80, a: 'c', s: 34 }); lab(s, [g.x1 / 2, g.y1 / 2, 0.11], 'motors move the arm', { c: 'mut', dy: 118, a: 'c', s: 24, b: false }); }
  else lab(s, cp, T('กล้อง', 'camera'), { c: 'mut', dx: 30, dy: -30, s: 26 });
  lab(s, [part[0], part[1], 0.07], T('ชิ้นงาน', 'part'), { c: 'am', dx: 26, dy: -6 }); return s; };
SC.jobs = (st, o = {}) => { const s = scene('jobs', 1300, 950, camAt([0.02, 0.36, 0.05], 3.5, -90, 54, 28), o), { l1, l2 } = CH1.LOOP; floor(s, 1.4, [0, 0.35]);
  tube(s, st.pts.concat([st.pts[0]]).map(p => [p[0], p[1], 0.02]), C.pk, 0.004);
  st.pts.forEach((p, i) => { sph(s, [p[0], p[1], 0.03], 0.028, C.am); lab(s, [p[0], p[1], 0.03], 'P' + (i + 1), { c: 'am', dx: 18, dy: -14, s: 28 }); });
  const arm = harm(s, [l1, l2], st.q, { open: st.open }); xyz(s, st, arm.pts[0], arm.pts[2]); return s; };

/* ---- page 1.2 ---- */
/* st.fit (page 1.2, where every joint goes from −150° to 150°): the camera follows the arm so that it never leaves the picture (S3.fitCam) */
SC.chain = (st, o = {}) => { const cm = st.cam || [0.3, 0.42, 2.25], w = st.w || 800, h = st.h || 700, pts = st.fit ? CH1.chainPts(st.types, st.q) : null;
  const s = scene(st.name || 'chain3', w, h, pts ? S3.fitCam(st, pts, -76, 10, 26, w, h, { m: 0.26, minD: 2.6 }) : vcam(cm[0], cm[1], cm[2]), { ...o, noShadow: st.noShadow }); if (!st.noShadow) floor(s, 1.3, [cm[0], 0]);
  const r = vchain(s, st.types, st.q), k = r.k, ar = st.ar || 0.17, ls = st.ls || 44;
  if (st.vars) CH1.chainSpec(st.types).forEach((j, i) => { const a0 = i ? k.ang[i - 1] : 0, a1 = k.ang[i], O = v(r.P[i], Y, 0.075), sub = st.sub || String(i + 1);
    if (j.type === 'R') { dashed(s, O, [O[0] + 0.3 * cos(a0), O[1], O[2] + 0.3 * sin(a0)]); if (abs(a1 - a0) > 0.04) arcV(s, O, ar, a0, a1, s.pal.ink); const am = (a0 + a1) / 2; lab(s, [O[0] + (ar + 0.08) * cos(am), O[1], O[2] + (ar + 0.08) * sin(am)], 'θ', { sub, m: 1, s: ls, dy: 14, a: 'c' }); }
    else { const a = r.P[i], b = r.P[i + 1], n = [sin(a1), 0, -cos(a1)], A = [a[0] + n[0] * 0.1, -0.09, a[2] + n[2] * 0.1], B = [b[0] + n[0] * 0.1, -0.09, b[2] + n[2] * 0.1]; arrow(s, A, B, s.pal.ink, 0.005); arrow(s, B, A, s.pal.ink, 0.005);
      lab(s, [(A[0] + B[0]) / 2 + n[0] * 0.08, -0.09, (A[2] + B[2]) / 2 + n[2] * 0.08], 'd', { sub, m: 1, s: ls, dy: 14, a: 'c' }); } });
  xyz(s, st, r.P[0], r.P[r.P.length - 1], { axes: 0.22, axIdx: [0, 2] });   // the arm stands in the x–z plane: y points away from the camera
  if (st.axis) { const O = r.P[0]; arrow(s, v(O, [0, 1, 0], 0.22), v(O, Y, 0.36), s.pal.z, 0.005); lab(s, v(O, Y, 0.36), T('แกนของข้อต่อ', 'joint axis'), { c: 'z', dx: 18, dy: 30, s: 28 }); }
  return s; };
SC.dof = (st = {}, o = {}) => { const s = scene('dof', 1600, 760, camAt([0.05, 0, 0.36], 6.6, st.az === undefined ? -68 : st.az, st.el === undefined ? 20 : st.el, 28), o); floor(s, 2.2);
  const og = [-1.3, -0.32, 0.012], ph = rad(28), c = [-0.7, 0.14, 0.03], dx = [cos(ph), sin(ph), 0];
  arrow(s, og, [og[0] + 1.1, og[1], og[2]], C.x, 0.008); arrow(s, og, [og[0], og[1] + 0.95, og[2]], C.y, 0.008); box(s, v(c, dx, -0.27), v(c, dx, 0.27), 0.26, 0.04, s.L[2], 1, [0, 0, 1]);
  dashed(s, [c[0], og[1], 0.012], [c[0], c[1], 0.012], null, 0.004); dashed(s, [og[0], c[1], 0.012], [c[0], c[1], 0.012], null, 0.004); dashed(s, [c[0], c[1], 0.06], [c[0] + 0.42, c[1], 0.06], null, 0.004); arrow(s, [c[0], c[1], 0.06], v([c[0], c[1], 0.06], dx, 0.42), s.pal.ink, 0.006); arcH(s, [c[0], c[1], 0.06], 0.3, 0, ph, C.pk, 0.006);
  lab(s, [c[0], og[1], 0], 'x', { m: 1, c: 'x', s: 44, dy: 40, a: 'c' }); lab(s, [og[0], c[1], 0], 'y', { m: 1, c: 'y', s: 44, dx: -22, a: 'r' }); lab(s, [c[0] + 0.38, c[1] + 0.1, 0.06], 'φ', { m: 1, c: 'pk', s: 44, dx: 10 }); lab(s, [-0.75, -0.5, 0], T('ในระนาบ: 3 องศาอิสระ', 'In a plane: 3 DOF'), { a: 'c', dy: 50, s: 28 });
  const B = [0.8, 0, 0.5]; box(s, v(B, [1, 0, 0], -0.3), v(B, [1, 0, 0], 0.3), 0.36, 0.2, s.L[2], 1, [0, 0, 1]); shadow(s, [B[0] - 0.3, 0, 0.5], [B[0] + 0.3, 0, 0.5], 0.17);
  [[[1, 0, 0], C.x, 'x', 'roll'], [[0, 1, 0], C.y, 'y', 'pitch'], [[0, 0, 1], s.pal.z, 'z', 'yaw']].forEach(([d, col, nm, rot], i) => { const e = v(B, d, i === 1 ? 0.9 : i === 2 ? 0.6 : 0.74), m = v(B, d, i === 1 ? 0.68 : i === 2 ? 0.42 : 0.56); arrow(s, v(B, d, -0.2), e, col, 0.007); arc(s, m, d, 0.09, 0.5, 5.7, col, 0.006);
    lab(s, e, nm, { m: 1, c: nm, s: 44, dx: i === 2 ? 14 : 12, dy: i === 2 ? 4 : 30 }); lab(s, m, rot, { c: nm, s: 24, dx: i === 2 ? 46 : 0, dy: i === 2 ? 8 : -46, a: i === 2 ? 'l' : 'c' }); });
  lab(s, [0.8, -0.5, 0], T('ในปริภูมิ: 6 องศาอิสระ', 'In space: 6 DOF'), { a: 'c', dy: 50, s: 28 }); return s; };
SC.ws2 = (st, o = {}) => { const R = st.R || 0.78, s = scene(st.name || 'ws2', 1200, 1050, topcam(R), o), { l1, l2 } = st, lim = st.lim || CH1.FULL, rg = RB.ring2(l1, l2), k = st.k === undefined ? 9 : st.k, f = R / 0.78; floor(s, R * 1.3);
  const full = lim.b1 - lim.a1 >= 2 * PI - 1e-6 && lim.b2 - lim.a2 >= 2 * PI - 1e-6;
  if (k >= 3) cached(s, ['ws2', l1, l2, lim.a1, lim.b1, lim.a2, lim.b2].join(), t => { if (full) annulus(t, rg.rmin, rg.rmax, 0.004, C.b, 0.3); else cells(t, (x, y) => RB.ws2Has(l1, l2, lim, x, y), l1 + l2 + 0.02, 0.004, C.b, 0.3, 170); });
  if (full && k >= 1) { circle(s, rg.rmax, 0.007, s.L[2]); const a = rad(32), p = [0.64 * rg.rmax * cos(a), 0.64 * rg.rmax * sin(a), 0.012]; arrow(s, [0, 0, 0.012], [rg.rmax * cos(a), rg.rmax * sin(a), 0.012], s.L[2], 0.005);
    lab(s, p, 'r', { sub: 'max', m: 1, c: 'b', s: 40, dx: 6, dy: 44 }); lab(s, p, `= ${RB.u(rg.rmax, 2)} m`, { c: 'b', s: 30, dx: 96, dy: 44 }); }
  if (full && k >= 2 && rg.rmin > 1e-6) { circle(s, rg.rmin, 0.007, C.pk); const p = [-0.2 * f, -0.16 * f, 0.01]; lab(s, p, 'r', { sub: 'min', m: 1, c: 'pk', s: 40, dx: -210, dy: 30 }); lab(s, p, `= ${RB.u(rg.rmin, 2)} m`, { c: 'pk', s: 30, dx: -120, dy: 30 }); }
  if (st.trace && st.trace.length > 1) { const n = st.trace.length, stp = max(1, Math.ceil(n / 400)), pts = []; for (let i = 0; i < n; i += stp) pts.push([st.trace[i][0], st.trace[i][1], 0.11]); if (pts.length > 1) tube(s, pts, C.am, 0.004); }
  const arm = harm(s, [l1, l2], [st.t1, st.t2]); xyz(s, st, arm.pts[0], arm.pts[2]); return s; };
SC.dex = (st, o = {}) => { const R = st.R || 0.86, s = scene(st.name || 'dex', 1200, 1050, topcam(R), o), { l1, l2, l3, x, y } = st, rg = RB.ring3(l1, l2, l3), z0 = 0.11; floor(s, R * 1.3);
  cached(s, ['dex', l1, l2, l3].join(), t => { annulus(t, rg.reach[0], rg.reach[1], 0.004, C.b, 0.25); circle(t, rg.reach[1], 0.01, t.L[2]); if (rg.dex) { annulus(t, rg.dex[0], rg.dex[1], 0.008, C.gr, 0.42); circle(t, rg.dex[0], 0.012, C.gr); circle(t, rg.dex[1], 0.012, C.gr); } });
  const ok = RB.dexOk(l1, l2, l3, x, y), N = 40; let good = 0;
  if (ok(st.phi)) { const q = RB.ik2(l1, l2, x - l3 * cos(st.phi), y - l3 * sin(st.phi), st.elbow || 1); const arm = harm(s, [l1, l2, l3], [q.t1, q.t2, st.phi - q.t1 - q.t2], { grip: false }); xyz(s, st, arm.pts[0], arm.pts[3]); }
  else { pedestal(s, [0, 0], z0 - 0.02); arrow(s, [x - 0.22 * cos(st.phi), y - 0.22 * sin(st.phi), z0], [x, y, z0], C.rd, 0.007); sph(s, [x, y, z0], 0.022, s.pal.ink); }
  for (let i = 0; i < N; i++) { const a0 = 2 * PI * i / N, a1 = 2 * PI * (i + 0.7) / N, g = ok((a0 + a1) / 2); if (g) good++; cyl(s, [x + 0.075 * cos(a0), y + 0.075 * sin(a0), z0 + 0.03], [x + 0.075 * cos(a1), y + 0.075 * sin(a1), z0 + 0.03], 0.0075, g ? C.gr : C.rd); }
  s.good = good / N; lab(s, [0, -(rg.reach[1] - 0.1), 0], T('ไปถึงได้', 'reachable'), { c: 'b', a: 'c', s: 30 }); if (rg.dex) lab(s, [0, -(rg.dex[0] + rg.dex[1]) / 2 - 0.02, 0], T('คล่องแคล่ว', 'dexterous'), { c: 'gr', a: 'c', s: 30 }); return s; };

/* ---- page 1.3 ---- */
SC.path = (st, o = {}) => { const s = scene('path', 1400, 1000, camAt([0.42, 0.33, 0.05], 3.8, -90, 52, 28), o), { l1, l2 } = CH1.PATH, z0 = 0.11, A = st.A, B = st.B, n = max(2, st.n | 0); floor(s, 1.35, [0.4, 0.3]);
  box(s, [A[0], A[1], 0.006], [B[0], B[1], 0.006], 0.035, 0.012, s.pal.base, 1, [0, 0, 1]); tube(s, [[A[0], A[1], z0], [B[0], B[1], z0]], C.gr, 0.005);
  const pts = []; for (let k = 0; k <= 40; k++) { const r = CH1.pathAt(st, k / 40, 'ptp'); pts.push([r.p[0], r.p[1], z0]); } tube(s, pts, C.pk, 0.0065);
  for (let i = 0; i < n; i++) sph(s, [A[0] + (B[0] - A[0]) * i / (n - 1), A[1] + (B[1] - A[1]) * i / (n - 1), z0], 0.02, C.am);
  const r = CH1.pathAt(st, st.u, st.mode || 'ptp'); if (r.ok) { const arm = harm(s, [l1, l2], r.q, { grip: false }); xyz(s, st, arm.pts[0], arm.pts[2]); } [A, B].forEach((p, i) => lab(s, [p[0], p[1], z0], 'AB'[i], { dx: i ? 22 : -22, dy: i ? 30 : -14, a: i ? 'l' : 'r', s: 38 }));
  lab(s, pts[20], T('เส้นทางแบบจุดต่อจุด', 'point-to-point path'), { c: 'pk', dx: 26, dy: -26, s: 28 }); lab(s, [0.44, 0.46, 0.006], T('แนวเชื่อม', 'seam'), { c: 'gr', dx: -30, dy: 56, a: 'r', s: 28 }); return s; };
/* the spherical wrist drawn from primitives (the slides show the wrist of a real arm instead) */
SC.wristS = (st, o = {}) => { const W = [0, 0, 0.62], s = scene('wristS', 1300, 1000, camAt([0.05, 0, 0.62], 3.5, st.az === undefined ? -64 : st.az, st.el === undefined ? 18 : st.el, 28), o), R = CH1.wristR(st), d = RB.mv(R, [1, 0, 0]), sd = RB.mv(R, [0, 1, 0]); floor(s, 1.4);
  bar(s, [-0.95, 0, 0.62], W, 0.055, s.L[2]); lab(s, [-0.8, 0, 0.62], T('ปลายแขน', 'forearm'), { c: 'b', dy: 64, s: 26 });
  [[[0, 0, 1], s.pal.z, 'z', T('หันซ้ายขวา (yaw)', 'yaw')], [RB.mv(RB.Rz(st.yaw), [0, 1, 0]), C.y, 'y', T('ก้มเงย (pitch)', 'pitch')], [d, C.x, 'x', T('หมุนรอบแกนเครื่องมือ (roll)', 'roll')]].forEach(([ax, col, key, t], i) => { const e = v(W, ax, i === 2 ? 0.92 : 0.6);
    arrow(s, v(W, ax, -0.42), e, col, 0.006); arc(s, v(W, ax, i === 2 ? 0.76 : 0.46), ax, 0.07, 0.4, 5.7, col, 0.005); lab(s, e, t, { c: key, dx: 16, dy: i === 0 ? 8 : -12, s: 28 }); });
  const e = v(W, d, 0.34), a = v(e, sd, -0.12), b = v(e, sd, 0.12); bar(s, W, e, 0.036, s.L[3]); box(s, a, b, 0.05, 0.06, s.pal.ink); [a, b].forEach(q => box(s, q, v(q, d, 0.22), 0.04, 0.06, s.pal.ink)); sph(s, v(b, d, 0.22), 0.03, C.am);
  sph(s, W, 0.085, s.pal.joint); lab(s, W, T('จุดศูนย์กลางข้อมือ: แกนทั้งสามตัดกันที่จุดนี้', 'wrist centre: the three axes meet here'), { a: 'c', dy: 190, s: 26 }); return s; };
SC.resol = (st, o = {}) => { const rot = st.show === 'rot', th = st.th, s = scene(st.name || 'resol', 1400, rot ? 880 : 950, camAt([0, 0, rot ? 0.56 : 0.6], rot ? 4.7 : 4.4, -80, 9, 26), o), L = 0.82, O = [0, 0, 0.22], a0 = PI / 2 + th / 2, a1 = PI / 2 - th / 2, P = a => [L * cos(a), 0, O[2] + L * sin(a)];
  const N = 2 ** st.bits, nt = min(N, 64), u = st.quant === false ? st.u : round(st.u * N) / N, ang = a0 + (a1 - a0) * u, S = P(a0), E = P(a1), arcp = []; floor(s, 1.5); pedestal(s, [0, 0], 0.19);
  for (let k = 0; k <= 48; k++) arcp.push(P(a0 + (a1 - a0) * k / 48)); tube(s, arcp, s.L[1], 0.007);
  for (let k = 0; k <= nt; k++) { const a = a0 + (a1 - a0) * k / nt, d = [cos(a), 0, sin(a)]; cyl(s, v(P(a), d, -0.022), v(P(a), d, 0.022), 0.0045, s.L[1]); }
  if (!rot) { box(s, [S[0] - 0.08, 0.03, S[2]], [E[0] + 0.08, 0.03, E[2]], 0.02, 0.03, s.L[2], 1, [0, 1, 0]); for (let k = 0; k <= nt; k++) { const x = S[0] + (E[0] - S[0]) * k / nt; cyl(s, [x, 0.03, S[2] - 0.028], [x, 0.03, S[2] + 0.028], 0.0045, s.L[2]); }
    const cx = S[0] + (E[0] - S[0]) * u; box(s, [cx - 0.045, 0.03, S[2]], [cx + 0.045, 0.03, S[2]], 0.06, 0.06, s.pal.sleeve); arrow(s, [0, -0.02, S[2] - 0.09], [E[0], -0.02, S[2] - 0.09], s.L[2], 0.005); arrow(s, [0, -0.02, S[2] - 0.09], [S[0], -0.02, S[2] - 0.09], s.L[2], 0.005); lab(s, [-0.3, -0.02, S[2] - 0.09], 'd', { m: 1, c: 'b', s: 44, dy: 58, a: 'c' });
    if (u !== st.u) sph(s, [S[0] + (E[0] - S[0]) * st.u, -0.02, S[2]], 0.02, C.pk); }
  if (u !== st.u) sph(s, v(P(a0 + (a1 - a0) * st.u), Y, 0.03), 0.02, C.pk);   // the commanded position, between two encoder steps
  bar(s, O, P(ang), 0.028, s.L[1]); rj(s, O, Y); sph(s, P(ang), 0.03, s.pal.ink); arcV(s, v(O, Y, 0.09), 0.17, a1, a0, s.pal.ink); arcV(s, v(O, Y, 0.09), 0.17, a0, a1, s.pal.ink);
  const m = P(ang); lab(s, [0, -0.09, O[2] + 0.17], 'θ', { m: 1, s: 46, dy: -16, a: 'c' }); lab(s, [m[0] / 2, 0, (m[2] + O[2]) / 2], 'l', { m: 1, c: 'o', s: 48, dx: 34, dy: 12 }); lab(s, P(PI / 2), 'lθ', { m: 1, c: 'o', s: 48, dy: -24, a: 'c' }); return s; };

/* ---- page 1.4 ---- */
const ARRV = { RRR: [[0, 0, 0.5], 6.0], RRP: [[0, 0, 0.5], 5.6], SCARA: [[0, 0, 0.32], 4.8], RPP: [[0, 0, 0.45], 5.2], PPP: [[0.5, 0.42, 0.4], 4.9] };
SC.arr = (st, o = {}) => { const type = st.type, A = RB.ARR[type], K = A.kin(RB.arrQ(type, st.q)), [tg, dist] = ARRV[type], s = scene('arr' + type, 1400, 1100, camAt(tg, dist, st.az === undefined ? -58 : st.az, st.el === undefined ? 22 : st.el, 28), o); floor(s, 1.6, [tg[0], tg[1]]);
  if (st.ws !== false) cached(s, 'arr' + type, t => { const Lm = A.lim.map((l, i) => A.unit[i] === 'deg' ? [rad(l[0]), rad(l[1])] : [l[0], l[1]]);
    /* two joint settings can reach the same surface (SCARA elbow left or right, articulated arm leaning forward or back): keep the two sheets a hair apart, or they flicker */
    const tipW = q => { const p = RB.arrTip(type, q); if (type === 'SCARA') p[2] += 0.006 * q[1]; if (type === 'RRR') { const k = 1 + 0.012 * q[1]; p[0] *= k; p[1] *= k; p[2] = 0.45 + (p[2] - 0.45) * k; } return p; };
    for (let j = 0; j < 3; j++) { const ot = [0, 1, 2].filter(k => k !== j); for (const e of [0, 1]) grid(t, 26, 26, (u, w) => { const q = []; q[j] = Lm[j][e]; q[ot[0]] = Lm[ot[0]][0] + (Lm[ot[0]][1] - Lm[ot[0]][0]) * u; q[ot[1]] = Lm[ot[1]][0] + (Lm[ot[1]][1] - Lm[ot[1]][0]) * w; return tipW(q); }, C.b, 0.13); }
    RB.arrWire(type, 40).filter(w => w.edge).forEach(w => tube(t, w.line, t.L[2], 0.0035)); });
  if (type === 'PPP') { box(s, [0, 0, 0.025], [1, 0, 0.025], 0.12, 0.05, s.pal.base, 1, [0, 0, 1]); shadow(s, [0, 0, 0.02], [1, 0, 0.02], 0.06); } else pedestal(s, [0, 0], 0.06, 0.1); if (type === 'RPP') cyl(s, [0, 0, 0], [0, 0, 0.97], 0.022, s.pal.cap);
  K.pts.slice(0, -1).forEach((p, i) => bar(s, p, K.pts[i + 1], A.seg[i] <= 1 ? 0.036 : 0.03, s.L[A.seg[i]]));
  K.axes.forEach((a, i) => { if (a.t === 'R') rj(s, a.o, a.d); else pj(s, a.o, a.d); arrow(s, v(a.o, a.d, -0.2), v(a.o, a.d, 0.36), s.pal.z, 0.0055); if (a.t === 'R') arc(s, v(a.o, a.d, 0.23), a.d, 0.075, 0.4, 5.7, s.pal.ink, 0.0045);
    lab(s, v(a.o, a.d, 0.36), 'Z', { sub: String(i + 1), m: 1, c: 'z', s: 34, dx: i ? 12 : -12, dy: 4, a: i ? 'l' : 'r' }); lab(s, a.o, a.t === 'R' ? 'θ' : 'd', { sub: String(i + 1), m: 1, s: 34, dx: 40, dy: 44 }); });
  sph(s, K.pts[K.pts.length - 1], 0.034, C.pk); xyz(s, st, [0, 0, 0], K.pts[K.pts.length - 1], { axes: 0.36, r: 0.036 }); return s; };
SC.fivebar = (st, o = {}) => { const s = scene('fivebar', 1300, 1000, camAt([0, 0.33, 0.05], 3.7, -90, 54, 28), o), { c, La, Lb } = CH1.FB, z0 = 0.11, F = RB.fivebar(c, La, Lb, st.t1, st.t2), P3 = p => [p[0], p[1], z0]; floor(s, 1.3, [0, 0.3]);
  cached(s, 'fivebar', t => { const g = new Set(), h = 0.012; for (let i = 0; i <= 260; i++) for (let j = 0; j <= 260; j++) { const r = RB.fivebar(c, La, Lb, rad(10 + 160 * i / 260), rad(10 + 160 * j / 260)); if (r.ok && r.P[1] > 0.04) g.add(flr(r.P[0] / h + 1000) + ',' + flr(r.P[1] / h + 1000)); }
    cells(t, (x, y) => g.has(flr(x / h + 1000) + ',' + flr(y / h + 1000)), 0.84, 0.004, C.b, 0.3, 140); });
  const m1 = [-c, 0, z0], m2 = [c, 0, z0]; [m1, m2].forEach(m => pedestal(s, [m[0], 0], z0 - 0.02, 0.06)); box(s, [-c, 0, 0.03], [c, 0, 0.03], 0.07, 0.04, s.pal.base, 1, [0, 0, 1]);
  bar(s, m1, P3(F.A), 0.026, s.L[1]); bar(s, m2, P3(F.B), 0.026, s.L[1]); s.fail = !F.ok;
  if (F.ok) { bar(s, P3(F.A), P3(F.P), 0.022, s.L[2]); bar(s, P3(F.B), P3(F.P), 0.022, s.L[2]); sph(s, P3(F.P), 0.036, C.pk); if (st.xyz) xyz(s, st, [0, 0, z0], P3(F.P), { head: T('จุดเครื่องมือ เทียบกึ่งกลางฐาน', 'tool point, from the middle of the base'), r: 0.038 }); else lab(s, P3(F.P), T('จุดเครื่องมือ', 'tool point'), { c: 'pk', dx: 30, dy: -14 }); }
  [m1, m2].forEach(m => rj(s, m, [0, 0, 1], { r: 0.05, h: 0.05, c: s.L[1] })); [F.A, F.B].forEach(p => rj(s, P3(p), [0, 0, 1], { r: 0.036, h: 0.045, c: s.pal.cap }));
  lab(s, m1, 'θ', { sub: '1', m: 1, s: 42, dx: -34, dy: 70, a: 'r' }); lab(s, m2, 'θ', { sub: '2', m: 1, s: 42, dx: 34, dy: 70 }); lab(s, [0, -0.02, 0], T('มอเตอร์สองตัวอยู่ที่ฐาน', 'two motors on the base'), { c: 'o', a: 'c', dy: 120, s: 26 }); return s; };
SC.ik2 = (st, o = {}) => { const ctr = st.ctr || [0.35, 0.33], s = scene(st.name || 'ik2', 1300, 1050, camAt([ctr[0], ctr[1], 0.05], st.dist || 3.9, -90, 56, 28), o), { l1, l2, x, y } = st, z0 = 0.11, e1 = st.elbow || 1, q = RB.ik2(l1, l2, x, y, e1), X = st.ax || [0.95, 0.85]; floor(s, 1.5, ctr);
  arrow(s, [-0.15, 0, 0.01], [X[0], 0, 0.01], C.x, 0.006); arrow(s, [0, -0.15, 0.01], [0, X[1], 0.01], C.y, 0.006); circle(s, l1 + l2, 0.006, s.pal.cap, 0.003); if (abs(l1 - l2) > 1e-6 && st.inner) circle(s, abs(l1 - l2), 0.006, s.pal.cap, 0.003);
  lab(s, [X[0], 0, 0.01], 'x', { m: 1, c: 'x', s: 44, dx: 10, dy: 36 }); lab(s, [0, X[1], 0.01], 'y', { m: 1, c: 'y', s: 44, dx: -30, dy: 4, a: 'r' }); s.fail = !q.ok;
  if (!q.ok) { const t = Math.atan2(y, x), far = x * x + y * y > l1 * l1; harm(s, [l1, l2], [t, far ? 0 : PI], { al: 0.45, grip: false }); pedestal(s, [0, 0], z0 - 0.02); sph(s, [x, y, z0], 0.03, C.rd); return s; }
  if (st.both !== false) { const q2 = RB.ik2(l1, l2, x, y, -e1); harm(s, [l1, l2], [q2.t1, q2.t2], { al: 0.28, grip: false }); }
  const r = harm(s, [l1, l2], [q.t1, q.t2], { grip: false }), e = r.pts[1], zt = z0 + 0.055; sph(s, [x, y, z0], 0.03, C.pk); dashed(s, [x, y, 0.01], [x, 0, 0.01]); dashed(s, [x, y, 0.01], [0, y, 0.01]);
  if (abs(q.t1) > 0.05) arcH(s, [0, 0, zt], 0.16, 0, q.t1, s.pal.ink); dashed(s, [e[0], e[1], zt], [e[0] + 0.24 * cos(q.t1), e[1] + 0.24 * sin(q.t1), zt]); if (abs(q.t2) > 0.05) arcH(s, [e[0], e[1], zt], 0.13, q.t1, q.t1 + q.t2, s.pal.ink);
  lab(s, [0.24 * cos(q.t1 / 2), 0.24 * sin(q.t1 / 2), zt], 'θ', { sub: '1', m: 1, s: 42, a: 'c', dy: 12 }); lab(s, [e[0] + 0.2 * cos(q.t1 + q.t2 / 2), e[1] + 0.2 * sin(q.t1 + q.t2 / 2), zt], 'θ', { sub: '2', m: 1, s: 42, a: 'c', dy: 12 });
  lab(s, [e[0] / 2, e[1] / 2, z0], 'a', { sub: '1', m: 1, c: 'o', s: 42, dx: -44, a: 'r' }); lab(s, [(e[0] + x) / 2, (e[1] + y) / 2, z0], 'a', { sub: '2', m: 1, c: 'b', s: 42, dy: -34, a: 'c' }); if (st.xyz) xyz(s, st, [0, 0, z0], [x, y, z0], { axes: 0, path: false, r: 0.032 }); else lab(s, [x, y, z0], '(x, y)', { m: 1, c: 'pk', s: 40, dx: 26, dy: -10 }); return s; };

/* ================= live views for the pages (browser only) =================
   CH1.view.name(ctx, W, H, state, palette): draws the 3-D scene with gl3d.js, then the labels and a few 2-D notes on top.
   Without WebGL it falls back to the flat drawing of ch1.js. The camera of the last frame is kept in state._v for CH1.pick. */
if (typeof document === 'undefined') return;
const G = RB.G, KEY = { ink: 'ink', mut: 'muted', o: 'l1', b: 'l2', p: 'l3', x: 'x', y: 'y', z: 'z', pk: 'sel', gr: 'good', am: 'warn' }, V = CH1.view = {};
/* the picture, then its labels, moved off the picture and off each other where needed (S3.place, as on the slides), with leader lines */
const symW = (c, L, size) => { c.save(); c.font = `italic 500 ${size * 1.05}px ${RB.SERIF}`; let w = c.measureText(L.t).width; if (L.sub) { c.font = `500 ${Math.round(size * 1.05 * 0.68)}px ${RB.SERIF}`; w += c.measureText(L.sub).width + 0.5; } c.restore(); return w; };
const txtW = (c, L, size) => { c.save(); c.font = `${L.b ? 700 : 500} ${size}px ${RB.FONT}`; const w = c.measureText(L.t).width; c.restore(); return w; };
const paint = (c, W, H, s, P, st, res) => { global.R3.draw(c, W, H, s); const k = W / s.ow;
  const items = s.labels.map(L => { const q = S3.proj(s.cam, W, H, L.p), size = max(11, L.s * 1.3 * k), w = L.m ? symW(c, L, size) : txtW(c, L, size), x = q[0] + L.dx * k; return { L, size, w, x: L.a === 'c' ? x - w / 2 : L.a === 'r' ? x - w : x, y: q[1] + L.dy * k, ax: q[0], ay: q[1] }; });
  S3.place(c, W, H, items, P.bg, st ? (st._lm || (st._lm = [])) : null, res); items.forEach(it => { if (it.lead) S3.leader(c, it, P.muted); });
  items.forEach(({ L, X, Y, size }) => { const col = P[KEY[L.c]] || P.ink; if (L.m) G.sym(c, L.t, L.sub, X, Y, { color: col, size: size * 1.05, align: 'left', halo: P.bg }); else G.text(c, L.t, X, Y, { color: col, size, weight: L.b ? 700 : 500, align: 'left', halo: P.bg, haloW: 3.5 }); }); };
/* reserve(W, H): boxes the after-drawing covers (S3.place keeps labels out of them) */
const mk = (name, fb, after, reserve) => { V[name] = (c, W, H, st, P) => { if (!global.R3 || !global.R3.ok()) { st._v = null; if (fb) CH1.draw[fb](c, W, H, st, P); return; }
    const s = SC[name](st, { dark: P === RB.DARK }); if (st.ov) st.ov(s, st);   // the open step of the page's solution (S3.OV)
    S3.floorGrid(s);   // the faint 0.1 m grid (scene3d.js), in the arm's plane for the planar arms
    st._v = { cam: s.cam, W, H }; const ny = name === 'loop' ? H - 8 : 17;   /* the grid's note, top right (bottom right under page 1.1's three boxes) */
    paint(c, W, H, s, P, st, [...(reserve ? reserve(W, H) : []), [W - 120, ny - 17, W, ny + 9]]); G.text(c, T('ตาราง 0.1 m', 'grid 0.1 m'), W - 10, ny, { color: P.muted, size: 11.5, align: 'right', halo: P.bg }); if (after) after(c, W, H, st, P, s); }; };
/* the point of the table (height z0) under a pointer position, for dragging a target */
CH1.pick = (st, p, z0 = 0.11) => st._v ? S3.unproject(st._v.cam, st._v.W, st._v.H, p.x, p.y, z0) : null;
const pillAt = (c, t, col, y = 18) => G.pill(c, t, 10, y, { fill: RB.rgba(col, 0.16), stroke: col, color: col, size: 12.5 });
mk('loop', 'loop', (c, W, H, s, P) => { const { l1, l2 } = CH1.LOOP, names = [T('รับรู้', 'SENSE'), T('คิด', 'THINK'), T('กระทำ', 'ACT')], subs = [T('กล้องวัดตำแหน่ง', 'camera finds the part'), T('คำนวณมุมข้อต่อ', 'compute joint angles'), T('มอเตอร์ขับแขน', 'motors move the arm')];
  const on = [s.sense, s.think, s.act], cols = [P.l2, P.l3, P.l1], bw = min(168, W * 0.28), gap = (W - 3 * bw) / 4;
  names.forEach((n, i) => { const x = gap + i * (bw + gap), hot = on[i] && s.stage === i, col = on[i] ? cols[i] : P.muted; c.save(); c.beginPath(); if (c.roundRect) c.roundRect(x, 8, bw, 42, 9); else c.rect(x, 8, bw, 42); c.fillStyle = RB.rgba(P.bg, 0.86); c.fill(); c.fillStyle = RB.rgba(col, hot ? 0.34 : 0.1); c.fill(); c.strokeStyle = col; c.lineWidth = hot ? 2.6 : 1.4; if (!on[i]) c.setLineDash([5, 4]); c.stroke(); c.restore();
    G.text(c, n, x + bw / 2, 26, { color: on[i] ? P.ink : P.muted, size: 15, weight: 700, align: 'center' }); G.text(c, on[i] ? subs[i] : T('ปิดอยู่', 'switched off'), x + bw / 2, 42, { color: P.muted, size: 11.5, align: 'center' }); if (i < 2) G.arrow(c, x + bw + 5, 29, x + bw + gap - 5, 29, on[i] ? P.muted : P.faint, 2, 8); });
  const f = RB.fk2(l1, l2, s.q[0], s.q[1]), e = Math.hypot(f.x - s.tx, f.y - s.ty), hit = e < 0.025; G.pill(c, hit ? T('ปลายแขนถึงชิ้นงาน ✓', 'the tool is on the part ✓') : T(`ปลายแขนห่างชิ้นงาน ${round(e * 100)} cm`, `the tool is ${round(e * 100)} cm from the part`), 10, H - 18, { fill: RB.rgba(hit ? P.good : P.warn, 0.16), stroke: hit ? P.good : P.warn, color: hit ? P.good : P.warn, size: 12.5 }); },
  (W, H) => [[0, 0, W, 58], [0, H - 36, Math.min(W, 260), H]]);   // the three boxes on top and the note at the bottom left
mk('jobs', null, (c, W, H, s, P) => G.text(c, T('โปรแกรม = รายการจุดที่สอน', 'program = list of taught points'), 10, 20, { color: P.muted, size: 12.5, weight: 600 }));
mk('chain', 'chain', (c, W, H, s, P) => { if (s.title) G.text(c, s.types.join(''), 14, 30, { color: P.ink, size: 24, weight: 800, mono: true }); });
mk('dof', 'dof'); mk('ws2', 'ws2'); mk('wristS', 'wrist');
mk('dex', 'dex', (c, W, H, s, P, sc) => { const g = sc.good, v = g === 1 ? [T('ไปถึงได้ทุกทิศ: อยู่ในปริภูมิทำงานแบบคล่องแคล่ว', 'every direction works: in the dexterous workspace'), P.good] : g ? [T(`ไปถึงได้บางทิศ (${round(100 * g)}%): ไปถึงได้ แต่ไม่คล่องแคล่ว`, `only some directions (${round(100 * g)}%): reachable, not dexterous`), P.warn] : [T('ไปไม่ถึง: อยู่นอกปริภูมิทำงาน', 'out of reach: outside the workspace'), P.bad]; pillAt(c, v[0], v[1]); });
mk('path', 'path', (c, W, H, s, P) => { const dv = CH1.pathDev(s), n = max(2, s.n | 0), y0 = H - 34; G.line(c, 12, y0, 40, y0, P.sel, 2.4); G.text(c, T(`จุดต่อจุด ${n} จุดสอน: ออกนอกแนวมากสุด ${isNaN(dv) ? '—' : RB.u(dv * 1000, 1)} mm`, `point-to-point, ${n} taught points: up to ${isNaN(dv) ? '—' : RB.u(dv * 1000, 1)} mm off the seam`), 46, y0 + 4, { color: P.sel, size: 12.5, weight: 600, halo: P.bg });
  G.line(c, 12, y0 + 19, 40, y0 + 19, P.good, 2); G.text(c, T('เส้นทางต่อเนื่อง: อยู่บนแนวเชื่อมตลอด', 'continuous path: on the seam all the way'), 46, y0 + 23, { color: P.good, size: 12.5, weight: 600, halo: P.bg }); });
mk('resol', 'resol', (c, W, H, s, P) => { const N = 2 ** s.bits; if (N > 64) G.text(c, T(`${N} ขั้น: วาดขีดเพียง 64 ขีด`, `${N} steps: only 64 ticks are drawn`), W - 10, H - 10, { color: P.muted, size: 11.5, align: 'right' }); });
mk('arr', 'arr', (c, W, H, s, P) => { if (s.title !== false) G.text(c, s.type === 'SCARA' ? 'SCARA · RRP' : RB.ARR[s.type].joints, 12, 28, { color: P.ink, size: 22, weight: 800, mono: true }); });
mk('fivebar', 'fivebar', (c, W, H, s, P, sc) => { if (sc.fail) G.pill(c, T('ประกอบไม่ได้: ก้านปลายสั้นเกินกว่าจะมาบรรจบกัน', 'cannot close: the outer links do not meet'), W / 2, 22, { align: 'center', fill: RB.rgba(P.bad, 0.16), stroke: P.bad, color: P.bad, size: 12.5 }); });
mk('ik2', 'ik2', (c, W, H, s, P, sc) => { if (sc.fail) pillAt(c, T('นอกปริภูมิทำงาน: |D| > 1 ไม่มีคำตอบ', 'outside the workspace: |D| > 1, no solution'), P.bad); });
})(typeof window !== 'undefined' ? window : globalThis);
