/* ch3_3d.js — the 3-D scenes of chapter 3 (forward kinematics). CH3.S3.name(state, {dark}) returns a scene table (scene3d.js);
   the pages draw them with CH2.sim / CH2.view (ch2_3d.js looks scenes up in CH3.S3 too), and
   IRE_Slides/build/coppelia/gen_scenes_ch3.mjs renders the same tables in CoppeliaSim for the slides.
   The central drawing is a Denavit–Hartenberg skeleton: for each row the link runs a_{i−1} along X_{i−1} to the next joint axis,
   then d_i along that axis Z_i to the origin of frame {i}. Needs robot.js, ch2.js, scene3d.js, ch2_3d.js, ch3.js. */
(function (global) {
'use strict';
const RB = global.RB, S3 = global.S3, CH2 = global.CH2, CH3 = global.CH3, NUM = global.NUM, C = S3.C, { sin, cos, PI, abs, max, min, hypot } = Math, rad = RB.rad, v = S3.v;
const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
const { bar, cyl, sph, box, arrow, tube, dashed, floor, lab, scene, shadow, rj, pedestal, grip } = S3;
const { frame, vec, angArc, turn, view, cam, camFit, axc } = CH2.H;
const SC = CH3.S3 = {}, I = RB.I3(), O0 = [0, 0, 0];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]], P = T4 => RB.Tp(T4), col = (T4, j) => RB.col(RB.TR(T4), j), len = a => hypot(a[0], a[1], a[2]);
const SUB = ['₀', '₁', '₂', '₃', '₄', '₅', '₆'];
const linkCol = (s, k) => k ? s.L[1 + (k - 1) % 3] : s.L[0];
/* a two-headed dimension arrow, set off from the part it measures by `off` */
const dim = (s, a, b, c, off = [0, 0, 0], r = 0.004) => { const A = v(a, off), B = v(b, off); if (len(sub(B, A)) < 0.02) return null; arrow(s, A, B, c, r); arrow(s, B, A, c, r); dashed(s, a, A, c, 0.0022, 5); dashed(s, b, B, c, 0.0022, 5); return v(A, sub(B, A), 0.5); };

/* ---- the DH skeleton of a table. F = frames in world coordinates (F[0] = {0}); op: frames (indices to draw), fl (axis length), lab (write X_i, Z_i),
   tags ({i} at the origins), axes (long joint-axis lines), show (row k: mark a_{k−1}, α_{k−1}, d_k, θ_k), grip ('x' | 'z': the tool direction), y (draw Y too), al (see-through: a second solution drawn as a ghost) ---- */
const drawDH = CH3.drawDH = (s, rows, F, op = {}) => { const n = rows.length, fl = op.fl || 0.16;
  for (let i = 1; i <= n; i++) { const r = rows[i - 1], Op = P(F[i - 1]), Xp = col(F[i - 1], 0), Oi = P(F[i]), Zi = col(F[i], 2), Q = v(Op, Xp, op.conv === 'c' ? 0 : r.a), fixed = !r.j;
    if (op.conv === 'c') { /* classical: d along Z_{i−1}, then a along X_i */ const Zp = col(F[i - 1], 2), M = v(Op, Zp, r.d + (r.j === 'P' ? (op.q[i - 1] || 0) : 0)); if (len(sub(M, Op)) > 1e-6) bar(s, Op, M, 0.016, linkCol(s, i), op.al); if (len(sub(Oi, M)) > 1e-6) bar(s, M, Oi, 0.02, linkCol(s, i), op.al); continue; }
    if (len(sub(Q, Op)) > 1e-6) bar(s, Op, Q, i === 1 ? 0.03 : 0.022, linkCol(s, i - 1), op.al);
    if (len(sub(Oi, Q)) > 1e-6) bar(s, Q, Oi, r.j === 'P' ? 0.014 : 0.02, fixed ? linkCol(s, i - 1) : linkCol(s, i), op.al);
    if (r.j === 'R') rj(s, i === 1 ? Oi : Q, Zi, { r: op.jr || 0.04, h: op.jh || 0.04, al: op.al });
    else if (r.j === 'P') box(s, v(Q, Zi, -0.05), v(Q, Zi, 0.05), 0.06, 0.06, s.pal.sleeve, op.al || 1, col(F[i - 1], 0)); }
  if (op.axes) for (let i = 1; i <= n; i++) { if (!rows[i - 1].j) continue; const Oi = P(F[i]), Zi = col(F[i], 2); cyl(s, v(Oi, Zi, -op.axes), v(Oi, Zi, op.axes), 0.003, s.pal.z, 0.45); }
  const fr = op.frames || F.map((_, i) => i);
  fr.forEach(i => { const O = P(F[i]); [0, 2].concat(op.y ? [1] : []).forEach(j => { arrow(s, O, v(O, col(F[i], j), fl), axc(s, j), 0.0045);
      if (op.lab !== false) lab(s, v(O, col(F[i], j), fl + 0.04), 'XYZ'[j], { sub: String(i), m: 1, c: 'xyz'[j], s: op.ls || 28, a: 'c', dy: 10, ...((op.labAt && op.labAt[i] && op.labAt[i][j]) || {}) }); });
    sph(s, O, 0.011, s.pal.ink); if (op.tags) lab(s, O, `{${i}}`, { s: 26, a: 'r', dx: -12, dy: 30, ...((op.tagAt && op.tagAt[i]) || {}) }); });
  if (op.grip) { const E = F[n], d = col(E, op.grip === 'z' ? 2 : 0), sd = col(E, 1); if (op.soft) S3.softGrip(s, P(E), d, sd, { k: 0.8, al: op.al }); else grip(s, P(E), d, sd, { w: 0.03, L: 0.06, al: op.al }); }   // soft: the GP8's three-finger soft gripper
  if (op.show) { const k = op.show, r = rows[k - 1], Op = P(F[k - 1]), Xp = col(F[k - 1], 0), Zp = col(F[k - 1], 2), Ok = P(F[k]), Zk = col(F[k], 2), Xk = col(F[k], 0), Q = v(Op, Xp, r.a);
    const m1 = dim(s, Op, Q, C.am, RB.sc(Zp, -0.06)); if (m1) lab(s, m1, 'a', { sub: (k - 1) + '', m: 1, c: 'am', s: 34, a: 'c', dy: 36 });
    const m2 = dim(s, Q, Ok, C.p, RB.sc(Xp, -0.07)); if (m2) lab(s, m2, 'd', { sub: k + '', m: 1, c: 'p', s: 34, a: 'r', dx: -14 });
    const ma = angArc(s, Q, Zp, Zk, 0.12, C.am, 0.004); if (ma) { dashed(s, Q, v(Q, Zp, 0.18), C.am, 0.0025, 7); lab(s, ma, 'α', { sub: (k - 1) + '', m: 1, c: 'am', s: 34, a: 'c', dx: 16, dy: -10 }); }
    const mt = angArc(s, Ok, Xp, Xk, 0.12, C.pk, 0.004); if (mt) { dashed(s, Ok, v(Ok, Xp, 0.18), C.pk, 0.0025, 7); lab(s, mt, 'θ', { sub: k + '', m: 1, c: 'pk', s: 34, a: 'c', dx: 14, dy: -8 }); } } };

/* ---- where the end of the arm is: the origin of the last frame {N} in {0} (column 4 of ⁰_NT), marked with a dot and written next to it as x, y, z,
   with the dashed path x along X₀, then y along Y₀, then z along Z₀ that adds up to it. F: frames in world coordinates (F[0] = {0}); o.dx, o.dy move the readout ---- */
const tipXYZ = CH3.tipXYZ = (s, F, o = {}) => { if (F.length < 2) return null; const B = F[0];
  return S3.xyz(s, P(B), P(F[F.length - 1]), { ...o, A: [0, 1, 2].map(j => col(B, j)), head: T('ปลายแขน เทียบ {0}', 'end of arm in {0}') }); };

/* where each example stands and how it is seen */
const EXV = { p3r: { T0: [0, 0, 0.11], cam: [[0.35, 0.3, 0.05], 3.0, -90, 58], grip: 'x', base: true },
  rpr: { T0: [0, 0, 0.45], cam: [[0.05, -0.2, 0.3], 3.0, 28, 18], grip: 'z', base: true },
  rprHW: { T0: [0, 0, 0], cam: [[0.05, -0.2, 0.3], 3.0, 28, 18], grip: 'z', base: false },
  twist: { T0: [0, 0, 0.35], cam: [[0.3, 0.1, 0.4], 3.2, -62, 20], grip: 'x', base: true },
  arm4: { T0: [0, 0, 0], cam: [[0.35, 0.1, 0.45], 3.6, -62, 18], grip: 'x', base: false },
  scara: { T0: [0, 0, 0.6], cam: [[0.3, 0.15, 0.35], 3.4, -60, 22], grip: 'z', base: true },
  gp8: { T0: [0, 0, 0.33], cam: [[0.22, 0, 0.5], 4.0, -60, 16], grip: 'z', soft: true, base: true } };
CH3.EXV = EXV;   // chapter 4 shows the same arms
/* the frames of example `ex` (CH3.EX) at joint values q, in world coordinates */
CH3.exFrames = (ex, q, conv) => { const E = CH3.EX[ex === 'rprHW' ? 'rpr' : ex], L = E.L, rows = ex === 'rprHW' ? E.hwRows(L) : conv === 'c' ? E.rowsC(L) : E.rows(L), V = EXV[ex];
  const full = ex === 'p3r' && q.length > 3 ? [...rows, E.tool(L)] : rows; return { rows: full, F: CH3.frames(full, q, conv, RB.trans(...V.T0)) }; };

/* ---- one example arm: st.ex, st.q (joint values), st.tool (3R: add the tool frame {4}), st.show, st.frames, st.conv ---- */
/* the points a picture of the arm must hold: the base, every frame origin and the tips of its axes, the end of the tool (for st.fit, S3.fitCam) */
const armPts = (F, T0, grip) => { const E = F[F.length - 1], g = grip ? v(P(E), col(E, grip === 'z' ? 2 : 0), 0.1) : P(E); return [[T0[0] - 0.15, T0[1], 0], [T0[0] + 0.15, T0[1], 0], [T0[0], T0[1] - 0.15, 0], [T0[0], T0[1] + 0.15, 0], ...F.map(P), ...F.flatMap(T4 => [v(P(T4), col(T4, 0), 0.22), v(P(T4), col(T4, 2), 0.22)]), g]; };   // with the X and Z arrows of every frame and their names
SC.dh = (st = {}, o = {}) => { const ex = st.ex || 'p3r', V = EXV[ex], q = st.q || CH3.EX[ex === 'rprHW' ? 'rpr' : ex].q, c0 = V.cam, w = st.w || 1300, h = st.h || 1000;
  const { rows, F } = CH3.exFrames(ex, st.tool ? [...q, 0] : q, st.conv);   // st.fit (the pages): the camera follows the arm, which the sliders can swing anywhere
  const s = scene(st.name || 'dh_' + ex, w, h, st.fit ? camFit(st, armPts(F, V.T0, V.grip), c0[2], c0[3], w, h, { m: 0.16, minD: 2.4 }) : cam(st, c0[0], c0[1], c0[2], c0[3]), o); floor(s, 1.3, [c0[0][0], c0[0][1]]); if (V.base) pedestal(s, [V.T0[0], V.T0[1]], V.T0[2] - 0.02, 0.09);
  drawDH(s, rows, F, { frames: st.frames, lab: st.lab, tags: !!st.tags, show: st.show, grip: st.grip === false ? null : V.grip, soft: V.soft, axes: st.axes, fl: st.fl, labAt: st.labAt, tagAt: st.tagAt, conv: st.conv, q });
  if (st.fk) { const On = P(F[F.length - 1]), O = P(F[0]); CH2.H.dvec(s, O, On, C.pk, { key: 'pk', t: 'T', pre: '0', sub: 'N', s: 38, at: 0.5, dx: 26, dy: 20, a: 'l', r: 0.0055 }); }
  if (st.xyz) s.tip = tipXYZ(s, F, st.xyzAt);   // the pages turn this on; the slide figures leave it off (the examples' answers are the tip position)
  s.F = F; s.rows = rows; return s; };

/* ---- any DH table (the table explorer of page 3.2): st.rows (Craig's convention), st.q, st.h0 (height of {0}), st.toolDir ---- */
SC.free = (st = {}, o = {}) => { const rows = st.rows, q = st.q || [], h0 = st.h0 === undefined ? 0.4 : st.h0, F = CH3.frames(rows, q, null, RB.trans(0, 0, h0)), pts = CH3.frames(rows, [], null, RB.trans(0, 0, h0)).map(P);   // without st.fit: framed on the zero pose; with st.fit (the page) the camera follows the arm
  const lo = [0, 1].map(k => min(...pts.map(p => p[k]))), hi = [0, 1].map(k => max(...pts.map(p => p[k]))), ctr = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, max(0.25, (max(...pts.map(p => p[2])) + 0) / 2)], span = max(0.6, hi[0] - lo[0], hi[1] - lo[1], max(...pts.map(p => p[2])));
  const s = scene(st.name || 'free', st.w || 1300, st.h || 1000, st.fit && rows.length ? camFit(st, armPts(F, [0, 0, 0], st.toolDir || 'x'), -60, 22, st.w || 1300, st.h || 1000, { m: 0.16, minD: 2.2 }) : cam(st, ctr, 1.5 + span * 2.0, -60, 22), o); floor(s, span * 0.9 + 0.4, [ctr[0], ctr[1]]); if (h0 > 0.05) pedestal(s, [0, 0], h0 - 0.02, 0.08);
  drawDH(s, rows, F, { grip: st.toolDir || 'x', q, fl: 0.14, ls: 26, show: st.show }); if (st.xyz) s.tip = tipXYZ(s, F, st.xyzAt); s.F = F; return s; };

/* ---- the link description (Craig figures 3.4 and 3.5): joint axes i−1, i, i+1 as long lines, links along their common normals.
   st: a, al (a_{i−1}, α_{i−1}), d, th (d_i, θ_i), show 'a' (length and twist) | 'd' (offset and angle) | 'all' ---- */
SC.link = (st = {}, o = {}) => { const al = st.al === undefined ? rad(-40) : st.al, a = st.a === undefined ? 0.5 : st.a, d = st.d === undefined ? 0.22 : st.d, th = st.th === undefined ? rad(55) : st.th, show = st.show || 'all';
  const s = scene(st.name || 'link', st.w || 1300, st.h || 1000, cam(st, [0.42, 0.14, 0.34], 3.0, -55, 20), o), T0 = RB.trans(0, 0, 0.25);
  const rows = [{ al, a, d, th }, { al: rad(30), a: 0.42, d: 0, th: 0 }], F = [T0, RB.Tm(T0, RB.dhT(al, a, th, d))]; F.push(RB.Tm(F[1], RB.dhT(rows[1].al, rows[1].a, 0, 0)));
  const ax = (T4, nm, k) => { const O = P(T4), Z = col(T4, 2); cyl(s, v(O, Z, -0.36), v(O, Z, 0.36), 0.009, s.pal.base); lab(s, v(O, Z, 0.38), T('แกน ', 'axis ') + nm, { c: 'mut', s: 26, a: 'c', dy: -8 }); };
  ax(F[0], 'i − 1'); ax(F[1], 'i'); ax(F[2], 'i + 1');
  const Q1 = v(P(F[0]), col(F[0], 0), a), Q2 = v(P(F[1]), col(F[1], 0), rows[1].a);
  bar(s, P(F[0]), Q1, 0.02, linkCol(s, 1)); bar(s, P(F[1]), Q2, 0.02, linkCol(s, 2)); if (abs(d) > 1e-3) cyl(s, Q1, P(F[1]), 0.012, s.pal.cap);
  lab(s, v(P(F[0]), col(F[0], 0), a * 0.35), T('ก้าน i − 1', 'link i − 1'), { c: 'o', s: 26, a: 'c', dy: -30 }); lab(s, v(P(F[1]), col(F[1], 0), rows[1].a * 0.7), T('ก้าน i', 'link i'), { c: 'b', s: 26, a: 'c', dy: -30 });
  [0, 1].forEach(i => { const O = P(F[i]); [0, 2].forEach(j => { arrow(s, O, v(O, col(F[i], j), 0.2), axc(s, j), 0.0055); lab(s, v(O, col(F[i], j), 0.25), 'XZ'[j / 2], { sub: i ? 'i' : 'i−1', m: 1, c: 'xyz'[j], s: 30, a: 'c', dy: 10 }); }); sph(s, O, 0.012, s.pal.ink); });
  const Zp = col(F[0], 2), Zi = col(F[1], 2), Xp = col(F[0], 0), Xi = col(F[1], 0), Oi = P(F[1]);
  if (show !== 'd') { const m = dim(s, P(F[0]), Q1, C.am, RB.sc(Zp, -0.08)); if (m) lab(s, m, 'a', { sub: 'i−1', m: 1, c: 'am', s: 36, a: 'c', dy: 40 });
    const ma = angArc(s, Q1, Zp, Zi, 0.16, C.am, 0.0045); dashed(s, Q1, v(Q1, Zp, 0.26), C.am, 0.003, 9); if (ma) lab(s, ma, 'α', { sub: 'i−1', m: 1, c: 'am', s: 36, a: 'l', dx: 14, dy: -8 }); }
  if (show !== 'a') { const m = dim(s, Q1, Oi, C.p, RB.sc(Xp, -0.1)); if (m) lab(s, m, 'd', { sub: 'i', m: 1, c: 'p', s: 36, a: 'r', dx: -12 });
    const mt = angArc(s, Oi, Xp, Xi, 0.15, C.pk, 0.0045); dashed(s, Oi, v(Oi, Xp, 0.25), C.pk, 0.003, 9); if (mt) lab(s, mt, 'θ', { sub: 'i', m: 1, c: 'pk', s: 36, a: 'l', dx: 12, dy: -6 }); }
  return s; };

/* ---- the link transform as four steps (Craig 3.5): {i−1} → {R} → {Q} → {P} → {i}; st.u = 0…4 (progress), the parameters as in SC.link ---- */
SC.steps4 = (st = {}, o = {}) => { const al = st.al === undefined ? rad(-40) : st.al, a = st.a === undefined ? 0.5 : st.a, d = st.d === undefined ? 0.22 : st.d, th = st.th === undefined ? rad(55) : st.th, u = st.u === undefined ? 4 : st.u;
  const s = scene(st.name || 'steps4', st.w || 1300, st.h || 1000, cam(st, [0.28, 0.1, 0.34], 2.5, -55, 20), o), T0 = RB.trans(0, 0, 0.25), Fs = CH3.steps4(al, a, th, d, u).map(X => RB.Tm(T0, X)), names = ['i−1', 'R', 'Q', 'P', 'i'];
  const Fe = CH3.steps4(al, a, th, d, 4).map(X => RB.Tm(T0, X)); [Fe[0], Fe[4]].forEach(T4 => { const O = P(T4), Z = col(T4, 2); cyl(s, v(O, Z, -0.35), v(O, Z, 0.45), 0.008, s.pal.base, 0.5); });
  frame(s, I, P(Fs[0]), { name: 'i−1', ref: true, L: 0.24, r: 0.005, ls: 28, lab: [{}, { dy: 34 }, {}] });
  const k = min(4, Math.ceil(u - 1e-9)); for (let i = 1; i < k; i++) frame(s, RB.TR(Fs[i]), P(Fs[i]), { L: 0.2, r: 0.0035, al: 0.3 });
  for (let i = 1; i < k; i++) lab(s, P(Fs[i]), `{${names[i]}}`, { s: 24, c: 'mut', a: 'r', dx: -10, dy: -10, b: false });
  if (u > 0) { const Fk = Fs[k], last = k === 4 && u >= 3.999; frame(s, RB.TR(Fk), P(Fk), { name: last ? 'i' : names[k], L: 0.22, r: 0.006, ls: 28 }); }
  if (u > 0.02 && abs(al) > 0.01) { const m = angArc(s, P(Fs[0]), col(Fs[0], 2), col(Fs[1], 2), 0.13, C.am, 0.004); if (m) lab(s, m, 'α', { sub: 'i−1', m: 1, c: 'am', s: 32, a: 'l', dx: 10 }); }
  if (u > 1.02) { const m = v(P(Fs[1]), sub(P(Fs[2]), P(Fs[1])), 0.5); arrow(s, P(Fs[1]), P(Fs[2]), C.am, 0.005); lab(s, m, 'a', { sub: 'i−1', m: 1, c: 'am', s: 32, a: 'c', dy: 36 }); }
  if (u > 2.02 && abs(th) > 0.01) { const m = angArc(s, P(Fs[2]), col(Fs[2], 0), col(Fs[3], 0), 0.13, C.pk, 0.004); if (m) lab(s, m, 'θ', { sub: 'i', m: 1, c: 'pk', s: 32, a: 'l', dx: 10 }); }
  if (u > 3.02) { const m = v(P(Fs[3]), sub(P(Fs[4]), P(Fs[3])), 0.5); arrow(s, P(Fs[3]), P(Fs[4]), C.p, 0.005); lab(s, m, 'd', { sub: 'i', m: 1, c: 'p', s: 32, a: 'r', dx: -12 }); }
  s.Fs = Fs; return s; };

/* ---- the first link (Craig 3.4): {0} is chosen to coincide with {1} when the first joint variable is zero; st.kind 'R' (θ_1 varies, d_1 = 0) or 'P' (d_1 varies, θ_1 = 0), st.q the joint value ---- */
SC.first = (st = {}, o = {}) => { const kind = st.kind || 'R', qv = st.q === undefined ? (kind === 'R' ? rad(40) : 0.18) : st.q, s = scene(st.name || 'first' + kind, 900, 1000, cam(st, [0, 0, 0.42], 2.4, 28, 18), o), O = [0, 0, 0.42]; floor(s, 0.6);
  pedestal(s, [0, 0], O[2] - 0.16, 0.08); const T1 = kind === 'R' ? RB.T(RB.Rz(qv), O) : RB.T(I, v(O, [0, 0, 1], qv));
  if (kind === 'R') { rj(s, O, [0, 0, 1], { r: 0.06, h: 0.05 }); bar(s, O, v(O, RB.col(RB.TR(T1), 0), 0.3), 0.022, s.L[1]); }
  else { box(s, v(O, [0, 0, 1], -0.1), O, 0.09, 0.09, s.pal.sleeve, 1, [1, 0, 0]); bar(s, v(O, [0, 0, 1], -0.05), P(T1), 0.02, s.L[1]); bar(s, P(T1), v(P(T1), [1, 0, 0], 0.28), 0.02, s.L[1]); }
  frame(s, I, O, { name: '0', ref: true, L: 0.36, r: 0.005, ls: 30 }); frame(s, RB.TR(T1), P(T1), { name: '1', L: 0.28, ls: 30, lab: [{ dy: 34 }, {}, { dx: 22 }] });
  if (kind === 'R') { const m = angArc(s, O, [1, 0, 0], RB.col(RB.TR(T1), 0), 0.14, C.pk, 0.0045); if (m) lab(s, m, 'θ', { sub: '1', m: 1, c: 'pk', s: 36, a: 'c', dy: 40 }); }
  else { const m = v(O, [0, 0, 1], qv / 2); arrow(s, v(O, [-1, 0, 0], 0.12), v(P(T1), [-1, 0, 0], 0.12), C.p, 0.004); lab(s, v(m, [-1, 0, 0], 0.12), 'd', { sub: '1', m: 1, c: 'p', s: 36, a: 'r', dx: -12 }); }
  s.F = [RB.T(I, O), T1]; return s; };   // the frames {0}, {1} in world coordinates, for the page's step overlays

/* ---- the same SCARA with Craig's frames (left) and the classical frames (right); the arms reach to the right, so the pair sits left of centre ---- */
SC.convs = (st = {}, o = {}) => { const E = CH3.EX.scara, L = E.L, q = st.q || E.q, c3 = view([0.09, 0.05, 0.36], 5.6, -62, 20), s = scene(st.name || 'convs', 1800, 860, c3, o), A = S3.axes(c3), right = [-A.x[0], -A.x[1], 0];
  [['m', -0.95], ['c', 0.55]].forEach(([cv, off]) => { const base = v([0, 0, 0], right, off), T0 = RB.Tm(RB.trans(base[0], base[1], 0.6)), rows = cv === 'c' ? E.rowsC(L) : E.rows(L), F = CH3.frames(rows, q, cv, T0), G = CH3.frames(E.rows(L), q, null, T0);
    floor(s, 0.75, [base[0], base[1]]); pedestal(s, [base[0], base[1]], 0.58, 0.09);
    drawDH(s, E.rows(L), G, { frames: [], lab: false, grip: 'z', q });   // the robot itself: the same in both pictures
    F.forEach((T4, i) => { const O = P(T4); [0, 2].forEach(j => { arrow(s, O, v(O, col(T4, j), 0.17), axc(s, j), 0.0045); lab(s, v(O, col(T4, j), 0.21), cv === 'c' ? 'xz'[j / 2] : 'XZ'[j / 2], { sub: String(i), m: 1, c: 'xyz'[j], s: 24, a: 'c', dy: 8 }); }); sph(s, O, 0.01, s.pal.ink); });
    lab(s, base, cv === 'c' ? T('แบบดั้งเดิม (Spong)', 'classical (Spong)') : T('แบบดัดแปลง (Craig)', 'modified (Craig)'), { a: 'c', dy: 120, s: 30 }); });
  return s; };
})(typeof window !== 'undefined' ? window : globalThis);
