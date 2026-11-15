/* ch4_3d.js — the 3-D scenes of chapter 4 (inverse kinematics). CH4.S3.name(state, {dark}) returns a scene table (scene3d.js);
   the pages draw them with CH2.sim / CH2.view (ch2_3d.js looks scenes up in CH4.S3 too), and
   IRE_Slides/build/coppelia/gen_scenes_ch4.mjs renders the same tables in CoppeliaSim for the slides.
   The arms are the ones of chapter 3 (CH3.EX, drawn by CH3.drawDH) plus the homework RRP arm (CH4.rrp). Needs robot.js, ch2.js, ch3.js, ch4.js,
   scene3d.js, ch2_3d.js, ch3_3d.js. */
(function (global) {
'use strict';
const RB = global.RB, S3 = global.S3, CH2 = global.CH2, CH3 = global.CH3, CH4 = global.CH4, C = S3.C, { sin, cos, PI, abs, max, hypot, atan2 } = Math, rad = RB.rad, v = S3.v;
const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
const { bar, cyl, sph, box, arrow, dashed, floor, lab, scene, rj, pedestal, grip, harm, tube, mesh } = S3;
const { frame, angArc, view, cam, camFit } = CH2.H;
const SC = CH4.S3 = {}, EXV = CH3.EXV, P = T4 => RB.Tp(T4), col = (T4, j) => RB.col(RB.TR(T4), j), sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const GOAL = () => T('เป้าหมาย', 'goal');
const ELBOW = { p3r: 2, arm4: 3, scara: 2, irb: 3, rrp: 2 };   // the frame at each arm's elbow: where a solution's number is written

/* ---- an arm of chapter 3 (or the homework RRP arm) at joint values q: its frames and how it is seen ---- */
const ARM = { rrp: { T0: [0, 0, 0], cam: [[0.3, 0.12, 0.42], 3.3, -58, 16], grip: 'z', base: false } };
const armOf = (ex, q) => { if (ex === 'rrp') { const rows = CH4.rrp.rows(CH4.rrp.L); return { rows, F: CH3.frames(rows, q, null, RB.trans(0, 0, 0)), V: ARM.rrp }; }
  const r = CH3.exFrames(ex, q); return { ...r, V: EXV[ex] }; };
const drawArm = (s, ex, q, op = {}) => { const { rows, F, V } = armOf(ex, q); if (V.base && !op.al) pedestal(s, [V.T0[0], V.T0[1]], V.T0[2] - 0.02, 0.09);
  CH3.drawDH(s, rows, F, { frames: op.frames || [], lab: op.lab, grip: op.grip === false ? null : V.grip, al: op.al, q, fl: op.fl || 0.15, ls: op.ls || 26, labAt: op.labAt }); return F; };
/* the goal: a see-through pink ball where the last frame must go, named */
const goalMark = (s, p, o = {}) => { sph(s, p, o.r || 0.034, C.pk, 0.35); lab(s, p, o.t || GOAL(), { c: 'pk', s: o.s || 26, dx: o.dx === undefined ? 30 : o.dx, dy: o.dy === undefined ? 44 : o.dy, b: true }); };

/* ---- one inverse solution drawn solid, the others of the same goal as ghosts. st: ex, q (the solution shown), ghost [q…], goal (true: mark it),
   frames (indices of the solid arm's frames to draw, default the base and the last), tags ([text for the solid arm, text for each ghost]) ---- */
SC.sol = (st = {}, o = {}) => { const ex = st.ex || 'p3r', q = st.q || CH4.EX[ex === 'irb' ? 'irb' : ex].q, { F, V } = armOf(ex, q), n = F.length - 1, w = st.w || 1300, h = st.h || 1000, c0 = st.cam || V.cam;
  const pts = st.fit ? [F, ...(st.ghost || []).map(g => armOf(ex, g).F)].flatMap(G => G.map(P)).concat([[V.T0[0] - 0.15, V.T0[1], 0], [V.T0[0] + 0.15, V.T0[1], 0]]) : null;
  const s = scene(st.name || 'sol_' + ex, w, h, pts ? camFit(st, pts, c0[2], c0[3], w, h, { m: 0.2, minD: 2.4 }) : cam(st, c0[0], c0[1], c0[2], c0[3]), o); floor(s, 1.4, [c0[0][0], c0[0][1]]);
  (st.ghost || []).forEach((g, k) => { const G = drawArm(s, ex, g, { al: 0.3 }); if (st.tags && st.tags[k + 1]) lab(s, P(G[ELBOW[ex]]), st.tags[k + 1], { c: 'mut', s: 24, dx: 18, dy: -18, b: true }); });
  drawArm(s, ex, q, { frames: st.frames || [0, n], labAt: st.labAt });
  if (st.tags && st.tags[0]) lab(s, P(F[ELBOW[ex]]), st.tags[0], { c: 'ink', s: 24, dx: 18, dy: -18, b: true });
  if (st.goal !== false) goalMark(s, P(F[n]), st.goalAt || {});
  if (st.wc) { const W = P(F[4]); lab(s, W, T('จุดศูนย์กลางข้อมือ', 'wrist centre'), { c: 'pk', s: 24, dx: -30, dy: -40, a: 'r' }); }
  if (st.xyz) s.tip = CH3.tipXYZ(s, F, st.xyzAt);
  s.F = F; return s; };

/* ---- the workspace of the two-link arm (Craig 4.2, figure 4.1): reachable = the ring between |L1 − L2| and L1 + L2, dexterous = the origin when L1 = L2 ---- */
SC.reach = (st = {}, o = {}) => { const l1 = st.l1 || 0.4, l2 = st.l2 || 0.4, R = l1 + l2, r = abs(l1 - l2), z = 0.012, q = st.q || [rad(35), rad(70)], s = scene(st.name || 'reach', 1100, 1000, view([0, 0, 0.05], R * 5.4, -90, 62), o); floor(s, R * 1.25);
  const ring = (r0, r1, c, al, zz) => mesh(s, Array.from({ length: 90 }, (_, i) => { const a = 2 * PI * i / 90, b = 2 * PI * (i + 1) / 90; return [[r0 * cos(a), r0 * sin(a), zz], [r1 * cos(a), r1 * sin(a), zz], [r1 * cos(b), r1 * sin(b), zz], [r0 * cos(b), r0 * sin(b), zz]]; }), c, al);
  ring(r, R, C.b, 0.28, z); tube(s, RB.ring([0, 0, z], [0, 0, 1], R, 0, 2 * PI, 72), s.L[2], 0.005); if (r > 1e-6) tube(s, RB.ring([0, 0, z], [0, 0, 1], r, 0, 2 * PI, 48), C.pk, 0.005);
  if (st.goal) { const G = st.goal, sol = [1, -1].map(e => RB.ik2(l1, l2, G[0], G[1], e)).filter(x => x.ok); s.n = !sol.length ? 0 : Math.abs(sol[0].t2) < 1e-6 ? 1 : 2;
    if (!sol.length) { const t = atan2(G[1], G[0]), far = hypot(G[0], G[1]) > l1 + l2; harm(s, [l1, l2], [t, far ? 0 : PI], { al: 0.45, grip: false }); }
    else { if (s.n === 2) harm(s, [l1, l2], [sol[1].t1, sol[1].t2], { al: 0.3, grip: false }); harm(s, [l1, l2], [sol[0].t1, sol[0].t2], { grip: false }); }
    sph(s, [G[0], G[1], 0.11], 0.026, s.n ? C.pk : C.rd); }
  else harm(s, [l1, l2], q, { grip: false });
  const a = rad(-35); arrow(s, [0, 0, z + 0.004], [R * cos(a), R * sin(a), z + 0.004], s.L[2], 0.005); lab(s, [0.62 * R * cos(a), 0.62 * R * sin(a), z], 'L₁ + L₂', { c: 'b', s: 30, dx: 10, dy: 40 });
  lab(s, [0, -0.86 * R, 0], T('ไปถึงได้ (reachable)', 'reachable'), { c: 'b', s: 28, a: 'c' });
  if (r > 1e-6) { arrow(s, [0, 0, z + 0.004], [-r, 0, z + 0.004], C.pk, 0.005); lab(s, [-r, 0, z], '|L₁ − L₂|', { c: 'pk', s: 28, a: 'r', dx: -14, dy: 10 }); lab(s, [0, -r, z], T('รูตรงกลางไปไม่ถึง', 'the hole: unreachable'), { c: 'pk', s: 24, a: 'c', dy: 40 }); }
  else { sph(s, [0, 0, z + 0.01], 0.022, C.gr); lab(s, [0, -0.25 * R, z], T('คล่องแคล่ว = จุดกำเนิดจุดเดียว', 'dexterous: the origin only'), { c: 'gr', s: 26, a: 'c', dy: 20 }); }
  return s; };

/* ---- choosing a solution (Craig 4.2, figure 4.3): the arm at A, a goal B with two solutions; the closer one would hit an obstacle ---- */
SC.obst = (st = {}, o = {}) => { const l1 = 0.4, l2 = 0.35, qA = st.qA || [rad(-15), rad(95)], B = st.B || [-0.12, 0.6], s = scene(st.name || 'obst', 1300, 1000, view([0.08, 0.32, 0.05], 3.3, -90, 60), o); floor(s, 1.0, [0.08, 0.3]);
  const sols = [1, -1].map(e => RB.ik2(l1, l2, B[0], B[1], e)).map(r => [r.t1, r.t2]), near = CH4.nearest(sols, qA).s, far = sols.find(x => x !== near), f2 = q => RB.fk2(l1, l2, q[0], q[1]);
  harm(s, [l1, l2], near, { al: 0.3, grip: false }); harm(s, [l1, l2], far, { al: 0.3, grip: false }); const A = harm(s, [l1, l2], qA, { grip: false });
  const e = f2(near); box(s, [e.x1, e.y1, 0], [e.x1, e.y1, 0.2], 0.13, 0.13, C.rd, 0.85); lab(s, [e.x1, e.y1, 0.2], T('สิ่งกีดขวาง', 'obstacle'), { c: 'ink', s: 26, dx: 40, dy: -30 });
  sph(s, A.pts[2], 0.025, s.pal.ink); lab(s, A.pts[2], 'A', { s: 40, dx: 24, dy: 10 }); sph(s, [B[0], B[1], 0.11], 0.025, C.pk); lab(s, [B[0], B[1], 0.11], 'B', { c: 'pk', s: 40, dx: -36, dy: -10, a: 'r' });
  const ef = f2(far); lab(s, [ef.x1, ef.y1, 0.11], T('ไกลกว่า แต่ไม่ชน', 'farther, but clear'), { c: 'gr', s: 24, dx: -24, dy: 36, a: 'r' }); return s; };

/* ---- redundancy: a three-link arm with only the tool point given (two numbers, three joints) reaches it in infinitely many ways (self-motion) ---- */
SC.redund = (st = {}, o = {}) => { const L = [0.5, 0.4, 0.2], p = st.p || [0.62, 0.38], phis = st.phis || [-50, -10, 30, 70], s = scene(st.name || 'redund', 1200, 1000, view([0.32, 0.25, 0.05], 3.2, -90, 60), o); floor(s, 1.0, [0.3, 0.25]);
  const one = (d, al) => { const ph = rad(d), w = [p[0] - L[2] * cos(ph), p[1] - L[2] * sin(ph)], r = RB.ik2(L[0], L[1], w[0], w[1], 1); if (r.ok) harm(s, L, [r.t1, r.t2, ph - r.t1 - r.t2], { al, grip: false }); return r.ok; };
  if (st.phi === undefined) phis.forEach((d, k) => one(d, k === phis.length - 1 ? undefined : 0.3)); else { phis.forEach(d => one(d, 0.18)); s.ok = one(st.phi); }
  sph(s, [p[0], p[1], 0.11], 0.03, C.pk); lab(s, [p[0], p[1], 0.11], T('จุดปลายเครื่องมือเดียวกัน', 'the same tool point'), { c: 'pk', s: 26, dx: 30, dy: -10 }); return s; };

/* ---- the geometric solution of the two-link part (Craig 4.4, figure 4.8): the triangle base – elbow – wrist, β, ψ and the angle at the elbow ---- */
SC.geo = (st = {}, o = {}) => { const L1 = 0.5, L2 = 0.4, g = st.g || CH4.EX.p3r.goal, x = g[0], y = g[1], sol = CH4.p3rGeo(L1, L2, x, y, 0), z0 = 0.11, s = scene(st.name || 'geo', 1200, 1000, view([0.3, 0.33, 0.05], 3.1, -90, 62), o); floor(s, 1.0, [0.3, 0.3]);
  if (!sol.ok) { const t = atan2(y, x); harm(s, [L1, L2], [t, hypot(x, y) > L1 + L2 ? 0 : PI], { al: 0.45, grip: false }); sph(s, [x, y, z0], 0.026, C.rd); lab(s, [x, y, z0], T('ไปไม่ถึง', 'out of reach'), { c: 'ink', s: 28, dx: 24 }); s.fail = true; return s; }
  const q = sol.sols[st.k || 0];
  const A = harm(s, [L1, L2], [q[0], q[1]], { grip: false }), E = A.pts[1], W = A.pts[2], O = [0, 0, z0 + 0.03];
  arrow(s, [0, 0, z0], [0.62, 0, z0], C.x, 0.005); lab(s, [0.62, 0, z0], 'X', { sub: '0', m: 1, c: 'x', s: 30, dx: 14, dy: 10 }); dashed(s, [0, 0, z0], W, s.pal.ink, 0.004, 21); lab(s, v([0, 0, z0], sub(W, [0, 0, z0]), 0.55), '√(x² + y²)', { s: 26, dx: -20, dy: -16, a: 'r' });
  const mb = angArc(s, O, [1, 0, 0], sub(W, [0, 0, z0]), 0.2, C.am, 0.004); if (mb) lab(s, mb, 'β', { m: 1, c: 'am', s: 36, dx: 14, dy: 12 });
  const mp = angArc(s, v(O, [0, 0, 0.005]), sub(E, [0, 0, z0]), sub(W, [0, 0, z0]), 0.3, C.pk, 0.004); if (mp) lab(s, mp, 'ψ', { m: 1, c: 'pk', s: 36, dx: 10, dy: -4 });
  const me = angArc(s, v(E, [0, 0, 0.03]), sub([0, 0, z0], E), sub(W, E), 0.12, C.p, 0.004); if (me) lab(s, me, '180° − θ', { sub: '2', m: 1, c: 'p', s: 30, dx: 20, dy: 4 });
  lab(s, v([0, 0, z0], sub(E, [0, 0, z0]), 0.5), 'L', { sub: '1', m: 1, c: 'o', s: 32, dx: 22, dy: 26 }); lab(s, v(E, sub(W, E), 0.5), 'L', { sub: '2', m: 1, c: 'b', s: 32, dx: 22, dy: 10 });
  sph(s, W, 0.024, C.pk); lab(s, W, '(x, y)', { c: 'pk', s: 30, dx: 22, dy: -10 }); return s; };

/* ---- the RRR arm of chapter 3 solved with geometry (old part 2): the top view gives θ1 and r1, the side view the triangle r1, ẑ, r2 ---- */
SC.rrrGeo = (st = {}, o = {}) => { const E = CH4.EX.arm4, q = st.q || E.q, L = E.L, { F } = armOf('arm4', q), s = scene(st.name || 'rrrGeo', 1300, 1000, cam(st, [0.3, 0.2, 0.42], 3.6, -48, 16), o); floor(s, 1.3, [0.3, 0.15]);
  drawArm(s, 'arm4', q, { frames: [0], lab: true }); const p = P(F[F.length - 1]), S = [0, 0, L.L1 + L.L2], foot = [p[0], p[1], 0.005], Sp = [p[0], p[1], S[2]];
  dashed(s, [0, 0, 0.005], foot, s.pal.ink, 0.003, 17); dashed(s, foot, p, s.pal.dash, 0.0025, 21); lab(s, v([0, 0, 0.005], foot, 0.55), 'r', { sub: '1', m: 1, s: 32, dx: 10, dy: 34 });
  const m1 = angArc(s, [0, 0, 0.01], [1, 0, 0], foot, 0.25, C.am, 0.004); if (m1) lab(s, m1, 'θ', { sub: '1', m: 1, c: 'am', s: 32, dx: 12, dy: 20 });
  dashed(s, S, Sp, C.am, 0.003, 17); dashed(s, Sp, p, C.am, 0.003, 9); dashed(s, S, p, C.pk, 0.0035, 21); lab(s, v(Sp, sub(p, Sp), 0.5), 'ẑ', { m: 1, c: 'am', s: 32, dx: 14 }); lab(s, v(S, sub(p, S), 0.55), 'r', { sub: '2', m: 1, c: 'pk', s: 32, dx: -10, dy: -18, a: 'r' });
  sph(s, p, 0.026, C.pk); lab(s, p, GOAL(), { c: 'pk', s: 26, dx: 26, dy: -16 }); return s; };

/* ---- homework 4: the RRP arm with its lengths and joint variables marked (no answers) ---- */
SC.hw = (st = {}, o = {}) => { const q = st.q || CH4.rrp.q, L = CH4.rrp.L, { F } = armOf('rrp', q), s = scene(st.name || 'hw', 1300, 1000, cam(st, [0.3, 0.12, 0.42], 3.2, -58, 16), o); floor(s, 1.2, [0.3, 0.1]);
  drawArm(s, 'rrp', q, { frames: [0], lab: true }); const O1 = P(F[1]), O2 = P(F[2]), O3 = P(F[3]), Z = [0, 0, 1];
  lab(s, [0, 0, L.L1 / 2], 'L', { sub: '1', m: 1, c: 'o', s: 34, dx: -24, a: 'r' }); lab(s, v(O1, sub(O2, O1), 0.5), 'L', { sub: '2', m: 1, c: 'o', s: 34, dy: -26, a: 'c' });
  lab(s, v(O2, sub(O3, O2), 0.55), 'd', { sub: '3', m: 1, c: 'p', s: 34, dx: 16, dy: 34 });
  const m1 = angArc(s, [0, 0, 0.02], [1, 0, 0], v(O2, Z, -O2[2]), 0.22, C.am, 0.004); if (m1) lab(s, m1, 'θ', { sub: '1', m: 1, c: 'am', s: 34, dx: 10, dy: 24 });
  const hz = sub(O2, O1); const m2 = angArc(s, O2, hz, sub(O3, O2), 0.16, C.pk, 0.004); dashed(s, O2, v(O2, hz, 0.3 / Math.hypot(...hz)), C.pk, 0.003, 9); if (m2) lab(s, m2, 'θ', { sub: '2', m: 1, c: 'pk', s: 34, dx: 14, dy: 6 });
  sph(s, O3, 0.024, C.pk); lab(s, O3, '(x, y, z)', { c: 'pk', s: 28, dx: 24, dy: -12 }); return s; };

/* ---- decoupling (Craig 4.6; old part 2): axes 4, 5, 6 of the IRB 140 meet at the wrist centre. With a tool L_T long, the wrist centre is the goal
   moved back L_T along its own Z: the first three joints place it, the last three turn the tool ---- */
SC.wc = (st = {}, o = {}) => { const X0 = CH4.EX.solve, LT = st.LT === undefined ? X0.LT : st.LT, X = { ...X0, LT }, sw = CH4.irbIK(RB.Tm(X0.G, RB.trans(0, 0, -LT))), ok = sw.filter(x => CH4.inLim(x.q)), pick = CH4.nearest(ok.length ? ok : sw, CH3.EX.irb.zero), q = st.q || (pick ? pick.s.q : X0.best.s.q), { F, V } = armOf('irb', q), T0 = RB.trans(...V.T0), G = RB.Tm(T0, X.G), s = scene(st.name || 'wc', 1300, 1050, cam(st, [0.3, 0.06, 0.5], 2.5, -50, 14), o); floor(s, 1.4, [0.25, 0]);
  drawArm(s, 'irb', q, { frames: [0], grip: false }); const W = P(F[6]), Z6 = col(F[6], 2), Tt = v(W, Z6, X.LT); bar(s, W, Tt, 0.012, s.pal.joint); grip(s, Tt, Z6, col(F[6], 1), { w: 0.03, L: 0.05 });
  frame(s, RB.TR(G), P(G), { name: 'G', L: 0.16, r: 0.0045, ls: 28, lab: [{ dx: 14 }, {}, { dx: 16, dy: 30 }] }); sph(s, W, 0.03, C.pk, 0.55);
  lab(s, W, T('จุดศูนย์กลางข้อมือ', 'wrist centre'), { c: 'pk', s: 26, dx: -36, dy: -36, a: 'r' }); lab(s, v(W, Z6, X.LT / 2), 'L', { sub: 'T', m: 1, s: 28, dx: 22, dy: 6 });
  lab(s, P(F[3]), T('ข้อต่อ 1–3 วางจุดศูนย์กลางข้อมือ', 'joints 1–3 place the wrist centre'), { c: 'b', s: 24, dx: 20, dy: 70, a: 'l' }); lab(s, W, T('ข้อต่อ 4–6 หมุนเครื่องมือ', 'joints 4–6 turn the tool'), { c: 'p', s: 24, dx: 40, dy: -64 });
  s.q = q; return s; };
})(typeof window !== 'undefined' ? window : globalThis);
