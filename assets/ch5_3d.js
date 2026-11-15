/* ch5_3d.js — the 3-D scenes of chapter 5 (the Jacobian). CH5.S3.name(state, {dark}) returns a scene table (scene3d.js);
   the pages draw them with CH2.sim / CH2.view (ch2_3d.js looks scenes up in CH5.S3 too), and
   IRE_Slides/build/coppelia/gen_scenes_ch5.mjs renders the same tables in CoppeliaSim for the slides.
   Every scene leaves what it drew in s.info (the Jacobian, the velocity …) so that a page can print the same numbers under the picture.
   Needs robot.js, ch2.js, ch3.js, ch4.js, ch5.js, scene3d.js, ch2_3d.js, ch3_3d.js. */
(function (global) {
'use strict';
const RB = global.RB, S3 = global.S3, CH2 = global.CH2, CH3 = global.CH3, CH4 = global.CH4, CH5 = global.CH5, C = S3.C, { sin, cos, PI, abs, max, min, hypot, atan2, sign } = Math, rad = RB.rad, v = S3.v, OV = S3.OV;
const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
const { bar, cyl, sph, box, arrow, dashed, floor, lab, scene, rj, pedestal, grip, harm, tube, mesh } = S3;
const { frame, turn, cam, camFit } = CH2.H;
const SC = CH5.S3 = {}, P = T4 => RB.Tp(T4), col = (T4, j) => RB.col(RB.TR(T4), j);
CH5.OV = OV;   // the step overlays live in scene3d.js (shared by every chapter)
const Z0 = 0.11, UP = 0.075, KEY = ['pk', 'p', 'am'], COL = [C.pk, C.p, C.am], SUB = '₁₂₃';   // planar arms move in the plane z = Z0; arrows lie UP above it, over the links
const lift = p => [p[0], p[1], Z0 + UP], to = (a, d, k = 1) => [a[0] + d[0] * k, a[1] + d[1] * k, a[2]], mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
/* forward kinematics of a planar chain of turning joints: the tool point, and every joint */
const fkP = L => q => { let x = 0, y = 0, a = 0; L.forEach((l, i) => { a += q[i]; x += l * cos(a); y += l * sin(a); }); return [x, y]; };
const jointsP = (L, q) => { const out = [[0, 0]]; let a = 0; L.forEach((l, i) => { a += q[i]; const p = out[i]; out.push([p[0] + l * cos(a), p[1] + l * sin(a)]); }); return out; };
/* the camera of a planar arm, seen from above and a little from the front: it follows the arm (st.fit !== false) or stays where fx puts it */
const planCam = (st, pts, w, h, fx) => st.fit === false ? cam(st, fx[0], fx[1], -90, fx[2] || 62) : camFit(st, pts.map(p => [p[0], p[1], p[2] === undefined ? Z0 : p[2]]).concat([[-0.16, 0, 0], [0.16, 0, 0], [0, -0.16, 0], [0, 0.16, 0]]), -90, fx[2] || 62, w, h, { m: fx[3] || 0.17, minD: 2.2 });
const xyz = (s, st, O, tip) => { if (st.xyz) s.tip = S3.xyz(s, O, tip, { head: T('ปลายแขน เทียบฐาน', 'end of arm, from the base'), axes: 0.24, axIdx: [0, 1] }); };
/* a joint rate drawn as a turning arrow about the joint (counter-clockwise seen from above when the rate is positive), named θ̇_i */
const rateArc = (s, p, rate, i, r = 0.075) => { if (abs(rate) < 1e-9) return; turn(s, [p[0], p[1], Z0 + 0.085], [0, 0, sign(rate)], r, COL[i], 0.6, 0.6 + min(5.0, 1.3 + 2.4 * abs(rate)), 0.004);
  lab(s, [p[0], p[1] - r, Z0 + 0.085], 'θ̇', { sub: String(i + 1), m: 1, c: KEY[i], s: 26, a: 'c', dy: 34 }); };

/* ---- a small step of the joints (old part 1 slide 4: δθ ↔ δX). The arm at q (solid) and at q + δq (ghost); from the tool point:
   the step it really makes (green) and the step the Jacobian predicts, J δq (orange). st: L (1, 2 or 3 links), q, dq, show ('both' | 'act' | 'pred'),
   cols (the prediction built from one arrow per joint, J_j δθ_j), arcs (name the joint steps) ---- */
SC.step = (st = {}, o = {}) => { const L = st.L || CH5.two.L, q = st.q || CH5.two.q.slice(0, L.length), dq = st.dq || [rad(10), rad(-15)].slice(0, L.length), fk = fkP(L), q2 = q.map((x, i) => x + dq[i]), p = fk(q), p2 = fk(q2);
  const J = CH5.numJ(fk, q), pred = CH5.mul(J, dq), w = st.w || 1200, h = st.h || 1000, A = lift(p), B = lift(p2), Pd = to(A, pred), show = st.show || 'both';
  const s = scene(st.name || 'step', w, h, planCam(st, [p, p2, [p[0] + pred[0], p[1] + pred[1]], ...jointsP(L, q)], w, h, [[0.3, 0.3, 0.05], 3.4]), o); floor(s, 1.3, [0.3, 0.3]);
  harm(s, L, q2, { al: 0.3, grip: false }); const arm = harm(s, L, q, { grip: false });
  if (st.cols) { let a = [A[0], A[1], A[2] - 0.012]; dq.forEach((d, j) => { if (abs(d) < 1e-9) return; const b = to(a, [J[0][j] * d, J[1][j] * d]); arrow(s, a, b, COL[j], 0.0042); lab(s, mid(a, b), 'J', { sub: String(j + 1), post: ' δθ' + SUB[j], m: 1, c: KEY[j], s: 24, a: 'c', dy: -16 }); a = b; }); }
  if (show !== 'act') { arrow(s, A, Pd, C.o, 0.006); lab(s, Pd, 'J δθ', { m: 1, c: 'o', s: 28, dx: 16, dy: -12 }); }
  if (show !== 'pred') { arrow(s, A, B, C.gr, 0.006); lab(s, B, T('ขยับจริง δX', 'real step δX'), { c: 'gr', s: 24, dx: 16, dy: 22 }); }
  if (st.arcs !== false) { const jt = jointsP(L, q); let a = 0; L.forEach((l, i) => { const a0 = a + q[i], O = [jt[i][0], jt[i][1], Z0 + UP]; if (abs(dq[i]) > 0.02) { const m = S3.angArc(s, O, [cos(a0), sin(a0), 0], [cos(a0 + dq[i]), sin(a0 + dq[i]), 0], 0.2 - 0.03 * i, COL[i], 0.004); if (m) lab(s, m, 'δθ', { sub: String(i + 1), m: 1, c: KEY[i], s: 24, a: 'c', dx: 20 * cos(a0) , dy: -20 * sin(a0) }); } a = a0; }); }
  xyz(s, st, arm.pts[0], arm.pts[L.length]); s.info = { p, p2, J, pred, act: [p2[0] - p[0], p2[1] - p[1]] }; return s; };

/* ---- the planar arm named for the direct derivative (old part 1 slide 8): l1, l2, θ1, θ2 and the tool point (x, y). st.xy: its x and y drawn too ---- */
SC.two = (st = {}, o = {}) => { const L = st.L || CH5.two.L, q = st.q || CH5.two.q, w = st.w || 1200, h = st.h || 1000, jt = jointsP(L, q), p = jt[2], a1 = q[0], a2 = q[0] + q[1], d1 = [cos(a1), sin(a1), 0], d2 = [cos(a2), sin(a2), 0];
  const s = scene(st.name || 'two', w, h, planCam(st, jt.concat([[0.42, 0], [0, 0.42]]), w, h, [[0.3, 0.3, 0.05], 3.4]), o); floor(s, 1.3, [0.3, 0.3]); const arm = harm(s, L, q, { grip: false }), O = [0, 0, Z0 + UP], E = lift(jt[1]), Pt = lift(p);
  arrow(s, [0, 0, Z0 + 0.02], [0.42, 0, Z0 + 0.02], C.x, 0.0045); lab(s, [0.42, 0, Z0], 'X', { sub: '0', m: 1, c: 'x', s: 26, dx: 14, dy: 12 }); arrow(s, [0, 0, Z0 + 0.02], [0, 0.42, Z0 + 0.02], C.y, 0.0045); lab(s, [0, 0.42, Z0], 'Y', { sub: '0', m: 1, c: 'y', s: 26, dx: -6, dy: -14, a: 'r' });
  OV.ang(s, O, [1, 0, 0], d1, 0.2, 'pk', 'θ', { sub: '1', dx: 18, dy: -6 }); OV.ref(s, E, d1, 0.24); OV.ang(s, E, d1, d2, 0.13, 'p', 'θ', { sub: '2', dx: 16, dy: -10 });
  const n1 = [-d1[1], d1[0], 0], n2 = [-d2[1], d2[0], 0]; lab(s, v(mid(O, E), n1, 0.06), 'l', { sub: '1', m: 1, c: 'o', s: 30, a: 'c' }); lab(s, v(mid(E, Pt), n2, 0.06), 'l', { sub: '2', m: 1, c: 'b', s: 30, a: 'c' });
  if (st.xy) { OV.dist(s, [0, 0, Z0 + 0.03], [p[0], 0, Z0 + 0.03], 'x', 'x', { dy: 28 }); OV.dist(s, [p[0], 0, Z0 + 0.03], [p[0], p[1], Z0 + 0.03], 'y', 'y', { dx: 18 }); }
  lab(s, Pt, '(x, y)', { m: 1, c: 'gr', s: 28, dx: 18, dy: -14 }); xyz(s, st, arm.pts[0], arm.pts[2]); s.info = { p }; return s; };

/* ---- joint rates and the velocity of the tool point (v = J θ̇). st: L, q, qd (rad/s), vs (metres of arrow per m/s), cols (one arrow per joint: J_j θ̇_j),
   rad (the line from each joint to the tool point: column j is square to it), path (where the tool point goes in the next st.T seconds at these rates),
   rates (false: no turning arrows at the joints), tool (the last link carries a gripper: the 3R arm of homework 5.1) ---- */
SC.jvel = (st = {}, o = {}) => { const L = st.L || CH5.two.L, n = L.length, q = st.q || (n === 3 ? CH5.p3r.q : CH5.two.q), qd = st.qd || (n === 3 ? CH5.EX.p3r.qd : CH5.EX.two.qd), fk = fkP(L), p = fk(q), J = CH5.numJ(fk, q), vv = CH5.mul(J, qd), vs = st.vs || 0.8;
  const w = st.w || 1200, h = st.h || 1000, jt = jointsP(L, q), A = lift(p), V = to(A, vv, vs), path = st.path ? Array.from({ length: 25 }, (_, k) => { const t = (st.T || 1) * k / 24, e = fk(q.map((x, i) => x + qd[i] * t)); return [e[0], e[1], Z0 + 0.012]; }) : [];
  const s = scene(st.name || 'jvel', w, h, planCam(st, [...jt, [V[0], V[1]], ...path], w, h, [[0.3, 0.3, 0.05], 3.4]), o); floor(s, 1.3, [0.3, 0.3]);
  if (path.length) tube(s, path, s.pal.dash, 0.0035);
  const arm = harm(s, L, q, st.tool ? {} : { grip: false });
  if (st.rad) jt.slice(0, n).forEach((e, j) => { if (abs(qd[j]) > 1e-9) dashed(s, lift(e), A, COL[j], 0.0028, 15); });
  if (st.cols) { let a = [A[0], A[1], A[2] - 0.012]; qd.forEach((d, j) => { if (abs(d) < 1e-9) return; const b = to(a, [J[0][j] * d, J[1][j] * d], vs); arrow(s, a, b, COL[j], 0.0045); lab(s, mid(a, b), 'J', { sub: String(j + 1), post: ' θ̇' + SUB[j], m: 1, c: KEY[j], s: 24, a: 'c', dy: -16 }); a = b; }); }
  if (hypot(vv[0], vv[1]) > 1e-9) { arrow(s, A, V, C.gr, 0.0065); lab(s, V, 'v', { m: 1, c: 'gr', s: 32, dx: 16, dy: -10 }); }
  if (st.rates !== false) jt.slice(0, n).forEach((e, i) => rateArc(s, e, qd[i], i, 0.08 - 0.008 * i));
  xyz(s, st, arm.pts[0], arm.pts[n]); s.info = { p, J, v: vv, phid: qd.reduce((a, b) => a + b, 0) }; return s; };

/* ---- the RP (polar) arm of the class exercise: joint 1 turns a sleeve, the rod slides through it by d2 ---- */
SC.rp = (st = {}, o = {}) => { const q = st.q || CH5.rp.q, qd = st.qd || CH5.EX.rp.qd, vs = st.vs || 0.8, th = q[0], d = q[1], dir = [cos(th), sin(th), 0], nrm = [-sin(th), cos(th), 0], p = [d * dir[0], d * dir[1]], J = CH5.rp.J(q), vv = CH5.mul(J, qd);
  const w = st.w || 1200, h = st.h || 1000, O = [0, 0, Z0], tip = [p[0], p[1], Z0], A = lift(p), V = to(A, vv, vs), s = scene(st.name || 'rp', w, h, planCam(st, [[0, 0], p, [V[0], V[1]], [0.4, 0]], w, h, [[0.2, 0.25, 0.05], 3.0]), o); floor(s, 1.2, [0.2, 0.2]); s.gridZ = Z0;
  pedestal(s, [0, 0], Z0 - 0.02); bar(s, v(O, dir, -0.06), tip, 0.015, s.L[2]); box(s, v(O, dir, 0.06), v(O, dir, 0.18), 0.07, 0.07, s.pal.sleeve, 1, [0, 0, 1]); rj(s, O, [0, 0, 1], { r: 0.046, h: 0.05 }); grip(s, tip, dir, nrm, { w: 0.03, L: 0.05 });
  if (st.names !== false) { OV.ref(s, [0, 0, Z0 + UP], [1, 0, 0], 0.34); OV.ang(s, [0, 0, Z0 + UP], [1, 0, 0], dir, 0.2, 'pk', 'θ', { sub: '1', dx: 18, dy: -6 }); OV.dist(s, v([0, 0, Z0 + UP], nrm, -0.07), v(A, nrm, -0.07), 'p', 'd', { sub: '2', dy: 26 }); }
  if (st.cols) { const a = [A[0], A[1], A[2] - 0.012], b = to(a, [J[0][0] * qd[0], J[1][0] * qd[0]], vs), c = to(b, [J[0][1] * qd[1], J[1][1] * qd[1]], vs);
    if (abs(qd[0]) > 1e-9) { arrow(s, a, b, COL[0], 0.0045); lab(s, mid(a, b), 'J', { sub: '1', post: ' θ̇₁', m: 1, c: 'pk', s: 24, a: 'c', dy: -16 }); } if (abs(qd[1]) > 1e-9) { arrow(s, b, c, COL[1], 0.0045); lab(s, mid(b, c), 'J', { sub: '2', post: ' ḋ₂', m: 1, c: 'p', s: 24, a: 'c', dy: -16 }); } }
  if (hypot(vv[0], vv[1]) > 1e-9) { arrow(s, A, V, C.gr, 0.0065); lab(s, V, 'v', { m: 1, c: 'gr', s: 32, dx: 16, dy: -10 }); }
  if (st.rates !== false) rateArc(s, [0, 0], qd[0], 0, 0.085);
  xyz(s, st, O, tip); s.info = { p, J, v: vv }; return s; };

/* ---- an arm in space with the velocity of its last frame: v (green) and the angular velocity ω (purple), [v; ω] = J0 q̇.
   st: ex ('stan' the Stanford arm's first three joints | 'rrp' the arm of homework 4 | 'gp8'), q, qd, vs (m of arrow per m/s), ws (m per rad/s), om (false: no ω) ---- */
const ARM = { stan: { T0: [0, 0, 0.45], cam: [[0.2, 0.15, 0.42], 3.7, -52, 20], grip: 'z', base: true }, rrp: { T0: [0, 0, 0], cam: [[0.3, 0.12, 0.42], 3.3, -58, 16], grip: 'z', base: false } };
const armOf = (ex, q) => { if (ex === 'stan') { const rows = CH5.stan.rows(CH5.stan.L2); return { rows, F: CH3.frames(rows, q, null, RB.trans(...ARM.stan.T0)), V: ARM.stan }; }
  if (ex === 'rrp') { const rows = CH4.rrp.rows(CH4.rrp.L); return { rows, F: CH3.frames(rows, q, null, RB.trans(0, 0, 0)), V: ARM.rrp }; } const r = CH3.exFrames(ex, q); return { ...r, V: CH3.EXV[ex] }; };
SC.sv = (st = {}, o = {}) => { const ex = st.ex || 'stan', E = CH5.EX[ex], q = st.q || E.q, qd = st.qd || (ex === 'gp8' ? E.qd6 : E.qd), { rows, F, V } = armOf(ex, q), n = F.length - 1, w = st.w || 1300, h = st.h || 1000, c0 = st.cam || V.cam, vs = st.vs || 0.8, ws = st.ws || 0.16;
  const J0 = CH5.J0(rows, q), vw = CH5.mul(J0, qd.concat([0, 0, 0, 0, 0, 0]).slice(0, J0[0].length)), vv = vw.slice(0, 3), om = vw.slice(3), tip = P(F[n]), Ve = v(tip, vv, vs), We = v(tip, om, ws);
  const pts = st.fit ? F.map(P).concat([Ve, [V.T0[0] - 0.15, V.T0[1], 0], [V.T0[0] + 0.15, V.T0[1], 0]]) : null, s = scene(st.name || 'sv_' + ex, w, h, pts ? camFit(st, pts, c0[2], c0[3], w, h, { m: 0.2, minD: 2.4 }) : cam(st, c0[0], c0[1], c0[2], c0[3]), o); floor(s, 1.4, [c0[0][0], c0[0][1]]);
  if (V.base) pedestal(s, [V.T0[0], V.T0[1]], V.T0[2] - 0.02, 0.09);
  CH3.drawDH(s, rows, F, { frames: st.frames || [0], grip: V.grip, soft: V.soft, q, fl: 0.15, ls: 26, labAt: st.labAt });
  if (hypot(...vv) > 1e-9) { arrow(s, tip, Ve, C.gr, 0.007); lab(s, Ve, 'v', { m: 1, c: 'gr', s: 32, dx: 16, dy: -10 }); }
  if (st.om !== false && hypot(...om) > 1e-9) { arrow(s, tip, We, C.p, 0.006); lab(s, We, 'ω', { m: 1, c: 'p', s: 32, dx: 16, dy: 10 }); }
  if (st.xyz) s.tip = CH3.tipXYZ(s, F, st.xyzAt); s.F = F; s.info = { p: tip, v: vv, w: om, J0 }; return s; };

/* ---- how a position is written (old part 2 slides 5–7): the same point P and the same velocity v in Cartesian, cylindrical or spherical coordinates.
   st: p, v, mode ('cart' | 'cyl' | 'sph'), vs ---- */
SC.coords = (st = {}, o = {}) => { const p = st.p || CH5.EX.rep.p, vel = st.v || CH5.EX.rep.v, mode = st.mode || 'cyl', vs = st.vs || 1.6, s = scene(st.name || 'coords', 1200, 1000, cam(st, [0.2, 0.2, 0.14], 2.7, -52, 26), o); floor(s, 1.0, [0.2, 0.2]);
  const O = [0, 0, 0], foot = [p[0], p[1], 0], th = atan2(p[1], p[0]), rho = hypot(...p), er = [cos(th), sin(th), 0], et = [-sin(th), cos(th), 0], k = 0.17, un = (pt, d, key, sub) => { arrow(s, pt, v(pt, d, k), ({ am: C.am, pk: C.pk, p: C.p })[key], 0.0045); lab(s, v(pt, d, k), 'e', { sub, m: 1, c: key, s: 24, dx: 12, dy: -6 }); };
  [[[1, 0, 0], C.x, 'X', 'x'], [[0, 1, 0], C.y, 'Y', 'y'], [[0, 0, 1], s.pal.z, 'Z', 'z']].forEach(([d, c, t, key]) => { arrow(s, O, v(O, d, 0.55), c, 0.005); lab(s, v(O, d, 0.55), t, { m: 1, c: key, s: 28, dx: 12, dy: -8 }); });
  sph(s, p, 0.022, s.pal.ink); lab(s, p, 'P', { m: 1, s: 32, dx: -16, dy: -16, a: 'r' });
  if (mode === 'cart') { OV.dist(s, O, [p[0], 0, 0], 'x', 'x', { dy: 26 }); OV.dist(s, [p[0], 0, 0], foot, 'y', 'y', { dx: 16, dy: 16 }); OV.dist(s, foot, p, 'z', 'z', { dx: 18 }); }
  else if (mode === 'cyl') { OV.dist(s, O, foot, 'am', 'ρ', { dy: 28 }); OV.dist(s, foot, p, 'p', 'z', { dx: 18 }); OV.ang(s, O, [1, 0, 0], er, 0.2, 'pk', 'θ', { dx: 16, dy: 14 }); un(p, er, 'am', 'ρ'); un(p, et, 'pk', 'θ'); un(p, [0, 0, 1], 'p', 'z'); }
  else { const ph = Math.acos(p[2] / rho), ep = [cos(th) * cos(ph), sin(th) * cos(ph), -sin(ph)], ur = [p[0] / rho, p[1] / rho, p[2] / rho]; OV.dist(s, O, p, 'am', 'ρ', { dx: -16, dy: -10 }); dashed(s, O, foot, s.pal.dash, 0.003, 13); dashed(s, foot, p, s.pal.dash, 0.003, 9);
    OV.ang(s, O, [1, 0, 0], er, 0.2, 'pk', 'θ', { dx: 16, dy: 14 }); OV.ang(s, O, [0, 0, 1], ur, 0.24, 'p', 'φ', { dx: 14, dy: -10 }); un(p, ur, 'am', 'ρ'); un(p, et, 'pk', 'θ'); un(p, ep, 'p', 'φ'); }
  if (hypot(...vel) > 1e-9) { arrow(s, p, v(p, vel, vs), C.gr, 0.0065); lab(s, v(p, vel, vs), 'v', { m: 1, c: 'gr', s: 32, dx: 16, dy: -8 }); }
  s.info = { cyl: CH5.rep.cyl(p), sph: CH5.rep.sph(p) }; return s; };

/* ---- the notation for velocities (Craig example 5.1, old part 2 slides 13–15): a fixed frame {U}, a train with {T} and a car with {C}, both driving along X_U.
   st.t (0…1): how far they have driven (the page plays it); the arrows are the speeds, 100 and 30 km/h ---- */
SC.train = (st = {}, o = {}) => { const X = CH5.EX.train, t = st.t || 0, k = 0.011, xT = -0.25 + X.vT * k * 0.9 * t, xC = -0.05 + X.vC * k * 0.9 * t, yT = 0.5, yC = -0.42, s = scene(st.name || 'train', 1400, 900, cam(st, [0.55, 0.05, 0.12], 5.4, -68, 30), o); floor(s, 2.2, [0.6, 0]);
  const wheel = (x, y, r) => cyl(s, [x, y - 0.012, r], [x, y + 0.012, r], r, s.pal.ink), grey = s.dark ? [0.3, 0.33, 0.42] : [0.86, 0.88, 0.91];
  box(s, [-1.0, yT, 0.004], [2.6, yT, 0.004], 0.26, 0.008, grey, 1, [0, 0, 1]); box(s, [-1.0, yC, 0.004], [2.6, yC, 0.004], 0.34, 0.008, grey, 1, [0, 0, 1]);                         // the track and the road
  [0, -0.58].forEach((dx, i) => { const x = xT + dx; box(s, [x - 0.26, yT, 0.15], [x + 0.26, yT, 0.15], 0.16, 0.17, i ? [0.2, 0.52, 0.76] : C.b, 1, [0, 0, 1]); [-0.17, 0.17].forEach(a => [-0.085, 0.085].forEach(b => wheel(x + a, yT + b, 0.035))); });
  box(s, [xT - 0.32, yT, 0.12], [xT - 0.26, yT, 0.12], 0.05, 0.05, s.pal.ink, 1, [0, 0, 1]);
  box(s, [xC - 0.19, yC, 0.085], [xC + 0.19, yC, 0.085], 0.17, 0.08, C.o, 1, [0, 0, 1]); box(s, [xC - 0.11, yC, 0.16], [xC + 0.07, yC, 0.16], 0.15, 0.07, [0.96, 0.6, 0.35], 1, [0, 0, 1]); [-0.12, 0.12].forEach(a => [-0.09, 0.09].forEach(b => wheel(xC + a, yC + b, 0.04)));
  frame(s, RB.I3(), [0, 0, 0.006], { name: 'U', L: 0.42, tag: true, labels: false, ts: 34, tagAt: { dx: -30, dy: 34 } });
  frame(s, X.RT, [xT, yT, 0.24], { name: 'T', L: 0.26, tag: true, labels: false, ts: 34, tagAt: { dx: -26, dy: -18 } }); frame(s, X.RC, [xC, yC, 0.2], { name: 'C', L: 0.26, tag: true, labels: false, ts: 34, tagAt: { dx: -28, dy: -16 } });
  const aT = [xT + 0.3, yT, 0.15], aC = [xC + 0.22, yC, 0.085]; arrow(s, aT, v(aT, [1, 0, 0], X.vT * k), C.gr, 0.008); lab(s, v(aT, [1, 0, 0], X.vT * k), '100 km/h', { c: 'gr', s: 26, dx: 14, dy: -8 }); arrow(s, aC, v(aC, [1, 0, 0], X.vC * k), C.gr, 0.008); lab(s, v(aC, [1, 0, 0], X.vC * k), '30 km/h', { c: 'gr', s: 26, dx: 14, dy: -8 });
  return s; };

/* ---- a singular pose of the two-link arm (Craig 5.8, examples 5.4 and 5.5). The two columns of J drawn at the tool point: when the arm is straight they
   lie on one line and the tool point cannot move along the arm. st: x (the tool point on the X axis, the arm found by inverse kinematics) or q, v (the wanted
   velocity), ring (false: no workspace boundary) ---- */
SC.sing = (st = {}, o = {}) => { const L = st.L || CH5.two.L, R = L[0] + L[1], q = st.q || (() => { const r = RB.ik2(L[0], L[1], min(R, max(abs(L[0] - L[1]), st.x === undefined ? 0.8 : st.x)), 0, -1); return [r.t1, r.t2]; })(), vd = st.v || [0.1, 0], vs = st.vs || 2.4, js = st.js || 0.32;
  const w = st.w || 1300, h = st.h || 1000, p = CH5.two.P(L, q), J = CH5.two.J(L, q), det = CH5.two.det(L, q), A = lift(p), s = scene(st.name || 'sing', w, h, cam(st, [0.2, 0.05, 0.05], 4.7, -90, 62), o); floor(s, 1.5, [0.2, 0]);
  if (st.ring !== false) { tube(s, RB.ring([0, 0, 0.012], [0, 0, 1], R, 0, 2 * PI, 90), s.L[2], 0.0045); lab(s, [R * cos(rad(125)), R * sin(rad(125)), 0.012], T('ขอบปริภูมิทำงาน', 'workspace boundary'), { c: 'b', s: 24, a: 'r', dx: -12, dy: -8 }); }
  harm(s, L, q, { grip: false });
  [0, 1].forEach(j => { const b = to([A[0], A[1], A[2] - 0.01 * j], [J[0][j], J[1][j]], js); arrow(s, [A[0], A[1], A[2] - 0.01 * j], b, COL[j], 0.005); lab(s, b, 'J', { sub: String(j + 1), m: 1, c: KEY[j], s: 28, dx: j ? 16 : -16, dy: j ? 20 : -12, a: j ? 'l' : 'r' }); });
  if (hypot(vd[0], vd[1]) > 1e-9) { const V = to(A, vd, vs); arrow(s, A, V, C.gr, 0.0065); lab(s, V, 'v', { m: 1, c: 'gr', s: 32, dx: 16, dy: -8 }); }
  if (abs(det) < 0.012) lab(s, A, T('เอกฐาน: J₁ กับ J₂ อยู่แนวเดียวกัน', 'singular: J₁ and J₂ on one line'), { c: 'rd', s: 24, a: 'c', dy: 70 });
  s.info = { p, J, det, qd: CH5.two.rates(L, q, vd) }; return s; };

/* ---- static forces (Craig 5.9–5.10): the force F the tool applies (red) and the joint torques that hold it, τ = Jᵀ F.
   st: q, F (N), fs (m of arrow per N), lev (the lever arms: the distance from each joint to the line the force acts along) ---- */
SC.force = (st = {}, o = {}) => { const L = st.L || CH5.two.L, q = st.q || CH5.two.q, F = st.F || CH5.EX.force.F, fs = st.fs || 0.022, w = st.w || 1200, h = st.h || 1000, jt = jointsP(L, q), p = jt[2], tau = CH5.two.tau(L, q, F), A = lift(p), Fe = to(A, F, fs);
  const s = scene(st.name || 'force', w, h, planCam(st, [...jt, [Fe[0], Fe[1]]], w, h, [[0.3, 0.3, 0.05], 3.4]), o); floor(s, 1.3, [0.3, 0.3]); const arm = harm(s, L, q, { grip: false }), fm = hypot(F[0], F[1]);
  if (fm > 1e-9) { arrow(s, A, Fe, C.rd, 0.007); lab(s, Fe, 'F', { m: 1, c: 'rd', s: 32, dx: 16, dy: 12 });
    if (st.lev) { const u = [F[0] / fm, F[1] / fm, 0]; dashed(s, v(A, u, -0.55), v(A, u, 0.3), s.pal.dash, 0.0028, 25); jt.slice(0, 2).forEach((e, j) => { const O = lift(e), t = (A[0] - O[0]) * u[0] + (A[1] - O[1]) * u[1], foot = v(A, u, -t); if (hypot(foot[0] - O[0], foot[1] - O[1]) > 0.01) OV.dist(s, O, foot, KEY[j], 'r', { sub: String(j + 1), dy: j ? -18 : 26 }); }); } }
  tau.forEach((t, i) => { if (abs(t) < 1e-9) return; const e = jt[i]; turn(s, [e[0], e[1], Z0 + 0.085], [0, 0, sign(t)], 0.085 - 0.008 * i, COL[i], 0.6, 0.6 + min(5.0, 1.2 + 0.45 * abs(t)), 0.0045); lab(s, [e[0], e[1] - 0.085, Z0 + 0.085], 'τ', { sub: String(i + 1), m: 1, c: KEY[i], s: 28, a: 'c', dy: 36 }); });
  xyz(s, st, arm.pts[0], arm.pts[2]); s.info = { p, tau, J: CH5.two.J(L, q) }; return s; };
})(typeof window !== 'undefined' ? window : globalThis);
