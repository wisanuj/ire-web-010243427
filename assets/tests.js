/* tests.js — numeric tests of robot.js and ch1.js against hand calculations and the answers on the chapter 1 slides.
   Used by tests.html (browser) and by deno:  deno run --allow-read assets/run_tests.mjs  (no node on this Mac).
   RBTESTS(T): T(name, got, expected, tol) records one test; expected may be a string (exact match). */
(function (global) {
'use strict';
global.RBTESTS = T => {
  const RB = global.RB, CH1 = global.CH1, d2r = RB.rad, r2d = RB.deg, PI = Math.PI, FULL = { a1: -PI, b1: PI, a2: -PI, b2: PI };
  /* forward and inverse kinematics of the two-link arm */
  const f = RB.fk2(0.5, 0.4, d2r(30), d2r(45)); T('fk2 x (0.5, 0.4, 30°, 45°)', f.x, 0.5 * Math.cos(d2r(30)) + 0.4 * Math.cos(d2r(75)), 1e-12); T('fk2 y', f.y, 0.5 * Math.sin(d2r(30)) + 0.4 * Math.sin(d2r(75)), 1e-12);
  [1, -1].forEach(e => { const q = RB.ik2(0.5, 0.4, f.x, f.y, e), g = RB.fk2(0.5, 0.4, q.t1, q.t2); T(`ik2 → fk2 round trip x (elbow ${e})`, g.x, f.x, 1e-12); T(`ik2 → fk2 round trip y (elbow ${e})`, g.y, f.y, 1e-12); });
  T('ik2 elbow +1 gives θ2 = +45°', r2d(RB.ik2(0.5, 0.4, f.x, f.y, 1).t2), 45, 1e-9); T('ik2 elbow +1 gives θ1 = 30°', r2d(RB.ik2(0.5, 0.4, f.x, f.y, 1).t1), 30, 1e-9);
  T('ik2 outside the workspace has no solution', String(RB.ik2(0.5, 0.4, 1.0, 0, 1).ok), 'false');
  /* homework 1.3: a1 = 0.5, a2 = 0.4, (x, y) = (0.55, 0.5) */
  const h = RB.ik2(0.5, 0.4, 0.55, 0.5, 1), h2 = RB.ik2(0.5, 0.4, 0.55, 0.5, -1);
  T('HW 1.3 D', h.D, (0.55 ** 2 + 0.5 ** 2 - 0.25 - 0.16) / 0.4, 1e-12); T('HW 1.3 θ2 (+)', r2d(h.t2), r2d(Math.acos(0.35625)), 1e-9); T('HW 1.3 θ1 (+)', r2d(h.t1), 12.086, 0.001); T('HW 1.3 θ1 (−)', r2d(h2.t1), 72.461, 0.001);
  /* worked example 1: workspace ring of l1 = 0.4, l2 = 0.3 */
  const rg = RB.ring2(0.4, 0.3); T('WE1 r_max', rg.rmax, 0.7, 1e-12); T('WE1 r_min', rg.rmin, 0.1, 1e-12); T('WE1 area = 0.48π', rg.area, 1.508, 0.001);
  T('grid area of the free 2R workspace ≈ ring area', RB.ws2Area(0.4, 0.3, FULL, 300), rg.area, 0.01);
  T('joint limits shrink the area (−30…150°, 0…140°)', RB.ws2Area(0.4, 0.3, { a1: d2r(-30), b1: d2r(150), a2: 0, b2: d2r(140) }, 300) < 0.5 * rg.area ? 'yes' : 'no', 'yes');
  T('ws2Has: a point at r = 0.05 is not reachable', String(RB.ws2Has(0.4, 0.3, FULL, 0.05, 0)), 'false'); T('ws2Has: a point at r = 0.5 is reachable', String(RB.ws2Has(0.4, 0.3, FULL, 0.3, 0.4)), 'true');
  /* worked example 2 and the practice: reachable and dexterous rings of a 3R arm */
  const r3 = RB.ring3(0.4, 0.3, 0.1); T('WE2 reachable outer', r3.reach[1], 0.8, 1e-12); T('WE2 reachable inner', r3.reach[0], 0, 1e-12); T('WE2 dexterous inner', r3.dex[0], 0.2, 1e-12); T('WE2 dexterous outer', r3.dex[1], 0.6, 1e-12);
  const p3 = RB.ring3(0.5, 0.5, 0.2); T('practice reachable outer', p3.reach[1], 1.2, 1e-12); T('practice dexterous', p3.dex[0] + p3.dex[1], 1.0, 1e-12);
  const all = (fn, n = 360) => { for (let i = 0; i < n; i++) if (!fn(2 * PI * i / n)) return false; return true; }, some = (fn, n = 360) => { for (let i = 0; i < n; i++) if (fn(2 * PI * i / n)) return true; return false; };
  T('a point at r = 0.4 is dexterous (every direction)', String(all(RB.dexOk(0.4, 0.3, 0.1, 0.4, 0))), 'true'); T('a point at r = 0.7 is reachable but not dexterous', String(some(RB.dexOk(0.4, 0.3, 0.1, 0.7, 0)) && !all(RB.dexOk(0.4, 0.3, 0.1, 0.7, 0))), 'true');
  T('practice (c): r = 1.0 is not dexterous', String(all(RB.dexOk(0.5, 0.5, 0.2, 1.0, 0))), 'false');
  /* worked example 3 and homework 1.2: control resolution */
  T('WE3 chord d = √2 m', RB.chord(1, d2r(90)), Math.SQRT2, 1e-12); T('WE3 linear resolution (slide 1.3811e-3 m)', RB.chord(1, d2r(90)) / 1024, 1.3811e-3, 5e-8); T('WE3 rotational resolution (slide 1.534e-3 m)', RB.resolution(PI / 2, 10), 1.534e-3, 5e-7);
  T('HW 1.2 resolution (slide 6.136e-3 m)', RB.resolution(0.5 * PI, 8), 6.136e-3, 5e-7);
  /* Grübler */
  T('Grübler five-bar', RB.gruebler(5, 5), 2, 0); T('Grübler four-bar', RB.gruebler(4, 4), 1, 0);
  const F = RB.fivebar(CH1.FB.c, CH1.FB.La, CH1.FB.Lb, d2r(115), d2r(60)); T('five-bar closes: |AP| = Lb', Math.hypot(F.P[0] - F.A[0], F.P[1] - F.A[1]), CH1.FB.Lb, 1e-12); T('five-bar closes: |BP| = Lb', Math.hypot(F.P[0] - F.B[0], F.P[1] - F.B[1]), CH1.FB.Lb, 1e-12);
  /* point-to-point path of slide 27 */
  const S = { A: [0.2, 0.62], B: [0.8, 0.22] }, dv = n => CH1.pathDev({ ...S, n });
  T('PTP, 2 taught points: deviation (mm)', dv(2) * 1000, 107.52, 0.05); T('PTP deviation falls with more points', dv(2) > dv(3) && dv(3) > dv(5) && dv(5) > dv(9) ? 'yes' : 'no', 'yes'); T('PTP with 9 points < 3 mm', dv(9) * 1000 < 3 ? 'yes' : 'no', 'yes');
  T('CP stays on the seam (midpoint)', Math.hypot(...CH1.pathAt({ ...S, n: 2 }, 0.5, 'cp').p.map((v, i) => v - (S.A[i] + S.B[i]) / 2)), 0, 1e-9);
  /* the five arrangements */
  const tip = (ty, q) => RB.arrTip(ty, RB.arrQ(ty, q));
  T('Cartesian tip x = d1', tip('PPP', [0.6, 0.5, 0.45])[0], 0.6, 1e-12); T('Cartesian tip y = d2', tip('PPP', [0.6, 0.5, 0.45])[1], 0.5, 1e-12); T('Cartesian tip z = 0.85 − d3', tip('PPP', [0.6, 0.5, 0.45])[2], 0.4, 1e-12);
  T('cylindrical: radius = d3', Math.hypot(...tip('RPP', [35, 0.6, 0.7]).slice(0, 2)), 0.7, 1e-12); T('cylindrical: height = d2', tip('RPP', [35, 0.6, 0.7])[2], 0.6, 1e-12);
  const sp = tip('RRP', [30, 30, 0.65]); T('spherical: distance from the shoulder = d3', Math.hypot(sp[0], sp[1], sp[2] - 0.5), 0.65, 1e-12);
  T('SCARA: planar reach at θ2 = 0 is a1 + a2', Math.hypot(...tip('SCARA', [40, 0, 0.2]).slice(0, 2)), 0.8, 1e-12); T('SCARA: height = 0.6 − d3', tip('SCARA', [40, 60, 0.2])[2], 0.4, 1e-12);
  /* accuracy and repeatability: the statistics of a set of points */
  const st = CH1.shotStats([[1, 0], [1, 0.2], [1, -0.2], [1.2, 0], [0.8, 0]]); T('shots: accuracy = |mean|', st.acc, 1, 1e-12); T('shots: repeatability = max spread', st.rep, 0.2, 1e-12);
  /* the x, y, z written at the end of the arm in the chapter 1 simulations (needs ch1_3d.js: run_tests.mjs and tests.html load it) */
  if (CH1.S3) { const dd = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
    [['RRR', [40, 30, -50]], ['RRP', [40, 20, 0.6]], ['SCARA', [40, 60, 0.2]], ['RPP', [35, 0.6, 0.7]], ['PPP', [0.6, 0.5, 0.45]]].forEach(([ty, q]) => T(`tip readout: ${ty} = the arm's tip from the base`, dd(CH1.S3.arr({ type: ty, q, xyz: true }).tip, tip(ty, q)), 0, 1e-12));
    const f2 = RB.fk2(CH1.LOOP.l1, CH1.LOOP.l2, 0.4, 1.1); T('tip readout: two-link arm = fk2 (z = 0 in the arm plane)', dd(CH1.S3.jobs({ pts: [[0.3, 0.3]], q: [0.4, 1.1], xyz: true }).tip, [f2.x, f2.y, 0]), 0, 1e-12);
    T('tip readout: five-bar tool point', dd(CH1.S3.fivebar({ t1: d2r(115), t2: d2r(60), xyz: true }).tip, [F.P[0], F.P[1], 0]), 0, 1e-12);
    const kc = RB.chain(CH1.chainSpec(['R', 'P', 'R']), [0.5, 0.1, -0.4]), ec = kc.pts[kc.pts.length - 1]; T('tip readout: vertical chain = (x, 0, z)', dd(CH1.S3.chain({ types: ['R', 'P', 'R'], q: [0.5, 0.1, -0.4], xyz: true }).tip, [ec[0], 0, ec[1]]), 0, 1e-12); }
  if (global.CH2) ch2(T);
  if (global.CH3) ch3(T);
};
/* ---------------- chapter 3: forward kinematics (Craig ch. 3) ---------------- */
function ch3(T) {
  const RB = global.RB, CH3 = global.CH3, E = CH3.EX, d2r = RB.rad, H = Math.PI / 2;
  const near = (A, B) => { let e = 0; A.forEach((r, i) => r.forEach((x, j) => { e = Math.max(e, Math.abs(x - B[i][j])); })); return e; };
  /* the link transform is the product of its four elementary transforms (Craig 3.6), in both conventions */
  const al = d2r(-35), a = 0.3, th = d2r(50), d = 0.2;
  T('Craig 3.6: R_X(α) D_X(a) R_Z(θ) D_Z(d)', near(RB.dhT(al, a, th, d), RB.Tm(RB.T(RB.Rx(al)), RB.trans(a, 0, 0), RB.T(RB.Rz(th)), RB.trans(0, 0, d))), 0, 1e-12);
  T('classical: R_Z(θ) D_Z(d) D_X(a) R_X(α)', near(RB.dhTc(th, d, a, al), RB.Tm(RB.T(RB.Rz(th)), RB.trans(0, 0, d), RB.trans(a, 0, 0), RB.T(RB.Rx(al)))), 0, 1e-12);
  T('the four steps end on the link frame', near(CH3.steps4(al, a, th, d)[4], RB.dhT(al, a, th, d)), 0, 1e-12);
  T('row 2, column 4 of the link transform is −sin α · d', RB.dhT(al, a, th, d)[1][3], -Math.sin(al) * d, 1e-12);
  /* closed forms against the products of the link transforms */
  const P = E.p3r, Lp = P.L, F3 = RB.fkDH(P.rows(Lp), P.q);
  T('3R planar: ^0_3T = closed form (Craig 3.4)', near(F3[3], P.T03(Lp, P.q)), 0, 1e-12); T('3R planar with tool: ^0_4T = closed form (homework 3b)', near(RB.fkDH([...P.rows(Lp), P.tool(Lp)], P.q)[4], P.T04(Lp, P.q)), 0, 1e-12);
  T('3R planar at (30°, 45°, −60°): tip x', P.T03(Lp, P.q)[0][3], 0.5 * Math.cos(d2r(30)) + 0.4 * Math.cos(d2r(75)), 1e-12);
  const R = E.rpr; T('RPR: ^0_3T = closed form (Craig 3.4)', near(RB.fkDH(R.rows(R.L), R.q)[3], R.T03(R.L, R.q)), 0, 1e-12);
  T('RPR homework: ^0_4T = closed form', near(RB.fkDH(R.hwRows(R.L), R.q)[4], R.T04(R.L, R.q)), 0, 1e-12);
  T('RPR: the arm slides horizontally (z of the tip = L1 above the floor)', R.T04(R.L, R.q)[2][3], R.L.L1, 1e-12);
  const W = E.twist; T('90° twist: ^0_3T = closed form', near(RB.fkDH(W.rows(W.L), W.q)[3], W.T03(W.L, W.q)), 0, 1e-12);
  const A4 = E.arm4; T('four-frame arm: ^0_4T = closed form', near(RB.fkDH(A4.rows(A4.L), A4.q)[4], A4.T04(A4.L, A4.q)), 0, 1e-12);
  /* SCARA: Craig's table and the classical table describe the same robot */
  const S = E.scara, Fm = RB.fkDH(S.rows(S.L), S.q), Fc = RB.fkDH(S.rowsC(S.L), S.q, 'c'), tip = S.tip(S.L, S.q);
  T('SCARA modified: tip', Math.hypot(...RB.Tp(Fm[4]).map((x, i) => x - tip[i])), 0, 1e-12); T('SCARA classical: tip', Math.hypot(...RB.Tp(Fc[4]).map((x, i) => x - tip[i])), 0, 1e-12);
  T('SCARA: both tables give the same tool frame', near(Fm[4], Fc[4]), 0, 1e-12); T('SCARA: the tool points down', Fm[4][2][2], -1, 1e-12);
  /* ABB IRB 140: the DH frames agree with the joint axes of CoppeliaSim's model at its zero pose */
  const I = E.irb, F = CH3.frames(I.rows(), I.zero, null, RB.trans(0, 0, I.base)), p = k => RB.Tp(F[k]), z = k => RB.col(RB.TR(F[k]), 2);
  T('IRB 140: axis 2 passes through (0.07, ·, 0.352)', Math.hypot(p(2)[0] - 0.07, p(2)[2] - 0.352), 0, 1e-12); T('IRB 140: axis 2 is horizontal (along y)', Math.abs(z(2)[1]), 1, 1e-12);
  T('IRB 140: axis 3 at height 0.712', p(3)[2], 0.712, 1e-12); T('IRB 140: wrist centre at (0.45, 0, 0.712)', Math.hypot(p(4)[0] - 0.45, p(4)[1], p(4)[2] - 0.712), 0, 1e-12);
  T('IRB 140: axis 4 along x, axis 5 along y', Math.abs(z(4)[0]) + Math.abs(z(5)[1]), 2, 1e-12); T('IRB 140: axis 6 points down at the zero pose', z(6)[2], -1, 1e-12);
  /* the six-axis kinematic equations (Craig 3.7 method, IRB 140 numbers) against the product of the link transforms */
  const Fq = RB.fkDH(I.rows(), I.q), D = (i, j) => RB.Tm(...Fq.slice(1).slice(i, j).map((_, k) => RB.dhT(I.rows()[i + k].al, I.rows()[i + k].a, I.rows()[i + k].th + I.q[i + k], I.rows()[i + k].d)));
  T('six-axis: ^4_6T', near(D(4, 6), I.T46(I.q)), 0, 1e-12); T('six-axis: ^3_6T', near(D(3, 6), I.T36(I.q, I.d4)), 0, 1e-12); T('six-axis: ^1_3T', near(D(1, 3), I.T13(I.q, I.a1, I.a2)), 0, 1e-12);
  T('six-axis: ^0_6T, all twelve entries', near(Fq[6], I.T06(I.q, I.a1, I.a2, I.d4)), 0, 1e-12); T('six-axis: at the zero pose the wrist centre is 0.45 m out', I.T06(I.zero, I.a1, I.a2, I.d4)[0][3], 0.45, 1e-12);
  /* the x, y, z written at the end of the arm in the simulations is column 4 of ^0_NT (needs the scene files: run_tests.mjs and tests.html load them) */
  if (!CH3.S3) return;
  const col4 = M => [M[0][3], M[1][3], M[2][3]], dist = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]), tipOf = st => CH3.S3.dh({ ...st, xyz: true }).tip;
  const Rr = E.rpr, qs = [d2r(-40), d2r(70), d2r(25)];
  T('tip readout: planar 3R = ^0_3T', dist(tipOf({ ex: 'p3r', q: qs }), col4(P.T03(Lp, qs))), 0, 1e-12);
  T('tip readout: RPR = ^0_3T', dist(tipOf({ ex: 'rpr', q: [d2r(-50), 0.42, d2r(80)] }), col4(Rr.T03(Rr.L, [d2r(-50), 0.42, d2r(80)]))), 0, 1e-12);
  T('tip readout: 90° twist = ^0_3T', dist(tipOf({ ex: 'twist', q: qs }), col4(W.T03(W.L, qs))), 0, 1e-12);
  T('tip readout: four-frame arm = ^0_4T', dist(tipOf({ ex: 'arm4', q: qs }), col4(A4.T04(A4.L, qs))), 0, 1e-12);
  T('tip readout: IRB 140 = ^0_6T (wrist centre)', dist(tipOf({ ex: 'irb', q: I.q }), col4(I.T06(I.q, I.a1, I.a2, I.d4))), 0, 1e-12);
  T('tip readout: SCARA, both conventions', dist(tipOf({ ex: 'scara', q: S.q }), tip) + dist(tipOf({ ex: 'scara', q: S.q, conv: 'c' }), tip), 0, 1e-12);
  const rowsF = [{ al: 0, a: 0, d: 0, th: 0, j: 'R' }, { al: H, a: 0.2, d: 0.1, th: 0, j: 'R' }, { al: 0, a: 0.35, d: 0, th: 0, j: 'P' }], qf = [d2r(30), d2r(-20), 0.15], Ff = RB.fkDH(rowsF, qf);
  T('tip readout: the DH table explorer = ^0_NT', dist(CH3.S3.free({ rows: rowsF, q: qf, h0: 0.4, xyz: true }).tip, col4(Ff[Ff.length - 1])), 0, 1e-12);
}
/* ---------------- chapter 2: frames and transforms (answers of Craig ch. 2 and of the old Part I / Part II slides) ---------------- */
function ch2(T) {
  const RB = global.RB, CH2 = global.CH2, E = CH2.EX, d2r = RB.rad, r2d = RB.deg, s3 = Math.sqrt(3) / 2, M = (A, i, j) => A[i][j];
  const near = (A, B) => { let e = 0; A.forEach((r, i) => r.forEach((x, j) => { e = Math.max(e, Math.abs(x - B[i][j])); })); return e; };
  /* rotation matrices */
  T('Part I ex. 1: R = R_X(90°)', near(E.ex1.R, RB.Rx(d2r(90))), 0, 1e-12); T('Part I ex. 1: column 2 = Z_A, i.e. r32 = 1', M(E.ex1.R, 2, 1), 1, 1e-12);
  const R = RB.Rzyx(d2r(40), d2r(-20), d2r(25)), ck = RB.rotCheck(R); T('a rotation matrix: RᵀR = I', ck.orth, 0, 1e-12); T('a rotation matrix: det R = +1', ck.det, 1, 1e-12);
  T('inverse = transpose: R Rᵀ = I', near(RB.mm(R, RB.tr(R)), RB.I3()), 0, 1e-12); T('a reflection has det = −1', RB.rotCheck([[1, 0, 0], [0, 1, 0], [0, 0, -1]]).det, -1, 1e-12);
  T('columns of R_Z(θ): r11 = cos θ', M(RB.Rz(d2r(30)), 0, 0), s3, 1e-12); T('R_Y(θ): r13 = +sin θ', M(RB.Ry(d2r(30)), 0, 2), 0.5, 1e-12); T('R_X(θ): r23 = −sin θ', M(RB.Rx(d2r(30)), 1, 2), -0.5, 1e-12);
  /* mappings (Craig examples 2.1 and 2.2) */
  T('Craig 2.1: ᴬP_x = −1', E.ex21.ap[0], -1, 1e-12); T('Craig 2.1: ᴬP_y = 1.732', E.ex21.ap[1], Math.sqrt(3), 1e-12);
  T('Craig 2.2: ᴬP_x = 9.098', E.ex22.ap[0], 9.098, 0.0005); T('Craig 2.2: ᴬP_y = 12.562', E.ex22.ap[1], 12.562, 0.0005); T('Craig 2.2: ᴬP_z = 0', E.ex22.ap[2], 0, 1e-12);
  T('mapping in two steps (rotate, then add ᴬP_BORG) = one 4 × 4 product', Math.hypot(...RB.add(RB.mv(RB.Rz(d2r(30)), [3, 7, 0]), [10, 5, 0]).map((x, i) => x - E.ex22.ap[i])), 0, 1e-12);
  /* operators (Craig 2.4): the same numbers as 2.2, read as a moved point */
  T('Craig 2.4: ᴬP₂ = (9.098, 12.562, 0)', Math.hypot(E.ex24.p2[0] - 9.098, E.ex24.p2[1] - 12.562, E.ex24.p2[2]), 0, 0.0006);
  /* inverse and compound transforms (Craig 2.5) */
  T('Craig 2.5: ᴮ_A T p_x = −4.964', E.ex25.Ti[0][3], -4.964, 0.0005); T('Craig 2.5: ᴮ_A T p_y = −0.598', E.ex25.Ti[1][3], -0.598, 0.0005); T('Craig 2.5: rotation part = R_Z(−30°)', near(RB.TR(E.ex25.Ti), RB.Rz(d2r(-30))), 0, 1e-12);
  T('T T⁻¹ = I (4 × 4)', near(RB.Tm(E.ex25.T, E.ex25.Ti), RB.T(RB.I3())), 0, 1e-12);
  const TA = RB.T(RB.Rzyx(0.3, -0.5, 0.9), [0.2, -0.4, 0.7]), TB = RB.T(RB.Rzyx(-1.1, 0.2, 0.4), [1, 2, 3]);
  T('(AB)⁻¹ = B⁻¹ A⁻¹', near(RB.Ti(RB.Tm(TA, TB)), RB.Tm(RB.Ti(TB), RB.Ti(TA))), 0, 1e-12);
  /* homework 2.1: the three frames of the wedge */
  const H = E.hw21; T('HW 2.1: ⁰_1T is a rotation', RB.rotCheck(RB.TR(H.T01)).det, 1, 1e-12); T('HW 2.1: ⁰_2T is a rotation', RB.rotCheck(RB.TR(H.T02)).det, 1, 1e-12);
  T('HW 2.1: ⁰_2T = ⁰_1T ¹_2T', near(RB.Tm(H.T01, H.T12), H.T02), 0, 1e-12); T('HW 2.1: origin 2 seen from frame 1 is (1, 0, −1)', Math.hypot(H.T12[0][3] - 1, H.T12[1][3], H.T12[2][3] + 1), 0, 1e-12);
  T('HW 2.1: the distance from origin 1 to origin 2 is √2', Math.hypot(...RB.Tp(H.T12)), Math.SQRT2, 1e-12);
  /* transform equation: the arm reaching for the bolt */
  const B = E.bolt; T('bolt: ᴮ_T T ᵀ_G T = ᴮ_S T ˢ_G T', near(RB.Tm(B.BT, B.TG), RB.Tm(B.BS, B.SG)), 0, 1e-12); T('bolt: ᵀP_GORG = (0.1, 0.5, 0.3)', Math.hypot(B.TG[0][3] - 0.1, B.TG[1][3] - 0.5, B.TG[2][3] - 0.3), 0, 1e-12);
  T('bolt: the arm reaches the tool position', String(!!CH2.armTo(RB.Tp(B.BT))), 'true'); { const q = CH2.armTo(RB.Tp(B.BT)), w = RB.arrTip('RRR', q); T('bolt: the wrist of the arm is 0.12 m above the tool point', Math.hypot(w[0] - 0.5, w[1] - 0.2, w[2] - 0.72), 0, 1e-9); }
  /* X–Y–Z fixed angles and Z–Y–X Euler angles */
  const rt = RB.zyx(RB.Rzyx(d2r(60), d2r(30), d2r(-45))); T('angles back from R: α', r2d(rt.al), 60, 1e-9); T('angles back from R: β', r2d(rt.be), 30, 1e-9); T('angles back from R: γ', r2d(rt.ga), -45, 1e-9);
  const dg = RB.zyx(RB.Rzyx(d2r(20), d2r(90), d2r(50))); T('β = 90°: α is set to 0', dg.al, 0, 1e-12); T('β = 90°: γ = γ₀ − α₀ = 30°', r2d(dg.ga), 30, 1e-6); T('β = 90° is flagged', String(dg.deg), 'true');
  const dn = RB.zyx(RB.Rzyx(d2r(20), d2r(-90), d2r(50))); T('β = −90°: γ = γ₀ + α₀ = 70°', r2d(dn.ga), 70, 1e-6);
  const fx = CH2.seqR([['x', 60, 'fixed'], ['y', 40, 'fixed'], ['z', 50, 'fixed']]), eu = CH2.seqR([['z', 50, 'euler'], ['y', 40, 'euler'], ['x', 60, 'euler']]);
  T('X–Y–Z fixed (γ, β, α) = Z–Y–X Euler (α, β, γ)', near(fx, eu), 0, 1e-12); T('… = R_Z(α) R_Y(β) R_X(γ)', near(fx, RB.Rzyx(d2r(50), d2r(40), d2r(60))), 0, 1e-12);
  T('the same three turns in the other order give another matrix', near(fx, CH2.seqR([['x', 60, 'euler'], ['y', 40, 'euler'], ['z', 50, 'euler']])) > 0.1 ? 'yes' : 'no', 'yes');
  T('Part II slide 18: R = R_Y(45) R_X(30) R_X(30) R_Y(45) R_Z(60)', near(E.seq.R, [RB.Ry(d2r(45)), RB.Rx(d2r(60)), RB.Ry(d2r(45)), RB.Rz(d2r(60))].reduce(RB.mm)), 0, 1e-12);
  T('Part II slide 18: r11 = 0.655', E.seq.R[0][0], 0.6553, 0.0001);
  /* equivalent angle–axis and Euler parameters (Craig 2.6 and 2.7) */
  const K = E.ex26.R; T('Craig 2.6: r11 = 0.933', K[0][0], 0.933, 0.0005); T('Craig 2.6: r12 = 0.067', K[0][1], 0.067, 0.0005); T('Craig 2.6: r13 = 0.354', K[0][2], 0.354, 0.0005); T('Craig 2.6: r23 = −0.354', K[1][2], -0.354, 0.0005); T('Craig 2.6: r33 = 0.866', K[2][2], 0.866, 0.0005);
  T('Craig 2.7: p = (−1.13, 1.13, 0.05)', Math.hypot(E.ex27.T[0][3] + 1.1276, E.ex27.T[1][3] - 1.1276, E.ex27.T[2][3] - 0.0484), 0, 0.0005); T('Craig 2.7: the point (1, 2, 3) stays where it is', Math.hypot(...RB.Tv(E.ex27.T, [1, 2, 3]).map((x, i) => x - [1, 2, 3][i])), 0, 1e-12);
  const aa = RB.axisAngle(K); T('angle back from R: 30°', r2d(aa.th), 30, 1e-9); T('axis back from R: k_x = 0.707', aa.k[0], Math.SQRT1_2, 1e-9);
  const ep = RB.eulerP(E.ex26.k, d2r(30)); T('Euler parameters: ε1² + ε2² + ε3² + ε4² = 1', ep.reduce((a, x) => a + x * x, 0), 1, 1e-12); T('Euler parameters: ε4 = cos 15°', ep[3], Math.cos(d2r(15)), 1e-12);
  T('Euler parameters → R = R_K(θ)', near(RB.Reps(ep), K), 0, 1e-12); T('R → Euler parameters → R', near(RB.Reps(RB.eulerPR(E.seq.R)), E.seq.R), 0, 1e-12);
  /* homework 2.2 */
  const W = E.hw22; T('HW 2.2: ¹_2R = R_X(60°)', near(W.R12, RB.Rx(d2r(60))), 0, 1e-12); T('HW 2.2: ¹_3R = R_Y(−90°)', near(W.R13, RB.Ry(d2r(-90))), 0, 1e-12); T('HW 2.2: ¹_2R ²_3R = ¹_3R', near(RB.mm(W.R12, W.R23), W.R13), 0, 1e-12);
  T('HW 2.2: r21 of ²_3R = √3/2', W.R23[1][0], s3, 1e-12);
}
})(typeof window !== 'undefined' ? window : globalThis);
