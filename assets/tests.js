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
  if (global.CH4) ch4(T);
  if (global.CH5) ch5(T);
  if (global.CH6) ch6(T);
  if (global.CH7) ch7(T);
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
  /* Yaskawa GP8: the DH frames at the pose of the pictures (flange down), then the DH chain against the robot description's own chain */
  const I = E.gp8, F = CH3.frames(I.rows(), I.zero, null, RB.trans(0, 0, I.base)), p = k => RB.Tp(F[k]), z = k => RB.col(RB.TR(F[k]), 2);
  T('GP8: axis 2 (L) passes through (0.04, ·, 0.330)', Math.hypot(p(2)[0] - 0.04, p(2)[2] - 0.33), 0, 1e-12); T('GP8: axis 2 is horizontal (along y)', Math.abs(z(2)[1]), 1, 1e-12);
  T('GP8: axis 3 (U) at height 0.675', p(3)[2], 0.675, 1e-12); T('GP8: wrist centre at (0.38, 0, 0.715)', Math.hypot(p(4)[0] - 0.38, p(4)[1], p(4)[2] - 0.715), 0, 1e-12);
  T('GP8: axis 4 along x, axis 5 along y', Math.abs(z(4)[0]) + Math.abs(z(5)[1]), 2, 1e-12); T('GP8: axis 6 points down at the pose of the pictures', z(6)[2], -1, 1e-12);
  { let ep = 0, ea = 0, ex = 0, sd = 11; const rn = () => { sd = (sd * 16807) % 2147483647; return (sd - 1) / 2147483646; }, T0 = RB.trans(0, 0, I.base);
    for (let k = 0; k < 100; k++) { const y = [170, 105, 130, 190, 135, 360].map(m => d2r((2 * rn() - 1) * m)), L = I.links(y), G6 = RB.Tm(T0, RB.fkDH(I.rows(), I.fromY(y))[6]), dd = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
      ep = Math.max(ep, dd(RB.Tp(L[4]), RB.Tp(G6))); ea = Math.max(ea, dd(RB.col(RB.TR(L[6]), 0), RB.col(RB.TR(G6), 2))); ex = Math.max(ex, dd(RB.col(RB.TR(L[6]), 2), RB.col(RB.TR(G6), 0))); }
    T('GP8: DH wrist centre = the robot description\'s, θ = (S, L − 90°, −U, −R, −B, −T)', ep, 0, 1e-12); T('GP8: Z6 = the flange axis of the description', ea, 0, 1e-12); T('GP8: X6 = the description\'s link_6 z axis (no offset on θ6)', ex, 0, 1e-12); }
  /* the soft gripper drawn on the GP8 (scene3d.js): its tool point is 0.12 m × size from the mounting point along the tool direction */
  if (global.S3 && global.S3.softGrip) { const S3 = global.S3, sc = S3.scene('t', 100, 100, S3.camAt([0, 0, 0], 1, 0, 0)), g = S3.softGrip(sc, [0.1, 0.2, 0.3], [0, 0, -2], [1, 0, 0], { k: 0.8 });
    T('soft gripper: tool point 0.096 m below the mount (size 0.8)', Math.hypot(g.tip[0] - 0.1, g.tip[1] - 0.2, g.tip[2] - (0.3 - 0.096)), 0, 1e-12);
    const far = Math.max(...sc.items.map(o => Math.max(...[o.a, o.b].map(p => Math.hypot(p[0] - 0.1, p[1] - 0.2, p[2] - 0.3))))); T('soft gripper: every part within 0.115 m of the mount (size 0.8)', far < 0.115 ? 'yes' : 'no', 'yes');
    const sc2 = S3.scene('t', 100, 100, S3.camAt([0, 0, 0], 1, 0, 0)), g2 = S3.softGrip(sc2, [0, 0, 0], [1, 0, 0], [0, 1, 0], { tip: true }); T('soft gripper: given the tool point, the mount is 0.12 m back', Math.hypot(g2.mount[0] + 0.12, g2.mount[1], g2.mount[2]), 0, 1e-12);
    T('soft gripper: three fingers of seven folds each', sc.items.filter(o => o.t === 'cyl').length, 2 + 3 * 7, 0); }
  T('GP8: home (pendant all zero) puts the wrist centre at (0.380, 0, 0.385) in {0}', Math.hypot(I.T06(I.home)[0][3] - 0.38, I.T06(I.home)[2][3] - 0.385), 0, 1e-12);
  T('GP8: reach 0.040 + 0.345 + √(0.340² + 0.040²) = 0.727 m (data sheet)', I.a1 + I.a2 + Math.hypot(I.d4, I.a3), 0.727, 5e-4);
  /* the six-axis kinematic equations (Craig 3.7 method, GP8 numbers) against the product of the link transforms */
  const Fq = RB.fkDH(I.rows(), I.q), D = (i, j) => RB.Tm(...Fq.slice(1).slice(i, j).map((_, k) => RB.dhT(I.rows()[i + k].al, I.rows()[i + k].a, I.rows()[i + k].th + I.q[i + k], I.rows()[i + k].d)));
  T('six-axis: ^4_6T', near(D(4, 6), I.T46(I.q)), 0, 1e-12); T('six-axis: ^3_6T', near(D(3, 6), I.T36(I.q, I.d4)), 0, 1e-12); T('six-axis: ^1_3T', near(D(1, 3), I.T13(I.q, I.a1, I.a2)), 0, 1e-12);
  T('six-axis: ^0_6T, all twelve entries', near(Fq[6], I.T06(I.q, I.a1, I.a2, I.d4)), 0, 1e-12); T('six-axis: at the home pose the wrist centre is 0.38 m out', I.T06(I.home)[0][3], 0.38, 1e-12);
  /* the x, y, z written at the end of the arm in the simulations is column 4 of ^0_NT (needs the scene files: run_tests.mjs and tests.html load them) */
  if (!CH3.S3) return;
  const col4 = M => [M[0][3], M[1][3], M[2][3]], dist = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]), tipOf = st => CH3.S3.dh({ ...st, xyz: true }).tip;
  const Rr = E.rpr, qs = [d2r(-40), d2r(70), d2r(25)];
  T('tip readout: planar 3R = ^0_3T', dist(tipOf({ ex: 'p3r', q: qs }), col4(P.T03(Lp, qs))), 0, 1e-12);
  T('tip readout: RPR = ^0_3T', dist(tipOf({ ex: 'rpr', q: [d2r(-50), 0.42, d2r(80)] }), col4(Rr.T03(Rr.L, [d2r(-50), 0.42, d2r(80)]))), 0, 1e-12);
  T('tip readout: 90° twist = ^0_3T', dist(tipOf({ ex: 'twist', q: qs }), col4(W.T03(W.L, qs))), 0, 1e-12);
  T('tip readout: four-frame arm = ^0_4T', dist(tipOf({ ex: 'arm4', q: qs }), col4(A4.T04(A4.L, qs))), 0, 1e-12);
  T('tip readout: GP8 = ^0_6T (wrist centre)', dist(tipOf({ ex: 'gp8', q: I.q }), col4(I.T06(I.q, I.a1, I.a2, I.d4))), 0, 1e-12);
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
/* ---------------- chapter 4: inverse kinematics (Craig ch. 4) ---------------- */
function ch4(T) {
  const RB = global.RB, CH3 = global.CH3, CH4 = global.CH4, d2r = RB.rad, w = RB.wrap;
  const near = (A, B) => { let e = 0; A.forEach((r, i) => r.forEach((x, j) => { e = Math.max(e, Math.abs(x - B[i][j])); })); return e; };
  let seed = 7; const rnd = () => { seed = (seed * 16807) % 2147483647; return (seed - 1) / 2147483646; };
  /* planar 3R (Craig 4.4): both solutions, algebraic and geometric, put back through the forward kinematics */
  let e3 = 0, same = 0; for (let k = 0; k < 300; k++) { const q = [d2r(-180 + 360 * rnd()), d2r(-170 + 340 * rnd()), d2r(-180 + 360 * rnd())], [x, y, f] = CH4.p3rFK(0.5, 0.4, q), A = CH4.p3r(0.5, 0.4, x, y, f), G = CH4.p3rGeo(0.5, 0.4, x, y, f);
    [...A.sols, ...G.sols].forEach(s => { const g = CH4.p3rFK(0.5, 0.4, s); e3 = Math.max(e3, Math.abs(g[0] - x), Math.abs(g[1] - y), Math.abs(w(g[2] - f))); });
    if (A.sols.every(a => G.sols.some(g => a.every((v, i) => Math.abs(w(v - g[i])) < 1e-9)))) same++; }
  T('planar 3R: every algebraic and geometric solution reaches the goal', e3, 0, 1e-9); T('planar 3R: algebra and geometry give the same two solutions (300 goals)', same, 300, 0);
  T('planar 3R: out of reach (x = 1, y = 0) has no solution', CH4.p3r(0.5, 0.4, 1, 0, 0).sols.length, 0, 0);
  T('planar 3R: on the boundary (x = 0.9) the two solutions coincide', Math.abs(CH4.p3r(0.5, 0.4, 0.9, 0, 0).sols[0][1] - CH4.p3r(0.5, 0.4, 0.9, 0, 0).sols[1][1]), 0, 1e-6);
  /* tangent half-angle (Craig 4.5, example 4.3) */
  let eh = 0; for (let k = 0; k < 300; k++) { const a = 4 * rnd() - 2, b = 4 * rnd() - 2, c = 4 * rnd() - 2; CH4.half(a, b, c).sols.forEach(t => { eh = Math.max(eh, Math.abs(a * Math.cos(t) + b * Math.sin(t) - c)); }); }
  T('a cos θ + b sin θ = c: every root satisfies the equation', eh, 0, 1e-9); T('c² > a² + b²: no real root', CH4.half(1, 1, 2).sols.length, 0, 0);
  /* GP8: eight solutions, each one reaches the goal frame, the original among them */
  let ei = 0, found = 0, n8 = 0; for (let k = 0; k < 200; k++) { const q = [d2r(-170 + 340 * rnd()), d2r(-170 * rnd()), d2r(-85 + 170 * rnd()), d2r(-170 + 340 * rnd()), d2r(-110 + 220 * rnd()), d2r(-170 + 340 * rnd())], Tq = CH4.gp8FK(q), S = CH4.gp8IK(Tq);
    S.forEach(s => { ei = Math.max(ei, near(CH4.gp8FK(s.q), Tq)); }); if (S.some(s => s.q.every((v, i) => Math.abs(w(v - q[i])) < 1e-7))) found++; if (S.length === 8) n8++; }
  T('GP8: every inverse solution gives the goal ^0_6T (200 goals)', ei, 0, 1e-9); T('GP8: the joint angles we started from are always among the solutions', found, 200, 0); T('GP8: most goals have 8 solutions', n8 > 100 ? 'yes' : 'no', 'yes');
  const Z = CH4.gp8IK(CH4.gp8FK(CH3.EX.gp8.zero)); T('GP8 zero pose: 8 solutions', Z.length, 8, 0);
  T('GP8 zero pose: front, elbow up, no flip = the zero pose', Z.find(s => s.shoulder === 'front' && s.elbow === 'up' && s.wrist === 'no flip').q.every((v, i) => Math.abs(v - CH3.EX.gp8.zero[i]) < 1e-9) ? 'yes' : 'no', 'yes');
  T('GP8: a wrist flip is θ4 + 180°, −θ5, θ6 + 180°', (() => { const a = Z[0].q, b = Z[1].q; return Math.abs(w(b[3] - a[3] - Math.PI)) + Math.abs(b[4] + a[4]) + Math.abs(w(b[5] - a[5] - Math.PI)); })(), 0, 1e-9);
  /* closest solution and the RPR arm */
  T('closest solution: from the zero pose it is the zero pose', CH4.nearest(Z, CH3.EX.gp8.zero).d, 0, 1e-9);
  const R = CH3.EX.rpr, qr = [d2r(30), 0.35, d2r(45)], sr = CH4.rprIK(R.L.L2, R.T03(R.L, qr)).sols[0]; T('RPR inverse: θ1, d2, θ3 back', Math.abs(sr[0] - qr[0]) + Math.abs(sr[1] - qr[1]) + Math.abs(sr[2] - qr[2]), 0, 1e-12);
  /* the worked examples on the slides */
  const X = CH4.EX, dg = RB.deg, dd = (p, q) => Math.hypot(...p.map((x, i) => x - q[i]));
  T('ex 3R: goal x = 0.537', X.p3r.goal[0], 0.5365, 5e-5); T('ex 3R: goal y = 0.636', X.p3r.goal[1], 0.6364, 5e-5); T('ex 3R: goal φ = 15°', dg(X.p3r.goal[2]), 15, 1e-9);
  T('ex 3R: c2 = 0.7071 (θ2 = ±45°)', X.p3r.sol.c2, Math.SQRT1_2, 1e-9); T('ex 3R: second solution θ1 = 69.73°', dg(X.p3r.sol.sols[1][0]), 69.73, 0.005); T('ex 3R: second solution θ3 = −9.73°', dg(X.p3r.sol.sols[1][2]), -9.73, 0.005);
  T('ex half-angle: 3 cos θ + 4 sin θ = 2 roots 119.55°, −13.29°', Math.abs(dg(X.half.sol.sols[0]) - 119.55) + Math.abs(dg(X.half.sol.sols[1]) + 13.29), 0, 0.01);
  T('ex RRR: four solutions', X.arm4.sols.length, 4, 0); T('ex RRR: every solution reaches the goal', Math.max(...X.arm4.sols.map(s => dd(CH4.arm4P(X.arm4.L, s.q), X.arm4.p))), 0, 1e-9);
  T('ex RRR: elbow-down twin θ2 = −17.66°, θ3 = 60°', Math.abs(dg(X.arm4.sols[0].q[1]) + 17.66) + Math.abs(dg(X.arm4.sols[0].q[2]) - 60), 0, 0.01);
  T('ex SCARA: two solutions reach the goal', Math.max(...X.scara.sols.map(q => { const P = CH4.scaraPose(X.scara.L, q); return dd(P.p, X.scara.p) + Math.abs(RB.wrap(P.phi - X.scara.phi)); })), 0, 1e-9);
  T('ex SCARA: θ4 = θ1 + θ2 − φ', dg(X.scara.sols[1][3]), 30, 1e-9);
  T('ex GP8: 8 solutions, 4 inside the joint ranges', X.gp8.sols.length * 10 + X.gp8.sols.filter(s => s.ok).length, 84, 0);
  /* the numbers the slides and pages 4.1 / 4.4 print for the GP8 example (pose 30°, −60°, 20°, 40°, 50°, −30°) */
  { const G = CH3.EX.gp8, P = RB.Tp(X.gp8.T), u = Math.hypot(P[0], P[1]) - G.a1, K = (u * u + P[2] * P[2] - G.a2 ** 2 - G.a3 ** 2 - G.d4 ** 2) / (2 * G.a2), rt = Math.sqrt(G.a3 ** 2 + G.d4 ** 2 - K * K);
    const c3 = sg => Math.atan2(G.a3, G.d4) - Math.atan2(K, sg * rt), fd = X.gp8.sols.find(s => s.shoulder === 'front' && s.elbow === 'down' && s.wrist === 'no flip');
    T('ex GP8: u = 0.4217, K = −0.0787', Math.abs(u - 0.4217) + Math.abs(K + 0.0787), 0, 1e-4);
    T('ex GP8: Craig\'s θ3 = atan2(a3, d4) − atan2(K, ±√(a3² + d4² − K²)) gives 20° and −186.58°', Math.abs(RB.wrap(c3(1) - d2r(20))) + Math.abs(RB.wrap(c3(-1) - fd.q[2])), 0, 1e-9);
    T('ex GP8: the elbow-down solution is written inside the θ3 range (−186.58°), θ2 = 42.73°', Math.abs(dg(fd.q[2]) + 186.58) + Math.abs(dg(fd.q[1]) - 42.73), 0, 0.01);
    T('ex GP8: solutions 1, 2, 7, 8 are inside the joint ranges', X.gp8.sols.map((s, i) => s.ok ? i + 1 : '').join(''), '1278');
    T('ex GP8: the elbow-down pair in front fails on θ5 only', CH4.outOf(fd.q).join(','), '4');
    T('fitLim: an angle a full turn outside its range comes back inside (θ3 173.42° → −186.58°, θ4 200° → −160°), one that cannot stays (θ2 144.85°)', (() => { const q = CH4.fitLim([0, d2r(144.85), d2r(173.42), d2r(200), 0, 0]); return Math.abs(dg(q[1]) - 144.85) + Math.abs(dg(q[2]) + 186.58) + Math.abs(dg(q[3]) + 160); })(), 0, 1e-9); }
  T('ex SOLVE: 4 of the 8 solutions are inside the joint ranges', X.solve.sols.length * 10 + X.solve.sols.filter(s => s.ok).length, 84, 0);
  T('ex SOLVE: closest to the home pose is (18.43°, −71.54°, 6.51°, 0°, 65.03°, 18.43°)', Math.max(...[18.43, -71.54, 6.51, 0, 65.03, 18.43].map((v, i) => Math.abs(dg(X.solve.best.s.q[i]) - v))), 0, 0.006);
  T('ex SOLVE: wrist frame at (0.45, 0.15, 0.22)', dd(RB.Tp(X.solve.W), [0.45, 0.15, 0.22]), 0, 1e-12); T('ex SOLVE: closest solution θ1 = 18.43°', dg(X.solve.best.s.q[0]), 18.43, 0.005);
  T('ex SOLVE: tool frame of the chosen solution = {G}', near(RB.Tm(CH4.gp8FK(X.solve.best.s.q), RB.trans(0, 0, X.solve.LT)), X.solve.G), 0, 1e-9);
  T('HW 4: (x, y, z) = (0.5427, 0.3800, 0.6690)', dd(X.hw.p, [0.5427, 0.3800, 0.6690]), 0, 1e-4); T('HW 4: second solution (−145°, 170.04°, 0.9773)', Math.abs(dg(X.hw.sols[1][0]) + 145) + Math.abs(dg(X.hw.sols[1][1]) - 170.04) + Math.abs(X.hw.sols[1][2] - 0.9773), 0, 0.01);
  T('HW 4: both solutions reach the goal', Math.max(...X.hw.sols.map(q => dd(CH4.rrp.P(X.hw.L, q), X.hw.p))), 0, 1e-12); T('HW 4: closed form = product of the link transforms', dd(RB.Tp(CH4.rrp.T03(X.hw.L, X.hw.q)), X.hw.p), 0, 1e-12);
}
/* ---------------- chapter 5: the Jacobian ---------------- */
function ch5(T) { const RB = global.RB, CH3 = global.CH3, CH4 = global.CH4, CH5 = global.CH5, X = CH5.EX, d2r = RB.rad, dg = RB.deg;
  const near = (A, B) => { let e = 0; A.forEach((r, i) => r.forEach((x, j) => { e = Math.max(e, Math.abs(x - B[i][j])); })); return e; }, dd = (p, q) => Math.hypot(...p.map((x, i) => x - q[i]));
  let seed = 11; const rnd = () => { seed = (seed * 16807) % 2147483647; return (seed - 1) / 2147483646; }, ang = () => d2r(-170 + 340 * rnd());
  /* every closed form against the numerical derivative of the forward kinematics, at 100 random poses each */
  let e2 = 0, e23 = 0, erp = 0, e3 = 0, err = 0, est = 0, egp = 0, ej0v = 0, ej0w = 0, ed = 0;
  for (let k = 0; k < 100; k++) { const L = [0.2 + 0.6 * rnd(), 0.2 + 0.6 * rnd()], q = [ang(), ang()], J = CH5.two.J(L, q); e2 = Math.max(e2, near(J, CH5.numJ(x => CH5.two.P(L, x), q)));
    const a = q[0] + q[1], Rz = [[Math.cos(a), -Math.sin(a)], [Math.sin(a), Math.cos(a)]]; e23 = Math.max(e23, near(J, CH5.mm(Rz, CH5.two.J3(L, q)))); ed = Math.max(ed, Math.abs(CH5.det2(J) - CH5.two.det(L, q)));
    const qr = [ang(), 0.1 + rnd()]; erp = Math.max(erp, near(CH5.rp.J(qr), CH5.numJ(CH5.rp.P, qr)));
    const L3 = [0.3 + rnd(), 0.3 + rnd(), 0.1 + rnd()], q3 = [d2r(-80 + 160 * rnd()), d2r(-80 + 160 * rnd()), d2r(-80 + 160 * rnd())]; e3 = Math.max(e3, near(CH5.p3r.J(L3, q3), CH5.numJ(x => CH5.p3r.X(L3, x), q3)));
    const qq = [ang(), ang(), 0.1 + rnd()]; err = Math.max(err, near(CH5.rrp.J(CH5.rrp.L, qq), CH5.numJ(x => CH5.rrp.P(CH5.rrp.L, x), qq))); est = Math.max(est, near(CH5.stan.Jv(0.15, qq), CH5.numJ(x => CH5.stan.P(0.15, x), qq)));
    const g = [ang(), ang(), ang(), ang(), ang(), ang()], G = CH3.EX.gp8, J0 = CH5.J0(G.rows(), g), Jn = CH5.numJ(x => CH5.gp8.P(x), g); egp = Math.max(egp, near(CH5.gp8.Jv(g), Jn.map(r => r.slice(0, 3)))); ej0v = Math.max(ej0v, near(J0.slice(0, 3), Jn));
    /* the angular part: ω of joint j alone, read from Ṙ Rᵀ with a numerical Ṙ */
    const R = x => RB.TR(G.T06(x)), h = 1e-6; g.forEach((_, j) => { const a2 = g.slice(), b2 = g.slice(); a2[j] += h; b2[j] -= h; const Ra = R(a2), Rb = R(b2), Rd = Ra.map((r, i) => r.map((x, m) => (x - Rb[i][m]) / (2 * h))), w = CH5.omegaOf(R(g), Rd); ej0w = Math.max(ej0w, dd(w, [J0[3][j], J0[4][j], J0[5][j]])); }); }
  T('two-link: J = numerical derivative of (x, y) (100 poses)', e2, 0, 1e-8); T('two-link: ^0J = R_Z(θ1 + θ2) ^3J (Craig 5.66 and 5.67)', e23, 0, 1e-12); T('two-link: det J = l1 l2 s2', ed, 0, 1e-12);
  T('RP arm: J = numerical derivative', erp, 0, 1e-8); T('planar 3R with tool: 3 × 3 J of (x, y, φ) = numerical derivative', e3, 0, 1e-8); T('RRP arm of homework 4: J = numerical derivative', err, 0, 1e-8);
  T('Stanford arm: position Jacobian = numerical derivative', est, 0, 1e-8); T('Stanford arm: position = column 4 of its DH chain', dd(CH5.stan.P(0.15, CH5.stan.q), RB.Tp(RB.fkDH(CH5.stan.rows(0.15), CH5.stan.q).pop())), 0, 1e-12);
  T('GP8: closed-form wrist-centre Jacobian = numerical derivative (columns 1–3)', egp, 0, 1e-8); T('GP8: basic Jacobian, linear part = numerical derivative (6 columns)', ej0v, 0, 1e-8); T('GP8: basic Jacobian, angular part = ω from Ṙ Rᵀ', ej0w, 0, 1e-7);
  T('GP8: joints 4–6 do not move the wrist centre', Math.max(...X.gp8.J0.slice(0, 3).flatMap(r => r.slice(3).map(Math.abs))), 0, 1e-12);
  T('basic Jacobian of the planar 3R arm: ω_z row = (1, 1, 1)', (() => { const J = CH5.J0(CH3.EX.p3r.rows(CH3.EX.p3r.L), CH3.EX.p3r.q); return J[5].join(','); })(), '1,1,1');
  T('basic Jacobian of the homework RRP arm: linear part = its closed form', near(CH5.J0(CH4.rrp.rows(CH4.rrp.L), CH4.rrp.q).slice(0, 3), X.rrp.J), 0, 1e-12);
  T('sliding joint: no angular velocity (RRP column 3 of ω)', Math.hypot(...CH5.J0(CH4.rrp.rows(CH4.rrp.L), CH4.rrp.q).slice(3).map(r => r[2])), 0, 1e-12);
  /* representations */
  let ec = 0, es = 0, ee = 0; for (let k = 0; k < 100; k++) { const p = [0.2 + rnd(), 0.2 + rnd(), 0.2 + rnd()]; ec = Math.max(ec, near(CH5.rep.Ecyl(p), CH5.numJ(CH5.rep.cyl, p))); es = Math.max(es, near(CH5.rep.Esph(p), CH5.numJ(CH5.rep.sph, p)));
    const e = [d2r(-150 + 300 * rnd()), d2r(20 + 140 * rnd()), d2r(-150 + 300 * rnd())], ed3 = [rnd() - 0.5, rnd() - 0.5, rnd() - 0.5], h = 1e-6, Rt = t => CH5.rep.Rzxz(e[0] + ed3[0] * t, e[1] + ed3[1] * t, e[2] + ed3[2] * t), Ra = Rt(h), Rb = Rt(-h);
    const w = CH5.omegaOf(Rt(0), Ra.map((r, i) => r.map((x, m) => (x - Rb[i][m]) / (2 * h)))); ee = Math.max(ee, dd(CH5.mul(CH5.rep.Ezxz(e[0], e[1]), w), ed3)); const back = CH5.rep.zxz(Rt(0)); ee = Math.max(ee, dd(back, e)); }
  T('cylindrical coordinates: E_P = derivative of (ρ, θ, z) by (x, y, z)', ec, 0, 1e-7); T('spherical coordinates: E_P = derivative of (ρ, θ, φ)', es, 0, 1e-6); T('Z–X–Z Euler angles: E_R ω = (α̇, β̇, γ̇), and the angles are read back', ee, 0, 1e-6);
  /* the worked numbers */
  T('ex two-link: tool point (0.5365, 0.6364)', dd(X.two.p, [0.5365, 0.6364]), 0, 1e-4); T('ex two-link: J = [−0.6364 −0.3864; 0.5365 0.1035]', near(X.two.J, [[-0.6364, -0.3864], [0.5365, 0.1035]]), 0, 1e-4); T('ex two-link: det J = 0.1414', X.two.det, 0.1414, 1e-4);
  T('ex two-link: θ̇ = (0.5, −1) rad/s gives v = (0.0682, 0.1647) m/s', dd(X.two.v, [0.0682, 0.1647]), 0, 1e-4); T('ex two-link: the two column parts add up to v', dd(X.two.cols[0].map((x, i) => x + X.two.cols[1][i]), X.two.v), 0, 1e-12);
  T('ex small step (2°, −3°): predicted (−0.0020, 0.0133) m', dd(X.two.step.pred, [-0.0020, 0.0133]), 0, 1e-4); T('ex small step: actual (−0.0023, 0.0131) m', dd(X.two.step.act, [-0.0023, 0.0131]), 0, 1e-4);
  T('the prediction gets worse as the step grows (1°, 5°, 20°)', X.two.sizes[0].rel < X.two.sizes[1].rel && X.two.sizes[1].rel < X.two.sizes[2].rel ? 'yes' : 'no', 'yes'); T('1° step: prediction within 2 %', X.two.sizes[0].rel < 0.02 ? 'yes' : 'no', 'yes');
  T('ex RP: v = (−0.1232, 0.1866) m/s', dd(X.rp.v, [-0.1232, 0.1866]), 0, 1e-4); T('ex RP: det J = −d2', CH5.det2(X.rp.J), -0.5, 1e-12);
  T('HW 5.1: J row 1 = (−0.6752, −0.4252, −0.0388)', dd(X.p3r.J[0], [-0.6752, -0.4252, -0.0388]), 0, 1e-4); T('HW 5.1: Ẋ = (0.0117, 0.1239, 0.4)', dd(X.p3r.Xd, [0.0117, 0.1239, 0.4]), 0, 1e-4);
  T('HW 5.2: predicted step = J δq agrees with the real step to 0.3 mm', dd(X.rrp.pred, X.rrp.act) < 3e-4 ? 'yes' : 'no', 'yes'); T('HW 5.2: v = (−0.0492, 0.2082, −0.0514) m/s', dd(X.rrp.v, [-0.0492, 0.2082, -0.0514]), 0, 1e-4); T('HW 5.2: v = J q̇', dd(X.rrp.v, CH5.mul(X.rrp.J, X.rrp.qd)), 0, 1e-12);
  T('ex cylindrical: p = (0.3, 0.4, 0.25) is ρ = 0.5, θ = 53.13°', Math.abs(X.rep.cyl[0] - 0.5) + Math.abs(dg(X.rep.cyl[1]) - 53.13), 0, 0.005); T('ex cylindrical: (ρ̇, θ̇, ż) = (0.02, −0.22, 0.02)', dd(X.rep.rate, [0.02, -0.22, 0.02]), 0, 1e-9);
  T('ex train and car: ^C(^UV_TORG) = (0, 100, 0) km/h', dd(X.train.b, [0, 100, 0]), 0, 1e-9); T('ex train and car: ^C(^TV_CORG) = (0, −70, 0) km/h', dd(X.train.c, [0, -70, 0]), 0, 1e-9);
  /* using the Jacobian: Craig examples 5.4–5.6 */
  T('ex inverse: v = (0.1, 0) m/s needs θ̇ = (0.0732, −0.3794) rad/s', dd(X.inv.qd, [0.0732, -0.3794]), 0, 1e-4); T('ex inverse: J θ̇ gives v back', dd(CH5.mul(X.two.J, X.inv.qd), X.inv.v), 0, 1e-12);
  let e55 = 0; for (let k = 0; k < 50; k++) { const L = [0.3 + rnd(), 0.3 + rnd()], q = [ang(), d2r(10 + 160 * rnd())], r = CH5.two.rates(L, q, [1, 0]), c12 = Math.cos(q[0] + q[1]), s2 = Math.sin(q[1]); e55 = Math.max(e55, Math.abs(r[0] - c12 / (L[0] * s2)) + Math.abs(r[1] - (-Math.cos(q[0]) / (L[1] * s2) - c12 / (L[0] * s2)))); }
  T('Craig example 5.5: θ̇1 = c12 / (l1 s2), θ̇2 = −c1 / (l2 s2) − c12 / (l1 s2) for 1 m/s along X', e55, 0, 1e-9);
  T('singular when the arm is straight: det J = 0 at θ2 = 0 and 180°', Math.abs(CH5.two.det([0.5, 0.4], [1, 0])) + Math.abs(CH5.two.det([0.5, 0.4], [1, Math.PI])), 0, 1e-12); T('at a singularity there are no joint rates', CH5.two.rates([0.5, 0.4], [1, 0], [0.1, 0]) === null ? 'yes' : 'no', 'yes');
  T('joint rates grow towards the singularity (x = 0.6 … 0.898)', X.near.every((r, i) => !i || Math.hypot(...r.qd) > Math.hypot(...X.near[i - 1].qd)) ? 'yes' : 'no', 'yes'); T('ex near the singularity: the tool point is on the X axis', Math.max(...X.near.map(r => dd(CH5.two.P(X.two.L, r.q), [r.x, 0]))), 0, 1e-12);
  T('ex static force: F = (0, −10) N needs τ = (−5.365, −1.035) N·m', dd(X.force.tau, [-5.365, -1.035]), 0, 1e-3);
  T('static force: τ1 is the moment of F about joint 1, τ2 about joint 2', (() => { const L = X.two.L, q = X.two.q, p = X.two.p, e = [L[0] * Math.cos(q[0]), L[0] * Math.sin(q[0])], F = X.force.F; return Math.abs(X.force.tau[0] - (p[0] * F[1] - p[1] * F[0])) + Math.abs(X.force.tau[1] - ((p[0] - e[0]) * F[1] - (p[1] - e[1]) * F[0])); })(), 0, 1e-12);
  T('Craig example 5.6: τ1 = l1 s2 fx + (l2 + l1 c2) fy, τ2 = l2 fy (force written in {3})', (() => { const L = [0.5, 0.4], q = [0.3, 0.9], f = [3, -2], t = CH5.mul(CH5.tr(CH5.two.J3(L, q)), f); return Math.abs(t[0] - (L[0] * Math.sin(q[1]) * f[0] + (L[1] + L[0] * Math.cos(q[1])) * f[1])) + Math.abs(t[1] - L[1] * f[1]); })(), 0, 1e-12);
  T('power is the same on both sides: F · v = τ · θ̇', (() => { const v = X.two.v, t = CH5.two.tau(X.two.L, X.two.q, [3, -2]); return Math.abs(3 * v[0] - 2 * v[1] - (t[0] * X.two.qd[0] + t[1] * X.two.qd[1])); })(), 0, 1e-12);
}
/* ---------------- chapter 6: velocity propagation and the explicit form ---------------- */
function ch6(T) { const RB = global.RB, CH3 = global.CH3, CH4 = global.CH4, CH5 = global.CH5, CH6 = global.CH6, X = CH6.EX, d2r = RB.rad;
  const near = (A, B) => { let e = 0; A.forEach((r, i) => r.forEach((x, j) => { e = Math.max(e, Math.abs(x - B[i][j])); })); return e; }, dd = (p, q) => Math.hypot(...p.map((x, i) => x - q[i]));
  let seed = 23; const rnd = () => { seed = (seed * 16807) % 2147483647; return (seed - 1) / 2147483646; }, ang = () => d2r(-170 + 340 * rnd()), rate = () => 2 * rnd() - 1;
  /* rigid body: the cross product as a matrix, and Craig 5.13 against a numerical derivative of the point's position */
  let es = 0, er = 0; for (let k = 0; k < 100; k++) { const w = [rate(), rate(), rate()], p = [rate(), rate(), rate()]; es = Math.max(es, dd(RB.mv(CH6.skew(w), p), RB.cross(w, p)));
    const ax = RB.unit([rate(), rate(), rate()]), om = 0.5 + rnd(), th0 = ang(), vB = [rate(), rate(), rate()], vQ = [rate(), rate(), rate()], Q = [rate(), rate(), rate()], O = [rate(), rate(), rate()];
    const pos = t => RB.add(RB.add(O, RB.sc(vB, t)), RB.mv(RB.Rk(ax, th0 + om * t), RB.add(Q, RB.sc(vQ, t)))), h = 1e-6, num = pos(h).map((x, i) => (x - pos(-h)[i]) / (2 * h)); er = Math.max(er, dd(CH6.vQ(RB.Rk(ax, th0), vB, vQ, RB.sc(ax, om), Q).v, num)); }
  T('skew matrix: S(Ω) p = Ω × p', es, 0, 1e-12); T('rigid body (Craig 5.13): translation + relative + Ω × Q = numerical derivative of the point (100 motions)', er, 0, 1e-7);
  T('ex rigid body: the three parts add up to ^AV_Q', dd(RB.add(RB.add(X.rigid.trans, X.rigid.rel), X.rigid.rot), X.rigid.v), 0, 1e-12);
  /* propagation = basic Jacobian × rates, for every arm of the course, in {0} and in the last frame */
  const arms = [['two-link', L => CH6.two.rows([0.3 + rnd(), 0.3 + rnd()]), () => [ang(), ang()]], ['planar 3R with tool', () => [...CH3.EX.p3r.rows(CH3.EX.p3r.L), CH3.EX.p3r.tool(CH3.EX.p3r.L)], () => [ang(), ang(), ang()]],
    ['RRP', () => CH4.rrp.rows(CH4.rrp.L), () => [ang(), ang(), 0.1 + rnd()]], ['RPR', () => CH3.EX.rpr.rows(CH3.EX.rpr.L), () => [ang(), 0.1 + rnd(), ang()]], ['SCARA', () => CH3.EX.scara.rows(CH3.EX.scara.L), () => [ang(), ang(), 0.3 * rnd(), ang()]],
    ['Stanford (6 joints)', () => CH6.stan.rows(0.15), () => [ang(), ang(), 0.2 + rnd(), ang(), ang(), ang()]], ['GP8', () => CH3.EX.gp8.rows(), () => [ang(), ang(), ang(), ang(), ang(), ang()]]];
  arms.forEach(([name, mk, mq]) => { let e0 = 0, en = 0, ej = 0; for (let k = 0; k < 40; k++) { const rows = mk(), q = mq(), qd = q.map(rate), P = CH6.prop(rows, q, qd), n = P.length - 1, J0 = CH5.J0(rows, q), vw = CH5.mul(J0, qd);
      e0 = Math.max(e0, dd(P[n].v0, vw.slice(0, 3)), dd(P[n].w0, vw.slice(3))); const Jn = CH6.Jn(rows, q), vn = CH5.mul(Jn, qd); en = Math.max(en, dd(P[n].v, vn.slice(0, 3)), dd(P[n].w, vn.slice(3)));
      const C = CH6.cols(rows, q); ej = Math.max(ej, near(J0, [0, 1, 2, 3, 4, 5].map(m => C.map(c => m < 3 ? c.lin[m] : c.ang[m - 3])))); }
    T(name + ': propagated v and ω = J0 q̇ (written in {0})', e0, 0, 1e-10); T(name + ': propagated ^nv_n and ^nω_n = ^nJ q̇ (written in the last frame)', en, 0, 1e-10); T(name + ': explicit-form columns = the basic Jacobian', ej, 0, 1e-12); });
  /* closed forms */
  let e3 = 0, er3 = 0, eh = 0, ez = 0; for (let k = 0; k < 60; k++) { const L = [0.3 + rnd(), 0.3 + rnd()], q = [ang(), ang()], qd = [rate(), rate()], P = CH6.prop(CH6.two.rows(L), q, qd)[3];
    e3 = Math.max(e3, dd(P.v, CH6.two.v3(L, q, qd)), dd(P.w, CH6.two.w3(qd)), dd(P.v0, CH6.two.v0(L, q, qd)));
    const rq = [ang(), ang(), 0.1 + rnd()], rqd = [rate(), rate(), rate()], PR = CH6.prop(CH4.rrp.rows(CH4.rrp.L), rq, rqd)[3]; er3 = Math.max(er3, dd(PR.v, CH6.rrp.v3(CH4.rrp.L, rq, rqd)), dd(PR.w, CH6.rrp.w3(rq, rqd)), near(CH6.Jn(CH4.rrp.rows(CH4.rrp.L), rq).slice(0, 3), CH6.rrp.J3(CH4.rrp.L, rq)));
    const hq = [ang(), 0.1 + rnd(), ang()], hqd = [rate(), rate(), rate()], LH = CH3.EX.rpr.L, PH = CH6.prop(CH3.EX.rpr.rows(LH), hq, hqd)[3]; eh = Math.max(eh, dd(PH.v, CH6.rpr.v3(LH, hq, hqd)), dd(PH.w, CH6.rpr.w3(hq, hqd)), dd(PH.v0, CH6.rpr.v0(LH, hq, hqd)), dd(PH.w0, CH6.rpr.w0(hq, hqd)));
    const sq = [ang(), ang(), 0.2 + rnd(), ang(), ang(), ang()], Zs = CH6.stan.Z(sq), C = CH6.cols(CH6.stan.rows(0.15), sq); ez = Math.max(ez, ...C.map((c, i) => dd(c.z, Zs[i]))); }
  T('Craig example 5.3: ^3v_3, ^3ω_3 and ^0v_3 of the two-link arm (closed forms)', e3, 0, 1e-12); T('class exercise RRP: ^3v_3 = (−d3θ̇2, (L2 + d3c2)θ̇1, ḋ3), ^3ω_3, and ^3J', er3, 0, 1e-12);
  T('HW 6 (RPR): ^3v_3, ^3ω_3, ^0v_3, ^0ω_3 in closed form', eh, 0, 1e-12); T('Stanford arm: the six joint axes Z_1 … Z_6 of the old slides', ez, 0, 1e-12);
  T('two-link: det ^3J = det ^0J = l1 l2 s2 (a frame change does not change the determinant)', Math.abs(CH5.det2(X.two.J3v) - CH5.det2(X.two.J0v)) + Math.abs(CH5.det2(X.two.J3v) - CH5.EX.two.det), 0, 1e-12);
  T('two-link: ^3J = [l1 s2, 0; l1 c2 + l2, l2] (Craig 5.66)', near(X.two.J3v, CH5.two.J3(X.two.L, X.two.q)), 0, 1e-12);
  T('Jacobian in a frame and back: ^0J → ^3J → ^0J', near(CH6.Jin(RB.TR(RB.fkDH(X.two.rows, X.two.q).pop()), X.two.J3), X.two.J0), 0, 1e-12);
  /* the worked numbers */
  T('ex two-link: ^3v_3 = (0.1768, −0.0232, 0) m/s', dd(X.two.v3, [0.1768, -0.0232, 0]), 0, 1e-4); T('ex two-link: ^3ω_3 = (0, 0, −0.5) rad/s', dd(X.two.w3, [0, 0, -0.5]), 0, 1e-12); T('ex two-link: ^0v_3 = the v of chapter 5', dd(X.two.v0.slice(0, 2), CH5.EX.two.v), 0, 1e-12);
  T('ex RRP: ^0v_3 = the v of homework 5.2', dd(X.rrp.v0, CH5.EX.rrp.v), 0, 1e-12); T('ex GP8: propagated v, ω = chapter 5', dd(X.gp8.prop[6].v0, CH5.EX.gp8.v6) + dd(X.gp8.prop[6].w0, CH5.EX.gp8.w6), 0, 1e-12);
  T('ex Stanford: joints 4–6 add no linear velocity to the wrist point', Math.max(...X.stan.cols.slice(3).map(c => Math.hypot(...c.lin))), 0, 1e-12); T('ex Stanford: the sliding joint 3 adds no angular velocity', Math.hypot(...X.stan.cols[2].ang), 0, 1e-12);
  T('ex Stanford: linear part of J0 = chapter 5\'s position Jacobian', near(X.stan.J0.slice(0, 3).map(r => r.slice(0, 3)), CH5.stan.Jv(X.stan.L2, X.stan.q)), 0, 1e-12);
}
/* ---------------- chapter 7: trajectory generation ---------------- */
function ch7(T) { const RB = global.RB, CH5 = global.CH5, CH7 = global.CH7, X = CH7.EX, d2r = RB.rad, dg = RB.deg;
  let seed = 31; const rnd = () => { seed = (seed * 16807) % 2147483647; return (seed - 1) / 2147483646; }, R = (a, b) => a + (b - a) * rnd();
  /* the polynomials meet their boundary conditions */
  let ec = 0, eq = 0; for (let k = 0; k < 200; k++) { const th0 = R(-90, 90), thf = R(-90, 90), tf = R(0.5, 5), v0 = R(-30, 30), vf = R(-30, 30), q0 = R(-50, 50), qf = R(-50, 50);
    const c = CH7.cubic(th0, thf, tf, v0, vf), s0 = CH7.poly(c, 0), s1 = CH7.poly(c, tf); ec = Math.max(ec, Math.abs(s0[0] - th0), Math.abs(s1[0] - thf), Math.abs(s0[1] - v0), Math.abs(s1[1] - vf));
    const q = CH7.quintic(th0, thf, tf, v0, vf, q0, qf), u0 = CH7.poly(q, 0), u1 = CH7.poly(q, tf); eq = Math.max(eq, Math.abs(u0[0] - th0), Math.abs(u1[0] - thf), Math.abs(u0[1] - v0), Math.abs(u1[1] - vf), Math.abs(u0[2] - q0), Math.abs(u1[2] - qf)); }
  T('cubic: θ and θ̇ at both ends as asked (200 random cases)', ec, 0, 1e-8); T('quintic: θ, θ̇, θ̈ at both ends as asked (200 random cases)', eq, 0, 1e-7);
  T('poly: derivatives = numerical derivatives', (() => { const a = [1, -2, 0.5, 0.3, -0.07, 0.01], t = 1.3, h = 1e-5, f = x => CH7.poly(a, x); return Math.abs(f(t)[1] - (f(t + h)[0] - f(t - h)[0]) / (2 * h)) + Math.abs(f(t)[2] - (f(t + h)[1] - f(t - h)[1]) / (2 * h)); })(), 0, 1e-6);
  /* via points */
  /* the largest jump of θ, θ̇, θ̈ across any instant (sampled 2·10⁻⁷ s apart: a smooth function changes by far less than 0.01, a real jump by much more) */
  const contin = (tr, n = 6000) => { let jp = 0, jv = 0, ja = 0; for (let i = 1; i < n; i++) { const t = tr.T * i / n, a = tr.at(t - 1e-7), b = tr.at(t + 1e-7); jp = Math.max(jp, Math.abs(a[0] - b[0])); jv = Math.max(jv, Math.abs(a[1] - b[1])); ja = Math.max(ja, Math.abs(a[2] - b[2])); } return [jp, jv, ja]; };
  let ev = 0, es = 0; for (let k = 0; k < 60; k++) { const n = 3 + Math.floor(rnd() * 4), pts = Array.from({ length: n }, () => R(-60, 60)), durs = Array.from({ length: n - 1 }, () => R(0.5, 3));
    const ch = CH7.cubics(pts, durs, CH7.viaHeuristic(pts, durs)), cs = CH7.cubics(pts, durs, CH7.viaSpline(pts, durs)), j1 = contin(ch, 3000), j2 = contin(cs, 3000);
    ev = Math.max(ev, j1[0], j1[1], ...ch.ts.map((t, i) => Math.abs(ch.at(t)[0] - pts[i]))); es = Math.max(es, j2[0], j2[1], j2[2], ...cs.ts.map((t, i) => Math.abs(cs.at(t)[0] - pts[i]))); }
  T('cubics through via points: pass every path point, θ and θ̇ continuous (heuristic velocities)', ev, 0, 1e-2); T('cubic spline: θ, θ̇ and θ̈ continuous at the via points', es, 0, 1e-2);
  T('heuristic: zero where the slope changes sign, the average where it does not', X.via.heur.map(v => +v.toFixed(4)).join(','), '0,0,-7.5,0');
  /* parabolic blends */
  let el = 0, elv = 0; for (let k = 0; k < 100; k++) { const th0 = R(-90, 90), thf = R(-90, 90), tf = R(0.5, 4), acc = CH7.lspbMin(th0, thf, tf) * R(1, 4), L = CH7.lspb(th0, thf, tf, acc), j = contin(L, 2000);
    el = Math.max(el, j[0], j[1], Math.abs(L.at(tf / 2)[0] - (th0 + thf) / 2), Math.abs(L.at(tf)[0] - thf)); if (!L.ok) elv = 1; }
  T('one segment with blends: θ and θ̇ continuous, the middle point halfway, ends reached (100 cases)', el, 0, 1e-2); T('one segment: every acceleration above the minimum works', elv, 0, 0);
  T('below the minimum acceleration there is no solution', CH7.lspb(15, 75, 3, 20).ok ? 'yes' : 'no', 'no'); T('at the minimum the straight part has shrunk to nothing', CH7.lspb(15, 75, 3, CH7.lspbMin(15, 75, 3)).tl, 0, 1e-9);
  let eb = 0, ebt = 0; for (let k = 0; k < 60; k++) { const n = 3 + Math.floor(rnd() * 4), pts = Array.from({ length: n }, () => R(-60, 60)), durs = Array.from({ length: n - 1 }, () => R(1.5, 3)), lv = CH7.lspbVia(pts, durs, 400), j = contin(lv, 4000);
    if (!lv.ok) continue; eb = Math.max(eb, j[0], j[1], Math.abs(lv.at(lv.T)[0] - pts[n - 1]), Math.abs(lv.at(0)[0] - pts[0]));
    ebt = Math.max(ebt, Math.abs(lv.tb[0] + lv.tl[0] + lv.tb[1] / 2 - durs[0]), Math.abs(lv.tb[n - 1] + lv.tl[n - 2] + lv.tb[n - 2] / 2 - durs[n - 2])); }
  T('blends through via points: θ and θ̇ continuous, the ends reached (random paths)', eb, 0, 1e-2); T('blends through via points: blend and straight times add up to each segment', ebt, 0, 1e-9);
  /* the worked numbers */
  T('ex 7.1 cubic: a = (15, 0, 20, −4.44)', Math.abs(X.cubic.a[2] - 20) + Math.abs(X.cubic.a[3] + 40 / 9), 0, 1e-12); T('ex 7.1: top speed 30 °/s at t = 1.5 s, θ̈ = ±40 °/s² at the ends', Math.abs(X.cubic.vmax - 30) + Math.abs(X.cubic.acc0 - 40) + Math.abs(X.cubic.accf + 40), 0, 1e-9);
  T('quintic 15° → 75° in 3 s: a3, a4, a5 = 22.22, −11.11, 1.481', Math.abs(X.quintic.a[3] - 200 / 9) + Math.abs(X.quintic.a[4] + 100 / 9) + Math.abs(X.quintic.a[5] - 40 / 27), 0, 1e-9); T('quintic: top speed 37.5 °/s', X.quintic.vmax, 37.5, 1e-9);
  T('cubic leaving at 10 °/s: a = (15, 10, 13.33, −3.33)', Math.abs(X.cubicV.a[1] - 10) + Math.abs(X.cubicV.a[2] - 40 / 3) + Math.abs(X.cubicV.a[3] + 10 / 3), 0, 1e-9);
  T('spline via velocities (0, 0.714, −13.393, 0) °/s', Math.abs(X.via.spline[1] - 5 / 7) + Math.abs(X.via.spline[2] + 375 / 28), 0, 1e-9);
  T('blends at 40 °/s²: tb = 0.634 s, straight at 25.36 °/s', Math.abs(X.lspb.a40.tb - 0.6340) + Math.abs(X.lspb.a40.v - 25.359), 0, 2e-4); T('blends: minimum acceleration 26.67 °/s²', X.lspb.amin, 80 / 3, 1e-9);
  const lv = X.lspbVia; T('via example: blends 0.268, 0.468, 0.098, 0.102 s', Math.abs(lv.tb[0] - 0.2679) + Math.abs(lv.tb[1] - 0.4679) + Math.abs(lv.tb[2] - 0.0983) + Math.abs(lv.tb[3] - 0.1017), 0, 4e-4);
  T('via example: velocities 13.40, −10, −5.09 °/s', Math.abs(lv.v[0] - 13.3975) + Math.abs(lv.v[1] + 10) + Math.abs(lv.v[2] + 5.0862), 0, 4e-4); T('via example: straight times 1.498, 0.717, 2.849 s', Math.abs(lv.tl[0] - 1.4981) + Math.abs(lv.tl[1] - 0.7169) + Math.abs(lv.tl[2] - 2.8491), 0, 4e-4);
  T('via example: the arm passes near 35°, not through it (33.63° at t = 2 s)', lv.at(2)[0], 33.631, 1e-3);
  T('HW 7: v12 = 5.109, v23 = −13.229 °/s', Math.abs(X.hw.v[0] - 5.1087) + Math.abs(X.hw.v[1] + 13.2292), 0, 4e-4); T('HW 7: t1, t2, t3 = 0.085, 0.306, 0.221 s', Math.abs(X.hw.tb[0] - 0.0851) + Math.abs(X.hw.tb[1] - 0.3056) + Math.abs(X.hw.tb[2] - 0.2205), 0, 4e-4);
  /* Cartesian paths */
  const two = X.two; let el2 = 0; two.line.forEach(s => { if (!s.ok) { el2 = 1; return; } const f = RB.fk2(two.L[0], two.L[1], s.q[0], s.q[1]), u = [two.B[0] - two.A[0], two.B[1] - two.A[1]]; el2 = Math.max(el2, Math.abs(u[0] * (f.y - two.A[1]) - u[1] * (f.x - two.A[0])) / Math.hypot(...u)); });
  T('straight line: inverse kinematics keeps the tool on the line', el2, 0, 1e-12); T('straight line: starts at the pose of A and ends at the pose of B', Math.hypot(...two.line[0].q.map((x, i) => x - two.qA[i])) + Math.hypot(...two.line[two.line.length - 1].q.map((x, i) => x - two.qB[i])), 0, 1e-9);
  T('joint-space move: the tool leaves the straight line by more than 0.3 m', (() => { let m = 0; const u = [two.B[0] - two.A[0], two.B[1] - two.A[1]]; two.joint.forEach(s => { m = Math.max(m, Math.abs(u[0] * (s.p[1] - two.A[1]) - u[1] * (s.p[0] - two.A[0])) / Math.hypot(...u)); }); return m > 0.3 ? 'yes' : 'no'; })(), 'yes');
  T('the line through the hole has points out of reach', X.hole.line.some(s => !s.ok) ? 'yes' : 'no', 'yes'); T('the line 0.26 m from the base (just outside the hole) is reachable everywhere', X.near.line.every(s => s.ok) ? 'yes' : 'no', 'yes');
  T('near the singularity joint 1 must turn more than twice as fast as on the far line', CH7.peakRates(X.near.line)[0] > 2 * CH7.peakRates(X.far.line)[0] ? 'yes' : 'no', 'yes');
  T('joint rates along the line = J⁻¹ v (chapter 5)', (() => { const s = two.line[25], J = CH5.two.J(two.L, s.q), v = CH5.mul(J, s.qd), sd = CH7.sOf(s.t, two.T)[1]; return Math.hypot(v[0] - sd * (two.B[0] - two.A[0]), v[1] - sd * (two.B[1] - two.A[1])); })(), 0, 1e-12);
}
})(typeof window !== 'undefined' ? window : globalThis);
