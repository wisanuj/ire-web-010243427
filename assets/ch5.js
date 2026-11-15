/* ch5.js — chapter 5, the manipulator Jacobian (Craig, Introduction to Robotics, 3rd ed., chapter 5: 5.1–5.2 and 5.7–5.10; the old slides
   "Chapter 5 Jacobians: Velocities", parts 1 and 2, which follow Khatib's lecture notes).
   Every Jacobian written here in closed form is checked in tests.js against a numerical derivative of the forward kinematics (CH5.numJ).
   Works in the browser (window.CH5) and in deno (globalThis.CH5). Needs robot.js, ch3.js and ch4.js. */
(function (global) {
'use strict';
const RB = global.RB, CH3 = global.CH3, CH4 = global.CH4, { sin, cos, atan2, sqrt, abs, hypot, PI, acos } = Math, rad = RB.rad;
const CH5 = {}, H = PI / 2, row = (al, a, d, th, j) => ({ al, a, d, th, j });

/* ---- matrices of any size (a Jacobian need not be square) ---- */
CH5.mul = (A, x) => A.map(r => r.reduce((s, a, i) => s + a * x[i], 0));                                   // A x
CH5.tr = A => A[0].map((_, j) => A.map(r => r[j]));                                                       // Aᵀ
CH5.mm = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, a, k) => s + a * B[k][j], 0)));              // A B
CH5.det2 = J => J[0][0] * J[1][1] - J[0][1] * J[1][0];
CH5.inv2 = J => { const d = CH5.det2(J); return abs(d) < 1e-12 ? null : [[J[1][1] / d, -J[0][1] / d], [-J[1][0] / d, J[0][0] / d]]; };
CH5.det3 = RB.det3;
CH5.inv3 = A => { const d = RB.det3(A); if (abs(d) < 1e-14) return null; const c = (i, j) => { const r = [0, 1, 2].filter(k => k !== i), k = [0, 1, 2].filter(m => m !== j); return ((i + j) % 2 ? -1 : 1) * (A[r[0]][k[0]] * A[r[1]][k[1]] - A[r[0]][k[1]] * A[r[1]][k[0]]); };
  return [0, 1, 2].map(i => [0, 1, 2].map(j => c(j, i) / d)); };
/* the Jacobian of f at q by central differences: column j = (f(q + h e_j) − f(q − h e_j)) / 2h. The check every closed form below must pass. */
CH5.numJ = (f, q, h = 1e-6) => { const J = f(q).map(() => []); q.forEach((_, j) => { const a = q.slice(), b = q.slice(); a[j] += h; b[j] -= h; const fa = f(a), fb = f(b); fa.forEach((x, i) => { J[i][j] = (x - fb[i]) / (2 * h); }); }); return J; };

/* ---- the two-link planar arm (old part 1 slides 8–9; Craig example 5.3): x = l1 c1 + l2 c12, y = l1 s1 + l2 s12.
   J in frame {0}; J3 the same Jacobian written in the tool's own frame {3} (Craig eq. 5.66): ^0J = R_Z(θ1 + θ2) ^3J. det J = l1 l2 s2 ---- */
CH5.two = { L: [0.5, 0.4], q: [rad(30), rad(45)],
  P: (L, q) => [L[0] * cos(q[0]) + L[1] * cos(q[0] + q[1]), L[0] * sin(q[0]) + L[1] * sin(q[0] + q[1])],
  J: (L, q) => { const s1 = sin(q[0]), c1 = cos(q[0]), s12 = sin(q[0] + q[1]), c12 = cos(q[0] + q[1]); return [[-L[0] * s1 - L[1] * s12, -L[1] * s12], [L[0] * c1 + L[1] * c12, L[1] * c12]]; },
  J3: (L, q) => [[L[0] * sin(q[1]), 0], [L[0] * cos(q[1]) + L[1], L[1]]],
  det: (L, q) => L[0] * L[1] * sin(q[1]),
  /* joint rates for a tool-point velocity v (null at a singularity) and joint torques for a force F the tool applies (Craig 5.10: τ = Jᵀ F) */
  rates: (L, q, v) => { const Ji = CH5.inv2(CH5.two.J(L, q)); return Ji ? CH5.mul(Ji, v) : null; },
  tau: (L, q, F) => CH5.mul(CH5.tr(CH5.two.J(L, q)), F) };

/* ---- a turning joint with a sliding link (RP, the polar arm; the class exercise): x = d2 c1, y = d2 s1 ---- */
CH5.rp = { q: [rad(60), 0.5], P: q => [q[1] * cos(q[0]), q[1] * sin(q[0])], J: q => [[-q[1] * sin(q[0]), cos(q[0])], [q[1] * cos(q[0]), sin(q[0])]] };

/* ---- the planar arm with three turning joints and a tool of length L3 (chapter 3's arm, CH3.EX.p3r): X = (x, y, φ) of the tool frame {4}. Homework 5.1 ---- */
CH5.p3r = { L: (() => { const L = CH3.EX.p3r.L; return [L.L1, L.L2, L.L3]; })(), q: CH3.EX.p3r.q,
  X: (L, q) => { const a = q[0], b = a + q[1], c = b + q[2]; return [L[0] * cos(a) + L[1] * cos(b) + L[2] * cos(c), L[0] * sin(a) + L[1] * sin(b) + L[2] * sin(c), c]; },
  J: (L, q) => { const a = q[0], b = a + q[1], c = b + q[2], sx = [L[0] * sin(a), L[1] * sin(b), L[2] * sin(c)], cx = [L[0] * cos(a), L[1] * cos(b), L[2] * cos(c)];
    return [[-sx[0] - sx[1] - sx[2], -sx[1] - sx[2], -sx[2]], [cx[0] + cx[1] + cx[2], cx[1] + cx[2], cx[2]], [1, 1, 1]]; } };

/* ---- the RRP arm of homework 4 (CH4.rrp): x = c1 (L2 + d3 c2), y = s1 (L2 + d3 c2), z = L1 + d3 s2. Homework 5.2 ---- */
CH5.rrp = { L: CH4.rrp.L, q: CH4.rrp.q, P: CH4.rrp.P,
  J: (L, q) => { const [t1, t2, d3] = q, s1 = sin(t1), c1 = cos(t1), s2 = sin(t2), c2 = cos(t2), k = L.L2 + d3 * c2; return [[-s1 * k, -c1 * d3 * s2, c1 * c2], [c1 * k, -s1 * d3 * s2, s1 * c2], [0, d3 * c2, s2]]; } };

/* ---- the Stanford arm's first three joints (old part 1 slides 12–13, from Khatib): turn θ1 about the vertical, shoulder offset L2, turn θ2, slide d3.
   Craig's table (α, a, d, θ): (0, 0, 0, θ1), (−90°, 0, L2, θ2), (90°, 0, d3, 0).  x_P = (c1 s2 d3 − s1 L2, s1 s2 d3 + c1 L2, c2 d3).
   Jv: the three columns that are not zero of the 3 × 6 Jacobian of the position (the wrist joints do not move the wrist point) ---- */
CH5.stan = { L2: 0.15, q: [rad(30), rad(60), 0.5],
  rows: L2 => [row(0, 0, 0, 0, 'R'), row(-H, 0, L2, 0, 'R'), row(H, 0, 0, 0, 'P')],
  sym: [['0', '0', '0', 'θ₁'], ['−90°', '0', 'L₂', 'θ₂'], ['90°', '0', 'd₃', '0']],
  P: (L2, q) => { const [t1, t2, d3] = q, s1 = sin(t1), c1 = cos(t1), s2 = sin(t2), c2 = cos(t2); return [c1 * s2 * d3 - s1 * L2, s1 * s2 * d3 + c1 * L2, c2 * d3]; },
  Jv: (L2, q) => { const [t1, t2, d3] = q, s1 = sin(t1), c1 = cos(t1), s2 = sin(t2), c2 = cos(t2); return [[-s1 * s2 * d3 - c1 * L2, c1 * c2 * d3, c1 * s2], [c1 * s2 * d3 - s1 * L2, s1 * c2 * d3, s1 * s2], [0, -s2 * d3, c2]]; } };

/* ---- the department's Yaskawa GP8 (CH3.EX.gp8): the wrist centre is r = a1 + a2 c2 + a3 c23 − d4 s23 out and p_z = −a2 s2 − a3 s23 − d4 c23 up, turned by θ1.
   Jv: derivative of (c1 r, s1 r, p_z) by θ1, θ2, θ3 (θ4–θ6 do not move the wrist centre: those three columns are zero).
   ∂r/∂θ2 = p_z, ∂p_z/∂θ2 = −(r − a1); ∂r/∂θ3 = w = −a3 s23 − d4 c23, ∂p_z/∂θ3 = −u, u = a3 c23 − d4 s23 ---- */
CH5.gp8 = { P: q => RB.Tp(CH3.EX.gp8.T06(q)),
  parts: q => { const G = CH3.EX.gp8, s2 = sin(q[1]), c2 = cos(q[1]), s23 = sin(q[1] + q[2]), c23 = cos(q[1] + q[2]), u = G.a3 * c23 - G.d4 * s23, w = -G.a3 * s23 - G.d4 * c23; return { r: G.a1 + G.a2 * c2 + u, pz: -G.a2 * s2 + w, u, w, a1: G.a1 }; },
  Jv: q => { const { r, pz, u, w, a1 } = CH5.gp8.parts(q), s1 = sin(q[0]), c1 = cos(q[0]); return [[-s1 * r, c1 * pz, c1 * w], [c1 * r, s1 * pz, s1 * w], [0, -(r - a1), -u]]; } };

/* ---- the basic Jacobian of any DH table (Craig's convention), in frame {0}, for the origin of the last frame: [v; ω] = J0 q̇.
   Column i: [Z_i × (p_n − p_i); Z_i] for a turning joint, [Z_i; 0] for a sliding one (the explicit form, derived in chapter 6). 6 × n ---- */
CH5.J0 = (rows, q) => { const F = RB.fkDH(rows, q), pn = RB.Tp(F[F.length - 1]), J = [[], [], [], [], [], []]; let k = 0;
  rows.forEach((r, i) => { if (!r.j) return; const T = F[i + 1], z = RB.col(RB.TR(T), 2), p = RB.Tp(T), turn = r.j === 'R', lin = turn ? RB.cross(z, [pn[0] - p[0], pn[1] - p[1], pn[2] - p[2]]) : z;
    [...lin, ...(turn ? z : [0, 0, 0])].forEach((x, m) => { J[m][k] = x; }); k++; }); return J; };
/* the angular velocity read from a rotation matrix that changes by dR in dt: Ṙ Rᵀ is the skew matrix of ω (used by the tests) */
CH5.omegaOf = (R, Rd) => { const S = RB.mm(Rd, RB.tr(R)); return [(S[2][1] - S[1][2]) / 2, (S[0][2] - S[2][0]) / 2, (S[1][0] - S[0][1]) / 2]; };

/* ---- how a position or an orientation is written (old part 2 slides 4–9, Khatib): ẋ_P = E_P(x_P) v and ẋ_R = E_R(x_R) ω.
   cyl: (ρ, θ, z); sph: (ρ, θ, φ) with φ measured from Z; zxz: Z–X–Z Euler angles (α, β, γ), R = R_Z(α) R_X(β) R_Z(γ) ---- */
CH5.rep = {
  cyl: p => [hypot(p[0], p[1]), atan2(p[1], p[0]), p[2]],
  Ecyl: p => { const r = hypot(p[0], p[1]), c = p[0] / r, s = p[1] / r; return [[c, s, 0], [-s / r, c / r, 0], [0, 0, 1]]; },
  sph: p => { const rho = hypot(p[0], p[1], p[2]); return [rho, atan2(p[1], p[0]), acos(p[2] / rho)]; },
  Esph: p => { const rho = hypot(p[0], p[1], p[2]), th = atan2(p[1], p[0]), ph = acos(p[2] / rho), c = cos(th), s = sin(th), cp = cos(ph), sp = sin(ph);
    return [[c * sp, s * sp, cp], [-s / (rho * sp), c / (rho * sp), 0], [c * cp / rho, s * cp / rho, -sp / rho]]; },
  Rzxz: (al, be, ga) => RB.mm(RB.mm(RB.Rz(al), RB.Rx(be)), RB.Rz(ga)),
  zxz: R => [atan2(R[0][2], -R[1][2]), atan2(hypot(R[0][2], R[1][2]), R[2][2]), atan2(R[2][0], R[2][1])],
  Ezxz: (al, be) => { const sa = sin(al), ca = cos(al), sb = sin(be), cb = cos(be); return [[-sa * cb / sb, ca * cb / sb, 1], [ca, sa, 0], [sa / sb, -ca / sb, 0]]; } };

/* ---- the numbers of the worked examples (slides and pages show these; tests.js checks them) ---- */
CH5.EX = (() => { const add = (a, b) => a.map((x, i) => x + b[i]), sub = (a, b) => a.map((x, i) => x - b[i]), norm = a => sqrt(a.reduce((s, x) => s + x * x, 0)), T = CH5.two, L = T.L, q = T.q, J = T.J(L, q), p = T.P(L, q);
  /* a small step: what the Jacobian predicts against what the arm really does */
  const stepAt = dq => { const pred = CH5.mul(J, dq), act = sub(T.P(L, add(q, dq)), p); return { dq, pred, act, err: norm(sub(act, pred)), rel: norm(sub(act, pred)) / norm(act) }; };
  const qd = [0.5, -1.0], v = CH5.mul(J, qd);
  const RP = CH5.rp, rq = RP.q, rqd = [0.4, 0.1], P3 = CH5.p3r, pq = P3.q, pqd = [0.2, -0.4, 0.6], RR = CH5.rrp, rrq = RR.q, rrqd = [0.3, -0.2, 0.05], rrdq = [rad(1), rad(-1), 0.005], rrJ = RR.J(RR.L, rrq);
  const S = CH5.stan, sq = S.q, sqd = [0.3, -0.2, 0.1], G = CH3.EX.gp8, gq = G.q, gqd = [0.5, -0.3, 0.4], gqd6 = [0.5, -0.3, 0.4, 1.0, -0.5, 0.8], gJ0 = CH5.J0(G.rows(), gq), gvw = CH5.mul(gJ0, gqd6);
  const cp = [0.3, 0.4, 0.25], cv = [0.1, -0.05, 0.02];
  /* train and car (Craig example 5.1, in km/h): both drive along X_U; {T} is parallel to {U}, {C} has its Y axis pointing forward (^U_C R = R_Z(−90°)) */
  const RC = RB.Rz(-H), RT = RB.I3(), vT = [100, 0, 0], vC = [30, 0, 0], RCt = RB.tr(RC);
  /* the tool point driven along +X at 0.1 m/s while the arm stretches out (Craig example 5.5): the joint rates grow without bound as θ2 → 0 */
  const vx = [0.1, 0], near = [0.6, 0.8, 0.88, 0.898].map(x => { const s = RB.ik2(L[0], L[1], x, 0, -1), qq = [s.t1, s.t2]; return { x, q: qq, det: T.det(L, qq), qd: T.rates(L, qq, vx) }; });
  const F = [0, -10];
  return {
    two: { L, q, p, J, det: T.det(L, q), step: stepAt([rad(2), rad(-3)]), big: stepAt([rad(10), rad(-15)]), sizes: [1, 5, 20].map(d => stepAt([rad(d), rad(d)])), qd, v, speed: norm(v), cols: [0, 1].map(j => [J[0][j] * qd[j], J[1][j] * qd[j]]) },
    rp: { q: rq, qd: rqd, p: RP.P(rq), J: RP.J(rq), v: CH5.mul(RP.J(rq), rqd) },
    p3r: { L: P3.L, q: pq, qd: pqd, X: P3.X(P3.L, pq), J: P3.J(P3.L, pq), Xd: CH5.mul(P3.J(P3.L, pq), pqd) },
    rrp: { L: RR.L, q: rrq, qd: rrqd, p: RR.P(RR.L, rrq), J: rrJ, v: CH5.mul(rrJ, rrqd), dq: rrdq, pred: CH5.mul(rrJ, rrdq), act: sub(RR.P(RR.L, add(rrq, rrdq)), RR.P(RR.L, rrq)) },
    stan: { L2: S.L2, q: sq, qd: sqd, p: S.P(S.L2, sq), J: S.Jv(S.L2, sq), v: CH5.mul(S.Jv(S.L2, sq), sqd) },
    gp8: { q: gq, qd: gqd, p: CH5.gp8.P(gq), parts: CH5.gp8.parts(gq), J: CH5.gp8.Jv(gq), v: CH5.mul(CH5.gp8.Jv(gq), gqd), qd6: gqd6, J0: gJ0, v6: gvw.slice(0, 3), w6: gvw.slice(3) },
    rep: { p: cp, v: cv, cyl: CH5.rep.cyl(cp), E: CH5.rep.Ecyl(cp), rate: CH5.mul(CH5.rep.Ecyl(cp), cv) },
    train: { vT: 100, vC: 30, RC, RT, a: vC, b: RB.mv(RCt, vT), c: RB.mv(RCt, sub(vC, vT)) },
    inv: { q, v: vx, qd: T.rates(L, q, vx) },
    near,
    force: { q, F, tau: T.tau(L, q, F) } }; })();

global.CH5 = CH5;
})(typeof window !== 'undefined' ? window : globalThis);
