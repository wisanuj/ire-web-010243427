/* ch6.js — chapter 6, the velocity of rigid bodies, its propagation from link to link and the Jacobian that comes out of it
   (Craig, Introduction to Robotics, 3rd ed., 5.3–5.7; the old slides "Chapter 6 Linear and Rotational Velocity of Rigid Bodies", part I,
   and part II: the Jacobian in explicit form, after Khatib).
   Every result of the propagation is checked in tests.js against the basic Jacobian of ch5.js (itself checked against numerical derivatives).
   Works in the browser (window.CH6) and in deno (globalThis.CH6). Needs robot.js, ch3.js, ch4.js and ch5.js. */
(function (global) {
'use strict';
const RB = global.RB, CH3 = global.CH3, CH4 = global.CH4, CH5 = global.CH5, { sin, cos, PI, hypot } = Math, rad = RB.rad;
const CH6 = {}, H = PI / 2, row = (al, a, d, th, j) => ({ al, a, d, th, j }), add = RB.add, cross = RB.cross, mv = RB.mv, tr = RB.tr;

/* ---- the velocity of a rigid body (Craig 5.3–5.4) ---- */
/* the skew-symmetric matrix of Ω: S p = Ω × p */
CH6.skew = w => [[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]];
/* a point Q of a moving frame {B}, watched from {A} (Craig 5.13): ^AV_Q = ^AV_BORG + ^A_BR ^BV_Q + ^AΩ_B × ^A_BR ^BQ.
   R = ^A_BR, vB = ^AV_BORG, vQ = ^BV_Q, W = ^AΩ_B, Q = ^BQ. Returns the three parts, ^A_BR ^BQ and the sum */
CH6.vQ = (R, vB, vQ, W, Q) => { const rel = mv(R, vQ), rQ = mv(R, Q), rot = cross(W, rQ); return { trans: vB.slice(), rel, rot, rQ, v: add(add(vB, rel), rot) }; };

/* ---- velocity propagation (Craig 5.6). rows: a DH table in Craig's convention (robot.js), q the joint values, qd their rates (one per row, as in RB.fkDH).
   Turning joint (5.45, 5.47):  ^{i+1}ω_{i+1} = ^{i+1}_iR ^iω_i + θ̇_{i+1} Ẑ,   ^{i+1}v_{i+1} = ^{i+1}_iR (^iv_i + ^iω_i × ^iP_{i+1})
   Sliding joint (5.48):        ^{i+1}ω_{i+1} = ^{i+1}_iR ^iω_i,                ^{i+1}v_{i+1} = ^{i+1}_iR (^iv_i + ^iω_i × ^iP_{i+1}) + ḋ_{i+1} Ẑ
   A row without a joint (a tool frame) passes the velocity on with no rate of its own.
   Returns one entry per frame 0…n: { w, v } written in the frame's own axes, { w0, v0 } the same vectors written in {0}, and for i ≥ 1 the pieces a
   worked solution shows: R = ^{i-1}_iR, P = ^{i-1}P_i, Rw = ^i_{i-1}R ^{i-1}ω_{i-1}, wxP = ^{i-1}ω_{i-1} × ^{i-1}P_i, sum = ^{i-1}v_{i-1} + wxP ---- */
CH6.prop = (rows, q, qd) => { const out = [{ w: [0, 0, 0], v: [0, 0, 0], w0: [0, 0, 0], v0: [0, 0, 0], R0: RB.I3() }]; let R0 = RB.I3();
  rows.forEach((r, i) => { const th = r.th + (r.j === 'R' ? (q[i] || 0) : 0), d = r.d + (r.j === 'P' ? (q[i] || 0) : 0), T = RB.dhT(r.al, r.a, th, d), R = RB.TR(T), Rt = tr(R), P = RB.Tp(T), p = out[i], rate = r.j ? (qd[i] || 0) : 0;
    const Rw = mv(Rt, p.w), wxP = cross(p.w, P), sum = add(p.v, wxP), w = add(Rw, r.j === 'R' ? [0, 0, rate] : [0, 0, 0]), v = add(mv(Rt, sum), r.j === 'P' ? [0, 0, rate] : [0, 0, 0]);
    R0 = RB.mm(R0, R); out.push({ w, v, w0: mv(R0, w), v0: mv(R0, v), R0, R, P, kind: r.j || 'F', rate, Rw, wxP, sum }); });
  return out; };
/* a Jacobian written in another frame (Craig 5.71): ^BJ = [^B_AR 0; 0 ^B_AR] ^AJ. R = ^B_AR; J has 3 rows (v) or 6 (v and ω) */
CH6.Jin = (R, J) => { const out = J.map(() => []); for (let j = 0; j < J[0].length; j++) for (let b = 0; b < J.length; b += 3) mv(R, [J[b][j], J[b + 1][j], J[b + 2][j]]).forEach((x, k) => { out[b + k][j] = x; }); return out; };
/* the Jacobian of a DH table written in its own last frame {n}: ^nJ = [^n_0R 0; 0 ^n_0R] ^0J */
CH6.Jn = (rows, q) => CH6.Jin(tr(RB.TR(RB.fkDH(rows, q).pop())), CH5.J0(rows, q));

/* ---- the Jacobian in explicit form (old part II, Khatib): one column per joint, read off the picture.
   Turning joint i: [Z_i × (p_n − p_i); Z_i].  Sliding joint i: [Z_i; 0].  Z_i = the third column of ^0_iT, p_i its fourth.
   Returns for every joint { k, kind, z, p, arm = p_n − p_i, lin, ang }, everything written in {0} ---- */
CH6.cols = (rows, q) => { const F = RB.fkDH(rows, q), pn = RB.Tp(F[F.length - 1]); let k = 0;
  return rows.map((r, i) => ({ r, T: F[i + 1] })).filter(x => x.r.j).map(({ r, T }) => { const z = RB.col(RB.TR(T), 2), p = RB.Tp(T), arm = [pn[0] - p[0], pn[1] - p[1], pn[2] - p[2]], turn = r.j === 'R'; return { k: ++k, kind: r.j, z, p, arm, lin: turn ? cross(z, arm) : z, ang: turn ? z : [0, 0, 0] }; }); };

/* ---- the two-link arm of Craig's example 5.3 as a DH table: two turning joints and a frame {3} at the tip with the same orientation as {2} ---- */
CH6.two = { L: CH5.two.L, q: CH5.two.q, rows: L => [row(0, 0, 0, 0, 'R'), row(0, L[0], 0, 0, 'R'), row(0, L[1], 0, 0)],
  sym: [['0', '0', '0', 'θ₁'], ['0', 'l₁', '0', 'θ₂'], ['0', 'l₂', '0', '0']],
  /* Craig 5.55–5.57: the velocity of the tip written in {3} and in {0} */
  w3: qd => [0, 0, qd[0] + qd[1]],
  v3: (L, q, qd) => [L[0] * sin(q[1]) * qd[0], L[0] * cos(q[1]) * qd[0] + L[1] * (qd[0] + qd[1]), 0],
  v0: (L, q, qd) => { const s1 = sin(q[0]), c1 = cos(q[0]), s12 = sin(q[0] + q[1]), c12 = cos(q[0] + q[1]), w = qd[0] + qd[1]; return [-L[0] * s1 * qd[0] - L[1] * s12 * w, L[0] * c1 * qd[0] + L[1] * c12 * w, 0]; } };

/* ---- the RRP arm of homework 4 (CH4.rrp), the class exercise in pairs: its velocity in closed form, written in {3} ---- */
CH6.rrp = { L: CH4.rrp.L, q: CH4.rrp.q, rows: CH4.rrp.rows,
  w3: (q, qd) => [-cos(q[1]) * qd[0], -qd[1], sin(q[1]) * qd[0]],
  v3: (L, q, qd) => [-q[2] * qd[1], (L.L2 + q[2] * cos(q[1])) * qd[0], qd[2]],
  J3: (L, q) => [[0, -q[2], 0], [L.L2 + q[2] * cos(q[1]), 0, 0], [0, 0, 1]] };

/* ---- the RPR arm of chapter 3 (CH3.EX.rpr), homework 6: its velocity in closed form, written in {3} and in {0}; r = d2 + L2 ---- */
CH6.rpr = { L: CH3.EX.rpr.L, q: CH3.EX.rpr.q, rows: CH3.EX.rpr.rows,
  w3: (q, qd) => [sin(q[2]) * qd[0], cos(q[2]) * qd[0], qd[2]],
  v3: (L, q, qd) => { const r = q[1] + L.L2; return [cos(q[2]) * r * qd[0], -sin(q[2]) * r * qd[0], qd[1]]; },
  v0: (L, q, qd) => { const r = q[1] + L.L2, s1 = sin(q[0]), c1 = cos(q[0]); return [c1 * r * qd[0] + s1 * qd[1], s1 * r * qd[0] - c1 * qd[1], 0]; },
  w0: (q, qd) => [sin(q[0]) * qd[2], -cos(q[0]) * qd[2], qd[0]] };

/* ---- the Stanford arm with all six joints (old part II slides 9–17): Craig's table (α, a, d, θ) =
   (0, 0, 0, θ1), (−90°, 0, d2, θ2), (90°, 0, d3, 0), (0, 0, 0, θ4), (−90°, 0, 0, θ5), (90°, 0, 0, θ6); d2 = L2 fixed.
   Z: the six joint axes in {0} as the old slides list them (the third columns of ^0_1T … ^0_6T) ---- */
CH6.stan = { L2: CH5.stan.L2, q: [...CH5.stan.q, rad(20), rad(40), rad(-30)],
  rows: L2 => [...CH5.stan.rows(L2), row(0, 0, 0, 0, 'R'), row(-H, 0, 0, 0, 'R'), row(H, 0, 0, 0, 'R')],
  sym: [['0', '0', '0', 'θ₁'], ['−90°', '0', 'd₂', 'θ₂'], ['90°', '0', 'd₃', '0'], ['0', '0', '0', 'θ₄'], ['−90°', '0', '0', 'θ₅'], ['90°', '0', '0', 'θ₆']],
  Z: q => { const [t1, t2, , t4, t5] = q, s1 = sin(t1), c1 = cos(t1), s2 = sin(t2), c2 = cos(t2), s4 = sin(t4), c4 = cos(t4), s5 = sin(t5), c5 = cos(t5);
    return [[0, 0, 1], [-s1, c1, 0], [c1 * s2, s1 * s2, c2], [c1 * s2, s1 * s2, c2], [-c1 * c2 * s4 - s1 * c4, -s1 * c2 * s4 + c1 * c4, s2 * s4],
      [c1 * c2 * c4 * s5 - s1 * s4 * s5 + c1 * s2 * c5, s1 * c2 * c4 * s5 + c1 * s4 * s5 + s1 * s2 * c5, -s2 * c4 * s5 + c2 * c5]]; } };

/* ---- the numbers of the worked examples (slides and pages show these; tests.js checks them) ---- */
CH6.EX = (() => { const T = CH6.two, L = T.L, q = T.q, qd = CH5.EX.two.qd, P2 = CH6.prop(T.rows(L), q, qd), J0 = CH5.J0(T.rows(L), q), J3 = CH6.Jn(T.rows(L), q);
  /* a body {B} that slides and turns, and a point Q that crawls along it (the three parts of Craig 5.13) */
  const Rb = RB.Rz(rad(30)), rig = { th: rad(30), R: Rb, vB: [0.1, 0.05, 0], vQ: [0.08, 0, 0], W: [0, 0, 0.5], Q: [0.3, 0.1, 0], O: [0.25, 0.15, 0] };
  const R = CH6.rrp, rq = R.q, rqd = CH5.EX.rrp.qd, PR = CH6.prop(R.rows(R.L), rq, rqd), G = CH3.EX.gp8, gq = G.q, gqd = CH5.EX.gp8.qd6, S = CH6.stan, sqd = [0.3, -0.2, 0.1, 0.5, -0.4, 0.6];
  const H6 = CH6.rpr, hq = H6.q, hqd = [0.4, 0.1, -0.6], PH = CH6.prop(H6.rows(H6.L), hq, hqd);
  return {
    rigid: { ...rig, ...CH6.vQ(Rb, rig.vB, rig.vQ, rig.W, rig.Q) },
    two: { L, q, qd, rows: T.rows(L), prop: P2, w3: P2[3].w, v3: P2[3].v, v0: P2[3].v0, J0, J3, J0v: J0.slice(0, 2).map(r => r.slice()), J3v: J3.slice(0, 2).map(r => r.slice()) },
    rrp: { L: R.L, q: rq, qd: rqd, prop: PR, w3: PR[3].w, v3: PR[3].v, v0: PR[3].v0, w0: PR[3].w0, J3: R.J3(R.L, rq), J0: CH5.rrp.J(R.L, rq) },
    gp8: { q: gq, qd: gqd, prop: CH6.prop(G.rows(), gq, gqd), cols: CH6.cols(G.rows(), gq), J0: CH5.J0(G.rows(), gq) },
    stan: { L2: S.L2, q: S.q, qd: sqd, rows: S.rows(S.L2), cols: CH6.cols(S.rows(S.L2), S.q), J0: CH5.J0(S.rows(S.L2), S.q), Z: S.Z(S.q), vw: CH5.mul(CH5.J0(S.rows(S.L2), S.q), sqd) },
    hw: { L: H6.L, q: hq, qd: hqd, prop: PH, w3: PH[3].w, v3: PH[3].v, v0: PH[3].v0, w0: PH[3].w0, J0: CH5.J0(H6.rows(H6.L), hq), J3: CH6.Jn(H6.rows(H6.L), hq) } }; })();

global.CH6 = CH6;
})(typeof window !== 'undefined' ? window : globalThis);
