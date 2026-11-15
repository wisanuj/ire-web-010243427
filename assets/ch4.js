/* ch4.js — chapter 4, inverse manipulator kinematics (Craig, Introduction to Robotics, 3rd ed., chapter 4).
   Every solver here is checked in tests.js by putting its answers back through the forward kinematics of ch3.js / robot.js.
   Works in the browser (window.CH4) and in deno (globalThis.CH4). Needs robot.js and ch3.js. */
(function (global) {
'use strict';
const RB = global.RB, CH3 = global.CH3, { sin, cos, atan2, sqrt, abs, hypot, PI, min } = Math, rad = RB.rad, wrap = RB.wrap;
const CH4 = {};

/* ---- the planar 3R arm (Craig 4.4): goal = the wrist frame {3} as (x, y, φ) in {0}; L1, L2 the link lengths.
   Algebraic: c2 = (x² + y² − L1² − L2²) / (2 L1 L2), s2 = ±√(1 − c2²), θ1 = atan2(y, x) − atan2(k2, k1) with k1 = L1 + L2 c2, k2 = L2 s2, θ3 = φ − θ1 − θ2.
   Returns { ok, c2, sols: [[θ1, θ2, θ3] for s2 > 0, for s2 < 0] }; ok false (no sols) when |c2| > 1 (the goal is out of reach) ---- */
CH4.p3r = (L1, L2, x, y, phi) => { const c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2); if (abs(c2) > 1 + 1e-12) return { ok: false, c2, sols: [] };
  const sols = [1, -1].map(sg => { const s2 = sg * sqrt(Math.max(0, 1 - c2 * c2)), t2 = atan2(s2, c2), k1 = L1 + L2 * c2, k2 = L2 * s2, t1 = atan2(y, x) - atan2(k2, k1); return [wrap(t1), t2, wrap(phi - t1 - t2)]; });
  return { ok: true, c2, k: sols.map(([, t2]) => [L1 + L2 * cos(t2), L2 * sin(t2)]), sols }; };
/* the same goal solved with geometry (Craig 4.4, second half): law of cosines in the triangle base–elbow–wrist.
   β = atan2(y, x), cos ψ = (x² + y² + L1² − L2²) / (2 L1 √(x² + y²)), θ1 = β ± ψ, θ2 = ∓(180° − angle at the elbow) */
CH4.p3rGeo = (L1, L2, x, y, phi) => { const r2 = x * x + y * y, r = sqrt(r2), cE = (L1 * L1 + L2 * L2 - r2) / (2 * L1 * L2), cP = (r2 + L1 * L1 - L2 * L2) / (2 * L1 * r);
  if (abs(cE) > 1 + 1e-12 || r < 1e-12) return { ok: false, sols: [] }; const beta = atan2(y, x), psi = Math.acos(Math.max(-1, Math.min(1, cP))), elbow = Math.acos(Math.max(-1, Math.min(1, cE)));
  const sols = [[beta - psi, PI - elbow], [beta + psi, -(PI - elbow)]].map(([t1, t2]) => [wrap(t1), wrap(t2), wrap(phi - t1 - t2)]); return { ok: true, beta, psi, elbow, sols }; };
/* where the planar 3R arm puts its wrist frame (x, y, φ): the forward kinematics the solutions are checked with */
CH4.p3rFK = (L1, L2, q) => [L1 * cos(q[0]) + L2 * cos(q[0] + q[1]), L1 * sin(q[0]) + L2 * sin(q[0] + q[1]), wrap(q[0] + q[1] + q[2])];

/* ---- a cos θ + b sin θ = c (Craig 4.5, example 4.3): with u = tan(θ/2), cos θ = (1 − u²)/(1 + u²), sin θ = 2u/(1 + u²) the equation becomes
   (a + c) u² − 2b u + (c − a) = 0, so u = (b ± √(b² + a² − c²)) / (a + c). No real solution when c² > a² + b²; when a + c = 0 one root is θ = 180°.
   Returns { ok, u: [roots], sols: [θ] } ---- */
CH4.half = (a, b, c) => { const D = b * b + a * a - c * c; if (D < -1e-12) return { ok: false, D, u: [], sols: [] }; const sq = sqrt(Math.max(0, D));
  if (abs(a + c) < 1e-12) { const u = abs(b) < 1e-12 ? [] : [(c - a) / (2 * b)]; return { ok: true, D, u, sols: [PI, ...u.map(x => 2 * Math.atan(x))] }; }
  const u = [(b + sq) / (a + c), (b - sq) / (a + c)]; return { ok: true, D, u, sols: u.map(x => 2 * Math.atan(x)) }; };

/* ---- the Yaskawa GP8 (the six-axis arm of chapter 3, CH3.EX.gp8): axes 4, 5, 6 meet at the wrist centre, so the position of the wrist
   centre fixes θ1, θ2, θ3 and the orientation then fixes θ4, θ5, θ6 (Pieper, Craig 4.6; worked as Craig 4.7 works the PUMA 560, which also has a3 ≠ 0).
   From ^0_6T (chapter 3): p_x = c1 (a1 + a2 c2 + a3 c23 − d4 s23), p_y = s1 (…), p_z = −a2 s2 − a3 s23 − d4 c23.
   θ1 = atan2(p_y, p_x) (arm in front) or that + 180° (arm reaching over its back);  r = c1 p_x + s1 p_y, u = r − a1;
   u² + p_z² = a2² + a3² + d4² + 2 a2 (a3 c3 − d4 s3)  ⟹  a3 c3 − d4 s3 = K, K = (u² + p_z² − a2² − a3² − d4²) / (2 a2): the equation
   a cos θ + b sin θ = c of Craig 4.5 (CH4.half), two roots (elbow up or down);
   (u, −p_z) is (a2 + K, a3 s3 + d4 c3) turned by θ2  ⟹  θ2 = atan2(−p_z, u) − atan2(a3 s3 + d4 c3, a2 + K);
   ^3_6R = (^0_3R)ᵀ ^0_6R holds c5 = r23, −c4 s5 = r13, s4 s5 = r33, s5 c6 = r21, −s5 s6 = r22  ⟹  θ5 = atan2(±√(r13² + r33²), r23), θ4, θ6 from s5;
   the other wrist: θ4 + 180°, −θ5, θ6 + 180°. Up to 8 solutions. Each: { q, shoulder: 'front'|'back', elbow: 'up'|'down', wrist: 'no flip'|'flip', sing, K } ---- */
const GP8 = () => CH3.EX.gp8;
CH4.R03 = q => { const E = GP8(), F = RB.fkDH(E.rows().slice(0, 3), q.slice(0, 3)); return RB.TR(F[3]); };
/* elbow up or down, as one sees it: is the elbow (origin of {3}) above the straight line from the shoulder (origin of {2}) to the wrist centre p? */
CH4.elbowUp = (q3, p) => { const F = RB.fkDH(GP8().rows().slice(0, 3), q3), S = RB.Tp(F[2]), E = RB.Tp(F[3]), d = p.map((x, i) => x - S[i]), e = E.map((x, i) => x - S[i]), dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
  const t = dd < 1e-12 ? 0 : (e[0] * d[0] + e[1] * d[1] + e[2] * d[2]) / dd; return E[2] > S[2] + t * d[2] + 1e-12; };
CH4.gp8IK = T => { const E = GP8(), { a1, a2, a3, d4 } = E, [px, py, pz] = [T[0][3], T[1][3], T[2][3]], out = [];
  const R06 = RB.TR(T), base = hypot(px, py) < 1e-9 ? 0 : atan2(py, px);
  [['front', base], ['back', base + PI]].forEach(([sh, t1]) => { const r = cos(t1) * px + sin(t1) * py, u = r - a1, K = (u * u + pz * pz - a2 * a2 - a3 * a3 - d4 * d4) / (2 * a2), H3 = CH4.half(a3, -d4, K);
    if (!H3.ok) return; const roots = Math.abs(H3.D) < 1e-12 ? H3.sols.slice(0, 1) : H3.sols;
    roots.forEach(t3r => { const t3 = wrap(t3r), t2 = atan2(-pz, u) - atan2(a3 * sin(t3) + d4 * cos(t3), a2 + K), q3 = [wrap(t1), wrap(t2), t3], el = CH4.elbowUp(q3, [px, py, pz]) ? 'up' : 'down';
      const R = RB.mm(RB.tr(CH4.R03(q3)), R06), r13 = R[0][2], r23 = R[1][2], r33 = R[2][2], r21 = R[1][0], r22 = R[1][1], s5 = hypot(r13, r33);
      if (s5 < 1e-9) { /* θ5 = 0 (or 180°): axes 4 and 6 line up, only θ4 + θ6 is fixed; choose θ4 = 0 (Craig: the wrist singularity) */
        const t5 = r23 > 0 ? 0 : PI, t46 = atan2(-R[0][1], R[0][0]); out.push({ q: [...q3, 0, t5, wrap(r23 > 0 ? t46 : -t46)], shoulder: sh, elbow: el, wrist: 'no flip', sing: true, K }); return; }
      [['no flip', 1], ['flip', -1]].forEach(([wr, sg]) => { const s5s = sg * s5, t5 = atan2(s5s, r23), t4 = atan2(r33 / s5s, -r13 / s5s), t6 = atan2(-r22 / s5s, r21 / s5s);
        out.push({ q: [...q3, wrap(t4), wrap(t5), wrap(t6)], shoulder: sh, elbow: el, wrist: wr, sing: false, K }); }); }); });
  return out.map(s => ({ ...s, q: CH4.fitLim(s.q) })); };
/* the forward kinematics the solutions are checked with: ^0_6T of CH3.EX.gp8 (ch3.js, checked there against the six link transforms) */
CH4.gp8FK = q => GP8().T06(q);
/* is every joint inside the robot's range? The ranges are the GP8's own (data sheet), in DH degrees (CH3.EX.gp8.lim); an angle may also be
   reached a full turn away (θ6: ±360°) */
const in1 = (x, r) => { const d = RB.deg(x); return d >= r[0] - 1e-6 && d <= r[1] + 1e-6; };
CH4.inLim = (q, lim = GP8().lim) => q.every((x, i) => [0, 2 * PI, -2 * PI].some(k => in1(x + k, lim[i])));
/* the joints that are outside their range (indices), for the pages to name */
CH4.outOf = (q, lim = GP8().lim) => q.map((x, i) => i).filter(i => ![0, 2 * PI, -2 * PI].some(k => in1(q[i] + k, lim[i])));
/* the same pose written inside the ranges where it can be: an angle outside its range is moved a full turn if that brings it inside
   (the GP8's θ3 runs from −190° to 70°, so 173.4° is written −186.6°) */
CH4.fitLim = (q, lim = GP8().lim) => q.map((x, i) => { if (in1(x, lim[i])) return x; const k = [2 * PI, -2 * PI].find(k => in1(x + k, lim[i])); return k === undefined ? x : x + k; });

/* ---- the closest solution (Craig 4.2, exercise 4.6): weighted joint-space distance, each difference wrapped into (−180°, 180°];
   w: weights (heavier for the big joints near the base, so the arm prefers to move its wrist) ---- */
CH4.dist = (a, b, w) => sqrt(a.reduce((s, x, i) => s + (w ? w[i] : 1) * wrap(x - b[i]) ** 2, 0));
CH4.nearest = (sols, now, w) => sols.reduce((best, s) => { const d = CH4.dist(s.q || s, now, w); return !best || d < best.d ? { s, d } : best; }, null);

/* ---- the RPR arm of chapter 3 (CH3.EX.rpr): ^0_3T = [c1c3 −c1s3 s1 s1(d2 + L2); s1c3 −s1s3 −c1 −c1(d2 + L2); s3 c3 0 0].
   Position: (p_x, p_y) = (d2 + L2)(s1, −c1) ⟹ θ1 = atan2(p_x, −p_y), d2 = √(p_x² + p_y²) − L2 (the other root, θ1 + 180° with a negative
   d2 + L2, is outside the slide's range); orientation: (r31, r32) = (s3, c3) ⟹ θ3 = atan2(r31, r32) ---- */
CH4.rprIK = (L2, T) => { const px = T[0][3], py = T[1][3], r = hypot(px, py); return { ok: true, sols: [[atan2(px, -py), r - L2, atan2(T[2][0], T[2][1])]] }; };

/* ---- the RRR arm of chapter 3 (CH3.EX.arm4, Craig exercise 3.4; old chapter 4 part 2 slides 3–16): the geometric solution.
   ^0_4T position: p_x = c1 (L3 c2 + L4 c23), p_y = s1 (…), p_z = L1 + L2 + L3 s2 + L4 s23.
   Top view: θ1 = atan2(p_y, p_x) or that + 180°, r = c1 p_x + s1 p_y (= ±√(p_x² + p_y²)). Side view: ẑ = p_z − (L1 + L2),
   c3 = (r² + ẑ² − L3² − L4²) / (2 L3 L4), θ3 = atan2(±√(1 − c3²), c3), θ2 = atan2(ẑ, r) − atan2(L4 s3, L3 + L4 c3). Up to 4 solutions ---- */
CH4.arm4IK = (L, p) => { const out = [], base = atan2(p[1], p[0]);
  [['front', base], ['back', base + PI]].forEach(([sh, t1]) => { const r = cos(t1) * p[0] + sin(t1) * p[1], z = p[2] - (L.L1 + L.L2), c3 = (r * r + z * z - L.L3 * L.L3 - L.L4 * L.L4) / (2 * L.L3 * L.L4);
    if (abs(c3) > 1 + 1e-12) return; const s = sqrt(Math.max(0, 1 - c3 * c3));
    [s, -s].forEach((s3, k) => { if (k && s < 1e-12) return; const t3 = atan2(s3, c3), t2 = atan2(z, r) - atan2(L.L4 * s3, L.L3 + L.L4 * c3); const S = L.L1 + L.L2, Ez = S + L.L3 * sin(t2), Er = L.L3 * cos(t2), t = (Er * r + (Ez - S) * z) / (r * r + z * z || 1);   // elbow above the line shoulder → goal (in the arm's plane: r out, z up)?
      out.push({ q: [wrap(t1), wrap(t2), t3], shoulder: sh, elbow: Ez > S + t * z + 1e-12 ? 'up' : 'down', r, z, c3 }); }); });
  return out; };
CH4.arm4P = (L, q) => { const [t1, t2, t3] = q, k = L.L3 * cos(t2) + L.L4 * cos(t2 + t3); return [cos(t1) * k, sin(t1) * k, L.L1 + L.L2 + L.L3 * sin(t2) + L.L4 * sin(t2 + t3)]; };

/* ---- the SCARA of chapter 3 (CH3.EX.scara, Craig's table with α2 = 180°): tip = (L1 c1 + L2 c12, L1 s1 + L2 s12, −d3 − d4) in {0},
   the tool's X axis turned φ = θ1 + θ2 − θ4 about Z0 (Z4 points down). θ1, θ2: the two-link arm; d3 = −z − d4; θ4 = θ1 + θ2 − φ. Two solutions ---- */
CH4.scaraIK = (L, x, y, z, phi) => { const S = CH4.p3r(L.L1, L.L2, x, y, 0); if (!S.ok) return []; return S.sols.map(([t1, t2]) => [t1, t2, -z - L.d4, wrap(t1 + t2 - phi)]); };
CH4.scaraPose = (L, q) => { const T = RB.fkDH(CH3.EX.scara.rows(L), q).pop(); return { p: RB.Tp(T), phi: atan2(T[1][0], T[0][0]), T }; };

/* ---- homework 4 (old part 1, the teacher's homework slide): an RRP arm. A column L1 (θ1 about the vertical), a horizontal link L2 to the
   elbow θ2 (horizontal axis), then a slide d3 along the forearm. Craig's table: (α, a, d, θ) = (0, 0, L1, θ1), (90°, L2, 0, θ2 − 90°), (−90°, 0, d3, 0).
   x = c1 (L2 + d3 c2), y = s1 (L2 + d3 c2), z = L1 + d3 s2. Inverse: θ1 = atan2(y, x) or + 180°, ρ = c1 x + s1 y, θ2 = atan2(z − L1, ρ − L2),
   d3 = √((ρ − L2)² + (z − L1)²) ---- */
const row = (al, a, d, th, j) => ({ al, a, d, th, j }), H = PI / 2;
CH4.rrp = { L: { L1: 0.5, L2: 0.3 }, q: [rad(35), rad(25), 0.4],
  rows: L => [row(0, 0, L.L1, 0, 'R'), row(H, L.L2, 0, -H, 'R'), row(-H, 0, 0, 0, 'P')],
  sym: [['0', '0', 'L₁', 'θ₁'], ['90°', 'L₂', '0', 'θ₂ − 90°'], ['−90°', '0', 'd₃', '0']],
  P: (L, q) => { const [t1, t2, d3] = q, k = L.L2 + d3 * cos(t2); return [cos(t1) * k, sin(t1) * k, L.L1 + d3 * sin(t2)]; },
  T03: (L, q) => RB.fkDH(CH4.rrp.rows(L), q).pop(),
  IK: (L, p) => { const base = atan2(p[1], p[0]); return [base, base + PI].map(t1 => { const rho = cos(t1) * p[0] + sin(t1) * p[1], u = rho - L.L2, w = p[2] - L.L1; return [wrap(t1), atan2(w, u), hypot(u, w)]; }); } };

/* ---- the numbers of the worked examples (slides and pages show these; tests.js checks them) ---- */
CH4.EX = (() => { const P = CH3.EX.p3r, A4 = CH3.EX.arm4, SC = CH3.EX.scara, I = CH3.EX.gp8, d = RB.deg;
  const g3 = CH4.p3rFK(P.L.L1, P.L.L2, P.q), p4 = CH4.arm4P(A4.L, A4.q), sp = CH4.scaraPose(SC.L, SC.q), Ti = CH4.gp8FK(I.q);
  /* SOLVE (Craig 4.9): a tool 0.12 m long on the flange (^6_TT = D_Z(0.12)), goal {G} 0.45 m out, 0.15 m to the side, 0.10 m above {0}, pointing down */
  const LT = 0.12, G = [[1, 0, 0, 0.45], [0, -1, 0, 0.15], [0, 0, -1, 0.10], [0, 0, 0, 1]], T6T = RB.trans(0, 0, LT), W = RB.Tm(G, RB.Ti(T6T)), SW = CH4.gp8IK(W);
  return {
    p3r: { L: P.L, q: P.q, goal: g3, sol: CH4.p3r(P.L.L1, P.L.L2, g3[0], g3[1], g3[2]) },
    half: { a: 3, b: 4, c: 2, sol: CH4.half(3, 4, 2) },
    arm4: { L: A4.L, q: A4.q, p: p4, sols: CH4.arm4IK(A4.L, p4) },
    scara: { L: SC.L, q: SC.q, p: sp.p, phi: sp.phi, sols: CH4.scaraIK(SC.L, sp.p[0], sp.p[1], sp.p[2], sp.phi) },
    gp8: { q: I.q, T: Ti, sols: CH4.gp8IK(Ti).map(s => ({ ...s, ok: CH4.inLim(s.q) })) },
    solve: { LT, G, W, sols: SW.map(s => ({ ...s, ok: CH4.inLim(s.q) })), best: CH4.nearest(SW.filter(s => CH4.inLim(s.q)), I.home) },   // the present pose is the GP8's home position (every pendant angle 0)
    hw: { L: CH4.rrp.L, q: CH4.rrp.q, p: CH4.rrp.P(CH4.rrp.L, CH4.rrp.q), sols: CH4.rrp.IK(CH4.rrp.L, CH4.rrp.P(CH4.rrp.L, CH4.rrp.q)) } }; })();

global.CH4 = CH4;
})(typeof window !== 'undefined' ? window : globalThis);
