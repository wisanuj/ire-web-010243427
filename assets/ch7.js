/* ch7.js — chapter 7, trajectory generation (Craig, Introduction to Robotics, 3rd ed., chapter 7: 7.1–7.5; the old slides
   "Chapter 7 Trajectory generation", parts I and II).
   Joint-space schemes: cubic polynomials (rest to rest, with given end velocities, through via points with a heuristic or with continuous
   acceleration), the quintic, linear functions with parabolic blends (one segment, and through via points: Craig 7.3, eq. 7.19–7.28).
   Cartesian-space schemes for the two-link arm: a straight line followed by inverse kinematics at every instant, and its geometric problems (Craig 7.4–7.5).
   Every function here is checked in tests.js (boundary conditions, continuity, the worked numbers).
   Works in the browser (window.CH7) and in deno (globalThis.CH7). Needs robot.js and ch5.js. The plot helpers at the end need a canvas. */
(function (global) {
'use strict';
const RB = global.RB, CH5 = global.CH5, { sin, cos, sqrt, abs, max, min, hypot, atan2, PI, sign } = Math, rad = RB.rad, deg = RB.deg;
const CH7 = {};

/* ---- polynomials: a = [a0, a1, …]; returns [θ, θ̇, θ̈] at time t ---- */
CH7.poly = (a, t) => { let p = 0, v = 0, q = 0; for (let i = a.length - 1; i >= 0; i--) p = p * t + a[i]; for (let i = a.length - 1; i >= 1; i--) v = v * t + i * a[i]; for (let i = a.length - 1; i >= 2; i--) q = q * t + i * (i - 1) * a[i]; return [p, v, q]; };
/* the cubic from θ0 to θf in tf seconds with end velocities v0, vf (Craig 7.3 rest to rest, 7.11 in general) */
CH7.cubic = (th0, thf, tf, v0 = 0, vf = 0) => { const d = thf - th0; return [th0, v0, 3 * d / (tf * tf) - 2 * v0 / tf - vf / tf, -2 * d / (tf * tf * tf) + (vf + v0) / (tf * tf)]; };
/* the quintic with end positions, velocities and accelerations (Craig 7.16–7.18) */
CH7.quintic = (th0, thf, tf, v0 = 0, vf = 0, q0 = 0, qf = 0) => { const t = tf, t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
  return [th0, v0, q0 / 2, (20 * thf - 20 * th0 - (8 * vf + 12 * v0) * t - (3 * q0 - qf) * t2) / (2 * t3), (30 * th0 - 30 * thf + (14 * vf + 16 * v0) * t + (3 * q0 - 2 * qf) * t2) / (2 * t4), (12 * thf - 12 * th0 - (6 * vf + 6 * v0) * t - (q0 - qf) * t2) / (2 * t5)]; };
/* a single polynomial as a trajectory: { T, at(t) } (held at the ends outside 0…T) */
CH7.polyTraj = (a, T) => ({ T, kind: 'poly', a, at: t => { if (t <= 0) return [a[0], 0, 0]; if (t >= T) { const e = CH7.poly(a, T); return [e[0], 0, 0]; } return CH7.poly(a, t); } });

/* ---- via points. pts: the path points (θ at each), durs: the duration of every segment ---- */
const times = durs => durs.reduce((acc, d) => (acc.push(acc[acc.length - 1] + d), acc), [0]);
CH7.times = times;
/* Craig's simple heuristic for the velocities at the via points: the slopes of the straight lines into and out of a via point; zero where the slope
   changes sign (or one is zero), else their average. Zero at the two ends */
CH7.viaHeuristic = (pts, durs) => pts.map((p, k) => { if (k === 0 || k === pts.length - 1) return 0; const a = (p - pts[k - 1]) / durs[k - 1], b = (pts[k + 1] - p) / durs[k]; return sign(a) !== sign(b) || a === 0 || b === 0 ? 0 : (a + b) / 2; });
/* the velocities at the via points that make the acceleration continuous there (a cubic spline at rest at both ends, Craig 7.3).
   For segments T, T' with rises Δ, Δ' around via point k:  v_{k−1}/T + 2v_k(1/T + 1/T') + v_{k+1}/T' = 3(Δ/T² + Δ'/T'²)   (tridiagonal) */
CH7.viaSpline = (pts, durs) => { const n = pts.length, v = new Array(n).fill(0); if (n < 3) return v; const m = n - 2, A = [], B = [], Cc = [], D = [];
  for (let k = 1; k <= m; k++) { const T = durs[k - 1], U = durs[k], d0 = pts[k] - pts[k - 1], d1 = pts[k + 1] - pts[k]; A.push(1 / T); B.push(2 * (1 / T + 1 / U)); Cc.push(1 / U); D.push(3 * (d0 / (T * T) + d1 / (U * U))); }
  for (let i = 1; i < m; i++) { const w = A[i] / B[i - 1]; B[i] -= w * Cc[i - 1]; D[i] -= w * D[i - 1]; }   // the Thomas algorithm
  const x = new Array(m); x[m - 1] = D[m - 1] / B[m - 1]; for (let i = m - 2; i >= 0; i--) x[i] = (D[i] - Cc[i] * x[i + 1]) / B[i];
  x.forEach((xi, i) => { v[i + 1] = xi; }); return v; };
/* one cubic per segment through the path points with the given velocities: { T, segs: [{ t0, T, a }], at(t) } */
CH7.cubics = (pts, durs, vels) => { const ts = times(durs), segs = durs.map((d, i) => ({ t0: ts[i], T: d, a: CH7.cubic(pts[i], pts[i + 1], d, vels[i], vels[i + 1]) })), Tt = ts[ts.length - 1];
  return { T: Tt, kind: 'cubics', pts, durs, vels, segs, ts, at: t => { if (t <= 0) return [pts[0], 0, 0]; if (t >= Tt) return [pts[pts.length - 1], 0, 0]; let i = segs.findIndex(s => t < s.t0 + s.T); if (i < 0) i = segs.length - 1; return CH7.poly(segs[i].a, t - segs[i].t0); } }; };

/* ---- a linear function with parabolic blends, one segment (Craig 7.3, eq. 7.19–7.23): rest at θ0, rest at θf after tf, blends of acceleration acc.
   The two blends last tb each and are symmetric about the middle; the acceleration must be at least 4|θf − θ0|/tf² (then the straight part shrinks to nothing) ---- */
CH7.lspbMin = (th0, thf, tf) => 4 * abs(thf - th0) / (tf * tf);
CH7.lspb = (th0, thf, tf, acc) => { const d = thf - th0, A = abs(acc), s = d >= 0 ? 1 : -1, amin = CH7.lspbMin(th0, thf, tf), ok = A >= amin - 1e-9;
  const tb = ok ? (A < 1e-12 ? tf / 2 : tf / 2 - sqrt(max(0, A * A * tf * tf - 4 * A * abs(d))) / (2 * A)) : tf / 2, a = s * A, v = a * tb, thb = th0 + a * tb * tb / 2;
  return { ok, amin, tb, a, v, thb, th0, thf, T: tf, kind: 'lspb', tl: tf - 2 * tb,
    at: t => { if (t <= 0) return [th0, 0, 0]; if (t >= tf) return [thf, 0, 0]; if (t < tb) return [th0 + a * t * t / 2, a * t, a]; if (t <= tf - tb) return [thb + v * (t - tb), v, 0]; const r = tf - t; return [thf - a * r * r / 2, a * r, -a]; } }; };

/* ---- a linear function with parabolic blends through via points (Craig 7.3, eq. 7.24–7.28; the worked numbers are Craig example 7.4). pts: path points, durs: segment durations,
   acc: the magnitude of the blend acceleration (a number, or one per path point). The straight lines pass through the via points, but the
   blends round them off: the arm passes near a via point, not through it (only the ends are reached exactly).
   Returns per path point k: acc[k] (signed), tb[k] (blend duration); per segment: v[j] (velocity), tl[j] (duration of the straight part); ts[k] the time
   of path point k; ok false when a blend does not fit (acceleration too small) ---- */
CH7.lspbVia = (pts, durs, accIn) => { const n = pts.length, ts = times(durs), Am = k => abs(Array.isArray(accIn) ? accIn[k] : accIn), acc = new Array(n).fill(0), tb = new Array(n).fill(0), v = new Array(n - 1).fill(0), tl = new Array(n - 1).fill(0); let ok = true;
  if (n === 2) { const L = CH7.lspb(pts[0], pts[1], durs[0], Am(0)); acc[0] = L.a; acc[1] = -L.a; tb[0] = tb[1] = L.tb; v[0] = L.v; tl[0] = L.tl; ok = L.ok; }
  else {
    /* first segment: the blend starts at rest at θ1 at time 0 and lasts the whole t1 inside the segment */
    { const d = pts[1] - pts[0], T = durs[0], s = d >= 0 ? 1 : -1, a = s * Am(0), r = T * T - 2 * d / a; if (r < 0) ok = false; acc[0] = a; tb[0] = T - sqrt(max(0, r)); v[0] = d / (T - tb[0] / 2); }
    /* last segment: the blend ends at rest at θn at the end */
    { const j = n - 2, d = pts[n - 1] - pts[n - 2], T = durs[j], s = pts[n - 2] - pts[n - 1] >= 0 ? 1 : -1, a = s * Am(n - 1), r = T * T + 2 * d / a; if (r < 0) ok = false; acc[n - 1] = a; tb[n - 1] = T - sqrt(max(0, r)); v[j] = d / (T - tb[n - 1] / 2); }
    /* the segments in between: the slope between the two via points */
    for (let j = 1; j < n - 2; j++) v[j] = (pts[j + 1] - pts[j]) / durs[j];
    /* the blends at the via points: from one slope to the next */
    for (let k = 1; k < n - 1; k++) { const dv = v[k] - v[k - 1], s = dv >= 0 ? 1 : -1; acc[k] = s * Am(k); tb[k] = abs(dv) < 1e-12 ? 0 : dv / acc[k]; }
    /* the straight parts: what is left of each segment */
    for (let j = 0; j < n - 1; j++) { const first = j === 0, last = j === n - 2; tl[j] = durs[j] - (first ? tb[0] : tb[j] / 2) - (last ? tb[n - 1] : tb[j + 1] / 2); if (tl[j] < -1e-9) ok = false; } }
  /* the straight line of segment j: slope v[j] through a point it passes: (t of path point j + 1, θ) for every segment but the last, (t of point n − 1, θ) for the last */
  const line = (j, t) => { const last = j === n - 2 && n > 2, k = last ? j : j + 1; return pts[k] + v[j] * (t - ts[k]); }, Tt = ts[n - 1];
  const at = t => { if (t <= 0) return [pts[0], 0, 0]; if (t >= Tt) return [pts[n - 1], 0, 0];
    if (n === 2) return CH7.lspb(pts[0], pts[1], durs[0], Am(0)).at(t);
    if (t < tb[0]) return [pts[0] + acc[0] * t * t / 2, acc[0] * t, acc[0]];
    if (t > Tt - tb[n - 1]) { const r = Tt - t; return [pts[n - 1] + acc[n - 1] * r * r / 2, -acc[n - 1] * r, acc[n - 1]]; }
    for (let k = 1; k < n - 1; k++) { const b0 = ts[k] - tb[k] / 2, b1 = ts[k] + tb[k] / 2; if (t >= b0 && t <= b1) { const u = t - b0; return [line(k - 1, t) + acc[k] * u * u / 2, v[k - 1] + acc[k] * u, acc[k]]; } }
    let j = 0; while (j < n - 2 && t > ts[j + 1]) j++; return [line(j, t), v[j], 0]; };
  /* where the straight parts and the blends begin and end, for drawing */
  const blends = [[0, tb[0]], ...pts.slice(1, -1).map((_, i) => [ts[i + 1] - tb[i + 1] / 2, ts[i + 1] + tb[i + 1] / 2]), [Tt - tb[n - 1], Tt]];
  return { ok, pts, durs, ts, acc, tb, v, tl, T: Tt, kind: 'lspbVia', blends, at }; };

/* ---- several joints together (Craig 7.2): every joint gets its own function, all with the same durations, so that they reach each path point together.
   make(i) builds joint i's trajectory; returns { T, at(t) → [q, qd, qdd] } with one entry per joint ---- */
CH7.multi = trajs => ({ T: max(...trajs.map(x => x.T)), trajs, at: t => { const r = trajs.map(x => x.at(t)); return [r.map(e => e[0]), r.map(e => e[1]), r.map(e => e[2])]; } });

/* ---- Cartesian-space paths for the two-link arm (Craig 7.4–7.5) ----
   joint space: θ(t) = θA + s(t)(θB − θA): every joint moves smoothly, the tool point follows a curve.
   Cartesian space: p(t) = A + s(t)(B − A): the tool point follows the straight line, the joints come from inverse kinematics at every instant.
   s(t) is the rest-to-rest cubic from 0 to 1 in T seconds. elbow: +1 or −1 (RB.ik2). Returns samples { t, s, p, q, ok } and, along the line, the joint rates */
CH7.sOf = (t, T) => { const a = CH7.cubic(0, 1, T), u = min(T, max(0, t)), e = CH7.poly(a, u); return t <= 0 ? [0, 0, 0] : t >= T ? [1, 0, 0] : e; };
CH7.jointMove2 = (L, qA, qB, T, n = 60) => Array.from({ length: n + 1 }, (_, i) => { const t = T * i / n, s = CH7.sOf(t, T)[0], q = qA.map((x, j) => x + s * (qB[j] - x)), f = RB.fk2(L[0], L[1], q[0], q[1]); return { t, s, q, p: [f.x, f.y], ok: true }; });
CH7.lineMove2 = (L, A, B, T, elbow = 1, n = 60) => Array.from({ length: n + 1 }, (_, i) => { const t = T * i / n, [s, sd] = CH7.sOf(t, T), p = [A[0] + s * (B[0] - A[0]), A[1] + s * (B[1] - A[1])], r = RB.ik2(L[0], L[1], p[0], p[1], elbow);
  const vel = [sd * (B[0] - A[0]), sd * (B[1] - A[1])], qd = r.ok ? CH5.two.rates(L, [r.t1, r.t2], vel) : null; return { t, s, p, q: r.ok ? [r.t1, r.t2] : null, ok: r.ok, qd }; });
/* the closest the line A–B comes to the base, and where (the inner and outer edges of the workspace are the circles |l1 − l2| and l1 + l2) */
CH7.lineGap = (A, B) => { const d = [B[0] - A[0], B[1] - A[1]], L2 = d[0] * d[0] + d[1] * d[1], u = L2 < 1e-12 ? 0 : max(0, min(1, -(A[0] * d[0] + A[1] * d[1]) / L2)); return { u, r: hypot(A[0] + u * d[0], A[1] + u * d[1]) }; };
/* the largest joint rate (rad/s) along a straight-line move */
CH7.peakRates = samples => samples.reduce((m, x) => x.qd ? [max(m[0], abs(x.qd[0])), max(m[1], abs(x.qd[1]))] : m, [0, 0]);

/* ---- the numbers of the worked examples (slides and pages show these; tests.js checks them). Angles of the joint-space examples in degrees ---- */
CH7.EX = (() => { const L = CH5.two.L;
  const c1 = CH7.cubic(15, 75, 3), q1 = CH7.quintic(15, 75, 3);
  /* a cubic that starts moving: the joint leaves 15° at 10°/s and arrives at 75° at rest (Craig 7.11 with a non-zero start velocity) */
  const cv = CH7.cubic(15, 75, 3, 10, 0);
  /* via points: Craig's exercise path 10°, 35°, 25°, 10° in 2, 1, 3 s */
  const vp = [10, 35, 25, 10], vd = [2, 1, 3], vh = CH7.viaHeuristic(vp, vd), vs = CH7.viaSpline(vp, vd);
  /* one segment with parabolic blends: the motion of Craig example 7.1 with blend accelerations 40 and 80 °/s² */
  const b40 = CH7.lspb(15, 75, 3, 40), b80 = CH7.lspb(15, 75, 3, 80);
  /* through via points (old part II example = Craig example 7.4; Craig prints θ̇12 = 13.50, a slip: 25/(2 − 0.134) = 13.40): the same path, 50 °/s² at every blend; homework 7: 5°, 15°, −10° in 2 + 2 s with 60 °/s² */
  const lv = CH7.lspbVia(vp, vd, 50), hw = CH7.lspbVia([5, 15, -10], [2, 2], 60);
  /* two joints together: the two-link arm from (−20°, 70°) to (100°, 40°) in 2 s, each joint a rest-to-rest cubic (both poses with θ2 > 0, RB.ik2 elbow +1) */
  const qA = [rad(-20), rad(70)], qB = [rad(100), rad(40)], T2 = 2;
  /* Cartesian: the straight line between the same two tool points; then, for an arm with a short second link (0.5 m and 0.25 m: the hole in the middle of its
     workspace, radius |l1 − l2| = 0.25 m, shows clearly), three horizontal lines from x = 0.5 m to x = −0.4 m: one through the hole, one just outside it
     (near the singular pose θ2 = 180°) and one far from it */
  const fA = RB.fk2(L[0], L[1], qA[0], qA[1]), fB = RB.fk2(L[0], L[1], qB[0], qB[1]), A = [fA.x, fA.y], B = [fB.x, fB.y], Ls = [0.5, 0.25], hl = y => ({ L: Ls, A: [0.5, y], B: [-0.4, y] });
  const hole = hl(0.15), near = hl(0.26), far = hl(0.5);
  return {
    cubic: { th0: 15, thf: 75, tf: 3, a: c1, vmax: CH7.poly(c1, 1.5)[1], acc0: CH7.poly(c1, 0)[2], accf: CH7.poly(c1, 3)[2] },
    quintic: { a: q1, vmax: CH7.poly(q1, 1.5)[1], accMax: max(...Array.from({ length: 301 }, (_, i) => abs(CH7.poly(q1, 3 * i / 300)[2]))) },
    cubicV: { v0: 10, a: cv },
    via: { pts: vp, durs: vd, heur: vh, spline: vs, cH: CH7.cubics(vp, vd, vh), cS: CH7.cubics(vp, vd, vs) },
    lspb: { a40: b40, a80: b80, amin: CH7.lspbMin(15, 75, 3) },
    lspbVia: lv, hw,
    two: { L, qA, qB, T: T2, A, B, joint: CH7.jointMove2(L, qA, qB, T2), line: CH7.lineMove2(L, A, B, T2, 1) },
    hole: { ...hole, gap: CH7.lineGap(hole.A, hole.B), line: CH7.lineMove2(Ls, hole.A, hole.B, 2, 1, 400) },
    near: { ...near, gap: CH7.lineGap(near.A, near.B), line: CH7.lineMove2(Ls, near.A, near.B, 2, 1, 800) },
    far: { ...far, gap: CH7.lineGap(far.A, far.B), line: CH7.lineMove2(Ls, far.A, far.B, 2, 1, 400) } }; })();

/* ======================= plots (browser and figures): position, velocity and acceleration against time ======================= */
const nice = (lo, hi, n = 4) => { if (hi - lo < 1e-9) { lo -= 1; hi += 1; } const raw = (hi - lo) / n, m = Math.pow(10, Math.floor(Math.log10(raw))), f = raw / m, st = (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * m; return { lo: Math.floor(lo / st - 1e-9) * st, hi: Math.ceil(hi / st + 1e-9) * st, st }; };
const fmt = x => { const s = Math.abs(x) < 1e-9 ? '0' : (Math.abs(x) >= 100 || Number.isInteger(+x.toFixed(6)) ? x.toFixed(0) : Math.abs(x) >= 10 ? x.toFixed(0) : x.toFixed(1)); return s.replace('-', '−'); };
/* draw stacked panels. o: { T, curves: [{ f: t → [θ, θ̇, θ̈], col (palette key or css colour), w (line width), dash, label }], rows (any of 'p', 'v', 'a'),
   now (a time to mark), marks: [{ t, y }] dots on the position panel, blends: [[t0, t1]…] shaded, units: ['°', '°/s', '°/s²'], names (row titles), t0 } */
CH7.plots = (c, W, H, P, o) => { const G = RB.G, rows = o.rows || ['p', 'v', 'a'], nR = rows.length, T = o.T, t0 = o.t0 || 0, ml = 52, mr = 14, mt = 8, mb = 30, gap = 14, ph = (H - mt - mb - gap * (nR - 1)) / nR, pw = W - ml - mr;
  const idx = { p: 0, v: 1, a: 2 }, U = o.units || ['°', '°/s', '°/s²'], N = o.names || [['θ', ''], ['θ̇', ''], ['θ̈', '']], col = k => P[k] || k;
  const xs = t => ml + (t - t0) / (T - t0) * pw;
  rows.forEach((r, ri) => { const j = idx[r], y0 = mt + ri * (ph + gap); let lo = Infinity, hi = -Infinity; const S = 240;
    o.curves.forEach(cu => { for (let i = 0; i <= S; i++) { const t = t0 + (T - t0) * i / S, y = cu.f(t)[j]; if (isFinite(y)) { lo = min(lo, y); hi = max(hi, y); } } });
    if (r === 'p' && o.marks) o.marks.forEach(m => { lo = min(lo, m.y); hi = max(hi, m.y); });
    if (o.yr && o.yr[r]) { lo = min(lo, o.yr[r][0]); hi = max(hi, o.yr[r][1]); }
    const sc = nice(lo, hi, ph > 90 ? 4 : 3), ys = y => y0 + ph - (y - sc.lo) / (sc.hi - sc.lo) * ph;
    c.save(); c.fillStyle = P.bg; c.fillRect(ml, y0, pw, ph); c.restore();
    (o.blends || []).forEach(b => { const a = xs(max(t0, b[0])), e = xs(min(T, b[1])); if (e > a) { c.save(); c.fillStyle = RB.rgba(P.warn, 0.13); c.fillRect(a, y0, e - a, ph); c.restore(); } });
    for (let y = sc.lo; y <= sc.hi + sc.st * 1e-6; y += sc.st) { const Y = ys(y); G.line(c, ml, Y, ml + pw, Y, Math.abs(y) < 1e-9 ? P.faint : P.grid, Math.abs(y) < 1e-9 ? 1.4 : 1); G.text(c, fmt(y), ml - 6, Y + 4, { size: 11, align: 'right', color: P.muted, mono: true }); }
    const tst = nice(t0, T, max(3, Math.round(pw / 90))).st; for (let t = Math.ceil(t0 / tst - 1e-9) * tst; t <= T + 1e-9; t += tst) { const X = xs(t); G.line(c, X, y0, X, y0 + ph, P.grid, 1); if (ri === nR - 1) G.text(c, fmt(t), X, y0 + ph + 15, { size: 11, align: 'center', color: P.muted, mono: true }); }
    G.line(c, ml, y0 + ph, ml + pw, y0 + ph, P.faint, 1.2); G.line(c, ml, y0, ml, y0 + ph, P.faint, 1.2);
    o.curves.forEach(cu => { const pts = []; for (let i = 0; i <= S; i++) { const t = t0 + (T - t0) * i / S, y = cu.f(t)[j]; pts.push([xs(t), ys(y)]); } G.poly(c, pts, { stroke: col(cu.col || 'l2'), lw: cu.w || 2.2, dash: cu.dash }); });
    if (r === 'p' && o.marks) o.marks.forEach(m => G.circle(c, xs(m.t), ys(m.y), 4, { fill: P.bg, stroke: P.ink, lw: 1.8 }));
    if (o.now !== undefined && o.now >= t0 && o.now <= T) { const X = xs(o.now); G.line(c, X, y0, X, y0 + ph, P.sel, 1.4, [4, 3]); o.curves.slice(0, 1).forEach(cu => G.circle(c, X, ys(cu.f(o.now)[j]), 4.2, { fill: col(cu.col || 'l2') })); }
    /* the row title: the symbol with its dots drawn as small discs (a combining dot sits badly on an italic θ in canvas text), then the unit */
    const nm = N[j], dec = nm[0].normalize('NFD'), nd = (dec.match(/\u0307/g) || []).length + 2 * (dec.match(/\u0308/g) || []).length, w = G.sym(c, dec.replace(/[\u0307\u0308]/g, ''), nm[1], ml + 8, y0 + 19, { size: 16, color: P.ink, halo: P.bg });
    for (let d = 0; d < nd; d++) G.circle(c, ml + 8 + (nm[1] ? 9 : w) * 0.5 + (d - (nd - 1) / 2) * 4.4, y0 + 4, 1.5, { fill: P.ink });
    G.text(c, '(' + U[j] + ')', ml + 8 + w + 6, y0 + 18, { size: 11.5, color: P.muted, halo: P.bg }); });
  if (o.tlabel !== false) G.text(c, o.tlabel || 't (s)', ml + pw, H - 4, { size: 11.5, align: 'right', color: P.muted }); };
/* a canvas that shows the plots of state S: S.plot() returns the options of CH7.plots; o.tick(dt) runs first (to play the motion) */
CH7.plotSim = (cv, S, o = {}) => { if (typeof cv === 'string') cv = document.getElementById(cv);
  return RB.stage(cv, o.aspect || 0.62, (c, W, H, dt, P) => { if (o.tick) o.tick(dt); RB.G.clear(c, W, H, P); CH7.plots(c, W, H, P, S.plot()); }, { maxH: o.maxH || 420 }); };

global.CH7 = CH7;
})(typeof window !== 'undefined' ? window : globalThis);
