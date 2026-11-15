/* robot.js — numbers, canvas helpers, kinematics and drawing primitives shared by every page of the Industrial Robotics Engineering site.
   Loads in the browser (window.RB) and in deno (globalThis.RB), so tests.html, the pages and the slide build use the same math.
   Conventions: world coordinates in metres with y (or z) up, angles in radians, counter-clockwise positive; joint numbering follows Craig
   (link 0 is the fixed base, joint i connects link i-1 to link i); axis colours are fixed for the whole course: X red, Y green, Z blue. */
(function (global) {
'use strict';
const RB = {}, NUM = global.NUM || {};
NUM.fmt = NUM.fmt || ((x, d = 3) => { if (x === null || x === undefined || Number.isNaN(x)) return '—'; if (!isFinite(x)) return x > 0 ? '∞' : '−∞';
  const s = Math.abs(x) < 1e-12 ? (0).toFixed(d) : (Math.abs(x) >= 1e5 || Math.abs(x) < 1e-3 ? x.toExponential(d - 1) : x.toFixed(d)); return s.replace('-', '−'); });
NUM.clamp = NUM.clamp || ((x, lo, hi) => Math.max(lo, Math.min(hi, x)));
global.NUM = NUM;
const { sin, cos, atan2, sqrt, abs, PI, min, max, hypot } = Math;
const rad = d => d * PI / 180, deg = r => r * 180 / PI;
RB.rad = rad; RB.deg = deg;
RB.wrap = a => atan2(sin(a), cos(a));                       // into (−π, π]
RB.d = (x, n = 3) => { let s = (+x).toFixed(n); if (s.includes('.')) s = s.replace(/0+$/, '').replace(/\.$/, ''); return s === '-0' ? '0' : s; };   // plain decimal, ASCII minus (TeX)
RB.u = (x, n = 3) => RB.d(x, n).replace('-', '−');          // display text, Unicode minus

/* ---------------- palettes: dark for the on-screen simulations, light for figures exported to the slides ---------------- */
RB.DARK = { bg: '#0f1220', ink: '#e8ecf7', muted: '#9aa3c7', grid: '#1e2440', faint: '#2a3156', l1: '#ffb454', l2: '#5ad1ff', l3: '#c792ea', base: '#8a93b8', joint: '#0f1220',
  good: '#7ee787', bad: '#ff6b6b', warn: '#ffd166', sel: '#ff7eb6', x: '#ff6b6b', y: '#7ee787', z: '#6aa8ff', ws: '#5ad1ff', wsA: 0.16, dex: '#7ee787', dexA: 0.30, ghost: 0.35 };
RB.LIGHT = { bg: '#ffffff', ink: '#1b2a41', muted: '#5b6b82', grid: '#e6ebf2', faint: '#cfd8e3', l1: '#e8590c', l2: '#1f6fb2', l3: '#7b3fb5', base: '#5b6b82', joint: '#ffffff',
  good: '#1d8a4a', bad: '#c0392b', warn: '#b7791f', sel: '#c2356f', x: '#d62828', y: '#2b9348', z: '#1d4ed8', ws: '#1f6fb2', wsA: 0.14, dex: '#1d8a4a', dexA: 0.28, ghost: 0.4 };
RB.FONT = '"IBM Plex Sans Thai", system-ui, -apple-system, "Segoe UI", sans-serif';
RB.MONO = '"IBM Plex Mono", ui-monospace, Menlo, monospace';
RB.SERIF = '"Times New Roman", Times, serif';
const rgba = (hex, a) => { const n = parseInt(hex.slice(1), 16); return `rgba(${n >> 16},${(n >> 8) & 255},${n & 255},${a})`; };
RB.rgba = rgba;

/* ---------------- planar kinematics ---------------- */
/* two-link planar arm (2R): joint 1 at the origin, link lengths l1, l2 */
RB.fk2 = (l1, l2, t1, t2) => { const x1 = l1 * cos(t1), y1 = l1 * sin(t1); return { x1, y1, x: x1 + l2 * cos(t1 + t2), y: y1 + l2 * sin(t1 + t2) }; };
/* inverse kinematics of the 2R arm (the formulas of problem 1.3): D = cos θ2; elbow = +1 or −1 picks the sign of sin θ2 */
RB.ik2 = (l1, l2, x, y, elbow = 1) => { const D = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2); if (abs(D) > 1 + 1e-12) return { ok: false, D };
  const Dc = NUM.clamp(D, -1, 1), t2 = atan2(elbow * sqrt(1 - Dc * Dc), Dc), t1 = RB.wrap(atan2(y, x) - atan2(l2 * sin(t2), l1 + l2 * cos(t2))); return { ok: true, D, t1, t2 }; };
/* planar serial chain with revolute (R) and prismatic (P) joints. spec = [{type:'R', a: link length} | {type:'P', a: fixed part}], q = joint variables.
   A prismatic joint slides along the direction of the previous link. Returns the joint positions, the tip and the heading. */
RB.chain = (spec, q) => { let x = 0, y = 0, th = 0; const pts = [[0, 0]], ang = [];
  spec.forEach((j, i) => { if (j.type === 'R') { th += q[i]; ang.push(th); x += j.a * cos(th); y += j.a * sin(th); } else { ang.push(th); x += (j.a + q[i]) * cos(th); y += (j.a + q[i]) * sin(th); } pts.push([x, y]); });
  return { pts, ang, tip: [x, y], th }; };
/* is (x, y) in the workspace of the 2R arm with joint limits lim = {a1, b1, a2, b2} (radians)? */
const inRange = (t, a, b) => { if (b - a >= 2 * PI - 1e-9) return true; let u = t; while (u < a - 1e-9) u += 2 * PI; while (u > b + 1e-9) u -= 2 * PI; return u >= a - 1e-9; };
RB.inRange = inRange;
RB.ws2Has = (l1, l2, lim, x, y) => [1, -1].some(e => { const s = RB.ik2(l1, l2, x, y, e); return s.ok && inRange(s.t1, lim.a1, lim.b1) && inRange(s.t2, lim.a2, lim.b2); });
/* area of that workspace by counting cells of an n × n grid over the square |x|, |y| ≤ l1 + l2 */
RB.ws2Area = (l1, l2, lim, n = 240) => { const R = l1 + l2, h = 2 * R / n; let c = 0; for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) if (RB.ws2Has(l1, l2, lim, -R + (i + .5) * h, -R + (j + .5) * h)) c++; return c * h * h; };
/* radial limits of the 2R workspace when both joints turn freely, and of the 3R planar arm (reachable and dexterous rings) */
RB.ring2 = (l1, l2) => ({ rmin: abs(l1 - l2), rmax: l1 + l2, area: PI * ((l1 + l2) ** 2 - (l1 - l2) ** 2) });
RB.ring3 = (l1, l2, l3) => { const wi = abs(l1 - l2), wo = l1 + l2; return { wrist: [wi, wo], reach: [max(0, wi - l3), wo + l3], dex: wi + l3 <= wo - l3 ? [wi + l3, wo - l3] : null }; };
/* which tool directions φ (world angle of link 3) can the 3R arm hold with its tip at (x, y)? Returns a function φ → bool */
RB.dexOk = (l1, l2, l3, x, y) => phi => { const wx = x - l3 * cos(phi), wy = y - l3 * sin(phi), r = hypot(wx, wy); return r >= abs(l1 - l2) - 1e-9 && r <= l1 + l2 + 1e-9; };
/* resolution problems: chord between the two ends of a swing θ of a link of length l, and the smallest step with n bits */
RB.chord = (l, th) => l * sqrt(2 * (1 - cos(th)));
RB.resolution = (travel, bits) => travel / 2 ** bits;
/* planar five-bar parallel arm: motors at (∓c, 0), proximal links La at angles t1, t2, distal links Lb meeting at the tool point (upper branch) */
RB.fivebar = (c, La, Lb, t1, t2) => { const A = [-c + La * cos(t1), La * sin(t1)], B = [c + La * cos(t2), La * sin(t2)], dx = B[0] - A[0], dy = B[1] - A[1], d = hypot(dx, dy);
  if (d > 2 * Lb || d < 1e-9) return { ok: false, A, B }; const h = sqrt(Lb * Lb - d * d / 4), mx = (A[0] + B[0]) / 2, my = (A[1] + B[1]) / 2; return { ok: true, A, B, P: [mx - h * dy / d, my + h * dx / d] }; };
/* Grübler's formula for a planar mechanism: n links (the ground included), j one-degree-of-freedom joints */
RB.gruebler = (n, j) => 3 * (n - 1) - 2 * j;

/* ---------------- 3-D helpers ---------------- */
RB.Rx = a => [[1, 0, 0], [0, cos(a), -sin(a)], [0, sin(a), cos(a)]];
RB.Ry = a => [[cos(a), 0, sin(a)], [0, 1, 0], [-sin(a), 0, cos(a)]];
RB.Rz = a => [[cos(a), -sin(a), 0], [sin(a), cos(a), 0], [0, 0, 1]];
RB.mm = (A, B) => A.map(r => [0, 1, 2].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
RB.mv = (A, v) => A.map(r => r[0] * v[0] + r[1] * v[1] + r[2] * v[2]);
RB.add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
RB.sc = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
RB.cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
RB.unit = a => { const n = hypot(a[0], a[1], a[2]) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };
/* ---------------- frames and homogeneous transforms (Craig ch. 2) ----------------
   R: 3 × 3 rotation matrix as an array of rows; its columns are the unit axes of the moved frame. p: [x, y, z].
   T: 4 × 4 homogeneous transform as an array of rows, last row [0, 0, 0, 1]. Angles in radians. */
RB.I3 = () => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
RB.tr = A => [0, 1, 2].map(i => [A[0][i], A[1][i], A[2][i]]);
RB.col = (A, j) => [A[0][j], A[1][j], A[2][j]];
RB.dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
RB.det3 = A => A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
/* how far a matrix is from a rotation: largest entry of RᵀR − I, and the determinant (+1 for a rotation, −1 for a reflection) */
RB.rotCheck = A => { const P = RB.mm(RB.tr(A), A); let e = 0; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) e = max(e, abs(P[i][j] - (i === j ? 1 : 0))); return { orth: e, det: RB.det3(A) }; };
RB.T = (R, p = [0, 0, 0]) => [[R[0][0], R[0][1], R[0][2], p[0]], [R[1][0], R[1][1], R[1][2], p[1]], [R[2][0], R[2][1], R[2][2], p[2]], [0, 0, 0, 1]];
RB.TR = T => [T[0].slice(0, 3), T[1].slice(0, 3), T[2].slice(0, 3)];
RB.Tp = T => [T[0][3], T[1][3], T[2][3]];
RB.Tm = (...Ts) => Ts.reduce((A, B) => A.map(r => [0, 1, 2, 3].map(j => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j] + r[3] * B[3][j])));
/* inverse of a homogeneous transform without a general matrix inverse (Craig 2.45): [Rᵀ, −Rᵀp; 0 0 0 1] */
RB.Ti = T => { const Rt = RB.tr(RB.TR(T)), q = RB.mv(Rt, RB.Tp(T)); return RB.T(Rt, [-q[0], -q[1], -q[2]]); };
RB.Tv = (T, p) => [0, 1, 2].map(i => T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);
RB.trans = (x, y, z) => RB.T(RB.I3(), [x, y, z]);
/* rotation by θ about the unit vector k through the origin (Craig 2.77, the equivalent angle–axis) */
RB.Rk = (k, th) => { const [x, y, z] = RB.unit(k), c = cos(th), s = sin(th), w = 1 - c;
  return [[x * x * w + c, x * y * w - z * s, x * z * w + y * s], [x * y * w + z * s, y * y * w + c, y * z * w - x * s], [x * z * w - y * s, y * z * w + x * s, z * z * w + c]]; };
/* back from R to the angle and the axis (Craig 2.80); at θ = 0 the axis is undefined, at θ = 180° the formula divides by zero */
RB.axisAngle = R => { const th = Math.acos(NUM.clamp((R[0][0] + R[1][1] + R[2][2] - 1) / 2, -1, 1)), s = sin(th);
  if (abs(s) < 1e-9) return { th, k: null }; return { th, k: [(R[2][1] - R[1][2]) / (2 * s), (R[0][2] - R[2][0]) / (2 * s), (R[1][0] - R[0][1]) / (2 * s)] }; };
/* X–Y–Z fixed angles (γ about X_A, then β about Y_A, then α about Z_A) and Z–Y–X Euler angles (α about Z_B, then β about Y_B, then γ about X_B)
   give the same matrix R_Z(α) R_Y(β) R_X(γ) (Craig 2.64 and 2.71) */
RB.Rzyx = (al, be, ga) => RB.mm(RB.mm(RB.Rz(al), RB.Ry(be)), RB.Rx(ga));
/* the angles back from R (Craig 2.66). At cos β = 0 only α − γ (or α + γ) is fixed: Craig's convention takes α = 0 (2.67, 2.68) */
RB.zyx = R => { const cb = hypot(R[0][0], R[1][0]), be = atan2(-R[2][0], cb);
  if (cb < 1e-9) return be > 0 ? { al: 0, be: PI / 2, ga: atan2(R[0][1], R[1][1]), deg: true } : { al: 0, be: -PI / 2, ga: -atan2(R[0][1], R[1][1]), deg: true };
  return { al: atan2(R[1][0] / cb, R[0][0] / cb), be, ga: atan2(R[2][1] / cb, R[2][2] / cb), deg: false }; };
/* Euler parameters (unit quaternion) ε1…ε4 from an axis and angle (Craig 2.91), from a matrix (2.93), and the matrix back (2.92) */
RB.eulerP = (k, th) => { const u = RB.unit(k), s = sin(th / 2); return [u[0] * s, u[1] * s, u[2] * s, cos(th / 2)]; };
RB.eulerPR = R => { const e4 = 0.5 * sqrt(max(0, 1 + R[0][0] + R[1][1] + R[2][2])); return [(R[2][1] - R[1][2]) / (4 * e4), (R[0][2] - R[2][0]) / (4 * e4), (R[1][0] - R[0][1]) / (4 * e4), e4]; };
RB.Reps = e => { const [a, b, c, d] = e; return [[1 - 2 * b * b - 2 * c * c, 2 * (a * b - c * d), 2 * (a * c + b * d)], [2 * (a * b + c * d), 1 - 2 * a * a - 2 * c * c, 2 * (b * c - a * d)], [2 * (a * c - b * d), 2 * (b * c + a * d), 1 - 2 * a * a - 2 * b * b]]; };
/* ---------------- Denavit–Hartenberg link transforms (Craig ch. 3) ---------------- */
/* Craig's modified convention: ^{i−1}_i T = R_X(α_{i−1}) D_X(a_{i−1}) R_Z(θ_i) D_Z(d_i) (Craig 3.6) */
RB.dhT = (al, a, th, d) => { const ca = cos(al), sa = sin(al), ct = cos(th), st = sin(th);
  return [[ct, -st, 0, a], [st * ca, ct * ca, -sa, -sa * d], [st * sa, ct * sa, ca, ca * d], [0, 0, 0, 1]]; };
/* the classical (standard) convention of Spong and Corke: ^{i−1}_i T = R_Z(θ_i) D_Z(d_i) D_X(a_i) R_X(α_i) */
RB.dhTc = (th, d, a, al) => { const ca = cos(al), sa = sin(al), ct = cos(th), st = sin(th);
  return [[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]]; };
/* forward kinematics of a DH table. rows = [{al, a, d, th, j}] (angles in rad; for the classical convention the same names hold α_i, a_i, d_i, θ_i);
   j = 'R' adds q[i] to θ, j = 'P' adds q[i] to d, no j: a fixed frame (a tool). Returns every frame ^0_i T, i = 0…n */
RB.fkDH = (rows, q = [], conv) => { const out = [RB.T(RB.I3())]; let T = out[0];
  rows.forEach((r, i) => { const th = r.th + (r.j === 'R' ? (q[i] || 0) : 0), d = r.d + (r.j === 'P' ? (q[i] || 0) : 0); T = RB.Tm(T, conv === 'c' ? RB.dhTc(th, d, r.a, r.al) : RB.dhT(r.al, r.a, th, d)); out.push(T); });
  return out; };
/* points of a circle of radius r about the axis through o along d, from angle a0 to a1 */
RB.ring = (o, d, r, a0 = 0, a1 = 2 * PI, n = 40) => { const u = RB.unit(RB.cross(d, abs(d[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1])), v = RB.cross(d, u), out = []; for (let k = 0; k <= n; k++) { const a = a0 + (a1 - a0) * k / n; out.push(RB.add(o, RB.add(RB.sc(u, r * cos(a)), RB.sc(v, r * sin(a))))); } return out; };
/* orthographic camera looking at the origin from azimuth az and elevation el (z is up). cam.p([x,y,z]) → [sx, sy, depth] */
RB.cam = (az, el, s, cx, cy) => { const ca = cos(az), sa = sin(az), ce = cos(el), se = sin(el);
  return { s, cx, cy, p: v => [cx + s * (-sa * v[0] + ca * v[1]), cy - s * (-ca * se * v[0] - sa * se * v[1] + ce * v[2]), ca * ce * v[0] + sa * ce * v[1] + se * v[2]] }; };
/* the five common arrangements, named by their first three joints. q = the three joint variables (rad or m).
   Each returns the points of the stick figure from the floor to the tool point, and the three joint axes {o: point, d: unit direction, t: 'R' | 'P'}. */
RB.ARR = {
  RRR: { joints: 'RRR', seg: [1, 2, 3], ctr: [0, 0, 0.08], lim: [[-170, 170, 30], [-30, 150, 55], [-150, 0, -70]], unit: ['deg', 'deg', 'deg'],
    kin: q => { const [t1, t2, t3] = q, h = 0.45, a2 = 0.5, a3 = 0.45, c = cos(t1), s = sin(t1), r2 = a2 * cos(t2), z2 = h + a2 * sin(t2), r3 = r2 + a3 * cos(t2 + t3), z3 = z2 + a3 * sin(t2 + t3);
      return { pts: [[0, 0, 0], [0, 0, h], [r2 * c, r2 * s, z2], [r3 * c, r3 * s, z3]], axes: [{ o: [0, 0, h * 0.5], d: [0, 0, 1], t: 'R' }, { o: [0, 0, h], d: [-s, c, 0], t: 'R' }, { o: [r2 * c, r2 * s, z2], d: [-s, c, 0], t: 'R' }] }; } },
  RRP: { joints: 'RRP', seg: [1, 2, 3], lim: [[-170, 170, 30], [-20, 80, 30], [0.3, 0.9, 0.65]], unit: ['deg', 'deg', 'm'],
    kin: q => { const [t1, t2, d3] = q, h = 0.5, c = cos(t1), s = sin(t1), r = d3 * cos(t2), z = h + d3 * sin(t2), b = 0.28;
      return { pts: [[0, 0, 0], [0, 0, h], [b * cos(t2) * c, b * cos(t2) * s, h + b * sin(t2)], [r * c, r * s, z]], axes: [{ o: [0, 0, h * 0.5], d: [0, 0, 1], t: 'R' }, { o: [0, 0, h], d: [-s, c, 0], t: 'R' }, { o: [b * cos(t2) * c, b * cos(t2) * s, h + b * sin(t2)], d: [cos(t2) * c, cos(t2) * s, sin(t2)], t: 'P' }] }; } },
  SCARA: { joints: 'RRP', seg: [0, 1, 2, 3], lim: [[-130, 130, 35], [-140, 140, 60], [0.05, 0.35, 0.2]], unit: ['deg', 'deg', 'm'],
    kin: q => { const [t1, t2, d3] = q, h = 0.6, a1 = 0.45, a2 = 0.35, e = [a1 * cos(t1), a1 * sin(t1)], w = [e[0] + a2 * cos(t1 + t2), e[1] + a2 * sin(t1 + t2)];
      return { pts: [[0, 0, 0], [0, 0, h], [e[0], e[1], h], [w[0], w[1], h], [w[0], w[1], h - d3]], axes: [{ o: [0, 0, h], d: [0, 0, 1], t: 'R' }, { o: [e[0], e[1], h], d: [0, 0, 1], t: 'R' }, { o: [w[0], w[1], h], d: [0, 0, -1], t: 'P' }] }; } },
  RPP: { joints: 'RPP', seg: [1, 2, 3], lim: [[-170, 170, 35], [0.25, 0.85, 0.6], [0.3, 0.85, 0.6]], unit: ['deg', 'm', 'm'],
    kin: q => { const [t1, d2, d3] = q, c = cos(t1), s = sin(t1), b = 0.2;
      return { pts: [[0, 0, 0], [0, 0, d2], [b * c, b * s, d2], [d3 * c, d3 * s, d2]], axes: [{ o: [0, 0, 0.12], d: [0, 0, 1], t: 'R' }, { o: [0, 0, d2], d: [0, 0, 1], t: 'P' }, { o: [b * c, b * s, d2], d: [c, s, 0], t: 'P' }] }; } },
  PPP: { joints: 'PPP', seg: [1, 2, 3], ctr: [0.5, 0.42, 0.05], lim: [[0.1, 0.9, 0.6], [0.1, 0.8, 0.5], [0.1, 0.7, 0.45]], unit: ['m', 'm', 'm'],
    kin: q => { const [d1, d2, d3] = q, H = 0.85;
      return { pts: [[d1, 0, 0], [d1, 0, H], [d1, d2, H], [d1, d2, H - d3]], axes: [{ o: [d1, 0, 0.06], d: [1, 0, 0], t: 'P' }, { o: [d1, d2 * 0.5, H], d: [0, 1, 0], t: 'P' }, { o: [d1, d2, H - 0.06], d: [0, 0, -1], t: 'P' }] }; } }
};
RB.arrQ = (type, q) => q.map((v, i) => RB.ARR[type].unit[i] === 'deg' ? rad(v) : v);          // slider values (deg or m) → rad or m
RB.arrTip = (type, q) => { const p = RB.ARR[type].kin(q).pts; return p[p.length - 1]; };
/* wire frame of the workspace: sweep one joint over its range while the other two sit at their low, middle and high values */
RB.arrWire = (type, n = 36) => { const A = RB.ARR[type], L = A.lim.map((l, i) => A.unit[i] === 'deg' ? [rad(l[0]), rad(l[1])] : [l[0], l[1]]), out = [];
  for (let j = 0; j < 3; j++) { const o = [0, 1, 2].filter(k => k !== j); for (const u of [0, .5, 1]) for (const v of [0, .5, 1]) { const line = [];
    for (let k = 0; k <= n; k++) { const q = []; q[j] = L[j][0] + (L[j][1] - L[j][0]) * k / n; q[o[0]] = L[o[0]][0] + (L[o[0]][1] - L[o[0]][0]) * u; q[o[1]] = L[o[1]][0] + (L[o[1]][1] - L[o[1]][0]) * v; line.push(RB.arrTip(type, q)); }
    out.push({ j, edge: (u === 0 || u === 1) && (v === 0 || v === 1), line }); } }
  return out; };

/* ---------------- canvas helpers ---------------- */
RB.setup = (canvas, w, h) => { const dpr = RB.DPR || Math.min(global.devicePixelRatio || 1, 2.5); canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
  canvas.style.width = w + 'px'; canvas.style.height = h + 'px'; const ctx = canvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); return ctx; };
RB.fit = (canvas, aspect = 0.62, minW = 260, maxH = 0) => { const w = max(minW, Math.floor(canvas.parentElement.clientWidth - 12)); let h = Math.round(w * aspect); if (maxH && h > maxH) h = maxH; return { ctx: RB.setup(canvas, w, h), w, h }; };
/* a canvas that keeps its size in step with its card and is redrawn every frame by the shared loop of lesson.js:
   frame(ctx, W, H, dt, P). Returns {ctx, W, H, P} (updated in place when the width changes). */
RB.stage = (canvas, aspect, frame, o = {}) => { const st = { ctx: null, W: 0, H: 0, P: o.pal || RB.DARK, pw: -1 };
  const size = () => { const r = RB.fit(canvas, aspect, o.minW || 260, o.maxH || 0); st.ctx = r.ctx; st.W = r.w; st.H = r.h; };
  /* light pictures (as on the slides) for the light reading themes, dark ones for the dark theme */
  const tick = dt => { const pw = canvas.parentElement.clientWidth; if (pw !== st.pw) { st.pw = pw; size(); } if (!o.pal) st.P = (global.MC && global.MC.theme === 'dark') ? RB.DARK : RB.LIGHT; frame(st.ctx, st.W, st.H, dt, st.P); };
  if (global.LS && global.LS.anim) global.LS.anim(canvas, tick); else tick(0);
  if (canvas.id) RB.stages[canvas.id] = { canvas, frame };
  return st; };
/* checking a page without watching it (QA, a hidden browser): RB.snap(id, W, H) draws one frame of canvas id into a fresh light canvas
   and returns it as a PNG data URL (no time passes: dt = 0) */
RB.stages = {};
RB.snap = (id, W = 640, H = 480, dark) => { const S = RB.stages[id]; if (!S) return null; const c = document.createElement('canvas'); c.width = W; c.height = H; const x = c.getContext('2d');
  const P = dark ? RB.DARK : RB.LIGHT; x.fillStyle = P.bg; x.fillRect(0, 0, W, H); S.frame(x, W, H, 0, P); return c.toDataURL(); };
/* world → screen for the planar scenes: origin at (cx, cy) px, s px per metre, y up */
RB.fitWorld = (W, H, x0, x1, y0, y1, pad = 8, ox = 0, oy = 0) => { const s = min((W - 2 * pad) / (x1 - x0), (H - 2 * pad) / (y1 - y0)); return RB.map(ox + (W - s * (x1 + x0)) / 2, oy + (H + s * (y1 + y0)) / 2, s); };
RB.map = (cx, cy, s) => ({ cx, cy, s, x: wx => cx + s * wx, y: wy => cy - s * wy, p: (wx, wy) => [cx + s * wx, cy - s * wy], inv: (sx, sy) => [(sx - cx) / s, (cy - sy) / s] });
/* pointer dragging on a canvas (mouse and touch): on(p, phase) with p = {x, y} in CSS px and phase 'down' | 'move' | 'up'; hit(p) decides whether a drag starts */
RB.drag = (canvas, hit, on) => { let act = false; const pt = ev => { const r = canvas.getBoundingClientRect(), t = ev.touches && ev.touches.length ? ev.touches[0] : ev; return { x: t.clientX - r.left, y: t.clientY - r.top }; };
  const down = ev => { const p = pt(ev); if (!hit || hit(p)) { act = true; ev.preventDefault(); on(p, 'down'); } }, move = ev => { if (!act) return; ev.preventDefault(); on(pt(ev), 'move'); }, up = () => { if (act) { act = false; on(null, 'up'); } };
  canvas.addEventListener('mousedown', down); global.addEventListener('mousemove', move); global.addEventListener('mouseup', up);
  canvas.addEventListener('touchstart', down, { passive: false }); global.addEventListener('touchmove', move, { passive: false }); global.addEventListener('touchend', up); };

/* ---------------- drawing primitives (screen coordinates unless a name says world) ---------------- */
const G = {};
G.clear = (c, W, H, P) => { c.fillStyle = P.bg; c.fillRect(0, 0, W, H); };
G.line = (c, x1, y1, x2, y2, col, lw = 1.5, dash) => { c.save(); c.strokeStyle = col; c.lineWidth = lw; c.lineCap = 'round'; if (dash) c.setLineDash(dash); c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke(); c.restore(); };
G.poly = (c, pts, o = {}) => { if (!pts.length) return; c.save(); c.beginPath(); pts.forEach((p, i) => i ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1])); if (o.close) c.closePath();
  if (o.fill) { c.fillStyle = o.fill; c.fill(); } if (o.stroke) { c.strokeStyle = o.stroke; c.lineWidth = o.lw || 1.5; c.lineJoin = 'round'; c.lineCap = 'round'; if (o.dash) c.setLineDash(o.dash); c.stroke(); } c.restore(); };
G.circle = (c, x, y, r, o = {}) => { c.save(); c.beginPath(); c.arc(x, y, r, 0, 2 * PI); if (o.fill) { c.fillStyle = o.fill; c.fill(); } if (o.stroke) { c.strokeStyle = o.stroke; c.lineWidth = o.lw || 1.5; if (o.dash) c.setLineDash(o.dash); c.stroke(); } c.restore(); };
G.head = (c, x, y, ang, col, size = 9) => { c.save(); c.translate(x, y); c.rotate(ang); c.fillStyle = col; c.beginPath(); c.moveTo(0, 0); c.lineTo(-size, size * 0.42); c.lineTo(-size, -size * 0.42); c.closePath(); c.fill(); c.restore(); };
G.arrow = (c, x1, y1, x2, y2, col, lw = 2, size = 9, both = false, dash) => { const a = atan2(y2 - y1, x2 - x1), k = size * 0.7; G.line(c, x1 + (both ? k * cos(a) : 0), y1 + (both ? k * sin(a) : 0), x2 - k * cos(a), y2 - k * sin(a), col, lw, dash);
  G.head(c, x2, y2, a, col, size); if (both) G.head(c, x1, y1, a + PI, col, size); };
/* arc about (cx, cy) from world angle a0 to a1 (counter-clockwise positive, as on paper); o.head puts an arrow head at a1, o.both at both ends */
G.arc = (c, cx, cy, r, a0, a1, col, lw = 1.5, o = {}) => { if (abs(a1 - a0) < 1e-6) return; c.save(); c.strokeStyle = col; c.lineWidth = lw; c.lineCap = 'round'; if (o.dash) c.setLineDash(o.dash);
  c.beginPath(); c.arc(cx, cy, r, -a0, -a1, a1 > a0); c.stroke(); c.restore();
  const hd = (a, dir) => { const hs = o.size || 8, x = cx + r * cos(a), y = cy - r * sin(a), back = a - dir * min(abs(a1 - a0), hs / r) * 0.6; G.head(c, x, y, atan2(-(sin(a) - sin(back)), cos(a) - cos(back)), col, hs); };
  if (o.head || o.both) hd(a1, a1 > a0 ? 1 : -1); if (o.both) hd(a0, a1 > a0 ? -1 : 1); };
G.text = (c, s, x, y, o = {}) => { c.save(); c.font = `${o.italic ? 'italic ' : ''}${o.weight || 500} ${o.size || 13}px ${o.font || (o.mono ? RB.MONO : RB.FONT)}`; c.textAlign = o.align || 'left'; c.textBaseline = o.base || 'alphabetic';
  if (o.halo) { c.lineWidth = o.haloW || 4; c.strokeStyle = o.halo; c.lineJoin = 'round'; c.strokeText(s, x, y); } c.fillStyle = o.color || '#000'; c.fillText(s, x, y); const w = c.measureText(s).width; c.restore(); return w; };
/* a math symbol with a subscript, e.g. θ₁: italic serif letter, small upright index. Returns the width drawn. o.align: left | center | right */
G.sym = (c, s, sub, x, y, o = {}) => { const size = o.size || 16, f = `italic 500 ${size}px ${RB.SERIF}`, f2 = `500 ${Math.round(size * 0.68)}px ${RB.SERIF}`; c.save(); c.font = f; const w1 = c.measureText(s).width; c.font = f2; const w2 = sub ? c.measureText(sub).width : 0;
  const x0 = o.align === 'center' ? x - (w1 + w2) / 2 : o.align === 'right' ? x - w1 - w2 : x; c.textBaseline = 'alphabetic'; c.textAlign = 'left';
  const put = (str, font, px, py) => { c.font = font; if (o.halo) { c.lineWidth = 4; c.strokeStyle = o.halo; c.lineJoin = 'round'; c.strokeText(str, px, py); } c.fillStyle = o.color || '#000'; c.fillText(str, px, py); };
  put(s, f, x0, y); if (sub) put(sub, f2, x0 + w1 + 0.5, y + size * 0.22); c.restore(); return w1 + w2; };
/* a rounded label box */
G.pill = (c, s, x, y, o = {}) => { const size = o.size || 13; c.save(); c.font = `${o.weight || 600} ${size}px ${RB.FONT}`; const w = c.measureText(s).width + 16, h = size + 11, x0 = o.align === 'center' ? x - w / 2 : o.align === 'right' ? x - w : x;
  c.beginPath(); if (c.roundRect) c.roundRect(x0, y - h / 2, w, h, h / 2); else c.rect(x0, y - h / 2, w, h); if (o.fill) { c.fillStyle = o.fill; c.fill(); } if (o.stroke) { c.strokeStyle = o.stroke; c.lineWidth = 1.2; c.stroke(); }
  c.fillStyle = o.color || '#000'; c.textAlign = 'left'; c.textBaseline = 'middle'; c.fillText(s, x0 + 8, y + 0.5); c.restore(); return w; };
/* a link: a thick bar with round ends */
G.link = (c, x1, y1, x2, y2, col, w = 12, P) => { c.save(); c.lineCap = 'round'; if (P) { c.strokeStyle = P.ink; c.globalAlpha *= 0.55; c.lineWidth = w + 2.4; c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke(); c.globalAlpha /= 0.55; }
  c.strokeStyle = col; c.lineWidth = w; c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke(); c.restore(); };
/* revolute joint seen along its axis: a ring with a centre dot */
G.rjoint = (c, x, y, r, P, col) => { G.circle(c, x, y, r, { fill: P.joint, stroke: col || P.ink, lw: 2.4 }); G.circle(c, x, y, r * 0.3, { fill: col || P.ink }); };
/* prismatic joint: a sleeve (open box) around the sliding link, drawn along the screen angle ang */
G.pjoint = (c, x, y, ang, len, w, P, col) => { c.save(); c.translate(x, y); c.rotate(ang); c.fillStyle = P.joint; c.strokeStyle = col || P.ink; c.lineWidth = 2.4; c.beginPath(); c.rect(-len / 2, -w / 2, len, w); c.fill(); c.stroke(); c.restore(); };
/* fixed ground: a line with hatching underneath */
G.ground = (c, x, y, w, P) => { G.line(c, x - w / 2, y, x + w / 2, y, P.base, 2.4); c.save(); c.strokeStyle = P.base; c.lineWidth = 1.2; c.beginPath(); for (let i = -w / 2 + 4; i <= w / 2; i += 9) { c.moveTo(x + i, y + 1); c.lineTo(x + i - 7, y + 9); } c.stroke(); c.restore(); };
/* a two-finger gripper at (x, y), pointing along the world angle ang; open 0..1 */
G.gripper = (c, x, y, ang, s, col, open = 1) => { c.save(); c.translate(x, y); c.rotate(-ang); c.strokeStyle = col; c.lineWidth = max(2, s * 0.22); c.lineCap = 'round'; c.lineJoin = 'round'; const g = s * (0.28 + 0.34 * open);
  c.beginPath(); c.moveTo(0, -g); c.lineTo(0, g); c.moveTo(0, -g); c.lineTo(s, -g); c.moveTo(0, g); c.lineTo(s, g); c.stroke(); c.restore(); };
/* light grid in world units */
G.grid = (c, M, x0, x1, y0, y1, step, col) => { c.save(); c.strokeStyle = col; c.lineWidth = 1; c.beginPath(); for (let x = Math.ceil(x0 / step) * step; x <= x1 + 1e-9; x += step) { c.moveTo(M.x(x), M.y(y0)); c.lineTo(M.x(x), M.y(y1)); }
  for (let y = Math.ceil(y0 / step) * step; y <= y1 + 1e-9; y += step) { c.moveTo(M.x(x0), M.y(y)); c.lineTo(M.x(x1), M.y(y)); } c.stroke(); c.restore(); };
RB.G = G;

global.RB = RB;
})(typeof window !== 'undefined' ? window : globalThis);
