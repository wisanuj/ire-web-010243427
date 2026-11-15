/* ch2.js — chapter 2, spatial descriptions and transformations (Craig ch. 2): the numbers of the worked examples and the homework,
   rotation sequences, and helpers that write matrices as TeX or HTML. Works in the browser (window.CH2) and in deno (globalThis.CH2).
   Needs robot.js. The 3-D scenes of the chapter are in ch2_3d.js; tests.js checks every number below against Craig and the old slides. */
(function (global) {
'use strict';
const RB = global.RB, rad = RB.rad, { sqrt, abs } = Math;
const CH2 = {};

/* ---------- writing numbers and matrices ---------- */
/* exact: write ±√3/2, ±1/2, ±√2/2 (and ±1/√2) as fractions, the way the slides do */
const EXACT = [[sqrt(3) / 2, '\\tfrac{\\sqrt{3}}{2}', '√3/2'], [0.5, '\\tfrac{1}{2}', '1/2'], [sqrt(2) / 2, '\\tfrac{1}{\\sqrt{2}}', '1/√2']];
CH2.num = (x, d = 3, exact = false) => { if (abs(x) < 5e-10) return '0'; if (exact) for (const [v, t] of EXACT) if (abs(abs(x) - v) < 1e-9) return (x < 0 ? '-' : '') + t; return RB.d(x, d); };
CH2.txt = (x, d = 3, exact = false) => { if (abs(x) < 5e-10) return '0'; if (exact) for (const [v, , t] of EXACT) if (abs(abs(x) - v) < 1e-9) return (x < 0 ? '−' : '') + t; return RB.u(x, d); };
/* a matrix as TeX; o.cols = colours of the columns (e.g. the X, Y, Z axes of the moved frame), o.exact = fractions */
CH2.COL = { x: '#d9363e', y: '#22985a', z: '#3a6ee8', p: '#c2356f', o: '#e8590c' };
CH2.texM = (M, d = 3, o = {}) => `\\begin{bmatrix} ${M.map(r => r.map((x, j) => { const t = CH2.num(x, d, o.exact); return o.cols && o.cols[j] ? `\\textcolor{${o.cols[j]}}{${t}}` : t; }).join(' & ')).join(' \\\\ ')} \\end{bmatrix}`;
CH2.texV = (p, d = 3, o = {}) => CH2.texM(p.map(x => [x]), d, o);
/* a live matrix for readouts (HTML, cheap to redraw at every slider move): cls = class per column (mx, my, mz, …) */
CH2.htmlM = (M, d = 3, o = {}) => `<span class="mat" style="grid-template-columns:repeat(${M[0].length},auto)">` + M.map((r, i) => r.map((x, j) => `<span class="${o.cls && o.cls[j] ? o.cls[j] : ''}${o.sep && j === o.sep ? ' sep' : ''}${o.last && i === M.length - 1 ? ' lastrow' : ''}">${CH2.txt(x, d, o.exact)}</span>`).join('')).join('') + '</span>';
CH2.R3cls = ['mx', 'my', 'mz'];

/* ---------- rotation sequences ----------
   steps = [[axis 'x' | 'y' | 'z', angle in degrees, 'fixed' | 'euler'], …]. A rotation about an axis of the fixed (reference) frame multiplies
   on the left, a rotation about an axis of the moving frame multiplies on the right (Craig 2.8; slide "difference between fixed and Euler angles") */
const ROT = { x: RB.Rx, y: RB.Ry, z: RB.Rz };
CH2.seqR = steps => steps.reduce((R, [a, d, kind]) => kind === 'fixed' ? RB.mm(ROT[a](rad(d)), R) : RB.mm(R, ROT[a](rad(d))), RB.I3());
/* the same as a product of named factors, e.g. "R_Y(45°) R_X(30°) …" in TeX */
CH2.seqTeX = steps => { let f = []; steps.forEach(([a, d, kind]) => { const t = `R_${a.toUpperCase()}(${d}^\\circ)`; if (kind === 'fixed') f = [t, ...f]; else f = [...f, t]; }); return f.join('\\,'); };

/* ---------- the numbers of the chapter ---------- */
const s3 = sqrt(3) / 2, R2 = 1 / sqrt(2);
CH2.EX = {
  /* Part I slide 16: X_B along X_A, Y_B along Z_A, Z_B along −Y_A */
  ex1: { R: [[1, 0, 0], [0, 0, -1], [0, 1, 0]] },
  /* Craig example 2.1: {B} turned 30° about Z_A, ^B P = (0, 2, 0) */
  ex21: { th: 30, bp: [0, 2, 0] },
  /* Craig example 2.2: turned 30° about Z_A, moved 10 along X_A and 5 along Y_A, ^B P = (3, 7, 0) */
  ex22: { th: 30, p: [10, 5, 0], bp: [3, 7, 0] },
  /* Craig example 2.4 (operator): turn ^A P_1 = (3, 7, 0) by 30° about Z_A, then move it by (10, 5, 0) */
  ex24: { th: 30, q: [10, 5, 0], p1: [3, 7, 0] },
  /* Craig example 2.5: {B} turned 30° about Z_A and moved 4 along X_A, 3 along Y_A: find ^A_B T and ^B_A T */
  ex25: { th: 30, p: [4, 3, 0] },
  /* Craig examples 2.6 and 2.7: rotate about K = (0.707, 0.707, 0) by 30°, through the origin and through ^A P = (1, 2, 3) */
  ex26: { k: [R2, R2, 0], th: 30 },
  ex27: { k: [R2, R2, 0], th: 30, p: [1, 2, 3] },
  /* Part II slide 18: the five rotations of the in-class example (1 and 4, 5 about the fixed axes, 2 and 3 about the moving axes) */
  seq: [['x', 30, 'fixed'], ['y', 45, 'euler'], ['z', 60, 'euler'], ['x', 30, 'fixed'], ['y', 45, 'fixed']],
  /* a matrix to take back to angles (new practice): made from α = 60°, β = 30°, γ = −45° */
  inv: { al: 60, be: 30, ga: -45 },
  /* homework 2.1 (Part I slide 37, Spong problem 2-37): frame 0 at a corner of a wedge, x0 into the page, y0 to the left, z0 up;
     frame 1 one unit above it (x1 down, y1 = x0, z1 = −y0); frame 2 one unit along y0 (x2 = −y0, y2 = z0, z2 = −x0) */
  hw21: { T01: RB.T([[0, 1, 0], [0, 0, -1], [-1, 0, 0]], [0, 0, 1]), T02: RB.T([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], [0, 1, 0]) },
  /* homework 2.2 (Part II slide 26): ^1_2 R and ^1_3 R are given, find ^2_3 R */
  hw22: { R12: [[1, 0, 0], [0, 0.5, -s3], [0, s3, 0.5]], R13: [[0, 0, -1], [0, 1, 0], [1, 0, 0]] },
  /* transform equation (Craig 2.6, the arm reaching for a bolt), with numbers made for this course:
     tool {T} pointing down above the table, station {S} at a corner of the table, goal {G} on the bolt */
  bolt: { BT: RB.T(RB.Rx(Math.PI), [0.5, 0.2, 0.6]), BS: RB.T(RB.Rz(Math.PI / 2), [0.9, -0.5, 0.3]), SG: RB.T(RB.I3(), [0.2, 0.3, 0]) }
};
/* answers, worked out with the functions of robot.js (tests.js compares them with Craig and the old slides) */
const E = CH2.EX;
E.ex21.ap = RB.mv(RB.Rz(rad(30)), E.ex21.bp);
E.ex22.T = RB.T(RB.Rz(rad(30)), E.ex22.p); E.ex22.ap = RB.Tv(E.ex22.T, E.ex22.bp);
E.ex24.T = RB.T(RB.Rz(rad(30)), E.ex24.q); E.ex24.p2 = RB.Tv(E.ex24.T, E.ex24.p1);
E.ex25.T = RB.T(RB.Rz(rad(30)), E.ex25.p); E.ex25.Ti = RB.Ti(E.ex25.T);
E.ex26.R = RB.Rk(E.ex26.k, rad(30));
E.ex27.T = RB.Tm(RB.trans(...E.ex27.p), RB.T(RB.Rk(E.ex27.k, rad(30))), RB.trans(-1, -2, -3));
E.seq.R = CH2.seqR(E.seq);
E.inv.R = RB.Rzyx(rad(60), rad(30), rad(-45));
E.hw21.T12 = RB.Tm(RB.Ti(E.hw21.T01), E.hw21.T02);
E.hw22.R23 = RB.mm(RB.tr(E.hw22.R12), E.hw22.R13);
E.bolt.TG = RB.Tm(RB.Ti(E.bolt.BT), E.bolt.BS, E.bolt.SG);
E.bolt.BG = RB.Tm(E.bolt.BS, E.bolt.SG);

/* the articulated arm that holds the tool in the transform-equation scene: the RRR arm of chapter 1 (RB.ARR.RRR) with a 0.12 m tool
   pointing straight down under its wrist. Returns the three joint angles (elbow up), or null when the wrist is out of reach */
CH2.TOOL = 0.12;
CH2.armTo = p => { const w = [p[0], p[1], p[2] + CH2.TOOL], t1 = Math.atan2(w[1], w[0]), r = Math.hypot(w[0], w[1]), q = RB.ik2(0.5, 0.45, r, w[2] - 0.45, -1); return q.ok ? [t1, q.t1, q.t2] : null; };

CH2.FIGS = {};   // every figure of chapter 2 is a 3-D render (IRE_Slides/build/coppelia/gen_scenes_ch2.mjs)
global.CH2 = CH2;
})(typeof window !== 'undefined' ? window : globalThis);
