/* ch3.js — chapter 3, forward kinematics (Craig ch. 3): the example manipulators as Denavit–Hartenberg tables, their closed-form
   kinematic equations, and helpers for the pages (a DH table to fill in with drop-down cells). Works in the browser (window.CH3)
   and in deno (globalThis.CH3). Needs robot.js (RB.dhT, RB.dhTc, RB.fkDH) and ch2.js (matrix formatting).
   Table rows: {al, a, d, th, j} as in RB.fkDH, angles in rad; sym = the same table in symbols, as written on the slides. */
(function (global) {
'use strict';
const RB = global.RB, rad = RB.rad, { sin, cos, PI } = Math, H = PI / 2;
const CH3 = {};
const c = Math.cos, s = Math.sin;
const row = (al, a, d, th, j) => ({ al, a, d, th, j });
/* the columns of Craig's table and of the classical table */
CH3.COLS = ['α<sub>i−1</sub>', 'a<sub>i−1</sub>', 'd<sub>i</sub>', 'θ<sub>i</sub>'];
CH3.COLS_C = ['θ<sub>i</sub>', 'd<sub>i</sub>', 'a<sub>i</sub>', 'α<sub>i</sub>'];

CH3.EX = {
  /* Craig example 3.3: the planar arm with three revolute joints (old Part I slides 13–14). Tool frame {4} at the tip: homework 3(b) */
  p3r: { name: '3R', L: { L1: 0.5, L2: 0.4, L3: 0.15 }, q: [rad(30), rad(45), rad(-60)],
    rows: L => [row(0, 0, 0, 0, 'R'), row(0, L.L1, 0, 0, 'R'), row(0, L.L2, 0, 0, 'R')], tool: L => row(0, L.L3, 0, 0),
    sym: [['0', '0', '0', 'θ₁'], ['0', 'L₁', '0', 'θ₂'], ['0', 'L₂', '0', 'θ₃']], symTool: ['0', 'L₃', '0', '0'],
    /* ^0_3 T in closed form (Craig 3.4 … the product of the three link transforms) */
    T03: (L, q) => { const [a, b, g] = q, c1 = c(a), s1 = s(a), c12 = c(a + b), s12 = s(a + b), c123 = c(a + b + g), s123 = s(a + b + g);
      return [[c123, -s123, 0, L.L1 * c1 + L.L2 * c12], [s123, c123, 0, L.L1 * s1 + L.L2 * s12], [0, 0, 1, 0], [0, 0, 0, 1]]; },
    T04: (L, q) => { const T = CH3.EX.p3r.T03(L, q), c123 = T[0][0], s123 = T[1][0]; return [[c123, -s123, 0, T[0][3] + L.L3 * c123], [s123, c123, 0, T[1][3] + L.L3 * s123], [0, 0, 1, 0], [0, 0, 0, 1]]; } },
  /* Craig example 3.4: the RPR arm (old chapter 3.1 slides 2–3); frame {0} = {1} on top of the column of height L1 */
  rpr: { name: 'RPR', L: { L1: 0.45, L2: 0.15, L3: 0.1 }, q: [rad(30), 0.35, rad(45)],
    rows: L => [row(0, 0, 0, 0, 'R'), row(H, 0, 0, 0, 'P'), row(0, 0, L.L2, 0, 'R')],
    sym: [['0', '0', '0', 'θ₁'], ['90°', '0', 'd₂', '0'], ['0', '0', 'L₂', 'θ₃']],
    T03: (L, q) => { const [t1, d2, t3] = q, c1 = c(t1), s1 = s(t1), c3 = c(t3), s3 = s(t3), r = d2 + L.L2;
      return [[c1 * c3, -c1 * s3, s1, s1 * r], [s1 * c3, -s1 * s3, -c1, -c1 * r], [s3, c3, 0, 0], [0, 0, 0, 1]]; },
    /* homework 3(a) (old chapter 3.1 slide 4): the same arm with {0} on the floor (d1 = L1) and a tool frame {4} at d4 = L3 */
    hwRows: L => [row(0, 0, L.L1, 0, 'R'), row(H, 0, 0, 0, 'P'), row(0, 0, L.L2, 0, 'R'), row(0, 0, L.L3, 0)],
    hwSym: [['0', '0', 'L₁', 'θ₁'], ['90°', '0', 'd₂', '0'], ['0', '0', 'L₂', 'θ₃'], ['0', '0', 'L₃', '0']],
    T04: (L, q) => { const [t1, d2, t3] = q, c1 = c(t1), s1 = s(t1), c3 = c(t3), s3 = s(t3), r = d2 + L.L2 + L.L3;
      return [[c1 * c3, -c1 * s3, s1, s1 * r], [s1 * c3, -s1 * s3, -c1, -c1 * r], [s3, c3, 0, L.L1], [0, 0, 0, 1]]; } },
  /* the arm whose first axis is twisted 90° from the other two (old chapter 3.1 slides 6–7): the wrist frame {W} = {3}, no L3 needed */
  twist: { name: 'RRR', L: { L1: 0.25, L2: 0.5 }, q: [rad(30), rad(40), rad(-50)],
    rows: L => [row(0, 0, 0, 0, 'R'), row(H, L.L1, 0, 0, 'R'), row(0, L.L2, 0, 0, 'R')],
    sym: [['0', '0', '0', 'θ₁'], ['90°', 'L₁', '0', 'θ₂'], ['0', 'L₂', '0', 'θ₃']],
    T03: (L, q) => { const [t1, t2, t3] = q, c1 = c(t1), s1 = s(t1), c23 = c(t2 + t3), s23 = s(t2 + t3), r = L.L1 + L.L2 * c(t2);
      return [[c1 * c23, -c1 * s23, s1, c1 * r], [s1 * c23, -s1 * s23, -c1, s1 * r], [s23, c23, 0, L.L2 * s(t2)], [0, 0, 0, 1]]; } },
  /* the arm of old chapter 3.1 slides 8–9: column L1 + L2, upper arm L3, forearm L4 to the tool frame {4} */
  arm4: { name: 'RRR', L: { L1: 0.3, L2: 0.2, L3: 0.5, L4: 0.4 }, q: [rad(30), rad(35), rad(-60)],
    rows: L => [row(0, 0, L.L1 + L.L2, 0, 'R'), row(H, 0, 0, 0, 'R'), row(0, L.L3, 0, 0, 'R'), row(0, L.L4, 0, 0)],
    sym: [['0', '0', 'L₁ + L₂', 'θ₁'], ['90°', '0', '0', 'θ₂'], ['0', 'L₃', '0', 'θ₃'], ['0', 'L₄', '0', '0']],
    T04: (L, q) => { const [t1, t2, t3] = q, c1 = c(t1), s1 = s(t1), c23 = c(t2 + t3), s23 = s(t2 + t3), r = L.L3 * c(t2) + L.L4 * c23;
      return [[c1 * c23, -c1 * s23, s1, c1 * r], [s1 * c23, -s1 * s23, -c1, s1 * r], [s23, c23, 0, L.L1 + L.L2 + L.L3 * s(t2) + L.L4 * s23], [0, 0, 0, 1]]; } },
  /* a SCARA (old Part II slides 6 and 10) in both conventions; {0} at the height of the arm, joint 3 slides down, d4 = the tool length */
  scara: { name: 'SCARA', L: { L1: 0.45, L2: 0.35, d4: 0.05 }, q: [rad(35), rad(-60), 0.15, rad(30)],
    rows: L => [row(0, 0, 0, 0, 'R'), row(0, L.L1, 0, 0, 'R'), row(PI, L.L2, 0, 0, 'P'), row(0, 0, L.d4, 0, 'R')],
    sym: [['0', '0', '0', 'θ₁'], ['0', 'L₁', '0', 'θ₂'], ['180°', 'L₂', 'd₃', '0'], ['0', '0', 'd₄', 'θ₄']],
    /* the classical table: the same names hold θ_i, d_i, a_i, α_i */
    rowsC: L => [row(0, L.L1, 0, 0, 'R'), row(PI, L.L2, 0, 0, 'R'), row(0, 0, 0, 0, 'P'), row(0, 0, L.d4, 0, 'R')],
    symC: [['θ₁', '0', 'L₁', '0'], ['θ₂', '0', 'L₂', '180°'], ['0', 'd₃', '0', '0'], ['θ₄', 'd₄', '0', '0']],
    tip: (L, q) => [L.L1 * c(q[0]) + L.L2 * c(q[0] + q[1]), L.L1 * s(q[0]) + L.L2 * s(q[0] + q[1]), -q[2] - L.d4] },
  /* the Yaskawa Motoman GP8 (the department's own robot), in Craig's convention; frame {0} = {1} on the S axis at the height of the L axis, 0.330 m.
     Link sizes and axis directions are those of ROS-Industrial's robot description (motoman_gp8_support, BSD licence): S–L offset 0.040, L–U 0.345,
     U–R offset 0.040, U–B 0.340, B–flange 0.080 m; reach 0.040 + 0.345 + √(0.340² + 0.040²) = 0.727 m, as the data sheet says.
     The pendant shows (S, L, U, R, B, T); the DH angles are θ = (S, L − 90°, −U, −R, −B, −T) (fromY / toY; tests.js checks the DH chain against
     the description's own chain `links`). Home (pendant all zero): upper arm vertical, forearm horizontal, flange forward = θ (0, −90°, 0, 0, 0, 0),
     which is a wrist singularity (θ5 = 0); `zero` is the pose of the pictures, B = −90° (flange down, θ5 = 90°). `lim`: the joint ranges in DH degrees */
  gp8: { name: 'GP8', base: 0.330, zero: [0, -H, 0, 0, H, 0], home: [0, -H, 0, 0, 0, 0], flange: 0.080,
    rows: () => [row(0, 0, 0, 0, 'R'), row(-H, 0.04, 0, 0, 'R'), row(0, 0.345, 0, 0, 'R'), row(-H, 0.04, 0.34, 0, 'R'), row(H, 0, 0, 0, 'R'), row(-H, 0, 0, 0, 'R')],
    sym: [['0', '0', '0', 'θ₁'], ['−90°', '0.040', '0', 'θ₂'], ['0', '0.345', '0', 'θ₃'], ['−90°', '0.040', '0.340', 'θ₄'], ['90°', '0', '0', 'θ₅'], ['−90°', '0', '0', 'θ₆']],
    symL: [['0', '0', '0', 'θ₁'], ['−90°', 'a₁', '0', 'θ₂'], ['0', 'a₂', '0', 'θ₃'], ['−90°', 'a₃', 'd₄', 'θ₄'], ['90°', '0', '0', 'θ₅'], ['−90°', '0', '0', 'θ₆']],
    a1: 0.04, a2: 0.345, a3: 0.04, d4: 0.34, q: [rad(30), rad(-60), rad(20), rad(40), rad(50), rad(-30)],
    lim: [[-170, 170], [-155, 55], [-190, 70], [-190, 190], [-135, 135], [-360, 360]],
    fromY: y => [y[0], y[1] - H, -y[2], -y[3], -y[4], -y[5]], toY: q => [q[0], q[1] + H, -q[2], -q[3], -q[4], -q[5]],
    /* the description's own chain: the frame of each link (base_link, link_1_s … link_6_t) in the world for pendant angles y: the meshes of the
       real robot are drawn in these frames (IRE_Slides/build/coppelia), and the DH table is tested against them */
    links: y => { const Tr = (x, yy, z) => RB.trans(x, yy, z), R = (ax, a) => RB.T(ax === 'x' ? RB.Rx(a) : ax === 'y' ? RB.Ry(a) : RB.Rz(a)), L = [RB.trans(0, 0, 0)];
      L.push(RB.Tm(L[0], Tr(0, 0, 0.33), R('z', y[0]))); L.push(RB.Tm(L[1], Tr(0.04, 0, 0), R('y', y[1]))); L.push(RB.Tm(L[2], Tr(0, 0, 0.345), R('y', -y[2])));
      L.push(RB.Tm(L[3], Tr(0.34, 0, 0.04), R('x', -y[3]))); L.push(RB.Tm(L[4], R('y', -y[4]))); L.push(RB.Tm(L[5], R('x', -y[5]))); return L; },
    /* the kinematic equations of the six-axis arm, worked the way Craig 3.7 works the PUMA 560: ^4_6T, ^3_6T, ^1_3T, ^1_6T, then ^0_6T.
       Every entry below is written out on the slides; tests.js checks it against the product of the six link transforms */
    T46: q => { const [, , , , t5, t6] = q, c5 = c(t5), s5 = s(t5), c6 = c(t6), s6 = s(t6); return [[c5 * c6, -c5 * s6, -s5, 0], [s6, c6, 0, 0], [s5 * c6, -s5 * s6, c5, 0], [0, 0, 0, 1]]; },
    T36: q => { const [, , , t4, t5, t6] = q, c4 = c(t4), s4 = s(t4), c5 = c(t5), s5 = s(t5), c6 = c(t6), s6 = s(t6), G = CH3.EX.gp8;
      return [[c4 * c5 * c6 - s4 * s6, -c4 * c5 * s6 - s4 * c6, -c4 * s5, G.a3], [s5 * c6, -s5 * s6, c5, G.d4], [-s4 * c5 * c6 - c4 * s6, s4 * c5 * s6 - c4 * c6, s4 * s5, 0], [0, 0, 0, 1]]; },
    T13: q => { const [, t2, t3] = q, c2 = c(t2), s2 = s(t2), c23 = c(t2 + t3), s23 = s(t2 + t3), G = CH3.EX.gp8; return [[c23, -s23, 0, G.a1 + G.a2 * c2], [0, 0, 1, 0], [-s23, -c23, 0, -G.a2 * s2], [0, 0, 0, 1]]; },
    T06: q => { const [t1, t2, t3, t4, t5, t6] = q, G = CH3.EX.gp8, c1 = c(t1), s1 = s(t1), c2 = c(t2), s2 = s(t2), c23 = c(t2 + t3), s23 = s(t2 + t3), c4 = c(t4), s4 = s(t4), c5 = c(t5), s5 = s(t5), c6 = c(t6), s6 = s(t6);
      const r11 = c23 * (c4 * c5 * c6 - s4 * s6) - s23 * s5 * c6, r21 = -s4 * c5 * c6 - c4 * s6, r31 = -s23 * (c4 * c5 * c6 - s4 * s6) - c23 * s5 * c6;
      const r12 = -c23 * (c4 * c5 * s6 + s4 * c6) + s23 * s5 * s6, r22 = s4 * c5 * s6 - c4 * c6, r32 = s23 * (c4 * c5 * s6 + s4 * c6) + c23 * s5 * s6;
      const r13 = -c23 * c4 * s5 - s23 * c5, r23 = s4 * s5, r33 = s23 * c4 * s5 - c23 * c5, px = G.a1 + G.a2 * c2 + G.a3 * c23 - G.d4 * s23, pz = -G.a2 * s2 - G.a3 * s23 - G.d4 * c23;   // ^1_6T: columns of the frame and the position in {1}
      return [[c1 * r11 - s1 * r21, c1 * r12 - s1 * r22, c1 * r13 - s1 * r23, c1 * px], [s1 * r11 + c1 * r21, s1 * r12 + c1 * r22, s1 * r13 + c1 * r23, s1 * px], [r31, r32, r33, pz], [0, 0, 0, 1]]; } }
};
/* the frames of an example in world coordinates: {0} placed by T0 (default: identity) */
CH3.frames = (rows, q, conv, T0) => { const F = RB.fkDH(rows, q, conv); return T0 ? F.map(T => RB.Tm(T0, T)) : F; };

/* the four elementary steps of one link transform (Craig 3.5): {i−1} → {R} (turn α about X) → {Q} (move a along X) → {P} (turn θ about Z) → {i} (move d along Z).
   u = 0…4 gives the frame part-way; returns the frames passed so far */
CH3.steps4 = (al, a, th, d, u = 4) => { const f = k => Math.max(0, Math.min(1, u - k)), R = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
  const A = RB.Tm(R, RB.T(RB.Rx(al * f(0)))), B = RB.Tm(A, RB.trans(a * f(1), 0, 0)), Cc = RB.Tm(B, RB.T(RB.Rz(th * f(2)))), D = RB.Tm(Cc, RB.trans(0, 0, d * f(3))); return [R, A, B, Cc, D]; };

/* ---------- pages only ---------- */
if (typeof document !== 'undefined') {
  const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
  /* "try it first" for a DH table: every cell is a drop-down. o = { cols (header html), ans: rows of strings, opts: the choices, rowLab (html per row), onOk } */
  CH3.dhAsk = (el, o) => { const cols = o.cols || CH3.COLS, opts = ['—', ...o.opts];
    el.classList.add('dhask');
    el.innerHTML = `<div class="asklab">${T('ลองเติมตารางก่อน:', 'Fill in the table first:')}</div><table class="dhtab"><tr><th>i</th>${cols.map(h => `<th>${h}</th>`).join('')}</tr>` +
      o.ans.map((r, i) => `<tr><td>${o.rowLab ? o.rowLab[i] : i + 1}</td>${r.map((_, j) => `<td><select aria-label="row ${i + 1} column ${j + 1}">${opts.map(x => `<option>${x}</option>`).join('')}</select></td>`).join('')}</tr>`).join('') +
      `</table><div class="pickbar"><button type="button" class="btn on">${T('ตรวจ', 'Check')}</button><span class="askres"></span></div>`;
    const sel = [...el.querySelectorAll('select')], res = el.querySelector('.askres'), m = o.ans[0].length;
    el.querySelector('button').addEventListener('click', () => { let ok = 0, blank = 0; sel.forEach((x, k) => { const want = o.ans[Math.floor(k / m)][k % m], g = x.value === want; if (x.value === '—') blank++; if (g) ok++; x.classList.toggle('ok', g); x.classList.toggle('no', !g); });
      if (ok === sel.length) { res.innerHTML = `<span class="pill good">✓</span> ${T('ถูกทุกช่อง!', 'Every cell is right!')}`; if (o.onOk) o.onOk(); return; }
      res.innerHTML = `<span class="pill bad">${ok} / ${sel.length}</span> ` + (blank ? T('ยังมีช่องว่าง', 'some cells are still empty') : (o.check ? o.check(sel.map(x => x.value)) || '' : '') || T('ช่องสีแดงยังผิด เปิดเฉลยทีละขั้นได้', 'the red cells are wrong; the solution opens step by step')); });
    return { reset() { sel.forEach(x => { x.value = '—'; x.classList.remove('ok', 'no'); }); res.innerHTML = ''; }, fill(rows) { sel.forEach((x, k) => { x.value = rows[Math.floor(k / m)][k % m]; }); } }; };
  /* a DH table for reading (html), optionally with one row highlighted */
  CH3.dhTable = (sym, o = {}) => `<table class="dhtab">` + `<tr><th>i</th>${(o.cols || CH3.COLS).map(h => `<th>${h}</th>`).join('')}</tr>` + sym.map((r, i) => `<tr class="${o.hl === i ? 'hl' : ''}"><td>${o.rowLab ? o.rowLab[i] : i + 1}</td>${r.map(x => `<td>${x}</td>`).join('')}</tr>`).join('') + '</table>';
}
CH3.FIGS = {};   // every figure of chapter 3 is a 3-D render (IRE_Slides/build/coppelia/gen_scenes_ch3.mjs)
global.CH3 = CH3;
})(typeof window !== 'undefined' ? window : globalThis);
