/* ch6_3d.js — the 3-D scenes of chapter 6 (velocity of rigid bodies, velocity propagation, the Jacobian in a frame and in explicit form).
   CH6.S3.name(state, {dark}) returns a scene table (scene3d.js); the pages draw them with CH2.sim / CH2.view (ch2_3d.js looks scenes up in CH6.S3 too),
   and IRE_Slides/build/coppelia/gen_scenes_ch6.mjs renders the same tables in CoppeliaSim for the slides.
   Every scene leaves what it drew in s.info so that a page can print the same numbers under the picture.
   Needs robot.js, ch2.js … ch6.js, scene3d.js, ch2_3d.js, ch3_3d.js. */
(function (global) {
'use strict';
const RB = global.RB, S3 = global.S3, CH2 = global.CH2, CH3 = global.CH3, CH4 = global.CH4, CH5 = global.CH5, CH6 = global.CH6, C = S3.C, { sin, cos, PI, abs, max, min, hypot, sign } = Math, rad = RB.rad, v = S3.v, OV = S3.OV;
const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
const { bar, cyl, sph, box, arrow, dashed, floor, lab, scene, pedestal, tube } = S3;
const { frame, turn, part, cam, camFit } = CH2.H;
const SC = CH6.S3 = {}, P = T4 => RB.Tp(T4), col = (T4, j) => RB.col(RB.TR(T4), j), len = a => hypot(a[0], a[1], a[2]), mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
CH6.OV = OV;   // the step overlays live in scene3d.js (shared by every chapter)

/* ---- the velocity of a point Q of a moving body {B}, watched from the fixed frame {A} (Craig 5.3, figures 5.2–5.4; old part I slides 4–7).
   st: mode ('trans': the body slides and Q crawls on it | 'rot': the body only turns | 'both'), th (angle of {B} about Z_A), vB, vQ, W (rad/s about Z_A),
   Q (in {B}), O (origin of {B}), vs (metres of arrow per m/s), parts (false: only the sum). The parts are drawn tip to tail at Q:
   ^AV_BORG orange, ^A_BR ^BV_Q purple, ^AΩ_B × ^AQ pink; their sum ^AV_Q green ---- */
SC.rigid = (st = {}, o = {}) => { const X = CH6.EX.rigid, mode = st.mode || 'both', th = st.th === undefined ? X.th : st.th, R = RB.Rz(th), O = st.O || X.O, Q = st.Q || X.Q, vs = st.vs || 1.2;
  const vB = mode === 'rot' ? [0, 0, 0] : (st.vB || X.vB), vQ = mode === 'rot' ? [0, 0, 0] : (st.vQ || X.vQ), wz = mode === 'trans' ? 0 : (st.W === undefined ? X.W[2] : st.W), W = [0, 0, wz], r = CH6.vQ(R, vB, vQ, W, Q), Ob = [O[0], O[1], 0.02], Qw = v(Ob, r.rQ);
  const s = scene(st.name || 'rigid', 1300, 1000, cam(st, [0.3, 0.22, 0.08], 3.1, -62, 34), o); floor(s, 1.2, [0.3, 0.2]);
  frame(s, RB.I3(), [0, 0, 0.004], { name: 'A', L: 0.36, tag: true, labels: false, ts: 34, tagAt: { dx: -28, dy: 36 } });
  part(s, R, v(Ob, RB.mv(R, [0.17, 0.06, 0])), [0.5, 0.3, 0.02], s.dark ? [0.42, 0.5, 0.66] : [0.72, 0.78, 0.88], 0.55); frame(s, R, Ob, { name: 'B', L: 0.26, tag: true, labels: false, ts: 34, tagAt: { dx: -24, dy: -22 } });
  dashed(s, Ob, Qw, s.pal.dash, 0.003, 13); lab(s, mid(Ob, Qw), 'Q', { pre: 'B', m: 1, s: 28, c: 'mut', a: 'c', dy: 30 }); sph(s, Qw, 0.02, s.pal.ink); lab(s, Qw, 'Q', { m: 1, s: 34, dx: -18, dy: -14, a: 'r' });
  if (abs(wz) > 1e-9) { turn(s, v(Ob, [0, 0, 1], 0.14), [0, 0, sign(wz)], 0.07, C.pk, 0.5, 0.5 + min(5, 1.5 + 2.5 * abs(wz)), 0.0045); arrow(s, Ob, v(Ob, [0, 0, 1], 0.3 * sign(wz)), C.pk, 0.0045); lab(s, v(Ob, [0, 0, 1], 0.3 * sign(wz)), 'Ω', { pre: 'A', sub: 'B', m: 1, c: 'pk', s: 30, dx: 16, dy: -4 }); }
  const A = v(Qw, [0, 0, 1], 0.012), parts = [[r.trans, C.o, 'o', 'V', { pre: 'A', sub: 'BORG' }], [r.rel, C.p, 'p', 'V', { pre: 'B', sub: 'Q' }], [r.rot, C.pk, 'pk', 'Ω × Q', { pre: 'A' }]];   // the middle one is ^BV_Q turned into {A}
  if (st.parts !== false) { let a = A; parts.forEach(([d, c, key, t, lo]) => { if (len(d) < 1e-9) return; const b = v(a, d, vs); arrow(s, a, b, c, 0.0052); lab(s, mid(a, b), t, { ...lo, m: 1, c: key, s: 24, a: 'c', dy: -18 }); a = b; }); }
  if (len(r.v) > 1e-9) { arrow(s, A, v(A, r.v, vs), C.gr, 0.0068); lab(s, v(A, r.v, vs), 'V', { pre: 'A', sub: 'Q', m: 1, c: 'gr', s: 32, dx: 18, dy: -8 }); }
  s.info = r; return s; };

/* ---- the arms of this chapter, each as a DH table with the way it is seen. 'two': Craig's example 5.3 (two turning joints and a frame {3} at the tip);
   'rrp': the arm of homework 4 (the class exercise); 'stan': the Stanford arm with all six joints; the others are chapter 3's (CH3.EX) ---- */
const ARM = { two: { T0: [0, 0, 0.11], cam: [[0.35, 0.3, 0.05], 3.0, -90, 58], grip: null, base: true }, rrp: { T0: [0, 0, 0], cam: [[0.3, 0.12, 0.42], 3.3, -58, 16], grip: 'z', base: false },
  stan: { T0: [0, 0, 0.45], cam: [[0.2, 0.15, 0.42], 3.7, -52, 20], grip: 'z', base: true } };
const armOf = CH6.armOf = (ex, q) => { const mk = (rows, V) => ({ rows, F: CH3.frames(rows, q, null, RB.trans(...V.T0)), V });
  if (ex === 'two') return mk(CH6.two.rows(CH6.two.L), ARM.two); if (ex === 'rrp') return mk(CH4.rrp.rows(CH4.rrp.L), ARM.rrp); if (ex === 'stan') return mk(CH6.stan.rows(CH6.stan.L2), ARM.stan);
  const r = CH3.exFrames(ex, q); return { ...r, V: CH3.EXV[ex] }; };
const defQ = ex => ex === 'two' ? CH6.two.q : ex === 'rrp' ? CH4.rrp.q : ex === 'stan' ? CH6.stan.q : CH3.EX[ex].q;
const sceneOf = (st, name, ex, q, extra, o) => { const A = armOf(ex, q), V = A.V, c0 = st.cam || V.cam, w = st.w || 1300, h = st.h || 1000, pts = st.fit ? A.F.map(P).concat(extra || [], [[V.T0[0] - 0.15, V.T0[1], 0], [V.T0[0] + 0.15, V.T0[1], 0]]) : null;
  const s = scene(st.name || name, w, h, pts ? camFit(st, pts, c0[2], c0[3], w, h, { m: 0.2, minD: 2.4 }) : cam(st, c0[0], c0[1], c0[2], c0[3]), o); floor(s, 1.4, [c0[0][0], c0[0][1]]); if (V.base) pedestal(s, [V.T0[0], V.T0[1]], V.T0[2] - 0.02, 0.09); return { s, ...A }; };

/* ---- velocity propagation (Craig 5.6; old part I slides 8–23): the velocity of every link frame, passed on from the base to the tip.
   At the origin of frame {i}: its linear velocity v_i (green) and its angular velocity ω_i (purple).
   A planar arm turns about the vertical only, so its ω is drawn as a turning arrow about the joint instead of an arrow along Z (which a view from above hides).
   st: ex, q, qd, upto (only frames from…upto have their velocities drawn: the page's steps walk up the arm; from defaults to 1), show ('both' | 'v' | 'w'), frames (which frames have
   their axes drawn), comp (0 or the last frame's number: the tip velocity split along that frame's axes, the two ways the answer is written),
   vs (m of arrow per m/s), ws (m per rad/s) ---- */
SC.prop = (st = {}, o = {}) => { const ex = st.ex || 'two', q = st.q || defQ(ex), X = CH6.EX[ex === 'rpr' ? 'hw' : ex] || {}, qd = st.qd || X.qd || q.map(() => 0), vs = st.vs || 0.8, ws = st.ws || 0.18, show = st.show || 'both';
  const A0 = armOf(ex, q), Pr = CH6.prop(A0.rows, q, qd), n = A0.F.length - 1, upto = st.upto === undefined ? n : st.upto, from = st.from || 1, flat = ex === 'two' || ex === 'p3r', up = flat ? 0.06 : 0, lift = p => [p[0], p[1], p[2] + up], ends = [];
  for (let i = from; i <= upto; i++) { const O = lift(P(A0.F[i])); ends.push(v(O, Pr[i].v0, vs)); if (show !== 'v' && !flat) ends.push(v(O, Pr[i].w0, ws)); }
  const { s, rows, F, V } = sceneOf(st, 'prop_' + ex, ex, q, ends, o);
  CH3.drawDH(s, rows, F, { frames: st.frames || [0, n], grip: V.grip, soft: V.soft, q, fl: 0.14, ls: 24, labAt: st.labAt });
  for (let i = from; i <= upto; i++) { const O = lift(P(F[i])), pv = Pr[i].v0, pw = Pr[i].w0, last = i === upto;
    if (show !== 'w' && len(pv) > 1e-9) { arrow(s, O, v(O, pv, vs), C.gr, last ? 0.0065 : 0.0045, last ? 1 : 0.55); lab(s, v(O, pv, vs), 'v', { sub: String(i), m: 1, c: 'gr', s: last ? 30 : 24, dx: 14, dy: -8 }); }
    if (show !== 'v' && len(pw) > 1e-9) { if (flat) { const r = 0.085; turn(s, [O[0], O[1], O[2] + 0.03], [0, 0, sign(pw[2])], r, C.p, 0.6, 0.6 + min(5, 1.3 + 2.4 * abs(pw[2])), last ? 0.0048 : 0.0036); lab(s, [O[0] - r, O[1], O[2] + 0.03], 'ω', { sub: String(i), m: 1, c: 'p', s: last ? 30 : 24, a: 'r', dx: -14, dy: 8 }); }
      else { arrow(s, O, v(O, pw, ws), C.p, last ? 0.0058 : 0.004, last ? 1 : 0.55); lab(s, v(O, pw, ws), 'ω', { sub: String(i), m: 1, c: 'p', s: last ? 30 : 24, dx: 14, dy: 12 }); } } }
  if (st.comp !== undefined && len(Pr[n].v0) > 1e-9) { const O = lift(P(F[n])), Fr = F[st.comp], pv = Pr[n].v0; let a = O; [0, 1, 2].forEach(j => { const d = col(Fr, j), k = RB.dot(pv, d); if (abs(k) < 1e-6) return; const b = v(a, d, k * vs); dashed(s, a, b, [C.x, C.y, s.pal.z][j], 0.0035, 9);
      lab(s, mid(a, b), 'v', { pre: String(st.comp), sub: 'xyz'[j], m: 1, c: 'xyz'[j], s: 22, a: 'c', dy: j ? 20 : -16 }); a = b; }); }
  if (st.xyz) s.tip = CH3.tipXYZ(s, F, st.xyzAt); s.F = F; s.info = { prop: Pr, n }; return s; };

/* ---- the Jacobian in explicit form (old part II; Khatib): one column per joint, read off the picture. Every joint axis Z_i is drawn (blue);
   for the chosen joint st.j the vector from its axis to the tip (dashed, P_jn = p_n − p_j) and, at the tip, the column's linear part:
   Z_j × P_jn for a turning joint, Z_j itself for a sliding one (pink), with its angular part Z_j (purple, turning joints only).
   st: ex, q, j (0: the axes only), js (m of arrow per unit of the column), axes (false: only the chosen joint's axis) ---- */
SC.explicit = (st = {}, o = {}) => { const ex = st.ex || 'stan', q = st.q || defQ(ex), A0 = armOf(ex, q), Cs = CH6.cols(A0.rows, q), j = st.j === undefined ? 1 : st.j, js = st.js || 0.5, n = A0.F.length - 1, tip0 = P(A0.F[n]), T0 = A0.V.T0, c = j ? Cs[j - 1] : null;
  const { s, rows, F, V } = sceneOf(st, 'explicit_' + ex, ex, q, c ? [v(tip0, c.lin, js)] : [], o), wp = p => [p[0] + T0[0], p[1] + T0[1], p[2] + T0[2]], tip = P(F[n]);
  CH3.drawDH(s, rows, F, { frames: st.frames || [0], grip: V.grip, soft: V.soft, q, fl: 0.14, ls: 24 });
  Cs.forEach(cc => { const on = c && cc.k === c.k; if (st.axes === false && !on) return; const O = wp(cc.p), L = on ? 0.3 : 0.2; arrow(s, v(O, cc.z, on ? -0.12 : 0), v(O, cc.z, L), s.pal.z, on ? 0.0055 : 0.0035, on ? 1 : 0.6); lab(s, v(O, cc.z, L), 'Z', { sub: String(cc.k), m: 1, c: 'z', s: on ? 30 : 22, dx: 14, dy: -6 }); });
  if (c) { const O = wp(c.p); if (c.kind === 'R' && len(c.arm) > 1e-6) { dashed(s, O, tip, C.am, 0.0035, 15); lab(s, mid(O, tip), 'P', { sub: c.k + 'n', m: 1, c: 'am', s: 26, a: 'c', dy: 26 }); }
    if (len(c.lin) > 1e-9) { arrow(s, tip, v(tip, c.lin, js), C.pk, 0.0065); lab(s, v(tip, c.lin, js), 'Z', { sub: String(c.k), post: c.kind === 'R' ? ' × P' : '', m: 1, c: 'pk', s: 28, dx: 16, dy: -10 }); }
    if (len(c.ang) > 1e-9 && st.ang !== false) arrow(s, tip, v(tip, c.ang, 0.16), C.p, 0.0045); }
  if (st.xyz) s.tip = CH3.tipXYZ(s, F, st.xyzAt); s.F = F; s.info = { cols: Cs, J0: CH5.J0(rows, q) }; return s; };
})(typeof window !== 'undefined' ? window : globalThis);
