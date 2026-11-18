/* ch7_3d.js — the 3-D scenes of chapter 7 (trajectory generation). CH7.S3.name(state, {dark}) returns a scene table (scene3d.js);
   the pages draw them with CH2.sim / CH2.view (ch2_3d.js looks scenes up in CH7.S3 too) beside the plots of CH7.plots, and
   IRE_Slides/build/coppelia/gen_scenes_ch7.mjs renders the same tables in CoppeliaSim for the slides (the plots there are native PowerPoint charts).
   Needs robot.js, ch2.js … ch5.js, ch7.js, scene3d.js, ch2_3d.js, ch3_3d.js. */
(function (global) {
'use strict';
const RB = global.RB, S3 = global.S3, CH2 = global.CH2, CH5 = global.CH5, CH7 = global.CH7, C = S3.C, { sin, cos, PI, abs, hypot } = Math, rad = RB.rad, v = S3.v, OV = S3.OV;
const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
const { bar, cyl, sph, arrow, dashed, floor, lab, scene, harm, tube, mesh } = S3;
const { cam, camFit } = CH2.H;
const SC = CH7.S3 = {}, Z0 = 0.11;
CH7.OV = OV;   // the step overlays live in scene3d.js (shared by every chapter)
const planCam = (st, pts, w, h, fx) => st.fit === false ? cam(st, fx[0], fx[1], -90, fx[2] || 62) : camFit(st, pts.map(p => [p[0], p[1], Z0]).concat([[-0.16, 0, 0], [0.16, 0, 0], [0, -0.16, 0], [0, 0.16, 0]]), -90, fx[2] || 62, w, h, { m: fx[3] || 0.17, minD: 2.2 });
const xyz = (s, st, O, tip) => { if (st.xyz) s.tip = S3.xyz(s, O, tip, { head: T('ปลายแขน เทียบฐาน', 'end of arm, from the base'), axes: 0.24, axIdx: [0, 1] }); };

/* ---- one turning joint (Craig example 7.1: "a single-link robot with a rotary joint"). st: th (now, rad), th0, thf (rad; ghosts at the start and the goal),
   marks (more angles to show as ghosts, e.g. via points), L (link length), name (false: no θ label) ---- */
SC.joint1 = (st = {}, o = {}) => { const L = st.L || 0.5, th = st.th === undefined ? rad(15) : st.th, w = st.w || 1100, h = st.h || 1000, ends = [st.th0, st.thf, ...(st.marks || [])].filter(x => x !== undefined);
  const s = scene(st.name || 'joint1', w, h, planCam(st, [[L, 0], [0, L], [-L, 0], [0, -L]].map(p => p).concat(ends.map(a => [L * cos(a), L * sin(a)])), w, h, [[0.05, 0.1, 0.05], 2.6, 62, 0.12]), o); floor(s, 1.0);
  ends.forEach(a => harm(s, [L], [a], { al: 0.25, grip: false }));
  const arm = harm(s, [L], [th]), O = [0, 0, Z0 + 0.075];
  OV.ref(s, O, [1, 0, 0], L * 0.75); if (abs(th) > 0.04) OV.ang(s, O, [1, 0, 0], [cos(th), sin(th), 0], 0.18, 'pk', st.label === false ? null : 'θ', { dx: 16, dy: -8 });
  [[st.th0, 'θ', '0'], [st.thf, 'θ', 'f']].forEach(([a, t, sub]) => { if (a !== undefined) lab(s, [L * 1.08 * cos(a), L * 1.08 * sin(a), Z0], t, { sub, m: 1, c: 'mut', s: 26, a: 'c' }); });
  xyz(s, st, arm.pts[0], arm.pts[1]); s.info = { th }; return s; };

/* ---- the two-link arm with the path of its tool point (Craig 7.2–7.5). st: q (now), L, trace (the tool path so far or all of it: [[x, y]…]), traceKey (colour key),
   trace2 (a second path, drawn faint: the other scheme), line ([A, B]: the straight line, dashed), pts (path points [[x, y]…] as dots), ghosts ([q…]),
   ring (true: the workspace, inner and outer edge), bad ([[x, y]…]: points out of reach, red), xyz,
   keep ([[x, y]…]: more points the camera keeps in view: pass every elbow and tool point of a motion that plays, so that the camera stays still) ---- */
const COL = { gr: C.gr, b: C.b, o: C.o, p: C.p, pk: C.pk, am: C.am, rd: C.rd };
SC.arm2 = (st = {}, o = {}) => { const L = st.L || CH5.two.L, q = st.q || CH7.EX.two.qA, w = st.w || 1200, h = st.h || 1000, R = L[0] + L[1], r = abs(L[0] - L[1]), z = Z0 - 0.02;
  const f = RB.fk2(L[0], L[1], q[0], q[1]), keep = [[0, 0], [f.x1, f.y1], [f.x, f.y], ...(st.trace || []), ...(st.trace2 || []), ...(st.line || []), ...(st.pts || []), ...(st.keep || [])];
  const s = scene(st.name || 'arm2', w, h, st.ring && st.fit === false ? cam(st, [0.05, 0.05, 0.05], 4.8, -90, 62) : planCam(st, st.ring ? keep.concat([[R, 0], [-R, 0], [0, R], [0, -R]]) : keep, w, h, [[0.2, 0.3, 0.05], 3.4]), o); floor(s, 1.4);
  if (st.ring) { mesh(s, Array.from({ length: 90 }, (_, i) => { const a = 2 * PI * i / 90, b = 2 * PI * (i + 1) / 90; return [[r * cos(a), r * sin(a), z], [R * cos(a), R * sin(a), z], [R * cos(b), R * sin(b), z], [r * cos(b), r * sin(b), z]]; }), C.b, 0.12);
    tube(s, RB.ring([0, 0, z], [0, 0, 1], R, 0, 2 * PI, 90), s.L[2], 0.003); tube(s, RB.ring([0, 0, z + 0.001], [0, 0, 1], r, 0, 2 * PI, 40), C.pk, 0.003); }
  (st.ghosts || []).forEach(g => harm(s, L, g, { al: 0.25, grip: false }));
  if (st.line) dashed(s, [st.line[0][0], st.line[0][1], Z0], [st.line[1][0], st.line[1][1], Z0], s.pal.dash, 0.003, 31);
  if (st.trace2 && st.trace2.length > 1) tube(s, st.trace2.map(p => [p[0], p[1], Z0 + 0.002]), COL[st.trace2Key] || s.pal.dash, 0.0035, false, 0.55);
  if (st.trace && st.trace.length > 1) tube(s, st.trace.map(p => [p[0], p[1], Z0 + 0.004]), COL[st.traceKey] || C.gr, 0.0055);
  (st.bad || []).forEach(p => sph(s, [p[0], p[1], Z0 + 0.004], 0.008, C.rd));
  (st.pts || []).forEach((p, i) => { sph(s, [p[0], p[1], Z0 + 0.006], 0.016, s.pal.ink); if (st.ptNames) lab(s, [p[0], p[1], Z0], st.ptNames[i], { m: 1, s: 30, dx: 16, dy: -12 }); });
  const arm = harm(s, L, q, { grip: false }); xyz(s, st, arm.pts[0], arm.pts[2]); s.info = { p: [f.x, f.y] }; return s; };
})(typeof window !== 'undefined' ? window : globalThis);
