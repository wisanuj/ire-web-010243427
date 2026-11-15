/* scene3d.js — builds 3-D scenes as plain lists of primitives (bars, cylinders, boxes, arrows, tubes, meshes) and labels.
   One scene table has two renderers: gl3d.js draws it live in the browser (WebGL), and IRE_Slides/build/coppelia/render.lua
   draws it in CoppeliaSim for the slide figures, so the website and the slides show the same picture.
   Works in the browser (window.S3) and in deno (globalThis.S3). Needs robot.js. World: metres, z up.
   Camera convention (measured on CoppeliaSim's vision sensor): the view angle spans the larger side of the picture. */
(function (global) {
'use strict';
const RB = global.RB, { sin, cos, PI, hypot, tan, max } = Math, rad = RB.rad, un = RB.unit, cr = RB.cross;
const S3 = {};
const C = S3.C = { o: [0.91, 0.35, 0.05], b: [0.12, 0.44, 0.70], p: [0.48, 0.25, 0.71], g: [0.45, 0.50, 0.58], d: [0.17, 0.21, 0.28], x: [0.84, 0.16, 0.16], y: [0.17, 0.58, 0.28], z: [0.11, 0.31, 0.85],
  pk: [0.76, 0.21, 0.44], gr: [0.11, 0.54, 0.29], am: [0.90, 0.62, 0.10], lt: [0.78, 0.81, 0.85], k: [0.10, 0.12, 0.16], rd: [0.75, 0.22, 0.17] };
/* colours that depend on the background: light for the slides and the light reading themes, dark for the dark theme */
S3.PAL = { light: { ink: C.k, joint: [0.2, 0.25, 0.33], sleeve: [0.27, 0.33, 0.42], base: C.g, dash: C.g, cap: C.lt, shadow: 0.14, bg: [1, 1, 1], z: C.z, link: [C.g, C.o, C.b, C.p] },
  dark: { ink: [0.91, 0.93, 0.97], joint: [0.62, 0.67, 0.77], sleeve: [0.52, 0.58, 0.69], base: [0.42, 0.47, 0.58], dash: [0.62, 0.66, 0.8], cap: [0.92, 0.94, 0.97], shadow: 0.5, bg: [0.059, 0.071, 0.125], floor: [0.1, 0.12, 0.2], z: [0.42, 0.66, 1], link: [[0.42, 0.47, 0.58], [1, 0.6, 0.25], [0.3, 0.72, 0.95], [0.72, 0.52, 0.92]] } };
const v = S3.v = (a, b, k = 1) => [a[0] + b[0] * k, a[1] + b[1] * k, a[2] + b[2] * k];
S3.camAt = (tgt, dist, az, el, fov = 28) => ({ tgt, fov, dist, az, el, eye: [tgt[0] + dist * cos(rad(el)) * cos(rad(az)), tgt[1] + dist * cos(rad(el)) * sin(rad(az)), tgt[2] + dist * sin(rad(el))] });
/* a camera that keeps every point of pts in a W × H picture (seen from az, el), with a margin m in metres, for arms whose joints the
   reader can move anywhere: centred on what is there, as close as it fits. st (the page state) keeps the last framing in st._fit and
   eases towards the new one: out quickly (nothing may leave the picture), in slowly and only for a clear gain (no breathing while dragging) */
S3.fitCam = (st, pts, az, el, fov, W, H, o = {}) => { const e = [cos(rad(el)) * cos(rad(az)), cos(rad(el)) * sin(rad(az)), sin(rad(el))], z = [-e[0], -e[1], -e[2]], x = un(cr([0, 0, 1], z)), y = cr(z, x), m = o.m === undefined ? 0.2 : o.m;
  const P = pts.map(p => [dot(p, x), dot(p, y), dot(p, z)]), lo = k => Math.min(...P.map(q => q[k])), hi = k => max(...P.map(q => q[k])), hu = (hi(0) - lo(0)) / 2 + m, hv = (hi(1) - lo(1)) / 2 + m, cu = (hi(0) + lo(0)) / 2, cv = (hi(1) + lo(1)) / 2, cz = (hi(2) + lo(2)) / 2;
  const tn = tan(rad(fov) / 2), d1 = max(o.minD || 0, (W >= H ? max(hu, hv * W / H) : max(hu * H / W, hv)) / tn + (hi(2) - lo(2)) / 2), t1 = [0, 1, 2].map(k => cu * x[k] + cv * y[k] + cz * z[k]);
  let f = st && st._fit; if (!f || o.snap) f = { t: t1, d: d1 }; else { const kd = d1 > f.d ? 0.35 : d1 < 0.8 * f.d ? 0.08 : 0; f = { t: f.t.map((a, k) => a + (t1[k] - a) * 0.2), d: f.d + (d1 - f.d) * kd }; if (d1 > f.d) f.d = max(f.d, d1 * 0.97); }
  if (st) st._fit = f; return S3.camAt(f.t, f.d, az, el, fov); };
S3.scene = (name, w, h, cam, o = {}) => { const pal = S3.PAL[o.dark ? 'dark' : 'light']; return { name, ow: w, oh: h, cam, items: [], labels: [], pal, dark: !!o.dark, noShadow: !!o.noShadow, L: pal.link }; };
/* camera axes: z looks at the target, x points to the left of the picture, y up */
S3.axes = cam => { const z = un([cam.tgt[0] - cam.eye[0], cam.tgt[1] - cam.eye[1], cam.tgt[2] - cam.eye[2]]), x = un(cr([0, 0, 1], z)); return { x, y: cr(z, x), z }; };
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
S3.focal = (cam, W, H) => max(W, H) / 2 / tan(rad(cam.fov) / 2);
/* pixel position of a 3-D point in a picture of W × H px */
S3.proj = (cam, W, H, p) => { const A = S3.axes(cam), d = [p[0] - cam.eye[0], p[1] - cam.eye[1], p[2] - cam.eye[2]], f = S3.focal(cam, W, H), zc = dot(d, A.z); return [W / 2 - f * dot(d, A.x) / zc, H / 2 - f * dot(d, A.y) / zc, zc]; };
/* the point of the horizontal plane z = z0 seen at pixel (px, py): for dragging things on the table */
S3.unproject = (cam, W, H, px, py, z0 = 0) => { const A = S3.axes(cam), f = S3.focal(cam, W, H), a = (W / 2 - px) / f, b = (H / 2 - py) / f, d = [A.x[0] * a + A.y[0] * b + A.z[0], A.x[1] * a + A.y[1] * b + A.z[1], A.x[2] * a + A.y[2] * b + A.z[2]];
  if (Math.abs(d[2]) < 1e-9) return null; const t = (z0 - cam.eye[2]) / d[2]; return t > 0 ? [cam.eye[0] + d[0] * t, cam.eye[1] + d[1] * t] : null; };
/* a label at a 3-D point: t text, sub subscript, pre a leading superscript (Craig's frame of reference, ᴬP), post text after the subscript,
   c colour key (ink, mut, o, b, p, x, y, z, pk, gr, am), s size and dx, dy offsets in px of the design width */
S3.lab = (s, p, t, o = {}) => s.labels.push({ p, t, sub: o.sub || '', pre: o.pre || '', post: o.post || '', c: o.c || 'ink', s: o.s || 30, m: o.m ? 1 : 0, a: o.a || 'l', b: o.b === false ? 0 : 1, dx: o.dx || 0, dy: o.dy || 0 });

/* ---- primitives ---- */
const add = S3.add = (s, o) => { s.items.push(o); return o; };
const shadow = S3.shadow = (s, a, b, r) => { if (s.noShadow) return; const f = q => [q[0] + q[2] * 0.22, q[1] + q[2] * 0.3, 0.003], a0 = f(a), b0 = f(b), al = s.pal.shadow;
  if (hypot(b0[0] - a0[0], b0[1] - a0[1]) < 1e-3) add(s, { t: 'cyl', a: [a0[0], a0[1], 0.002], b: [a0[0], a0[1], 0.0045], r: r * 1.15, c: [0, 0, 0], al }); else add(s, { t: 'box', a: a0, b: b0, sx: 2.1 * r, sy: 0.0025, c: [0, 0, 0], al, up: [0, 0, 1] }); };
const bar = S3.bar = (s, a, b, r, c, al) => { add(s, { t: 'bar', a, b, r, c, al }); if (!al || al >= 1) shadow(s, a, b, r); };
const cyl = S3.cyl = (s, a, b, r, c, al) => add(s, { t: 'cyl', a, b, r, c, al });
const sph = S3.sph = (s, p, r, c, al) => add(s, { t: 'sph', p, r, c, al });
const box = S3.box = (s, a, b, sx, sy, c, al, up) => add(s, { t: 'box', a, b, sx, sy, c, al, up });
S3.arrow = (s, a, b, c, r = 0.0055, al) => add(s, { t: 'arrow', a, b, r, c, al });
const tube = S3.tube = (s, pts, c, r = 0.005, head = false, al) => add(s, { t: 'tube', pts, r, c, head, al });
S3.dashed = (s, a, b, c, r = 0.003, n = 11) => { const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]; for (let i = 0; i < n; i += 2) cyl(s, v(a, d, i / n), v(a, d, (i + 1) / n), r, c || s.pal.dash); };
/* the slides have no floor (white page, shadows only); on a dark canvas a floor disc gives the shadows something to fall on */
S3.floor = (s, r, ctr = [0, 0]) => { if (s.dark) cyl(s, [ctr[0], ctr[1], -0.03], [ctr[0], ctr[1], -0.001], r, s.pal.floor); };
S3.arc = (s, o, ax, r, a0, a1, c, tr = 0.0055, head = true, n = 26) => tube(s, RB.ring(o, ax, r, a0, a1, n), c, tr, head);   // a turn about an axis (start direction not controlled)
/* an angle drawn from a0 to a1: arcH in a horizontal plane (from +x towards +y), arcV in the vertical x–z plane (from +x towards +z) */
S3.arcH = (s, o, r, a0, a1, c, tr = 0.005, n = 24) => tube(s, Array.from({ length: n + 1 }, (_, k) => { const a = a0 + (a1 - a0) * k / n; return [o[0] + r * cos(a), o[1] + r * sin(a), o[2]]; }), c, tr, true);
S3.arcV = (s, o, r, a0, a1, c, tr = 0.005, n = 24) => tube(s, Array.from({ length: n + 1 }, (_, k) => { const a = a0 + (a1 - a0) * k / n; return [o[0] + r * cos(a), o[1], o[2] + r * sin(a)]; }), c, tr, true);
const rj = S3.rj = (s, p, ax, o = {}) => { const r = o.r || 0.056, h = o.h || 0.07; cyl(s, v(p, ax, -h), v(p, ax, h), r, o.c || s.pal.joint, o.al); cyl(s, v(p, ax, -h - 0.012), v(p, ax, h + 0.012), r * 0.42, s.pal.cap, o.al); };
S3.pj = (s, p, d, o = {}) => box(s, v(p, d, -(o.len || 0.075)), v(p, d, o.len || 0.075), o.w || 0.085, o.w || 0.085, s.pal.sleeve, o.al, o.up);
const grip = S3.grip = (s, p, d, sd, o = {}) => { const w = o.w || 0.045, L = o.L || 0.085, c = o.c || s.pal.ink, a = v(p, sd, -w), b = v(p, sd, w); box(s, a, b, 0.024, 0.03, c, o.al); [a, b].forEach(q => box(s, q, v(q, d, L), 0.018, 0.03, c, o.al)); };
const pedestal = S3.pedestal = (s, p, h, r = 0.085) => { cyl(s, [p[0], p[1], 0], [p[0], p[1], 0.03], r * 1.7, s.pal.base); cyl(s, [p[0], p[1], 0.03], [p[0], p[1], h], r * 0.75, s.pal.base); shadow(s, [p[0], p[1], 0.02], [p[0], p[1], 0.021], r * 1.7); };
const mesh = S3.mesh = (s, quads, c, al) => { const vv = [], ii = []; quads.forEach(q => { const n = vv.length / 3; q.forEach(p => vv.push(p[0], p[1], p[2])); ii.push(n, n + 1, n + 2, n, n + 2, n + 3); }); if (ii.length) add(s, { t: 'mesh', v: vv, i: ii, c, al }); };
/* a smooth surface: a grid of shared vertices, f(u, w) → point for u, w in 0…1 */
S3.grid = (s, n, m, f, c, al) => { const vv = [], ii = []; for (let a = 0; a <= n; a++) for (let b = 0; b <= m; b++) vv.push(...f(a / n, b / m)); for (let a = 0; a < n; a++) for (let b = 0; b < m; b++) { const k = a * (m + 1) + b; ii.push(k, k + m + 1, k + m + 2, k, k + m + 2, k + 1); } add(s, { t: 'mesh', v: vv, i: ii, c, al }); };
S3.annulus = (s, r0, r1, z, c, al, n = 90) => mesh(s, Array.from({ length: n }, (_, i) => { const a = 2 * PI * i / n, b = 2 * PI * (i + 1) / n; return [[r0 * cos(a), r0 * sin(a), z], [r1 * cos(a), r1 * sin(a), z], [r1 * cos(b), r1 * sin(b), z], [r0 * cos(b), r0 * sin(b), z]]; }), c, al);
S3.circle = (s, r, z, c, tr = 0.004) => tube(s, RB.ring([0, 0, z], [0, 0, 1], r, 0, 2 * PI, 72), c, tr);
/* cells of a planar region as long quads (run-length along x) */
S3.cells = (s, has, R, z, c, al, n = 150) => { const h = 2 * R / n, q = []; for (let j = 0; j < n; j++) { const y = -R + (j + 0.5) * h; let st = null; for (let i = 0; i <= n; i++) { const inside = i < n && has(-R + (i + 0.5) * h, y);
      if (inside && st === null) st = i; if (!inside && st !== null) { q.push([[-R + st * h, y - h / 2, z], [-R + i * h, y - h / 2, z], [-R + i * h, y + h / 2, z], [-R + st * h, y + h / 2, z]]); st = null; } } } mesh(s, q, c, al); };
/* parts of a scene that are costly and change rarely (workspace surfaces): built once per key, then the same item objects are reused,
   which also lets the WebGL renderer keep their buffers */
const CACHE = new Map();
S3.cached = (s, key, build) => { const k = key + (s.dark ? '|d' : '|l'); let it = CACHE.get(k); if (!it) { const t = { ...s, items: [], labels: [] }; build(t); it = t.items; CACHE.set(k, it); if (CACHE.size > 60) CACHE.delete(CACHE.keys().next().value); } it.forEach(o => s.items.push(o)); };

/* a revolute arm lying in a horizontal plane at height z0 (the two- and three-link arm of the chapter) */
S3.harm = (s, L, q, o = {}) => { const z0 = o.z0 || 0.11, al = o.al, cols = o.cols || s.L.slice(1), pts = [[0, 0, z0]]; let th = 0; L.forEach((l, i) => { th += q[i]; const p = pts[i]; pts.push([p[0] + l * cos(th), p[1] + l * sin(th), z0]); });
  if (!al) pedestal(s, [0, 0], z0 - 0.02);
  L.forEach((l, i) => bar(s, pts[i], pts[i + 1], 0.027 - i * 0.003, cols[i], al)); L.forEach((l, i) => rj(s, pts[i], [0, 0, 1], { r: 0.046 - i * 0.004, h: 0.05, al }));
  const d = [cos(th), sin(th), 0], sd = [-sin(th), cos(th), 0]; if (o.grip !== false) grip(s, pts[L.length], d, sd, { al, w: 0.014 + 0.022 * (o.open === undefined ? 1 : o.open), L: 0.065 }); else sph(s, pts[L.length], 0.022, s.pal.ink, al); return { pts, th }; };
/* ---- where the end of an arm is: tip (a world point) in the frame with origin O and axes A = [X, Y, Z] (unit world vectors, default the world axes).
   Marks the tip, draws the dashed path x along X, then y along Y, then z along Z that adds up to it, and writes x, y, z beside the tip under
   the heading o.head (one block of labels: they share the anchor, so the placement moves them together). o.axes: also draw the frame's
   axes that long, named x, y, z (chapter 1, where no frame is drawn yet), o.axIdx which of them (default all three; [0, 1] where z would point at the camera or hide in the robot's column); o.path false: no dashed path; o.dx, o.dy move the block.
   Returns the coordinates [x, y, z]. ---- */
S3.xyz = (s, O, tip, o = {}) => { const A = o.A || [[1, 0, 0], [0, 1, 0], [0, 0, 1]], w = [tip[0] - O[0], tip[1] - O[1], tip[2] - O[2]], p = A.map(e => e[0] * w[0] + e[1] * w[1] + e[2] * w[2]), col = [C.x, C.y, s.pal.z];
  if (o.axes) (o.axIdx || [0, 1, 2]).forEach(j => { const e = A[j]; S3.arrow(s, O, v(O, e, o.axes), col[j], 0.0045); S3.lab(s, v(O, e, o.axes + 0.035), 'xyz'[j], { m: 1, c: 'xyz'[j], s: 28, a: 'c', dy: 10 }); });
  if (o.path !== false) { let a = O; A.forEach((e, j) => { const b = v(a, e, p[j]), d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], L = hypot(d[0], d[1], d[2]), n = Math.max(3, 2 * Math.round(L / 0.05) + 1);
      if (L > 0.012) for (let i = 0; i < n; i += 2) cyl(s, v(a, d, i / n), v(a, d, (i + 1) / n), 0.0032, col[j], 0.85); a = b; }); }
  /* sizes are for a scene 1300 px wide: scaled with the scene, the block takes the same share of every picture */
  sph(s, tip, o.r || 0.018, C.pk); const k = s.ow / 1300, dx = (o.dx === undefined ? 30 : o.dx) * k, dy = (o.dy === undefined ? -64 : o.dy) * k, f3 = x => (x < -5e-4 ? '−' : '') + Math.abs(x).toFixed(3);   // fixed decimals: the numbers do not jump about while a slider moves
  if (o.head) S3.lab(s, tip, o.head, { c: 'mut', s: 19 * k, b: false, a: 'l', dx, dy });
  ['x', 'y', 'z'].forEach((n, j) => S3.lab(s, tip, `${n} = ${f3(p[j])} m`, { c: 'xyz'[j], s: 23 * k, a: 'l', dx, dy: dy + (33 + 33 * j) * k }));
  return p; };
/* ---- label placement for the live views (the slide figures do the same in IRE_Slides/build/coppelia/compose.py) ----
   items: [{x, y, w, size, ax, ay}] — x, y the left end of the baseline the scene asked for, w the width, (ax, ay) the point the label names.
   Each label stays put unless its box covers the picture (pixels that differ from the background) or an earlier label; then the clearest
   nearby spot wins, with a small cost for moving. memo (an array kept by the page state) holds the last offsets so labels do not jump
   from frame to frame. reserved: boxes [x0, y0, x1, y1] the page draws over the picture afterwards (buttons, notes), kept free like placed labels.
   Sets X, Y (where to draw) and lead (moved far: draw a leader line). Browser only. */
let probe = null;
S3.place = (ctx, W, H, items, bg, memo, reserved) => { if (typeof document === 'undefined') return items;
  const k = 3, pw = Math.max(1, Math.ceil(W / k)), ph = Math.max(1, Math.ceil(H / k)); if (!probe) probe = document.createElement('canvas'); probe.width = pw; probe.height = ph;
  const pc = probe.getContext('2d', { willReadFrequently: true }); pc.drawImage(ctx.canvas, 0, 0, pw, ph); const d = pc.getImageData(0, 0, pw, ph).data, B = bg.match ? [parseInt(bg.slice(1, 3), 16), parseInt(bg.slice(3, 5), 16), parseInt(bg.slice(5, 7), 16)] : bg;
  const S = new Float32Array((pw + 1) * (ph + 1));   // summed-area table of the "ink"
  for (let y = 0; y < ph; y++) { let row = 0; for (let x = 0; x < pw; x++) { const i = 4 * (y * pw + x); row += (Math.abs(d[i] - B[0]) + Math.abs(d[i + 1] - B[1]) + Math.abs(d[i + 2] - B[2]) > 70) ? 1 : 0; S[(y + 1) * (pw + 1) + x + 1] = S[y * (pw + 1) + x + 1] + row; } }
  const frac = (x0, y0, x1, y1) => { const a = Math.max(0, Math.floor(x0 / k)), b = Math.max(0, Math.floor(y0 / k)), c = Math.min(pw, Math.ceil(x1 / k)), e = Math.min(ph, Math.ceil(y1 / k)), area = Math.max(1, Math.ceil((x1 - x0) / k) * Math.ceil((y1 - y0) / k));
    const ins = c <= a || e <= b ? 0 : S[e * (pw + 1) + c] - S[b * (pw + 1) + c] - S[e * (pw + 1) + a] + S[b * (pw + 1) + a], cover = Math.max(0, c - a) * Math.max(0, e - b); return (ins + Math.max(0, area - cover)) / area; };
  const placed = (reserved || []).map(b => b.slice()), ov = (p, q) => Math.max(0, Math.min(p[2], q[2]) - Math.max(p[0], q[0])) * Math.max(0, Math.min(p[3], q[3]) - Math.max(p[1], q[1]));
  /* labels that name the same point (consecutive, same anchor) move together as one block */
  const groups = []; items.forEach(it => { const pad = Math.max(2, it.size / 7); it.box0 = [it.x - pad, it.y - it.size * 0.82 - pad, it.x + it.w + pad, it.y + it.size * 0.3 + pad];
    const g = groups[groups.length - 1]; if (g && it.ax !== undefined && Math.abs(g[0].ax - it.ax) < 0.5 && Math.abs(g[0].ay - it.ay) < 0.5) g.push(it); else groups.push([it]); });
  groups.forEach((g, n) => { const ub = [Math.min(...g.map(i => i.box0[0])), Math.min(...g.map(i => i.box0[1])), Math.max(...g.map(i => i.box0[2])), Math.max(...g.map(i => i.box0[3]))], size = Math.max(...g.map(i => i.size)), sh = (dx, dy) => [ub[0] + dx, ub[1] + dy, ub[2] + dx, ub[3] + dy];
    const reach = Math.max(size, 0.5 * Math.min(ub[2] - ub[0], ub[3] - ub[1]));   // a big block (the x, y, z readout of S3.xyz) searches further and pays less per pixel moved
    const cost = (dx, dy, base) => { const b = sh(dx, dy), area = (b[2] - b[0]) * (b[3] - b[1]), lo = placed.reduce((a, q) => a + ov(b, q), 0) / area;
      const cut = 1 - Math.max(0, Math.min(W, b[2]) - Math.max(0, b[0])) * Math.max(0, Math.min(H, b[3]) - Math.max(0, b[1])) / area;   // the part off the canvas: always worse than shading, which is all there is where a workspace fills the picture
      return Math.min(1, 4 * frac(...b)) + 2 * cut + 3 * lo + (lo > 0.01 ? 0.5 : 0) + 0.02 * (Math.hypot(dx, dy) / reach) ** 2 + base; };   // touching another label costs extra: where the whole picture is shaded (a workspace), a slight overlap would otherwise win over moving
    let best = [cost(0, 0, 0), 0, 0]; const prev = memo && memo[n];
    if (prev) { const c = cost(prev[0], prev[1], -0.03); if (c < best[0]) best = [c, prev[0], prev[1]]; }
    if (best[0] > 0.06) for (const r of [0.45, 0.9, 1.4, 2.0, 2.7, 3.5, 4.4]) for (let j = 0; j < 16; j++) { const a = 2 * Math.PI * j / 16, dx = r * reach * Math.cos(a) * 1.3, dy = r * reach * Math.sin(a), c = cost(dx, dy, 0.02); if (c < best[0]) best = [c, dx, dy]; }
    const [, dx, dy] = best, b = sh(dx, dy); placed.push(b); if (memo) memo[n] = [dx, dy];
    g.forEach((it, k) => { it.X = it.x + dx; it.Y = it.y + dy; it.box = b; it.lead = !k && it.ax !== undefined && Math.hypot(dx, dy) > 0.9 * size; }); });
  return items; };
/* a leader line from a label back to the point it names */
S3.leader = (ctx, it, col) => { const b = it.box, cx = Math.min(Math.max(it.ax, b[0]), b[2]), cy = Math.min(Math.max(it.ay, b[1]), b[3]); ctx.save(); ctx.strokeStyle = col; ctx.globalAlpha = 0.7; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(it.ax, it.ay); ctx.lineTo(cx, cy); ctx.stroke(); ctx.restore(); };
global.S3 = S3;
})(typeof window !== 'undefined' ? window : globalThis);
