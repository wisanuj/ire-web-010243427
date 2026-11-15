/* ch1.js — the scenes of chapter 1 (Introduction). Every scene is a pure drawing function CH1.draw.name(ctx, W, H, state, palette),
   so the same code draws the live simulation on a page (dark palette) and the figure exported to the slides (light palette, CH1.FIGS).
   The interactive wrappers (sliders, dragging, steppers) are in ch1ui.js. Needs robot.js. */
(function (global) {
'use strict';
const RB = global.RB, G = RB.G, { sin, cos, atan2, sqrt, abs, PI, min, max, hypot, round, floor } = Math, rad = RB.rad, deg = RB.deg;
const T = (th, en) => (global.MC && global.MC.t) ? global.MC.t(th, en) : en;
const CH1 = { draw: {} }, D = CH1.draw;
const rr = (c, x, y, w, h, r) => { c.beginPath(); if (c.roundRect) c.roundRect(x, y, w, h, r); else c.rect(x, y, w, h); };
const pedestal = (c, x, y, P, h = 15, w = 26) => { G.poly(c, [[x - w * 0.3, y], [x + w * 0.3, y], [x + w * 0.5, y + h], [x - w * 0.5, y + h]], { close: true, fill: P.faint, stroke: P.base, lw: 1.6 }); G.ground(c, x, y + h, w * 2.4, P); };
/* the two-link arm used all through the chapter: link 1 orange, link 2 blue */
const arm2 = (c, M, l1, l2, t1, t2, P, o = {}) => { const f = RB.fk2(l1, l2, t1, t2), a = M.p(0, 0), b = M.p(f.x1, f.y1), e = M.p(f.x, f.y), w = o.w || 11;
  c.save(); if (o.alpha) c.globalAlpha = o.alpha; if (o.base !== false) pedestal(c, a[0], a[1] + w * 0.5, P);
  G.link(c, a[0], a[1], b[0], b[1], o.c1 || P.l1, w, P); G.link(c, b[0], b[1], e[0], e[1], o.c2 || P.l2, w * 0.86, P); G.rjoint(c, a[0], a[1], w * 0.74, P); G.rjoint(c, b[0], b[1], w * 0.64, P);
  if (o.grip === false) G.circle(c, e[0], e[1], w * 0.34, { fill: P.ink }); else G.gripper(c, e[0], e[1], t1 + t2, w * 1.25, P.ink, o.open === undefined ? 1 : o.open); c.restore(); return { f, a, b, e }; };
CH1.arm2 = arm2;
/* an opaque layer drawn off screen and laid over the scene with one alpha, so overlapping pieces do not darken (workspace regions) */
const layer = (s, key, W, H, paint) => { if (typeof document === 'undefined') return null; if (!s._lay || s._key !== key) { const cv = s._lay || document.createElement('canvas'), d = RB.DPR || min(global.devicePixelRatio || 1, 2.5);
    cv.width = round(W * d); cv.height = round(H * d); const x = cv.getContext('2d'); x.setTransform(d, 0, 0, d, 0, 0); x.clearRect(0, 0, W, H); paint(x); s._lay = cv; s._key = key; } return s._lay; };
const put = (c, lay, W, H, alpha) => { if (!lay) return; c.save(); c.globalAlpha = alpha; c.drawImage(lay, 0, 0, W, H); c.restore(); };

/* ---------- 1. sense – think – act (page 1.1): a two-link arm seen from above reaches for a part that a camera locates ---------- */
CH1.LOOP = { l1: 0.5, l2: 0.4 };
D.loop = (c, W, H, s, P) => { G.clear(c, W, H, P); const { l1, l2 } = CH1.LOOP, top = 58, M = RB.fitWorld(W, H - top, -0.98, 0.98, -0.3, 0.98, 4, 0, top);
  const names = [T('รับรู้', 'SENSE'), T('คิด', 'THINK'), T('กระทำ', 'ACT')], subs = [T('กล้องวัดตำแหน่ง', 'camera finds the part'), T('คำนวณมุมข้อต่อ', 'compute joint angles'), T('มอเตอร์ขับแขน', 'motors move the arm')];
  const on = [s.sense, s.think, s.act], cols = [P.l2, P.l3, P.l1], bw = min(168, W * 0.28), gap = (W - 3 * bw) / 4;
  names.forEach((n, i) => { const x = gap + i * (bw + gap), hot = on[i] && s.stage === i, col = on[i] ? cols[i] : P.muted; c.save(); rr(c, x, 8, bw, 42, 9); c.fillStyle = RB.rgba(col, hot ? 0.34 : 0.1); c.fill(); c.strokeStyle = col; c.lineWidth = hot ? 2.6 : 1.4; if (!on[i]) c.setLineDash([5, 4]); c.stroke(); c.restore();
    G.text(c, n, x + bw / 2, 26, { color: on[i] ? P.ink : P.muted, size: 15, weight: 700, align: 'center' }); G.text(c, on[i] ? subs[i] : T('ปิดอยู่', 'switched off'), x + bw / 2, 42, { color: P.muted, size: 11.5, align: 'center' });
    if (i < 2) G.arrow(c, x + bw + 5, 29, x + bw + gap - 5, 29, on[i] ? P.muted : P.faint, 2, 8); });
  G.circle(c, M.x(0), M.y(0), (l1 + l2) * M.s, { stroke: P.faint, lw: 1.2, dash: [4, 5] });
  const pp = M.p(s.tx, s.ty), cam = [W - 34, top + 24];
  c.save(); c.fillStyle = P.muted; rr(c, cam[0] - 15, cam[1] - 10, 30, 20, 4); c.fill(); c.restore(); G.circle(c, cam[0], cam[1], 5.5, { fill: P.bg, stroke: P.ink, lw: 1.5 }); G.text(c, T('กล้อง', 'camera'), cam[0], cam[1] + 25, { color: P.muted, size: 11.5, align: 'center' });
  if (s.mx !== null && s.mx !== undefined) { const m = M.p(s.mx, s.my); if (s.sense) G.line(c, cam[0], cam[1], m[0], m[1], RB.rgba(P.l2, 0.55), 1.2, [5, 5]);
    G.line(c, m[0] - 13, m[1], m[0] + 13, m[1], P.l2, 1.8); G.line(c, m[0], m[1] - 13, m[0], m[1] + 13, P.l2, 1.8); G.circle(c, m[0], m[1], 8, { stroke: P.l2, lw: 1.8 }); }
  if (s.qd) arm2(c, M, l1, l2, s.qd[0], s.qd[1], P, { alpha: P.ghost, grip: false, base: false, c1: P.l3, c2: P.l3, w: 7 });
  const A = arm2(c, M, l1, l2, s.q[0], s.q[1], P, { open: s.grip ? 0.25 : 1 });
  c.save(); c.fillStyle = P.warn; c.strokeStyle = P.ink; c.lineWidth = 1.5; rr(c, pp[0] - 9, pp[1] - 9, 18, 18, 3); c.fill(); c.stroke(); c.restore();
  G.text(c, T('ชิ้นงาน', 'part'), pp[0] + 14, pp[1] - 10, { color: P.warn, size: 12, weight: 600, halo: P.bg });
  const e = hypot(A.f.x - s.tx, A.f.y - s.ty), hit = e < 0.025;
  G.pill(c, hit ? T('ปลายแขนถึงชิ้นงาน ✓', 'the tool is on the part ✓') : T(`ปลายแขนห่างชิ้นงาน ${round(e * 100)} cm`, `the tool is ${round(e * 100)} cm from the part`), 10, H - 18, { fill: RB.rgba(hit ? P.good : P.warn, 0.16), stroke: hit ? P.good : P.warn, color: hit ? P.good : P.warn, size: 12.5 }); };

/* ---------- 2. a planar chain of links and joints (page 1.2): each joint is revolute (R) or prismatic (P) ---------- */
CH1.chainSpec = types => types.map(t => t === 'R' ? { type: 'R', a: 0.34 } : { type: 'P', a: 0.17 });
D.chain = (c, W, H, s, P) => { G.clear(c, W, H, P); const spec = CH1.chainSpec(s.types), k = RB.chain(spec, s.q), win = s.win || [-0.32, 1.22, -0.3, 0.98], M = RB.fitWorld(W, H, win[0], win[1], win[2], win[3], 6);
  const cols = [P.l1, P.l2, P.l3], S = k.pts.map(p => M.p(p[0], p[1])), lw = s.lw || max(9, min(13, W / 46)), n = spec.length;
  pedestal(c, S[0][0], S[0][1] + lw * 0.5, P);
  spec.forEach((j, i) => { const a = S[i], b = S[i + 1]; G.link(c, a[0], a[1], b[0], b[1], cols[i % 3], j.type === 'R' ? lw : lw * 0.5, P); });
  spec.forEach((j, i) => { const a = S[i]; if (j.type === 'R') G.rjoint(c, a[0], a[1], lw * 0.74, P); else { const ang = k.ang[i], m = M.p(k.pts[i][0] + 0.085 * cos(ang), k.pts[i][1] + 0.085 * sin(ang)); G.pjoint(c, m[0], m[1], -ang, 0.17 * M.s, lw * 1.3, P); } });
  G.gripper(c, S[n][0], S[n][1], k.th, lw * 1.2, P.ink, 1);
  if (s.vars !== false) spec.forEach((j, i) => { const a = S[i], p0 = i ? k.ang[i - 1] : 0, p1 = k.ang[i];
    if (j.type === 'R') { const R = 0.13 * M.s; G.line(c, a[0], a[1], a[0] + (R + 22) * cos(p0), a[1] - (R + 22) * sin(p0), P.muted, 1.2, [4, 4]); G.arc(c, a[0], a[1], R, p0, p1, P.ink, 1.6, { head: true, size: 7 });
      const am = (p0 + p1) / 2; G.sym(c, 'θ', String(i + 1), a[0] + (R + 13) * cos(am), a[1] - (R + 13) * sin(am) + 5, { color: P.ink, size: 16, align: 'center', halo: P.bg }); }
    else { const b = S[i + 1], nx = -sin(-p1) * 17, ny = cos(-p1) * 17; G.arrow(c, a[0] + nx, a[1] + ny, b[0] + nx, b[1] + ny, P.ink, 1.5, 7, true); G.sym(c, 'd', String(i + 1), (a[0] + b[0]) / 2 + nx * 1.9, (a[1] + b[1]) / 2 + ny * 1.9 + 5, { color: P.ink, size: 16, align: 'center', halo: P.bg }); } });
  if (s.names) { spec.forEach((j, i) => { const a = S[i], b = S[i + 1], ang = k.ang[i], nx = sin(ang) * -1, ny = -cos(ang), off = lw + 9;
      G.text(c, T(`ก้านต่อ ${i + 1}`, `Link ${i + 1}`), (a[0] + b[0]) / 2 + nx * off, (a[1] + b[1]) / 2 + ny * off, { color: cols[i % 3], size: 13, weight: 700, align: 'center', base: 'middle', halo: P.bg });
      G.text(c, T(`ข้อต่อ ${i + 1}`, `Joint ${i + 1}`), a[0] - nx * (off + 30) + 6, a[1] - ny * (off + 14), { color: P.ink, size: 12.5, weight: 600, align: 'left', base: 'middle', halo: P.bg }); });
    G.text(c, T('ฐาน = ก้านต่อ 0', 'Base = link 0'), S[0][0] - 40, S[0][1] + lw + 14, { color: P.base, size: 12.5, weight: 600, align: 'right', halo: P.bg });
    G.text(c, T('ปลายแขน (end-effector)', 'End-effector'), S[n][0] + 12, S[n][1] - 16, { color: P.ink, size: 12.5, weight: 600, halo: P.bg }); }
  if (s.title) G.text(c, s.types.join(''), 14, 30, { color: P.ink, size: 24, weight: 800, mono: true }); };

/* ---------- 3. degrees of freedom of a rigid body: 3 in a plane, 6 in space ---------- */
const dofPlane = (c, x0, y0, w, h, P) => { const M = RB.fitWorld(w, h - 30, -0.12, 1.12, -0.12, 0.92, 4, x0, y0 + 30), o = M.p(0, 0), bx = 0.66, by = 0.5, ph = rad(28), b = M.p(bx, by);
  G.arrow(c, o[0], o[1], M.x(1.08), o[1], P.x, 2, 9); G.arrow(c, o[0], o[1], o[0], M.y(0.9), P.y, 2, 9); G.text(c, 'X', M.x(1.08) - 4, o[1] + 17, { color: P.x, size: 13, weight: 700 }); G.text(c, 'Y', o[0] - 15, M.y(0.9) + 6, { color: P.y, size: 13, weight: 700 });
  const hw = 0.23, hh = 0.11, cr = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(([u, v]) => M.p(bx + u * cos(ph) - v * sin(ph), by + u * sin(ph) + v * cos(ph)));
  G.poly(c, cr, { close: true, fill: RB.rgba(P.l2, 0.25), stroke: P.l2, lw: 2 }); G.line(c, b[0], o[1], b[0], b[1], P.muted, 1.2, [4, 4]); G.line(c, o[0], b[1], b[0], b[1], P.muted, 1.2, [4, 4]);
  G.sym(c, 'x', '', b[0], o[1] + 17, { color: P.x, size: 17, align: 'center' }); G.sym(c, 'y', '', o[0] - 12, b[1] + 5, { color: P.y, size: 17, align: 'center' });
  G.line(c, b[0], b[1], b[0] + 0.3 * M.s, b[1], P.muted, 1.2, [4, 4]); G.arrow(c, b[0], b[1], b[0] + 0.3 * M.s * cos(ph), b[1] - 0.3 * M.s * sin(ph), P.ink, 1.8, 8); G.arc(c, b[0], b[1], 0.2 * M.s, 0, ph, P.sel, 1.8, { head: true, size: 7 });
  G.sym(c, 'φ', '', b[0] + 0.3 * M.s, b[1] - 0.05 * M.s, { color: P.sel, size: 17, halo: P.bg }); G.circle(c, b[0], b[1], 3.5, { fill: P.ink });
  G.text(c, T('ในระนาบ: 3 องศาอิสระ', 'In a plane: 3 DOF'), x0 + w / 2, y0 + 18, { color: P.ink, size: 14.5, weight: 700, align: 'center' }); };
const dofSpace = (c, x0, y0, w, h, P) => { const sc = min(w * 0.4, (h - 30) * 0.56), cam = RB.cam(rad(-58), rad(22), sc, x0 + w * 0.46, y0 + 30 + (h - 30) * 0.58), pr = v => cam.p(v), hx = 0.44, hy = 0.28, hz = 0.16;
  const V = []; for (const i of [-1, 1]) for (const j of [-1, 1]) for (const k of [-1, 1]) V.push([i * hx, j * hy, k * hz]); const E = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];
  const face = ids => G.poly(c, ids.map(i => pr(V[i])), { close: true, fill: RB.rgba(P.l2, 0.2) }); face([1, 3, 7, 5]); face([4, 5, 7, 6]); face([2, 3, 7, 6]);
  E.forEach(([a, b]) => { const p = pr(V[a]), q = pr(V[b]); G.line(c, p[0], p[1], q[0], q[1], P.l2, 1.6); });
  [[[1, 0, 0], P.x, 'x', T('หมุนรอบ X (roll)', 'roll')], [[0, 1, 0], P.y, 'y', T('หมุนรอบ Y (pitch)', 'pitch')], [[0, 0, 1], P.z, 'z', T('หมุนรอบ Z (yaw)', 'yaw')]].forEach(([d, col, nm, rot]) => { const a = pr(RB.sc(d, -0.2)), b = pr(RB.sc(d, 1.02));
    G.arrow(c, a[0], a[1], b[0], b[1], col, 2, 9); G.sym(c, nm, '', b[0] + 6, b[1] + 4, { color: col, size: 17 });
    const ring = RB.ring(RB.sc(d, 0.74), d, 0.13, 0.5, 5.6, 30).map(pr); G.poly(c, ring, { stroke: col, lw: 1.8 }); const q = ring[ring.length - 1], q0 = ring[ring.length - 3]; G.head(c, q[0], q[1], atan2(q[1] - q0[1], q[0] - q0[0]), col, 7);
    const lp = pr(RB.sc(d, 0.74)); G.text(c, rot, lp[0] + (nm === 'y' ? 14 : nm === 'x' ? -14 : 16), lp[1] + (nm === 'z' ? 4 : nm === 'x' ? 30 : -20), { color: col, size: 11.5, weight: 600, align: nm === 'x' ? 'right' : 'left', halo: P.bg }); });
  G.text(c, T('ในปริภูมิ: 6 องศาอิสระ', 'In space: 6 DOF'), x0 + w / 2, y0 + 18, { color: P.ink, size: 14.5, weight: 700, align: 'center' }); };
D.dof = (c, W, H, s, P) => { G.clear(c, W, H, P); if (s.mode === 'plane') dofPlane(c, 0, 0, W, H, P); else if (s.mode === 'space') dofSpace(c, 0, 0, W, H, P); else { dofPlane(c, 0, 0, W * 0.47, H, P); G.line(c, W * 0.48, 16, W * 0.48, H - 16, P.faint, 1); dofSpace(c, W * 0.49, 0, W * 0.51, H, P); } };

/* ---------- 4. workspace of the two-link arm (page 1.2) ---------- */
CH1.FULL = { a1: -PI, b1: PI, a2: -PI, b2: PI };
D.ws2 = (c, W, H, s, P) => { G.clear(c, W, H, P); const R = s.R || 1, M = RB.fitWorld(W, H, -R * 1.06, R * 1.06, -R * 1.06, R * 1.06, 4), { l1, l2 } = s, lim = s.lim || CH1.FULL, o = M.p(0, 0), k = s.k === undefined ? 9 : s.k;
  G.grid(c, M, -R, R, -R, R, 0.2, P.grid); G.line(c, M.x(-R), o[1], M.x(R), o[1], P.faint, 1.2); G.line(c, o[0], M.y(-R), o[0], M.y(R), P.faint, 1.2);
  if (k >= 3) { const key = [W, H, l1, l2, lim.a1, lim.b1, lim.a2, lim.b2, P.ws].join('|'), lay = layer(s, key, W, H, x => { const N = 72, Mm = 72, r = t => hypot(l1 + l2 * cos(t), l2 * sin(t)), be = t => atan2(l2 * sin(t), l1 + l2 * cos(t));
      const arcp = t => { const out = []; for (let i = 0; i <= Mm; i++) { const a = lim.a1 + (lim.b1 - lim.a1) * i / Mm + be(t); out.push(M.p(r(t) * cos(a), r(t) * sin(a))); } return out; };
      let prev = arcp(lim.a2); for (let j = 1; j <= N; j++) { const cur = arcp(lim.a2 + (lim.b2 - lim.a2) * j / N); G.poly(x, prev.concat(cur.slice().reverse()), { close: true, fill: P.ws, stroke: P.ws, lw: 1 }); prev = cur; } });
    put(c, lay, W, H, P.wsA * 1.6); }
  const full = lim.b1 - lim.a1 >= 2 * PI - 1e-6 && lim.b2 - lim.a2 >= 2 * PI - 1e-6, rg = RB.ring2(l1, l2);
  if (full && k >= 1) { G.circle(c, o[0], o[1], rg.rmax * M.s, { stroke: P.ws, lw: 2 }); const a = rad(38), e = M.p(rg.rmax * cos(a), rg.rmax * sin(a)); G.arrow(c, o[0], o[1], e[0], e[1], P.ws, 1.6, 8);
    const lp = M.p(0.5 * rg.rmax * cos(rad(22)), 0.5 * rg.rmax * sin(rad(22))), lx = lp[0], ly = lp[1] + 6, w = G.sym(c, 'r', 'max', lx, ly, { color: P.ws, size: 16, halo: P.bg }); G.text(c, ` = ${RB.u(rg.rmax, 2)} m`, lx + w, ly, { color: P.ws, size: 13, weight: 700, halo: P.bg }); }
  if (full && k >= 2 && rg.rmin > 1e-9) { G.circle(c, o[0], o[1], rg.rmin * M.s, { stroke: P.sel, lw: 2 }); const a = rad(150), e = M.p(rg.rmin * cos(a), rg.rmin * sin(a)), t = M.p(0.52 * R * cos(a), 0.52 * R * sin(a)); G.line(c, e[0], e[1], t[0], t[1], P.sel, 1.2);
    const w = G.sym(c, 'r', 'min', t[0] - 92, t[1] - 6, { color: P.sel, size: 16, halo: P.bg }); G.text(c, ` = ${RB.u(rg.rmin, 2)} m`, t[0] - 92 + w, t[1] - 6, { color: P.sel, size: 13, weight: 700, halo: P.bg }); }
  if (s.arm !== false) arm2(c, M, l1, l2, s.t1, s.t2, P, { w: s.w || 9 });
  if (s.trace && s.trace.length > 1) G.poly(c, s.trace.map(p => M.p(p[0], p[1])), { stroke: P.warn, lw: 1.6 }); };

/* ---------- 5. reachable and dexterous workspace of a three-link planar arm (page 1.2) ---------- */
D.dex = (c, W, H, s, P) => { G.clear(c, W, H, P); const R = s.R || 1, M = RB.fitWorld(W, H, -R * 1.06, R * 1.06, -R * 1.06, R * 1.06, 4), { l1, l2, l3 } = s, o = M.p(0, 0), rg = RB.ring3(l1, l2, l3);
  G.grid(c, M, -R, R, -R, R, 0.2, P.grid);
  const ann = (r0, r1, col, a) => { c.save(); c.beginPath(); c.arc(o[0], o[1], r1 * M.s, 0, 2 * PI); if (r0 > 1e-9) c.arc(o[0], o[1], r0 * M.s, 0, 2 * PI, true); c.fillStyle = RB.rgba(col, a); c.fill('evenodd'); c.restore(); G.circle(c, o[0], o[1], r1 * M.s, { stroke: col, lw: 1.6 }); if (r0 > 1e-9) G.circle(c, o[0], o[1], r0 * M.s, { stroke: col, lw: 1.6 }); };
  ann(rg.reach[0], rg.reach[1], P.ws, P.wsA * 1.4); if (rg.dex) ann(rg.dex[0], rg.dex[1], P.dex, P.dexA);
  const ok = RB.dexOk(l1, l2, l3, s.x, s.y), p = M.p(s.x, s.y), N = 60; let good = 0;
  for (let i = 0; i < N; i++) { const a0 = 2 * PI * i / N, a1 = 2 * PI * (i + 0.86) / N, g = ok((a0 + a1) / 2); if (g) good++; c.save(); c.strokeStyle = g ? P.good : P.bad; c.lineWidth = 6; c.beginPath(); c.arc(p[0], p[1], 27, -a0, -a1, true); c.stroke(); c.restore(); }
  if (ok(s.phi)) { const wx = s.x - l3 * cos(s.phi), wy = s.y - l3 * sin(s.phi), q = RB.ik2(l1, l2, wx, wy, s.elbow || 1), f = RB.fk2(l1, l2, q.t1, q.t2), w = s.w || 8;
    pedestal(c, o[0], o[1] + w * 0.5, P); const b = M.p(f.x1, f.y1), e = M.p(wx, wy);
    G.link(c, o[0], o[1], b[0], b[1], P.l1, w, P); G.link(c, b[0], b[1], e[0], e[1], P.l2, w * 0.86, P); G.link(c, e[0], e[1], p[0], p[1], P.l3, w * 0.74, P); G.rjoint(c, o[0], o[1], w * 0.74, P); G.rjoint(c, b[0], b[1], w * 0.64, P); G.rjoint(c, e[0], e[1], w * 0.58, P); }
  else { pedestal(c, o[0], o[1] + 4, P); G.arrow(c, p[0] - 44 * cos(s.phi), p[1] + 44 * sin(s.phi), p[0], p[1], P.bad, 2.4, 9, false, [5, 4]); }
  G.circle(c, p[0], p[1], 5, { fill: P.ink, stroke: P.bg, lw: 1.5 });
  const v = good === N ? [T('ไปถึงได้ทุกทิศ: อยู่ในปริภูมิทำงานแบบคล่องแคล่ว', 'every direction works: in the dexterous workspace'), P.good] : good ? [T(`ไปถึงได้บางทิศ (${round(100 * good / N)}%): ไปถึงได้ แต่ไม่คล่องแคล่ว`, `only some directions (${round(100 * good / N)}%): reachable, not dexterous`), P.warn] : [T('ไปไม่ถึง: อยู่นอกปริภูมิทำงาน', 'out of reach: outside the workspace'), P.bad];
  G.pill(c, v[0], 10, 18, { fill: RB.rgba(v[1], 0.16), stroke: v[1], color: v[1], size: 12.5 }); s._good = good / N; };

/* ---------- 6. point-to-point against continuous-path control (page 1.3) ---------- */
CH1.PATH = { l1: 0.5, l2: 0.4, e: -1 };
CH1.pathAt = (s, u, mode) => { const { l1, l2, e } = CH1.PATH, A = s.A, B = s.B, at = v => [A[0] + (B[0] - A[0]) * v, A[1] + (B[1] - A[1]) * v];
  if (mode === 'cp') { const p = at(u), q = RB.ik2(l1, l2, p[0], p[1], e); return { p, q: [q.t1, q.t2], ok: q.ok }; }
  const n = max(2, s.n | 0), x = min(n - 1 - 1e-9, max(0, u * (n - 1))), i = floor(x), v = x - i, qa = RB.ik2(l1, l2, ...at(i / (n - 1)), e), qb = RB.ik2(l1, l2, ...at((i + 1) / (n - 1)), e);
  if (!qa.ok || !qb.ok) return { ok: false, p: at(u), q: [0, 0] }; const q = [qa.t1 + RB.wrap(qb.t1 - qa.t1) * v, qa.t2 + (qb.t2 - qa.t2) * v], f = RB.fk2(l1, l2, q[0], q[1]); return { p: [f.x, f.y], q, ok: true }; };
/* largest distance between the point-to-point tool path and the straight seam A–B (metres) */
CH1.pathDev = s => { const A = s.A, B = s.B, L = hypot(B[0] - A[0], B[1] - A[1]) || 1; let m = 0; for (let k = 0; k <= 240; k++) { const r = CH1.pathAt(s, k / 240, 'ptp'); if (!r.ok) return NaN; m = max(m, abs((B[0] - A[0]) * (r.p[1] - A[1]) - (B[1] - A[1]) * (r.p[0] - A[0])) / L); } return m; };
D.path = (c, W, H, s, P) => { G.clear(c, W, H, P); const { l1, l2 } = CH1.PATH, M = RB.fitWorld(W, H, -0.32, 1.02, -0.22, 0.9, 4), a = M.p(...s.A), b = M.p(...s.B);
  G.grid(c, M, -0.3, 1, -0.2, 0.9, 0.1, P.grid); G.circle(c, M.x(0), M.y(0), (l1 + l2) * M.s, { stroke: P.faint, lw: 1.2, dash: [4, 5] });
  G.line(c, a[0], a[1], b[0], b[1], RB.rgba(P.muted, 0.55), 7); G.line(c, a[0], a[1], b[0], b[1], P.good, 1.8, [7, 5]);
  const pts = []; let okAll = true; for (let k = 0; k <= 160; k++) { const r = CH1.pathAt(s, k / 160, 'ptp'); if (!r.ok) okAll = false; pts.push(M.p(...r.p)); } if (okAll) G.poly(c, pts, { stroke: P.sel, lw: 2.4 });
  const n = max(2, s.n | 0); for (let i = 0; i < n; i++) { const p = M.p(s.A[0] + (s.B[0] - s.A[0]) * i / (n - 1), s.A[1] + (s.B[1] - s.A[1]) * i / (n - 1)); G.circle(c, p[0], p[1], 5, { fill: P.warn, stroke: P.bg, lw: 1.5 }); }
  const r = CH1.pathAt(s, s.u, s.mode); if (r.ok) arm2(c, M, l1, l2, r.q[0], r.q[1], P, { w: 9, grip: false }); const tp = M.p(...r.p); G.circle(c, tp[0], tp[1], 5.5, { fill: s.mode === 'cp' ? P.good : P.sel, stroke: P.bg, lw: 1.5 });
  G.text(c, 'A', a[0] - 16, a[1] + 18, { color: P.ink, size: 14, weight: 800, halo: P.bg }); G.text(c, 'B', b[0] + 8, b[1] - 8, { color: P.ink, size: 14, weight: 800, halo: P.bg });
  const dv = CH1.pathDev(s), y0 = H - 40; G.line(c, 12, y0, 40, y0, P.sel, 2.4); G.text(c, T(`จุดต่อจุด ${n} จุดสอน: ออกนอกแนวมากสุด ${isNaN(dv) ? '—' : RB.u(dv * 1000, 1)} mm`, `point-to-point, ${n} taught points: up to ${isNaN(dv) ? '—' : RB.u(dv * 1000, 1)} mm off the seam`), 46, y0 + 4, { color: P.sel, size: 12.5, weight: 600, halo: P.bg });
  G.line(c, 12, y0 + 20, 40, y0 + 20, P.good, 1.8, [7, 5]); G.text(c, T('เส้นทางต่อเนื่อง: อยู่บนแนวเชื่อมตลอด', 'continuous path: on the seam all the way'), 46, y0 + 24, { color: P.good, size: 12.5, weight: 600, halo: P.bg }); };

/* ---------- 7. the spherical wrist: three axes through one point (page 1.3) ---------- */
CH1.wristR = s => RB.mm(RB.mm(RB.Rz(s.yaw), RB.Ry(-s.pitch)), RB.Rx(s.roll));
D.wrist = (c, W, H, s, P) => { G.clear(c, W, H, P); const sc = min(W * 0.36, H * 0.56), cam = RB.cam(rad(-35), rad(24), sc, W * 0.47, H * 0.5), pr = v => cam.p(v), R = CH1.wristR(s), L = (a, b, col, lw, dash) => { const p = pr(a), q = pr(b); G.line(c, p[0], p[1], q[0], q[1], col, lw, dash); };
  const ax = [{ d: [0, 0, 1], col: P.z, th: T('หันซ้ายขวา (yaw)', 'yaw') }, { d: RB.mv(RB.Rz(s.yaw), [0, 1, 0]), col: P.y, th: T('ก้มเงย (pitch)', 'pitch') }, { d: RB.mv(R, [1, 0, 0]), col: P.x, th: T('หมุนรอบแกนเครื่องมือ (roll)', 'roll') }];
  const fore = pr([-1.05, 0, 0]), o = pr([0, 0, 0]); G.link(c, fore[0], fore[1], o[0], o[1], P.l2, 15, P); G.text(c, T('ปลายแขน (forearm)', 'forearm'), fore[0] + 4, fore[1] + 30, { color: P.l2, size: 12.5, weight: 600, halo: P.bg });
  ax.forEach((a, i) => { L(RB.sc(a.d, -0.62), RB.sc(a.d, 0.9), a.col, 1.7, [8, 3, 2, 3]); const e = pr(RB.sc(a.d, 0.9)), e0 = pr(RB.sc(a.d, 0.8)); G.head(c, e[0], e[1], atan2(e[1] - e0[1], e[0] - e0[0]), a.col, 8);
    const ring = RB.ring(RB.sc(a.d, i === 2 ? 0.74 : 0.66), a.d, 0.085, 0.4, 5.7, 28).map(pr); G.poly(c, ring, { stroke: a.col, lw: 1.8 }); const q = ring[ring.length - 1], q0 = ring[ring.length - 3]; G.head(c, q[0], q[1], atan2(q[1] - q0[1], q[0] - q0[0]), a.col, 7);
    G.text(c, a.th, e[0] + 8, e[1] + (i === 0 ? 4 : i === 1 ? -6 : 16), { color: a.col, size: 12.5, weight: 700, halo: P.bg }); });
  const tl = v => RB.mv(R, v), g = 0.14; [[[0, 0, 0], [0.3, 0, 0]], [[0.3, -g, 0], [0.3, g, 0]], [[0.3, -g, 0], [0.52, -g, 0]], [[0.3, g, 0], [0.52, g, 0]]].forEach(([a, b], i) => { const p = pr(tl(a)), q = pr(tl(b)); G.link(c, p[0], p[1], q[0], q[1], i ? P.ink : P.l3, i ? 6 : 11, i ? null : P); });
  const dot = pr(tl([0.52, g, 0])); G.circle(c, dot[0], dot[1], 4.5, { fill: P.warn }); G.circle(c, o[0], o[1], 8, { fill: P.joint, stroke: P.ink, lw: 2.4 }); G.circle(c, o[0], o[1], 2.6, { fill: P.ink });
  G.line(c, o[0] + 6, o[1] + 8, o[0] + 40, o[1] + 62, P.muted, 1.1); G.text(c, T('จุดศูนย์กลางข้อมือ: แกนทั้งสามตัดกันที่จุดนี้', 'wrist centre: the three axes meet here'), o[0] + 30, o[1] + 78, { color: P.ink, size: 12.5, weight: 600, halo: P.bg }); };

/* ---------- 8. resolution of a linear axis and of a rotating link reaching the same two points (page 1.3) ---------- */
D.resol = (c, W, H, s, P) => { G.clear(c, W, H, P); const l = s.l, th = s.th, M = RB.fitWorld(W, H, -1.22, 1.22, -0.36, 1.3, 4), o = M.p(0, 0), a0 = PI / 2 + th / 2, a1 = PI / 2 - th / 2, S = M.p(l * cos(a0), l * sin(a0)), E = M.p(l * cos(a1), l * sin(a1));
  const N = 2 ** s.bits, uq = s.quant === false ? s.u : round(s.u * N) / N, ang = a0 + (a1 - a0) * uq, tip = M.p(l * cos(ang), l * sin(ang)), car = [S[0] + (E[0] - S[0]) * uq, S[1]], show = s.show || 'both';
  pedestal(c, o[0], o[1] + 6, P);
  if (show !== 'rot') { G.line(c, S[0] - 26, S[1], E[0] + 26, S[1], P.base, 5); G.line(c, S[0], S[1], E[0], S[1], P.l2, 2.2); }
  if (show !== 'lin') G.arc(c, o[0], o[1], l * M.s, a0, a1, P.l1, 2.2);
  const step = max(1, N / 64); if (s.ticks !== false) for (let k = 0; k <= N; k += step) { const u = k / N, a = a0 + (a1 - a0) * u, x = S[0] + (E[0] - S[0]) * u;
    if (show !== 'lin') G.line(c, o[0] + (l * M.s - 5) * cos(a), o[1] - (l * M.s - 5) * sin(a), o[0] + (l * M.s + 5) * cos(a), o[1] - (l * M.s + 5) * sin(a), P.l1, 1.2); if (show !== 'rot') G.line(c, x, S[1] - 5, x, S[1] + 5, P.l2, 1.2); }
  if (show !== 'lin') { G.link(c, o[0], o[1], tip[0], tip[1], P.l1, 10, P); G.rjoint(c, o[0], o[1], 8, P); G.circle(c, tip[0], tip[1], 4.5, { fill: P.ink });
    G.arc(c, o[0], o[1], 34, a0, a1, P.ink, 1.4, { both: true, size: 6 }); G.sym(c, 'θ', '', o[0], o[1] - 44, { color: P.ink, size: 17, align: 'center', halo: P.bg });
    const m = (a0 + ang) / 2 + 0.0; G.sym(c, 'l', '', (o[0] + tip[0]) / 2 + 12 * cos(ang - PI / 2), (o[1] + tip[1]) / 2 - 12 * sin(ang - PI / 2) + 5, { color: P.l1, size: 18, align: 'center', halo: P.bg });
    const top = M.p(0, l); G.sym(c, 'lθ', '', top[0], top[1] - 12, { color: P.l1, size: 18, align: 'center', halo: P.bg }); }
  if (show !== 'rot') { c.save(); c.fillStyle = P.joint; c.strokeStyle = P.l2; c.lineWidth = 2.4; rr(c, car[0] - 11, car[1] - 8, 22, 16, 3); c.fill(); c.stroke(); c.restore();
    G.arrow(c, S[0], S[1] + 22, E[0], S[1] + 22, P.l2, 1.5, 7, true); G.sym(c, 'd', '', (S[0] + E[0]) / 2, S[1] + 42, { color: P.l2, size: 18, align: 'center', halo: P.bg }); }
  if (s.quant !== false && abs(uq - s.u) > 1e-9) { const x = S[0] + (E[0] - S[0]) * s.u, a = a0 + (a1 - a0) * s.u; if (show !== 'rot') G.head(c, x, S[1] - 9, PI / 2, P.sel, 9); if (show !== 'lin') G.circle(c, o[0] + l * M.s * cos(a), o[1] - l * M.s * sin(a), 4, { stroke: P.sel, lw: 2 }); }
  if (N > 64 && s.ticks !== false) G.text(c, T(`${N} ขั้น: วาดเพียงทุก ${step} ขั้น`, `${N} steps: only every ${step}th is drawn`), W - 10, H - 10, { color: P.muted, size: 11.5, align: 'right' }); };

/* ---------- 9. the five common arrangements in 3-D with their workspace (page 1.4) ---------- */
D.arr = (c, W, H, s, P) => { G.clear(c, W, H, P); const A = RB.ARR[s.type], q = RB.arrQ(s.type, s.q), K = A.kin(q), sc = (s.zoom || 1) * min(W * 0.4, H * 0.5), cam = RB.cam(s.az === undefined ? rad(-58) : s.az, s.el === undefined ? rad(24) : s.el, sc, W * 0.5, H * 0.68), ctr = A.ctr || [0, 0, 0], pr = v => cam.p([v[0] - ctr[0], v[1] - ctr[1], v[2] - ctr[2]]);
  const L = (a, b, col, lw, dash) => { const p = pr(a), r = pr(b); G.line(c, p[0], p[1], r[0], r[1], col, lw, dash); };
  for (let i = -4; i <= 4; i++) { L([i * 0.25, -1, 0], [i * 0.25, 1, 0], P.grid, 1); L([-1, i * 0.25, 0], [1, i * 0.25, 0], P.grid, 1); }
  [[[1, 0, 0], P.x, 'X'], [[0, 1, 0], P.y, 'Y'], [[0, 0, 1], P.z, 'Z']].forEach(([d, col, nm]) => { const a = pr([-1, -1, 0]), b = pr(RB.add([-1, -1, 0], RB.sc(d, 0.3))); G.arrow(c, a[0], a[1], b[0], b[1], col, 2, 7); G.text(c, nm, b[0] + 4, b[1] + 4, { color: col, size: 12, weight: 800 }); });
  if (s.ws !== false) { if (!s._wire || s._wt !== s.type) { s._wt = s.type; s._wire = RB.arrWire(s.type); } s._wire.forEach(w => G.poly(c, w.line.map(pr), { stroke: RB.rgba(P.ws, w.edge ? 0.8 : 0.3), lw: w.edge ? 1.5 : 1 })); }
  G.poly(c, K.pts.map(p => pr([p[0], p[1], 0])), { stroke: RB.rgba(P.ink, 0.2), lw: 5 });
  if (s.type === 'PPP') L([0, 0, 0], [1, 0, 0], P.base, 5); if (s.type === 'RPP') L([0, 0, 0], [0, 0, 0.95], P.base, 5); if (s.type !== 'PPP') G.poly(c, RB.ring([0, 0, 0], [0, 0, 1], 0.13).map(pr), { close: true, fill: P.faint, stroke: P.base, lw: 1.5 });
  const S = K.pts.map(pr), cols = [P.base, P.l1, P.l2, P.l3]; for (let i = 0; i < S.length - 1; i++) G.link(c, S[i][0], S[i][1], S[i + 1][0], S[i + 1][1], cols[A.seg[i]], A.seg[i] <= 1 ? 12 : 9, P);
  K.axes.forEach((a, i) => { const o = pr(a.o), e1 = pr(RB.add(a.o, RB.sc(a.d, -0.2))), e2 = pr(RB.add(a.o, RB.sc(a.d, 0.34))); G.line(c, e1[0], e1[1], e2[0], e2[1], P.z, 1.7, [8, 3, 2, 3]); G.head(c, e2[0], e2[1], atan2(e2[1] - e1[1], e2[0] - e1[0]), P.z, 8);
    if (a.t === 'R') { const ring = RB.ring(RB.add(a.o, RB.sc(a.d, 0.2)), a.d, 0.07, 0.4, 5.7, 24).map(pr); G.poly(c, ring, { stroke: P.ink, lw: 1.5 }); const r1 = ring[ring.length - 1], r0 = ring[ring.length - 3]; G.head(c, r1[0], r1[1], atan2(r1[1] - r0[1], r1[0] - r0[0]), P.ink, 6); G.circle(c, o[0], o[1], 7, { fill: P.joint, stroke: P.ink, lw: 2.2 }); }
    else { c.save(); c.fillStyle = P.joint; c.strokeStyle = P.ink; c.lineWidth = 2.2; c.beginPath(); c.rect(o[0] - 6.5, o[1] - 6.5, 13, 13); c.fill(); c.stroke(); c.restore(); }
    if (s.labels !== false) { G.sym(c, 'Z', String(i + 1), e2[0] + (i ? 5 : -5), e2[1] - 2, { color: P.z, size: 15, halo: P.bg, align: i ? 'left' : 'right' }); G.sym(c, a.t === 'R' ? 'θ' : 'd', String(i + 1), o[0] + 11, o[1] + 18, { color: P.ink, size: 16, halo: P.bg }); } });
  const t = S[S.length - 1]; G.circle(c, t[0], t[1], 5.5, { fill: P.sel, stroke: P.bg, lw: 1.5 });
  if (s.title !== false) { G.text(c, s.type === 'SCARA' ? 'SCARA · RRP' : A.joints, 12, 28, { color: P.ink, size: 22, weight: 800, mono: true }); } };

/* ---------- 10. a planar five-bar parallel arm (page 1.4) ---------- */
CH1.FB = { c: 0.14, La: 0.3, Lb: 0.46 };
D.fivebar = (c, W, H, s, P) => { G.clear(c, W, H, P); const { c: cc, La, Lb } = CH1.FB, M = RB.fitWorld(W, H, -0.78, 0.78, -0.2, 0.86, 4), F = RB.fivebar(cc, La, Lb, s.t1, s.t2), m1 = M.p(-cc, 0), m2 = M.p(cc, 0);
  G.grid(c, M, -0.75, 0.75, -0.15, 0.85, 0.1, P.grid);
  if (s.ws !== false) { const lay = layer(s, [W, H, P.ws].join('|'), W, H, x => { x.fillStyle = P.ws; for (let i = 0; i <= 150; i++) for (let j = 0; j <= 150; j++) { const r = RB.fivebar(cc, La, Lb, rad(10 + 160 * i / 150), rad(10 + 160 * j / 150)); if (r.ok && r.P[1] > 0.04) { const p = M.p(...r.P); x.beginPath(); x.arc(p[0], p[1], 2.6, 0, 2 * PI); x.fill(); } } }); put(c, lay, W, H, P.wsA * 1.5); }
  G.ground(c, (m1[0] + m2[0]) / 2, m1[1] + 12, (m2[0] - m1[0]) + 70, P); G.line(c, m1[0], m1[1], m2[0], m2[1], P.base, 6);
  const a = M.p(...F.A), b = M.p(...F.B); G.link(c, m1[0], m1[1], a[0], a[1], P.l1, 10, P); G.link(c, m2[0], m2[1], b[0], b[1], P.l1, 10, P);
  if (F.ok) { const p = M.p(...F.P); G.link(c, a[0], a[1], p[0], p[1], P.l2, 8.5, P); G.link(c, b[0], b[1], p[0], p[1], P.l2, 8.5, P); G.circle(c, p[0], p[1], 7, { fill: P.sel, stroke: P.bg, lw: 1.5 }); G.text(c, T('จุดเครื่องมือ', 'tool point'), p[0] + 11, p[1] - 9, { color: P.sel, size: 12.5, weight: 700, halo: P.bg }); }
  else G.pill(c, T('ประกอบไม่ได้: ก้านปลายสั้นเกินกว่าจะมาบรรจบกัน', 'cannot close: the outer links do not meet'), W / 2, 22, { align: 'center', fill: RB.rgba(P.bad, 0.16), stroke: P.bad, color: P.bad, size: 12.5 });
  [m1, m2].forEach((m, i) => { G.circle(c, m[0], m[1], 9, { fill: P.l1, stroke: P.ink, lw: 2.2 }); G.sym(c, 'θ', String(i + 1), m[0] + (i ? 12 : -30), m[1] + 30, { color: P.ink, size: 16, halo: P.bg }); });
  [a, b].forEach(j => G.circle(c, j[0], j[1], 6.5, { fill: P.joint, stroke: P.ink, lw: 2.2 }));
  G.text(c, T('● ข้อต่อที่มีมอเตอร์   ○ ข้อต่ออิสระ', '● motor joint   ○ free joint'), 10, H - 10, { color: P.muted, size: 12 }); };

/* ---------- 11. inverse kinematics of the two-link arm (page 1.4, problem 1.3) ---------- */
D.ik2 = (c, W, H, s, P) => { G.clear(c, W, H, P); const { l1, l2 } = s, win = s.win || [-0.3, 1.05, -0.22, 0.95], M = RB.fitWorld(W, H, win[0], win[1], win[2], win[3], 4), o = M.p(0, 0), e = s.elbow || 1, q = RB.ik2(l1, l2, s.x, s.y, e), p = M.p(s.x, s.y);
  if (s.grid !== false) G.grid(c, M, win[0], win[1], win[2], win[3], 0.1, P.grid);
  if (s.ring !== false) { G.circle(c, o[0], o[1], (l1 + l2) * M.s, { stroke: P.faint, lw: 1.3, dash: [5, 5] }); if (abs(l1 - l2) > 1e-9) G.circle(c, o[0], o[1], abs(l1 - l2) * M.s, { stroke: P.faint, lw: 1.3, dash: [5, 5] }); }
  if (s.axes !== false) { G.arrow(c, M.x(win[0] + 0.03), o[1], M.x(win[1] - 0.02), o[1], P.x, 1.6, 8); G.arrow(c, o[0], M.y(win[2] + 0.03), o[0], M.y(win[3] - 0.02), P.y, 1.6, 8); G.sym(c, 'x', '', M.x(win[1] - 0.02) - 6, o[1] + 18, { color: P.x, size: 16 }); G.sym(c, 'y', '', o[0] - 16, M.y(win[3] - 0.02) + 10, { color: P.y, size: 16 }); }
  if (!q.ok) { const t = atan2(s.y, s.x), far = s.x * s.x + s.y * s.y > l1 * l1; arm2(c, M, l1, l2, t, far ? 0 : PI, P, { alpha: 0.55, w: s.w || 10 }); G.line(c, p[0] - 9, p[1] - 9, p[0] + 9, p[1] + 9, P.bad, 2.6); G.line(c, p[0] - 9, p[1] + 9, p[0] + 9, p[1] - 9, P.bad, 2.6);
    G.pill(c, T('นอกปริภูมิทำงาน: |D| > 1 ไม่มีคำตอบ', 'outside the workspace: |D| > 1, no solution'), 10, 18, { fill: RB.rgba(P.bad, 0.16), stroke: P.bad, color: P.bad, size: 12.5 }); return; }
  if (s.both !== false) { const q2 = RB.ik2(l1, l2, s.x, s.y, -e); arm2(c, M, l1, l2, q2.t1, q2.t2, P, { alpha: P.ghost, grip: false, base: false, w: (s.w || 10) * 0.8 }); }
  const A = arm2(c, M, l1, l2, q.t1, q.t2, P, { w: s.w || 10, grip: false });
  if (s.labels !== false) { G.line(c, p[0], p[1], p[0], o[1], P.muted, 1.2, [4, 4]); G.line(c, p[0], p[1], o[0], p[1], P.muted, 1.2, [4, 4]);
    G.arc(c, o[0], o[1], 38, 0, q.t1, P.ink, 1.5, { head: true, size: 6 }); G.sym(c, 'θ', '1', o[0] + 50 * cos(q.t1 / 2), o[1] - 50 * sin(q.t1 / 2) + 5, { color: P.ink, size: 16, align: 'center', halo: P.bg });
    G.line(c, A.b[0], A.b[1], A.b[0] + 58 * cos(q.t1), A.b[1] - 58 * sin(q.t1), P.muted, 1.2, [4, 4]); G.arc(c, A.b[0], A.b[1], 30, q.t1, q.t1 + q.t2, P.ink, 1.5, { head: true, size: 6 });
    G.sym(c, 'θ', '2', A.b[0] + 44 * cos(q.t1 + q.t2 / 2), A.b[1] - 44 * sin(q.t1 + q.t2 / 2) + 5, { color: P.ink, size: 16, align: 'center', halo: P.bg });
    const nm = (a, b, ang, sub, col) => G.sym(c, 'a', sub, (a[0] + b[0]) / 2 + 15 * sin(ang) * e, (a[1] + b[1]) / 2 + 15 * cos(ang) * e + 5, { color: col, size: 16, align: 'center', halo: P.bg }); nm(A.a, A.b, q.t1, '1', P.l1); nm(A.b, A.e, q.t1 + q.t2, '2', P.l2); }
  G.circle(c, p[0], p[1], 6, { fill: P.sel, stroke: P.bg, lw: 1.5 }); if (s.labels !== false) G.text(c, '(x, y)', p[0] + 9, p[1] - 9, { color: P.sel, size: 13, weight: 700, halo: P.bg, font: RB.SERIF, italic: true }); };

/* ---------- 12. accuracy and repeatability (page 1.3): where the tool lands when sent to the centre five times ---------- */
D.target = (c, W, H, s, P) => { G.clear(c, W, H, P); const sets = [[[0.05, 0.03], [-0.04, 0.06], [0.02, -0.05], [-0.06, -0.02], [0.07, -0.03]], [[0.5, 0.36], [0.44, 0.43], [0.53, 0.3], [0.41, 0.33], [0.48, 0.27]], [[-0.6, 0.2], [0.5, 0.55], [0.1, -0.65], [-0.3, -0.4], [0.65, -0.15]]];
  const cap = [[T('แม่นยำ และทำซ้ำได้ดี', 'accurate and repeatable'), P.good], [T('ทำซ้ำได้ดี แต่ไม่แม่นยำ', 'repeatable, not accurate'), P.warn], [T('ไม่แม่นยำ และทำซ้ำไม่ได้', 'neither'), P.bad]], r = min(W / 7.2, (H - 44) / 2.3);
  sets.forEach((pts, i) => { const cx = W * (i + 0.5) / 3, cy = 12 + r * 1.08; [1, 0.72, 0.44, 0.16].forEach((k, j) => G.circle(c, cx, cy, r * k, { fill: j === 3 ? RB.rgba(P.sel, 0.5) : undefined, stroke: P.muted, lw: 1.3 }));
    G.line(c, cx - r, cy, cx + r, cy, P.faint, 1); G.line(c, cx, cy - r, cx, cy + r, P.faint, 1); pts.forEach(p => G.circle(c, cx + p[0] * r, cy - p[1] * r, 4.4, { fill: P.ink, stroke: P.bg, lw: 1.2 }));
    G.text(c, cap[i][0], cx, cy + r + 24, { color: cap[i][1], size: 13, weight: 700, align: 'center' }); }); };

/* ---------- 13. accuracy and repeatability as a live target (page 1.3): bias = mean error, spread = scatter, both in mm ---------- */
CH1.rand = seed => { let x = seed >>> 0 || 1; return () => { x ^= x << 13; x >>>= 0; x ^= x >> 17; x ^= x << 5; x >>>= 0; return x / 4294967296; }; };
CH1.shots = (bias, ang, spread, n, seed) => { const r = CH1.rand(seed), out = []; for (let i = 0; i < n; i++) { const u = max(1e-9, r()), v = r(), g = sqrt(-2 * Math.log(u)), a = 2 * PI * v;
  out.push([bias * cos(ang) + spread * 0.5 * g * cos(a), bias * sin(ang) + spread * 0.5 * g * sin(a)]); } return out; };
CH1.shotStats = pts => { const n = pts.length, m = [pts.reduce((a, p) => a + p[0], 0) / n, pts.reduce((a, p) => a + p[1], 0) / n]; return { mean: m, acc: hypot(m[0], m[1]), rep: max(...pts.map(p => hypot(p[0] - m[0], p[1] - m[1]))) }; };
D.shots = (c, W, H, s, P) => { G.clear(c, W, H, P); const R = s.R || 2, r = min(W * 0.3, H * 0.44), cx = r + 18, cy = H / 2, k = r / R;
  [1, 0.75, 0.5, 0.25].forEach(f => G.circle(c, cx, cy, r * f, { stroke: P.faint, lw: 1.2 })); G.line(c, cx - r, cy, cx + r, cy, P.faint, 1); G.line(c, cx, cy - r, cx, cy + r, P.faint, 1);
  G.text(c, `${RB.u(R * 0.5, 2)} mm`, cx + r * 0.5 + 3, cy - 4, { color: P.muted, size: 11 }); G.text(c, `${RB.u(R, 2)} mm`, cx + r + 3, cy - 4, { color: P.muted, size: 11 });
  G.circle(c, cx, cy, 7, { fill: P.sel }); const st = CH1.shotStats(s.pts), m = [cx + st.mean[0] * k, cy - st.mean[1] * k];
  G.circle(c, m[0], m[1], st.rep * k, { stroke: P.warn, lw: 1.6, dash: [5, 4] }); G.line(c, cx, cy, m[0], m[1], P.l2, 2);
  s.pts.forEach(p => G.circle(c, cx + p[0] * k, cy - p[1] * k, 4.6, { fill: P.ink, stroke: P.bg, lw: 1.2 })); G.line(c, m[0] - 7, m[1], m[0] + 7, m[1], P.l2, 2.4); G.line(c, m[0], m[1] - 7, m[0], m[1] + 7, P.l2, 2.4);
  const x0 = cx + r + 52, lab = (t, v, col, y) => { G.text(c, t, x0, y, { color: col, size: 13, weight: 700 }); G.text(c, v, x0, y + 20, { color: P.ink, size: 15, weight: 700, mono: true }); };
  lab(T('ความแม่นยำ (ระยะเฉลี่ยจากเป้า)', 'accuracy (mean miss)'), `${RB.u(st.acc, 3)} mm`, P.l2, cy - 60); lab(T('ทำซ้ำได้ (รัศมีกลุ่มจุด)', 'repeatability (spread radius)'), `± ${RB.u(st.rep, 3)} mm`, P.warn, cy + 6);
  G.text(c, T('● จุดที่ไปถึงจริง   ● เป้าที่สั่ง   + ค่าเฉลี่ย', '● where it went   ● commanded point   + mean'), 10, H - 10, { color: P.muted, size: 11.5 }); return st; };

/* ---------- figures exported to the slides (light palette): name → size in px and a draw call ---------- */
const ikq = (x, y, e = 1) => { const q = RB.ik2(CH1.LOOP.l1, CH1.LOOP.l2, x, y, e); return [q.t1, q.t2]; };
CH1.FIGS = {
  loop: { w: 560, h: 380, draw: (c, W, H, P) => D.loop(c, W, H, { tx: 0.45, ty: 0.6, mx: 0.45, my: 0.6, qd: ikq(0.45, 0.6), q: [rad(-8), rad(118)], sense: true, think: true, act: true, stage: 1 }, P) },
  chain: { w: 560, h: 380, draw: (c, W, H, P) => D.chain(c, W, H, { types: ['R', 'R', 'R'], q: [rad(62), rad(-48), rad(-42)], names: true, vars: false }, P) },
  jointR: { w: 300, h: 250, draw: (c, W, H, P) => D.chain(c, W, H, { types: ['R'], q: [rad(42)], win: [-0.22, 0.5, -0.22, 0.38], lw: 12 }, P) },
  jointP: { w: 300, h: 250, draw: (c, W, H, P) => D.chain(c, W, H, { types: ['P'], q: [0.2], win: [-0.22, 0.5, -0.3, 0.3], lw: 12 }, P) },
  nameA: { w: 280, h: 240, draw: (c, W, H, P) => D.chain(c, W, H, { types: ['R', 'R'], q: [rad(70), rad(-75)], win: [-0.25, 0.75, -0.25, 0.62], vars: false }, P) },
  nameB: { w: 280, h: 240, draw: (c, W, H, P) => D.chain(c, W, H, { types: ['R', 'P'], q: [rad(40), 0.18], win: [-0.25, 0.75, -0.25, 0.62], vars: false }, P) },
  nameC: { w: 280, h: 240, draw: (c, W, H, P) => D.chain(c, W, H, { types: ['P', 'R'], q: [0.16, rad(55)], win: [-0.25, 0.75, -0.25, 0.62], vars: false }, P) },
  nameD: { w: 280, h: 240, draw: (c, W, H, P) => D.chain(c, W, H, { types: ['R', 'R', 'P'], q: [rad(75), rad(-70), 0.12], win: [-0.25, 0.95, -0.25, 0.72], vars: false }, P) },
  config: { w: 520, h: 380, draw: (c, W, H, P) => D.chain(c, W, H, { types: ['R', 'R'], q: [rad(50), rad(-65)], win: [-0.2, 0.74, -0.16, 0.52], lw: 13 }, P) },
  dof: { w: 760, h: 360, draw: (c, W, H, P) => D.dof(c, W, H, { mode: 'both' }, P) },
  ws2: { w: 460, h: 460, draw: (c, W, H, P) => D.ws2(c, W, H, { l1: 0.4, l2: 0.3, t1: rad(55), t2: rad(70), R: 0.78 }, P) },
  ws2lim: { w: 460, h: 460, draw: (c, W, H, P) => D.ws2(c, W, H, { l1: 0.4, l2: 0.3, t1: rad(55), t2: rad(70), R: 0.78, lim: { a1: rad(-30), b1: rad(150), a2: rad(0), b2: rad(140) } }, P) },
  dex: { w: 460, h: 460, draw: (c, W, H, P) => D.dex(c, W, H, { l1: 0.4, l2: 0.3, l3: 0.1, x: 0.36, y: 0.3, phi: rad(20), R: 0.86 }, P) },
  dexEdge: { w: 460, h: 460, draw: (c, W, H, P) => D.dex(c, W, H, { l1: 0.4, l2: 0.3, l3: 0.1, x: 0.56, y: 0.42, phi: rad(35), R: 0.86 }, P) },
  path: { w: 560, h: 400, draw: (c, W, H, P) => D.path(c, W, H, { A: [0.2, 0.62], B: [0.8, 0.22], n: 2, mode: 'ptp', u: 0.5 }, P) },
  wrist: { w: 520, h: 380, draw: (c, W, H, P) => D.wrist(c, W, H, { yaw: rad(25), pitch: rad(30), roll: rad(35) }, P) },
  resol: { w: 560, h: 380, draw: (c, W, H, P) => D.resol(c, W, H, { l: 1, th: rad(90), bits: 4, u: 0.75, quant: false }, P) },
  resolHW: { w: 520, h: 330, draw: (c, W, H, P) => D.resol(c, W, H, { l: 1, th: rad(180), bits: 4, u: 0.3, quant: false, show: 'rot' }, P) },
  target: { w: 660, h: 270, draw: (c, W, H, P) => D.target(c, W, H, {}, P) },
  arrRRR: { w: 520, h: 420, draw: (c, W, H, P) => D.arr(c, W, H, { type: 'RRR', q: [30, 55, -70], title: false }, P) },
  arrRRP: { w: 520, h: 420, draw: (c, W, H, P) => D.arr(c, W, H, { type: 'RRP', q: [30, 30, 0.65], title: false }, P) },
  arrSCARA: { w: 520, h: 420, draw: (c, W, H, P) => D.arr(c, W, H, { type: 'SCARA', q: [35, 60, 0.2], title: false }, P) },
  arrRPP: { w: 520, h: 420, draw: (c, W, H, P) => D.arr(c, W, H, { type: 'RPP', q: [35, 0.6, 0.6], title: false }, P) },
  arrPPP: { w: 520, h: 420, draw: (c, W, H, P) => D.arr(c, W, H, { type: 'PPP', q: [0.6, 0.5, 0.45], title: false }, P) },
  fivebar: { w: 520, h: 380, draw: (c, W, H, P) => D.fivebar(c, W, H, { t1: rad(115), t2: rad(60) }, P) },
  ik2: { w: 520, h: 420, draw: (c, W, H, P) => D.ik2(c, W, H, { l1: 0.5, l2: 0.4, x: 0.55, y: 0.5, elbow: -1, grid: false }, P) }
};
global.CH1 = CH1;
})(typeof window !== 'undefined' ? window : globalThis);
