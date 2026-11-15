/* gl3d.js — a small WebGL renderer for the scene tables of scene3d.js (no library: the site has no build step and loads nothing extra).
   R3.draw(ctx2d, W, H, scene) renders the scene on one shared off-screen WebGL canvas and copies the picture onto the page's own
   2-D canvas, so every simulation keeps its ordinary canvas (labels and notes are then drawn on top in 2-D) and the page needs
   a single WebGL context however many simulations it has. R3.ok() is false when WebGL is not available (the pages then fall back
   to the flat drawings of chN.js). Lighting: two fixed lights and ambient light, per-pixel; translucent parts are drawn last, far to near. */
(function (global) {
'use strict';
const R3 = {}, RB = global.RB, S3 = global.S3;
let cv = null, gl = null, failed = false, U = {}, A = {}, prim = {}, tick = 0; const meshes = new Map();
const VS = 'attribute vec3 aP; attribute vec3 aN; uniform mat4 uVP; uniform mat3 uR; uniform vec3 uS; uniform vec3 uT; varying vec3 vN; varying vec3 vW;' +
  'void main(){ vec3 w = uR * (aP * uS) + uT; vW = w; vN = uR * (aN / uS); gl_Position = uVP * vec4(w, 1.0); }';
const FS = 'precision mediump float; uniform vec4 uC; uniform vec3 uEye; varying vec3 vN; varying vec3 vW;' +
  'void main(){ vec3 n = normalize(vN); vec3 e = normalize(uEye - vW); if (dot(n, e) < 0.0) n = -n;' +
  ' vec3 l1 = normalize(vec3(-0.35, -0.55, 0.76)); vec3 l2 = normalize(vec3(0.6, 0.35, 0.4));' +
  ' float d = 0.58 * max(dot(n, l1), 0.0) + 0.2 * max(dot(n, l2), 0.0); float sp = pow(max(dot(n, normalize(l1 + e)), 0.0), 40.0) * 0.22 * uC.a;' +
  ' gl_FragColor = vec4(uC.rgb * (0.46 + d) + vec3(sp), uC.a); }';
const sh = (type, src) => { const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); return s; };
const geo = (pos, nor, idx) => { const g = { p: gl.createBuffer(), n: gl.createBuffer(), i: gl.createBuffer(), count: idx.length };
  gl.bindBuffer(gl.ARRAY_BUFFER, g.p); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(pos), gl.STATIC_DRAW); gl.bindBuffer(gl.ARRAY_BUFFER, g.n); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(nor), gl.STATIC_DRAW);
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, g.i); gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(idx), gl.STATIC_DRAW); return g; };
/* unit shapes: cylinder and cone of radius 1 along z from −½ to ½ (cone tip at +½), sphere of radius 1, cube of side 1 */
function shapes() { const N = 28, P = [], Nn = [], I = [];
  for (let k = 0; k <= N; k++) { const a = 2 * Math.PI * k / N, c = Math.cos(a), s = Math.sin(a); P.push(c, s, -0.5, c, s, 0.5); Nn.push(c, s, 0, c, s, 0); } for (let k = 0; k < N; k++) I.push(2 * k, 2 * k + 2, 2 * k + 1, 2 * k + 1, 2 * k + 2, 2 * k + 3);
  for (const z of [-0.5, 0.5]) { const b = P.length / 3; P.push(0, 0, z); Nn.push(0, 0, z * 2); for (let k = 0; k <= N; k++) { const a = 2 * Math.PI * k / N; P.push(Math.cos(a), Math.sin(a), z); Nn.push(0, 0, z * 2); } for (let k = 0; k < N; k++) I.push(b, b + 1 + k, b + 2 + k); }
  prim.cyl = geo(P, Nn, I);
  { const P = [], Nn = [], I = []; for (let k = 0; k <= N; k++) { const a = 2 * Math.PI * k / N, c = Math.cos(a), s = Math.sin(a); P.push(c, s, -0.5, 0, 0, 0.5); Nn.push(c, s, 0.5, c, s, 0.5); } for (let k = 0; k < N; k++) I.push(2 * k, 2 * k + 2, 2 * k + 1);
    const b = P.length / 3; P.push(0, 0, -0.5); Nn.push(0, 0, -1); for (let k = 0; k <= N; k++) { const a = 2 * Math.PI * k / N; P.push(Math.cos(a), Math.sin(a), -0.5); Nn.push(0, 0, -1); } for (let k = 0; k < N; k++) I.push(b, b + 1 + k, b + 2 + k); prim.cone = geo(P, Nn, I); }
  { const P = [], I = [], M = 14, L = 22; for (let i = 0; i <= M; i++) { const t = Math.PI * i / M; for (let j = 0; j <= L; j++) { const a = 2 * Math.PI * j / L; P.push(Math.sin(t) * Math.cos(a), Math.sin(t) * Math.sin(a), Math.cos(t)); } }
    for (let i = 0; i < M; i++) for (let j = 0; j < L; j++) { const k = i * (L + 1) + j; I.push(k, k + L + 1, k + 1, k + 1, k + L + 1, k + L + 2); } prim.sph = geo(P, P, I); }
  { const P = [], Nn = [], I = [], F = [[[1, 0, 0], [0, 1, 0], [0, 0, 1]], [[-1, 0, 0], [0, 0, 1], [0, 1, 0]], [[0, 1, 0], [0, 0, 1], [1, 0, 0]], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], [[0, 0, 1], [1, 0, 0], [0, 1, 0]], [[0, 0, -1], [0, 1, 0], [1, 0, 0]]];
    F.forEach(([n, u, w]) => { const b = P.length / 3; [[-1, -1], [1, -1], [1, 1], [-1, 1]].forEach(([s, t]) => { P.push((n[0] + s * u[0] + t * w[0]) / 2, (n[1] + s * u[1] + t * w[1]) / 2, (n[2] + s * u[2] + t * w[2]) / 2); Nn.push(n[0], n[1], n[2]); }); I.push(b, b + 1, b + 2, b, b + 2, b + 3); }); prim.box = geo(P, Nn, I); } }
function init() { if (gl) return true; if (failed || typeof document === 'undefined') return false;
  try { cv = document.createElement('canvas'); cv.width = 16; cv.height = 16; gl = cv.getContext('webgl', { antialias: true, alpha: false, depth: true }); if (!gl) throw new Error('no WebGL');
    const pr = gl.createProgram(); gl.attachShader(pr, sh(gl.VERTEX_SHADER, VS)); gl.attachShader(pr, sh(gl.FRAGMENT_SHADER, FS)); gl.linkProgram(pr); if (!gl.getProgramParameter(pr, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(pr));
    gl.useProgram(pr); ['uVP', 'uR', 'uS', 'uT', 'uC', 'uEye'].forEach(n => { U[n] = gl.getUniformLocation(pr, n); }); A.p = gl.getAttribLocation(pr, 'aP'); A.n = gl.getAttribLocation(pr, 'aN'); gl.enableVertexAttribArray(A.p); gl.enableVertexAttribArray(A.n);
    shapes(); gl.enable(gl.DEPTH_TEST); gl.depthFunc(gl.LEQUAL); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    cv.addEventListener('webglcontextlost', e => { e.preventDefault(); gl = null; failed = true; }); return true; }
  catch (e) { console.warn('gl3d: WebGL is not available, the pages use their flat drawings', e); gl = null; failed = true; return false; } }
R3.ok = () => init();
/* object matrix whose z axis runs along d (the same rule as frameZ in render.lua): columns x, y, z */
const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const frame = (d, up) => { const z = RB.unit(d); let u = up || [0, 0, 1]; if (Math.abs(z[0] * u[0] + z[1] * u[1] + z[2] * u[2]) > 0.999) u = [1, 0, 0]; const x = RB.unit(RB.cross(u, z)), y = RB.cross(z, x); return [x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]]; };
const mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2], dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
/* turn one scene item into draw records {g, R, S, T, c, al} */
const X = {
  cyl: (o, L) => { const l = dist(o.a, o.b); if (l > 1e-9) L.push({ g: prim.cyl, R: frame([o.b[0] - o.a[0], o.b[1] - o.a[1], o.b[2] - o.a[2]]), S: [o.r, o.r, l], T: mid(o.a, o.b), c: o.c, al: o.al }); },
  sph: (o, L) => L.push({ g: prim.sph, R: I3, S: [o.r, o.r, o.r], T: o.p, c: o.c, al: o.al }),
  cone: (o, L) => { const l = dist(o.a, o.b); if (l > 1e-9) L.push({ g: prim.cone, R: frame([o.b[0] - o.a[0], o.b[1] - o.a[1], o.b[2] - o.a[2]]), S: [o.r, o.r, l], T: mid(o.a, o.b), c: o.c, al: o.al }); },
  box: (o, L) => { const l = dist(o.a, o.b); if (l > 1e-9) L.push({ g: prim.box, R: frame([o.b[0] - o.a[0], o.b[1] - o.a[1], o.b[2] - o.a[2]], o.up), S: [o.sx, o.sy, l], T: mid(o.a, o.b), c: o.c, al: o.al }); },
  bar: (o, L) => { X.cyl(o, L); X.sph({ p: o.a, r: o.r, c: o.c, al: o.al }, L); X.sph({ p: o.b, r: o.r, c: o.c, al: o.al }, L); },
  arrow: (o, L) => { const l = dist(o.a, o.b); if (l < 1e-9) return; const hl = Math.min(o.head || o.r * 6, l * 0.6), k = 1 - hl / l, m = [o.a[0] + (o.b[0] - o.a[0]) * k, o.a[1] + (o.b[1] - o.a[1]) * k, o.a[2] + (o.b[2] - o.a[2]) * k];
    X.cyl({ a: o.a, b: m, r: o.r, c: o.c, al: o.al }, L); X.cone({ a: m, b: o.b, r: o.r * 2.6, c: o.c, al: o.al }, L); },
  tube: (o, L) => { const n = o.pts.length; for (let i = 0; i < n - 1; i++) { X.cyl({ a: o.pts[i], b: o.pts[i + 1], r: o.r, c: o.c, al: o.al }, L); if (i && o.r >= 0.0045) X.sph({ p: o.pts[i], r: o.r, c: o.c, al: o.al }, L); }
    if (o.head && n > 1) { const a = o.pts[n - 1], b = o.pts[n - 2], d = RB.unit([a[0] - b[0], a[1] - b[1], a[2] - b[2]]); X.cone({ a, b: [a[0] + d[0] * o.r * 7, a[1] + d[1] * o.r * 7, a[2] + d[2] * o.r * 7], r: o.r * 2.8, c: o.c, al: o.al }, L); } },
  mesh: (o, L) => { let m = meshes.get(o.v); if (!m) { const v = o.v, ix = o.i, nor = new Float32Array(v.length); let cx = 0, cy = 0, cz = 0;
      for (let t = 0; t < ix.length; t += 3) { const a = ix[t] * 3, b = ix[t + 1] * 3, c = ix[t + 2] * 3, ux = v[b] - v[a], uy = v[b + 1] - v[a + 1], uz = v[b + 2] - v[a + 2], wx = v[c] - v[a], wy = v[c + 1] - v[a + 1], wz = v[c + 2] - v[a + 2], nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
        for (const q of [a, b, c]) { nor[q] += nx; nor[q + 1] += ny; nor[q + 2] += nz; } }
      for (let q = 0; q < v.length; q += 3) { cx += v[q]; cy += v[q + 1]; cz += v[q + 2]; if (!nor[q] && !nor[q + 1] && !nor[q + 2]) nor[q + 2] = 1; } const n = v.length / 3 || 1;
      m = geo(v, nor, ix); m.ctr = [cx / n, cy / n, cz / n]; meshes.set(o.v, m); } m.last = tick; L.push({ g: m, R: I3, S: [1, 1, 1], T: [0, 0, 0], c: o.c, al: o.al, ctr: m.ctr }); }
};
R3.draw = (ctx, W, H, s) => { if (!init()) return false; const dpr = RB.DPR || Math.min(global.devicePixelRatio || 1, 2), w = Math.round(W * dpr), h = Math.round(H * dpr);
  if (cv.width < w || cv.height < h) { cv.width = Math.max(cv.width, w); cv.height = Math.max(cv.height, h); }
  gl.viewport(0, 0, w, h); gl.enable(gl.SCISSOR_TEST); gl.scissor(0, 0, w, h); const bg = s.pal.bg; gl.clearColor(bg[0], bg[1], bg[2], 1); gl.depthMask(true); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  /* view-projection for the camera convention of scene3d.js: x to the left of the picture, y up, z forward */
  const cam = s.cam, ax = S3.axes(cam), f = S3.focal(cam, W, H), e = cam.eye, n = 0.05, fr = 80, a = (fr + n) / (fr - n), b = -2 * fr * n / (fr - n), px = -2 * f / W, py = 2 * f / H, tx = -(ax.x[0] * e[0] + ax.x[1] * e[1] + ax.x[2] * e[2]), ty = -(ax.y[0] * e[0] + ax.y[1] * e[1] + ax.y[2] * e[2]), tz = -(ax.z[0] * e[0] + ax.z[1] * e[1] + ax.z[2] * e[2]);
  gl.uniformMatrix4fv(U.uVP, false, [px * ax.x[0], py * ax.y[0], a * ax.z[0], ax.z[0], px * ax.x[1], py * ax.y[1], a * ax.z[1], ax.z[1], px * ax.x[2], py * ax.y[2], a * ax.z[2], ax.z[2], px * tx, py * ty, a * tz + b, tz]); gl.uniform3fv(U.uEye, e);
  const L = []; tick++; s.items.forEach(o => { const fn = X[o.t]; if (fn) fn(o, L); });
  const solid = L.filter(r => !(r.al < 1)), clear = L.filter(r => r.al < 1); clear.forEach(r => { r.d = dist(r.ctr || r.T, e); }); clear.sort((p, q) => q.d - p.d);
  let cur = null; const put = r => { if (r.g !== cur) { cur = r.g; gl.bindBuffer(gl.ARRAY_BUFFER, cur.p); gl.vertexAttribPointer(A.p, 3, gl.FLOAT, false, 0, 0); gl.bindBuffer(gl.ARRAY_BUFFER, cur.n); gl.vertexAttribPointer(A.n, 3, gl.FLOAT, false, 0, 0); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, cur.i); }
    gl.uniformMatrix3fv(U.uR, false, r.R); gl.uniform3fv(U.uS, r.S); gl.uniform3fv(U.uT, r.T); gl.uniform4f(U.uC, r.c[0], r.c[1], r.c[2], r.al < 1 ? r.al : 1); gl.drawElements(gl.TRIANGLES, cur.count, gl.UNSIGNED_SHORT, 0); };
  gl.disable(gl.BLEND); solid.forEach(put); gl.enable(gl.BLEND); gl.depthMask(false); clear.forEach(put); gl.depthMask(true);
  if (tick % 240 === 0) meshes.forEach((m, k) => { if (m.last < tick - 1200) { gl.deleteBuffer(m.p); gl.deleteBuffer(m.n); gl.deleteBuffer(m.i); meshes.delete(k); } });   // surfaces no scene has used for a while
  ctx.drawImage(cv, 0, cv.height - h, w, h, 0, 0, W, H); return true; };
/* without WebGL: a flat drawing of the same scene (strokes and discs painted far to near), so a page still shows the geometry */
R3.flat = (c, W, H, s) => { const cam = s.cam, f = S3.focal(cam, W, H), P = p => S3.proj(cam, W, H, p), L = [], rgb = (k, al) => `rgba(${Math.round(k[0] * 255)},${Math.round(k[1] * 255)},${Math.round(k[2] * 255)},${al < 1 ? al : 1})`;
  c.fillStyle = rgb(s.pal.bg); c.fillRect(0, 0, W, H);
  const seg = (a, b, r, k, al, head) => { const A = P(a), B = P(b); if (A[2] <= 0 || B[2] <= 0) return; L.push({ z: (A[2] + B[2]) / 2, d: () => { c.strokeStyle = rgb(k, al); c.lineWidth = Math.max(1, 2 * r * f / ((A[2] + B[2]) / 2)); c.lineCap = 'round'; c.beginPath(); c.moveTo(A[0], A[1]); c.lineTo(B[0], B[1]); c.stroke();
    if (head) { const g = Math.atan2(B[1] - A[1], B[0] - A[0]), h = Math.max(7, c.lineWidth * 3.2); c.fillStyle = rgb(k, al); c.beginPath(); c.moveTo(B[0], B[1]); c.lineTo(B[0] - h * Math.cos(g - 0.4), B[1] - h * Math.sin(g - 0.4)); c.lineTo(B[0] - h * Math.cos(g + 0.4), B[1] - h * Math.sin(g + 0.4)); c.closePath(); c.fill(); } } }); };
  const disc = (p, r, k, al) => { const A = P(p); if (A[2] <= 0) return; L.push({ z: A[2], d: () => { c.fillStyle = rgb(k, al); c.beginPath(); c.arc(A[0], A[1], Math.max(1.5, r * f / A[2]), 0, 2 * Math.PI); c.fill(); } }); };
  const tri = (a, b, d, k, al) => { const A = P(a), B = P(b), D = P(d); if (A[2] <= 0 || B[2] <= 0 || D[2] <= 0) return; L.push({ z: (A[2] + B[2] + D[2]) / 3, d: () => { c.fillStyle = rgb(k, (al < 1 ? al : 1) * 0.6); c.beginPath(); c.moveTo(A[0], A[1]); c.lineTo(B[0], B[1]); c.lineTo(D[0], D[1]); c.closePath(); c.fill(); } }); };
  s.items.forEach(o => { if (o.al < 0.6 && o.c[0] + o.c[1] + o.c[2] === 0) return;   // shadows
    if (o.t === 'cyl' || o.t === 'bar' || o.t === 'cone') seg(o.a, o.b, o.r, o.c, o.al); else if (o.t === 'box') seg(o.a, o.b, Math.min(o.sx, o.sy) / 2, o.c, o.al); else if (o.t === 'sph') disc(o.p, o.r, o.c, o.al);
    else if (o.t === 'arrow') seg(o.a, o.b, o.r, o.c, o.al, true); else if (o.t === 'tube') o.pts.forEach((p, i) => { if (i) seg(o.pts[i - 1], p, o.r, o.c, o.al, o.head && i === o.pts.length - 1); });
    else if (o.t === 'mesh' && o.i.length < 6000) for (let t = 0; t < o.i.length; t += 3) { const q = k => [o.v[3 * o.i[t + k]], o.v[3 * o.i[t + k] + 1], o.v[3 * o.i[t + k] + 2]]; tri(q(0), q(1), q(2), o.c, o.al); } });
  L.sort((a, b) => b.z - a.z).forEach(x => x.d()); };
global.R3 = R3;
})(typeof window !== 'undefined' ? window : globalThis);
