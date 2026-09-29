'use strict';
/* =============================================================================
 * microduck imu_to_dxl - IMU bench frontend
 *
 * Plain ES2020, no build step, no external dependencies (fully offline).
 *  - 3D attitude: hand-written WebGL1 renderer with inline GLSL string shaders.
 *  - Plots: 2D canvas, drawn once per requestAnimationFrame (never per message).
 *  - Live data: WebSocket "/ws" (auto reconnect + 1 s backoff), ports "/api/ports".
 *
 * Wire protocol (server -> browser): hello | sample | stats | status | error
 * Wire protocol (browser -> server): connect | disconnect | pause | config | command
 * ========================================================================== */
(function () {

// ----------------------------- constants -----------------------------

var RING_CAP    = 1200;   // sample ring buffer (12 s at 100 Hz)
var PLOT_SPAN_S = 6;      // seconds visible in the scrolling plots
var GAP_S       = 0.25;   // break a trace when the sample gap exceeds this
var HEAT_COLS   = 600;    // heat-map time columns (x axis), right edge = now
var HEAT_BINS   = 96;     // heat-map value bins (y axis)
var HEAT_COL_MS = 25;     // 40 columns per second
var STATS_MS    = 250;    // statistics refresh, ~4x/second
var TRACE_N     = 900;    // orientation-trace points (9 s at 100 Hz)
var SPHERE_R    = 46;     // orientation-trace sphere radius (world units)

var AXIS_C = ['#ff5c5c', '#4ade80', '#60a5fa'];     // X red, Y green, Z blue
var SIM_NAMES = ['static', 'sine', 'drift', 'noise', 'spin', 'sflp-wait', 'frozen'];

// The two scrolling plots. gyro uses the trunk-frame vector (mount corrected).
var ATT_SERIES = [
  { name: 'roll',  color: AXIS_C[0], get: function (s) { return s.euler[0]; } },
  { name: 'pitch', color: AXIS_C[1], get: function (s) { return s.euler[1]; } },
  { name: 'yaw',   color: AXIS_C[2], get: function (s) { return s.euler[2]; } }
];
var GYRO_SERIES = [
  { name: 'gyro x', color: AXIS_C[0], get: function (s) { return s.gyroTrunk[0]; } },
  { name: 'gyro y', color: AXIS_C[1], get: function (s) { return s.gyroTrunk[1]; } },
  { name: 'gyro z', color: AXIS_C[2], get: function (s) { return s.gyroTrunk[2]; } }
];

// Selectable heat-map quantities. def = initial [min,max] value range.
var HEAT_QTY = {
  gyro_x:    { label: 'gyro_x',    unit: 'dps', color: AXIS_C[0], get: function (s) { return s.gyroTrunk[0]; }, def: [-250, 250] },
  gyro_y:    { label: 'gyro_y',    unit: 'dps', color: AXIS_C[1], get: function (s) { return s.gyroTrunk[1]; }, def: [-250, 250] },
  gyro_z:    { label: 'gyro_z',    unit: 'dps', color: AXIS_C[2], get: function (s) { return s.gyroTrunk[2]; }, def: [-250, 250] },
  gravity_x: { label: 'gravity_x', unit: 'g',   color: AXIS_C[0], get: function (s) { return s.gravity[0]; },   def: [-1, 1] },
  gravity_y: { label: 'gravity_y', unit: 'g',   color: AXIS_C[1], get: function (s) { return s.gravity[1]; },   def: [-1, 1] },
  roll:      { label: 'roll',      unit: 'deg', color: AXIS_C[0], get: function (s) { return s.euler[0]; },     def: [-180, 180] },
  pitch:     { label: 'pitch',     unit: 'deg', color: AXIS_C[1], get: function (s) { return s.euler[1]; },     def: [-180, 180] }
};

var FONT_2D = '10px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace';

// ----------------------------- small helpers -----------------------------

function $(id) { return document.getElementById(id); }

function num(v, dflt) {
  var x = (typeof v === 'number') ? v : parseFloat(v);
  return (typeof x === 'number' && isFinite(x)) ? x : dflt;
}
function int(v, dflt) {
  var x = num(v, NaN);
  return isFinite(x) ? Math.round(x) : dflt;
}
function vec3(v, dflt) {
  if (!v || v.length < 3) return dflt ? dflt.slice() : [0, 0, 0];
  return [num(v[0], 0), num(v[1], 0), num(v[2], 0)];
}
function vec4(v, dflt) {
  if (!v || v.length < 4) return dflt ? dflt.slice() : [1, 0, 0, 0];
  return [num(v[0], 1), num(v[1], 0), num(v[2], 0), num(v[3], 0)];
}
function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }

// adaptive fixed-point formatting: keeps jitter digits meaningful at any scale
function fmt(v, digits) {
  if (!isFinite(v)) return '--';
  if (digits !== undefined) return v.toFixed(digits);
  var a = Math.abs(v);
  var d = a >= 1000 ? 0 : a >= 100 ? 1 : a >= 10 ? 2 : a >= 1 ? 3 : 4;
  return v.toFixed(d);
}
function localStamp() {
  var d = new Date(), p = function (x) { return (x < 10 ? '0' : '') + x; };
  return d.getFullYear() + p(d.getMonth() + 1) + p(d.getDate()) + '_' +
         p(d.getHours()) + p(d.getMinutes()) + p(d.getSeconds());
}
function nowMs() { return (window.performance && performance.now) ? performance.now() : Date.now(); }

// ----------------------------- ring buffer -----------------------------

// Fixed-capacity chronological buffer. at(0) is the oldest retained sample.
function Ring(cap) {
  this.cap = cap;
  this.buf = new Array(cap);
  this.n = 0;
  this.head = 0;
  this.total = 0;
}
Ring.prototype.push = function (v) {
  this.buf[this.head] = v;
  this.head = (this.head + 1) % this.cap;
  if (this.n < this.cap) this.n++;
  this.total++;
};
Ring.prototype.at = function (i) {
  var start = (this.head - this.n + this.cap) % this.cap;
  return this.buf[(start + i) % this.cap];
};
Ring.prototype.latest = function (k) {
  k = k || 0;
  return (this.n > k) ? this.at(this.n - 1 - k) : null;
};
Ring.prototype.clear = function () {
  this.buf = new Array(this.cap);
  this.n = 0;
  this.head = 0;
  this.total = 0;
};
// Index range [i0,i1) of samples with t in [t0,t1]. Assumes t is monotonic,
// which the server clock is; a rare out-of-order sample is clipped, not fatal.
Ring.prototype.window = function (t0, t1) {
  var i0 = 0, i1 = this.n;
  while (i0 < i1 && this.at(i0).t < t0) i0++;
  while (i1 > i0 && this.at(i1 - 1).t > t1) i1--;
  return [i0, i1];
};

// ----------------------------- state -----------------------------

var state = {
  ws: null,
  wsOpen: false,
  reconnectTimer: 0,
  intent: { connected: false },   // what the user asked for (survives reconnects)
  paused: false,
  connected: false,
  device: null,
  link: null,
  protoLock: 'none',
  ring: new Ring(RING_CAP),
  pending: [],                    // samples not yet binned into a heat column
  tNow: 0,
  lastSampleMs: 0,
  lastSample: null,
  achievedHz: NaN,
  linkStats: null,
  counters: { prev: null, missing: 0, jumps: 0, dups: 0, errs: 0 },
  stale: { run: 0, max: 0, total: 0 },
  recvTimes: [],
  flags: 0,
  flagNames: [],
  flagsSeen: {}
};

// ----------------------------- quaternion / matrix math -----------------------------

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
function dot3(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function norm3(v) {
  var n = Math.sqrt(dot3(v, v));
  return n > 1e-12 ? [v[0] / n, v[1] / n, v[2] / n] : [0, 0, 0];
}

// Quaternion (w,x,y,z) -> rotation matrix, column-major for WebGL.
// Convention: active body->world rotation, world is Z-up, so the identity
// quaternion means "board flat, +Z up". The quaternion is normalised first so a
// slightly denormalised telemetry value cannot shear the model.
function quatToMat4(q) {
  var w = num(q[0], 1), x = num(q[1], 0), y = num(q[2], 0), z = num(q[3], 0);
  var n = Math.sqrt(w * w + x * x + y * y + z * z);
  if (!(n > 1e-9)) { w = 1; x = y = z = 0; } else { w /= n; x /= n; y /= n; z /= n; }
  var xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z;
  var wx = w * x, wy = w * y, wz = w * z;
  var m = new Float32Array(16);
  m[0] = 1 - 2 * (yy + zz); m[1] = 2 * (xy + wz);     m[2] = 2 * (xz - wy);     m[3] = 0;
  m[4] = 2 * (xy - wz);     m[5] = 1 - 2 * (xx + zz); m[6] = 2 * (yz + wx);     m[7] = 0;
  m[8] = 2 * (xz + wy);     m[9] = 2 * (yz - wx);     m[10] = 1 - 2 * (xx + yy); m[11] = 0;
  m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
  return m;
}
function mat4Rotate(m, v) {
  return [
    m[0] * v[0] + m[4] * v[1] + m[8] * v[2],
    m[1] * v[0] + m[5] * v[1] + m[9] * v[2],
    m[2] * v[0] + m[6] * v[1] + m[10] * v[2]
  ];
}
function perspective(fovyDeg, aspect, near, far) {
  var f = 1 / Math.tan(fovyDeg * Math.PI / 360);
  var nf = 1 / (near - far);
  return new Float32Array([
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (far + near) * nf, -1,
    0, 0, 2 * far * near * nf, 0
  ]);
}
function lookAt(eye, center, up) {
  var z = norm3([eye[0] - center[0], eye[1] - center[1], eye[2] - center[2]]);
  var x = norm3(cross(up, z));
  var y = cross(z, x);
  return new Float32Array([
    x[0], y[0], z[0], 0,
    x[1], y[1], z[1], 0,
    x[2], y[2], z[2], 0,
    -dot3(x, eye), -dot3(y, eye), -dot3(z, eye), 1
  ]);
}
function mat4Mul(a, b) {   // returns a * b (column-major)
  var o = new Float32Array(16);
  for (var c = 0; c < 4; c++) {
    for (var r = 0; r < 4; r++) {
      o[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] +
                     a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
    }
  }
  return o;
}

// angle between two directions, in degrees (used for the gravity-vs-up readout)
function angleBetween(a, b) {
  var ua = norm3(a), ub = norm3(b);
  if (!dot3(ua, ua) || !dot3(ub, ub)) return NaN;
  return Math.acos(clamp(dot3(ua, ub), -1, 1)) * 180 / Math.PI;
}

// ----------------------------- WebGL board view -----------------------------

var VS_SRC = [
  'attribute vec3 aPos;',
  'attribute vec3 aCol;',
  'uniform mat4 uMVP;',
  'varying vec3 vCol;',
  'void main() {',
  '  vCol = aCol;',
  '  gl_Position = uMVP * vec4(aPos, 1.0);',
  '}'
].join('\n');

var FS_SRC = [
  'precision mediump float;',
  'varying vec3 vCol;',
  'uniform float uAlpha;',
  'void main() {',
  '  gl_FragColor = vec4(vCol * uAlpha, 1.0);',
  '}'
].join('\n');

function hex2rgb(hex) {
  var v = parseInt(hex.slice(1), 16);
  return [((v >> 16) & 255) / 255, ((v >> 8) & 255) / 255, (v & 255) / 255];
}

// Tiny interleaved (pos + colour) geometry builder.
function Geom() { this.pos = []; this.col = []; }
Geom.prototype.v = function (x, y, z, c) { this.pos.push(x, y, z); this.col.push(c[0], c[1], c[2]); };
Geom.prototype.line = function (a, b, c) { this.v(a[0], a[1], a[2], c); this.v(b[0], b[1], b[2], c); };
Geom.prototype.tri = function (a, b, c, col) {
  this.v(a[0], a[1], a[2], col); this.v(b[0], b[1], b[2], col); this.v(c[0], c[1], c[2], col);
};
Geom.prototype.quad = function (a, b, c, d, col) { this.tri(a, b, c, col); this.tri(a, c, d, col); };
Geom.prototype.box = function (cx, cy, cz, sx, sy, sz, cols) {
  var x0 = cx - sx / 2, x1 = cx + sx / 2;
  var y0 = cy - sy / 2, y1 = cy + sy / 2;
  var z0 = cz - sz / 2, z1 = cz + sz / 2;
  this.quad([x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1], cols.px);
  this.quad([x0, y1, z0], [x0, y0, z0], [x0, y0, z1], [x0, y1, z1], cols.nx);
  this.quad([x1, y1, z0], [x0, y1, z0], [x0, y1, z1], [x1, y1, z1], cols.py);
  this.quad([x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1], cols.ny);
  this.quad([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1], cols.pz);   // top
  this.quad([x0, y1, z0], [x1, y1, z0], [x1, y0, z0], [x0, y0, z0], cols.nz);   // bottom
};
Geom.prototype.boxEdges = function (cx, cy, cz, sx, sy, sz, c) {
  var x0 = cx - sx / 2, x1 = cx + sx / 2;
  var y0 = cy - sy / 2, y1 = cy + sy / 2;
  var z0 = cz - sz / 2, z1 = cz + sz / 2;
  var p = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
           [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
  var e = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4],
           [0, 4], [1, 5], [2, 6], [3, 7]];
  for (var i = 0; i < e.length; i++) this.line(p[e[i][0]], p[e[i][1]], c);
};
Geom.prototype.arrow = function (from, dir, len, c) {
  var d = norm3(dir);
  if (!dot3(d, d)) return;
  var tip = [from[0] + d[0] * len, from[1] + d[1] * len, from[2] + d[2] * len];
  this.line(from, tip, c);
  var up = Math.abs(d[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1];
  var u = norm3(cross(up, d));
  var vv = cross(d, u);
  var hl = len * 0.16, hw = hl * 0.6;
  var base = [tip[0] - d[0] * hl, tip[1] - d[1] * hl, tip[2] - d[2] * hl];
  var s = [[1, 0], [-1, 0], [0, 1], [0, -1]];
  for (var i = 0; i < s.length; i++) {
    this.line(tip, [base[0] + u[0] * hw * s[i][0] + vv[0] * hw * s[i][1],
                    base[1] + u[1] * hw * s[i][0] + vv[1] * hw * s[i][1],
                    base[2] + u[2] * hw * s[i][0] + vv[2] * hw * s[i][1]], c);
  }
};
Geom.prototype.circle = function (radius, plane, seg, c) {
  for (var i = 0; i < seg; i++) {
    var a0 = i / seg * Math.PI * 2, a1 = (i + 1) / seg * Math.PI * 2;
    var p0, p1;
    if (plane === 0) {          // XY
      p0 = [Math.cos(a0) * radius, Math.sin(a0) * radius, 0];
      p1 = [Math.cos(a1) * radius, Math.sin(a1) * radius, 0];
    } else if (plane === 1) {   // YZ
      p0 = [0, Math.cos(a0) * radius, Math.sin(a0) * radius];
      p1 = [0, Math.cos(a1) * radius, Math.sin(a1) * radius];
    } else {                    // XZ
      p0 = [Math.cos(a0) * radius, 0, Math.sin(a0) * radius];
      p1 = [Math.cos(a1) * radius, 0, Math.sin(a1) * radius];
    }
    this.line(p0, p1, c);
  }
};
Geom.prototype.data = function () {
  return { pos: new Float32Array(this.pos), col: new Float32Array(this.col), count: this.pos.length / 3 };
};

// The board is mounted so that trunk = [+raw_z, +raw_y, -raw_x]: chip +Z = trunk +X
// (a +90 degree rotation about Y), chip +X = trunk -Z, chip +Y = trunk +Y.  Same
// constant as microduck's SflpDecoder::DEFAULT_MOUNT and host/bus.py's DEFAULT_MOUNT.
// The board mesh below is authored flat in the board's own frame (PCB in local X-Y,
// normal +Z), so it has to be pre-rotated by this or a correctly mounted vertical
// board renders horizontal.  Never fold this into the gravity arrow or the trace --
// those are trunk-frame quantities and would get the rotation twice.
var BOARD_MOUNT = [Math.SQRT1_2, 0, Math.SQRT1_2, 0];

function BoardView(canvas) {
  this.canvas = canvas;
  this.gl = null;
  this.ok = false;
  this.buffers = {};
  this.tracePts = new Float32Array(TRACE_N * 3);
  this.traceHead = 0;
  this.traceN = 0;
  this.traceSeq = -1;
  this.gravDev = NaN;

  var gl = null;
  try {
    gl = canvas.getContext('webgl', { antialias: true, alpha: false, depth: true, preserveDrawingBuffer: false })
      || canvas.getContext('experimental-webgl', { antialias: true, alpha: false });
  } catch (e) { gl = null; }
  if (!gl) return;
  this.gl = gl;

  var vs = gl.createShader(gl.VERTEX_SHADER);
  gl.shaderSource(vs, VS_SRC); gl.compileShader(vs);
  var fs = gl.createShader(gl.FRAGMENT_SHADER);
  gl.shaderSource(fs, FS_SRC); gl.compileShader(fs);
  if (!gl.getShaderParameter(vs, gl.COMPILE_STATUS) || !gl.getShaderParameter(fs, gl.COMPILE_STATUS)) {
    this.err = 'shader compile failed';
    return;
  }
  var prog = gl.createProgram();
  gl.attachShader(prog, vs); gl.attachShader(prog, fs); gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) { this.err = 'program link failed'; return; }
  gl.useProgram(prog);
  this.prog = prog;
  this.aPos = gl.getAttribLocation(prog, 'aPos');
  this.aCol = gl.getAttribLocation(prog, 'aCol');
  this.uMVP = gl.getUniformLocation(prog, 'uMVP');
  this.uAlpha = gl.getUniformLocation(prog, 'uAlpha');
  gl.enableVertexAttribArray(this.aPos);
  gl.enableVertexAttribArray(this.aCol);
  gl.enable(gl.DEPTH_TEST);
  gl.disable(gl.CULL_FACE);

  /* ---- static: world frame (ground grid + wire sphere of the trace) ---- */
  var gw = new Geom();
  var gridC = hex2rgb('#16202c'), gridMain = hex2rgb('#22334a');
  var GZ = -38, EXT = 100, STEP = 20;
  for (var i = -5; i <= 5; i++) {
    var c = (i === 0) ? gridMain : gridC;
    gw.line([i * STEP, -EXT, GZ], [i * STEP, EXT, GZ], c);
    gw.line([-EXT, i * STEP, GZ], [EXT, i * STEP, GZ], c);
  }
  var sphC = hex2rgb('#1b2836');
  gw.circle(SPHERE_R, 0, 48, sphC);
  gw.circle(SPHERE_R, 1, 48, sphC);
  gw.circle(SPHERE_R, 2, 48, sphC);
  this.mkBuffer('world', gw.data(), gl.LINES);

  /* ---- static: fixed world "up" reference arrow (+Z) ---- */
  var gu = new Geom();
  gu.arrow([0, 0, 0], [0, 0, 1], 44, hex2rgb('#22d3ee'));
  this.mkBuffer('up', gu.data(), gl.LINES);

  /* ---- static: body frame (board 60x40x4 + connector nub + axes) ---- */
  var top = hex2rgb('#1f7f8c');
  var gb = new Geom();
  gb.box(0, 0, 0, 60, 40, 4, {
    pz: top, nz: hex2rgb('#232c38'), px: hex2rgb('#3a4552'),
    nx: hex2rgb('#303946'), py: hex2rgb('#3f4a58'), ny: hex2rgb('#2c343f')
  });
  gb.box(-24, 0, 4.5, 10, 8, 5, {            // connector nub on the top face
    pz: hex2rgb('#d7dee7'), nz: hex2rgb('#374151'), px: hex2rgb('#4b5563'),
    nx: hex2rgb('#414b58'), py: hex2rgb('#556070'), ny: hex2rgb('#3b444f')
  });
  this.mkBuffer('board', gb.data(), gl.TRIANGLES);

  var gl2 = new Geom();
  gl2.boxEdges(0, 0, 0, 60, 40, 4, hex2rgb('#93a7bd'));
  gl2.boxEdges(-24, 0, 4.5, 10, 8, 5, hex2rgb('#cbd5e1'));
  gl2.arrow([0, 0, 0], [1, 0, 0], 46, hex2rgb(AXIS_C[0]));   // body +X red
  gl2.arrow([0, 0, 0], [0, 1, 0], 36, hex2rgb(AXIS_C[1]));   // body +Y green
  gl2.arrow([0, 0, 0], [0, 0, 1], 30, hex2rgb(AXIS_C[2]));   // body +Z blue
  gl2.line([0, 0, 0], [-18, 0, 0], hex2rgb('#7f2b2b'));
  gl2.line([0, 0, 0], [0, -18, 0], hex2rgb('#2b6b3f'));
  gl2.line([0, 0, 0], [0, 0, -16], hex2rgb('#2b4a7f'));
  this.mkBuffer('bodylines', gl2.data(), gl.LINES);

  this.ok = true;
}

BoardView.prototype.mkBuffer = function (name, data, mode) {
  var gl = this.gl;
  var buf = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, buf);
  gl.bufferData(gl.ARRAY_BUFFER, data.pos, gl.STATIC_DRAW);
  var cbuf = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, cbuf);
  gl.bufferData(gl.ARRAY_BUFFER, data.col, gl.STATIC_DRAW);
  this.buffers[name] = { pos: buf, col: cbuf, count: data.count, mode: mode, dynamic: false };
};

BoardView.prototype.upBuffer = function (name, pos, col, mode) {
  var gl = this.gl, b = this.buffers[name];
  if (!b) {
    b = this.buffers[name] = { pos: gl.createBuffer(), col: gl.createBuffer(), count: 0, mode: mode, dynamic: true };
  }
  gl.bindBuffer(gl.ARRAY_BUFFER, b.pos);
  gl.bufferData(gl.ARRAY_BUFFER, pos, gl.DYNAMIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, b.col);
  gl.bufferData(gl.ARRAY_BUFFER, col, gl.DYNAMIC_DRAW);
  b.count = pos.length / 3;
  b.mode = mode;
};

BoardView.prototype.drawBuffer = function (name, mvp, alpha) {
  var gl = this.gl, b = this.buffers[name];
  if (!b || !b.count) return;
  gl.uniformMatrix4fv(this.uMVP, false, mvp);
  gl.uniform1f(this.uAlpha, alpha);
  gl.bindBuffer(gl.ARRAY_BUFFER, b.pos);
  gl.vertexAttribPointer(this.aPos, 3, gl.FLOAT, false, 0, 0);
  gl.bindBuffer(gl.ARRAY_BUFFER, b.col);
  gl.vertexAttribPointer(this.aCol, 3, gl.FLOAT, false, 0, 0);
  gl.drawArrays(b.mode, 0, b.count);
};

// Push the world-frame direction of the board +Z axis into the trace ring.
BoardView.prototype.pushTrace = function (tip) {
  var o = this.traceHead * 3;
  this.tracePts[o] = tip[0]; this.tracePts[o + 1] = tip[1]; this.tracePts[o + 2] = tip[2];
  this.traceHead = (this.traceHead + 1) % TRACE_N;
  if (this.traceN < TRACE_N) this.traceN++;
};

BoardView.prototype.draw = function (sample, seq) {
  var gl = this.gl;
  if (!this.ok || !gl) return;
  var c = this.canvas;
  var w = c.clientWidth, h = c.clientHeight;
  if (w <= 0 || h <= 0) return;                      // panel hidden -> skip
  var dpr = Math.min(2, window.devicePixelRatio || 1);
  var bw = Math.round(w * dpr), bh = Math.round(h * dpr);
  if (c.width !== bw || c.height !== bh) { c.width = bw; c.height = bh; }
  gl.viewport(0, 0, c.width, c.height);
  gl.clearColor(0.027, 0.043, 0.063, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.useProgram(this.prog);

  var proj = perspective(42, c.width / Math.max(1, c.height), 1, 900);
  var view = lookAt([112, -138, 84], [0, 0, 0], [0, 0, 1]);
  var vp = mat4Mul(proj, view);

  this.drawBuffer('world', vp, 0.85);
  this.drawBuffer('up', vp, 0.95);

  // trunk attitude (bus.py already applied the mount correction).  Gravity is a
  // trunk-frame quantity, so the gravity arrow and the +Z trace keep using this one.
  var trunkModel = (sample && sample.quat) ? quatToMat4(sample.quat) : quatToMat4([1, 0, 0, 0]);
  // board model: trunk attitude * chip->trunk mount, because the mesh is authored in
  // the board's own frame.  Without BOARD_MOUNT a vertical board draws as flat.
  var model = mat4Mul(trunkModel, quatToMat4(BOARD_MOUNT));
  var mvp = mat4Mul(vp, model);
  this.drawBuffer('board', mvp, 1.0);
  this.drawBuffer('bodylines', mvp, 1.0);

  // dynamic world-frame geometry: measured gravity, +Z trace, tip marker
  var gp = [], gc = [];
  var orange = hex2rgb('#fb923c'), traceC = hex2rgb('#38bdf8');
  var tipWorld = [0, 0, 0];
  if (sample) {
    var zAxis = mat4Rotate(trunkModel, [0, 0, 1]);
    tipWorld = [zAxis[0] * SPHERE_R, zAxis[1] * SPHERE_R, zAxis[2] * SPHERE_R];
    if (seq !== this.traceSeq) { this.traceSeq = seq; this.pushTrace(tipWorld); }

    // gravity is reported in the trunk frame -> rotate it into the world frame
    var gBody = norm3(sample.gravity);
    var gWorld = [trunkModel[0] * gBody[0] + trunkModel[4] * gBody[1] + trunkModel[8] * gBody[2],
                  trunkModel[1] * gBody[0] + trunkModel[5] * gBody[1] + trunkModel[9] * gBody[2],
                  trunkModel[2] * gBody[0] + trunkModel[6] * gBody[1] + trunkModel[10] * gBody[2]];
    this.gravDev = angleBetween(gWorld, [0, 0, -1]);
    var arrowFrom = [0, 0, 0];
    var arrowDir = [gWorld[0], gWorld[1], gWorld[2]];
    var gg = new Geom();
    gg.arrow(arrowFrom, arrowDir, 44, orange);
    gp = gg.pos; gc = gg.col;
  } else {
    this.gravDev = NaN;
  }
  // small world-frame marker at the current +Z tip
  var m = 6, wc = hex2rgb('#e2e8f0');
  var cross3 = [[m, 0, 0], [0, m, 0], [0, 0, m]];
  for (var i = 0; i < 3; i++) {
    gp.push(tipWorld[0] - cross3[i][0], tipWorld[1] - cross3[i][1], tipWorld[2] - cross3[i][2]);
    gp.push(tipWorld[0] + cross3[i][0], tipWorld[1] + cross3[i][1], tipWorld[2] + cross3[i][2]);
    for (var k = 0; k < 2; k++) gc.push(wc[0], wc[1], wc[2]);
  }
  this.upBuffer('dyn', new Float32Array(gp), new Float32Array(gc), gl.LINES);
  this.drawBuffer('dyn', vp, 1.0);

  // orientation trace (oldest -> newest) drawn as a faint line strip on the sphere
  if (this.traceN > 1) {
    var tp = new Float32Array(this.traceN * 3), tc = new Float32Array(this.traceN * 3);
    var start = (this.traceHead - this.traceN + TRACE_N) % TRACE_N;
    var fade = hex2rgb('#0ea5b7');
    for (var j = 0; j < this.traceN; j++) {
      var src = ((start + j) % TRACE_N) * 3;
      tp[j * 3] = this.tracePts[src]; tp[j * 3 + 1] = this.tracePts[src + 1]; tp[j * 3 + 2] = this.tracePts[src + 2];
      var f = 0.35 + 0.65 * (j / this.traceN);       // older points are fainter
      tc[j * 3] = fade[0] * f; tc[j * 3 + 1] = fade[1] * f; tc[j * 3 + 2] = fade[2] * f;
    }
    this.upBuffer('trace', tp, tc, gl.LINE_STRIP);
    this.drawBuffer('trace', vp, 0.75);
  }
};

// ----------------------------- heat-map waterfall -----------------------------

// 256-entry colour LUT (black -> blue -> teal -> green -> yellow -> white).
function makeLUT() {
  var stops = [
    [0.00, [6, 10, 18]], [0.20, [26, 55, 130]], [0.40, [13, 116, 144]],
    [0.60, [34, 197, 94]], [0.80, [250, 204, 21]], [1.00, [255, 250, 240]]
  ];
  var lut = new Uint8Array(256 * 3);
  for (var i = 0; i < 256; i++) {
    var t = i / 255, a = stops[0], b = stops[stops.length - 1];
    for (var s = 0; s < stops.length - 1; s++) {
      if (t >= stops[s][0] && t <= stops[s + 1][0]) { a = stops[s]; b = stops[s + 1]; break; }
    }
    var f = (b[0] > a[0]) ? (t - a[0]) / (b[0] - a[0]) : 0;
    lut[i * 3]     = Math.round(a[1][0] + (b[1][0] - a[1][0]) * f);
    lut[i * 3 + 1] = Math.round(a[1][1] + (b[1][1] - a[1][1]) * f);
    lut[i * 3 + 2] = Math.round(a[1][2] + (b[1][2] - a[1][2]) * f);
  }
  return lut;
}

function Heatmap(canvas, cols, bins) {
  this.canvas = canvas;
  this.ctx = canvas.getContext('2d');
  this.w = cols;
  this.h = bins;
  this.counts = new Uint16Array(cols * bins);
  this.lut = makeLUT();
  this.off = document.createElement('canvas');
  this.off.width = cols; this.off.height = bins;
  this.offCtx = this.off.getContext('2d');
  this.img = this.offCtx.createImageData(cols, bins);
  this.lo = 1; this.hi = -1;
  this.dirty = true;
  this.W = 0; this.H = 0; this.dpr = 1;
}

// Shift the image one column to the left and write the samples of this slice
// into the newest (right) column. Column c=0 is the oldest, c=w-1 is "now".
Heatmap.prototype.push = function (samples, qtyKey) {
  var q = HEAT_QTY[qtyKey] || HEAT_QTY.gyro_x;
  var w = this.w, h = this.h, counts = this.counts;
  if (!(this.hi > this.lo)) { this.lo = q.def[0]; this.hi = q.def[1]; }

  counts.copyWithin(0, h, w * h);       // move columns 1..w-1 to 0..w-2
  counts.fill(0, (w - 1) * h, w * h);   // clear the newest column

  // Range adaptation: expand immediately so nothing is clipped, then contract
  // slowly (1% per column) so the display keeps useful resolution.
  var lo = Infinity, hi = -Infinity, i, v;
  for (i = 0; i < samples.length; i++) {
    v = q.get(samples[i]);
    if (isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; }
  }
  if (isFinite(lo)) {
    if (lo < this.lo) this.lo = lo;
    if (hi > this.hi) this.hi = hi;
    this.lo += (lo - this.lo) * 0.01;
    this.hi += (hi - this.hi) * 0.01;
    if (!(this.hi - this.lo > 1e-6)) { this.lo -= 0.5; this.hi += 0.5; }
  }
  var span = this.hi - this.lo;
  var base = (w - 1) * h;
  for (i = 0; i < samples.length; i++) {
    v = q.get(samples[i]);
    if (!isFinite(v)) continue;
    var b = Math.floor((v - this.lo) / span * h);
    if (b < 0) b = 0; else if (b >= h) b = h - 1;
    var idx = base + b;
    if (counts[idx] < 65535) counts[idx]++;
  }
  this.dirty = true;
};

Heatmap.prototype.clear = function (qtyKey) {
  var q = HEAT_QTY[qtyKey] || HEAT_QTY.gyro_x;
  this.counts.fill(0);
  this.lo = q.def[0]; this.hi = q.def[1];
  this.dirty = true;
};

// Colour a pixel per bin: density is normalised per time column so a single
// sample in a slice is still visible, then square-root boosted for contrast.
Heatmap.prototype.renderImage = function () {
  var w = this.w, h = this.h, counts = this.counts, img = this.img, lut = this.lut;
  for (var c = 0; c < w; c++) {
    var base = c * h, mx = 0;
    for (var b = 0; b < h; b++) { var v = counts[base + b]; if (v > mx) mx = v; }
    var inv = mx > 0 ? 1 / mx : 0;
    for (b = 0; b < h; b++) {
      var t = Math.sqrt(counts[base + b] * inv);
      var li = (t * 255) | 0; if (li < 0) li = 0; else if (li > 255) li = 255;
      var o = ((h - 1 - b) * w + c) * 4;     // flip: lowest bin at the bottom
      img.data[o] = lut[li * 3]; img.data[o + 1] = lut[li * 3 + 1]; img.data[o + 2] = lut[li * 3 + 2];
      img.data[o + 3] = 255;
    }
  }
  this.offCtx.putImageData(img, 0, 0);
  this.dirty = false;
};

Heatmap.prototype.sync = function () {
  var c = this.canvas, w = c.clientWidth, h = c.clientHeight;
  if (w <= 0 || h <= 0) return false;
  var dpr = Math.min(2, window.devicePixelRatio || 1);
  var bw = Math.round(w * dpr), bh = Math.round(h * dpr);
  if (c.width !== bw || c.height !== bh) { c.width = bw; c.height = bh; }
  this.W = w; this.H = h; this.dpr = dpr;
  return true;
};

Heatmap.prototype.draw = function (qtyKey, latest) {
  if (!this.sync()) return;
  var ctx = this.ctx, W = this.W, H = this.H, dpr = this.dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  var q = HEAT_QTY[qtyKey] || HEAT_QTY.gyro_x;
  var mL = 48, mR = 8, mT = 8, mB = 13;
  var pw = Math.max(10, W - mL - mR), ph = Math.max(10, H - mT - mB);

  ctx.fillStyle = '#0b1119';
  ctx.fillRect(0, 0, W, H);

  if (this.dirty) this.renderImage();
  if (this.hi > this.lo) {
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.off, 0, 0, this.w, this.h, mL, mT, pw, ph);
  }

  // y grid + value labels (bin range)
  ctx.font = FONT_2D;
  ctx.textBaseline = 'middle';
  ctx.textAlign = 'right';
  var ticks = 5;
  for (var k = 0; k < ticks; k++) {
    var t = k / (ticks - 1);
    var y = Math.round(mT + t * ph) + 0.5;
    var val = this.hi - t * (this.hi - this.lo);
    ctx.strokeStyle = (k === 0 || k === ticks - 1) ? '#2b3a4d' : '#1a2531';
    ctx.beginPath(); ctx.moveTo(mL, y); ctx.lineTo(mL + pw, y); ctx.stroke();
    ctx.fillStyle = '#7f8ea3';
    ctx.fillText(fmt(val), mL - 4, y);
  }
  // zero line (offset reference)
  if (this.lo < 0 && this.hi > 0) {
    var yz = Math.round(mT + (1 - (0 - this.lo) / (this.hi - this.lo)) * ph) + 0.5;
    ctx.strokeStyle = '#64748b';
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(mL, yz); ctx.lineTo(mL + pw, yz); ctx.stroke();
    ctx.setLineDash([]);
  }
  // x grid: one line per second, right edge = now
  var spanS = this.w * (HEAT_COL_MS / 1000);
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  for (var sec = 1; sec <= Math.floor(spanS); sec++) {
    if (sec % 2 !== 0 && sec !== 1) continue;
    var x = Math.round(mL + pw - (sec / spanS) * pw) + 0.5;
    if (x < mL + 2) continue;
    ctx.strokeStyle = '#1a2531';
    ctx.beginPath(); ctx.moveTo(x, mT); ctx.lineTo(x, mT + ph); ctx.stroke();
    ctx.fillStyle = '#5b6b80';
    ctx.fillText('-' + sec + 's', x, mT + ph + 1);
  }
  ctx.strokeStyle = '#334155';
  ctx.strokeRect(mL + 0.5, mT + 0.5, pw - 1, ph - 1);
  ctx.textAlign = 'right';
  ctx.fillStyle = '#94a3b8';
  ctx.fillText('now', mL + pw, mT + ph + 1);

  // marker for the newest sample value
  if (latest && this.hi > this.lo) {
    var lv = q.get(latest);
    if (isFinite(lv)) {
      var yl = clamp(mT + (1 - (lv - this.lo) / (this.hi - this.lo)) * ph, mT + 1, mT + ph - 1);
      ctx.fillStyle = q.color;
      ctx.beginPath();
      ctx.moveTo(mL + pw, yl); ctx.lineTo(mL + pw - 6, yl - 3); ctx.lineTo(mL + pw - 6, yl + 3);
      ctx.closePath(); ctx.fill();
    }
  }
  // unit label
  ctx.textAlign = 'left';
  ctx.fillStyle = '#64748b';
  ctx.fillText(q.label + (q.unit ? ' [' + q.unit + ']' : ''), mL + 4, mT + 2);
};

// ----------------------------- scrolling time-series plot -----------------------------

function TimeSeriesPlot(canvas, series, opts) {
  this.canvas = canvas;
  this.ctx = canvas.getContext('2d');
  this.series = series;
  this.opts = Object.assign({ span: PLOT_SPAN_S, unit: '', title: '' }, opts || {});
  this.yMin = null;
  this.yMax = null;
  this.W = 0; this.H = 0; this.dpr = 1;
}

TimeSeriesPlot.prototype.sync = function () {
  var c = this.canvas, w = c.clientWidth, h = c.clientHeight;
  if (w <= 0 || h <= 0) return false;
  var dpr = Math.min(2, window.devicePixelRatio || 1);
  var bw = Math.round(w * dpr), bh = Math.round(h * dpr);
  if (c.width !== bw || c.height !== bh) { c.width = bw; c.height = bh; }
  this.W = w; this.H = h; this.dpr = dpr;
  return true;
};

TimeSeriesPlot.prototype.draw = function (ring, tNow, lock) {
  if (!this.sync()) return;
  var ctx = this.ctx, W = this.W, H = this.H;
  ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
  var mL = 48, mR = 8, mT = 15, mB = 13;
  var pw = Math.max(10, W - mL - mR), ph = Math.max(10, H - mT - mB);
  var span = this.opts.span;
  var t1 = tNow, t0 = t1 - span;
  var px = pw / span;
  var win = ring.window(t0, t1), i0 = win[0], i1 = win[1];

  // ---- y range: locked by the user, or autoscaled (expand fast, relax slowly)
  var lo = Infinity, hi = -Infinity, i, si, s;
  for (i = i0; i < i1; i++) {
    s = ring.at(i);
    for (si = 0; si < this.series.length; si++) {
      var v = this.series[si].get(s);
      if (isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; }
    }
  }
  if (!isFinite(lo)) { lo = -1; hi = 1; }
  if (lock && isFinite(lock[0]) && isFinite(lock[1]) && lock[1] > lock[0]) {
    this.yMin = lock[0]; this.yMax = lock[1];
  } else {
    var pad = (hi - lo) * 0.12;
    if (!(pad > 0)) pad = Math.max(0.5, Math.abs(hi) * 0.1);
    var tlo = lo - pad, thi = hi + pad;
    if (this.yMin === null) { this.yMin = tlo; this.yMax = thi; }
    else {
      this.yMin = (tlo < this.yMin) ? tlo : this.yMin + (tlo - this.yMin) * 0.10;
      this.yMax = (thi > this.yMax) ? thi : this.yMax + (thi - this.yMax) * 0.10;
    }
  }
  if (!(this.yMax > this.yMin)) { this.yMin -= 0.5; this.yMax += 0.5; }
  var yLo = this.yMin, yHi = this.yMax;
  var yv = function (t) { return yHi - t * (yHi - yLo); };
  var yPix = function (v) { return mT + (1 - (v - yLo) / (yHi - yLo)) * ph; };

  ctx.fillStyle = '#0b1119';
  ctx.fillRect(0, 0, W, H);
  ctx.fillStyle = '#0e151e';
  ctx.fillRect(mL, mT, pw, ph);

  // ---- grid + y labels
  ctx.font = FONT_2D;
  ctx.textBaseline = 'middle';
  ctx.textAlign = 'right';
  var ticks = 5;
  for (var k = 0; k < ticks; k++) {
    var t = k / (ticks - 1);
    var y = Math.round(mT + t * ph) + 0.5;
    ctx.strokeStyle = (k === 0 || k === ticks - 1) ? '#2b3a4d' : '#1a2531';
    ctx.beginPath(); ctx.moveTo(mL, y); ctx.lineTo(mL + pw, y); ctx.stroke();
    ctx.fillStyle = '#7f8ea3';
    ctx.fillText(fmt(yv(t)), mL - 4, y);
  }
  // ---- x grid, one line per second (right edge = now)
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  for (var sec = 1; sec <= span; sec++) {
    var x = Math.round(mL + pw - sec * px) + 0.5;
    if (x < mL + 2) continue;
    ctx.strokeStyle = '#1a2531';
    ctx.beginPath(); ctx.moveTo(x, mT); ctx.lineTo(x, mT + ph); ctx.stroke();
    ctx.fillStyle = '#5b6b80';
    ctx.fillText('-' + sec + 's', x, mT + ph + 1);
  }

  // ---- traces (clipped, broken across gaps so missing samples are visible)
  ctx.save();
  ctx.beginPath(); ctx.rect(mL, mT, pw, ph); ctx.clip();
  for (si = 0; si < this.series.length; si++) {
    var ser = this.series[si];
    ctx.strokeStyle = ser.color;
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    var started = false, prevT = 0;
    for (i = i0; i < i1; i++) {
      s = ring.at(i);
      var val = ser.get(s);
      if (!isFinite(val)) { started = false; continue; }
      var xp = mL + pw - (t1 - s.t) * px;
      var yp = yPix(val);
      if (!started || (s.t - prevT) > GAP_S || s.t < prevT) { ctx.moveTo(xp, yp); started = true; }
      else ctx.lineTo(xp, yp);
      prevT = s.t;
    }
    ctx.stroke();
  }
  ctx.restore();

  // ---- moving time cursor at the newest sample
  var last = ring.latest(0);
  if (last) {
    var xc = clamp(mL + pw - (t1 - last.t) * px, mL, mL + pw);
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(xc + 0.5, mT); ctx.lineTo(xc + 0.5, mT + ph); ctx.stroke();
    ctx.fillStyle = '#e2e8f0';
    ctx.beginPath();
    ctx.moveTo(xc, mT + ph); ctx.lineTo(xc - 3, mT + ph + 4); ctx.lineTo(xc + 3, mT + ph + 4);
    ctx.closePath(); ctx.fill();
  }

  ctx.strokeStyle = '#334155';
  ctx.lineWidth = 1;
  ctx.strokeRect(mL + 0.5, mT + 0.5, pw - 1, ph - 1);

  // ---- legend (top-left) with live values (top-right)
  ctx.textBaseline = 'middle';
  var lx = mL + 4;
  ctx.textAlign = 'left';
  for (si = 0; si < this.series.length; si++) {
    var se2 = this.series[si];
    ctx.fillStyle = se2.color;
    ctx.fillRect(lx, mT + 3, 7, 7);
    lx += 10;
    ctx.fillStyle = '#a8bacd';
    ctx.fillText(se2.name, lx, mT + 7);
    lx += ctx.measureText(se2.name).width + 9;
  }
  ctx.textAlign = 'right';
  var rx = mL + pw - 4;
  if (last) {
    for (si = this.series.length - 1; si >= 0; si--) {
      var se3 = this.series[si];
      var txt = fmt(se3.get(last));
      ctx.fillStyle = se3.color;
      ctx.fillText(txt, rx, mT + ph - 6 - (this.series.length - 1 - si) * 11);
      rx -= ctx.measureText(txt).width + 6;
      ctx.fillStyle = '#7f8ea3';
      ctx.fillText(se3.name === 'roll' ? 'R' : se3.name === 'pitch' ? 'P' : se3.name === 'yaw' ? 'Y' : '', rx, mT + ph - 6 - (this.series.length - 1 - si) * 11);
    }
  }
  // unit / title
  ctx.textAlign = 'left';
  ctx.fillStyle = '#64748b';
  ctx.fillText(this.opts.title + (this.opts.unit ? ' [' + this.opts.unit + ']' : ''), mL + 4, mT + ph - 6);
};

// ----------------------------- statistics accumulation -----------------------------

// Least-squares fit v(t) = a + b*t; b is the drift slope in units per second.
// t is relative to the first sample of the window, which keeps the normal
// equations well conditioned even when the sample clock is epoch seconds.
function axisStats(values, times, n) {
  if (n < 2) return null;
  var sum = 0, min = Infinity, max = -Infinity, i, v;
  for (i = 0; i < n; i++) { v = values[i]; sum += v; if (v < min) min = v; if (v > max) max = v; }
  var mean = sum / n, ss = 0;
  for (i = 0; i < n; i++) { var d = values[i] - mean; ss += d * d; }
  var std = Math.sqrt(ss / n);
  var t0 = times[0], St = 0, Stt = 0, Stv = 0;
  for (i = 0; i < n; i++) {
    var dt = times[i] - t0;
    St += dt; Stt += dt * dt; Stv += dt * values[i];
  }
  var den = n * Stt - St * St;
  var slope = Math.abs(den) > 1e-12 ? (n * Stv - St * sum) / den : 0;
  return { n: n, mean: mean, std: std, min: min, max: max, p2p: max - min, slope: slope };
}

var statsRefs = null;

function td(text, cls) {
  var e = document.createElement('td');
  e.textContent = text;
  if (cls) e.className = cls;
  return e;
}
function th(text) {
  var e = document.createElement('th');
  e.textContent = text;
  return e;
}

function buildAxisStats(host, title, prefix) {
  var h = document.createElement('h4');
  h.textContent = title;
  host.appendChild(h);
  var cols = ['mean', 'std', 'p2p', 'drift/s', 'min', 'max'];
  var table = document.createElement('table');
  table.className = 'stats';
  var thead = document.createElement('thead'), tr = document.createElement('tr');
  tr.appendChild(th('axis'));
  for (var c = 0; c < cols.length; c++) tr.appendChild(th(cols[c]));
  thead.appendChild(tr); table.appendChild(thead);
  var tbody = document.createElement('tbody');
  var refs = {};
  var axes = ['x', 'y', 'z'];
  for (var a = 0; a < axes.length; a++) {
    var row = document.createElement('tr');
    row.appendChild(td(axes[a].toUpperCase(), 'k'));
    refs[axes[a]] = {};
    for (c = 0; c < cols.length; c++) {
      var cell = td('--', 'v');
      row.appendChild(cell);
      refs[axes[a]][cols[c]] = cell;
    }
    tbody.appendChild(row);
  }
  table.appendChild(tbody);
  host.appendChild(table);
  return refs;
}

function buildStatsPanel() {
  var host = $('statsRoot');
  if (!host) return;
  host.textContent = '';
  statsRefs = {
    gyro: buildAxisStats(host, 'Gyro (deg/s) - trunk frame', 'gyro'),
    grav: buildAxisStats(host, 'Gravity (unit vector) - trunk frame', 'grav'),
    link: {}
  };
  var h4 = document.createElement('h4');
  h4.textContent = 'Link / health';
  host.appendChild(h4);
  var kv = document.createElement('div');
  kv.className = 'kv';
  var items = [
    ['rate', 'stHz'], ['latency med', 'stLatMed'], ['latency p95', 'stLatP95'],
    ['latency min', 'stLatMin'], ['latency max', 'stLatMax'], ['latency n', 'stLatN'],
    ['polls', 'stPolls'], ['ok', 'stOk'], ['timeouts', 'stTimeouts'],
    ['bad frames', 'stBad'], ['extra bytes', 'stExtra'], ['samples', 'stSamples'],
    ['missing', 'stMissing'], ['counter jumps', 'stJumps'], ['duplicate cnt', 'stDups'],
    ['stale run', 'stStaleRun'], ['stale max', 'stStaleMax'], ['stale total', 'stStaleTotal'],
    ['errors', 'stErrs'], ['drop rate', 'stDrop']
  ];
  for (var i = 0; i < items.length; i++) {
    var k = document.createElement('div'); k.className = 'k'; k.textContent = items[i][0];
    var v = document.createElement('div'); v.className = 'v'; v.id = items[i][1]; v.textContent = '--';
    kv.appendChild(k); kv.appendChild(v);
    statsRefs.link[items[i][1]] = v;
  }
  host.appendChild(kv);
  var h5 = document.createElement('h4');
  h5.textContent = 'Raw flags (latest sample)';
  host.appendChild(h5);
  var fl = document.createElement('div');
  fl.className = 'flags'; fl.id = 'flagBox'; fl.textContent = '--';
  host.appendChild(fl);
}

function setRef(ref, value, bad, warn) {
  if (!ref) return;
  ref.textContent = value;
  ref.className = 'v' + (bad ? ' bad' : (warn ? ' warn' : ''));
}

function updateStats() {
  if (!statsRefs) return;
  var ring = state.ring, n = ring.n;
  var st = dataState();
  var hasData = n >= 2 && st === 'ok';

  var nd = $('statsNoData');
  if (nd) {
    nd.classList.toggle('hidden', hasData);
    nd.textContent = stateLabel(st);
  }
  if (!hasData) {
    for (var key in statsRefs.gyro) {
      for (var c in statsRefs.gyro[key]) setRef(statsRefs.gyro[key][c], '--', false, false);
      for (c in statsRefs.grav[key]) setRef(statsRefs.grav[key][c], '--', false, false);
    }
  }

  if (hasData) {
    var times = new Float64Array(n);
    var g = [new Float64Array(n), new Float64Array(n), new Float64Array(n)];
    var gr = [new Float64Array(n), new Float64Array(n), new Float64Array(n)];
    var t0 = ring.at(0).t;
    for (var i = 0; i < n; i++) {
      var s = ring.at(i);
      times[i] = s.t - t0;
      for (var a = 0; a < 3; a++) { g[a][i] = s.gyroTrunk[a]; gr[a][i] = s.gravity[a]; }
    }
    var axes = ['x', 'y', 'z'];
    for (a = 0; a < 3; a++) {
      var sg = axisStats(g[a], times, n);
      var sr = axisStats(gr[a], times, n);
      var gyroBad = sg && Math.abs(sg.slope) > 0.5;
      var gravBad = sr && Math.abs(sr.slope) > 0.01;
      setRef(statsRefs.gyro[axes[a]].mean, sg ? fmt(sg.mean, 3) : '--');
      setRef(statsRefs.gyro[axes[a]].std, sg ? fmt(sg.std, 3) : '--', false, sg && sg.std > 5);
      setRef(statsRefs.gyro[axes[a]].p2p, sg ? fmt(sg.p2p, 3) : '--');
      setRef(statsRefs.gyro[axes[a]].drift, sg ? fmt(sg.slope, 4) : '--', gyroBad);
      setRef(statsRefs.gyro[axes[a]].min, sg ? fmt(sg.min, 3) : '--');
      setRef(statsRefs.gyro[axes[a]].max, sg ? fmt(sg.max, 3) : '--');
      setRef(statsRefs.grav[axes[a]].mean, sr ? fmt(sr.mean, 4) : '--');
      setRef(statsRefs.grav[axes[a]].std, sr ? fmt(sr.std, 4) : '--', false, sr && sr.std > 0.02);
      setRef(statsRefs.grav[axes[a]].p2p, sr ? fmt(sr.p2p, 4) : '--');
      setRef(statsRefs.grav[axes[a]].drift, sr ? fmt(sr.slope, 5) : '--', gravBad);
      setRef(statsRefs.grav[axes[a]].min, sr ? fmt(sr.min, 4) : '--');
      setRef(statsRefs.grav[axes[a]].max, sr ? fmt(sr.max, 4) : '--');
    }
  }

  var L = statsRefs.link;
  var ls = state.linkStats;
  var hzLocal = localHz();
  setRef(L.stHz, isFinite(state.achievedHz)
    ? fmt(state.achievedHz, 2) + ' Hz' + (isFinite(hzLocal) ? ' (rx ' + fmt(hzLocal, 1) + ')' : '')
    : (isFinite(hzLocal) ? fmt(hzLocal, 2) + ' Hz' : '--'));
  // latency lives under stats.latency: {n, min_ms, median_ms, p95_ms, max_ms}
  var lat = (ls && ls.latency && typeof ls.latency === 'object') ? ls.latency : null;
  if (lat && lat.n) {
    setRef(L.stLatMed, isFinite(lat.median_ms) ? fmt(lat.median_ms, 2) + ' ms' : '--');
    setRef(L.stLatP95, isFinite(lat.p95_ms) ? fmt(lat.p95_ms, 2) + ' ms' : '--');
    setRef(L.stLatMin, isFinite(lat.min_ms) ? fmt(lat.min_ms, 2) + ' ms' : '--');
    setRef(L.stLatMax, isFinite(lat.max_ms) ? fmt(lat.max_ms, 2) + ' ms' : '--');
    setRef(L.stLatN, String(lat.n));
  } else {
    setRef(L.stLatMed, '--'); setRef(L.stLatP95, '--'); setRef(L.stLatMin, '--');
    setRef(L.stLatMax, '--'); setRef(L.stLatN, '--');
  }
  // poll counters come from the "stats" message (state.link only carries the
  // live link description: port / baud / mode / rate / protocol)
  var lp = state.linkStats;
  setRef(L.stPolls, lp && isFinite(lp.polls) ? String(lp.polls) : '--');
  setRef(L.stOk, lp && isFinite(lp.ok) ? String(lp.ok) : '--');
  setRef(L.stTimeouts, lp && isFinite(lp.timeouts) ? String(lp.timeouts) : '--', lp && lp.timeouts > 0);
  setRef(L.stBad, lp && isFinite(lp.bad_frames) ? String(lp.bad_frames) : '--', lp && lp.bad_frames > 0);
  setRef(L.stExtra, lp && isFinite(lp.extra_bytes) ? String(lp.extra_bytes) : '--', lp && lp.extra_bytes > 0, lp && lp.extra_bytes > 0);
  var c = state.counters;
  setRef(L.stSamples, ring.n ? String(ring.n) + ' buf / ' + state.lastSample.samples : '--');
  setRef(L.stMissing, String(c.missing), c.missing > 0, c.missing > 0);
  setRef(L.stJumps, String(c.jumps), c.jumps > 0, c.jumps > 0);
  setRef(L.stDups, String(c.dups));
  setRef(L.stStaleRun, String(state.stale.run), state.stale.run > 3, state.stale.run > 0);
  setRef(L.stStaleMax, String(state.stale.max), state.stale.max > 10, state.stale.max > 3);
  setRef(L.stStaleTotal, String(state.stale.total));
  setRef(L.stErrs, String(c.errs), c.errs > 0, c.errs > 0);
  var polls = (lp && isFinite(lp.polls)) ? lp.polls : 0;
  var dropped = lp ? (num(lp.timeouts, 0) + num(lp.bad_frames, 0)) : 0;
  setRef(L.stDrop, polls > 0 ? fmt(dropped / polls * 100, 2) + ' %' : '--', polls > 0 && dropped / polls > 0.01);

  // decoded flags
  var fb = $('flagBox');
  if (fb) {
    fb.textContent = '';
    var hex = document.createElement('span');
    hex.className = 'hex';
    hex.textContent = '0x' + state.flags.toString(16).padStart(2, '0') + ' ';
    fb.appendChild(hex);
    if (!state.flagNames.length) {
      var none = document.createElement('span');
      none.className = 'none';
      none.textContent = 'none';
      fb.appendChild(none);
    } else {
      for (var fi = 0; fi < state.flagNames.length; fi++) {
        var nm = String(state.flagNames[fi]).replace(/[^a-z0-9_]/gi, '');
        var sp = document.createElement('span');
        sp.className = 'f-' + nm;
        sp.textContent = state.flagNames[fi] + (fi < state.flagNames.length - 1 ? ', ' : '');
        fb.appendChild(sp);
      }
    }
    var seen = Object.keys(state.flagsSeen);
    if (seen.length) {
      var s2 = document.createElement('div');
      s2.className = 'none';
      s2.textContent = 'seen since connect: ' + seen.join(', ');
      fb.appendChild(s2);
    }
  }
}

function localHz() {
  var a = state.recvTimes;
  if (a.length < 2) return NaN;
  var dt = (a[a.length - 1] - a[0]) / 1000;
  return dt > 0 ? (a.length - 1) / dt : NaN;
}

// ----------------------------- data / link state helpers -----------------------------

function dataState() {
  var now = nowMs();
  if (state.paused) return 'paused';
  if (!state.connected) return 'disconnected';
  if (!state.lastSampleMs) return 'waiting';
  return (now - state.lastSampleMs) > 1200 ? 'stalled' : 'ok';
}
function stateLabel(st) {
  if (st === 'paused') return 'PAUSED';
  if (st === 'disconnected') return 'NO DATA - not connected';
  if (st === 'waiting') return 'NO DATA - waiting for samples';
  if (st === 'stalled') return 'NO DATA - no samples for ' + fmt((nowMs() - state.lastSampleMs) / 1000, 1) + ' s';
  return '';
}

function resetSession() {
  state.ring.clear();
  state.pending = [];
  state.counters = { prev: null, missing: 0, jumps: 0, dups: 0, errs: 0 };
  state.stale = { run: 0, max: 0, total: 0 };
  state.flags = 0;
  state.flagNames = [];
  state.flagsSeen = {};
  state.linkStats = null;
  state.lastSample = null;
  state.recvTimes = [];
  state.tNow = 0;
  if (heat) heat.clear(heatQtyKey());
  if (board) { board.traceN = 0; board.traceHead = 0; board.traceSeq = -1; }
}

// ----------------------------- toast -----------------------------

var toastTimer = 0;
function toast(msg) {
  var t = $('toast');
  if (!t) return;
  t.textContent = String(msg);
  t.classList.remove('hidden');
  if (toastTimer) clearTimeout(toastTimer);
  toastTimer = setTimeout(function () { t.classList.add('hidden'); }, 6000);
}

// ----------------------------- CSV recording -----------------------------

var CSV_HEADER = [
  't', 'recv_s', 'roll_deg', 'pitch_deg', 'yaw_deg',
  'quat_w', 'quat_x', 'quat_y', 'quat_z',
  'quat_raw_w', 'quat_raw_x', 'quat_raw_y', 'quat_raw_z',
  'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps',
  'gyro_trunk_x_dps', 'gyro_trunk_y_dps', 'gyro_trunk_z_dps',
  'gravity_x', 'gravity_y', 'gravity_z',
  'accel_x_mg', 'accel_y_mg', 'accel_z_mg',
  'counter', 'flags', 'flag_names', 'quat_valid', 'ready',
  'stale_run', 'stale_total', 'samples', 'latency_ms', 'error'
].join(',');

var rec = { on: false, rows: [], cap: 400000 };

function csvRow(s) {
  var q = s.quat, qr = s.quatRaw;
  return [
    s.t.toFixed(6), s.recv.toFixed(6),
    s.euler[0].toFixed(5), s.euler[1].toFixed(5), s.euler[2].toFixed(5),
    q[0].toFixed(6), q[1].toFixed(6), q[2].toFixed(6), q[3].toFixed(6),
    qr[0].toFixed(6), qr[1].toFixed(6), qr[2].toFixed(6), qr[3].toFixed(6),
    s.gyro[0].toFixed(5), s.gyro[1].toFixed(5), s.gyro[2].toFixed(5),
    s.gyroTrunk[0].toFixed(5), s.gyroTrunk[1].toFixed(5), s.gyroTrunk[2].toFixed(5),
    s.gravity[0].toFixed(6), s.gravity[1].toFixed(6), s.gravity[2].toFixed(6),
    s.accel[0].toFixed(4), s.accel[1].toFixed(4), s.accel[2].toFixed(4),
    s.counter, s.flags, '"' + s.flagNames.join('|') + '"',
    s.quatValid ? 1 : 0, s.ready ? 1 : 0, s.staleRun, s.staleTotal, s.samples,
    isFinite(s.latency) ? s.latency.toFixed(3) : '', s.error
  ].join(',');
}

function toggleRecord() {
  var btn = $('btnRecord'), info = $('recInfo');
  if (!rec.on) {
    rec.on = true; rec.rows = [CSV_HEADER];
    if (btn) { btn.classList.add('on'); btn.textContent = 'Stop CSV'; }
    if (info) info.textContent = 'recording...';
  } else {
    rec.on = false;
    if (btn) { btn.classList.remove('on'); btn.textContent = 'Record CSV'; }
    var text = rec.rows.join('\n') + '\n';
    var count = rec.rows.length - 1;
    rec.rows = [];
    try {
      var blob = new Blob([text], { type: 'text/csv' });
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url;
      a.download = 'imu_record_' + localStamp() + '.csv';
      document.body.appendChild(a);
      a.click();
      a.remove();
      setTimeout(function () { URL.revokeObjectURL(url); }, 5000);
      if (info) info.textContent = count + ' samples saved';
    } catch (e) {
      if (info) info.textContent = 'download failed';
      toast('CSV download failed: ' + e.message);
    }
  }
}

// ----------------------------- message handling -----------------------------

function normalizeSample(s) {
  if (!s || typeof s !== 'object') return null;
  var t = num(s.t, NaN);
  if (!(t > 0)) t = nowMs() / 1000;
  var lat = num(s.latency_ms, NaN);
  return {
    t: t,
    recv: nowMs() / 1000,
    gyro: vec3(s.gyro_dps),
    gyroTrunk: vec3(s.gyro_trunk_dps),
    quat: vec4(s.quat, [1, 0, 0, 0]),
    quatRaw: vec4(s.quat_raw, [1, 0, 0, 0]),
    gravity: vec3(s.gravity),
    accel: vec3(s.accel_mg),
    euler: vec3(s.euler),
    counter: int(s.counter, 0) & 0xFF,
    flags: int(s.flags, 0),
    flagNames: Array.isArray(s.flag_names) ? s.flag_names.filter(function (x) { return typeof x === 'string'; }) : [],
    quatValid: !!s.quat_valid,
    ready: !!s.ready,
    staleRun: int(s.stale_run, 0),
    staleTotal: int(s.stale_total, 0),
    samples: int(s.samples, 0),
    latency: lat,
    error: int(s.error, 0)
  };
}

function onSample(s) {
  var c = state.counters;
  if (c.prev !== null) {
    var delta = (s.counter - c.prev) & 0xFF;
    if (delta === 0) c.dups++;
    else if (delta > 1) { c.missing += delta - 1; c.jumps++; }
  }
  c.prev = s.counter;
  if (s.error) c.errs++;

  state.stale.run = s.staleRun;
  if (s.staleRun > state.stale.max) state.stale.max = s.staleRun;
  if (s.staleTotal > state.stale.total) state.stale.total = s.staleTotal;
  state.flags = s.flags;
  state.flagNames = s.flagNames;
  for (var i = 0; i < s.flagNames.length; i++) state.flagsSeen[s.flagNames[i]] = true;

  state.lastSample = s;
  state.lastSampleMs = nowMs();
  state.recvTimes.push(state.lastSampleMs);
  while (state.recvTimes.length > 400) state.recvTimes.shift();
  state.tNow = s.t;
  state.ring.push(s);
  if (state.pending.length < 1024) state.pending.push(s);

  if (rec.on) {
    rec.rows.push(csvRow(s));
    if (rec.rows.length >= rec.cap) { toggleRecord(); toast('CSV recording stopped at ' + (rec.cap - 1) + ' samples'); }
  }
}

function applyConfigToForm(cfg) {
  if (!cfg || typeof cfg !== 'object') return;
  if (typeof cfg.proto_lock === 'string') state.protoLock = cfg.proto_lock;
  setField('selSimMode', typeof cfg.sim_mode_name === 'string' ? cfg.sim_mode_name
    : (isFinite(cfg.sim_mode) ? SIM_NAMES[int(cfg.sim_mode, 0)] : null));
  setField('rngAmp', isFinite(cfg.sim_amp) ? String(int(cfg.sim_amp, 1000)) : null);
  setField('rngFreq', isFinite(cfg.sim_freq) ? String(int(cfg.sim_freq, 1000)) : null);
  if (cfg.gyro_bias && cfg.gyro_bias.length >= 3) {
    setField('inpBiasX', String(int(cfg.gyro_bias[0], 0)));
    setField('inpBiasY', String(int(cfg.gyro_bias[1], 0)));
    setField('inpBiasZ', String(int(cfg.gyro_bias[2], 0)));
  }
  if (isFinite(cfg.report_frame)) setField('selReportFrame', String(int(cfg.report_frame, 0)));
  if (isFinite(cfg.dbg_level)) setField('selDbgLevel', String(int(cfg.dbg_level, 0)));
  updateRangeLabels();

  var cs = $('cfgStatus');
  if (cs) {
    var bits = [];
    bits.push('status ' + int(cfg.status, 0));
    if (cfg.cfg_dirty) bits.push('DIRTY');
    if (cfg.cfg_from_flash) bits.push('from-flash');
    if (cfg.sim_active) bits.push('sim-active');
    bits.push('lock:' + state.protoLock);
    cs.textContent = 'cfg: ' + bits.join(' ');
  }
}

function applyStatus(msg) {
  var was = state.connected;
  state.connected = !!msg.connected;
  if (msg.device && typeof msg.device === 'object') state.device = msg.device;
  if (msg.link && typeof msg.link === 'object') state.link = msg.link;
  if (msg.config) applyConfigToForm(msg.config);
  var btn = $('btnPause');
  if (btn) btn.disabled = !state.connected;
  if (state.connected && !was) resetSession();
  updateLinkUi();
}

function handleMessage(msg) {
  if (!msg || typeof msg !== 'object') return;
  switch (msg.type) {
    case 'hello':
      if (msg.device) state.device = msg.device;
      if (msg.link) state.link = msg.link;
      if (msg.config) applyConfigToForm(msg.config);
      if (typeof msg.protocol === 'string') setField('selProtocol', msg.protocol);
      updateLinkUi();
      break;
    case 'sample':
      var s = normalizeSample(msg.s);
      if (s) onSample(s);
      break;
    case 'stats':
      state.linkStats = msg.stats && typeof msg.stats === 'object' ? msg.stats : null;
      state.achievedHz = num(msg.achieved_hz, NaN);
      if (msg.config) applyConfigToForm(msg.config);
      break;
    case 'status':
      applyStatus(msg);
      break;
    case 'error':
      toast('server error: ' + String(msg.message || 'unknown'));
      break;
    default:
      break;   // unknown message types are ignored, never fatal
  }
}

// ----------------------------- websocket -----------------------------

function wsUrl() {
  return (location.protocol === 'https:' ? 'wss' : 'ws') + '://' + location.host + '/ws';
}

function connectWS() {
  if (state.ws && (state.ws.readyState === 0 || state.ws.readyState === 1)) return;
  var ws;
  try { ws = new WebSocket(wsUrl()); } catch (e) { scheduleReconnect(); return; }
  state.ws = ws;

  ws.onopen = function () {
    state.wsOpen = true;
    setConnIndicator();
    resendSettings();
  };
  ws.onmessage = function (ev) {
    // never let a malformed or partial message throw: parse + handle in try/catch
    try {
      handleMessage(JSON.parse(ev.data));
    } catch (e) {
      /* drop bad message */
    }
  };
  ws.onerror = function () { /* onclose follows */ };
  ws.onclose = function () {
    state.wsOpen = false;
    state.ws = null;
    state.connected = false;
    var btn = $('btnPause');
    if (btn) btn.disabled = true;
    setConnIndicator();
    updateLinkUi();
    scheduleReconnect();
  };
}

function scheduleReconnect() {
  if (state.reconnectTimer) return;
  state.reconnectTimer = setTimeout(function () {
    state.reconnectTimer = 0;
    connectWS();
  }, 1000);   // 1 s backoff
}

function send(obj) {
  var ws = state.ws;
  if (!ws || ws.readyState !== 1) return false;
  try { ws.send(JSON.stringify(obj)); return true; } catch (e) { return false; }
}

// Re-send the current settings after every (re)connect.
function resendSettings() {
  if (state.intent.connected) sendConnect();
  sendConfig();
  send({ op: 'pause', paused: state.paused });
}

function collectLink() {
  return {
    port: $('selPort') ? $('selPort').value : '',
    baud: int($('selBaud') ? $('selBaud').value : 1000000, 1000000),
    protocol: $('selProtocol') ? $('selProtocol').value : 'dxl',
    mode: $('selMode') ? $('selMode').value : 'sync_read',
    read_len: int($('selReadLen') ? $('selReadLen').value : 20, 20),
    rate: int($('rngRate') ? $('rngRate').value : 100, 100)
  };
}
function sendConnect() {
  var msg = collectLink();
  msg.op = 'connect';
  return send(msg);
}
function collectConfig() {
  return {
    op: 'config',
    sim_mode: $('selSimMode') ? $('selSimMode').value : 'static',
    sim_amp: int($('rngAmp') ? $('rngAmp').value : 1000, 1000),
    sim_freq: int($('rngFreq') ? $('rngFreq').value : 1000, 1000),
    gyro_bias: [
      int($('inpBiasX') ? $('inpBiasX').value : 0, 0),
      int($('inpBiasY') ? $('inpBiasY').value : 0, 0),
      int($('inpBiasZ') ? $('inpBiasZ').value : 0, 0)
    ],
    report_frame: int($('selReportFrame') ? $('selReportFrame').value : 0, 0),
    proto_lock: state.protoLock || 'none',
    dbg_level: int($('selDbgLevel') ? $('selDbgLevel').value : 2, 2)
  };
}
function sendConfig() { return send(collectConfig()); }
function sendCommand(cmd) { return send({ op: 'command', cmd: cmd }); }

// ----------------------------- UI wiring -----------------------------

function setConnIndicator() {
  var dot = $('connDot'), txt = $('connText');
  if (!dot || !txt) return;
  if (state.connected) {
    dot.className = 'dot' + (state.paused ? ' warn' : ' on');
    txt.textContent = state.paused ? 'paused' : 'connected';
  } else if (state.wsOpen && state.intent.connected) {
    dot.className = 'dot warn';
    txt.textContent = 'connecting';
  } else if (state.wsOpen) {
    dot.className = 'dot off';
    txt.textContent = 'link idle';
  } else {
    dot.className = 'dot off';
    txt.textContent = 'no server';
  }
}

function updateLinkUi() {
  var btn = $('btnConnect'), dev = $('devText'), pause = $('btnPause');
  var lk = state.link;
  if (btn) {
    btn.textContent = state.intent.connected ? 'Disconnect' : 'Connect';
    btn.classList.toggle('on', state.intent.connected);
  }
  if (pause) {
    pause.textContent = state.paused ? 'Resume' : 'Pause';
    pause.disabled = !state.connected;
  }
  if (dev) {
    var d = state.device;
    var parts = [];
    if (d && isFinite(d.model)) parts.push('model ' + d.model + ' (0x' + (int(d.model, 0) & 0xFFFF).toString(16) + ')');
    if (d && d.firmware !== null && d.firmware !== undefined && isFinite(d.firmware)) parts.push('fw ' + d.firmware);
    if (lk) {
      if (lk.port) parts.push(String(lk.port));
      if (isFinite(lk.baud)) parts.push(String(lk.baud) + ' baud');
      if (lk.protocol) parts.push(String(lk.protocol));
      if (lk.mode) parts.push(String(lk.mode));
      if (isFinite(lk.rate)) parts.push(String(lk.rate) + ' Hz');
    }
    dev.textContent = parts.length ? parts.join(' | ') : 'no device';
  }
  setConnIndicator();
}

function setField(id, value) {
  if (value === null || value === undefined) return;
  if (dirtyFields[id]) return;            // do not fight a pending manual edit
  var el = $(id);
  if (!el) return;
  var v = String(value);
  if (el.value !== v) el.value = v;
}

var dirtyFields = {};

function markDirty(id) { dirtyFields[id] = true; }

function updateRangeLabels() {
  var a = $('rngAmp'), f = $('rngFreq'), r = $('rngRate');
  if (a && $('lblAmp')) $('lblAmp').textContent = a.value;
  if (f && $('lblFreq')) $('lblFreq').textContent = f.value;
  if (r && $('lblRate')) $('lblRate').textContent = r.value + ' Hz';
}

function fetchPorts() {
  var sel = $('selPort');
  if (!sel) return;
  fetch('/api/ports', { cache: 'no-store' })
    .then(function (r) { return r.json(); })
    .then(function (data) {
      var ports = (data && Array.isArray(data.ports)) ? data.ports : [];
      var keep = sel.value;
      sel.textContent = '';
      if (!ports.length) {
        var o = document.createElement('option');
        o.value = ''; o.textContent = '(no ports found)';
        sel.appendChild(o);
        return;
      }
      for (var i = 0; i < ports.length; i++) {
        var p = ports[i] || {};
        var dev = typeof p.device === 'string' ? p.device : String(p.device || '');
        var o2 = document.createElement('option');
        o2.value = dev;
        o2.textContent = dev + (p.description ? ' - ' + p.description : '');
        sel.appendChild(o2);
      }
      var found = false;
      for (i = 0; i < sel.options.length; i++) if (sel.options[i].value === keep) found = true;
      if (found) sel.value = keep;
      else if (sel.options.length) sel.selectedIndex = 0;   // default to the first port
    })
    .catch(function () {
      sel.textContent = '';
      var o = document.createElement('option');
      o.value = ''; o.textContent = '(ports unavailable)';
      sel.appendChild(o);
    });
}

function heatQtyKey() {
  var el = $('selHeatQty');
  return (el && HEAT_QTY[el.value]) ? el.value : 'gyro_x';
}

function lockRange(minId, maxId) {
  var a = $(minId), b = $(maxId);
  if (!a || !b) return null;
  var lo = parseFloat(a.value), hi = parseFloat(b.value);
  if (!isFinite(lo) || !isFinite(hi) || !(hi > lo)) return null;
  return [lo, hi];
}

function wireUi() {
  var b;

  // port / link controls: re-issue "connect" when they change while connected
  var rein = ['selPort', 'selBaud', 'selProtocol', 'selMode', 'selReadLen'];
  for (var i = 0; i < rein.length; i++) {
    (function (id) {
      var el = $(id);
      if (!el) return;
      el.addEventListener('change', function () {
        markDirty(id);
        if (state.intent.connected) sendConnect();
      });
    })(rein[i]);
  }

  var rr = $('rngRate');
  if (rr) {
    rr.addEventListener('input', function () { markDirty('rngRate'); updateRangeLabels(); });
    rr.addEventListener('change', function () { if (state.intent.connected) sendConnect(); });
  }

  if ($('btnRefreshPorts')) $('btnRefreshPorts').addEventListener('click', fetchPorts);

  if ($('btnConnect')) {
    $('btnConnect').addEventListener('click', function () {
      if (state.intent.connected) {
        state.intent.connected = false;
        send({ op: 'disconnect' });
      } else {
        state.intent.connected = true;
        if (!sendConnect()) toast('server link not open yet - will connect after reconnect');
      }
      updateLinkUi();
    });
  }

  if ($('btnPause')) {
    $('btnPause').addEventListener('click', function () {
      state.paused = !state.paused;
      send({ op: 'pause', paused: state.paused });
      updateLinkUi();
    });
  }

  if ($('btnRecord')) $('btnRecord').addEventListener('click', toggleRecord);

  // board settings
  var cfgIds = ['selSimMode', 'rngAmp', 'rngFreq', 'inpBiasX', 'inpBiasY', 'inpBiasZ', 'selReportFrame', 'selDbgLevel'];
  for (i = 0; i < cfgIds.length; i++) {
    (function (id) {
      var el = $(id);
      if (!el) return;
      el.addEventListener('input', function () { markDirty(id); updateRangeLabels(); });
      el.addEventListener('change', function () { markDirty(id); updateRangeLabels(); });
    })(cfgIds[i]);
  }

  function applyClicked(extraCmd) {
    for (var k in dirtyFields) delete dirtyFields[k];
    sendConfig();
    if (extraCmd) sendCommand(extraCmd);
  }
  if ($('btnApply')) $('btnApply').addEventListener('click', function () { applyClicked(null); });
  if ($('btnSave')) $('btnSave').addEventListener('click', function () {
    applyClicked('save');
    toast('configuration written and save-to-flash requested');
  });
  if ($('btnDefaults')) $('btnDefaults').addEventListener('click', function () { sendCommand('defaults'); });
  if ($('btnReboot')) $('btnReboot').addEventListener('click', function () { sendCommand('reboot'); });
  if ($('btnResetSim')) $('btnResetSim').addEventListener('click', function () { sendCommand('reset_sim'); });

  // y-axis lock
  var lock = $('chkLockY');
  var lockInputs = ['inpAttYMin', 'inpAttYMax', 'inpGyroYMin', 'inpGyroYMax'];
  function syncLock() {
    var on = !!(lock && lock.checked);
    for (var k = 0; k < lockInputs.length; k++) {
      var el = $(lockInputs[k]);
      if (el) el.disabled = !on;
    }
  }
  if (lock) lock.addEventListener('change', syncLock);
  syncLock();

  // heat quantity switch -> clear history (the value range changes)
  var hq = $('selHeatQty');
  if (hq) hq.addEventListener('change', function () { heat.clear(heatQtyKey()); });

  updateRangeLabels();
}

// ----------------------------- panels / render loop -----------------------------

var board = null, attPlot = null, gyroPlot = null, heat = null;

function visible(el) {
  if (!el) return false;
  return el.clientWidth > 0 && el.clientHeight > 0;
}

function setNoData(el, show, text) {
  if (!el) return;
  el.classList.toggle('hidden', !show);
  if (text) el.textContent = text;
}

var lastHeatFlush = 0, lastStatsAt = 0;

function tick() {
  requestAnimationFrame(tick);
  if (document.hidden) return;                   // skip drawing when the tab is hidden
  var now = nowMs();

  // ---- heat-map columns advance on wall time, not per message, so bus stalls
  // show up as empty (dark) columns instead of a compressed time axis
  if (lastHeatFlush === 0) lastHeatFlush = now - HEAT_COL_MS;
  var guard = 0;
  while (now - lastHeatFlush >= HEAT_COL_MS && guard < 4) {
    var slice = state.pending;
    state.pending = [];
    heat.push(slice, heatQtyKey());
    lastHeatFlush += HEAT_COL_MS;
    guard++;
  }
  if (guard >= 4) lastHeatFlush = now;

  var st = dataState();
  var label = stateLabel(st);

  // ---- 3D
  if (visible(board.canvas)) {
    board.draw(state.lastSample, state.ring.total);
    var l = state.lastSample;
    if ($('ovRoll')) {
      if (l) {
        $('ovRoll').textContent = fmt(l.euler[0], 2);
        $('ovPitch').textContent = fmt(l.euler[1], 2);
        $('ovYaw').textContent = fmt(l.euler[2], 2);
        $('ovQuat').textContent = 'w ' + fmt(l.quat[0], 3) + (l.quatValid ? '' : ' (invalid)');
        $('ovGravDev').textContent = isFinite(board.gravDev) ? fmt(board.gravDev, 1) + ' deg' : '--';
        $('ovSrc').textContent = (state.link && state.link.mode ? state.link.mode : '-') +
          (l.ready ? ' ready' : ' warming');
      } else {
        $('ovRoll').textContent = '--'; $('ovPitch').textContent = '--'; $('ovYaw').textContent = '--';
        $('ovQuat').textContent = '--'; $('ovGravDev').textContent = '--';
        $('ovSrc').textContent = 'no data';
      }
    }
  }

  var tNow = state.tNow;
  var lockOn = $('chkLockY') && $('chkLockY').checked;

  // ---- scrolling plots
  if (attPlot && visible(attPlot.canvas)) {
    if (tNow > 0) attPlot.draw(state.ring, tNow, lockOn ? lockRange('inpAttYMin', 'inpAttYMax') : null);
    setNoData($('ndAtt'), st !== 'ok' || !state.ring.n, label || 'NO DATA');
  }
  if (gyroPlot && visible(gyroPlot.canvas)) {
    if (tNow > 0) gyroPlot.draw(state.ring, tNow, lockOn ? lockRange('inpGyroYMin', 'inpGyroYMax') : null);
    setNoData($('ndGyro'), st !== 'ok' || !state.ring.n, label || 'NO DATA');
  }

  // ---- density waterfall
  if (heat && visible(heat.canvas)) {
    heat.draw(heatQtyKey(), state.lastSample);
    setNoData($('ndHeat'), st !== 'ok', label || 'NO DATA');
  }

  // ---- statistics at ~4 Hz
  if (now - lastStatsAt >= STATS_MS) {
    lastStatsAt = now;
    var sc = $('stConn');
    if (sc) sc.textContent = st === 'ok' ? 'live' : (label || st);
    updateStats();
  }
}

// ----------------------------- init -----------------------------

function init() {
  var cv3d = $('cv3d');
  board = new BoardView(cv3d);
  if (!board.ok) {
    var ov = $('ov3d');
    if (ov) ov.textContent = 'WebGL unavailable: ' + (board.err || 'no context') + ' - the plots still work';
    toast('WebGL unavailable - 3D view disabled');
  }
  attPlot = new TimeSeriesPlot($('cvAtt'), ATT_SERIES, { span: PLOT_SPAN_S, unit: 'deg', title: 'attitude' });
  gyroPlot = new TimeSeriesPlot($('cvGyro'), GYRO_SERIES, { span: PLOT_SPAN_S, unit: 'deg/s', title: 'gyro (trunk)' });
  heat = new Heatmap($('cvHeat'), HEAT_COLS, HEAT_BINS);

  buildStatsPanel();
  wireUi();
  updateLinkUi();
  fetchPorts();
  connectWS();
  requestAnimationFrame(tick);
}

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', init);
} else {
  init();
}

})();
