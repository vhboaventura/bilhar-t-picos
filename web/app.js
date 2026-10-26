'use strict';

// Frontend: só desenha e envia entrada. Toda a matemática (T1, T2, física das
// bolas) roda no servidor Julia (src/Server.jl + src/BilharSim.jl).

const $ = (id) => document.getElementById(id);
const canvas = $('table');
const ctx = canvas.getContext('2d');

const COLORS = ['#f7cc1a', '#1a45c7', '#db2620', '#6b2994', '#f58019', '#168542', '#851a29', '#121212'];
const CYAN = '#2edcf5', RED = '#ff5148', YELLOW = '#ffd640';

let W = 200, H = 100;                           // mesa (unidades do modelo), vem de /api/info
const view = { x0: -50, x1: 217, y0: -17, y1: 117 };
let k = 1;                                      // pixels CSS por unidade do modelo

const cue = { P: [-1.5, 50], d: [1, 0], dragging: false };
let state = null;
let frameNo = 0;

// ---------------------------------------------------------------- geometria
const X = (x) => (x - view.x0) * k;
const Y = (y) => (view.y1 - y) * k;
const S = (len) => len * k;

function resize() {
  const stage = canvas.parentElement;
  const aspect = (view.x1 - view.x0) / (view.y1 - view.y0);
  const maxW = stage.clientWidth - 32;
  const maxH = window.innerHeight - 110;
  const w = Math.max(320, Math.min(maxW, maxH * aspect));
  const h = w / aspect;
  const dpr = window.devicePixelRatio || 1;
  canvas.style.width = w + 'px';
  canvas.style.height = h + 'px';
  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  k = w / (view.x1 - view.x0);
  for (const c of [errC, timeC]) sizeChart(c);
}

function sizeChart(c) {
  const dpr = window.devicePixelRatio || 1;
  const r = c.getBoundingClientRect();
  c.width = Math.max(1, Math.round(r.width * dpr));
  c.height = Math.max(1, Math.round(r.height * dpr));
  c.getContext('2d').setTransform(dpr, 0, 0, dpr, 0, 0);
}

// ----------------------------------------------------------------- desenho
function rrect(x, y, w, h, r) {
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(x, y, w, h, r); else ctx.rect(x, y, w, h);
}

function drawTable() {
  const pad = 9;
  // madeira
  let g = ctx.createLinearGradient(X(-pad), Y(H + pad), X(W + pad), Y(-pad));
  g.addColorStop(0, '#5c3318'); g.addColorStop(1, '#3a1e0c');
  rrect(X(-pad), Y(H + pad), S(W + 2 * pad), S(H + 2 * pad), S(5));
  ctx.fillStyle = g; ctx.fill();
  ctx.lineWidth = 2; ctx.strokeStyle = '#2b1507'; ctx.stroke();
  // trilho interno
  rrect(X(-0.45 * pad), Y(H + 0.45 * pad), S(W + 0.9 * pad), S(H + 0.9 * pad), S(3));
  ctx.fillStyle = '#6a3b1c'; ctx.fill();
  // feltro
  g = ctx.createRadialGradient(X(W / 2), Y(H / 2), S(10), X(W / 2), Y(H / 2), S(W * 0.62));
  g.addColorStop(0, '#0a6a3c'); g.addColorStop(1, '#064a2a');
  ctx.fillStyle = g; ctx.fillRect(X(0), Y(H), S(W), S(H));
  ctx.lineWidth = 3; ctx.strokeStyle = '#033a20'; ctx.strokeRect(X(0), Y(H), S(W), S(H));
  // diamantes
  ctx.fillStyle = '#e1cd9e';
  const dia = (x, y) => { ctx.save(); ctx.translate(X(x), Y(y)); ctx.rotate(Math.PI / 4);
    ctx.fillRect(-2.5, -2.5, 5, 5); ctx.restore(); };
  for (let i = 1; i <= 7; i++) if (i !== 4) { dia(W * i / 8, -0.72 * pad); dia(W * i / 8, H + 0.72 * pad); }
  for (let i = 1; i <= 3; i++) { dia(-0.72 * pad, H * i / 4); dia(W + 0.72 * pad, H * i / 4); }
  // caçapas
  for (const [x, y] of [[0, 0], [W, 0], [0, H], [W, H], [W / 2, -0.1], [W / 2, H + 0.1]]) {
    const pg = ctx.createRadialGradient(X(x), Y(y), 0, X(x), Y(y), S(7.8));
    pg.addColorStop(0, '#000'); pg.addColorStop(0.8, '#050505'); pg.addColorStop(1, '#1a1109');
    ctx.fillStyle = pg; ctx.beginPath(); ctx.arc(X(x), Y(y), S(7.8), 0, 2 * Math.PI); ctx.fill();
  }
}

function drawBall(x, y, r, n) {
  const cx = X(x), cy = Y(y), R = S(r);
  // sombra
  const sg = ctx.createRadialGradient(cx + 0.2 * R, cy + 0.25 * R, R * 0.4, cx + 0.2 * R, cy + 0.25 * R, R * 1.15);
  sg.addColorStop(0, 'rgba(0,0,0,.45)'); sg.addColorStop(1, 'rgba(0,0,0,0)');
  ctx.fillStyle = sg; ctx.beginPath(); ctx.arc(cx + 0.2 * R, cy + 0.25 * R, R * 1.15, 0, 2 * Math.PI); ctx.fill();
  // corpo (1–8 lisas, 9+ listradas)
  const base = COLORS[(n - 1) % 8];
  const striped = n > 8;
  const body = ctx.createRadialGradient(cx - 0.35 * R, cy - 0.38 * R, R * 0.1, cx, cy, R);
  if (striped) { body.addColorStop(0, '#ffffff'); body.addColorStop(1, '#c9ccd2'); }
  else { body.addColorStop(0, shade(base, 0.45)); body.addColorStop(0.55, base); body.addColorStop(1, shade(base, -0.45)); }
  ctx.fillStyle = body; ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.fill();
  if (striped) {
    ctx.save(); ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.clip();
    const bg = ctx.createLinearGradient(0, cy - R, 0, cy + R);
    bg.addColorStop(0, shade(base, 0.3)); bg.addColorStop(0.5, base); bg.addColorStop(1, shade(base, -0.4));
    ctx.fillStyle = bg; ctx.fillRect(cx - R, cy - 0.58 * R, 2 * R, 1.16 * R);
    ctx.restore();
  }
  ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(0,0,0,.55)';
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, 2 * Math.PI); ctx.stroke();
  // disco com número
  ctx.fillStyle = '#f7f5ee'; ctx.beginPath(); ctx.arc(cx, cy, R * 0.5, 0, 2 * Math.PI); ctx.fill();
  if (R > 5) {
    ctx.fillStyle = '#101010'; ctx.font = `700 ${Math.max(6, R * 0.62)}px system-ui, sans-serif`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(String(n), cx, cy + R * 0.04);
  }
  // brilho
  ctx.fillStyle = 'rgba(255,255,255,.5)';
  ctx.beginPath(); ctx.ellipse(cx - 0.4 * R, cy - 0.42 * R, R * 0.2, R * 0.13, -0.6, 0, 2 * Math.PI); ctx.fill();
}

function shade(hex, amt) {
  const n = parseInt(hex.slice(1), 16);
  const f = (v) => Math.max(0, Math.min(255, Math.round(amt >= 0 ? v + (255 - v) * amt : v * (1 + amt))));
  return `rgb(${f(n >> 16)},${f((n >> 8) & 255)},${f(n & 255)})`;
}

function ring(x, y, r, color, width, dash) {
  ctx.save(); ctx.lineWidth = width; ctx.strokeStyle = color; if (dash) ctx.setLineDash(dash);
  ctx.beginPath(); ctx.arc(X(x), Y(y), S(r), 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
}

function line(x1, y1, x2, y2, color, width, dash, cap = 'butt') {
  ctx.save(); ctx.lineWidth = width; ctx.strokeStyle = color; ctx.lineCap = cap;
  if (dash) ctx.setLineDash(dash);
  ctx.beginPath(); ctx.moveTo(X(x1), Y(y1)); ctx.lineTo(X(x2), Y(y2)); ctx.stroke(); ctx.restore();
}

function drawCue() {
  const [px, py] = cue.P, [dx, dy] = cue.d;
  // taco: da ponta (P) para trás, afinando na ponta
  const seg = (a, b, wa, wb, color) => {
    const nx = -dy, ny = dx;
    const ax = px - a * dx, ay = py - a * dy, bx = px - b * dx, by = py - b * dy;
    ctx.beginPath();
    ctx.moveTo(X(ax + nx * wa / 2), Y(ay + ny * wa / 2));
    ctx.lineTo(X(bx + nx * wb / 2), Y(by + ny * wb / 2));
    ctx.lineTo(X(bx - nx * wb / 2), Y(by - ny * wb / 2));
    ctx.lineTo(X(ax - nx * wa / 2), Y(ay - ny * wa / 2));
    ctx.closePath(); ctx.fillStyle = color; ctx.fill();
  };
  seg(3, 112, 1.4, 3.2, '#2b150a');          // sombra/contorno
  seg(5, 62, 1.1, 2.1, '#eacb8f');            // haste clara
  seg(62, 112, 2.1, 3.0, '#4a2511');          // base escura
  seg(2.4, 5, 1.1, 1.1, '#f4f4f4');           // virola
  seg(0, 2.4, 1.0, 1.1, '#4d9cf2');           // ponta com giz
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawTable();
  if (!state) return;
  const b = state.balls, n = b.length / 3;
  const showT1 = $('showT1').checked;

  for (let i = 0; i < n; i++) drawBall(b[3 * i], b[3 * i + 1], b[3 * i + 2], i + 1);

  // anéis dos alvos
  if (state.t2) { const i = state.t2.idx; ring(b[3 * i], b[3 * i + 1], b[3 * i + 2] + 2.2, CYAN, 3); }
  if (showT1 && state.t1 && !state.same) {
    const i = state.t1.idx; ring(b[3 * i], b[3 * i + 1], b[3 * i + 2] + 4.5, RED, 2, [5, 4]);
  }

  // linha de mira e trajetória da bola atingida
  const [px, py] = cue.P, [dx, dy] = cue.d;
  const end = state.t2 ? [state.t2.x, state.t2.y] : [px + 400 * dx, py + 400 * dy];
  line(px, py, end[0], end[1], 'rgba(255,255,255,.65)', 1.6, [6, 6]);
  if (state.t2) {
    const i = state.t2.idx, cx = b[3 * i], cy = b[3 * i + 1];
    let nx = cx - state.t2.x, ny = cy - state.t2.y; const m = Math.hypot(nx, ny) || 1; nx /= m; ny /= m;
    const ex = cx + 38 * nx, ey = cy + 38 * ny;
    line(cx, cy, ex, ey, YELLOW, 2.6);
    ctx.fillStyle = YELLOW; ctx.beginPath();
    ctx.moveTo(X(ex + 2.2 * nx), Y(ey + 2.2 * ny));
    ctx.lineTo(X(ex - 1.6 * ny), Y(ey + 1.6 * nx));
    ctx.lineTo(X(ex + 1.6 * ny), Y(ey - 1.6 * nx)); ctx.closePath(); ctx.fill();
  }
  drawCue();

  // marcadores de contato
  if (showT1 && state.t1) {
    const x = X(state.t1.x), y = Y(state.t1.y), s = 7;
    for (const [w, c] of [[5, '#fff'], [3, RED]]) {
      ctx.lineWidth = w; ctx.strokeStyle = c; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(x - s, y - s); ctx.lineTo(x + s, y + s);
      ctx.moveTo(x - s, y + s); ctx.lineTo(x + s, y - s); ctx.stroke();
    }
  }
  if (state.t2) {
    ctx.lineWidth = 3; ctx.strokeStyle = CYAN;
    ctx.beginPath(); ctx.arc(X(state.t2.x), Y(state.t2.y), 11, 0, 2 * Math.PI); ctx.stroke();
  }
}

// ----------------------------------------------------------------- gráficos
const errC = $('errc'), timeC = $('timec');
const N = 240;
const hist = { err: Array(N).fill(null), miss: Array(N).fill(false), anchor: Array(N).fill(false),
               t1: Array(N).fill(null), t2: Array(N).fill(null) };
let timeTop = 1;
const pushHist = (a, v) => { a.shift(); a.push(v); };

function drawErrChart() {
  const c = errC.getContext('2d'), w = errC.clientWidth, h = errC.clientHeight;
  c.clearRect(0, 0, w, h);
  const lo = -15, hi = 3, yy = (v) => 6 + (1 - (Math.log10(v) - lo) / (hi - lo)) * (h - 12);
  c.font = '10px system-ui'; c.fillStyle = '#6f7888'; c.strokeStyle = '#242a36'; c.lineWidth = 1;
  for (const [v, t] of [[1e-12, '1e-12'], [1e-6, '1e-6'], [1, '1']]) {
    c.beginPath(); c.moveTo(34, yy(v)); c.lineTo(w, yy(v)); c.stroke(); c.fillText(t, 3, yy(v) + 3);
  }
  const xx = (i) => 36 + (i / (N - 1)) * (w - 40);
  c.strokeStyle = CYAN; c.lineWidth = 1.6; c.beginPath(); let pen = false;
  hist.err.forEach((v, i) => {
    if (v === null) { pen = false; return; }
    const y = yy(Math.max(v, 1e-15)); pen ? c.lineTo(xx(i), y) : c.moveTo(xx(i), y); pen = true;
  });
  c.stroke();
  c.fillStyle = RED; hist.miss.forEach((m, i) => { if (m) { c.beginPath(); c.arc(xx(i), yy(30), 2.6, 0, 7); c.fill(); } });
  c.fillStyle = YELLOW; hist.anchor.forEach((a, i) => { if (a) { const x = xx(i), y = h - 8;
    c.beginPath(); c.moveTo(x, y - 4); c.lineTo(x - 3, y + 2); c.lineTo(x + 3, y + 2); c.fill(); } });
}

function drawTimeChart() {
  const c = timeC.getContext('2d'), w = timeC.clientWidth, h = timeC.clientHeight;
  c.clearRect(0, 0, w, h);
  const peak = Math.max(0.05, ...hist.t1.filter((v) => v !== null), ...hist.t2.filter((v) => v !== null));
  timeTop += (peak * 1.25 - timeTop) * 0.05;
  const yy = (v) => h - 6 - (v / timeTop) * (h - 12), xx = (i) => 36 + (i / (N - 1)) * (w - 40);
  c.font = '10px system-ui'; c.fillStyle = '#6f7888'; c.strokeStyle = '#242a36'; c.lineWidth = 1;
  for (const v of [0, timeTop / 2, timeTop]) {
    c.beginPath(); c.moveTo(34, yy(v)); c.lineTo(w, yy(v)); c.stroke(); c.fillText(v.toFixed(1), 3, yy(v) + 3);
  }
  for (const [arr, col] of [[hist.t1, RED], [hist.t2, CYAN]]) {
    c.strokeStyle = col; c.lineWidth = 2; c.beginPath(); let pen = false;
    arr.forEach((v, i) => { if (v === null) { pen = false; return; }
      pen ? c.lineTo(xx(i), yy(v)) : c.moveTo(xx(i), yy(v)); pen = true; });
    c.stroke();
  }
}

// ------------------------------------------------------------------ medidas
const fmtUs = (ns) => (ns / 1e3).toFixed(2) + ' µs';
function updateStats(s) {
  $('s_t1').textContent = fmtUs(s.ema_bf);
  $('s_t2').textContent = fmtUs(s.ema_tr);
  $('s_speed').textContent = (s.ema_bf / Math.max(s.ema_tr, 1)).toFixed(1) + '×';
  $('s_err').textContent = s.err === null ? '—' : s.err.toExponential(1);
  $('s_newton').textContent = s.frames > 0 ? (s.newton_total / Math.max(s.frames - (s.bf_calls - 1), 1)).toFixed(2) : '—';
  $('s_bf').textContent = s.recent_bf_pct.toFixed(0) + '% (últimos 120)';
  const t = $('s_target');
  if (s.same) { t.textContent = s.t1 ? `correto (bola ${s.t2.idx + 1})` : 'sem contato'; t.className = 'ok'; }
  else { t.textContent = 'desatualizado'; t.className = 'bad'; }
}

// ----------------------------------------------------------------- controles
for (const id of ['newton', 'reanchor', 'nballs', 'radius']) {
  const el = $(id), out = $(id + '_o');
  const show = () => { out.textContent = id === 'radius' ? Number(el.value).toFixed(1) : el.value; };
  el.addEventListener('input', show); show();
}
let configTimer = null;
function postConfig(body) {
  return fetch('/api/config', { method: 'POST', body: JSON.stringify(body) }).catch(() => {});
}
for (const id of ['nballs', 'radius']) {
  $(id).addEventListener('input', () => {
    clearTimeout(configTimer);
    configTimer = setTimeout(() => postConfig({ n_balls: +$('nballs').value, radius: +$('radius').value }), 120);
  });
}
$('shuffle').addEventListener('click', () => postConfig({ shuffle: true }));

// mouse / toque: o clique define a ponta do taco P, o ponteiro define a direção d
function toWorld(ev) {
  const r = canvas.getBoundingClientRect();
  return [view.x0 + (ev.clientX - r.left) / k, view.y1 - (ev.clientY - r.top) / k];
}
canvas.addEventListener('pointerdown', (ev) => {
  canvas.setPointerCapture(ev.pointerId);
  cue.dragging = true; $('auto').checked = false; cue.P = toWorld(ev);
});
canvas.addEventListener('pointermove', (ev) => {
  if (!cue.dragging) return;
  const w = toWorld(ev), vx = w[0] - cue.P[0], vy = w[1] - cue.P[1], m = Math.hypot(vx, vy);
  if (m > 0.05) cue.d = [vx / m, vy / m];
});
const endDrag = () => { cue.dragging = false; };
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);

// --------------------------------------------------------------------- laço
const nextFrame = () => new Promise(requestAnimationFrame);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function loop() {
  let last = performance.now();
  const t0 = last;
  for (;;) {
    await nextFrame();
    const now = performance.now();
    const dt = Math.min((now - last) / 1000, 0.05);
    last = now;
    if ($('auto').checked) {
      const t = (now - t0) / 1000;
      cue.P = [-1.5, H * (0.5 + 0.30 * Math.sin(0.17 * t))];
      const th = 0.42 * Math.sin(0.23 * t) + 0.07 * Math.sin(0.9 * t);
      cue.d = [Math.cos(th), Math.sin(th)];
    }
    let s;
    try {
      const r = await fetch('/api/tick', { method: 'POST', body: JSON.stringify({
        dt, P: cue.P, d: cue.d, paused: $('paused').checked, predictor: $('predictor').value,
        newton_iters: +$('newton').value, recompute_every: +$('reanchor').value }) });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      s = await r.json();
      $('offline').hidden = true;
    } catch (e) {
      $('offline').hidden = false;
      await sleep(700);
      continue;
    }
    state = s; frameNo++;
    pushHist(hist.err, s.same && s.err !== null ? s.err : null);
    pushHist(hist.miss, !s.same);
    pushHist(hist.anchor, s.anchored);
    pushHist(hist.t1, frameNo > 3 ? s.ema_bf / 1e3 : null);
    pushHist(hist.t2, frameNo > 3 ? s.ema_tr / 1e3 : null);
    draw(); drawErrChart(); drawTimeChart();
    if (frameNo % 6 === 0) updateStats(s);
  }
}

async function init() {
  try {
    const info = await (await fetch('/api/info')).json();
    W = info.width; H = info.height;
    view.x1 = W + 17; view.y1 = H + 17;
    $('nballs').value = info.n_balls; $('radius').value = info.radius;
    $('nballs').dispatchEvent(new Event('input')); $('radius').dispatchEvent(new Event('input'));
  } catch (e) { $('offline').hidden = false; }
  resize();
  window.addEventListener('resize', resize);
  loop();
}
init();
