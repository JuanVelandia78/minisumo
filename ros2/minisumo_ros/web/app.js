// CAR-LOSS · consola en vivo. Datos por WebSocket desde panel_server (ROS2).
import * as THREE from 'three';
import { STLLoader } from './vendor/STLLoader.js';
import { OrbitControls } from './vendor/OrbitControls.js';

const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => [...r.querySelectorAll(s)];
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const C = {
  tawara: css('--tawara'), muted: css('--muted'), faint: css('--faint'), rule: css('--rule'),
  car: css('--car'), fusa: css('--fusa'), umber: css('--umber'), umberHi: css('--umber-hi'),
  amber: css('--amber'), red: css('--red'), ok: css('--ok'), shell: css('--shell'),
};
const DEG = 180 / Math.PI;
const R_WHEEL = 0.0162, B_WHEEL = 0.079, W_MAX = 13.5;
const DOJO_R = 0.385, TAWARA = 0.025, INNER_R = DOJO_R - TAWARA, ROBOT_HALF = 0.05;
const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)').matches;

// ---------- formato (coma decimal y signo menos tipográfico) ----------
const num = (v, d = 1) => (v == null || !isFinite(v)) ? '—' : v.toFixed(d).replace('-', '−').replace('.', ',');
const sgn = (v, d = 1) => (v == null || !isFinite(v)) ? '—' : (v > 0 ? '+' : '') + num(v, d);
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const clock = () => new Date().toLocaleTimeString('es-CO', { hour12: false });
const setText = (id, txt) => { const el = document.getElementById(id); if (el && el.textContent !== txt) el.textContent = txt; };

// ---------- estado ----------
const S = { estopT: 0, ws: null, wsOk: false, wsRate: 0, wsCount: 0, last: null, view: 'vivo', estop: false,
  everRobot: false, robotOk: false, gloveOk: false, batLevel: 'none', edgeLevel: 'ok', lastEdgeLog: 0,
  push: null, paused: false, win: 10, showFw: false };
const SER = new Map();               // serie -> {t:[], v:[]}
const HIST_V = { t: [], v: [] };     // velocidad para la mini historia (20 s)
const HIST_BAT = { t: [], v: [] };   // batería en la sesión
let tNow = 0;

function addSeries(name, pts) {
  let s = SER.get(name);
  if (!s) { s = { t: [], v: [] }; SER.set(name, s); }
  for (const [t, v] of pts) { s.t.push(t); s.v.push(v); }
  const cut = tNow - 25;
  let i = 0; while (i < s.t.length && s.t[i] < cut) i++;
  if (i > 0) { s.t.splice(0, i); s.v.splice(0, i); }
}

// ---------- conexión ----------
function connect() {
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  S.ws = ws;
  ws.onopen = () => { S.wsOk = true; logEv('Panel conectado a ROS2', 'good'); if (S.estop) send({ t: 'estop', v: true }); subscribe(); };
  ws.onclose = () => {
    if (S.wsOk) logEv('Se perdió la conexión con el panel; reintentando', 'crit');
    S.wsOk = false; setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    S.wsCount++; tNow = d.t; S.last = d;
    if (!S.paused) for (const k in d.series) addSeries(k, d.series[k]);
    onState(d);
    send({ t: 'ack' });                       // listo para el siguiente: el servidor no acumula cola
  };
}
const send = (o) => { if (S.ws && S.ws.readyState === 1) S.ws.send(JSON.stringify(o)); };
setInterval(() => { S.wsRate = S.wsCount; S.wsCount = 0; }, 1000);

// ---------- registro de eventos ----------
const logEl = $('#log');
function logEv(txt, kind = '') {
  logEl.querySelector('.log-empty')?.closest('li').remove();
  const li = document.createElement('li');
  li.innerHTML = `<time>${clock()}</time><span class="ev-${kind}"></span>`;
  li.lastChild.textContent = txt;
  logEl.prepend(li);
  while (logEl.children.length > 40) logEl.lastChild.remove();
}
let toastT = 0;
function toast(txt) {
  const t = $('#toast'); t.textContent = txt; t.hidden = false;
  clearTimeout(toastT); toastT = setTimeout(() => { t.hidden = true; }, 2600);
}

// ---------- estado entrante ----------
let lastUi = 0;
function onState(d) {
  const L = d.link || {};
  const rAge = L.robot?.age, gAge = L.glove?.age;
  const robotOk = rAge != null && rAge < 0.6;
  const gloveOk = gAge != null && gAge < 0.8;
  if (robotOk !== S.robotOk) { logEv(robotOk ? 'Robot conectado' : 'Se perdió la telemetría del robot', robotOk ? 'good' : 'crit'); S.robotOk = robotOk; }
  if (gloveOk !== S.gloveOk) { logEv(gloveOk ? 'Guante conectado' : 'Guante sin datos', gloveOk ? 'good' : 'warn'); S.gloveOk = gloveOk; }
  if (robotOk) S.everRobot = true;
  if (d.estop !== S.estop && performance.now() - S.estopT > 1500) setEstopUi(d.estop);

  if (robotOk) {
    HIST_V.t.push(d.t); HIST_V.v.push(d.v * 100);
    while (HIST_V.t.length && HIST_V.t[0] < d.t - 20) { HIST_V.t.shift(); HIST_V.v.shift(); }
  }
  if (d.v_bat != null && d.v_bat > 3 && (!HIST_BAT.t.length || d.t - HIST_BAT.t[HIST_BAT.t.length - 1] > 2)) {
    HIST_BAT.t.push(d.t); HIST_BAT.v.push(d.v_bat);
  }
  detectEvents(d, robotOk);
  paintGoal(d.goal);
  scene.update(d, robotOk);
  const now = performance.now();
  if (now - lastUi > 55) { lastUi = now; paintUi(d, robotOk, gloveOk); }
}

function detectEvents(d, robotOk) {
  if (!robotOk) return;
  // batería
  const vb = d.v_bat;
  const lvl = vb == null || vb < 3 ? 'none' : vb < 6.6 ? 'crit' : vb < 7.0 ? 'warn' : 'ok';
  if (lvl !== S.batLevel) {
    if (lvl === 'warn') logEv(`Batería baja: ${num(vb, 2)} V`, 'warn');
    if (lvl === 'crit') logEv(`Batería crítica: ${num(vb, 2)} V, carga la 2S`, 'crit');
    S.batLevel = lvl;
  }
  // borde
  const dist = INNER_R - Math.hypot(d.x, d.y) - ROBOT_HALF;
  const el = dist <= 0.06 ? 'crit' : dist <= 0.12 ? 'warn' : 'ok';   // mismo umbral que el arco 3D
  if (el !== 'ok' && el !== S.edgeLevel && performance.now() - S.lastEdgeLog > 3000) {
    logEv(`A ${num(Math.max(0, dist) * 100, 0)} cm del borde`, el); S.lastEdgeLog = performance.now();
  }
  S.edgeLevel = el;
  // perturbación: el rumbo real se aleja del pedido y el control lo devuelve
  const moving = Math.abs(d.wl_ref) + Math.abs(d.wr_ref) > 0.5 && d.th_des != null;
  if (!moving) { S.push = null; return; }
  const err = wrap(d.th - d.th_des) * DEG;
  if (!S.push && Math.abs(err) > 8) S.push = { t0: d.t, peak: err };
  else if (S.push) {
    if (Math.abs(err) > Math.abs(S.push.peak)) S.push.peak = err;
    if (Math.abs(err) < 3) {
      logEv(`Perturbación de ${sgn(S.push.peak, 1)}° corregida en ${num(d.t - S.push.t0, 2)} s`, 'good');
      S.push = null;
    } else if (d.t - S.push.t0 > 4) { logEv(`El rumbo sigue desviado ${sgn(err, 0)}°`, 'warn'); S.push = null; }
  }
}

const MODES = { 0: 'detenido', 1: 'velocidad PID', 2: 'lazo abierto' };
function paintUi(d0, robotOk, gloveOk) {
  const L = d0.link || {};
  // Sin telemetría fresca, las lecturas del robot se muestran como faltantes, nunca como cero
  const d = robotOk ? d0 : { ...d0, v: NaN, th: NaN, gz: NaN, x: NaN, y: NaN, wl: NaN, wr: NaN, wl_ref: NaN, wr_ref: NaN,
    duty_l: NaN, duty_r: NaN, th_des: null, v_bat: null, i_bat: null, p_bat: null };
  linkPill('#link-robot', robotOk ? (L.robot.rate > 100 ? 'ok' : 'warn') : 'off',
    robotOk ? `${L.robot.rate} /s` : (L.robot?.age != null ? `hace ${num(L.robot.age, 0)} s` : 'sin datos'));
  linkPill('#link-glove', gloveOk ? 'ok' : 'off', gloveOk ? `${L.glove.rate} /s` : 'sin datos');
  $('#link-mode .link-v').textContent = robotOk ? (MODES[d.mode] ?? '—') : '—';

  // telemetría
  $('.telemetry').classList.toggle('stale', !robotOk);
  $('#stage-empty').classList.toggle('gone', S.everRobot);
  if (S.everRobot && !robotOk) { $('#stage-empty .se-t').textContent = 'Telemetría detenida'; $('#stage-empty').classList.remove('gone'); }
  setText('t-v', num(d.v * 100, 1));
  setText('t-th', num(wrap(d.th) * DEG, 1));
  setText('t-err', d.th_des == null ? '—' : `${sgn(wrap(d.th - d.th_des) * DEG, 1)}°`);
  setText('t-gz', num(d.gz, 0));
  setText('t-pos', `${num(d.x * 100, 0)} , ${num(d.y * 100, 0)} cm`);
  wheel('l', d.wl, d.wl_ref, d.duty_l); wheel('r', d.wr, d.wr_ref, d.duty_r);
  setText('t-vb', d.v_bat != null && d.v_bat > 3 ? num(d.v_bat, 2) : '—');
  const batOk = d.v_bat != null && d.v_bat > 3;          // sin batería el INA219 da corriente sin sentido
  setText('t-ib', batOk && d.i_bat != null ? num(d.i_bat * 1000, 0) : '—');
  setText('t-pb', batOk && d.p_bat != null ? num(d.p_bat, 2) : '—');
  $('#mod-bat').dataset.level = S.batLevel;
  const manual = S.drive.active;
  const cmdFresh = d.cmd_age != null && d.cmd_age < 1;
  setText('t-src', manual ? 'manual' : (gloveOk && cmdFresh ? 'guante' : (cmdFresh ? 'cmd_vel' : '—')));
  setText('t-cv', cmdFresh ? num(d.cmd_v * 100, 1) : '—');
  setText('t-cw', cmdFresh ? num(d.cmd_w, 2) : '—');
  // escenario
  const dist = INNER_R - Math.hypot(d.x, d.y) - ROBOT_HALF;
  setText('edge-v', robotOk ? num(Math.max(0, dist) * 100, 0) : '—');
  $('#edge').dataset.level = robotOk ? S.edgeLevel : 'ok';
  setText('lg-real', `${num(wrap(d.th) * DEG, 1)}°`);
  setText('lg-cmd', d.th_des == null ? '—' : `${num(wrap(d.th_des) * DEG, 1)}°`);
  drawHist();
  if (S.view === 'guante') paintGlove(d, gloveOk);
  if (S.view === 'sistema') paintSystem(d, robotOk, gloveOk);
}
const STATE_TXT = { ok: 'activo', warn: 'intermitente', off: 'sin datos' };
function linkPill(sel, state, txt) {
  const el = $(sel); el.dataset.state = state; el.querySelector('.link-v').textContent = txt;
  el.setAttribute('aria-label', `${el.querySelector('b').textContent}: ${STATE_TXT[state]}, ${txt}`);
  el.title = el.getAttribute('aria-label');
}
function wheel(s, w, ref, duty) {
  setText(`t-w${s}`, num(w, 1)); setText(`t-w${s}r`, num(ref, 1)); setText(`t-d${s}`, num(duty, 2));
  const m = $(`#m-w${s}`);
  m.querySelector('.m-meas').style.width = `${isFinite(w) ? Math.min(100, Math.abs(w) / W_MAX * 100) : 0}%`;
  m.querySelector('.m-ref').style.left = `calc(${isFinite(ref) ? Math.min(100, Math.abs(ref) / W_MAX * 100) : 0}% - 1px)`;
}

// ---------- lienzos ----------
function fitCanvas(cv) {
  const dpr = Math.min(devicePixelRatio || 1, 2);
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}
function drawHist() {
  const cv = $('#h-v'); if (!cv.clientWidth) return;
  const { ctx, w, h } = fitCanvas(cv); ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = C.rule; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(0, h - .5); ctx.lineTo(w, h - .5); ctx.stroke();
  const n = HIST_V.t.length; if (n < 2) return;
  const t1 = HIST_V.t[n - 1], t0 = t1 - 20;
  const vmax = Math.max(5, ...HIST_V.v.map(Math.abs));
  ctx.strokeStyle = C.tawara; ctx.lineWidth = 1.25; ctx.beginPath();
  for (let i = 0; i < n; i++) {
    const x = (HIST_V.t[i] - t0) / 20 * w, y = h - 3 - (Math.abs(HIST_V.v[i]) / vmax) * (h - 20);
    i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
  }
  ctx.stroke();
  ctx.fillStyle = C.faint; ctx.font = '500 11px Barlow'; ctx.fillText('−20 s', 2, 11); ctx.textAlign = 'right'; ctx.fillText(`máx ${num(vmax, 0)} cm/s`, w - 2, 11); ctx.textAlign = 'left';
}

// ---------- gráficas de señales ----------
const PLOTS = [
  { id: 'gz', t: 'Giro del robot (eje z)', u: '°/s' },
  { id: 'ax', t: 'Aceleración del robot (eje x)', u: 'm/s²' },
  { id: 'wl', t: 'Velocidad rueda izquierda', u: 'rad/s' },
  { id: 'wr', t: 'Velocidad rueda derecha', u: 'rad/s' },
  { id: 'gp', t: 'Mano: cabeceo', u: '°' },
  { id: 'gr', t: 'Mano: alabeo', u: '°' },
];
const plotsEl = $('#plots');
for (const p of PLOTS) {
  const div = document.createElement('div'); div.className = 'plot';
  div.innerHTML = `<div class="plot-h"><h3 class="plot-t">${p.t}</h3><span class="plot-u">${p.u}</span>
    <span class="plot-s"><span>σ crudo <b data-s="raw">—</b></span><span>σ Kalman <b data-s="k">—</b></span><span class="red">reducción <b data-s="red">—</b></span></span></div>
    <canvas aria-label="${p.t}"></canvas>`;
  plotsEl.append(div); p.el = div; p.cv = div.querySelector('canvas');
}
function windowed(name, t0) {
  const s = SER.get(name); if (!s) return null;
  let i = 0; while (i < s.t.length && s.t[i] < t0) i++;
  return { t: s.t.slice(i), v: s.v.slice(i) };
}
const sd = (a) => { if (a.length < 3) return NaN; const m = a.reduce((x, y) => x + y, 0) / a.length; return Math.sqrt(a.reduce((x, y) => x + (y - m) ** 2, 0) / a.length); };
function drawPlot(cv, defs, t1, win, statsEl) {
  const { ctx, w, h } = fitCanvas(cv); ctx.clearRect(0, 0, w, h);
  const t0 = t1 - win, padL = 42, padB = 18, pw = w - padL - 4, ph = h - padB - 6;
  const data = defs.map((d) => ({ ...d, s: windowed(d.name, t0) })).filter((d) => d.s && d.s.t.length > 1);
  let lo = Infinity, hi = -Infinity;
  for (const d of data) for (const v of d.s.v) { if (v < lo) lo = v; if (v > hi) hi = v; }
  if (!isFinite(lo)) { lo = -1; hi = 1; }
  if (hi - lo < 1e-3) { hi += .5; lo -= .5; }
  const pad = (hi - lo) * .08; lo -= pad; hi += pad;
  const X = (t) => padL + (t - t0) / win * pw, Y = (v) => 6 + (hi - v) / (hi - lo) * ph;
  ctx.font = '500 11px Barlow'; ctx.fillStyle = C.faint; ctx.strokeStyle = 'rgba(58,49,43,.7)'; ctx.lineWidth = 1;
  for (let k = 0; k <= 4; k++) {
    const v = lo + (hi - lo) * k / 4, y = Math.round(Y(v)) + .5;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - 4, y); ctx.stroke();
    ctx.textAlign = 'right'; ctx.fillText(num(v, Math.abs(hi - lo) < 4 ? 2 : (Math.abs(hi - lo) < 40 ? 1 : 0)), padL - 6, y + 4);
  }
  ctx.textAlign = 'center';
  for (let k = 0; k <= 4; k++) { const tt = t0 + win * k / 4; ctx.textAlign = k === 4 ? 'right' : (k === 0 ? 'left' : 'center'); ctx.fillText(k === 4 ? 'ahora' : `−${num(win - win * k / 4, 0)} s`, X(tt), h - 3); }
  for (const d of data) {
    ctx.strokeStyle = d.color; ctx.lineWidth = d.lw; ctx.globalAlpha = d.alpha ?? 1; ctx.setLineDash(d.dash || []); ctx.beginPath();
    const { t, v } = d.s;
    for (let i = 0; i < t.length; i++) { const x = X(t[i]), y = Y(v[i]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }
    ctx.stroke(); ctx.globalAlpha = 1; ctx.setLineDash([]);
  }
  if (statsEl) {
    const raw = data.find((d) => d.kind === 'raw'), k = data.find((d) => d.kind === 'k');
    const a = raw ? sd(raw.s.v) : NaN, b = k ? sd(k.s.v) : NaN;
    statsEl.querySelector('[data-s=raw]').textContent = num(a, 2);
    statsEl.querySelector('[data-s=k]').textContent = num(b, 2);
    statsEl.querySelector('[data-s=red]').textContent = isFinite(a) && isFinite(b) && a > 0 ? `${num((1 - b / a) * 100, 0)} %` : '—';
  }
  if (!data.length) { ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.font = '500 13px Barlow'; ctx.fillText('Sin datos en esta ventana', padL + pw / 2, 6 + ph / 2); }
}
const defsFor = (id) => [
  { name: `${id}_raw`, kind: 'raw', color: C.tawara, lw: 1, alpha: .42 },
  ...(S.showFw ? [{ name: `${id}_fw`, kind: 'fw', color: C.umberHi, lw: 1.3 }] : []),
  { name: `${id}_k`, kind: 'k', color: C.fusa, lw: 1.7 },
];
let plotT1 = 0;
function drawPlots() {
  if (!S.paused) plotT1 = tNow;
  for (const p of PLOTS) drawPlot(p.cv, defsFor(p.id), plotT1, S.win, p.el);
}

// ---------- guante ----------
function dial(svg, sideView) {
  const ticks = [];
  for (let a = -90; a <= 90; a += 10) {
    const r1 = 92, r2 = a % 30 === 0 ? 80 : 86, t = (a - 90) * Math.PI / 180;
    ticks.push(`<line x1="${(r1 * Math.cos(t)).toFixed(1)}" y1="${(r1 * Math.sin(t)).toFixed(1)}" x2="${(r2 * Math.cos(t)).toFixed(1)}" y2="${(r2 * Math.sin(t)).toFixed(1)}" stroke="${a % 30 ? C.faint : C.tawara}" stroke-width="${a % 30 ? 1 : 1.5}"/>`);
  }
  const dz = sideView ? 8 : 12, arc = (a1, a2, r) => { const p = (a) => { const t = (a - 90) * Math.PI / 180; return `${(r * Math.cos(t)).toFixed(1)} ${(r * Math.sin(t)).toFixed(1)}`; }; return `M ${p(a1)} A ${r} ${r} 0 0 1 ${p(a2)}`; };
  svg.innerHTML = `
    <circle r="100" fill="${css('--ring')}" stroke="${C.tawara}" stroke-width="6"/>
    <path d="${arc(-dz, dz, 74)}" stroke="${C.umber}" stroke-width="8" fill="none"/>
    <path d="${arc(-30, -dz, 74)}" stroke="${C.rule}" stroke-width="3" fill="none"/>
    <path d="${arc(dz, 30, 74)}" stroke="${C.rule}" stroke-width="3" fill="none"/>
    ${ticks.join('')}
    <text x="-46" y="40" fill="${C.muted}" font-size="13" font-family="Barlow" text-anchor="middle">${sideView ? 'atrás' : 'izq'}</text>
    <text x="46" y="40" fill="${C.muted}" font-size="13" font-family="Barlow" text-anchor="middle">${sideView ? 'adelante' : 'der'}</text>
    <g class="needle"><rect x="-6" y="-78" width="12" height="78" rx="2" fill="${C.car}"/><circle r="9" fill="${C.car}"/></g>`;
}
dial($('#dial-p'), true); dial($('#dial-r'), false);
$('#vec').innerHTML = `<circle r="92" fill="none" stroke="${C.rule}" stroke-width="1.5"/><circle r="46" fill="none" stroke="${C.rule}" stroke-width="1" stroke-dasharray="3 5"/>
  <line x1="-92" y1="0" x2="92" y2="0" stroke="${C.rule}"/><line x1="0" y1="-92" x2="0" y2="92" stroke="${C.rule}"/>
  <text x="0" y="-70" fill="${C.muted}" font-size="15" font-family="Barlow" text-anchor="middle">adelante</text><text x="-70" y="5" fill="${C.muted}" font-size="15" font-family="Barlow" text-anchor="middle">izq</text><text x="70" y="5" fill="${C.muted}" font-size="15" font-family="Barlow" text-anchor="middle">der</text>
  <line id="vec-l" x1="0" y1="0" x2="0" y2="0" stroke="${C.car}" stroke-width="4" stroke-linecap="round"/><circle id="vec-p" r="7" fill="${C.car}"/>`;
const needle = (svg, a) => { const g = svg.querySelector('.needle'); g.style.transform = `rotate(${Math.max(-90, Math.min(90, a))}deg)`; };
function paintGlove(d, ok) {
  const p = d.hand ? d.hand[0] : 0, r = d.hand ? d.hand[1] : 0;
  needle($('#dial-p'), ok ? p : 0); needle($('#dial-r'), ok ? r : 0);
  setText('gl-p', ok && d.hand ? sgn(p, 0) : '—'); setText('gl-r', ok && d.hand ? sgn(r, 0) : '—');
  const fresh = d.cmd_age != null && d.cmd_age < 1;
  const vx = fresh ? -d.cmd_w / 3 * 80 : 0, vy = fresh ? -d.cmd_v / 0.19 * 80 : 0;
  $('#vec-l').setAttribute('x2', vx.toFixed(1)); $('#vec-l').setAttribute('y2', vy.toFixed(1));
  $('#vec-p').setAttribute('cx', vx.toFixed(1)); $('#vec-p').setAttribute('cy', vy.toFixed(1));
  setText('gl-v', fresh ? num(d.cmd_v * 100, 1) : '—'); setText('gl-w', fresh ? num(d.cmd_w, 2) : '—');
  setText('gl-rate', ok ? String(d.link.glove.rate) : '—');
}
function drawGlovePlot() {
  drawPlot($('#gl-plot'), [
    { name: 'gp_raw', kind: 'raw', color: C.tawara, lw: 1, alpha: .4 }, { name: 'gp_k', kind: 'k', color: C.fusa, lw: 1.7 },
    { name: 'gr_raw', kind: 'raw2', color: C.tawara, lw: 1, alpha: .4, dash: [5, 4] }, { name: 'gr_k', kind: 'k2', color: C.fusa, lw: 1.7, dash: [7, 4] },
  ], tNow, 10, null);
}

// ---------- sistema ----------
function paintSystem(d, robotOk, gloveOk) {
  const st = (ok, warn) => ok ? '<span class="st-ok">activo</span>' : (warn ? '<span class="st-warn">intermitente</span>' : '<span class="st-off">sin datos</span>');
  $('#s-ws').innerHTML = S.wsOk ? '<span class="st-ok">conectado</span>' : '<span class="st-off">desconectado</span>';
  setText('s-ws-r', `${S.wsRate} /s`);
  $('#s-rb').innerHTML = st(robotOk && d.link.robot.rate > 100, robotOk);
  setText('s-rb-r', d.link.robot.rate ? `${d.link.robot.rate} /s` : '—');
  setText('s-rb-a', d.link.robot.age != null ? `${num(d.link.robot.age, 1)} s` : '—');
  $('#s-gl').innerHTML = st(gloveOk, false);
  setText('s-gl-r', d.link.glove.rate ? `${d.link.glove.rate} /s` : '—');
  setText('s-gl-a', d.link.glove.age != null ? `${num(d.link.glove.age, 1)} s` : '—');
  setText('s-ticks', `${d.ticks_l} / ${d.ticks_r}`);
  paintUrls(d);
  const cv = $('#bat-plot'); const { ctx, w, h } = fitCanvas(cv); ctx.clearRect(0, 0, w, h);
  const lo = 6.2, hi = 8.6, Y = (v) => 6 + (hi - v) / (hi - lo) * (h - 24);
  ctx.font = '500 11px Barlow';
  for (const [v, c] of [[7.0, C.amber], [6.6, C.red], [8.4, C.faint]]) {
    ctx.strokeStyle = c; ctx.globalAlpha = .55; ctx.setLineDash([3, 4]); ctx.beginPath(); ctx.moveTo(36, Y(v)); ctx.lineTo(w, Y(v)); ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
    ctx.fillStyle = c; ctx.textAlign = 'right'; ctx.fillText(num(v, 1), 30, Y(v) + 4);
  }
  const n = HIST_BAT.t.length;
  if (n > 1) {
    const t0 = HIST_BAT.t[0], t1 = Math.max(HIST_BAT.t[n - 1], t0 + 60);
    ctx.strokeStyle = C.tawara; ctx.lineWidth = 1.5; ctx.beginPath();
    HIST_BAT.t.forEach((t, i) => { const x = 36 + (t - t0) / (t1 - t0) * (w - 40), y = Y(HIST_BAT.v[i]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke(); ctx.fillStyle = C.faint; ctx.textAlign = 'left'; ctx.fillText(`${num((t1 - t0) / 60, 0)} min de sesión`, 38, h - 4);
  } else { ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.fillText('Esperando lecturas del INA219', w / 2, h / 2); }
}
function paintUrls(d) {
  const ips = (d.ips && d.ips.length) ? d.ips : [location.hostname];
  const txt = ips.map((ip) => `http://${ip}:${d.port || location.port}`).join('\n');
  setText('s-url', txt);
}

// ---------- navegación ----------
function subscribe() {
  let series = [];
  if (S.view === 'senales') for (const p of PLOTS) series.push(`${p.id}_raw`, `${p.id}_k`, ...(S.showFw ? [`${p.id}_fw`] : []));
  else if (S.view === 'guante') series = ['gp_raw', 'gp_k', 'gr_raw', 'gr_k'];
  send({ t: 'sub', series });
}
function go(view) {
  S.view = view; document.body.dataset.view = view; subscribe();
  $$('.tab').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.go === view)));
  $$('.view').forEach((v) => { v.hidden = v.id !== `v-${view}`; });
  if (view === 'vivo') scene.resize();
  if (S.last) paintUi(S.last, S.robotOk, S.gloveOk);
  try { localStorage.setItem('carloss.view', view); } catch (e) { /* sin almacenamiento */ }
}
$$('.tab').forEach((b) => b.addEventListener('click', () => go(b.dataset.go)));

// ---------- parada de emergencia ----------
function setEstopUi(on) {
  S.estop = on;
  const b = $('#btn-stop'); b.setAttribute('aria-pressed', String(on)); b.querySelector('.stop-l').textContent = on ? 'Liberar' : 'Parar';
  $('#estop-banner').hidden = !on;
}
function estop(on) {
  if (on === S.estop) return;
  if (on) { stopDrive(); logEv('PARADA DE EMERGENCIA', 'crit'); } else logEv('Parada liberada', 'good');
  S.estopT = performance.now(); setEstopUi(on); send({ t: 'estop', v: on });
  if (scene.resize) scene.resize();
}
$('#btn-stop').addEventListener('click', () => estop(!S.estop));
$('#btn-release').addEventListener('click', () => estop(false));

// ---------- acciones rápidas ----------
$$('.act').forEach((b) => b.addEventListener('click', () => {
  send({ t: 'cmd', v: b.dataset.cmd }); toast(b.dataset.msg); logEv(b.dataset.msg, 'cmd');
  b.classList.add('sent'); setTimeout(() => b.classList.remove('sent'), 900);
  if (b.dataset.cmd === 'R') scene.clearTrail();
}));

// ---------- ganancias ----------
const GAINS = {
  'g-pid': [['kp', 'Kp', 'proporcional', .02, 0, .1, .001], ['ki', 'Ki', 'integral', .4, 0, 2, .01], ['kd', 'Kd', 'derivativa', 0, 0, .01, .0005], ['kff', 'Kff', 'prealimentación', .045, 0, .1, .001]],
  'g-yaw': [['kp', 'Kp', 'sobre el giro', 1.5, 0, 5, .1], ['ki', 'Ki', 'sobre el ángulo', 40, 0, 100, 1], ['kb', 'Kb', 'integral del ángulo', 20, 0, 60, 1]],
  'g-dz': [['dl', 'Izquierdo', 'ciclo mínimo', .052, 0, .3, .001], ['dr', 'Derecho', 'ciclo mínimo', .041, 0, .3, .001], ['trim', 'Trim', 'corrección máx. PID', .15, 0, .5, .01]],
};
const decimals = (step) => Math.max(0, -Math.floor(Math.log10(step)));
for (const [fid, fields] of Object.entries(GAINS)) {
  const form = document.getElementById(fid), box = form.querySelector('.fields');
  let applied = null; try { applied = JSON.parse(localStorage.getItem(`carloss.${fid}`)); } catch (e) { applied = null; }
  for (const [k, name, help, def, mn, mx, st] of fields) {
    const val = applied?.[k] ?? def;
    const row = document.createElement('div'); row.className = 'field'; row.dataset.k = k; row.dataset.applied = val;
    row.innerHTML = `<label for="${fid}-${k}">${name}<small>${help}</small></label>
      <input type="range" min="${mn}" max="${mx}" step="${st}" value="${val}" aria-label="${name}">
      <input type="number" id="${fid}-${k}" min="${mn}" max="${mx}" step="${st}" value="${val.toFixed(decimals(st))}">`;
    const [rng, nb] = row.querySelectorAll('input');
    const dirty = () => row.classList.toggle('dirty', Math.abs(parseFloat(nb.value) - parseFloat(row.dataset.applied)) > st / 2);
    rng.addEventListener('input', () => { nb.value = parseFloat(rng.value).toFixed(decimals(st)); dirty(); });
    nb.addEventListener('input', () => { rng.value = nb.value; dirty(); });
    box.append(row);
  }
  form.querySelector('[data-reset]').addEventListener('click', () => {
    for (const [k, , , def, , , st] of fields) { const row = box.querySelector(`[data-k=${k}]`); const [rng, nb] = row.querySelectorAll('input'); rng.value = def; nb.value = def.toFixed(decimals(st)); row.classList.toggle('dirty', Math.abs(def - row.dataset.applied) > st / 2); }
  });
  form.addEventListener('submit', (e) => {
    e.preventDefault();
    const vals = {}; for (const [k] of fields) vals[k] = parseFloat(box.querySelector(`[data-k=${k}] input[type=number]`).value);
    if (Object.values(vals).some((v) => !isFinite(v))) { toast('Revisa los valores: hay un campo vacío o inválido'); return; }
    const cmd = `${form.dataset.cmd},${Object.values(vals).map((v) => +v.toFixed(5)).join(',')}`;
    if (!S.wsOk) { toast('Sin conexión con el panel: no se envió'); return; }
    send({ t: 'cmd', v: cmd }); toast(`Enviado al robot: ${cmd}`); logEv(`Ganancias ${cmd}`, 'cmd');
    box.querySelectorAll('.field').forEach((row) => { row.dataset.applied = vals[row.dataset.k]; row.classList.remove('dirty'); });
    try { localStorage.setItem(`carloss.${fid}`, JSON.stringify(vals)); } catch (e2) { /* sin almacenamiento */ }
  });
}

// ---------- manejo manual ----------
S.drive = { active: false, x: 0, y: 0, keys: new Set(), timer: 0 };
const pad = $('#pad'), knob = $('#pad-knob');
const lim = () => parseInt($('#d-lim').value, 10) / 100;
$('#d-lim').addEventListener('input', () => setText('d-lim-v', $('#d-lim').value));
function driveVec(x, y) {
  const r = Math.hypot(x, y); if (r > 1) { x /= r; y /= r; }
  S.drive.x = x; S.drive.y = y;
  knob.style.transform = `translate(${x * 85}px, ${y * 85}px)`;
  const v = -y * 0.19 * lim(), w = -x * 3.0 * lim();
  setText('d-v', num(v * 100, 1)); setText('d-w', num(w, 2));
  return { v, w };
}
function startDrive() {
  if (S.estop) { toast('Parada de emergencia activa: libérala para manejar'); return false; }
  if (!S.drive.active) { S.drive.active = true; logEv('Manejo manual desde el panel', 'cmd'); }
  clearInterval(S.drive.timer);
  S.drive.timer = setInterval(() => { const { v, w } = driveVec(S.drive.x, S.drive.y); send({ t: 'drive', v, w }); }, 50);
  return true;
}
function stopDrive() {
  if (!S.drive.active) return;
  S.drive.active = false; clearInterval(S.drive.timer); driveVec(0, 0); send({ t: 'drive', v: 0, w: 0 });
  pad.classList.remove('active');
}
pad.addEventListener('pointerdown', (e) => {
  if (!startDrive()) return;
  pad.setPointerCapture(e.pointerId); pad.classList.add('active');
  const move = (ev) => { const r = pad.getBoundingClientRect(); driveVec((ev.clientX - r.left - r.width / 2) / (r.width / 2 - 20), (ev.clientY - r.top - r.height / 2) / (r.height / 2 - 20)); };
  move(e);
  const up = () => { pad.removeEventListener('pointermove', move); stopDrive(); };
  pad.addEventListener('pointermove', move);
  pad.addEventListener('pointerup', up, { once: true }); pad.addEventListener('pointercancel', up, { once: true });
});
const KEYMAP = { w: [0, -1], arrowup: [0, -1], s: [0, 1], arrowdown: [0, 1], a: [-1, 0], arrowleft: [-1, 0], d: [1, 0], arrowright: [1, 0] };
function keyVec() { let x = 0, y = 0; for (const k of S.drive.keys) { x += KEYMAP[k][0]; y += KEYMAP[k][1]; } return [x, y]; }

// ---------- teclado ----------
addEventListener('keydown', (e) => {
  if (e.target.matches('input, textarea')) return;
  const k = e.key.toLowerCase();
  if (e.code === 'Space') { e.preventDefault(); estop(true); return; }
  if (k === 'g' && !e.ctrlKey && !e.metaKey && S.view === 'vivo') { goalMode(!S.goalMode); return; }
  if (e.key === 'Escape') { if (S.goalSt && S.goalSt.state === 'yendo') cancelGoal(); goalMode(false); return; }
  if (/^[1-5]$/.test(k) && !e.ctrlKey && !e.metaKey) { go(['vivo', 'senales', 'control', 'guante', 'sistema'][+k - 1]); return; }
  if (e.shiftKey && k === 'r') { $('.act[data-cmd=R]').click(); return; }
  if (S.view === 'control' && KEYMAP[k]) {
    e.preventDefault();
    if (!S.drive.keys.has(k)) { S.drive.keys.add(k); if (startDrive()) { pad.classList.add('active'); driveVec(...keyVec()); } }
  }
});
addEventListener('keyup', (e) => {
  const k = e.key.toLowerCase();
  if (S.drive.keys.delete(k)) { if (S.drive.keys.size) driveVec(...keyVec()); else stopDrive(); }
});
addEventListener('blur', () => { S.drive.keys.clear(); stopDrive(); });

// ---------- herramientas de señales ----------
$$('[data-win]').forEach((b) => b.addEventListener('click', () => { S.win = +b.dataset.win; $$('[data-win]').forEach((x) => x.setAttribute('aria-pressed', String(x === b))); }));
$('#chk-fw').addEventListener('change', (e) => { S.showFw = e.target.checked; subscribe(); });
$('#btn-pause').addEventListener('click', () => {
  S.paused = !S.paused; const b = $('#btn-pause');
  b.setAttribute('aria-pressed', String(S.paused)); b.querySelector('span').textContent = S.paused ? 'Reanudar' : 'Pausar';
  b.querySelector('use').setAttribute('href', S.paused ? '#i-play' : '#i-pause');
});
const stamp = () => new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-');
function download(name, blob) { const a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000); }
$('#btn-csv').addEventListener('click', () => {
  const t0 = plotT1 - S.win, rows = ['serie,t_s,valor'];
  for (const p of PLOTS) for (const d of defsFor(p.id)) { const s = windowed(d.name, t0); if (s) s.t.forEach((t, i) => rows.push(`${d.name},${t.toFixed(3)},${s.v[i]}`)); }
  download(`carloss_senales_${stamp()}.csv`, new Blob([rows.join('\n')], { type: 'text/csv' })); toast('CSV de la ventana visible descargado');
});
$('#btn-png').addEventListener('click', () => {
  const cvs = PLOTS.map((p) => p.cv), cw = cvs[0].width, ch = cvs[0].height, k = cw / cvs[0].clientWidth;
  const out = document.createElement('canvas'); out.width = cw * 2; out.height = (ch + 34 * k) * 3;
  const ctx = out.getContext('2d'); ctx.fillStyle = '#111112'; ctx.fillRect(0, 0, out.width, out.height);
  PLOTS.forEach((p, i) => {
    const x = (i % 2) * cw, y = Math.floor(i / 2) * (ch + 34 * k);
    ctx.fillStyle = C.tawara; ctx.font = `600 ${15 * k}px "Barlow Semi Condensed"`; ctx.fillText(`${p.t} (${p.u})`, x + 12 * k, y + 22 * k);
    ctx.drawImage(p.cv, x, y + 34 * k);
  });
  out.toBlob((b) => download(`carloss_senales_${stamp()}.png`, b)); toast('Imagen de las gráficas descargada');
});

// ---------- escena 3D: el dohyō ----------
const scene = (() => {
  const canvas = $('#scene'), stage = $('#stage');
  // En la VM, Firefox suele dibujar sin tarjeta gráfica (llvmpipe): ahí se baja la carga del 3D
  const softGL = (() => { try { const g = document.createElement('canvas').getContext('webgl'); const e = g && g.getExtension('WEBGL_debug_renderer_info');
    return /llvmpipe|swiftshader|software|softpipe/i.test(e ? g.getParameter(e.UNMASKED_RENDERER_WEBGL) : ''); } catch (err) { return false; } })();
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: !softGL, alpha: true, powerPreference: 'low-power' });
  renderer.setPixelRatio(softGL ? 1 : Math.min(devicePixelRatio || 1, 1.5));
  const sc = new THREE.Scene();
  const cam = new THREE.PerspectiveCamera(38, 1, 0.01, 20); cam.up.set(0, 0, 1);
  const ctl = new OrbitControls(cam, canvas); ctl.enableDamping = true;
  // En el celular un dedo desliza entre pantallas; dos dedos giran y acercan la cámara
  const narrow = matchMedia('(max-width: 820px)');
  const touchMode = () => { ctl.touches.ONE = narrow.matches ? null : THREE.TOUCH.ROTATE; canvas.style.touchAction = narrow.matches ? 'pan-y' : 'none'; };
  touchMode(); narrow.addEventListener('change', touchMode); ctl.dampingFactor = .12; ctl.minDistance = .2; ctl.maxDistance = 3; ctl.maxPolarAngle = Math.PI * .49;
  sc.add(new THREE.HemisphereLight(0xfff4e6, 0x1a1613, 1.1));
  const key = new THREE.DirectionalLight(0xffffff, 1.6); key.position.set(-.6, -.9, 1.6); sc.add(key);
  const rim = new THREE.DirectionalLight(0xffe2b8, .5); rim.position.set(.8, .9, .6); sc.add(rim);
  const col = (c) => new THREE.Color(c);

  // base de arcilla, superficie negra, borde tawara y líneas de salida
  const apron = new THREE.Mesh(new THREE.CylinderGeometry(.47, .49, .05, 160), new THREE.MeshStandardMaterial({ color: col('#211b17'), roughness: .95 }));
  apron.rotation.x = Math.PI / 2; apron.position.z = -.031; sc.add(apron);
  const top = new THREE.Mesh(new THREE.CircleGeometry(DOJO_R, 160), new THREE.MeshStandardMaterial({ color: col('#0f0f10'), roughness: .92 }));
  top.position.z = -.0055; sc.add(top);
  const side = new THREE.Mesh(new THREE.CylinderGeometry(DOJO_R, DOJO_R, .025, 160, 1, true), new THREE.MeshStandardMaterial({ color: col('#0b0b0c'), roughness: .9 }));
  side.rotation.x = Math.PI / 2; side.position.z = -.018; sc.add(side);
  const tawara = new THREE.Mesh(new THREE.RingGeometry(INNER_R, DOJO_R, 200), new THREE.MeshStandardMaterial({ color: col(C.tawara), roughness: .7 }));
  tawara.position.z = -.005; sc.add(tawara);
  for (const sx of [-1, 1]) {
    const sh = new THREE.Mesh(new THREE.PlaneGeometry(.01, .1), new THREE.MeshStandardMaterial({ color: col(C.umber), roughness: .9 }));
    sh.position.set(sx * .06, 0, -.0048); sc.add(sh);
  }
  // escala de rumbo sobre la base (cada 10°, más larga cada 90°)
  const scalePts = [];
  for (let a = 0; a < 360; a += 10) {
    const t = a * Math.PI / 180, r1 = DOJO_R + .012, r2 = r1 + (a % 90 === 0 ? .032 : a % 30 === 0 ? .02 : .011);
    scalePts.push(new THREE.Vector3(r1 * Math.cos(t), r1 * Math.sin(t), -.0055), new THREE.Vector3(r2 * Math.cos(t), r2 * Math.sin(t), -.0055));
  }
  sc.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(scalePts), new THREE.LineBasicMaterial({ color: col('#7d7166') })));
  // marcas de rumbo en el borde: real (blanca) y pedida (amarilla)
  const tickGeo = new THREE.BoxGeometry(.075, .02, .02);
  const tickReal = new THREE.Mesh(tickGeo, new THREE.MeshBasicMaterial({ color: col(C.tawara) }));
  const tickCmd = new THREE.Mesh(new THREE.BoxGeometry(.075, .02, .014), new THREE.MeshBasicMaterial({ color: col(C.car) }));
  sc.add(tickReal, tickCmd); tickCmd.visible = false;
  // arco de proximidad al borde
  // El tramo del borde frente al robot se angosta (franja negra por dentro) y toma color pleno
  const ARC_IN = INNER_R + .6 * TAWARA, ARC_SPAN = Math.PI / 3;
  const edgeMat = new THREE.MeshBasicMaterial({ color: col(C.amber), toneMapped: false });
  const edgeArc = new THREE.Group();
  const arcCol = new THREE.Mesh(new THREE.RingGeometry(ARC_IN, DOJO_R, 48, 1, -ARC_SPAN / 2, ARC_SPAN), edgeMat);
  const arcCut = new THREE.Mesh(new THREE.RingGeometry(INNER_R - .0005, ARC_IN, 48, 1, -ARC_SPAN / 2, ARC_SPAN), new THREE.MeshBasicMaterial({ color: col('#0f0f10') }));
  edgeArc.add(arcCol, arcCut); edgeArc.position.z = -.0045; edgeArc.visible = false; sc.add(edgeArc);

  // robot: modelo de SolidWorks
  const robot = new THREE.Group(); sc.add(robot);
  const shadowTex = (() => { const c = document.createElement('canvas'); c.width = c.height = 64; const g = c.getContext('2d'); const r = g.createRadialGradient(32, 32, 4, 32, 32, 32); r.addColorStop(0, 'rgba(0,0,0,.75)'); r.addColorStop(1, 'rgba(0,0,0,0)'); g.fillStyle = r; g.fillRect(0, 0, 64, 64); return new THREE.CanvasTexture(c); })();
  const shadow = new THREE.Mesh(new THREE.PlaneGeometry(.16, .14), new THREE.MeshBasicMaterial({ map: shadowTex, transparent: true, depthWrite: false }));
  shadow.position.set(.03, 0, -.0044); robot.add(shadow);
  const wheelL = new THREE.Group(), wheelR = new THREE.Group();
  wheelL.position.set(0, B_WHEEL / 2, R_WHEEL); wheelR.position.set(0, -B_WHEEL / 2, R_WHEEL); robot.add(wheelL, wheelR);
  const loader = new STLLoader();
  const parts = [
    ['chasis.stl', robot, { color: '#151516', roughness: .78, metalness: .05 }],
    ['soportes.stl', robot, { color: '#cfd0d4', roughness: .5, metalness: .1 }],
    ['motores.stl', robot, { color: '#9aa0a8', roughness: .35, metalness: .75 }],
    ['rueda_izq.stl', wheelL, { color: C.car, roughness: .6, metalness: 0 }],
    ['rueda_der.stl', wheelR, { color: C.car, roughness: .6, metalness: 0 }],
  ];
  for (const [f, parent, m] of parts) {
    loader.load(`meshes/${f}`, (geo) => { geo.computeVertexNormals(); parent.add(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ ...m, color: col(m.color) }))); },
      undefined, () => { if (f === 'chasis.stl') { const box = new THREE.Mesh(new THREE.BoxGeometry(.1, .1, .07), new THREE.MeshStandardMaterial({ color: col('#151516') })); box.position.set(.034, 0, .045); robot.add(box); } });
  }

  // trayectoria: tinta reservada (amarillo)
  const MAXP = 4000, trailPos = new Float32Array(MAXP * 3);
  const trailGeo = new THREE.BufferGeometry(); trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPos, 3)); trailGeo.setDrawRange(0, 0);
  sc.add(new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: col(C.car), transparent: true, opacity: .8 })));
  let trailN = 0, lastPt = null;
  function clearTrail() { trailN = 0; lastPt = null; trailGeo.setDrawRange(0, 0); }
  function addTrail(x, y) {
    if (lastPt && Math.hypot(x - lastPt[0], y - lastPt[1]) < .004) return;
    if (lastPt && Math.hypot(x, y) < .005 && Math.hypot(lastPt[0], lastPt[1]) > .03) clearTrail();   // odometría reiniciada
    if (trailN >= MAXP) { trailPos.copyWithin(0, 3); trailN--; }
    trailPos.set([x, y, -.0042], trailN * 3); trailN++; lastPt = [x, y];
    trailGeo.setDrawRange(0, trailN); trailGeo.attributes.position.needsUpdate = true;
  }

  // estado animado (amortiguado)
  const cur = { x: 0, y: 0, th: 0, thd: null, wl: 0, wr: 0, angL: 0, angR: 0, edge: 0 };
  const tgt = { x: 0, y: 0, th: 0, thd: null, edgeK: 0 };
  let lastT = performance.now();
  function update(d, ok) {
    tgt.x = d.x; tgt.y = d.y; tgt.th = d.th; tgt.thd = d.th_des; cur.wl = d.wl; cur.wr = d.wr;
    if (ok) addTrail(d.x, d.y);
    const dist = INNER_R - Math.hypot(d.x, d.y) - ROBOT_HALF;
    tgt.edgeK = !ok ? 0 : dist <= .06 ? 1 : dist <= .12 ? .25 : 0;
  }

  // cámara
  const PRESETS = {
    // en vertical la cámara va más cenital para que el aro llene la altura
    dojo: () => { const el = (cam.aspect < 1 ? 72 : 48) * Math.PI / 180, az = -1.5; const d = fitDist(el); return { p: new THREE.Vector3(d * Math.cos(el) * Math.cos(az), d * Math.cos(el) * Math.sin(az), d * Math.sin(el)), t: new THREE.Vector3(0, .02, -.02) }; },
    top: () => ({ p: new THREE.Vector3(0, -.001, fitDist(Math.PI / 2)), t: new THREE.Vector3(0, 0, 0) }),
    follow: () => { const c = Math.cos(cur.th), s = Math.sin(cur.th); return { p: new THREE.Vector3(cur.x - .3 * c, cur.y - .3 * s, .2), t: new THREE.Vector3(cur.x + .03 * c, cur.y + .03 * s, .03) }; },
  };
  // Distancia para que la base (r 0,49) ocupe ~92 % de la dimensión que limita, vista con elevación el
  function fitDist(el) {
    const r = .49 / .92, vf = cam.fov * Math.PI / 360, hf = Math.atan(Math.tan(vf) * cam.aspect);
    const dh = r / Math.tan(hf);                                  // ancho del aro
    const dv = (r * Math.sin(el) + .03 * Math.cos(el)) / Math.tan(vf);   // alto proyectado
    return Math.max(dh, dv) * 1.02;
  }
  let camMode = 'dojo', camAnim = null, userMoved = false;
  function setCam(mode, instant) {
    camMode = mode; userMoved = false; const g = PRESETS[mode]();
    $$('.stage-views button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.cam === mode)));
    if (instant || reduceMotion) { cam.position.copy(g.p); ctl.target.copy(g.t); ctl.update(); return; }
    camAnim = { p0: cam.position.clone(), t0: ctl.target.clone(), p1: g.p, t1: g.t, s: performance.now() };
  }
  $$('.stage-views button').forEach((b) => b.addEventListener('click', () => setCam(b.dataset.cam)));
  ctl.addEventListener('start', () => { camAnim = null; userMoved = true; });

  function resize() {
    const w = stage.clientWidth, h = stage.clientHeight; if (!w || !h) return;
    renderer.setSize(w, h, false); cam.aspect = w / h;
    cam.fov = w / h < 1 ? 50 : 36; cam.updateProjectionMatrix();
    if (!userMoved && camMode !== 'follow') setCam(camMode, true);
  }
  new ResizeObserver(resize).observe(stage);

  let lastFrame = 0;
  function frame(now) {
    requestAnimationFrame(frame);
    if (document.hidden || S.view !== 'vivo' || now - lastFrame < 31) return;   // ~30 fps como máximo
    const dt = Math.min(.1, (now - lastT) / 1000); lastT = now; lastFrame = now;
    const k = reduceMotion ? 1 : 1 - Math.exp(-dt / .03);
    cur.x += (tgt.x - cur.x) * k; cur.y += (tgt.y - cur.y) * k; cur.th += wrap(tgt.th - cur.th) * k;
    if (tgt.thd == null) cur.thd = null; else cur.thd = cur.thd == null ? tgt.thd : cur.thd + wrap(tgt.thd - cur.thd) * k;
    cur.edge += (tgt.edgeK - cur.edge) * (reduceMotion ? 1 : 1 - Math.exp(-dt / .15));
    robot.position.set(cur.x, cur.y, 0); robot.rotation.z = cur.th;
    cur.angL += cur.wl * dt; cur.angR += cur.wr * dt; wheelL.rotation.y = cur.angL; wheelR.rotation.y = cur.angR;
    const place = (m, a, z) => { const r = DOJO_R + .055; m.position.set(r * Math.cos(a), r * Math.sin(a), z); m.rotation.z = a; };
    place(tickReal, cur.th, .006);
    tickCmd.visible = cur.thd != null; if (cur.thd != null) place(tickCmd, cur.thd, .012);
    // la firma: el tramo del borde más cercano al robot se enciende ámbar y luego rojo
    edgeArc.rotation.z = Math.atan2(cur.y, cur.x);
    edgeArc.visible = tgt.edgeK > 0;                        // escalonado: ≤12 cm ámbar, ≤6 cm rojo
    edgeMat.color.set(tgt.edgeK >= .5 ? C.red : C.amber);
    goalFrame(dt);
    if (camAnim) {
      const u = Math.min(1, (now - camAnim.s) / 420), e = 1 - Math.pow(1 - u, 4);
      cam.position.lerpVectors(camAnim.p0, camAnim.p1, e); ctl.target.lerpVectors(camAnim.t0, camAnim.t1, e);
      if (u >= 1) camAnim = null;
    } else if (camMode === 'follow') {
      const g = PRESETS.follow(); ctl.target.lerp(g.t, .15); cam.position.lerp(g.p, .08);
    }
    ctl.update(); renderer.render(sc, cam);
  }
  resize(); setCam('dojo', true); requestAnimationFrame(frame);
  // ---- objetivo (ir a punto): marcador amarillo, línea al robot y área segura
  const goalGrp = new THREE.Group(); goalGrp.visible = false; sc.add(goalGrp);
  const yellow = () => new THREE.MeshBasicMaterial({ color: col(C.car), toneMapped: false, side: THREE.DoubleSide });
  const gRing = new THREE.Mesh(new THREE.RingGeometry(.018, .027, 40, 1, 0, Math.PI * 1.6), yellow()); gRing.position.z = -.003; goalGrp.add(gRing);
  const gDot = new THREE.Mesh(new THREE.CircleGeometry(.006, 20), yellow()); gDot.position.z = -.003; goalGrp.add(gDot);
  const gPin = new THREE.Mesh(new THREE.CylinderGeometry(.0022, .0022, .09, 8), yellow()); gPin.rotation.x = Math.PI / 2; gPin.position.z = .042; goalGrp.add(gPin);
  const gLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
    new THREE.LineDashedMaterial({ color: col(C.car), dashSize: .012, gapSize: .009 }));
  gLine.visible = false; sc.add(gLine);
  const SAFE_R = .31;
  const safe = new THREE.Line(new THREE.BufferGeometry().setFromPoints(Array.from({ length: 121 }, (_, i) => { const a = i / 120 * 2 * Math.PI; return new THREE.Vector3(SAFE_R * Math.cos(a), SAFE_R * Math.sin(a), -.0042); })),
    new THREE.LineDashedMaterial({ color: col(C.car), dashSize: .015, gapSize: .012, transparent: true, opacity: .6 }));
  safe.computeLineDistances(); safe.visible = false; sc.add(safe);
  let goalXY = null, goalLive = false;
  function setGoal(g, live) { goalXY = g; goalLive = !!live; goalGrp.visible = !!g; gLine.visible = !!g && goalLive; if (g) goalGrp.position.set(g[0], g[1], 0); }
  function setGoalMode(on) { safe.visible = on; }
  // un clic sin arrastrar sobre el dojo da el punto del plano del piso (z = 0)
  const ray = new THREE.Raycaster(), floor = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), hit = new THREE.Vector3();
  let down = null;
  canvas.addEventListener('pointerdown', (e) => { down = { x: e.clientX, y: e.clientY, t: performance.now() }; });
  canvas.addEventListener('pointerup', (e) => {
    const d0 = down; down = null;
    if (!d0 || !S.goalMode || Math.hypot(e.clientX - d0.x, e.clientY - d0.y) > 6 || performance.now() - d0.t > 600) return;
    const r = canvas.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), cam);
    if (ray.ray.intersectPlane(floor, hit)) onGoalClick(hit.x, hit.y);
  });
  function goalFrame(dt) {
    if (!goalXY) return;
    gRing.rotation.z += dt * (goalLive ? 1.6 : 0);
    if (gLine.visible) {
      const p = gLine.geometry.attributes.position;
      p.setXYZ(0, cur.x, cur.y, .002); p.setXYZ(1, goalXY[0], goalXY[1], .002); p.needsUpdate = true; gLine.computeLineDistances();
    }
  }

  return { update, resize, clearTrail, setGoal, setGoalMode };
})();

// ---------- ir a un punto del dojo ----------
S.goalMode = false; S.goalSt = null; S.lastGoal = null;
const SAFE_R = 0.31;
function goalMode(on) {
  S.goalMode = on; document.body.classList.toggle('goal-mode', on);
  $('#btn-goal').setAttribute('aria-pressed', String(on)); $('#goal-hint').hidden = !on; scene.setGoalMode(on);
}
function onGoalClick(x, y) {
  const r = Math.hypot(x, y);
  if (r > SAFE_R) { toast(`Fuera del área segura: ${num(r * 100, 0)} cm del centro (máximo ${SAFE_R * 100} cm)`); return; }
  if (S.estop) { toast('Parada de emergencia activa: libérala para mover el robot'); return; }
  if (!S.robotOk) { toast('Sin telemetría del robot: no se envió el objetivo'); return; }
  stopDrive();
  send({ t: 'goal', x, y }); S.lastGoal = [x, y]; S.goalClickT = performance.now(); scene.setGoal([x, y], true);
  logEv(`Objetivo: x ${num(x * 100, 1)} cm, y ${num(y * 100, 1)} cm`, 'cmd');
}
function cancelGoal() { send({ t: 'goal_cancel' }); }
$('#btn-goal').addEventListener('click', () => goalMode(!S.goalMode));
$('#btn-goal-cancel').addEventListener('click', cancelGoal);
const GOAL_NAMES = { libre: 'sin objetivo', yendo: 'yendo', 'llegó': 'llegó', cancelado: 'cancelado', rechazado: 'rechazado' };
function paintGoal(g) {
  const st = g ? g.state : 'libre', prev = S.goalSt ? S.goalSt.state : null;
  if (prev && st !== prev) {
    if (st === 'llegó') logEv(`Llegó al objetivo (${g.detail})`, 'good');
    else if (st === 'cancelado') logEv(`Objetivo cancelado: ${g.detail}`, 'warn');
    else if (st === 'rechazado') { logEv(`Objetivo rechazado: ${g.detail}`, 'warn'); toast(`Objetivo rechazado: ${g.detail}`); }
  }
  S.goalSt = g;
  if (g && g.goal) S.lastGoal = g.goal;
  $('#mod-goal').dataset.state = st;
  setText('g-state', GOAL_NAMES[st] || st);
  setText('g-dist', g && g.dist != null ? num(g.dist * 100, 1) : '—');
  setText('g-xy', S.lastGoal ? `${num(S.lastGoal[0] * 100, 0)} , ${num(S.lastGoal[1] * 100, 0)} cm` : '—');
  setText('g-detail', g && g.detail ? g.detail
    : (st === 'yendo' ? 'Corrigiendo con la odometría (encoders + giroscopio)' : 'Pon el robot en el centro mirando a +x, pulsa Reiniciar odometría y usa Ir a punto.'));
  $('#btn-goal-cancel').disabled = st !== 'yendo';
  if (st === 'yendo' && g.goal) scene.setGoal(g.goal, true);
  else if (st === 'llegó' && S.lastGoal) scene.setGoal(S.lastGoal, false);
  else if (performance.now() - (S.goalClickT || 0) > 1500) scene.setGoal(null);   // espera la confirmación del clic
}

// ---------- bucle de gráficas (solo en la vista visible) ----------
setInterval(() => {
  if (document.hidden) return;
  if (S.view === 'senales') drawPlots();
  else if (S.view === 'guante') drawGlovePlot();
}, 66);

// ---------- arranque ----------
let saved = 'vivo'; try { saved = localStorage.getItem('carloss.view') || 'vivo'; } catch (e) { saved = 'vivo'; }
// Accesos directos por URL: ?vista=senales · ?camara=top · ?ir=1 (modo Ir a punto)
const QS = new URLSearchParams(location.search);
if (QS.get('vista')) saved = QS.get('vista');
go(['vivo', 'senales', 'control', 'guante', 'sistema'].includes(saved) ? saved : 'vivo');
if (['dojo', 'top', 'follow'].includes(QS.get('camara'))) $(`[data-cam=${QS.get('camara')}]`).click();
if (QS.get('ir') === '1') goalMode(true);
logEl.innerHTML = '<li><time>—</time><span class="log-empty">Sin eventos todavía</span></li>';
connect();
