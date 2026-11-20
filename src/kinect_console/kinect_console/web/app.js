// Kinect v2 Console — browser side. No build step, no dependencies.
// Video: <img> on an MJPEG stream (/stream?src=...), only while CAMERAS shows.
// Audio + status: one WebSocket; audio flows only while MIC ARRAY shows.

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];

const STREAMS = {
  color: { topic: '/kinect/rgb/compressed', res: '960 × 540 (rgb_scale 0.5)', enc: 'JPEG from the driver',
    frame: 'kinect_rgb_optical_frame', depth: false,
    about: 'Full-HD color camera, scaled by rgb_scale and JPEG-encoded by the driver.' },
  ir: { topic: '/kinect/ir', res: '512 × 424', enc: 'mono16 → sqrt auto-gain', frame: 'kinect_ir_optical_frame',
    depth: false, about: 'Active infrared: sees in the dark. Same sensor and pixels as depth.' },
  depth: { topic: '/kinect/depth', res: '512 × 424', enc: 'mono16 mm → turbo', frame: 'kinect_depth_optical_frame',
    depth: true, about: 'Time-of-flight depth in millimetres, 0.5–4.5 m. The driver publishes it at half rate.' },
  registered: { topic: '/kinect/rgb/registered', res: '512 × 424', enc: 'bgr8', frame: 'kinect_depth_optical_frame',
    depth: false, about: 'Color resampled onto the depth grid: one color per depth pixel (black = not seen by color).' },
  aligned: { topic: '/kinect/depth/color_aligned', res: '960 × 540', enc: 'mono16 mm → turbo', frame: 'kinect_rgb_optical_frame',
    depth: true, about: 'Depth re-projected onto the color image, pixel for pixel, so a color detection can read its distance.' },
};

let screen = 'cameras';
let source = 'color';
let lastStatus = null;
let lastFrameAt = 0;

// ---------------- screens ----------------
function route() {
  screen = location.hash === '#mics' ? 'mics' : 'cameras';
  $$('.screens a').forEach((a) => {
    if (a.dataset.screen === screen) a.setAttribute('aria-current', 'page');
    else a.removeAttribute('aria-current');
  });
  $('#screen-cameras').hidden = screen !== 'cameras';
  $('#screen-mics').hidden = screen !== 'mics';
  syncVideo();
  syncAudio();
}
window.addEventListener('hashchange', route);
document.addEventListener('visibilitychange', () => { syncVideo(); syncAudio(); });

// ---------------- cameras ----------------
const img = $('#live');
let streaming = null;

function syncVideo() {
  const want = screen === 'cameras' && !document.hidden ? source : null;
  if (want === streaming) return;
  streaming = want;
  img.removeAttribute('src');          // closes the old stream -> server releases it
  if (want) {
    $('#placeholder').hidden = false;
    $('#placeholder').textContent = 'WAITING FOR FRAMES…';
    img.src = `/stream?src=${want}&t=${Date.now()}`;
  }
}
img.addEventListener('load', () => { lastFrameAt = Date.now(); $('#placeholder').hidden = true; });

function selectSource(src) {
  source = src;
  $$('[data-src]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.src === src)));
  const s = STREAMS[src];
  $('#topic-badge').textContent = s.topic;
  $('#s-res').textContent = s.res;
  $('#s-enc').textContent = s.enc;
  $('#s-frame').textContent = s.frame;
  $('#s-about').textContent = s.about;
  $('#legend').hidden = !s.depth;
  $('#click-hint').textContent = s.depth ? 'Click the image to read the distance at that point.'
    : 'Switch to DEPTH or ALIGNED DEPTH to read distances by clicking.';
  $('#viewport .marker')?.remove();
  syncVideo();
  renderStatus();
}
$$('[data-src]').forEach((b) => b.addEventListener('click', () => selectSource(b.dataset.src)));

// Click -> distance. The image is object-fit: contain, so map through the letterbox.
img.addEventListener('click', async (ev) => {
  if (!STREAMS[source].depth || !img.naturalWidth) return;
  const r = img.getBoundingClientRect();
  const scale = Math.min(r.width / img.naturalWidth, r.height / img.naturalHeight);
  const w = img.naturalWidth * scale, h = img.naturalHeight * scale;
  const ox = (r.width - w) / 2, oy = (r.height - h) / 2;
  const x = (ev.clientX - r.left - ox) / w, y = (ev.clientY - r.top - oy) / h;
  if (x < 0 || x > 1 || y < 0 || y > 1) return;
  let m = $('#viewport .marker');
  if (!m) { m = document.createElement('span'); m.className = 'marker'; $('#viewport').appendChild(m); }
  m.style.left = `${ev.clientX - r.left}px`; m.style.top = `${ev.clientY - r.top}px`;
  try {
    const res = await fetch(`/api/depth_at?src=${source}&x=${x.toFixed(4)}&y=${y.toFixed(4)}`);
    const d = await res.json();
    $('#dist-value').textContent = d.mm == null ? 'no reading' : `${(d.mm / 1000).toFixed(2)} m`;
    $('#dist-where').textContent = `at (${Math.round(x * img.naturalWidth)}, ${Math.round(y * img.naturalHeight)}) in ${source}`;
  } catch { $('#dist-value').textContent = '—'; }
});

// ---------------- WebSocket: status + audio ----------------
let ws = null;
let audioOn = false;
const handlers = { status: onStatus, audio: onAudio, doa_method: onMethod };

function connect() {
  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => { audioOn = false; syncAudio(); };
  ws.onmessage = (e) => { const m = JSON.parse(e.data); handlers[m.type]?.(m); };
  ws.onclose = () => { setUsb('bad', 'CONSOLE DISCONNECTED'); setTimeout(connect, 2000); };
}
function send(m) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(m)); }

function setUsb(cls, text) {
  const p = $('#usb-pill');
  p.className = `pill ${cls}`;
  $('#usb-text').textContent = text;
}

function onStatus(s) { lastStatus = s; renderStatus(); }

function renderStatus() {
  const s = lastStatus;
  if (!s) return;
  const u = s.usb || {};
  if (!u.present) setUsb('bad', 'KINECT NOT ON USB');
  else if (!u.superspeed) setUsb('warn', `KINECT AT ${u.speed} — NEEDS USB 3`);
  else setUsb('ok', `KINECT ON USB · ${u.speed}`);
  $('#serial-pill').textContent = `SERIAL ${u.serial || '—'}`;
  $('#clients-pill').textContent = `${s.clients} ${s.clients === 1 ? 'VIEWER' : 'VIEWERS'}`;
  const st = s.streams?.[source] || { fps: 0 };
  $('#fps-badge').textContent = `${st.fps.toFixed(1)} FPS`;
  $('#s-fps').textContent = `${st.fps.toFixed(1)} fps here`;
  // An MJPEG <img> fires 'load' only once in Chrome, so judge liveness by the
  // server's measured rate for this stream (plus the first load, for speed).
  const live = st.fps > 0 || Date.now() - lastFrameAt < 3000;
  const b = $('#live-badge');
  b.textContent = live ? '● LIVE' : (u.present ? '● WAITING FOR DRIVER' : '● NO KINECT');
  b.classList.toggle('on', live);
  $('#viewer-list').innerHTML = Object.entries(s.streams || {})
    .map(([k, v]) => `<dt>${k}</dt><dd>${v.viewers} · ${v.fps.toFixed(1)} fps</dd>`).join('');
}
setInterval(renderStatus, 1000);

// ---------------- mic array ----------------
let latest = null, latestAt = 0, shownDoa = 0, doaGlow = 0;
let peaks = [0, 0, 0, 0];
const history = [];       // [{t, doa}]
let raf = null;

function syncAudio() {
  const want = screen === 'mics' && !document.hidden;
  if (want !== audioOn && ws && ws.readyState === 1) { audioOn = want; send({ type: 'audio', on: want }); }
  if (want && !raf) raf = requestAnimationFrame(draw);
}

// Left/right of the fan. The published angle's sign depends on the estimator
// (srp and gcc disagree) and "left" depends on where you stand (behind the
// Kinect, your left is its left; facing it, they are swapped). Clap on one
// side once; if the needle goes the other way, FLIP L/R. Remembered per browser.
let flip = false;
try { flip = localStorage.getItem('kinect.doaFlip') === '1'; } catch { /* private mode */ }
const flipBtn = $('#flip-btn');
function renderFlip() { flipBtn.setAttribute('aria-pressed', String(flip)); }
flipBtn.addEventListener('click', () => {
  flip = !flip;
  try { localStorage.setItem('kinect.doaFlip', flip ? '1' : '0'); } catch { /* ignore */ }
  shownDoa = -shownDoa;
  history.forEach((h) => { if (h.doa != null) h.doa = -h.doa; });
  renderFlip();
});
renderFlip();

function onAudio(m) {
  if (typeof m.doa === 'number' && flip) m.doa = -m.doa;
  latest = m; latestAt = Date.now();
  if (typeof m.doa === 'number') { doaGlow = 1; }
  history.push({ t: Date.now(), doa: typeof m.doa === 'number' ? m.doa : null });
  while (history.length && Date.now() - history[0].t > 10000) history.shift();
}

function onMethod(m) {
  $$('[data-method]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.method === m.method)));
  $('#method-about').textContent = !m.ok ? 'Could not reach audio_publisher to change the method.'
    : m.method === 'srp' ? 'All 6 mic pairs, 300–5000 Hz, 2° grid. Steady: the default.'
      : 'Outer pair only (CH0–CH3). Lighter, noisier.';
}
$$('[data-method]').forEach((b) => b.addEventListener('click', () => send({ type: 'doa_method', method: b.dataset.method })));

const meters = $('#meters');
meters.innerHTML = [0, 1, 2, 3].map((i) =>
  `<div class="meter"><div class="bar"><div class="fill"></div><div class="peak"></div></div><span>CH${i}</span><span class="db">— dB</span></div>`).join('');
const historyEl = $('#history');
historyEl.innerHTML = '<i></i>'.repeat(40);

const toFrac = (rms) => (rms > 0 ? Math.max(0, Math.min(1, (20 * Math.log10(rms) + 60) / 50)) : 0);

function draw() {
  raf = screen === 'mics' ? requestAnimationFrame(draw) : null;
  if (!raf) return;
  const fresh = Date.now() - latestAt < 2500;
  $('#mic-status').textContent = !fresh ? 'no audio — is audio_publisher running?'
    : (doaGlow > 0.05 ? 'listening · sound located' : 'listening');

  const levels = (fresh && latest?.levels) || [0, 0, 0, 0];
  $$('.meter').forEach((m, i) => {
    const f = toFrac(levels[i] || 0);
    peaks[i] = Math.max(f, peaks[i] - 0.008);
    m.querySelector('.fill').style.height = `${Math.round(f * 100)}%`;
    m.querySelector('.peak').style.bottom = `${Math.round(peaks[i] * 100)}%`;
    m.querySelector('.db').textContent = levels[i] > 0 ? `${Math.round(20 * Math.log10(levels[i]))} dB` : '— dB';
  });

  if (fresh && typeof latest?.doa === 'number') shownDoa += (latest.doa - shownDoa) * 0.25;
  doaGlow = Math.max(0, doaGlow - 0.015);
  if (doaGlow > 0.02) {
    $('#doa-value').textContent = `${shownDoa >= 0 ? '+' : '−'}${Math.abs(Math.round(shownDoa))}°`;
    $('#doa-side').textContent = Math.abs(shownDoa) < 5 ? 'straight ahead'
      : shownDoa > 0 ? 'to the right' : 'to the left';
  } else { $('#doa-side').textContent = fresh ? 'quiet — speak or clap' : 'waiting for sound'; }

  // history bars: 40 slots over 10 s
  const bars = historyEl.children, now = Date.now();
  for (let i = 0; i < bars.length; i++) {
    const t0 = now - 10000 + i * 250;
    const hits = history.filter((h) => h.doa != null && h.t >= t0 && h.t < t0 + 250);
    const a = hits.length ? hits[hits.length - 1].doa : null;
    bars[i].style.height = a == null ? '4px' : `${Math.round(8 + Math.abs(a) / 90 * 56)}px`;
    bars[i].style.background = a == null ? '' : (a >= 0 ? 'var(--cyan)' : 'var(--amber)');
  }
  drawFan();
}

function drawFan() {
  const c = $('#fan');
  const dpr = window.devicePixelRatio || 1;
  const w = c.clientWidth, h = c.clientHeight;
  if (!w || !h) return;
  if (c.width !== w * dpr) { c.width = w * dpr; c.height = h * dpr; }
  const ctx = c.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const cx = w / 2, cy = h - 28, R = Math.min(h - 56, w / 2 - 40);
  ctx.strokeStyle = 'rgba(79,216,235,.2)'; ctx.lineWidth = 1;
  for (const r of [R, R * 0.66, R * 0.33]) { ctx.beginPath(); ctx.arc(cx, cy, r, Math.PI, 2 * Math.PI); ctx.stroke(); }
  ctx.fillStyle = '#5c7590'; ctx.font = '11px "IBM Plex Mono", monospace'; ctx.textAlign = 'center';
  for (let a = -90; a <= 90; a += 30) {
    const rad = (a - 90) * Math.PI / 180;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(rad) * R, cy + Math.sin(rad) * R); ctx.stroke();
    const label = a === 0 ? 'AHEAD' : `${Math.abs(a)}° ${a < 0 ? 'L' : 'R'}`;
    ctx.fillText(label, cx + Math.cos(rad) * (R + 18), cy + Math.sin(rad) * (R + 18) + 4);
  }
  // the sensor bar with its four mics (Microsoft layout, metres along the bar)
  const micX = [-0.113, 0.036, 0.076, 0.113], barW = Math.min(140, R * 0.6);
  ctx.fillStyle = '#10161e'; ctx.strokeStyle = '#29455c';
  ctx.beginPath(); ctx.roundRect(cx - barW / 2 - 8, cy - 7, barW + 16, 14, 4); ctx.fill(); ctx.stroke();
  ctx.fillStyle = '#7ee7f4';
  const side = flip ? -1 : 1;   // keep the drawn mic layout consistent with the flip
  micX.forEach((x) => { ctx.beginPath(); ctx.arc(cx + side * x / 0.113 * barW / 2, cy, 3, 0, 2 * Math.PI); ctx.fill(); });
  if (doaGlow > 0.02) {
    const rad = (shownDoa - 90) * Math.PI / 180;
    ctx.fillStyle = `rgba(79,216,235,${0.12 * doaGlow})`;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.arc(cx, cy, R * 0.97, rad - 0.12, rad + 0.12); ctx.closePath(); ctx.fill();
    ctx.strokeStyle = `rgba(79,216,235,${doaGlow})`; ctx.lineWidth = 3; ctx.lineCap = 'round';
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + Math.cos(rad) * R * 0.95, cy + Math.sin(rad) * R * 0.95); ctx.stroke();
    ctx.fillStyle = `rgba(79,216,235,${doaGlow})`;
    ctx.beginPath(); ctx.arc(cx + Math.cos(rad) * R * 0.95, cy + Math.sin(rad) * R * 0.95, 6, 0, 2 * Math.PI); ctx.fill();
  }
}

selectSource('color');
connect();
route();
