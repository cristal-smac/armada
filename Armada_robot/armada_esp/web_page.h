#pragma once
#include <Arduino.h>

// Page de commande (generee) servie par l'ESP32  : joystick + distances du lidar autour.
// La page appelle /cmd toutes les 100 ms (consigne + réception de l'état) :
// si ces appels s'arrêtent, le robot s'arrête (voir Robot::set_command_timeout).
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ARMADA</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 12px; min-height: 100vh;
    background: #0e1514; color: #e5eeec;
    font-family: system-ui, "Segoe UI", sans-serif;
    display: flex; flex-direction: column; align-items: center; gap: 12px;
    user-select: none; -webkit-user-select: none; touch-action: none;
  }
  h1 { margin: 0; font-size: 18px; letter-spacing: 0.1em; }
  #status { padding: 4px 14px; border-radius: 999px; font-size: 13px; background: #7a2b25; }
  #status.ok { background: #1f5a34; }
  #floor { padding: 6px 18px; border-radius: 8px; font-size: 15px; font-weight: 600; font-variant-numeric: tabular-nums; background: #263634; color: #94a6a2; }
  #floor.ok { background: #1f5a34; color: #e5eeec; }
  #floor.bad { background: #b02a21; color: #fff; }
  #pad { width: min(94vw, 440px); height: auto; aspect-ratio: 1; touch-action: none; cursor: grab; }
  .row { display: flex; flex-wrap: wrap; gap: 12px 20px; align-items: center; justify-content: center; font-size: 14px; }
  #info { font-variant-numeric: tabular-nums; color: #94a6a2; }
  button {
    font: inherit; font-weight: 600; padding: 10px 28px; border: 0; border-radius: 6px;
    background: #b02a21; color: #fff; cursor: pointer;
  }
  input[type=range] { width: 160px; }
  .hint { font-size: 12px; color: #94a6a2; text-align: center; max-width: 420px; }
</style>
</head>
<body>
  <h1>ARMADA</h1>
  <div id="status">Déconnecté</div>
  <div id="floor">Sol : --</div>
  <canvas id="pad"></canvas>
  <div id="info">vx 0.00 m/s · vw 0.00 rad/s</div>
  <div class="row">
    <label>Vitesse max <span id="spv">50</span> %
      <input id="sp" type="range" min="10" max="100" value="50"></label>
    <button id="stop" type="button">STOP</button>
  </div>
  <div class="hint">Joystick tactile ou souris, flèches ou ZQSD au clavier. Cercles : distance mesurée (en cm) par chaque capteur, plus proche du joystick = plus proche du robot. Si la connexion est perdue, le robot s'arrête.</div>

<script>
(function () {
  var SZ = 440, C = SZ / 2, R_BASE = 80, R_KNOB = 28, R_IN = 108, R_OUT = 178, MAXR = 1.2;
  // Angle de chaque capteur (degrés, 90 = avant = haut de l'écran, sens anti-horaire)
  var ANG = { F: 90, FL: 135, L: 180, BL: 225, B: 270, BR: 315, R: 0, FR: 45 };

  var cv = document.getElementById('pad'), ctx = cv.getContext('2d');
  var dpr = window.devicePixelRatio || 1;
  cv.width = SZ * dpr; cv.height = SZ * dpr; ctx.scale(dpr, dpr);

  var stick = { x: 0, y: 0 }, dragging = false, keys = {}, lidar = [], connected = false, busy = false;
  var elStatus = document.getElementById('status'), elInfo = document.getElementById('info');
  var elSp = document.getElementById('sp'), elSpv = document.getElementById('spv');

  // Capteur ultrason vers le sol : distance nominale mesurée à la règle (cm) et tolérance
  var FLOOR_NOM = 5.0, FLOOR_TOL = 2.0;
  var elFloor = document.getElementById('floor');
  function showFloor(d) {
    if (d === null || d === undefined) {
      elFloor.textContent = 'Sol : -- (pas d\'écho)'; elFloor.className = 'bad'; return;
    }
    var cm = d * 100;
    elFloor.textContent = 'Sol : ' + cm.toFixed(1) + ' cm';
    elFloor.className = Math.abs(cm - FLOOR_NOM) <= FLOOR_TOL ? 'ok' : 'bad';
  }

  function scale() { return elSp.value / 100; }
  elSp.addEventListener('input', function () { elSpv.textContent = elSp.value; });

  function setStatus(ok) {
    connected = ok;
    elStatus.textContent = ok ? 'Connecté' : 'Déconnecté (le robot s\'arrête)';
    elStatus.className = ok ? 'ok' : '';
  }

  // ─── Joystick ────────────────────────────────────────────────────────────
  function pos(e) {
    var r = cv.getBoundingClientRect();
    return { x: (e.clientX - r.left) * SZ / r.width - C, y: (e.clientY - r.top) * SZ / r.height - C };
  }
  function move(p) {
    var d = Math.hypot(p.x, p.y), k = d > R_BASE ? R_BASE / d : 1;
    stick.x = p.x * k / R_BASE; stick.y = -p.y * k / R_BASE; draw();
  }
  function release() { dragging = false; stick.x = 0; stick.y = 0; keysToStick(); draw(); }
  cv.addEventListener('pointerdown', function (e) {
    var p = pos(e);
    if (Math.hypot(p.x, p.y) > R_BASE + 30) return;
    dragging = true; cv.setPointerCapture(e.pointerId); move(p);
  });
  cv.addEventListener('pointermove', function (e) { if (dragging) move(pos(e)); });
  cv.addEventListener('pointerup', release);
  cv.addEventListener('pointercancel', release);

  // ─── Clavier ─────────────────────────────────────────────────────────────
  var UP = ['arrowup', 'z', 'w'], DOWN = ['arrowdown', 's'], LEFT = ['arrowleft', 'q', 'a'], RIGHT = ['arrowright', 'd'];
  function any(list) { return list.some(function (k) { return keys[k]; }) ? 1 : 0; }
  function keysToStick() {
    if (dragging) return;
    stick.x = any(RIGHT) - any(LEFT);
    stick.y = any(UP) - any(DOWN);
    var n = Math.hypot(stick.x, stick.y);
    if (n > 1) { stick.x /= n; stick.y /= n; }
  }
  window.addEventListener('keydown', function (e) { keys[e.key.toLowerCase()] = true; keysToStick(); draw(); });
  window.addEventListener('keyup', function (e) { keys[e.key.toLowerCase()] = false; keysToStick(); draw(); });
  // Page cachée ou fenêtre inactive : on lâche tout
  function dropAll() { keys = {}; release(); }
  document.addEventListener('visibilitychange', function () { if (document.hidden) dropAll(); });
  window.addEventListener('blur', dropAll);

  // ─── STOP ────────────────────────────────────────────────────────────────
  document.getElementById('stop').addEventListener('click', function () {
    keys = {}; release();
    fetch('/stop', { cache: 'no-store' }).catch(function () {});
  });

  // ─── Échanges avec le robot (toutes les 100 ms) ──────────────────────────
  function tick() {
    if (busy) return;
    busy = true;
    var ctl = new AbortController(), to = setTimeout(function () { ctl.abort(); }, 800);
    var s = scale();
    var url = '/cmd?x=' + (stick.x * s).toFixed(2) + '&y=' + (stick.y * s).toFixed(2);
    fetch(url, { cache: 'no-store', signal: ctl.signal })
      .then(function (r) { return r.json(); })
      .then(function (j) {
        lidar = j.lidar || [];
        showFloor(j.floor);
        elInfo.textContent = 'vx ' + j.vx.toFixed(2) + ' m/s · vw ' + j.vw.toFixed(2) + ' rad/s';
        setStatus(true); draw();
      })
      .catch(function () { setStatus(false); elFloor.textContent = 'Sol : --'; elFloor.className = ''; })
      .then(function () { clearTimeout(to); busy = false; });
  }
  setInterval(tick, 100);

  // ─── Dessin ──────────────────────────────────────────────────────────────
  function rOf(d) { return R_IN + Math.min(d, MAXR) / MAXR * (R_OUT - R_IN); }
  function draw() {
    ctx.clearRect(0, 0, SZ, SZ);
    ctx.lineWidth = 1;
    // Cercles de distance : 0,4 m, 0,8 m, 1,2 m
    ctx.strokeStyle = '#263634';
    [0.4, 0.8, 1.2].forEach(function (d) { ctx.beginPath(); ctx.arc(C, C, rOf(d), 0, 6.2832); ctx.stroke(); });
    // Noms des capteurs sur le bord
    ctx.fillStyle = '#5d716d'; ctx.font = '12px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    Object.keys(ANG).forEach(function (n) {
      var a = ANG[n] * Math.PI / 180;
      ctx.fillText(n, C + (R_OUT + 24) * Math.cos(a), C - (R_OUT + 24) * Math.sin(a));
    });
    // Marqueurs de distance
    lidar.forEach(function (s) {
      var a = (ANG[s.n] || 0) * Math.PI / 180;
      if (s.d === null) {
        var gx = C + R_OUT * Math.cos(a), gy = C - R_OUT * Math.sin(a);
        ctx.strokeStyle = '#5d716d'; ctx.beginPath(); ctx.arc(gx, gy, 9, 0, 6.2832); ctx.stroke();
        ctx.fillStyle = '#5d716d'; ctx.font = '11px system-ui'; ctx.fillText('--', gx, gy);
        return;
      }
      var r = rOf(s.d), x = C + r * Math.cos(a), y = C - r * Math.sin(a);
      ctx.fillStyle = s.d < 0.15 ? '#f58b82' : (s.d < 0.4 ? '#f0b45f' : '#6fcf8a');
      ctx.beginPath(); ctx.arc(x, y, 15, 0, 6.2832); ctx.fill();
      ctx.fillStyle = '#0e1514'; ctx.font = 'bold 11px system-ui';
      ctx.fillText(String(Math.round(s.d * 100)), x, y);
    });
    // Joystick
    ctx.strokeStyle = connected ? '#52c6bf' : '#7a2b25'; ctx.lineWidth = 2;
    ctx.fillStyle = '#142120';
    ctx.beginPath(); ctx.arc(C, C, R_BASE, 0, 6.2832); ctx.fill(); ctx.stroke();
    ctx.fillStyle = dragging || stick.x || stick.y ? '#52c6bf' : '#3a4f4c';
    ctx.beginPath(); ctx.arc(C + stick.x * R_BASE, C - stick.y * R_BASE, R_KNOB, 0, 6.2832); ctx.fill();
  }
  draw();
})();
</script>
</body>
</html>
)HTML";
