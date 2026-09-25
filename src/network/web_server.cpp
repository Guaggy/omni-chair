#include "web_server.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "configs/config.h"
#include "controllers/controller.h"
#include "safety/safety.h"
#include "sensors/collision_sensors.h"
#include "sensors/lidar_sensor.h"
#include "log.h"
#include "recorder.h"
#include "stats.h"
#include "motors/motor_control.h"
#include "settings/settings.h"

bool Web_UI_Enabled = Web_UI_Enabled_At_Start;

static WebServer Server(80);
static bool AP_Running = false;

// The dashboard page, it asks /api/status for new data every 300 ms
static const char Dashboard_Page[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>OmniChair</title>
<style>
  body { background:#111; color:#eee; font-family:sans-serif; margin:0; padding:10px; }
  h1 { font-size:1.3em; margin:0 0 10px; }
  .grid { display:flex; flex-wrap:wrap; gap:14px; }
  .card { background:#1c1c1c; border:1px solid #333; border-radius:6px; padding:10px; }
  canvas { background:#000; border-radius:4px; }
  table { border-collapse:collapse; }
  td { padding:2px 10px 2px 0; font-size:0.9em; }
  td.label { color:#aaa; }
  .ok { color:#4caf50; } .warn { color:#ff9800; } .bad { color:#f44336; }
  #log { width:100%; height:140px; background:#000; color:#0f0; font-family:monospace;
    font-size:0.8em; border:1px solid #333; resize:vertical; }
  button { background:#2a2a2a; color:#eee; border:1px solid #444; border-radius:4px;
    padding:6px 10px; margin:2px; cursor:pointer; }
  button.on { background:#2e7d32; border-color:#4caf50; }
</style>
</head>
<body>
<h1>OmniChair</h1>
<div class="grid">
  <div class="card">
    <canvas id="lidar" width="320" height="180"></canvas>
    <table id="lidarReadout"></table>
  </div>
  <div class="card"><canvas id="psd" width="220" height="160"></canvas></div>
  <div class="card"><canvas id="joystick" width="160" height="160"></canvas></div>
  <div class="card">
    <div id="settingButtons"></div>
    <table id="config"></table>
  </div>
  <div class="card"><div>Stats</div><table id="stats"></table></div>
  <div class="card">
    <div>Recording</div>
    <div id="recordStatus" style="margin:6px 0;"></div>
    <button onclick="record('start')">Start</button>
    <button onclick="record('stop')">Stop</button>
    <a href="/api/record.csv"><button>Download CSV</button></a>
    <div style="font-size:0.8em; color:#aaa; margin-top:6px;">Downloading stops the motors, center the joystick to drive again</div>
  </div>
</div>
<div class="card" style="margin-top:14px;">
  <div>Debug output to log</div>
  <div id="debugButtons"></div>
  <div style="margin-top:8px;">Log</div>
  <textarea id="log" readonly></textarea>
</div>
<script>
const lidarCanvas = document.getElementById('lidar').getContext('2d');
const psdCanvas = document.getElementById('psd').getContext('2d');
const joyCanvas = document.getElementById('joystick').getContext('2d');

// Uses the same distances as config.h, sent along in /api/status
function zoneColor(d, t) {
  if (d <= 0) return '#555';
  if (d <= t.crawl) return '#f44336';
  if (d <= t.slow) return '#ff9800';
  return '#4caf50';
}

function drawLidar(l) {
  const ctx = lidarCanvas, cx = 160, cy = 170, r = 150;
  const half = l.visible_angle / 2;
  const pt = (deg, rad) => [cx + Math.sin(deg * Math.PI / 180) * rad, cy - Math.cos(deg * Math.PI / 180) * rad];
  ctx.clearRect(0, 0, 320, 180);
  ctx.strokeStyle = '#444';
  for (const f of [1/3, 2/3, 1]) {
    ctx.beginPath();
    for (let a = -half; a <= half; a += 4) { const p = pt(a, r * f); a === -half ? ctx.moveTo(p[0], p[1]) : ctx.lineTo(p[0], p[1]); }
    const e = pt(half, r * f); ctx.lineTo(e[0], e[1]);
    ctx.stroke();
  }
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(...pt(0, r)); ctx.stroke();
  ctx.strokeStyle = '#ffeb3b';
  for (const e of [-half, half]) { ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(...pt(e, r)); ctx.stroke(); }
  ctx.fillStyle = '#fff'; ctx.font = '13px sans-serif';
  ctx.fillText('F', cx - 4, 12);
  for (const p of l.points) {
    const [x, y] = pt(p.angle, Math.min(p.distance / 200, 1) * r);
    ctx.fillStyle = zoneColor(p.distance, l);
    ctx.beginPath(); ctx.arc(x, y, 4, 0, 7); ctx.fill();
  }
  ctx.fillStyle = '#fff';
  ctx.beginPath(); ctx.moveTo(cx, cy - 10); ctx.lineTo(cx - 8, cy + 8); ctx.lineTo(cx + 8, cy + 8); ctx.fill();

  const rows = [['Front-Left', l.front_left], ['Front-Right', l.front_right], ['Closest', l.closest]];
  document.getElementById('lidarReadout').innerHTML = rows.map(r =>
    `<tr><td class="label">${r[0]}</td><td style="color:${zoneColor(r[1], l)}">${r[1] > 0 ? r[1] + 'cm' : '--'}</td></tr>`
  ).join('');
}

function drawPsd(psd) {
  const ctx = psdCanvas;
  ctx.clearRect(0, 0, 220, 160);
  const names = ['FL', 'FR', 'SL', 'SR', 'B'];
  const values = [psd.front_left, psd.front_right, psd.side_left, psd.side_right, psd.back];
  values.forEach((v, i) => {
    const x = 10 + i * 42, h = Math.min(v, 80) / 80 * 110;
    ctx.strokeStyle = '#666'; ctx.strokeRect(x, 130 - 110, 30, 110);
    ctx.fillStyle = zoneColor(v, psd);
    ctx.fillRect(x + 1, 130 - h, 28, h);
    ctx.fillStyle = '#fff'; ctx.font = '11px sans-serif';
    ctx.fillText(names[i], x + 6, 14);
    ctx.fillText(v, x + 4, 145);
  });
}

function drawJoystick(j) {
  const ctx = joyCanvas, cx = 80, cy = 80, r = 60;
  ctx.clearRect(0, 0, 160, 160);
  ctx.strokeStyle = '#666'; ctx.beginPath(); ctx.arc(cx, cy, r, 0, 7); ctx.stroke();
  ctx.fillStyle = j.valid ? '#f44336' : '#555';
  ctx.beginPath(); ctx.arc(cx + j.x * (r - 10), cy - j.y * (r - 10), 6, 0, 7); ctx.fill();
  ctx.fillStyle = '#fff'; ctx.font = '11px sans-serif';
  ctx.fillText('speed ' + Math.round(j.speed * 100) + '%', 10, 150);
}

const settingNames = [['collision', 'Collision'], ['hardstop', 'Hardstop'], ['square', 'Square input'],
  ['ignore_front_psd', 'Ignore front PSD']];

function drawConfig(c, wifi) {
  document.getElementById('settingButtons').innerHTML = settingNames.map(n =>
    `<button class="${c[n[0]] ? 'on' : ''}" onclick="toggleSetting('${n[0]}', ${!c[n[0]]})">${n[1]}: ${c[n[0]] ? 'ON' : 'OFF'}</button>`
  ).join('');
  const rows = [
    ['Motors enabled', c.motors_enabled ? 'ON' : 'OFF'],
    ['Controller (USB)', c.controller_valid ? 'OK' : 'LOST'],
    ['WiFi', wifi.ssid + ' (' + wifi.ip + ')'],
  ];
  document.getElementById('config').innerHTML = rows.map(r =>
    `<tr><td class="label">${r[0]}</td><td>${r[1]}</td></tr>`
  ).join('');
}

function drawStats(s) {
  const rows = [
    ['Uptime', Math.floor(s.uptime / 60) + ' min ' + (s.uptime % 60) + ' s'],
    ['Loop rate', s.loop_rate + ' /s'],
    ['LiDAR', s.lidar_rate.toFixed(1) + ' rotations/s'],
    ['LiDAR bad packets', s.lidar_bad_packets],
    ['Joystick', s.joystick_rate + ' reports/s'],
    ['Free memory', Math.round(s.free_heap / 1024) + ' KB'],
    ['Free PSRAM', Math.round(s.free_psram / 1024) + ' KB'],
    ['WiFi clients', s.wifi_clients],
    ['Last reset', s.reset_reason],
  ];
  document.getElementById('stats').innerHTML = rows.map(r =>
    `<tr><td class="label">${r[0]}</td><td>${r[1]}</td></tr>`
  ).join('');
}

function drawRecord(r) {
  const time = Math.floor(r.seconds / 60) + ':' + String(r.seconds % 60).padStart(2, '0');
  const el = document.getElementById('recordStatus');
  el.textContent = r.available ? (r.active ? 'Recording ' : 'Stopped, ') + time + ' (' + r.rows + ' rows)' : 'Not available, no PSRAM';
  el.className = r.active ? 'bad' : '';
}

async function toggleSetting(name, value) {
  try { await fetch('/api/setting?name=' + name + '&value=' + (value ? 1 : 0), { method: 'POST' }); } catch (e) {}
  poll();
}

async function record(action) {
  try { await fetch('/api/record?action=' + action, { method: 'POST' }); } catch (e) {}
  poll();
}

function drawLog(lines) {
  const box = document.getElementById('log');
  const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 4;
  box.value = lines.join('\n');
  if (atBottom) box.scrollTop = box.scrollHeight;
}

const debugNames = [['psd', 'PSD'], ['joystick', 'Joystick'], ['joystick_raw', 'Joystick raw'],
  ['lidar', 'Lidar'], ['collision', 'Collision'], ['motors', 'Motors']];

function drawDebug(d) {
  document.getElementById('debugButtons').innerHTML = debugNames.map(n =>
    `<button class="${d[n[0]] ? 'on' : ''}" onclick="toggleDebug('${n[0]}', ${!d[n[0]]})">${n[1]}: ${d[n[0]] ? 'ON' : 'OFF'}</button>`
  ).join('');
}

async function toggleDebug(name, value) {
  try { await fetch('/api/debug?name=' + name + '&value=' + (value ? 1 : 0), { method: 'POST' }); } catch (e) {}
  poll();
}

async function poll() {
  try {
    const r = await fetch('/api/status');
    const s = await r.json();
    drawLidar(s.lidar);
    drawPsd(s.psd);
    drawJoystick(s.joystick);
    drawConfig(s.config, s.wifi);
    drawStats(s.stats);
    drawRecord(s.record);
    drawDebug(s.debug);
    drawLog(s.log);
  } catch (e) { /* device likely rebooting or out of range - just retry */ }
}
setInterval(poll, 300);
poll();
</script>
</body>
</html>
)HTML";

static const char *Json_Bool(bool Value) {
  return Value ? "true" : "false";
}

static void Handle_Root() {
  Server.send_P(200, "text/html", Dashboard_Page);
}

// Everything the dashboard shows, sent as JSON
static void Handle_Status() {
  LidarData Lidar = Get_Lidar_Data();
  PSD_Distances PSD = Get_PSD_Distances();
  ControllerInput Input = Get_Last_Controller_Input();

  const int Max_Points = Lidar_Visible_Angle / Lidar_Radar_Point_Step + 1;
  LidarDisplayPoint Points[Max_Points];
  int Point_Count = Get_Lidar_Display_Points(Points, Max_Points);

  String Points_Json = "[";
  for (int i = 0; i < Point_Count; i++) {
    if (i > 0) Points_Json += ",";
    Points_Json += "{\"angle\":" + String(Points[i].Angle) + ",\"distance\":" + String(Points[i].Distance) + "}";
  }
  Points_Json += "]";

  String Log_Lines[Log_Buffer_Size];
  int Log_Count = Get_Log_Lines(Log_Lines, Log_Buffer_Size);
  String Log_Json = "[";
  for (int i = 0; i < Log_Count; i++) {
    if (i > 0) Log_Json += ",";
    Log_Json += "\"" + Log_Lines[i] + "\"";
  }
  Log_Json += "]";

  String Json = "{";
  Json += "\"lidar\":{\"connected\":" + String(Json_Bool(Lidar.Connected));
  Json += ",\"front_left\":" + String(Lidar.Front_Left) + ",\"front_right\":" + String(Lidar.Front_Right);
  Json += ",\"visible_angle\":" + String(Lidar_Visible_Angle);
  Json += ",\"slow\":" + String(Lidar_Slow_Distance) + ",\"crawl\":0";
  Json += ",\"closest\":" + String(Lidar.Closest) + ",\"points\":" + Points_Json + "},";
  Json += "\"psd\":{\"front_left\":" + String(PSD.Front_Left) + ",\"front_right\":" + String(PSD.Front_Right);
  Json += ",\"side_left\":" + String(PSD.Side_Left) + ",\"side_right\":" + String(PSD.Side_Right);
  Json += ",\"back\":" + String(PSD.Back);
  Json += ",\"slow\":" + String(PSD_Slow_Distance) + ",\"crawl\":" + String(PSD_Crawl_Distance) + "},";
  Json += "\"joystick\":{\"x\":" + String(Input.X, 2) + ",\"y\":" + String(Input.Y, 2);
  Json += ",\"rotation\":" + String(Input.Rotation, 2) + ",\"speed\":" + String(Input.Speed, 2);
  Json += ",\"button\":" + String(Input.Button) + ",\"valid\":" + String(Json_Bool(USB_Controller_Is_Valid())) + "},";
  Json += "\"config\":{\"collision\":" + String(Json_Bool(Collision_Enabled));
  Json += ",\"hardstop\":" + String(Json_Bool(Hardstop_Enabled));
  Json += ",\"square\":" + String(Json_Bool(Square_Inputs));
  Json += ",\"ignore_front_psd\":" + String(Json_Bool(Ignore_Front_PSD));
  Json += ",\"motors_enabled\":" + String(Json_Bool(Motors_Are_Enabled()));
  Json += ",\"controller_valid\":" + String(Json_Bool(USB_Controller_Is_Valid())) + "},";
  Json += "\"wifi\":{\"ssid\":\"" + String(AP_Name) + "\",\"ip\":\"" + WiFi.softAPIP().toString() + "\"},";
  Json += "\"debug\":{\"psd\":" + String(Json_Bool(Debug_PSD));
  Json += ",\"joystick\":" + String(Json_Bool(Debug_Joystick));
  Json += ",\"joystick_raw\":" + String(Json_Bool(Debug_Joystick_Raw));
  Json += ",\"lidar\":" + String(Json_Bool(Debug_Lidar));
  Json += ",\"collision\":" + String(Json_Bool(Debug_Collision));
  Json += ",\"motors\":" + String(Json_Bool(Debug_Motors)) + "},";
  SystemStats Stats = Get_Stats();
  Json += "\"stats\":{\"uptime\":" + String(Stats.Uptime) + ",\"loop_rate\":" + String(Stats.Loop_Rate);
  Json += ",\"lidar_rate\":" + String(Stats.Lidar_Rate, 1) + ",\"lidar_bad_packets\":" + String(Stats.Lidar_Bad_Packets);
  Json += ",\"joystick_rate\":" + String(Stats.Joystick_Rate) + ",\"free_heap\":" + String(Stats.Free_Heap);
  Json += ",\"free_psram\":" + String(Stats.Free_PSRAM) + ",\"wifi_clients\":" + String(Stats.WiFi_Clients);
  Json += ",\"reset_reason\":\"" + String(Stats.Reset_Reason) + "\"},";
  Json += "\"record\":{\"available\":" + String(Json_Bool(Recorder_Available()));
  Json += ",\"active\":" + String(Json_Bool(Is_Recording()));
  Json += ",\"rows\":" + String(Recorded_Rows()) + ",\"seconds\":" + String(Recorded_Seconds()) + "},";
  Json += "\"log\":" + Log_Json;
  Json += "}";

  Server.send(200, "application/json", Json);
}

// Turn one debug output on or off, like /api/debug?name=psd&value=1
static void Handle_Debug() {
  String Name = Server.arg("name");
  bool Value = Server.arg("value") == "1";

  if (Name == "psd") Debug_PSD = Value;
  else if (Name == "joystick") Debug_Joystick = Value;
  else if (Name == "joystick_raw") Debug_Joystick_Raw = Value;
  else if (Name == "lidar") Debug_Lidar = Value;
  else if (Name == "collision") Debug_Collision = Value;
  else if (Name == "motors") Debug_Motors = Value;
  else {
    Server.send(400, "text/plain", "unknown debug name");
    return;
  }

  Log_Line("Debug " + Name + (Value ? " ON" : " OFF"));
  Save_Settings();
  Server.send(200, "text/plain", "ok");
}

// Turn a setting on or off, like /api/setting?name=hardstop&value=1, all but collision are saved
static void Handle_Setting() {
  String Name = Server.arg("name");
  bool Value = Server.arg("value") == "1";

  if (Name == "collision") Collision_Enabled = Value;
  else if (Name == "hardstop") Hardstop_Enabled = Value;
  else if (Name == "square") Square_Inputs = Value;
  else if (Name == "ignore_front_psd") Ignore_Front_PSD = Value;
  else {
    Server.send(400, "text/plain", "unknown setting");
    return;
  }

  Log_Line("Setting " + Name + (Value ? " ON" : " OFF") + " (web)");
  if (Name != "collision") Save_Settings();
  Server.send(200, "text/plain", "ok");
}

// Start or stop recording, like /api/record?action=start
static void Handle_Record() {
  String Action = Server.arg("action");
  if (Action == "start") Start_Recording();
  else if (Action == "stop") Stop_Recording();
  else {
    Server.send(400, "text/plain", "unknown action");
    return;
  }
  Server.send(200, "text/plain", "ok");
}

// Send the recording as a CSV file, the motors stop first since the main loop waits meanwhile
static void Handle_Record_CSV() {
  Disable_Motors("CSV download");
  const int Stopped[4] = {0, 0, 0, 0};
  Send_Motor_Speeds(Stopped);

  Server.sendHeader("Content-Disposition", "attachment; filename=omnichair_recording.csv");
  Server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  Server.send(200, "text/csv", CSV_Header());

  const int Chunk_Rows = 50;
  for (int Start = 0; Start < Recorded_Rows(); Start += Chunk_Rows) {
    Server.sendContent(CSV_Rows(Start, Chunk_Rows));
  }
  Server.sendContent("");
}

// Start the open WiFi network, the omnichair.local name and the web pages
static void Start_Access_Point() {
  WiFi.softAP(AP_Name);
  MDNS.begin(Web_Hostname);
  Server.on("/", Handle_Root);
  Server.on("/api/status", Handle_Status);
  Server.on("/api/debug", HTTP_POST, Handle_Debug);
  Server.on("/api/setting", HTTP_POST, Handle_Setting);
  Server.on("/api/record", HTTP_POST, Handle_Record);
  Server.on("/api/record.csv", Handle_Record_CSV);
  Server.begin();
}

void Setup_Web_Server() {
  if (!Web_UI_Enabled) return;
  Start_Access_Point();
  AP_Running = true;
}

// Turn WiFi on or off with button 11 and answer web requests while it's on
void Update_Web_Server() {
  if (Web_UI_Enabled != AP_Running) {
    if (Web_UI_Enabled) {
      Start_Access_Point();
    } else {
      Server.stop();
      WiFi.softAPdisconnect(true);
    }
    AP_Running = Web_UI_Enabled;
  }

  if (AP_Running) Server.handleClient();
}
