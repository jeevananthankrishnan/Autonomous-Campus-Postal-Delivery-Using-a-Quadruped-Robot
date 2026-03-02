#include <WiFi.h>
#include <WebServer.h>
#include <esp_camera.h>
#include <esp_now.h>
#include "ArduinoJson.h"


// ===== WiFi credentials for AP mode =====
const char* ssid = "ESP32-CAM-Robot";
const char* password = "12345678";


// ===== MAC address of ESP32-Dev Module (update this!) =====
uint8_t devModuleMac[] = {0xF4, 0x65, 0x0B, 0x42, 0x06, 0xF4}; 


// ===== Structures (MUST MATCH ESP32-Dev EXACTLY) =====
typedef struct {
  uint16_t us_front, us_back, us_left, us_right, us_incline;
  float imu_roll, imu_pitch, imu_yaw;
  double gps_lat, gps_lon;
  bool gps_valid;
  uint16_t lidar_closest_dist;
  float lidar_closest_angle;
  bool lidar_valid;
  uint32_t timestamp;
  float local_N_to_monument;
  float local_E_to_monument;
  float gps_distance_to_monument;
  int8_t nav_command; // 0=Wait, 1=Left, 2=Right, 3=Forward, 4=NotClose 
} struct_message;


// On ESP32, this struct is 8 bytes (4 float + 2 uint16 + 2 padding)
typedef struct {
  float angle;
  uint16_t distance;
} LidarPoint;


#define LIDAR_POINTS_PER_PACKET 38
typedef struct {
  uint8_t packet_index;
  uint8_t total_packets;
  LidarPoint points[LIDAR_POINTS_PER_PACKET];
} LidarDataPacket;


typedef struct {
  char command[16];
} CommandMessage;


// ===== Camera Pinout (AI-THINKER) =====
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


#define LED_PIN 4


// ===== Globals =====
struct_message sensorData;
// The buffer is now a "State Map" of 360 degrees
LidarPoint lidarScanData[360]; 
CommandMessage commandMsg;


WebServer server(80);
unsigned long lastDataReceived = 0;


// ===== ESP-NOW Receive Callback (UPDATED) =====
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(struct_message)) {
    memcpy(&sensorData, incomingData, sizeof(sensorData));
    lastDataReceived = millis();
  } 
  else if (len == sizeof(LidarDataPacket)) {
    LidarDataPacket lidarPacket;
    memcpy(&lidarPacket, incomingData, sizeof(lidarPacket));
    
    // --- FIXED LOGIC FOR 360 SCAN ---
    // Instead of relying on packet_index (which causes gaps if packets drop),
    // we use the actual Angle of the point to determine where to store it.
    for (int i = 0; i < LIDAR_POINTS_PER_PACKET; ++i) {
      float angle = lidarPacket.points[i].angle;
      uint16_t dist = lidarPacket.points[i].distance;
      
      // Convert angle to integer index (0-359)
      int angleIndex = (int)angle; 
      
      // Safety check to prevent crash
      if (angleIndex >= 0 && angleIndex < 360) {
        // Update the specific degree with the new data
        lidarScanData[angleIndex].angle = angle;
        lidarScanData[angleIndex].distance = dist;
      }
    }
  }
}


// ===== Camera Init =====
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QQVGA; // Small size for speed
  config.jpeg_quality = 20; 
  config.fb_count = 1;
  if (esp_camera_init(&config) != ESP_OK)
    Serial.println("Camera init failed");
  else
    Serial.println("Camera initialized");
}


// ===== Web Handlers =====
void handleRoot();
void handleData();
void handleStream();
void handleLidarScan(); // Binary Transfer
void handleCommand();


// --- HTML / JS DASHBOARD ---
void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html lang="en">
  <head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Robot Dog Control</title>


  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&family=JetBrains+Mono:wght@400;600&family=Orbitron:wght@600;800&display=swap" rel="stylesheet">


  <style>
    :root{
      --bg:#0b0f14; --bg2:#0f141c; --card:#101824; --border:#1b2635;
      --muted:#9aa4b2; --text:#e7edf4; --neon:#ff3636;
      --neon-glow:0 0 12px rgba(255,54,54,.55), 0 0 28px rgba(255,54,54,.35);
      --radius:14px; --shadow:0 10px 30px rgba(0,0,0,.35);
    }
    @font-face{ font-family:"LASTICA"; src:url("./Lastica.woff2") format("woff2"); font-display:swap; }
    *{ box-sizing:border-box }
    html,body{ height:100% }
    body{
      margin:0;
      background: radial-gradient(900px 600px at 10% -10%, #0d1421 0%, var(--bg) 60%) fixed;
      color:var(--text);
      font-family: 'Inter', system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif;
      overflow-y:auto;
    }
    .header{ position:sticky; top:0; z-index:5; background:linear-gradient(180deg, rgba(11,15,20,.86), rgba(11,15,20,.55)); border-bottom:1px solid var(--border); backdrop-filter: blur(6px); }
    .header-inner{ max-width:1220px; margin:auto; padding:10px 16px; display:flex; align-items:center; gap:14px; justify-content:space-between; flex-wrap:wrap }
    .title{ display:flex; align-items:center; gap:12px; font-family:"LASTICA",'Orbitron',sans-serif; font-weight:800; font-size:22px; letter-spacing:.6px; color:var(--neon); text-shadow:var(--neon-glow); }
    .toolbar{ display:flex; gap:8px; align-items:center; flex-wrap:wrap }
    .input,.button{ height:34px; border-radius:10px; border:1px solid var(--border); background:#0c121b; color:var(--text); padding:0 12px; font-size:14px; outline:none; }
    .input{ min-width:260px; max-width:60vw }
    .input:focus{ box-shadow:0 0 0 2px rgba(255,54,54,.25) }
    .button{ cursor:pointer; font-weight:800; letter-spacing:.3px; background:linear-gradient(180deg,#1a202a,#121821); transition:all 0.15s ease; }
    .button.accent{ color:#220a0a; border-color:#4a1212; background:linear-gradient(180deg,#ff5858,#ff3030); box-shadow:0 4px 14px rgba(255,48,48,.25) }
    .button:hover{ transform:translateY(-1px) } .button:active{ transform:translateY(0) }


    .wrap{ max-width:1220px; margin:12px auto 24px; padding:0 16px }
    .grid{
      display:grid; gap:12px;
      grid-template-columns: repeat(12, 1fr);
      grid-template-areas:
        "cam cam cam cam cam cam lid lid lid lid lid lid"
        "us us us nav nav nav nav nav gyro gyro gyro gyro"
        "ctrl ctrl ctrl ctrl ctrl ctrl ctrl ctrl ctrl ctrl ctrl ctrl";
      grid-auto-rows:minmax(140px, auto);
    }
    .cam{grid-area:cam;} .lid{grid-area:lid;} .us{grid-area:us;} .nav{grid-area:nav;}
    .gyro{grid-area:gyro;} .ctrl{grid-area:ctrl;}
    .card{background:linear-gradient(180deg,var(--card),var(--bg2));border:1px solid var(--border);border-radius:var(--radius);box-shadow:var(--shadow);display:flex;flex-direction:column;}
    .card .head{padding:10px 12px;border-bottom:1px solid var(--border);display:flex;align-items:center;justify-content:space-between}
    .card h3{margin:0;font-size:13px;letter-spacing:.6px;text-transform:uppercase;font-family:"LASTICA",'Orbitron',sans-serif;color:var(--neon);text-shadow:var(--neon-glow)}
    .card .body{padding:12px;}
    .video-frame{border:1px solid #1b2532;border-radius:12px;overflow:hidden;display:flex;align-items:center;justify-content:center;height:100%;background:#0b0f16}
    .video-frame img{width:100%;height:auto;object-fit:contain; transition: opacity 0.3s;}
    .lidar{background:#000;border:1px solid #213045;border-radius:12px;display:flex;align-items:center;justify-content:center;padding:6px;height:100%}
    canvas{width:100%;height:100%}
    .kv{display:grid;grid-template-columns:repeat(2, 1fr);gap:8px 10px;font-family:'JetBrains Mono',monospace;font-size:13px}
    .kv label{color:var(--muted)} .kv .val{font-weight:800;color:#ffd3d3;text-shadow:0 0 8px rgba(255,54,54,.4)}
    .kv .nav-header { grid-column: 1 / span 2; font-weight: 600; color: var(--text); margin-top: 6px; border-bottom: 1px dashed var(--border); padding-bottom: 2px; }
    .control-grid{display:grid;grid-template-columns:repeat(auto-fit, minmax(180px, 1fr));gap:10px;}
    .ctrl-btn{height:48px;border-radius:12px;border:1px solid var(--border);background:linear-gradient(180deg,#1a2533,#12192a);color:var(--text);font-size:15px;font-weight:800;letter-spacing:.4px;cursor:pointer;transition:all 0.2s ease;font-family:'Orbitron',sans-serif;}
    .ctrl-btn:hover{transform:translateY(-2px);border-color:#ff3636;box-shadow:0 6px 20px rgba(255,48,48,.3);}
    .ctrl-btn:active{transform:translateY(0);}
    .ctrl-btn.active{background:linear-gradient(180deg,#ff5858,#ff3030);color:#220a0a;border-color:#4a1212;box-shadow:0 4px 16px rgba(255,48,48,.4);}
    
    .switch {position: relative; display: inline-block; width: 40px; height: 20px;}
    .switch input {opacity: 0; width: 0; height: 0;}
    .slider {position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #2c3e50; transition: .4s; border-radius: 20px; border: 1px solid var(--border);}
    .slider:before {position: absolute; content: ""; height: 14px; width: 14px; left: 2px; bottom: 2px; background-color: #9aa4b2; transition: .4s; border-radius: 50%;}
    input:checked + .slider {background-color: rgba(255, 54, 54, 0.2); border-color: var(--neon);}
    input:checked + .slider:before {transform: translateX(20px); background-color: var(--neon); box-shadow: var(--neon-glow);}


    @media (max-width: 900px) {
      .grid {
        grid-template-columns: 1fr 1fr;
        grid-template-areas:
          "cam cam"
          "lid lid"
          "us nav"
          "gyro gyro"
          "ctrl ctrl";
      }
    }
  </style>
  </head>
  <body>
    <div class="header">
      <div class="header-inner">
        <div class="title">Robot Dog Control</div>
        <div class="toolbar">
          <input id="camUrl" class="input" placeholder="Camera URL (e.g. http://192.168.4.1/stream)" />
          <button id="setUrlBtn" class="button accent">Set URL</button>
        </div>
      </div>
    </div>


    <div class="wrap">
      <div class="grid">
        <section class="card cam">
          <div class="head">
            <h3>Camera</h3>
            <label class="switch">
              <input type="checkbox" id="streamToggle" checked>
              <span class="slider"></span>
            </label>
          </div>
          <div class="body"><div class="video-frame"><img id="cameraFeed" src="/stream" alt="Camera feed"></div></div>
        </section>


        <section class="card lid"><div class="head"><h3>LiDAR</h3></div><div class="body"><div class="lidar"><canvas id="lidar" width="520" height="520"></canvas></div></div></section>


        <section class="card us"><div class="head"><h3>Ultrasonic Sensors (CM)</h3></div><div class="body"><div class="kv">
          <label>Front</label><div id="usFront" class="val">0</div>
          <label>Left</label><div id="usLeft" class="val">0</div>
          <label>Right</label><div id="usRight" class="val">0</div>
          <label>Back</label><div id="usBack" class="val">0</div>
          <label>Incline</label><div id="usIncline" class="val">0</div>
        </div></div></section>


        <section class="card nav"><div class="head"><h3>NAVIGATION</h3></div><div class="body">
          <div class="kv">
            <div class="nav-header">STATUS</div>
            <label>Distance</label><div id="distToMon" class="val">--</div>
            <div class="nav-header">RAW GPS</div>
            <label>Latitude</label><div id="gpsLat" class="val">--</div>
            <label>Longitude</label><div id="gpsLon" class="val">--</div>
            <div class="nav-header">DIRECTION</div>
            <label>CMD</label><div id="navStatus" class="val" style="color:#00FF00;">--</div>
          </div>
        </div></section>


        <section class="card gyro"><div class="head"><h3>IMU Orientation (°)</h3></div><div class="body"><div class="kv">
          <label>Roll</label><div id="imuRoll" class="val">0.0</div>
          <label>Pitch</label><div id="imuPitch" class="val">0.0</div>
          <label>Yaw</label><div id="imuYaw" class="val">0.0</div>
        </div></div></section>


        <section class="card ctrl"><div class="head"><h3>Controls</h3></div><div class="body"><div class="control-grid">
          
          <!-- BUTTON ADDED HERE -->
          <button class="ctrl-btn" style="border-color:#34b7ff; color:#34b7ff;" onclick="sendCommand('INIT')">INITIALIZE</button>
          
          <button class="ctrl-btn" onclick="sendCommand('POST')">POST</button>
          <button class="ctrl-btn" onclick="sendCommand('H_BUILDING')">H_BUILDING</button>
          <button id="btnStop" class="ctrl-btn" onclick="toggleStop()">STOP</button>
        </div></div></section>
      </div>
    </div>


  <script>
  // --- CAMERA LOGIC ---
  const img = document.getElementById('cameraFeed');
  const toggle = document.getElementById('streamToggle');
  
  setInterval(() => {
    // Only fetch new frame if toggle is CHECKED (ON)
    if (toggle.checked) {
      img.src = '/stream?ts=' + Date.now();
      img.style.opacity = "1";
    } else {
      img.style.opacity = "0.3"; 
    }
  }, 400);


  // UI refs
  const us = ['usFront','usLeft','usRight','usBack','usIncline'].map(id => document.getElementById(id));
  const imuRoll = document.getElementById('imuRoll'), imuPitch = document.getElementById('imuPitch'), imuYaw = document.getElementById('imuYaw');
  const gpsLat  = document.getElementById('gpsLat'),  gpsLon  = document.getElementById('gpsLon');
  const distToMon = document.getElementById('distToMon');
  const navStatus = document.getElementById('navStatus');


  // LiDAR canvas
  const canvas = document.getElementById('lidar'), ctx = canvas.getContext('2d');
  // live buffers
  let currentLidarScan = [];
  let ultra = { front:0, left:0, right:0, back:0, incline:0 };
  const ULTRA_ANGLES = { front:0, right:270, back:180, left:90, incline:0 };
  const lidarConfig = { maxDist: 4000, gridColor:'#333333', fontColor:'#9aa4b2', robotColor:'#FFFFFF' };
  const LIDAR_DOT_R = 2.5;
  const US_DOT_R    = Math.sqrt(3) * LIDAR_DOT_R;
  const US_COLOR    = '#34b7ff';
  
  function toCanvasRad(deg){
    return (270 - deg) * Math.PI / 180;
  }


  function drawPolarPlot(){
    const w=canvas.width, h=canvas.height, cx=w/2, cy=h/2;
    const radius = Math.min(cx,cy)*0.8;
    ctx.fillStyle='#000'; ctx.fillRect(0,0,w,h);
    ctx.strokeStyle = lidarConfig.gridColor; ctx.lineWidth=1;
    ctx.font = '12px "JetBrains Mono"'; ctx.fillStyle = lidarConfig.fontColor;
    ctx.textAlign='center'; ctx.textBaseline='middle';
    for(let i=1;i<=4;i++){ ctx.beginPath(); ctx.arc(cx,cy,(i/4)*radius,0,2*Math.PI); ctx.stroke();
    }
    for(let a=0;a<360;a+=30){
      const rad=toCanvasRad(a);
      ctx.beginPath(); ctx.moveTo(cx,cy); ctx.lineTo(cx+radius*Math.cos(rad), cy+radius*Math.sin(rad)); ctx.stroke();
      const r2=radius*1.1;
      ctx.fillText(String(a), cx+r2*Math.cos(rad), cy+r2*Math.sin(rad));
    }


    const scale = radius / lidarConfig.maxDist;
    currentLidarScan.forEach(p=>{
      const d = p.distance;
      if (d > 0 && d <= lidarConfig.maxDist) {
        if (d <= 1000)      ctx.fillStyle = '#FF3636';
        else if (d <= 2000) ctx.fillStyle = '#FFA500';
        else if (d <= 3000) ctx.fillStyle = '#FFFF00';
        else                ctx.fillStyle = '#00FF00';
  
        const angleRot  = (p.angle + 90) % 360;
        const anglePlot = (360 - angleRot) % 360;
        const rad = toCanvasRad(anglePlot);
        const r   = d * scale;
        ctx.beginPath();
        ctx.arc(cx + r * Math.cos(rad), cy + r * Math.sin(rad), LIDAR_DOT_R, 0, 2 * Math.PI);
        ctx.fill();
      }
    });


    drawUSDot(cx,cy,radius, ULTRA_ANGLES.front,   ultra.front);
    drawUSDot(cx,cy,radius, ULTRA_ANGLES.right,   ultra.right);
    drawUSDot(cx,cy,radius, ULTRA_ANGLES.back,    ultra.back);
    drawUSDot(cx,cy,radius, ULTRA_ANGLES.left,    ultra.left);
    drawUSDot(cx,cy,radius, ULTRA_ANGLES.incline, ultra.incline);


    ctx.fillStyle = lidarConfig.robotColor;
    ctx.beginPath(); ctx.arc(cx,cy,5,0,2*Math.PI); ctx.fill();
  }


  function drawUSDot(cx,cy,radius, angleDeg, distMM){
    if (!distMM || distMM <= 0) return;
    const scale = radius / lidarConfig.maxDist;
    const rad = toCanvasRad(angleDeg);
    const r = Math.min(distMM, lidarConfig.maxDist) * scale;
    ctx.fillStyle = US_COLOR;
    ctx.shadowColor = US_COLOR;
    ctx.shadowBlur = 10;
    ctx.beginPath(); ctx.arc(cx + r*Math.cos(rad), cy + r*Math.sin(rad), US_DOT_R, 0, 2*Math.PI); ctx.fill();
    ctx.shadowBlur = 0;
  }


  // --- BINARY LIDAR FETCH ---
  function updateLidar(){
    fetch('/lidar_scan')
      .then(response => response.arrayBuffer()) 
      .then(buffer => {
        const dataView = new DataView(buffer);
        const points = [];
        const pointSize = 8;
        const count = buffer.byteLength / pointSize;


        for (let i = 0; i < count; i++) {
          const offset = i * pointSize;
          const angle = dataView.getFloat32(offset, true);
          const dist = dataView.getUint16(offset + 4, true);


          if (dist > 0) {
            points.push({ angle: angle, distance: dist });
          }
        }
        currentLidarScan = points;
      })
      .catch(err => {});
  }

  // ============== NEW JAVASCRIPT LOGIC START ==============
  let isPaused = false;

  function toggleStop() {
    const btn = document.getElementById('btnStop');
    
    // Send "STOP" (Robot toggles internally)
    sendCommand('STOP'); 
    
    isPaused = !isPaused;

    if (isPaused) {
      // Change Appearance to RESUME Button
      btn.innerText = "RESUME";
      btn.style.background = "linear-gradient(180deg, #2ecc71, #27ae60)"; // Green
      btn.style.borderColor = "#2ecc71";
      btn.style.color = "#fff";
    } else {
      // Change Appearance back to STOP Button
      btn.innerText = "STOP (3 Blinks)";
      btn.style.background = ""; // Revert to default
      btn.style.borderColor = "";
      btn.style.color = "";
    }
  }
  // ============== NEW JAVASCRIPT LOGIC END ==============
  
  function sendCommand(cmd){
    const btn = event.target;
    btn.classList.add('active');
    setTimeout(()=>btn.classList.remove('active'),300);
    fetch('/command?cmd=' + cmd).catch(()=>{});
  }


  function updateData(){
    fetch('/data').then(r=>r.json()).then(d=>{
      us[0].textContent = d.us_front;
      us[1].textContent = d.us_left;
      us[2].textContent = d.us_right;
      us[3].textContent = d.us_back;
      us[4].textContent = d.us_incline;


      ultra.front   = (d.us_front   || 0) * 10;
      ultra.left    = (d.us_left    || 0) * 10;
      ultra.right   = (d.us_right   || 0) * 10;
      ultra.back    = (d.us_back    || 0) * 10;
      ultra.incline = (d.us_incline || 0) * 10;


      imuRoll.textContent  = d.imu_roll.toFixed(1);
      imuPitch.textContent = d.imu_pitch.toFixed(1);
      imuYaw.textContent   = d.imu_yaw.toFixed(1);


      distToMon.textContent = d.gps_distance_to_monument !== undefined && d.gps_distance_to_monument >= 0
        ? d.gps_distance_to_monument.toFixed(1)
        : '--';


      if (d.gps_valid){
        gpsLat.textContent = d.gps_lat;
        gpsLon.textContent = d.gps_lon;
      } else {
        gpsLat.textContent = '--';
        gpsLon.textContent = '--';
      }


      // --- NEW: Mirror OLED logic for commands ---
      // Obstacle condition (same as Display.update: front < 50cm and > 0)
      const front = d.us_front || 0;
      const left  = d.us_left  || 0;
      const right = d.us_right || 0;

      if (front > 0 && front < 50) {
        // Obstacle mode: show OBSTACLE + S_LEFT / S_RIGHT style suggestion
        // Here we only have one CMD field; choose the suggestion text as primary.
        if (left > right) {
          navStatus.textContent = "S_LEFT";
        } else {
          navStatus.textContent = "S_RIGHT";
        }
      } else {
        // Normal navigation mode: map NavCommand like Display.printNavCommand
        // 0=WAIT, 1=R_LEFT, 2=R_RIGHT, 3=FRWD, 4=FAR
        // Added S_LEFT and S_RIGHT to match Robot Enums 6 and 7
        const cmdMap = ["WAIT", "R_LEFT", "R_RIGHT", "FRWD", "FAR", "W_ORD", "S_LEFT", "S_RIGHT"];
        if (d.nav_cmd !== undefined && d.nav_cmd >= 0 && d.nav_cmd < cmdMap.length) {
          const direction = cmdMap[d.nav_cmd];
          navStatus.textContent = direction;
        } else {
          navStatus.textContent = "--";
        }
      }
      
    }).catch(()=>{});
  }


  window.onload = () => {
    setInterval(updateData, 500);
    setInterval(updateLidar, 250); 
    (function render(){ drawPolarPlot(); requestAnimationFrame(render); })();
  };
  </script>
  </body></html>
  )rawliteral";
  server.send(200, "text/html", html);
}


// --- Data Handler ---
void handleData() {
  JsonDocument doc;
  doc["us_front"]   = sensorData.us_front;
  doc["us_back"]    = sensorData.us_back;
  doc["us_left"]    = sensorData.us_left;
  doc["us_right"]   = sensorData.us_right;
  doc["us_incline"] = sensorData.us_incline;
  doc["imu_roll"]   = sensorData.imu_roll;
  doc["imu_pitch"]  = sensorData.imu_pitch;
  doc["imu_yaw"]    = sensorData.imu_yaw;
  doc["gps_lat"]    = serialized(String(sensorData.gps_lat, 6));
  doc["gps_lon"]    = serialized(String(sensorData.gps_lon, 6));
  doc["gps_valid"]  = sensorData.gps_valid;
  doc["gps_distance_to_monument"] = sensorData.gps_distance_to_monument;
  doc["nav_cmd"] = sensorData.nav_command; 
  doc["lidar_dist"]   = sensorData.lidar_closest_dist;
  doc["lidar_angle"]  = sensorData.lidar_closest_angle;
  doc["lidar_valid"]  = sensorData.lidar_valid;


  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}


// --- Binary Lidar Scan Handler ---
void handleLidarScan() {
  server.sendHeader("Content-Type", "application/octet-stream");
  server.sendContent((char*)lidarScanData, sizeof(lidarScanData));
}


// --- Command Handler ---
void handleCommand() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    Serial.printf("Received web command: %s\n", cmd.c_str());
    memset(&commandMsg, 0, sizeof(commandMsg));
    cmd.toCharArray(commandMsg.command, sizeof(commandMsg));
    esp_err_t result = esp_now_send(devModuleMac, (uint8_t *)&commandMsg, sizeof(commandMsg));
    if (result == ESP_OK) server.send(200, "text/plain", "Command sent to Dev");
    else server.send(500, "text/plain", "Failed to send");
  }
}


// --- Stream Handler ---
void handleStream() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(503, "text/plain", "Camera capture failed");
    return;
  }
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}


// ===== Setup / Loop =====
void setup() {
  Serial.begin(115200);
  setupCamera();
  pinMode(LED_PIN, OUTPUT);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  Serial.println("AP started: " + WiFi.softAPIP().toString());


  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);


  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, devModuleMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);


  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/stream", handleStream);
  server.on("/command", handleCommand);
  server.on("/lidar_scan", handleLidarScan);
  server.begin();


  Serial.println("Web server + ESP-NOW ready.");
}


void loop() {
  server.handleClient();
}