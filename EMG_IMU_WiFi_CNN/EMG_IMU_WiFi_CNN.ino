// EMG_IMU_WiFi_CNN.ino
// Main firmware: 500 Hz sampling, WiFi station, HTTP dashboard (port 80),
// WebSocket server (port 81) streaming live samples and 21×100 CNN windows.
//
// Hardware (from copilot_context.md):
//   EMG: GPIO34 (EMG1), GPIO35 (EMG2), GPIO32 (EMG3) — INPUT ONLY
//   IMU Bus 0 (Wire):  SDA=21 SCL=22 → IMU1 @ 0x68, IMU2 @ 0x69
//   IMU Bus 1 (Wire1): SDA=25 SCL=26 → IMU3 @ 0x68
//
// Libraries required:
//   - "WebSockets" by Markus Sattler (WebSocketsServer)
//   - "ArduinoJson" (v6 or v7)
//
// Uses ESP32 core v3.x timer API.

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ── WiFi credentials — update before flashing ───────────────────────────────
const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// ── Constants ────────────────────────────────────────────────────────────────
#define WINDOW_SIZE     100
#define TOTAL_ROWS      21
#define SAMPLE_RATE_HZ  500

// Row index map (matches copilot_context.md row layout)
#define ROW_EMG1    0
#define ROW_IMU1_AX 1
#define ROW_IMU1_AY 2
#define ROW_IMU1_AZ 3
#define ROW_IMU1_GX 4
#define ROW_IMU1_GY 5
#define ROW_IMU1_GZ 6
#define ROW_EMG2    7
#define ROW_IMU2_AX 8
#define ROW_IMU2_AY 9
#define ROW_IMU2_AZ 10
#define ROW_IMU2_GX 11
#define ROW_IMU2_GY 12
#define ROW_IMU2_GZ 13
#define ROW_EMG3    14
#define ROW_IMU3_AX 15
#define ROW_IMU3_AY 16
#define ROW_IMU3_AZ 17
#define ROW_IMU3_GX 18
#define ROW_IMU3_GY 19
#define ROW_IMU3_GZ 20

// ── Pin / address constants ──────────────────────────────────────────────────
static const int     EMG_PIN1    = 34;
static const int     EMG_PIN2    = 35;
static const int     EMG_PIN3    = 32;

static const uint8_t IMU1_ADDR   = 0x68;
static const uint8_t IMU2_ADDR   = 0x69;
static const uint8_t IMU3_ADDR   = 0x68;

// MPU6050 register addresses
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_CONFIG     = 0x1A;
static const uint8_t REG_ACCEL_XOUT = 0x3B;

// ── Timer ────────────────────────────────────────────────────────────────────
static hw_timer_t*   tmr         = nullptr;
static volatile bool sampleReady = false;

void IRAM_ATTR onTimerISR() {
    sampleReady = true;
}

// ── Servers ──────────────────────────────────────────────────────────────────
WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ── Circular / snapshot buffer ───────────────────────────────────────────────
static float sensorBuffer[TOTAL_ROWS][WINDOW_SIZE];
static int   sampleCount  = 0;   // 0 .. WINDOW_SIZE-1
static int   liveSendDiv  = 0;   // throttle live samples to ~50 Hz (every 10th)

// ── HTML dashboard (PROGMEM) ─────────────────────────────────────────────────
static const char DASHBOARD_HTML[] PROGMEM = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>EMG + IMU Live Dashboard</title>
<style>
  body { font-family: monospace; background: #111; color: #0f0; margin: 20px; }
  h2   { color: #0ff; }
  .bar-wrap { display:flex; align-items:center; margin: 4px 0; }
  .bar-label { width: 80px; }
  .bar-bg    { background:#222; width:300px; height:18px; border-radius:4px; overflow:hidden; }
  .bar-fill  { background:#0f0; height:100%; width:0%; transition:width 0.1s; }
  .num       { margin-left:10px; width:60px; }
  #status    { color:#ff0; margin-bottom:10px; }
</style>
</head>
<body>
<h2>Knee Brace — Live Biosignal Dashboard</h2>
<div id="status">Connecting…</div>

<h3>EMG Channels (0–4095)</h3>
<div class="bar-wrap"><span class="bar-label">EMG 1</span>
  <div class="bar-bg"><div class="bar-fill" id="b_emg1"></div></div>
  <span class="num" id="n_emg1">—</span></div>
<div class="bar-wrap"><span class="bar-label">EMG 2</span>
  <div class="bar-bg"><div class="bar-fill" id="b_emg2"></div></div>
  <span class="num" id="n_emg2">—</span></div>
<div class="bar-wrap"><span class="bar-label">EMG 3</span>
  <div class="bar-bg"><div class="bar-fill" id="b_emg3"></div></div>
  <span class="num" id="n_emg3">—</span></div>

<h3>IMU1 Accelerometer (raw int16)</h3>
<div class="bar-wrap"><span class="bar-label">ax</span>
  <div class="bar-bg"><div class="bar-fill" id="b_ax"></div></div>
  <span class="num" id="n_ax">—</span></div>
<div class="bar-wrap"><span class="bar-label">ay</span>
  <div class="bar-bg"><div class="bar-fill" id="b_ay"></div></div>
  <span class="num" id="n_ay">—</span></div>
<div class="bar-wrap"><span class="bar-label">az</span>
  <div class="bar-bg"><div class="bar-fill" id="b_az"></div></div>
  <span class="num" id="n_az">—</span></div>

<p id="window_info" style="color:#888">Waiting for CNN window…</p>

<script>
var ESP_IP = window.location.hostname;
var ws = new WebSocket("ws://" + ESP_IP + ":81");

ws.onopen = function() {
  document.getElementById("status").textContent = "Connected to ws://" + ESP_IP + ":81";
};
ws.onclose = function() {
  document.getElementById("status").textContent = "Disconnected — reload to reconnect";
};
ws.onerror = function(e) {
  document.getElementById("status").textContent = "WebSocket error";
};

function setBar(barId, numId, val, maxVal) {
  var pct = Math.min(100, Math.max(0, (val / maxVal) * 100));
  document.getElementById(barId).style.width = pct + "%";
  document.getElementById(numId).textContent = val;
}

ws.onmessage = function(evt) {
  var d = JSON.parse(evt.data);
  if (d.t === "s") {
    setBar("b_emg1","n_emg1", d.emg[0], 4095);
    setBar("b_emg2","n_emg2", d.emg[1], 4095);
    setBar("b_emg3","n_emg3", d.emg[2], 4095);
    setBar("b_ax","n_ax", d.imu1[0], 32767);
    setBar("b_ay","n_ay", d.imu1[1], 32767);
    setBar("b_az","n_az", d.imu1[2], 32767);
  } else if (d.t === "w") {
    document.getElementById("window_info").textContent =
      "CNN window received — " + d.rows.length + " rows × " + d.rows[0].length + " cols";
  }
};
</script>
</body>
</html>)rawhtml";

// ── MPU6050 helpers ──────────────────────────────────────────────────────────
static void mpuWrite(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t val) {
    bus.beginTransmission(addr);
    bus.write(reg);
    bus.write(val);
    bus.endTransmission();
}

void readMPU(TwoWire& bus, uint8_t addr,
             int16_t& ax, int16_t& ay, int16_t& az,
             int16_t& gx, int16_t& gy, int16_t& gz) {
    bus.beginTransmission(addr);
    bus.write(REG_ACCEL_XOUT);
    bus.endTransmission(false);

    bus.requestFrom((uint8_t)addr, (uint8_t)14);

    uint8_t buf[14] = {0};
    for (int i = 0; i < 14 && bus.available(); i++) {
        buf[i] = bus.read();
    }

    ax = (int16_t)((buf[0]  << 8) | buf[1]);
    ay = (int16_t)((buf[2]  << 8) | buf[3]);
    az = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6:7] = temperature — skipped
    gx = (int16_t)((buf[8]  << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);
}

static void initMPU(TwoWire& bus, uint8_t addr) {
    mpuWrite(bus, addr, REG_PWR_MGMT_1, 0x00);
    mpuWrite(bus, addr, REG_CONFIG,     0x03);
}

// ── Normalisation ─────────────────────────────────────────────────────────────
// Per-row, per-window min-max normalisation to [0.0, 1.0].
void normaliseWindow(float buf[TOTAL_ROWS][WINDOW_SIZE]) {
    for (int row = 0; row < TOTAL_ROWS; row++) {
        float mn = buf[row][0];
        float mx = buf[row][0];
        for (int col = 1; col < WINDOW_SIZE; col++) {
            if (buf[row][col] < mn) mn = buf[row][col];
            if (buf[row][col] > mx) mx = buf[row][col];
        }
        float range = mx - mn;
        for (int col = 0; col < WINDOW_SIZE; col++) {
            buf[row][col] = (range == 0.0f) ? 0.0f : (buf[row][col] - mn) / range;
        }
    }
}

// ── WebSocket event handler ──────────────────────────────────────────────────
void onWebSocketEvent(uint8_t num, WStype_t type,
                      uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.printf("[WS] Client #%u connected\n", num);
            break;
        case WStype_DISCONNECTED:
            Serial.printf("[WS] Client #%u disconnected\n", num);
            break;
        case WStype_TEXT:
            Serial.printf("[WS] Client #%u text: %s\n", num, payload);
            break;
        default:
            break;
    }
}

// ── HTTP route handlers ───────────────────────────────────────────────────────
void handleRoot() {
    httpServer.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleNotFound() {
    httpServer.send(404, "text/plain", "Not found");
}

// ── setup() ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // ADC configuration — pins are INPUT ONLY, never set as OUTPUT
    analogSetAttenuation(ADC_11db);

    // I2C buses
    Wire.begin(21, 22);
    Wire1.begin(25, 26);

    // Wake and configure all three MPU6050s
    initMPU(Wire,  IMU1_ADDR);
    initMPU(Wire,  IMU2_ADDR);
    initMPU(Wire1, IMU3_ADDR);

    // Initialise buffer to zero
    memset(sensorBuffer, 0, sizeof(sensorBuffer));

    // WiFi — station mode
    Serial.print("Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // HTTP server
    httpServer.on("/", handleRoot);
    httpServer.onNotFound(handleNotFound);
    httpServer.begin();
    Serial.println("HTTP server started on port 80");

    // WebSocket server
    wsServer.begin();
    wsServer.onEvent(onWebSocketEvent);
    Serial.println("WebSocket server started on port 81");

    // Hardware timer: 1 MHz tick, alarm every 2 000 ticks = 500 Hz
    tmr = timerBegin(1000000);
    timerAttachInterrupt(tmr, &onTimerISR);
    timerAlarm(tmr, 2000, true, 0);
}

// ── loop() ───────────────────────────────────────────────────────────────────
void loop() {
    httpServer.handleClient();
    wsServer.loop();

    if (!sampleReady) return;
    sampleReady = false;

    // ── Read all sensors ─────────────────────────────────────────────────────
    int emg1 = analogRead(EMG_PIN1);
    int emg2 = analogRead(EMG_PIN2);
    int emg3 = analogRead(EMG_PIN3);

    int16_t ax1, ay1, az1, gx1, gy1, gz1;
    readMPU(Wire,  IMU1_ADDR, ax1, ay1, az1, gx1, gy1, gz1);

    int16_t ax2, ay2, az2, gx2, gy2, gz2;
    readMPU(Wire,  IMU2_ADDR, ax2, ay2, az2, gx2, gy2, gz2);

    int16_t ax3, ay3, az3, gx3, gy3, gz3;
    readMPU(Wire1, IMU3_ADDR, ax3, ay3, az3, gx3, gy3, gz3);

    // ── Fill circular buffer (row layout from copilot_context.md) ────────────
    int col = sampleCount;
    sensorBuffer[ROW_EMG1][col]    = (float)emg1;
    sensorBuffer[ROW_IMU1_AX][col] = (float)ax1;
    sensorBuffer[ROW_IMU1_AY][col] = (float)ay1;
    sensorBuffer[ROW_IMU1_AZ][col] = (float)az1;
    sensorBuffer[ROW_IMU1_GX][col] = (float)gx1;
    sensorBuffer[ROW_IMU1_GY][col] = (float)gy1;
    sensorBuffer[ROW_IMU1_GZ][col] = (float)gz1;
    sensorBuffer[ROW_EMG2][col]    = (float)emg2;
    sensorBuffer[ROW_IMU2_AX][col] = (float)ax2;
    sensorBuffer[ROW_IMU2_AY][col] = (float)ay2;
    sensorBuffer[ROW_IMU2_AZ][col] = (float)az2;
    sensorBuffer[ROW_IMU2_GX][col] = (float)gx2;
    sensorBuffer[ROW_IMU2_GY][col] = (float)gy2;
    sensorBuffer[ROW_IMU2_GZ][col] = (float)gz2;
    sensorBuffer[ROW_EMG3][col]    = (float)emg3;
    sensorBuffer[ROW_IMU3_AX][col] = (float)ax3;
    sensorBuffer[ROW_IMU3_AY][col] = (float)ay3;
    sensorBuffer[ROW_IMU3_AZ][col] = (float)az3;
    sensorBuffer[ROW_IMU3_GX][col] = (float)gx3;
    sensorBuffer[ROW_IMU3_GY][col] = (float)gy3;
    sensorBuffer[ROW_IMU3_GZ][col] = (float)gz3;

    sampleCount++;

    // ── Live sample broadcast (throttled to ~50 Hz = every 10th sample) ──────
    liveSendDiv++;
    if (liveSendDiv >= 10) {
        liveSendDiv = 0;

        // Build compact JSON: {"t":"s","emg":[v1,v2,v3],"imu1":[ax,ay,az]}
        StaticJsonDocument<128> doc;
        doc["t"] = "s";
        JsonArray emgArr  = doc.createNestedArray("emg");
        emgArr.add(emg1);
        emgArr.add(emg2);
        emgArr.add(emg3);
        JsonArray imu1Arr = doc.createNestedArray("imu1");
        imu1Arr.add(ax1);
        imu1Arr.add(ay1);
        imu1Arr.add(az1);

        String liveMsg;
        serializeJson(doc, liveMsg);
        wsServer.broadcastTXT(liveMsg);
    }

    // ── CNN window: send when WINDOW_SIZE samples collected ──────────────────
    if (sampleCount >= WINDOW_SIZE) {
        sampleCount = 0;

        // Normalise a copy so the raw buffer is preserved for the next window
        float normBuf[TOTAL_ROWS][WINDOW_SIZE];
        memcpy(normBuf, sensorBuffer, sizeof(normBuf));
        normaliseWindow(normBuf);

        // Build JSON: {"t":"w","rows":[[...100 floats...] × 21]}
        // Use String concatenation to avoid heap fragmentation from DynamicJsonDocument
        // with 2100 floats.
        String msg;
        msg.reserve(12000);
        msg = "{\"t\":\"w\",\"rows\":[";
        for (int row = 0; row < TOTAL_ROWS; row++) {
            if (row > 0) msg += ',';
            msg += '[';
            for (int c = 0; c < WINDOW_SIZE; c++) {
                if (c > 0) msg += ',';
                // 4 decimal places is sufficient for [0,1] normalised values
                msg += String(normBuf[row][c], 4);
            }
            msg += ']';
        }
        msg += "]}";
        wsServer.broadcastTXT(msg);
    }
}
