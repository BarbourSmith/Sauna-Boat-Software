// main.cpp – Sauna Boat Steering Module
// ESP32-S3 firmware for the Maslow 4 control board
//
// Hardware: Bottom-Right motor port on Maslow 4 board
//   Motor forward pin : GPIO 9  (brIn2Pin – wires swapped vs encoder direction)
//   Motor backward pin: GPIO 3  (brIn1Pin – wires swapped vs encoder direction)
//   Current sense pin : GPIO 7  (brADCPin)
//   I2C SDA           : GPIO 5
//   I2C SCL           : GPIO 4
//   I2C speed         : 200 kHz
//   I2C Mux address   : 0x70
//   LEDC PWM channel 1: 6
//   LEDC PWM channel 2: 7
//
// Speed commands are accepted from two sources (last write wins):
//   1. Phone web UI  →  WebSocket JSON   {"speed": 0.5}
//   2. Controller    →  ESP-NOW mesh     MSG_SET_SPEED
//
// Web interface: Connect to WiFi AP "SaunaBoatSteering" (password: 12345678)
// Then open http://192.168.4.1 in a browser.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <Update.h>
#include "MotorUnit.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"
#include "mesh_protocol.h"

// ---------------------------------------------------------------------------
// WiFi Access Point credentials
// ---------------------------------------------------------------------------
static const char* AP_SSID = "SaunaBoatSteering";
static const char* AP_PASS = "12345678";

// ---------------------------------------------------------------------------
// Maslow 4 board pin definitions – Bottom-Right motor port
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN      5
#define I2C_SCL_PIN      4
#define I2C_FREQ_HZ      200000
#define I2C_MUX_ADDR     0x70

#define BR_FORWARD_PIN   3   // brIn2Pin (swapped)
#define BR_BACKWARD_PIN  9   // brIn1Pin (swapped)
#define BR_ADC_PIN       7
#define BR_ENCODER_CH    0
#define BR_PWM_CHANNEL1  6
#define BR_PWM_CHANNEL2  7

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static QWIICMUX       i2cMux;
static MotorUnit      steeringMotor;
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static Preferences    prefs;

// Most recent normalized speed command (-1.0 to 1.0).
static float         g_currentSpeed = 0.0f;

// Timestamp of the last speed command from any source (ms).
// Watchdog stops the motor if this goes stale.
static unsigned long g_lastCmdTime  = 0;

// Motor stops if no command is received within this window.
// 200 ms means even one missed 100 ms heartbeat triggers a stop.
static constexpr unsigned long WATCHDOG_TIMEOUT_MS = 200;

// Accumulation buffer for POST /settings body
static String g_settingsBody;

// Set to true by the OTA completion handler; loop() watches this and restarts
// after the HTTP response has had time to be sent.
static bool g_pendingRestart = false;

// ---------------------------------------------------------------------------
// OTA update page (served inline so it's always reachable even if LittleFS
// content is stale or missing after a partial filesystem flash).
// Styled to match the main steering UI.
// ---------------------------------------------------------------------------
static const char OTA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Firmware Update</title>
  <style>
    :root { --bg:#0d1b2a; --panel:#1b2d42; --accent:#00b4d8; --text:#e0e0e0; --subtext:#8fa8c0; }
    * { box-sizing:border-box; margin:0; padding:0; }
    body { font-family:'Segoe UI',Arial,sans-serif; background:var(--bg); color:var(--text);
           display:flex; flex-direction:column; align-items:center;
           padding:40px 16px; gap:24px; }
    h1   { font-size:1.1rem; color:var(--subtext); text-transform:uppercase;
           letter-spacing:0.05em; }
    .card { background:var(--panel); border-radius:16px; padding:28px 24px;
            width:100%; max-width:400px; display:flex; flex-direction:column; gap:16px; }
    p, .hint { font-size:0.82rem; color:var(--subtext); }
    code { background:rgba(0,180,216,0.12); border-radius:4px; padding:2px 5px;
           font-size:0.8rem; color:var(--accent); }
    input[type=file] { background:var(--bg); color:var(--text);
                       border:1px solid rgba(255,255,255,0.12); border-radius:6px;
                       padding:8px 10px; font-size:0.9rem; width:100%; cursor:pointer; }
    button { background:var(--accent); color:#000; border:none; border-radius:8px;
             padding:10px 24px; font-size:0.95rem; font-weight:600; cursor:pointer; width:100%; }
    button:disabled { opacity:0.5; cursor:not-allowed; }
    progress { width:100%; accent-color:var(--accent); border-radius:4px; height:6px; }
    #status { text-align:center; font-size:0.85rem; min-height:1.2em; }
    #status.ok  { color:#4ade80; }
    #status.err { color:#f87171; }
    a { color:var(--accent); font-size:0.82rem; text-decoration:none; }
    a:hover { text-decoration:underline; }
  </style>
</head>
<body>
  <h1>Firmware Update</h1>
  <div class="card">
    <p>Upload the firmware binary produced by PlatformIO:</p>
    <p><code>.pio/build/steering/firmware.bin</code></p>
    <form id="form">
      <input type="file" id="file" accept=".bin" required style="margin-bottom:12px;" />
      <button type="submit" id="btn">Upload &amp; Flash</button>
    </form>
    <progress id="bar" value="0" max="100" style="display:none"></progress>
    <div id="status"></div>
    <a href="/">&#8592; Back to steering control</a>
  </div>
  <script>
    var form = document.getElementById('form');
    var btn  = document.getElementById('btn');
    var bar  = document.getElementById('bar');
    var st   = document.getElementById('status');
    form.addEventListener('submit', function(e) {
      e.preventDefault();
      var file = document.getElementById('file').files[0];
      if (!file) return;
      btn.disabled = true;
      bar.style.display = 'block';
      bar.value = 0;
      st.className = '';
      st.textContent = 'Uploading…';
      var xhr = new XMLHttpRequest();
      xhr.upload.onprogress = function(ev) {
        if (ev.lengthComputable) bar.value = (ev.loaded / ev.total) * 100;
      };
      xhr.onload = function() {
        bar.value = 100;
        if (xhr.status === 200) {
          st.className = 'ok';
          st.textContent = '✓ ' + xhr.responseText;
        } else {
          st.className = 'err';
          st.textContent = '✗ ' + xhr.responseText;
          btn.disabled = false;
        }
      };
      xhr.onerror = function() {
        st.className = 'err';
        st.textContent = '✗ Connection lost — the board may have already rebooted.';
      };
      xhr.open('POST', '/update');
      var fd = new FormData();
      fd.append('firmware', file, file.name);
      xhr.send(fd);
    });
  </script>
</body>
</html>
)rawliteral";

// Motor direction reversal.
// When true, all commanded speeds are negated before reaching the motor,
// swapping which physical direction is "port" and which is "starboard".
// Persisted in NVS under key "dirReverse".
static bool g_dirReverse = false;

// Apply the direction setting to a normalised speed [-1, 1] before driving
// the motor. All call-sites must go through this function.
static inline float applyDir(float speed) {
    return g_dirReverse ? -speed : speed;
}

// ---------------------------------------------------------------------------
// Mesh diagnostics
// ---------------------------------------------------------------------------
// Tracks the ESP-NOW link to the controller module so we can print its health.
static unsigned long g_lastMeshRxTime  = 0;    // millis() of last received ESP-NOW packet
static float         g_lastMeshSpeed   = 0.0f; // last speed value received over mesh
static uint8_t       g_lastMeshSrc     = 0x00; // MODULE_* id of last sender

// How long without a mesh packet before we consider the link "lost" (ms).
static constexpr unsigned long MESH_TIMEOUT_MS = 500;

// Which source set the current speed ("WS"=phone, "Mesh"=controller, "---"=none yet)
static const char* g_cmdSource = "---";

// Last known PS3 controller battery status received over ESP-NOW.
// g_ctrlBattery: -1=unknown, 0=shutdown, 1=dying, 2=low, 3=high, 4=full.
// g_ctrlCharging: true when plugged in and charging.
static int  g_ctrlBattery  = -1;
static bool g_ctrlCharging = false;

// Forward declaration — defined after broadcastStatus() below.
static String buildStatusJson();

// ---------------------------------------------------------------------------
// ESP-NOW receive callback
// Runs in the WiFi driver task. Keep it short.
// g_currentSpeed and g_lastCmdTime are 32-bit aligned — assignments are atomic
// on Xtensa, so no mutex is needed for these simple status variables.
// ---------------------------------------------------------------------------
static void onMeshReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (len < static_cast<int>(sizeof(MeshMessage))) {
        Serial.printf("[Mesh] RX: packet too short (%d bytes), ignoring\n", len);
        return;
    }

    MeshMessage msg;
    memcpy(&msg, data, sizeof(msg));

    // Record that the link is alive regardless of message type.
    g_lastMeshRxTime = millis();
    g_lastMeshSrc    = msg.src;

    if (msg.type == MSG_SET_SPEED) {
        float speed = constrain(msg.value1, -1.0f, 1.0f);
        steeringMotor.setSpeed(applyDir(speed));
        g_currentSpeed  = speed;
        g_lastCmdTime   = millis();
        g_lastMeshSpeed = speed;
        g_cmdSource     = "Mesh";

        Serial.printf("[Mesh] MSG_SET_SPEED from module 0x%02X → %.2f\n",
                      msg.src, speed);
    } else if (msg.type == MSG_CONTROLLER_STATUS) {
        g_ctrlBattery  = static_cast<int>(msg.value1);
        g_ctrlCharging = (msg.value2 >= 0.5f);
        Serial.printf("[Mesh] MSG_CONTROLLER_STATUS from module 0x%02X → battery=%d  charging=%s\n",
                      msg.src, g_ctrlBattery, g_ctrlCharging ? "yes" : "no");
    } else {
        Serial.printf("[Mesh] RX: unknown type 0x%02X from module 0x%02X, ignoring\n",
                      msg.type, msg.src);
    }
}

// ---------------------------------------------------------------------------
// WebSocket event handler
// ---------------------------------------------------------------------------
static void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client #%u connected from %s\n",
                      client->id(), client->remoteIP().toString().c_str());
        // Send current state so the UI is in sync immediately.
        client->text(buildStatusJson());

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
        // Stop the motor and reset speed when the phone disconnects so the
        // watchdog doesn't race to do it first with a stale non-zero speed.
        steeringMotor.stop();
        g_currentSpeed = 0.0f;
        g_cmdSource    = "---";
        // Release the WS slot immediately so a rapid page-refresh reconnects
        // without waiting for the 1-second periodic cleanup.
        srv->cleanupClients();

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = reinterpret_cast<AwsFrameInfo*>(arg);
        if (info->final && info->index == 0 && info->len == len
                && info->opcode == WS_TEXT) {
            String msg(reinterpret_cast<const char*>(data), len);

            // Parse {"speed":0.5}
            int idx = msg.indexOf("\"speed\":");
            if (idx >= 0) {
                float speed = msg.substring(idx + 8).toFloat();
                speed = constrain(speed, -1.0f, 1.0f);
                steeringMotor.setSpeed(applyDir(speed));
                g_currentSpeed = speed;
                g_lastCmdTime  = millis();
                g_cmdSource    = "WS";
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Build the status JSON sent to WebSocket clients.
// Includes speed and the latest controller battery info.
// ---------------------------------------------------------------------------
static String buildStatusJson() {
    String json = "{\"speed\":"    + String(g_currentSpeed, 2)
                + ",\"battery\":"  + String(g_ctrlBattery)
                + ",\"charging\":" + (g_ctrlCharging ? "true" : "false")
                + "}";
    return json;
}

static void broadcastStatus() {
    if (ws.count() == 0) return;
    ws.textAll(buildStatusJson());
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Sauna Boat Steering Module ===");

    // --- I2C bus (needed by MotorUnit even in speed-only mode) ---
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    Wire.setTimeOut(10);

    // --- I2C Multiplexer ---
    if (!i2cMux.begin(I2C_MUX_ADDR, Wire)) {
        Serial.println("[Setup] WARNING: I2C Mux not found – encoder unavailable.");
    } else {
        Serial.println("[Setup] I2C Mux connected.");
    }

    // --- Motor (speed-based; encoder init may fail but motor still works) ---
    steeringMotor.begin(BR_FORWARD_PIN, BR_BACKWARD_PIN, BR_ADC_PIN,
                        BR_ENCODER_CH, BR_PWM_CHANNEL1, BR_PWM_CHANNEL2,
                        i2cMux);

    // --- LittleFS (serves index.html) ---
    if (!LittleFS.begin(true)) {
        Serial.println("[Setup] ERROR: LittleFS mount failed!");
    } else {
        Serial.println("[Setup] LittleFS mounted.");
    }

    // --- NVS: load persisted settings ---
    prefs.begin("steering", false);
    steeringMotor.setMaxSpeed(prefs.getInt("maxSpeed", 512));
    int snapTimeout = prefs.getInt("snapTimeout", 100);
    int rampMs      = prefs.getInt("rampMs", 500);
    g_dirReverse    = prefs.getBool("dirReverse", false);
    steeringMotor.setRampRate(rampMs > 0 ? 1000.0f / rampMs : 0.0f);
    Serial.printf("[Setup] Settings – maxSpeed=%d  snapTimeout=%d  rampMs=%d  dirReverse=%s\n",
                  steeringMotor.getMaxSpeed(), snapTimeout, rampMs,
                  g_dirReverse ? "true" : "false");

    // --- WiFi AP pinned to MESH_WIFI_CHANNEL ---
    // ESP-NOW and the soft-AP must share a channel; we fix both to channel 1.
    WiFi.softAP(AP_SSID, AP_PASS, MESH_WIFI_CHANNEL);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("[Setup] WiFi AP \"%s\" started (channel %d)\n",
                  AP_SSID, MESH_WIFI_CHANNEL);
    Serial.printf("[Setup] Open http://%s in a browser\n", ip.toString().c_str());
    Serial.printf("[Setup] Steering MAC: %s\n", WiFi.softAPmacAddress().c_str());

    // --- ESP-NOW ---
    if (esp_now_init() != ESP_OK) {
        Serial.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Serial.println("[Setup] ESP-NOW initialised – listening for mesh packets");
        esp_now_register_recv_cb(onMeshReceive);
    }

    // --- WebSocket ---
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // --- HTTP routes ---
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
        String json = "{\"maxSpeed\":"    + String(steeringMotor.getMaxSpeed())
                    + ",\"snapTimeout\":" + String(prefs.getInt("snapTimeout", 100))
                    + ",\"rampMs\":"      + String(prefs.getInt("rampMs", 500))
                    + ",\"dirReverse\":"  + (g_dirReverse ? "true" : "false")
                    + "}";
        request->send(200, "application/json", json);
    });

    server.on("/settings", HTTP_POST,
        [](AsyncWebServerRequest* request) {
            auto parseField = [](const String& json, const char* key, float fallback) -> float {
                String search = "\""; search += key; search += "\":";
                int idx = json.indexOf(search);
                if (idx < 0) return fallback;
                return json.substring(idx + search.length()).toFloat();
            };

            auto parseBool = [](const String& json, const char* key, bool fallback) -> bool {
                String search = "\""; search += key; search += "\":";
                int idx = json.indexOf(search);
                if (idx < 0) return fallback;
                String rest = json.substring(idx + search.length());
                rest.trim();
                if (rest.startsWith("true"))  return true;
                if (rest.startsWith("false")) return false;
                return fallback;
            };

            int maxSpeed = constrain(static_cast<int>(
                parseField(g_settingsBody, "maxSpeed",
                           static_cast<float>(steeringMotor.getMaxSpeed()))), 0, 1023);

            int snapTimeout = constrain(static_cast<int>(
                parseField(g_settingsBody, "snapTimeout",
                           static_cast<float>(prefs.getInt("snapTimeout", 100)))), 50, 5000);

            int rampMs = constrain(static_cast<int>(
                parseField(g_settingsBody, "rampMs",
                           static_cast<float>(prefs.getInt("rampMs", 500)))), 0, 2000);

            bool dirReverse = parseBool(g_settingsBody, "dirReverse", g_dirReverse);

            steeringMotor.setMaxSpeed(maxSpeed);
            steeringMotor.setRampRate(rampMs > 0 ? 1000.0f / rampMs : 0.0f);
            g_dirReverse = dirReverse;
            prefs.putInt("maxSpeed",    maxSpeed);
            prefs.putInt("snapTimeout", snapTimeout);
            prefs.putInt("rampMs",      rampMs);
            prefs.putBool("dirReverse", dirReverse);

            Serial.printf("[Settings] Saved – maxSpeed=%d  snapTimeout=%d  rampMs=%d  dirReverse=%s\n",
                          maxSpeed, snapTimeout, rampMs, dirReverse ? "true" : "false");
            request->send(200, "application/json", "{\"ok\":true}");
        },
        nullptr,
        [](AsyncWebServerRequest*, uint8_t* data, size_t len, size_t index, size_t) {
            if (index == 0) g_settingsBody = "";
            g_settingsBody += String(reinterpret_cast<const char*>(data), len);
        }
    );

    // --- OTA update routes ---
    // GET  /update  → upload page (dark-themed, styled to match main UI)
    // POST /update  → receives multipart firmware.bin, flashes it, reboots
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", OTA_HTML);
    });

    server.on("/update", HTTP_POST,
        // Called once when the upload is fully received.
        [](AsyncWebServerRequest* request) {
            bool ok = !Update.hasError();
            AsyncWebServerResponse* resp = request->beginResponse(
                ok ? 200 : 500,
                "text/plain",
                ok ? "Update complete — rebooting in 1 s" : Update.errorString());
            resp->addHeader("Connection", "close");
            request->send(resp);
            if (ok) g_pendingRestart = true;
        },
        // Called for each chunk of the uploaded file.
        [](AsyncWebServerRequest* /*request*/, const String& filename,
           size_t index, uint8_t* data, size_t len, bool final) {
            if (index == 0) {
                Serial.printf("[OTA] Starting update: %s\n", filename.c_str());
                // Stop the motor immediately so it doesn't run unattended
                // while the CPU is busy flashing.
                steeringMotor.stop();
                if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
                    Update.printError(Serial);
                }
            }
            if (len && Update.write(data, len) != len) {
                Update.printError(Serial);
            }
            if (final) {
                if (Update.end(true)) {
                    Serial.printf("[OTA] Flash complete: %u bytes written\n", index + len);
                } else {
                    Update.printError(Serial);
                }
            }
        }
    );

    server.begin();
    Serial.println("[Setup] HTTP server started.");
    Serial.println("[Setup] OTA update available at http://192.168.4.1/update");
    Serial.println("[Setup] Ready – waiting for commands from phone (WS) or controller (Mesh)");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    // OTA restart deferred until after the HTTP response has been sent.
    if (g_pendingRestart) {
        delay(1000);
        ESP.restart();
    }

    unsigned long now = millis();

    // --- Safety watchdog ---
    // Stop the motor if no command (from any source) arrives in time.
    if (g_currentSpeed != 0.0f && (now - g_lastCmdTime) > WATCHDOG_TIMEOUT_MS) {
        Serial.printf("[Loop] Watchdog triggered (%.0f ms since last cmd) – stopping\n",
                      static_cast<float>(now - g_lastCmdTime));
        steeringMotor.stop();
        g_currentSpeed = 0.0f;
        g_cmdSource    = "---";
    }

    // --- Diagnostics (every 500 ms) ---
    static unsigned long lastDiag = 0;
    if (now - lastDiag >= 500) {
        lastDiag = now;

        // Mesh link health
        bool meshLive = (g_lastMeshRxTime > 0) &&
                        (now - g_lastMeshRxTime < MESH_TIMEOUT_MS);
        const char* meshStatus = (g_lastMeshRxTime == 0) ? "NONE"
                               : (meshLive            ? "LIVE" : "LOST");
        unsigned long meshAge  = (g_lastMeshRxTime > 0) ? (now - g_lastMeshRxTime) : 0;

        Serial.printf("[Loop] speed=%.2f  src=%s | mesh=%s  lastRx=%lums ago  lastSpeed=%.2f  src=0x%02X | WS clients=%u\n",
                      g_currentSpeed,
                      g_cmdSource,
                      meshStatus,
                      meshAge,
                      g_lastMeshSpeed,
                      g_lastMeshSrc,
                      ws.count());
    }

    // --- Ramp motor toward commanded speed ---
    steeringMotor.updateRamp();

    // --- Broadcast speed to WebSocket clients (100 ms) ---
    static unsigned long lastBroadcast = 0;
    if (now - lastBroadcast >= 100) {
        lastBroadcast = now;
        broadcastStatus();
    }

    // --- WebSocket cleanup (1 s) ---
    static unsigned long lastCleanup = 0;
    if (now - lastCleanup >= 1000) {
        lastCleanup = now;
        ws.cleanupClients();
    }

    delay(10); // ~100 Hz loop
}
