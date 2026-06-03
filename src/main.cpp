// main.cpp – Sauna Boat Steering System
// ESP32-S3 firmware driving an RC servo via GPIO 48.
//
// Hardware:
//   Servo signal pin : GPIO 48
//   LEDC channel     : 6
//
// Web interface: Connect to WiFi AP "SaunaBoatSteering" (password: 12345678)
// Then open http://192.168.4.1 in a browser.
//
// Mesh: accepts MSG_SET_STEERING commands from any module over ESP-NOW.
// The soft-AP is pinned to MESH_WIFI_CHANNEL so both share one radio channel.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <Preferences.h>
#include "MotorUnit.h"
#include "mesh_protocol.h"

// ---------------------------------------------------------------------------
// WiFi Access Point credentials
// ---------------------------------------------------------------------------
static const char* AP_SSID = "SaunaBoatSteering";
static const char* AP_PASS = "12345678";

// ---------------------------------------------------------------------------
// Steering servo pin definitions
// ---------------------------------------------------------------------------
#define SERVO_PIN  48   // RC servo signal wire

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static MotorUnit      steeringMotor;
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static Preferences    prefs;

// Most recent normalized speed command received from the UI (-1.0 to 1.0).
static float          g_currentSpeed = 0.0f;

// Accumulation buffer for POST /settings request body
static String g_settingsBody;

// ---------------------------------------------------------------------------
// Source-priority for MSG_SET_STEERING
// ---------------------------------------------------------------------------
// Both the handheld and the navigation module send MSG_SET_STEERING.
// The handheld always sends (so steering works without the nav module).
// When the nav module is actively sending (heading hold), its commands take
// priority only while handheld is straight/idle. Any non-straight handheld
// steering command immediately takes over so the user can override heading hold.
static constexpr unsigned long NAV_PRIORITY_MS = 600;  // suppress handheld for this long after a nav command
static unsigned long g_lastNavCmdTime = 0;             // timestamp of last nav module command
static constexpr float HANDHELD_STRAIGHT_DEADZONE = 10.0f / 127.0f;
static constexpr unsigned long HANDHELD_INPUT_TIMEOUT_MS = 300;
static float g_lastHandheldLx = 0.0f;
static unsigned long g_lastHandheldInputMs = 0;

// ---------------------------------------------------------------------------
// ESP-NOW receive callback – runs in the WiFi driver task context.
// Handles MSG_SET_STEERING from any module, with nav module taking priority.
// Updating g_currentSpeed and g_lastCmdTime here is safe: both are 32-bit
// aligned values and the ESP32 performs 32-bit loads/stores atomically.
// ---------------------------------------------------------------------------
static void onMeshReceive(const uint8_t* /*mac*/, const uint8_t* data, int len) {
    if (len < 1) return;
    uint8_t type = data[0];

    if (type == MSG_CONTROLLER_INPUT) {
        if (len < static_cast<int>(sizeof(ControllerInputMessage))) return;
        ControllerInputMessage ci;
        memcpy(&ci, data, sizeof(ci));
        if (ci.src == MODULE_HANDHELD) {
            g_lastHandheldLx = ci.lx;
            g_lastHandheldInputMs = millis();
        }
        return;
    }

    if (len < static_cast<int>(sizeof(MeshMessage))) return;
    MeshMessage msg;
    memcpy(&msg, data, sizeof(msg));

    if (msg.type == MSG_SET_STEERING) {
        unsigned long now = millis();

        if (msg.src == MODULE_NAVIGATION) {
            // Navigation module (heading hold) — always accepted, refreshes priority window.
            g_lastNavCmdTime = now;
        } else if (msg.src == MODULE_HANDHELD) {
            bool handheldInputFresh =
                (now - g_lastHandheldInputMs) <= HANDHELD_INPUT_TIMEOUT_MS;
            bool handheldActive = handheldInputFresh &&
                (g_lastHandheldLx <= -HANDHELD_STRAIGHT_DEADZONE ||
                 g_lastHandheldLx >= HANDHELD_STRAIGHT_DEADZONE);

            // While nav is active, only suppress handheld when the stick is straight/idle.
            if ((now - g_lastNavCmdTime) < NAV_PRIORITY_MS && !handheldActive) {
                return;  // ignore idle handheld input — nav keeps priority
            }
        }

        float speed = constrain(msg.value1, -1.0f, 1.0f);
        steeringMotor.setSpeed(speed);
        g_currentSpeed = speed;
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
        // Send current speed immediately on connect
        String msg = "{\"speed\":" + String(g_currentSpeed, 2) + "}";
        client->text(msg);

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
        // Center the servo immediately when a client disconnects
        steeringMotor.stop();
        g_currentSpeed = 0.0f;

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = reinterpret_cast<AwsFrameInfo*>(arg);
        // Only handle complete, single-frame text messages
        if (info->final && info->index == 0 && info->len == len
                && info->opcode == WS_TEXT) {
            // Copy to a null-terminated String to avoid out-of-bounds write on data[]
            String msg(reinterpret_cast<const char*>(data), len);

            // Parse simple JSON: {"speed":0.5}
            int idx = msg.indexOf("\"speed\":");
            if (idx >= 0) {
                float speed = msg.substring(idx + 8).toFloat();
                speed = constrain(speed, -1.0f, 1.0f);
                steeringMotor.setSpeed(speed);
                g_currentSpeed = speed;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Broadcast current servo position to all connected WebSocket clients
// ---------------------------------------------------------------------------
static void broadcastStatus() {
    if (ws.count() == 0) return;
    String msg = "{\"speed\":" + String(g_currentSpeed, 2) + "}";
    ws.textAll(msg);
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Sauna Boat Steering System ===");

    // --- Steering servo ---
    steeringMotor.begin(SERVO_PIN);

    // --- LittleFS (stores index.html) ---
    if (!LittleFS.begin(true)) {
        Serial.println("[Setup] ERROR: LittleFS mount failed!");
    } else {
        Serial.println("[Setup] LittleFS mounted.");
    }

    // --- NVS: load persisted settings ---
    prefs.begin("steering", false);
    // "maxTravel" replaces the old "maxSpeed" key (which stored the DC-motor PWM
    // limit defaulting to 512).  Using a new key forces the correct default of
    // 1023 (full servo range) on existing devices that have the old value stored.
    steeringMotor.setMaxSpeed(prefs.getInt("maxTravel", 1023));
    // snapTimeout is a UI-only value; the firmware just stores and serves it.
    int snapTimeout = prefs.getInt("snapTimeout", 100);
    // rampMs: time in ms to ramp from 0 to full deflection (0 = instant).
    int rampMs = prefs.getInt("rampMs", 500);
    steeringMotor.setRampRate(rampMs > 0 ? 1000.0f / rampMs : 0.0f);
    // dirReverse: invert servo direction.
    steeringMotor.setReversed(prefs.getBool("dirReverse", false));
    // trimUs: servo center trim offset in µs (-500 to +500).
    steeringMotor.setTrimUs(prefs.getInt("trimUs", 0));
    Serial.printf("[Setup] Settings loaded – maxTravel=%d snapTimeout=%d rampMs=%d dirReverse=%d trimUs=%d\n",
                  steeringMotor.getMaxSpeed(), snapTimeout, rampMs,
                  steeringMotor.getReversed(), steeringMotor.getTrimUs());

    // --- WiFi Access Point ---
    // Pin to MESH_WIFI_CHANNEL so the phone AP and ESP-NOW share one channel.
    WiFi.softAP(AP_SSID, AP_PASS, MESH_WIFI_CHANNEL);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("[Setup] WiFi AP \"%s\" on channel %d started.\n", AP_SSID, MESH_WIFI_CHANNEL);
    Serial.printf("[Setup] Open http://%s in your browser.\n", ip.toString().c_str());

    // --- ESP-NOW (mesh receive) ---
    if (esp_now_init() != ESP_OK) {
        Serial.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Serial.println("[Setup] ESP-NOW initialised.");
        esp_now_register_recv_cb(onMeshReceive);
    }

    // --- WebSocket ---
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // --- HTTP routes ---
    // Serve the web UI
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    // GET /settings – returns all tunable parameters as JSON
    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
        String json = "{\"maxSpeed\":"    + String(steeringMotor.getMaxSpeed())
                    + ",\"snapTimeout\":" + String(prefs.getInt("snapTimeout", 100))
                    + ",\"rampMs\":"      + String(prefs.getInt("rampMs", 500))
                    + ",\"dirReverse\":"  + String(steeringMotor.getReversed() ? "true" : "false")
                    + ",\"trimUs\":"      + String(steeringMotor.getTrimUs())
                    + "}";
        request->send(200, "application/json", json);
    });

    // POST /settings – accepts JSON body with any subset of the above fields
    server.on("/settings", HTTP_POST,
        [](AsyncWebServerRequest* request) {
            auto parseField = [](const String& json, const char* key, float fallback) -> float {
                String search = "\""; search += key; search += "\":";
                int idx = json.indexOf(search);
                if (idx < 0) return fallback;
                return json.substring(idx + search.length()).toFloat();
            };

            int maxSpeed = static_cast<int>(
                parseField(g_settingsBody, "maxSpeed",
                           static_cast<float>(steeringMotor.getMaxSpeed())));
            maxSpeed = constrain(maxSpeed, 0, 1023);
            // Save under the new NVS key

            int snapTimeout = static_cast<int>(
                parseField(g_settingsBody, "snapTimeout",
                           static_cast<float>(prefs.getInt("snapTimeout", 100))));
            snapTimeout = constrain(snapTimeout, 50, 5000);

            int rampMs = static_cast<int>(
                parseField(g_settingsBody, "rampMs",
                           static_cast<float>(prefs.getInt("rampMs", 500))));
            rampMs = constrain(rampMs, 0, 5000);

            // dirReverse: parse boolean ("true"/"false" or 1/0)
            bool dirReverse = steeringMotor.getReversed();
            {
                String search = "\"dirReverse\":";
                int idx = g_settingsBody.indexOf(search);
                if (idx >= 0) {
                    String rest = g_settingsBody.substring(idx + search.length());
                    rest.trim();
                    dirReverse = rest.startsWith("true") || rest.startsWith("1");
                }
            }

            int trimUs = static_cast<int>(
                parseField(g_settingsBody, "trimUs",
                           static_cast<float>(steeringMotor.getTrimUs())));
            trimUs = constrain(trimUs, -500, 500);

            steeringMotor.setMaxSpeed(maxSpeed);
            steeringMotor.setRampRate(rampMs > 0 ? 1000.0f / rampMs : 0.0f);
            steeringMotor.setReversed(dirReverse);
            steeringMotor.setTrimUs(trimUs);
            prefs.putInt("maxTravel",  maxSpeed);
            prefs.putInt("snapTimeout", snapTimeout);
            prefs.putInt("rampMs",     rampMs);
            prefs.putBool("dirReverse", dirReverse);
            prefs.putInt("trimUs",     trimUs);

            Serial.printf("[Settings] Saved – maxTravel=%d snapTimeout=%d rampMs=%d dirReverse=%d trimUs=%d\n",
                          maxSpeed, snapTimeout, rampMs, dirReverse, trimUs);
            request->send(200, "application/json", "{\"ok\":true}");
        },
        nullptr,  // upload handler (not needed)
        [](AsyncWebServerRequest* /*request*/, uint8_t* data, size_t len,
           size_t index, size_t /*total*/) {
            if (index == 0) g_settingsBody = "";
            g_settingsBody += String(reinterpret_cast<const char*>(data), len);
        }
    );

    server.begin();
    Serial.println("[Setup] HTTP server started.");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    // Periodically log diagnostics to serial for debugging
    static unsigned long lastDiag = 0;
    if (millis() - lastDiag >= 500) {
        lastDiag = millis();
        Serial.printf("[Loop] speed=%.2f\n", g_currentSpeed);
    }

    // Broadcast speed to WebSocket clients every 100 ms
    static unsigned long lastBroadcast = 0;
    if (millis() - lastBroadcast >= 100) {
        lastBroadcast = millis();
        broadcastStatus();
    }

    // Periodically clean up disconnected WebSocket clients
    static unsigned long lastCleanup = 0;
    if (millis() - lastCleanup >= 1000) {
        lastCleanup = millis();
        ws.cleanupClients();
    }

    // Step the servo position toward the commanded speed at the configured ramp rate
    steeringMotor.updateRamp();

    delay(10);  // ~100 Hz loop
}
