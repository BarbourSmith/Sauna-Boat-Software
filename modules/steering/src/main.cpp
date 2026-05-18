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
// Mesh: accepts MSG_CONTROLLER_INPUT from input modules over ESP-NOW.
// Raw stick and button state is received here; this module applies dead-zone
// and interprets D-pad left/right as trim jog commands.
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

// Timestamp of the last speed command received over mesh or WebSocket (ms).
// Used by the watchdog to stop the motor on connection loss.
static unsigned long  g_lastCmdTime  = 0;

// Trim state – D-pad left/right nudge the servo centre offset while held.
// The value is applied inside MotorUnit::updateRamp() as a position offset.
// g_trimDirty is set whenever trim changes; the loop saves it to NVS once
// the buttons have been released for TRIM_SAVE_DEBOUNCE_MS to avoid flash wear.
static uint16_t       g_prevButtons        = 0;
static unsigned long  g_trimLastChange     = 0;
static bool           g_trimDirty         = false;

// Motor stops if no command arrives within this window (milliseconds).
// Set to 200 ms so the motor stops if even one 100 ms heartbeat is missed.
static constexpr unsigned long WATCHDOG_TIMEOUT_MS = 200;

// Dead-zone applied to raw left-stick X from MSG_CONTROLLER_INPUT.
// Sticks naturally drift; anything within this band is treated as centred.
static constexpr float STICK_DEADZONE = 10.0f / 127.0f;  // ~8% of full range

// Trim step applied each 50 ms frame while the D-pad button is held.
// 0.01 ≈ 10 µs of servo pulse width; at 20 Hz this gives ~0.2 normalised/sec.
static constexpr float TRIM_STEP = 0.01f;

// How long (ms) to wait after the last trim change before saving to NVS.
// Prevents flash wear when the button is held for many frames.
static constexpr unsigned long TRIM_SAVE_DEBOUNCE_MS = 500;

// Accumulation buffer for POST /settings request body
static String g_settingsBody;

// ---------------------------------------------------------------------------
// ESP-NOW receive callback – runs in the WiFi driver task context.
// All 32-bit-aligned globals written here are updated atomically on ESP32.
// ---------------------------------------------------------------------------
static void onMeshReceive(const uint8_t* /*mac*/, const uint8_t* data, int len) {
    if (len < 1) return;
    uint8_t type = data[0];

    if (type == MSG_CONTROLLER_INPUT) {
        if (len < static_cast<int>(sizeof(ControllerInputMessage))) return;
        ControllerInputMessage msg;
        memcpy(&msg, data, sizeof(msg));

        g_lastCmdTime = millis();  // keep the watchdog alive

        // D-pad left/right: adjust trim while the button is held (level-triggered).
        // Each 50 ms frame the button is held nudges the servo centre by TRIM_STEP.
        uint16_t btns = msg.buttons;
        if (btns & CTRL_BTN_LEFT) {
            float trim = constrain(steeringMotor.getTrim() - TRIM_STEP, -1.0f, 1.0f);
            steeringMotor.setTrim(trim);
            g_trimLastChange = millis();
            g_trimDirty = true;
        }
        if (btns & CTRL_BTN_RIGHT) {
            float trim = constrain(steeringMotor.getTrim() + TRIM_STEP, -1.0f, 1.0f);
            steeringMotor.setTrim(trim);
            g_trimLastChange = millis();
            g_trimDirty = true;
        }
        if (g_trimDirty && (btns & (CTRL_BTN_LEFT | CTRL_BTN_RIGHT)) == 0
                        && (g_prevButtons & (CTRL_BTN_LEFT | CTRL_BTN_RIGHT)) != 0) {
            // Both buttons just released — log the settled trim value.
            Serial.printf("[Trim] Adjusted to %.4f (will save to NVS shortly)\n",
                          steeringMotor.getTrim());
        }
        g_prevButtons = btns;

        // Apply dead-zone to left stick X and command the steering speed.
        float lx = msg.lx;
        if (lx > -STICK_DEADZONE && lx < STICK_DEADZONE) lx = 0.0f;
        float speed = constrain(lx, -1.0f, 1.0f);

        steeringMotor.setSpeed(speed);
        g_currentSpeed = speed;

    } else if (type == MSG_SET_STEERING) {
        // Legacy fallback: accept pre-computed speed from any sender.
        if (len < static_cast<int>(sizeof(MeshMessage))) return;
        MeshMessage msg;
        memcpy(&msg, data, sizeof(msg));
        float speed = constrain(msg.value1, -1.0f, 1.0f);
        steeringMotor.setSpeed(speed);
        g_currentSpeed = speed;
        g_lastCmdTime  = millis();
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
                g_lastCmdTime  = millis();
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
    // trim: persistent servo centre offset set via D-pad buttons on the controller.
    steeringMotor.setTrim(prefs.getFloat("trim", 0.0f));
    Serial.printf("[Setup] Settings loaded – maxTravel=%d snapTimeout=%d rampMs=%d trim=%.4f\n",
                  steeringMotor.getMaxSpeed(), snapTimeout, rampMs, steeringMotor.getTrim());

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
        String json = "{\"maxSpeed\":" + String(steeringMotor.getMaxSpeed())
                    + ",\"snapTimeout\":" + String(prefs.getInt("snapTimeout", 100))
                    + ",\"rampMs\":" + String(prefs.getInt("rampMs", 500))
                    + "}";
        // Note: the JSON key is kept as "maxSpeed" for UI compatibility;
        // the NVS key is "maxTravel" internally.
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
            rampMs = constrain(rampMs, 0, 2000);

            steeringMotor.setMaxSpeed(maxSpeed);
            steeringMotor.setRampRate(rampMs > 0 ? 1000.0f / rampMs : 0.0f);
            prefs.putInt("maxTravel", maxSpeed);
            prefs.putInt("snapTimeout", snapTimeout);
            prefs.putInt("rampMs", rampMs);

            Serial.printf("[Settings] Saved – maxTravel=%d snapTimeout=%d rampMs=%d\n",
                          maxSpeed, snapTimeout, rampMs);
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
    // Trim NVS save: write to flash once the D-pad has been released for
    // TRIM_SAVE_DEBOUNCE_MS, avoiding repeated writes while the button is held.
    if (g_trimDirty && (millis() - g_trimLastChange) >= TRIM_SAVE_DEBOUNCE_MS) {
        prefs.putFloat("trim", steeringMotor.getTrim());
        g_trimDirty = false;
        Serial.printf("[Trim] Saved to NVS: %.4f\n", steeringMotor.getTrim());
    }

    // Safety watchdog: if no speed command has been received for WATCHDOG_TIMEOUT_MS
    // and the servo is active, center it (handles network loss or browser close).
    if (g_currentSpeed != 0.0f && (millis() - g_lastCmdTime) > WATCHDOG_TIMEOUT_MS) {
        Serial.println("[Loop] Watchdog: no recent command – centering servo.");
        steeringMotor.stop();
        g_currentSpeed = 0.0f;
    }

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
