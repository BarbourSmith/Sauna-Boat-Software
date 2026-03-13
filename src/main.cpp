// main.cpp – Sauna Boat Steering System
// ESP32-S3 firmware for the Maslow 4 control board
//
// Hardware: Bottom-Right motor port on Maslow 4 board
//   Motor forward pin : GPIO 9  (brIn1Pin)
//   Motor backward pin: GPIO 3  (brIn2Pin)
//   Current sense pin : GPIO 7  (brADCPin)
//   Encoder I2C mux ch: 0       (BREncoderLine)
//   LEDC PWM channel 1: 6       (brIn1Channel)
//   LEDC PWM channel 2: 7       (brIn2Channel)
//   I2C SDA           : GPIO 5
//   I2C SCL           : GPIO 4
//   I2C speed         : 200 kHz
//   I2C Mux address   : 0x70
//
// Web interface: Connect to WiFi AP "SaunaBoatSteering" (password: 12345678)
// Then open http://192.168.4.1 in a browser.

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <Preferences.h>
#include "MotorUnit.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// ---------------------------------------------------------------------------
// WiFi Access Point credentials
// ---------------------------------------------------------------------------
static const char* AP_SSID = "SaunaBoatSteering";
static const char* AP_PASS = "12345678";

// ---------------------------------------------------------------------------
// Maslow 4 board pin definitions – Bottom-Right motor port
// ---------------------------------------------------------------------------
#define I2C_SDA_PIN       5
#define I2C_SCL_PIN       4
#define I2C_FREQ_HZ       200000
#define I2C_MUX_ADDR      0x70

#define BR_FORWARD_PIN    3    // brIn2Pin (swapped: motor wires are inverse to encoder direction)
#define BR_BACKWARD_PIN   9    // brIn1Pin (swapped: motor wires are inverse to encoder direction)
#define BR_ADC_PIN        7    // brADCPin
#define BR_ENCODER_CH     0    // BREncoderLine
#define BR_PWM_CHANNEL1   6    // brIn1Channel
#define BR_PWM_CHANNEL2   7    // brIn2Channel

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static QWIICMUX    i2cMux;
static MotorUnit   steeringMotor;
static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static Preferences prefs;

// Most recent normalized speed command received from the UI (-1.0 to 1.0).
static float          g_currentSpeed = 0.0f;

// Timestamp of the last speed command received over WebSocket (ms).
// Used by the watchdog to stop the motor on connection loss.
static unsigned long  g_lastCmdTime  = 0;

// Motor stops if no speed command arrives within this window (milliseconds).
// Set to 200 ms so the motor stops if even one 100 ms heartbeat is missed.
static constexpr unsigned long WATCHDOG_TIMEOUT_MS = 200;

// Accumulation buffer for POST /settings request body
static String g_settingsBody;

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
        // Stop the motor immediately when a client disconnects
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
// Broadcast current motor speed to all connected WebSocket clients
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

    // --- I2C bus ---
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    Wire.setTimeOut(10);

    // --- I2C Multiplexer (TCA9548A on Maslow 4 board) ---
    if (!i2cMux.begin(I2C_MUX_ADDR, Wire)) {
        Serial.println("[Setup] WARNING: I2C Mux not found – check wiring.");
    } else {
        Serial.println("[Setup] I2C Mux connected.");
    }

    // --- Steering motor (Bottom-Right port) ---
    steeringMotor.begin(BR_FORWARD_PIN, BR_BACKWARD_PIN, BR_ADC_PIN,
                        BR_ENCODER_CH, BR_PWM_CHANNEL1, BR_PWM_CHANNEL2,
                        i2cMux);

    // --- LittleFS (stores index.html) ---
    if (!LittleFS.begin(true)) {
        Serial.println("[Setup] ERROR: LittleFS mount failed!");
    } else {
        Serial.println("[Setup] LittleFS mounted.");
    }

    // --- NVS: load persisted settings ---
    prefs.begin("steering", false);
    steeringMotor.setMaxSpeed(prefs.getInt("maxSpeed", 512));
    // snapTimeout is a UI-only value; the firmware just stores and serves it.
    int snapTimeout = prefs.getInt("snapTimeout", 100);
    // rampMs: time in ms to ramp from 0 to full speed (0 = instant).
    int rampMs = prefs.getInt("rampMs", 500);
    steeringMotor.setRampRate(rampMs > 0 ? 1000.0f / rampMs : 0.0f);
    Serial.printf("[Setup] Settings loaded – maxSpeed=%d snapTimeout=%d rampMs=%d\n",
                  steeringMotor.getMaxSpeed(), snapTimeout, rampMs);

    // --- WiFi Access Point ---
    WiFi.softAP(AP_SSID, AP_PASS);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("[Setup] WiFi AP \"%s\" started.\n", AP_SSID);
    Serial.printf("[Setup] Open http://%s in your browser.\n", ip.toString().c_str());

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
            prefs.putInt("maxSpeed", maxSpeed);
            prefs.putInt("snapTimeout", snapTimeout);
            prefs.putInt("rampMs", rampMs);

            Serial.printf("[Settings] Saved – maxSpeed=%d snapTimeout=%d rampMs=%d\n",
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
    // Safety watchdog: if no speed command has been received for WATCHDOG_TIMEOUT_MS
    // and the motor is running, stop it (handles network loss or browser close).
    if (g_currentSpeed != 0.0f && (millis() - g_lastCmdTime) > WATCHDOG_TIMEOUT_MS) {
        Serial.println("[Loop] Watchdog: no recent command – stopping motor.");
        steeringMotor.stop();
        g_currentSpeed = 0.0f;
    }

    // Periodically log diagnostics to serial for debugging
    static unsigned long lastDiag = 0;
    if (millis() - lastDiag >= 500) {
        lastDiag = millis();
        Serial.printf("[Loop] speed=%.2f motorCurrent=%.0f\n",
                      g_currentSpeed,
                      steeringMotor.getMotorCurrent());
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

    // Step the motor output toward the commanded speed at the configured ramp rate
    steeringMotor.updateRamp();

    delay(10);  // ~100 Hz loop
}
