// main.cpp – Sauna Boat Auto Steering System
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

// North-trim offset (degrees): the raw encoder angle that maps to heading 0°.
// Applied client-side for display; persisted here so the browser can reload it.
static float g_northTrim = 0.0f;

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
        // Send current state immediately on connect
        float current = steeringMotor.getCurrentAngle();
        float target  = steeringMotor.getTargetAngle();
        String msg = "{\"current\":" + String(current, 1)
                   + ",\"target\":"  + String(target, 1) + "}";
        client->text(msg);

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = reinterpret_cast<AwsFrameInfo*>(arg);
        // Only handle complete, single-frame text messages
        if (info->final && info->index == 0 && info->len == len
                && info->opcode == WS_TEXT) {
            // Copy to a null-terminated String to avoid out-of-bounds write on data[]
            String msg(reinterpret_cast<const char*>(data), len);

            // Parse simple JSON: {"angle":180.0}
            int idx = msg.indexOf("\"angle\":");
            if (idx >= 0) {
                float angle = msg.substring(idx + 8).toFloat();
                angle = constrain(angle, 0.0f, 360.0f);
                steeringMotor.setTargetAngle(angle);
                Serial.printf("[WS] New target angle: %.1f\xC2\xB0\n", angle);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Broadcast current and target angles to all connected WebSocket clients
// ---------------------------------------------------------------------------
static void broadcastStatus() {
    if (ws.count() == 0) return;
    float current = steeringMotor.getCurrentAngle();
    float target  = steeringMotor.getTargetAngle();
    String msg = "{\"current\":" + String(current, 1)
               + ",\"target\":"  + String(target, 1) + "}";
    ws.textAll(msg);
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Sauna Boat Auto Steering System ===");

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
    Serial.printf("[Setup] Encoder connected: %s\n",
                  steeringMotor.isEncoderConnected() ? "YES" : "NO");
    Serial.printf("[Setup] Magnet detected:   %s\n",
                  steeringMotor.hasMagnet() ? "YES" : "NO");
    if (steeringMotor.updateEncoderPosition()) {
        Serial.printf("[Setup] Initial angle: %.1f deg\n", steeringMotor.getCurrentAngle());
    } else {
        Serial.println("[Setup] WARNING: Could not read encoder angle on startup!");
    }

    // --- LittleFS (stores index.html) ---
    if (!LittleFS.begin(true)) {
        Serial.println("[Setup] ERROR: LittleFS mount failed!");
    } else {
        Serial.println("[Setup] LittleFS mounted.");
    }

    // --- NVS: load persisted settings ---
    prefs.begin("steering", false);
    steeringMotor.setKp(      prefs.getFloat("kp",       3.5f));
    steeringMotor.setKi(      prefs.getFloat("ki",       0.005f));
    steeringMotor.setKd(      prefs.getFloat("kd",       0.3f));
    steeringMotor.setDeadband(prefs.getFloat("deadband", 3.0f));
    g_northTrim = prefs.getFloat("northTrim", 0.0f);
    Serial.printf("[Setup] Settings loaded – Kp=%.3f Ki=%.4f Kd=%.3f deadband=%.1f northTrim=%.1f\n",
                  steeringMotor.getKp(), steeringMotor.getKi(),
                  steeringMotor.getKd(), steeringMotor.getDeadband(), g_northTrim);

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

    // JSON status endpoint for polling clients
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
        float current = steeringMotor.getCurrentAngle();
        float target  = steeringMotor.getTargetAngle();
        String json = "{\"current\":" + String(current, 1)
                    + ",\"target\":"  + String(target, 1) + "}";
        request->send(200, "application/json", json);
    });

    // GET /settings – returns all tunable parameters as JSON
    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
        String json = "{\"kp\":"        + String(steeringMotor.getKp(),       3)
                    + ",\"ki\":"        + String(steeringMotor.getKi(),       4)
                    + ",\"kd\":"        + String(steeringMotor.getKd(),       3)
                    + ",\"deadband\":"+  String(steeringMotor.getDeadband(),  1)
                    + ",\"northTrim\":" + String(g_northTrim,                 1)
                    + "}";
        request->send(200, "application/json", json);
    });

    // POST /settings – accepts JSON body with any subset of the above fields
    server.on("/settings", HTTP_POST,
        [](AsyncWebServerRequest* request) {
            // Parse fields from the accumulated body
            auto parseField = [](const String& json, const char* key, float fallback) -> float {
                String search = "\""; search += key; search += "\":";
                int idx = json.indexOf(search);
                if (idx < 0) return fallback;
                return json.substring(idx + search.length()).toFloat();
            };

            float kp       = parseField(g_settingsBody, "kp",        steeringMotor.getKp());
            float ki       = parseField(g_settingsBody, "ki",        steeringMotor.getKi());
            float kd       = parseField(g_settingsBody, "kd",        steeringMotor.getKd());
            float deadband = parseField(g_settingsBody, "deadband",  steeringMotor.getDeadband());
            float northTrim= parseField(g_settingsBody, "northTrim", g_northTrim);

            steeringMotor.setKp(kp);
            steeringMotor.setKi(ki);
            steeringMotor.setKd(kd);
            steeringMotor.setDeadband(deadband);
            g_northTrim = northTrim;

            prefs.putFloat("kp",        kp);
            prefs.putFloat("ki",        ki);
            prefs.putFloat("kd",        kd);
            prefs.putFloat("deadband",  deadband);
            prefs.putFloat("northTrim", northTrim);

            Serial.printf("[Settings] Saved – Kp=%.3f Ki=%.4f Kd=%.3f deadband=%.1f northTrim=%.1f\n",
                          kp, ki, kd, deadband, northTrim);
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
    // Update encoder reading (~100 Hz is sufficient for steering)
    bool encoderOk = steeringMotor.updateEncoderPosition();

    // Periodically log diagnostics to serial for debugging
    static unsigned long lastDiag = 0;
    if (millis() - lastDiag >= 500) {
        lastDiag = millis();
        Serial.printf("[Loop] encoder=%s current=%.1f target=%.1f motorCurrent=%.0f\n",
                      encoderOk ? "OK" : "FAIL",
                      steeringMotor.getCurrentAngle(),
                      steeringMotor.getTargetAngle(),
                      steeringMotor.getMotorCurrent());
    }

    // Only run PID if the encoder is responding; otherwise stop the motor
    if (encoderOk) {
        steeringMotor.recomputePID();
    } else {
        steeringMotor.stop();
    }

    // Broadcast angle data to WebSocket clients every 100 ms
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

    delay(10);  // ~100 Hz control loop
}
