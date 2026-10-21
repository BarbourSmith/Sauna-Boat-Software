// main.cpp – Sauna Boat Steering System
// ESP32-S3 firmware driving an RC servo via GPIO 48.
//
// Hardware:
//   Servo signal pin : GPIO 48
//   LEDC channel     : 6
//
// Logs, serial commands and firmware updates also go over the mesh, through
// the handheld's USB port (see mesh_remote.h).
//
// Mesh: accepts MSG_SET_STEERING commands from any module over ESP-NOW.
// The radio is fixed on MESH_WIFI_CHANNEL and joins no WiFi network.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include "MotorUnit.h"
#include "mesh_protocol.h"
#include "mesh_remote.h"

// ---------------------------------------------------------------------------
// Steering servo pin definitions
// ---------------------------------------------------------------------------
#define SERVO_PIN  48   // RC servo signal wire

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static MotorUnit      steeringMotor;
static Preferences    prefs;

// Most recent normalized speed command received over the mesh (-1.0 to 1.0).
static float          g_currentSpeed = 0.0f;

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
static constexpr float HANDHELD_OVERRIDE_ENTER = 0.20f;
static constexpr float HANDHELD_OVERRIDE_EXIT = 0.10f;
static constexpr unsigned long HANDHELD_INPUT_TIMEOUT_MS = 300;
static float g_lastHandheldLx = 0.0f;
static unsigned long g_lastHandheldInputMs = 0;
static bool g_handheldOverrideActive = false;

// ---------------------------------------------------------------------------
// ESP-NOW receive callback – runs in the WiFi driver task context.
// Handles MSG_SET_STEERING from any module, with nav module taking priority.
// Updating g_currentSpeed and g_lastCmdTime here is safe: both are 32-bit
// aligned values and the ESP32 performs 32-bit loads/stores atomically.
// ---------------------------------------------------------------------------
static void onMeshReceive(const uint8_t* /*mac*/, const uint8_t* data, int len) {
    if (meshRemoteReceive(data, len)) return;
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
            float absLx = fabsf(g_lastHandheldLx);
            if (handheldInputFresh) {
                if (g_handheldOverrideActive) {
                    g_handheldOverrideActive = absLx >= HANDHELD_OVERRIDE_EXIT;
                } else {
                    g_handheldOverrideActive = absLx >= HANDHELD_OVERRIDE_ENTER;
                }
            } else {
                g_handheldOverrideActive = false;
            }

            // While nav is active, only suppress handheld when the stick is straight/idle.
            if ((now - g_lastNavCmdTime) < NAV_PRIORITY_MS && !g_handheldOverrideActive) {
                return;  // ignore idle handheld input — nav keeps priority
            }
        }

        float speed = constrain(msg.value1, -1.0f, 1.0f);
        steeringMotor.setSpeed(speed);
        g_currentSpeed = speed;
    }
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Log.println("\n=== Sauna Boat Steering System ===");

    // --- Steering servo ---
    steeringMotor.begin(SERVO_PIN);

    // --- NVS: load persisted settings ---
    prefs.begin("steering", false);
    // "maxTravel" replaces the old "maxSpeed" key (which stored the DC-motor PWM
    // limit defaulting to 512).  Using a new key forces the correct default of
    // 1023 (full servo range) on existing devices that have the old value stored.
    steeringMotor.setMaxSpeed(prefs.getInt("maxTravel", 1023));
    // rampMs: time in ms to ramp from 0 to full deflection (0 = instant).
    int rampMs = prefs.getInt("rampMs", 500);
    steeringMotor.setRampRate(rampMs > 0 ? 1000.0f / rampMs : 0.0f);
    // dirReverse: invert servo direction.
    steeringMotor.setReversed(prefs.getBool("dirReverse", false));
    // trimUs: servo center trim offset in µs (-500 to +500).
    steeringMotor.setTrimUs(prefs.getInt("trimUs", 0));
    Log.printf("[Setup] Settings loaded – maxTravel=%d rampMs=%d dirReverse=%d trimUs=%d\n",
               steeringMotor.getMaxSpeed(), rampMs,
               steeringMotor.getReversed(), steeringMotor.getTrimUs());

    // --- WiFi + ESP-NOW (mesh receive) ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(MESH_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Log.printf("[Setup] WiFi MAC: %s  (channel %d)\n",
               WiFi.macAddress().c_str(), MESH_WIFI_CHANNEL);

    if (esp_now_init() != ESP_OK) {
        Log.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Log.println("[Setup] ESP-NOW initialised.");
        meshRemoteBegin(MODULE_STEERING);
        esp_now_register_recv_cb(onMeshReceive);
    }
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    if (meshRemoteLoop()) return;  // receiving new firmware: hold the rudder where it is

    // Periodically log diagnostics to serial for debugging
    static unsigned long lastDiag = 0;
    if (millis() - lastDiag >= 500) {
        lastDiag = millis();
        Log.printf("[Loop] speed=%.2f\n", g_currentSpeed);
    }

    // Step the servo position toward the commanded speed at the configured ramp rate
    steeringMotor.updateRamp();

    delay(10);  // ~100 Hz loop
}
