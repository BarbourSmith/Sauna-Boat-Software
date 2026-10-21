// main.cpp – Sauna Boat Steering System
// ESP32-S3 firmware driving an RC servo via GPIO 48.
//
// Hardware:
//   Servo signal pin : GPIO 48
//   LEDC channel     : 6
//
// Mesh: accepts MSG_CONTROLLER_INPUT from input modules over ESP-NOW.
// Raw stick and button state is received here; this module applies dead-zone
// and interprets D-pad left/right as trim jog commands.
// The radio is fixed on MESH_WIFI_CHANNEL and joins no WiFi network.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include "MotorUnit.h"
#include "mesh_protocol.h"

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

// Trim state – D-pad left/right nudge the servo centre offset while held.
// The value is applied inside MotorUnit::updateRamp() as a position offset.
// g_trimDirty is set whenever trim changes; the loop saves it to NVS once
// the buttons have been released for TRIM_SAVE_DEBOUNCE_MS to avoid flash wear.
static uint16_t       g_prevButtons        = 0;
static unsigned long  g_trimLastChange     = 0;
static bool           g_trimDirty         = false;

// Dead-zone applied to raw left-stick X from MSG_CONTROLLER_INPUT.
// Sticks naturally drift; anything within this band is treated as centred.
static constexpr float STICK_DEADZONE = 10.0f / 127.0f;  // ~8% of full range
static constexpr float HANDHELD_OVERRIDE_ENTER = 0.20f;
static constexpr float HANDHELD_OVERRIDE_EXIT = 0.10f;

// Trim step applied each 50 ms frame while the D-pad button is held.
// 0.01 ≈ 10 µs of servo pulse width; at 20 Hz this gives ~0.2 normalised/sec.
static constexpr float TRIM_STEP = 0.01f;

// How long (ms) to wait after the last trim change before saving to NVS.
// Prevents flash wear when the button is held for many frames.
static constexpr unsigned long TRIM_SAVE_DEBOUNCE_MS = 500;

// If heading-hold status goes stale, fall back to manual control.
static constexpr unsigned long HEADING_HOLD_STATUS_TIMEOUT_MS = 1500;
static constexpr unsigned long HANDHELD_INPUT_TIMEOUT_MS = 300;

// Navigation module heading-hold state broadcast over MSG_HEADING_HOLD_STATUS.
static bool           g_headingHoldActive = false;
static unsigned long  g_lastHeadingHoldStatusMs = 0;
static float          g_lastHandheldLx = 0.0f;
static unsigned long  g_lastHandheldInputMs = 0;
static bool           g_handheldOverrideActive = false;

// ---------------------------------------------------------------------------
// ESP-NOW receive callback – runs in the WiFi driver task context.
// All 32-bit-aligned globals written here are updated atomically on ESP32.
// ---------------------------------------------------------------------------
static void onMeshReceive(const uint8_t* /*mac*/, const uint8_t* data, int len) {
    if (len < 1) return;
    uint8_t type = data[0];

    bool holdActive = g_headingHoldActive;
    if (holdActive && (millis() - g_lastHeadingHoldStatusMs > HEADING_HOLD_STATUS_TIMEOUT_MS)) {
        holdActive = false;
        g_headingHoldActive = false;
    }

    if (type == MSG_CONTROLLER_INPUT) {
        if (len < static_cast<int>(sizeof(ControllerInputMessage))) return;
        ControllerInputMessage msg;
        memcpy(&msg, data, sizeof(msg));

        if (msg.src == MODULE_HANDHELD) {
            g_lastHandheldLx = msg.lx;
            g_lastHandheldInputMs = millis();
        }

        // In heading hold: only meaningful handheld stick deflection should override.
        float lx = msg.lx;
        if (holdActive) {
            bool handheldInputFresh =
                (millis() - g_lastHandheldInputMs) <= HANDHELD_INPUT_TIMEOUT_MS;
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
            if (!g_handheldOverrideActive) return;
        }

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
        if (lx > -STICK_DEADZONE && lx < STICK_DEADZONE) lx = 0.0f;
        float speed = constrain(lx, -1.0f, 1.0f);

        steeringMotor.setSpeed(speed);
        g_currentSpeed = speed;

    } else if (type == MSG_SET_STEERING) {
        // Legacy fallback: accept pre-computed speed from any sender.
        if (len < static_cast<int>(sizeof(MeshMessage))) return;
        MeshMessage msg;
        memcpy(&msg, data, sizeof(msg));

        // In heading hold: let handheld override only when it sends non-straight steering.
        if (holdActive && msg.src == MODULE_HANDHELD) {
            bool handheldInputFresh =
                (millis() - g_lastHandheldInputMs) <= HANDHELD_INPUT_TIMEOUT_MS;
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
            if (!g_handheldOverrideActive) return;
        } else if (holdActive && msg.src != MODULE_NAVIGATION) {
            return;
        }

        float speed = constrain(msg.value1, -1.0f, 1.0f);
        steeringMotor.setSpeed(speed);
        g_currentSpeed = speed;

    } else if (type == MSG_HEADING_HOLD_STATUS) {
        if (len < static_cast<int>(sizeof(MeshMessage))) return;
        MeshMessage msg;
        memcpy(&msg, data, sizeof(msg));
        if (msg.src != MODULE_NAVIGATION) return;

        g_headingHoldActive = (msg.value1 > 0.5f);
        g_lastHeadingHoldStatusMs = millis();
    }
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

    // --- NVS: load persisted settings ---
    prefs.begin("steering", false);
    // "maxTravel" replaces the old "maxSpeed" key (which stored the DC-motor PWM
    // limit defaulting to 512).  Using a new key forces the correct default of
    // 1023 (full servo range) on existing devices that have the old value stored.
    steeringMotor.setMaxSpeed(prefs.getInt("maxTravel", 1023));
    // rampMs: time in ms to ramp from 0 to full deflection (0 = instant).
    int rampMs = prefs.getInt("rampMs", 500);
    steeringMotor.setRampRate(rampMs > 0 ? 1000.0f / rampMs : 0.0f);
    // trim: persistent servo centre offset set via D-pad buttons on the controller.
    steeringMotor.setTrim(prefs.getFloat("trim", 0.0f));
    Serial.printf("[Setup] Settings loaded – maxTravel=%d rampMs=%d trim=%.4f\n",
                  steeringMotor.getMaxSpeed(), rampMs, steeringMotor.getTrim());

    // --- WiFi + ESP-NOW (mesh receive) ---
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(MESH_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[Setup] WiFi MAC: %s  (channel %d)\n",
                  WiFi.macAddress().c_str(), MESH_WIFI_CHANNEL);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Serial.println("[Setup] ESP-NOW initialised.");
        esp_now_register_recv_cb(onMeshReceive);
    }
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    static bool lastHoldState = false;

    if (g_headingHoldActive &&
        (millis() - g_lastHeadingHoldStatusMs > HEADING_HOLD_STATUS_TIMEOUT_MS)) {
        g_headingHoldActive = false;
    }

    if (g_headingHoldActive != lastHoldState) {
        Serial.printf("[Hold] %s\n", g_headingHoldActive ? "ACTIVE" : "INACTIVE");
        lastHoldState = g_headingHoldActive;
    }

    // Trim NVS save: write to flash once the D-pad has been released for
    // TRIM_SAVE_DEBOUNCE_MS, avoiding repeated writes while the button is held.
    if (g_trimDirty && (millis() - g_trimLastChange) >= TRIM_SAVE_DEBOUNCE_MS) {
        prefs.putFloat("trim", steeringMotor.getTrim());
        g_trimDirty = false;
        Serial.printf("[Trim] Saved to NVS: %.4f\n", steeringMotor.getTrim());
    }

    // Periodically log diagnostics to serial for debugging
    static unsigned long lastDiag = 0;
    if (millis() - lastDiag >= 500) {
        lastDiag = millis();
        Serial.printf("[Loop] speed=%.2f\n", g_currentSpeed);
    }

    // Step the servo position toward the commanded speed at the configured ramp rate
    steeringMotor.updateRamp();

    delay(10);  // ~100 Hz loop
}
