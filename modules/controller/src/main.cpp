// main.cpp – Sauna Boat Controller Module
// Plain ESP32 (not S3) — reads a PS3 DualShock 3 controller via Bluetooth
// and sends steering speed commands to the steering module via ESP-NOW.
//
// Left analog stick X-axis → motor speed (-1.0 to 1.0).
//   Full left  = full port speed.
//   Stick centre (within dead-zone) = stop.
//   Full right = full starboard speed.
//
// D-pad left / right → jog: sends full speed in that direction for
//   JOG_DURATION_MS milliseconds, then returns to stick control.
//   Useful for making small, repeatable trim adjustments.
//
// The steering module's existing speed ramp and max-speed settings still
// apply, so the feel can be tuned from the phone UI without reflashing.
//
// Pairing (one-time):
//   Flash this firmware, note the "Bluetooth MAC" on the serial monitor,
//   then use SixaxisPairTool to write it into your PS3 controller.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Ps3Controller.h>
#include "mesh_protocol.h"

// ---------------------------------------------------------------------------
// Tuning
// ---------------------------------------------------------------------------

// Raw stick dead-zone (units centred at 0, range ±128).
// Increase if the motor creeps when the stick is released.
static constexpr int STICK_DEADZONE = 10;

// How often to send a speed update (ms).
// Must be shorter than the steering module's watchdog (200 ms).
static constexpr unsigned long SEND_INTERVAL_MS = 50;

// How long a D-pad jog pulse lasts (ms).
// The steering module's ramp setting applies, so the motor will ramp up
// then back down — giving a smooth, repeatable trim nudge.
static constexpr unsigned long JOG_DURATION_MS = 200;

// How often to broadcast controller status (battery level, charging).
// Battery changes slowly so 5 s is more than frequent enough.
static constexpr unsigned long BATTERY_INTERVAL_MS = 5000;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static bool          g_ps3Connected  = false;

// ESP-NOW send diagnostics
static uint32_t      g_sendCount     = 0;   // total packets sent
static uint32_t      g_sendOkCount   = 0;   // successful sends
static uint32_t      g_sendFailCount = 0;   // failed sends
static float         g_lastSentSpeed = 0.0f;

// D-pad jog state
// g_jogUntil: millis() timestamp after which the jog is over (0 = no jog active).
// g_jogSpeed: normalised speed to send during the jog (-1.0 or +1.0).
static unsigned long g_jogUntil = 0;
static float         g_jogSpeed = 0.0f;

// ---------------------------------------------------------------------------
// ESP-NOW helpers
// ---------------------------------------------------------------------------

// Map the PS3 battery enum to a simple 0-4 integer level.
// Returns -1 if the value is unrecognised.
static int ps3BatteryLevel() {
    switch (Ps3.data.status.battery) {
        case ps3_status_battery_shutdown: return 0;
        case ps3_status_battery_dying:    return 1;
        case ps3_status_battery_low:      return 2;
        case ps3_status_battery_high:     return 3;
        case ps3_status_battery_full:     return 4;
        default:                          return -1;
    }
}

// ps3_status_battery_charging (0xEE) means the controller is plugged in.
static bool ps3IsCharging() {
    return Ps3.data.status.battery == ps3_status_battery_charging;
}

static void sendBattery() {
    int  level    = ps3BatteryLevel();
    bool charging = ps3IsCharging();
    MeshMessage msg;
    msg.type   = MSG_CONTROLLER_STATUS;
    msg.src    = MODULE_CONTROLLER;
    msg.value1 = static_cast<float>(level);
    msg.value2 = charging ? 1.0f : 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
    Serial.printf("[Battery] level=%d  charging=%s\n", level, charging ? "yes" : "no");
}

static void sendSpeed(float speed) {
    MeshMessage msg;
    msg.type   = MSG_SET_SPEED;
    msg.src    = MODULE_CONTROLLER;
    msg.value1 = speed;
    msg.value2 = 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
    ++g_sendCount;
    g_lastSentSpeed = speed;
}

static void onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ++g_sendOkCount;
    } else {
        ++g_sendFailCount;
        Serial.println("[Mesh] WARNING: Send FAILED!");
    }
}

// ---------------------------------------------------------------------------
// PS3 callbacks
// ---------------------------------------------------------------------------
static void onPs3Connect() {
    Serial.println("[PS3] Controller connected.");
    g_ps3Connected = true;
    // Send an immediate battery report so the UI updates straight away
    // rather than waiting up to BATTERY_INTERVAL_MS.
    sendBattery();
}

static void onPs3Disconnect() {
    Serial.println("[PS3] Controller disconnected — sending stop.");
    g_ps3Connected = false;
    // Immediately command zero speed so the motor doesn't coast.
    sendSpeed(0.0f);
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Sauna Boat Controller Module ===");

    // --- WiFi: STA mode required for ESP-NOW ---
    // We don't associate with any AP; we only need the radio on the right channel.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Both modules must operate on the same channel as the steering AP.
    esp_wifi_set_channel(MESH_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[Setup] WiFi STA MAC: %s  (channel %d)\n",
                  WiFi.macAddress().c_str(), MESH_WIFI_CHANNEL);

    // --- ESP-NOW ---
    if (esp_now_init() != ESP_OK) {
        Serial.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Serial.println("[Setup] ESP-NOW initialised.");
    }
    esp_now_register_send_cb(onDataSent);

    // Register the broadcast address as a peer.
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, MESH_BROADCAST_ADDR, 6);
    peer.channel = MESH_WIFI_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.println("[Setup] Broadcast peer registered.");
    } else {
        Serial.println("[Setup] WARNING: Failed to register broadcast peer.");
    }

    // --- PS3 controller (Bluetooth) ---
    Ps3.attachOnConnect(onPs3Connect);
    Ps3.attachOnDisconnect(onPs3Disconnect);
    Ps3.begin();

    Serial.printf("[Setup] Bluetooth MAC: %s\n", Ps3.getAddress().c_str());
    Serial.println("[Setup] Waiting for PS3 controller…");
    Serial.println("[Setup] Press the PS button on the controller to connect.");
    Serial.println("[Setup] (Pair the controller with the above MAC via SixaxisPairTool first.)");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    static unsigned long lastSend    = 0;
    static unsigned long lastLog     = 0;
    static unsigned long lastBattery = 0;
    static bool          prevLeft    = false;  // D-pad rising-edge detection
    static bool          prevRight   = false;
    unsigned long now = millis();

    // --- D-pad jog detection (checked every loop tick for low latency) ---
    if (g_ps3Connected) {
        bool curLeft  = Ps3.data.button.left;
        bool curRight = Ps3.data.button.right;

        if (curLeft && !prevLeft) {
            // Left arrow just pressed — jog port (negative speed)
            g_jogSpeed = -1.0f;
            g_jogUntil = now + JOG_DURATION_MS;
            Serial.printf("[Jog] LEFT  → speed=%.1f for %lums\n", g_jogSpeed, JOG_DURATION_MS);
        }
        if (curRight && !prevRight) {
            // Right arrow just pressed — jog starboard (positive speed)
            g_jogSpeed = 1.0f;
            g_jogUntil = now + JOG_DURATION_MS;
            Serial.printf("[Jog] RIGHT → speed=%.1f for %lums\n", g_jogSpeed, JOG_DURATION_MS);
        }

        prevLeft  = curLeft;
        prevRight = curRight;
    } else {
        // Reset edge state when disconnected so re-connection doesn't
        // trigger a spurious jog from a button held at connect time.
        prevLeft  = false;
        prevRight = false;
    }

    // --- Speed send (50 ms cadence) ---
    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;

        float speed = 0.0f;

        if (g_ps3Connected) {
            if (now < g_jogUntil) {
                // D-pad jog is active — override stick with full jog speed.
                speed = g_jogSpeed;
            } else {
                // Normal stick control.
                int8_t rawX = Ps3.data.analog.stick.lx;

                // Apply dead-zone.
                if (rawX > -STICK_DEADZONE && rawX < STICK_DEADZONE) rawX = 0;

                // Map directly to normalised speed [-1.0, 1.0].
                speed = rawX / 127.0f;
                speed = constrain(speed, -1.0f, 1.0f);
            }
        }
        // When the controller is disconnected, speed stays 0.0f and we keep
        // sending it so the steering watchdog doesn't time out and the motor
        // stays stopped cleanly.
        sendSpeed(speed);
    }

    // --- Battery status broadcast (every 5 s while PS3 is connected) ---
    if (g_ps3Connected && (now - lastBattery >= BATTERY_INTERVAL_MS)) {
        lastBattery = now;
        sendBattery();
    }

    // Periodic serial log for debugging.
    if (now - lastLog >= 500) {
        lastLog = now;
        if (g_ps3Connected) {
            Serial.printf("[Loop] PS3=OK  stickX=%d  speed=%.2f | "
                          "mesh: sent=%lu ok=%lu fail=%lu  lastSpeed=%.2f\n",
                          static_cast<int>(Ps3.data.analog.stick.lx),
                          Ps3.data.analog.stick.lx / 127.0f,
                          (unsigned long)g_sendCount,
                          (unsigned long)g_sendOkCount,
                          (unsigned long)g_sendFailCount,
                          g_lastSentSpeed);
        } else {
            Serial.printf("[Loop] PS3=WAITING | "
                          "mesh: sent=%lu ok=%lu fail=%lu  lastSpeed=%.2f\n",
                          (unsigned long)g_sendCount,
                          (unsigned long)g_sendOkCount,
                          (unsigned long)g_sendFailCount,
                          g_lastSentSpeed);
        }
    }

    delay(10);
}
