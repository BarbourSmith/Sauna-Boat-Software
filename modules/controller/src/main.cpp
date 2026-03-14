// main.cpp – Sauna Boat Controller Module
// Plain ESP32 (not S3) — reads a PS3 DualShock 3 controller via Bluetooth
// and sends steering angle commands to the steering module via ESP-NOW.
//
// Left analog stick X-axis = turn rate (proportional to deflection).
// Full left  → HEADING_RATE_DEG_S left per second.
// Full right → HEADING_RATE_DEG_S right per second.
// Stick centred → hold current heading command.
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

// Maximum heading change rate when the stick is fully deflected (degrees/sec).
static constexpr float HEADING_RATE_DEG_S = 90.0f;

// Raw stick dead-zone (units, centred at 0, range ±128).
static constexpr int STICK_DEADZONE = 10;

// How often to send a heading update (ms). 20 Hz is plenty for a boat.
static constexpr unsigned long SEND_INTERVAL_MS = 50;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static float g_commandedAngle = 0.0f;  // current heading command, 0–360°
static bool  g_ps3Connected   = false;

// ---------------------------------------------------------------------------
// ESP-NOW helpers
// ---------------------------------------------------------------------------
static void sendAngle(float angle) {
    MeshMessage msg;
    msg.type   = MSG_SET_ANGLE;
    msg.src    = MODULE_CONTROLLER;
    msg.value1 = angle;
    msg.value2 = 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    // Uncomment for debugging:
    // Serial.printf("[Mesh] Send status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ---------------------------------------------------------------------------
// PS3 callbacks
// ---------------------------------------------------------------------------
static void onPs3Connect() {
    Serial.println("[PS3] Controller connected.");
    g_ps3Connected = true;
}

static void onPs3Disconnect() {
    Serial.println("[PS3] Controller disconnected.");
    g_ps3Connected = false;
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Sauna Boat Controller Module ===");

    // --- WiFi: STA mode required for ESP-NOW ---
    // We do not associate with any AP; we only need the radio on the right channel.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Both modules must operate on the same channel.
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

    // Register the broadcast address as a peer so we can send to it.
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
    static unsigned long lastSend = 0;
    static unsigned long lastLog  = 0;
    unsigned long now = millis();

    if (g_ps3Connected && (now - lastSend >= SEND_INTERVAL_MS)) {
        float dt = (now - lastSend) / 1000.0f;
        lastSend = now;

        // Left stick X-axis: raw range is -128 to 127, centre = 0.
        int8_t rawX = Ps3.data.analog.stick.lx;

        // Apply dead-zone.
        if (rawX > -STICK_DEADZONE && rawX < STICK_DEADZONE) rawX = 0;

        // Map to a normalised rate (-1.0 to 1.0) and accumulate heading.
        float rate = (rawX / 127.0f) * HEADING_RATE_DEG_S;
        g_commandedAngle = fmodf(g_commandedAngle + rate * dt + 360.0f, 360.0f);

        sendAngle(g_commandedAngle);
    }

    if (!g_ps3Connected) {
        // Reset timer so dt is sensible on reconnect.
        lastSend = millis();
    }

    // Periodic serial log (every 500 ms) for debugging.
    if (now - lastLog >= 500) {
        lastLog = now;
        if (g_ps3Connected) {
            Serial.printf("[Loop] stickX=%d  commanded=%.1f°\n",
                          static_cast<int>(Ps3.data.analog.stick.lx),
                          g_commandedAngle);
        } else {
            Serial.println("[Loop] Waiting for PS3 controller…");
        }
    }

    delay(10);
}
