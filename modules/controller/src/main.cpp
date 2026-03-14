// main.cpp – Sauna Boat Controller Module
// Plain ESP32 (not S3) — reads a PS3 DualShock 3 controller via Bluetooth
// and sends steering speed commands to the steering module via ESP-NOW.
//
// Left analog stick X-axis → motor speed (-1.0 to 1.0).
//   Full left  = full port speed.
//   Stick centre (within dead-zone) = stop.
//   Full right = full starboard speed.
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

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static bool g_ps3Connected = false;

// ---------------------------------------------------------------------------
// ESP-NOW helpers
// ---------------------------------------------------------------------------
static void sendSpeed(float speed) {
    MeshMessage msg;
    msg.type   = MSG_SET_SPEED;
    msg.src    = MODULE_CONTROLLER;
    msg.value1 = speed;
    msg.value2 = 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t /*status*/) {
    // Uncomment to debug send failures:
    // Serial.printf("[Mesh] %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ---------------------------------------------------------------------------
// PS3 callbacks
// ---------------------------------------------------------------------------
static void onPs3Connect() {
    Serial.println("[PS3] Controller connected.");
    g_ps3Connected = true;
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
    static unsigned long lastSend = 0;
    static unsigned long lastLog  = 0;
    unsigned long now = millis();

    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;

        float speed = 0.0f;

        if (g_ps3Connected) {
            // Left stick X-axis: raw range -128 to 127, centre = 0.
            int8_t rawX = Ps3.data.analog.stick.lx;

            // Apply dead-zone.
            if (rawX > -STICK_DEADZONE && rawX < STICK_DEADZONE) rawX = 0;

            // Map directly to normalised speed [-1.0, 1.0].
            speed = rawX / 127.0f;
            speed = constrain(speed, -1.0f, 1.0f);
        }
        // When the controller is disconnected, speed stays 0.0f and we keep
        // sending it so the steering watchdog doesn't time out and the motor
        // stays stopped cleanly.
        sendSpeed(speed);
    }

    // Periodic serial log for debugging.
    if (now - lastLog >= 500) {
        lastLog = now;
        if (g_ps3Connected) {
            Serial.printf("[Loop] stickX=%d  speed=%.2f\n",
                          static_cast<int>(Ps3.data.analog.stick.lx),
                          Ps3.data.analog.stick.lx / 127.0f);
        } else {
            Serial.println("[Loop] Waiting for PS3 controller…");
        }
    }

    delay(10);
}
