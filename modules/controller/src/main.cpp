// main.cpp – Sauna Boat Controller Module
// Plain ESP32 (not S3) — reads a PS3 DualShock 3 controller via Bluetooth
// and relays every raw input to other modules via ESP-NOW (MSG_CONTROLLER_INPUT).
//
// The controller applies NO dead-zone and NO processing — it forwards raw stick
// values and button states every SEND_INTERVAL_MS.  Receiving modules (e.g. the
// steering module) decide how to interpret the inputs.
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

// How often to broadcast a controller-input packet (ms).
// Must be shorter than the steering module's watchdog (200 ms).
static constexpr unsigned long SEND_INTERVAL_MS = 50;

// How often to broadcast controller status (battery level, charging).
static constexpr unsigned long BATTERY_INTERVAL_MS = 5000;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static bool     g_ps3Connected  = false;

// ESP-NOW send diagnostics
static uint32_t g_sendCount     = 0;
static uint32_t g_sendOkCount   = 0;
static uint32_t g_sendFailCount = 0;

// ---------------------------------------------------------------------------
// ESP-NOW helpers
// ---------------------------------------------------------------------------

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

// Send a raw steering command so the system works even without the navigation
// module.  No dead zone is applied — the receiving module decides.
// When the navigation module IS present and heading hold is active, it will
// also send MSG_SET_STEERING — the steering module prioritises the nav module's
// commands (see steering onMeshReceive).
static void sendSteering(float stickX) {
    MeshMessage msg;
    msg.type   = MSG_SET_STEERING;
    msg.src    = MODULE_CONTROLLER;
    msg.value1 = constrain(stickX, -1.0f, 1.0f);
    msg.value2 = 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

// Broadcast the raw PS3 controller state.  When the controller is disconnected
// all fields are zero, which keeps the steering watchdog alive and stops motion.
static void sendControllerInput() {
    ControllerInputMessage msg = {};
    msg.type = MSG_CONTROLLER_INPUT;
    msg.src  = MODULE_CONTROLLER;

    if (g_ps3Connected) {
        // Normalise raw int8 values to [-1.0, 1.0].  No dead-zone is applied
        // here — the receiving module applies its own.
        msg.lx = constrain(Ps3.data.analog.stick.lx / 127.0f, -1.0f, 1.0f);
        msg.ly = constrain(Ps3.data.analog.stick.ly / 127.0f, -1.0f, 1.0f);
        msg.rx = constrain(Ps3.data.analog.stick.rx / 127.0f, -1.0f, 1.0f);
        msg.ry = constrain(Ps3.data.analog.stick.ry / 127.0f, -1.0f, 1.0f);

        uint16_t btns = 0;
        if (Ps3.data.button.cross)    btns |= CTRL_BTN_CROSS;
        if (Ps3.data.button.circle)   btns |= CTRL_BTN_CIRCLE;
        if (Ps3.data.button.square)   btns |= CTRL_BTN_SQUARE;
        if (Ps3.data.button.triangle) btns |= CTRL_BTN_TRIANGLE;
        if (Ps3.data.button.l1)       btns |= CTRL_BTN_L1;
        if (Ps3.data.button.r1)       btns |= CTRL_BTN_R1;
        if (Ps3.data.button.l2)       btns |= CTRL_BTN_L2;
        if (Ps3.data.button.r2)       btns |= CTRL_BTN_R2;
        if (Ps3.data.button.l3)       btns |= CTRL_BTN_L3;
        if (Ps3.data.button.r3)       btns |= CTRL_BTN_R3;
        if (Ps3.data.button.up)       btns |= CTRL_BTN_UP;
        if (Ps3.data.button.down)     btns |= CTRL_BTN_DOWN;
        if (Ps3.data.button.left)     btns |= CTRL_BTN_LEFT;
        if (Ps3.data.button.right)    btns |= CTRL_BTN_RIGHT;
        if (Ps3.data.button.start)    btns |= CTRL_BTN_START;
        if (Ps3.data.button.select)   btns |= CTRL_BTN_SELECT;
        msg.buttons = btns;
    }
    // Disconnected: all fields remain zero (zeroed by `= {}`).

    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
    ++g_sendCount;
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
    sendBattery();
}

static void onPs3Disconnect() {
    Serial.println("[PS3] Controller disconnected — sending zero input.");
    g_ps3Connected = false;
    // Send an immediate zero-input packet so the steering module stops without
    // waiting up to SEND_INTERVAL_MS.
    sendControllerInput();
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Sauna Boat Controller Module ===");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(MESH_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[Setup] WiFi STA MAC: %s  (channel %d)\n",
                  WiFi.macAddress().c_str(), MESH_WIFI_CHANNEL);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Serial.println("[Setup] ESP-NOW initialised.");
    }
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, MESH_BROADCAST_ADDR, 6);
    peer.channel = MESH_WIFI_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.println("[Setup] Broadcast peer registered.");
    } else {
        Serial.println("[Setup] WARNING: Failed to register broadcast peer.");
    }

    Ps3.attachOnConnect(onPs3Connect);
    Ps3.attachOnDisconnect(onPs3Disconnect);
    Ps3.begin();

    Serial.printf("[Setup] Bluetooth MAC: %s\n", Ps3.getAddress().c_str());
    Serial.println("[Setup] Waiting for PS3 controller…");
    Serial.println("[Setup] (Pair the controller with the above MAC via SixaxisPairTool first.)");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    static unsigned long lastSend    = 0;
    static unsigned long lastLog     = 0;
    static unsigned long lastBattery = 0;
    unsigned long now = millis();

    // Broadcast raw controller input + direct steering speed at SEND_INTERVAL_MS.
    // MSG_CONTROLLER_INPUT goes to the nav module (if present) for heading hold.
    // MSG_SET_STEERING goes directly to the steering module as a fallback.
    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;
        sendControllerInput();
        float lx = g_ps3Connected
            ? constrain(Ps3.data.analog.stick.lx / 127.0f, -1.0f, 1.0f)
            : 0.0f;
        sendSteering(lx);
    }

    // Battery status broadcast (every 5 s while PS3 is connected).
    if (g_ps3Connected && (now - lastBattery >= BATTERY_INTERVAL_MS)) {
        lastBattery = now;
        sendBattery();
    }

    // Periodic serial diagnostics.
    if (now - lastLog >= 500) {
        lastLog = now;
        if (g_ps3Connected) {
            Serial.printf("[Loop] PS3=OK  lx=%.2f ly=%.2f rx=%.2f ry=%.2f | "
                          "mesh: sent=%lu ok=%lu fail=%lu\n",
                          Ps3.data.analog.stick.lx / 127.0f,
                          Ps3.data.analog.stick.ly / 127.0f,
                          Ps3.data.analog.stick.rx / 127.0f,
                          Ps3.data.analog.stick.ry / 127.0f,
                          (unsigned long)g_sendCount,
                          (unsigned long)g_sendOkCount,
                          (unsigned long)g_sendFailCount);
        } else {
            Serial.printf("[Loop] PS3=WAITING | mesh: sent=%lu ok=%lu fail=%lu\n",
                          (unsigned long)g_sendCount,
                          (unsigned long)g_sendOkCount,
                          (unsigned long)g_sendFailCount);
        }
    }

    delay(10);
}
