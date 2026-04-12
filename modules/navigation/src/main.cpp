// main.cpp – Sauna Boat Navigation Module
// Plain ESP32 — reads a BMM150 3-axis magnetometer (I2C) and NEO-6M GPS
// (UART2) and broadcasts position + heading to other modules via ESP-NOW.
//
// Wiring:
//   BMM150  SDA  → GPIO 21      NEO-6M  TX → GPIO 16  (UART2 RX)
//   BMM150  SCL  → GPIO 22      NEO-6M  RX → GPIO 17  (UART2 TX)

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <DFRobot_BMM150.h>
#include "mesh_protocol.h"

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
#define GPS_RX_PIN  16   // ESP32 UART2 RX ← NEO-6M TX
#define GPS_TX_PIN  17   // ESP32 UART2 TX → NEO-6M RX
#define GPS_BAUD    9600 // NEO-6M default baud rate

#define BMM150_I2C_ADDR  0x13  // BMM150 default I2C address

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static constexpr unsigned long NAV_BROADCAST_MS  = 500;  // 2 Hz mesh broadcast
static constexpr unsigned long DIAG_LOG_MS       = 1000; // 1 Hz serial diagnostics
static constexpr unsigned long CAL_DURATION_MS   = 15000; // calibration collection window
static constexpr unsigned long CAL_PROGRESS_MS   = 1000;  // calibration progress prints

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------
static TinyGPSPlus        gps;
static DFRobot_BMM150_I2C bmm150(&Wire, BMM150_I2C_ADDR);
static Preferences        prefs;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static float g_heading = 0.0f;   // magnetic heading in degrees [0, 360)
static bool  g_bmm150Ok = false; // magnetometer initialised successfully

// Exponential moving average (EMA) filter for heading.
// We filter in the sin/cos domain to avoid discontinuities at 0°/360°.
// Alpha = 0.0–1.0; lower = smoother but slower to respond.
static constexpr float HEADING_EMA_ALPHA = 0.15f;
static float g_emaSin = 0.0f;  // filtered sin(heading)
static float g_emaCos = 1.0f;  // filtered cos(heading) — initialised to 0° (north)

// Hard-iron calibration offsets (subtracted from raw readings before heading calc)
static float g_calOffsetX = 0.0f;
static float g_calOffsetY = 0.0f;
static float g_calOffsetZ = 0.0f;
static bool  g_calibrated = false; // true if valid calibration is loaded

// Calibration collection state
static bool          g_calActive    = false;
static unsigned long g_calStartTime = 0;
static float g_calMinX, g_calMaxX;
static float g_calMinY, g_calMaxY;
static float g_calMinZ, g_calMaxZ;

// ESP-NOW diagnostics
static uint32_t g_sendCount     = 0;
static uint32_t g_sendOkCount   = 0;
static uint32_t g_sendFailCount = 0;

// ---------------------------------------------------------------------------
// Compass calibration (hard-iron offset removal)
// ---------------------------------------------------------------------------
// Hard-iron distortion (nearby ferrous materials, PCB traces, etc.) adds a
// constant offset to each magnetometer axis.  We find it by rotating the
// sensor through all orientations and recording the min/max per axis.
// The offset for each axis is (max + min) / 2.

static void calLoadFromNVS() {
    // Returns 0.0 (uncalibrated) if keys don't exist yet.
    g_calOffsetX = prefs.getFloat("calX", 0.0f);
    g_calOffsetY = prefs.getFloat("calY", 0.0f);
    g_calOffsetZ = prefs.getFloat("calZ", 0.0f);
    g_calibrated = prefs.getBool("calOk", false);
    if (g_calibrated) {
        Serial.printf("[Cal] Loaded offsets from NVS: X=%.2f  Y=%.2f  Z=%.2f\n",
                      g_calOffsetX, g_calOffsetY, g_calOffsetZ);
    } else {
        Serial.println("[Cal] No saved calibration — heading may be inaccurate.");
        Serial.println("[Cal] Send 'c' over serial to start calibration.");
    }
}

static void calSaveToNVS() {
    prefs.putFloat("calX", g_calOffsetX);
    prefs.putFloat("calY", g_calOffsetY);
    prefs.putFloat("calZ", g_calOffsetZ);
    prefs.putBool("calOk", true);
}

static void calStart() {
    if (!g_bmm150Ok) {
        Serial.println("[Cal] ERROR: BMM150 not initialised — cannot calibrate.");
        return;
    }
    Serial.println("[Cal] ========================================");
    Serial.println("[Cal]  COMPASS CALIBRATION STARTED");
    Serial.printf( "[Cal]  Slowly rotate the sensor through all\n");
    Serial.printf( "[Cal]  orientations for %lu seconds…\n", CAL_DURATION_MS / 1000);
    Serial.println("[Cal] ========================================");

    // Seed min/max with the first reading.
    sBmm150MagData_t mag = bmm150.getGeomagneticData();
    g_calMinX = g_calMaxX = mag.x;
    g_calMinY = g_calMaxY = mag.y;
    g_calMinZ = g_calMaxZ = mag.z;

    g_calStartTime = millis();
    g_calActive    = true;
}

// Feed a new sample into the running min/max tracker.
static void calFeedSample(float x, float y, float z) {
    if (x < g_calMinX) g_calMinX = x;
    if (x > g_calMaxX) g_calMaxX = x;
    if (y < g_calMinY) g_calMinY = y;
    if (y > g_calMaxY) g_calMaxY = y;
    if (z < g_calMinZ) g_calMinZ = z;
    if (z > g_calMaxZ) g_calMaxZ = z;
}

// Finish calibration: compute offsets, save to NVS, and print results.
static void calFinish() {
    g_calActive = false;

    g_calOffsetX = (g_calMaxX + g_calMinX) / 2.0f;
    g_calOffsetY = (g_calMaxY + g_calMinY) / 2.0f;
    g_calOffsetZ = (g_calMaxZ + g_calMinZ) / 2.0f;
    g_calibrated = true;

    calSaveToNVS();

    Serial.println("[Cal] ========================================");
    Serial.println("[Cal]  CALIBRATION COMPLETE — offsets saved.");
    Serial.printf( "[Cal]  X: min=%.2f  max=%.2f  offset=%.2f\n", g_calMinX, g_calMaxX, g_calOffsetX);
    Serial.printf( "[Cal]  Y: min=%.2f  max=%.2f  offset=%.2f\n", g_calMinY, g_calMaxY, g_calOffsetY);
    Serial.printf( "[Cal]  Z: min=%.2f  max=%.2f  offset=%.2f\n", g_calMinZ, g_calMaxZ, g_calOffsetZ);
    Serial.println("[Cal] ========================================");
}

// ---------------------------------------------------------------------------
// Compass heading
// ---------------------------------------------------------------------------

// Read the BMM150 and compute a tilt-uncompensated 2-D heading.
// Applies hard-iron offset correction and EMA smoothing.
// Returns degrees [0, 360).
static float readHeading() {
    sBmm150MagData_t mag = bmm150.getGeomagneticData();

    // If calibration is running, feed the raw sample.
    if (g_calActive) {
        calFeedSample(mag.x, mag.y, mag.z);
    }

    // Apply hard-iron correction.
    float cx = mag.x - g_calOffsetX;
    float cy = mag.y - g_calOffsetY;

    float rawDeg = atan2(cx, cy) * 180.0f / PI;
    if (rawDeg < 0) rawDeg += 360.0f;

    // EMA in the sin/cos domain so the filter wraps correctly around 0°/360°.
    float rawRad = rawDeg * PI / 180.0f;
    g_emaSin = HEADING_EMA_ALPHA * sinf(rawRad) + (1.0f - HEADING_EMA_ALPHA) * g_emaSin;
    g_emaCos = HEADING_EMA_ALPHA * cosf(rawRad) + (1.0f - HEADING_EMA_ALPHA) * g_emaCos;

    float h = atan2f(g_emaSin, g_emaCos) * 180.0f / PI;
    if (h < 0) h += 360.0f;
    return h;
}

// ---------------------------------------------------------------------------
// ESP-NOW helpers
// ---------------------------------------------------------------------------
static void onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ++g_sendOkCount;
    } else {
        ++g_sendFailCount;
        Serial.println("[Mesh] WARNING: Send FAILED!");
    }
}

static void broadcastNavStatus() {
    NavigationMessage msg = {};
    msg.type = MSG_NAV_STATUS;
    msg.src  = MODULE_NAVIGATION;

    if (gps.location.isValid()) {
        msg.fixType    = 1;
        msg.lat        = (float)gps.location.lat();
        msg.lon        = (float)gps.location.lng();
    }
    if (gps.satellites.isValid()) {
        msg.satellites = (uint8_t)gps.satellites.value();
    }
    if (gps.speed.isValid()) {
        msg.speedKnots = (float)gps.speed.knots();
    }
    if (gps.course.isValid()) {
        msg.course = (float)gps.course.deg();
    }
    if (g_bmm150Ok) {
        msg.heading = g_heading;
    }

    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
    ++g_sendCount;
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Sauna Boat Navigation Module ===");

    // --- GPS (UART2) ---
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("[Setup] GPS UART2 started (RX=%d TX=%d @ %d baud)\n",
                  GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

    // --- BMM150 (I2C) ---
    Wire.begin();  // SDA=21, SCL=22 (ESP32 defaults)
    while (bmm150.begin()) {
        Serial.println("[Setup] BMM150 not found — retrying in 1 s…");
        delay(1000);
    }
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_30HZ);
    g_bmm150Ok = true;
    Serial.println("[Setup] BMM150 magnetometer initialised (30 Hz, high accuracy).");

    // --- Calibration (NVS) ---
    prefs.begin("nav", false);  // namespace "nav", read-write
    calLoadFromNVS();

    // --- WiFi + ESP-NOW ---
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

    Serial.println("[Setup] Navigation module ready — waiting for GPS fix…");
    Serial.println("[Setup] Send 'c' over serial to calibrate compass.");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    static unsigned long lastBroadcast  = 0;
    static unsigned long lastLog        = 0;
    static unsigned long lastCalProg    = 0;
    unsigned long now = millis();

    // --- Serial command handling ---
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == 'c' || ch == 'C') {
            if (g_calActive) {
                Serial.println("[Cal] Calibration already in progress…");
            } else {
                calStart();
            }
        }
    }

    // Feed all available GPS bytes into the parser.
    while (Serial2.available()) {
        gps.encode(Serial2.read());
    }

    // Update compass heading (also feeds calibration when active).
    if (g_bmm150Ok) {
        g_heading = readHeading();
    }

    // Calibration progress / completion.
    if (g_calActive) {
        unsigned long elapsed = millis() - g_calStartTime;
        if (elapsed >= CAL_DURATION_MS) {
            calFinish();
        } else if (now - lastCalProg >= CAL_PROGRESS_MS) {
            lastCalProg = now;
            unsigned long remaining = (CAL_DURATION_MS - elapsed) / 1000;
            Serial.printf("[Cal] Collecting… %lus remaining  "
                          "X=[%.1f, %.1f]  Y=[%.1f, %.1f]  Z=[%.1f, %.1f]\n",
                          remaining,
                          g_calMinX, g_calMaxX,
                          g_calMinY, g_calMaxY,
                          g_calMinZ, g_calMaxZ);
        }
    }

    // Broadcast navigation data over ESP-NOW.
    if (now - lastBroadcast >= NAV_BROADCAST_MS) {
        lastBroadcast = now;
        broadcastNavStatus();
    }

    // Periodic serial diagnostics.
    if (!g_calActive && (now - lastLog >= DIAG_LOG_MS)) {
        lastLog = now;

        const char* calTag = g_calibrated ? "CAL" : "UNCAL";

        if (gps.location.isValid()) {
            Serial.printf("[Nav] GPS: %.6f, %.6f  sats=%lu  spd=%.1fkn  crs=%.1f° | "
                          "hdg=%.1f° [%s] | mesh: sent=%lu ok=%lu fail=%lu\n",
                          gps.location.lat(), gps.location.lng(),
                          (unsigned long)gps.satellites.value(),
                          gps.speed.knots(), gps.course.deg(),
                          g_heading, calTag,
                          (unsigned long)g_sendCount,
                          (unsigned long)g_sendOkCount,
                          (unsigned long)g_sendFailCount);
        } else if (gps.charsProcessed() < 10) {
            // No meaningful data received — likely a wiring or baud rate issue.
            Serial.printf("[Nav] GPS: NO DATA (check wiring: TX→GPIO%d, baud=%d) | "
                          "hdg=%.1f° [%s] | mesh: sent=%lu ok=%lu fail=%lu\n",
                          GPS_RX_PIN, GPS_BAUD,
                          g_heading, calTag,
                          (unsigned long)g_sendCount,
                          (unsigned long)g_sendOkCount,
                          (unsigned long)g_sendFailCount);
        } else {
            // Receiving NMEA sentences but no position fix yet.
            Serial.printf("[Nav] GPS: WAITING FOR FIX  sats=%lu  sentences=%lu  "
                          "failed=%lu | hdg=%.1f° [%s] | mesh: sent=%lu ok=%lu fail=%lu\n",
                          (unsigned long)gps.satellites.value(),
                          gps.sentencesWithFix(),
                          gps.failedChecksum(),
                          g_heading, calTag,
                          (unsigned long)g_sendCount,
                          (unsigned long)g_sendOkCount,
                          (unsigned long)g_sendFailCount);
        }
    }

    delay(10);
}
