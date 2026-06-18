// main.cpp – Sauna Boat Navigation Module
// Plain ESP32 — reads a BMM150 3-axis magnetometer (I2C) and NEO-6M GPS
// (UART2) and broadcasts position + heading to other modules via ESP-NOW.
//
// Web UI at http://192.168.4.1 — connect to WiFi AP "SaunaBoatNav".
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
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
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
// WiFi Access Point credentials
// ---------------------------------------------------------------------------
static const char* AP_SSID = "SaunaBoatNav";
static const char* AP_PASS = "12345678";

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static constexpr unsigned long NAV_BROADCAST_MS  = 500;   // 2 Hz mesh broadcast
static constexpr unsigned long DIAG_LOG_MS       = 1000;  // 1 Hz serial diagnostics
static constexpr unsigned long CAL_DURATION_MS   = 15000; // calibration collection window
static constexpr unsigned long CAL_PROGRESS_MS   = 1000;  // calibration progress prints
static constexpr unsigned long AUTOPILOT_MS      = 200;   // 5 Hz autopilot compute rate
static constexpr unsigned long AP_SEND_MS        = 50;    // 20 Hz send rate — must be well under the steering watchdog (200 ms)
static constexpr unsigned long WS_BROADCAST_MS   = 200;   // 5 Hz WebSocket status broadcast
static constexpr float         STICK_OVERRIDE_ENTER = 0.20f; // enter manual-yield above this magnitude
static constexpr float         STICK_OVERRIDE_EXIT  = 0.10f; // remain in manual-yield until below this magnitude
static constexpr unsigned long CTRL_INPUT_TIMEOUT_MS = 1000; // controller input considered stale after this
static constexpr unsigned long HOLD_RESUME_DELAY_MS = 300; // stick must remain centered this long before hold resumes
static constexpr unsigned long ROUTE_GPS_STALE_MS = 3000;

// ---------------------------------------------------------------------------
// Heading-hold autopilot tuning (PD controller) — loaded from NVS
// ---------------------------------------------------------------------------
static float g_apKp         = 0.008f;  // proportional gain
static float g_apKi         = 0.003f;  // integral gain
static float g_apKd         = 0.12f;   // derivative gain
static float g_apMaxOutput  = 1.0f;    // max steering command magnitude [0, 1]
static float g_apDeadband   = 3.0f;    // ignore errors smaller than this (degrees)
static float g_apRateLimit  = 0.05f;   // max change in output per autopilot tick

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------
static TinyGPSPlus        gps;
static DFRobot_BMM150_I2C bmm150(&Wire, BMM150_I2C_ADDR);
static Preferences        prefs;
static AsyncWebServer     server(80);
static AsyncWebSocket     ws("/ws");

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static float g_heading = 0.0f;   // magnetic heading in degrees [0, 360)
static bool  g_bmm150Ok = false; // magnetometer initialised successfully

// Exponential moving average (EMA) filter for heading.
static constexpr float HEADING_EMA_ALPHA = 0.15f;
static float g_emaSin = 0.0f;
static float g_emaCos = 1.0f;

// Hard-iron calibration offsets
static float g_calOffsetX = 0.0f;
static float g_calOffsetY = 0.0f;
static float g_calOffsetZ = 0.0f;
static bool  g_calibrated = false;

// Calibration collection state
static bool          g_calActive    = false;
static unsigned long g_calStartTime = 0;
static float g_calMinX, g_calMaxX;
static float g_calMinY, g_calMaxY;
static float g_calMinZ, g_calMaxZ;

// Heading hold state
static bool          g_headingHoldActive = false;
static bool          g_headingHoldYielding = false;
static unsigned long g_lastManualOverrideMs = 0;
static float         g_targetHeading     = 0.0f;
static float         g_apOutput          = 0.0f;
static float         g_prevError         = 0.0f;
static float         g_integral          = 0.0f;   // accumulated integral of error
static unsigned long g_prevErrorTime     = 0;

// Latest controller input
static float    g_stickLX    = 0.0f;
static uint16_t g_ctrlBtns   = 0;
static uint16_t g_prevBtns   = 0;
static bool     g_ctrlOnline = false;
static unsigned long g_lastCtrlInputMs = 0;

// Route upload/execution state.
static WaypointData g_routePoints[ROUTE_MAX_WAYPOINTS_PER_ROUTE] = {};
static uint8_t g_routeId = 0;
static uint8_t g_routeCount = 0;
static bool g_routeUploadActive = false;
static bool g_routeRunning = false;
static uint8_t g_routeCurrentIndex = 0;
static uint8_t g_routeExpectedWaypoints = 0;
static uint8_t g_routeExpectedChunks = 0;
static uint8_t g_routeReceivedWaypoints = 0;
static uint8_t g_routeNextChunk = 0;
static uint16_t g_routeDistanceM = 0;

// ESP-NOW diagnostics
static uint32_t g_sendCount     = 0;
static uint32_t g_sendOkCount   = 0;
static uint32_t g_sendFailCount = 0;

// ---------------------------------------------------------------------------
// NVS helpers — autopilot tuning
// ---------------------------------------------------------------------------
static void loadTuningFromNVS() {
    g_apKp        = prefs.getFloat("apKp",       0.008f);
    g_apKi        = prefs.getFloat("apKi",       0.0f);
    g_apKd        = prefs.getFloat("apKd",       0.2f);
    g_apMaxOutput = prefs.getFloat("apMax",      1.0f);
    g_apDeadband  = prefs.getFloat("apDead",     3.0f);
    g_apRateLimit = prefs.getFloat("apRate",     0.05f);

    // Use full steering authority by default for heading hold.
    if (g_apMaxOutput < 1.0f) {
        g_apMaxOutput = 1.0f;
        prefs.putFloat("apMax", g_apMaxOutput);
    }

    Serial.printf("[Setup] Autopilot tuning: Kp=%.4f Ki=%.4f Kd=%.3f max=%.2f dead=%.1f rate=%.3f\n",
                  g_apKp, g_apKi, g_apKd, g_apMaxOutput, g_apDeadband, g_apRateLimit);
}

static void saveTuningToNVS() {
    prefs.putFloat("apKp",   g_apKp);
    prefs.putFloat("apKi",   g_apKi);
    prefs.putFloat("apKd",   g_apKd);
    prefs.putFloat("apMax",  g_apMaxOutput);
    prefs.putFloat("apDead", g_apDeadband);
    prefs.putFloat("apRate", g_apRateLimit);
}

// ---------------------------------------------------------------------------
// Compass calibration (hard-iron offset removal)
// ---------------------------------------------------------------------------

static void calLoadFromNVS() {
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

    sBmm150MagData_t mag = bmm150.getGeomagneticData();
    g_calMinX = g_calMaxX = mag.x;
    g_calMinY = g_calMaxY = mag.y;
    g_calMinZ = g_calMaxZ = mag.z;

    g_calStartTime = millis();
    g_calActive    = true;
}

static void calFeedSample(float x, float y, float z) {
    if (x < g_calMinX) g_calMinX = x;
    if (x > g_calMaxX) g_calMaxX = x;
    if (y < g_calMinY) g_calMinY = y;
    if (y > g_calMaxY) g_calMaxY = y;
    if (z < g_calMinZ) g_calMinZ = z;
    if (z > g_calMaxZ) g_calMaxZ = z;
}

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

static float readHeading() {
    sBmm150MagData_t mag = bmm150.getGeomagneticData();

    if (g_calActive) {
        calFeedSample(mag.x, mag.y, mag.z);
    }

    float cx = mag.x - g_calOffsetX;
    float cy = mag.y - g_calOffsetY;

    // Sensor is mounted with its Z axis pointing DOWN, which mirrors the
    // rotation sense of the X-Y plane (heading would count backwards: N->E
    // reading 0->270). Negate the X component to un-mirror so the heading
    // increases as the boat turns to starboard (N=0, E=90, S=180, W=270).
    float rawDeg = atan2(-cx, cy) * 180.0f / PI;
    if (rawDeg < 0) rawDeg += 360.0f;

    float rawRad = rawDeg * PI / 180.0f;
    g_emaSin = HEADING_EMA_ALPHA * sinf(rawRad) + (1.0f - HEADING_EMA_ALPHA) * g_emaSin;
    g_emaCos = HEADING_EMA_ALPHA * cosf(rawRad) + (1.0f - HEADING_EMA_ALPHA) * g_emaCos;

    float h = atan2f(g_emaSin, g_emaCos) * 180.0f / PI;
    if (h < 0) h += 360.0f;
    return h;
}

// ---------------------------------------------------------------------------
// Heading-hold autopilot (PD controller)
// ---------------------------------------------------------------------------

static float headingError(float from, float to) {
    float err = to - from;
    if (err > 180.0f)  err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

static void autopilotUpdate() {
    float err = headingError(g_heading, g_targetHeading);
    unsigned long now = millis();

    float dt = (now - g_prevErrorTime) / 1000.0f;
    float dErr = 0.0f;
    if (dt > 0.001f) {
        dErr = headingError(g_prevError, err) / dt;
    }
    g_prevError     = err;
    g_prevErrorTime = now;

    float desired = 0.0f;
    if (fabsf(err) > g_apDeadband) {
        // Accumulate integral (only outside dead band to prevent windup at setpoint)
        if (dt > 0.001f) {
            g_integral += err * dt;
            // Clamp integral to prevent windup — limit its contribution to max output
            if (g_apKi > 0.0f) {
                float maxIntegral = g_apMaxOutput / g_apKi;
                g_integral = constrain(g_integral, -maxIntegral, maxIntegral);
            }
        }
        desired = g_apKp * err + g_apKi * g_integral + g_apKd * dErr;
        desired = constrain(desired, -g_apMaxOutput, g_apMaxOutput);
    } else {
        // Inside dead band — decay integral toward zero to prevent stale buildup
        g_integral *= 0.95f;
    }

    float delta = desired - g_apOutput;
    if (delta >  g_apRateLimit) delta =  g_apRateLimit;
    if (delta < -g_apRateLimit) delta = -g_apRateLimit;
    g_apOutput += delta;
}

static void engageHeadingHold() {
    g_targetHeading     = g_heading;
    g_headingHoldActive = true;
    g_apOutput          = 0.0f;
    g_prevError         = 0.0f;
    g_integral          = 0.0f;
    g_prevErrorTime     = millis();
    Serial.printf("[AP] Heading hold ENGAGED at %.1f°\n", g_targetHeading);
}

static void disengageHeadingHold() {
    if (!g_headingHoldActive) return;
    g_headingHoldActive = false;
    g_apOutput          = 0.0f;
    Serial.println("[AP] Heading hold DISENGAGED — manual control.");
}

// ---------------------------------------------------------------------------
// ESP-NOW helpers
// ---------------------------------------------------------------------------

static void sendSteeringSpeed(float speed) {
    MeshMessage msg;
    msg.type   = MSG_SET_STEERING;
    msg.src    = MODULE_NAVIGATION;
    msg.value1 = constrain(speed, -1.0f, 1.0f);
    msg.value2 = 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void onDataSent(const wifi_tx_info_t* /*tx_info*/, esp_now_send_status_t status) {
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

static void broadcastHeadingHoldStatus() {
    MeshMessage msg;
    msg.type   = MSG_HEADING_HOLD_STATUS;
    msg.src    = MODULE_NAVIGATION;
    msg.value1 = g_headingHoldActive ? 1.0f : 0.0f;
    msg.value2 = g_headingHoldActive ? g_targetHeading : 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void broadcastApTuningStatus() {
    const uint8_t paramIds[] = {
        AP_PARAM_KP,
        AP_PARAM_KI,
        AP_PARAM_KD,
        AP_PARAM_MAX_OUTPUT,
        AP_PARAM_DEADBAND,
        AP_PARAM_RATE_LIMIT,
    };
    const float values[] = {
        g_apKp,
        g_apKi,
        g_apKd,
        g_apMaxOutput,
        g_apDeadband,
        g_apRateLimit,
    };

    for (size_t i = 0; i < (sizeof(paramIds) / sizeof(paramIds[0])); ++i) {
        MeshMessage msg;
        msg.type   = MSG_AP_TUNING_STATUS;
        msg.src    = MODULE_NAVIGATION;
        msg.value1 = (float)paramIds[i];
        msg.value2 = values[i];
        esp_now_send(MESH_BROADCAST_ADDR,
                     reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
    }
}

static void broadcastRouteStatus(uint8_t state) {
    RouteStatusMessage msg = {};
    msg.type = MSG_ROUTE_STATUS;
    msg.src = MODULE_NAVIGATION;
    msg.routeId = g_routeId;
    msg.state = state;
    msg.currentIndex = g_routeCurrentIndex;
    msg.totalWaypoints = g_routeCount;
    msg.distanceM = g_routeDistanceM;
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void broadcastWaypointReached(uint8_t waypointIndex) {
    WaypointReachedMessage msg = {};
    msg.type = MSG_WAYPOINT_REACHED;
    msg.src = MODULE_NAVIGATION;
    msg.routeId = g_routeId;
    msg.waypointIndex = waypointIndex;
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void broadcastRouteComplete() {
    RouteCompleteMessage msg = {};
    msg.type = MSG_ROUTE_COMPLETE;
    msg.src = MODULE_NAVIGATION;
    msg.routeId = g_routeId;
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void broadcastRouteAbort(uint8_t reason) {
    RouteAbortMessage msg = {};
    msg.type = MSG_ROUTE_ABORT;
    msg.src = MODULE_NAVIGATION;
    msg.routeId = g_routeId;
    msg.reason = reason;
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static bool isValidWaypoint(const WaypointData& wp) {
    double lat = (double)wp.latE7 / 10000000.0;
    double lon = (double)wp.lonE7 / 10000000.0;
    if (lat < -90.0 || lat > 90.0) return false;
    if (lon < -180.0 || lon > 180.0) return false;
    if (wp.radiusM == 0) return false;
    return true;
}

static void resetRouteUploadState() {
    g_routeUploadActive = false;
    g_routeExpectedWaypoints = 0;
    g_routeExpectedChunks = 0;
    g_routeReceivedWaypoints = 0;
    g_routeNextChunk = 0;
}

static void abortRoute(uint8_t reason) {
    g_routeRunning = false;
    g_routeCurrentIndex = 0;
    g_routeDistanceM = 0;
    disengageHeadingHold();
    sendSteeringSpeed(0.0f);
    broadcastRouteAbort(reason);
    broadcastRouteStatus(ROUTE_STATE_ABORTED);
}

static bool startRouteExecution() {
    if (g_routeCount == 0) {
        abortRoute(ROUTE_ABORT_EMPTY_ROUTE);
        return false;
    }
    if (!g_bmm150Ok) {
        abortRoute(ROUTE_ABORT_NO_HEADING);
        return false;
    }
    if (!gps.location.isValid() || gps.location.age() > ROUTE_GPS_STALE_MS) {
        abortRoute(ROUTE_ABORT_NO_GPS_FIX);
        return false;
    }

    g_routeRunning = true;
    g_routeCurrentIndex = 0;
    g_routeDistanceM = 0;
    if (!g_headingHoldActive) {
        engageHeadingHold();
    }
    broadcastRouteStatus(ROUTE_STATE_RUNNING);
    Serial.printf("[Route] START route=%u waypoints=%u\n", g_routeId, g_routeCount);
    return true;
}

static void updateRouteGuidance() {
    if (!g_routeRunning) return;

    if (!gps.location.isValid() || gps.location.age() > ROUTE_GPS_STALE_MS) {
        abortRoute(ROUTE_ABORT_NO_GPS_FIX);
        return;
    }

    if (!g_bmm150Ok) {
        abortRoute(ROUTE_ABORT_NO_HEADING);
        return;
    }

    if (g_routeCurrentIndex >= g_routeCount) {
        g_routeRunning = false;
        broadcastRouteComplete();
        broadcastRouteStatus(ROUTE_STATE_COMPLETE);
        disengageHeadingHold();
        sendSteeringSpeed(0.0f);
        return;
    }

    WaypointData wp = g_routePoints[g_routeCurrentIndex];
    double tgtLat = (double)wp.latE7 / 10000000.0;
    double tgtLon = (double)wp.lonE7 / 10000000.0;
    double curLat = gps.location.lat();
    double curLon = gps.location.lng();

    double distance = TinyGPSPlus::distanceBetween(curLat, curLon, tgtLat, tgtLon);
    if (distance < 0.0) distance = 0.0;
    if (distance > 65535.0) distance = 65535.0;
    g_routeDistanceM = (uint16_t)distance;

    uint16_t radiusM = wp.radiusM == 0 ? ROUTE_DEFAULT_RADIUS_M : wp.radiusM;
    if (distance <= radiusM) {
        broadcastWaypointReached(g_routeCurrentIndex);
        ++g_routeCurrentIndex;
        if (g_routeCurrentIndex >= g_routeCount) {
            g_routeRunning = false;
            broadcastRouteComplete();
            broadcastRouteStatus(ROUTE_STATE_COMPLETE);
            disengageHeadingHold();
            sendSteeringSpeed(0.0f);
            Serial.println("[Route] COMPLETE");
            return;
        }
    }

    WaypointData nextWp = g_routePoints[g_routeCurrentIndex];
    double nextLat = (double)nextWp.latE7 / 10000000.0;
    double nextLon = (double)nextWp.lonE7 / 10000000.0;
    g_targetHeading = (float)TinyGPSPlus::courseTo(curLat, curLon, nextLat, nextLon);
    if (!g_headingHoldActive) {
        engageHeadingHold();
    }
}

// ---------------------------------------------------------------------------
// ESP-NOW receive callback
// ---------------------------------------------------------------------------
static void onMeshReceive(const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len) {
    if (len < 2) return;
    uint8_t type = data[0];

    if (type == MSG_CONTROLLER_INPUT &&
        len >= static_cast<int>(sizeof(ControllerInputMessage))) {
        ControllerInputMessage ci;
        memcpy(&ci, data, sizeof(ci));
        g_stickLX    = ci.lx;
        g_ctrlBtns   = ci.buttons;
        g_ctrlOnline = true;
        g_lastCtrlInputMs = millis();
        return;
    }

    if (type == MSG_AP_TUNING && len >= static_cast<int>(sizeof(MeshMessage))) {
        MeshMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_HANDHELD) return;
        uint8_t paramId = (uint8_t)m.value1;
        float   value   = m.value2;
        switch (paramId) {
            case AP_PARAM_KP:         g_apKp        = constrain(value,  0.0f,  1.0f); break;
            case AP_PARAM_KI:         g_apKi        = constrain(value,  0.0f,  1.0f); break;
            case AP_PARAM_KD:         g_apKd        = constrain(value,  0.0f,  5.0f); break;
            case AP_PARAM_MAX_OUTPUT: g_apMaxOutput = constrain(value,  0.05f, 1.0f); break;
            case AP_PARAM_DEADBAND:   g_apDeadband  = constrain(value,  0.0f, 30.0f); break;
            case AP_PARAM_RATE_LIMIT: g_apRateLimit = constrain(value,  0.001f,1.0f); break;
            default: return;
        }
        g_integral = 0.0f;  // reset integral when tuning changes
        saveTuningToNVS();
        broadcastApTuningStatus();
        Serial.printf("[APTune] param=%u val=%.4f (Kp=%.4f Ki=%.4f Kd=%.3f max=%.2f dead=%.1f rate=%.3f)\n",
                      paramId, value, g_apKp, g_apKi, g_apKd, g_apMaxOutput, g_apDeadband, g_apRateLimit);
        return;
    }

    if (type == MSG_ROUTE_START && len >= static_cast<int>(sizeof(RouteStartMessage))) {
        RouteStartMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_HANDHELD) return;

        if (m.totalWaypoints == 0 || m.totalWaypoints > ROUTE_MAX_WAYPOINTS_PER_ROUTE) {
            broadcastRouteAbort(ROUTE_ABORT_BAD_FORMAT);
            return;
        }

        g_routeId = m.routeId;
        g_routeCount = 0;
        g_routeExpectedWaypoints = m.totalWaypoints;
        g_routeExpectedChunks = m.totalChunks;
        g_routeReceivedWaypoints = 0;
        g_routeNextChunk = 0;
        g_routeUploadActive = true;
        g_routeRunning = false;
        memset(g_routePoints, 0, sizeof(g_routePoints));
        broadcastRouteStatus(ROUTE_STATE_UPLOADING);
        Serial.printf("[Route] START upload route=%u wps=%u chunks=%u\n",
                      g_routeId, g_routeExpectedWaypoints, g_routeExpectedChunks);
        return;
    }

    if (type == MSG_ROUTE_CHUNK && len >= static_cast<int>(sizeof(RouteChunkMessage))) {
        RouteChunkMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_HANDHELD) return;
        if (!g_routeUploadActive || m.routeId != g_routeId) return;

        if (m.chunkIndex != g_routeNextChunk) {
            resetRouteUploadState();
            broadcastRouteAbort(ROUTE_ABORT_BAD_SEQUENCE);
            return;
        }

        if (m.waypointCount == 0 || m.waypointCount > ROUTE_WAYPOINTS_PER_CHUNK) {
            resetRouteUploadState();
            broadcastRouteAbort(ROUTE_ABORT_BAD_FORMAT);
            return;
        }

        for (uint8_t i = 0; i < m.waypointCount; ++i) {
            if (g_routeReceivedWaypoints >= ROUTE_MAX_WAYPOINTS_PER_ROUTE) {
                resetRouteUploadState();
                broadcastRouteAbort(ROUTE_ABORT_BAD_FORMAT);
                return;
            }

            WaypointData wp = m.waypoints[i];
            if (wp.radiusM == 0) wp.radiusM = ROUTE_DEFAULT_RADIUS_M;
            if (!isValidWaypoint(wp)) {
                resetRouteUploadState();
                broadcastRouteAbort(ROUTE_ABORT_BAD_WAYPOINT);
                return;
            }
            g_routePoints[g_routeReceivedWaypoints++] = wp;
        }

        ++g_routeNextChunk;
        return;
    }

    if (type == MSG_ROUTE_END && len >= static_cast<int>(sizeof(RouteEndMessage))) {
        RouteEndMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_HANDHELD) return;
        if (!g_routeUploadActive || m.routeId != g_routeId) return;

        bool ok = (g_routeReceivedWaypoints == g_routeExpectedWaypoints) &&
                  (g_routeNextChunk == g_routeExpectedChunks);
        if (!ok) {
            resetRouteUploadState();
            broadcastRouteAbort(ROUTE_ABORT_BAD_SEQUENCE);
            return;
        }

        g_routeCount = g_routeReceivedWaypoints;
        resetRouteUploadState();
        broadcastRouteStatus(ROUTE_STATE_READY);
        Serial.printf("[Route] Upload complete route=%u wps=%u\n", g_routeId, g_routeCount);
        return;
    }

    if (type == MSG_ROUTE_CONTROL && len >= static_cast<int>(sizeof(RouteControlMessage))) {
        RouteControlMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_HANDHELD) return;
        if (m.routeId != g_routeId) return;

        if (m.action == ROUTE_ACTION_START) {
            startRouteExecution();
        } else if (m.action == ROUTE_ACTION_STOP) {
            abortRoute(ROUTE_ABORT_STOP_REQUESTED);
        }
        return;
    }
}

// ---------------------------------------------------------------------------
// WebSocket helpers
// ---------------------------------------------------------------------------
static void buildStatusJson(String& out) {
    float err = headingError(g_heading, g_targetHeading);
    out = "{\"hdg\":";       out += String(g_heading, 1);
    out += ",\"cal\":";      out += g_calibrated ? "true" : "false";
    out += ",\"calActive\":"; out += g_calActive ? "true" : "false";
    if (g_calActive) {
        unsigned long elapsed = millis() - g_calStartTime;
        unsigned long remaining = (elapsed < CAL_DURATION_MS) ? (CAL_DURATION_MS - elapsed) / 1000 : 0;
        out += ",\"calRemain\":"; out += remaining;
    }
    out += ",\"hold\":";     out += g_headingHoldActive ? "true" : "false";
    out += ",\"target\":";   out += String(g_targetHeading, 1);
    out += ",\"err\":";      out += String(err, 1);
    out += ",\"steer\":";    out += String(g_apOutput, 3);
    out += ",\"ctrl\":";     out += g_ctrlOnline ? "true" : "false";
    out += ",\"stick\":";    out += String(g_stickLX, 2);
    out += ",\"routeId\":"; out += (unsigned long)g_routeId;
    out += ",\"routeCount\":"; out += (unsigned long)g_routeCount;
    out += ",\"routeIdx\":"; out += (unsigned long)g_routeCurrentIndex;
    out += ",\"routeRun\":"; out += g_routeRunning ? "true" : "false";
    out += ",\"routeUp\":"; out += g_routeUploadActive ? "true" : "false";
    out += ",\"routeDist\":"; out += (unsigned long)g_routeDistanceM;
    out += ",\"gpsFix\":";   out += gps.location.isValid() ? "true" : "false";
    out += ",\"gpsSats\":";  out += (unsigned long)gps.satellites.value();
    if (gps.location.isValid()) {
        out += ",\"lat\":";  out += String(gps.location.lat(), 6);
        out += ",\"lon\":";  out += String(gps.location.lng(), 6);
        out += ",\"spd\":";  out += String(gps.speed.knots(), 1);
        out += ",\"crs\":";  out += String(gps.course.deg(), 1);
    }
    out += ",\"gpsChars\":"; out += (unsigned long)gps.charsProcessed();
    out += "}";
}

static void onWsEvent(AsyncWebSocket* /*srv*/, AsyncWebSocketClient* client,
                      AwsEventType type, void* /*arg*/, uint8_t* /*data*/, size_t /*len*/) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client #%u connected\n", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
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
    Wire.begin();
    while (bmm150.begin()) {
        Serial.println("[Setup] BMM150 not found — retrying in 1 s…");
        delay(1000);
    }
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_30HZ);
    g_bmm150Ok = true;
    Serial.println("[Setup] BMM150 magnetometer initialised (30 Hz, high accuracy).");

    // --- NVS ---
    prefs.begin("nav", false);
    calLoadFromNVS();
    loadTuningFromNVS();

    // --- LittleFS ---
    if (!LittleFS.begin(true)) {
        Serial.println("[Setup] ERROR: LittleFS mount failed!");
    } else {
        Serial.println("[Setup] LittleFS mounted.");
    }

    // --- WiFi AP+STA + ESP-NOW ---
    // AP_STA mode is required so the STA interface is available for ESP-NOW
    // reception while the AP interface serves the web UI.
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS, MESH_WIFI_CHANNEL);
    WiFi.disconnect();  // STA not joining any network, just kept active for ESP-NOW
    esp_wifi_set_channel(MESH_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    IPAddress ip = WiFi.softAPIP();
    Serial.printf("[Setup] WiFi AP \"%s\" on channel %d started.\n", AP_SSID, MESH_WIFI_CHANNEL);
    Serial.printf("[Setup] Open http://%s in your browser.\n", ip.toString().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Serial.println("[Setup] ESP-NOW initialised.");
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onMeshReceive);

    // Register broadcast peer on BOTH interfaces so ESP-NOW works regardless
    // of which interface the sender used.
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, MESH_BROADCAST_ADDR, 6);
    peer.channel = MESH_WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;  // receive/send via STA interface
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Serial.println("[Setup] Broadcast peer registered.");
    } else {
        Serial.println("[Setup] WARNING: Failed to register broadcast peer.");
    }

    // --- WebSocket ---
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // --- HTTP routes ---
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    // GET /settings — returns autopilot tuning as JSON
    server.on("/settings", HTTP_GET, [](AsyncWebServerRequest* request) {
        String json = "{\"kp\":";    json += String(g_apKp, 4);
        json += ",\"ki\":";          json += String(g_apKi, 4);
        json += ",\"kd\":";          json += String(g_apKd, 4);
        json += ",\"maxOutput\":";   json += String(g_apMaxOutput, 3);
        json += ",\"deadband\":";    json += String(g_apDeadband, 1);
        json += ",\"rateLimit\":";   json += String(g_apRateLimit, 4);
        json += "}";
        request->send(200, "application/json", json);
    });

    // POST /settings — save autopilot tuning
    static String settingsBody;
    server.on("/settings", HTTP_POST,
        [](AsyncWebServerRequest* request) {
            auto parseField = [](const String& json, const char* key, float fallback) -> float {
                String search = "\""; search += key; search += "\":";
                int idx = json.indexOf(search);
                if (idx < 0) return fallback;
                return json.substring(idx + search.length()).toFloat();
            };

            g_apKp        = parseField(settingsBody, "kp",        g_apKp);
            g_apKi        = parseField(settingsBody, "ki",        g_apKi);
            g_apKd        = parseField(settingsBody, "kd",        g_apKd);
            g_apMaxOutput = parseField(settingsBody, "maxOutput",  g_apMaxOutput);
            g_apDeadband  = parseField(settingsBody, "deadband",   g_apDeadband);
            g_apRateLimit = parseField(settingsBody, "rateLimit",  g_apRateLimit);

            // Clamp to sane ranges
            g_apKp        = constrain(g_apKp,        0.0f, 1.0f);
            g_apKi        = constrain(g_apKi,        0.0f, 1.0f);
            g_apKd        = constrain(g_apKd,        0.0f, 5.0f);
            g_apMaxOutput = constrain(g_apMaxOutput,  0.05f, 1.0f);
            g_apDeadband  = constrain(g_apDeadband,   0.0f, 30.0f);
            g_apRateLimit = constrain(g_apRateLimit,  0.001f, 1.0f);

            saveTuningToNVS();

            // Reset integral when tuning changes to prevent stale buildup
            g_integral = 0.0f;

            Serial.printf("[Settings] Saved — Kp=%.4f Ki=%.4f Kd=%.3f max=%.2f dead=%.1f rate=%.3f\n",
                          g_apKp, g_apKi, g_apKd, g_apMaxOutput, g_apDeadband, g_apRateLimit);
            request->send(200, "application/json", "{\"ok\":true}");
        },
        nullptr,
        [](AsyncWebServerRequest*, uint8_t* data, size_t len,
           size_t index, size_t) {
            if (index == 0) settingsBody = "";
            settingsBody += String(reinterpret_cast<const char*>(data), len);
        }
    );

    // POST /calibrate — trigger compass calibration from the web UI
    server.on("/calibrate", HTTP_POST, [](AsyncWebServerRequest* request) {
        if (g_calActive) {
            request->send(200, "application/json", "{\"ok\":false,\"msg\":\"already running\"}");
        } else {
            calStart();
            request->send(200, "application/json", "{\"ok\":true}");
        }
    });

    server.begin();
    Serial.println("[Setup] HTTP server started.");

    Serial.println("[Setup] Navigation module ready.");
    Serial.println("[Setup] Send 'c' over serial to calibrate compass.");
    Serial.println("[Setup] D-pad UP on controller to engage heading hold.");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    static unsigned long lastBroadcast  = 0;
    static unsigned long lastLog        = 0;
    static unsigned long lastCalProg    = 0;
    static unsigned long lastAutopilot  = 0;
    static unsigned long lastApSend     = 0;
    static unsigned long lastWsBcast    = 0;
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

    // --- Controller input → heading hold ---
    if (g_ctrlOnline) {
        bool upNow   = (g_ctrlBtns & CTRL_BTN_UP)   != 0;
        bool upPrev  = (g_prevBtns & CTRL_BTN_UP)   != 0;
        bool dnNow   = (g_ctrlBtns & CTRL_BTN_DOWN) != 0;
        bool dnPrev  = (g_prevBtns & CTRL_BTN_DOWN) != 0;
        if (upNow && !upPrev && g_bmm150Ok) {
            engageHeadingHold();
        }
        if (dnNow && !dnPrev && g_headingHoldActive) {
            if (g_routeRunning) {
                abortRoute(ROUTE_ABORT_STOP_REQUESTED);
            } else {
                disengageHeadingHold();
            }
        }
        g_prevBtns = g_ctrlBtns;
    }

    updateRouteGuidance();

    if (g_headingHoldActive) {
            bool ctrlFresh = g_ctrlOnline && ((now - g_lastCtrlInputMs) <= CTRL_INPUT_TIMEOUT_MS);
            float absStick = fabsf(g_stickLX);
            bool manualOverride = false;
            if (ctrlFresh) {
                manualOverride = g_headingHoldYielding
                    ? (absStick > STICK_OVERRIDE_EXIT)
                    : (absStick > STICK_OVERRIDE_ENTER);
            }

            if (manualOverride) {
                if (!g_headingHoldYielding) {
                    Serial.println("[AP] HOLD YIELDING to manual stick input.");
                }
                g_headingHoldYielding = true;
                g_lastManualOverrideMs = now;
                // Freeze AP output while yielding so resume is smooth.
                g_apOutput = 0.0f;
                g_integral = 0.0f;
                g_prevError = 0.0f;
                g_prevErrorTime = now;
            } else {
                bool centeredLongEnough =
                    (now - g_lastManualOverrideMs) >= HOLD_RESUME_DELAY_MS;

                if (g_headingHoldYielding && !centeredLongEnough) {
                    // Keep yielding until centered state is stable.
                } else {
                if (g_headingHoldYielding) {
                    Serial.println("[AP] HOLD RESUMED.");
                }
                g_headingHoldYielding = false;

                if (now - lastAutopilot >= AUTOPILOT_MS) {
                    lastAutopilot = now;
                    autopilotUpdate();
                }
                if (now - lastApSend >= AP_SEND_MS) {
                    lastApSend = now;
                    sendSteeringSpeed(g_apOutput);
                }
                }
            }
    } else {
        g_headingHoldYielding = false;
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

    // Broadcast navigation + heading hold status over ESP-NOW.
    if (now - lastBroadcast >= NAV_BROADCAST_MS) {
        lastBroadcast = now;
        broadcastNavStatus();
        broadcastHeadingHoldStatus();
        broadcastApTuningStatus();
        if (g_routeUploadActive) {
            broadcastRouteStatus(ROUTE_STATE_UPLOADING);
        } else if (g_routeRunning) {
            broadcastRouteStatus(ROUTE_STATE_RUNNING);
        } else if (g_routeCount > 0) {
            broadcastRouteStatus(ROUTE_STATE_READY);
        } else {
            broadcastRouteStatus(ROUTE_STATE_IDLE);
        }
    }

    // Broadcast status to WebSocket clients.
    if (now - lastWsBcast >= WS_BROADCAST_MS) {
        lastWsBcast = now;
        if (ws.count() > 0) {
            String json;
            buildStatusJson(json);
            ws.textAll(json);
        }
    }

    // Periodically clean up disconnected WebSocket clients.
    static unsigned long lastCleanup = 0;
    if (now - lastCleanup >= 1000) {
        lastCleanup = now;
        ws.cleanupClients();
    }

    // Periodic serial diagnostics.
    if (!g_calActive && (now - lastLog >= DIAG_LOG_MS)) {
        lastLog = now;

        const char* calTag = g_calibrated ? "CAL" : "UNCAL";

        if (g_headingHoldActive) {
            float err = headingError(g_heading, g_targetHeading);
            Serial.printf("[AP] %s %.1f°  hdg=%.1f°  err=%+.1f°  steer=%+.3f\n",
                          g_headingHoldYielding ? "YIELD" : "HOLD",
                          g_targetHeading, g_heading, err, g_apOutput);
        } else {
            Serial.printf("[AP] MANUAL  hdg=%.1f° [%s]  stick=%.2f\n",
                          g_heading, calTag, g_stickLX);
        }

        if (gps.location.isValid()) {
            Serial.printf("[Nav] GPS: %.6f, %.6f  sats=%lu  spd=%.1fkn  crs=%.1f°\n",
                          gps.location.lat(), gps.location.lng(),
                          (unsigned long)gps.satellites.value(),
                          gps.speed.knots(), gps.course.deg());
        } else if (gps.charsProcessed() < 10) {
            Serial.printf("[Nav] GPS: NO DATA (check wiring: TX→GPIO%d, baud=%d)\n",
                          GPS_RX_PIN, GPS_BAUD);
        } else {
            Serial.printf("[Nav] GPS: WAITING FOR FIX  sats=%lu  sentences=%lu  failed=%lu\n",
                          (unsigned long)gps.satellites.value(),
                          gps.sentencesWithFix(),
                          gps.failedChecksum());
        }
    }

    delay(10);
}
