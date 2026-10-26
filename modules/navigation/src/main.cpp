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
#include <esp_system.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <DFRobot_BMM150.h>
#include "mesh_protocol.h"
#include "route_track.h"
#include "mesh_remote.h"  // logs, commands and firmware updates through the handheld

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
#define GPS_RX_PIN  16   // ESP32 UART2 RX ← NEO-6M TX
#define GPS_TX_PIN  17   // ESP32 UART2 TX → NEO-6M RX
#define GPS_BAUD    9600 // NEO-6M default baud rate

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define BMM150_I2C_ADDR  0x13  // BMM150 default I2C address

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static constexpr unsigned long NAV_BROADCAST_MS  = 500;   // 2 Hz mesh broadcast
static constexpr unsigned long DIAG_LOG_MS       = 1000;  // 1 Hz serial diagnostics
static constexpr unsigned long CAL_DURATION_MS   = 45000; // calibration collection window
static constexpr unsigned long CAL_PROGRESS_MS   = 1000;  // calibration progress prints
static constexpr unsigned long AUTOPILOT_MS      = 200;   // 5 Hz autopilot compute rate
static constexpr unsigned long AP_SEND_MS        = 50;    // 20 Hz send rate — must be well under the steering watchdog (200 ms)
static constexpr float         STICK_OVERRIDE_ENTER = 0.20f; // enter manual-yield above this magnitude
static constexpr float         STICK_OVERRIDE_EXIT  = 0.10f; // remain in manual-yield until below this magnitude
static constexpr unsigned long CTRL_INPUT_TIMEOUT_MS = 1000; // controller input considered stale after this
static constexpr unsigned long HOLD_RESUME_DELAY_MS = 300; // stick must remain centered this long before hold resumes
static constexpr unsigned long ROUTE_GPS_STALE_MS = 3000;
static constexpr unsigned long COMPASS_RETRY_MS     = 1000; // re-init attempt interval while the compass is down
static constexpr unsigned long COMPASS_TIMEOUT_MS   = 1000; // no valid read for this long → compass lost
static constexpr unsigned long COMPASS_STALL_MS     = 2000; // no new measurement for this long → compass lost
static constexpr unsigned long COMPASS_LOST_HOLD_MS = 5000; // release heading hold / abort route after this long without a compass

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
// Route following (see route_track.h)
// ---------------------------------------------------------------------------
static constexpr float ROUTE_LOOKAHEAD_M = 30.0f; // steer for the point this far along the track beyond the boat

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------

// DFRobot's getGeomagneticData() ignores I2C errors and keeps returning
// plausible-looking numbers when the bus is dead or the sensor has reset
// itself into suspend mode, so the heading silently freezes. readMag() reads
// the same registers but reports whether the read actually worked.
class CheckedBMM150 : public DFRobot_BMM150_I2C {
public:
    using DFRobot_BMM150_I2C::DFRobot_BMM150_I2C;

    // Returns false on an I2C error or a sample with no valid data.
    // newSample is the sensor's data-ready bit: set when it has finished a
    // measurement since the previous read.
    bool readMag(sBmm150MagData_t& out, bool& newSample) {
        uint8_t raw[BMM150_LEN_XYZR_DATA];
        Wire.beginTransmission(BMM150_I2C_ADDR);
        Wire.write(BMM150_REG_DATA_X_LSB);
        if (Wire.endTransmission() != 0) return false;
        if (Wire.requestFrom((uint8_t)BMM150_I2C_ADDR, (uint8_t)sizeof(raw)) != sizeof(raw)) return false;
        for (uint8_t& b : raw) b = Wire.read();

        int16_t  x = (int16_t)(((int16_t)(int8_t)raw[1]) * 32  | BMM150_GET_BITS(raw[0], BMM150_DATA_X));
        int16_t  y = (int16_t)(((int16_t)(int8_t)raw[3]) * 32  | BMM150_GET_BITS(raw[2], BMM150_DATA_Y));
        int16_t  z = (int16_t)(((int16_t)(int8_t)raw[5]) * 128 | BMM150_GET_BITS(raw[4], BMM150_DATA_Z));
        uint16_t rhall = (uint16_t)(((uint16_t)raw[7] << 6) | BMM150_GET_BITS(raw[6], BMM150_DATA_RHALL));

        // A suspended (freshly reset) sensor reads back all zeros; the hall
        // resistance is never zero in a real measurement.
        if (rhall == 0) return false;

        out.x = compensateX(x, rhall);
        out.y = compensateY(y, rhall);
        out.z = compensateZ(z, rhall);
        if (out.x == BMM150_OVERFLOW_OUTPUT || out.y == BMM150_OVERFLOW_OUTPUT) return false;

        newSample = (raw[6] & BMM150_DRDY_STATUS_MSK) != 0;
        return true;
    }
};

static TinyGPSPlus        gps;
static CheckedBMM150      bmm150(&Wire, BMM150_I2C_ADDR);
static Preferences        prefs;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static float g_heading = 0.0f;   // magnetic heading in degrees [0, 360)
static bool  g_bmm150Ok = false; // magnetometer initialised and responding

// Compass health. The sensor is re-initialised whenever reads fail or it
// stops producing new measurements, instead of needing a power cycle.
static sBmm150MagData_t g_lastMag = {};
static unsigned long g_compassLastValidMs = 0;  // last read that returned usable data
static unsigned long g_compassLastFreshMs = 0;  // last read with the data-ready bit set
static unsigned long g_compassLastInitMs  = 0;  // last (re)init attempt
static unsigned long g_compassLastHeadingMs = 0; // last time g_heading was updated from the sensor
static bool          g_compassDrdyWorks   = false; // data-ready bit seen at least once; arms the stall check
static bool          g_compassReseed      = true;  // restart the heading filter on the next sample
static uint32_t      g_compassLostCount   = 0;
static const char*   g_resetReason        = "";

// True when g_heading is current: the sensor is up and has produced a sample
// since it was last (re)initialised.
static bool compassHeadingValid() {
    return g_bmm150Ok && !g_compassReseed;
}

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
static volatile bool g_routeStartRequested = false; // set by ESP-NOW; started from loop() so the track is only touched there
static RouteTrack    g_routeTrack = {};             // the running route; a new upload doesn't disturb it
static RouteGuidance g_routeGuide = {};             // latest guidance while a route runs
static float         g_routeFinishRadiusM = ROUTE_DEFAULT_RADIUS_M;

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

    Log.printf("[Setup] Autopilot tuning: Kp=%.4f Ki=%.4f Kd=%.3f max=%.2f dead=%.1f rate=%.3f\n",
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
        Log.printf("[Cal] Loaded offsets from NVS: X=%.2f  Y=%.2f  Z=%.2f\n",
                   g_calOffsetX, g_calOffsetY, g_calOffsetZ);
    } else {
        Log.println("[Cal] No saved calibration — heading may be inaccurate.");
        Log.println("[Cal] Send 'c' over serial to start calibration.");
    }
}

static void calSaveToNVS() {
    prefs.putFloat("calX", g_calOffsetX);
    prefs.putFloat("calY", g_calOffsetY);
    prefs.putFloat("calZ", g_calOffsetZ);
    prefs.putBool("calOk", true);
}

static void calStart() {
    if (!compassHeadingValid()) {
        Log.println("[Cal] ERROR: BMM150 not responding — cannot calibrate.");
        return;
    }
    Log.println("[Cal] ========================================");
    Log.println("[Cal]  COMPASS CALIBRATION STARTED");
    Log.printf( "[Cal]  Slowly rotate the sensor through all\n");
    Log.printf( "[Cal]  orientations for %lu seconds…\n", CAL_DURATION_MS / 1000);
    Log.println("[Cal] ========================================");

    g_calMinX = g_calMaxX = g_lastMag.x;
    g_calMinY = g_calMaxY = g_lastMag.y;
    g_calMinZ = g_calMaxZ = g_lastMag.z;

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

    Log.println("[Cal] ========================================");
    Log.println("[Cal]  CALIBRATION COMPLETE — offsets saved.");
    Log.printf( "[Cal]  X: min=%.2f  max=%.2f  offset=%.2f\n", g_calMinX, g_calMaxX, g_calOffsetX);
    Log.printf( "[Cal]  Y: min=%.2f  max=%.2f  offset=%.2f\n", g_calMinY, g_calMaxY, g_calOffsetY);
    Log.printf( "[Cal]  Z: min=%.2f  max=%.2f  offset=%.2f\n", g_calMinZ, g_calMaxZ, g_calOffsetZ);
    Log.println("[Cal] ========================================");
}

// ---------------------------------------------------------------------------
// Compass heading
// ---------------------------------------------------------------------------

static float headingFromMag(const sBmm150MagData_t& mag) {
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
    if (g_compassReseed) {
        // First sample after (re)init: start the filter here rather than
        // blending from wherever it was before the sensor dropped out.
        g_compassReseed = false;
        g_emaSin = sinf(rawRad);
        g_emaCos = cosf(rawRad);
    }
    g_emaSin = HEADING_EMA_ALPHA * sinf(rawRad) + (1.0f - HEADING_EMA_ALPHA) * g_emaSin;
    g_emaCos = HEADING_EMA_ALPHA * cosf(rawRad) + (1.0f - HEADING_EMA_ALPHA) * g_emaCos;

    float h = atan2f(g_emaSin, g_emaCos) * 180.0f / PI;
    if (h < 0) h += 360.0f;
    return h;
}

// A reset in the middle of a read can leave the BMM150 holding SDA low while
// it waits for clock pulses that never come; the bus then stays dead until the
// sensor loses power. Clock SCL until it lets go, then send a STOP.
static void i2cBusClear() {
    pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN | PULLUP);
    pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN | PULLUP);
    digitalWrite(I2C_SDA_PIN, HIGH);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(10);

    for (int i = 0; i < 18 && digitalRead(I2C_SDA_PIN) == LOW; ++i) {
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(10);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(10);
    }

    // STOP: SDA rises while SCL is high.
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(10);
}

// Tears down the I2C bus, frees it if it is stuck, and brings the BMM150 back
// up from scratch. Safe to call repeatedly; takes well under a second.
static bool compassTryInit(unsigned long now) {
    g_compassLastInitMs = now;

    Wire.end();
    i2cBusClear();
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (bmm150.begin() != 0) return false;
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_30HZ);

    g_bmm150Ok          = true;
    g_compassReseed     = true;
    g_compassLastValidMs = now;
    g_compassLastFreshMs = now;
    return true;
}

static void compassMarkLost(const char* why, unsigned long now) {
    g_bmm150Ok = false;
    g_compassLastInitMs = now - COMPASS_RETRY_MS;  // retry straight away
    ++g_compassLostCount;
    Log.printf("[Compass] LOST (%s) — reinitialising. Lost %lu time(s) since boot.\n",
               why, (unsigned long)g_compassLostCount);
    if (g_calActive) {
        // Finishing now would save offsets from too few samples.
        g_calActive = false;
        Log.println("[Cal] Calibration aborted — compass lost. Previous calibration kept.");
    }
}

static float headingError(float from, float to);

// Reads the compass and updates g_heading, re-initialising the sensor if it
// has stopped responding.
static void updateCompass(unsigned long now) {
    if (!g_bmm150Ok) {
        if (now - g_compassLastInitMs < COMPASS_RETRY_MS) return;
        if (!compassTryInit(now)) return;
        Log.println("[Compass] BMM150 initialised (30 Hz, high accuracy).");
    }

    sBmm150MagData_t mag;
    bool fresh = false;
    if (bmm150.readMag(mag, fresh)) {
        g_compassLastValidMs = now;
        if (fresh) {
            g_compassLastFreshMs = now;
            g_compassDrdyWorks   = true;
        }
        // After a re-init, wait for the sensor's first real measurement
        // before restarting the heading filter from it.
        if (fresh || !g_compassReseed || !g_compassDrdyWorks) {
            bool reseeding = g_compassReseed;
            g_lastMag = mag;
            g_heading = headingFromMag(mag);
            g_compassLastHeadingMs = now;
            if (reseeding) {
                // Heading may have jumped while the compass was down; don't
                // let the autopilot see that as a derivative kick.
                g_prevError     = headingError(g_heading, g_targetHeading);
                g_prevErrorTime = now;
            }
        }
    }

    if (now - g_compassLastValidMs > COMPASS_TIMEOUT_MS) {
        compassMarkLost("no valid reads", now);
    } else if (g_compassDrdyWorks && now - g_compassLastFreshMs > COMPASS_STALL_MS) {
        compassMarkLost("sensor stopped measuring", now);
    }
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
    Log.printf("[AP] Heading hold ENGAGED at %.1f°\n", g_targetHeading);
}

static void disengageHeadingHold() {
    if (!g_headingHoldActive) return;
    g_headingHoldActive = false;
    g_apOutput          = 0.0f;
    Log.println("[AP] Heading hold DISENGAGED — manual control.");
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
        Log.println("[Mesh] WARNING: Send FAILED!");
    }
}

// esp_now_send() errors never reach onDataSent(); report them here instead,
// at most every 10 s.
static void reportSendError(esp_err_t err) {
    static unsigned long lastReport = 0;
    if (err == ESP_OK || (lastReport != 0 && millis() - lastReport < 10000)) return;
    lastReport = millis();
    Log.printf("[Mesh] WARNING: esp_now_send failed: %s\n", esp_err_to_name(err));
}

static void broadcastNavStatus() {
    NavigationMessage msg = {};
    msg.type = MSG_NAV_STATUS;
    msg.src  = MODULE_NAVIGATION;

    // TinyGPS keeps isValid() true after the fix is lost (or the GPS stops
    // talking), so also require a recent update.
    if (gps.location.isValid() && gps.location.age() <= ROUTE_GPS_STALE_MS) {
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
    // NaN tells receivers there is no heading (the handheld shows "HDG ---")
    // rather than a stale or made-up one.
    msg.heading = compassHeadingValid() ? g_heading : NAN;

    reportSendError(esp_now_send(MESH_BROADCAST_ADDR,
                                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg)));
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
    g_routeGuide = {};
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
    if (!compassHeadingValid()) {
        abortRoute(ROUTE_ABORT_NO_HEADING);
        return false;
    }
    if (!gps.location.isValid() || gps.location.age() > ROUTE_GPS_STALE_MS) {
        abortRoute(ROUTE_ABORT_NO_GPS_FIX);
        return false;
    }

    // Join the track wherever the boat is closest to it; waypoints before
    // that point are skipped.
    routeTrackBuild(g_routeTrack, g_routePoints, g_routeCount);
    routeTrackJoin(g_routeTrack, gps.location.lat(), gps.location.lng());
    uint16_t finishRadius = g_routePoints[g_routeCount - 1].radiusM;
    g_routeFinishRadiusM = finishRadius == 0 ? ROUTE_DEFAULT_RADIUS_M : finishRadius;
    g_routeGuide = routeTrackUpdate(g_routeTrack, gps.location.lat(), gps.location.lng(),
                                    ROUTE_LOOKAHEAD_M, g_routeFinishRadiusM);

    g_routeRunning = true;
    g_routeCurrentIndex = g_routeGuide.nextIndex;
    g_routeDistanceM = 0;
    if (!g_headingHoldActive) {
        engageHeadingHold();
    }
    broadcastRouteStatus(ROUTE_STATE_RUNNING);
    Log.printf("[Route] START route=%u waypoints=%u, joining %.0f m along the %.0f m track (next wp %u)\n",
               g_routeId, g_routeCount, g_routeGuide.progressM, g_routeGuide.lengthM,
               g_routeCurrentIndex + 1);
    return true;
}

static void updateRouteGuidance() {
    if (!g_routeRunning) return;

    if (!gps.location.isValid() || gps.location.age() > ROUTE_GPS_STALE_MS) {
        abortRoute(ROUTE_ABORT_NO_GPS_FIX);
        return;
    }

    // Compass down: pause guidance while it recovers. loop() aborts the route
    // if it stays down for COMPASS_LOST_HOLD_MS.
    if (!compassHeadingValid()) return;

    g_routeGuide = routeTrackUpdate(g_routeTrack, gps.location.lat(), gps.location.lng(),
                                    ROUTE_LOOKAHEAD_M, g_routeFinishRadiusM);
    g_routeDistanceM = (uint16_t)constrain(g_routeGuide.nextDistanceM, 0.0f, 65535.0f);

    // A waypoint counts as reached once the boat is past it along the track,
    // however close it came.
    while (g_routeCurrentIndex < g_routeGuide.nextIndex) {
        broadcastWaypointReached(g_routeCurrentIndex++);
    }

    if (g_routeGuide.finished) {
        while (g_routeCurrentIndex < g_routeCount) {
            broadcastWaypointReached(g_routeCurrentIndex++);
        }
        g_routeRunning = false;
        broadcastRouteComplete();
        broadcastRouteStatus(ROUTE_STATE_COMPLETE);
        disengageHeadingHold();
        sendSteeringSpeed(0.0f);
        Log.println("[Route] COMPLETE");
        return;
    }

    if (!g_headingHoldActive) {
        engageHeadingHold();
    }
    g_targetHeading = g_routeGuide.targetHeading;
}

// ---------------------------------------------------------------------------
// ESP-NOW receive callback
// ---------------------------------------------------------------------------
static void onMeshReceive(const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len) {
    if (meshRemoteReceive(data, len)) return;
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
        Log.printf("[APTune] param=%u val=%.4f (Kp=%.4f Ki=%.4f Kd=%.3f max=%.2f dead=%.1f rate=%.3f)\n",
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
        g_routeStartRequested = false;
        memset(g_routePoints, 0, sizeof(g_routePoints));
        broadcastRouteStatus(ROUTE_STATE_UPLOADING);
        Log.printf("[Route] START upload route=%u wps=%u chunks=%u\n",
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
        Log.printf("[Route] Upload complete route=%u wps=%u\n", g_routeId, g_routeCount);
        return;
    }

    if (type == MSG_ROUTE_CONTROL && len >= static_cast<int>(sizeof(RouteControlMessage))) {
        RouteControlMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_HANDHELD) return;
        if (m.routeId != g_routeId) return;

        if (m.action == ROUTE_ACTION_START) {
            g_routeStartRequested = true;
        } else if (m.action == ROUTE_ACTION_STOP) {
            g_routeStartRequested = false;
            abortRoute(ROUTE_ABORT_STOP_REQUESTED);
        }
        return;
    }
}

static const char* resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "power-on";
        case ESP_RST_EXT:       return "reset pin";
        case ESP_RST_SW:        return "software";
        case ESP_RST_PANIC:     return "crash";
        case ESP_RST_INT_WDT:   return "interrupt watchdog";
        case ESP_RST_TASK_WDT:  return "task watchdog";
        case ESP_RST_WDT:       return "watchdog";
        case ESP_RST_DEEPSLEEP: return "deep sleep";
        case ESP_RST_BROWNOUT:  return "brownout";
        default:                return "other";
    }
}

// ---------------------------------------------------------------------------
// Commands, typed into the serial port or sent through the handheld
// ---------------------------------------------------------------------------
// 'c' starts compass calibration.
static void handleCommand(char ch) {
    if (ch == 'c' || ch == 'C') {
        if (g_calActive) {
            Log.println("[Cal] Calibration already in progress…");
        } else {
            calStart();
        }
    }
}

static void onRemoteCommand(const char* text) {
    for (const char* p = text; *p; ++p) handleCommand(*p);
}

// New firmware isn't accepted while the autopilot is steering the boat.
static bool canUpdateFirmware() {
    return !g_headingHoldActive && !g_routeRunning;
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);
    Log.println("\n=== Sauna Boat Navigation Module ===");
    g_resetReason = resetReasonName(esp_reset_reason());
    Log.printf("[Setup] Reset reason: %s\n", g_resetReason);

    // --- GPS (UART2) ---
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Log.printf("[Setup] GPS UART2 started (RX=%d TX=%d @ %d baud)\n",
               GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

    // --- BMM150 (I2C) ---
    // Don't wait here for the compass: if it's wedged after a reset the mesh
    // would never come up. loop() keeps retrying until it responds.
    if (compassTryInit(millis())) {
        Log.println("[Setup] BMM150 magnetometer initialised (30 Hz, high accuracy).");
    } else {
        Log.println("[Setup] WARNING: BMM150 not responding — will keep retrying.");
    }

    // --- NVS ---
    prefs.begin("nav", false);
    calLoadFromNVS();
    loadTuningFromNVS();

    // --- WiFi + ESP-NOW ---
    // Arduino 3 starts the station in the background; the channel can only be
    // set, and ESP-NOW can only send, once it has started.
    WiFi.mode(WIFI_STA);
    for (int i = 0; i < 200 && !WiFi.STA.started(); ++i) delay(10);
    // The station defaults to modem sleep, which (without an access point to
    // keep the radio awake) starves ESP-NOW: sends queue up and fail with
    // ESP_ERR_ESPNOW_NO_MEM.
    WiFi.setSleep(false);
    esp_err_t chanErr = esp_wifi_set_channel(MESH_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    uint8_t channel = 0;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&channel, &second);
    Log.printf("[Setup] WiFi MAC: %s  started=%d  channel %u (set: %s)\n",
               WiFi.macAddress().c_str(), WiFi.STA.started() ? 1 : 0, channel,
               esp_err_to_name(chanErr));

    if (esp_now_init() != ESP_OK) {
        Log.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Log.println("[Setup] ESP-NOW initialised.");
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onMeshReceive);

    // Register the broadcast peer so this module can send to the mesh.
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, MESH_BROADCAST_ADDR, 6);
    peer.channel = MESH_WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;  // receive/send via STA interface
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
        Log.println("[Setup] Broadcast peer registered.");
    } else {
        Log.println("[Setup] WARNING: Failed to register broadcast peer.");
    }
    meshRemoteBegin(MODULE_NAVIGATION, canUpdateFirmware, onRemoteCommand);

    // If loop() ever hangs, reboot (after 5 s) rather than sit dead until
    // someone power-cycles the module.
    enableLoopWDT();

    Log.println("[Setup] Navigation module ready.");
    Log.println("[Setup] Send 'c' over serial or the mesh gateway to calibrate compass.");
    Log.println("[Setup] D-pad UP on controller to engage heading hold.");
}

// ---------------------------------------------------------------------------
// Arduino loop
// ---------------------------------------------------------------------------
void loop() {
    if (meshRemoteLoop()) return;  // receiving new firmware

    static unsigned long lastBroadcast  = 0;
    static unsigned long lastLog        = 0;
    static unsigned long lastCalProg    = 0;
    static unsigned long lastAutopilot  = 0;
    static unsigned long lastApSend     = 0;
    unsigned long now = millis();

    // --- Serial command handling ---
    while (Serial.available()) {
        handleCommand(Serial.read());
    }

    // Feed all available GPS bytes into the parser.
    while (Serial2.available()) {
        gps.encode(Serial2.read());
    }

    // Update compass heading (also feeds calibration when active, and
    // re-initialises the sensor if it has stopped responding).
    updateCompass(now);

    // While the compass is down the autopilot is paused (see below). If it
    // doesn't come back, hand control back to the operator.
    if (g_headingHoldActive && !compassHeadingValid() &&
        now - g_compassLastHeadingMs >= COMPASS_LOST_HOLD_MS) {
        Log.println("[AP] Compass lost — releasing heading hold.");
        if (g_routeRunning) {
            abortRoute(ROUTE_ABORT_NO_HEADING);
        } else {
            disengageHeadingHold();
        }
    }

    // --- Controller input → heading hold ---
    if (g_ctrlOnline) {
        bool upNow   = (g_ctrlBtns & CTRL_BTN_UP)   != 0;
        bool upPrev  = (g_prevBtns & CTRL_BTN_UP)   != 0;
        bool dnNow   = (g_ctrlBtns & CTRL_BTN_DOWN) != 0;
        bool dnPrev  = (g_prevBtns & CTRL_BTN_DOWN) != 0;
        if (upNow && !upPrev && compassHeadingValid()) {
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

    if (g_routeStartRequested) {
        g_routeStartRequested = false;
        startRouteExecution();
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
                    Log.println("[AP] HOLD YIELDING to manual stick input.");
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
                    Log.println("[AP] HOLD RESUMED.");
                }
                g_headingHoldYielding = false;

                if (now - lastAutopilot >= AUTOPILOT_MS) {
                    lastAutopilot = now;
                    if (compassHeadingValid()) {
                        autopilotUpdate();
                    } else {
                        // No heading: stop moving the rudder until the
                        // compass recovers.
                        g_apOutput = 0.0f;
                    }
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
            Log.printf("[Cal] Collecting… %lus remaining  "
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

    // Periodic serial diagnostics.
    if (!g_calActive && (now - lastLog >= DIAG_LOG_MS)) {
        lastLog = now;

        const char* calTag = !compassHeadingValid() ? "NO COMPASS"
                           : g_calibrated           ? "CAL" : "UNCAL";

        if (g_headingHoldActive) {
            float err = headingError(g_heading, g_targetHeading);
            Log.printf("[AP] %s %.1f°  hdg=%.1f°  err=%+.1f°  steer=%+.3f\n",
                       g_headingHoldYielding ? "YIELD" : "HOLD",
                       g_targetHeading, g_heading, err, g_apOutput);
        } else {
            Log.printf("[AP] MANUAL  hdg=%.1f° [%s]  stick=%.2f\n",
                       g_heading, calTag, g_stickLX);
        }

        if (g_routeRunning) {
            Log.printf("[Route] next wp %u/%u %um  along %.0f/%.0fm  off-track %+.1fm\n",
                       g_routeCurrentIndex + 1, g_routeCount, g_routeDistanceM,
                       g_routeGuide.progressM, g_routeGuide.lengthM, g_routeGuide.crossTrackM);
        }

        if (gps.location.isValid()) {
            Log.printf("[Nav] GPS: %.6f, %.6f  sats=%lu  spd=%.1fkn  crs=%.1f°\n",
                       gps.location.lat(), gps.location.lng(),
                       (unsigned long)gps.satellites.value(),
                       gps.speed.knots(), gps.course.deg());
        } else if (gps.charsProcessed() < 10) {
            Log.printf("[Nav] GPS: NO DATA (check wiring: TX→GPIO%d, baud=%d)\n",
                       GPS_RX_PIN, GPS_BAUD);
        } else {
            Log.printf("[Nav] GPS: WAITING FOR FIX  sats=%lu  sentences=%lu  failed=%lu\n",
                       (unsigned long)gps.satellites.value(),
                       gps.sentencesWithFix(),
                       gps.failedChecksum());
        }
    }

    delay(10);
}
