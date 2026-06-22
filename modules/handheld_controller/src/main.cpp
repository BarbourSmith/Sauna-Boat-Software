// main.cpp – Sauna Boat Handheld Controller Module
//
// ESP32-S3 (Pavloff board) — custom handheld controller with an analog
// joystick, an SSD1306 0.96" OLED, an MPU-6050 (used for wake-on-motion),
// and an on-board LiPo + battery monitor.
//
// Behaviour from the boat's perspective:
// it broadcasts MSG_CONTROLLER_INPUT and MSG_SET_STEERING over ESP-NOW on
// MESH_WIFI_CHANNEL, so steering / navigation receivers need no changes.
//
// Display shows controller diagnostics (battery, Wi-Fi/ESP-NOW link status,
// steering status, joystick values) for field debugging.
//
// Power management: after IDLE_SLEEP_MS of no joystick deflection or button
// press the board enters deep sleep. It wakes on either MPU-6050 motion
// (EXT1 / GPIO 18) or joystick button click (EXT0 / GPIO 6 pulled LOW),
// depending on settings.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <Preferences.h>

#include <MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "mesh_protocol.h"

// ---------------------------------------------------------------------------
// Pin definitions — Pavloff board (fixed)
// ---------------------------------------------------------------------------
#define SDA_PIN       15   // I2C SDA — MPU-6050 + OLED share this bus
#define SCL_PIN       16   // I2C SCL
#define MPU_INT_PIN   18   // MPU-6050 INT → EXT1 wake-up source
#define BATTERY_PIN    7   // ADC battery voltage divider
#define BLUE_LED_PIN  47   // Status LED (active HIGH)

// ---------------------------------------------------------------------------
// Pin definitions — joystick (defaults; rewire as needed for your build)
// ---------------------------------------------------------------------------
#define JOY_X_PIN      4   // analog X (ADC1_CH3)
#define JOY_Y_PIN      5   // analog Y (ADC1_CH4)
#define JOY_BTN_PIN    6   // digital button, INPUT_PULLUP — active LOW

// ---------------------------------------------------------------------------
// Battery — voltage divider on BATTERY_PIN: R1 = 27k (Vbat→pin), R2 = 68k (pin→GND)
// ---------------------------------------------------------------------------
constexpr float BATTERY_R1            = 27000.0f;
constexpr float BATTERY_R2            = 68000.0f;
constexpr float BATTERY_DIVIDER_RATIO = BATTERY_R2 / (BATTERY_R1 + BATTERY_R2);
// Calibrated so a freshly charged remote (observed around 4.10V) reports 100%.
constexpr float BATTERY_VOLTAGE_FULL  = 4.10f;
constexpr float BATTERY_VOLTAGE_EMPTY = 3.00f;

// ---------------------------------------------------------------------------
// Joystick calibration / behaviour
// ---------------------------------------------------------------------------
// 12-bit ADC; centre is stored in NVS and can be recalibrated from settings.
constexpr int   JOY_ADC_MAX        = 4095;
constexpr int   JOY_DEAD_COUNTS    = 120;   // raw counts around centre treated as zero
constexpr float JOY_OUTPUT_GAMMA   = 1.0f;  // >1 = softer near centre; 1 = linear
constexpr int   STEERING_TRIM_MIN  = -600;
constexpr int   STEERING_TRIM_MAX  = 600;
constexpr int   STEERING_TRIM_STEP = 10;

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
constexpr unsigned long SEND_INTERVAL_MS    = 50;    // 20 Hz mesh broadcast
constexpr unsigned long DISPLAY_INTERVAL_MS = 100;   // 10 Hz OLED refresh
constexpr unsigned long BATTERY_INTERVAL_MS = 5000;
constexpr unsigned long LOG_INTERVAL_MS     = 1000;
constexpr unsigned long MENU_NAV_REPEAT_MS  = 220;
constexpr unsigned long MENU_BTN_DEBOUNCE_MS = 180;

// ---------------------------------------------------------------------------
// Sleep timeout — enter deep sleep after this much idle (no stick / button).
// ---------------------------------------------------------------------------
constexpr bool DISABLE_SLEEP_FOR_DEBUG = false;         // keep false for normal operation
constexpr unsigned long IDLE_SLEEP_MS = 60UL * 1000UL;  // 1 minute

// ---------------------------------------------------------------------------
// OLED — SSD1306 128x64 over I2C, default address 0x3C
// ---------------------------------------------------------------------------
constexpr uint8_t OLED_ADDR    = 0x3C;
constexpr int     OLED_WIDTH   = 128;
constexpr int     OLED_HEIGHT  = 64;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
MPU6050           mpu;
Adafruit_SSD1306  display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Preferences       preferences;

static int   g_joyCenterX     = 2048;
static int   g_joyCenterY     = 2048;
static int   g_steeringTrimCounts = 0;
static bool  g_oledOk         = false;
static bool  g_mpuOk          = false;

static float g_batteryVoltage = 0.0f;
static int   g_batteryPercent = 0;

// Latest telemetry received from the boat (NaN until first packet).
static float        g_navHeading       = NAN;
static float        g_navCourse        = NAN;
static float        g_navSpeedKnots    = NAN;
static uint8_t      g_navFixType       = 0;
static uint8_t      g_navSatellites    = 0;
static unsigned long g_lastNavMs       = 0;
static bool         g_navHoldActive    = false;
static float        g_navHoldTarget    = NAN;
static float        g_navSteerCmd      = NAN;
static unsigned long g_lastNavSteerMs  = 0;

static float        g_steeringAngle    = NAN;
static float        g_steeringTarget   = NAN;
static unsigned long g_lastSteeringMs  = 0;

// Last time the user did anything (used for idle-sleep timeout).
static unsigned long g_lastActivityMs  = 0;

// ESP-NOW diagnostics
static uint32_t g_sendOkCount   = 0;
static uint32_t g_sendFailCount = 0;
static bool g_lastSendOk        = false;
static unsigned long g_lastSendMs = 0;

// Whether inactivity timeout is allowed to put the controller into deep sleep.
// Toggleable via the menu settings and persisted under `idleSleep`.
static bool g_idleSleepEnabled = true;

// Whether joystick click (EXT0 on JOY_BTN_PIN) is armed as a wake source.
// Toggleable via the "settings" preferences namespace, key `wakeOnClick`.
static bool g_wakeOnClick = true;

// Whether to arm the MPU-6050 motion interrupt before sleeping.
// Toggleable via the "settings" preferences namespace, key `wakeOnMove`.
static bool g_wakeOnMovement = true;

enum class MenuScreen : uint8_t {
    NONE,
    ROOT,
    SETTINGS,
    NAVIGATION,
    ROUTES,
    ROUTE_RENAME,
    NAV_INFO,
    STEERING_TRIM,
    BATTERY,
    AP_TUNING
};

static MenuScreen g_menuScreen = MenuScreen::NONE;
static int g_menuSelectionRoot = 0;
static int g_menuSelectionSettings = 0;
static int g_menuSelectionNav  = 0;
static int g_menuSelectionRoutes = 0;
static int g_menuSelectionApTuning = 0;
static bool g_settingsDirty = false;
static uint8_t g_holdRequestPulses   = 0;
static uint8_t g_holdDisengagePulses = 0;

struct RouteStore {
    uint8_t count;
    WaypointData points[ROUTE_MAX_WAYPOINTS_PER_ROUTE];
};

static RouteStore g_routes[ROUTE_MAX_ROUTES] = {};
static uint8_t g_activeRouteId = 0;
static uint8_t g_selectedWaypointIndex = 0;
static char g_routeNames[ROUTE_MAX_ROUTES][16] = {};
static char g_renameBuffer[16] = {};
static uint8_t g_renameCursor = 0;
static char g_routeNotice[32] = "Route idle";

static float g_navLat = NAN;
static float g_navLon = NAN;

enum class RouteTxStage : uint8_t {
    IDLE,
    SEND_START,
    SEND_CHUNK,
    SEND_END,
    SEND_CONTROL,
};

static RouteTxStage g_routeTxStage = RouteTxStage::IDLE;
static uint8_t g_routeTxRouteId = 0;
static uint8_t g_routeTxChunkIndex = 0;
static unsigned long g_routeTxNextMs = 0;

// Local mirror of nav autopilot tuning values (received via MSG_AP_TUNING echo or initialised on connect)
static float g_apKp        = 0.008f;
static float g_apKi        = 0.0f;
static float g_apKd        = 0.2f;
static float g_apMaxOutput = 1.0f;
static float g_apDeadband  = 3.0f;
static float g_apRateLimit = 0.05f;

static bool g_prevButtonPressed = false;
static unsigned long g_lastMenuMoveMs = 0;
static unsigned long g_lastButtonEventMs = 0;

static constexpr uint16_t ROUTE_RADIUS_MIN_M = 1;
static constexpr uint16_t ROUTE_RADIUS_MAX_M = 100;
static constexpr char ROUTE_RENAME_ALPHABET[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_";

static void setRouteNotice(const char* text);
static void saveRoutesToNVS();

static void setDefaultRouteName(uint8_t routeId, char* out, size_t outSize) {
    snprintf(out, outSize, "Route %u", (unsigned)(routeId + 1));
}

static void initRouteNames() {
    for (uint8_t i = 0; i < ROUTE_MAX_ROUTES; ++i) {
        setDefaultRouteName(i, g_routeNames[i], sizeof(g_routeNames[i]));
    }
}

static void clampSelectedWaypoint() {
    uint8_t count = g_routes[g_activeRouteId].count;
    if (count == 0) {
        g_selectedWaypointIndex = 0;
        return;
    }
    if (g_selectedWaypointIndex >= count) {
        g_selectedWaypointIndex = count - 1;
    }
}

static WaypointData* selectedWaypoint() {
    RouteStore& route = g_routes[g_activeRouteId];
    if (route.count == 0) return nullptr;
    clampSelectedWaypoint();
    return &route.points[g_selectedWaypointIndex];
}

static void selectNextWaypoint(int delta) {
    RouteStore& route = g_routes[g_activeRouteId];
    if (route.count == 0) {
        setRouteNotice("No waypoints");
        return;
    }

    int next = (int)g_selectedWaypointIndex + delta;
    if (next < 0) next = route.count - 1;
    if (next >= route.count) next = 0;
    g_selectedWaypointIndex = (uint8_t)next;
    setRouteNotice("Waypoint selected");
}

static void adjustSelectedWaypointRadius(int delta) {
    WaypointData* wp = selectedWaypoint();
    if (!wp) {
        setRouteNotice("No waypoints");
        return;
    }

    int next = (int)wp->radiusM + delta;
    if (next < (int)ROUTE_RADIUS_MIN_M) next = ROUTE_RADIUS_MIN_M;
    if (next > (int)ROUTE_RADIUS_MAX_M) next = ROUTE_RADIUS_MAX_M;
    wp->radiusM = (uint16_t)next;
    saveRoutesToNVS();
    setRouteNotice("Radius updated");
}

static void deleteSelectedWaypoint() {
    RouteStore& route = g_routes[g_activeRouteId];
    if (route.count == 0) {
        setRouteNotice("No waypoints");
        return;
    }

    clampSelectedWaypoint();
    for (uint8_t i = g_selectedWaypointIndex + 1; i < route.count; ++i) {
        route.points[i - 1] = route.points[i];
    }
    --route.count;
    clampSelectedWaypoint();
    saveRoutesToNVS();
    setRouteNotice("Waypoint deleted");
}

static void beginRenameRoute() {
    snprintf(g_renameBuffer, sizeof(g_renameBuffer), "%s", g_routeNames[g_activeRouteId]);
    g_renameCursor = 0;
    g_menuScreen = MenuScreen::ROUTE_RENAME;
    setRouteNotice("Rename route");
}

static int renameAlphabetIndex(char ch) {
    const char* p = strchr(ROUTE_RENAME_ALPHABET, ch);
    if (!p) return 0;
    return (int)(p - ROUTE_RENAME_ALPHABET);
}

static void renameAdjustChar(int delta) {
    size_t n = strlen(ROUTE_RENAME_ALPHABET);
    if (n == 0 || g_renameCursor >= sizeof(g_renameBuffer) - 1) return;

    char cur = g_renameBuffer[g_renameCursor];
    int idx = renameAlphabetIndex(cur);
    idx += delta;
    if (idx < 0) idx = (int)n - 1;
    if (idx >= (int)n) idx = 0;
    g_renameBuffer[g_renameCursor] = ROUTE_RENAME_ALPHABET[idx];
}

static void renameMoveCursor(int delta) {
    int next = (int)g_renameCursor + delta;
    int maxPos = (int)sizeof(g_renameBuffer) - 2;
    if (next < 0) next = maxPos;
    if (next > maxPos) next = 0;
    g_renameCursor = (uint8_t)next;
}

static void commitRenameRoute() {
    // Trim trailing spaces to keep display compact.
    int end = (int)strlen(g_renameBuffer) - 1;
    while (end >= 0 && g_renameBuffer[end] == ' ') {
        g_renameBuffer[end] = '\0';
        --end;
    }
    if (strlen(g_renameBuffer) == 0) {
        setDefaultRouteName(g_activeRouteId, g_routeNames[g_activeRouteId], sizeof(g_routeNames[g_activeRouteId]));
    } else {
        snprintf(g_routeNames[g_activeRouteId], sizeof(g_routeNames[g_activeRouteId]), "%s", g_renameBuffer);
    }
    saveRoutesToNVS();
    setRouteNotice("Name saved");
}

static void setRouteNotice(const char* text) {
    snprintf(g_routeNotice, sizeof(g_routeNotice), "%s", text);
}

static void saveRoutesToNVS() {
    preferences.begin("routes", false);
    preferences.putUChar("ver", 1);
    preferences.putUChar("active", g_activeRouteId);

    for (uint8_t i = 0; i < ROUTE_MAX_ROUTES; ++i) {
        char countKey[8] = {};
        char dataKey[8] = {};
        char nameKey[8] = {};
        snprintf(countKey, sizeof(countKey), "c%u", i);
        snprintf(dataKey, sizeof(dataKey), "r%u", i);
        snprintf(nameKey, sizeof(nameKey), "n%u", i);

        uint8_t count = g_routes[i].count;
        if (count > ROUTE_MAX_WAYPOINTS_PER_ROUTE) {
            count = ROUTE_MAX_WAYPOINTS_PER_ROUTE;
        }

        preferences.putUChar(countKey, count);
        preferences.putString(nameKey, g_routeNames[i]);
        if (count > 0) {
            preferences.putBytes(dataKey, g_routes[i].points, count * sizeof(WaypointData));
        }
    }
    preferences.end();
}

static void loadRoutesFromNVS() {
    memset(g_routes, 0, sizeof(g_routes));
    g_activeRouteId = 0;
    g_selectedWaypointIndex = 0;
    initRouteNames();

    preferences.begin("routes", true);
    uint8_t ver = preferences.getUChar("ver", 0);
    if (ver == 1) {
        g_activeRouteId = preferences.getUChar("active", 0);
        if (g_activeRouteId >= ROUTE_MAX_ROUTES) g_activeRouteId = 0;

        for (uint8_t i = 0; i < ROUTE_MAX_ROUTES; ++i) {
            char countKey[8] = {};
            char dataKey[8] = {};
            char nameKey[8] = {};
            snprintf(countKey, sizeof(countKey), "c%u", i);
            snprintf(dataKey, sizeof(dataKey), "r%u", i);
            snprintf(nameKey, sizeof(nameKey), "n%u", i);

            String routeName = preferences.getString(nameKey, "");
            if (routeName.length() > 0) {
                routeName.toCharArray(g_routeNames[i], sizeof(g_routeNames[i]));
            }

            uint8_t count = preferences.getUChar(countKey, 0);
            if (count > ROUTE_MAX_WAYPOINTS_PER_ROUTE) count = ROUTE_MAX_WAYPOINTS_PER_ROUTE;
            g_routes[i].count = count;

            if (count > 0) {
                size_t got = preferences.getBytes(dataKey, g_routes[i].points, count * sizeof(WaypointData));
                if (got != count * sizeof(WaypointData)) {
                    g_routes[i].count = 0;
                }
            }
        }
    }
    preferences.end();

    clampSelectedWaypoint();
}

static bool hasFreshNavFix(unsigned long now) {
    bool fresh = (g_lastNavMs != 0) && (now - g_lastNavMs < 2000);
    bool valid = g_navFixType > 0 && !isnan(g_navLat) && !isnan(g_navLon);
    return fresh && valid;
}

static void appendCurrentLocationToActiveRoute(unsigned long now) {
    if (!hasFreshNavFix(now)) {
        setRouteNotice("No GPS fix");
        return;
    }

    RouteStore& route = g_routes[g_activeRouteId];
    if (route.count >= ROUTE_MAX_WAYPOINTS_PER_ROUTE) {
        setRouteNotice("Route full");
        return;
    }

    WaypointData wp = {};
    wp.latE7 = (int32_t)lroundf(g_navLat * 10000000.0f);
    wp.lonE7 = (int32_t)lroundf(g_navLon * 10000000.0f);
    wp.radiusM = ROUTE_DEFAULT_RADIUS_M;
    route.points[route.count++] = wp;
    g_selectedWaypointIndex = route.count - 1;
    saveRoutesToNVS();
    setRouteNotice("Waypoint added");
}

static void clearActiveRoute() {
    g_routes[g_activeRouteId].count = 0;
    g_selectedWaypointIndex = 0;
    saveRoutesToNVS();
    setRouteNotice("Route cleared");
}

static uint8_t routeChunkCount(uint8_t waypointCount) {
    return (waypointCount + ROUTE_WAYPOINTS_PER_CHUNK - 1) / ROUTE_WAYPOINTS_PER_CHUNK;
}

static void startRouteTransfer() {
    const RouteStore& route = g_routes[g_activeRouteId];
    if (route.count == 0) {
        setRouteNotice("Route empty");
        return;
    }

    g_routeTxRouteId = g_activeRouteId;
    g_routeTxChunkIndex = 0;
    g_routeTxStage = RouteTxStage::SEND_START;
    g_routeTxNextMs = 0;
    setRouteNotice("Uploading...");
}

static void processRouteTransfer(unsigned long now) {
    if (g_routeTxStage == RouteTxStage::IDLE) return;
    if (now < g_routeTxNextMs) return;

    const RouteStore& route = g_routes[g_routeTxRouteId];
    uint8_t totalChunks = routeChunkCount(route.count);

    if (g_routeTxStage == RouteTxStage::SEND_START) {
        RouteStartMessage start = {};
        start.type = MSG_ROUTE_START;
        start.src = MODULE_HANDHELD;
        start.routeId = g_routeTxRouteId;
        start.totalWaypoints = route.count;
        start.totalChunks = totalChunks;
        esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&start), sizeof(start));
        g_routeTxStage = RouteTxStage::SEND_CHUNK;
        g_routeTxNextMs = now + SEND_INTERVAL_MS;
        return;
    }

    if (g_routeTxStage == RouteTxStage::SEND_CHUNK) {
        if (g_routeTxChunkIndex >= totalChunks) {
            g_routeTxStage = RouteTxStage::SEND_END;
            g_routeTxNextMs = now + SEND_INTERVAL_MS;
            return;
        }

        RouteChunkMessage chunk = {};
        chunk.type = MSG_ROUTE_CHUNK;
        chunk.src = MODULE_HANDHELD;
        chunk.routeId = g_routeTxRouteId;
        chunk.chunkIndex = g_routeTxChunkIndex;

        uint8_t base = g_routeTxChunkIndex * ROUTE_WAYPOINTS_PER_CHUNK;
        uint8_t remaining = route.count - base;
        chunk.waypointCount = remaining > ROUTE_WAYPOINTS_PER_CHUNK ? ROUTE_WAYPOINTS_PER_CHUNK : remaining;
        for (uint8_t i = 0; i < chunk.waypointCount; ++i) {
            chunk.waypoints[i] = route.points[base + i];
        }

        esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&chunk), sizeof(chunk));
        ++g_routeTxChunkIndex;
        g_routeTxNextMs = now + SEND_INTERVAL_MS;
        return;
    }

    if (g_routeTxStage == RouteTxStage::SEND_END) {
        RouteEndMessage end = {};
        end.type = MSG_ROUTE_END;
        end.src = MODULE_HANDHELD;
        end.routeId = g_routeTxRouteId;
        end.crc16 = 0;
        esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&end), sizeof(end));
        g_routeTxStage = RouteTxStage::SEND_CONTROL;
        g_routeTxNextMs = now + SEND_INTERVAL_MS;
        return;
    }

    if (g_routeTxStage == RouteTxStage::SEND_CONTROL) {
        RouteControlMessage ctrl = {};
        ctrl.type = MSG_ROUTE_CONTROL;
        ctrl.src = MODULE_HANDHELD;
        ctrl.action = ROUTE_ACTION_START;
        ctrl.routeId = g_routeTxRouteId;
        esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&ctrl), sizeof(ctrl));
        g_routeTxStage = RouteTxStage::IDLE;
        setRouteNotice("Route sent");
    }
}

// ---------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------
static void readBatteryVoltage() {
    uint32_t mv = analogReadMilliVolts(BATTERY_PIN);
    g_batteryVoltage = (mv / 1000.0f) / BATTERY_DIVIDER_RATIO;

    if (g_batteryVoltage >= BATTERY_VOLTAGE_FULL) {
        g_batteryPercent = 100;
    } else if (g_batteryVoltage <= BATTERY_VOLTAGE_EMPTY) {
        g_batteryPercent = 0;
    } else {
        g_batteryPercent = (int)(((g_batteryVoltage - BATTERY_VOLTAGE_EMPTY) /
                                  (BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY)) * 100.0f);
    }
}

// ---------------------------------------------------------------------------
// Joystick
// ---------------------------------------------------------------------------
static void calibrateJoystick() {
    // Average several samples; user should release the stick while calibrating.
    long sx = 0, sy = 0;
    constexpr int N = 32;
    for (int i = 0; i < N; i++) {
        sx += analogRead(JOY_X_PIN);
        sy += analogRead(JOY_Y_PIN);
        delay(2);
    }
    g_joyCenterX = (int)(sy / N);  // X axis reads from JOY_Y_PIN (swapped for mount orientation)
    g_joyCenterY = (int)(sx / N);  // Y axis reads from JOY_X_PIN
}

static void saveJoystickCalibration() {
    preferences.begin("settings", false);
    preferences.putInt("joyCtrX", g_joyCenterX);
    preferences.putInt("joyCtrY", g_joyCenterY);
    preferences.end();
}

static void calibrateAndSaveJoystick() {
    calibrateJoystick();
    saveJoystickCalibration();
}

static int steeringCenterCounts() {
    return constrain(g_joyCenterY + g_steeringTrimCounts,
                     JOY_DEAD_COUNTS + 1,
                     JOY_ADC_MAX - JOY_DEAD_COUNTS - 1);
}

static void loadSettings() {
    preferences.begin("settings", true);
    g_idleSleepEnabled = preferences.getBool("idleSleep", true);
    g_wakeOnClick = preferences.getBool("wakeOnClick", true);
    g_wakeOnMovement = preferences.getBool("wakeOnMove", true);
    g_steeringTrimCounts = preferences.getInt("steerTrim", 0);
    g_joyCenterX = preferences.getInt("joyCtrX", 2048);
    g_joyCenterY = preferences.getInt("joyCtrY", 2048);
    preferences.end();

    g_steeringTrimCounts = constrain(g_steeringTrimCounts,
                                     STEERING_TRIM_MIN,
                                     STEERING_TRIM_MAX);
    g_settingsDirty = false;
}

static void saveSettings() {
    preferences.begin("settings", false);
    preferences.putBool("idleSleep", g_idleSleepEnabled);
    preferences.putBool("wakeOnClick", g_wakeOnClick);
    preferences.putBool("wakeOnMove", g_wakeOnMovement);
    preferences.putInt("steerTrim", g_steeringTrimCounts);
    preferences.putInt("joyCtrX", g_joyCenterX);
    preferences.putInt("joyCtrY", g_joyCenterY);
    preferences.end();
    g_settingsDirty = false;
}

// Convert a raw ADC reading to a signed [-1.0, 1.0] value with dead-zone.
static float joyNormalize(int raw, int center) {
    int delta = raw - center;
    if (delta > -JOY_DEAD_COUNTS && delta < JOY_DEAD_COUNTS) return 0.0f;

    float span;
    if (delta > 0) {
        delta -= JOY_DEAD_COUNTS;
        span   = (float)(JOY_ADC_MAX - center - JOY_DEAD_COUNTS);
    } else {
        delta += JOY_DEAD_COUNTS;
        span   = (float)(center - JOY_DEAD_COUNTS);
    }
    if (span < 1.0f) return 0.0f;

    float norm = (float)delta / span;
    if (norm >  1.0f) norm =  1.0f;
    if (norm < -1.0f) norm = -1.0f;

    if (JOY_OUTPUT_GAMMA != 1.0f) {
        float sign = (norm < 0) ? -1.0f : 1.0f;
        norm = sign * powf(fabsf(norm), JOY_OUTPUT_GAMMA);
    }
    return norm;
}

struct JoystickReading {
    float    x;
    float    y;
    bool     button;
    uint16_t rawX;
    uint16_t rawY;
};

static JoystickReading readJoystick() {
    JoystickReading r;
    r.rawX   = analogRead(JOY_Y_PIN);
    r.rawY   = analogRead(JOY_X_PIN);
    r.x      = joyNormalize(r.rawX, steeringCenterCounts());
    r.y      = -joyNormalize(r.rawY, g_joyCenterX);
    r.button = digitalRead(JOY_BTN_PIN) == LOW;
    return r;
}

// ---------------------------------------------------------------------------
// ESP-NOW
// ---------------------------------------------------------------------------
static void onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    g_lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
    g_lastSendMs = millis();
    if (g_lastSendOk) ++g_sendOkCount;
    else              ++g_sendFailCount;
}

static void onDataRecv(const uint8_t* /*mac*/,
                       const uint8_t* data, int len) {
    if (len < 2) return;
    uint8_t type = data[0];

    if (type == MSG_NAV_STATUS && len >= (int)sizeof(NavigationMessage)) {
        NavigationMessage m;
        memcpy(&m, data, sizeof(m));
        g_navHeading    = m.heading;
        g_navCourse     = m.course;
        g_navSpeedKnots = m.speedKnots;
        g_navLat        = m.lat;
        g_navLon        = m.lon;
        g_navFixType    = m.fixType;
        g_navSatellites = m.satellites;
        g_lastNavMs     = millis();
        return;
    }

    if (type == MSG_ANGLE_STATUS && len >= (int)sizeof(MeshMessage)) {
        MeshMessage m;
        memcpy(&m, data, sizeof(m));
        g_steeringAngle  = m.value1;
        g_steeringTarget = m.value2;
        g_lastSteeringMs = millis();
        return;
    }

    if (type == MSG_HEADING_HOLD_STATUS && len >= (int)sizeof(MeshMessage)) {
        MeshMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_NAVIGATION) return;
        g_navHoldActive = (m.value1 > 0.5f);
        g_navHoldTarget = g_navHoldActive ? m.value2 : NAN;
        return;
    }

    if (type == MSG_AP_TUNING_STATUS && len >= (int)sizeof(MeshMessage)) {
        MeshMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_NAVIGATION) return;

        switch ((uint8_t)m.value1) {
            case AP_PARAM_KP:
                g_apKp = m.value2;
                break;
            case AP_PARAM_KI:
                g_apKi = m.value2;
                break;
            case AP_PARAM_KD:
                g_apKd = m.value2;
                break;
            case AP_PARAM_MAX_OUTPUT:
                g_apMaxOutput = m.value2;
                break;
            case AP_PARAM_DEADBAND:
                g_apDeadband = m.value2;
                break;
            case AP_PARAM_RATE_LIMIT:
                g_apRateLimit = m.value2;
                break;
            default:
                break;
        }
        return;
    }

    if (type == MSG_SET_STEERING && len >= (int)sizeof(MeshMessage)) {
        MeshMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_NAVIGATION) return;
        g_navSteerCmd = constrain(m.value1, -1.0f, 1.0f);
        g_lastNavSteerMs = millis();
        return;
    }

    if (type == MSG_ROUTE_STATUS && len >= (int)sizeof(RouteStatusMessage)) {
        RouteStatusMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_NAVIGATION) return;
        if (m.state == ROUTE_STATE_RUNNING) {
            snprintf(g_routeNotice, sizeof(g_routeNotice), "Run %u/%u %um",
                     (unsigned)(m.currentIndex + 1),
                     (unsigned)m.totalWaypoints,
                     (unsigned)m.distanceM);
        } else if (m.state == ROUTE_STATE_READY) {
            setRouteNotice("Route ready");
        } else if (m.state == ROUTE_STATE_UPLOADING) {
            setRouteNotice("Uploading...");
        }
        return;
    }

    if (type == MSG_WAYPOINT_REACHED && len >= (int)sizeof(WaypointReachedMessage)) {
        WaypointReachedMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_NAVIGATION) return;
        snprintf(g_routeNotice, sizeof(g_routeNotice), "Reached wp %u", (unsigned)(m.waypointIndex + 1));
        return;
    }

    if (type == MSG_ROUTE_COMPLETE && len >= (int)sizeof(RouteCompleteMessage)) {
        RouteCompleteMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_NAVIGATION) return;
        setRouteNotice("Route complete");
        return;
    }

    if (type == MSG_ROUTE_ABORT && len >= (int)sizeof(RouteAbortMessage)) {
        RouteAbortMessage m;
        memcpy(&m, data, sizeof(m));
        if (m.src != MODULE_NAVIGATION) return;
        snprintf(g_routeNotice, sizeof(g_routeNotice), "Route abort %u", (unsigned)m.reason);
        return;
    }
}

static void sendApTuning(uint8_t paramId, float value) {
    MeshMessage msg;
    msg.type   = MSG_AP_TUNING;
    msg.src    = MODULE_HANDHELD;
    msg.value1 = (float)paramId;
    msg.value2 = value;
    esp_now_send(MESH_BROADCAST_ADDR, reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
    Serial.printf("[APTune] param=%u val=%.4f\n", paramId, value);
}

static void sendControllerInput(const JoystickReading& joy, uint16_t extraButtons = 0) {
    ControllerInputMessage msg = {};
    msg.type    = MSG_CONTROLLER_INPUT;
    msg.src     = MODULE_HANDHELD;
    msg.lx      = joy.x;
    msg.ly      = joy.y;
    msg.rx      = 0.0f;
    msg.ry      = 0.0f;
    msg.buttons = (joy.button ? CTRL_BTN_L3 : 0) | extraButtons;  // joystick click → L3
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static float steeringTrimBias() {
    // Convert trim counts to a steering-space offset and clamp to safe range.
    constexpr float spanCounts = (JOY_ADC_MAX / 2.0f) - JOY_DEAD_COUNTS;
    if (spanCounts <= 1.0f) return 0.0f;
    return constrain((float)g_steeringTrimCounts / spanCounts, -1.0f, 1.0f);
}

static void sendSteering(float stickX) {
    MeshMessage msg;
    msg.type   = MSG_SET_STEERING;
    msg.src    = MODULE_HANDHELD;
    float steerCmd = stickX + steeringTrimBias();
    msg.value1 = constrain(steerCmd, -1.0f, 1.0f);
    msg.value2 = 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void sendBatteryStatus() {
    MeshMessage msg;
    msg.type   = MSG_CONTROLLER_STATUS;
    msg.src    = MODULE_HANDHELD;
    // Use the shared controller-status battery scale: -1=unknown..4=full.
    msg.value1 = (g_batteryPercent <= 0)  ? 0.0f
               : (g_batteryPercent < 25)  ? 1.0f
               : (g_batteryPercent < 50)  ? 2.0f
               : (g_batteryPercent < 90)  ? 3.0f
               :                            4.0f;
    msg.value2 = 0.0f;  // no charge-detect line on this board
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

// ---------------------------------------------------------------------------
// MPU-6050 motion-interrupt setup (lifted from the Pavloff example)
// ---------------------------------------------------------------------------
static void configureMPUMotionInterrupt() {
    mpu.setIntEnabled(0x00);
    mpu.setIntFreefallEnabled(false);
    mpu.setIntMotionEnabled(false);
    mpu.setIntZeroMotionEnabled(false);

    mpu.setInterruptMode(false);       // active HIGH
    mpu.setInterruptDrive(false);      // push-pull
    mpu.setInterruptLatch(true);
    mpu.setInterruptLatchClear(true);

    mpu.setMotionDetectionThreshold(16);  // ~32 mg @ ±2g
    mpu.setMotionDetectionDuration(5);

    mpu.setDHPFMode(MPU6050_DHPF_RESET);
    delay(10);
    mpu.setDHPFMode(MPU6050_DHPF_5);

    mpu.setIntMotionEnabled(true);
}

static void wakeupMPU() {
    mpu.getIntStatus();
    mpu.setIntMotionEnabled(false);
    mpu.setWakeCycleEnabled(false);
    mpu.setSleepEnabled(false);

    mpu.setStandbyXGyroEnabled(false);
    mpu.setStandbyYGyroEnabled(false);
    mpu.setStandbyZGyroEnabled(false);
    mpu.setStandbyXAccelEnabled(false);
    mpu.setStandbyYAccelEnabled(false);
    mpu.setStandbyZAccelEnabled(false);

    mpu.setTempSensorEnabled(true);
    delay(200);
}

// ---------------------------------------------------------------------------
// Deep sleep
// ---------------------------------------------------------------------------
static void enterDeepSleep() {
    Serial.println("\n[Sleep] Entering deep sleep…");
    bool hasWakeSource = false;

    // Show a brief sleep notice on the OLED, then power it down.
    if (g_oledOk) {
        display.clearDisplay();
        display.setCursor(0, 24);
        display.setTextSize(2);
        display.println(F("  ZZZ"));
        display.setTextSize(1);
        display.display();
        delay(400);
        display.ssd1306_command(SSD1306_DISPLAYOFF);
    }

    // LED off + held during sleep.
    pinMode(BLUE_LED_PIN, OUTPUT);
    digitalWrite(BLUE_LED_PIN, LOW);
    gpio_hold_en((gpio_num_t)BLUE_LED_PIN);
    gpio_deep_sleep_hold_en();

    // EXT0: joystick button (active LOW) — optional wake-on-click source.
    if (g_wakeOnClick) {
        esp_sleep_enable_ext0_wakeup((gpio_num_t)JOY_BTN_PIN, 0);
        gpio_pullup_en((gpio_num_t)JOY_BTN_PIN);
        gpio_pulldown_dis((gpio_num_t)JOY_BTN_PIN);
        hasWakeSource = true;
    }

    // EXT1: MPU-6050 motion interrupt (active HIGH).
    if (g_wakeOnMovement && g_mpuOk) {
        configureMPUMotionInterrupt();
        mpu.setTempSensorEnabled(false);
        mpu.setStandbyXGyroEnabled(true);
        mpu.setStandbyYGyroEnabled(true);
        mpu.setStandbyZGyroEnabled(true);
        mpu.setStandbyXAccelEnabled(false);
        mpu.setStandbyYAccelEnabled(false);
        mpu.setStandbyZAccelEnabled(false);
        mpu.setWakeCycleEnabled(false);
        mpu.setSleepEnabled(false);
        mpu.getIntStatus();  // clear stale latch
        delay(200);

        uint64_t wakeMask = (1ULL << MPU_INT_PIN);
        esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_HIGH);
        gpio_pulldown_en((gpio_num_t)MPU_INT_PIN);
        gpio_pullup_dis((gpio_num_t)MPU_INT_PIN);
        hasWakeSource = true;
    } else if (g_mpuOk) {
        mpu.setSleepEnabled(true);
    }

    // Never enter deep sleep without any wake source configured.
    if (!hasWakeSource) {
        Serial.println("[Sleep] No wake source enabled; forcing wake-on-click");
        esp_sleep_enable_ext0_wakeup((gpio_num_t)JOY_BTN_PIN, 0);
        gpio_pullup_en((gpio_num_t)JOY_BTN_PIN);
        gpio_pulldown_dis((gpio_num_t)JOY_BTN_PIN);
    }

    delay(50);
    esp_deep_sleep_start();
}

// ---------------------------------------------------------------------------
// OLED rendering
// ---------------------------------------------------------------------------
static void drawDisplay(const JoystickReading& joy) {
    if (!g_oledOk) return;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    if (g_menuScreen != MenuScreen::NONE) {
        auto drawMenuLine = [](int y, bool selected, const char* text) {
            display.setCursor(0, y);
            display.print(selected ? F("> ") : F("  "));
            display.print(text);
        };

        if (g_menuScreen == MenuScreen::ROOT) {
            display.setCursor(0, 0);
            display.println(F("Menu"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
            drawMenuLine(14, g_menuSelectionRoot == 0, "Navigation");
            drawMenuLine(22, g_menuSelectionRoot == 1, "Settings");
            drawMenuLine(30, g_menuSelectionRoot == 2, "Steering Trim");
            drawMenuLine(38, g_menuSelectionRoot == 3, "Battery");
            drawMenuLine(46, g_menuSelectionRoot == 4, "Close");
            display.display();
            return;
        }

        if (g_menuScreen == MenuScreen::SETTINGS) {
            display.setCursor(0, 0);
            display.println(F("Settings"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

            display.setCursor(0, 14);
            display.print(g_menuSelectionSettings == 0 ? F("> ") : F("  "));
            display.print(F("Sleep Timeout: "));
            display.print(g_idleSleepEnabled ? F("ON") : F("OFF"));

            display.setCursor(0, 22);
            display.print(g_menuSelectionSettings == 1 ? F("> ") : F("  "));
            display.print(F("Wake on Click: "));
            display.print(g_wakeOnClick ? F("ON") : F("OFF"));

            display.setCursor(0, 30);
            display.print(g_menuSelectionSettings == 2 ? F("> ") : F("  "));
            display.print(F("Wake on Move: "));
            display.print(g_wakeOnMovement ? F("ON") : F("OFF"));

            drawMenuLine(38, g_menuSelectionSettings == 3, "Calibrate Stick");
            drawMenuLine(46, g_menuSelectionSettings == 4, "Sleep Now");
            drawMenuLine(54, g_menuSelectionSettings == 5, "Back");
            display.display();
            return;
        }

        if (g_menuScreen == MenuScreen::NAVIGATION) {
            display.setCursor(0, 0);
            display.println(F("Navigation"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
            drawMenuLine(14, g_menuSelectionNav == 0, "Heading Hold");
            drawMenuLine(24, g_menuSelectionNav == 1, "AP Tuning");
            drawMenuLine(34, g_menuSelectionNav == 2, "Routes");
            drawMenuLine(44, g_menuSelectionNav == 3, "Info");
            display.setCursor(96, 16);
            display.print(g_navHoldActive ? F("ON") : F("OFF"));
            drawMenuLine(54, g_menuSelectionNav == 4, "Back");
            display.display();
            return;
        }

        if (g_menuScreen == MenuScreen::ROUTES) {
            const RouteStore& route = g_routes[g_activeRouteId];
            int startIndex = g_menuSelectionRoutes - 2;
            if (startIndex < 0) startIndex = 0;
            if (startIndex > 4) startIndex = 4;

            display.setCursor(0, 0);
            display.println(F("Routes"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
            display.setCursor(0, 12);
            display.printf("%s", g_routeNames[g_activeRouteId]);
            display.setCursor(0, 20);
            if (route.count > 0) {
                uint8_t idx = g_selectedWaypointIndex;
                if (idx >= route.count) idx = route.count - 1;
                display.printf("WP %u/%u R%um", (unsigned)(idx + 1), (unsigned)route.count,
                               (unsigned)route.points[idx].radiusM);
            } else {
                display.print(F("WP 0/0"));
            }

            for (int row = 0; row < 5; ++row) {
                int item = startIndex + row;
                int y = 28 + row * 7;
                display.setCursor(0, y);
                display.print(g_menuSelectionRoutes == item ? F("> ") : F("  "));
                switch (item) {
                    case 0: display.print(F("Append current")); break;
                    case 1: display.print(F("Select waypoint")); break;
                    case 2: display.print(F("Delete waypoint")); break;
                    case 3: display.print(F("Radius +/-")); break;
                    case 4: display.print(F("Send + Start")); break;
                    case 5: display.print(F("Rename route")); break;
                    case 6: display.print(F("Next route")); break;
                    case 7: display.print(F("Clear route")); break;
                    default: display.print(F("Back")); break;
                }
            }
            display.display();
            return;
        }

        if (g_menuScreen == MenuScreen::ROUTE_RENAME) {
            display.setCursor(0, 0);
            display.println(F("Rename Route"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
            display.setCursor(0, 18);
            display.print(g_renameBuffer);
            display.setCursor(0, 30);
            for (uint8_t i = 0; i < sizeof(g_renameBuffer) - 1; ++i) {
                display.print(i == g_renameCursor ? '^' : ' ');
            }
            display.setCursor(0, 44);
            display.print(F("Y:cursor X:char"));
            display.setCursor(0, 54);
            display.print(F("Click: Save"));
            display.display();
            return;
        }

        if (g_menuScreen == MenuScreen::NAV_INFO) {
            display.setCursor(0, 0);
            display.println(F("Nav Info"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
            display.setCursor(0, 14);
            display.printf("Sat: %u", g_navSatellites);
            display.setCursor(64, 14);
            display.printf("Fix: %u", g_navFixType);
            display.setCursor(0, 24);
            if (!isnan(g_navHeading)) display.printf("HDG: %5.1f", g_navHeading);
            else                      display.print(F("HDG: ---"));
            display.setCursor(0, 34);
            if (!isnan(g_navCourse)) display.printf("COG: %5.1f", g_navCourse);
            else                     display.print(F("COG: ---"));
            display.setCursor(0, 44);
            if (!isnan(g_navSpeedKnots)) display.printf("SOG: %4.1fkn", g_navSpeedKnots);
            else                         display.print(F("SOG: ---"));
            display.setCursor(0, 54);
            display.println(F("Click: Back"));
            display.display();
            return;
        }

        if (g_menuScreen == MenuScreen::STEERING_TRIM) {
            display.setCursor(0, 0);
            display.println(F("Steering Trim"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
            display.setCursor(0, 18);
            display.printf("Trim: %+4d", g_steeringTrimCounts);
            display.setCursor(0, 30);
            display.printf("Ctr : %4d", steeringCenterCounts());
            display.setCursor(0, 42);
            display.println(F("Left/Right adjust"));
            display.setCursor(0, 54);
            display.println(F("Click: Back"));
            display.display();
            return;
        }

        if (g_menuScreen == MenuScreen::BATTERY) {
            display.setCursor(0, 0);
            display.println(F("Battery"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
            display.setCursor(0, 18);
            display.printf("%3d%%", g_batteryPercent);
            display.setCursor(0, 30);
            display.printf("%4.2fV", g_batteryVoltage);
            display.setCursor(0, 44);
            display.println(F("Details coming soon"));
            display.setCursor(0, 54);
            display.println(F("Click: Back"));
            display.display();
            return;
        }

        if (g_menuScreen == MenuScreen::AP_TUNING) {
            // 6 params: Kp Ki Kd Max Dead Rate
            static const char* const paramNames[] = { "Kp", "Ki", "Kd", "Max", "Dead", "Rate" };
            const float paramValues[] = { g_apKp, g_apKi, g_apKd, g_apMaxOutput, g_apDeadband, g_apRateLimit };
            int startIndex = g_menuSelectionApTuning - 2;
            if (startIndex < 0) startIndex = 0;
            if (startIndex > 1) startIndex = 1;
            display.setCursor(0, 0);
            display.println(F("AP Tuning"));
            display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);
            for (int row = 0; row < 5; ++row) {
                int i = startIndex + row;
                int y = 14 + row * 9;
                display.setCursor(0, y);
                display.print(g_menuSelectionApTuning == i ? F("> ") : F("  "));
                display.printf("%-4s %6.4f", paramNames[i], paramValues[i]);
            }
            display.setCursor(0, 56);
            display.print(F("L/R adj  click back"));
            display.display();
            return;
        }
    }

    // Top: battery + receive-link heartbeat.
    display.setCursor(0, 0);
    display.printf("Bat %3d%% %4.2fV", g_batteryPercent, g_batteryVoltage);
    display.setCursor(96, 0);
    bool linkOk = (g_lastNavMs != 0) && (millis() - g_lastNavMs < 2000);
    display.print(linkOk ? F(" LNK") : F("  --"));

    display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

    // Heading status.
    display.setCursor(0, 14);
    if (!isnan(g_navHeading)) display.printf("HDG %5.1f", g_navHeading);
    else                      display.print(F("HDG  ---"));

    display.setCursor(0, 26);
    if (g_lastNavSteerMs != 0 && (millis() - g_lastNavSteerMs) < 2000 && !isnan(g_navSteerCmd)) {
        display.printf("NAV CMD %+.3f", g_navSteerCmd);
    } else {
        display.print(F("NAV CMD ---"));
    }

    display.setCursor(0, 38);
    if (g_navHoldActive && !isnan(g_navHoldTarget)) {
        display.printf("HOLD %5.1f", g_navHoldTarget);
    } else {
        display.print(F("HOLD ---"));
    }

    // Joystick visualisation: keep icon box, hide XY text.
    if (joy.button) {
        display.setCursor(0, 52);
        display.print(F("[BTN]"));
    }

    constexpr int boxX = 104, boxY = 44, boxW = 20, boxH = 20;
    display.drawRect(boxX, boxY, boxW, boxH, SSD1306_WHITE);
    int dotX = boxX + boxW / 2 + (int)(joy.x * (boxW / 2 - 2));
    int dotY = boxY + boxH / 2 + (int)(joy.y * (boxH / 2 - 2));
    display.fillCircle(dotX, dotY, 2, SSD1306_WHITE);

    display.display();
}

static bool isMenuActive() {
    return g_menuScreen != MenuScreen::NONE;
}

static void moveMenuSelection(int delta) {
    if (delta == 0) return;
    if (g_menuScreen == MenuScreen::ROOT) {
        g_menuSelectionRoot += delta;
        if (g_menuSelectionRoot < 0) g_menuSelectionRoot = 4;
        if (g_menuSelectionRoot > 4) g_menuSelectionRoot = 0;
    } else if (g_menuScreen == MenuScreen::SETTINGS) {
        g_menuSelectionSettings += delta;
        if (g_menuSelectionSettings < 0) g_menuSelectionSettings = 5;
        if (g_menuSelectionSettings > 5) g_menuSelectionSettings = 0;
    } else if (g_menuScreen == MenuScreen::NAVIGATION) {
        g_menuSelectionNav += delta;
        if (g_menuSelectionNav < 0) g_menuSelectionNav = 4;
        if (g_menuSelectionNav > 4) g_menuSelectionNav = 0;
    } else if (g_menuScreen == MenuScreen::ROUTES) {
        g_menuSelectionRoutes += delta;
        if (g_menuSelectionRoutes < 0) g_menuSelectionRoutes = 8;
        if (g_menuSelectionRoutes > 8) g_menuSelectionRoutes = 0;
    } else if (g_menuScreen == MenuScreen::AP_TUNING) {
        g_menuSelectionApTuning += delta;
        if (g_menuSelectionApTuning < 0) g_menuSelectionApTuning = 5;
        if (g_menuSelectionApTuning > 5) g_menuSelectionApTuning = 0;
    }
}

static void adjustSteeringTrim(int delta) {
    if (delta == 0) return;

    int nextTrim = constrain(g_steeringTrimCounts + delta,
                             STEERING_TRIM_MIN,
                             STEERING_TRIM_MAX);
    if (nextTrim == g_steeringTrimCounts) return;

    g_steeringTrimCounts = nextTrim;
    g_settingsDirty = true;
}

static void onMenuClick() {
    if (g_menuScreen == MenuScreen::NONE) {
        g_menuScreen = MenuScreen::ROOT;
        return;
    }

    if (g_menuScreen == MenuScreen::ROOT) {
        if (g_menuSelectionRoot == 0) g_menuScreen = MenuScreen::NAVIGATION;
        else if (g_menuSelectionRoot == 1) g_menuScreen = MenuScreen::SETTINGS;
        else if (g_menuSelectionRoot == 2) g_menuScreen = MenuScreen::STEERING_TRIM;
        else if (g_menuSelectionRoot == 3) g_menuScreen = MenuScreen::BATTERY;
        else g_menuScreen = MenuScreen::NONE;
        return;
    }

    if (g_menuScreen == MenuScreen::SETTINGS) {
        if (g_menuSelectionSettings == 0) {
            g_idleSleepEnabled = !g_idleSleepEnabled;
            g_settingsDirty = true;
        } else if (g_menuSelectionSettings == 1) {
            g_wakeOnClick = !g_wakeOnClick;
            g_settingsDirty = true;
        } else if (g_menuSelectionSettings == 2) {
            g_wakeOnMovement = !g_wakeOnMovement;
            g_settingsDirty = true;
        } else if (g_menuSelectionSettings == 3) {
            calibrateAndSaveJoystick();
            Serial.printf("[Settings] joystick centre recalibrated: x=%d y=%d\n",
                          g_joyCenterX, g_joyCenterY);
        } else if (g_menuSelectionSettings == 4) {
            if (g_settingsDirty) saveSettings();
            enterDeepSleep();
        } else {
            if (g_settingsDirty) saveSettings();
            g_menuScreen = MenuScreen::ROOT;
        }
        return;
    }

    if (g_menuScreen == MenuScreen::NAVIGATION) {
        if (g_menuSelectionNav == 0) {
            if (g_navHoldActive) {
                // Disengage: send D-pad DOWN pulse.
                g_holdDisengagePulses = 2;
            } else {
                // Engage: send D-pad UP pulse.
                g_holdRequestPulses = 2;
            }
            g_menuScreen = MenuScreen::NONE;
        } else if (g_menuSelectionNav == 1) {
            g_menuScreen = MenuScreen::AP_TUNING;
        } else if (g_menuSelectionNav == 2) {
            g_menuScreen = MenuScreen::ROUTES;
        } else if (g_menuSelectionNav == 3) {
            g_menuScreen = MenuScreen::NAV_INFO;
        } else {
            g_menuScreen = MenuScreen::ROOT;
        }
        return;
    }

    if (g_menuScreen == MenuScreen::ROUTES) {
        if (g_menuSelectionRoutes == 0) {
            appendCurrentLocationToActiveRoute(millis());
        } else if (g_menuSelectionRoutes == 1) {
            selectNextWaypoint(1);
        } else if (g_menuSelectionRoutes == 2) {
            deleteSelectedWaypoint();
        } else if (g_menuSelectionRoutes == 3) {
            adjustSelectedWaypointRadius(1);
        } else if (g_menuSelectionRoutes == 4) {
            startRouteTransfer();
        } else if (g_menuSelectionRoutes == 5) {
            beginRenameRoute();
        } else if (g_menuSelectionRoutes == 6) {
            g_activeRouteId = (g_activeRouteId + 1) % ROUTE_MAX_ROUTES;
            clampSelectedWaypoint();
            saveRoutesToNVS();
            setRouteNotice("Route selected");
        } else if (g_menuSelectionRoutes == 7) {
            clearActiveRoute();
        } else {
            g_menuScreen = MenuScreen::NAVIGATION;
        }
        return;
    }

    if (g_menuScreen == MenuScreen::ROUTE_RENAME) {
        commitRenameRoute();
        g_menuScreen = MenuScreen::ROUTES;
        return;
    }

    if (g_menuScreen == MenuScreen::NAV_INFO) {
        g_menuScreen = MenuScreen::NAVIGATION;
        return;
    }

    if (g_menuScreen == MenuScreen::STEERING_TRIM) {
        if (g_settingsDirty) saveSettings();
        g_menuScreen = MenuScreen::ROOT;
        return;
    }

    if (g_menuScreen == MenuScreen::BATTERY) {
        g_menuScreen = MenuScreen::ROOT;
        return;
    }

    if (g_menuScreen == MenuScreen::AP_TUNING) {
        g_menuScreen = MenuScreen::NAVIGATION;
        return;
    }
}

static void handleMenuInput(const JoystickReading& joy, unsigned long now) {
    bool buttonPressed = joy.button;
    bool clicked = buttonPressed && !g_prevButtonPressed;

    if (clicked && (now - g_lastButtonEventMs >= MENU_BTN_DEBOUNCE_MS)) {
        onMenuClick();
        g_lastButtonEventMs = now;
    }

    if (isMenuActive() && (g_menuScreen == MenuScreen::ROOT ||
                           g_menuScreen == MenuScreen::SETTINGS ||
                           g_menuScreen == MenuScreen::NAVIGATION)) {
        int menuDelta = 0;
        if (joy.y > 0.6f) menuDelta = 1;
        else if (joy.y < -0.6f) menuDelta = -1;

        if (menuDelta != 0 && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            moveMenuSelection(menuDelta);
            g_lastMenuMoveMs = now;
        }
    } else if (g_menuScreen == MenuScreen::ROUTES) {
        int menuDelta = 0;
        if (joy.y > 0.6f) menuDelta = 1;
        else if (joy.y < -0.6f) menuDelta = -1;
        if (menuDelta != 0 && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            moveMenuSelection(menuDelta);
            g_lastMenuMoveMs = now;
        }

        int xDelta = 0;
        if (joy.x > 0.6f) xDelta = 1;
        else if (joy.x < -0.6f) xDelta = -1;
        if (xDelta != 0 && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            if (g_menuSelectionRoutes == 1) {
                selectNextWaypoint(xDelta);
            } else if (g_menuSelectionRoutes == 3) {
                adjustSelectedWaypointRadius(xDelta);
            }
            g_lastMenuMoveMs = now;
        }
    } else if (g_menuScreen == MenuScreen::ROUTE_RENAME) {
        int yDelta = 0;
        if (joy.y > 0.6f) yDelta = 1;
        else if (joy.y < -0.6f) yDelta = -1;
        if (yDelta != 0 && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            renameMoveCursor(yDelta);
            g_lastMenuMoveMs = now;
        }

        int xDelta = 0;
        if (joy.x > 0.6f) xDelta = 1;
        else if (joy.x < -0.6f) xDelta = -1;
        if (xDelta != 0 && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            renameAdjustChar(xDelta);
            g_lastMenuMoveMs = now;
        }
    } else if (g_menuScreen == MenuScreen::STEERING_TRIM) {
        int trimDelta = 0;
        if (joy.x > 0.6f) trimDelta = STEERING_TRIM_STEP;
        else if (joy.x < -0.6f) trimDelta = -STEERING_TRIM_STEP;

        if (trimDelta != 0 && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            adjustSteeringTrim(trimDelta);
            g_lastMenuMoveMs = now;
        }
    } else if (g_menuScreen == MenuScreen::AP_TUNING) {
        // Y scrolls between parameters.
        int menuDelta = 0;
        if (joy.y > 0.6f) menuDelta = 1;
        else if (joy.y < -0.6f) menuDelta = -1;
        if (menuDelta != 0 && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            moveMenuSelection(menuDelta);
            g_lastMenuMoveMs = now;
        }
        // X adjusts the selected parameter's value.
        float jx = joy.x;
        if (fabsf(jx) > 0.6f && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            float sign = (jx > 0) ? 1.0f : -1.0f;
            static const float steps[] = { 0.001f, 0.001f, 0.01f, 0.05f, 0.5f, 0.005f };
            float step = sign * steps[g_menuSelectionApTuning];
            switch (g_menuSelectionApTuning) {
                case AP_PARAM_KP:         g_apKp        = constrain(g_apKp        + step, 0.0f,   1.0f);  sendApTuning(AP_PARAM_KP,         g_apKp);        break;
                case AP_PARAM_KI:         g_apKi        = constrain(g_apKi        + step, 0.0f,   1.0f);  sendApTuning(AP_PARAM_KI,         g_apKi);        break;
                case AP_PARAM_KD:         g_apKd        = constrain(g_apKd        + step, 0.0f,   5.0f);  sendApTuning(AP_PARAM_KD,         g_apKd);        break;
                case AP_PARAM_MAX_OUTPUT: g_apMaxOutput = constrain(g_apMaxOutput + step, 0.05f,  1.0f);  sendApTuning(AP_PARAM_MAX_OUTPUT, g_apMaxOutput); break;
                case AP_PARAM_DEADBAND:   g_apDeadband  = constrain(g_apDeadband  + step, 0.0f,  30.0f);  sendApTuning(AP_PARAM_DEADBAND,   g_apDeadband);  break;
                case AP_PARAM_RATE_LIMIT: g_apRateLimit = constrain(g_apRateLimit + step, 0.001f, 1.0f);  sendApTuning(AP_PARAM_RATE_LIMIT, g_apRateLimit); break;
            }
            g_lastMenuMoveMs = now;
        }
    }

    g_prevButtonPressed = buttonPressed;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
    for (int i = 0; i < 30 && !Serial; i++) delay(100);
#else
    delay(500);
#endif
    Serial.println("\n=== Sauna Boat Handheld Controller ===");

    analogReadResolution(12);

    // Release any LED hold from before deep sleep.
    gpio_hold_dis((gpio_num_t)BLUE_LED_PIN);
    pinMode(BLUE_LED_PIN, OUTPUT);
    digitalWrite(BLUE_LED_PIN, HIGH);

    pinMode(JOY_BTN_PIN, INPUT_PULLUP);

    loadSettings();
    loadRoutesFromNVS();
    Serial.printf("[Setup] idle sleep timeout = %s\n",
                  g_idleSleepEnabled ? "ENABLED" : "DISABLED");
    Serial.printf("[Setup] wake-on-click = %s\n",
                  g_wakeOnClick ? "ENABLED" : "DISABLED");
    Serial.printf("[Setup] wake-on-movement = %s\n",
                  g_wakeOnMovement ? "ENABLED" : "DISABLED");
    Serial.printf("[Setup] steering trim = %+d counts (center=%d)\n",
                  g_steeringTrimCounts, steeringCenterCounts());
    if (DISABLE_SLEEP_FOR_DEBUG) {
        Serial.println("[Setup] DEBUG: idle deep sleep is DISABLED");
    }

    esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
    switch (wake) {
        case ESP_SLEEP_WAKEUP_EXT0: Serial.println("[Setup] Wake reason: BUTTON"); break;
        case ESP_SLEEP_WAKEUP_EXT1: Serial.println("[Setup] Wake reason: MOTION"); break;
        default:                    Serial.println("[Setup] Wake reason: POWER-ON"); break;
    }

    // I2C bus shared by MPU-6050 and the OLED.
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    if (wake == ESP_SLEEP_WAKEUP_EXT1) wakeupMPU();
    mpu.initialize();
    g_mpuOk = mpu.testConnection();
    Serial.printf("[Setup] MPU-6050: %s\n", g_mpuOk ? "OK" : "NOT FOUND");
    if (g_mpuOk) {
        mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    }

    g_oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    Serial.printf("[Setup] SSD1306: %s\n", g_oledOk ? "OK" : "NOT FOUND");
    if (g_oledOk) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("Sauna Boat"));
        display.println(F("Handheld Controller"));
        display.println();
        display.println(F("loading settings..."));
        display.display();
    }
    Serial.printf("[Setup] joystick centre (NVS): x=%d y=%d\n",
                  g_joyCenterX, g_joyCenterY);

    // ---- WiFi + ESP-NOW ----
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(MESH_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[Setup] WiFi MAC: %s  (channel %d)\n",
                  WiFi.macAddress().c_str(), MESH_WIFI_CHANNEL);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[Setup] ERROR: ESP-NOW init failed!");
    } else {
        Serial.println("[Setup] ESP-NOW initialised.");
    }
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, MESH_BROADCAST_ADDR, 6);
    peer.channel = MESH_WIFI_CHANNEL;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[Setup] WARNING: failed to add broadcast peer.");
    }

    readBatteryVoltage();
    g_lastActivityMs = millis();
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    static unsigned long lastSend     = 0;
    static unsigned long lastDisplay  = 0;
    static unsigned long lastBattery  = 0;
    static unsigned long lastLog      = 0;

    unsigned long now = millis();

    JoystickReading joy = readJoystick();
    handleMenuInput(joy, now);
    processRouteTransfer(now);

    // Activity detection: any deflection or button press resets the idle timer.
    if (fabsf(joy.x) > 0.05f || fabsf(joy.y) > 0.05f || joy.button) {
        g_lastActivityMs = now;
    }

    // Heartbeat LED (slow blink).
    digitalWrite(BLUE_LED_PIN, (now / 500) % 2 == 0 ? HIGH : LOW);

    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;
        uint16_t extraButtons = 0;
        if (g_holdRequestPulses > 0) {
            extraButtons |= CTRL_BTN_UP;
            --g_holdRequestPulses;
        }
        if (g_holdDisengagePulses > 0) {
            extraButtons |= CTRL_BTN_DOWN;
            --g_holdDisengagePulses;
        }

        if (isMenuActive()) {
            JoystickReading neutralJoy = joy;
            neutralJoy.x = 0.0f;
            neutralJoy.y = 0.0f;
            neutralJoy.button = false;
            sendControllerInput(neutralJoy, extraButtons);
            // Keep steering responsive while a menu is open.
            sendSteering(joy.x);
        } else {
            sendControllerInput(joy, extraButtons);
            sendSteering(joy.x);
        }
    }

    if (now - lastDisplay >= DISPLAY_INTERVAL_MS) {
        lastDisplay = now;
        drawDisplay(joy);
    }

    if (now - lastBattery >= BATTERY_INTERVAL_MS) {
        lastBattery = now;
        readBatteryVoltage();
        sendBatteryStatus();
    }

    if (now - lastLog >= LOG_INTERVAL_MS) {
        lastLog = now;
        Serial.printf("[Loop] joy x=%+.2f y=%+.2f btn=%d | menu=%u hold=%d trim=%+d | "
                  "bat=%.2fV %d%% | mesh ok=%lu fail=%lu\n",
                      joy.x, joy.y, joy.button ? 1 : 0,
                      (unsigned int)g_menuScreen,
                      g_navHoldActive ? 1 : 0,
                  g_steeringTrimCounts,
                      g_batteryVoltage, g_batteryPercent,
                      (unsigned long)g_sendOkCount,
                      (unsigned long)g_sendFailCount);
    }

    if (!DISABLE_SLEEP_FOR_DEBUG && g_idleSleepEnabled &&
        now - g_lastActivityMs >= IDLE_SLEEP_MS) {
        enterDeepSleep();
    }

    delay(5);
}
