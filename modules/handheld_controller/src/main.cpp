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
constexpr float BATTERY_VOLTAGE_FULL  = 4.20f;
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
    NAV_INFO,
    STEERING_TRIM,
    BATTERY
};

static MenuScreen g_menuScreen = MenuScreen::NONE;
static int g_menuSelectionRoot = 0;
static int g_menuSelectionSettings = 0;
static int g_menuSelectionNav  = 0;
static bool g_headingHoldEnabled = false;
static bool g_settingsDirty = false;

static bool g_prevButtonPressed = false;
static unsigned long g_lastMenuMoveMs = 0;
static unsigned long g_lastButtonEventMs = 0;

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
}

static void sendControllerInput(const JoystickReading& joy) {
    ControllerInputMessage msg = {};
    msg.type    = MSG_CONTROLLER_INPUT;
    msg.src     = MODULE_HANDHELD;
    msg.lx      = joy.x;
    msg.ly      = joy.y;
    msg.rx      = 0.0f;
    msg.ry      = 0.0f;
    msg.buttons = joy.button ? CTRL_BTN_L3 : 0;  // joystick click → L3
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
            drawMenuLine(24, g_menuSelectionRoot == 1, "Settings");
            drawMenuLine(34, g_menuSelectionRoot == 2, "Steering Trim");
            drawMenuLine(44, g_menuSelectionRoot == 3, "Battery");
            drawMenuLine(54, g_menuSelectionRoot == 4, "Close");
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
            drawMenuLine(16, g_menuSelectionNav == 0, "Info");
            display.setCursor(0, 28);
            display.print(g_menuSelectionNav == 1 ? F("> ") : F("  "));
            display.print(F("Heading Hold: "));
            display.print(g_headingHoldEnabled ? F("ON") : F("OFF"));
            drawMenuLine(40, g_menuSelectionNav == 2, "Back");
            display.setCursor(0, 54);
            display.println(F("Click to select"));
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
    }

    // Top: battery + receive-link heartbeat.
    display.setCursor(0, 0);
    display.printf("Bat %3d%% %4.2fV", g_batteryPercent, g_batteryVoltage);
    display.setCursor(96, 0);
    bool linkOk = (g_lastNavMs != 0) && (millis() - g_lastNavMs < 2000);
    display.print(linkOk ? F(" LNK") : F("  --"));

    display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

    // Wi-Fi / ESP-NOW diagnostics (replaces heading/satellite fields).
    display.setCursor(0, 14);
    display.printf("WiFi ch%u", (unsigned int)MESH_WIFI_CHANNEL);
    display.setCursor(72, 14);
    display.printf("Fix %u", g_navFixType);

    display.setCursor(0, 26);
    if (g_lastSendMs != 0) {
        unsigned long txAgeMs = millis() - g_lastSendMs;
        display.printf("TX %s %4lums", g_lastSendOk ? "OK" : "FL", txAgeMs);
    } else {
        display.print(F("TX --"));
    }
    display.setCursor(72, 26);
    display.printf("%lu/%lu",
                   (unsigned long)g_sendOkCount,
                   (unsigned long)g_sendFailCount);

    // Steering target / actual from boat feedback.
    display.setCursor(0, 38);
    if (!isnan(g_steeringAngle)) {
        display.printf("RUD %5.1f -> %5.1f", g_steeringAngle, g_steeringTarget);
    } else {
        display.print(F("RUD  ---"));
    }

    // Joystick visualisation: small box bottom-right, value bottom-left.
    display.setCursor(0, 52);
    display.printf("X%+.2f Y%+.2f", joy.x, joy.y);
    if (joy.button) {
        display.setCursor(78, 52);
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
        if (g_menuSelectionNav < 0) g_menuSelectionNav = 2;
        if (g_menuSelectionNav > 2) g_menuSelectionNav = 0;
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
            g_menuScreen = MenuScreen::NAV_INFO;
        } else if (g_menuSelectionNav == 1) {
            g_headingHoldEnabled = !g_headingHoldEnabled;
        } else {
            g_menuScreen = MenuScreen::ROOT;
        }
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
        if (joy.y > 0.6f) menuDelta = -1;
        else if (joy.y < -0.6f) menuDelta = 1;

        if (menuDelta != 0 && (now - g_lastMenuMoveMs >= MENU_NAV_REPEAT_MS)) {
            moveMenuSelection(menuDelta);
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

    // Activity detection: any deflection or button press resets the idle timer.
    if (fabsf(joy.x) > 0.05f || fabsf(joy.y) > 0.05f || joy.button) {
        g_lastActivityMs = now;
    }

    // Heartbeat LED (slow blink).
    digitalWrite(BLUE_LED_PIN, (now / 500) % 2 == 0 ? HIGH : LOW);

    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;
        if (isMenuActive()) {
            JoystickReading neutralJoy = joy;
            neutralJoy.x = 0.0f;
            neutralJoy.y = 0.0f;
            neutralJoy.button = false;
            sendControllerInput(neutralJoy);
            sendSteering(0.0f);
        } else {
            sendControllerInput(joy);
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
                      g_headingHoldEnabled ? 1 : 0,
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
