// main.cpp – Sauna Boat Handheld Controller Module
//
// ESP32-S3 (Pavloff board) — custom handheld controller with an analog
// joystick, an SSD1306 0.96" OLED, an MPU-6050 (used for wake-on-motion),
// and an on-board LiPo + battery monitor.
//
// Behaviour mirrors the PS3 `controller` module from the boat's perspective:
// it broadcasts MSG_CONTROLLER_INPUT and MSG_SET_STEERING over ESP-NOW on
// MESH_WIFI_CHANNEL, so steering / navigation receivers need no changes.
//
// Display shows boat telemetry (heading, GPS fix, target heading) received
// from the navigation and steering modules, plus local battery %.
//
// Power management: after IDLE_SLEEP_MS of no joystick deflection or button
// press the board enters deep sleep. It wakes on either MPU-6050 motion
// (EXT1 / GPIO 18) or the joystick button (EXT0 / GPIO 6 pulled LOW).

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
#define SDA_PIN        8   // I2C SDA — MPU-6050 + OLED share this bus
#define SCL_PIN        9   // I2C SCL
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
// 12-bit ADC; centre is auto-captured at boot in calibrateJoystick().
constexpr int   JOY_ADC_MAX        = 4095;
constexpr int   JOY_DEAD_COUNTS    = 120;   // raw counts around centre treated as zero
constexpr float JOY_OUTPUT_GAMMA   = 1.0f;  // >1 = softer near centre; 1 = linear

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
constexpr unsigned long SEND_INTERVAL_MS    = 50;    // 20 Hz mesh broadcast
constexpr unsigned long DISPLAY_INTERVAL_MS = 100;   // 10 Hz OLED refresh
constexpr unsigned long BATTERY_INTERVAL_MS = 5000;
constexpr unsigned long LOG_INTERVAL_MS     = 1000;

// ---------------------------------------------------------------------------
// Sleep timeout — enter deep sleep after this much idle (no stick / button).
// ---------------------------------------------------------------------------
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

// Whether to arm the MPU-6050 motion interrupt before sleeping.
// Toggleable via the "settings" preferences namespace, key `wakeOnMove`.
static bool g_wakeOnMovement = true;

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
    // Average several samples at boot; assumes the user isn't touching it.
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
    r.x      = joyNormalize(r.rawX, g_joyCenterY);
    r.y      = -joyNormalize(r.rawY, g_joyCenterX);
    r.button = digitalRead(JOY_BTN_PIN) == LOW;
    return r;
}

// ---------------------------------------------------------------------------
// ESP-NOW
// ---------------------------------------------------------------------------
static void onDataSent(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) ++g_sendOkCount;
    else                                ++g_sendFailCount;
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

static void sendSteering(float stickX) {
    MeshMessage msg;
    msg.type   = MSG_SET_STEERING;
    msg.src    = MODULE_HANDHELD;
    msg.value1 = constrain(stickX, -1.0f, 1.0f);
    msg.value2 = 0.0f;
    esp_now_send(MESH_BROADCAST_ADDR,
                 reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
}

static void sendBatteryStatus() {
    MeshMessage msg;
    msg.type   = MSG_CONTROLLER_STATUS;
    msg.src    = MODULE_HANDHELD;
    // Re-use the PS3 battery scale: -1=unknown..4=full. Linear from %.
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

    // EXT0: joystick button (active LOW) — single-pin, low-pin-count wake.
    esp_sleep_enable_ext0_wakeup((gpio_num_t)JOY_BTN_PIN, 0);
    gpio_pullup_en((gpio_num_t)JOY_BTN_PIN);
    gpio_pulldown_dis((gpio_num_t)JOY_BTN_PIN);

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
    } else if (g_mpuOk) {
        mpu.setSleepEnabled(true);
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

    // Top: battery + send status.
    display.setCursor(0, 0);
    display.printf("Bat %3d%% %4.2fV", g_batteryPercent, g_batteryVoltage);
    display.setCursor(96, 0);
    bool linkOk = (g_lastNavMs != 0) && (millis() - g_lastNavMs < 2000);
    display.print(linkOk ? F(" LNK") : F("  --"));

    display.drawFastHLine(0, 10, OLED_WIDTH, SSD1306_WHITE);

    // Heading + GPS fix
    display.setCursor(0, 14);
    if (!isnan(g_navHeading)) {
        display.printf("HDG %5.1f", g_navHeading);
    } else {
        display.print(F("HDG  ---"));
    }
    display.setCursor(72, 14);
    display.printf("Sat %2u", g_navSatellites);

    display.setCursor(0, 26);
    if (!isnan(g_navSpeedKnots)) {
        display.printf("SOG %4.1fkn", g_navSpeedKnots);
    } else {
        display.print(F("SOG --"));
    }
    display.setCursor(72, 26);
    display.print(g_navFixType > 0 ? F("FIX") : F("---"));

    // Steering target / actual
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

    preferences.begin("settings", true);
    g_wakeOnMovement = preferences.getBool("wakeOnMove", true);
    preferences.end();
    Serial.printf("[Setup] wake-on-movement = %s\n",
                  g_wakeOnMovement ? "ENABLED" : "DISABLED");

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
        display.println(F("calibrating stick..."));
        display.display();
    }

    calibrateJoystick();
    Serial.printf("[Setup] joystick centre: x=%d y=%d\n",
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

    // Activity detection: any deflection or button press resets the idle timer.
    if (fabsf(joy.x) > 0.05f || fabsf(joy.y) > 0.05f || joy.button) {
        g_lastActivityMs = now;
    }

    // Heartbeat LED (slow blink).
    digitalWrite(BLUE_LED_PIN, (now / 500) % 2 == 0 ? HIGH : LOW);

    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;
        sendControllerInput(joy);
        sendSteering(joy.x);
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
        Serial.printf("[Loop] joy x=%+.2f y=%+.2f btn=%d | "
                      "bat=%.2fV %d%% | mesh ok=%lu fail=%lu\n",
                      joy.x, joy.y, joy.button ? 1 : 0,
                      g_batteryVoltage, g_batteryPercent,
                      (unsigned long)g_sendOkCount,
                      (unsigned long)g_sendFailCount);
    }

    if (now - g_lastActivityMs >= IDLE_SLEEP_MS) {
        enterDeepSleep();
    }

    delay(5);
}
