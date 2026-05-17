// mesh_protocol.h – Sauna Boat inter-module communication protocol
//
// All modules communicate via ESP-NOW using a simple broadcast model that
// mirrors a CAN bus: every module can send any message, and every module
// receives every message, filtering by type as needed.
//
// To add a new module:
//   1. Assign it a MODULE_* ID below.
//   2. Define any new MSG_* types it produces.
//   3. Include this header in both the sender and receiver firmware.
//
// All modules must use MESH_WIFI_CHANNEL for their ESP-NOW radio channel.

#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// WiFi channel used by all modules for ESP-NOW (must match across all nodes).
// The steering module's soft-AP is pinned to this channel.
// ---------------------------------------------------------------------------
#define MESH_WIFI_CHANNEL 1

// ---------------------------------------------------------------------------
// Module IDs (source field in every message)
// ---------------------------------------------------------------------------
#define MODULE_STEERING    0x01
#define MODULE_CONTROLLER  0x02
#define MODULE_NAVIGATION  0x03
#define MODULE_HANDHELD    0x04   // custom ESP32-S3 handheld with joystick + OLED
// Add future modules here: #define MODULE_ENGINE 0x05, etc.

// ---------------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------------
// MSG_SET_STEERING – sent by the controller (or navigation module) to the
//   steering module.
//   value1 = normalized rudder position [-1.0, 1.0].
//             Negative = port/left, positive = starboard/right.
//   value2 = unused (0).
#define MSG_SET_STEERING      0x01

// MSG_SET_ANGLE  – sent by the controller to a PID angle-based steering module.
//   value1 = desired heading in degrees [0, 360).
//   value2 = unused (0).
#define MSG_SET_ANGLE      0x02

// MSG_ANGLE_STATUS – broadcast by an angle-based steering module at ~10 Hz.
//   value1 = current measured angle (degrees).
//   value2 = current target angle   (degrees).
#define MSG_ANGLE_STATUS   0x03

// MSG_CONTROLLER_STATUS – broadcast by the controller module at ~0.2 Hz.
//   value1 = PS3 battery level:  -1=unknown, 0=shutdown, 1=dying, 2=low,
//                                 3=high, 4=full.
//   value2 = charging flag:       1.0 = plugged in and charging, 0.0 = on battery.
#define MSG_CONTROLLER_STATUS 0x04

// MSG_CONTROLLER_INPUT – raw PS3 controller state, broadcast by the controller
//   at ~20 Hz.  The receiving module decides how to interpret the inputs.
//   All analog values are normalised to [-1.0, 1.0] with NO dead-zone applied.
//   Sent even when the PS3 controller is disconnected (all fields zero).
//   See ControllerInputMessage struct and CTRL_BTN_* bitmask flags below.
#define MSG_CONTROLLER_INPUT  0x05

// MSG_NAV_STATUS – broadcast by the navigation module at ~2 Hz.
//   Contains GPS position, speed, course, and magnetic heading.
//   See NavigationMessage struct below.
#define MSG_NAV_STATUS        0x06

// MSG_HEADING_HOLD_STATUS – broadcast by the navigation module at ~2 Hz.
//   value1 = 1.0 if heading hold is active, 0.0 if manual.
//   value2 = target heading in degrees [0, 360) when active, 0 otherwise.
#define MSG_HEADING_HOLD_STATUS 0x07

// ---------------------------------------------------------------------------
// Button bitmask flags used in ControllerInputMessage.buttons
// ---------------------------------------------------------------------------
#define CTRL_BTN_CROSS    (1u << 0)
#define CTRL_BTN_CIRCLE   (1u << 1)
#define CTRL_BTN_SQUARE   (1u << 2)
#define CTRL_BTN_TRIANGLE (1u << 3)
#define CTRL_BTN_L1       (1u << 4)
#define CTRL_BTN_R1       (1u << 5)
#define CTRL_BTN_L2       (1u << 6)
#define CTRL_BTN_R2       (1u << 7)
#define CTRL_BTN_L3       (1u << 8)
#define CTRL_BTN_R3       (1u << 9)
#define CTRL_BTN_UP       (1u << 10)
#define CTRL_BTN_DOWN     (1u << 11)
#define CTRL_BTN_LEFT     (1u << 12)
#define CTRL_BTN_RIGHT    (1u << 13)
#define CTRL_BTN_START    (1u << 14)
#define CTRL_BTN_SELECT   (1u << 15)

// ---------------------------------------------------------------------------
// Message structs (max ESP-NOW payload is 250 bytes)
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct MeshMessage {
    uint8_t type;    // MSG_* constant
    uint8_t src;     // MODULE_* constant identifying the sender
    float   value1;  // primary payload value
    float   value2;  // secondary payload value (0 if unused)
};  // 10 bytes

// Raw PS3 controller state.  All analog axes are normalised; dead-zone is
// intentionally NOT applied here so each receiving module can choose its own.
struct ControllerInputMessage {
    uint8_t  type;     // MSG_CONTROLLER_INPUT
    uint8_t  src;      // MODULE_CONTROLLER
    float    lx;       // left  stick X, [-1.0, 1.0]  (negative = left)
    float    ly;       // left  stick Y, [-1.0, 1.0]  (negative = up)
    float    rx;       // right stick X, [-1.0, 1.0]
    float    ry;       // right stick Y, [-1.0, 1.0]
    uint16_t buttons;  // CTRL_BTN_* bitmask; 0 when controller is disconnected
};  // 20 bytes

// Navigation module broadcast – GPS + compass data.
struct NavigationMessage {
    uint8_t  type;      // MSG_NAV_STATUS
    uint8_t  src;       // MODULE_NAVIGATION
    uint8_t  fixType;   // 0 = no fix, 1 = GPS fix, 2 = DGPS
    uint8_t  satellites; // number of satellites in use
    float    lat;       // latitude  in degrees (negative = south)
    float    lon;       // longitude in degrees (negative = west)
    float    speedKnots; // speed over ground in knots
    float    course;    // GPS course over ground in degrees [0, 360)
    float    heading;   // magnetic compass heading in degrees [0, 360)
};  // 24 bytes
#pragma pack(pop)

// Broadcast MAC address – send here to reach every ESP-NOW peer at once.
static const uint8_t MESH_BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
