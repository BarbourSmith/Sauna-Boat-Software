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
// ---------------------------------------------------------------------------
#define MESH_WIFI_CHANNEL 1

// ---------------------------------------------------------------------------
// Module IDs (source field in every message)
// ---------------------------------------------------------------------------
#define MODULE_STEERING    0x01
// 0x02 reserved for legacy sender IDs
#define MODULE_NAVIGATION  0x03
#define MODULE_HANDHELD    0x04   // custom ESP32-S3 handheld with joystick + OLED
// Add future modules here: #define MODULE_ENGINE 0x05, etc.

// ---------------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------------
// MSG_SET_STEERING – sent by manual input modules and/or the navigation module to the
//   steering module.
//   value1 = normalized rudder position [-1.0, 1.0].
//             Negative = port/left, positive = starboard/right.
//   value2 = unused (0).
#define MSG_SET_STEERING      0x01

// MSG_SET_ANGLE  – sent by an input module to a PID angle-based steering module.
//   value1 = desired heading in degrees [0, 360).
//   value2 = unused (0).
#define MSG_SET_ANGLE      0x02

// MSG_ANGLE_STATUS – broadcast by an angle-based steering module at ~10 Hz.
//   value1 = current measured angle (degrees).
//   value2 = current target angle   (degrees).
#define MSG_ANGLE_STATUS   0x03

// MSG_CONTROLLER_STATUS – broadcast by an input module at ~0.2 Hz.
//   value1 = input-device battery level: -1=unknown, 0=shutdown, 1=dying, 2=low,
//                                 3=high, 4=full.
//   value2 = charging flag:       1.0 = plugged in and charging, 0.0 = on battery.
#define MSG_CONTROLLER_STATUS 0x04

// MSG_CONTROLLER_INPUT – raw input-device state, broadcast by a controller source
//   at ~20 Hz.  The receiving module decides how to interpret the inputs.
//   All analog values are normalised to [-1.0, 1.0] with NO dead-zone applied.
//   Sent even when the input device is disconnected or idle (all fields zero).
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

// MSG_AP_TUNING – sent by the handheld to update one autopilot PID parameter.
//   value1 = parameter ID (see AP_PARAM_* constants below).
//   value2 = new value for that parameter.
#define MSG_AP_TUNING 0x08

// MSG_AP_TUNING_STATUS – broadcast by the navigation module.
//   value1 = parameter ID (see AP_PARAM_* constants below).
//   value2 = current value for that parameter.
#define MSG_AP_TUNING_STATUS 0x09

// MSG_ROUTE_START – sent by handheld before chunked route upload.
//   routeId identifies the handheld-stored route.
//   totalWaypoints and totalChunks define the expected payload.
#define MSG_ROUTE_START 0x0A

// MSG_ROUTE_CHUNK – sent by handheld with 1..ROUTE_WAYPOINTS_PER_CHUNK waypoints.
//   chunkIndex is zero-based and must be contiguous.
#define MSG_ROUTE_CHUNK 0x0B

// MSG_ROUTE_END – sent by handheld after all route chunks.
//   crc16 is optional (set 0 if unused).
#define MSG_ROUTE_END 0x0C

// MSG_ROUTE_CONTROL – sent by handheld to start/stop execution.
//   action uses ROUTE_ACTION_* constants below.
//   Navigation follows the line through the waypoints rather than steering
//   for each one, joining it wherever the boat is closest; waypoints before
//   that point are skipped.
#define MSG_ROUTE_CONTROL 0x0D

// MSG_ROUTE_STATUS – broadcast by navigation for route upload/run state.
//   state uses ROUTE_STATE_* constants below.
#define MSG_ROUTE_STATUS 0x0E

// MSG_WAYPOINT_REACHED – broadcast by navigation once the boat is past a
//   waypoint along the route, however close it came to it.
#define MSG_WAYPOINT_REACHED 0x0F

// MSG_ROUTE_COMPLETE – broadcast by navigation when the boat is inside the
//   final waypoint's radius, or past the end of the route.
#define MSG_ROUTE_COMPLETE 0x10

// MSG_ROUTE_ABORT – broadcast by navigation when route execution aborts.
//   reason uses ROUTE_ABORT_* constants below.
#define MSG_ROUTE_ABORT 0x11

// ---------------------------------------------------------------------------
// Remote access through the handheld (see shared/mesh_remote.h)
// ---------------------------------------------------------------------------
// The steering and navigation boards' USB ports are hard to reach on the
// boat, so the handheld relays between its USB port and the mesh
// (tools/mesh_gateway): it forwards their logs to a computer, and serial
// commands and firmware updates to them.

// MSG_REMOTE_PING – broadcast by the handheld about once a second while a
//   computer is listening through its USB port. Modules broadcast their log
//   (MSG_REMOTE_LOG) while pings keep arriving and answer each ping with
//   MSG_REMOTE_INFO.
#define MSG_REMOTE_PING 0x12

// MSG_REMOTE_INFO – broadcast by a module at boot and in answer to a ping:
//   firmware build time, reset reason, uptime and update state.
#define MSG_REMOTE_INFO 0x13

// MSG_REMOTE_LOG – one line of a module's serial log.
#define MSG_REMOTE_LOG 0x14

// MSG_REMOTE_COMMAND – sent by the handheld: text handled by the target
//   module as if it had been typed into its serial port.
#define MSG_REMOTE_COMMAND 0x15

// MSG_OTA_BEGIN / DATA / END / ABORT – sent by the handheld to write new
//   firmware to one module (the target). The target answers each with
//   MSG_OTA_ACK; the handheld resends anything that isn't acknowledged.
//   The image is checked against its MD5 before the module restarts into it.
#define MSG_OTA_BEGIN 0x16
#define MSG_OTA_DATA  0x17
#define MSG_OTA_END   0x18
#define MSG_OTA_ABORT 0x19
#define MSG_OTA_ACK   0x1A

// Parameter IDs for MSG_AP_TUNING
#define AP_PARAM_KP         0
#define AP_PARAM_KI         1
#define AP_PARAM_KD         2
#define AP_PARAM_MAX_OUTPUT 3
#define AP_PARAM_DEADBAND   4
#define AP_PARAM_RATE_LIMIT 5

// Route limits for first implementation.
#define ROUTE_MAX_ROUTES               10
#define ROUTE_MAX_WAYPOINTS_PER_ROUTE  50
#define ROUTE_WAYPOINTS_PER_CHUNK      4
#define ROUTE_DEFAULT_RADIUS_M         5

// ROUTE_ACTION_* for MSG_ROUTE_CONTROL.
#define ROUTE_ACTION_START 1
#define ROUTE_ACTION_STOP  2

// ROUTE_STATE_* for MSG_ROUTE_STATUS.
#define ROUTE_STATE_IDLE       0
#define ROUTE_STATE_UPLOADING  1
#define ROUTE_STATE_READY      2
#define ROUTE_STATE_RUNNING    3
#define ROUTE_STATE_ABORTED    4
#define ROUTE_STATE_COMPLETE   5

// ROUTE_ABORT_* for MSG_ROUTE_ABORT.
#define ROUTE_ABORT_NONE            0
#define ROUTE_ABORT_BAD_FORMAT      1
#define ROUTE_ABORT_BAD_SEQUENCE    2
#define ROUTE_ABORT_BAD_WAYPOINT    3
#define ROUTE_ABORT_NO_GPS_FIX      4
#define ROUTE_ABORT_NO_HEADING      5
#define ROUTE_ABORT_STOP_REQUESTED  6
#define ROUTE_ABORT_EMPTY_ROUTE     7

// REMOTE_FW_* for MSG_REMOTE_INFO.otaState.
#define REMOTE_FW_CONFIRMED   0  // running firmware is kept
#define REMOTE_FW_UNCONFIRMED 1  // just updated: kept once it hears the mesh, else rolled back
#define REMOTE_FW_UPDATING    2  // receiving new firmware

// OTA_STATUS_* for MSG_OTA_ACK.
#define OTA_STATUS_OK         0
#define OTA_STATUS_BUSY       1  // module is steering the boat (heading hold or a route)
#define OTA_STATUS_TOO_BIG    2  // image doesn't fit the module's update partition
#define OTA_STATUS_FLASH      3  // couldn't write the update partition
#define OTA_STATUS_VERIFY     4  // image incomplete, MD5 mismatch, or not an app for this chip
#define OTA_STATUS_NO_SESSION 5  // no update in progress with this session

#define REMOTE_LOG_TEXT_MAX     240
#define REMOTE_COMMAND_TEXT_MAX 64
#define OTA_CHUNK_MAX           240

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

// Raw controller/input state.  All analog axes are normalised; dead-zone is
// intentionally NOT applied here so each receiving module can choose its own.
struct ControllerInputMessage {
    uint8_t  type;     // MSG_CONTROLLER_INPUT
    uint8_t  src;      // sender module ID (MODULE_HANDHELD or other input module)
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

// Fixed-point waypoint for deterministic transfer and storage.
// latE7/lonE7 are degrees scaled by 1e7 (WGS84), radius in meters. Only the
// final waypoint's radius is used: the route finishes inside it.
struct WaypointData {
    int32_t  latE7;
    int32_t  lonE7;
    uint16_t radiusM;
};  // 10 bytes

struct RouteStartMessage {
    uint8_t  type;           // MSG_ROUTE_START
    uint8_t  src;            // MODULE_HANDHELD
    uint8_t  routeId;        // [0, ROUTE_MAX_ROUTES)
    uint8_t  totalWaypoints; // [1, ROUTE_MAX_WAYPOINTS_PER_ROUTE]
    uint8_t  totalChunks;    // ceil(totalWaypoints / ROUTE_WAYPOINTS_PER_CHUNK)
    uint8_t  reserved;
};  // 6 bytes

struct RouteChunkMessage {
    uint8_t      type;          // MSG_ROUTE_CHUNK
    uint8_t      src;           // MODULE_HANDHELD
    uint8_t      routeId;
    uint8_t      chunkIndex;    // zero-based
    uint8_t      waypointCount; // 1..ROUTE_WAYPOINTS_PER_CHUNK
    uint8_t      reserved;
    WaypointData waypoints[ROUTE_WAYPOINTS_PER_CHUNK];
};  // 46 bytes

struct RouteEndMessage {
    uint8_t  type;      // MSG_ROUTE_END
    uint8_t  src;       // MODULE_HANDHELD
    uint8_t  routeId;
    uint8_t  reserved;
    uint16_t crc16;     // optional; can be 0 in v1
};  // 6 bytes

struct RouteControlMessage {
    uint8_t  type;      // MSG_ROUTE_CONTROL
    uint8_t  src;       // MODULE_HANDHELD
    uint8_t  action;    // ROUTE_ACTION_*
    uint8_t  routeId;
};  // 4 bytes

struct RouteStatusMessage {
    uint8_t  type;          // MSG_ROUTE_STATUS
    uint8_t  src;           // MODULE_NAVIGATION
    uint8_t  routeId;
    uint8_t  state;         // ROUTE_STATE_*
    uint8_t  currentIndex;  // [0, totalWaypoints): next waypoint ahead along the route
    uint8_t  totalWaypoints;
    uint16_t distanceM;     // straight-line distance to that waypoint when running
};  // 8 bytes

struct WaypointReachedMessage {
    uint8_t  type;          // MSG_WAYPOINT_REACHED
    uint8_t  src;           // MODULE_NAVIGATION
    uint8_t  routeId;
    uint8_t  waypointIndex; // reached waypoint index
};  // 4 bytes

struct RouteCompleteMessage {
    uint8_t  type;      // MSG_ROUTE_COMPLETE
    uint8_t  src;       // MODULE_NAVIGATION
    uint8_t  routeId;
    uint8_t  reserved;
};  // 4 bytes

struct RouteAbortMessage {
    uint8_t  type;      // MSG_ROUTE_ABORT
    uint8_t  src;       // MODULE_NAVIGATION
    uint8_t  routeId;
    uint8_t  reason;    // ROUTE_ABORT_*
};  // 4 bytes

struct RemotePingMessage {
    uint8_t  type;      // MSG_REMOTE_PING
    uint8_t  src;       // MODULE_HANDHELD
};  // 2 bytes

struct RemoteInfoMessage {
    uint8_t  type;        // MSG_REMOTE_INFO
    uint8_t  src;         // module reporting
    uint8_t  resetReason; // esp_reset_reason_t of the last restart
    uint8_t  otaState;    // REMOTE_FW_*
    uint32_t uptimeS;
    char     build[24];   // firmware build date and time, null-terminated
};  // 32 bytes

// Sent with only as much of text[] as the line needs (no terminator).
struct RemoteLogMessage {
    uint8_t  type;      // MSG_REMOTE_LOG
    uint8_t  src;       // module logging
    uint8_t  seq;       // +1 per line; a gap means lines were dropped
    char     text[REMOTE_LOG_TEXT_MAX];
};  // 3 + text bytes

// Sent with only as much of text[] as the command needs (no terminator).
struct RemoteCommandMessage {
    uint8_t  type;      // MSG_REMOTE_COMMAND
    uint8_t  src;       // MODULE_HANDHELD
    uint8_t  target;    // MODULE_* that should run it
    char     text[REMOTE_COMMAND_TEXT_MAX];
};  // 3 + text bytes

struct OtaBeginMessage {
    uint8_t  type;      // MSG_OTA_BEGIN
    uint8_t  src;       // MODULE_HANDHELD
    uint8_t  target;    // MODULE_* being updated
    uint8_t  session;   // picked per update; packets from other sessions are ignored
    uint32_t size;      // image size in bytes
    uint8_t  md5[16];   // MD5 of the whole image
};  // 24 bytes

// Sent with only as much of data[] as the chunk needs.
struct OtaDataMessage {
    uint8_t  type;      // MSG_OTA_DATA
    uint8_t  src;       // MODULE_HANDHELD
    uint8_t  target;
    uint8_t  session;
    uint32_t offset;    // position of data[0] in the image
    uint8_t  data[OTA_CHUNK_MAX];
};  // 8 + data bytes

struct OtaControlMessage {
    uint8_t  type;      // MSG_OTA_END or MSG_OTA_ABORT
    uint8_t  src;       // MODULE_HANDHELD
    uint8_t  target;
    uint8_t  session;
};  // 4 bytes

struct OtaAckMessage {
    uint8_t  type;       // MSG_OTA_ACK
    uint8_t  src;        // module being updated
    uint8_t  session;
    uint8_t  status;     // OTA_STATUS_*
    uint32_t nextOffset; // bytes received so far: where the next chunk should start
};  // 8 bytes
#pragma pack(pop)

// Broadcast MAC address – send here to reach every ESP-NOW peer at once.
static const uint8_t MESH_BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
