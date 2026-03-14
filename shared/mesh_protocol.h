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
// Add future modules here: #define MODULE_ENGINE 0x03, etc.

// ---------------------------------------------------------------------------
// Message types
// ---------------------------------------------------------------------------
// MSG_SET_ANGLE  – sent by the controller to the steering module.
//   value1 = desired heading in degrees [0, 360).
//   value2 = unused (0).
#define MSG_SET_ANGLE      0x01

// MSG_ANGLE_STATUS – broadcast by the steering module at ~10 Hz.
//   value1 = current measured angle (degrees).
//   value2 = current target angle   (degrees).
#define MSG_ANGLE_STATUS   0x02

// ---------------------------------------------------------------------------
// Message struct (max ESP-NOW payload is 250 bytes; this is 10 bytes)
// ---------------------------------------------------------------------------
#pragma pack(push, 1)
struct MeshMessage {
    uint8_t type;    // MSG_* constant
    uint8_t src;     // MODULE_* constant identifying the sender
    float   value1;  // primary payload value
    float   value2;  // secondary payload value (0 if unused)
};
#pragma pack(pop)

// Broadcast MAC address – send here to reach every ESP-NOW peer at once.
static const uint8_t MESH_BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
