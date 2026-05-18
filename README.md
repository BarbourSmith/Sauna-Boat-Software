# Sauna Boat Software

Firmware for the sauna boat's embedded systems. The boat is controlled by a
network of ESP32 modules that communicate over **ESP-NOW** — a connectionless
WiFi-layer protocol that behaves like a CAN bus: any module can broadcast a
message, and any other module can listen and act on it.

---

## Repository layout

```
.
├── shared/
│   └── mesh_protocol.h     ← inter-module message definitions (included by all modules)
└── modules/
    ├── steering/            ← ESP32-S3 on Maslow 4 board; drives the rudder motor
    │   ├── platformio.ini
    │   ├── src/             ← C++ firmware
    │   └── data/            ← LittleFS web UI (index.html)
    └── handheld_controller/ ← ESP32-S3 (Pavloff board); custom handheld with
        ├── platformio.ini     analog joystick, SSD1306 OLED, MPU-6050 wake-on-
        └── src/               motion, and on-board LiPo
```

Adding a new module: create `modules/<name>/`, give it a `platformio.ini` with
`-I../../shared` in `build_flags`, assign it a `MODULE_*` ID and any new
`MSG_*` types in `shared/mesh_protocol.h`.

---

## Modules

### `steering` — ESP32-S3 (Maslow 4 control board)

Controls the boat's rudder via a DC motor with an AS5600 magnetic encoder for
position feedback. Runs a PID loop to hold a target heading.

**Interfaces:**
| Interface | Details |
|-----------|---------|
| WiFi AP | SSID `SaunaBoatSteering`, password `12345678`, IP `192.168.4.1` |
| Web UI | Open `http://192.168.4.1` — compass display, heading slider, PID settings |
| ESP-NOW | Receives `MSG_SET_ANGLE` from any module on WiFi channel 1 |

**Build & flash:**
```bash
cd modules/steering
pio run -t upload       # flash firmware
pio run -t uploadfs     # flash web UI (index.html → LittleFS)
```

---

### `handheld_controller` — ESP32-S3 (Pavloff board)

Custom handheld controller built on the Pavloff PCB. It is a self-contained
input device with an analog joystick for steering, a
0.96" SSD1306 OLED for live boat telemetry, an on-board LiPo with battery
monitor, and an MPU-6050 used as a wake-on-motion source.

It joins the boat over the existing ESP-NOW mesh on `MESH_WIFI_CHANNEL`
(no Bluetooth, no pairing) and broadcasts the same `MSG_CONTROLLER_INPUT`
and `MSG_SET_STEERING` messages expected by steering and navigation receivers.

**Stick X-axis** → `MSG_SET_STEERING.value1` in `[-1.0, 1.0]` (raw, no
deadzone applied here beyond the joystick centring band).
**Stick click** → `CTRL_BTN_L3` bit in `MSG_CONTROLLER_INPUT.buttons`.

**Display:** battery %, link state, GPS heading + fix + speed (from
`MSG_NAV_STATUS`), rudder actual → target (from `MSG_ANGLE_STATUS`), and a
live joystick crosshair.

**Power:** auto deep-sleep after 60 s of no stick deflection or button
press. Wakes on either MPU-6050 motion (EXT1, GPIO 18) or the joystick
button (EXT0, GPIO 6 LOW). Toggle motion-wake by writing the `wakeOnMove`
bool in the `settings` Preferences namespace.

**Wiring:**
| Signal | GPIO | Notes |
|--------|------|-------|
| MPU-6050 SDA / OLED SDA | 8  | shared I2C bus, 400 kHz |
| MPU-6050 SCL / OLED SCL | 9  | |
| MPU-6050 INT            | 18 | EXT1 wake source |
| Battery sense           | 7  | 27k / 68k divider |
| Status LED              | 47 | active HIGH |
| Joystick VRx            | 4  | ADC1_CH3 (default — change in `main.cpp` if rewired) |
| Joystick VRy            | 5  | ADC1_CH4 |
| Joystick SW             | 6  | INPUT_PULLUP, active LOW |

**Build & flash:**
```bash
cd modules/handheld_controller
pio run -t upload
```

---

## Mesh protocol

All inter-module traffic uses the structs and constants in `shared/mesh_protocol.h`.

| Field | Size | Description |
|-------|------|-------------|
| `type` | 1 byte | `MSG_*` constant identifying the message |
| `src`  | 1 byte | `MODULE_*` constant identifying the sender |
| `value1` | 4 bytes float | primary payload |
| `value2` | 4 bytes float | secondary payload (0 if unused) |

| Message | Type byte | Sender | value1 | value2 |
|---------|-----------|--------|--------|--------|
| `MSG_SET_STEERING` | `0x01` | handheld_controller / navigation | normalized steering [-1.0, 1.0] | — |
| `MSG_ANGLE_STATUS` | `0x02` | steering | current angle (°) | target angle (°) |

All modules use **WiFi channel 1** (`MESH_WIFI_CHANNEL`). The steering module's
soft-AP is pinned to that channel so phone connections and ESP-NOW coexist on
the same radio.

---

## Hardware notes

| Board | Module | Chip |
|-------|--------|------|
| Maslow 4 | steering | ESP32-S3 |
| Pavloff PCB | handheld_controller | ESP32-S3 (battery + MPU-6050 on-board) |
