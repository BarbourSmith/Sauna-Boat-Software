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
    └── controller/          ← plain ESP32; reads a PS3 controller via Bluetooth
        ├── platformio.ini
        └── src/             ← C++ firmware
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

### `controller` — ESP32 (plain, not S3)

Connects to a PS3 DualShock 3 controller via Bluetooth and translates joystick
input into heading commands sent to the steering module over ESP-NOW.

**Left stick X-axis** → heading change rate (max ±90 °/s at full deflection).
Stick centred → hold current heading.

**One-time pairing (required before first use):**
1. Flash the firmware and open the serial monitor.
2. Note the **Bluetooth MAC** printed on startup.
3. Use [SixaxisPairTool](https://github.com/user-attachments/files/11638679/SixaxisPairTool-0.3.1.zip) (Windows/Mac)
   or `sixaxispairer` (Linux) to write that MAC into your PS3 controller.
4. Press the **PS button** on the controller — it connects automatically.

**Build & flash:**
```bash
cd modules/controller
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
| `MSG_SET_ANGLE` | `0x01` | controller | target heading (°) | — |
| `MSG_ANGLE_STATUS` | `0x02` | steering | current angle (°) | target angle (°) |

All modules use **WiFi channel 1** (`MESH_WIFI_CHANNEL`). The steering module's
soft-AP is pinned to that channel so phone connections and ESP-NOW coexist on
the same radio.

---

## Hardware notes

| Board | Module | Chip |
|-------|--------|------|
| Maslow 4 | steering | ESP32-S3 |
| Generic dev board | controller | ESP32 (not S3) |

The PS3 controller communicates over Bluetooth; the ESP32-S3 used by the
steering module does not have the correct Bluetooth stack for `esp32-ps3`, which
is why the controller module uses a plain ESP32.
