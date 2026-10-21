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
│   ├── mesh_protocol.h     ← inter-module message definitions (included by all modules)
│   └── mesh_remote.h       ← steering/navigation side of the mesh gateway
├── tools/
│   ├── route_planner/      ← web route planner (GitHub Pages); uploads routes over USB
│   └── mesh_gateway/       ← logs, commands and firmware updates through the handheld
└── modules/
    ├── steering/            ← ESP32-S3 on Maslow 4 board; drives the rudder motor
    │   ├── platformio.ini
    │   └── src/             ← C++ firmware
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
| ESP-NOW | Receives `MSG_SET_ANGLE` from any module on WiFi channel 1 |

**Build & flash:**
```bash
cd modules/steering
pio run -t upload       # flash firmware over USB
pio run -t meshupload   # or over the mesh, through the handheld (see below)
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

**Routes:** stores up to 10 routes of up to 50 waypoints in NVS. Routes can
be recorded on the device (Navigation → Routes → Append current) or planned
on a map with the [route planner](#route-planner) and uploaded over USB.
"Send + Start" sends the active route to the navigation module.

The navigation module follows the route as a track — the line through the
waypoints — rather than steering for each waypoint in turn. It joins the
track at the point closest to the boat (so a route can be started from
anywhere along it) and steers for a point 30 m further along the track.
Progress only moves forward: a waypoint the boat misses is left behind, not
circled back for. Only the last waypoint's radius matters; the route
finishes inside it, or once the boat is past the end.

**Build & flash:**
```bash
cd modules/handheld_controller
pio run -t upload
```

---

## Route planner

`tools/route_planner/` is a static web app, published to GitHub Pages by
`.github/workflows/route-planner-pages.yml` on every push to `main` that
touches it (one-time setup: repo **Settings → Pages → Source: GitHub
Actions**). Once deployed it lives at
<https://barboursmith.github.io/Sauna-Boat-Software/>.

- Shows all 10 controller route slots on an OpenStreetMap / satellite map
  with OpenSeaMap sea marks.
- Click the map to add waypoints, drag to move, click a **+** between two
  points to insert one; set each waypoint's arrival radius (1–100 m; only
  the last waypoint's is used, to decide when the route is finished).
- Plug the handheld into a computer with USB and press **Connect
  controller** (Chrome or Edge on desktop — it uses the Web Serial API).
  The planner reads every route, shows which have unsaved changes, uploads
  them (verified by reading back), and can set the active route. While
  connected it also shows the boat's live GPS position.
- Works offline too: routes are kept in the browser until uploaded, and can
  be imported/exported as GPX (long GPX tracks are simplified to 50 points).
- Open the page with `?sim` to try it against a simulated controller.

To run it locally: `python3 -m http.server 8000 --bind 127.0.0.1 --directory tools/route_planner`
and open <http://127.0.0.1:8000/> (Web Serial needs `localhost`/`127.0.0.1` or HTTPS).

### USB route protocol

The handheld's USB serial port (115200 baud) carries a line protocol alongside
its debug log; protocol lines start with `$`. The full command list is in the
"USB serial route protocol" section of `modules/handheld_controller/src/main.cpp`.
For example, from `pio device monitor`:

```
$HELLO        → $HELLO,SAUNA_HANDHELD,1,10,50,1,100,15,0
$GET,0        → $ROUTE,0,2,Harbor loop
                $WP,0,0,476395000,-1223365000,5
                $WP,0,1,476440000,-1223345000,8
                $OK,GET,0
```

Uploads are staged (`$PUT` → `$WP`… → `$COMMIT`) and only replace the stored
route once every waypoint has arrived and validated.

The same port also carries the mesh gateway's `$MON`, `$CMD` and `$OTA`
commands, listed in the "USB gateway to the mesh" section of the same file.

---

## Debugging and updating over the mesh

The steering and navigation boards' USB ports are hard to reach on the boat,
so the handheld doubles as a gateway: plug the handheld into the laptop and
`tools/mesh_gateway/gateway.py` reaches the other modules over ESP-NOW. No
WiFi network is involved, so the laptop can stay on a phone hotspot.

From a module's folder (`modules/navigation`, `modules/steering`, or the repo
root for the steering firmware in `src/`):

```bash
pio run -t meshupload    # build, then install over the mesh
pio run -t meshmonitor   # show every module's log
```

Or run the script with any Python that has pyserial (PlatformIO's does):

```bash
python tools/mesh_gateway/gateway.py monitor             # logs; --traffic adds decoded mesh messages
python tools/mesh_gateway/gateway.py command navigation c   # start compass calibration
python tools/mesh_gateway/gateway.py upload navigation modules/navigation/.pio/build/navigation/firmware.bin
```

How it behaves:

- **Logs.** Steering and navigation print through `Log` (`shared/mesh_remote.h`):
  output still goes to their USB port and, while the gateway is listening,
  is also broadcast over the mesh. Prints from other source files (such as
  `RCServo.cpp`) only reach the USB port.
- **Commands.** Text sent with `command` is handled as if typed into the
  module's serial port. Navigation takes `c` to calibrate the compass.
- **Updates.** The image is written to the module's spare app partition and
  checked (length, MD5, valid app for that chip) before the module restarts
  into it; a failed or interrupted transfer leaves the running firmware
  alone. Navigation refuses an update while heading hold or a route is
  active. While receiving, a module ignores all other mesh messages, so the
  rudder holds where it was: update at the dock.
- **Rollback.** New firmware is kept once it hears the mesh. If it crashes
  before then, or hears nothing for two minutes, the bootloader goes back to
  the previous firmware.
- **First time.** Each board needs one USB flash of firmware with this code
  before it can be updated over the mesh, and the handheld needs its own
  update over its USB port.
- The mesh isn't authenticated: anything in ESP-NOW range that speaks the
  protocol could send firmware, just as it can already send steering
  commands.

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

All modules use **WiFi channel 1** (`MESH_WIFI_CHANNEL`). None of them hosts a
web page or creates a WiFi network; the radio is used only for ESP-NOW.

---

## Hardware notes

| Board | Module | Chip |
|-------|--------|------|
| Maslow 4 | steering | ESP32-S3 |
| Pavloff PCB | handheld_controller | ESP32-S3 (battery + MPU-6050 on-board) |
