#!/usr/bin/env python3
"""Reach the steering and navigation modules through the handheld's USB port.

The handheld relays between its USB port and the boat's ESP-NOW mesh (see
"USB gateway to the mesh" in modules/handheld_controller/src/main.cpp), so the
modules whose USB ports are hard to get at can still be debugged and updated.

    gateway.py monitor [--traffic]          show the modules' logs as they run
    gateway.py command <module> <text>      run a serial command on a module
    gateway.py upload <module> <firmware>   install new firmware over the mesh

<module> is steering or navigation. Needs pyserial, which PlatformIO's Python
has: run it as ~/.platformio/penv/bin/python gateway.py ..., or use
`pio run -t meshupload` / `pio run -t meshmonitor` in a module's folder.
"""

import argparse
import base64
import hashlib
import struct
import sys
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is missing: run this with PlatformIO's Python "
             "(~/.platformio/penv/bin/python) or `pip install pyserial`.")

MODULES = {"steering": 0x01, "navigation": 0x03}
MODULE_NAMES = {0x01: "steering", 0x03: "navigation", 0x04: "handheld"}

RESET_REASONS = {
    1: "power-on", 2: "reset pin", 3: "software restart", 4: "crash",
    5: "interrupt watchdog", 6: "task watchdog", 7: "watchdog", 8: "deep sleep",
    9: "brownout",
}
FW_CONFIRMED, FW_UNCONFIRMED, FW_UPDATING = 0, 1, 2

MON_LOGS, MON_TRAFFIC = 1, 2
MON_REFRESH_S = 2.0       # the handheld stops relaying 5 s after the last $MON
OTA_CHUNK = 240           # OTA_CHUNK_MAX in shared/mesh_protocol.h
ESPRESSIF_USB_VID = 0x303A


class GatewayError(Exception):
    pass


# ---------------------------------------------------------------------------
# Talking to the handheld
# ---------------------------------------------------------------------------
class Gateway:
    """The handheld's line protocol over USB."""

    def __init__(self, port):
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        # The ESP32-S3's USB-Serial-JTAG port holds the chip in reset while
        # RTS is high and DTR low. Lines are set one at a time, so drop RTS
        # before DTR to never pass through that state (as the route planner
        # ends up: both low).
        self.ser.open()
        try:
            self.ser.rts = False
            self.ser.dtr = False
        except (OSError, serial.SerialException):
            pass  # not every port has control lines
        self.buf = b""
        self.on_line = None    # gets every line that isn't a reply being waited for
        self.mon_flags = 0
        self.mon_sent = 0.0

    def close(self):
        self.ser.close()

    def send(self, line):
        self.ser.write((line + "\n").encode())

    def read_line(self, timeout):
        """Next line from the handheld, or None after timeout seconds."""
        deadline = time.monotonic() + timeout
        while True:
            nl = self.buf.find(b"\n")
            if nl >= 0:
                line, self.buf = self.buf[:nl], self.buf[nl + 1:]
                return line.decode(errors="replace").rstrip("\r")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.ser.timeout = min(remaining, 0.1)
            self.buf += self.ser.read(max(1, self.ser.in_waiting))

    def request(self, line, timeout=5.0):
        """Sends a command and returns its $OK/$ERR reply."""
        cmd = line.split(",", 1)[0][1:]
        self.send(line)
        deadline = time.monotonic() + timeout
        while True:
            self.keep_monitoring()
            reply = self.read_line(max(0.0, deadline - time.monotonic()))
            if reply is None:
                if time.monotonic() >= deadline:
                    raise GatewayError(f"the handheld didn't answer {cmd}")
                continue
            if reply.startswith(f"$OK,{cmd}") or reply.startswith(f"$ERR,{cmd}"):
                return reply
            if self.on_line:
                self.on_line(reply)

    def monitor(self, flags):
        """Asks the handheld to relay the mesh (MON_* flags) until stopped."""
        self.mon_flags = flags
        self.mon_sent = 0.0
        self.keep_monitoring()

    def keep_monitoring(self):
        if self.mon_flags and time.monotonic() - self.mon_sent >= MON_REFRESH_S:
            self.send(f"$MON,{self.mon_flags}")
            self.mon_sent = time.monotonic()

    def poll(self, timeout):
        """Reads lines for up to timeout seconds, passing them to on_line."""
        deadline = time.monotonic() + timeout
        while True:
            self.keep_monitoring()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            line = self.read_line(min(remaining, 0.2))
            if line is not None and self.on_line:
                self.on_line(line)


def open_handheld(port):
    """Finds the handheld among the USB serial ports by asking each $HELLO."""
    if port:
        candidates = [port]
    else:
        ports = [p for p in list_ports.comports() if p.vid is not None]
        ports.sort(key=lambda p: p.vid != ESPRESSIF_USB_VID)
        candidates = [p.device for p in ports]

    for device in candidates:
        try:
            gw = Gateway(device)
        except (OSError, serial.SerialException):
            continue
        gw.send("$HELLO")
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            line = gw.read_line(deadline - time.monotonic())
            if line and line.startswith("$HELLO,SAUNA_HANDHELD"):
                return gw
        gw.close()
    sys.exit("Couldn't find the handheld. Plug it in over USB (and close the route "
             "planner if it has the port open), or pass --port.")


# ---------------------------------------------------------------------------
# Showing what the handheld relays
# ---------------------------------------------------------------------------
COLORS = {"steering": "36", "navigation": "35", "handheld": "33"}


def fmt_uptime(seconds):
    h, rest = divmod(seconds, 3600)
    return f"{h}:{rest // 60:02}:{rest % 60:02}"


class ModuleState:
    def __init__(self, uptime, build, fw_state):
        self.uptime = uptime
        self.build = build
        self.fw_state = fw_state
        self.seen = time.monotonic()
        self.log_seq = None


class Printer:
    """Prints relayed lines; keeps track of each module's last $INFO."""

    def __init__(self, traffic=False, only=None):
        self.traffic = traffic
        self.only = only               # (module name, text) -> bool: what to show
        self.modules = {}              # module id -> ModuleState
        self.last_shown = {}           # (type, src) -> time, to thin out periodic traffic
        self.color = sys.stdout.isatty()

    def out(self, who, text):
        if self.only and not self.only(who, text):
            return
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        tag = f"[{who}]"
        if self.color and who in COLORS:
            tag = f"\033[{COLORS[who]}m{tag}\033[0m"
        print(f"{stamp} {tag} {text}", flush=True)

    def line(self, line):
        if not line.startswith("$"):
            self.out("handheld", line)
            return
        kind, _, rest = line.partition(",")
        try:
            if kind == "$LOG":
                module, seq, text = rest.split(",", 2)
                self.log(int(module), int(seq), text)
            elif kind == "$INFO":
                module, reset, fw_state, uptime, build = rest.split(",", 4)
                self.info(int(module), int(reset), int(fw_state), int(uptime), build)
            elif kind == "$RX" and self.traffic:
                self.packet(bytes.fromhex(rest))
            elif line.startswith("$ERR,MON"):
                sys.exit("The handheld doesn't support the mesh gateway yet: "
                         "flash the latest handheld firmware over USB.")
        except ValueError:
            self.out("handheld", line)

    def log(self, module, seq, text):
        name = MODULE_NAMES.get(module, f"module {module}")
        state = self.modules.get(module)
        if state is not None:
            if state.log_seq is not None and seq != (state.log_seq + 1) % 256:
                self.out(name, f"... {(seq - state.log_seq - 1) % 256} log line(s) lost")
            state.log_seq = seq
        self.out(name, text)

    def info(self, module, reset, fw_state, uptime, build):
        name = MODULE_NAMES.get(module, f"module {module}")
        prev = self.modules.get(module)
        state = ModuleState(uptime, build, fw_state)
        if prev is None:
            self.out(name, f"online: firmware built {build}, up {fmt_uptime(uptime)}, "
                           f"last restart: {RESET_REASONS.get(reset, 'unknown')}")
        else:
            state.log_seq = prev.log_seq
            if uptime < prev.uptime:
                state.log_seq = None
                self.out(name, f"restarted ({RESET_REASONS.get(reset, 'unknown')}): "
                               f"firmware built {build}")
            elif fw_state != prev.fw_state and fw_state == FW_CONFIRMED:
                self.out(name, "new firmware kept")
        if fw_state == FW_UNCONFIRMED and (prev is None or prev.fw_state != FW_UNCONFIRMED):
            self.out(name, "running new firmware; it is kept once it hears the mesh")
        self.modules[module] = state

    def packet(self, pkt):
        if len(pkt) < 2:
            return
        key = (pkt[0], pkt[1])
        if pkt[0] == 0x09:
            key += (pkt[2:6],)  # autopilot settings: once a second per setting
        if pkt[0] in PERIODIC:
            now = time.monotonic()
            if now - self.last_shown.get(key, 0) < 1.0:
                return
            self.last_shown[key] = now
        self.out(MODULE_NAMES.get(pkt[1], f"module {pkt[1]}"), describe(pkt))


# Messages sent several times a second; --traffic shows each at most once a second.
PERIODIC = {0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x09, 0x0E}

AP_PARAMS = ["Kp", "Ki", "Kd", "max output", "deadband", "rate limit"]
ROUTE_STATES = ["idle", "uploading", "ready", "running", "aborted", "complete"]
ROUTE_ABORTS = ["none", "bad format", "bad sequence", "bad waypoint", "no GPS fix",
                "no heading", "stop requested", "empty route"]


def name_of(names, i):
    return names[i] if 0 <= i < len(names) else str(i)


def describe(pkt):
    """One-line description of a mesh message (layouts in shared/mesh_protocol.h)."""
    t = pkt[0]
    try:
        if t in (0x01, 0x02, 0x03, 0x04, 0x07, 0x08, 0x09):
            _, _, v1, v2 = struct.unpack_from("<BBff", pkt)
            if t == 0x01:
                return f"steering command {v1:+.2f}"
            if t == 0x02:
                return f"set angle {v1:.1f}°"
            if t == 0x03:
                return f"rudder angle {v1:.1f}° (target {v2:.1f}°)"
            if t == 0x04:
                return f"battery level {v1:.0f}{', charging' if v2 > 0.5 else ''}"
            if t == 0x07:
                return f"heading hold on, target {v2:.0f}°" if v1 > 0.5 else "heading hold off"
            which = "set" if t == 0x08 else "autopilot"
            return f"{which} {name_of(AP_PARAMS, int(v1))} = {v2:.4g}"
        if t == 0x05:
            _, _, lx, ly, rx, ry, buttons = struct.unpack_from("<BBffffH", pkt)
            return f"stick x {lx:+.2f} y {ly:+.2f}, buttons {buttons:#06x}"
        if t == 0x06:
            _, _, fix, sats, lat, lon, knots, course, heading = struct.unpack_from("<BBBBfffff", pkt)
            gps = f"{lat:.6f}, {lon:.6f}, {knots:.1f} kn, course {course:.0f}°" if fix else "no fix"
            return f"GPS {gps} ({sats} sats), heading {heading:.0f}°"
        if t == 0x0A:
            _, _, route, wps, chunks, _ = struct.unpack_from("<BBBBBB", pkt)
            return f"route {route} upload start: {wps} waypoints in {chunks} chunks"
        if t == 0x0B:
            _, _, route, index, count, _ = struct.unpack_from("<BBBBBB", pkt)
            return f"route {route} chunk {index} ({count} waypoints)"
        if t == 0x0C:
            return f"route {pkt[2]} upload end"
        if t == 0x0D:
            action = {1: "start", 2: "stop"}.get(pkt[2], str(pkt[2]))
            return f"route {pkt[3]} {action}"
        if t == 0x0E:
            _, _, route, st, idx, total, dist = struct.unpack_from("<BBBBBBH", pkt)
            return (f"route {route} {name_of(ROUTE_STATES, st)}: "
                    f"next waypoint {idx + 1}/{total}, {dist} m away")
        if t == 0x0F:
            return f"route {pkt[2]} passed waypoint {pkt[3] + 1}"
        if t == 0x10:
            return f"route {pkt[2]} complete"
        if t == 0x11:
            return f"route {pkt[2]} aborted: {name_of(ROUTE_ABORTS, pkt[3])}"
    except struct.error:
        pass
    return f"message {t:#04x}: {pkt.hex()}"


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
def cmd_monitor(gw, args):
    printer = Printer(traffic=args.traffic)
    gw.on_line = printer.line
    gw.monitor(MON_LOGS | (MON_TRAFFIC if args.traffic else 0))
    print("Listening through the handheld; Ctrl-C to stop.", flush=True)
    try:
        while True:
            gw.poll(1.0)
    except KeyboardInterrupt:
        gw.send("$MON,0")


def cmd_command(gw, args):
    name = args.module
    printer = Printer(only=lambda who, _text: who == name)
    gw.on_line = printer.line
    gw.monitor(MON_LOGS)
    gw.poll(1.5)  # let the module hear a ping, so it sends its log
    reply = gw.request(f"$CMD,{MODULES[name]},{args.text}")
    if not reply.startswith("$OK"):
        sys.exit(f"The handheld refused the command: {reply}")
    print(f"Sent; showing {name}'s log. Ctrl-C to stop.", flush=True)
    try:
        while True:
            gw.poll(1.0)
    except KeyboardInterrupt:
        gw.send("$MON,0")


def cmd_upload(gw, args):
    name = args.module
    module = MODULES[name]
    with open(args.firmware, "rb") as f:
        image = f.read()
    if not image or image[0] != 0xE9:
        sys.exit(f"{args.firmware} doesn't look like ESP32 app firmware (firmware.bin).")

    # Show only the module's status and the update's progress from its log
    # (log lines start with a "[Tag]"); its $INFO says when it has restarted
    # and whether it kept the new firmware.
    printer = Printer(only=lambda who, text: who == name and
                      (not text.startswith("[") or text.startswith("[Remote]")))
    gw.on_line = printer.line
    gw.monitor(MON_LOGS)
    gw.poll(1.5)

    def ota(line, timeout=5.0):
        reply = gw.request(line, timeout)
        if not reply.startswith("$OK"):
            raise GatewayError(reply.split(",", 2)[2] if reply.count(",") >= 2 else reply)
        return reply

    size = len(image)
    print(f"Sending {args.firmware} ({size // 1024} kB) to {name}...", flush=True)
    start = time.monotonic()
    try:
        ota(f"$OTA,BEGIN,{module},{size},{hashlib.md5(image).hexdigest()}", timeout=20)
        offset = 0
        shown = -1
        while offset < size:
            chunk = base64.b64encode(image[offset:offset + OTA_CHUNK]).decode()
            next_offset = int(ota(f"$OTA,DATA,{offset},{chunk}").split(",")[2])
            if not 0 <= next_offset <= size:
                raise GatewayError(f"module asked for offset {next_offset}")
            offset = next_offset
            percent = offset * 100 // size
            if percent // 10 != shown // 10:
                shown = percent
                rate = offset / 1024 / max(time.monotonic() - start, 0.001)
                print(f"  {percent:3}%  ({rate:.1f} kB/s)", flush=True)
        ota("$OTA,END", timeout=30)
    except KeyboardInterrupt:
        gw.send("$OTA,ABORT")
        sys.exit("Cancelled; the module keeps its current firmware.")
    except GatewayError as e:
        try:
            gw.request("$OTA,ABORT", timeout=2)
        except GatewayError:
            pass
        sys.exit(f"Update failed: {e}. The module keeps its current firmware.")

    print(f"Sent in {time.monotonic() - start:.0f} s; {name} checked it and is restarting.",
          flush=True)
    # Done once the module reports, from an uptime that started after END,
    # that it has kept the new firmware.
    ended = time.monotonic()
    while time.monotonic() - ended < 60:
        gw.poll(0.5)
        now = printer.modules.get(module)
        if (now and now.fw_state == FW_CONFIRMED and now.seen > ended and
                now.uptime <= now.seen - ended + 2):
            print(f"Done: {name} is running firmware built {now.build}.")
            gw.send("$MON,0")
            return
    gw.send("$MON,0")
    sys.exit(f"{name} hasn't reported back since restarting. Run `gateway.py monitor` to "
             "check it; if the new firmware can't hear the mesh it goes back to the old one "
             "within two minutes.")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", help="the handheld's serial port (found automatically)")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("monitor", help="show the modules' logs")
    p.add_argument("--traffic", action="store_true",
                   help="also show the mesh messages (periodic ones at most once a second)")

    p = sub.add_parser("command", help="run a serial command on a module")
    p.add_argument("module", choices=MODULES)
    p.add_argument("text", help="e.g. c to start compass calibration on navigation")

    p = sub.add_parser("upload", help="install new firmware on a module over the mesh")
    p.add_argument("module", choices=MODULES)
    p.add_argument("firmware", help="the module's firmware.bin")

    args = parser.parse_args()
    gw = open_handheld(args.port)
    try:
        {"monitor": cmd_monitor, "command": cmd_command, "upload": cmd_upload}[args.command](gw, args)
    except GatewayError as e:
        sys.exit(str(e))
    finally:
        gw.close()


if __name__ == "__main__":
    main()
