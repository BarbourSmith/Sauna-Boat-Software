// controller.js – client for the handheld controller's USB serial route protocol.
//
// The protocol is documented next to its implementation in
// modules/handheld_controller/src/main.cpp ("USB serial route protocol").
// Protocol lines start with '$'; everything else on the port is debug log.

const BAUD_RATE = 115200;
const COMMAND_TIMEOUT_MS = 3000;
const CONNECT_TIMEOUT_MS = 10000;
const PROTOCOL_VERSION = 1;

export class ProtocolError extends Error {}

// Web Serial transport (Chrome / Edge on desktop).
export class WebSerialTransport {
  static get supported() {
    return 'serial' in navigator;
  }

  constructor() {
    this.onLine = () => {};
    this.onClose = () => {};
    this.port = null;
    this.reader = null;
    this.writer = null;
    this.closing = false;
  }

  async open() {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: BAUD_RATE });
    // The ESP32-S3's USB-Serial-JTAG port resets the chip for some DTR/RTS
    // combinations, so hold both low.
    try {
      await this.port.setSignals({ dataTerminalReady: false, requestToSend: false });
    } catch {
      // Not all platforms support setting signals; harmless if it fails.
    }
    this.writer = this.port.writable.getWriter();
    this.readDone = this.readLoop();
  }

  async readLoop() {
    const decoder = new TextDecoder();
    let buffer = '';
    while (this.port.readable && !this.closing) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          buffer += decoder.decode(value, { stream: true });
          let newline;
          while ((newline = buffer.indexOf('\n')) >= 0) {
            const line = buffer.slice(0, newline).replace(/\r$/, '');
            buffer = buffer.slice(newline + 1);
            this.onLine(line);
          }
          if (buffer.length > 4096) buffer = '';
        }
      } catch {
        // Framing/overrun errors are recoverable; a lost device ends the loop
        // because port.readable becomes null.
      } finally {
        this.reader.releaseLock();
        this.reader = null;
      }
    }
    if (!this.closing) this.onClose();
  }

  async write(text) {
    await this.writer.write(new TextEncoder().encode(text));
  }

  async close() {
    this.closing = true;
    try { await this.reader?.cancel(); } catch { /* already gone */ }
    await this.readDone;
    try { this.writer?.releaseLock(); } catch { /* already released */ }
    try { await this.port?.close(); } catch { /* already closed */ }
  }
}

// Splits a protocol line into at most `count` fields; the last keeps commas.
function fields(line, count) {
  const parts = line.split(',');
  if (parts.length <= count) return parts;
  return [...parts.slice(0, count - 1), parts.slice(count - 1).join(',')];
}

export class Controller {
  constructor(transport) {
    this.transport = transport;
    this.pending = null;
    this.queue = Promise.resolve();
    this.queued = 0;
    this.onLog = () => {};
    this.onDisconnect = () => {};
    transport.onLine = (line) => this.handleLine(line);
    transport.onClose = () => {
      this.pending?.fail(new Error('Controller disconnected'));
      this.onDisconnect();
    };
  }

  get busy() {
    return this.queued > 0;
  }

  // Opens the port and waits for the firmware to answer $HELLO. The chip may
  // reboot when the port opens, so keep asking for a few seconds.
  async connect() {
    await this.transport.open();
    const deadline = Date.now() + CONNECT_TIMEOUT_MS;
    for (;;) {
      try {
        this.info = await this.hello(800);
        return this.info;
      } catch (err) {
        if (err instanceof ProtocolError || Date.now() > deadline) {
          await this.close();
          throw err instanceof ProtocolError ? err : new Error(
            'The controller did not answer. Check that it is running firmware with USB route support.');
        }
      }
    }
  }

  async close() {
    await this.transport.close();
  }

  handleLine(line) {
    this.onLog(line);
    if (line.startsWith('$') && this.pending) this.pending.onLine(line);
  }

  // Sends one command and resolves with the protocol lines of its reply.
  // Commands are serialised so replies can't interleave.
  command(line, { isDone, timeout = COMMAND_TIMEOUT_MS } = {}) {
    const name = line.slice(1).split(',')[0];
    const done = isDone || ((l) => l === `$OK,${name}` || l.startsWith(`$OK,${name},`));
    const run = () => new Promise((resolve, reject) => {
      const lines = [];
      const finish = () => {
        clearTimeout(timer);
        this.pending = null;
      };
      const timer = setTimeout(() => {
        finish();
        reject(new Error(`No reply from the controller to ${name}`));
      }, timeout);
      this.pending = {
        onLine: (l) => {
          if (l.startsWith(`$ERR,${name},`) || l.startsWith('$ERR,LINE,')) {
            finish();
            reject(new ProtocolError(fields(l, 3)[2] || 'error'));
            return;
          }
          lines.push(l);
          if (done(l)) {
            finish();
            resolve(lines);
          }
        },
        fail: (err) => {
          finish();
          reject(err);
        },
      };
      this.onLog(`> ${line}`);
      this.transport.write(line + '\n').catch((err) => this.pending?.fail(err));
    });

    this.queued++;
    const result = this.queue.then(run, run).finally(() => this.queued--);
    this.queue = result.catch(() => {});
    return result;
  }

  async hello(timeout) {
    const [line] = await this.command('$HELLO', { isDone: (l) => l.startsWith('$HELLO,'), timeout });
    const f = line.split(',');
    if (f[1] !== 'SAUNA_HANDHELD') throw new ProtocolError('This device is not a Sauna Boat handheld controller.');
    const version = Number(f[2]);
    if (version !== PROTOCOL_VERSION) {
      throw new ProtocolError(`The controller speaks route protocol v${version}; this page supports v${PROTOCOL_VERSION}.`);
    }
    return {
      version,
      routes: Number(f[3]),
      waypoints: Number(f[4]),
      radiusMin: Number(f[5]),
      radiusMax: Number(f[6]),
      nameLength: Number(f[7]),
      activeRoute: Number(f[8]),
    };
  }

  async getRoute(id) {
    const lines = await this.command(`$GET,${id}`);
    let name = '';
    let count = -1;
    const waypoints = [];
    for (const line of lines) {
      if (line.startsWith('$ROUTE,')) {
        const f = fields(line, 4);
        count = Number(f[2]);
        name = f[3] ?? '';
      } else if (line.startsWith('$WP,')) {
        const f = line.split(',');
        waypoints[Number(f[2])] = { latE7: Number(f[3]), lonE7: Number(f[4]), radius: Number(f[5]) };
      }
    }
    if (count < 0 || waypoints.length !== count || waypoints.includes(undefined)) {
      throw new Error(`Route ${id + 1} came back incomplete from the controller`);
    }
    return { name, waypoints };
  }

  async putRoute(id, route) {
    await this.command(`$PUT,${id},${route.waypoints.length},${route.name}`);
    for (const [i, wp] of route.waypoints.entries()) {
      await this.command(`$WP,${id},${i},${wp.latE7},${wp.lonE7},${wp.radius}`);
    }
    await this.command(`$COMMIT,${id}`);
  }

  async setActive(id) {
    await this.command(`$ACTIVE,${id}`);
  }

  async position() {
    const [line] = await this.command('$POS', { isDone: (l) => l.startsWith('$POS,') });
    const f = line.split(',');
    return { fix: Number(f[1]), latE7: Number(f[2]), lonE7: Number(f[3]), heading: Number(f[4]) };
  }
}
