// sim.js – in-browser stand-in for the handheld controller (open the page
// with ?sim). Speaks the same line protocol as the firmware so the planner can
// be tried, and tested, without hardware. Routes live only in memory.

const ROUTES = 10;
const MAX_WAYPOINTS = 50;
const RADIUS_MIN = 1;
const RADIUS_MAX = 100;
const NAME_LENGTH = 15;

function demoRoute(name, points) {
  return {
    name,
    waypoints: points.map(([lat, lon, radius = 5]) => ({
      latE7: Math.round(lat * 1e7),
      lonE7: Math.round(lon * 1e7),
      radius,
    })),
  };
}

export class SimTransport {
  constructor() {
    this.onLine = () => {};
    this.onClose = () => {};
    this.active = 0;
    this.put = null;
    this.routes = Array.from({ length: ROUTES }, (_, i) => ({ name: `Route ${i + 1}`, waypoints: [] }));
    this.routes[0] = demoRoute('Lake loop', [
      [47.6395, -122.3365, 8], [47.6440, -122.3345], [47.6462, -122.3300],
      [47.6431, -122.3262, 10], [47.6390, -122.3298],
    ]);
    this.routes[3] = demoRoute('To the bridge', [
      [47.6455, -122.3385], [47.6492, -122.3440, 15], [47.6512, -122.3478, 20],
    ]);
    this.boat = { lat: 47.6418, lon: -122.3330, heading: 40 };
  }

  async open() {
    this.emit('=== Sauna Boat Handheld Controller (simulated) ===');
    this.logTimer = setInterval(() => {
      this.boat.heading = (this.boat.heading + 3) % 360;
      this.boat.lat += Math.cos(this.boat.heading * Math.PI / 180) * 0.00002;
      this.boat.lon += Math.sin(this.boat.heading * Math.PI / 180) * 0.00003;
      this.emit('[Loop] joy x=+0.00 y=+0.00 btn=0 | menu=0 hold=0 trim=+0 | bat=4.05V 87% | mesh ok=0 fail=0');
    }, 1000);
  }

  async close() {
    clearInterval(this.logTimer);
  }

  async write(text) {
    for (const line of text.split('\n')) {
      if (line) setTimeout(() => this.handle(line), 4);
    }
  }

  emit(line) {
    this.onLine(line);
  }

  handle(line) {
    const comma = line.indexOf(',');
    const cmd = comma < 0 ? line : line.slice(0, comma);
    const args = comma < 0 ? '' : line.slice(comma + 1);
    const name = cmd.slice(1);
    const int = (s, min, max) => (/^-?\d+$/.test(s ?? '') && +s >= min && +s <= max ? +s : null);

    switch (cmd) {
      case '$HELLO':
        this.emit(`$HELLO,SAUNA_HANDHELD,1,${ROUTES},${MAX_WAYPOINTS},${RADIUS_MIN},${RADIUS_MAX},${NAME_LENGTH},${this.active}`);
        return;
      case '$PING':
        this.emit('$OK,PING');
        return;
      case '$POS': {
        const { lat, lon, heading } = this.boat;
        this.emit(`$POS,1,${Math.round(lat * 1e7)},${Math.round(lon * 1e7)},${heading.toFixed(1)}`);
        return;
      }
      case '$GET': {
        const id = int(args, 0, ROUTES - 1);
        if (id === null) return this.emit('$ERR,GET,bad route id');
        const route = this.routes[id];
        this.emit(`$ROUTE,${id},${route.waypoints.length},${route.name}`);
        route.waypoints.forEach((wp, i) => this.emit(`$WP,${id},${i},${wp.latE7},${wp.lonE7},${wp.radius}`));
        this.emit(`$OK,GET,${id}`);
        return;
      }
      case '$PUT': {
        const [idText, countText, ...rest] = args.split(',');
        const id = int(idText, 0, ROUTES - 1);
        const count = int(countText, 0, MAX_WAYPOINTS);
        if (id === null || count === null) return this.emit('$ERR,PUT,bad route id or waypoint count');
        const routeName = rest.join(',').replace(/[^\x20-\x7e]/g, ' ').slice(0, NAME_LENGTH).trimEnd();
        this.put = { id, count, name: routeName, waypoints: [] };
        this.emit(`$OK,PUT,${id}`);
        return;
      }
      case '$WP': {
        if (!this.put) return this.emit('$ERR,WP,no upload in progress');
        const f = args.split(',');
        const id = int(f[0], 0, ROUTES - 1);
        const idx = int(f[1], 0, MAX_WAYPOINTS - 1);
        const latE7 = int(f[2], -900000000, 900000000);
        const lonE7 = int(f[3], -1800000000, 1800000000);
        const radius = int(f[4], RADIUS_MIN, RADIUS_MAX);
        if (f.length < 5 || [id, idx, latE7, lonE7, radius].includes(null)) return this.emit('$ERR,WP,bad waypoint');
        if (id !== this.put.id || idx !== this.put.waypoints.length || idx >= this.put.count) {
          return this.emit(`$ERR,WP,expected waypoint ${this.put.waypoints.length} of route ${this.put.id}`);
        }
        this.put.waypoints.push({ latE7, lonE7, radius });
        this.emit(`$OK,WP,${idx}`);
        return;
      }
      case '$COMMIT': {
        const id = int(args, 0, ROUTES - 1);
        if (id === null || !this.put || id !== this.put.id) return this.emit('$ERR,COMMIT,no upload in progress for that route');
        if (this.put.waypoints.length !== this.put.count) {
          return this.emit(`$ERR,COMMIT,received ${this.put.waypoints.length} of ${this.put.count} waypoints`);
        }
        this.routes[id] = { name: this.put.name || `Route ${id + 1}`, waypoints: this.put.waypoints };
        this.put = null;
        this.emit(`$OK,COMMIT,${id}`);
        return;
      }
      case '$ACTIVE': {
        const id = int(args, 0, ROUTES - 1);
        if (id === null) return this.emit('$ERR,ACTIVE,bad route id');
        this.active = id;
        this.emit(`$OK,ACTIVE,${id}`);
        return;
      }
      default:
        this.emit(`$ERR,${name},unknown command`);
    }
  }
}
