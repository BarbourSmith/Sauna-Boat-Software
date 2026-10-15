// gpx.js – GPX import/export for routes.
//
// Export writes a GPX 1.1 <rte>; each point's arrival radius goes in a
// <sauna:radius> extension. Import accepts the first route, else the first
// track, else loose waypoints, and simplifies anything longer than the
// controller can store.

const GPX_NS = 'http://www.topografix.com/GPX/1/1';
const SAUNA_NS = 'https://github.com/BarbourSmith/Sauna-Boat-Software';

function escapeXml(text) {
  return String(text).replace(/[<>&'"]/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', "'": '&apos;', '"': '&quot;' }[c]));
}

export function routeToGpx(route) {
  const points = route.waypoints.map((wp, i) => [
    `    <rtept lat="${(wp.latE7 / 1e7).toFixed(7)}" lon="${(wp.lonE7 / 1e7).toFixed(7)}">`,
    `      <name>WP${i + 1}</name>`,
    `      <extensions><sauna:radius>${wp.radius}</sauna:radius></extensions>`,
    '    </rtept>',
  ].join('\n'));
  return [
    '<?xml version="1.0" encoding="UTF-8"?>',
    `<gpx version="1.1" creator="Sauna Boat Route Planner" xmlns="${GPX_NS}" xmlns:sauna="${SAUNA_NS}">`,
    '  <rte>',
    `    <name>${escapeXml(route.name)}</name>`,
    ...points,
    '  </rte>',
    '</gpx>',
    '',
  ].join('\n');
}

function children(parent, localName) {
  return Array.from(parent.getElementsByTagNameNS('*', localName));
}

function directChildText(parent, localName) {
  const el = Array.from(parent.children).find((c) => c.localName === localName);
  return el?.textContent.trim() ?? '';
}

function readPoint(el) {
  const lat = parseFloat(el.getAttribute('lat'));
  const lon = parseFloat(el.getAttribute('lon'));
  if (!Number.isFinite(lat) || !Number.isFinite(lon) || Math.abs(lat) > 90 || Math.abs(lon) > 180) return null;
  const radius = parseFloat(children(el, 'radius')[0]?.textContent);
  return { lat, lon, radius: Number.isFinite(radius) ? radius : null };
}

// Returns { name, points: [{lat, lon, radius|null}], source, originalCount }.
export function parseGpx(text, maxPoints) {
  const doc = new DOMParser().parseFromString(text, 'application/xml');
  if (doc.getElementsByTagName('parsererror').length) throw new Error('That file is not valid GPX.');

  let name = '';
  let points = [];
  let source = '';

  const route = children(doc, 'rte').find((r) => children(r, 'rtept').length);
  const track = children(doc, 'trk').find((t) => children(t, 'trkpt').length);
  if (route) {
    name = directChildText(route, 'name');
    points = children(route, 'rtept').map(readPoint);
    source = 'route';
  } else if (track) {
    name = directChildText(track, 'name');
    points = children(track, 'trkpt').map(readPoint);
    source = 'track';
  } else {
    points = children(doc, 'wpt').map(readPoint);
    source = 'waypoints';
  }
  points = points.filter(Boolean);
  if (!points.length) throw new Error('No routes, tracks or waypoints found in that file.');

  const originalCount = points.length;
  if (points.length > maxPoints) points = simplify(points, maxPoints);
  return { name, points, source, originalCount };
}

// Ramer–Douglas–Peucker on a local flat projection, with the tolerance
// binary-searched so the result fits in maxPoints.
function simplify(points, maxPoints) {
  const lat0 = (points.reduce((sum, p) => sum + p.lat, 0) / points.length) * Math.PI / 180;
  const metersPerDegree = 111320;
  const xy = points.map((p) => [p.lon * metersPerDegree * Math.cos(lat0), p.lat * metersPerDegree]);

  let lo = 0;
  let hi = 2e6;
  let best = [0, points.length - 1];
  for (let i = 0; i < 40; i++) {
    const tolerance = (lo + hi) / 2;
    const kept = rdp(xy, tolerance);
    if (kept.length <= maxPoints) {
      best = kept;
      hi = tolerance;
    } else {
      lo = tolerance;
    }
  }
  return best.map((i) => points[i]);
}

function rdp(xy, tolerance) {
  const keep = new Uint8Array(xy.length);
  keep[0] = keep[xy.length - 1] = 1;
  const stack = [[0, xy.length - 1]];
  while (stack.length) {
    const [start, end] = stack.pop();
    let maxDist = -1;
    let index = -1;
    for (let i = start + 1; i < end; i++) {
      const d = segmentDistance(xy[i], xy[start], xy[end]);
      if (d > maxDist) {
        maxDist = d;
        index = i;
      }
    }
    if (index >= 0 && maxDist > tolerance) {
      keep[index] = 1;
      stack.push([start, index], [index, end]);
    }
  }
  return Array.from(keep.keys()).filter((i) => keep[i]);
}

function segmentDistance([px, py], [ax, ay], [bx, by]) {
  const dx = bx - ax;
  const dy = by - ay;
  const lengthSq = dx * dx + dy * dy;
  const t = lengthSq ? Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / lengthSq)) : 0;
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}
