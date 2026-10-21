// app.js – Sauna Boat route planner: map editor plus USB sync with the
// handheld controller's route slots.
//
// The planner mirrors the controller's fixed route slots. Each slot holds the
// working copy being edited (`route`) and the last copy read from or written
// to the controller (`device`, null until the controller has been seen), so
// unsaved edits survive reloads and reconnects. Both are kept in localStorage.

import { Controller, WebSerialTransport } from './controller.js';
import { SimTransport } from './sim.js';
import { parseGpx, routeToGpx } from './gpx.js';

const SIM = new URLSearchParams(location.search).has('sim');
const STORAGE_KEY = SIM ? 'saunaBoatRoutes.sim' : 'saunaBoatRoutes';
const VIEW_KEY = 'saunaBoatRoutes.view';
const DEFAULT_LIMITS = { routes: 10, waypoints: 50, radiusMin: 1, radiusMax: 100, nameLength: 15 };
const DEFAULT_RADIUS_M = 5;  // ROUTE_DEFAULT_RADIUS_M in shared/mesh_protocol.h
const POSITION_POLL_MS = 2000;
const UNDO_LIMIT = 100;
const ROUTE_COLOR = '#00b4d8';
const OTHER_ROUTE_COLOR = '#5f7a94';

const $ = (id) => document.getElementById(id);

const state = {
  limits: { ...DEFAULT_LIMITS },
  slots: [],
  activeRoute: null,   // controller's active route slot, once known
  selected: 0,
  selectedWp: null,
  controller: null,
  connecting: false,
  busyText: null,      // shown in the status pill while syncing
  boatCentered: false,
};
const undoStack = [];

// ---------------------------------------------------------------------------
// Route model helpers. Waypoints are { latE7, lonE7, radius } – the same
// fixed-point form the firmware stores, so comparisons are exact.
// ---------------------------------------------------------------------------
const defaultName = (i) => `Route ${i + 1}`;
const emptyRoute = (i) => ({ name: defaultName(i), waypoints: [] });
const cloneRoute = (r) => ({ name: r.name, waypoints: r.waypoints.map((wp) => ({ ...wp })) });
const isEmpty = (route, i) => route.waypoints.length === 0 && route.name === defaultName(i);
const currentSlot = () => state.slots[state.selected];
const currentRoute = () => currentSlot().route;

function sameRoute(a, b) {
  return a.name === b.name && a.waypoints.length === b.waypoints.length &&
    a.waypoints.every((wp, i) => {
      const other = b.waypoints[i];
      return wp.latE7 === other.latE7 && wp.lonE7 === other.lonE7 && wp.radius === other.radius;
    });
}

// Mirrors sanitizeRouteName() in the firmware so local and stored names match.
// Accents are stripped first so "Café" becomes "Cafe" rather than "Caf ".
function normalizeName(name, i) {
  const clean = name.normalize('NFD').replace(/[̀-ͯ]/g, '')
    .replace(/[^\x20-\x7e]/g, ' ').slice(0, state.limits.nameLength).trimEnd();
  return clean || defaultName(i);
}

function clampRadius(radius) {
  const r = Math.round(Number.isFinite(radius) ? radius : DEFAULT_RADIUS_M);
  return Math.min(state.limits.radiusMax, Math.max(state.limits.radiusMin, r));
}

// 'synced' | 'changed' (differs from controller) | 'local' (controller never
// seen) | 'empty'
function slotStatus(i) {
  const slot = state.slots[i];
  if (slot.device) return sameRoute(slot.route, slot.device) ? 'synced' : 'changed';
  return isEmpty(slot.route, i) ? 'empty' : 'local';
}

function slotInUse(i) {
  const slot = state.slots[i];
  return !isEmpty(slot.route, i) || (slot.device && !isEmpty(slot.device, i));
}

// ---------------------------------------------------------------------------
// Geometry & formatting
// ---------------------------------------------------------------------------
const toLatLng = (wp) => L.latLng(wp.latE7 / 1e7, wp.lonE7 / 1e7);
const midpoint = (a, b) => L.latLng((a.lat + b.lat) / 2, (a.lng + b.lng) / 2);

function toWaypoint(latlng, radius) {
  const p = latlng.wrap();
  return { latE7: Math.round(p.lat * 1e7), lonE7: Math.round(p.lng * 1e7), radius };
}

function routeLength(route) {
  let total = 0;
  for (let i = 1; i < route.waypoints.length; i++) {
    total += toLatLng(route.waypoints[i - 1]).distanceTo(toLatLng(route.waypoints[i]));
  }
  return total;
}

function formatDistance(m) {
  if (m < 1000) return `${Math.round(m)} m`;
  return `${(m / 1000).toFixed(m < 10000 ? 2 : 1)} km`;
}

const formatNm = (m) => `${(m / 1852).toFixed(2)} nm`;
const plural = (n, word) => `${n} ${word}${n === 1 ? '' : 's'}`;

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------
function isValidRoute(r) {
  return r && typeof r.name === 'string' && Array.isArray(r.waypoints) &&
    r.waypoints.every((wp) => wp && Number.isInteger(wp.latE7) && Number.isInteger(wp.lonE7) && Number.isInteger(wp.radius));
}

function load() {
  let saved = null;
  try {
    saved = JSON.parse(localStorage.getItem(STORAGE_KEY));
  } catch {
    // Unreadable or blocked storage: start fresh.
  }
  if (saved?.limits && Object.keys(DEFAULT_LIMITS).every((k) => Number.isInteger(saved.limits[k]) && saved.limits[k] > 0)) {
    state.limits = { ...saved.limits };
  }
  state.slots = Array.from({ length: state.limits.routes }, (_, i) => {
    const s = saved?.slots?.[i];
    return {
      route: isValidRoute(s?.route) ? cloneRoute(s.route) : emptyRoute(i),
      device: isValidRoute(s?.device) ? cloneRoute(s.device) : null,
    };
  });
  if (Number.isInteger(saved?.activeRoute)) state.activeRoute = saved.activeRoute;
  if (Number.isInteger(saved?.selected) && saved.selected < state.limits.routes) state.selected = saved.selected;
}

function save() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({
      version: 1,
      limits: state.limits,
      slots: state.slots,
      activeRoute: state.activeRoute,
      selected: state.selected,
    }));
  } catch {
    // Storage full or disabled; the session still works.
  }
}

function loadView() {
  try {
    return JSON.parse(localStorage.getItem(VIEW_KEY)) || {};
  } catch {
    return {};
  }
}

function saveView() {
  const center = map.getCenter();
  const view = {
    center: [center.lat, center.lng],
    zoom: map.getZoom(),
    base: map.hasLayer(layers.satellite) ? 'Satellite' : 'Map',
    seamarks: map.hasLayer(layers.seamarks),
  };
  try {
    localStorage.setItem(VIEW_KEY, JSON.stringify(view));
  } catch {
    // Ignore; view memory is a convenience.
  }
}

// ---------------------------------------------------------------------------
// Editing (every edit to the selected route goes through edit() for undo)
// ---------------------------------------------------------------------------
function pushUndo() {
  undoStack.push({ slot: state.selected, route: cloneRoute(currentRoute()), selectedWp: state.selectedWp });
  if (undoStack.length > UNDO_LIMIT) undoStack.shift();
}

function edit(fn, { selectWp } = {}) {
  pushUndo();
  fn(currentRoute());
  if (selectWp !== undefined) state.selectedWp = selectWp;
  changed();
}

function undo() {
  const entry = undoStack.pop();
  if (!entry) return;
  state.slots[entry.slot].route = entry.route;
  state.selected = entry.slot;
  state.selectedWp = entry.selectedWp;
  changed();
}

function changed() {
  const count = currentRoute().waypoints.length;
  if (state.selectedWp !== null && state.selectedWp >= count) state.selectedWp = count ? count - 1 : null;
  save();
  scheduleRender();
}

function addWaypoint(index, latlng) {
  const route = currentRoute();
  if (route.waypoints.length >= state.limits.waypoints) {
    toast(`A route can hold at most ${state.limits.waypoints} waypoints.`, 'error');
    return;
  }
  // New points inherit the arrival radius of their neighbour.
  const neighbour = route.waypoints[index - 1] ?? route.waypoints[index];
  const wp = toWaypoint(latlng, neighbour?.radius ?? DEFAULT_RADIUS_M);
  edit((r) => r.waypoints.splice(index, 0, wp), { selectWp: index });
}

function deleteWaypoint(index) {
  edit((r) => r.waypoints.splice(index, 1), { selectWp: null });
}

function selectRoute(i, { fit = true } = {}) {
  if (i === state.selected) return;
  state.selected = i;
  state.selectedWp = null;
  save();
  scheduleRender();
  if (fit) fitRoute({ quiet: true });
}

function selectWaypoint(index, { pan = false, redrawMap = false } = {}) {
  state.selectedWp = index;
  scheduleRender({ map: redrawMap });
  updateMarkerSelection();
  if (pan && index !== null) {
    const latlng = toLatLng(currentRoute().waypoints[index]);
    if (!map.getBounds().pad(-0.1).contains(latlng)) map.panTo(latlng);
  }
}

function newRoute() {
  const free = state.slots.findIndex((_, i) => !slotInUse(i));
  if (free < 0) {
    toast(`All ${state.limits.routes} route slots are in use. Delete one first.`, 'error');
    return;
  }
  selectRoute(free, { fit: false });
  requestAnimationFrame(() => {
    $('route-name').focus();
    $('route-name').select();
  });
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
let pendingRender = null;

function scheduleRender({ map: redrawMap = true } = {}) {
  if (!pendingRender) {
    pendingRender = { map: false };
    requestAnimationFrame(() => {
      const { map: drawMap } = pendingRender;
      pendingRender = null;
      render({ map: drawMap });
    });
  }
  pendingRender.map ||= redrawMap;
}

function render({ map: drawMap = true } = {}) {
  renderConnection();
  renderRouteList();
  renderEditor();
  if (drawMap) renderMap();
}

function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (value === false || value == null) continue;
    if (key === 'class') node.className = value;
    else if (key.startsWith('on')) node.addEventListener(key.slice(2), value);
    else node.setAttribute(key, value === true ? '' : value);
  }
  node.append(...children.filter((c) => c != null && c !== false));
  return node;
}

function icon(name, className = 'icon') {
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('class', className);
  const use = document.createElementNS('http://www.w3.org/2000/svg', 'use');
  use.setAttribute('href', `#${name}`);
  svg.append(use);
  return svg;
}

function statusChip(status) {
  if (status === 'changed') return el('span', { class: 'chip warn' }, 'Unsaved');
  if (status === 'local') return el('span', { class: 'chip warn', title: 'Connect the controller to upload' }, 'Not uploaded');
  return null;
}

function renderConnection() {
  const connected = Boolean(state.controller);
  const status = $('conn-status');
  status.classList.toggle('connected', connected && !state.busyText);
  status.classList.toggle('connecting', state.connecting || Boolean(state.busyText));
  $('conn-text').textContent = state.busyText ||
    (state.connecting ? 'Connecting…' : connected ? 'Controller connected' : 'Not connected');

  const button = $('connect-btn');
  button.querySelector('span').textContent = connected ? 'Disconnect' : 'Connect controller';
  button.classList.toggle('primary', !connected);
  button.disabled = state.connecting || Boolean(state.busyText) || (!SIM && !WebSerialTransport.supported);
}

function renderRouteList() {
  const list = $('route-list');
  list.replaceChildren();
  let used = 0;
  state.slots.forEach((slot, i) => {
    const inUse = slotInUse(i);
    if (inUse) used++;
    if (!inUse && i !== state.selected) return;

    const route = slot.route;
    const meta = route.waypoints.length
      ? `${plural(route.waypoints.length, 'waypoint')} · ${formatDistance(routeLength(route))}`
      : 'No waypoints yet';
    const flags = el('span', { class: 'route-flags' },
      statusChip(slotStatus(i)),
      state.activeRoute === i ? el('span', { title: 'Active route on the controller' }, icon('i-star', 'icon star')) : null);

    list.append(el('li', { class: `route-item${i === state.selected ? ' selected' : ''}` },
      el('button', { class: 'route-btn', type: 'button', onclick: () => selectRoute(i) },
        el('span', { class: 'slot-badge' }, String(i + 1)),
        el('span', { class: 'route-main' },
          el('span', { class: 'route-name' }, route.name),
          el('span', { class: 'route-meta' }, meta)),
        flags)));
  });

  $('slots-note').textContent = `${used} of ${state.limits.routes} route slots in use. The number is the slot on the controller.`;
  const changedCount = state.slots.filter((_, i) => slotStatus(i) === 'changed').length;
  const uploadAll = $('upload-all-btn');
  uploadAll.hidden = !state.controller || changedCount < 2;
  uploadAll.disabled = Boolean(state.busyText);
}

function renderEditor() {
  const i = state.selected;
  const slot = currentSlot();
  const route = slot.route;
  const status = slotStatus(i);
  const connected = Boolean(state.controller);
  const idle = !state.busyText && !state.connecting;
  const count = route.waypoints.length;
  const max = state.limits.waypoints;

  $('editor-slot').textContent = String(i + 1);
  const nameInput = $('route-name');
  nameInput.maxLength = state.limits.nameLength;
  nameInput.placeholder = defaultName(i);
  if (document.activeElement !== nameInput) nameInput.value = route.name;

  const length = routeLength(route);
  $('route-stats').textContent = `${count} / ${max} waypoints` +
    (count > 1 ? ` · ${formatDistance(length)} (${formatNm(length)})` : '');
  $('editor-badges').replaceChildren(...[
    state.activeRoute === i ? el('span', { class: 'chip active' }, icon('i-star'), 'Active') : null,
    status === 'synced' ? el('span', { class: 'chip ok' }, icon('i-check'), 'On controller') : statusChip(status),
  ].filter(Boolean));

  const upload = $('upload-btn');
  upload.disabled = !connected || !idle || status === 'synced';
  upload.title = connected ? '' : 'Connect the controller to upload';
  upload.querySelector('span').textContent = status === 'synced' ? 'Saved on controller' : 'Upload to controller';

  const active = $('active-btn');
  active.disabled = !connected || !idle || state.activeRoute === i;
  active.querySelector('span').textContent = state.activeRoute === i ? 'Active route' : 'Make active';

  $('revert-btn').hidden = !(slot.device && status === 'changed');
  $('revert-btn').disabled = !idle;

  $('undo-btn').disabled = undoStack.length === 0;
  $('reverse-btn').disabled = count < 2;
  $('export-btn').disabled = count === 0;
  $('delete-btn').disabled = isEmpty(route, i);

  $('editor-hint').textContent = count === 0
    ? 'Click the map to place the first waypoint. Search for a place above, or use the locate button on the map.'
    : count >= max
      ? `This route is full (${max} waypoints).`
      : 'Click the map to add a waypoint. Drag points to move them, or click a + between two points to insert one.';

  renderWaypointList();
}

function renderWaypointList() {
  const list = $('wp-list');
  // Re-rendering replaces the inputs, so carry focus across by key.
  const focused = document.activeElement;
  const focusKey = list.contains(focused) ? focused.dataset.key : null;

  const route = currentRoute();
  const { radiusMin, radiusMax } = state.limits;
  list.replaceChildren(...route.waypoints.map((wp, idx) => {
    const selected = idx === state.selectedWp;
    const lat = wp.latE7 / 1e7;
    const lon = wp.lonE7 / 1e7;

    let position;
    if (selected) {
      const commitPosition = () => {
        const newLat = parseFloat(latInput.value);
        const newLon = parseFloat(lonInput.value);
        if (!(Math.abs(newLat) <= 90 && Math.abs(newLon) <= 180)) {
          toast('Latitude must be between -90 and 90, longitude between -180 and 180.', 'error');
          scheduleRender({ map: false });
          return;
        }
        const next = toWaypoint(L.latLng(newLat, newLon), wp.radius);
        if (next.latE7 === wp.latE7 && next.lonE7 === wp.lonE7) return;
        edit((r) => Object.assign(r.waypoints[idx], next));
      };
      const latInput = el('input', { type: 'text', inputmode: 'decimal', 'data-key': `lat-${idx}`, value: lat.toFixed(7), 'aria-label': 'Latitude', onchange: commitPosition });
      const lonInput = el('input', { type: 'text', inputmode: 'decimal', 'data-key': `lon-${idx}`, value: lon.toFixed(7), 'aria-label': 'Longitude', onchange: commitPosition });
      position = el('div', { class: 'wp-pos' }, latInput, lonInput);
    } else {
      position = el('div', { class: 'wp-pos' },
        el('span', { class: 'coord' }, `${Math.abs(lat).toFixed(5)}° ${lat >= 0 ? 'N' : 'S'}`),
        el('span', { class: 'coord' }, `${Math.abs(lon).toFixed(5)}° ${lon >= 0 ? 'E' : 'W'}`));
    }

    const leg = idx === 0 ? 'Start' : formatDistance(toLatLng(route.waypoints[idx - 1]).distanceTo(toLatLng(wp)));
    const radius = el('input', {
      type: 'number', min: radiusMin, max: radiusMax, step: 1, value: wp.radius,
      'data-key': `radius-${idx}`, 'aria-label': `Arrival radius for waypoint ${idx + 1}, metres`,
      oninput: (e) => {
        const value = Number(e.target.value);
        if (value >= radiusMin && value <= radiusMax) wpCircles[idx]?.setRadius(value);
      },
      onchange: (e) => {
        const value = clampRadius(Number(e.target.value));
        if (value === wp.radius) {
          e.target.value = value;
          wpCircles[idx]?.setRadius(value);
          return;
        }
        edit((r) => { r.waypoints[idx].radius = value; });
      },
    });

    return el('li', {
      class: `wp${selected ? ' selected' : ''}`,
      onclick: (e) => {
        if (!e.target.closest('.wp-delete') && state.selectedWp !== idx) selectWaypoint(idx, { pan: true });
      },
    },
      el('span', { class: `wp-num${idx === 0 ? ' start' : ''}` }, String(idx + 1)),
      position,
      el('span', { class: 'wp-leg', title: idx ? 'Distance from the previous waypoint' : null }, leg),
      el('label', { class: 'wp-radius', title: 'Arrival radius. The boat follows the line through the waypoints without needing to reach them, so only the last waypoint\'s radius is used: the route finishes inside it' }, radius, 'm'),
      el('button', {
        class: 'icon-btn wp-delete', type: 'button', title: 'Delete waypoint', 'data-key': `delete-${idx}`,
        onclick: () => deleteWaypoint(idx),
      }, icon('i-x')));
  }));

  if (focusKey) {
    const next = list.querySelector(`[data-key="${focusKey}"]`);
    next?.focus();
  }
  if (state.selectedWp !== null && !focusKey) {
    list.children[state.selectedWp]?.scrollIntoView({ block: 'nearest' });
  }
}

// ---------------------------------------------------------------------------
// Map
// ---------------------------------------------------------------------------
let map;
let routeLayer;
let otherLayer;
let boatMarker = null;
let routeLine = null;
let wpMarkers = [];
let wpCircles = [];
let midMarkers = [];
const layers = {};

const MID_ICON = L.divIcon({ className: 'mid-icon', html: '<div class="mid-pin">+</div>', iconSize: [18, 18], iconAnchor: [9, 9] });

function waypointIcon(i) {
  return L.divIcon({
    className: `wp-icon${i === 0 ? ' start' : ''}`,
    html: `<div class="wp-pin">${i + 1}</div>`,
    iconSize: [26, 26],
    iconAnchor: [13, 13],
  });
}

function initMap() {
  const view = loadView();
  map = L.map('map', { zoomControl: false, worldCopyJump: true, maxZoom: 19 });

  layers.map = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  });
  layers.satellite = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxZoom: 19,
    attribution: 'Imagery &copy; Esri, Maxar, Earthstar Geographics',
  });
  layers.seamarks = L.tileLayer('https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png', {
    maxZoom: 19,
    maxNativeZoom: 18,
    attribution: 'Sea marks &copy; <a href="https://www.openseamap.org">OpenSeaMap</a>',
  });

  (view.base === 'Satellite' ? layers.satellite : layers.map).addTo(map);
  if (view.seamarks !== false) layers.seamarks.addTo(map);

  L.control.zoom({ position: 'topright' }).addTo(map);
  L.control.layers({ Map: layers.map, Satellite: layers.satellite }, { 'Sea marks': layers.seamarks }, { position: 'topright' }).addTo(map);
  new (L.Control.extend({
    options: { position: 'topright' },
    onAdd() {
      const bar = L.DomUtil.create('div', 'leaflet-bar map-buttons');
      for (const [iconId, title, handler] of [
        ['i-fit', 'Zoom to route', () => fitRoute()],
        ['i-locate', 'Show my location', locateMe],
        ['i-boat', 'Zoom to the boat', zoomToBoat],
      ]) {
        const a = L.DomUtil.create('a', '', bar);
        a.href = '#';
        a.title = title;
        a.setAttribute('role', 'button');
        a.setAttribute('aria-label', title);
        a.append(icon(iconId));
        L.DomEvent.on(a, 'click', (e) => {
          L.DomEvent.stop(e);
          handler();
        });
      }
      L.DomEvent.disableClickPropagation(bar);
      return bar;
    },
  }))().addTo(map);
  L.control.scale({ imperial: false, position: 'bottomleft' }).addTo(map);

  otherLayer = L.layerGroup().addTo(map);
  routeLayer = L.layerGroup().addTo(map);

  if (Array.isArray(view.center) && Number.isFinite(view.zoom)) {
    map.setView(view.center, view.zoom);
  } else {
    const all = state.slots.flatMap((s) => s.route.waypoints.map(toLatLng));
    if (all.length) map.fitBounds(L.latLngBounds(all), { padding: [40, 40], maxZoom: 16 });
    else map.setView([30, 0], 2);
  }

  map.on('click', (e) => addWaypoint(currentRoute().waypoints.length, e.latlng));
  map.on('moveend baselayerchange overlayadd overlayremove', saveView);
  map.on('locationfound', (e) => {
    L.circleMarker(e.latlng, { radius: 7, color: '#fff', weight: 2, fillColor: '#3b82f6', fillOpacity: 1, interactive: false }).addTo(map);
  });
  map.on('locationerror', () => toast('Couldn’t get your location.', 'error'));
}

function renderMap() {
  otherLayer.clearLayers();
  routeLayer.clearLayers();
  wpMarkers = [];
  wpCircles = [];
  midMarkers = [];

  // Other routes: faint and clickable to switch to them.
  state.slots.forEach((slot, i) => {
    if (i === state.selected || slot.route.waypoints.length === 0) return;
    const pts = slot.route.waypoints.map(toLatLng);
    const style = { color: OTHER_ROUTE_COLOR, weight: 3, opacity: 0.8, bubblingMouseEvents: false };
    const shape = pts.length > 1
      ? L.polyline(pts, { ...style, dashArray: '6 6' })
      : L.circleMarker(pts[0], { ...style, radius: 6, fillOpacity: 0.6 });
    shape.bindTooltip(`${i + 1} · ${slot.route.name}`, { sticky: true })
      .on('click', () => selectRoute(i, { fit: false }))
      .addTo(otherLayer);
  });

  const route = currentRoute();
  const pts = route.waypoints.map(toLatLng);
  routeLine = L.polyline(pts, { color: ROUTE_COLOR, weight: 4, opacity: 0.9, interactive: false }).addTo(routeLayer);

  route.waypoints.forEach((wp, i) => {
    wpCircles.push(L.circle(pts[i], {
      radius: wp.radius, color: ROUTE_COLOR, weight: 1, opacity: 0.8,
      fillColor: ROUTE_COLOR, fillOpacity: 0.12, interactive: false,
    }).addTo(routeLayer));
  });

  for (let i = 0; i < pts.length - 1; i++) {
    const mid = L.marker(midpoint(pts[i], pts[i + 1]), { icon: MID_ICON, draggable: true, keyboard: false, title: 'Insert a waypoint here' });
    mid.on('click', () => addWaypoint(i + 1, mid.getLatLng()));
    mid.on('dragend', () => addWaypoint(i + 1, mid.getLatLng()));
    midMarkers.push(mid.addTo(routeLayer));
  }

  pts.forEach((p, i) => {
    const marker = L.marker(p, { icon: waypointIcon(i), draggable: true, autoPan: true, keyboard: false, zIndexOffset: 1000 });
    marker.on('click', () => selectWaypoint(i, { pan: false }));
    marker.on('dragstart', () => {
      pushUndo();
      selectWaypoint(i);
    });
    marker.on('drag', () => {
      pts[i] = marker.getLatLng();
      routeLine.setLatLngs(pts);
      wpCircles[i].setLatLng(pts[i]);
      if (i > 0) midMarkers[i - 1].setLatLng(midpoint(pts[i - 1], pts[i]));
      if (i < pts.length - 1) midMarkers[i].setLatLng(midpoint(pts[i], pts[i + 1]));
    });
    marker.on('dragend', () => {
      const wp = currentRoute().waypoints[i];
      Object.assign(wp, toWaypoint(marker.getLatLng(), wp.radius));
      changed();
    });
    wpMarkers.push(marker.addTo(routeLayer));
  });
  updateMarkerSelection();
}

function updateMarkerSelection() {
  wpMarkers.forEach((m, i) => m.getElement()?.classList.toggle('selected', i === state.selectedWp));
}

function fitRoute({ quiet = false } = {}) {
  const pts = currentRoute().waypoints.map(toLatLng);
  if (pts.length === 0) {
    if (!quiet) toast('This route has no waypoints yet.');
    return;
  }
  if (pts.length === 1) map.setView(pts[0], Math.max(map.getZoom(), 16));
  else map.fitBounds(L.latLngBounds(pts), { padding: [50, 50], maxZoom: 17 });
}

function locateMe() {
  map.locate({ setView: true, maxZoom: 16 });
}

function zoomToBoat() {
  if (boatMarker) map.setView(boatMarker.getLatLng(), Math.max(map.getZoom(), 16));
  else toast(state.controller ? 'The controller has no GPS fix from the boat yet.' : 'Connect the controller to see the boat’s position.');
}

function updateBoat(pos) {
  if (!pos || pos.fix === 0) {
    boatMarker?.remove();
    boatMarker = null;
    return;
  }
  const latlng = L.latLng(pos.latE7 / 1e7, pos.lonE7 / 1e7);
  if (!boatMarker) {
    boatMarker = L.marker(latlng, {
      icon: L.divIcon({ className: 'boat-icon', html: '<div class="boat-pin"><svg class="icon"><use href="#i-boat"/></svg></div>', iconSize: [30, 30], iconAnchor: [15, 15] }),
      keyboard: false,
      zIndexOffset: 2000,
    }).bindTooltip('Boat (live GPS)').addTo(map);
  } else {
    boatMarker.setLatLng(latlng);
  }
  const pin = boatMarker.getElement()?.querySelector('.boat-pin');
  if (pin) {
    const hasHeading = pos.heading >= 0;
    pin.classList.toggle('no-heading', !hasHeading);
    pin.style.transform = hasHeading ? `rotate(${pos.heading}deg)` : '';
  }
  if (!state.boatCentered && currentRoute().waypoints.length === 0) {
    map.setView(latlng, 16);
  }
  state.boatCentered = true;
}

// ---------------------------------------------------------------------------
// Controller connection & sync
// ---------------------------------------------------------------------------
let pollTimer = null;

function setBusy(text) {
  state.busyText = text;
  renderConnection();
}

async function toggleConnection() {
  if (state.controller) {
    const controller = state.controller;
    dropConnection();
    await controller.close().catch(() => {});
    toast('Disconnected.');
    return;
  }

  // Nothing may be awaited before requestPort(): it needs the click's user activation.
  const controller = new Controller(SIM ? new SimTransport() : new WebSerialTransport());
  controller.onLog = appendLog;
  controller.onDisconnect = () => {
    if (state.controller !== controller) return;
    dropConnection();
    controller.close().catch(() => {});
    toast('The controller was disconnected.', 'error');
  };

  state.connecting = true;
  render({ map: false });
  try {
    const info = await controller.connect();
    state.controller = controller;
    applyLimits(info);
    state.activeRoute = info.activeRoute;
    await readAllRoutes();
    fitRoute({ quiet: true });
    startPolling();
  } catch (err) {
    if (state.controller === controller) dropConnection();
    await controller.close().catch(() => {});
    if (err.name !== 'NotFoundError') toast(connectionErrorMessage(err), 'error');  // NotFoundError: picker cancelled
  } finally {
    state.connecting = false;
    state.busyText = null;
    scheduleRender();
  }
}

function connectionErrorMessage(err) {
  if (err.name === 'NetworkError' || /open/i.test(err.message)) {
    return 'Couldn’t open the serial port. Close any other program using it (such as a PlatformIO serial monitor) and try again.';
  }
  return err.message;
}

function dropConnection() {
  clearInterval(pollTimer);
  pollTimer = null;
  state.controller = null;
  state.busyText = null;
  state.boatCentered = false;
  updateBoat(null);
  scheduleRender();
}

function applyLimits(info) {
  state.limits = {
    routes: info.routes,
    waypoints: info.waypoints,
    radiusMin: info.radiusMin,
    radiusMax: info.radiusMax,
    nameLength: info.nameLength,
  };
  while (state.slots.length < info.routes) state.slots.push({ route: emptyRoute(state.slots.length), device: null });
  state.slots.length = info.routes;
  if (state.selected >= info.routes) state.selected = 0;
}

// Reads every slot. Slots without local edits take the controller's copy;
// slots with unsaved local edits keep them and show as "Unsaved".
async function readAllRoutes() {
  const kept = [];
  for (let i = 0; i < state.limits.routes; i++) {
    setBusy(`Reading routes ${i + 1}/${state.limits.routes}…`);
    const stored = await state.controller.getRoute(i);
    const slot = state.slots[i];
    const hasLocalEdits = slot.device ? !sameRoute(slot.route, slot.device) : !isEmpty(slot.route, i);
    slot.device = stored;
    if (!hasLocalEdits) slot.route = cloneRoute(stored);
    else if (!sameRoute(slot.route, stored)) kept.push(slot.route.name);
  }
  setBusy(null);
  save();
  const onController = state.slots.filter((s, i) => !isEmpty(s.device, i)).length;
  toast(kept.length
    ? `Connected. Kept your unsaved edits to ${kept.join(', ')} — upload or revert them.`
    : `Connected. Read ${plural(onController, 'route')} from the controller.`);
}

async function uploadSlot(i) {
  const route = cloneRoute(state.slots[i].route);
  setBusy(`Uploading ${route.name}…`);
  await state.controller.putRoute(i, route);
  const stored = await state.controller.getRoute(i);
  state.slots[i].device = stored;
  if (!sameRoute(stored, route)) throw new Error(`${route.name} read back differently from what was sent. Try again.`);
}

async function runSync(task) {
  try {
    await task();
  } catch (err) {
    toast(`Upload failed: ${err.message}`, 'error');
  } finally {
    setBusy(null);
    save();
    scheduleRender();
  }
}

function uploadSelected() {
  const i = state.selected;
  return runSync(async () => {
    await uploadSlot(i);
    toast(`Saved “${state.slots[i].device.name}” to slot ${i + 1} on the controller.`);
  });
}

function uploadAllChanged() {
  return runSync(async () => {
    const changedSlots = state.slots.map((_, i) => i).filter((i) => slotStatus(i) === 'changed');
    for (const i of changedSlots) await uploadSlot(i);
    toast(`Uploaded ${plural(changedSlots.length, 'route')} to the controller.`);
  });
}

function makeActive() {
  const i = state.selected;
  return runSync(async () => {
    setBusy('Updating active route…');
    await state.controller.setActive(i);
    state.activeRoute = i;
    toast(`“${currentRoute().name}” is now the active route. “Send + Start” on the controller will run it.`);
  });
}

function startPolling() {
  clearInterval(pollTimer);
  const poll = async () => {
    const controller = state.controller;
    if (!controller || controller.busy) return;
    try {
      const pos = await controller.position();
      if (state.controller === controller) updateBoat(pos);
    } catch {
      // A missed poll is harmless; real disconnects are reported separately.
    }
  };
  poll();
  pollTimer = setInterval(poll, POSITION_POLL_MS);
}

// ---------------------------------------------------------------------------
// Log, toast, search
// ---------------------------------------------------------------------------
const logLines = [];
let logRenderQueued = false;

function appendLog(line) {
  if (/^(> )?\$(POS|PING)\b|^\$OK,PING/.test(line)) return;  // hide position polling
  logLines.push(line);
  if (logLines.length > 400) logLines.shift();
  if (!$('log').open || logRenderQueued) return;
  logRenderQueued = true;
  requestAnimationFrame(renderLog);
}

function renderLog() {
  logRenderQueued = false;
  const out = $('log-output');
  if (!logLines.length) return;
  const atBottom = out.scrollTop + out.clientHeight >= out.scrollHeight - 20;
  out.textContent = logLines.join('\n');
  if (atBottom) out.scrollTop = out.scrollHeight;
}

let toastTimer = null;

function toast(message, kind = 'info', action = null) {
  const box = $('toast');
  box.className = `toast ${kind}`;
  $('toast-text').textContent = message;
  const button = $('toast-action');
  button.hidden = !action;
  if (action) {
    button.textContent = action.label;
    button.onclick = () => {
      action.run();
      box.hidden = true;
    };
  }
  box.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { box.hidden = true; }, kind === 'error' ? 9000 : 5000);
}

async function searchPlaces(query) {
  const results = $('search-results');
  const url = `https://nominatim.openstreetmap.org/search?format=jsonv2&limit=5&q=${encodeURIComponent(query)}`;
  let places;
  try {
    const res = await fetch(url, { headers: { 'Accept-Language': navigator.language || 'en' } });
    if (!res.ok) throw new Error(res.statusText);
    places = await res.json();
  } catch {
    toast('Place search is unavailable right now.', 'error');
    return;
  }
  const goTo = (place) => {
    const [south, north, west, east] = place.boundingbox.map(Number);
    map.fitBounds([[south, west], [north, east]], { maxZoom: 16 });
    results.hidden = true;
  };
  results.replaceChildren(...(places.length
    ? places.map((place) => el('li', {}, el('button', { type: 'button', onclick: () => goTo(place) }, place.display_name)))
    : [el('li', { class: 'empty' }, 'No places found.')]));
  results.hidden = false;
  if (places.length) {
    goTo(places[0]);
    results.hidden = places.length === 1;  // keep alternatives on screen
  }
}

// ---------------------------------------------------------------------------
// GPX
// ---------------------------------------------------------------------------
async function importGpx(file) {
  let parsed;
  try {
    parsed = parseGpx(await file.text(), state.limits.waypoints);
  } catch (err) {
    toast(err.message, 'error');
    return;
  }
  const i = state.selected;
  edit((r) => {
    if (parsed.name) r.name = normalizeName(parsed.name, i);
    r.waypoints = parsed.points.map((p) => ({
      latE7: Math.round(p.lat * 1e7),
      lonE7: Math.round(p.lon * 1e7),
      radius: clampRadius(p.radius ?? DEFAULT_RADIUS_M),
    }));
  }, { selectWp: null });
  requestAnimationFrame(() => fitRoute({ quiet: true }));
  const simplified = parsed.originalCount > parsed.points.length
    ? ` (simplified from ${parsed.originalCount} ${parsed.source === 'track' ? 'track points' : 'points'})`
    : '';
  toast(`Imported ${plural(parsed.points.length, 'waypoint')}${simplified}.`, 'info', { label: 'Undo', run: undo });
}

function exportGpx() {
  const route = currentRoute();
  const blob = new Blob([routeToGpx(route)], { type: 'application/gpx+xml' });
  const a = el('a', {
    href: URL.createObjectURL(blob),
    download: `${route.name.trim().replace(/[^a-z0-9-_]+/gi, '_') || 'route'}.gpx`,
  });
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------
function bindUi() {
  $('connect-btn').addEventListener('click', toggleConnection);
  $('new-route-btn').addEventListener('click', newRoute);
  $('upload-btn').addEventListener('click', uploadSelected);
  $('upload-all-btn').addEventListener('click', uploadAllChanged);
  $('active-btn').addEventListener('click', makeActive);
  $('revert-btn').addEventListener('click', () => {
    const stored = currentSlot().device;
    edit((r) => Object.assign(r, cloneRoute(stored)), { selectWp: null });
    toast('Reverted to the copy on the controller.', 'info', { label: 'Undo', run: undo });
  });
  $('undo-btn').addEventListener('click', undo);
  $('reverse-btn').addEventListener('click', () => edit((r) => r.waypoints.reverse(), { selectWp: null }));
  $('delete-btn').addEventListener('click', () => {
    const name = currentRoute().name;
    const i = state.selected;
    edit((r) => Object.assign(r, emptyRoute(i)), { selectWp: null });
    toast(`Deleted “${name}”.${state.controller ? ' Upload to remove it from the controller too.' : ''}`, 'info', { label: 'Undo', run: undo });
  });
  $('import-btn').addEventListener('click', () => $('import-input').click());
  $('import-input').addEventListener('change', (e) => {
    const file = e.target.files[0];
    e.target.value = '';
    if (file) importGpx(file);
  });
  $('export-btn').addEventListener('click', exportGpx);

  const nameInput = $('route-name');
  nameInput.addEventListener('change', () => {
    const name = normalizeName(nameInput.value, state.selected);
    nameInput.value = name;
    if (name !== currentRoute().name) edit((r) => { r.name = name; });
  });
  nameInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') nameInput.blur();
  });

  $('log').addEventListener('toggle', () => {
    if ($('log').open) renderLog();
  });

  const searchInput = $('search-input');
  const results = $('search-results');
  $('search-form').addEventListener('submit', (e) => {
    e.preventDefault();
    const query = searchInput.value.trim();
    if (query) searchPlaces(query);
  });
  document.addEventListener('click', (e) => {
    if (!e.target.closest('#search-form')) results.hidden = true;
  });

  document.addEventListener('keydown', (e) => {
    const typing = e.target.closest('input, textarea, select, [contenteditable="true"]');
    if (e.key === 'Escape') {
      results.hidden = true;
      if (!typing && state.selectedWp !== null) selectWaypoint(null);
      return;
    }
    if (typing) return;
    if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'z' && !e.shiftKey) {
      e.preventDefault();
      undo();
    } else if ((e.key === 'Delete' || e.key === 'Backspace') && state.selectedWp !== null) {
      e.preventDefault();
      deleteWaypoint(state.selectedWp);
    }
  });

  window.addEventListener('beforeunload', (e) => {
    if (state.busyText) e.preventDefault();
  });
}

function init() {
  load();
  initMap();
  bindUi();
  $('sim-chip').hidden = !SIM;
  $('serial-unsupported').hidden = SIM || WebSerialTransport.supported;
  render();
}

init();
