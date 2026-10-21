// Helpers and the one live connection shared by every view.
//
// The server pushes typed messages over /ws (see web/server/app.py):
//   telemetry, presence, lease, activity, ui, speed, health, graph, echo, welcome
// Views subscribe with on(type, fn); the last value of each type is kept in
// `live` and replayed to new subscribers, so a view opened later is up to date.

export function esc(value) {
  return String(value ?? "").replace(/[&<>"']/g, (c) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  })[c]);
}

export async function getJSON(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
  return res.json();
}

export function fmt(value, digits = 1) {
  return value == null || Number.isNaN(value) ? "—" : Number(value).toFixed(digits);
}

export function fmtDate(iso, opts = { day: "2-digit", month: "short", year: "numeric" }) {
  return iso ? new Date(iso).toLocaleDateString("en-GB", opts) : "—";
}

export function fmtTime(seconds) {
  return seconds ? new Date(seconds * 1000).toLocaleTimeString("en-GB") : "—";
}

// ---- Identity: a name per browser, an id per tab ------------------------------

function stored(storage, key) {
  try { return storage.getItem(key); } catch (_) { return null; }
}
function store(storage, key, value) {
  try { storage.setItem(key, value); } catch (_) { /* storage blocked */ }
}

// crypto.randomUUID needs a secure context, and the Jetson is served over plain http
function newId() {
  return Date.now().toString(36) + Math.random().toString(36).slice(2, 10);
}

export const me = {
  id: stored(sessionStorage, "braco-client-id") || newId(),
  name: stored(localStorage, "braco-name") || "",
};
store(sessionStorage, "braco-client-id", me.id);

export function setName(name) {
  me.name = name.trim().slice(0, 32);
  store(localStorage, "braco-name", me.name);
  send({ type: "rename", name: me.name });
}

// ---- Live connection --------------------------------------------------------

const handlers = new Map();
export const live = {
  online: false, telemetry: null, lease: null, presence: [], activity: [],
  ui: null, speed: null, health: null, graph: null,
};
const REPLAY = ["telemetry", "lease", "presence", "health", "graph", "ui", "speed"];

export function on(type, fn) {
  if (!handlers.has(type)) handlers.set(type, new Set());
  handlers.get(type).add(fn);
  if (REPLAY.includes(type) && live[type] != null) fn(live[type]);
  return () => handlers.get(type).delete(fn);
}

function emit(type, value) {
  handlers.get(type)?.forEach((fn) => {
    try { fn(value); } catch (e) { console.error(e); }
  });
}

// Kept for the views written before the shared connection
export function onTelemetry(fn) {
  return on("telemetry", fn);
}

let socket = null;
let page = "";

export function send(message) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

export function setPage(name) {
  page = name;
  send({ type: "page", page });
}

export function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${proto}://${location.host}/ws`);
  socket.onopen = () => {
    socket.send(JSON.stringify({ type: "hello", id: me.id, name: me.name, page }));
    live.online = true;
    emit("online", true);
  };
  socket.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    switch (msg.type) {
      case "telemetry":
      case "health":
      case "graph":
        live[msg.type] = msg.data;
        emit(msg.type, msg.data);
        break;
      case "presence":
        live.presence = msg.clients;
        emit("presence", msg.clients);
        break;
      case "lease":
        live.lease = msg;
        emit("lease", msg);
        break;
      case "activity":
        if (msg.entries) {
          live.activity = msg.entries;
          emit("activity", { all: msg.entries });
        } else {
          live.activity.push(msg.entry);
          if (live.activity.length > 200) live.activity.shift();
          emit("activity", { entry: msg.entry });
        }
        break;
      case "ui":
        live.ui = msg.state ? msg : null;
        emit("ui", live.ui);
        break;
      case "speed":
        live.speed = msg;
        emit("speed", msg);
        break;
      default:
        emit(msg.type, msg);
    }
  };
  socket.onclose = () => {
    live.online = false;
    emit("online", false);
    setTimeout(connect, 2000);
  };
}

export function isOperator() {
  return live.lease?.holder?.id === me.id;
}

export function holderName() {
  return live.lease?.holder?.name || null;
}

// ---- Commands -----------------------------------------------------------------

// POST with this tab's id. Resolves to { ok, data, error } and never throws.
export async function post(url, body) {
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json", "X-Client-Id": me.id },
      body: body ? JSON.stringify(body) : undefined,
    });
    const data = await res.json().catch(() => ({}));
    if (res.ok) return { ok: true, data };
    const detail = typeof data.detail === "string" ? data.detail
      : Array.isArray(data.detail) ? data.detail.map((d) => d.msg).join("; ") : res.statusText;
    const prefix = res.status === 423 ? "Locked: " : res.status === 409 ? "Busy: " : "";
    return { ok: false, status: res.status, error: prefix + detail };
  } catch (e) {
    return { ok: false, error: e.message };
  }
}

export async function takeControl() {
  const r = await post("/api/control/take");
  if (!r.ok) toast(r.error, "hot");
  return r.ok;
}

export async function releaseControl() {
  return (await post("/api/control/release")).ok;
}

// Throttled sender for the operator's control panel, mirrored to the other pages.
// Leading edge: a change goes out at once; a burst (slider drag) is collapsed to one
// message per UI_EVERY ms, and the trailing timer sends the last value.
const UI_EVERY = 80;
let uiTimer = null;
let uiPending = null;
let uiLast = 0;
export function shareUi(state) {
  if (!isOperator()) return;
  uiPending = state;
  const wait = UI_EVERY - (Date.now() - uiLast);
  if (wait <= 0 && !uiTimer) {
    uiLast = Date.now();
    send({ type: "ui", state });
    return;
  }
  if (uiTimer) return;
  uiTimer = setTimeout(() => {
    uiTimer = null;
    uiLast = Date.now();
    send({ type: "ui", state: uiPending });
  }, Math.max(wait, 0));
}

// ---- Toasts -------------------------------------------------------------------

export function toast(text, kind = "") {
  const box = document.getElementById("toasts");
  if (!box) return;
  const el = document.createElement("div");
  el.className = `toast ${kind}`;
  el.textContent = text;
  box.appendChild(el);
  setTimeout(() => el.classList.add("out"), 4500);
  setTimeout(() => el.remove(), 5000);
  while (box.children.length > 4) box.firstChild.remove();
}
