// Helpers and the shared telemetry stream used by every view.

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
  return iso ? new Date(iso).toLocaleDateString("pt-BR", opts) : "—";
}

// ---- Telemetry: one WebSocket for the whole app ----------------------------

const listeners = new Set();
export const telemetry = { last: null, online: false };

export function onTelemetry(fn) {
  listeners.add(fn);
  if (telemetry.last) fn(telemetry.last);
  return () => listeners.delete(fn);
}

export function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws/telemetry`);
  ws.onopen = () => { telemetry.online = true; };
  ws.onmessage = (ev) => {
    telemetry.last = JSON.parse(ev.data);
    updateConn(telemetry.last);
    listeners.forEach((fn) => fn(telemetry.last));
  };
  ws.onclose = () => {
    telemetry.online = false;
    updateConn(null);
    setTimeout(connect, 2000);
  };
}

function updateConn(s) {
  const dot = document.querySelector("#conn .dot");
  const text = document.querySelector("#conn .conn-text");
  dot.className = "dot";
  if (!s) {
    dot.classList.add("bad");
    text.textContent = "servidor offline";
  } else if (!s.connected) {
    dot.classList.add("bad");
    text.textContent = "braço desconectado";
  } else if (s.source === "gazebo") {
    dot.classList.add("mock");
    text.textContent = "simulação Gazebo";
  } else if (s.mock) {
    dot.classList.add("mock");
    text.textContent = "modo simulado";
  } else {
    dot.classList.add("ok");
    text.textContent = "braço conectado";
  }
}

