// ROS 2: health of the arm nodes and a read-only graph explorer.
//
//   #/ros          health      arm_driver / temperature_monitor / joystick_teleop, /diagnostics
//   #/ros/graph    graph       nodes, topics (live echo), services, actions

import { esc, fmtTime, live, on, send } from "../util.js";

const TABS = [["", "Health"], ["graph", "Graph"]];
const LEVEL_PILL = { ok: "ok", warn: "warn", error: "hot", stale: "", unknown: "" };

let state = null;   // { root, tab, watched, filter, internal, offs }

function unavailable(data) {
  return `<div class="card error-box">ROS 2 is not available here: ${esc(data?.error || "no data yet")}.
    <div class="small" style="margin-top:6px">The page runs inside the system container: make up (the real arm on the Jetson, the simulation on the PC).</div></div>`;
}

// ---- Health -----------------------------------------------------------------

function healthHTML(h) {
  if (!h) return '<div class="card empty">Waiting for the first health report…</div>';
  if (!h.available) return unavailable(h);
  const up = h.nodes.filter((n) => n.present).length;
  return `
    <div class="grid cols-2">
      <section class="card">
        <h2>Arm nodes <span class="pill ${up === h.nodes.length ? "ok" : "hot"}">${up}/${h.nodes.length} up</span></h2>
        <table><tbody>${h.nodes.map((n) => `
          <tr><td><span class="state-dot ${n.present ? "ok" : "bad"}"></span><b class="mono">${esc(n.name)}</b></td>
          <td class="muted small">${esc(n.role)}</td>
          <td>${n.present ? '<span class="pill ok">running</span>' : '<span class="pill hot">missing</span>'}</td></tr>`).join("")}
        </tbody></table>
        <p class="muted small">From ros2_ws/src/braco_bringup/launch/braco.launch.py. A missing node: <code>make logs</code> on the Jetson.</p>
      </section>
      <section class="card">
        <h2>arm_driver services</h2>
        <table><tbody>${h.services.map((s) => `
          <tr><td class="mono">${esc(s.name)}</td>
          <td>${s.present ? '<span class="pill ok">available</span>' : '<span class="pill hot">missing</span>'}</td></tr>`).join("")}
        </tbody></table>
      </section>
      <section class="card span-2">
        <h2>/diagnostics <span class="muted small" style="text-transform:none;letter-spacing:0">published by temperature_monitor</span></h2>
        ${h.diagnostics.length ? `<div class="table-wrap"><table>
          <thead><tr><th>Name</th><th>Level</th><th>Message</th><th>Values</th><th>Received</th></tr></thead>
          <tbody>${h.diagnostics.map((d) => `
            <tr><td class="mono">${esc(d.name)}</td>
            <td><span class="pill ${LEVEL_PILL[d.level] ?? ""}">${esc(d.level)}</span></td>
            <td>${esc(d.message)}</td>
            <td class="mono small">${d.values.map(([k, v]) => `${esc(k)}=${esc(v)}`).join("<br>")}</td>
            <td class="mono small">${fmtTime(d.t)}</td></tr>`).join("")}
          </tbody></table></div>` : '<div class="empty small">No /diagnostics message received yet.</div>'}
      </section>
    </div>`;
}

// ---- Graph ------------------------------------------------------------------

function match(name) {
  const q = state.filter.trim().toLowerCase();
  return !q || name.toLowerCase().includes(q);
}

function graphHTML(g) {
  if (!g) return '<div class="card empty">Reading the ROS graph…</div>';
  if (!g.available) return unavailable(g);
  const topics = g.topics.filter((t) => match(t.name));
  const services = g.services.filter((s) => (state.internal || !s.internal) && match(s.name));
  const actions = g.actions.filter((a) => match(a.name));
  const nodes = g.nodes.filter(match);
  return `
    <div class="stats">
      <div class="stat"><b>${g.nodes.length}</b><span>nodes</span></div>
      <div class="stat"><b>${g.topics.length}</b><span>topics</span></div>
      <div class="stat"><b>${g.services.filter((s) => !s.internal).length}</b><span>services (+${g.services.filter((s) => s.internal).length} internal)</span></div>
      <div class="stat"><b>${g.actions.length}</b><span>actions</span></div>
    </div>
    <div class="graph-layout">
      <div class="grid">
        <section class="card">
          <h2>Topics <span class="muted small" style="text-transform:none;letter-spacing:0">click one to watch it live</span></h2>
          <div class="table-wrap"><table>
            <thead><tr><th>Topic</th><th>Type</th><th>Pub</th><th>Sub</th><th></th></tr></thead>
            <tbody>${topics.map((t) => `
              <tr class="${state.watched === t.name ? "selected" : ""}">
                <td class="mono">${esc(t.name)}</td><td class="mono small">${esc(t.types.join(", "))}</td>
                <td class="mono">${t.publishers}</td><td class="mono">${t.subscribers}</td>
                <td><button class="small" data-watch="${esc(t.name)}">${state.watched === t.name ? "Watching" : "Echo"}</button></td>
              </tr>`).join("") || '<tr><td colspan="5" class="muted">No topic matches.</td></tr>'}
            </tbody></table></div>
        </section>
        <section class="card">
          <h2>Services <label class="arm-switch"><input type="checkbox" id="g-internal" ${state.internal ? "checked" : ""}> <span>show parameter / internal</span></label></h2>
          <div class="table-wrap"><table>
            <thead><tr><th>Service</th><th>Type</th></tr></thead>
            <tbody>${services.map((s) => `<tr><td class="mono">${esc(s.name)}</td><td class="mono small">${esc(s.types.join(", "))}</td></tr>`).join("")
              || '<tr><td colspan="2" class="muted">No service matches.</td></tr>'}</tbody></table></div>
        </section>
        <div class="grid cols-2">
          <section class="card">
            <h2>Nodes</h2>
            <ul class="plain mono small">${nodes.map((n) => `<li>${esc(n)}</li>`).join("") || '<li class="muted">none</li>'}</ul>
          </section>
          <section class="card">
            <h2>Actions</h2>
            <ul class="plain mono small">${actions.map((a) => `<li>${esc(a.name)} <span class="muted">${esc(a.types.join(", "))}</span></li>`).join("") || '<li class="muted">none</li>'}</ul>
          </section>
        </div>
      </div>
      <section class="card echo" id="g-echo">${echoHTML(state.echo?.topic === state.watched ? state.echo : null)}</section>
    </div>`;
}

function echoHTML(e) {
  if (!state.watched) return '<h2>Echo</h2><div class="empty small">Pick a topic to see its last message and rate. Read only: nothing is published.</div>';
  if (!e) return `<h2>Echo <span class="mono small">${esc(state.watched)}</span></h2><div class="empty small">Subscribing…</div>`;
  if (e.error) return `<h2>Echo <span class="mono small">${esc(state.watched)}</span></h2><div class="error-box">${esc(e.error)}</div>`;
  const body = e.message == null ? "no message yet"
    : typeof e.message === "string" ? e.message : JSON.stringify(e.message, null, 2);
  return `
    <h2>Echo <button class="small" id="g-unwatch">Stop</button></h2>
    <div class="mono small"><b>${esc(e.topic)}</b></div>
    <div class="muted small mono">${esc(e.type)}</div>
    <div class="echo-meta">
      <span class="pill">${e.hz == null ? "— Hz" : `${e.hz} Hz`}</span>
      <span class="pill">${e.count} msgs</span>
      <span class="pill">${e.last_t ? fmtTime(e.last_t) : "—"}</span>
    </div>
    <pre class="echo-msg">${esc(body)}</pre>`;
}

function watch(topic) {
  if (state.watched) send({ type: "unwatch", topic: state.watched });
  state.watched = topic;
  if (topic) send({ type: "watch", topic });
  draw();
}

// ---- Page -------------------------------------------------------------------

function draw() {
  const { root, tab } = state;
  root.querySelectorAll("#r-tabs a").forEach((a) => a.classList.toggle("active", a.dataset.tab === tab));
  root.querySelector("#r-search").hidden = tab !== "graph";
  const body = root.querySelector("#r-body");
  // Keep focus and scroll while the tables refresh
  const y = window.scrollY;
  if (tab === "graph") body.innerHTML = graphHTML(live.graph);
  else body.innerHTML = healthHTML(live.health);
  window.scrollTo(0, y);
}

function drawEcho(e) {
  if (e.topic !== state.watched) return;
  state.echo = e;
  if (state.tab !== "graph") return;
  const box = state.root.querySelector("#g-echo");
  if (box) box.innerHTML = echoHTML(e);
}

function bind(root) {
  root.addEventListener("click", (e) => {
    const w = e.target.closest("[data-watch]");
    if (w) watch(w.dataset.watch === state.watched ? null : w.dataset.watch);
    if (e.target.id === "g-unwatch") watch(null);
  });
  root.addEventListener("change", (e) => {
    if (e.target.id === "g-internal") { state.internal = e.target.checked; draw(); }
  });
  root.querySelector("#r-search").addEventListener("input", (e) => { state.filter = e.target.value; draw(); });
}

// Redraw a tab only while the user is not typing in one of its inputs
function redrawUnlessEditing(tab) {
  if (state?.tab !== tab) return;
  const el = document.activeElement;
  if (el && state.root.contains(el) && el.matches("#r-body input")) return;
  draw();
}

export function render(root, param) {
  root.innerHTML = `
    <div class="page-head">
      <div><h1>ROS 2</h1><p>The arm's ROS 2 graph, live: node health, every topic and service</p></div>
      <div class="filters">
        <input type="search" id="r-search" class="search" placeholder="Filter names…" autocomplete="off" hidden>
        <nav class="tabs" id="r-tabs">${TABS.map(([k, label]) => `<a href="#/ros${k ? "/" + k : ""}" data-tab="${k}">${label}</a>`).join("")}</nav>
      </div>
    </div>
    <div id="r-body"></div>`;
  state = { root, tab: TABS.some(([k]) => k === param) ? param : "", watched: null, filter: "", internal: false, offs: [] };
  bind(root);
  state.offs.push(on("health", () => redrawUnlessEditing("")));
  state.offs.push(on("graph", () => redrawUnlessEditing("graph")));
  state.offs.push(on("echo", drawEcho));
  draw();
  return () => {
    if (state.watched) send({ type: "unwatch", topic: state.watched });
    state.offs.forEach((off) => off());
    state = null;
  };
}

export function update(param) {
  if (!state) return;
  state.tab = TABS.some(([k]) => k === param) ? param : "";
  draw();
}
