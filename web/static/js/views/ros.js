// ROS 2: health of the arm nodes, a read-only graph explorer, and the issue-18 demos.
//
//   #/ros          health      arm_driver / temperature_monitor / joystick_teleop, /diagnostics
//   #/ros/graph    graph       nodes, topics (live echo), services, actions
//   #/ros/demos    demos       chatter, AddTwoInts, Fibonacci action, lifecycle talkers

import { esc, fmtTime, isOperator, live, on, post, send, toast } from "../util.js";

const TABS = [["", "Health"], ["graph", "Graph"], ["demos", "Demos"]];
const LIFECYCLE_STATES = ["unconfigured", "inactive", "active", "finalized"];
const LEVEL_PILL = { ok: "ok", warn: "warn", error: "hot", stale: "", unknown: "" };

let state = null;   // { root, tab, watched, filter, internal, offs }

function unavailable(data) {
  return `<div class="card error-box">ROS 2 is not available here: ${esc(data?.error || "no data yet")}.
    <div class="small" style="margin-top:6px">On the Jetson the page runs inside the arm container (make arm-up); on the PC use make web-sim.</div></div>`;
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
        <p class="muted small">From ros2_ws/src/braco_bringup/launch/braco.launch.py. A missing node: <code>make arm-logs</code> on the Jetson.</p>
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

// ---- Demos ------------------------------------------------------------------

function demosHTML(d) {
  if (!d) return '<div class="card empty">Waiting for the demos…</div>';
  if (!d.available) return unavailable(d);
  const op = isOperator();
  const dis = op ? "" : "disabled";
  const fib = d.fibonacci;
  const running = fib.state === "running" || fib.state === "sending";
  const expected = (fib.order || 0) + 1;
  const done = (fib.result || fib.partial || []).length;
  const notRunning = (what) => `<div class="hint-box small">${what} is not running. On the Jetson: <code>make arm-basics</code> (docker/).</div>`;
  return `
    ${d.error ? `<div class="error-box" style="margin-bottom:12px">${esc(d.error)}</div>` : ""}
    ${op ? "" : '<div class="lease-banner taken" style="margin-bottom:16px"><span>👁 View only: take control (sidebar) to call services, send goals and change lifecycle states.</span></div>'}
    <div class="grid cols-2">
      <section class="card">
        <h2>/chatter <span class="muted small" style="text-transform:none;letter-spacing:0">talker_cpp → listener_py</span></h2>
        ${feedHTML(d.chatter, "No /chatter message yet.")}
      </section>
      <section class="card">
        <h2>/lifecycle_chatter <span class="muted small" style="text-transform:none;letter-spacing:0">only while a managed talker is active</span></h2>
        ${feedHTML(d.lifecycle_chatter, "Silent: no managed talker is active.")}
      </section>
      <section class="card">
        <h2>Service · /add_two_ints ${d.add_two_ints.present ? '<span class="pill ok">available</span>' : '<span class="pill hot">missing</span>'}</h2>
        ${d.add_two_ints.present ? "" : notRunning("add_two_ints_server")}
        <form class="inline-form" id="d-sum">
          <input type="number" name="a" value="2" step="1" ${dis}> <span>+</span>
          <input type="number" name="b" value="3" step="1" ${dis}>
          <button type="submit" ${dis}>Call</button>
        </form>
        ${d.add_two_ints.last ? `<p class="result"><b class="mono">${d.add_two_ints.last.a} + ${d.add_two_ints.last.b} = ${d.add_two_ints.last.sum}</b>
          <span class="muted small">by ${esc(d.add_two_ints.last.by)} · ${fmtTime(d.add_two_ints.last.t)}</span></p>` : ""}
        <p class="muted small">ros2_basics_interfaces/srv/AddTwoInts · request/response: the call blocks until the server answers.</p>
      </section>
      <section class="card">
        <h2>Action · /fibonacci ${fib.present ? '<span class="pill ok">available</span>' : '<span class="pill hot">missing</span>'}</h2>
        ${fib.present ? "" : notRunning("fibonacci_server")}
        <form class="inline-form" id="d-fib">
          <label class="small muted">order</label>
          <input type="number" name="order" value="${fib.order || 10}" min="1" max="${fib.max_order}" step="1" ${dis}>
          <button type="submit" ${op && !running ? "" : "disabled"}>Send goal</button>
          <button type="button" id="d-fib-cancel" ${op && running ? "" : "disabled"}>Cancel</button>
          <span class="pill ${fib.state === "succeeded" ? "ok" : fib.state === "running" ? "accent" : ["aborted", "rejected"].includes(fib.state) ? "hot" : ""}">${esc(fib.state)}</span>
        </form>
        ${fib.state !== "idle" ? `
          <div class="sprint-progress" style="margin:10px 0"><div class="progress"><span style="width:${Math.min(100, Math.round((done / expected) * 100))}%"></span></div><span class="mono">${done}/${expected}</span></div>
          <div class="seq mono small">${esc((fib.result || fib.partial || []).join(", ")) || "…"}</div>
          <p class="muted small">by ${esc(fib.by || "?")} · feedback every second, cancellable while running</p>` : '<p class="muted small">ros2_basics_interfaces/action/Fibonacci · goal → feedback (partial sequence) → result.</p>'}
      </section>
      ${d.lifecycle.map((n) => lifecycleHTML(n, op)).join("")}
    </div>`;
}

function feedHTML(items, empty) {
  if (!items.length) return `<div class="empty small">${empty}</div>`;
  return `<ul class="feed mono small">${items.slice().reverse().map((m) =>
    `<li><time>${fmtTime(m.t)}</time>${esc(m.data)}</li>`).join("")}</ul>`;
}

function lifecycleHTML(n, op) {
  return `
    <section class="card">
      <h2>Lifecycle · ${esc(n.node)} ${n.present ? `<span class="pill ${n.state === "active" ? "ok" : "accent"}">${esc(n.state || "?")}</span>` : '<span class="pill hot">missing</span>'}</h2>
      ${n.present ? "" : '<div class="hint-box small">Not running. On the Jetson: <code>make arm-basics</code> (docker/).</div>'}
      <div class="lc-states">${LIFECYCLE_STATES.map((s) => `<span class="lc ${s === n.state ? "current" : ""}">${s}</span>`).join('<i>→</i>')}</div>
      <div class="btns">${(n.transitions || []).map((t) =>
        `<button data-node="${esc(n.node)}" data-transition="${esc(t)}" ${op ? "" : "disabled"}>${esc(t)}</button>`).join("")
        || (n.present ? '<span class="muted small">no transition available</span>' : "")}</div>
      ${n.error ? `<div class="error-box small" style="margin-top:8px">${esc(n.error)}</div>` : ""}
    </section>`;
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
  else if (tab === "demos") body.innerHTML = demosHTML(live.demos);
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

async function act(url, body, label) {
  const r = await post(url, body);
  toast(r.ok ? `${label}: ok` : `${label}: ${r.error}`, r.ok ? "ok" : "hot");
  return r;
}

function bind(root) {
  root.addEventListener("click", (e) => {
    const w = e.target.closest("[data-watch]");
    if (w) watch(w.dataset.watch === state.watched ? null : w.dataset.watch);
    if (e.target.id === "g-unwatch") watch(null);
    const t = e.target.closest("[data-transition]");
    if (t) act(`/api/ros/demos/lifecycle/${encodeURIComponent(t.dataset.node)}`,
      { transition: t.dataset.transition }, `${t.dataset.node} ${t.dataset.transition}`);
    if (e.target.id === "d-fib-cancel") act("/api/ros/demos/fibonacci/cancel", null, "fibonacci cancel");
  });
  root.addEventListener("change", (e) => {
    if (e.target.id === "g-internal") { state.internal = e.target.checked; draw(); }
  });
  root.addEventListener("submit", async (e) => {
    e.preventDefault();
    const f = new FormData(e.target);
    if (e.target.id === "d-sum") {
      const r = await act("/api/ros/demos/add_two_ints", { a: Number(f.get("a")), b: Number(f.get("b")) }, "add_two_ints");
      if (r.ok) toast(`sum = ${r.data.sum}`, "ok");
    }
    if (e.target.id === "d-fib") act("/api/ros/demos/fibonacci", { order: Number(f.get("order")) }, "fibonacci goal");
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
      <div><h1>ROS 2</h1><p>The arm's ROS 2 graph, live: node health, every topic and service, and the ros2_basics demos</p></div>
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
  state.offs.push(on("demos", () => redrawUnlessEditing("demos")));
  state.offs.push(on("lease", () => redrawUnlessEditing("demos")));
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
