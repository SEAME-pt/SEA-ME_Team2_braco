// Shell: hash router, the live connection, control lease, who is online, theme.

import * as home from "./views/home.js";
import * as arm from "./views/arm.js";
import * as control from "./views/control.js";
import * as rosView from "./views/ros.js";
import * as docs from "./views/docs.js";
import * as sprints from "./views/sprints.js";
import {
  connect, esc, isOperator, live, me, on, OWNER_NAMES, outsideOwner, releaseControl, setName, setPage,
  takeControl, toast,
} from "./util.js";

const ROUTES = { "": home, arm, control, ros: rosView, docs, sprints };
const PAGE_NAMES = { "": "home", arm: "arm", control: "control", ros: "ROS 2", docs: "docs", sprints: "sprints" };

// ---- Theme --------------------------------------------------------------------

function initTheme() {
  const root = document.documentElement;
  try {
    const saved = localStorage.getItem("theme");
    if (saved) root.dataset.theme = saved;
  } catch (_) { /* storage blocked */ }
  document.getElementById("theme-toggle").addEventListener("click", () => {
    const dark = root.dataset.theme
      ? root.dataset.theme === "dark"
      : matchMedia("(prefers-color-scheme: dark)").matches;
    root.dataset.theme = dark ? "light" : "dark";
    try { localStorage.setItem("theme", root.dataset.theme); } catch (_) { /* ignore */ }
    window.dispatchEvent(new Event("themechange"));
  });
}

// ---- Name ---------------------------------------------------------------------

function askName(initial) {
  const dialog = document.getElementById("name-dialog");
  const input = document.getElementById("name-input");
  input.value = initial || "";
  return new Promise((resolve) => {
    dialog.addEventListener("close", () => resolve(input.value.trim()), { once: true });
    dialog.addEventListener("cancel", (e) => { if (!me.name) e.preventDefault(); });
    dialog.showModal();
    input.focus();
  });
}

function showName() {
  document.getElementById("me-name").textContent = me.name || "—";
}

function initName() {
  document.getElementById("me-btn").addEventListener("click", async () => {
    const name = await askName(me.name);
    if (name && name !== me.name) {
      setName(name);
      showName();
    }
  });
}

// ---- Connection, control lease and presence ----------------------------------

function drawConn() {
  const dot = document.querySelector("#conn .dot");
  const text = document.querySelector("#conn .conn-text");
  const s = live.telemetry;
  dot.className = "dot";
  if (!live.online) {
    dot.classList.add("bad");
    text.textContent = "server offline";
  } else if (!s || !s.connected) {
    dot.classList.add("bad");
    text.textContent = "arm disconnected";
  } else if (s.source === "gazebo") {
    dot.classList.add("mock");
    text.textContent = "Gazebo simulation";
  } else if (s.mock) {
    dot.classList.add("mock");
    text.textContent = "mock arm";
  } else {
    dot.classList.add("ok");
    text.textContent = s.source === "ros" ? "arm connected (ROS 2)" : "arm connected";
  }
}

function drawControl() {
  const box = document.getElementById("control");
  const text = box.querySelector(".control-text");
  const btn = document.getElementById("control-btn");
  const lease = live.lease;
  const mine = isOperator();
  box.classList.toggle("mine", mine);
  btn.disabled = !live.online || (!!lease?.holder && !mine);
  const outside = outsideOwner();
  box.classList.toggle("taken", (!!lease?.holder && !mine) || (!lease?.holder && !!outside));
  if (!live.online) {
    text.textContent = "offline";
  } else if (!lease?.holder && outside) {
    text.innerHTML = `<b>${esc(OWNER_NAMES[outside] || outside)}</b> is in control of the arm`;
  } else if (!lease?.holder) {
    text.innerHTML = "<b>Free</b> · nobody is in control";
  } else if (mine) {
    text.innerHTML = `<b>You</b> are in control · idle release in ${Math.ceil(lease.idle_left)} s`;
  } else {
    text.innerHTML = `<b>${esc(lease.holder.name)}</b> is in control · free in ≤ ${Math.ceil(lease.idle_left)} s`;
  }
  btn.textContent = mine ? "Release control" : "Take control";
  document.body.classList.toggle("is-operator", mine);
}

function drawPresence(clients) {
  const holder = live.lease?.holder?.id;
  const online = clients.filter((c) => c.online);
  document.getElementById("online-count").textContent = `(${online.length})`;
  document.getElementById("online-list").innerHTML = online.map((c) => `
    <li class="${c.id === me.id ? "me" : ""}">
      <i class="dot ${c.id === holder ? "ok" : ""}"></i>
      <span class="who">${esc(c.name)}${c.id === me.id ? " (you)" : ""}</span>
      <small>${esc(PAGE_NAMES[c.page] ?? c.page)}</small>
    </li>`).join("");
}

function initLive() {
  on("online", () => { drawConn(); drawControl(); });
  let owner = null;
  on("telemetry", (s) => {
    drawConn();
    if ((s?.control_owner ?? null) !== owner) { owner = s?.control_owner ?? null; drawControl(); }
  });
  on("lease", () => { drawControl(); drawPresence(live.presence); });
  on("presence", drawPresence);
  on("activity", ({ entry }) => {
    if (entry && entry.by !== me.id) {
      const kind = !entry.ok ? "hot" : /STOP/.test(entry.action) ? "hot" : "";
      toast(`${entry.who}: ${entry.action}${entry.detail ? ` (${entry.detail})` : ""}`, kind);
    }
  });
  document.getElementById("control-btn").addEventListener("click", async () => {
    if (isOperator()) await releaseControl();
    else await takeControl();
  });
  // Closing the tab frees the lease a few seconds later (server side), a reload keeps it.
}

// ---- Router -----------------------------------------------------------------

let cleanup = null;
let currentRoute = null;

function route() {
  const hash = location.hash.replace(/^#\/?/, "");
  const [name, ...rest] = hash.split("/");
  const key = name in ROUTES ? name : "";
  const view = ROUTES[key];
  const param = decodeURIComponent(rest.join("/"));
  const main = document.getElementById("view");

  // Docs and ROS handle their own sub-navigation without a full re-render
  if (currentRoute === key && view.update) {
    view.update(param);
    return;
  }
  if (cleanup) cleanup();
  cleanup = null;
  currentRoute = key;
  setPage(key || "");
  main.innerHTML = "";
  main.scrollTop = 0;
  window.scrollTo(0, 0);

  document.querySelectorAll(".nav a").forEach((a) => {
    a.classList.toggle("active", a.dataset.route === (key || "home"));
  });
  cleanup = view.render(main, param) || null;
}

async function start() {
  initTheme();
  initName();
  initLive();
  if (!me.name) setName(await askName("") || "Guest");
  showName();
  connect();
  window.addEventListener("hashchange", route);
  route();
}

start();
