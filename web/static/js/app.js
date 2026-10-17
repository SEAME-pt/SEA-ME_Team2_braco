// Shell: hash router, shared telemetry stream, theme toggle.

import * as home from "./views/home.js";
import * as arm from "./views/arm.js";
import * as docs from "./views/docs.js";
import * as sprints from "./views/sprints.js";
import { connect } from "./util.js";

const ROUTES = { "": home, arm, docs, sprints };

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

// ---- Router -----------------------------------------------------------------

let cleanup = null;
let currentRoute = null;

function route() {
  const hash = location.hash.replace(/^#\/?/, "");
  const [name, ...rest] = hash.split("/");
  const view = ROUTES[name] || home;
  const param = decodeURIComponent(rest.join("/"));
  const main = document.getElementById("view");

  // Docs handles its own sub-navigation without a full re-render
  if (currentRoute === name && view.update) {
    view.update(param);
    return;
  }
  if (cleanup) cleanup();
  cleanup = null;
  currentRoute = name;
  main.innerHTML = "";
  main.scrollTop = 0;
  window.scrollTo(0, 0);

  document.querySelectorAll(".nav a").forEach((a) => {
    a.classList.toggle("active", a.dataset.route === (name || "home"));
  });
  cleanup = view.render(main, param) || null;
}

initTheme();
connect();
window.addEventListener("hashchange", route);
route();
