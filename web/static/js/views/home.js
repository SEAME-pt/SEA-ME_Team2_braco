// Overview: arm summary, who is here, ROS health, activity, current sprint, quick links.

import { mountActivity } from "../activity.js";
import { esc, fmt, fmtDate, getJSON, live, me, on } from "../util.js";
import { JOINTS } from "../specs.js";
import { currentMilestone, sprintProgress } from "./sprints.js";

const QUICK = [
  ["#/docs/docs/guides/dev-setup.md", "Development setup", "dev-setup.md"],
  ["#/docs/docs/guides/safety.md", "Safety rules", "safety.md"],
  ["#/docs/docs/hardware/protocol_commands.md", "Protocol commands", "protocol_commands.md"],
  ["#/docs/docs/hardware/inventory/inventory.md", "Hardware inventory", "inventory.md"],
  ["#/docs/docker/README.md", "ROS 2 containers", "docker/README.md"],
  ["#/ros/graph", "ROS 2 graph", "nodes, topics, services"],
  ["#/arm", "Command reference", "codes, limits, PID"],
];

export function render(root) {
  root.innerHTML = `
    <div class="page-head">
      <div>
        <h1>braco</h1>
        <p>myCobot 280 Jetson Nano · everything the team needs in one place, shared live</p>
      </div>
      <span class="pill" id="h-updated">—</span>
    </div>
    <div class="hero">
      <section class="card">
        <h2>Arm now <a href="#/arm" class="small">open panel →</a> <a href="#/control" class="small">controls →</a></h2>
        <div class="stat-row" style="margin-bottom:14px">
          <div class="stat"><b id="h-conn">—</b><span>connection</span></div>
          <div class="stat"><b id="h-power">—</b><span>servos</span></div>
          <div class="stat"><b id="h-moving">—</b><span>motion</span></div>
          <div class="stat"><b id="h-temp">—</b><span>hottest servo</span></div>
        </div>
        <div class="mini-joints" id="h-joints">
          ${JOINTS.map((j) => `<div><b>—</b><span>${j}</span></div>`).join("")}
        </div>
      </section>
      <section class="card">
        <h2>Team on the page <span class="pill" id="h-lease">—</span></h2>
        <div id="h-people" class="chips"></div>
        <h2 style="margin-top:16px">ROS 2 <a href="#/ros" class="small">health →</a></h2>
        <div id="h-ros" class="chips"></div>
      </section>
    </div>
    <div class="hero">
      <section class="card">
        <h2>Activity <a href="#/arm" class="small">arm panel →</a></h2>
        <div id="h-activity" class="activity-box"></div>
      </section>
      <section class="card" id="h-sprint">
        <h2>Current sprint <a href="#/sprints" class="small">all sprints →</a></h2>
        <div class="empty">Loading from GitHub…</div>
      </section>
    </div>
    <section class="card">
      <h2>Quick links</h2>
      <div class="quick">
        ${QUICK.map(([href, title, sub]) => `<a href="${href}"><b>${esc(title)}</b><small>${esc(sub)}</small></a>`).join("")}
      </div>
    </section>
  `;

  const $ = (id) => root.querySelector(id);
  const cells = [...root.querySelectorAll("#h-joints b")];
  const offs = [];

  offs.push(on("telemetry", (s) => {
    $("#h-conn").textContent = !s.connected ? "off" : s.source === "gazebo" ? "gazebo" : s.mock ? "mock" : s.source === "ros" ? "ros" : "on";
    $("#h-power").textContent = s.resting ? "resting" : s.power_on == null ? "—" : s.power_on ? "on" : "released";
    $("#h-moving").textContent = s.moving == null ? "—" : s.moving ? "yes" : "still";
    if (s.temps) {
      const max = Math.max(...s.temps);
      const level = s.temp_levels[s.temps.indexOf(max)];
      const el = $("#h-temp");
      el.textContent = `${max}°C`;
      el.className = `temp-cell ${level}`;
    }
    (s.angles || []).forEach((a, i) => { cells[i].textContent = `${fmt(a, 0)}°`; });
    $("#h-updated").textContent = s.updated
      ? `updated ${new Date(s.updated * 1000).toLocaleTimeString("en-GB")}` : "—";
  }));

  const people = () => {
    const holder = live.lease?.holder;
    $("#h-lease").textContent = holder ? `${holder.name} in control` : "arm free";
    $("#h-lease").className = `pill ${holder ? "accent" : "ok"}`;
    $("#h-people").innerHTML = live.presence.filter((c) => c.online).map((c) =>
      `<span class="pill ${c.id === holder?.id ? "accent" : ""}">${esc(c.name)}${c.id === me.id ? " (you)" : ""}</span>`).join("")
      || '<span class="muted small">nobody</span>';
  };
  offs.push(on("presence", people), on("lease", people));

  offs.push(on("health", (h) => {
    $("#h-ros").innerHTML = !h.available
      ? `<span class="pill">${esc(h.error || "not available")}</span>`
      : h.nodes.map((n) => `<span class="pill ${n.present ? "ok" : "hot"}">${esc(n.name)}</span>`).join("");
  }));

  offs.push(mountActivity($("#h-activity"), 12));

  getJSON("/api/sprints").then((data) => {
    const box = $("#h-sprint");
    const ms = currentMilestone(data.milestones);
    const sprint = data.sprints.find((s) => s.state === "open") || data.sprints[0];
    if (!sprint && !ms) {
      box.querySelector(".empty").textContent = data.error || "No sprint found.";
      return;
    }
    const p = sprint ? sprintProgress(sprint) : null;
    const days = ms?.due_on ? Math.ceil((new Date(ms.due_on) - Date.now()) / 86400000) : null;
    box.innerHTML = `
      <h2>Current sprint <a href="#/sprints" class="small">all sprints →</a></h2>
      ${sprint ? `
        <div class="sprint-head" style="margin-bottom:10px">
          <div><span class="num">SPRINT ${sprint.sprint}</span><h3>${esc(sprint.name)}</h3></div>
          <a class="pill" href="${esc(sprint.url)}" target="_blank" rel="noopener">#${sprint.number}</a>
        </div>
        <div class="sprint-progress" style="margin-bottom:14px">
          <div class="progress"><span style="width:${p.pct}%"></span></div>${p.done}/${p.total}
        </div>` : ""}
      ${ms ? `
        <div class="countdown">
          <b>${days == null ? "—" : Math.max(days, 0)}</b>
          <div><div>${days != null && days < 0 ? "days (deadline passed)" : "days to delivery"}</div>
          <div class="muted small">${esc(ms.title)} · ${fmtDate(ms.due_on)}</div></div>
        </div>` : ""}
    `;
  }).catch((e) => {
    const empty = $("#h-sprint .empty");
    if (empty) empty.textContent = `Could not load sprints: ${e.message}`;
  });

  return () => offs.forEach((off) => off());
}
