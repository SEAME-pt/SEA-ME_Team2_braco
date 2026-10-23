// Live arm panel: 3D model, camera, joints, temperatures, history, reference. The controls are in control.js.

import { mountActivity } from "../activity.js";
import { mountCamera } from "../camera.js";
import { mountChart } from "../chart.js";
import { esc, fmt, on } from "../util.js";
import {
  COMMANDS, CONNECTION, FACTORY_PID, JOINT_LIMITS, JOINTS, JOINT_NAMES,
  TEMP_HOT_C, TEMP_WARM_C,
} from "../specs.js";

const JC = ["--j1", "--j2", "--j3", "--j4", "--j5", "--j6"];
const LEVEL_TEXT = { normal: "ok", warm: "warm", hot: "HOT" };
const SOURCE_PORT = { ros: "through arm_driver (ROS 2)", gazebo: "in Gazebo", mock: "mock data" };

function pct(v, i) {
  const [lo, hi] = JOINT_LIMITS[i];
  return ((v - lo) / (hi - lo)) * 100;
}

function jointCard(i) {
  const [lo, hi] = JOINT_LIMITS[i];
  return `
    <div class="joint" style="--jc:var(${JC[i]})" data-j="${i}">
      <div class="joint-head"><div><b class="ang">—</b></div><span>${JOINTS[i]} · ${JOINT_NAMES[i]}</span></div>
      <div class="range" title="whole bar = joint limits">
        <div class="zero" style="left:${pct(0, i)}%"></div>
        <div class="needle" style="left:${pct(0, i)}%"></div>
      </div>
      <div class="range-labels"><span>${lo}°</span><span>${hi}°</span></div>
      <div class="temp"><span class="muted">temperature</span><span class="temp-cell tval">—</span></div>
      <div class="temp"><span class="muted">voltage</span><span class="temp-cell vval">—</span></div>
    </div>`;
}

function referenceHTML() {
  return `
    <div class="grid cols-2" style="margin-top:16px">
      <section class="card span-2">
        <h2>Protocol commands <span class="muted small" style="text-transform:none;letter-spacing:0">ros2_ws/src/mycobot_cpp/includes/mycobot.hpp · pymycobot</span></h2>
        <div class="table-wrap"><table>
          <thead><tr><th>Code</th><th>Name</th><th>Python</th><th>C++</th><th>Description</th><th></th></tr></thead>
          <tbody>${COMMANDS.map((c) => `
            <tr><td class="mono">${c.code}</td><td class="mono">${c.name}</td>
            <td><code>${esc(c.py)}</code></td><td><code>${esc(c.cpp)}</code></td>
            <td>${esc(c.desc)}</td>
            <td>${c.write ? '<span class="pill warn rw">writes</span>' : '<span class="pill ok rw">read</span>'}</td></tr>`).join("")}
          </tbody></table></div>
      </section>
      <section class="card">
        <h2>Limits and factory PID</h2>
        <div class="table-wrap"><table>
          <thead><tr><th>Joint</th><th>Limits</th><th>P</th><th>I</th><th>D</th></tr></thead>
          <tbody>${JOINTS.map((j, i) => `
            <tr><td><b>${j}</b> <span class="muted small">${JOINT_NAMES[i]}</span></td>
            <td class="mono">${JOINT_LIMITS[i].join(" … ")}</td>
            ${FACTORY_PID[i].map((v) => `<td class="mono">${v}</td>`).join("")}</tr>`).join("")}
          </tbody></table></div>
        <p class="muted small">setPid() accepts P and D up to 1.5× the factory value and I up to 8.</p>
      </section>
      <section class="card">
        <h2>Connection and units</h2>
        <table><tbody>${CONNECTION.map(([k, v]) => `<tr><th>${k}</th><td class="mono">${esc(v)}</td></tr>`).join("")}</tbody></table>
        <p class="muted small">Temperature: normal &lt; ${TEMP_WARM_C}°C · warm ≥ ${TEMP_WARM_C}°C · hot ≥ ${TEMP_HOT_C}°C (servos protect themselves at ~70°C).</p>
      </section>
    </div>`;
}

export function render(root) {
  root.innerHTML = `
    <div class="page-head">
      <div>
        <h1>Arm</h1>
        <p>Live telemetry · myCobot 280 <span class="mono" id="a-port">—</span> · <a href="#/control">controls →</a></p>
      </div>
      <div class="filters" id="a-badges"></div>
    </div>
    <div id="a-error"></div>
    <div class="arm-top">
      <section class="card viewer" id="a-viewer">
        <div class="overlay" id="a-overlay"></div>
        <div class="viewer-tools">
          <div class="tabs" id="a-views">
            <button data-v="iso" class="active">3D</button><button data-v="front">Front</button><button data-v="side">Side</button><button data-v="top">Top</button>
          </div>
          <div class="tabs" id="a-toggles">
            <button data-t="labels" class="active" title="Joint names">Joints</button><button data-t="trail" class="active" title="Path of the tool tip">Trail</button>
          </div>
        </div>
        <div class="hint">drag to rotate · scroll to zoom · double click back to 3D · orange ring = 280 mm reach</div>
      </section>
      <section class="card">
        <h2>Joints</h2>
        <div class="joints">${JOINTS.map((_, i) => jointCard(i)).join("")}</div>
      </section>
    </div>
    <section class="card history" style="margin-bottom:16px">
      <h2>History <span class="muted small" style="text-transform:none;letter-spacing:0">angle · temperature · servo voltage per joint</span></h2>
      <div id="a-history"></div>
    </section>
    <div class="grid cols-2">
      <section class="card camera-card">
        <h2>Camera <span class="muted small" style="text-transform:none;letter-spacing:0">eye-in-hand · /camera/image_raw/compressed</span></h2>
        <div id="a-camera"></div>
      </section>
      <section class="card">
        <h2>Cartesian pose</h2>
        <div class="kv" id="a-coords">
          ${["X", "Y", "Z", "RX", "RY", "RZ"].map((k) => `<div><span>${k} ${k.length === 1 ? "mm" : "°"}</span><b>—</b></div>`).join("")}
        </div>
      </section>
      <section class="card span-2">
        <h2>Activity <span class="muted small" style="text-transform:none;letter-spacing:0">everybody's commands, live</span></h2>
        <div id="a-activity" class="activity-box"></div>
      </section>
    </div>
    ${referenceHTML()}
  `;

  const $ = (s) => root.querySelector(s);
  const cards = [...root.querySelectorAll(".joint")];
  const coordCells = [...root.querySelectorAll("#a-coords b")];
  let arm3d = null;
  let alive = true;

  import("../arm3d.js")
    .then(({ createArm3D }) => { if (alive) arm3d = createArm3D($("#a-viewer")); })
    .catch(() => { $("#a-viewer").insertAdjacentHTML("beforeend", '<div class="empty">3D model unavailable (no access to the CDN?)</div>'); });

  $("#a-views").addEventListener("click", (e) => {
    const v = e.target.dataset.v;
    if (!v) return;
    arm3d?.setView(v);
    root.querySelectorAll("#a-views button").forEach((b) => b.classList.toggle("active", b.dataset.v === v));
  });

  $("#a-toggles").addEventListener("click", (e) => {
    const t = e.target.dataset.t;
    if (!t) return;
    const isOn = !e.target.classList.contains("active");
    e.target.classList.toggle("active", isOn);
    if (t === "labels") arm3d?.setLabels(isOn);
    if (t === "trail") arm3d?.setTrail(isOn);
  });

  const offs = [mountChart($("#a-history"))];
  offs.push(mountActivity($("#a-activity"), 40));
  offs.push(mountCamera($("#a-camera")));

  offs.push(on("telemetry", (s) => {
    $("#a-port").textContent = SOURCE_PORT[s.source] || "on /dev/ttyTHS1";
    $("#a-badges").innerHTML = [
      !s.connected ? '<span class="pill hot">disconnected</span>'
        : s.source === "gazebo" ? '<span class="pill info">Gazebo simulation</span>'
        : s.mock ? '<span class="pill info">mock arm</span>' : '<span class="pill ok">connected</span>',
      s.resting ? '<span class="pill warn">resting</span>'
        : s.power_on == null ? "" : s.power_on ? '<span class="pill">servos on</span>' : '<span class="pill warn">servos released</span>',
      s.moving == null ? "" : s.moving ? '<span class="pill accent">moving</span>' : '<span class="pill">still</span>',
    ].join("");
    $("#a-overlay").innerHTML = s.angles ? "" : '<span class="pill">no angle reading</span>';
    $("#a-error").innerHTML = s.error ? `<div class="error-box">${esc(s.error)}</div>` : "";
    cards.forEach((card, i) => {
      const a = s.angles?.[i];
      card.querySelector(".ang").textContent = a == null ? "—" : `${fmt(a)}°`;
      if (a != null) card.querySelector(".needle").style.left = `${Math.max(0, Math.min(100, pct(a, i)))}%`;
      const t = s.temps?.[i];
      const lv = s.temp_levels?.[i];
      const tc = card.querySelector(".tval");
      tc.textContent = t == null ? "—" : `${t}°C ${LEVEL_TEXT[lv] === "ok" ? "" : "· " + LEVEL_TEXT[lv]}`;
      tc.className = `temp-cell tval ${lv || ""}`;
      const v = s.volts?.[i];
      card.querySelector(".vval").textContent = v == null ? "—" : `${v.toFixed(1)} V`;
    });
    (s.coords || []).forEach((v, i) => { coordCells[i].textContent = fmt(v); });

    arm3d?.setAngles(s.angles);
  }));

  return () => {
    alive = false;
    offs.forEach((off) => off());
    arm3d?.dispose();
  };
}
