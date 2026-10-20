// Live arm panel: 3D model, joints, temperatures, history, control, reference.
//
// Only the operator (the tab holding the control lease) can move the arm; STOP
// works for everybody. The operator's control panel (speed, step, the slider
// being dragged) is mirrored live to every other page. The speed is one shared value
// held by the server: only the operator changes it, everybody sees it, and above the
// limit (80) the operator has to accept a warning.

import { mountActivity } from "../activity.js";
import {
  esc, fmt, holderName, isOperator, live, on, post, shareUi, takeControl,
} from "../util.js";
import {
  COMMANDS, CONNECTION, FACTORY_PID, FIRMWARE_LIMITS, JOINTS, JOINT_NAMES,
  SOFT_LIMITS, TEMP_HOT_C, TEMP_WARM_C,
} from "../specs.js";

const JC = ["--j1", "--j2", "--j3", "--j4", "--j5", "--j6"];
const HISTORY_S = 60;
let movementConfirmed = false;   // asked once per page load, kept while switching views
const LEVEL_TEXT = { normal: "ok", warm: "warm", hot: "HOT" };
const SOURCE_PORT = { ros: "through arm_driver (ROS 2)", gazebo: "in Gazebo", mock: "mock data" };

function pct(v, i) {
  const [lo, hi] = FIRMWARE_LIMITS[i];
  return ((v - lo) / (hi - lo)) * 100;
}

function jointCard(i) {
  const [slo, shi] = SOFT_LIMITS[i];
  const [flo, fhi] = FIRMWARE_LIMITS[i];
  return `
    <div class="joint" style="--jc:var(${JC[i]})" data-j="${i}">
      <div class="joint-head"><div><b class="ang">—</b></div><span>${JOINTS[i]} · ${JOINT_NAMES[i]}</span></div>
      <div class="range" title="yellow band = soft limit; whole bar = firmware limit">
        <div class="soft" style="left:${pct(slo, i)}%;right:${100 - pct(shi, i)}%"></div>
        <div class="zero" style="left:${pct(0, i)}%"></div>
        <div class="needle" style="left:${pct(0, i)}%"></div>
      </div>
      <div class="range-labels"><span>${flo}°</span><span>${slo}° … ${shi}°</span><span>${fhi}°</span></div>
      <div class="temp"><span class="muted">temperature</span><span class="temp-cell tval">—</span></div>
    </div>`;
}

function referenceHTML() {
  return `
    <div class="grid cols-2" style="margin-top:16px">
      <section class="card span-2">
        <h2>Protocol commands <span class="muted small" style="text-transform:none;letter-spacing:0">libs/cpp/includes/mycobot.hpp · pymycobot</span></h2>
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
          <thead><tr><th>Joint</th><th>Firmware</th><th>Soft</th><th>P</th><th>I</th><th>D</th></tr></thead>
          <tbody>${JOINTS.map((j, i) => `
            <tr><td><b>${j}</b> <span class="muted small">${JOINT_NAMES[i]}</span></td>
            <td class="mono">${FIRMWARE_LIMITS[i].join(" … ")}</td>
            <td class="mono">${SOFT_LIMITS[i].join(" … ")}</td>
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

function controlRow(i) {
  const [lo, hi] = SOFT_LIMITS[i];
  return `
    <div class="ctl-row" style="--jc:var(${JC[i]})" data-j="${i}">
      <div class="ctl-name"><b>${JOINTS[i]}</b><span class="muted small">${JOINT_NAMES[i]}</span></div>
      <button class="step" data-d="-1" title="decrease">−</button>
      <input type="range" min="${lo}" max="${hi}" step="0.5" value="0">
      <button class="step" data-d="1" title="increase">+</button>
      <input type="number" class="num" min="${lo}" max="${hi}" step="0.5" value="0">
      <button class="go" title="send this angle">Go</button>
      <span class="ctl-now mono muted small">—</span>
    </div>`;
}

function controlHTML() {
  return `
    <section class="card ctl" style="margin-bottom:16px">
      <h2>Control
        <label class="arm-switch"><input type="checkbox" id="c-arm"> <span>enable motion</span></label>
      </h2>
      <div class="lease-banner" id="c-lease"></div>
      <div class="rest-banner" id="c-rest" hidden>
        <div><b>☾ Arm resting</b><span class="muted small">Motion is refused (rest mode). Wake up leaves rest mode and powers the servos; the arm stays where it is.</span></div>
        <button id="c-wake">☀ Wake up</button>
      </div>
      <div class="ctl-top">
        <button class="stop-btn" id="a-stop" title="Anyone can stop the arm, operator or not">■ STOP</button>
        <div class="ctl-poses">
          <button data-pose="park" title="goes to [0, 131, -148, 146, 4.5, 0] and stays powered">⌂ Park</button>
          <button data-pose="zero" title="all joints to 0°">◎ Zero</button>
        </div>
        <label class="ctl-speed">speed <b id="c-speed-v">30</b>
          <input type="range" id="c-speed" min="1" max="100" value="30">
          <span class="pill warn" id="c-speed-warn" hidden>⚠ fast</span>
          <span class="muted small" id="c-speed-lock" hidden>🔒 set by the operator</span>
        </label>
        <label class="ctl-speed">step ±
          <select id="c-step"><option>0.5</option><option>1</option><option selected>5</option><option>10</option></select> °
        </label>
      </div>
      <div class="ctl-rows">${JOINTS.map((_, i) => controlRow(i)).join("")}</div>
      <div class="ctl-extra" id="c-extra">
        <div class="extra" data-cap="gripper">
          <span class="muted small">Gripper <b id="c-grip-state">—</b></span>
          <div class="btns"><button data-grip="0">Open</button><button data-grip="1">Close</button></div>
        </div>
        <div class="extra" data-cap="led">
          <span class="muted small">Atom LED</span>
          <div class="btns"><input type="color" id="c-led" value="#f59e0b"><button id="c-led-set">Set</button></div>
        </div>
        <div class="extra" data-cap="rest">
          <span class="muted small">Rest mode <b id="c-rest-state">—</b></span>
          <div class="btns"><button id="c-rest-on" title="refuse motion, keep reading temperatures">Rest on</button></div>
        </div>
      </div>
      <p class="muted small" id="a-stop-msg">Each joint is sent when you release its control (or with Enter / Go) and stays inside the soft limit. Park goes to the rest pose; Zero moves every joint to 0°. Speed is shared by everybody and capped at 80 unless the operator accepts the warning. Tick “enable motion” to unlock the controls.</p>
      <div id="a-error"></div>
    </section>`;
}

function drawChart(canvas, history, key, range) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  canvas.width = w * dpr; canvas.height = h * dpr;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  const css = getComputedStyle(document.documentElement);
  const now = Date.now() / 1000;
  const [lo, hi] = range;
  const x = (t) => w - ((now - t) / HISTORY_S) * w;
  const y = (v) => h - 4 - ((v - lo) / (hi - lo)) * (h - 8);

  ctx.strokeStyle = css.getPropertyValue("--border");
  ctx.lineWidth = 1;
  ctx.font = `10px ${css.getPropertyValue("--mono")}`;
  ctx.fillStyle = css.getPropertyValue("--muted");
  const ticks = key === "temps" ? [30, TEMP_WARM_C, TEMP_HOT_C] : [-150, 0, 150];
  ticks.forEach((v) => {
    ctx.beginPath(); ctx.moveTo(0, y(v)); ctx.lineTo(w, y(v)); ctx.stroke();
    ctx.fillText(`${v}${key === "temps" ? "°C" : "°"}`, 4, y(v) - 3);
  });

  for (let j = 0; j < 6; j++) {
    ctx.strokeStyle = css.getPropertyValue(JC[j]);
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    let started = false;
    for (const p of history) {
      const v = p[key]?.[j];
      if (v == null) continue;
      if (!started) { ctx.moveTo(x(p.t), y(v)); started = true; } else ctx.lineTo(x(p.t), y(v));
    }
    ctx.stroke();
  }
}

const hex = (rgb) => "#" + rgb.map((v) => v.toString(16).padStart(2, "0")).join("");

export function render(root) {
  root.innerHTML = `
    <div class="page-head">
      <div>
        <h1>Arm</h1>
        <p>Live telemetry · myCobot 280 <span class="mono" id="a-port">—</span></p>
      </div>
      <div class="filters" id="a-badges"></div>
    </div>
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
    ${controlHTML()}
    <div class="grid cols-2">
      <section class="card">
        <h2>Cartesian pose</h2>
        <div class="kv" id="a-coords">
          ${["X", "Y", "Z", "RX", "RY", "RZ"].map((k) => `<div><span>${k} ${k.length === 1 ? "mm" : "°"}</span><b>—</b></div>`).join("")}
        </div>
      </section>
      <section class="card">
        <h2>History (${HISTORY_S}s)
          <div class="tabs" id="a-tabs"><button class="active" data-k="angles">angles</button><button data-k="temps">temperatures</button></div>
        </h2>
        <canvas class="chart" id="a-chart"></canvas>
        <div class="legend">${JOINTS.map((j, i) => `<span><i style="background:var(${JC[i]})"></i>${j}</span>`).join("")}</div>
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
  const history = [];
  let chartKey = "angles";
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

  $("#a-tabs").addEventListener("click", (e) => {
    const k = e.target.dataset.k;
    if (!k) return;
    chartKey = k;
    root.querySelectorAll("#a-tabs button").forEach((b) => b.classList.toggle("active", b.dataset.k === k));
    redraw();
  });

  function redraw() {
    drawChart($("#a-chart"), history, chartKey, chartKey === "temps" ? [25, 75] : [-180, 180]);
  }

  const armBox = $("#c-arm");
  const speedEl = $("#c-speed");
  const stepEl = $("#c-step");
  const rows = [...root.querySelectorAll(".ctl-row")];
  const lastSent = rows.map(() => 0);
  const dragging = rows.map(() => false);
  let connected = false;
  let resting = false;
  let operator = isOperator();
  let remote = null;   // the operator's mirrored panel, when someone else is in control

  const msg = (html) => { $("#a-stop-msg").innerHTML = html; };
  const stamp = () => new Date().toLocaleTimeString("en-GB");

  async function send(url, body, label) {
    const r = await post(url, body);
    msg(r.ok ? `<span class="pill ok">${esc(label)} · ${stamp()}</span>`
      : `<span class="pill hot">Failed: ${esc(r.error)}</span>`);
    return r.ok;
  }

  // What the operator's panel looks like right now, for the other pages
  function panelState(drag = null) {
    return { step: stepEl.value, enabled: armBox.checked, drag };
  }
  const share = (drag) => shareUi(panelState(drag));

  function updateEnabled() {
    const canMove = operator && armBox.checked && connected && !resting;
    root.querySelectorAll(".ctl-rows button, .ctl-rows input, [data-pose], .ctl-extra button, .ctl-extra input")
      .forEach((el) => { el.disabled = !canMove; });
    // rest mode on/off and LED do not move the arm: operator + connected is enough
    root.querySelectorAll("#c-rest-on, #c-led, #c-led-set").forEach((el) => { el.disabled = !(operator && connected); });
    armBox.disabled = !operator;
    speedEl.disabled = stepEl.disabled = !operator;
    $("#c-speed-lock").hidden = operator;
    $("#a-stop").disabled = !connected;
    $("#c-rest").hidden = !resting;
    $("#c-wake").disabled = !(operator && connected);
    $(".ctl").classList.toggle("is-resting", resting);
    $(".ctl").classList.toggle("view-only", !operator);
    drawLease();
  }

  function drawLease() {
    const box = $("#c-lease");
    const holder = holderName();
    if (operator) {
      box.className = "lease-banner mine";
      box.innerHTML = "<span><b>You are in control.</b> The others see your panel move live. Release it in the sidebar when you are done.</span>";
    } else if (holder) {
      box.className = "lease-banner taken";
      box.innerHTML = `<span>👁 <b>View only:</b> ${esc(holder)} is in control. Their panel is mirrored here. STOP still works for you.</span>`;
    } else {
      box.className = "lease-banner";
      box.innerHTML = '<span>Nobody is in control. Take control to move the arm.</span><button id="c-take">Take control</button>';
      box.querySelector("#c-take").addEventListener("click", takeControl);
    }
  }

  // The one-time question; true when movement may be enabled.
  function confirmRelease() {
    if (movementConfirmed) return true;
    movementConfirmed = confirm("Enable arm motion?\n\nThe joint controls, Park and Zero will move the arm without asking again. Keep the area around it clear.");
    return movementConfirmed;
  }

  armBox.addEventListener("change", () => {
    if (armBox.checked && !confirmRelease()) armBox.checked = false;
    updateEnabled();
    share();
  });

  $("#c-wake").addEventListener("click", async () => {
    if (!confirmRelease()) return;
    armBox.checked = true;
    share();
    if (await send("/api/wake", null, "wake up sent")) {
      resting = false;   // the next telemetry confirms it
      updateEnabled();
    }
  });
  // The shared speed, as the server holds it
  let limit = 80;
  let fast = false;
  function showSpeed(value) {
    $("#c-speed-v").textContent = value;
    $("#c-speed-warn").hidden = !(value > limit);
  }
  function applySpeed(s) {
    limit = s.limit;
    speedEl.max = s.max;
    fast = s.fast;
    if (!speedDragging) speedEl.value = s.value;
    showSpeed(Number(speedEl.value));
  }
  let speedDragging = false;
  speedEl.addEventListener("pointerdown", () => { speedDragging = true; });
  speedEl.addEventListener("input", () => showSpeed(Number(speedEl.value)));
  speedEl.addEventListener("change", async () => {
    speedDragging = false;
    let value = Number(speedEl.value);
    let unlocked = fast;
    if (value > limit && !unlocked) {
      unlocked = confirm(`Speed ${value} is above the safe limit of ${limit}.\n\nThe arm will move fast for everybody using it. Keep clear of it and keep STOP at hand.\n\nUse this speed?`);
      if (!unlocked) value = limit;
    }
    const r = await post("/api/speed", { value, unlocked });
    if (!r.ok) {
      msg(`<span class="pill hot">Failed: ${esc(r.error)}</span>`);
      applySpeed(live.speed || { value: 30, fast: false, limit, max: 100 });
    }
  });
  speedEl.addEventListener("pointerup", () => { speedDragging = false; });
  stepEl.addEventListener("change", () => share());

  $("#a-stop").addEventListener("click", () => {
    if (confirm("Send STOP to the arm?")) send("/api/stop", null, "STOP sent");
  });

  root.querySelectorAll("[data-pose]").forEach((btn) => btn.addEventListener("click", () => {
    const name = btn.dataset.pose;
    send(`/api/pose/${name}`, null, `${name} sent`);
    rows.forEach((_, i) => { lastSent[i] = Date.now(); });
  }));

  root.querySelectorAll("[data-grip]").forEach((btn) => btn.addEventListener("click", () => {
    const closed = btn.dataset.grip === "1";
    send("/api/gripper", { closed }, `gripper ${closed ? "closed" : "opened"}`);
  }));
  $("#c-led-set").addEventListener("click", () => {
    const v = $("#c-led").value;
    const rgb = [1, 3, 5].map((k) => parseInt(v.slice(k, k + 2), 16));
    send("/api/led", { r: rgb[0], g: rgb[1], b: rgb[2] }, `LED ${v}`);
  });
  $("#c-rest-on").addEventListener("click", () => {
    const turnOn = !resting;
    if (turnOn && !confirm("Enter rest mode? Motion is refused until Wake up.")) return;
    send("/api/rest", { on: turnOn }, `rest mode ${turnOn ? "on" : "off"}`);
  });

  rows.forEach((row, i) => {
    const [lo, hi] = SOFT_LIMITS[i];
    const slider = row.querySelector('input[type="range"]');
    const num = row.querySelector(".num");
    const clamp = (v) => Math.max(lo, Math.min(hi, v));
    const go = (v) => {
      v = clamp(Number.isFinite(v) ? v : 0);
      slider.value = num.value = v;
      lastSent[i] = Date.now();
      share();
      send(`/api/joint/${i + 1}`, { angle: v }, `${JOINTS[i]} → ${fmt(v)}°`);
    };
    slider.addEventListener("pointerdown", () => { dragging[i] = true; });
    slider.addEventListener("input", () => {
      num.value = slider.value;
      share({ j: i, value: Number(slider.value) });
    });
    slider.addEventListener("change", () => { dragging[i] = false; go(Number(slider.value)); });
    num.addEventListener("keydown", (e) => { if (e.key === "Enter") go(Number(num.value)); });
    row.querySelector(".go").addEventListener("click", () => go(Number(num.value)));
    row.querySelectorAll(".step").forEach((b) => b.addEventListener("click", () => {
      go(clamp(Number(num.value) + Number(b.dataset.d) * Number(stepEl.value)));
    }));
  });

  // ---- the operator's panel, mirrored -----------------------------------------
  function applyRemote(ui) {
    remote = !operator && ui?.state ? ui : null;
    rows.forEach((row) => row.classList.remove("remote"));
    if (!remote) return;
    const st = remote.state;
    stepEl.value = st.step;
    armBox.checked = !!st.enabled;
    if (st.drag) {
      const row = rows[st.drag.j];
      if (row) {
        row.classList.add("remote");
        row.querySelector('input[type="range"]').value = st.drag.value;
        row.querySelector(".num").value = st.drag.value;
        row.querySelector(".ctl-now").textContent = `${remote.name} → ${fmt(st.drag.value)}°`;
      }
    }
  }

  const offs = [];
  offs.push(on("lease", () => {
    const now = isOperator();
    if (now !== operator) {
      operator = now;
      if (!operator) armBox.checked = false;
      else share();
    }
    updateEnabled();
    applyRemote(live.ui);
  }));
  offs.push(on("ui", applyRemote));
  offs.push(on("speed", applySpeed));
  offs.push(mountActivity($("#a-activity"), 40));

  updateEnabled();

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
    const caps = s.capabilities || [];
    root.querySelectorAll("[data-cap]").forEach((el) => { el.hidden = !caps.includes(el.dataset.cap); });
    $("#c-extra").hidden = !["gripper", "led", "rest"].some((c) => caps.includes(c));
    $("#c-grip-state").textContent = s.gripper || "—";
    $("#c-rest-state").textContent = s.resting ? "on" : "off";
    $("#c-rest-on").textContent = s.resting ? "Rest off" : "Rest on";
    if (s.led && document.activeElement !== $("#c-led")) $("#c-led").value = hex(s.led);

    const nowResting = !!s.resting;
    if (connected !== !!s.connected || resting !== nowResting) {
      connected = !!s.connected;
      resting = nowResting;
      updateEnabled();
    }
    rows.forEach((row, i) => {
      const a = s.angles?.[i];
      const mirrored = remote?.state?.drag?.j === i;
      if (!mirrored) row.querySelector(".ctl-now").textContent = a == null ? "—" : `now ${fmt(a)}°`;
      // follow the real angle until the user moves a control, then wait for the arm to arrive
      if (a != null && !mirrored && !dragging[i] && Date.now() - lastSent[i] > 4000
          && document.activeElement !== row.querySelector(".num")) {
        row.querySelector('input[type="range"]').value = a;
        row.querySelector(".num").value = Math.round(a * 10) / 10;
      }
    });

    cards.forEach((card, i) => {
      const a = s.angles?.[i];
      card.querySelector(".ang").textContent = a == null ? "—" : `${fmt(a)}°`;
      if (a != null) card.querySelector(".needle").style.left = `${Math.max(0, Math.min(100, pct(a, i)))}%`;
      const t = s.temps?.[i];
      const lv = s.temp_levels?.[i];
      const tc = card.querySelector(".tval");
      tc.textContent = t == null ? "—" : `${t}°C ${LEVEL_TEXT[lv] === "ok" ? "" : "· " + LEVEL_TEXT[lv]}`;
      tc.className = `temp-cell tval ${lv || ""}`;
    });
    (s.coords || []).forEach((v, i) => { coordCells[i].textContent = fmt(v); });

    arm3d?.setAngles(s.angles);

    const now = Date.now() / 1000;
    history.push({ t: now, angles: s.angles, temps: s.temps });
    while (history.length && history[0].t < now - HISTORY_S) history.shift();
    redraw();
  }));

  const onTheme = () => redraw();
  window.addEventListener("themechange", onTheme);

  return () => {
    alive = false;
    offs.forEach((off) => off());
    window.removeEventListener("themechange", onTheme);
    arm3d?.dispose();
  };
}
