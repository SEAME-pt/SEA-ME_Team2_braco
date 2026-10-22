// Arm control page: STOP, Park / Zero, shared speed, per-joint controls, gripper, LED, rest mode.
//
// Only the operator (the tab holding the control lease) can move the arm; STOP works for
// everybody. While the joystick or zero/park has the arm (arm_driver's control owner),
// nobody on the page can take control. The operator's panel (speed, step, the slider
// being dragged) is mirrored live to every other page. The speed is one shared value
// held by the server: only the operator changes it, everybody sees it, and above the
// limit (80) the operator has to accept a warning.

import { mountActivity } from "../activity.js";
import {
  esc, fmt, holderName, isOperator, live, on, OWNER_NAMES, outsideOwner, post, shareUi, takeControl,
} from "../util.js";
import { JOINT_LIMITS, JOINTS, JOINT_NAMES } from "../specs.js";

const JC = ["--j1", "--j2", "--j3", "--j4", "--j5", "--j6"];
let movementConfirmed = false;   // asked once per page load, kept while switching views

function controlRow(i) {
  const [lo, hi] = JOINT_LIMITS[i];
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
          <button data-pose="park" title="goes to [0, 131, -148, 146, 4.5, 0] then releases the joints (rest mode)">⌂ Park</button>
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
      <p class="muted small" id="a-stop-msg">Each joint is sent when you release its control (or with Enter / Go) and stays inside the joint limits. Park goes to the rest pose; Zero moves every joint to 0°. Speed is shared by everybody and capped at 80 unless the operator accepts the warning. Tick “enable motion” to unlock the controls.</p>
      <div id="a-error"></div>
    </section>`;
}

const hex = (rgb) => "#" + rgb.map((v) => v.toString(16).padStart(2, "0")).join("");

export function render(root) {
  root.innerHTML = `
    <div class="page-head">
      <div>
        <h1>Control</h1>
        <p>Move the myCobot 280 · take control in the sidebar first</p>
      </div>
      <div class="filters" id="a-badges"></div>
    </div>
    ${controlHTML()}
    <section class="card">
      <h2>Activity <span class="muted small" style="text-transform:none;letter-spacing:0">everybody's commands, live</span></h2>
      <div id="a-activity" class="activity-box"></div>
    </section>
  `;

  const $ = (s) => root.querySelector(s);

  const armBox = $("#c-arm");
  const speedEl = $("#c-speed");
  const stepEl = $("#c-step");
  const rows = [...root.querySelectorAll(".ctl-row")];
  const lastSent = rows.map(() => 0);
  const dragging = rows.map(() => false);
  let connected = false;
  let resting = false;
  let owner = null;     // arm_driver's control owner, last drawn
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
    } else if (outsideOwner()) {
      const who = OWNER_NAMES[outsideOwner()] || outsideOwner();
      box.className = "lease-banner taken";
      box.innerHTML = `<span>🎮 <b>${esc(who)} is in control</b> of the arm. The page can take it back once it is free (5 s after the last move). STOP still works for you.</span>`;
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
    const [lo, hi] = JOINT_LIMITS[i];
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
    $("#a-badges").innerHTML = [
      !s.connected ? '<span class="pill hot">disconnected</span>'
        : s.source === "gazebo" ? '<span class="pill info">Gazebo simulation</span>'
        : s.mock ? '<span class="pill info">mock arm</span>' : '<span class="pill ok">connected</span>',
      s.resting ? '<span class="pill warn">resting</span>'
        : s.power_on == null ? "" : s.power_on ? '<span class="pill">servos on</span>' : '<span class="pill warn">servos released</span>',
      s.moving == null ? "" : s.moving ? '<span class="pill accent">moving</span>' : '<span class="pill">still</span>',
    ].join("");
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
    if ((s.control_owner ?? null) !== owner) {
      owner = s.control_owner ?? null;
      drawLease();
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

  }));

  return () => offs.forEach((off) => off());
}
