// Pick & place page: the arm seen from above, where it can reach with the tool pointing down,
// and the two points of a /pick_and_place goal, chosen by clicking (or dragging) on the map.
//
// The page only chooses the points: braco_mission plans and moves the arm (MoveIt 2), takes
// arm_driver's control as `mission` for the goal (the operator lends it, web/server/session.py)
// and reports every stage. Only the operator (the control lease) sends a goal; the others see
// the operator's points move live and the stages of the running goal. STOP works for everybody.
//
// Coordinates are braco_mission's planning frame `world`: the table surface under the arm,
// x forward, y to the arm's left, z up, in mm on the page (metres to the server).

import {
  BASE_KEEPOUT_MM, forward, reachable, whyNot,
} from "../kinematics.js";
import {
  esc, fmt, holderName, isOperator, live, on, OWNER_NAMES, outsideOwner, post, shareUi, takeControl,
} from "../util.js";

const VIEW_MM = 340;          // the map shows ±VIEW_MM around the base
const CELL_MM = 3;            // reach raster resolution
const Z_MIN = 20, Z_MAX = 250, APPROACH_MIN = 20, APPROACH_MAX = 100;
const DEFAULTS = {            // the Gazebo demo of braco_mission/README.md
  pick: [200, -40, 25], place: [200, 80, 25], approach: 50,
};
// The lab bench of the Gazebo world (braco_simulation/worlds/lab_bench.sdf) in `world`, mm:
// two tables side by side, their near edge 70 mm behind the base, a gap at y = -85.
const TABLE_NEAR_X = -70;
const TABLE_GAP = [-90, -80];
const STORE_KEY = "braco-pick-place";

const STAGE_LABELS = {
  control: ["Control", "real arm: braco_mission takes the control, arm_controller on"],
  open: ["Open gripper", ""],
  pre_grasp: ["Above the object", "point-to-point move, approach height above the pick"],
  approach_pick: ["Down to the object", "straight line"],
  close: ["Close gripper", "stalling on the object counts as closed"],
  retreat_pick: ["Lift", "straight line up"],
  pre_place: ["Above the target", "point-to-point move"],
  approach_place: ["Down to the target", "straight line"],
  release: ["Open gripper", "let go of the object"],
  retreat_place: ["Lift", "straight line up"],
};

function loadSaved() {
  try {
    const s = JSON.parse(localStorage.getItem(STORE_KEY) || "null");
    if (s && Array.isArray(s.pick) && Array.isArray(s.place) && Number.isFinite(s.approach)) return s;
  } catch (_) { /* storage blocked or bad JSON */ }
  return null;
}

function save(job) {
  try { localStorage.setItem(STORE_KEY, JSON.stringify(job)); } catch (_) { /* ignore */ }
}

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function rgb(color) {
  const hex = color.replace("#", "");
  if (/^[0-9a-f]{6}$/i.test(hex)) return [0, 2, 4].map((k) => parseInt(hex.slice(k, k + 2), 16));
  const m = color.match(/\d+(\.\d+)?/g);
  return m ? m.slice(0, 3).map(Number) : [128, 128, 128];
}

function pointHTML(id, label, hint) {
  return `
    <fieldset class="pp-point" id="pp-${id}">
      <legend><i class="pp-key ${id}"></i>${label} <span class="muted small">${hint}</span></legend>
      <label>x <input type="number" step="1" data-k="0"></label>
      <label>y <input type="number" step="1" data-k="1"></label>
      <label>z <input type="number" step="1" min="${Z_MIN}" max="${Z_MAX}" data-k="2"></label>
      <span class="pp-check"></span>
    </fieldset>`;
}

export function render(root) {
  const saved = loadSaved();
  const job = {
    pick: [...(saved?.pick || DEFAULTS.pick)],
    place: [...(saved?.place || DEFAULTS.place)],
    approach: saved?.approach ?? DEFAULTS.approach,
  };
  let armed = "pick";          // what a click on the map sets
  let operator = isOperator();
  let mission = live.mission || null;
  let telemetry = live.telemetry || null;
  let hover = null;            // [x, y] mm under the pointer
  let drag = null;             // "pick" | "place" while dragging a marker
  let remote = null;           // the operator's points, mirrored, when someone else is in control

  root.innerHTML = `
    <div class="page-head">
      <div>
        <h1>Pick &amp; place</h1>
        <p>Click where the object is, then where it goes · braco_mission plans and moves the arm</p>
      </div>
      <div class="filters" id="pp-badges"></div>
    </div>
    <div class="pp-layout">
      <section class="card pp-map-card">
        <h2>Top view
          <span class="tabs" id="pp-armed" role="group" aria-label="A click on the map sets">
            <button type="button" data-armed="pick" class="active">Set pick</button>
            <button type="button" data-armed="place">Set place</button>
          </span>
        </h2>
        <div class="pp-map" id="pp-map">
          <canvas id="pp-canvas" aria-label="Top view of the arm and its reach"></canvas>
          <div class="pp-readout mono small" id="pp-readout"></div>
        </div>
        <div class="legend">
          <span><i style="background:var(--pp-reach)"></i>reach, tool down, at the pick height</span>
          <span id="pp-legend-place" hidden><i style="background:var(--pp-reach-2)"></i>at the place height</span>
          <span><i style="background:var(--muted)"></i>base (no target)</span>
          <span><i style="background:var(--j2)"></i>the arm now</span>
        </div>
      </section>
      <div class="pp-side">
        <section class="card">
          <h2>Goal</h2>
          <div class="lease-banner" id="pp-lease"></div>
          <div class="stop-banner" id="pp-stopped" hidden>
            <div><b>■ STOP latched</b><span class="muted small">Nothing moves until the STOP is released (Control page).</span></div>
          </div>
          ${pointHTML("pick", "Pick", "the object, mm")}
          ${pointHTML("place", "Place", "where it goes, mm")}
          <label class="pp-approach">approach height
            <input type="number" id="pp-approach" min="${APPROACH_MIN}" max="${APPROACH_MAX}" step="5"> mm
            <span class="muted small">above each point, then a straight line down</span>
          </label>
          <div class="pp-actions">
            <button class="primary" id="pp-run" type="button">▶ Run pick &amp; place</button>
            <button id="pp-cancel" type="button">Cancel</button>
            <button class="stop-btn" id="pp-stop" type="button" title="Anyone can stop the arm, operator or not">■ STOP</button>
          </div>
          <p class="muted small" id="pp-msg">The tcp is the point between the fingertips; z is its height above the table (25 mm grips a 40 mm block by its middle). The tool points down, fingers closing along x. braco_mission moves at its own velocity scaling, not the page's speed.</p>
          <div id="pp-error"></div>
        </section>
        <section class="card">
          <h2>Progress <span class="muted small" id="pp-who" style="text-transform:none;letter-spacing:0"></span></h2>
          <ol class="pp-stages" id="pp-stages"></ol>
          <div id="pp-result"></div>
        </section>
      </div>
    </div>`;

  const $ = (s) => root.querySelector(s);
  const canvas = $("#pp-canvas");
  const ctx = canvas.getContext("2d");
  const fields = { pick: $("#pp-pick"), place: $("#pp-place") };
  const approachEl = $("#pp-approach");

  // ---- theme colours ----------------------------------------------------------------
  let colors = {};
  function readColors() {
    colors = {
      text: cssVar("--text"), muted: cssVar("--muted"), border: cssVar("--border"),
      surface: cssVar("--surface"), surface2: cssVar("--surface-2"), accent: cssVar("--accent"),
      ok: cssVar("--ok"), info: cssVar("--info"), hot: cssVar("--hot"), warn: cssVar("--warn"),
      arm: cssVar("--j2"), joints: [1, 2, 3, 4, 5, 6].map((i) => cssVar(`--j${i}`)),
      reach: cssVar("--accent"),
      reach2: cssVar("--info"),
    };
  }

  // ---- geometry: world mm <-> canvas px (x up, y to the left) ------------------------
  let size = 400, scale = 1;
  const toPx = (x, y) => [size / 2 - y * scale, size / 2 - x * scale];
  const toMm = (px, py) => [(size / 2 - py) / scale, (size / 2 - px) / scale];

  // ---- the reach raster, recomputed when a height changes ---------------------------
  const cells = Math.round((2 * VIEW_MM) / CELL_MM);
  const reachCanvas = document.createElement("canvas");
  reachCanvas.width = reachCanvas.height = cells;
  let reachKey = "";
  function buildReach() {
    const pickZ = job.pick[2], placeZ = job.place[2], h = job.approach;
    const key = [pickZ, placeZ, h, colors.reach, colors.reach2].join("|");
    if (key === reachKey) return;
    reachKey = key;
    const both = Math.round(pickZ) !== Math.round(placeZ);
    $("#pp-legend-place").hidden = !both;
    const rc = reachCanvas.getContext("2d");
    const img = rc.createImageData(cells, cells);
    const [r1, g1, b1] = rgb(colors.reach), [r2, g2, b2] = rgb(colors.reach2);
    for (let row = 0; row < cells; row++) {
      const x = VIEW_MM - (row + 0.5) * CELL_MM;
      for (let col = 0; col < cells; col++) {
        const y = VIEW_MM - (col + 0.5) * CELL_MM;
        const a = reachable(x, y, pickZ, h);
        const b = both && reachable(x, y, placeZ, h);
        if (!a && !b) continue;
        const i = (row * cells + col) * 4;
        const [r, g, bl] = a ? [r1, g1, b1] : [r2, g2, b2];
        img.data[i] = r; img.data[i + 1] = g; img.data[i + 2] = bl;
        img.data[i + 3] = a && b ? 95 : 60;
      }
    }
    rc.putImageData(img, 0, 0);
  }

  // ---- drawing -----------------------------------------------------------------------
  function shown() {
    // The running goal for everybody; else the operator's points (mirrored) or mine
    if (mission?.running && mission.goal) {
      const g = mission.goal;
      return { pick: g.pick.map((v) => v * 1000), place: g.place.map((v) => v * 1000), running: true };
    }
    if (remote) return { pick: remote.pick, place: remote.place, running: false };
    return { pick: job.pick, place: job.place, running: false };
  }

  function line(points, color, width, dash = []) {
    ctx.beginPath();
    points.forEach(([x, y], i) => {
      const [px, py] = toPx(x, y);
      if (i) ctx.lineTo(px, py); else ctx.moveTo(px, py);
    });
    ctx.setLineDash(dash);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function label(text, x, y, color, dx = 10, dy = -10) {
    const [px, py] = toPx(x, y);
    ctx.font = "600 11px Inter, system-ui, sans-serif";
    const w = ctx.measureText(text).width + 10;
    if (px + dx + w > size - 2) dx = -dx - w + 6;   // near the right edge: on the other side
    if (py + dy - 9 < 2) dy = -dy;
    ctx.fillStyle = colors.surface;
    ctx.globalAlpha = 0.85;
    ctx.fillRect(px + dx - 2, py + dy - 9, w, 16);
    ctx.globalAlpha = 1;
    ctx.fillStyle = color;
    ctx.fillText(text, px + dx + 3, py + dy + 3);
  }

  function marker(kind, [x, y, z], live_) {
    const [px, py] = toPx(x, y);
    const color = kind === "pick" ? colors.ok : colors.info;
    const bad = whyNot(x, y, z, job.approach);
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = bad ? colors.hot : color;
    ctx.fillStyle = color;
    if (kind === "pick") {
      // the object, a 20 x 20 mm block seen from above
      const s = 10 * scale;
      ctx.globalAlpha = 0.25;
      ctx.fillRect(px - s, py - s, 2 * s, 2 * s);
      ctx.globalAlpha = 1;
      ctx.strokeRect(px - s, py - s, 2 * s, 2 * s);
    } else {
      ctx.beginPath();
      ctx.arc(px, py, 12 * scale, 0, 2 * Math.PI);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(px, py, 3, 0, 2 * Math.PI);
      ctx.fill();
    }
    if (drag === kind || (live_ && hover && Math.hypot(hover[0] - x, hover[1] - y) < 14)) {
      ctx.beginPath();
      ctx.arc(px, py, 18 * scale + 4, 0, 2 * Math.PI);
      ctx.strokeStyle = color;
      ctx.lineWidth = 1;
      ctx.stroke();
    }
    label(`${kind === "pick" ? "PICK" : "PLACE"} z ${Math.round(z)}`, x, y, bad ? colors.hot : color);
  }

  function draw() {
    if (!size) return;
    buildReach();
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, size, size);

    // tables of the lab bench, and what is off them
    const [gx0, gy0] = toPx(VIEW_MM, TABLE_GAP[1]);
    const [gx1] = toPx(VIEW_MM, TABLE_GAP[0]);
    const [, edgeY] = toPx(TABLE_NEAR_X, 0);
    ctx.fillStyle = colors.surface2;
    ctx.fillRect(0, 0, size, edgeY);
    ctx.fillStyle = colors.border;
    ctx.fillRect(gx0, gy0, gx1 - gx0, edgeY - gy0);
    ctx.save();
    ctx.beginPath();
    ctx.rect(0, edgeY, size, size - edgeY);
    ctx.clip();
    ctx.strokeStyle = colors.border;
    ctx.lineWidth = 1;
    for (let k = -size; k < size * 2; k += 10) {
      ctx.beginPath();
      ctx.moveTo(k, size);
      ctx.lineTo(k + size, 0);
      ctx.stroke();
    }
    ctx.restore();

    // grid every 50 mm, labels every 100 mm
    ctx.lineWidth = 1;
    ctx.font = "10px 'JetBrains Mono', monospace";
    for (let v = -300; v <= 300; v += 50) {
      ctx.strokeStyle = colors.border;
      ctx.globalAlpha = v === 0 ? 1 : 0.6;
      line([[v, -VIEW_MM], [v, VIEW_MM]], colors.border, v === 0 ? 1.5 : 1);
      line([[-VIEW_MM, v], [VIEW_MM, v]], colors.border, v === 0 ? 1.5 : 1);
      ctx.globalAlpha = 1;
      if (v % 100 === 0 && v !== 0) {
        ctx.fillStyle = colors.muted;
        const [px, py] = toPx(v, 0);
        ctx.fillText(`${v}`, px + 4, py - 3);
        const [qx, qy] = toPx(0, v);
        ctx.fillText(`${v}`, qx + 3, qy - 4);
      }
    }
    // reach
    ctx.imageSmoothingEnabled = true;
    const [rx, ry] = toPx(VIEW_MM, VIEW_MM);
    ctx.drawImage(reachCanvas, rx, ry, 2 * VIEW_MM * scale, 2 * VIEW_MM * scale);
    // reach rings, for scale
    ctx.setLineDash([3, 4]);
    ctx.strokeStyle = colors.muted;
    ctx.globalAlpha = 0.5;
    for (const r of [100, 200, 280]) {
      const [cx, cy] = toPx(0, 0);
      ctx.beginPath();
      ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    ctx.setLineDash([]);
    // base keep-out and the G-base plate
    {
      const [bx, by] = toPx(BASE_KEEPOUT_MM.x, BASE_KEEPOUT_MM.y);
      ctx.fillStyle = colors.muted;
      ctx.globalAlpha = 0.18;
      ctx.fillRect(bx, by, 2 * BASE_KEEPOUT_MM.y * scale, 2 * BASE_KEEPOUT_MM.x * scale);
      ctx.globalAlpha = 0.55;
      const [px, py] = toPx(55, 75);
      ctx.fillRect(px, py, 150 * scale, 110 * scale);
      ctx.globalAlpha = 1;
    }
    // axes
    {
      const [ox, oy] = toPx(0, 0);
      ctx.strokeStyle = colors.hot;
      ctx.fillStyle = colors.hot;
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox, oy - 40); ctx.stroke();
      ctx.fillText("x", ox + 4, oy - 34);
      ctx.strokeStyle = colors.ok;
      ctx.fillStyle = colors.ok;
      ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox - 40, oy); ctx.stroke();
      ctx.fillText("y", ox - 44, oy - 5);
    }
    ctx.fillStyle = colors.muted;
    ctx.font = "11px Inter, system-ui, sans-serif";
    ctx.fillText("front of the arm ↑", 10, 18);
    ctx.fillText("off the table (lab bench)", 10, size - 10);

    // the goal: path, then the markers
    const s = shown();
    line([s.pick, s.place], s.running ? colors.accent : colors.muted, 2, [6, 5]);
    {
      const [ax, ay] = toPx(s.place[0], s.place[1]);
      const [bx, by] = toPx(s.pick[0], s.pick[1]);
      const ang = Math.atan2(ay - by, ax - bx);
      const tip = [ax - Math.cos(ang) * 16, ay - Math.sin(ang) * 16];
      ctx.fillStyle = s.running ? colors.accent : colors.muted;
      ctx.beginPath();
      ctx.moveTo(tip[0], tip[1]);
      ctx.lineTo(tip[0] - 10 * Math.cos(ang - 0.4), tip[1] - 10 * Math.sin(ang - 0.4));
      ctx.lineTo(tip[0] - 10 * Math.cos(ang + 0.4), tip[1] - 10 * Math.sin(ang + 0.4));
      ctx.fill();
    }
    marker("pick", s.pick, !s.running);
    marker("place", s.place, !s.running);

    // the arm, now
    const angles = telemetry?.angles;
    if (angles) {
      const k = forward(angles);
      const pts = k.points.slice(1);
      line(pts.map((p) => [p[0], p[1]]), colors.arm, 7);
      pts.slice(1).forEach((p, i) => {
        const [px, py] = toPx(p[0], p[1]);
        ctx.beginPath();
        ctx.arc(px, py, 4.5, 0, 2 * Math.PI);
        ctx.fillStyle = colors.joints[Math.min(i, 5)];
        ctx.fill();
      });
      line([k.flange, k.tcp].map((p) => [p[0], p[1]]), colors.arm, 3);
      const [tx, ty] = toPx(k.tcp[0], k.tcp[1]);
      ctx.beginPath();
      ctx.arc(tx, ty, 6, 0, 2 * Math.PI);
      ctx.fillStyle = colors.accent;
      ctx.fill();
      ctx.strokeStyle = colors.surface;
      ctx.lineWidth = 2;
      ctx.stroke();
      label(`tcp z ${Math.round(k.tcp[2])}`, k.tcp[0], k.tcp[1], colors.text, 10, 16);
    }

    // hover crosshair
    if (hover && !drag) {
      const [px, py] = toPx(hover[0], hover[1]);
      ctx.strokeStyle = colors.text;
      ctx.globalAlpha = 0.35;
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px - 8, py); ctx.lineTo(px + 8, py); ctx.moveTo(px, py - 8); ctx.lineTo(px, py + 8); ctx.stroke();
      ctx.globalAlpha = 1;
    }
  }

  let frame = 0;
  const redraw = () => {
    if (frame) return;
    frame = requestAnimationFrame(() => { frame = 0; draw(); });
  };

  function resize() {
    const box = $("#pp-map");
    const w = Math.max(240, Math.floor(box.clientWidth));
    size = Math.min(w, 720);
    const dpr = window.devicePixelRatio || 1;
    canvas.width = size * dpr;
    canvas.height = size * dpr;
    canvas.style.width = canvas.style.height = `${size}px`;
    scale = size / (2 * VIEW_MM);
    redraw();
  }
  const observer = new ResizeObserver(resize);
  observer.observe($("#pp-map"));

  // ---- the form ----------------------------------------------------------------------
  function writeFields() {
    const s = shown();
    for (const kind of ["pick", "place"]) {
      fields[kind].querySelectorAll("input").forEach((el) => {
        if (document.activeElement !== el) el.value = Math.round(s[kind][Number(el.dataset.k)]);
      });
    }
    if (document.activeElement !== approachEl) {
      approachEl.value = mission?.running && mission.goal ? Math.round(mission.goal.approach_m * 1000)
        : remote ? remote.approach : job.approach;
    }
  }

  function problems() {
    const out = {};
    for (const kind of ["pick", "place"]) {
      const [x, y, z] = job[kind];
      if (![x, y, z].every(Number.isFinite)) out[kind] = "enter x, y and z";
      else if (z < Z_MIN || z > Z_MAX) out[kind] = `z must be ${Z_MIN}–${Z_MAX} mm`;
      else out[kind] = whyNot(x, y, z, job.approach);
    }
    if (!Number.isFinite(job.approach) || job.approach < APPROACH_MIN || job.approach > APPROACH_MAX) {
      out.approach = `approach must be ${APPROACH_MIN}–${APPROACH_MAX} mm`;
    }
    if (!out.pick && !out.place && Math.hypot(...job.pick.map((v, i) => v - job.place[i])) < 10) {
      out.place = "same point as the pick";
    }
    return out;
  }

  // Soft warnings: the lab bench of the Gazebo world, not checked by the page
  function warning(kind) {
    const [x, y] = job[kind];
    if (x < TABLE_NEAR_X) return "off the table";
    if (y > TABLE_GAP[0] - 10 && y < TABLE_GAP[1] + 10) return "on the gap between the tables";
    return null;
  }

  function drawChecks() {
    const p = problems();
    for (const kind of ["pick", "place"]) {
      const el = fields[kind].querySelector(".pp-check");
      const w = warning(kind);
      el.innerHTML = p[kind] ? `<span class="pill hot">${esc(p[kind])}</span>`
        : w ? `<span class="pill warn">⚠ ${esc(w)}</span>` : '<span class="pill ok">reachable</span>';
      fields[kind].classList.toggle("bad", !!p[kind]);
    }
    return p;
  }

  function changed() {
    save(job);
    shareUi({ view: "pick", pick: job.pick, place: job.place, approach: job.approach });
    updateEnabled();
    redraw();
  }

  for (const kind of ["pick", "place"]) {
    fields[kind].querySelectorAll("input").forEach((el) => {
      el.addEventListener("input", () => {
        job[kind][Number(el.dataset.k)] = el.value === "" ? NaN : Number(el.value);
        changed();
      });
    });
  }
  approachEl.addEventListener("input", () => {
    job.approach = approachEl.value === "" ? NaN : Number(approachEl.value);
    changed();
  });

  function setArmed(kind) {
    armed = kind;
    root.querySelectorAll("[data-armed]").forEach((b) => b.classList.toggle("active", b.dataset.armed === kind));
  }
  root.querySelectorAll("[data-armed]").forEach((b) => b.addEventListener("click", () => setArmed(b.dataset.armed)));

  // ---- the map: click to set, drag a marker to move it ---------------------------------
  const editable = () => operator && !mission?.running;
  function eventMm(e) {
    const r = canvas.getBoundingClientRect();
    return toMm(e.clientX - r.left, e.clientY - r.top);
  }
  function near(mm) {
    for (const kind of ["pick", "place"]) {
      if (Math.hypot(mm[0] - job[kind][0], mm[1] - job[kind][1]) < 14 / scale) return kind;
    }
    return null;
  }
  function place(kind, [x, y]) {
    job[kind][0] = Math.round(x);
    job[kind][1] = Math.round(y);
    writeFields();
    changed();
  }

  canvas.addEventListener("pointerdown", (e) => {
    if (!editable()) return;
    const mm = eventMm(e);
    const hit = near(mm);
    if (hit) {
      drag = hit;
      canvas.setPointerCapture(e.pointerId);
      return;
    }
    place(armed, mm);
    if (armed === "pick") setArmed("place");
  });
  canvas.addEventListener("pointermove", (e) => {
    const mm = eventMm(e);
    hover = mm;
    if (drag) place(drag, mm);
    canvas.style.cursor = !editable() ? "default" : drag || near(mm) ? "grab" : "crosshair";
    const z = job[armed][2];
    const why = whyNot(mm[0], mm[1], z, job.approach);
    $("#pp-readout").innerHTML = `x ${fmt(mm[0], 0)} · y ${fmt(mm[1], 0)} mm · `
      + (why ? `<span style="color:var(--hot)">${esc(why)} at z ${Math.round(z)}</span>` : `<span style="color:var(--ok)">reachable at z ${Math.round(z)}</span>`);
    redraw();
  });
  const endDrag = () => { drag = null; redraw(); };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);
  canvas.addEventListener("pointerleave", () => {
    hover = null;
    $("#pp-readout").textContent = "";
    redraw();
  });

  // ---- commands ------------------------------------------------------------------------
  const msg = (html) => { $("#pp-msg").innerHTML = html; };

  $("#pp-run").addEventListener("click", async () => {
    if (Object.values(drawChecks()).some(Boolean)) return;
    const [p, q] = [job.pick, job.place];
    if (!confirm(`Run pick & place?\n\nPick at (${p.join(", ")}) mm, place at (${q.join(", ")}) mm, approach ${job.approach} mm.\nThe arm moves on its own until the goal ends. Keep the area clear and STOP at hand.`)) return;
    const m = (v) => v / 1000;
    const r = await post("/api/pick_place", {
      pick: { x: m(p[0]), y: m(p[1]), z: m(p[2]) },
      place: { x: m(q[0]), y: m(q[1]), z: m(q[2]) },
      approach_m: m(job.approach),
    });
    msg(r.ok ? `<span class="pill ok">goal accepted · ${new Date().toLocaleTimeString("en-GB")}</span>`
      : `<span class="pill hot">Failed: ${esc(r.error)}</span>`);
  });
  $("#pp-cancel").addEventListener("click", async () => {
    const r = await post("/api/pick_place/cancel");
    if (!r.ok) msg(`<span class="pill hot">Failed: ${esc(r.error)}</span>`);
  });
  $("#pp-stop").addEventListener("click", () => post("/api/stop"));   // no confirmation, ever

  // ---- state -----------------------------------------------------------------------------
  function drawLease() {
    const box = $("#pp-lease");
    const holder = holderName();
    if (mission?.running) {
      box.className = "lease-banner taken";
      box.innerHTML = `<span>🤖 <b>braco_mission is moving the arm</b> for ${esc(mission.by || "someone")}. STOP still works for everybody.</span>`;
    } else if (operator) {
      box.className = "lease-banner mine";
      box.innerHTML = "<span><b>You are in control.</b> Click the map to set the pick, then the place; drag them to adjust.</span>";
    } else if (holder) {
      box.className = "lease-banner taken";
      box.innerHTML = `<span>👁 <b>View only:</b> ${esc(holder)} is in control; their points are shown live.</span>`;
    } else if (outsideOwner()) {
      const who = OWNER_NAMES[outsideOwner()] || outsideOwner();
      box.className = "lease-banner taken";
      box.innerHTML = `<span>🎮 <b>${esc(who)} is in control</b> of the arm.</span>`;
    } else {
      box.className = "lease-banner";
      box.innerHTML = '<span>Nobody is in control. Take control to choose the points and run.</span><button id="pp-take">Take control</button>';
      box.querySelector("#pp-take").addEventListener("click", takeControl);
    }
  }

  function updateEnabled() {
    const p = drawChecks();
    const running = !!mission?.running;
    const s = telemetry || {};
    const can = operator && !running;
    root.querySelectorAll(".pp-point input, #pp-approach, [data-armed]").forEach((el) => { el.disabled = !can; });
    const ready = can && mission?.available && s.connected && !s.stopped && !s.resting
      && !Object.values(p).some(Boolean);
    $("#pp-run").disabled = !ready;
    $("#pp-run").title = !operator ? "take control first" : running ? "a goal is running"
      : !mission?.available ? (mission?.error || "braco_mission is not running")
      : !s.connected ? "the arm is not connected" : s.stopped ? "STOP latched" : s.resting ? "the arm is resting: wake it up on the Control page"
      : Object.values(p).find(Boolean) || "";
    $("#pp-cancel").disabled = !(operator && running);
    $("#pp-stop").disabled = !s.connected;
    $("#pp-stopped").hidden = !s.stopped;
    root.querySelector(".pp-map-card").classList.toggle("view-only", !editable());
    drawLease();
  }

  function drawMission() {
    const m = mission;
    const err = $("#pp-error");
    err.innerHTML = m && !m.available && m.error ? `<div class="error-box">${esc(m.error)}</div>` : "";
    const stages = m?.stages || Object.keys(STAGE_LABELS);
    const r = m?.result;
    const current = m?.running ? m.stage : null;
    const failed = !m?.running && r && !r.success ? r.failed_stage : null;
    const reached = current ? stages.indexOf(current) : failed ? stages.indexOf(failed) : r?.success ? stages.length : -1;
    $("#pp-stages").innerHTML = stages.map((st, i) => {
      const [name, hint] = STAGE_LABELS[st] || [st, ""];
      const cls = st === failed ? "failed" : st === current ? "current" : i < reached ? "done" : "";
      return `<li class="${cls}"><b>${esc(name)}</b>${hint ? `<span class="muted small">${esc(hint)}</span>` : ""}</li>`;
    }).join("");
    $("#pp-who").textContent = m?.running ? `running for ${m.by || "someone"}${m.stage === "sending" ? " · sending" : ""}`
      : m?.finished ? `last run by ${m.by || "someone"} at ${new Date(m.finished * 1000).toLocaleTimeString("en-GB")}` : "";
    if (!r || m.running) {
      $("#pp-result").innerHTML = m?.running ? "" : '<p class="muted small">No goal yet. The stages show here as braco_mission reports them.</p>';
      return;
    }
    const mm = (v) => (v == null ? "—" : `${fmt(v * 1000, 1)} mm`);
    const deg = (v) => (v == null ? "—" : `${fmt((v * 180) / Math.PI, 2)}°`);
    const took = m.started && m.finished ? `${fmt(m.finished - m.started, 0)} s` : "—";
    $("#pp-result").innerHTML = `
      <div class="pp-result ${r.success ? "ok" : "bad"}">
        <b>${r.success ? "✓ Done" : r.cancelled ? `■ Cancelled at ${esc(r.failed_stage)}` : `✕ Failed at ${esc(r.failed_stage || "?")}`}</b>
        <span class="muted small">${esc(r.message || "")}</span>
      </div>
      <div class="kv">
        <div><span>pick error</span><b>${mm(r.pick_position_error_m)}</b></div>
        <div><span>place error</span><b>${mm(r.place_position_error_m)}</b></div>
        <div><span>took</span><b>${took}</b></div>
        <div><span>pick tilt</span><b>${deg(r.pick_orientation_error_rad)}</b></div>
        <div><span>place tilt</span><b>${deg(r.place_orientation_error_rad)}</b></div>
      </div>`;
  }

  function drawBadges() {
    const s = telemetry || {};
    $("#pp-badges").innerHTML = [
      !s.connected ? '<span class="pill hot">arm disconnected</span>'
        : s.source === "gazebo" ? '<span class="pill info">Gazebo simulation</span>'
        : s.mock ? '<span class="pill info">mock arm</span>' : '<span class="pill ok">arm connected</span>',
      mission?.available ? `<span class="pill ok">braco_mission ready${mission.mock ? " (mock)" : ""}</span>`
        : '<span class="pill warn">braco_mission not running</span>',
      s.stopped ? '<span class="pill hot">STOP latched</span>' : "",
      s.resting ? '<span class="pill warn">resting</span>' : "",
    ].join("");
  }

  function applyRemote(ui) {
    const st = ui?.state;
    remote = !operator && st?.view === "pick" && Array.isArray(st.pick) && Array.isArray(st.place) ? st : null;
    writeFields();
    redraw();
  }

  const offs = [];
  offs.push(on("telemetry", (s) => {
    const before = telemetry;
    telemetry = s;
    if (!before || before.connected !== s.connected || before.stopped !== s.stopped
        || before.resting !== s.resting || before.control_owner !== s.control_owner) {
      updateEnabled();
      drawBadges();
    }
    redraw();
  }));
  offs.push(on("mission", (m) => {
    const wasRunning = mission?.running;
    mission = m;
    if (wasRunning && !m.running) writeFields();
    drawMission();
    drawBadges();
    updateEnabled();
    writeFields();
    redraw();
  }));
  offs.push(on("lease", () => {
    const now = isOperator();
    if (now !== operator) {
      operator = now;
      if (operator) changed();
    }
    applyRemote(live.ui);
    updateEnabled();
  }));
  offs.push(on("ui", applyRemote));
  const onTheme = () => { readColors(); reachKey = ""; redraw(); };
  window.addEventListener("themechange", onTheme);
  const media = matchMedia("(prefers-color-scheme: dark)");
  media.addEventListener("change", onTheme);

  readColors();
  writeFields();
  drawMission();
  drawBadges();
  updateEnabled();
  resize();

  return () => {
    offs.forEach((off) => off());
    observer.disconnect();
    window.removeEventListener("themechange", onTheme);
    media.removeEventListener("change", onTheme);
    if (frame) cancelAnimationFrame(frame);
  };
}
