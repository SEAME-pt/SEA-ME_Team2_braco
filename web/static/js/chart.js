// Joint history chart: angles, temperatures and servo voltage over time.
//
// The history is recorded here, from the first telemetry on, whatever page is open, so
// switching views does not empty it. The chart has one panel per metric (never two
// scales on one axis): a single metric big, or all three stacked on one shared time axis
// so a move, a temperature rise and a voltage sag can be read at the same instant.

import { esc, fmt, on } from "./util.js";
import { JOINTS, JOINT_NAMES, TEMP_HOT_C, TEMP_WARM_C } from "./specs.js";

const JC = ["--j1", "--j2", "--j3", "--j4", "--j5", "--j6"];
const SAMPLE_EVERY_S = 0.5;
const KEEP_S = 30 * 60;
const WINDOWS = [[60, "1 min"], [300, "5 min"], [900, "15 min"], [1800, "30 min"]];

const METRICS = {
  angles: { title: "Angle", unit: "°", key: "angles", digits: 1, minSpan: 20, pad: 0.1 },
  temps: { title: "Temperature", unit: "°C", key: "temps", digits: 0, minSpan: 10, pad: 0.15,
    lines: [[TEMP_WARM_C, "warm", "--warn"], [TEMP_HOT_C, "hot", "--hot"]] },
  volts: { title: "Voltage", unit: " V", key: "volts", digits: 1, minSpan: 1, pad: 0.2 },
};
const MODES = [["angles", "Angle"], ["temps", "Temperature"], ["volts", "Voltage"], ["all", "All"]];

// ---- Recording ------------------------------------------------------------------

const samples = [];
let lastSample = 0;
const subscribers = new Set();

on("telemetry", (s) => {
  const now = Date.now() / 1000;
  if (now - lastSample < SAMPLE_EVERY_S) return;
  lastSample = now;
  samples.push({ t: now, angles: s.angles, temps: s.temps, volts: s.volts });
  while (samples.length && samples[0].t < now - KEEP_S) samples.shift();
  subscribers.forEach((fn) => fn());
});

// ---- Helpers --------------------------------------------------------------------

function niceStep(span, target) {
  const raw = span / target;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const f = raw / mag;
  return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * mag;
}

function timeLabel(ago) {
  if (ago <= 0) return "now";
  return ago >= 120 ? `−${Math.round(ago / 60)} min` : `−${Math.round(ago)} s`;
}

// First sample index with t >= x
function lowerBound(arr, x) {
  let lo = 0, hi = arr.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (arr[mid].t < x) lo = mid + 1; else hi = mid;
  }
  return lo;
}

// ---- The chart ------------------------------------------------------------------

export function mountChart(root) {
  const state = {
    mode: "angles", windowS: 60, paused: false, pausedAt: 0,
    hidden: new Set(), hover: null,   // hover: x in CSS px inside the plot, or null
  };

  root.innerHTML = `
    <div class="hc-bar">
      <div class="tabs" data-role="mode">${MODES.map(([k, l]) => `<button data-k="${k}">${l}</button>`).join("")}</div>
      <div class="tabs" data-role="window">${WINDOWS.map(([s, l]) => `<button data-s="${s}">${l}</button>`).join("")}</div>
      <button class="hc-pause" data-role="pause" title="Freeze the chart to inspect it (recording continues)">❚❚ Pause</button>
    </div>
    <div class="hc-wrap"><canvas class="hc-canvas"></canvas><div class="hc-tip" hidden></div></div>
    <div class="hc-legend"></div>
    <p class="muted small hc-note"></p>`;

  const $ = (s) => root.querySelector(s);
  const canvas = $(".hc-canvas");
  const tip = $(".hc-tip");
  const legend = $(".hc-legend");

  const metricsShown = () => (state.mode === "all" ? ["angles", "temps", "volts"] : [state.mode]);
  const viewEnd = () => (state.paused ? state.pausedAt : Date.now() / 1000);

  // ---- layout -----------------------------------------------------------------
  const PAD = { l: 52, r: 12, t: 8, b: 22 };
  function layout(w, h) {
    const shown = metricsShown();
    const gap = 14;
    const ph = (h - PAD.t - PAD.b - gap * (shown.length - 1)) / shown.length;
    return shown.map((m, i) => ({ m, x: PAD.l, y: PAD.t + i * (ph + gap), w: w - PAD.l - PAD.r, h: ph }));
  }

  function visible(metric, from, to) {
    const out = [];
    const i0 = lowerBound(samples, from);
    for (let i = i0; i < samples.length && samples[i].t <= to; i++) out.push(samples[i]);
    return { points: out, key: METRICS[metric].key };
  }

  function range(metric, points) {
    const cfg = METRICS[metric];
    let lo = Infinity, hi = -Infinity;
    for (const p of points) {
      const arr = p[cfg.key];
      if (!arr) continue;
      arr.forEach((v, j) => {
        if (v == null || state.hidden.has(j)) return;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      });
    }
    if (!Number.isFinite(lo)) { lo = 0; hi = cfg.minSpan; }
    if (cfg.lines) {   // keep the nearest threshold in view when a joint approaches it
      for (const [v] of cfg.lines) if (hi >= v - 8) hi = Math.max(hi, v + 2);
    }
    const span = Math.max(hi - lo, cfg.minSpan);
    const mid = (hi + lo) / 2;
    lo = mid - span / 2 - span * cfg.pad / 2;
    hi = mid + span / 2 + span * cfg.pad / 2;
    return [lo, hi];
  }

  // ---- drawing ----------------------------------------------------------------
  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const css = getComputedStyle(document.documentElement);
    const col = (n) => css.getPropertyValue(n).trim();
    const end = viewEnd(), start = end - state.windowS;
    const tx = (t, p) => p.x + ((t - start) / state.windowS) * p.w;
    const panels = layout(w, h);
    state.panels = panels;
    state.geom = { start, end };

    ctx.font = `10px ${col("--mono")}`;
    for (const p of panels) {
      const cfg = METRICS[p.m];
      const { points } = visible(p.m, start, end);
      const [lo, hi] = range(p.m, points);
      p.lo = lo; p.hi = hi;
      const ty = (v) => p.y + p.h - ((v - lo) / (hi - lo)) * p.h;

      // y grid and labels
      const step = niceStep(hi - lo, Math.max(2, p.h / 40));
      ctx.textAlign = "right"; ctx.textBaseline = "middle";
      ctx.lineWidth = 1;
      for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
        const y = Math.round(ty(v)) + 0.5;
        ctx.strokeStyle = col("--border");
        ctx.beginPath(); ctx.moveTo(p.x, y); ctx.lineTo(p.x + p.w, y); ctx.stroke();
        ctx.fillStyle = col("--muted");
        ctx.fillText(fmt(Math.round(v * 100) / 100), p.x - 6, y);
      }
      // y axis unit
      ctx.save();
      ctx.translate(10, p.y + p.h / 2); ctx.rotate(-Math.PI / 2);
      ctx.textAlign = "center"; ctx.fillStyle = col("--muted");
      ctx.fillText(`${cfg.title} (${cfg.unit.trim()})`, 0, 0);
      ctx.restore();

      // threshold lines (temperature)
      for (const [v, label, token] of cfg.lines || []) {
        if (v < lo || v > hi) continue;
        const y = Math.round(ty(v)) + 0.5;
        ctx.strokeStyle = col(token); ctx.setLineDash([5, 4]); ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(p.x, y); ctx.lineTo(p.x + p.w, y); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = col("--muted"); ctx.textAlign = "right"; ctx.textBaseline = "bottom";
        ctx.fillText(`${label} ${v}°C`, p.x + p.w - 4, y - 2);
      }

      // series: one line per joint, a gap where there is no data
      ctx.save();
      ctx.beginPath(); ctx.rect(p.x, p.y - 2, p.w, p.h + 4); ctx.clip();
      for (let j = 0; j < 6; j++) {
        if (state.hidden.has(j)) continue;
        ctx.strokeStyle = col(JC[j]); ctx.lineWidth = 1.8; ctx.lineJoin = "round";
        ctx.beginPath();
        let pen = false, prev = 0;
        for (const s of points) {
          const v = s[cfg.key]?.[j];
          if (v == null || (pen && s.t - prev > 3)) { pen = false; if (v == null) continue; }
          const x = tx(s.t, p), y = ty(v);
          if (!pen) { ctx.moveTo(x, y); pen = true; } else ctx.lineTo(x, y);
          prev = s.t;
        }
        ctx.stroke();
      }
      ctx.restore();

      // empty hint
      if (!points.some((s) => s[cfg.key])) {
        ctx.fillStyle = col("--muted"); ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText(p.m === "volts" ? "no voltage reading (needs the updated arm_driver)" : "no data yet",
          p.x + p.w / 2, p.y + p.h / 2);
      }
      // plot border (left axis line)
      ctx.strokeStyle = col("--border");
      ctx.beginPath(); ctx.moveTo(p.x + 0.5, p.y); ctx.lineTo(p.x + 0.5, p.y + p.h); ctx.stroke();
    }

    // time axis under the last panel
    const last = panels[panels.length - 1];
    ctx.fillStyle = col("--muted"); ctx.textBaseline = "top";
    const tstep = niceStep(state.windowS, 6);
    for (let a = 0; a <= state.windowS; a += tstep) {
      const x = last.x + last.w - (a / state.windowS) * last.w;
      ctx.textAlign = a === 0 ? "right" : a >= state.windowS - tstep / 2 ? "left" : "center";
      ctx.fillText(timeLabel(a + (Date.now() / 1000 - end)), x, last.y + last.h + 6);
      panels.forEach((p) => {
        ctx.strokeStyle = col("--border"); ctx.globalAlpha = .5;
        ctx.beginPath(); ctx.moveTo(Math.round(x) + 0.5, p.y + p.h); ctx.lineTo(Math.round(x) + 0.5, p.y + p.h + 3); ctx.stroke();
        ctx.globalAlpha = 1;
      });
    }

    // crosshair on every panel, same instant
    if (state.hover != null) {
      const x = state.hover;
      ctx.strokeStyle = col("--muted"); ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(Math.round(x) + 0.5, panels[0].y); ctx.lineTo(Math.round(x) + 0.5, last.y + last.h); ctx.stroke();
      ctx.setLineDash([]);
      const t = start + ((x - last.x) / last.w) * state.windowS;
      const s = nearest(t);
      if (s) {
        for (const p of panels) {
          const cfg = METRICS[p.m];
          for (let j = 0; j < 6; j++) {
            const v = s[cfg.key]?.[j];
            if (v == null || state.hidden.has(j)) continue;
            const mx = p.x + ((s.t - start) / state.windowS) * p.w;
            const my = p.y + p.h - ((v - p.lo) / (p.hi - p.lo)) * p.h;
            ctx.fillStyle = col(JC[j]); ctx.strokeStyle = col("--surface"); ctx.lineWidth = 2;
            ctx.beginPath(); ctx.arc(mx, my, 3.5, 0, Math.PI * 2); ctx.stroke(); ctx.fill();
          }
        }
      }
    }
  }

  function nearest(t) {
    const i = lowerBound(samples, t);
    const a = samples[i - 1], b = samples[i];
    if (!a) return b;
    if (!b) return a;
    return t - a.t < b.t - t ? a : b;
  }

  // ---- tooltip ----------------------------------------------------------------
  function showTip(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const x = clientX - rect.left;
    const last = state.panels?.[state.panels.length - 1];
    if (!last || x < last.x || x > last.x + last.w) return hideTip();
    state.hover = x;
    const t = state.geom.start + ((x - last.x) / last.w) * state.windowS;
    const s = nearest(t);
    if (!s) return hideTip();
    const ago = Date.now() / 1000 - s.t;
    const rows = metricsShown().map((m) => {
      const cfg = METRICS[m];
      const cells = JOINTS.map((name, j) => {
        if (state.hidden.has(j)) return "";
        const v = s[cfg.key]?.[j];
        return `<div class="hc-row"><i style="background:var(${JC[j]})"></i><span>${name}</span><b>${v == null ? "—" : fmt(Number(v).toFixed(cfg.digits)) + cfg.unit}</b></div>`;
      }).join("");
      return `<div class="hc-group"><div class="hc-gtitle">${cfg.title}</div>${cells}</div>`;
    }).join("");
    tip.innerHTML = `<div class="hc-time">${ago < 1 ? "now" : timeLabel(ago)} · ${new Date(s.t * 1000).toLocaleTimeString("en-GB")}</div>${rows}`;
    tip.hidden = false;
    const wrap = $(".hc-wrap").getBoundingClientRect();
    const tw = tip.offsetWidth, th = tip.offsetHeight;
    let left = clientX - wrap.left + 14;
    if (left + tw > wrap.width) left = clientX - wrap.left - tw - 14;
    tip.style.left = `${Math.max(0, left)}px`;
    tip.style.top = `${Math.max(0, Math.min(clientY - wrap.top - 10, wrap.height - th))}px`;
    schedule();
  }
  function hideTip() {
    tip.hidden = true;
    if (state.hover != null) { state.hover = null; schedule(); }
  }
  canvas.addEventListener("pointermove", (e) => showTip(e.clientX, e.clientY));
  canvas.addEventListener("pointerleave", hideTip);

  // ---- legend: the live value, the range in the window; click hides, double click isolates
  function drawLegend() {
    const end = viewEnd(), start = end - state.windowS;
    const metric = state.mode === "all" ? "angles" : state.mode;
    const cfg = METRICS[metric];
    const latest = samples[samples.length - 1];
    legend.innerHTML = JOINTS.map((name, j) => {
      let lo = Infinity, hi = -Infinity;
      for (const s of visible(metric, start, end).points) {
        const v = s[cfg.key]?.[j];
        if (v == null) continue;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
      const now = latest?.[cfg.key]?.[j];
      const f = (v) => fmt(Number(v).toFixed(cfg.digits));
      return `<button class="hc-chip${state.hidden.has(j) ? " off" : ""}" data-j="${j}" style="--jc:var(${JC[j]})" title="${esc(JOINT_NAMES[j])}: click to hide/show, double click to show only this joint">
        <i></i><b>${name}</b><span class="mono">${now == null ? "—" : f(now) + cfg.unit}</span>
        <small class="mono muted">${Number.isFinite(lo) ? `${f(lo)} … ${f(hi)}` : ""}</small></button>`;
    }).join("");
    $(".hc-note").textContent =
      `Each chip: current value and the min … max of the ${metric === "angles" ? "angle" : cfg.title.toLowerCase()} in the window. ` +
      "Click a joint to hide it, double click to isolate it. Hover the chart to compare all metrics at the same instant." +
      (state.paused ? " Paused: the chart is frozen, recording continues." : "");
  }

  // ---- controls -----------------------------------------------------------------
  function sync() {
    root.querySelectorAll('[data-role="mode"] button').forEach((b) => b.classList.toggle("active", b.dataset.k === state.mode));
    root.querySelectorAll('[data-role="window"] button').forEach((b) => b.classList.toggle("active", Number(b.dataset.s) === state.windowS));
    const pause = $('[data-role="pause"]');
    pause.textContent = state.paused ? "▶ Resume" : "❚❚ Pause";
    pause.classList.toggle("active", state.paused);
    canvas.style.height = state.mode === "all" ? "560px" : "340px";
    schedule(true);
  }
  $('[data-role="mode"]').addEventListener("click", (e) => { if (e.target.dataset.k) { state.mode = e.target.dataset.k; sync(); } });
  $('[data-role="window"]').addEventListener("click", (e) => { if (e.target.dataset.s) { state.windowS = Number(e.target.dataset.s); sync(); } });
  $('[data-role="pause"]').addEventListener("click", () => {
    state.paused = !state.paused;
    state.pausedAt = Date.now() / 1000;
    sync();
  });
  let clickTimer = null;
  legend.addEventListener("click", (e) => {
    const chip = e.target.closest(".hc-chip");
    if (!chip) return;
    const j = Number(chip.dataset.j);
    clearTimeout(clickTimer);
    clickTimer = setTimeout(() => {   // a double click cancels this one
      state.hidden.has(j) ? state.hidden.delete(j) : state.hidden.add(j);
      schedule(true);
    }, 220);
  });
  legend.addEventListener("dblclick", (e) => {
    const chip = e.target.closest(".hc-chip");
    if (!chip) return;
    clearTimeout(clickTimer);
    const j = Number(chip.dataset.j);
    const only = state.hidden.size === 5 && !state.hidden.has(j);
    state.hidden = new Set(only ? [] : [0, 1, 2, 3, 4, 5].filter((k) => k !== j));
    schedule(true);
  });

  // ---- redraw scheduling --------------------------------------------------------
  let raf = 0, legendDue = true;
  function schedule(withLegend = false) {
    legendDue = legendDue || withLegend;
    if (raf) return;
    raf = requestAnimationFrame(() => {
      raf = 0;
      draw();
      if (legendDue) { legendDue = false; drawLegend(); }
    });
  }
  const onSample = () => { if (!state.paused) schedule(!legend.matches(":hover")); };   // a legend under the cursor is not rebuilt mid-click
  subscribers.add(onSample);
  const onTheme = () => schedule();
  window.addEventListener("themechange", onTheme);
  const ro = new ResizeObserver(() => schedule());
  ro.observe(canvas);

  sync();
  return () => {
    subscribers.delete(onSample);
    window.removeEventListener("themechange", onTheme);
    ro.disconnect();
    cancelAnimationFrame(raf);
    clearTimeout(clickTimer);
  };
}
