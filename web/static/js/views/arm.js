// Live arm panel: 3D model, joints, temperatures, history, STOP, reference.

import { esc, fmt, onTelemetry } from "../util.js";
import {
  COMMANDS, CONNECTION, FACTORY_PID, FIRMWARE_LIMITS, JOINTS, JOINT_NAMES,
  SOFT_LIMITS, TEMP_HOT_C, TEMP_WARM_C,
} from "../specs.js";

const JC = ["--j1", "--j2", "--j3", "--j4", "--j5", "--j6"];
const HISTORY_S = 60;
const LEVEL_TEXT = { normal: "ok", warm: "morno", hot: "QUENTE" };

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
      <div class="range" title="faixa amarela = soft limit; barra inteira = limite de firmware">
        <div class="soft" style="left:${pct(slo, i)}%;right:${100 - pct(shi, i)}%"></div>
        <div class="zero" style="left:${pct(0, i)}%"></div>
        <div class="needle" style="left:${pct(0, i)}%"></div>
      </div>
      <div class="range-labels"><span>${flo}°</span><span>${slo}° … ${shi}°</span><span>${fhi}°</span></div>
      <div class="temp"><span class="muted">temperatura</span><span class="temp-cell tval">—</span></div>
    </div>`;
}

function referenceHTML() {
  return `
    <div class="grid cols-2" style="margin-top:16px">
      <section class="card span-2">
        <h2>Comandos do protocolo <span class="muted small" style="text-transform:none;letter-spacing:0">libs/cpp/includes/mycobot.hpp · pymycobot</span></h2>
        <div class="table-wrap"><table>
          <thead><tr><th>Código</th><th>Nome</th><th>Python</th><th>C++</th><th>Descrição</th><th></th></tr></thead>
          <tbody>${COMMANDS.map((c) => `
            <tr><td class="mono">${c.code}</td><td class="mono">${c.name}</td>
            <td><code>${esc(c.py)}</code></td><td><code>${esc(c.cpp)}</code></td>
            <td>${esc(c.desc)}</td>
            <td>${c.write ? '<span class="pill warn rw">escreve</span>' : '<span class="pill ok rw">leitura</span>'}</td></tr>`).join("")}
          </tbody></table></div>
      </section>
      <section class="card">
        <h2>Limites e PID de fábrica</h2>
        <div class="table-wrap"><table>
          <thead><tr><th>Junta</th><th>Firmware</th><th>Soft</th><th>P</th><th>I</th><th>D</th></tr></thead>
          <tbody>${JOINTS.map((j, i) => `
            <tr><td><b>${j}</b> <span class="muted small">${JOINT_NAMES[i]}</span></td>
            <td class="mono">${FIRMWARE_LIMITS[i].join(" … ")}</td>
            <td class="mono">${SOFT_LIMITS[i].join(" … ")}</td>
            ${FACTORY_PID[i].map((v) => `<td class="mono">${v}</td>`).join("")}</tr>`).join("")}
          </tbody></table></div>
        <p class="muted small">setPid() aceita P e D até 1,5× o valor de fábrica e I até 8.</p>
      </section>
      <section class="card">
        <h2>Conexão e unidades</h2>
        <table><tbody>${CONNECTION.map(([k, v]) => `<tr><th>${k}</th><td class="mono">${esc(v)}</td></tr>`).join("")}</tbody></table>
        <p class="muted small">Temperatura: normal &lt; ${TEMP_WARM_C}°C · morno ≥ ${TEMP_WARM_C}°C · quente ≥ ${TEMP_HOT_C}°C (servos se protegem ~70°C).</p>
      </section>
    </div>`;
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

export function render(root) {
  root.innerHTML = `
    <div class="page-head">
      <div>
        <h1>Braço</h1>
        <p>Telemetria ao vivo (somente leitura) · myCobot 280 em <span class="mono" id="a-port">/dev/ttyTHS1</span></p>
      </div>
      <div class="filters" id="a-badges"></div>
    </div>
    <div class="arm-top">
      <section class="card viewer" id="a-viewer">
        <div class="overlay" id="a-overlay"></div>
        <div class="hint">arraste para girar · role para zoom</div>
      </section>
      <section class="card">
        <h2>Juntas</h2>
        <div class="joints">${JOINTS.map((_, i) => jointCard(i)).join("")}</div>
      </section>
    </div>
    <div class="grid cols-3">
      <section class="card">
        <h2>Pose cartesiana</h2>
        <div class="kv" id="a-coords">
          ${["X", "Y", "Z", "RX", "RY", "RZ"].map((k) => `<div><span>${k} ${k.length === 1 ? "mm" : "°"}</span><b>—</b></div>`).join("")}
        </div>
      </section>
      <section class="card">
        <h2>Histórico (${HISTORY_S}s)
          <div class="tabs" id="a-tabs"><button class="active" data-k="angles">ângulos</button><button data-k="temps">temperaturas</button></div>
        </h2>
        <canvas class="chart" id="a-chart"></canvas>
        <div class="legend">${JOINTS.map((j, i) => `<span><i style="background:var(${JC[i]})"></i>${j}</span>`).join("")}</div>
      </section>
      <section class="card">
        <h2>Parada</h2>
        <button class="stop-btn" id="a-stop">■ STOP</button>
        <p class="muted small" id="a-stop-msg">Envia <code>mc.stop()</code>: interrompe o movimento atual. É o único comando que esta página envia ao braço.</p>
        <div id="a-error"></div>
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
    .catch(() => { $("#a-viewer").insertAdjacentHTML("beforeend", '<div class="empty">Modelo 3D indisponível (sem acesso ao CDN?)</div>'); });

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

  $("#a-stop").addEventListener("click", async () => {
    if (!confirm("Enviar STOP ao braço?")) return;
    const msg = $("#a-stop-msg");
    try {
      const res = await fetch("/api/stop", { method: "POST" });
      const body = await res.json();
      msg.innerHTML = res.ok
        ? `<span class="pill ok">STOP enviado ${new Date().toLocaleTimeString("pt-BR")}</span>`
        : `<span class="pill hot">Falhou: ${esc(body.detail)}</span>`;
    } catch (e) {
      msg.innerHTML = `<span class="pill hot">Falhou: ${esc(e.message)}</span>`;
    }
  });

  const off = onTelemetry((s) => {
    $("#a-badges").innerHTML = [
      !s.connected ? '<span class="pill hot">desconectado</span>'
        : s.mock ? '<span class="pill info">modo simulado</span>' : '<span class="pill ok">conectado</span>',
      s.power_on == null ? "" : s.power_on ? '<span class="pill">servos ligados</span>' : '<span class="pill warn">servos soltos</span>',
      s.moving == null ? "" : s.moving ? '<span class="pill accent">em movimento</span>' : '<span class="pill">parado</span>',
    ].join("");
    $("#a-overlay").innerHTML = s.angles ? "" : '<span class="pill">sem leitura de ângulos</span>';
    $("#a-error").innerHTML = s.error ? `<div class="error-box">${esc(s.error)}</div>` : "";
    $("#a-stop").disabled = !s.connected;

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
  });

  const onTheme = () => redraw();
  window.addEventListener("themechange", onTheme);

  return () => {
    alive = false;
    off();
    window.removeEventListener("themechange", onTheme);
    arm3d?.dispose();
  };
}
