// Live arm panel: 3D model, joints, temperatures, history, STOP, reference.

import { esc, fmt, onTelemetry } from "../util.js";
import {
  COMMANDS, CONNECTION, FACTORY_PID, FIRMWARE_LIMITS, JOINTS, JOINT_NAMES,
  SOFT_LIMITS, TEMP_HOT_C, TEMP_WARM_C,
} from "../specs.js";

const JC = ["--j1", "--j2", "--j3", "--j4", "--j5", "--j6"];
const HISTORY_S = 60;
let movementConfirmed = false;   // asked once per page load, kept while switching views
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

function controlRow(i) {
  const [lo, hi] = SOFT_LIMITS[i];
  return `
    <div class="ctl-row" style="--jc:var(${JC[i]})" data-j="${i}">
      <div class="ctl-name"><b>${JOINTS[i]}</b><span class="muted small">${JOINT_NAMES[i]}</span></div>
      <button class="step" data-d="-1" title="diminuir">−</button>
      <input type="range" min="${lo}" max="${hi}" step="0.5" value="0">
      <button class="step" data-d="1" title="aumentar">+</button>
      <input type="number" class="num" min="${lo}" max="${hi}" step="0.5" value="0">
      <button class="go" title="enviar este ângulo">Ir</button>
      <span class="ctl-now mono muted small">—</span>
    </div>`;
}

function controlHTML() {
  return `
    <section class="card ctl" style="margin-bottom:16px">
      <h2>Controle
        <label class="arm-switch"><input type="checkbox" id="c-arm"> <span>liberar movimento</span></label>
      </h2>
      <div class="rest-banner" id="c-rest" hidden>
        <div><b>☾ Braço em repouso</b><span class="muted small">Servos soltos. Os controles ficam bloqueados até o wake up, que religa os servos e leva o braço ao zero.</span></div>
        <button id="c-wake">☀ Wake up</button>
      </div>
      <div class="ctl-top">
        <button class="stop-btn" id="a-stop">■ STOP</button>
        <div class="ctl-poses">
          <button data-pose="park" title="vai para [0, 131, -150, 146, 4.5, 0] e solta os servos">⌂ Repouso</button>
          <button data-pose="zero" title="send_angles([0, 0, 0, 0, 0, 0])">◎ Zerar</button>
        </div>
        <label class="ctl-speed">velocidade <b id="c-speed-v">30</b>
          <input type="range" id="c-speed" min="1" max="100" value="30">
        </label>
        <label class="ctl-speed">passo ±
          <select id="c-step"><option>0.5</option><option>1</option><option selected>5</option><option>10</option></select> °
        </label>
      </div>
      <div class="ctl-rows">${JOINTS.map((_, i) => controlRow(i)).join("")}</div>
      <p class="muted small" id="a-stop-msg">Cada junta é enviada ao soltar o controle (ou com Enter / Ir) e respeita o soft limit. Repouso vai para a pose de descanso e solta os servos ao chegar; Zerar leva todas as juntas a 0°. Em repouso os controles somem e só o Wake up acorda o braço. Marque “liberar movimento” para habilitar.</p>
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

export function render(root) {
  root.innerHTML = `
    <div class="page-head">
      <div>
        <h1>Braço</h1>
        <p>Telemetria ao vivo · myCobot 280 em <span class="mono" id="a-port">/dev/ttyTHS1</span></p>
      </div>
      <div class="filters" id="a-badges"></div>
    </div>
    <div class="arm-top">
      <section class="card viewer" id="a-viewer">
        <div class="overlay" id="a-overlay"></div>
        <div class="viewer-tools">
          <div class="tabs" id="a-views">
            <button data-v="iso" class="active">3D</button><button data-v="front">Frente</button><button data-v="side">Lado</button><button data-v="top">Topo</button>
          </div>
          <div class="tabs" id="a-toggles">
            <button data-t="labels" class="active" title="Nomes das juntas">Juntas</button><button data-t="trail" class="active" title="Caminho percorrido pela ponta">Rastro</button>
          </div>
        </div>
        <div class="hint">arraste para girar · role para zoom · duplo clique volta ao 3D · anel laranja = alcance de 280 mm</div>
      </section>
      <section class="card">
        <h2>Juntas</h2>
        <div class="joints">${JOINTS.map((_, i) => jointCard(i)).join("")}</div>
      </section>
    </div>
    ${controlHTML()}
    <div class="grid cols-2">
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

  $("#a-views").addEventListener("click", (e) => {
    const v = e.target.dataset.v;
    if (!v) return;
    arm3d?.setView(v);
    root.querySelectorAll("#a-views button").forEach((b) => b.classList.toggle("active", b.dataset.v === v));
  });

  $("#a-toggles").addEventListener("click", (e) => {
    const t = e.target.dataset.t;
    if (!t) return;
    const on = !e.target.classList.contains("active");
    e.target.classList.toggle("active", on);
    if (t === "labels") arm3d?.setLabels(on);
    if (t === "trail") arm3d?.setTrail(on);
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
  const rows = [...root.querySelectorAll(".ctl-row")];
  const lastSent = rows.map(() => 0);
  const dragging = rows.map(() => false);
  let connected = false;

  const msg = (html) => { $("#a-stop-msg").innerHTML = html; };
  const stamp = () => new Date().toLocaleTimeString("pt-BR");
  const speed = () => Number(speedEl.value);

  async function post(url, body, label) {
    try {
      const res = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: body ? JSON.stringify(body) : undefined,
      });
      const data = await res.json();
      msg(res.ok ? `<span class="pill ok">${label} ${stamp()}</span>`
        : `<span class="pill hot">Falhou: ${esc(typeof data.detail === "string" ? data.detail : "valor inválido")}</span>`);
      return res.ok;
    } catch (e) {
      msg(`<span class="pill hot">Falhou: ${esc(e.message)}</span>`);
      return false;
    }
  }

  let resting = false;

  function updateEnabled() {
    const on = armBox.checked && connected && !resting;
    root.querySelectorAll(".ctl-rows button, .ctl-rows input, [data-pose]").forEach((el) => { el.disabled = !on; });
    $("#a-stop").disabled = !connected;
    $("#c-rest").hidden = !resting;
    $("#c-wake").disabled = !connected;
    $(".ctl").classList.toggle("is-resting", resting);
  }

  // The one-time question; true when movement may be released.
  function confirmRelease() {
    if (movementConfirmed) return true;
    movementConfirmed = confirm("Liberar os movimentos do braço?\n\nOs controles de junta, Repouso e Zerar passam a mover o braço sem pedir nova confirmação. Mantenha a área ao redor livre.");
    return movementConfirmed;
  }

  armBox.addEventListener("change", () => {
    if (armBox.checked && !confirmRelease()) armBox.checked = false;
    updateEnabled();
  });

  $("#c-wake").addEventListener("click", async () => {
    if (!confirmRelease()) return;
    armBox.checked = true;
    if (await post(`/api/wake?speed=${speed()}`, null, "wake up enviado")) {
      resting = false;   // the next telemetry confirms it
      updateEnabled();
    }
  });
  speedEl.addEventListener("input", () => { $("#c-speed-v").textContent = speedEl.value; });

  $("#a-stop").addEventListener("click", () => {
    if (confirm("Enviar STOP ao braço?")) post("/api/stop", null, "STOP enviado");
  });

  root.querySelectorAll("[data-pose]").forEach((btn) => btn.addEventListener("click", () => {
    const name = btn.dataset.pose;
    post(`/api/pose/${name}?speed=${speed()}`, null, `${name} enviado`);
    rows.forEach((_, i) => { lastSent[i] = Date.now(); });
  }));

  rows.forEach((row, i) => {
    const [lo, hi] = SOFT_LIMITS[i];
    const slider = row.querySelector('input[type="range"]');
    const num = row.querySelector(".num");
    const clamp = (v) => Math.max(lo, Math.min(hi, v));
    const send = (v) => {
      v = clamp(Number.isFinite(v) ? v : 0);
      slider.value = num.value = v;
      lastSent[i] = Date.now();
      post(`/api/joint/${i + 1}`, { angle: v, speed: speed() }, `${JOINTS[i]} → ${fmt(v)}°`);
    };
    slider.addEventListener("pointerdown", () => { dragging[i] = true; });
    slider.addEventListener("input", () => { num.value = slider.value; });
    slider.addEventListener("change", () => { dragging[i] = false; send(Number(slider.value)); });
    num.addEventListener("keydown", (e) => { if (e.key === "Enter") send(Number(num.value)); });
    row.querySelector(".go").addEventListener("click", () => send(Number(num.value)));
    row.querySelectorAll(".step").forEach((b) => b.addEventListener("click", () => {
      send(clamp(Number(num.value) + Number(b.dataset.d) * Number($("#c-step").value)));
    }));
  });

  updateEnabled();

  const off = onTelemetry((s) => {
    $("#a-badges").innerHTML = [
      !s.connected ? '<span class="pill hot">desconectado</span>'
        : s.mock ? '<span class="pill info">modo simulado</span>' : '<span class="pill ok">conectado</span>',
      s.power_on == null ? "" : s.resting ? '<span class="pill warn">em repouso</span>' : s.power_on ? '<span class="pill">servos ligados</span>' : '<span class="pill warn">em repouso</span>',
      s.moving == null ? "" : s.moving ? '<span class="pill accent">em movimento</span>' : '<span class="pill">parado</span>',
    ].join("");
    $("#a-overlay").innerHTML = s.angles ? "" : '<span class="pill">sem leitura de ângulos</span>';
    $("#a-error").innerHTML = s.error ? `<div class="error-box">${esc(s.error)}</div>` : "";
    const nowResting = !!s.resting;
    if (connected !== !!s.connected || resting !== nowResting) {
      connected = !!s.connected;
      resting = nowResting;
      updateEnabled();
    }
    rows.forEach((row, i) => {
      const a = s.angles?.[i];
      row.querySelector(".ctl-now").textContent = a == null ? "—" : `agora ${fmt(a)}°`;
      // follow the real angle until the user moves a control, then wait for the arm to arrive
      if (a != null && !dragging[i] && Date.now() - lastSent[i] > 4000
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
