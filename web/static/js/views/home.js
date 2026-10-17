// Overview: arm summary, current sprint, quick links.

import { esc, fmt, fmtDate, getJSON, onTelemetry } from "../util.js";
import { JOINTS } from "../specs.js";
import { currentMilestone, sprintProgress } from "./sprints.js";

const QUICK = [
  ["#/docs/docs/dev-setup.md", "Setup de desenvolvimento", "dev-setup.md"],
  ["#/docs/docs/safety.md", "Regras de segurança", "safety.md"],
  ["#/docs/docs/protocol_commands.md", "Comandos do protocolo", "protocol_commands.md"],
  ["#/docs/docs/inventory/inventory.md", "Inventário de hardware", "inventory.md"],
  ["#/docs/docker/README.md", "Containers ROS 2", "docker/README.md"],
  ["#/arm", "Referência de comandos", "códigos, limites, PID"],
];

export function render(root) {
  root.innerHTML = `
    <div class="page-head">
      <div>
        <h1>Portal do Braço</h1>
        <p>myCobot 280 Jetson Nano · tudo que o time precisa num só lugar</p>
      </div>
      <span class="pill" id="h-updated">—</span>
    </div>
    <div class="hero">
      <section class="card">
        <h2>Braço agora <a href="#/arm" class="small">abrir painel →</a></h2>
        <div class="stat-row" style="margin-bottom:14px">
          <div class="stat"><b id="h-conn">—</b><span>conexão</span></div>
          <div class="stat"><b id="h-power">—</b><span>servos</span></div>
          <div class="stat"><b id="h-moving">—</b><span>movimento</span></div>
          <div class="stat"><b id="h-temp">—</b><span>servo mais quente</span></div>
        </div>
        <div class="mini-joints" id="h-joints">
          ${JOINTS.map((j) => `<div><b>—</b><span>${j}</span></div>`).join("")}
        </div>
      </section>
      <section class="card" id="h-sprint">
        <h2>Sprint atual <a href="#/sprints" class="small">ver sprints →</a></h2>
        <div class="empty">Carregando do GitHub…</div>
      </section>
    </div>
    <section class="card">
      <h2>Acesso rápido</h2>
      <div class="quick">
        ${QUICK.map(([href, title, sub]) => `<a href="${href}"><b>${esc(title)}</b><small>${esc(sub)}</small></a>`).join("")}
      </div>
    </section>
  `;

  const $ = (id) => root.querySelector(id);
  const cells = [...root.querySelectorAll("#h-joints b")];

  const off = onTelemetry((s) => {
    $("#h-conn").textContent = !s.connected ? "off" : s.mock ? "mock" : "on";
    $("#h-power").textContent = s.power_on == null ? "—" : s.power_on ? "ligados" : "soltos";
    $("#h-moving").textContent = s.moving == null ? "—" : s.moving ? "sim" : "parado";
    if (s.temps) {
      const max = Math.max(...s.temps);
      const level = s.temp_levels[s.temps.indexOf(max)];
      const el = $("#h-temp");
      el.textContent = `${max}°C`;
      el.className = `temp-cell ${level}`;
    }
    (s.angles || []).forEach((a, i) => { cells[i].textContent = `${fmt(a, 0)}°`; });
    $("#h-updated").textContent = s.updated
      ? `atualizado ${new Date(s.updated * 1000).toLocaleTimeString("pt-BR")}` : "—";
  });

  getJSON("/api/sprints").then((data) => {
    const box = $("#h-sprint");
    const ms = currentMilestone(data.milestones);
    const sprint = data.sprints.find((s) => s.state === "open") || data.sprints[0];
    if (!sprint && !ms) {
      box.querySelector(".empty").textContent = data.error || "Nenhum sprint encontrado.";
      return;
    }
    const p = sprint ? sprintProgress(sprint) : null;
    const days = ms?.due_on ? Math.ceil((new Date(ms.due_on) - Date.now()) / 86400000) : null;
    box.innerHTML = `
      <h2>Sprint atual <a href="#/sprints" class="small">ver sprints →</a></h2>
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
          <div><div>${days != null && days < 0 ? "dias (prazo passou)" : "dias até a entrega"}</div>
          <div class="muted small">${esc(ms.title)} · ${fmtDate(ms.due_on)}</div></div>
        </div>` : ""}
    `;
  }).catch((e) => {
    $("#h-sprint .empty").textContent = `Erro ao carregar sprints: ${e.message}`;
  });

  return off;
}
