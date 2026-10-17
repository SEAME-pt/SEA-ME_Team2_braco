// Sprint tracker: milestone timeline and one card per "SPRINT N" issue.

import { esc, fmtDate, getJSON } from "../util.js";
import * as detail from "./sprint-detail.js";

export function sprintProgress(sprint) {
  const total = sprint.sub_issues.length;
  const done = sprint.sub_issues.filter((i) => i.state === "closed").length;
  return { total, done, pct: total ? Math.round((done / total) * 100) : 0 };
}

// First open milestone whose due date has not passed, else the last one.
export function currentMilestone(milestones) {
  const withDate = (milestones || []).filter((m) => m.due_on);
  const now = Date.now();
  return withDate.find((m) => m.state === "open" && new Date(m.due_on) >= now - 86400000)
    || withDate[withDate.length - 1] || null;
}

function timeline(milestones) {
  const ms = milestones.filter((m) => m.due_on);
  if (ms.length < 2) return "";
  const times = ms.map((m) => new Date(m.due_on).getTime());
  const span = times[times.length - 1] - times[0];
  const start = times[0] - span * 0.06;
  const end = times[times.length - 1] + span * 0.06;
  const pos = (t) => ((t - start) / (end - start)) * 100;
  const now = Date.now();
  const nowPct = Math.max(0, Math.min(100, pos(now)));
  const next = currentMilestone(milestones);
  return `
    <div class="timeline">
      <div class="track">
        <div class="done" style="width:${nowPct}%"></div>
        <div class="today" style="left:${nowPct}%"><span>hoje</span></div>
        ${ms.map((m, i) => `
          <div class="ms ${times[i] < now ? "past" : ""} ${m === next ? "next" : ""} ${i > 0 && pos(times[i]) - pos(times[i - 1]) < 10 && i % 2 ? "low" : ""}" style="left:${pos(times[i])}%">
            <i></i>
            <div class="lbl"><a href="${esc(m.url)}" target="_blank" rel="noopener">${esc(m.title.replace("Milestone", "M"))}</a>
            <small>${fmtDate(m.due_on, { day: "2-digit", month: "short" })}</small></div>
          </div>`).join("")}
      </div>
      <div class="timeline-pad"></div>
    </div>`;
}

function sprintCard(sprint, milestones, isCurrent, filter) {
  const p = sprintProgress(sprint);
  const ms = milestones.find((m) => m.number === sprint.milestone);
  const issues = sprint.sub_issues.filter((i) => filter === "all" || i.state === filter);
  return `
    <section class="card sprint ${isCurrent ? "current" : ""}">
      <div class="sprint-head">
        <div>
          <span class="num">SPRINT ${sprint.sprint}${isCurrent ? " · ATUAL" : ""}</span>
          <h3><a href="#/sprints/${sprint.sprint}">${esc(sprint.name || sprint.title)}</a></h3>
        </div>
        <span class="pill ${sprint.state === "closed" ? "ok" : ""}">${sprint.state === "closed" ? "concluído" : "aberto"}</span>
      </div>
      <div class="sprint-progress">
        <div class="progress"><span style="width:${p.pct}%"></span></div>
        <span class="mono">${p.done}/${p.total}</span><span>${p.pct}%</span>
      </div>
      <div class="filters small muted">
        ${ms ? `<span>${esc(ms.title)} · entrega ${fmtDate(ms.due_on)}</span>` : ""}
        <span><a href="#/sprints/${sprint.sprint}">ver detalhes →</a> · <a href="${esc(sprint.url)}" target="_blank" rel="noopener">#${sprint.number} ↗</a></span>
      </div>
      ${issues.length ? `<ul class="issues">
        ${issues.map((i) => `
          <li class="${i.state}">
            <span class="st" title="${i.state === "closed" ? "fechada" : "aberta"}"></span>
            <a href="${esc(i.url)}" target="_blank" rel="noopener">${esc(i.title)}</a>
            <span class="who">${i.assignees.length ? i.assignees.map((a) => "@" + esc(a)).join(" ") : "#" + i.number}</span>
          </li>`).join("")}
      </ul>` : `<div class="empty small">${sprint.sub_issues.length ? "Nenhuma issue neste filtro." : "Sem sub-issues."}</div>`}
    </section>`;
}

export function render(root, param) {
  if (param) return detail.render(root, param);
  root.innerHTML = `
    <div class="page-head">
      <div><h1>Sprints</h1><p>Direto do GitHub: milestones, issues SPRINT e suas sub-issues</p></div>
      <div class="filters">
        <div class="tabs" id="s-filter">
          <button data-f="all" class="active">todas</button><button data-f="open">abertas</button><button data-f="closed">fechadas</button>
        </div>
        <button id="s-refresh" type="button">↻ Atualizar</button>
      </div>
    </div>
    <div id="s-body"><div class="card empty">Carregando do GitHub…</div></div>`;

  let data = null;
  let filter = "all";
  let alive = true;

  function draw() {
    const body = root.querySelector("#s-body");
    if (!data.sprints.length && !data.milestones.length) {
      body.innerHTML = `<div class="card error-box">${esc(data.error || "Nada encontrado no GitHub.")}</div>`;
      return;
    }
    const ms = currentMilestone(data.milestones);
    const current = data.sprints.find((s) => s.state === "open");
    const days = ms ? Math.ceil((new Date(ms.due_on) - Date.now()) / 86400000) : null;
    const openAll = data.issues.filter((i) => i.state === "open").length;
    body.innerHTML = `
      ${data.error ? `<div class="error-box" style="margin-bottom:12px">GitHub indisponível, mostrando cópia em cache: ${esc(data.error)}</div>` : ""}
      <div class="grid cols-3" style="margin-bottom:16px">
        <section class="card span-2">
          <h2>Linha do tempo</h2>
          ${timeline(data.milestones)}
        </section>
        <section class="card">
          <h2>Próxima entrega</h2>
          ${ms ? `
            <div class="countdown">
              <b>${Math.max(days, 0)}</b>
              <div><div>${days < 0 ? "dias (prazo passou)" : "dias restantes"}</div>
              <div class="muted small">${esc(ms.title)} · ${fmtDate(ms.due_on)}</div></div>
            </div>
            <div class="sprint-progress" style="margin-top:12px">
              <div class="progress"><span style="width:${ms.open + ms.closed ? Math.round(ms.closed / (ms.open + ms.closed) * 100) : 0}%"></span></div>
              <span class="mono">${ms.closed}/${ms.open + ms.closed}</span>
            </div>` : '<div class="empty">Sem datas.</div>'}
          <p class="muted small">${openAll} issues abertas no total · atualizado ${new Date(data.fetched * 1000).toLocaleTimeString("pt-BR")}</p>
        </section>
      </div>
      <div class="sprints">
        ${data.sprints.map((s) => sprintCard(s, data.milestones, s === current, filter)).join("")}
      </div>`;
  }

  async function load(refresh = false) {
    try {
      const next = await getJSON(`/api/sprints${refresh ? "?refresh=1" : ""}`);
      if (!alive) return;
      data = next;
      draw();
    } catch (e) {
      if (alive) root.querySelector("#s-body").innerHTML = `<div class="card error-box">Erro: ${esc(e.message)}</div>`;
    }
  }

  root.querySelector("#s-filter").addEventListener("click", (e) => {
    if (!e.target.dataset.f) return;
    filter = e.target.dataset.f;
    root.querySelectorAll("#s-filter button").forEach((b) => b.classList.toggle("active", b.dataset.f === filter));
    if (data) draw();
  });
  root.querySelector("#s-refresh").addEventListener("click", () => load(true));
  load();

  return () => { alive = false; };
}
