// One page per sprint: details, progress and the evolutions delivered in it.

import { esc, fmtDate, getJSON } from "../util.js";

// Strip active content from issue Markdown before it goes into innerHTML.
function markdown(md) {
  if (!window.marked) return `<pre>${esc(md)}</pre>`;
  const doc = new DOMParser().parseFromString(window.marked.parse(md), "text/html");
  doc.querySelectorAll("script,iframe,object,embed,style,link").forEach((n) => n.remove());
  doc.querySelectorAll("*").forEach((n) => {
    [...n.attributes].forEach((a) => {
      if (/^on/i.test(a.name) || /^\s*javascript:/i.test(a.value)) n.removeAttribute(a.name);
    });
  });
  doc.querySelectorAll("a").forEach((a) => { a.target = "_blank"; a.rel = "noopener"; });
  return doc.body.innerHTML;
}

// Sprint period: previous milestone's due date -> this milestone's due date.
export function sprintWindow(sprint, sprints, milestones) {
  const due = (s) => milestones.find((m) => m.number === s.milestone)?.due_on;
  const idx = sprints.indexOf(sprint);
  const prev = idx > 0 ? due(sprints[idx - 1]) : null;
  return {
    start: prev || sprint.created_at,
    end: due(sprint) || null,
  };
}

// Merged PRs belonging to the sprint: by milestone, else by merge date inside the period.
function sprintPulls(sprint, win, pulls) {
  const t = (iso) => new Date(iso).getTime();
  return pulls.filter((p) => {
    if (p.milestone) return p.milestone === sprint.milestone;
    return (!win.start || t(p.merged_at) > t(win.start)) && (!win.end || t(p.merged_at) <= t(win.end) + 86400000);
  });
}

function feed(sprint, pulls) {
  const events = [
    ...pulls.map((p) => ({ kind: "pr", at: p.merged_at, title: p.title, url: p.url, who: p.author ? "@" + p.author : "", ref: "PR #" + p.number })),
    ...sprint.sub_issues.filter((i) => i.state === "closed" && i.closed_at).map((i) => ({
      kind: "issue", at: i.closed_at, title: i.title, url: i.url,
      who: i.assignees.map((a) => "@" + a).join(" "), ref: "#" + i.number,
    })),
  ].sort((a, b) => new Date(b.at) - new Date(a.at));
  if (!events.length) return `<div class="empty small">No deliveries recorded yet.</div>`;

  const days = [];
  events.forEach((e) => {
    const day = e.at.slice(0, 10);
    const last = days[days.length - 1];
    if (last && last.day === day) last.items.push(e); else days.push({ day, items: [e] });
  });
  return `<div class="evo">${days.map((d) => `
    <div class="evo-day">
      <div class="evo-date">${fmtDate(d.day + "T12:00:00Z", { day: "2-digit", month: "short" })}</div>
      <ul>${d.items.map((e) => `
        <li class="${e.kind}">
          <span class="tag">${e.kind === "pr" ? "delivery" : "task"}</span>
          <a href="${esc(e.url)}" target="_blank" rel="noopener">${esc(e.title)}</a>
          <span class="who">${esc(e.who)} ${esc(e.ref)}</span>
        </li>`).join("")}</ul>
    </div>`).join("")}</div>`;
}

function draw(body, data, n) {
  const sprint = data.sprints.find((s) => s.sprint === n);
  if (!sprint) {
    body.innerHTML = `<div class="card error-box">Sprint ${esc(n)} not found.</div>`;
    return;
  }
  const ms = data.milestones.find((m) => m.number === sprint.milestone);
  const win = sprintWindow(sprint, data.sprints, data.milestones);
  const pulls = sprintPulls(sprint, win, data.pulls || []);
  const total = sprint.sub_issues.length;
  const done = sprint.sub_issues.filter((i) => i.state === "closed").length;
  const pct = total ? Math.round((done / total) * 100) : 0;
  const people = new Set([
    ...sprint.sub_issues.flatMap((i) => i.assignees),
    ...pulls.map((p) => p.author).filter(Boolean),
  ]);
  const pending = sprint.sub_issues.filter((i) => i.state !== "closed");
  const idx = data.sprints.indexOf(sprint);
  const prev = data.sprints[idx - 1];
  const next = data.sprints[idx + 1];
  const days = win.end ? Math.ceil((new Date(win.end) - Date.now()) / 86400000) : null;

  body.innerHTML = `
    ${data.error ? `<div class="error-box" style="margin-bottom:12px">GitHub unavailable, showing the cached copy: ${esc(data.error)}</div>` : ""}
    <section class="card sprint-hero ${sprint.state === "open" ? "current" : ""}">
      <div class="sprint-head">
        <div>
          <span class="num">SPRINT ${sprint.sprint}</span>
          <h2 class="hero-title">${esc(sprint.name || sprint.title)}</h2>
          <p class="muted small">
            ${fmtDate(win.start)} → ${win.end ? fmtDate(win.end) : "no deadline"}
            ${ms ? ` · <a href="${esc(ms.url)}" target="_blank" rel="noopener">${esc(ms.title)}</a>` : ""}
            ${sprint.state === "open" && days != null ? ` · ${days < 0 ? "deadline passed " + -days + " day(s) ago" : days + " day(s) left"}` : ""}
          </p>
        </div>
        <div class="hero-actions">
          <span class="pill ${sprint.state === "closed" ? "ok" : ""}">${sprint.state === "closed" ? "done" : "open"}</span>
          <a class="pill" href="${esc(sprint.url)}" target="_blank" rel="noopener">#${sprint.number} ↗</a>
        </div>
      </div>
      <div class="sprint-progress"><div class="progress"><span style="width:${pct}%"></span></div><span>${pct}%</span></div>
    </section>

    ${sprint.body.trim() ? `<section class="card markdown" style="margin-bottom:16px"><h2 style="display:flex">Description</h2>${markdown(sprint.body)}</section>` : ""}

    <div class="stats">
      <div class="stat"><b>${done}/${total}</b><span>tasks done</span></div>
      <div class="stat"><b>${pulls.length}</b><span>PRs merged</span></div>
      <div class="stat"><b>${people.size}</b><span>people involved</span></div>
      <div class="stat"><b>${pending.length}</b><span>pending</span></div>
    </div>

    <div class="grid cols-3">
      <section class="card span-2">
        <h2>Project progress <span class="muted small">${pulls.length + done} deliveries</span></h2>
        ${feed(sprint, pulls)}
      </section>
      <div style="display:flex;flex-direction:column;gap:16px">
        <section class="card">
          <h2>Pending</h2>
          ${pending.length ? `<ul class="issues">${pending.map((i) => `
            <li><span class="st"></span><a href="${esc(i.url)}" target="_blank" rel="noopener">${esc(i.title)}</a>
            <span class="who">${i.assignees.length ? i.assignees.map((a) => "@" + esc(a)).join(" ") : "#" + i.number}</span></li>`).join("")}</ul>`
            : `<div class="empty small">Nothing pending.</div>`}
        </section>
        <section class="card">
          <h2>Team</h2>
          ${people.size ? `<div class="chips">${[...people].sort().map((p) => `<a class="pill" href="https://github.com/${esc(p)}" target="_blank" rel="noopener">@${esc(p)}</a>`).join("")}</div>`
            : `<div class="empty small">Nobody assigned.</div>`}
        </section>
      </div>
    </div>

    <nav class="sprint-nav">
      ${prev ? `<a href="#/sprints/${prev.sprint}">← Sprint ${prev.sprint} · ${esc(prev.name)}</a>` : "<span></span>"}
      ${next ? `<a href="#/sprints/${next.sprint}">Sprint ${next.sprint} · ${esc(next.name)} →</a>` : "<span></span>"}
    </nav>`;
}

export function render(root, param) {
  const n = Number(param);
  root.innerHTML = `
    <div class="page-head">
      <div><h1>Sprint ${esc(n)}</h1><p><a href="#/sprints">← all sprints</a></p></div>
      <div class="filters"><button id="sd-refresh" type="button">↻ Refresh</button></div>
    </div>
    <div id="sd-body"><div class="card empty">Loading from GitHub…</div></div>`;
  let alive = true;
  async function load(refresh = false) {
    try {
      const data = await getJSON(`/api/sprints${refresh ? "?refresh=1" : ""}`);
      if (alive) draw(root.querySelector("#sd-body"), data, n);
    } catch (e) {
      if (alive) root.querySelector("#sd-body").innerHTML = `<div class="card error-box">Error: ${esc(e.message)}</div>`;
    }
  }
  root.querySelector("#sd-refresh").addEventListener("click", () => load(true));
  load();
  return () => { alive = false; };
}
