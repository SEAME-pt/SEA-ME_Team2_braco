// Shared activity log: who did what, for everybody on the page.

import { esc, fmtTime, live, on } from "./util.js";

function row(e) {
  return `
    <li class="${e.ok ? "" : "failed"}">
      <time>${fmtTime(e.t)}</time>
      <b>${esc(e.who)}</b>
      <span>${esc(e.action)}${e.detail ? ` <em>${esc(e.detail)}</em>` : ""}</span>
    </li>`;
}

// Fill `el` with the newest `limit` entries and keep it updated. Returns the cleanup.
export function mountActivity(el, limit = 30) {
  const draw = () => {
    const entries = live.activity.slice(-limit).reverse();
    el.innerHTML = entries.length
      ? `<ul class="activity">${entries.map(row).join("")}</ul>`
      : '<div class="empty small">Nothing yet. Commands, STOPs and control changes from everybody show up here.</div>';
  };
  draw();
  return on("activity", draw);
}
