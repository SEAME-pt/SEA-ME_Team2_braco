// Documentation browser: Markdown files from docs/ rendered with marked.

import { esc, getJSON } from "../util.js";

const DEFAULT_DOC = "docs/dev-setup.md";

let state = null;   // { root, list, current }

function resolveLink(href, current) {
  // Relative links between docs become in-app routes
  if (/^[a-z]+:|^#|^\//i.test(href)) return null;
  const url = new URL(href, `http://x/${current}`);
  const path = url.pathname.slice(1);
  return path.endsWith(".md") ? `#/docs/${path}` : null;
}

function renderList(filter = "") {
  const { root, list, current } = state;
  const q = filter.trim().toLowerCase();
  const groups = {};
  list
    .filter((d) => !q || d.title.toLowerCase().includes(q) || d.path.toLowerCase().includes(q))
    .forEach((d) => { (groups[d.group] ||= []).push(d); });
  root.querySelector("#d-items").innerHTML = Object.entries(groups).map(([g, docs]) => `
    <h3>${esc(g)}</h3>
    ${docs.map((d) => `
      <a href="#/docs/${esc(d.path)}" class="${d.path === current ? "active" : ""}">
        ${esc(d.title)}<small>${esc(d.path)}</small>
      </a>`).join("")}
  `).join("") || '<div class="empty small">Nada encontrado.</div>';
}

async function show(path) {
  state.current = path || DEFAULT_DOC;
  renderList(state.root.querySelector("#d-search").value);
  const box = state.root.querySelector("#d-content");
  box.innerHTML = '<div class="empty">Carregando…</div>';
  try {
    const res = await fetch(`/api/docs/${state.current}`);
    if (!res.ok) throw new Error(res.status === 404 ? "Documento não encontrado." : res.statusText);
    // Inline LaTeX like $0^\circ$ has no renderer here: show plain text
    const md = (await res.text()).replace(/\$([^$\n]*[\\^][^$\n]*)\$/g, (_, m) =>
      m.replace(/\^\\circ/g, "°").replace(/\\(times|cdot)/g, "×").replace(/\\([a-zA-Z]+)/g, "$1"));
    const html = window.marked ? window.marked.parse(md) : `<pre>${esc(md)}</pre>`;
    const github = `https://github.com/SEAME-pt/SEA-ME_Team2_braco/blob/main/${state.current}`;
    box.innerHTML = `
      <div class="doc-meta">
        <span class="mono">${esc(state.current)}</span>
        <a href="${github}" target="_blank" rel="noopener">ver no GitHub ↗</a>
      </div>
      <article>${html}</article>`;
    box.querySelectorAll("a[href]").forEach((a) => {
      const to = resolveLink(a.getAttribute("href"), state.current);
      if (to) a.setAttribute("href", to);
      else if (/^https?:/.test(a.getAttribute("href"))) a.target = "_blank";
    });
    if (window.hljs) box.querySelectorAll("pre code").forEach((el) => window.hljs.highlightElement(el));
    window.scrollTo(0, 0);
  } catch (e) {
    box.innerHTML = `<div class="error-box">${esc(e.message)}</div>`;
  }
}

export function render(root, path) {
  root.innerHTML = `
    <div class="page-head">
      <div><h1>Documentação</h1><p>Todos os arquivos Markdown do repositório, organizados por tema, sempre na versão do branch atual</p></div>
    </div>
    <div class="docs">
      <nav class="card doc-list">
        <input id="d-search" type="search" placeholder="Buscar documento…" autocomplete="off">
        <div id="d-items"><div class="empty small">Carregando…</div></div>
      </nav>
      <section class="card markdown" id="d-content"></section>
    </div>`;
  state = { root, list: [], current: path || DEFAULT_DOC };
  const s = state;
  root.querySelector("#d-search").addEventListener("input", (e) => renderList(e.target.value));
  getJSON("/api/docs").then((list) => {
    if (state !== s) return;
    s.list = list;
    renderList();
  });
  show(path);
  return () => { state = null; };
}

export function update(path) {
  if (state) show(path);
}
