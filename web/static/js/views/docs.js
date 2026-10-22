// Documentation browser: Markdown files rendered with marked, HTML pages from docs/ in an iframe.

import { esc, getJSON } from "../util.js";

const DEFAULT_DOC = "docs/dev-setup.md";

let state = null;   // { root, list, current }

function resolveLink(href, current) {
  // Relative links between docs become in-app routes
  if (/^[a-z]+:|^#|^\//i.test(href)) return null;
  const url = new URL(href, `http://x/${current}`);
  const path = url.pathname.slice(1);
  return /\.(md|html)$/.test(path) ? `#/docs/${path}` : null;
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
  `).join("") || '<div class="empty small">Nothing found.</div>';
}

async function show(path) {
  state.current = path || DEFAULT_DOC;
  renderList(state.root.querySelector("#d-search").value);
  const box = state.root.querySelector("#d-content");
  box.innerHTML = '<div class="empty">Loading…</div>';
  try {
    const res = await fetch(`/api/docs/${state.current}`);
    if (!res.ok) throw new Error(res.status === 404 ? "Document not found." : res.statusText);
    if (state.current.endsWith(".html")) return showHtml(box, await res.text());
    // Inline LaTeX like $0^\circ$ has no renderer here: show plain text
    const md = (await res.text()).replace(/\$([^$\n]*[\\^][^$\n]*)\$/g, (_, m) =>
      m.replace(/\^\\circ/g, "°").replace(/\\(times|cdot)/g, "×").replace(/\\([a-zA-Z]+)/g, "$1"));
    const html = window.marked ? window.marked.parse(md) : `<pre>${esc(md)}</pre>`;
    box.innerHTML = `${docMeta()}
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

function docMeta() {
  const github = `https://github.com/SEAME-pt/SEA-ME_Team2_braco/blob/main/${state.current}`;
  return `
      <div class="doc-meta">
        <span class="mono">${esc(state.current)}</span>
        <a href="${github}" target="_blank" rel="noopener">view on GitHub ↗</a>
      </div>`;
}

// A full HTML page (its own styles and figures) shown as is, grown to its content height
function showHtml(box, html) {
  box.innerHTML = `${docMeta()}<iframe class="doc-frame" title="${esc(state.current)}"></iframe>`;
  const frame = box.querySelector("iframe");
  const fit = () => { frame.style.height = `${frame.contentDocument.documentElement.scrollHeight}px`; };
  frame.addEventListener("load", () => {
    const doc = frame.contentDocument;
    doc.querySelectorAll("a[href]").forEach((a) => {
      const href = a.getAttribute("href");
      const to = resolveLink(href, state.current);
      if (to) { a.setAttribute("href", to); a.target = "_top"; }
      else if (/^https?:/.test(href)) a.target = "_blank";
      else if (href.startsWith("#")) {
        // the frame never scrolls: scroll the app to the anchor instead
        a.addEventListener("click", (e) => {
          const el = doc.getElementById(href.slice(1));
          if (!el) return;
          e.preventDefault();
          window.scrollTo({ top: frame.getBoundingClientRect().top + window.scrollY + el.offsetTop, behavior: "smooth" });
        });
      } else if (!/^[a-z]+:|^\//i.test(href)) {
        // other repo files (code, folders): on GitHub, since the frame has no base URL
        const path = new URL(href, `http://x/${state.current}`).pathname.slice(1);
        a.href = `https://github.com/SEAME-pt/SEA-ME_Team2_braco/blob/main/${path}`;
        a.target = "_blank";
      }
    });
    fit();
    doc.fonts?.ready.then(fit);
    new ResizeObserver(fit).observe(doc.documentElement);
  });
  frame.srcdoc = html;
  window.scrollTo(0, 0);
}

export function render(root, path) {
  root.innerHTML = `
    <div class="page-head">
      <div><h1>Docs</h1><p>Every Markdown file in the repository and the HTML pages in docs/, grouped by topic, always as in the current branch</p></div>
    </div>
    <div class="docs">
      <nav class="card doc-list">
        <input id="d-search" type="search" placeholder="Search docs…" autocomplete="off">
        <div id="d-items"><div class="empty small">Loading…</div></div>
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
