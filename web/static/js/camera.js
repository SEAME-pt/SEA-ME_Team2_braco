// The arm's camera: the MJPEG stream of /api/camera/stream in an <img>, with its state.
//
// The stream ends when the camera goes offline; the status is polled every 2 s and the
// stream is opened again as soon as frames come back. Leaving the page closes it.

import { esc } from "./util.js";

const POLL_MS = 2000;

export function mountCamera(el) {
  el.classList.add("camera");
  el.innerHTML = `
    <img alt="arm camera" hidden>
    <div class="camera-off empty">connecting to the camera…</div>
    <div class="camera-tools">
      <span class="pill" data-k="state">—</span>
      <button class="ghost small" data-k="snap" title="Save the current frame" disabled>⤓ Frame</button>
      <button class="ghost small" data-k="full" title="Full screen">⛶</button>
    </div>`;
  const img = el.querySelector("img");
  const off = el.querySelector(".camera-off");
  const state = el.querySelector('[data-k="state"]');
  const snap = el.querySelector('[data-k="snap"]');
  let streaming = false;
  let alive = true;
  let timer = null;

  function show(online, text) {
    img.hidden = !online;
    off.hidden = online;
    snap.disabled = !online;
    if (!online) off.textContent = text || "camera offline";
  }

  function open() {
    streaming = true;
    img.src = `/api/camera/stream?t=${Date.now()}`;
    show(true);   // an MJPEG <img> does not reliably fire "load"; "error" still does
  }

  function close() {
    streaming = false;
    img.removeAttribute("src");
  }

  img.addEventListener("error", () => { if (streaming) { close(); show(false); } });

  async function poll() {
    try {
      const s = await (await fetch("/api/camera/status")).json();
      if (!alive) return;
      state.className = `pill ${s.online ? "ok" : "warn"}`;
      state.textContent = s.online ? `live · ${s.fps} fps` : "offline";
      if (s.online && !streaming) open();
      if (!s.online) {
        if (streaming) close();
        show(false, s.error ? `Camera offline: ${s.error}` : "camera offline");
      }
    } catch {
      if (alive) { state.className = "pill hot"; state.textContent = "no server"; }
    }
    if (alive) timer = setTimeout(poll, POLL_MS);
  }

  snap.addEventListener("click", () => {
    const a = document.createElement("a");
    a.href = `/api/camera/snapshot?t=${Date.now()}`;
    a.download = `braco-camera-${new Date().toISOString().replace(/[:.]/g, "-")}.jpg`;
    a.click();
  });
  el.querySelector('[data-k="full"]').addEventListener("click", () => {
    if (document.fullscreenElement) document.exitFullscreen();
    else el.requestFullscreen?.().catch((e) => { off.innerHTML = esc(e.message); });
  });

  poll();
  return () => {
    alive = false;
    clearTimeout(timer);
    close();   // the server's generator ends when the browser drops the connection
  };
}
