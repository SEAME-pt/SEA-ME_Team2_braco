// Simplified 3D model of the myCobot 280 driven by the live joint angles.
// Link lengths roughly follow the 280's DH parameters (mm, scaled 1:100).

import * as THREE from "three";

const COLORS = [0xf59e0b, 0x3b82f6, 0x10b981, 0xec4899, 0x8b5cf6, 0x06b6d4];


export function createArm3D(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(40, 1, 0.1, 100);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.4);
  sun.position.set(4, 8, 5);
  scene.add(sun);

  const grid = new THREE.GridHelper(8, 16);
  scene.add(grid);

  const body = (color) => new THREE.MeshStandardMaterial({ color, roughness: 0.45, metalness: 0.15 });
  const white = body(0xe8eaee);
  const dark = body(0x2b3038);

  function cyl(r, h, mat, axis = "y") {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 32), mat);
    if (axis === "z") m.rotation.x = Math.PI / 2;
    if (axis === "x") m.rotation.z = Math.PI / 2;
    return m;
  }
  function box(w, h, d, mat) {
    return new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  }
  function ring(color, axis) {
    const m = cyl(0.42, 0.06, body(color), axis);
    return m;
  }

  // Base plate
  const base = new THREE.Group();
  scene.add(base);
  const plate = box(2.2, 0.2, 2.2, dark);
  plate.position.y = 0.1;
  base.add(plate);
  const tower = cyl(0.55, 0.9, white);
  tower.position.y = 0.65;
  base.add(tower);

  // J1: yaw about Y
  const j1 = new THREE.Group();
  j1.position.y = 1.1;
  base.add(j1);
  j1.add(ring(COLORS[0], "y"));
  const shoulder = cyl(0.42, 0.6, white);
  shoulder.position.y = 0.3;
  j1.add(shoulder);

  // J2: pitch about Z
  const j2 = new THREE.Group();
  j2.position.set(0, 0.6, 0.45);
  j1.add(j2);
  j2.add(ring(COLORS[1], "z"));
  const upper = box(0.5, 1.1, 0.4, white);
  upper.position.y = 0.55;
  j2.add(upper);

  // J3: pitch about Z
  const j3 = new THREE.Group();
  j3.position.set(0, 1.1, -0.4);
  j2.add(j3);
  j3.add(ring(COLORS[2], "z"));
  const fore = box(0.45, 0.96, 0.38, white);
  fore.position.y = 0.48;
  j3.add(fore);

  // J4: pitch about Z
  const j4 = new THREE.Group();
  j4.position.set(0, 0.96, 0.38);
  j3.add(j4);
  j4.add(ring(COLORS[3], "z"));
  const wrist = cyl(0.3, 0.5, white);
  wrist.position.y = 0.25;
  j4.add(wrist);

  // J5: yaw about Y
  const j5 = new THREE.Group();
  j5.position.y = 0.55;
  j4.add(j5);
  j5.add(ring(COLORS[4], "y"));
  const head = box(0.5, 0.4, 0.5, white);
  head.position.set(0, 0.2, 0);
  j5.add(head);

  // J6: flange roll about Z
  const j6 = new THREE.Group();
  j6.position.set(0, 0.2, 0.3);
  j5.add(j6);
  j6.add(ring(COLORS[5], "z"));
  const flange = cyl(0.32, 0.12, dark, "z");
  flange.position.z = 0.1;
  j6.add(flange);
  const marker = box(0.08, 0.2, 0.06, body(COLORS[5]));
  marker.position.set(0, 0.2, 0.16);
  j6.add(marker);

  const joints = [j1, j2, j3, j4, j5, j6];
  const axes = ["y", "z", "z", "z", "y", "z"];
  const target = [0, 0, 0, 0, 0, 0];
  const current = [0, 0, 0, 0, 0, 0];

  // Simple orbit: drag to rotate, wheel to zoom
  let yaw = 0.7, pitch = 0.35, dist = 9;
  let dragging = null;
  const el = renderer.domElement;
  el.addEventListener("pointerdown", (e) => { dragging = { x: e.clientX, y: e.clientY }; el.setPointerCapture(e.pointerId); });
  el.addEventListener("pointerup", () => { dragging = null; });
  el.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    yaw -= (e.clientX - dragging.x) * 0.008;
    pitch = Math.max(-0.1, Math.min(1.3, pitch + (e.clientY - dragging.y) * 0.006));
    dragging = { x: e.clientX, y: e.clientY };
  });
  el.addEventListener("wheel", (e) => {
    e.preventDefault();
    dist = Math.max(5, Math.min(16, dist + e.deltaY * 0.01));
  }, { passive: false });

  function applyTheme() {
    const dark = getComputedStyle(document.documentElement).colorScheme === "dark";
    grid.material.color.set(dark ? 0x2a303b : 0xd0d5dc);
    grid.material.opacity = 0.8;
  }
  applyTheme();
  window.addEventListener("themechange", applyTheme);

  function resize() {
    const w = container.clientWidth, h = container.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  const ro = new ResizeObserver(resize);
  ro.observe(container);
  resize();

  let raf = 0;
  function frame() {
    raf = requestAnimationFrame(frame);
    for (let i = 0; i < 6; i++) {
      current[i] += (target[i] - current[i]) * 0.2;
      joints[i].rotation[axes[i]] = THREE.MathUtils.degToRad(current[i]);
    }
    camera.position.set(
      dist * Math.cos(pitch) * Math.sin(yaw),
      2.2 + dist * Math.sin(pitch),
      dist * Math.cos(pitch) * Math.cos(yaw),
    );
    camera.lookAt(0, 2.2, 0);
    renderer.render(scene, camera);
  }
  frame();

  return {
    setAngles(angles) {
      if (!angles) return;
      angles.forEach((a, i) => { target[i] = a; });
    },
    dispose() {
      cancelAnimationFrame(raf);
      ro.disconnect();
      window.removeEventListener("themechange", applyTheme);
      renderer.dispose();
      el.remove();
    },
  };
}
