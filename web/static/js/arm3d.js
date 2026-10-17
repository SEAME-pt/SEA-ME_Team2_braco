// 3D model of the myCobot 280 driven by the live joint angles.
//
// The kinematic chain uses the arm's standard DH parameters, so the pose on
// screen is the real pose: with all joints at 0 the arm stands upright and the
// flange sits at about (45.6, -63.4, 412.7) mm, like get_coords() reports.
//
// Each joint is drawn as a housing aligned with its rotation axis, with a
// coloured ring in the joint's colour (same colours as the joint cards and the
// chart) and a floating label. The flange leaves a fading trail.

import * as THREE from "three";
import { JOINTS, SOFT_LIMITS } from "./specs.js";

// theta offset (rad), d (mm), a (mm), alpha (rad)
const DH = [
  [0, 131.22, 0, Math.PI / 2],
  [-Math.PI / 2, 0, -110.4, 0],
  [0, 0, -96, 0],
  [-Math.PI / 2, 63.4, 0, Math.PI / 2],
  [Math.PI / 2, 75.05, 0, -Math.PI / 2],
  [0, 45.6, 0, 0],
];
const MM = 1 / 100;                       // scene units per mm
const COLORS = [0xf59e0b, 0x3b82f6, 0x10b981, 0xec4899, 0x8b5cf6, 0x06b6d4];
const HOUSING = [                         // radius, length (mm) of each joint housing
  [34, 70], [30, 74], [27, 66], [24, 56], [24, 54], [22, 30],
];
const TRAIL_POINTS = 240;
const VIEWS = {
  iso: { yaw: 0.75, pitch: 0.34, dist: 9.4 },
  front: { yaw: 0, pitch: 0.04, dist: 9.4 },
  side: { yaw: Math.PI / 2, pitch: 0.04, dist: 9.4 },
  top: { yaw: 0, pitch: 1.5, dist: 9.5 },
};

function isDark() {
  return getComputedStyle(document.documentElement).colorScheme === "dark";
}

export function createArm3D(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  container.prepend(renderer.domElement);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(38, 1, 0.1, 100);

  // ---- Lights and floor --------------------------------------------------------
  const hemi = new THREE.HemisphereLight(0xffffff, 0x8090a0, 1.3);
  scene.add(hemi);
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(3, 9, 4);
  sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  Object.assign(sun.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5, near: 1, far: 20 });
  scene.add(sun);
  const rim = new THREE.DirectionalLight(0xffffff, 0.5);
  rim.position.set(-4, 3, -5);
  scene.add(rim);

  const floor = new THREE.Mesh(new THREE.CircleGeometry(4.5, 64), new THREE.ShadowMaterial({ opacity: 0.18 }));
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);
  const grid = new THREE.PolarGridHelper(4.5, 12, 6, 64);
  grid.material.transparent = true;
  scene.add(grid);

  // Reach circle: the 280 mm working radius from safety.md
  const reach = new THREE.Mesh(
    new THREE.RingGeometry(2.8 - 0.015, 2.8 + 0.015, 96),
    new THREE.MeshBasicMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.6 }),
  );
  reach.rotation.x = -Math.PI / 2;
  reach.position.y = 0.002;
  scene.add(reach);

  // ---- Materials -----------------------------------------------------------------
  const shell = new THREE.MeshStandardMaterial({ color: 0xf2f3f5, roughness: 0.35, metalness: 0.05 });
  const housingMat = new THREE.MeshStandardMaterial({ color: 0xdfe2e7, roughness: 0.4, metalness: 0.1 });
  const darkMat = new THREE.MeshStandardMaterial({ color: 0x2b3038, roughness: 0.6, metalness: 0.2 });
  const ringMats = COLORS.map((c) => new THREE.MeshStandardMaterial({
    color: c, roughness: 0.35, emissive: c, emissiveIntensity: 0.15,
  }));

  function shadowed(mesh) {
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  // Rounded bar between two points (local coordinates of `parent`)
  function bone(parent, from, to, radius) {
    const a = new THREE.Vector3(...from).multiplyScalar(MM);
    const b = new THREE.Vector3(...to).multiplyScalar(MM);
    const len = a.distanceTo(b);
    if (len < 1e-6) return;
    const mesh = shadowed(new THREE.Mesh(new THREE.CapsuleGeometry(radius * MM, len, 8, 24), shell));
    mesh.position.copy(a).add(b).multiplyScalar(0.5);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), b.clone().sub(a).normalize());
    parent.add(mesh);
  }

  // Housing aligned with the local Z axis (the joint's rotation axis)
  function housing(parent, i) {
    const [r, len] = HOUSING[i];
    const group = new THREE.Group();
    group.rotation.x = Math.PI / 2;   // cylinder Y axis -> local Z
    group.add(shadowed(new THREE.Mesh(new THREE.CylinderGeometry(r * MM, r * MM, len * MM, 40), housingMat)));
    for (const side of [-1, 1]) {
      // coloured band near each end (open cylinder: no caps to z-fight)
      const band = new THREE.Mesh(
        new THREE.CylinderGeometry((r + 1.2) * MM, (r + 1.2) * MM, 8 * MM, 40, 1, true), ringMats[i]);
      band.position.y = side * (len / 2 - 8) * MM;
      group.add(band);
      // coloured cap marking the rotation axis, just outside the housing end
      const cap = new THREE.Mesh(new THREE.CircleGeometry(r * 0.55 * MM, 32), ringMats[i]);
      cap.position.y = side * (len / 2 + 0.8) * MM;
      cap.rotation.x = side * -Math.PI / 2;
      group.add(cap);
    }
    parent.add(group);
    return group;
  }

  // ---- Kinematic chain -------------------------------------------------------
  // Robot frame is Z-up; the scene is Y-up.
  const robot = new THREE.Group();
  robot.rotation.x = -Math.PI / 2;
  scene.add(robot);

  // Base: dark plate + white tower up to J1
  const plate = shadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.62, 0.68, 0.12, 48), darkMat));
  plate.rotation.x = Math.PI / 2;
  plate.position.z = 0.06;
  robot.add(plate);
  const tower = shadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.42, 0.46, 0.7, 40), shell));
  tower.rotation.x = Math.PI / 2;
  tower.position.z = 0.47;
  robot.add(tower);

  const joints = [];   // rotate about local Z
  const anchors = [];  // housing centres, for the labels
  let parent = robot;

  for (let i = 0; i < 6; i++) {
    const [, d, a, alpha] = DH[i];
    const joint = new THREE.Group();
    parent.add(joint);

    // J1 turns about the vertical axis inside the base: draw its housing on
    // top of the tower so its ring is visible. The others sit on their axis.
    const h = housing(joint, i);
    if (i === 0) h.position.z = 92 * MM;
    anchors.push(h);

    // Link drawn in the joint frame: along Z by d, then along X by a
    bone(joint, [0, 0, i === 0 ? 127 : 0], [0, 0, d], 30);
    bone(joint, [0, 0, d], [a, 0, d], 26);

    const next = new THREE.Group();
    next.position.set(a * MM, 0, d * MM);
    next.rotation.x = alpha;
    joint.add(next);
    joints.push(joint);
    parent = next;
  }

  // Flange + tool marker on the last frame
  const flange = shadowed(new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.06, 40), darkMat));
  flange.rotation.x = Math.PI / 2;
  flange.position.z = 0.03;
  parent.add(flange);
  const tool = new THREE.AxesHelper(0.45);   // red X, green Y, blue Z of the tool
  tool.position.z = 0.06;
  parent.add(tool);
  const tcp = new THREE.Object3D();
  tcp.position.z = 0.06;
  parent.add(tcp);

  // ---- Flange trail ------------------------------------------------------------
  const trailPos = new Float32Array(TRAIL_POINTS * 3);
  const trailCol = new Float32Array(TRAIL_POINTS * 3);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute("position", new THREE.BufferAttribute(trailPos, 3));
  trailGeo.setAttribute("color", new THREE.BufferAttribute(trailCol, 3));
  trailGeo.setDrawRange(0, 0);
  const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true }));
  scene.add(trail);
  const trailPts = [];
  const trailColor = new THREE.Color(COLORS[5]);
  let showTrail = true;

  function pushTrail() {
    const p = tcp.getWorldPosition(new THREE.Vector3());
    const last = trailPts[trailPts.length - 1];
    if (last && last.distanceTo(p) < 0.01) return;
    trailPts.push(p);
    if (trailPts.length > TRAIL_POINTS) trailPts.shift();
    const bg = new THREE.Color(isDark() ? 0x161a21 : 0xffffff);
    trailPts.forEach((pt, i) => {
      pt.toArray(trailPos, i * 3);
      const t = (i + 1) / trailPts.length;   // old points fade into the background
      bg.clone().lerp(trailColor, t).toArray(trailCol, i * 3);
    });
    trailGeo.setDrawRange(0, trailPts.length);
    trailGeo.attributes.position.needsUpdate = true;
    trailGeo.attributes.color.needsUpdate = true;
  }

  // ---- Labels (HTML, projected each frame) -------------------------------------
  const labelLayer = document.createElement("div");
  labelLayer.className = "j3d-labels";
  container.appendChild(labelLayer);
  const labels = JOINTS.map((name, i) => {
    const el = document.createElement("span");
    el.className = "j3d-label";
    el.style.setProperty("--jc", `#${COLORS[i].toString(16).padStart(6, "0")}`);
    el.textContent = name;
    labelLayer.appendChild(el);
    return el;
  });
  let showLabels = true;

  // ---- Camera: drag to orbit, wheel to zoom, presets ----------------------------
  const view = { ...VIEWS.iso };
  const goal = { ...VIEWS.iso };
  let dragging = null;
  const el = renderer.domElement;
  el.style.cursor = "grab";
  el.addEventListener("pointerdown", (e) => {
    dragging = { x: e.clientX, y: e.clientY };
    el.setPointerCapture(e.pointerId);
    el.style.cursor = "grabbing";
  });
  el.addEventListener("pointerup", () => { dragging = null; el.style.cursor = "grab"; });
  el.addEventListener("pointermove", (e) => {
    if (!dragging) return;
    goal.yaw -= (e.clientX - dragging.x) * 0.008;
    goal.pitch = Math.max(-0.05, Math.min(1.5, goal.pitch + (e.clientY - dragging.y) * 0.006));
    Object.assign(view, { yaw: goal.yaw, pitch: goal.pitch });
    dragging = { x: e.clientX, y: e.clientY };
  });
  el.addEventListener("wheel", (e) => {
    e.preventDefault();
    goal.dist = Math.max(5, Math.min(18, goal.dist + e.deltaY * 0.01));
  }, { passive: false });
  el.addEventListener("dblclick", () => Object.assign(goal, VIEWS.iso));

  function applyTheme() {
    const dark = isDark();
    grid.material.color.set(dark ? 0x3a414d : 0xd5dae1);
    grid.material.opacity = dark ? 0.7 : 0.6;
    hemi.intensity = dark ? 1.1 : 1.3;
  }
  applyTheme();
  window.addEventListener("themechange", applyTheme);

  let zoomFit = 1;
  function resize() {
    const w = container.clientWidth, h = container.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    // narrow (phone) viewers: back off so the reach still fits sideways
    zoomFit = Math.max(1, 1.25 / camera.aspect);
  }
  const ro = new ResizeObserver(resize);
  ro.observe(container);
  resize();

  // ---- Animation --------------------------------------------------------------
  const target = [0, 0, 0, 0, 0, 0];
  const current = [0, 0, 0, 0, 0, 0];
  const lookAt = new THREE.Vector3(0, 2.05, 0);
  const tmp = new THREE.Vector3();
  let raf = 0;

  function frame() {
    raf = requestAnimationFrame(frame);
    for (let i = 0; i < 6; i++) {
      current[i] += (target[i] - current[i]) * 0.25;
      joints[i].rotation.z = THREE.MathUtils.degToRad(current[i]) + DH[i][0];
      const [lo, hi] = SOFT_LIMITS[i];
      const outside = current[i] < lo || current[i] > hi;
      ringMats[i].emissive.set(outside ? 0xff0000 : COLORS[i]);
      ringMats[i].emissiveIntensity = outside ? 0.9 : 0.15;
    }
    // ease the camera towards the chosen view; shortest way round for yaw
    const dy = Math.atan2(Math.sin(goal.yaw - view.yaw), Math.cos(goal.yaw - view.yaw));
    view.yaw += dy * 0.12;
    view.pitch += (goal.pitch - view.pitch) * 0.12;
    view.dist += (goal.dist - view.dist) * 0.12;
    const dist = view.dist * zoomFit;
    camera.position.set(
      dist * Math.cos(view.pitch) * Math.sin(view.yaw),
      lookAt.y + dist * Math.sin(view.pitch),
      dist * Math.cos(view.pitch) * Math.cos(view.yaw),
    );
    camera.lookAt(lookAt);
    scene.updateMatrixWorld();
    if (showTrail) pushTrail();
    renderer.render(scene, camera);

    const w = container.clientWidth, h = container.clientHeight;
    anchors.forEach((anchor, i) => {
      anchor.getWorldPosition(tmp).project(camera);
      const visible = showLabels && tmp.z < 1;
      labels[i].style.display = visible ? "" : "none";
      if (visible) {
        labels[i].style.transform =
          `translate(${(tmp.x * 0.5 + 0.5) * w + 14}px, ${(-tmp.y * 0.5 + 0.5) * h - 10}px)`;
      }
    });
  }
  frame();

  return {
    setAngles(angles) {
      if (!angles) return;
      angles.forEach((a, i) => { target[i] = a; });
    },
    setView(name) {
      if (VIEWS[name]) Object.assign(goal, VIEWS[name]);
    },
    setLabels(on) { showLabels = on; },
    setTrail(on) {
      showTrail = on;
      trail.visible = on;
      if (!on) { trailPts.length = 0; trailGeo.setDrawRange(0, 0); }
    },
    dispose() {
      cancelAnimationFrame(raf);
      ro.disconnect();
      window.removeEventListener("themechange", applyTheme);
      renderer.dispose();
      el.remove();
      labelLayer.remove();
    },
  };
}
