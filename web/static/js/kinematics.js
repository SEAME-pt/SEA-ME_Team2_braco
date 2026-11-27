// The arm's kinematics in the planning frame (`world`: the table surface under the arm), in mm.
//
// The joint chain is braco_description's URDF (mycobot_280.urdf.xacro, braco.urdf.xacro), so a
// point here is the point MoveIt plans for: FK of [0, -0.5, -1.0, -0.1, 0, 0] rad gives the tcp
// at (216.7, -64.6, 112.6) mm, as /compute_fk does (braco_mission/config/goal_set.yaml).
//
// The reach is the tcp's with the tool pointing straight down, the orientation the page sends to
// /pick_and_place. Then joint5 = 0 and joint4 = -90° - joint2 - joint3: the arm is a planar chain
// in a vertical plane turned by joint1, with the tcp always TCP_SIDE_MM to the side of that plane.
// So the tool-down reach at a height is a set of distances along the plane (sampled over joint2 x
// joint3), turned by joint1 within its limits. With joint3 at most ELBOW_MAX_DEG, as braco_mission
// plans (elbow_max_deg). It is the kinematic reach: MoveIt still checks collisions and plans.

import { JOINT_LIMITS } from "./specs.js";

export const BASE_HEIGHT_MM = 30;     // g_base_height: world (table) -> base_link
export const TCP_LENGTH_MM = 130;     // tcp_length: tool0 -> tcp
export const FLANGE_YAW_DEG = -45;    // link6 -> tool0
export const ELBOW_MAX_DEG = 20;      // braco_mission's elbow_max_deg
// G-base plate (collision box, 110 x 150 mm) and a margin for the open fingers: no tcp target here
export const BASE_KEEPOUT_MM = { x: 55 + 30, y: 75 + 30 };

// URDF joint origins: xyz (m) and rpy (rad) of joint1..joint6, all turning about their z
const JOINTS = [
  [[0, 0, 0.15756], [0, 0, 0]],
  [[0, 0, -0.001], [0, 1.5708, -1.5708]],
  [[-0.1104, 0, 0], [0, 0, 0]],
  [[-0.096, 0, 0.06462], [0, 0, -1.5708]],
  [[0, -0.07318, 0], [1.5708, -1.5708, 0]],
  [[0, 0.0456, 0], [-1.5708, 0, 0]],
];

// 4x4 matrices as flat row-major arrays of 16
function mul(a, b) {
  const m = new Array(16);
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      m[i * 4 + j] = a[i * 4] * b[j] + a[i * 4 + 1] * b[4 + j] + a[i * 4 + 2] * b[8 + j] + a[i * 4 + 3] * b[12 + j];
    }
  }
  return m;
}

function origin([x, y, z], [r, p, yw]) {
  const [cr, sr, cp, sp, cy, sy] = [Math.cos(r), Math.sin(r), Math.cos(p), Math.sin(p), Math.cos(yw), Math.sin(yw)];
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x * 1000,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y * 1000,
    -sp, cp * sr, cp * cr, z * 1000,
    0, 0, 0, 1,
  ];
}

function rotZ(q) {
  const c = Math.cos(q), s = Math.sin(q);
  return [c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
}

const ORIGINS = JOINTS.map(([xyz, rpy]) => origin(xyz, rpy));
const RAD = Math.PI / 180;

// anglesDeg: joint1..joint6 in degrees (the page's telemetry). Returns the base, every joint's
// origin and the tcp as [x, y, z] in mm, and the tool's pointing direction (unit vector).
export function forward(anglesDeg) {
  let m = origin([0, 0, BASE_HEIGHT_MM / 1000], [0, 0, 0]);
  const points = [[0, 0, 0], [m[3], m[7], m[11]]];
  for (let i = 0; i < 6; i++) {
    m = mul(mul(m, ORIGINS[i]), rotZ((anglesDeg[i] || 0) * RAD));
    points.push([m[3], m[7], m[11]]);
  }
  m = mul(m, rotZ(FLANGE_YAW_DEG * RAD));
  const tool = [m[2], m[6], m[10]];
  const tcp = [m[3] + tool[0] * TCP_LENGTH_MM, m[7] + tool[1] * TCP_LENGTH_MM, m[11] + tool[2] * TCP_LENGTH_MM];
  return { points, flange: points[points.length - 1], tcp, tool };
}

// ---- tool-down reach -------------------------------------------------------------

const STEP_DEG = 0.5;
let samples = null;   // {u, z, side}: tcp along the arm's plane and its height, per (joint2, joint3)
let side = null;

function sampleToolDown() {
  if (samples) return samples;
  const [l2, l3, l4] = [JOINT_LIMITS[1], JOINT_LIMITS[2], JOINT_LIMITS[3]];
  const u = [], z = [];
  let lateral = 0;
  for (let j2 = l2[0]; j2 <= l2[1]; j2 += STEP_DEG) {
    for (let j3 = l3[0]; j3 <= Math.min(l3[1], ELBOW_MAX_DEG); j3 += STEP_DEG) {
      const j4 = -90 - j2 - j3;
      if (j4 < l4[0] || j4 > l4[1]) continue;
      const { tcp } = forward([0, j2, j3, j4, 0, 0]);
      u.push(tcp[0]);
      z.push(tcp[2]);
      lateral = tcp[1];
    }
  }
  side = lateral;   // the same for every sample: TCP_SIDE_MM
  samples = { u: Float32Array.from(u), z: Float32Array.from(z) };
  return samples;
}

// The tcp is this far to the side of the arm's vertical plane (mm, negative: to its right)
export function tcpSide() {
  sampleToolDown();
  return side;
}

const reachCache = new Map();
const Z_TOL_MM = 1.5;
const GAP_MM = 3;

// Distances along the arm's plane (mm) where the tcp reaches height z tool-down: [[lo, hi], ...]
export function planeIntervals(zMm) {
  const key = Math.round(zMm);
  if (reachCache.has(key)) return reachCache.get(key);
  const { u, z } = sampleToolDown();
  const hits = [];
  for (let i = 0; i < u.length; i++) if (Math.abs(z[i] - key) <= Z_TOL_MM) hits.push(u[i]);
  hits.sort((a, b) => a - b);
  const out = [];
  for (const h of hits) {
    const last = out[out.length - 1];
    if (last && h - last[1] <= GAP_MM) last[1] = h;
    else out.push([h, h]);
  }
  reachCache.set(key, out);
  return out;
}

function inIntervals(v, intervals) {
  for (const [lo, hi] of intervals) if (v >= lo && v <= hi) return true;
  return false;
}

// Is (x, y) reachable tool-down at height z (mm)?
export function reachableAt(x, y, zMm) {
  if (Math.abs(x) < BASE_KEEPOUT_MM.x && Math.abs(y) < BASE_KEEPOUT_MM.y) return false;
  const s = tcpSide();
  const r2 = x * x + y * y;
  if (r2 < s * s) return false;
  const along = Math.sqrt(r2 - s * s);
  const intervals = planeIntervals(zMm);
  const [lo1, hi1] = JOINT_LIMITS[0];
  // Only the arm reaching forward (along > 0): the solutions with the tcp behind the shoulder fold
  // the arm back over its own base, which MoveIt would have to plan around.
  if (!inIntervals(along, intervals)) return false;
  // joint1 turns the plane so that (along, side) lands on (x, y)
  let j1 = (Math.atan2(y, x) - Math.atan2(s, along)) / RAD;
  j1 = ((j1 + 540) % 360) - 180;
  return j1 >= lo1 && j1 <= hi1;
}

// The whole pick (or place): the pose, the pre-grasp above it and the straight line between.
export function reachable(x, y, zMm, approachMm) {
  for (const f of [0, 0.5, 1]) if (!reachableAt(x, y, zMm + f * approachMm)) return false;
  return true;
}

// Why a point cannot be used, or null
export function whyNot(x, y, zMm, approachMm) {
  if (Math.abs(x) < BASE_KEEPOUT_MM.x && Math.abs(y) < BASE_KEEPOUT_MM.y) return "on the arm's base";
  if (!reachableAt(x, y, zMm)) return "out of reach at this height";
  if (!reachable(x, y, zMm, approachMm)) return "the approach above it is out of reach";
  return null;
}
