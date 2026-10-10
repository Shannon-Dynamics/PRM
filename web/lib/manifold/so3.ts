/**
 * SO(3) stored as a unit quaternion (Choset App. E.4).
 *
 * Three facts drive every line here:
 *
 *   1. Q and −Q are the same rotation (the double cover), so the metric is
 *      θ = 2·acos(|⟨Q, Q'⟩|) — the axis–angle θ of RᵀR' (App. E.3) — and slerp
 *      flips the sign of one endpoint before it starts, or it would take the
 *      long way round the 3-sphere.
 *   2. Uniform on SO(3) means Haar-uniform: a normalized Gaussian 4-vector is
 *      uniform on S³, hence uniform on SO(3). Uniform *Euler angles* are not,
 *      and Chapter 11's samplers depend on the difference.
 *   3. Euler Z-Y-Z angles (§3.6, eqs. 3.6–3.8) are a chart only where
 *      R₃₃ ≠ ±1; at R₃₃ = 1 only φ + ψ is determined (eq. E.2, E.9). That is
 *      gimbal lock, and it is a missing chart, not a mechanical defect. The
 *      quaternion's four charts Uᵢ = {|qᵢ| largest} cover everything.
 *
 * Only what the chapter and Appendix B need is here; the full Lie-group
 * machinery (exp/log, adjoints) is the sister book's Chapter 3 and Appendix C.
 */

import type { Rng } from '../prob/rng';
import type { Chart } from './manifold';
import type { Vec3 } from './torus';

/** Unit quaternion (w, x, y, z) = (cos θ/2, n sin θ/2), Choset eq. E.26. */
export interface Quat {
  w: number;
  x: number;
  y: number;
  z: number;
}

export type Mat3x3 = [Vec3, Vec3, Vec3];

export const QUAT_IDENTITY: Quat = { w: 1, x: 0, y: 0, z: 0 };

export function quatNormalize(q: Quat): Quat {
  const n = Math.hypot(q.w, q.x, q.y, q.z);
  if (n < 1e-300) return { ...QUAT_IDENTITY };
  return { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n };
}

/** Hamilton product, Choset's z = xy with z₀ = x₀y₀ − xᵀy, z = x₀y + y₀x + x × y. */
export function quatMul(a: Quat, b: Quat): Quat {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
    z: a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x,
  };
}

/** Conjugate Q* = (q₀, −q), eq. E.31 — the inverse rotation for a unit quaternion. */
export const quatConj = (q: Quat): Quat => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z });

export const quatDot = (a: Quat, b: Quat) => a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

/** Eq. E.26: rotation by θ about the unit axis n. */
export function quatFromAxisAngle(axis: Vec3, theta: number): Quat {
  const n = Math.hypot(axis[0], axis[1], axis[2]);
  if (n < 1e-15) return { ...QUAT_IDENTITY };
  const s = Math.sin(theta / 2) / n;
  return { w: Math.cos(theta / 2), x: axis[0] * s, y: axis[1] * s, z: axis[2] * s };
}

/**
 * Eqs. E.19–E.21: axis and angle of a rotation, θ ∈ [0, π]. The sign is fixed
 * by choosing the representative with w ≥ 0 — the same convention as the metric.
 */
export function axisAngle(q: Quat): { axis: Vec3; angle: number } {
  const s = q.w < 0 ? -1 : 1;
  const w = Math.min(1, Math.max(-1, s * q.w));
  const angle = 2 * Math.acos(w);
  const n = Math.hypot(q.x, q.y, q.z);
  if (n < 1e-12) return { axis: [0, 0, 1], angle: 0 };
  return { axis: [(s * q.x) / n, (s * q.y) / n, (s * q.z) / n], angle };
}

/** The exponential coordinates θ·n of eq. E.20 — the chart `SO3.coords` hands to a kd-tree. */
export function rotationVector(q: Quat): Vec3 {
  const { axis, angle } = axisAngle(q);
  return [axis[0] * angle, axis[1] * angle, axis[2] * angle];
}

export function quatFromRotationVector(v: Vec3): Quat {
  const angle = Math.hypot(v[0], v[1], v[2]);
  if (angle < 1e-15) return { ...QUAT_IDENTITY };
  return quatFromAxisAngle([v[0] / angle, v[1] / angle, v[2] / angle], angle);
}

/** Eq. E.28: the rotation matrix of a unit quaternion. */
export function quatToMatrix(q: Quat): Mat3x3 {
  const { w, x, y, z } = quatNormalize(q);
  return [
    [2 * (w * w + x * x) - 1, 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 2 * (w * w + y * y) - 1, 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 2 * (w * w + z * z) - 1],
  ];
}

/**
 * Inverse of E.28. Choset's eqs. E.29–E.30 use q₀ = ½√(1 + tr R), which loses
 * precision as θ → π; picking the largest diagonal term (Shepperd's method) is
 * the numerically safe version of the same algebra — and it is exactly the
 * "which chart Uᵢ applies" decision of App. E.4, made for numerical reasons.
 */
export function matrixToQuat(R: Mat3x3): Quat {
  const tr = R[0][0] + R[1][1] + R[2][2];
  let q: Quat;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = { w: s / 4, x: (R[2][1] - R[1][2]) / s, y: (R[0][2] - R[2][0]) / s, z: (R[1][0] - R[0][1]) / s };
  } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2;
    q = { w: (R[2][1] - R[1][2]) / s, x: s / 4, y: (R[0][1] + R[1][0]) / s, z: (R[0][2] + R[2][0]) / s };
  } else if (R[1][1] > R[2][2]) {
    const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2;
    q = { w: (R[0][2] - R[2][0]) / s, x: (R[0][1] + R[1][0]) / s, y: s / 4, z: (R[1][2] + R[2][1]) / s };
  } else {
    const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2;
    q = { w: (R[1][0] - R[0][1]) / s, x: (R[0][2] + R[2][0]) / s, y: (R[1][2] + R[2][1]) / s, z: s / 4 };
  }
  return quatNormalize(q);
}

/** Rotate a vector: Q (0, v) Q*, eq. E.35. */
export function rotateVec(q: Quat, v: Vec3): Vec3 {
  const p: Quat = { w: 0, x: v[0], y: v[1], z: v[2] };
  const r = quatMul(quatMul(q, p), quatConj(q));
  return [r.x, r.y, r.z];
}

/** Which of Choset's four quaternion charts U₀…U₃ the point lies in (largest |qᵢ|). */
export function quaternionChart(q: Quat): { index: 0 | 1 | 2 | 3; coords: Vec3 } {
  const comps = [q.w, q.x, q.y, q.z];
  let index: 0 | 1 | 2 | 3 = 0;
  for (let i = 1; i < 4; i++) if (Math.abs(comps[i]) > Math.abs(comps[index])) index = i as 0 | 1 | 2 | 3;
  const d = Math.abs(comps[index]);
  const rest = comps.filter((_, i) => i !== index).map((c) => c / d);
  return { index, coords: [rest[0], rest[1], rest[2]] };
}

/* -------------------------------------------------------------------------- */
/* Euler angles (§3.6, App. E.1)                                               */
/* -------------------------------------------------------------------------- */

export type EulerConvention = 'ZYZ' | 'ZYX';

export type EulerResult =
  | { ok: true; angles: [number, number, number] }
  | { ok: false; gimbalLock: true; /** the one angle that *is* determined: φ ± ψ (E.9/E.11), or φ ± ψ for ZYX */ sum: number };

const clamp1 = (x: number) => Math.min(1, Math.max(-1, x));

/** R = R_{z,φ} R_{y,θ} R_{z,ψ}, eqs. 3.6–3.8. */
export function eulerZYZToMatrix(phi: number, theta: number, psi: number): Mat3x3 {
  const cf = Math.cos(phi), sf = Math.sin(phi);
  const ct = Math.cos(theta), st = Math.sin(theta);
  const cp = Math.cos(psi), sp = Math.sin(psi);
  return [
    [cf * ct * cp - sf * sp, -cf * ct * sp - sf * cp, cf * st],
    [sf * ct * cp + cf * sp, -sf * ct * sp + cf * cp, sf * st],
    [-st * cp, st * sp, ct],
  ];
}

/** Roll–pitch–yaw, eq. E.12: R = R_{z,φ} R_{y,θ} R_{x,ψ}. Its lost chart is at θ = ±π/2. */
export function eulerZYXToMatrix(phi: number, theta: number, psi: number): Mat3x3 {
  const cf = Math.cos(phi), sf = Math.sin(phi);
  const ct = Math.cos(theta), st = Math.sin(theta);
  const cp = Math.cos(psi), sp = Math.sin(psi);
  return [
    [cf * ct, -sf * cp + cf * st * sp, sf * sp + cf * st * cp],
    [sf * ct, cf * cp + sf * st * sp, -cf * sp + sf * st * cp],
    [-st, ct * sp, ct * cp],
  ];
}

/**
 * Eqs. E.3, E.5, E.6 — the branch with sin θ > 0, θ ∈ (0, π). Returns the
 * gimbal-lock failure iff |R₃₃| = 1 within `eps`, carrying the one quantity that
 * survives: φ + ψ at R₃₃ = 1 (eq. E.9) or φ − ψ at R₃₃ = −1 (eq. E.11).
 */
export function eulerZYZ(R: Mat3x3, eps = 1e-9): EulerResult {
  const r33 = clamp1(R[2][2]);
  if (1 - Math.abs(r33) < eps) {
    const sum = r33 > 0 ? Math.atan2(R[1][0], R[0][0]) : Math.atan2(-R[0][1], -R[0][0]);
    return { ok: false, gimbalLock: true, sum };
  }
  const theta = Math.atan2(Math.sqrt(1 - r33 * r33), r33);
  const phi = Math.atan2(R[1][2], R[0][2]);
  const psi = Math.atan2(R[2][1], -R[2][0]);
  return { ok: true, angles: [phi, theta, psi] };
}

/** The same inverse for roll–pitch–yaw; lock when R₃₁ = ∓1, i.e. θ = ±π/2. */
export function eulerZYX(R: Mat3x3, eps = 1e-9): EulerResult {
  const r31 = clamp1(R[2][0]);
  if (1 - Math.abs(r31) < eps) {
    // θ = ∓π/2: only φ ∓ ψ is determined.
    const sum = Math.atan2(-R[0][1], R[1][1]);
    return { ok: false, gimbalLock: true, sum };
  }
  const theta = Math.atan2(-r31, Math.hypot(R[0][0], R[1][0]));
  const phi = Math.atan2(R[1][0], R[0][0]);
  const psi = Math.atan2(R[2][1], R[2][2]);
  return { ok: true, angles: [phi, theta, psi] };
}

export function eulerToMatrix(conv: EulerConvention, a: [number, number, number]): Mat3x3 {
  return conv === 'ZYZ' ? eulerZYZToMatrix(a[0], a[1], a[2]) : eulerZYXToMatrix(a[0], a[1], a[2]);
}

export function eulerFromMatrix(conv: EulerConvention, R: Mat3x3, eps = 1e-9): EulerResult {
  return conv === 'ZYZ' ? eulerZYZ(R, eps) : eulerZYX(R, eps);
}

/* -------------------------------------------------------------------------- */
/* The manifold                                                                */
/* -------------------------------------------------------------------------- */

/**
 * d(Q, Q′) = 2 acos |⟨Q, Q′⟩| ∈ [0, π]: the rotation angle between them, with
 * the double cover accounted for by the absolute value. Evaluated as
 * 4·atan2(‖Q − Q′‖, ‖Q + Q′‖) after flipping Q′ into Q's hemisphere: the same
 * number, but acos loses eight digits near 1 (dist(Q, Q) came out 6e−8), and
 * a metric that is not definite to machine precision breaks `equals`.
 */
export function so3Dist(a: Quat, b: Quat): number {
  const s = quatDot(a, b) < 0 ? -1 : 1;
  const dm = Math.hypot(a.w - s * b.w, a.x - s * b.x, a.y - s * b.y, a.z - s * b.z);
  const dp = Math.hypot(a.w + s * b.w, a.x + s * b.x, a.y + s * b.y, a.z + s * b.z);
  return 4 * Math.atan2(dm, dp);
}

/**
 * Slerp along the shorter great-circle arc of S³ *after* flipping b into the
 * hemisphere of a — the quaternion analogue of "the short way". At d = 0
 * (a rotation by exactly π between them) the arc is not unique; we take the
 * one through the representative with the sign flip applied, deterministically.
 */
export function so3Interpolate(a: Quat, b: Quat, t: number): Quat {
  let d = quatDot(a, b);
  let bb = b;
  if (d < 0) {
    bb = { w: -b.w, x: -b.x, y: -b.y, z: -b.z };
    d = -d;
  }
  d = clamp1(d);
  if (d > 1 - 1e-10) {
    // Nearly identical: nlerp is exact to first order and avoids 0/0.
    return quatNormalize({
      w: a.w + t * (bb.w - a.w),
      x: a.x + t * (bb.x - a.x),
      y: a.y + t * (bb.y - a.y),
      z: a.z + t * (bb.z - a.z),
    });
  }
  const omega = Math.acos(d);
  const so = Math.sin(omega);
  const ka = Math.sin((1 - t) * omega) / so;
  const kb = Math.sin(t * omega) / so;
  return quatNormalize({
    w: ka * a.w + kb * bb.w,
    x: ka * a.x + kb * bb.x,
    y: ka * a.y + kb * bb.y,
    z: ka * a.z + kb * bb.z,
  });
}

/** Haar-uniform: a normalized Gaussian 4-vector is uniform on S³, hence on SO(3). */
export function so3Sample(rng: Rng): Quat {
  return quatNormalize({ w: rng.normal(), x: rng.normal(), y: rng.normal(), z: rng.normal() });
}

/** Canonical representative: unit norm, w ≥ 0 (so the chart `coords` is single-valued for θ < π). */
export function so3Wrap(q: Quat): Quat {
  const n = quatNormalize(q);
  return n.w < 0 ? { w: -n.w, x: -n.x, y: -n.y, z: -n.z } : n;
}

export const SO3: Chart<Quat> = {
  name: 'SO(3)',
  dim: 3,
  periodic: [false, false, false],
  period: 2 * Math.PI,
  dist: so3Dist,
  interpolate: so3Interpolate,
  sample: so3Sample,
  wrap: so3Wrap,
  equals: (a, b, tol = 1e-9) => so3Dist(a, b) <= tol,
  /**
   * Exponential coordinates θn: a chart on {θ < π}, i.e. everywhere but the
   * rotations by exactly π. Not periodic in any coordinate — a kd-tree over it
   * is approximate near the boundary of the chart, which Chapter 11 discusses.
   */
  coords: (q) => rotationVector(q),
  fromCoords: (c) => quatFromRotationVector([c[0], c[1], c[2]]),
};

/** The axis–angle θ of RᵀR′ computed from matrices — the independent route the check compares to `so3Dist`. */
export function rotationAngleBetween(Ra: Mat3x3, Rb: Mat3x3): number {
  // tr(RaᵀRb) = Σᵢⱼ Ra[i][j]·Rb[i][j]
  let tr = 0;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) tr += Ra[i][j] * Rb[i][j];
  return Math.acos(clamp1((tr - 1) / 2));
}

/**
 * Expected rotation angle of a Haar-uniform rotation: the angle has density
 * (1 − cos θ)/π on [0, π], whose mean is π/2 + 2/π ≈ 2.2074. A sampler that
 * were uniform in Euler angles would miss this by a wide margin.
 */
export const HAAR_MEAN_ANGLE = Math.PI / 2 + 2 / Math.PI;
