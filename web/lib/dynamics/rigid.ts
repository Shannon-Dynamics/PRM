/**
 * Rigid-body rotation (Choset §10.4): planar inertia and the parallel-axis
 * theorem, the spatial inertia tensor, principal axes, composite bodies, and
 * Euler's equation integrated in the body frame,
 *
 *     Ṙ = R ω̂,      τ = ω̂ 𝓘 ω + 𝓘 ω̇                              (10.43–10.44)
 *
 * The orientation is a unit quaternion from Chapter 5's `manifold/so3`, so
 * R ← R exp(h ω̂) is one quaternion product and never drifts off SO(3). The
 * angular momentum 𝓘_s ω_s = R 𝓘 ω and the kinetic energy ½ ωᵀ𝓘ω are the two
 * conserved quantities the torque-free check pins; the intermediate-axis
 * flip of w17.3 is Choset's "ω̇ may not be zero even if τ is zero" made visible.
 */

import { quatMul, quatToMatrix, type Mat3x3, type Quat } from '../manifold/so3';
import { inv, matVec, type Mat, type Vec } from '../prob/linalg';

export type Vec3 = [number, number, number];

/* --------------------------------- planar --------------------------------- */

/** I_zz of a uniform rod of length L about its center: mL²/12. */
export const rodInertia = (m: number, L: number) => (m * L * L) / 12;

/** I_zz of a uniform disc of radius R about its center: mR²/2. */
export const discInertia = (m: number, R: number) => (m * R * R) / 2;

/** Parallel-axis theorem for planar rotation (10.33): I_z'z' = I_zz + m d². */
export const parallelAxis = (Izz: number, m: number, d: number) => Izz + m * d * d;

/* --------------------------------- spatial -------------------------------- */

/** Principal-axis inertia tensors of Figure 10.5 (uniform density). */
export const brickInertia = (m: number, a: number, b: number, c: number): Mat => [
  [(m * (b * b + c * c)) / 12, 0, 0],
  [0, (m * (a * a + c * c)) / 12, 0],
  [0, 0, (m * (a * a + b * b)) / 12],
];

export const cylinderInertia = (m: number, R: number, h: number): Mat => [
  [(m * (3 * R * R + h * h)) / 12, 0, 0],
  [0, (m * (3 * R * R + h * h)) / 12, 0],
  [0, 0, (m * R * R) / 2],
];

export const sphereInertia = (m: number, R: number): Mat => [
  [(2 * m * R * R) / 5, 0, 0],
  [0, (2 * m * R * R) / 5, 0],
  [0, 0, (2 * m * R * R) / 5],
];

/** Spatial parallel-axis theorem (10.46): 𝓘' = 𝓘 + m(‖r‖² 1 − r rᵀ). */
export function parallelAxis3(I: Mat, m: number, r: Vec3): Mat {
  const rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
  return I.map((row, i) => row.map((v, j) => v + m * ((i === j ? rr : 0) - r[i] * r[j])));
}

/** Rotate a tensor into another frame: 𝓘'' = R 𝓘 Rᵀ (10.41 read backwards). */
export function rotateInertia(I: Mat, R: Mat): Mat {
  const Rt = R[0].map((_, c) => R.map((row) => row[c]));
  const tmp = mul3(R, I);
  return mul3(tmp, Rt);
}

function mul3(A: Mat, B: Mat): Mat {
  return A.map((row) => B[0].map((_, j) => row.reduce((s, v, k) => s + v * B[k][j], 0)));
}

export const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

/** ω̂ — the skew matrix with ω̂ v = ω × v. */
export const hat = (w: Vec3): Mat => [
  [0, -w[2], w[1]],
  [w[2], 0, -w[0]],
  [-w[1], w[0], 0],
];

/** ω̇ = 𝓘⁻¹(τ − ω × 𝓘ω): Euler's equation in the body frame solved for ω̇. */
export function eulerBodyAccel(I: Mat, omega: Vec3, tau: Vec3): Vec3 {
  const Iw = matVec(I, omega) as Vec3;
  const gyro = cross(omega, Iw);
  const rhs: Vec3 = [tau[0] - gyro[0], tau[1] - gyro[1], tau[2] - gyro[2]];
  return matVec(inv(I), rhs) as Vec3;
}

/** The principal-axis form (10.45), for a diagonal 𝓘. */
export function eulerPrincipal(Ixx: number, Iyy: number, Izz: number, w: Vec3, tau: Vec3): Vec3 {
  return [
    (tau[0] - (Izz - Iyy) * w[1] * w[2]) / Ixx,
    (tau[1] - (Ixx - Izz) * w[0] * w[2]) / Iyy,
    (tau[2] - (Iyy - Ixx) * w[0] * w[1]) / Izz,
  ];
}

export interface RigidState {
  /** Body-to-spatial orientation R as a unit quaternion. */
  R: Quat;
  /** Body-frame angular velocity ω. */
  omega: Vec3;
}

/**
 * One RK4 step of (10.43)–(10.44) on the seven-number state (R as a unit
 * quaternion, ω). The quaternion kinematics are q̇ = ½ q ⊗ (0, ω) — the
 * right-multiplied body-frame form of Ṙ = R ω̂ — and the step is renormalized
 * at the end, so the orientation never leaves SO(3) and the global error is
 * fourth order in both R and ω. That is what lets the torque-free check hold
 * angular momentum in the *spatial* frame to 1e-6 over thousands of steps.
 */
export function eulerBodyStep(state: RigidState, tau: Vec3, I: Mat, h: number): RigidState {
  const Iinv = inv(I);
  const f = (x: number[]): number[] => {
    const q: Quat = { w: x[0], x: x[1], y: x[2], z: x[3] };
    const w: Vec3 = [x[4], x[5], x[6]];
    const dq = quatMul(q, { w: 0, x: w[0], y: w[1], z: w[2] });
    const Iw = matVec(I, w) as Vec3;
    const gyro = cross(w, Iw);
    const dw = matVec(Iinv, [tau[0] - gyro[0], tau[1] - gyro[1], tau[2] - gyro[2]]);
    return [0.5 * dq.w, 0.5 * dq.x, 0.5 * dq.y, 0.5 * dq.z, dw[0], dw[1], dw[2]];
  };
  const x0 = [state.R.w, state.R.x, state.R.y, state.R.z, state.omega[0], state.omega[1], state.omega[2]];
  const k1 = f(x0);
  const k2 = f(x0.map((v, i) => v + (h / 2) * k1[i]));
  const k3 = f(x0.map((v, i) => v + (h / 2) * k2[i]));
  const k4 = f(x0.map((v, i) => v + h * k3[i]));
  const x = x0.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  const nrm = Math.hypot(x[0], x[1], x[2], x[3]);
  return {
    R: { w: x[0] / nrm, x: x[1] / nrm, y: x[2] / nrm, z: x[3] / nrm },
    omega: [x[4], x[5], x[6]],
  };
}

/** K = ½ ωᵀ 𝓘 ω (10.40). */
export function rotationalKineticEnergy(I: Mat, omega: Vec3): number {
  const Iw = matVec(I, omega);
  return 0.5 * (omega[0] * Iw[0] + omega[1] * Iw[1] + omega[2] * Iw[2]);
}

/** Angular momentum in the spatial frame, 𝓘_s ω_s = R 𝓘 ω — constant when τ = 0. */
export function spatialAngularMomentum(state: RigidState, I: Mat): Vec3 {
  const Iw = matVec(I, state.omega);
  const R: Mat3x3 = quatToMatrix(state.R);
  return [
    R[0][0] * Iw[0] + R[0][1] * Iw[1] + R[0][2] * Iw[2],
    R[1][0] * Iw[0] + R[1][1] * Iw[1] + R[1][2] * Iw[2],
    R[2][0] * Iw[0] + R[2][1] * Iw[1] + R[2][2] * Iw[2],
  ];
}

/** Which principal axis (0, 1, 2) is intermediate — the one about which spin is unstable. */
export function intermediateAxis(I: Mat): number {
  const d = [I[0][0], I[1][1], I[2][2]];
  const order = [0, 1, 2].sort((a, b) => d[a] - d[b]);
  return order[1];
}
