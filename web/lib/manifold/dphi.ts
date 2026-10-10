/**
 * Dφ — Reach's Jacobian read as the differential of the forward map
 * (Choset §3.8, Example 3.8.1; App. C.5).
 *
 * Chapter 4 computed J(q) for the 2R arm. This chapter's point is that J is the
 * *matrix* of a chart-independent linear map Dφ_q : T_qT² → T_{φ(q)}ℝ², and
 * the chain rule (C.5) says how that matrix changes under a chart change h:
 * D(φ∘h) = Dφ·Dh. For the torus's angle atlas the transition maps are
 * translations by 2π with Dh = I, so J is literally the same matrix in every
 * chart — which is what `dphiChartShiftInvariance` measures.
 *
 * Chapter 4 owns the Jacobian proper; this file only re-states the 2R case
 * needed for Derivation 5 and the check, with Choset's numbers.
 */

import { TAU } from './s1';

export interface Arm2R {
  L1: number;
  L2: number;
}

export const CHOSET_ARM: Arm2R = { L1: 1, L2: 1 };

/** Forward kinematics φ : T² → ℝ², the fingertip position. */
export function fk2R(arm: Arm2R, q: [number, number]): [number, number] {
  const [t1, t2] = q;
  return [
    arm.L1 * Math.cos(t1) + arm.L2 * Math.cos(t1 + t2),
    arm.L1 * Math.sin(t1) + arm.L2 * Math.sin(t1 + t2),
  ];
}

/** The two link segments in the plane, base at the origin — for the Workbench raster. */
export function links2R(arm: Arm2R, q: [number, number]): [[number, number], [number, number], [number, number]] {
  const [t1, t2] = q;
  const elbow: [number, number] = [arm.L1 * Math.cos(t1), arm.L1 * Math.sin(t1)];
  const tip: [number, number] = [elbow[0] + arm.L2 * Math.cos(t1 + t2), elbow[1] + arm.L2 * Math.sin(t1 + t2)];
  return [[0, 0], elbow, tip];
}

export type Mat2 = [[number, number], [number, number]];

/** J(q) = ∂φ/∂q, Choset's matrix in §3.8. */
export function jacobian2R(arm: Arm2R, q: [number, number]): Mat2 {
  const [t1, t2] = q;
  const s1 = Math.sin(t1), c1 = Math.cos(t1);
  const s12 = Math.sin(t1 + t2), c12 = Math.cos(t1 + t2);
  return [
    [-arm.L1 * s1 - arm.L2 * s12, -arm.L2 * s12],
    [arm.L1 * c1 + arm.L2 * c12, arm.L2 * c12],
  ];
}

/** Dφ_q applied to a tangent vector q̇: ẋ = J(q) q̇. */
export function dphi(arm: Arm2R, q: [number, number], qdot: [number, number]): [number, number] {
  const J = jacobian2R(arm, q);
  return [J[0][0] * qdot[0] + J[0][1] * qdot[1], J[1][0] * qdot[0] + J[1][1] * qdot[1]];
}

/**
 * Change the chart of θ₁ by a full turn (the transition map of the angle atlas,
 * h(θ₁, θ₂) = (θ₁ + 2π, θ₂), Dh = I) and report the largest change in any
 * Jacobian entry. Mathematically zero; numerically ~1e−15.
 */
export function dphiChartShiftInvariance(arm: Arm2R, q: [number, number]): number {
  const J0 = jacobian2R(arm, q);
  const J1 = jacobian2R(arm, [q[0] + TAU, q[1]]);
  let worst = 0;
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) worst = Math.max(worst, Math.abs(J0[i][j] - J1[i][j]));
  return worst;
}

/**
 * Example C.5.1 as a worked check of the chain rule: c(t) = (2, 2πt) in polar
 * coordinates, h polar→Cartesian, g(x) = x₁; d/dt (g∘h∘c) = −4π sin(2πt).
 */
export function chainRuleExample(t: number): { chainRule: number; direct: number } {
  const y1 = 2;
  const y2 = TAU * t;
  // ∂g/∂h · ∂h/∂c · ċ with ∂g/∂h = [1 0], ∂h/∂c = [[cos y₂, −y₁ sin y₂],[sin y₂, y₁ cos y₂]], ċ = [0, 2π]
  const chainRule = 1 * (Math.cos(y2) * 0 + -y1 * Math.sin(y2) * TAU);
  const direct = -2 * TAU * Math.sin(y2); // d/dt 2cos(2πt)
  return { chainRule, direct };
}
