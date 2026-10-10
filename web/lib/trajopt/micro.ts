/**
 * The chapter's presets, shared by the checks and the widgets.
 *
 *   microChomp        the three-waypoint point robot of §3: q₀ = (0, 0), q₂ = (2, 0),
 *                     free q₁ = (1, 0.2), one disc at (1, 0) of radius ½, ε = ½, λ = 1, η = 1.
 *   minEffort         Choset Example 11.3.1 discretized: q̈ = u, d = 1, t_f = 1, N = 100.
 *   reachWorkbench    Reach on the Workbench: body points along both links, the
 *                     brushfire field of the Workbench, and two initial paths
 *                     around the block — one per homotopy class.
 */

import { Reach } from '../robots/reach';
import { WORKBENCH } from '../robots/worlds';
import { controlPointJacobian, controlPointPosition } from '../potential/lifted';
import type { Vec } from '../prob/linalg';
import { POINT_BODY, chompStableEta, straightLine, type BodyPoint, type ChompProblem } from './chomp';
import { DiscField, GridField, type DistanceField2 } from './field';
import { diagMat, quadraticCost, type StageCost } from './ilqr';
import { DoubleIntegrator } from './models';

export function microChomp(lambdaObs = 1, eps = 0.5, eta = 1, form: 'pointwise' | 'arclength' = 'pointwise'): { p: ChompProblem; xi0: Vec[] } {
  return {
    p: { q0: [0, 0], qEnd: [2, 0], field: new DiscField({ x: 1, y: 0 }, 0.5), body: POINT_BODY, eps, lambdaObs, eta, obstacleForm: form },
    xi0: [[1, 0.2]],
  };
}

/**
 * The covariant update on the micro-example is y ← y(1 − (1 + λ)/η) + λ/η inside the
 * margin, a contraction iff η > (1 + λ)/2. Below that it oscillates — at η = 1 and
 * λ = 2 between the disc centre and the edge of the margin — so "any η ≥ 1" is not
 * enough; the chapter states this bound and Exercise 1 derives it.
 */
export const stableEta = (lambda: number): number => chompStableEta(1, lambda, 0.5);

/** The equilibrium of the micro-example, y* = λ/(1 + λ), and its clearance D* = (λ − 1)/(2(1 + λ)). */
export const microEquilibrium = (lambda: number) => ({ y: lambda / (1 + lambda), clearance: (lambda - 1) / (2 * (1 + lambda)) });

/** Choset Example 11.3.1 discretized: minimum effort, rest to rest over d in t_f. */
export function minEffort(N = 100, tf = 1, d = 1, terminalWeight = 1e6): { f: DoubleIntegrator; cost: StageCost; x0: Vec; uInit: Vec[]; dt: number } {
  const dt = tf / N;
  const f = new DoubleIntegrator(dt);
  // ℓ = u² Δt (Choset's L = u², integrated), terminal weight on (q, q̇) = (d, 0).
  const cost = quadraticCost(diagMat([0, 0]), [[2 * dt]], diagMat([2 * terminalWeight, 2 * terminalWeight]), [d, 0]);
  return { f, cost, x0: [0, 0], uInit: Array.from({ length: N }, () => [0]), dt };
}

/** Pontryagin's extremal control for the min-effort double integrator: u(t) = 6d/t_f² − 12d t/t_f³. */
export const pontryaginControl = (t: number, d = 1, tf = 1): number => (6 * d) / (tf * tf) - (12 * d * t) / (tf * tf * tf);

/** Body points along both links of Reach (m per link), with Jacobians from Chapter 7's lift. */
export function reachBodyPoints(arm: Reach, perLink = 4): BodyPoint[] {
  const out: BodyPoint[] = [];
  for (let link = 0; link < arm.n; link++) {
    for (let k = 1; k <= perLink; k++) {
      const t = k / perLink;
      out.push((q) => ({ x: controlPointPosition(arm, q, link, t), J: controlPointJacobian(arm, q, link, t) }));
    }
  }
  return out;
}

export interface ReachScene {
  arm: Reach;
  field: DistanceField2;
  body: BodyPoint[];
  q0: Vec;
  qEnd: Vec;
  /** Two initial paths: the straight line (elbow passing one side of the block) and a detour through the other class. */
  initial: { name: string; xi: Vec[] }[];
}

/**
 * Reach reaching from the lower-right to the upper-left of the Workbench past
 * the block, with the brushfire field at 5 cm. The straight chart line and a
 * via-point detour start in different homotopy classes around the block in
 * the workspace; CHOMP polishes whichever it is handed and never crosses over.
 */
export function reachWorkbench(m = 24, cellSize = 0.05): ReachScene {
  const arm = Reach.workbench();
  const field = GridField.fromPolyWorld(WORKBENCH, cellSize);
  const q0: Vec = [-0.3, 0.9];
  const qEnd: Vec = [1.9, 0.6];
  const straight = straightLine(q0, qEnd, m);
  // A detour via a folded configuration that swings the elbow under the block.
  const via: Vec = [0.8, 2.3];
  const detour: Vec[] = [];
  const half = Math.floor(m / 2);
  for (let k = 1; k <= m; k++) {
    if (k <= half) detour.push(q0.map((v, i) => v + (k / (half + 1)) * (via[i] - v)));
    else detour.push(via.map((v, i) => v + ((k - half) / (m - half + 1)) * (qEnd[i] - v)));
  }
  return { arm, field, body: reachBodyPoints(arm, 4), q0, qEnd, initial: [{ name: 'straight line in the chart', xi: straight }, { name: 'detour via a folded elbow', xi: detour }] };
}
