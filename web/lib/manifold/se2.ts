/**
 * SE(2) as a manifold — the ported `Pose2` with a metric attached.
 *
 * The type is not redefined: `Pose2` from `geom/se2.ts` is the group element,
 * with `compose`, `inverse`, and `transformPoint` already there (the sister
 * book's Chapter 3 derives them as a Lie group). What Chapter 5 adds is the
 * planner's view: a *distance* between poses, which needs an exchange rate
 * between metres and radians. That rate `w` has no canonical value — the text
 * says so, and Chapter 11 measures what a PRM does as it varies — so it is a
 * constructor argument here, never a hidden default.
 *
 * The second half is Choset's three-frame example (§3.5.1, figures 3.17–3.19)
 * restricted to the plane: T_AB T_BC = T_AC, the point w in each frame, and
 * body- versus world-frame displacement of B by T₁. It is the one place in
 * Choset where SE(n) can be checked by hand, and the chapter pins every number.
 */

import {
  compose,
  inverse,
  normalizeAngle,
  transformPoint,
  type Pose2,
} from '../geom/se2';
import type { Rng } from '../prob/rng';
import type { Chart } from './manifold';
import { s1Dist, s1Interpolate, TAU } from './s1';

export interface Aabb {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

export class SE2 implements Chart<Pose2> {
  readonly name: string;
  readonly dim = 3;
  readonly periodic: readonly boolean[] = [false, false, true];
  readonly period = TAU;

  /**
   * @param w       metres per radian — how much a full radian of heading change
   *                "costs" relative to a metre of translation.
   * @param bounds  sampling box for (x, y); θ is sampled uniformly on S¹.
   */
  constructor(
    readonly w: number,
    readonly bounds: Aabb,
  ) {
    this.name = `SE(2), w=${w}`;
  }

  /** d^w = √(Δx² + Δy² + w²·d_{S¹}(θ, θ')²). A metric for every w > 0 (Derivation 4). */
  dist(a: Pose2, b: Pose2): number {
    return Math.hypot(a.x - b.x, a.y - b.y, this.w * s1Dist(a.theta, b.theta));
  }

  /** Straight in (x, y), short way in θ — the geodesic of the product metric. */
  interpolate(a: Pose2, b: Pose2, t: number): Pose2 {
    return {
      x: a.x + t * (b.x - a.x),
      y: a.y + t * (b.y - a.y),
      theta: s1Interpolate(a.theta, b.theta, t),
    };
  }

  sample(rng: Rng): Pose2 {
    return {
      x: rng.uniform(this.bounds.minX, this.bounds.maxX),
      y: rng.uniform(this.bounds.minY, this.bounds.maxY),
      theta: rng.uniform(-Math.PI, Math.PI),
    };
  }

  wrap(p: Pose2): Pose2 {
    return { x: p.x, y: p.y, theta: normalizeAngle(p.theta) };
  }

  equals(a: Pose2, b: Pose2, tol = 1e-9): boolean {
    return this.dist(a, b) <= tol;
  }

  /** Chart coordinates (x, y, θ): the kd-tree splits on these, with θ periodic. */
  coords(p: Pose2): number[] {
    return [p.x, p.y, normalizeAngle(p.theta)];
  }

  fromCoords(c: number[]): Pose2 {
    return { x: c[0], y: c[1], theta: normalizeAngle(c[2]) };
  }
}

/* -------------------------------------------------------------------------- */
/* Matrices, for the Frame Composer's readout                                 */
/* -------------------------------------------------------------------------- */

export type Mat3 = [[number, number, number], [number, number, number], [number, number, number]];

/** The homogeneous matrix [R p; 0 1] of a planar pose — Choset's T, in 3 × 3. */
export function toMatrix(p: Pose2): Mat3 {
  const c = Math.cos(p.theta);
  const s = Math.sin(p.theta);
  return [
    [c, -s, p.x],
    [s, c, p.y],
    [0, 0, 1],
  ];
}

/** Clean up −0 and 1e−17 so a readout shows Choset's integers as integers. */
export function tidy(x: number, eps = 1e-9): number {
  const r = Math.round(x);
  return Math.abs(x - r) < eps ? r + 0 : x;
}

/* -------------------------------------------------------------------------- */
/* Choset's frames, §3.5.1                                                     */
/* -------------------------------------------------------------------------- */

export interface ChosetFrames {
  /** Frame B in A: R = rot(π), p = (−2, 0). */
  T_AB: Pose2;
  /** Frame C in B: R = rot(−π/2), p = (−4, −1). */
  T_BC: Pose2;
  /** The point w in C's coordinates. */
  w_C: [number, number];
  /** The displacement of figure 3.19. */
  T1: Pose2;
}

export const CHOSET_FRAMES: ChosetFrames = {
  T_AB: { x: -2, y: 0, theta: Math.PI },
  T_BC: { x: -4, y: -1, theta: -Math.PI / 2 },
  w_C: [-2, 1],
  T1: { x: 0, y: 1, theta: Math.PI },
};

export interface FrameWorked {
  T_AC: Pose2;
  w_B: [number, number];
  w_A: [number, number];
  /** T_AB w_A — the *displacement* of w by the motion carrying A to B (figure 3.18). */
  w_displaced: [number, number];
  /** T_AB T₁ — body-frame transformation: B′. */
  body: Pose2;
  /** T₁ T_AB — world-frame transformation: B″. */
  world: Pose2;
  /** T_AB⁻¹ T_AC = T_BC, the round trip. */
  T_BC_recovered: Pose2;
}

/**
 * Subscript cancellation as code: T_AB T_BC = T_AC, w_B = T_BC w_C,
 * w_A = T_AB w_B; then the two placements of B under T₁, which differ because
 * composition does not commute.
 */
export function frameWorked(f: ChosetFrames = CHOSET_FRAMES): FrameWorked {
  const T_AC = compose(f.T_AB, f.T_BC);
  const w_B = transformPoint(f.T_BC, f.w_C);
  const w_A = transformPoint(f.T_AB, w_B);
  const w_displaced = transformPoint(f.T_AB, w_A);
  return {
    T_AC,
    w_B,
    w_A,
    w_displaced,
    body: compose(f.T_AB, f.T1),
    world: compose(f.T1, f.T_AB),
    T_BC_recovered: compose(inverse(f.T_AB), T_AC),
  };
}
