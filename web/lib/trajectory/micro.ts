/**
 * The chapter's micro-example, as a preset every widget and check shares.
 *
 * Reach from the Chapter 17 micro-example (L = 1, 1; m = 2, 1; uniform rods)
 * on a horizontal Workbench (a_g = 0), joint 2 held at π/2 while joint 1
 * sweeps a quarter turn: q(s) = (πs/2, π/2). Limits |τ₁| ≤ 20, |τ₂| ≤ 12 N·m.
 * Then a = (π, π/6), b = (0, 1.2337), c = 0, v(s) = 3.5254 for every s, one
 * switch at s = ½, t_s = 0.3963 s, t_f = 0.7927 s — Choset's Example 11.2.1
 * situation (the limit curve is never touched) on this book's arm.
 */

import { Reach2R } from '../dynamics/reach2r';
import { PathDynamics } from './constrained';
import { symmetricLimits, type TorqueLimits } from './limits';
import { CubicSpline, Segment, type Path } from './path';

export const MICRO_LIMITS: TorqueLimits = symmetricLimits([20, 12]);

export function microPath(): Path {
  return new Segment([0, Math.PI / 2], [Math.PI / 2, Math.PI / 2]);
}

export function microArm(aG = 0): Reach2R {
  return Reach2R.micro(aG);
}

export function microProblem(aG = 0): { pd: PathDynamics; lim: TorqueLimits } {
  return { pd: new PathDynamics(microArm(aG), microPath()), lim: MICRO_LIMITS };
}

/**
 * The widgets' default spline: Reach hanging below the shoulder on a vertical
 * Workbench, three waypoints on T², limits 20/12 N·m. Solvable under gravity,
 * with the limit curve active (three switches at the default limits and a
 * singular slide when the limits are raised), so the phase plane has something
 * to show. The replay check runs on the same path.
 */
export const RACER_WAYPOINTS: readonly (readonly [number, number])[] = [
  [-2.2, 1.2],
  [-1.5, 1.9],
  [-0.9, 1.1],
];

export function racerProblem(aG = 9.81, waypoints: readonly (readonly number[])[] = RACER_WAYPOINTS): { pd: PathDynamics; lim: TorqueLimits; path: CubicSpline } {
  const path = CubicSpline.onTorus(waypoints.map((w) => [...w]));
  return { pd: new PathDynamics(microArm(aG), path), lim: MICRO_LIMITS, path };
}

/**
 * The "other clock" of the hook: a trapezoidal time scaling with `ramp`
 * fraction of t_f spent accelerating and decelerating and constant ṡ between.
 * Returns (s, ṡ, s̈) at time t. A literally constant ṡ has an impulsive start,
 * so this is what "run the path in t_f seconds at constant speed" honestly means.
 */
export function trapezoidPhase(tf: number, t: number, ramp = 0.1): { s: number; sdot: number; sddot: number } {
  const tr = ramp * tf;
  const vc = 1 / (tf - tr); // cruise ṡ: area under the trapezoid is 1
  const acc = vc / tr;
  const tt = Math.min(tf, Math.max(0, t));
  if (tt < tr) return { s: 0.5 * acc * tt * tt, sdot: acc * tt, sddot: acc };
  if (tt <= tf - tr) return { s: 0.5 * vc * tr + vc * (tt - tr), sdot: vc, sddot: 0 };
  const tau = tf - tt;
  return { s: 1 - 0.5 * acc * tau * tau, sdot: acc * tau, sddot: -acc };
}
