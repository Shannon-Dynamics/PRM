/**
 * The curve tracer — THE OWNED ARTIFACT of Chapter 3 (Choset §2.3.3, eq. 2.5,
 * Appendix D). Chapters 7–10 import `traceCurve` to follow level sets, Voronoi
 * edges, the GVG, and critical-point slices; here it follows an obstacle
 * boundary at a fixed offset W* as the zero set of G(x) = D(x) − W*.
 *
 * It is a predictor–corrector:
 *
 *   predict   y₀ = x + Δλ·t,          t ⊥ ∇G(x)      (the tangent, §2.3.1)
 *   correct   y_{h+1} = y_h − G(y_h) / D_Y G(y_h),  along the correcting line
 *             through y₀ orthogonal to t, with D_Y G = ∇G(y_h)·n, n = ∇G(x)/|∇G(x)|
 *
 * The implicit function theorem (D.1.1) says the zero set *is* locally a
 * curve y(λ) wherever D_Y G ≠ 0; the Newton–Raphson theorem (D.2.1) says the
 * corrector converges quadratically from inside a ball whose radius is set by
 * the Lipschitz constant of DG. When D_Y G collapses the hypothesis has
 * failed and the tracer says so — `stop: 'singular'` — instead of guessing.
 * That honesty is the whole reason this is a separate, reusable module.
 */

import type { Point2 } from '../sim/world';
import { add, dist, dot, norm, perp, pt, scale, sub, type Turn } from './geometry';

export type ScalarField = (x: Point2) => number;
export type GradientField = (x: Point2) => Point2;

export interface TraceParams {
  /** Δλ: the predictor step along the tangent. */
  step: number;
  /** Newton iterations per correction (3 is usually one too many). */
  newtonIters?: number;
  /** Stop correcting once |G| falls below this. */
  tol?: number;
  /** |D_Y G| below this means the correcting line is tangent to the curve: singular. */
  singularTol?: number;
  /** Give up after this many predictor steps. */
  maxSteps?: number;
}

export type TraceStop = 'closed' | 'reached' | 'singular' | 'budget' | 'diverged';

/** One predictor–corrector step, kept whole so a widget can draw every hop. */
export interface TraceRecord {
  /** The point the step started from (on the curve). */
  from: Point2;
  /** Unit tangent used for the prediction. */
  tangent: Point2;
  /** The predicted point x + Δλ·t. */
  predicted: Point2;
  /** Successive Newton iterates y₁, y₂, … (y₀ is `predicted`). */
  corrections: Point2[];
  /** |G(y_h)| for h = 0, 1, 2, … — the squares the chapter asks you to watch. */
  residuals: number[];
  /** D_Y G at the predicted point. */
  dyG: number;
}

export interface Trace {
  /** The samples on the curve, starting at the corrected x₀. */
  points: Point2[];
  records: TraceRecord[];
  stop: TraceStop;
  /** Polyline length of `points` (plus the closing chord when the curve closed). */
  length: number;
  /** Largest |G| over the samples that were kept. */
  maxResidual: number;
}

const DEFAULTS = { newtonIters: 3, tol: 1e-12, singularTol: 0.2, maxSteps: 5000 };

/** The tangent (∇G)^⊥ oriented by `turn` — see `tangentOf` in scan.ts for the convention. */
export function tangentFromGradient(grad: Point2, turn: Turn): Point2 {
  const g = norm(grad);
  if (g < 1e-15) return pt(0, 0);
  const t = perp(scale(grad, 1 / g));
  return turn === 'left' ? pt(-t.x, -t.y) : t;
}

/**
 * Newton along a fixed direction n: the corrector of eq. 2.5 with D_Y G the
 * scalar ∇G·n. Returns the iterates and residuals; `singular` is set when the
 * directional derivative vanished before the tolerance was met.
 */
export function correctAlong(
  G: ScalarField,
  DG: GradientField,
  y0: Point2,
  n: Point2,
  p: Required<Pick<TraceParams, 'newtonIters' | 'tol' | 'singularTol'>>,
): { y: Point2; corrections: Point2[]; residuals: number[]; dyG: number; singular: boolean } {
  let y = y0;
  const corrections: Point2[] = [];
  const residuals: number[] = [Math.abs(G(y0))];
  const dyG0 = dot(DG(y0), n);
  let singular = false;
  for (let h = 0; h < p.newtonIters; h++) {
    const g = G(y);
    if (Math.abs(g) <= p.tol) break;
    const dyG = dot(DG(y), n);
    if (Math.abs(dyG) < p.singularTol) {
      singular = true;
      break;
    }
    y = sub(y, scale(n, g / dyG));
    corrections.push(y);
    residuals.push(Math.abs(G(y)));
  }
  return { y, corrections, residuals, dyG: dyG0, singular };
}

/**
 * The plain Newton step in the full gradient direction — the robust fallback
 * a *follower* uses when the fixed-plane corrector is singular (an inside
 * corner of the obstacle, where ∇D jumps). It is not the tracer's own rule and
 * `traceCurve` never uses it: the tracer's contract is to report the failure.
 */
export function correctGradient(G: ScalarField, DG: GradientField, y0: Point2, iters: number, tol: number): Point2 {
  let y = y0;
  for (let h = 0; h < iters; h++) {
    const g = G(y);
    if (Math.abs(g) <= tol) break;
    const grad = DG(y);
    const n2 = dot(grad, grad);
    if (n2 < 1e-18) break;
    y = sub(y, scale(grad, g / n2));
  }
  return y;
}

/** One predictor–corrector step from a point on the curve. */
export function traceStep(
  G: ScalarField,
  DG: GradientField,
  x: Point2,
  turn: Turn,
  params: TraceParams,
): { x: Point2; record: TraceRecord; singular: boolean } {
  const p = { ...DEFAULTS, ...params };
  const grad = DG(x);
  const gn = norm(grad);
  const n = gn > 1e-15 ? scale(grad, 1 / gn) : pt(0, 0);
  const tangent = tangentFromGradient(grad, turn);
  const predicted = add(x, scale(tangent, p.step));
  const c = correctAlong(G, DG, predicted, n, p);
  return {
    x: c.y,
    singular: c.singular || gn < 1e-15,
    record: { from: x, tangent, predicted, corrections: c.corrections, residuals: c.residuals, dyG: c.dyG },
  };
}

/**
 * Trace the zero set of G from x₀ in the direction `turn` until it closes on
 * itself, `stop(x)` fires, the corrector goes singular, or the budget runs out.
 * x₀ need not be on the curve: it is first Newton-corrected along ∇G(x₀).
 */
export function traceCurve(
  G: ScalarField,
  DG: GradientField,
  x0: Point2,
  turn: Turn,
  params: TraceParams,
  stop?: (x: Point2) => boolean,
): Trace {
  const p = { ...DEFAULTS, ...params };
  const start = correctGradient(G, DG, x0, 8, p.tol);
  const points: Point2[] = [start];
  const records: TraceRecord[] = [];
  let x = start;
  let length = 0;
  let maxResidual = Math.abs(G(start));
  let outcome: TraceStop = 'budget';

  for (let k = 0; k < p.maxSteps; k++) {
    const s = traceStep(G, DG, x, turn, p);
    if (s.singular) {
      records.push(s.record);
      outcome = 'singular';
      break;
    }
    const hop = dist(x, s.x);
    if (!Number.isFinite(hop) || hop > 4 * p.step) {
      records.push(s.record);
      outcome = 'diverged';
      break;
    }
    x = s.x;
    length += hop;
    points.push(x);
    records.push(s.record);
    maxResidual = Math.max(maxResidual, Math.abs(G(x)));
    if (stop && stop(x)) {
      outcome = 'reached';
      break;
    }
    // Closure: back within a step of the start after having gone somewhere.
    if (length > 3 * p.step && dist(x, start) < 0.75 * p.step) {
      length += dist(x, start);
      points.push(start);
      outcome = 'closed';
      break;
    }
  }
  return { points, records, stop: outcome, length, maxResidual };
}
