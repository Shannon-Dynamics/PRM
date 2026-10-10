/**
 * The Morse definition box — introduced here, imported by Chapters 9–10.
 *
 * A smooth real-valued f is *Morse* if every critical point is nondegenerate
 * (nonsingular Hessian). Nondegenerate critical points are isolated and carry
 * an *index*: the number of negative Hessian eigenvalues — 0 a minimum, dim M
 * a maximum, anything else a saddle. The fact Chapters 9–10 lean on: the
 * topology of the level sets f⁻¹(λ) changes only as λ crosses a critical
 * value, and the alternating count Σ(−1)^index equals the Euler characteristic
 * of the manifold.
 *
 * The box's example is the height function h(θ₁, θ₂) = cos θ₁ + cos θ₂ on
 * Reach's torus T²: four critical points — one maximum, two saddles, one
 * minimum — and 1 − 2 + 1 = 0 = χ(T²). The preimage theorem's warm-up,
 * f(x, y) = x² + y² with the regular value 91,538, is here too: Df ≠ 0 on the
 * whole level set, so it is a one-manifold (a circle).
 *
 * Chapter 7's Hessian and index utilities do the arithmetic; nothing is
 * re-derived.
 */

import { FnPotential, classifyCritical, eigenSym, hessian, morseIndex, type CriticalType, type Vec } from '../potential/field';

export interface MorsePoint {
  q: Vec;
  value: number;
  eigs: number[];
  index: number;
  type: CriticalType;
  nondegenerate: boolean;
}

/** Hessian eigenvalues, Morse index and type of f at q (q is assumed critical). */
export function morseData(f: (q: Vec) => number, q: Vec, degenerateTol = 1e-7): MorsePoint {
  const U = new FnPotential(q.length, f);
  const eigs = eigenSym(hessian(U, q, 1e-4));
  const type = classifyCritical(eigs, degenerateTol);
  return { q: q.slice(), value: f(q), eigs, index: morseIndex(eigs, degenerateTol), type, nondegenerate: type !== 'degenerate' };
}

/** Σ (−1)^index over a list of nondegenerate critical points. */
export function eulerCharacteristicFromCriticalPoints(points: readonly MorsePoint[]): number {
  return points.reduce((s, p) => s + (p.index % 2 === 0 ? 1 : -1), 0);
}

/** The height function on T² and its four critical points. */
export const TORUS_HEIGHT = (q: Vec) => Math.cos(q[0]) + Math.cos(q[1]);
export const TORUS_HEIGHT_CRITICAL: Vec[] = [
  [0, 0], // maximum, index 2
  [Math.PI, 0], // saddle
  [0, Math.PI], // saddle
  [Math.PI, Math.PI], // minimum, index 0
];

export function torusHeightMorsePoints(): MorsePoint[] {
  return TORUS_HEIGHT_CRITICAL.map((q) => morseData(TORUS_HEIGHT, q));
}

/**
 * Regular value test for a scalar G at sample points of its level set: n is
 * regular iff DG is surjective (here: nonzero) at every preimage point. The
 * preimage theorem then makes G⁻¹(n) a submanifold of dimension dim M − 1.
 */
export function isRegularValueOnSamples(DG: (q: Vec) => Vec, samples: readonly Vec[], tol = 1e-9): { regular: boolean; worstGradient: number } {
  let worst = Infinity;
  for (const q of samples) {
    const g = DG(q);
    worst = Math.min(worst, Math.hypot(...g));
  }
  return { regular: worst > tol, worstGradient: worst };
}

/** The warm-up: points on f⁻¹(c), f = x² + y², and Df there. */
export function circleLevelSet(c: number, n = 64): { samples: Vec[]; DG: (q: Vec) => Vec } {
  const r = Math.sqrt(c);
  const samples: Vec[] = [];
  for (let k = 0; k < n; k++) samples.push([r * Math.cos((2 * Math.PI * k) / n), r * Math.sin((2 * Math.PI * k) / n)]);
  return { samples, DG: (q) => [2 * q[0], 2 * q[1]] };
}
