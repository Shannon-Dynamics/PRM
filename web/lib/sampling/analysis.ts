/**
 * Choset §7.4 as arithmetic: the numbers behind the Foundation section.
 *
 * Theorem 7.4.1 tiles a known path of length L and clearance ρ with
 * m = ⌈2L/ρ⌉ balls of radius ρ/2 and bounds the probability that some ball
 * receives no sample. Theorem 7.4.2 counts the samples an (ε, α, β)-expansive
 * space needs for every induced subgraph to be connected with probability
 * 1 − δ. Both are exponential in shape and hopeless in constants, and the
 * prose says so with the numbers these functions print.
 */

/** ζ_d, the volume of the unit ball in ℝᵈ: π^{d/2} / Γ(d/2 + 1). */
export function unitBallVolume(d: number): number {
  return Math.pow(Math.PI, d / 2) / gamma(d / 2 + 1);
}

/** Γ for half-integers and integers (all this book needs); Lanczos otherwise. */
export function gamma(x: number): number {
  if (Number.isInteger(x) && x > 0) {
    let f = 1;
    for (let i = 2; i < x; i++) f *= i;
    return f;
  }
  if (Number.isInteger(x * 2) && x > 0) {
    // Γ(n + 1/2) = (2n)! √π / (4ⁿ n!)
    const n = Math.round(x - 0.5);
    let num = 1;
    for (let i = 2; i <= 2 * n; i++) num *= i;
    let den = Math.pow(4, n);
    for (let i = 2; i <= n; i++) den *= i;
    return (num * Math.sqrt(Math.PI)) / den;
  }
  // Lanczos approximation, g = 7.
  const g = 7;
  const c = [0.99999999999980993, 676.5203681218851, -1259.1392167224028, 771.32342877765313, -176.61502916214059, 12.507343278686905, -0.13857109526572012, 9.9843695780195716e-6, 1.5056327351493116e-7];
  if (x < 0.5) return Math.PI / (Math.sin(Math.PI * x) * gamma(1 - x));
  x -= 1;
  let a = c[0];
  const t = x + g + 0.5;
  for (let i = 1; i < g + 2; i++) a += c[i] / (x + i);
  return Math.sqrt(2 * Math.PI) * Math.pow(t, x + 0.5) * Math.exp(-t) * a;
}

/** σ = μ(B₁) / (2ᵈ μ(𝒬_free)) — Theorem 7.4.1's constant. */
export function sigma(d: number, muFree: number): number {
  return unitBallVolume(d) / (Math.pow(2, d) * muFree);
}

/** m = ⌈2L / ρ⌉ balls tile the path. */
export function tilingBalls(L: number, rho: number): number {
  return Math.ceil((2 * L) / rho);
}

/** Pr[FAILURE] ≤ ⌈2L/ρ⌉ e^{−σρᵈn}. */
export function prmFailureBound(d: number, muFree: number, L: number, rho: number, n: number): number {
  return tilingBalls(L, rho) * Math.exp(-sigma(d, muFree) * Math.pow(rho, d) * n);
}

/** The exact (pre-exponential) version from the proof: m (1 − σρᵈ)ⁿ. */
export function prmFailureBoundExact(d: number, muFree: number, L: number, rho: number, n: number): number {
  return tilingBalls(L, rho) * Math.pow(1 - sigma(d, muFree) * Math.pow(rho, d), n);
}

/** Smallest n with the exponential bound ≤ pFail. */
export function samplesForFailure(d: number, muFree: number, L: number, rho: number, pFail: number): number {
  const m = tilingBalls(L, rho);
  const rate = sigma(d, muFree) * Math.pow(rho, d);
  return Math.max(0, Math.ceil(Math.log(m / pFail) / rate));
}

/** Theorem 7.4.2: n = ⌈8 ln(8/(εαδ)) / (εα) + 3/β⌉; the theorem draws 2n + 2 samples. */
export function expansiveSampleCount(eps: number, alpha: number, beta: number, delta: number): { n: number; samples: number } {
  const n = Math.ceil((8 * Math.log(8 / (eps * alpha * delta))) / (eps * alpha) + 3 / beta);
  return { n, samples: 2 * n + 2 };
}

/** Figure 7.19's estimate for two W-squares joined by a corridor of width w: ε ≈ α ≈ β ≈ w/W. */
export function corridorExpansiveness(w: number, W: number): number {
  return Math.min(1, Math.max(0, w / W));
}

/**
 * Lemma 7.4.6's per-point failure term s e^{−(n−t−1)/s} with s = 1/(αε), and
 * Lemma 7.4.7's linking-sequence length t = ⌈β⁻¹ ln 4⌉ — the two pieces the
 * Theorem 7.4.2 sketch multiplies together.
 */
export function linkingSequence(eps: number, alpha: number, beta: number, n: number): { s: number; t: number; pNoSequence: number } {
  const s = 1 / (alpha * eps);
  const t = Math.ceil(Math.log(4) / beta);
  return { s, t, pNoSequence: Math.min(1, s * Math.exp(-(n - t - 1) / s)) };
}

/* -------------------------------------------------------------------------- */
/* Expansiveness lookup by sampling                                            */
/* -------------------------------------------------------------------------- */

import type { Rng } from '../prob/rng';
import type { Chart } from '../manifold/manifold';
import type { FreeSpace } from './free-space';
import type { Steer } from './steer';
import { drawFree, Uniform } from './sampler';

export interface GoodnessEstimate {
  /** min over sampled x of the estimated μ(reach(x))/μ(𝒬_free): the ε of Def. 7.4.2, estimated. */
  epsilon: number;
  mean: number;
  /** Per-point visibility fractions, for a histogram. */
  fractions: number[];
}

/**
 * Estimate ε-goodness (Definition 7.4.2) by Monte Carlo: for each of `points`
 * free configurations x, the fraction of `probes` free configurations that Δ
 * connects to x. Exact for no world and useful for every one — it is the
 * "expansiveness lookup" behind w11.2's readout.
 */
export function estimateEpsilonGood<P>(
  space: Chart<P>,
  free: FreeSpace<P>,
  steer: Steer<P>,
  rng: Rng,
  points = 24,
  probes = 60,
): GoodnessEstimate {
  const u = new Uniform<P>();
  const xs: P[] = [];
  const ys: P[] = [];
  for (let i = 0; i < points; i++) {
    const q = drawFree(u, space, free, rng);
    if (q) xs.push(q);
  }
  for (let i = 0; i < probes; i++) {
    const q = drawFree(u, space, free, rng);
    if (q) ys.push(q);
  }
  const fractions = xs.map((x) => {
    let seen = 0;
    for (const y of ys) if (steer.steer(space, free, x, y)) seen++;
    return ys.length ? seen / ys.length : 0;
  });
  const epsilon = fractions.length ? Math.min(...fractions) : 0;
  const mean = fractions.length ? fractions.reduce((a, b) => a + b, 0) / fractions.length : 0;
  return { epsilon, mean, fractions };
}

/* -------------------------------------------------------------------------- */
/* The §3 micro-example as data                                                */
/* -------------------------------------------------------------------------- */

/** s₁ … s₆ of the six-sample roadmap. */
export const SIX_SAMPLES: number[][] = [
  [1, 2],
  [3, 5],
  [2, 8],
  [7, 2],
  [8, 6],
  [6, 9],
];

export const SIX_SAMPLE_CONSTANTS = { d: 2, muFree: 0.9, L: 1.2, rho: 0.1 } as const;
