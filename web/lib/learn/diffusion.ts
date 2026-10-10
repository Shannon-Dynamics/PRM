/**
 * A toy trajectory-diffusion planner in one configuration dimension (D22.5,
 * Derivation 5) — the smallest thing that shows what a Diffuser-style model
 * is and is not.
 *
 * A trajectory is τ ∈ ℝ^T, a configuration per time step, pinned to 0 at both
 * ends. Demonstrations swing around a slab obstacle that blocks |q| < 0.3
 * during the middle steps — half of them above, half below. The forward
 * process adds Gaussian noise on a β-schedule until nothing of the swing is
 * left; a small net learns to predict the noise; ancestral sampling runs it
 * backward from pure noise and gets a *plausible* swing. Guidance tilts each
 * step down the gradient of a smooth cost J. Then — the point of the exercise —
 * Chapter 2's hard collision test runs on every sample, and some fail.
 */

import type { Rng } from '../prob/rng';
import { Mlp, mseLoss, Adam } from './mlp';

/** Waypoints per trajectory. */
export const DIFF_T = 16;
/** Denoising steps. */
export const DIFF_K = 40;
/** The slab: |q| < HALF_WIDTH is in collision for t ∈ [T0, T1]. */
export const SLAB = { t0: 5, t1: 10, halfWidth: 0.3 } as const;
export const DENOISER_SIZES: readonly number[] = [DIFF_T + 1, 64, DIFF_T];

export interface Schedule {
  beta: Float64Array;
  alpha: Float64Array;
  alphaBar: Float64Array;
}

/** Linear β_k from 1e-3 to 0.25: ᾱ_K ≈ 0.007, so τ^K is noise to the eye and to the net. */
export function linearSchedule(K = DIFF_K, betaMin = 1e-3, betaMax = 0.25): Schedule {
  const beta = new Float64Array(K + 1);
  const alpha = new Float64Array(K + 1);
  const alphaBar = new Float64Array(K + 1);
  alphaBar[0] = 1;
  for (let k = 1; k <= K; k++) {
    beta[k] = betaMin + ((betaMax - betaMin) * (k - 1)) / (K - 1);
    alpha[k] = 1 - beta[k];
    alphaBar[k] = alphaBar[k - 1] * alpha[k];
  }
  return { beta, alpha, alphaBar };
}

/** One demonstration: a half-sine swing of amplitude A ∈ [0.5, 0.9] above or below the slab. */
export function demonstration(rng: Rng): Float64Array {
  const sign = rng.next() < 0.5 ? -1 : 1;
  const amp = rng.uniform(0.5, 0.9);
  const tau = new Float64Array(DIFF_T);
  for (let t = 0; t < DIFF_T; t++) {
    tau[t] = sign * amp * Math.sin((Math.PI * t) / (DIFF_T - 1)) + (t === 0 || t === DIFF_T - 1 ? 0 : rng.normal(0, 0.02));
  }
  return tau;
}

/** The hard verifier: a trajectory is valid iff it clears the slab at every blocked step. */
export function trajectoryValid(tau: ArrayLike<number>): boolean {
  for (let t = SLAB.t0; t <= SLAB.t1; t++) if (Math.abs(tau[t]) < SLAB.halfWidth) return false;
  return true;
}

export function validFraction(trajs: readonly ArrayLike<number>[]): number {
  if (trajs.length === 0) return 0;
  return trajs.filter(trajectoryValid).length / trajs.length;
}

/** Which way a (valid or not) trajectory went: sign of its mid-swing. */
export function swingSide(tau: ArrayLike<number>): 1 | -1 {
  const mid = (SLAB.t0 + SLAB.t1) >> 1;
  return tau[mid] >= 0 ? 1 : -1;
}

function denoiserInput(tau: ArrayLike<number>, k: number, K: number): Float64Array {
  const x = new Float64Array(DIFF_T + 1);
  for (let t = 0; t < DIFF_T; t++) x[t] = tau[t];
  x[DIFF_T] = k / K;
  return x;
}

/** τ^k = √ᾱ_k τ⁰ + √(1 − ᾱ_k) ε, and the ε that was used. */
export function noise(tau0: ArrayLike<number>, k: number, sch: Schedule, rng: Rng): { tauK: Float64Array; eps: Float64Array } {
  const a = Math.sqrt(sch.alphaBar[k]);
  const s = Math.sqrt(1 - sch.alphaBar[k]);
  const tauK = new Float64Array(DIFF_T);
  const eps = new Float64Array(DIFF_T);
  for (let t = 0; t < DIFF_T; t++) {
    eps[t] = rng.normal();
    tauK[t] = a * tau0[t] + s * eps[t];
  }
  return { tauK, eps };
}

export interface DenoiserTraining {
  samples: number;
  epochs: number;
  losses: number[];
}

/** Denoising-score matching: minimize E‖ε − ε_θ(τ^k, k)‖² over demonstrations, steps and noise. */
export function trainDenoiser(net: Mlp, rng: Rng, samples: number, epochs: number, lr = 2e-3, sch = linearSchedule()): DenoiserTraining {
  const demos: Float64Array[] = [];
  for (let i = 0; i < samples; i++) demos.push(demonstration(rng));
  const adam = new Adam(net, lr);
  const losses: number[] = [];
  const batch = 32;
  for (let e = 0; e < epochs; e++) {
    let total = 0;
    for (let b = 0; b < samples; b += batch) {
      const end = Math.min(samples, b + batch);
      for (let i = b; i < end; i++) {
        const k = 1 + Math.floor(rng.next() * DIFF_K);
        const { tauK, eps } = noise(demos[i], k, sch, rng);
        const y = net.forward(denoiserInput(tauK, k, DIFF_K), true);
        const lg = mseLoss(y, eps);
        total += lg.loss;
        net.backward(lg.dy);
      }
      adam.step(end - b);
    }
    losses.push(total / samples);
    adam.lr *= 0.93;
  }
  return { samples, epochs, losses };
}

/* -------------------------------------------------------------------------- */
/* Guidance: a smooth surrogate cost J and its gradient                         */
/* -------------------------------------------------------------------------- */

export interface GuidanceCost {
  /** Weight on Σ (τ_{t+1} − τ_t)² — Chapter 19's smoothness term. */
  smooth: number;
  /** Weight on Σ_{t in slab} max(0, m − |τ_t|)² with margin m — a *soft* stand-in for the hard constraint. */
  obstacle: number;
  margin: number;
}

export const DEFAULT_GUIDANCE: GuidanceCost = { smooth: 0.5, obstacle: 4.0, margin: SLAB.halfWidth + 0.12 };

export function guidanceGradient(tau: ArrayLike<number>, g: GuidanceCost): Float64Array {
  const grad = new Float64Array(DIFF_T);
  for (let t = 0; t + 1 < DIFF_T; t++) {
    const d = tau[t + 1] - tau[t];
    grad[t] -= 2 * g.smooth * d;
    grad[t + 1] += 2 * g.smooth * d;
  }
  for (let t = SLAB.t0; t <= SLAB.t1; t++) {
    const a = Math.abs(tau[t]);
    if (a < g.margin) {
      // d/dτ (m − |τ|)² = −2 (m − |τ|) sign(τ); at τ = 0 push upward (a convention, not a derivation).
      const s = tau[t] >= 0 ? 1 : -1;
      grad[t] += -2 * g.obstacle * (g.margin - a) * s;
    }
  }
  return grad;
}

export interface DenoiseRun {
  /** steps[0] is τ^K (pure noise), steps[K] the final samples; each holds `count` trajectories. */
  steps: Float64Array[][];
  valid: boolean[];
  validFraction: number;
  /** How many of the final samples swung above / below. */
  above: number;
  below: number;
}

/**
 * Ancestral sampling, K steps from Gaussian noise, endpoints pinned (Diffuser's
 * inpainting of start and goal), optionally guided by −w_k ∇J with w_k ∝ β_k.
 */
export function denoise(net: Mlp, rng: Rng, count: number, opts: { guidance?: GuidanceCost | null; guidanceScale?: number; schedule?: Schedule } = {}): DenoiseRun {
  const sch = opts.schedule ?? linearSchedule();
  const K = DIFF_K;
  const guide = opts.guidance ?? null;
  const gScale = opts.guidanceScale ?? 1.5;
  let cur: Float64Array[] = [];
  for (let i = 0; i < count; i++) {
    const t = new Float64Array(DIFF_T);
    for (let s = 0; s < DIFF_T; s++) t[s] = rng.normal();
    t[0] = 0;
    t[DIFF_T - 1] = 0;
    cur.push(t);
  }
  const steps: Float64Array[][] = [cur.map((t) => Float64Array.from(t))];
  for (let k = K; k >= 1; k--) {
    const next: Float64Array[] = [];
    const a = sch.alpha[k];
    const ab = sch.alphaBar[k];
    const b = sch.beta[k];
    for (const tau of cur) {
      const eps = net.forward(denoiserInput(tau, k, K));
      const out = new Float64Array(DIFF_T);
      const c = 1 / Math.sqrt(a);
      const d = b / Math.sqrt(1 - ab);
      const grad = guide ? guidanceGradient(tau, guide) : null;
      for (let t = 0; t < DIFF_T; t++) {
        let mean = c * (tau[t] - d * eps[t]);
        if (grad) mean -= gScale * b * grad[t];
        out[t] = k > 1 ? mean + Math.sqrt(b) * rng.normal() : mean;
      }
      out[0] = 0;
      out[DIFF_T - 1] = 0;
      next.push(out);
    }
    cur = next;
    steps.push(cur.map((t) => Float64Array.from(t)));
  }
  const valid = cur.map(trajectoryValid);
  const above = cur.filter((t) => swingSide(t) === 1).length;
  return { steps, valid, validFraction: valid.filter(Boolean).length / Math.max(1, count), above, below: count - above };
}
