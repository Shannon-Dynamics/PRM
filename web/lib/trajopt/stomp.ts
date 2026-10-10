/**
 * STOMP — stochastic trajectory optimization for motion planning
 * (Kalakrishnan, Chitta, Theodorou, Pastor & Schaal 2011), sharing CHOMP's
 * problem and cost.
 *
 * Instead of a gradient, K seeded rollouts: perturbations δξ_k ~ N(0, A⁻¹) —
 * the smoothness metric again, now as a covariance, so the noise is smooth —
 * each evaluated by the full cost F(ξ + δξ_k). The update is the exponentiated
 * cost-weighted average of the perturbations,
 *
 *     w_k ∝ exp(−h (S_k − S_min)/(S_max − S_min)),      δξ = Σ_k w_k δξ_k,
 *
 * smoothed once more through A⁻¹ (STOMP's M matrix, columns scaled so the
 * largest entry is 1/N) and applied. No gradient is ever formed, which is the
 * point: any cost that can be *evaluated* can be optimized, including the
 * non-differentiable ones. The price is variance, which the seed makes
 * reproducible and the chapter's check keeps honest.
 */

import { cholesky, inv, matVec, type Vec } from '../prob/linalg';
import { Rng } from '../prob/rng';
import { chompEvaluate, smoothnessMatrix, type ChompProblem, type ChompStats } from './chomp';

export interface StompOptions {
  /** Rollouts per iteration. */
  K?: number;
  /** Noise scale multiplying the N(0, A⁻¹) samples. */
  noise?: number;
  /** Temperature h in the exponentiated weights. */
  h?: number;
}

export interface StompState {
  xi: Vec[];
  rng: Rng;
  /** L with L Lᵀ = A⁻¹, so L z ~ N(0, A⁻¹) for z ~ N(0, I). */
  L: number[][];
  /** The STOMP smoothing matrix M (A⁻¹ with columns scaled so max = 1/m). */
  M: number[][];
  history: ChompStats[];
  /** The rollouts of the last iteration, for drawing. */
  lastRollouts: Vec[][];
}

export function stompInit(p: ChompProblem, xi0: readonly Vec[], seed: number): StompState {
  const m = xi0.length;
  const Ainv = inv(smoothnessMatrix(m));
  const L = cholesky(Ainv);
  const M = Ainv.map((row) => row.slice());
  for (let j = 0; j < m; j++) {
    let mx = 0;
    for (let i = 0; i < m; i++) mx = Math.max(mx, Math.abs(Ainv[i][j]));
    for (let i = 0; i < m; i++) M[i][j] = Ainv[i][j] / (mx * m);
  }
  return { xi: xi0.map((q) => q.slice()), rng: new Rng(seed), L, M, history: [chompEvaluate(p, xi0).stats], lastRollouts: [] };
}

/** One STOMP iteration, in place. Returns the statistics after the update. */
export function stompStep(p: ChompProblem, st: StompState, opts: StompOptions = {}): ChompStats {
  const K = opts.K ?? 20;
  const noise = opts.noise ?? 0.3;
  const h = opts.h ?? 10;
  const m = st.xi.length;
  const n = p.q0.length;
  const rollouts: Vec[][] = [];
  const deltas: Vec[][] = [];
  const costs: number[] = [];
  for (let k = 0; k < K; k++) {
    // One smooth noise sample per coordinate: δ_i = noise · L z_i.
    const delta: Vec[] = Array.from({ length: m }, () => new Array<number>(n).fill(0));
    for (let i = 0; i < n; i++) {
      const z = Array.from({ length: m }, () => st.rng.normal());
      const s = matVec(st.L, z);
      for (let r = 0; r < m; r++) delta[r][i] = noise * s[r];
    }
    const trial = st.xi.map((q, r) => q.map((v, i) => v + delta[r][i]));
    rollouts.push(trial);
    deltas.push(delta);
    costs.push(chompEvaluate(p, trial).stats.f);
  }
  const sMin = Math.min(...costs);
  const sMax = Math.max(...costs);
  const range = sMax - sMin > 1e-12 ? sMax - sMin : 1;
  const w = costs.map((c) => Math.exp((-h * (c - sMin)) / range));
  const wSum = w.reduce((a, b) => a + b, 0);
  // Weighted average of the perturbations, then smoothed by M.
  const avg: Vec[] = Array.from({ length: m }, () => new Array<number>(n).fill(0));
  for (let k = 0; k < K; k++) for (let r = 0; r < m; r++) for (let i = 0; i < n; i++) avg[r][i] += (w[k] / wSum) * deltas[k][r][i];
  const update: Vec[] = Array.from({ length: m }, () => new Array<number>(n).fill(0));
  for (let i = 0; i < n; i++) {
    const col = avg.map((row) => row[i]);
    const sm = matVec(st.M, col);
    for (let r = 0; r < m; r++) update[r][i] = sm[r] * m;
  }
  const trialXi = st.xi.map((q, r) => q.map((v, i) => v + update[r][i]));
  // Accept only if the cost does not rise — the paper's practical safeguard.
  const before = st.history[st.history.length - 1].f;
  const after = chompEvaluate(p, trialXi).stats;
  if (after.f <= before) st.xi = trialXi;
  const stats = chompEvaluate(p, st.xi).stats;
  st.history.push(stats);
  st.lastRollouts = rollouts;
  return stats;
}

/** Run `iters` STOMP iterations from ξ₀ with a seed. */
export function stomp(p: ChompProblem, xi0: readonly Vec[], iters: number, seed: number, opts: StompOptions = {}): StompState {
  const st = stompInit(p, xi0, seed);
  for (let it = 0; it < iters; it++) stompStep(p, st, opts);
  return st;
}
