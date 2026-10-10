/**
 * Actuator limits in the phase plane (Choset eqs. 11.7–11.11).
 *
 * Each row i of u_min ≤ a s̈ + b ṡ² + c ≤ u_max is one interval of allowed s̈:
 *
 *     α_i = (u_max,i − b_i ṡ² − c_i)/a_i,   β_i = (u_min,i − b_i ṡ² − c_i)/a_i,
 *     U_i = α_i, L_i = β_i  if a_i > 0;   U_i = β_i, L_i = α_i  if a_i < 0.
 *
 * Dividing by a negative a_i flips the inequality — that swap is the whole
 * content of (11.8). Then L = max_i L_i, U = min_i U_i, and the state is
 * admissible when L ≤ U. A zero-inertia row (a_i ≈ 0) contributes no
 * acceleration bound at all; it bounds ṡ directly through (11.11), and that
 * bound is folded into the limit curve by `sdotMax`.
 *
 * The velocity-limit curve v(s) is where the cone collapses, L = U. Because
 * every L_i and U_j is *linear in x = ṡ²* at fixed s, each pair i ≠ j crosses
 * at a closed-form x_ij, and v(s) = min over pairs of √x_ij. The min rule is
 * exact, not heuristic: if any L_i = U_j then L ≥ L_i = U_j ≥ U, so the state is
 * already on or past the boundary. (This leans on Choset's standing assumption
 * that ṡ = 0 is admissible and admissibility is lost at most once as ṡ grows.)
 */

import type { Vec } from '../prob/linalg';
import type { Abc } from './constrained';

export interface TorqueLimits {
  min: Vec;
  max: Vec;
}

/** Symmetric limits |u_i| ≤ max_i. */
export const symmetricLimits = (max: Vec): TorqueLimits => ({ min: max.map((m) => -m), max: max.slice() });

/** Optional joint-rate limits |q̇_i| ≤ v_i (Exercise 5); `null` entries are unlimited. */
export type JointVelocityLimits = (number | null)[];

/** Below this |a_i| a row is treated as zero-inertia. */
export const ZERO_INERTIA_EPS = 1e-9;

export interface AccelBounds {
  L: number;
  U: number;
  /** Which row set L and which set U — for drawing the cone edges. */
  argL: number;
  argU: number;
  /** Rows with a_i ≈ 0 at this s. */
  zeroInertiaRows: number[];
}

/** L(s, ṡ) and U(s, ṡ) — eq. (11.9). */
export function accelBounds(abc: Abc, lim: TorqueLimits, sdot: number): AccelBounds {
  const x = sdot * sdot;
  let L = -Infinity;
  let U = Infinity;
  let argL = -1;
  let argU = -1;
  const zeroInertiaRows: number[] = [];
  for (let i = 0; i < abc.a.length; i++) {
    const ai = abc.a[i];
    if (Math.abs(ai) < ZERO_INERTIA_EPS) {
      zeroInertiaRows.push(i);
      continue;
    }
    const alpha = (lim.max[i] - abc.b[i] * x - abc.c[i]) / ai;
    const beta = (lim.min[i] - abc.b[i] * x - abc.c[i]) / ai;
    const Ui = ai > 0 ? alpha : beta;
    const Li = ai > 0 ? beta : alpha;
    if (Li > L) {
      L = Li;
      argL = i;
    }
    if (Ui < U) {
      U = Ui;
      argU = i;
    }
  }
  return { L, U, argL, argU, zeroInertiaRows };
}

/** Per-row (L_i, U_i) at a state, NaN for zero-inertia rows — the widget's cone fan. */
export function rowBounds(abc: Abc, lim: TorqueLimits, sdot: number): { L: number; U: number }[] {
  const x = sdot * sdot;
  return abc.a.map((ai, i) => {
    if (Math.abs(ai) < ZERO_INERTIA_EPS) return { L: NaN, U: NaN };
    const alpha = (lim.max[i] - abc.b[i] * x - abc.c[i]) / ai;
    const beta = (lim.min[i] - abc.b[i] * x - abc.c[i]) / ai;
    return ai > 0 ? { L: beta, U: alpha } : { L: alpha, U: beta };
  });
}

/**
 * v(s) from the pairwise crossings L_i = U_j (eq. 11.10). Each bound is
 * p + m·x in x = ṡ², so the crossing is x = (p_U − p_L)/(m_L − m_U). Only
 * positive roots count; +∞ when no pair of rows can collapse the cone.
 */
export function velocityLimit(abc: Abc, lim: TorqueLimits): number {
  const n = abc.a.length;
  const rows: { pL: number; mL: number; pU: number; mU: number }[] = [];
  for (let i = 0; i < n; i++) {
    const ai = abc.a[i];
    if (Math.abs(ai) < ZERO_INERTIA_EPS) continue;
    const alphaP = (lim.max[i] - abc.c[i]) / ai;
    const betaP = (lim.min[i] - abc.c[i]) / ai;
    const slope = -abc.b[i] / ai;
    rows.push(ai > 0 ? { pL: betaP, mL: slope, pU: alphaP, mU: slope } : { pL: alphaP, mL: slope, pU: betaP, mU: slope });
  }
  let best = Infinity;
  for (let i = 0; i < rows.length; i++) {
    for (let j = 0; j < rows.length; j++) {
      if (i === j) continue;
      const den = rows[i].mL - rows[j].mU;
      if (Math.abs(den) < 1e-14) continue;
      const x = (rows[j].pU - rows[i].pL) / den;
      if (x > 0 && x < best) best = x;
    }
  }
  return Math.sqrt(best);
}

/**
 * ṡ_max_zip(s): the direct velocity bound from zero-inertia rows (eq. 11.11),
 * u_min ≤ b_i ṡ² + c_i ≤ u_max. +∞ when no row is zero-inertia or b_i = 0 too
 * (then the row constrains nothing — Choset's Problem 11.6, and we assume the
 * robot can hold any configuration statically).
 */
export function zeroInertiaLimit(abc: Abc, lim: TorqueLimits): number {
  let best = Infinity;
  for (let i = 0; i < abc.a.length; i++) {
    if (Math.abs(abc.a[i]) >= ZERO_INERTIA_EPS) continue;
    const bi = abc.b[i];
    if (Math.abs(bi) < 1e-14) continue;
    const x = bi > 0 ? (lim.max[i] - abc.c[i]) / bi : (lim.min[i] - abc.c[i]) / bi;
    if (x >= 0 && x < best) best = x;
    else if (x < 0) best = 0;
  }
  return Math.sqrt(best);
}

/** Joint-rate limits as rows of the velocity limit: ṡ ≤ v_i / |q'_i|. */
export function jointRateLimit(dq: Vec, vmax: JointVelocityLimits | null): number {
  if (!vmax) return Infinity;
  let best = Infinity;
  for (let i = 0; i < dq.length; i++) {
    const v = vmax[i];
    if (v === null || v === undefined) continue;
    if (Math.abs(dq[i]) < 1e-14) continue;
    best = Math.min(best, v / Math.abs(dq[i]));
  }
  return best;
}

export type LimitKind = 'cone' | 'zip' | 'rate' | 'none';

export interface LimitPoint {
  s: number;
  /** ṡ_max(s) = min(v, ṡ_zip, ṡ_rate). */
  sdotMax: number;
  /** Which bound is active. 'cone' is Choset's v(s) (L = U there); 'zip'/'rate' are critical-type (L < U). */
  kind: LimitKind;
  v: number;
  zip: number;
  rate: number;
}

/** ṡ_max(s) with its provenance — the true limit curve of §11.2.1. */
export function limitPoint(s: number, abc: Abc, dq: Vec, lim: TorqueLimits, vmax: JointVelocityLimits | null = null): LimitPoint {
  const v = velocityLimit(abc, lim);
  const zip = zeroInertiaLimit(abc, lim);
  const rate = jointRateLimit(dq, vmax);
  let sdotMax = v;
  let kind: LimitKind = Number.isFinite(v) ? 'cone' : 'none';
  if (zip < sdotMax) {
    sdotMax = zip;
    kind = 'zip';
  }
  if (rate < sdotMax) {
    sdotMax = rate;
    kind = 'rate';
  }
  return { s, sdotMax, kind, v, zip, rate };
}

/** Is (s, ṡ) admissible — does the motion cone exist? */
export function isAdmissible(abc: Abc, lim: TorqueLimits, sdot: number): boolean {
  const { L, U } = accelBounds(abc, lim, sdot);
  return L <= U;
}

/**
 * Brute-force velocity limit by scanning ṡ upward until L > U — the definition,
 * O(steps), for the checks only. `velocityLimit` must agree with it.
 */
export function velocityLimitBruteForce(abc: Abc, lim: TorqueLimits, sdotMax = 50, steps = 50_000): number {
  let prev = 0;
  for (let k = 1; k <= steps; k++) {
    const sd = (k / steps) * sdotMax;
    if (!isAdmissible(abc, lim, sd)) {
      // Bisect between prev and sd.
      let lo = prev;
      let hi = sd;
      for (let it = 0; it < 60; it++) {
        const mid = (lo + hi) / 2;
        if (isAdmissible(abc, lim, mid)) lo = mid;
        else hi = mid;
      }
      return (lo + hi) / 2;
    }
    prev = sd;
  }
  return Infinity;
}
