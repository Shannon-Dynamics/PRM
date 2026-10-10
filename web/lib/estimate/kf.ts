/**
 * Choset's Kalman step (§8.2.5, eqs. 8.26–8.32) as a thin adapter over the
 * sister book's `Kf` — nothing here re-derives or re-implements the filter.
 *
 * What this file adds is the vocabulary Chapter 15 teaches with:
 *
 *   - the step returns ν, S and R (Choset's gain) because data association
 *     (§8.3.2) and the Observer Bench need them;
 *   - the §8.2.6 dead-reckoning constants, so the worked example and its check
 *     live in one place;
 *   - the Riccati recursion to its fixed point — the bridge from "knob" to
 *     "schedule": the steady-state gain is the one fixed-gain observer the
 *     Kalman filter eventually becomes.
 *
 * Symbol hazard, stated once: Choset's R is the gain and V, W are covariances;
 * the sister (Thrun) uses K for the gain and R, Q for covariances. `Kf` speaks
 * Thrun; this file translates at the boundary and nowhere else.
 */

import { Kf, type Gaussian } from '../filters/kf';
import { inv, matMul, transpose, type Mat, type Vec } from '../prob/linalg';
import type { Lti } from './lti';
import { covariancePredict, observerCovarianceUpdate } from './observer';

export type { Gaussian };

/** Everything one Choset step produces, prediction and update kept apart. */
export interface KalmanStepResult {
  /** x̂(k+1 | k), P(k+1 | k) — eqs. 8.26–8.27. */
  predicted: Gaussian;
  /** x̂(k+1 | k+1), P(k+1 | k+1) — eqs. 8.28–8.29. */
  posterior: Gaussian;
  /** ν = y − H x̂(k+1 | k), eq. 8.30. */
  nu: Vec;
  /** S = H P⁻ Hᵀ + W, eq. 8.31. */
  S: Mat;
  /** R = P⁻ Hᵀ S⁻¹, eq. 8.32 — Choset's letter for the gain. */
  R: Mat;
}

/**
 * One predict-then-update cycle through the vendored `Kf`.
 *
 * The sister's `updateWith` uses the Joseph form for P⁺; for the optimal gain
 * it equals Choset's P⁻ − R H P⁻ exactly, so the §8.2.6 numbers come out the
 * same to round-off. The check file asserts that they do.
 */
export function kalmanStep(sys: Lti, bel: Gaussian, u: Vec, y: Vec): KalmanStepResult {
  const kf = new Kf(bel.x, bel.P);
  kf.predictWith(sys.F, sys.V, sys.G, u);
  const predicted = kf.belief();
  const info = kf.updateWith(y, sys.H, sys.W);
  return {
    predicted,
    posterior: kf.belief(),
    nu: info.innovation,
    S: info.S,
    R: info.K,
  };
}

/** Prediction only — dead reckoning between sightings, eqs. 8.26–8.27. */
export function kalmanPredict(sys: Lti, bel: Gaussian, u: Vec): Gaussian {
  const kf = new Kf(bel.x, bel.P);
  kf.predictWith(sys.F, sys.V, sys.G, u);
  return kf.belief();
}

/** Choset's gain for a given predicted covariance: R = P⁻ Hᵀ (H P⁻ Hᵀ + W)⁻¹. */
export function kalmanGain(Pminus: Mat, H: Mat, W: Mat): Mat {
  const Ht = transpose(H);
  const S = matMul(matMul(H, Pminus), Ht).map((r, i) => r.map((x, j) => x + W[i][j]));
  return matMul(matMul(Pminus, Ht), inv(S));
}

/* -------------------------------------------------------------------------- */
/* §8.2.6 — the dead-reckoning robot                                           */
/* -------------------------------------------------------------------------- */

export type RailSensor = 'velocity' | 'position';

export interface DeadReckoningOptions {
  /** Sample period T (s). Choset: 0.5. */
  T?: number;
  /** Mass m (kg). Choset: 1. */
  m?: number;
  /** Which single sensor the rail robot carries. Choset: velocity. */
  sensor?: RailSensor;
  /** Measurement variance W. Choset: 0.5. */
  W?: number;
  /** Scale on Choset's V, for the widget's disclosure slider. */
  processScale?: number;
}

/**
 * Choset §8.2.6: a robot on a rail with state x = [x_r, v_r]ᵀ, a force input,
 * Newton's law discretized at T, and one sensor.
 *
 *   F = [1 T; 0 1],  G = [0; T/m],  V = [0.2 0.05; 0.05 0.1],  W = 0.5,
 *   H = [0 1] (velocity sensor)  or  [1 0] (position sensor).
 *
 * Returns the system together with Choset's initial belief
 * x̂(k|k) = [2, 4]ᵀ, P(k|k) = diag(1, 2).
 */
export function deadReckoning(opts: DeadReckoningOptions = {}): { sys: Lti; bel: Gaussian } {
  const { T = 0.5, m = 1, sensor = 'velocity', W = 0.5, processScale = 1 } = opts;
  const sys: Lti = {
    F: [
      [1, T],
      [0, 1],
    ],
    G: [[0], [T / m]],
    H: sensor === 'velocity' ? [[0, 1]] : [[1, 0]],
    V: [
      [0.2 * processScale, 0.05 * processScale],
      [0.05 * processScale, 0.1 * processScale],
    ],
    W: [[W]],
  };
  return {
    sys,
    bel: {
      x: [2, 4],
      P: [
        [1, 0],
        [0, 2],
      ],
    },
  };
}

/* -------------------------------------------------------------------------- */
/* From knob to schedule: the Riccati recursion                                */
/* -------------------------------------------------------------------------- */

export interface RiccatiResult {
  /** Fixed point of the predicted covariance P(k+1 | k). */
  Pminus: Mat;
  /** The matching posterior covariance P(k+1 | k+1). */
  Pplus: Mat;
  /** The steady-state gain R∞ = P⁻ Hᵀ (H P⁻ Hᵀ + W)⁻¹. */
  R: Mat;
  /** Iterations used, and whether the recursion settled within `tol`. */
  iterations: number;
  converged: boolean;
}

/**
 * Iterate predict → update on the covariance alone until P(k+1 | k) stops
 * moving. The gain at the fixed point is the one constant observer gain that
 * the Kalman filter converges to; feeding it to `observerStep` reproduces the
 * filter's mean asymptotically (Exercise 5).
 *
 * For an unobservable pair the recursion never settles — the unobserved
 * variance grows without bound — and `converged` reports false. That is the
 * §8.2.7 lesson in a boolean.
 */
export function riccatiSteadyState(sys: Lti, P0: Mat, maxIters = 2000, tol = 1e-12): RiccatiResult {
  let P = P0.map((r) => r.slice());
  let Pminus = P;
  let R = kalmanGain(covariancePredict(P, sys.F, sys.V), sys.H, sys.W);
  let converged = false;
  let k = 0;
  for (; k < maxIters; k++) {
    const next = covariancePredict(P, sys.F, sys.V);
    const Rk = kalmanGain(next, sys.H, sys.W);
    // Joseph form here too, so the fixed point is the same one a fixed-gain
    // observer with R∞ would settle on.
    const n = P.length;
    const Pplus = observerCovarianceUpdate(next, sys.H, sys.W, Rk);
    let delta = 0;
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) delta = Math.max(delta, Math.abs(next[i][j] - Pminus[i][j]));
    Pminus = next;
    R = Rk;
    P = Pplus;
    if (delta < tol) {
      converged = true;
      k++;
      break;
    }
  }
  return { Pminus, Pplus: P, R, iterations: k, converged };
}

/**
 * The gain schedule R(1), R(2), …, R(n) from an initial belief: what the
 * Observer Bench animates when the reader hands the knob to the filter. Depends
 * on the covariances only, never on the measurements — which is why it can be
 * computed before a single sensor reading arrives.
 */
export function gainSchedule(sys: Lti, P0: Mat, steps: number): Mat[] {
  const out: Mat[] = [];
  let P = P0;
  for (let k = 0; k < steps; k++) {
    const Pminus = covariancePredict(P, sys.F, sys.V);
    const R = kalmanGain(Pminus, sys.H, sys.W);
    out.push(R);
    P = observerCovarianceUpdate(Pminus, sys.H, sys.W, R);
  }
  return out;
}
