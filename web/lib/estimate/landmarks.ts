/**
 * Choset's landmark models — §8.3.1 (range and bearing), §8.3.3 (range only),
 * §8.3.2 (data association) — and an EKF localizer that runs them through the
 * sister book's vendored `Ekf`.
 *
 * Choset's unicycle differs from Thrun's velocity model in one honest way: the
 * input is a displacement [u₁, u₂] = [forward distance, heading change] per
 * sample, and the noise is additive in state space (V), not pushed through a
 * control Jacobian. That is the model eqs. 8.37–8.45 linearize, so it is the
 * one this file implements. The sister's `EkfLocalizer` (its Ch. 11) remains
 * the richer tool; its `predictMeasurement`/`mlAssociate` are reused below for
 * the range–bearing case rather than duplicated.
 */

import { Ekf, numericJacobian } from '../filters/ekf';
import {
  featureInnovation,
  mahalanobis2,
  mlAssociate,
  predictAll,
  type Feature,
} from '../filters/ekf-localization';
import { angleDiff, normalizeAngle, type Pose2 } from '../geom/se2';
import { type Mat, type Vec } from '../prob/linalg';
import type { Landmark } from '../sim/world';
import { observabilityMatrix, matrixRank, unobservableSubspace } from './lti';

export type SensorMode = 'range-bearing' | 'range-only';

/** Choset's input: forward displacement u₁ and heading change u₂ over one sample. */
export type UnicycleInput = [number, number];

/* -------------------------------------------------------------------------- */
/* Motion model, §8.3.1                                                        */
/* -------------------------------------------------------------------------- */

/** f(x, u) = [x_r + u₁ cos θ_r, y_r + u₁ sin θ_r, θ_r + u₂]. */
export function unicycleStep(x: Vec, u: UnicycleInput): Vec {
  return [x[0] + u[0] * Math.cos(x[2]), x[1] + u[0] * Math.sin(x[2]), normalizeAngle(x[2] + u[1])];
}

/**
 * F = ∂f/∂x at x̂(k|k):
 *
 *   [ 1  0  −u₁ sin θ ]
 *   [ 0  1   u₁ cos θ ]
 *   [ 0  0      1     ]
 *
 * The third column is the whole story of heading uncertainty: a small heading
 * error becomes a position error of size u₁ per step, sideways to the motion.
 */
export function unicycleJacobian(x: Vec, u: UnicycleInput): Mat {
  return [
    [1, 0, -u[0] * Math.sin(x[2])],
    [0, 1, u[0] * Math.cos(x[2])],
    [0, 0, 1],
  ];
}

/* -------------------------------------------------------------------------- */
/* Measurement models, §8.3.1 and §8.3.3                                       */
/* -------------------------------------------------------------------------- */

/** h_j(x) = [ρ_j, atan2(y_r − y_ℓj, x_r − x_ℓj) − θ_r] — Choset's sign convention. */
export function rangeBearingH(x: Vec, lm: Landmark): Vec {
  const dx = x[0] - lm.x;
  const dy = x[1] - lm.y;
  return [Math.hypot(dx, dy), normalizeAngle(Math.atan2(dy, dx) - x[2])];
}

/** The range row alone, §8.3.3. */
export function rangeOnlyH(x: Vec, lm: Landmark): Vec {
  return [Math.hypot(x[0] - lm.x, x[1] - lm.y)];
}

/**
 * Jacobian rows of h_j, simplified:
 *
 *   range   [ Δx/ρ,    Δy/ρ,    0 ]
 *   bearing [ −Δy/ρ²,  Δx/ρ²,  −1 ]      Δ = x̂ − x_ℓj.
 *
 * Choset prints the bearing row before simplifying, as the raw derivative of
 * atan2; `rangeBearingJacobianRaw` keeps that form so the check can show the
 * two agree.
 */
export function rangeBearingJacobian(x: Vec, lm: Landmark): Mat {
  const dx = x[0] - lm.x;
  const dy = x[1] - lm.y;
  const q = Math.max(dx * dx + dy * dy, 1e-12);
  const rho = Math.sqrt(q);
  return [
    [dx / rho, dy / rho, 0],
    [-dy / q, dx / q, -1],
  ];
}

/** Choset's unsimplified atan2 derivative (p. 292), kept for the agreement check. */
export function rangeBearingJacobianRaw(x: Vec, lm: Landmark): Mat {
  const dx = x[0] - lm.x;
  const dy = x[1] - lm.y;
  const rho = Math.hypot(dx, dy);
  const t = dy / dx;
  const denom = 1 + t * t;
  return [
    [dx / rho, dy / rho, 0],
    [-dy / (denom * dx * dx), 1 / (denom * dx), -1],
  ];
}

export function rangeOnlyJacobian(x: Vec, lm: Landmark): Mat {
  return [rangeBearingJacobian(x, lm)[0]];
}

/** Stack h_j over the visible landmarks, in map order. */
export function stackedH(x: Vec, landmarks: readonly Landmark[], mode: SensorMode): Vec {
  const out: Vec = [];
  for (const lm of landmarks) out.push(...(mode === 'range-bearing' ? rangeBearingH(x, lm) : rangeOnlyH(x, lm)));
  return out;
}

/** Stack the Jacobian rows over the visible landmarks. */
export function stackedJacobian(x: Vec, landmarks: readonly Landmark[], mode: SensorMode): Mat {
  const rows: Mat = [];
  for (const lm of landmarks) rows.push(...(mode === 'range-bearing' ? rangeBearingJacobian(x, lm) : rangeOnlyJacobian(x, lm)));
  return rows;
}

/** Block-diagonal W for p landmarks: diag(σ_ρ², σ_φ²) or [σ_ρ²] each. */
export function stackedNoise(count: number, mode: SensorMode, sigmaRange: number, sigmaBearing: number): Mat {
  const per = mode === 'range-bearing' ? 2 : 1;
  const n = count * per;
  const W: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  for (let i = 0; i < count; i++) {
    W[i * per][i * per] = sigmaRange * sigmaRange;
    if (per === 2) W[i * per + 1][i * per + 1] = sigmaBearing * sigmaBearing;
  }
  return W;
}

/** ν with bearing components wrapped — the one place a plain subtraction is wrong. */
export function stackedResidual(mode: SensorMode): (z: Vec, zPred: Vec) => Vec {
  return (z, zPred) =>
    z.map((zi, i) => (mode === 'range-bearing' && i % 2 === 1 ? angleDiff(zi, zPred[i]) : zi - zPred[i]));
}

/* -------------------------------------------------------------------------- */
/* Observability of the linearized pair (§8.2.7 applied to §8.3)              */
/* -------------------------------------------------------------------------- */

export interface LinearizedObservability {
  rank: number;
  /** Q = [H; HF; HF²] of the linearized pair at this estimate and input. */
  Q: Mat;
  /** Unit vectors spanning the directions no sighting can correct. Empty when rank 3. */
  blind: Mat;
  observable: boolean;
}

/**
 * Thm. 8.2.2 applied to the pair (∂f/∂x, ∂h/∂x) at the current estimate.
 *
 * Honesty: for a nonlinear system this is a *local* statement about the
 * linearization, valid at this pose, this input, these landmarks. One range-only
 * landmark gives rank 2 — the circle of constant range is invisible. Two
 * non-collinear ones give rank 3 when the robot moves. A single range–bearing
 * landmark gives rank 3 only while u₁ ≠ 0: standing still, two numbers cannot
 * pin three.
 */
export function linearizedObservability(
  x: Vec,
  u: UnicycleInput,
  landmarks: readonly Landmark[],
  mode: SensorMode,
  tol = 1e-6,
): LinearizedObservability {
  if (landmarks.length === 0) {
    return { rank: 0, Q: [], blind: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], observable: false };
  }
  const F = unicycleJacobian(x, u);
  const H = stackedJacobian(x, landmarks, mode);
  const Q = observabilityMatrix(F, H);
  const rank = matrixRank(Q, tol);
  return { rank, Q, blind: rank === 3 ? [] : unobservableSubspace(F, H, tol), observable: rank === 3 };
}

/* -------------------------------------------------------------------------- */
/* Data association, §8.3.2 / §8.4.2                                          */
/* -------------------------------------------------------------------------- */

export interface ChosetAssociation {
  /** a(i): index of the landmark that best explains measurement i, or −1 for "none". */
  landmark: number;
  /** χ²_ij at the argmin. */
  chi2: number;
  /** True when the best χ² exceeds `chi2New` — §8.4.2's "initialize a new landmark". */
  isNew: boolean;
}

/**
 * a(i) = argmin_j ν_ijᵀ S_ij⁻¹ ν_ij with S_ij = H_j P⁻ H_jᵀ + W_i, eq. 8.48.
 *
 * Choset minimizes the Mahalanobis innovation alone. The sister book's
 * `mlAssociate` adds the log det S_j term that turns this into true maximum
 * likelihood when gates differ in size; for range–bearing features that is
 * the better algorithm and is what `ChosetEkf` uses. This function is Choset's
 * rule verbatim, for both sensor modes, plus his new-landmark threshold.
 */
export function associateChi2(
  y: Vec,
  xPred: Vec,
  Pminus: Mat,
  landmarks: readonly Landmark[],
  mode: SensorMode,
  W: Mat,
  chi2New = Infinity,
): ChosetAssociation {
  let best = -1;
  let bestChi2 = Infinity;
  for (let j = 0; j < landmarks.length; j++) {
    const lm = landmarks[j];
    const h = mode === 'range-bearing' ? rangeBearingH(xPred, lm) : rangeOnlyH(xPred, lm);
    const Hj = mode === 'range-bearing' ? rangeBearingJacobian(xPred, lm) : rangeOnlyJacobian(xPred, lm);
    const nu = stackedResidual(mode)(y, h);
    const S = innovationCov(Hj, Pminus, W);
    const chi2 = mode === 'range-bearing' ? mahalanobis2(nu, S) : (nu[0] * nu[0]) / S[0][0];
    if (chi2 < bestChi2) {
      bestChi2 = chi2;
      best = j;
    }
  }
  return { landmark: best, chi2: bestChi2, isNew: bestChi2 > chi2New };
}

function innovationCov(H: Mat, P: Mat, W: Mat): Mat {
  const p = H.length;
  const n = H[0].length;
  const out: Mat = Array.from({ length: p }, () => new Array<number>(p).fill(0));
  for (let i = 0; i < p; i++) {
    for (let j = 0; j < p; j++) {
      let s = W[i][j];
      for (let a = 0; a < n; a++) for (let b = 0; b < n; b++) s += H[i][a] * P[a][b] * H[j][b];
      out[i][j] = s;
    }
  }
  return out;
}

/* -------------------------------------------------------------------------- */
/* The localizer: Choset's models inside the sister's Ekf                      */
/* -------------------------------------------------------------------------- */

export interface ChosetEkfOptions {
  landmarks: readonly Landmark[];
  mode: SensorMode;
  /** Additive process covariance V (3×3), Choset's convention. */
  V: Mat;
  sigmaRange: number;
  sigmaBearing: number;
  /** χ² gate for range–bearing association (2 dof) or range-only (1 dof). */
  gate2?: number;
}

export interface SightingResult {
  /** Which map landmark each measurement was attributed to (−1 = rejected). */
  associations: number[];
  nu: Vec;
  S: Mat;
  d2: number;
}

/**
 * §8.3.1 + §8.3.3 localization, eqs. 8.37–8.45, with the sister's `Ekf` doing
 * the linear algebra. Heading is wrapped through `boxplus`, bearing residuals
 * through `angleDiff`; the 2005 text is silent on both and wrong near ±π.
 */
export class ChosetEkf {
  readonly ekf: Ekf;
  readonly landmarks: readonly Landmark[];
  mode: SensorMode;
  V: Mat;
  sigmaRange: number;
  sigmaBearing: number;
  gate2: number;

  constructor(x0: Vec, P0: Mat, opts: ChosetEkfOptions) {
    this.ekf = new Ekf(x0, P0, {
      boxplus: (x, dx) => [x[0] + dx[0], x[1] + dx[1], normalizeAngle(x[2] + dx[2])],
    });
    this.landmarks = opts.landmarks;
    this.mode = opts.mode;
    this.V = opts.V;
    this.sigmaRange = opts.sigmaRange;
    this.sigmaBearing = opts.sigmaBearing;
    this.gate2 = opts.gate2 ?? (opts.mode === 'range-bearing' ? 9.21 : 6.63);
  }

  get x(): Vec {
    return this.ekf.x;
  }
  get P(): Mat {
    return this.ekf.P;
  }
  pose(): Pose2 {
    return { x: this.ekf.x[0], y: this.ekf.x[1], theta: this.ekf.x[2] };
  }

  /** Eqs. 8.37–8.39. */
  predict(u: UnicycleInput): void {
    this.ekf.predict((x) => unicycleStep(x, u), (x) => unicycleJacobian(x, u), this.V);
  }

  /**
   * Eqs. 8.40–8.45 for a batch of sightings with *known* correspondence
   * (`which[i]` indexes the map). Stacking all visible landmarks into one H is
   * Choset's presentation; it is equivalent to sequential scalar updates for
   * block-diagonal W.
   */
  updateKnown(ys: Vec[], which: number[]): SightingResult {
    const lms = which.map((j) => this.landmarks[j]);
    const z = ys.flat();
    const W = stackedNoise(lms.length, this.mode, this.sigmaRange, this.sigmaBearing);
    const info = this.ekf.update(
      z,
      (x) => stackedH(x, lms, this.mode),
      (x) => stackedJacobian(x, lms, this.mode),
      W,
      { residual: stackedResidual(this.mode) },
    );
    const d2 = chi2Of(info.innovation, info.S);
    return { associations: which, nu: info.innovation, S: info.S, d2 };
  }

  /**
   * §8.3.2: associate each sighting by minimum Mahalanobis innovation, then
   * update with the accepted ones. Range–bearing features go through the
   * sister's gated `mlAssociate`; range-only features through `associateChi2`
   * (the sister's routine assumes a bearing row).
   */
  updateAssociate(ys: Vec[]): SightingResult {
    const accepted: number[] = [];
    const associations: number[] = [];
    const used = new Set<number>();
    const Wrb = [
      [this.sigmaRange ** 2, 0],
      [0, this.sigmaBearing ** 2],
    ];
    for (const y of ys) {
      let j = -1;
      if (this.mode === 'range-bearing') {
        const bel = { mu: this.pose(), Sigma: this.ekf.P };
        const preds = predictAll(bel, this.landmarks as Landmark[], Wrb);
        // Choset's bearing is atan2(y_r − y_ℓ, x_r − x_ℓ) − θ; Thrun's is the
        // reverse difference, i.e. rotated by π. Translate before comparing.
        const feat: Feature = { r: y[0], phi: normalizeAngle(y[1] + Math.PI) };
        const a = mlAssociate(feat, preds, { gate2: this.gate2 });
        if (a.kind === 'match') j = a.index;
      } else {
        const a = associateChi2(y, this.ekf.x, this.ekf.P, this.landmarks, 'range-only', [[this.sigmaRange ** 2]]);
        if (a.chi2 <= this.gate2) j = a.landmark;
      }
      if (j >= 0 && used.has(j)) j = -1;
      associations.push(j);
      if (j >= 0) {
        used.add(j);
        accepted.push(ys.indexOf(y));
      }
    }
    if (accepted.length === 0) return { associations, nu: [], S: [], d2: 0 };
    const res = this.updateKnown(
      accepted.map((i) => ys[i]),
      accepted.map((i) => associations[i]),
    );
    return { ...res, associations };
  }

  /** The residual the sister's `featureInnovation` would compute, for cross-checks. */
  static thrunInnovation(z: Feature, zHat: [number, number]): Vec {
    return featureInnovation(z, zHat);
  }
}

/** νᵀ S⁻¹ ν by Gaussian elimination, any dimension. */
export function chi2Of(nu: Vec, S: Mat): number {
  const n = nu.length;
  if (n === 0) return 0;
  const a = S.map((r, i) => [...r, nu[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]];
    if (Math.abs(a[c][c]) < 1e-15) return Infinity;
    for (let r = c + 1; r < n; r++) {
      const f = a[r][c] / a[c][c];
      for (let j = c; j <= n; j++) a[r][j] -= f * a[c][j];
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let s = a[i][n];
    for (let j = i + 1; j < n; j++) s -= a[i][j] * x[j];
    x[i] = s / a[i][i];
  }
  let d2 = 0;
  for (let i = 0; i < n; i++) d2 += nu[i] * x[i];
  return Math.max(d2, 0);
}

/** Central-difference check of the analytic H against the sister's `numericJacobian`. */
export function jacobianDiscrepancy(x: Vec, lm: Landmark, mode: SensorMode): number {
  const analytic = mode === 'range-bearing' ? rangeBearingJacobian(x, lm) : rangeOnlyJacobian(x, lm);
  const numeric = numericJacobian((v) => (mode === 'range-bearing' ? rangeBearingH(v, lm) : rangeOnlyH(v, lm)), x);
  let worst = 0;
  for (let i = 0; i < analytic.length; i++)
    for (let j = 0; j < 3; j++) worst = Math.max(worst, Math.abs(analytic[i][j] - numeric[i][j]));
  return worst;
}
