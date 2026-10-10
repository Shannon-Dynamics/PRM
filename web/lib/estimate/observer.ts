/**
 * The fixed-gain Luenberger observer — App. J.4, discretized by J.5 — and the
 * algebra of its error.
 *
 * This is the knob the Kalman filter turns. An observer is a copy of the plant
 * tugged toward what the sensor saw:
 *
 *   x̂(k+1 | k)   = F x̂(k | k) + G u(k)
 *   x̂(k+1 | k+1) = x̂(k+1 | k) + K [ y(k+1) − H x̂(k+1 | k) ]
 *
 * Subtract this from the true dynamics and the input cancels, the noise stays,
 * and what is left is an unforced linear system in the error e = x − x̂:
 *
 *   e(k+1) = (I − K H) F e(k) + (I − K H) v(k) − K w(k).
 *
 * Thm. J.5.1 says the noise-free part decays iff every eigenvalue of
 * (I − K H) F lies inside the unit disc. Choosing K is choosing where those
 * eigenvalues sit (Thm. J.3.2, transposed). The Kalman filter makes that choice
 * for you, afresh every step, by minimizing the error covariance instead.
 */

import {
  add,
  eye,
  inv,
  matAdd,
  matMul,
  matSub,
  matVec,
  sub,
  symmetrize,
  transpose,
  type Mat,
  type Vec,
} from '../prob/linalg';
import type { Lti } from './lti';
import { observabilityMatrix } from './lti';

/** One step of the fixed-gain observer. Returns the new estimate and the innovation it used. */
export function observerStep(
  sys: Pick<Lti, 'F' | 'G' | 'H'>,
  xhat: Vec,
  K: Mat,
  u: Vec,
  y: Vec,
): { xhat: Vec; predicted: Vec; nu: Vec } {
  const predicted = add(matVec(sys.F, xhat), matVec(sys.G, u));
  const nu = sub(y, matVec(sys.H, predicted));
  return { xhat: add(predicted, matVec(K, nu)), predicted, nu };
}

/** The error-dynamics matrix (I − K H) F whose eigenvalues decide stability. */
export function errorMatrix(F: Mat, H: Mat, K: Mat): Mat {
  const n = F.length;
  return matMul(matSub(eye(n), matMul(K, H)), F);
}

/**
 * The error covariance a fixed gain K actually produces, in Joseph form:
 *
 *   P⁺ = (I − K H) P⁻ (I − K H)ᵀ + K W Kᵀ.
 *
 * Valid for *any* K, not only the optimal one — the short form P⁻ − K H P⁻ of
 * eq. (8.29) is a consequence of optimality, not a definition, and quoting it
 * for a hand-chosen gain silently under-reports the error. This is the function
 * the Kalman gain minimizes the trace of; see `argminTraceGain` below.
 */
export function observerCovarianceUpdate(P: Mat, H: Mat, W: Mat, K: Mat): Mat {
  const n = P.length;
  const IKH = matSub(eye(n), matMul(K, H));
  return symmetrize(
    matAdd(matMul(matMul(IKH, P), transpose(IKH)), matMul(matMul(K, W), transpose(K))),
  );
}

/** Covariance prediction, eq. (8.27): P⁻ = F P Fᵀ + V. Shared by observer and filter. */
export function covariancePredict(P: Mat, F: Mat, V: Mat): Mat {
  return symmetrize(matAdd(matMul(matMul(F, P), transpose(F)), V));
}

/* -------------------------------------------------------------------------- */
/* Eigenvalues                                                                 */
/* -------------------------------------------------------------------------- */

export interface Complex {
  re: number;
  im: number;
}

export const cabs = (z: Complex): number => Math.hypot(z.re, z.im);

/** Closed-form eigenvalues of a real 2×2 matrix (the observer bench is 2-state). */
export function eig2(M: Mat): [Complex, Complex] {
  const [a, b] = M[0];
  const [c, d] = M[1];
  const tr = a + d;
  const det = a * d - b * c;
  const disc = (tr * tr) / 4 - det;
  if (disc >= 0) {
    const s = Math.sqrt(disc);
    return [
      { re: tr / 2 + s, im: 0 },
      { re: tr / 2 - s, im: 0 },
    ];
  }
  const s = Math.sqrt(-disc);
  return [
    { re: tr / 2, im: s },
    { re: tr / 2, im: -s },
  ];
}

/**
 * Spectral radius of a general real matrix by power iteration on M Mᵀ-free
 * products, i.e. the dominant |λ|. Used where the state is larger than two and
 * only the stability verdict is needed. Converges slowly for near-equal
 * moduli, which is why the 2×2 case has its own closed form.
 */
export function spectralRadius(M: Mat, iters = 400): number {
  const n = M.length;
  let v: Vec = Array.from({ length: n }, (_, i) => 1 / Math.sqrt(n) + 1e-3 * i);
  let rho = 0;
  for (let k = 0; k < iters; k++) {
    const w = matVec(M, v);
    const norm = Math.hypot(...w);
    if (norm < 1e-300) return 0;
    rho = norm;
    v = w.map((x) => x / norm);
  }
  return rho;
}

/** Thm. J.5.1: asymptotically stable iff every |λ| < 1. */
export function isStableGain(F: Mat, H: Mat, K: Mat): boolean {
  const E = errorMatrix(F, H, K);
  if (E.length === 2) return eig2(E).every((z) => cabs(z) < 1);
  return spectralRadius(E) < 1;
}

/* -------------------------------------------------------------------------- */
/* Eigenvalue placement (Thm. J.3.2 transposed)                               */
/* -------------------------------------------------------------------------- */

/** Coefficients of ∏(λ − λᵢ) as a polynomial in λ, lowest degree first. Real roots only. */
function polyFromRoots(roots: number[]): number[] {
  let coeffs = [1];
  for (const r of roots) {
    const next = new Array<number>(coeffs.length + 1).fill(0);
    for (let i = 0; i < coeffs.length; i++) {
      next[i] -= r * coeffs[i];
      next[i + 1] += coeffs[i];
    }
    coeffs = next;
  }
  return coeffs;
}

/** p(M) = Σ cᵢ Mⁱ for a polynomial given lowest degree first. */
function polyOfMatrix(coeffs: number[], M: Mat): Mat {
  const n = M.length;
  let out = eye(n, coeffs[0]);
  let Mk = eye(n);
  for (let i = 1; i < coeffs.length; i++) {
    Mk = matMul(Mk, M);
    out = matAdd(out, Mk.map((r) => r.map((x) => x * coeffs[i])));
  }
  return out;
}

/**
 * Ackermann's formula for the single-output observer: choose K so that the
 * error matrix (I − K H) F has the requested real eigenvalues.
 *
 * (I − K H) F = F − K (H F), so this is pole placement for the pair (F, H F):
 *
 *   K = p(F) · O⁻¹ · e_n,   O = [ H F ; H F² ; … ; H Fⁿ ],
 *
 * with p the characteristic polynomial whose roots are the desired eigenvalues.
 * O is Q·F, so the placement is possible iff (F, H) is observable and F is
 * invertible — which is exactly Choset's footnote 2 made constructive. For the
 * dead-reckoning robot with a velocity sensor O is singular and this throws:
 * no gain can move the position eigenvalue off 1, because no gain can see
 * position.
 */
export function placeObserverGain(F: Mat, H: Mat, desired: number[]): Mat {
  const n = F.length;
  if (H.length !== 1) throw new Error('placeObserverGain: single-output systems only');
  if (desired.length !== n) throw new Error('placeObserverGain: need n desired eigenvalues');
  const O = matMul(observabilityMatrix(F, H), F);
  const pF = polyOfMatrix(polyFromRoots(desired), F);
  let Oinv: Mat;
  try {
    Oinv = inv(O);
  } catch {
    throw new Error('placeObserverGain: (F, H) is not observable — poles cannot be placed');
  }
  const en = new Array<number>(n).fill(0);
  en[n - 1] = 1;
  const k = matVec(matMul(pF, Oinv), en);
  return k.map((x) => [x]);
}

/* -------------------------------------------------------------------------- */
/* The gain that minimizes the posterior variance                              */
/* -------------------------------------------------------------------------- */

/**
 * Brute-force argmin of tr P⁺(K) over a grid of scalar gains, for a scalar
 * state with a scalar sensor. Exists only so a test can show, without appeal
 * to any formula, that the minimizer is P⁻ H / (H P⁻ H + W) — the Kalman gain.
 */
export function argminTraceGain(
  Pminus: number,
  H: number,
  W: number,
  grid = 20001,
  range: [number, number] = [-1, 2],
): { gain: number; variance: number } {
  let best = { gain: NaN, variance: Infinity };
  for (let i = 0; i < grid; i++) {
    const k = range[0] + ((range[1] - range[0]) * i) / (grid - 1);
    const v = observerCovarianceUpdate([[Pminus]], [[H]], [[W]], [[k]])[0][0];
    if (v < best.variance) best = { gain: k, variance: v };
  }
  return best;
}
