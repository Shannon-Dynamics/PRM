/**
 * Linear time-invariant systems in Choset's letters — Ch. 8 §8.2.1, App. J.
 *
 *   x(k+1) = F x(k) + G u(k) + v(k),   v ~ N(0, V)
 *   y(k)   = H x(k) + w(k),            w ~ N(0, W)
 *
 * The letters are Choset's on purpose, and every field names the Thrun symbol
 * the sister book uses, because the two conventions collide in the worst
 * possible place: Choset's **R is the Kalman gain**, Thrun's **R is the process
 * covariance**. A reader of both books is one rename away from a bug, so the
 * translation lives here, once, in the type.
 *
 * Nothing in this file is a filter. It is the vocabulary the observer (J.4),
 * the Kalman step (§8.2.5), and the observability test (Thm. 8.2.2) share.
 */

import { eye, matAdd, matMul, matScale, type Mat } from '../prob/linalg';

/** An LTI system with Gaussian noise, in Choset's notation. */
export interface Lti {
  /** Dynamics, n×n. Thrun's A_t. */
  F: Mat;
  /** Input map, n×m. Thrun's B_t. */
  G: Mat;
  /** Output map, p×n, assumed full row rank. Thrun's C_t. */
  H: Mat;
  /** Process-noise covariance, n×n. Thrun's R_t. */
  V: Mat;
  /** Measurement-noise covariance, p×p. Thrun's Q_t. */
  W: Mat;
}

export const stateDim = (sys: Lti): number => sys.F.length;
export const inputDim = (sys: Lti): number => sys.G[0].length;
export const outputDim = (sys: Lti): number => sys.H.length;

/**
 * App. J.5: the forward-Euler discretization of ẋ = A x + B u, y = C x at
 * sample period T:
 *
 *   F = I + T A,   G = T B,   H = C.
 *
 * Choset's dead-reckoning robot (§8.2.6) is exactly this with A = [[0,1],[0,0]],
 * B = [0, 1/m]ᵀ — which is why its F has T in the corner and its G has T/m.
 */
export function discretize(A: Mat, B: Mat, C: Mat, T: number): Pick<Lti, 'F' | 'G' | 'H'> {
  const n = A.length;
  return {
    F: matAdd(eye(n), matScale(A, T)),
    G: matScale(B, T),
    H: C.map((r) => r.slice()),
  };
}

/**
 * Thm. 8.2.2 / Thm. J.4.1: the observability matrix
 *
 *   Q = [ H ; H F ; H F² ; … ; H F^{n−1} ]      (np × n)
 *
 * Stacking stops at n−1 because, by Cayley–Hamilton, F^n is a combination of
 * lower powers and adds no new rows to the span.
 */
export function observabilityMatrix(F: Mat, H: Mat): Mat {
  const n = F.length;
  const rows: Mat = [];
  let HFk = H.map((r) => r.slice());
  for (let k = 0; k < n; k++) {
    for (const r of HFk) rows.push(r.slice());
    HFk = matMul(HFk, F);
  }
  return rows;
}

/**
 * Numerical rank by Gaussian elimination with full pivoting and a *relative*
 * tolerance: a pivot counts only if it exceeds `tol` times the largest entry
 * of the matrix. The tolerance is a parameter, not a magic number, because the
 * right value depends on how the matrix was produced — an exact 0/1 matrix like
 * Choset's needs none, a linearized Jacobian with metres and radians in the
 * same rows needs a generous one.
 */
export function matrixRank(M: Mat, tol = 1e-9): number {
  const rows = M.length;
  if (rows === 0) return 0;
  const cols = M[0].length;
  const a = M.map((r) => r.slice());
  let scale = 0;
  for (const r of a) for (const x of r) scale = Math.max(scale, Math.abs(x));
  if (scale === 0) return 0;
  const eps = tol * scale;

  let rank = 0;
  const usedCol = new Array<boolean>(cols).fill(false);
  for (let step = 0; step < Math.min(rows, cols); step++) {
    // Full pivoting: the largest remaining entry anywhere below row `rank`.
    let pr = -1;
    let pc = -1;
    let best = eps;
    for (let i = rank; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        if (usedCol[j]) continue;
        if (Math.abs(a[i][j]) > best) {
          best = Math.abs(a[i][j]);
          pr = i;
          pc = j;
        }
      }
    }
    if (pr < 0) break;
    [a[rank], a[pr]] = [a[pr], a[rank]];
    usedCol[pc] = true;
    for (let i = rank + 1; i < rows; i++) {
      const f = a[i][pc] / a[rank][pc];
      if (f === 0) continue;
      for (let j = 0; j < cols; j++) a[i][j] -= f * a[rank][j];
    }
    rank++;
  }
  return rank;
}

/** rank Q for the pair (F, H). */
export function observabilityRank(F: Mat, H: Mat, tol = 1e-9): number {
  return matrixRank(observabilityMatrix(F, H), tol);
}

/** Thm. 8.2.2: observable iff rank Q = n. */
export function isObservable(sys: Pick<Lti, 'F' | 'H'>, tol = 1e-9): boolean {
  return observabilityRank(sys.F, sys.H, tol) === sys.F.length;
}

/**
 * The directions the sensors never see: a basis of null(Q), found by
 * back-substitution on the reduced row-echelon form. Along these directions the
 * Kalman filter's covariance can only grow (§8.2.7), which is what the
 * Observability Probe draws as the ellipse that never shrinks.
 */
export function unobservableSubspace(F: Mat, H: Mat, tol = 1e-9): Mat {
  const Q = observabilityMatrix(F, H);
  const rows = Q.length;
  const n = F.length;
  const a = Q.map((r) => r.slice());
  let scale = 0;
  for (const r of a) for (const x of r) scale = Math.max(scale, Math.abs(x));
  const eps = tol * Math.max(scale, 1e-300);

  // Reduced row echelon form with partial pivoting; track pivot columns.
  const pivotCols: number[] = [];
  let r = 0;
  for (let c = 0; c < n && r < rows; c++) {
    let pr = r;
    for (let i = r + 1; i < rows; i++) if (Math.abs(a[i][c]) > Math.abs(a[pr][c])) pr = i;
    if (Math.abs(a[pr][c]) <= eps) continue;
    [a[r], a[pr]] = [a[pr], a[r]];
    const d = a[r][c];
    for (let j = 0; j < n; j++) a[r][j] /= d;
    for (let i = 0; i < rows; i++) {
      if (i === r) continue;
      const f = a[i][c];
      if (f === 0) continue;
      for (let j = 0; j < n; j++) a[i][j] -= f * a[r][j];
    }
    pivotCols.push(c);
    r++;
  }

  const free = Array.from({ length: n }, (_, j) => j).filter((j) => !pivotCols.includes(j));
  return free.map((fc) => {
    const v = new Array<number>(n).fill(0);
    v[fc] = 1;
    pivotCols.forEach((pc, i) => {
      v[pc] = -a[i][fc];
    });
    const norm = Math.hypot(...v);
    return v.map((x) => x / norm);
  });
}
