/**
 * Pfaffian constraints and constrained dynamics (Choset §10.3).
 *
 *     A(q) q̇ = 0                                              (10.15)
 *     M q̈ + C q̇ + g = u + Aᵀ λ                                (10.16)
 *     A q̈ + Ȧ q̇ = 0                                           (10.17)
 *
 * Solved as one (n + k) × (n + k) KKT system for (q̈, λ): the constraint
 * forces λ_j a_j(q) are what the constraints must apply to be maintained.
 * The projection P = I − M⁻¹Aᵀ(AM⁻¹Aᵀ)⁻¹A (10.19) and its dual
 * P_u = I − Aᵀ(AM⁻¹Aᵀ)⁻¹AM⁻¹ = Pᵀ (10.18) are built explicitly so the
 * knife-edge example (10.3.1) — Rusty's axle — can be shown to have rank 2.
 */

import { inv, matMul, matVec, solve, type Mat, type Vec } from '../prob/linalg';
import type { Dynamics } from './model';

export interface Pfaffian {
  /** k × n constraint matrix A(q). */
  A(q: Vec): Mat;
  /** k × n time derivative Ȧ(q, q̇) = Σ_k (∂A/∂q_k) q̇_k. */
  Adot(q: Vec, qd: Vec): Mat;
}

/** Ȧ by central differences, for constraints without an analytic derivative. */
export function numericAdot(A: (q: Vec) => Mat, q: Vec, qd: Vec, h = 1e-6): Mat {
  const base = A(q);
  const out: Mat = base.map((row) => row.map(() => 0));
  for (let k = 0; k < q.length; k++) {
    if (qd[k] === 0) continue;
    const qp = q.slice();
    const qm = q.slice();
    qp[k] += h;
    qm[k] -= h;
    const Ap = A(qp);
    const Am = A(qm);
    for (let r = 0; r < base.length; r++) for (let c = 0; c < q.length; c++) out[r][c] += ((Ap[r][c] - Am[r][c]) / (2 * h)) * qd[k];
  }
  return out;
}

/**
 * (q̈, λ) from the KKT system
 *
 *     [ M   −Aᵀ ] [ q̈ ]   [ u − C q̇ − g − b ]
 *     [ A    0  ] [ λ ] = [ −Ȧ q̇            ]
 */
export function constrainedForward(
  dyn: Dynamics,
  c: Pfaffian,
  q: Vec,
  qd: Vec,
  u: Vec,
): { qdd: Vec; lambda: Vec } {
  const n = dyn.n;
  const M = dyn.inertia(q);
  const A = c.A(q);
  const Ad = c.Adot(q, qd);
  const k = A.length;
  const cv = dyn.velocityProduct(q, qd);
  const g = dyn.gravity(q);
  const b = dyn.dissipation(q, qd);
  const K: Mat = Array.from({ length: n + k }, () => new Array<number>(n + k).fill(0));
  const rhs = new Array<number>(n + k).fill(0);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) K[i][j] = M[i][j];
    for (let j = 0; j < k; j++) K[i][n + j] = -A[j][i];
    rhs[i] = u[i] - cv[i] - g[i] - b[i];
  }
  const Adqd = matVec(Ad, qd);
  for (let j = 0; j < k; j++) {
    for (let i = 0; i < n; i++) K[n + j][i] = A[j][i];
    rhs[n + j] = -Adqd[j];
  }
  const x = solve(K, rhs);
  return { qdd: x.slice(0, n), lambda: x.slice(n) };
}

/** P = I − M⁻¹Aᵀ(AM⁻¹Aᵀ)⁻¹A (10.19): projects motions onto those satisfying A q̇ = 0. */
export function projectionP(M: Mat, A: Mat): Mat {
  const n = M.length;
  const Minv = inv(M);
  const At = A[0].map((_, c) => A.map((row) => row[c]));
  const AMinv = matMul(A, Minv);
  const S = inv(matMul(AMinv, At)); // (A M⁻¹ Aᵀ)⁻¹, k × k
  const X = matMul(matMul(matMul(Minv, At), S), A);
  return Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0) - X[i][j]));
}

/** P_u = I − Aᵀ(AM⁻¹Aᵀ)⁻¹AM⁻¹ = Pᵀ (10.18): projects generalized forces onto those that do work. */
export function projectionPu(M: Mat, A: Mat): Mat {
  const P = projectionP(M, A);
  return P[0].map((_, c) => P.map((row) => row[c]));
}

/** Numerical rank by Gaussian elimination with a tolerance — P should have rank n − k. */
export function matrixRank(Min: Mat, tol = 1e-9): number {
  const M = Min.map((r) => r.slice());
  const rows = M.length;
  const cols = M[0].length;
  let rank = 0;
  let r = 0;
  for (let c = 0; c < cols && r < rows; c++) {
    let piv = r;
    for (let i = r + 1; i < rows; i++) if (Math.abs(M[i][c]) > Math.abs(M[piv][c])) piv = i;
    if (Math.abs(M[piv][c]) < tol) continue;
    [M[r], M[piv]] = [M[piv], M[r]];
    for (let i = 0; i < rows; i++) {
      if (i === r) continue;
      const f = M[i][c] / M[r][c];
      for (let j = c; j < cols; j++) M[i][j] -= f * M[r][j];
    }
    r++;
    rank++;
  }
  return rank;
}

/* -------------------------------------------------------------------------- */
/* Example 10.3.1 — the knife-edge (Rusty's axle)                              */
/* -------------------------------------------------------------------------- */

/**
 * A planar body of mass m and inertia I with q = (q₁, q₂, q₃) = (x, y, heading)
 * that may slide along its heading and spin, but not sideways:
 * q̇₁ sin q₃ − q̇₂ cos q₃ = 0. Choset's Example 10.3.1, and the no-side-slip
 * constraint of every wheeled robot in the book.
 */
export class KnifeEdge implements Dynamics {
  readonly n = 3;
  constructor(readonly m: number, readonly I: number) {}
  inertia(): Mat {
    return [
      [this.m, 0, 0],
      [0, this.m, 0],
      [0, 0, this.I],
    ];
  }
  velocityProduct(): Vec {
    return [0, 0, 0];
  }
  gravity(): Vec {
    return [0, 0, 0];
  }
  dissipation(): Vec {
    return [0, 0, 0];
  }
  potentialEnergy(): number {
    return 0;
  }
  christoffel(): number[][][] {
    return [0, 1, 2].map(() => [0, 1, 2].map(() => [0, 0, 0]));
  }
  /** The constraint as a `Pfaffian`, with Ȧ = [q̇₃ cos q₃, q̇₃ sin q₃, 0] (10.21). */
  constraint(): Pfaffian {
    return {
      A: (q) => [[Math.sin(q[2]), -Math.cos(q[2]), 0]],
      Adot: (q, qd) => [[qd[2] * Math.cos(q[2]), qd[2] * Math.sin(q[2]), 0]],
    };
  }
  /** Choset's closed forms (10.25)–(10.28) for the cross-check. */
  closedForm(q: Vec, qd: Vec, u: Vec): { qdd: Vec; lambda: number } {
    const { m, I } = this;
    const c = Math.cos(q[2]);
    const s = Math.sin(q[2]);
    const qdd1 = (u[0] * c * c + (u[1] - m * qd[0] * qd[2]) * c * s - m * qd[1] * qd[2] * s * s) / m;
    const qdd2 = (u[1] * s * s + (u[0] + m * qd[1] * qd[2]) * c * s + m * qd[0] * qd[2] * c * c) / m;
    const qdd3 = u[2] / I;
    const lambda = (u[1] - m * qd[0] * qd[2]) * c - (u[0] + m * qd[1] * qd[2]) * s;
    return { qdd: [qdd1, qdd2, qdd3], lambda };
  }
}
