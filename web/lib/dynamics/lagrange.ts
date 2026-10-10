/**
 * The Lagrange recipe's second half (Choset §10.2): from an inertia matrix to
 * its Christoffel symbols, the velocity-product vector, the Coriolis matrix,
 * and the skew-symmetry diagnostic that makes energy conservation a test.
 *
 *     Γ^i_jk(q) = ½ ( ∂M_ij/∂q_k + ∂M_ik/∂q_j − ∂M_jk/∂q_i )           (10.9)
 *
 * The n³ symbols are stored as `Gamma[i][j][k]`, so `Gamma[i]` is Choset's
 * symmetric n×n slice Γ^i(q) and q̇ᵀ Γ^i q̇ is the i-th velocity-product term.
 * Centrifugal terms are the j = k diagonal; Coriolis terms are the rest.
 *
 * Why finite differences are allowed here: Choset derives Γ symbolically with
 * Mathematica. The 2R arm has a four-symbol closed form (`reach2r.ts`), and the
 * check `christoffel_closed_form_matches_finite_differences` pins the two
 * against each other to 1e-7. For the 3R arm only the finite-difference path
 * exists, and the text says so.
 */

import type { Mat, Vec } from '../prob/linalg';

/** Γ[i][j][k] — n slices, each an n×n symmetric matrix in (j, k). */
export type Christoffel = number[][][];

/** ∂M/∂q_k for every k, by central differences of step h. */
export function inertiaDerivatives(M: (q: Vec) => Mat, q: Vec, h = 1e-6): Mat[] {
  const n = q.length;
  const out: Mat[] = [];
  for (let k = 0; k < n; k++) {
    const qp = q.slice();
    const qm = q.slice();
    qp[k] += h;
    qm[k] -= h;
    const Mp = M(qp);
    const Mm = M(qm);
    out.push(Mp.map((row, i) => row.map((v, j) => (v - Mm[i][j]) / (2 * h))));
  }
  return out;
}

/** Equation (10.9) from the partials ∂M/∂q_k. */
export function christoffelFromDerivatives(dM: Mat[]): Christoffel {
  const n = dM.length;
  const G: Christoffel = [];
  for (let i = 0; i < n; i++) {
    const slice: number[][] = [];
    for (let j = 0; j < n; j++) {
      const row: number[] = [];
      for (let k = 0; k < n; k++) {
        row.push(0.5 * (dM[k][i][j] + dM[j][i][k] - dM[i][j][k]));
      }
      slice.push(row);
    }
    G.push(slice);
  }
  return G;
}

/** The generic `christoffel`: central differences of M, then (10.9). O(n³) symbols from 2n evaluations of M. */
export function christoffelFromInertia(M: (q: Vec) => Mat, q: Vec, h = 1e-6): Christoffel {
  return christoffelFromDerivatives(inertiaDerivatives(M, q, h));
}

/** c_i(q, q̇) = Σ_jk Γ^i_jk q̇_j q̇_k (10.8). */
export function velocityProductFromChristoffel(G: Christoffel, qd: Vec): Vec {
  const n = G.length;
  const out = new Array<number>(n).fill(0);
  for (let i = 0; i < n; i++) {
    let s = 0;
    for (let j = 0; j < n; j++) for (let k = 0; k < n; k++) s += G[i][j][k] * qd[j] * qd[k];
    out[i] = s;
  }
  return out;
}

/**
 * One common Coriolis matrix: C_ij = Σ_k Γ^i_jk q̇_k (Choset §10.2; Murray,
 * Li & Sastry's Γ_ijk with the force index first). Contracting with q̇_j gives
 * back c_i = Σ_jk Γ^i_jk q̇_j q̇_k exactly, and this is the choice for which
 * Ṁ − 2C is skew-symmetric — the property Derivation 2 turns into the
 * chapter's energy test. Any C' = C + S with S q̇ = 0 gives the same equations
 * of motion but may lose the skew property (Exercise 1).
 */
export function coriolisMatrix(G: Christoffel, qd: Vec): Mat {
  const n = G.length;
  const C: Mat = [];
  for (let i = 0; i < n; i++) {
    const row: number[] = [];
    for (let j = 0; j < n; j++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += G[i][j][k] * qd[k];
      row.push(s);
    }
    C.push(row);
  }
  return C;
}

/** Ṁ = Σ_k (∂M/∂q_k) q̇_k. */
export function inertiaRate(dM: Mat[], qd: Vec): Mat {
  const n = dM.length;
  const out: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  for (let k = 0; k < n; k++) for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) out[i][j] += dM[k][i][j] * qd[k];
  return out;
}

/**
 * max |(Ṁ − 2C)_ij + (Ṉ − 2C)_ji| — zero (to rounding) when C is the
 * Christoffel choice. The design doc's `skew_symmetry_defect` diagnostic.
 */
export function skewSymmetryDefect(M: (q: Vec) => Mat, q: Vec, qd: Vec, h = 1e-6): number {
  const dM = inertiaDerivatives(M, q, h);
  const G = christoffelFromDerivatives(dM);
  const C = coriolisMatrix(G, qd);
  const Md = inertiaRate(dM, qd);
  const n = q.length;
  let worst = 0;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      const sij = Md[i][j] - 2 * C[i][j];
      const sji = Md[j][i] - 2 * C[j][i];
      worst = Math.max(worst, Math.abs(sij + sji));
    }
  }
  return worst;
}

/** Symmetry defect of Γ in its lower indices: max_ijk |Γ^i_jk − Γ^i_kj|. Zero by construction of (10.9). */
export function christoffelAsymmetry(G: Christoffel): number {
  let worst = 0;
  const n = G.length;
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) for (let k = 0; k < n; k++) worst = Math.max(worst, Math.abs(G[i][j][k] - G[i][k][j]));
  return worst;
}

/** Eigen-decomposition of a symmetric 2×2: eigenvalues descending and unit eigenvectors (the inertia ellipse). */
export function symmetricEigen2(M: Mat): { values: [number, number]; vectors: [[number, number], [number, number]] } {
  const a = M[0][0];
  const b = M[0][1];
  const d = M[1][1];
  const tr = a + d;
  const det = a * d - b * b;
  const disc = Math.sqrt(Math.max((tr * tr) / 4 - det, 0));
  const l1 = tr / 2 + disc;
  const l2 = tr / 2 - disc;
  let v1: [number, number];
  if (Math.abs(b) < 1e-14) v1 = a >= d ? [1, 0] : [0, 1];
  else {
    const vx = l1 - d;
    const vy = b;
    const nrm = Math.hypot(vx, vy);
    v1 = [vx / nrm, vy / nrm];
  }
  const v2: [number, number] = [-v1[1], v1[0]];
  return { values: [l1, l2], vectors: [v1, v2] };
}
