/**
 * The standard form of a fully actuated mechanical system (Choset §10.2):
 *
 *     M(q) q̈ + C(q, q̇) q̇ + g(q) + b(q, q̇) = u            (10.7, with friction)
 *
 * Everything downstream — time scaling (Ch. 18), iLQR (Ch. 19), kinodynamic
 * planning (Ch. 21), the capstone — talks to this interface and nothing else.
 * The Rust side is `trait Dynamics<const N: usize>`; here dimensions are
 * runtime (`n`) and vectors are plain arrays from `prob/linalg`, because the
 * routines are meant to be *read* beside the equations they implement.
 *
 * Two deliberate choices, both Choset's:
 *
 *  - `velocityProduct` returns the *vector* C(q, q̇) q̇ = q̇ᵀ Γ(q) q̇, not a
 *    matrix. The matrix C is not unique; only the product is a generalized
 *    force (§10.2, "neither M q̈ nor C q̇ individually should be thought of as a
 *    generalized force"). `coriolis()` builds the Christoffel choice of C when
 *    a matrix is wanted — it is the one that makes Ṁ − 2C skew-symmetric.
 *  - gravity is ∂V/∂q, so every model carries its potential energy and the
 *    energy bookkeeping E = K + V is available to the integrator tests.
 */

import { cholesky, matVec, solve, type Mat, type Vec } from '../prob/linalg';
import { christoffelFromInertia, coriolisMatrix, type Christoffel } from './lagrange';

export interface Dynamics {
  /** Degrees of freedom n_Q. */
  readonly n: number;
  /** M(q): symmetric positive definite inertia matrix (10.13–10.14). */
  inertia(q: Vec): Mat;
  /** C(q, q̇) q̇ = q̇ᵀ Γ(q) q̇ — the velocity-product *vector* (10.8, 10.10). */
  velocityProduct(q: Vec, qd: Vec): Vec;
  /** g(q) = ∂V/∂q. */
  gravity(q: Vec): Vec;
  /** Dissipation b(q, q̇); zero for a frictionless model. */
  dissipation(q: Vec, qd: Vec): Vec;
  /** V(q), so that E = K + V can be tracked. */
  potentialEnergy(q: Vec): number;
  /**
   * Christoffel symbols Γ(q), n slices of n×n. Closed form where the model has
   * one (Reach2R), central differences of `inertia` otherwise.
   */
  christoffel(q: Vec): Christoffel;
}

/**
 * The shared half of every model: the standard-form algebra, given M, Γ, g, b, V.
 * Subclasses supply the physics; this class never knows what a link is.
 */
export abstract class MechanicalSystem implements Dynamics {
  abstract readonly n: number;
  abstract inertia(q: Vec): Mat;
  abstract gravity(q: Vec): Vec;
  abstract potentialEnergy(q: Vec): number;

  /** Viscous damping coefficient per joint; 0 by default (the conservative model the tests want). */
  damping: Vec | null = null;

  /** Default: central differences of M(q). Override with a closed form when there is one. */
  christoffel(q: Vec): Christoffel {
    return christoffelFromInertia((x) => this.inertia(x), q);
  }

  /** c_i = Σ_jk Γ^i_jk q̇_j q̇_k (10.8). */
  velocityProduct(q: Vec, qd: Vec): Vec {
    const G = this.christoffel(q);
    const out = new Array<number>(this.n).fill(0);
    for (let i = 0; i < this.n; i++) {
      let s = 0;
      for (let j = 0; j < this.n; j++) for (let k = 0; k < this.n; k++) s += G[i][j][k] * qd[j] * qd[k];
      out[i] = s;
    }
    return out;
  }

  dissipation(_q: Vec, qd: Vec): Vec {
    if (!this.damping) return new Array<number>(this.n).fill(0);
    return qd.map((v, i) => (this.damping as Vec)[i] * v);
  }
}

/** The Christoffel choice C_ij = Σ_k Γ^k_ij q̇_k — the matrix for which Ṁ − 2C is skew (Derivation 2). */
export function coriolis(dyn: Dynamics, q: Vec, qd: Vec): Mat {
  return coriolisMatrix(dyn.christoffel(q), qd);
}

/** K = ½ q̇ᵀ M(q) q̇ (10.13). */
export function kineticEnergy(dyn: Dynamics, q: Vec, qd: Vec): number {
  const Mqd = matVec(dyn.inertia(q), qd);
  let k = 0;
  for (let i = 0; i < qd.length; i++) k += qd[i] * Mqd[i];
  return 0.5 * k;
}

/** E = K + V — the quantity the integrator tests watch. */
export function totalEnergy(dyn: Dynamics, q: Vec, qd: Vec): number {
  return kineticEnergy(dyn, q, qd) + dyn.potentialEnergy(q);
}

/**
 * Forward dynamics q̈ = M⁻¹(u − C q̇ − g − b) (Choset eq. 11.46): one n×n
 * solve. M is symmetric positive definite, so a Cholesky solve would do; for
 * n ≤ 3 the pivoted Gaussian solve from `prob/linalg` is just as fast and does
 * not care whether a reader's experimental model is a hair indefinite.
 */
export function forward(dyn: Dynamics, q: Vec, qd: Vec, tau: Vec): Vec {
  const c = dyn.velocityProduct(q, qd);
  const g = dyn.gravity(q);
  const b = dyn.dissipation(q, qd);
  const rhs = tau.map((t, i) => t - c[i] - g[i] - b[i]);
  return solve(dyn.inertia(q), rhs);
}

/** Inverse dynamics u = M q̈ + C q̇ + g + b — the torque that produces a given acceleration. */
export function inverse(dyn: Dynamics, q: Vec, qd: Vec, qdd: Vec): Vec {
  const Mqdd = matVec(dyn.inertia(q), qdd);
  const c = dyn.velocityProduct(q, qd);
  const g = dyn.gravity(q);
  const b = dyn.dissipation(q, qd);
  return Mqdd.map((m, i) => m + c[i] + g[i] + b[i]);
}

/**
 * Jᵀ actuation (Choset eq. 10.12): forces f applied at a body point whose
 * workspace Jacobian is J become the generalized force u = Jᵀ f. The same
 * virtual-work identity Chapter 7 §4.7 used to pull workspace potentials into
 * joint space, and the one Chapter 19's CHOMP uses to pull obstacle gradients.
 */
export function jtActuation(J: Mat, f: Vec): Vec {
  const n = J[0].length;
  const u = new Array<number>(n).fill(0);
  for (let r = 0; r < J.length; r++) for (let c = 0; c < n; c++) u[c] += J[r][c] * f[r];
  return u;
}

/**
 * Positive definiteness by attempted Cholesky with a strict pivot test. The
 * library `cholesky` clamps tiny pivots to keep filters alive; a dynamics
 * test must not be that forgiving, so this checks the pivots directly.
 */
export function isPositiveDefinite(M: Mat, tol = 1e-12): boolean {
  const n = M.length;
  const L = cholesky(M);
  for (let i = 0; i < n; i++) {
    let s = 0;
    for (let k = 0; k < i; k++) s += L[i][k] * L[i][k];
    if (M[i][i] - s <= tol) return false;
    if (Math.abs(M[i][i] - (s + L[i][i] * L[i][i])) > 1e-9 * Math.max(1, Math.abs(M[i][i]))) return false;
  }
  return true;
}
