/**
 * Reach's Jacobian and its singularities — Choset §3.8, Examples 3.8.1 and
 * 3.8.2. The forward kinematics φ lives in `robots/reach.ts` (Chapter 2); this
 * module differentiates it.
 *
 *   ẋ = J(q) q̇,   J(q) = ∂φ/∂q,
 *
 * and for the 2R arm
 *
 *   J = [ −L₁ sin θ₁ − L₂ sin(θ₁+θ₂)   −L₂ sin(θ₁+θ₂) ]
 *       [  L₁ cos θ₁ + L₂ cos(θ₁+θ₂)    L₂ cos(θ₁+θ₂) ],     det J = L₁ L₂ sin θ₂.
 *
 * Column i of J is the tip velocity produced by turning joint i alone at unit
 * rate: every link beyond joint i swings about it, so the column is the sum,
 * over links k ≥ i, of L_k times the link's heading rotated by 90°. That
 * description is the implementation, and it serves any N.
 *
 * A singular configuration is one where J loses rank: for 2R exactly
 * sin θ₂ = 0, arm straight or folded, where both columns point along the arm
 * and the tip cannot move radially *at this instant*. One direction is
 * unavailable; the arm has not broken.
 */

import type { Pose2 } from '../geom/se2';
import type { Point2 } from '../sim/world';
import type { Reach, Tn } from '../robots/reach';

/** A 2 × N matrix as two rows. */
export type Jacobian = [number[], number[]];
export type Mat2 = [[number, number], [number, number]];

/** J(q) = ∂φ/∂q for an N-link planar arm. O(N²) as written, O(N) with a suffix sum. */
export function jacobian(reach: Reach, q: Tn): Jacobian {
  const n = reach.n;
  const row0: number[] = new Array(n).fill(0);
  const row1: number[] = new Array(n).fill(0);
  // Headings of the links.
  const headings: number[] = [];
  let h = 0;
  for (let k = 0; k < n; k++) {
    h += q[k];
    headings.push(h);
  }
  for (let i = 0; i < n; i++) {
    // Joint i swings links i..N−1: d/dθ_i of Σ_{k≥i} L_k (cos h_k, sin h_k) = Σ_{k≥i} L_k (−sin h_k, cos h_k).
    for (let k = i; k < n; k++) {
      row0[i] += -reach.lengths[k] * Math.sin(headings[k]);
      row1[i] += reach.lengths[k] * Math.cos(headings[k]);
    }
  }
  return [row0, row1];
}

/** The 2R case as a 2 × 2 matrix. */
export function jacobian2(reach: Reach, q: Tn): Mat2 {
  if (reach.n !== 2) throw new Error('jacobian2: the 2 × 2 Jacobian is defined for the 2R arm');
  const J = jacobian(reach, q);
  return [
    [J[0][0], J[0][1]],
    [J[1][0], J[1][1]],
  ];
}

/** det J = L₁ L₂ sin θ₂ for the 2R arm, computed from the matrix (the identity is a check). */
export function detJacobian(reach: Reach, q: Tn): number {
  const J = jacobian2(reach, q);
  return J[0][0] * J[1][1] - J[0][1] * J[1][0];
}

/** Choset's remark made a predicate: singular iff |sin θ₂| ≤ tol (2R) or |det| ≤ tol. */
export function isSingular(reach: Reach, q: Tn, tol = 1e-9): boolean {
  return Math.abs(detJacobian(reach, q)) <= tol;
}

/** ẋ = J(q) q̇: the tip velocity for a joint velocity. */
export function tipVelocity(reach: Reach, q: Tn, qdot: readonly number[]): Point2 {
  const J = jacobian(reach, q);
  let vx = 0;
  let vy = 0;
  for (let i = 0; i < reach.n; i++) {
    vx += J[0][i] * qdot[i];
    vy += J[1][i] * qdot[i];
  }
  return { x: vx, y: vy };
}

/**
 * τ = Jᵀ f: a force at the tip pulled back to joint torques. The transpose is
 * the force map because power is coordinate-free: τ·q̇ = f·ẋ = f·(J q̇) for
 * every q̇. Chapter 7 lifts workspace potentials through this, Chapter 17
 * makes it dynamics.
 */
export function jointTorques(reach: Reach, q: Tn, force: Point2): number[] {
  const J = jacobian(reach, q);
  const out: number[] = [];
  for (let i = 0; i < reach.n; i++) out.push(J[0][i] * force.x + J[1][i] * force.y);
  return out;
}

/** Central differences of φ — the brute-force check that `jacobian` differentiates the right map. */
export function finiteDifferenceJacobian(reach: Reach, q: Tn, h = 1e-6): Jacobian {
  const n = reach.n;
  const row0: number[] = [];
  const row1: number[] = [];
  for (let i = 0; i < n; i++) {
    const plus = q.map((v, k) => (k === i ? v + h : v));
    const minus = q.map((v, k) => (k === i ? v - h : v));
    const a = reach.tip(plus);
    const b = reach.tip(minus);
    row0.push((a.x - b.x) / (2 * h));
    row1.push((a.y - b.y) / (2 * h));
  }
  return [row0, row1];
}

/** Singular values and vectors of a 2 × 2 matrix via the eigen-decomposition of M Mᵀ and Mᵀ M. */
export interface Svd2 {
  /** σ₁ ≥ σ₂ ≥ 0 */
  sigma: [number, number];
  /** Left singular vectors (workspace directions of the ellipse's axes). */
  u: [Point2, Point2];
  /** Right singular vectors (joint-space directions mapped onto them). */
  v: [Point2, Point2];
}

function symEigen(a: number, b: number, d: number): { l1: number; l2: number; e1: Point2; e2: Point2 } {
  // Eigenvalues of [[a, b], [b, d]].
  const tr = a + d;
  const det = a * d - b * b;
  const disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
  const l1 = tr / 2 + disc;
  const l2 = tr / 2 - disc;
  let e1: Point2;
  if (Math.abs(b) > 1e-15) e1 = { x: l1 - d, y: b };
  else if (a >= d) e1 = { x: 1, y: 0 };
  else e1 = { x: 0, y: 1 };
  const n = Math.hypot(e1.x, e1.y) || 1;
  e1 = { x: e1.x / n, y: e1.y / n };
  return { l1, l2, e1, e2: { x: -e1.y, y: e1.x } };
}

export function svd2(M: Mat2): Svd2 {
  const [[a, b], [c, d]] = M;
  // M Mᵀ gives the left vectors, Mᵀ M the right ones.
  const left = symEigen(a * a + b * b, a * c + b * d, c * c + d * d);
  const right = symEigen(a * a + c * c, a * b + c * d, b * b + d * d);
  const s1 = Math.sqrt(Math.max(0, left.l1));
  const s2 = Math.sqrt(Math.max(0, left.l2));
  // Align the signs: u_i = M v_i / σ_i when σ_i > 0.
  const apply = (v: Point2): Point2 => ({ x: a * v.x + b * v.y, y: c * v.x + d * v.y });
  const fix = (u: Point2, v: Point2, s: number): Point2 => {
    if (s < 1e-14) return u;
    const mv = apply(v);
    return mv.x * u.x + mv.y * u.y < 0 ? { x: -u.x, y: -u.y } : u;
  };
  return {
    sigma: [s1, s2],
    u: [fix(left.e1, right.e1, s1), fix(left.e2, right.e2, s2)],
    v: [right.e1, right.e2],
  };
}

/**
 * The velocity ellipse: the image of the unit circle of joint velocities under
 * J. Semi-axes σ₁, σ₂ along u₁, u₂. Its area is π σ₁ σ₂ = π |det J|, Yoshikawa's
 * manipulability for a square Jacobian — zero exactly at a singularity, where
 * the ellipse collapses to a segment along the arm.
 */
export function velocityEllipse(reach: Reach, q: Tn): { rx: number; ry: number; angle: number; svd: Svd2 } {
  const svd = svd2(jacobian2(reach, q));
  return { rx: svd.sigma[0], ry: svd.sigma[1], angle: Math.atan2(svd.u[0].y, svd.u[0].x), svd };
}

/** |det J| = σ₁ σ₂: Yoshikawa's manipulability measure for the 2R arm. */
export function manipulability(reach: Reach, q: Tn): number {
  return Math.abs(detJacobian(reach, q));
}

/**
 * Example 3.8.2 — a point r fixed on a polygon with configuration
 * q = (q₁, q₂, q₃) ∈ R² × S¹: x = (q₁, q₂) + R(q₃) r. Its Jacobian
 *   [ 1  0  −r₁ sin q₃ − r₂ cos q₃ ]
 *   [ 0  1   r₁ cos q₃ − r₂ sin q₃ ]
 * has rank 2 always: a rigid body in the plane is never singular. The third
 * column is the velocity of the point under pure rotation — the lever arm
 * rotated by 90° — which Chapter 7 uses as the force map for control points
 * on a rigid body.
 */
export function bodyPointJacobian(r: Point2, q3: number): [[number, number, number], [number, number, number]] {
  const c = Math.cos(q3);
  const s = Math.sin(q3);
  return [
    [1, 0, -r.x * s - r.y * c],
    [0, 1, r.x * c - r.y * s],
  ];
}

/** φ for Example 3.8.2: the world position of the body point r at pose q. */
export function bodyPointFk(q: Pose2, r: Point2): Point2 {
  const c = Math.cos(q.theta);
  const s = Math.sin(q.theta);
  return { x: q.x + c * r.x - s * r.y, y: q.y + s * r.x + c * r.y };
}

/** Choset's figure 3.21 numbers: L₁ = L₂ = 1, q = (π/4, π/2), q̇ = (1, 0). */
export const CHOSET_JACOBIAN_EXAMPLE = {
  q: [Math.PI / 4, Math.PI / 2] as Tn,
  qdot: [1, 0] as readonly number[],
  J: [
    [-Math.SQRT2, -Math.SQRT1_2],
    [0, -Math.SQRT1_2],
  ] as Mat2,
  xdot: { x: -Math.SQRT2, y: 0 } as Point2,
  det: 1,
};
