/**
 * The generalized Voronoi graph — Choset §5.3 — and the Lyapunov tracing law
 * of §5.3.3 (eq. 5.5), in any dimension the book needs (2 and 3).
 *
 * In R^m the GVG is the set of points equidistant to m closest obstacles with
 * distinct gradients; the constraint vector
 *
 *     G(q) = [d_1 − d_2, d_1 − d_3, …, d_1 − d_m]ᵀ          ((m−1) rows)
 *
 * has DG with rows ∇d_1 − ∇d_j, and G⁻¹(0) is one-dimensional wherever DG has
 * full rank (the preimage theorem, with transversality supplying the rank).
 * The tracing law
 *
 *     q̇ = α v + β (DG)† G,   v ∈ Null(DG), ‖v‖ = 1,   (DG)† = DGᵀ (DG DGᵀ)⁻¹
 *
 * has Γ = ½ GᵀG as a Lyapunov function: Γ̇ = β GᵀG, strictly negative off the
 * GVG when β < 0. Homing onto a meet point is the same law with one more row
 * (an empty null space, so v = 0). Everything here is sensor-ready: the only
 * input is the sorted list of the nearest obstacles with their distances and
 * gradients, which a range ring provides as the local minima of ρ (Chapter 3,
 * Chapter 7's `scanLocalMinima`) and which exact geometry provides for the
 * checks. Nothing in this file knows what an obstacle is.
 *
 * Chapter 8's `gvd.ts` traced the planar GVD with Chapter 3's discrete
 * predictor–corrector. This file is its continuous cousin: no predictor, no
 * corrector, one velocity. The sign of the tangent is the planner's choice
 * (Choset's footnote 4): we keep it aligned with the previous heading.
 */

import { scanLocalMinima } from '../potential/planar';
import type { Scan } from '../bugs/scan';

/** A configuration or tangent vector as a plain array (nalgebra's SVector in the browser). */
export type Vec = number[];

/** One of the k nearest obstacles: its distance, unit gradient ∇d_i (away from it) and closest point. */
export interface Nearest {
  d: number;
  grad: Vec;
  closest?: Vec;
  /** An index when the query can name obstacles (geometry); −1 from a scan, which cannot. */
  index: number;
}

/** A query answering "what are the nearest obstacles at q, nearest first?" — geometry or a scan. */
export type NearestQuery = (q: Vec) => Nearest[];

/* -------------------------------------------------------------------------- */
/* Small dense linear algebra (rows ≤ 3)                                        */
/* -------------------------------------------------------------------------- */

export const vdot = (a: Vec, b: Vec): number => a.reduce((s, x, i) => s + x * b[i], 0);
export const vnorm = (a: Vec): number => Math.sqrt(vdot(a, a));
export const vsub = (a: Vec, b: Vec): Vec => a.map((x, i) => x - b[i]);
export const vadd = (a: Vec, b: Vec): Vec => a.map((x, i) => x + b[i]);
export const vscale = (a: Vec, k: number): Vec => a.map((x) => x * k);
export const vunit = (a: Vec): Vec => {
  const n = vnorm(a);
  return n > 1e-15 ? vscale(a, 1 / n) : a.map(() => 0);
};
export const vdist = (a: Vec, b: Vec): number => vnorm(vsub(a, b));
export const cross3 = (a: Vec, b: Vec): Vec => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

/** Solve the small square system A x = b by Gaussian elimination with partial pivoting; null if singular. */
export function solveSquare(A: number[][], b: Vec, singularTol = 1e-12): Vec | null {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < singularTol) return null;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}

/**
 * The Moore–Penrose pseudoinverse of a full-row-rank r × m matrix,
 * DG† = DGᵀ (DG DGᵀ)⁻¹, returned as an m × r matrix. `null` when DG DGᵀ is
 * singular — exactly the configurations where the law has no neighborhood
 * of stability (two gradients parallel, or a meet point for the edge law).
 */
export function pseudoInverse(DG: number[][], singularTol = 1e-10): number[][] | null {
  const r = DG.length;
  const m = DG[0].length;
  const gram: number[][] = Array.from({ length: r }, (_, i) => Array.from({ length: r }, (_, j) => vdot(DG[i], DG[j])));
  // Invert the Gram matrix column by column.
  const inv: number[][] = Array.from({ length: r }, () => new Array<number>(r).fill(0));
  for (let c = 0; c < r; c++) {
    const e = new Array<number>(r).fill(0);
    e[c] = 1;
    const col = solveSquare(gram, e, singularTol);
    if (!col) return null;
    for (let i = 0; i < r; i++) inv[i][c] = col[i];
  }
  // DGᵀ · inv : (m × r)
  const out: number[][] = Array.from({ length: m }, () => new Array<number>(r).fill(0));
  for (let i = 0; i < m; i++) for (let j = 0; j < r; j++) for (let k = 0; k < r; k++) out[i][j] += DG[k][i] * inv[k][j];
  return out;
}

/** y = M x for a dense matrix. */
export function matVec(M: number[][], x: Vec): Vec {
  return M.map((row) => vdot(row, x));
}

/**
 * The unit tangent of G⁻¹(0): a basis vector of Null(DG) when DG has m − 1
 * rows in R^m. In the plane it is the perpendicular of the single row; in
 * space the cross product of the two rows. The sign follows `heading`
 * (Choset's footnote 4: "the direction that directs the robot to continue
 * in the same direction"). The zero vector when the rows are dependent.
 */
export function nullSpaceDirection(DG: number[][], heading: Vec): Vec {
  const m = DG[0].length;
  let v: Vec;
  if (m === 2 && DG.length === 1) v = [-DG[0][1], DG[0][0]];
  else if (m === 3 && DG.length === 2) v = cross3(DG[0], DG[1]);
  else return heading.map(() => 0);
  const n = vnorm(v);
  if (n < 1e-12) return heading.map(() => 0);
  v = vscale(v, 1 / n);
  return vdot(v, heading) < 0 ? vscale(v, -1) : v;
}

/* -------------------------------------------------------------------------- */
/* Transversality (§5.3.1)                                                     */
/* -------------------------------------------------------------------------- */

/**
 * How transversal is the intersection of the sheets S_1j at q? The rows of
 * DG are the sheet normals ∇d_1 − ∇d_j; the sheets meet transversally iff the
 * rows are independent, and the smallest singular value of DG (equivalently
 * √λ_min of the Gram matrix) measures the margin. Zero means two sheets are
 * tangent — the unstable "parallel" or "overlap" pictures of C Fig. 5.17 —
 * and the preimage theorem no longer promises a curve.
 */
export function transversalityMargin(DG: number[][]): number {
  const r = DG.length;
  const gram = Array.from({ length: r }, (_, i) => Array.from({ length: r }, (_, j) => vdot(DG[i], DG[j])));
  if (r === 1) return Math.sqrt(gram[0][0]);
  if (r === 2) {
    const [a, b] = [gram[0][0], gram[1][1]];
    const c = gram[0][1];
    const tr = a + b;
    const det = a * b - c * c;
    const lmin = tr / 2 - Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
    return Math.sqrt(Math.max(0, lmin));
  }
  // r = 3: smallest eigenvalue of a 3×3 symmetric matrix by a few inverse-free iterations (Gershgorin bisection).
  const f = (x: number) => {
    // det(gram − xI)
    const M = gram.map((row, i) => row.map((v, j) => (i === j ? v - x : v)));
    return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  };
  let lo = 0;
  let hi = Math.max(...gram.map((row) => row.reduce((s, v) => s + Math.abs(v), 0)));
  // The characteristic polynomial is negative below the smallest eigenvalue (leading coefficient −1), positive just above it.
  for (let k = 0; k < 80; k++) {
    const mid = 0.5 * (lo + hi);
    if (f(mid) < 0) lo = mid;
    else hi = mid;
  }
  return Math.sqrt(Math.max(0, 0.5 * (lo + hi)));
}

/* -------------------------------------------------------------------------- */
/* The constraint vector and the law (eq. 5.5)                                 */
/* -------------------------------------------------------------------------- */

/** G and DG from the k nearest obstacles: rows d_1 − d_j, ∇d_1 − ∇d_j for j = 2..k. */
export function constraint(near: readonly Nearest[], k: number): { G: Vec; DG: number[][] } {
  const G: Vec = [];
  const DG: number[][] = [];
  for (let j = 1; j < k; j++) {
    G.push(near[0].d - near[j].d);
    DG.push(vsub(near[0].grad, near[j].grad));
  }
  return { G, DG };
}

export interface LawStep {
  /** The commanded velocity q̇. */
  qdot: Vec;
  G: Vec;
  DG: number[][];
  DGdag: number[][] | null;
  /** The tangent term α v (zero for homing). */
  tangent: Vec;
  /** The correction term β (DG)† G. */
  correction: Vec;
  /** Γ = ½ GᵀG and its derivative along the law, Γ̇ = β GᵀG. */
  gamma: number;
  dGamma: number;
  /** Smallest singular value of DG: the transversality margin at q. */
  margin: number;
  /** DG DGᵀ was singular: the law is undefined here (a meet point for the edge law, or parallel gradients). */
  singular: boolean;
}

/**
 * One evaluation of eq. (5.5) at a configuration where the k nearest
 * obstacles are known. With k = m (the ambient dimension) this traces an
 * edge: G has m − 1 rows, Null(DG) is a line, and v is its unit vector
 * aligned with `heading`. With k = m + 1 it homes onto a meet point: DG is
 * square, Null(DG) = {0}, and q̇ = β (DG)† G = β DG⁻¹ G is a Newton step
 * toward the centre of the sphere through the m + 1 closest points.
 */
export function lawStep(near: readonly Nearest[], k: number, alpha: number, beta: number, heading: Vec): LawStep {
  const m = heading.length;
  const { G, DG } = constraint(near, k);
  const DGdag = pseudoInverse(DG);
  const margin = transversalityMargin(DG);
  const zero = new Array<number>(m).fill(0);
  if (!DGdag) {
    return { qdot: zero, G, DG, DGdag: null, tangent: zero, correction: zero, gamma: 0.5 * vdot(G, G), dGamma: 0, margin, singular: true };
  }
  const v = k === m ? nullSpaceDirection(DG, heading) : zero;
  const tangent = vscale(v, alpha);
  const correction = vscale(matVec(DGdag, G), beta);
  const qdot = vadd(tangent, correction);
  const gg = vdot(G, G);
  return { qdot, G, DG, DGdag, tangent, correction, gamma: 0.5 * gg, dGamma: beta * gg, margin, singular: false };
}

/** Edge tracing (k = dim): q̇ = α v + β (DG)† G. */
export const traceLaw = (near: readonly Nearest[], alpha: number, beta: number, heading: Vec): LawStep => lawStep(near, heading.length, alpha, beta, heading);

/** Meet-point homing (k = dim + 1): q̇ = β (DG)† G, the null space being empty. */
export const homeLaw = (near: readonly Nearest[], beta: number, dim: number): LawStep => lawStep(near, dim + 1, 0, beta, new Array<number>(dim).fill(0));

/**
 * Integrate the tracing law with explicit Euler steps of `dt` from q0 for
 * `steps` steps, recording Γ along the way — the Lyapunov box's example:
 * Γ never increases when β < 0, and the robot ends on the GVG.
 */
export function integrateTracer(
  query: NearestQuery,
  q0: Vec,
  heading0: Vec,
  alpha: number,
  beta: number,
  dt: number,
  steps: number,
): { path: Vec[]; gamma: number[]; headings: Vec[] } {
  let q = q0.slice();
  let heading = vunit(heading0);
  const path = [q.slice()];
  const gamma: number[] = [];
  const headings = [heading.slice()];
  for (let s = 0; s < steps; s++) {
    const near = query(q);
    if (near.length < q.length) break;
    const st = traceLaw(near, alpha, beta, heading);
    gamma.push(st.gamma);
    if (st.singular) break;
    q = vadd(q, vscale(st.qdot, dt));
    const t = vunit(st.tangent);
    if (vnorm(t) > 0) heading = t;
    path.push(q.slice());
    headings.push(heading.slice());
  }
  const near = query(q);
  if (near.length >= q.length) gamma.push(0.5 * vdot(constraint(near, q.length).G, constraint(near, q.length).G));
  return { path, gamma, headings };
}

/**
 * Newton homing to a meet point: iterate q ← q + (DG)⁻¹(−G) with the
 * m + 1 nearest obstacles until ‖G‖ < tol. This is the homing law run to
 * convergence with β dt = −1, i.e. the fixed point the robot converges to.
 * Returns null when the Jacobian is singular or Newton wanders.
 */
export function homeToMeetPoint(query: NearestQuery, q0: Vec, iters = 50, tol = 1e-11, acceptTol = 1e-6): { q: Vec; near: Nearest[]; residual: number } | null {
  let q = q0.slice();
  const m = q.length;
  // Quantized or noisy readings make ‖G‖ stall above `tol`; keep the best iterate and accept it
  // when it is below `acceptTol`.
  let best: { q: Vec; near: Nearest[]; residual: number } | null = null;
  for (let it = 0; it < iters; it++) {
    const near = query(q);
    if (near.length < m + 1) return best && best.residual < acceptTol ? best : null;
    const { G, DG } = constraint(near, m + 1);
    const r = vnorm(G);
    if (!best || r < best.residual) best = { q: q.slice(), near, residual: r };
    if (r < tol) return { q, near, residual: r };
    const dq = solveSquare(DG, vscale(G, -1), 1e-12);
    if (!dq) break;
    const n = vnorm(dq);
    // Distance functions are 1-Lipschitz: never jump farther than the current clearance.
    const cap = Math.max(0.25 * near[0].d, 1e-3);
    q = vadd(q, n > cap ? vscale(dq, cap / n) : dq);
  }
  return best && best.residual < acceptTol ? best : null;
}

/**
 * Newton projection onto the GVG edge through the m nearest obstacles —
 * the correction term alone, iterated: q ← q − (DG)† G. Used to place a
 * seed exactly on an edge before tracing it.
 */
export function projectToGvg(query: NearestQuery, q0: Vec, iters = 40, tol = 1e-12): Vec | null {
  let q = q0.slice();
  const m = q.length;
  for (let it = 0; it < iters; it++) {
    const near = query(q);
    if (near.length < m) return null;
    const { G, DG } = constraint(near, m);
    if (vnorm(G) < tol) return q;
    const dag = pseudoInverse(DG);
    if (!dag) return null;
    const dq = matVec(dag, vscale(G, -1));
    const n = vnorm(dq);
    const cap = Math.max(0.5 * near[0].d, 1e-3);
    q = vadd(q, n > cap ? vscale(dq, cap / n) : dq);
  }
  const near = query(q);
  return near.length >= m && vnorm(constraint(near, m).G) < 1e-8 ? q : null;
}

/* -------------------------------------------------------------------------- */
/* Departure directions at a meet point                                        */
/* -------------------------------------------------------------------------- */

/**
 * The m + 1 edges leaving a meet point in R^m. Edge e omits the e-th nearest
 * obstacle: along it the other m stay tied while the omitted one falls
 * behind, so its tangent spans Null of the rows ∇d_a − ∇d_b over the kept
 * obstacles and is signed so that d_omitted − d_kept increases.
 */
export function departureDirections(near: readonly Nearest[], dim: number): Vec[] {
  const out: Vec[] = [];
  for (let omit = 0; omit <= dim; omit++) {
    const kept = near.filter((_, i) => i !== omit).slice(0, dim);
    const rows: number[][] = [];
    for (let j = 1; j < dim; j++) rows.push(vsub(kept[0].grad, kept[j].grad));
    const grow = vsub(near[omit].grad, kept[0].grad); // ∇(d_omit − d_kept)
    let v: Vec;
    if (dim === 2) v = [-rows[0][1], rows[0][0]];
    else v = cross3(rows[0], rows[1]);
    const n = vnorm(v);
    if (n < 1e-12) {
      out.push(v.map(() => 0));
      continue;
    }
    v = vscale(v, 1 / n);
    if (vdot(v, grow) < 0) v = vscale(v, -1);
    out.push(v);
  }
  return out;
}

/**
 * Departures at a *nearly degenerate* planar meet point: k ≥ 4 readings tie
 * to within the sensor's resolution, so what is geometrically two (or more)
 * meet points joined by edges shorter than the node tolerance is one node to
 * the robot. Transversality says exact degeneracy has probability zero; the
 * Apartment's mirrored doors produce ties 2 mm apart all the same. The GVG
 * edges leaving the cluster are the bisectors of *adjacent* closest points
 * around the robot — k of them — each heading into the sector between its
 * two obstacles (toward the chord), where those two stay nearest and every
 * other obstacle falls behind.
 */
export function degenerateDepartures2D(near: readonly Nearest[], k: number): Vec[] {
  const idx = near.slice(0, k).map((n, i) => ({ i, phi: Math.atan2(-n.grad[1], -n.grad[0]) })).sort((a, b) => a.phi - b.phi);
  const out: Vec[] = [];
  for (let a = 0; a < k; a++) {
    const i = idx[a].i;
    const j = idx[(a + 1) % k].i;
    const row = vsub(near[i].grad, near[j].grad);
    let t: Vec = [-row[1], row[0]];
    const n = vnorm(t);
    if (n < 1e-12) continue;
    t = vscale(t, 1 / n);
    // Toward the chord c_i c_j, i.e. against the mean gradient of the pair.
    if (vdot(t, vadd(near[i].grad, near[j].grad)) > 0) t = vscale(t, -1);
    out.push(t);
  }
  return out;
}

/* -------------------------------------------------------------------------- */
/* Range-ring front end (§5.3.3: "solely from range sensor information")       */
/* -------------------------------------------------------------------------- */

/**
 * The nearest obstacles as a range ring sees them: the local minima of ρ,
 * sorted by distance. The gradient of each is the unit vector *back along*
 * its ray — Chapter 7's reading of the scan — and no obstacle has a name.
 */
export function nearestFromScan(scan: Scan): Nearest[] {
  return scanLocalMinima(scan)
    .map((o) => ({ d: o.d, grad: o.grad, closest: o.closest, index: -1 }))
    .sort((a, b) => a.d - b.d);
}

/**
 * Two readings that come from the same obstacle feature (a wall seen across
 * the 0/2π seam, a plateau the minima finder split) would make DG singular
 * and fool the meet detector. Merge minima whose gradients are within
 * `angle` radians and whose distances agree to `dtol`.
 */
export function mergeDuplicateMinima(near: Nearest[], angle = 0.05, dtol = 1e-6): Nearest[] {
  const out: Nearest[] = [];
  for (const n of near) {
    const dup = out.some((o) => Math.abs(o.d - n.d) < dtol && Math.acos(Math.max(-1, Math.min(1, vdot(o.grad, n.grad)))) < angle);
    if (!dup) out.push(n);
  }
  return out;
}

/* -------------------------------------------------------------------------- */
/* The design doc's micro-example                                              */
/* -------------------------------------------------------------------------- */

/** Point sites as a `NearestQuery` in the plane: the micro-world of §3 and w9.3. */
export function pointSitesQuery(sites: readonly Vec[]): NearestQuery {
  return (q) =>
    sites
      .map((c, index) => {
        const diff = vsub(q, c);
        const d = vnorm(diff);
        return { d, grad: d > 1e-15 ? vscale(diff, 1 / d) : diff.map(() => 0), closest: c.slice(), index };
      })
      .sort((a, b) => a.d - b.d);
}

export const MICRO_SITES: Vec[] = [
  [0, 0],
  [4, 0],
];
export const MICRO_Q: Vec = [2.5, 1];

/**
 * Two point obstacles at (0, 0) and (4, 0), Rusty at (2.5, 1), α = 1, β = −1:
 * the chapter's contract with its check. Prints G, DG, (DG)†, v, q̇, Γ, Γ̇.
 */
export function lyapunovMicroExample(alpha = 1, beta = -1) {
  const query = pointSitesQuery(MICRO_SITES);
  // G = d₁ − d₂ with the sites as *labelled* (site 1 at the origin), not sorted by distance:
  // the law is the same either way (q̇, Γ, Γ̇ do not depend on the sign of G), the printed DG is not.
  const near = query(MICRO_Q).sort((a, b) => a.index - b.index);
  const st = traceLaw(near, alpha, beta, [0, 1]);
  return {
    d1: near.find((n) => n.index === 0)!.d,
    d2: near.find((n) => n.index === 1)!.d,
    G: st.G[0],
    DG: st.DG[0],
    DGdag: st.DGdag ? [st.DGdag[0][0], st.DGdag[1][0]] : [NaN, NaN],
    v: vunit(st.tangent),
    correction: st.correction,
    qdot: st.qdot,
    gamma: st.gamma,
    dGamma: st.dGamma,
  };
}

/* -------------------------------------------------------------------------- */
/* Complexity (§5.5.1 and Appendix D)                                          */
/* -------------------------------------------------------------------------- */

/**
 * Canny's bound p^n (log p) w^{O(n⁴)} with the O(·) constant made explicit:
 * the number of "operations" for p polynomial constraints of degree ≤ w in an
 * n-dimensional configuration space. Singly exponential in n — complete, and
 * never practical, which is the honest closing of the chapter.
 */
export function cannyBound(p: number, n: number, w: number, c = 1): number {
  return Math.pow(p, n) * Math.log(p) * Math.pow(w, c * Math.pow(n, 4));
}

/** log₁₀ of Canny's bound, for tables that would otherwise overflow a double. */
export function cannyBoundLog10(p: number, n: number, w: number, c = 1): number {
  return n * Math.log10(p) + Math.log10(Math.log(p)) + c * Math.pow(n, 4) * Math.log10(w);
}
