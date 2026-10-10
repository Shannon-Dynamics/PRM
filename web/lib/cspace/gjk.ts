/**
 * GJK — the distance between convex polygons, Choset Appendix F.5 (Alg. 23,
 * Gilbert, Johnson and Keerthi 1988). This is what `parry2d::query::distance`
 * does inside the Rust crate and what Chapter 2 asked the reader to take on
 * faith. Opened here in two dimensions.
 *
 * The idea: d(A, B) = min_{a∈A, b∈B} ‖a − b‖ = min_{z ∈ A ⊖ B} ‖z‖ (eq. F.10),
 * so the distance between two bodies is the distance from one convex set, the
 * Minkowski difference Z = A ⊖ B, to the origin. Z is never built. All GJK
 * ever asks of Z is its support function h_Z(x) = max z·x and support point
 * s_Z(x), and both compose (eqs. F.13–F.14):
 *
 *     h_{A⊖B}(x) = h_A(x) + h_B(−x),     s_{A⊖B}(x) = s_A(x) − s_B(−x).
 *
 * The iteration keeps a simplex of at most three points of Z, finds the point
 * x_k of that simplex nearest the origin, and asks Z for its support point in
 * direction −x_k. If that point is no closer to the origin than x_k already
 * is, x_k is the answer; otherwise it joins the simplex and the step repeats.
 * Because the norm is strictly convex and Z convex, the minimizer z* is unique
 * (the witness pair a*, b* need not be — Choset's remark after eq. F.10).
 */

import type { Point2 } from '../sim/world';
import type { Polygon } from '../collide/polygon';

/** A point of Z = A ⊖ B, remembered with the pair that produced it. */
interface ZPoint {
  z: Point2;
  a: Point2;
  b: Point2;
}

export interface GjkResult {
  dist: number;
  /** Witnesses: the nearest points on A and on B (any contact point when intersecting). */
  onA: Point2;
  onB: Point2;
  iterations: number;
  /** The simplices visited, for the exposition widget and the checks. */
  simplices: Point2[][];
  intersecting: boolean;
}

/** h_P(x) and s_P(x): the most extreme vertex of a polygon in a direction. */
export function support(poly: Polygon, dx: number, dy: number): { point: Point2; value: number } {
  let best = poly[0];
  let bestV = -Infinity;
  for (const p of poly) {
    const v = p.x * dx + p.y * dy;
    if (v > bestV) {
      bestV = v;
      best = p;
    }
  }
  return { point: best, value: bestV };
}

/** s_{A⊖B}(x) = s_A(x) − s_B(−x), carrying both witnesses. */
function supportDiff(A: Polygon, B: Polygon, dx: number, dy: number): ZPoint {
  const a = support(A, dx, dy).point;
  const b = support(B, -dx, -dy).point;
  return { z: { x: a.x - b.x, y: a.y - b.y }, a, b };
}

/** Nearest point of a segment to the origin, with barycentric weight of the second endpoint. */
function nearestOnSegment(p: Point2, q: Point2): { x: number; y: number; t: number } {
  const dx = q.x - p.x;
  const dy = q.y - p.y;
  const len2 = dx * dx + dy * dy;
  let t = len2 > 1e-24 ? -(p.x * dx + p.y * dy) / len2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return { x: p.x + t * dx, y: p.y + t * dy, t };
}

/**
 * Distance and witnesses between two convex polygons by GJK. `maxIter`
 * bounds the loop; in exact arithmetic the simplex never revisits a vertex
 * and the iteration count is at most the number of vertices of Z.
 */
export function gjkDistance(A: Polygon, B: Polygon, maxIter = 64, tol = 1e-12): GjkResult {
  const simplices: Point2[][] = [];
  // Initial direction: from B's first vertex toward A's — any nonzero direction works.
  let d0x = A[0].x - B[0].x;
  let d0y = A[0].y - B[0].y;
  if (Math.abs(d0x) + Math.abs(d0y) < 1e-15) {
    d0x = 1;
    d0y = 0;
  }
  let simplex: ZPoint[] = [supportDiff(A, B, d0x, d0y)];
  let iterations = 0;

  // The nearest point x_k of the current simplex and its barycentric weights.
  const nearest = (): { x: number; y: number; w: number[]; inside: boolean } => {
    if (simplex.length === 1) return { x: simplex[0].z.x, y: simplex[0].z.y, w: [1], inside: false };
    if (simplex.length === 2) {
      const n = nearestOnSegment(simplex[0].z, simplex[1].z);
      return { x: n.x, y: n.y, w: [1 - n.t, n.t], inside: false };
    }
    // Three points: is the origin inside the triangle?
    const [p, q, r] = simplex.map((s) => s.z);
    const cross = (u: Point2, v: Point2, w: Point2) => (v.x - u.x) * (w.y - u.y) - (v.y - u.y) * (w.x - u.x);
    const o = { x: 0, y: 0 };
    const c1 = cross(p, q, o);
    const c2 = cross(q, r, o);
    const c3 = cross(r, p, o);
    const area = cross(p, q, r);
    if (Math.abs(area) > 1e-18 && ((c1 >= -tol && c2 >= -tol && c3 >= -tol) || (c1 <= tol && c2 <= tol && c3 <= tol))) {
      // Barycentric weights of the origin.
      return { x: 0, y: 0, w: [c2 / area, c3 / area, c1 / area], inside: true };
    }
    // Otherwise x_k lies on one edge; keep that edge (Choset's step 6).
    let best: { x: number; y: number; w: number[]; keep: [number, number] } | null = null;
    const edges: [number, number][] = [
      [0, 1],
      [1, 2],
      [2, 0],
    ];
    for (const [i, j] of edges) {
      const n = nearestOnSegment(simplex[i].z, simplex[j].z);
      const dd = n.x * n.x + n.y * n.y;
      if (!best || dd < best.x * best.x + best.y * best.y) best = { x: n.x, y: n.y, w: [1 - n.t, n.t], keep: [i, j] };
    }
    const kept = best!;
    simplex = [simplex[kept.keep[0]], simplex[kept.keep[1]]];
    return { x: kept.x, y: kept.y, w: kept.w, inside: false };
  };

  const witnesses = (w: number[]) => {
    let ax = 0, ay = 0, bx = 0, by = 0;
    for (let k = 0; k < simplex.length; k++) {
      ax += w[k] * simplex[k].a.x;
      ay += w[k] * simplex[k].a.y;
      bx += w[k] * simplex[k].b.x;
      by += w[k] * simplex[k].b.y;
    }
    return { onA: { x: ax, y: ay }, onB: { x: bx, y: by } };
  };

  for (;;) {
    iterations++;
    simplices.push(simplex.map((s) => s.z));
    const xk = nearest();
    if (xk.inside) {
      const w = witnesses(xk.w);
      return { dist: 0, ...w, iterations, simplices, intersecting: true };
    }
    const norm2 = xk.x * xk.x + xk.y * xk.y;
    if (norm2 < tol * tol) {
      const w = witnesses(xk.w);
      return { dist: 0, ...w, iterations, simplices, intersecting: true };
    }
    // Step 4–5: the support point of Z in direction −x_k. If it makes no
    // progress toward the origin — z·(−x_k) ≤ −‖x_k‖², i.e. h_Z(−x_k) = −‖x_k‖²
    // — then x_k is already the nearest point of Z and we stop.
    const zk = supportDiff(A, B, -xk.x, -xk.y);
    const progress = norm2 - (zk.z.x * xk.x + zk.z.y * xk.y);
    if (progress <= tol * Math.max(1, norm2) || iterations >= maxIter) {
      const w = witnesses(xk.w);
      return { dist: Math.sqrt(norm2), ...w, iterations, simplices, intersecting: false };
    }
    // Step 6: a degenerate repeat means we are done up to rounding.
    if (simplex.some((s) => Math.abs(s.z.x - zk.z.x) < 1e-15 && Math.abs(s.z.y - zk.z.y) < 1e-15)) {
      const w = witnesses(xk.w);
      return { dist: Math.sqrt(norm2), ...w, iterations, simplices, intersecting: false };
    }
    simplex.push(zk);
  }
}

/** A segment as a two-vertex "polygon" — GJK needs only a support function, so links work too. */
export function segmentPolygon(x1: number, y1: number, x2: number, y2: number): Polygon {
  return [
    { x: x1, y: y1 },
    { x: x2, y: y2 },
  ];
}
