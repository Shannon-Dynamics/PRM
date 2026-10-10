/**
 * Accessibility by gradient ascent — Choset eq. (5.2), Lemma 5.2.1 — and the
 * deformation retraction it implicitly defines (§5.2.2–5.2.3).
 *
 *     dc/dt = ∇D(c(t)),  c(0) = q_start,
 *
 * followed until two obstacles tie. With QO_i the unique closest obstacle,
 * D = d_i nearby and the flow is ∇d_i; d_i grows, the environment is bounded,
 * so some d_j must stop exceeding it; continuity gives the tie. The map
 * RM(q) = "where the ascent from q lands" is the retraction, and
 * H(q, t) = c_q(t) the homotopy from the identity to it — the melting candy.
 *
 * Two refinements the port needs and the prose keeps:
 *   - SS_ij: a tie with ∇d_i = ∇d_j (two collinear wall pieces sharing an
 *     endpoint, seen from beyond the endpoint) is *not* on the GVD. Such ties
 *     are skipped and the ascent continues on the new nearest obstacle.
 *   - The crossing is detected as a change of nearest obstacle between two
 *     steps and then bisected to 10⁻¹², so the landing point is on S_ij to
 *     machine precision rather than within a step of it.
 *
 * The distance query is Chapter 7's `DistanceQuery` (`PlanarObstacles` for
 * geometry); Chapter 9 adds the range-scan version.
 */

import type { DistanceQuery, ObstacleDistance, Vec } from '../potential/field';
import type { Bounds, Point2 } from '../sim/world';

export interface RetractOptions {
  step?: number;
  maxSteps?: number;
  bounds?: Bounds;
  /** Minimum angle between ∇d_i and ∇d_j for a tie to count as SS_ij (radians). */
  gradTol?: number;
}

export interface RetractResult {
  /** The ascent path c(t), from q to the landing point (last). */
  path: Point2[];
  landed: boolean;
  /** The two tied obstacles at the landing point. */
  pair: [number, number] | null;
  /** D at the landing point. */
  clearance: number;
  /** |d_i − d_j| at the landing point — zero to tolerance when landed. */
  residual: number;
  /** Why the ascent stopped. */
  stop: 'gvd' | 'bounds' | 'budget' | 'noObstacle';
}

const two = (ds: ObstacleDistance[]) => ds.slice().sort((a, b) => a.d - b.d);

function angleBetween(a: Vec, b: Vec): number {
  const c = Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1]));
  return Math.acos(c);
}

/**
 * Gradient ascent of D from q until the GVD (SS_ij) is reached. Returns the
 * path and the landing point; `landed` is false if the window was left (an
 * unbounded environment, which Lemma 5.2.1 excludes) or the budget ran out.
 */
export function retract(dist: DistanceQuery, q0: Point2, opts: RetractOptions = {}): RetractResult {
  const { step = 0.02, maxSteps = 5000, bounds, gradTol = 1e-3 } = opts;
  const path: Point2[] = [{ x: q0.x, y: q0.y }];
  const inB = (p: Point2) => !bounds || (p.x >= bounds.minX && p.x <= bounds.maxX && p.y >= bounds.minY && p.y <= bounds.maxY);
  const query = (p: Point2) => two(dist.distances([p.x, p.y]));

  let q = path[0];
  let ds = query(q);
  if (ds.length === 0) return { path, landed: false, pair: null, clearance: Infinity, residual: Infinity, stop: 'noObstacle' };

  const tied = (d: ObstacleDistance[], tol: number) => d.length >= 2 && d[1].d - d[0].d <= tol && angleBetween(d[0].grad, d[1].grad) > gradTol;

  if (tied(ds, 1e-12)) {
    return { path, landed: true, pair: [ds[0].index, ds[1].index], clearance: ds[0].d, residual: ds[1].d - ds[0].d, stop: 'gvd' };
  }

  for (let it = 0; it < maxSteps; it++) {
    const i = ds[0].index;
    const g = ds[0].grad;
    const next = { x: q.x + step * g[0], y: q.y + step * g[1] };
    if (!inB(next)) {
      path.push(next);
      return { path, landed: false, pair: null, clearance: ds[0].d, residual: Infinity, stop: 'bounds' };
    }
    const dsNext = query(next);
    const j = dsNext[0].index;
    if (j !== i) {
      // The nearest obstacle changed between q and next: S_ij was crossed. Is it SS_ij?
      const di = (p: Point2) => dist.distances([p.x, p.y]).find((o) => o.index === i)!;
      const dj = (p: Point2) => dist.distances([p.x, p.y]).find((o) => o.index === j)!;
      const gq = di(q).grad;
      const gj = dj(q).grad;
      if (angleBetween(gq, gj) > gradTol) {
        // Bisect f(t) = d_i − d_j on [q, next]: f(0) < 0 ≤ f(1).
        let lo = 0;
        let hi = 1;
        const at = (t: number): Point2 => ({ x: q.x + t * (next.x - q.x), y: q.y + t * (next.y - q.y) });
        for (let b = 0; b < 60; b++) {
          const mid = 0.5 * (lo + hi);
          const p = at(mid);
          if (di(p).d - dj(p).d < 0) lo = mid;
          else hi = mid;
        }
        const land = at(0.5 * (lo + hi));
        const dl = two(dist.distances([land.x, land.y]));
        path.push(land);
        return { path, landed: true, pair: [i, j], clearance: dl[0].d, residual: Math.abs(di(land).d - dj(land).d), stop: 'gvd' };
      }
      // Non-surjective tie (∇d_i = ∇d_j): not the GVD. Continue with the new nearest obstacle.
    }
    q = next;
    ds = dsNext;
    path.push(q);
    if (tied(ds, 1e-12)) return { path, landed: true, pair: [ds[0].index, ds[1].index], clearance: ds[0].d, residual: ds[1].d - ds[0].d, stop: 'gvd' };
  }
  return { path, landed: false, pair: null, clearance: ds[0].d, residual: Infinity, stop: 'budget' };
}

/** Arc-length parametrisation: the point at fraction t ∈ [0, 1] along a path — H(q, t). */
export function alongPath(path: readonly Point2[], t: number): Point2 {
  if (path.length === 0) return { x: 0, y: 0 };
  if (path.length === 1 || t <= 0) return path[0];
  if (t >= 1) return path[path.length - 1];
  const cum: number[] = [0];
  for (let i = 1; i < path.length; i++) cum.push(cum[i - 1] + Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y));
  const L = cum[cum.length - 1];
  if (L === 0) return path[0];
  const s = t * L;
  let k = 1;
  while (k < cum.length && cum[k] < s) k++;
  const a = path[k - 1];
  const b = path[k];
  const seg = cum[k] - cum[k - 1] || 1;
  const u = (s - cum[k - 1]) / seg;
  return { x: a.x + u * (b.x - a.x), y: a.y + u * (b.y - a.y) };
}

/**
 * The deformation retraction H : Q_free × [0, 1] → Q_free of §5.2.3 for a set
 * of points: H(q, 0) = q, H(q, 1) = RM(q), H(a, t) = a on the GVD. Each point
 * carries its own ascent path, so the whole free space can be "melted" by
 * scrubbing t — the Retract Candy widget.
 */
export function retractionHomotopy(dist: DistanceQuery, points: readonly Point2[], opts: RetractOptions = {}): { paths: Point2[][]; results: RetractResult[]; at: (t: number) => Point2[] } {
  const results = points.map((p) => retract(dist, p, opts));
  const paths = results.map((r) => r.path);
  return { paths, results, at: (t) => paths.map((p) => alongPath(p, t)) };
}

/**
 * A retraction that is *not* a deformation retraction: everything to one
 * point along straight lines. It is continuous and fixes the point, so it is a
 * retraction onto {a}; but a loop around a hole cannot follow it without
 * crossing the hole — the straight lines pierce obstacles. `pierces` counts
 * how many of the straight paths leave the free space, which is the widget's
 * evidence.
 */
export function retractToPoint(points: readonly Point2[], a: Point2, isFree: (p: Point2) => boolean, samples = 24): { at: (t: number) => Point2[]; pierces: number } {
  let pierces = 0;
  for (const p of points) {
    for (let k = 1; k < samples; k++) {
      const t = k / samples;
      if (!isFree({ x: p.x + t * (a.x - p.x), y: p.y + t * (a.y - p.y) })) {
        pierces++;
        break;
      }
    }
  }
  return { at: (t) => points.map((p) => ({ x: p.x + t * (a.x - p.x), y: p.y + t * (a.y - p.y) })), pierces };
}

/**
 * Winding number of a closed polyline about a point — the loop-class
 * bookkeeping of the Retract Candy. A loop whose retraction keeps a nonzero
 * winding number about an obstacle still encloses it: the hole survived.
 */
export function windingNumber(loop: readonly Point2[], about: Point2): number {
  let total = 0;
  for (let i = 0; i < loop.length; i++) {
    const a = loop[i];
    const b = loop[(i + 1) % loop.length];
    const a1 = Math.atan2(a.y - about.y, a.x - about.x);
    const a2 = Math.atan2(b.y - about.y, b.x - about.x);
    let d = a2 - a1;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    total += d;
  }
  return Math.round(total / (2 * Math.PI));
}
