/**
 * Range-sensor geometry (Choset §2.2, §2.3.1–2.3.2) — an artifact Chapters 7–10
 * import.
 *
 *  - `rangeScan`      the saturated raw distance function ρ_R(x, ·) on n rays,
 *                     built on the ported `rayCast`; ∞ where ρ ≥ R.
 *  - `distanceFromScan` / `gradientFromScan` / `tangentFromScan`
 *                     D(x) = min_s ρ(x, s) and ∇D as the minimizing ray
 *                     (eq. 2.4), from the scan alone — no world access.
 *  - `continuityIntervals` / `intervalEndpoints`
 *                     the intervals of continuity of ρ_R and their endpoints
 *                     O_i, refined by angular bisection so a tangent endpoint
 *                     lands on the obstacle's silhouette to ~1e-9.
 *
 * Everything here is "sensor-only": it takes a scan, or the world plus a pose
 * to *make* a scan, and never looks at polygons. That is the point of §2.3 —
 * what a robot can infer from range data — and it is what the Tangent Bug and
 * the sensor-based roadmaps of Chapter 9 run on.
 */

import { rayCast, type Point2, type World } from '../sim/world';
import { closestOnSegment, dist, perp, pt, segmentHit, type Turn } from './geometry';

export interface Scan {
  /** Where the scan was taken. */
  x: Point2;
  /** Ray bearings s_k = 2πk/n, world-frame. */
  angles: Float64Array;
  /** ρ_R(x, s_k): the raw distance, or +∞ when the ray saturates at R. */
  ranges: Float64Array;
  /** The sensing radius R. */
  maxRange: number;
}

/** The world point a ray ends at (only meaningful for finite rays). */
export function scanPoint(scan: Scan, k: number): Point2 {
  const r = scan.ranges[k];
  return pt(scan.x.x + r * Math.cos(scan.angles[k]), scan.x.y + r * Math.sin(scan.angles[k]));
}

/**
 * ρ_R(x, ·) on `nRays` evenly spaced bearings. A finite reading is the exact
 * distance along the ray to the first wall; a ray that reaches R without a hit
 * reads ∞ — Choset's saturated raw distance function, literally.
 */
export function rangeScan(world: World, x: number, y: number, nRays: number, maxRange: number): Scan {
  const angles = new Float64Array(nRays);
  const ranges = new Float64Array(nRays);
  for (let k = 0; k < nRays; k++) {
    const a = (2 * Math.PI * k) / nRays;
    angles[k] = a;
    ranges[k] = rho(world, x, y, a, maxRange);
  }
  return { x: pt(x, y), angles, ranges, maxRange };
}

/** One saturated ray: ρ(x, θ) if it is below R, otherwise ∞. */
export function rho(world: World, x: number, y: number, theta: number, maxRange: number): number {
  const r = rayCast(world, x, y, theta, maxRange);
  return r < maxRange ? r : Infinity;
}

// ---------------------------------------------------------------------------
// Distance and gradient from the scan (Choset eq. 2.4, problem 2.1)
// ---------------------------------------------------------------------------

/** Index of the shortest ray, or -1 when every ray saturates. */
export function argminRay(scan: Scan): number {
  let best = -1;
  let bestR = Infinity;
  for (let k = 0; k < scan.ranges.length; k++) {
    if (scan.ranges[k] < bestR) {
      bestR = scan.ranges[k];
      best = k;
    }
  }
  return best;
}

/**
 * D(x) = min_s ρ(x, s). With finitely many rays the minimum is attained on a
 * grid of bearings, so this overestimates D by at most D·(1 − cos(π/n)) for a
 * flat wall and by O(D·2π/n) near a corner. `refine` closes most of that gap
 * by golden-section search on ρ(x, ·) between the two rays flanking the
 * minimum — still sensor-only: it is simply more rays where they matter.
 */
export function distanceFromScan(scan: Scan, refine?: { world: World; iters?: number }): number {
  const k = argminRay(scan);
  if (k < 0) return Infinity;
  if (!refine) return scan.ranges[k];
  return refineMinimum(scan, k, refine.world, refine.iters ?? 40).r;
}

/**
 * ∇D(x): the unit vector along the minimizing ray, pointing *away* from the
 * obstacle, i.e. −(cos s*, sin s*). Undefined (zero vector) when nothing is in
 * range.
 */
export function gradientFromScan(scan: Scan, refine?: { world: World; iters?: number }): Point2 {
  const k = argminRay(scan);
  if (k < 0) return pt(0, 0);
  const s = refine ? refineMinimum(scan, k, refine.world, refine.iters ?? 40).theta : scan.angles[k];
  return pt(-Math.cos(s), -Math.sin(s));
}

/**
 * The tangent to the offset curve, (∇D)^⊥, with its sign fixed by the side the
 * obstacle is kept on: `left` keeps the obstacle on the robot's right (so the
 * robot circles it clockwise), `right` the reverse. Choset §2.3.1 fixes the
 * sign "by the previous direction"; fixing it by side is equivalent along a
 * smooth curve and survives the corners where the previous direction is
 * useless.
 */
export function tangentFromScan(scan: Scan, turn: Turn, refine?: { world: World }): Point2 {
  return tangentOf(gradientFromScan(scan, refine), turn);
}

/** (∇D)^⊥ with the sign that keeps the obstacle on the side `turn` implies. */
export function tangentOf(grad: Point2, turn: Turn): Point2 {
  const t = perp(grad);
  return turn === 'left' ? pt(-t.x, -t.y) : t;
}

function refineMinimum(scan: Scan, k: number, world: World, iters: number): { r: number; theta: number } {
  const n = scan.ranges.length;
  const step = (2 * Math.PI) / n;
  let a = scan.angles[k] - step;
  let b = scan.angles[k] + step;
  const f = (t: number) => rayCast(world, scan.x.x, scan.x.y, t, scan.maxRange);
  const g = (Math.sqrt(5) - 1) / 2;
  let c = b - g * (b - a);
  let d = a + g * (b - a);
  let fc = f(c);
  let fd = f(d);
  for (let i = 0; i < iters; i++) {
    if (fc < fd) {
      b = d;
      d = c;
      fd = fc;
      c = b - g * (b - a);
      fc = f(c);
    } else {
      a = c;
      c = d;
      fc = fd;
      d = a + g * (b - a);
      fd = f(d);
    }
  }
  const theta = (a + b) / 2;
  return { r: Math.min(f(theta), scan.ranges[k]), theta };
}

// ---------------------------------------------------------------------------
// Intervals of continuity and their endpoints O_i (Choset §2.2, figure 2.6)
// ---------------------------------------------------------------------------

export interface Interval {
  /** First and last ray index (inclusive); `to < from` means the run wraps past ray 0. */
  from: number;
  to: number;
  /** Number of rays in the run. */
  count: number;
}

/** Ray index k + d modulo n. */
const wrap = (k: number, d: number, n: number) => (((k + d) % n) + n) % n;

/**
 * Are two consecutive finite readings part of one continuous piece of
 * boundary? On a flat wall at incidence φ the endpoint spacing is
 * ρ·Δθ / cos φ, so a jump larger than `kappa` times ρ·Δθ (κ = 8 tolerates
 * incidence up to ~83°) or larger than `jumpAbs` is a discontinuity: either an
 * occlusion (the ray is tangent to a nearer obstacle) or the range limit.
 */
function continuous(scan: Scan, k: number, l: number, kappa: number, jumpAbs: number, world?: World): boolean {
  const rk = scan.ranges[k];
  const rl = scan.ranges[l];
  if (!Number.isFinite(rk) || !Number.isFinite(rl)) return false;
  const dtheta = (2 * Math.PI) / scan.ranges.length;
  const a = scanPoint(scan, k);
  const b = scanPoint(scan, l);
  const gap = dist(a, b);
  if (gap <= Math.max(jumpAbs, kappa * Math.min(rk, rl) * dtheta)) return true;
  if (!world) return false;
  // A wall seen nearly edge-on spreads consecutive hits far apart yet is one
  // continuous surface. One extra ray settles it: if the mid-bearing hit lies
  // on the chord between the two, the surface is flat between them.
  return midRayOnChord(scan, k, l, a, b, world);
}

function midRayOnChord(scan: Scan, k: number, l: number, a: Point2, b: Point2, world: World): boolean {
  const n = scan.ranges.length;
  const step = (2 * Math.PI) / n;
  const mid = scan.angles[k] + (l === (k + 1) % n ? step / 2 : -step / 2);
  const r = rayCast(world, scan.x.x, scan.x.y, mid, scan.maxRange);
  if (r >= scan.maxRange) return false;
  const m = pt(scan.x.x + r * Math.cos(mid), scan.x.y + r * Math.sin(mid));
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len = Math.hypot(dx, dy);
  if (len < 1e-12) return true;
  const off = Math.abs((m.x - a.x) * dy - (m.y - a.y) * dx) / len;
  const t = ((m.x - a.x) * dx + (m.y - a.y) * dy) / (len * len);
  return off <= 1e-7 * Math.max(1, len) && t >= -1e-9 && t <= 1 + 1e-9;
}

export interface IntervalOptions {
  /** Relative jump factor; see `continuous`. */
  kappa?: number;
  /** Absolute jump floor, in metres. */
  jumpAbs?: number;
  /** When given, a mid-bearing ray disambiguates walls seen nearly edge-on. */
  world?: World;
}

/**
 * Maximal runs of rays over which ρ_R is finite and continuous. Runs that
 * straddle the 0/2π seam are merged into one wrapped interval. An all-finite,
 * all-continuous scan (the robot inside a closed room) is one interval with
 * no endpoints.
 */
export function continuityIntervals(scan: Scan, opts: IntervalOptions = {}): Interval[] {
  const n = scan.ranges.length;
  const kappa = opts.kappa ?? 8;
  const jumpAbs = opts.jumpAbs ?? 0.1;
  const world = opts.world;
  // Find a break to start from so wrapped runs come out whole.
  let start = -1;
  for (let k = 0; k < n; k++) {
    if (!continuous(scan, wrap(k, -1, n), k, kappa, jumpAbs, world)) {
      start = k;
      break;
    }
  }
  if (start < 0) {
    return Number.isFinite(scan.ranges[0]) ? [{ from: 0, to: n - 1, count: n }] : [];
  }
  const out: Interval[] = [];
  let k = start;
  let visited = 0;
  while (visited < n) {
    if (!Number.isFinite(scan.ranges[k])) {
      k = wrap(k, 1, n);
      visited++;
      continue;
    }
    const from = k;
    let count = 1;
    while (visited + count < n && continuous(scan, wrap(k, count - 1, n), wrap(k, count, n), kappa, jumpAbs, world)) count++;
    out.push({ from, to: wrap(from, count - 1, n), count });
    k = wrap(k, count, n);
    visited += count;
  }
  return out;
}

export interface Endpoint {
  /** The O_i itself: a point on the free-space boundary. */
  point: Point2;
  /** Interval the endpoint belongs to and which end it is. */
  interval: number;
  end: 'from' | 'to';
  /** Why ρ_R lost continuity here. */
  kind: 'occlusion' | 'range';
  /** Bearing of the discontinuity. */
  theta: number;
}

/**
 * The endpoints O_i of every interval. Where an interval ends against a
 * discontinuity the exact bearing is found by bisection between the last ray
 * inside and the first ray outside (40 halvings: 2π/n → ~1e-12 rad), so a
 * tangent endpoint is the silhouette point itself — on a polygon world, its
 * vertex — rather than "the last ray that happened to hit". That refinement is
 * what the tangent-line check compares against brute force.
 */
export function intervalEndpoints(
  scan: Scan,
  intervals: Interval[],
  world: World,
  opts: IntervalOptions & { iters?: number } = {},
): Endpoint[] {
  const n = scan.ranges.length;
  const kappa = opts.kappa ?? 8;
  const jumpAbs = opts.jumpAbs ?? 0.1;
  const iters = opts.iters ?? 40;
  const out: Endpoint[] = [];
  const dtheta = (2 * Math.PI) / n;
  const rhoAt = (t: number) => rho(world, scan.x.x, scan.x.y, t, scan.maxRange);
  const endpointAt = (t: number, r: number): Point2 => pt(scan.x.x + r * Math.cos(t), scan.x.y + r * Math.sin(t));

  /**
   * March the bearing from the last ray inside the interval toward the first
   * ray outside, accepting a move while the hit stays continuous with what came
   * before: a small step on a smooth surface (the κ test), a vanishing step
   * (which carries us from one edge onto the next at a corner), or a point
   * collinear with the last two — a flat wall seen edge-on, where consecutive
   * hits race apart. The step doubles after each acceptance and halves after
   * each rejection, so the march converges on the discontinuity to ~1e-12 rad.
   */
  const refine = (inside: number, dir: 1 | -1, count: number): { theta: number; r: number; kind: 'occlusion' | 'range' } => {
    let theta = scan.angles[inside];
    let r = scan.ranges[inside];
    let p = endpointAt(theta, r);
    const limit = theta + dir * dtheta;
    const outside = wrap(inside, dir, n);
    const kind: 'occlusion' | 'range' = Number.isFinite(scan.ranges[outside]) ? 'occlusion' : 'range';
    const innerIdx = wrap(inside, -dir, n);
    let pPrev: Point2 | null = count >= 2 && Number.isFinite(scan.ranges[innerIdx]) ? scanPoint(scan, innerIdx) : null;
    let delta = dtheta / 2;
    for (let i = 0; i < 4 * iters && delta > 1e-13; i++) {
      const t = theta + dir * delta;
      if (dir > 0 ? t > limit : t < limit) {
        delta /= 2;
        continue;
      }
      const rt = rhoAt(t);
      if (!Number.isFinite(rt)) {
        delta /= 2;
        continue;
      }
      const q = endpointAt(t, rt);
      const move = dist(q, p);
      let ok = move <= Math.max(jumpAbs, kappa * Math.min(r, rt) * delta) || move <= 1e-7;
      if (!ok && pPrev) {
        const ux = p.x - pPrev.x;
        const uy = p.y - pPrev.y;
        const len = Math.hypot(ux, uy);
        if (len > 1e-12) {
          const off = Math.abs((q.x - pPrev.x) * uy - (q.y - pPrev.y) * ux) / len;
          const ahead = ((q.x - p.x) * ux + (q.y - p.y) * uy) / len;
          ok = off <= 1e-7 * Math.max(1, len) && ahead >= -1e-9;
        }
      }
      if (ok) {
        pPrev = p;
        p = q;
        theta = t;
        r = rt;
        delta = Math.min(delta * 2, dtheta / 2);
      } else {
        delta /= 2;
      }
    }
    return { theta, r, kind };
  };

  intervals.forEach((iv, i) => {
    if (iv.count === n) return; // a closed room: no discontinuity anywhere
    const a = refine(iv.from, -1, iv.count);
    out.push({ point: endpointAt(a.theta, a.r), interval: i, end: 'from', kind: a.kind, theta: a.theta });
    const b = refine(iv.to, 1, iv.count);
    out.push({ point: endpointAt(b.theta, b.r), interval: i, end: 'to', kind: b.kind, theta: b.theta });
  });
  return out;
}

/**
 * Brute-force silhouette points for a polygon world, used only to *test*
 * `intervalEndpoints`. A visible vertex is a silhouette point when both of its
 * neighbours lie on the same side of the ray through it — the ray is tangent
 * to the polygon there. The far side's hit (if finite) is the matching
 * occlusion endpoint on whatever lies behind. Returns null when some vertex
 * has an edge within `degenerate` radians of radial, where the notion of
 * "tangent" is numerically ill-posed for any method.
 */
export function exactSilhouettePoints(
  world: World,
  polygons: readonly { pts: readonly Point2[] }[],
  x: Point2,
  maxRange: number,
  degenerate = 1e-4,
): Point2[] | null {
  const out: Point2[] = [];
  const eps = 1e-9;
  for (const poly of polygons) {
    const m = poly.pts.length;
    for (let i = 0; i < m; i++) {
      const v = poly.pts[i];
      const d = dist(v, x);
      if (d >= maxRange) continue;
      const dir = pt((v.x - x.x) / d, (v.y - x.y) / d);
      const prev = poly.pts[(i + m - 1) % m];
      const next = poly.pts[(i + 1) % m];
      const sideOf = (q: Point2) => {
        const e = pt(q.x - v.x, q.y - v.y);
        const len = Math.hypot(e.x, e.y);
        return (dir.x * e.y - dir.y * e.x) / len; // sin of the angle from the ray to the edge
      };
      const sp = sideOf(prev);
      const sn = sideOf(next);
      if (Math.abs(sp) < degenerate || Math.abs(sn) < degenerate) return null;
      if (sp > 0 !== sn > 0) continue; // the ray passes through the polygon here: not tangent
      // Visible? No wall that does not itself touch v may cross the segment x→v.
      const blocked = world.walls.some((w) => {
        const touchesV = closestOnSegment(v, { x: w.x1, y: w.y1 }, { x: w.x2, y: w.y2 }).d < 1e-9;
        return !touchesV && segmentHit(x, v, w) !== null;
      });
      if (blocked) continue;
      out.push(v);
      // Free side: the side opposite to both neighbours.
      const theta = Math.atan2(dir.y, dir.x) + (sp > 0 ? -eps : eps);
      const far = rho(world, x.x, x.y, theta, maxRange);
      if (Number.isFinite(far) && far > d + 1e-7) out.push(pt(x.x + far * Math.cos(theta), x.y + far * Math.sin(theta)));
    }
  }
  return out;
}

/**
 * Tangent Bug's point T: where the sensing circle meets the segment x→goal, or
 * the goal itself when it is within range (Choset p. 28).
 */
export function pointT(x: Point2, goal: Point2, maxRange: number): Point2 {
  const d = dist(x, goal);
  if (d <= maxRange) return goal;
  return pt(x.x + ((goal.x - x.x) * maxRange) / d, x.y + ((goal.y - x.y) * maxRange) / d);
}
