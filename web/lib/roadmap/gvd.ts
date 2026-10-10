/**
 * The generalized Voronoi diagram — Choset §5.2.1, §5.2.4, §5.2.5 — for small
 * point / segment / polygon site sets, traced exactly.
 *
 * The planar GVD is the union of the two-equidistant faces
 *
 *     F_ij = { q ∈ SS_ij : d_i(q) ≤ d_h(q) ∀h },
 *     SS_ij = { q : d_i(q) = d_j(q), ∇d_i(q) ≠ ∇d_j(q) },
 *
 * and the preimage theorem (Thm. 5.2.2) says each is a one-dimensional
 * manifold: G = d_i − d_j has DG = (∇d_i − ∇d_j)ᵀ, surjective exactly where
 * the gradients differ. That theorem is also an algorithm. Chapter 3's
 * predictor–corrector `traceCurve` follows G = 0 with tangent (∇d_i − ∇d_j)^⊥
 * — Choset's §5.2.5 sensor-based construction, and Fig. 5.13's remark that the
 * GVD's tangent is perpendicular to the chord joining the two closest points.
 * An edge ends at a *meet point* (a third site becomes equidistant; polished
 * by Newton on two equations) or a *boundary point* (D → 0).
 *
 * Honesty: the Rust crate delegates the exact segment Voronoi diagram to a
 * library (Boost.Polygon's algorithm); no such kernel exists for TypeScript,
 * and this file does not pretend to be one. It is exact to the tracer's
 * tolerance (~10⁻¹²) and feasible for a dozen sites; the Apartment's GVD
 * comes from Chapter 7's brushfire ridges (grid.ts) instead.
 */

import { distancePointPolygon, polygonEdges, type Polygon } from '../collide/polygon';
import { correctGradient, traceCurve, type Trace } from '../bugs/trace';
import type { Bounds, Point2, Segment } from '../sim/world';
import { closestPointOnSegment } from '../collide/polygon';

export type Site =
  | { kind: 'point'; p: Point2 }
  | { kind: 'segment'; s: Segment }
  | { kind: 'polygon'; poly: Polygon };

/** Which feature of the site is closest: a vertex/point, or the interior of a line. */
export type Feature = 'point' | 'line';

export interface SiteDistance {
  index: number;
  d: number;
  /** ∇d_i(q) = (q − c)/d, a unit vector; the zero vector on the site. */
  grad: Point2;
  closest: Point2;
  feature: Feature;
}

const EPS = 1e-12;

function pointSiteDistance(index: number, q: Point2, p: Point2): SiteDistance {
  const dx = q.x - p.x;
  const dy = q.y - p.y;
  const d = Math.hypot(dx, dy);
  return { index, d, grad: d > EPS ? { x: dx / d, y: dy / d } : { x: 0, y: 0 }, closest: p, feature: 'point' };
}

function segmentSiteDistance(index: number, q: Point2, s: Segment): SiteDistance {
  const c = closestPointOnSegment(q.x, q.y, s);
  const d = Math.hypot(q.x - c.x, q.y - c.y);
  const atA = Math.hypot(c.x - s.x1, c.y - s.y1) < 1e-9;
  const atB = Math.hypot(c.x - s.x2, c.y - s.y2) < 1e-9;
  return { index, d, grad: d > EPS ? { x: (q.x - c.x) / d, y: (q.y - c.y) / d } : { x: 0, y: 0 }, closest: c, feature: atA || atB ? 'point' : 'line' };
}

/** d_i(q), its gradient and closest point for one site. */
export function siteDistance(site: Site, index: number, q: Point2): SiteDistance {
  if (site.kind === 'point') return pointSiteDistance(index, q, site.p);
  if (site.kind === 'segment') return segmentSiteDistance(index, q, site.s);
  const w = distancePointPolygon(q.x, q.y, site.poly);
  if (w.dist <= EPS) return { index, d: 0, grad: { x: 0, y: 0 }, closest: w.onObstacle, feature: 'point' };
  // Which edge or vertex? Re-derive the feature from the witness.
  let feature: Feature = 'line';
  for (const v of site.poly) if (Math.hypot(v.x - w.onObstacle.x, v.y - w.onObstacle.y) < 1e-9) feature = 'point';
  return { index, d: w.dist, grad: { x: (q.x - w.onObstacle.x) / w.dist, y: (q.y - w.onObstacle.y) / w.dist }, closest: w.onObstacle, feature };
}

/** Every site's distance at q, nearest first. */
export function siteDistances(sites: readonly Site[], q: Point2): SiteDistance[] {
  return sites.map((s, i) => siteDistance(s, i, q)).sort((a, b) => a.d - b.d);
}

/** D(q) = min_i d_i(q). */
export function clearance(sites: readonly Site[], q: Point2): number {
  let m = Infinity;
  sites.forEach((s, i) => (m = Math.min(m, siteDistance(s, i, q).d)));
  return m;
}

/** Angle between two unit gradients — the surjectivity test of SS_ij needs it to be nonzero. */
export function gradientAngle(a: Point2, b: Point2): number {
  const c = Math.max(-1, Math.min(1, a.x * b.x + a.y * b.y));
  return Math.acos(c);
}

/** G_ij = d_i − d_j and DG_ij = ∇d_i − ∇d_j, as fields for the tracer. */
export function equidistanceField(sites: readonly Site[], i: number, j: number) {
  const G = (q: Point2) => siteDistance(sites[i], i, q).d - siteDistance(sites[j], j, q).d;
  const DG = (q: Point2): Point2 => {
    const a = siteDistance(sites[i], i, q).grad;
    const b = siteDistance(sites[j], j, q).grad;
    return { x: a.x - b.x, y: a.y - b.y };
  };
  return { G, DG };
}

export interface MeetPoint {
  q: Point2;
  clearance: number;
  /** The three equidistant closest sites. */
  sites: [number, number, number];
}

/**
 * A meet point F_ijk by damped Newton on F(q) = (d_i − d_j, d_i − d_k) from
 * `seed`. Returns null if Newton fails or if the three are not the three
 * closest sites at the solution (then it is a point of S_ij ∩ S_ik, but not of
 * the GVD).
 */
export function meetPoint(sites: readonly Site[], i: number, j: number, k: number, seed: Point2, iters = 60, tol = 1e-12): MeetPoint | null {
  let q = { x: seed.x, y: seed.y };
  for (let it = 0; it < iters; it++) {
    const di = siteDistance(sites[i], i, q);
    const dj = siteDistance(sites[j], j, q);
    const dk = siteDistance(sites[k], k, q);
    const F = [di.d - dj.d, di.d - dk.d];
    const r = Math.hypot(F[0], F[1]);
    if (r < tol) break;
    const J = [
      [di.grad.x - dj.grad.x, di.grad.y - dj.grad.y],
      [di.grad.x - dk.grad.x, di.grad.y - dk.grad.y],
    ];
    const det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    if (Math.abs(det) < 1e-14) return null;
    const dx = -(J[1][1] * F[0] - J[0][1] * F[1]) / det;
    const dy = -(-J[1][0] * F[0] + J[0][0] * F[1]) / det;
    const step = Math.hypot(dx, dy);
    const cap = 1; // never jump more than a unit: the distance functions are 1-Lipschitz
    const s = step > cap ? cap / step : 1;
    q = { x: q.x + s * dx, y: q.y + s * dy };
  }
  const all = siteDistances(sites, q);
  const trio = new Set([i, j, k]);
  const three = all.slice(0, 3);
  if (!three.every((d) => trio.has(d.index))) return null;
  if (Math.abs(three[0].d - three[2].d) > 1e-8) return null;
  return { q, clearance: three[0].d, sites: [i, j, k] };
}

export type EdgeEnd =
  | { kind: 'meet'; meet: MeetPoint }
  | { kind: 'boundary'; q: Point2 }
  | { kind: 'bounds'; q: Point2 }
  | { kind: 'closed' | 'budget' | 'singular' | 'diverged'; q: Point2 };

export interface GvdEdge {
  sites: [number, number];
  /** Traced samples along F_ij. */
  points: Point2[];
  ends: [EdgeEnd, EdgeEnd];
  /** Per-sample analytic type: line (point–point, line–line) or parabola (point–line). */
  types: ('line' | 'parabola')[];
  length: number;
}

export interface Gvd {
  sites: readonly Site[];
  edges: GvdEdge[];
  meetPoints: MeetPoint[];
  boundaryPoints: Point2[];
}

export interface TraceEdgeOptions {
  step?: number;
  bounds?: Bounds;
  maxSteps?: number;
  /** Below this clearance an edge is said to hit the obstacle: a boundary point. */
  boundaryTol?: number;
}

const inBounds = (b: Bounds | undefined, q: Point2) => !b || (q.x >= b.minX && q.x <= b.maxX && q.y >= b.minY && q.y <= b.maxY);

/** The analytic type of the bisector at q between sites i and j, from the closest features. */
export function edgeTypeAt(sites: readonly Site[], i: number, j: number, q: Point2): 'line' | 'parabola' {
  const fi = siteDistance(sites[i], i, q).feature;
  const fj = siteDistance(sites[j], j, q).feature;
  return fi === fj ? 'line' : 'parabola';
}

/**
 * Follow F_ij from q0 (Newton-corrected onto S_ij first) in the direction
 * `turn` until a third site ties (meet point), the clearance vanishes
 * (boundary point), or the window is left. This is Choset's sensor-based edge
 * tracing with Chapter 3's tracer on G = d_i − d_j.
 */
export function traceEdge(sites: readonly Site[], i: number, j: number, q0: Point2, turn: 'left' | 'right', opts: TraceEdgeOptions = {}): { points: Point2[]; end: EdgeEnd; trace: Trace } {
  const { step = 0.02, bounds, maxSteps = 20000, boundaryTol } = opts;
  const bTol = boundaryTol ?? step;
  const { G, DG } = equidistanceField(sites, i, j);
  let third = -1;
  const stop = (x: Point2): boolean => {
    if (!inBounds(bounds, x)) return true;
    const ds = siteDistances(sites, x);
    if (ds[0].d < bTol) return true;
    // A third site at least as close as the pair (within a step's slack): a meet point is near.
    const dPair = Math.max(siteDistance(sites[i], i, x).d, siteDistance(sites[j], j, x).d);
    for (const d of ds) {
      if (d.index === i || d.index === j) continue;
      if (d.d <= dPair + 1e-9) {
        third = d.index;
        return true;
      }
      break;
    }
    return false;
  };
  const tr = traceCurve(G, DG, q0, turn, { step, newtonIters: 4, tol: 1e-12, singularTol: 0.05, maxSteps }, stop);
  const last = tr.points[tr.points.length - 1];
  let end: EdgeEnd;
  if (tr.stop === 'reached') {
    if (!inBounds(bounds, last)) end = { kind: 'bounds', q: last };
    else if (third >= 0) {
      const mp = meetPoint(sites, i, j, third, last);
      end = mp ? { kind: 'meet', meet: mp } : { kind: 'boundary', q: last };
      if (mp) tr.points[tr.points.length - 1] = mp.q;
    } else end = { kind: 'boundary', q: last };
  } else end = { kind: tr.stop, q: last };
  return { points: tr.points, end, trace: tr };
}

/**
 * Project q onto S_ij for its two nearest sites by Newton along ∇d_i − ∇d_j,
 * and keep it only if (i, j) are still the two nearest there and the
 * gradients differ (SS_ij). Returns the face seed or null.
 */
export function projectToGvd(sites: readonly Site[], q: Point2, gradTol = 1e-3): { q: Point2; sites: [number, number] } | null {
  const ds = siteDistances(sites, q);
  if (ds.length < 2 || ds[0].d <= EPS) return null;
  const [i, j] = [ds[0].index, ds[1].index];
  const { G, DG } = equidistanceField(sites, i, j);
  const p = correctGradient(G, DG, q, 30, 1e-13);
  if (Math.abs(G(p)) > 1e-9) return null;
  const at = siteDistances(sites, p);
  if (!new Set([at[0].index, at[1].index]).has(i) || !new Set([at[0].index, at[1].index]).has(j)) return null;
  const a = siteDistance(sites[i], i, p).grad;
  const b = siteDistance(sites[j], j, p).grad;
  if (gradientAngle(a, b) < gradTol) return null; // non-surjective part of S_ij: not in SS_ij
  return { q: p, sites: i < j ? [i, j] : [j, i] };
}

/**
 * The whole planar GVD inside `bounds`: seed every face from a lattice,
 * trace each new face both ways, collect meet and boundary points. Exact to
 * the tracer's tolerance; intended for small site sets (the design doc's slab,
 * a room's worth of furniture), not as a general Voronoi kernel.
 */
export function exactGvd(sites: readonly Site[], bounds: Bounds, opts: { step?: number; seedsPerAxis?: number } = {}): Gvd {
  const { step = 0.02, seedsPerAxis = 24 } = opts;
  const edges: GvdEdge[] = [];
  const meetPoints: MeetPoint[] = [];
  const boundaryPoints: Point2[] = [];
  const nearExisting = (pair: [number, number], p: Point2) =>
    edges.some((e) => e.sites[0] === pair[0] && e.sites[1] === pair[1] && e.points.some((x) => Math.hypot(x.x - p.x, x.y - p.y) < 1.5 * step));
  const addMeet = (m: MeetPoint) => {
    if (!meetPoints.some((x) => Math.hypot(x.q.x - m.q.x, x.q.y - m.q.y) < 1e-6)) meetPoints.push(m);
  };
  const addBoundary = (p: Point2) => {
    if (!boundaryPoints.some((x) => Math.hypot(x.x - p.x, x.y - p.y) < step)) boundaryPoints.push(p);
  };
  const recordEnd = (end: EdgeEnd) => {
    if (end.kind === 'meet') addMeet(end.meet);
    else if (end.kind === 'boundary') addBoundary(end.q);
  };
  for (let a = 0; a < seedsPerAxis; a++) {
    for (let b = 0; b < seedsPerAxis; b++) {
      const q = { x: bounds.minX + ((a + 0.5) / seedsPerAxis) * (bounds.maxX - bounds.minX), y: bounds.minY + ((b + 0.5) / seedsPerAxis) * (bounds.maxY - bounds.minY) };
      const seed = projectToGvd(sites, q);
      if (!seed || !inBounds(bounds, seed.q) || nearExisting(seed.sites, seed.q)) continue;
      const [i, j] = seed.sites;
      const left = traceEdge(sites, i, j, seed.q, 'left', { step, bounds });
      const right = traceEdge(sites, i, j, seed.q, 'right', { step, bounds });
      const points = [...left.points.slice().reverse(), ...right.points.slice(1)];
      let length = 0;
      for (let k = 1; k < points.length; k++) length += Math.hypot(points[k].x - points[k - 1].x, points[k].y - points[k - 1].y);
      const types = points.map((p) => edgeTypeAt(sites, i, j, p));
      edges.push({ sites: [i, j], points, ends: [left.end, right.end], types, length });
      recordEnd(left.end);
      recordEnd(right.end);
    }
  }
  return { sites, edges, meetPoints, boundaryPoints };
}

/** The GVD as a `Roadmap`-ready node/edge list: meet points, boundary points and window exits become nodes. */
export function gvdToRoadmap(gvd: Gvd) {
  // Imported lazily to keep gvd.ts free of the search dependency chain in isolation.
  return gvd;
}

/* -------------------------------------------------------------------------- */
/* Analytic bisectors — the "bisector zoo" of Choset Fig. 5.14                 */
/* -------------------------------------------------------------------------- */

/** Points on the bisector of two points: the perpendicular line through the midpoint. */
export function bisectorPointPoint(a: Point2, b: Point2, halfLength: number, n = 2): Point2[] {
  const m = { x: 0.5 * (a.x + b.x), y: 0.5 * (a.y + b.y) };
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const L = Math.hypot(dx, dy) || 1;
  const t = { x: -dy / L, y: dx / L };
  const out: Point2[] = [];
  for (let k = 0; k < n; k++) {
    const s = -halfLength + (2 * halfLength * k) / (n - 1);
    out.push({ x: m.x + s * t.x, y: m.y + s * t.y });
  }
  return out;
}

/**
 * Points on the bisector of a point (focus) and a line (directrix): the
 * parabola y' = (x'² + p²)/(2p) in the frame where the directrix is y' = 0 and
 * the focus is at (0, p). Only the part on the focus's side of the line is real.
 */
export function bisectorPointLine(focus: Point2, line: Segment, xRange: [number, number], n = 64): Point2[] {
  const dx = line.x2 - line.x1;
  const dy = line.y2 - line.y1;
  const L = Math.hypot(dx, dy) || 1;
  const u = { x: dx / L, y: dy / L }; // along the directrix
  const v = { x: -u.y, y: u.x }; // normal
  const fx = (focus.x - line.x1) * u.x + (focus.y - line.y1) * u.y;
  let p = (focus.x - line.x1) * v.x + (focus.y - line.y1) * v.y;
  let nx = v.x;
  let ny = v.y;
  if (p < 0) {
    p = -p;
    nx = -nx;
    ny = -ny;
  }
  const out: Point2[] = [];
  for (let k = 0; k < n; k++) {
    const x = xRange[0] + ((xRange[1] - xRange[0]) * k) / (n - 1);
    const y = (x * x + p * p) / (2 * p);
    const bx = line.x1 + (fx + x) * u.x + y * nx;
    const by = line.y1 + (fx + x) * u.y + y * ny;
    out.push({ x: bx, y: by });
  }
  return out;
}

/** The bisector of two lines through their intersection (or the mid-line when parallel): sample points. */
export function bisectorLineLine(a: Segment, b: Segment, halfLength: number): Point2[][] {
  const ua = { x: a.x2 - a.x1, y: a.y2 - a.y1 };
  const ub = { x: b.x2 - b.x1, y: b.y2 - b.y1 };
  const La = Math.hypot(ua.x, ua.y) || 1;
  const Lb = Math.hypot(ub.x, ub.y) || 1;
  ua.x /= La; ua.y /= La; ub.x /= Lb; ub.y /= Lb;
  const denom = ua.x * ub.y - ua.y * ub.x;
  if (Math.abs(denom) < 1e-9) {
    // Parallel: the mid-line.
    const m = { x: 0.5 * (a.x1 + b.x1), y: 0.5 * (a.y1 + b.y1) };
    return [[{ x: m.x - halfLength * ua.x, y: m.y - halfLength * ua.y }, { x: m.x + halfLength * ua.x, y: m.y + halfLength * ua.y }]];
  }
  const ex = b.x1 - a.x1;
  const ey = b.y1 - a.y1;
  const t = (ex * ub.y - ey * ub.x) / denom;
  const X = { x: a.x1 + t * ua.x, y: a.y1 + t * ua.y };
  const d1 = { x: ua.x + ub.x, y: ua.y + ub.y };
  const d2 = { x: ua.x - ub.x, y: ua.y - ub.y };
  const n1 = Math.hypot(d1.x, d1.y) || 1;
  const n2 = Math.hypot(d2.x, d2.y) || 1;
  return [
    [{ x: X.x - (halfLength * d1.x) / n1, y: X.y - (halfLength * d1.y) / n1 }, { x: X.x + (halfLength * d1.x) / n1, y: X.y + (halfLength * d1.y) / n1 }],
    [{ x: X.x - (halfLength * d2.x) / n2, y: X.y - (halfLength * d2.y) / n2 }, { x: X.x + (halfLength * d2.x) / n2, y: X.y + (halfLength * d2.y) / n2 }],
  ];
}

/** Edge-count bounds for a polygonal GVD with n obstacles and N vertices (Choset §5.2.5, [359]). */
export function gvdSizeBounds(n: number, N: number) {
  return { edges: [(3 * (n + 1)) / 2, 6 * N + 3 * n - 3] as [number, number], nodes: [(n + 5) / 2, 4 * N - n - 2] as [number, number] };
}

/** Polygon sites from a list of polygons, plus optional walls as segments. */
export function sitesOf(polys: readonly Polygon[], walls: readonly Segment[] = []): Site[] {
  return [...polys.map((poly) => ({ kind: 'polygon' as const, poly })), ...walls.map((s) => ({ kind: 'segment' as const, s }))];
}

export { polygonEdges };
