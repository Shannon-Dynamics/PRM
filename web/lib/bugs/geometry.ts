/**
 * Planar geometry the Bug algorithms need and the shared `sim/world.ts` does
 * not provide: polygon obstacles, the nearest-point *witness* behind D(x) and
 * ∇D(x) (Choset eq. 2.4), m-line crossing counts for the Bug2 bound, and the
 * seeded obstacle fields the arena re-rolls.
 *
 * Everything is exact segment geometry. A `World` is a list of wall segments,
 * so a polygon obstacle becomes its edges and every shared primitive
 * (`rayCast`, `distanceToWalls`, `collides`) works unchanged; the polygon is
 * kept beside the walls because Bug1/Bug2 on a polygon world can follow the
 * boundary *exactly* (edge by edge), which is what makes the chapter's worked
 * example land on 34 and 16 to machine precision.
 */

import type { Rng } from '../prob/rng';
import { distanceToSegment, type Bounds, type Point2, type Segment, type World } from '../sim/world';

export type Turn = 'left' | 'right';

/** A simple closed polygon; the last vertex joins back to the first. */
export interface Polygon {
  readonly pts: readonly Point2[];
}

/** A `World` whose walls are exactly the edges of its polygon obstacles. */
export interface PolygonWorld extends World {
  readonly obstacles: readonly Polygon[];
  /** For wall k, the index of the polygon it belongs to. */
  readonly wallOwner: readonly number[];
}

// ---------------------------------------------------------------------------
// Vector helpers
// ---------------------------------------------------------------------------

export const pt = (x: number, y: number): Point2 => ({ x, y });
export const add = (a: Point2, b: Point2): Point2 => ({ x: a.x + b.x, y: a.y + b.y });
export const sub = (a: Point2, b: Point2): Point2 => ({ x: a.x - b.x, y: a.y - b.y });
export const scale = (a: Point2, k: number): Point2 => ({ x: a.x * k, y: a.y * k });
export const dot = (a: Point2, b: Point2): number => a.x * b.x + a.y * b.y;
/** z-component of a × b: positive when b is counter-clockwise from a. */
export const cross = (a: Point2, b: Point2): number => a.x * b.y - a.y * b.x;
export const norm = (a: Point2): number => Math.hypot(a.x, a.y);
export const dist = (a: Point2, b: Point2): number => Math.hypot(a.x - b.x, a.y - b.y);
/** Rotate by +90°: the counter-clockwise perpendicular. */
export const perp = (a: Point2): Point2 => ({ x: -a.y, y: a.x });
export const unit = (a: Point2): Point2 => {
  const n = norm(a);
  return n > 0 ? { x: a.x / n, y: a.y / n } : { x: 0, y: 0 };
};
export const lerp = (a: Point2, b: Point2, t: number): Point2 => ({
  x: a.x + (b.x - a.x) * t,
  y: a.y + (b.y - a.y) * t,
});

/** Length of a polyline. */
export function polylineLength(pts: readonly Point2[]): number {
  let L = 0;
  for (let i = 1; i < pts.length; i++) L += dist(pts[i - 1], pts[i]);
  return L;
}

// ---------------------------------------------------------------------------
// Segments
// ---------------------------------------------------------------------------

/** Closest point on segment ab to p, with its parameter t ∈ [0, 1]. */
export function closestOnSegment(p: Point2, a: Point2, b: Point2): { q: Point2; t: number; d: number } {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  let t = 0;
  if (len2 > 1e-24) t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
  const q = { x: a.x + t * dx, y: a.y + t * dy };
  return { q, t, d: dist(p, q) };
}

/**
 * Where does the directed segment a→b cross the segment s? Returns the
 * parameters (t along a→b, u along s) or null. Collinear overlap is reported as
 * a miss — a robot moving *along* an edge is grazing it, and Choset's Bug1
 * "need not invoke boundary following" for a graze.
 */
export function segmentHit(a: Point2, b: Point2, s: Segment): { t: number; u: number } | null {
  const rx = b.x - a.x;
  const ry = b.y - a.y;
  const sx = s.x2 - s.x1;
  const sy = s.y2 - s.y1;
  const denom = rx * sy - ry * sx;
  if (Math.abs(denom) < 1e-14) return null;
  const ex = s.x1 - a.x;
  const ey = s.y1 - a.y;
  const t = (ex * sy - ey * sx) / denom;
  const u = (ex * ry - ey * rx) / denom;
  if (t < -1e-12 || t > 1 + 1e-12 || u < -1e-12 || u > 1 + 1e-12) return null;
  return { t: Math.max(0, Math.min(1, t)), u: Math.max(0, Math.min(1, u)) };
}

// ---------------------------------------------------------------------------
// The witness: D(x) and ∇D(x) from exact geometry (Choset eq. 2.4)
// ---------------------------------------------------------------------------

export interface Witness {
  /** D(x): distance to the nearest wall. */
  d: number;
  /** The nearest wall point c*. */
  q: Point2;
  /** ∇D(x): the unit vector from c* to x, i.e. *away* from the obstacle. Zero at D = 0. */
  grad: Point2;
  /** Index of the wall segment that realizes the minimum. */
  wall: number;
}

/**
 * The nearest obstacle point and the gradient of distance. `D` is continuous
 * everywhere and differentiable off the medial axis (where two walls tie); on
 * a tie the first wall wins, which is the arbitrary-but-deterministic choice
 * a tracer needs.
 */
export function witness(world: World, p: Point2): Witness {
  let best: Witness = { d: Infinity, q: p, grad: { x: 0, y: 0 }, wall: -1 };
  for (let i = 0; i < world.walls.length; i++) {
    const s = world.walls[i];
    const c = closestOnSegment(p, { x: s.x1, y: s.y1 }, { x: s.x2, y: s.y2 });
    if (c.d < best.d) best = { d: c.d, q: c.q, grad: { x: 0, y: 0 }, wall: i };
  }
  if (best.d > 1e-15) best.grad = { x: (p.x - best.q.x) / best.d, y: (p.y - best.q.y) / best.d };
  return best;
}

// ---------------------------------------------------------------------------
// Polygons
// ---------------------------------------------------------------------------

export const rectangle = (x0: number, y0: number, x1: number, y1: number): Polygon => ({
  pts: [pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)],
});

export function polygonEdges(poly: Polygon): Segment[] {
  const n = poly.pts.length;
  const out: Segment[] = [];
  for (let i = 0; i < n; i++) {
    const a = poly.pts[i];
    const b = poly.pts[(i + 1) % n];
    out.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y });
  }
  return out;
}

export function polygonPerimeter(poly: Polygon): number {
  const n = poly.pts.length;
  let L = 0;
  for (let i = 0; i < n; i++) L += dist(poly.pts[i], poly.pts[(i + 1) % n]);
  return L;
}

/** Even–odd crossing test; boundary points are reported as inside. */
export function pointInPolygon(p: Point2, poly: Polygon): boolean {
  const v = poly.pts;
  const n = v.length;
  let inside = false;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const a = v[i];
    const b = v[j];
    // On an edge counts as inside: the robot is touching.
    if (closestOnSegment(p, a, b).d < 1e-12) return true;
    const crosses = a.y > p.y !== b.y > p.y && p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x;
    if (crosses) inside = !inside;
  }
  return inside;
}

/** Signed area: positive for counter-clockwise vertex order. */
export function signedArea(poly: Polygon): number {
  const v = poly.pts;
  let A = 0;
  for (let i = 0; i < v.length; i++) A += cross(v[i], v[(i + 1) % v.length]);
  return A / 2;
}

/** Distance between two polygons' boundaries (0 if they touch or cross). */
export function polygonGap(a: Polygon, b: Polygon): number {
  let best = Infinity;
  const ea = polygonEdges(a);
  const eb = polygonEdges(b);
  for (const s of ea) for (const t of eb) if (segmentHit({ x: s.x1, y: s.y1 }, { x: s.x2, y: s.y2 }, t)) return 0;
  for (const p of a.pts) for (const t of eb) best = Math.min(best, distanceToSegment(p.x, p.y, t));
  for (const p of b.pts) for (const s of ea) best = Math.min(best, distanceToSegment(p.x, p.y, s));
  return best;
}

/**
 * Number of times the *line* through a and b crosses the polygon boundary —
 * Choset's n_i in the Bug2 bound (2.2). A vertex exactly on the line is
 * counted once, by the half-open convention.
 */
export function lineCrossings(poly: Polygon, a: Point2, b: Point2): number {
  const d = sub(b, a);
  const side = (p: Point2) => cross(d, sub(p, a));
  const v = poly.pts;
  let n = 0;
  for (let i = 0; i < v.length; i++) {
    const s0 = side(v[i]);
    const s1 = side(v[(i + 1) % v.length]);
    if (s0 > 0 !== s1 > 0) n++;
  }
  return n;
}

/** A world whose walls are exactly the polygon edges. Bounds are limits, not walls. */
export function polygonWorld(name: string, bounds: Bounds, obstacles: readonly Polygon[]): PolygonWorld {
  const walls: Segment[] = [];
  const wallOwner: number[] = [];
  obstacles.forEach((poly, i) => {
    for (const e of polygonEdges(poly)) {
      walls.push(e);
      wallOwner.push(i);
    }
  });
  return { name, bounds, walls, obstacles, wallOwner };
}

/** Is this a polygon world (does it know its obstacles)? */
export function isPolygonWorld(world: World): world is PolygonWorld {
  return Array.isArray((world as PolygonWorld).obstacles) && Array.isArray((world as PolygonWorld).wallOwner);
}

/** The polygon (index) that contains p, or -1. */
export function containingPolygon(world: PolygonWorld, p: Point2): number {
  return world.obstacles.findIndex((poly) => pointInPolygon(p, poly));
}

// ---------------------------------------------------------------------------
// Constructions
// ---------------------------------------------------------------------------

/** Andrew's monotone chain; returns the hull counter-clockwise. */
export function convexHull(points: readonly Point2[]): Point2[] {
  const p = [...points].sort((u, v) => u.x - v.x || u.y - v.y);
  if (p.length < 3) return p;
  const half = (src: Point2[]) => {
    const h: Point2[] = [];
    for (const q of src) {
      while (h.length >= 2 && cross(sub(h[h.length - 1], h[h.length - 2]), sub(q, h[h.length - 2])) <= 0) h.pop();
      h.push(q);
    }
    h.pop();
    return h;
  };
  return [...half(p), ...half([...p].reverse())];
}

/**
 * A rectilinear spiral of wall thickness `w`, built as the outline of a thick
 * axis-aligned polyline whose segment lengths run L, L, 2L, 2L, 3L, 3L, …
 * (right, up, left, down, counter-clockwise outward). It is a single simple
 * polygon whose boundary crosses a horizontal m-line through its centre
 * 4·turns times — the construction behind Choset's figure 2.4: an obstacle
 * whose leave points are almost all on the wrong side, so Bug2 pays nearly a
 * full perimeter per crossing while Bug1 pays the perimeter once.
 */
export function spiralPolygon(center: Point2, L: number, w: number, turns: number): Polygon {
  const dirs = [pt(1, 0), pt(0, 1), pt(-1, 0), pt(0, -1)];
  const centerline: Point2[] = [center];
  for (let k = 0; k < 4 * turns; k++) {
    const len = L * (Math.floor(k / 2) + 1);
    centerline.push(add(centerline[k], scale(dirs[k % 4], len)));
  }
  return thickPolyline(centerline, w);
}

/**
 * Outline of an axis-aligned polyline with square caps. At each 90° corner the
 * two offset lines meet at v + (w/2)(n₁ + n₂); the result is one simple polygon
 * as long as every segment is longer than w.
 */
export function thickPolyline(cl: readonly Point2[], w: number): Polygon {
  const h = w / 2;
  const n = cl.length;
  const dirAt = (i: number) => unit(sub(cl[i + 1], cl[i]));
  const left: Point2[] = [];
  const right: Point2[] = [];
  for (let i = 0; i < n; i++) {
    let dIn = i > 0 ? dirAt(i - 1) : dirAt(0);
    let dOut = i < n - 1 ? dirAt(i) : dirAt(n - 2);
    if (i === 0) {
      // Start cap: extend backwards by h.
      const nL = perp(dOut);
      left.push(add(sub(cl[0], scale(dOut, h)), scale(nL, h)));
      right.push(sub(sub(cl[0], scale(dOut, h)), scale(nL, h)));
      continue;
    }
    if (i === n - 1) {
      const nL = perp(dIn);
      left.push(add(add(cl[i], scale(dIn, h)), scale(nL, h)));
      right.push(sub(add(cl[i], scale(dIn, h)), scale(nL, h)));
      continue;
    }
    dIn = unit(dIn);
    dOut = unit(dOut);
    const n1 = perp(dIn);
    const n2 = perp(dOut);
    left.push(add(cl[i], scale(add(n1, n2), h)));
    right.push(sub(cl[i], scale(add(n1, n2), h)));
  }
  return { pts: [...left, ...right.reverse()] };
}

export interface FieldOptions {
  /** Number of obstacles. */
  count: number;
  /** Radius range of the random blobs. */
  radius: [number, number];
  /** Minimum gap between obstacles, and between an obstacle and the start/goal. */
  margin: number;
  /** Points that must stay outside every obstacle (start and goal). */
  keepFree: readonly Point2[];
}

/**
 * A seeded field of disjoint convex polygons: random points on a jittered
 * circle, hulled, rejected when they come within `margin` of anything already
 * placed. Disjointness is what lets Bug1/Bug2 walk the boundary edge by edge.
 */
export function randomConvexField(rng: Rng, bounds: Bounds, opts: FieldOptions): Polygon[] {
  const out: Polygon[] = [];
  let attempts = 0;
  while (out.length < opts.count && attempts < 400) {
    attempts++;
    const r = rng.uniform(opts.radius[0], opts.radius[1]);
    const cx = rng.uniform(bounds.minX + r + opts.margin, bounds.maxX - r - opts.margin);
    const cy = rng.uniform(bounds.minY + r + opts.margin, bounds.maxY - r - opts.margin);
    const k = 4 + Math.floor(rng.uniform(0, 5));
    const pts: Point2[] = [];
    for (let i = 0; i < k; i++) {
      const a = (i / k) * 2 * Math.PI + rng.uniform(-0.25, 0.25);
      const rr = r * rng.uniform(0.55, 1);
      pts.push(pt(cx + rr * Math.cos(a), cy + rr * Math.sin(a)));
    }
    const poly: Polygon = { pts: convexHull(pts) };
    if (poly.pts.length < 3) continue;
    const edges = polygonEdges(poly);
    const tooClose = opts.keepFree.some(
      (p) => pointInPolygon(p, poly) || edges.some((e) => distanceToSegment(p.x, p.y, e) < opts.margin),
    );
    if (tooClose) continue;
    if (out.some((o) => polygonGap(o, poly) < opts.margin)) continue;
    out.push(poly);
  }
  return out;
}
