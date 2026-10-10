/**
 * The planar polygonal free space every decomposition in Chapter 10 runs on.
 *
 * Choset §6.1 assumes "the free space is bounded by a polygon and all of the
 * obstacles are polygonal": a `CellWorld` is exactly that — one counter-
 * clockwise boundary polygon and a list of counter-clockwise obstacle
 * polygons (holes) strictly inside it. The polygons are *C-obstacles*: for
 * disc-Rusty they are Chapter 4's inflations (`inflateDisc`), for Reach's
 * end-effector they are the Workbench's objects themselves, and the robot is a
 * point. Everything downstream — the sweep, the Morse slices, the coverage
 * raster, the range ring of the online coverer, the visibility polygon of the
 * pursuit coda — reads this one shape.
 *
 * General position (no two vertices share an x-coordinate) is the sweep's
 * standing assumption. Rectilinear worlds violate it on every wall, so
 * `generalPosition` applies a tiny deterministic shear x ← x + σy, which keeps
 * edges straight and polygons simple and moves nothing by more than a
 * millimetre. The text discloses it; the checks run on the sheared world.
 */

import type { Polygon } from '../collide/polygon';
import { polygonArea, pointInPolygon } from '../collide/polygon';
import { toCcw } from '../cspace/polygon';
import type { Bounds, Point2, Segment, World } from '../sim/world';

export interface CellWorld {
  name: string;
  /** The outer boundary of Q_free, counter-clockwise. */
  boundary: Polygon;
  /** Holes (C-obstacles), counter-clockwise, disjoint, strictly inside the boundary. */
  obstacles: Polygon[];
  /** Names for the obstacles, for labels ("the sofa", "T", "K"). */
  obstacleNames?: string[];
}

/** One polygon edge with a global id: boundary edges first, then each obstacle's. */
export interface CellEdge {
  id: number;
  /** −1 for the boundary, otherwise the obstacle index. */
  owner: number;
  /** Index of the edge within its polygon (edge k joins vertex k to k + 1). */
  k: number;
  a: Point2;
  b: Point2;
}

/** Normalize orientation: every polygon counter-clockwise. */
export function cellWorld(name: string, boundary: Polygon, obstacles: Polygon[], obstacleNames?: string[]): CellWorld {
  return { name, boundary: toCcw(boundary), obstacles: obstacles.map(toCcw), obstacleNames };
}

/** Every polygon of the world: the boundary at index 0, then the obstacles. */
export function polygonsOf(w: CellWorld): Polygon[] {
  return [w.boundary, ...w.obstacles];
}

export function allEdges(w: CellWorld): CellEdge[] {
  const out: CellEdge[] = [];
  const push = (poly: Polygon, owner: number) => {
    for (let k = 0; k < poly.length; k++) out.push({ id: out.length, owner, k, a: poly[k], b: poly[(k + 1) % poly.length] });
  };
  push(w.boundary, -1);
  w.obstacles.forEach((o, i) => push(o, i));
  return out;
}

export function worldBounds(w: CellWorld): Bounds {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const p of w.boundary) {
    minX = Math.min(minX, p.x);
    minY = Math.min(minY, p.y);
    maxX = Math.max(maxX, p.x);
    maxY = Math.max(maxY, p.y);
  }
  return { minX, minY, maxX, maxY };
}

/**
 * The world as the shared polyline `World` — every polygon edge a wall — so
 * `rayCast`, `rangeScan`, `distanceToWalls` and Chapter 3's offset follower
 * run on it unchanged.
 */
export function asWorld(w: CellWorld): World {
  const walls: Segment[] = allEdges(w).map((e) => ({ x1: e.a.x, y1: e.a.y, x2: e.b.x, y2: e.b.y }));
  const b = worldBounds(w);
  return { name: w.name, bounds: { minX: b.minX - 0.05, minY: b.minY - 0.05, maxX: b.maxX + 0.05, maxY: b.maxY + 0.05 }, walls };
}

/** Area of Q_free: the boundary's area less the holes'. */
export function freeArea(w: CellWorld): number {
  return Math.abs(polygonArea(w.boundary)) - w.obstacles.reduce((s, o) => s + Math.abs(polygonArea(o)), 0);
}

export function perimeter(poly: Polygon): number {
  let p = 0;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    p += Math.hypot(b.x - a.x, b.y - a.y);
  }
  return p;
}

/** P_total: the perimeter of every obstacle plus the outer boundary (Choset §6.2.5). */
export function totalPerimeter(w: CellWorld): number {
  return polygonsOf(w).reduce((s, p) => s + perimeter(p), 0);
}

/** Strictly inside a polygon, i.e. inside and farther than `tol` from every edge. */
export function strictlyInsidePolygon(p: Point2, poly: Polygon, tol = 1e-9): boolean {
  if (!pointInPolygon(p.x, p.y, poly)) return false;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    if (distPointSegment(p, a, b) <= tol) return false;
  }
  return true;
}

export function distPointSegment(p: Point2, a: Point2, b: Point2): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const L2 = dx * dx + dy * dy;
  let t = L2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return Math.hypot(a.x + t * dx - p.x, a.y + t * dy - p.y);
}

/** p ∈ cl(Q_free): inside the boundary (edges included) and not strictly inside any hole. */
export function isFreePoint(w: CellWorld, p: Point2, tol = 1e-9): boolean {
  if (!pointInPolygon(p.x, p.y, w.boundary)) {
    // pointInPolygon counts the boundary as inside; allow a hair of slack for round-off.
    let near = false;
    for (let i = 0; i < w.boundary.length; i++) {
      if (distPointSegment(p, w.boundary[i], w.boundary[(i + 1) % w.boundary.length]) <= tol) near = true;
    }
    if (!near) return false;
  }
  for (const o of w.obstacles) if (strictlyInsidePolygon(p, o, tol)) return false;
  return true;
}

/** Distance from p to the nearest edge of the world. */
export function distanceToBoundary(w: CellWorld, p: Point2): number {
  let d = Infinity;
  for (const e of allEdges(w)) d = Math.min(d, distPointSegment(p, e.a, e.b));
  return d;
}

// ---------------------------------------------------------------------------
// General position
// ---------------------------------------------------------------------------

/** Smallest gap between distinct vertex x-coordinates (0 when two coincide). */
export function minVertexXGap(w: CellWorld): number {
  const xs = polygonsOf(w).flat().map((p) => p.x).sort((a, b) => a - b);
  let g = Infinity;
  for (let i = 1; i < xs.length; i++) g = Math.min(g, xs[i] - xs[i - 1]);
  return g;
}

/** The shear x ← x + σ·y applied to every vertex. */
export function shearWorld(w: CellWorld, sigma: number): CellWorld {
  const s = (poly: Polygon) => poly.map((p) => ({ x: p.x + sigma * p.y, y: p.y }));
  return { ...w, boundary: s(w.boundary), obstacles: w.obstacles.map(s) };
}

/** The shear `generalPosition` uses: about a milli-radian, an irrational-ish constant. */
export const GENERAL_POSITION_SHEAR = 1.0e-3 * Math.SQRT2;

/**
 * Enforce Choset's general-position assumption: if two vertices share an
 * x-coordinate (to within `tol`), shear the world by σ = √2·10⁻³. A vertical
 * wall of a rectilinear room becomes a wall leaning by a milli-radian; its two
 * end vertices then have distinct x, one of them a critical point and the
 * other a pass-through, which is exactly the "perturbation fix" of §6.1.
 */
export function generalPosition(w: CellWorld, tol = 1e-7): { world: CellWorld; sheared: boolean } {
  if (minVertexXGap(w) > tol) return { world: w, sheared: false };
  return { world: shearWorld(w, GENERAL_POSITION_SHEAR), sheared: true };
}

// ---------------------------------------------------------------------------
// Validity (for the draggable widgets)
// ---------------------------------------------------------------------------

function segmentsProperlyCross(a: Point2, b: Point2, c: Point2, d: Point2): boolean {
  const o = (p: Point2, q: Point2, r: Point2) => (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
  const d1 = o(a, b, c), d2 = o(a, b, d), d3 = o(c, d, a), d4 = o(c, d, b);
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

/** Is the polygon simple (no two non-adjacent edges cross)? */
export function isSimplePolygon(poly: Polygon): boolean {
  const n = poly.length;
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      if (j === i + 1 || (i === 0 && j === n - 1)) continue;
      if (segmentsProperlyCross(poly[i], poly[(i + 1) % n], poly[j], poly[(j + 1) % n])) return false;
    }
  }
  return true;
}

/**
 * A world the sweep can digest: simple polygons, holes strictly inside the
 * boundary and pairwise disjoint (with a margin), vertices in general position.
 */
export function isValidWorld(w: CellWorld, margin = 0.05): boolean {
  const polys = polygonsOf(w);
  if (!polys.every(isSimplePolygon)) return false;
  for (const o of w.obstacles) {
    for (const p of o) {
      if (!strictlyInsidePolygon(p, w.boundary, margin)) return false;
    }
  }
  for (let i = 0; i < w.obstacles.length; i++) {
    for (let j = i + 1; j < w.obstacles.length; j++) {
      const A = w.obstacles[i], B = w.obstacles[j];
      for (const p of A) if (pointInPolygon(p.x, p.y, B)) return false;
      for (const p of B) if (pointInPolygon(p.x, p.y, A)) return false;
      for (let a = 0; a < A.length; a++) {
        for (let b = 0; b < B.length; b++) {
          if (segmentsProperlyCross(A[a], A[(a + 1) % A.length], B[b], B[(b + 1) % B.length])) return false;
        }
      }
      for (const p of A) for (let b = 0; b < B.length; b++) if (distPointSegment(p, B[b], B[(b + 1) % B.length]) < margin) return false;
      for (const p of B) for (let a = 0; a < A.length; a++) if (distPointSegment(p, A[a], A[(a + 1) % A.length]) < margin) return false;
    }
  }
  return minVertexXGap(w) > 1e-9;
}

// ---------------------------------------------------------------------------
// The smallest enclosing disk (Δ of §6.2.5)
// ---------------------------------------------------------------------------

interface Circle {
  c: Point2;
  r: number;
}

const inCircle = (p: Point2, C: Circle) => Math.hypot(p.x - C.c.x, p.y - C.c.y) <= C.r + 1e-9;

function circleFrom2(a: Point2, b: Point2): Circle {
  const c = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
  return { c, r: Math.hypot(a.x - c.x, a.y - c.y) };
}

function circleFrom3(a: Point2, b: Point2, c: Point2): Circle | null {
  const d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
  if (Math.abs(d) < 1e-14) return null;
  const a2 = a.x * a.x + a.y * a.y, b2 = b.x * b.x + b.y * b.y, c2 = c.x * c.x + c.y * c.y;
  const ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
  const uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
  return { c: { x: ux, y: uy }, r: Math.hypot(a.x - ux, a.y - uy) };
}

/**
 * Minimum enclosing circle by the incremental (Welzl-style) construction,
 * deterministic order. O(n) expected on shuffled input; the worlds here have
 * a few dozen boundary vertices, so the order does not matter.
 */
export function minEnclosingCircle(points: readonly Point2[]): Circle {
  let C: Circle = { c: points[0], r: 0 };
  for (let i = 1; i < points.length; i++) {
    if (inCircle(points[i], C)) continue;
    C = { c: points[i], r: 0 };
    for (let j = 0; j < i; j++) {
      if (inCircle(points[j], C)) continue;
      C = circleFrom2(points[i], points[j]);
      for (let k = 0; k < j; k++) {
        if (inCircle(points[k], C)) continue;
        C = circleFrom3(points[i], points[j], points[k]) ?? C;
      }
    }
  }
  return C;
}

/** Δ: the diameter of the smallest disk containing the free space. */
export function enclosingDiameter(w: CellWorld): number {
  return 2 * minEnclosingCircle(w.boundary).r;
}
