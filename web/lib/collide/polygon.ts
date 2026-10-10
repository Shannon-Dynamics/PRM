/**
 * Planar polygon geometry — the hand-rolled core of the "parry2d black box".
 *
 * The Rust side of the book delegates every distance and intersection query to
 * `parry2d` (GJK, EPA, a BVH broad phase) and never looks inside; Chapter 4 and
 * Appendix C explain what it does. The browser port has no such crate, so this
 * module *is* the box: point-in-polygon, segment–segment, segment–polygon,
 * polygon–polygon (separating axis for convex pairs, edge tests otherwise), and
 * the distance/witness versions of each. Everything returns exact answers for
 * polygons with straight edges; nothing here is approximate.
 *
 * Conventions: a `Polygon` is a simple closed loop of vertices, listed in any
 * winding (the code never assumes CCW). A point *on* the boundary counts as
 * inside, and two shapes that touch count as intersecting — the closed-set
 * convention Choset uses for R(q) ∩ WO_i ≠ ∅.
 */

import type { Point2, Segment } from '../sim/world';

export type Polygon = Point2[];

/** A nearest pair of points between two shapes, and their separation. */
export interface Witness {
  /** The minimizing point on the first shape (the robot, in collision queries). */
  onRobot: Point2;
  /** The minimizing point on the second shape (the obstacle). */
  onObstacle: Point2;
  /** ‖onRobot − onObstacle‖, exactly zero when the shapes intersect. */
  dist: number;
}

const EPS = 1e-12;

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

/** An axis-aligned rectangle [x0, x1] × [y0, y1] as a polygon. */
export function rect(x0: number, y0: number, x1: number, y1: number): Polygon {
  return [
    { x: x0, y: y0 },
    { x: x1, y: y0 },
    { x: x1, y: y1 },
    { x: x0, y: y1 },
  ];
}

/**
 * An oriented rectangle: center (cx, cy), half-extents (hl, hw) along and
 * across the heading θ. This is R(q) for every rigid rectangular body in the
 * book (the sofa, Hitch's chassis, the trailer).
 */
export function orientedRect(cx: number, cy: number, hl: number, hw: number, theta: number): Polygon {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  const corner = (a: number, b: number): Point2 => ({ x: cx + a * c - b * s, y: cy + a * s + b * c });
  return [corner(hl, hw), corner(-hl, hw), corner(-hl, -hw), corner(hl, -hw)];
}

/** The edges of a polygon, as wall-style segments. */
export function polygonEdges(poly: Polygon): Segment[] {
  const out: Segment[] = [];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    out.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y });
  }
  return out;
}

export function polygonBounds(poly: Polygon): { minX: number; minY: number; maxX: number; maxY: number } {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const p of poly) {
    if (p.x < minX) minX = p.x;
    if (p.x > maxX) maxX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.y > maxY) maxY = p.y;
  }
  return { minX, minY, maxX, maxY };
}

/** Signed area (shoelace). Positive for counter-clockwise vertex order. */
export function polygonArea(poly: Polygon): number {
  let a = 0;
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    a += p.x * q.y - q.x * p.y;
  }
  return a / 2;
}

/** Translate every vertex. */
export function translatePolygon(poly: Polygon, dx: number, dy: number): Polygon {
  return poly.map((p) => ({ x: p.x + dx, y: p.y + dy }));
}

/** Is every interior angle convex? (Collinear triples are tolerated.) */
export function isConvex(poly: Polygon): boolean {
  const n = poly.length;
  if (n < 3) return false;
  let sign = 0;
  for (let i = 0; i < n; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % n];
    const c = poly[(i + 2) % n];
    const cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    if (Math.abs(cross) < EPS) continue;
    const s = cross > 0 ? 1 : -1;
    if (sign === 0) sign = s;
    else if (s !== sign) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Point queries
// ---------------------------------------------------------------------------

/** Squared distance from p to the segment ab, plus the nearest point. */
export function closestPointOnSegment(px: number, py: number, s: Segment): Point2 {
  const dx = s.x2 - s.x1;
  const dy = s.y2 - s.y1;
  const len2 = dx * dx + dy * dy;
  let t = 0;
  if (len2 > EPS) {
    t = ((px - s.x1) * dx + (py - s.y1) * dy) / len2;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
  }
  return { x: s.x1 + t * dx, y: s.y1 + t * dy };
}

/** Is `p` on the segment (within tolerance)? */
function pointOnSegment(px: number, py: number, s: Segment, tol = 1e-9): boolean {
  const c = closestPointOnSegment(px, py, s);
  return Math.hypot(c.x - px, c.y - py) <= tol;
}

/**
 * Point-in-polygon by the even–odd ray rule, with boundary points counted as
 * inside. The boundary check comes first so a vertex or an edge grazed exactly
 * by the horizontal ray cannot flip the parity by accident.
 */
export function pointInPolygon(px: number, py: number, poly: Polygon): boolean {
  const n = poly.length;
  for (let i = 0; i < n; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % n];
    if (pointOnSegment(px, py, { x1: a.x, y1: a.y, x2: b.x, y2: b.y })) return true;
  }
  let inside = false;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const a = poly[i];
    const b = poly[j];
    const crosses = a.y > py !== b.y > py;
    if (crosses) {
      const xAt = ((b.x - a.x) * (py - a.y)) / (b.y - a.y) + a.x;
      if (px < xAt) inside = !inside;
    }
  }
  return inside;
}

/**
 * Distance from a point to a polygon with its witness: zero (and the point
 * itself) when inside, otherwise the nearest boundary point.
 */
export function distancePointPolygon(px: number, py: number, poly: Polygon): Witness {
  if (pointInPolygon(px, py, poly)) {
    return { onRobot: { x: px, y: py }, onObstacle: { x: px, y: py }, dist: 0 };
  }
  let best: Point2 = poly[0];
  let bestD = Infinity;
  for (const e of polygonEdges(poly)) {
    const c = closestPointOnSegment(px, py, e);
    const d = Math.hypot(c.x - px, c.y - py);
    if (d < bestD) {
      bestD = d;
      best = c;
    }
  }
  return { onRobot: { x: px, y: py }, onObstacle: best, dist: bestD };
}

// ---------------------------------------------------------------------------
// Segment queries
// ---------------------------------------------------------------------------

/**
 * Segment–segment intersection *including* touching and collinear overlap.
 * (The ported `segmentsIntersect` in sim/world ignores the collinear case,
 * which is right for a ray cast and wrong for a collision test: a link lying
 * exactly along a wall is in contact.)
 */
export function segmentsTouch(a: Segment, b: Segment): boolean {
  const orient = (px: number, py: number, qx: number, qy: number, rx: number, ry: number) => {
    const v = (qx - px) * (ry - py) - (qy - py) * (rx - px);
    return Math.abs(v) < EPS ? 0 : v > 0 ? 1 : -1;
  };
  const o1 = orient(a.x1, a.y1, a.x2, a.y2, b.x1, b.y1);
  const o2 = orient(a.x1, a.y1, a.x2, a.y2, b.x2, b.y2);
  const o3 = orient(b.x1, b.y1, b.x2, b.y2, a.x1, a.y1);
  const o4 = orient(b.x1, b.y1, b.x2, b.y2, a.x2, a.y2);
  if (o1 !== o2 && o3 !== o4) return true;
  // Collinear or touching cases: an endpoint lies on the other segment.
  if (o1 === 0 && pointOnSegment(b.x1, b.y1, a)) return true;
  if (o2 === 0 && pointOnSegment(b.x2, b.y2, a)) return true;
  if (o3 === 0 && pointOnSegment(a.x1, a.y1, b)) return true;
  if (o4 === 0 && pointOnSegment(a.x2, a.y2, b)) return true;
  return false;
}

/**
 * Closest points between two segments. For non-intersecting segments the
 * minimum is attained at an endpoint of one against the other, so four
 * point–segment projections suffice; intersecting ones return the crossing.
 */
export function distanceSegmentSegment(a: Segment, b: Segment): Witness {
  if (segmentsTouch(a, b)) {
    const x = segmentCrossing(a, b);
    return { onRobot: x, onObstacle: x, dist: 0 };
  }
  let best: Witness = { onRobot: { x: a.x1, y: a.y1 }, onObstacle: { x: b.x1, y: b.y1 }, dist: Infinity };
  const consider = (p: Point2, s: Segment, pIsRobot: boolean) => {
    const c = closestPointOnSegment(p.x, p.y, s);
    const d = Math.hypot(c.x - p.x, c.y - p.y);
    if (d < best.dist) {
      best = pIsRobot ? { onRobot: p, onObstacle: c, dist: d } : { onRobot: c, onObstacle: p, dist: d };
    }
  };
  consider({ x: a.x1, y: a.y1 }, b, true);
  consider({ x: a.x2, y: a.y2 }, b, true);
  consider({ x: b.x1, y: b.y1 }, a, false);
  consider({ x: b.x2, y: b.y2 }, a, false);
  return best;
}

/** A point common to two touching segments (the crossing, or a shared endpoint). */
function segmentCrossing(a: Segment, b: Segment): Point2 {
  const r1 = a.x2 - a.x1, r2 = a.y2 - a.y1;
  const s1 = b.x2 - b.x1, s2 = b.y2 - b.y1;
  const denom = r1 * s2 - r2 * s1;
  if (Math.abs(denom) > EPS) {
    const ex = b.x1 - a.x1, ey = b.y1 - a.y1;
    const t = (ex * s2 - ey * s1) / denom;
    return { x: a.x1 + t * r1, y: a.y1 + t * r2 };
  }
  // Collinear overlap: any endpoint lying on the other segment will do.
  for (const p of [{ x: b.x1, y: b.y1 }, { x: b.x2, y: b.y2 }]) if (pointOnSegment(p.x, p.y, a)) return p;
  return { x: a.x1, y: a.y1 };
}

/** Does the segment touch the polygon (cross an edge, or lie inside)? */
export function segmentIntersectsPolygon(s: Segment, poly: Polygon): boolean {
  if (pointInPolygon(s.x1, s.y1, poly)) return true;
  for (const e of polygonEdges(poly)) if (segmentsTouch(s, e)) return true;
  return false;
}

/** Distance between a segment and a polygon, with witness; zero when touching. */
export function distanceSegmentPolygon(s: Segment, poly: Polygon): Witness {
  if (pointInPolygon(s.x1, s.y1, poly)) {
    const p = { x: s.x1, y: s.y1 };
    return { onRobot: p, onObstacle: p, dist: 0 };
  }
  let best: Witness = { onRobot: { x: s.x1, y: s.y1 }, onObstacle: poly[0], dist: Infinity };
  for (const e of polygonEdges(poly)) {
    const w = distanceSegmentSegment(s, e);
    if (w.dist < best.dist) best = w;
    if (best.dist === 0) break;
  }
  return best;
}

// ---------------------------------------------------------------------------
// Polygon–polygon
// ---------------------------------------------------------------------------

/**
 * Separating-axis test for two *convex* polygons: they are disjoint iff some
 * edge normal of either separates their vertex projections. Touching counts
 * as intersecting.
 */
export function convexPolygonsIntersectSAT(a: Polygon, b: Polygon): boolean {
  const axes = (poly: Polygon) => {
    const out: Point2[] = [];
    for (let i = 0; i < poly.length; i++) {
      const p = poly[i];
      const q = poly[(i + 1) % poly.length];
      const nx = -(q.y - p.y);
      const ny = q.x - p.x;
      const len = Math.hypot(nx, ny);
      if (len > EPS) out.push({ x: nx / len, y: ny / len });
    }
    return out;
  };
  const project = (poly: Polygon, ax: Point2) => {
    let lo = Infinity, hi = -Infinity;
    for (const p of poly) {
      const v = p.x * ax.x + p.y * ax.y;
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    return [lo, hi] as const;
  };
  for (const ax of [...axes(a), ...axes(b)]) {
    const [aLo, aHi] = project(a, ax);
    const [bLo, bHi] = project(b, ax);
    if (aHi < bLo - EPS || bHi < aLo - EPS) return false;
  }
  return true;
}

/**
 * General polygon intersection by edge tests plus containment — correct for
 * any simple polygons, convex or not. This is the brute-force reference the
 * checks compare SAT against.
 */
export function polygonsIntersectBrute(a: Polygon, b: Polygon): boolean {
  const ea = polygonEdges(a);
  const eb = polygonEdges(b);
  for (const x of ea) for (const y of eb) if (segmentsTouch(x, y)) return true;
  return pointInPolygon(a[0].x, a[0].y, b) || pointInPolygon(b[0].x, b[0].y, a);
}

/** Intersection test: SAT when both are convex, edge tests otherwise. */
export function polygonsIntersect(a: Polygon, b: Polygon): boolean {
  if (isConvex(a) && isConvex(b)) return convexPolygonsIntersectSAT(a, b);
  return polygonsIntersectBrute(a, b);
}

/** Distance between two polygons with witness; zero when they intersect. */
export function distancePolygonPolygon(a: Polygon, b: Polygon): Witness {
  if (polygonsIntersect(a, b)) {
    const p = a[0];
    return { onRobot: p, onObstacle: p, dist: 0 };
  }
  let best: Witness = { onRobot: a[0], onObstacle: b[0], dist: Infinity };
  for (const e of polygonEdges(a)) {
    const w = distanceSegmentPolygon(e, b);
    if (w.dist < best.dist) best = w;
  }
  return best;
}

/** Distance between a polygon and a bare segment (a wall), with witness. */
export function distancePolygonSegment(poly: Polygon, s: Segment): Witness {
  // The roles are swapped relative to distanceSegmentPolygon: the polygon is the robot.
  const w = distanceSegmentPolygon(s, poly);
  return { onRobot: w.onObstacle, onObstacle: w.onRobot, dist: w.dist };
}
