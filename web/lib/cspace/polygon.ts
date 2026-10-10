/**
 * Convex polygons and the contact test — Choset Appendix F.1–F.2.
 *
 * Chapter 2 sealed collision geometry in a box (`collide/polygon.ts`: SAT,
 * edge tests, point-in-polygon). This module opens it the way Choset does:
 * a convex polygon is an intersection of negative half-planes h(x, y) =
 * ax + by − c ≤ 0 (F.1), two convex bodies can touch without overlapping in
 * exactly two ways — a robot edge against an obstacle vertex (Type A) or an
 * obstacle edge against a robot vertex (Type B) — and for a fixed heading θ
 * the C-obstacle is the intersection of one half-plane per *applicable*
 * contact (eqs. F.3–F.6). Nothing here replaces the Chapter 2 primitives;
 * the checks hold the two routes to the same verdict on thousands of poses.
 *
 * Conventions: every polygon handed to this module is simple and convex,
 * listed counterclockwise (`toCcw` fixes the winding; `isCcwConvex` tests
 * it). Edge i runs from vertex i to vertex i + 1 (mod n), and its outward
 * unit normal nᵢ is the edge direction rotated clockwise by 90°.
 */

import type { Pose2 } from '../geom/se2';
import type { Point2 } from '../sim/world';
import { isConvex, polygonArea, type Polygon } from '../collide/polygon';

export type { Polygon } from '../collide/polygon';

const EPS = 1e-12;

// ---------------------------------------------------------------------------
// F.1 — half-planes and winding
// ---------------------------------------------------------------------------

/** The line h(x, y) = a x + b y − c = 0 with unit normal (a, b); h⁻ is the inside. */
export interface HalfPlane {
  a: number;
  b: number;
  c: number;
}

export function evalHalfPlane(h: HalfPlane, x: number, y: number): number {
  return h.a * x + h.b * y - h.c;
}

/**
 * The half-plane of a counterclockwise edge p → q: its outward normal is the
 * edge rotated clockwise, so the polygon's interior is where h ≤ 0 — Choset's
 * "negative half-plane", chosen once so every polygon reads the same way.
 */
export function halfPlaneOfEdge(p: Point2, q: Point2): HalfPlane {
  const dx = q.x - p.x;
  const dy = q.y - p.y;
  const len = Math.hypot(dx, dy) || 1;
  const a = dy / len;
  const b = -dx / len;
  return { a, b, c: a * p.x + b * p.y };
}

/** The convex polygon as Choset writes it: a list of half-planes, one per edge. */
export function halfPlanes(poly: Polygon): HalfPlane[] {
  const n = poly.length;
  const out: HalfPlane[] = [];
  for (let i = 0; i < n; i++) out.push(halfPlaneOfEdge(poly[i], poly[(i + 1) % n]));
  return out;
}

/** Inside (boundary included) iff every half-plane value is ≤ 0. */
export function pointInConvex(poly: Polygon, x: number, y: number, tol = 1e-12): boolean {
  for (const h of halfPlanes(poly)) if (evalHalfPlane(h, x, y) > tol) return false;
  return true;
}

/** Counterclockwise copy (reverses a clockwise list; leaves CCW alone). */
export function toCcw(poly: Polygon): Polygon {
  return polygonArea(poly) < 0 ? [...poly].reverse() : [...poly];
}

export function isCcwConvex(poly: Polygon): boolean {
  return poly.length >= 3 && polygonArea(poly) > 0 && isConvex(poly);
}

/** Outward unit normals, one per edge, in edge order. */
export function edgeNormals(poly: Polygon): Point2[] {
  return halfPlanes(poly).map((h) => ({ x: h.a, y: h.b }));
}

/** Angle of a direction in [0, 2π). */
export function angleOf(v: Point2): number {
  const a = Math.atan2(v.y, v.x);
  return a < 0 ? a + 2 * Math.PI : a;
}

/** rᵢ(x, y, θ) = R(θ) rᵢ + (x, y): the robot's vertices at configuration q. */
export function transformPolygon(poly: Polygon, q: Pose2): Polygon {
  const c = Math.cos(q.theta);
  const s = Math.sin(q.theta);
  return poly.map((p) => ({ x: q.x + c * p.x - s * p.y, y: q.y + s * p.x + c * p.y }));
}

export function rotatePolygon(poly: Polygon, theta: number): Polygon {
  return transformPolygon(poly, { x: 0, y: 0, theta });
}

/** Point reflection −P: the robot's shape seen from the obstacle (⊖ is ⊕ with this). */
export function negatePolygon(poly: Polygon): Polygon {
  return poly.map((p) => ({ x: -p.x, y: -p.y }));
}

export function centroid(poly: Polygon): Point2 {
  let x = 0;
  let y = 0;
  for (const p of poly) {
    x += p.x;
    y += p.y;
  }
  return { x: x / poly.length, y: y / poly.length };
}

/**
 * Convex hull by Andrew's monotone chain, counterclockwise, collinear points
 * dropped. This is the brute-force oracle's only nontrivial step: the hull of
 * all pairwise differences oⱼ − rᵢ *is* the Minkowski difference, and the
 * checks hold the star algorithm to it.
 */
export function convexHull(points: readonly Point2[]): Polygon {
  const pts = [...points].sort((p, q) => p.x - q.x || p.y - q.y);
  const uniq: Point2[] = [];
  for (const p of pts) {
    const last = uniq[uniq.length - 1];
    if (!last || Math.abs(last.x - p.x) > EPS || Math.abs(last.y - p.y) > EPS) uniq.push(p);
  }
  if (uniq.length < 3) return uniq;
  const cross = (o: Point2, a: Point2, b: Point2) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  const lower: Point2[] = [];
  for (const p of uniq) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 1e-12) lower.pop();
    lower.push(p);
  }
  const upper: Point2[] = [];
  for (let i = uniq.length - 1; i >= 0; i--) {
    const p = uniq[i];
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 1e-12) upper.pop();
    upper.push(p);
  }
  lower.pop();
  upper.pop();
  return [...lower, ...upper];
}

/** Drop vertices that lie on the segment between their neighbours (within tol). */
export function removeCollinear(poly: Polygon, tol = 1e-9): Polygon {
  const n = poly.length;
  if (n < 4) return [...poly];
  const out: Point2[] = [];
  for (let i = 0; i < n; i++) {
    const a = poly[(i + n - 1) % n];
    const b = poly[i];
    const c = poly[(i + 1) % n];
    const cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    const scale = Math.hypot(c.x - a.x, c.y - a.y) || 1;
    if (Math.abs(cross) / scale > tol) out.push(b);
  }
  return out;
}

/**
 * A seeded random convex polygon with `n` vertices inside a disc of the given
 * radius: random angles sorted, random radii — always convex because the
 * points are then hulled. Used by the property checks and by w4.2's re-roll.
 */
export function randomConvexPolygon(
  rng: { uniform(a: number, b: number): number },
  n: number,
  radius: number,
  center: Point2 = { x: 0, y: 0 },
): Polygon {
  const pts: Point2[] = [];
  for (let k = 0; k < n; k++) {
    const a = rng.uniform(0, 2 * Math.PI);
    const r = rng.uniform(0.45 * radius, radius);
    pts.push({ x: center.x + r * Math.cos(a), y: center.y + r * Math.sin(a) });
  }
  const hull = convexHull(pts);
  // A hull of fewer than three points is a degenerate draw; redraw.
  return hull.length >= 3 ? hull : randomConvexPolygon(rng, n, radius, center);
}

// ---------------------------------------------------------------------------
// F.2 — contacts, applicability, and the half-plane intersection test
// ---------------------------------------------------------------------------

/**
 * The two ways convex bodies touch without overlapping (eq. F.2, figure F.4).
 * `robotEdge`/`obstacleEdge` index an edge; `robotVertex`/`obstacleVertex` a
 * vertex. Type A: edge E^R_i contains vertex o_j. Type B: edge E^W_j contains
 * vertex r_i.
 */
export type Contact =
  | { kind: 'A'; robotEdge: number; obstacleVertex: number }
  | { kind: 'B'; obstacleEdge: number; robotVertex: number };

/**
 * Applicability (eqs. F.3 and F.5), written as Choset writes them — two dot
 * products with the vertex's neighbours — rather than as the angular
 * reformulation the star algorithm uses. The two agree; `star.ts` relies on
 * the angular one and the checks compare the emitted pairs with these.
 *
 * Type A: robot edge i can touch obstacle vertex j only if both edges leaving
 * o_j point *along or away from* the robot's normal: (o_{j−1} − o_j)·n^R_i ≥ 0
 * and (o_{j+1} − o_j)·n^R_i ≥ 0. Type B is the same with the roles swapped.
 * Only θ matters: the robot's normals rotate with it, its position does not
 * enter.
 */
export function applicableContacts(robot: Polygon, theta: number, obstacle: Polygon, tol = 1e-12): Contact[] {
  const r = rotatePolygon(robot, theta);
  const nR = edgeNormals(r);
  const nW = edgeNormals(obstacle);
  const nr = r.length;
  const nw = obstacle.length;
  const out: Contact[] = [];
  for (let i = 0; i < nr; i++) {
    for (let j = 0; j < nw; j++) {
      const o = obstacle[j];
      const prev = obstacle[(j + nw - 1) % nw];
      const next = obstacle[(j + 1) % nw];
      const d1 = (prev.x - o.x) * nR[i].x + (prev.y - o.y) * nR[i].y;
      const d2 = (next.x - o.x) * nR[i].x + (next.y - o.y) * nR[i].y;
      if (d1 >= -tol && d2 >= -tol) out.push({ kind: 'A', robotEdge: i, obstacleVertex: j });
    }
  }
  for (let j = 0; j < nw; j++) {
    for (let i = 0; i < nr; i++) {
      const v = r[i];
      const prev = r[(i + nr - 1) % nr];
      const next = r[(i + 1) % nr];
      const d1 = (prev.x - v.x) * nW[j].x + (prev.y - v.y) * nW[j].y;
      const d2 = (next.x - v.x) * nW[j].x + (next.y - v.y) * nW[j].y;
      if (d1 >= -tol && d2 >= -tol) out.push({ kind: 'B', obstacleEdge: j, robotVertex: i });
    }
  }
  return out;
}

/**
 * The half-plane value of one contact at configuration q (eqs. F.4, F.6):
 *   Type A:  f^R_ij(q) = n^R_i(θ) · (o_j − r_i(q))
 *   Type B:  f^W_ij(q) = n^W_j · (r_i(q) − o_j)
 * Collision requires every applicable value to be ≤ 0.
 */
export function contactValue(contact: Contact, robot: Polygon, q: Pose2, obstacle: Polygon): number {
  const r = transformPolygon(robot, q);
  if (contact.kind === 'A') {
    const n = edgeNormals(r)[contact.robotEdge];
    const o = obstacle[contact.obstacleVertex];
    const ri = r[contact.robotEdge];
    return n.x * (o.x - ri.x) + n.y * (o.y - ri.y);
  }
  const n = edgeNormals(obstacle)[contact.obstacleEdge];
  const o = obstacle[contact.obstacleEdge];
  const ri = r[contact.robotVertex];
  return n.x * (ri.x - o.x) + n.y * (ri.y - o.y);
}

/** One row of Choset's figure F.7 table: the contact, its value, and the verdict. */
export interface ContactRow {
  contact: Contact;
  value: number;
  satisfied: boolean;
}

/** The whole table for a configuration — what the prose prints beside figure F.7. */
export function contactTable(robot: Polygon, q: Pose2, obstacle: Polygon, tol = 1e-12): ContactRow[] {
  return applicableContacts(robot, q.theta, obstacle, tol).map((contact) => {
    const value = contactValue(contact, robot, q, obstacle);
    return { contact, value, satisfied: value <= tol };
  });
}

/**
 * The intersection test of App. F.2: q ∈ QO(θ) iff every applicable
 * half-plane constraint holds. For convex bodies this is exact and, with the
 * normals pre-sorted, linear — but it is written here for clarity, not speed:
 * its job is to make `convexPolygonsIntersectSAT` (the Chapter 2 primitive)
 * a *verified* black box rather than a trusted one.
 */
export function polygonIntersectsByContacts(robot: Polygon, q: Pose2, obstacle: Polygon, tol = 1e-12): boolean {
  for (const c of applicableContacts(robot, q.theta, obstacle, tol)) {
    if (contactValue(c, robot, q, obstacle) > tol) return false;
  }
  return true;
}

/** A readable name for a contact, as in Choset's tables: "E^R_1, o_4" or "E^W_3, r_1". */
export function contactName(c: Contact): string {
  return c.kind === 'A'
    ? `E^R_${c.robotEdge + 1}, o_${c.obstacleVertex + 1}`
    : `E^W_${c.obstacleEdge + 1}, r_${c.robotVertex + 1}`;
}
