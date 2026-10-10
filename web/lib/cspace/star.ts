/**
 * Minkowski sums and differences — Choset §3.2.1 and Appendix F.3–F.4.
 *
 * Three constructions of a C-obstacle, each exact for the bodies it covers:
 *
 *   inflateDisc      a disc translating among polygons: QO = WO ⊕ B_r, the
 *                    polygon grown by r with rounded corners (§3.2.1);
 *   starAlgorithm    a convex polygon translating at fixed heading θ among a
 *                    convex polygon: QO(θ) = WO ⊖ R(0, 0, θ), computed by
 *                    merging two sorted lists of edge normals (F.3);
 *   se2Slices        the same body rotating too: QO ⊂ SE(2) as a stack of
 *                    θ-slices (F.4, figure F.8).
 *
 * The star algorithm is the owned artifact of this chapter — Chapters 8 and
 * 10 import it — so it is written to be *checked*, not merely used: every
 * emitted vertex carries the contact that produced it, the scan is recorded
 * step by step for the Minkowski Studio widget, and `minkowskiBruteForce`
 * (the hull of all pairwise differences oⱼ − rᵢ) is the oracle the checks
 * hold it to on hundreds of seeded pairs.
 */

import type { Point2, Segment } from '../sim/world';
import { polygonArea, type Polygon } from '../collide/polygon';
import { angleOf, convexHull, edgeNormals, removeCollinear, rotatePolygon, toCcw } from './polygon';

const TAU = 2 * Math.PI;

// ---------------------------------------------------------------------------
// §3.2.1 — the disc: inflation
// ---------------------------------------------------------------------------

/**
 * WO ⊕ B_r for a convex polygon: edges offset outward by r, vertices replaced
 * by arcs between the adjacent edge normals. `arcSegments` is the number of
 * chords per full turn; the result is a polygon that converges to the true
 * inflation from the inside as it grows (chords cut the arcs).
 */
export function inflateDisc(obstacle: Polygon, r: number, arcSegments = 32): Polygon {
  const poly = toCcw(obstacle);
  const n = poly.length;
  if (r <= 0) return poly;
  const normals = edgeNormals(poly);
  const out: Point2[] = [];
  const dphi = TAU / Math.max(4, arcSegments);
  for (let j = 0; j < n; j++) {
    const o = poly[j];
    // The arc at vertex j sweeps from the normal of the incoming edge to that
    // of the outgoing edge, counterclockwise.
    const a0 = angleOf(normals[(j + n - 1) % n]);
    let a1 = angleOf(normals[j]);
    if (a1 < a0) a1 += TAU;
    const steps = Math.max(1, Math.ceil((a1 - a0) / dphi - 1e-9));
    for (let k = 0; k <= steps; k++) {
      const a = a0 + ((a1 - a0) * k) / steps;
      out.push({ x: o.x + r * Math.cos(a), y: o.y + r * Math.sin(a) });
    }
  }
  return out;
}

/**
 * |WO ⊕ B_r| = |WO| + r·p + π r² for a convex WO of perimeter p: the original
 * area, a strip of width r along the boundary, and the corner arcs, which
 * together make exactly one disc (their angles sum to 2π).
 */
export function inflatedArea(obstacle: Polygon, r: number): number {
  const poly = toCcw(obstacle);
  let p = 0;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    p += Math.hypot(b.x - a.x, b.y - a.y);
  }
  return Math.abs(polygonArea(poly)) + r * p + Math.PI * r * r;
}

/** A wall segment grown by r: a capsule (stadium), the inflation of a zero-width obstacle. */
export function inflateSegment(s: Segment, r: number, arcSegments = 32): Polygon {
  const dx = s.x2 - s.x1;
  const dy = s.y2 - s.y1;
  const len = Math.hypot(dx, dy) || 1;
  const ux = dx / len;
  const uy = dy / len;
  const base = Math.atan2(uy, ux);
  const half = Math.max(2, Math.ceil(arcSegments / 2));
  const out: Point2[] = [];
  // Right side of the segment, then the arc around the far end, left side, arc around the near end.
  for (let k = 0; k <= half; k++) {
    const a = base - Math.PI / 2 + (Math.PI * k) / half;
    out.push({ x: s.x2 + r * Math.cos(a), y: s.y2 + r * Math.sin(a) });
  }
  for (let k = 0; k <= half; k++) {
    const a = base + Math.PI / 2 + (Math.PI * k) / half;
    out.push({ x: s.x1 + r * Math.cos(a), y: s.y1 + r * Math.sin(a) });
  }
  return out;
}

// ---------------------------------------------------------------------------
// F.3 — the star algorithm
// ---------------------------------------------------------------------------

/** One entry of the merged normal list: whose edge, which edge, at what angle. */
export interface MergedNormal {
  source: 'robot' | 'obstacle';
  edge: number;
  /** Angle in [0, 2π) of the obstacle normal, or of the *negated* robot normal. */
  angle: number;
}

/** A vertex of QO(θ) with the contact that produced it: o_j − r_i. */
export interface StarVertex {
  point: Point2;
  kind: 'A' | 'B';
  robotVertex: number;
  obstacleVertex: number;
}

/** One step of the scan, recorded for the widget: the entry consumed and the vertex it emitted. */
export interface StarStep {
  entry: MergedNormal;
  kind: 'A' | 'B';
  /** The edge that slides: robot edge (A) or obstacle edge (B). */
  edge: number;
  /** The vertex it slides along: obstacle vertex (A) or robot vertex (B). */
  vertex: number;
  from: Point2;
  to: Point2;
}

export interface StarResult {
  /** QO(θ) counterclockwise, collinear vertices removed. */
  polygon: Polygon;
  /** Every emitted vertex in scan order (may contain collinear runs when edges are parallel). */
  vertices: StarVertex[];
  merged: MergedNormal[];
  steps: StarStep[];
  /** The robot's vertices at heading θ (reference point at the origin). */
  robotAtTheta: Polygon;
}

/**
 * QO(θ) = WO ⊖ R(0, 0, θ) by Choset's star algorithm.
 *
 * Why it works, in one paragraph: the boundary of a Minkowski sum of two
 * convex polygons is made of the edges of both, in order of their outward
 * normals. WO ⊖ R is WO ⊕ (−R), and −R's normals are the negated normals of
 * R, so: negate the robot's normals, merge them with the obstacle's by angle,
 * and walk the merged list. Walking an obstacle edge E^W_j while the current
 * robot vertex r_i stays put is a Type B contact and ends at o_{j+1} − r_i;
 * walking a (negated) robot edge E^R_i while the obstacle vertex o_j stays put
 * is a Type A contact and ends at o_j − r_{i+1}. The start is the vertex pair
 * extreme in the +x direction, o_{j₀} − r_{i₀}, where j₀, i₀ are the edges
 * whose normals come first in angle. Cost: O((n_R + n_W) log(n_R + n_W)) for
 * the sort, linear after — and here both lists are already in order because
 * a convex polygon's normals rotate monotonically, so it is one merge.
 */
export function starAlgorithm(robot: Polygon, theta: number, obstacle: Polygon): StarResult {
  const R = rotatePolygon(toCcw(robot), theta);
  const W = toCcw(obstacle);
  const nr = R.length;
  const nw = W.length;
  const nR = edgeNormals(R);
  const nW = edgeNormals(W);

  const robotEntries: MergedNormal[] = nR.map((n, i) => ({ source: 'robot', edge: i, angle: angleOf({ x: -n.x, y: -n.y }) }));
  const obsEntries: MergedNormal[] = nW.map((n, j) => ({ source: 'obstacle', edge: j, angle: angleOf(n) }));

  // Each list is cyclically sorted; rotate so each starts at its smallest angle.
  const startAt = (list: MergedNormal[]) => {
    let k = 0;
    for (let m = 1; m < list.length; m++) if (list[m].angle < list[k].angle) k = m;
    return [...list.slice(k), ...list.slice(0, k)];
  };
  const rs = startAt(robotEntries);
  const os = startAt(obsEntries);

  // The merge.
  const merged: MergedNormal[] = [];
  let a = 0;
  let b = 0;
  while (a < rs.length || b < os.length) {
    if (b >= os.length || (a < rs.length && rs[a].angle <= os[b].angle)) merged.push(rs[a++]);
    else merged.push(os[b++]);
  }

  // Current vertex indices: the vertex at the *start* of each first edge is the
  // one extreme in the +x direction, so their difference is a vertex of QO(θ).
  let i = rs[0].edge;
  let j = os[0].edge;
  const diff = (jj: number, ii: number): Point2 => ({ x: W[jj].x - R[ii].x, y: W[jj].y - R[ii].y });

  const vertices: StarVertex[] = [];
  const steps: StarStep[] = [];
  let current = diff(j, i);
  // The first vertex is emitted by the last step's return; record it up front
  // so the polygon starts where the scan starts.
  vertices.push({ point: current, kind: merged[0].source === 'robot' ? 'A' : 'B', robotVertex: i, obstacleVertex: j });

  for (let k = 0; k < merged.length; k++) {
    const e = merged[k];
    let next: Point2;
    let step: StarStep;
    if (e.source === 'robot') {
      // Type A: robot edge i slides along obstacle vertex j, from o_j − r_i to o_j − r_{i+1}.
      const i2 = (e.edge + 1) % nr;
      next = diff(j, i2);
      step = { entry: e, kind: 'A', edge: e.edge, vertex: j, from: current, to: next };
      i = i2;
      vertices.push({ point: next, kind: 'A', robotVertex: i, obstacleVertex: j });
    } else {
      // Type B: obstacle edge j slides along robot vertex i, from o_j − r_i to o_{j+1} − r_i.
      const j2 = (e.edge + 1) % nw;
      next = diff(j2, i);
      step = { entry: e, kind: 'B', edge: e.edge, vertex: i, from: current, to: next };
      j = j2;
      vertices.push({ point: next, kind: 'B', robotVertex: i, obstacleVertex: j });
    }
    steps.push(step);
    current = next;
  }
  // The scan closes on its start; drop the duplicate closing vertex.
  vertices.pop();

  return {
    polygon: removeCollinear(vertices.map((v) => v.point)),
    vertices,
    merged,
    steps,
    robotAtTheta: R,
  };
}

/**
 * The oracle (eq. F.7 read literally): WO ⊖ R(θ) = {o − r} is the convex hull
 * of the n_R · n_W pairwise differences. O(n_R n_W log(n_R n_W)) — fine for a
 * test, wasteful in a planner, and exactly what the star algorithm must equal.
 */
export function minkowskiBruteForce(robot: Polygon, theta: number, obstacle: Polygon): Polygon {
  const R = rotatePolygon(toCcw(robot), theta);
  const pts: Point2[] = [];
  for (const o of obstacle) for (const r of R) pts.push({ x: o.x - r.x, y: o.y - r.y });
  return removeCollinear(convexHull(pts));
}

/** Minkowski *sum* A ⊕ B of two convex polygons — the same merge with nothing negated. */
export function minkowskiSum(a: Polygon, b: Polygon): Polygon {
  // A ⊕ B = A ⊖ (−B): reuse the star algorithm with the robot reflected.
  return starAlgorithm(b.map((p) => ({ x: -p.x, y: -p.y })), 0, a).polygon;
}

/**
 * Do two convex polygons, read as vertex loops, describe the same polygon?
 * Same vertex count after collinear removal and every vertex of one within
 * `tol` of some vertex of the other. The checks use this; it is deliberately
 * indifferent to where the loop starts.
 */
export function samePolygon(a: Polygon, b: Polygon, tol = 1e-9): boolean {
  const pa = removeCollinear(a, tol);
  const pb = removeCollinear(b, tol);
  if (pa.length !== pb.length) return false;
  for (const p of pa) {
    if (!pb.some((q) => Math.hypot(p.x - q.x, p.y - q.y) <= tol)) return false;
  }
  return Math.abs(Math.abs(polygonArea(pa)) - Math.abs(polygonArea(pb))) <= tol * (1 + Math.abs(polygonArea(pa)));
}

// ---------------------------------------------------------------------------
// F.4 — SE(2) as stacked slices
// ---------------------------------------------------------------------------

export interface Se2Slice {
  theta: number;
  polygon: Polygon;
}

/**
 * QO ⊂ SE(2) for a rotating polygon: n_θ slices QO(θ_k), θ_k uniform on
 * [−π, π). Each slice is exact; the stack is a picture (figure F.8), and the
 * θ-resolution is the same honesty item as the arm's raster — a path between
 * two slices is not certified by the slices.
 */
export function se2Slices(robot: Polygon, obstacle: Polygon, nTheta: number): Se2Slice[] {
  const out: Se2Slice[] = [];
  for (let k = 0; k < nTheta; k++) {
    const theta = -Math.PI + (TAU * k) / nTheta;
    out.push({ theta, polygon: starAlgorithm(robot, theta, obstacle).polygon });
  }
  return out;
}

/** The preset of the design doc: a right triangle robot and the unit square [1, 2]². */
export const STAR_PRESET = {
  robot: [
    { x: 0, y: 0 },
    { x: 0.2, y: 0 },
    { x: 0, y: 0.2 },
  ] as Polygon,
  obstacle: [
    { x: 1, y: 1 },
    { x: 2, y: 1 },
    { x: 2, y: 2 },
    { x: 1, y: 2 },
  ] as Polygon,
};
