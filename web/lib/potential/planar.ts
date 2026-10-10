/**
 * Distance queries in the plane — the three ways Choset §4.3 computes d_i(q)
 * for a point robot, each as a `DistanceQuery`:
 *
 *   PlanarObstacles   exact geometry: discs, polygons, wall segments, with the
 *                     closest point c and ∇d_i = (q − c)/‖q − c‖ (eq. 4.7). The
 *                     Chapter 2 polygon utilities do the work; nothing here is
 *                     re-derived.
 *   SensorDistance    §4.3.1: the local minima of ρ(q, θ) over the bearings of a
 *                     Chapter 3 range scan, one per visible obstacle, with
 *                     ∇d_i = −(cos θ*, sin θ*). Sensor-only — no polygons.
 *   GridDistance      §4.3.2: the brushfire label at the robot's cell, read as
 *                     (label − 1)·cellSize, with the gradient pointing from the
 *                     back-pointer cell to the robot's cell.
 */

import { closestPointOnSegment, distancePointPolygon, pointInPolygon, type Polygon } from '../collide/polygon';
import type { Scene } from '../collide/scene';
import { rangeScan, type Scan } from '../bugs/scan';
import type { Point2, Segment, World } from '../sim/world';
import { cellAt, cellCenter, cellIJ, type OccupancyGrid } from '../search/grid';
import type { Bounds } from '../sim/world';
import type { DistGrid } from './brushfire';
import type { DistanceQuery, ObstacleDistance, Vec } from './field';

export type PlanarObstacle =
  | { kind: 'disc'; c: Point2; r: number }
  | { kind: 'polygon'; poly: Polygon }
  | { kind: 'segment'; s: Segment };

/** Signed distance to a disc: negative inside; the gradient is radial either way. */
function discDistance(q: Point2, c: Point2, r: number): { d: number; grad: Vec; closest: Vec } {
  const dx = q.x - c.x;
  const dy = q.y - c.y;
  const rho = Math.hypot(dx, dy);
  if (rho < 1e-12) return { d: -r, grad: [1, 0], closest: [c.x + r, c.y] };
  const ux = dx / rho;
  const uy = dy / rho;
  return { d: rho - r, grad: [ux, uy], closest: [c.x + r * ux, c.y + r * uy] };
}

/**
 * Exact planar distances with witnesses. Polygons report d = 0 inside (their
 * gradient then points from the polygon's centroid, so a descent that has
 * wandered in is pushed out rather than left with a zero vector).
 */
export class PlanarObstacles implements DistanceQuery {
  readonly dim = 2;

  constructor(readonly obstacles: PlanarObstacle[]) {}

  static fromScene(scene: Scene): PlanarObstacles {
    const obs: PlanarObstacle[] = [];
    for (const o of scene.obstacles) obs.push({ kind: 'polygon', poly: o.poly });
    for (const s of scene.walls) obs.push({ kind: 'segment', s });
    return new PlanarObstacles(obs);
  }

  inside(q: Point2): boolean {
    for (const o of this.obstacles) {
      if (o.kind === 'disc' && Math.hypot(q.x - o.c.x, q.y - o.c.y) <= o.r) return true;
      if (o.kind === 'polygon' && pointInPolygon(q.x, q.y, o.poly)) return true;
    }
    return false;
  }

  /** D(q) = min_i d_i(q). */
  nearest(q: Vec): ObstacleDistance {
    let best: ObstacleDistance | null = null;
    for (const o of this.distances(q)) if (!best || o.d < best.d) best = o;
    return best ?? { d: Infinity, grad: [0, 0], index: -1 };
  }

  distances(q: Vec): ObstacleDistance[] {
    const p = { x: q[0], y: q[1] };
    const out: ObstacleDistance[] = [];
    this.obstacles.forEach((o, index) => {
      if (o.kind === 'disc') {
        const r = discDistance(p, o.c, o.r);
        out.push({ ...r, index });
      } else if (o.kind === 'segment') {
        const c = closestPointOnSegment(p.x, p.y, o.s);
        const d = Math.hypot(p.x - c.x, p.y - c.y);
        const grad: Vec = d > 1e-12 ? [(p.x - c.x) / d, (p.y - c.y) / d] : [0, 0];
        out.push({ d, grad, closest: [c.x, c.y], index });
      } else {
        const w = distancePointPolygon(p.x, p.y, o.poly);
        let grad: Vec;
        if (w.dist > 1e-12) {
          grad = [(p.x - w.onObstacle.x) / w.dist, (p.y - w.onObstacle.y) / w.dist];
        } else {
          // Inside: push away from the centroid so the descent has a direction out.
          let cx = 0;
          let cy = 0;
          for (const v of o.poly) {
            cx += v.x;
            cy += v.y;
          }
          cx /= o.poly.length;
          cy /= o.poly.length;
          const n = Math.hypot(p.x - cx, p.y - cy) || 1;
          grad = [(p.x - cx) / n, (p.y - cy) / n];
        }
        out.push({ d: w.dist, grad, closest: [w.onObstacle.x, w.onObstacle.y], index });
      }
    });
    return out;
  }
}

/**
 * The horseshoe of Choset Fig. 4.10 as a polygon: a U open toward −x, so a
 * bead descending from the left toward a goal behind it rolls into the mouth
 * and stops where the base's repulsion cancels the goal's attraction.
 */
export function horseshoe(center: Point2, width = 1.6, depth = 1.2, thickness = 0.3, opening: 'left' | 'right' = 'left'): Polygon {
  const s = opening === 'left' ? 1 : -1;
  const hw = width / 2;
  // Outer rectangle minus the mouth: a U traced counter-clockwise.
  const pts: Point2[] = [
    { x: -depth / 2, y: -hw },
    { x: depth / 2, y: -hw },
    { x: depth / 2, y: hw },
    { x: -depth / 2, y: hw },
    { x: -depth / 2, y: hw - thickness },
    { x: depth / 2 - thickness, y: hw - thickness },
    { x: depth / 2 - thickness, y: -hw + thickness },
    { x: -depth / 2, y: -hw + thickness },
  ];
  return pts.map((p) => ({ x: center.x + s * p.x, y: center.y + p.y }));
}

/**
 * The same horseshoe as three convex rectangles — base and two arms. Choset
 * §4.1 sums one repulsive term *per convex obstacle* so that ∇U never jumps
 * when the closest point switches; a single nonconvex polygon has a d_i that
 * is nonsmooth along its axis of symmetry (two closest points tie), and a bead
 * there chatters instead of settling. The Potential Well offers both.
 */
export function horseshoePieces(center: Point2, width = 1.6, depth = 1.2, thickness = 0.3, opening: 'left' | 'right' = 'left'): Polygon[] {
  const s = opening === 'left' ? 1 : -1;
  const hw = width / 2;
  const box = (x0: number, y0: number, x1: number, y1: number): Polygon =>
    [{ x: x0, y: y0 }, { x: x1, y: y0 }, { x: x1, y: y1 }, { x: x0, y: y1 }].map((p) => ({ x: center.x + s * p.x, y: center.y + p.y }));
  return [
    box(depth / 2 - thickness, -hw, depth / 2, hw), // base
    box(-depth / 2, hw - thickness, depth / 2 - thickness, hw), // upper arm
    box(-depth / 2, -hw, depth / 2 - thickness, -hw + thickness), // lower arm
  ];
}

/* -------------------------------------------------------------------------- */
/* §4.3.1 — distance from a range scan                                         */
/* -------------------------------------------------------------------------- */

/**
 * The local minima of ρ(q, θ) over θ: each is a d_i(q) with ∇d_i pointing back
 * along the ray (Choset Fig. 4.7). A plateau of equal readings counts once, at
 * its first ray. Rays at the saturation range are not minima.
 */
export function scanLocalMinima(scan: Scan): ObstacleDistance[] {
  const n = scan.ranges.length;
  const out: ObstacleDistance[] = [];
  for (let k = 0; k < n; k++) {
    const r = scan.ranges[k];
    if (!Number.isFinite(r)) continue;
    const prev = scan.ranges[(k - 1 + n) % n];
    const next = scan.ranges[(k + 1) % n];
    // A strict local minimum, or the first ray of a plateau that is one.
    const isMin = r <= prev && r <= next && (r < prev || r < next);
    if (!isMin || r === prev) continue;
    const th = scan.angles[k];
    out.push({ d: r, grad: [-Math.cos(th), -Math.sin(th)], closest: [scan.x.x + r * Math.cos(th), scan.x.y + r * Math.sin(th)], index: out.length });
  }
  return out;
}

/** A `DistanceQuery` that only ever looks at a range scan — Rusty's first *online* potential. */
export class SensorDistance implements DistanceQuery {
  readonly dim = 2;
  /** The most recent scan, for drawing. */
  lastScan: Scan | null = null;

  constructor(
    readonly world: World,
    readonly nRays = 72,
    readonly maxRange = 3,
  ) {}

  distances(q: Vec): ObstacleDistance[] {
    const scan = rangeScan(this.world, q[0], q[1], this.nRays, this.maxRange);
    this.lastScan = scan;
    return scanLocalMinima(scan);
  }
}

/* -------------------------------------------------------------------------- */
/* §4.3.2 — distance from a brushfire grid                                     */
/* -------------------------------------------------------------------------- */

/**
 * Distance read off a brushfire grid: (label − 1)·cellSize at the robot's cell,
 * with ∇D pointing from the back-pointer cell toward the robot's cell (the grid
 * version of "away from the nearest obstacle"). One obstacle only — the
 * nearest — so this is the D(q) form of U_rep, with the oscillation Choset
 * warns about available for inspection.
 */
export class GridDistance implements DistanceQuery {
  readonly dim = 2;

  constructor(readonly dist: DistGrid) {}

  get grid(): OccupancyGrid {
    return this.dist.grid;
  }

  distances(q: Vec): ObstacleDistance[] {
    const g = this.dist.grid;
    const cell = cellAt(g, q[0], q[1]);
    const lab = this.dist.label[cell];
    if (lab <= 0) return [];
    const d = (lab - 1) * g.cellSize;
    const back = this.dist.back[cell];
    let grad: Vec = [0, 0];
    if (back >= 0) {
      const [ci, cj] = cellIJ(g, cell);
      const [bi, bj] = cellIJ(g, back);
      const n = Math.hypot(ci - bi, cj - bj) || 1;
      grad = [(ci - bi) / n, (cj - bj) / n];
    }
    const c = back >= 0 ? cellCenter(g, back) : cellCenter(g, cell);
    return [{ d, grad, closest: [c.x, c.y], index: 0 }];
  }
}

/* -------------------------------------------------------------------------- */
/* Rasterizing planar obstacles for brushfire / wave-front                      */
/* -------------------------------------------------------------------------- */

/**
 * An occupancy grid over `bounds` for a point robot (or a disc of radius
 * `inflation`, the Chapter 4 reduction): a cell is occupied when its centre is
 * within `inflation` of any obstacle. Cells outside the bounds do not exist, so
 * the border is free unless the caller adds walls — the wave-front then runs
 * to the edge, which is what a square table without a rim does.
 */
export function rasterizeObstacles(obs: PlanarObstacles, bounds: Bounds, cellSize: number, inflation = 0): OccupancyGrid {
  const nx = Math.max(1, Math.round((bounds.maxX - bounds.minX) / cellSize));
  const ny = Math.max(1, Math.round((bounds.maxY - bounds.minY) / cellSize));
  const occupied = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++) {
    for (let i = 0; i < nx; i++) {
      const x = bounds.minX + (i + 0.5) * cellSize;
      const y = bounds.minY + (j + 0.5) * cellSize;
      if (obs.inside({ x, y }) || obs.nearest([x, y]).d <= inflation) occupied[j * nx + i] = 1;
    }
  }
  return { nx, ny, cellSize, origin: { x: bounds.minX, y: bounds.minY }, occupied };
}
