/**
 * A `Scene` is what a collision checker is checked *against*: the walls and
 * filled obstacles of a world, flattened into one list each and answering the
 * four primitive questions for a point, a segment, or a polygon footprint.
 *
 * On the Rust side this is `collide::Scene` wrapping a `parry2d::Qbvh` and
 * the `query::{distance, closest_points, intersection_test, cast_shapes}`
 * family. Here the broad phase is a linear scan — the worlds have at most a
 * few dozen primitives, and a linear scan over forty segments is cheaper than
 * maintaining a tree would be.
 *
 * Witness convention: `onRobot` is on the footprint, `onObstacle` on the scene.
 */

import { distanceToSegment, type Bounds, type Point2, type Segment, type World } from '../sim/world';
import type { NamedPolygon, PolyWorld } from '../robots/worlds';
import {
  closestPointOnSegment,
  distancePolygonPolygon,
  distancePolygonSegment,
  distanceSegmentPolygon,
  distanceSegmentSegment,
  polygonEdges,
  polygonsIntersect,
  segmentIntersectsPolygon,
  segmentsTouch,
  pointInPolygon,
  type Polygon,
  type Witness,
} from './polygon';

const EPS = 1e-12;

export class Scene {
  readonly walls: readonly Segment[];
  readonly obstacles: readonly NamedPolygon[];
  readonly bounds: Bounds;
  /** Walls plus every obstacle edge — what a range sensor's rays can hit. */
  private readonly segs: readonly Segment[];

  constructor(walls: readonly Segment[], obstacles: readonly NamedPolygon[], bounds: Bounds) {
    this.walls = walls;
    this.obstacles = obstacles;
    this.bounds = bounds;
    this.segs = [...walls, ...obstacles.flatMap((o) => polygonEdges(o.poly))];
  }

  /** Apartment walls become thin segment obstacles; Workbench/Lot polygons stay filled. */
  static fromWorld(w: World | PolyWorld): Scene {
    const obstacles = (w as PolyWorld).obstacles ?? [];
    return new Scene(w.walls, obstacles, w.bounds);
  }

  /** A scene with one more obstacle — used for scratch scenes in the worked example. */
  withObstacle(name: string, poly: Polygon): Scene {
    return new Scene(this.walls, [...this.obstacles, { name, poly }], this.bounds);
  }

  /** An empty scene with only the given walls (the worked example's single wall). */
  static ofWalls(walls: Segment[], bounds: Bounds): Scene {
    return new Scene(walls, [], bounds);
  }

  /** All ray-castable segments. */
  segments(): readonly Segment[] {
    return this.segs;
  }

  // -- point -----------------------------------------------------------------

  /** Inside any filled obstacle (boundary included)? Walls have no interior. */
  pointInObstacle(x: number, y: number): boolean {
    for (const o of this.obstacles) if (pointInPolygon(x, y, o.poly)) return true;
    return false;
  }

  /** D(x): distance from a point to the nearest wall or obstacle; 0 inside an obstacle. */
  distancePoint(x: number, y: number): Witness {
    const p = { x, y };
    let best: Witness = { onRobot: p, onObstacle: p, dist: Infinity };
    for (const o of this.obstacles) {
      if (pointInPolygon(x, y, o.poly)) return { onRobot: p, onObstacle: p, dist: 0 };
    }
    for (const s of this.segs) {
      const c = closestPointOnSegment(x, y, s);
      const d = Math.hypot(c.x - x, c.y - y);
      if (d < best.dist) best = { onRobot: p, onObstacle: c, dist: d };
    }
    return best;
  }

  // -- segment (a zero-thickness link) --------------------------------------

  intersectsSegment(s: Segment): boolean {
    for (const w of this.walls) if (segmentsTouch(s, w)) return true;
    for (const o of this.obstacles) if (segmentIntersectsPolygon(s, o.poly)) return true;
    return false;
  }

  distanceSegment(s: Segment): Witness {
    let best: Witness = { onRobot: { x: s.x1, y: s.y1 }, onObstacle: { x: s.x1, y: s.y1 }, dist: Infinity };
    for (const o of this.obstacles) {
      const w = distanceSegmentPolygon(s, o.poly);
      if (w.dist < best.dist) best = w;
      if (best.dist === 0) return best;
    }
    for (const w of this.walls) {
      const d = distanceSegmentSegment(s, w);
      if (d.dist < best.dist) best = d;
      if (best.dist === 0) return best;
    }
    return best;
  }

  // -- polygon (a rigid body) ------------------------------------------------

  intersectsPolygon(poly: Polygon): boolean {
    for (const o of this.obstacles) if (polygonsIntersect(poly, o.poly)) return true;
    for (const w of this.walls) if (segmentIntersectsPolygon(w, poly)) return true;
    return false;
  }

  distancePolygon(poly: Polygon): Witness {
    let best: Witness = { onRobot: poly[0], onObstacle: poly[0], dist: Infinity };
    for (const o of this.obstacles) {
      const w = distancePolygonPolygon(poly, o.poly);
      if (w.dist < best.dist) best = w;
      if (best.dist === 0) return best;
    }
    for (const w of this.walls) {
      const d = distancePolygonSegment(poly, w);
      if (d.dist < best.dist) best = d;
      if (best.dist === 0) return best;
    }
    return best;
  }

  // -- rays ------------------------------------------------------------------

  /**
   * Exact first-hit distance along a ray, over walls *and* obstacle edges —
   * the ported `rayCast` only knows walls. Returns `maxRange` on a miss.
   */
  raycast(ox: number, oy: number, angle: number, maxRange: number): number {
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);
    let best = maxRange;
    for (const s of this.segs) {
      const sx = s.x2 - s.x1;
      const sy = s.y2 - s.y1;
      const denom = dx * sy - dy * sx;
      if (Math.abs(denom) < EPS) continue;
      const ex = s.x1 - ox;
      const ey = s.y1 - oy;
      const t = (ex * sy - ey * sx) / denom;
      const u = (ex * dy - ey * dx) / denom;
      if (t >= 0 && t < best && u >= 0 && u <= 1) best = t;
    }
    return best;
  }

  /** Inside the scene's bounding box? */
  inBounds(p: Point2): boolean {
    const b = this.bounds;
    return p.x >= b.minX && p.x <= b.maxX && p.y >= b.minY && p.y <= b.maxY;
  }

  /** Wall distance only (ignores filled obstacles) — matches the ported `distanceToWalls`. */
  wallDistance(x: number, y: number): number {
    let best = Infinity;
    for (const w of this.walls) best = Math.min(best, distanceToSegment(x, y, w));
    return best;
  }
}
