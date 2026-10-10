/**
 * The `Collision` contract — the four questions a planner may ask of geometry,
 * and nothing else:
 *
 *   isFree(q)      is R(q) disjoint from the scene?                (intersects)
 *   distance(q)    min ‖a − b‖ over a ∈ R(q), b ∈ scene; 0 if not  (distance)
 *   witness(q)     a minimizing pair (a*, b*)                      (witness)
 *   edgeFree(a,b)  is the straight chart line from a to b free?    (swept)
 *
 * Every planner from Chapter 7 onward takes a `Collision<Q>` and a robot; it
 * never sees a polygon. The three implementations here cover the lab's three
 * robots: a disc for Rusty, a chain of segments (or capsules) for Reach, and a
 * list of rigid rectangles for Hitch.
 *
 * The swept query is *conservative sphere marching*: if the footprint is at
 * distance d from the scene and no point of it can move faster than L per unit
 * of chart parameter, then the whole interval [s, s + d/L] is free. Marching
 * with that step visits every configuration a dense sampler could catch, with
 * a number of queries proportional to the log of the clearance rather than to
 * the edge length. `edgeFreeDense` is the brute-force reference the checks
 * compare it against.
 */

import { normalizeAngle, type Pose2 } from '../geom/se2';
import type { Point2, Segment } from '../sim/world';
import type { Polygon, Witness } from './polygon';
import type { Scene } from './scene';
import type { Hitch, HitchCfg } from '../robots/hitch';
import type { Reach, Tn } from '../robots/reach';

export type { Witness } from './polygon';

export interface Collision<Q> {
  isFree(q: Q): boolean;
  /** Zero when `q` is in collision. */
  distance(q: Q): number;
  witness(q: Q): Witness | null;
  /** Rigid straight-line sweep in the chart from `a` to `b`, endpoints included. */
  edgeFree(a: Q, b: Q): boolean;
}

/**
 * Sphere marching over any configuration type, given the straight chart line
 * `interpolate(a, b, s)` and a Lipschitz bound `sweepBound(a, b)` on how far
 * any footprint point can travel over the whole edge.
 */
export abstract class SweptChecker<Q> implements Collision<Q> {
  abstract isFree(q: Q): boolean;
  abstract distance(q: Q): number;
  abstract witness(q: Q): Witness | null;
  abstract interpolate(a: Q, b: Q, s: number): Q;
  /** Max workspace displacement of any footprint point along the full edge a→b. */
  abstract sweepBound(a: Q, b: Q): number;

  /** Below this clearance we stop trusting the bound and step by `minStep`. */
  protected minClearance = 1e-9;
  protected minStep = 1e-4;

  edgeFree(a: Q, b: Q): boolean {
    const L = this.sweepBound(a, b);
    if (!this.isFree(a) || !this.isFree(b)) return false;
    if (L <= 0) return true;
    let s = 0;
    let guard = 0;
    while (s < 1 && guard++ < 100000) {
      const d = this.distance(this.interpolate(a, b, s));
      if (d <= 0) return false;
      // Everything within d of the current footprint is free, and the footprint
      // moves at most L per unit s, so s + d/L is the first parameter at which a
      // collision could possibly begin. Touching the obstacle exactly at that
      // parameter counts as a collision in the closed-set convention.
      const step = Math.max(d / L, this.minStep);
      s += step;
      if (d < this.minClearance) return false;
    }
    return true;
  }

  /** Dense sampling at chart spacing `step` — the brute-force reference. */
  edgeFreeDense(a: Q, b: Q, step: number): boolean {
    const L = this.sweepBound(a, b);
    const n = Math.max(2, Math.ceil(L / step));
    for (let i = 0; i <= n; i++) if (!this.isFree(this.interpolate(a, b, i / n))) return false;
    return true;
  }
}

// ---------------------------------------------------------------------------
// Rusty: a disc in R²
// ---------------------------------------------------------------------------

/** Rusty's configuration space: the position of the disc's center. */
export interface R2 {
  x: number;
  y: number;
}

/**
 * A disc of radius r against walls and obstacles. Derivation 3 of the chapter:
 * the disc collides iff D(center) < r, so the distance query is D − r and the
 * witness is the scene's nearest point and the center shifted r toward it.
 */
export class DiscChecker extends SweptChecker<R2> {
  constructor(
    readonly scene: Scene,
    readonly radius: number,
  ) {
    super();
  }

  isFree(q: R2): boolean {
    if (!this.scene.inBounds(q)) return false;
    return this.scene.distancePoint(q.x, q.y).dist > this.radius;
  }

  distance(q: R2): number {
    return Math.max(0, this.scene.distancePoint(q.x, q.y).dist - this.radius);
  }

  witness(q: R2): Witness | null {
    const w = this.scene.distancePoint(q.x, q.y);
    if (!Number.isFinite(w.dist)) return null;
    if (w.dist <= this.radius) return { onRobot: w.onObstacle, onObstacle: w.onObstacle, dist: 0 };
    const ux = (w.onObstacle.x - q.x) / w.dist;
    const uy = (w.onObstacle.y - q.y) / w.dist;
    return {
      onRobot: { x: q.x + ux * this.radius, y: q.y + uy * this.radius },
      onObstacle: w.onObstacle,
      dist: w.dist - this.radius,
    };
  }

  interpolate(a: R2, b: R2, s: number): R2 {
    return { x: a.x + s * (b.x - a.x), y: a.y + s * (b.y - a.y) };
  }

  sweepBound(a: R2, b: R2): number {
    return Math.hypot(b.x - a.x, b.y - a.y);
  }
}

// ---------------------------------------------------------------------------
// Reach: a chain of links on Tⁿ
// ---------------------------------------------------------------------------

/**
 * Reach's links as segments (link radius 0 — every worked number) or capsules
 * (link radius > 0 — the default Workbench) against the scene.
 *
 * Interpolation is the straight line in joint space *through the seam*: the
 * shorter arc on each S¹, which is what "straight chart line" means on a torus.
 */
export class ChainChecker extends SweptChecker<Tn> {
  constructor(
    readonly scene: Scene,
    readonly arm: Reach,
  ) {
    super();
  }

  private links(q: Tn): Segment[] {
    return this.arm.linkSegments(q);
  }

  isFree(q: Tn): boolean {
    const r = this.arm.linkRadius;
    for (const s of this.links(q)) {
      if (r <= 0) {
        if (this.scene.intersectsSegment(s)) return false;
      } else if (this.scene.distanceSegment(s).dist <= r) {
        return false;
      }
    }
    return true;
  }

  distance(q: Tn): number {
    const w = this.witness(q);
    return w ? w.dist : Infinity;
  }

  witness(q: Tn): Witness | null {
    let best: Witness | null = null;
    for (const s of this.links(q)) {
      const w = this.scene.distanceSegment(s);
      if (!best || w.dist < best.dist) best = w;
    }
    if (!best) return null;
    const r = this.arm.linkRadius;
    if (r <= 0) return best;
    if (best.dist <= r) return { onRobot: best.onObstacle, onObstacle: best.onObstacle, dist: 0 };
    // Push the witness from the link's axis out to the capsule's surface.
    const ux = (best.onObstacle.x - best.onRobot.x) / best.dist;
    const uy = (best.onObstacle.y - best.onRobot.y) / best.dist;
    return {
      onRobot: { x: best.onRobot.x + ux * r, y: best.onRobot.y + uy * r },
      onObstacle: best.onObstacle,
      dist: best.dist - r,
    };
  }

  interpolate(a: Tn, b: Tn, s: number): Tn {
    return a.map((ai, i) => normalizeAngle(ai + s * normalizeAngle(b[i] - ai)));
  }

  /**
   * A point on link k moves at most Σ_{i≤k} (reach beyond joint i) · |Δθ_i|.
   * Bounding every link by the full arm length gives L · Σ|Δθ_i|, with
   * L = Σ lengths: simple, and tight within a factor of two.
   */
  sweepBound(a: Tn, b: Tn): number {
    const L = this.arm.lengths.reduce((x, y) => x + y, 0);
    let sum = 0;
    for (let i = 0; i < a.length; i++) sum += Math.abs(normalizeAngle(b[i] - a[i]));
    return L * sum;
  }
}

// ---------------------------------------------------------------------------
// Hitch: rigid rectangles on SE(2) × S¹
// ---------------------------------------------------------------------------

/**
 * Hitch's chassis (and trailer) as rectangles against the scene. The chart is
 * (x, y, θ, ψ); interpolation is linear in position and shortest-arc in each
 * angle. The sweep bound uses the body's circumscribed radius: a point at most
 * R from the rear axle moves at most ‖Δp‖ + R·|Δθ| (and the trailer likewise).
 */
export class FootprintChecker extends SweptChecker<HitchCfg> {
  constructor(
    readonly scene: Scene,
    readonly car: Hitch,
  ) {
    super();
  }

  private parts(q: HitchCfg): Polygon[] {
    return this.car.footprint(q);
  }

  isFree(q: HitchCfg): boolean {
    if (!this.scene.inBounds(q.body)) return false;
    for (const poly of this.parts(q)) if (this.scene.intersectsPolygon(poly)) return false;
    return true;
  }

  distance(q: HitchCfg): number {
    const w = this.witness(q);
    return w ? w.dist : Infinity;
  }

  witness(q: HitchCfg): Witness | null {
    let best: Witness | null = null;
    for (const poly of this.parts(q)) {
      const w = this.scene.distancePolygon(poly);
      if (!best || w.dist < best.dist) best = w;
    }
    return best;
  }

  interpolate(a: HitchCfg, b: HitchCfg, s: number): HitchCfg {
    const body: Pose2 = {
      x: a.body.x + s * (b.body.x - a.body.x),
      y: a.body.y + s * (b.body.y - a.body.y),
      theta: normalizeAngle(a.body.theta + s * normalizeAngle(b.body.theta - a.body.theta)),
    };
    const trailer =
      a.trailer === null || b.trailer === null
        ? null
        : normalizeAngle(a.trailer + s * normalizeAngle(b.trailer - a.trailer));
    return { body, trailer };
  }

  sweepBound(a: HitchCfg, b: HitchCfg): number {
    const dp = Math.hypot(b.body.x - a.body.x, b.body.y - a.body.y);
    const dth = Math.abs(normalizeAngle(b.body.theta - a.body.theta));
    let bound = dp + this.car.bodyRadius() * dth;
    if (a.trailer !== null && b.trailer !== null) {
      // The trailer hangs off the rear axle: its points also swing with ψ.
      const dpsi = Math.abs(normalizeAngle(b.trailer - a.trailer));
      bound = Math.max(bound, dp + this.car.trailerRadius() * dpsi);
    }
    return bound;
  }
}

/** Convenience for widgets: a footprint's bounding box. */
export function pointsBounds(pts: Point2[]): { minX: number; minY: number; maxX: number; maxY: number } {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const p of pts) {
    minX = Math.min(minX, p.x);
    maxX = Math.max(maxX, p.x);
    minY = Math.min(minY, p.y);
    maxY = Math.max(maxY, p.y);
  }
  return { minX, minY, maxX, maxY };
}
