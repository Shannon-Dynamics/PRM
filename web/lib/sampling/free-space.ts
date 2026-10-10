/**
 * `FreeSpace<P>` — the one question a sampling-based planner asks of the world.
 *
 * Choset §7 opens with the observation that made the field: sampling-based
 * planners "do not attempt to explicitly construct the boundaries of the
 * configuration space obstacles". They need exactly one predicate, "is q
 * free?", and the whole chapter's cost accounting is in units of that
 * predicate. So the adapter here does two things and nothing else: it narrows
 * Chapter 2's four-question `Collision<Q>` to `isFree`, and it *counts* — the
 * counter is the book's cost unit, the thing w11.1 shows separately from
 * local-planner calls and w12.2 compares across planners on one seed.
 *
 * The point type `P` is the manifold's (Chapter 5), not the checker's, because
 * the kd-tree and the steer work in `P`; the adapter owns the conversion.
 */

import type { Pose2 } from '../geom/se2';
import type { Collision, R2 } from '../collide/collision';
import { DiscChecker, ChainChecker, FootprintChecker } from '../collide/collision';
import { Scene } from '../collide/scene';
import type { Reach } from '../robots/reach';
import type { Hitch, HitchCfg } from '../robots/hitch';
import { Rd, type Box } from '../manifold/rd';

export interface FreeSpace<P> {
  isFree(q: P): boolean;
  /** How many times `isFree` has been asked. The book's cost unit. */
  readonly checks: number;
}

/** Wrap any predicate with the counter. */
export class CountingFreeSpace<P> implements FreeSpace<P> {
  checks = 0;

  constructor(
    private readonly pred: (q: P) => boolean,
    /** Optional clearance oracle (metres) — the medial-axis sampler needs one. */
    readonly clearance?: (q: P) => number,
  ) {}

  isFree(q: P): boolean {
    this.checks++;
    return this.pred(q);
  }

  reset(): void {
    this.checks = 0;
  }
}

/** Any Chapter 2 checker, through a conversion from the planner's point type. */
export function freeSpaceFromCollision<Q, P>(
  checker: Collision<Q>,
  toQ: (p: P) => Q,
): CountingFreeSpace<P> {
  return new CountingFreeSpace<P>(
    (p) => checker.isFree(toQ(p)),
    (p) => checker.distance(toQ(p)),
  );
}

/** Rusty as a disc: P = [x, y] on ℝ². */
export function discFreeSpace(scene: Scene, radius: number): CountingFreeSpace<number[]> {
  const checker = new DiscChecker(scene, radius);
  return freeSpaceFromCollision<R2, number[]>(checker, (p) => ({ x: p[0], y: p[1] }));
}

/** Reach's link chain: P = joint angles on Tⁿ. */
export function chainFreeSpace(scene: Scene, arm: Reach): CountingFreeSpace<number[]> {
  const checker = new ChainChecker(scene, arm);
  return freeSpaceFromCollision<readonly number[], number[]>(checker, (p) => p);
}

/** Hitch's rigid footprint, no trailer: P = Pose2 on SE(2). */
export function footprintFreeSpace(scene: Scene, car: Hitch): CountingFreeSpace<Pose2> {
  const checker = new FootprintChecker(scene, car);
  return freeSpaceFromCollision<HitchCfg, Pose2>(checker, (p) => ({ body: p, trailer: null }));
}

/* -------------------------------------------------------------------------- */
/* The micro-example world: a point robot among axis-aligned boxes            */
/* -------------------------------------------------------------------------- */

export interface AxisBox {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

/**
 * A point robot in a rectangle with rectangular C-obstacles — the world of
 * every hand-traceable example in Chapters 11–13 (the §3 micro-examples use
 * one wall, one block, or no obstacle at all). Collision is closed: touching
 * the boundary of a box counts.
 */
export class BoxWorld {
  readonly space: Rd;
  readonly free: CountingFreeSpace<number[]>;

  constructor(
    readonly bounds: AxisBox,
    readonly obstacles: readonly AxisBox[],
  ) {
    const box: Box = { lo: [bounds.minX, bounds.minY], hi: [bounds.maxX, bounds.maxY] };
    this.space = new Rd(2, box);
    this.free = new CountingFreeSpace<number[]>(
      (q) => this.contains(q) && !this.obstacles.some((o) => inBox(q, o)),
      (q) => this.clearance(q),
    );
  }

  contains(q: number[]): boolean {
    const b = this.bounds;
    return q[0] >= b.minX && q[0] <= b.maxX && q[1] >= b.minY && q[1] <= b.maxY;
  }

  /** Distance to the nearest obstacle or boundary (0 inside an obstacle). */
  clearance(q: number[]): number {
    const b = this.bounds;
    let d = Math.min(q[0] - b.minX, b.maxX - q[0], q[1] - b.minY, b.maxY - q[1]);
    for (const o of this.obstacles) {
      if (inBox(q, o)) return 0;
      const dx = Math.max(o.minX - q[0], 0, q[0] - o.maxX);
      const dy = Math.max(o.minY - q[1], 0, q[1] - o.maxY);
      d = Math.min(d, Math.hypot(dx, dy));
    }
    return Math.max(0, d);
  }

  /** Lebesgue measure of the free part, for the σ of Theorem 7.4.1 and the γ* of Chapter 13. */
  freeMeasure(): number {
    const b = this.bounds;
    let area = (b.maxX - b.minX) * (b.maxY - b.minY);
    for (const o of this.obstacles) {
      const w = Math.max(0, Math.min(o.maxX, b.maxX) - Math.max(o.minX, b.minX));
      const h = Math.max(0, Math.min(o.maxY, b.maxY) - Math.max(o.minY, b.minY));
      area -= w * h;
    }
    return area;
  }

  /** As a Chapter 2 `Scene`, so the same world can be drawn with `drawWorld`. */
  toScene(): Scene {
    const b = this.bounds;
    const walls = [
      { x1: b.minX, y1: b.minY, x2: b.maxX, y2: b.minY },
      { x1: b.maxX, y1: b.minY, x2: b.maxX, y2: b.maxY },
      { x1: b.maxX, y1: b.maxY, x2: b.minX, y2: b.maxY },
      { x1: b.minX, y1: b.maxY, x2: b.minX, y2: b.minY },
    ];
    const obstacles = this.obstacles.map((o, i) => ({
      name: `box ${i + 1}`,
      poly: [
        { x: o.minX, y: o.minY },
        { x: o.maxX, y: o.minY },
        { x: o.maxX, y: o.maxY },
        { x: o.minX, y: o.maxY },
      ],
    }));
    return new Scene(walls, obstacles, b);
  }
}

const inBox = (q: number[], o: AxisBox) => q[0] >= o.minX && q[0] <= o.maxX && q[1] >= o.minY && q[1] <= o.maxY;

/** Chapter 11 §3: 𝒲 = [0,10]², one wall [4.5, 5.5] × [0, 7] — the gap is above y = 7. */
export const SIX_SAMPLE_WORLD = new BoxWorld(
  { minX: 0, minY: 0, maxX: 10, maxY: 10 },
  [{ minX: 4.5, minY: 0, maxX: 5.5, maxY: 7 }],
);

/** Chapter 12 §3: 𝒬 = [0,10]², obstacle [3, 4] × [0, 3]. */
export const THREE_EXTENSION_WORLD = new BoxWorld(
  { minX: 0, minY: 0, maxX: 10, maxY: 10 },
  [{ minX: 3, minY: 0, maxX: 4, maxY: 3 }],
);

/** Chapter 13 §3: the empty plane (a generous box). */
export const EMPTY_PLANE = new BoxWorld({ minX: -10, minY: -10, maxX: 10, maxY: 10 }, []);
