/**
 * Composite configuration spaces — Choset §7.5.2 "Centralized Planning".
 *
 * Two robots have one configuration space: the Cartesian product of theirs.
 * Chapter 5's `Product<A, B>` already *is* that space for two factors; this
 * file adds the m-ary form `CompositeSpace<P>` (m bodies of one kind — three
 * Rustys — so a widget can slide the body count), and the one new thing the
 * product needs that no single-robot chapter had: a free-space predicate that
 * asks two kinds of question. Each body against the world (Chapter 2's
 * `Collision`, unchanged) and then every *pair* of bodies against each other.
 * The pairwise query is the same geometric primitive as the world query — a
 * disc against a disc is Chapter 4's Minkowski argument with both bodies
 * moving — so `MultiBody` keeps two counters: how much of the bill is the
 * world, how much is the pairs. The chapter's lab reports both.
 *
 * Nothing about the planners changes. `CompositeSpace` is a `Chart`, so the
 * Chapter 11 kd-tree, the straight-line `Steer`, `Prm`, `Rrt`, `RrtConnect`
 * and `RrtStar` all run on it with no edits — only the dimension, and with it
 * Theorem 7.4.1's sample count (1/ρ)^d, has grown.
 */

import type { Rng } from '../prob/rng';
import type { Chart } from '../manifold/manifold';
import { Rd } from '../manifold/rd';
import { Scene } from '../collide/scene';
import { DiscChecker, type Collision, type R2 } from '../collide/collision';
import type { FreeSpace } from '../sampling/free-space';
import { checkSchedule, type EdgeCheck } from '../sampling/steer';

/* -------------------------------------------------------------------------- */
/* The product of m copies of one chart                                        */
/* -------------------------------------------------------------------------- */

/**
 * 𝒬 = 𝒬₁ × ⋯ × 𝒬ₘ for m bodies whose spaces share a chart type. A point is the
 * array of body configurations. The metric is the ℓ² combination of the
 * factors' metrics — the same choice as Chapter 5's `Product`, and for the
 * same reason: Theorem 7.4.1's balls stay balls, and the kd-tree's box bounds
 * stay valid. Interpolation is componentwise, which is exactly why Chapter 11's
 * straight-line local planner moves every robot at once along its own geodesic.
 */
export class CompositeSpace<P> implements Chart<P[]> {
  readonly name: string;
  readonly dim: number;
  readonly periodic: readonly boolean[];
  readonly period: number;

  constructor(readonly factors: readonly Chart<P>[]) {
    if (factors.length === 0) throw new Error('CompositeSpace: at least one body');
    this.name = factors.map((f) => f.name).join(' × ');
    this.dim = factors.reduce((s, f) => s + f.dim, 0);
    this.periodic = factors.flatMap((f) => [...f.periodic]);
    this.period = factors[0].period;
    for (const f of factors) if (f.period !== this.period) throw new Error('CompositeSpace: factors must share a period');
  }

  get bodies(): number {
    return this.factors.length;
  }

  dist(a: P[], b: P[]): number {
    let s = 0;
    for (let i = 0; i < this.factors.length; i++) {
      const d = this.factors[i].dist(a[i], b[i]);
      s += d * d;
    }
    return Math.sqrt(s);
  }

  interpolate(a: P[], b: P[], t: number): P[] {
    return this.factors.map((f, i) => f.interpolate(a[i], b[i], t));
  }

  sample(rng: Rng): P[] {
    return this.factors.map((f) => f.sample(rng));
  }

  wrap(p: P[]): P[] {
    return this.factors.map((f, i) => f.wrap(p[i]));
  }

  equals(a: P[], b: P[], tol = 1e-9): boolean {
    return this.factors.every((f, i) => f.equals(a[i], b[i], tol));
  }

  coords(p: P[]): number[] {
    return this.factors.flatMap((f, i) => f.coords(p[i]));
  }

  fromCoords(c: number[]): P[] {
    const out: P[] = [];
    let k = 0;
    for (const f of this.factors) {
      out.push(f.fromCoords(c.slice(k, k + f.dim)));
      k += f.dim;
    }
    return out;
  }

  /** The projection π_i: the i-th body's configuration. */
  project(q: P[], i: number): P {
    return q[i];
  }
}

/** m discs translating in a box: (ℝ²)ᵐ, the space of every multi-Rusty example in the chapter. */
export function discsSpace(m: number, bounds: { minX: number; minY: number; maxX: number; maxY: number }): CompositeSpace<number[]> {
  const r2 = new Rd(2, { lo: [bounds.minX, bounds.minY], hi: [bounds.maxX, bounds.maxY] });
  return new CompositeSpace<number[]>(new Array<Chart<number[]>>(m).fill(r2));
}

/* -------------------------------------------------------------------------- */
/* The robot–robot primitive                                                   */
/* -------------------------------------------------------------------------- */

export interface DiscPair {
  /** Do the closed discs overlap? Touching counts, matching Chapter 2's closed-set convention. */
  collide: boolean;
  /** ‖p₁ − p₂‖. */
  dist: number;
  /** r₁ + r₂ − ‖p₁ − p₂‖ when colliding, else 0. */
  penetration: number;
}

/**
 * Two discs overlap iff their center distance is below the sum of their
 * radii — Chapter 4's disc C-obstacle, read with *both* bodies moving: the set
 * {q : ‖p₁ − p₂‖ < r₁ + r₂} depends on q only through p₁ − p₂, which is why
 * 𝒬𝒪₁₂ is a cylinder over a ball in the difference coordinate (Derivation 1).
 * On the Rust side this is one `parry2d::query::distance` call, the same one
 * the world check makes; here it is a hypot.
 */
export function discsCollide(p1: readonly number[], r1: number, p2: readonly number[], r2: number): DiscPair {
  const dist = Math.hypot(p1[0] - p2[0], p1[1] - p2[1]);
  const sum = r1 + r2;
  return { collide: dist <= sum, dist, penetration: dist <= sum ? sum - dist : 0 };
}

/* -------------------------------------------------------------------------- */
/* MultiBody: a FreeSpace over the composite                                   */
/* -------------------------------------------------------------------------- */

/** A disc in the workspace, as the pairwise checker sees a body. */
export interface BodyDisc {
  x: number;
  y: number;
  r: number;
}

/**
 * One body of a composite: its Chapter 2 checker against the world and the
 * discs that stand in for its footprint in the pairwise test. A Rusty is one
 * disc; a chain or a car would be a short list of discs (a conservative cover),
 * which is how `parry2d` would see it too before the exact narrow phase.
 */
export interface Body<P> {
  name: string;
  world: Collision<P>;
  footprint(q: P): BodyDisc[];
}

/**
 * 𝒬_free of m bodies: ⋂ᵢ πᵢ⁻¹(𝒬_free,i) \ ⋃ᵢ<ⱼ 𝒬𝒪ᵢⱼ. World checks first — they
 * are the cheaper rejection and most random composite samples die there —
 * then every pair. The two counters are the chapter's accounting: the pair
 * term grows like m², and it is the part no single-robot planner ever paid.
 */
export class MultiBody<P> implements FreeSpace<P[]> {
  checks = 0;
  worldChecks = 0;
  pairChecks = 0;
  /** How many `isFree` calls failed on a pair after every body passed its own check — the hook, counted. */
  pairOnlyRejections = 0;

  constructor(readonly bodies: readonly Body<P>[]) {}

  get m(): number {
    return this.bodies.length;
  }

  /** Each body against the world only — what "plan each robot alone" checks. */
  individuallyFree(q: readonly P[]): boolean {
    for (let i = 0; i < this.bodies.length; i++) {
      this.worldChecks++;
      if (!this.bodies[i].world.isFree(q[i])) return false;
    }
    return true;
  }

  /** The pairwise term alone: is any pair of bodies overlapping? */
  pairwiseFree(q: readonly P[]): boolean {
    const discs = this.bodies.map((b, i) => b.footprint(q[i]));
    for (let i = 0; i < discs.length; i++) {
      for (let j = i + 1; j < discs.length; j++) {
        this.pairChecks++;
        for (const a of discs[i]) for (const b of discs[j]) if (discsCollide([a.x, a.y], a.r, [b.x, b.y], b.r).collide) return false;
      }
    }
    return true;
  }

  isFree(q: P[]): boolean {
    this.checks++;
    if (!this.individuallyFree(q)) return false;
    if (!this.pairwiseFree(q)) {
      this.pairOnlyRejections++;
      return false;
    }
    return true;
  }

  /** Which pair collides first at q, or null — for the widget's amber flash. */
  firstCollidingPair(q: readonly P[]): [number, number] | null {
    const discs = this.bodies.map((b, i) => b.footprint(q[i]));
    for (let i = 0; i < discs.length; i++) {
      for (let j = i + 1; j < discs.length; j++) {
        for (const a of discs[i]) for (const b of discs[j]) if (discsCollide([a.x, a.y], a.r, [b.x, b.y], b.r).collide) return [i, j];
      }
    }
    return null;
  }

  reset(): void {
    this.checks = 0;
    this.worldChecks = 0;
    this.pairChecks = 0;
    this.pairOnlyRejections = 0;
  }
}

/** A disc robot of radius r as a `Body` over P = [x, y]. */
export function discBody(name: string, scene: Scene, r: number): Body<number[]> {
  const checker = new DiscChecker(scene, r);
  const asR2 = (q: readonly number[]): R2 => ({ x: q[0], y: q[1] });
  return {
    name,
    world: {
      isFree: (q) => checker.isFree(asR2(q)),
      distance: (q) => checker.distance(asR2(q)),
      witness: (q) => checker.witness(asR2(q)),
      edgeFree: (a, b) => checker.edgeFree(asR2(a), asR2(b)),
    },
    footprint: (q) => [{ x: q[0], y: q[1], r }],
  };
}

/* -------------------------------------------------------------------------- */
/* The corridor of the micro-example, and the corridor with an alcove          */
/* -------------------------------------------------------------------------- */

export interface Corridor {
  /** Corridor length along x. */
  length: number;
  /** Corridor height (walls at y = 0 and y = height). */
  height: number;
  /** Alcove: a notch of this width and depth cut into the north wall, centered at x = alcoveX; null for a plain corridor. */
  alcove: { x: number; width: number; depth: number } | null;
  scene: Scene;
  bounds: { minX: number; minY: number; maxX: number; maxY: number };
}

/**
 * The design doc's corridor: walls at y = 0 and y = height, open ends, and
 * optionally one alcove in the north wall — the only place a disc can step
 * aside. Walls are segments (Chapter 2's `Scene.ofWalls`), so a disc of radius
 * r is free iff its center is farther than r from every wall.
 */
export function corridor(length: number, height: number, alcove: Corridor['alcove'] = null): Corridor {
  const walls = [{ x1: 0, y1: 0, x2: length, y2: 0 }];
  let maxY = height;
  if (alcove) {
    const x0 = alcove.x - alcove.width / 2;
    const x1 = alcove.x + alcove.width / 2;
    const top = height + alcove.depth;
    walls.push({ x1: 0, y1: height, x2: x0, y2: height });
    walls.push({ x1: x0, y1: height, x2: x0, y2: top });
    walls.push({ x1: x0, y1: top, x2: x1, y2: top });
    walls.push({ x1: x1, y1: top, x2: x1, y2: height });
    walls.push({ x1: x1, y1: height, x2: length, y2: height });
    maxY = top;
  } else {
    walls.push({ x1: 0, y1: height, x2: length, y2: height });
  }
  // The open ends are walls too, so a disc cannot leave the corridor sideways.
  walls.push({ x1: 0, y1: 0, x2: 0, y2: height });
  walls.push({ x1: length, y1: 0, x2: length, y2: height });
  const bounds = { minX: 0, minY: 0, maxX: length, maxY };
  return { length, height, alcove, scene: Scene.ofWalls(walls, bounds), bounds };
}

/** Two discs of radius r in a corridor: the composite space (ℝ²)² and its MultiBody. */
export function twoDiscLab(c: Corridor, r: number): { space: CompositeSpace<number[]>; free: MultiBody<number[]>; r: number } {
  const space = discsSpace(2, c.bounds);
  const free = new MultiBody<number[]>([discBody('robot 1', c.scene, r), discBody('robot 2', c.scene, r)]);
  return { space, free, r };
}

/** m discs of radius r in a corridor. */
export function discsLab(c: Corridor, r: number, m: number): { space: CompositeSpace<number[]>; free: MultiBody<number[]>; r: number } {
  const space = discsSpace(m, c.bounds);
  const free = new MultiBody<number[]>(Array.from({ length: m }, (_, i) => discBody(`robot ${i + 1}`, c.scene, r)));
  return { space, free, r };
}

/* -------------------------------------------------------------------------- */
/* The micro-example (design doc §3), as numbers a check can pin               */
/* -------------------------------------------------------------------------- */

export interface TwoDiscWorked {
  /** ‖p₁ − p₂‖ at q = (1.0, 1.0, 1.8, 1.4): 0.8944. */
  distQ: number;
  penetrationQ: number;
  collideQ: boolean;
  individuallyFreeQ: boolean;
  /** ‖p₁ − p₂‖ at q′ = (1.0, 1.0, 2.0, 1.4): 1.0770. */
  distQPrime: number;
  collideQPrime: boolean;
  /** Where the straight edge q′ → q enters 𝒬𝒪₁₂: x₂ = 1 + √0.84 = 1.9165. */
  boundaryX2: number;
  /** The subdivision schedule's first interior check: x₂ = 1.9, ‖p₁ − p₂‖ = 0.9849 → rejected. */
  firstCheckX2: number;
  firstCheckDist: number;
  firstCheckRejects: boolean;
  /** Robot 2's slice at x₂ = 1.0 with robot 1 fixed at (1, 1): fraction of its own free band blocked — 1.0. */
  sliceBlockedFraction: number;
}

/** The corridor y ∈ [0, 2] of the micro-example, long enough that the ends do not matter. */
export const MICRO_CORRIDOR = corridor(6, 2);
export const MICRO_RADIUS = 0.5;
export const MICRO_Q: number[][] = [[1.0, 1.0], [1.8, 1.4]];
export const MICRO_Q_PRIME: number[][] = [[1.0, 1.0], [2.0, 1.4]];

/**
 * Every number the prose prints, produced by the same code the widget runs.
 * The edge check uses Chapter 11's subdivision schedule at step 0.1 on an
 * edge of length 0.2: exactly one interior check, the midpoint, x₂ = 1.9.
 * (`StraightLine.steer` tests the far endpoint q first and would already
 * reject there; the midpoint is the first *interior* check of the schedule.)
 */
export function twoDiscWorked(check: EdgeCheck = { kind: 'subdivision', step: 0.1 }): TwoDiscWorked {
  const lab = twoDiscLab(MICRO_CORRIDOR, MICRO_RADIUS);
  const r = MICRO_RADIUS;
  const q = MICRO_Q;
  const qp = MICRO_Q_PRIME;
  const dq = discsCollide(q[0], r, q[1], r);
  const dqp = discsCollide(qp[0], r, qp[1], r);
  // Boundary along the edge: (x₂ − 1)² + 0.4² = 1 ⇒ x₂ = 1 + √(1 − 0.16).
  const dy = q[1][1] - q[0][1];
  const boundaryX2 = q[0][0] + Math.sqrt((2 * r) ** 2 - dy * dy);
  const length = lab.space.dist(qp, q);
  const ts = checkSchedule(check, length);
  const t0 = ts[0];
  const mid = lab.space.interpolate(qp, q, t0);
  const dmid = discsCollide(mid[0], r, mid[1], r);
  // Slice: robot 1 pinned at (1, 1); robot 2 scans its own free band y ∈ [r, height − r] at x₂ = 1.
  let blocked = 0;
  const n = 400;
  for (let k = 0; k < n; k++) {
    const y = r + ((k + 0.5) / n) * (MICRO_CORRIDOR.height - 2 * r);
    if (!lab.free.pairwiseFree([q[0], [1.0, y]])) blocked++;
  }
  return {
    distQ: dq.dist,
    penetrationQ: dq.penetration,
    collideQ: dq.collide,
    individuallyFreeQ: lab.free.individuallyFree(q),
    distQPrime: dqp.dist,
    collideQPrime: dqp.collide,
    boundaryX2,
    firstCheckX2: mid[1][0],
    firstCheckDist: dmid.dist,
    firstCheckRejects: dmid.collide,
    sliceBlockedFraction: blocked / n,
  };
}

/* -------------------------------------------------------------------------- */
/* Slice of the composite C-obstacle, for w14.1's middle panel                  */
/* -------------------------------------------------------------------------- */

export interface SliceRaster {
  nx: number;
  ny: number;
  /** 0 free, 1 world C-obstacle (robot 2 alone), 2 robot–robot C-obstacle (the moving hole). */
  cells: Uint8Array;
  bounds: { minX: number; minY: number; maxX: number; maxY: number };
}

/**
 * Robot j's 2-D slice of the composite C-space with every other body frozen
 * at q: cell by cell, the world term and the pairwise term reported
 * separately, so the amber disc of radius rᵢ + rⱼ can be drawn as the thing it
 * is — a C-obstacle that moves when the other robot does.
 */
export function sliceRaster<P extends number[]>(free: MultiBody<P>, q: readonly P[], j: number, bounds: SliceRaster['bounds'], nx: number, ny: number): SliceRaster {
  const cells = new Uint8Array(nx * ny);
  const probe = q.slice() as P[];
  for (let jy = 0; jy < ny; jy++) {
    for (let ix = 0; ix < nx; ix++) {
      const x = bounds.minX + ((ix + 0.5) / nx) * (bounds.maxX - bounds.minX);
      const y = bounds.minY + ((jy + 0.5) / ny) * (bounds.maxY - bounds.minY);
      probe[j] = [x, y] as P;
      if (!free.bodies[j].world.isFree(probe[j])) cells[jy * nx + ix] = 1;
      else if (!free.pairwiseFree(probe)) cells[jy * nx + ix] = 2;
    }
  }
  return { nx, ny, cells, bounds };
}
