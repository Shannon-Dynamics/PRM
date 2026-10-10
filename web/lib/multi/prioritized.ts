/**
 * Prioritized planning in configuration–time space — Choset §7.5.2: "robots
 * are processed in some preassigned order and a robot is treated as a moving
 * obstacle as soon as its path has been computed."
 *
 * Robot i plans in 𝒬ᵢ × [0, T]. Its obstacles are the walls (Chapter 2's
 * checker, as always) and the robots before it in the order, each a disc
 * whose center is a known function of time. Time is a coordinate the planner
 * may only increase, so the local planner is asymmetric — exactly the case
 * Choset notes after Theorem 7.4.3: "the proof will still hold for an
 * asymmetric and irreflexive local planner. This is a natural way to
 * incorporate the notion of time into PRM planning." The tree below is
 * Chapter 12's RRT with that asymmetry built into `nearest` (only earlier
 * nodes may be parents) and into `extend` (a bounded speed, forward in time).
 *
 * Also here: a coupled plan on the composite space, executed as synchronized
 * trajectories, and the per-robot "plan alone" paths the decoupled and hook
 * widgets start from — Chapter 6 A* on the inflated lattice, shortcut by
 * Chapter 11's greedy postprocessor. Nothing here is a new planner; it is the
 * old ones with time as one more coordinate.
 */

import type { Rng } from '../prob/rng';
import type { World } from '../sim/world';
import { Scene } from '../collide/scene';
import { DiscChecker } from '../collide/collision';
import { Rd } from '../manifold/rd';
import { astar } from '../search/best-first';
import { GridGraph, cellCenter, freeCellNear, gridFromWorld, gridHeuristic } from '../search/grid';
import { CountingFreeSpace } from '../sampling/free-space';
import { StraightLine, shortcutGreedy } from '../sampling/steer';
import { discsCollide, type Body, type Corridor } from './composite';

/* -------------------------------------------------------------------------- */
/* Trajectories                                                                */
/* -------------------------------------------------------------------------- */

/** A disc's center sampled every `dt` seconds; held at the last sample forever. */
export interface Trajectory {
  dt: number;
  q: number[][];
}

export function trajectoryAt(tr: Trajectory, t: number): number[] {
  if (t <= 0) return tr.q[0];
  const k = t / tr.dt;
  const i = Math.floor(k);
  if (i >= tr.q.length - 1) return tr.q[tr.q.length - 1];
  const f = k - i;
  const a = tr.q[i];
  const b = tr.q[i + 1];
  return [a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])];
}

export const trajectoryDuration = (tr: Trajectory) => (tr.q.length - 1) * tr.dt;

/** A robot already planned, seen by the next one in the order. */
export interface MovingObstacle {
  r: number;
  at(t: number): number[];
}

export const movingObstacle = (tr: Trajectory, r: number): MovingObstacle => ({ r, at: (t) => trajectoryAt(tr, t) });

/** First time at which some pair of trajectories overlaps, or null — the validator the checks use. */
export function trajectoriesConflict(trs: readonly Trajectory[], r: number, step = 0.02): { i: number; j: number; t: number } | null {
  const T = Math.max(...trs.map(trajectoryDuration));
  for (let t = 0; t <= T + 1e-9; t += step) {
    for (let i = 0; i < trs.length; i++) {
      for (let j = i + 1; j < trs.length; j++) {
        if (discsCollide(trajectoryAt(trs[i], t), r, trajectoryAt(trs[j], t), r).collide) return { i, j, t };
      }
    }
  }
  return null;
}

/* -------------------------------------------------------------------------- */
/* RRT in 𝒬 × [0, T]                                                           */
/* -------------------------------------------------------------------------- */

export interface TimeRrtOptions {
  /** Top speed of the disc, m/s. */
  vmax: number;
  /** Largest time advance of one extension, s. */
  etaT: number;
  /** Planning horizon T. */
  horizon: number;
  /** Collision-check spacing along an extension, s. */
  checkDt: number;
  goalBias: number;
  /** A node this close to the goal (in position) tries to connect and then rest. */
  goalRadius: number;
  /** Metres per second: the exchange rate between position distance and time distance in `nearest`. */
  timeWeight?: number;
}

interface TimeNode {
  q: number[];
  t: number;
  parent: number;
}

/**
 * Chapter 12's RRT with time as a monotone coordinate. `nearest` ranks tree
 * nodes by ‖q − q_rand‖ + w·(t_rand − t) over nodes *earlier* than t_rand —
 * an asymmetric relation, which is the whole point. `extend` advances at most
 * etaT in time and at most vmax·Δt in position, so every edge is executable by
 * a robot that cannot exceed vmax; a robot that waits is an edge with Δq = 0.
 */
export class TimeRrt {
  readonly nodes: TimeNode[] = [];
  iterations = 0;
  trapped = 0;
  goalNode: number | null = null;
  /** How many `isFree`-equivalent samples were spent on moving obstacles alone. */
  movingChecks = 0;

  constructor(
    readonly body: Body<number[]>,
    readonly r: number,
    readonly bounds: { minX: number; minY: number; maxX: number; maxY: number },
    readonly moving: readonly MovingObstacle[],
    readonly start: number[],
    readonly goal: number[],
    readonly opts: TimeRrtOptions,
    readonly rng: Rng,
  ) {
    this.nodes.push({ q: start, t: 0, parent: -1 });
  }

  get solved(): boolean {
    return this.goalNode !== null;
  }

  /** Free of walls and of every earlier robot at time t. */
  freeAt(q: number[], t: number): boolean {
    if (!this.body.world.isFree(q)) return false;
    for (const m of this.moving) {
      this.movingChecks++;
      if (discsCollide(q, this.r, m.at(t), m.r).collide) return false;
    }
    return true;
  }

  /** The straight segment (q₀, t₀) → (q₁, t₁), sampled every checkDt. */
  segmentFree(q0: number[], t0: number, q1: number[], t1: number): boolean {
    const n = Math.max(1, Math.ceil((t1 - t0) / this.opts.checkDt));
    for (let k = 1; k <= n; k++) {
      const f = k / n;
      if (!this.freeAt([q0[0] + f * (q1[0] - q0[0]), q0[1] + f * (q1[1] - q0[1])], t0 + f * (t1 - t0))) return false;
    }
    return true;
  }

  nearest(q: number[], t: number): number {
    const w = this.opts.timeWeight ?? this.opts.vmax;
    let best = -1;
    let bestD = Infinity;
    for (let i = 0; i < this.nodes.length; i++) {
      const n = this.nodes[i];
      if (n.t >= t) continue;
      const d = Math.hypot(q[0] - n.q[0], q[1] - n.q[1]) + w * (t - n.t);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    }
    return best;
  }

  extendOnce(): number | null {
    this.iterations++;
    const o = this.opts;
    const b = this.bounds;
    let qRand: number[];
    let tRand: number;
    if (this.rng.next() < o.goalBias) {
      qRand = this.goal;
      tRand = this.rng.uniform(0, o.horizon);
    } else {
      qRand = [this.rng.uniform(b.minX, b.maxX), this.rng.uniform(b.minY, b.maxY)];
      tRand = this.rng.uniform(0, o.horizon);
    }
    const ni = this.nearest(qRand, tRand);
    if (ni < 0) {
      this.trapped++;
      return null;
    }
    const near = this.nodes[ni];
    const dt = Math.min(o.etaT, tRand - near.t);
    const dx = qRand[0] - near.q[0];
    const dy = qRand[1] - near.q[1];
    const dist = Math.hypot(dx, dy);
    const reach = Math.min(dist, o.vmax * dt);
    const qNew = dist > 1e-12 ? [near.q[0] + (dx / dist) * reach, near.q[1] + (dy / dist) * reach] : near.q.slice();
    const tNew = near.t + dt;
    if (!this.segmentFree(near.q, near.t, qNew, tNew)) {
      this.trapped++;
      return null;
    }
    const id = this.nodes.length;
    this.nodes.push({ q: qNew, t: tNew, parent: ni });
    this.tryGoal(id);
    return id;
  }

  /** From a node near the goal: drive straight to it at vmax, then rest there until T; both must be free. */
  private tryGoal(id: number): void {
    if (this.goalNode !== null) return;
    const n = this.nodes[id];
    const d = Math.hypot(this.goal[0] - n.q[0], this.goal[1] - n.q[1]);
    if (d > this.opts.goalRadius) return;
    const tArr = n.t + d / this.opts.vmax;
    if (tArr > this.opts.horizon) return;
    if (d > 1e-12 && !this.segmentFree(n.q, n.t, this.goal, tArr)) return;
    if (!this.segmentFree(this.goal, tArr, this.goal, this.opts.horizon)) return;
    const gid = this.nodes.length;
    this.nodes.push({ q: this.goal, t: tArr, parent: id });
    this.goalNode = gid;
  }

  plan(n: number): Trajectory | null {
    for (let i = 0; i < n && !this.solved; i++) this.extendOnce();
    return this.trajectory();
  }

  /** The solution as a sampled trajectory, resting at the goal until T. */
  trajectory(dt = 0.05): Trajectory | null {
    if (this.goalNode === null) return null;
    const chain: TimeNode[] = [];
    for (let i = this.goalNode; i >= 0; i = this.nodes[i].parent) chain.push(this.nodes[i]);
    chain.reverse();
    const q: number[][] = [];
    const T = this.opts.horizon;
    for (let t = 0; t <= T + 1e-9; t += dt) {
      let k = 0;
      while (k < chain.length - 1 && chain[k + 1].t <= t) k++;
      if (k >= chain.length - 1) q.push(chain[chain.length - 1].q);
      else {
        const a = chain[k];
        const b = chain[k + 1];
        const f = (t - a.t) / Math.max(1e-12, b.t - a.t);
        q.push([a.q[0] + f * (b.q[0] - a.q[0]), a.q[1] + f * (b.q[1] - a.q[1])]);
      }
    }
    return { dt, q };
  }

  /** Space–time edges, for drawing: (q, t) → (q', t'). */
  edges(): { a: TimeNode; b: TimeNode }[] {
    const out: { a: TimeNode; b: TimeNode }[] = [];
    for (let i = 1; i < this.nodes.length; i++) out.push({ a: this.nodes[this.nodes[i].parent], b: this.nodes[i] });
    return out;
  }
}

/* -------------------------------------------------------------------------- */
/* The prioritized planner                                                     */
/* -------------------------------------------------------------------------- */

export interface PrioritizedContinuous {
  trajectories: (Trajectory | null)[];
  failedAt: number | null;
  iterations: number;
}

export const DEFAULT_TIME_RRT: TimeRrtOptions = { vmax: 1.0, etaT: 0.6, horizon: 14, checkDt: 0.05, goalBias: 0.1, goalRadius: 0.8 };

/**
 * Plan the robots in `order`, each against the walls and the trajectories of
 * the robots before it. A robot that finds no trajectory within `budget`
 * iterations fails the whole plan — there is no going back to re-plan an
 * earlier robot, which is what makes the method incomplete (Derivation 2).
 */
export function planPrioritized(
  bodies: readonly Body<number[]>[],
  r: number,
  bounds: { minX: number; minY: number; maxX: number; maxY: number },
  starts: readonly number[][],
  goals: readonly number[][],
  order: readonly number[],
  rng: Rng,
  opts: TimeRrtOptions = DEFAULT_TIME_RRT,
  budget = 3000,
): PrioritizedContinuous {
  const trajectories: (Trajectory | null)[] = bodies.map(() => null);
  const moving: MovingObstacle[] = [];
  let iterations = 0;
  for (let k = 0; k < order.length; k++) {
    const i = order[k];
    const rrt = new TimeRrt(bodies[i], r, bounds, moving, starts[i], goals[i], opts, rng.fork());
    const tr = rrt.plan(budget);
    iterations += rrt.iterations;
    if (!tr) return { trajectories, failedAt: k, iterations };
    trajectories[i] = tr;
    moving.push(movingObstacle(tr, r));
  }
  return { trajectories, failedAt: null, iterations };
}

/* -------------------------------------------------------------------------- */
/* Coupled path → synchronized trajectories                                    */
/* -------------------------------------------------------------------------- */

/**
 * A path in the composite space, driven at a composite speed `vmax` (the ℓ²
 * speed of all bodies together, so no single body exceeds vmax), sampled every
 * dt. All m robots move at once — the coupled plan has no notion of one robot
 * waiting for another unless the path itself holds a body still.
 */
export function compositePathToTrajectories(path: readonly number[][][], vmax: number, dt = 0.05): Trajectory[] {
  const m = path[0].length;
  const segLen: number[] = [];
  let L = 0;
  for (let k = 1; k < path.length; k++) {
    let s = 0;
    for (let i = 0; i < m; i++) s += (path[k][i][0] - path[k - 1][i][0]) ** 2 + (path[k][i][1] - path[k - 1][i][1]) ** 2;
    segLen.push(Math.sqrt(s));
    L += Math.sqrt(s);
  }
  const T = L / vmax;
  const out: Trajectory[] = Array.from({ length: m }, () => ({ dt, q: [] }));
  for (let t = 0; t <= T + 1e-9; t += dt) {
    let s = Math.min(L, t * vmax);
    let k = 0;
    while (k < segLen.length - 1 && s > segLen[k]) {
      s -= segLen[k];
      k++;
    }
    const f = segLen.length ? Math.min(1, s / Math.max(1e-12, segLen[k])) : 0;
    for (let i = 0; i < m; i++) {
      const a = path[k][i];
      const b = path[Math.min(path.length - 1, k + 1)][i];
      out[i].q.push([a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])]);
    }
  }
  for (let i = 0; i < m; i++) out[i].q.push(path[path.length - 1][i]);
  return out;
}

/* -------------------------------------------------------------------------- */
/* "Plan each robot alone": lattice A* + shortcutting in a corridor             */
/* -------------------------------------------------------------------------- */

/** The corridor as a ported `World`, so Chapter 6's `gridFromWorld` can rasterize it. */
export function corridorWorld(c: Corridor): World {
  return { name: 'corridor', bounds: c.bounds, walls: [...c.scene.walls] };
}

/**
 * One disc's shortest lattice path, shortcut greedily — the path a single-robot
 * planner hands to the decoupled coordinator. In the swap instance it is the
 * corridor axis for both robots, which is why coordination then fails.
 */
export function planAlone(c: Corridor, r: number, start: readonly number[], goal: readonly number[], cellSize = 0.1): number[][] | null {
  const world = corridorWorld(c);
  const grid = gridFromWorld(world, cellSize, r, c.bounds);
  const graph = new GridGraph(grid, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
  const s = freeCellNear(grid, start[0], start[1]);
  const g = freeCellNear(grid, goal[0], goal[1]);
  const t = astar(graph, s, g, gridHeuristic(grid, g, 'octile', Math.SQRT2));
  if (t.status !== 'found') return null;
  const raw = [start.slice(), ...t.path.map((cell) => {
    const p = cellCenter(grid, cell);
    return [p.x, p.y];
  }), goal.slice()];
  const checker = new DiscChecker(c.scene, r);
  const free = new CountingFreeSpace<number[]>((q) => checker.isFree({ x: q[0], y: q[1] }));
  const space = new Rd(2, { lo: [c.bounds.minX, c.bounds.minY], hi: [c.bounds.maxX, c.bounds.maxY] });
  return shortcutGreedy(new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), space, free, raw);
}

/** A disc driven along a polyline at vmax, sampled every dt. */
export function pathToTrajectory(path: readonly number[][], vmax: number, dt = 0.05): Trajectory {
  return compositePathToTrajectories(path.map((q) => [q]), vmax, dt)[0];
}

/** The Scene behind a corridor, re-exported for widgets that draw it. */
export const corridorScene = (c: Corridor): Scene => c.scene;
