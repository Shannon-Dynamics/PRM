/**
 * Choset Algorithm 22, CAR GRID SEARCH (Barraquand & Latombe [C ref. 41]),
 * implemented as written — bucketed OPEN list included.
 *
 *   1: T, OPEN ← {q_start}
 *   2: while OPEN ≠ ∅ and |T| < MAXTREESIZE
 *   3:   q ← first(OPEN)
 *   4:   if q ∈ G(q_goal): SUCCESS, return path
 *   7:   if q is not in a previously occupied cell
 *   8:     mark q's cell occupied
 *   9:     for a ∈ {L±, R±, S±}
 *  10:       integrate a forward a fixed time → q_new
 *  11:       if the motion is collision-free
 *  12–14:      add q_new under q in T, cost it, insert into OPEN by cost
 *  19: FAILURE
 *
 * Cost = a·(motion steps) + b·(steering changes) + c·(cusps) with nonnegative
 * *integers* a, b, c, so OPEN is an array of FIFO buckets indexed by cost and
 * insertion is O(1) — Choset's linked lists at element C of the array. The
 * goal is tested when a node is *popped*, not when generated (line 4), which
 * is what makes the result cost-optimal at the given step and resolution.
 *
 * The occupancy grid has d cells per dimension of (x, y, θ) — d³ boxes — and,
 * with a trailer, a fourth axis for the hitch angle θ₁ ∈ (−π/2, π/2): Choset's
 * "add a dimension … and integrate numerically to get the trailer's change",
 * here by Chapter 2's `Hitch.step` (exact body arc, RK4 trailer). The optional
 * jackknife guard prunes |θ₁| ≥ π/2 − δ (Exercise 5).
 *
 * Honesty items kept: the planner is resolution complete, not exact (it stops
 * in a goal *region*), and its paths have curvature jumps at every switch.
 */

import { normalizeAngle, type Pose2 } from '../geom/se2';
import type { Hitch, HitchCfg } from '../robots/hitch';
import { Scene } from '../collide/scene';
import { rect } from '../collide/polygon';
import { FootprintChecker } from '../collide/collision';
import type { CarSeg } from './car-path';
import type { RsSteer } from './reeds-shepp';

export type Action = 'L+' | 'L-' | 'S+' | 'S-' | 'R+' | 'R-';
export const ALL_ACTIONS: readonly Action[] = ['L+', 'S+', 'R+', 'L-', 'S-', 'R-'];
export const FORWARD_ACTIONS: readonly Action[] = ['L+', 'S+', 'R+'];

export interface GridSearchCost {
  /** Per motion step. */
  a: number;
  /** Per change of steering letter (L/S/R). */
  b: number;
  /** Per cusp (change of direction). */
  c: number;
}

export interface GoalRegion {
  pose: Pose2;
  posTol: number;
  headTol: number;
}

export interface CarGridOptions {
  car: Hitch;
  scene: Scene;
  start: HitchCfg;
  goal: GoalRegion;
  /** Arc length driven by one action (speed × fixed integration time). */
  step: number;
  /** Cells per dimension: the occupancy grid has d³ (d⁴ with a trailer) boxes. */
  d: number;
  cost: GridSearchCost;
  maxTree?: number;
  actions?: readonly Action[];
  /** Collision checks along one action. */
  substeps?: number;
  /** Prune nodes with |θ₁| ≥ this (trailer only). */
  jackknife?: number;
  /** The steering letter the car starts with (default S: wheels straight). */
  initialSteer?: RsSteer;
}

export interface CarGridNode {
  id: number;
  q: HitchCfg;
  parent: number;
  action: Action | null;
  cost: number;
  depth: number;
  /** Popped and found in a fresh cell — its children were generated. */
  expanded: boolean;
  /** Generated but its motion collided: drawn gray, never in OPEN (Choset Fig. 12.25). */
  collided: boolean;
  /** Intermediate body poses along the action, for drawing curved edges. */
  sweep: Pose2[];
}

export type CarGridStatus = 'running' | 'found' | 'failed';

const steerOf = (a: Action): RsSteer => a[0] as RsSteer;
const dirOf = (a: Action): number => (a[1] === '+' ? 1 : -1);

export class CarGridSearch {
  readonly nodes: CarGridNode[] = [];
  /** OPEN as buckets of node ids, indexed by integer cost. */
  readonly buckets: number[][] = [];
  private lowest = 0;
  private openCount = 0;
  readonly occupied: Uint8Array;
  readonly cells: number;
  status: CarGridStatus = 'running';
  goalNode: number | null = null;
  expansions = 0;
  pops = 0;
  maxOpen = 0;
  /** Order in which nodes were popped (for the animation). */
  readonly popOrder: number[] = [];
  private readonly checker: FootprintChecker;
  private readonly maxTree: number;
  private readonly actions: readonly Action[];
  private readonly substeps: number;
  private readonly withTrailer: boolean;

  constructor(readonly opts: CarGridOptions) {
    this.checker = new FootprintChecker(opts.scene, opts.car);
    this.maxTree = opts.maxTree ?? 200000;
    this.actions = opts.actions ?? ALL_ACTIONS;
    this.substeps = opts.substeps ?? 4;
    this.withTrailer = opts.car.hasTrailer && opts.start.trailer !== null;
    const d = opts.d;
    this.cells = this.withTrailer ? d * d * d * d : d * d * d;
    this.occupied = new Uint8Array(this.cells);
    this.nodes.push({ id: 0, q: opts.start, parent: -1, action: null, cost: 0, depth: 0, expanded: false, collided: false, sweep: [opts.start.body] });
    this.pushOpen(0, 0);
  }

  get openSize(): number {
    return this.openCount;
  }

  /** Free (non-collided) nodes in T — what MAXTREESIZE bounds. */
  treeSize = 1;

  private pushOpen(id: number, cost: number): void {
    while (this.buckets.length <= cost) this.buckets.push([]);
    this.buckets[cost].push(id);
    this.openCount++;
    if (cost < this.lowest) this.lowest = cost;
    this.maxOpen = Math.max(this.maxOpen, this.openCount);
  }

  private popOpen(): number | null {
    while (this.lowest < this.buckets.length && this.buckets[this.lowest].length === 0) this.lowest++;
    if (this.lowest >= this.buckets.length) return null;
    this.openCount--;
    return this.buckets[this.lowest].shift()!;
  }

  /** OPEN in Choset's order: by cost, FIFO within a cost. */
  openList(): number[] {
    const out: number[] = [];
    for (let c = 0; c < this.buckets.length; c++) out.push(...this.buckets[c]);
    return out;
  }

  /** The d³ (or d⁴) cell of a configuration. */
  cellOf(q: HitchCfg): number {
    const { d, scene } = this.opts;
    const b = scene.bounds;
    const clampI = (v: number) => Math.max(0, Math.min(d - 1, Math.floor(v)));
    const ix = clampI(((q.body.x - b.minX) / (b.maxX - b.minX)) * d);
    const iy = clampI(((q.body.y - b.minY) / (b.maxY - b.minY)) * d);
    let th = q.body.theta % (2 * Math.PI);
    if (th < 0) th += 2 * Math.PI;
    const it = clampI((th / (2 * Math.PI)) * d);
    let idx = (it * d + iy) * d + ix;
    if (this.withTrailer && q.trailer !== null) {
      const h = normalizeAngle(q.body.theta - q.trailer);
      const ih = clampI(((h + Math.PI / 2) / Math.PI) * d);
      idx = idx * d + ih;
    }
    return idx;
  }

  inGoal(q: HitchCfg): boolean {
    const g = this.opts.goal;
    return Math.hypot(q.body.x - g.pose.x, q.body.y - g.pose.y) <= g.posTol && Math.abs(normalizeAngle(q.body.theta - g.pose.theta)) <= g.headTol;
  }

  /** Line 10–11: integrate one action and check the motion. */
  private apply(q: HitchCfg, a: Action): { q: HitchCfg; sweep: Pose2[]; free: boolean } {
    const { car, step } = this.opts;
    const phi = a[0] === 'L' ? car.p.maxSteer : a[0] === 'R' ? -car.p.maxSteer : 0;
    const v = dirOf(a);
    const h = step / this.substeps;
    let cur = q;
    const sweep: Pose2[] = [q.body];
    let free = true;
    for (let k = 0; k < this.substeps; k++) {
      cur = car.step(cur, { v, phi }, h);
      sweep.push(cur.body);
      if (free && !this.checker.isFree(cur)) free = false;
    }
    if (free && this.withTrailer && this.opts.jackknife !== undefined && cur.trailer !== null) {
      if (Math.abs(normalizeAngle(cur.body.theta - cur.trailer)) >= this.opts.jackknife) free = false;
    }
    return { q: cur, sweep, free };
  }

  /** One pass of the while loop: pop, goal test, occupancy test, expand. */
  step(): CarGridStatus {
    if (this.status !== 'running') return this.status;
    if (this.openCount === 0 || this.treeSize >= this.maxTree) {
      this.status = 'failed';
      return this.status;
    }
    const id = this.popOpen();
    if (id === null) {
      this.status = 'failed';
      return this.status;
    }
    this.pops++;
    this.popOrder.push(id);
    const node = this.nodes[id];
    if (this.inGoal(node.q)) {
      this.goalNode = id;
      this.status = 'found';
      return this.status;
    }
    const cell = this.cellOf(node.q);
    if (this.occupied[cell]) return this.status;
    this.occupied[cell] = 1;
    node.expanded = true;
    this.expansions++;
    const { a, b, c } = this.opts.cost;
    const prevSteer = node.action ? steerOf(node.action) : this.opts.initialSteer ?? null;
    const prevDir = node.action ? dirOf(node.action) : 0;
    for (const act of this.actions) {
      const r = this.apply(node.q, act);
      let cost = node.cost + a;
      if (prevSteer !== null && steerOf(act) !== prevSteer) cost += b;
      if (prevDir !== 0 && dirOf(act) !== prevDir) cost += c;
      const child: CarGridNode = { id: this.nodes.length, q: r.q, parent: id, action: act, cost, depth: node.depth + 1, expanded: false, collided: !r.free, sweep: r.sweep };
      this.nodes.push(child);
      if (r.free) {
        this.treeSize++;
        this.pushOpen(child.id, cost);
      }
    }
    return this.status;
  }

  run(maxPops = Infinity): CarGridStatus {
    let n = 0;
    while (this.status === 'running' && n++ < maxPops) this.step();
    return this.status;
  }

  /** Root → node. */
  chain(id: number): CarGridNode[] {
    const out: CarGridNode[] = [];
    let cur = id;
    while (cur >= 0) {
      out.push(this.nodes[cur]);
      cur = this.nodes[cur].parent;
    }
    return out.reverse();
  }

  path(): CarGridNode[] | null {
    return this.goalNode === null ? null : this.chain(this.goalNode);
  }

  /** The solution as signed metric segments (consecutive equal actions merged). */
  segments(): CarSeg[] | null {
    const p = this.path();
    if (!p) return null;
    const out: CarSeg[] = [];
    for (const n of p.slice(1)) {
      const act = n.action!;
      const len = dirOf(act) * this.opts.step;
      const last = out[out.length - 1];
      if (last && last.steer === steerOf(act) && Math.sign(last.len) === Math.sign(len)) last.len += len;
      else out.push({ steer: steerOf(act), len });
    }
    return out;
  }

  /** Steps, steering changes and cusps of the solution — the three terms of the cost. */
  costBreakdown(): { steps: number; changes: number; cusps: number } | null {
    const p = this.path();
    if (!p) return null;
    let changes = 0;
    let cusps = 0;
    let prev: Action | null = null;
    const init = this.opts.initialSteer ?? null;
    for (const n of p.slice(1)) {
      const act = n.action!;
      const ps = prev ? steerOf(prev) : init;
      if (ps !== null && steerOf(act) !== ps) changes++;
      if (prev && dirOf(act) !== dirOf(prev)) cusps++;
      prev = act;
    }
    return { steps: p.length - 1, changes, cusps };
  }
}

/* -------------------------------------------------------------------------- */
/* Figure f21.7: a small search in the spirit of Choset's Fig. 12.25            */
/* -------------------------------------------------------------------------- */

export interface Fig1225 {
  nodes: CarGridNode[];
  open: { id: number; cost: number }[];
  popOrder: number[];
  step: number;
  scene: Scene;
  start: HitchCfg;
}

/** The figure's world: an open 16 × 10 m floor with one long block below the car's lane. */
export const FIG1225_BLOCK = { minX: 3.5, minY: 2.6, maxX: 10.5, maxY: 3.7 };

/**
 * Forward actions only (L⁺, S⁺, R⁺), motion cost 1, steering change 5, cusps
 * irrelevant; the car starts with its wheels straight at (3, 5, 0); the block
 * grays out every right turn. Three pops, then the tree and OPEN are frozen —
 * the same bookkeeping as Choset's Fig. 12.25 (whose own costs are 3, 7, 7, 7, 8).
 */
export function figure1225(car: Hitch, pops = 3, step = 1.5): Fig1225 {
  const b = FIG1225_BLOCK;
  const scene = new Scene([], [{ name: 'block', poly: rect(b.minX, b.minY, b.maxX, b.maxY) }], { minX: 0, minY: 0, maxX: 16, maxY: 10 });
  const start: HitchCfg = { body: { x: 3, y: 5, theta: 0 }, trailer: null };
  const s = new CarGridSearch({
    car,
    scene,
    start,
    goal: { pose: { x: 1e9, y: 1e9, theta: 0 }, posTol: 0, headTol: 0 },
    step,
    d: 64,
    cost: { a: 1, b: 5, c: 0 },
    actions: FORWARD_ACTIONS,
    initialSteer: 'S',
  });
  for (let i = 0; i < pops; i++) s.step();
  const open = s.openList().map((id) => ({ id, cost: s.nodes[id].cost }));
  return { nodes: s.nodes, open, popOrder: s.popOrder, step, scene, start };
}
