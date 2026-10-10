/**
 * Tree planners — Choset §7.2 and §7.3: EST (Algs. 8–9), RRT (Algs. 10–13),
 * RRT-Connect, SBL, SRT (Alg. 14) and KPIECE, all over one `Tree<P>`.
 *
 * Chapter 11 learned all of 𝒬_free so that any query is cheap. Most robots
 * have one query, now; a tree grown from the configuration you are in toward
 * the one you want samples *conditionally* on what the tree already knows and
 * stops the instant the two meet. Two instincts: EST picks a lonely node and
 * pushes outward from it; RRT picks a random target and pulls the nearest
 * node toward it. The pull is a Voronoi bias — the probability a node is
 * chosen equals the volume of its Voronoi region — and `voronoiOwners` below
 * is the raster that lets w12.1 draw that fact.
 *
 * Everything from Chapter 11 is imported, nothing re-introduced: `Sampler`,
 * `KdTree`, `Steer` + `extend`, `FreeSpace`. Chapter 13 wraps `Tree<P>` with
 * costs; it does not fork it, so RRT and RRT* can be fed identical samples
 * side by side.
 */

import type { Rng } from '../prob/rng';
import type { Chart } from '../manifold/manifold';
import { Rd } from '../manifold/rd';
import type { FreeSpace } from './free-space';
import { makeNn, type Neighbor, type NearestNeighbors } from './kdtree';
import { UnionFind } from './prm';
import { Uniform, sampleBall, type Sampler } from './sampler';
import { extend, type ExtendStatus, type Steer } from './steer';

/* -------------------------------------------------------------------------- */
/* Tree                                                                        */
/* -------------------------------------------------------------------------- */

export type NodeId = number;

export interface TreeNode<P> {
  q: P;
  /** −1 for the root. */
  parent: NodeId;
  depth: number;
  /** Iteration at which the node was added (for time scrubbing and fading). */
  born: number;
}

/**
 * One tree type for every planner in Part III: a node arena with parent
 * indices and a Chapter 11 nearest-neighbor structure kept in sync. Every node
 * is reachable from the root by construction (Choset §7.2.1: "there is a path
 * from the root of T to every configuration in T").
 */
export class Tree<P> {
  readonly nodes: TreeNode<P>[] = [];
  readonly nn: NearestNeighbors<P>;
  /** Iteration counter the owning planner advances; stamped onto new nodes. */
  clock = 0;

  constructor(
    readonly space: Chart<P>,
    root: P,
    nnKind: 'kd' | 'linear' = 'kd',
  ) {
    this.nn = makeNn(space, nnKind);
    this.add(root, -1);
  }

  get size(): number {
    return this.nodes.length;
  }

  get root(): P {
    return this.nodes[0].q;
  }

  nearest(q: P): Neighbor {
    const n = this.nn.nearest(q);
    if (!n) throw new Error('Tree.nearest on an empty tree');
    return n;
  }

  kNearest(q: P, k: number): Neighbor[] {
    return this.nn.kNearest(q, k);
  }

  within(q: P, r: number): Neighbor[] {
    return this.nn.within(q, r);
  }

  add(q: P, parent: NodeId): NodeId {
    const id = this.nodes.length;
    this.nodes.push({ q, parent, depth: parent < 0 ? 0 : this.nodes[parent].depth + 1, born: this.clock });
    this.nn.insert(id, q);
    return id;
  }

  /** Node indices from v up to the root, root last. */
  chainToRoot(v: NodeId): NodeId[] {
    const out: NodeId[] = [];
    for (let i = v; i >= 0; i = this.nodes[i].parent) out.push(i);
    return out;
  }

  /** Configurations from the root down to v. */
  pathToRoot(v: NodeId): P[] {
    return this.chainToRoot(v)
      .reverse()
      .map((i) => this.nodes[i].q);
  }

  /** Configurations from a to b through their lowest common ancestor. */
  pathBetween(a: NodeId, b: NodeId): P[] {
    const up = this.chainToRoot(a);
    const onA = new Set(up);
    const down: NodeId[] = [];
    let j = b;
    while (!onA.has(j)) {
      down.push(j);
      j = this.nodes[j].parent;
    }
    const upTo = up.slice(0, up.indexOf(j) + 1);
    return [...upTo, ...down.reverse()].map((i) => this.nodes[i].q);
  }

  /** Every (parent, child) pair, child-indexed. */
  edges(): { a: NodeId; b: NodeId }[] {
    const out: { a: NodeId; b: NodeId }[] = [];
    for (let i = 1; i < this.nodes.length; i++) out.push({ a: this.nodes[i].parent, b: i });
    return out;
  }

  /** Sum of edge lengths under the manifold metric. */
  totalLength(): number {
    let L = 0;
    for (let i = 1; i < this.nodes.length; i++) L += this.space.dist(this.nodes[this.nodes[i].parent].q, this.nodes[i].q);
    return L;
  }

  /** Length of the tree path from the root to v. */
  costToRoot(v: NodeId): number {
    let c = 0;
    for (let i = v; this.nodes[i].parent >= 0; i = this.nodes[i].parent) c += this.space.dist(this.nodes[this.nodes[i].parent].q, this.nodes[i].q);
    return c;
  }
}

/* -------------------------------------------------------------------------- */
/* Voronoi bias, rasterized                                                    */
/* -------------------------------------------------------------------------- */

export interface VoronoiRaster {
  n1: number;
  n2: number;
  /** Owner node id per cell, row-major j·n1 + i. */
  owner: Int32Array;
  /** Fraction of the raster owned by each node (sums to 1 over free cells). */
  share: Float64Array;
  /** Chart coordinates of cell (i, j) centers. */
  lo: [number, number];
  hi: [number, number];
}

/**
 * The Voronoi diagram of the tree's nodes under the manifold metric, as a
 * raster over the first two chart coordinates. With q_rand uniform, a node is
 * extended with probability equal to its share: this is derivation F.1 made a
 * picture. Optional `free` restricts the measure to 𝒬_free (owner −1 where
 * the cell is in collision).
 */
export function voronoiOwners<P>(tree: Tree<P>, n1: number, n2: number, free?: FreeSpace<P>): VoronoiRaster {
  const space = tree.space;
  const [lo, hi] = chartBox2(space);
  const owner = new Int32Array(n1 * n2).fill(-1);
  const share = new Float64Array(tree.size);
  let freeCells = 0;
  for (let j = 0; j < n2; j++) {
    for (let i = 0; i < n1; i++) {
      const c = [lo[0] + ((i + 0.5) / n1) * (hi[0] - lo[0]), lo[1] + ((j + 0.5) / n2) * (hi[1] - lo[1])];
      const q = space.fromCoords(padCoords(space, c));
      if (free && !free.isFree(q)) continue;
      const o = tree.nearest(q).idx;
      owner[j * n1 + i] = o;
      share[o]++;
      freeCells++;
    }
  }
  if (freeCells > 0) for (let k = 0; k < share.length; k++) share[k] /= freeCells;
  return { n1, n2, owner, share, lo, hi };
}

/** The box the first two chart coordinates range over: ℝᵈ bounds or [−π, π) on a periodic axis. */
export function chartBox2<P>(space: Chart<P>): [[number, number], [number, number]] {
  const lo: [number, number] = [-Math.PI, -Math.PI];
  const hi: [number, number] = [Math.PI, Math.PI];
  for (let a = 0; a < 2; a++) {
    if (space.periodic[a]) {
      lo[a] = -space.period / 2;
      hi[a] = space.period / 2;
    } else if (space instanceof Rd) {
      lo[a] = space.bounds.lo[a];
      hi[a] = space.bounds.hi[a];
    } else {
      const b = (space as unknown as { bounds?: { minX: number; minY: number; maxX: number; maxY: number } }).bounds;
      if (b) {
        lo[a] = a === 0 ? b.minX : b.minY;
        hi[a] = a === 0 ? b.maxX : b.maxY;
      }
    }
  }
  return [lo, hi];
}

const padCoords = <P>(space: Chart<P>, c: number[]): number[] => {
  const out = c.slice();
  while (out.length < space.dim) out.push(0);
  return out;
};

/* -------------------------------------------------------------------------- */
/* RRT — Choset Algorithms 10–12                                               */
/* -------------------------------------------------------------------------- */

export interface RrtOptions<P> {
  steer: Steer<P>;
  /** Default: uniform. `Scripted` replays the hand-traced example. */
  sampler?: Sampler<P>;
  /** η: how far `extend` moves from q_near toward q_rand. */
  eta: number;
  /** p: probability that q_rand := q_goal (Choset §7.2.2; experimental 0.05). */
  goalBias?: number;
  /** A node this close to the goal (with a free segment to it) solves the query. Default η. */
  goalRadius?: number;
  nn?: 'kd' | 'linear';
}

export interface RrtIteration<P> {
  qRand: P | null;
  near: NodeId | null;
  status: ExtendStatus<P>;
  added: NodeId | null;
}

/**
 * C Alg. 10 driven by a sampler, with Alg. 11 as `extendToward` and Alg. 12 as
 * `connect`. `trapped` counts the iterations that added nothing — the gap
 * between iterations and nodes w12.1 plots.
 */
export class Rrt<P> {
  readonly tree: Tree<P>;
  readonly sampler: Sampler<P>;
  readonly eta: number;
  readonly goalBias: number;
  readonly goalRadius: number;
  iterations = 0;
  trapped = 0;
  /** Consecutive `Trapped` results — the "stuck" badge at p = 1. */
  trappedStreak = 0;
  /** The node from which the goal was reached, once. */
  goalNode: NodeId | null = null;
  /** Iteration of the first solution. */
  solvedAt: number | null = null;
  /** Node count sampled every iteration (the nodes-vs-iterations curve). */
  readonly nodeHistory: number[] = [];
  /** How often each node has been chosen as q_near — the measured Voronoi bias. */
  readonly chosen: number[] = [0];

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    root: P,
    readonly goal: P | null,
    readonly opts: RrtOptions<P>,
    readonly rng: Rng,
  ) {
    this.tree = new Tree(space, root, opts.nn ?? 'kd');
    this.sampler = opts.sampler ?? new Uniform<P>();
    this.eta = opts.eta;
    this.goalBias = opts.goalBias ?? 0.05;
    this.goalRadius = opts.goalRadius ?? opts.eta;
  }

  get steer(): Steer<P> {
    return this.opts.steer;
  }

  get solved(): boolean {
    return this.goalNode !== null;
  }

  /** C Alg. 11: q_near ← nearest; q_new ← progress η toward q; add if free. */
  extendToward(q: P): { status: ExtendStatus<P>; near: NodeId; added: NodeId | null } {
    const nb = this.tree.nearest(q);
    this.chosen[nb.idx] = (this.chosen[nb.idx] ?? 0) + 1;
    const st = extend(this.steer, this.space, this.free, this.tree.nodes[nb.idx].q, q, this.eta);
    if (st.kind === 'trapped') {
      this.trapped++;
      this.trappedStreak++;
      return { status: st, near: nb.idx, added: null };
    }
    this.trappedStreak = 0;
    const id = this.tree.add(st.q, nb.idx);
    this.chosen.push(0);
    this.checkGoal(id);
    return { status: st, near: nb.idx, added: id };
  }

  /** One iteration of C Alg. 10, with the goal bias of §7.2.2 mixed into the sampler. */
  extendOnce(): RrtIteration<P> {
    this.iterations++;
    this.tree.clock = this.iterations;
    let qRand: P | null;
    if (this.goal && this.goalBias > 0 && this.rng.next() < this.goalBias) qRand = this.goal;
    else qRand = this.sampler.sample(this.space, this.free, this.rng);
    if (qRand === null) {
      this.nodeHistory.push(this.tree.size);
      return { qRand: null, near: null, status: { kind: 'trapped' }, added: null };
    }
    const r = this.extendToward(qRand);
    this.nodeHistory.push(this.tree.size);
    return { qRand, ...r };
  }

  /** C Alg. 12: repeat extend toward q until Reached or Trapped. */
  connect(q: P): { connected: boolean; last: NodeId | null } {
    let last: NodeId | null = null;
    for (;;) {
      const r = this.extendToward(q);
      if (r.added !== null) last = r.added;
      if (r.status.kind === 'reached') return { connected: true, last };
      if (r.status.kind === 'trapped') return { connected: false, last };
    }
  }

  /** C Alg. 10 plus the goal test: run up to n iterations, return the path if one appeared. */
  plan(n: number): P[] | null {
    for (let i = 0; i < n && !this.solved; i++) this.extendOnce();
    return this.path();
  }

  /** The root-to-goal path (root … goalNode, goal), or null. */
  path(): P[] | null {
    if (this.goalNode === null || !this.goal) return null;
    return [...this.tree.pathToRoot(this.goalNode), this.goal];
  }

  private checkGoal(id: NodeId): void {
    if (!this.goal || this.goalNode !== null) return;
    const q = this.tree.nodes[id].q;
    if (this.space.dist(q, this.goal) <= this.goalRadius && this.steer.steer(this.space, this.free, q, this.goal)) {
      this.goalNode = id;
      this.solvedAt = this.iterations;
    }
  }
}

/* -------------------------------------------------------------------------- */
/* EST — Choset Algorithms 8–9                                                  */
/* -------------------------------------------------------------------------- */

export type Density = { kind: 'naive'; radius: number } | { kind: 'grid'; cell: number };

export interface EstOptions<P> {
  steer: Steer<P>;
  /** Neighborhood radius for q_new around the chosen node (Alg. 9 line 1). */
  radius: number;
  /** How w_T is measured: node count within a ball (O(n)), or grid-cell occupancy (SBL, O(1)). */
  density?: Density;
  goalRadius?: number;
  nn?: 'kd' | 'linear';
}

/**
 * Expansive-Spaces Tree. Node q is chosen with probability π_T(q) ∝ 1/w_T(q),
 * where w_T counts the configurations within a neighborhood of q (§7.2.1,
 * "Guiding the Sampling"): sparse neighborhoods — the frontier — are more
 * likely to be pushed. q_new is a free sample in the ball of `radius` around q,
 * accepted only if Δ(q, q_new) succeeds.
 */
export class Est<P> {
  readonly tree: Tree<P>;
  readonly density: Density;
  /** w_T per node under the naive density (kept incrementally). */
  readonly weight: number[] = [1];
  iterations = 0;
  rejected = 0;
  goalNode: NodeId | null = null;
  solvedAt: number | null = null;
  readonly nodeHistory: number[] = [];
  private readonly cells = new Map<string, NodeId[]>();

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    root: P,
    readonly goal: P | null,
    readonly opts: EstOptions<P>,
    readonly rng: Rng,
  ) {
    this.tree = new Tree(space, root, opts.nn ?? 'kd');
    this.density = opts.density ?? { kind: 'naive', radius: opts.radius };
    this.indexCell(0);
  }

  get solved(): boolean {
    return this.goalNode !== null;
  }

  private cellKey(q: P): string {
    const d = this.density;
    if (d.kind !== 'grid') return '';
    return this.space
      .coords(q)
      .map((x) => Math.floor(x / d.cell))
      .join(',');
  }

  private indexCell(id: NodeId): void {
    if (this.density.kind !== 'grid') return;
    const k = this.cellKey(this.tree.nodes[id].q);
    const list = this.cells.get(k);
    if (list) list.push(id);
    else this.cells.set(k, [id]);
  }

  /** π_T(q): the node to grow from. */
  select(): NodeId {
    if (this.density.kind === 'grid') {
      // A random non-empty cell, then a random node in it: nodes in sparse cells are favored (§7.2.3).
      const keys = [...this.cells.keys()];
      const list = this.cells.get(keys[Math.floor(this.rng.next() * keys.length)])!;
      return list[Math.floor(this.rng.next() * list.length)];
    }
    return this.rng.choice(this.weight.map((w) => 1 / w));
  }

  /** C Alg. 9 from a chosen q. */
  extendFrom(from: NodeId): NodeId | null {
    const q = this.tree.nodes[from].q;
    const qNew = sampleBall(this.space, q, this.opts.radius, this.rng);
    if (!this.free.isFree(qNew)) {
      this.rejected++;
      return null;
    }
    if (!this.opts.steer.steer(this.space, this.free, q, qNew)) {
      this.rejected++;
      return null;
    }
    const id = this.tree.add(qNew, from);
    this.register(id);
    return id;
  }

  private register(id: NodeId): void {
    if (this.density.kind === 'naive') {
      const nbs = this.tree.within(this.tree.nodes[id].q, this.density.radius);
      this.weight.push(nbs.length); // includes itself
      for (const nb of nbs) if (nb.idx !== id) this.weight[nb.idx]++;
    } else {
      this.weight.push(1);
      this.indexCell(id);
    }
    if (this.goal && this.goalNode === null) {
      const q = this.tree.nodes[id].q;
      const gr = this.opts.goalRadius ?? this.opts.radius;
      if (this.space.dist(q, this.goal) <= gr && this.opts.steer.steer(this.space, this.free, q, this.goal)) {
        this.goalNode = id;
        this.solvedAt = this.iterations;
      }
    }
  }

  /** One iteration of C Alg. 8. */
  extendOnce(): { from: NodeId; added: NodeId | null } {
    this.iterations++;
    this.tree.clock = this.iterations;
    const from = this.select();
    const added = this.extendFrom(from);
    this.nodeHistory.push(this.tree.size);
    return { from, added };
  }

  plan(n: number): P[] | null {
    for (let i = 0; i < n && !this.solved; i++) this.extendOnce();
    return this.path();
  }

  path(): P[] | null {
    if (this.goalNode === null || !this.goal) return null;
    return [...this.tree.pathToRoot(this.goalNode), this.goal];
  }
}

/* -------------------------------------------------------------------------- */
/* Bidirectional RRT — C Alg. 13 with a greediness switch; RRT-Connect          */
/* -------------------------------------------------------------------------- */

export type Greed = 'extend' | 'connect';

export interface RrtConnectOptions<P> extends Omit<RrtOptions<P>, 'goalBias' | 'goalRadius'> {
  /** Greediness of (the tree extended toward q_rand, the tree extended toward q_new). */
  greed?: [Greed, Greed];
}

/**
 * Two trees grown toward each other. (Extend, Extend) is Choset's Alg. 13 as
 * written; (Extend, Connect) is Kuffner & LaValle's RRT-Connect [C ref. 249];
 * (Connect, Connect) is the greediest setting. Trees swap after every
 * successful first extension so neither starves (Exercise 2).
 */
export class RrtConnect<P> {
  /** [T_init, T_goal]. */
  readonly trees: [Rrt<P>, Rrt<P>];
  readonly greed: [Greed, Greed];
  /** Which tree is T₁ (receives q_rand) this iteration; Alg. 13 line 9 swaps it. */
  private cur: 0 | 1 = 0;
  attempts = 0;
  merged: { init: NodeId; goal: NodeId } | null = null;
  mergedAt: number | null = null;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    readonly start: P,
    readonly goal: P,
    readonly opts: RrtConnectOptions<P>,
    readonly rng: Rng,
  ) {
    const base: RrtOptions<P> = { ...opts, goalBias: 0 };
    this.trees = [new Rrt(space, free, start, null, base, rng), new Rrt(space, free, goal, null, base, rng)];
    this.greed = opts.greed ?? ['extend', 'extend'];
  }

  get initTree(): Rrt<P> {
    return this.trees[0];
  }

  get goalTree(): Rrt<P> {
    return this.trees[1];
  }

  get solved(): boolean {
    return this.merged !== null;
  }

  /** One iteration of C Alg. 13 (lines 2–10). */
  step(): 'merged' | 'running' {
    if (this.merged) return 'merged';
    this.attempts++;
    const T1 = this.trees[this.cur];
    const T2 = this.trees[1 - this.cur];
    for (const t of this.trees) {
      t.iterations++;
      t.tree.clock = this.attempts;
    }
    const qRand = T1.sampler.sample(this.space, this.free, this.rng);
    if (qRand === null) return 'running';
    const first = this.grow(T1, qRand, this.greed[0]);
    if (first === null) return 'running';
    const qNew1 = T1.tree.nodes[first].q;
    const second = this.grow(T2, qNew1, this.greed[1]);
    if (second !== null && this.space.equals(T2.tree.nodes[second].q, qNew1, 1e-9)) {
      this.merged = this.cur === 0 ? { init: first, goal: second } : { init: second, goal: first };
      this.mergedAt = this.attempts;
      return 'merged';
    }
    this.cur = this.cur === 0 ? 1 : 0; // SWAP(T1, T2)
    return 'running';
  }

  private grow(t: Rrt<P>, q: P, g: Greed): NodeId | null {
    if (g === 'extend') return t.extendToward(q).added;
    return t.connect(q).last;
  }

  /** Run up to `attempts` iterations; the path if the trees merged. */
  merge(attempts: number): P[] | null {
    for (let i = 0; i < attempts && !this.merged; i++) this.step();
    return this.path();
  }

  path(): P[] | null {
    if (!this.merged) return null;
    const fromStart = this.initTree.tree.pathToRoot(this.merged.init);
    const toGoal = this.goalTree.tree.pathToRoot(this.merged.goal).reverse();
    // The merge node appears in both; drop the duplicate.
    return [...fromStart, ...toGoal.slice(1)];
  }

  get nodeCount(): number {
    return this.trees[0].tree.size + this.trees[1].tree.size;
  }
}

/* -------------------------------------------------------------------------- */
/* SBL — Sánchez & Latombe [C ref. 367]: bidirectional, lazy EST                */
/* -------------------------------------------------------------------------- */

interface SblNode<P> {
  q: P;
  parent: number;
  /** Which tree the node belongs to (0 = start, 1 = goal). */
  tree: 0 | 1;
  /** Has the edge to the parent been verified by Δ? Lazy: false until it lies on a candidate path. */
  checked: boolean;
}

export interface SblOptions<P> {
  steer: Steer<P>;
  /** Expansion radius ρ (new samples are drawn in the ρ-ball of the chosen node). */
  radius: number;
  /** Grid cell size for the §7.2.3 cell-based selection. */
  cell: number;
}

/**
 * Single-query, Bi-directional, Lazy collision checking. Nodes are checked on
 * creation; an edge is checked exactly once, only when it lies on the path that
 * would join the two trees. When a lazy edge fails, SBL cuts it and hands the
 * cut-off piece of the path to the other tree, so no sample is wasted.
 */
export class Sbl<P> {
  readonly nodes: SblNode<P>[] = [];
  iterations = 0;
  edgeChecks = 0;
  edgeFailures = 0;
  path: P[] | null = null;
  solvedAt: number | null = null;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    start: P,
    goal: P,
    readonly opts: SblOptions<P>,
    readonly rng: Rng,
  ) {
    this.nodes.push({ q: start, parent: -1, tree: 0, checked: true });
    this.nodes.push({ q: goal, parent: -1, tree: 1, checked: true });
  }

  get solved(): boolean {
    return this.path !== null;
  }

  private cellKey(q: P): string {
    return this.space
      .coords(q)
      .map((x) => Math.floor(x / this.opts.cell))
      .join(',');
  }

  /** §7.2.3: a random non-empty cell of tree t, then a random node of that cell. */
  private select(t: 0 | 1): number {
    const cells = new Map<string, number[]>();
    for (let i = 0; i < this.nodes.length; i++) {
      if (this.nodes[i].tree !== t) continue;
      const k = this.cellKey(this.nodes[i].q);
      const l = cells.get(k);
      if (l) l.push(i);
      else cells.set(k, [i]);
    }
    const keys = [...cells.keys()];
    const list = cells.get(keys[Math.floor(this.rng.next() * keys.length)])!;
    return list[Math.floor(this.rng.next() * list.length)];
  }

  private nearestIn(t: 0 | 1, q: P): { idx: number; dist: number } | null {
    let best = -1;
    let bd = Infinity;
    for (let i = 0; i < this.nodes.length; i++) {
      if (this.nodes[i].tree !== t) continue;
      const d = this.space.dist(this.nodes[i].q, q);
      if (d < bd) {
        bd = d;
        best = i;
      }
    }
    return best < 0 ? null : { idx: best, dist: bd };
  }

  private chain(i: number): number[] {
    const out: number[] = [];
    for (let k = i; k >= 0; k = this.nodes[k].parent) out.push(k);
    return out;
  }

  /** One SBL iteration: expand the tree whose turn it is, then try to bridge. */
  step(): 'solved' | 'running' {
    if (this.path) return 'solved';
    this.iterations++;
    const t: 0 | 1 = (this.iterations % 2) as 0 | 1;
    const from = this.select(t);
    // Shrink the ball until a free sample appears (a few tries), as the paper does.
    let qNew: P | null = null;
    for (let k = 0; k < 4 && qNew === null; k++) {
      const q = sampleBall(this.space, this.nodes[from].q, this.opts.radius / (k + 1), this.rng);
      if (this.free.isFree(q)) qNew = q;
    }
    if (qNew === null) return 'running';
    const id = this.nodes.length;
    this.nodes.push({ q: qNew, parent: from, tree: t, checked: false }); // lazy: the edge is assumed
    // Bridge: the nearest node of the other tree, if within ρ.
    const other = this.nearestIn(t === 0 ? 1 : 0, qNew);
    if (!other || other.dist > this.opts.radius) return 'running';
    return this.tryBridge(id, other.idx) ? 'solved' : 'running';
  }

  /**
   * Check the candidate path a … bridge … b lazily: unchecked edges are tested
   * longest first; the first failure cuts that edge, re-roots the severed
   * piece onto the other tree through the bridge, and the step returns.
   */
  private tryBridge(a: number, b: number): boolean {
    const ca = this.chain(a); // a … root_a
    const cb = this.chain(b); // b … root_b
    const edges: { child: number; len: number }[] = [];
    for (const list of [ca, cb]) {
      for (const n of list) {
        if (this.nodes[n].parent >= 0 && !this.nodes[n].checked) {
          edges.push({ child: n, len: this.space.dist(this.nodes[n].q, this.nodes[this.nodes[n].parent].q) });
        }
      }
    }
    // The bridge itself first: it is the newest and most suspect segment.
    this.edgeChecks++;
    if (!this.opts.steer.steer(this.space, this.free, this.nodes[a].q, this.nodes[b].q)) {
      this.edgeFailures++;
      return false;
    }
    edges.sort((x, y) => y.len - x.len);
    for (const e of edges) {
      this.edgeChecks++;
      const n = this.nodes[e.child];
      if (this.opts.steer.steer(this.space, this.free, this.nodes[n.parent].q, n.q)) {
        n.checked = true;
        continue;
      }
      this.edgeFailures++;
      this.cutAndTransfer(e.child, a, b);
      return false;
    }
    const fromStart = (this.nodes[a].tree === 0 ? ca : cb).reverse().map((i) => this.nodes[i].q);
    const toGoal = (this.nodes[a].tree === 0 ? cb : ca).map((i) => this.nodes[i].q);
    this.path = [...fromStart, ...toGoal];
    this.solvedAt = this.iterations;
    return true;
  }

  /**
   * Edge (parent(child) → child) failed on the path. The piece of the path from
   * `child` to the bridge endpoint on its side moves to the other tree: parent
   * pointers along that piece are reversed and the bridge becomes a (verified)
   * tree edge. Everything hanging off the moved piece moves with it.
   */
  private cutAndTransfer(child: number, a: number, b: number): void {
    const side = this.nodes[child].tree;
    const end = this.nodes[a].tree === side ? a : b;
    const otherEnd = end === a ? b : a;
    const newTree = this.nodes[otherEnd].tree;
    // Collect the subtree under `child` (it contains `end`).
    const sub = new Set<number>([child]);
    let grew = true;
    while (grew) {
      grew = false;
      for (let i = 0; i < this.nodes.length; i++) {
        if (!sub.has(i) && this.nodes[i].parent >= 0 && sub.has(this.nodes[i].parent)) {
          sub.add(i);
          grew = true;
        }
      }
    }
    // Reverse parent pointers along end → child, then attach end to otherEnd.
    const seq: number[] = [];
    for (let k = end; k !== child; k = this.nodes[k].parent) seq.push(k);
    seq.push(child);
    for (let i = seq.length - 1; i > 0; i--) {
      const lower = seq[i]; // closer to child
      const upper = seq[i - 1]; // closer to end
      const wasChecked = this.nodes[upper].checked;
      this.nodes[lower].parent = upper;
      this.nodes[lower].checked = wasChecked;
    }
    this.nodes[end].parent = otherEnd;
    this.nodes[end].checked = true; // the bridge was verified first
    for (const i of sub) this.nodes[i].tree = newTree;
  }

  plan(n: number): P[] | null {
    for (let i = 0; i < n && !this.path; i++) this.step();
    return this.path;
  }

  /** Edges as (parent, child) pairs with their lazy status, for drawing. */
  edges(): { a: number; b: number; checked: boolean; tree: 0 | 1 }[] {
    const out: { a: number; b: number; checked: boolean; tree: 0 | 1 }[] = [];
    for (let i = 0; i < this.nodes.length; i++) if (this.nodes[i].parent >= 0) out.push({ a: this.nodes[i].parent, b: i, checked: this.nodes[i].checked, tree: this.nodes[i].tree });
    return out;
  }
}

/* -------------------------------------------------------------------------- */
/* SRT — Choset §7.3, Algorithm 14                                             */
/* -------------------------------------------------------------------------- */

export interface SrtOptions<P> {
  steer: Steer<P>;
  /** Configurations grown per tree (1 collapses to PRM). */
  nodesPerTree: number;
  /** Close pairs tried with Δ per candidate edge (Alg. 14 lines 7–14). */
  closePairs: number;
  /** Iterations of the bidirectional tree planner per candidate edge (line 15); 0 disables merging. */
  mergeIters: number;
  /** k nearest and r random neighbor trees. */
  k: number;
  r: number;
  /** Which tree planner grows the nodes. */
  grower?: 'rrt' | 'est';
  eta: number;
  /** Roots come from this sampler (Scripted in the degeneration check). */
  rootSampler?: Sampler<P>;
  nn?: 'kd' | 'linear';
}

export interface SrtEdge<P> {
  i: number;
  j: number;
  /** The configurations joined, and whether a merge (not a close pair) did it. */
  qi: NodeId;
  qj: NodeId;
  merged: boolean;
}

/**
 * Sampling-Based Roadmap of Trees: a PRM whose nodes are trees. Each tree's
 * representative is its root; trees are joined by a close-pair Δ check first and
 * a bidirectional merge second. With (1 node, 1 pair, 0 merges) it is C Alg. 6;
 * with no roadmap trees it is a single bidirectional RRT or EST (derivation F.5).
 */
export class Srt<P> {
  readonly trees: Tree<P>[] = [];
  readonly edges: SrtEdge<P>[] = [];
  readonly uf = new UnionFind();
  private readonly rootNn: NearestNeighbors<P>;
  private readonly rootSampler: Sampler<P>;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    readonly opts: SrtOptions<P>,
    readonly rng: Rng,
  ) {
    this.rootNn = makeNn(space, opts.nn ?? 'kd');
    this.rootSampler = opts.rootSampler ?? new Uniform<P>();
  }

  get components(): number {
    return this.uf.count;
  }

  /** Grow a tree from `root` (or a sampled one) to `nodesPerTree` nodes and add it. */
  addTree(root?: P): number | null {
    let q: P | null = root ?? null;
    for (let t = 0; t < 200 && q === null; t++) q = this.rootSampler.sample(this.space, this.free, this.rng);
    if (q === null) return null;
    const tree = this.growTree(q, this.opts.nodesPerTree);
    const i = this.trees.length;
    this.trees.push(tree);
    this.rootNn.insert(i, q);
    this.uf.make();
    return i;
  }

  private growTree(root: P, n: number): Tree<P> {
    const base = { steer: this.opts.steer, eta: this.opts.eta, nn: this.opts.nn };
    if (this.opts.grower === 'est') {
      const est = new Est(this.space, this.free, root, null, { ...base, radius: this.opts.eta }, this.rng);
      let guard = 0;
      while (est.tree.size < n && guard++ < 20 * n) est.extendOnce();
      return est.tree;
    }
    const rrt = new Rrt(this.space, this.free, root, null, { ...base, goalBias: 0 }, this.rng);
    let guard = 0;
    while (rrt.tree.size < n && guard++ < 20 * n) rrt.extendOnce();
    return rrt.tree;
  }

  /** Sample `count` trees. */
  addTrees(count: number): void {
    for (let i = 0; i < count; i++) this.addTree();
  }

  /** N_Ti: k nearest (by root distance) and r random other trees. */
  neighborsOf(i: number): number[] {
    const q = this.trees[i].root;
    const near = this.rootNn.kNearest(q, this.opts.k, i).map((nb) => nb.idx);
    const set = new Set(near);
    const others = this.trees.map((_, j) => j).filter((j) => j !== i && !set.has(j));
    for (let t = 0; t < this.opts.r && others.length; t++) {
      const k = Math.floor(this.rng.next() * others.length);
      set.add(others[k]);
      others.splice(k, 1);
    }
    return [...set];
  }

  /** Alg. 14 lines 5–18 for one pair. */
  tryConnect(i: number, j: number): boolean {
    if (this.uf.same(i, j)) return false;
    const Ti = this.trees[i];
    const Tj = this.trees[j];
    // Close pairs: random configurations of Ti against their nearest in Tj.
    for (let c = 0; c < this.opts.closePairs; c++) {
      const qi = this.opts.closePairs === 1 && Ti.size === 1 ? 0 : Math.floor(this.rng.next() * Ti.size);
      const nb = Tj.nearest(Ti.nodes[qi].q);
      if (this.opts.steer.steer(this.space, this.free, Ti.nodes[qi].q, Tj.nodes[nb.idx].q)) {
        this.edges.push({ i, j, qi, qj: nb.idx, merged: false });
        this.uf.union(i, j);
        return true;
      }
    }
    if (this.opts.mergeIters > 0) {
      const m = this.mergeTrees(Ti, Tj, this.opts.mergeIters);
      if (m) {
        this.edges.push({ i, j, qi: m[0], qj: m[1], merged: true });
        this.uf.union(i, j);
        return true;
      }
    }
    return false;
  }

  /** Alg. 13 between two existing trees (nodes are added to both). */
  private mergeTrees(Ti: Tree<P>, Tj: Tree<P>, iters: number): [NodeId, NodeId] | null {
    let A = Ti;
    let B = Tj;
    let swapped = false;
    const u = new Uniform<P>();
    for (let it = 0; it < iters; it++) {
      const qRand = u.sample(this.space, this.free, this.rng);
      if (qRand === null) continue;
      const na = A.nearest(qRand);
      const s1 = extend(this.opts.steer, this.space, this.free, A.nodes[na.idx].q, qRand, this.opts.eta);
      if (s1.kind === 'trapped') continue;
      const ida = A.add(s1.q, na.idx);
      const nb = B.nearest(s1.q);
      const s2 = extend(this.opts.steer, this.space, this.free, B.nodes[nb.idx].q, s1.q, this.opts.eta);
      if (s2.kind === 'reached') {
        const idb = B.add(s2.q, nb.idx);
        return swapped ? [idb, ida] : [ida, idb];
      }
      if (s2.kind === 'advanced') B.add(s2.q, nb.idx);
      [A, B] = [B, A];
      swapped = !swapped;
    }
    return null;
  }

  /** Alg. 14 over every tree. */
  connectAll(): void {
    for (let i = 0; i < this.trees.length; i++) for (const j of this.neighborsOf(i)) this.tryConnect(i, j);
  }

  /** Answering a query (§7.3): grow T_init, T_goal, add them, connect, then walk the roadmap of trees. */
  query(start: P, goal: P): P[] | null {
    const i0 = this.addTree(start);
    const i1 = this.addTree(goal);
    if (i0 === null || i1 === null) return null;
    for (const i of [i0, i1]) for (const j of this.neighborsOf(i)) this.tryConnect(i, j);
    if (this.trees.length === 2 && !this.uf.same(i0, i1)) this.tryConnect(i0, i1);
    if (!this.uf.same(i0, i1)) return null;
    return this.treePath(i0, 0, i1, 0);
  }

  /** BFS over the roadmap of trees, concatenating within-tree paths. */
  treePath(ti: number, ni: NodeId, tj: number, nj: NodeId): P[] | null {
    const adj = new Map<number, SrtEdge<P>[]>();
    for (const e of this.edges) {
      (adj.get(e.i) ?? adj.set(e.i, []).get(e.i)!).push(e);
      (adj.get(e.j) ?? adj.set(e.j, []).get(e.j)!).push(e);
    }
    const prev = new Map<number, SrtEdge<P> | null>([[ti, null]]);
    const queue = [ti];
    while (queue.length) {
      const t = queue.shift()!;
      if (t === tj) break;
      for (const e of adj.get(t) ?? []) {
        const o = e.i === t ? e.j : e.i;
        if (!prev.has(o)) {
          prev.set(o, e);
          queue.push(o);
        }
      }
    }
    if (!prev.has(tj)) return null;
    // Unwind the tree sequence.
    const seq: { tree: number; enter: NodeId; exit: NodeId }[] = [];
    let t = tj;
    let exit = nj;
    while (t !== ti) {
      const e = prev.get(t)!;
      const enter = e.i === t ? e.qi : e.qj;
      seq.push({ tree: t, enter, exit });
      const o = e.i === t ? e.j : e.i;
      exit = e.i === t ? e.qj : e.qi;
      t = o;
    }
    seq.push({ tree: ti, enter: ni, exit });
    seq.reverse();
    const out: P[] = [];
    for (const s of seq) {
      const piece = this.trees[s.tree].pathBetween(s.enter, s.exit);
      for (let k = out.length ? 1 : 0; k < piece.length; k++) out.push(piece[k]);
    }
    return out;
  }

  /** Derivation F.5: SRT with one node per tree, one close pair, no merging — C Alg. 6. */
  static prmLike<P>(space: Chart<P>, free: FreeSpace<P>, steer: Steer<P>, k: number, rng: Rng, rootSampler?: Sampler<P>): Srt<P> {
    return new Srt(space, free, { steer, nodesPerTree: 1, closePairs: 1, mergeIters: 0, k, r: 0, eta: 1, rootSampler }, rng);
  }

  /** Derivation F.5: no roadmap trees, no close pairs — the query is a bidirectional RRT. */
  static rrtLike<P>(space: Chart<P>, free: FreeSpace<P>, steer: Steer<P>, eta: number, mergeIters: number, rng: Rng): Srt<P> {
    return new Srt(space, free, { steer, nodesPerTree: 1, closePairs: 0, mergeIters, k: 1, r: 0, eta }, rng);
  }

  /** Every configuration of every tree with its tree index, for drawing. */
  allNodes(): { tree: number; q: P }[] {
    const out: { tree: number; q: P }[] = [];
    this.trees.forEach((t, i) => t.nodes.forEach((n) => out.push({ tree: i, q: n.q })));
    return out;
  }
}

/* -------------------------------------------------------------------------- */
/* KPIECE — Şucan & Kavraki (WAFR 2008): cell importance over a projection       */
/* -------------------------------------------------------------------------- */

export interface KpieceCell {
  key: string;
  motions: NodeId[];
  /** Iteration at which the cell was first covered. */
  created: number;
  selections: number;
  /**
   * Productivity, decayed multiplicatively: ×0.9 after a successful expansion,
   * ×0.45 after a failed one, so a cell that keeps being chosen without
   * yielding motions loses its claim on the planner's attention.
   */
  score: number;
  /** Successful / attempted expansions from this cell, for the readout. */
  good: number;
  attempts: number;
}

export interface KpieceOptions<P> {
  steer: Steer<P>;
  /** Projection cell size (over the first two chart coordinates). */
  cell: number;
  /** Expansion radius: new samples are drawn in the η-ball of the chosen motion (the EST push). */
  eta: number;
  goalRadius?: number;
  /** Probability of expanding from an exterior (under-neighbored) cell. Default 0.9. */
  exteriorBias?: number;
  nn?: 'kd' | 'linear';
}

/**
 * The projection-based descendant of EST's grid trick. Tree motions are
 * binned by a low-dimensional projection; the cell of greatest importance
 *   I(c) = score(c) / (selections(c) · (1 + neighbors(c)) · coverage(c))
 * — productive, rarely selected, isolated and sparse cells win — is chosen
 * (exterior cells with probability `exteriorBias`), a motion in it is picked
 * at random, and a new configuration is sampled in the η-ball around that
 * motion and joined to it by Δ, exactly as EST pushes. A new cell's score
 * starts at 1 + log(iteration), so the frontier a recent expansion opened is
 * favored. Single-level here (the paper uses several); the chapter labels it
 * modern, not Choset's.
 */
export class Kpiece<P> {
  readonly tree: Tree<P>;
  readonly cells = new Map<string, KpieceCell>();
  iterations = 0;
  goalNode: NodeId | null = null;
  solvedAt: number | null = null;
  rejected = 0;
  /** Which cell was expanded last (for the widget's highlight). */
  lastCell: string | null = null;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    root: P,
    readonly goal: P | null,
    readonly opts: KpieceOptions<P>,
    readonly rng: Rng,
  ) {
    this.tree = new Tree(space, root, opts.nn ?? 'kd');
    this.addMotion(0);
  }

  get solved(): boolean {
    return this.goalNode !== null;
  }

  private key(q: P): string {
    const c = this.space.coords(q);
    return `${Math.floor(c[0] / this.opts.cell)},${Math.floor((c[1] ?? 0) / this.opts.cell)}`;
  }

  private addMotion(id: NodeId): void {
    const k = this.key(this.tree.nodes[id].q);
    const c = this.cells.get(k);
    if (c) c.motions.push(id);
    else this.cells.set(k, { key: k, motions: [id], created: this.iterations + 1, selections: 1, score: 1 + Math.log(this.iterations + 1), good: 0, attempts: 0 });
  }

  /** Occupied cells among the four axis neighbors; a cell with all four is interior. */
  neighbors(c: KpieceCell): number {
    const [i, j] = c.key.split(',').map(Number);
    let n = 0;
    for (const [di, dj] of [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
    ]) {
      if (this.cells.has(`${i + di},${j + dj}`)) n++;
    }
    return n;
  }

  importance(c: KpieceCell): number {
    return c.score / (c.selections * (1 + this.neighbors(c)) * c.motions.length);
  }

  /** The most important cell, from the exterior ones with probability `exteriorBias`. */
  selectCell(): KpieceCell {
    const all = [...this.cells.values()];
    const exterior = all.filter((c) => this.neighbors(c) < 4);
    const pool = exterior.length && this.rng.next() < (this.opts.exteriorBias ?? 0.9) ? exterior : all;
    let best = pool[0];
    let bi = -Infinity;
    for (const c of pool) {
      const I = this.importance(c);
      if (I > bi) {
        bi = I;
        best = c;
      }
    }
    return best;
  }

  extendOnce(): { cell: string; added: NodeId | null } {
    this.iterations++;
    this.tree.clock = this.iterations;
    const cell = this.selectCell();
    this.lastCell = cell.key;
    cell.selections++;
    cell.attempts++;
    const from = cell.motions[Math.floor(this.rng.next() * cell.motions.length)];
    const qFrom = this.tree.nodes[from].q;
    // The EST push: a configuration near the chosen motion, joined by Δ.
    const qNew = sampleBall(this.space, qFrom, this.opts.eta, this.rng);
    const ok = this.free.isFree(qNew) && this.opts.steer.steer(this.space, this.free, qFrom, qNew) !== null;
    if (!ok) {
      this.rejected++;
      cell.score *= 0.45;
      return { cell: cell.key, added: null };
    }
    cell.score *= 0.9;
    cell.good++;
    const id = this.tree.add(qNew, from);
    this.addMotion(id);
    if (this.goal && this.goalNode === null) {
      const gr = this.opts.goalRadius ?? this.opts.eta;
      if (this.space.dist(qNew, this.goal) <= gr && this.opts.steer.steer(this.space, this.free, qNew, this.goal)) {
        this.goalNode = id;
        this.solvedAt = this.iterations;
      }
    }
    return { cell: cell.key, added: id };
  }

  plan(n: number): P[] | null {
    for (let i = 0; i < n && !this.solved; i++) this.extendOnce();
    return this.path();
  }

  path(): P[] | null {
    if (this.goalNode === null || !this.goal) return null;
    return [...this.tree.pathToRoot(this.goalNode), this.goal];
  }
}

/* -------------------------------------------------------------------------- */
/* The §3 micro-example as data                                                */
/* -------------------------------------------------------------------------- */

/** Chapter 12 §3: root (1, 1), η = 1, the three replayed samples. */
export const THREE_EXTENSION_ROOT: number[] = [1, 1];
export const THREE_EXTENSION_SAMPLES: number[][] = [
  [5, 4],
  [2, 6],
  [4, 1.5],
];

/** An independent dense recheck of a path: every `step` along every segment is asked of `free`. */
export function pathIsFree<P>(space: Chart<P>, free: FreeSpace<P>, path: readonly P[], step = 0.005): boolean {
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1];
    const b = path[i];
    const n = Math.max(2, Math.ceil(space.dist(a, b) / step));
    for (let t = 0; t <= n; t++) if (!free.isFree(space.interpolate(a, b, t / n))) return false;
  }
  return true;
}
