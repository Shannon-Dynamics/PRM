/**
 * Optimal sampling-based planning — RRT*, PRM*, Informed RRT*, FMT*, BIT*
 * (Chapter 13; nothing of this is in Choset, which is the point).
 *
 * Chapters 11 and 12 bought feasibility with randomness and proved the price.
 * They said nothing about the *quality* of a path, and RRT's first path is a
 * jagged accident of sample order that more samples never repair (Karaman &
 * Frazzoli 2011). The repair is two local operations — choose-parent and
 * rewire — inside a ball of radius r_n = γ (log n / n)^{1/d} that shrinks
 * just slowly enough: fast enough that |Q_near| stays Θ(log n), slow enough
 * that Chapter 11's tiling balls keep receiving samples forever.
 *
 * `CostTree` wraps Chapter 12's `Tree` with costs and child lists; it does not
 * fork it. `RrtStar` with `rewire: false` is plain RRT fed the same samples,
 * which is how w13.1 puts the two side by side on one seeded stream.
 */

import type { Rng } from '../prob/rng';
import type { Chart } from '../manifold/manifold';
import { Rd } from '../manifold/rd';
import { KeyedHeap } from '../search/heap';
import { unitBallVolume } from './analysis';
import type { FreeSpace } from './free-space';
import { makeNn, type Neighbor } from './kdtree';
import { Prm, type Connect } from './prm';
import { Uniform, randomDirection, type Sampler } from './sampler';
import { extend, type Steer } from './steer';
import { Tree, type NodeId } from './tree';

/* -------------------------------------------------------------------------- */
/* Radii                                                                       */
/* -------------------------------------------------------------------------- */

/** r_n = γ (log n / n)^{1/d}. γ is a parameter, not hidden, so a widget can violate the bound. */
export function rN(n: number, d: number, gamma: number): number {
  if (n < 2) return Infinity;
  return gamma * Math.pow(Math.log(n) / n, 1 / d);
}

/** γ* = 2 (1 + 1/d)^{1/d} (μ(𝒬_free)/ζ_d)^{1/d} — Karaman & Frazzoli (2011), Theorem 38. */
export function gammaStarKf2011(d: number, muFree: number): number {
  return 2 * Math.pow(1 + 1 / d, 1 / d) * Math.pow(muFree / unitBallVolume(d), 1 / d);
}

/** γ* = (2 (1 + 1/d))^{1/d} (μ(𝒬_free)/ζ_d)^{1/d} — Solovey, Janson, Schmerling, Frazzoli & Pavone (2020). */
export function gammaStarSolovey2020(d: number, muFree: number): number {
  return Math.pow(2 * (1 + 1 / d), 1 / d) * Math.pow(muFree / unitBallVolume(d), 1 / d);
}

/** γ* for FMT*: 2 (1/d)^{1/d} (μ(𝒬_free)/ζ_d)^{1/d} — Janson et al. (2015). */
export function gammaStarFmt(d: number, muFree: number): number {
  return 2 * Math.pow(1 / d, 1 / d) * Math.pow(muFree / unitBallVolume(d), 1 / d);
}

/** k* = e (1 + 1/d), the k-nearest constant. */
export function kStar(d: number): number {
  return Math.E * (1 + 1 / d);
}

/** k_n = ⌈k* log n⌉. */
export function kN(n: number, d: number, k = kStar(d)): number {
  if (n < 2) return 1;
  return Math.ceil(k * Math.log(n));
}

export type RadiusConstant = '2011' | '2020';

export function gammaStar(d: number, muFree: number, which: RadiusConstant): number {
  return which === '2011' ? gammaStarKf2011(d, muFree) : gammaStarSolovey2020(d, muFree);
}

/* -------------------------------------------------------------------------- */
/* CostTree                                                                    */
/* -------------------------------------------------------------------------- */

export interface Rewire {
  node: NodeId;
  from: number;
  to: number;
  oldParent: NodeId;
  newParent: NodeId;
  /** Descendants whose cost was propagated. */
  propagated: number;
}

/** Chapter 12's tree plus cost-to-come and child lists. */
export class CostTree<P> {
  readonly inner: Tree<P>;
  readonly cost: number[] = [0];
  readonly children: NodeId[][] = [[]];

  constructor(
    readonly space: Chart<P>,
    root: P,
    nnKind: 'kd' | 'linear' = 'kd',
  ) {
    this.inner = new Tree(space, root, nnKind);
  }

  get size(): number {
    return this.inner.size;
  }

  q(v: NodeId): P {
    return this.inner.nodes[v].q;
  }

  parent(v: NodeId): NodeId {
    return this.inner.nodes[v].parent;
  }

  nearest(q: P): Neighbor {
    return this.inner.nearest(q);
  }

  /** Q_near as a ball — Chapter 11's `KdTree.within`. */
  near(q: P, r: number): Neighbor[] {
    return this.inner.within(q, r);
  }

  kNear(q: P, k: number): Neighbor[] {
    return this.inner.kNearest(q, k);
  }

  add(q: P, parent: NodeId, edgeCost: number): NodeId {
    const id = this.inner.add(q, parent);
    this.cost.push(this.cost[parent] + edgeCost);
    this.children.push([]);
    this.children[parent].push(id);
    return id;
  }

  /** Change v's parent and propagate the cost change through its subtree (DFS). */
  reparent(v: NodeId, newParent: NodeId, edgeCost: number): number {
    const old = this.inner.nodes[v].parent;
    if (old >= 0) {
      const list = this.children[old];
      const k = list.indexOf(v);
      if (k >= 0) list.splice(k, 1);
    }
    this.inner.nodes[v].parent = newParent;
    this.inner.nodes[v].depth = this.inner.nodes[newParent].depth + 1;
    this.children[newParent].push(v);
    const delta = this.cost[newParent] + edgeCost - this.cost[v];
    let touched = 0;
    const stack = [v];
    while (stack.length) {
      const u = stack.pop()!;
      this.cost[u] += delta;
      touched++;
      for (const c of this.children[u]) {
        this.inner.nodes[c].depth = this.inner.nodes[u].depth + 1;
        stack.push(c);
      }
    }
    return touched - 1;
  }

  pathToRoot(v: NodeId): P[] {
    return this.inner.pathToRoot(v);
  }
}

/* -------------------------------------------------------------------------- */
/* Anytime contract                                                            */
/* -------------------------------------------------------------------------- */

export interface Solution<P> {
  path: P[];
  cost: number;
}

export interface CostSample {
  /** Iterations (samples) spent when this cost was reached. */
  n: number;
  /** Wall-clock milliseconds since the planner started, when measured. */
  t: number;
  cost: number;
}

/**
 * "Deadline in, best-so-far out." One unit of work per `step`, which returns
 * the new best cost when it improved; `best()` is always available. Chapter 23
 * consumes exactly this.
 */
export interface Anytime<P> {
  readonly name: string;
  step(): number | null;
  best(): Solution<P> | null;
  readonly iterations: number;
  readonly costHistory: CostSample[];
}

/** Run `step` until the clock passes `deadlineMs` (relative to `now()` at entry). */
export function runUntil<P>(p: Anytime<P>, deadlineMs: number, now: () => number = () => performance.now()): Solution<P> | null {
  const t0 = now();
  while (now() - t0 < deadlineMs) p.step();
  return p.best();
}

/* -------------------------------------------------------------------------- */
/* RRT* (and RRT on the same samples)                                           */
/* -------------------------------------------------------------------------- */

export type Neighborhood = { kind: 'radius'; gamma: number } | { kind: 'kNearest'; kStar: number };

export interface RrtStarOptions<P> {
  steer: Steer<P>;
  sampler?: Sampler<P>;
  eta: number;
  goalBias?: number;
  goalRadius?: number;
  nbhd: Neighborhood;
  /** false: plain RRT — identical sampling, parent := q_near, no rewire. */
  rewire?: boolean;
  nn?: 'kd' | 'linear';
  /** Wall clock for cost-history timestamps. */
  now?: () => number;
}

export interface RrtStarIteration<P> {
  qRand: P | null;
  qNew: P | null;
  added: NodeId | null;
  nearCount: number;
  rewires: Rewire[];
  improved: number | null;
}

/**
 * One iteration: sample → nearest → extend → Q_near → choose-parent → rewire →
 * goal test. The goal is reached through any node within `goalRadius` whose
 * segment to the goal is free; its cost is Cost(v) + dist(v, goal), so a
 * rewire deep in the tree lowers the incumbent without touching the goal.
 */
export class RrtStar<P> implements Anytime<P> {
  readonly name: string;
  readonly tree: CostTree<P>;
  readonly sampler: Sampler<P>;
  readonly rewireEnabled: boolean;
  iterations = 0;
  trapped = 0;
  rewires = 0;
  propagated = 0;
  /** |Q_near| at every successful insertion — hovers near log n. */
  readonly nearSizes: number[] = [];
  readonly costHistory: CostSample[] = [];
  /** Nodes with a verified free segment to the goal. */
  readonly goalNodes: NodeId[] = [];
  bestNode: NodeId | null = null;
  bestCost = Infinity;
  solvedAt: number | null = null;
  /** The radius or k used at the last insertion, for the readout. */
  lastRadius = Infinity;
  private readonly t0: number;
  private readonly now: () => number;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    readonly start: P,
    readonly goal: P,
    readonly opts: RrtStarOptions<P>,
    readonly rng: Rng,
  ) {
    this.tree = new CostTree(space, start, opts.nn ?? 'kd');
    this.sampler = opts.sampler ?? new Uniform<P>();
    this.rewireEnabled = opts.rewire ?? true;
    this.name = this.rewireEnabled ? 'RRT*' : 'RRT';
    this.now = opts.now ?? (() => (typeof performance !== 'undefined' ? performance.now() : Date.now()));
    this.t0 = this.now();
  }

  get steer(): Steer<P> {
    return this.opts.steer;
  }

  get solved(): boolean {
    return this.bestNode !== null;
  }

  /** The neighborhood for a tree of size n (before insertion of the new node). */
  neighborhood(q: P): Neighbor[] {
    const n = this.tree.size + 1;
    const nb = this.opts.nbhd;
    if (nb.kind === 'radius') {
      // The η cap: never ask for neighbors farther than the longest edge the steer will make.
      const r = Math.min(rN(n, this.space.dim, nb.gamma), this.opts.eta);
      this.lastRadius = r;
      return this.tree.near(q, r);
    }
    const k = kN(n, this.space.dim, nb.kStar);
    this.lastRadius = k;
    return this.tree.kNear(q, k);
  }

  /**
   * Insert q_new with a given neighbor set: choose-parent, add, rewire. The
   * near set must contain a node whose segment to q_new is already known free
   * (`knownFree`), or every candidate is checked.
   */
  insert(qNew: P, near: Neighbor[], knownFree: NodeId | null): { added: NodeId; rewires: Rewire[] } {
    // Choose-parent: argmin Cost(v) + c(v, q_new) over near, subject to Δ.
    const sorted = near.slice().sort((a, b) => this.tree.cost[a.idx] + a.dist - (this.tree.cost[b.idx] + b.dist) || a.idx - b.idx);
    let parent: NodeId = knownFree ?? sorted[0].idx;
    let edge = knownFree !== null ? this.space.dist(this.tree.q(knownFree), qNew) : sorted[0].dist;
    if (this.rewireEnabled) {
      for (const c of sorted) {
        const total = this.tree.cost[c.idx] + c.dist;
        if (total >= this.tree.cost[parent] + edge - 1e-12) break; // sorted: nothing better remains
        if (c.idx === knownFree || this.steer.steer(this.space, this.free, this.tree.q(c.idx), qNew)) {
          parent = c.idx;
          edge = c.dist;
          break;
        }
      }
    }
    const id = this.tree.add(qNew, parent, edge);
    const rewires: Rewire[] = [];
    if (this.rewireEnabled) {
      const cNew = this.tree.cost[id];
      for (const c of near) {
        if (c.idx === parent) continue;
        const through = cNew + c.dist;
        if (through < this.tree.cost[c.idx] - 1e-12 && this.steer.steer(this.space, this.free, qNew, this.tree.q(c.idx))) {
          const from = this.tree.cost[c.idx];
          const oldParent = this.tree.parent(c.idx);
          const propagated = this.tree.reparent(c.idx, id, c.dist);
          rewires.push({ node: c.idx, from, to: this.tree.cost[c.idx], oldParent, newParent: id, propagated });
          this.rewires++;
          this.propagated += propagated;
        }
      }
    }
    return { added: id, rewires };
  }

  iterate(): RrtStarIteration<P> {
    this.iterations++;
    this.tree.inner.clock = this.iterations;
    const none = (qRand: P | null): RrtStarIteration<P> => ({ qRand, qNew: null, added: null, nearCount: 0, rewires: [], improved: null });
    let qRand: P | null;
    if ((this.opts.goalBias ?? 0) > 0 && this.rng.next() < (this.opts.goalBias ?? 0)) qRand = this.goal;
    else qRand = this.sampler.sample(this.space, this.free, this.rng);
    if (qRand === null) return none(null);
    const nearest = this.tree.nearest(qRand);
    const st = extend(this.steer, this.space, this.free, this.tree.q(nearest.idx), qRand, this.opts.eta);
    if (st.kind === 'trapped') {
      this.trapped++;
      return none(qRand);
    }
    const qNew = st.q;
    let near = this.rewireEnabled ? this.neighborhood(qNew) : [];
    if (!near.some((n) => n.idx === nearest.idx)) near = [{ idx: nearest.idx, dist: this.space.dist(this.tree.q(nearest.idx), qNew) }, ...near];
    const { added, rewires } = this.insert(qNew, near, nearest.idx);
    this.nearSizes.push(near.length);
    // Goal test for the new node.
    const gr = this.opts.goalRadius ?? this.opts.eta;
    const dg = this.space.dist(qNew, this.goal);
    if (dg <= gr && (dg < 1e-12 || this.steer.steer(this.space, this.free, qNew, this.goal))) this.goalNodes.push(added);
    const improved = this.refreshBest();
    return { qRand, qNew, added, nearCount: near.length, rewires, improved };
  }

  /** Recompute the incumbent over the goal-connected nodes (their costs may have dropped by propagation). */
  private refreshBest(): number | null {
    let best = Infinity;
    let node: NodeId | null = null;
    for (const v of this.goalNodes) {
      const c = this.tree.cost[v] + this.space.dist(this.tree.q(v), this.goal);
      if (c < best) {
        best = c;
        node = v;
      }
    }
    if (node === null) return null;
    if (best < this.bestCost - 1e-12) {
      if (this.bestNode === null) this.solvedAt = this.iterations;
      this.bestCost = best;
      this.bestNode = node;
      this.costHistory.push({ n: this.iterations, t: this.now() - this.t0, cost: best });
      return best;
    }
    return null;
  }

  step(): number | null {
    return this.iterate().improved;
  }

  best(): Solution<P> | null {
    if (this.bestNode === null) return null;
    return { path: [...this.tree.pathToRoot(this.bestNode), this.goal], cost: this.bestCost };
  }

  /** The current best path, or null. */
  bestPath(): P[] | null {
    return this.best()?.path ?? null;
  }
}

/* -------------------------------------------------------------------------- */
/* PRM*                                                                        */
/* -------------------------------------------------------------------------- */

export interface PrmStarOptions<P> {
  steer: Steer<P>;
  sampler?: Sampler<P>;
  nbhd: Neighborhood;
  nn?: 'kd' | 'linear';
}

/**
 * Chapter 11's `Prm` with the connection rule tied to n: a ball of radius r_n,
 * or the k_n nearest. Each new node is connected to the current roadmap under
 * the current radius (the incremental form); the expected number of edges is
 * O(n log n), which is what makes the roadmap both connected and affordable.
 */
export class PrmStar<P> {
  readonly prm: Prm<P>;
  private readonly connect: Connect;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    readonly opts: PrmStarOptions<P>,
    readonly rng: Rng,
  ) {
    this.connect = opts.nbhd.kind === 'radius' ? { kind: 'radius', r: Infinity } : { kind: 'kNearest', k: 1 };
    this.prm = new Prm(space, free, { sampler: opts.sampler ?? new Uniform<P>(), steer: opts.steer, connect: this.connect, nn: opts.nn }, rng);
  }

  get n(): number {
    return this.prm.n;
  }

  /** The rule in force for the next insertion. */
  currentRule(): { r?: number; k?: number } {
    const n = this.prm.n + 1;
    const nb = this.opts.nbhd;
    if (nb.kind === 'radius') return { r: rN(n, this.space.dim, nb.gamma) };
    return { k: kN(n, this.space.dim, nb.kStar) };
  }

  addNode(q: P): number {
    this.refreshRule();
    const i = this.prm.addNode(q);
    this.prm.connectNode(i, this.prm.candidates(i));
    return i;
  }

  addSample(): number | null {
    this.refreshRule();
    return this.prm.addSample();
  }

  grow(n: number): void {
    let guard = 0;
    while (this.prm.n < n && guard++ < 50 * n + 1000) this.addSample();
  }

  private refreshRule(): void {
    const rule = this.currentRule();
    if (this.connect.kind === 'radius' && rule.r !== undefined) this.connect.r = rule.r;
    if (this.connect.kind === 'kNearest' && rule.k !== undefined) this.connect.k = rule.k;
  }

  query(start: P, goal: P, k = 10) {
    return this.prm.query(start, goal, k);
  }
}

/* -------------------------------------------------------------------------- */
/* Informed sampling — Gammell, Srinivasa & Barfoot (2014)                     */
/* -------------------------------------------------------------------------- */

/**
 * The prolate hyperspheroid X_f̂ = {x : dist(start, x) + dist(x, goal) ≤ c_best}.
 * In ℝᵈ it is sampled directly: a uniform point of the unit ball, scaled by
 * diag(c_best/2, √(c_best² − c_min²)/2, …), rotated so e₁ points along
 * goal − start, translated to the midpoint, rejected outside the chart's box.
 * On any other chart (T² through the seam) it falls back to rejection on the
 * manifold metric, which is exact and merely slower. It is a Chapter 11
 * `Sampler`, so Informed RRT* is RRT* with a different sampler.
 */
export class InformedSampler<P> implements Sampler<P> {
  readonly name = 'informed';
  attempts = 0;
  accepted = 0;
  /** Samples drawn after a solution that fell outside the box (ℝᵈ) or the set (rejection). */
  rejectedOutside = 0;
  cBest = Infinity;
  readonly cMin: number;
  private readonly center: number[] | null;
  private readonly axis: number[] | null;
  private readonly fallback = new Uniform<P>();

  constructor(
    readonly space: Chart<P>,
    readonly start: P,
    readonly goal: P,
  ) {
    this.cMin = space.dist(start, goal);
    if (space instanceof Rd) {
      const a = space.coords(start);
      const b = space.coords(goal);
      this.center = a.map((x, i) => (x + b[i]) / 2);
      const dir = b.map((x, i) => x - a[i]);
      const n = Math.hypot(...dir);
      this.axis = n > 1e-12 ? dir.map((x) => x / n) : null;
    } else {
      this.center = null;
      this.axis = null;
    }
  }

  update(cBest: number): void {
    this.cBest = cBest;
  }

  /** x ∈ X_f̂ under the manifold metric (the geodesic through the seam on T²). */
  contains(q: P): boolean {
    return this.space.dist(this.start, q) + this.space.dist(q, this.goal) <= this.cBest + 1e-12;
  }

  /** Semi-axes: transverse a = c_best/2 along the foci, conjugate b = √(c_best² − c_min²)/2. */
  semiAxes(): { a: number; b: number } {
    return { a: this.cBest / 2, b: Math.sqrt(Math.max(0, this.cBest * this.cBest - this.cMin * this.cMin)) / 2 };
  }

  /** μ(X_f̂) of the full hyperspheroid, ζ_d · a · b^{d−1} — the textbook formula, box ignored. */
  measureUnclipped(): number {
    if (!Number.isFinite(this.cBest)) return Infinity;
    const { a, b } = this.semiAxes();
    return unitBallVolume(this.space.dim) * a * Math.pow(b, this.space.dim - 1);
  }

  /**
   * μ(X_f̂ ∩ 𝒬): the part of the hyperspheroid the planner can actually sample.
   * In the plane the ellipse is clipped against the chart's box numerically
   * (`ellipseBoxArea`); elsewhere the unclipped formula is used. Early on the
   * ellipse spills far outside the room and the two differ a lot — which is
   * exactly the share of draws `rejectedOutside` counts.
   */
  measure(): number {
    if (!Number.isFinite(this.cBest)) return Infinity;
    if (this.center && this.axis && this.space instanceof Rd && this.space.dim === 2) {
      const { a, b } = this.semiAxes();
      const bx = this.space.bounds;
      return ellipseBoxArea(this.center, this.axis, a, b, { lo: [bx.lo[0], bx.lo[1]], hi: [bx.hi[0], bx.hi[1]] });
    }
    return this.measureUnclipped();
  }

  /** μ(X_f̂ ∩ 𝒬)/μ(𝒬) — "how much of the space is still worth sampling" (w13.2's readout). */
  measureRatio(muSpace: number): number {
    if (!Number.isFinite(this.cBest)) return 1;
    return Math.min(1, this.measure() / muSpace);
  }

  sample(space: Chart<P>, free: FreeSpace<P>, rng: Rng): P | null {
    this.attempts++;
    if (!Number.isFinite(this.cBest)) {
      const q = this.fallback.sample(space, free, rng);
      if (q !== null) this.accepted++;
      return q;
    }
    let q: P;
    if (this.center && this.axis && space instanceof Rd) {
      const d = space.dim;
      // Uniform in the unit ball.
      const dir = randomDirection(d, rng);
      const rad = Math.pow(rng.next(), 1 / d);
      const u = dir.map((x) => x * rad);
      // Scale: transverse semi-axis along e₁, conjugate semi-axes elsewhere.
      const a = this.cBest / 2;
      const b = Math.sqrt(Math.max(0, this.cBest * this.cBest - this.cMin * this.cMin)) / 2;
      const s = u.map((x, i) => (i === 0 ? a * x : b * x));
      // Rotate e₁ → axis by a Householder reflection H = I − 2vvᵀ, v ∝ e₁ − axis (a reflection maps the ball correctly).
      const v = this.axis.map((x, i) => (i === 0 ? 1 : 0) - x);
      const vn = Math.hypot(...v);
      let r = s;
      if (vn > 1e-12) {
        const vv = v.map((x) => x / vn);
        const dot = s.reduce((acc, x, i) => acc + x * vv[i], 0);
        r = s.map((x, i) => x - 2 * dot * vv[i]);
      }
      const c = r.map((x, i) => x + this.center![i]);
      for (let i = 0; i < d; i++) {
        if (c[i] < space.bounds.lo[i] || c[i] > space.bounds.hi[i]) {
          this.rejectedOutside++;
          return null;
        }
      }
      q = space.fromCoords(c);
    } else {
      q = space.sample(rng);
      if (!this.contains(q)) {
        this.rejectedOutside++;
        return null;
      }
    }
    if (!free.isFree(q)) return null;
    this.accepted++;
    return q;
  }
}

/**
 * Area of the ellipse with the given center, unit transverse `axis` and
 * semi-axes a (along the axis), b (across it), clipped to an axis-aligned box.
 * Slices perpendicular to the transverse axis are segments; each is clipped to
 * the box (Liang–Barsky) and the lengths are integrated by Simpson's rule in
 * the angle φ with u = a sin φ, which makes the integrand smooth at the tips.
 */
export function ellipseBoxArea(center: readonly number[], axis: readonly number[], a: number, b: number, box: { lo: [number, number]; hi: [number, number] }, slices = 512): number {
  if (!(a > 0) || !(b > 0)) return 0;
  const perp = [-axis[1], axis[0]];
  const chord = (phi: number): number => {
    const u = a * Math.sin(phi);
    const h = b * Math.cos(phi);
    const mx = center[0] + u * axis[0];
    const my = center[1] + u * axis[1];
    // Segment from t = −h to t = +h along perp; clip t to the box on both axes.
    let t0 = -h;
    let t1 = h;
    for (let k = 0; k < 2; k++) {
      const m = k === 0 ? mx : my;
      const d = perp[k];
      const lo = box.lo[k];
      const hi = box.hi[k];
      if (Math.abs(d) < 1e-15) {
        if (m < lo || m > hi) return 0;
        continue;
      }
      let ta = (lo - m) / d;
      let tb = (hi - m) / d;
      if (ta > tb) [ta, tb] = [tb, ta];
      t0 = Math.max(t0, ta);
      t1 = Math.min(t1, tb);
      if (t0 >= t1) return 0;
    }
    return (t1 - t0) * a * Math.cos(phi); // × du/dφ
  };
  const n = slices % 2 === 0 ? slices : slices + 1;
  const hstep = Math.PI / n;
  let sum = chord(-Math.PI / 2) + chord(Math.PI / 2);
  for (let i = 1; i < n; i++) sum += (i % 2 === 1 ? 4 : 2) * chord(-Math.PI / 2 + i * hstep);
  return (sum * hstep) / 3;
}

/** RRT* whose sampler is the informed set, updated on every improvement. */
export class InformedRrtStar<P> extends RrtStar<P> {
  readonly informed: InformedSampler<P>;
  readonly name = 'Informed RRT*';

  constructor(space: Chart<P>, free: FreeSpace<P>, start: P, goal: P, opts: Omit<RrtStarOptions<P>, 'sampler'>, rng: Rng) {
    const informed = new InformedSampler(space, start, goal);
    super(space, free, start, goal, { ...opts, sampler: informed }, rng);
    this.informed = informed;
  }

  iterate(): RrtStarIteration<P> {
    const r = super.iterate();
    if (r.improved !== null) this.informed.update(r.improved);
    return r;
  }
}

/* -------------------------------------------------------------------------- */
/* FMT* — Janson, Schmerling, Clark & Pavone (2015)                            */
/* -------------------------------------------------------------------------- */

export interface FmtOptions<P> {
  steer: Steer<P>;
  n: number;
  gamma: number;
  sampler?: Sampler<P>;
  nn?: 'kd' | 'linear';
  now?: () => number;
}

/**
 * Fast Marching Tree: one batch of n samples, a cost-ordered open set, lazy
 * dynamic programming. The frontier node z of least cost-to-come is expanded:
 * each unvisited x within r_n is given the parent y ∈ open ∩ B(x, r_n) that is
 * locally optimal *ignoring obstacles*, and only that one edge is checked. Wrong
 * exactly when the optimal in-ball parent is blocked, which inside a δ-clear
 * corridor stops happening as r_n → 0.
 */
export class Fmt<P> implements Anytime<P> {
  readonly name = 'FMT*';
  readonly samples: P[] = [];
  readonly cost: number[] = [];
  readonly parent: number[] = [];
  readonly state: ('unvisited' | 'open' | 'closed')[] = [];
  readonly r: number;
  iterations = 0;
  readonly costHistory: CostSample[] = [];
  private readonly heap = new KeyedHeap<number>();
  private readonly nn;
  private done = false;
  private solution: Solution<P> | null = null;
  private readonly t0: number;
  private readonly now: () => number;
  /** Index of the goal sample (1) and start sample (0). */
  readonly goalIdx = 1;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    readonly start: P,
    readonly goal: P,
    readonly opts: FmtOptions<P>,
    readonly rng: Rng,
  ) {
    this.now = opts.now ?? (() => (typeof performance !== 'undefined' ? performance.now() : Date.now()));
    this.t0 = this.now();
    const sampler = opts.sampler ?? new Uniform<P>();
    this.samples.push(start, goal);
    let guard = 0;
    while (this.samples.length < opts.n + 2 && guard++ < 50 * opts.n + 1000) {
      const q = sampler.sample(space, free, rng);
      if (q !== null) this.samples.push(q);
    }
    this.nn = makeNn(space, opts.nn ?? 'kd');
    this.samples.forEach((q, i) => this.nn.insert(i, q));
    const N = this.samples.length;
    this.r = rN(N, space.dim, opts.gamma);
    for (let i = 0; i < N; i++) {
      this.cost.push(Infinity);
      this.parent.push(-1);
      this.state.push('unvisited');
    }
    this.cost[0] = 0;
    this.state[0] = 'open';
    this.heap.push([0], 0);
  }

  get finished(): boolean {
    return this.done;
  }

  /** Expand one frontier node. */
  step(): number | null {
    if (this.done) return null;
    this.iterations++;
    let z = -1;
    while (this.heap.size > 0) {
      const e = this.heap.pop()!;
      if (this.state[e.value] === 'open' && Math.abs(e.key[0] - this.cost[e.value]) < 1e-12) {
        z = e.value;
        break;
      }
    }
    if (z < 0) {
      this.done = true;
      return null;
    }
    const qz = this.samples[z];
    for (const x of this.nn.within(qz, this.r, z)) {
      if (this.state[x.idx] !== 'unvisited') continue;
      const qx = this.samples[x.idx];
      // Locally optimal parent among open nodes in the ball, obstacles ignored.
      let y = -1;
      let best = Infinity;
      for (const nb of this.nn.within(qx, this.r, x.idx)) {
        if (this.state[nb.idx] !== 'open') continue;
        const c = this.cost[nb.idx] + nb.dist;
        if (c < best) {
          best = c;
          y = nb.idx;
        }
      }
      if (y < 0) continue;
      // Then check only that edge.
      if (this.opts.steer.steer(this.space, this.free, this.samples[y], qx)) {
        this.cost[x.idx] = best;
        this.parent[x.idx] = y;
        this.state[x.idx] = 'open';
        this.heap.push([best], x.idx);
      }
    }
    this.state[z] = 'closed';
    if (z === this.goalIdx) {
      this.done = true;
      const path: P[] = [];
      for (let i = z; i >= 0; i = this.parent[i]) path.push(this.samples[i]);
      path.reverse();
      this.solution = { path, cost: this.cost[z] };
      this.costHistory.push({ n: this.samples.length, t: this.now() - this.t0, cost: this.cost[z] });
      return this.cost[z];
    }
    return null;
  }

  run(): Solution<P> | null {
    while (!this.done) this.step();
    return this.solution;
  }

  best(): Solution<P> | null {
    return this.solution;
  }

  /** Tree edges (parent → child) for drawing. */
  edges(): { a: number; b: number }[] {
    const out: { a: number; b: number }[] = [];
    for (let i = 0; i < this.parent.length; i++) if (this.parent[i] >= 0) out.push({ a: this.parent[i], b: i });
    return out;
  }
}

/* -------------------------------------------------------------------------- */
/* BIT* — Gammell, Srinivasa & Barfoot (2015); Gammell, Barfoot & Srinivasa (2020) */
/* -------------------------------------------------------------------------- */

export interface BitOptions<P> {
  steer: Steer<P>;
  /** Samples per batch. */
  batch: number;
  gamma: number;
  nn?: 'kd' | 'linear';
  now?: () => number;
}

/**
 * Batch Informed Trees. Each batch adds `batch` informed samples to an implicit
 * random geometric graph of radius r_n and searches it like Chapter 6's A*:
 * an edge queue keyed ĝ(v) + ĉ(v, x) + ĥ(x), a vertex queue keyed ĝ(v) + ĥ(v),
 * collision checks only when an edge is popped. After a solution, samples and
 * vertices that cannot improve it are pruned, and the tree is kept across
 * batches, so the next batch repairs rather than rebuilds.
 */
export class Bit<P> implements Anytime<P> {
  readonly name = 'BIT*';
  /** All configurations ever drawn; index 0 start, 1 goal. */
  readonly points: P[] = [];
  /** Tree membership: cost-to-come, parent, children. */
  readonly g: number[] = [];
  readonly parent: number[] = [];
  readonly inTree: boolean[] = [];
  /** Unconnected samples (indices into `points`). */
  private samples = new Set<number>();
  private vertices = new Set<number>();
  private qV = new KeyedHeap<number>();
  private qE = new KeyedHeap<[number, number]>();
  private expanded = new Set<number>();
  private r = Infinity;
  batches = 0;
  iterations = 0;
  edgeChecks = 0;
  bestCost = Infinity;
  readonly costHistory: CostSample[] = [];
  private readonly informed: InformedSampler<P>;
  private readonly nn;
  private readonly t0: number;
  private readonly now: () => number;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    readonly start: P,
    readonly goal: P,
    readonly opts: BitOptions<P>,
    readonly rng: Rng,
  ) {
    this.now = opts.now ?? (() => (typeof performance !== 'undefined' ? performance.now() : Date.now()));
    this.t0 = this.now();
    this.informed = new InformedSampler(space, start, goal);
    this.nn = makeNn(space, opts.nn ?? 'kd');
    this.addPoint(start);
    this.addPoint(goal);
    this.g[0] = 0;
    this.inTree[0] = true;
    this.vertices.add(0);
    this.samples.add(1);
  }

  private addPoint(q: P): number {
    const i = this.points.length;
    this.points.push(q);
    this.g.push(Infinity);
    this.parent.push(-1);
    this.inTree.push(false);
    this.nn.insert(i, q);
    return i;
  }

  private gHat(i: number): number {
    return this.space.dist(this.start, this.points[i]);
  }

  private hHat(i: number): number {
    return this.space.dist(this.points[i], this.goal);
  }

  get solved(): boolean {
    return Number.isFinite(this.bestCost);
  }

  /** Start a new batch: prune, sample, reset the vertex queue. */
  private newBatch(): void {
    this.batches++;
    if (this.solved) {
      // Prune everything that cannot lie on a better path (ĝ + ĥ ≥ c_best).
      for (const s of [...this.samples]) if (this.gHat(s) + this.hHat(s) >= this.bestCost) this.samples.delete(s);
      for (const v of [...this.vertices]) {
        if (v === 0 || v === 1) continue;
        if (this.gHat(v) + this.hHat(v) >= this.bestCost || this.g[v] + this.hHat(v) >= this.bestCost) {
          // Disconnected vertices are returned to the sample set if they could still help.
          this.vertices.delete(v);
          this.inTree[v] = false;
          this.g[v] = Infinity;
          this.parent[v] = -1;
          if (this.gHat(v) + this.hHat(v) < this.bestCost) this.samples.add(v);
        }
      }
      // Children of pruned vertices lose their parent: repair by detaching them too.
      let changed = true;
      while (changed) {
        changed = false;
        for (const v of [...this.vertices]) {
          const p = this.parent[v];
          if (v !== 0 && (p < 0 || !this.inTree[p])) {
            this.vertices.delete(v);
            this.inTree[v] = false;
            this.g[v] = Infinity;
            this.parent[v] = -1;
            if (this.gHat(v) + this.hHat(v) < this.bestCost) this.samples.add(v);
            changed = true;
          }
        }
      }
    }
    this.informed.update(this.bestCost);
    let guard = 0;
    let added = 0;
    while (added < this.opts.batch && guard++ < 50 * this.opts.batch + 1000) {
      const q = this.informed.sample(this.space, this.free, this.rng);
      if (q === null) continue;
      this.samples.add(this.addPoint(q));
      added++;
    }
    const n = this.vertices.size + this.samples.size;
    this.r = rN(n, this.space.dim, this.opts.gamma);
    this.expanded = new Set();
    this.qV = new KeyedHeap();
    this.qE = new KeyedHeap();
    for (const v of this.vertices) this.qV.push([this.g[v] + this.hHat(v)], v);
  }

  private bestVertexKey(): number {
    const t = this.qV.peek();
    return t ? t.key[0] : Infinity;
  }

  private bestEdgeKey(): number {
    const t = this.qE.peek();
    return t ? t.key[0] : Infinity;
  }

  /** Expand a vertex: queue its edges to nearby samples (and, once, to nearby vertices that could be rewired). */
  private expandVertex(v: number): void {
    const qv = this.points[v];
    for (const nb of this.nn.within(qv, this.r, v)) {
      const x = nb.idx;
      const est = this.gHat(v) + nb.dist + this.hHat(x);
      if (est >= this.bestCost) continue;
      if (this.samples.has(x)) this.qE.push([est, nb.dist], [v, x]);
      else if (this.inTree[x] && !this.expanded.has(v) && this.g[v] + nb.dist < this.g[x]) this.qE.push([est, nb.dist], [v, x]);
    }
    this.expanded.add(v);
  }

  /** One unit of work: process the best edge (expanding vertices as needed), or open a new batch. */
  step(): number | null {
    this.iterations++;
    if (this.qV.size === 0 && this.qE.size === 0) {
      this.newBatch();
      return null;
    }
    while (this.bestVertexKey() <= this.bestEdgeKey()) {
      const t = this.qV.pop();
      if (!t) break;
      this.expandVertex(t.value);
    }
    const e = this.qE.pop();
    if (!e) return null;
    const [v, x] = e.value;
    if (!this.inTree[v]) return null;
    const c = this.space.dist(this.points[v], this.points[x]);
    if (this.g[v] + c + this.hHat(x) >= this.bestCost) {
      // Nothing left in the queue can beat the incumbent: end the batch.
      this.qV = new KeyedHeap();
      this.qE = new KeyedHeap();
      return null;
    }
    if (this.g[v] + c >= this.g[x]) return null;
    this.edgeChecks++;
    if (!this.opts.steer.steer(this.space, this.free, this.points[v], this.points[x])) return null;
    if (this.inTree[x]) {
      // Rewire x under v and propagate.
      const delta = this.g[v] + c - this.g[x];
      this.parent[x] = v;
      const stack = [x];
      while (stack.length) {
        const u = stack.pop()!;
        this.g[u] += delta;
        for (const w of this.vertices) if (this.parent[w] === u) stack.push(w);
      }
    } else {
      this.samples.delete(x);
      this.inTree[x] = true;
      this.vertices.add(x);
      this.g[x] = this.g[v] + c;
      this.parent[x] = v;
      this.qV.push([this.g[x] + this.hHat(x)], x);
    }
    if (this.inTree[1] && this.g[1] < this.bestCost - 1e-12) {
      this.bestCost = this.g[1];
      this.costHistory.push({ n: this.points.length - 2, t: this.now() - this.t0, cost: this.bestCost });
      return this.bestCost;
    }
    return null;
  }

  best(): Solution<P> | null {
    if (!this.solved) return null;
    const path: P[] = [];
    for (let i = 1; i >= 0; i = this.parent[i]) path.push(this.points[i]);
    path.reverse();
    return { path, cost: this.bestCost };
  }

  /** Tree edges for drawing. */
  edges(): { a: number; b: number }[] {
    const out: { a: number; b: number }[] = [];
    for (const v of this.vertices) if (this.parent[v] >= 0) out.push({ a: this.parent[v], b: v });
    return out;
  }

  get sampleCount(): number {
    return this.samples.size;
  }

  get vertexCount(): number {
    return this.vertices.size;
  }

  get radius(): number {
    return this.r;
  }

  /** The informed set's measure ratio (w13.3's readout for the BIT* lane). */
  measureRatio(muSpace: number): number {
    return this.informed.measureRatio(muSpace);
  }

  /** Unconnected sample configurations, for drawing. */
  sampleList(): P[] {
    return [...this.samples].map((i) => this.points[i]);
  }
}

/* -------------------------------------------------------------------------- */
/* The §3 micro-example as data                                                */
/* -------------------------------------------------------------------------- */

/** Chapter 13 §3: the five-node tree. A root, B←A, C←B, D←A; new sample E; radius 2.5. */
export const FIVE_NODE = {
  A: [0, 0],
  B: [3, 0],
  C: [3, 3],
  D: [0, 3],
  E: [1.5, 1.5],
  r: 2.5,
} as const;

/**
 * Build the five-node tree and perform the one RRT* insertion of the worked
 * example, returning every intermediate number the prose prints.
 */
export function fiveNodeRewire(space: Chart<number[]>, free: FreeSpace<number[]>, steer: Steer<number[]>) {
  const star = new RrtStar<number[]>(space, free, [...FIVE_NODE.A], [100, 100], { steer, eta: 10, nbhd: { kind: 'radius', gamma: 1 } }, { next: () => 0 } as unknown as Rng);
  const t = star.tree;
  const B = t.add([...FIVE_NODE.B], 0, 3);
  const C = t.add([...FIVE_NODE.C], B, 3);
  const D = t.add([...FIVE_NODE.D], 0, 3);
  const E = [...FIVE_NODE.E];
  const near = t.near(E, FIVE_NODE.r);
  const table = near.map((nb) => ({ node: ['A', 'B', 'C', 'D'][nb.idx], dist: nb.dist, viaCost: t.cost[nb.idx] + nb.dist }));
  const { added, rewires } = star.insert(E, near, null);
  return { near, table, added, costE: t.cost[added], rewires, costC: t.cost[C], parentC: t.parent(C), costB: t.cost[B], costD: t.cost[D], ids: { B, C, D } };
}
