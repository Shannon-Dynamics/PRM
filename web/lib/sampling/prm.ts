/**
 * BASIC PRM — Choset Algorithms 6 and 7, generic over a Chapter 5 `Chart`.
 *
 * Two phases. Learning: sample n free configurations (coarse rain), then for
 * each node ask the kd-tree for its k closest neighbors and call Δ on each
 * (fine rain); an edge is added when Δ succeeds. Query: connect q_start and
 * q_goal to the roadmap by the same Δ, then run Chapter 6's Dijkstra. The
 * roadmap is an undirected graph plus a union–find over its nodes, so the
 * number of connected components — the quantity w11.1 watches collapse — is
 * O(1) to read at every step.
 *
 * Two build orders are offered because they are both in Choset. `build(n)` is
 * Algorithm 6 literally: all nodes first, then every node's k nearest among
 * *all* nodes — the semantics the §3 micro-example is traced under. `grow(n)`
 * is the incremental form of §7.1.1 "Adding to the Roadmap" and §7.2's
 * single-query PRM: each new node connects to its k nearest among the nodes
 * already present, so the roadmap can be scrubbed by time (every edge's
 * insertion index is the larger of its endpoints).
 *
 * Connection strategies (§7.1.4): k-nearest, a radius ball (Chapter 13's PRM*
 * uses it), sparse (one edge per neighboring component plus a few redundant
 * ones — a forest), and lazy (edges assumed free, checked coarse-to-fine only
 * when they lie on a candidate path; Bohlin & Kavraki [C ref. 53]).
 */

import type { Rng } from '../prob/rng';
import type { Chart } from '../manifold/manifold';
import type { Edge, Graph } from '../search/graph';
import { dijkstra } from '../search/best-first';
import type { FreeSpace } from './free-space';
import { makeNn, type Neighbor, type NearestNeighbors } from './kdtree';
import type { Sampler } from './sampler';
import type { Steer } from './steer';

/* -------------------------------------------------------------------------- */
/* Union–find                                                                  */
/* -------------------------------------------------------------------------- */

export class UnionFind {
  private readonly parent: number[] = [];
  private readonly rank: number[] = [];
  /** Number of disjoint sets. */
  count = 0;

  make(): number {
    const i = this.parent.length;
    this.parent.push(i);
    this.rank.push(0);
    this.count++;
    return i;
  }

  find(i: number): number {
    while (this.parent[i] !== i) {
      this.parent[i] = this.parent[this.parent[i]];
      i = this.parent[i];
    }
    return i;
  }

  /** True if the two were in different sets (and are now joined). */
  union(a: number, b: number): boolean {
    let ra = this.find(a);
    let rb = this.find(b);
    if (ra === rb) return false;
    if (this.rank[ra] < this.rank[rb]) [ra, rb] = [rb, ra];
    this.parent[rb] = ra;
    if (this.rank[ra] === this.rank[rb]) this.rank[ra]++;
    this.count--;
    return true;
  }

  same(a: number, b: number): boolean {
    return this.find(a) === this.find(b);
  }

  get size(): number {
    return this.parent.length;
  }
}

/* -------------------------------------------------------------------------- */
/* The roadmap                                                                 */
/* -------------------------------------------------------------------------- */

export interface RoadmapEdge extends Edge<number> {
  to: number;
  cost: number;
  /** Lazy evaluation: false until Δ has verified the edge at full resolution. */
  checked: boolean;
}

const edgeKey = (a: number, b: number) => (a < b ? `${a}-${b}` : `${b}-${a}`);

export class Roadmap<P> implements Graph<number> {
  readonly nodes: P[] = [];
  readonly adj: RoadmapEdge[][] = [];
  readonly uf = new UnionFind();
  private readonly edgeSet = new Set<string>();
  /** Nodes with index ≥ this are invisible to `neighbors` — the time scrubber. */
  limit = Infinity;

  constructor(readonly space: Chart<P>) {}

  addNode(q: P): number {
    this.nodes.push(q);
    this.adj.push([]);
    return this.uf.make();
  }

  hasEdge(a: number, b: number): boolean {
    return this.edgeSet.has(edgeKey(a, b));
  }

  /** Add an undirected edge; false if it already existed. */
  addEdge(a: number, b: number, cost: number, checked = true): boolean {
    if (a === b) return false;
    const k = edgeKey(a, b);
    if (this.edgeSet.has(k)) return false;
    this.edgeSet.add(k);
    this.adj[a].push({ to: b, cost, checked });
    this.adj[b].push({ to: a, cost, checked });
    this.uf.union(a, b);
    return true;
  }

  /**
   * Remove an edge (lazy PRM deleting a blocked one). Union–find cannot
   * un-union, so components are recomputed from the adjacency afterwards —
   * the honest O(V + E) price of laziness.
   */
  removeEdge(a: number, b: number): void {
    const k = edgeKey(a, b);
    if (!this.edgeSet.delete(k)) return;
    this.adj[a] = this.adj[a].filter((e) => e.to !== b);
    this.adj[b] = this.adj[b].filter((e) => e.to !== a);
    this.rebuildComponents();
  }

  markChecked(a: number, b: number): void {
    for (const e of this.adj[a]) if (e.to === b) e.checked = true;
    for (const e of this.adj[b]) if (e.to === a) e.checked = true;
  }

  edge(a: number, b: number): RoadmapEdge | undefined {
    return this.adj[a].find((e) => e.to === b);
  }

  private rebuildComponents(): void {
    const uf = new UnionFind();
    for (let i = 0; i < this.nodes.length; i++) uf.make();
    for (let a = 0; a < this.adj.length; a++) for (const e of this.adj[a]) if (a < e.to) uf.union(a, e.to);
    // Replace in place so references stay valid.
    (this as { uf: UnionFind }).uf = uf;
  }

  get edgeCount(): number {
    return this.edgeSet.size;
  }

  /** All edges as index pairs with a < b, in a deterministic order. */
  edges(): { a: number; b: number; cost: number; checked: boolean }[] {
    const out: { a: number; b: number; cost: number; checked: boolean }[] = [];
    for (let a = 0; a < this.adj.length; a++) for (const e of this.adj[a]) if (a < e.to) out.push({ a, b: e.to, cost: e.cost, checked: e.checked });
    return out;
  }

  components(): number {
    return this.uf.count;
  }

  sameComponent(a: number, b: number): boolean {
    return this.uf.same(a, b);
  }

  componentOf(i: number): number {
    return this.uf.find(i);
  }

  /** Component counts among the first `n` nodes only (for the time scrubber). */
  componentsUpTo(n: number): number {
    const uf = new UnionFind();
    for (let i = 0; i < n; i++) uf.make();
    for (let a = 0; a < n; a++) for (const e of this.adj[a]) if (e.to < n && a < e.to) uf.union(a, e.to);
    return uf.count;
  }

  // -- Graph<number> for Chapter 6's search ---------------------------------

  neighbors(n: number): readonly Edge<number>[] {
    if (this.limit === Infinity) return this.adj[n];
    return this.adj[n].filter((e) => e.to < this.limit);
  }

  key(n: number): number {
    return n;
  }

  /** Dijkstra between two roadmap nodes. Returns node indices, or null. */
  shortestPath(a: number, b: number, limit = Infinity): { path: number[]; cost: number } | null {
    const prev = this.limit;
    this.limit = limit;
    const t = dijkstra<number>(this, a, b);
    this.limit = prev;
    return t.status === 'found' ? { path: t.path, cost: t.cost } : null;
  }

  pathPoints(idx: readonly number[]): P[] {
    return idx.map((i) => this.nodes[i]);
  }
}

/* -------------------------------------------------------------------------- */
/* Connection strategies                                                       */
/* -------------------------------------------------------------------------- */

export type Connect =
  | { kind: 'kNearest'; k: number }
  | { kind: 'radius'; r: number }
  | { kind: 'sparse'; k: number; redundant: number }
  | { kind: 'lazy'; k: number };

export interface PrmOptions<P> {
  sampler: Sampler<P>;
  steer: Steer<P>;
  connect: Connect;
  nn?: 'kd' | 'linear';
  /** Skip candidate edges longer than this (saves Δ calls; the roadmap stays local). */
  maxEdgeLength?: number;
}

export interface QueryResult<P> {
  path: P[];
  cost: number;
  /** Roadmap node indices the endpoints attached to. */
  attach: [number, number];
  /** Collision checks spent by the query itself. */
  checks: number;
}

/**
 * The planner. Resumable: `grow` and `build` extend the roadmap they find.
 * `rng` is seeded by the caller; the widget shows the seed.
 */
export class Prm<P> {
  readonly roadmap: Roadmap<P>;
  readonly nn: NearestNeighbors<P>;
  /** components() after each node was added — the sparkline of w11.1. */
  readonly componentHistory: number[] = [];
  /** Failed sampler attempts so far (rejections are the biased samplers' cost). */
  rejected = 0;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    readonly opts: PrmOptions<P>,
    readonly rng: Rng,
  ) {
    this.roadmap = new Roadmap(space);
    this.nn = makeNn(space, opts.nn ?? 'kd');
  }

  get n(): number {
    return this.roadmap.nodes.length;
  }

  /** Insert a known-free configuration (q_start, q_goal, or a scripted sample) as a node. */
  addNode(q: P): number {
    const i = this.roadmap.addNode(q);
    this.nn.insert(i, q);
    return i;
  }

  /** One sampler attempt; a node index if it produced a free configuration. */
  sampleNode(): number | null {
    const q = this.opts.sampler.sample(this.space, this.free, this.rng);
    if (q === null) {
      this.rejected++;
      return null;
    }
    return this.addNode(q);
  }

  /** The candidate set N_q for node i under the connection rule. */
  candidates(i: number): Neighbor[] {
    const q = this.roadmap.nodes[i];
    const c = this.opts.connect;
    let cands: Neighbor[];
    switch (c.kind) {
      case 'kNearest':
      case 'lazy':
        cands = this.nn.kNearest(q, c.k, i);
        break;
      case 'radius':
        cands = this.nn.within(q, c.r, i);
        break;
      case 'sparse': {
        // Nearest node of each other component, then `redundant` extra closest ones.
        const pool = this.nn.kNearest(q, Math.max(c.k * 4, 16), i);
        const seen = new Set<number>();
        const first: Neighbor[] = [];
        const extra: Neighbor[] = [];
        for (const nb of pool) {
          const comp = this.roadmap.componentOf(nb.idx);
          if (this.roadmap.sameComponent(i, nb.idx)) {
            if (extra.length < c.redundant) extra.push(nb);
          } else if (!seen.has(comp)) {
            seen.add(comp);
            first.push(nb);
          }
        }
        cands = [...first.slice(0, c.k), ...extra];
        break;
      }
    }
    const maxLen = this.opts.maxEdgeLength;
    return maxLen === undefined ? cands : cands.filter((nb) => nb.dist <= maxLen);
  }

  /** Try the local planner on each candidate; add the edges that succeed (Alg. 6 lines 11–15). */
  connectNode(i: number, cands: readonly Neighbor[]): number {
    const q = this.roadmap.nodes[i];
    let added = 0;
    for (const nb of cands) {
      if (this.roadmap.hasEdge(i, nb.idx)) continue;
      if (this.opts.connect.kind === 'sparse' && this.roadmap.sameComponent(i, nb.idx)) {
        // Redundant edges are allowed only up to the quota computed in `candidates`.
      }
      if (this.opts.connect.kind === 'lazy') {
        this.roadmap.addEdge(i, nb.idx, nb.dist, false);
        added++;
        continue;
      }
      const lp = this.opts.steer.steer(this.space, this.free, q, this.roadmap.nodes[nb.idx]);
      if (lp) {
        this.roadmap.addEdge(i, nb.idx, lp.length, true);
        added++;
      }
    }
    return added;
  }

  /** Incremental step: sample one node and connect it to its neighbors so far. */
  addSample(): number | null {
    const i = this.sampleNode();
    if (i === null) return null;
    this.connectNode(i, this.candidates(i));
    this.componentHistory.push(this.roadmap.components());
    return i;
  }

  /** Grow incrementally until the roadmap holds n nodes (or the sampler gives up). */
  grow(n: number, maxAttempts = 50 * n + 1000): void {
    let attempts = 0;
    while (this.n < n && attempts++ < maxAttempts) this.addSample();
  }

  /**
   * Choset Algorithm 6, literally: fill V to n nodes, then connect every node to
   * its k closest neighbors chosen from all of V.
   */
  build(n: number, maxAttempts = 50 * n + 1000): void {
    const start = this.n;
    let attempts = 0;
    while (this.n < n && attempts++ < maxAttempts) this.sampleNode();
    for (let i = 0; i < this.n; i++) {
      // Nodes present before this call already have their edges; only new pairs matter,
      // but every node's neighbor list may now include new nodes.
      if (i < start && this.opts.connect.kind === 'kNearest') {
        const cands = this.candidates(i).filter((nb) => nb.idx >= start);
        this.connectNode(i, cands);
      } else {
        this.connectNode(i, this.candidates(i));
      }
    }
    this.componentHistory.push(this.roadmap.components());
  }

  /**
   * Choset Algorithm 7. Connect q_start to its k closest nodes in order until Δ
   * succeeds, likewise q_goal (preferring the start's component), then
   * Dijkstra. Does not mutate the roadmap.
   */
  query(qStart: P, qGoal: P, k: number): QueryResult<P> | null {
    const checks0 = this.free.checks;
    const ns = this.nn.kNearest(qStart, k);
    let a = -1;
    let costA = 0;
    for (const nb of ns) {
      if (this.opts.steer.steer(this.space, this.free, qStart, this.roadmap.nodes[nb.idx])) {
        a = nb.idx;
        costA = nb.dist;
        break;
      }
    }
    if (a < 0) return null;
    const ng = this.nn.kNearest(qGoal, k);
    const ordered = [...ng.filter((nb) => this.roadmap.sameComponent(a, nb.idx)), ...ng.filter((nb) => !this.roadmap.sameComponent(a, nb.idx))];
    let b = -1;
    let costB = 0;
    for (const nb of ordered) {
      if (this.opts.steer.steer(this.space, this.free, this.roadmap.nodes[nb.idx], qGoal)) {
        b = nb.idx;
        costB = nb.dist;
        break;
      }
    }
    if (b < 0) return null;
    const sp = this.opts.connect.kind === 'lazy' ? this.lazySearch(a, b) : this.roadmap.shortestPath(a, b);
    if (!sp) return null;
    return {
      path: [qStart, ...this.roadmap.pathPoints(sp.path), qGoal],
      cost: costA + sp.cost + costB,
      attach: [a, b],
      checks: this.free.checks - checks0,
    };
  }

  /**
   * Lazy PRM's query loop (§7.1.4 "Lazy Evaluation"): search, then check the
   * path's unchecked edges coarse-to-fine; delete the first blocked edge and
   * search again. Nodes were checked when sampled.
   */
  lazySearch(a: number, b: number, maxRounds = 10000): { path: number[]; cost: number } | null {
    for (let round = 0; round < maxRounds; round++) {
      const sp = this.roadmap.shortestPath(a, b);
      if (!sp) return null;
      let blocked: [number, number] | null = null;
      // Coarse-to-fine: level ℓ checks the odd multiples of 2^{-ℓ} on every unchecked edge.
      const step = stepOf(this.opts.steer);
      const maxLevel = Math.max(1, Math.ceil(Math.log2(Math.max(1, longest(this.roadmap, sp.path) / step))));
      outer: for (let level = 1; level <= maxLevel; level++) {
        for (let i = 1; i < sp.path.length; i++) {
          const u = sp.path[i - 1];
          const v = sp.path[i];
          const e = this.roadmap.edge(u, v)!;
          if (e.checked) continue;
          const qu = this.roadmap.nodes[u];
          const qv = this.roadmap.nodes[v];
          const m = 1 << level;
          if (e.cost / m < step / 2 && level > 1) continue;
          for (let j = 1; j < m; j += 2) {
            if (!this.free.isFree(this.space.interpolate(qu, qv, j / m))) {
              blocked = [u, v];
              break outer;
            }
          }
        }
      }
      if (blocked) {
        this.roadmap.removeEdge(blocked[0], blocked[1]);
        continue;
      }
      for (let i = 1; i < sp.path.length; i++) this.roadmap.markChecked(sp.path[i - 1], sp.path[i]);
      return sp;
    }
    return null;
  }

  /**
   * Connection sampling (§7.1.3): pick a node with probability ∝ 1/(deg + 1),
   * expand it with a nearby free sample, connect as usual. Never creates a new
   * component by itself — at worst it fails to reduce the count.
   */
  connectionSample(radius: number): number | null {
    const weights = this.roadmap.adj.map((a) => 1 / (a.length + 1));
    const i = this.rng.choice(weights);
    const q = this.roadmap.nodes[i];
    for (let t = 0; t < 20; t++) {
      const c = this.space.coords(q).map((x) => x + this.rng.uniform(-radius, radius));
      const qn = this.space.fromCoords(c);
      if (!this.free.isFree(qn)) continue;
      const lp = this.opts.steer.steer(this.space, this.free, q, qn);
      if (!lp) continue;
      const j = this.addNode(qn);
      this.roadmap.addEdge(i, j, lp.length);
      this.connectNode(j, this.candidates(j));
      this.componentHistory.push(this.roadmap.components());
      return j;
    }
    return null;
  }
}

const stepOf = <P>(s: Steer<P>): number => {
  const c = (s as { check?: { step: number } }).check;
  return c ? c.step : 0.1;
};

const longest = <P>(r: Roadmap<P>, path: readonly number[]): number => {
  let L = 0;
  for (let i = 1; i < path.length; i++) L = Math.max(L, r.edge(path[i - 1], path[i])?.cost ?? 0);
  return L;
};
