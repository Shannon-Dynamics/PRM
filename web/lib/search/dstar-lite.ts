/**
 * D* Lite — Koenig & Likhachev (AAAI 2002; IEEE T-RO 2005), the chapter's
 * default replanner.
 *
 * Same contract as D*: search backward from the goal, repair locally when an
 * edge cost changes, move the robot along the greedy descent of the resulting
 * cost-to-goal. D* Lite reaches it from a different direction — it is Lifelong
 * Planning A* run backward — and is about a page of code:
 *
 *   g(s)    the current cost-to-goal estimate of s
 *   rhs(s)  the one-step lookahead min_{s'} [c(s, s') + g(s')]  (rhs(goal) = 0)
 *   U       a priority queue of *inconsistent* vertices (g ≠ rhs), keyed on
 *           [min(g, rhs) + h(start, s) + k_m ; min(g, rhs)]
 *
 * ComputeShortestPath pops inconsistent vertices in key order and makes them
 * consistent; an overconsistent vertex (g > rhs) is simply set to rhs, an
 * underconsistent one (g < rhs) is reset to ∞ so its neighbours re-derive it.
 * The key modifier k_m is the trick that makes the robot's own motion free:
 * rather than re-key the whole heap when the start moves (h is measured from
 * the start), add the heuristic distance travelled to every *future* key,
 * which keeps old keys lower bounds on the new ones.
 *
 * Obstacles here are modern: an arc into an occupied cell costs ∞, not 10000.
 * The check that D* Lite's cost from the current start equals a fresh A*'s on
 * the same (changed) grid is the chapter's correctness pin for this file.
 */

import { compareKeys, compareKeysEps, KeyedHeap } from './heap';
import { GridGraph, cloneGrid, gridHeuristic, type GridGraphOptions, type OccupancyGrid } from './grid';

export interface DStarLiteStats {
  /** Vertices popped by ComputeShortestPath in the last call. */
  pops: number;
  /** Distinct vertices whose g or rhs was touched in the last call. */
  touched: Set<number>;
}

export class DStarLite {
  readonly believed: OccupancyGrid;
  readonly graph: GridGraph;
  readonly g: Float64Array;
  readonly rhs: Float64Array;
  readonly goal: number;
  start: number;
  private last: number;
  private km = 0;
  private readonly heap = new KeyedHeap<number>();
  /** The key each vertex was last inserted with, or null if not in U. */
  private readonly inU: (readonly number[] | null)[];
  private readonly diagonal: number;
  stats: DStarLiteStats = { pops: 0, touched: new Set() };

  constructor(grid: OccupancyGrid, start: number, goal: number, opts: GridGraphOptions = {}) {
    this.believed = cloneGrid(grid);
    this.graph = new GridGraph(this.believed, opts);
    this.diagonal = this.graph.diagonal;
    const n = grid.nx * grid.ny;
    this.g = new Float64Array(n).fill(Infinity);
    this.rhs = new Float64Array(n).fill(Infinity);
    this.inU = new Array<readonly number[] | null>(n).fill(null);
    this.goal = goal;
    this.start = start;
    this.last = start;
    this.rhs[goal] = 0;
    this.insert(goal, this.calculateKey(goal));
    this.computeShortestPath();
  }

  /** The heuristic between two cells — octile with the grid's diagonal, consistent as required. */
  private h(a: number, b: number): number {
    return gridHeuristic(this.believed, b, 'octile', this.diagonal)(a);
  }

  /** c(u, v): ∞ into or out of an occupied cell. */
  cost(u: number, v: number): number {
    const occ = this.believed.occupied;
    if (occ[u] || occ[v]) return Infinity;
    if (!this.graph.cornerCutting) {
      const nx = this.believed.nx;
      const ui = u % nx, uj = (u - ui) / nx, vi = v % nx, vj = (v - vi) / nx;
      if (ui !== vi && uj !== vj && (occ[uj * nx + vi] || occ[vj * nx + ui])) return Infinity;
    }
    return this.graph.stepCost(u, v);
  }

  private calculateKey(s: number): [number, number] {
    const m = Math.min(this.g[s], this.rhs[s]);
    return [m + this.h(this.start, s) + this.km, m];
  }

  private insert(s: number, key: readonly number[]): void {
    this.inU[s] = key;
    this.heap.push(key, s);
  }

  private remove(s: number): void {
    this.inU[s] = null; // the heap entry becomes stale and is skipped on pop
  }

  /** Discard stale heap entries; return the live top or undefined. */
  private top(): { key: readonly number[]; s: number } | undefined {
    for (;;) {
      const e = this.heap.peek();
      if (!e) return undefined;
      const live = this.inU[e.value];
      if (!live || compareKeys(live, e.key) !== 0) {
        this.heap.pop();
        continue;
      }
      return { key: e.key, s: e.value };
    }
  }

  private updateVertex(u: number): void {
    if (u !== this.goal) {
      let best = Infinity;
      for (const v of this.graph.latticeNeighbors(u)) {
        const c = this.cost(u, v);
        if (c === Infinity) continue;
        best = Math.min(best, c + this.g[v]);
      }
      this.rhs[u] = best;
    }
    if (this.inU[u]) this.remove(u);
    if (this.g[u] !== this.rhs[u]) this.insert(u, this.calculateKey(u));
    this.stats.touched.add(u);
  }

  /** Koenig & Likhachev's ComputeShortestPath. Returns the number of pops. */
  computeShortestPath(): number {
    this.stats = { pops: 0, touched: new Set() };
    for (;;) {
      const t = this.top();
      const startKey = this.calculateKey(this.start);
      const startInconsistent = this.rhs[this.start] !== this.g[this.start];
      // Koenig & Likhachev stop at TopKey() < Key(start). We also process vertices
      // whose key *ties* the start's: a stale vertex left at an equal key can still
      // win a tie in the greedy descent and send the robot down a path whose real
      // cost is higher than g said. Processing ties makes the whole f ≤ g(start)
      // band consistent, at the price of a few extra pops.
      if (!t || (compareKeysEps(t.key, startKey) > 0 && !startInconsistent)) break;
      const u = t.s;
      const kold = t.key;
      this.heap.pop();
      this.remove(u);
      this.stats.pops++;
      this.stats.touched.add(u);
      const knew = this.calculateKey(u);
      if (compareKeys(kold, knew) < 0) {
        // The key went stale because k_m grew: re-insert, do not expand.
        this.insert(u, knew);
      } else if (this.g[u] > this.rhs[u]) {
        // Overconsistent: the good news case. Commit and tell the predecessors.
        this.g[u] = this.rhs[u];
        for (const s of this.graph.latticeNeighbors(u)) this.updateVertex(s);
      } else {
        // Underconsistent: a path through u got worse. Forget g(u) and let the
        // neighbourhood (including u itself) re-derive it.
        this.g[u] = Infinity;
        for (const s of this.graph.latticeNeighbors(u)) this.updateVertex(s);
        this.updateVertex(u);
      }
    }
    return this.stats.pops;
  }

  /** Cost-to-goal from the current start, ∞ when none. */
  costFromStart(): number {
    return this.g[this.start];
  }

  /** The successor the robot takes: argmin c(start, s′) + g(s′). Null when no path. */
  bestSuccessor(from: number = this.start): number | null {
    let best = -1;
    let bestVal = Infinity;
    for (const v of this.graph.latticeNeighbors(from)) {
      const val = this.cost(from, v) + this.g[v];
      if (val < bestVal - 1e-12 || (Math.abs(val - bestVal) <= 1e-12 && best >= 0 && this.graph.order(v) < this.graph.order(best))) {
        bestVal = val;
        best = v;
      }
    }
    return Number.isFinite(bestVal) ? best : null;
  }

  /** Move the robot one step along the greedy descent. Returns the new start, or null if stuck. */
  nextStep(): number | null {
    if (!Number.isFinite(this.g[this.start])) return null;
    if (this.start === this.goal) return this.goal;
    const s = this.bestSuccessor();
    if (s === null) return null;
    this.start = s;
    return s;
  }

  /** The path the robot would follow from the current start, by greedy descent over g. */
  pathFromStart(): number[] | null {
    if (!Number.isFinite(this.g[this.start])) return null;
    const out = [this.start];
    let cur = this.start;
    const guard = this.believed.nx * this.believed.ny;
    for (let i = 0; i < guard && cur !== this.goal; i++) {
      const s = this.bestSuccessor(cur);
      if (s === null) return null;
      out.push(s);
      cur = s;
    }
    return cur === this.goal ? out : null;
  }

  /**
   * Report observed occupancy changes (the robot's sensor disagreeing with the
   * map), then repair. Every arc incident to a changed cell has a new cost, so
   * both its endpoints get UpdateVertex. Returns the number of vertices touched.
   */
  updateCells(changes: readonly { cell: number; occupied: boolean }[]): number {
    let any = false;
    for (const ch of changes) {
      if ((this.believed.occupied[ch.cell] === 1) === ch.occupied) continue;
      any = true;
    }
    if (!any) {
      this.stats = { pops: 0, touched: new Set() };
      return 0;
    }
    // The start has moved since the last repair: fold that into k_m once.
    this.km += this.h(this.last, this.start);
    this.last = this.start;
    const toUpdate = new Set<number>();
    for (const ch of changes) {
      if ((this.believed.occupied[ch.cell] === 1) === ch.occupied) continue;
      this.believed.occupied[ch.cell] = ch.occupied ? 1 : 0;
      toUpdate.add(ch.cell);
      for (const v of this.graph.latticeNeighbors(ch.cell)) toUpdate.add(v);
    }
    const stats: DStarLiteStats = { pops: 0, touched: new Set() };
    this.stats = stats;
    for (const u of toUpdate) this.updateVertex(u);
    const touchedByUpdates = new Set(this.stats.touched);
    this.computeShortestPath();
    for (const u of touchedByUpdates) this.stats.touched.add(u);
    return this.stats.touched.size;
  }
}
