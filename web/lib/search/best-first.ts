/**
 * Best-first search: one algorithm, one knob (Choset Alg. 24 and §H.1–H.2.3).
 *
 * The priority a node is popped with decides which search this is:
 *
 *   link length (depth)   → breadth-first search
 *   g, the cost-to-come   → Dijkstra
 *   h alone               → greedy search
 *   g + ε·h               → A* (ε = 1), weighted A* (ε > 1), Dijkstra (ε = 0)
 *
 * The open set is a lazy binary heap keyed on [f, h, order]: ties in f break
 * toward smaller h, then toward the graph's own order (lexicographic on a
 * grid), then insertion — the design doc's rule, so the hand trace reproduces.
 * The closed set is a hash set. Alg. 24 as printed updates an improved open
 * node's back pointer but not, explicitly, its g and priority; this does both
 * (honesty item ii). It exits when the goal is *popped*, which is also what
 * the §H.2 walk-through does once you notice that a node is popped only when
 * nothing in O beats it (Derivation 3's reconciliation).
 *
 * The engine is a state machine rather than a function so the Search Theater
 * can advance four of them in lockstep, a few expansions per frame, and paint
 * the open and closed sets as they grow. `bestFirst()` wraps it for callers
 * who just want the answer.
 */

import type { Graph } from './graph';
import { KeyedHeap } from './heap';

export type Priority =
  | { kind: 'linkLength' }
  | { kind: 'costToCome' }
  | { kind: 'heuristicOnly' }
  | { kind: 'sum'; eps: number };

export const PRIORITY_BFS: Priority = { kind: 'linkLength' };
export const PRIORITY_DIJKSTRA: Priority = { kind: 'costToCome' };
export const PRIORITY_GREEDY: Priority = { kind: 'heuristicOnly' };
export const PRIORITY_ASTAR: Priority = { kind: 'sum', eps: 1 };
export const weightedAStar = (eps: number): Priority => ({ kind: 'sum', eps });

export type SearchStatus = 'running' | 'found' | 'failed';

/** One row of the trace table: the node popped, with its g, h, f at that moment. */
export interface Pop<N> {
  node: N;
  g: number;
  h: number;
  f: number;
  /** Link length from the start along the back-pointer path. */
  depth: number;
}

export interface Trace<N> {
  status: SearchStatus;
  pops: Pop<N>[];
  path: N[];
  cost: number;
  /** Number of nodes expanded (= pops.length). */
  expansions: number;
  /** Closed nodes that had to be reopened — zero under a consistent heuristic (Derivation 4). */
  reopened: number;
}

interface OpenEntry<N> {
  node: N;
  g: number;
}

export class BestFirst<N> {
  readonly g = new Map<number | string, number>();
  readonly parent = new Map<number | string, N>();
  readonly depth = new Map<number | string, number>();
  readonly closed = new Set<number | string>();
  /** Nodes currently in the open set (the frontier), for drawing. */
  readonly open = new Map<number | string, N>();
  readonly pops: Pop<N>[] = [];
  status: SearchStatus = 'running';
  reopened = 0;

  private readonly heap = new KeyedHeap<OpenEntry<N>>();
  private seq = 0;

  constructor(
    readonly graph: Graph<N>,
    readonly start: N,
    readonly goal: N,
    readonly h: (n: N) => number,
    readonly prio: Priority = PRIORITY_ASTAR,
  ) {
    const k = graph.key(start);
    this.g.set(k, 0);
    this.depth.set(k, 0);
    this.push(start, 0, 0);
  }

  /** f as the knob defines it. */
  priority(g: number, h: number, depth: number): number {
    switch (this.prio.kind) {
      case 'linkLength':
        return depth;
      case 'costToCome':
        return g;
      case 'heuristicOnly':
        return h;
      case 'sum':
        return g + this.prio.eps * h;
    }
  }

  private push(n: N, g: number, depth: number): void {
    const h = this.h(n);
    const f = this.priority(g, h, depth);
    // BFS must be FIFO within a level; everything else breaks ties toward
    // smaller h and then the graph's order.
    const tie = this.prio.kind === 'linkLength' ? this.seq++ : (this.graph.order?.(n) ?? this.seq++);
    this.heap.push([f, this.prio.kind === 'linkLength' ? 0 : h, tie], { node: n, g });
    this.open.set(this.graph.key(n), n);
  }

  /** Expand one node. Returns the new status. */
  step(): SearchStatus {
    if (this.status !== 'running') return this.status;
    for (;;) {
      const top = this.heap.pop();
      if (!top) {
        this.status = 'failed';
        return this.status;
      }
      const { node, g } = top.value;
      const k = this.graph.key(node);
      // Stale entry: a better g for this node was pushed later, or it is closed with a g ≤ this one.
      if (this.closed.has(k)) continue;
      if (g > (this.g.get(k) ?? Infinity) + 1e-12) continue;

      this.closed.add(k);
      this.open.delete(k);
      const h = this.h(node);
      const depth = this.depth.get(k) ?? 0;
      this.pops.push({ node, g, h, f: this.priority(g, h, depth), depth });

      if (k === this.graph.key(this.goal)) {
        this.status = 'found';
        return this.status;
      }

      for (const { to, cost } of this.graph.neighbors(node)) {
        const tk = this.graph.key(to);
        const tentative = g + cost;
        const old = this.g.get(tk);
        // What counts as "better" is the knob's own measure: breadth-first
        // relaxes on link length (first discovery wins, exactly as a FIFO queue
        // would), every other rule on weighted cost. On a 1/1.4 grid the two
        // disagree — four axial steps (4.0) are cheaper than three diagonals (4.2).
        const better =
          this.prio.kind === 'linkLength'
            ? depth + 1 < (this.depth.get(tk) ?? Infinity)
            : old === undefined || tentative < old - 1e-12;
        if (!better) continue;
        if (this.closed.has(tk)) {
          // Only an inconsistent heuristic (or ε > 1) can get here: a closed node
          // turned out to have a cheaper path. Reopen it — correctness over speed.
          this.closed.delete(tk);
          this.reopened++;
        }
        this.g.set(tk, tentative);
        this.parent.set(tk, node);
        this.depth.set(tk, depth + 1);
        this.push(to, tentative, depth + 1);
      }
      return this.status;
    }
  }

  run(limit = 1e7): SearchStatus {
    let n = 0;
    while (this.status === 'running' && n++ < limit) this.step();
    return this.status;
  }

  /** The back-pointer path from start to goal, empty unless found. */
  path(): N[] {
    if (this.status !== 'found') return [];
    const out: N[] = [];
    let cur: N | undefined = this.goal;
    const guard = this.g.size + 1;
    for (let i = 0; cur !== undefined && i < guard; i++) {
      out.push(cur);
      if (this.graph.key(cur) === this.graph.key(this.start)) break;
      cur = this.parent.get(this.graph.key(cur));
    }
    return out.reverse();
  }

  cost(): number {
    return this.status === 'found' ? (this.g.get(this.graph.key(this.goal)) ?? Infinity) : Infinity;
  }

  trace(): Trace<N> {
    return {
      status: this.status,
      pops: this.pops,
      path: this.path(),
      cost: this.cost(),
      expansions: this.pops.length,
      reopened: this.reopened,
    };
  }

  isOpen(n: N): boolean {
    return this.open.has(this.graph.key(n));
  }

  isClosed(n: N): boolean {
    return this.closed.has(this.graph.key(n));
  }
}

/** Run a search to completion. */
export function bestFirst<N>(graph: Graph<N>, start: N, goal: N, h: (n: N) => number, prio: Priority): Trace<N> {
  const s = new BestFirst(graph, start, goal, h, prio);
  s.run();
  return s.trace();
}

export const ZERO_HEURISTIC = () => 0;

/** Choset Alg. 24. */
export const astar = <N>(graph: Graph<N>, start: N, goal: N, h: (n: N) => number, eps = 1) =>
  bestFirst(graph, start, goal, h, eps === 1 ? PRIORITY_ASTAR : weightedAStar(eps));

/** Dijkstra: f = g (§H.2.3). */
export const dijkstra = <N>(graph: Graph<N>, start: N, goal: N) =>
  bestFirst(graph, start, goal, ZERO_HEURISTIC, PRIORITY_DIJKSTRA);

/** Greedy: f = h (§H.1, §H.2.3). Finds *a* path, with no guarantee about its cost. */
export const greedy = <N>(graph: Graph<N>, start: N, goal: N, h: (n: N) => number) =>
  bestFirst(graph, start, goal, h, PRIORITY_GREEDY);

/* -------------------------------------------------------------------------- */
/* BFS and DFS with the literal queue and stack of figure H.5                  */
/* -------------------------------------------------------------------------- */

/**
 * Breadth-first search with a true FIFO queue — not a heap with constant keys
 * (Exercise 5). A node's depth at first dequeue is its shortest link length
 * from the start (Derivation 1), so the back-pointer path is link-optimal.
 * The reported `cost` is the weighted cost of that path, which need not be
 * minimal unless all weights are equal.
 */
export function bfs<N>(graph: Graph<N>, start: N, goal: N): Trace<N> {
  const parent = new Map<number | string, N>();
  const depth = new Map<number | string, number>();
  const gcost = new Map<number | string, number>();
  const visited = new Set<number | string>();
  const queue: N[] = [start];
  let head = 0;
  visited.add(graph.key(start));
  depth.set(graph.key(start), 0);
  gcost.set(graph.key(start), 0);
  const pops: Pop<N>[] = [];
  const goalKey = graph.key(goal);
  let found = false;
  while (head < queue.length) {
    const n = queue[head++];
    const k = graph.key(n);
    pops.push({ node: n, g: gcost.get(k)!, h: 0, f: depth.get(k)!, depth: depth.get(k)! });
    if (k === goalKey) {
      found = true;
      break;
    }
    for (const { to, cost } of graph.neighbors(n)) {
      const tk = graph.key(to);
      if (visited.has(tk)) continue;
      visited.add(tk);
      parent.set(tk, n);
      depth.set(tk, depth.get(k)! + 1);
      gcost.set(tk, gcost.get(k)! + cost);
      queue.push(to);
    }
  }
  return finishTrace(graph, start, goal, parent, gcost, pops, found);
}

/**
 * Depth-first search with a stack: pushes and pops on the same end, so it
 * commits to a branch and backs up only at a dead end. Complete on a finite
 * graph, optimal in nothing — Choset's figures H.8–H.9 in one sentence.
 */
export function dfs<N>(graph: Graph<N>, start: N, goal: N): Trace<N> {
  const parent = new Map<number | string, N>();
  const depth = new Map<number | string, number>();
  const gcost = new Map<number | string, number>();
  const visited = new Set<number | string>();
  const stack: N[] = [start];
  depth.set(graph.key(start), 0);
  gcost.set(graph.key(start), 0);
  const pops: Pop<N>[] = [];
  const goalKey = graph.key(goal);
  let found = false;
  while (stack.length) {
    const n = stack.pop()!;
    const k = graph.key(n);
    if (visited.has(k)) continue;
    visited.add(k);
    pops.push({ node: n, g: gcost.get(k)!, h: 0, f: depth.get(k)!, depth: depth.get(k)! });
    if (k === goalKey) {
      found = true;
      break;
    }
    for (const { to, cost } of graph.neighbors(n)) {
      const tk = graph.key(to);
      if (visited.has(tk)) continue;
      // The latest discovery wins the back pointer: that is what "deep first" means.
      parent.set(tk, n);
      depth.set(tk, depth.get(k)! + 1);
      gcost.set(tk, gcost.get(k)! + cost);
      stack.push(to);
    }
  }
  return finishTrace(graph, start, goal, parent, gcost, pops, found);
}

function finishTrace<N>(
  graph: Graph<N>,
  start: N,
  goal: N,
  parent: Map<number | string, N>,
  gcost: Map<number | string, number>,
  pops: Pop<N>[],
  found: boolean,
): Trace<N> {
  const path: N[] = [];
  if (found) {
    let cur: N | undefined = goal;
    const guard = parent.size + 2;
    for (let i = 0; cur !== undefined && i < guard; i++) {
      path.push(cur);
      if (graph.key(cur) === graph.key(start)) break;
      cur = parent.get(graph.key(cur));
    }
    path.reverse();
  }
  return {
    status: found ? 'found' : 'failed',
    pops,
    path,
    cost: found ? (gcost.get(graph.key(goal)) ?? Infinity) : Infinity,
    expansions: pops.length,
    reopened: 0,
  };
}

/** Sum of edge costs along a node path, re-measured on the graph. */
export function pathCost<N>(graph: Graph<N>, path: readonly N[]): number {
  let total = 0;
  for (let i = 1; i < path.length; i++) {
    const e = graph.neighbors(path[i - 1]).find((x) => graph.key(x.to) === graph.key(path[i]));
    if (!e) return Infinity;
    total += e.cost;
  }
  return total;
}
