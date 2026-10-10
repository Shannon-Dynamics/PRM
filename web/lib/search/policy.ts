/**
 * Universal plans — Choset App. H.4.
 *
 * Every search in this chapter returns a path. A path is a plan for *one*
 * start. Run Dijkstra backward from the goal with no early exit and you get a
 * plan for every node at once: its cost-to-goal and the successor to take.
 * Choset calls this a universal plan or policy; the labels satisfy the
 * fixed-point equation
 *
 *     h(X) = min_Y [ c(X, Y) + h(Y) ],   h(goal) = 0,
 *
 * which is a Bellman equation with deterministic transitions. The sister
 * book's Chapter 21 replaces min with an expectation and calls h a value
 * function; nothing else changes. Chapter 7's wave-front planner is this on a
 * 4-connected grid with unit costs.
 *
 * Backward search needs predecessors. For the graphs this chapter searches
 * (grids, roadmaps) every edge is symmetric, so neighbours serve; a directed
 * graph would need its reverse, and the generic version says so.
 */

import { KeyedHeap } from './heap';
import type { Graph } from './graph';
import type { GridGraph } from './grid';

export interface UniversalPlan<N> {
  goal: N;
  costToGo: Map<number | string, number>;
  /** The successor to take from each node; the goal maps to itself. */
  next: Map<number | string, N>;
  /** The node objects, by key, so a widget can iterate the field. */
  nodes: Map<number | string, N>;
}

/**
 * Backward Dijkstra on a graph with symmetric edges. Runs to exhaustion: the
 * cost of a universal plan is one full Dijkstra, after which every query —
 * from anywhere, after any disturbance — is a table lookup.
 */
export function universalPlan<N>(graph: Graph<N>, goal: N): UniversalPlan<N> {
  const costToGo = new Map<number | string, number>();
  const next = new Map<number | string, N>();
  const nodes = new Map<number | string, N>();
  const closed = new Set<number | string>();
  const heap = new KeyedHeap<N>();
  const gk = graph.key(goal);
  costToGo.set(gk, 0);
  next.set(gk, goal);
  nodes.set(gk, goal);
  heap.push([0, graph.order?.(goal) ?? 0], goal);
  for (;;) {
    const top = heap.pop();
    if (!top) break;
    const u = top.value;
    const uk = graph.key(u);
    if (closed.has(uk)) continue;
    if (top.key[0] > (costToGo.get(uk) ?? Infinity) + 1e-12) continue;
    closed.add(uk);
    const hu = costToGo.get(uk)!;
    for (const { to, cost } of graph.neighbors(u)) {
      const tk = graph.key(to);
      const cand = hu + cost;
      if (cand < (costToGo.get(tk) ?? Infinity) - 1e-12) {
        costToGo.set(tk, cand);
        next.set(tk, u);
        nodes.set(tk, to);
        heap.push([cand, graph.order?.(to) ?? 0], to);
      }
    }
  }
  return { goal, costToGo, next, nodes };
}

/** Execute the policy from `start`: follow successors to the goal. Empty if `start` is unreachable. */
export function followPlan<N>(graph: Graph<N>, plan: UniversalPlan<N>, start: N, maxSteps = 1e6): N[] {
  const out: N[] = [];
  let cur = start;
  for (let i = 0; i < maxSteps; i++) {
    const k = graph.key(cur);
    if (!plan.costToGo.has(k)) return [];
    out.push(cur);
    if (k === graph.key(plan.goal)) return out;
    cur = plan.next.get(k)!;
  }
  return out;
}

/**
 * The largest violation of h(X) = min_Y [c(X, Y) + h(Y)] over the labelled
 * nodes — zero (to rounding) for a correct plan. This is the test that the
 * table *is* a fixed point of the Bellman operator, which is what Chapter 7
 * and the sister book's value iteration converge to.
 */
export function bellmanResidual<N>(graph: Graph<N>, plan: UniversalPlan<N>): number {
  let worst = 0;
  for (const [k, X] of plan.nodes) {
    if (k === graph.key(plan.goal)) continue;
    const hX = plan.costToGo.get(k)!;
    let best = Infinity;
    for (const { to, cost } of graph.neighbors(X)) {
      const hY = plan.costToGo.get(graph.key(to));
      if (hY !== undefined) best = Math.min(best, cost + hY);
    }
    worst = Math.max(worst, Math.abs(hX - best));
  }
  return worst;
}

/* -------------------------------------------------------------------------- */
/* Grid specialization, for the Universal Plan Field widget                   */
/* -------------------------------------------------------------------------- */

export interface GridPlan {
  goal: number;
  /** Cost-to-goal per cell, ∞ where unreachable or occupied. */
  costToGo: Float64Array;
  /** Successor cell per cell, −1 where none; the goal points to itself. */
  next: Int32Array;
  /** Largest finite cost-to-goal, for colour scaling. */
  maxCost: number;
}

export function universalPlanGrid(graph: GridGraph, goal: number): GridPlan {
  const n = graph.grid.nx * graph.grid.ny;
  const costToGo = new Float64Array(n).fill(Infinity);
  const next = new Int32Array(n).fill(-1);
  const closed = new Uint8Array(n);
  const heap = new KeyedHeap<number>();
  costToGo[goal] = 0;
  next[goal] = goal;
  heap.push([0, graph.order(goal)], goal);
  let maxCost = 0;
  for (;;) {
    const top = heap.pop();
    if (!top) break;
    const u = top.value;
    if (closed[u]) continue;
    if (top.key[0] > costToGo[u] + 1e-12) continue;
    closed[u] = 1;
    maxCost = Math.max(maxCost, costToGo[u]);
    for (const { to, cost } of graph.neighbors(u)) {
      const cand = costToGo[u] + cost;
      if (cand < costToGo[to] - 1e-12) {
        costToGo[to] = cand;
        next[to] = u;
        heap.push([cand, graph.order(to)], to);
      }
    }
  }
  return { goal, costToGo, next, maxCost };
}

export function followGridPlan(plan: GridPlan, start: number, maxSteps = 1e6): number[] {
  const out: number[] = [];
  let cur = start;
  for (let i = 0; i < maxSteps; i++) {
    if (!Number.isFinite(plan.costToGo[cur])) return [];
    out.push(cur);
    if (cur === plan.goal) return out;
    cur = plan.next[cur];
  }
  return out;
}
