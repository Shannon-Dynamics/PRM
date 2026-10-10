/**
 * Choset Algorithm 21, GRID SEARCH (§11.3.3), on Chapter 6's best-first search.
 *
 * A double integrator q̈_i = a_i with a_i ∈ {−a_max, 0, a_max} held for time h
 * moves q̇_i by a multiple of a_max h and q_i by a multiple of ½ a_max h²:
 *
 *     Δm = c,            m := q̇ / (a_max h)             (velocity index)
 *     Δp = 2m + c,       p := q / (½ a_max h²)           (position index)
 *
 * so every reachable state lies on an integer lattice and the search tree of
 * figure 11.8 *is* a graph on that lattice (figure 11.9). Level = elapsed
 * time, so Choset's breadth-first expansion is uniform-cost search with unit
 * edges; A* with the per-axis bang-bang time as heuristic finds the same
 * level with fewer expansions, because the continuous minimum time on each
 * axis separately is a lower bound on the lattice time (it ignores the other
 * axes, the obstacles, the velocity limit and the discretization).
 *
 * Safety is Choset's δ_v(c₀, c₁): a node is kept only if every sampled point of
 * the quadratic arc into it has clearance at least c₀ + c₁‖q̇‖ and |q̇_i| ≤ v_max.
 */

import { BestFirst, PRIORITY_ASTAR, PRIORITY_BFS, type Priority } from '../search/best-first';
import type { Edge, Graph } from '../search/graph';

export interface LatticeState {
  /** Integer position indices, one per axis: q_i = p_i · ½ a_max h². */
  p: number[];
  /** Integer velocity indices: q̇_i = m_i · a_max h. */
  m: number[];
}

export interface LatticeParams {
  aMax: number;
  h: number;
  vMax: number;
  /** Clearance check at a configuration; the lattice calls it along every arc. */
  clearance?: (q: number[]) => number;
  /** δ_v(c₀, c₁) safety margin: required clearance c₀ + c₁‖q̇‖. */
  c0?: number;
  c1?: number;
  /** Sub-samples per arc for the safety check. */
  arcSamples?: number;
  /** Goal neighbourhood: |q − q_goal|∞ ≤ goalPos and |q̇ − q̇_goal|∞ ≤ goalVel (lattice units if omitted: exact). */
  goalPos?: number;
  goalVel?: number;
}

export function latticeToState(n: LatticeState, prm: LatticeParams): { q: number[]; qd: number[] } {
  const dq = 0.5 * prm.aMax * prm.h * prm.h;
  const dv = prm.aMax * prm.h;
  return { q: n.p.map((v) => v * dq), qd: n.m.map((v) => v * dv) };
}

/** Snap a continuous state to the lattice the way Theorem 11.3.2 does: nearest velocity multiple, position adjusted. */
export function stateToLattice(q: number[], qd: number[], prm: LatticeParams): LatticeState {
  const dq = 0.5 * prm.aMax * prm.h * prm.h;
  const dv = prm.aMax * prm.h;
  return { p: q.map((v) => Math.round(v / dq)), m: qd.map((v) => Math.round(v / dv)) };
}

/** All 3ⁿ controls in {−1, 0, 1}ⁿ (units of a_max), lexicographic. */
export function controlSet(n: number): number[][] {
  const out: number[][] = [];
  const total = 3 ** n;
  for (let k = 0; k < total; k++) {
    const c: number[] = [];
    let r = k;
    for (let i = 0; i < n; i++) {
      c.push((r % 3) - 1);
      r = Math.floor(r / 3);
    }
    out.push(c);
  }
  return out;
}

/**
 * The lattice as a `Graph` for Chapter 6's search. `key` collapses every state
 * inside the goal neighbourhood to the single key "GOAL", so the search can
 * stop at a region rather than a point — Choset's "trajectory enters G".
 */
export class LatticeGraph implements Graph<LatticeState> {
  readonly n: number;
  readonly controls: number[][];
  readonly goal: LatticeState;
  private readonly goalQ: number[];
  private readonly goalQd: number[];
  /** How many nodes the safety test rejected — the widget's pruned-node count. */
  pruned = 0;

  constructor(
    readonly prm: LatticeParams,
    goal: LatticeState,
  ) {
    this.n = goal.p.length;
    this.controls = controlSet(this.n);
    this.goal = goal;
    const g = latticeToState(goal, prm);
    this.goalQ = g.q;
    this.goalQd = g.qd;
  }

  inGoal(s: LatticeState): boolean {
    const { goalPos, goalVel } = this.prm;
    if (goalPos === undefined && goalVel === undefined) {
      return s.p.every((v, i) => v === this.goal.p[i]) && s.m.every((v, i) => v === this.goal.m[i]);
    }
    const st = latticeToState(s, this.prm);
    const gp = goalPos ?? 0;
    const gv = goalVel ?? 0;
    for (let i = 0; i < this.n; i++) {
      if (Math.abs(st.q[i] - this.goalQ[i]) > gp + 1e-12) return false;
      if (Math.abs(st.qd[i] - this.goalQd[i]) > gv + 1e-12) return false;
    }
    return true;
  }

  key(s: LatticeState): string {
    if (this.inGoal(s)) return 'GOAL';
    return `${s.p.join(',')}|${s.m.join(',')}`;
  }

  /** Is the arc from `from` under control c safe? Samples the quadratic q(τ) = q + q̇τ + ½aτ². */
  private arcSafe(from: LatticeState, c: number[]): boolean {
    const { aMax, h, vMax, clearance, c0 = 0, c1 = 0, arcSamples = 4 } = this.prm;
    const st = latticeToState(from, this.prm);
    const dv = aMax * h;
    // Velocity limit at the end of the step (velocity is monotone along the arc).
    for (let i = 0; i < this.n; i++) if (Math.abs((from.m[i] + c[i]) * dv) > vMax + 1e-12) return false;
    if (!clearance) return true;
    for (let k = 1; k <= arcSamples; k++) {
      const tau = (k / arcSamples) * h;
      const q = st.q.map((v, i) => v + st.qd[i] * tau + 0.5 * c[i] * aMax * tau * tau);
      const qd = st.qd.map((v, i) => v + c[i] * aMax * tau);
      const speed = Math.hypot(...qd);
      if (clearance(q) < c0 + c1 * speed) return false;
    }
    return true;
  }

  neighbors(s: LatticeState): Edge<LatticeState>[] {
    const out: Edge<LatticeState>[] = [];
    if (this.inGoal(s)) return out;
    for (const c of this.controls) {
      const next: LatticeState = { p: s.p.map((v, i) => v + 2 * s.m[i] + c[i]), m: s.m.map((v, i) => v + c[i]) };
      if (!this.arcSafe(s, c)) {
        this.pruned++;
        continue;
      }
      out.push({ to: next, cost: this.prm.h });
    }
    return out;
  }
}

/**
 * Minimum time for a 1-D double integrator |q̈| ≤ a from (q₀, v₀) to (q₁, v₁):
 * one switch, accelerate-then-brake or brake-then-accelerate. Closed form via
 * the peak velocity v_p: (v_p² − v₀²)/2a + (v_p² − v₁²)/2a = Δq for the first
 * family and the mirror image for the second. This is also Chapter 19's
 * min-time double integrator read off the minimum principle.
 */
export function bangBangTime1D(q0: number, v0: number, q1: number, v1: number, a: number): number {
  const dq = q1 - q0;
  let best = Infinity;
  const r1 = a * dq + 0.5 * (v0 * v0 + v1 * v1);
  if (r1 >= 0) {
    const vp = Math.sqrt(r1);
    if (vp >= Math.max(v0, v1) - 1e-12) best = Math.min(best, (vp - v0) / a + (vp - v1) / a);
  }
  const r2 = -a * dq + 0.5 * (v0 * v0 + v1 * v1);
  if (r2 >= 0) {
    const vp = -Math.sqrt(r2);
    if (vp <= Math.min(v0, v1) + 1e-12) best = Math.min(best, (v0 - vp) / a + (v1 - vp) / a);
  }
  return Number.isFinite(best) ? best : 0;
}

/** h(n) = max_i (per-axis bang-bang time to the goal): admissible and consistent. */
export function bangBangHeuristic(graph: LatticeGraph): (n: LatticeState) => number {
  const g = latticeToState(graph.goal, graph.prm);
  return (n) => {
    if (graph.inGoal(n)) return 0;
    const st = latticeToState(n, graph.prm);
    let hmax = 0;
    for (let i = 0; i < graph.n; i++) hmax = Math.max(hmax, bangBangTime1D(st.q[i], st.qd[i], g.q[i], g.qd[i], graph.prm.aMax));
    return hmax;
  };
}

export interface LatticeResult {
  found: boolean;
  /** Piecewise-constant accelerations (units of a_max), one per level. */
  controls: number[][];
  /** The lattice states visited, start first. */
  states: LatticeState[];
  levels: number;
  tf: number;
  expansions: number;
  pruned: number;
  /** Continuous bang-bang lower bound max_i T_i from the start. */
  continuousBound: number;
}

/** Recover the control sequence from consecutive lattice states: c = Δm. */
export function controlsFromStates(states: readonly LatticeState[]): number[][] {
  const out: number[][] = [];
  for (let k = 1; k < states.length; k++) out.push(states[k].m.map((v, i) => v - states[k - 1].m[i]));
  return out;
}

/**
 * `lattice_search`: Algorithm 21 with A* (default) or Choset's literal
 * breadth-first expansion. Both return a trajectory of the same number of
 * levels; A* expands fewer nodes.
 */
export function latticeSearch(start: LatticeState, goal: LatticeState, prm: LatticeParams, variant: 'astar' | 'bfs' = 'astar', limit = 2e5): LatticeResult {
  const graph = new LatticeGraph(prm, goal);
  const h = variant === 'astar' ? bangBangHeuristic(graph) : () => 0;
  const prio: Priority = variant === 'astar' ? PRIORITY_ASTAR : PRIORITY_BFS;
  const search = new BestFirst<LatticeState>(graph, start, goal, h, prio);
  search.run(limit);
  const trace = search.trace();
  const found = trace.status === 'found';
  const states = found ? trace.path : [];
  const s0 = latticeToState(start, prm);
  const g0 = latticeToState(goal, prm);
  let bound = 0;
  for (let i = 0; i < start.p.length; i++) bound = Math.max(bound, bangBangTime1D(s0.q[i], s0.qd[i], g0.q[i], g0.qd[i], prm.aMax));
  return {
    found,
    controls: controlsFromStates(states),
    states,
    levels: found ? states.length - 1 : 0,
    tf: found ? (states.length - 1) * prm.h : Infinity,
    expansions: trace.expansions,
    pruned: graph.pruned,
    continuousBound: bound,
  };
}

/**
 * Incremental version for the widget: one `step()` per expansion, exposing the
 * open/closed sets and the level of each node so the tree can be painted as it grows.
 */
export class LatticeSearchStepper {
  readonly graph: LatticeGraph;
  readonly search: BestFirst<LatticeState>;
  constructor(
    readonly start: LatticeState,
    readonly goal: LatticeState,
    readonly prm: LatticeParams,
    variant: 'astar' | 'bfs' = 'astar',
  ) {
    this.graph = new LatticeGraph(prm, goal);
    const h = variant === 'astar' ? bangBangHeuristic(this.graph) : () => 0;
    this.search = new BestFirst<LatticeState>(this.graph, start, goal, h, variant === 'astar' ? PRIORITY_ASTAR : PRIORITY_BFS);
  }
  step(n = 1): void {
    for (let i = 0; i < n && this.search.status === 'running'; i++) this.search.step();
  }
  get status() {
    return this.search.status;
  }
  result(): LatticeResult {
    const trace = this.search.trace();
    const found = trace.status === 'found';
    const states = found ? trace.path : [];
    return {
      found,
      controls: controlsFromStates(states),
      states,
      levels: found ? states.length - 1 : 0,
      tf: found ? (states.length - 1) * this.prm.h : Infinity,
      expansions: trace.expansions,
      pruned: this.graph.pruned,
      continuousBound: 0,
    };
  }
}

/**
 * Choset's Algorithm 21 *as printed*: expand the tree level by level, prune
 * unsafe or repeated nodes, and stop as soon as a generated node enters G.
 * Testing the goal at generation (line 13) saves one full level of expansion
 * over a best-first search that tests it at the pop, which for a 2-DOF lattice
 * is a factor of nine. The result has the same number of levels as A* — both
 * are time-optimal for the discretization — and this is the version the
 * prose quotes.
 */
export function gridSearch(start: LatticeState, goal: LatticeState, prm: LatticeParams, maxLevels = 200): LatticeResult {
  const graph = new LatticeGraph(prm, goal);
  const keyOf = (s: LatticeState) => `${s.p.join(',')}|${s.m.join(',')}`;
  const parent = new Map<string, LatticeState | null>([[keyOf(start), null]]);
  let level: LatticeState[] = [start];
  let expansions = 0;
  let found: LatticeState | null = null;
  const s0 = latticeToState(start, prm);
  const g0 = latticeToState(goal, prm);
  let bound = 0;
  for (let i = 0; i < start.p.length; i++) bound = Math.max(bound, bangBangTime1D(s0.q[i], s0.qd[i], g0.q[i], g0.qd[i], prm.aMax));
  if (graph.inGoal(start)) found = start;
  for (let depth = 0; depth < maxLevels && !found && level.length > 0; depth++) {
    const next: LatticeState[] = [];
    for (const node of level) {
      expansions++;
      for (const { to } of graph.neighbors(node)) {
        const k = keyOf(to);
        if (parent.has(k)) continue;
        parent.set(k, node);
        next.push(to);
        if (graph.inGoal(to)) {
          found = to;
          break;
        }
      }
      if (found) break;
    }
    level = next;
  }
  const states: LatticeState[] = [];
  for (let cur: LatticeState | null = found; cur; cur = parent.get(keyOf(cur)) ?? null) states.push(cur);
  states.reverse();
  return {
    found: found !== null,
    controls: controlsFromStates(states),
    states,
    levels: found ? states.length - 1 : 0,
    tf: found ? (states.length - 1) * prm.h : Infinity,
    expansions,
    pruned: graph.pruned,
    continuousBound: bound,
  };
}

/** Every distinct lattice state reachable in exactly `levels` steps from rest at the origin, no obstacles (figure 11.9's count). */
export function reachableStates(n: number, levels: number): number {
  const controls = controlSet(n);
  let frontier = new Map<string, LatticeState>([['0', { p: new Array(n).fill(0), m: new Array(n).fill(0) }]]);
  for (let l = 0; l < levels; l++) {
    const next = new Map<string, LatticeState>();
    for (const s of frontier.values()) {
      for (const c of controls) {
        const ns: LatticeState = { p: s.p.map((v, i) => v + 2 * s.m[i] + c[i]), m: s.m.map((v, i) => v + c[i]) };
        next.set(`${ns.p.join(',')}|${ns.m.join(',')}`, ns);
      }
    }
    frontier = next;
  }
  return frontier.size;
}
