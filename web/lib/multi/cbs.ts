/**
 * Conflict-Based Search on a grid — Sharon, Stern, Felner & Sturtevant (AIJ
 * 2015) — with the two planners it is measured against: prioritized planning
 * (Choset §7.5.2's "treat a robot as a moving obstacle as soon as its path has
 * been computed") and brute-force coupled A* over the joint state.
 *
 * All three share one low level: Chapter 6's A* run over (cell, time) on a
 * 4-connected grid with unit-time moves and waits. The only thing a
 * *constraint* does is delete some (cell, time) nodes — or some (from, to,
 * time) arcs — from that graph for one agent. CBS's high level is best-first
 * search over a binary tree of such constraint sets, keyed on the sum of
 * individual costs; prioritized planning is the degenerate tree with one
 * branch, where earlier robots become constraints that are never relaxed.
 *
 * Objective. An agent's cost is the number of time steps it spends not resting
 * at its goal: every move costs 1, every wait away from the goal costs 1, and
 * a wait at the goal costs 0. The sum over agents is SIC. This is the
 * joint-space convention (an agent that parks at its goal and never leaves is
 * charged exactly its arrival time, which is Sharon et al.'s path length), and
 * it is what lets the coupled A* and CBS minimize *the same* number — the
 * agreement the chapter's check asserts.
 *
 * Grid conventions are Chapter 6's: cells are `j·nx + i`, `isFreeCell`,
 * `cellIJ`; no diagonals here because an edge conflict needs a swap to be a
 * swap of one arc.
 */

import { BestFirst } from '../search/best-first';
import type { Edge, Graph } from '../search/graph';
import { cellIJ, cellIndex, gridFromAscii, isFreeCell, type OccupancyGrid } from '../search/grid';
import { KeyedHeap } from '../search/heap';

/* -------------------------------------------------------------------------- */
/* Instances                                                                   */
/* -------------------------------------------------------------------------- */

export interface MapfInstance {
  grid: OccupancyGrid;
  starts: number[];
  goals: number[];
  name?: string;
}

/** A cell per time step; `path[t]` is where the agent is at time t. */
export type TimedPath = number[];

/** 4-neighbors of a free cell, plus the cell itself (a wait). */
export function moves(grid: OccupancyGrid, cell: number): number[] {
  const [i, j] = cellIJ(grid, cell);
  const out = [cell];
  const steps: [number, number][] = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ];
  for (const [di, dj] of steps) {
    const ni = i + di;
    const nj = j + dj;
    if (ni < 0 || nj < 0 || ni >= grid.nx || nj >= grid.ny) continue;
    const nb = cellIndex(grid, ni, nj);
    if (isFreeCell(grid, nb)) out.push(nb);
  }
  return out;
}

export function manhattan(grid: OccupancyGrid, a: number, b: number): number {
  const [ai, aj] = cellIJ(grid, a);
  const [bi, bj] = cellIJ(grid, b);
  return Math.abs(ai - bi) + Math.abs(aj - bj);
}

/** Where an agent is at time t if it rests at the last cell of its path forever. */
export const at = (path: TimedPath, t: number) => path[Math.min(t, path.length - 1)];

/** SIC of a path under the convention above: steps not spent resting at the goal. */
export function pathCost(path: TimedPath, goal: number): number {
  let c = 0;
  for (let t = 1; t < path.length; t++) if (!(path[t] === goal && path[t - 1] === goal)) c++;
  return c;
}

export const sumOfCosts = (paths: readonly TimedPath[], goals: readonly number[]) => paths.reduce((s, p, i) => s + pathCost(p, goals[i]), 0);

export const makespan = (paths: readonly TimedPath[]) => Math.max(0, ...paths.map((p) => p.length - 1));

/* -------------------------------------------------------------------------- */
/* Constraints and conflicts                                                   */
/* -------------------------------------------------------------------------- */

/**
 * (agent, cell, t): the agent may not be at `cell` at time t. With `from` set
 * it is an edge constraint: the agent may not traverse from → cell arriving
 * at time t (Sharon et al.'s (aᵢ, u, v, t)).
 */
export interface Constraint {
  agent: number;
  cell: number;
  t: number;
  from?: number;
}

/**
 * Agents a and b occupy `cell` at time t (vertex conflict), or swap along an
 * arc between t − 1 and t (edge conflict: a goes aFrom → cell, b goes bFrom →
 * cell' with cell' = aFrom and bFrom = cell). For an edge conflict `cell` is
 * where a arrives.
 */
export interface Conflict {
  a: number;
  b: number;
  cell: number;
  t: number;
  edge: { aFrom: number; bFrom: number } | null;
}

/** The first conflict in time order, or null — the validator CBS and the checks share. */
export function firstConflict(paths: readonly TimedPath[]): Conflict | null {
  const T = makespan(paths);
  for (let t = 0; t <= T; t++) {
    for (let a = 0; a < paths.length; a++) {
      for (let b = a + 1; b < paths.length; b++) {
        const pa = at(paths[a], t);
        const pb = at(paths[b], t);
        if (pa === pb) return { a, b, cell: pa, t, edge: null };
        if (t > 0) {
          const qa = at(paths[a], t - 1);
          const qb = at(paths[b], t - 1);
          if (qa === pb && qb === pa && qa !== pa) return { a, b, cell: pa, t, edge: { aFrom: qa, bFrom: qb } };
        }
      }
    }
  }
  return null;
}

/** Every conflict, for the widget's count. */
export function countConflicts(paths: readonly TimedPath[]): number {
  let n = 0;
  const T = makespan(paths);
  for (let t = 0; t <= T; t++) {
    for (let a = 0; a < paths.length; a++) {
      for (let b = a + 1; b < paths.length; b++) {
        if (at(paths[a], t) === at(paths[b], t)) n++;
        else if (t > 0 && at(paths[a], t - 1) === at(paths[b], t) && at(paths[b], t - 1) === at(paths[a], t)) n++;
      }
    }
  }
  return n;
}

/** The two children of a conflict: one constraint for each agent (Sharon et al., Alg. 1 lines 13–17). */
export function splitConflict(c: Conflict): [Constraint, Constraint] {
  if (c.edge === null) {
    return [
      { agent: c.a, cell: c.cell, t: c.t },
      { agent: c.b, cell: c.cell, t: c.t },
    ];
  }
  // a traversed aFrom → cell; b traversed bFrom → aFrom (= cell of b at t).
  return [
    { agent: c.a, cell: c.cell, t: c.t, from: c.edge.aFrom },
    { agent: c.b, cell: c.edge.aFrom, t: c.t, from: c.edge.bFrom },
  ];
}

/* -------------------------------------------------------------------------- */
/* Low level: Chapter 6 A* over (cell, time)                                    */
/* -------------------------------------------------------------------------- */

interface TimedNode {
  cell: number;
  t: number;
}

/**
 * The time-expanded grid as a Chapter 6 `Graph`. Nodes are (cell, t); arcs are
 * the five moves to (cell′, t + 1), minus those a constraint deletes; the
 * horizon bounds t so that an infeasible query terminates. The goal is a
 * virtual node reached from (goal, t) whenever t is at or past the last vertex
 * constraint on the goal cell — the agent can rest there forever.
 */
class TimedGrid implements Graph<TimedNode> {
  private readonly vertexBan = new Set<number>();
  private readonly edgeBan = new Set<string>();
  readonly lastGoalBan: number;
  readonly ncells: number;

  constructor(
    readonly grid: OccupancyGrid,
    readonly goal: number,
    constraints: readonly Constraint[],
    readonly horizon: number,
  ) {
    this.ncells = grid.nx * grid.ny;
    let last = -1;
    for (const c of constraints) {
      if (c.from === undefined) {
        this.vertexBan.add(c.t * this.ncells + c.cell);
        if (c.cell === goal) last = Math.max(last, c.t);
      } else {
        this.edgeBan.add(`${c.from}>${c.cell}@${c.t}`);
      }
    }
    this.lastGoalBan = last;
  }

  /** The virtual terminal: t = −1 marks "resting at the goal from here on". */
  static readonly DONE: TimedNode = { cell: -1, t: -1 };

  neighbors(n: TimedNode): Edge<TimedNode>[] {
    if (n.t < 0) return [];
    const out: Edge<TimedNode>[] = [];
    if (n.cell === this.goal && n.t >= this.lastGoalBan) out.push({ to: TimedGrid.DONE, cost: 0 });
    if (n.t >= this.horizon) return out;
    const t1 = n.t + 1;
    for (const nb of moves(this.grid, n.cell)) {
      if (this.vertexBan.has(t1 * this.ncells + nb)) continue;
      if (nb !== n.cell && this.edgeBan.has(`${n.cell}>${nb}@${t1}`)) continue;
      // A wait at the goal is free; everything else costs one time step.
      const cost = nb === this.goal && n.cell === this.goal ? 0 : 1;
      out.push({ to: { cell: nb, t: t1 }, cost });
    }
    return out;
  }

  key(n: TimedNode): number {
    return n.t < 0 ? -1 : n.t * this.ncells + n.cell;
  }

  order(n: TimedNode): number {
    return n.t < 0 ? -1 : n.cell;
  }
}

export interface LowLevelResult {
  path: TimedPath | null;
  cost: number;
  expansions: number;
}

/**
 * One agent's optimal path under its constraints — Chapter 6's A* with the
 * Manhattan heuristic (admissible and consistent on a 4-connected grid with
 * unit moves and free goal-waits). `horizon` is the latest time step the
 * search may reach; a failure under the horizon is reported as `null`.
 */
export function timedAStar(grid: OccupancyGrid, start: number, goal: number, constraints: readonly Constraint[], horizon: number): LowLevelResult {
  const g = new TimedGrid(grid, goal, constraints, horizon);
  const h = (n: TimedNode) => (n.t < 0 ? 0 : manhattan(grid, n.cell, goal));
  const s = new BestFirst<TimedNode>(g, { cell: start, t: 0 }, TimedGrid.DONE, h);
  s.run();
  if (s.status !== 'found') return { path: null, cost: Infinity, expansions: s.pops.length };
  const nodes = s.path().filter((n) => n.t >= 0);
  const path = nodes.map((n) => n.cell);
  // Trailing goal-waits cost nothing and say nothing: trim them so the path ends at its final arrival.
  while (path.length > 1 && path[path.length - 1] === goal && path[path.length - 2] === goal) path.pop();
  return { path, cost: s.cost(), expansions: s.pops.length };
}

/** A sensible horizon: no optimal single-agent path on a grid of n free cells needs more than a few n steps. */
export function defaultHorizon(grid: OccupancyGrid, agents: number): number {
  let free = 0;
  for (let k = 0; k < grid.occupied.length; k++) if (grid.occupied[k] === 0) free++;
  return 2 * free + 4 * agents;
}

/* -------------------------------------------------------------------------- */
/* Prioritized planning (Choset §7.5.2)                                        */
/* -------------------------------------------------------------------------- */

/** Earlier robots as moving obstacles: every (cell, t) they occupy, forever at their goal, and every swap arc. */
export function constraintsFromPath(agent: number, path: TimedPath, horizon: number): Constraint[] {
  const out: Constraint[] = [];
  for (let t = 0; t <= horizon + 1; t++) {
    out.push({ agent, cell: at(path, t), t });
    if (t > 0 && at(path, t) !== at(path, t - 1)) out.push({ agent, cell: at(path, t - 1), t, from: at(path, t) });
  }
  return out;
}

export interface PrioritizedResult {
  paths: (TimedPath | null)[];
  /** Index (in `order`) of the first agent that found no path, or null when all succeeded. */
  failedAt: number | null;
  cost: number;
  expansions: number;
}

/**
 * Robots planned one at a time in `order`; each treats the robots before it as
 * moving obstacles. No backtracking, no re-planning of an earlier robot — which
 * is why it is incomplete (Derivation 2), and why it is so cheap.
 */
export function prioritizedPlan(inst: MapfInstance, order: readonly number[], horizon = defaultHorizon(inst.grid, inst.starts.length)): PrioritizedResult {
  const paths: (TimedPath | null)[] = inst.starts.map(() => null);
  const constraints: Constraint[] = [];
  let expansions = 0;
  for (let k = 0; k < order.length; k++) {
    const a = order[k];
    const r = timedAStar(inst.grid, inst.starts[a], inst.goals[a], constraints.map((c) => ({ ...c, agent: a })), horizon);
    expansions += r.expansions;
    if (!r.path) return { paths, failedAt: k, cost: Infinity, expansions };
    paths[a] = r.path;
    constraints.push(...constraintsFromPath(a, r.path, horizon));
  }
  return { paths, failedAt: null, cost: sumOfCosts(paths as TimedPath[], inst.goals), expansions };
}

/* -------------------------------------------------------------------------- */
/* CBS                                                                         */
/* -------------------------------------------------------------------------- */

export interface CtNode {
  id: number;
  parent: number | null;
  /** The one constraint this node added to its parent's set (none at the root). */
  added: Constraint | null;
  /** All constraints in force at this node. */
  constraints: Constraint[];
  paths: TimedPath[];
  /** SIC(N). */
  cost: number;
  /** The first conflict in `paths`, found when the node was expanded; null if conflict-free or not yet expanded. */
  conflict: Conflict | null;
  expanded: boolean;
  /** Low-level expansions spent re-planning for this node. */
  lowLevelExpansions: number;
}

export interface CbsResult {
  paths: TimedPath[];
  cost: number;
  /** High-level nodes expanded. */
  expansions: number;
  /** High-level nodes generated. */
  generated: number;
  lowLevelExpansions: number;
  /** Depth of the solution node in the constraint tree. */
  depth: number;
}

/**
 * Sharon et al., Algorithm 1. Resumable one high-level expansion at a time so
 * w14.4 can step through the constraint tree; `solve()` runs it to the end.
 */
export class Cbs {
  readonly nodes: CtNode[] = [];
  private readonly open = new KeyedHeap<number>();
  expansions = 0;
  lowLevelExpansions = 0;
  solution: CtNode | null = null;
  /** The tree is finite only because the low level has a horizon; this caps the high level too. */
  readonly maxExpansions: number;
  readonly horizon: number;

  constructor(
    readonly inst: MapfInstance,
    opts: { horizon?: number; maxExpansions?: number } = {},
  ) {
    this.horizon = opts.horizon ?? defaultHorizon(inst.grid, inst.starts.length);
    this.maxExpansions = opts.maxExpansions ?? 20000;
    // Root: every agent planned alone (Alg. 1 lines 1–4).
    const paths: TimedPath[] = [];
    let low = 0;
    for (let a = 0; a < inst.starts.length; a++) {
      const r = timedAStar(inst.grid, inst.starts[a], inst.goals[a], [], this.horizon);
      low += r.expansions;
      if (!r.path) {
        // An agent with no path at all: the instance is unsolvable; leave the tree empty.
        this.lowLevelExpansions = low;
        return;
      }
      paths.push(r.path);
    }
    this.lowLevelExpansions = low;
    this.push({ id: 0, parent: null, added: null, constraints: [], paths, cost: sumOfCosts(paths, inst.goals), conflict: null, expanded: false, lowLevelExpansions: low });
  }

  private push(n: CtNode): void {
    this.nodes.push(n);
    // Best-first on SIC; ties toward fewer conflicts (a common, optimality-preserving tie-break), then FIFO.
    this.open.push([n.cost, countConflicts(n.paths)], n.id);
  }

  get solved(): boolean {
    return this.solution !== null;
  }

  get exhausted(): boolean {
    return this.open.size === 0 || this.expansions >= this.maxExpansions;
  }

  /** The node the next `step()` will expand, for the widget's highlight. */
  peek(): CtNode | null {
    const e = this.open.peek();
    return e ? this.nodes[e.value] : null;
  }

  /**
   * One high-level expansion (Alg. 1 lines 6–17): pop the cheapest node, find
   * its first conflict, return it as the solution if there is none, otherwise
   * split. Returns the node expanded, or null when nothing is left.
   */
  step(): CtNode | null {
    if (this.solution || this.exhausted) return null;
    const top = this.open.pop();
    if (!top) return null;
    const n = this.nodes[top.value];
    this.expansions++;
    n.expanded = true;
    n.conflict = firstConflict(n.paths);
    if (n.conflict === null) {
      this.solution = n;
      return n;
    }
    for (const c of splitConflict(n.conflict)) {
      const constraints = [...n.constraints, c];
      const own = constraints.filter((k) => k.agent === c.agent);
      // Re-plan only the constrained agent (line 15): the other paths are inherited verbatim.
      const r = timedAStar(this.inst.grid, this.inst.starts[c.agent], this.inst.goals[c.agent], own, this.horizon);
      this.lowLevelExpansions += r.expansions;
      if (!r.path) continue; // the child is infeasible: prune (Alg. 1 line 17's "if cost < ∞")
      const paths = n.paths.slice();
      paths[c.agent] = r.path;
      this.push({ id: this.nodes.length, parent: n.id, added: c, constraints, paths, cost: sumOfCosts(paths, this.inst.goals), conflict: null, expanded: false, lowLevelExpansions: r.expansions });
    }
    return n;
  }

  solve(): CbsResult | null {
    while (!this.solution && !this.exhausted) this.step();
    if (!this.solution) return null;
    let depth = 0;
    for (let n: CtNode | null = this.solution; n && n.parent !== null; n = this.nodes[n.parent]) depth++;
    return { paths: this.solution.paths, cost: this.solution.cost, expansions: this.expansions, generated: this.nodes.length, lowLevelExpansions: this.lowLevelExpansions, depth };
  }
}

export function cbs(inst: MapfInstance, opts: { horizon?: number; maxExpansions?: number } = {}): CbsResult | null {
  return new Cbs(inst, opts).solve();
}

/* -------------------------------------------------------------------------- */
/* Brute force: A* over the joint state                                        */
/* -------------------------------------------------------------------------- */

export interface JointResult {
  paths: TimedPath[] | null;
  cost: number;
  /** Joint states expanded. */
  expansions: number;
  /** Joint successors generated (the branching factor, summed). */
  generated: number;
  /** Agents^5 — the branching factor an unpruned joint search faces. */
  branching: number;
}

/**
 * Centralized planning done literally: A* on the product of the m grids, with
 * the cartesian product of the five moves as the successor set, minus the
 * joint moves with a vertex or edge collision. Cost per step = the number of
 * agents not resting at their goal; heuristic = Σ Manhattan. Exact, optimal
 * under the same SIC as CBS, and exponential in m — which is the point of
 * running it next to CBS on the same instance.
 */
export function jointAStar(inst: MapfInstance, maxExpansions = 200000): JointResult {
  const { grid, starts, goals } = inst;
  const m = starts.length;
  const key = (s: readonly number[]) => s.join(',');
  const h = (s: readonly number[]) => s.reduce((acc, c, i) => acc + manhattan(grid, c, goals[i]), 0);
  const heap = new KeyedHeap<{ state: number[]; g: number }>();
  const gBest = new Map<string, number>();
  const parent = new Map<string, number[] | null>();
  const closed = new Set<string>();
  const startKey = key(starts);
  gBest.set(startKey, 0);
  parent.set(startKey, null);
  heap.push([h(starts), h(starts)], { state: starts.slice(), g: 0 });
  let expansions = 0;
  let generated = 0;
  const goalKey = key(goals);
  while (heap.size > 0 && expansions < maxExpansions) {
    const top = heap.pop()!;
    const { state, g } = top.value;
    const k = key(state);
    if (closed.has(k) || g > (gBest.get(k) ?? Infinity) + 1e-12) continue;
    closed.add(k);
    expansions++;
    if (k === goalKey) {
      const states: number[][] = [];
      for (let cur: number[] | null = state; cur; cur = parent.get(key(cur)) ?? null) states.push(cur);
      states.reverse();
      const paths: TimedPath[] = starts.map((_, i) => states.map((s) => s[i]));
      for (let i = 0; i < m; i++) {
        const p = paths[i];
        while (p.length > 1 && p[p.length - 1] === goals[i] && p[p.length - 2] === goals[i]) p.pop();
      }
      return { paths, cost: g, expansions, generated, branching: 5 ** m };
    }
    // Enumerate the product of per-agent moves with an odometer.
    const options = state.map((c) => moves(grid, c));
    const idx = new Array<number>(m).fill(0);
    for (;;) {
      const next = idx.map((o, i) => options[i][o]);
      let ok = true;
      for (let a = 0; a < m && ok; a++) {
        for (let b = a + 1; b < m; b++) {
          if (next[a] === next[b] || (next[a] === state[b] && next[b] === state[a] && next[a] !== state[a])) {
            ok = false;
            break;
          }
        }
      }
      if (ok) {
        generated++;
        let step = 0;
        for (let i = 0; i < m; i++) if (!(next[i] === goals[i] && state[i] === goals[i])) step++;
        const nk = key(next);
        const ng = g + step;
        if (ng < (gBest.get(nk) ?? Infinity) - 1e-12) {
          gBest.set(nk, ng);
          parent.set(nk, state);
          const hh = h(next);
          heap.push([ng + hh, hh], { state: next, g: ng });
        }
      }
      let d = 0;
      while (d < m) {
        idx[d]++;
        if (idx[d] < options[d].length) break;
        idx[d] = 0;
        d++;
      }
      if (d === m) break;
    }
  }
  return { paths: null, cost: Infinity, expansions, generated, branching: 5 ** m };
}

/* -------------------------------------------------------------------------- */
/* Instances                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * Derivation 2's instance: a one-cell-wide corridor with a single alcove in
 * its north wall, and two robots that must swap ends. Each robot's shortest
 * path alone is the corridor axis; whoever is planned first takes it and the
 * second robot can never pass — in either order. The coupled answer is "step
 * into the alcove and wait", which only CBS (or the joint search) can say.
 */
export const SWAP_GRID = gridFromAscii(['###.###', '.......']);
export const SWAP_INSTANCE: MapfInstance = {
  name: 'swap corridor',
  grid: SWAP_GRID,
  starts: [cellIndex(SWAP_GRID, 0, 0), cellIndex(SWAP_GRID, 6, 0)],
  goals: [cellIndex(SWAP_GRID, 6, 0), cellIndex(SWAP_GRID, 0, 0)],
};

/** The same corridor, both robots heading east: no swap needed, prioritized planning succeeds. */
export const FOLLOW_INSTANCE: MapfInstance = {
  name: 'follow corridor',
  grid: SWAP_GRID,
  starts: [cellIndex(SWAP_GRID, 0, 0), cellIndex(SWAP_GRID, 1, 0)],
  goals: [cellIndex(SWAP_GRID, 5, 0), cellIndex(SWAP_GRID, 6, 0)],
};

/**
 * A small room with two pillars for w14.4: 2–5 Rustys crossing. Starts along
 * the west wall, goals along the east wall in reversed order, so every pair
 * must cross somewhere in the middle.
 */
export const ROOM_GRID = gridFromAscii(['.......', '..#....', '.......', '....#..', '.......']);

export function roomInstance(agents: number): MapfInstance {
  const n = Math.max(2, Math.min(5, agents));
  const rows = [0, 4, 2, 1, 3].slice(0, n);
  const starts = rows.map((j) => cellIndex(ROOM_GRID, 0, j));
  const goals = rows.map((j) => cellIndex(ROOM_GRID, 6, ROOM_GRID.ny - 1 - j));
  return { name: `room, ${n} agents`, grid: ROOM_GRID, starts, goals };
}
