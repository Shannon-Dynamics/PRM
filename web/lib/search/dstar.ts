/**
 * D* — Choset App. H.3, Algorithms 25–31 (Stentz's original), the teachable
 * incremental planner.
 *
 * D* runs Dijkstra *backward* from the goal once, labelling every state X with
 * h(X), its cost-to-goal, a minimum-key k(X), a tag t(X) ∈ {NEW, OPEN, CLOSED},
 * and a back pointer b(X) toward the goal. When the robot discovers that an
 * arc cost differs from what the map said, the affected states are re-inserted
 * and PROCESS-STATE propagates the change only as far as it needs to: until the
 * smallest key on the open list is no smaller than the robot's own h, at which
 * point no cheaper repair can reach it (Alg. 25, line 20).
 *
 * The two kinds of state are the whole insight. A RAISE state (k < h) has
 * learned its old path got worse and looks for a neighbour whose chain avoided
 * the change; a LOWER state (k = h) has good news and hands it to its
 * descendants. Figure H.26's "h(4,1) + 1.4 = 7.6" is a LOWER neighbour
 * rescuing a RAISE state.
 *
 * Honesty items the chapter keeps:
 *  (i)  Choset's h(X) is a cost-to-goal, not A*'s heuristic. The arrays here are
 *       named `costToGo` and `kmin` and the comments say h and k.
 *  (ii) Obstacle cells stay in the graph as 10000-cost arcs, as in §H.2.4, so
 *       the printed numbers (h(3,2) = 10004.6 while raised) are reproducible.
 *       Chapter-wide, obstacles are deleted from the graph; this file is the
 *       one deliberate exception.
 *  (iii) Alg. 31 as typeset in Choset makes the LOWER test an `else if` after
 *       the RAISE fix-up; Stentz's original makes it a plain `if`, so a state
 *       that was just lowered still propagates in the same call. We follow
 *       Stentz, and say so in the text.
 *
 * The `latticeNeighbors` of the grid graph are used throughout — D* must
 * reason about arcs into obstacles, not only the free graph.
 */

import { KeyedHeap } from './heap';
import { GridGraph, cloneGrid, type GridGraphOptions, type OccupancyGrid } from './grid';

export const TAG_NEW = 0;
export const TAG_OPEN = 1;
export const TAG_CLOSED = 2;

/** Choset's "arbitrarily high" cost for an arc that touches an obstacle (figure H.16). */
export const OBSTACLE_COST = 10000;

const EPS = 1e-9;

export interface RepairStats {
  /** States popped by PROCESS-STATE during this repair, in order. */
  popped: number[];
  /** Of those, the ones popped as RAISE states (k < h). */
  raised: number[];
  /** Of those, the ones popped as LOWER states (k = h). */
  lowered: number[];
  /** Distinct states inserted into the open list during this repair. */
  inserted: number[];
}

export class DStar {
  readonly believed: OccupancyGrid;
  readonly graph: GridGraph;
  readonly tag: Uint8Array;
  /** h(X): the estimated cost from X to the goal. */
  readonly costToGo: Float64Array;
  /** k(X): the minimum h(X) has taken since X was last put on the open list. */
  readonly kmin: Float64Array;
  /** b(X): the parent toward the goal, −1 when none. */
  readonly back: Int32Array;
  readonly goal: number;

  private readonly heap = new KeyedHeap<number>();
  private recording: RepairStats | null = null;
  /** Total PROCESS-STATE pops since construction — the "work done" counter. */
  expansions = 0;

  constructor(grid: OccupancyGrid, goal: number, opts: GridGraphOptions = {}) {
    this.believed = cloneGrid(grid);
    this.graph = new GridGraph(this.believed, opts);
    const n = grid.nx * grid.ny;
    this.tag = new Uint8Array(n);
    this.costToGo = new Float64Array(n).fill(Infinity);
    this.kmin = new Float64Array(n).fill(Infinity);
    this.back = new Int32Array(n).fill(-1);
    this.goal = goal;
    this.costToGo[goal] = 0;
    this.insert(goal, 0);
  }

  /**
   * c(X, Y): the step length, plus Choset's penalty when either state is an
   * obstacle. Symmetric, so backward and forward costs agree.
   */
  cost(X: number, Y: number): number {
    const occ = this.believed.occupied;
    let blocked = occ[X] === 1 || occ[Y] === 1;
    if (!blocked && !this.graph.cornerCutting) {
      // A diagonal squeeze between two orthogonally adjacent obstacles counts as
      // touching them — the same rule the free graph and D* Lite apply.
      const nx = this.believed.nx;
      const xi = X % nx, xj = (X - xi) / nx, yi = Y % nx, yj = (Y - yi) / nx;
      if (xi !== yi && xj !== yj && (occ[xj * nx + yi] || occ[yj * nx + xi])) blocked = true;
    }
    return this.graph.stepCost(X, Y) + (blocked ? OBSTACLE_COST : 0);
  }

  /** Algorithm 27. */
  insert(X: number, hnew: number): void {
    const t = this.tag[X];
    if (t === TAG_NEW) this.kmin[X] = hnew;
    else if (t === TAG_OPEN) this.kmin[X] = Math.min(this.kmin[X], hnew);
    else this.kmin[X] = Math.min(this.costToGo[X], hnew);
    this.costToGo[X] = hnew;
    this.tag[X] = TAG_OPEN;
    this.heap.push([this.kmin[X]], X);
    if (this.recording && !this.recording.inserted.includes(X)) this.recording.inserted.push(X);
  }

  /** Algorithm 29 — with stale heap entries discarded on the way. */
  private minState(): number {
    for (;;) {
      const top = this.heap.peek();
      if (!top) return -1;
      const X = top.value;
      if (this.tag[X] !== TAG_OPEN || Math.abs(top.key[0] - this.kmin[X]) > EPS) {
        this.heap.pop();
        continue;
      }
      return X;
    }
  }

  /** Algorithm 30. */
  getKmin(): number {
    const X = this.minState();
    return X < 0 ? -1 : this.kmin[X];
  }

  /** Algorithm 31, in Stentz's original structure (see honesty item iii). */
  processState(): number {
    const X = this.minState();
    if (X < 0) return -1;
    const kold = this.kmin[X];
    this.heap.pop();
    this.tag[X] = TAG_CLOSED;
    this.expansions++;

    const h = this.costToGo;
    const b = this.back;
    const neighbors = this.graph.latticeNeighbors(X);
    const raise = kold < h[X] - EPS;
    if (this.recording) {
      this.recording.popped.push(X);
      (raise ? this.recording.raised : this.recording.lowered).push(X);
    }

    if (raise) {
      // RAISE: look for a neighbour whose own path did not get worse (h(Y) ≤ k_old)
      // and that offers a cheaper route — figure H.26's (4,1) rescuing (3,2).
      for (const Y of neighbors) {
        const c = this.cost(Y, X);
        if (h[Y] <= kold + EPS && h[X] > h[Y] + c + EPS) {
          b[X] = Y;
          h[X] = h[Y] + c;
        }
      }
    }

    if (Math.abs(kold - h[X]) <= EPS) {
      // LOWER: propagate the (possibly new) h to every descendant and to any
      // neighbour we can now improve.
      for (const Y of neighbors) {
        const c = this.cost(X, Y);
        if (
          this.tag[Y] === TAG_NEW ||
          (b[Y] === X && Math.abs(h[Y] - (h[X] + c)) > EPS) ||
          (b[Y] !== X && h[Y] > h[X] + c + EPS)
        ) {
          b[Y] = X;
          this.insert(Y, h[X] + c);
        }
      }
    } else {
      // Still RAISE after the fix-up: tell descendants, and re-open any neighbour
      // that could be the one to rescue us later (lines 22–31 of Alg. 31).
      for (const Y of neighbors) {
        const c = this.cost(X, Y);
        if (this.tag[Y] === TAG_NEW || (b[Y] === X && Math.abs(h[Y] - (h[X] + c)) > EPS)) {
          b[Y] = X;
          this.insert(Y, h[X] + c);
        } else if (b[Y] !== X && h[Y] > h[X] + c + EPS) {
          this.insert(X, h[X]);
        } else if (b[Y] !== X && h[X] > h[Y] + c + EPS && this.tag[Y] === TAG_CLOSED && h[Y] > kold + EPS) {
          this.insert(Y, h[Y]);
        }
      }
    }
    return this.getKmin();
  }

  /**
   * Algorithm 25, lines 7–9: the initial Dijkstra from the goal, run until the
   * start is CLOSED (or the open list empties — no path).
   */
  planInitial(start: number): boolean {
    let kmin = this.getKmin();
    while (kmin !== -1 && this.tag[start] !== TAG_CLOSED) kmin = this.processState();
    return this.tag[start] === TAG_CLOSED && this.costToGo[start] < OBSTACLE_COST;
  }

  /**
   * Algorithm 28 for every arc incident to a cell whose occupancy changed.
   * Both endpoints of a changed arc are re-inserted at their current h — a
   * legitimate D* operation (Alg. 31 does it in lines 27 and 29) that handles
   * cost *decreases* (a gate opening) as well as increases.
   */
  setOccupied(cell: number, occupied: boolean): void {
    if ((this.believed.occupied[cell] === 1) === occupied) return;
    this.believed.occupied[cell] = occupied ? 1 : 0;
    const touched = [cell, ...this.graph.latticeNeighbors(cell)];
    for (const X of touched) if (this.tag[X] === TAG_CLOSED) this.insert(X, this.costToGo[X]);
  }

  /**
   * Algorithm 25, lines 18–20: propagate until nothing on the open list could
   * still lower the robot's own cost-to-goal. Returns what was touched, for
   * the widget's count.
   */
  repair(Xc: number): RepairStats {
    const stats: RepairStats = { popped: [], raised: [], lowered: [], inserted: [] };
    this.recording = stats;
    let kmin = this.getKmin();
    while (kmin !== -1 && kmin < this.costToGo[Xc] - EPS) kmin = this.processState();
    this.recording = null;
    return stats;
  }

  /**
   * Alg. 25 lines 15–17 with a sensor: compare the believed occupancy of the
   * robot's neighbourhood with the truth, fix the map, and repair.
   */
  senseAndRepair(Xc: number, truth: OccupancyGrid, radius = 1): RepairStats {
    const [ci, cj] = [Xc % this.believed.nx, Math.floor(Xc / this.believed.nx)];
    let changed = false;
    for (let dj = -radius; dj <= radius; dj++) {
      for (let di = -radius; di <= radius; di++) {
        const i = ci + di;
        const j = cj + dj;
        if (i < 0 || j < 0 || i >= this.believed.nx || j >= this.believed.ny) continue;
        const Y = j * this.believed.nx + i;
        if (this.believed.occupied[Y] !== truth.occupied[Y]) {
          this.setOccupied(Y, truth.occupied[Y] === 1);
          changed = true;
        }
      }
    }
    if (!changed) return { popped: [], raised: [], lowered: [], inserted: [] };
    return this.repair(Xc);
  }

  /** Algorithm 26: follow back pointers from `from` to the goal. Null if broken. */
  backpointerPath(from: number): number[] | null {
    const out: number[] = [];
    let X = from;
    const guard = this.believed.nx * this.believed.ny + 1;
    for (let i = 0; i < guard; i++) {
      out.push(X);
      if (X === this.goal) return out;
      X = this.back[X];
      if (X < 0) return null;
    }
    return null;
  }

  /** Sum of c(X, Y) along a back-pointer path — ≥ 10000 means it goes through an obstacle. */
  pathCost(path: readonly number[]): number {
    let s = 0;
    for (let i = 1; i < path.length; i++) s += this.cost(path[i - 1], path[i]);
    return s;
  }

  /** Is X currently a RAISE state (k < h)? For the widget's orange outlines. */
  isRaised(X: number): boolean {
    return this.tag[X] === TAG_OPEN && this.kmin[X] < this.costToGo[X] - EPS;
  }
}

/* -------------------------------------------------------------------------- */
/* Choset's gate scenario, §H.3                                                */
/* -------------------------------------------------------------------------- */

export interface GateScenario {
  grid: OccupancyGrid;
  start: number;
  goal: number;
  gate: number;
}

/**
 * The 7 × 6 world of figures H.21–H.26 as far as the text determines it:
 * start (2,1), goal (7,6), the gate at (4,3), and the obstacle cells the text
 * names — (2,3), (3,3), (4,2) in figure H.18 and (3,5), (3,6) in figure H.22.
 * With these, the initial optimal path is the pure diagonal
 * (2,1) (3,2) (4,3) (5,4) (6,5) (7,6) of cost 7.0, h(4,1) = 6.2 as in the
 * text, and closing the gate repairs (3,2) to k = 7.6 via (4,1).
 */
export function chosetGateScenario(): GateScenario {
  const nx = 7, ny = 6;
  const occupied = new Uint8Array(nx * ny);
  const at = (col: number, row: number) => (row - 1) * nx + (col - 1);
  for (const [c, r] of [[2, 3], [3, 3], [4, 2], [3, 5], [3, 6]] as const) occupied[at(c, r)] = 1;
  const grid: OccupancyGrid = { nx, ny, cellSize: 1, origin: { x: 0, y: 0 }, occupied };
  return { grid, start: at(2, 1), goal: at(7, 6), gate: at(4, 3) };
}
