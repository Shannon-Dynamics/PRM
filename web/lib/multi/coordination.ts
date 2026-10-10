/**
 * Decoupled planning's second stage — Choset §7.5.2, after Kant & Zucker
 * [C ref. 219]: the paths are fixed, only the *timing* is planned.
 *
 * Two robots with individual paths c₁, c₂ : [0, 1] → 𝒬_free,i. The coordination
 * space is the unit square of path parameters (s₁, s₂); its obstacle is the set
 * of parameter pairs at which the two bodies overlap. A velocity schedule is a
 * monotone path from (0, 0) to (1, 1) — monotone because neither robot drives
 * backward along its path — and it exists iff the obstacle does not separate
 * the two corners. Finding it is Chapter 6's grid search on the rasterized
 * square with three moves: advance robot 1, advance robot 2, advance both.
 *
 * Derivation 2's instance shows what the diagram cannot do. When both paths are
 * the corridor axis and the robots must swap ends, the obstacle is the full
 * anti-diagonal band s₁ + s₂ ≈ 1 and no monotone path crosses it. No wait,
 * however long, fixes that; the detour into the alcove changes the *path*, and
 * the diagram has no coordinate for it.
 */

import { astar } from '../search/best-first';
import type { Edge, Graph } from '../search/graph';
import { discsCollide } from './composite';

/** Arc-length parametrization of a polyline: the point at s ∈ [0, 1]. */
export function pointAt(path: readonly number[][], s: number): number[] {
  if (path.length === 1) return path[0].slice();
  const lens: number[] = [0];
  for (let i = 1; i < path.length; i++) lens.push(lens[i - 1] + Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]));
  const L = lens[lens.length - 1];
  if (L === 0) return path[0].slice();
  const target = Math.max(0, Math.min(1, s)) * L;
  let i = 1;
  while (i < path.length - 1 && lens[i] < target) i++;
  const seg = lens[i] - lens[i - 1];
  const u = seg > 0 ? (target - lens[i - 1]) / seg : 0;
  return [path[i - 1][0] + u * (path[i][0] - path[i - 1][0]), path[i - 1][1] + u * (path[i][1] - path[i - 1][1])];
}

export function polylineLength(path: readonly number[][]): number {
  let L = 0;
  for (let i = 1; i < path.length; i++) L += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]);
  return L;
}

export interface CoordinationDiagram {
  /** Cells per axis. */
  n: number;
  /** 1 where the two discs overlap, row-major j·n + i with i ↔ s₁, j ↔ s₂. */
  occupied: Uint8Array;
  /** Share of the square that is obstacle. */
  fraction: number;
}

/** Rasterize the coordination obstacle for two disc robots on fixed paths. */
export function coordinationDiagram(pathA: readonly number[][], pathB: readonly number[][], rA: number, rB: number, n = 64): CoordinationDiagram {
  const occupied = new Uint8Array(n * n);
  const ptsA: number[][] = [];
  const ptsB: number[][] = [];
  for (let k = 0; k < n; k++) {
    ptsA.push(pointAt(pathA, (k + 0.5) / n));
    ptsB.push(pointAt(pathB, (k + 0.5) / n));
  }
  let count = 0;
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      if (discsCollide(ptsA[i], rA, ptsB[j], rB).collide) {
        occupied[j * n + i] = 1;
        count++;
      }
    }
  }
  return { n, occupied, fraction: count / (n * n) };
}

/** The three monotone moves of a velocity schedule. */
class MonotoneGrid implements Graph<number> {
  constructor(readonly d: CoordinationDiagram) {}

  neighbors(cell: number): Edge<number>[] {
    const n = this.d.n;
    const i = cell % n;
    const j = (cell - i) / n;
    const out: Edge<number>[] = [];
    const tryCell = (ii: number, jj: number, cost: number) => {
      if (ii >= n || jj >= n) return;
      const c = jj * n + ii;
      if (this.d.occupied[c]) return;
      out.push({ to: c, cost });
    };
    tryCell(i + 1, j, 1);
    tryCell(i, j + 1, 1);
    // Advancing both is the diagonal; it is the cheapest in makespan, so weight it like a unit step.
    tryCell(i + 1, j + 1, 1);
    return out;
  }

  key(cell: number): number {
    return cell;
  }
}

export interface Schedule {
  /** Cells (i, j) from (0, 0) to (n − 1, n − 1). */
  cells: [number, number][];
  /** (s₁, s₂) pairs at cell centers, for executing the two robots in lockstep. */
  params: [number, number][];
  expansions: number;
}

/**
 * A monotone free path through the diagram, by A* with the Chebyshev
 * heuristic (exact for the three moves on an empty square). `null` when the
 * obstacle separates (0, 0) from (1, 1): the paths cannot be coordinated by
 * any choice of velocities — Choset's "it may be impossible to coordinate some
 * of the paths generated during the first stage".
 */
export function monotoneSchedule(d: CoordinationDiagram): Schedule | null {
  const n = d.n;
  const start = 0;
  const goal = n * n - 1;
  if (d.occupied[start] || d.occupied[goal]) return null;
  const h = (cell: number) => {
    const i = cell % n;
    const j = (cell - i) / n;
    return Math.max(n - 1 - i, n - 1 - j);
  };
  const t = astar(new MonotoneGrid(d), start, goal, h);
  if (t.status !== 'found') return null;
  const cells = t.path.map((c): [number, number] => [c % n, Math.floor(c / n)]);
  return { cells, params: cells.map(([i, j]) => [(i + 0.5) / n, (j + 0.5) / n]), expansions: t.expansions };
}

/**
 * The two robots executed under a schedule: positions at K equally spaced
 * instants, each robot moving along its own path at the speed the schedule
 * dictates (zero while the other advances alone).
 */
export function executeSchedule(pathA: readonly number[][], pathB: readonly number[][], sched: Schedule, K: number): { a: number[]; b: number[] }[] {
  const out: { a: number[]; b: number[] }[] = [];
  const m = sched.params.length;
  for (let k = 0; k < K; k++) {
    const u = (k / Math.max(1, K - 1)) * (m - 1);
    const i0 = Math.floor(u);
    const i1 = Math.min(m - 1, i0 + 1);
    const f = u - i0;
    const s1 = sched.params[i0][0] + f * (sched.params[i1][0] - sched.params[i0][0]);
    const s2 = sched.params[i0][1] + f * (sched.params[i1][1] - sched.params[i0][1]);
    out.push({ a: pointAt(pathA, s1), b: pointAt(pathB, s2) });
  }
  // Snap the ends onto the true endpoints (cell centers stop half a cell short).
  if (out.length) {
    out[0] = { a: pointAt(pathA, 0), b: pointAt(pathB, 0) };
    out[out.length - 1] = { a: pointAt(pathA, 1), b: pointAt(pathB, 1) };
  }
  return out;
}
