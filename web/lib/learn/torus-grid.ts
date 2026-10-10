/**
 * Reach's C-space raster as a Chapter 6 graph.
 *
 * Chapter 6's `GridGraph` lives on a planar occupancy grid; Reach's
 * configuration space is a torus, so the lattice here identifies opposite
 * edges. Nothing else changes: the same `astar` / `dijkstra` engines run on
 * it, and the octile distance with wrapped deltas is the exact cost-to-go on
 * an empty torus lattice — admissible and consistent, the Euclidean-style
 * baseline the learned heuristic is measured against.
 *
 * `costToGo` is the "backward Dijkstra" that labels the heuristic's training
 * data: every free cell's true h* to a goal cell, in one sweep.
 */

import type { Edge, Graph } from '../search/graph';
import { BestFirst, PRIORITY_DIJKSTRA } from '../search/best-first';
import type { BitGrid } from '../manifold/torus';
import { cellAngles, cellOf } from '../manifold/torus';

const TAU = 2 * Math.PI;

const STEPS8: readonly [number, number][] = [
  [1, 0], [-1, 0], [0, 1], [0, -1],
  [1, 1], [1, -1], [-1, 1], [-1, -1],
];

export class TorusGridGraph implements Graph<number> {
  /** Radians per cell along each axis (the raster is square). */
  readonly cellSize: number;
  readonly diagonal: number;

  constructor(readonly grid: BitGrid) {
    if (grid.n1 !== grid.n2) throw new Error('TorusGridGraph: the raster must be square');
    this.cellSize = TAU / grid.n1;
    this.diagonal = Math.SQRT2 * this.cellSize;
  }

  get n(): number {
    return this.grid.n1;
  }

  get size(): number {
    return this.grid.n1 * this.grid.n2;
  }

  isFree(cell: number): boolean {
    return this.grid.occupied[cell] === 0;
  }

  ij(cell: number): [number, number] {
    const i = cell % this.n;
    return [i, (cell - i) / this.n];
  }

  index(i: number, j: number): number {
    const n = this.n;
    return (((j % n) + n) % n) * n + (((i % n) + n) % n);
  }

  /** 8-connected with wraparound; a diagonal move may not cut a corner between two blocked cells. */
  neighbors(cell: number): Edge<number>[] {
    const [i, j] = this.ij(cell);
    const out: Edge<number>[] = [];
    for (const [di, dj] of STEPS8) {
      const nb = this.index(i + di, j + dj);
      if (!this.isFree(nb)) continue;
      if (di !== 0 && dj !== 0) {
        if (!this.isFree(this.index(i + di, j)) || !this.isFree(this.index(i, j + dj))) continue;
        out.push({ to: nb, cost: this.diagonal });
      } else out.push({ to: nb, cost: this.cellSize });
    }
    return out;
  }

  key(cell: number): number {
    return cell;
  }

  order(cell: number): number {
    return cell;
  }

  /** Chart angles of a cell's centre. */
  center(cell: number): [number, number] {
    const [i, j] = this.ij(cell);
    return cellAngles(this.grid, i, j);
  }

  /** The cell containing a configuration (wrapping). */
  cellAt(q: readonly number[]): number {
    const [i, j] = cellOf(this.grid, [q[0], q[1]]);
    return j * this.n + i;
  }

  /** Nearest free cell to q by expanding rings on the torus (q's own cell if free). */
  freeCellNear(q: readonly number[]): number {
    const c = this.cellAt(q);
    if (this.isFree(c)) return c;
    const [ci, cj] = this.ij(c);
    for (let r = 1; r <= this.n / 2; r++) {
      for (let di = -r; di <= r; di++) {
        for (let dj = -r; dj <= r; dj++) {
          if (Math.max(Math.abs(di), Math.abs(dj)) !== r) continue;
          const k = this.index(ci + di, cj + dj);
          if (this.isFree(k)) return k;
        }
      }
    }
    return -1;
  }
}

/** Wrapped |Δ| in cells along one axis of an n-periodic lattice. */
const wrapDelta = (a: number, b: number, n: number) => {
  const d = Math.abs(a - b) % n;
  return Math.min(d, n - d);
};

/**
 * Octile distance with wrapped deltas: the exact lattice cost-to-go on an
 * empty torus, hence a lower bound on any obstructed one (Choset App. H.2's
 * optimistic heuristic, D22.3 at ε = 0).
 */
export function torusOctile(g: TorusGridGraph, goal: number): (cell: number) => number {
  const [gi, gj] = g.ij(goal);
  return (cell) => {
    const [i, j] = g.ij(cell);
    const dx = wrapDelta(i, gi, g.n);
    const dy = wrapDelta(j, gj, g.n);
    return g.diagonal * Math.min(dx, dy) + g.cellSize * Math.abs(dx - dy);
  };
}

/**
 * h*(v) for every cell: Dijkstra from the goal run to exhaustion (the graph is
 * undirected, so cost-from-goal equals cost-to-goal). Unreachable cells and
 * obstacles read +∞.
 */
export function costToGo(g: TorusGridGraph, goal: number): Float64Array {
  const out = new Float64Array(g.size).fill(Infinity);
  if (!g.isFree(goal)) return out;
  // A goal key no cell has makes the engine sweep the whole component.
  const sweep = new BestFirst<number>(g, goal, -1, () => 0, PRIORITY_DIJKSTRA);
  sweep.run();
  for (const [k, v] of sweep.g) out[k as number] = v;
  return out;
}
