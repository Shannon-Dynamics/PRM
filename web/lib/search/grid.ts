/**
 * Grids as graphs — Choset App. H.1, figure H.4 — and the occupancy array a
 * grid is built on.
 *
 * Nodes are cells, indexed `j·nx + i` (row-major, i the column). Four-point
 * connectivity joins N/S/E/W at cost 1; eight-point adds the diagonals at
 * `diagonal`, which is 1.4 for Choset's grid metric (figure H.16, hand
 * arithmetic) or √2 for the exact one. Obstacle cells are *deleted* from the
 * graph rather than kept as 10000-cost nodes (§8 of the design doc; Choset's
 * convention is kept in `dstar.ts`, where his worked numbers need it).
 *
 * An occupancy grid comes from one of three places: an ASCII picture (the
 * hand-traced examples), a seeded random layout (the Search Theater), or the
 * Apartment rasterized at a cell size with the robot's radius as inflation —
 * the Chapter 4 reduction of a disc robot to a point, computed with the ported
 * exact distance transform.
 */

import { distanceAt, exactDistanceField } from '../mapping/edt';
import type { Rng } from '../prob/rng';
import type { Bounds, World } from '../sim/world';
import type { Edge, Graph } from './graph';

export interface OccupancyGrid {
  nx: number;
  ny: number;
  /** Metres per cell; 1 for the abstract examples. */
  cellSize: number;
  /** World coordinates of the grid's lower-left corner. */
  origin: { x: number; y: number };
  /** 1 = obstacle, 0 = free; row-major j·nx + i. */
  occupied: Uint8Array;
}

export const cellIndex = (g: OccupancyGrid, i: number, j: number) => j * g.nx + i;
export const cellIJ = (g: OccupancyGrid, cell: number): [number, number] => [cell % g.nx, Math.floor(cell / g.nx)];

/** Choset's (col, row) labels are 1-based; cell (1, 1) is the bottom-left corner. */
export const cellFromColRow = (g: OccupancyGrid, col: number, row: number) => cellIndex(g, col - 1, row - 1);
export const colRowOf = (g: OccupancyGrid, cell: number): [number, number] => {
  const [i, j] = cellIJ(g, cell);
  return [i + 1, j + 1];
};

export function cellCenter(g: OccupancyGrid, cell: number): { x: number; y: number } {
  const [i, j] = cellIJ(g, cell);
  return { x: g.origin.x + (i + 0.5) * g.cellSize, y: g.origin.y + (j + 0.5) * g.cellSize };
}

export function cellAt(g: OccupancyGrid, x: number, y: number): number {
  const i = Math.min(g.nx - 1, Math.max(0, Math.floor((x - g.origin.x) / g.cellSize)));
  const j = Math.min(g.ny - 1, Math.max(0, Math.floor((y - g.origin.y) / g.cellSize)));
  return cellIndex(g, i, j);
}

export const isFreeCell = (g: OccupancyGrid, cell: number) => cell >= 0 && cell < g.occupied.length && g.occupied[cell] === 0;

/** Nearest free cell to a world point, by expanding rings; the cell itself if free. */
export function freeCellNear(g: OccupancyGrid, x: number, y: number): number {
  const c = cellAt(g, x, y);
  if (isFreeCell(g, c)) return c;
  const [ci, cj] = cellIJ(g, c);
  for (let r = 1; r < Math.max(g.nx, g.ny); r++) {
    for (let dj = -r; dj <= r; dj++) {
      for (let di = -r; di <= r; di++) {
        if (Math.max(Math.abs(di), Math.abs(dj)) !== r) continue;
        const i = ci + di;
        const j = cj + dj;
        if (i < 0 || j < 0 || i >= g.nx || j >= g.ny) continue;
        const k = cellIndex(g, i, j);
        if (isFreeCell(g, k)) return k;
      }
    }
  }
  return c;
}

export function cloneGrid(g: OccupancyGrid): OccupancyGrid {
  return { ...g, occupied: Uint8Array.from(g.occupied) };
}

/**
 * Build a grid from an ASCII picture. The *first* string is the top row, so
 * the picture reads like a map; `#` is an obstacle, anything else is free.
 */
export function gridFromAscii(rows: readonly string[], cellSize = 1): OccupancyGrid {
  const ny = rows.length;
  const nx = Math.max(...rows.map((r) => r.length));
  const occupied = new Uint8Array(nx * ny);
  rows.forEach((row, fromTop) => {
    const j = ny - 1 - fromTop;
    for (let i = 0; i < row.length; i++) if (row[i] === '#') occupied[j * nx + i] = 1;
  });
  return { nx, ny, cellSize, origin: { x: 0, y: 0 }, occupied };
}

/**
 * The design doc's 4 × 3 micro-example: cells (col, row), obstacles at (2, 1)
 * and (2, 2), start (1, 1), goal (4, 1). Five A* expansions at cost 5.2; ten
 * for Dijkstra.
 */
export const MICRO_GRID = gridFromAscii([
  '....', // row 3
  '.#..', // row 2
  '.#..', // row 1
]);
export const MICRO_START = cellFromColRow(MICRO_GRID, 1, 1);
export const MICRO_GOAL = cellFromColRow(MICRO_GRID, 4, 1);

/**
 * Rasterize a polyline world for a disc robot of radius `inflation`: a cell is
 * occupied when its *centre* lies within `inflation` of a wall. The exact
 * Euclidean distance transform (ported from the sister book) does the whole
 * grid in one pass at a fine resolution, so the raster is the Minkowski
 * inflation of Chapter 4 sampled at cell centres — not an approximation of it.
 *
 * Sampling at centres, rather than taking the worst distance anywhere in the
 * cell, is deliberate: a 0.9 m doorway at 0.5 m cells has a centre 0.25 m from
 * the jamb, which a 0.22 m robot passes; the worst-case rule would wall every
 * door in the Apartment shut at that resolution. The cost is that a coarse
 * cell can straddle a thin wall and still read free — which is why the grid
 * graph offers `cornerCutting: false` and why Chapter 7 refines the cells.
 */
export function gridFromWorld(world: World, cellSize: number, inflation: number, bounds: Bounds = world.bounds): OccupancyGrid {
  const fine = Math.min(cellSize / 4, 0.05);
  const field = exactDistanceField(world, fine, bounds);
  const nx = Math.max(1, Math.round((bounds.maxX - bounds.minX) / cellSize));
  const ny = Math.max(1, Math.round((bounds.maxY - bounds.minY) / cellSize));
  const occupied = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++) {
    for (let i = 0; i < nx; i++) {
      const cx = bounds.minX + (i + 0.5) * cellSize;
      const cy = bounds.minY + (j + 0.5) * cellSize;
      if (distanceAt(field, cx, cy) < inflation) occupied[j * nx + i] = 1;
    }
  }
  return { nx, ny, cellSize, origin: { x: bounds.minX, y: bounds.minY }, occupied };
}

/** A seeded random layout: each cell is an obstacle with probability `density`, start and goal kept free. */
export function randomGrid(nx: number, ny: number, density: number, rng: Rng, keepFree: readonly number[] = []): OccupancyGrid {
  const occupied = new Uint8Array(nx * ny);
  for (let k = 0; k < nx * ny; k++) if (rng.next() < density) occupied[k] = 1;
  for (const c of keepFree) occupied[c] = 0;
  return { nx, ny, cellSize: 1, origin: { x: 0, y: 0 }, occupied };
}

/* -------------------------------------------------------------------------- */
/* The grid as a Graph                                                         */
/* -------------------------------------------------------------------------- */

export type Connectivity = 4 | 8;

export interface GridGraphOptions {
  connectivity?: Connectivity;
  /** Cost of a diagonal step: 1.4 (Choset, figure H.16) or Math.SQRT2. */
  diagonal?: number;
  /**
   * May a diagonal step squeeze between two orthogonally adjacent obstacles?
   * Choset's §H.2.4 allows it ((3, 2) is adjacent to (4, 3) with (3, 3) and
   * (4, 2) blocked); a disc robot on a real map should not. Default: allowed,
   * so the hand examples reproduce.
   */
  cornerCutting?: boolean;
}

/** Choset's approximation of √2, which makes the hand arithmetic come out in tenths. */
export const CHOSET_DIAGONAL = 1.4;

const STEPS8: readonly [number, number][] = [
  [1, 0], [-1, 0], [0, 1], [0, -1],
  [1, 1], [1, -1], [-1, 1], [-1, -1],
];

export class GridGraph implements Graph<number> {
  readonly connectivity: Connectivity;
  readonly diagonal: number;
  readonly cornerCutting: boolean;

  constructor(
    readonly grid: OccupancyGrid,
    opts: GridGraphOptions = {},
  ) {
    this.connectivity = opts.connectivity ?? 8;
    this.diagonal = opts.diagonal ?? CHOSET_DIAGONAL;
    this.cornerCutting = opts.cornerCutting ?? true;
  }

  neighbors(cell: number): Edge<number>[] {
    const g = this.grid;
    const i = cell % g.nx;
    const j = (cell - i) / g.nx;
    const out: Edge<number>[] = [];
    const n = this.connectivity === 8 ? 8 : 4;
    for (let s = 0; s < n; s++) {
      const [di, dj] = STEPS8[s];
      const ni = i + di;
      const nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= g.nx || nj >= g.ny) continue;
      const nb = nj * g.nx + ni;
      if (g.occupied[nb]) continue;
      const diag = di !== 0 && dj !== 0;
      if (diag && !this.cornerCutting && (g.occupied[j * g.nx + ni] || g.occupied[nj * g.nx + i])) continue;
      out.push({ to: nb, cost: (diag ? this.diagonal : 1) * g.cellSize });
    }
    return out;
  }

  key(cell: number): number {
    return cell;
  }

  /** Lexicographic (col, row) order — the design doc's final tie-break. */
  order(cell: number): number {
    const i = cell % this.grid.nx;
    const j = (cell - i) / this.grid.nx;
    return i * this.grid.ny + j;
  }

  /** Step cost between two cells assumed adjacent and free — for D* Lite's edge costs. */
  stepCost(a: number, b: number): number {
    const [ai, aj] = cellIJ(this.grid, a);
    const [bi, bj] = cellIJ(this.grid, b);
    const diag = ai !== bi && aj !== bj;
    return (diag ? this.diagonal : 1) * this.grid.cellSize;
  }

  /** All lattice neighbours regardless of occupancy — D* and D* Lite reason about blocked arcs too. */
  latticeNeighbors(cell: number): number[] {
    const g = this.grid;
    const i = cell % g.nx;
    const j = (cell - i) / g.nx;
    const out: number[] = [];
    const n = this.connectivity === 8 ? 8 : 4;
    for (let s = 0; s < n; s++) {
      const [di, dj] = STEPS8[s];
      const ni = i + di;
      const nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= g.nx || nj >= g.ny) continue;
      out.push(nj * g.nx + ni);
    }
    return out;
  }
}

/* -------------------------------------------------------------------------- */
/* Heuristics                                                                  */
/* -------------------------------------------------------------------------- */

export type Heuristic<N> = (n: N) => number;

export type HeuristicKind = 'zero' | 'manhattan' | 'euclidean' | 'octile' | 'chebyshev';

/**
 * Heuristics to a fixed goal cell, in grid units × cellSize.
 *
 *   octile    — diag·min(Δx, Δy) + |Δx − Δy|: the exact cost-to-go on an empty
 *               8-connected grid with that diagonal cost, hence admissible and
 *               consistent (Exercise 1 asks for the proof — and for the case
 *               where diag = 1.4 and the true metric is √2, where it is not).
 *   manhattan — exact on an empty 4-connected grid; *inadmissible* with diagonals.
 *   euclidean — admissible when diag ≥ √2; with Choset's 1.4 it very slightly
 *               is not (two diagonals cost 2.8 < 2√2 = 2.83). Octile with the
 *               grid's own diagonal is the honest default.
 *   chebyshev — max(Δx, Δy): admissible for any diag ≥ 1.
 */
export function gridHeuristic(grid: OccupancyGrid, goal: number, kind: HeuristicKind, diagonal = CHOSET_DIAGONAL): Heuristic<number> {
  const [gi, gj] = cellIJ(grid, goal);
  const s = grid.cellSize;
  return (cell: number) => {
    const i = cell % grid.nx;
    const j = (cell - i) / grid.nx;
    const dx = Math.abs(i - gi);
    const dy = Math.abs(j - gj);
    switch (kind) {
      case 'zero':
        return 0;
      case 'manhattan':
        return (dx + dy) * s;
      case 'euclidean':
        return Math.hypot(dx, dy) * s;
      case 'chebyshev':
        return Math.max(dx, dy) * s;
      case 'octile':
      default:
        return (diagonal * Math.min(dx, dy) + Math.abs(dx - dy)) * s;
    }
  };
}

/** Choset's heuristic in §H.2.4 and §H.3: the 1/1.4 octile distance. */
export const chosetHeuristic = (grid: OccupancyGrid, goal: number) => gridHeuristic(grid, goal, 'octile', CHOSET_DIAGONAL);
