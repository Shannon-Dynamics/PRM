/**
 * Brushfire and wave-front — Choset §4.3.2 and §4.5 — THE OWNED grid distance
 * transform of Chapter 7, imported by Chapter 8 (grid GVD) and Chapter 10.
 *
 * Both are breadth-first search on the Chapter 6 grid graph with a different
 * seed set (Derivation 4):
 *
 *   brushfire   every obstacle pixel is a source at depth 0; a free pixel's
 *               label is 1 + its link length to the nearest obstacle — the
 *               Manhattan distance under 4-point connectivity, the chessboard
 *               distance under 8-point. Each pixel keeps a *back pointer* to the
 *               pixel that labelled it and the *source* obstacle it descends
 *               from; a pixel two fronts reach from different obstacles keeps
 *               both pointers — Choset §5.2.5's grid GVD, Chapter 8's ridge.
 *   wave-front  the goal pixel is the one source, seeded with 2; a pixel's label
 *               is 2 + its link length to the goal, and "point to a lowest
 *               neighbor" from any start descends to the goal along a grid-
 *               shortest path, because every non-goal pixel has a neighbor
 *               labelled exactly one less.
 *
 * The queue is a plain FIFO array over `GridGraph.neighbors()` — the same
 * adjacency the Chapter 6 searches use, so a 4- or 8-connected brushfire is the
 * same code with the graph's connectivity switched. Choset's labelling starts
 * at 1 for obstacles, 2 for their neighbors; we keep his numbers exactly so
 * the 7 × 3 micro-example reads off the grid as printed.
 */

import { GridGraph, cellIJ, gridFromAscii, type Connectivity, type OccupancyGrid } from '../search/grid';

export type Conn = Connectivity;

export interface DistGrid {
  grid: OccupancyGrid;
  conn: Conn;
  /** Choset's labels: obstacle 1, free pixels 2, 3, …; 0 = never reached. For wave-front, goal = 2. */
  label: Int32Array;
  /** The pixel that labelled this one (−1 for sources). Points toward the nearest obstacle / the goal. */
  back: Int32Array;
  /** A second back pointer from a *different* source, when two fronts reached the pixel together; −1 otherwise. */
  back2: Int32Array;
  /** Source id: the obstacle component (brushfire) or 0 (wave-front) this pixel's front came from; −1 if unreached. */
  source: Int32Array;
  /** Pixels where fronts from different obstacles collide (Choset §5.2.5), including the odd-gap case where the collision falls between two pixels. */
  ridge: Uint8Array;
  maxLabel: number;
  /** Number of distinct obstacle components (brushfire) — 1 for wave-front. */
  nSources: number;
}

/* -------------------------------------------------------------------------- */
/* Obstacle components                                                         */
/* -------------------------------------------------------------------------- */

/**
 * 8-connected components of the occupied pixels. A front's *source* is a
 * component, not a pixel: two fronts from the same wall meeting inside its own
 * concavity are not a Voronoi edge, and Choset's "two different back pointers"
 * means two different obstacles.
 */
export function labelObstacleComponents(grid: OccupancyGrid): { count: number; comp: Int32Array } {
  const { nx, ny, occupied } = grid;
  const comp = new Int32Array(nx * ny).fill(-1);
  const stack: number[] = [];
  let count = 0;
  for (let s = 0; s < occupied.length; s++) {
    if (!occupied[s] || comp[s] >= 0) continue;
    comp[s] = count;
    stack.push(s);
    while (stack.length) {
      const k = stack.pop()!;
      const i = k % nx;
      const j = (k - i) / nx;
      for (let dj = -1; dj <= 1; dj++) {
        for (let di = -1; di <= 1; di++) {
          if (!di && !dj) continue;
          const ii = i + di;
          const jj = j + dj;
          if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
          const t = jj * nx + ii;
          if (occupied[t] && comp[t] < 0) {
            comp[t] = count;
            stack.push(t);
          }
        }
      }
    }
    count++;
  }
  return { count, comp };
}

/* -------------------------------------------------------------------------- */
/* The multi-source BFS both planners share                                    */
/* -------------------------------------------------------------------------- */

function propagate(
  graph: GridGraph,
  seeds: readonly { cell: number; label: number; source: number }[],
  nSources: number,
  markRidges: boolean,
): DistGrid {
  const g = graph.grid;
  const n = g.nx * g.ny;
  const label = new Int32Array(n);
  const back = new Int32Array(n).fill(-1);
  const back2 = new Int32Array(n).fill(-1);
  const source = new Int32Array(n).fill(-1);
  const ridge = new Uint8Array(n);
  // Obstacle pixels carry Choset's label 1 and never enter the queue as free pixels.
  for (let k = 0; k < n; k++) if (g.occupied[k]) label[k] = 1;

  const queue: number[] = [];
  for (const s of seeds) {
    if (label[s.cell] !== 0 && !g.occupied[s.cell]) continue;
    label[s.cell] = s.label;
    source[s.cell] = s.source;
    queue.push(s.cell);
  }
  let head = 0;
  let maxLabel = 0;
  while (head < queue.length) {
    const c = queue[head++];
    const lc = label[c];
    const sc = source[c];
    if (lc > maxLabel) maxLabel = lc;
    // Star(c) restricted to free pixels — `GridGraph` already deletes occupied cells from the graph.
    for (const { to } of graph.neighbors(c)) {
      if (label[to] === 0) {
        label[to] = lc + 1;
        back[to] = c;
        source[to] = sc;
        queue.push(to);
      } else if (markRidges && source[to] !== sc && source[to] >= 0 && label[to] >= lc) {
        // Two fronts meet. Even gap: the pixel was already labelled lc + 1 by the
        // other front — it gets its second back pointer (Choset's rule). Odd gap:
        // both pixels carry label lc and the collision falls on their shared edge;
        // we flag both so the grid GVD has no holes where the exact one is continuous.
        if (label[to] === lc + 1 && back2[to] < 0) back2[to] = c;
        ridge[to] = 1;
        if (label[to] === lc) ridge[c] = 1;
      }
    }
  }
  return { grid: g, conn: graph.connectivity, label, back, back2, source, ridge, maxLabel, nSources };
}

/**
 * Brushfire (Choset §4.3.2): label every free pixel with 1 + its link length to
 * the nearest obstacle pixel. Step 1 of the text — "all zero-valued pixels
 * neighboring one-valued pixels are labelled 2" — is the seeding below; the
 * rest is the BFS.
 */
export function brushfire(grid: OccupancyGrid, conn: Conn = 4): DistGrid {
  const graph = new GridGraph(grid, { connectivity: conn, cornerCutting: true });
  const { count, comp } = labelObstacleComponents(grid);
  const seeds: { cell: number; label: number; source: number }[] = [];
  const seen = new Uint8Array(grid.nx * grid.ny);
  for (let k = 0; k < grid.occupied.length; k++) {
    if (!grid.occupied[k]) continue;
    for (const nb of graph.latticeNeighbors(k)) {
      if (grid.occupied[nb] || seen[nb]) continue;
      seen[nb] = 1;
      seeds.push({ cell: nb, label: 2, source: comp[k] });
    }
  }
  // A seed adjacent to two obstacles is itself a collision pixel; record the second source.
  const dist = propagate(graph, seeds, count, true);
  for (let k = 0; k < grid.occupied.length; k++) {
    if (!grid.occupied[k]) continue;
    for (const nb of graph.latticeNeighbors(k)) {
      if (grid.occupied[nb]) continue;
      if (dist.source[nb] !== comp[k]) {
        dist.ridge[nb] = 1;
        if (dist.back2[nb] < 0) dist.back2[nb] = k;
      }
    }
  }
  // Sources' own back pointers: the obstacle pixel that seeded them.
  for (let k = 0; k < grid.occupied.length; k++) {
    if (!grid.occupied[k]) continue;
    for (const nb of graph.latticeNeighbors(k)) {
      if (!grid.occupied[nb] && dist.label[nb] === 2 && dist.back[nb] < 0) dist.back[nb] = k;
    }
  }
  return dist;
}

/**
 * Wave-front (Choset §4.5): the goal pixel is labelled 2 and the front grows
 * from it; a pixel's label is 2 + its grid distance to the goal. Pixels the
 * front never reaches (walled off) stay 0.
 */
export function wavefront(grid: OccupancyGrid, goal: number, conn: Conn = 4): DistGrid {
  if (grid.occupied[goal]) throw new Error('wavefront: the goal pixel is an obstacle');
  const graph = new GridGraph(grid, { connectivity: conn, cornerCutting: true });
  return propagate(graph, [{ cell: goal, label: 2, source: 0 }], 1, false);
}

/* -------------------------------------------------------------------------- */
/* Reading the grid                                                            */
/* -------------------------------------------------------------------------- */

/** (label − 1)·cellSize: the grid distance in metres; `Infinity` for unreached pixels. */
export function gridDistance(dist: DistGrid, cell: number): number {
  const l = dist.label[cell];
  return l <= 0 ? Infinity : (l - 1) * dist.grid.cellSize;
}

/** Pixels with two back pointers — Choset §5.2.5's literal rule. */
export function doubleBackPointerCells(dist: DistGrid): number[] {
  const out: number[] = [];
  for (let k = 0; k < dist.back2.length; k++) if (dist.back2[k] >= 0 && !dist.grid.occupied[k]) out.push(k);
  return out;
}

/** Pixels where fronts from different obstacles collide — the grid GVD (Chapter 8). */
export function ridgeCells(dist: DistGrid): number[] {
  const out: number[] = [];
  for (let k = 0; k < dist.ridge.length; k++) if (dist.ridge[k] && !dist.grid.occupied[k]) out.push(k);
  return out;
}

/**
 * "The gradient of distance at a pixel is determined by finding a neighbor with
 * the lowest pixel value" (Choset §4.3.2). Ties go to the first neighbor in the
 * graph's scan order E, W, N, S (then the diagonals), which is what makes the
 * micro-example's "(6,3) or (5,2) — first in scan order" a definite answer.
 * Returns the neighbor cell, or −1 at a source or an isolated pixel.
 */
export function gridGradient(dist: DistGrid, cell: number): number {
  const graph = new GridGraph(dist.grid, { connectivity: dist.conn, cornerCutting: true });
  let best = -1;
  let bestLabel = Infinity;
  for (const { to } of graph.neighbors(cell)) {
    const l = dist.label[to];
    if (l > 0 && l < bestLabel) {
      bestLabel = l;
      best = to;
    }
  }
  return bestLabel < dist.label[cell] ? best : -1;
}

/**
 * Wave-front descent: from `start`, step to a neighbor labelled one less until
 * the goal (label 2) is reached. Returns the cell path, or [] if the start was
 * never reached by the front. Choset's guarantee — "there will always be a
 * neighboring pixel whose value is one less" — is what the loop relies on, and
 * the chapter's check asserts the path has exactly label(start) − 2 steps.
 */
export function descendWavefront(dist: DistGrid, start: number): number[] {
  if (dist.label[start] < 2) return [];
  const path = [start];
  let cur = start;
  const guard = dist.label.length + 1;
  for (let i = 0; i < guard && dist.label[cur] > 2; i++) {
    const next = gridGradient(dist, cur);
    if (next < 0) return path;
    cur = next;
    path.push(cur);
  }
  return path;
}

/** Labels as rows, top row first, for printing — the design doc's three lines. */
export function labelRows(dist: DistGrid): number[][] {
  const { nx, ny } = dist.grid;
  const rows: number[][] = [];
  for (let j = ny - 1; j >= 0; j--) {
    const row: number[] = [];
    for (let i = 0; i < nx; i++) row.push(dist.label[j * nx + i]);
    rows.push(row);
  }
  return rows;
}

/** Choset-style (col, row), 1-based, bottom-left origin. */
export function colRow(dist: DistGrid, cell: number): [number, number] {
  const [i, j] = cellIJ(dist.grid, cell);
  return [i + 1, j + 1];
}

/**
 * The design doc's 7 × 3 micro-grid: obstacles at (1, 2) and (7, 2). With
 * 4-point connectivity the labels are
 *
 *     row 3:  2 3 4 5 4 3 2
 *     row 2:  1 2 3 4 3 2 1
 *     row 1:  2 3 4 5 4 3 2
 *
 * and column 4, equidistant from both obstacles, carries two back pointers.
 */
export const MICRO_7X3: OccupancyGrid = gridFromAscii([
  '.......', // row 3
  '#.....#', // row 2
  '.......', // row 1
]);
