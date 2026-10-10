/**
 * The C-obstacle rasterizer — Choset §3.2.2, the "pixel method" behind
 * figures 3.6–3.8, made generic over any `Collision<Q>` and any 2-D chart of
 * Q. For Reach the chart is the unwrapped torus [−π, π)², for a disc Rusty it
 * is a window of the plane, and the same loop produces both pictures.
 *
 * Two honesty items ride along, as code:
 *
 *   - A free cell is a free *point*, not a free square (Choset b. 46–47). The
 *     grid certifies nothing about the curve between two free cells unless the
 *     footprint was thickened enough for a free sample to imply a free
 *     neighbourhood; the caller thickens by choosing the checker (a capsule
 *     radius for Reach, a larger disc for Rusty). Even then the planner is only
 *     resolution complete.
 *   - A raster is expensive: 1° on T² is 129,600 collision queries. `RasterJob`
 *     evaluates row by row under a per-call cell budget so a widget can keep
 *     drawing while the picture fills in, and the result is identical to the
 *     one-shot `rasterizeCobstacle`.
 */

import { normalizeAngle } from '../geom/se2';
import type { Collision, R2 } from '../collide/collision';
import type { Bounds } from '../sim/world';
import type { T2 } from '../robots/reach';

/** A uniform 2-D chart: n1 × n2 cell centres, with a configuration at each. */
export interface Chart2<Q> {
  n1: number;
  n2: number;
  /** Which axes wrap (both for a torus, neither for a plane window). */
  periodic: readonly [boolean, boolean];
  /** The configuration at cell centre (i, j). */
  point(i: number, j: number): Q;
  /** Chart coordinate of row i / column j (for drawing axes). */
  axis1(i: number): number;
  axis2(j: number): number;
  /** Cell containing a chart coordinate pair, for marking a configuration on the picture. */
  cellOf(u: number, v: number): [number, number];
  /** Cell edge lengths in chart units. */
  readonly d1: number;
  readonly d2: number;
  /** Chart extents. */
  readonly min1: number;
  readonly min2: number;
}

/**
 * The unwrapped torus with cells *centred* at −π + i·2π/n. With n = 360 the
 * centres sit at integer degrees, which is what makes the micro-example's
 * "29 cells in the θ₁ = 0° row" a statement about this grid and no other.
 */
export function torusChart(n1: number, n2 = n1): Chart2<T2> {
  const d1 = (2 * Math.PI) / n1;
  const d2 = (2 * Math.PI) / n2;
  const wrap = (k: number, n: number) => ((k % n) + n) % n;
  return {
    n1,
    n2,
    periodic: [true, true],
    d1,
    d2,
    min1: -Math.PI,
    min2: -Math.PI,
    point: (i, j) => [-Math.PI + i * d1, -Math.PI + j * d2],
    axis1: (i) => -Math.PI + i * d1,
    axis2: (j) => -Math.PI + j * d2,
    cellOf: (u, v) => [wrap(Math.round((normalizeAngle(u) + Math.PI) / d1), n1), wrap(Math.round((normalizeAngle(v) + Math.PI) / d2), n2)],
  };
}

/** A window of the plane, cells centred on a uniform nx × ny lattice over `bounds`. */
export function planeChart(bounds: Bounds, nx: number, ny: number): Chart2<R2> {
  const d1 = (bounds.maxX - bounds.minX) / nx;
  const d2 = (bounds.maxY - bounds.minY) / ny;
  const clamp = (k: number, n: number) => Math.max(0, Math.min(n - 1, k));
  return {
    n1: nx,
    n2: ny,
    periodic: [false, false],
    d1,
    d2,
    min1: bounds.minX,
    min2: bounds.minY,
    point: (i, j) => ({ x: bounds.minX + (i + 0.5) * d1, y: bounds.minY + (j + 0.5) * d2 }),
    axis1: (i) => bounds.minX + (i + 0.5) * d1,
    axis2: (j) => bounds.minY + (j + 0.5) * d2,
    cellOf: (u, v) => [clamp(Math.floor((u - bounds.minX) / d1), nx), clamp(Math.floor((v - bounds.minY) / d2), ny)],
  };
}

/** An n1 × n2 bit grid: 1 = the cell's configuration is in collision. Row-major by i. */
export class BitGrid {
  readonly bits: Uint8Array;

  constructor(
    readonly n1: number,
    readonly n2: number,
    bits?: Uint8Array,
  ) {
    this.bits = bits ?? new Uint8Array(n1 * n2);
  }

  get(i: number, j: number): boolean {
    return this.bits[i * this.n2 + j] === 1;
  }

  set(i: number, j: number, occupied: boolean): void {
    this.bits[i * this.n2 + j] = occupied ? 1 : 0;
  }

  /** Occupied cells in row i — the slice of QO at a fixed first coordinate. */
  rowCount(i: number): number {
    let c = 0;
    const base = i * this.n2;
    for (let j = 0; j < this.n2; j++) c += this.bits[base + j];
    return c;
  }

  /** Total occupied cells. */
  count(): number {
    let c = 0;
    for (let k = 0; k < this.bits.length; k++) c += this.bits[k];
    return c;
  }

  clone(): BitGrid {
    return new BitGrid(this.n1, this.n2, new Uint8Array(this.bits));
  }

  /**
   * 4-connected components of the cells equal to `occupied`, wrapping across
   * the seams the chart says are periodic. Returns the count and a label per
   * cell (−1 for cells of the other kind). On the torus this is the
   * connectivity the chapter *measures* before Chapter 5 states it: two
   * configurations in different free components cannot be joined by any path.
   */
  components(occupied: boolean, periodic: readonly [boolean, boolean]): { count: number; labels: Int32Array } {
    const { n1, n2, bits } = this;
    const want = occupied ? 1 : 0;
    const labels = new Int32Array(n1 * n2).fill(-1);
    const stack: number[] = [];
    let count = 0;
    for (let s = 0; s < bits.length; s++) {
      if (bits[s] !== want || labels[s] >= 0) continue;
      labels[s] = count;
      stack.push(s);
      while (stack.length) {
        const k = stack.pop()!;
        const i = Math.floor(k / n2);
        const j = k - i * n2;
        const visit = (ii: number, jj: number) => {
          if (ii < 0 || ii >= n1) {
            if (!periodic[0]) return;
            ii = (ii + n1) % n1;
          }
          if (jj < 0 || jj >= n2) {
            if (!periodic[1]) return;
            jj = (jj + n2) % n2;
          }
          const t = ii * n2 + jj;
          if (bits[t] === want && labels[t] < 0) {
            labels[t] = count;
            stack.push(t);
          }
        };
        visit(i - 1, j);
        visit(i + 1, j);
        visit(i, j - 1);
        visit(i, j + 1);
      }
      count++;
    }
    return { count, labels };
  }

  /** Does any occupied cell sit on the i = 0 or j = 0 seam? (A blob that "wraps" must.) */
  touchesSeam(): boolean {
    for (let j = 0; j < this.n2; j++) if (this.get(0, j)) return true;
    for (let i = 0; i < this.n1; i++) if (this.get(i, 0)) return true;
    return false;
  }
}

/** Choset's pixel method in one shot: one `isFree` query per cell centre. */
export function rasterizeCobstacle<Q>(checker: Collision<Q>, chart: Chart2<Q>): BitGrid {
  const job = new RasterJob(checker, chart);
  job.advance(Infinity);
  return job.grid;
}

/**
 * The same raster, evaluated incrementally. `advance(budget)` fills at most
 * `budget` cells and returns how many it did; `done` flips when the last row
 * is in. Rows are filled in order so a partially complete picture reads as a
 * sweep from θ₁ = −π upward.
 */
export class RasterJob<Q> {
  readonly grid: BitGrid;
  private i = 0;
  private j = 0;

  constructor(
    readonly checker: Collision<Q>,
    readonly chart: Chart2<Q>,
  ) {
    this.grid = new BitGrid(chart.n1, chart.n2);
  }

  get done(): boolean {
    return this.i >= this.chart.n1;
  }

  /** Fraction of cells evaluated so far. */
  get progress(): number {
    return Math.min(1, (this.i * this.chart.n2 + this.j) / (this.chart.n1 * this.chart.n2));
  }

  /** Rows completed so far. */
  get rowsDone(): number {
    return this.i;
  }

  advance(budget: number): number {
    let n = 0;
    const { n1, n2 } = this.chart;
    while (this.i < n1 && n < budget) {
      const q = this.chart.point(this.i, this.j);
      this.grid.set(this.i, this.j, !this.checker.isFree(q));
      n++;
      if (++this.j >= n2) {
        this.j = 0;
        this.i++;
      }
    }
    return n;
  }
}

/**
 * Shortest 4-connected path through *free* cells by breadth-first flood from
 * `from` — a flood fill with parent pointers, which is all the connectivity
 * question needs. Chapter 6 does search properly; this exists so the disc
 * widget can draw the route a point robot would take through the inflated
 * map, and returns null when the two cells lie in different components.
 */
export function shortestCellPath(
  grid: BitGrid,
  from: [number, number],
  to: [number, number],
  periodic: readonly [boolean, boolean] = [false, false],
): [number, number][] | null {
  const { n1, n2 } = grid;
  const idx = (i: number, j: number) => i * n2 + j;
  if (grid.get(from[0], from[1]) || grid.get(to[0], to[1])) return null;
  const parent = new Int32Array(n1 * n2).fill(-2);
  const queue: number[] = [idx(from[0], from[1])];
  parent[queue[0]] = -1;
  let head = 0;
  const target = idx(to[0], to[1]);
  while (head < queue.length) {
    const k = queue[head++];
    if (k === target) break;
    const i = Math.floor(k / n2);
    const j = k - i * n2;
    const push = (ii: number, jj: number) => {
      if (ii < 0 || ii >= n1) {
        if (!periodic[0]) return;
        ii = (ii + n1) % n1;
      }
      if (jj < 0 || jj >= n2) {
        if (!periodic[1]) return;
        jj = (jj + n2) % n2;
      }
      const t = idx(ii, jj);
      if (parent[t] !== -2 || grid.bits[t] === 1) return;
      parent[t] = k;
      queue.push(t);
    };
    push(i - 1, j);
    push(i + 1, j);
    push(i, j - 1);
    push(i, j + 1);
  }
  if (parent[target] === -2) return null;
  const path: [number, number][] = [];
  for (let k = target; k !== -1; k = parent[k]) path.push([Math.floor(k / n2), k - Math.floor(k / n2) * n2]);
  return path.reverse();
}
