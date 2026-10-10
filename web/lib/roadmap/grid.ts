/**
 * The grid GVD — Choset §5.2.5's brushfire method, on Chapter 7's `DistGrid`.
 *
 * A pixel two fronts from different obstacles reach is a GVD pixel; Chapter 7's
 * `brushfire` already records it (`ridge`, `back2`). This file turns those
 * pixels into a roadmap: a graph over ridge cells with 8-neighbour adjacency,
 * an *access* map that climbs the brushfire labels from any free cell to the
 * ridge (the grid version of gradient ascent of D), and A* between ridge cells
 * through Chapter 6's engine. The thinning is a heuristic — one pass of the
 * Zhang–Suen rule, kept only where it does not break connectivity — and the
 * prose says so: grid ridges are jagged (Choset Problem 14), and the exact
 * diagram is what they approximate.
 */

import type { DistGrid } from '../potential/brushfire';
import { bestFirst, PRIORITY_ASTAR } from '../search/best-first';
import { KeyedHeap } from '../search/heap';
import { cellCenter, cellIJ, type OccupancyGrid } from '../search/grid';
import type { Edge, Graph } from '../search/graph';
import type { Point2 } from '../sim/world';

const STEPS8: readonly [number, number][] = [
  [1, 0], [-1, 0], [0, 1], [0, -1],
  [1, 1], [1, -1], [-1, 1], [-1, -1],
];

function neighbours8(g: OccupancyGrid, cell: number): number[] {
  const i = cell % g.nx;
  const j = (cell - i) / g.nx;
  const out: number[] = [];
  for (const [di, dj] of STEPS8) {
    const ii = i + di;
    const jj = j + dj;
    if (ii < 0 || jj < 0 || ii >= g.nx || jj >= g.ny) continue;
    out.push(jj * g.nx + ii);
  }
  return out;
}

/**
 * Zhang–Suen thinning restricted to the ridge mask: a ridge pixel is deleted
 * when it has 2–6 ridge neighbours, exactly one 0→1 transition around it, and
 * the directional conditions hold. Iterated until nothing changes. Heuristic:
 * it keeps the skeleton 8-connected but does not promise to keep it centred.
 */
export function thinRidge(dist: DistGrid): Uint8Array {
  const g = dist.grid;
  const mask = Uint8Array.from(dist.ridge);
  for (let k = 0; k < mask.length; k++) if (g.occupied[k]) mask[k] = 0;
  const at = (i: number, j: number) => (i < 0 || j < 0 || i >= g.nx || j >= g.ny ? 0 : mask[j * g.nx + i]);
  let changed = true;
  let guard = 0;
  while (changed && guard++ < 50) {
    changed = false;
    for (const pass of [0, 1]) {
      const kill: number[] = [];
      for (let j = 0; j < g.ny; j++) {
        for (let i = 0; i < g.nx; i++) {
          if (!mask[j * g.nx + i]) continue;
          const p2 = at(i, j + 1), p3 = at(i + 1, j + 1), p4 = at(i + 1, j), p5 = at(i + 1, j - 1);
          const p6 = at(i, j - 1), p7 = at(i - 1, j - 1), p8 = at(i - 1, j), p9 = at(i - 1, j + 1);
          const ring = [p2, p3, p4, p5, p6, p7, p8, p9];
          const B = ring.reduce((s, v) => s + v, 0);
          if (B < 2 || B > 6) continue;
          let A = 0;
          for (let k = 0; k < 8; k++) if (ring[k] === 0 && ring[(k + 1) % 8] === 1) A++;
          if (A !== 1) continue;
          const c1 = pass === 0 ? p2 * p4 * p6 === 0 && p4 * p6 * p8 === 0 : p2 * p4 * p8 === 0 && p2 * p6 * p8 === 0;
          if (c1) kill.push(j * g.nx + i);
        }
      }
      for (const k of kill) mask[k] = 0;
      if (kill.length) changed = true;
    }
  }
  return mask;
}

/* -------------------------------------------------------------------------- */
/* Connected walls: the feature-jump ridge                                     */
/* -------------------------------------------------------------------------- */

/**
 * Choset's two-back-pointer rule names *obstacles*, and Chapter 7's brushfire
 * takes an obstacle to be an 8-connected component of occupied pixels. The
 * Apartment's walls are one component — every interior wall meets the shell —
 * so that rule finds nothing there, exactly as S_ij is empty for a single
 * obstacle until the SS_ij refinement asks for two *distinct closest points*
 * with distinct gradients. The grid version of that refinement is a feature
 * transform: each free pixel remembers the obstacle pixel nearest to it (its
 * literal back pointer), and a pixel is on the GVD where a neighbour's nearest
 * obstacle pixel is far from its own — the closest point jumps. Brushfire's
 * labels are a grid metric and its origin pixels are tie-broken by queue
 * order, which makes that jump test unreliable along long walls; so the
 * features here are propagated under the *Euclidean* distance (Danielsson's
 * vector distance transform, Dijkstra over 8-neighbours keyed on the true
 * distance to the propagated origin), which is exact up to rare sub-pixel
 * cases and is the same quantity Chapter 7 compared brushfire against.
 */
export interface FeatureTransform {
  /** Squared Euclidean distance (in cells) from each pixel to its nearest obstacle pixel. */
  dist2: Float64Array;
  /** The nearest obstacle pixel — Choset's back pointer to "the obstacle pixel from which the wave originated". */
  origin: Int32Array;
}

export function featureTransform(g: OccupancyGrid): FeatureTransform {
  const n = g.nx * g.ny;
  const dist2 = new Float64Array(n).fill(Infinity);
  const origin = new Int32Array(n).fill(-1);
  const heap = new KeyedHeap<number>();
  for (let k = 0; k < n; k++) {
    if (g.occupied[k]) {
      dist2[k] = 0;
      origin[k] = k;
      heap.push([0], k);
    }
  }
  const sq = (a: number, b: number) => {
    const [ai, aj] = cellIJ(g, a);
    const [bi, bj] = cellIJ(g, b);
    return (ai - bi) ** 2 + (aj - bj) ** 2;
  };
  for (;;) {
    const top = heap.pop();
    if (!top) break;
    const c = top.value;
    if (top.key[0] > dist2[c]) continue; // stale
    const o = origin[c];
    for (const q of neighbours8(g, c)) {
      if (g.occupied[q]) continue;
      const d2 = sq(q, o);
      if (d2 < dist2[q]) {
        dist2[q] = d2;
        origin[q] = o;
        heap.push([d2], q);
      }
    }
  }
  return { dist2, origin };
}

/**
 * The grid GVD of a possibly connected obstacle set: pixel p is a ridge pixel
 * when some 8-neighbour q has a nearest obstacle pixel at least `gamma` cells
 * from p's, the two directions to those pixels differ by more than `minAngle`
 * (the ∇d_i ≠ ∇d_j of SS_ij), and p is at least as far from the obstacles as
 * q (so the ridge is one pixel wide where it can be).
 */
export function ridgeMaskFromFeatures(g: OccupancyGrid, ft: FeatureTransform, gamma = 2.5, minAngle = Math.PI / 9): Uint8Array {
  const mask = new Uint8Array(g.nx * g.ny);
  const gamma2 = gamma * gamma;
  for (let p = 0; p < mask.length; p++) {
    if (g.occupied[p] || ft.origin[p] < 0) continue;
    const [pi, pj] = cellIJ(g, p);
    const [oi, oj] = cellIJ(g, ft.origin[p]);
    for (const q of neighbours8(g, p)) {
      if (g.occupied[q] || ft.origin[q] < 0 || ft.origin[q] === ft.origin[p]) continue;
      if (ft.dist2[q] > ft.dist2[p]) continue;
      const [qi, qj] = cellIJ(g, ft.origin[q]);
      if ((qi - oi) ** 2 + (qj - oj) ** 2 < gamma2) continue;
      const a1 = Math.atan2(pj - oj, pi - oi);
      const a2 = Math.atan2(pj - qj, pi - qi);
      let da = Math.abs(a1 - a2);
      if (da > Math.PI) da = 2 * Math.PI - da;
      if (da < minAngle) continue;
      mask[p] = 1;
      break;
    }
  }
  return mask;
}

/** The ridge cells as a graph: 8-neighbour adjacency, Euclidean step costs in metres. */
export class RidgeGraph implements Graph<number> {
  readonly cells: number[];
  private readonly isRidge: Uint8Array;

  constructor(
    readonly dist: DistGrid,
    mask?: Uint8Array,
  ) {
    this.isRidge = mask ?? Uint8Array.from(dist.ridge);
    for (let k = 0; k < this.isRidge.length; k++) if (dist.grid.occupied[k]) this.isRidge[k] = 0;
    this.cells = [];
    for (let k = 0; k < this.isRidge.length; k++) if (this.isRidge[k]) this.cells.push(k);
  }

  has(cell: number): boolean {
    return this.isRidge[cell] === 1;
  }

  neighbors(cell: number): readonly Edge<number>[] {
    const g = this.dist.grid;
    const [i, j] = cellIJ(g, cell);
    const out: Edge<number>[] = [];
    for (const nb of neighbours8(g, cell)) {
      if (!this.isRidge[nb]) continue;
      const [ni, nj] = cellIJ(g, nb);
      out.push({ to: nb, cost: Math.hypot(ni - i, nj - j) * g.cellSize });
    }
    return out;
  }

  key(cell: number): number {
    return cell;
  }

  /** Connected components of the ridge; the Apartment has one per free-space component (Derivation 4). */
  components(): number {
    const seen = new Uint8Array(this.isRidge.length);
    let count = 0;
    for (const s of this.cells) {
      if (seen[s]) continue;
      count++;
      const stack = [s];
      seen[s] = 1;
      while (stack.length) {
        const c = stack.pop()!;
        for (const { to } of this.neighbors(c)) {
          if (!seen[to]) {
            seen[to] = 1;
            stack.push(to);
          }
        }
      }
    }
    return count;
  }

  /** Degree-based classification: ≥ 3 ridge neighbours is a meet cell, 1 a spoke end. */
  meetCells(): number[] {
    return this.cells.filter((c) => this.neighbors(c).length >= 3);
  }

  /** A* between two ridge cells with the Euclidean heuristic. */
  shortestPath(a: number, b: number): { cells: number[]; length: number; expansions: number } | null {
    const g = this.dist.grid;
    const [bi, bj] = cellIJ(g, b);
    const t = bestFirst(this, a, b, (c) => {
      const [i, j] = cellIJ(g, c);
      return Math.hypot(i - bi, j - bj) * g.cellSize;
    }, PRIORITY_ASTAR);
    return t.status === 'found' ? { cells: t.path, length: t.cost, expansions: t.expansions } : null;
  }

  nearest(cell: number): number {
    const g = this.dist.grid;
    const [i, j] = cellIJ(g, cell);
    let best = -1;
    let bd = Infinity;
    for (const c of this.cells) {
      const [ci, cj] = cellIJ(g, c);
      const d = Math.hypot(ci - i, cj - j);
      if (d < bd) {
        bd = d;
        best = c;
      }
    }
    return best;
  }
}

/**
 * Grid accessibility: from a free cell, step to the 8-neighbour with the
 * largest brushfire label (ties to the first in scan order) until a ridge
 * cell is reached or no neighbour is higher — the discrete ∇D ascent. Returns
 * the cell path; its last cell is on the ridge when `landed`.
 */
export function gridAscent(dist: DistGrid, start: number, ridge: RidgeGraph, maxSteps = 10000): { cells: number[]; landed: boolean } {
  const g = dist.grid;
  const cells = [start];
  let c = start;
  for (let it = 0; it < maxSteps; it++) {
    if (ridge.has(c)) return { cells, landed: true };
    let best = -1;
    let bestLabel = dist.label[c];
    for (const nb of neighbours8(g, c)) {
      if (g.occupied[nb]) continue;
      // Prefer a ridge neighbour outright: the front met there.
      if (ridge.has(nb)) {
        cells.push(nb);
        return { cells, landed: true };
      }
      if (dist.label[nb] > bestLabel) {
        bestLabel = dist.label[nb];
        best = nb;
      }
    }
    if (best < 0) {
      // A local maximum of the label that is not flagged (a plateau between fronts): snap to the nearest ridge cell.
      const n = ridge.nearest(c);
      if (n >= 0) {
        cells.push(n);
        return { cells, landed: true };
      }
      return { cells, landed: false };
    }
    c = best;
    cells.push(c);
  }
  return { cells, landed: false };
}

export interface GridRoadmapQuery {
  /** Access path (cells) from the start to the ridge. */
  access: number[];
  /** Ridge path (cells). */
  ride: number[];
  /** Departure path (cells) from the ridge to the goal — the goal's access reversed. */
  depart: number[];
  /** All points, start to goal, in metres. */
  points: Point2[];
  length: number;
  minClearance: number;
}

/** Get on, ride, get off — on the grid GVD. */
export function gridGvdQuery(dist: DistGrid, ridge: RidgeGraph, start: number, goal: number): GridRoadmapQuery | null {
  const a = gridAscent(dist, start, ridge);
  const b = gridAscent(dist, goal, ridge);
  if (!a.landed || !b.landed) return null;
  const ride = ridge.shortestPath(a.cells[a.cells.length - 1], b.cells[b.cells.length - 1]);
  if (!ride) return null;
  const depart = b.cells.slice().reverse();
  const cells = [...a.cells, ...ride.cells.slice(1), ...depart.slice(1)];
  const points = cells.map((c) => cellCenter(dist.grid, c));
  let length = 0;
  let minClearance = Infinity;
  for (let k = 0; k < cells.length; k++) {
    if (k > 0) length += Math.hypot(points[k].x - points[k - 1].x, points[k].y - points[k - 1].y);
    minClearance = Math.min(minClearance, (dist.label[cells[k]] - 1) * dist.grid.cellSize);
  }
  return { access: a.cells, ride: ride.cells, depart, points, length, minClearance };
}
