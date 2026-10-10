/**
 * Coverage on a cell decomposition — Choset Ch. 6 intro, §6.2.1, §6.2.5.
 *
 * "Coverage can be reduced to finding an exhaustive walk through the
 * adjacency graph": a depth-first search lists every cell (Choset Fig. 6.10's
 * walk 1–2–4–2–3–…), and inside each cell the robot laps along the slice,
 * stepping one effector width 2r over along the floor or ceiling between laps.
 * Moving between cells that are not next to each other is a planning query,
 * answered by Chapter 6's A* on the trapezoids (which refine any boustrophedon
 * cell, and are convex, so the midpoint–centroid path is collision-free).
 *
 * What the module measures, rather than asserts:
 *   - `coverageRaster`: the covered fraction of Q_free on a raster, with the
 *     effector a disc of radius r swept along the path;
 *   - `uncoveredBound`: a bound on the area the lawnmower can miss. Every
 *     point of a cell lies within r horizontally of some lap; the lap at x_j
 *     spans [f(x_j), c(x_j)], so a point at (x, y) is missed only if y < f(x_j)
 *     or y > c(x_j) for its nearest lap — a sliver under a floor dip or over a
 *     ceiling bump. Integrating, the missed area is at most r times the total
 *     vertical variation of all floors and ceilings;
 *   - `pathLengthBound`: Choset's Δ²/(2r) + 4ΔN_ce + 5P_total.
 */

import type { Point2 } from '../sim/world';
import { AdjacencyGraph, cellRoute, chainRoute } from './plan';
import { freeRaster, type FreeRaster } from './slices';
import { SlabLocator, chainY, trapezoidal, type Cell, type Decomposition } from './sweep';
import { enclosingDiameter, totalPerimeter, type CellWorld } from './world';

const dist = (a: Point2, b: Point2) => Math.hypot(a.x - b.x, a.y - b.y);

// ---------------------------------------------------------------------------
// The exhaustive walk (DFS on the adjacency graph)
// ---------------------------------------------------------------------------

/**
 * Depth-first exhaustive walk from `start`, including the backtracking steps,
 * exactly as Choset prints it: 1–2–4–2–3–5–6–5–7–5–8–1. `order` lists each
 * cell at its first visit. O(V + E).
 */
export function exhaustiveWalk(g: AdjacencyGraph, start: number): { walk: number[]; order: number[] } {
  const n = g.d.cells.length;
  const seen = new Uint8Array(n);
  const walk: number[] = [start];
  const order: number[] = [start];
  seen[start] = 1;
  const stack: number[] = [start];
  while (stack.length) {
    const top = stack[stack.length - 1];
    const nbrs = [...new Set(g.neighbors(top).map((e) => e.to))].sort((a, b) => a - b);
    const nxt = nbrs.find((m) => !seen[m]);
    if (nxt === undefined) {
      stack.pop();
      if (stack.length) walk.push(stack[stack.length - 1]);
      continue;
    }
    seen[nxt] = 1;
    stack.push(nxt);
    walk.push(nxt);
    order.push(nxt);
  }
  return { walk, order };
}

// ---------------------------------------------------------------------------
// Within-cell lapping
// ---------------------------------------------------------------------------

/**
 * Lap abscissae for a cell spanning [xl, xr] with effector width w = 2r:
 * ⌈(xr − xl)/w⌉ laps, the outer ones r inside the cell's ends and the rest
 * evenly spaced, so no point is farther than r (horizontally) from a lap.
 */
export function lapXs(xl: number, xr: number, w: number): number[] {
  const width = xr - xl;
  const n = Math.max(1, Math.ceil(width / w - 1e-9));
  if (n === 1) return [(xl + xr) / 2];
  const r = w / 2;
  const s = (width - 2 * r) / (n - 1);
  return Array.from({ length: n }, (_, k) => xl + r + k * s);
}

/** Chain points strictly between xa and xb, in travel order, with both ends evaluated. */
function chainBetween(chain: readonly Point2[], xa: number, xb: number): Point2[] {
  const lo = Math.min(xa, xb), hi = Math.max(xa, xb);
  const inner = chain.filter((p) => p.x > lo + 1e-12 && p.x < hi - 1e-12);
  if (xa > xb) inner.reverse();
  return [{ x: xa, y: chainY(chain, xa) }, ...inner, { x: xb, y: chainY(chain, xb) }];
}

export type PieceKind = 'lap' | 'boundary' | 'transit';

export interface PathPiece {
  kind: PieceKind;
  /** The cell being covered (−1 for a transit). */
  cell: number;
  pts: Point2[];
}

/** The lawnmower for one cell, sweeping in `dir`, first lap starting on the floor or the ceiling. */
export function cellPattern(cell: Cell, w: number, dir: 1 | -1, startOnFloor: boolean): PathPiece[] {
  const xs = lapXs(cell.xRange[0], cell.xRange[1], w);
  if (dir === -1) xs.reverse();
  const out: PathPiece[] = [];
  let onFloor = startOnFloor;
  xs.forEach((x, k) => {
    const f = { x, y: chainY(cell.floor, x) };
    const c = { x, y: chainY(cell.ceiling, x) };
    out.push({ kind: 'lap', cell: cell.id, pts: onFloor ? [f, c] : [c, f] });
    onFloor = !onFloor;
    if (k + 1 < xs.length) {
      // Step one width over along whichever boundary the lap ended on.
      const chain = onFloor ? cell.floor : cell.ceiling;
      out.push({ kind: 'boundary', cell: cell.id, pts: chainBetween(chain, x, xs[k + 1]) });
    }
  });
  return out;
}

export const piecesLength = (pieces: readonly PathPiece[], kind?: PieceKind) =>
  pieces.filter((p) => !kind || p.kind === kind).reduce((s, p) => {
    let L = 0;
    for (let i = 1; i < p.pts.length; i++) L += dist(p.pts[i - 1], p.pts[i]);
    return s + L;
  }, 0);

export function flatten(pieces: readonly PathPiece[]): Point2[] {
  const out: Point2[] = [];
  for (const p of pieces) for (const q of p.pts) {
    const last = out[out.length - 1];
    if (!last || dist(last, q) > 1e-12) out.push(q);
  }
  return out;
}

// ---------------------------------------------------------------------------
// The coverage path
// ---------------------------------------------------------------------------

export interface CoveragePath {
  pieces: PathPiece[];
  points: Point2[];
  /** Cells in the order they are covered (DFS first visits). */
  order: number[];
  /** The DFS walk with backtracking, for the figure. */
  walk: number[];
  laps: number;
  lapLength: number;
  boundaryLength: number;
  transitLength: number;
  length: number;
  /** Laps used in each cell, by cell id. */
  lapsPerCell: number[];
}

export interface CoverageOptions {
  /** Where the robot starts; defaults to the first cell's leftmost point. */
  start?: Point2;
  /** Decomposition used for transits (convex cells); defaults to the world's trapezoids. */
  transit?: Decomposition;
}

/**
 * Offline coverage of a decomposition with effector width 2r = `twoR`:
 * exhaustive walk for the cell order, lawnmower inside each cell (entered at
 * whichever corner is nearest), A*-planned transits between cells.
 */
export function coveragePath(d: Decomposition, twoR: number, opts: CoverageOptions = {}): CoveragePath {
  const world = d.world;
  const trap = opts.transit ?? (d.mode === 'trapezoidal' ? d : trapezoidal(world));
  const tg = new AdjacencyGraph(trap);
  const loc = new SlabLocator(trap);
  const g = new AdjacencyGraph(d);
  const startCell = opts.start ? (new SlabLocator(d).locate(opts.start) ?? 0) : 0;
  const { walk, order } = exhaustiveWalk(g, startCell);
  const pieces: PathPiece[] = [];
  let cur: Point2 | null = opts.start ?? null;
  const lapsPerCell = d.cells.map(() => 0);

  const transit = (p: Point2, q: Point2) => {
    if (dist(p, q) < 1e-12) return;
    const a = loc.locate(p, 1e-7);
    const b = loc.locate(q, 1e-7);
    if (a === null || b === null) throw new Error('coveragePath: transit endpoint outside the free space');
    const route = cellRoute(tg, a, b);
    if (!route) throw new Error('coveragePath: no route between cells');
    pieces.push({ kind: 'transit', cell: -1, pts: chainRoute(tg, route, p, q) });
  };

  for (const id of order) {
    const cell = d.cells[id];
    const xs = lapXs(cell.xRange[0], cell.xRange[1], twoR);
    lapsPerCell[id] = xs.length;
    // Pick the entry corner nearest to where we are.
    let best: { dir: 1 | -1; floor: boolean; q: Point2 } | null = null;
    for (const dir of [1, -1] as const) {
      for (const floor of [true, false]) {
        const x = dir === 1 ? xs[0] : xs[xs.length - 1];
        const q = { x, y: floor ? chainY(cell.floor, x) : chainY(cell.ceiling, x) };
        if (!best || (cur && dist(cur, q) < dist(cur, best.q))) best = { dir, floor, q };
        if (!cur) break;
      }
      if (!cur) break;
    }
    const pattern = cellPattern(cell, twoR, best!.dir, best!.floor);
    if (cur) transit(cur, pattern[0].pts[0]);
    pieces.push(...pattern);
    const last = pattern[pattern.length - 1];
    cur = last.pts[last.pts.length - 1];
  }

  const lapLength = piecesLength(pieces, 'lap');
  const boundaryLength = piecesLength(pieces, 'boundary');
  const transitLength = piecesLength(pieces, 'transit');
  return {
    pieces,
    points: flatten(pieces),
    order,
    walk,
    laps: lapsPerCell.reduce((s, n) => s + n, 0),
    lapLength,
    boundaryLength,
    transitLength,
    length: lapLength + boundaryLength + transitLength,
    lapsPerCell,
  };
}

// ---------------------------------------------------------------------------
// Measuring coverage
// ---------------------------------------------------------------------------

export interface CoverageReport {
  raster: FreeRaster;
  covered: Uint8Array;
  freePixels: number;
  coveredPixels: number;
  fraction: number;
  /** Area of free pixels whose centre is farther than r from the path. */
  uncoveredArea: number;
}

/** Mark every free pixel within r of a polyline. Reuses a raster if given. */
export function markCovered(r: FreeRaster, covered: Uint8Array, pts: readonly Point2[], radius: number): void {
  const R2 = radius * radius;
  for (let s = 0; s < pts.length; s++) {
    const a = pts[s];
    const b = s + 1 < pts.length ? pts[s + 1] : a;
    const i0 = Math.max(0, Math.floor((Math.min(a.x, b.x) - radius - r.x0) / r.cell));
    const i1 = Math.min(r.nx - 1, Math.floor((Math.max(a.x, b.x) + radius - r.x0) / r.cell));
    const j0 = Math.max(0, Math.floor((Math.min(a.y, b.y) - radius - r.y0) / r.cell));
    const j1 = Math.min(r.ny - 1, Math.floor((Math.max(a.y, b.y) + radius - r.y0) / r.cell));
    const dx = b.x - a.x, dy = b.y - a.y;
    const L2 = dx * dx + dy * dy;
    for (let j = j0; j <= j1; j++) {
      const py = r.y0 + (j + 0.5) * r.cell;
      for (let i = i0; i <= i1; i++) {
        const k = j * r.nx + i;
        if (!r.free[k] || covered[k]) continue;
        const px = r.x0 + (i + 0.5) * r.cell;
        let t = L2 > 0 ? ((px - a.x) * dx + (py - a.y) * dy) / L2 : 0;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        const ex = a.x + t * dx - px, ey = a.y + t * dy - py;
        if (ex * ex + ey * ey <= R2) covered[k] = 1;
      }
    }
  }
}

export function coverageReport(world: CellWorld, pts: readonly Point2[], radius: number, res = 0.05, raster?: FreeRaster): CoverageReport {
  const r = raster ?? freeRaster(world, res);
  const covered = new Uint8Array(r.free.length);
  markCovered(r, covered, pts, radius);
  let freePixels = 0, coveredPixels = 0;
  for (let k = 0; k < r.free.length; k++) {
    if (!r.free[k]) continue;
    freePixels++;
    if (covered[k]) coveredPixels++;
  }
  return {
    raster: r,
    covered,
    freePixels,
    coveredPixels,
    fraction: coveredPixels / freePixels,
    uncoveredArea: (freePixels - coveredPixels) * r.cell * r.cell,
  };
}

/** Total variation in y of a chain: Σ |Δy| over its edges. */
function variation(chain: readonly Point2[]): number {
  let v = 0;
  for (let i = 1; i < chain.length; i++) v += Math.abs(chain[i].y - chain[i - 1].y);
  return v;
}

/**
 * The lawnmower's missed-area bound: r · Σ_cells (TV(floor) + TV(ceiling)).
 * (See the module comment for the two-line proof.)
 */
export function uncoveredBound(d: Decomposition, r: number): number {
  return r * d.cells.reduce((s, c) => s + variation(c.floor) + variation(c.ceiling), 0);
}

export interface LengthBound {
  delta: number;
  pTotal: number;
  nCe: number;
  /** Δ²/(2r) + 4ΔN_ce + 5P_total — Choset §6.2.5. */
  bound: number;
  /** ⌈Δ/2r⌉ + N_ce: the most laps the argument allows. */
  maxLaps: number;
}

export function pathLengthBound(world: CellWorld, twoR: number, nCe: number): LengthBound {
  const delta = enclosingDiameter(world);
  const pTotal = totalPerimeter(world);
  return {
    delta,
    pTotal,
    nCe,
    bound: (delta * delta) / twoR + 4 * delta * nCe + 5 * pTotal,
    maxLaps: Math.ceil(delta / twoR) + nCe,
  };
}

/** Dense free-space check of a polyline: every sample at spacing `h` lies in cl(Q_free). */
export function polylineInFreeSpace(
  isFree: (p: Point2) => boolean,
  pts: readonly Point2[],
  h = 0.01,
): { ok: boolean; samples: number; worst: Point2 | null } {
  let samples = 0;
  for (let i = 0; i + 1 < pts.length; i++) {
    const a = pts[i], b = pts[i + 1];
    const n = Math.max(1, Math.ceil(dist(a, b) / h));
    for (let k = 0; k <= n; k++) {
      const p = { x: a.x + ((b.x - a.x) * k) / n, y: a.y + ((b.y - a.y) * k) / n };
      samples++;
      if (!isFree(p)) return { ok: false, samples, worst: p };
    }
  }
  return { ok: true, samples, worst: null };
}

/**
 * For animation: the arc length at which each free pixel is first covered by
 * the effector (∞ if never). Revealing pixels with value ≤ s draws coverage
 * "growing" behind the robot without re-rasterizing every frame.
 */
export function coverageTimeline(r: FreeRaster, pts: readonly Point2[], radius: number): Float32Array {
  const first = new Float32Array(r.free.length).fill(Infinity);
  const R2 = radius * radius;
  let s = 0;
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i];
    const b = i + 1 < pts.length ? pts[i + 1] : a;
    const L = dist(a, b);
    const i0 = Math.max(0, Math.floor((Math.min(a.x, b.x) - radius - r.x0) / r.cell));
    const i1 = Math.min(r.nx - 1, Math.floor((Math.max(a.x, b.x) + radius - r.x0) / r.cell));
    const j0 = Math.max(0, Math.floor((Math.min(a.y, b.y) - radius - r.y0) / r.cell));
    const j1 = Math.min(r.ny - 1, Math.floor((Math.max(a.y, b.y) + radius - r.y0) / r.cell));
    const dx = b.x - a.x, dy = b.y - a.y;
    const L2 = dx * dx + dy * dy;
    for (let j = j0; j <= j1; j++) {
      const py = r.y0 + (j + 0.5) * r.cell;
      for (let ii = i0; ii <= i1; ii++) {
        const k = j * r.nx + ii;
        if (!r.free[k] || first[k] !== Infinity) continue;
        const px = r.x0 + (ii + 0.5) * r.cell;
        let t = L2 > 0 ? ((px - a.x) * dx + (py - a.y) * dy) / L2 : 0;
        t = t < 0 ? 0 : t > 1 ? 1 : t;
        const ex = a.x + t * dx - px, ey = a.y + t * dy - py;
        if (ex * ex + ey * ey <= R2) first[k] = s + t * L;
      }
    }
    s += L;
  }
  return first;
}

/** Cumulative arc length of a polyline, and the point at arc length s. */
export function arcLengths(pts: readonly Point2[]): number[] {
  const out = [0];
  for (let i = 1; i < pts.length; i++) out.push(out[i - 1] + dist(pts[i - 1], pts[i]));
  return out;
}

export function pointAtArc(pts: readonly Point2[], cum: readonly number[], s: number): { p: Point2; i: number } {
  if (s <= 0) return { p: pts[0], i: 0 };
  if (s >= cum[cum.length - 1]) return { p: pts[pts.length - 1], i: pts.length - 1 };
  let lo = 0, hi = cum.length - 1;
  while (hi - lo > 1) {
    const m = (lo + hi) >> 1;
    if (cum[m] <= s) lo = m;
    else hi = m;
  }
  const t = (s - cum[lo]) / Math.max(cum[hi] - cum[lo], 1e-12);
  return { p: { x: pts[lo].x + (pts[hi].x - pts[lo].x) * t, y: pts[lo].y + (pts[hi].y - pts[lo].y) * t }, i: lo };
}
