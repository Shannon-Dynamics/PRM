/**
 * The vertical sweep — Choset §6.1 (trapezoidal) and §6.2.1 (boustrophedon) as
 * one algorithm with two bookkeeping rules.
 *
 * A vertical line sweeps left to right and stops only at vertices (events).
 * Between events nothing happens: the list L of edges the line crosses keeps
 * its order, and the free intervals between consecutive edges — the slice
 * intervals Q^j_free,λ — only stretch or shrink. At an event v the two edges
 * incident to v are inserted, deleted, or one replaces the other (the four
 * event types of Choset Fig. 6.4), and the edges e_LOWER, e_UPPER of L just
 * below and above v bound its vertical extensions, which exist exactly where
 * free space lies above or below v.
 *
 *  - trapezoidal:   every event closes the cells it touches and opens new
 *                   ones; every extension is a cell wall. Cells are
 *                   trapezoids (or triangles), convex, so the midpoint–
 *                   centroid path through them is collision-free.
 *  - boustrophedon: only events with extensions both up *and* down (split,
 *                   merge) and the slice's birth and death touch the cell
 *                   structure; a one-sided event (a pass-through vertex) just
 *                   bends the floor or ceiling of the cell it lies on. Those
 *                   are exactly the points where Q_free,λ changes
 *                   connectivity — the critical points of h(x, y) = x on the
 *                   boundary (Chapter 9, Lemma 5.5.2).
 *
 * The port keeps L as a sorted array, O(n) per update rather than a balanced
 * tree's O(log n); the order of events and every list in Choset Figs. 6.5–6.8
 * are the same either way, and the checks compare L against a brute-force
 * re-sort at every event.
 */

import type { Point2 } from '../sim/world';
import { allEdges, polygonsOf, type CellEdge, type CellWorld } from './world';

/** Choset Fig. 6.4: e_lower is the incident edge whose other vertex is lower. */
export type EventKind = 'BothRight' | 'BothLeft' | 'LowerLeftUpperRight' | 'LowerRightUpperLeft';

/** What the event does to the slice Q_free,λ. Only `pass` is not a critical point. */
export type SliceEvent = 'birth' | 'split' | 'merge' | 'death' | 'pass';

export type DecompositionMode = 'trapezoidal' | 'boustrophedon';

export interface Extension {
  a: Point2;
  b: Point2;
}

export interface Event {
  index: number;
  v: Point2;
  /** Polygon the vertex belongs to (−1 = boundary) and its index there. */
  owner: number;
  vertex: number;
  kind: EventKind;
  slice: SliceEvent;
  /** Choset's e_lower and e_upper: the two edges that contain v. */
  eLower: number;
  eUpper: number;
  /** The edges of L immediately below and above v (null at the boundary's extremes). */
  eLOWER: number | null;
  eUPPER: number | null;
  /** L just before and just after the event, bottom to top. */
  Lbefore: number[];
  Lafter: number[];
  extUp: Extension | null;
  extDown: Extension | null;
  /** Cells this event closed and opened (ids into Decomposition.cells). */
  closed: number[];
  opened: number[];
}

export interface Cell {
  id: number;
  /** Floor and ceiling chains, left to right, as polylines. */
  floor: Point2[];
  ceiling: Point2[];
  /** Closed polygon: the floor then the ceiling reversed. */
  polygon: Point2[];
  xRange: [number, number];
  openEvent: number;
  closeEvent: number;
  area: number;
  centroid: Point2;
}

export interface Adjacency {
  a: number;
  b: number;
  /** The shared vertical extension. */
  segment: Extension;
  event: number;
}

export interface Decomposition {
  mode: DecompositionMode;
  world: CellWorld;
  edges: CellEdge[];
  cells: Cell[];
  adjacency: Adjacency[];
  events: Event[];
}

const TAU = 2 * Math.PI;
const wrap = (a: number) => ((a % TAU) + TAU) % TAU;

/** y of edge e at abscissa x (general position: no vertical edges). */
export function edgeYAt(e: CellEdge, x: number): number {
  const dx = e.b.x - e.a.x;
  if (Math.abs(dx) < 1e-15) return Math.min(e.a.y, e.b.y);
  return e.a.y + ((e.b.y - e.a.y) * (x - e.a.x)) / dx;
}

/** Piecewise-linear y of a left-to-right chain at abscissa x (clamped at the ends). */
export function chainY(chain: readonly Point2[], x: number): number {
  if (x <= chain[0].x) return chain[0].y;
  for (let i = 1; i < chain.length; i++) {
    const a = chain[i - 1];
    const b = chain[i];
    if (x <= b.x) {
      const dx = b.x - a.x;
      return dx > 1e-15 ? a.y + ((b.y - a.y) * (x - a.x)) / dx : Math.max(a.y, b.y);
    }
  }
  return chain[chain.length - 1].y;
}

function polygonCentroid(poly: readonly Point2[]): { area: number; c: Point2 } {
  let A = 0, cx = 0, cy = 0;
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    const cr = p.x * q.y - q.x * p.y;
    A += cr;
    cx += (p.x + q.x) * cr;
    cy += (p.y + q.y) * cr;
  }
  A /= 2;
  if (Math.abs(A) < 1e-14) {
    const m = poly.reduce((s, p) => ({ x: s.x + p.x / poly.length, y: s.y + p.y / poly.length }), { x: 0, y: 0 });
    return { area: 0, c: m };
  }
  return { area: Math.abs(A), c: { x: cx / (6 * A), y: cy / (6 * A) } };
}

function signedArea(poly: readonly Point2[]): number {
  let A = 0;
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    const q = poly[(i + 1) % poly.length];
    A += p.x * q.y - q.x * p.y;
  }
  return A / 2;
}

function dedupe(pts: Point2[]): Point2[] {
  const out: Point2[] = [];
  for (const p of pts) {
    const q = out[out.length - 1];
    if (!q || Math.hypot(q.x - p.x, q.y - p.y) > 1e-12) out.push(p);
  }
  if (out.length > 1 && Math.hypot(out[0].x - out[out.length - 1].x, out[0].y - out[out.length - 1].y) <= 1e-12) out.pop();
  return out;
}

/**
 * Is direction θ inside the interior sector of the (CCW) polygon at its
 * vertex v? The interior sector runs counter-clockwise from the direction to
 * the next vertex to the direction to the previous one.
 */
function inInteriorSector(v: Point2, prev: Point2, next: Point2, theta: number): boolean {
  const aNext = Math.atan2(next.y - v.y, next.x - v.x);
  const aPrev = Math.atan2(prev.y - v.y, prev.x - v.x);
  const span = wrap(aPrev - aNext);
  const d = wrap(theta - aNext);
  return d > 1e-12 && d < span - 1e-12;
}

interface OpenCell {
  id: number;
  floorEdge: number;
  ceilEdge: number;
  floor: Point2[];
  ceiling: Point2[];
  x0: number;
  openEvent: number;
}

/**
 * Sweep the world. Throws if two vertices share an x-coordinate: run
 * `generalPosition` first (Choset's assumption, enforced by a disclosed shear).
 */
export function sweepDecompose(world: CellWorld, mode: DecompositionMode): Decomposition {
  const edges = allEdges(world);
  const polys = polygonsOf(world);
  // Global edge id of edge k of polygon p.
  const base: number[] = [];
  let acc = 0;
  for (const p of polys) {
    base.push(acc);
    acc += p.length;
  }
  const edgeId = (pi: number, k: number) => base[pi] + ((k + polys[pi].length) % polys[pi].length);

  interface V {
    p: Point2;
    pi: number;
    k: number;
  }
  const verts: V[] = [];
  polys.forEach((poly, pi) => poly.forEach((p, k) => verts.push({ p, pi, k })));
  verts.sort((a, b) => a.p.x - b.p.x);
  for (let i = 1; i < verts.length; i++) {
    if (verts[i].p.x - verts[i - 1].p.x <= 1e-12) {
      throw new Error(`sweepDecompose: vertices at x = ${verts[i].p.x} are not in general position`);
    }
  }

  const orientation = polys.map((poly) => polygonCentroid(poly).area * Math.sign(signedArea(poly)));
  const y = (e: number, x: number) => edgeYAt(edges[e], x);
  let L: number[] = [];
  const open = new Map<number, OpenCell>(); // keyed by floor edge
  const cells: Cell[] = [];
  const adjacency: Adjacency[] = [];
  const events: Event[] = [];
  const pending: OpenCell[] = [];

  const openCell = (floorEdge: number, ceilEdge: number, x0: number, floorStart: Point2, ceilStart: Point2, ev: number): OpenCell => {
    const c: OpenCell = { id: pending.length, floorEdge, ceilEdge, floor: [floorStart], ceiling: [ceilStart], x0, openEvent: ev };
    pending.push(c);
    open.set(floorEdge, c);
    return c;
  };
  const closeCell = (c: OpenCell, x1: number, floorEnd: Point2, ceilEnd: Point2, ev: number) => {
    c.floor.push(floorEnd);
    c.ceiling.push(ceilEnd);
    if (open.get(c.floorEdge) === c) open.delete(c.floorEdge);
    const floor = dedupe(c.floor);
    const ceiling = dedupe(c.ceiling);
    const polygon = dedupe([...floor, ...[...ceiling].reverse()]);
    const { area, c: cen } = polygonCentroid(polygon);
    cells[c.id] = { id: c.id, floor, ceiling, polygon, xRange: [c.x0, x1], openEvent: c.openEvent, closeEvent: ev, area, centroid: cen };
  };

  verts.forEach((vt, index) => {
    const v = vt.p;
    const poly = polys[vt.pi];
    const n = poly.length;
    const prev = poly[(vt.k - 1 + n) % n];
    const next = poly[(vt.k + 1) % n];
    const ePrev = edgeId(vt.pi, vt.k - 1);
    const eNext = edgeId(vt.pi, vt.k);
    const prevRight = prev.x > v.x;
    const nextRight = next.x > v.x;

    // Choset's labels: e_lower is the incident edge whose other vertex is lower.
    const eLower = prev.y < next.y ? ePrev : eNext;
    const eUpper = eLower === ePrev ? eNext : ePrev;
    const lowerRight = (eLower === ePrev ? prev : next).x > v.x;
    const kind: EventKind =
      prevRight && nextRight ? 'BothRight' : !prevRight && !nextRight ? 'BothLeft' : !lowerRight ? 'LowerLeftUpperRight' : 'LowerRightUpperLeft';

    // Free directions at v: inside the boundary's interior sector, outside a hole's.
    // (Orientation-agnostic: a clockwise polygon — a dragged vertex can flip
    // one — has its interior on the other side of each edge.)
    const isBoundary = vt.pi === 0;
    const ccw = orientation[vt.pi] > 0;
    const inside = (theta: number) => (ccw ? inInteriorSector(v, prev, next, theta) : inInteriorSector(v, next, prev, theta));
    const free = (theta: number) => (isBoundary ? inside(theta) : !inside(theta));
    const freeUp = free(Math.PI / 2);
    const freeDown = free(-Math.PI / 2);

    const Lbefore = L.slice();
    const rest = L.filter((e) => e !== ePrev && e !== eNext);
    let i = 0;
    while (i < rest.length && y(rest[i], v.x) < v.y) i++;
    const eLOWER = i > 0 ? rest[i - 1] : null;
    const eUPPER = i < rest.length ? rest[i] : null;
    const extUp = freeUp && eUPPER !== null ? { a: v, b: { x: v.x, y: y(eUPPER, v.x) } } : null;
    const extDown = freeDown && eLOWER !== null ? { a: v, b: { x: v.x, y: y(eLOWER, v.x) } } : null;
    const closed: number[] = [];
    const opened: number[] = [];
    let slice: SliceEvent = 'pass';

    const slope = (e: number, other: Point2) => (other.y - v.y) / (other.x - v.x);
    const sPrev = slope(ePrev, prev);
    const sNext = slope(eNext, next);

    if (kind === 'BothRight') {
      // Lower edge just to the right of v has the smaller slope.
      const lo = sPrev < sNext ? ePrev : eNext;
      const hi = lo === ePrev ? eNext : ePrev;
      L = [...rest.slice(0, i), lo, hi, ...rest.slice(i)];
      if (freeUp && freeDown && eLOWER !== null && eUPPER !== null) {
        slice = 'split';
        const C = open.get(eLOWER)!;
        closeCell(C, v.x, extDown!.b, extUp!.b, index);
        closed.push(C.id);
        const A = openCell(eLOWER, lo, v.x, extDown!.b, v, index);
        const B = openCell(hi, eUPPER, v.x, v, extUp!.b, index);
        opened.push(A.id, B.id);
        adjacency.push({ a: C.id, b: A.id, segment: extDown!, event: index });
        adjacency.push({ a: C.id, b: B.id, segment: extUp!, event: index });
      } else {
        slice = 'birth';
        const A = openCell(lo, hi, v.x, v, v, index);
        opened.push(A.id);
      }
    } else if (kind === 'BothLeft') {
      // Lower edge just to the left of v has the larger slope.
      const lo = sPrev > sNext ? ePrev : eNext;
      const hi = lo === ePrev ? eNext : ePrev;
      L = rest;
      if (freeUp && freeDown && eLOWER !== null && eUPPER !== null) {
        slice = 'merge';
        const A = open.get(eLOWER)!;
        const B = open.get(hi)!;
        closeCell(A, v.x, extDown!.b, v, index);
        closeCell(B, v.x, v, extUp!.b, index);
        closed.push(A.id, B.id);
        const Cc = openCell(eLOWER, eUPPER, v.x, extDown!.b, extUp!.b, index);
        opened.push(Cc.id);
        adjacency.push({ a: A.id, b: Cc.id, segment: extDown!, event: index });
        adjacency.push({ a: B.id, b: Cc.id, segment: extUp!, event: index });
      } else {
        slice = 'death';
        const A = open.get(lo)!;
        closeCell(A, v.x, v, v, index);
        closed.push(A.id);
      }
    } else {
      // One edge on each side: the left one leaves L, the right one takes its slot.
      const eL = prevRight ? eNext : ePrev;
      const eR = eL === ePrev ? eNext : ePrev;
      L = L.map((e) => (e === eL ? eR : e));
      if (freeUp) {
        // v lies on the floor of the cell above it.
        const X = open.get(eL)!;
        if (mode === 'trapezoidal') {
          closeCell(X, v.x, v, extUp ? extUp.b : v, index);
          closed.push(X.id);
          const Y = openCell(eR, X.ceilEdge, v.x, v, extUp ? extUp.b : v, index);
          opened.push(Y.id);
          if (extUp) adjacency.push({ a: X.id, b: Y.id, segment: extUp, event: index });
        } else {
          X.floor.push(v);
          open.delete(eL);
          X.floorEdge = eR;
          open.set(eR, X);
        }
      } else {
        // v lies on the ceiling of the cell below it.
        const X = open.get(eLOWER!)!;
        if (mode === 'trapezoidal') {
          closeCell(X, v.x, extDown ? extDown.b : v, v, index);
          closed.push(X.id);
          const Y = openCell(X.floorEdge, eR, v.x, extDown ? extDown.b : v, v, index);
          opened.push(Y.id);
          if (extDown) adjacency.push({ a: X.id, b: Y.id, segment: extDown, event: index });
        } else {
          X.ceiling.push(v);
          X.ceilEdge = eR;
        }
      }
    }

    events.push({
      index,
      v,
      owner: vt.pi - 1,
      vertex: vt.k,
      kind,
      slice,
      eLower,
      eUpper,
      eLOWER,
      eUPPER,
      Lbefore,
      Lafter: L.slice(),
      extUp,
      extDown,
      closed,
      opened,
    });
  });

  return { mode, world, edges, cells, adjacency, events };
}

/** C §6.1: every vertex gets the extensions that free space allows; cells are trapezoids. */
export const trapezoidal = (world: CellWorld) => sweepDecompose(world, 'trapezoidal');

/** C §6.2.1: extensions only at critical points of h(x, y) = x. */
export const boustrophedon = (world: CellWorld) => sweepDecompose(world, 'boustrophedon');

/** The events that are critical points of the slice (boustrophedon nodes). */
export function criticalEvents(d: Decomposition): Event[] {
  return d.events.filter((e) => e.slice !== 'pass');
}

/**
 * Brute-force L at abscissa x: every edge whose x-span strictly contains x,
 * sorted by its y there. The sweep's incremental list must equal this between
 * events — the check runs it at every inter-event midpoint.
 */
export function bruteForceL(d: Decomposition, x: number): number[] {
  return d.edges
    .filter((e) => Math.min(e.a.x, e.b.x) < x && Math.max(e.a.x, e.b.x) > x)
    .sort((p, q) => edgeYAt(p, x) - edgeYAt(q, x))
    .map((e) => e.id);
}

/** Faces of the planar adjacency graph by Euler's formula, f = 2 − V + E. */
export function adjacencyFaces(d: Decomposition): number {
  return 2 - d.cells.length + d.adjacency.length;
}

/** Number of cells a sweep opens per event, for the per-vertex counting rule. */
export function cellsOpenedPerEvent(d: Decomposition): number[] {
  return d.events.map((e) => e.opened.length);
}

/**
 * The closed form for convex holes in a convex boundary, general position:
 * (m − 1) + Σ (k_i + 1) trapezoids. Each boundary vertex but the leftmost
 * opens one cell (the rightmost closes the last); each hole's leftmost vertex
 * opens two and every other vertex one.
 */
export function trapezoidCountConvex(world: CellWorld): number {
  return world.boundary.length - 1 + world.obstacles.reduce((s, o) => s + o.length + 1, 0);
}

// ---------------------------------------------------------------------------
// Point location by slabs (C §6.1, end)
// ---------------------------------------------------------------------------

/**
 * Slab point location: vertical lines through every event cut the plane into
 * slabs; a binary search finds q's slab, and among the cells spanning it the
 * one whose floor and ceiling bracket q is the answer. O(log n) to find the
 * slab; the port then scans that slab's cells, which is what the second
 * O(log n) search of the text does with a sorted edge list.
 */
export class SlabLocator {
  private readonly xs: number[];
  private readonly slabs: number[][];

  constructor(private readonly d: Decomposition) {
    this.xs = d.events.map((e) => e.v.x);
    this.slabs = [];
    for (let s = 0; s + 1 < this.xs.length; s++) {
      const mid = (this.xs[s] + this.xs[s + 1]) / 2;
      this.slabs.push(d.cells.filter((c) => c.xRange[0] <= mid && c.xRange[1] >= mid).map((c) => c.id));
    }
  }

  locate(q: Point2, tol = 1e-9): number | null {
    const xs = this.xs;
    if (q.x < xs[0] - tol || q.x > xs[xs.length - 1] + tol) return null;
    let lo = 0;
    let hi = xs.length - 1;
    while (hi - lo > 1) {
      const m = (lo + hi) >> 1;
      if (xs[m] <= q.x) lo = m;
      else hi = m;
    }
    const cand = new Set(this.slabs[Math.min(lo, this.slabs.length - 1)]);
    // On a slab wall the point may belong to the neighbouring slab's cells too.
    if (Math.abs(q.x - xs[lo]) <= tol && lo > 0) for (const c of this.slabs[lo - 1]) cand.add(c);
    if (Math.abs(q.x - xs[hi]) <= tol && hi < this.slabs.length) for (const c of this.slabs[hi]) cand.add(c);
    for (const id of cand) {
      const c = this.d.cells[id];
      if (q.x < c.xRange[0] - tol || q.x > c.xRange[1] + tol) continue;
      if (q.y >= chainY(c.floor, q.x) - tol && q.y <= chainY(c.ceiling, q.x) + tol) return id;
    }
    return null;
  }
}

/** Convenience: locate one point (builds the slab structure each call). */
export function locate(d: Decomposition, q: Point2): number | null {
  return new SlabLocator(d).locate(q);
}
