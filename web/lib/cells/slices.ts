/**
 * Morse decompositions for any slice function — Choset §6.2.2–6.2.3.
 *
 * Definition 6.2.1: the cells are the connected components of Q_free \ I*,
 * where I* is the set of slice intervals (components of h⁻¹(λ) ∩ Q_free) that
 * contain a critical point. For h = x on polygons the sweep in `sweep.ts`
 * computes this exactly. For the variable slices of §6.2.3 — circles
 * (spiral), diamonds (squarel), rays (spike), brushfire fronts, wave-fronts —
 * this module computes it on a raster, and computes it the same way for all of
 * them, which is the point of the definition: one theorem, six patterns.
 *
 * The raster construction is a discrete Reeb graph. Quantize h into bands of
 * width Δ; split each band into its 4-connected components (the slice
 * intervals, thickened by one band); join components of neighbouring bands
 * that touch. A component with exactly one neighbour below and one above is
 * *regular* — inside a cell the slice only stretches or shrinks (Derivation
 * 3). Every other component holds a critical point: nothing below is a birth,
 * nothing above a death, two below a merge, two above a split. Chains of
 * regular components between critical ones are the cells; the critical
 * components are I*. Pixel noise creates a few spurious stubs and two-band
 * loops; `simplify` prunes stubs shorter than `minBranch` bands and collapses
 * loops that short, which is a persistence threshold in disguise.
 *
 * The brushfire slice is Chapter 7's grid brushfire, the wave-front slice its
 * wave-front, read off the Chapter 7 labels directly (one band per label).
 */

import { brushfire, wavefront } from '../potential/brushfire';
import { morseData, type MorsePoint } from '../roadmap/morse';
import type { OccupancyGrid } from '../search/grid';
import type { Point2 } from '../sim/world';
import { isFreePoint, worldBounds, type CellWorld } from './world';

// ---------------------------------------------------------------------------
// Slice functions
// ---------------------------------------------------------------------------

export type SliceKind = 'vertical' | 'radial' | 'squarel' | 'spike' | 'brushfire' | 'wavefront';

export interface SliceSpec {
  kind: SliceKind;
  /** Centre for radial / squarel / spike; source for wave-front. */
  center?: Point2;
}

export const SLICE_LABELS: Record<SliceKind, { name: string; formula: string; pattern: string }> = {
  vertical: { name: 'boustrophedon', formula: 'h = x', pattern: 'back-and-forth laps' },
  radial: { name: 'spiral', formula: 'h = √(x² + y²)', pattern: 'concentric circles, or a spiral' },
  squarel: { name: 'squarel', formula: 'h = |x| + |y|', pattern: 'diamonds of straight laps' },
  spike: { name: 'spike', formula: 'h = atan(y/x)', pattern: 'rays from the centre' },
  brushfire: { name: 'brushfire', formula: 'h = D(x)', pattern: 'fronts at fixed distance from obstacles' },
  wavefront: { name: 'wave-front', formula: 'h = path length to a point', pattern: 'fronts of a tether' },
};

/** Analytic h for the smooth-ish slices (undefined for the grid ones). */
export function sliceValue(spec: SliceSpec, p: Point2): number {
  const c = spec.center ?? { x: 0, y: 0 };
  switch (spec.kind) {
    case 'vertical':
      return p.x;
    case 'radial':
      return Math.hypot(p.x - c.x, p.y - c.y);
    case 'squarel':
      return Math.abs(p.x - c.x) + Math.abs(p.y - c.y);
    case 'spike':
      return Math.atan2(p.y - c.y, p.x - c.x);
    default:
      throw new Error(`sliceValue: ${spec.kind} is a grid slice`);
  }
}

/**
 * The interior critical point of radial and squarel slices: their centre, a
 * minimum. h = |q − c| is not differentiable there, so we classify the smooth
 * surrogate h² (same level sets, same critical points) with Chapter 8's
 * `morseData`: a nondegenerate minimum of index 0.
 */
export function interiorCriticalPoints(world: CellWorld, spec: SliceSpec): MorsePoint[] {
  if ((spec.kind !== 'radial' && spec.kind !== 'squarel') || !spec.center) return [];
  const c = spec.center;
  if (!isFreePoint(world, c)) return [];
  return [morseData((q) => (q[0] - c.x) ** 2 + (q[1] - c.y) ** 2, [c.x, c.y])];
}

// ---------------------------------------------------------------------------
// Exact boundary critical points of an analytic slice on polygons
// ---------------------------------------------------------------------------

export interface BoundaryCritical {
  q: Point2;
  /** Local minimum or maximum of h along its boundary loop. */
  type: 'minimum' | 'maximum';
  /** −1 = outer boundary, otherwise obstacle index. */
  owner: number;
}

/**
 * Local extrema of h along each boundary loop, by dense sampling with a
 * golden-section polish — the connectivity-changing points of Derivation 3
 * (∇h ∥ ∇m, Lemma 5.5.2). On polygons they are vertices or, for curved
 * slices, the feet of tangency on edges.
 */
export function boundaryCriticalPoints(world: CellWorld, spec: SliceSpec, perEdge = 64): BoundaryCritical[] {
  const loops = [world.boundary, ...world.obstacles];
  const out: BoundaryCritical[] = [];
  loops.forEach((poly, li) => {
    const n = poly.length;
    const samples: { p: Point2; h: number }[] = [];
    for (let k = 0; k < n; k++) {
      const a = poly[k];
      const b = poly[(k + 1) % n];
      for (let s = 0; s < perEdge; s++) {
        const t = s / perEdge;
        const p = { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
        samples.push({ p, h: sliceValue(spec, p) });
      }
    }
    const m = samples.length;
    for (let i = 0; i < m; i++) {
      const h0 = samples[i].h;
      // Plateaus (a slice parallel to an edge) are degenerate: take the first sample.
      let jL = (i - 1 + m) % m;
      while (Math.abs(samples[jL].h - h0) < 1e-12 && jL !== i) jL = (jL - 1 + m) % m;
      let jR = (i + 1) % m;
      while (Math.abs(samples[jR].h - h0) < 1e-12 && jR !== i) jR = (jR + 1) % m;
      if (Math.abs(samples[(i - 1 + m) % m].h - h0) < 1e-12) continue; // not the first of a plateau
      const hl = samples[jL].h;
      const hr = samples[jR].h;
      if (spec.kind === 'spike' && (Math.abs(hl - h0) > 1 || Math.abs(hr - h0) > 1)) continue; // branch cut
      if (h0 < hl && h0 < hr) out.push({ q: samples[i].p, type: 'minimum', owner: li - 1 });
      else if (h0 > hl && h0 > hr) out.push({ q: samples[i].p, type: 'maximum', owner: li - 1 });
    }
  });
  return out;
}

// ---------------------------------------------------------------------------
// The raster
// ---------------------------------------------------------------------------

export interface FreeRaster {
  nx: number;
  ny: number;
  cell: number;
  /** Centre of pixel (i, j) is (x0 + (i + ½)·cell, y0 + (j + ½)·cell). */
  x0: number;
  y0: number;
  /** 1 where the pixel centre lies in cl(Q_free). */
  free: Uint8Array;
}

export function freeRaster(world: CellWorld, cell: number): FreeRaster {
  const b = worldBounds(world);
  // One pixel of padding so the outside is a ring around the free space.
  const x0 = b.minX - cell;
  const y0 = b.minY - cell;
  const nx = Math.ceil((b.maxX - b.minX) / cell) + 2;
  const ny = Math.ceil((b.maxY - b.minY) / cell) + 2;
  const free = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++) {
    for (let i = 0; i < nx; i++) {
      const p = { x: x0 + (i + 0.5) * cell, y: y0 + (j + 0.5) * cell };
      if (isFreePoint(world, p, 0)) free[j * nx + i] = 1;
    }
  }
  return { nx, ny, cell, x0, y0, free };
}

export const pixelCenter = (r: FreeRaster, k: number): Point2 => {
  const i = k % r.nx;
  const j = (k - i) / r.nx;
  return { x: r.x0 + (i + 0.5) * r.cell, y: r.y0 + (j + 0.5) * r.cell };
};

export function pixelAt(r: FreeRaster, p: Point2): number {
  const i = Math.floor((p.x - r.x0) / r.cell);
  const j = Math.floor((p.y - r.y0) / r.cell);
  if (i < 0 || j < 0 || i >= r.nx || j >= r.ny) return -1;
  return j * r.nx + i;
}

export function asOccupancy(r: FreeRaster): OccupancyGrid {
  const occupied = new Uint8Array(r.nx * r.ny);
  for (let k = 0; k < occupied.length; k++) occupied[k] = r.free[k] ? 0 : 1;
  return { nx: r.nx, ny: r.ny, cellSize: r.cell, origin: { x: r.x0, y: r.y0 }, occupied };
}

/** h on every free pixel (NaN elsewhere) and the band width that suits it. */
export function sliceField(world: CellWorld, r: FreeRaster, spec: SliceSpec): { h: Float64Array; band: number } {
  const h = new Float64Array(r.nx * r.ny).fill(NaN);
  if (spec.kind === 'brushfire' || spec.kind === 'wavefront') {
    const grid = asOccupancy(r);
    let dist;
    if (spec.kind === 'brushfire') dist = brushfire(grid, 8);
    else {
      const src = pixelAt(r, spec.center ?? { x: 0, y: 0 });
      let goal = src;
      if (goal < 0 || !r.free[goal]) {
        // Nearest free pixel to the requested source.
        let best = Infinity;
        for (let k = 0; k < r.free.length; k++) {
          if (!r.free[k]) continue;
          const c = pixelCenter(r, k);
          const d = Math.hypot(c.x - (spec.center?.x ?? 0), c.y - (spec.center?.y ?? 0));
          if (d < best) {
            best = d;
            goal = k;
          }
        }
      }
      dist = wavefront(grid, goal, 8);
    }
    for (let k = 0; k < h.length; k++) if (r.free[k] && dist.label[k] > 0) h[k] = dist.label[k];
    return { h, band: 1 };
  }
  for (let k = 0; k < h.length; k++) if (r.free[k]) h[k] = sliceValue(spec, pixelCenter(r, k));
  // A band must be thick enough to stay 4-connected where its level sets run
  // diagonally: one pixel for vertical slices, a few for curved ones.
  if (spec.kind === 'spike') {
    // Angular bands thin out toward the centre; size them so that even the
    // nearest free pixel's band is a couple of pixels wide.
    const c = spec.center ?? { x: 0, y: 0 };
    let rmin = Infinity;
    for (let k = 0; k < h.length; k++) if (r.free[k]) {
      const p = pixelCenter(r, k);
      rmin = Math.min(rmin, Math.hypot(p.x - c.x, p.y - c.y));
    }
    return { h, band: (1.5 * r.cell) / Math.max(rmin, r.cell) };
  }
  if (spec.kind === 'vertical') return { h, band: r.cell };
  return { h, band: 2.5 * r.cell };
}

// ---------------------------------------------------------------------------
// Discrete Reeb graph
// ---------------------------------------------------------------------------

export interface RasterNode {
  id: number;
  kind: 'birth' | 'death' | 'split' | 'merge' | 'mixed';
  /** Band components that make up this critical slice. */
  comps: number[];
  /** A representative point for drawing the critical point. */
  q: Point2;
  band: number;
}

export interface RasterArc {
  id: number;
  from: number;
  to: number;
  comps: number[];
  /** Number of bands the cell spans. */
  length: number;
}

export interface RasterMorse {
  raster: FreeRaster;
  spec: SliceSpec;
  /** Band index per pixel (−1 off the free space). */
  bandOf: Int32Array;
  /** Band component per pixel (−1 off the free space). */
  compOf: Int32Array;
  /** Cell (arc) per pixel; −1 on a critical slice or off Q_free. */
  cellOf: Int32Array;
  /** Critical node per pixel; −1 elsewhere. */
  nodeOf: Int32Array;
  nodes: RasterNode[];
  arcs: RasterArc[];
  /** Components of the obstacle set (outside included) — N_ob + 1. */
  obstacleComponents: number;
}

class DSU {
  p: Int32Array;
  constructor(n: number) {
    this.p = new Int32Array(n);
    for (let i = 0; i < n; i++) this.p[i] = i;
  }
  find(a: number): number {
    while (this.p[a] !== a) {
      this.p[a] = this.p[this.p[a]];
      a = this.p[a];
    }
    return a;
  }
  union(a: number, b: number) {
    const ra = this.find(a), rb = this.find(b);
    if (ra !== rb) this.p[ra] = rb;
  }
}

function obstacleComponentCount(r: FreeRaster): number {
  const dsu = new DSU(r.free.length);
  for (let j = 0; j < r.ny; j++) {
    for (let i = 0; i < r.nx; i++) {
      const k = j * r.nx + i;
      if (r.free[k]) continue;
      for (const [di, dj] of [[1, 0], [0, 1], [1, 1], [-1, 1]]) {
        const ni = i + di, nj = j + dj;
        if (ni < 0 || nj < 0 || ni >= r.nx || nj >= r.ny) continue;
        const m = nj * r.nx + ni;
        if (!r.free[m]) dsu.union(k, m);
      }
    }
  }
  const roots = new Set<number>();
  for (let k = 0; k < r.free.length; k++) if (!r.free[k]) roots.add(dsu.find(k));
  return roots.size;
}

export interface RasterMorseOptions {
  cell?: number;
  /** Prune birth/death stubs and collapse loops shorter than this many bands. */
  minBranch?: number;
}

/**
 * Morse decomposition of Q_free for the slice `spec`, on a raster of pixel
 * size `cell`. Returns the cells (arcs), the critical slices (nodes) and a
 * per-pixel labelling.
 */
export function rasterMorse(world: CellWorld, spec: SliceSpec, opts: RasterMorseOptions = {}): RasterMorse {
  const r = freeRaster(world, opts.cell ?? 0.05);
  const { h, band } = sliceField(world, r, spec);
  const N = r.nx * r.ny;
  let hmin = Infinity;
  for (let k = 0; k < N; k++) if (!Number.isNaN(h[k])) hmin = Math.min(hmin, h[k]);
  const bandOf = new Int32Array(N).fill(-1);
  for (let k = 0; k < N; k++) if (!Number.isNaN(h[k])) bandOf[k] = Math.floor((h[k] - hmin) / band + 1e-9);

  // Band components.
  const dsu = new DSU(N);
  for (let j = 0; j < r.ny; j++) {
    for (let i = 0; i < r.nx; i++) {
      const k = j * r.nx + i;
      if (bandOf[k] < 0) continue;
      if (i + 1 < r.nx && bandOf[k + 1] === bandOf[k]) dsu.union(k, k + 1);
      if (j + 1 < r.ny && bandOf[k + r.nx] === bandOf[k]) dsu.union(k, k + r.nx);
    }
  }
  const compOf = new Int32Array(N).fill(-1);
  const compIndex = new Map<number, number>();
  const compBand: number[] = [];
  const compPixels: number[][] = [];
  for (let k = 0; k < N; k++) {
    if (bandOf[k] < 0) continue;
    const root = dsu.find(k);
    let c = compIndex.get(root);
    if (c === undefined) {
      c = compBand.length;
      compIndex.set(root, c);
      compBand.push(bandOf[k]);
      compPixels.push([]);
    }
    compOf[k] = c;
    compPixels[c].push(k);
  }
  const C = compBand.length;
  const down: Set<number>[] = Array.from({ length: C }, () => new Set());
  const up: Set<number>[] = Array.from({ length: C }, () => new Set());
  const link = (a: number, b: number) => {
    const ca = compOf[a], cb = compOf[b];
    if (ca < 0 || cb < 0 || ca === cb) return;
    // The spike's branch cut: bands far apart are not neighbours.
    if (Math.abs(compBand[ca] - compBand[cb]) > (spec.kind === 'spike' ? 3 : 1e9)) return;
    if (compBand[ca] < compBand[cb]) {
      up[ca].add(cb);
      down[cb].add(ca);
    } else if (compBand[ca] > compBand[cb]) {
      up[cb].add(ca);
      down[ca].add(cb);
    }
  };
  for (let j = 0; j < r.ny; j++) {
    for (let i = 0; i < r.nx; i++) {
      const k = j * r.nx + i;
      if (bandOf[k] < 0) continue;
      if (i + 1 < r.nx) link(k, k + 1);
      if (j + 1 < r.ny) link(k, k + r.nx);
    }
  }

  // Graph on components: critical = not (one down and one up).
  const isCritical = (c: number) => !(down[c].size === 1 && up[c].size === 1);
  // Group adjacent critical components of the same event into one node (a
  // critical slice may break into several band components only through noise).
  const compNode = new Int32Array(C).fill(-1);
  const nodes: { comps: number[] }[] = [];
  for (let c = 0; c < C; c++) {
    if (!isCritical(c)) continue;
    compNode[c] = nodes.length;
    nodes.push({ comps: [c] });
  }
  // Arcs: walk chains of regular components between critical ones.
  interface Arc {
    from: number;
    to: number;
    comps: number[];
  }
  const arcs: Arc[] = [];
  const usedRegular = new Uint8Array(C);
  const usedDirect = new Set<string>();
  for (let c = 0; c < C; c++) {
    if (!isCritical(c)) continue;
    for (const start of [...up[c]]) {
      // Walk upward from critical c through `start`.
      if (isCritical(start)) {
        const key = `${c}>${start}`;
        if (usedDirect.has(key)) continue;
        usedDirect.add(key);
        arcs.push({ from: compNode[c], to: compNode[start], comps: [] });
        continue;
      }
      if (usedRegular[start]) continue;
      const chain: number[] = [];
      let cur = start;
      while (!isCritical(cur) && !usedRegular[cur]) {
        usedRegular[cur] = 1;
        chain.push(cur);
        cur = [...up[cur]][0];
      }
      arcs.push({ from: compNode[c], to: isCritical(cur) ? compNode[cur] : -1, comps: chain });
    }
  }
  // Regular chains that never touch a critical component (a slice that sweeps
  // a whole annulus) become loops with no node; there are none in our worlds,
  // but keep the pixels labelled.
  for (let c = 0; c < C; c++) {
    if (isCritical(c) || usedRegular[c]) continue;
    const chain: number[] = [];
    let cur = c;
    while (!isCritical(cur) && !usedRegular[cur]) {
      usedRegular[cur] = 1;
      chain.push(cur);
      cur = [...up[cur]][0];
    }
    arcs.push({ from: -1, to: isCritical(cur) ? compNode[cur] : -1, comps: chain });
  }

  // ---- simplify: prune short stubs, collapse short two-arc loops, contract degree-2 regular nodes.
  const minBranch = opts.minBranch ?? 3;
  const alive = arcs.map(() => true);
  const nodeAlive = nodes.map(() => true);
  const nodeExtra: number[][] = nodes.map(() => []); // comps absorbed into a node
  const span = (a: Arc) => a.comps.length + 1;
  const nodeArcs = (n: number) => arcs.map((a, i) => i).filter((i) => alive[i] && (arcs[i].from === n || arcs[i].to === n));
  const absorbNodeIntoArc = (n: number, ai: number) => {
    const comps = [...nodes[n].comps, ...nodeExtra[n]];
    arcs[ai].comps.push(...comps);
    nodeAlive[n] = false;
  };
  let changed = true;
  let guard = 0;
  while (changed && guard++ < 1000) {
    changed = false;
    // 1. stubs: a node with one arc whose span is short.
    for (let n = 0; n < nodes.length; n++) {
      if (!nodeAlive[n]) continue;
      const as = nodeArcs(n);
      if (as.length !== 1) continue;
      const a = arcs[as[0]];
      if (a.from === a.to) continue;
      if (span(a) >= minBranch) continue;
      const other = a.from === n ? a.to : a.from;
      if (other < 0) continue;
      // The stub and its node melt into the node at the other end.
      nodeExtra[other].push(...a.comps, ...nodes[n].comps, ...nodeExtra[n]);
      alive[as[0]] = false;
      nodeAlive[n] = false;
      changed = true;
    }
    // 2. short loops: two arcs joining the same pair of nodes, both short.
    for (let n = 0; n < nodes.length; n++) {
      if (!nodeAlive[n]) continue;
      const as = nodeArcs(n);
      for (let i = 0; i < as.length && !changed; i++) {
        for (let j = i + 1; j < as.length && !changed; j++) {
          const A = arcs[as[i]], B = arcs[as[j]];
          const oA = A.from === n ? A.to : A.from;
          const oB = B.from === n ? B.to : B.from;
          if (oA !== oB || oA === n || oA < 0) continue;
          if (span(A) >= minBranch || span(B) >= minBranch) continue;
          // Collapse: both arcs and node oA melt into n.
          nodeExtra[n].push(...A.comps, ...B.comps, ...nodes[oA].comps, ...nodeExtra[oA]);
          alive[as[i]] = false;
          alive[as[j]] = false;
          nodeAlive[oA] = false;
          for (let k = 0; k < arcs.length; k++) {
            if (!alive[k]) continue;
            if (arcs[k].from === oA) arcs[k].from = n;
            if (arcs[k].to === oA) arcs[k].to = n;
          }
          changed = true;
        }
      }
    }
    // 3. contract nodes left with exactly two arcs (no longer critical).
    for (let n = 0; n < nodes.length; n++) {
      if (!nodeAlive[n]) continue;
      const as = nodeArcs(n);
      if (as.length !== 2) continue;
      const [i, j] = as;
      if (i === j) continue;
      const A = arcs[i], B = arcs[j];
      // Only a pass-through: one arc arrives from below, the other leaves above.
      const throughAB = A.to === n && B.from === n && A.from !== n && B.to !== n;
      const throughBA = B.to === n && A.from === n && B.from !== n && A.to !== n;
      if (!throughAB && !throughBA) continue;
      const oA = A.from === n ? A.to : A.from;
      const oB = B.from === n ? B.to : B.from;
      // Merge B into A through n.
      A.comps.push(...nodes[n].comps, ...nodeExtra[n], ...B.comps);
      if (A.from === n) A.from = oB;
      else A.to = oB;
      void oA;
      alive[j] = false;
      nodeAlive[n] = false;
      changed = true;
    }
  }

  // Compact ids.
  const nodeId = new Int32Array(nodes.length).fill(-1);
  const outNodes: RasterNode[] = [];
  nodes.forEach((nd, n) => {
    if (!nodeAlive[n]) return;
    nodeId[n] = outNodes.length;
    const comps = [...nd.comps, ...nodeExtra[n]];
    const as = nodeArcs(n);
    let nDown = 0, nUp = 0;
    for (const ai of as) {
      const a = arcs[ai];
      // An arc leaving upward from this node starts at this node.
      if (a.from === n) nUp++;
      if (a.to === n) nDown++;
    }
    const kind: RasterNode['kind'] =
      nDown === 0 && nUp >= 1 ? 'birth' : nUp === 0 && nDown >= 1 ? 'death' : nDown === 1 && nUp === 2 ? 'split' : nDown === 2 && nUp === 1 ? 'merge' : 'mixed';
    outNodes.push({ id: outNodes.length, kind, comps, q: representative(r, compPixels, nd.comps, up, down, compOf), band: compBand[nd.comps[0]] });
  });
  const outArcs: RasterArc[] = [];
  arcs.forEach((a, i) => {
    if (!alive[i]) return;
    outArcs.push({ id: outArcs.length, from: a.from >= 0 ? nodeId[a.from] : -1, to: a.to >= 0 ? nodeId[a.to] : -1, comps: a.comps, length: a.comps.length + 1 });
  });

  const cellOf = new Int32Array(N).fill(-1);
  const nodeOf = new Int32Array(N).fill(-1);
  for (const a of outArcs) for (const c of a.comps) for (const k of compPixels[c]) cellOf[k] = a.id;
  for (const nd of outNodes) for (const c of nd.comps) for (const k of compPixels[c]) if (cellOf[k] < 0) nodeOf[k] = nd.id;

  return { raster: r, spec, bandOf, compOf, cellOf, nodeOf, nodes: outNodes, arcs: outArcs, obstacleComponents: obstacleComponentCount(r) };
}

/** Where to draw a critical point: the pinch of the critical slice, else its centroid. */
function representative(r: FreeRaster, compPixels: number[][], comps: number[], up: Set<number>[], down: Set<number>[], compOf: Int32Array): Point2 {
  const c0 = comps[0];
  const side = up[c0].size >= 2 ? up[c0] : down[c0].size >= 2 ? down[c0] : null;
  const pix = comps.flatMap((c) => compPixels[c]);
  if (side) {
    let best = -1;
    let bestScore = -1;
    for (const k of pix) {
      const i = k % r.nx;
      const j = (k - i) / r.nx;
      const seen = new Set<number>();
      for (let dj = -2; dj <= 2; dj++) {
        for (let di = -2; di <= 2; di++) {
          const ni = i + di, nj = j + dj;
          if (ni < 0 || nj < 0 || ni >= r.nx || nj >= r.ny) continue;
          const c = compOf[nj * r.nx + ni];
          if (c >= 0 && side.has(c)) seen.add(c);
        }
      }
      if (seen.size > bestScore) {
        bestScore = seen.size;
        best = k;
      }
    }
    if (best >= 0) return pixelCenter(r, best);
  }
  let x = 0, y = 0;
  for (const k of pix) {
    const p = pixelCenter(r, k);
    x += p.x;
    y += p.y;
  }
  return { x: x / pix.length, y: y / pix.length };
}

/** N_cp, N_ce and the obstacle count of a raster decomposition. */
export function rasterCounts(m: RasterMorse): { nCp: number; nCe: number; nOb: number } {
  return { nCp: m.nodes.length, nCe: m.arcs.length, nOb: m.obstacleComponents - 1 };
}

// ---------------------------------------------------------------------------
// Level sets for drawing the coverage pattern (marching squares)
// ---------------------------------------------------------------------------

/**
 * Segments of the level set h = λ inside the free space, by marching squares
 * on the pixel-centre lattice (a square is used only if its four corners are
 * free). These are the laps of the variable-slice patterns of §6.2.3.
 */
export function levelSegments(world: CellWorld, r: FreeRaster, spec: SliceSpec, levels: readonly number[], h?: Float64Array): [Point2, Point2][] {
  const field = h ?? sliceField(world, r, spec).h;
  const out: [Point2, Point2][] = [];
  const P = (i: number, j: number) => ({ x: r.x0 + (i + 0.5) * r.cell, y: r.y0 + (j + 0.5) * r.cell });
  for (let j = 0; j + 1 < r.ny; j++) {
    for (let i = 0; i + 1 < r.nx; i++) {
      const k00 = j * r.nx + i, k10 = k00 + 1, k01 = k00 + r.nx, k11 = k01 + 1;
      const v = [field[k00], field[k10], field[k11], field[k01]];
      if (v.some((x) => Number.isNaN(x))) continue;
      if (spec.kind === 'spike' && Math.max(...v) - Math.min(...v) > 1) continue;
      const corners = [P(i, j), P(i + 1, j), P(i + 1, j + 1), P(i, j + 1)];
      for (const lam of levels) {
        const pts: Point2[] = [];
        for (let e = 0; e < 4; e++) {
          const a = v[e], b = v[(e + 1) % 4];
          if ((a < lam && b >= lam) || (a >= lam && b < lam)) {
            const t = (lam - a) / (b - a);
            const A = corners[e], B = corners[(e + 1) % 4];
            pts.push({ x: A.x + (B.x - A.x) * t, y: A.y + (B.y - A.y) * t });
          }
        }
        if (pts.length === 2) out.push([pts[0], pts[1]]);
        else if (pts.length === 4) {
          out.push([pts[0], pts[1]]);
          out.push([pts[2], pts[3]]);
        }
      }
    }
  }
  return out;
}
