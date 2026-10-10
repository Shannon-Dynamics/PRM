/**
 * The pursuit/evasion coda — Choset §6.3 (Suzuki & Yamashita; LaValle,
 * Guibas and coworkers).
 *
 * Point pursuers search a polygonal free space for a point evader that is
 * arbitrarily fast and knows everything. A pursuer at x sees its visibility
 * polygon V(x); the evader is caught the moment it is inside some V(γ_i(t)).
 * Everything else is *contaminated* (may hold the evader) or *clear*. The
 * edges of V(x) that lie in free space are the gap edges; each carries a label,
 * 0 if the region behind it is clear and 1 if it is contaminated, and
 * B(x) — the vector of labels — together with x is the information state.
 *
 * This module computes, in the workspace (C Problem 7: the agents are points,
 * so W_free is the space they move in):
 *
 *   - `visibilityPolygon`: V(x) by an angular sweep of rays through every
 *     vertex (and a hair to either side), each ray cast exactly; checked
 *     against brute-force ray sampling.
 *   - `gapsOf`: the gap edges of V(x), each anchored at the reflex vertex that
 *     makes it, with the side its shadow lies on.
 *   - `ContaminationField`: the information state as a raster — visible pixels
 *     are cleared; contamination then floods every hidden pixel it can reach,
 *     because the evader is arbitrarily fast. Gap labels are read off it. The
 *     four transition rules of §6.3 (a gap disappears: nothing; appears: 0;
 *     merges: OR; splits: copy) are *consequences* of this flood, not code.
 *   - `conservativeRegions` and `solveInformationGraph`: the cell
 *     decomposition by information. Pixels are grouped by which polygon
 *     vertices they cannot see (a refinement of Choset's conservative regions:
 *     B(x) cannot change inside one); the information graph G_i has a node per
 *     (region, labels) and an edge per region crossing, with the labels
 *     carried across by overlap of the shadows on either side; breadth-first
 *     search looks for a node whose labels are all zero. A second pursuer, if
 *     given, is static — Choset's "static pursuer" of the O(log n) argument.
 *     `replayPursuit` then drives the found path through the raster flood and
 *     confirms that nothing contaminated is left.
 */

import { rayCast, type Point2, type World } from '../sim/world';
import { freeRaster, pixelCenter, pixelAt, type FreeRaster } from './slices';
import { asWorld, polygonsOf, type CellWorld } from './world';

const TAU = 2 * Math.PI;
const dist = (a: Point2, b: Point2) => Math.hypot(a.x - b.x, a.y - b.y);

// ---------------------------------------------------------------------------
// Visibility polygon
// ---------------------------------------------------------------------------

export interface VisibilityPolygon {
  x: Point2;
  /** Boundary points in increasing angle. */
  pts: Point2[];
  angles: number[];
  ranges: number[];
  /** For each point, the world vertex its ray was aimed at (−1 if none). */
  vertexOf: number[];
}

/** The vertices of a polyline world, deduplicated. */
export function worldVertices(world: World): Point2[] {
  const out: Point2[] = [];
  const seen = new Set<string>();
  for (const s of world.walls) {
    for (const p of [{ x: s.x1, y: s.y1 }, { x: s.x2, y: s.y2 }]) {
      const k = `${p.x.toFixed(9)},${p.y.toFixed(9)}`;
      if (!seen.has(k)) {
        seen.add(k);
        out.push(p);
      }
    }
  }
  return out;
}

/**
 * V(x) by angular sweep: cast a ray at every vertex's bearing and at ± eps
 * around it. Between consecutive bearings the nearest wall is one and the same
 * segment (no vertex in between), so joining the hits gives the exact polygon.
 */
export function visibilityPolygon(world: World, x: Point2, vertices = worldVertices(world), eps = 1e-5, far = 1e3): VisibilityPolygon {
  const rays: { a: number; v: number }[] = [];
  vertices.forEach((v, i) => {
    const a = Math.atan2(v.y - x.y, v.x - x.x);
    rays.push({ a: a - eps, v: -1 }, { a, v: i }, { a: a + eps, v: -1 });
  });
  rays.forEach((r) => (r.a = ((r.a % TAU) + TAU) % TAU));
  rays.sort((p, q) => p.a - q.a);
  const pts: Point2[] = [];
  const angles: number[] = [];
  const ranges: number[] = [];
  const vertexOf: number[] = [];
  for (const r of rays) {
    const d = rayCast(world, x.x, x.y, r.a, far);
    pts.push({ x: x.x + d * Math.cos(r.a), y: x.y + d * Math.sin(r.a) });
    angles.push(r.a);
    ranges.push(d);
    vertexOf.push(r.v);
  }
  return { x, pts, angles, ranges, vertexOf };
}

/** Range of the polygon boundary along bearing φ (ray–polygon intersection; V(x) is star-shaped from x). */
export function polygonRange(vp: VisibilityPolygon, phi: number): number {
  const n = vp.pts.length;
  let best = Infinity;
  const dx = Math.cos(phi), dy = Math.sin(phi);
  for (let i = 0; i < n; i++) {
    const a = vp.pts[i], b = vp.pts[(i + 1) % n];
    const sx = b.x - a.x, sy = b.y - a.y;
    const den = dx * sy - dy * sx;
    if (Math.abs(den) < 1e-15) continue;
    const ex = a.x - vp.x.x, ey = a.y - vp.x.y;
    const t = (ex * sy - ey * sx) / den;
    const u = (ex * dy - ey * dx) / den;
    if (t >= 0 && u >= -1e-9 && u <= 1 + 1e-9 && t < best) best = t;
  }
  return best;
}

/** Brute force: sample bearings, compare the polygon's range with a fresh ray cast. */
export function visibilityAgreement(world: World, vp: VisibilityPolygon, nRays = 3600): { worst: number; samples: number } {
  let worst = 0;
  let samples = 0;
  for (let k = 0; k < nRays; k++) {
    const phi = ((k + 0.5) / nRays) * TAU;
    // Skip bearings within a hair of a vertex ray, where the range is discontinuous.
    if (vp.angles.some((a) => Math.abs(((phi - a + Math.PI) % TAU + TAU) % TAU - Math.PI) < 1e-4)) continue;
    const truth = rayCast(world, vp.x.x, vp.x.y, phi, 1e3);
    const poly = polygonRange(vp, phi);
    worst = Math.max(worst, Math.abs(truth - poly));
    samples++;
  }
  return { worst, samples };
}

export function polygonAreaOf(pts: readonly Point2[]): number {
  let A = 0;
  for (let i = 0; i < pts.length; i++) {
    const p = pts[i], q = pts[(i + 1) % pts.length];
    A += p.x * q.y - q.x * p.y;
  }
  return Math.abs(A) / 2;
}

// ---------------------------------------------------------------------------
// Gap edges
// ---------------------------------------------------------------------------

export interface Gap {
  /** The reflex vertex the gap hangs from (index into the world vertices). */
  vertex: number;
  near: Point2;
  far: Point2;
  /** Unit normal of the gap edge pointing into its shadow. */
  shadowSide: Point2;
  angle: number;
}

/** Gap edges: consecutive rays around a vertex whose ranges jump by more than `tol`. */
export function gapsOf(vp: VisibilityPolygon, tol = 1e-3): Gap[] {
  const n = vp.pts.length;
  const out: Gap[] = [];
  for (let i = 0; i < n; i++) {
    if (vp.vertexOf[i] < 0) continue;
    const prev = (i - 1 + n) % n, next = (i + 1) % n;
    const r0 = vp.ranges[i];
    const rp = vp.ranges[prev], rn = vp.ranges[next];
    // The ray at the vertex stops at the vertex only if the vertex is visible.
    const v = vp.pts[i];
    for (const [j, rj, sgn] of [
      [prev, rp, -1],
      [next, rn, 1],
    ] as const) {
      if (rj > r0 + tol) {
        // The shadow lies on the side of the short ray, behind the vertex.
        const a = vp.angles[i];
        const d = { x: Math.cos(a), y: Math.sin(a) };
        const ccw = { x: -d.y, y: d.x };
        const toward = sgn === 1 ? -1 : 1; // the far ray is on side sgn; the shadow on the other
        out.push({ vertex: vp.vertexOf[i], near: v, far: vp.pts[j], shadowSide: { x: toward * ccw.x, y: toward * ccw.y }, angle: a });
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// The information state as a raster
// ---------------------------------------------------------------------------

/** Pixels of the free raster visible from x, by one exact ray cast per pixel centre (the reference). */
export function visibleMaskByRays(world: World, r: FreeRaster, x: Point2, out: Uint8Array = new Uint8Array(r.free.length)): Uint8Array {
  out.fill(0);
  for (let k = 0; k < r.free.length; k++) {
    if (!r.free[k]) continue;
    const p = pixelCenter(r, k);
    const d = dist(p, x);
    if (d < 1e-9) {
      out[k] = 1;
      continue;
    }
    const a = Math.atan2(p.y - x.y, p.x - x.x);
    if (rayCast(world, x.x, x.y, a, d) >= d - 1e-9) out[k] = 1;
  }
  return out;
}

/**
 * The same mask from V(x): V(x) is star-shaped about x, so a pixel is visible
 * iff it is no farther than the polygon along its bearing — a binary search on
 * the polygon's bearings and one segment intersection, O(log n) per pixel
 * instead of a ray cast against every wall.
 */
export function visibleMask(world: World, r: FreeRaster, x: Point2, out: Uint8Array = new Uint8Array(r.free.length), vp?: VisibilityPolygon): Uint8Array {
  const poly = vp ?? visibilityPolygon(world, x);
  const A = poly.angles;
  const n = A.length;
  out.fill(0);
  for (let k = 0; k < r.free.length; k++) {
    if (!r.free[k]) continue;
    const p = pixelCenter(r, k);
    const dx = p.x - x.x, dy = p.y - x.y;
    const d = Math.hypot(dx, dy);
    if (d < 1e-9) {
      out[k] = 1;
      continue;
    }
    let a = Math.atan2(dy, dx);
    if (a < 0) a += TAU;
    // Largest i with A[i] ≤ a (wrapping below the first bearing).
    let lo = 0, hi = n - 1, i = n - 1;
    if (a >= A[0]) {
      while (lo < hi) {
        const m = (lo + hi + 1) >> 1;
        if (A[m] <= a) lo = m;
        else hi = m - 1;
      }
      i = lo;
    }
    const P = poly.pts[i], Q = poly.pts[(i + 1) % n];
    const ux = dx / d, uy = dy / d;
    const sx = Q.x - P.x, sy = Q.y - P.y;
    const den = ux * sy - uy * sx;
    let R: number;
    if (Math.abs(den) < 1e-15) R = Math.min(dist(P, x), dist(Q, x));
    else R = ((P.x - x.x) * sy - (P.y - x.y) * sx) / den;
    if (d <= R + 1e-9) out[k] = 1;
  }
  return out;
}

/** 4-connected components of the hidden free pixels: the shadows. */
export function shadowComponents(r: FreeRaster, visible: Uint8Array): { comp: Int32Array; count: number; sizes: number[] } {
  const comp = new Int32Array(r.free.length).fill(-1);
  const sizes: number[] = [];
  let count = 0;
  const stack: number[] = [];
  for (let k = 0; k < r.free.length; k++) {
    if (!r.free[k] || visible[k] || comp[k] >= 0) continue;
    comp[k] = count;
    stack.push(k);
    let size = 0;
    while (stack.length) {
      const q = stack.pop()!;
      size++;
      const i = q % r.nx;
      const j = (q - i) / r.nx;
      const nb = [i > 0 ? q - 1 : -1, i + 1 < r.nx ? q + 1 : -1, j > 0 ? q - r.nx : -1, j + 1 < r.ny ? q + r.nx : -1];
      for (const m of nb) {
        if (m < 0 || !r.free[m] || visible[m] || comp[m] >= 0) continue;
        comp[m] = count;
        stack.push(m);
      }
    }
    sizes.push(size);
    count++;
  }
  return { comp, count, sizes };
}

/**
 * The contamination field. `contaminated[k] = 1` where the evader may be.
 * Moving the pursuers clears what they see; then contamination floods every
 * hidden pixel connected to a contaminated one — recontamination included.
 */
export class ContaminationField {
  readonly world: World;
  readonly raster: FreeRaster;
  readonly vertices: Point2[];
  contaminated: Uint8Array;
  private vis: Uint8Array;
  private vis2: Uint8Array;

  constructor(
    readonly cw: CellWorld,
    cell = 0.125,
    raster?: FreeRaster,
  ) {
    this.world = asWorld(cw);
    this.raster = raster ?? freeRaster(cw, cell);
    this.vertices = worldVertices(this.world);
    this.contaminated = new Uint8Array(this.raster.free.length);
    this.vis = new Uint8Array(this.raster.free.length);
    this.vis2 = new Uint8Array(this.raster.free.length);
    this.reset();
  }

  /** Everything contaminated: the evader could be anywhere. */
  reset() {
    this.contaminated.set(this.raster.free);
  }

  /** Pursuers stand at `xs`: clear what they see, then let contamination flood. */
  observe(xs: readonly Point2[]): Uint8Array {
    const r = this.raster;
    const vis = this.vis;
    vis.fill(0);
    for (const x of xs) {
      visibleMask(this.world, r, x, this.vis2);
      for (let k = 0; k < vis.length; k++) if (this.vis2[k]) vis[k] = 1;
    }
    const c = this.contaminated;
    for (let k = 0; k < c.length; k++) if (vis[k]) c[k] = 0;
    // Flood through hidden pixels.
    const stack: number[] = [];
    for (let k = 0; k < c.length; k++) if (c[k]) stack.push(k);
    while (stack.length) {
      const q = stack.pop()!;
      const i = q % r.nx;
      const j = (q - i) / r.nx;
      const nb = [i > 0 ? q - 1 : -1, i + 1 < r.nx ? q + 1 : -1, j > 0 ? q - r.nx : -1, j + 1 < r.ny ? q + r.nx : -1];
      for (const m of nb) {
        if (m < 0 || !r.free[m] || vis[m] || c[m]) continue;
        c[m] = 1;
        stack.push(m);
      }
    }
    return vis;
  }

  /**
   * Move pursuers in straight lines from `from` to `to`, observing every
   * `step` metres — the evader is fast, so the flood runs at every step.
   */
  move(from: readonly Point2[], to: readonly Point2[], step = this.raster.cell / 2): void {
    const L = Math.max(...from.map((p, i) => dist(p, to[i])));
    const n = Math.max(1, Math.ceil(L / step));
    for (let k = 1; k <= n; k++) {
      const t = k / n;
      this.observe(from.map((p, i) => ({ x: p.x + (to[i].x - p.x) * t, y: p.y + (to[i].y - p.y) * t })));
    }
  }

  /** Visibility mask of the most recent `observe` (the pursuers' current view). */
  get lastVisible(): Uint8Array {
    return this.vis;
  }

  contaminatedArea(): number {
    let n = 0;
    for (let k = 0; k < this.contaminated.length; k++) n += this.contaminated[k];
    return n * this.raster.cell * this.raster.cell;
  }

  /** The label of a gap: is the shadow just behind it contaminated? */
  gapLabel(g: Gap, visible: Uint8Array): 0 | 1 {
    const r = this.raster;
    for (const t of [0.15, 0.35, 0.5, 0.65, 0.85]) {
      for (const off of [1.0, 1.8]) {
        const p = {
          x: g.near.x + (g.far.x - g.near.x) * t + g.shadowSide.x * off * r.cell,
          y: g.near.y + (g.far.y - g.near.y) * t + g.shadowSide.y * off * r.cell,
        };
        const k = pixelAt(r, p);
        if (k >= 0 && r.free[k] && !visible[k] && this.contaminated[k]) return 1;
      }
    }
    return 0;
  }
}

// ---------------------------------------------------------------------------
// Conservative regions and the information graph
// ---------------------------------------------------------------------------

export interface ConservativeRegion {
  id: number;
  pixels: number[];
  /** A pixel near the region's middle. */
  rep: number;
  centroid: Point2;
  /** Which vertices the moving pursuer cannot see from here (sorted indices). */
  hidden: number[];
}

export interface RegionDecomposition {
  raster: FreeRaster;
  world: World;
  regionOf: Int32Array;
  regions: ConservativeRegion[];
  /** Adjacent region pairs with one witnessing pixel pair (a in region A, b in region B). */
  adjacency: { A: number; B: number; a: number; b: number }[];
  /** Vertices used in signatures: the world's, plus the static pursuer's view polygon corners. */
  keyPoints: Point2[];
  staticPursuer: Point2 | null;
}

/** Is q visible from p (the straight segment stays in the free space)? */
function sees(world: World, p: Point2, q: Point2): boolean {
  const d = dist(p, q);
  if (d < 1e-12) return true;
  return rayCast(world, p.x, p.y, Math.atan2(q.y - p.y, q.x - p.x), d) >= d - 1e-7;
}

/**
 * Group free pixels by the set of key points they cannot see. Inside one group
 * no gap appears, disappears, merges or splits — so B(x) is constant there:
 * these regions refine Choset's conservative regions (Fig. 6.33–6.35).
 */
export function conservativeRegions(cw: CellWorld, cell = 0.125, staticPursuer: Point2 | null = null, raster?: FreeRaster): RegionDecomposition {
  const world = asWorld(cw);
  const r = raster ?? freeRaster(cw, cell);
  const keyPoints = worldVertices(world).slice();
  if (staticPursuer) {
    const vp = visibilityPolygon(world, staticPursuer);
    for (const p of vp.pts) keyPoints.push(p);
  }
  // Nudge key points a hair into free space so a ray to a vertex is not stopped by its own walls.
  const sig = new Map<string, number>();
  const sigOf = new Int32Array(r.free.length).fill(-1);
  const sigHidden: number[][] = [];
  for (let k = 0; k < r.free.length; k++) {
    if (!r.free[k]) continue;
    const p = pixelCenter(r, k);
    const hidden: number[] = [];
    keyPoints.forEach((v, i) => {
      if (!sees(world, p, v)) hidden.push(i);
    });
    const key = hidden.join(',');
    let id = sig.get(key);
    if (id === undefined) {
      id = sigHidden.length;
      sig.set(key, id);
      sigHidden.push(hidden);
    }
    sigOf[k] = id;
  }
  // Connected components of equal signature.
  const regionOf = new Int32Array(r.free.length).fill(-1);
  const regions: ConservativeRegion[] = [];
  for (let k = 0; k < r.free.length; k++) {
    if (!r.free[k] || regionOf[k] >= 0) continue;
    const id = regions.length;
    const pixels: number[] = [];
    const stack = [k];
    regionOf[k] = id;
    while (stack.length) {
      const q = stack.pop()!;
      pixels.push(q);
      const i = q % r.nx;
      const j = (q - i) / r.nx;
      const nb = [i > 0 ? q - 1 : -1, i + 1 < r.nx ? q + 1 : -1, j > 0 ? q - r.nx : -1, j + 1 < r.ny ? q + r.nx : -1];
      for (const m of nb) {
        if (m < 0 || !r.free[m] || regionOf[m] >= 0 || sigOf[m] !== sigOf[k]) continue;
        regionOf[m] = id;
        stack.push(m);
      }
    }
    let cx = 0, cy = 0;
    for (const q of pixels) {
      const p = pixelCenter(r, q);
      cx += p.x;
      cy += p.y;
    }
    const centroid = { x: cx / pixels.length, y: cy / pixels.length };
    let rep = pixels[0];
    let bd = Infinity;
    for (const q of pixels) {
      const d = dist(pixelCenter(r, q), centroid);
      if (d < bd) {
        bd = d;
        rep = q;
      }
    }
    regions.push({ id, pixels, rep, centroid, hidden: sigHidden[sigOf[k]] });
  }
  const seen = new Set<string>();
  const adjacency: RegionDecomposition['adjacency'] = [];
  for (let k = 0; k < r.free.length; k++) {
    if (regionOf[k] < 0) continue;
    const i = k % r.nx;
    for (const m of [i + 1 < r.nx ? k + 1 : -1, k + r.nx < r.free.length ? k + r.nx : -1]) {
      if (m < 0 || regionOf[m] < 0 || regionOf[m] === regionOf[k]) continue;
      const A = regionOf[k], B = regionOf[m];
      const key = A < B ? `${A},${B}` : `${B},${A}`;
      if (seen.has(key)) continue;
      seen.add(key);
      adjacency.push({ A, B, a: k, b: m });
    }
  }
  return { raster: r, world, regionOf, regions, adjacency, keyPoints, staticPursuer };
}

interface ShadowInfo {
  comp: Int32Array;
  count: number;
  /** For each component, the hidden key points that fall in it. */
  keysOf: number[][];
  /** For each hidden key point, its component (−1 if no pixel of its shadow was found). */
  compOfKey: Map<number, number>;
}

function shadowsAt(dec: RegionDecomposition, k: number, staticMask: Uint8Array | null, keyPixels: number[][]): ShadowInfo {
  const r = dec.raster;
  const x = pixelCenter(r, k);
  const vis = visibleMask(dec.world, r, x);
  if (staticMask) for (let q = 0; q < vis.length; q++) if (staticMask[q]) vis[q] = 1;
  const { comp, count } = shadowComponents(r, vis);
  const keysOf: number[][] = Array.from({ length: count }, () => []);
  const compOfKey = new Map<number, number>();
  for (const h of dec.regions[dec.regionOf[k]].hidden) {
    let c = -1;
    for (const q of keyPixels[h]) if (comp[q] >= 0) {
      c = comp[q];
      break;
    }
    compOfKey.set(h, c);
    if (c >= 0) keysOf[c].push(h);
  }
  return { comp, count, keysOf, compOfKey };
}

export interface PursuitSolution {
  found: boolean;
  /** Information-graph nodes discovered by the search. */
  statesExplored: number;
  /** Regions visited along the solution. */
  regionPath: number[];
  /** Pixel-by-pixel pursuer path (centres), for the replay. */
  path: Point2[];
}

/**
 * Breadth-first search of the information graph G_i from (start region, all
 * labels 1) to any node with all labels 0. Labels live on the hidden key
 * points of each region (every key point inside one shadow shares its
 * shadow's label); a crossing A → B carries labels by overlap: a shadow at b
 * is contaminated iff it overlaps a contaminated shadow at a.
 */
export function solveInformationGraph(dec: RegionDecomposition, start: Point2, maxStates = 200000): PursuitSolution {
  const r = dec.raster;
  const staticMask = dec.staticPursuer ? visibleMask(dec.world, r, dec.staticPursuer) : null;
  // For each key point, free pixels next to it (where its shadow would be seen).
  const keyPixels: number[][] = dec.keyPoints.map((v) => {
    const out: { k: number; d: number }[] = [];
    const i0 = Math.floor((v.x - r.x0) / r.cell), j0 = Math.floor((v.y - r.y0) / r.cell);
    for (let dj = -2; dj <= 2; dj++) for (let di = -2; di <= 2; di++) {
      const i = i0 + di, j = j0 + dj;
      if (i < 0 || j < 0 || i >= r.nx || j >= r.ny) continue;
      const k = j * r.nx + i;
      if (!r.free[k]) continue;
      out.push({ k, d: dist(pixelCenter(r, k), v) });
    }
    return out.sort((a, b) => a.d - b.d).map((o) => o.k);
  });
  const nbrs = dec.regions.map(() => [] as { to: number; a: number; b: number }[]);
  for (const e of dec.adjacency) {
    nbrs[e.A].push({ to: e.B, a: e.a, b: e.b });
    nbrs[e.B].push({ to: e.A, a: e.b, b: e.a });
  }
  const cache = new Map<string, { sa: ShadowInfo; sb: ShadowInfo; overlap: Set<number>[] }>();
  const transition = (a: number, b: number) => {
    const key = `${a}>${b}`;
    let t = cache.get(key);
    if (!t) {
      const sa = shadowsAt(dec, a, staticMask, keyPixels);
      const sb = shadowsAt(dec, b, staticMask, keyPixels);
      const overlap = Array.from({ length: sb.count }, () => new Set<number>());
      for (let q = 0; q < r.free.length; q++) if (sb.comp[q] >= 0 && sa.comp[q] >= 0) overlap[sb.comp[q]].add(sa.comp[q]);
      t = { sa, sb, overlap };
      cache.set(key, t);
    }
    return t;
  };
  const startPix = pixelAt(r, start);
  const startRegion = dec.regionOf[startPix];
  if (startRegion < 0) return { found: false, statesExplored: 0, regionPath: [], path: [] };
  const labelsKey = (labels: Map<number, number>) => [...labels.entries()].sort((p, q) => p[0] - q[0]).map(([k, v]) => `${k}:${v}`).join(',');
  const initial = new Map<number, number>(dec.regions[startRegion].hidden.map((h) => [h, 1]));
  interface Node {
    region: number;
    labels: Map<number, number>;
    parent: number;
    via: { a: number; b: number } | null;
  }
  const nodes: Node[] = [{ region: startRegion, labels: initial, parent: -1, via: null }];
  const seen = new Set<string>([`${startRegion}|${labelsKey(initial)}`]);
  let goal = -1;
  for (let head = 0; head < nodes.length && nodes.length < maxStates; head++) {
    const nd = nodes[head];
    if ([...nd.labels.values()].every((v) => v === 0)) {
      goal = head;
      break;
    }
    for (const e of nbrs[nd.region]) {
      const t = transition(e.a, e.b);
      // Shadow labels at a.
      const la = t.sa.keysOf.map((ks) => (ks.length === 0 ? 1 : ks.some((h) => (nd.labels.get(h) ?? 1) === 1) ? 1 : 0));
      const lb = t.overlap.map((set) => ([...set].some((c) => la[c] === 1) ? 1 : 0));
      const labels = new Map<number, number>();
      for (const h of dec.regions[e.to].hidden) {
        const c = t.sb.compOfKey.get(h) ?? -1;
        labels.set(h, c >= 0 ? lb[c] : 1);
      }
      const key = `${e.to}|${labelsKey(labels)}`;
      if (seen.has(key)) continue;
      seen.add(key);
      nodes.push({ region: e.to, labels, parent: head, via: { a: e.a, b: e.b } });
    }
  }
  if (goal < 0) return { found: false, statesExplored: nodes.length, regionPath: [], path: [] };
  // Reconstruct: regions and the pixel path through them.
  const chain: Node[] = [];
  for (let i = goal; i >= 0; i = nodes[i].parent) chain.push(nodes[i]);
  chain.reverse();
  const regionPath = chain.map((n) => n.region);
  const path: Point2[] = [];
  let cur = startPix;
  path.push(pixelCenter(r, cur));
  for (let i = 1; i < chain.length; i++) {
    const via = chain[i].via!;
    // Walk inside the current region to pixel a, then step to b.
    const seg = pixelRoute(dec, cur, via.a, dec.regionOf[cur]);
    for (const q of seg.slice(1)) path.push(pixelCenter(r, q));
    path.push(pixelCenter(r, via.b));
    cur = via.b;
  }
  return { found: true, statesExplored: nodes.length, regionPath, path };
}

/** BFS over 4-adjacent pixels of one region. */
function pixelRoute(dec: RegionDecomposition, from: number, to: number, region: number): number[] {
  if (from === to) return [from];
  const r = dec.raster;
  const parent = new Map<number, number>([[from, -1]]);
  const queue = [from];
  for (let h = 0; h < queue.length; h++) {
    const q = queue[h];
    if (q === to) break;
    const i = q % r.nx;
    const j = (q - i) / r.nx;
    for (const m of [i > 0 ? q - 1 : -1, i + 1 < r.nx ? q + 1 : -1, j > 0 ? q - r.nx : -1, j + 1 < r.ny ? q + r.nx : -1]) {
      if (m < 0 || parent.has(m) || dec.regionOf[m] !== region) continue;
      parent.set(m, q);
      queue.push(m);
    }
  }
  if (!parent.has(to)) return [from, to];
  const out: number[] = [];
  for (let q = to; q !== -1; q = parent.get(q)!) out.push(q);
  return out.reverse();
}

/** Drive a pursuer along `path` (with an optional static partner) and return the contaminated area left. */
export function replayPursuit(cw: CellWorld, path: readonly Point2[], staticPursuer: Point2 | null = null, cell = 0.125, raster?: FreeRaster): { area: number; field: ContaminationField } {
  const f = new ContaminationField(cw, cell, raster);
  const withStatic = (p: Point2) => (staticPursuer ? [p, staticPursuer] : [p]);
  if (path.length) f.observe(withStatic(path[0]));
  for (let i = 1; i < path.length; i++) f.move(withStatic(path[i - 1]), withStatic(path[i]));
  return { area: f.contaminatedArea(), field: f };
}

// ---------------------------------------------------------------------------
// Worlds: a hub with U-shaped pockets (C Fig. 6.40's U-world, in miniature)
// ---------------------------------------------------------------------------

/** One U-pocket in local coordinates: opening (0,0)–(1,0), first leg up, hairpin, second leg down. */
const POCKET_LOCAL: Point2[] = [
  { x: 1, y: 0 },
  { x: 1, y: 3 },
  { x: 2, y: 3 },
  { x: 2, y: 1 },
  { x: 3, y: 1 },
  { x: 3, y: 4 },
  { x: 0, y: 4 },
  { x: 0, y: 0 },
];

/**
 * A square hub [−2, 2]² with U-pockets on the chosen sides (right, top, left).
 * From the hub one sees each pocket's first leg but never its far leg; to clear
 * a pocket a pursuer must walk to the hairpin, and while it is there it cannot
 * watch the hub.
 */
export function pocketWorld(sides: readonly ('right' | 'top' | 'left')[]): CellWorld {
  const pts: Point2[] = [{ x: -2, y: -2 }, { x: 2, y: -2 }];
  const place = (O: Point2, e1: Point2, e2: Point2) => POCKET_LOCAL.forEach((p) => pts.push({ x: O.x + p.x * e1.x + p.y * e2.x, y: O.y + p.x * e1.y + p.y * e2.y }));
  if (sides.includes('right')) place({ x: 2, y: 0.5 }, { x: 0, y: -1 }, { x: 1, y: 0 });
  pts.push({ x: 2, y: 2 });
  if (sides.includes('top')) place({ x: -0.5, y: 2 }, { x: 1, y: 0 }, { x: 0, y: 1 });
  pts.push({ x: -2, y: 2 });
  if (sides.includes('left')) place({ x: -2, y: -0.5 }, { x: 0, y: 1 }, { x: -1, y: 0 });
  const name = `${sides.length}-pocket world`;
  return { name, boundary: pts, obstacles: [] };
}

export const TWO_POCKETS = pocketWorld(['right', 'top']);
export const THREE_POCKETS = pocketWorld(['right', 'top', 'left']);
/** Where the pursuer starts and where a second, static pursuer stands. */
export const PURSUER_START: Point2 = { x: 0.06, y: -1.06 };
export const STATIC_PURSUER: Point2 = { x: 0.06, y: 0.06 };

/** Edge count of a pursuit world, for the O(log n) discussion. */
export const edgeCount = (cw: CellWorld) => polygonsOf(cw).reduce((s, p) => s + p.length, 0);

/** Shortest 4-connected route over free pixels between two points (pixel centres). */
export function freePixelRoute(r: FreeRaster, from: Point2, to: Point2): Point2[] {
  const a = pixelAt(r, from), b = pixelAt(r, to);
  if (a < 0 || b < 0 || !r.free[a] || !r.free[b]) return [from, to];
  const parent = new Int32Array(r.free.length).fill(-2);
  parent[a] = -1;
  const queue = [a];
  for (let h = 0; h < queue.length; h++) {
    const q = queue[h];
    if (q === b) break;
    const i = q % r.nx;
    const j = (q - i) / r.nx;
    for (const m of [i > 0 ? q - 1 : -1, i + 1 < r.nx ? q + 1 : -1, j > 0 ? q - r.nx : -1, j + 1 < r.ny ? q + r.nx : -1]) {
      if (m < 0 || !r.free[m] || parent[m] !== -2) continue;
      parent[m] = q;
      queue.push(m);
    }
  }
  if (parent[b] === -2) return [from, to];
  const out: Point2[] = [];
  for (let q = b; q !== -1; q = parent[q]) out.push(pixelCenter(r, q));
  return out.reverse();
}

/**
 * A naive patrol for the three-pocket world: visit each pocket's far leg in
 * turn and come back to the hub. One pursuer doing this clears each pocket
 * and watches it be recontaminated from the hub while it clears the next.
 */
export function pocketTour(cw: CellWorld, r: FreeRaster, start: Point2): Point2[] {
  const tips: Point2[] = [];
  // The far leg of each pocket, from the pocket template: local (2.5, 1.6).
  const b = cw.boundary;
  for (let i = 0; i + 7 < b.length; i++) {
    // A pocket's 8 vertices run from local (1,0) to (0,0); detect by the hub corners around them.
    const o = b[i + 7];
    const e1x = b[i].x - o.x, e1y = b[i].y - o.y;
    const len = Math.hypot(e1x, e1y);
    if (Math.abs(len - 1) > 1e-9) continue;
    const e2 = { x: b[i + 1].x - b[i].x, y: b[i + 1].y - b[i].y };
    const l2 = Math.hypot(e2.x, e2.y);
    if (Math.abs(l2 - 3) > 1e-9) continue;
    const u = { x: e1x, y: e1y };
    const n = { x: e2.x / 3, y: e2.y / 3 };
    tips.push({ x: o.x + 2.5 * u.x + 1.6 * n.x, y: o.y + 2.5 * u.y + 1.6 * n.y });
  }
  const out: Point2[] = [start];
  let cur = start;
  for (const t of tips) {
    out.push(...freePixelRoute(r, cur, t).slice(1));
    out.push(...freePixelRoute(r, t, start).slice(1));
    cur = start;
  }
  return out;
}
