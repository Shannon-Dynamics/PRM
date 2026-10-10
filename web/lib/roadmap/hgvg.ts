/**
 * The GVG in R³ and the hierarchical GVG — Choset §5.3.1–5.3.2 on a small
 * synthetic configuration space: a rectangular room (six planes) with a
 * floating rectangular box in its interior, the environment of C Fig. 5.19.
 *
 * Why the box: equidistance to three obstacles in R³ is a curve (the GVG),
 * but the curves need not connect. The room's walls give an outer network;
 * the floor, ceiling and box give a *halo* — a GVG cycle, homeomorphic to
 * S¹, that no edge of the outer network reaches. The floor–ceiling sheet
 * F_fc is the plane z = h/2, and the GVG edges lie on sheet boundaries; the
 * halo bounds a hole in F_fc. The repair (§5.3.2) draws planar GVG edges *on*
 * the sheet — second-order edges, where the closest pair ties and the
 * second-closest pair ties — finds a closed loop of them with a common
 * second-closest obstacle (a *period*), and descends the projected gradient
 * −π_{T F_fc} ∇d_box from the period until d_box = d_floor = d_ceiling:
 * the halo.
 *
 * The second-order edges are traced by the same explorer that traces the
 * GVG, run on the sheet with the sheet's own nearest-obstacle query: the
 * planar GVD of the slice, with floor and ceiling (tied by construction)
 * left out. Nothing is re-implemented.
 */

import { GvgExplorer, type ExplorerOptions } from './explore';
import { pseudoInverse, matVec, nullSpaceDirection, projectToGvg, vadd, vdist, vdot, vnorm, vscale, vsub, vunit, constraint, type Nearest, type NearestQuery, type Vec } from './gvg';

/* -------------------------------------------------------------------------- */
/* Analytic sites in R³                                                        */
/* -------------------------------------------------------------------------- */

export type Site3 =
  /** The boundary of a half-space n·q ≥ c (n a unit normal pointing into the free space). */
  | { kind: 'plane'; n: Vec; c: number; name: string }
  /** An axis-aligned box [lo, hi]. */
  | { kind: 'box'; lo: Vec; hi: Vec; name: string };

/** d_i and ∇d_i for one site at q (outside the site). */
export function site3Distance(site: Site3, q: Vec, index: number): Nearest {
  if (site.kind === 'plane') {
    const d = vdot(site.n, q) - site.c;
    return { d, grad: site.n.slice(), closest: vsub(q, vscale(site.n, d)), index };
  }
  const diff = q.map((x, i) => (x < site.lo[i] ? x - site.lo[i] : x > site.hi[i] ? x - site.hi[i] : 0));
  const d = vnorm(diff);
  const closest = q.map((x, i) => Math.max(site.lo[i], Math.min(site.hi[i], x)));
  return { d, grad: d > 1e-15 ? vscale(diff, 1 / d) : [0, 0, 0], closest, index };
}

/** A nearest-obstacle query over analytic sites, nearest first. */
export function sites3Query(sites: readonly Site3[]): NearestQuery {
  return (q) => sites.map((s, i) => site3Distance(s, q, i)).sort((a, b) => a.d - b.d);
}

/** A w × l × h room with its floor at z = 0: six planes, named. */
export function roomSites(w: number, l: number, h: number): Site3[] {
  return [
    { kind: 'plane', n: [0, 0, 1], c: 0, name: 'floor' },
    { kind: 'plane', n: [0, 0, -1], c: -h, name: 'ceiling' },
    { kind: 'plane', n: [1, 0, 0], c: 0, name: 'west' },
    { kind: 'plane', n: [-1, 0, 0], c: -w, name: 'east' },
    { kind: 'plane', n: [0, 1, 0], c: 0, name: 'south' },
    { kind: 'plane', n: [0, -1, 0], c: -l, name: 'north' },
  ];
}

/** C Fig. 5.19: a 10 × 8 × 3 room with a 1 × 1 × 1 box floating at mid-height in the middle. */
export const FIG_5_19_ROOM = { w: 10, l: 8, h: 3 };
export const FIG_5_19_BOX: Site3 = { kind: 'box', lo: [4.5, 3.5, 1], hi: [5.5, 4.5, 2], name: 'box' };
export const FIG_5_19_SITES: Site3[] = [...roomSites(10, 8, 3), FIG_5_19_BOX];
export const FIG_5_19_BOX_CENTER: Vec = [5, 4, 1.5];
/** The halo's clearance: d_floor = d_ceiling = h/2 = 1.5, so it sits 1.5 from the box's side faces. */
export const FIG_5_19_HALO_CLEARANCE = 1.5;

/* -------------------------------------------------------------------------- */
/* Second-order edges on a sheet                                               */
/* -------------------------------------------------------------------------- */

/**
 * The nearest-obstacle query *on the sheet* F_ij for two parallel planes i, j
 * (the floor–ceiling sheet z = h/2): a planar query in (x, y) that reports
 * every other obstacle's distance from the sheet point and its gradient
 * projected onto the sheet. The planar GVD of this query is exactly the set
 * of second-order edges F_kl|F_ij of eq. (5.4) — points where, with i and j
 * tied by construction, k and l tie for second-closest.
 */
export function sheetQuery(sites: readonly Site3[], i: number, j: number, z: number): NearestQuery {
  return (p) => {
    const q = [p[0], p[1], z];
    const out: Nearest[] = [];
    sites.forEach((s, k) => {
      if (k === i || k === j) return;
      const n = site3Distance(s, q, k);
      out.push({ d: n.d, grad: [n.grad[0], n.grad[1]], closest: n.closest ? [n.closest[0], n.closest[1]] : undefined, index: k });
    });
    return out.sort((a, b) => a.d - b.d);
  };
}

/** The height of the floor–ceiling sheet. */
export const sheetHeight = (h: number) => h / 2;

/**
 * Explore the second-order GVG on the floor–ceiling sheet with the planar
 * explorer. Returns the explorer (nodes, edges) and, for every edge, the
 * pair of obstacles tied along it (sampled at its midpoint).
 */
export function exploreSheet(sites: readonly Site3[], floor: number, ceiling: number, z: number, q0: Vec, opts: ExplorerOptions = {}) {
  const query = sheetQuery(sites, floor, ceiling, z);
  const ex = new GvgExplorer(query, q0, { alpha: 1, beta: -4, dt: 0.03, meetTol: 0.06, boundaryTol: 0.1, nodeTol: 0.02, ...opts }).run();
  const pairs = ex.edges.map((e) => {
    if (e.to < 0 || e.polyline.length < 3) return null;
    const mid = e.polyline[Math.floor(e.polyline.length / 2)];
    const near = query(mid);
    return [near[0].index, near[1].index] as [number, number];
  });
  return { explorer: ex, query, pairs };
}

/**
 * The period: the second-order edges whose tied pair includes `obstacle`
 * (the common second-closest obstacle). Reports whether they close a loop
 * around `center` — the angular coverage of their samples about it, and the
 * largest angular gap.
 */
export function periodAround(explorerEdges: { polyline: Vec[]; to: number }[], pairs: ([number, number] | null)[], obstacle: number, center: Vec) {
  const edges = explorerEdges.map((e, i) => ({ e, pair: pairs[i] })).filter(({ e, pair }) => e.to >= 0 && pair && (pair[0] === obstacle || pair[1] === obstacle));
  const angles: number[] = [];
  for (const { e } of edges) for (const p of e.polyline) angles.push(Math.atan2(p[1] - center[1], p[0] - center[0]));
  angles.sort((a, b) => a - b);
  let maxGap = angles.length ? angles[0] + 2 * Math.PI - angles[angles.length - 1] : 2 * Math.PI;
  for (let k = 1; k < angles.length; k++) maxGap = Math.max(maxGap, angles[k] - angles[k - 1]);
  return { edges: edges.map(({ e }) => e), maxGap, closes: angles.length > 0 && maxGap < 0.15 };
}

/* -------------------------------------------------------------------------- */
/* Period following: ċ = −π_{T F_ij} ∇d_k                                      */
/* -------------------------------------------------------------------------- */

export interface PeriodFollow {
  path: Vec[];
  /** 'gvg' when d_k came down to d_i = d_j; 'vanished' when the projected gradient did; 'budget' otherwise. */
  stop: 'gvg' | 'vanished' | 'budget';
  /** The three distances at the end. */
  end: { di: number; dj: number; dk: number };
}

/**
 * From a point of the period (on F_ij, with k the common second-closest
 * obstacle), descend d_k while staying on the sheet: step along
 * −π ∇d_k with π = I − n nᵀ, n = unit(∇d_i − ∇d_j), then Newton-correct
 * d_i − d_j back to zero. Stops on the GVG edge F_ijk (d_k = d_i = d_j) or
 * when the projected gradient vanishes — Choset: "then no such GVG edge
 * exists", and the robot returns to the period.
 */
export function followPeriod(query: NearestQuery, i: number, j: number, k: number, q0: Vec, step = 0.02, maxSteps = 5000, tol = 1e-6): PeriodFollow {
  let q = q0.slice();
  const path = [q.slice()];
  const byIndex = (near: Nearest[], idx: number) => near.find((n) => n.index === idx)!;
  let stop: PeriodFollow['stop'] = 'budget';
  for (let s = 0; s < maxSteps; s++) {
    const near = query(q);
    const ni = byIndex(near, i);
    const nj = byIndex(near, j);
    const nk = byIndex(near, k);
    if (nk.d - ni.d < tol) {
      stop = 'gvg';
      break;
    }
    const n = vunit(vsub(ni.grad, nj.grad));
    const g = nk.grad;
    const proj = vsub(g, vscale(n, vdot(n, g)));
    if (vnorm(proj) < 1e-6) {
      stop = 'vanished';
      break;
    }
    // Do not overshoot the tie: the step is at most the gap d_k − d_i.
    const h = Math.min(step, Math.max(0.5 * (nk.d - ni.d), 1e-4));
    q = vsub(q, vscale(vunit(proj), h));
    // Newton back onto the sheet along n.
    for (let it = 0; it < 4; it++) {
      const nn = query(q);
      const a = byIndex(nn, i);
      const b = byIndex(nn, j);
      const G = a.d - b.d;
      if (Math.abs(G) < 1e-12) break;
      const dG = vsub(a.grad, b.grad);
      const dd = vdot(dG, dG);
      if (dd < 1e-18) break;
      q = vsub(q, vscale(dG, G / dd));
    }
    path.push(q.slice());
  }
  if (stop === 'gvg') {
    // The last step may overshoot the tie by its own length: polish onto the GVG edge.
    const on = projectToGvg(query, q);
    if (on) {
      q = on;
      path.push(q.slice());
    }
  }
  const near = query(q);
  return { path, stop, end: { di: byIndex(near, i).d, dj: byIndex(near, j).d, dk: byIndex(near, k).d } };
}

/**
 * Trace a GVG cycle from a point on it with the edge law until it closes
 * (the halo of Fig. 5.19 is homeomorphic to S¹). Returns the loop, its
 * length, whether it closed, and the worst |d_a − d_b| over the three tied
 * obstacles along it.
 */
export function traceCycle(query: NearestQuery, q0: Vec, heading0: Vec, step = 0.02, maxSteps = 20000) {
  const m = q0.length;
  let q = q0.slice();
  let heading = vunit(heading0);
  const loop = [q.slice()];
  let length = 0;
  let worstTie = 0;
  let closed = false;
  for (let s = 0; s < maxSteps; s++) {
    const near = query(q);
    const { G, DG } = constraint(near, m);
    worstTie = Math.max(worstTie, ...G.map(Math.abs));
    const dag = pseudoInverse(DG);
    if (!dag) break;
    // Tangent: the null-space direction aligned with the heading; correction: −DG† G (a full Newton step).
    const v = nullSpaceDirection(DG, heading);
    if (vnorm(v) < 1e-9) break;
    let next = vadd(q, vscale(v, step));
    for (let it = 0; it < 6; it++) {
      const nn = query(next);
      const c = constraint(nn, m);
      const d2 = pseudoInverse(c.DG);
      if (!d2) break;
      next = vadd(next, matVec(d2, vscale(c.G, -1)));
    }
    length += vdist(q, next);
    heading = vunit(vsub(next, q));
    q = next;
    loop.push(q.slice());
    if (length > 4 * step && vdist(q, loop[0]) < 0.75 * step) {
      closed = true;
      loop.push(loop[0].slice());
      length += vdist(q, loop[0]);
      break;
    }
  }
  return { loop, length, closed, worstTie };
}

/* -------------------------------------------------------------------------- */
/* The whole §5.3.2 story on the Fig. 5.19 room                                 */
/* -------------------------------------------------------------------------- */

export interface HgvgResult {
  /** The outer GVG network explored from a start in the open room. */
  outer: GvgExplorer;
  /** The second-order GVG on the floor–ceiling sheet. */
  sheet: ReturnType<typeof exploreSheet>;
  /** The period around the box and whether it closes. */
  period: ReturnType<typeof periodAround>;
  /** The descent from the period to the halo. */
  link: PeriodFollow | null;
  /** The halo traced as a GVG cycle. */
  halo: ReturnType<typeof traceCycle> | null;
}

/** Index helpers for the Fig. 5.19 site list. */
export const FIG_5_19_INDEX = { floor: 0, ceiling: 1, west: 2, east: 3, south: 4, north: 5, box: 6 } as const;

export function buildFig519Hgvg(start: Vec = [2.2, 1.3, 0.9]): HgvgResult {
  const sites = FIG_5_19_SITES;
  const query = sites3Query(sites);
  const outer = new GvgExplorer(query, start, { alpha: 1, beta: -4, dt: 0.03, meetTol: 0.08, boundaryTol: 0.12, nodeTol: 0.02 }).run();
  const z = sheetHeight(FIG_5_19_ROOM.h);
  const sheet = exploreSheet(sites, FIG_5_19_INDEX.floor, FIG_5_19_INDEX.ceiling, z, [1.3, 1.1]);
  const period = periodAround(sheet.explorer.edges, sheet.pairs, FIG_5_19_INDEX.box, FIG_5_19_BOX_CENTER);
  let link: PeriodFollow | null = null;
  let halo: ReturnType<typeof traceCycle> | null = null;
  if (period.edges.length) {
    const e = period.edges[0];
    const p = e.polyline[Math.floor(e.polyline.length / 2)];
    link = followPeriod(query, FIG_5_19_INDEX.floor, FIG_5_19_INDEX.ceiling, FIG_5_19_INDEX.box, [p[0], p[1], z]);
    if (link.stop === 'gvg') {
      const q0 = link.path[link.path.length - 1];
      // Any tangent will do to start; take the horizontal direction perpendicular to the radius from the box.
      const r = vsub(q0, FIG_5_19_BOX_CENTER);
      halo = traceCycle(query, q0, [-r[1], r[0], 0], 0.02);
    }
  }
  return { outer, sheet, period, link, halo };
}
