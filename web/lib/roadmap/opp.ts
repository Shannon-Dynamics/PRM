/**
 * The opportunistic path planner — Choset §5.5.2 (Canny & Lin) — in the
 * plane, on a bounded polygonal world.
 *
 * Sweep a vertical slice x = λ. Its free part splits into *channels* (slice
 * intervals). On each channel the slice-restricted distance
 * D̃(p; λ) = min_i d̃_i(p; λ) is continuous but not smooth: it has a kink
 * wherever two obstacles tie. The traces of its local maxima are *freeways*.
 * At a local maximum no direction along the slice increases every active
 * d_i, which is the generalized-gradient condition 0 ∈ ∂D̃ = Co{∂d_i/∂y :
 * i ∈ Z(q*)}: for a smooth maximum the one slope vanishes, for a kink the
 * slopes bracket zero. Convex obstacles have convex d_i (Choset Problem 19),
 * so no smooth maximum exists and every freeway point is a tie — a GVD
 * point (Problem 12: OPP ⊂ GVG for any slice direction).
 *
 * Freeways are not connected; *bridge curves* join them where channels
 * merge or split — the interesting critical points, which are exactly the
 * boustrophedon critical points Chapter 10 builds cells from. The bridge is
 * the free slice segment, just past the critical value on the merged side,
 * between the freeway points it joins.
 */

import { pointInPolygon, type Polygon } from '../collide/polygon';
import type { Point2, Segment } from '../sim/world';
import { siteDistances, type Site } from './gvd';

/**
 * A bounded polygonal world. The boundary is a convex polygon with no edge
 * parallel to the sweep direction: along a slice parallel to a wall, D̃ is
 * constant wherever that wall is nearest, every point is a (degenerate) local
 * maximum, and the OPP's Morse assumption fails. Tilting the boundary is the
 * general-position fix.
 */
export interface OppWorld {
  boundary: Polygon;
  obstacles: Polygon[];
}

export function oppBounds(w: OppWorld) {
  const xs = w.boundary.map((p) => p.x);
  const ys = w.boundary.map((p) => p.y);
  return { minX: Math.min(...xs), maxX: Math.max(...xs), minY: Math.min(...ys), maxY: Math.max(...ys) };
}

/** The world's sites for distance queries: the boundary's edges and the obstacles. */
export function oppSites(w: OppWorld): Site[] {
  const n = w.boundary.length;
  const walls: Segment[] = w.boundary.map((p, i) => ({ x1: p.x, y1: p.y, x2: w.boundary[(i + 1) % n].x, y2: w.boundary[(i + 1) % n].y }));
  return [...walls.map((s) => ({ kind: 'segment' as const, s })), ...w.obstacles.map((poly) => ({ kind: 'polygon' as const, poly }))];
}

/** The free intervals (channels) of the slice x = λ, as [y_lo, y_hi] pairs. */
export function channels(w: OppWorld, lambda: number): [number, number][] {
  const ys: number[] = [];
  for (const poly of [w.boundary, ...w.obstacles]) {
    for (let i = 0; i < poly.length; i++) {
      const a = poly[i];
      const b = poly[(i + 1) % poly.length];
      if ((a.x <= lambda && b.x > lambda) || (b.x <= lambda && a.x > lambda)) {
        const t = (lambda - a.x) / (b.x - a.x);
        ys.push(a.y + t * (b.y - a.y));
      }
    }
  }
  ys.sort((p, q) => p - q);
  const out: [number, number][] = [];
  for (let k = 0; k + 1 < ys.length; k++) {
    const lo = ys[k];
    const hi = ys[k + 1];
    if (hi - lo < 1e-9) continue;
    const mid = 0.5 * (lo + hi);
    if (!pointInPolygon(lambda, mid, w.boundary)) continue;
    if (w.obstacles.some((poly) => pointInPolygon(lambda, mid, poly))) continue;
    out.push([lo, hi]);
  }
  return out;
}

export interface FreewayPoint {
  q: Point2;
  /** D̃ at the maximum. */
  D: number;
  /** Indices of the active (tied) obstacles Z(q*). */
  active: number[];
  /** Slice slopes ∂d_i/∂y of the active obstacles: 0 ∈ Co{·} at a maximum. */
  slopes: number[];
  /** Whether 0 lies in the convex hull of the slopes. */
  generalizedGradientZero: boolean;
  channel: number;
}

/** Local maxima of D̃ on every channel of the slice x = λ. */
export function sliceMaxima(w: OppWorld, sites: readonly Site[], lambda: number, samples = 160, tieTol = 1e-6): FreewayPoint[] {
  const out: FreewayPoint[] = [];
  const D = (y: number) => siteDistances(sites, { x: lambda, y })[0].d;
  channels(w, lambda).forEach(([lo, hi], ci) => {
    if (hi - lo < 1e-6) return;
    const ys = Array.from({ length: samples + 1 }, (_, k) => lo + ((hi - lo) * k) / samples);
    const ds = ys.map(D);
    for (let k = 1; k < samples; k++) {
      // A strict interior maximum with some prominence: D̃ is 1-Lipschitz, so a true kink maximum rises
      // above its neighbours by a definite amount, while round-off on a plateau does not.
      if (!(ds[k] > ds[k - 1] && ds[k] > ds[k + 1])) continue;
      if (ds[k] - Math.max(ds[k - 1], ds[k + 1]) < 1e-9) continue;
      // Golden-section refinement on [y_{k−1}, y_{k+1}].
      let a = ys[k - 1];
      let b = ys[k + 1];
      const g = (Math.sqrt(5) - 1) / 2;
      let c = b - g * (b - a);
      let d = a + g * (b - a);
      let fc = D(c);
      let fd = D(d);
      for (let it = 0; it < 60; it++) {
        if (fc > fd) {
          b = d;
          d = c;
          fd = fc;
          c = b - g * (b - a);
          fc = D(c);
        } else {
          a = c;
          c = d;
          fc = fd;
          d = a + g * (b - a);
          fd = D(d);
        }
      }
      const y = 0.5 * (a + b);
      const q = { x: lambda, y };
      const all = siteDistances(sites, q);
      const Dq = all[0].d;
      const act = all.filter((s) => s.d - Dq <= Math.max(tieTol, 1e-7 * Dq) + 1e-9);
      const slopes = act.map((s) => s.grad.y);
      const gz = Math.min(...slopes) <= 1e-7 && Math.max(...slopes) >= -1e-7;
      out.push({ q, D: Dq, active: act.map((s) => s.index), slopes, generalizedGradientZero: gz, channel: ci });
    }
  });
  return out;
}

export interface OppRoadmap {
  lambdas: number[];
  /** Freeway points per slice. */
  slices: FreewayPoint[][];
  /** Freeways: chains of maxima across slices. */
  freeways: Point2[][];
  /** Interesting critical values: where the channel count changes. */
  critical: { lambda: number; before: number; after: number }[];
  /** Bridge curves: free slice segments joining freeways at the critical values. */
  bridges: Point2[][];
  componentsWithoutBridges: number;
  componentsWithBridges: number;
}

/**
 * Sweep λ across the world, trace freeways, find the interesting critical
 * values by bisection on the channel count, build bridges, and count the
 * roadmap's components with and without them.
 */
export function oppRoadmap(w: OppWorld, n = 100): OppRoadmap {
  const sites = oppSites(w);
  const b = oppBounds(w);
  const pad = 1e-3;
  const lambdas = Array.from({ length: n + 1 }, (_, k) => b.minX + pad + ((b.maxX - b.minX - 2 * pad) * k) / n);
  const slices = lambdas.map((lam) => sliceMaxima(w, sites, lam));
  const dl = lambdas[1] - lambdas[0];

  // Freeways by nearest-neighbour chaining in y.
  const freeways: Point2[][] = [];
  let open: { pts: Point2[] }[] = [];
  for (const sl of slices) {
    const used = new Set<number>();
    const next: typeof open = [];
    for (const o of open) {
      const last = o.pts[o.pts.length - 1];
      let best = -1;
      let bd = 0.35;
      sl.forEach((fp, i) => {
        if (used.has(i)) return;
        const d = Math.abs(fp.q.y - last.y);
        if (d < bd) {
          bd = d;
          best = i;
        }
      });
      if (best >= 0) {
        used.add(best);
        o.pts.push(sl[best].q);
        next.push(o);
      }
    }
    sl.forEach((fp, i) => {
      if (used.has(i)) return;
      const o = { pts: [fp.q] };
      freeways.push(o.pts);
      next.push(o);
    });
    open = next;
  }

  // Interesting critical values: the channel count changes.
  const critical: OppRoadmap['critical'] = [];
  for (let k = 1; k < lambdas.length; k++) {
    const ca = channels(w, lambdas[k - 1]).length;
    const cb = channels(w, lambdas[k]).length;
    if (ca === cb) continue;
    let lo = lambdas[k - 1];
    let hi = lambdas[k];
    for (let it = 0; it < 40; it++) {
      const mid = 0.5 * (lo + hi);
      if (channels(w, mid).length === ca) lo = mid;
      else hi = mid;
    }
    critical.push({ lambda: 0.5 * (lo + hi), before: ca, after: cb });
  }

  // Bridges: on the merged side (fewer channels), just past λ*, the slice segment through the
  // channel that spans the splitting ones, between the extreme freeway points on it.
  const bridges: Point2[][] = [];
  for (const c of critical) {
    const eps = 0.5 * dl;
    const side = c.before < c.after ? c.lambda - eps : c.lambda + eps;
    const other = c.before < c.after ? c.lambda + eps : c.lambda - eps;
    const merged = sliceMaxima(w, sites, side);
    const split = sliceMaxima(w, sites, other);
    // The channel of the merged slice that contains the splitting slice's two new channels' maxima (by y).
    const chs = channels(w, side);
    for (let ci = 0; ci < chs.length; ci++) {
      const [lo, hi] = chs[ci];
      const inside = split.filter((fp) => fp.q.y > lo && fp.q.y < hi);
      const own = merged.filter((fp) => fp.channel === ci);
      const ys = [...inside.map((fp) => fp.q.y), ...own.map((fp) => fp.q.y)];
      if (inside.length >= 2 || (inside.length >= 1 && own.length >= 1 && new Set(ys.map((y) => y.toFixed(3))).size >= 2)) {
        bridges.push([
          { x: side, y: Math.min(...ys) },
          { x: side, y: Math.max(...ys) },
        ]);
      }
    }
  }

  // Bifurcation bridges (Choset: "bridge curves are also built when freeways terminate in the free
  // space at bifurcation points, where traces of local maxima and local minima meet"): a freeway
  // that ends away from the sweep's ends and from every interesting critical value is joined, along
  // its last slice, to the nearest freeway point in the same channel.
  const fwAll = freeways.filter((f) => f.length >= 2);
  const atCritical = (x: number) => critical.some((c) => Math.abs(c.lambda - x) <= dl + 1e-9) || x <= lambdas[0] + 1e-9 || x >= lambdas[lambdas.length - 1] - 1e-9;
  for (const f of fwAll) {
    for (const end of [f[0], f[f.length - 1]]) {
      if (atCritical(end.x)) continue;
      const ch = channels(w, end.x).find(([lo, hi]) => end.y > lo && end.y < hi);
      if (!ch) continue;
      let best: Point2 | null = null;
      let bd = Infinity;
      for (const g of fwAll) {
        if (g === f) continue;
        for (const p of g) {
          if (Math.abs(p.x - end.x) > dl + 1e-9 || p.y <= ch[0] || p.y >= ch[1]) continue;
          const d = Math.abs(p.y - end.y);
          if (d < bd) {
            bd = d;
            best = p;
          }
        }
      }
      if (best) bridges.push([end, { x: end.x, y: best.y }]);
    }
  }

  // Connectivity.
  const near = (a: Point2, b: Point2, tol: number) => Math.hypot(a.x - b.x, a.y - b.y) < tol;
  const touches = (c1: Point2[], c2: Point2[], tol: number) => c1.some((p) => c2.some((q) => near(p, q, tol)));
  const comps = (items: Point2[][], tol: number) => {
    const label = new Array<number>(items.length).fill(-1);
    let count = 0;
    for (let i = 0; i < items.length; i++) {
      if (label[i] >= 0) continue;
      const stack = [i];
      label[i] = count;
      while (stack.length) {
        const a = stack.pop()!;
        for (let b2 = 0; b2 < items.length; b2++) if (label[b2] < 0 && touches(items[a], items[b2], tol)) {
          label[b2] = count;
          stack.push(b2);
        }
      }
      count++;
    }
    return { count, label };
  };
  const tol = 1.5 * dl + 0.02;
  const fw = freeways.filter((f) => f.length >= 2);
  const without = comps(fw, tol).count;
  // Bridges link freeways whose points lie on them (same x within dl, y within the segment).
  const densified = bridges.map((bq) => {
    const out: Point2[] = [];
    const steps = Math.max(2, Math.ceil(Math.abs(bq[1].y - bq[0].y) / 0.02));
    for (let k = 0; k <= steps; k++) out.push({ x: bq[0].x, y: bq[0].y + ((bq[1].y - bq[0].y) * k) / steps });
    return out;
  });
  const withB = comps([...fw, ...densified], tol);
  const withBridges = new Set(fw.map((_, i) => withB.label[i])).size;
  return { lambdas, slices, freeways: fw, critical, bridges, componentsWithoutBridges: without, componentsWithBridges: withBridges };
}

/** C Fig. 5.34's spirit: a tilted 10 × 6 room with two convex obstacles whose channels merge and split. */
export const OPP_WORLD: OppWorld = {
  boundary: [
    { x: 0, y: 0.25 },
    { x: 10, y: 0 },
    { x: 10.3, y: 6 },
    { x: -0.2, y: 5.8 },
  ],
  obstacles: [
    [
      { x: 2.2, y: 2.3 },
      { x: 4.1, y: 1.6 },
      { x: 4.6, y: 3.4 },
      { x: 3.0, y: 4.3 },
    ],
    [
      { x: 6.0, y: 0.9 },
      { x: 8.2, y: 1.4 },
      { x: 7.9, y: 3.1 },
      { x: 6.4, y: 2.6 },
    ],
  ],
};
