/**
 * Nearest-obstacle queries for the Apartment: from Rusty's range ring (the
 * sensor the exploration actually runs on) and from exact wall geometry (the
 * ground truth the checks compare against). Both return the same `Nearest`
 * shape, so the explorer cannot tell them apart — which is the point of
 * §5.3.3: G and DG are sensor quantities.
 */

import { rangeScan, type Scan } from '../bugs/scan';
import type { Rng } from '../prob/rng';
import { APARTMENT, rayCast, type World } from '../sim/world';
import { apartmentDistanceQuery } from './examples';
import { exactGvd, siteDistance, type Gvd, type Site } from './gvd';
import { mergeDuplicateMinima, nearestFromScan, type Nearest, type NearestQuery, type Vec } from './gvg';

/** A `NearestQuery` over Chapter 8 sites (points, segments, polygons) — the micro-world and w9.3. */
export function sitesQuery(sites: readonly Site[]): NearestQuery {
  return (q: Vec) =>
    sites
      .map((s, i) => siteDistance(s, i, { x: q[0], y: q[1] }))
      .sort((a, b) => a.d - b.d)
      .map((s) => ({ d: s.d, grad: [s.grad.x, s.grad.y], closest: [s.closest.x, s.closest.y], index: s.index }));
}

export interface ScanQueryOptions {
  nRays?: number;
  maxRange?: number;
  /** Gaussian range noise σ (m), drawn from `rng`; zero for the ideal ring. */
  sigma?: number;
  rng?: Rng;
  /**
   * Refine each local minimum's bearing by golden-section search between its
   * neighbouring rays (Chapter 3's "more rays where they matter"): the
   * minimum of ρ then sits on the obstacle's nearest point, not on the nearest
   * ray, and the homing converges to the geometric meet point. On by default.
   */
  refine?: boolean;
}

/**
 * The range-ring query: scan, perturb (optionally), take the local minima of
 * ρ as the obstacles, merge duplicates. `lastScan` keeps the scan for drawing.
 */
export class ScanQuery {
  readonly nRays: number;
  readonly maxRange: number;
  readonly refine: boolean;
  sigma: number;
  lastScan: Scan | null = null;
  lastNearest: Nearest[] = [];
  /** Range queries issued so far (one per ray, plus refinement rays). */
  rays = 0;

  constructor(
    readonly world: World = APARTMENT,
    opts: ScanQueryOptions = {},
    readonly rng?: Rng,
  ) {
    this.nRays = opts.nRays ?? 360;
    this.maxRange = opts.maxRange ?? 6;
    this.sigma = opts.sigma ?? 0;
    this.refine = opts.refine ?? true;
  }

  /** Golden-section search of ρ(θ) between the rays flanking local-minimum ray k. */
  private refineMinimum(q: Vec, scan: Scan, k: number): { d: number; theta: number } {
    const n = scan.ranges.length;
    const step = (2 * Math.PI) / n;
    let a = scan.angles[k] - step;
    let b = scan.angles[k] + step;
    const f = (t: number) => {
      this.rays++;
      return rayCast(this.world, q[0], q[1], t, this.maxRange);
    };
    const g = (Math.sqrt(5) - 1) / 2;
    let c = b - g * (b - a);
    let d = a + g * (b - a);
    let fc = f(c);
    let fd = f(d);
    for (let i = 0; i < 36; i++) {
      if (fc < fd) {
        b = d;
        d = c;
        fd = fc;
        c = b - g * (b - a);
        fc = f(c);
      } else {
        a = c;
        c = d;
        fc = fd;
        d = a + g * (b - a);
        fd = f(d);
      }
    }
    const theta = 0.5 * (a + b);
    return { d: Math.min(f(theta), scan.ranges[k], fc, fd), theta };
  }

  readonly query: NearestQuery = (q) => {
    const scan = rangeScan(this.world, q[0], q[1], this.nRays, this.maxRange);
    this.rays += this.nRays;
    this.lastScan = scan;
    let near = nearestFromScan(scan);
    if (this.refine) {
      // Each minimum's ray index: recover it from the gradient's bearing.
      near = near.map((nr) => {
        const theta0 = Math.atan2(-nr.grad[1], -nr.grad[0]);
        let k = Math.round((theta0 / (2 * Math.PI)) * this.nRays);
        k = ((k % this.nRays) + this.nRays) % this.nRays;
        const r = this.refineMinimum(q, scan, k);
        if (!Number.isFinite(r.d) || r.d >= this.maxRange) return nr;
        return { d: r.d, grad: [-Math.cos(r.theta), -Math.sin(r.theta)], closest: [q[0] + r.d * Math.cos(r.theta), q[1] + r.d * Math.sin(r.theta)], index: -1 };
      });
    }
    if (this.sigma > 0 && this.rng) {
      near = near.map((nr) => ({ ...nr, d: Math.max(1e-3, nr.d + this.rng!.normal(0, this.sigma)) }));
    }
    near.sort((a, b) => a.d - b.d);
    near = mergeDuplicateMinima(near, 0.05, Math.max(1e-6, 2 * this.sigma));
    this.lastNearest = near;
    return near;
  };
}

/**
 * Exact wall geometry as a `NearestQuery` (Chapter 8's `PlanarObstacles`, one
 * obstacle per wall). Two walls meeting at a corner both report the corner as
 * their closest point from inside the angle; that is one obstacle feature with
 * one gradient — the SS_ij refinement — and a range ring sees one local
 * minimum there, so the readings are merged by closest point.
 */
export function apartmentExactQuery(world: World = APARTMENT): NearestQuery {
  const dq = apartmentDistanceQuery(world);
  return (q) => {
    const all = dq
      .distances([q[0], q[1]])
      .map((o) => ({ d: o.d, grad: [o.grad[0], o.grad[1]], closest: o.closest, index: o.index }))
      .sort((a, b) => a.d - b.d);
    const out: Nearest[] = [];
    for (const n of all) {
      const dup = out.some((o) => o.closest && n.closest && Math.hypot(o.closest[0] - n.closest[0], o.closest[1] - n.closest[1]) < 1e-7);
      if (!dup) out.push(n);
    }
    return out;
  };
}

/**
 * Chapter 8's exact planar GVD of the Apartment's walls — the ground truth
 * w9.1 overlays. Chapter 8 traced separated sites; the Apartment's walls share
 * corners, and a point whose three "equidistant" sites include two walls met
 * at their common corner is equidistant to two *features*, not three. Such
 * meet points are dropped: they are S_ij ∩ S_ik points that the SS_ijk
 * refinement excludes, and a range ring never reports them.
 */
export function apartmentExactGvd(world: World = APARTMENT, step = 0.03, seedsPerAxis = 36): Gvd {
  const sites = world.walls.map((s) => ({ kind: 'segment' as const, s }));
  const gvd = exactGvd(sites, world.bounds, { step, seedsPerAxis });
  const genuine = gvd.meetPoints.filter((m) => {
    const c = m.sites.map((i) => siteDistance(sites[i], i, m.q).closest);
    return Math.hypot(c[0].x - c[1].x, c[0].y - c[1].y) > 1e-6 && Math.hypot(c[0].x - c[2].x, c[0].y - c[2].y) > 1e-6 && Math.hypot(c[1].x - c[2].x, c[1].y - c[2].y) > 1e-6;
  });
  return { ...gvd, meetPoints: genuine };
}
