/**
 * The integration lab: Reach's Workbench raster on T², with and without the
 * edge identification (Choset §3.4.3's connectedness, made a measured quantity).
 *
 * Chapter 4 owns the general C-space raster (`lib/cspace`). It had not landed
 * when this chapter was written, so the *minimal* thing the lab needs — a 2R
 * arm among circular obstacles, rasterized on [−π, π]² — lives here. It is a
 * stand-in, not a second implementation of Chapter 4's machinery: three disc
 * obstacles, segment–disc distance, nothing else. When `lib/cspace` lands, the
 * Torus Unwrapper can be pointed at its raster with no other change.
 */

import { Rng } from '../prob/rng';
import { CHOSET_ARM, links2R, type Arm2R } from './dphi';
import { T2, cellAngles, cellOf, labelComponents, type BitGrid, type T2Point } from './torus';

export interface Disc {
  x: number;
  y: number;
  r: number;
}

/** The Workbench stand-in: three discs the arm must thread between. */
export const WORKBENCH_DISCS: readonly Disc[] = [
  // Within reach of link 1: its C-obstacle is a band across *all* θ₂, which cuts
  // the square chart in two — and is one connected region on the torus.
  { x: 0.8, y: 0.35, r: 0.3 },
  // Reachable only by link 2: blobs, not bands.
  { x: -1.3, y: 0.9, r: 0.4 },
  { x: 0.3, y: -1.5, r: 0.45 },
];

function segDiscCollides(a: [number, number], b: [number, number], d: Disc): boolean {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const len2 = dx * dx + dy * dy;
  let t = 0;
  if (len2 > 1e-18) t = Math.max(0, Math.min(1, ((d.x - a[0]) * dx + (d.y - a[1]) * dy) / len2));
  const px = a[0] + t * dx;
  const py = a[1] + t * dy;
  return Math.hypot(px - d.x, py - d.y) < d.r;
}

/** Is the arm at q in collision with any disc? */
export function armCollides(q: T2Point, arm: Arm2R = CHOSET_ARM, discs: readonly Disc[] = WORKBENCH_DISCS): boolean {
  const [base, elbow, tip] = links2R(arm, q);
  for (const d of discs) {
    if (segDiscCollides(base, elbow, d) || segDiscCollides(elbow, tip, d)) return true;
  }
  return false;
}

/** Rasterize the C-obstacles on an n × n grid over [−π, π]². */
export function armRaster(n: number, arm: Arm2R = CHOSET_ARM, discs: readonly Disc[] = WORKBENCH_DISCS): BitGrid {
  const grid: BitGrid = { n1: n, n2: n, occupied: new Uint8Array(n * n) };
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      if (armCollides(cellAngles(grid, i, j), arm, discs)) grid.occupied[j * n + i] = 1;
    }
  }
  return grid;
}

/**
 * Joint limits |θ₁| < limit₁, |θ₂| < limit₂ cut the torus open: the cells
 * outside the limits become forbidden. The result is (topologically) an open
 * rectangle — diffeomorphic to ℝ², no longer compact, and no path may wrap.
 */
export function withJointLimits(grid: BitGrid, limit1: number, limit2: number): BitGrid {
  const out: BitGrid = { n1: grid.n1, n2: grid.n2, occupied: Uint8Array.from(grid.occupied) };
  for (let j = 0; j < grid.n2; j++) {
    for (let i = 0; i < grid.n1; i++) {
      const [t1, t2] = cellAngles(grid, i, j);
      if (Math.abs(t1) >= limit1 || Math.abs(t2) >= limit2) out.occupied[j * grid.n1 + i] = 1;
    }
  }
  return out;
}

export interface ComponentReport {
  /** Components counted on the square chart (no identification). */
  onSquare: number;
  /** Components counted on the torus (edges identified). */
  onTorus: number;
  labelsSquare: Int32Array;
  labelsTorus: Int32Array;
}

/** The lab's headline number: k components on the square, k′ ≤ k on the torus. */
export function componentReport(grid: BitGrid): ComponentReport {
  const sq = labelComponents(grid, false);
  const to = labelComponents(grid, true);
  return { onSquare: sq.count, onTorus: to.count, labelsSquare: sq.labels, labelsTorus: to.labels };
}

export interface SeamConnection {
  a: T2Point;
  b: T2Point;
  /** `T2.dist(a, b)`: the length of the geodesic that crosses the seam. */
  dist: number;
  /** The chart's Euclidean distance between the same two cell centres. */
  chartDist: number;
}

/**
 * Find two free cells that lie in different components on the square but the
 * same component on the torus, and are joined by a geodesic across a seam.
 * Returns the closest such pair (by `T2.dist`), or null when the identification
 * merges nothing — which is itself the lab's answer for that raster.
 */
export function seamConnection(grid: BitGrid, report: ComponentReport, rng: Rng = new Rng(5), tries = 4000): SeamConnection | null {
  const free: number[] = [];
  for (let k = 0; k < grid.occupied.length; k++) if (!grid.occupied[k]) free.push(k);
  if (free.length < 2) return null;
  let best: SeamConnection | null = null;
  for (let t = 0; t < tries; t++) {
    const ka = free[Math.floor(rng.uniform(0, free.length))];
    const kb = free[Math.floor(rng.uniform(0, free.length))];
    if (report.labelsSquare[ka] === report.labelsSquare[kb]) continue;
    if (report.labelsTorus[ka] !== report.labelsTorus[kb]) continue;
    const a = cellAngles(grid, ka % grid.n1, Math.floor(ka / grid.n1));
    const b = cellAngles(grid, kb % grid.n1, Math.floor(kb / grid.n1));
    const d = T2.dist(a, b);
    if (!best || d < best.dist) best = { a, b, dist: d, chartDist: Math.hypot(a[0] - b[0], a[1] - b[1]) };
  }
  return best;
}

/** Is the configuration's raster cell free? */
export function isFreeCell(grid: BitGrid, q: T2Point): boolean {
  const [i, j] = cellOf(grid, q);
  return grid.occupied[j * grid.n1 + i] === 0;
}
