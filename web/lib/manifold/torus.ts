/**
 * Tⁿ = S¹ × ⋯ × S¹ — Reach's configuration space, with the ℓ² product metric.
 *
 * Chapter 4 drew T² as the square [−π, π]² and kept saying "the edges are
 * identified". This file is what that means operationally: `T2.dist` and
 * `T2.interpolate` never see a seam, and `foldSquare` is the embedding of the
 * square into ℝ³ that the Torus Unwrapper animates — flat sheet, then cylinder,
 * then doughnut — so the reader can watch a path that is two pieces on the
 * square become one curve on the torus.
 */

import type { Rng } from '../prob/rng';
import type { Chart } from './manifold';
import { s1Dist, s1Interpolate, wrapAngle, TAU } from './s1';

export type T2Point = [number, number];

/** Tⁿ for any n; `T2` below is the instance the book uses most. */
export class Torus implements Chart<number[]> {
  readonly name: string;
  readonly periodic: readonly boolean[];
  readonly period = TAU;

  constructor(readonly dim: number) {
    this.name = dim === 2 ? 'T²' : `T^${dim}`;
    this.periodic = new Array<boolean>(dim).fill(true);
  }

  /** ℓ² combination of per-angle circle distances: a metric by Minkowski's inequality. */
  dist(a: number[], b: number[]): number {
    let s = 0;
    for (let i = 0; i < this.dim; i++) s += s1Dist(a[i], b[i]) ** 2;
    return Math.sqrt(s);
  }

  /** Component-wise short-way geodesic. */
  interpolate(a: number[], b: number[], t: number): number[] {
    const out = new Array<number>(this.dim);
    for (let i = 0; i < this.dim; i++) out[i] = s1Interpolate(a[i], b[i], t);
    return out;
  }

  sample(rng: Rng): number[] {
    const out = new Array<number>(this.dim);
    for (let i = 0; i < this.dim; i++) out[i] = rng.uniform(-Math.PI, Math.PI);
    return out;
  }

  wrap(p: number[]): number[] {
    return p.map(wrapAngle);
  }

  equals(a: number[], b: number[], tol = 1e-9): boolean {
    return this.dist(a, b) <= tol;
  }

  coords(p: number[]): number[] {
    return this.wrap(p);
  }

  fromCoords(c: number[]): number[] {
    return this.wrap(c);
  }
}

export const T2 = new Torus(2);

/** The chart's answer when the seam is ignored: Euclidean distance in the square. */
export function chartDistanceT2(a: T2Point, b: T2Point): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1]);
}

/**
 * Does the short-way geodesic from a to b cross a seam of the [−π, π]² chart?
 * Exactly when `dist` beats the chart distance: the seam is where the chart
 * lies about the space.
 */
export function crossesSeam(a: T2Point, b: T2Point): [boolean, boolean] {
  return [Math.abs(a[0] - b[0]) > Math.PI, Math.abs(a[1] - b[1]) > Math.PI];
}

/**
 * The geodesic as it appears *on the chart*: a list of polyline pieces, split
 * wherever it crosses a seam, so a widget can draw the two disconnected pieces
 * on the square and the one connected curve on the torus from the same points.
 */
export function geodesicPiecesOnChart(a: T2Point, b: T2Point, n = 64): T2Point[][] {
  const pieces: T2Point[][] = [[]];
  let prev: T2Point | null = null;
  for (let i = 0; i <= n; i++) {
    const p = T2.interpolate(a, b, i / n) as T2Point;
    if (prev && (Math.abs(p[0] - prev[0]) > Math.PI || Math.abs(p[1] - prev[1]) > Math.PI)) {
      pieces.push([]);
    }
    pieces[pieces.length - 1].push(p);
    prev = p;
  }
  return pieces.filter((piece) => piece.length > 1);
}

/* -------------------------------------------------------------------------- */
/* Embedding T² in ℝ³ — and the fold from square to doughnut                   */
/* -------------------------------------------------------------------------- */

export type Vec3 = [number, number, number];

/**
 * The standard doughnut: major radius R, tube radius r. Choset's §3.5 point is
 * that T² cannot be embedded in ℝ², which is why it is drawn in ℝ³; the
 * identification of the square's edges is what the embedding *does*.
 */
export function torusEmbed(theta1: number, theta2: number, R = 0.9, r = 0.5): Vec3 {
  const w = R + r * Math.cos(theta2);
  return [w * Math.cos(theta1), w * Math.sin(theta1), r * Math.sin(theta2)];
}

/**
 * Bend a straight segment of length L (coordinate s ∈ [−L/2, L/2]) into an arc
 * subtending `alpha` radians, radius L/alpha, tangent to the original line at
 * s = 0. As alpha → 0 this is the identity; at alpha = 2π the ends meet.
 * Returns (along, outward) in the bending plane.
 */
function bendArc(s: number, L: number, alpha: number): [number, number] {
  if (alpha < 1e-6) return [s, 0];
  const rho = L / alpha;
  const phi = (s / L) * alpha;
  return [rho * Math.sin(phi), rho * (1 - Math.cos(phi))];
}

/** Tube radius of the finished doughnut and its major radius, in chart units. */
export const FOLD_TUBE_RADIUS = 0.5;
export const FOLD_MAJOR_RADIUS = 1.4;

/**
 * The fold animation: (u, v) ∈ [−π, π]² to ℝ³ as a function of `fold` ∈ [0, 1].
 *
 *   fold ∈ [0, ½]  — the u-direction curls into a circle (sheet → cylinder);
 *   fold ∈ [½, 1]  — the cylinder's axis (the v-direction) curls into a circle
 *                    (cylinder → torus).
 *
 * At fold = 0 the map is the identity on the square; at fold = 1 it is a torus
 * with tube radius FOLD_TUBE_RADIUS and major radius FOLD_MAJOR_RADIUS − tube.
 * The two stages are the two edge identifications of T², performed one at a
 * time so the reader can see each seam disappear.
 */
export function foldSquare(u: number, v: number, fold: number): Vec3 {
  const f = Math.min(1, Math.max(0, fold));
  const sA = Math.min(1, f * 2);
  const sB = Math.max(0, f * 2 - 1);

  // Stage A: curl u. Radius shrinks from ∞ to the tube radius.
  const alphaA = sA * TAU;
  const LA = TAU * FOLD_TUBE_RADIUS; // circumference of the finished tube
  const uScaled = (u / Math.PI) * (LA / 2);
  // Keep the sheet's apparent width while flat, shrinking to the tube's circumference as it curls.
  const widthScale = 1 + (1 - sA) * (Math.PI / (LA / 2) - 1);
  const [cx, cy] = bendArc(uScaled * widthScale, LA * widthScale, alphaA);

  // Stage B: curl the axis. The tube's "outward" direction cy rotates with the axis.
  const alphaB = sB * TAU;
  const LB = TAU * FOLD_MAJOR_RADIUS;
  const vScaled = (v / Math.PI) * (LB / 2);
  const vScale = 1 + (1 - sB) * (Math.PI / (LB / 2) - 1);
  const [az, ay] = bendArc(vScaled * vScale, LB * vScale, alphaB);
  const beta = alphaB < 1e-6 ? 0 : (vScaled * vScale) / (LB * vScale / alphaB);
  // Normal of the bent axis, pointing toward the bend centre.
  const nY = Math.cos(beta);
  const nZ = -Math.sin(beta);

  return [cx, ay + cy * nY, az + cy * nZ];
}

/* -------------------------------------------------------------------------- */
/* Connected components on a T² raster — §3.4.3 made a measured quantity        */
/* -------------------------------------------------------------------------- */

export interface BitGrid {
  n1: number;
  n2: number;
  /** 1 where the cell is a C-obstacle, 0 where free; row-major j·n1 + i. */
  occupied: Uint8Array;
}

/**
 * Label the free cells' connected components with 4-connectivity, optionally
 * identifying opposite edges (the torus) or not (the square chart, or a robot
 * with joint limits). Returns the label array (−1 on obstacles) and the count.
 *
 * Wrapping can only merge components, never split them — that monotonicity is
 * the check `torus_components_wrap` asserts, and the Torus Unwrapper displays.
 */
export function labelComponents(grid: BitGrid, wrap: boolean): { labels: Int32Array; count: number } {
  const { n1, n2, occupied } = grid;
  const labels = new Int32Array(n1 * n2).fill(-1);
  let count = 0;
  const stack: number[] = [];
  for (let start = 0; start < n1 * n2; start++) {
    if (occupied[start] || labels[start] >= 0) continue;
    labels[start] = count;
    stack.push(start);
    while (stack.length) {
      const k = stack.pop()!;
      const i = k % n1;
      const j = (k - i) / n1;
      const steps: [number, number][] = [
        [i + 1, j],
        [i - 1, j],
        [i, j + 1],
        [i, j - 1],
      ];
      for (let [ni, nj] of steps) {
        if (wrap) {
          ni = (ni + n1) % n1;
          nj = (nj + n2) % n2;
        } else if (ni < 0 || nj < 0 || ni >= n1 || nj >= n2) continue;
        const nk = nj * n1 + ni;
        if (occupied[nk] || labels[nk] >= 0) continue;
        labels[nk] = count;
        stack.push(nk);
      }
    }
    count++;
  }
  return { labels, count };
}

/** Chart coordinates (θ₁, θ₂) ∈ [−π, π)² of a raster cell's centre. */
export function cellAngles(grid: BitGrid, i: number, j: number): T2Point {
  return [-Math.PI + ((i + 0.5) / grid.n1) * TAU, -Math.PI + ((j + 0.5) / grid.n2) * TAU];
}

/** Raster cell of a configuration, with wraparound. */
export function cellOf(grid: BitGrid, q: T2Point): [number, number] {
  const i = Math.floor(((wrapAngle(q[0]) + Math.PI) / TAU) * grid.n1) % grid.n1;
  const j = Math.floor(((wrapAngle(q[1]) + Math.PI) / TAU) * grid.n2) % grid.n2;
  return [(i + grid.n1) % grid.n1, (j + grid.n2) % grid.n2];
}
