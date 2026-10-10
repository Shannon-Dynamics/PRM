/**
 * ℝⁿ — the one configuration space where coordinates *are* the space.
 *
 * It is here mostly as the base case: a disc robot translating in the plane
 * (Rusty in Part II) lives in ℝ², and the kd-tree of Chapter 11 is first tested
 * here before it has to cope with seams. Sampling needs a box because ℝⁿ is not
 * compact (§3.4.3): there is no uniform distribution on all of it, so the
 * `bounds` are part of the space's description, not an afterthought.
 */

import type { Rng } from '../prob/rng';
import type { Chart } from './manifold';

export interface Box {
  lo: readonly number[];
  hi: readonly number[];
}

export class Rd implements Chart<number[]> {
  readonly name: string;
  readonly periodic: readonly boolean[];
  readonly period = 2 * Math.PI;

  constructor(
    readonly dim: number,
    readonly bounds: Box,
  ) {
    if (bounds.lo.length !== dim || bounds.hi.length !== dim) {
      throw new Error(`Rd(${dim}): bounds must have ${dim} entries`);
    }
    this.name = `ℝ^${dim}`;
    this.periodic = new Array<boolean>(dim).fill(false);
  }

  dist(a: number[], b: number[]): number {
    let s = 0;
    for (let i = 0; i < this.dim; i++) s += (a[i] - b[i]) ** 2;
    return Math.sqrt(s);
  }

  interpolate(a: number[], b: number[], t: number): number[] {
    const out = new Array<number>(this.dim);
    for (let i = 0; i < this.dim; i++) out[i] = a[i] + t * (b[i] - a[i]);
    return out;
  }

  sample(rng: Rng): number[] {
    const out = new Array<number>(this.dim);
    for (let i = 0; i < this.dim; i++) out[i] = rng.uniform(this.bounds.lo[i], this.bounds.hi[i]);
    return out;
  }

  wrap(p: number[]): number[] {
    return p.slice();
  }

  equals(a: number[], b: number[], tol = 1e-9): boolean {
    return this.dist(a, b) <= tol;
  }

  coords(p: number[]): number[] {
    return p.slice();
  }

  fromCoords(c: number[]): number[] {
    return c.slice();
  }
}

/** The plane with a sampling box — Rusty's configuration space as a disc. */
export const R2 = (lo: [number, number], hi: [number, number]) => new Rd(2, { lo, hi });
