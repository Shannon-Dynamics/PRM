/**
 * The owned artifact of Chapter 5: the operations a planner needs from a
 * configuration space, abstracted once.
 *
 * Choset's §3.4 says a configuration space is a manifold — locally like ℝᵏ,
 * usually with no single global chart. The practical consequence, which every
 * planner from Chapter 6 onward depends on, is that a planner must never do
 * arithmetic on coordinates. It must ask the *space* three questions:
 *
 *   dist(a, b)            — how far apart are two configurations? (App. C.2: a metric)
 *   interpolate(a, b, t)  — what lies between them? (the geodesic, the short way)
 *   sample(rng)           — give me a configuration, uniformly w.r.t. the natural measure
 *
 * Pretending T² is ℝ² or SE(2) is ℝ³ gets all three wrong, and the chapter's
 * micro-example shows exactly how. This file holds the interface, the product
 * construction, and the property checks (metric axioms, geodesic property) that
 * every implementation is held to. The Rust side is `trait Manifold`; here the
 * point type is a generic parameter `P` and the space is a value, so a kd-tree
 * (Chapter 11) can be built over any of them without knowing which.
 */

import type { Rng } from '../prob/rng';

export interface Manifold<P> {
  /** Human-readable, e.g. "T²" — used in widget readouts and check names. */
  readonly name: string;
  /** Manifold dimension — the number of local coordinates, NOT the embedding dimension. */
  readonly dim: number;
  /** A metric in the sense of Definition C.2.1: definite, symmetric, triangle inequality. */
  dist(a: P, b: P): number;
  /** The geodesic from a to b at parameter t ∈ [0, 1], taken the short way. */
  interpolate(a: P, b: P, t: number): P;
  /** Uniform with respect to the natural measure (Haar on groups, Lebesgue on boxes). */
  sample(rng: Rng): P;
  /** Canonical representative of a point (wrap angles, renormalize quaternions). */
  wrap(p: P): P;
  /** Equality up to tolerance, *as points of the manifold* (so θ and θ + 2π are equal). */
  equals(a: P, b: P, tol?: number): boolean;
}

/**
 * A manifold that also hands out local coordinates — what Chapter 11's kd-tree
 * splits on. `periodic[i]` says the i-th coordinate wraps with period `period`,
 * so a nearest-neighbour query knows to look across the seam. `coords` is a
 * chart in Choset's sense (Definition 3.4.4) on the part of the space where the
 * coordinates are unique; the seam is where the chart ends.
 */
export interface Chart<P> extends Manifold<P> {
  coords(p: P): number[];
  fromCoords(c: number[]): P;
  readonly periodic: readonly boolean[];
  /** The period of every periodic coordinate (2π for every chart in this book). */
  readonly period: number;
}

/** The first generic planner-shaped function in the book: a geodesic local path. */
export function geodesicPath<P>(m: Manifold<P>, a: P, b: P, n: number): P[] {
  const out: P[] = [];
  for (let i = 0; i <= n; i++) out.push(m.interpolate(a, b, i / n));
  return out;
}

/** Length of a polyline measured with the manifold's own metric. */
export function pathLength<P>(m: Manifold<P>, pts: readonly P[]): number {
  let total = 0;
  for (let i = 1; i < pts.length; i++) total += m.dist(pts[i - 1], pts[i]);
  return total;
}

export interface MetricAxiomReport {
  trials: number;
  /** Largest dist(a, a) observed — must be 0. */
  definiteness: number;
  /** Largest |dist(a, b) − dist(b, a)|. */
  symmetry: number;
  /** Largest positive violation dist(a, c) − dist(a, b) − dist(b, c). */
  triangle: number;
  /** Largest dist(a, b) with a ≠ b that came out non-positive (a definiteness failure the other way). */
  nondegeneracy: number;
}

/**
 * Sampled check of Definition C.2.1. Not a proof — the proofs are in the
 * chapter — but a metric that is wrong anywhere is usually wrong on a random
 * triple, and a planner that sorts neighbours by a non-metric produces
 * nonsense silently.
 */
export function checkMetricAxioms<P>(m: Manifold<P>, rng: Rng, trials = 2000): MetricAxiomReport {
  let definiteness = 0;
  let symmetry = 0;
  let triangle = 0;
  let nondegeneracy = 0;
  for (let k = 0; k < trials; k++) {
    const a = m.sample(rng);
    const b = m.sample(rng);
    const c = m.sample(rng);
    const ab = m.dist(a, b);
    const ba = m.dist(b, a);
    const bc = m.dist(b, c);
    const ac = m.dist(a, c);
    definiteness = Math.max(definiteness, Math.abs(m.dist(a, a)), Math.abs(m.dist(m.wrap(a), a)));
    symmetry = Math.max(symmetry, Math.abs(ab - ba));
    triangle = Math.max(triangle, ac - ab - bc);
    if (!m.equals(a, b, 1e-9) && ab <= 0) nondegeneracy = Math.max(nondegeneracy, 1);
  }
  return { trials, definiteness, symmetry, triangle, nondegeneracy };
}

export interface GeodesicReport {
  trials: number;
  /** Largest dist(interpolate(a,b,0), a) or dist(interpolate(a,b,1), b). */
  endpoint: number;
  /** Largest |dist(a, interpolate(a,b,t)) − t·dist(a,b)| — the geodesic is traversed at unit speed. */
  proportional: number;
  /** Largest dist(a, γ(t)) + dist(γ(t), b) − dist(a, b): a point on a geodesic lies on the triangle's equality case. */
  additivity: number;
}

/** `interpolate` follows the metric's own geodesics: endpoints land, and distance grows linearly in t. */
export function checkGeodesic<P>(m: Manifold<P>, rng: Rng, trials = 500): GeodesicReport {
  let endpoint = 0;
  let proportional = 0;
  let additivity = 0;
  for (let k = 0; k < trials; k++) {
    const a = m.sample(rng);
    const b = m.sample(rng);
    const d = m.dist(a, b);
    endpoint = Math.max(endpoint, m.dist(m.interpolate(a, b, 0), a), m.dist(m.interpolate(a, b, 1), b));
    for (const t of [0.25, 0.5, 0.75, rng.next()]) {
      const g = m.interpolate(a, b, t);
      proportional = Math.max(proportional, Math.abs(m.dist(a, g) - t * d));
      additivity = Math.max(additivity, m.dist(a, g) + m.dist(g, b) - d);
    }
  }
  return { trials, endpoint, proportional, additivity };
}

/**
 * The product manifold A × B with the ℓ² product metric
 * d((a,b),(a',b')) = √(d_A(a,a')² + d_B(b,b')²). Minkowski's inequality is what
 * makes this a metric again (Derivation 4), and component-wise interpolation is
 * its geodesic because the factors do not interact. SE(2) × S¹ for Hitch, and
 * Chapter 14's composite spaces, are built with this.
 */
export class Product<A, B> implements Chart<[A, B]> {
  readonly name: string;
  readonly dim: number;
  readonly periodic: readonly boolean[];
  readonly period: number;

  constructor(
    readonly first: Chart<A>,
    readonly second: Chart<B>,
    /** Exchange rate applied to the second factor's distance (metres per radian, say). Default 1. */
    readonly weight = 1,
  ) {
    this.name = `${first.name} × ${second.name}`;
    this.dim = first.dim + second.dim;
    this.periodic = [...first.periodic, ...second.periodic];
    if (first.period !== second.period) {
      throw new Error('Product: factors must share a period for their periodic coordinates');
    }
    this.period = first.period;
  }

  dist(a: [A, B], b: [A, B]): number {
    const da = this.first.dist(a[0], b[0]);
    const db = this.weight * this.second.dist(a[1], b[1]);
    return Math.hypot(da, db);
  }

  interpolate(a: [A, B], b: [A, B], t: number): [A, B] {
    return [this.first.interpolate(a[0], b[0], t), this.second.interpolate(a[1], b[1], t)];
  }

  sample(rng: Rng): [A, B] {
    return [this.first.sample(rng), this.second.sample(rng)];
  }

  wrap(p: [A, B]): [A, B] {
    return [this.first.wrap(p[0]), this.second.wrap(p[1])];
  }

  equals(a: [A, B], b: [A, B], tol = 1e-9): boolean {
    return this.first.equals(a[0], b[0], tol) && this.second.equals(a[1], b[1], tol);
  }

  coords(p: [A, B]): number[] {
    return [...this.first.coords(p[0]), ...this.second.coords(p[1])];
  }

  fromCoords(c: number[]): [A, B] {
    return [
      this.first.fromCoords(c.slice(0, this.first.dim)),
      this.second.fromCoords(c.slice(this.first.dim)),
    ];
  }
}
