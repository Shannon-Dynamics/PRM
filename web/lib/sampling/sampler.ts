/**
 * The sampler zoo of Choset §7.1.3 — node sampling, "coarse rain".
 *
 * A `Sampler` returns one free configuration or `null` for a rejected attempt.
 * Rejection is the normal case for the biased samplers (the bridge test
 * throws away most pairs by design), so callers loop on attempts, and the
 * `attempts` / `accepted` counters are how w11.2 shows *where each strategy
 * puts its mass* and what it pays to put it there.
 *
 * Everything is generic over Chapter 5's `Chart<P>`: perturbations are done in
 * chart coordinates and mapped back with `fromCoords`, which wraps periodic
 * axes — so the Gaussian sampler on T² samples across the seam without a
 * special case.
 */

import type { Rng } from '../prob/rng';
import type { Chart } from '../manifold/manifold';
import { Rd } from '../manifold/rd';
import type { FreeSpace } from './free-space';

export interface Sampler<P> {
  readonly name: string;
  /** One attempt: a free configuration, or `null` if this attempt was rejected. */
  sample(space: Chart<P>, free: FreeSpace<P>, rng: Rng): P | null;
  attempts: number;
  accepted: number;
}

/** Draw until the sampler accepts, up to `maxAttempts`. */
export function drawFree<P>(s: Sampler<P>, space: Chart<P>, free: FreeSpace<P>, rng: Rng, maxAttempts = 10000): P | null {
  for (let i = 0; i < maxAttempts; i++) {
    const q = s.sample(space, free, rng);
    if (q !== null) return q;
  }
  return null;
}

/* -------------------------------------------------------------------------- */
/* Chart-coordinate helpers shared by the biased samplers                      */
/* -------------------------------------------------------------------------- */

/** A random unit vector in ℝᵈ (normalize a standard Gaussian). */
export function randomDirection(d: number, rng: Rng): number[] {
  for (;;) {
    const v = new Array<number>(d);
    let n = 0;
    for (let i = 0; i < d; i++) {
      v[i] = rng.normal();
      n += v[i] * v[i];
    }
    if (n > 1e-12) {
      const s = 1 / Math.sqrt(n);
      return v.map((x) => x * s);
    }
  }
}

/** q moved by the chart-coordinate vector v, wrapped back onto the manifold. */
export function perturb<P>(space: Chart<P>, q: P, v: readonly number[]): P {
  const c = space.coords(q);
  return space.fromCoords(c.map((x, i) => x + v[i]));
}

/** A uniform point of the chart-coordinate ball of radius r around q. */
export function sampleBall<P>(space: Chart<P>, q: P, r: number, rng: Rng): P {
  const dir = randomDirection(space.dim, rng);
  const rad = r * Math.pow(rng.next(), 1 / space.dim);
  return perturb(space, q, dir.map((x) => x * rad));
}

/* -------------------------------------------------------------------------- */
/* Uniform — Choset §7.1.2: the baseline, and the one Theorem 7.4.1 is about  */
/* -------------------------------------------------------------------------- */

export class Uniform<P> implements Sampler<P> {
  readonly name = 'uniform';
  attempts = 0;
  accepted = 0;

  sample(space: Chart<P>, free: FreeSpace<P>, rng: Rng): P | null {
    this.attempts++;
    const q = space.sample(rng);
    if (!free.isFree(q)) return null;
    this.accepted++;
    return q;
  }
}

/* -------------------------------------------------------------------------- */
/* Gaussian — Boor, Overmars & van der Stappen [C ref. 59]                    */
/* -------------------------------------------------------------------------- */

/**
 * Two configurations, the second a Gaussian step from the first; keep the free
 * one only if the other is in collision. The acceptance probability at q is
 * the probability that a σ-ball around q straddles ∂𝒬𝒪 (Derivation 4), so the
 * mass concentrates within O(σ) of obstacle boundaries — including the two
 * walls of a narrow passage, which is the point.
 */
export class Gaussian<P> implements Sampler<P> {
  readonly name = 'gaussian';
  attempts = 0;
  accepted = 0;

  constructor(readonly sigma: number) {}

  sample(space: Chart<P>, free: FreeSpace<P>, rng: Rng): P | null {
    this.attempts++;
    const q1 = space.sample(rng);
    const step = Math.abs(rng.normal(0, this.sigma));
    const q2 = perturb(space, q1, randomDirection(space.dim, rng).map((x) => x * step));
    const f1 = free.isFree(q1);
    const f2 = free.isFree(q2);
    if (f1 === f2) return null;
    this.accepted++;
    return f1 ? q1 : q2;
  }
}

/* -------------------------------------------------------------------------- */
/* Bridge test — Hsu, Jiang, Reif & Sun [C ref. 193]                          */
/* -------------------------------------------------------------------------- */

/**
 * Two piers in collision, a midpoint over free space: a short bridge is easy
 * to build inside a corridor narrower than the bridge and nearly impossible in
 * open space, so this is the sampler that lands *inside* narrow passages.
 */
export class Bridge<P> implements Sampler<P> {
  readonly name = 'bridge';
  attempts = 0;
  accepted = 0;

  constructor(readonly length: number) {}

  sample(space: Chart<P>, free: FreeSpace<P>, rng: Rng): P | null {
    this.attempts++;
    const q1 = space.sample(rng);
    if (free.isFree(q1)) return null;
    const len = Math.abs(rng.normal(0, this.length));
    const q2 = perturb(space, q1, randomDirection(space.dim, rng).map((x) => x * len));
    if (free.isFree(q2)) return null;
    const qm = space.interpolate(q1, q2, 0.5);
    if (!free.isFree(qm)) return null;
    this.accepted++;
    return qm;
  }
}

/* -------------------------------------------------------------------------- */
/* OBPRM — Amato et al. [C ref. 18]                                            */
/* -------------------------------------------------------------------------- */

/**
 * From a configuration in collision, march in a random direction until free,
 * then bisect back toward the surface. The accepted sample lies within one
 * bisection `step` of ∂𝒬𝒪, on the free side.
 */
export class Obprm<P> implements Sampler<P> {
  readonly name = 'obprm';
  attempts = 0;
  accepted = 0;

  constructor(
    readonly step: number,
    readonly maxMarch = 40,
  ) {}

  sample(space: Chart<P>, free: FreeSpace<P>, rng: Rng): P | null {
    this.attempts++;
    const qIn = space.sample(rng);
    if (free.isFree(qIn)) return null;
    const dir = randomDirection(space.dim, rng);
    let qOut: P | null = null;
    for (let k = 1; k <= this.maxMarch; k++) {
      const q = perturb(space, qIn, dir.map((x) => x * k * this.step));
      if (free.isFree(q)) {
        qOut = q;
        break;
      }
    }
    if (qOut === null) return null;
    // Binary search between a colliding and a free endpoint, keeping the free one.
    let lo = qIn;
    let hi: P = qOut as P;
    for (let i = 0; i < 6; i++) {
      const mid = space.interpolate(lo, hi, 0.5);
      if (free.isFree(mid)) hi = mid;
      else lo = mid;
    }
    this.accepted++;
    return hi;
  }
}

/* -------------------------------------------------------------------------- */
/* Medial-axis retraction — Wilmarth, Amato & Stiller [C ref. 427]            */
/* -------------------------------------------------------------------------- */

/**
 * Push a free sample toward the Chapter 8 GVD by ascending the clearance
 * function: move away from the nearest obstacle point until a second obstacle
 * becomes equidistant, which is where the clearance gradient stops pointing
 * anywhere in particular. Done by finite-difference gradient ascent in chart
 * coordinates on the free space's `clearance` oracle; samplers without one
 * fall back to the uniform sample.
 */
export class MedialAxis<P> implements Sampler<P> {
  readonly name = 'medial-axis';
  attempts = 0;
  accepted = 0;

  constructor(
    readonly clearance: ((q: P) => number) | undefined,
    readonly step: number,
    readonly maxSteps = 30,
  ) {}

  sample(space: Chart<P>, free: FreeSpace<P>, rng: Rng): P | null {
    this.attempts++;
    let q = space.sample(rng);
    if (!free.isFree(q)) return null;
    if (this.clearance) {
      const h = this.step * 0.25;
      let c = this.clearance(q);
      for (let k = 0; k < this.maxSteps; k++) {
        const base = space.coords(q);
        const g = new Array<number>(space.dim);
        let gn = 0;
        for (let i = 0; i < space.dim; i++) {
          const plus = base.slice();
          plus[i] += h;
          const minus = base.slice();
          minus[i] -= h;
          g[i] = (this.clearance(space.fromCoords(plus)) - this.clearance(space.fromCoords(minus))) / (2 * h);
          gn += g[i] * g[i];
        }
        gn = Math.sqrt(gn);
        // On the medial axis the one-sided gradients disagree and the central
        // difference nearly cancels: that is the stopping signal.
        if (gn < 0.5) break;
        const next = perturb(space, q, g.map((x) => (x / gn) * this.step));
        const cn = this.clearance(next);
        if (!(cn > c) || !free.isFree(next)) break;
        q = next;
        c = cn;
      }
    }
    this.accepted++;
    return q;
  }
}

/* -------------------------------------------------------------------------- */
/* Hybrid, Scripted, Halton                                                     */
/* -------------------------------------------------------------------------- */

/** π = Σ wᵢ πᵢ — Choset's weighted hybrid (§7.1.3, "Choosing Among Different Sampling Strategies"). */
export class Hybrid<P> implements Sampler<P> {
  readonly name: string;
  attempts = 0;
  accepted = 0;

  constructor(readonly parts: readonly { sampler: Sampler<P>; weight: number }[]) {
    this.name = `hybrid(${parts.map((p) => `${p.sampler.name}:${p.weight}`).join(',')})`;
  }

  sample(space: Chart<P>, free: FreeSpace<P>, rng: Rng): P | null {
    this.attempts++;
    const i = rng.choice(this.parts.map((p) => p.weight));
    const q = this.parts[i].sampler.sample(space, free, rng);
    if (q !== null) this.accepted++;
    return q;
  }
}

/**
 * Replays a fixed list — the device that makes the hand-traced micro-examples
 * independent of any RNG's bit stream. Returns `null` forever once exhausted.
 */
export class Scripted<P> implements Sampler<P> {
  readonly name = 'scripted';
  attempts = 0;
  accepted = 0;
  private i = 0;

  /**
   * `requireFree = false` replays the list as *targets*, not nodes: Choset's
   * RRT draws q_rand from all of 𝒬 (Algorithm 10, RANDOM_CONFIG), and the
   * Chapter 12 trace deliberately aims at (4, 1.5) on the obstacle's edge —
   * `extend` moves only η toward it and checks what it reaches.
   */
  constructor(
    readonly list: readonly P[],
    readonly requireFree = true,
  ) {}

  get exhausted(): boolean {
    return this.i >= this.list.length;
  }

  sample(_space: Chart<P>, free: FreeSpace<P>, _rng: Rng): P | null {
    if (this.i >= this.list.length) return null;
    this.attempts++;
    const q = this.list[this.i++];
    if (this.requireFree && !free.isFree(q)) return null;
    this.accepted++;
    return q;
  }

  reset(): void {
    this.i = 0;
  }
}

/* -------------------------------------------------------------------------- */
/* The zoo by name — what the widgets' strategy chips construct                 */
/* -------------------------------------------------------------------------- */

export type StrategyName = 'uniform' | 'gaussian' | 'bridge' | 'obprm' | 'medial-axis';

export const STRATEGY_NAMES: readonly StrategyName[] = ['uniform', 'gaussian', 'bridge', 'obprm', 'medial-axis'];

/**
 * One sampler per strategy, scaled by a characteristic length (a robot radius
 * in metres, a link width in radians). The bridge test is mixed 50/50 with
 * uniform sampling, as Hsu, Jiang, Reif & Sun recommend: on its own it never
 * samples open space, so a roadmap of bridges alone cannot be reached from a
 * start or goal standing in a room — it finds the passage and nothing else.
 */
export function strategySampler<P>(name: StrategyName, scale: number, clearance?: (q: P) => number): Sampler<P> {
  switch (name) {
    case 'uniform':
      return new Uniform<P>();
    case 'gaussian':
      return new Gaussian<P>(0.35 * scale);
    case 'bridge':
      return new Hybrid<P>([
        { sampler: new Bridge<P>(1.0 * scale), weight: 0.5 },
        { sampler: new Uniform<P>(), weight: 0.5 },
      ]);
    case 'obprm':
      return new Obprm<P>(0.1 * scale);
    case 'medial-axis':
      return new MedialAxis<P>(clearance, 0.1 * scale);
  }
}

/** Van der Corput radical inverse Φ_b(n) — Choset §7.1.3, quasirandom sampling. */
export function radicalInverse(n: number, base: number): number {
  let inv = 1 / base;
  let x = 0;
  while (n > 0) {
    x += (n % base) * inv;
    n = Math.floor(n / base);
    inv /= base;
  }
  return x;
}

const PRIMES = [2, 3, 5, 7, 11, 13, 17, 19];

/**
 * Halton sequence mapped onto the chart's box (ℝᵈ bounds, or [−π, π) on a
 * periodic axis). Deterministic: two runs at the same n give the same roadmap,
 * which is Exercise 6's point, and the planner is resolution complete rather
 * than probabilistically complete.
 */
export class Halton<P> implements Sampler<P> {
  readonly name = 'halton';
  attempts = 0;
  accepted = 0;
  private n = 1;

  sample(space: Chart<P>, free: FreeSpace<P>, _rng: Rng): P | null {
    this.attempts++;
    const c = new Array<number>(space.dim);
    for (let i = 0; i < space.dim; i++) {
      const u = radicalInverse(this.n, PRIMES[i % PRIMES.length]);
      if (space.periodic[i]) c[i] = -space.period / 2 + u * space.period;
      else if (space instanceof Rd) c[i] = space.bounds.lo[i] + u * (space.bounds.hi[i] - space.bounds.lo[i]);
      else c[i] = u;
    }
    this.n++;
    const q = space.fromCoords(c);
    if (!free.isFree(q)) return null;
    this.accepted++;
    return q;
  }
}

/** Dispersion of a point set under the manifold metric, estimated on probe points (Def. 7.1.2). */
export function dispersion<P>(space: Chart<P>, points: readonly P[], probes: readonly P[]): number {
  let worst = 0;
  for (const x of probes) {
    let best = Infinity;
    for (const p of points) best = Math.min(best, space.dist(x, p));
    worst = Math.max(worst, best);
  }
  return worst;
}

/** The Sukharev bound: N ≥ (1 / 2δ*)ᵈ samples to reach dispersion δ* under L∞ on the unit cube. */
export function sukharevSamples(delta: number, d: number): number {
  return Math.ceil(Math.pow(1 / (2 * delta), d));
}
