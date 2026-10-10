/**
 * The learned proposal (D22.1) and its completeness-preserving wrapper (D22.2).
 *
 * The net is a conditional categorical over a 12 × 12 coarsening of T²: given
 * the Workbench raster and the query's endpoints it emits logits for every
 * cell, and a sample is a cell drawn from the softmax plus a uniform point
 * inside it. Training is maximum likelihood on the cells of Chapter 11
 * roadmap-path nodes, which by Derivation 2 is minimizing D_KL(p* ‖ p̂_θ).
 *
 * It implements Chapter 11's `Sampler` and nothing else — the free-space test
 * is the same one every other sampler runs. What it may not do is be used on
 * its own: `mixtureSampler` wraps it in Chapter 11's `Hybrid` with `Uniform`
 * at weight λ, and refuses λ = 0, because Derivation 1 needs a floor.
 */

import type { Rng } from '../prob/rng';
import type { Chart } from '../manifold/manifold';
import type { FreeSpace } from '../sampling/free-space';
import { Hybrid, Uniform, type Sampler } from '../sampling/sampler';
import { Mlp, softmax } from './mlp';
import { ENC_DIM, encodeAngles, type EnvEncoding } from './worlds';
import { CoarseHistogram } from './info';

const TAU = 2 * Math.PI;

/** Side of the sampler's cell grid on T²; the categorical has CELLS_N² outcomes. */
export const CELLS_N = 12;
export const CELLS = CELLS_N * CELLS_N;
/** Input width: 64 raster values + cos/sin of the start's and the goal's two angles. */
export const SAMPLER_IN = ENC_DIM + 8;

/** Architecture of the shipped sampler net. */
export const SAMPLER_SIZES: readonly number[] = [SAMPLER_IN, 64, CELLS];

export function samplerFeatures(enc: EnvEncoding, start: readonly number[], goal: readonly number[]): Float64Array {
  const x = new Float64Array(SAMPLER_IN);
  x.set(enc, 0);
  x.set(encodeAngles(start), ENC_DIM);
  x.set(encodeAngles(goal), ENC_DIM + 4);
  return x;
}

const wrap = (a: number) => {
  let x = (a + Math.PI) % TAU;
  if (x < 0) x += TAU;
  return x - Math.PI;
};

/** Cell index of a configuration on the CELLS_N × CELLS_N coarsening (row-major j·n + i). */
export function cellOfConfig(q: readonly number[], n = CELLS_N): number {
  const i = Math.min(n - 1, Math.floor(((wrap(q[0]) + Math.PI) / TAU) * n));
  const j = Math.min(n - 1, Math.floor(((wrap(q[1]) + Math.PI) / TAU) * n));
  return j * n + i;
}

/** Quadrant of a cell (the §3(a) four-cell coarsening): 0 = (−,−), 1 = (+,−), 2 = (−,+), 3 = (+,+). */
export function quadrantOfCell(cell: number, n = CELLS_N): number {
  const i = cell % n;
  const j = (cell - i) / n;
  return (i >= n / 2 ? 1 : 0) + (j >= n / 2 ? 2 : 0);
}

export class LearnedSampler implements Sampler<number[]> {
  readonly name = 'learned';
  attempts = 0;
  accepted = 0;
  /** Cell masses π_c of the conditional categorical — fixed once the query is known. */
  readonly probs: Float64Array;
  private readonly cdf: Float64Array;
  /** Inference cost of this proposal, in milliseconds (one forward pass). */
  readonly inferenceMs: number;

  constructor(
    readonly net: Mlp,
    readonly enc: EnvEncoding,
    readonly start: readonly number[],
    readonly goal: readonly number[],
    readonly n = CELLS_N,
  ) {
    const t0 = typeof performance !== 'undefined' ? performance.now() : Date.now();
    this.probs = softmax(net.forward(samplerFeatures(enc, start, goal)));
    this.inferenceMs = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - t0;
    this.cdf = new Float64Array(this.probs.length);
    let acc = 0;
    for (let i = 0; i < this.probs.length; i++) {
      acc += this.probs[i];
      this.cdf[i] = acc;
    }
    this.cdf[this.cdf.length - 1] = 1;
  }

  /** A cell drawn from the categorical, by binary search on the CDF. */
  drawCell(rng: Rng): number {
    const u = rng.next();
    let lo = 0;
    let hi = this.cdf.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (this.cdf[mid] < u) lo = mid + 1;
      else hi = mid;
    }
    return lo;
  }

  /** The raw proposal q ~ p̂_θ(· | 𝓔, start, goal) — *no* free-space test. */
  drawRaw(rng: Rng): number[] {
    const c = this.drawCell(rng);
    const i = c % this.n;
    const j = (c - i) / this.n;
    const w = TAU / this.n;
    return [-Math.PI + (i + rng.next()) * w, -Math.PI + (j + rng.next()) * w];
  }

  /** Chapter 11's contract: a free configuration or null for a rejected attempt. */
  sample(_space: Chart<number[]>, free: FreeSpace<number[]>, rng: Rng): number[] | null {
    this.attempts++;
    const q = this.drawRaw(rng);
    if (!free.isFree(q)) return null;
    this.accepted++;
    return q;
  }

  /** Density relative to uniform at q: p̂(q) / u(q) = N · π_c. */
  densityRatio(q: readonly number[]): number {
    return this.probs[cellOfConfig(q, this.n)] * this.probs.length;
  }

  /** D_KL(p̂ ‖ u) on the cell partition — the concentration the heat layer shows. */
  klToUniform(): number {
    let s = 0;
    const N = this.probs.length;
    for (let i = 0; i < N; i++) if (this.probs[i] > 0) s += this.probs[i] * Math.log(this.probs[i] * N);
    return s;
  }

  /** The four-quadrant coarsening of the cell masses, for the §3(a) histogram. */
  quadrantMasses(): number[] {
    const out = [0, 0, 0, 0];
    for (let c = 0; c < this.probs.length; c++) out[quadrantOfCell(c, this.n)] += this.probs[c];
    return out;
  }
}

/**
 * p_λ = (1 − λ) p̂_θ + λ u, built from Chapter 11's own `Hybrid` with `Uniform`
 * as the floor. Rejects λ ≤ 0: without the floor Derivation 1's bound is void,
 * and the constructor is where that fact is enforced, not the prose.
 */
export function mixtureSampler(learned: LearnedSampler, lambda: number): Hybrid<number[]> {
  if (!(lambda > 0 && lambda <= 1)) throw new Error(`mixtureSampler: λ must lie in (0, 1], got ${lambda}`);
  return new Hybrid<number[]>([
    { sampler: learned, weight: 1 - lambda },
    { sampler: new Uniform<number[]>(), weight: lambda },
  ]);
}

/** Cell masses of the mixture and the D22.2 floor check — analytic, no sampling. */
export function mixtureMasses(learned: LearnedSampler, lambda: number): { masses: Float64Array; min: number; floor: number; holds: boolean } {
  const N = learned.probs.length;
  const masses = new Float64Array(N);
  let min = Infinity;
  for (let c = 0; c < N; c++) {
    masses[c] = (1 - lambda) * learned.probs[c] + lambda / N;
    if (masses[c] < min) min = masses[c];
  }
  const floor = lambda / N;
  return { masses, min, floor, holds: min >= floor - 1e-12 };
}

/** Histogram of raw draws from a sampler-like `drawRaw`, on the cell grid. */
export function rawDrawHistogram(draw: (rng: Rng) => number[], rng: Rng, count: number, n = CELLS_N): CoarseHistogram {
  const cells = new Int32Array(count);
  for (let k = 0; k < count; k++) cells[k] = cellOfConfig(draw(rng), n);
  return CoarseHistogram.fromCells(cells, n * n);
}

/** Uniform-on-T² draws through the same cell grid, for a side-by-side histogram. */
export function uniformDraw(rng: Rng): number[] {
  return [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
}
