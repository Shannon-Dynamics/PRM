/**
 * `estimate/bayes` — Choset's Chapter 9 (§9.1 localization, §9.2 mapping) as
 * thin wrappers over the sister book's vendored filters.
 *
 * Nothing in this file re-implements a Bayes filter, a particle filter, a
 * grid localizer or an occupancy grid. Those are `../filters/bayes` (the
 * histogram filter), `../filters/pf` (Algs. 17–18), `../localize/*` (grid
 * localization and Augmented MCL), `../mapping/occgrid` (Alg. 19 in log odds)
 * and `../filters/rbpf` (Alg. 20). What this file adds is what Choset has and
 * the sister does not, plus the glue a *planner* needs:
 *
 *   - the five-cell corridor of the §3 micro-example (problems 6–7) and
 *     Choset's problem 6 itself, both on the sister's `HistogramFilter1D`;
 *   - the (α, β, d) odometry increment of Fig. 9.5 with its noise model
 *     (9.18–9.21), as an adapter onto the sister's `OdomDelta`;
 *   - the proximity sensor model of §9.1.5: the binned four-way mixture
 *     (9.23–9.24), the product over a subsampled scan (9.25), the 256-row
 *     histogram cache and the ray-cast lookup table, and an ML fit of the
 *     mixture weights;
 *   - a `Belief` interface — weighted samples on the planning lattice, the
 *     mode, the entropy — implemented for the grid and for the particle set,
 *     so `replan.ts` is representation-agnostic;
 *   - one recorded Apartment run that both localizers replay (w16.1);
 *   - mapping with known poses (Alg. 19) over the ported occupancy grid, the
 *     odds form (9.39) and Choset's cone model (9.41–9.42);
 *   - a compact Rao-Blackwellized SLAM demo (Alg. 20) over the ported `GridRbpf`.
 */

import { HistogramFilter1D } from '../filters/bayes';
import { ParticleFilter, type Particle } from '../filters/pf';
import { GridRbpf, DEFAULT_RBPF_OPTIONS, type GridParticle, type RbpfReport } from '../filters/rbpf';
import { normalizeAngle, type Pose2 } from '../geom/se2';
import { GridLocalizer, coarsenedSigma, type GridSpec } from '../localize/grid-localization';
import { SurpriseDetector, augmentedResample, averageWeight, dominantCluster, type SurpriseParams } from '../localize/augmented-mcl';
import { DEFAULT_INVERSE_MODEL, OccupancyGrid, logOddsToProb, probToLogOdds, type InverseModelParams } from '../mapping/occgrid';
import { applyOdom, odomFromPoses, type OdomDelta } from '../models/motion';
import { discreteEntropy, normalCdf, normalPdf } from '../prob/gaussian';
import { Rng } from '../prob/rng';
import { cellAt, type OccupancyGrid as Lattice } from '../search/grid';
import { pursuePoint } from '../sim/rusty';
import { beamAngles, collides, distanceToWalls, isFree, rayCast, simulateScan, type ScanParams, type World } from '../sim/world';

/* -------------------------------------------------------------------------- */
/* §9.1.1 — the corridor, on the sister's histogram filter                     */
/* -------------------------------------------------------------------------- */

export interface CorridorSpec {
  /** Number of places on the loop. */
  cells: number;
  /** Which places carry a door (a landmark). */
  doors: readonly number[];
  /** P(door seen | door cell), P(door seen | wall cell) — Choset problem 6: 0.8 / 0.4. */
  pDoorAtDoor: number;
  pDoorAtWall: number;
  /** P(move succeeds); the robot stays put otherwise — problem 7: 0.8 / 0.2. */
  pMove: number;
}

/** The §3 micro-example: five cells, doors at 1 and 3, problem-6 likelihoods, problem-7 motion. */
export const FIVE_CELLS: CorridorSpec = { cells: 5, doors: [1, 3], pDoorAtDoor: 0.8, pDoorAtWall: 0.4, pMove: 0.8 };

/** Choset problem 6: ten places on a circle, landmarks at 0, 3, 6. */
export const PROBLEM_6: CorridorSpec = { cells: 10, doors: [0, 3, 6], pDoorAtDoor: 0.8, pDoorAtWall: 0.4, pMove: 1 };

export interface SenseResult {
  /** P(y | x) at every cell — the green strip of Fig. 9.1. */
  likelihood: number[];
  /** P(y | x) · P(x) before normalization — Alg. 16 line 8. */
  unnormalized: number[];
  /** η⁻¹ = Σ_x P(y | x) P(x), eq. 9.6 — the running sum of lines 6–10. */
  evidence: number;
  posterior: number[];
}

/**
 * Choset's Fig. 9.1 corridor as an instance of the sister's `HistogramFilter1D`
 * with unit cells on a loop. No filter code lives here: `sense` is its
 * `correct`, `move` is its `predict` with the two-point kernel of problem 7.
 */
export class CorridorFilter {
  readonly filter: HistogramFilter1D;
  readonly spec: CorridorSpec;

  constructor(spec: CorridorSpec = FIVE_CELLS) {
    this.spec = spec;
    this.filter = new HistogramFilter1D({ length: spec.cells, cells: spec.cells, wrap: true });
    this.filter.setUniform();
  }

  belief(): number[] {
    return this.filter.belief();
  }

  isDoor(cell: number): boolean {
    return this.spec.doors.includes(((cell % this.spec.cells) + this.spec.cells) % this.spec.cells);
  }

  /** The likelihood of the reading at every cell. */
  likelihood(sawDoor: boolean): number[] {
    const { pDoorAtDoor, pDoorAtWall } = this.spec;
    return Array.from({ length: this.spec.cells }, (_, c) => {
      const pDoor = this.isDoor(c) ? pDoorAtDoor : pDoorAtWall;
      return sawDoor ? pDoor : 1 - pDoor;
    });
  }

  /** Update step (9.4)–(9.6): multiply, sum, divide. */
  sense(sawDoor: boolean): SenseResult {
    const likelihood = this.likelihood(sawDoor);
    const prior = this.filter.belief();
    const unnormalized = prior.map((p, c) => p * likelihood[c]);
    this.filter.correct((x) => likelihood[Math.floor(x)]);
    return { likelihood, unnormalized, evidence: this.filter.lastEvidence, posterior: this.filter.belief() };
  }

  /**
   * Prediction step (9.3) for "move `steps` places": with probability pMove the
   * robot lands `steps` ahead, otherwise it stays. The kernel is written on the
   * displacement error the filter hands it (landed ⇒ 0, stayed ⇒ −steps), with
   * the loop's wrap taken into account so a 3-step move on a 5-cell loop is not
   * mistaken for a 2-step one.
   */
  move(steps: number): number[] {
    const { cells, pMove } = this.spec;
    const wrap = (d: number) => d - cells * Math.round(d / cells);
    this.filter.predict(steps, (d) => {
      if (Math.abs(wrap(d)) < 0.5) return pMove;
      if (Math.abs(wrap(d + steps)) < 0.5) return 1 - pMove;
      return 0;
    });
    return this.filter.belief();
  }

  entropyBits(): number {
    return this.filter.entropy();
  }

  mode(): number {
    return Math.floor(this.filter.mode());
  }
}

/** Entropy in bits of any discrete belief — the number printed under every stage of f16.3. */
export const entropyBits = (p: readonly number[]): number => discreteEntropy([...p]);

export interface FiveCellTrace {
  prior: number[];
  sense: SenseResult;
  moved: number[];
  entropy: { prior: number; sensed: number; moved: number };
}

/** The §3 Part A micro-example, end to end: uniform → sense door → move right 1. */
export function fiveCellSenseMove(spec: CorridorSpec = FIVE_CELLS): FiveCellTrace {
  const f = new CorridorFilter(spec);
  const prior = f.belief();
  const hPrior = f.entropyBits();
  const sense = f.sense(true);
  const hSensed = f.entropyBits();
  const moved = f.move(1);
  const hMoved = f.entropyBits();
  return { prior, sense, moved, entropy: { prior: hPrior, sensed: hSensed, moved: hMoved } };
}

/**
 * Choset problem 6 (and problem 7 with `pMove = 0.8`): detect a landmark, move
 * 3 counterclockwise, detect a landmark, move 4, detect none.
 */
export function chosetProblem6(pMove = 1): number[] {
  const f = new CorridorFilter({ ...PROBLEM_6, pMove });
  f.sense(true);
  f.move(3);
  f.sense(true);
  f.move(4);
  f.sense(false);
  return f.belief();
}

/* -------------------------------------------------------------------------- */
/* §9.1.4 — Choset's odometry increment (α, β, d), Fig. 9.5, eqs. 9.18–9.21    */
/* -------------------------------------------------------------------------- */

/** An incremental motion: rotate α toward the destination, drive d, rotate β to the final heading. */
export interface ChosetIncrement {
  alpha: number;
  d: number;
  beta: number;
}

/** σ₁ … σ₆ of eqs. 9.19–9.21. Units: σ₁, σ₃, σ₅ dimensionless; σ₂, σ₄ rad/m; σ₆ m/rad. */
export type ChosetSigmas = [number, number, number, number, number, number];

export const DEFAULT_CHOSET_SIGMAS: ChosetSigmas = [0.08, 0.03, 0.08, 0.03, 0.08, 0.02];

/** (α, β, d) is the sister's rotate–translate–rotate decomposition under other letters. */
export const toOdomDelta = (u: ChosetIncrement): OdomDelta => ({ rot1: u.alpha, trans: u.d, rot2: u.beta });
export const fromOdomDelta = (u: OdomDelta): ChosetIncrement => ({ alpha: u.rot1, d: u.trans, beta: u.rot2 });

/** The increment that carries x₁ to x₂ exactly. */
export function incrementBetween(x1: Pose2, x2: Pose2): ChosetIncrement {
  return fromOdomDelta(odomFromPoses(x1, x2));
}

/** Eq. 9.18 with α′ = α, β′ = β, d′ = d: the noise-free motion. */
export function applyIncrement(x: Pose2, u: ChosetIncrement): Pose2 {
  return applyOdom(x, toOdomDelta(u));
}

/**
 * Eqs. 9.18–9.21: perturb (α, β, d) by Gaussians whose widths grow with the
 * rotations *and* the translation, then apply. This is Choset's sampler; the
 * sister's Table 5.6 is the same idea with its α's squared into variances.
 */
export function sampleChosetMotion(x: Pose2, u: ChosetIncrement, s: ChosetSigmas, rng: Rng): Pose2 {
  const [s1, s2, s3, s4, s5, s6] = s;
  const n = (sigma: number) => (sigma > 0 ? rng.normal(0, sigma) : 0);
  const alpha = u.alpha + u.alpha * n(s1) + u.d * n(s2);
  const beta = u.beta + u.beta * n(s3) + u.d * n(s4);
  const d = u.d + u.d * n(s5) + (u.alpha + u.beta) * n(s6);
  return {
    x: x.x + d * Math.cos(x.theta + alpha),
    y: x.y + d * Math.sin(x.theta + alpha),
    theta: normalizeAngle(x.theta + alpha + beta),
  };
}

/* -------------------------------------------------------------------------- */
/* §9.1.5 — the proximity sensor model, eqs. 9.22–9.25                          */
/* -------------------------------------------------------------------------- */

export interface ProximityParams {
  /** σ of the hit Gaussian N(y; d, σ), situation 1. */
  sigma: number;
  /** Rate of the short exponential λe^{−λy}, situation 2. */
  lambda: number;
  /** Mixture weights of the four situations; they sum to one so (9.24) holds by construction. */
  alpha: number;
  beta: number;
  gamma: number;
  delta: number;
  /** Bin width Δ of the discretized range, metres. Choset's problem 3 says 3 inches. */
  deltaBin: number;
  maxRange: number;
}

/** A laser: tight hit peak, few short returns. */
export const LASER_PROXIMITY: ProximityParams = {
  sigma: 0.08,
  lambda: 1.0,
  alpha: 0.78,
  beta: 0.06,
  gamma: 0.12,
  delta: 0.04,
  deltaBin: 0.0762,
  maxRange: 8,
};

/**
 * The laser model as a *localizer* uses it. Choset: "the variances of the
 * Gaussians do not depend solely on the accuracy of the sensor. They also encode
 * the uncertainty of the map." With σ = 0.08 a uniform cloud of a few hundred
 * particles never lands within one σ of the truth and starves (problem 9); at
 * σ = 0.25 the same cloud converges in a handful of scans. The check file pins both.
 */
export const LOCALIZATION_PROXIMITY: ProximityParams = { ...LASER_PROXIMITY, sigma: 0.25 };

/** Ultrasound: wide peak, many short and random returns — Fig. 9.10 versus Fig. 9.11. */
export const ULTRASOUND_PROXIMITY: ProximityParams = {
  ...LASER_PROXIMITY,
  sigma: 0.3,
  alpha: 0.55,
  beta: 0.15,
  gamma: 0.2,
  delta: 0.1,
};

/**
 * Choset's histogram sensor model. Row i is the distribution of the measured
 * bin j given expected distance d_i (9.23); the row sums to one (9.24). Rows
 * are computed lazily and cached — with 256 bins and 256 expected distances
 * that is the "256 histograms with 256 bins each" of §9.1.5.
 *
 * The constants α, β, γ, δ are mixture *weights*. Choset leaves them as free
 * constants and imposes (9.24) afterwards; normalizing each component on
 * [0, d_max) and making the weights sum to one is the same model with the
 * constraint built in, and it is what the ML fit below estimates.
 */
export class ProximityModel {
  readonly params: ProximityParams;
  /** Number of bins, the last one being the max-range reading. */
  readonly n: number;
  private readonly rows: (Float64Array | null)[];

  constructor(params: ProximityParams = LASER_PROXIMITY) {
    this.params = params;
    this.n = Math.ceil(params.maxRange / params.deltaBin) + 1;
    this.rows = new Array<Float64Array | null>(this.n).fill(null);
  }

  /** Bin index of a distance; anything at or beyond d_max is the max-range bin n−1. */
  bin(d: number): number {
    const { deltaBin, maxRange } = this.params;
    if (d >= maxRange - 1e-9) return this.n - 1;
    return Math.max(0, Math.min(this.n - 2, Math.floor(d / deltaBin)));
  }

  /** Centre of bin j, metres. */
  binCenter(j: number): number {
    return j === this.n - 1 ? this.params.maxRange : (j + 0.5) * this.params.deltaBin;
  }

  /**
   * Masses of the three continuous components in every non-max bin, each
   * normalized to one over [0, d_max). Shared by `row` and by the ML fit.
   */
  componentMasses(dExpected: number): { hit: Float64Array; short: Float64Array; random: Float64Array } {
    const { sigma, lambda, deltaBin, maxRange } = this.params;
    const m = this.n - 1;
    const hit = new Float64Array(m);
    const short = new Float64Array(m);
    const random = new Float64Array(m);
    const hitNorm = Math.max(normalCdf(maxRange, dExpected, sigma) - normalCdf(0, dExpected, sigma), 1e-12);
    const shortNorm = 1 - Math.exp(-lambda * maxRange);
    for (let j = 0; j < m; j++) {
      const lo = j * deltaBin;
      const hi = Math.min((j + 1) * deltaBin, maxRange);
      hit[j] = (normalCdf(hi, dExpected, sigma) - normalCdf(lo, dExpected, sigma)) / hitNorm;
      short[j] = (Math.exp(-lambda * lo) - Math.exp(-lambda * hi)) / shortNorm;
      random[j] = (hi - lo) / maxRange;
    }
    return { hit, short, random };
  }

  /** Histogram h_i: P(bin j | expected distance in bin i), eq. 9.23, summing to one (9.24). */
  row(i: number): Float64Array {
    const cached = this.rows[i];
    if (cached) return cached;
    const { alpha, beta, gamma, delta } = this.params;
    const d = this.binCenter(Math.min(i, this.n - 2));
    const { hit, short, random } = this.componentMasses(d);
    const h = new Float64Array(this.n);
    for (let j = 0; j < this.n - 1; j++) h[j] = alpha * hit[j] + beta * short[j] + gamma * random[j];
    h[this.n - 1] = delta;
    // (9.24): the continuous part integrates to α + β + γ; scale it to 1 − δ exactly.
    let s = 0;
    for (let j = 0; j < this.n - 1; j++) s += h[j];
    const scale = s > 0 ? (1 - delta) / s : 0;
    for (let j = 0; j < this.n - 1; j++) h[j] *= scale;
    this.rows[i] = h;
    return h;
  }

  /** P(y | d) = h_{bin(d), bin(y)}, eq. 9.22 with the table doing the work. */
  likelihood(y: number, dExpected: number): number {
    return this.row(this.bin(dExpected))[this.bin(y)];
  }

  /**
   * log Π_k P(y_k | d_k) over every `every`-th beam — eq. 9.25 with Choset's
   * remedy for its false independence assumption (Fox et al. used 60 of 181).
   */
  logScanLikelihood(ranges: readonly number[], expected: readonly number[], every = 1): number {
    let q = 0;
    for (let k = 0; k < ranges.length; k += every) {
      q += Math.log(Math.max(this.likelihood(ranges[k], expected[k]), 1e-300));
    }
    return q;
  }

  /** Bytes of the full 256-row cache if every row were materialized. */
  get cacheBytes(): number {
    return this.n * this.n * 8;
  }
}

/** The ray-cast expected distance d(x) along one bearing — the quantity (9.22) conditions on. */
export function expectedDistance(world: World, pose: Pose2, bearing: number, maxRange: number): number {
  return rayCast(world, pose.x, pose.y, pose.theta + bearing, maxRange);
}

/** Expected scan at a pose: d(x) for every beam angle. */
export function expectedScan(world: World, pose: Pose2, angles: readonly number[], maxRange: number): number[] {
  return angles.map((a) => rayCast(world, pose.x, pose.y, pose.theta + a, maxRange));
}

/**
 * Log-likelihood maximization of the mixture weights (§9.1.5, "a typical
 * approach to determine the optimal values … is log-likelihood maximization"),
 * by EM over (α, β, γ, δ) with σ and λ held fixed. Each datum is a (measured,
 * expected) pair. Returns the fitted weights and the final mean log-likelihood.
 */
export function fitMixtureWeights(
  data: readonly { y: number; d: number }[],
  base: ProximityParams,
  iters = 60,
): { params: ProximityParams; logLikelihood: number } {
  const model = new ProximityModel(base);
  let w = [base.alpha, base.beta, base.gamma, base.delta];
  const comps = data.map(({ y, d }) => {
    const j = model.bin(y);
    if (j === model.n - 1) return [0, 0, 0, 1];
    const c = model.componentMasses(model.binCenter(model.bin(d)));
    return [c.hit[j], c.short[j], c.random[j], 0];
  });
  let ll = -Infinity;
  for (let it = 0; it < iters; it++) {
    const acc = [0, 0, 0, 0];
    ll = 0;
    for (const c of comps) {
      const p = c.map((v, k) => w[k] * v);
      const s = p[0] + p[1] + p[2] + p[3];
      if (s <= 0) continue;
      ll += Math.log(s);
      for (let k = 0; k < 4; k++) acc[k] += p[k] / s;
    }
    const total = acc[0] + acc[1] + acc[2] + acc[3];
    if (total <= 0) break;
    w = acc.map((a) => a / total);
  }
  return {
    params: { ...base, alpha: w[0], beta: w[1], gamma: w[2], delta: w[3] },
    logLikelihood: data.length > 0 ? ll / data.length : 0,
  };
}

/** Draw one reading from the mixture — the generative twin of `likelihood`, for tests. */
export function sampleProximity(dExpected: number, p: ProximityParams, rng: Rng): number {
  const u = rng.next();
  if (u < p.delta) return p.maxRange;
  if (u < p.delta + p.gamma) return rng.uniform(0, p.maxRange);
  if (u < p.delta + p.gamma + p.beta) {
    // Exponential truncated to [0, d_max): inverse CDF.
    const z = rng.next() * (1 - Math.exp(-p.lambda * p.maxRange));
    return -Math.log(1 - z) / p.lambda;
  }
  for (let tries = 0; tries < 50; tries++) {
    const y = rng.normal(dExpected, p.sigma);
    if (y >= 0 && y < p.maxRange) return y;
  }
  return Math.max(0, Math.min(p.maxRange - 1e-6, dExpected));
}

/* -------------------------------------------------------------------------- */
/* The expected-distance lookup table of §9.1.5                               */
/* -------------------------------------------------------------------------- */

/**
 * "Precompute all expected distances and store them in a large lookup table …
 * if one limits the resolution of the range data such that each beam can have
 * no more than 256 values, only one byte is needed for each entry." One byte
 * per (cell, heading bin): the ray cast d(x) for a beam leaving cell (i, j)
 * along the centre of heading bin k. A beam with bearing φ_b from a robot in
 * heading bin k reads bin (k + b) mod nθ, so the table is indexed by absolute
 * heading and shared by every beam of every scan.
 */
export class ExpectedDistanceTable {
  readonly codes: Uint8Array;
  readonly quantum: number;
  readonly nx: number;
  readonly ny: number;
  readonly nTheta: number;
  /** How many ray casts the table cost to build — the "once" that pays for every later lookup. */
  readonly rayCasts: number;

  constructor(world: World, g: GridLocalizer, readonly maxRange: number) {
    this.nx = g.nx;
    this.ny = g.ny;
    this.nTheta = g.nTheta;
    this.quantum = maxRange / 255;
    this.codes = new Uint8Array(g.nx * g.ny * g.nTheta);
    let casts = 0;
    for (let k = 0; k < g.nTheta; k++) {
      const theta = -Math.PI + (k + 0.5) * g.dTheta;
      for (let j = 0; j < g.ny; j++) {
        for (let i = 0; i < g.nx; i++) {
          if (!g.free[j * g.nx + i]) continue;
          const [cx, cy] = g.cellCenterXY(i, j);
          const d = rayCast(world, cx, cy, theta, maxRange);
          this.codes[g.index(i, j, k)] = Math.min(255, Math.round(d / this.quantum));
          casts++;
        }
      }
    }
    this.rayCasts = casts;
  }

  /** Expected distance from cell (i, j) along absolute heading bin k (wrapped). */
  at(i: number, j: number, k: number): number {
    const kk = ((k % this.nTheta) + this.nTheta) % this.nTheta;
    return this.codes[(kk * this.ny + j) * this.nx + i] * this.quantum;
  }

  get bytes(): number {
    return this.codes.byteLength;
  }
}

/* -------------------------------------------------------------------------- */
/* Beliefs a planner can consume                                               */
/* -------------------------------------------------------------------------- */

/** A weighted hypothesis on the planning lattice: Σ weight = 1. */
export interface WeightedCell {
  cell: number;
  weight: number;
}

/**
 * What `replan.ts` needs from any posterior: weighted samples on the Ch. 6
 * lattice, the mode, and the entropy. Implemented by the grid belief, the
 * particle belief, and (in the Rust) the Ch. 15 Gaussian.
 */
export interface Belief {
  samples(k: number, lattice: Lattice, rng: Rng): WeightedCell[];
  mode(lattice: Lattice): number;
  modePose(): Pose2;
  meanXY(): { x: number; y: number };
  entropyBits(): number;
}

/** Merge duplicate cells and renormalize — several particles often share one lattice cell. */
export function consolidateCells(raw: readonly WeightedCell[]): WeightedCell[] {
  const acc = new Map<number, number>();
  let total = 0;
  for (const { cell, weight } of raw) {
    acc.set(cell, (acc.get(cell) ?? 0) + weight);
    total += weight;
  }
  const out: WeightedCell[] = [];
  for (const [cell, w] of acc) out.push({ cell, weight: total > 0 ? w / total : 0 });
  out.sort((a, b) => b.weight - a.weight);
  return out;
}

export interface GridBeliefOptions {
  spec: GridSpec;
  sensor?: ProximityParams;
  /** Use every `beamStride`-th heading bin as a beam: Choset's 60-of-181 subsampling. */
  beamStride?: number;
  motionNoise?: { sigmaTrans: number; sigmaRot: number };
}

/**
 * Alg. 16 over the sister's `GridLocalizer`, with Choset's sensor model fed by
 * the one-byte expected-distance table. The beams it uses are heading bins, so
 * the scan handed to `correct` must be indexed by absolute bearing bins too
 * (`scanBinsFor` does that from a recorded scan).
 */
export class GridBelief implements Belief {
  readonly grid: GridLocalizer;
  readonly table: ExpectedDistanceTable;
  readonly model: ProximityModel;
  readonly beamStride: number;
  readonly motionNoise: { sigmaTrans: number; sigmaRot: number };
  /** Per-beam average likelihood p̃ of the last correction, §9.1.4's failure monitor. */
  pTilde = 1;
  lastStepMs = 0;

  constructor(world: World, opts: GridBeliefOptions) {
    this.grid = new GridLocalizer(world, opts.spec);
    // The sister's coarsening correction: a cell centre stands for a whole cell,
    // so the hit peak is widened by half the cell diagonal, in quadrature.
    const base = opts.sensor ?? LOCALIZATION_PROXIMITY;
    this.model = new ProximityModel({ ...base, sigma: coarsenedSigma(base.sigma, opts.spec.cellSize) });
    this.table = new ExpectedDistanceTable(world, this.grid, this.model.params.maxRange);
    this.beamStride = Math.max(1, opts.beamStride ?? 1);
    this.motionNoise = opts.motionNoise ?? { sigmaTrans: 0.08, sigmaRot: 0.06 };
  }

  get nBeams(): number {
    return Math.floor(this.grid.nTheta / this.beamStride);
  }

  /** Line 4 of Alg. 16 by shift-and-smear: the sister's bounded-kernel scatter. */
  predict(u: ChosetIncrement): void {
    const t0 = now();
    this.grid.predict(toOdomDelta(u), this.motionNoise);
    this.lastStepMs += now() - t0;
  }

  /**
   * Lines 6–13 of Alg. 16. `scanByBin[b]` is the reading along *relative* heading
   * bin b·stride, so a cell in heading bin k compares it against the table at
   * absolute bin k + b·stride. Returns p̃ — the normalizer η⁻¹ of (9.6) put on a
   * per-beam scale so a threshold can be chosen once.
   */
  correct(scanByBin: readonly number[]): number {
    const t0 = now();
    const g = this.grid;
    const stride = this.beamStride;
    const logEvidence = g.correctLog((pose) => {
      const i = Math.min(g.nx - 1, Math.max(0, Math.floor((pose.x - g.bounds.minX) / g.cellSize)));
      const j = Math.min(g.ny - 1, Math.max(0, Math.floor((pose.y - g.bounds.minY) / g.cellSize)));
      const k = Math.min(g.nTheta - 1, Math.floor((normalizeAngle(pose.theta) + Math.PI) / g.dTheta));
      let q = 0;
      for (let b = 0; b < scanByBin.length; b++) {
        const d = this.table.at(i, j, k + b * stride);
        q += Math.log(Math.max(this.model.likelihood(scanByBin[b], d), 1e-300));
      }
      return q;
    });
    this.pTilde = Math.exp(logEvidence / Math.max(1, scanByBin.length));
    this.lastStepMs += now() - t0;
    return this.pTilde;
  }

  /** Burgard et al.'s remedy for the kidnapped robot: restart global localization when p̃ dives. */
  restart(): void {
    this.grid.setUniform();
  }

  /** Readings indexed by relative heading bin, picked out of a dense recorded scan. */
  scanBinsFor(scan: readonly number[], scanAngles: readonly number[]): number[] {
    const out: number[] = [];
    for (let b = 0; b < this.nBeams; b++) {
      const bearing = b * this.beamStride * this.grid.dTheta;
      // The beam whose bearing is nearest the bin boundary; recorded scans are dense enough.
      let best = 0;
      let bestDiff = Infinity;
      for (let s = 0; s < scanAngles.length; s++) {
        const diff = Math.abs(normalizeAngle(scanAngles[s] - bearing));
        if (diff < bestDiff) {
          bestDiff = diff;
          best = s;
        }
      }
      out.push(scan[best]);
    }
    return out;
  }

  /** The K heaviest (x, y) cells of the marginal, renormalized: a deterministic sample set. */
  samples(k: number, lattice: Lattice): WeightedCell[] {
    const m = this.grid.marginalXY();
    const idx: number[] = [];
    for (let n = 0; n < m.length; n++) if (m[n] > 0) idx.push(n);
    idx.sort((a, b) => m[b] - m[a]);
    const raw: WeightedCell[] = [];
    for (const n of idx.slice(0, k)) {
      const [cx, cy] = this.grid.cellCenterXY(n % this.grid.nx, Math.floor(n / this.grid.nx));
      raw.push({ cell: cellAt(lattice, cx, cy), weight: m[n] });
    }
    return consolidateCells(raw);
  }

  mode(lattice: Lattice): number {
    const { pose } = this.grid.argmax();
    return cellAt(lattice, pose.x, pose.y);
  }

  modePose(): Pose2 {
    return this.grid.argmax().pose;
  }

  meanXY(): { x: number; y: number } {
    return this.grid.meanXY();
  }

  entropyBits(): number {
    return this.grid.entropyBits();
  }

  get bytes(): number {
    return this.grid.bytes + this.table.bytes;
  }
}

export interface ParticleBeliefOptions {
  n: number;
  sensor?: ProximityParams;
  sigmas?: ChosetSigmas;
  /** Use every `every`-th beam of the recorded scan. */
  every?: number;
  /** Augmented MCL gains; omit to disable injection (plain Alg. 17). */
  surprise?: SurpriseParams | null;
  /**
   * Lenser and Veloso's sensor resetting [C ref. 275], as an absolute floor:
   * when the per-beam average likelihood p̃ falls below `resetFloor`, a fraction
   * `resetFraction` of the cloud is redrawn uniformly over free space. The
   * Augmented-MCL detector only reacts to a *dive* in p̃; a cloud that was
   * wrong from its first scan never dives, and this is the remedy for that.
   * 0 (the default) disables it.
   */
  resetFloor?: number;
  resetFraction?: number;
  clearance?: number;
}

/**
 * Algs. 17–18 over the sister's `ParticleFilter`, with Choset's motion sampler
 * (9.18–9.21), his ray-cast sensor model, and — behind a flag — the sister's
 * Augmented MCL injection as the kidnapped-robot remedy of Fox et al.
 */
export class ParticleBelief implements Belief {
  pf: ParticleFilter;
  readonly model: ProximityModel;
  readonly sigmas: ChosetSigmas;
  readonly every: number;
  readonly detector: SurpriseDetector | null;
  readonly resetFloor: number;
  readonly resetFraction: number;
  readonly clearance: number;
  readonly world: World;
  pTilde = 1;
  injected = 0;
  lastStepMs = 0;

  constructor(world: World, rng: Rng, opts: ParticleBeliefOptions) {
    this.world = world;
    this.clearance = opts.clearance ?? 0.15;
    this.pf = ParticleFilter.uniformInWorld(opts.n, world, rng, this.clearance);
    this.model = new ProximityModel(opts.sensor ?? LOCALIZATION_PROXIMITY);
    this.sigmas = opts.sigmas ?? DEFAULT_CHOSET_SIGMAS;
    this.every = Math.max(1, opts.every ?? 1);
    this.detector = opts.surprise === null ? null : new SurpriseDetector(opts.surprise);
    this.resetFloor = opts.resetFloor ?? 0;
    this.resetFraction = opts.resetFraction ?? 0.1;
  }

  /** Position tracking from a mixture of Gaussian clusters — the w16.2 default belief. */
  static clustered(
    world: World,
    rng: Rng,
    n: number,
    clusters: readonly { pose: Pose2; weight: number; std: number }[],
    opts: Omit<ParticleBeliefOptions, 'n'> = {},
  ): ParticleBelief {
    const b = new ParticleBelief(world, rng, { ...opts, n });
    const particles: Particle[] = [];
    for (const c of clusters) {
      const count = Math.round(n * c.weight);
      for (let i = 0; i < count; i++) {
        let state: Pose2 = c.pose;
        for (let tries = 0; tries < 20; tries++) {
          state = { x: rng.normal(c.pose.x, c.std), y: rng.normal(c.pose.y, c.std), theta: rng.normal(c.pose.theta, 0.2) };
          if (isFree(world, state.x, state.y, b.clearance)) break;
        }
        particles.push({ state, weight: 1 / n });
      }
    }
    b.pf = new ParticleFilter(particles);
    return b;
  }

  get size(): number {
    return this.pf.size;
  }

  /** Alg. 17 lines 2–5 with (9.18)–(9.21); particles that cross a wall are left where they were, as Choset suggests. */
  predict(u: ChosetIncrement, rng: Rng): void {
    const t0 = now();
    this.pf.predict((x) => {
      const next = sampleChosetMotion(x, u, this.sigmas, rng);
      return collides(this.world, x, next) ? x : next;
    });
    this.lastStepMs += now() - t0;
  }

  /** Alg. 17 lines 6–13: w_j = P(y | x_j) by ray casting d(x_j) per beam; returns p̃ = N⁻¹ Σ w_j on the per-beam scale. */
  correct(scan: readonly number[], angles: readonly number[]): number {
    const t0 = now();
    const maxRange = this.model.params.maxRange;
    const nBeams = Math.ceil(scan.length / this.every);
    const logs = this.pf.particles.map((p) => {
      if (!isFree(this.world, p.state.x, p.state.y, 0.05)) return -Infinity;
      let q = 0;
      for (let k = 0; k < scan.length; k += this.every) {
        const d = rayCast(this.world, p.state.x, p.state.y, p.state.theta + angles[k], maxRange);
        q += Math.log(Math.max(this.model.likelihood(scan[k], d), 1e-300));
      }
      return q;
    });
    // p̃ before normalization, on a per-beam scale so the detector's gains are portable.
    const unnormalized = logs.map((l) => (Number.isFinite(l) ? Math.exp(l / nBeams) : 0));
    this.pTilde = averageWeight(unnormalized.map((w) => ({ state: { x: 0, y: 0, theta: 0 }, weight: w })));
    let i = 0;
    this.pf.correctLog(() => logs[i++]);
    this.lastStepMs += now() - t0;
    return this.pTilde;
  }

  /**
   * The bumper is a sensor too (Chapter 3's contact sensor). Before the move,
   * each particle predicts whether the commanded step would hit a wall of the
   * map; P(bump | blocked) = `hitGivenWall`, P(bump | free) = `hitGivenFree`.
   * A bump in a room whose mirror image has a doorway is exactly the evidence
   * that separates the two — and a clean step is weak evidence against every
   * hypothesis that stands at a wall.
   */
  correctContact(u: ChosetIncrement, bumped: boolean, p: { hitGivenWall: number; hitGivenFree: number } = { hitGivenWall: 0.9, hitGivenFree: 0.05 }): void {
    const t0 = now();
    this.pf.correct((x) => {
      const next = applyIncrement(x, u);
      const blocked = collides(this.world, x, next) || !isFree(this.world, next.x, next.y, this.clearance);
      const pBump = blocked ? p.hitGivenWall : p.hitGivenFree;
      return bumped ? pBump : 1 - pBump;
    });
    this.lastStepMs += now() - t0;
  }

  /** Alg. 18, with the Augmented-MCL injection when the surprise detector asks for it, or the sensor-resetting floor. */
  resample(rng: Rng): void {
    const t0 = now();
    let pInject = this.detector ? this.detector.update(this.pTilde) : 0;
    if (this.resetFloor > 0 && this.pTilde < this.resetFloor) pInject = Math.max(pInject, this.resetFraction);
    if (pInject > 0) {
      const { particles, injected } = augmentedResample(this.pf.particles, rng, pInject, (r) => {
        for (let tries = 0; tries < 200; tries++) {
          const x = r.uniform(this.world.bounds.minX, this.world.bounds.maxX);
          const y = r.uniform(this.world.bounds.minY, this.world.bounds.maxY);
          if (isFree(this.world, x, y, this.clearance)) return { x, y, theta: r.uniform(-Math.PI, Math.PI) };
        }
        return { x: 0, y: 0, theta: 0 };
      });
      this.pf.particles = particles;
      this.injected = injected;
    } else {
      this.pf.resample(rng, 'lowVariance');
      this.injected = 0;
    }
    this.lastStepMs += now() - t0;
  }

  /** K draws by the low-variance comb, mapped to lattice cells and merged. */
  samples(k: number, lattice: Lattice, rng: Rng): WeightedCell[] {
    const ps = this.pf.particles;
    if (ps.length === 0) return [];
    let total = 0;
    for (const p of ps) total += p.weight;
    const step = total / k;
    const r = rng.uniform(0, step);
    const raw: WeightedCell[] = [];
    let c = ps[0].weight;
    let i = 0;
    for (let m = 0; m < k; m++) {
      const u = r + m * step;
      while (u > c && i < ps.length - 1) {
        i++;
        c += ps[i].weight;
      }
      raw.push({ cell: cellAt(lattice, ps[i].state.x, ps[i].state.y), weight: 1 / k });
    }
    return consolidateCells(raw);
  }

  mode(lattice: Lattice): number {
    const p = this.modePose();
    return cellAt(lattice, p.x, p.y);
  }

  /** The dominant cluster's centre — Choset's "average over a small region around the maximum". */
  modePose(): Pose2 {
    return dominantCluster(this.pf.particles, 0.6).pose;
  }

  meanXY(): { x: number; y: number } {
    const m = this.pf.mean();
    return { x: m.x, y: m.y };
  }

  /** Entropy of the cloud binned on a 0.5 m × 0.5 m × 30° grid, in bits. */
  entropyBits(binXY = 0.5, binTheta = Math.PI / 6): number {
    const acc = new Map<string, number>();
    for (const p of this.pf.particles) {
      const key = `${Math.floor(p.state.x / binXY)},${Math.floor(p.state.y / binXY)},${Math.floor((p.state.theta + Math.PI) / binTheta)}`;
      acc.set(key, (acc.get(key) ?? 0) + p.weight);
    }
    return discreteEntropy([...acc.values()]);
  }

  get bytes(): number {
    return this.pf.size * 4 * 8;
  }
}

const now = (): number => (typeof performance !== 'undefined' ? performance.now() : Date.now());

/* -------------------------------------------------------------------------- */
/* One recorded run, replayed into both localizers                            */
/* -------------------------------------------------------------------------- */

export interface LoggedStep {
  /** Ground truth after the move — the simulator knows it, the robot does not. */
  truth: Pose2;
  /** What the odometry reported for the move: Choset's (α, β, d). */
  u: ChosetIncrement;
  scan: number[];
  /** True on the step where the robot was carried away. */
  kidnapped: boolean;
}

export interface RunLog {
  steps: LoggedStep[];
  angles: number[];
  scanParams: ScanParams;
  start: Pose2;
}

export interface RecordOptions {
  seed: number;
  steps: number;
  start?: Pose2;
  waypoints?: readonly { x: number; y: number }[];
  /** Tick at which truth jumps to `kidnapTo` while odometry carries on unaware. */
  kidnapAt?: number;
  kidnapTo?: Pose2;
  /** Multiplicative slip σ on each wheel's distance, and additive heading σ per step. */
  slip?: number;
  dt?: number;
  speed?: number;
  scan?: Partial<ScanParams>;
}

/** The default tour: room A → corridor → east → room C → back. */
export const DEFAULT_TOUR: readonly { x: number; y: number }[] = [
  { x: 2.0, y: 4.4 },
  { x: 5.0, y: 4.4 },
  { x: 7.8, y: 4.4 },
  { x: 7.8, y: 6.8 },
  { x: 7.8, y: 4.4 },
  { x: 10.0, y: 4.4 },
  { x: 10.0, y: 2.0 },
  { x: 10.0, y: 4.4 },
  { x: 3.2, y: 4.4 },
  { x: 3.2, y: 7.0 },
  { x: 3.2, y: 4.4 },
  { x: 2.0, y: 4.4 },
  { x: 2.0, y: 2.0 },
];

export const DEFAULT_SCAN: ScanParams = { nBeams: 36, fov: 2 * Math.PI, maxRange: 8, sigma: 0.04, zRand: 0.02, zMax: 0.01 };

/**
 * Drive Rusty along the tour with a proportional pursuit controller (the Ch. 2
 * "cheat" that closes the loop on truth so the demo keeps driving), slipping
 * wheels, and a noisy 36-beam scan every step. Everything is seeded; both
 * localizers consume the same `u` and `scan` streams, which is what makes the
 * w16.1 comparison fair.
 */
export function recordRun(world: World, opts: RecordOptions): RunLog {
  const motionRng = new Rng(opts.seed);
  const scanRng = motionRng.fork();
  const scanParams: ScanParams = { ...DEFAULT_SCAN, ...opts.scan };
  const angles = beamAngles(scanParams);
  const dt = opts.dt ?? 0.5;
  const speed = opts.speed ?? 0.5;
  const slip = opts.slip ?? 0.04;
  const waypoints = opts.waypoints ?? DEFAULT_TOUR;
  let truth: Pose2 = opts.start ?? { x: 2.0, y: 2.0, theta: Math.PI / 2 };
  const start = truth;
  let wp = 0;
  const steps: LoggedStep[] = [];
  for (let t = 0; t < opts.steps; t++) {
    let kidnapped = false;
    if (opts.kidnapAt !== undefined && t === opts.kidnapAt && opts.kidnapTo) {
      truth = { ...opts.kidnapTo };
      kidnapped = true;
    }
    const target = waypoints[wp % waypoints.length];
    if (Math.hypot(target.x - truth.x, target.y - truth.y) < 0.3) wp++;
    const cmd = pursuePoint(truth, waypoints[wp % waypoints.length], { speed, gain: 2.0, maxOmega: 1.4, turnFirst: 0.6 });
    const ds = cmd.v * dt;
    const dth = cmd.omega * dt;
    // Odometry reports the commanded arc; the floor delivers a slipped one.
    const nominal = applyOdom({ x: 0, y: 0, theta: 0 }, { rot1: dth / 2, trans: ds, rot2: dth / 2 });
    const u = incrementBetween({ x: 0, y: 0, theta: 0 }, nominal);
    const dsReal = ds * (1 + motionRng.normal(0, slip));
    const dthReal = dth + motionRng.normal(0, slip * 0.5) * Math.max(Math.abs(dth), 0.05);
    const next = applyOdom(truth, { rot1: dthReal / 2, trans: dsReal, rot2: dthReal / 2 });
    if (!collides(world, truth, next) && distanceToWalls(world, next.x, next.y) > 0.15) truth = next;
    const scan = simulateScan(world, truth, scanParams, scanRng);
    steps.push({ truth, u, scan, kidnapped });
  }
  return { steps, angles, scanParams, start };
}

/** Dead reckoning from the log: what the robot would believe with no sensor at all. */
export function deadReckon(log: RunLog): Pose2[] {
  const out: Pose2[] = [];
  let x = log.start;
  for (const s of log.steps) {
    x = applyIncrement(x, s.u);
    out.push(x);
  }
  return out;
}

/* -------------------------------------------------------------------------- */
/* §9.2.1 — mapping with known poses, Alg. 19                                   */
/* -------------------------------------------------------------------------- */

/**
 * Alg. 19 line 3, eq. 9.39, for one cell: fold P(m | x(k), y(k)) into the
 * running posterior P_m with prior P(m). This is the odds form; the log-odds
 * form (9.35) the sister's `OccupancyGrid` uses is the same update with
 * multiplication turned into addition. The check file asserts they agree.
 */
export function oddsUpdate(pPrev: number, pInverse: number, pPrior: number): number {
  const a = (1 - pInverse) / pInverse;
  const b = pPrior / (1 - pPrior);
  const c = (1 - pPrev) / pPrev;
  return 1 / (1 + a * b * c);
}

/** The same step in log odds, eq. 9.35: l ← l_inv − l₀ + l_prev. */
export function logOddsUpdate(pPrev: number, pInverse: number, pPrior: number): number {
  return logOddsToProb(probToLogOdds(pInverse) - probToLogOdds(pPrior) + probToLogOdds(pPrev));
}

export interface MappingResult {
  map: OccupancyGrid;
  /** Σ_cells H(p) in bits after each scan — the curve that falls as the map resolves. */
  entropy: number[];
}

/**
 * Mapping with known locations: integrate every (pose, scan) pair of a run
 * into the ported occupancy grid. The inverse sensor model is the sister's
 * Table 9.2 — Choset's cone model of (9.41) is `chosetConeModel` below and is
 * meant for sonar; a laser's beam is thin enough that the α-band model is the
 * honest choice.
 */
export function mapWithKnownPoses(
  world: World,
  poses: readonly Pose2[],
  scans: readonly number[][],
  angles: readonly number[],
  cellSize: number,
  inverse: InverseModelParams = DEFAULT_INVERSE_MODEL,
): MappingResult {
  const map = OccupancyGrid.forWorld(world, cellSize);
  const entropy: number[] = [];
  for (let k = 0; k < poses.length; k++) {
    map.integrateScan(poses[k], scans[k] as number[], angles as number[], inverse);
    entropy.push(map.entropy());
  }
  return { map, entropy };
}

/**
 * How faithfully a map reproduces the walls it was built from: the fraction of
 * *observed* cells within half a cell of a wall that read occupied, and the
 * fraction of observed cells more than a cell away from any wall that read free.
 */
export function wallAgreement(map: OccupancyGrid, world: World): { wallRecall: number; freePrecision: number; observed: number } {
  let wallCells = 0;
  let wallHit = 0;
  let freeCells = 0;
  let freeOk = 0;
  let observed = 0;
  for (let j = 0; j < map.height; j++) {
    for (let i = 0; i < map.width; i++) {
      const l = map.logOdds[map.index(i, j)];
      if (Math.abs(l - map.l0) < 1e-9) continue; // never seen
      observed++;
      const [cx, cy] = map.cellCenter(i, j);
      const d = distanceToWalls(world, cx, cy);
      const p = logOddsToProb(l);
      if (d < map.cellSize * 0.5) {
        wallCells++;
        if (p > 0.5) wallHit++;
      } else if (d > map.cellSize) {
        freeCells++;
        if (p < 0.5) freeOk++;
      }
    }
  }
  return { wallRecall: wallCells ? wallHit / wallCells : 0, freePrecision: freeCells ? freeOk / freeCells : 0, observed };
}

export interface ConeParams {
  /** The three depths d₁ < d₂ < d₃ of the piecewise-linear profile, metres. */
  d1: number;
  d2: number;
  d3: number;
  /** σ_θ of the angular Gaussian in (9.42); Choset: 0.05 rad. */
  sigmaTheta: number;
  /** Slope of the linear g(y) in (9.42); Fig. 9.13 reads 0.01 per metre. */
  gSlope: number;
  prior: number;
}

export const CHOSET_CONE: ConeParams = { d1: 0.1, d2: 0.3, d3: 0.6, sigmaTheta: 0.05, gSlope: 0.01, prior: 0.5 };

/** s(y, θ) = g(y) · N(θ; 0, σ_θ), eq. 9.42. At y = 2 m, θ = 0 this is ≈ 0.16, as Choset states. */
export function coneDeviation(y: number, theta: number, c: ConeParams = CHOSET_CONE): number {
  return c.gSlope * y * normalPdf(theta, 0, c.sigmaTheta);
}

/**
 * Eq. 9.41: the occupancy probability of a cell at distance d and off-axis angle
 * θ from a sonar that measured y. Below the prior in front of the arc, rising
 * through it, held high just behind it, relaxing to the prior beyond y + d₃.
 */
export function chosetConeModel(d: number, theta: number, y: number, c: ConeParams = CHOSET_CONE): number {
  const s = coneDeviation(y, theta, c);
  let dev: number;
  if (d < y - c.d1) dev = -s;
  else if (d < y + c.d1) dev = -s + (s / c.d1) * (d - y + c.d1);
  else if (d < y + c.d2) dev = s;
  else if (d < y + c.d3) dev = s - (s / (c.d3 - c.d2)) * (d - y - c.d2);
  else dev = 0;
  return Math.max(1e-6, Math.min(1 - 1e-6, c.prior + dev));
}

/* -------------------------------------------------------------------------- */
/* §9.2.2 — Bayesian SLAM by Rao-Blackwellization, Alg. 20                     */
/* -------------------------------------------------------------------------- */

export interface SlamDemoResult {
  rbpf: GridRbpf;
  best: GridParticle;
  reports: RbpfReport[];
  /** Final pose error of the best particle, metres. */
  slamError: number;
  /** Final pose error of dead reckoning on the same odometry, metres. */
  odometryError: number;
  /** Mean error along the path, both ways. */
  slamMeanError: number;
  odometryMeanError: number;
}

/**
 * Alg. 20 over the sister's `GridRbpf` (the gmapping recipe): every particle is
 * a path with its own map, weighted by the likelihood of the scan under that
 * map (line 11), resampled when N_eff drops. Fed the same recorded run as the
 * localizers, so the only thing that changed is that the map is no longer given.
 */
export function rbpfSlamDemo(world: World, log: RunLog, m: number, rng: Rng, cellSize = 0.2): SlamDemoResult {
  const makeMap = () => OccupancyGrid.forWorld(world, cellSize);
  const rbpf = GridRbpf.atPose(m, log.start, makeMap, {
    ...DEFAULT_RBPF_OPTIONS,
    angles: log.angles,
    inverse: { ...DEFAULT_INVERSE_MODEL, maxRange: log.scanParams.maxRange },
    score: { zHit: 0.85, zRand: 0.08, maxRange: log.scanParams.maxRange, blur: 0 },
  });
  // The first scan seeds every map before any motion is asked of it.
  for (const p of rbpf.particles) p.map.write().integrateScan(log.start, log.steps[0]?.scan ?? [], log.angles, rbpf.opts.inverse);
  const reports: RbpfReport[] = [];
  const dr = deadReckon(log);
  let slamSum = 0;
  let odomSum = 0;
  for (let k = 0; k < log.steps.length; k++) {
    const s = log.steps[k];
    reports.push(rbpf.step(toOdomDelta(s.u), s.scan, rng));
    const b = rbpf.best().pose;
    slamSum += Math.hypot(b.x - s.truth.x, b.y - s.truth.y);
    odomSum += Math.hypot(dr[k].x - s.truth.x, dr[k].y - s.truth.y);
  }
  const best = rbpf.best();
  const last = log.steps[log.steps.length - 1].truth;
  const drLast = dr[dr.length - 1];
  return {
    rbpf,
    best,
    reports,
    slamError: Math.hypot(best.pose.x - last.x, best.pose.y - last.y),
    odometryError: Math.hypot(drLast.x - last.x, drLast.y - last.y),
    slamMeanError: slamSum / log.steps.length,
    odometryMeanError: odomSum / log.steps.length,
  };
}
