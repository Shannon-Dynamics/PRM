/**
 * KL divergence on a coarse partition — the ruler this chapter measures a
 * sampler with (Derivation 2).
 *
 * Coarsen 𝒬 into N cells. A proposal's empirical cell frequencies p̂ against
 * the uniform u = 1/N give D_KL(p̂ ‖ u) = ln N − H(p̂): how *concentrated* the
 * sampler is. Against the target p* (where roadmap-path nodes actually fall)
 * D_KL(p* ‖ p̂) says how well it is *aimed*, and minimizing that over the
 * weights is exactly maximum likelihood on path-node samples. Both numbers are
 * in nats; the bits are ln 2 away and printed in parentheses in the prose.
 */

export const LN2 = Math.LN2;

/** Σ pᵢ ln(pᵢ / qᵢ) in nats, with the 0·ln 0 = 0 convention; +∞ where q vanishes under p. */
export function klDivergence(p: ArrayLike<number>, q: ArrayLike<number>): number {
  if (p.length !== q.length) throw new Error('klDivergence: length mismatch');
  let s = 0;
  for (let i = 0; i < p.length; i++) {
    if (p[i] <= 0) continue;
    if (q[i] <= 0) return Infinity;
    s += p[i] * Math.log(p[i] / q[i]);
  }
  return s;
}

/** Shannon entropy −Σ pᵢ ln pᵢ in nats. */
export function entropy(p: ArrayLike<number>): number {
  let h = 0;
  for (let i = 0; i < p.length; i++) if (p[i] > 0) h -= p[i] * Math.log(p[i]);
  return h;
}

export const natsToBits = (x: number) => x / LN2;

/** Counts on N cells, with the two divergences the chapter prints. */
export class CoarseHistogram {
  readonly counts: number[];

  constructor(counts: readonly number[]) {
    if (counts.length === 0) throw new Error('CoarseHistogram: need at least one cell');
    this.counts = [...counts];
  }

  get n(): number {
    return this.counts.length;
  }

  get total(): number {
    return this.counts.reduce((a, b) => a + b, 0);
  }

  /** p̂ᵢ = countᵢ / Σ counts. */
  probabilities(): number[] {
    const t = this.total;
    return t > 0 ? this.counts.map((c) => c / t) : this.counts.map(() => 1 / this.n);
  }

  uniform(): number[] {
    return this.counts.map(() => 1 / this.n);
  }

  /** D_KL(p̂ ‖ u) — concentration relative to Chapter 11's default proposal. */
  klToUniform(): number {
    return klDivergence(this.probabilities(), this.uniform());
  }

  /** D_KL(target ‖ p̂) — how well the proposal is aimed at where paths actually go. */
  klFrom(target: readonly number[]): number {
    return klDivergence(target, this.probabilities());
  }

  entropy(): number {
    return entropy(this.probabilities());
  }

  /** The bounded-below mixture p_λ = (1 − λ) p̂ + λ u, cell by cell. */
  mixture(lambda: number): number[] {
    const p = this.probabilities();
    return p.map((pi) => (1 - lambda) * pi + lambda / this.n);
  }

  /** Smallest cell mass under the mixture — D22.2 asks this to be ≥ λ / N. */
  mixtureFloor(lambda: number): { min: number; floor: number; holds: boolean } {
    const m = this.mixture(lambda);
    const min = Math.min(...m);
    const floor = lambda / this.n;
    return { min, floor, holds: min >= floor - 1e-12 };
  }

  /** Bin a list of cell indices. */
  static fromCells(cells: ArrayLike<number>, n: number): CoarseHistogram {
    const counts = new Array<number>(n).fill(0);
    for (let i = 0; i < cells.length; i++) counts[cells[i]]++;
    return new CoarseHistogram(counts);
  }
}

/* -------------------------------------------------------------------------- */
/* The §3(a) micro-example                                                      */
/* -------------------------------------------------------------------------- */

/** Four quadrants of Reach's torus; 200 draws of the raw learned sampler at λ = 0. */
export const FOUR_CELL_COUNTS: readonly number[] = [100, 60, 30, 10];
/** Where the same world's roadmap-path nodes fall, p*. */
export const FOUR_CELL_TARGET: readonly number[] = [0.55, 0.3, 0.1, 0.05];

export interface FourCellReport {
  pHat: number[];
  klHatUniformNats: number;
  klHatUniformBits: number;
  klTargetHat: number;
  klTargetUniform: number;
  /** ln 4 − H(p*) — the entropy identity for the uniform case. */
  entropyIdentity: number;
  aimRatio: number;
  mixture: number[];
  mixtureMin: number;
  mixtureFloor: number;
}

export function fourCellExample(lambda = 0.1): FourCellReport {
  const h = new CoarseHistogram(FOUR_CELL_COUNTS);
  const klHatUniform = h.klToUniform();
  const klTargetHat = h.klFrom(FOUR_CELL_TARGET);
  const klTargetUniform = klDivergence(FOUR_CELL_TARGET, h.uniform());
  const mix = h.mixtureFloor(lambda);
  return {
    pHat: h.probabilities(),
    klHatUniformNats: klHatUniform,
    klHatUniformBits: natsToBits(klHatUniform),
    klTargetHat,
    klTargetUniform,
    entropyIdentity: Math.log(4) - entropy(FOUR_CELL_TARGET),
    aimRatio: klTargetUniform / klTargetHat,
    mixture: h.mixture(lambda),
    mixtureMin: mix.min,
    mixtureFloor: mix.floor,
  };
}

/**
 * Expected number of draws to land in a region of mass p(A): the geometric
 * waiting time 1 / p(A). Under the mixture the wait is capped at 1 / (λ u(A)).
 */
export function expectedHits(massUnderProposal: number, lambda: number, uniformMass: number): { proposal: number; mixture: number; cap: number } {
  const mixed = (1 - lambda) * massUnderProposal + lambda * uniformMass;
  return {
    proposal: massUnderProposal > 0 ? 1 / massUnderProposal : Infinity,
    mixture: mixed > 0 ? 1 / mixed : Infinity,
    cap: lambda * uniformMass > 0 ? 1 / (lambda * uniformMass) : Infinity,
  };
}
