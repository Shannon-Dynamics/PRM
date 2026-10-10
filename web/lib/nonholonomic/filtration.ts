/**
 * Filtrations, the involutive closure, and the controllability tests of
 * Choset §12.1.3 and §12.3: the LARC / Chow's theorem (Thm. 12.3.1) and
 * Sussmann's good-and-bad-bracket condition (Thm. 12.3.3).
 *
 * Choset's recursion is followed literally:
 *
 *     G₁ = G,   G_{i+1} = G_i ∪ { [g_j, g_k] : g_j ∈ G₁, g_k ∈ G_i },   D_i = span(G_i),
 *
 * with two harmless prunings — [g, g] = 0 and [g_j, g_k] = −[g_k, g_j] at
 * degree 2, and a Lie product that evaluates to zero at the test point *and*
 * at a few nearby probes is dropped as identically zero. What is never pruned
 * is a product that merely happens to lie in the span at the test point: a
 * field can be dependent at x and have independent brackets (Example 12.1.7
 * on a coordinate plane is exactly that), so pointwise dependence must not
 * cut the recursion. The price is exponential growth in the degree; m ≤ 3
 * generators to degree 5 is a few hundred products, fine for a widget.
 *
 * Every product carries the multiset of generators it was built from, so the
 * good/bad tagging of Sussmann's theorem is a counting exercise: bad iff g₀
 * appears an odd number of times and every control field an even number.
 */

import type { Vec } from '../prob/linalg';
import { flow, independentSubset, lieBracket, rankOf, relativeResidual, type VectorField } from './field';

/** A Lie product with its provenance. */
export interface LieProduct {
  field: VectorField;
  /** Number of original fields appearing (Choset's degree). */
  degree: number;
  /** counts[i] = how many times generator i appears; index 0 is the drift when there is one. */
  counts: number[];
  /** Value at the test point. */
  value: Vec;
  /** Sussmann's tag; only meaningful for systems with drift (see `tagBad`). */
  bad: boolean;
}

export interface FiltrationStep {
  degree: number;
  /** All products of this degree that survived the zero-pruning. */
  products: LieProduct[];
  /** dim D_i at the test point. */
  dim: number;
}

export interface Filtration {
  n: number;
  /** dim D₁, dim D₂, … — Choset's ladder; for the car (2, 3, 4). */
  dims: number[];
  steps: FiltrationStep[];
  /** Every product of every degree, in generation order. */
  products: LieProduct[];
  /** Indices (into `products`) of a greedy basis of the final distribution at x. */
  basis: number[];
  /** True when the closure is certain: full rank reached, or no nonzero product left to bracket. */
  terminated: boolean;
  /** How many trailing degrees added no dimension (a regular filtration that stalls has closed). */
  stalled: number;
  /** dim D_∞ as far as computed. */
  rank: number;
}

export interface FiltrationOptions {
  /** Highest Lie-product degree to form. */
  maxDeg?: number;
  /** Relative rank tolerance. */
  tol?: number;
  /** Index of the drift field in `fields`, or −1 for a driftless system (affects only the bad tags). */
  driftIndex?: number;
  /** Probe radius for the identically-zero pruning. */
  probe?: number;
  /** Hard cap on the number of products, as a safety valve. */
  maxProducts?: number;
}

/** Sussmann's rule: bad iff the drift appears an odd number of times and each control field an even number. */
export function tagBad(counts: readonly number[], driftIndex: number): boolean {
  if (driftIndex < 0) return false;
  if (counts[driftIndex] % 2 === 0) return false;
  for (let i = 0; i < counts.length; i++) if (i !== driftIndex && counts[i] % 2 === 1) return false;
  return true;
}

/** Is a product identically (not just pointwise) negligible? Checked at x and at 4 fixed probes around it. */
function negligibleEverywhere(g: VectorField, x: Vec, probe: number, scale: number): boolean {
  const n = x.length;
  const probes: Vec[] = [x];
  for (let k = 0; k < 4; k++) probes.push(x.map((v, i) => v + probe * Math.sin(1.7 * k + 2.3 * i + 0.5)));
  for (const p of probes) {
    const v = g.eval(p);
    if (Math.hypot(...v) > 1e-9 * scale) return false;
  }
  return true;
}

/**
 * The filtration D₁ ⊆ D₂ ⊆ … of span(fields) evaluated at x. `dims[k-1]` is
 * dim D_k. The last step is also reported as terminated when it added no new
 * dimension — for a regular filtration that is the involutive closure.
 */
export function filtration(fields: readonly VectorField[], x: Vec, opts: FiltrationOptions = {}): Filtration {
  const maxDeg = opts.maxDeg ?? 4;
  const tol = opts.tol ?? 1e-7;
  const driftIndex = opts.driftIndex ?? -1;
  const probe = opts.probe ?? 0.37;
  const maxProducts = opts.maxProducts ?? 600;
  const n = x.length;
  const m = fields.length;

  const products: LieProduct[] = fields.map((g, i) => {
    const counts = new Array<number>(m).fill(0);
    counts[i] = 1;
    return { field: g, degree: 1, counts, value: g.eval(x), bad: tagBad(counts, driftIndex) };
  });
  let scale = 0;
  for (const p of products) scale = Math.max(scale, Math.hypot(...p.value));
  if (scale === 0) scale = 1;

  const steps: FiltrationStep[] = [{ degree: 1, products: products.slice(), dim: rankOf(products.map((p) => p.value), tol) }];
  const dims = [steps[0].dim];
  let frontier = products.slice();
  let terminated = false;

  for (let deg = 2; deg <= maxDeg; deg++) {
    const fresh: LieProduct[] = [];
    for (let j = 0; j < m; j++) {
      for (const h of frontier) {
        // Degree 2: [g_j, g_k] with j < k only (antisymmetry; [g, g] = 0).
        if (deg === 2) {
          const k = h.counts.findIndex((c) => c === 1);
          if (j >= k) continue;
        }
        const g = lieBracket(fields[j], h.field);
        if (negligibleEverywhere(g, x, probe, scale)) continue;
        const counts = h.counts.slice();
        counts[j]++;
        fresh.push({ field: g, degree: deg, counts, value: g.eval(x), bad: tagBad(counts, driftIndex) });
        if (products.length + fresh.length >= maxProducts) break;
      }
      if (products.length + fresh.length >= maxProducts) break;
    }
    products.push(...fresh);
    const dim = rankOf(products.map((p) => p.value), tol);
    steps.push({ degree: deg, products: fresh, dim });
    dims.push(dim);
    frontier = fresh;
    // Full rank, or no nonzero product left to bracket (a nilpotent algebra):
    // the closure is in hand. A *stall* is not a stop — a non-regular
    // filtration may pause and grow again (the knife-edge of §12.4.3 at
    // q₃ = ±π/2 needs degree 3 after degree 2 adds nothing new), so the
    // recursion runs to maxDeg and `stalled` reports how long it has paused.
    if (dim === n || fresh.length === 0) {
      terminated = true;
      break;
    }
  }
  let stalled = 0;
  for (let i = dims.length - 1; i > 0 && dims[i] === dims[i - 1]; i--) stalled++;
  const basis = independentSubset(products.map((p) => p.value), tol);
  return { n, dims, steps, products, basis, terminated, stalled, rank: dims[dims.length - 1] };
}

/* -------------------------------------------------------------------------- */
/* Control-affine systems and the verdicts                                      */
/* -------------------------------------------------------------------------- */

/** Choset's two classes of control sets (§12.2): U± positively spans ℝᵐ; U⁺ merely spans it. */
export type ControlSetClass = 'U±' | 'U+';

export interface ControlAffine {
  name: string;
  n: number;
  /** g₀; null for a driftless system. */
  drift: VectorField | null;
  /** g₁ … g_m. */
  controls: VectorField[];
  controlSet: ControlSetClass;
}

export type Verdict =
  | { kind: 'stlc-symmetric'; degree: number; detail: string }
  | { kind: 'stlc-sussmann'; degree: number; detail: string; neutralized: string[] }
  | { kind: 'stla'; degree: number; detail: string }
  | { kind: 'confined'; dim: number; detail: string }
  | { kind: 'undecided'; detail: string };

/** The generators Lie(·) is taken over: g₀ first when present, then the controls. */
export function generators(sys: ControlAffine): { fields: VectorField[]; driftIndex: number } {
  return sys.drift ? { fields: [sys.drift, ...sys.controls], driftIndex: 0 } : { fields: sys.controls.slice(), driftIndex: -1 };
}

/**
 * Chow / LARC (Theorem 12.3.1). STLA at x iff Lie({g₀,…,g_m})(x) = T_xM (the
 * "only if" needs analytic fields — true for every system in this book); a
 * symmetric system (driftless, U±) is then STLC. A filtration that closes
 * below n confines the system to a leaf of that dimension.
 */
export function larc(sys: ControlAffine, x: Vec, maxDeg = 4, tol = 1e-7): { verdict: Verdict; filtration: Filtration } {
  const { fields, driftIndex } = generators(sys);
  const f = filtration(fields, x, { maxDeg, tol, driftIndex });
  const degree = f.dims.findIndex((d) => d === sys.n) + 1;
  if (f.rank === sys.n) {
    if (!sys.drift && sys.controlSet === 'U±') {
      return { verdict: { kind: 'stlc-symmetric', degree, detail: `LARC holds at degree ${degree} and the system is symmetric: STLC (and STLA) at x` }, filtration: f };
    }
    return { verdict: { kind: 'stla', degree, detail: `LARC holds at degree ${degree}: STLA at x${sys.drift ? ' (drift: STLC needs Sussmann)' : ' (U⁺ only: symmetry unavailable)'}` }, filtration: f };
  }
  if (f.terminated) {
    return { verdict: { kind: 'confined', dim: f.rank, detail: `the filtration closed at dimension ${f.rank} < ${sys.n}: the system stays on a ${f.rank}-dimensional leaf (Frobenius)` }, filtration: f };
  }
  if (f.stalled >= 2) {
    return { verdict: { kind: 'confined', dim: f.rank, detail: `rank ${f.rank} < ${sys.n}, unchanged over the last ${f.stalled} degrees: for a regular filtration this is the involutive closure, a ${f.rank}-dimensional leaf (if the distribution is not regular here, raise maxDeg)` }, filtration: f };
  }
  return { verdict: { kind: 'undecided', detail: `rank ${f.rank} < ${sys.n} after degree ${maxDeg}; the filtration has not closed — raise maxDeg` }, filtration: f };
}

/**
 * Sussmann's sufficient condition (Theorem 12.3.3) at an equilibrium:
 * g₀(x) = 0, U ∈ U±, good brackets up to degree k span T_xM, and every bad
 * bracket of degree j ≤ k is a combination of good brackets of degree < j.
 * Returns the verdict and the per-bad-bracket neutralization residuals.
 */
export function sussmannStlc(sys: ControlAffine, x: Vec, maxDeg = 4, tol = 1e-7): { verdict: Verdict; filtration: Filtration; bad: { label: string; degree: number; residual: number }[] } {
  const { fields, driftIndex } = generators(sys);
  const f = filtration(fields, x, { maxDeg, tol, driftIndex });
  const bad: { label: string; degree: number; residual: number }[] = [];
  if (!sys.drift) {
    const r = larc(sys, x, maxDeg, tol);
    return { verdict: r.verdict, filtration: f, bad };
  }
  const g0 = sys.drift.eval(x);
  const scale = Math.max(1, ...f.products.map((p) => Math.hypot(...p.value)));
  if (Math.hypot(...g0) > 1e-8 * scale) {
    return { verdict: { kind: 'undecided', detail: 'g₀(x) ≠ 0: Sussmann\'s theorem applies only at equilibria (the degree-1 bad bracket cannot be neutralized)' }, filtration: f, bad };
  }
  if (sys.controlSet !== 'U±') {
    return { verdict: { kind: 'undecided', detail: 'U ∉ U±: the theorem needs a positively spanning control set' }, filtration: f, bad };
  }
  // Smallest degree k at which the GOOD brackets alone span T_xM.
  let k = -1;
  for (let deg = 1; deg <= f.dims.length; deg++) {
    const good = f.products.filter((p) => !p.bad && p.degree <= deg).map((p) => p.value);
    if (rankOf(good, tol) === sys.n) {
      k = deg;
      break;
    }
  }
  // Residuals of every bad bracket against the good ones of lower degree —
  // reported whatever the verdict, so a reader can see *which* bracket spoils it.
  const residualOf = (p: LieProduct) => relativeResidual(p.value, f.products.filter((q) => !q.bad && q.degree < p.degree).map((q) => q.value));
  if (k < 0) {
    for (const p of f.products) if (p.bad) bad.push({ label: p.field.name, degree: p.degree, residual: residualOf(p) });
    const all = f.rank === sys.n;
    return {
      verdict: all
        ? { kind: 'stla', degree: f.dims.findIndex((d) => d === sys.n) + 1, detail: 'LARC holds but good brackets alone do not span: STLA; Sussmann is inconclusive about STLC' }
        : { kind: 'undecided', detail: `rank ${f.rank} < ${sys.n} within degree ${maxDeg}` },
      filtration: f,
      bad,
    };
  }
  const neutralized: string[] = [];
  let ok = true;
  for (const p of f.products) {
    if (!p.bad || p.degree > k) continue;
    const res = residualOf(p);
    bad.push({ label: p.field.name, degree: p.degree, residual: res });
    if (res > 1e-6) ok = false;
    else neutralized.push(p.field.name);
  }
  if (ok) {
    return { verdict: { kind: 'stlc-sussmann', degree: k, detail: `good brackets span T_xM at degree ${k}; every bad bracket of degree ≤ ${k} is neutralized by lower-degree good ones: STLC at the equilibrium x`, neutralized }, filtration: f, bad };
  }
  return { verdict: { kind: 'stla', degree: k, detail: `LARC holds at degree ${k} but a bad bracket is not neutralized (${bad.filter((b) => b.residual > 1e-6).map((b) => b.label).join(', ')}): STLA; STLC undecided by this test`, }, filtration: f, bad };
}

/* -------------------------------------------------------------------------- */
/* Reachable sets by sampling (w20.2)                                           */
/* -------------------------------------------------------------------------- */

export interface ReachableOptions {
  /** Horizon T. */
  T: number;
  /** Radius of the neighborhood V (Euclidean in the chart); trajectories leaving it are discarded. */
  radius: number;
  /** Number of random control histories. */
  samples: number;
  /** Piecewise-constant segments per history. */
  segments: number;
  /** Integration substeps per segment. */
  substeps?: number;
  /** Which control fields are available (indices into `controls`); default all. */
  active?: number[];
  /** Keep every intermediate state (R^V(x, ≤ T)) rather than endpoints only. */
  keepAll?: boolean;
}

/**
 * R^V(x₀, ≤ T) grown by Monte Carlo: piecewise-constant controls drawn from
 * the control set ([−1, 1]ᵐ for U±, [0, 1]ᵐ for U⁺), integrated with RK4, kept
 * while the trajectory stays in the ball V. Honest about what it is — a sample
 * of the reachable set, not the set — which is what the widget needs to show
 * a curve, a half-space, or a blob.
 */
export function sampleReachable(sys: ControlAffine, x0: Vec, opts: ReachableOptions, draw: () => number): Vec[] {
  const active = opts.active ?? sys.controls.map((_, i) => i);
  const substeps = opts.substeps ?? 4;
  const out: Vec[] = [];
  const n = sys.n;
  const dt = opts.T / opts.segments;
  const rhs = (x: Vec, u: number[]): Vec => {
    const v = sys.drift ? sys.drift.eval(x) : new Array<number>(n).fill(0);
    active.forEach((ci, k) => {
      if (u[k] === 0) return;
      const g = sys.controls[ci].eval(x);
      for (let i = 0; i < n; i++) v[i] += u[k] * g[i];
    });
    return v;
  };
  for (let s = 0; s < opts.samples; s++) {
    let x = x0.slice();
    let alive = true;
    for (let seg = 0; seg < opts.segments && alive; seg++) {
      const u = active.map(() => (sys.controlSet === 'U±' ? 2 * draw() - 1 : draw()));
      const h = dt / substeps;
      for (let k = 0; k < substeps; k++) {
        const k1 = rhs(x, u);
        const k2 = rhs(x.map((v, i) => v + 0.5 * h * k1[i]), u);
        const k3 = rhs(x.map((v, i) => v + 0.5 * h * k2[i]), u);
        const k4 = rhs(x.map((v, i) => v + h * k3[i]), u);
        x = x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
        let r = 0;
        for (let i = 0; i < n; i++) r += (x[i] - x0[i]) ** 2;
        if (Math.sqrt(r) > opts.radius) {
          alive = false;
          break;
        }
        if (opts.keepAll) out.push(x.slice());
      }
    }
    if (alive && !opts.keepAll) out.push(x);
  }
  return out;
}

/**
 * The flow of a fixed control u for time t — a "macro" the widget can add to
 * the control set: the four-flow loop as one input, say.
 */
export function flowWithControl(sys: ControlAffine, x: Vec, u: readonly number[], t: number, steps = 8): Vec {
  const n = sys.n;
  const g = {
    name: 'u',
    n,
    eval: (y: Vec) => {
      const v = sys.drift ? sys.drift.eval(y) : new Array<number>(n).fill(0);
      sys.controls.forEach((gi, i) => {
        if (u[i] === 0) return;
        const w = gi.eval(y);
        for (let k = 0; k < n; k++) v[k] += u[i] * w[k];
      });
      return v;
    },
  };
  return flow(g, x, t, steps);
}
