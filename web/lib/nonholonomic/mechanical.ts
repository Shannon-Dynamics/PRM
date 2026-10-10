/**
 * Simple mechanical control systems (Choset §12.4): the kinetic-energy metric,
 * the covariant derivative, the symmetric product, the Lewis–Murray tests at
 * rest (Theorems 12.4.1–12.4.2), and nonholonomic constraints through the
 * projection P (§12.4.3). Everything about M(q), Γ(q) and P comes from
 * Chapter 17's `dynamics`; this file only differentiates vector fields on Q.
 *
 *     ∇_{Y₁} Y₂ = (∂Y₂/∂q) Y₁ + M⁻¹(q) Y₁ᵀ Γ(q) Y₂,          ⟨Y₁ : Y₂⟩ = ∇_{Y₁}Y₂ + ∇_{Y₂}Y₁,
 *
 * where (Y₁ᵀ Γ Y₂)_i = Σ_jk Γ^i_jk Y₁_j Y₂_k with Chapter 17's Christoffel
 * convention (no M⁻¹ folded in — Choset's footnote 2). With Pfaffian
 * constraints A q̇ = 0 the inputs become Ỹ_i = P Y_i and the connection
 * ∇̃_{Ỹ₁}Ỹ₂ = P ∇_{Ỹ₁}Ỹ₂ (eq. 12.22 and the line after it).
 *
 * Why the symmetric product halves the work: on TQ with g₀ = [q̇; −M⁻¹Cq̇] and
 * g_i = [0; Y_i], at rest [g_i, g_j] = 0, [g₀, g_i] = [−Y_i; 0] and
 * [g_i, [g₀, g_j]] = [0; ⟨Y_i : Y_j⟩]. `liftToStateSpace` builds those six-
 * dimensional fields so the checks can confirm the identities numerically.
 */

import { inv, matVec, type Mat, type Vec } from '../prob/linalg';
import type { Dynamics } from '../dynamics/model';
import { projectionP, type Pfaffian } from '../dynamics/constraint';
import { MechanicalSystem } from '../dynamics/model';
import type { Christoffel } from '../dynamics/lagrange';
import { field, independentSubset, jacobianOf, lieBracket, rankOf, relativeResidual, type VectorField } from './field';
import type { ControlAffine } from './filtration';
import { filtration } from './filtration';

/* -------------------------------------------------------------------------- */
/* Connection and symmetric product                                             */
/* -------------------------------------------------------------------------- */

/** (Y₁ᵀ Γ Y₂)_i = Σ_jk Γ^i_jk Y₁_j Y₂_k — the bilinear velocity-product form at one point. */
export function christoffelForm(G: Christoffel, y1: Vec, y2: Vec): Vec {
  const n = G.length;
  const out = new Array<number>(n).fill(0);
  for (let i = 0; i < n; i++) {
    let s = 0;
    for (let j = 0; j < n; j++) for (let k = 0; k < n; k++) s += G[i][j][k] * y1[j] * y2[k];
    out[i] = s;
  }
  return out;
}

export interface Connection {
  /** The configuration dimension n_Q. */
  n: number;
  /** ∇_{Y₁}Y₂ at q. */
  covariant(Y1: VectorField, Y2: VectorField, q: Vec): Vec;
}

/** The Levi-Civita connection of the kinetic-energy metric M(q), using Chapter 17's Christoffel symbols. */
export function kineticConnection(dyn: Dynamics): Connection {
  return {
    n: dyn.n,
    covariant(Y1, Y2, q) {
      const J2 = jacobianOf(Y2, q);
      const y1 = Y1.eval(q);
      const y2 = Y2.eval(q);
      const a = matVec(J2, y1);
      const G = dyn.christoffel(q);
      const b = matVec(inv(dyn.inertia(q)), christoffelForm(G, y1, y2));
      return a.map((v, i) => v + b[i]);
    },
  };
}

/**
 * The constrained connection of §12.4.3: ∇̃_{Ỹ₁}Ỹ₂ = P(q) ∇_{Ỹ₁}Ỹ₂ for fields
 * in the constraint distribution D = {q̇ : A(q) q̇ = 0}.
 */
export function constrainedConnection(dyn: Dynamics, constraint: Pfaffian): Connection {
  const base = kineticConnection(dyn);
  return {
    n: dyn.n,
    covariant(Y1, Y2, q) {
      const P = projectionP(dyn.inertia(q), constraint.A(q));
      return matVec(P, base.covariant(Y1, Y2, q));
    },
  };
}

/** ⟨Y₁ : Y₂⟩ as a field on Q. */
export function symmetricProduct(conn: Connection, Y1: VectorField, Y2: VectorField, name = `<${Y1.name}:${Y2.name}>`): VectorField {
  return field(name, conn.n, (q) => {
    const a = conn.covariant(Y1, Y2, q);
    const b = conn.covariant(Y2, Y1, q);
    return a.map((v, i) => v + b[i]);
  });
}

/** ∇_V V as a field — the quantity Theorem 12.4.7 tests for a decoupling vector field. */
export function selfCovariant(conn: Connection, V: VectorField, name = `∇_${V.name}${V.name}`): VectorField {
  return field(name, conn.n, (q) => conn.covariant(V, V, q));
}

/* -------------------------------------------------------------------------- */
/* Inputs Y_i = columns of M⁻¹T, and their constrained versions Ỹ_i = P Y_i      */
/* -------------------------------------------------------------------------- */

/** Y_i(q) = (M⁻¹(q) T(q))_{·i} for the input matrix T(q) (n_Q × m). */
export function inputFields(dyn: Dynamics, T: (q: Vec) => Mat, m: number, names?: string[]): VectorField[] {
  return Array.from({ length: m }, (_, i) =>
    field(names?.[i] ?? `Y${i + 1}`, dyn.n, (q) => {
      const Minv = inv(dyn.inertia(q));
      const t = T(q).map((row) => row[i]);
      return matVec(Minv, t);
    }),
  );
}

/** Ỹ_i = P(q) Y_i(q) with P = I − M⁻¹Aᵀ(AM⁻¹Aᵀ)⁻¹A (Choset eq. 12.21–12.22). */
export function constrainedInputs(dyn: Dynamics, constraint: Pfaffian, inputs: readonly VectorField[]): VectorField[] {
  return inputs.map((Y) =>
    field(`P${Y.name}`, dyn.n, (q) => {
      const P = projectionP(dyn.inertia(q), constraint.A(q));
      return matVec(P, Y.eval(q));
    }),
  );
}

/* -------------------------------------------------------------------------- */
/* Symmetric closure and the Lewis–Murray verdicts                              */
/* -------------------------------------------------------------------------- */

export interface SymProduct {
  field: VectorField;
  degree: number;
  counts: number[];
  value: Vec;
  /** Bad iff every Y_i appears an even number of times. */
  bad: boolean;
}

export interface SymClosure {
  /** dim Sym_1, dim Sym_2, … at q. */
  dims: number[];
  products: SymProduct[];
  rank: number;
}

const badSym = (counts: readonly number[]) => counts.every((c) => c % 2 === 0);

/**
 * Sym(Y) at q by the same recursion as the filtration, with ⟨·:·⟩ in place of
 * [·,·]: S₁ = Y, S_{i+1} = S_i ∪ {⟨Y_j : h⟩ : Y_j ∈ S₁, h new in S_i}. At degree 2
 * the product is symmetric so only j ≤ k is formed — j = k included, because
 * ⟨Y : Y⟩ is exactly the bad product that must be neutralized.
 */
export function symClosure(conn: Connection, inputs: readonly VectorField[], q: Vec, maxDeg = 3, tol = 1e-7): SymClosure {
  const m = inputs.length;
  const n = conn.n;
  const products: SymProduct[] = inputs.map((Y, i) => {
    const counts = new Array<number>(m).fill(0);
    counts[i] = 1;
    return { field: Y, degree: 1, counts, value: Y.eval(q), bad: false };
  });
  let scale = Math.max(1e-300, ...products.map((p) => Math.hypot(...p.value)));
  const dims = [rankOf(products.map((p) => p.value), tol)];
  let frontier = products.slice();
  for (let deg = 2; deg <= maxDeg; deg++) {
    const fresh: SymProduct[] = [];
    for (let j = 0; j < m; j++) {
      for (const h of frontier) {
        if (deg === 2) {
          const k = h.counts.findIndex((c) => c === 1);
          if (j > k) continue;
        }
        const g = symmetricProduct(conn, inputs[j], h.field);
        const value = g.eval(q);
        const counts = h.counts.slice();
        counts[j]++;
        // Drop products that vanish identically near q (probe four nearby points).
        let zero = Math.hypot(...value) <= 1e-9 * scale;
        if (zero) {
          for (let p = 0; p < 4 && zero; p++) {
            const qp = q.map((v, i) => v + 0.31 * Math.sin(1.3 * p + 2.1 * i + 0.7));
            if (Math.hypot(...g.eval(qp)) > 1e-9 * scale) zero = false;
          }
        }
        if (zero) continue;
        fresh.push({ field: g, degree: deg, counts, value, bad: badSym(counts) });
        scale = Math.max(scale, Math.hypot(...value));
      }
    }
    products.push(...fresh);
    dims.push(rankOf(products.map((p) => p.value), tol));
    frontier = fresh;
    if (dims[dims.length - 1] === n || fresh.length === 0) break;
  }
  return { dims, products, rank: dims[dims.length - 1] };
}

export type MechVerdictKind = 'stlc' | 'stla' | 'stlec' | 'stlca' | 'undecided' | 'confined';

export interface MechVerdict {
  kind: MechVerdictKind;
  detail: string;
  /** dim Sym(Y)(q): the reachable velocity directions from rest. */
  symDim: number;
  /** dim Lie(Sym(Y))(q): the reachable configuration directions from rest. */
  lieSymDim: number;
  /** Is Sym(Y) = span(Y) — the maximal-reducibility test of Theorem 12.4.5? */
  maximallyReducible: boolean;
  /** Each bad symmetric product with its neutralization residual against lower-degree good ones. */
  bad: { label: string; degree: number; residual: number }[];
  neutralized: boolean;
  sym: SymClosure;
  /** The ladder of Lie(Sym) dims, for the knife-edge's cos²q₃ → 2/(m²I⁴) story. */
  lieDims: number[];
}

/**
 * Theorems 12.4.1–12.4.2 from an equilibrium (q, 0). `target` is the dimension
 * the velocities can fill: n_Q unconstrained, n_Q − k under k Pfaffian
 * constraints (then "Sym = D" is the best case and the verdicts are STLCA/STLEC).
 */
export function lewisMurray(conn: Connection, inputs: readonly VectorField[], q: Vec, opts: { maxDeg?: number; lieDeg?: number; tol?: number; target?: number } = {}): MechVerdict {
  const maxDeg = opts.maxDeg ?? 3;
  const lieDeg = opts.lieDeg ?? 3;
  const tol = opts.tol ?? 1e-7;
  const n = conn.n;
  const target = opts.target ?? n;
  const sym = symClosure(conn, inputs, q, maxDeg, tol);
  const spanDim = rankOf(inputs.map((Y) => Y.eval(q)), tol);
  const maximallyReducible = sym.rank === spanDim;

  const bad: MechVerdict['bad'] = [];
  let neutralized = true;
  for (const p of sym.products) {
    if (!p.bad) continue;
    const lower = sym.products.filter((s) => !s.bad && s.degree < p.degree).map((s) => s.value);
    const residual = relativeResidual(p.value, lower);
    bad.push({ label: p.field.name, degree: p.degree, residual });
    if (residual > 1e-6) neutralized = false;
  }

  // Configuration reachability: Lie(Sym(Y)) — bracket a basis of the symmetric closure.
  const basisIdx = independentSubset(sym.products.map((p) => p.value), tol);
  const symFields = basisIdx.map((i) => sym.products[i].field);
  const lie = filtration(symFields, q, { maxDeg: lieDeg, tol });
  const lieSymDim = lie.rank;

  if (target === n && sym.rank >= target) {
    if (neutralized) {
      return { kind: 'stlc', detail: 'Sym(Y)(q) = T_qQ and every bad symmetric product is neutralized: STLC from rest (Thm. 12.4.1)', symDim: sym.rank, lieSymDim, maximallyReducible, bad, neutralized, sym, lieDims: lie.dims };
    }
    return { kind: 'stla', detail: 'Sym(Y)(q) = T_qQ but a bad symmetric product is not neutralized: STLA from rest; STLC undecided (Thm. 12.4.1)', symDim: sym.rank, lieSymDim, maximallyReducible, bad, neutralized, sym, lieDims: lie.dims };
  }
  // Under k Pfaffian constraints velocities live in D (dim n − k): STLA/STLC are
  // out of reach and the question is configuration reachability (§12.4.3).
  if (lieSymDim === n) {
    if (neutralized) {
      return { kind: 'stlec', detail: `Sym(Y)(q) has dimension ${sym.rank} < ${target} (not STLA) but Lie(Sym(Y))(q) = T_qQ and the bad products are neutralized: STLCC and STLEC (Thm. 12.4.2)`, symDim: sym.rank, lieSymDim, maximallyReducible, bad, neutralized, sym, lieDims: lie.dims };
    }
    return { kind: 'stlca', detail: `Lie(Sym(Y))(q) = T_qQ but a bad product is not neutralized: STLCA only (Thm. 12.4.2)`, symDim: sym.rank, lieSymDim, maximallyReducible, bad, neutralized, sym, lieDims: lie.dims };
  }
  if (lie.terminated) {
    return { kind: 'confined', detail: `Lie(Sym(Y))(q) closed at dimension ${lieSymDim} < ${n}: the reachable configurations form a leaf`, symDim: sym.rank, lieSymDim, maximallyReducible, bad, neutralized, sym, lieDims: lie.dims };
  }
  return { kind: 'undecided', detail: `dim Sym = ${sym.rank}, dim Lie(Sym) = ${lieSymDim} < ${n} within the degrees computed`, symDim: sym.rank, lieSymDim, maximallyReducible, bad, neutralized, sym, lieDims: lie.dims };
}

/* -------------------------------------------------------------------------- */
/* Lifting to the state space TQ, for the cross-check against Lie brackets        */
/* -------------------------------------------------------------------------- */

/**
 * The control-affine form (12.11) on M = TQ: x = (q, q̇), g₀ = [q̇; −M⁻¹(C q̇ + g)],
 * g_i = [0; Y_i(q)]. Jacobians by the module's finite differences — these fields
 * are only used to confirm the three bracket identities and the det = d⁴ example.
 */
export function liftToStateSpace(dyn: Dynamics, inputs: readonly VectorField[], name = 'mechanical'): ControlAffine {
  const nq = dyn.n;
  const n = 2 * nq;
  const drift = field('g0', n, (x) => {
    const q = x.slice(0, nq);
    const qd = x.slice(nq);
    const c = dyn.velocityProduct(q, qd);
    const g = dyn.gravity(q);
    const acc = matVec(inv(dyn.inertia(q)), c.map((v, i) => -(v + g[i])));
    return [...qd, ...acc];
  });
  const controls = inputs.map((Y, i) =>
    field(`g${i + 1}`, n, (x) => {
      const y = Y.eval(x.slice(0, nq));
      return [...new Array<number>(nq).fill(0), ...y];
    }),
  );
  return { name, n, drift, controls, controlSet: 'U±' };
}

/* -------------------------------------------------------------------------- */
/* The chapter's mechanical systems                                              */
/* -------------------------------------------------------------------------- */

/** The planar body with thrusters as a Chapter 17 `Dynamics`: M = diag(m, m, I), Γ = 0, optional gravity along −q₂. */
export class PbwtDynamics extends MechanicalSystem {
  readonly n = 3;
  constructor(
    readonly m = 1,
    readonly I = 1,
    readonly aG = 0,
  ) {
    super();
  }
  inertia(): Mat {
    return [
      [this.m, 0, 0],
      [0, this.m, 0],
      [0, 0, this.I],
    ];
  }
  christoffel(): Christoffel {
    return [0, 1, 2].map(() => [0, 1, 2].map(() => [0, 0, 0]));
  }
  velocityProduct(): Vec {
    return [0, 0, 0];
  }
  gravity(): Vec {
    return [0, this.m * this.aG, 0];
  }
  potentialEnergy(q: Vec): number {
    return this.m * this.aG * q[1];
  }
}

/** Y₁ = [cos q₃, sin q₃, 0]ᵀ/m, Y₂ = [−sin q₃, cos q₃, −d·m/I]ᵀ/m — the columns of M⁻¹T (Example 12.4.3). */
export function pbwtInputs(d = 1, m = 1, I = 1): VectorField[] {
  const Y1 = field('Y1', 3, (q) => [Math.cos(q[2]) / m, Math.sin(q[2]) / m, 0], (q) => [
    [0, 0, -Math.sin(q[2]) / m],
    [0, 0, Math.cos(q[2]) / m],
    [0, 0, 0],
  ]);
  const Y2 = field('Y2', 3, (q) => [-Math.sin(q[2]) / m, Math.cos(q[2]) / m, -d / I], (q) => [
    [0, 0, -Math.cos(q[2]) / m],
    [0, 0, -Math.sin(q[2]) / m],
    [0, 0, 0],
  ]);
  return [Y1, Y2];
}

/** Choset's hand result ⟨Y₁ : Y₂⟩ = [d sin q₃, −d cos q₃, 0]ᵀ for the unit PBWT. */
export const pbwtSymExact = (q: Vec, d = 1): Vec => [d * Math.sin(q[2]), -d * Math.cos(q[2]), 0];

/**
 * The single-leg hopper of Example 12.4.4: q = (body angle, leg angle, leg
 * extension), M = diag(I, m q₃², m), two actuators (leg torque relative to the
 * body, leg force). Its angular momentum is conserved, so Sym is only
 * two-dimensional: STLEC and equilibrium controllable, never STLA.
 */
export class HopperDynamics extends MechanicalSystem {
  readonly n = 3;
  constructor(
    readonly I = 1,
    readonly m = 1,
  ) {
    super();
  }
  inertia(q: Vec): Mat {
    return [
      [this.I, 0, 0],
      [0, this.m * q[2] * q[2], 0],
      [0, 0, this.m],
    ];
  }
  /** Γ²₂₃ = Γ²₃₂ = m q₃, Γ³₂₂ = −m q₃ (Choset, Example 12.4.4). */
  christoffel(q: Vec): Christoffel {
    const G: Christoffel = [0, 1, 2].map(() => [0, 1, 2].map(() => [0, 0, 0]));
    G[1][1][2] = this.m * q[2];
    G[1][2][1] = this.m * q[2];
    G[2][1][1] = -this.m * q[2];
    return G;
  }
  gravity(): Vec {
    return [0, 0, 0];
  }
  potentialEnergy(): number {
    return 0;
  }
}

/** Y₁ = [1/I, −1/(m q₃²), 0]ᵀ, Y₂ = [0, 0, 1/m]ᵀ. */
export function hopperInputs(I = 1, m = 1): VectorField[] {
  return [
    field('Y1', 3, (q) => [1 / I, -1 / (m * q[2] * q[2]), 0], (q) => [
      [0, 0, 0],
      [0, 0, 2 / (m * q[2] ** 3)],
      [0, 0, 0],
    ]),
    field('Y2', 3, () => [0, 0, 1 / m], () => [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ]),
  ];
}

/** Knife-edge inputs of Example 12.4.9: a force along q₁ and a torque about q₃ — Y₁ = e₁/m, Y₂ = e₃/I. */
export function knifeEdgeInputs(m: number, I: number): VectorField[] {
  const z = (): Mat => [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  return [field('Y1', 3, () => [1 / m, 0, 0], z), field('Y2', 3, () => [0, 0, 1 / I], z)];
}

/** Choset's closed forms for the knife-edge: Ỹ₁, ⟨Ỹ₁:Ỹ₂⟩, [Ỹ₁,Ỹ₂], [Ỹ₂,[Ỹ₁,Ỹ₂]] and the two determinants. */
export function knifeEdgeExact(q: Vec, m: number, I: number) {
  const c = Math.cos(q[2]);
  const s = Math.sin(q[2]);
  return {
    Y1t: [(c * c) / m, (s * c) / m, 0],
    Y2t: [0, 0, 1 / I],
    sym12: [(-s * c) / (m * I), (-s * s) / (m * I), 0],
    br12: [Math.sin(2 * q[2]) / (m * I), -Math.cos(2 * q[2]) / (m * I), 0],
    br2_12: [(2 * Math.cos(2 * q[2])) / (m * I * I), (2 * Math.sin(2 * q[2])) / (m * I * I), 0],
    det2: (c * c) / (m * m * I * I),
    det3: 2 / (m * m * I ** 4),
  };
}

/**
 * Reach's 3R with the third motor dead (Choset §12.5.7, Figure 12.32): a
 * Chapter 17 `PlanarArm` with inputs T = [e₁ e₂]. Away from q₂ ∈ {0, π} the
 * actuated joints can push the third joint in any direction, so the free
 * third link is Choset's planar body with thrusters, torque-less.
 */
export function deadJointInputs(dyn: Dynamics): VectorField[] {
  return inputFields(dyn, () => [
    [1, 0],
    [0, 1],
    [0, 0],
  ], 2);
}
