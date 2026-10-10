/**
 * Vector fields on a chart of an n-dimensional state manifold, their flows,
 * and the Lie bracket — Choset §12.1.1 and §12.1.3 (eqs. 12.3–12.5).
 *
 * A field is a function x ↦ g(x) ∈ T_xM written in local coordinates, so it is
 * a `Vec → Vec`. Brackets need Jacobians; a field may carry an analytic one
 * (`jacobian`) and every concrete system in `systems.ts` does, because a
 * nested bracket differentiates a bracket, and a finite difference of a finite
 * difference loses half its digits per level. The fallback is a fourth-order
 * central stencil with a *large* step (h = 10⁻³): truncation O(h⁴) ≈ 10⁻¹²
 * against round-off ε/h ≈ 10⁻¹³, so even two nested fallbacks keep ~10⁻⁷.
 *
 * Sign convention is Choset's (12.5): [g₁, g₂] = (∂g₂/∂x) g₁ − (∂g₁/∂x) g₂, so
 * that the four-flow loop g₁ → g₂ → −g₁ → −g₂ moves by +ε²[g₁, g₂]. Problem 8
 * notes that for linear fields this is (BA − AB)x, the *negative* of the usual
 * matrix commutator — the checks pin that sign.
 */

import type { Mat, Vec } from '../prob/linalg';

export interface VectorField {
  /** Human-readable, e.g. "g1" or "[g1,g2]" — the labels the filtration prints. */
  readonly name: string;
  readonly n: number;
  eval(x: Vec): Vec;
  /** ∂g/∂x at x, n×n, row i = ∂g_i/∂x. Optional: fourth-order central differences otherwise. */
  jacobian?(x: Vec): Mat;
}

/** Fourth-order central-difference Jacobian of a vector function. */
export function jacobianFD(f: (x: Vec) => Vec, x: Vec, h = 1e-3): Mat {
  const n = x.length;
  const f0 = f(x);
  const m = f0.length;
  const J: Mat = Array.from({ length: m }, () => new Array<number>(n).fill(0));
  for (let k = 0; k < n; k++) {
    const xp = x.slice();
    const xpp = x.slice();
    const xm = x.slice();
    const xmm = x.slice();
    xp[k] += h;
    xpp[k] += 2 * h;
    xm[k] -= h;
    xmm[k] -= 2 * h;
    const fp = f(xp);
    const fpp = f(xpp);
    const fm = f(xm);
    const fmm = f(xmm);
    for (let i = 0; i < m; i++) J[i][k] = (fmm[i] - 8 * fm[i] + 8 * fp[i] - fpp[i]) / (12 * h);
  }
  return J;
}

/** The field's Jacobian: analytic if it has one, finite differences otherwise. */
export function jacobianOf(g: VectorField, x: Vec): Mat {
  return g.jacobian ? g.jacobian(x) : jacobianFD((y) => g.eval(y), x);
}

/** Build a field from a closure, optionally with its analytic Jacobian. */
export function field(name: string, n: number, f: (x: Vec) => Vec, jac?: (x: Vec) => Mat): VectorField {
  return jac ? { name, n, eval: f, jacobian: jac } : { name, n, eval: f };
}

/** The constant coordinate field ∂/∂x_k (Choset writes g₂ᵘⁿⁱ = ∂/∂x₃). */
export function coordinateField(n: number, k: number, name = `e${k + 1}`): VectorField {
  const zero = (): Mat => Array.from({ length: n }, () => new Array<number>(n).fill(0));
  return field(
    name,
    n,
    () => {
      const v = new Array<number>(n).fill(0);
      v[k] = 1;
      return v;
    },
    zero,
  );
}

export function scaleField(g: VectorField, s: number, name = `${s}·${g.name}`): VectorField {
  return field(
    name,
    g.n,
    (x) => g.eval(x).map((v) => v * s),
    g.jacobian ? (x) => (g.jacobian as (y: Vec) => Mat)(x).map((r) => r.map((v) => v * s)) : undefined,
  );
}

export function addFields(a: VectorField, b: VectorField, name = `${a.name}+${b.name}`): VectorField {
  const jac =
    a.jacobian && b.jacobian
      ? (x: Vec) => {
          const Ja = (a.jacobian as (y: Vec) => Mat)(x);
          const Jb = (b.jacobian as (y: Vec) => Mat)(x);
          return Ja.map((r, i) => r.map((v, j) => v + Jb[i][j]));
        }
      : undefined;
  return field(
    name,
    a.n,
    (x) => {
      const u = a.eval(x);
      const v = b.eval(x);
      return u.map((p, i) => p + v[i]);
    },
    jac,
  );
}

/** Σ u_i g_i(x) — the right-hand side of a driftless control system at fixed u. */
export function combination(fields: readonly VectorField[], u: readonly number[], name = 'Σ u_i g_i'): VectorField {
  const n = fields[0].n;
  return field(
    name,
    n,
    (x) => {
      const out = new Array<number>(n).fill(0);
      fields.forEach((g, i) => {
        if (u[i] === 0) return;
        const v = g.eval(x);
        for (let k = 0; k < n; k++) out[k] += u[i] * v[k];
      });
      return out;
    },
    fields.every((g) => g.jacobian)
      ? (x) => {
          const J: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
          fields.forEach((g, i) => {
            if (u[i] === 0) return;
            const Jg = (g.jacobian as (y: Vec) => Mat)(x);
            for (let r = 0; r < n; r++) for (let c = 0; c < n; c++) J[r][c] += u[i] * Jg[r][c];
          });
          return J;
        }
      : undefined,
  );
}

/* -------------------------------------------------------------------------- */
/* The Lie bracket                                                             */
/* -------------------------------------------------------------------------- */

/** [g₁, g₂](x) = (∂g₂/∂x) g₁ − (∂g₁/∂x) g₂ evaluated at one point (12.5). */
export function bracketAt(g1: VectorField, g2: VectorField, x: Vec): Vec {
  const J1 = jacobianOf(g1, x);
  const J2 = jacobianOf(g2, x);
  const v1 = g1.eval(x);
  const v2 = g2.eval(x);
  const n = x.length;
  const out = new Array<number>(n).fill(0);
  for (let i = 0; i < n; i++) {
    let s = 0;
    for (let k = 0; k < n; k++) s += J2[i][k] * v1[k] - J1[i][k] * v2[k];
    out[i] = s;
  }
  return out;
}

/**
 * The bracket as a new field, so brackets nest: `lieBracket(g1, lieBracket(g1, g2))`
 * is Choset's g₄ for the car. Its Jacobian is left to finite differences of an
 * exactly evaluated bracket — the compromise the module header explains.
 */
export function lieBracket(g1: VectorField, g2: VectorField, name = `[${g1.name},${g2.name}]`): VectorField {
  return field(name, g1.n, (x) => bracketAt(g1, g2, x));
}

/* -------------------------------------------------------------------------- */
/* Flows                                                                       */
/* -------------------------------------------------------------------------- */

/** φᵍ_t(x): the flow of g for time t, by fixed-step RK4 (t may be negative: follow −g). */
export function flow(g: VectorField, x0: Vec, t: number, steps = 32): Vec {
  const h = t / steps;
  let x = x0.slice();
  const n = x.length;
  for (let s = 0; s < steps; s++) {
    const k1 = g.eval(x);
    const k2 = g.eval(x.map((v, i) => v + 0.5 * h * k1[i]));
    const k3 = g.eval(x.map((v, i) => v + 0.5 * h * k2[i]));
    const k4 = g.eval(x.map((v, i) => v + h * k3[i]));
    const next = new Array<number>(n);
    for (let i = 0; i < n; i++) next[i] = x[i] + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    x = next;
  }
  return x;
}

/** The flow sampled along the way, for drawing a leg of the four-flow loop. */
export function flowTrace(g: VectorField, x0: Vec, t: number, steps = 32): Vec[] {
  const out: Vec[] = [x0.slice()];
  const h = t / steps;
  let x = x0.slice();
  for (let s = 0; s < steps; s++) {
    x = flow(g, x, h, 1);
    out.push(x);
  }
  return out;
}

export interface FourFlowLoop {
  /** x₀, after g₁, after g₂, after −g₁, after −g₂. */
  corners: Vec[];
  /** The dense trace of all four legs, for drawing. */
  trace: Vec[];
  /** x(4ε) − x₀. */
  residue: Vec;
  /** ε²[g₁, g₂](x₀) — the prediction of (12.4). */
  predicted: Vec;
  /** ‖residue − predicted‖, the O(ε³) remainder. */
  remainder: number;
}

/**
 * Choset's eq. (12.3): follow g₁ for ε, g₂ for ε, −g₁ for ε, −g₂ for ε. The
 * net motion is ε²[g₁, g₂] + O(ε³) with no O(ε) term — the content of
 * Derivation 1 and the thing w20.1 measures.
 */
export function fourFlowLoop(g1: VectorField, g2: VectorField, x0: Vec, eps: number, steps = 24): FourFlowLoop {
  const legs: [VectorField, number][] = [
    [g1, eps],
    [g2, eps],
    [g1, -eps],
    [g2, -eps],
  ];
  const corners: Vec[] = [x0.slice()];
  const trace: Vec[] = [x0.slice()];
  let x = x0.slice();
  for (const [g, t] of legs) {
    const tr = flowTrace(g, x, t, steps);
    trace.push(...tr.slice(1));
    x = tr[tr.length - 1];
    corners.push(x);
  }
  const residue = x.map((v, i) => v - x0[i]);
  const br = bracketAt(g1, g2, x0);
  const predicted = br.map((v) => v * eps * eps);
  let rem = 0;
  for (let i = 0; i < residue.length; i++) rem += (residue[i] - predicted[i]) ** 2;
  return { corners, trace, residue, predicted, remainder: Math.sqrt(rem) };
}

/* -------------------------------------------------------------------------- */
/* Small linear algebra the rank tests need                                    */
/* -------------------------------------------------------------------------- */

/**
 * Numerical rank of a set of column vectors by modified Gram–Schmidt with a
 * tolerance *relative to the largest column*: a bracket of size 10⁻³ next to
 * fields of size 1 is a real direction; one of size 10⁻¹² is round-off.
 */
export function rankOf(columns: readonly Vec[], relTol = 1e-7): number {
  const basis: Vec[] = [];
  let scale = 0;
  for (const c of columns) scale = Math.max(scale, Math.hypot(...c));
  if (scale === 0) return 0;
  for (const c of columns) {
    const v = c.slice();
    for (let pass = 0; pass < 2; pass++) {
      for (const b of basis) {
        let d = 0;
        for (let i = 0; i < v.length; i++) d += v[i] * b[i];
        for (let i = 0; i < v.length; i++) v[i] -= d * b[i];
      }
    }
    const nv = Math.hypot(...v);
    if (nv > relTol * scale) basis.push(v.map((t) => t / nv));
  }
  return basis.length;
}

/** Greedy independent subset: indices of the columns that raised the rank, in order. */
export function independentSubset(columns: readonly Vec[], relTol = 1e-7): number[] {
  const basis: Vec[] = [];
  const picked: number[] = [];
  let scale = 0;
  for (const c of columns) scale = Math.max(scale, Math.hypot(...c));
  if (scale === 0) return picked;
  columns.forEach((c, idx) => {
    const v = c.slice();
    for (let pass = 0; pass < 2; pass++) {
      for (const b of basis) {
        let d = 0;
        for (let i = 0; i < v.length; i++) d += v[i] * b[i];
        for (let i = 0; i < v.length; i++) v[i] -= d * b[i];
      }
    }
    const nv = Math.hypot(...v);
    if (nv > relTol * scale) {
      basis.push(v.map((t) => t / nv));
      picked.push(idx);
    }
  });
  return picked;
}

/**
 * Least-squares residual of expressing `target` in span(columns), relative to
 * ‖target‖ — the neutralization test of Sussmann's and Lewis–Murray's theorems:
 * a bad bracket is neutralized when this is ≈ 0.
 */
export function relativeResidual(target: Vec, columns: readonly Vec[]): number {
  const nt = Math.hypot(...target);
  if (nt === 0) return 0;
  if (columns.length === 0) return 1;
  // Orthonormalize the columns, then project.
  const basis: Vec[] = [];
  let scale = 0;
  for (const c of columns) scale = Math.max(scale, Math.hypot(...c));
  for (const c of columns) {
    const v = c.slice();
    for (let pass = 0; pass < 2; pass++) {
      for (const b of basis) {
        let d = 0;
        for (let i = 0; i < v.length; i++) d += v[i] * b[i];
        for (let i = 0; i < v.length; i++) v[i] -= d * b[i];
      }
    }
    const nv = Math.hypot(...v);
    if (nv > 1e-10 * scale) basis.push(v.map((t) => t / nv));
  }
  const r = target.slice();
  for (const b of basis) {
    let d = 0;
    for (let i = 0; i < r.length; i++) d += r[i] * b[i];
    for (let i = 0; i < r.length; i++) r[i] -= d * b[i];
  }
  return Math.hypot(...r) / nt;
}

/** Determinant of a square matrix given as columns, by Gaussian elimination. */
export function detColumns(columns: readonly Vec[]): number {
  const n = columns.length;
  const A: Mat = Array.from({ length: n }, (_, i) => columns.map((c) => c[i]));
  let det = 1;
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[piv][c])) piv = r;
    if (Math.abs(A[piv][c]) < 1e-300) return 0;
    if (piv !== c) {
      [A[c], A[piv]] = [A[piv], A[c]];
      det = -det;
    }
    det *= A[c][c];
    for (let r = c + 1; r < n; r++) {
      const f = A[r][c] / A[c][c];
      for (let k = c; k < n; k++) A[r][k] -= f * A[c][k];
    }
  }
  return det;
}

export const fmtVec = (v: Vec, digits = 6): string => `[${v.map((t) => (Math.abs(t) < 0.5 * 10 ** -digits ? 0 : t).toFixed(digits)).join(', ')}]`;
