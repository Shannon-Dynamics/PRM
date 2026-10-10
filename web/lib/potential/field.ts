/**
 * Potential functions — Choset Ch. 4 intro and §4.1, as code.
 *
 * A potential is a differentiable scalar field U : R^m → R whose gradient the
 * robot reads as a *velocity* (first-order robots, Choset's convention for
 * §4.1–4.6) or as a *force* (§4.7, where it is lifted through Jᵀ). Everything
 * here is generic in the dimension so the same `Attractive` serves a point in
 * the plane and a control point on Reach; configurations are plain number
 * arrays, which is what `nalgebra::SVector<f64, N>` becomes in the browser.
 *
 * The additive construction U = U_att + Σ_i U_rep_i is kept exactly as Choset
 * writes it, including its honest failure: the sum of a valley and some hills
 * can have a second valley (§4.4). `hessian` and `classifyCritical` exist so a
 * widget can *report* that failure — eigenvalue signs at the stuck point —
 * rather than hide it.
 */

/** A configuration, or a tangent/force vector, as a plain array of length `dim`. */
export type Vec = number[];

export interface Potential {
  readonly dim: number;
  value(q: Vec): number;
  /** ∇U(q) = [∂U/∂q₁, …, ∂U/∂q_m]ᵀ — points uphill (Choset Ch. 4 intro). */
  gradient(q: Vec): Vec;
}

/** d_i(q) and ∇d_i(q) for one obstacle (Choset eqs. 4.6–4.7), plus the closest point c. */
export interface ObstacleDistance {
  /** d_i(q) = min_{c ∈ QO_i} d(q, c); zero (or negative, penetration depth) inside. */
  d: number;
  /** ∇d_i(q) = (q − c)/‖q − c‖, a unit vector pointing away from the obstacle. */
  grad: Vec;
  /** The minimizing point c, when the query can name it. */
  closest?: Vec;
  /** Which obstacle this is (index into the query's obstacle list). */
  index: number;
}

/**
 * Per-obstacle distance, the only thing a repulsive potential needs. Chapter 4's
 * `Collision` queries, Chapter 3's range scan (§4.3.1) and Chapter 7's own
 * brushfire grid (§4.3.2) all implement it — so one `Repulsive` serves all three.
 */
export interface DistanceQuery {
  readonly dim: number;
  /** d_i(q), ∇d_i(q) for every obstacle i (the caller filters by Q*_i). */
  distances(q: Vec): ObstacleDistance[];
}

/* -------------------------------------------------------------------------- */
/* Small vector helpers (kept local: no linear-algebra crate in the browser)   */
/* -------------------------------------------------------------------------- */

export const vsub = (a: Vec, b: Vec): Vec => a.map((v, i) => v - b[i]);
export const vadd = (a: Vec, b: Vec): Vec => a.map((v, i) => v + b[i]);
export const vscale = (a: Vec, k: number): Vec => a.map((v) => v * k);
export const vdot = (a: Vec, b: Vec): number => a.reduce((s, v, i) => s + v * b[i], 0);
export const vnorm = (a: Vec): number => Math.sqrt(vdot(a, a));
export const vzero = (n: number): Vec => new Array(n).fill(0);
export const vdist = (a: Vec, b: Vec): number => vnorm(vsub(a, b));

/* -------------------------------------------------------------------------- */
/* U_att — Choset eqs. (4.1)–(4.3)                                              */
/* -------------------------------------------------------------------------- */

/**
 * The blended attractive potential: quadratic within `dGoalStar` of the goal,
 * conic beyond it. The quadratic half gives a gradient that vanishes linearly
 * at the goal (no chattering); the conic half caps the gradient's magnitude at
 * ζ·d* so a far-away start does not demand an absurd velocity. The two agree
 * in value and gradient at d = d* (Derivation 1), which is why the blend is
 * C¹ and gradient descent never sees the seam.
 */
export class Attractive implements Potential {
  readonly dim: number;

  constructor(
    readonly goal: Vec,
    readonly zeta = 1,
    /** d*_goal; `Infinity` gives the pure quadratic of eq. (4.1). */
    readonly dGoalStar = Infinity,
  ) {
    this.dim = goal.length;
  }

  value(q: Vec): number {
    const d = vdist(q, this.goal);
    const { zeta, dGoalStar: ds } = this;
    return d <= ds ? 0.5 * zeta * d * d : ds * zeta * d - 0.5 * zeta * ds * ds;
  }

  gradient(q: Vec): Vec {
    const diff = vsub(q, this.goal);
    const d = vnorm(diff);
    const { zeta, dGoalStar: ds } = this;
    if (d <= ds) return vscale(diff, zeta); // eq. (4.1): ζ(q − q_goal)
    return vscale(diff, (ds * zeta) / d); // eq. (4.3), conic branch: magnitude ζ d*
  }
}

/* -------------------------------------------------------------------------- */
/* U_rep — Choset eqs. (4.4)–(4.5), summed per obstacle (§4.1, p. 84)          */
/* -------------------------------------------------------------------------- */

/**
 * Repulsion summed over obstacles, each with its own domain of influence Q*_i.
 * Choset first writes U_rep in terms of D(q), the distance to the *closest*
 * obstacle, then redefines it per obstacle so the gradient does not jump when
 * the nearest obstacle changes (the oscillation he warns about). This class is
 * the per-obstacle version; a `DistanceQuery` that returns only the nearest
 * obstacle (the sensor and grid versions) recovers the D(q) form.
 *
 * Inside an obstacle (d ≤ 0) the formula is undefined; we return a large finite
 * value and a gradient along ∇d so a widget can still draw something sensible.
 */
export class Repulsive implements Potential {
  readonly dim: number;

  constructor(
    readonly dist: DistanceQuery,
    readonly eta = 1,
    /** Q*: one radius for all obstacles, or one per obstacle index. */
    readonly qStar: number | readonly number[] = 1,
  ) {
    this.dim = dist.dim;
  }

  private qs(i: number): number {
    return typeof this.qStar === 'number' ? this.qStar : (this.qStar[i] ?? this.qStar[this.qStar.length - 1]);
  }

  value(q: Vec): number {
    let u = 0;
    for (const o of this.dist.distances(q)) {
      const Q = this.qs(o.index);
      if (o.d > Q) continue;
      if (o.d <= 1e-9) {
        u += 0.5 * this.eta * (1e9 - 1 / Q) ** 2 * 1e-9; // finite stand-in for +∞
        continue;
      }
      u += 0.5 * this.eta * (1 / o.d - 1 / Q) ** 2; // eq. (4.4)
    }
    return u;
  }

  gradient(q: Vec): Vec {
    const g = vzero(this.dim);
    for (const o of this.dist.distances(q)) {
      const Q = this.qs(o.index);
      if (o.d > Q) continue;
      const d = Math.max(o.d, 1e-6);
      // eq. (4.5): η (1/Q* − 1/d) (1/d²) ∇d — negative along ∇d, i.e. the *descent* pushes away.
      const k = this.eta * (1 / Q - 1 / d) * (1 / (d * d));
      for (let i = 0; i < this.dim; i++) g[i] += k * o.grad[i];
    }
    return g;
  }
}

/** U = Σ terms — "the whole method and its local minima" (design doc §5). */
export class Additive implements Potential {
  readonly dim: number;

  constructor(readonly terms: Potential[]) {
    if (terms.length === 0) throw new Error('Additive: at least one term');
    this.dim = terms[0].dim;
  }

  value(q: Vec): number {
    let u = 0;
    for (const t of this.terms) u += t.value(q);
    return u;
  }

  gradient(q: Vec): Vec {
    const g = vzero(this.dim);
    for (const t of this.terms) {
      const tg = t.gradient(q);
      for (let i = 0; i < this.dim; i++) g[i] += tg[i];
    }
    return g;
  }
}

/** A potential from closures — handy for the navigation function compositions and tests. */
export class FnPotential implements Potential {
  constructor(
    readonly dim: number,
    private readonly f: (q: Vec) => number,
    private readonly g?: (q: Vec) => Vec,
    private readonly h = 1e-6,
  ) {}

  value(q: Vec): number {
    return this.f(q);
  }

  gradient(q: Vec): Vec {
    if (this.g) return this.g(q);
    return finiteDifferenceGradient(this.f, q, this.h);
  }
}

/* -------------------------------------------------------------------------- */
/* Critical points: Hessian and its signature                                  */
/* -------------------------------------------------------------------------- */

/** Central differences of a scalar field. */
export function finiteDifferenceGradient(f: (q: Vec) => number, q: Vec, h = 1e-6): Vec {
  const g = vzero(q.length);
  for (let i = 0; i < q.length; i++) {
    const qp = q.slice();
    const qm = q.slice();
    qp[i] += h;
    qm[i] -= h;
    g[i] = (f(qp) - f(qm)) / (2 * h);
  }
  return g;
}

/**
 * The Hessian ∇²U by central differences of the gradient, symmetrized. Choset
 * reads the type of a critical point off its signature: positive definite is a
 * minimum, negative definite a maximum, indefinite a saddle; nonsingular means
 * *nondegenerate*, hence isolated (the Morse condition Chapter 8 names).
 */
export function hessian(U: Potential, q: Vec, h = 1e-4): number[][] {
  const n = q.length;
  const H: number[][] = Array.from({ length: n }, () => vzero(n));
  for (let j = 0; j < n; j++) {
    const qp = q.slice();
    const qm = q.slice();
    qp[j] += h;
    qm[j] -= h;
    const gp = U.gradient(qp);
    const gm = U.gradient(qm);
    for (let i = 0; i < n; i++) H[i][j] = (gp[i] - gm[i]) / (2 * h);
  }
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const s = 0.5 * (H[i][j] + H[j][i]);
      H[i][j] = s;
      H[j][i] = s;
    }
  }
  return H;
}

/** Eigenvalues of a symmetric 2 × 2 matrix, ascending. */
export function eigen2(H: number[][]): [number, number] {
  const a = H[0][0];
  const b = H[0][1];
  const d = H[1][1];
  const tr = a + d;
  const det = a * d - b * b;
  const disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
  return [tr / 2 - disc, tr / 2 + disc];
}

/** Eigenvalues of a symmetric matrix by Jacobi rotations (small n only). */
export function eigenSym(H: number[][]): number[] {
  const n = H.length;
  if (n === 1) return [H[0][0]];
  if (n === 2) return eigen2(H);
  const A = H.map((r) => r.slice());
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += A[i][j] * A[i][j];
    if (off < 1e-22) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        if (Math.abs(A[p][q]) < 1e-300) continue;
        const theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = A[k][p];
          const akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = A[p][k];
          const aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
      }
    }
  }
  return A.map((r, i) => r[i]).sort((x, y) => x - y);
}

export type CriticalType = 'minimum' | 'maximum' | 'saddle' | 'degenerate';

/** Signature → type. `tol` is the eigenvalue magnitude below which we call it degenerate. */
export function classifyCritical(eigs: readonly number[], tol = 1e-9): CriticalType {
  if (eigs.some((e) => Math.abs(e) <= tol)) return 'degenerate';
  if (eigs.every((e) => e > 0)) return 'minimum';
  if (eigs.every((e) => e < 0)) return 'maximum';
  return 'saddle';
}

/** Number of negative eigenvalues — the Morse index Chapter 8's definition box names. */
export function morseIndex(eigs: readonly number[], tol = 1e-9): number {
  return eigs.filter((e) => e < -tol).length;
}
