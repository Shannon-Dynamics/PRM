/**
 * `optim` — the shared nonlinear least-squares optimizer (Chapter 19 §5,
 * `trajopt::optim`), imported by Chapter 21 for nonlinear steering.
 *
 * Gauss–Newton with Levenberg–Marquardt damping and a trust-region flavour:
 *
 *     minimize  ½ ‖r(x)‖²,     (Jᵀ J + λ D) δ = −Jᵀ r,     x ← x + δ  if the true cost drops,
 *
 * where D = diag(JᵀJ) (Marquardt's scaling, so a badly scaled coordinate is
 * damped in its own units) and λ is raised when a step is rejected and
 * lowered when it is accepted. At λ → 0 the step is Gauss–Newton; at λ → ∞ it
 * is a short gradient step. The optional box `trust` clips ‖δ‖∞ — that is the
 * crude trust region the sequential-convex recipe in `sco.ts` leans on, and
 * the text says so.
 *
 * Jacobians default to central differences; a problem that knows its own
 * supplies `jacobian`. Everything is dense `prob/linalg`: the problems this
 * book hands the optimizer have at most a few hundred unknowns.
 */

import { matVec, solve, transpose, type Mat, type Vec } from '../prob/linalg';

export interface LeastSquares {
  /** r(x) ∈ ℝ^m. */
  residual(x: Vec): Vec;
  /** ∂r/∂x ∈ ℝ^{m×n}. Optional: central differences otherwise. */
  jacobian?(x: Vec): Mat;
}

export interface LmOptions {
  maxIter?: number;
  /** Stop when ‖δ‖∞ < tolStep or the relative cost decrease < tolCost. */
  tolStep?: number;
  tolCost?: number;
  /** Stop when ‖Jᵀr‖∞ < tolGrad (first-order optimality). */
  tolGrad?: number;
  lambda0?: number;
  lambdaUp?: number;
  lambdaDown?: number;
  /** Box trust radius on the step, ‖δ‖∞ ≤ trust; Infinity disables it. */
  trust?: number;
  /** Finite-difference step for a problem without an analytic Jacobian. */
  fdStep?: number;
}

export interface LmResult {
  x: Vec;
  /** ½‖r‖² at x. */
  cost: number;
  iterations: number;
  /** ½‖r‖² after each accepted step (history[0] is the initial cost). */
  history: number[];
  converged: boolean;
  /** Final damping. */
  lambda: number;
  /** ‖Jᵀr‖∞ at the solution. */
  gradNorm: number;
}

export const halfSq = (r: Vec): number => 0.5 * r.reduce((s, v) => s + v * v, 0);

/** Central-difference Jacobian of a vector function, one column per unknown. */
export function finiteDifferenceJacobian(f: (x: Vec) => Vec, x: Vec, h = 1e-6): Mat {
  const n = x.length;
  const cols: Vec[] = [];
  for (let j = 0; j < n; j++) {
    const xp = x.slice();
    const xm = x.slice();
    xp[j] += h;
    xm[j] -= h;
    const fp = f(xp);
    const fm = f(xm);
    cols.push(fp.map((v, i) => (v - fm[i]) / (2 * h)));
  }
  const m = cols[0]?.length ?? f(x).length;
  const J: Mat = [];
  for (let i = 0; i < m; i++) J.push(cols.map((c) => c[i]));
  return J;
}

/** The normal-equation pieces JᵀJ and Jᵀr, formed once per iteration. */
function normalEquations(J: Mat, r: Vec): { JtJ: Mat; Jtr: Vec } {
  const Jt = transpose(J);
  const n = Jt.length;
  const JtJ: Mat = [];
  for (let i = 0; i < n; i++) {
    const row = new Array<number>(n).fill(0);
    for (let j = 0; j < n; j++) {
      let s = 0;
      for (let k = 0; k < r.length; k++) s += Jt[i][k] * Jt[j][k];
      row[j] = s;
    }
    JtJ.push(row);
  }
  return { JtJ, Jtr: matVec(Jt, r) };
}

/**
 * Levenberg–Marquardt. Returns the best point found; `converged` is true when
 * a tolerance was met rather than the iteration budget.
 */
export function levenbergMarquardt(prob: LeastSquares, x0: Vec, opts: LmOptions = {}): LmResult {
  const maxIter = opts.maxIter ?? 100;
  const tolStep = opts.tolStep ?? 1e-10;
  const tolCost = opts.tolCost ?? 1e-14;
  const tolGrad = opts.tolGrad ?? 1e-10;
  const up = opts.lambdaUp ?? 10;
  const down = opts.lambdaDown ?? 0.3;
  const trust = opts.trust ?? Infinity;
  let lambda = opts.lambda0 ?? 1e-3;
  const jac = (x: Vec): Mat => (prob.jacobian ? prob.jacobian(x) : finiteDifferenceJacobian((y) => prob.residual(y), x, opts.fdStep ?? 1e-6));

  let x = x0.slice();
  let r = prob.residual(x);
  let cost = halfSq(r);
  const history = [cost];
  let converged = false;
  let gradNorm = Infinity;
  let it = 0;
  for (; it < maxIter; it++) {
    const J = jac(x);
    const { JtJ, Jtr } = normalEquations(J, r);
    gradNorm = Math.max(...Jtr.map(Math.abs));
    if (gradNorm < tolGrad) {
      converged = true;
      break;
    }
    // Inner loop: raise λ until a step is accepted.
    let accepted = false;
    for (let tries = 0; tries < 40; tries++) {
      const A = JtJ.map((row, i) => row.map((v, j) => (i === j ? v + lambda * Math.max(v, 1e-12) : v)));
      let delta: Vec;
      try {
        delta = solve(A, Jtr.map((v) => -v));
      } catch {
        lambda *= up;
        continue;
      }
      const stepInf = Math.max(...delta.map(Math.abs));
      if (stepInf > trust) {
        const k = trust / stepInf;
        delta = delta.map((v) => v * k);
      }
      const xTrial = x.map((v, i) => v + delta[i]);
      const rTrial = prob.residual(xTrial);
      const cTrial = halfSq(rTrial);
      if (cTrial < cost) {
        const rel = (cost - cTrial) / Math.max(cost, 1e-300);
        x = xTrial;
        r = rTrial;
        cost = cTrial;
        history.push(cost);
        lambda = Math.max(lambda * down, 1e-15);
        accepted = true;
        if (stepInf < tolStep || rel < tolCost) converged = true;
        break;
      }
      lambda *= up;
    }
    if (!accepted) {
      // λ grew without a descent step: we are at a stationary point to working precision.
      converged = true;
      break;
    }
    if (converged) {
      it++;
      break;
    }
  }
  return { x, cost, iterations: it, history, converged, lambda, gradNorm };
}

/**
 * Armijo backtracking on a scalar objective along a descent direction:
 * the largest α ∈ {1, β, β², …} with f(x + αd) ≤ f(x) + c α ∇fᵀd.
 * The same rule the iLQR forward pass applies to its expected decrease.
 */
export function backtrackingLineSearch(f: (x: Vec) => number, x: Vec, dir: Vec, grad: Vec, opts: { c?: number; beta?: number; maxTries?: number } = {}): { alpha: number; x: Vec; f: number } {
  const c = opts.c ?? 1e-4;
  const beta = opts.beta ?? 0.5;
  const maxTries = opts.maxTries ?? 30;
  const f0 = f(x);
  const slope = grad.reduce((s, g, i) => s + g * dir[i], 0);
  let alpha = 1;
  for (let k = 0; k < maxTries; k++) {
    const xt = x.map((v, i) => v + alpha * dir[i]);
    const ft = f(xt);
    if (ft <= f0 + c * alpha * slope) return { alpha, x: xt, f: ft };
    alpha *= beta;
  }
  return { alpha: 0, x, f: f0 };
}

/** The Rosenbrock residuals (10(y − x²), 1 − x): the standard LM test problem, minimum (1, 1). */
export const ROSENBROCK: LeastSquares = {
  residual: (x) => [10 * (x[1] - x[0] * x[0]), 1 - x[0]],
  jacobian: (x) => [
    [-20 * x[0], 10],
    [-1, 0],
  ],
};
