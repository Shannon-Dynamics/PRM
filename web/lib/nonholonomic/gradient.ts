/**
 * Gradient steering for driftless systems (Choset §12.5.4) and its Newton-type
 * cousin through Chapter 19's Levenberg–Marquardt (§12.5.3, a cross-link: the
 * nonlinear program is Chapter 19's, only the end-state map is new).
 *
 * The control history is a finite vector uᵖ ∈ ℝʳ — here piecewise-constant
 * (v, ω) on K equal intervals of [0, 1] for the reduced car, so the end-state
 * map f(x_start, uᵖ) is a composition of exact SE(2) arcs. The error
 * e = f(x_start, uᵖ) − x_goal is pushed down along vᵀ = −eᵀ ∂f/∂uᵖ (Choset's
 * update, with a backtracking line search for α). Convergence for small α
 * needs ∂f/∂uᵖ of rank n; at a *singular control* — uᵖ = 0 is the textbook one
 * for a car, where every column of the Jacobian lies in span{g₁, g₂} and the
 * sideways direction is missing — Choset appends a *generic loop*: a random
 * control with zero net motion. For a driftless system a control followed by
 * its own time-reversed negative retraces itself, so the loop is free to build
 * and is generically nonsingular.
 */

import { boxplus, type Pose2 } from '../geom/se2';
import type { Rng } from '../prob/rng';
import { levenbergMarquardt } from '../trajopt/optim';
import { rankOf } from './field';

export interface PiecewiseControl {
  /** v_1 … v_K followed by ω_1 … ω_K, each held for 1/K of unit time. */
  up: number[];
}

/** The end-state map: integrate the reduced car exactly (constant-twist arcs) and keep θ unwrapped. */
export function endState(x0: Pose2, up: readonly number[]): [number, number, number] {
  const K = up.length / 2;
  const h = 1 / K;
  let q = x0;
  let th = x0.theta;
  for (let i = 0; i < K; i++) {
    const v = up[i];
    const w = up[K + i];
    q = boxplus(q, [v * h, 0, w * h]);
    th += w * h;
  }
  return [q.x, q.y, th];
}

/** Dense trace of the same integration, for drawing. */
export function endStateTrace(x0: Pose2, up: readonly number[], sub = 8): Pose2[] {
  const K = up.length / 2;
  const h = 1 / K / sub;
  const out: Pose2[] = [x0];
  let q = x0;
  for (let i = 0; i < K; i++) for (let s = 0; s < sub; s++) {
    q = boxplus(q, [up[i] * h, 0, up[K + i] * h]);
    out.push(q);
  }
  return out;
}

/** ∂f/∂uᵖ by central differences (3 × r). */
export function endStateJacobian(x0: Pose2, up: readonly number[], h = 1e-6): number[][] {
  const r = up.length;
  const J = [new Array<number>(r).fill(0), new Array<number>(r).fill(0), new Array<number>(r).fill(0)];
  for (let j = 0; j < r; j++) {
    const a = up.slice();
    const b = up.slice();
    a[j] += h;
    b[j] -= h;
    const fa = endState(x0, a);
    const fb = endState(x0, b);
    for (let i = 0; i < 3; i++) J[i][j] = (fa[i] - fb[i]) / (2 * h);
  }
  return J;
}

/** rank ∂f/∂uᵖ — the singular-control test. */
export function endStateRank(x0: Pose2, up: readonly number[]): number {
  const J = endStateJacobian(x0, up);
  // rankOf works on columns of the transpose too; rows of J as vectors span the same row space.
  return rankOf(J, 1e-6);
}

/**
 * Append a generic loop: m random (v, ω) pieces followed by the same pieces
 * reversed and negated. The net motion of the appended part is exactly zero
 * for the driftless car (each arc is undone by its own reverse), so the end
 * state does not move — only the Jacobian gains rank.
 */
export function appendGenericLoop(up: readonly number[], m: number, rng: Rng, scale = 0.5): number[] {
  const K = up.length / 2;
  const v = up.slice(0, K);
  const w = up.slice(K);
  const rv: number[] = [];
  const rw: number[] = [];
  for (let i = 0; i < m; i++) {
    rv.push(rng.uniform(-scale, scale));
    rw.push(rng.uniform(-scale, scale));
  }
  // The time base is fixed (t_f = 1, equal pieces), so stretching K → K + 2m
  // rescales every existing piece's duration; to keep f unchanged the existing
  // rates are multiplied by (K + 2m)/K.
  const K2 = K + 2 * m;
  const sc = K2 / K;
  const vNew = [...v.map((x) => x * sc), ...rv.map((x) => x), ...rv.slice().reverse().map((x) => -x)];
  const wNew = [...w.map((x) => x * sc), ...rw.map((x) => x), ...rw.slice().reverse().map((x) => -x)];
  return [...vNew, ...wNew];
}

export interface GradientResult {
  up: number[];
  /** ‖e‖ after each iteration (index 0 = initial). */
  errors: number[];
  iterations: number;
  converged: boolean;
  /** Rank of ∂f/∂uᵖ at the start, and after any loops were appended. */
  rankHistory: number[];
  loopsAppended: number;
}

export interface GradientOptions {
  K?: number;
  maxIter?: number;
  eps?: number;
  /** Initial guess; zeros (the singular control) by default. */
  up0?: number[];
  rng: Rng;
  loopPieces?: number;
}

const errVec = (x0: Pose2, up: readonly number[], goal: Pose2): number[] => {
  const f = endState(x0, up);
  return [f[0] - goal.x, f[1] - goal.y, f[2] - goal.theta];
};
const norm = (e: readonly number[]) => Math.hypot(...e);

/**
 * Choset's iteration uᵖ ← uᵖ + αv, vᵀ = −eᵀ ∂f/∂uᵖ, α by backtracking from a
 * growing trial step (plain gradient descent on ½‖e‖²; slow near the end,
 * because the sideways direction is a bracket direction and its singular value
 * is small — Chapter 20's ε² showing up as conditioning).
 */
export function steerGradient(x0: Pose2, goal: Pose2, opts: GradientOptions): GradientResult {
  const K = opts.K ?? 6;
  const maxIter = opts.maxIter ?? 4000;
  const eps = opts.eps ?? 1e-4;
  let up = opts.up0 ? opts.up0.slice() : new Array<number>(2 * K).fill(0);
  const rankHistory = [endStateRank(x0, up)];
  let loops = 0;
  if (rankHistory[0] < 3) {
    up = appendGenericLoop(up, opts.loopPieces ?? 2, opts.rng);
    loops++;
    rankHistory.push(endStateRank(x0, up));
  }
  let e = errVec(x0, up, goal);
  const errors = [norm(e)];
  let alpha = 1;
  let it = 0;
  for (; it < maxIter && norm(e) > eps; it++) {
    const J = endStateJacobian(x0, up);
    const g = up.map((_, j) => J[0][j] * e[0] + J[1][j] * e[1] + J[2][j] * e[2]);
    const gn2 = g.reduce((s, x) => s + x * x, 0);
    if (gn2 < 1e-24) {
      // Stuck at a singular control mid-run: append another loop and continue.
      up = appendGenericLoop(up, opts.loopPieces ?? 2, opts.rng);
      loops++;
      rankHistory.push(endStateRank(x0, up));
      continue;
    }
    const f0 = 0.5 * norm(e) ** 2;
    alpha *= 2;
    let accepted = false;
    for (let tries = 0; tries < 40; tries++) {
      const trial = up.map((u, j) => u - alpha * g[j]);
      const et = errVec(x0, trial, goal);
      if (0.5 * norm(et) ** 2 <= f0 - 1e-4 * alpha * gn2) {
        up = trial;
        e = et;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) break;
    errors.push(norm(e));
  }
  return { up, errors, iterations: it, converged: norm(e) <= eps, rankHistory, loopsAppended: loops };
}

/**
 * The same end-state problem as a nonlinear least-squares program solved by
 * Chapter 19's Levenberg–Marquardt — §12.5.3's "nonlinear optimization", with
 * its caveat intact: it converges from here because the generic loop has
 * already made the Jacobian full rank, not because LM knows anything about cars.
 */
export function steerNewton(x0: Pose2, goal: Pose2, opts: GradientOptions): GradientResult {
  const K = opts.K ?? 6;
  let up = opts.up0 ? opts.up0.slice() : new Array<number>(2 * K).fill(0);
  const rankHistory = [endStateRank(x0, up)];
  let loops = 0;
  if (rankHistory[0] < 3) {
    up = appendGenericLoop(up, opts.loopPieces ?? 2, opts.rng);
    loops++;
    rankHistory.push(endStateRank(x0, up));
  }
  const res = levenbergMarquardt(
    {
      residual: (u) => errVec(x0, u, goal),
      jacobian: (u) => endStateJacobian(x0, u),
    },
    up,
    { maxIter: opts.maxIter ?? 100, tolGrad: 1e-14, tolStep: 1e-14, tolCost: 1e-20 },
  );
  const errors = res.history.map((c) => Math.sqrt(2 * c));
  const final = norm(errVec(x0, res.x, goal));
  return { up: res.x, errors, iterations: res.iterations, converged: final <= (opts.eps ?? 1e-9), rankHistory, loopsAppended: loops };
}
