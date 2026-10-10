/**
 * A sequential-convex recipe in the manner of TrajOpt (Schulman et al. 2013,
 * 2014) — and only in its manner. TrajOpt convexifies the objective and the
 * constraints, solves a QP inside an ℓ∞ trust region, and raises an ℓ₁ penalty
 * on constraint violation in an outer loop. This file keeps that *shape* and
 * replaces the QP with the book's own Levenberg–Marquardt on a hinge-residual
 * merit function, boxed by the same trust radius. The chapter says so in so
 * many words: `sco` is a labeled recipe, not TrajOpt.
 *
 *     minimize  ½ ‖K ξ‖²  +  μ Σ_k |d_safe − sd(q_k)|⁺
 *
 * for a point (or disc) robot in the plane, sd the signed distance from
 * `field`. The collision constraint is *linearized at the witness*:
 * sd(q + δq) ≈ sd(q) + n̂ᵀ δq with n̂ = ∇sd — exactly TrajOpt's contact-normal
 * linearization — and the hinge of that affine function is the convex model.
 * Each outer iteration: convexify at the current ξ, take an LM step with
 * ‖δξ‖∞ ≤ Δ on the convex model, accept it if the *true* merit improved
 * (expand Δ) and reject it otherwise (shrink Δ); when the inner loop
 * stalls, multiply μ until the worst violation is below tolerance.
 *
 * Why constraints beat costs for collision: CHOMP's obstacle term is a cost and
 * stops pushing when the cost's gradient balances smoothness — "nearly out".
 * The ℓ₁ penalty is exact for finite μ, so at convergence the violation is
 * zero, not small.
 */

import type { Vec } from '../prob/linalg';
import type { DistanceField2 } from './field';
import { smoothness } from './chomp';
import { levenbergMarquardt, type LeastSquares } from './optim';

export interface ScoProblem {
  q0: Vec;
  qEnd: Vec;
  field: DistanceField2;
  /** Required clearance d_safe (robot radius plus margin). */
  dSafe: number;
}

export interface ScoOptions {
  mu0?: number;
  muGrowth?: number;
  trust0?: number;
  maxOuter?: number;
  maxPenalty?: number;
  tolViolation?: number;
}

export interface ScoResult {
  xi: Vec[];
  /** Merit history: smoothness + μ Σ hinge, one entry per accepted step, with the μ it was measured under. */
  merit: number[];
  muTrace: number[];
  smoothness: number;
  /** Worst remaining violation max(0, d_safe − sd). */
  violation: number;
  mu: number;
  outerIterations: number;
}

/** Σ_k |d_safe − sd(q_k)|⁺ and the per-waypoint hinge values. */
export function violation(p: ScoProblem, xi: readonly Vec[]): { total: number; worst: number; hinge: number[] } {
  const hinge = xi.map((q) => Math.max(0, p.dSafe - p.field.dist({ x: q[0], y: q[1] })));
  return { total: hinge.reduce((a, b) => a + b, 0), worst: Math.max(0, ...hinge), hinge };
}

function flatten(xi: readonly Vec[]): Vec {
  return xi.flat();
}
function unflatten(x: Vec, n: number): Vec[] {
  const out: Vec[] = [];
  for (let k = 0; k < x.length; k += n) out.push(x.slice(k, k + n));
  return out;
}

/** The true (nonconvex) merit function. */
export function merit(p: ScoProblem, xi: readonly Vec[], mu: number): number {
  return smoothness(xi, p.q0, p.qEnd).cost + mu * violation(p, xi).total;
}

export function sco(p: ScoProblem, xi0: readonly Vec[], opts: ScoOptions = {}): ScoResult {
  const n = p.q0.length;
  const mu0 = opts.mu0 ?? 10;
  const growth = opts.muGrowth ?? 10;
  const maxOuter = opts.maxOuter ?? 60;
  const maxPenalty = opts.maxPenalty ?? 6;
  const tolV = opts.tolViolation ?? 1e-6;
  let trust = opts.trust0 ?? 0.3;
  let mu = mu0;
  let xi = xi0.map((q) => q.slice());
  const meritHist = [merit(p, xi, mu)];
  const muTrace = [mu];
  let outer = 0;

  for (let pen = 0; pen < maxPenalty; pen++) {
    let stalls = 0;
    for (let it = 0; it < maxOuter && stalls < 4; it++) {
      outer++;
      // Convexify: linearize sd at every waypoint's witness normal.
      const base = xi.map((q) => {
        const x = { x: q[0], y: q[1] };
        return { sd: p.field.dist(x), nHat: p.field.grad(x) };
      });
      const xFlat = flatten(xi);
      const model: LeastSquares = {
        residual: (x) => {
          const cand = unflatten(x, n);
          const r: Vec = [];
          // Smoothness residuals K ξ + e, including the two fixed ends.
          const full = [p.q0, ...cand, p.qEnd];
          for (let k = 0; k + 1 < full.length; k++) for (let i = 0; i < n; i++) r.push(full[k + 1][i] - full[k][i]);
          // Hinge of the affine model: √μ · max(0, d_safe − sd_k − n̂ᵀ(q − q_k)).
          cand.forEach((q, k) => {
            const lin = base[k].sd + base[k].nHat[0] * (q[0] - xi[k][0]) + base[k].nHat[1] * (q[1] - xi[k][1]);
            r.push(Math.sqrt(mu) * Math.max(0, p.dSafe - lin));
          });
          return r;
        },
      };
      const step = levenbergMarquardt(model, xFlat, { maxIter: 8, trust, lambda0: 1e-2, fdStep: 1e-7 });
      const cand = unflatten(step.x, n);
      const before = merit(p, xi, mu);
      const after = merit(p, cand, mu);
      if (after < before - 1e-12) {
        xi = cand;
        meritHist.push(after);
        muTrace.push(mu);
        trust = Math.min(trust * 1.5, 2);
      } else {
        trust *= 0.5;
        stalls++;
        if (trust < 1e-5) break;
      }
    }
    if (violation(p, xi).worst < tolV) break;
    mu *= growth;
    trust = Math.max(trust, opts.trust0 ?? 0.3);
  }
  return { xi, merit: meritHist, muTrace, smoothness: smoothness(xi, p.q0, p.qEnd).cost, violation: violation(p, xi).worst, mu, outerIterations: outer };
}
