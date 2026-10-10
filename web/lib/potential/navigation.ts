/**
 * Navigation functions — Choset §4.6, Rimon & Koditschek (1992).
 *
 * A navigation function (Def. 4.6.1) is a smooth φ : Q_free → [0, 1] with a
 * unique minimum at the goal, uniformly maximal on ∂Q_free, and Morse. On a
 * *sphere world* — a disc bounded by a circle, with disjoint disc obstacles —
 * eq. (4.13) is one, for κ large enough:
 *
 *     φ(q) = d²(q, q_goal) / [ d^{2κ}(q, q_goal) + β(q) ]^{1/κ},
 *     β(q) = Π_i β_i(q),   β_0 = r_0² − d²(q, q_0),   β_i = d²(q, q_i) − r_i².
 *
 * The construction is γ_κ/β (zero only at the goal, +∞ on every boundary),
 * squashed to [0, 1] by the analytic switch σ_1(x) = x/(1 + x), then sharpened
 * by ξ_κ(x) = x^{1/κ}. The switches are monotone, so they move no critical
 * point and change no critical point's type — which is why `criticalPoints`
 * may look for them on log(γ_κ/β) instead, where the arithmetic is tame, and
 * why the κ sweep of Choset Figs. 4.15–4.16 (three minima at κ = 3, one at 10)
 * can be counted rather than eyeballed.
 *
 * The star world of §4.6.2 is reached by a diffeomorphism h_λ (eqs. 4.14–4.19)
 * that maps each star-shaped obstacle onto a disc; φ ∘ h_λ is then a navigation
 * function on the star world because critical points correspond one-to-one
 * under a diffeomorphism.
 */

import type { Point2 } from '../sim/world';
import { classifyCritical, eigen2, hessian, FnPotential, type CriticalType, type Potential, type Vec } from './field';

export interface Disc {
  c: Point2;
  r: number;
}

/** A sphere world: the bounding circle (index 0) and n disjoint disc obstacles (indices 1…n). */
export interface SphereWorld {
  center: Point2;
  r0: number;
  obstacles: Disc[];
}

/** β_0 and β_i of §4.6.1, as one list indexed 0…n. */
export function betas(world: SphereWorld, q: Vec): number[] {
  const out = [world.r0 * world.r0 - (q[0] - world.center.x) ** 2 - (q[1] - world.center.y) ** 2];
  for (const o of world.obstacles) out.push((q[0] - o.c.x) ** 2 + (q[1] - o.c.y) ** 2 - o.r * o.r);
  return out;
}

/** ∇β_i: ∓2(q − q_i). */
export function betaGrads(world: SphereWorld, q: Vec): Vec[] {
  const out: Vec[] = [[-2 * (q[0] - world.center.x), -2 * (q[1] - world.center.y)]];
  for (const o of world.obstacles) out.push([2 * (q[0] - o.c.x), 2 * (q[1] - o.c.y)]);
  return out;
}

/** β = Π β_i and ∇β by the product rule. */
export function betaProduct(world: SphereWorld, q: Vec): { beta: number; grad: Vec } {
  const b = betas(world, q);
  const g = betaGrads(world, q);
  let beta = 1;
  for (const v of b) beta *= v;
  const grad: Vec = [0, 0];
  for (let i = 0; i < b.length; i++) {
    let others = 1;
    for (let j = 0; j < b.length; j++) if (j !== i) others *= b[j];
    grad[0] += g[i][0] * others;
    grad[1] += g[i][1] * others;
  }
  return { beta, grad };
}

export function inFreeSpace(world: SphereWorld, q: Vec): boolean {
  return betas(world, q).every((b) => b > 0);
}

/** Eq. (4.13) with its analytic gradient (quotient rule). */
export class NavigationFunction implements Potential {
  readonly dim = 2;

  constructor(
    readonly world: SphereWorld,
    readonly goal: Vec,
    readonly kappa: number,
  ) {}

  private pieces(q: Vec) {
    const dx = q[0] - this.goal[0];
    const dy = q[1] - this.goal[1];
    const A = dx * dx + dy * dy; // γ_1 = d²
    const { beta, grad: gb } = betaProduct(this.world, q);
    const Ak = Math.pow(A, this.kappa); // γ_κ = d^{2κ}
    const B = Ak + beta;
    return { dx, dy, A, Ak, beta, gb, B };
  }

  value(q: Vec): number {
    const { A, B } = this.pieces(q);
    if (B <= 0) return 1; // on or inside an obstacle: the maximal value
    return A * Math.pow(B, -1 / this.kappa);
  }

  gradient(q: Vec): Vec {
    const k = this.kappa;
    const { dx, dy, A, B, gb } = this.pieces(q);
    if (B <= 0) return [0, 0];
    const Bm = Math.pow(B, -1 / k);
    const dA: Vec = [2 * dx, 2 * dy];
    // ∇B = κ A^{κ−1} ∇A + ∇β
    const coef = k * Math.pow(A, k - 1);
    const dB: Vec = [coef * dA[0] + gb[0], coef * dA[1] + gb[1]];
    // ∇φ = ∇A·B^{−1/κ} − (A/κ) B^{−1/κ − 1} ∇B
    const c2 = (A / k) * (Bm / B);
    return [dA[0] * Bm - c2 * dB[0], dA[1] * Bm - c2 * dB[1]];
  }

  /** γ_κ/β before the switches — Choset's unbounded first attempt. */
  rawRatio(q: Vec): number {
    const { Ak, beta } = this.pieces(q);
    return Ak / beta;
  }
}

/* -------------------------------------------------------------------------- */
/* Critical points of φ                                                        */
/* -------------------------------------------------------------------------- */

export interface CriticalPoint {
  q: Vec;
  type: CriticalType;
  /** Eigenvalues of the Hessian of log(γ_κ/β) — same signs as φ's Hessian there. */
  eigs: [number, number];
  /** φ at the point. */
  value: number;
}

/**
 * ψ(q) = log(γ_κ/β) = 2κ log d − log β. Its critical points are exactly the
 * non-goal critical points of φ (log, σ and ξ are strictly increasing, and a
 * strictly increasing C² reparametrisation preserves a nondegenerate critical
 * point's type), but its gradient is O(1) where φ's is 10⁻⁸ — the plateau
 * problem Choset's last paragraph of §4.6.1 admits, sidestepped for the count.
 */
export function logRatio(world: SphereWorld, goal: Vec, kappa: number): Potential {
  const f = (q: Vec) => {
    const d2 = (q[0] - goal[0]) ** 2 + (q[1] - goal[1]) ** 2;
    let lb = 0;
    for (const b of betas(world, q)) lb += Math.log(Math.max(b, 1e-300));
    return kappa * Math.log(Math.max(d2, 1e-300)) - lb;
  };
  const g = (q: Vec): Vec => {
    const dx = q[0] - goal[0];
    const dy = q[1] - goal[1];
    const d2 = dx * dx + dy * dy;
    const b = betas(world, q);
    const gb = betaGrads(world, q);
    const out: Vec = [(2 * kappa * dx) / d2, (2 * kappa * dy) / d2];
    for (let i = 0; i < b.length; i++) {
      out[0] -= gb[i][0] / b[i];
      out[1] -= gb[i][1] / b[i];
    }
    return out;
  };
  return new FnPotential(2, f, g);
}

/**
 * Find the critical points of φ on a sphere world by damped Newton on ∇ψ from
 * a lattice of seeds, polish, deduplicate, classify. The goal is appended as
 * the minimum it always is. `minima` counts it, so Choset's "κ = 10: one
 * minimum" reads as `minima === 1`.
 */
export function criticalPoints(
  world: SphereWorld,
  goal: Vec,
  kappa: number,
  opts: { seedsPerAxis?: number; newtonIters?: number; tol?: number } = {},
): { points: CriticalPoint[]; minima: number; saddles: number; maxima: number } {
  const { seedsPerAxis = 36, newtonIters = 60, tol = 1e-9 } = opts;
  const psi = logRatio(world, goal, kappa);
  const phi = new NavigationFunction(world, goal, kappa);
  const found: CriticalPoint[] = [];
  const R = world.r0;
  const c = world.center;
  for (let a = 0; a < seedsPerAxis; a++) {
    for (let b = 0; b < seedsPerAxis; b++) {
      let q: Vec = [c.x - R + ((a + 0.5) / seedsPerAxis) * 2 * R, c.y - R + ((b + 0.5) / seedsPerAxis) * 2 * R];
      if (!inFreeSpace(world, q)) continue;
      if (Math.hypot(q[0] - goal[0], q[1] - goal[1]) < 1e-3) continue;
      let ok = false;
      for (let it = 0; it < newtonIters; it++) {
        const g = psi.gradient(q);
        const gn = Math.hypot(g[0], g[1]);
        if (gn < tol) {
          ok = true;
          break;
        }
        const H = hessian(psi, q, 1e-5);
        const det = H[0][0] * H[1][1] - H[0][1] * H[1][0];
        let step: Vec;
        if (Math.abs(det) < 1e-14) {
          step = [-g[0] * 1e-2, -g[1] * 1e-2];
        } else {
          step = [-(H[1][1] * g[0] - H[0][1] * g[1]) / det, -(-H[1][0] * g[0] + H[0][0] * g[1]) / det];
        }
        // Damp so a Newton step never jumps out of the free space.
        let lambda = 1;
        let next: Vec = [q[0] + step[0], q[1] + step[1]];
        while (lambda > 1e-4 && !inFreeSpace(world, next)) {
          lambda *= 0.5;
          next = [q[0] + lambda * step[0], q[1] + lambda * step[1]];
        }
        if (lambda <= 1e-4) break;
        const moved = Math.hypot(next[0] - q[0], next[1] - q[1]);
        q = next;
        if (moved < 1e-13) {
          ok = Math.hypot(...(psi.gradient(q) as [number, number])) < 1e-6;
          break;
        }
      }
      if (!ok || !inFreeSpace(world, q)) continue;
      if (Math.hypot(q[0] - goal[0], q[1] - goal[1]) < 1e-4) continue;
      if (found.some((p) => Math.hypot(p.q[0] - q[0], p.q[1] - q[1]) < 1e-5)) continue;
      const eigs = eigen2(hessian(psi, q, 1e-5));
      found.push({ q, type: classifyCritical(eigs, 1e-6), eigs, value: phi.value(q) });
    }
  }
  const goalPt: CriticalPoint = { q: goal.slice(), type: 'minimum', eigs: [0, 0], value: 0 };
  const points = [goalPt, ...found];
  return {
    points,
    minima: points.filter((p) => p.type === 'minimum').length,
    saddles: points.filter((p) => p.type === 'saddle').length,
    maxima: points.filter((p) => p.type === 'maximum').length,
  };
}

/**
 * The five-obstacle sphere world in the spirit of Choset Fig. 4.14: bounded by
 * a circle of radius 4.5, five disjoint discs, goal near the right. At κ = 3 the
 * navigation function has spurious minima; at κ = 10 exactly one (the check
 * pins both counts).
 */
export const FIVE_DISC_WORLD: SphereWorld = {
  center: { x: 0, y: 0 },
  r0: 4.5,
  obstacles: [
    { c: { x: -2.3, y: 1.6 }, r: 0.8 },
    { c: { x: 1.9, y: 2.3 }, r: 0.7 },
    { c: { x: -1.1, y: -2.3 }, r: 0.9 },
    { c: { x: 1.6, y: -1.7 }, r: 0.6 },
    { c: { x: 0.2, y: 0.5 }, r: 0.55 },
  ],
};

export const FIVE_DISC_GOAL: Vec = [3.3, -0.2];

/* -------------------------------------------------------------------------- */
/* §4.6.2 — star worlds and the diffeomorphism h_λ                              */
/* -------------------------------------------------------------------------- */

/**
 * A star-shaped obstacle with an analytic β_i: the boundary radius about the
 * centre q_i is ρ(θ) = r (1 + a (1.15 + cos(kθ + φ₀))), so ρ ≥ r(1 + 0.15a) > r,
 * and
 *
 *     β_i(q) = d²(q, q_i)/ρ(θ)² − 1,
 *
 * negative inside, zero on the boundary, positive outside, and ≥ −1 everywhere
 * (so (1 + β_i)^{1/2} in eq. (4.15) is real). The model sphere for this star is
 * the disc of radius r at the same centre (p_i = q_i, r_i = r), and that disc is
 * *inscribed* in the star: every boundary point moves inward under T_i. Rimon &
 * Koditschek need this — if a model disc poked outside its star, T_i − id would
 * point outward in the valleys while ∇s_i points inward, and det J_{h_λ} would
 * go negative in the switch collar for every large λ (we measured it before
 * choosing the inscribed disc).
 */
export interface StarObstacle {
  c: Point2;
  r: number;
  lobes: number;
  amp: number;
  phase: number;
}

export interface StarWorld {
  center: Point2;
  r0: number;
  obstacles: StarObstacle[];
}

export function starRadius(o: StarObstacle, theta: number): number {
  return o.r * (1 + o.amp * (1.15 + Math.cos(o.lobes * theta + o.phase)));
}

export function starBeta(o: StarObstacle, q: Vec): number {
  const dx = q[0] - o.c.x;
  const dy = q[1] - o.c.y;
  const rho = starRadius(o, Math.atan2(dy, dx));
  return (dx * dx + dy * dy) / (rho * rho) - 1;
}

/** Boundary of a star obstacle, sampled, for drawing and for the check that h_λ maps it onto a circle. */
export function starBoundary(o: StarObstacle, n = 96): Point2[] {
  const out: Point2[] = [];
  for (let k = 0; k < n; k++) {
    const th = (2 * Math.PI * k) / n;
    const rho = starRadius(o, th);
    out.push({ x: o.c.x + rho * Math.cos(th), y: o.c.y + rho * Math.sin(th) });
  }
  return out;
}

/** The sphere world h_λ maps onto: same bounding circle, each star replaced by its model disc. */
export function modelSphereWorld(world: StarWorld): SphereWorld {
  return { center: world.center, r0: world.r0, obstacles: world.obstacles.map((o) => ({ c: o.c, r: o.r })) };
}

/** Every β_i of the star world, index 0 the bounding circle. */
export function starBetas(world: StarWorld, q: Vec): number[] {
  const out = [world.r0 * world.r0 - (q[0] - world.center.x) ** 2 - (q[1] - world.center.y) ** 2];
  for (const o of world.obstacles) out.push(starBeta(o, q));
  return out;
}

export function inStarFreeSpace(world: StarWorld, q: Vec): boolean {
  return starBetas(world, q).every((b) => b > 0);
}

/**
 * A λ that makes the switches switch. s_i = γ_κ β̄_i / (γ_κ β̄_i + λ β_i) is ½
 * where λ β_i = γ_κ β̄_i, and γ_κ = d^{2κ} is enormous away from the goal, so a
 * λ of order one leaves every switch pinned at 1 and h_λ nowhere near a
 * diffeomorphism. Rimon & Koditschek prove a λ₀ exists beyond which h_λ is one;
 * this takes λ₀ = max over a lattice of free points of γ_κ β̄_i / β_i, restricted
 * to points at least a quarter-radius clear of obstacle i (β_i ≥ 9/16), so that
 * every switch is ≤ ½ outside that collar and rises to 1 only on the boundary.
 * The widget's λ slider is a multiple of this.
 */
export function suitableLambda(world: StarWorld, goal: Vec, kappa: number, perAxis = 40): number {
  let lam = 0;
  const R = world.r0;
  const n = world.obstacles.length;
  for (let a = 0; a < perAxis; a++) {
    for (let bI = 0; bI < perAxis; bI++) {
      const q: Vec = [world.center.x - R + ((a + 0.5) / perAxis) * 2 * R, world.center.y - R + ((bI + 0.5) / perAxis) * 2 * R];
      const b = starBetas(world, q);
      if (b.some((v) => v <= 0)) continue;
      const d2 = (q[0] - goal[0]) ** 2 + (q[1] - goal[1]) ** 2;
      const gammaK = Math.pow(d2, kappa);
      for (let i = 0; i <= n; i++) {
        if (b[i] < 9 / 16) continue;
        let betaBar = 1;
        for (let j = 0; j <= n; j++) if (j !== i) betaBar *= b[j];
        lam = Math.max(lam, (gammaK * betaBar) / b[i]);
      }
    }
  }
  return lam;
}

/**
 * h_λ of eq. (4.19): s_goal·q + Σ_{i=0}^{n} s_i(q, λ) T_i(q), with the translated
 * scaling maps T_i of eqs. (4.14)–(4.15) and the analytic switches s_i of
 * eqs. (4.16)–(4.18). On ∂QO_i every switch but s_i vanishes, so h_λ = T_i
 * there and the star boundary lands on the circle of radius r_i. The bounding
 * circle (index 0) is already a sphere, so its T_0 is the identity — which is
 * what eq. (4.14) reduces to when the star *is* its model disc.
 *
 * Numerics: γ_κ β̄_i is astronomically larger than λ β_i for any λ of order
 * one, so a β_i that is −10⁻¹⁶ by rounding on a boundary flips the *other*
 * obstacles' switches from 0 to garbage (their denominators go negative). The
 * boundary is therefore handled exactly: a β_i within `boundaryTol` of zero
 * (or negative, i.e. inside) sends q to T_i(q) outright, which is the value the
 * algebra assigns there anyway. A λ that actually makes the switches switch is
 * `suitableLambda`; for a λ far below it h_λ is continuous but folds over
 * itself and is no diffeomorphism — the chapter's check measures det J_h.
 */
export function hLambda(world: StarWorld, goal: Vec, kappa: number, lambda: number, boundaryTol = 1e-12): (q: Vec) => Vec {
  const n = world.obstacles.length;
  const T = (i: number, q: Vec, bi: number): Vec => {
    if (i === 0) return [q[0], q[1]];
    const o = world.obstacles[i - 1];
    const dx = q[0] - o.c.x;
    const dy = q[1] - o.c.y;
    const d = Math.hypot(dx, dy) || 1e-12;
    const nu = (Math.sqrt(Math.max(0, 1 + bi)) * o.r) / d; // eq. (4.15)
    return [nu * dx + o.c.x, nu * dy + o.c.y]; // eq. (4.14) with p_i = q_i
  };
  return (q: Vec): Vec => {
    const b = starBetas(world, q);
    // On (or inside) a boundary the switches are exactly {s_i = 1, all others 0}.
    for (let i = 0; i <= n; i++) if (b[i] <= boundaryTol) return T(i, q, Math.max(0, b[i]));
    const d2 = (q[0] - goal[0]) ** 2 + (q[1] - goal[1]) ** 2;
    const gammaK = Math.pow(d2, kappa);
    const out: Vec = [0, 0];
    let sum = 0;
    for (let i = 0; i <= n; i++) {
      let betaBar = 1;
      for (let j = 0; j <= n; j++) if (j !== i) betaBar *= b[j];
      const num = gammaK * betaBar;
      const s = num / (num + lambda * b[i]); // eq. (4.16); every factor is > 0 here
      sum += s;
      const t = T(i, q, b[i]);
      out[0] += s * t[0];
      out[1] += s * t[1];
    }
    const sGoal = 1 - sum; // eq. (4.18)
    out[0] += sGoal * q[0];
    out[1] += sGoal * q[1];
    return out;
  };
}

/** The Jacobian of h_λ by central differences, and its determinant — the diffeomorphism test the check runs. */
export function hLambdaJacobian(h: (q: Vec) => Vec, q: Vec, eps = 1e-6): { J: number[][]; det: number } {
  const J: number[][] = [[0, 0], [0, 0]];
  for (let i = 0; i < 2; i++) {
    const qp = q.slice();
    const qm = q.slice();
    qp[i] += eps;
    qm[i] -= eps;
    const hp = h(qp);
    const hm = h(qm);
    J[0][i] = (hp[0] - hm[0]) / (2 * eps);
    J[1][i] = (hp[1] - hm[1]) / (2 * eps);
  }
  return { J, det: J[0][0] * J[1][1] - J[0][1] * J[1][0] };
}

/**
 * h_λ⁻¹(p) by damped Newton with the finite-difference Jacobian, started at p
 * itself (h_λ is the identity far from the obstacles, so p is a good guess).
 * Returns null if Newton leaves the star world's free space or fails to
 * converge. Used to carry the model world's critical points back onto the star
 * world — the one-to-one correspondence the Sculptor draws.
 */
export function hLambdaInverse(world: StarWorld, h: (q: Vec) => Vec, p: Vec, iters = 40, tol = 1e-10): Vec | null {
  let q: Vec = p.slice();
  if (!inStarFreeSpace(world, q)) {
    // Fall back to the nearest free lattice point around p.
    let found: Vec | null = null;
    for (let r = 0.05; r < 1.5 && !found; r += 0.05) {
      for (let k = 0; k < 16 && !found; k++) {
        const c: Vec = [p[0] + r * Math.cos((2 * Math.PI * k) / 16), p[1] + r * Math.sin((2 * Math.PI * k) / 16)];
        if (inStarFreeSpace(world, c)) found = c;
      }
    }
    if (!found) return null;
    q = found;
  }
  for (let it = 0; it < iters; it++) {
    const hq = h(q);
    const r: Vec = [hq[0] - p[0], hq[1] - p[1]];
    const rn = Math.hypot(r[0], r[1]);
    if (rn < tol) return q;
    const { J, det } = hLambdaJacobian(h, q);
    if (Math.abs(det) < 1e-14) return null;
    const step: Vec = [-(J[1][1] * r[0] - J[0][1] * r[1]) / det, -(-J[1][0] * r[0] + J[0][0] * r[1]) / det];
    let lam = 1;
    let next: Vec = [q[0] + step[0], q[1] + step[1]];
    while (lam > 1e-4 && !inStarFreeSpace(world, next)) {
      lam *= 0.5;
      next = [q[0] + lam * step[0], q[1] + lam * step[1]];
    }
    if (lam <= 1e-4) return null;
    q = next;
  }
  const hq = h(q);
  return Math.hypot(hq[0] - p[0], hq[1] - p[1]) < 1e-6 ? q : null;
}

/**
 * φ ∘ h_λ on the star world: the pulled-back navigation function (§4.6.2),
 * with its gradient by the chain rule through a finite-difference Jacobian
 * of h_λ. Critical points correspond one-to-one with those of φ on the model
 * sphere world, which the Sculptor shows side by side.
 */
export function starNavigationFunction(world: StarWorld, goal: Vec, kappa: number, lambda: number): Potential {
  const model = modelSphereWorld(world);
  const phi = new NavigationFunction(model, goal, kappa);
  const h = hLambda(world, goal, kappa, lambda);
  const value = (q: Vec) => (inStarFreeSpace(world, q) ? phi.value(h(q)) : 1);
  const grad = (q: Vec): Vec => {
    if (!inStarFreeSpace(world, q)) return [0, 0];
    const eps = 1e-6;
    const hq = h(q);
    const gphi = phi.gradient(hq);
    const out: Vec = [0, 0];
    for (let i = 0; i < 2; i++) {
      const qp = q.slice();
      const qm = q.slice();
      qp[i] += eps;
      qm[i] -= eps;
      const hp = h(qp);
      const hm = h(qm);
      // column i of J_h, dotted with ∇φ: (J_hᵀ ∇φ)_i
      out[i] = ((hp[0] - hm[0]) / (2 * eps)) * gphi[0] + ((hp[1] - hm[1]) / (2 * eps)) * gphi[1];
    }
    return out;
  };
  return new FnPotential(2, value, grad);
}

/** The five discs of `FIVE_DISC_WORLD` grown lobes — the Sculptor's "star mode". */
export const FIVE_STAR_WORLD: StarWorld = {
  center: FIVE_DISC_WORLD.center,
  r0: FIVE_DISC_WORLD.r0,
  obstacles: FIVE_DISC_WORLD.obstacles.map((o, i) => ({
    c: o.c,
    r: o.r,
    lobes: 3 + (i % 3),
    amp: 0.14 + 0.03 * i,
    phase: 0.7 * i,
  })),
};
