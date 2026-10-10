/**
 * CHOMP — covariant Hamiltonian optimization for motion planning (Ratliff,
 * Zucker, Bagnell & Srinivasa 2009; Zucker et al. 2013), as the chapter uses it.
 *
 * A trajectory is ξ = (q₁, …, q_{N−1}) with fixed ends q₀, q_N. The objective
 *
 *     F(ξ) = F_smooth(ξ) + λ F_obs(ξ),
 *     F_smooth = ½ Σ_k ‖q_{k+1} − q_k‖²  = ½ ‖K ξ + e‖²,      A = KᵀK = tridiag(−1, 2, −1),
 *     F_obs    = Σ_k Σ_β c(D(x_β(q_k))) · w_k,
 *
 * where x_β is a body point and c(d) the obstacle cost of a distance:
 *     c(d) = −d + ε/2 for d < 0,   (ε − d)²/(2ε) on [0, ε],   0 above — C¹ at both seams.
 *
 * The update is steepest descent in the *smoothness metric*,
 *     ξ ← ξ − (1/η) A⁻¹ ∇F,
 * so a push on one waypoint is spread along the whole path instead of putting
 * a kink in it. A is block-tridiagonal and A⁻¹∇F is one Thomas sweep per
 * coordinate, O(N).
 *
 * Two forms of the obstacle gradient. `pointwise` is the one the micro-example
 * pins: ∇_{q_k} F_obs = Σ_β J_βᵀ c′(D) ∇D. `arclength` is the full CHOMP form,
 * invariant to retiming: F_obs = Σ_β ∫ c(D(x_β)) ‖x′_β‖ ds, whose gradient is
 *     Σ_β J_βᵀ ‖x′‖ [ (I − x̂′x̂′ᵀ) ∇c − c κ ],   κ = (I − x̂′x̂′ᵀ) x″ / ‖x′‖²,
 * with x′, x″ by finite differences along the trajectory. The projection says
 * "do not fight the path's own direction"; the curvature term says "a longer
 * path through cost costs more".
 */

import type { Mat, Vec } from '../prob/linalg';
import type { Point2 } from '../sim/world';
import type { DistanceField2 } from './field';

/** A body point: workspace position and its 2×n Jacobian at q. */
export type BodyPoint = (q: Vec) => { x: Point2; J: Mat };

export type ObstacleGradient = 'pointwise' | 'arclength';

export interface ChompProblem {
  q0: Vec;
  qEnd: Vec;
  field: DistanceField2;
  body: BodyPoint[];
  /** Cost margin ε: c(d) is zero beyond it. */
  eps: number;
  /** λ_obs — the weight of the obstacle functional. */
  lambdaObs: number;
  /** Step is 1/η. */
  eta: number;
  obstacleForm: ObstacleGradient;
}

export interface ChompStats {
  fSmooth: number;
  fObs: number;
  /** F_smooth + λ F_obs. */
  f: number;
  /** min over waypoints and body points of D. */
  minClearance: number;
  /** ‖∇F‖∞ before the step. */
  gradInf: number;
}

/** The point robot: one body point, identity Jacobian. */
export const POINT_BODY: BodyPoint[] = [(q) => ({ x: { x: q[0], y: q[1] }, J: [[1, 0], [0, 1]] })];

/** c(d) — Zucker et al. eq. (18). */
export function obstacleCost(d: number, eps: number): number {
  if (d < 0) return -d + eps / 2;
  if (d < eps) return ((eps - d) * (eps - d)) / (2 * eps);
  return 0;
}

/** c′(d): −1 below zero, −(ε − d)/ε on [0, ε], 0 above. Continuous at both seams. */
export function obstacleCostSlope(d: number, eps: number): number {
  if (d < 0) return -1;
  if (d < eps) return -(eps - d) / eps;
  return 0;
}

/** F_smooth and its gradient A ξ + b: 2q_k − q_{k−1} − q_{k+1} at each free waypoint. */
export function smoothness(xi: readonly Vec[], q0: Vec, qEnd: Vec): { cost: number; grad: Vec[] } {
  const n = q0.length;
  const full = [q0, ...xi, qEnd];
  let cost = 0;
  for (let k = 0; k + 1 < full.length; k++) for (let i = 0; i < n; i++) cost += 0.5 * (full[k + 1][i] - full[k][i]) ** 2;
  const grad: Vec[] = [];
  for (let k = 1; k < full.length - 1; k++) grad.push(full[k].map((v, i) => 2 * v - full[k - 1][i] - full[k + 1][i]));
  return { cost, grad };
}

/** Body-point positions along the trajectory, including the fixed ends: pos[k][β]. */
function bodyPositions(p: ChompProblem, xi: readonly Vec[]): Point2[][] {
  const full = [p.q0, ...xi, p.qEnd];
  return full.map((q) => p.body.map((b) => b(q).x));
}

/** F_obs and ∇F_obs in the requested form. Also reports the minimum clearance over free waypoints. */
export function obstacleTerm(p: ChompProblem, xi: readonly Vec[]): { cost: number; grad: Vec[]; minClearance: number } {
  const n = p.q0.length;
  const pos = bodyPositions(p, xi);
  const grad: Vec[] = xi.map(() => new Array<number>(n).fill(0));
  let cost = 0;
  let minClearance = Infinity;
  for (let k = 1; k < pos.length - 1; k++) {
    const q = xi[k - 1];
    for (let bIdx = 0; bIdx < p.body.length; bIdx++) {
      const { x, J } = p.body[bIdx](q);
      const d = p.field.dist(x);
      minClearance = Math.min(minClearance, d);
      const c = obstacleCost(d, p.eps);
      const cs = obstacleCostSlope(d, p.eps);
      const gD = p.field.grad(x);
      const gc: [number, number] = [cs * gD[0], cs * gD[1]];
      let f: [number, number];
      let w = 1;
      if (p.obstacleForm === 'pointwise') {
        f = gc;
      } else {
        // Workspace velocity and acceleration of this body point by central differences along the path.
        const xm = pos[k - 1][bIdx];
        const xp = pos[k + 1][bIdx];
        const vx = 0.5 * (xp.x - xm.x);
        const vy = 0.5 * (xp.y - xm.y);
        const ax = xp.x - 2 * x.x + xm.x;
        const ay = xp.y - 2 * x.y + xm.y;
        const speed = Math.hypot(vx, vy);
        if (speed < 1e-9) {
          f = gc;
        } else {
          const tx = vx / speed;
          const ty = vy / speed;
          // (I − t tᵀ) applied to ∇c and to x″.
          const pcx = gc[0] - tx * (tx * gc[0] + ty * gc[1]);
          const pcy = gc[1] - ty * (tx * gc[0] + ty * gc[1]);
          const kx = (ax - tx * (tx * ax + ty * ay)) / (speed * speed);
          const ky = (ay - ty * (tx * ax + ty * ay)) / (speed * speed);
          f = [speed * (pcx - c * kx), speed * (pcy - c * ky)];
          w = speed;
        }
      }
      cost += c * w;
      // Pull back to joint space: Jᵀ f.
      for (let i = 0; i < n; i++) grad[k - 1][i] += J[0][i] * f[0] + J[1][i] * f[1];
    }
  }
  return { cost, grad, minClearance };
}

/**
 * Solve A y = r for the block-tridiagonal A = tridiag(−1, 2, −1) ⊗ I, one Thomas
 * sweep per coordinate. This is the covariant preconditioner: A⁻¹ is dense and
 * positive, so a push at one waypoint moves every waypoint, most where it was pushed.
 */
export function solveSmoothnessMetric(r: readonly Vec[]): Vec[] {
  const m = r.length;
  if (m === 0) return [];
  const n = r[0].length;
  const out: Vec[] = r.map(() => new Array<number>(n).fill(0));
  for (let i = 0; i < n; i++) {
    // Forward sweep with the constant stencil: c′_k = −1/b′_{k−1}, b′_k = 2 − 1/b′_{k−1}.
    const bp = new Array<number>(m).fill(0);
    const dp = new Array<number>(m).fill(0);
    bp[0] = 2;
    dp[0] = r[0][i];
    for (let k = 1; k < m; k++) {
      const w = -1 / bp[k - 1];
      bp[k] = 2 - w * -1;
      dp[k] = r[k][i] - w * dp[k - 1];
    }
    out[m - 1][i] = dp[m - 1] / bp[m - 1];
    for (let k = m - 2; k >= 0; k--) out[k][i] = (dp[k] + out[k + 1][i]) / bp[k];
  }
  return out;
}

/** The dense A = tridiag(−1, 2, −1) of size m, for the checks that compare against the sweep. */
export function smoothnessMatrix(m: number): Mat {
  const A: Mat = [];
  for (let i = 0; i < m; i++) {
    const row = new Array<number>(m).fill(0);
    row[i] = 2;
    if (i > 0) row[i - 1] = -1;
    if (i + 1 < m) row[i + 1] = -1;
    A.push(row);
  }
  return A;
}

/** Full objective and gradient, without stepping. */
export function chompEvaluate(p: ChompProblem, xi: readonly Vec[]): { stats: ChompStats; grad: Vec[] } {
  const s = smoothness(xi, p.q0, p.qEnd);
  const o = obstacleTerm(p, xi);
  const grad = s.grad.map((g, k) => g.map((v, i) => v + p.lambdaObs * o.grad[k][i]));
  const gradInf = grad.reduce((m, g) => Math.max(m, ...g.map(Math.abs)), 0);
  return { stats: { fSmooth: s.cost, fObs: o.cost, f: s.cost + p.lambdaObs * o.cost, minClearance: o.minClearance, gradInf }, grad };
}

/**
 * One covariant step ξ ← ξ − (1/η) A⁻¹ ∇F, in place. Returns the statistics
 * *before* the step — the widget's cost bars and its "converged in collision"
 * badge read them.
 */
export function chompStep(p: ChompProblem, xi: Vec[]): ChompStats {
  const { stats, grad } = chompEvaluate(p, xi);
  const dir = solveSmoothnessMetric(grad);
  for (let k = 0; k < xi.length; k++) for (let i = 0; i < xi[k].length; i++) xi[k][i] -= dir[k][i] / p.eta;
  return stats;
}

/** Run `iters` steps from ξ₀ (copied). Returns the final trajectory and the per-step statistics. */
export function chomp(p: ChompProblem, xi0: readonly Vec[], iters: number): { xi: Vec[]; history: ChompStats[] } {
  const xi = xi0.map((q) => q.slice());
  const history: ChompStats[] = [];
  for (let it = 0; it < iters; it++) history.push(chompStep(p, xi));
  history.push(chompEvaluate(p, xi).stats);
  return { xi, history };
}

/**
 * A sufficient step bound. Linearize the obstacle term as a Hessian of at most
 * λ/ε per waypoint (c″ = 1/ε inside the margin, ‖∇D‖ ≤ 1). The covariant
 * iteration ξ ← ξ − (1/η) A⁻¹(Aξ + b + λ∇F_obs) then has iteration matrix
 * I − (1/η)(I + A⁻¹H) with eigenvalues 1 − (1 + μ_i)/η, μ_i ∈ [0, (λ/ε)/λ_min(A)],
 * and is a contraction iff η > ½(1 + (λ/ε)/λ_min(A)), λ_min(A) = 2 − 2cos(π/(m+1)).
 * For one waypoint and ε = ½ this is (1 + λ)/2; for 20 waypoints it is ≈ 90:
 * A⁻¹ amplifies a low-frequency push by (m+1)²/π², which is why the step that
 * is right for a short path oscillates on a long one. The bound is worst-case —
 * a path with few waypoints inside the margin tolerates a smaller η.
 */
export function chompStableEta(m: number, lambdaObs: number, eps: number): number {
  const lamMin = 2 - 2 * Math.cos(Math.PI / (m + 1));
  return 0.5 * (1 + lambdaObs / eps / lamMin);
}

/** The straight line from q0 to qEnd with m free waypoints — CHOMP's default initialization. */
export function straightLine(q0: Vec, qEnd: Vec, m: number): Vec[] {
  const out: Vec[] = [];
  for (let k = 1; k <= m; k++) out.push(q0.map((v, i) => v + (k / (m + 1)) * (qEnd[i] - v)));
  return out;
}

/** Resample a polyline (with its ends) to m free interior waypoints, by arc length in the chart. */
export function resamplePolyline(pts: readonly Vec[], m: number): { q0: Vec; qEnd: Vec; xi: Vec[] } {
  const cum = [0];
  for (let k = 1; k < pts.length; k++) cum.push(cum[k - 1] + Math.hypot(...pts[k].map((v, i) => v - pts[k - 1][i])));
  const L = cum[cum.length - 1];
  const at = (s: number): Vec => {
    const target = s * L;
    let k = 0;
    while (k + 1 < cum.length - 1 && cum[k + 1] < target) k++;
    const seg = cum[k + 1] - cum[k] || 1;
    const f = Math.min(1, Math.max(0, (target - cum[k]) / seg));
    return pts[k].map((v, i) => v + f * (pts[k + 1][i] - v));
  };
  const xi: Vec[] = [];
  for (let k = 1; k <= m; k++) xi.push(at(k / (m + 1)));
  return { q0: pts[0].slice(), qEnd: pts[pts.length - 1].slice(), xi };
}
