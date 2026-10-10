/**
 * iLQR / DDP — the backward value recursion and the closed-loop forward pass
 * (Jacobson & Mayne 1970; Li & Todorov 2004; Tassa, Erez & Todorov 2012).
 *
 * Bellman's equation V_k(x) = min_u [ℓ(x, u) + V_{k+1}(f(x, u))], expanded to
 * second order about the nominal (x̄_k, ū_k):
 *
 *     Q_x  = ℓ_x  + f_xᵀ V′_x               Q_u  = ℓ_u  + f_uᵀ V′_x
 *     Q_xx = ℓ_xx + f_xᵀ V′_xx f_x  (+ V′_x · f_xx)      Q_uu = ℓ_uu + f_uᵀ V′_xx f_u  (+ V′_x · f_uu)
 *     Q_ux = ℓ_ux + f_uᵀ V′_xx f_x  (+ V′_x · f_ux)
 *
 * The bracketed tensor terms are DDP; dropping them is iLQR. Minimizing the
 * quadratic in δu gives the local policy δu = k + K δx with
 *     k = −(Q_uu + μI)⁻¹ Q_u,     K = −(Q_uu + μI)⁻¹ Q_ux,
 * and the value model recurses as V_x = Q_x − Kᵀ Q_uu k, V_xx = Q_xx − Kᵀ Q_uu K
 * (the symmetric forms with the cross terms, as in Tassa et al.). The forward
 * pass rolls the *policy* out — u_k = ū_k + α k_k + K_k (x_k − x̄_k) — with
 * backtracking on α against the expected decrease ΔV(α) = α a + α² b. That
 * closed-loop rollout is what distinguishes iLQR from gradient descent on the
 * control sequence, and why it survives a nonlinear model.
 *
 * On a linear model with quadratic cost one backward pass *is* the discrete
 * Riccati recursion and the α = 1 forward pass is optimal in one iteration;
 * `riccatiLqr` below computes the recursion independently so the check can
 * say so to 1e-9.
 */

import { matMul, matVec, solve, transpose, type Mat, type Vec } from '../prob/linalg';
import type { DiscreteDynamics } from './models';

export interface RunningDerivs {
  l: number;
  lx: Vec;
  lu: Vec;
  lxx: Mat;
  luu: Mat;
  /** ∂²ℓ/∂u∂x, nu × nx. */
  lux: Mat;
}
export interface TerminalDerivs {
  l: number;
  lx: Vec;
  lxx: Mat;
}

export interface StageCost {
  running(x: Vec, u: Vec, k: number): RunningDerivs;
  terminal(x: Vec): TerminalDerivs;
}

const zeros = (r: number, c: number): Mat => Array.from({ length: r }, () => new Array<number>(c).fill(0));
const addM = (a: Mat, b: Mat): Mat => a.map((row, i) => row.map((v, j) => v + b[i][j]));
const subM = (a: Mat, b: Mat): Mat => a.map((row, i) => row.map((v, j) => v - b[i][j]));
const symm = (a: Mat): Mat => a.map((row, i) => row.map((v, j) => 0.5 * (v + a[j][i])));

/**
 * The quadratic cost most of the chapter uses:
 *     ℓ = ½ (x − x_g)ᵀ Q (x − x_g) + ½ uᵀ R u,     ℓ_f = ½ (x − x_g)ᵀ Q_f (x − x_g),
 * plus an optional extra differentiable state cost (obstacles, for MPC) added
 * by value and finite-difference derivatives.
 */
export function quadraticCost(Q: Mat, R: Mat, Qf: Mat, xGoal: Vec, extra?: (x: Vec, k: number) => number, fdStep = 1e-4): StageCost {
  const nx = Q.length;
  const nu = R.length;
  const extraDerivs = (x: Vec, k: number): { l: number; lx: Vec; lxx: Mat } => {
    if (!extra) return { l: 0, lx: new Array<number>(nx).fill(0), lxx: zeros(nx, nx) };
    const l0 = extra(x, k);
    const lx = new Array<number>(nx).fill(0);
    const lxx = zeros(nx, nx);
    const h = fdStep;
    const fp: number[] = [];
    const fm: number[] = [];
    for (let i = 0; i < nx; i++) {
      const xp = x.slice();
      const xm = x.slice();
      xp[i] += h;
      xm[i] -= h;
      fp.push(extra(xp, k));
      fm.push(extra(xm, k));
      lx[i] = (fp[i] - fm[i]) / (2 * h);
      lxx[i][i] = (fp[i] - 2 * l0 + fm[i]) / (h * h);
    }
    for (let i = 0; i < nx; i++) {
      for (let j = i + 1; j < nx; j++) {
        const xpp = x.slice();
        xpp[i] += h;
        xpp[j] += h;
        const v = (extra(xpp, k) - fp[i] - fp[j] + l0) / (h * h);
        lxx[i][j] = v;
        lxx[j][i] = v;
      }
    }
    // Gauss–Newton-style guard: the finite-difference Hessian of a barrier can be indefinite; keep it PSD-ish by clipping the diagonal.
    for (let i = 0; i < nx; i++) lxx[i][i] = Math.max(lxx[i][i], 0);
    return { l: l0, lx, lxx };
  };
  return {
    running: (x, u, k) => {
      const dx = x.map((v, i) => v - xGoal[i]);
      const Qdx = matVec(Q, dx);
      const Ru = matVec(R, u);
      const e = extraDerivs(x, k);
      return {
        l: 0.5 * dx.reduce((s, v, i) => s + v * Qdx[i], 0) + 0.5 * u.reduce((s, v, i) => s + v * Ru[i], 0) + e.l,
        lx: Qdx.map((v, i) => v + e.lx[i]),
        lu: Ru,
        lxx: addM(Q, e.lxx),
        luu: R,
        lux: zeros(nu, nx),
      };
    },
    terminal: (x) => {
      const dx = x.map((v, i) => v - xGoal[i]);
      const Qdx = matVec(Qf, dx);
      return { l: 0.5 * dx.reduce((s, v, i) => s + v * Qdx[i], 0), lx: Qdx, lxx: Qf };
    },
  };
}

export const diagMat = (d: Vec): Mat => d.map((v, i) => d.map((_, j) => (i === j ? v : 0)));

export interface BackwardPass {
  k: Vec[];
  K: Mat[];
  /** Expected decrease ΔV(α) = α·a + α²·b. */
  expectedDecrease: [number, number];
  /** V_x and V_xx at every stage (for drawing the value ellipses). */
  Vx: Vec[];
  Vxx: Mat[];
  /** Q_uu at every stage (the widget highlights it). */
  Quu: Mat[];
  /** True when every regularized Q_uu was positive definite. */
  ok: boolean;
}

export interface IlqrOptions {
  maxIter?: number;
  /** Initial regularization μ added to Q_uu. */
  mu0?: number;
  muMin?: number;
  muMax?: number;
  /** Stop when the relative cost decrease is below this. */
  tol?: number;
  /** Keep the second-order dynamics terms (DDP) by finite differences of the Jacobians. */
  ddp?: boolean;
  alphas?: number[];
}

export interface IlqrResult {
  xs: Vec[];
  us: Vec[];
  K: Mat[];
  k: Vec[];
  /** Total cost after each iteration; costs[0] is the initial rollout. */
  costs: number[];
  iterations: number;
  converged: boolean;
  mu: number;
  /** α accepted at each iteration. */
  alphas: number[];
  Vxx: Mat[];
}

export function totalCost(cost: StageCost, xs: readonly Vec[], us: readonly Vec[]): number {
  let J = 0;
  for (let k = 0; k < us.length; k++) J += cost.running(xs[k], us[k], k).l;
  return J + cost.terminal(xs[xs.length - 1]).l;
}

/** Contract a Jacobian's derivative with V_x: (V_x · f_xx)_{ij} = Σ_a V_x[a] ∂²f_a/∂x_i∂x_j, by differencing f_x. */
function secondOrderTerms(f: DiscreteDynamics, x: Vec, u: Vec, Vx: Vec, h = 1e-5): { xx: Mat; uu: Mat; ux: Mat } {
  const nx = f.nx;
  const nu = f.nu;
  const xx = zeros(nx, nx);
  const uu = zeros(nu, nu);
  const ux = zeros(nu, nx);
  // d/dx_j of (f_xᵀ V_x) and (f_uᵀ V_x).
  for (let j = 0; j < nx; j++) {
    const xp = x.slice();
    const xm = x.slice();
    xp[j] += h;
    xm[j] -= h;
    const Jp = f.jacobians(xp, u);
    const Jm = f.jacobians(xm, u);
    for (let i = 0; i < nx; i++) {
      let s = 0;
      for (let a = 0; a < nx; a++) s += Vx[a] * (Jp.fx[a][i] - Jm.fx[a][i]);
      xx[i][j] = s / (2 * h);
    }
    for (let i = 0; i < nu; i++) {
      let s = 0;
      for (let a = 0; a < nx; a++) s += Vx[a] * (Jp.fu[a][i] - Jm.fu[a][i]);
      ux[i][j] = s / (2 * h);
    }
  }
  for (let j = 0; j < nu; j++) {
    const up = u.slice();
    const um = u.slice();
    up[j] += h;
    um[j] -= h;
    const Jp = f.jacobians(x, up);
    const Jm = f.jacobians(x, um);
    for (let i = 0; i < nu; i++) {
      let s = 0;
      for (let a = 0; a < nx; a++) s += Vx[a] * (Jp.fu[a][i] - Jm.fu[a][i]);
      uu[i][j] = s / (2 * h);
    }
  }
  return { xx: symm(xx), uu: symm(uu), ux };
}

/** The backward pass along a nominal (xs, us) with regularization μ on Q_uu. */
export function backwardPass(f: DiscreteDynamics, cost: StageCost, xs: readonly Vec[], us: readonly Vec[], mu: number, ddp = false): BackwardPass {
  const N = us.length;
  const nu = f.nu;
  const term = cost.terminal(xs[N]);
  let Vx = term.lx.slice();
  let Vxx = term.lxx.map((r) => r.slice());
  const ks: Vec[] = new Array(N);
  const Ks: Mat[] = new Array(N);
  const Vxs: Vec[] = new Array(N + 1);
  const Vxxs: Mat[] = new Array(N + 1);
  const Quus: Mat[] = new Array(N);
  Vxs[N] = Vx;
  Vxxs[N] = Vxx;
  let a = 0;
  let b = 0;
  let ok = true;
  for (let k = N - 1; k >= 0; k--) {
    const { fx, fu } = f.jacobians(xs[k], us[k]);
    const l = cost.running(xs[k], us[k], k);
    const fxT = transpose(fx);
    const fuT = transpose(fu);
    const fxTV = matMul(fxT, Vxx);
    const fuTV = matMul(fuT, Vxx);
    const Qx = l.lx.map((v, i) => v + matVec(fxT, Vx)[i]);
    const Qu = l.lu.map((v, i) => v + matVec(fuT, Vx)[i]);
    let Qxx = addM(l.lxx, matMul(fxTV, fx));
    let Quu = addM(l.luu, matMul(fuTV, fu));
    let Qux = addM(l.lux, matMul(fuTV, fx));
    if (ddp) {
      const so = secondOrderTerms(f, xs[k], us[k], Vx);
      Qxx = addM(Qxx, so.xx);
      Quu = addM(Quu, so.uu);
      Qux = addM(Qux, so.ux);
    }
    const QuuReg = Quu.map((row, i) => row.map((v, j) => (i === j ? v + mu : v)));
    // Positive definiteness by attempted Cholesky pivots.
    if (!isPD(QuuReg)) ok = false;
    const kk = solve(QuuReg, Qu.map((v) => -v));
    const KK: Mat = [];
    const QuxT = transpose(Qux);
    for (let i = 0; i < nu; i++) KK.push(new Array<number>(f.nx).fill(0));
    for (let j = 0; j < f.nx; j++) {
      const col = solve(QuuReg, QuxT[j].map((v) => -v));
      for (let i = 0; i < nu; i++) KK[i][j] = col[i];
    }
    // ΔV(α) = α kᵀQ_u + ½α² kᵀQ_uu k
    a += kk.reduce((s, v, i) => s + v * Qu[i], 0);
    const Quuk = matVec(Quu, kk);
    b += 0.5 * kk.reduce((s, v, i) => s + v * Quuk[i], 0);
    // Value update (Tassa et al. 2012 eq. 11, with the cross terms).
    const KT = transpose(KK);
    const KTQuu = matMul(KT, Quu);
    const Vx1 = Qx.map((v, i) => v + matVec(KT, Qu)[i] + matVec(KTQuu, kk)[i] + matVec(transpose(Qux), kk)[i]);
    const Vxx1 = symm(addM(addM(Qxx, matMul(KTQuu, KK)), addM(matMul(KT, Qux), matMul(transpose(Qux), KK))));
    Vx = Vx1;
    Vxx = Vxx1;
    ks[k] = kk;
    Ks[k] = KK;
    Vxs[k] = Vx;
    Vxxs[k] = Vxx;
    Quus[k] = Quu;
  }
  return { k: ks, K: Ks, expectedDecrease: [a, b], Vx: Vxs, Vxx: Vxxs, Quu: Quus, ok };
}

function isPD(M: Mat): boolean {
  const n = M.length;
  const L = zeros(n, n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let s = M[i][j];
      for (let k = 0; k < j; k++) s -= L[i][k] * L[j][k];
      if (i === j) {
        if (s <= 1e-14) return false;
        L[i][i] = Math.sqrt(s);
      } else L[i][j] = s / L[j][j];
    }
  }
  return true;
}

/** Closed-loop rollout u_k = ū_k + α k_k + K_k (x_k − x̄_k). */
export function forwardPass(f: DiscreteDynamics, cost: StageCost, xs: readonly Vec[], us: readonly Vec[], bp: BackwardPass, alpha: number): { xs: Vec[]; us: Vec[]; cost: number } {
  const N = us.length;
  const xsNew: Vec[] = [xs[0].slice()];
  const usNew: Vec[] = [];
  for (let k = 0; k < N; k++) {
    const dx = xsNew[k].map((v, i) => v - xs[k][i]);
    const Kdx = matVec(bp.K[k], dx);
    const u = us[k].map((v, i) => v + alpha * bp.k[k][i] + Kdx[i]);
    usNew.push(u);
    xsNew.push(f.step(xsNew[k], u));
  }
  return { xs: xsNew, us: usNew, cost: totalCost(cost, xsNew, usNew) };
}

/** iLQR (DDP with `ddp: true`): regularized backward pass, line-searched closed-loop forward pass, repeat. */
export function ilqr(f: DiscreteDynamics, cost: StageCost, x0: Vec, uInit: readonly Vec[], opts: IlqrOptions = {}): IlqrResult {
  const maxIter = opts.maxIter ?? 50;
  const tol = opts.tol ?? 1e-9;
  const muMin = opts.muMin ?? 1e-8;
  const muMax = opts.muMax ?? 1e8;
  let mu = opts.mu0 ?? 1e-6;
  const alphas = opts.alphas ?? [1, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625];
  let us = uInit.map((u) => u.slice());
  let xs: Vec[] = [x0.slice()];
  for (const u of us) xs.push(f.step(xs[xs.length - 1], u));
  let J = totalCost(cost, xs, us);
  const costs = [J];
  const usedAlphas: number[] = [];
  let K: Mat[] = [];
  let kff: Vec[] = [];
  let Vxx: Mat[] = [];
  let converged = false;
  let it = 0;
  for (; it < maxIter; it++) {
    let bp = backwardPass(f, cost, xs, us, mu, opts.ddp ?? false);
    while (!bp.ok && mu < muMax) {
      mu = Math.max(mu * 10, 1e-6);
      bp = backwardPass(f, cost, xs, us, mu, opts.ddp ?? false);
    }
    K = bp.K;
    kff = bp.k;
    Vxx = bp.Vxx;
    let accepted = false;
    for (const alpha of alphas) {
      const trial = forwardPass(f, cost, xs, us, bp, alpha);
      const expected = -(alpha * bp.expectedDecrease[0] + alpha * alpha * bp.expectedDecrease[1]);
      const actual = J - trial.cost;
      // Accept a step that realizes a reasonable fraction of the model's promise (Tassa et al.), or any descent when the model is uninformative.
      if (actual > 0 && (expected <= 1e-12 || actual / expected > 1e-4)) {
        xs = trial.xs;
        us = trial.us;
        const rel = actual / Math.max(Math.abs(J), 1e-300);
        J = trial.cost;
        costs.push(J);
        usedAlphas.push(alpha);
        mu = Math.max(mu * 0.1, muMin);
        accepted = true;
        if (rel < tol) converged = true;
        break;
      }
    }
    if (!accepted) {
      mu *= 10;
      if (mu > muMax) break;
      continue;
    }
    if (converged) {
      it++;
      break;
    }
  }
  return { xs, us, K, k: kff, costs, iterations: it, converged, mu, alphas: usedAlphas, Vxx };
}

/**
 * The discrete-time finite-horizon LQR Riccati recursion for x⁺ = A x + B u,
 * cost Σ ½(xᵀQx + uᵀRu) + ½ x_Nᵀ Q_f x_N (about the origin):
 *     K_k = −(R + Bᵀ P_{k+1} B)⁻¹ Bᵀ P_{k+1} A,
 *     P_k = Q + Aᵀ P_{k+1} A + Aᵀ P_{k+1} B K_k.
 * Computed independently of the iLQR code so the two can be compared.
 */
export function riccatiLqr(A: Mat, B: Mat, Q: Mat, R: Mat, Qf: Mat, N: number): { K: Mat[]; P: Mat[] } {
  const Ks: Mat[] = new Array(N);
  const Ps: Mat[] = new Array(N + 1);
  let P = Qf;
  Ps[N] = P;
  const At = transpose(A);
  const Bt = transpose(B);
  for (let k = N - 1; k >= 0; k--) {
    const BtP = matMul(Bt, P);
    const S = addM(R, matMul(BtP, B));
    const BtPA = matMul(BtP, A);
    const Kk: Mat = [];
    const cols = transpose(BtPA).map((c) => solve(S, c.map((v) => -v)));
    for (let i = 0; i < R.length; i++) Kk.push(cols.map((c) => c[i]));
    P = symm(addM(addM(Q, matMul(matMul(At, P), A)), matMul(matMul(matMul(At, P), B), Kk)));
    Ks[k] = Kk;
    Ps[k] = P;
  }
  return { K: Ks, P: Ps };
}

export { subM };
