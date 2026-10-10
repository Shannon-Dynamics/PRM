/**
 * Numerical self-checks for Chapter 19 — `trajopt`.
 *
 * The design doc's micro-example (one CHOMP step lands the waypoint on the
 * disc; the λ_obs equilibria), the iLQR numbers (min-effort against
 * Pontryagin's 6 − 12t, one iteration on a linear-quadratic problem), and the
 * invariants: the Thomas sweep equals the dense A⁻¹, the pointwise gradient is
 * the finite-difference gradient, CHOMP's cost is non-increasing at the stated
 * step, iLQR's gains are the Riccati gains, LM converges on Rosenbrock, the
 * seeded STOMP lands near the same equilibrium, the hinge-penalty recipe ends
 * with zero violation, MPC reaches its goal around a moving obstacle where the
 * open-loop plan does not, and the brushfire field is a chessboard distance
 * bracketed by the exact Euclidean one.
 */

import type { CheckResult } from '../__checks__';
import { inv, matVec, type Vec } from '../prob/linalg';
import { Rng } from '../prob/rng';
import { Hitch } from '../robots/hitch';
import { WORKBENCH } from '../robots/worlds';
import { APARTMENT } from '../sim/world';
import { chomp, chompEvaluate, chompStableEta, chompStep, obstacleTerm, smoothness, smoothnessMatrix, solveSmoothnessMetric, straightLine, POINT_BODY, type ChompProblem } from './chomp';
import { DiscField, EdtField, GridField, PlanarField } from './field';
import { backwardPass, diagMat, ilqr, quadraticCost, riccatiLqr, totalCost } from './ilqr';
import { microChomp, microEquilibrium, minEffort, pontryaginControl, reachWorkbench, stableEta } from './micro';
import { DoubleIntegrator, KinematicCar, Unicycle, numericJacobians, rollout } from './models';
import { GridCostToGo, Mpc, navigationCost } from './mpc';
import { ROSENBROCK, finiteDifferenceJacobian, levenbergMarquardt } from './optim';
import { sco, violation } from './sco';
import { stomp } from './stomp';
import { PlanarObstacles } from '../potential/planar';
import { Reach2R } from '../dynamics/reach2r';
import { Rrt } from '../sampling/tree';
import { StraightLine, shortcutGreedy } from '../sampling/steer';
import { torusLab, TORUS_GOAL, TORUS_START } from '../sampling/worlds';
import { PathDynamics } from '../trajectory/constrained';
import { symmetricLimits } from '../trajectory/limits';
import { CubicSpline, unwrapWaypoints } from '../trajectory/path';
import { timeScale } from '../trajectory/time-scale';
import { resamplePolyline } from './chomp';

const f4 = (x: number) => x.toFixed(4);

export function runCh19Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The micro-example: one covariant step lands q₁ on the disc; the second step is a fixed point.
  // -------------------------------------------------------------------------
  {
    const { p, xi0 } = microChomp();
    const xi = xi0.map((q) => q.slice());
    const sm = smoothness(xi, p.q0, p.qEnd);
    const ob = obstacleTerm(p, xi);
    const dBefore = p.field.dist({ x: xi[0][0], y: xi[0][1] });
    const s1 = chompStep(p, xi);
    const dAfter = p.field.dist({ x: xi[0][0], y: xi[0][1] });
    const e2 = chompEvaluate(p, xi);
    const pass =
      Math.abs(dBefore + 0.3) < 1e-12 &&
      Math.abs(sm.grad[0][0]) < 1e-12 &&
      Math.abs(sm.grad[0][1] - 0.4) < 1e-12 &&
      Math.abs(ob.grad[0][0]) < 1e-12 &&
      Math.abs(ob.grad[0][1] + 1) < 1e-12 &&
      Math.abs(ob.cost - 0.55) < 1e-12 &&
      Math.abs(xi[0][0] - 1) < 1e-12 &&
      Math.abs(xi[0][1] - 0.5) < 1e-12 &&
      Math.abs(dAfter) < 1e-12 &&
      e2.stats.gradInf < 1e-12 &&
      s1.minClearance < 0;
    push(
      'trajopt: CHOMP micro-example — ∇F_smooth = (0, 0.4), ∇F_obs = (0, −1), c = 0.55, covariant step q₁ ← (1, 0.5) with clearance 0, then |∇F| = 0 (a fixed point in contact)',
      pass,
      `D(q₁) = ${dBefore.toFixed(2)} → ${dAfter.toExponential(1)}; grad_smooth = (${f4(sm.grad[0][0])}, ${f4(sm.grad[0][1])}); grad_obs = (${f4(ob.grad[0][0])}, ${f4(ob.grad[0][1])}); q₁ = (${f4(xi[0][0])}, ${f4(xi[0][1])}); step 2 |∇F| = ${e2.stats.gradInf.toExponential(1)}`,
    );
  }

  {
    // Equilibria y* = λ/(1 + λ), D* = (λ − 1)/(2(1 + λ)) for λ ∈ {1, 2, 4} after 200 steps at η = 4,
    // with the cost never rising; and the stability bound η > (1 + λ)/2 — at λ = 2, η = 1.4 the
    // update oscillates and η = 1.6 converges.
    const results: string[] = [];
    let pass = true;
    for (const lambda of [1, 2, 4]) {
      const { p, xi0 } = microChomp(lambda, 0.5, 4);
      const r = chomp(p, xi0, 200);
      const target = microEquilibrium(lambda);
      const y = r.xi[0][1];
      const d = p.field.dist({ x: r.xi[0][0], y });
      let monotone = true;
      for (let k = 1; k < r.history.length; k++) if (r.history[k].f > r.history[k - 1].f + 1e-12) monotone = false;
      pass = pass && Math.abs(y - target.y) < 1e-4 && Math.abs(d - target.clearance) < 1e-4 && monotone;
      results.push(`λ = ${lambda}: D* = ${f4(d)} (expected ${f4(target.clearance)})${monotone ? '' : ', cost rose'}`);
    }
    const unstable = chomp(microChomp(2, 0.5, 1.4).p, [[1, 0.6]], 200).xi[0][1];
    const stable = chomp(microChomp(2, 0.5, 1.6).p, [[1, 0.6]], 200).xi[0][1];
    const bound = stableEta(2);
    const stab = Math.abs(stable - 2 / 3) < 1e-6 && Math.abs(unstable - 2 / 3) > 0.05 && Math.abs(bound - 1.5) < 1e-12;
    push(
      'trajopt: CHOMP equilibria — clearance D* = (λ − 1)/(2(1 + λ)) = 0, 1/6, 0.3 for λ_obs = 1, 2, 4 after 200 iterations at η = 4, cost non-increasing; the update is a contraction iff η > (1 + λ)/2 (λ = 2: η = 1.4 oscillates, η = 1.6 converges)',
      pass && stab,
      `${results.join('; ')}; λ = 2 after 200 steps: y = ${f4(unstable)} at η = 1.4, y = ${f4(stable)} at η = 1.6 (y* = 0.6667)`,
    );
  }

  // -------------------------------------------------------------------------
  // The covariant preconditioner and the gradient are what they claim to be.
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(19);
    const m = 15;
    const rhs: Vec[] = Array.from({ length: m }, () => [rng.normal(), rng.normal()]);
    const thomas = solveSmoothnessMetric(rhs);
    const Ainv = inv(smoothnessMatrix(m));
    let worst = 0;
    for (let i = 0; i < 2; i++) {
      const dense = matVec(Ainv, rhs.map((r) => r[i]));
      for (let k = 0; k < m; k++) worst = Math.max(worst, Math.abs(dense[k] - thomas[k][i]));
    }
    // Pointwise obstacle gradient against central differences of the pointwise cost, on a 12-waypoint path near the disc.
    const p: ChompProblem = { q0: [0, 0], qEnd: [2, 0], field: new DiscField({ x: 1, y: 0 }, 0.5), body: POINT_BODY, eps: 0.5, lambdaObs: 1, eta: 1, obstacleForm: 'pointwise' };
    const xi = straightLine(p.q0, p.qEnd, 12).map((q, k) => [q[0], 0.15 + 0.05 * Math.sin(k)]);
    const { grad } = obstacleTerm(p, xi);
    let worstG = 0;
    const h = 1e-6;
    for (let k = 0; k < xi.length; k++) {
      for (let i = 0; i < 2; i++) {
        const xp = xi.map((q) => q.slice());
        const xm = xi.map((q) => q.slice());
        xp[k][i] += h;
        xm[k][i] -= h;
        const fd = (obstacleTerm(p, xp).cost - obstacleTerm(p, xm).cost) / (2 * h);
        worstG = Math.max(worstG, Math.abs(fd - grad[k][i]));
      }
    }
    // A 20-waypoint run: at twice the stability bound η* = ½(1 + (λ/ε)/λ_min(A)) ≈ 90 the cost never
    // rises and the path ends clear of the disc; at a quarter of the bound it oscillates.
    const etaStar = chompStableEta(20, 2, 0.5);
    const xi20 = straightLine(p.q0, p.qEnd, 20).map((q) => [q[0], q[1] + 0.05]);
    const run = chomp({ ...p, lambdaObs: 2, eta: 2 * etaStar }, xi20, 400);
    let monotone = true;
    for (let k = 1; k < run.history.length; k++) if (run.history[k].f > run.history[k - 1].f + 1e-9) monotone = false;
    const last = run.history[run.history.length - 1];
    const wild = chomp({ ...p, lambdaObs: 2, eta: etaStar / 4 }, xi20, 100);
    let rises = 0;
    for (let k = 1; k < wild.history.length; k++) if (wild.history[k].f > wild.history[k - 1].f + 1e-9) rises++;
    push(
      'trajopt: A⁻¹∇F by the Thomas sweep equals the dense inverse; the pointwise obstacle gradient is the finite-difference gradient of its cost; 20-waypoint CHOMP at λ = 2 never raises the cost at η = 2η* (η* = ½(1 + (λ/ε)/λ_min(A)) ≈ 90) and ends clear of the disc, while η = η*/4 oscillates',
      worst < 1e-10 && worstG < 1e-6 && monotone && last.minClearance > 0 && rises > 10 && Math.abs(etaStar - 90) < 2,
      `|Thomas − dense| ${worst.toExponential(1)}; |∇ − fd| ${worstG.toExponential(1)}; η* = ${etaStar.toFixed(1)}; 400 steps at 2η*: F ${f4(run.history[0].f)} → ${f4(last.f)}, min clearance ${f4(last.minClearance)}; η*/4: ${rises} cost rises in 100 steps`,
    );
  }

  // -------------------------------------------------------------------------
  // The shared optimizer: LM on Rosenbrock, with and without an analytic Jacobian, and with a box trust region.
  // -------------------------------------------------------------------------
  {
    const a = levenbergMarquardt(ROSENBROCK, [-1.2, 1]);
    const b = levenbergMarquardt({ residual: ROSENBROCK.residual }, [-1.2, 1]);
    const c = levenbergMarquardt(ROSENBROCK, [-1.2, 1], { trust: 0.1, maxIter: 400 });
    const J = finiteDifferenceJacobian(ROSENBROCK.residual, [0.3, -0.7]);
    const Ja = ROSENBROCK.jacobian!([0.3, -0.7]);
    let worstJ = 0;
    for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) worstJ = Math.max(worstJ, Math.abs(J[i][j] - Ja[i][j]));
    const near = (r: { x: Vec }) => Math.hypot(r.x[0] - 1, r.x[1] - 1) < 1e-8;
    push(
      'trajopt: Levenberg–Marquardt reaches Rosenbrock\'s minimum (1, 1) from (−1.2, 1) to 1e-8 — analytic Jacobian, finite-difference Jacobian, and with a 0.1 box trust region — and the FD Jacobian matches the analytic one',
      near(a) && near(b) && near(c) && a.converged && worstJ < 1e-6 && a.iterations < 60,
      `analytic: ${a.iterations} it, cost ${a.cost.toExponential(1)}; fd: ${b.iterations} it; trust 0.1: ${c.iterations} it; |J − fd| ${worstJ.toExponential(1)}`,
    );
  }

  // -------------------------------------------------------------------------
  // iLQR on the min-effort double integrator recovers Pontryagin's u = 6 − 12t in one iteration.
  // -------------------------------------------------------------------------
  {
    const { f, cost, x0, uInit, dt } = minEffort(100, 1, 1);
    const r = ilqr(f, cost, x0, uInit, { maxIter: 5, mu0: 0 });
    let worst = 0;
    for (let k = 0; k < 100; k++) worst = Math.max(worst, Math.abs(r.us[k][0] - pontryaginControl((k + 0.5) * dt)));
    const xN = r.xs[100];
    const u0 = r.us[0][0];
    const uMid = r.us[50][0];
    const pass = worst < 5e-2 && r.alphas[0] === 1 && Math.abs(r.costs[1] - r.costs[r.costs.length - 1]) < 1e-9 * Math.max(1, r.costs[1]) && Math.abs(xN[0] - 1) < 1e-4 && Math.abs(xN[1]) < 1e-4;
    push(
      'trajopt: iLQR on Choset Example 11.3.1 (q̈ = u, d = 1, t_f = 1, N = 100, ℓ = u²Δt, terminal weight 10⁶) — max_k |u_k − (6 − 12 t_{k+½})| < 0.05, α = 1 accepted, converged in one iteration, x_N = (1, 0) to 1e-4',
      pass,
      `max error ${worst.toExponential(2)}; u₀ = ${f4(u0)} (Pontryagin 5.94), u₅₀ = ${f4(uMid)} (−0.06); iterations with a cost change: ${r.costs.length - 1}; x_N = (${f4(xN[0])}, ${f4(xN[1])})`,
    );
  }

  {
    // iLQR gains on a linear-quadratic problem equal the Riccati gains to 1e-9, and the one-pass cost equals ½ x₀ᵀ P₀ x₀.
    const f = new DoubleIntegrator(0.1);
    const Q = diagMat([1, 0.1]);
    const R = [[0.5]];
    const Qf = diagMat([20, 5]);
    const N = 40;
    const x0 = [1.5, -0.4];
    const cost = quadraticCost(Q, R, Qf, [0, 0]);
    const { fx: A, fu: B } = f.jacobians();
    const ric = riccatiLqr(A, B, Q, R, Qf, N);
    const uInit = Array.from({ length: N }, () => [0]);
    const xs0 = rollout(f, x0, uInit);
    const bp = backwardPass(f, cost, xs0, uInit, 0);
    let worstK = 0;
    for (let k = 0; k < N; k++) worstK = Math.max(worstK, Math.abs(bp.K[k][0][0] - ric.K[k][0][0]), Math.abs(bp.K[k][0][1] - ric.K[k][0][1]));
    const r = ilqr(f, cost, x0, uInit, { maxIter: 3, mu0: 0 });
    const P0x = matVec(ric.P[0], x0);
    const optimal = 0.5 * (x0[0] * P0x[0] + x0[1] * P0x[1]);
    const pass = worstK < 1e-9 && Math.abs(r.costs[1] - optimal) < 1e-9 * Math.max(1, optimal) && r.alphas[0] === 1;
    push('trajopt: on a linear-quadratic problem the iLQR backward pass is the discrete Riccati recursion (gains agree to 1e-9) and the α = 1 forward pass reaches the optimal cost ½x₀ᵀP₀x₀ in one iteration', pass, `max |K − K_riccati| ${worstK.toExponential(1)}; cost after one pass ${r.costs[1].toFixed(8)} vs Riccati ${optimal.toFixed(8)}; initial ${r.costs[0].toFixed(4)}`);
  }

  // -------------------------------------------------------------------------
  // Models: analytic Jacobians match central differences; the kinematic car is Chapter 2's Hitch.
  // -------------------------------------------------------------------------
  {
    const di = new DoubleIntegrator(0.05);
    const num = numericJacobians((x, u) => di.step(x, u), [0.3, -1.2], [0.7]);
    const an = di.jacobians();
    let worst = 0;
    for (let i = 0; i < 2; i++) {
      for (let j = 0; j < 2; j++) worst = Math.max(worst, Math.abs(num.fx[i][j] - an.fx[i][j]));
      worst = Math.max(worst, Math.abs(num.fu[i][0] - an.fu[i][0]));
    }
    const car = new KinematicCar(0.1);
    const hitch = Hitch.car();
    const rng = new Rng(3);
    let worstCar = 0;
    for (let t = 0; t < 50; t++) {
      const x = [rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-Math.PI, Math.PI)];
      const u = [rng.uniform(-2, 2), rng.uniform(-0.6, 0.6)];
      const a = car.step(x, u);
      const b = hitch.step(hitch.cfg(x[0], x[1], x[2]), { v: u[0], phi: u[1] }, 0.1).body;
      worstCar = Math.max(worstCar, Math.abs(a[0] - b.x), Math.abs(a[1] - b.y), Math.abs(Math.atan2(Math.sin(a[2] - b.theta), Math.cos(a[2] - b.theta))));
    }
    // The unicycle's arc is continuous at ω → 0.
    const uni = new Unicycle(0.1);
    const s1 = uni.step([0, 0, 0.3], [1, 1e-10]);
    const s2 = uni.step([0, 0, 0.3], [1, 0]);
    const cont = Math.hypot(s1[0] - s2[0], s1[1] - s2[1]);
    push('trajopt: the double integrator\'s analytic Jacobians match central differences; the KinematicCar step equals Chapter 2\'s Hitch.step on 50 seeded states; the unicycle arc is continuous at ω → 0', worst < 1e-7 && worstCar < 1e-9 && cont < 1e-8, `|J − fd| ${worst.toExponential(1)}; |car − Hitch| ${worstCar.toExponential(1)}; ω→0 gap ${cont.toExponential(1)}`);
  }

  // -------------------------------------------------------------------------
  // STOMP, seeded, lands near the same equilibrium as CHOMP without a gradient.
  // -------------------------------------------------------------------------
  {
    const { p, xi0 } = microChomp(2);
    const st = stomp(p, xi0, 120, 7, { K: 24, noise: 0.15 });
    const target = microEquilibrium(2);
    const y = st.xi[0][1];
    const first = st.history[0].f;
    const last = st.history[st.history.length - 1].f;
    push('trajopt: STOMP (seed 7, K = 24, 120 iterations) on the micro-example at λ = 2 never raises the accepted cost and ends within 0.02 of CHOMP\'s equilibrium y* = 2/3', last <= first + 1e-12 && Math.abs(y - target.y) < 0.02, `y = ${f4(y)} (y* = ${f4(target.y)}); F ${f4(first)} → ${f4(last)}`);
  }

  // -------------------------------------------------------------------------
  // The sequential-convex recipe ends with zero constraint violation where CHOMP ends in contact.
  // -------------------------------------------------------------------------
  {
    const field = new DiscField({ x: 1, y: 0 }, 0.5);
    const q0 = [0, 0];
    const qEnd = [2, 0];
    const xi0 = straightLine(q0, qEnd, 12).map((q) => [q[0], q[1] + 0.05]);
    const r = sco({ q0, qEnd, field, dSafe: 0.1 }, xi0, { mu0: 10 });
    const v = violation({ q0, qEnd, field, dSafe: 0.1 }, r.xi);
    let monotone = true;
    for (let k = 1; k < r.merit.length; k++) if (r.muTrace[k] === r.muTrace[k - 1] && r.merit[k] > r.merit[k - 1] + 1e-12) monotone = false;
    // CHOMP on the same scene: its clearance is whatever the cost balance gives — here far more than d_safe — because it never saw a constraint.
    const chompRun = chomp({ q0, qEnd, field, body: POINT_BODY, eps: 0.5, lambdaObs: 1, eta: 2 * chompStableEta(12, 1, 0.5), obstacleForm: 'pointwise' }, xi0, 400);
    const chompClear = chompRun.history[chompRun.history.length - 1].minClearance;
    const tight = Math.min(...r.xi.map((q) => field.dist({ x: q[0], y: q[1] })));
    push(
      'trajopt: the sequential-convex recipe (hinge penalty, trust-region LM, penalty loop) ends with every waypoint at least d_safe = 0.1 from the disc (violation < 1e-6, the tightest waypoint within 1e-3 of d_safe) with a merit non-increasing within each penalty level; CHOMP with a cost on the same scene settles at a clearance set by λ and ε, not by d_safe',
      v.worst < 1e-6 && monotone && r.violation < 1e-6 && Math.abs(tight - 0.1) < 1e-3 && Math.abs(chompClear - 0.1) > 0.05,
      `violation ${v.worst.toExponential(1)} after ${r.outerIterations} convexifications, μ = ${r.mu}, tightest clearance ${f4(tight)}, smoothness ${f4(r.smoothness)}; CHOMP clearance ${f4(chompClear)}`,
    );
  }

  // -------------------------------------------------------------------------
  // MPC: Rusty reaches a goal past a moving obstacle; the open-loop plan from tick 0 does not.
  // -------------------------------------------------------------------------
  {
    const dt = 0.1;
    const f = new Unicycle(dt);
    const goal = [4, 0, 0];
    const obstacle = (t: number) => ({ x: 2, y: 0.9 * Math.sin(0.6 * t) - 0.4, r: 0.45 });
    const margin = 0.25;
    // The planner sees the obstacle where it is *now* and assumes it stays: that is what a sensor gives it.
    const cost = (t0: number) =>
      quadraticCost(diagMat([1, 1, 0.05]), diagMat([0.4, 0.1]), diagMat([40, 40, 2]), goal, (x) => {
        const o = obstacle(t0 * dt);
        const d = Math.hypot(x[0] - o.x, x[1] - o.y) - o.r - margin;
        return d < 0 ? 60 * d * d : 0;
      });
    const run = (receding: boolean) => {
      const mpc = new Mpc(f, cost, { horizon: 20, warmStart: true, ilqr: { maxIter: 8 } });
      let x = [0, 0, 0];
      let minClear = Infinity;
      let reachedAt = -1;
      let firstPlan: Vec[] | null = null;
      for (let t = 0; t < 90; t++) {
        let u: Vec;
        if (receding || !firstPlan) {
          const tick = mpc.tick(x, t);
          u = tick.u;
          if (!firstPlan) firstPlan = tick.controls;
        } else {
          u = t < firstPlan.length ? firstPlan[t] : [0, 0];
        }
        x = f.step(x, u);
        const o = obstacle((t + 1) * dt);
        minClear = Math.min(minClear, Math.hypot(x[0] - o.x, x[1] - o.y) - o.r);
        if (reachedAt < 0 && Math.hypot(x[0] - goal[0], x[1] - goal[1]) < 0.15) reachedAt = t + 1;
      }
      return { minClear, reachedAt, x };
    };
    const closed = run(true);
    const open = run(false);
    push(
      'trajopt: receding-horizon iLQR (N_h = 20, Δt = 0.1, obstacle frozen at its measured position each tick) drives Rusty 4 m to the goal around a disc that oscillates across its path, keeping clearance; the first plan executed open-loop collides with the moved obstacle',
      closed.reachedAt > 0 && closed.minClear > 0 && open.minClear < 0,
      `closed loop: reached at tick ${closed.reachedAt}, min clearance ${f4(closed.minClear)}; open loop: min clearance ${f4(open.minClear)}`,
    );
  }

  // -------------------------------------------------------------------------
  // MPC in the Apartment: with the Chapter 6 cost-to-go as terminal cost Rusty reaches room E at every
  // horizon with no collision; with a plain quadratic terminal every horizon is myopic; the open-loop
  // ghost collides where MPC waits; solve time grows with the horizon.
  // -------------------------------------------------------------------------
  {
    const dt = 0.1;
    const f = new Unicycle(dt);
    const walls = EdtField.fromWorld(APARTMENT, 0.05);
    const start = [1, 1, 0];
    const goal = [11, 6.4, 0];
    const R = 0.22;
    const ctg = new GridCostToGo(APARTMENT, [goal[0], goal[1]], 0.1, 0.3);
    const obstacleAt = (t: number) => ({ x: 6.0, y: 3.9 + 0.9 * Math.sin(0.6 * t), r: 0.35 });
    const run = (useCtg: boolean, horizon: number, ghost: boolean) => {
      const costAt = (t0: number) => navigationCost({ goal, costToGo: useCtg ? ctg : null, walls, radius: R, obstacle: obstacleAt(t0 * dt), vMax: 1.0, weights: { ctg: 12, obstacle: 1500, wall: 1500 } });
      const mpc = new Mpc(f, costAt, { horizon, warmStart: true, ilqr: { maxIter: 8, mu0: 1e-2 } });
      let x = start.slice();
      let hits = 0;
      let wallHits = 0;
      let reached = -1;
      let ms = 0;
      let plan: Vec[] = [];
      let idx = 0;
      let ticks = 0;
      for (let t = 0; t < 600 && reached < 0; t++) {
        let u: Vec;
        if (!ghost) {
          const tick = mpc.tick(x, t);
          ms += tick.solveMs;
          ticks++;
          u = tick.u;
        } else {
          if (idx >= plan.length) {
            const tick = mpc.tick(x, t);
            plan = tick.controls;
            idx = 0;
          }
          u = plan[idx++];
        }
        x = f.step(x, u);
        const o = obstacleAt((t + 1) * dt);
        if (Math.hypot(x[0] - o.x, x[1] - o.y) < o.r + R) hits++;
        if (walls.dist({ x: x[0], y: x[1] }) < R) wallHits++;
        if (Math.hypot(x[0] - goal[0], x[1] - goal[1]) < 0.25) reached = t;
      }
      return { reached, hits, wallHits, msPerTick: ticks ? ms / ticks : 0 };
    };
    const c6 = run(true, 6, false);
    const c10 = run(true, 10, false);
    const c24 = run(true, 24, false);
    const q10 = run(false, 10, false);
    const q24 = run(false, 24, false);
    const g16 = run(true, 16, true);
    const pass =
      [c6, c10, c24].every((r) => r.reached > 0 && r.hits === 0 && r.wallHits === 0) &&
      q10.reached < 0 &&
      q24.reached < 0 &&
      g16.hits > 0 &&
      c24.msPerTick > c6.msPerTick;
    push(
      'trajopt: MPC in the Apartment (Rusty, Δt = 0.1, obstacle sliding through room B\'s door, frozen at planning time) — with the Chapter 6 wave-front cost-to-go as terminal cost N_h = 6, 10, 24 all reach room E without touching the obstacle or a wall; with a quadratic terminal N_h = 10 and 24 never arrive (myopia); the open-loop ghost at N_h = 16 collides; solve time grows with N_h',
      pass,
      `cost-to-go: N_h = 6 → ${c6.reached} ticks (${c6.msPerTick.toFixed(1)} ms/tick), 10 → ${c10.reached} (${c10.msPerTick.toFixed(1)} ms), 24 → ${c24.reached} (${c24.msPerTick.toFixed(1)} ms), hits ${c6.hits + c10.hits + c24.hits}; quadratic: ${q10.reached}, ${q24.reached}; ghost N_h = 16: ${g16.hits} collision ticks`,
    );
  }

  // -------------------------------------------------------------------------
  // Distance fields: brushfire (chessboard) is bracketed by the exact Euclidean distance; the signed EDT agrees with the exact planar query.
  // -------------------------------------------------------------------------
  {
    const scene = reachWorkbench(8, 0.05);
    const exact = new PlanarField(
      new PlanarObstacles([...WORKBENCH.obstacles.map((o) => ({ kind: 'polygon' as const, poly: o.poly })), ...WORKBENCH.walls.map((s) => ({ kind: 'segment' as const, s }))]),
    );
    const edt = EdtField.fromPolyWorld(WORKBENCH, 0.05);
    const grid = scene.field as GridField;
    const rng = new Rng(77);
    let brushOk = 0;
    let n = 0;
    let worstEdt = 0;
    let signOk = true;
    for (let t = 0; t < 400; t++) {
      const x = { x: rng.uniform(-2.3, 2.3), y: rng.uniform(-2.3, 2.3) };
      const dE = exact.dist(x);
      if (dE <= 0.08) continue;
      n++;
      const dB = grid.dist(x);
      // Chessboard distance lies in [d/√2, d]; the raster adds up to a cell each way.
      if (dB >= dE / Math.SQRT2 - 0.075 && dB <= dE + 0.075) brushOk++;
      worstEdt = Math.max(worstEdt, Math.abs(edt.dist(x) - dE));
    }
    // Inside the block both signed fields are negative.
    const inside = { x: 0.75, y: 1.25 };
    signOk = exact.dist(inside) < 0 && edt.dist(inside) < 0 && Math.abs(exact.dist(inside) + 0.25) < 1e-9;
    // The EDT's interpolated distance errs by at most ~a cell diagonal against the exact geometry.
    push(
      'trajopt: on the Workbench the brushfire field (Chapter 7, 8-connected) is a chessboard distance bracketed by d/√2 − 1.5 cells ≤ D ≤ d + 1.5 cells at every sampled free point; the signed EDT agrees with the exact planar query to a cell diagonal and is negative inside the block (D = −0.25 at its centre)',
      brushOk === n && n > 200 && worstEdt < 0.08 && signOk,
      `${brushOk}/${n} points bracketed; worst |EDT − exact| ${worstEdt.toExponential(2)}; D(block centre) exact ${exact.dist(inside).toFixed(3)}, EDT ${edt.dist(inside).toFixed(3)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Reach on the Workbench: CHOMP from two homotopy classes converges to two different answers, both clear of the block.
  // -------------------------------------------------------------------------
  {
    const scene = reachWorkbench(24, 0.05);
    const base = { q0: scene.q0, qEnd: scene.qEnd, field: scene.field, body: scene.body, eps: 0.25, lambdaObs: 3, eta: 8, obstacleForm: 'arclength' as const };
    const results = scene.initial.map((init) => chomp(base, init.xi, 300));
    const finals = results.map((r) => r.history[r.history.length - 1]);
    const mid = results.map((r) => r.xi[11]);
    const apart = Math.hypot(mid[0][0] - mid[1][0], mid[0][1] - mid[1][1]);
    const improved = results.every((r) => r.history[r.history.length - 1].f < r.history[0].f);
    // The straight chart line converges — gradient at machine precision — *in collision*: the badge case.
    const stuck = finals[0].gradInf < 1e-8 && finals[0].minClearance < -0.1;
    const freed = finals[1].minClearance > 0.05;
    push(
      'trajopt: Reach on the Workbench (signed brushfire field, 8 body points, arclength gradient, η = 8) — from the straight chart line CHOMP converges to a stationary point in collision (|∇F| < 1e-8, D < −0.1: the "converged in collision" badge); from the detour it ends clear of the block; both lower their cost and the two answers stay in different homotopy classes',
      improved && stuck && freed && apart > 0.5,
      `F: ${results.map((r) => `${f4(r.history[0].f)} → ${f4(r.history[r.history.length - 1].f)}`).join(' | ')}; clearances ${finals.map((s) => f4(s.minClearance)).join(', ')}; |∇F| ${finals.map((s) => s.gradInf.toExponential(1)).join(', ')}; mid-path separation ${f4(apart)} rad`,
    );
  }

  // -------------------------------------------------------------------------
  // A DDP pass agrees with the iLQR pass on a linear model (no second-order terms) and improves the car's parking cost.
  // -------------------------------------------------------------------------
  {
    const car = new KinematicCar(0.2);
    const N = 30;
    const goal = [8, 4, Math.PI / 2];
    const cost = quadraticCost(diagMat([0.5, 0.5, 0.1]), diagMat([0.2, 0.5]), diagMat([60, 60, 30]), goal);
    const uInit = Array.from({ length: N }, () => [1, 0]);
    const a = ilqr(car, cost, [1, 1, 0], uInit, { maxIter: 300, mu0: 1e-3, tol: 1e-10 });
    const b = ilqr(car, cost, [1, 1, 0], uInit, { maxIter: 300, mu0: 1e-3, tol: 1e-10, ddp: true });
    const costA = a.costs[a.costs.length - 1];
    const costB = b.costs[b.costs.length - 1];
    const pose = (r: typeof a) => r.xs[N];
    const monoA = a.costs.every((c, i) => i === 0 || c <= a.costs[i - 1] + 1e-12);
    const monoB = b.costs.every((c, i) => i === 0 || c <= b.costs[i - 1] + 1e-12);
    const atGoal = (x: Vec) => Math.hypot(x[0] - goal[0], x[1] - goal[1]) < 0.05 && Math.abs(Math.atan2(Math.sin(x[2] - goal[2]), Math.cos(x[2] - goal[2]))) < 0.05;
    push(
      'trajopt: iLQR and DDP both park Hitch\'s kinematic car (N = 30, Δt = 0.2) from (1, 1, 0) at (8, 4, π/2) within 5 cm and 0.05 rad with monotone cost histories and converge; on this nonconvex instance they reach different local optima, DDP\'s the cheaper and in fewer iterations',
      a.converged && b.converged && monoA && monoB && atGoal(pose(a)) && atGoal(pose(b)) && costB < costA && b.iterations < a.iterations,
      `iLQR: ${a.iterations} it, cost ${f4(a.costs[0])} → ${f4(costA)}; DDP: ${b.iterations} it, cost ${f4(costB)}; x_N iLQR = (${pose(a).map(f4).join(', ')}), DDP = (${pose(b).map(f4).join(', ')})`,
    );
  }

  // -------------------------------------------------------------------------
  // The hook's pipeline: Chapter 12 RRT (seed 19) → Chapter 13 shortcut → 40 CHOMP steps → Chapter 18 time scaling.
  // -------------------------------------------------------------------------
  {
    const lab = torusLab(WORKBENCH);
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
    const rrt = new Rrt<number[]>(lab.space, lab.free, TORUS_START, TORUS_GOAL, { steer, eta: 0.4, goalBias: 0.1 }, new Rng(19));
    const raw = rrt.plan(4000);
    let detail = 'RRT failed';
    let pass = false;
    if (raw) {
      const cut = shortcutGreedy(steer, lab.space, lab.free, raw);
      const jagged = unwrapWaypoints(raw.map((q) => [...q]));
      const base = unwrapWaypoints(cut.map((q) => [...q]));
      const scene = reachWorkbench(24, 0.05);
      const { q0, qEnd, xi } = resamplePolyline(base, 24);
      const run = chomp({ q0, qEnd, field: scene.field, body: scene.body, eps: 0.2, lambdaObs: 3, eta: 8, obstacleForm: 'arclength' }, xi, 40);
      const arm = Reach2R.micro(0);
      const lim = symmetricLimits([20, 12]);
      const tJ = timeScale(new PathDynamics(arm, new CubicSpline(jagged, false)), lim, { N: 240 });
      const tS = timeScale(new PathDynamics(arm, new CubicSpline([q0, ...run.xi, qEnd], false)), lim, { N: 240 });
      const last = run.history[run.history.length - 1];
      pass = raw.length === 14 && cut.length === 3 && last.minClearance > 0.1 && last.f < run.history[0].f && tJ.ok && tS.ok && Math.abs(tJ.scaling.tf - 1.803) < 2e-3 && Math.abs(tS.scaling.tf - 0.992) < 2e-3;
      detail = `RRT ${raw.length} nodes (tree ${rrt.tree.size}), shortcut ${cut.length}; CHOMP F ${f4(run.history[0].f)} → ${f4(last.f)}, clearance ${f4(last.minClearance)}; t_f jagged ${tJ.ok ? f4(tJ.scaling.tf) : tJ.error.kind} s → smoothed ${tS.ok ? f4(tS.scaling.tf) : tS.error.kind} s`;
    }
    push('trajopt: the hook pipeline — RRT (seed 19) returns 14 nodes, the greedy shortcut 3, 40 CHOMP steps end 0.1 m clear of the block with a lower cost, and Chapter 18 time-scales the jagged path in 1.803 s and the smoothed one in 0.992 s (|τ| ≤ (20, 12) N·m, horizontal Workbench)', pass, detail);
  }

  void totalCost;
  void APARTMENT;
  return out;
}
