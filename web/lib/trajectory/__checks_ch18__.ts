/**
 * Numerical self-checks for Chapter 18 — `trajectory`.
 *
 * Same contract as `lib/__checks__.ts`: the design doc's micro-example to its
 * stated tolerance, plus invariants the mathematics guarantees — the
 * closed-form limit curve against a brute-force scan, bang-bang saturation at
 * every node, the trajectory touching the limit curve only at switches,
 * TOPP-RA agreeing with the switching-point construction, the lattice
 * reproducing the hand-traced bang-bang sequence, and the planned torques
 * replaying the path on the Chapter 17 integrator.
 */

import type { CheckResult } from '../__checks__';
import { Reach2R } from '../dynamics/reach2r';
import { rk4StepDynamics, type State } from '../dynamics/integrate';
import { Rng } from '../prob/rng';
import { PathDynamics } from './constrained';
import { latticeSearch, gridSearch, latticeToState, reachableStates, bangBangTime1D } from './lattice';
import { accelBounds, symmetricLimits, velocityLimit, velocityLimitBruteForce, zeroInertiaLimit } from './limits';
import { microProblem } from './micro';
import { CubicSpline, numericDerivatives } from './path';
import { RpArm, RpLinePath } from './rp-arm';
import { intoTrajectory, interpGrid, phaseAt, timeScale } from './time-scale';
import { toppra } from './toppra';

const f4 = (x: number) => x.toFixed(4);

export function runCh18Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The micro-example: a, b, c, v(s), the switch, t_s, t_f, peak ṡ, torque honesty.
  // -------------------------------------------------------------------------
  {
    const { pd, lim } = microProblem(0);
    const { a, b, c } = pd.abc(0.5);
    const v = velocityLimit(pd.abc(0.5), lim);
    const vAt0 = velocityLimit(pd.abc(0), lim);
    const res = timeScale(pd, lim, { N: 400 });
    let detail = `a = [${f4(a[0])}, ${f4(a[1])}]; b = [${f4(b[0])}, ${f4(b[1])}]; c = [${f4(c[0])}, ${f4(c[1])}]; v = ${f4(v)}`;
    let pass =
      Math.abs(a[0] - Math.PI) < 1e-3 &&
      Math.abs(a[1] - Math.PI / 6) < 1e-3 &&
      Math.abs(b[0]) < 1e-9 &&
      Math.abs(b[1] - 1.2337) < 1e-3 &&
      Math.abs(c[0]) < 1e-12 &&
      Math.abs(v - 3.5254) < 1e-3 &&
      Math.abs(vAt0 - v) < 1e-9;
    if (res.ok) {
      const sc = res.scaling;
      const kSwitch = Math.round(0.5 * sc.N);
      const ts = sc.t[kSwitch];
      const peak = Math.max(...sc.sdot);
      const { a: a2, b: b2 } = pd.abc(0.5);
      const U1 = 20 / Math.PI;
      const tau2peak = a2[1] * U1 + b2[1] * peak * peak;
      detail += `; switches = [${sc.switches.map(f4).join(', ')}]; t_s = ${f4(ts)} s; t_f = ${f4(sc.tf)} s; peak ṡ = ${f4(peak)}; τ₂ at the switch = ${tau2peak.toFixed(2)} N·m`;
      pass =
        pass &&
        sc.switches.length === 1 &&
        Math.abs(sc.switches[0] - 0.5) < 1e-3 &&
        Math.abs(ts - 0.3963) < 1e-3 &&
        Math.abs(sc.tf - 0.7927) < 1e-3 &&
        Math.abs(peak - 2.5231) < 1e-3 &&
        tau2peak < 12 &&
        Math.abs(tau2peak - 11.19) < 2e-2;
    } else {
      pass = false;
      detail += `; time scaling failed: ${res.error.kind} ${res.error.detail}`;
    }
    push('trajectory: micro-example a(½) = (π, π/6), b(½) = (0, 1.2337), v(s) = 3.5254, one switch at s = ½, t_s = 0.3963 s, t_f = 0.7927 s, peak ṡ = 2.5231', pass, detail);
  }

  {
    // Gravity on: c(½) = (10.405, −3.468), U₁(½) = 3.054, L₁(½) = −9.678, v(½) = 4.080; at s = 0, U₁ = 0.121.
    const { pd, lim } = microProblem(9.81);
    const abcHalf = pd.abc(0.5);
    const abc0 = pd.abc(0);
    const v = velocityLimit(abcHalf, lim);
    const U1 = (20 - abcHalf.c[0]) / abcHalf.a[0];
    const L1 = (-20 - abcHalf.c[0]) / abcHalf.a[0];
    const U10 = (20 - abc0.c[0]) / abc0.a[0];
    const bounds0 = accelBounds(abc0, lim, 0);
    const pass =
      Math.abs(abcHalf.c[0] - 10.405) < 1e-3 &&
      Math.abs(abcHalf.c[1] + 3.468) < 1e-3 &&
      Math.abs(U1 - 3.054) < 1e-3 &&
      Math.abs(L1 + 9.678) < 1e-3 &&
      Math.abs(v - 4.0799) < 1e-3 &&
      Math.abs(U10 - 0.121) < 1e-3 &&
      Math.abs(bounds0.U - U10) < 1e-9;
    push(
      'trajectory: gravity on — c(½) = (10.405, −3.468), U₁(½) = 3.054, L₁(½) = −9.678, v(½) = 4.0799, U₁(0) = 0.1210 (the arm can barely lift)',
      pass,
      `c = [${f4(abcHalf.c[0])}, ${f4(abcHalf.c[1])}]; U₁ = ${f4(U1)}; L₁ = ${f4(L1)}; v(½) = ${f4(v)}; U₁(0) = ${f4(U10)}`,
    );
  }

  // -------------------------------------------------------------------------
  // The pairwise closed form for v(s) equals the brute-force scan of L ≤ U.
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(18);
    let worst = 0;
    let finite = 0;
    for (let k = 0; k < 150; k++) {
      const n = 2 + (k % 2);
      const a = Array.from({ length: n }, () => rng.uniform(-3, 3));
      const b = Array.from({ length: n }, () => rng.uniform(-4, 4));
      const c = Array.from({ length: n }, () => rng.uniform(-5, 5));
      const lim = { min: Array.from({ length: n }, () => rng.uniform(-30, -8)), max: Array.from({ length: n }, () => rng.uniform(8, 30)) };
      const abc = { a, b, c };
      if (!(accelBounds(abc, lim, 0).L <= accelBounds(abc, lim, 0).U)) continue; // ṡ = 0 must be admissible (Choset's assumption)
      const closed = velocityLimit(abc, lim);
      const brute = velocityLimitBruteForce(abc, lim, 60, 60_000);
      if (Number.isFinite(closed) || Number.isFinite(brute)) {
        finite++;
        if (!Number.isFinite(closed) || !Number.isFinite(brute)) worst = Infinity;
        else worst = Math.max(worst, Math.abs(closed - brute));
      }
    }
    push('trajectory: pairwise closed form for v(s) equals the brute-force scan of L ≤ U on 150 seeded (a, b, c) rows', worst < 1e-6 && finite > 50, `${finite} finite cases, worst |Δv| ${worst.toExponential(2)}`);
  }

  // -------------------------------------------------------------------------
  // A natural cubic spline on T² passes through its waypoints, is C² (dq, ddq
  // match finite differences), and unwraps across the seam.
  // -------------------------------------------------------------------------
  {
    const wps = [
      [2.9, 0.4],
      [-3.0, 1.2],
      [-2.6, -2.8],
      [2.7, -1.0],
    ];
    const sp = CubicSpline.onTorus(wps);
    let worstQ = 0;
    for (let k = 0; k < wps.length; k++) {
      const q = sp.q(k / (wps.length - 1));
      for (let i = 0; i < 2; i++) {
        const d = Math.abs(((q[i] - wps[k][i] + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI);
        worstQ = Math.max(worstQ, d);
      }
    }
    let worstD = 0;
    let worstDD = 0;
    for (let k = 1; k < 40; k++) {
      const s = k / 40 + 0.011;
      if (s >= 1) break;
      const num = numericDerivatives(sp, s, 1e-4);
      const dq = sp.dq(s);
      const ddq = sp.ddq(s);
      for (let i = 0; i < 2; i++) {
        worstD = Math.max(worstD, Math.abs(dq[i] - num.dq[i]));
        worstDD = Math.max(worstDD, Math.abs(ddq[i] - num.ddq[i]));
      }
    }
    // Unwrapping: the first hop 2.9 → −3.0 is 0.38 rad through the seam, not 5.9 rad back across the chart.
    const hop = Math.abs(sp.pts[1][0] - sp.pts[0][0]);
    const ends = Math.max(...sp.ddq(0).map(Math.abs), ...sp.ddq(1).map(Math.abs));
    push(
      'trajectory: natural cubic spline on T² interpolates its waypoints, has analytic dq/ddq matching finite differences, unwraps the seam, and has q″ = 0 at both ends',
      worstQ < 1e-9 && worstD < 1e-6 && worstDD < 1e-4 && hop < 0.5 && ends < 1e-12,
      `waypoint error ${worstQ.toExponential(2)}, |dq − fd| ${worstD.toExponential(2)}, |ddq − fd| ${worstDD.toExponential(2)}, seam hop ${hop.toFixed(3)} rad`,
    );
  }

  // -------------------------------------------------------------------------
  // Bang-bang: at every grid node of the optimal profile some actuator is
  // saturated (except on singular arcs), and the profile never exceeds ṡ_max.
  // The limit curve is touched only at switching points (tangent points).
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(1811);
    let problems = 0;
    let touches = 0;
    let badTouch = 0;
    let unsaturated = 0;
    let exceeded = 0;
    let solved = 0;
    const N = 400;
    for (let trial = 0; trial < 12 && problems < 8; trial++) {
      const arm = Reach2R.micro(9.81);
      const wps = Array.from({ length: 4 }, () => [rng.uniform(-Math.PI, Math.PI), rng.uniform(-2.5, 2.5)]);
      const path = CubicSpline.onTorus(wps);
      const pd = new PathDynamics(arm, path);
      const lim = symmetricLimits([rng.uniform(14, 30), rng.uniform(6, 12)]);
      const res = timeScale(pd, lim, { N });
      problems++;
      if (!res.ok) continue;
      solved++;
      const sc = res.scaling;
      for (let k = 0; k <= N; k++) {
        const xmax = sc.limit[k].sdotMax;
        if (sc.sdot[k] > xmax * (1 + 1e-6) + 1e-9) exceeded++;
        const onCurve = Number.isFinite(xmax) && sc.sdot[k] >= xmax * (1 - 1e-4) - 1e-9;
        if (onCurve) {
          touches++;
          const nearSwitch = sc.switchPoints.some((sw) => sw.onLimit && Math.abs(sw.s - sc.s[k]) <= 1.5 / N);
          if (!nearSwitch && sc.mode[k] !== 'slide') badTouch++;
        }
        if (k === 0 || k === N) continue;
        if (sc.mode[k] === 'slide') continue;
        const u = pd.torque(sc.s[k], sc.sdot[k], sc.sddot[k]);
        const sat = u.some((ui, i) => ui >= lim.max[i] * (1 - 1e-6) - 1e-9 || ui <= lim.min[i] * (1 - 1e-6) + 1e-9);
        if (!sat) unsaturated++;
      }
    }
    push(
      'trajectory: on seeded Reach splines the optimal profile saturates an actuator at every node, never exceeds ṡ_max, and touches the limit curve only at tangent switching points',
      solved >= 4 && exceeded === 0 && badTouch === 0 && unsaturated === 0,
      `${solved}/${problems} problems solved, ${touches} limit-curve touches (all at switches), ${exceeded} exceedances, ${unsaturated} unsaturated nodes`,
    );
  }

  // -------------------------------------------------------------------------
  // The construction is self-consistent: x(s) is continuous across every switch
  // (stage accelerations reproduce the nodes), every node is admissible, and
  // TOPP-RA's reachability construction lands on the same t_f.
  // -------------------------------------------------------------------------
  {
    const { pd, lim } = microProblem(0);
    const ts = timeScale(pd, lim, { N: 400 });
    const tr = toppra(pd, lim, { N: 200 });
    const dMicro = ts.ok ? Math.abs(tr.tf - ts.scaling.tf) : Infinity;
    // And on gravity-loaded splines, where the limit curve is active and zero-inertia points occur.
    const rng = new Rng(7);
    let worstRel = 0;
    let compared = 0;
    let attempted = 0;
    let worstJump = 0;
    let inadmissible = 0;
    let disagree = 0;
    for (let trial = 0; trial < 8; trial++) {
      const arm = Reach2R.micro(9.81);
      const wps = Array.from({ length: 4 }, () => [rng.uniform(-Math.PI, Math.PI), rng.uniform(-2.5, 2.5)]);
      const pd2 = new PathDynamics(arm, CubicSpline.onTorus(wps));
      const lim2 = symmetricLimits([rng.uniform(16, 30), rng.uniform(7, 12)]);
      const N = 800;
      const a = timeScale(pd2, lim2, { N });
      const b = toppra(pd2, lim2, { N });
      attempted++;
      // Both algorithms must agree on *whether* the problem is solvable (gravity can make rest infeasible).
      if (a.ok !== b.ok) disagree++;
      if (!a.ok || !b.ok) continue;
      compared++;
      worstRel = Math.max(worstRel, Math.abs(a.scaling.tf - b.tf) / a.scaling.tf);
      const sc = a.scaling;
      for (let k = 0; k < N; k++) {
        // The stage acceleration must carry x_k to x_{k+1} exactly — a discontinuous profile is the bug this check guards.
        const xk = sc.sdot[k] * sc.sdot[k];
        const xn = sc.sdot[k + 1] * sc.sdot[k + 1];
        worstJump = Math.max(worstJump, Math.abs(xk + (2 * sc.stage[k]) / N - xn));
        const bd = accelBounds(pd2.abc(k / N), lim2, sc.sdot[k]);
        if (bd.L > bd.U + 1e-6) inadmissible++;
        // And the stage acceleration (an RK4 average over the stage) must lie inside the cone spanned by the two stage ends.
        const bdNext = accelBounds(pd2.abc((k + 1) / N), lim2, sc.sdot[k + 1]);
        const hi = Math.max(bd.U, bdNext.U);
        const lo = Math.min(bd.L, bdNext.L);
        // 1% slack: the RK4 average can overshoot an endpoint bound when L or U has an interior extremum within the stage.
        if (sc.stage[k] > hi + 1e-2 * Math.max(1, Math.abs(hi)) || sc.stage[k] < lo - 1e-2 * Math.max(1, Math.abs(lo))) inadmissible++;
      }
    }
    push(
      'trajectory: TOPP-RA (200-point grid) matches the Time-Scaling Algorithm on the micro-example to 5e-3 s and to 1% on seeded gravity-loaded splines (N = 800); both agree on solvability; the profile is continuous, admissible at every node, and every stage acceleration lies in the cone to 1%',
      tr.ok && dMicro < 5e-3 && compared >= 4 && worstRel < 1e-2 && disagree === 0 && worstJump < 1e-9 && inadmissible === 0,
      `micro |Δt_f| = ${dMicro.toExponential(2)} (TOPP-RA t_f = ${f4(tr.tf)}); ${compared}/${attempted} splines solvable and compared, worst relative ${worstRel.toExponential(2)}, ${disagree} solvability disagreements, worst stage jump ${worstJump.toExponential(1)}, ${inadmissible} inadmissible nodes`,
    );
  }

  // -------------------------------------------------------------------------
  // Choset Example 11.2.1: the RP arm's straight-line path. a₂(½) = b₂(½) = 0 is
  // a zero-inertia point with no velocity bound (Problem 11.5). With the
  // printed parameters the gravity torque at either end, 36.33 N·m, exceeds
  // the 20 N·m limit: U(0, 0) < 0 and L(1, 0) > 0, so the rest-to-rest problem
  // has no solution and both algorithms must say so (Problems 11.3/11.4).
  // Choset reports ≈ 0.888 s; that number is not reproducible from the printed
  // parameters, so the chapter reports the a_g = 0 execution instead: one
  // switch at s = ½ and t_f = 1.1445 s, TOPP-RA agreeing to the grid.
  // -------------------------------------------------------------------------
  {
    const lim = symmetricLimits([20, 40]);
    const printed = new PathDynamics(new RpArm(), new RpLinePath());
    const abcHalf = printed.abc(0.5);
    const zip = zeroInertiaLimit(abcHalf, lim);
    const abc0 = printed.abc(0);
    const abc1 = printed.abc(1);
    const b0 = accelBounds(abc0, lim, 0);
    const b1 = accelBounds(abc1, lim, 0);
    const resPrinted = timeScale(printed, lim, { N: 800 });
    const trPrinted = toppra(printed, lim, { N: 800 });
    const flat = new PathDynamics(new RpArm(5, 0.1, 0.2, 3, 0.05, 0), new RpLinePath());
    const resFlat = timeScale(flat, lim, { N: 800 });
    const trFlat = toppra(flat, lim, { N: 800 });
    const okZip = Math.abs(abcHalf.a[1]) < 1e-9 && Math.abs(abcHalf.b[1]) < 1e-9 && !Number.isFinite(zip);
    const okStatic = Math.abs(Math.abs(abc0.c[0]) - 36.33) < 0.01 && Math.abs(abc1.c[0] - 36.33) < 0.01 && b0.U < 0 && b1.L > 0;
    const okPrinted = !resPrinted.ok && resPrinted.error.kind === 'StallsAtZero' && !trPrinted.ok;
    const okFlat = resFlat.ok && trFlat.ok && resFlat.scaling.switches.length === 1 && Math.abs(resFlat.scaling.switches[0] - 0.5) < 1e-3 && Math.abs(resFlat.scaling.tf - 1.1445) < 1e-3 && Math.abs(trFlat.tf - resFlat.scaling.tf) < 2e-3;
    push(
      'trajectory: Choset Example 11.2.1 (RP arm) — a₂(½) = b₂(½) = 0 is a zero-inertia point with no velocity bound; as printed |c₁| = 36.33 > 20 N·m at both ends so U(0,0) < 0, L(1,0) > 0 and both algorithms report no rest-to-rest solution; with a_g = 0 one switch at s = ½ and t_f = 1.1445 s',
      okZip && okStatic && okPrinted && okFlat,
      `a₂(½) = ${abcHalf.a[1].toExponential(1)}, c₁(0) = ${abc0.c[0].toFixed(2)}, U(0,0) = ${b0.U.toFixed(3)}, L(1,0) = ${b1.L.toFixed(3)}; printed: ${resPrinted.ok ? 'solved?!' : resPrinted.error.kind} / TOPP-RA ${trPrinted.ok ? 'solved?!' : 'empty controllable set'}; a_g = 0: ${resFlat.ok ? `t_f = ${f4(resFlat.scaling.tf)} s, switches = [${resFlat.scaling.switches.map(f4).join(', ')}], TOPP-RA ${f4(trFlat.tf)} s` : resFlat.error.kind}`,
    );
  }

  // -------------------------------------------------------------------------
  // Replay: the planned torques u(t) = a s̈ + b ṡ² + c, integrated by Chapter 17's
  // RK4, track q(s(t)). The arm hangs below the shoulder here (a stable
  // pendulum): an open-loop replay through the inverted configuration would
  // diverge exponentially whatever the planner did, which is Chapter 19's cue.
  // -------------------------------------------------------------------------
  const replayPath = () => CubicSpline.onTorus([[-2.2, 1.2], [-1.5, 1.9], [-0.9, 1.1]]);
  {
    const arm = Reach2R.micro(9.81);
    const pd = new PathDynamics(arm, replayPath());
    const lim = symmetricLimits([20, 12]);
    const res = timeScale(pd, lim, { N: 800 });
    let worstQ = Infinity;
    let worstTorque = 0;
    let tf = 0;
    let nSamples = 0;
    if (res.ok) {
      const sc = res.scaling;
      tf = sc.tf;
      const traj = intoTrajectory(sc, pd, 1e-3);
      nSamples = traj.samples.length;
      const uAt = (t: number) => {
        const ph = phaseAt(sc, t);
        return pd.torque(ph.s, ph.sdot, ph.sddot);
      };
      let st: State = { q: traj.samples[0].q.slice(), qd: traj.samples[0].qd.slice() };
      worstQ = 0;
      const h = 2.5e-4;
      const steps = Math.round(sc.tf / h);
      for (let k = 0; k < steps; k++) {
        const t = k * h;
        st = rk4StepDynamics(arm, st, uAt(t + h / 2), h);
        const ph = phaseAt(sc, t + h);
        const ref = pd.path.q(ph.s);
        worstQ = Math.max(worstQ, Math.abs(st.q[0] - ref[0]), Math.abs(st.q[1] - ref[1]));
      }
      for (const smp of traj.samples) for (let i = 0; i < 2; i++) worstTorque = Math.max(worstTorque, Math.abs(smp.u[i]) / lim.max[i]);
    }
    push(
      'trajectory: replaying u(t) = a s̈ + b ṡ² + c through the Chapter 17 RK4 tracks q(s(t)) within 1e-3 rad over the whole motion, with every sampled torque inside its limit to 1% (the stage-constant s̈ discretization)',
      res.ok && worstQ < 1e-3 && worstTorque <= 1.01,
      res.ok ? `t_f = ${f4(tf)} s, ${nSamples} samples, worst |Δq| ${worstQ.toExponential(2)} rad, peak |τ|/τ_max ${worstTorque.toFixed(4)}` : `failed: ${res.error.kind} ${res.error.detail}`,
    );
  }

  // -------------------------------------------------------------------------
  // Monotonicity: stronger motors never make the optimal time longer.
  // -------------------------------------------------------------------------
  {
    const arm = Reach2R.micro(9.81);
    const pd = new PathDynamics(arm, replayPath());
    let prev = Infinity;
    let monotone = true;
    const tfs: number[] = [];
    for (const scale of [1, 1.25, 1.5, 2, 3]) {
      const res = timeScale(pd, symmetricLimits([20 * scale, 12 * scale]), { N: 400 });
      if (!res.ok) {
        monotone = false;
        break;
      }
      tfs.push(res.scaling.tf);
      if (res.scaling.tf > prev + 1e-9) monotone = false;
      prev = res.scaling.tf;
    }
    push('trajectory: t_f is non-increasing as the torque limits are scaled up 1× → 3×', monotone && tfs.length === 5, `t_f = [${tfs.map(f4).join(', ')}]`);
  }

  // -------------------------------------------------------------------------
  // GRID SEARCH: the 1-DOF hand trace, the lattice count, and A* vs BFS.
  // -------------------------------------------------------------------------
  {
    const prm = { aMax: 1, h: 0.5, vMax: 10 };
    const start = { p: [0], m: [0] };
    const goal = { p: [8], m: [0] }; // q = 8 · ½ · 1 · 0.25 = 1
    const astar = latticeSearch(start, goal, prm, 'astar');
    const bfs = latticeSearch(start, goal, prm, 'bfs');
    const controls = astar.controls.map((c) => c[0]);
    const vel = astar.states.slice(1).map((s) => latticeToState(s, prm).qd[0]);
    const pos = astar.states.slice(1).map((s) => latticeToState(s, prm).q[0]);
    const expected = { controls: [1, 1, -1, -1], vel: [0.5, 1, 0.5, 0], pos: [0.125, 0.5, 0.875, 1] };
    const same = (a: number[], b: number[]) => a.length === b.length && a.every((v, i) => Math.abs(v - b[i]) < 1e-12);
    const count = reachableStates(1, 4);
    const pass =
      astar.found &&
      bfs.found &&
      same(controls, expected.controls) &&
      same(vel, expected.vel) &&
      same(pos, expected.pos) &&
      Math.abs(astar.tf - 2) < 1e-12 &&
      Math.abs(astar.continuousBound - 2) < 1e-12 &&
      bfs.levels === astar.levels &&
      astar.expansions <= bfs.expansions &&
      count === 49;
    push(
      'trajectory: GRID SEARCH (1-DOF, a_max = 1, h = ½, (0,0) → (1,0)) returns (+1, +1, −1, −1), velocities 0.5, 1, 0.5, 0, positions 0.125, 0.5, 0.875, 1, t_f = 2 = 2√(d/a_max); 81 sequences → 49 states at level 4; A* expands no more than BFS',
      pass,
      `controls [${controls.join(', ')}], t_f ${astar.tf}, states at level 4: ${count}, expansions A* ${astar.expansions} vs BFS ${bfs.expansions}`,
    );
  }

  {
    // 2-DOF with an obstacle and a speed-dependent margin. Algorithm 21 as printed
    // (level-by-level, goal tested at generation), the heap BFS, and A* all return
    // the same level count; A* expands far fewer nodes; the lattice time never beats
    // the continuous bang-bang bound; a larger c₁ never gives a shorter trajectory.
    // The breadth-first comparisons run at h = ½ (the lattice below level 12 at
    // h = ¼ holds ~8·10⁵ states, which is Theorem 11.3.2's O((1/ε)^{3n}) in action).
    const clearance = (q: number[]) => Math.hypot(q[0] - 1, q[1] - 0.25) - 0.3; // a disc in the middle of the room
    const start = { p: [0, 0], m: [0, 0] };
    const run = (c1: number, h: number, breadth: boolean) => {
      const prm = { aMax: 1, h, vMax: 2, clearance, c0: 0.05, c1, arcSamples: 4 };
      const goal = { p: [Math.round(2 / (0.5 * h * h)), 0], m: [0, 0] };
      return {
        a: latticeSearch(start, goal, prm, 'astar'),
        b: breadth ? latticeSearch(start, goal, prm, 'bfs') : null,
        g: breadth ? gridSearch(start, goal, prm) : null,
      };
    };
    const coarse0 = run(0, 0.5, true);
    const coarse1 = run(0.5, 0.5, true);
    const fine0 = run(0, 0.25, false);
    const fine1 = run(0.5, 0.25, false);
    const sameLevels = (r: ReturnType<typeof run>) => r.a.found && r.b!.found && r.g!.found && r.a.levels === r.b!.levels && r.a.levels === r.g!.levels;
    const pass =
      sameLevels(coarse0) &&
      sameLevels(coarse1) &&
      coarse0.a.expansions < coarse0.g!.expansions &&
      coarse0.g!.expansions < coarse0.b!.expansions &&
      fine0.a.found &&
      fine1.a.found &&
      fine0.a.tf >= fine0.a.continuousBound - 1e-12 &&
      coarse0.a.tf >= coarse0.a.continuousBound - 1e-12 &&
      fine1.a.tf >= fine0.a.tf &&
      coarse1.a.tf >= coarse0.a.tf &&
      fine0.a.tf <= coarse0.a.tf + 1e-12;
    push(
      'trajectory: 2-DOF GRID SEARCH around a disc — Algorithm 21, heap BFS and A* return the same level count, A* expands fewest nodes, the lattice time never beats the continuous bound 2√2 s, a finer h never makes it longer, and δ_v safety with c₁ > 0 costs time',
      pass,
      `h = ½, c₁ = 0: ${coarse0.a.levels} levels (t_f ${coarse0.a.tf} s, bound ${f4(coarse0.a.continuousBound)}), expansions A* ${coarse0.a.expansions} / Alg. 21 ${coarse0.g!.expansions} / heap BFS ${coarse0.b!.expansions}; c₁ = ½: ${coarse1.a.levels} levels, ${coarse1.g!.pruned} pruned; h = ¼: ${fine0.a.levels} → ${fine1.a.levels} levels (t_f ${fine0.a.tf} → ${fine1.a.tf} s), A* ${fine0.a.expansions} expansions`,
    );
  }

  {
    // bangBangTime1D: rest-to-rest over d at a takes 2√(d/a); from v₀ = 1 a pure
    // brake covers only v₀²/2a = ½ m in 1 s, so reaching d = 1 needs a short
    // acceleration to v_p = √1.5 first (t = 2v_p − 1 = 1.4495 s); and the s-grid
    // interpolation helper is exact on linear data.
    const t = bangBangTime1D(0, 0, 1, 0, 1);
    const tBrake = bangBangTime1D(0, 1, 0.5, 0, 1);
    const tFar = bangBangTime1D(0, 1, 1, 0, 1);
    const lin = Array.from({ length: 11 }, (_, k) => 3 * (k / 10) + 1);
    const ip = interpGrid(lin, 0.437);
    push(
      'trajectory: bang-bang closed form — rest-to-rest over d = 1 at a = 1 takes 2 s; from v₀ = 1 the pure brake reaches d = ½ in 1 s and d = 1 in 2√1.5 − 1 = 1.4495 s',
      Math.abs(t - 2) < 1e-12 && Math.abs(tBrake - 1) < 1e-12 && Math.abs(tFar - (2 * Math.sqrt(1.5) - 1)) < 1e-12 && Math.abs(ip - (3 * 0.437 + 1)) < 1e-12,
      `t = ${t}, t_brake = ${tBrake}, t_far = ${tFar.toFixed(4)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Numbers the prose and the exercises quote beyond the micro-example.
  // -------------------------------------------------------------------------
  {
    const { pd } = microProblem(0);
    const r10 = timeScale(pd, symmetricLimits([20, 10]), { N: 400 });
    const v10 = velocityLimit(pd.abc(0.5), symmetricLimits([20, 10]));
    const g = microProblem(9.81);
    const rg = timeScale(g.pd, g.lim, { N: 400 });
    const prm = { aMax: 1, h: 0.125, vMax: 2, clearance: (q: number[]) => Math.min(q[0] + 0.3, 2.3 - q[0]), c0: 0.05, c1: 0, goalPos: 0.25 * 0.125 * 0.125, goalVel: 0 };
    const fine = latticeSearch({ p: [0], m: [0] }, { p: [Math.round(2 / (0.5 * 0.125 * 0.125))], m: [0] }, prm, 'astar');
    const racer = timeScale(new PathDynamics(Reach2R.micro(9.81), replayPath()), symmetricLimits([30, 18]), { N: 800 });
    const pass =
      r10.ok && Math.abs(v10 - 3.2875) < 1e-3 && Math.abs(r10.scaling.switches[0] - 0.5071) < 1e-3 && Math.abs(r10.scaling.tf - 0.7928) < 1e-3 && Math.max(...r10.scaling.sdot) < v10 &&
      rg.ok && Math.abs(rg.scaling.tf - 2.3276) < 1e-3 && rg.scaling.switches.length === 1 && Math.abs(rg.scaling.switches[0] - 0.7342) < 1e-3 &&
      fine.found && fine.levels === 23 && Math.abs(fine.tf - 2.875) < 1e-12 &&
      racer.ok && racer.scaling.mode.some((m) => m === 'slide');
    push(
      'trajectory: quoted numbers — micro-example with τ₂ = 10: v = 3.2875, switch 0.5071 (after ½, never touching v), t_f = 0.7928; with gravity on: t_f = 2.3276, one switch at 0.7342; 1-DOF lattice d = 2, h = ⅛: 23 levels, t_f = 2.875 s; the replay path at 1.5× limits has a singular slide',
      pass,
      `τ₂ = 10: v = ${f4(v10)}, ${r10.ok ? `switch ${f4(r10.scaling.switches[0])}, t_f ${f4(r10.scaling.tf)}` : r10.error.kind}; gravity: ${rg.ok ? `t_f ${f4(rg.scaling.tf)}, switches [${rg.scaling.switches.map(f4).join(', ')}]` : rg.error.kind}; h = ⅛: ${fine.levels} levels, t_f ${fine.tf}; 1.5×: ${racer.ok ? `${racer.scaling.mode.filter((m) => m === 'slide').length} slide nodes` : racer.error.kind}`,
    );
  }

  return out;
}
