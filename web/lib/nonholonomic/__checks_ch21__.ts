/**
 * Numerical self-checks for Chapter 21 — `nonholonomic` (steering).
 *
 * The design doc's micro-example (one Dubins word for (0,0,0) → (3,3,π/2) at
 * ρ = 1, its integration and its four rivals; Example 12.5.1's flat outputs at
 * t = 0.25 and 0.5), every Dubins and Reeds–Shepp word integrating to its goal
 * on 400 seeded pairs with RS ≤ Dubins, flatness recovery against the true
 * car-with-trailer ODE, sinusoids reaching the goal in chained coordinates
 * (and Brockett's closed form), gradient steering escaping the singular
 * control, CAR GRID SEARCH's bucketed OPEN and cost semantics, hybrid A*
 * drivable and no longer than the state lattice on the Lot, the forward-
 * propagation RRT reintegrating bit for bit, the two exact steers of
 * Kinodynamic RRT*, and Theorem 12.4.7 with the decoupling drive time-scaled
 * by Chapter 18 with one actuator always saturated.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { normalizeAngle, type Pose2 } from '../geom/se2';
import { Hitch } from '../robots/hitch';
import { LOT } from '../robots/worlds';
import { Scene } from '../collide/scene';
import { FootprintChecker } from '../collide/collision';
import { addFields, lieBracket, field } from './field';
import { PbwtDynamics, kineticConnection, pbwtInputs } from './mechanical';
import { dubinsAll, dubinsEndpoint, dubinsNormalize, dubinsSegment, dubinsWord, poseError } from './dubins';
import { PARK_METHODS, planPark } from './park';
import { reedsSheppLength, reedsSheppShortestPath, rsAll, rsEndpoint } from './reeds-shepp';
import { EXAMPLE_12_5_1, fitFlat, fitFlatTrailer, flatnessReplayError, liftCar } from './flat';
import { brockettControl, chainedRhs, integrateBrockett, sinusoidGain, steerBrockett, steerCarSinusoids, steerSinusoids } from './chained';
import { steerGradient, steerNewton } from './gradient';
import { CarGridSearch, figure1225 } from './car-grid';
import { LOT_REVERSE_GOAL, LOT_REVERSE_START, hybridAStar, stateLatticeSearch } from './lattice';
import { replaySegments, samplesFree, turningRadius } from './car-path';
import { diControl, diCostAt, diIntegrate, diOptimalTime, dynCarMetric, dynCarSystem, HITCH_DYNAMIC, KinoRrt, KinoRrtStar, reedsSheppSteer, type DiState, type DynCar } from './kinodynamic';
import { chosetDeadArm, deadArmConnection, deadArmQuery, deadArmInputs, decouplingGridSearch, isDecoupling, link3Fields, link3Pose, percussionDistance, timeScaleDrive } from './reduction';

const f6 = (x: number) => x.toFixed(6);
const f5 = (x: number) => x.toFixed(5);

/** Scoreboard lengths the prose prints (metres). */
const PIN = { rs: 7.369, flat: 9.195, sinusoid: 27.888, grid: 7.2, hybrid: 7.369, kino: 6.854 };

export function runCh21Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // ---------------------------------------------------------------------------
  // 1. The Dubins micro-example.
  // ---------------------------------------------------------------------------
  {
    const q0: Pose2 = { x: 0, y: 0, theta: 0 };
    const q1: Pose2 = { x: 3, y: 3, theta: Math.PI / 2 };
    const all = dubinsAll(q0, q1, 1);
    const lsl = all[0];
    const byWord = new Map(all.map((p) => [p.word, p.length]));
    const afterArc = dubinsSegment(q0, 'L', lsl.segs[0], 1);
    const end = dubinsEndpoint(lsl, q0);
    const rs = reedsSheppShortestPath(q0, q1, 1)!;
    // Exercise 3's goal (2, −1, 0): Dubins RSL = 2·atan(3/4) + 1 = 2.28700, and Reeds–Shepp cannot do better.
    const ex3 = dubinsAll(q0, { x: 2, y: -1, theta: 0 }, 1)[0];
    const ex3rs = reedsSheppShortestPath(q0, { x: 2, y: -1, theta: 0 }, 1)!;
    // Its follow-up, (0.5, −1, 0): Dubins loops (LSL 7.401), Reeds–Shepp backs up (C|CC|C, 2.355).
    const fd = dubinsAll(q0, { x: 0.5, y: -1, theta: 0 }, 1)[0];
    const fr = reedsSheppShortestPath(q0, { x: 0.5, y: -1, theta: 0 }, 1)!;
    // Exercise 1's claim, sampled: no optimal CCC word once the poses are 4ρ apart.
    const rng1 = new Rng(5);
    let cccFar = 0;
    for (let i = 0; i < 20000; i++) {
      const g = { x: rng1.uniform(-8, 8), y: rng1.uniform(-8, 8), theta: rng1.uniform(-Math.PI, Math.PI) };
      const w = dubinsAll(q0, g, 1)[0].word;
      if ((w === 'RLR' || w === 'LRL') && Math.hypot(g.x, g.y) >= 4) cccFar++;
    }
    const pass =
      fd.word === 'LSL' && Math.abs(fd.length - 7.401) < 5e-4 && fr.family === 'C|CC|C' && Math.abs(fr.length - 2.355) < 5e-4 && cccFar === 0 &&
      ex3.word === 'RSL' &&
      Math.abs(ex3.length - (2 * Math.atan(0.75) + 1)) < 1e-9 &&
      Math.abs(ex3.length - 2.287) < 1e-4 &&
      ex3rs.cusps === 0 &&
      Math.abs(ex3rs.length - ex3.length) < 1e-9 &&
      lsl.word === 'LSL' &&
      Math.abs(lsl.segs[0] - Math.PI / 4) < 1e-9 &&
      Math.abs(lsl.segs[1] - 2 * Math.SQRT2) < 1e-9 &&
      Math.abs(lsl.segs[2] - Math.PI / 4) < 1e-9 &&
      Math.abs(lsl.length - 4.39922) < 1e-5 &&
      Math.abs(afterArc.x - 0.70711) < 1e-5 &&
      Math.abs(afterArc.y - 0.29289) < 1e-5 &&
      poseError(end, q1) < 1e-9 &&
      Math.abs((byWord.get('LSR') ?? 0) - 10.56698) < 1e-5 &&
      Math.abs((byWord.get('RSL') ?? 0) - 10.56698) < 1e-5 &&
      Math.abs((byWord.get('LRL') ?? 0) - 10.99557) < 1e-5 &&
      Math.abs((byWord.get('RSR') ?? 0) - 16.65243) < 1e-5 &&
      !byWord.has('RLR') &&
      dubinsWord('RLR', dubinsNormalize(q0, q1, 1)) === null &&
      rs.word === 'L+S+L+' &&
      Math.abs(rs.length - lsl.length) < 1e-9;
    push(
      'nonholonomic: Dubins micro-example (0,0,0) → (3,3,π/2), ρ = 1 — LSL with t = π/4, p = 2√2, q = π/4, length π/2 + 2√2 = 4.39922; integration passes (0.70711, 0.29289, π/4) and ends at (3, 3, π/2); LSR = RSL = 10.56698, LRL = 10.99557, RSR = 16.65243, RLR infeasible; Reeds–Shepp returns the same LSL (and for Exercise 3\'s goal (2, −1, 0) both return RSL, 2·atan(3/4) + 1 = 2.28700; at (0.5, −1, 0) Dubins needs LSL 7.401 while Reeds–Shepp backs up, C|CC|C 2.355; no optimal CCC word on 20 000 seeded goals at d ≥ 4ρ)',
      pass,
      `LSL t=${f6(lsl.segs[0])} p=${f6(lsl.segs[1])} q=${f6(lsl.segs[2])} len=${f6(lsl.length)}; after arc (${f5(afterArc.x)}, ${f5(afterArc.y)}); end (${f6(end.x)}, ${f6(end.y)}, ${f6(end.theta)}); ${all.map((p) => `${p.word}=${f5(p.length)}`).join(' ')}; RS ${rs.word} ${f5(rs.length)}; Ex. 3: Dubins ${ex3.word} ${f5(ex3.length)}, RS ${ex3rs.word} ${f5(ex3rs.length)}; (0.5,−1,0): ${fd.word} ${fd.length.toFixed(3)} vs ${fr.word} ${fr.length.toFixed(3)}; far CCC optima ${cccFar}`,
    );
  }

  // ---------------------------------------------------------------------------
  // 2. Example 12.5.1's flat outputs.
  // ---------------------------------------------------------------------------
  {
    const s = liftCar(EXAMPLE_12_5_1, 0.25, 1);
    const m = liftCar(EXAMPLE_12_5_1, 0.5, 1);
    const fit = fitFlat({ x: 0, y: 0, theta: 0 }, { x: 1, y: 1, theta: 0 }, { degree: 3, tf: 1, speed: 1 });
    const fitOk = [0, 1, 0, 0].every((c, i) => Math.abs(fit.cx[i] - c) < 1e-12) && [0, 0, 3, -2].every((c, i) => Math.abs(fit.cy[i] - c) < 1e-12);
    const pass =
      Math.abs(s.pose.x - 0.25) < 1e-12 &&
      Math.abs(s.pose.y - 0.15625) < 1e-12 &&
      Math.abs(s.pose.theta - 0.844154) < 1e-6 &&
      Math.abs(s.v - 1.505199) < 1e-6 &&
      Math.abs(s.omega - 1.324138) < 1e-6 &&
      Math.abs(s.kappa - 0.879709) < 1e-6 &&
      Math.abs(s.phi - 0.721491) < 1e-6 &&
      Math.abs(m.omega) < 1e-12 &&
      Math.abs(m.phi) < 1e-12 &&
      fitOk;
    push(
      'nonholonomic: flat outputs of the car (Example 12.5.1, y₁ = t, y₂ = 3t² − 2t³) at t = 0.25 — θ = 0.844154 (48.37°), v = 1.505199, ω = 1.324138, κ = 0.879709, φ(L = 1) = 0.721491 (41.34°); ω = φ = 0 at the inflection t = 0.5; the cubic Hermite fit of (0,0,0) → (1,1,0) with unit end speed reproduces Choset\'s polynomials',
      pass,
      `t=0.25: theta=${f6(s.pose.theta)} v=${f6(s.v)} omega=${f6(s.omega)} kappa=${f6(s.kappa)} phi=${f6(s.phi)}; t=0.5: omega=${f6(m.omega)}; fit y1=[${fit.cx.map((c) => c.toFixed(3))}] y2=[${fit.cy.map((c) => c.toFixed(3))}]`,
    );
  }

  // ---------------------------------------------------------------------------
  // 3–4. Every word integrates; RS ≤ Dubins; RS is a symmetric metric bound.
  // ---------------------------------------------------------------------------
  {
    const rng = new Rng(2121);
    let worstD = 0;
    let worstR = 0;
    let wordsD = 0;
    let wordsR = 0;
    let rsOverDubins = 0;
    let underEuclid = 0;
    let asym = 0;
    const families = new Set<string>();
    for (let i = 0; i < 400; i++) {
      const q0: Pose2 = { x: rng.uniform(-6, 6), y: rng.uniform(-6, 6), theta: rng.uniform(-Math.PI, Math.PI) };
      const q1: Pose2 = { x: rng.uniform(-6, 6), y: rng.uniform(-6, 6), theta: rng.uniform(-Math.PI, Math.PI) };
      const rho = rng.uniform(0.5, 2.5);
      const ds = dubinsAll(q0, q1, rho);
      for (const p of ds) {
        wordsD++;
        worstD = Math.max(worstD, poseError(dubinsEndpoint(p, q0), q1));
      }
      const rs = rsAll(q0, q1, rho);
      for (const p of rs) {
        wordsR++;
        worstR = Math.max(worstR, poseError(rsEndpoint(p, q0), q1));
      }
      families.add(rs[0].family);
      if (rs[0].length > ds[0].length + 1e-9) rsOverDubins++;
      if (rs[0].length < Math.hypot(q1.x - q0.x, q1.y - q0.y) - 1e-9) underEuclid++;
      if (Math.abs(reedsSheppLength(q1, q0, rho) - rs[0].length) > 1e-9) asym++;
    }
    push(
      'nonholonomic: 400 seeded pose pairs (ρ ∈ [0.5, 2.5]) — every feasible Dubins word (of six) and every Reeds–Shepp candidate (44 formula evaluations: 11 base formulas × time-flip × reflect) integrates to its goal within 10⁻⁹; Euclidean ≤ Reeds–Shepp ≤ Dubins length on every pair (a missing family or mistyped formula would break the middle inequality), and the RS length is symmetric',
      worstD < 1e-9 && worstR < 1e-13 && wordsR === 2700 && rsOverDubins === 0 && underEuclid === 0 && asym === 0,
      `${wordsD} Dubins words, worst ${worstD.toExponential(2)}; ${wordsR} RS words, worst ${worstR.toExponential(2)}; RS > Dubins: ${rsOverDubins}, RS < Euclid: ${underEuclid}, asymmetric: ${asym}; optimal RS families seen: ${[...families].sort().join(', ')}`,
    );
  }

  // ---------------------------------------------------------------------------
  // 5. Flatness recovery against the true ODE.
  // ---------------------------------------------------------------------------
  {
    const car = fitFlat({ x: 5.5, y: 1.2, theta: 0 }, { x: 9.25, y: 6.4, theta: Math.PI / 2 }, { degree: 5, tf: 1 });
    const trl = fitFlatTrailer({ x: 5.5, y: 1.2, theta: 0 }, { x: 9.25, y: 6.4, theta: Math.PI / 2 }, 2.5, { tf: 1 });
    const eCar = flatnessReplayError(car, 2.6, null);
    const eTrl = flatnessReplayError(trl, 2.6, 2.5);
    push(
      'nonholonomic: flatness recovery — integrating the true car (and car-with-trailer, d = 2.5 m) ODE by RK4 under the controls (v, φ) read off a quintic flat output reproduces the lifted state (x, y, θ[, ψ]) along the whole trajectory',
      eCar < 1e-11 && eTrl < 1e-11,
      `worst disagreement: car ${eCar.toExponential(2)}, car+trailer ${eTrl.toExponential(2)}`,
    );
  }

  // ---------------------------------------------------------------------------
  // 6. Chained form: the bracket ladder, sinusoids, Brockett, the 4-state car.
  // ---------------------------------------------------------------------------
  {
    const n = 5;
    const g1 = field('g1', n, (x) => chainedRhs(x, 1, 0));
    const g2 = field('g2', n, (x) => chainedRhs(x, 0, 1));
    let ladderErr = 0;
    let b = g2;
    const x0 = [0.3, -0.2, 0.5, 0.1, -0.4];
    for (let k = 1; k <= n - 2; k++) {
      b = lieBracket(g1, b);
      const v = b.eval(x0);
      const expect = new Array<number>(n).fill(0);
      expect[k + 1] = (-1) ** k;
      ladderErr = Math.max(ladderErr, ...v.map((t, i) => Math.abs(t - expect[i])));
    }
    const goal = [1, -0.5, 0.3, 0.2, -0.4];
    const sin = steerSinusoids([0, 0, 0, 0, 0], goal, { stepsPerPhase: 600 });
    const sinErr = Math.max(...sin.final.map((v, i) => Math.abs(v - goal[i])));
    const gainErr = Math.max(...sin.phases.slice(1).map((p) => Math.abs(sinusoidGain(p.a, p.b, p.k) - p.target)));
    const phaseErr = Math.max(...sin.phases.slice(1).map((p) => Math.abs(p.achieved - p.target)));
    const br = steerBrockett(0.3);
    const tr = integrateBrockett((t) => brockettControl(br, t));
    const brEnd = tr[tr.length - 1];
    const brErr = Math.max(Math.abs(brEnd[0]), Math.abs(brEnd[1]), Math.abs(brEnd[2] - 0.3));
    const carPlan = steerCarSinusoids({ x: 5.5, y: 1.2, theta: 0 }, { x: 9.25, y: 6.4, theta: Math.PI / 2 }, 2.6, { stepsPerPhase: 600 });
    push(
      'nonholonomic: chained form (n = 5) — the ladder [g₁,[g₁,…[g₁,g₂]]] = (−1)ᵏe_{k+2} (Chapter 20\'s lieBracket); sinusoid steering reaches the goal in chained coordinates, each cycle moving x_{k+2} by (a/4π)ᵏb/k! (eq. 12.31); Brockett\'s u₁(0) = √(4π·0.3), k = −1 returns x₁, x₂ and sets x₃(1) = 0.3; Hitch\'s 4-state car in Murray–Sastry coordinates lands on its pose',
      ladderErr < 1e-10 && sinErr < 1e-9 && gainErr < 1e-12 && Math.abs(br.u10 - 1.94163) < 1e-5 && carPlan.cusps === 3 && Math.abs(carPlan.maxSteer - 1.22) < 5e-3 && phaseErr < 1e-10 && brErr < 1e-12 && carPlan.chainedError < 1e-9 && carPlan.poseError < 1e-6,
      `ladder ${ladderErr.toExponential(1)}; final ${sinErr.toExponential(1)}; per-cycle vs (12.31) ${phaseErr.toExponential(1)}; Brockett u₁(0) = ${br.u10.toFixed(5)}, error ${brErr.toExponential(1)}; car chained ${carPlan.chainedError.toExponential(1)}, pose ${carPlan.poseError.toExponential(1)}, ${carPlan.cusps} cusps, max |φ| ${carPlan.maxSteer.toFixed(3)} rad`,
    );
  }

  // ---------------------------------------------------------------------------
  // 7. Gradient steering and the generic loop.
  // ---------------------------------------------------------------------------
  {
    const x0: Pose2 = { x: 0, y: 0, theta: 0 };
    const goal: Pose2 = { x: 0, y: 0.5, theta: 0 };
    const g = steerGradient(x0, goal, { rng: new Rng(3), K: 6, eps: 1e-4 });
    const lm = steerNewton(x0, goal, { rng: new Rng(3), K: 6, eps: 1e-9 });
    const monotone = g.errors.every((e, i) => i === 0 || e <= g.errors[i - 1] + 1e-15);
    push(
      'nonholonomic: gradient steering (§12.5.4) for 0.5 m sideways — uᵖ = 0 is a singular control (rank ∂f/∂uᵖ = 2), a generic loop restores rank 3; Choset\'s update then decreases ‖e‖ monotonically to < 10⁻⁴, and Chapter 19\'s Levenberg–Marquardt on the same end-state map reaches < 10⁻⁹',
      g.rankHistory[0] === 2 && g.rankHistory[1] === 3 && g.converged && monotone && lm.converged && g.iterations === 58 && lm.iterations === 5,
      `gradient: ranks ${g.rankHistory.join('→')}, ${g.iterations} iterations, ‖e‖ ${g.errors[0].toFixed(3)} → ${g.errors[g.errors.length - 1].toExponential(2)}; LM: ${lm.iterations} iterations → ${lm.errors[lm.errors.length - 1].toExponential(2)}`,
    );
  }

  const scene = Scene.fromWorld(LOT);
  const car = Hitch.car();

  // ---------------------------------------------------------------------------
  // 8. CAR GRID SEARCH: bucketed OPEN, cost semantics, drivability.
  // ---------------------------------------------------------------------------
  {
    const fig = figure1225(car);
    const grays = fig.nodes.filter((n) => n.collided).length;
    const openCosts = fig.open.map((o) => o.cost);
    const sorted = openCosts.every((c, i) => i === 0 || c >= openCosts[i - 1]);
    const start = { body: LOT_REVERSE_START, trailer: null };
    const goal = { pose: LOT_REVERSE_GOAL, posTol: 0.5, headTol: 0.2 };
    const mk = (c: number) => new CarGridSearch({ car, scene, start, goal, step: 0.8, d: 40, cost: { a: 1, b: 0, c }, maxTree: 200000 });
    const short = mk(0);
    short.run();
    const fewCusps = mk(300);
    fewCusps.run();
    const bs = short.costBreakdown();
    const bc = fewCusps.costBreakdown();
    const segs = short.segments();
    const replay = segs ? replaySegments(car, start, segs) : null;
    const last = short.path()?.slice(-1)[0];
    const replayErr = replay && last ? poseError(replay.body, last.q.body) : Infinity;
    push(
      'nonholonomic: CAR GRID SEARCH (Alg. 22) — the bucketed OPEN is ordered by cost (Fig. 12.25-style search: costs ' +
        openCosts.join(', ') +
        ', three right turns grayed by the block); on the Lot\'s reverse-park query, a = 1, c = 0 gives the fewest steps and a = 1, c = 300 the fewest cusps (each cost-optimal at its resolution), and the step-optimal action list re-driven by Chapter 2\'s Hitch.step lands on the planner\'s last node',
      sorted && openCosts.join(',') === '3,6,7,8' && grays === 3 && short.status === 'found' && fewCusps.status === 'found' && !!bs && !!bc && bs.steps <= bc.steps && bc.cusps <= bs.cusps && bs.steps === 20 && bs.cusps === 1 && bc.steps === 20 && bc.cusps === 1 && fewCusps.expansions < 0.55 * short.expansions && replayErr < 1e-9,
      `c = 0: ${bs?.steps} steps, ${bs?.cusps} cusps, ${short.expansions} expansions; c = 300: ${bc?.steps} steps, ${bc?.cusps} cusps, ${fewCusps.expansions} expansions; replay error ${replayErr.toExponential(1)}`,
    );
  }

  // ---------------------------------------------------------------------------
  // 9. Hybrid A* vs the state lattice on the Lot.
  // ---------------------------------------------------------------------------
  {
    const lat = stateLatticeSearch({ car, scene, start: LOT_REVERSE_START, goal: LOT_REVERSE_GOAL, cell: 0.5, heuristic: 'dual', set: 'six' });
    const hyb = hybridAStar({ car, scene, start: LOT_REVERSE_START, goal: LOT_REVERSE_GOAL, cell: 0.5, heuristic: 'dual', shotEvery: 10 });
    const rho = turningRadius(car);
    const replay = replaySegments(car, { body: LOT_REVERSE_START, trailer: null }, hyb.segs);
    const checker = new FootprintChecker(scene, car);
    const free = samplesFree(hyb.samples, checker, false) && samplesFree(lat.samples, checker, false);
    const rsLB = reedsSheppLength(LOT_REVERSE_START, LOT_REVERSE_GOAL, rho);
    push(
      'nonholonomic: hybrid A* on the Lot (reverse into bay 3, cell 0.5 m, dual heuristic, RS shot every 10 expansions) — its path re-driven by Hitch.step ends exactly on the goal, every sample is free, and it is no longer than the six-primitive state lattice\'s (both ≥ the obstacle-blind RS length)',
      hyb.found && lat.found && lat.onLattice && poseError(replay.body, LOT_REVERSE_GOAL) < 1e-9 && free && hyb.length <= lat.length + 1e-9 && hyb.length >= rsLB - 1e-9 && lat.length >= rsLB - 1e-9 && Math.abs(hyb.length - 17.107) < 5e-4 && Math.abs(lat.length - 24.368) < 5e-4 && Math.abs(rsLB - 15.189) < 5e-4 && hyb.expansions === 161 && hyb.shots.length === 17 && hyb.cusps === 3 && lat.expansions === 298 && lat.cusps === 3,
      `hybrid ${hyb.length.toFixed(3)} m (${hyb.cusps} cusps, ${hyb.expansions} expansions, ${hyb.shots.length} shots); lattice ${lat.length.toFixed(3)} m (${lat.cusps} cusps, ${lat.expansions} expansions); RS bound ${rsLB.toFixed(3)} m; replay error ${poseError(replay.body, LOT_REVERSE_GOAL).toExponential(1)}`,
    );
  }

  // ---------------------------------------------------------------------------
  // 10. Forward propagation and the exact steers.
  // ---------------------------------------------------------------------------
  {
    const checker = new FootprintChecker(scene, car);
    const b = scene.bounds;
    const start = { x: 5.5, y: 1.2, theta: 0 };
    const goal = { x: 9.25, y: 6.4, theta: Math.PI / 2 };
    const rrt = new KinoRrt<DynCar, { acc: number; steerRate: number }>(
      dynCarSystem(HITCH_DYNAMIC),
      {
        start: { ...start, v: 0, phi: 0 },
        isGoal: (s) => Math.hypot(s.x - goal.x, s.y - goal.y) < 0.8 && Math.abs(normalizeAngle(s.theta - goal.theta)) < 0.35,
        sample: (g) => ({ x: g.uniform(b.minX, b.maxX), y: g.uniform(b.minY, b.maxY), theta: g.uniform(-Math.PI, Math.PI), v: g.uniform(-2, 2), phi: 0 }),
        goalSample: () => ({ ...goal, v: 0, phi: 0 }),
        metric: dynCarMetric(1.0, 0.3),
        isFree: (s) => checker.isFree({ body: s, trailer: null }),
      },
      { controlsPerExtend: 8, dt: 0.8, substeps: 4, goalBias: 0.15 },
      new Rng(21),
    );
    rrt.plan(3000);
    const dense = rrt.densePath();
    const re = rrt.reintegrate();
    const bitwise = !!dense && !!re && dense.length === re.length && dense.every((s, i) => s.x === re[i].x && s.y === re[i].y && s.theta === re[i].theta && s.v === re[i].v && s.phi === re[i].phi);
    // Webb & van den Berg: the closed form lands on the target and τ* minimizes c(τ).
    const a: DiState = [0, 0, 1, 0];
    const t: DiState = [2, 1, 0, 0.5];
    const opt = diOptimalTime(a, t, 0.5);
    const traj = diIntegrate(a, diControl(a, t, opt.tau, 0.5), opt.tau, 800);
    const end = traj[traj.length - 1];
    const diErr = Math.max(...end.map((v, i) => Math.abs(v - t[i])));
    let tauMin = true;
    for (let k = 1; k <= 60; k++) if (diCostAt(a, t, 0.1 * k, 0.5) < opt.cost - 1e-9) tauMin = false;
    // Kinodynamic RRT* with the RS steer in an empty box: costs never increase and never beat the exact RS distance.
    const s0: Pose2 = { x: 0, y: 0, theta: 0 };
    const s1: Pose2 = { x: 6, y: 3, theta: Math.PI };
    const star = new KinoRrtStar<Pose2>(
      { start: s0, goal: s1, sample: (g) => ({ x: g.uniform(-4, 10), y: g.uniform(-5, 8), theta: g.uniform(-Math.PI, Math.PI) }), isFree: () => true },
      { steer: reedsSheppSteer(1.5), eta: 4, gamma: 12, dim: 3, goalBias: 0.05, ds: 0.2 },
      new Rng(7),
    );
    for (let i = 0; i < 400; i++) star.iterate();
    const hist = star.costHistory.map((h) => h.cost);
    const mono = hist.every((c, i) => i === 0 || c <= hist[i - 1]);
    const lb = reedsSheppLength(s0, s1, 1.5);
    push(
      'nonholonomic: forward-propagation RRT on Hitch\'s dynamic model (best of 8 controls, Δt = 0.8 s, seed 21) solves the Lot query and its stored controls re-integrate bit for bit; Webb & van den Berg\'s double-integrator edge lands on its target with τ* minimizing c(τ); Kinodynamic RRT* with the Reeds–Shepp steer improves monotonically and never beats the exact RS distance',
      rrt.solved && rrt.solvedAt === 106 && bitwise && diErr < 1e-13 && tauMin && Math.abs(opt.tau - 2.3893) < 1e-4 && Math.abs(opt.cost - 3.0075) < 1e-4 && hist.length > 0 && mono && star.bestCost >= lb - 1e-9,
      `RRT: solved at extension ${rrt.solvedAt}, ${rrt.nodes.length} nodes, ${rrt.propagations} propagations (${rrt.wasted} collided); DI: τ* = ${opt.tau.toFixed(4)}, c* = ${opt.cost.toFixed(4)}, landing error ${diErr.toExponential(1)}; RRT*: ${hist.map((c) => c.toFixed(3)).join(' → ')} (RS bound ${lb.toFixed(3)}), ${star.rewires} rewires`,
    );
  }

  // ---------------------------------------------------------------------------
  // 11. Decoupling vector fields, Theorem 12.4.7, and the time-scaled drive.
  // ---------------------------------------------------------------------------
  {
    const arm = chosetDeadArm();
    const conn = deadArmConnection(arm);
    const Y = deadArmInputs(arm);
    const F = link3Fields(arm);
    let ok = true;
    let worstYes = 0;
    let bestNo = Infinity;
    for (const q of [
      [0.3, 1.1, -0.5],
      [1.0, -1.4, 0.8],
      [-0.6, 2.0, 0.4],
    ]) {
      const tr = isDecoupling(conn, Y, F.translate, q);
      const ro = isDecoupling(conn, Y, F.rotate, q);
      const com = isDecoupling(conn, Y, F.rotateAbout(arm.links[2].com), q);
      const mix = isDecoupling(conn, Y, addFields(F.translate, F.rotate), q);
      ok = ok && tr.decoupling && ro.decoupling && !com.decoupling && !mix.decoupling;
      worstYes = Math.max(worstYes, tr.inSpan, tr.covariantInSpan, ro.inSpan, ro.covariantInSpan);
      bestNo = Math.min(bestNo, Math.max(com.inSpan, com.covariantInSpan), Math.max(mix.inSpan, mix.covariantInSpan));
    }
    // PBWT, Example 12.4.8: V₁ = Y₁, V₂ = Y₂ are decoupling; V₁ + V₂ is not.
    const pd = new PbwtDynamics();
    const pconn = kineticConnection(pd);
    const pY = pbwtInputs();
    const pq = [0.2, -0.3, 0.7];
    const pbwtOk = isDecoupling(pconn, pY, pY[0], pq).decoupling && isDecoupling(pconn, pY, pY[1], pq).decoupling && !isDecoupling(pconn, pY, addFields(pY[0], pY[1]), pq).decoupling;
    const qq = deadArmQuery();
    const drive = decouplingGridSearch(qq);
    const ts = timeScaleDrive(arm, drive.segments, 1);
    const res3 = Math.max(...ts.parts.map((p) => p.joint3Residual));
    const sat = Math.min(...ts.parts.map((p) => p.saturatedFraction));
    push(
      'nonholonomic: Theorem 12.4.7 on Choset\'s Table 12.1 arm with u₃ = 0 — translation along link 3 and rotation about its center of percussion (k = (I₃ + m₃r₃²)/(m₃r₃) = 0.205 m) are decoupling at three configurations, rotation about the COM and the sum of the two are not; the PBWT\'s Y₁, Y₂ are decoupling and Y₁ + Y₂ is not (Example 12.4.8)',
      ok && pbwtOk && worstYes < 5e-10 && Math.abs(percussionDistance(arm) - 0.205) < 1e-12,
      `decoupling residuals ≤ ${worstYes.toExponential(1)}; non-decoupling residuals ≥ ${bestNo.toFixed(3)}; PBWT ${pbwtOk ? 'as printed' : 'MISMATCH'}`,
    );
    push(
      'nonholonomic: the switch-minimizing decoupling drive (four primitives, cost = switches) takes link 3 to its goal in 4 segments (3 switches); Chapter 18\'s time scaling of each rest-to-rest segment needs zero joint-3 torque (|a₃|, |b₃| ≈ 0) and keeps one actuator saturated at every grid node; total time 0.589 s',
      drive.found && drive.switches === 3 && drive.segments.map((g) => `${g.act}${g.amount.toFixed(2)}`).join(' ') === 'T-0.24 R+1.10 T+0.39 R-0.20' && res3 < 1e-6 && sat >= 0.95 && Math.abs(ts.tf - 0.589) < 1e-3,
      `${drive.segments.map((s) => `${s.act}:${s.amount.toFixed(2)}`).join(' ')}; ${drive.expansions} expansions; t_f = ${ts.parts.map((p) => p.tf.toFixed(3)).join(' + ')} = ${ts.tf.toFixed(4)} s; joint-3 residual ${res3.toExponential(1)}; saturated ≥ ${(100 * sat).toFixed(0)} % of nodes`,
    );
  }

  // ---------------------------------------------------------------------------
  // 12. The integration lab: Hitch with trailer parks six ways (seed 21).
  // ---------------------------------------------------------------------------
  {
    const hitched = Hitch.car().withTrailer(2.5);
    const q = { car: hitched, scene, start: { x: 5.5, y: 1.2, theta: 0 }, goal: { x: 9.25, y: 6.4, theta: Math.PI / 2 } };
    const rows = PARK_METHODS.map((m) => planPark(m, q, { seed: 21 }));
    const by = Object.fromEntries(rows.map((r) => [r.method, r]));
    const near = (a: number, b: number) => Math.abs(a - b) < 1e-3;
    const pass =
      rows.every((r) => r.ok) &&
      near(by.rs.score.length, PIN.rs) && by.rs.score.cusps === 1 && !by.rs.score.collides &&
      near(by.flat.score.length, PIN.flat) && by.flat.score.cusps === 0 && by.flat.score.collides && by.flat.steerViolated &&
      near(by.sinusoid.score.length, PIN.sinusoid) && by.sinusoid.score.cusps === 3 && by.sinusoid.score.maxHitch > Math.PI / 2 &&
      near(by.grid.score.length, PIN.grid) && by.grid.score.cusps === 0 && !by.grid.score.collides && by.grid.endError < 0.5 &&
      near(by.hybrid.score.length, PIN.hybrid) && !by.hybrid.score.collides && by.hybrid.endError < 1e-9 && by.hybrid.note.startsWith('1 expansions') &&
      near(by.kino.score.length, PIN.kino) && !by.kino.score.collides && by.kino.endError < 0.8 &&
      // The hitch swings and clearances the scoreboard table prints (two decimals).
      [['rs', 0.62], ['flat', 1.03], ['sinusoid', 2.39], ['grid', 0.63], ['hybrid', 0.62], ['kino', 0.66]].every(([m, h]) => Math.abs(by[m as string].score.maxHitch - (h as number)) < 5e-3) &&
      [['rs', 0.1], ['grid', 0.11], ['hybrid', 0.1], ['kino', 0.13]].every(([m, c]) => Math.abs(by[m as string].score.clearance - (c as number)) < 5e-3) &&
      Math.abs(by.flat.maxSteer - 1.39) < 5e-3;
    push(
      'nonholonomic: integration lab — Hitch with a 2.5 m trailer from (5.5, 1.2, 0) into bay 3 (seed 21), six ways: Reeds–Shepp 7.37 m with one cusp; the trailer-axle flat quintic 9.19 m, smooth but over the steering limit and in collision; sinusoids 27.89 m, three cusps, trailer swung past π/2 (jackknife); Alg. 22 7.20 m into its goal region; hybrid A* 7.37 m exactly onto the goal; forward-propagation RRT 6.85 m, ending inside its 0.8 m / 0.35 rad region',
      pass,
      rows.map((r) => `${r.method}: ${r.score.length.toFixed(4)} m, ${r.score.cusps} cusps, max|θ₁| ${r.score.maxHitch.toFixed(2)}, max|φ| ${r.maxSteer.toFixed(2)}, ${r.score.collides ? 'collides' : `clearance ${r.score.clearance.toFixed(2)}`}, end error ${r.endError.toExponential(1)}`).join('; '),
    );
  }

  return out;
}

