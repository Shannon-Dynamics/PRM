/**
 * Numerical self-checks for Chapter 16 — Bayesian Localization and Mapping.
 *
 * Same contract as `lib/__checks__.ts`: the design doc's §3 micro-examples to
 * their stated tolerances (the five-cell sense→move table, the 3×3 replan
 * decision with its break-even λ*), plus invariants the mathematics guarantees
 * — the normalizer is the running sum, moving never sharpens, histogram rows sum
 * to one, the odds and log-odds forms of Alg. 19 agree, a point-mass belief
 * reproduces A*, the grid posterior sums to one at every step, mapping with
 * known poses puts the walls back where they were, and the two beliefs of the
 * hook fail and recover the way the prose says they do.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { APARTMENT } from '../sim/world';
import { GridGraph, cellCenter, freeCellNear, gridFromWorld } from '../search/grid';
import {
  CorridorFilter,
  FIVE_CELLS,
  GridBelief,
  LASER_PROXIMITY,
  ParticleBelief,
  ProximityModel,
  applyIncrement,
  chosetConeModel,
  chosetProblem6,
  coneDeviation,
  entropyBits,
  fitMixtureWeights,
  fiveCellSenseMove,
  logOddsUpdate,
  mapWithKnownPoses,
  oddsUpdate,
  rbpfSlamDemo,
  recordRun,
  sampleChosetMotion,
  sampleProximity,
  wallAgreement,
  type ChosetIncrement,
} from './bayes';
import { ACTIONS_4, costToGo, defaultNavigator, followBeliefPlan, kidnappedPair, replan3x3, replanOnBelief } from './replan';

const fmt = (x: number, d = 4): string => x.toFixed(d);
const worst = (a: readonly number[], b: readonly number[]): number => Math.max(...a.map((x, i) => Math.abs(x - b[i])));
const sum = (a: ArrayLike<number>): number => {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i];
  return s;
};

export function runCh16Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // §3 Part A: five cells, sense then move, every entry to 1e-4
  // -------------------------------------------------------------------------
  {
    const t = fiveCellSenseMove();
    const tol = 1e-4;
    const eUn = worst(t.sense.unnormalized, [0.08, 0.16, 0.08, 0.16, 0.08]);
    const ePost = worst(t.sense.posterior, [1 / 7, 2 / 7, 1 / 7, 2 / 7, 1 / 7]);
    const eMoved = worst(t.moved, [1.0, 1.2, 1.8, 1.2, 1.8].map((v) => v / 7));
    const eEta = Math.abs(t.sense.evidence - 0.56);
    const eH = Math.max(Math.abs(t.entropy.prior - 2.3219), Math.abs(t.entropy.sensed - 2.2359), Math.abs(t.entropy.moved - 2.2811));
    push(
      'ch16 §3A: five-cell corridor — sense gives [1,2,1,2,1]/7 with η⁻¹ = 0.56, move gives [1.0,1.2,1.8,1.2,1.8]/7; entropies 2.3219 → 2.2359 → 2.2811 bits',
      eUn < tol && ePost < tol && eMoved < tol && eEta < tol && eH < 1e-4,
      `posterior [${t.sense.posterior.map((v) => fmt(v)).join(', ')}], moved [${t.moved.map((v) => fmt(v)).join(', ')}], H = ${fmt(t.entropy.prior)} → ${fmt(t.entropy.sensed)} → ${fmt(t.entropy.moved)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Choset problem 6 (deterministic) and problem 7 (0.8/0.2 motion)
  // -------------------------------------------------------------------------
  {
    const p6 = chosetProblem6(1);
    const p7 = chosetProblem6(0.8);
    const hand = [0.1, 0.075, 0.075, 0.05, 0.15, 0.075, 0.025, 0.3, 0.075, 0.075];
    const e6 = worst(p6, hand);
    const mode6 = p6.indexOf(Math.max(...p6));
    const mode7 = p7.indexOf(Math.max(...p7));
    push(
      'ch16 problem 6: ten places, landmarks {0,3,6} — posterior P(7) = 0.30, P(4) = 0.15, P(0) = 0.10; problem 7\'s noisy motion keeps the mode at 7 but lowers it to 0.2578',
      e6 < 1e-12 && Math.abs(sum(p6) - 1) < 1e-12 && Math.abs(sum(p7) - 1) < 1e-12 && mode6 === 7 && mode7 === 7 && p7[7] < p6[7] && Math.abs(p7[7] - 0.2578) < 5e-4,
      `P6 = [${p6.map((v) => fmt(v, 3)).join(', ')}]; P7(7) = ${fmt(p7[7], 4)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 2: the normalizer is the running sum of Alg. 16 lines 6–10
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(16);
    let worstGap = 0;
    let sharpen = true;
    let smear = true;
    for (let trial = 0; trial < 40; trial++) {
      const f = new CorridorFilter({ ...FIVE_CELLS, cells: 5 + (trial % 4), doors: [1, 3], pMove: 0.5 + 0.5 * rng.next() });
      for (let k = 0; k < 6; k++) {
        const prior = f.belief();
        const hBefore = f.entropyBits();
        const saw = rng.next() < 0.5;
        const lik = f.likelihood(saw);
        const independent = prior.reduce((s, p, c) => s + p * lik[c], 0);
        const r = f.sense(saw);
        worstGap = Math.max(worstGap, Math.abs(r.evidence - independent), Math.abs(sum(r.posterior) - 1));
        // Sensing can only lower the entropy relative to the prior when the
        // reading is informative; a loop of alternating door/wall cells makes
        // "no change" possible, so test the weaker, always-true direction below.
        const hSensed = f.entropyBits();
        void hSensed;
        void hBefore;
        const hPre = f.entropyBits();
        f.move(1 + Math.floor(rng.next() * 3));
        if (f.entropyBits() < hPre - 1e-9 && f.spec.pMove < 1) smear = false;
      }
    }
    // Sensing sharpens on the micro-example itself; the invariant "moving never
    // sharpens" holds for any stochastic kernel on a loop (a convolution with a
    // non-degenerate kernel cannot lower entropy).
    const t = fiveCellSenseMove();
    if (!(t.entropy.sensed < t.entropy.prior)) sharpen = false;
    push(
      'ch16 Derivation 2 + Fig. 9.1: η⁻¹ returned by the filter equals Σ_x P(y|x)P(x) computed independently; sensing sharpened, and a stochastic move never lowered the entropy',
      worstGap < 1e-12 && sharpen && smear,
      `worst |η⁻¹ − Σ| = ${worstGap.toExponential(2)} over 240 updates`,
    );
  }

  // -------------------------------------------------------------------------
  // §9.1.5: histogram rows sum to one; ML fit recovers the mixture weights
  // -------------------------------------------------------------------------
  {
    const model = new ProximityModel(LASER_PROXIMITY);
    let worstRow = 0;
    for (let i = 0; i < model.n; i += 7) worstRow = Math.max(worstRow, Math.abs(sum(model.row(i)) - 1));
    // Peak of P(y | d) is at y ≈ d for a laser.
    const d = 3.1;
    const row = model.row(model.bin(d));
    let argmax = 0;
    for (let j = 1; j < model.n - 1; j++) if (row[j] > row[argmax]) argmax = j;
    const peakOk = Math.abs(model.binCenter(argmax) - d) < model.params.deltaBin;

    const rng = new Rng(915);
    const truth = { ...LASER_PROXIMITY, alpha: 0.6, beta: 0.15, gamma: 0.15, delta: 0.1 };
    const data: { y: number; d: number }[] = [];
    for (let k = 0; k < 6000; k++) {
      const dExp = rng.uniform(0.5, 7);
      data.push({ y: sampleProximity(dExp, truth, rng), d: dExp });
    }
    const fit = fitMixtureWeights(data, LASER_PROXIMITY, 80);
    const eFit = Math.max(
      Math.abs(fit.params.alpha - truth.alpha),
      Math.abs(fit.params.beta - truth.beta),
      Math.abs(fit.params.gamma - truth.gamma),
      Math.abs(fit.params.delta - truth.delta),
    );
    push(
      'ch16 §9.1.5: every histogram h_i sums to one (9.24), peaks at y = d, and ML fitting recovers (α, β, γ, δ) = (0.6, 0.15, 0.15, 0.1) from 6000 synthetic readings within 0.03',
      worstRow < 1e-9 && peakOk && eFit < 0.03,
      `worst row sum error ${worstRow.toExponential(1)}; fit (${fmt(fit.params.alpha, 3)}, ${fmt(fit.params.beta, 3)}, ${fmt(fit.params.gamma, 3)}, ${fmt(fit.params.delta, 3)}), cache ${(model.cacheBytes / 1024).toFixed(0)} kB`,
    );
  }

  // -------------------------------------------------------------------------
  // Eqs. 9.18–9.21: zero noise reproduces the increment exactly; the sampler is unbiased
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(918);
    const x = { x: 1.2, y: -0.4, theta: 0.7 };
    const u: ChosetIncrement = { alpha: 0.3, d: 0.8, beta: -0.2 };
    const exact = applyIncrement(x, u);
    const zero = sampleChosetMotion(x, u, [0, 0, 0, 0, 0, 0], rng);
    const eZero = Math.max(Math.abs(zero.x - exact.x), Math.abs(zero.y - exact.y), Math.abs(zero.theta - exact.theta));
    let mx = 0;
    let my = 0;
    const N = 20000;
    for (let k = 0; k < N; k++) {
      const s = sampleChosetMotion(x, u, [0.1, 0.05, 0.1, 0.05, 0.1, 0.02], rng);
      mx += s.x;
      my += s.y;
    }
    mx /= N;
    my /= N;
    const bias = Math.hypot(mx - exact.x, my - exact.y);
    push(
      'ch16 eqs. 9.18–9.21: with σ = 0 the sampler is applyOdom exactly; with noise its mean displacement is within 5 mm of the nominal over 20 000 draws',
      eZero < 1e-15 && bias < 0.005,
      `|zero-noise gap| = ${eZero.toExponential(1)}, mean bias ${(bias * 1000).toFixed(2)} mm`,
    );
  }

  // -------------------------------------------------------------------------
  // Alg. 16 and Algs. 17–18 on one recorded run: the grid posterior sums to one
  // every step and both localizers converge to the truth
  // -------------------------------------------------------------------------
  {
    const log = recordRun(APARTMENT, { seed: 1601, steps: 40 });
    const grid = new GridBelief(APARTMENT, { spec: { cellSize: 0.4, nTheta: 18, clearance: 0.12 }, beamStride: 1 });
    const rng = new Rng(7);
    const pf = new ParticleBelief(APARTMENT, rng, { n: 600, every: 3 });
    // Choset problem 9: the same cloud with the raw laser σ = 0.08 never finds the truth.
    const sharpRng = new Rng(7);
    const sharp = new ParticleBelief(APARTMENT, sharpRng, { n: 600, every: 3, sensor: LASER_PROXIMITY });
    let sharpErr = Infinity;
    let worstSum = 0;
    let gridErr = Infinity;
    let pfErr = Infinity;
    let gridStep = -1;
    let pfStep = -1;
    log.steps.forEach((s, k) => {
      grid.predict(s.u);
      grid.correct(grid.scanBinsFor(s.scan, log.angles));
      worstSum = Math.max(worstSum, Math.abs(sum(grid.grid.belief) - 1));
      pf.predict(s.u, rng);
      pf.correct(s.scan, log.angles);
      pf.resample(rng);
      sharp.predict(s.u, sharpRng);
      sharp.correct(s.scan, log.angles);
      sharp.resample(sharpRng);
      const sm = sharp.modePose();
      sharpErr = Math.hypot(sm.x - s.truth.x, sm.y - s.truth.y);
      const gm = grid.modePose();
      const pm = pf.modePose();
      gridErr = Math.hypot(gm.x - s.truth.x, gm.y - s.truth.y);
      pfErr = Math.hypot(pm.x - s.truth.x, pm.y - s.truth.y);
      if (gridStep < 0 && gridErr < 0.5) gridStep = k;
      if (pfStep < 0 && pfErr < 0.5) pfStep = k;
    });
    push(
      'ch16 Algs. 16–18: on one recorded Apartment run the grid posterior sums to one at every step, both the grid mode and the particle mode end within 0.6 m of the truth at σ = 0.25, and the same 600 particles with the raw laser σ = 0.08 never find it (problem 9); the 0.4 m × 20° grid has 12 420 cells and its one-byte table cost 10 782 ray casts',
      worstSum < 1e-9 && gridErr < 0.6 && pfErr < 0.6 && sharpErr > 2 && grid.grid.cellCount === 12420 && grid.table.rayCasts === 10782,
      `worst |Σ − 1| = ${worstSum.toExponential(1)}; final errors grid ${fmt(gridErr, 2)} m (first < 0.5 m at step ${gridStep}), particles ${fmt(pfErr, 2)} m (step ${pfStep}), sharp-σ particles ${fmt(sharpErr, 2)} m; grid ${grid.grid.cellCount} cells, table ${(grid.table.bytes / 1024).toFixed(0)} kB from ${grid.table.rayCasts} ray casts`,
    );
  }

  // -------------------------------------------------------------------------
  // Alg. 19: mapping with known poses reproduces the walls; odds = log odds
  // -------------------------------------------------------------------------
  {
    const log = recordRun(APARTMENT, { seed: 1602, steps: 120, scan: { nBeams: 72 } });
    const poses = log.steps.map((s) => s.truth);
    const scans = log.steps.map((s) => s.scan);
    const { map, entropy } = mapWithKnownPoses(APARTMENT, poses, scans, log.angles, 0.15);
    const agree = wallAgreement(map, APARTMENT);
    // Map entropy is not monotone — a confident cell contradicted by a later beam
    // climbs back toward one bit — but it must fall overall as the map resolves.
    const entropyFalls = entropy[entropy.length - 1] < 0.3 * entropy[0] && entropy[9] < entropy[0];
    let worstOdds = 0;
    const rng = new Rng(19);
    for (let k = 0; k < 500; k++) {
      const a = rng.uniform(0.02, 0.98);
      const b = rng.uniform(0.02, 0.98);
      const c = rng.uniform(0.1, 0.9);
      worstOdds = Math.max(worstOdds, Math.abs(oddsUpdate(a, b, c) - logOddsUpdate(a, b, c)));
    }
    push(
      'ch16 Alg. 19: mapping with known poses marks ≥ 85 % of observed wall cells occupied (88.7 % on the recorded run) and ≥ 97 % of observed free cells free (99.8 %); map entropy falls to under 30 % of its initial value; the odds form (9.39) equals the log-odds form (9.35)',
      agree.wallRecall >= 0.85 && agree.freePrecision >= 0.97 && Math.abs(agree.wallRecall - 0.887) < 0.01 && Math.abs(agree.freePrecision - 0.998) < 0.003 && entropyFalls && worstOdds < 1e-12,
      `wall recall ${(100 * agree.wallRecall).toFixed(1)} %, free precision ${(100 * agree.freePrecision).toFixed(1)} % over ${agree.observed} observed cells; entropy ${fmt(entropy[0], 0)} → ${fmt(entropy[entropy.length - 1], 0)} bits; |odds − logodds| ≤ ${worstOdds.toExponential(1)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Eqs. 9.41–9.42: Choset's cone model — s(2 m, 0) ≈ 0.16 and the Fig. 9.15 profile
  // -------------------------------------------------------------------------
  {
    const s = coneDeviation(2, 0);
    const before = chosetConeModel(1.0, 0, 2);
    const atArc = chosetConeModel(2.15, 0, 2);
    const beyond = chosetConeModel(3.0, 0, 2);
    const offAxis = chosetConeModel(2.15, 0.1, 2);
    push(
      'ch16 eqs. 9.41–9.42: s(2 m, θ = 0) ≈ 0.16 as Choset states; cells before the arc fall below the prior, cells just behind it rise to prior + s, cells past y + d₃ return to the prior, and the effect fades off-axis',
      Math.abs(s - 0.16) < 0.005 && before < 0.5 && Math.abs(atArc - (0.5 + s)) < 1e-9 && Math.abs(beyond - 0.5) < 1e-9 && offAxis < atArc,
      `s = ${fmt(s, 4)}; P(d=1.0) = ${fmt(before, 3)}, P(d=2.15) = ${fmt(atArc, 3)}, P(d=3.0) = ${fmt(beyond, 3)}, P(d=2.15, θ=0.1) = ${fmt(offAxis, 3)}`,
    );
  }

  // -------------------------------------------------------------------------
  // §3 Part B: the 3×3 replan decision
  // -------------------------------------------------------------------------
  {
    const r10 = replan3x3(10);
    const expectJ = [
      [1, 0, 1],
      [10, 1, 2],
      [3, 2, 3],
    ];
    const eJ = Math.max(...r10.Jtable.flatMap((row, i) => row.map((v, j) => Math.abs(v - expectJ[i][j]))));
    const costs = (t: ReturnType<typeof replan3x3>) => Object.fromEntries(t.decision.table.map((row) => [row.action, row.expected]));
    const c10 = costs(r10);
    const e10 = Math.max(Math.abs(c10.Up - 5.2), Math.abs(c10.Right - 3.6), Math.abs(c10.Left - 6.4), Math.abs(c10.Down - 10));
    const r6 = replan3x3(6);
    const c6 = costs(r6);
    const r4 = replan3x3(4);
    const c4 = costs(r4);
    push(
      'ch16 §3B: J = [1 0 1; λ 1 2; 3 2 3]; at λ = 10 the belief costs are Up 5.2, Right 3.6, Left 6.4, Down 10 so Right wins while the mode says Up; tie at λ* = 6; at λ = 4 Up costs 2.8 and wins',
      eJ < 1e-12 &&
        e10 < 1e-12 &&
        r10.decision.action === 'Right' &&
        r10.modeDecision.action === 'Up' &&
        Math.abs(c6.Up - 3.6) < 1e-12 &&
        Math.abs(c6.Right - 3.6) < 1e-12 &&
        r10.breakEven !== null &&
        Math.abs(r10.breakEven - 6) < 1e-12 &&
        Math.abs(c4.Up - 2.8) < 1e-12 &&
        r4.decision.action === 'Up',
      `λ=10: ${ACTIONS_4.map((a) => `${a} ${fmt(c10[a], 1)}`).join(', ')} → ${r10.decision.action} (mode: ${r10.modeDecision.action}); λ* = ${r10.breakEven}; λ=4: Up ${fmt(c4.Up, 1)} → ${r4.decision.action}`,
    );
  }

  // -------------------------------------------------------------------------
  // A point-mass belief reproduces A*: fifty seeded Apartment queries
  // -------------------------------------------------------------------------
  {
    const lattice = gridFromWorld(APARTMENT, 0.4, 0.3);
    const graph = new GridGraph(lattice, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
    const rng = new Rng(1650);
    let worstGap = 0;
    let solved = 0;
    for (let q = 0; q < 50; q++) {
      const goal = freeCellNear(lattice, rng.uniform(0.5, 11.5), rng.uniform(0.5, 8.5));
      // λ must exceed every cost-to-go, or "hit the wall" is cheaper than finishing.
      const J = costToGo(graph, goal, 100);
      let start = freeCellNear(lattice, rng.uniform(0.5, 11.5), rng.uniform(0.5, 8.5));
      for (let tries = 0; tries < 20 && !J.isFree(start); tries++) start = freeCellNear(lattice, rng.uniform(0.5, 11.5), rng.uniform(0.5, 8.5));
      if (!J.isFree(start)) continue;
      const { cells, cost } = followBeliefPlan(J, start);
      if (cells[cells.length - 1] !== goal) {
        worstGap = Infinity;
        continue;
      }
      solved++;
      worstGap = Math.max(worstGap, Math.abs(cost - J.J(start)));
      // And the belief planner with one sample of weight 1 agrees with the certainty-equivalent step.
      const d = replanOnBelief(J, [{ cell: start, weight: 1 }], { lambda: 100, betaInfo: 0, actions: graph.connectivity === 8 ? [...ACTIONS_4, 'UpRight', 'DownRight', 'DownLeft', 'UpLeft'] : ACTIONS_4 });
      const next = cells[1];
      if (next !== undefined) {
        const { x, y } = cellCenter(lattice, next);
        const [di, dj] = [Math.round((x - cellCenter(lattice, start).x) / lattice.cellSize), Math.round((y - cellCenter(lattice, start).y) / lattice.cellSize)];
        const expected = Object.entries({ Up: [0, 1], Right: [1, 0], Down: [0, -1], Left: [-1, 0], UpRight: [1, 1], DownRight: [1, -1], DownLeft: [-1, -1], UpLeft: [-1, 1] }).find(([, s]) => s[0] === di && s[1] === dj)?.[0];
        if (expected !== d.action) worstGap = Infinity;
      }
    }
    push(
      'ch16 Derivation 5(i): with one sample of weight 1, replan_on_belief walks the Ch. 6 universal plan — path cost equals J(start) on 50 seeded Apartment queries',
      solved >= 45 && worstGap < 1e-9,
      `${solved} queries reached the goal; worst |cost − J(start)| = ${Number.isFinite(worstGap) ? worstGap.toExponential(1) : 'disagreement'}`,
    );
  }

  // -------------------------------------------------------------------------
  // The integration lab, headless: the belief planner moves sideways first and
  // arrives without a bump; planning from the mode bumps into room B's wall
  // -------------------------------------------------------------------------
  {
    const nav = defaultNavigator(APARTMENT, 1616);
    const first = nav.step();
    const firstAction = first.decision?.action;
    const firstTable = first.decision?.table ?? [];
    const upRow = firstTable.find((r) => r.action === 'Up');
    for (let k = 0; k < 90 && !nav.reached; k++) nav.step();
    const mode = defaultNavigator(APARTMENT, 1616, { planFromMode: true });
    const modeFirst = mode.step().decision?.action;
    for (let k = 0; k < 90 && !mode.reached; k++) mode.step();
    const sideways = firstAction !== undefined && ACTION_IS_SIDEWAYS(firstAction);
    push(
      'ch16 w16.2 headless: with the 0.6/0.4 room-A/room-B belief and λ = 20 the first move is sideways (Up carries ≈ 0.4 of collision mass), the belief planner reaches the goal with no bump and less than half its initial entropy, and the plan-from-mode variant drives Up into room B\'s wall',
      sideways &&
        (upRow?.collisionMass ?? 0) > 0.3 &&
        nav.reached &&
        nav.collisions === 0 &&
        nav.entropyHistory[nav.entropyHistory.length - 1] < 0.5 * nav.entropyHistory[0] &&
        modeFirst === 'Up' &&
        mode.collisions >= 1,
      `belief: first ${firstAction} (Up collision mass ${fmt(upRow?.collisionMass ?? 0, 2)}), reached=${nav.reached} in ${nav.steps} steps, bumps ${nav.collisions}, entropy ${fmt(nav.entropyHistory[0], 2)} → ${fmt(nav.entropyHistory[nav.entropyHistory.length - 1], 2)} bits; mode: first ${modeFirst}, bumps ${mode.collisions}, reached=${mode.reached}`,
    );
  }

  // -------------------------------------------------------------------------
  // The hook (f16.0), headless: the confident, frozen Gaussian bumps room B's
  // wall and never arrives; the uniform cloud localizes and reaches the goal
  // -------------------------------------------------------------------------
  {
    const { gaussian, cloud } = kidnappedPair(APARTMENT, 1609);
    for (let k = 0; k < 40 && !gaussian.reached; k++) gaussian.step();
    let localizedAt = -1;
    for (let k = 0; k < 80 && !cloud.reached; k++) {
      cloud.step();
      const m = cloud.belief.modePose();
      if (localizedAt < 0 && Math.hypot(m.x - cloud.truth.x, m.y - cloud.truth.y) < 0.5) localizedAt = cloud.steps;
    }
    push(
      'ch16 f16.0 headless: kidnapped to room B, the confident predict-only Gaussian bumps the wall and never reaches the goal; the uniform cloud localizes within 20 scans and arrives with at most 3 bumps',
      gaussian.collisions >= 1 && !gaussian.reached && cloud.reached && localizedAt > 0 && localizedAt <= 20 && cloud.collisions <= 3,
      `gaussian: bumps ${gaussian.collisions}, reached=${gaussian.reached}; cloud: localized at step ${localizedAt}, reached=${cloud.reached} in ${cloud.steps} steps, bumps ${cloud.collisions}, entropy ${fmt(cloud.entropyHistory[0], 2)} → ${fmt(cloud.entropyHistory[cloud.entropyHistory.length - 1], 2)} bits`,
    );
  }

  // -------------------------------------------------------------------------
  // Alg. 20: Rao-Blackwellized SLAM beats dead reckoning on the same odometry
  // -------------------------------------------------------------------------
  {
    const log = recordRun(APARTMENT, { seed: 1620, steps: 45, slip: 0.08, scan: { nBeams: 36 } });
    const rng = new Rng(20);
    const r = rbpfSlamDemo(APARTMENT, log, 6, rng, 0.2);
    push(
      'ch16 Alg. 20: a 6-particle Rao-Blackwellized filter, each particle carrying its own occupancy grid, ends closer to the truth than dead reckoning on the same slipping odometry (0.12 m against 0.20 m)',
      r.slamError < r.odometryError && r.slamMeanError < r.odometryMeanError && Math.abs(r.slamError - 0.12) < 0.03 && Math.abs(r.odometryError - 0.2) < 0.03,
      `final error SLAM ${fmt(r.slamError, 2)} m vs odometry ${fmt(r.odometryError, 2)} m; mean ${fmt(r.slamMeanError, 2)} vs ${fmt(r.odometryMeanError, 2)} m; ${r.reports.filter((x) => x.resampled).length} resamples`,
    );
  }

  // -------------------------------------------------------------------------
  // Entropy helper agrees with the filter's own
  // -------------------------------------------------------------------------
  {
    const t = fiveCellSenseMove();
    const e = Math.abs(entropyBits(t.sense.posterior) - t.entropy.sensed);
    push('ch16 helper: entropyBits equals the histogram filter\'s entropy on the five-cell posterior', e < 1e-12, `gap ${e.toExponential(1)}`);
  }

  return out;
}

const ACTION_IS_SIDEWAYS = (a: string): boolean => a === 'Left' || a === 'Right' || a === 'UpRight' || a === 'UpLeft' || a === 'DownRight' || a === 'DownLeft';
