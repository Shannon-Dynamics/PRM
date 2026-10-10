/**
 * Chapter 22 checks — the two §3 micro-examples, backprop against finite
 * differences, the MLP fitting XOR and a 1-D function under a fixed seed, the
 * mixture floor (analytic and empirical), the learned sampler's aim on fresh
 * worlds, the ε audit and Derivation 3's bound on the audit set, Derivation 3
 * as a property test, the proposal + verifier never returning a colliding
 * path, the toy diffuser's verifier, and the recorded Board's invariants.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { Mlp, gradientCheck, mseLoss, trainMlp } from './mlp';
import { CoarseHistogram, fourCellExample, klDivergence } from './info';
import { threeNodeExample } from './heuristic';
import { CELLS, LearnedSampler, cellOfConfig, mixtureMasses, mixtureSampler, uniformDraw } from './sampler';
import { HeuristicNet, aStarEps, latticeOf, measureEpsilon, randomFreeCell, withinEpsBound } from './heuristic';
import { costToGo, torusOctile } from './torus-grid';
import { MpnetLike } from './neural-planner';
import { DIFF_K, denoise, linearSchedule, trajectoryValid } from './diffusion';
import { pathCellHistogram, worldCases } from './data';
import { randomQuery } from './worlds';
import { PLANNERS, runQuery, successCurve, summarize } from './bench';
import { BENCH, EPSILON_VALIDATION, TRAINING_META, loadModels } from './models';

const fmt = (x: number, d = 4) => (Number.isFinite(x) ? x.toFixed(d) : String(x));

export function runCh22Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });
  const models = loadModels();

  // -------------------------------------------------------------------------
  // §3(a): KL between a learned and a uniform sampler on four cells
  // -------------------------------------------------------------------------
  {
    const r = fourCellExample(0.1);
    const identity = Math.abs(r.klTargetUniform - r.entropyIdentity) < 1e-12;
    const hat = new CoarseHistogram([100, 60, 30, 10]);
    const identityHat = Math.abs(hat.klToUniform() - (Math.log(4) - hat.entropy())) < 1e-12;
    const mixOk = r.mixture.every((m, i) => Math.abs(m - [0.475, 0.295, 0.16, 0.07][i]) < 1e-12) && r.mixtureMin >= r.mixtureFloor;
    push(
      'ch22 §3(a) four-cell KL: D(p̂‖u) = 0.2442 nats (0.352 bits), D(p*‖p̂) = 0.0119, D(p*‖u) = 0.3162 = ln 4 − H(p*); mixture floor at λ = 0.1',
      Math.abs(r.klHatUniformNats - 0.2442) < 1e-4 &&
        Math.abs(r.klHatUniformBits - 0.352) < 1e-3 &&
        Math.abs(r.klTargetHat - 0.0119) < 1e-4 &&
        Math.abs(r.klTargetUniform - 0.3162) < 1e-4 &&
        identity &&
        identityHat &&
        mixOk,
      `D(p̂‖u) = ${fmt(r.klHatUniformNats)} nats = ${fmt(r.klHatUniformBits, 3)} bits; D(p*‖p̂) = ${fmt(r.klTargetHat)}; D(p*‖u) = ${fmt(r.klTargetUniform)} (ln 4 − H(p*) = ${fmt(r.entropyIdentity)}); ${fmt(r.aimRatio, 1)}× better aimed; p_λ = (${r.mixture.map((m) => m.toFixed(3)).join(', ')}), min ${fmt(r.mixtureMin, 3)} ≥ λ/4 = ${fmt(r.mixtureFloor, 3)}`,
    );
  }

  // -------------------------------------------------------------------------
  // §3(b): admissibility of a learned heuristic on three nodes
  // -------------------------------------------------------------------------
  {
    const a = threeNodeExample();
    push(
      'ch22 §3(b) three-node audit: violated {b}, inconsistent at b, ε = 0.1333, bound (1 + ε)·5 = 5.667',
      a.violated.join('') === 'b' && a.inconsistent.join('') === 'b' && Math.abs(a.epsilon - 0.13333) < 1e-4 && Math.abs(a.bound - 5.6667) < 1e-3,
      `ratios (${a.ratios.map((r) => r.toFixed(4)).join(', ')}), violated {${a.violated.join(', ')}}, inconsistent {${a.inconsistent.join(', ')}}, ε = ${fmt(a.epsilon)}, bound ${fmt(a.bound, 3)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Backprop agrees with central differences
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(5);
    const worst = Math.max(
      gradientCheck(new Mlp([3, 7, 5, 2], 'tanh', 'linear', rng.fork()), [0.3, -0.8, 1.1], [0.5, -0.25]),
      gradientCheck(new Mlp([4, 6, 1], 'sigmoid', 'sigmoid', rng.fork()), [0.1, 0.2, -0.3, 0.9], [0.7]),
      gradientCheck(new Mlp([2, 9, 3], 'softplus', 'tanh', rng.fork()), [-0.4, 0.6], [0.1, -0.2, 0.3]),
    );
    push('ch22 MLP backprop matches finite differences (tanh, sigmoid, softplus nets) to 1e-6 relative', worst < 1e-6, `worst relative discrepancy ${worst.toExponential(2)}`);
  }

  // -------------------------------------------------------------------------
  // The MLP fits XOR and a 1-D function under a fixed seed
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(22);
    const xor = new Mlp([2, 8, 1], 'tanh', 'linear', rng.fork());
    const data = [
      { x: [0, 0], y: [0] },
      { x: [0, 1], y: [1] },
      { x: [1, 0], y: [1] },
      { x: [1, 1], y: [0] },
    ];
    trainMlp(xor, data, { epochs: 600, batch: 4, lr: 0.03, loss: 'mse', rng: rng.fork() });
    const xorErr = Math.max(...data.map((d) => Math.abs(xor.forward(d.x)[0] - d.y[0])));

    const f = (x: number) => Math.sin(2 * Math.PI * x) * 0.8 + 0.3 * x;
    const pts = Array.from({ length: 64 }, (_, i) => ({ x: [i / 63], y: [f(i / 63)] }));
    const net = new Mlp([1, 32, 1], 'tanh', 'linear', new Rng(22));
    const hist = trainMlp(net, pts, { epochs: 600, batch: 8, lr: 0.02, decay: 0.997, loss: 'mse', rng: new Rng(23) });
    let mse = 0;
    for (const p of pts) mse += mseLoss(net.forward(p.x), p.y).loss;
    mse /= pts.length;
    push(
      'ch22 MLP fits XOR (max error < 0.1) and sin(2πx)·0.8 + 0.3x on 64 points (MSE < 2e-3), seed 22',
      xorErr < 0.1 && mse < 2e-3,
      `XOR max |error| ${fmt(xorErr, 3)}; 1-D fit MSE ${mse.toExponential(2)} after ${hist.length} epochs (first ${hist[0].toExponential(2)})`,
    );
  }

  // -------------------------------------------------------------------------
  // The mixture floor: p_λ(cell) ≥ λ/N for every cell, analytic and empirical
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(31);
    const cases = [...worldCases(6000, 12, 'train'), ...worldCases(21000, 12, 'heldout')];
    let worst = Infinity;
    let allHold = true;
    let rawZeroCells = 0;
    for (const wc of cases) {
      const q = randomQuery(wc.lab, rng);
      if (!q) continue;
      const ls = new LearnedSampler(models.sampler, wc.enc, q[0], q[1]);
      for (const lambda of [0.05, 0.1, 0.3]) {
        const m = mixtureMasses(ls, lambda);
        worst = Math.min(worst, m.min / m.floor);
        if (!m.holds) allHold = false;
      }
      // How starved can the raw proposal leave a cell? (< 1/1000 of uniform counts as "effectively zero".)
      for (let c = 0; c < CELLS; c++) if (ls.probs[c] * CELLS < 1e-3) rawZeroCells++;
    }
    // Empirical: 100 000 raw draws of the mixture on one world; every cell must be hit at least half its floor.
    const wc = cases[0];
    const q = randomQuery(wc.lab, new Rng(2))!;
    const ls = new LearnedSampler(models.sampler, wc.enc, q[0], q[1]);
    const lambda = 0.1;
    const draws = 100000;
    const counts = new Array<number>(CELLS).fill(0);
    const drawRng = new Rng(77);
    for (let k = 0; k < draws; k++) {
      const qd = drawRng.next() < lambda ? uniformDraw(drawRng) : ls.drawRaw(drawRng);
      counts[cellOfConfig(qd)]++;
    }
    const floorHits = (lambda / CELLS) * draws;
    const minHits = Math.min(...counts);
    // Every cell within 5σ of its mixture expectation (Poisson), so a starved raw cell still shows the floor.
    const expected = mixtureMasses(ls, lambda).masses;
    const within5Sigma = counts.every((c, i) => c >= expected[i] * draws - 5 * Math.sqrt(expected[i] * draws));
    let threw = false;
    try {
      mixtureSampler(ls, 0);
    } catch {
      threw = true;
    }
    push(
      'ch22 mixture floor: min_c p_λ(c) ≥ λ/N on 24 worlds × 3 λ (analytic); 100k draws at λ = 0.1 hit every cell ≥ ½ its floor and within 5σ of p_λ; λ = 0 is rejected',
      allHold && worst >= 1 - 1e-9 && minHits >= 0.5 * floorHits && within5Sigma && threw,
      `min over cells of p_λ(c)/(λ/N) = ${fmt(worst, 3)}; raw proposal leaves ${rawZeroCells} of ${cases.length * CELLS} cells below 10⁻³·u; emptiest cell got ${minHits} hits vs floor ${floorHits.toFixed(0)}; mixtureSampler(·, 0) threw: ${threw}`,
    );
  }

  // -------------------------------------------------------------------------
  // Aim: on fresh training-distribution worlds the learned proposal is closer to p* than uniform is
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(41);
    const cases = worldCases(8000, 12, 'train');
    const held = worldCases(23000, 12, 'heldout');
    const aim = (list: typeof cases) => {
      let better = 0;
      let total = 0;
      let sumHat = 0;
      let sumUni = 0;
      for (const wc of list) {
        const q = randomQuery(wc.lab, rng);
        if (!q) continue;
        const pStar = pathCellHistogram(wc, q[0], q[1], rng.fork());
        if (!pStar) continue;
        const ls = new LearnedSampler(models.sampler, wc.enc, q[0], q[1]);
        const target = pStar.probabilities();
        const klHat = klDivergence(target, mixtureMasses(ls, 0.1).masses);
        const klUni = klDivergence(target, new Array(CELLS).fill(1 / CELLS));
        total++;
        sumHat += klHat;
        sumUni += klUni;
        if (klHat < klUni) better++;
      }
      return { better, total, meanHat: sumHat / total, meanUni: sumUni / total };
    };
    const a = aim(cases);
    const h = aim(held);
    push(
      'ch22 aim: D(p*‖p_λ) < D(p*‖u) on average and on a majority of fresh in-distribution worlds (λ = 0.1, one fresh PRM path per world as p*)',
      a.total >= 6 && a.better > 0.5 * a.total && a.meanHat < a.meanUni,
      `in-distribution: better on ${a.better}/${a.total}, mean D(p*‖p_λ) ${fmt(a.meanHat, 3)} vs D(p*‖u) ${fmt(a.meanUni, 3)}; held-out bars: better on ${h.better}/${h.total}, ${fmt(h.meanHat, 3)} vs ${fmt(h.meanUni, 3)}`,
    );
  }

  // -------------------------------------------------------------------------
  // ε audit: measure ε on the audit set, then Derivation 3's bound holds there
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(51);
    const audit = worldCases(5000, 8, 'train');
    const rep = measureEpsilon(models.heuristic, audit, 2, new Rng(99));
    let worstRatio = 0;
    let searches = 0;
    let boundOk = true;
    let octileOk = true;
    let expLearned = 0;
    let expOctile = 0;
    let reopened = 0;
    for (const wc of audit) {
      const g = latticeOf(wc.lab);
      const goalRng = new Rng(99);
      for (let k = 0; k < 2; k++) {
        const goal = randomFreeCell(g, goalRng);
        if (goal < 0) continue;
        const hStar = costToGo(g, goal);
        const hn = new HeuristicNet(models.heuristic, wc.enc, g, goal);
        const oct = torusOctile(g, goal);
        for (let s = 0; s < 3; s++) {
          const start = randomFreeCell(g, rng);
          if (start < 0 || !Number.isFinite(hStar[start]) || start === goal) continue;
          const cStar = hStar[start];
          const learned = aStarEps(g, start, goal, hn.h);
          const octile = aStarEps(g, start, goal, oct);
          searches++;
          expLearned += learned.expansions;
          expOctile += octile.expansions;
          reopened += learned.reopened;
          worstRatio = Math.max(worstRatio, learned.cost / cStar);
          if (!withinEpsBound(learned.cost, cStar, rep.epsilon)) boundOk = false;
          if (Math.abs(octile.cost - cStar) > 1e-9) octileOk = false;
        }
      }
    }
    push(
      'ch22 ε audit: on 8 audit worlds × 2 goals the learned ĥ has measured ε; every A* run there returns cost ≤ (1 + ε)C*; octile A* equals Dijkstra',
      boundOk && octileOk && searches >= 20 && rep.epsilon > 0,
      `ε_audit = ${fmt(rep.epsilon)} (${rep.violations}/${rep.audited} nodes over; mean ĥ/h* ${fmt(rep.meanRatio, 3)} vs octile ${fmt(rep.meanOctileRatio, 3)}); ${searches} searches, worst realized C/C* = ${fmt(worstRatio)} ≤ ${fmt(1 + rep.epsilon)}; expansions learned/octile = ${fmt(expLearned / Math.max(1, expOctile), 3)}, ${reopened} reopenings; shipped ε_val = ${fmt(EPSILON_VALIDATION)} (${TRAINING_META.epsilon.validation.worlds} worlds)`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 3 as a property test: ĥ = h*(1 + ε r), r ∈ [0, 1] random
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(61);
    const cases = worldCases(9000, 10, 'train');
    let runs = 0;
    let ok = true;
    let worst = 0;
    let anyReopen = 0;
    const eps = 0.3;
    for (const wc of cases) {
      const g = latticeOf(wc.lab);
      for (let k = 0; k < 5; k++) {
        const goal = randomFreeCell(g, rng);
        const start = randomFreeCell(g, rng);
        if (goal < 0 || start < 0 || start === goal) continue;
        const hStar = costToGo(g, goal);
        if (!Number.isFinite(hStar[start])) continue;
        const inflate = new Float64Array(g.size);
        for (let c = 0; c < g.size; c++) inflate[c] = 1 + eps * rng.next();
        const h = (c: number) => (Number.isFinite(hStar[c]) ? hStar[c] * inflate[c] : 0);
        const t = aStarEps(g, start, goal, h);
        runs++;
        anyReopen += t.reopened;
        worst = Math.max(worst, t.cost / hStar[start]);
        if (!withinEpsBound(t.cost, hStar[start], eps)) ok = false;
      }
    }
    push(
      'ch22 Derivation 3 property test: random ε-inflated heuristics (ε = 0.3) on 50 lattice queries return cost ≤ 1.3 C* with reopening',
      ok && runs >= 30,
      `${runs} searches, worst C/C* = ${fmt(worst)}, ${anyReopen} reopenings in total`,
    );
  }

  // -------------------------------------------------------------------------
  // Proposal + verifier: every returned path is collision-free under a dense recheck
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(71);
    const cases = [...worldCases(10000, 8, 'train'), ...worldCases(24000, 8, 'heldout')];
    let planned = 0;
    let returned = 0;
    let verified = 0;
    let rawOk = { train: 0, heldout: 0 };
    let rawTotal = { train: 0, heldout: 0 };
    let endpointsOk = true;
    let fellBack = 0;
    for (const wc of cases) {
      const q = randomQuery(wc.lab, rng);
      if (!q) continue;
      const planner = new MpnetLike(models.policy, wc, { repairAttempts: 300, fallbackAttempts: 1500 });
      const raw = planner.plan(q[0], q[1], rng.fork(), { repair: false });
      rawTotal[wc.gen]++;
      if (raw.path) rawOk[wc.gen]++;
      const plan = planner.plan(q[0], q[1], rng.fork());
      planned++;
      if (plan.fellBack) fellBack++;
      if (plan.path) {
        returned++;
        if (planner.verify(plan.path, 0.005)) verified++;
        const p = plan.path;
        if (!wc.lab.space.equals(p[0], q[0], 1e-9) || !wc.lab.space.equals(p[p.length - 1], q[1], 1e-9)) endpointsOk = false;
      }
    }
    push(
      'ch22 proposal + verifier: every path the MPNet-style planner returned passes an independent dense recheck (step 0.005) and joins the query endpoints',
      planned >= 10 && returned === verified && endpointsOk,
      `${planned} queries (8 in-distribution, 8 held-out): ${returned} paths returned, ${verified} verified, ${fellBack} fell back to RRT-Connect on the whole query; net alone (no repair) solved ${rawOk.train}/${rawTotal.train} in-distribution and ${rawOk.heldout}/${rawTotal.heldout} held-out`,
    );
  }

  // -------------------------------------------------------------------------
  // The toy diffuser: schedule destroys structure; samples are plausible, verified, bimodal
  // -------------------------------------------------------------------------
  {
    const sch = linearSchedule();
    const run = denoise(models.denoiser, new Rng(3), 64);
    const guided = denoise(models.denoiser, new Rng(3), 64, { guidance: { smooth: 0.5, obstacle: 4.0, margin: 0.42 } });
    const noiseIsNoise = sch.alphaBar[DIFF_K] < 0.01;
    const bimodal = run.above > 0 && run.below > 0;
    const finalMatchesVerifier = run.steps[DIFF_K].every((t, i) => trajectoryValid(t) === run.valid[i]);
    push(
      'ch22 toy diffuser: ᾱ_K < 0.01; 64 denoised trajectories are ≥ 50% valid, go both ways around the slab, and the recorded validity is the hard checker\'s',
      noiseIsNoise && run.validFraction >= 0.5 && bimodal && finalMatchesVerifier && run.steps.length === DIFF_K + 1,
      `ᾱ_K = ${sch.alphaBar[DIFF_K].toExponential(2)}; plain: ${(100 * run.validFraction).toFixed(0)}% valid (${run.above} above, ${run.below} below); guided: ${(100 * guided.validFraction).toFixed(0)}% valid; training loss ${TRAINING_META.finalLoss.denoiser}`,
    );
  }

  // -------------------------------------------------------------------------
  // The recorded Board: ratios ≥ 1, success monotone in budget, and a live run obeys the same rules
  // -------------------------------------------------------------------------
  {
    let ratiosOk = true;
    let monotoneOk = true;
    let sizesOk = true;
    for (const suite of [BENCH.suites.train, BENCH.suites.heldout]) {
      if (suite.queries.length !== BENCH.meta.worldsPerSuite) sizesOk = false;
      for (const q of suite.queries) {
        for (const r of q.runs) if (r.solved && r.cost < q.incumbent - 1e-6) ratiosOk = false;
      }
      for (const p of PLANNERS) {
        const curve = successCurve(suite, p.name, BENCH.budgets);
        for (let i = 1; i < curve.length; i++) if (curve[i].y < curve[i - 1].y - 1e-12) monotoneOk = false;
      }
    }
    // Live: one solvable world, two planners, same seed — the harness's own invariants.
    const wc = worldCases(70000, 1, 'train')[0];
    const q = randomQuery(wc.lab, new Rng(wc.seed ^ 0x5bd1e995))!;
    const live = runQuery(wc, q[0], q[1], { sampler: models.sampler, policy: models.policy }, 0.1, 6000, wc.seed, ['uniform', 'learned', 'rrt-connect']);
    const liveOk = live.runs.every((r) => r.checks <= 6000 && (!r.solved || r.cost >= live.incumbent - 1e-6));
    const rows = summarize(BENCH.suites.train, 2000);
    const learned = rows.find((r) => r.planner === 'learned')!;
    const uniform = rows.find((r) => r.planner === 'uniform')!;
    push(
      'ch22 Board invariants: every solved cost ≥ its query\'s incumbent, success monotone in budget for all 7 planners × 2 suites, suite sizes as recorded; a live run obeys the budget',
      ratiosOk && monotoneOk && sizesOk && liveOk,
      `${BENCH.meta.worldsPerSuite} queries per suite, budgets {${BENCH.budgets.join(', ')}}; at 2000 checks in-distribution: learned ${(100 * learned.success).toFixed(0)}% vs uniform ${(100 * uniform.success).toFixed(0)}%; live run: ${live.runs.map((r) => `${r.planner} ${r.solved ? `@${r.checks}` : 'x'}`).join(', ')}`,
    );
  }

  return out;
}
