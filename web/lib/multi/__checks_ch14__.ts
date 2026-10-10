/**
 * Chapter 14 checks — the two-disc micro-example, the composite space's
 * metric, CBS against brute-force joint A*, prioritized planning's failure on
 * the swap instance, the coordination diagram, the manipulation graph's modes,
 * and the forward-propagated Hitch tree's dynamic feasibility.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { checkGeodesic, checkMetricAxioms } from '../manifold/manifold';
import { StraightLine } from '../sampling/steer';
import { RrtConnect } from '../sampling/tree';
import { LOT_GOAL, LOT_START, se2Lab } from '../sampling/worlds';
import { corridor, discsCollide, discsSpace, twoDiscLab, twoDiscWorked } from './composite';
import { coordinationDiagram, monotoneSchedule } from './coordination';
import { Cbs, FOLLOW_INSTANCE, SWAP_INSTANCE, cbs, firstConflict, jointAStar, prioritizedPlan, roomInstance, sumOfCosts } from './cbs';
import { compositePathToTrajectories, planAlone, planPrioritized, trajectoriesConflict } from './prioritized';
import { manipScenario, planIsConsistent, planIsFree } from './manip';
import { KinoRrtPreview, headingMismatch, hitchControls, hitchPropagator } from './propagate';

const fmt = (x: number, d = 4) => x.toFixed(d);
const close = (a: number, b: number, tol: number) => Math.abs(a - b) <= tol;

export function runCh14Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The two-disc micro-example (design doc §3)
  // -------------------------------------------------------------------------
  {
    const w = twoDiscWorked();
    const ok =
      close(w.distQ, 0.8944, 1e-4) &&
      w.collideQ &&
      close(w.penetrationQ, 0.1056, 1e-4) &&
      w.individuallyFreeQ &&
      close(w.distQPrime, 1.077, 1e-4) &&
      !w.collideQPrime &&
      close(w.boundaryX2, 1.9165, 1e-4) &&
      close(w.firstCheckX2, 1.9, 1e-9) &&
      close(w.firstCheckDist, 0.9849, 1e-4) &&
      w.firstCheckRejects &&
      close(w.sliceBlockedFraction, 1, 1e-12);
    push(
      'ch14 two discs r = 0.5: q collides at ‖p₁−p₂‖ = 0.8944 (penetration 0.1056) though each is free alone; q′ free at 1.0770; edge enters 𝒬𝒪₁₂ at x₂ = 1.9165; subdivision midpoint x₂ = 1.9 gives 0.9849 → rejected; robot 2\'s slice at x₂ = 1 fully blocked',
      ok,
      `dist ${fmt(w.distQ)}, pen ${fmt(w.penetrationQ)}, dist′ ${fmt(w.distQPrime)}, boundary ${fmt(w.boundaryX2)}, mid x₂ ${fmt(w.firstCheckX2, 2)} → ${fmt(w.firstCheckDist)}, slice blocked ${fmt(w.sliceBlockedFraction, 2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // MultiBody agrees with the closed form on seeded composite configurations
  // -------------------------------------------------------------------------
  {
    const c = corridor(6, 2);
    const lab = twoDiscLab(c, 0.5);
    const rng = new Rng(14);
    let agree = 0;
    let pairOnly = 0;
    const N = 1000;
    for (let k = 0; k < N; k++) {
      const q = lab.space.sample(rng);
      const alone = q.every((qi) => lab.free.bodies[0].world.isFree(qi));
      const pair = !discsCollide(q[0], 0.5, q[1], 0.5).collide;
      const expected = alone && pair;
      if (lab.free.isFree(q) === expected) agree++;
      if (alone && !pair) pairOnly++;
    }
    const counters = lab.free.pairChecks <= lab.free.worldChecks && lab.free.pairOnlyRejections === pairOnly;
    push(
      'ch14 MultiBody.isFree = (every body free alone) ∧ (no pair overlaps) on 1000 seeded composite configurations; pair checks are only spent after the world checks pass',
      agree === N && counters,
      `agree ${agree}/${N}; world checks ${lab.free.worldChecks}, pair checks ${lab.free.pairChecks}, pair-only rejections ${lab.free.pairOnlyRejections} (${((100 * pairOnly) / N).toFixed(1)}% of samples die only on the pair term)`,
    );
  }

  // -------------------------------------------------------------------------
  // The composite space is a chart with the product metric
  // -------------------------------------------------------------------------
  {
    const space = discsSpace(3, { minX: 0, minY: 0, maxX: 6, maxY: 2 });
    const rng = new Rng(3);
    const m = checkMetricAxioms(space, rng, 1500);
    const g = checkGeodesic(space, rng, 400);
    const dimOk = space.dim === 6 && space.coords(space.sample(rng)).length === 6;
    push(
      'ch14 (ℝ²)³ is a Chart of dimension 6 whose ℓ² product distance satisfies the metric axioms and whose componentwise interpolation is its geodesic',
      dimOk && m.definiteness < 1e-12 && m.symmetry < 1e-12 && m.triangle < 1e-9 && m.nondegeneracy === 0 && g.endpoint < 1e-12 && g.proportional < 1e-9 && g.additivity < 1e-9,
      `dim ${space.dim}; triangle ${m.triangle.toExponential(1)}, proportional ${g.proportional.toExponential(1)}, additivity ${g.additivity.toExponential(1)}`,
    );
  }

  // -------------------------------------------------------------------------
  // CBS is conflict-free and optimal against brute-force joint A*
  // -------------------------------------------------------------------------
  {
    const c = cbs(SWAP_INSTANCE);
    const j = jointAStar(SWAP_INSTANCE);
    const r3 = roomInstance(3);
    const c3 = cbs(r3);
    const j3 = jointAStar(r3, 400000);
    const ok =
      !!c &&
      firstConflict(c.paths) === null &&
      sumOfCosts(c.paths, SWAP_INSTANCE.goals) === c.cost &&
      j.paths !== null &&
      c.cost === j.cost &&
      c.cost === 15 &&
      !!c3 &&
      firstConflict(c3.paths) === null &&
      j3.paths !== null &&
      c3.cost === j3.cost;
    push(
      'ch14 CBS solution is conflict-free and its SIC equals brute-force joint A* (swap corridor: 15; room with 3 agents) — optimal for sum-of-costs on the grid',
      ok,
      `swap: CBS SIC ${c?.cost} in ${c?.expansions} CT expansions (${c?.generated} nodes), joint A* ${j.cost} after ${j.expansions} joint expansions / ${j.generated} successors; room-3: CBS ${c3?.cost} (${c3?.expansions} exp), joint ${j3.cost} (${j3.expansions} exp, ${j3.generated} successors, branching ${j3.branching})`,
    );
  }

  // -------------------------------------------------------------------------
  // Prioritized planning fails on the swap instance in both orders; CBS succeeds
  // -------------------------------------------------------------------------
  {
    const p01 = prioritizedPlan(SWAP_INSTANCE, [0, 1]);
    const p10 = prioritizedPlan(SWAP_INSTANCE, [1, 0]);
    const f01 = prioritizedPlan(FOLLOW_INSTANCE, [0, 1]);
    const f10 = prioritizedPlan(FOLLOW_INSTANCE, [1, 0]);
    const c = cbs(SWAP_INSTANCE);
    const firstPathIsAxis = p01.paths[0] !== null && p01.paths[0]!.length === 7 && p01.paths[0]!.every((cell) => cell < SWAP_INSTANCE.grid.nx);
    const ok = p01.failedAt === 1 && p10.failedAt === 1 && firstPathIsAxis && c !== null && f01.failedAt === null && f10.failedAt === null && firstConflict(f01.paths as number[][]) === null;
    push(
      'ch14 prioritized planning fails on the swap corridor for both priority orders (the first robot takes the axis, the second can never pass) while CBS solves it; on the follow instance both orders succeed',
      ok,
      `swap 1→2 failedAt ${p01.failedAt}, 2→1 failedAt ${p10.failedAt}; first robot's path length ${p01.paths[0]?.length}; CBS SIC ${c?.cost}; follow failedAt ${f01.failedAt}/${f10.failedAt} (SIC ${f01.cost}, ${f10.cost})`,
    );
  }

  // -------------------------------------------------------------------------
  // Coordination diagram: the swap's anti-diagonal band admits no monotone path
  // -------------------------------------------------------------------------
  {
    const c = corridor(6, 1.6, { x: 3, width: 1.6, depth: 1.2 });
    const r = 0.5;
    const pA = planAlone(c, r, [0.8, 0.8], [5.2, 0.8]);
    const pB = planAlone(c, r, [5.2, 0.8], [0.8, 0.8]);
    const pA2 = planAlone(c, r, [0.8, 0.8], [4.0, 0.8]);
    const pB2 = planAlone(c, r, [2.0, 0.8], [5.2, 0.8]);
    let ok = !!pA && !!pB && !!pA2 && !!pB2;
    let detail = '';
    if (ok) {
      const dSwap = coordinationDiagram(pA!, pB!, r, r, 48);
      const dFollow = coordinationDiagram(pA2!, pB2!, r, r, 48);
      const sSwap = monotoneSchedule(dSwap);
      const sFollow = monotoneSchedule(dFollow);
      // Both individual paths are the corridor axis (two waypoints after shortcutting): the obstacle is the band |s₁ + s₂ − 1| small.
      const axis = pA!.length === 2 && pB!.length === 2;
      let band = true;
      for (let j = 0; j < 48; j++) for (let i = 0; i < 48; i++) if (dSwap.occupied[j * 48 + i]) band = band && Math.abs((i + 0.5) / 48 + (j + 0.5) / 48 - 1) < 0.25;
      ok = axis && band && sSwap === null && sFollow !== null;
      detail = `swap: paths are the axis (${pA!.length}, ${pB!.length} waypoints), obstacle ${(100 * dSwap.fraction).toFixed(1)}% of the square, all within |s₁+s₂−1| < 0.25, schedule ${sSwap ? 'found' : 'none'}; follow: obstacle ${(100 * dFollow.fraction).toFixed(1)}%, schedule ${sFollow ? `found (${sFollow.cells.length} cells)` : 'none'}`;
    }
    push('ch14 coordination diagram of the swap instance is an anti-diagonal band with no monotone (0,0) → (1,1) path; the follow instance has one', ok, detail);
  }

  // -------------------------------------------------------------------------
  // Continuous: prioritized RRT in 𝒬 × [0, T] vs. coupled RRT-Connect on (ℝ²)²
  // -------------------------------------------------------------------------
  {
    const c = corridor(6, 1.6, { x: 3, width: 1.6, depth: 1.2 });
    const r = 0.5;
    const lab = twoDiscLab(c, r);
    const s1 = [0.8, 0.8];
    const g1 = [5.2, 0.8];
    const pr = planPrioritized(lab.free.bodies, r, c.bounds, [s1, g1], [g1, s1], [0, 1], new Rng(14));
    const prRev = planPrioritized(lab.free.bodies, r, c.bounds, [s1, g1], [g1, s1], [1, 0], new Rng(14));
    const follow = planPrioritized(lab.free.bodies, r, c.bounds, [s1, [2.0, 0.8]], [[4.0, 0.8], g1], [0, 1], new Rng(14));
    const followOk = follow.failedAt === null && trajectoriesConflict(follow.trajectories as NonNullable<(typeof follow.trajectories)[number]>[], r) === null;
    const rc = new RrtConnect<number[][]>(lab.space, lab.free, [s1, g1], [g1, s1], { steer: new StraightLine<number[][]>({ kind: 'subdivision', step: 0.1 }), eta: 0.6, greed: ['extend', 'connect'] }, new Rng(14));
    const path = rc.merge(6000);
    let coupledOk = false;
    let alcove = false;
    let detail = '';
    if (path) {
      const trs = compositePathToTrajectories(path, 1.0);
      coupledOk = trajectoriesConflict(trs, r) === null;
      // Someone steps into the alcove: some robot's y rises above the corridor's height − r.
      alcove = path.some((q) => q.some((qi) => qi[1] > c.height - r + 1e-9));
      detail = `prioritized 1→2 failedAt ${pr.failedAt}, 2→1 failedAt ${prRev.failedAt} (${pr.iterations + prRev.iterations} iterations); follow ok ${followOk}; coupled RRT-Connect merged at attempt ${rc.mergedAt} with ${rc.nodeCount} nodes, ${lab.free.checks} composite checks of which ${lab.free.pairOnlyRejections} died only on the pair term; alcove used ${alcove}`;
    } else detail = 'coupled planner failed within 6000 attempts';
    push('ch14 in the continuous swap corridor the prioritized planner fails in both orders within budget and succeeds on the follow instance, while coupled RRT-Connect on (ℝ²)² finds a conflict-free plan that uses the alcove', pr.failedAt === 1 && prRev.failedAt === 1 && followOk && coupledOk && alcove, detail);
  }

  // -------------------------------------------------------------------------
  // Manipulation graph: modes alternate, grasps are kept, |𝒢| = 1 disconnects
  // -------------------------------------------------------------------------
  {
    const one = manipScenario(1, true, new Rng(14));
    const twoNoC = manipScenario(2, false, new Rng(14));
    const two = manipScenario(2, true, new Rng(14));
    const plan = two.plan();
    let ok = !one.connected() && !twoNoC.connected() && plan !== null;
    let detail = `|𝒢|=1 connected ${one.connected()} (goal nodes ${one.goalNodes.length}); |𝒢|=2 without C connected ${twoNoC.connected()}; |𝒢|=2 with C: `;
    if (plan) {
      const cons = planIsConsistent(plan, two.pa, two.placements, two.grasps);
      const free = planIsFree(plan, two);
      const kinds = plan.segments.map((s) => s.mode.kind);
      const regrasp = plan.segments.filter((s) => s.mode.kind === 'transfer').map((s) => (s.mode.kind === 'transfer' ? s.mode.grasp : -1));
      ok = ok && cons.alternates && cons.rigid && cons.still && cons.switchesAtPlacements && free && kinds.length === 4 && kinds[0] === 'transit' && regrasp[0] !== regrasp[1] && plan.verified < plan.totalEdges;
      detail += `${kinds.join(' → ')} using grasps ${regrasp.map((g) => two.grasps[g].name).join(' then ')}; alternates ${cons.alternates}, rigid ${cons.rigid}, puck still on transit ${cons.still}, switches at placements ${cons.switchesAtPlacements}, every segment free ${free}; verified ${plan.verified}/${plan.totalEdges} edges lazily (${plan.checks} checks, ${plan.rounds} round)`;
    } else detail += 'no plan';
    push('ch14 manipulation graph: with the front grasp alone MG is disconnected, with two grasps but no spare placement too; with two grasps and placement C the plan is transit → transfer → transit → transfer, every transfer keeps its grasp rigidly, the puck never moves on a transit, and only the path\'s edges were verified', ok, detail);
  }

  // -------------------------------------------------------------------------
  // Hitch preview: forward propagation is re-executable and collision-free
  // -------------------------------------------------------------------------
  {
    const lot = se2Lab();
    const kin = new KinoRrtPreview(lot.space, lot.free, hitchPropagator(lot.car), LOT_START, LOT_GOAL, { controlsPerExtension: 8, dt: 0.5, substeps: 5, goalBias: 0.1, goalRadius: 0.7, sampleControl: hitchControls(lot.car) }, new Rng(14));
    kin.plan(4000);
    let ok = kin.solved;
    let detail = `solved ${kin.solved} at iteration ${kin.solvedAt}, ${kin.tree.size} nodes, ${kin.propagations} propagations (${kin.wasted} wasted)`;
    if (kin.solved) {
      const p = kin.path()!;
      const re = kin.reintegrate()!;
      let worst = 0;
      for (let i = 0; i < p.length; i++) worst = Math.max(worst, Math.hypot(p[i].q.x - re[i].x, p[i].q.y - re[i].y), Math.abs(Math.atan2(Math.sin(p[i].q.theta - re[i].theta), Math.cos(p[i].q.theta - re[i].theta))));
      const dense = kin.densePath()!;
      const freeAll = dense.every((q) => lot.free.isFree(q));
      const mis = headingMismatch(p.map((x) => x.q));
      const steerOk = p.slice(1).every((x) => Math.abs(x.u!.phi) <= lot.car.p.maxSteer + 1e-12);
      ok = worst < 1e-12 && freeAll && mis < 1e-9 && steerOk;
      detail += `; re-integration error ${worst.toExponential(1)}; ${dense.length} simulator states all free ${freeAll}; chord-vs-mean-heading mismatch ${mis.toExponential(1)} (exact arcs); all |φ| ≤ ${lot.car.p.maxSteer}`;
    }
    push('ch14 Hitch without a steer: the forward-propagated RRT reaches the Lot goal within 4000 iterations; re-running the stored controls through Hitch.step reproduces every node exactly, every simulator state is collision-free, and every edge is an exact arc', ok, detail);
  }

  // -------------------------------------------------------------------------
  // Snapshot (seed 14): the integers the prose quotes, pinned
  // -------------------------------------------------------------------------
  {
    const c = cbs(SWAP_INSTANCE)!;
    const j = jointAStar(SWAP_INSTANCE);
    const r3 = roomInstance(3);
    const c3 = cbs(r3)!;
    const j3 = jointAStar(r3, 400000);
    const f = prioritizedPlan(FOLLOW_INSTANCE, [0, 1]);
    const two = manipScenario(2, true, new Rng(14));
    const plan = two.plan();
    const lot = se2Lab();
    const kin = new KinoRrtPreview(lot.space, lot.free, hitchPropagator(lot.car), LOT_START, LOT_GOAL, { controlsPerExtension: 8, dt: 0.5, substeps: 5, goalBias: 0.1, goalRadius: 0.7, sampleControl: hitchControls(lot.car) }, new Rng(14));
    kin.plan(4000);
    const cor = corridor(6, 1.6, { x: 3, width: 1.6, depth: 1.2 });
    const lab = twoDiscLab(cor, 0.5);
    const rc = new RrtConnect<number[][]>(lab.space, lab.free, [[0.8, 0.8], [5.2, 0.8]], [[5.2, 0.8], [0.8, 0.8]], { steer: new StraightLine<number[][]>({ kind: 'subdivision', step: 0.1 }), eta: 0.6, greed: ['extend', 'connect'] }, new Rng(14));
    rc.merge(6000);
    const pA = planAlone(cor, 0.5, [0.8, 0.8], [5.2, 0.8])!;
    const pB = planAlone(cor, 0.5, [5.2, 0.8], [0.8, 0.8])!;
    const d = coordinationDiagram(pA, pB, 0.5, 0.5, 48);
    const tree = new Cbs(SWAP_INSTANCE);
    tree.solve();
    const root = tree.nodes[0];
    const rootOk = root.cost === 12 && root.conflict !== null && root.conflict.edge === null && root.conflict.t === 3 && root.conflict.cell === 3 && tree.nodes[1].cost === 13 && tree.nodes[1].conflict?.edge !== null && tree.nodes[1].conflict?.t === 4;
    const solPaths = tree.solution!.paths;
    const alcoveUsed = solPaths.some((path) => path.includes(SWAP_INSTANCE.grid.nx + 3));
    const ok =
      rootOk && c.depth === 4 && alcoveUsed && solPaths.map((pth) => pth.length - 1).sort().join(',') === '7,8' && (kin.path()?.length ?? 0) === 14 &&
      c.expansions === 12 && c.generated === 23 && j.expansions === 19 && j.generated === 138 &&
      c3.expansions === 2 && j3.expansions === 27 && j3.generated === 1412 &&
      f.cost === 10 &&
      plan !== null && plan.verified === 4 && plan.totalEdges === 7 && plan.checks === 218 &&
      kin.solvedAt === 63 && kin.tree.size === 63 && kin.propagations === 504 && kin.wasted === 140 && (kin.densePath()?.length ?? 0) === 66 &&
      rc.mergedAt === 2562 && rc.nodeCount === 60 && lab.free.checks === 3278 && lab.free.pairOnlyRejections === 199 &&
      close(d.fraction, 0.3898, 5e-4);
    push(
      'ch14 snapshot (seed 14): CBS swap root SIC 12 with a vertex conflict at (3,0), t = 3, children 13 with edge conflicts at t = 4, solution at depth 4 using the alcove with steps 7 + 8; 12 expansions / 23 nodes vs joint A* 19 / 138; room-3 CBS 2 vs joint 27 / 1412; follow SIC 10; manipulation 4 of 7 edges, 218 checks; Hitch solved at 63 with 63 nodes, 504 propagations, 140 wasted, 66 states; coupled RRT-Connect at 2562 with 60 nodes, 3278 checks, 199 pair-only; diagram obstacle 39.0%',
      ok,
      `cbs ${c.expansions}/${c.generated}, joint ${j.expansions}/${j.generated}, room3 ${c3.expansions} vs ${j3.expansions}/${j3.generated}, follow ${f.cost}, manip ${plan?.verified}/${plan?.totalEdges}/${plan?.checks}, hitch depth ${c.depth} root ${root.cost}; hitch path ${kin.path()?.length} nodes, ${kin.solvedAt}/${kin.tree.size}/${kin.propagations}/${kin.wasted}/${kin.densePath()?.length}, coupled ${rc.mergedAt}/${rc.nodeCount}/${lab.free.checks}/${lab.free.pairOnlyRejections}, diagram ${(100 * d.fraction).toFixed(1)}%`,
    );
  }

  return out;
}
