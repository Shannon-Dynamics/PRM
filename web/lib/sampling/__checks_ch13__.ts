/**
 * Chapter 13 checks — the five-node rewire, the radius constants of 2011 and
 * 2020, RRT* against RRT on one seeded sample stream, the informed set, FMT*,
 * BIT*, and PRM*'s expected O(log n) degree.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { EMPTY_PLANE } from './free-space';
import { StraightLine } from './steer';
import { pathIsFree } from './tree';
import {
  Bit,
  Fmt,
  InformedRrtStar,
  InformedSampler,
  PrmStar,
  RrtStar,
  fiveNodeRewire,
  gammaStarFmt,
  gammaStarKf2011,
  gammaStarSolovey2020,
  kN,
  rN,
} from './optimal';
import { APARTMENT_GOAL, APARTMENT_START, discLab, latticeFloor } from './worlds';

const fmt = (x: number, d = 4) => x.toFixed(d);
const close = (a: number, b: number, tol: number) => Math.abs(a - b) <= tol;

export function runCh13Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The five-node rewire (design doc §3)
  // -------------------------------------------------------------------------
  {
    const w = EMPTY_PLANE;
    const r = fiveNodeRewire(w.space, w.free, new StraightLine<number[]>({ kind: 'subdivision', step: 0.25 }));
    const nearOk = r.near.length === 4 && r.near.every((nb) => close(nb.dist, 2.1213, 1e-4));
    const byNode = Object.fromEntries(r.table.map((t) => [t.node, t.viaCost]));
    const tableOk = close(byNode.A, 2.1213, 1e-4) && close(byNode.B, 5.1213, 1e-4) && close(byNode.D, 5.1213, 1e-4) && close(byNode.C, 8.1213, 1e-4);
    const parentOk = r.added === 4 && r.costE !== undefined && close(r.costE, 2.1213, 1e-4);
    const rewireOk = r.rewires.length === 1 && r.rewires[0].node === r.ids.C && close(r.rewires[0].from, 6, 1e-9) && close(r.rewires[0].to, 4.2426, 1e-4) && r.parentC === r.added && close(r.costB, 3, 1e-9) && close(r.costD, 3, 1e-9);
    push(
      'ch13 five-node rewire: Q_near = 4 nodes at 2.1213; parent A (2.1213 < 5.1213 < 8.1213); C rewired 6 → 4.2426, B and D kept',
      nearOk && tableOk && parentOk && rewireOk,
      `table A ${fmt(byNode.A)}, B ${fmt(byNode.B)}, D ${fmt(byNode.D)}, C ${fmt(byNode.C)}; Cost(E) ${fmt(r.costE)}; rewires ${r.rewires.map((x) => `${fmt(x.from)}→${fmt(x.to)}`).join(',')}; drop ${fmt((1 - 4.2426 / 6) * 100, 1)}%`,
    );
  }

  // -------------------------------------------------------------------------
  // Radii
  // -------------------------------------------------------------------------
  {
    const g11 = gammaStarKf2011(2, 1);
    const g20 = gammaStarSolovey2020(2, 1);
    const base = Math.pow(Math.log(1000) / 1000, 0.5);
    const r11 = rN(1000, 2, g11);
    const r20 = rN(1000, 2, g20);
    const k = kN(1000, 2);
    const ok = close(base, 0.08311, 1e-5) && close(g11, 1.382, 5e-4) && close(r11, 0.1149, 5e-5) && close(g20, 0.9772, 5e-4) && close(r20, 0.0812, 5e-5) && k === 29;
    push(
      'ch13 r_1000 in the unit square: (log n/n)^{1/2} = 0.08311; γ*₂₀₁₁ = 1.3820 → 0.1149; γ*₂₀₂₀ = 0.9772 → 0.0812; k_1000 = 29',
      ok,
      `base ${fmt(base, 5)}, γ*₂₀₁₁ ${fmt(g11)}, r ${fmt(r11)}, γ*₂₀₂₀ ${fmt(g20)}, r ${fmt(r20)}, k ${k}, γ*_FMT ${fmt(gammaStarFmt(2, 1))}`,
    );
  }
  {
    // r_n shrinks, n·r_nᵈ grows like log n: the per-iteration work stays O(log n) while the ball keeps being hit.
    let shrinks = true;
    let growsLog = true;
    for (let n = 100; n < 100000; n *= 2) {
      if (!(rN(2 * n, 2, 1) < rN(n, 2, 1))) shrinks = false;
      const a = (n * rN(n, 2, 1) ** 2) / Math.log(n);
      if (!close(a, 1, 1e-9)) growsLog = false;
    }
    push('ch13 r_n = γ(log n/n)^{1/d} decreases in n while n·r_nᵈ = γᵈ log n exactly — the two requirements the rate balances', shrinks && growsLog);
  }

  // -------------------------------------------------------------------------
  // RRT* vs RRT on one sample stream (seed 13)
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const muFree = lab.muBox * 0.8;
    const make = (rewire: boolean) =>
      new RrtStar(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, {
        steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }),
        eta: 0.8,
        goalBias: 0.05,
        goalRadius: 0.8,
        nbhd: { kind: 'radius', gamma: gammaStarSolovey2020(2, muFree) * 1.1 },
        rewire,
      }, new Rng(13));
    const plain = make(false);
    const star = make(true);
    const N = 6000;
    let monotone = true;
    let sameNodes = true;
    let prev = Infinity;
    for (let i = 0; i < N; i++) {
      plain.iterate();
      star.iterate();
      if (star.bestCost > prev + 1e-9) monotone = false;
      prev = star.bestCost;
      if (star.tree.size !== plain.tree.size) sameNodes = false;
    }
    const floor = latticeFloor(lab, APARTMENT_START, APARTMENT_GOAL);
    const starPath = star.bestPath();
    const plainPath = plain.bestPath();
    const nearMean = star.nearSizes.reduce((a, b) => a + b, 0) / Math.max(1, star.nearSizes.length);
    push(
      'ch13 RRT* (seed 13, 6 000 iterations): cost non-increasing, same node set as RRT, final cost below RRT’s and within 5% of the lattice floor, both paths dense-checked free',
      monotone && sameNodes && star.solved && plain.solved && star.bestCost < plain.bestCost && floor !== null && star.bestCost < floor.cost * 1.05 && starPath !== null && pathIsFree(lab.space, lab.free, starPath) && plainPath !== null && pathIsFree(lab.space, lab.free, plainPath),
      `RRT ${fmt(plain.bestCost, 3)} m (first at ${plain.solvedAt}), RRT* ${fmt(star.bestCost, 3)} m, lattice floor ${floor ? fmt(floor.cost, 3) : '—'} m, ${star.rewires} rewires, mean |Q_near| ${fmt(nearMean, 1)} vs log n ${fmt(Math.log(N), 1)}`,
    );
    // The scoreboard the prose prints (deterministic in the seed and the port): pinned to 2 mm.
    push(
      'ch13 seed-13 scoreboard after 6 000 iterations: RRT 13.680 m, RRT* 10.885 m, lattice floor 11.508 m',
      close(plain.bestCost, 13.68, 2e-3) && close(star.bestCost, 10.885, 2e-3) && floor !== null && close(floor.cost, 11.508, 2e-3),
      `RRT ${fmt(plain.bestCost, 3)}, RRT* ${fmt(star.bestCost, 3)}, floor ${floor ? fmt(floor.cost, 3) : '—'}`,
    );
    // RRT never rewires: its incumbent can only change when a *new* node happens to enter the goal
    // region more cheaply (Karaman & Frazzoli's "best path in the tree"), which is rare and bounded
    // away from optimal; RRT*'s incumbent drops through rewiring many times.
    push(
      'ch13 RRT’s cost changes only on a cheaper goal-region entry (≤ 3 times here) and ends ≥ 20% above RRT*; RRT*’s cost history is longer',
      plain.costHistory.length <= 3 && star.costHistory.length > plain.costHistory.length && plain.bestCost > star.bestCost * 1.2,
      `RRT ${plain.costHistory.length} entries ending at ${fmt(plain.bestCost, 3)} m, RRT* ${star.costHistory.length} entries ending at ${fmt(star.bestCost, 3)} m`,
    );
  }

  // -------------------------------------------------------------------------
  // k-nearest neighborhood reaches a comparable cost
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
    const kstar = new RrtStar(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, eta: 0.8, goalBias: 0.05, goalRadius: 0.8, nbhd: { kind: 'kNearest', kStar: Math.E * 1.5 } }, new Rng(13));
    for (let i = 0; i < 6000; i++) kstar.iterate();
    const rad = new RrtStar(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, eta: 0.8, goalBias: 0.05, goalRadius: 0.8, nbhd: { kind: 'radius', gamma: gammaStarSolovey2020(2, lab.muBox * 0.8) * 1.1 } }, new Rng(13));
    for (let i = 0; i < 6000; i++) rad.iterate();
    const ratio = kstar.bestCost / rad.bestCost;
    push('ch13 k-nearest RRT* (k_n = ⌈e(1+1/d) log n⌉) lands within 10% of the radius variant at 6 000 iterations, seed 13', kstar.solved && rad.solved && ratio < 1.1 && ratio > 0.9, `k_n cost ${fmt(kstar.bestCost, 3)}, r_n cost ${fmt(rad.bestCost, 3)}, ratio ${fmt(ratio, 3)}`);
  }

  // -------------------------------------------------------------------------
  // The informed set
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const rng = new Rng(4);
    const s = new InformedSampler(lab.space, APARTMENT_START, APARTMENT_GOAL);
    const cBest = s.cMin * 1.3;
    s.update(cBest);
    // (i) Every point on any path cheaper than c_best lies inside; sample random one-bend paths.
    let inside = 0;
    let outsideButShort = 0;
    let tested = 0;
    for (let t = 0; t < 3000; t++) {
      const x = lab.space.sample(rng);
      const pathCost = lab.space.dist(APARTMENT_START, x) + lab.space.dist(x, APARTMENT_GOAL);
      if (pathCost <= cBest) {
        tested++;
        if (s.contains(x)) inside++;
      } else if (!s.contains(x)) {
        // Outside the set: no path through x can beat c_best (the triangle inequality).
        if (pathCost < cBest) outsideButShort++;
      }
    }
    // (ii) The ellipse sampler never leaves the set, and its acceptance matches μ(X_f̂)/μ(𝒬).
    const free = { isFree: () => true, checks: 0 };
    let drawn = 0;
    let accepted = 0;
    let violations = 0;
    for (let t = 0; t < 4000; t++) {
      drawn++;
      const q = s.sample(lab.space, free, rng);
      if (q) {
        accepted++;
        if (!s.contains(q)) violations++;
      }
    }
    const ratio = s.measureRatio(lab.muBox);
    // Draws are rejected only for leaving the box, so the acceptance rate is the clipped-to-unclipped area ratio.
    const clipFrac = s.measure() / s.measureUnclipped();
    push(
      'ch13 informed set: every sampled shorter path’s bend lies inside X_f̂, nothing outside admits a shorter path, every ellipse draw satisfies ĝ + ĥ ≤ c_best, and the acceptance rate is μ(X_f̂ ∩ 𝒬)/μ(X_f̂)',
      inside === tested && tested > 100 && outsideButShort === 0 && violations === 0 && close(accepted / drawn, clipFrac, 0.02),
      `${inside}/${tested} bends inside, ${violations} violations in ${accepted} ellipse draws, acceptance ${fmt(accepted / drawn, 3)} vs clipped fraction ${fmt(clipFrac, 3)}, μ(X_f̂ ∩ 𝒬)/μ(𝒬) = ${fmt(ratio, 3)} at c_best = 1.3 c_min`,
    );
    // (iii) Monte Carlo area of the ellipse ∩ box against the clipped quadrature, and the unclipped π·a·b.
    let hits = 0;
    const M = 20000;
    for (let t = 0; t < M; t++) if (s.contains(lab.space.sample(rng))) hits++;
    const { a, b } = s.semiAxes();
    push(
      'ch13 informed set measure: Monte Carlo area of X_f̂ ∩ 𝒬 matches the clipped quadrature within 2%, and the unclipped measure is π·a·b',
      close(hits / M, ratio, 0.02 * ratio + 0.003) && close(s.measureUnclipped(), Math.PI * a * b, 1e-9),
      `MC ${fmt(hits / M, 4)}, clipped ${fmt(ratio, 4)}, unclipped π·a·b/μ(𝒬) = ${fmt((Math.PI * a * b) / lab.muBox, 4)}`,
    );
  }
  {
    const lab = discLab();
    const inf = new InformedRrtStar(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), eta: 0.8, goalBias: 0.05, goalRadius: 0.8, nbhd: { kind: 'radius', gamma: gammaStarSolovey2020(2, lab.muBox * 0.8) * 1.1 } }, new Rng(13));
    for (let i = 0; i < 6000; i++) inf.iterate();
    const path = inf.bestPath();
    push('ch13 Informed RRT* (seed 13): solved, sampler’s c_best tracks the incumbent, path dense-checked free', inf.solved && close(inf.informed.cBest, inf.bestCost, 1e-9) && path !== null && pathIsFree(lab.space, lab.free, path), `cost ${fmt(inf.bestCost, 3)} m, μ ratio ${fmt(inf.informed.measureRatio(lab.muBox), 3)}, ${inf.informed.rejectedOutside} draws rejected outside the box`);
  }

  // -------------------------------------------------------------------------
  // FMT* and BIT*
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const checks0 = lab.free.checks;
    const fmtStar = new Fmt(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), n: 1500, gamma: gammaStarFmt(2, lab.muBox * 0.8) * 1.5 }, new Rng(13));
    const sol = fmtStar.run();
    const ok = sol !== null && pathIsFree(lab.space, lab.free, sol.path);
    push('ch13 FMT* (one batch of 1 500 samples, seed 13): path dense-checked free', ok, sol ? `cost ${fmt(sol.cost, 3)} m, r_n ${fmt(fmtStar.r, 3)} m, ${fmtStar.iterations} expansions, ${lab.free.checks - checks0} collision checks` : 'no path');
  }
  {
    const lab = discLab();
    const bit = new Bit(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), batch: 100, gamma: gammaStarSolovey2020(2, lab.muBox * 0.8) * 1.5 }, new Rng(13));
    let monotone = true;
    let prev = Infinity;
    for (let i = 0; i < 6000; i++) {
      bit.step();
      if (bit.bestCost > prev + 1e-9) monotone = false;
      prev = bit.bestCost;
    }
    const sol = bit.best();
    push('ch13 BIT* (batches of 100, seed 13): solved, cost non-increasing across batches, path dense-checked free', sol !== null && monotone && pathIsFree(lab.space, lab.free, sol.path), sol ? `cost ${fmt(sol.cost, 3)} m after ${bit.batches} batches, ${bit.edgeChecks} edge checks, ${bit.costHistory.length} improvements` : 'no path');
  }

  // -------------------------------------------------------------------------
  // PRM*: degree grows like log n
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const ps = new PrmStar(lab.space, lab.free, { steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), nbhd: { kind: 'radius', gamma: gammaStarSolovey2020(2, lab.muBox * 0.8) * 1.1 } }, new Rng(13));
    ps.prm.addNode(APARTMENT_START);
    ps.prm.addNode(APARTMENT_GOAL);
    ps.grow(1500);
    const q = ps.query(APARTMENT_START, APARTMENT_GOAL, 10);
    const meanDeg = (2 * ps.prm.roadmap.edgeCount) / ps.n;
    const r = ps.currentRule().r ?? 0;
    push('ch13 PRM* (r_n ball, 1 500 nodes): connected query, mean degree of order log n, path dense-checked free', q !== null && pathIsFree(lab.space, lab.free, q.path) && meanDeg > 2 && meanDeg < 6 * Math.log(1500), `mean degree ${fmt(meanDeg, 2)} (log n = ${fmt(Math.log(1500), 2)}), r_n ${fmt(r, 3)} m, cost ${q ? fmt(q.cost, 3) : '—'} m`);
  }

  return out;
}
