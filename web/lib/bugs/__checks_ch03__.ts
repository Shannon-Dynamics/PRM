/**
 * Numerical self-checks for Chapter 3 — Bug algorithms.
 *
 * Same contract as `lib/__checks__.ts`: invariants the mathematics guarantees,
 * reproduced to a stated tolerance, not frozen outputs — except where a
 * number is printed in the chapter as a worked example, which is then pinned
 * here so prose and code cannot drift apart.
 *
 *     import { runCh03Checks } from './bugs/__checks_ch03__';
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { APARTMENT, distanceToWalls, type Point2 } from '../sim/world';
import { Bug1 } from './bug1';
import { Bug2 } from './bug2';
import { offsetContact, polygonContact } from './follow';
import { dist, lineCrossings, polygonPerimeter, witness } from './geometry';
import { bug1Bound, bug2Bound } from './planner';
import { ENCLOSED, FIG_2_8, KIDNEY, SPIRAL, TWO_RECTANGLES, randomPreset } from './presets';
import {
  continuityIntervals,
  distanceFromScan,
  exactSilhouettePoints,
  gradientFromScan,
  intervalEndpoints,
  rangeScan,
} from './scan';
import { TangentBug } from './tangent';
import { traceCurve } from './trace';

const fmt = (x: number): string => (Math.abs(x) < 1e-4 && x !== 0 ? x.toExponential(2) : x.toFixed(4));
const near = (a: Point2, b: Point2, tol: number) => dist(a, b) <= tol;

/** The chapter's lab query in the Apartment: room A to the bedroom, W* = 0.11. */
export const APARTMENT_QUERY = { start: { x: 1, y: 1 }, goal: { x: 11, y: 8 }, clearance: 0.11, step: 0.05 };

export function runCh03Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  const W = TWO_RECTANGLES;
  const perims = W.world.obstacles.map(polygonPerimeter);
  const contact = polygonContact(W.world);

  // -------------------------------------------------------------------------
  // The §3 micro-example
  // -------------------------------------------------------------------------
  {
    const s = new Bug1(contact, W.start, W.goal, { step: 0.05, turn: 'left' }).run();
    const bound = bug1Bound(W.start, W.goal, perims);
    const ok =
      s.done &&
      Math.abs(s.length - 34) < 1e-6 &&
      Math.abs(bound - 37) < 1e-12 &&
      s.hits.length === 2 &&
      near(s.hits[0], { x: 2, y: 0 }, 1e-9) &&
      near(s.hits[1], { x: 6, y: 0 }, 1e-9) &&
      s.leaves.length === 2 &&
      near(s.leaves[0], { x: 4, y: 0 }, 1e-9) &&
      near(s.leaves[1], { x: 7, y: 0 }, 1e-9);
    push(
      'bug1: two rectangles — L = 34 exactly, bound d + 1.5Σp = 37, hits (2,0),(6,0), leaves (4,0),(7,0)',
      ok,
      `L = ${fmt(s.length)}, bound ${fmt(bound)}, p = [${perims.join(', ')}]`,
    );
  }
  {
    const s = new Bug2(contact, W.start, W.goal, { step: 0.05, turn: 'left' }).run();
    const n = W.world.obstacles.map((o) => lineCrossings(o, W.start, W.goal));
    const bound = bug2Bound(W.start, W.goal, W.world.obstacles, perims);
    const ok =
      s.done &&
      Math.abs(s.length - 16) < 1e-6 &&
      Math.abs(bound - 28) < 1e-12 &&
      n.every((k) => k === 2) &&
      near(s.leaves[0], { x: 4, y: 0 }, 1e-9) &&
      near(s.leaves[1], { x: 7, y: 0 }, 1e-9);
    push(
      'bug2: two rectangles — L = 16 exactly, n_i = 2, bound d + ½Σ n_i p_i = 28',
      ok,
      `L = ${fmt(s.length)}, bound ${fmt(bound)}, n = [${n.join(', ')}]`,
    );
  }

  // -------------------------------------------------------------------------
  // Greedy wins on simple obstacles and loses on the spiral — within both bounds
  // -------------------------------------------------------------------------
  {
    const simple1 = new Bug1(contact, W.start, W.goal, { step: 0.05, turn: 'left' }).run().length;
    const simple2 = new Bug2(contact, W.start, W.goal, { step: 0.05, turn: 'left' }).run().length;
    const sc = polygonContact(SPIRAL.world);
    const sp = SPIRAL.world.obstacles.map(polygonPerimeter);
    const s1 = new Bug1(sc, SPIRAL.start, SPIRAL.goal, { step: 0.05, turn: 'right' }).run();
    const s2 = new Bug2(sc, SPIRAL.start, SPIRAL.goal, { step: 0.05, turn: 'right' }).run();
    const b1 = bug1Bound(SPIRAL.start, SPIRAL.goal, sp);
    const b2 = bug2Bound(SPIRAL.start, SPIRAL.goal, SPIRAL.world.obstacles, sp);
    const n1 = lineCrossings(SPIRAL.world.obstacles[0], SPIRAL.start, SPIRAL.goal);
    const s2left = new Bug2(sc, SPIRAL.start, SPIRAL.goal, { step: 0.05, turn: 'left' }).run();
    const ok =
      simple2 < simple1 &&
      s1.done &&
      s2.done &&
      s2.length > s1.length &&
      s1.length <= b1 + 1e-9 &&
      s2.length <= b2 + 1e-9 &&
      Math.abs(s1.length - 124.9) < 1e-6 &&
      Math.abs(s2.length - 132.4) < 1e-6 &&
      Math.abs(s2left.length - 52.2) < 1e-6;
    push(
      'bug2 vs bug1: greedy is shorter on the two rectangles (16 < 34) and longer on the spiral turning right (132.4 > 124.9); turning left it wins again (52.2); both bounds hold',
      ok,
      `spiral Bug1 ${fmt(s1.length)} ≤ ${fmt(b1)}, Bug2 right ${fmt(s2.length)} ≤ ${fmt(b2)}, Bug2 left ${fmt(s2left.length)} (n₁ = ${n1}, p₁ = ${fmt(sp[0])})`,
    );
  }

  // -------------------------------------------------------------------------
  // Bounds on seeded random fields (the property test of Chapter-03.md §5)
  // -------------------------------------------------------------------------
  {
    let runs = 0;
    let violations = 0;
    let unreached = 0;
    let worstSlack1 = Infinity;
    let worstSlack2 = Infinity;
    for (let seed = 1; seed <= 40; seed++) {
      const pr = randomPreset(seed);
      if (pr.world.obstacles.length === 0) continue;
      const c = polygonContact(pr.world);
      const p = pr.world.obstacles.map(polygonPerimeter);
      const b1 = bug1Bound(pr.start, pr.goal, p);
      const b2 = bug2Bound(pr.start, pr.goal, pr.world.obstacles, p);
      for (const turn of ['left', 'right'] as const) {
        const r1 = new Bug1(c, pr.start, pr.goal, { step: 0.05, turn }).run();
        const r2 = new Bug2(c, pr.start, pr.goal, { step: 0.05, turn }).run();
        runs += 2;
        if (!r1.done) unreached++;
        if (!r2.done) unreached++;
        if (r1.length > b1 + 1e-9) violations++;
        if (r2.length > b2 + 1e-9) violations++;
        worstSlack1 = Math.min(worstSlack1, b1 - r1.length);
        worstSlack2 = Math.min(worstSlack2, b2 - r2.length);
      }
    }
    push(
      'bounds: on 40 seeded fields of disjoint convex obstacles both Bugs reach the goal and both bounds hold',
      runs >= 60 && violations === 0 && unreached === 0,
      `${runs} runs, ${violations} violations, ${unreached} unreached; tightest slack Bug1 ${fmt(worstSlack1)}, Bug2 ${fmt(worstSlack2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // An enclosed goal: every Bug must give up, and say so
  // -------------------------------------------------------------------------
  {
    const c = polygonContact(ENCLOSED.world);
    const r1 = new Bug1(c, ENCLOSED.start, ENCLOSED.goal, { step: 0.05, turn: 'left' }).run();
    const r2 = new Bug2(c, ENCLOSED.start, ENCLOSED.goal, { step: 0.05, turn: 'left' }).run();
    const tb = new TangentBug(ENCLOSED.world, ENCLOSED.start, ENCLOSED.goal, {
      step: 0.05,
      turn: 'left',
      range: ENCLOSED.range,
      clearance: 0.1,
      nRays: 180,
    }).run(20000);
    const p2 = polygonPerimeter(ENCLOSED.world.obstacles[1]);
    // Bug1's second loop walks WO₂'s perimeter exactly once before returning to q^L.
    const ok = r1.failed && r2.failed && tb.failed && r1.hits.length === 2 && r1.length <= bug1Bound(ENCLOSED.start, ENCLOSED.goal, ENCLOSED.world.obstacles.map(polygonPerimeter));
    push(
      'unreachable: with the goal inside WO₂, Bug1, Bug2 and Tangent Bug all halt with "unreachable"',
      ok,
      `Bug1 "${r1.reason}" after ${fmt(r1.length)} (p₂ = ${p2}); Bug2 "${r2.reason}"; TB "${tb.reason}"`,
    );
  }

  // -------------------------------------------------------------------------
  // Tangent Bug's first decision (infinite range): the √5 + √65 tie
  // -------------------------------------------------------------------------
  {
    const tb = new TangentBug(W.world, W.start, W.goal, { step: 0.05, turn: 'left', range: Infinity, clearance: 0.1, nRays: 720 });
    const e = tb.evaluate(W.start);
    const expected = Math.sqrt(5) + Math.sqrt(65);
    const os = e.candidates.filter((c) => c.label.startsWith('O'));
    const up = os.find((c) => c.point.y > 0);
    const down = os.find((c) => c.point.y < 0);
    const ok =
      e.blocked &&
      os.length === 2 &&
      up !== undefined &&
      down !== undefined &&
      near(up.point, { x: 2, y: 1 }, 1e-6) &&
      near(down.point, { x: 2, y: -1 }, 1e-6) &&
      Math.abs(up.h - expected) < 1e-6 &&
      Math.abs(down.h - expected) < 1e-6 &&
      e.chosen !== null &&
      e.chosen.point.y > 0;
    const runInf = new TangentBug(W.world, W.start, W.goal, { step: 0.05, turn: 'left', range: Infinity, clearance: 0.1, nRays: 360 }).run(20000);
    const run3 = new TangentBug(W.world, W.start, W.goal, { step: 0.05, turn: 'left', range: 3, clearance: 0.1, nRays: 360 }).run(20000);
    push(
      'tangent bug: from (0,0) with R = ∞ the O_i are (2,±1), both at d(x,O)+d(O,goal) = √5+√65 = 10.2983; the tie goes left; the run walks 11.21 (R = ∞) and 11.22 (R = 3), against Bug2\'s 16',
      ok && runInf.done && run3.done && Math.abs(runInf.length - 11.213) < 1e-2 && Math.abs(run3.length - 11.222) < 1e-2,
      `h = ${up ? fmt(up.h) : '?'} / ${down ? fmt(down.h) : '?'} vs ${fmt(expected)}; chosen ${e.chosen ? `(${fmt(e.chosen.point.x)}, ${fmt(e.chosen.point.y)})` : 'none'}; L(R=∞) = ${fmt(runInf.length)}, L(R=3) = ${fmt(run3.length)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Figure 2.8: the heuristic's blind spot, and what more range buys
  // -------------------------------------------------------------------------
  {
    const mk = (R: number) => new TangentBug(FIG_2_8.world, FIG_2_8.start, FIG_2_8.goal, { step: 0.05, turn: 'left', range: R, clearance: 0.1, nRays: 360 });
    const e3 = mk(3).evaluate(FIG_2_8.start);
    const e6 = mk(6).evaluate(FIG_2_8.start);
    const top = { x: 4, y: 3 };
    const topAt6 = e6.candidates.find((c) => near(c.point, top, 1e-6));
    const r3 = mk(3).run(20000);
    const r6 = mk(6).run(20000);
    const ok =
      e3.chosen !== null &&
      near(e3.chosen.point, top, 1e-6) &&
      Math.abs(e3.chosen.h - 7.8284) < 1e-3 &&
      topAt6 !== undefined &&
      !Number.isFinite(topAt6.h) &&
      e6.chosen !== null &&
      near(e6.chosen.point, { x: 5.5, y: 4.5 }, 1e-6) &&
      r3.done &&
      r6.done &&
      Math.abs(r3.length - 8.46) < 1e-2 &&
      Math.abs(r6.length - 8.129) < 1e-2;
    push(
      'tangent bug (fig. 2.8): with R = 3 the top corner of WO₁ wins (h = 7.828); with R = 6 the robot sees WO₂ block that corner, costs it ∞, steers for WO₂\'s corner (5.5, 4.5), and the path shortens 8.46 → 8.13',
      ok,
      `R=3 chosen ${e3.chosen ? `(${fmt(e3.chosen.point.x)}, ${fmt(e3.chosen.point.y)}) h=${fmt(e3.chosen.h)}` : 'none'}; R=6 top h=${topAt6 ? String(topAt6.h) : '?'}, chosen ${e6.chosen ? `(${fmt(e6.chosen.point.x)}, ${fmt(e6.chosen.point.y)})` : 'none'}; lengths ${fmt(r3.length)} / ${fmt(r6.length)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Tangent-line endpoints vs brute force silhouettes
  // -------------------------------------------------------------------------
  {
    let viewpoints = 0;
    let mismatches = 0;
    let worst = 0;
    let degenerate = 0;
    const test = (world: typeof W.world, x: Point2) => {
      const want = exactSilhouettePoints(world, world.obstacles, x, 1e9);
      if (want === null) {
        degenerate++;
        return;
      }
      const scan = rangeScan(world, x.x, x.y, 1440, 1e9);
      const got = intervalEndpoints(scan, continuityIntervals(scan, { world }), world).map((e) => e.point);
      viewpoints++;
      if (got.length !== want.length) {
        mismatches++;
        return;
      }
      for (const g of got) {
        const d = Math.min(...want.map((w) => dist(g, w)));
        worst = Math.max(worst, d);
        if (d > 1e-5) mismatches++;
      }
    };
    for (const x of [W.start, { x: 0, y: 2.5 }, { x: 5, y: 2.2 }, { x: 8.5, y: -3 }]) test(W.world, x);
    for (let seed = 1; seed <= 12; seed++) {
      const pr = randomPreset(seed);
      if (pr.world.obstacles.length > 0) test(pr.world, pr.start);
    }
    push(
      'tangent line: refined interval endpoints O_i coincide with brute-force silhouette points',
      viewpoints >= 12 && mismatches === 0 && worst < 1e-5,
      `${viewpoints} viewpoints (${degenerate} skipped as degenerate), ${mismatches} mismatches, worst ${worst.toExponential(2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // The tracer: offset curve of WO₁ at W* = 0.5 has length 8 + π (Derivation 6)
  // -------------------------------------------------------------------------
  const G = (p: Point2) => witness(W.world, p).d - 0.5;
  const DG = (p: Point2) => witness(W.world, p).grad;
  {
    const tr = traceCurve(G, DG, { x: 1.4, y: 0 }, 'left', { step: 0.02, maxSteps: 5000 });
    const expected = 8 + Math.PI;
    const ok = tr.stop === 'closed' && Math.abs(tr.length - expected) < 1e-3 && tr.maxResidual < 1e-8 && tr.points.length - 1 === 558;
    push(
      'trace_curve: the W* = 0.5 offset of [2,4]×[−1,1] closes after 558 steps of Δλ = 0.02 with length 8 + π to 1e-3 and |G| < 1e-8 on every sample',
      ok,
      `${tr.stop} after ${tr.points.length - 1} steps, length ${tr.length.toFixed(5)} vs ${expected.toFixed(5)}, max |G| ${tr.maxResidual.toExponential(2)}`,
    );
  }
  {
    // Quadratic convergence of the corrector (Theorem D.2.1): r_{h+1} ≤ a r_h².
    const GK = (p: Point2) => witness(KIDNEY, p).d - 0.5;
    const DGK = (p: Point2) => witness(KIDNEY, p).grad;
    const tr = traceCurve(GK, DGK, { x: 2.0, y: 0 }, 'left', { step: 0.4, maxSteps: 1, newtonIters: 6 });
    const r = tr.records[0]?.residuals ?? [];
    const ratios: number[] = [];
    for (let h = 1; h < r.length && r[h] > 1e-15; h++) ratios.push(r[h] / (r[h - 1] * r[h - 1]));
    const ok = r.length >= 3 && ratios.length >= 2 && ratios.every((q) => q < 0.1) && Math.max(...ratios) / Math.min(...ratios) < 3;
    push(
      'trace_curve: Newton residuals square at each correction — |G(y₁)|/|G(y₀)|² ≈ |G(y₂)|/|G(y₁)|² (quadratic, Theorem D.2.1)',
      ok,
      `residuals ${r.map((v) => v.toExponential(2)).join(' → ')}; ratios a ≈ ${ratios.map((q) => q.toFixed(3)).join(', ')}`,
    );
  }
  {
    // The hypothesis failing: into the kidney's dent the offset set has a cusp.
    const run = (Wstar: number) =>
      traceCurve((p) => witness(KIDNEY, p).d - Wstar, (p) => witness(KIDNEY, p).grad, { x: 0, y: -(1.5 + Wstar) }, 'left', {
        step: 0.05,
        maxSteps: 4000,
      });
    const smooth = run(0.5);
    const cusp = run(0.25);
    push(
      'trace_curve: on the kidney it closes at W* = 0.5 but stops "singular" at W* = 0.25, where the offset set enters the dent (r_c ≈ 0.05) and D_Y G → 0',
      smooth.stop === 'closed' && cusp.stop === 'singular' && cusp.points.length > 10,
      `W* = 0.5: ${smooth.stop} after ${smooth.points.length - 1} steps (length ${smooth.length.toFixed(3)}); W* = 0.25: ${cusp.stop} after ${cusp.points.length - 1} steps, |D_Y G| = ${Math.abs(cusp.records[cusp.records.length - 1].dyG).toFixed(3)}`,
    );
  }

  // -------------------------------------------------------------------------
  // D(x) = min_s ρ(x, s) and ∇D from the scan (problem 2.1), against the witness
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(3);
    const rel: number[] = [];
    const ang: number[] = [];
    let under = 0;
    let n = 0;
    while (n < 1000) {
      const x = rng.uniform(0.2, 11.8);
      const y = rng.uniform(0.2, 8.8);
      const w = witness(APARTMENT, { x, y });
      if (w.d < 0.05) continue;
      n++;
      const scan = rangeScan(APARTMENT, x, y, 720, 50);
      const ds = distanceFromScan(scan);
      if (ds < w.d - 1e-9) under++;
      rel.push((ds - w.d) / w.d);
      const g = gradientFromScan(scan);
      ang.push(Math.acos(Math.max(-1, Math.min(1, g.x * w.grad.x + g.y * w.grad.y))));
    }
    rel.sort((a, b) => a - b);
    ang.sort((a, b) => a - b);
    const q = (a: number[], p: number) => a[Math.floor(p * (a.length - 1))];
    push(
      'scan: min_s ρ(x,s) never undershoots D(x) and, with 720 rays, is within 0.5 % of it (and ∇D within 0.5°) at 90 % of 1000 seeded points',
      under === 0 && q(rel, 0.9) < 5e-3 && q(ang, 0.9) < (0.5 * Math.PI) / 180,
      `undershoots ${under}; relative error p50 ${q(rel, 0.5).toExponential(1)}, p90 ${q(rel, 0.9).toExponential(1)}, max ${q(rel, 1).toExponential(1)} (edge-on corners); ∇D angle p90 ${((q(ang, 0.9) * 180) / Math.PI).toFixed(3)}°`,
    );
  }

  // -------------------------------------------------------------------------
  // The integration lab: Rusty as a point with W* = 0.11 in the Apartment
  // -------------------------------------------------------------------------
  {
    const { start, goal, clearance, step } = APARTMENT_QUERY;
    const c = offsetContact(APARTMENT, { clearance });
    const r1 = new Bug1(c, start, goal, { step, turn: 'left' }).run(200000);
    const r2 = new Bug2(c, start, goal, { step, turn: 'left' }).run(200000);
    const tb = new TangentBug(APARTMENT, start, goal, { step, turn: 'left', range: 3, clearance, nRays: 180 }).run(40000);
    const minClear = (path: Point2[]) => path.reduce((m, p) => Math.min(m, distanceToWalls(APARTMENT, p.x, p.y)), Infinity);
    const clear = Math.min(minClear(r1.path), minClear(r2.path), minClear(tb.path));
    const ok =
      r1.done &&
      r2.done &&
      tb.done &&
      clear >= clearance - 1e-6 &&
      Math.abs(r1.length - 173.372) < 1e-2 &&
      Math.abs(r2.length - 112.343) < 1e-2 &&
      Math.abs(tb.length - 13.508) < 1e-2;
    push(
      'apartment lab: from (1,1) to (11,8) at W* = 0.11, Bug1 walks 173.37 m, Bug2 112.34 m, Tangent Bug (R = 3) 13.51 m; all arrive and none comes closer than W* to a wall',
      ok,
      `Bug1 ${fmt(r1.length)} (${r1.hits.length} hit), Bug2 ${fmt(r2.length)} (${r2.hits.length} hits), TB ${fmt(tb.length)} (${tb.hits.length} local minima); min clearance ${clear.toFixed(4)}`,
    );
  }

  return out;
}
