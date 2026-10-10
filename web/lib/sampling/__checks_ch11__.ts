/**
 * Chapter 11 checks — the six-sample roadmap, the Theorem 7.4.1 and 7.4.2
 * numbers, kd-tree against brute force on ℝ², T² and SE(2), monotone
 * connectivity, and an independent dense collision check of every PRM path.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { T2 } from '../manifold/torus';
import { SE2 } from '../manifold/se2';
import { Rd } from '../manifold/rd';
import { SIX_SAMPLE_WORLD } from './free-space';
import { KdTree, LinearNn } from './kdtree';
import { Prm } from './prm';
import { Scripted, Uniform, Gaussian, Bridge, Obprm, drawFree } from './sampler';
import { StraightLine, checkSchedule } from './steer';
import {
  SIX_SAMPLES,
  SIX_SAMPLE_CONSTANTS as C,
  expansiveSampleCount,
  prmFailureBound,
  samplesForFailure,
  sigma,
  tilingBalls,
  unitBallVolume,
} from './analysis';
import { APARTMENT_GOAL, APARTMENT_START, discLab, torusLab } from './worlds';

const fmt = (x: number, d = 4) => x.toFixed(d);

export function runCh11Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The six-sample roadmap (design doc §3)
  // -------------------------------------------------------------------------
  const sixSample = (k: number) => {
    const world = SIX_SAMPLE_WORLD;
    world.free.reset();
    const prm = new Prm(world.space, world.free, {
      sampler: new Scripted(SIX_SAMPLES),
      steer: new StraightLine({ kind: 'subdivision', step: 0.25 }),
      connect: { kind: 'kNearest', k },
      nn: 'kd',
    }, new Rng(1));
    prm.build(6);
    return prm;
  };
  {
    const prm = sixSample(1);
    const edges = prm.roadmap.edges().map((e) => `${e.a + 1}${e.b + 1}`).sort();
    push(
      'ch11 six-sample roadmap, k = 1: edges {12, 23, 45, 56}, 2 components',
      edges.join(',') === '12,23,45,56' && prm.roadmap.components() === 2,
      `edges {${edges.join(', ')}}, ${prm.roadmap.components()} components`,
    );
  }
  {
    const prm = sixSample(2);
    const edges = prm.roadmap.edges().map((e) => `${e.a + 1}${e.b + 1}`).sort();
    const sp = prm.roadmap.shortestPath(0, 3);
    const ok = edges.join(',') === '12,23,36,45,56' && prm.roadmap.components() === 1 && sp !== null && Math.abs(sp.cost - 18.62) < 1e-3;
    push(
      'ch11 six-sample roadmap, k = 2: s1s4 and s2s4 rejected, 1 component, path s1→s4 = 18.620',
      ok,
      `edges {${edges.join(', ')}}, ${prm.roadmap.components()} component(s), path length ${sp ? fmt(sp.cost, 3) : 'none'}`,
    );
  }

  // -------------------------------------------------------------------------
  // Theorem 7.4.1 and 7.4.2 constants
  // -------------------------------------------------------------------------
  {
    const s = sigma(C.d, C.muFree);
    const m = tilingBalls(C.L, C.rho);
    const p500 = prmFailureBound(C.d, C.muFree, C.L, C.rho, 500);
    const p1000 = prmFailureBound(C.d, C.muFree, C.L, C.rho, 1000);
    const n1 = samplesForFailure(C.d, C.muFree, C.L, C.rho, 0.01);
    const ok = Math.abs(s - 0.8727) < 5e-4 && m === 24 && Math.abs(p500 - 0.306) < 1e-3 && Math.abs(p1000 - 0.0039) < 1e-4 && n1 === 892;
    push(
      'ch11 Theorem 7.4.1: σ = 0.8727, m = 24, bound 0.306 at n=500, 0.0039 at n=1000, n ≥ 892 for 1%',
      ok,
      `σ=${fmt(s)}, m=${m}, P(500)=${fmt(p500, 3)}, P(1000)=${fmt(p1000)}, n(1%)=${n1}; ζ₂=${fmt(unitBallVolume(2))}`,
    );
  }
  {
    const e = expansiveSampleCount(0.1, 0.1, 0.1, 0.05);
    push('ch11 Theorem 7.4.2 at ε=α=β=0.1, δ=0.05: n = 7775, 2n+2 = 15 552 samples', e.n === 7775 && e.samples === 15552, `n=${e.n}, samples=${e.samples}`);
  }

  // -------------------------------------------------------------------------
  // kd-tree equals brute force, including across the seam
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(11);
    const spaces: { name: string; make: () => { kd: KdTree<number[]>; lin: LinearNn<number[]>; sample: () => number[] } }[] = [
      {
        name: 'ℝ²',
        make: () => {
          const s = new Rd(2, { lo: [0, 0], hi: [10, 10] });
          return { kd: new KdTree(s), lin: new LinearNn(s), sample: () => s.sample(rng) };
        },
      },
      {
        name: 'T²',
        make: () => ({ kd: new KdTree(T2), lin: new LinearNn(T2), sample: () => T2.sample(rng) }),
      },
    ];
    let allOk = true;
    let detail = '';
    for (const sp of spaces) {
      const { kd, lin, sample } = sp.make();
      for (let i = 0; i < 2000; i++) {
        const q = sample();
        kd.insert(i, q);
        lin.insert(i, q);
      }
      let mismatches = 0;
      let seamNeighbors = 0;
      let visitedTotal = 0;
      for (let t = 0; t < 200; t++) {
        const q = sample();
        const a = kd.kNearest(q, 8);
        visitedTotal += kd.visited;
        const b = lin.kNearest(q, 8);
        if (a.length !== b.length || a.some((x, i) => x.idx !== b[i].idx || Math.abs(x.dist - b[i].dist) > 1e-12)) mismatches++;
        const r = 0.6;
        const wa = kd.within(q, r);
        const wb = lin.within(q, r);
        if (wa.length !== wb.length || wa.some((x, i) => x.idx !== wb[i].idx)) mismatches++;
        if (sp.name === 'T²') {
          for (const nb of a) {
            const p = kd.point(nb.idx);
            if (Math.abs(p[0] - q[0]) > Math.PI || Math.abs(p[1] - q[1]) > Math.PI) seamNeighbors++;
          }
        }
      }
      const pruned = visitedTotal / 200 < 2000 * 0.5;
      if (mismatches > 0 || !pruned || (sp.name === 'T²' && seamNeighbors === 0)) allOk = false;
      detail += `${sp.name}: ${mismatches} mismatches, ${(visitedTotal / 200).toFixed(0)} nodes/query${sp.name === 'T²' ? `, ${seamNeighbors} seam-crossing neighbors` : ''}; `;
    }
    push('ch11 kd-tree (periodic axes) equals brute force on ℝ² and T², with pruning and seam neighbors', allOk, detail.trim());
  }
  {
    // SE(2) with a non-unit exchange rate: the θ axis must be scaled in the pruning bound.
    const rng = new Rng(12);
    const s = new SE2(0.3, { minX: 0, minY: 0, maxX: 5, maxY: 5 });
    const kd = new KdTree(s);
    const lin = new LinearNn(s);
    for (let i = 0; i < 1500; i++) {
      const q = s.sample(rng);
      kd.insert(i, q);
      lin.insert(i, q);
    }
    let mismatches = 0;
    for (let t = 0; t < 150; t++) {
      const q = s.sample(rng);
      const a = kd.kNearest(q, 5);
      const b = lin.kNearest(q, 5);
      if (a.some((x, i) => x.idx !== b[i].idx)) mismatches++;
    }
    push('ch11 kd-tree on SE(2) with w = 0.3 equals brute force (weighted pruning bound)', mismatches === 0, `${mismatches} mismatches in 150 queries`);
  }

  // -------------------------------------------------------------------------
  // Check schedules
  // -------------------------------------------------------------------------
  {
    const inc = checkSchedule({ kind: 'incremental', step: 0.25 }, 1.0);
    const sub = checkSchedule({ kind: 'subdivision', step: 0.25 }, 1.0);
    const ok = inc.join(',') === '0.25,0.5,0.75' && sub.join(',') === '0.5,0.25,0.75';
    push('ch11 check schedules: incremental in order, subdivision midpoint-first (figure 7.6)', ok, `incremental [${inc.join(', ')}], subdivision [${sub.join(', ')}]`);
  }

  // -------------------------------------------------------------------------
  // Monotone connectivity and collision-free paths on the Apartment
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const rng = new Rng(7);
    const prm = new Prm(lab.space, lab.free, {
      sampler: new Uniform(),
      steer: new StraightLine({ kind: 'subdivision', step: 0.05 }),
      connect: { kind: 'kNearest', k: 6 },
    }, rng);
    const s = prm.addNode(APARTMENT_START);
    const g = prm.addNode(APARTMENT_GOAL);
    let connectedAt = -1;
    let monotone = true;
    let prevComponents = 2;
    while (prm.n < 500) {
      const before = prm.roadmap.sameComponent(s, g);
      prm.addSample();
      const after = prm.roadmap.sameComponent(s, g);
      if (before && !after) monotone = false;
      if (prm.roadmap.components() > prevComponents + 1) monotone = false;
      prevComponents = prm.roadmap.components();
      if (after && connectedAt < 0) connectedAt = prm.n;
    }
    push(
      'ch11 incremental PRM: start–goal connectivity never reverts and components grow by at most one per node',
      monotone && connectedAt > 0,
      `connected at n=${connectedAt}, ${prm.roadmap.components()} components at n=500, ${prm.roadmap.edgeCount} edges`,
    );

    const q = prm.query(APARTMENT_START, APARTMENT_GOAL, 8);
    let dense = true;
    if (q) {
      for (let i = 1; i < q.path.length; i++) {
        const a = q.path[i - 1];
        const b = q.path[i];
        const steps = Math.max(2, Math.ceil(lab.space.dist(a, b) / 0.005));
        for (let t = 0; t <= steps; t++) if (!lab.free.isFree(lab.space.interpolate(a, b, t / steps))) dense = false;
      }
    }
    push('ch11 PRM path is collision-free by an independent dense check (5 mm)', q !== null && dense, q ? `${q.path.length} waypoints, cost ${fmt(q.cost, 3)} m` : 'no path');
  }

  // -------------------------------------------------------------------------
  // Lazy PRM returns a verified path with fewer checks than classical
  // -------------------------------------------------------------------------
  {
    const run = (lazy: boolean, n: number) => {
      const lab = discLab();
      const prm = new Prm(lab.space, lab.free, {
        sampler: new Uniform(),
        steer: new StraightLine({ kind: 'subdivision', step: 0.05 }),
        connect: lazy ? { kind: 'lazy', k: 8 } : { kind: 'kNearest', k: 8 },
      }, new Rng(21));
      prm.grow(n);
      const q = prm.query(APARTMENT_START, APARTMENT_GOAL, 8);
      let dense = true;
      if (q) {
        for (let i = 1; i < q.path.length; i++) {
          const a = q.path[i - 1];
          const b = q.path[i];
          const steps = Math.max(2, Math.ceil(lab.space.dist(a, b) / 0.005));
          for (let t = 0; t <= steps; t++) if (!lab.free.isFree(lab.space.interpolate(a, b, t / steps))) dense = false;
        }
      }
      return { found: q !== null, dense, checks: lab.free.checks, n };
    };
    // Grow the classical roadmap until the query succeeds, then run lazy at the same n.
    let classical = run(false, 500);
    while (!classical.found && classical.n < 3000) classical = run(false, classical.n + 250);
    const lazy = run(true, classical.n);
    push(
      'ch11 lazy PRM: verified path, fewer collision checks than the classical k-nearest build',
      lazy.found && lazy.dense && classical.found && lazy.checks < classical.checks,
      `lazy ${lazy.checks} checks, classical ${classical.checks} checks (same seed, n=${classical.n})`,
    );
  }

  // -------------------------------------------------------------------------
  // Samplers put their mass where the chapter says
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const rng = new Rng(5);
    const clearanceOf = (q: number[]) => lab.free.clearance!(q);
    const meanClearance = (sampler: { sample: (s: typeof lab.space, f: typeof lab.free, r: Rng) => number[] | null }, n: number) => {
      let sum = 0;
      let got = 0;
      for (let i = 0; i < n; i++) {
        const q = drawFree(sampler as Uniform<number[]>, lab.space, lab.free, rng);
        if (q) {
          sum += clearanceOf(q);
          got++;
        }
      }
      return got ? sum / got : NaN;
    };
    const u = meanClearance(new Uniform(), 300);
    const gss = meanClearance(new Gaussian(0.3), 300);
    const br = meanClearance(new Bridge(0.6), 300);
    const ob = meanClearance(new Obprm(0.05), 300);
    push(
      'ch11 samplers: Gaussian, bridge and OBPRM samples lie nearer the walls than uniform ones',
      gss < u && br < u && ob < u,
      `mean clearance uniform ${fmt(u, 3)}, gaussian ${fmt(gss, 3)}, bridge ${fmt(br, 3)}, obprm ${fmt(ob, 3)} m`,
    );
  }

  // -------------------------------------------------------------------------
  // The same Prm on T²: Reach on the Workbench
  // -------------------------------------------------------------------------
  {
    const lab = torusLab();
    const prm = new Prm(lab.space, lab.free, {
      sampler: new Uniform(),
      steer: new StraightLine({ kind: 'subdivision', step: 0.05 }),
      connect: { kind: 'kNearest', k: 6 },
    }, new Rng(3));
    prm.grow(300);
    const q = prm.query([Math.PI / 4, Math.PI / 2], [-2.2, -1.2], 8);
    let seam = 0;
    let dense = true;
    if (q) {
      for (let i = 1; i < q.path.length; i++) {
        const a = q.path[i - 1];
        const b = q.path[i];
        if (Math.abs(a[0] - b[0]) > Math.PI || Math.abs(a[1] - b[1]) > Math.PI) seam++;
        const steps = Math.max(2, Math.ceil(lab.space.dist(a, b) / 0.01));
        for (let t = 0; t <= steps; t++) if (!lab.free.isFree(lab.space.interpolate(a, b, t / steps))) dense = false;
      }
    }
    push('ch11 the same Prm on T² (Reach, Workbench): path found and dense-checked free', q !== null && dense, q ? `${q.path.length} waypoints, ${seam} seam-crossing edge(s), cost ${fmt(q.cost, 3)} rad` : 'no path');
  }

  return out;
}
