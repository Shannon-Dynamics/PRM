/**
 * Chapter 12 checks — the three hand-traced RRT extensions (and the η = 1.5
 * `Trapped` branch), every tree planner's path dense-rechecked, the Voronoi
 * bias measured against its prediction, SRT collapsing to PRM and to RRT, and
 * the lazy SBL's edge accounting.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { THREE_EXTENSION_WORLD, SIX_SAMPLE_WORLD } from './free-space';
import { Scripted, Uniform } from './sampler';
import { StraightLine } from './steer';
import { SIX_SAMPLES } from './analysis';
import { Est, Kpiece, Rrt, RrtConnect, Sbl, Srt, THREE_EXTENSION_ROOT, THREE_EXTENSION_SAMPLES, pathIsFree, voronoiOwners } from './tree';
import { APARTMENT_GOAL, APARTMENT_START, TORUS_GOAL, TORUS_START, discLab, torusLab } from './worlds';

const fmt = (x: number, d = 4) => x.toFixed(d);
const close = (a: number, b: number, tol: number) => Math.abs(a - b) <= tol;

export function runCh12Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The three extensions (design doc §3)
  // -------------------------------------------------------------------------
  const traced = (eta: number) => {
    const w = THREE_EXTENSION_WORLD;
    w.free.reset();
    const rrt = new Rrt(w.space, w.free, [...THREE_EXTENSION_ROOT], null, {
      steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.25 }),
      sampler: new Scripted(THREE_EXTENSION_SAMPLES, false),
      eta,
      goalBias: 0,
    }, new Rng(12));
    const its = [rrt.extendOnce(), rrt.extendOnce(), rrt.extendOnce()];
    return { rrt, its };
  };
  {
    const { rrt, its } = traced(1.0);
    const n = rrt.tree.nodes;
    const want = [
      [1.8, 1.6],
      [1.8454, 2.599],
      [2.799, 1.5546],
    ];
    const coordsOk = want.every((q, i) => n[i + 1] && close(n[i + 1].q[0], q[0], 1e-4) && close(n[i + 1].q[1], q[1], 1e-4));
    const parentsOk = n.length === 4 && n[1].parent === 0 && n[2].parent === 1 && n[3].parent === 1;
    const statusOk = its.every((it) => it.status.kind === 'advanced');
    const nearOk = its[0].near === 0 && its[1].near === 1 && its[2].near === 1;
    const L = rrt.tree.totalLength();
    push(
      'ch12 three extensions (η = 1): q_new = (1.8, 1.6), (1.8454, 2.5990), (2.7990, 1.5546), all Advanced, branch at (1.8, 1.6), length 3.000',
      coordsOk && parentsOk && statusOk && nearOk && close(L, 3, 1e-9),
      `nodes ${n.slice(1).map((x) => `(${fmt(x.q[0])}, ${fmt(x.q[1])})`).join(', ')}; parents ${n.slice(1).map((x) => x.parent).join(',')}; length ${fmt(L, 3)}`,
    );
  }
  {
    const { rrt, its } = traced(1.5);
    const third = its[2];
    const ok = third.status.kind === 'trapped' && rrt.tree.size === 3 && rrt.trapped === 1 && third.near === 1;
    // The blocked configuration itself, recomputed: (1.8, 1.6) + 1.5 · (2.2, −0.1)/2.2023.
    const d = Math.hypot(2.2, -0.1);
    const qBlocked = [1.8 + (1.5 * 2.2) / d, 1.6 - (1.5 * 0.1) / d];
    push(
      'ch12 third extension with η = 1.5 lands at (3.2985, 1.5319) inside 𝒬𝒪 → Trapped, tree unchanged',
      ok && close(qBlocked[0], 3.2985, 1e-4) && close(qBlocked[1], 1.5319, 1e-4) && !THREE_EXTENSION_WORLD.free.isFree(qBlocked),
      `status ${third.status.kind}, ${rrt.tree.size} nodes, q_new would be (${fmt(qBlocked[0])}, ${fmt(qBlocked[1])})`,
    );
  }

  // -------------------------------------------------------------------------
  // Every tree planner finds a dense-checked free path on the Apartment
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
    const results: string[] = [];
    let allOk = true;
    const check = (name: string, path: number[][] | null, extra = '') => {
      const ok = path !== null && pathIsFree(lab.space, lab.free, path);
      if (!ok) allOk = false;
      results.push(`${name}: ${path ? `${path.length} waypoints${extra}` : 'no path'}${ok ? '' : ' ✗'}`);
    };
    {
      const rrt = new Rrt(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, eta: 0.6, goalBias: 0.05, goalRadius: 0.6 }, new Rng(12));
      check('RRT', rrt.plan(6000), `, ${rrt.iterations} iterations, ${rrt.trapped} trapped`);
    }
    {
      const est = new Est(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, radius: 0.8, goalRadius: 0.8 }, new Rng(12));
      check('EST', est.plan(8000), `, ${est.iterations} iterations`);
    }
    {
      const rc = new RrtConnect(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, eta: 0.6, greed: ['extend', 'connect'] }, new Rng(12));
      check('RRT-Connect', rc.merge(4000), `, ${rc.attempts} attempts`);
    }
    {
      const sbl = new Sbl(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, radius: 0.9, cell: 0.6 }, new Rng(12));
      check('SBL', sbl.plan(8000), `, ${sbl.edgeChecks} edge checks, ${sbl.edgeFailures} failed`);
    }
    {
      const kp = new Kpiece(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, cell: 0.75, eta: 0.6, goalRadius: 0.6 }, new Rng(12));
      check('KPIECE', kp.plan(8000), `, ${kp.iterations} iterations, ${kp.cells.size} cells`);
    }
    push('ch12 RRT, EST, RRT-Connect, SBL and KPIECE each return a path on the Apartment that a 5 mm dense recheck confirms free', allOk, results.join('; '));
  }

  // -------------------------------------------------------------------------
  // The same Rrt on T² (Reach), through the seam
  // -------------------------------------------------------------------------
  {
    const lab = torusLab();
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
    const rrt = new Rrt(lab.space, lab.free, TORUS_START, TORUS_GOAL, { steer, eta: 0.4, goalBias: 0.05, goalRadius: 0.4 }, new Rng(3));
    const path = rrt.plan(8000);
    const ok = path !== null && pathIsFree(lab.space, lab.free, path, 0.01);
    let seam = 0;
    if (path) for (let i = 1; i < path.length; i++) if (Math.abs(path[i][0] - path[i - 1][0]) > Math.PI || Math.abs(path[i][1] - path[i - 1][1]) > Math.PI) seam++;
    push('ch12 the same Rrt on T² (Reach, Workbench): path found and dense-checked free', ok, path ? `${path.length} waypoints, ${rrt.iterations} iterations, ${seam} seam-crossing edge(s)` : 'no path');
  }

  // -------------------------------------------------------------------------
  // Voronoi bias: selection frequency tracks Voronoi share
  // -------------------------------------------------------------------------
  {
    const w = THREE_EXTENSION_WORLD;
    const rng = new Rng(7);
    const rrt = new Rrt(w.space, w.free, [5, 5], null, { steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.25 }), eta: 0.7, goalBias: 0 }, rng);
    for (let i = 0; i < 40; i++) rrt.extendOnce();
    // Freeze the tree: count which node is nearest to many uniform samples.
    const vor = voronoiOwners(rrt.tree, 120, 120, w.free);
    const counts = new Array<number>(rrt.tree.size).fill(0);
    const trials = 20000;
    for (let t = 0; t < trials; t++) {
      const q = w.space.sample(rng);
      if (!w.free.isFree(q)) continue;
      counts[rrt.tree.nearest(q).idx]++;
    }
    const total = counts.reduce((a, b) => a + b, 0);
    let maxErr = 0;
    for (let i = 0; i < counts.length; i++) maxErr = Math.max(maxErr, Math.abs(counts[i] / total - vor.share[i]));
    // Frontier nodes own more than interior ones: the largest cell exceeds the mean by a wide margin.
    const maxShare = Math.max(...vor.share);
    push(
      'ch12 Voronoi bias: Pr[q_near = v] matches v’s Voronoi share within 1% on a 40-node tree, and the largest cell dwarfs the mean',
      maxErr < 0.01 && maxShare > 3 / rrt.tree.size,
      `max |freq − share| = ${fmt(maxErr)}, largest share ${fmt(maxShare, 3)} vs mean ${fmt(1 / rrt.tree.size, 3)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Goal bias p = 1 is a potential-field planner: it gets stuck
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
    const greedy = new Rrt(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, eta: 0.5, goalBias: 1, goalRadius: 0.5 }, new Rng(12));
    greedy.plan(400);
    const mixed = new Rrt(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, eta: 0.5, goalBias: 0.05, goalRadius: 0.5 }, new Rng(12));
    mixed.plan(6000);
    push(
      'ch12 goal bias p = 1 traps the tree (every iteration Trapped after the first wall); p = 0.05 solves the query',
      !greedy.solved && greedy.trappedStreak > 300 && mixed.solved,
      `p=1: ${greedy.tree.size} nodes, trapped streak ${greedy.trappedStreak}; p=0.05: solved at iteration ${mixed.solvedAt}`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation F.5: SRT degenerates to PRM and to RRT
  // -------------------------------------------------------------------------
  {
    const w = SIX_SAMPLE_WORLD;
    w.free.reset();
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.25 });
    const srt = Srt.prmLike(w.space, w.free, steer, 2, new Rng(1), new Scripted(SIX_SAMPLES));
    srt.addTrees(6);
    srt.connectAll();
    const edges = srt.edges.map((e) => (e.i < e.j ? `${e.i + 1}${e.j + 1}` : `${e.j + 1}${e.i + 1}`)).sort();
    const ok = edges.join(',') === '12,23,36,45,56' && srt.components === 1 && srt.edges.every((e) => !e.merged);
    push('ch12 SRT with (1 node, 1 close pair, 0 merges) reproduces the six-sample PRM edge set {12, 23, 36, 45, 56}', ok, `edges {${edges.join(', ')}}, ${srt.components} component(s)`);
  }
  {
    const lab = discLab();
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
    const srt = Srt.rrtLike(lab.space, lab.free, steer, 0.6, 4000, new Rng(12));
    const path = srt.query(APARTMENT_START, APARTMENT_GOAL);
    const ok = path !== null && srt.trees.length === 2 && pathIsFree(lab.space, lab.free, path) && srt.edges.length === 1 && srt.edges[0].merged;
    push('ch12 SRT with no roadmap trees and no close pairs is a bidirectional RRT: two trees, one merge edge, a free path', ok, path ? `${path.length} waypoints, trees of ${srt.trees[0].size} and ${srt.trees[1].size} nodes` : 'no path');
  }
  {
    // Between the corners: a real roadmap of trees whose query walks several trees.
    const lab = discLab();
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
    const srt = new Srt(lab.space, lab.free, { steer, nodesPerTree: 25, closePairs: 3, mergeIters: 60, k: 3, r: 1, eta: 0.5 }, new Rng(12));
    srt.addTrees(14);
    srt.connectAll();
    const path = srt.query(APARTMENT_START, APARTMENT_GOAL);
    const ok = path !== null && pathIsFree(lab.space, lab.free, path);
    push('ch12 SRT (25 nodes/tree, 3 close pairs, 60 merge iterations) answers the Apartment query with a free path', ok, path ? `${srt.trees.length} trees, ${srt.edges.length} edges (${srt.edges.filter((e) => e.merged).length} by merging), ${path.length} waypoints` : 'no path');
  }

  // -------------------------------------------------------------------------
  // SBL checks fewer edges than it creates
  // -------------------------------------------------------------------------
  {
    const lab = discLab();
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
    const sbl = new Sbl(lab.space, lab.free, APARTMENT_START, APARTMENT_GOAL, { steer, radius: 0.9, cell: 0.6 }, new Rng(5));
    const path = sbl.plan(8000);
    const edges = sbl.edges();
    const checked = edges.filter((e) => e.checked).length;
    push(
      'ch12 SBL: lazy edges — only those on candidate bridging paths were ever checked, and the returned path is free',
      path !== null && checked < edges.length && pathIsFree(lab.space, lab.free, path),
      `${edges.length} edges created, ${checked} verified, ${sbl.edgeChecks} Δ calls (${sbl.edgeFailures} failed), path ${path ? path.length : 0} waypoints`,
    );
  }

  return out;
}

/** The sampler kept exported so a widget can replay the hand trace. */
export { Uniform };
