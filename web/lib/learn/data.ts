/**
 * `generate_dataset`: randomized Workbenches → Chapter 11 roadmap paths →
 * (𝓔, query, q) pairs for the sampler and the policy; Chapter 6 backward
 * Dijkstra → (𝓔, v, goal, h*) for the heuristic.
 *
 * The circularity the honesty ledger names lives here: the sampler learns
 * where *this* PRM's paths go, so it can at best reproduce PRM-with-a-large-
 * budget, faster. The labels are real — every path node passed the Chapter 2
 * checker, every h* is a Dijkstra cost — and that is all the data promises.
 */

import { Rng } from '../prob/rng';
import { Prm } from '../sampling/prm';
import { Uniform } from '../sampling/sampler';
import { StraightLine } from '../sampling/steer';
import { CoarseHistogram } from './info';
import type { Sample } from './mlp';
import { CELLS, CELLS_N, cellOfConfig, samplerFeatures } from './sampler';
import { HEUR_RASTER_N, heuristicFeatures, latticeOf, randomFreeCell } from './heuristic';
import { POLICY_ETA, policyFeatures, torusDelta } from './neural-planner';
import { costToGo, torusOctile } from './torus-grid';
import { makeWorldCase, randomQuery, type WorldCase, type WorldGenerator } from './worlds';

export interface PrmPathOptions {
  /** Roadmap nodes beyond start and goal. */
  n: number;
  k: number;
  /** Steer check step (radians). */
  step: number;
}

export const DEFAULT_PRM: PrmPathOptions = { n: 160, k: 6, step: 0.2 };

/** A Chapter 11 single-query PRM path on `wc`, or null when the roadmap did not join. */
export function prmPath(wc: WorldCase, start: number[], goal: number[], rng: Rng, opts: PrmPathOptions = DEFAULT_PRM): number[][] | null {
  const prm = new Prm<number[]>(
    wc.lab.space,
    wc.lab.free,
    { sampler: new Uniform<number[]>(), steer: new StraightLine<number[]>({ kind: 'subdivision', step: opts.step }), connect: { kind: 'kNearest', k: opts.k } },
    rng,
  );
  const s = prm.addNode(start);
  prm.connectNode(s, prm.candidates(s));
  const g = prm.addNode(goal);
  prm.connectNode(g, prm.candidates(g));
  prm.grow(opts.n + 2);
  if (!prm.roadmap.sameComponent(s, g)) return null;
  const sp = prm.roadmap.shortestPath(s, g);
  return sp ? prm.roadmap.pathPoints(sp.path) : null;
}

/** Points every `step` radians along a torus polyline (endpoints included). */
export function densify(wc: WorldCase, path: readonly number[][], step: number): number[][] {
  const space = wc.lab.space;
  const out: number[][] = [path[0]];
  for (let i = 1; i < path.length; i++) {
    const d = space.dist(path[i - 1], path[i]);
    const m = Math.max(1, Math.ceil(d / step));
    for (let t = 1; t <= m; t++) out.push(space.interpolate(path[i - 1], path[i], t / m));
  }
  return out;
}

export interface PathDataset {
  /** (features, cell) pairs: the sampler's maximum-likelihood targets. */
  sampler: Sample[];
  /** (features, Δq) pairs: the policy's behavior-cloning targets. */
  policy: Sample[];
  /** Per world: how many of its queries produced a path. */
  solved: number;
  attempted: number;
  /** Mean path-node histogram over worlds (p* on the coarse grid), for the KL readouts. */
  pathCells: CoarseHistogram;
}

/**
 * Run `queriesPerWorld` seeded PRM queries on each case. Sampler targets are the
 * cells of path nodes at 0.2 rad spacing (every node on a roadmap path, plus the
 * points between them that the local planner certified). Policy targets are the
 * wrapped displacement to the point η further along the same path.
 */
export function generatePathData(cases: readonly WorldCase[], queriesPerWorld: number, rng: Rng, prm: PrmPathOptions = DEFAULT_PRM): PathDataset {
  const sampler: Sample[] = [];
  const policy: Sample[] = [];
  const cellCounts = new Array<number>(CELLS).fill(0);
  let solved = 0;
  let attempted = 0;
  for (const wc of cases) {
    for (let q = 0; q < queriesPerWorld; q++) {
      const query = randomQuery(wc.lab, rng);
      if (!query) continue;
      attempted++;
      const [s, g] = query;
      const path = prmPath(wc, s, g, rng.fork(), prm);
      if (!path) continue;
      solved++;
      const feats = samplerFeatures(wc.enc, s, g);
      const dense = densify(wc, path, 0.2);
      for (const p of dense) {
        const c = cellOfConfig(p, CELLS_N);
        cellCounts[c]++;
        sampler.push({ x: feats, y: c });
      }
      // Policy: from each dense point, aim at the point η ahead along the path.
      const fine = densify(wc, path, POLICY_ETA / 4);
      const space = wc.lab.space;
      for (let i = 0; i < fine.length - 1; i++) {
        let j = i;
        let acc = 0;
        while (j + 1 < fine.length && acc + space.dist(fine[j], fine[j + 1]) <= POLICY_ETA) {
          acc += space.dist(fine[j], fine[j + 1]);
          j++;
        }
        if (j === i) j = i + 1;
        const target = fine[j];
        const d = torusDelta(fine[i], target);
        // tanh(y)·η = Δ  ⇒  y = atanh(Δ/η), clipped away from ±1.
        const y = d.map((v) => Math.atanh(Math.max(-0.95, Math.min(0.95, v / POLICY_ETA))));
        policy.push({ x: policyFeatures(wc.enc, fine[i], g), y });
      }
    }
  }
  return { sampler, policy, solved, attempted, pathCells: new CoarseHistogram(cellCounts) };
}

export interface HeuristicDataset {
  samples: Sample[];
  worlds: number;
  goals: number;
}

/**
 * For each case, `goalsPerWorld` random free goal cells; the label of a free,
 * reachable cell is log(h* ÷ h_oct). Up to `cellsPerGoal` cells per goal are kept.
 */
export function generateHeuristicData(cases: readonly WorldCase[], goalsPerWorld: number, cellsPerGoal: number, rng: Rng): HeuristicDataset {
  const samples: Sample[] = [];
  let goals = 0;
  for (const wc of cases) {
    const g = latticeOf(wc.lab, HEUR_RASTER_N);
    for (let k = 0; k < goalsPerWorld; k++) {
      const goal = randomFreeCell(g, rng);
      if (goal < 0) continue;
      goals++;
      const hStar = costToGo(g, goal);
      const oct = torusOctile(g, goal);
      const candidates: number[] = [];
      for (let c = 0; c < g.size; c++) if (Number.isFinite(hStar[c]) && hStar[c] > 0) candidates.push(c);
      // Fisher–Yates partial shuffle for a seeded subsample.
      for (let i = 0; i < Math.min(cellsPerGoal, candidates.length); i++) {
        const j = i + Math.floor(rng.next() * (candidates.length - i));
        const t = candidates[i];
        candidates[i] = candidates[j];
        candidates[j] = t;
        const c = candidates[i];
        samples.push({ x: heuristicFeatures(wc.enc, g, c, goal), y: [Math.log(hStar[c] / oct(c))] });
      }
    }
  }
  return { samples, worlds: cases.length, goals };
}

/** `count` seeded cases from one generator, seeds `seed0 … seed0 + count − 1`. */
export function worldCases(seed0: number, count: number, gen: WorldGenerator): WorldCase[] {
  const out: WorldCase[] = [];
  for (let i = 0; i < count; i++) out.push(makeWorldCase(seed0 + i, gen));
  return out;
}

/** p* on the coarse grid for one world and query: the cells of a fresh PRM path. */
export function pathCellHistogram(wc: WorldCase, start: number[], goal: number[], rng: Rng, prm: PrmPathOptions = DEFAULT_PRM): CoarseHistogram | null {
  const path = prmPath(wc, start, goal, rng, prm);
  if (!path) return null;
  const cells = densify(wc, path, 0.2).map((p) => cellOfConfig(p, CELLS_N));
  return CoarseHistogram.fromCells(cells, CELLS);
}
