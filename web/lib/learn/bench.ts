/**
 * The Honest Benchmark Board (D22.6) — the harness, not the numbers.
 *
 * Every planner gets the same world, the same query, the same seed and the same
 * budget, measured in the book's cost unit: collision checks. A run records the
 * check count at which the planner first held a verified path, the cost of
 * that path after Chapter 11's greedy shortcut, the wall-clock, and — for the
 * learned rows — the inference time separately, so the Board can show it added
 * and amortized. "Success at budget b" is then a read-off (checks ≤ b) for every
 * b at once, and no planner is rerun with a different stopping rule.
 *
 * Cost ratios are against the *incumbent*: the shortest path any method found
 * for that query, including a dense PRM run only for this purpose. That is a
 * weaker reference than an RRT* incumbent after ten seconds (the Rust harness
 * uses one) and the prose says so; it has the virtue that every ratio is ≥ 1
 * by construction and nothing is normalized by a number nobody computed.
 */

import { Rng } from '../prob/rng';
import { Prm } from '../sampling/prm';
import { strategySampler, type Sampler } from '../sampling/sampler';
import { StraightLine, shortcutGreedy } from '../sampling/steer';
import { RrtConnect } from '../sampling/tree';
import { pathLength } from '../manifold/manifold';
import type { Mlp } from './mlp';
import { LearnedSampler, mixtureSampler } from './sampler';
import { MpnetLike, POLICY_ETA } from './neural-planner';
import { makeWorldCase, randomQuery, type WorldCase, type WorldGenerator } from './worlds';

export type PlannerName = 'uniform' | 'gaussian' | 'bridge' | 'obprm' | 'learned' | 'rrt-connect' | 'mpnet';

export interface PlannerInfo {
  name: PlannerName;
  label: string;
  learned: boolean;
  family: 'PRM' | 'tree' | 'neural';
}

export const PLANNERS: readonly PlannerInfo[] = [
  { name: 'uniform', label: 'PRM · uniform', learned: false, family: 'PRM' },
  { name: 'gaussian', label: 'PRM · Gaussian', learned: false, family: 'PRM' },
  { name: 'bridge', label: 'PRM · bridge + uniform', learned: false, family: 'PRM' },
  { name: 'obprm', label: 'PRM · OBPRM', learned: false, family: 'PRM' },
  { name: 'learned', label: 'PRM · learned (λ = 0.1)', learned: true, family: 'PRM' },
  { name: 'rrt-connect', label: 'RRT-Connect', learned: false, family: 'tree' },
  { name: 'mpnet', label: 'MPNet-style + repair', learned: true, family: 'neural' },
];

export interface RunRecord {
  planner: PlannerName;
  solved: boolean;
  /** Collision checks spent when the first verified path appeared (budget if unsolved). */
  checks: number;
  /** Path cost after greedy shortcutting; +∞ if unsolved. */
  cost: number;
  /** Wall-clock of the planning loop, ms (includes inference). */
  ms: number;
  /** Of which: forward passes of a net. */
  inferenceMs: number;
}

export interface BenchQuery {
  seed: number;
  gen: WorldGenerator;
  start: number[];
  goal: number[];
  /** Shortest cost any method found (incumbent). */
  incumbent: number;
  runs: RunRecord[];
}

export interface BenchSuite {
  gen: WorldGenerator;
  queries: BenchQuery[];
}

export interface BenchData {
  budgets: number[];
  maxBudget: number;
  lambda: number;
  suites: { train: BenchSuite; heldout: BenchSuite };
  /** Offline training cost, ms, to be amortized over k queries. */
  trainingMs: { sampler: number; policy: number };
  meta: { worldsPerSuite: number; seed: number; k: number; date: string };
}

export interface Models {
  sampler: Mlp;
  policy: Mlp;
}

const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

const STEER_STEP = 0.2;
const K = 6;

/** PRM with start/goal as nodes 0 and 1, grown one sample at a time until joined or out of budget. */
function runPrm(name: PlannerName, wc: WorldCase, start: number[], goal: number[], sampler: Sampler<number[]>, budget: number, rng: Rng, inferenceMs: number): RunRecord {
  const { space, free } = wc.lab;
  const checks0 = free.checks;
  const t0 = now();
  const steer = new StraightLine<number[]>({ kind: 'subdivision', step: STEER_STEP });
  const prm = new Prm<number[]>(space, free, { sampler, steer, connect: { kind: 'kNearest', k: K } }, rng);
  const s = prm.addNode(start);
  prm.connectNode(s, prm.candidates(s));
  const g = prm.addNode(goal);
  prm.connectNode(g, prm.candidates(g));
  let guard = 0;
  while (!prm.roadmap.sameComponent(s, g) && free.checks - checks0 < budget && guard++ < 50 * budget) prm.addSample();
  const spent = free.checks - checks0;
  if (!prm.roadmap.sameComponent(s, g)) return { planner: name, solved: false, checks: budget, cost: Infinity, ms: now() - t0, inferenceMs };
  const sp = prm.roadmap.shortestPath(s, g)!;
  const path = shortcutGreedy(steer, space, free, prm.roadmap.pathPoints(sp.path));
  return { planner: name, solved: true, checks: spent, cost: pathLength(space, path), ms: now() - t0, inferenceMs };
}

function runRrtConnect(wc: WorldCase, start: number[], goal: number[], budget: number, rng: Rng): RunRecord {
  const { space, free } = wc.lab;
  const checks0 = free.checks;
  const t0 = now();
  const steer = new StraightLine<number[]>({ kind: 'subdivision', step: STEER_STEP });
  const rc = new RrtConnect<number[]>(space, free, start, goal, { steer, eta: POLICY_ETA, greed: ['extend', 'connect'] }, rng);
  let guard = 0;
  while (!rc.solved && free.checks - checks0 < budget && guard++ < 50 * budget) rc.step();
  const spent = free.checks - checks0;
  const raw = rc.path();
  if (!raw) return { planner: 'rrt-connect', solved: false, checks: budget, cost: Infinity, ms: now() - t0, inferenceMs: 0 };
  const path = shortcutGreedy(steer, space, free, raw);
  return { planner: 'rrt-connect', solved: true, checks: spent, cost: pathLength(space, path), ms: now() - t0, inferenceMs: 0 };
}

function runMpnet(wc: WorldCase, start: number[], goal: number[], policy: Mlp, budget: number, rng: Rng): RunRecord {
  const { space, free } = wc.lab;
  const t0 = now();
  const planner = new MpnetLike(policy, wc, { repairAttempts: 300, fallbackAttempts: 2500 });
  const plan = planner.plan(start, goal, rng);
  const solved = plan.path !== null && plan.checks <= budget;
  if (!solved) return { planner: 'mpnet', solved: false, checks: budget, cost: Infinity, ms: now() - t0, inferenceMs: plan.inferenceMs };
  const path = shortcutGreedy(planner.steer, space, free, plan.path!);
  return { planner: 'mpnet', solved: true, checks: plan.checks, cost: pathLength(space, path), ms: now() - t0, inferenceMs: plan.inferenceMs };
}

/** A dense PRM run only to tighten the incumbent. */
function referenceCost(wc: WorldCase, start: number[], goal: number[], rng: Rng): number {
  const { space, free } = wc.lab;
  const steer = new StraightLine<number[]>({ kind: 'subdivision', step: STEER_STEP });
  const prm = new Prm<number[]>(space, free, { sampler: strategySampler<number[]>('uniform', 0.8), steer, connect: { kind: 'kNearest', k: 10 } }, rng);
  const s = prm.addNode(start);
  prm.connectNode(s, prm.candidates(s));
  const g = prm.addNode(goal);
  prm.connectNode(g, prm.candidates(g));
  prm.grow(600);
  if (!prm.roadmap.sameComponent(s, g)) return Infinity;
  const sp = prm.roadmap.shortestPath(s, g)!;
  return pathLength(space, shortcutGreedy(steer, space, free, prm.roadmap.pathPoints(sp.path)));
}

/** Every planner on one query, same seed family, same budget. */
export function runQuery(wc: WorldCase, start: number[], goal: number[], models: Models, lambda: number, maxBudget: number, seed: number, planners: readonly PlannerName[] = PLANNERS.map((p) => p.name), reference?: number): BenchQuery {
  const runs: RunRecord[] = [];
  for (const name of planners) {
    const rng = new Rng(seed * 7919 + 17);
    switch (name) {
      case 'uniform':
      case 'gaussian':
      case 'bridge':
      case 'obprm':
        runs.push(runPrm(name, wc, start, goal, strategySampler<number[]>(name, 0.8, wc.lab.free.clearance), maxBudget, rng, 0));
        break;
      case 'learned': {
        const t0 = now();
        const ls = new LearnedSampler(models.sampler, wc.enc, start, goal);
        const infer = now() - t0;
        runs.push(runPrm('learned', wc, start, goal, mixtureSampler(ls, lambda), maxBudget, rng, infer));
        break;
      }
      case 'rrt-connect':
        runs.push(runRrtConnect(wc, start, goal, maxBudget, rng));
        break;
      case 'mpnet':
        runs.push(runMpnet(wc, start, goal, models.policy, maxBudget, rng));
        break;
    }
  }
  const ref = reference ?? referenceCost(wc, start, goal, new Rng(seed * 31 + 5));
  const incumbent = Math.min(ref, ...runs.map((r) => r.cost));
  return { seed: wc.seed, gen: wc.gen, start, goal, incumbent, runs };
}

export interface BenchConfig {
  worldsPerSuite: number;
  /** Seeds for the two suites start here (train) and here + 10 000 (held-out). */
  seed: number;
  budgets: number[];
  lambda: number;
  trainingMs: { sampler: number; policy: number };
  planners?: readonly PlannerName[];
  onProgress?: (done: number, total: number) => void;
}

/**
 * `count` *solvable* queries from seeds `seed0, seed0 + 1, …`: a world is kept
 * only if the dense reference PRM connects its query, so "success at budget" is
 * measured on queries that have an answer. Reach's free space on a cluttered
 * table is often disconnected; those queries would fail every planner alike
 * and say nothing about the proposals.
 */
export function runSuite(gen: WorldGenerator, seed0: number, count: number, models: Models, cfg: BenchConfig): BenchSuite {
  const maxBudget = Math.max(...cfg.budgets);
  const queries: BenchQuery[] = [];
  let seed = seed0;
  let guard = 0;
  while (queries.length < count && guard++ < 20 * count) {
    const wc = makeWorldCase(seed++, gen);
    const q = randomQuery(wc.lab, new Rng(wc.seed ^ 0x5bd1e995));
    if (!q) continue;
    const ref = referenceCost(wc, q[0], q[1], new Rng(wc.seed * 31 + 5));
    if (!Number.isFinite(ref)) continue;
    queries.push(runQuery(wc, q[0], q[1], models, cfg.lambda, maxBudget, wc.seed, cfg.planners, ref));
    cfg.onProgress?.(queries.length, count);
  }
  return { gen, queries };
}

export function runBenchmark(models: Models, cfg: BenchConfig): BenchData {
  const train = runSuite('train', cfg.seed, cfg.worldsPerSuite, models, cfg);
  const heldout = runSuite('heldout', cfg.seed + 10000, cfg.worldsPerSuite, models, cfg);
  return {
    budgets: [...cfg.budgets],
    maxBudget: Math.max(...cfg.budgets),
    lambda: cfg.lambda,
    suites: { train, heldout },
    trainingMs: cfg.trainingMs,
    meta: { worldsPerSuite: cfg.worldsPerSuite, seed: cfg.seed, k: K, date: new Date().toISOString().slice(0, 10) },
  };
}

/* -------------------------------------------------------------------------- */
/* Reading the Board                                                           */
/* -------------------------------------------------------------------------- */

export interface BoardRow {
  planner: PlannerName;
  label: string;
  learned: boolean;
  /** Fraction of queries solved within `budget` checks. */
  success: number;
  /** Mean cost / incumbent over the queries solved within budget (NaN if none). */
  costRatio: number;
  /** Mean wall-clock of the planning loop, ms, over all queries (including failures). */
  ms: number;
  /** Mean inference ms (0 for classical rows). */
  inferenceMs: number;
  /** Mean checks spent by the solved queries. */
  checks: number;
  solvedCount: number;
  total: number;
}

export function summarize(suite: BenchSuite, budget: number): BoardRow[] {
  return PLANNERS.map((p) => {
    const recs = suite.queries.map((q) => ({ q, r: q.runs.find((r) => r.planner === p.name) })).filter((x) => x.r) as { q: BenchQuery; r: RunRecord }[];
    const solved = recs.filter(({ r }) => r.solved && r.checks <= budget);
    const ratio = solved.length ? solved.reduce((s, { q, r }) => s + r.cost / q.incumbent, 0) / solved.length : NaN;
    return {
      planner: p.name,
      label: p.label,
      learned: p.learned,
      success: recs.length ? solved.length / recs.length : 0,
      costRatio: ratio,
      ms: recs.length ? recs.reduce((s, { r }) => s + r.ms, 0) / recs.length : 0,
      inferenceMs: recs.length ? recs.reduce((s, { r }) => s + r.inferenceMs, 0) / recs.length : 0,
      checks: solved.length ? solved.reduce((s, { r }) => s + r.checks, 0) / solved.length : NaN,
      solvedCount: solved.length,
      total: recs.length,
    };
  });
}

/** Success fraction of one planner at each budget. */
export function successCurve(suite: BenchSuite, planner: PlannerName, budgets: readonly number[]): { x: number; y: number }[] {
  return budgets.map((b) => {
    const recs = suite.queries.map((q) => q.runs.find((r) => r.planner === planner)).filter(Boolean) as RunRecord[];
    const ok = recs.filter((r) => r.solved && r.checks <= b).length;
    return { x: b, y: recs.length ? ok / recs.length : 0 };
  });
}

export function costRatioCurve(suite: BenchSuite, planner: PlannerName, budgets: readonly number[]): { x: number; y: number }[] {
  return budgets.map((b) => {
    const rows = summarize(suite, b).find((r) => r.planner === planner)!;
    return { x: b, y: rows.costRatio };
  });
}

/** Wall-clock per query once training is spread over k queries on similar worlds. */
export function amortizedMs(row: BoardRow, trainingMs: number, k: number): number {
  return row.learned ? row.ms + trainingMs / Math.max(1, k) : row.ms;
}

/** The k at which the learned row's amortized time drops below a classical row's. */
export function breakEvenQueries(learned: BoardRow, classical: BoardRow, trainingMs: number): number {
  const gain = classical.ms - learned.ms;
  if (gain <= 0) return Infinity;
  return Math.ceil(trainingMs / gain);
}
