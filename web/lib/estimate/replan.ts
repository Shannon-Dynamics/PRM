/**
 * `estimate/replan` — planning when q_start is a distribution. This chapter's
 * one owned artifact.
 *
 * The question Parts II–III never asked: what does a planner do when the start
 * is a belief with three peaks and no mean worth driving toward? Not "plan from
 * the mean" (the mean can be inside a wall), not "solve the POMDP" (the sister's
 * Ch. 22), but the loop every fielded stack runs:
 *
 *   1. compute the Ch. 6 universal plan J once per goal — backward Dijkstra on
 *      the inflated lattice, J(obstacle) = J(off-grid) = λ;
 *   2. sample the belief: K weighted cells (x_j, ω_j);
 *   3. score each first action a by  𝒥_bel(a) = Σ_j ω_j [ c(a) + J(f(x_j, a)) ] − β_info ΔH_a;
 *   4. take the argmin, sense, update, repeat.
 *
 * Step 3 is Bellman's principle with the expectation moved outside the min —
 * the QMDP approximation — and step 4 is what gives back the observations that
 * approximation ignored. Because J is a field, one decision costs O(K·|A|)
 * after a single O(|V| log |V|) Dijkstra, which is what makes planning on a
 * belief cheap enough to do every tick.
 *
 * The 3×3 micro-example of the design doc (§3 Part B) lives here as data, so the
 * check file and the w16.2 inset table read the same numbers.
 */

import type { Pose2 } from '../geom/se2';
import { Rng } from '../prob/rng';
import { discreteEntropy } from '../prob/gaussian';
import { GridGraph, cellAt, cellCenter, cellIJ, cellIndex, freeCellNear, gridFromAscii, gridFromWorld, type OccupancyGrid as Lattice } from '../search/grid';
import { followGridPlan, universalPlanGrid, type GridPlan } from '../search/policy';
import { beamAngles, collides, isFree, rayCast, simulateScan, type ScanParams, type World } from '../sim/world';
import {
  DEFAULT_CHOSET_SIGMAS,
  ParticleBelief,
  incrementBetween,
  sampleChosetMotion,
  type ChosetIncrement,
  type ChosetSigmas,
  type WeightedCell,
} from './bayes';

/* -------------------------------------------------------------------------- */
/* Actions on the lattice                                                      */
/* -------------------------------------------------------------------------- */

export type Action = 'Up' | 'Right' | 'Down' | 'Left' | 'UpRight' | 'DownRight' | 'DownLeft' | 'UpLeft';

/**
 * (Δi, Δj) per action. The lattice's j grows upward (`gridFromAscii` puts the
 * first string at the top), so the design doc's "Up — toward row 0" is +j.
 */
export const ACTION_STEP: Record<Action, readonly [number, number]> = {
  Up: [0, 1],
  Right: [1, 0],
  Down: [0, -1],
  Left: [-1, 0],
  UpRight: [1, 1],
  DownRight: [1, -1],
  DownLeft: [-1, -1],
  UpLeft: [-1, 1],
};

export const ACTIONS_4: readonly Action[] = ['Up', 'Right', 'Down', 'Left'];
export const ACTIONS_8: readonly Action[] = [...ACTIONS_4, 'UpRight', 'DownRight', 'DownLeft', 'UpLeft'];

export const isDiagonal = (a: Action): boolean => ACTION_STEP[a][0] !== 0 && ACTION_STEP[a][1] !== 0;

/** f(x, a): the cell reached from `cell` by `a`, or −1 when it leaves the grid. */
export function successor(lattice: Lattice, cell: number, a: Action): number {
  const [i, j] = cellIJ(lattice, cell);
  const [di, dj] = ACTION_STEP[a];
  const ni = i + di;
  const nj = j + dj;
  if (ni < 0 || nj < 0 || ni >= lattice.nx || nj >= lattice.ny) return -1;
  return cellIndex(lattice, ni, nj);
}

/* -------------------------------------------------------------------------- */
/* The cost-to-go field: Ch. 6's universal plan with λ outside the free set     */
/* -------------------------------------------------------------------------- */

/**
 * J on the lattice. Backward Dijkstra (`universalPlanGrid`) runs once; every
 * query afterwards is an array read. Obstacles, off-grid cells and cells the
 * goal cannot reach all read λ — the price of "take an action that ends in a
 * wall", which the chapter leaves as an explicit parameter because the answer
 * depends on it.
 */
export class CostToGo {
  readonly plan: GridPlan;
  readonly lattice: Lattice;

  constructor(
    readonly graph: GridGraph,
    readonly goal: number,
    readonly lambda: number,
  ) {
    this.lattice = graph.grid;
    this.plan = universalPlanGrid(graph, goal);
  }

  /** Is the cell on the grid, free, and connected to the goal? */
  isFree(cell: number): boolean {
    return cell >= 0 && cell < this.lattice.occupied.length && this.lattice.occupied[cell] === 0 && Number.isFinite(this.plan.costToGo[cell]);
  }

  /** J(cell), with λ wherever the plan has nothing to say. */
  J(cell: number): number {
    return this.isFree(cell) ? this.plan.costToGo[cell] : this.lambda;
  }

  /**
   * Can `a` be executed from `cell`? The successor must be free and, when the
   * graph forbids corner cutting, a diagonal may not squeeze between two
   * orthogonally adjacent obstacles — the same rule `GridGraph.neighbors` applies,
   * so the planner never takes an edge the field was not computed on.
   */
  feasible(cell: number, a: Action): boolean {
    const next = successor(this.lattice, cell, a);
    if (!this.isFree(next)) return false;
    if (isDiagonal(a) && !this.graph.cornerCutting) {
      const [i, j] = cellIJ(this.lattice, cell);
      const [di, dj] = ACTION_STEP[a];
      const g = this.lattice;
      if (g.occupied[cellIndex(g, i + di, j)] || g.occupied[cellIndex(g, i, j + dj)]) return false;
    }
    return true;
  }

  /** c(a): one lattice step, the diagonal at the graph's own ratio. */
  stepCost(a: Action): number {
    return (isDiagonal(a) ? this.graph.diagonal : 1) * this.lattice.cellSize;
  }

  /** The optimal path from a known cell — what A* would have returned. */
  pathFrom(cell: number): number[] {
    return followGridPlan(this.plan, cell);
  }

  /** The same field at another λ; Dijkstra is not rerun because λ never enters it. */
  withLambda(lambda: number): CostToGo {
    const out = Object.create(CostToGo.prototype) as CostToGo;
    Object.assign(out, { graph: this.graph, goal: this.goal, lambda, lattice: this.lattice, plan: this.plan });
    return out;
  }
}

/** `cost_to_go(lattice, goal, λ)` of the design doc. */
export function costToGo(graph: GridGraph, goal: number, lambda: number): CostToGo {
  return new CostToGo(graph, goal, lambda);
}

/* -------------------------------------------------------------------------- */
/* replan_on_belief                                                             */
/* -------------------------------------------------------------------------- */

export interface ReplanConfig {
  /** Penalty for an action that ends in an obstacle or off the grid. */
  lambda: number;
  /** Weight of the information bonus; 0 disables it. */
  betaInfo: number;
  actions: readonly Action[];
}

export const REPLAN_DEFAULT: ReplanConfig = { lambda: 10, betaInfo: 0, actions: ACTIONS_4 };

/** The expected one-step entropy reduction a sensor would deliver after an action — a heuristic, named so. */
export interface InfoBonus {
  /** `after` holds each sample's successor cell (or −1) with its weight. */
  expectedEntropyDrop(after: readonly WeightedCell[], action: Action): number;
}

export interface ActionScore {
  action: Action;
  /** Σ_j ω_j [c(a) + J(f(x_j, a))] — the expected cost before any bonus. */
  expected: number;
  /** β_info · ΔH_a, subtracted. */
  infoBonus: number;
  /** expected − infoBonus: what the argmin ranks. */
  total: number;
  /** Mass of the samples for which `a` hits a wall or leaves the grid — the coefficient of λ. */
  collisionMass: number;
}

export interface ReplanDecision {
  action: Action;
  /** One row per action, in the configured action order. */
  table: ActionScore[];
}

/** c(a) + J(f(x, a)), or λ when the action is infeasible from x — the single-hypothesis cost. */
export function actionCost(J: CostToGo, cell: number, a: Action): number {
  if (!J.feasible(cell, a)) return J.lambda;
  return J.stepCost(a) + J.J(successor(J.lattice, cell, a));
}

/**
 * a* = argmin_a Σ_j ω_j [c(a) + J(f(x_j, a))] − β_info ΔH_a.
 *
 * Returns the whole table because the w16.2 inset and the §3 micro-example both
 * need every row, not just the winner. Ties fall to action order.
 */
export function replanOnBelief(J: CostToGo, samples: readonly WeightedCell[], cfg: ReplanConfig, info?: InfoBonus): ReplanDecision {
  const table: ActionScore[] = [];
  let best: ActionScore | null = null;
  for (const a of cfg.actions) {
    let expected = 0;
    let collisionMass = 0;
    const after: WeightedCell[] = [];
    for (const { cell, weight } of samples) {
      const next = successor(J.lattice, cell, a);
      const feasible = J.feasible(cell, a);
      expected += weight * (feasible ? J.stepCost(a) + J.J(next) : J.lambda);
      if (!feasible) collisionMass += weight;
      after.push({ cell: feasible ? next : -1, weight });
    }
    const infoBonus = info && cfg.betaInfo > 0 ? cfg.betaInfo * info.expectedEntropyDrop(after, a) : 0;
    const row: ActionScore = { action: a, expected, infoBonus, total: expected - infoBonus, collisionMass };
    table.push(row);
    if (!best || row.total < best.total - 1e-12) best = row;
  }
  return { action: (best ?? table[0]).action, table };
}

/** The certainty-equivalent choice: plan as if the mode were the truth. */
export function planFromMode(J: CostToGo, modeCell: number, cfg: ReplanConfig): ReplanDecision {
  return replanOnBelief(J, [{ cell: modeCell, weight: 1 }], { ...cfg, betaInfo: 0 });
}

export interface CostCoefficients {
  action: Action;
  /** 𝒥_bel(a) = A + B·λ: A is the feasible part, B the collision mass. */
  A: number;
  B: number;
}

/**
 * Every expected cost is affine in λ, so the whole table can be written down
 * for all penalties at once. This is how the break-even is computed exactly
 * rather than by scanning the slider.
 */
export function costCoefficients(J: CostToGo, samples: readonly WeightedCell[], cfg: ReplanConfig): CostCoefficients[] {
  return cfg.actions.map((a) => {
    let A = 0;
    let B = 0;
    for (const { cell, weight } of samples) {
      if (J.feasible(cell, a)) A += weight * (J.stepCost(a) + J.J(successor(J.lattice, cell, a)));
      else B += weight;
    }
    return { action: a, A, B };
  });
}

/** λ* at which two actions cost the same, or null when they never cross (parallel lines). */
export function breakEvenLambda(coeffs: readonly CostCoefficients[], a1: Action, a2: Action): number | null {
  const c1 = coeffs.find((c) => c.action === a1);
  const c2 = coeffs.find((c) => c.action === a2);
  if (!c1 || !c2 || Math.abs(c1.B - c2.B) < 1e-12) return null;
  return (c2.A - c1.A) / (c1.B - c2.B);
}

/**
 * The λ at which the belief planner's choice would change from what it is at
 * `cfg.lambda`: the nearest crossing of the current winner with any other row.
 * The w16.2 slider marks this point.
 */
export function nextFlipLambda(J: CostToGo, samples: readonly WeightedCell[], cfg: ReplanConfig): number | null {
  const coeffs = costCoefficients(J, samples, cfg);
  const winner = replanOnBelief(J, samples, { ...cfg, betaInfo: 0 }).action;
  let best: number | null = null;
  for (const c of coeffs) {
    if (c.action === winner) continue;
    const l = breakEvenLambda(coeffs, winner, c.action);
    if (l === null || l <= 0) continue;
    if (best === null || Math.abs(l - cfg.lambda) < Math.abs(best - cfg.lambda)) best = l;
  }
  return best;
}

/* -------------------------------------------------------------------------- */
/* The §3 Part B micro-example                                                  */
/* -------------------------------------------------------------------------- */

/** Design-doc (row, col) with rows counted from the top, to a lattice cell. */
export function cellAtRowCol(lattice: Lattice, row: number, col: number): number {
  return cellIndex(lattice, col, lattice.ny - 1 - row);
}

/**
 * 3×3, 4-connected, unit steps, obstacle at (1, 0), goal at (0, 1). Belief:
 * (2, 1) with weight 0.6 (the mode) and (2, 0) with weight 0.4. Up from (2, 0)
 * hits the obstacle, so Up costs 0.6·2 + 0.4·λ while Right costs 3.6 for every λ:
 * the planner moves sideways first, and the break-even is λ* = 6.
 */
export function replan3x3(lambda = 10): {
  lattice: Lattice;
  graph: GridGraph;
  J: CostToGo;
  samples: WeightedCell[];
  Jtable: number[][];
  decision: ReplanDecision;
  modeDecision: ReplanDecision;
  breakEven: number | null;
} {
  const lattice = gridFromAscii(['...', '#..', '...']);
  const graph = new GridGraph(lattice, { connectivity: 4, diagonal: Math.SQRT2, cornerCutting: false });
  const goal = cellAtRowCol(lattice, 0, 1);
  const J = costToGo(graph, goal, lambda);
  const samples: WeightedCell[] = [
    { cell: cellAtRowCol(lattice, 2, 1), weight: 0.6 },
    { cell: cellAtRowCol(lattice, 2, 0), weight: 0.4 },
  ];
  const cfg: ReplanConfig = { lambda, betaInfo: 0, actions: ACTIONS_4 };
  const Jtable = [0, 1, 2].map((r) => [0, 1, 2].map((c) => J.J(cellAtRowCol(lattice, r, c))));
  const decision = replanOnBelief(J, samples, cfg);
  const modeDecision = planFromMode(J, samples[0].cell, cfg);
  const breakEven = breakEvenLambda(costCoefficients(J, samples, cfg), 'Up', 'Right');
  return { lattice, graph, J, samples, Jtable, decision, modeDecision, breakEven };
}

/* -------------------------------------------------------------------------- */
/* The information bonus                                                        */
/* -------------------------------------------------------------------------- */

/**
 * ΔH_a as the mutual information between "which hypothesis is true" and the
 * scan the robot would read after the move, under a Gaussian beam model with
 * noise-free anticipation: every hypothesis l predicts a range signature ŷ_l
 * (ray cast at the cell centre, absolute bearings), the posterior over
 * hypotheses after reading ŷ_l is ∝ ω_j N(ŷ_l; ŷ_j, σ²I), and
 *
 *   ΔH_a = H(ω) − Σ_l ω_l H( post_l ).
 *
 * Positions whose predicted scans differ are told apart by one reading; those
 * whose scans agree (mirror rooms) are not, and the bonus says so. A heuristic:
 * it anticipates one reading, ignores heading, and takes K² · beams ray-cast
 * lookups, cached per cell.
 */
export class ScanInfoBonus implements InfoBonus {
  private readonly cache = new Map<number, Float64Array>();
  readonly angles: number[];

  constructor(
    readonly world: World,
    readonly lattice: Lattice,
    opts: { nBeams?: number; maxRange?: number; sigma?: number } = {},
  ) {
    this.angles = beamAngles({ nBeams: opts.nBeams ?? 12, fov: 2 * Math.PI });
    this.maxRange = opts.maxRange ?? 8;
    this.sigma = opts.sigma ?? 0.35;
  }
  readonly maxRange: number;
  readonly sigma: number;

  signature(cell: number): Float64Array {
    const hit = this.cache.get(cell);
    if (hit) return hit;
    const { x, y } = cellCenter(this.lattice, cell);
    const sig = new Float64Array(this.angles.length);
    for (let b = 0; b < this.angles.length; b++) sig[b] = rayCast(this.world, x, y, this.angles[b], this.maxRange);
    this.cache.set(cell, sig);
    return sig;
  }

  expectedEntropyDrop(after: readonly WeightedCell[]): number {
    const live = after.filter((s) => s.cell >= 0 && s.weight > 0);
    if (live.length < 2) return 0;
    const w = live.map((s) => s.weight);
    const total = w.reduce((a, b) => a + b, 0);
    const omega = w.map((x) => x / total);
    const sigs = live.map((s) => this.signature(s.cell));
    const inv2 = 1 / (2 * this.sigma * this.sigma);
    let expectedPosterior = 0;
    for (let l = 0; l < live.length; l++) {
      const logs = sigs.map((sj) => {
        let d2 = 0;
        for (let b = 0; b < sj.length; b++) {
          const d = sigs[l][b] - sj[b];
          d2 += d * d;
        }
        return -d2 * inv2;
      });
      const max = Math.max(...logs);
      const post = logs.map((lg, j) => omega[j] * Math.exp(lg - max));
      const z = post.reduce((a, b) => a + b, 0);
      expectedPosterior += omega[l] * discreteEntropy(post.map((p) => p / z));
    }
    return Math.max(0, discreteEntropy(omega) - expectedPosterior);
  }
}

/* -------------------------------------------------------------------------- */
/* The loop: sense → update → replan → act                                      */
/* -------------------------------------------------------------------------- */

export interface NavigatorOptions {
  world: World;
  rng: Rng;
  /** Where Rusty really is. Snapped to the lattice. */
  truth: Pose2;
  goal: { x: number; y: number };
  belief: ParticleBelief;
  cellSize?: number;
  /** Robot radius plus margin, metres — the Ch. 4 inflation of the lattice. */
  inflation?: number;
  lambda?: number;
  betaInfo?: number;
  kSamples?: number;
  planFromMode?: boolean;
  /** Stop correcting: the belief dead-reckons and the planner replans on stale information. */
  freezeBelief?: boolean;
  scan?: Partial<ScanParams>;
  /** Noise the floor actually applies when an action is executed. */
  execSigmas?: ChosetSigmas;
  infoBonus?: ScanInfoBonus | null;
}

export interface NavigatorSnapshot {
  truth: Pose2;
  decision: ReplanDecision | null;
  samples: WeightedCell[];
  flipLambda: number | null;
  entropyBits: number;
  collisions: number;
  steps: number;
  reached: boolean;
}

/**
 * `BeliefPlanner` of the design doc: one loop for a grid, a cloud, or a Gaussian.
 * `step` decides against the sampled belief, drives Rusty one lattice step in the
 * Ch. 2 Apartment (with slip), reads a scan, and updates the belief. The planner
 * never sees `truth`; it is here so the widget can draw it and the check can score it.
 */
export class BeliefNavigator {
  readonly world: World;
  readonly rng: Rng;
  readonly lattice: Lattice;
  readonly graph: GridGraph;
  readonly goalCell: number;
  J: CostToGo;
  cfg: ReplanConfig;
  kSamples: number;
  planFromMode: boolean;
  freezeBelief: boolean;
  readonly belief: ParticleBelief;
  readonly scanParams: ScanParams;
  readonly angles: number[];
  readonly execSigmas: ChosetSigmas;
  info: ScanInfoBonus | null;

  truth: Pose2;
  trail: Pose2[] = [];
  collisions = 0;
  steps = 0;
  reached = false;
  lastDecision: ReplanDecision | null = null;
  lastSamples: WeightedCell[] = [];
  lastScan: number[] = [];
  entropyHistory: number[] = [];

  constructor(opts: NavigatorOptions) {
    this.world = opts.world;
    this.rng = opts.rng;
    const cellSize = opts.cellSize ?? 0.4;
    this.lattice = gridFromWorld(opts.world, cellSize, opts.inflation ?? 0.3);
    this.graph = new GridGraph(this.lattice, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
    this.goalCell = freeCellNear(this.lattice, opts.goal.x, opts.goal.y);
    this.cfg = { lambda: opts.lambda ?? 20, betaInfo: opts.betaInfo ?? 0, actions: ACTIONS_8 };
    this.J = costToGo(this.graph, this.goalCell, this.cfg.lambda);
    this.kSamples = opts.kSamples ?? 24;
    this.planFromMode = opts.planFromMode ?? false;
    this.freezeBelief = opts.freezeBelief ?? false;
    this.belief = opts.belief;
    this.scanParams = { nBeams: 36, fov: 2 * Math.PI, maxRange: 8, sigma: 0.04, zRand: 0.02, zMax: 0.01, ...opts.scan };
    this.angles = beamAngles(this.scanParams);
    this.execSigmas = opts.execSigmas ?? [0.03, 0.01, 0.03, 0.01, 0.04, 0.01];
    this.info = opts.infoBonus === undefined ? new ScanInfoBonus(opts.world, this.lattice) : opts.infoBonus;
    const c = cellCenter(this.lattice, freeCellNear(this.lattice, opts.truth.x, opts.truth.y));
    this.truth = { x: c.x, y: c.y, theta: opts.truth.theta };
    this.entropyHistory.push(this.belief.entropyBits());
  }

  /** Change λ without rerunning Dijkstra. */
  setLambda(lambda: number): void {
    this.cfg = { ...this.cfg, lambda };
    this.J = this.J.withLambda(lambda);
  }

  /** The samples the planner will score on the next tick. */
  currentSamples(): WeightedCell[] {
    return this.planFromMode ? [{ cell: this.belief.mode(this.lattice), weight: 1 }] : this.belief.samples(this.kSamples, this.lattice, this.rng);
  }

  decide(samples: WeightedCell[]): ReplanDecision {
    return this.planFromMode
      ? planFromMode(this.J, samples[0].cell, this.cfg)
      : replanOnBelief(this.J, samples, this.cfg, this.cfg.betaInfo > 0 ? this.info ?? undefined : undefined);
  }

  /** One sense → update → replan → act cycle. */
  step(): NavigatorSnapshot {
    if (!this.reached) {
      const samples = this.currentSamples();
      const decision = this.decide(samples);
      this.lastSamples = samples;
      this.lastDecision = decision;

      // Act: one lattice step along the chosen action, as a holonomic disc.
      const [di, dj] = ACTION_STEP[decision.action];
      const dx = di * this.lattice.cellSize;
      const dy = dj * this.lattice.cellSize;
      const heading = Math.atan2(dy, dx);
      const nominal: Pose2 = { x: this.truth.x + dx, y: this.truth.y + dy, theta: heading };
      const u: ChosetIncrement = incrementBetween(this.truth, nominal);
      const realized = sampleChosetMotion(this.truth, u, this.execSigmas, this.rng);
      this.trail.push(this.truth);
      const bumped = collides(this.world, this.truth, realized) || !isFree(this.world, realized.x, realized.y, 0.12);
      if (bumped) {
        // Bumped: the wheels (and so the odometry) still report the move, but the
        // chassis stays — re-centred in its lattice cell, the Chapter 2 simulator's
        // stand-in for a bumper stop plus a low-level back-off. Without it a robot
        // that slip has carried a few centimetres off-centre can clip the same
        // wall edge on every retry, which the lattice has no way to express.
        this.collisions++;
        const home = cellCenter(this.lattice, freeCellNear(this.lattice, this.truth.x, this.truth.y));
        this.truth = { x: home.x, y: home.y, theta: heading };
      } else {
        this.truth = realized;
      }

      // Sense, then update — unless the belief is frozen, in which case it dead-reckons.
      // The bumper reading is folded in first, against the pre-move particles.
      if (!this.freezeBelief) this.belief.correctContact(u, bumped);
      this.lastScan = simulateScan(this.world, this.truth, this.scanParams, this.rng);
      this.belief.predict(u, this.rng);
      if (!this.freezeBelief) {
        this.belief.correct(this.lastScan, this.angles);
        if (this.belief.pf.effectiveSampleSize() < this.belief.size / 2) this.belief.resample(this.rng);
      }
      this.steps++;
      const g = cellCenter(this.lattice, this.goalCell);
      if (Math.hypot(this.truth.x - g.x, this.truth.y - g.y) < this.lattice.cellSize * 0.75) this.reached = true;
      this.entropyHistory.push(this.belief.entropyBits());
    }
    return this.snapshot();
  }

  snapshot(): NavigatorSnapshot {
    return {
      truth: this.truth,
      decision: this.lastDecision,
      samples: this.lastSamples,
      flipLambda: this.lastSamples.length > 0 ? nextFlipLambda(this.J, this.lastSamples, this.cfg) : null,
      entropyBits: this.entropyHistory[this.entropyHistory.length - 1],
      collisions: this.collisions,
      steps: this.steps,
      reached: this.reached,
    };
  }

  /** The route the mode would follow — drawn as the path, in the planning colour. */
  pathFromMode(): { x: number; y: number }[] {
    return this.J.pathFrom(this.belief.mode(this.lattice)).map((c) => cellCenter(this.lattice, c));
  }

  /** The cell Rusty is actually in. */
  truthCell(): number {
    return cellAt(this.lattice, this.truth.x, this.truth.y);
  }
}

/**
 * Run the certainty-equivalent planner from one known cell until the goal: the
 * path a point-mass belief produces. With one sample of weight one,
 * argmin_a [c(a) + J(f(x, a))] walks down J, so the path cost equals J(start) —
 * the check that `replan_on_belief` collapses to A* when nothing is uncertain.
 */
export function followBeliefPlan(J: CostToGo, start: number, maxSteps = 10_000): { cells: number[]; cost: number } {
  const cfg: ReplanConfig = { lambda: J.lambda, betaInfo: 0, actions: ACTIONS_8 };
  const cells = [start];
  let cost = 0;
  let cur = start;
  for (let n = 0; n < maxSteps && cur !== J.goal; n++) {
    const { action } = planFromMode(J, cur, cfg);
    if (!J.feasible(cur, action)) break;
    cost += J.stepCost(action);
    cur = successor(J.lattice, cur, action);
    cells.push(cur);
  }
  return { cells, cost };
}

/**
 * The default w16.2 scene. Rusty is truly in room B, flush under the corridor
 * wall at (5.0, 3.4); the belief says room A under A's open doorway with weight
 * 0.6 and room B with weight 0.4. From A, "Up" passes through the door; from B,
 * "Up" is the wall — so the mode says Up and the belief says sideways, which is
 * the 3×3 micro-example with the Apartment's walls. The goal is the west end of
 * the corridor. λ = 20 exceeds every cost-to-go on this lattice.
 */
export function defaultNavigator(world: World, seed: number, overrides: Partial<NavigatorOptions> = {}): BeliefNavigator {
  const rng = new Rng(seed);
  const belief = ParticleBelief.clustered(
    world,
    rng.fork(),
    overrides.belief?.size ?? 400,
    [
      { pose: { x: 2.0, y: 3.4, theta: Math.PI / 2 }, weight: 0.6, std: 0.18 },
      { pose: { x: 5.0, y: 3.4, theta: Math.PI / 2 }, weight: 0.4, std: 0.18 },
    ],
    { every: 3, sigmas: DEFAULT_CHOSET_SIGMAS, surprise: null },
  );
  return new BeliefNavigator({
    world,
    rng,
    truth: { x: 5.0, y: 3.4, theta: Math.PI / 2 },
    goal: { x: 1.0, y: 4.4 },
    belief,
    ...overrides,
  });
}

/**
 * The chapter's hook (f16.0): Rusty is switched on in room B but was last seen in
 * room A. `gaussian` is the Chapter 15 belief a position tracker would hand the
 * planner — a tight cluster at the wrong room, planned from its mode and never
 * corrected (`freezeBelief`), so the odometry carries it into the corridor while
 * the chassis bumps B's wall. `cloud` is global localization: 800 particles
 * uniform over free space, every third beam (Choset's 60-of-181 remedy for the
 * false independence of (9.25)), Augmented MCL plus Lenser and Veloso's
 * sensor-resetting floor, replanned every tick on the expected cost-to-go. Both
 * share the truth and the goal; the check file pins that one bumps and the other arrives.
 */
export function kidnappedPair(world: World, seed: number): { truth: Pose2; gaussian: BeliefNavigator; cloud: BeliefNavigator } {
  const rng = new Rng(seed);
  const goal = { x: 1.0, y: 4.4 };
  const believed: Pose2 = { x: 2.0, y: 2.0, theta: Math.PI / 2 };
  // Kidnapped: room B, between the kitchen counter and the corridor wall, left of B's door.
  const truth: Pose2 = { x: rng.uniform(4.6, 5.2), y: rng.uniform(2.4, 3.0), theta: Math.PI / 2 };
  const confident = ParticleBelief.clustered(world, rng.fork(), 120, [{ pose: believed, weight: 1, std: 0.1 }], { every: 3, sigmas: DEFAULT_CHOSET_SIGMAS, surprise: null });
  const gaussian = new BeliefNavigator({ world, rng: rng.fork(), truth, goal, belief: confident, planFromMode: true, freezeBelief: true, lambda: 20 });
  const uniform = new ParticleBelief(world, rng.fork(), { n: 800, every: 3, resetFloor: 0.03, resetFraction: 0.1 });
  const cloud = new BeliefNavigator({ world, rng: rng.fork(), truth, goal, belief: uniform, lambda: 20, kSamples: 64 });
  return { truth, gaussian, cloud };
}
