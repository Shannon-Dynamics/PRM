/**
 * `reach_stack` — Reach on the Workbench, one pick after another.
 *
 *   goal (tip target) → singularity guard (Ch. 4 Jacobian, F3)
 *     → RRT-Connect (Ch. 12) for a first feasible path
 *     → Informed RRT* (Ch. 13), anytime within an iteration budget
 *     → greedy shortcut (Ch. 11 Steer)
 *     → time-optimal time scaling (Ch. 18)          → T₁
 *     → CHOMP smoothing (Ch. 19) → time scaling again → T₂
 *     → verify (Ch. 2 swept checker), else fall back to the T₁ trajectory
 *     → execute the Trajectory, then a straight 12 cm Cartesian approach by
 *       resolved rates q̇ = J⁻¹ ẋ (Ch. 4) — the move a singular goal breaks.
 *
 * Every stage is time-sliced: a stage does a bounded chunk of work per 50 ms
 * tick (40 tree iterations, 10 CHOMP steps), so the widget never blocks a
 * frame on a planner. The composed guarantee is probabilistic completeness —
 * RRT-Connect's — and it survives CHOMP only because CHOMP's output is checked
 * and discarded when it collides (D23.3).
 */

import { Rng } from '../prob/rng';
import type { Vec } from '../prob/linalg';
import { solve } from '../prob/linalg';
import type { Point2 } from '../sim/world';
import { REACH, type Tn } from '../robots/reach';
import { WORKBENCH } from '../robots/worlds';
import { Scene } from '../collide/scene';
import { ChainChecker } from '../collide/collision';
import { jacobian2 } from '../cspace/jacobian';
import { T2 } from '../manifold/torus';
import { chainFreeSpace, type CountingFreeSpace } from '../sampling/free-space';
import { StraightLine, shortcutGreedy } from '../sampling/steer';
import { RrtConnect } from '../sampling/tree';
import { InformedRrtStar, gammaStarSolovey2020 } from '../sampling/optimal';
import { LearnedSampler, mixtureSampler } from '../learn/sampler';
import { encodeWorld } from '../learn/worlds';
import { loadModels } from '../learn/models';
import { Reach2R } from '../dynamics/reach2r';
import { PathDynamics } from '../trajectory/constrained';
import { CubicSpline, Segment, unwrapWaypoints, type Path as GeoPath } from '../trajectory/path';
import { symmetricLimits, type TorqueLimits } from '../trajectory/limits';
import { intoTrajectory, timeScale, type ScaledSample, type TimeScaling } from '../trajectory/time-scale';
import type { Trajectory } from '../trajectory/trajectory';
import { GridField } from '../trajopt/field';
import { chomp, chompStableEta, resamplePolyline, type ChompProblem } from '../trajopt/chomp';
import { reachBodyPoints } from '../trajopt/micro';
import { Bus, stamp, type SimTime } from './bus';
import { Guarantee, RoundRobin, type StageInfo } from './stage';
import { armConditioning, singularityGuard } from './guards';
import { PLAN, modeName, supervisorStep, type Mode, type SupervisorEvent } from './supervisor';
import { APPROACH_LEN, REACH_HOME, REACH_TASKS, fullStretch, type ReachTask } from './worlds';
import type { RustyEventLog } from './rusty';

const BASE_MS = 50;
export const REACH_LIMITS: TorqueLimits = symmetricLimits([20, 12]);
/** μ_min: below |sin θ₂| = 0.25 (θ₂ < 14.5°) the approach's joint rates exceed 1/σ_min amplification we allow. */
export const MU_MIN = 0.25;
/** How far the guard pulls a singular target toward the base (F3's δ). */
export const PULL_DELTA = 0.15;
/** Joint-rate limit for the resolved-rate approach, rad/s. */
export const QDOT_MAX = 2;
/** 12 ticks for 12 cm: the approach runs at ‖ẋ‖ = 0.2 m/s, so F3's bound ‖q̇‖ ≤ ‖ẋ‖/σ_min stays under QDOT_MAX whenever μ ≥ μ_min. */
export const APPROACH_TICKS = 12;
const RRT_BUDGET = 400;
const CONNECT_MAX = 3000;
const SLICE = 40;
const CHOMP_ITERS = 30;
const CHOMP_SLICE = 10;
const WAYPOINTS = 24;

export const REACH_STAGES: StageInfo[] = [
  {
    id: 'reach.guard', robot: 'reach', label: 'Singularity guard', chapter: 4, slug: 'ch04-configuration-space-i', periodMs: BASE_MS,
    role: 'guard', guarantee: Guarantee.None, input: 'Stamped<Tip> (bench)', output: 'Stamped<T2> goal (torus)',
    degradation: 'A full-stretch goal reaches the approach: q̇ = J⁻¹ẋ blows up as σ_min → 0 and the joint-rate limit ends the task.',
  },
  {
    id: 'reach.plan', robot: 'reach', label: 'RRT-Connect → Informed RRT*', chapter: 13, slug: 'ch13-optimal-sampling', periodMs: BASE_MS,
    role: 'rejecting', guarantee: Guarantee.ProbabilisticallyComplete, input: 'goal (torus)', output: 'Stamped<Path<T2>> (torus)',
    degradation: 'No path: Reach never leaves the pose it is in.',
  },
  {
    id: 'reach.scale', robot: 'reach', label: 'Time scaling', chapter: 18, slug: 'ch18-trajectory-planning', periodMs: BASE_MS,
    role: 'verified', guarantee: Guarantee.ProbabilisticallyComplete, input: 'Path<T2>', output: 'Stamped<Trajectory<T2>> (torus)',
    degradation: 'The tracker is handed a Path: in Rust that is error[E0308] at compile time; here the tracker refuses it and the task fails.',
  },
  {
    id: 'reach.chomp', robot: 'reach', label: 'CHOMP smoothing', chapter: 19, slug: 'ch19-trajectory-optimization', periodMs: BASE_MS,
    role: 'verified', guarantee: Guarantee.ProbabilisticallyComplete, input: 'Trajectory<T2>', output: 'Stamped<Trajectory<T2>> (torus)',
    degradation: 'No smoothing: the shortcut path is executed as is — correct, a little slower.',
  },
  {
    id: 'reach.verify', robot: 'reach', label: 'Verify (swept check)', chapter: 2, slug: 'ch02-the-lab', periodMs: BASE_MS,
    role: 'guard', guarantee: Guarantee.None, input: 'smoothed + fallback Trajectory', output: 'Stamped<Trajectory<T2>> (torus)',
    degradation: 'CHOMP becomes an unverified post-processor: the composed guarantee drops to none, whether or not this seed collides.',
  },
  {
    id: 'reach.track', robot: 'reach', label: 'Tracker + approach', chapter: 17, slug: 'ch17-robot-dynamics', periodMs: BASE_MS,
    role: 'executor', guarantee: Guarantee.None, input: 'Trajectory<T2>', output: 'Stamped<Torque> (torus)',
    degradation: 'Nothing executes: every trajectory is verified and the arm stays still.',
  },
];

/** A Path and a Trajectory travel as different message kinds; only one of them can be executed (D23.1). */
export type ReachMotion = { kind: 'Path'; path: number[][] } | { kind: 'Trajectory'; traj: Trajectory<Vec> & { samples: ScaledSample[] }; scaling: TimeScaling | null; source: string };

type Phase = 'guard' | 'connect' | 'rrtstar' | 'shortcut' | 'scale' | 'chomp' | 'verify' | 'execute' | 'approach' | 'dwell' | 'idle';

export interface ReachTaskRecord {
  task: number;
  name: string;
  tip: Point2;
  /** Path cost in rad (ℓ² on T², summed along the executed geometric path). */
  cost: number;
  durationBefore: number;
  durationAfter: number;
  chompAccepted: boolean;
  /** max |τ_i|/τ_max,i at the time-scaling's grid nodes — where Choset's construction is exact. */
  maxTorqueRatio: number;
  /** The same ratio on the 50 ms replay samples (interpolation between nodes overshoots a little). */
  replayTorqueRatio: number;
  guardFired: boolean;
  nodes: number;
}

export interface ReachOptions {
  seed: number;
  /** Simulated time after which the next task is pushed to full stretch, or null. */
  singularAt: SimTime | null;
  disabled?: readonly { id: string; from: SimTime }[];
  /** Number of picks before the arm is done. */
  tasks?: number;
  /** Chapter 22's toggle: RRT-Connect samples from (1 − λ) p̂_θ + λ u; null = uniform. */
  learnedLambda?: number | null;
}

/** Path length on T² of a configuration sequence. */
export function torusLength(pts: readonly number[][]): number {
  let L = 0;
  for (let i = 1; i < pts.length; i++) L += T2.dist(pts[i - 1], pts[i]);
  return L;
}

export class ReachStack {
  readonly bus: Bus;
  readonly opts: ReachOptions;
  readonly arm = REACH;
  readonly scene = Scene.fromWorld(WORKBENCH);
  readonly checker: ChainChecker;
  readonly free: CountingFreeSpace<number[]>;
  readonly dyn = Reach2R.micro(0);
  readonly field: GridField;
  readonly stages = REACH_STAGES;

  q: number[] = [...REACH_HOME];
  qd: number[] = [0, 0];
  trail: number[][] = [];
  mode: Mode = PLAN;
  phase: Phase = 'guard';
  taskIndex = 0;
  tip: Point2 = REACH_TASKS[0].tip;
  qGoal: number[] | null = null;
  qPre: number[] | null = null;
  mu = 0;
  sigmaMin = 0;
  guardFiredThisTask = false;
  singularPending = false;
  singularApplied = false;
  /** λ of the learned-sampler mixture, or null for uniform (applies from the next pick). */
  learnedLambda: number | null;

  connect: RrtConnect<number[]> | null = null;
  rrt: InformedRrtStar<number[]> | null = null;
  geoPath: number[][] | null = null;
  shortPath: number[][] | null = null;
  traj1: (Trajectory<Vec> & { samples: ScaledSample[] }) | null = null;
  scaling1: TimeScaling | null = null;
  traj2: (Trajectory<Vec> & { samples: ScaledSample[] }) | null = null;
  scaling2: TimeScaling | null = null;
  /** The geometric paths the two scalings were computed on (null for the segment fallback). */
  path1: GeoPath | null = null;
  path2: GeoPath | null = null;
  chompXi: Vec[] | null = null;
  chompDone = 0;
  private chompProblem: ChompProblem | null = null;
  executing: ReachMotion | null = null;
  execIndex = 0;
  approachStep = 0;
  approachFrom: Point2 | null = null;
  dwell = 0;
  readonly records: ReachTaskRecord[] = [];
  readonly log: RustyEventLog[] = [];
  readonly executed: number[][] = [];
  collisions = 0;
  private record: Partial<ReachTaskRecord> = {};

  constructor(bus: Bus, opts: ReachOptions) {
    this.bus = bus;
    this.opts = opts;
    this.checker = new ChainChecker(this.scene, this.arm);
    this.free = chainFreeSpace(this.scene, this.arm);
    this.field = GridField.fromPolyWorld(WORKBENCH, 0.05);
    this.executed.push(this.q.slice());
    this.learnedLambda = opts.learnedLambda ?? null;
  }

  get totalTasks(): number {
    return this.opts.tasks ?? REACH_TASKS.length;
  }

  private disabled(id: string, now: SimTime): boolean {
    return (this.opts.disabled ?? []).some((d) => d.id === id && now >= d.from);
  }

  private event(t: SimTime, text: string, kind: RustyEventLog['kind'] = 'info'): void {
    this.log.push({ t, text, kind });
  }

  private sup(t: SimTime, ev: SupervisorEvent): void {
    const before = modeName(this.mode);
    this.mode = supervisorStep(this.mode, ev);
    const after = modeName(this.mode);
    if (after !== before) this.event(t, `${before} → ${after}`, this.mode.kind === 'Failed' ? 'fail' : this.mode.kind === 'Done' ? 'done' : 'mode');
  }

  register(rr: RoundRobin): void {
    const [guard, plan, scale, ch, verify, track] = this.stages;
    rr.add(guard, (now) => this.guardStage(now));
    rr.add(plan, (now) => this.planStage(now));
    rr.add(scale, (now) => this.scaleStage(now));
    rr.add(ch, (now) => this.chompStage(now));
    rr.add(verify, (now) => this.verifyStage(now));
    rr.add(track, (now) => this.trackStage(now));
  }

  worldTick(now: SimTime): void {
    if (this.opts.singularAt !== null && !this.singularPending && now >= this.opts.singularAt) {
      this.singularPending = true;
      this.event(now, 'chaos: the next pick is pushed out to full stretch', 'chaos');
    }
  }

  /** The *Singular Goal* chaos button: the next pick the guard sees is pushed to full stretch. */
  pushSingular(now: SimTime): void {
    if (this.done || this.mode.kind === 'Failed') return;
    this.singularPending = true;
    this.singularApplied = false;
    this.event(now, 'chaos: the next pick is pushed out to full stretch', 'chaos');
  }

  /** The configuration-space solution for a tip: both IK branches, keep the free one nearest `near`. */
  private solveIk(tip: Point2, near: number[]): number[] | null {
    const sols = this.arm.ik(tip).filter((q): q is readonly [number, number] => q !== null).map((q) => [q[0], q[1]]);
    const ok = sols.filter((q) => this.checker.isFree(q));
    if (ok.length === 0) return null;
    ok.sort((a, b) => T2.dist(a, near) - T2.dist(b, near));
    return ok[0];
  }

  /* ---------------------------------------------------------------- stages */

  guardStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.phase !== 'guard' || this.mode.kind === 'Failed' || this.mode.kind === 'Done') return { emitted: false, inputAge: NaN };
    if (this.singularPending && !this.singularApplied) {
      this.tip = fullStretch(this.tip);
      this.singularApplied = true;
    }
    const r = Math.hypot(this.tip.x, this.tip.y) || 1;
    const radial = { x: this.tip.x / r, y: this.tip.y / r };
    const pre = { x: this.tip.x - APPROACH_LEN * radial.x, y: this.tip.y - APPROACH_LEN * radial.y };
    const qPre = this.solveIk(pre, this.q);
    const qGoal = qPre ? this.solveIk(this.tip, qPre) : null;
    if (!qPre || !qGoal) {
      this.sup(now, { kind: 'NoPath', recover: null });
      return { emitted: false, inputAge: 0 };
    }
    const c = armConditioning(this.arm, qGoal);
    this.mu = c.mu;
    this.sigmaMin = c.sigmaMin;
    if (!this.disabled('reach.guard', now)) {
      const g = singularityGuard(this.arm, qGoal, MU_MIN);
      if (!g.ok) {
        this.guardFiredThisTask = true;
        this.event(now, `singularity guard: μ(q_goal) = ${g.err.mu.toFixed(3)} < μ_min = ${MU_MIN} (σ_min = ${g.err.sigmaMin.toFixed(3)})`, 'detect');
        this.sup(now, { kind: 'Singular', delta: PULL_DELTA });
        // Recover(PullGoalInward): move the target δ toward the base and try the guard again.
        this.tip = { x: this.tip.x - PULL_DELTA * radial.x, y: this.tip.y - PULL_DELTA * radial.y };
        this.sup(now, { kind: 'Recovered' });
        return { emitted: false, inputAge: 0 };
      }
    }
    this.qGoal = qGoal;
    this.qPre = qPre;
    this.bus.publish('reach/goal', stamp(now, 'torus', { qGoal, qPre, tip: this.tip }));
    this.phase = 'connect';
    const seed = this.opts.seed * 97 + this.taskIndex * 13 + 5;
    const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.04 });
    // Chapter 22, behind its floor: the learned proposal only ever ships inside a mixture with λ > 0.
    const sampler = this.learnedLambda !== null ? mixtureSampler(new LearnedSampler(loadModels().sampler, encodeWorld(WORKBENCH), this.q, qPre), this.learnedLambda) : undefined;
    this.connect = new RrtConnect<number[]>(T2, this.free, this.q.slice(), qPre, { steer, eta: 0.4, greed: ['extend', 'connect'], sampler }, new Rng(seed));
    this.rrt = new InformedRrtStar<number[]>(T2, this.free, this.q.slice(), qPre, { steer, eta: 0.4, goalBias: 0.05, goalRadius: 0.4, nbhd: { kind: 'radius', gamma: gammaStarSolovey2020(2, (2 * Math.PI) ** 2 * 0.7) * 1.1 } }, new Rng(seed + 1));
    this.record = { task: this.records.length, name: REACH_TASKS[this.taskIndex % REACH_TASKS.length].name, tip: this.tip, guardFired: this.guardFiredThisTask };
    return { emitted: true, inputAge: 0 };
  }

  planStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.disabled('reach.plan', now)) return { emitted: false, inputAge: NaN };
    if (this.phase === 'connect' && this.connect) {
      for (let i = 0; i < SLICE && !this.connect.solved && this.connect.attempts < CONNECT_MAX; i++) this.connect.step();
      if (this.connect.solved) this.phase = 'rrtstar';
      else if (this.connect.attempts >= CONNECT_MAX) {
        this.sup(now, { kind: 'NoPath', recover: null });
        this.phase = 'idle';
      }
      return { emitted: false, inputAge: 0 };
    }
    if (this.phase === 'rrtstar' && this.rrt) {
      for (let i = 0; i < SLICE && this.rrt.iterations < RRT_BUDGET; i++) this.rrt.iterate();
      if (this.rrt.iterations < RRT_BUDGET) return { emitted: false, inputAge: 0 };
      const viaConnect = this.connect?.path() ?? null;
      const viaStar = this.rrt.bestPath();
      const cConnect = viaConnect ? torusLength(viaConnect) : Infinity;
      const cStar = viaStar ? torusLength(viaStar) : Infinity;
      this.geoPath = (cStar <= cConnect ? viaStar : viaConnect)!.map((q) => q.slice());
      this.record.nodes = this.rrt.tree.size + (this.connect?.nodeCount ?? 0);
      // Chapter 11's postprocessing, then hand the geometric curve downstream.
      const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.04 });
      this.shortPath = shortcutGreedy(steer, T2, this.free, this.geoPath);
      this.record.cost = torusLength(this.shortPath);
      this.event(now, `plan: RRT-Connect ${cConnect.toFixed(2)} rad, RRT* ${cStar.toFixed(2)} rad → shortcut ${this.record.cost.toFixed(2)} rad`);
      this.bus.publish<ReachMotion>('reach/path', stamp(now, 'torus', { kind: 'Path', path: this.shortPath }));
      this.phase = 'scale';
      return { emitted: true, inputAge: 0 };
    }
    return { emitted: false, inputAge: NaN };
  }

  /** A spline that hugs the polyline (resampled to 26 knots), or null when even that cuts a corner. */
  private hugSpline(poly: readonly number[][]): CubicSpline | null {
    const lifted = unwrapWaypoints(poly);
    const { q0, qEnd, xi } = resamplePolyline(lifted, WAYPOINTS);
    const spline = new CubicSpline([q0, ...xi, qEnd], false);
    return this.pathFree(spline) ? spline : null;
  }

  /** Dense swept check of a geometric path: consecutive samples by Chapter 2's sphere marching. */
  pathFree(p: GeoPath, n = 160): boolean {
    let prev = p.q(0);
    for (let k = 1; k <= n; k++) {
      const q = p.q(k / n);
      if (!this.checker.edgeFree(prev, q)) return false;
      prev = q;
    }
    return true;
  }

  /** Time-scale a path, or the polyline segment by segment rest-to-rest when the spline would collide. */
  private scale(path: GeoPath | null, poly: readonly number[][]): { traj: Trajectory<Vec> & { samples: ScaledSample[] }; scaling: TimeScaling | null; source: string; path: GeoPath | null } | null {
    if (path) {
      const res = timeScale(new PathDynamics(this.dyn, path), REACH_LIMITS, { N: 400 });
      if (res.ok) return { traj: intoTrajectory(res.scaling, new PathDynamics(this.dyn, path), BASE_MS / 1000), scaling: res.scaling, source: 'spline', path };
    }
    // Fallback: each straight chart segment, rest to rest. Straight segments were verified by the planner's Steer.
    const lifted = unwrapWaypoints(poly);
    const samples: ScaledSample[] = [];
    let t0 = 0;
    for (let i = 0; i + 1 < lifted.length; i++) {
      const pd = new PathDynamics(this.dyn, new Segment(lifted[i], lifted[i + 1]));
      const res = timeScale(pd, REACH_LIMITS, { N: 60 });
      if (!res.ok) return null;
      const tr = intoTrajectory(res.scaling, pd, BASE_MS / 1000);
      for (const s of tr.samples) if (samples.length === 0 || s.t > 0) samples.push({ ...s, t: s.t + t0 });
      t0 += tr.tf;
    }
    return { traj: { samples, tf: t0 }, scaling: null, source: 'segments', path: null };
  }

  scaleStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.phase !== 'scale' || !this.shortPath) return { emitted: false, inputAge: NaN };
    if (this.disabled('reach.scale', now)) {
      // The tracker gets the geometric path itself — the D23.1 violation the type system forbids.
      this.executing = { kind: 'Path', path: this.shortPath };
      this.phase = 'execute';
      this.sup(now, { kind: 'PlanReady' });
      return { emitted: false, inputAge: 0 };
    }
    const spline = this.hugSpline(this.shortPath);
    const out = this.scale(spline, this.shortPath);
    if (!out) {
      this.sup(now, { kind: 'Fault', reason: 'NoPath', detail: 'time scaling found no admissible profile' });
      this.phase = 'idle';
      return { emitted: false, inputAge: 0 };
    }
    this.traj1 = out.traj;
    this.scaling1 = out.scaling;
    this.path1 = out.path;
    this.record.durationBefore = out.traj.tf;
    this.event(now, `time scaling: t_f = ${out.traj.tf.toFixed(2)} s (${out.source})`);
    this.bus.publish<ReachMotion>('reach/traj', stamp(now, 'torus', { kind: 'Trajectory', traj: out.traj, scaling: out.scaling, source: 'time-scaled shortcut' }));
    // Seed CHOMP with the same 24 interior waypoints the spline was fitted to.
    const lifted = unwrapWaypoints(this.shortPath);
    const { q0, qEnd, xi } = resamplePolyline(lifted, WAYPOINTS);
    const lambdaObs = 1;
    const eps = 0.1;
    this.chompProblem = { q0, qEnd, field: this.field, body: reachBodyPoints(this.arm, 4), eps, lambdaObs, eta: chompStableEta(WAYPOINTS, lambdaObs, eps) * 0.25, obstacleForm: 'pointwise' };
    this.chompXi = xi;
    this.chompDone = 0;
    this.phase = 'chomp';
    return { emitted: true, inputAge: 0 };
  }

  chompStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.phase !== 'chomp' || !this.chompProblem || !this.chompXi) return { emitted: false, inputAge: NaN };
    if (this.disabled('reach.chomp', now)) {
      this.traj2 = null;
      this.phase = 'verify';
      return { emitted: false, inputAge: 0 };
    }
    const res = chomp(this.chompProblem, this.chompXi, CHOMP_SLICE);
    this.chompXi = res.xi;
    this.chompDone += CHOMP_SLICE;
    if (this.chompDone < CHOMP_ITERS) return { emitted: false, inputAge: 0 };
    const p = this.chompProblem;
    const spline = new CubicSpline([p.q0, ...this.chompXi, p.qEnd], false);
    const res2 = timeScale(new PathDynamics(this.dyn, spline), REACH_LIMITS, { N: 400 });
    if (res2.ok) {
      this.traj2 = intoTrajectory(res2.scaling, new PathDynamics(this.dyn, spline), BASE_MS / 1000);
      this.scaling2 = res2.scaling;
      this.path2 = spline;
      this.event(now, `CHOMP: ${CHOMP_ITERS} covariant steps → t_f = ${this.traj2.tf.toFixed(2)} s`);
      this.bus.publish<ReachMotion>('reach/smooth', stamp(now, 'torus', { kind: 'Trajectory', traj: this.traj2, scaling: res2.scaling, source: 'CHOMP' }));
    } else {
      this.traj2 = null;
    }
    this.phase = 'verify';
    return { emitted: this.traj2 !== null, inputAge: 0 };
  }

  /** CHOMP's current waypoints with their fixed ends, wrapped to the chart (for drawing), or null. */
  chompProblemPoints(): number[][] | null {
    if (!this.chompProblem || !this.chompXi || this.phase !== 'chomp') return null;
    return [this.chompProblem.q0, ...this.chompXi, this.chompProblem.qEnd].map((q) => T2.wrap(q));
  }

  /** Dense recheck of a trajectory's samples, consecutive pairs by the swept checker. */
  trajectoryFree(traj: Trajectory<Vec>): boolean {
    const s = traj.samples;
    for (let i = 1; i < s.length; i++) if (!this.checker.edgeFree(s[i - 1].q, s[i].q)) return false;
    return s.length > 0 && this.checker.isFree(s[0].q);
  }

  verifyStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.phase !== 'verify' || !this.traj1) return { emitted: false, inputAge: NaN };
    let chosen: Trajectory<Vec> & { samples: ScaledSample[] } = this.traj1;
    let accepted = false;
    if (this.traj2) {
      if (this.disabled('reach.verify', now)) {
        chosen = this.traj2;
        accepted = true;
        this.event(now, 'verify is off: CHOMP output executed unchecked', 'detect');
      } else if (this.trajectoryFree(this.traj2)) {
        chosen = this.traj2;
        accepted = true;
      } else {
        this.event(now, 'verify: the CHOMP trajectory touches an obstacle — falling back to the time-scaled shortcut', 'detect');
      }
    }
    this.record.chompAccepted = accepted;
    this.record.durationAfter = chosen.tf;
    let replay = 0;
    for (const s of chosen.samples) for (let i = 0; i < 2; i++) replay = Math.max(replay, Math.abs(s.u[i]) / REACH_LIMITS.max[i]);
    const sc = accepted ? this.scaling2 : this.scaling1;
    const gp = accepted ? this.path2 : this.path1;
    let nodes = replay;
    if (sc && gp) {
      const pd = new PathDynamics(this.dyn, gp);
      nodes = 0;
      for (let k = 0; k <= sc.N; k++) {
        const u = pd.torque(sc.s[k], sc.sdot[k], sc.sddot[k]);
        for (let i = 0; i < 2; i++) nodes = Math.max(nodes, Math.abs(u[i]) / REACH_LIMITS.max[i]);
      }
    }
    this.record.maxTorqueRatio = nodes;
    this.record.replayTorqueRatio = replay;
    this.executing = { kind: 'Trajectory', traj: chosen as Trajectory<Vec> & { samples: ScaledSample[] }, scaling: accepted ? this.scaling2 : this.scaling1, source: accepted ? 'CHOMP' : 'shortcut' };
    this.bus.publish<ReachMotion>('reach/verified', stamp(now, 'torus', this.executing));
    this.execIndex = 0;
    this.phase = 'execute';
    this.sup(now, { kind: 'PlanReady' });
    return { emitted: true, inputAge: 0 };
  }

  trackStage(now: SimTime): { emitted: boolean; inputAge: number } {
    const off = this.disabled('reach.track', now);
    if (off || this.mode.kind === 'Failed') return { emitted: false, inputAge: NaN };
    if (this.phase === 'execute' && this.executing) {
      if (this.executing.kind === 'Path') {
        this.sup(now, { kind: 'Fault', reason: 'PathNotTrajectory', detail: 'error[E0308]: mismatched types — expected `Trajectory<Torus<2>>`, found `Path<Torus<2>>`' });
        this.phase = 'idle';
        return { emitted: false, inputAge: 0 };
      }
      const s = this.executing.traj.samples;
      const k = Math.min(this.execIndex, s.length - 1);
      this.moveTo(s[k].q, s[k].qd, now);
      this.execIndex++;
      if (this.execIndex >= s.length) {
        this.phase = 'approach';
        this.approachStep = 0;
        this.approachFrom = this.arm.tip(this.q);
      }
      return { emitted: true, inputAge: 0 };
    }
    if (this.phase === 'approach' && this.approachFrom) {
      // Resolved rates: q̇ = J(q)⁻¹ ẋ along the straight approach, with a little position feedback.
      const dt = BASE_MS / 1000;
      this.approachStep++;
      const f = Math.min(1, this.approachStep / APPROACH_TICKS);
      const target = { x: this.approachFrom.x + f * (this.tip.x - this.approachFrom.x), y: this.approachFrom.y + f * (this.tip.y - this.approachFrom.y) };
      const cur = this.arm.tip(this.q);
      const xdot = [(target.x - cur.x) / dt, (target.y - cur.y) / dt];
      const J = jacobian2(this.arm, this.q);
      let qdot: number[];
      try {
        qdot = solve([[J[0][0], J[0][1]], [J[1][0], J[1][1]]], xdot);
      } catch {
        qdot = [Infinity, Infinity];
      }
      const rate = Math.max(Math.abs(qdot[0]), Math.abs(qdot[1]));
      if (!Number.isFinite(rate) || rate > QDOT_MAX) {
        this.event(now, `approach: |q̇| = ${Number.isFinite(rate) ? rate.toFixed(1) : '∞'} rad/s exceeds ${QDOT_MAX} rad/s at μ = ${armConditioning(this.arm, this.q).mu.toFixed(3)}`, 'fail');
        this.sup(now, { kind: 'Fault', reason: 'JointRate', detail: 'resolved-rate approach through a singularity' });
        this.phase = 'idle';
        return { emitted: false, inputAge: 0 };
      }
      this.moveTo([this.q[0] + qdot[0] * dt, this.q[1] + qdot[1] * dt], qdot, now);
      if (this.approachStep >= APPROACH_TICKS) {
        this.records.push(this.record as ReachTaskRecord);
        this.event(now, `pick ${this.records.length} done at (${this.tip.x.toFixed(2)}, ${this.tip.y.toFixed(2)})`, 'done');
        this.sup(now, { kind: 'Arrived' });
        this.phase = 'dwell';
        this.dwell = 0;
      }
      return { emitted: true, inputAge: 0 };
    }
    if (this.phase === 'dwell') {
      this.dwell++;
      this.qd = [0, 0];
      if (this.dwell >= 10 && this.records.length < this.totalTasks) {
        this.taskIndex++;
        const base = REACH_TASKS[this.taskIndex % REACH_TASKS.length].tip;
        this.tip = base;
        this.guardFiredThisTask = false;
        this.phase = 'guard';
        this.sup(now, { kind: 'NextTask' });
      }
    }
    return { emitted: false, inputAge: NaN };
  }

  private moveTo(q: readonly number[], qd: readonly number[], now: SimTime): void {
    const next = [q[0], q[1]];
    if (!this.checker.edgeFree(this.q, next)) {
      this.collisions++;
      this.event(now, 'contact: the arm touched an obstacle', 'fail');
      this.sup(now, { kind: 'Fault', reason: 'Collision', detail: 'executed motion intersects the Workbench' });
      this.phase = 'idle';
      return;
    }
    this.q = next;
    this.qd = [qd[0], qd[1]];
    this.executed.push(next.slice());
  }

  get done(): boolean {
    return this.records.length >= this.totalTasks;
  }
}
