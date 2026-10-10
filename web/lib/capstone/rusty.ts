/**
 * `rusty_stack` — Rusty in an Apartment it has not seen furnished.
 *
 *   scan (Ch. 2/3, 10 Hz) → belief (Ch. 16 particle localizer, 10 Hz)
 *     → map (Ch. 4 disc inflation of confirmed returns, 5 Hz)
 *     → plan (Ch. 6 D* Lite repair + Ch. 16 replan_on_belief check, 2 Hz)
 *     → D23.2 validity check → track (Ch. 19 receding-horizon iLQR, 20 Hz)
 *   fallback: Ch. 3 Tangent Bug when the grid has no path.
 *
 * Every algorithm is an import. What this file adds is composition: which
 * stamped message each stage reads, what it publishes, and what the supervisor
 * does when a statistic says the plan is no longer valid.
 *
 * The planner's map starts as the *blueprint* inflated by r + margin (Chapter
 * 4's reduction of a disc to a point, computed by Chapter 6's `gridFromWorld`
 * on the ported exact distance transform) with every door assumed open — D*
 * Lite's free-space assumption. A cell becomes an obstacle only when two
 * consecutive scans, projected through the *belief* (never the truth), put a
 * return in it that the blueprint does not explain.
 */

import type { Pose2 } from '../geom/se2';
import { Rng } from '../prob/rng';
import type { Vec } from '../prob/linalg';
import { beamAngles, collides, isFree, simulateScan, type Point2, type ScanParams, type World } from '../sim/world';
import { distanceAt, exactDistanceField, type ExactDistanceField } from '../mapping/edt';
import { RUSTY_RADIUS } from '../robots/rusty';
import { DStarLite } from '../search/dstar-lite';
import { astar } from '../search/best-first';
import { GridGraph, cellAt, cellCenter, cellIJ, cloneGrid, freeCellNear, gridFromWorld, gridHeuristic, type OccupancyGrid } from '../search/grid';
import { ACTION_STEP, ACTIONS_8, CostToGo, replanOnBelief, type Action, type ReplanDecision } from '../estimate/replan';
import { DEFAULT_CHOSET_SIGMAS, LOCALIZATION_PROXIMITY, ParticleBelief, incrementBetween } from '../estimate/bayes';
import { Mpc } from '../trajopt/mpc';
import { Unicycle } from '../trajopt/models';
import type { RunningDerivs, StageCost, TerminalDerivs } from '../trajopt/ilqr';
import { TangentBug } from '../bugs/tangent';
import { R2 } from '../manifold/rd';
import { StraightLine, shortcutGreedy } from '../sampling/steer';
import { CountingFreeSpace } from '../sampling/free-space';
import { Bus, ageOf, stamp, type SimTime, type Stamped } from './bus';
import { Guarantee, RoundRobin, type StageInfo } from './stage';
import { replanningBudget, type RateTable, RUSTY_A_MAX, RUSTY_V_MAX, D_DETECT } from './budget';
import { corridorBlocked, pointAtArc, polylineLength, projectOnPath } from './guards';
import { PLAN, modeName, supervisorStep, type Mode, type SupervisorEvent } from './supervisor';
import { BLUEPRINT, DOORS, RUSTY_GOAL, RUSTY_START, trueApartment, type Furniture } from './worlds';
import { segmentsIntersect, type Segment } from '../sim/world';

export const BASE_MS = 50;

/** Inflation: Rusty's radius plus a 0.15 m margin — three standard deviations of a 5 cm localization error (Ch. 15). */
export const RUSTY_MARGIN = 0.15;
export const RUSTY_INFLATION = RUSTY_RADIUS + RUSTY_MARGIN;
export const RUSTY_CELL = 0.1;
/** A return the blueprint explains to within this distance is a known wall, not news. */
const EXPLAINED = 0.15;
/**
 * The map trusts a projected return only while the belief is localized:
 * σ ≤ 5 cm, the budget behind the 0.15 m (= 3σ) inflation margin. Projecting
 * scans through a loose belief paints walls where there are none — F1's
 * "given a localized belief", enforced rather than assumed.
 */
export const SIGMA_MAP = 0.05;
const SCAN: ScanParams = { nBeams: 72, fov: 2 * Math.PI, maxRange: 4, sigma: 0.02, zRand: 0.01, zMax: 0.005 };

export interface RustyEventLog {
  t: SimTime;
  text: string;
  kind: 'info' | 'chaos' | 'detect' | 'mode' | 'done' | 'fail';
}

export interface RustyOptions {
  seed: number;
  rates: RateTable;
  furniture: readonly Furniture[];
  /** Simulated time at which the bedroom door shuts, or null. */
  doorCloseAt: SimTime | null;
  /** Stage ids switched off from a given time (w23.2's toggles). */
  disabled?: readonly { id: string; from: SimTime }[];
}

/** The planner's report, as it travels on the bus. */
export interface RustyPlanMsg {
  path: Point2[];
  cost: number;
  /** Vertices D* Lite touched in the last repair (Koenig & Likhachev's locality, measured). */
  touched: number;
  pops: number;
  /** replan_on_belief's verdict on the first step. */
  belief: ReplanDecision | null;
  modeAction: Action | null;
  /** Belief mass for which the planned first step would hit something. */
  beliefRisk: number;
  fresh: boolean;
}

export interface RustyBeliefMsg {
  pose: Pose2;
  /** Position covariance of the cloud. */
  cov: [[number, number], [number, number]];
  /** √λ_max of `cov`. */
  sigma: number;
}

export interface MapChange {
  cell: number;
  occupied: boolean;
}

export const RUSTY_STAGES = (r: RateTable): StageInfo[] => [
  {
    id: 'rusty.scan', robot: 'rusty', label: 'Range scan', chapter: 3, slug: 'ch03-bug-algorithms', periodMs: 1000 / r.scanHz,
    role: 'source', guarantee: Guarantee.None, input: '—', output: 'Stamped<Scan> (base)',
    degradation: 'No scans: the belief coasts on odometry and the map stops learning — a shut door is invisible until the bumper finds it.',
  },
  {
    id: 'rusty.belief', robot: 'rusty', label: 'Particle localizer', chapter: 16, slug: 'ch16-bayesian-localization', periodMs: 1000 / r.beliefHz,
    role: 'source', guarantee: Guarantee.None, input: 'Scan, Odom', output: 'Stamped<PoseBelief> (map)',
    degradation: 'Belief frozen: Rusty plans from where it was, and the slip it never corrects slides it toward a wall.',
  },
  {
    id: 'rusty.map', robot: 'rusty', label: 'Inflated grid', chapter: 4, slug: 'ch04-configuration-space-i', periodMs: 1000 / r.mapHz,
    role: 'source', guarantee: Guarantee.None, input: 'Scan, PoseBelief', output: 'Stamped<MapPatch> (map)',
    degradation: 'The map never changes: no replans, no D23.2 stop — Rusty drives into the shut door and the bumper ends the mission.',
  },
  {
    id: 'rusty.plan', robot: 'rusty', label: 'D* Lite', chapter: 6, slug: 'ch06-graph-search', periodMs: 1000 / r.planHz,
    role: 'rejecting', guarantee: Guarantee.ResolutionComplete, input: 'MapPatch, PoseBelief', output: 'Stamped<Path<R2>> (map)',
    degradation: 'No repairs: Rusty follows a stale path until the D23.2 check sees the new obstacle inside L_valid and stops it for good.',
  },
  {
    id: 'rusty.ctrl', robot: 'rusty', label: 'MPC tracker', chapter: 19, slug: 'ch19-trajectory-optimization', periodMs: 1000 / r.ctrlHz,
    role: 'verified', guarantee: Guarantee.ResolutionComplete, input: 'Path, PoseBelief, newest map', output: 'Stamped<Twist> (base)',
    degradation: 'No commands: every plan is correct and nothing moves.',
  },
];

/** A `CostToGo` whose field is D* Lite's own cost-to-goal (min(g, rhs)), so `replan_on_belief` scores against the repaired map. */
function fieldCostToGo(d: DStarLite, lambda: number): CostToGo {
  const n = d.g.length;
  const field = new Float64Array(n);
  let maxCost = 0;
  for (let i = 0; i < n; i++) {
    field[i] = Math.min(d.g[i], d.rhs[i]);
    if (Number.isFinite(field[i])) maxCost = Math.max(maxCost, field[i]);
  }
  // CostToGo's own `withLambda` builds instances this way: the field is data, Dijkstra is not rerun.
  const out = Object.create(CostToGo.prototype) as CostToGo;
  Object.assign(out, {
    graph: d.graph,
    goal: d.goal,
    lambda,
    lattice: d.believed,
    plan: { goal: d.goal, costToGo: field, next: new Int32Array(n).fill(-1), maxCost },
  });
  return out;
}

function actionBetween(g: OccupancyGrid, a: number, b: number): Action | null {
  const [ai, aj] = cellIJ(g, a);
  const [bi, bj] = cellIJ(g, b);
  for (const act of ACTIONS_8) {
    const [di, dj] = ACTION_STEP[act];
    if (ai + di === bi && aj + dj === bj) return act;
  }
  return null;
}

/** Rusty's tracking cost: follow the reference points r_k, prefer forward, respect v_max softly. */
function trackingCost(refs: readonly Point2[], vMax: number): StageCost {
  const wp = 12;
  const wf = 40;
  const ru: [number, number] = [0.4, 0.04];
  const wsp = 200;
  const wrev = 30;
  return {
    running(x: Vec, u: Vec, k: number): RunningDerivs {
      const r = refs[Math.min(k, refs.length - 1)];
      const ex = x[0] - r.x;
      const ey = x[1] - r.y;
      const over = Math.max(0, Math.abs(u[0]) - vMax);
      const back = Math.min(0, u[0]);
      const sgn = u[0] >= 0 ? 1 : -1;
      const l = 0.5 * wp * (ex * ex + ey * ey) + 0.5 * (ru[0] * u[0] * u[0] + ru[1] * u[1] * u[1]) + 0.5 * wsp * over * over + 0.5 * wrev * back * back;
      return {
        l,
        lx: [wp * ex, wp * ey, 0],
        lu: [ru[0] * u[0] + wsp * over * sgn + wrev * back, ru[1] * u[1]],
        lxx: [[wp, 0, 0], [0, wp, 0], [0, 0, 0]],
        luu: [[ru[0] + (over > 0 ? wsp : 0) + (back < 0 ? wrev : 0), 0], [0, ru[1]]],
        lux: [[0, 0, 0], [0, 0, 0]],
      };
    },
    terminal(x: Vec): TerminalDerivs {
      const r = refs[refs.length - 1];
      const ex = x[0] - r.x;
      const ey = x[1] - r.y;
      return { l: 0.5 * wf * (ex * ex + ey * ey), lx: [wf * ex, wf * ey, 0], lxx: [[wf, 0, 0], [0, wf, 0], [0, 0, 0]] };
    },
  };
}

export class RustyStack {
  readonly bus: Bus;
  readonly rng: Rng;
  readonly opts: RustyOptions;
  readonly stages: StageInfo[];
  readonly angles: number[];
  readonly vMax = RUSTY_V_MAX;
  readonly aMax = RUSTY_A_MAX;
  readonly lValid: number;
  readonly goal: Point2 = RUSTY_GOAL;

  // Ground truth: the simulator owns it; no stage reads it except the sensor.
  truth: Pose2;
  trueWorld: World;
  doorClosed = false;
  /** The door the chaos shut, if any. */
  shutDoor: Segment | null = null;
  shutDoorName: string | null = null;
  trail: Point2[] = [];
  driven = 0;
  collisions = 0;

  // Odometry (what the wheels report) and the commanded twist.
  odom: Pose2;
  v = 0;
  omega = 0;
  private odomAtBelief: Pose2;

  readonly belief: ParticleBelief;
  private readonly blueprintField: ExactDistanceField;

  /** The map stage's grid: the newest map, which D23.2 checks against. */
  readonly grid: OccupancyGrid;
  readonly changes: MapChange[] = [];
  private prevHits = new Set<number>();
  readonly confirmed = new Set<number>();
  private changeCursor = 0;

  dstar: DStarLite;
  path: Point2[] = [];
  /** The path the planner last published (for drawing). */
  lastPlan: RustyPlanMsg | null = null;
  replans = 0;
  repairs = 0;
  fullReplans = 0;

  readonly mpc: Mpc;
  private refs: Point2[] = [];
  rollout: Point2[] = [];
  blocked: { at: number; point: Point2 } | null = null;

  tangent: TangentBug | null = null;
  mode: Mode = PLAN;
  readonly log: RustyEventLog[] = [];
  doneAt: SimTime | null = null;
  readonly startCell: number;
  readonly goalCell: number;

  constructor(bus: Bus, opts: RustyOptions) {
    this.bus = bus;
    this.opts = opts;
    this.rng = new Rng(opts.seed * 31 + 1);
    this.stages = RUSTY_STAGES(opts.rates);
    this.angles = beamAngles(SCAN);
    this.lValid = replanningBudget(opts.rates, RUSTY_V_MAX, RUSTY_A_MAX, D_DETECT).lValid;
    this.truth = { ...RUSTY_START };
    this.odom = { ...RUSTY_START };
    this.odomAtBelief = { ...RUSTY_START };
    this.trueWorld = trueApartment(opts.furniture, null);
    this.trail.push({ x: this.truth.x, y: this.truth.y });
    this.belief = ParticleBelief.clustered(BLUEPRINT, this.rng.fork(), 100, [{ pose: RUSTY_START, weight: 1, std: 0.04 }], {
      every: 3,
      sigmas: DEFAULT_CHOSET_SIGMAS,
      surprise: null,
      sensor: { ...LOCALIZATION_PROXIMITY, maxRange: SCAN.maxRange },
    });
    this.blueprintField = exactDistanceField(BLUEPRINT, 0.05, BLUEPRINT.bounds);
    this.grid = gridFromWorld(BLUEPRINT, RUSTY_CELL, RUSTY_INFLATION);
    this.startCell = freeCellNear(this.grid, RUSTY_START.x, RUSTY_START.y);
    this.goalCell = freeCellNear(this.grid, RUSTY_GOAL.x, RUSTY_GOAL.y);
    this.dstar = new DStarLite(this.grid, this.startCell, this.goalCell, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
    this.mpc = new Mpc(new Unicycle(BASE_MS / 1000), () => trackingCost(this.refs, this.vMax), { horizon: 10, ilqr: { maxIter: 3 } });
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

  /** Register the five stages with a scheduler, in topological order. */
  register(rr: RoundRobin): void {
    const [scan, belief, map, plan, ctrl] = this.stages;
    rr.add(scan, (now) => this.scanStage(now));
    rr.add(belief, (now) => this.beliefStage(now));
    rr.add(map, (now) => this.mapStage(now));
    rr.add(plan, (now) => this.planStage(now));
    rr.add(ctrl, (now) => this.ctrlStage(now));
  }

  /** World events the simulator applies before the stages run (chaos). */
  worldTick(now: SimTime): void {
    if (this.opts.doorCloseAt !== null && !this.doorClosed && now >= this.opts.doorCloseAt) this.closeDoor(now);
  }

  /**
   * *Close Door*: shut the next door Rusty's current path goes through. Both
   * doors lead into the bedroom, so one shut door is a replanning problem, not
   * a dead end. If the path crosses neither (Rusty is already through), say so.
   */
  closeDoor(now: SimTime): void {
    if (this.doorClosed) return;
    this.doorClosed = true;
    const pose = this.truth;
    const s0 = this.path.length > 1 ? projectOnPath(this.path, pose).seg : 0;
    let pick: (typeof DOORS)[number] | null = null;
    for (let i = s0; i + 1 < this.path.length && !pick; i++) {
      const leg: Segment = { x1: this.path[i].x, y1: this.path[i].y, x2: this.path[i + 1].x, y2: this.path[i + 1].y };
      pick = DOORS.find((d) => segmentsIntersect(leg, d.seg)) ?? null;
    }
    if (!pick) {
      this.event(now, 'chaos: Close Door — no door left on Rusty’s path', 'chaos');
      return;
    }
    this.shutDoor = pick.seg;
    this.shutDoorName = pick.name;
    this.trueWorld = trueApartment(this.opts.furniture, pick.seg);
    this.event(now, `chaos: ${pick.name} swings shut`, 'chaos');
  }

  /* ---------------------------------------------------------------- stages */

  scanStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.disabled('rusty.scan', now)) return { emitted: false, inputAge: NaN };
    const ranges = simulateScan(this.trueWorld, this.truth, SCAN, this.rng);
    this.bus.publish<number[]>('rusty/scan', stamp(now, 'base', ranges));
    return { emitted: true, inputAge: 0 };
  }

  beliefStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.disabled('rusty.belief', now)) return { emitted: false, inputAge: NaN };
    const scan = this.bus.latest<number[]>('rusty/scan');
    const u = incrementBetween(this.odomAtBelief, this.odom);
    this.odomAtBelief = { ...this.odom };
    this.belief.predict(u, this.rng);
    if (scan) {
      this.belief.correct(scan.v, this.angles);
      if (this.belief.pf.effectiveSampleSize() < this.belief.size / 2) this.belief.resample(this.rng);
    }
    this.bus.publish<RustyBeliefMsg>('rusty/belief', stamp(now, 'map', this.beliefMsg()));
    return { emitted: true, inputAge: ageOf(scan, now) };
  }

  beliefMsg(): RustyBeliefMsg {
    const ps = this.belief.pf.particles;
    const pose = this.belief.modePose();
    let W = 0, mx = 0, my = 0;
    for (const p of ps) {
      W += p.weight;
      mx += p.weight * p.state.x;
      my += p.weight * p.state.y;
    }
    mx /= W || 1;
    my /= W || 1;
    let cxx = 0, cxy = 0, cyy = 0;
    for (const p of ps) {
      const dx = p.state.x - mx;
      const dy = p.state.y - my;
      cxx += p.weight * dx * dx;
      cxy += p.weight * dx * dy;
      cyy += p.weight * dy * dy;
    }
    cxx /= W || 1;
    cxy /= W || 1;
    cyy /= W || 1;
    const tr = cxx + cyy;
    const lmax = tr / 2 + Math.sqrt(Math.max(0, (tr * tr) / 4 - (cxx * cyy - cxy * cxy)));
    return { pose, cov: [[cxx, cxy], [cxy, cyy]], sigma: Math.sqrt(Math.max(0, lmax)) };
  }

  /** The pose every downstream stage uses: the belief's, never the truth. */
  beliefPose(): Pose2 {
    return this.bus.latest<RustyBeliefMsg>('rusty/belief')?.v.pose ?? RUSTY_START;
  }

  mapStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.disabled('rusty.map', now)) return { emitted: false, inputAge: NaN };
    const scan = this.bus.latest<number[]>('rusty/scan');
    const bel = this.bus.latest<RustyBeliefMsg>('rusty/belief');
    if (!scan || !bel) return { emitted: false, inputAge: NaN };
    const pose = bel.v.pose;
    if (bel.v.sigma > SIGMA_MAP) {
      this.prevHits = new Set();
      this.bus.publish<{ version: number; total: number }>('rusty/map', stamp(now, 'map', { version: this.changes.length, total: this.confirmed.size }));
      return { emitted: true, inputAge: Math.max(ageOf(scan, now), ageOf(bel, now)) };
    }
    const hits = new Set<number>();
    for (let i = 0; i < scan.v.length; i++) {
      const r = scan.v[i];
      if (r > D_DETECT || r >= SCAN.maxRange - 1e-6) continue;
      const a = pose.theta + this.angles[i];
      const px = pose.x + r * Math.cos(a);
      const py = pose.y + r * Math.sin(a);
      if (px < 0 || py < 0 || px > 12 || py > 9) continue;
      if (distanceAt(this.blueprintField, px, py) < EXPLAINED) continue;
      hits.add(cellAt(this.grid, px, py));
    }
    // Two consecutive scans must agree (to within a cell) before a return becomes an obstacle.
    const fresh: MapChange[] = [];
    const nx = this.grid.nx;
    for (const c of hits) {
      if (this.confirmed.has(c)) continue;
      let seenBefore = false;
      for (let dj = -1; dj <= 1 && !seenBefore; dj++) for (let di = -1; di <= 1; di++) if (this.prevHits.has(c + dj * nx + di)) seenBefore = true;
      if (!seenBefore) continue;
      this.confirmed.add(c);
      // Chapter 4: the disc's C-obstacle of a point obstacle is a disc of radius r + margin.
      const ctr = cellCenter(this.grid, c);
      const R = Math.ceil(RUSTY_INFLATION / RUSTY_CELL);
      const [ci, cj] = cellIJ(this.grid, c);
      for (let dj = -R; dj <= R; dj++) {
        for (let di = -R; di <= R; di++) {
          const i = ci + di;
          const j = cj + dj;
          if (i < 0 || j < 0 || i >= nx || j >= this.grid.ny) continue;
          const k = j * nx + i;
          const cc = cellCenter(this.grid, k);
          if (Math.hypot(cc.x - ctr.x, cc.y - ctr.y) > RUSTY_INFLATION) continue;
          if (this.grid.occupied[k]) continue;
          this.grid.occupied[k] = 1;
          fresh.push({ cell: k, occupied: true });
        }
      }
    }
    this.prevHits = hits;
    if (fresh.length) {
      this.changes.push(...fresh);
      this.event(now, `map: ${fresh.length} cells inflated around a confirmed return`, 'detect');
    }
    this.bus.publish<{ version: number; total: number }>('rusty/map', stamp(now, 'map', { version: this.changes.length, total: this.confirmed.size }));
    return { emitted: true, inputAge: Math.max(ageOf(scan, now), ageOf(bel, now)) };
  }

  planStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.disabled('rusty.plan', now)) return { emitted: false, inputAge: NaN };
    if (this.mode.kind === 'Done' || this.mode.kind === 'Failed') return { emitted: false, inputAge: NaN };
    const mapMsg = this.bus.latest<{ version: number }>('rusty/map');
    const bel = this.bus.latest<RustyBeliefMsg>('rusty/belief');
    const pose = this.beliefPose();
    const pending = this.changes.slice(this.changeCursor);
    this.changeCursor = this.changes.length;
    // D* Lite's start is the belief's cell; its k_m bookkeeping happens inside updateCells.
    const start = freeCellNear(this.dstar.believed, pose.x, pose.y);
    this.dstar.start = start;
    let touched = 0;
    if (pending.length) {
      touched = this.dstar.updateCells(pending);
      this.repairs++;
    }
    let cells = this.dstar.pathFromStart();
    if (!cells) {
      // The start drifted off D* Lite's consistent band (or the map sealed it): one full search.
      this.dstar = new DStarLite(this.dstar.believed, start, this.goalCell, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
      this.fullReplans++;
      cells = this.dstar.pathFromStart();
    }
    if (!cells) {
      this.bus.publish<RustyPlanMsg | null>('rusty/plan', stamp(now, 'map', null));
      if (this.mode.kind !== 'Recover') {
        this.event(now, 'D* Lite: no path on the inflated grid', 'detect');
        this.sup(now, { kind: 'NoPath', recover: { kind: 'TangentBug' } });
        this.tangent = new TangentBug(this.trueWorld, { x: this.truth.x, y: this.truth.y }, this.goal, { step: this.vMax * (BASE_MS / 1000), turn: 'left', range: 4, nRays: 72, clearance: 0.2, goalTol: 0.15 });
      }
      return { emitted: true, inputAge: Math.max(ageOf(mapMsg, now), ageOf(bel, now)) };
    }
    const believed = this.dstar.believed;
    const pts = cells.map((c) => cellCenter(believed, c));
    // Decimate, then Chapter 11's greedy shortcut on the same grid, so the tracker gets corners, not stairs.
    const coarse: number[][] = [[pose.x, pose.y]];
    for (let i = 3; i < pts.length - 1; i += 3) coarse.push([pts[i].x, pts[i].y]);
    coarse.push([this.goal.x, this.goal.y]);
    const free = new CountingFreeSpace<number[]>((q) => {
      const c = cellAt(believed, q[0], q[1]);
      return believed.occupied[c] === 0;
    });
    const space = R2([0, 0], [12, 9]);
    const short = shortcutGreedy(new StraightLine<number[]>({ kind: 'subdivision', step: RUSTY_CELL / 2 }), space, free, coarse);
    const path = short.map((q) => ({ x: q[0], y: q[1] }));
    // Chapter 16: score the first move against the whole belief, on D* Lite's own cost field.
    let decision: ReplanDecision | null = null;
    let modeAction: Action | null = null;
    let beliefRisk = 0;
    if (cells.length > 1) {
      const J = fieldCostToGo(this.dstar, 50);
      const samples = this.belief.samples(16, believed, this.rng);
      decision = replanOnBelief(J, samples, { lambda: 50, betaInfo: 0, actions: ACTIONS_8 });
      modeAction = actionBetween(believed, cells[0], cells[1]);
      beliefRisk = decision.table.find((r) => r.action === modeAction)?.collisionMass ?? 0;
    }
    const changed = pending.length > 0 || this.path.length === 0;
    if (changed && this.path.length > 0) {
      this.replans++;
      this.event(now, `D* Lite repaired ${touched} vertices; path ${polylineLength(path).toFixed(2)} m`, 'detect');
    }
    this.path = path;
    const msg: RustyPlanMsg = { path, cost: this.dstar.costFromStart(), touched, pops: this.dstar.stats.pops, belief: decision, modeAction, beliefRisk, fresh: changed };
    this.lastPlan = msg;
    this.bus.publish<RustyPlanMsg>('rusty/plan', stamp(now, 'map', msg));
    if (this.mode.kind === 'Plan' || this.mode.kind === 'Replan' || this.mode.kind === 'Recover') {
      const clear = corridorBlocked(path, this.grid, pose, this.lValid, RUSTY_CELL * 1.5);
      if (clear.ok) {
        if (this.mode.kind === 'Recover') this.tangent = null;
        this.sup(now, { kind: 'PlanReady' });
      }
    }
    return { emitted: true, inputAge: Math.max(ageOf(mapMsg, now), ageOf(bel, now)) };
  }

  ctrlStage(now: SimTime): { emitted: boolean; inputAge: number } {
    const dt = BASE_MS / 1000;
    const bel = this.bus.latest<RustyBeliefMsg>('rusty/belief');
    const plan = this.bus.latest<RustyPlanMsg>('rusty/plan');
    const pose = this.beliefPose();
    let vCmd = 0;
    let wCmd = 0;
    const off = this.disabled('rusty.ctrl', now);
    if (!off && (this.mode.kind === 'Execute' || this.mode.kind === 'Recover')) {
      if (Math.hypot(pose.x - this.goal.x, pose.y - this.goal.y) < 0.15) {
        this.sup(now, { kind: 'Arrived' });
        this.doneAt = now;
      } else if (this.mode.kind === 'Recover' && this.tangent) {
        // Chapter 3's Tangent Bug in the true world: step its point when Rusty has caught up, steer at it.
        const tb = this.tangent;
        if (Math.hypot(tb.state.x.x - pose.x, tb.state.x.y - pose.y) < 0.12 && !tb.state.done && !tb.state.failed) tb.step();
        [vCmd, wCmd] = this.pursue(pose, tb.state.x);
        if (tb.state.failed) this.sup(now, { kind: 'Fault', reason: 'NoPath', detail: 'Tangent Bug: the goal is unreachable' });
      } else if (this.mode.kind === 'Execute') {
        const check = corridorBlocked(this.path, this.grid, pose, this.lValid, RUSTY_CELL * 1.5);
        if (!check.ok) {
          this.blocked = { at: check.err.at, point: check.err.point };
          this.event(now, `D23.2: occupied cell ${check.err.at.toFixed(2)} m ahead inside L_valid = ${this.lValid.toFixed(2)} m — braking`, 'detect');
          this.sup(now, { kind: 'Blocked', at: check.err.at });
        } else {
          this.blocked = null;
          [vCmd, wCmd] = this.track(pose, now);
        }
      }
    }
    // Actuator limits: |a| ≤ a_max (so braking takes v²/2a), |v| ≤ v_max.
    const dv = Math.max(-this.aMax * dt, Math.min(this.aMax * dt, vCmd - this.v));
    this.v = Math.max(-this.vMax, Math.min(this.vMax, this.v + dv));
    this.omega = Math.max(-2, Math.min(2, wCmd));
    this.drive(now, dt);
    return { emitted: !off, inputAge: Math.max(ageOf(bel, now), ageOf(plan, now)) };
  }

  /** Pure pursuit toward a point: turn first, then go (used by the Tangent Bug fallback). */
  private pursue(pose: Pose2, target: Point2): [number, number] {
    const a = Math.atan2(target.y - pose.y, target.x - pose.x);
    let e = a - pose.theta;
    while (e > Math.PI) e -= 2 * Math.PI;
    while (e < -Math.PI) e += 2 * Math.PI;
    const d = Math.hypot(target.x - pose.x, target.y - pose.y);
    return [Math.abs(e) > 0.6 ? 0 : Math.min(this.vMax * 0.6, 2 * d), 2.5 * e];
  }

  /** Chapter 19's receding horizon: reference points v_ref Δt apart along the path, iLQR from the belief. */
  private track(pose: Pose2, now: SimTime): [number, number] {
    if (this.path.length < 2) return [0, 0];
    const dt = BASE_MS / 1000;
    const total = polylineLength(this.path);
    const s0 = projectOnPath(this.path, pose).s;
    const remaining = total - s0;
    // Slow down into the goal: v_ref² ≤ 2 a d (the same braking law the budget assumes).
    const vRef = Math.min(this.vMax, Math.sqrt(2 * this.aMax * 0.5 * Math.max(0, remaining)) + 0.05);
    // A differential drive may turn in place: when the path doubles back (a repaired plan often
    // does), rotate toward it first instead of asking a short-horizon iLQR to discover a U-turn.
    const ahead = pointAtArc(this.path, Math.min(total, s0 + 0.3));
    let err = Math.atan2(ahead.y - pose.y, ahead.x - pose.x) - pose.theta;
    while (err > Math.PI) err -= 2 * Math.PI;
    while (err < -Math.PI) err += 2 * Math.PI;
    if (Math.abs(err) > 1.0 && Math.hypot(ahead.x - pose.x, ahead.y - pose.y) > 0.05) {
      this.mpc.reset();
      this.rollout = [];
      return [0, 1.6 * Math.sign(err)];
    }
    const H = this.mpc.opts.horizon;
    this.refs = [];
    for (let k = 1; k <= H + 1; k++) this.refs.push(pointAtArc(this.path, Math.min(total, s0 + vRef * dt * k)));
    const tick = this.mpc.tick([pose.x, pose.y, pose.theta], now / BASE_MS);
    this.rollout = tick.plan.map((x) => ({ x: x[0], y: x[1] }));
    return [tick.u[0], tick.u[1]];
  }

  /** The floor: the commanded twist with a little slip, a bumper, and the wheels' (noise-free) odometry. */
  private drive(now: SimTime, dt: number): void {
    const f = new Unicycle(dt);
    const odomNext = f.step([this.odom.x, this.odom.y, this.odom.theta], [this.v, this.omega]);
    this.odom = { x: odomNext[0], y: odomNext[1], theta: odomNext[2] };
    if (Math.abs(this.v) < 1e-9 && Math.abs(this.omega) < 1e-9) return;
    const vt = this.v * (1 + this.rng.normal(0, 0.02));
    const wt = this.omega + this.rng.normal(0, 0.02);
    const next = f.step([this.truth.x, this.truth.y, this.truth.theta], [vt, wt]);
    const to: Pose2 = { x: next[0], y: next[1], theta: next[2] };
    if (collides(this.trueWorld, this.truth, to) || !isFree(this.trueWorld, to.x, to.y, RUSTY_RADIUS)) {
      this.collisions++;
      this.v = 0;
      this.event(now, 'bumper: Rusty touched an obstacle', 'fail');
      this.sup(now, { kind: 'Fault', reason: 'Collision', detail: 'the bumper closed — an obstacle the map never learned' });
      return;
    }
    this.driven += Math.hypot(to.x - this.truth.x, to.y - this.truth.y);
    this.truth = to;
    const last = this.trail[this.trail.length - 1];
    if (Math.hypot(last.x - to.x, last.y - to.y) > 0.04) this.trail.push({ x: to.x, y: to.y });
  }

  /**
   * The final-map optimum: Chapter 6's A* on the true world as it ended,
   * inflated the same way, then the same greedy shortcut the planner applies —
   * so the ratio compares like with like (any-angle against any-angle).
   */
  optimalLength(): number {
    const g = gridFromWorld(this.trueWorld, RUSTY_CELL, RUSTY_INFLATION);
    const graph = new GridGraph(g, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
    const s = freeCellNear(g, RUSTY_START.x, RUSTY_START.y);
    const t = freeCellNear(g, this.goal.x, this.goal.y);
    const tr = astar(graph, s, t, gridHeuristic(g, t, 'octile', Math.SQRT2));
    if (tr.status !== 'found') return Infinity;
    const pts: number[][] = [[RUSTY_START.x, RUSTY_START.y]];
    for (let i = 3; i < tr.path.length - 1; i += 3) {
      const c = cellCenter(g, tr.path[i]);
      pts.push([c.x, c.y]);
    }
    pts.push([this.goal.x, this.goal.y]);
    const free = new CountingFreeSpace<number[]>((q) => g.occupied[cellAt(g, q[0], q[1])] === 0);
    const short = shortcutGreedy(new StraightLine<number[]>({ kind: 'subdivision', step: RUSTY_CELL / 2 }), R2([0, 0], [12, 9]), free, pts);
    let L = 0;
    for (let i = 1; i < short.length; i++) L += Math.hypot(short[i][0] - short[i - 1][0], short[i][1] - short[i - 1][1]);
    return L;
  }

  /** A snapshot of the planner's believed grid (for drawing the inflated map). */
  believedGrid(): OccupancyGrid {
    return cloneGrid(this.dstar.believed);
  }
}

export { SCAN as RUSTY_SCAN };
