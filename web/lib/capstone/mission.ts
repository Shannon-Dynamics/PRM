/**
 * `run_mission` — all three pipelines on one clock, deterministic.
 *
 * The three panels do not share a world: this is three stacks, not one robot
 * team (Chapter 14 is where robots couple). They share a *scheduler* — one
 * round-robin over fifteen stages at 20 Hz base rate — because that is the
 * claim the capstone is making: every planner in the book, running together,
 * at real time, inside one browser frame budget.
 *
 * `Mission` is the steppable object the Grand Plan drives a few ticks per
 * animation frame; `runMission` runs it to the end and returns the report the
 * checks pin. Same seed, same configuration → the same report, bit for bit:
 * every random draw comes from a seeded `Rng`, time is integer milliseconds,
 * and nothing in the report reads a wall clock.
 */

import type { Pose2 } from '../geom/se2';
import { collides, isFree, type Point2 } from '../sim/world';
import { RUSTY_RADIUS } from '../robots/rusty';
import type { HitchCfg } from '../robots/hitch';
import { Bus, type SimTime } from './bus';
import { GUARANTEE_NAMES, Guarantee, RoundRobin, composedGuarantee, type RobotId, type StageInfo } from './stage';
import { DEFAULT_RATES, budgetLine, replanningBudget, type BudgetReport, type RateTable } from './budget';
import { RustyStack, BASE_MS, type RustyEventLog } from './rusty';
import { ReachStack, type ReachTaskRecord } from './reach';
import { HitchStack } from './hitch';
import { modeName } from './supervisor';
import { pickFurniture, trueApartment, type Furniture } from './worlds';

export { BASE_MS };

export type ChaosEvent =
  | { kind: 'close-door'; t: SimTime }
  | { kind: 'singular-goal'; t: SimTime }
  | { kind: 'reverse-hard'; t: SimTime };

export interface MissionCfg {
  seed: number;
  rates?: RateTable;
  chaos?: readonly ChaosEvent[];
  /** Stage ids switched off from a given time (w23.2). */
  disabled?: readonly { id: string; from: SimTime }[];
  /** Which panels run (the Failure Tour runs one at a time). Default: all three. */
  robots?: readonly RobotId[];
  /** Hard stop, ms of simulated time. */
  tMaxMs?: number;
  reachTasks?: number;
  /** Chapter 22's learned sampler for Reach's RRT-Connect (λ of the uniform floor), or null. */
  learnedLambda?: number | null;
}

/** The book's canonical regression mission: seed 7, door at 12 s, singular goal, reverse hard. */
export const SEED_7_CHAOS: readonly ChaosEvent[] = [
  { kind: 'close-door', t: 9000 },
  { kind: 'singular-goal', t: 4000 },
  { kind: 'reverse-hard', t: 2500 },
];

export interface RustyReport {
  done: boolean;
  mode: string;
  tDone: number | null;
  pathLength: number;
  optimalLength: number;
  ratio: number;
  replans: number;
  repairs: number;
  collisions: number;
  furniture: string[];
}

export interface ReachReport {
  done: boolean;
  mode: string;
  picks: number;
  cost: number;
  durationBefore: number;
  durationAfter: number;
  maxTorqueRatio: number;
  replayTorqueRatio: number;
  guardFired: boolean;
  collisions: number;
  /** Per-pick records (cost, T₁, T₂, …) in order. */
  picksDetail: ReachTaskRecord[];
}

export interface HitchReport {
  done: boolean;
  mode: string;
  tDone: number | null;
  posErr: number;
  headErr: number;
  cusps: number;
  maxPhi: number;
  guardFired: boolean;
  replans: number;
  collisions: number;
}

export interface MissionEvent {
  t: SimTime;
  robot: RobotId;
  text: string;
  kind: RustyEventLog['kind'];
}

export interface MissionReport {
  seed: number;
  tEnd: SimTime;
  budget: BudgetReport;
  rusty: RustyReport | null;
  reach: ReachReport | null;
  hitch: HitchReport | null;
  events: MissionEvent[];
  /** Dense recheck of every executed motion against the world it happened in. */
  collisionFree: { rusty: boolean; reach: boolean; hitch: boolean };
  composed: Record<RobotId, string>;
}

/** Stage roles as configured: switching `reach.verify` off turns CHOMP into an unverified post-processor. */
export function effectiveStages(infos: readonly StageInfo[], disabledIds: readonly string[]): StageInfo[] {
  return infos
    .filter((s) => !disabledIds.includes(s.id) || s.role !== 'rejecting')
    .map((s) => (s.id === 'reach.chomp' && disabledIds.includes('reach.verify') ? { ...s, role: 'unverified' as const } : s));
}

export class Mission {
  readonly cfg: MissionCfg;
  readonly bus = new Bus();
  readonly rr = new RoundRobin();
  readonly rates: RateTable;
  readonly budget: BudgetReport;
  readonly furniture: Furniture[];
  readonly rusty: RustyStack | null;
  readonly reach: ReachStack | null;
  readonly hitch: HitchStack | null;
  now: SimTime = 0;
  /** Rusty's truth trail with time stamps, for the dense recheck. */
  private readonly rustyTimed: { t: SimTime; p: Pose2 }[] = [];
  private doorClosedAt: SimTime | null = null;

  constructor(cfg: MissionCfg) {
    this.cfg = cfg;
    this.rates = cfg.rates ?? DEFAULT_RATES;
    for (const hz of Object.values(this.rates)) {
      const period = 1000 / hz;
      if (Math.abs(period / BASE_MS - Math.round(period / BASE_MS)) > 1e-9) throw new Error(`rate ${hz} Hz is not a multiple of the ${BASE_MS} ms base tick`);
    }
    this.budget = replanningBudget(this.rates);
    this.furniture = pickFurniture(cfg.seed);
    const robots = cfg.robots ?? ['rusty', 'reach', 'hitch'];
    const chaos = cfg.chaos ?? [];
    const at = (k: ChaosEvent['kind']) => chaos.find((c) => c.kind === k)?.t ?? null;
    this.rusty = robots.includes('rusty') ? new RustyStack(this.bus, { seed: cfg.seed, rates: this.rates, furniture: this.furniture, doorCloseAt: at('close-door'), disabled: cfg.disabled }) : null;
    this.reach = robots.includes('reach') ? new ReachStack(this.bus, { seed: cfg.seed, singularAt: at('singular-goal'), disabled: cfg.disabled, tasks: cfg.reachTasks, learnedLambda: cfg.learnedLambda ?? null }) : null;
    this.hitch = robots.includes('hitch') ? new HitchStack(this.bus, { seed: cfg.seed, reverseHardAt: at('reverse-hard'), disabled: cfg.disabled }) : null;
    this.rusty?.register(this.rr);
    this.reach?.register(this.rr);
    this.hitch?.register(this.rr);
    if (this.rusty) this.rustyTimed.push({ t: 0, p: { ...this.rusty.truth } });
  }

  get tMax(): SimTime {
    return this.cfg.tMaxMs ?? 90000;
  }

  /** Every panel finished (Done or Failed) or the clock ran out. */
  get finished(): boolean {
    if (this.now >= this.tMax) return true;
    const end = (m: { kind: string } | undefined) => m === undefined || m.kind === 'Done' || m.kind === 'Failed';
    const reachEnd = !this.reach || this.reach.mode.kind === 'Failed' || this.reach.done;
    return end(this.rusty?.mode) && reachEnd && end(this.hitch?.mode);
  }

  /** Advance one 50 ms base tick: world events, then every due stage in topological order. */
  step(): string[] {
    this.now += BASE_MS;
    this.rusty?.worldTick(this.now);
    this.reach?.worldTick(this.now);
    this.hitch?.worldTick(this.now);
    if (this.rusty?.shutDoor && this.doorClosedAt === null) this.doorClosedAt = this.now;
    const ran = this.rr.tick(this.now);
    if (this.rusty) this.rustyTimed.push({ t: this.now, p: { ...this.rusty.truth } });
    return ran;
  }

  run(): MissionReport {
    while (!this.finished) this.step();
    return this.report();
  }

  stages(): StageInfo[] {
    return this.rr.infos();
  }

  composed(robot: RobotId): Guarantee {
    const off = (this.cfg.disabled ?? []).filter((d) => this.now >= d.from).map((d) => d.id);
    return composedGuarantee(effectiveStages(this.rr.infos().filter((s) => s.robot === robot), off));
  }

  events(): MissionEvent[] {
    const out: MissionEvent[] = [];
    if (this.rusty) for (const e of this.rusty.log) out.push({ ...e, robot: 'rusty' });
    if (this.reach) for (const e of this.reach.log) out.push({ ...e, robot: 'reach' });
    if (this.hitch) for (const e of this.hitch.log) out.push({ ...e, robot: 'hitch' });
    return out.sort((a, b) => a.t - b.t || a.robot.localeCompare(b.robot));
  }

  /** Rusty's executed motion against the world as it was at each instant (the door shuts mid-mission). */
  rustyCollisionFree(): boolean {
    if (!this.rusty) return true;
    const open = trueApartment(this.furniture, null);
    const shut = trueApartment(this.furniture, this.rusty.shutDoor);
    for (let i = 1; i < this.rustyTimed.length; i++) {
      const w = this.doorClosedAt !== null && this.rustyTimed[i].t >= this.doorClosedAt ? shut : open;
      const a = this.rustyTimed[i - 1].p;
      const b = this.rustyTimed[i].p;
      if (a.x === b.x && a.y === b.y) continue;
      if (collides(w, a, b) || !isFree(w, b.x, b.y, RUSTY_RADIUS)) return false;
    }
    return true;
  }

  reachCollisionFree(): boolean {
    if (!this.reach) return true;
    const ex = this.reach.executed;
    for (let i = 1; i < ex.length; i++) if (!this.reach.checker.edgeFree(ex[i - 1], ex[i])) return false;
    return true;
  }

  hitchCollisionFree(): boolean {
    if (!this.hitch) return true;
    const ex: HitchCfg[] = this.hitch.executed;
    for (let i = 1; i < ex.length; i++) if (!this.hitch.checker.edgeFree(ex[i - 1], ex[i])) return false;
    return true;
  }

  report(): MissionReport {
    const r = this.rusty;
    const a = this.reach;
    const h = this.hitch;
    let rusty: RustyReport | null = null;
    if (r) {
      const opt = r.optimalLength();
      rusty = {
        done: r.mode.kind === 'Done',
        mode: modeName(r.mode),
        tDone: r.doneAt,
        pathLength: r.driven,
        optimalLength: opt,
        ratio: r.driven / opt,
        replans: r.replans,
        repairs: r.repairs,
        collisions: r.collisions,
        furniture: this.furniture.map((f) => f.name),
      };
    }
    let reach: ReachReport | null = null;
    if (a) {
      const recs = a.records;
      reach = {
        done: a.done,
        mode: modeName(a.mode),
        picks: recs.length,
        cost: recs.reduce((s, x) => s + x.cost, 0),
        durationBefore: recs.reduce((s, x) => s + x.durationBefore, 0),
        durationAfter: recs.reduce((s, x) => s + x.durationAfter, 0),
        maxTorqueRatio: recs.reduce((s, x) => Math.max(s, x.maxTorqueRatio), 0),
        replayTorqueRatio: recs.reduce((s, x) => Math.max(s, x.replayTorqueRatio), 0),
        picksDetail: recs.map((x) => ({ ...x, tip: { ...x.tip } })),
        guardFired: recs.some((x) => x.guardFired),
        collisions: a.collisions,
      };
    }
    let hitch: HitchReport | null = null;
    if (h) {
      const pe = h.parkError();
      hitch = {
        done: h.mode.kind === 'Done',
        mode: modeName(h.mode),
        tDone: h.doneAt,
        posErr: pe.pos,
        headErr: pe.head,
        cusps: h.cusps,
        maxPhi: h.maxPhi,
        guardFired: h.guardFired,
        replans: h.replans,
        collisions: h.collisions,
      };
    }
    return {
      seed: this.cfg.seed,
      tEnd: this.now,
      budget: this.budget,
      rusty,
      reach,
      hitch,
      events: this.events(),
      collisionFree: { rusty: this.rustyCollisionFree(), reach: this.reachCollisionFree(), hitch: this.hitchCollisionFree() },
      composed: {
        rusty: GUARANTEE_NAMES[this.composed('rusty')],
        reach: GUARANTEE_NAMES[this.composed('reach')],
        hitch: GUARANTEE_NAMES[this.composed('hitch')],
      },
    };
  }
}

export function runMission(cfg: MissionCfg): MissionReport {
  return new Mission(cfg).run();
}

/* -------------------------------------------------------------------------- */
/* Recording, for the trace-replay widgets                                     */
/* -------------------------------------------------------------------------- */

/** What one recorded tick holds: enough to redraw every panel and every rate. */
export interface MissionFrame {
  t: SimTime;
  rusty: { truth: Pose2; belief: Pose2; sigma: number; mode: string; detail: string; path: Point2[]; blocked: boolean; doorClosed: boolean; confirmed: number; v: number; trail: number } | null;
  reach: { q: number[]; mode: string; detail: string; phase: string; mu: number; tip: Point2; path: number[][] | null; picks: number } | null;
  hitch: { q: HitchCfg; mode: string; detail: string; phi: number; v: number; plan: Point2[]; executed: number } | null;
  /** Messages emitted by each stage so far. */
  emitted: Record<string, number>;
  /** Age of each stage's newest input at its last run (ms). */
  age: Record<string, number>;
  ran: string[];
}

/**
 * Runs a mission a few ticks at a time and keeps one frame per tick, so a
 * widget can scrub a recording while the rest of it is still being computed —
 * time-slicing, not blocking.
 */
export class MissionRecorder {
  readonly mission: Mission;
  readonly frames: MissionFrame[] = [];
  private lastPlan: unknown = null;
  private lastPlanPts: Point2[] = [];

  constructor(cfg: MissionCfg) {
    this.mission = new Mission(cfg);
    this.frames.push(this.capture([]));
  }

  get done(): boolean {
    return this.mission.finished;
  }

  /** Record up to `n` more ticks; returns how many were recorded. */
  advance(n: number): number {
    let k = 0;
    while (k < n && !this.mission.finished) {
      const ran = this.mission.step();
      this.frames.push(this.capture(ran));
      k++;
    }
    return k;
  }

  private capture(ran: string[]): MissionFrame {
    const m = this.mission;
    const stats = m.rr.allStats();
    const emitted: Record<string, number> = {};
    const age: Record<string, number> = {};
    for (const [id, s] of Object.entries(stats)) {
      emitted[id] = s.emitted;
      age[id] = s.lastInputAge;
    }
    const r = m.rusty;
    const a = m.reach;
    const h = m.hitch;
    const bel = r ? r.beliefMsg() : null;
    const detail = (mode: { kind: string; detail?: string }) => (mode.kind === 'Failed' ? mode.detail ?? '' : '');
    let plan: Point2[] = [];
    if (h?.plan) {
      // The plan's samples change only on a replan; reuse the mapped array between frames.
      if (this.lastPlan !== h.plan) {
        this.lastPlan = h.plan;
        this.lastPlanPts = h.plan.samples.map((s) => ({ x: s.q.x, y: s.q.y }));
      }
      plan = this.lastPlanPts;
    }
    return {
      t: m.now,
      rusty: r && bel ? { truth: { ...r.truth }, belief: bel.pose, sigma: bel.sigma, mode: modeName(r.mode), detail: detail(r.mode), path: r.path, blocked: r.blocked !== null, doorClosed: r.shutDoor !== null, confirmed: r.confirmed.size, v: r.v, trail: r.trail.length } : null,
      reach: a ? { q: a.q.slice(), mode: modeName(a.mode), detail: detail(a.mode), phase: a.phase, mu: a.mu, tip: a.tip, path: a.shortPath, picks: a.records.length } : null,
      hitch: h ? { q: h.q, mode: modeName(h.mode), detail: detail(h.mode), phi: h.phi, v: h.v, plan, executed: h.executed.length } : null,
      emitted,
      age,
      ran,
    };
  }
}

/**
 * The worked example's printout (`cargo run --example mission -- --seed 7 …`):
 * the budget line, the event timeline (chaos, detections, recoveries, arrivals),
 * and the report. The chapter prints exactly this, and a check pins it.
 */
export function formatMission(r: MissionReport): string[] {
  const out: string[] = [budgetLine(r.budget)];
  const shown = r.events.filter((e) => (e.kind !== 'info' && e.kind !== 'mode' && !/^map: /.test(e.text) && !e.text.includes(' → ')) || e.kind === 'fail');
  // Consecutive D* Lite repairs collapse to one line: the count and the range of vertices touched.
  for (let i = 0; i < shown.length; i++) {
    const e = shown[i];
    const t = `${(e.t / 1000).toFixed(2).padStart(6)} s  ${e.robot.padEnd(5)}  `;
    const rep = /^D\* Lite repaired (\d+) vertices/;
    if (rep.test(e.text)) {
      let j = i;
      const touched: number[] = [];
      while (j < shown.length && shown[j].robot === e.robot && rep.test(shown[j].text)) {
        touched.push(Number(rep.exec(shown[j].text)![1]));
        j++;
      }
      if (touched.length > 1) {
        out.push(`${t}D* Lite repaired ${touched.length} times, ${Math.min(...touched)}–${Math.max(...touched)} vertices each (until ${(shown[j - 1].t / 1000).toFixed(2)} s)`);
        i = j - 1;
        continue;
      }
    }
    out.push(`${t}${e.text}`);
  }
  if (r.rusty) out.push(`rusty  ${r.rusty.mode}: driven ${r.rusty.pathLength.toFixed(2)} m vs ${r.rusty.optimalLength.toFixed(2)} m final-map optimum (ratio ${r.rusty.ratio.toFixed(2)}), ${r.rusty.replans} repairs`);
  if (r.reach) {
    const first = r.reach.picksDetail[0];
    out.push(`reach  ${r.reach.mode}: ${r.reach.picks} picks; pick 1 cost ${first ? first.cost.toFixed(2) : '—'} rad, t_f ${first ? first.durationBefore.toFixed(2) : '—'} s → ${first ? first.durationAfter.toFixed(2) : '—'} s after CHOMP; max |τ|/τ_max ${r.reach.maxTorqueRatio.toFixed(2)} at the grid nodes`);
  }
  if (r.hitch) out.push(`hitch  ${r.hitch.mode}: parked within ${(r.hitch.posErr * 100).toFixed(1)} cm / ${((r.hitch.headErr * 180) / Math.PI).toFixed(1)}°, ${r.hitch.cusps} cusps, max |φ| ${r.hitch.maxPhi.toFixed(2)} rad`);
  out.push(`dense recheck: rusty ${r.collisionFree.rusty}, reach ${r.collisionFree.reach}, hitch ${r.collisionFree.hitch}`);
  return out;
}
