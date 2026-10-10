/**
 * `hitch_stack` — Hitch tows its trailer across the Lot into bay 3.
 *
 *   plan (Ch. 21 hybrid A* with Reeds–Shepp shots, car footprint)
 *     → trailer filter (Ch. 20 kinematics: ψ simulated along the path,
 *       |φ| ≤ φ_plan and the trailer's footprint checked)
 *     → tracker (the segment list replayed on Chapter 2's Hitch.step)
 *   guard: jackknife_guard (F4) at 20 Hz; recovery PullForward, then replan.
 *
 * Honesty item the prose keeps: Chapter 21's hybrid A* expands the *car*; the
 * trailer is checked on each candidate path, not inside the expansion. So the
 * composed planner is resolution-complete with respect to the car's lattice
 * *and* the filter — a trailer-safe path the filter would accept but the car
 * search never proposes is missed. The stack mitigates (it retries with a
 * finer cell and a growing reverse penalty, one attempt per tick) but does not
 * close that gap; Chapter 21's CAR GRID SEARCH with the trailer in the state
 * would, at a hundred times the expansions.
 *
 * Thresholds: φ_plan = 0.85 < φ_warn = 0.90 < φ_max = 1.30 rad. The planner
 * leaves headroom, the guard sits in it, the hard bound is the mechanism's.
 */

import { normalizeAngle, type Pose2 } from '../geom/se2';
import type { Point2 } from '../sim/world';
import { HITCH_WITH_TRAILER, type HitchCfg } from '../robots/hitch';
import { LOT } from '../robots/worlds';
import { Scene } from '../collide/scene';
import { FootprintChecker } from '../collide/collision';
import { hybridAStar, type CarSearchResult } from '../nonholonomic/lattice';
import { trailerAlong, scorePath, turningRadius, type CarSeg, type CarSample } from '../nonholonomic/car-path';
import { Bus, stamp, type SimTime } from './bus';
import { Guarantee, RoundRobin, type StageInfo } from './stage';
import { jackknifeGuard } from './guards';
import { PLAN, modeName, supervisorStep, type Mode, type SupervisorEvent } from './supervisor';
import { HITCH_GOAL, HITCH_START } from './worlds';
import type { RustyEventLog } from './rusty';

const BASE_MS = 50;
export const PHI_PLAN = 0.85;
export const PHI_WARN = 0.9;
export const PHI_MAX = 1.3;
export const PHI_OK = 0.15;
export const HITCH_SPEED = 1.0;
const PULL_SPEED = 0.8;

/**
 * The planner's retry ladder, one rung per tick. Direct rungs vary the cell
 * and the reverse penalty; *staged* rungs plan to a staging pose first — a
 * pose east of the bays from which the approach is known to be trailer-safe
 * when the trailer arrives aligned, which forward driving guarantees (F4) — and
 * then on to the bay. Re-approaching from a known-good pose is the classic
 * recovery behaviour of production stacks; here it is two hybrid A* calls.
 */
export const HITCH_ATTEMPTS: readonly { cell: number; reversePenalty: number; via: Pose2 | null }[] = [
  { cell: 0.5, reversePenalty: 1, via: null },
  { cell: 0.4, reversePenalty: 3, via: null },
  { cell: 0.5, reversePenalty: 1, via: { x: 12.4, y: 4.2, theta: 0 } },
  { cell: 0.5, reversePenalty: 1, via: { x: 12.0, y: 4.0, theta: 0 } },
  { cell: 0.3, reversePenalty: 1, via: null },
  { cell: 0.4, reversePenalty: 4, via: { x: 12.4, y: 4.2, theta: 0 } },
];

export const HITCH_STAGES: StageInfo[] = [
  {
    id: 'hitch.plan', robot: 'hitch', label: 'Hybrid A* + RS shots', chapter: 21, slug: 'ch21-nonholonomic-ii', periodMs: BASE_MS,
    role: 'rejecting', guarantee: Guarantee.ResolutionComplete, input: 'Stamped<HitchCfg> (lot)', output: 'Stamped<CarPath> (lot)',
    degradation: 'No plan: Hitch waits at the curb forever, and after a recovery it cannot replan.',
  },
  {
    id: 'hitch.filter', robot: 'hitch', label: 'Trailer filter', chapter: 20, slug: 'ch20-nonholonomic-i', periodMs: BASE_MS,
    role: 'rejecting', guarantee: Guarantee.None, input: 'Stamped<CarPath> candidate (lot)', output: 'Stamped<CarPath> (lot), or reject',
    degradation: 'Unfiltered car paths are executed with the trailer attached: on the default query the trailer clears, after Reverse Hard the replanned path can swing it past φ_plan.',
  },
  {
    id: 'hitch.guard', robot: 'hitch', label: 'Jackknife guard', chapter: 20, slug: 'ch20-nonholonomic-i', periodMs: BASE_MS,
    role: 'guard', guarantee: Guarantee.None, input: 'Stamped<HitchCfg>, v', output: 'Mode event',
    degradation: 'Reverse Hard keeps folding the trailer past φ_max: the Lot panel jackknifes and the mission fails.',
  },
  {
    id: 'hitch.track', robot: 'hitch', label: 'Segment tracker', chapter: 21, slug: 'ch21-nonholonomic-ii', periodMs: BASE_MS,
    role: 'executor', guarantee: Guarantee.None, input: 'Stamped<CarPath> (lot)', output: 'Stamped<CarCmd> (base)',
    degradation: 'Nothing drives: the plan is perfect and Hitch never moves.',
  },
];

export interface HitchOptions {
  seed: number;
  /** Simulated time of the *Reverse Hard* disturbance, or null. */
  reverseHardAt: SimTime | null;
  disabled?: readonly { id: string; from: SimTime }[];
}

export interface HitchPlanMsg {
  segs: CarSeg[];
  samples: CarSample[];
  /** One search per leg (two for a staged rung). */
  searches: CarSearchResult[];
  via: Pose2 | null;
  maxHitch: number;
  attempt: number;
  cusps: number;
}

export class HitchStack {
  readonly bus: Bus;
  readonly opts: HitchOptions;
  readonly car = HITCH_WITH_TRAILER;
  readonly scene = Scene.fromWorld(LOT);
  readonly checker: FootprintChecker;
  readonly rho: number;
  readonly stages = HITCH_STAGES;
  readonly goal: Pose2 = HITCH_GOAL;

  q: HitchCfg;
  v = 0;
  steer = 0;
  mode: Mode = PLAN;
  plan: HitchPlanMsg | null = null;
  /** The planner's latest car path, waiting for the trailer filter. */
  candidate: { segs: CarSeg[]; carSamples: CarSample[]; searches: CarSearchResult[]; via: Pose2 | null; attempt: number; label: string } | null = null;
  /** Rejected candidates of the current planning round (drawn faintly). */
  rejected: { samples: CarSample[]; why: string }[] = [];
  attempt = 0;
  segIndex = 0;
  segDone = 0;
  override: { until: SimTime } | null = null;
  overrideDone = false;
  readonly executed: HitchCfg[] = [];
  readonly log: RustyEventLog[] = [];
  maxPhi = 0;
  guardFired = false;
  cusps = 0;
  replans = 0;
  collisions = 0;
  doneAt: SimTime | null = null;
  private lastDir = 0;

  constructor(bus: Bus, opts: HitchOptions) {
    this.bus = bus;
    this.opts = opts;
    this.checker = new FootprintChecker(this.scene, this.car);
    this.rho = turningRadius(this.car);
    this.q = this.car.cfg(HITCH_START.x, HITCH_START.y, HITCH_START.theta);
    this.executed.push(this.q);
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
    const [plan, filter, guard, track] = this.stages;
    rr.add(plan, (now) => this.planStage(now));
    rr.add(filter, (now) => this.filterStage(now));
    rr.add(guard, (now) => this.guardStage(now));
    rr.add(track, (now) => this.trackStage(now));
  }

  /** Hitch angle φ = θ_car − ψ_trailer (Chapter 2's convention). */
  get phi(): number {
    return this.car.hitchAngle(this.q) ?? 0;
  }

  worldTick(now: SimTime): void {
    if (this.opts.reverseHardAt !== null && !this.overrideDone && now >= this.opts.reverseHardAt) this.reverseHard(now);
  }

  /** The *Reverse Hard* chaos button: full reverse at full lock, the way that folds the trailer, for up to 3 s. */
  reverseHard(now: SimTime): void {
    if (this.override || this.mode.kind === 'Done' || this.mode.kind === 'Failed') return;
    this.override = { until: now + 3000 };
    this.overrideDone = true;
    this.event(now, 'chaos: Reverse Hard — full lock, full reverse', 'chaos');
  }

  planStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.disabled('hitch.plan', now)) return { emitted: false, inputAge: NaN };
    if (this.mode.kind !== 'Plan' && this.mode.kind !== 'Replan') return { emitted: false, inputAge: NaN };
    if (this.override || this.candidate) return { emitted: false, inputAge: NaN };
    if (this.attempt >= HITCH_ATTEMPTS.length) {
      this.sup(now, { kind: 'Fault', reason: 'NoPath', detail: 'no trailer-safe path on any rung of the retry ladder' });
      return { emitted: false, inputAge: 0 };
    }
    const a = HITCH_ATTEMPTS[this.attempt];
    this.attempt++;
    const start = this.q.body;
    const legGoals = a.via ? [a.via, this.goal] : [this.goal];
    const searches: CarSearchResult[] = [];
    let from = start;
    for (const g of legGoals) {
      const r = hybridAStar({ car: this.car.withTrailer(null), scene: this.scene, start: from, goal: g, cell: a.cell, heuristic: 'dual', shotEvery: 5, reversePenalty: a.reversePenalty, maxExpansions: 4000 });
      searches.push(r);
      if (!r.found) break;
      from = g;
    }
    const label = `cell ${a.cell} m, reverse ×${a.reversePenalty}${a.via ? `, via (${a.via.x}, ${a.via.y})` : ''}`;
    if (searches.some((r) => !r.found)) {
      this.event(now, `hybrid A* (${label}): no path in ${searches.reduce((n, r) => n + r.expansions, 0)} expansions`);
      return { emitted: false, inputAge: 0 };
    }
    const segs = searches.flatMap((r) => r.segs);
    const carSamples = searches.flatMap((r, i) => (i === 0 ? r.samples : r.samples.slice(1)));
    this.candidate = { segs, carSamples, searches, via: a.via, attempt: this.attempt - 1, label };
    this.bus.publish('hitch/candidate', stamp(now, 'lot', { segs, length: searches.reduce((n, r) => n + r.length, 0) }));
    return { emitted: true, inputAge: 0 };
  }

  /**
   * The trailer filter (a rejecting stage with no completeness of its own): simulate ψ along the
   * car's candidate from the trailer's *current* heading (Chapter 20's ψ̇ = (v/d) sin(θ − ψ)),
   * reject if the trailer touches anything or |φ| exceeds φ_plan, otherwise publish the plan.
   */
  filterStage(now: SimTime): { emitted: boolean; inputAge: number } {
    const c = this.candidate;
    if (!c) return { emitted: false, inputAge: NaN };
    this.candidate = null;
    const age = now - (this.bus.latest('hitch/candidate')?.t ?? now);
    if (this.disabled('hitch.filter', now)) {
      const samples = trailerAlong(c.carSamples, this.car.p.hitchLen!, this.q.trailer ?? this.q.body.theta);
      return this.accept(now, c, samples, scorePath(samples, this.car, this.scene).maxHitch, age);
    }
    const samples = trailerAlong(c.carSamples, this.car.p.hitchLen!, this.q.trailer ?? this.q.body.theta);
    const sc = scorePath(samples, this.car, this.scene);
    if (sc.collides || sc.maxHitch > PHI_PLAN) {
      const why = sc.collides ? 'the trailer clips an obstacle' : `|φ| reaches ${sc.maxHitch.toFixed(2)} > φ_plan`;
      this.rejected.push({ samples, why });
      this.event(now, `trailer filter rejects hybrid A* (${c.label}): ${why}`, 'detect');
      return { emitted: false, inputAge: age };
    }
    return this.accept(now, c, samples, sc.maxHitch, age);
  }

  private accept(now: SimTime, c: NonNullable<HitchStack['candidate']>, samples: CarSample[], maxHitch: number, age: number): { emitted: boolean; inputAge: number } {
    const length = c.searches.reduce((n, r) => n + r.length, 0);
    const expansions = c.searches.reduce((n, r) => n + r.expansions, 0);
    let cusps = 0;
    let prev = 0;
    for (const s of samples) {
      if (Math.abs(s.ds) < 1e-9) continue;
      const sg = Math.sign(s.ds);
      if (prev !== 0 && sg !== prev) cusps++;
      prev = sg;
    }
    this.plan = { segs: c.segs, samples, searches: c.searches, via: c.via, maxHitch, attempt: c.attempt, cusps };
    this.segIndex = 0;
    this.segDone = 0;
    this.event(now, `hybrid A* (${c.label}): ${length.toFixed(2)} m, ${cusps} cusp${cusps === 1 ? '' : 's'}, ${expansions} expansions, max |φ| ${maxHitch.toFixed(2)} rad`, this.replans > 0 ? 'detect' : 'info');
    this.bus.publish<HitchPlanMsg>('hitch/plan', stamp(now, 'lot', this.plan));
    this.sup(now, { kind: 'PlanReady' });
    return { emitted: true, inputAge: age };
  }

  guardStage(now: SimTime): { emitted: boolean; inputAge: number } {
    if (this.mode.kind === 'Done' || this.mode.kind === 'Failed') return { emitted: false, inputAge: NaN };
    const phi = this.phi;
    if (Math.abs(phi) > PHI_MAX) {
      this.event(now, `jackknife: |φ| = ${Math.abs(phi).toFixed(2)} rad > φ_max = ${PHI_MAX}`, 'fail');
      this.sup(now, { kind: 'Fault', reason: 'Jackknife', detail: 'the trailer folded into the car' });
      this.override = null;
      this.v = 0;
      return { emitted: false, inputAge: 0 };
    }
    if (this.disabled('hitch.guard', now)) return { emitted: false, inputAge: NaN };
    const g = jackknifeGuard(phi, this.v, PHI_WARN);
    if (!g.ok && this.mode.kind !== 'Recover') {
      this.guardFired = true;
      this.override = null;
      this.event(now, `jackknife guard: |φ| = ${Math.abs(g.err.phi).toFixed(2)} rad > φ_warn = ${PHI_WARN} while reversing`, 'detect');
      this.sup(now, { kind: 'Jackknife', untilPhi: PHI_OK });
      return { emitted: true, inputAge: 0 };
    }
    return { emitted: false, inputAge: 0 };
  }

  /** One exact arc of the Chapter 2 car; refuses (and reports) a step that would collide. */
  private stepCar(now: SimTime, v: number, steer: number, dt: number): boolean {
    const next = this.car.step(this.q, { v, phi: steer }, dt);
    if (!this.checker.isFree(next)) return false;
    const dir = Math.sign(v);
    if (dir !== 0 && this.lastDir !== 0 && dir !== this.lastDir) this.cusps++;
    if (dir !== 0) this.lastDir = dir;
    this.q = next;
    this.v = v;
    this.steer = steer;
    this.executed.push(next);
    this.maxPhi = Math.max(this.maxPhi, Math.abs(this.phi));
    return true;
  }

  trackStage(now: SimTime): { emitted: boolean; inputAge: number } {
    const dt = BASE_MS / 1000;
    if (this.mode.kind === 'Done' || this.mode.kind === 'Failed') {
      this.v = 0;
      return { emitted: false, inputAge: NaN };
    }
    if (this.override && now < this.override.until) {
      // The disturbance: reverse at full lock, steering the way that folds the trailer further.
      const phi = this.phi;
      const steer = (phi >= 0 ? -1 : 1) * this.car.p.maxSteer;
      if (!this.stepCar(now, -HITCH_SPEED, steer, dt)) {
        this.v = 0;
        this.releaseOverride(now, 'Reverse Hard stopped short of the curb');
      }
      return { emitted: true, inputAge: 0 };
    }
    if (this.override) {
      this.releaseOverride(now, 'Reverse Hard released');
      return { emitted: false, inputAge: 0 };
    }
    if (this.disabled('hitch.track', now)) {
      this.v = 0;
      return { emitted: false, inputAge: NaN };
    }
    if (this.mode.kind === 'Recover' && this.mode.recover.kind === 'PullForward') {
      const phi = this.phi;
      if (Math.abs(phi) < this.mode.recover.untilPhi) {
        this.v = 0;
        this.event(now, `recovered: |φ| = ${Math.abs(phi).toFixed(2)} < φ_ok = ${PHI_OK}; replanning from here`, 'done');
        this.sup(now, { kind: 'Recovered' });
        this.attempt = 0;
        this.rejected = [];
        this.plan = null;
        this.candidate = null;
        this.replans++;
        return { emitted: false, inputAge: 0 };
      }
      // Forward is the stable direction (φ̇ ≈ −(v/d) φ); steer toward the trailer to straighten faster.
      const steer = Math.max(-this.car.p.maxSteer, Math.min(this.car.p.maxSteer, -1.5 * phi));
      if (!this.stepCar(now, PULL_SPEED, steer, dt) && !this.stepCar(now, PULL_SPEED, 0, dt)) {
        this.sup(now, { kind: 'Fault', reason: 'Boxed', detail: 'no room to pull forward' });
      }
      return { emitted: true, inputAge: 0 };
    }
    if (this.mode.kind !== 'Execute' || !this.plan) {
      this.v = 0;
      return { emitted: false, inputAge: NaN };
    }
    // Replay the segment list: |v| = 1 m/s, φ = ±φ_max or 0, never overshooting a segment's end.
    let budget = HITCH_SPEED * dt;
    while (budget > 1e-12 && this.segIndex < this.plan.segs.length) {
      const g = this.plan.segs[this.segIndex];
      const left = Math.abs(g.len) - this.segDone;
      const ds = Math.min(budget, left);
      const steer = g.steer === 'L' ? this.car.p.maxSteer : g.steer === 'R' ? -this.car.p.maxSteer : 0;
      const v = Math.sign(g.len) * HITCH_SPEED;
      if (ds > 1e-12 && !this.stepCar(now, v, steer, ds / HITCH_SPEED)) {
        this.collisions++;
        this.sup(now, { kind: 'Fault', reason: 'Collision', detail: 'the executed footprint touched an obstacle' });
        return { emitted: false, inputAge: 0 };
      }
      this.segDone += ds;
      budget -= ds;
      if (this.segDone >= Math.abs(g.len) - 1e-12) {
        this.segIndex++;
        this.segDone = 0;
      }
    }
    if (this.segIndex >= this.plan.segs.length) {
      this.v = 0;
      this.doneAt = now;
      this.event(now, `parked: ${this.parkError().pos.toFixed(3)} m, ${((this.parkError().head * 180) / Math.PI).toFixed(2)}°`, 'done');
      this.sup(now, { kind: 'Arrived' });
    }
    return { emitted: true, inputAge: 0 };
  }

  /** The disturbance ended without the guard firing: the plan no longer starts where Hitch is, so replan. */
  private releaseOverride(now: SimTime, why: string): void {
    this.override = null;
    this.event(now, `${why}; the plan is stale — replanning`);
    if (this.mode.kind === 'Execute') this.sup(now, { kind: 'Blocked', at: 0 });
    this.attempt = 0;
    this.plan = null;
    this.candidate = null;
    this.replans++;
  }

  parkError(): { pos: number; head: number } {
    const b = this.q.body;
    return { pos: Math.hypot(b.x - this.goal.x, b.y - this.goal.y), head: Math.abs(normalizeAngle(b.theta - this.goal.theta)) };
  }

  /** Cusps of the current plan, for the panel's markers. */
  cuspPoints(): Point2[] {
    if (!this.plan) return [];
    const out: Point2[] = [];
    const s = this.plan.samples;
    for (let i = 1; i + 1 < s.length; i++) if (Math.sign(s[i].ds) !== Math.sign(s[i + 1].ds) && Math.abs(s[i + 1].ds) > 1e-9) out.push({ x: s[i].q.x, y: s[i].q.y });
    return out;
  }
}
