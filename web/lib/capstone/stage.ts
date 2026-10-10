/**
 * Stages, contracts, guarantees, and the scheduler that runs them (D23.1, D23.3).
 *
 * A stage is one cell of Choset's task × robot × algorithm table with a clock:
 * it consumes `Stamped` inputs, emits one `Stamped` output at period τ, and
 * states the guarantee it provides *given valid inputs*. The composed stack
 * has the guarantee of its weakest rejecting stage — and only that, provided
 * every post-processor either verifies its output or falls back to its input.
 *
 * The scheduler is the browser half of the concurrency story. Native Rust runs
 * each stage on its own thread at its own rate; WASM has no threads, so a
 * deterministic round-robin ticks, inside one frame, every stage whose period
 * has elapsed, in the stack's topological order. Same stage code, same message
 * types, same seeds — and therefore the same mission, bit for bit. What the
 * browser demonstrates is *throughput* (every stage keeps its rate inside a
 * 60 fps frame budget); it proves nothing about preemptive real-time
 * *scheduling*, and the chapter says so.
 */

import type { SimTime } from './bus';

/**
 * 𝒢, ordered weakest first so that `Math.min` over the enum is "the weakest
 * link". In Rust this is `#[derive(PartialOrd, Ord)]` on the enum.
 */
export enum Guarantee {
  None = 0,
  ProbabilisticallyComplete = 1,
  ResolutionComplete = 2,
  Complete = 3,
}

export const GUARANTEE_NAMES: Record<Guarantee, string> = {
  [Guarantee.None]: 'none',
  [Guarantee.ProbabilisticallyComplete]: 'probabilistically complete',
  [Guarantee.ResolutionComplete]: 'resolution-complete',
  [Guarantee.Complete]: 'complete',
};

export type RobotId = 'rusty' | 'reach' | 'hitch';

/**
 * What a stage is, as data: the contract card w23.2 shows when a block is
 * clicked. `role` decides how the stage enters D23.3:
 *
 *   'source'    produces data (a sensor) — no completeness claim at all
 *   'rejecting' a planner that may answer "no path": it carries 𝒢_i
 *   'verified'  a post-processor that checks its output and falls back to its input
 *   'unverified' a post-processor that may silently break its input's guarantee
 *   'executor'  consumes a Trajectory and moves the robot
 *   'guard'     watches a statistic against a threshold and switches mode
 */
export interface StageInfo {
  id: string;
  robot: RobotId;
  label: string;
  /** The chapter whose artifact this stage runs. */
  chapter: number;
  slug: string;
  periodMs: number;
  role: 'source' | 'rejecting' | 'verified' | 'unverified' | 'executor' | 'guard';
  guarantee: Guarantee;
  input: string;
  output: string;
  /** What visibly breaks when this stage is switched off (w23.2). */
  degradation: string;
}

/**
 * D23.3, as code: the minimum of 𝒢_i over the stages that can reject a query,
 * provided no unverified post-processor sits downstream of them; one unverified
 * post-processor and the pipeline promises nothing.
 */
export function composedGuarantee(stages: readonly Pick<StageInfo, 'role' | 'guarantee'>[]): Guarantee {
  if (stages.some((s) => s.role === 'unverified')) return Guarantee.None;
  const rejecting = stages.filter((s) => s.role === 'rejecting');
  if (rejecting.length === 0) return Guarantee.None;
  let g: Guarantee = Guarantee.Complete;
  for (const s of rejecting) if (s.guarantee < g) g = s.guarantee;
  return g;
}

/** Per-stage bookkeeping the scheduler keeps, for the Timing tab and w23.2. */
export interface StageStats {
  runs: number;
  emitted: number;
  /** Time of the last run, or −∞. */
  lastRun: SimTime;
  /** Age of this stage's newest input at its last run (ms) — the measured ς. */
  lastInputAge: number;
  /** Worst input age seen (ms). */
  maxInputAge: number;
}

interface Entry {
  info: StageInfo;
  /** Runs the stage; returns whether it emitted a message and the age of the input it consumed. */
  run: (now: SimTime) => { emitted: boolean; inputAge: number };
  /** Phase offset (ms): a stage first runs at `phase`, then every `periodMs`. */
  phase: number;
  stats: StageStats;
  enabled: boolean;
}

/**
 * The deterministic round-robin of the WASM build. One `tick(now)` runs every
 * enabled stage whose period divides `now − phase`, in registration order
 * (the stack's topological order, producers before consumers).
 */
export class RoundRobin {
  private readonly entries: Entry[] = [];

  add(info: StageInfo, run: Entry['run'], phase = 0): void {
    this.entries.push({
      info,
      run,
      phase,
      enabled: true,
      stats: { runs: 0, emitted: 0, lastRun: -Infinity, lastInputAge: 0, maxInputAge: 0 },
    });
  }

  setEnabled(id: string, on: boolean): void {
    for (const e of this.entries) if (e.info.id === id) e.enabled = on;
  }

  isEnabled(id: string): boolean {
    return this.entries.find((e) => e.info.id === id)?.enabled ?? false;
  }

  /** Due at `now`? Integer arithmetic only — this is what keeps replays bit-identical. */
  static due(now: SimTime, periodMs: number, phase: number): boolean {
    return now >= phase && (now - phase) % periodMs === 0;
  }

  tick(now: SimTime): string[] {
    const ran: string[] = [];
    for (const e of this.entries) {
      if (!e.enabled || !RoundRobin.due(now, e.info.periodMs, e.phase)) continue;
      const r = e.run(now);
      e.stats.runs++;
      if (r.emitted) e.stats.emitted++;
      e.stats.lastRun = now;
      if (Number.isFinite(r.inputAge)) {
        e.stats.lastInputAge = r.inputAge;
        e.stats.maxInputAge = Math.max(e.stats.maxInputAge, r.inputAge);
      }
      ran.push(e.info.id);
    }
    return ran;
  }

  infos(): StageInfo[] {
    return this.entries.map((e) => e.info);
  }

  stats(id: string): StageStats | null {
    return this.entries.find((e) => e.info.id === id)?.stats ?? null;
  }

  allStats(): Record<string, StageStats> {
    const out: Record<string, StageStats> = {};
    for (const e of this.entries) out[e.info.id] = { ...e.stats };
    return out;
  }
}
