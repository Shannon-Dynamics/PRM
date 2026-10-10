/**
 * The supervisor: one mode machine per robot (D23.4).
 *
 *   Plan | Execute | Replan | Recover(kind) | Done | Failed(reason)
 *
 * `supervisorStep` is a pure function (Mode, Event) → Mode written as an
 * exhaustive `switch` with an `assertNever` default — the TypeScript shadow of
 * Rust's exhaustive `match`. Add a variant to `Recover` or `Mode` and every
 * switch that forgot it stops compiling; that is the fourth vignette of the
 * Retrospective Scorecard, and Exercise 4 asks the reader to feel it firsthand.
 *
 * Recoveries carry their parameters because "recover" is not a plan:
 * `PullGoalInward { delta }` says how far the arm's target moves toward the
 * base (F3), `PullForward { untilPhi }` says when the trailer is straight enough
 * to replan (F4), `TangentBug` hands Rusty to Chapter 3's complete local planner
 * when the grid has no path.
 */

export type Recover =
  | { kind: 'PullGoalInward'; delta: number }
  | { kind: 'PullForward'; untilPhi: number }
  | { kind: 'TangentBug' };

export type FailReason = 'NoPath' | 'Collision' | 'Timeout' | 'Jackknife' | 'JointRate' | 'PathNotTrajectory' | 'Boxed';

export type Mode =
  | { kind: 'Plan' }
  | { kind: 'Execute' }
  | { kind: 'Replan' }
  | { kind: 'Recover'; recover: Recover }
  | { kind: 'Done' }
  | { kind: 'Failed'; reason: FailReason; detail: string };

export type SupervisorEvent =
  | { kind: 'PlanReady' }
  | { kind: 'NoPath'; recover: Recover | null }
  | { kind: 'Blocked'; at: number }
  | { kind: 'Arrived' }
  | { kind: 'NextTask' }
  | { kind: 'Singular'; delta: number }
  | { kind: 'Jackknife'; untilPhi: number }
  | { kind: 'Recovered' }
  | { kind: 'Fault'; reason: FailReason; detail: string };

export function assertNever(x: never): never {
  throw new Error(`unhandled variant: ${JSON.stringify(x)}`);
}

export const PLAN: Mode = { kind: 'Plan' };

/** The mode machine. Events that make no sense in a mode leave it unchanged. */
export function supervisorStep(mode: Mode, ev: SupervisorEvent): Mode {
  // A fault ends any live mode; terminal modes ignore everything except a new task.
  if (ev.kind === 'Fault' && mode.kind !== 'Failed' && mode.kind !== 'Done') return { kind: 'Failed', reason: ev.reason, detail: ev.detail };
  switch (mode.kind) {
    case 'Plan':
    case 'Replan':
      switch (ev.kind) {
        case 'PlanReady':
          return { kind: 'Execute' };
        case 'NoPath':
          return ev.recover ? { kind: 'Recover', recover: ev.recover } : { kind: 'Failed', reason: 'NoPath', detail: 'the planner rejected the query and no recovery applies' };
        case 'Singular':
          return { kind: 'Recover', recover: { kind: 'PullGoalInward', delta: ev.delta } };
        case 'Jackknife':
          return { kind: 'Recover', recover: { kind: 'PullForward', untilPhi: ev.untilPhi } };
        case 'Blocked':
        case 'Arrived':
        case 'NextTask':
        case 'Recovered':
        case 'Fault':
          return mode;
        default:
          return assertNever(ev);
      }
    case 'Execute':
      switch (ev.kind) {
        case 'Blocked':
          return { kind: 'Replan' };
        case 'Arrived':
          return { kind: 'Done' };
        case 'Jackknife':
          return { kind: 'Recover', recover: { kind: 'PullForward', untilPhi: ev.untilPhi } };
        case 'Singular':
          return { kind: 'Recover', recover: { kind: 'PullGoalInward', delta: ev.delta } };
        case 'NoPath':
          return ev.recover ? { kind: 'Recover', recover: ev.recover } : { kind: 'Replan' };
        case 'PlanReady':
        case 'NextTask':
        case 'Recovered':
        case 'Fault':
          return mode;
        default:
          return assertNever(ev);
      }
    case 'Recover':
      switch (ev.kind) {
        case 'Recovered':
          return { kind: 'Replan' };
        case 'PlanReady':
          // TangentBug may hand back to a fresh grid plan without an explicit Recovered.
          return mode.recover.kind === 'TangentBug' ? { kind: 'Execute' } : mode;
        case 'Arrived':
          return { kind: 'Done' };
        case 'NoPath':
        case 'Blocked':
        case 'NextTask':
        case 'Singular':
        case 'Jackknife':
        case 'Fault':
          return mode;
        default:
          return assertNever(ev);
      }
    case 'Done':
      return ev.kind === 'NextTask' ? { kind: 'Plan' } : mode;
    case 'Failed':
      return mode;
    default:
      return assertNever(mode);
  }
}

/** The mode as the panel prints it, Rust-style. */
export function modeName(m: Mode): string {
  switch (m.kind) {
    case 'Plan':
    case 'Execute':
    case 'Replan':
    case 'Done':
      return m.kind;
    case 'Recover':
      switch (m.recover.kind) {
        case 'PullGoalInward':
          return `Recover(PullGoalInward { delta: ${m.recover.delta.toFixed(2)} })`;
        case 'PullForward':
          return `Recover(PullForward { until_phi: ${m.recover.untilPhi.toFixed(2)} })`;
        case 'TangentBug':
          return 'Recover(TangentBug)';
        default:
          return assertNever(m.recover);
      }
    case 'Failed':
      return `Failed(${m.reason})`;
    default:
      return assertNever(m);
  }
}

/** The six top-level variants, in the order the left rail draws them. */
export const MODE_KINDS: readonly Mode['kind'][] = ['Plan', 'Execute', 'Replan', 'Recover', 'Done', 'Failed'];
