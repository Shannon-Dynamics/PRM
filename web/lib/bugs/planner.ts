/**
 * What every Bug exposes: a step-wise state machine over a point robot with
 * perfect position, so a widget can animate it one step at a time and read
 * off its mode, path, hit and leave points, and whether it finished.
 */

import type { Point2 } from '../sim/world';
import { dist, lineCrossings, type Polygon, type Turn } from './geometry';

export type BugMode = 'motion-to-goal' | 'boundary-following' | 'done' | 'failed';

export interface BugState {
  /** Current position. */
  x: Point2;
  mode: BugMode;
  /** Every point the robot has been at, in order. */
  path: Point2[];
  /** Length of `path`. */
  length: number;
  /** Hit points q^H_i (Tangent Bug: the local minima M_i where following began). */
  hits: Point2[];
  /** Leave points q^L_i. */
  leaves: Point2[];
  done: boolean;
  failed: boolean;
  /** Why it failed, or a one-line description of what it is doing. */
  reason: string;
  steps: number;
}

export interface BugOptions {
  /** Motion resolution: how far one `step()` moves. */
  step: number;
  /** Which way to turn at a hit point. */
  turn: Turn;
  /** Reaching within this of the goal counts as arriving. */
  goalTol?: number;
}

export interface BugPlanner {
  readonly name: string;
  readonly state: BugState;
  /** Advance one motion step. Idempotent once done or failed. */
  step(): BugState;
  /** Run to completion or `maxSteps`, whichever first. */
  run(maxSteps?: number): BugState;
}

export function initialState(start: Point2): BugState {
  return {
    x: start,
    mode: 'motion-to-goal',
    path: [start],
    length: 0,
    hits: [],
    leaves: [],
    done: false,
    failed: false,
    reason: 'motion-to-goal',
    steps: 0,
  };
}

/** Append a point to the path and account for its length. */
export function pushPoint(s: BugState, p: Point2): void {
  s.length += dist(s.x, p);
  s.path.push(p);
  s.x = p;
}

/** Default: run until done, with a generous ceiling so a widget never hangs. */
export function runPlanner(planner: BugPlanner, maxSteps = 200_000): BugState {
  let n = 0;
  while (!planner.state.done && !planner.state.failed && n < maxSteps) {
    planner.step();
    n++;
  }
  if (!planner.state.done && !planner.state.failed) {
    planner.state.failed = true;
    planner.state.mode = 'failed';
    planner.state.reason = `step budget of ${maxSteps} exhausted`;
  }
  return planner.state;
}

// ---------------------------------------------------------------------------
// The bounds (Choset eqs. 2.1 and 2.2)
// ---------------------------------------------------------------------------

/** L_Bug1 ≤ d(q_start, q_goal) + 1.5 Σ p_i. */
export function bug1Bound(start: Point2, goal: Point2, perimeters: readonly number[]): number {
  return dist(start, goal) + 1.5 * perimeters.reduce((a, b) => a + b, 0);
}

/** L_Bug2 ≤ d(q_start, q_goal) + ½ Σ n_i p_i, with n_i the m-line crossings of ∂WO_i. */
export function bug2Bound(start: Point2, goal: Point2, obstacles: readonly Polygon[], perimeters: readonly number[]): number {
  let sum = 0;
  obstacles.forEach((poly, i) => {
    sum += lineCrossings(poly, start, goal) * perimeters[i];
  });
  return dist(start, goal) + 0.5 * sum;
}
