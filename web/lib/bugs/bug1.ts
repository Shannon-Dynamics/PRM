/**
 * BUG1 (Choset Algorithm 1). Exhaustive: at a hit point, circumnavigate the
 * whole obstacle, remember the perimeter point closest to the goal, walk back
 * to it the shorter way, and leave. If leaving would re-enter the obstacle,
 * the goal is enclosed and Bug1 says so.
 *
 *   L_Bug1 ≤ d(q_start, q_goal) + 1.5 Σ p_i                       (eq. 2.1)
 *
 * The phases are the lines of Algorithm 1: `to-goal` (lines 2–7),
 * `circumnavigate` (8–10), `return` (11–12), and the test on line 13.
 */

import type { Point2 } from '../sim/world';
import type { Contact, Follower } from './follow';
import { closestOnSegment, dist, sub, unit } from './geometry';
import { initialState, pushPoint, runPlanner, type BugOptions, type BugPlanner, type BugState } from './planner';

type Phase = 'to-goal' | 'circumnavigate' | 'return';

export class Bug1 implements BugPlanner {
  readonly name = 'Bug1';
  readonly state: BugState;
  private phase: Phase = 'to-goal';
  private follower: Follower | null = null;
  /** The best leave candidate seen on this obstacle: point, arc length from the hit, distance to goal. */
  private best = { x: { x: 0, y: 0 }, s: 0, d: Infinity };
  private remaining = 0;
  private readonly goalTol: number;

  constructor(
    private readonly contact: Contact,
    start: Point2,
    private readonly goal: Point2,
    private readonly opts: BugOptions,
  ) {
    this.state = initialState(start);
    this.goalTol = opts.goalTol ?? 1e-9;
  }

  run(maxSteps?: number): BugState {
    return runPlanner(this, maxSteps);
  }

  step(): BugState {
    const s = this.state;
    if (s.done || s.failed) return s;
    s.steps++;
    switch (this.phase) {
      case 'to-goal':
        this.motionToGoal();
        break;
      case 'circumnavigate':
        this.circumnavigate();
        break;
      case 'return':
        this.returnToLeave();
        break;
    }
    return s;
  }

  /** Lines 2–7: straight toward the goal until it is reached or something is hit. */
  private motionToGoal(): void {
    const s = this.state;
    const d = dist(s.x, this.goal);
    if (d <= this.opts.step + this.goalTol) {
      pushPoint(s, this.goal);
      this.finish('reached the goal');
      return;
    }
    const heading = unit(sub(this.goal, s.x));
    const r = this.contact.moveToward(s.x, this.goal, this.opts.step);
    pushPoint(s, r.x);
    if (r.hit) {
      s.hits.push(r.x);
      s.mode = 'boundary-following';
      s.reason = `circumnavigating from q^H_${s.hits.length}`;
      this.follower = this.contact.follow(r.x, heading, this.opts.turn);
      this.best = { x: r.x, s: 0, d: dist(r.x, this.goal) };
      this.phase = 'circumnavigate';
    }
  }

  /** Lines 8–11: a full loop, remembering the closest perimeter point to the goal. */
  private circumnavigate(): void {
    const s = this.state;
    const f = this.follower!;
    const mv = f.advance(this.opts.step);
    for (const piece of mv.pieces) {
      const c = closestOnSegment(this.goal, piece.a, piece.b);
      if (c.d <= this.goalTol) {
        pushPoint(s, this.goal);
        this.finish('reached the goal while following');
        return;
      }
      if (c.d < this.best.d - 1e-12) {
        this.best = { x: c.q, s: piece.s0 + c.t * dist(piece.a, piece.b), d: c.d };
      }
      pushPoint(s, piece.b);
    }
    if (f.closed()) {
      // Line 12: go to q^L the shorter way round.
      const P = f.perimeter!;
      const forward = this.best.s;
      const backward = P - this.best.s;
      if (backward < forward) {
        f.reverse();
        this.remaining = backward;
      } else {
        this.remaining = forward;
      }
      this.phase = 'return';
      s.reason = `returning to q^L_${s.hits.length} the shorter way (${Math.min(forward, backward).toFixed(2)})`;
      if (this.remaining <= 1e-12) this.arriveAtLeave();
    }
  }

  private returnToLeave(): void {
    const s = this.state;
    const f = this.follower!;
    const ds = Math.min(this.opts.step, this.remaining);
    const mv = f.advance(ds);
    for (const piece of mv.pieces) pushPoint(s, piece.b);
    this.remaining -= ds;
    if (this.remaining <= 1e-12) this.arriveAtLeave();
  }

  /** Lines 13–15: at q^L, would heading for the goal re-enter the obstacle? */
  private arriveAtLeave(): void {
    const s = this.state;
    // Snap onto the exact leave point (the follower lands within a hop of it).
    if (dist(s.x, this.best.x) > 1e-12) pushPoint(s, this.best.x);
    s.leaves.push(this.best.x);
    if (this.contact.blocked(this.best.x, this.goal)) {
      s.failed = true;
      s.mode = 'failed';
      s.reason = 'unreachable: leaving q^L toward the goal re-enters the obstacle';
      return;
    }
    this.follower = null;
    this.phase = 'to-goal';
    s.mode = 'motion-to-goal';
    s.reason = `motion-to-goal from q^L_${s.leaves.length}`;
  }

  private finish(reason: string): void {
    this.state.done = true;
    this.state.mode = 'done';
    this.state.reason = reason;
  }
}
