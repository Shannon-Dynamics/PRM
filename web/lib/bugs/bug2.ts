/**
 * BUG2 (Choset Algorithm 2). Greedy: the m-line is the fixed segment from
 * q_start to q_goal; at a hit point, follow the boundary until the m-line is
 * met again at a point m that is strictly closer to the goal than the hit
 * point and from which heading for the goal is free — then leave. If the hit
 * point comes round again first, the goal is unreachable.
 *
 *   L_Bug2 ≤ d(q_start, q_goal) + ½ Σ n_i p_i                     (eq. 2.2)
 *
 * The leave test is the three conditions on lines 11–14 of Algorithm 2, taken
 * on every straight piece the follower reports, so the crossing point is
 * exact and the robot stops *on* the m-line rather than a step past it.
 */

import type { Point2, Segment } from '../sim/world';
import type { Contact, Follower } from './follow';
import { closestOnSegment, dist, lerp, segmentHit, sub, unit } from './geometry';
import { initialState, pushPoint, runPlanner, type BugOptions, type BugPlanner, type BugState } from './planner';

export class Bug2 implements BugPlanner {
  readonly name = 'Bug2';
  readonly state: BugState;
  private following = false;
  private follower: Follower | null = null;
  private hit: Point2 | null = null;
  private dHit = Infinity;
  private readonly mline: Segment;
  private readonly goalTol: number;

  constructor(
    private readonly contact: Contact,
    start: Point2,
    private readonly goal: Point2,
    private readonly opts: BugOptions,
  ) {
    this.state = initialState(start);
    this.mline = { x1: start.x, y1: start.y, x2: goal.x, y2: goal.y };
    this.goalTol = opts.goalTol ?? 1e-9;
  }

  run(maxSteps?: number): BugState {
    return runPlanner(this, maxSteps);
  }

  step(): BugState {
    const s = this.state;
    if (s.done || s.failed) return s;
    s.steps++;
    if (this.following) this.followBoundary();
    else this.motionToGoal();
    return s;
  }

  /** Lines 2–4: along the m-line toward the goal. */
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
      this.hit = r.x;
      this.dHit = dist(r.x, this.goal);
      this.follower = this.contact.follow(r.x, heading, this.opts.turn);
      this.following = true;
      s.mode = 'boundary-following';
      s.reason = `following from q^H_${s.hits.length}, turning ${this.opts.turn}`;
    }
  }

  /** Lines 6–15: follow until the goal, the hit point, or a valid m-line point. */
  private followBoundary(): void {
    const s = this.state;
    const f = this.follower!;
    const mv = f.advance(this.opts.step);
    for (const piece of mv.pieces) {
      const len = dist(piece.a, piece.b);
      const c = closestOnSegment(this.goal, piece.a, piece.b);
      if (c.d <= this.goalTol) {
        pushPoint(s, this.goal);
        this.finish('reached the goal while following');
        return;
      }
      const h = segmentHit(piece.a, piece.b, this.mline);
      if (h) {
        const m = lerp(piece.a, piece.b, h.t);
        const isHit = dist(m, this.hit!) < 1e-9;
        const closer = dist(m, this.goal) < this.dHit - 1e-12;
        if (!isHit && closer && !this.contact.blocked(m, this.goal)) {
          // Lines 11–15: a valid leave point. Stop exactly on it.
          const sm = piece.s0 + h.t * len;
          const exact = f.seek(sm);
          pushPoint(s, exact);
          s.leaves.push(exact);
          this.leave();
          return;
        }
      }
      pushPoint(s, piece.b);
    }
    if (f.closed()) {
      s.failed = true;
      s.mode = 'failed';
      s.reason = 'unreachable: q^H re-encountered before a valid leave point';
    }
  }

  private leave(): void {
    const s = this.state;
    this.following = false;
    this.follower = null;
    s.mode = 'motion-to-goal';
    s.reason = `motion-to-goal from q^L_${s.leaves.length}`;
  }

  private finish(reason: string): void {
    this.state.done = true;
    this.state.mode = 'done';
    this.state.reason = reason;
  }
}
