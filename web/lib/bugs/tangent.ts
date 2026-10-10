/**
 * TANGENT BUG (Choset Algorithm 3). The same two behaviors as Bug1/Bug2, but
 * seen through a range sensor of radius R:
 *
 *  motion-to-goal      move toward the n ∈ {T, O_i} minimizing d(x, n) + d(n, goal),
 *                      where the O_i are the endpoints of the intervals of
 *                      continuity of ρ_R(x, ·) and T is the point of the sensing
 *                      circle on the segment to the goal — until that direction
 *                      begins to *increase* d(x, goal): a local minimum M.
 *  boundary-following  follow the obstacle in the direction of the most recent
 *                      motion, updating d_reach (closest visible point of the
 *                      followed obstacle to the goal) and d_followed (closest
 *                      point sensed so far), until d_reach < d_followed.
 *
 * The robot keeps a clearance W* from every wall (Choset §2.3.2: "follow a
 * path at a safe distance"), so motion-to-goal has the hugging phase of
 * figure 2.9 — when the chosen direction points into the band, the robot slides
 * along the tangent (∇D)^⊥ instead. Boundary following reuses the offset
 * follower of `follow.ts`, which is the predictor–corrector of `trace.ts`.
 *
 * Tangent Bug's completeness is Choset's problem 2.9 and is not claimed here;
 * the planner carries a step budget so a widget can never hang on it.
 */

import { rayCast, type Point2, type World } from '../sim/world';
import { offsetContact, type Contact, type Follower } from './follow';
import { add, cross, dist, dot, isPolygonWorld, scale, segmentHit, sub, unit, witness } from './geometry';
import { initialState, pushPoint, runPlanner, type BugOptions, type BugPlanner, type BugState } from './planner';
import {
  argminRay,
  continuityIntervals,
  intervalEndpoints,
  pointT,
  rangeScan,
  scanPoint,
  type Endpoint,
  type Interval,
  type Scan,
} from './scan';
import { correctGradient, tangentFromGradient } from './trace';

export interface Candidate {
  /** The n ∈ {T, O_i} itself, on the free-space boundary (or on the sensing circle). */
  point: Point2;
  /**
   * Where the robot actually steers: O_i displaced by the clearance toward the
   * free side of its discontinuity, so that a point robot with a safety band
   * rounds the corner instead of pressing into the wall beside it.
   */
  steer: Point2;
  /** d(x, n) + d(n, q_goal); ∞ when the robot can see that n→goal is blocked. */
  h: number;
  label: string;
}

/** What the sensor saw at the last step, for the Tangent Scope and the arena to draw. */
export interface TangentSensing {
  scan: Scan;
  intervals: Interval[];
  endpoints: Endpoint[];
  candidates: Candidate[];
  chosen: Candidate | null;
  /** Is the straight line to the goal blocked within sensing range? */
  blocked: boolean;
  dReach: number;
  dFollowed: number;
}

export interface TangentOptions extends BugOptions {
  /** Sensing radius R; use Infinity for Choset's infinite-range figure 2.13. */
  range: number;
  /** Rays per scan. */
  nRays?: number;
  /** W*: the clearance the point robot keeps from walls. */
  clearance: number;
}

export class TangentBug implements BugPlanner {
  readonly name = 'Tangent Bug';
  readonly state: BugState;
  sensing: TangentSensing | null = null;
  private readonly contact: Contact;
  private following = false;
  private follower: Follower | null = null;
  private M: Point2 | null = null;
  /** d_followed at each M_i, so a return to M_i with no new knowledge is recognised as a cycle. */
  private readonly knowledgeAtM: number[] = [];
  /** Which side of the goal ray the previous subgoal lay on (+1 left, −1 right, 0 none): hysteresis. */
  private side = 0;
  private dFollowed = Infinity;
  /** Direction of the most recent motion-to-goal step, which fixes the following direction. */
  private lastDir: Point2;
  private readonly nRays: number;
  private readonly goalTol: number;
  private readonly R: number;

  constructor(
    private readonly world: World,
    start: Point2,
    private readonly goal: Point2,
    private readonly opts: TangentOptions,
  ) {
    this.state = initialState(start);
    this.contact = offsetContact(world, { clearance: opts.clearance });
    this.nRays = opts.nRays ?? 360;
    this.goalTol = opts.goalTol ?? 1e-9;
    this.R = Number.isFinite(opts.range) ? opts.range : 1e9;
    this.lastDir = unit(sub(goal, start));
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

  /** ρ_R at x and everything the planner derives from it. */
  sense(x: Point2): Omit<TangentSensing, 'chosen' | 'dReach' | 'dFollowed'> {
    const scan = rangeScan(this.world, x.x, x.y, this.nRays, this.R);
    const intervals = continuityIntervals(scan, { world: this.world });
    const endpoints = intervalEndpoints(scan, intervals, this.world);
    const dGoal = dist(x, this.goal);
    const toGoal = Math.atan2(this.goal.y - x.y, this.goal.x - x.x);
    const reach = Math.min(this.R, dGoal);
    const blocked = rayCast(this.world, x.x, x.y, toGoal, reach) < reach - 1e-9;
    const candidates: Candidate[] = [];
    if (!blocked) {
      const T = pointT(x, this.goal, this.R);
      candidates.push({ point: T, steer: T, h: dGoal, label: dist(T, this.goal) < 1e-12 ? 'q_goal' : 'T' });
    }
    const W = this.opts.clearance;
    endpoints.forEach((e, i) => {
      const h = this.visiblyBlocked(x, e) ? Infinity : dist(x, e.point) + dist(e.point, this.goal);
      // The interval lies at increasing bearing from a 'from' end and at
      // decreasing bearing from a 'to' end; free space is the other way.
      const free = e.end === 'from' ? -1 : 1;
      const across = { x: -Math.sin(e.theta) * free, y: Math.cos(e.theta) * free };
      const along = { x: Math.cos(e.theta), y: Math.sin(e.theta) };
      const steer = add(e.point, add(scale(across, 1.2 * W), scale(along, 0.5 * W)));
      candidates.push({ point: e.point, steer, h, label: `O${i + 1}` });
    });
    return { scan, intervals, endpoints, candidates, blocked };
  }

  /**
   * The whole motion-to-goal decision at an arbitrary point, without moving:
   * what the Tangent Scope draws. `dReach` is the distance from the goal to the
   * closest visible point of the nearest obstacle.
   */
  evaluate(x: Point2): TangentSensing {
    const sensed = this.sense(x);
    const saveSide = this.side;
    this.side = 0;
    const d = this.decide(x, sensed);
    this.side = saveSide;
    return { ...sensed, chosen: d.chosen, dReach: this.reach(sensed.scan, sensed.intervals), dFollowed: this.dFollowed };
  }

  /**
   * Figure 2.8's refinement: if the segment O_i → goal cuts through *another*
   * obstacle at a point the robot can currently see, O_i gets infinite cost.
   * Needs obstacle identity, so it applies on polygon worlds only.
   */
  private visiblyBlocked(x: Point2, e: Endpoint): boolean {
    if (!isPolygonWorld(this.world)) return false;
    const w = witness(this.world, e.point);
    const owner = this.world.wallOwner[w.wall];
    let bestT = Infinity;
    let bestWall = -1;
    this.world.walls.forEach((seg, k) => {
      const h = segmentHit(e.point, this.goal, seg);
      if (h && h.t > 1e-6 && h.t < bestT) {
        bestT = h.t;
        bestWall = k;
      }
    });
    if (bestWall < 0 || this.world.wallOwner[bestWall] === owner) return false;
    const q = add(e.point, scale(sub(this.goal, e.point), bestT));
    return dist(x, q) <= this.R;
  }

  /**
   * The motion-to-goal decision at x: which n ∈ {T, O_i} to head for, the unit
   * direction u to move in (projected onto the band's tangent when the robot is
   * hugging a wall), and whether that direction still decreases d(x, q_goal).
   */
  private decide(x: Point2, sensed: ReturnType<TangentBug['sense']>): {
    chosen: Candidate | null;
    u: Point2;
    sliding: boolean;
    progress: boolean;
  } {
    const finite = sensed.candidates.filter((c) => Number.isFinite(c.h));
    let chosen: Candidate | null = null;
    const goalDir = unit(sub(this.goal, x));
    if (finite.length > 0) {
      finite.sort((a, b) => a.h - b.h);
      chosen = finite[0];
      // A tie (the worked example's √5 + √65 on both sides) is broken by the
      // configured turn: left prefers the candidate on the left of the goal ray.
      const ties = finite.filter((c) => Math.abs(c.h - chosen!.h) < 1e-9);
      if (ties.length > 1) {
        const want = this.opts.turn === 'left' ? 1 : -1;
        const side = ties.find((c) => Math.sign(cross(goalDir, sub(c.point, x))) === want);
        if (side) chosen = side;
      }
      // Hysteresis. An endpoint created by the range limit is a horizon, not a
      // feature: it recedes as the robot approaches, and two such endpoints on
      // either side of the goal ray can trade places every step. Stay on the
      // side already chosen unless the other side wins by a full step.
      const sideOf = (c: Candidate) => Math.sign(cross(goalDir, sub(c.point, x)));
      if (this.side !== 0 && sideOf(chosen) !== this.side) {
        const same = finite.find((c) => sideOf(c) === this.side);
        if (same && same.h <= chosen.h + this.opts.step) chosen = same;
      }
    }
    if (!chosen) return { chosen, u: goalDir, sliding: false, progress: false };
    let u = unit(sub(chosen.steer, x));
    const w = witness(this.world, x);
    const onBand = w.d <= this.opts.clearance + 1e-7;
    let sliding = false;
    if (onBand && dot(w.grad, u) < 0) {
      // Hugging phase (figure 2.9): project the desired direction onto the band's tangent.
      const t = sub(u, scale(w.grad, dot(u, w.grad)));
      u = dist(t, { x: 0, y: 0 }) > 1e-9 ? unit(t) : tangentFromGradient(w.grad, this.opts.turn);
      sliding = true;
    }
    // Line 4 of Algorithm 3: does the chosen direction still decrease d(x, q_goal)?
    const progress = !sensed.blocked || dot(u, goalDir) > 1e-9;
    return { chosen, u, sliding, progress };
  }

  /** Lines 2–4 of Algorithm 3. */
  private motionToGoal(): void {
    const s = this.state;
    const x = s.x;
    const dGoal = dist(x, this.goal);
    if (dGoal <= this.opts.step + this.goalTol) {
      pushPoint(s, this.goal);
      this.finish('reached the goal');
      return;
    }
    const sensed = this.sense(x);
    const dReach = this.reach(sensed.scan, sensed.intervals);
    const d = this.decide(x, sensed);
    this.sensing = { ...sensed, chosen: d.chosen, dReach, dFollowed: this.dFollowed };
    this.dFollowed = Math.min(this.dFollowed, dReach);
    if (d.chosen) this.side = Math.sign(cross(unit(sub(this.goal, x)), sub(d.chosen.point, x)));

    if (!d.chosen) {
      this.beginFollowing(x, 'no finite candidate');
      return;
    }
    if (!d.progress) {
      // A local minimum of d(·, q_goal) along the motion-to-goal path: M.
      this.beginFollowing(x, 'local minimum of d(·, q_goal)');
      return;
    }
    this.lastDir = d.u;
    let next: Point2;
    if (d.sliding) {
      const W = this.opts.clearance;
      const G = (p: Point2) => witness(this.world, p).d - W;
      const DG = (p: Point2) => witness(this.world, p).grad;
      next = correctGradient(G, DG, add(x, scale(d.u, this.opts.step)), 6, 1e-12);
    } else {
      next = this.contact.moveToward(x, add(x, scale(d.u, this.opts.step)), this.opts.step).x;
    }
    pushPoint(s, next);
    s.reason = `motion-to-goal toward ${d.chosen.label}${d.sliding ? ' (hugging the boundary)' : ''}`;
  }

  /** Line 5: start boundary following in the direction of the most recent motion. */
  private beginFollowing(x: Point2, why: string): void {
    const s = this.state;
    // Revisiting an earlier local minimum means the leave rule has led the robot
    // round the same obstacle once already: the goal is enclosed. Choset leaves
    // Tangent Bug's completeness as problem 2.9; this is the termination test a
    // running implementation needs so it never circles forever.
    const tol = Math.max(4 * this.opts.step, this.opts.clearance);
    const cycle = s.hits.findIndex((m, i) => dist(m, x) < tol && this.dFollowed >= this.knowledgeAtM[i] - 1e-9);
    if (cycle >= 0) {
      s.failed = true;
      s.mode = 'failed';
      s.reason = `unreachable: back at M_${cycle + 1} with nothing new sensed since`;
      return;
    }
    this.M = x;
    this.side = 0;
    s.hits.push(x);
    this.knowledgeAtM.push(this.dFollowed);
    // The follower turns relative to a heading *into* the obstacle; we want the
    // tangent that continues `lastDir`, so pick the turn whose tangent agrees.
    const grad = witness(this.world, x).grad;
    const tLeft = tangentFromGradient(grad, 'left');
    const turn = dot(tLeft, this.lastDir) >= 0 ? 'left' : 'right';
    this.follower = this.contact.follow(x, scale(grad, -1), turn);
    this.following = true;
    const sensed = this.sense(x);
    const dReach = this.reach(sensed.scan, sensed.intervals);
    // d_followed is the closest *any* sensed boundary has come to the goal so
    // far. Keeping it across phases is what makes "d_reach < d_followed" a
    // statement of progress rather than a trigger that fires on the first step.
    this.dFollowed = Math.min(this.dFollowed, dReach);
    this.sensing = { ...sensed, chosen: null, dReach, dFollowed: this.dFollowed };
    s.mode = 'boundary-following';
    s.reason = `boundary-following from M_${s.hits.length} (${why})`;
  }

  /**
   * d_reach: the distance from the goal to the closest *visible* point of the
   * followed obstacle — the interval of continuity that contains the shortest
   * ray, which is the wall the robot is hugging.
   */
  private reach(scan: Scan, intervals: Interval[]): number {
    const k = argminRay(scan);
    if (k < 0) return Infinity;
    const n = scan.ranges.length;
    const iv = intervals.find((v) => {
      const rel = (((k - v.from) % n) + n) % n;
      return rel < v.count;
    });
    if (!iv) return dist(scanPoint(scan, k), this.goal);
    let best = Infinity;
    for (let i = 0; i < iv.count; i++) {
      const idx = (iv.from + i) % n;
      best = Math.min(best, dist(scanPoint(scan, idx), this.goal));
    }
    return best;
  }

  /** Lines 6–9. */
  private followBoundary(): void {
    const s = this.state;
    const f = this.follower!;
    const mv = f.advance(this.opts.step);
    for (const piece of mv.pieces) pushPoint(s, piece.b);
    const x = s.x;
    if (dist(x, this.goal) <= this.opts.step + this.goalTol) {
      pushPoint(s, this.goal);
      this.finish('reached the goal while following');
      return;
    }
    const sensed = this.sense(x);
    const dReach = this.reach(sensed.scan, sensed.intervals);
    this.sensing = { ...sensed, chosen: null, dReach, dFollowed: this.dFollowed };
    // A margin of one step keeps ray discretisation from manufacturing "closer".
    const margin = this.opts.step;
    const closer = dReach < this.dFollowed - margin || (!sensed.blocked && dist(x, this.goal) < this.dFollowed - margin);
    // Leave only if motion-to-goal would actually make progress from here;
    // otherwise line 4 would fire again on the next step and the robot would
    // chatter between the two behaviors at the same point.
    if (closer && this.decide(x, sensed).progress) {
      this.dFollowed = Math.min(this.dFollowed, dReach);
      s.leaves.push(x);
      this.following = false;
      this.follower = null;
      this.side = 0;
      this.lastDir = unit(sub(this.goal, x));
      s.mode = 'motion-to-goal';
      s.reason = `d_reach ${dReach.toFixed(3)} < d_followed: leaving at L_${s.leaves.length}`;
      return;
    }
    this.dFollowed = Math.min(this.dFollowed, dReach);
    if (f.closed()) {
      s.failed = true;
      s.mode = 'failed';
      s.reason = 'unreachable: completed a cycle around the followed obstacle';
    }
  }

  private finish(reason: string): void {
    this.state.done = true;
    this.state.mode = 'done';
    this.state.reason = reason;
  }
}
