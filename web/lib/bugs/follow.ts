/**
 * The two behaviors every Bug is made of, as a strategy the planners share:
 * `moveToward` (motion-to-goal, stopping at contact) and `follow` (boundary
 * following). Two implementations, chosen by the world:
 *
 *  - `polygonContact`  — a polygon world, zero clearance. Contact is an exact
 *    segment/edge intersection and following walks the polygon edge by edge.
 *    Lengths are exact, which is what lets the worked example assert 34 and 16.
 *  - `offsetContact`   — any `World` (the Apartment), clearance W* > 0. Contact
 *    is D(x) reaching W*; following is the predictor–corrector of `trace.ts`
 *    on G = D − W*. This is Choset's own prescription (§2.3.2: follow at a
 *    safe distance) and the disc-to-point reduction for Rusty.
 *
 * A follower reports each move as straight *pieces* with their arc length, so
 * the planners can do exact bookkeeping on them: the closest point to the goal
 * on a piece (Bug1's leave point) or the crossing with the m-line (Bug2's).
 */

import { type Point2, type World, distanceToWalls } from '../sim/world';
import {
  add,
  closestOnSegment,
  cross,
  dist,
  dot,
  lerp,
  norm,
  pointInPolygon,
  polygonPerimeter,
  scale,
  segmentHit,
  sub,
  unit,
  witness,
  type PolygonWorld,
  type Turn,
} from './geometry';
import { correctAlong, correctGradient, tangentFromGradient } from './trace';

/** A straight piece of a move, from arc length s0 to s0 + |b − a|. */
export interface Piece {
  a: Point2;
  b: Point2;
  s0: number;
}

export interface Move {
  pieces: Piece[];
  x: Point2;
  /** Arc length walked since following began. */
  s: number;
}

export interface Follower {
  readonly x: Point2;
  /** Arc length since the follower started. */
  readonly s: number;
  /** Walk at most `ds` along the boundary in the current direction. */
  advance(ds: number): Move;
  /** Flip the walking direction (Bug1's return to the leave point). */
  reverse(): void;
  /** Jump back to the point at arc length `s` on the walked path (s ≤ current s). */
  seek(s: number): Point2;
  /** The perimeter, once a full loop has been closed (polygon: known at once). */
  readonly perimeter: number | null;
  /** Has the follower returned to its starting point? */
  closed(): boolean;
}

export interface Contact {
  readonly world: World;
  /** W*: the distance the robot keeps from walls (0 for exact polygon walking). */
  readonly clearance: number;
  /** Move from `x` toward `target` by at most `ds`; stop at first contact. */
  moveToward(x: Point2, target: Point2, ds: number): { x: Point2; hit: boolean };
  /** Begin boundary following at contact point `x`, robot heading `heading`, turning `turn`. */
  follow(x: Point2, heading: Point2, turn: Turn): Follower;
  /** Would an infinitesimal move from `x` toward `target` enter the obstacle? */
  blocked(x: Point2, target: Point2): boolean;
  /** Perimeters of the obstacles, when the world knows them (for the bounds). */
  perimeters(): number[] | null;
}

// ---------------------------------------------------------------------------
// Exact polygon walking
// ---------------------------------------------------------------------------

class PolygonFollower implements Follower {
  x: Point2;
  s = 0;
  readonly perimeter: number;
  private edge: number;
  private t: number;
  private dir: 1 | -1;
  private readonly n: number;
  private readonly trail: Piece[] = [];

  constructor(
    private readonly poly: PolygonWorld['obstacles'][number],
    edge: number,
    t: number,
    dir: 1 | -1,
  ) {
    this.n = poly.pts.length;
    this.edge = edge;
    this.t = t;
    this.dir = dir;
    this.perimeter = polygonPerimeter(poly);
    this.x = this.position();
  }

  private edgePts(e: number): [Point2, Point2] {
    return [this.poly.pts[e], this.poly.pts[(e + 1) % this.n]];
  }

  private position(): Point2 {
    const [a, b] = this.edgePts(this.edge);
    return lerp(a, b, this.t);
  }

  advance(ds: number): Move {
    const pieces: Piece[] = [];
    let remaining = ds;
    while (remaining > 1e-15) {
      const [a, b] = this.edgePts(this.edge);
      const len = dist(a, b);
      // Distance to the end of this edge in the walking direction.
      const toEnd = this.dir === 1 ? (1 - this.t) * len : this.t * len;
      const start = this.position();
      if (toEnd > remaining + 1e-15) {
        this.t += (this.dir * remaining) / len;
        const end = this.position();
        pieces.push({ a: start, b: end, s0: this.s });
        this.s += remaining;
        remaining = 0;
      } else {
        const end = this.dir === 1 ? b : a;
        if (toEnd > 1e-15) pieces.push({ a: start, b: end, s0: this.s });
        this.s += toEnd;
        remaining -= toEnd;
        this.edge = (((this.edge + this.dir) % this.n) + this.n) % this.n;
        this.t = this.dir === 1 ? 0 : 1;
      }
    }
    this.trail.push(...pieces);
    this.x = this.position();
    return { pieces, x: this.x, s: this.s };
  }

  reverse(): void {
    this.dir = this.dir === 1 ? -1 : 1;
  }

  seek(s: number): Point2 {
    // Walk the trail back to arc length s and reposition exactly.
    for (let i = this.trail.length - 1; i >= 0; i--) {
      const p = this.trail[i];
      const len = dist(p.a, p.b);
      if (s >= p.s0 - 1e-12 && s <= p.s0 + len + 1e-12) {
        const q = lerp(p.a, p.b, len > 0 ? (s - p.s0) / len : 0);
        // Find the edge containing q.
        let best = { e: 0, t: 0, d: Infinity };
        for (let e = 0; e < this.n; e++) {
          const [a, b] = this.edgePts(e);
          const c = closestOnSegment(q, a, b);
          if (c.d < best.d) best = { e, t: c.t, d: c.d };
        }
        this.edge = best.e;
        this.t = best.t;
        this.s = s;
        this.trail.length = i + 1;
        this.trail[i] = { ...p, b: q };
        this.x = this.position();
        return this.x;
      }
    }
    return this.x;
  }

  closed(): boolean {
    return this.s >= this.perimeter - 1e-9;
  }
}

/** Contact and following on a world of disjoint simple polygons, at zero clearance. */
export function polygonContact(world: PolygonWorld): Contact {
  const eps = 1e-9;
  return {
    world,
    clearance: 0,
    perimeters: () => world.obstacles.map(polygonPerimeter),
    moveToward(x, target, ds) {
      const d = dist(x, target);
      if (d < 1e-15) return { x, hit: false };
      const step = Math.min(ds, d);
      const to = add(x, scale(unit(sub(target, x)), step));
      let bestT = Infinity;
      for (const poly of world.obstacles) {
        const n = poly.pts.length;
        for (let e = 0; e < n; e++) {
          const a = poly.pts[e];
          const b = poly.pts[(e + 1) % n];
          const h = segmentHit(x, to, { x1: a.x, y1: a.y, x2: b.x, y2: b.y });
          // A hit at t ≈ 0 is the point we are standing on — we are leaving it.
          if (h && h.t > eps && h.t < bestT) bestT = h.t;
        }
      }
      if (bestT === Infinity) return { x: to, hit: false };
      return { x: lerp(x, to, bestT), hit: true };
    },
    follow(x, heading, turn) {
      // Which polygon and edge are we touching?
      let best = { poly: -1, edge: 0, t: 0, d: Infinity };
      world.obstacles.forEach((poly, pi) => {
        const n = poly.pts.length;
        for (let e = 0; e < n; e++) {
          const c = closestOnSegment(x, poly.pts[e], poly.pts[(e + 1) % n]);
          if (c.d < best.d - 1e-12) best = { poly: pi, edge: e, t: c.t, d: c.d };
        }
      });
      const poly = world.obstacles[best.poly];
      const n = poly.pts.length;
      // Candidate walking directions along the touched edge(s); at a vertex both
      // incident edges are candidates. "Turn left" means the robot, facing the
      // obstacle along `heading`, swings to its left: pick the direction with
      // the largest component along the left perpendicular of the heading.
      const left = { x: -heading.y, y: heading.x };
      const want = turn === 'left' ? left : scale(left, -1);
      const candidates: { edge: number; t: number; dir: 1 | -1; score: number }[] = [];
      const consider = (edge: number, t: number) => {
        const a = poly.pts[edge];
        const b = poly.pts[(edge + 1) % n];
        const e = unit(sub(b, a));
        candidates.push({ edge, t, dir: 1, score: dot(e, want) });
        candidates.push({ edge, t, dir: -1, score: -dot(e, want) });
      };
      consider(best.edge, best.t);
      if (best.t < 1e-9) consider((best.edge - 1 + n) % n, 1);
      if (best.t > 1 - 1e-9) consider((best.edge + 1) % n, 0);
      candidates.sort((u, v) => v.score - u.score);
      const c = candidates[0];
      return new PolygonFollower(poly, c.edge, c.t, c.dir);
    },
    blocked(x, target) {
      const u = unit(sub(target, x));
      const probe = add(x, scale(u, 1e-6));
      return world.obstacles.some((poly) => pointInPolygon(probe, poly) && !onBoundary(probe, poly));
    },
  };
}

function onBoundary(p: Point2, poly: PolygonWorld['obstacles'][number]): boolean {
  const n = poly.pts.length;
  for (let e = 0; e < n; e++) if (closestOnSegment(p, poly.pts[e], poly.pts[(e + 1) % n]).d < 1e-9) return true;
  return false;
}

// ---------------------------------------------------------------------------
// Offset-curve following on any World (the Apartment)
// ---------------------------------------------------------------------------

class OffsetFollower implements Follower {
  x: Point2;
  s = 0;
  perimeter: number | null = null;
  private turn: Turn;
  private readonly start: Point2;
  private readonly trail: Piece[] = [];
  /** Fixed-plane corrections that went singular and fell back to the gradient step. */
  singularEvents = 0;

  constructor(
    private readonly G: (x: Point2) => number,
    private readonly DG: (x: Point2) => Point2,
    x: Point2,
    turn: Turn,
    private readonly maxHop: number,
  ) {
    this.x = correctGradient(G, DG, x, 6, 1e-12);
    this.start = this.x;
    this.turn = turn;
  }

  advance(ds: number): Move {
    const pieces: Piece[] = [];
    let remaining = ds;
    while (remaining > 1e-12) {
      const h = Math.min(remaining, this.maxHop);
      const next = this.hop(h);
      const moved = dist(this.x, next);
      pieces.push({ a: this.x, b: next, s0: this.s });
      this.s += moved;
      this.x = next;
      remaining -= h;
      if (this.perimeter === null && this.s > 3 * this.maxHop && dist(this.x, this.start) < 0.75 * this.maxHop) {
        // Closed the loop: snap onto the start so the hit point is re-encountered exactly.
        pieces.push({ a: this.x, b: this.start, s0: this.s });
        this.s += dist(this.x, this.start);
        this.x = this.start;
        this.perimeter = this.s;
        break;
      }
    }
    this.trail.push(...pieces);
    return { pieces, x: this.x, s: this.s };
  }

  /** One predictor–corrector step of length `h`; falls back at inside corners. */
  private hop(h: number): Point2 {
    const grad = this.DG(this.x);
    const gn = norm(grad);
    if (gn < 1e-12) return this.x;
    const n = scale(grad, 1 / gn);
    const t = tangentFromGradient(grad, this.turn);
    const predicted = add(this.x, scale(t, h));
    const c = correctAlong(this.G, this.DG, predicted, n, { newtonIters: 4, tol: 1e-12, singularTol: 0.25 });
    if (!c.singular && Math.abs(this.G(c.y)) < 1e-6 && dist(c.y, this.x) > 0.3 * h) return c.y;
    // The correcting line is (nearly) tangent to the level set — an inside
    // corner, where ∇D jumps. Theorem D.1.1's hypothesis has failed for the
    // fixed-plane corrector. Correct along the *current* gradient instead; if
    // that only pushes us back onto the corner, the wall we ran into is the one
    // to follow now: take its normal (the gradient at the predicted point) and
    // step along *its* tangent, which turns the corner.
    this.singularEvents++;
    const fallback = correctGradient(this.G, this.DG, predicted, 8, 1e-12);
    if (dist(fallback, this.x) > 0.3 * h) return fallback;
    const grad2 = this.DG(predicted);
    const g2 = norm(grad2);
    if (g2 < 1e-12) return fallback;
    const n2 = scale(grad2, 1 / g2);
    const t2 = tangentFromGradient(grad2, this.turn);
    const predicted2 = add(this.x, scale(t2, h));
    const c2 = correctAlong(this.G, this.DG, predicted2, n2, { newtonIters: 4, tol: 1e-12, singularTol: 0.25 });
    if (!c2.singular && Math.abs(this.G(c2.y)) < 1e-6) return c2.y;
    return correctGradient(this.G, this.DG, predicted2, 8, 1e-12);
  }

  reverse(): void {
    this.turn = this.turn === 'left' ? 'right' : 'left';
  }

  seek(s: number): Point2 {
    for (let i = this.trail.length - 1; i >= 0; i--) {
      const p = this.trail[i];
      const len = dist(p.a, p.b);
      if (s >= p.s0 - 1e-12 && s <= p.s0 + len + 1e-12) {
        const q = lerp(p.a, p.b, len > 0 ? (s - p.s0) / len : 0);
        this.x = correctGradient(this.G, this.DG, q, 6, 1e-12);
        this.s = s;
        this.trail.length = i + 1;
        this.trail[i] = { ...p, b: this.x };
        return this.x;
      }
    }
    return this.x;
  }

  closed(): boolean {
    return this.perimeter !== null;
  }
}

export interface OffsetOptions {
  /** W*: the offset the robot keeps from every wall. */
  clearance: number;
  /** Largest predictor hop; defaults to half of W* so a convex corner's arc (radius W*) is resolved. */
  maxHop?: number;
}

/**
 * Contact at D(x) = W* and following on the offset curve G = D − W*. D and ∇D
 * come from the exact witness (`geometry.witness`); the scan-based estimates
 * in `scan.ts` agree with them to sensor resolution, which the checks assert.
 */
export function offsetContact(world: World, opts: OffsetOptions): Contact {
  const W = opts.clearance;
  const maxHop = opts.maxHop ?? W / 2;
  const G = (x: Point2) => witness(world, x).d - W;
  const DG = (x: Point2) => witness(world, x).grad;
  const inside = (p: Point2) => distanceToWalls(world, p.x, p.y) < W - 1e-12 || outOfBounds(world, p);
  return {
    world,
    clearance: W,
    perimeters: () => null,
    moveToward(x, target, ds) {
      const d = dist(x, target);
      if (d < 1e-15) return { x, hit: false };
      const step = Math.min(ds, d);
      const u = unit(sub(target, x));
      // Sub-step so a chord never skips across a thin wall.
      const chunks = Math.max(1, Math.ceil(step / Math.max(W / 2, 1e-6)));
      let cur = x;
      for (let i = 0; i < chunks; i++) {
        const nxt = add(cur, scale(u, step / chunks));
        if (inside(nxt)) {
          // Bisect on the chord for the point where D = W*.
          let lo = 0;
          let hi = 1;
          for (let k = 0; k < 50; k++) {
            const mid = (lo + hi) / 2;
            if (inside(lerp(cur, nxt, mid))) hi = mid;
            else lo = mid;
          }
          return { x: lerp(cur, nxt, lo), hit: true };
        }
        cur = nxt;
      }
      return { x: cur, hit: false };
    },
    follow(x, heading, turn) {
      // Facing the obstacle along `heading`, "turn left" puts the obstacle on the
      // robot's right: the tangent sign that makes cross(heading, t) > 0.
      const grad = DG(x);
      const t = tangentFromGradient(grad, 'right');
      const leftTurn = cross(heading, t) > 0 ? 'right' : 'left';
      const side: Turn = turn === 'left' ? leftTurn : leftTurn === 'left' ? 'right' : 'left';
      return new OffsetFollower(G, DG, x, side, maxHop);
    },
    blocked(x, target) {
      const w = witness(world, x);
      if (w.d > W + 1e-7) return false;
      return dot(w.grad, unit(sub(target, x))) < -1e-9;
    },
  };
}

function outOfBounds(world: World, p: Point2): boolean {
  const b = world.bounds;
  return p.x < b.minX || p.x > b.maxX || p.y < b.minY || p.y > b.maxY;
}
