/**
 * Δ(q, q′): the local planner — "fine rain" along an edge (Choset §7.1.2).
 *
 * `Steer` is the shared artifact Chapter 11 introduces and Chapters 12, 13 and
 * 21 import. The straight-line planner follows the manifold's own geodesic
 * (`interpolate`, which on T² goes the short way through the seam) and samples
 * it at a resolution `step` far finer than the node sampling — that is the
 * two-resolution idea the chapter is named for. Two check schedules are
 * implemented, exactly as Choset's figure 7.6 draws them:
 *
 *   incremental   — q₁, q₂, … in order from q′; stops at the first collision.
 *   subdivision   — midpoint first, then the quarter points, … breadth-first;
 *                   shorter free segments are declared free sooner, which is
 *                   why it tends to win [C ref. 162, 367].
 *
 * Honesty item, kept: a finite `step` can miss a collision thinner than the
 * step. Only a distance-based schedule like Chapter 2's `edgeFree` sphere
 * marching cannot. `checkSchedule` exposes the order so w11.1 can draw the
 * numbered checks on a hovered edge.
 *
 * Chapter 12's addition, `extend`, lives here too: move at most η toward a
 * target along the interpolant, then steer that short piece. Every `Steer`
 * gets it — including Chapter 21's Dubins steer, which is the point.
 */

import type { Manifold } from '../manifold/manifold';
import type { FreeSpace } from './free-space';

export type EdgeCheck = { kind: 'incremental'; step: number } | { kind: 'subdivision'; step: number };

export interface LocalPath<P> {
  /** from, intermediate configurations, to — at the check resolution. */
  waypoints: P[];
  length: number;
}

export interface Steer<P> {
  readonly name: string;
  /** Collision-free local path from `from` to `to`, or null (Choset's NIL). */
  steer(space: Manifold<P>, free: FreeSpace<P>, from: P, to: P): LocalPath<P> | null;
  /** Can every local path be driven backward? Straight lines yes; Dubins no (Chapter 21). */
  readonly symmetric: boolean;
  /** How many times Δ has been called — the second cost counter of w11.1. */
  calls: number;
}

/**
 * The interior parameters t ∈ (0, 1) to check, in the order each schedule
 * checks them, for an edge of the given length. Endpoints are excluded: nodes
 * are checked when they are sampled, and `steer` checks `to` first anyway.
 */
export function checkSchedule(check: EdgeCheck, length: number): number[] {
  const n = Math.max(1, Math.ceil(length / check.step));
  if (check.kind === 'incremental') {
    const out: number[] = [];
    for (let i = 1; i < n; i++) out.push(i / n);
    return out;
  }
  // Subdivision: breadth-first over intervals; recurse while a piece is longer than step.
  const out: number[] = [];
  let level: [number, number][] = [[0, 1]];
  while (level.length) {
    const next: [number, number][] = [];
    for (const [a, b] of level) {
      if ((b - a) * length <= check.step) continue;
      const m = (a + b) / 2;
      out.push(m);
      next.push([a, m], [m, b]);
    }
    level = next;
  }
  return out;
}

export class StraightLine<P> implements Steer<P> {
  readonly name: string;
  readonly symmetric = true;
  calls = 0;

  constructor(readonly check: EdgeCheck = { kind: 'subdivision', step: 0.25 }) {
    this.name = `straight-line/${check.kind}@${check.step}`;
  }

  steer(space: Manifold<P>, free: FreeSpace<P>, from: P, to: P): LocalPath<P> | null {
    this.calls++;
    const length = space.dist(from, to);
    // The far endpoint first: in RRT it is a brand-new configuration, and a
    // blocked endpoint is the cheapest possible rejection.
    if (!free.isFree(to)) return null;
    const ts = checkSchedule(this.check, length);
    for (const t of ts) if (!free.isFree(space.interpolate(from, to, t))) return null;
    // Waypoints in path order (not check order), for executing or drawing the edge.
    const sorted = ts.slice().sort((a, b) => a - b);
    const waypoints = [from, ...sorted.map((t) => space.interpolate(from, to, t)), to];
    return { waypoints, length };
  }
}

/* -------------------------------------------------------------------------- */
/* Chapter 12: extend                                                          */
/* -------------------------------------------------------------------------- */

/** The three-valued answer RRT-Connect's greedy loop needs (C Algs. 11–12). */
export type ExtendStatus<P> = { kind: 'reached'; q: P } | { kind: 'advanced'; q: P } | { kind: 'trapped' };

/**
 * Move at most η from `from` toward `toward` along the geodesic, then run Δ on
 * that piece. A fixed stride is what turns RRT's nearest-neighbor pull into a
 * Voronoi bias: the node that owns q_rand's cell moves, by η, toward it.
 */
export function extend<P>(
  steer: Steer<P>,
  space: Manifold<P>,
  free: FreeSpace<P>,
  from: P,
  toward: P,
  eta: number,
): ExtendStatus<P> {
  const d = space.dist(from, toward);
  if (d <= eta) {
    return steer.steer(space, free, from, toward) ? { kind: 'reached', q: toward } : { kind: 'trapped' };
  }
  const qNew = space.interpolate(from, toward, eta / d);
  return steer.steer(space, free, from, qNew) ? { kind: 'advanced', q: qNew } : { kind: 'trapped' };
}

/* -------------------------------------------------------------------------- */
/* Postprocessing: greedy shortcutting (Choset §7.1.2, figure 7.7)              */
/* -------------------------------------------------------------------------- */

/**
 * From q_init try to connect straight to q_goal; failing that, to the
 * configuration before it, and so on; then repeat from the landing point.
 * Never lengthens a path (every replaced piece is a geodesic between two of
 * its own points), never changes its homotopy class — Chapter 13's point about
 * why postprocessing is not optimization.
 */
export function shortcutGreedy<P>(steer: Steer<P>, space: Manifold<P>, free: FreeSpace<P>, path: readonly P[]): P[] {
  if (path.length <= 2) return path.slice();
  const out: P[] = [path[0]];
  let i = 0;
  while (i < path.length - 1) {
    let j = path.length - 1;
    while (j > i + 1 && !steer.steer(space, free, path[i], path[j])) j--;
    out.push(path[j]);
    i = j;
  }
  return out;
}
