/**
 * Dubins and Reeds–Shepp as Chapter 11's `Steer<Pose2>` — the local planners
 * that turn PRM and RRT into car planners (Choset §12.5.6, "Randomized
 * Methods") — and Laumond's omnidirectional-to-nonholonomic path
 * transformation [C ref. 267] built on the Reeds–Shepp steer.
 *
 * Ch. 11's interface asks for a collision-free local path from `from` to `to`
 * or null. The chart's `dist` is *not* the car's distance, so the returned
 * `length` is the word's length, and a planner that wants car-optimal costs
 * must use it (Chapter 13's RRT* uses the chart metric; `KinoRrtStar` in
 * `kinodynamic.ts` uses the steer's cost, which is the point).
 */

import type { Pose2 } from '../geom/se2';
import type { Manifold } from '../manifold/manifold';
import type { FreeSpace } from '../sampling/free-space';
import type { LocalPath, Steer } from '../sampling/steer';
import { dubinsSample, dubinsShortestPath } from './dubins';
import { reedsSheppShortestPath, rsSample } from './reeds-shepp';

export class ReedsSheppLocal implements Steer<Pose2> {
  readonly name: string;
  /** A Reeds–Shepp path driven backward is the Reeds–Shepp path of the reversed query: symmetric. */
  readonly symmetric = true;
  calls = 0;
  constructor(
    readonly rho: number,
    readonly ds = 0.1,
  ) {
    this.name = `reeds-shepp(ρ=${rho.toFixed(2)})`;
  }
  steer(_space: Manifold<Pose2>, free: FreeSpace<Pose2>, from: Pose2, to: Pose2): LocalPath<Pose2> | null {
    this.calls++;
    if (!free.isFree(to)) return null;
    const p = reedsSheppShortestPath(from, to, this.rho);
    if (!p) return null;
    const waypoints = rsSample(p, from, this.ds).map((s) => s.q);
    for (const q of waypoints) if (!free.isFree(q)) return null;
    return { waypoints, length: p.length };
  }
}

export class DubinsLocal implements Steer<Pose2> {
  readonly name: string;
  /** Forward only: a Dubins path cannot be driven backward. */
  readonly symmetric = false;
  calls = 0;
  constructor(
    readonly rho: number,
    readonly ds = 0.1,
  ) {
    this.name = `dubins(ρ=${rho.toFixed(2)})`;
  }
  steer(_space: Manifold<Pose2>, free: FreeSpace<Pose2>, from: Pose2, to: Pose2): LocalPath<Pose2> | null {
    this.calls++;
    if (!free.isFree(to)) return null;
    const p = dubinsShortestPath(from, to, this.rho);
    if (!p) return null;
    const waypoints = dubinsSample(p, from, this.ds);
    for (const q of waypoints) if (!free.isFree(q)) return null;
    return { waypoints, length: p.length };
  }
}

export interface TransformResult {
  /** The Reeds–Shepp pieces, in order. */
  pieces: LocalPath<Pose2>[];
  /** Sample parameters s ∈ [0, 1] of the free-flying path at the piece boundaries. */
  breakpoints: number[];
  /** Local-planner queries spent. */
  queries: number;
  ok: boolean;
}

/**
 * Step 2 of Laumond's method: parameterize the free-flying path by s ∈ [0, 1]
 * (here, linear interpolation along its waypoints by index), try the RS path
 * q(0) → q(1); on collision bisect and recurse. It terminates when the
 * holonomic path keeps an open free neighborhood (Choset's topological
 * property) — `maxDepth` is the honest guard for paths that graze obstacles.
 */
export function pathTransform(path: readonly Pose2[], local: Steer<Pose2>, space: Manifold<Pose2>, free: FreeSpace<Pose2>, maxDepth = 14): TransformResult {
  const at = (s: number): Pose2 => {
    const x = s * (path.length - 1);
    const i = Math.min(path.length - 2, Math.floor(x));
    const f = x - i;
    return space.interpolate(path[i], path[i + 1], f);
  };
  let queries = 0;
  const pieces: LocalPath<Pose2>[] = [];
  const breaks: number[] = [0];
  let ok = true;
  const rec = (a: number, b: number, depth: number) => {
    if (!ok) return;
    queries++;
    const lp = local.steer(space, free, at(a), at(b));
    if (lp) {
      pieces.push(lp);
      breaks.push(b);
      return;
    }
    if (depth >= maxDepth) {
      ok = false;
      return;
    }
    const m = 0.5 * (a + b);
    rec(a, m, depth + 1);
    rec(m, b, depth + 1);
  };
  rec(0, 1, 0);
  return { pieces, breakpoints: breaks, queries, ok };
}
