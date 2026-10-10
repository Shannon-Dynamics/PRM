/**
 * The verifier behind the neural planner: Chapter 11's `Steer` contract,
 * fulfilled by Chapter 2's *sound* swept check instead of a fixed-step
 * subdivision.
 *
 * Chapter 11 was honest that a finite check step can miss a collision thinner
 * than the step. A proposal that is wrong in creative ways (a net's hop can
 * graze a corner the roadmap never went near) deserves the certificate that
 * cannot miss: sphere marching on the arm's clearance, where each step is the
 * distance to the nearest obstacle divided by the Lipschitz bound on how fast
 * any point of the arm moves along the edge. The loop is Chapter 2's
 * `SweptChecker.edgeFree`, written out so that every distance query is counted
 * in the book's cost unit.
 */

import type { Manifold } from '../manifold/manifold';
import { ChainChecker } from '../collide/collision';
import type { FreeSpace } from '../sampling/free-space';
import type { LocalPath, Steer } from '../sampling/steer';
import type { TorusLab } from '../sampling/worlds';

export class SweptSteer implements Steer<number[]> {
  readonly name = 'swept (sphere marching)';
  readonly symmetric = true;
  calls = 0;
  /** Distance queries made by the marching loops (counted into `free.checks` too). */
  distanceQueries = 0;
  private readonly checker: ChainChecker;

  constructor(
    lab: Pick<TorusLab, 'scene' | 'arm'>,
    /** Floor on the parameter step, so a grazing edge cannot stall the loop. */
    readonly minStep = 1e-4,
  ) {
    this.checker = new ChainChecker(lab.scene, lab.arm);
  }

  steer(space: Manifold<number[]>, free: FreeSpace<number[]>, from: number[], to: number[]): LocalPath<number[]> | null {
    this.calls++;
    if (!free.isFree(from) || !free.isFree(to)) return null;
    const L = this.checker.sweepBound(from, to);
    const waypoints: number[][] = [from];
    if (L > 0) {
      let s = 0;
      let guard = 0;
      const counter = free as { checks?: number };
      while (s < 1 && guard++ < 100000) {
        const q = space.interpolate(from, to, s);
        const d = this.checker.distance(q);
        this.distanceQueries++;
        if (typeof counter.checks === 'number') counter.checks++;
        if (d <= 0) return null;
        if (s > 0) waypoints.push(q);
        s += Math.max(d / L, this.minStep);
      }
    }
    waypoints.push(to);
    return { waypoints, length: space.dist(from, to) };
  }
}
