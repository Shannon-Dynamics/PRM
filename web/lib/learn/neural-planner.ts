/**
 * An MPNet-style neural planner (D22.4): a waypoint policy iterated from
 * q_start toward q_goal, every hop handed to Chapter 11's `Steer`, every
 * failed hop repaired by Chapter 12's RRT-Connect, and the whole thing
 * replaced by RRT-Connect on the original query if the repairs run out.
 *
 * The repair planner is a field, not an option: Derivation 4 says the
 * composition's guarantee *is* the repair planner's, so a planner without one
 * has only a success rate. `plan(…, { repair: false })` exists for the autopsy
 * widget, which wants to show exactly that.
 */

import type { Rng } from '../prob/rng';
import type { Steer } from '../sampling/steer';
import { SweptSteer } from './verify';
import { RrtConnect, pathIsFree } from '../sampling/tree';
import { pathLength } from '../manifold/manifold';
import type { Mlp } from './mlp';
import { ENC_DIM, encodeAngles, type EnvEncoding, type WorldCase } from './worlds';

const TAU = 2 * Math.PI;
const wrap = (a: number) => {
  let x = (a + Math.PI) % TAU;
  if (x < 0) x += TAU;
  return x - Math.PI;
};

export const POLICY_IN = ENC_DIM + 8;
export const POLICY_SIZES: readonly number[] = [POLICY_IN, 64, 2];
/** Hop length η in radians on T² — also the Steer stride of the repair trees. */
export const POLICY_ETA = 0.35;

export function policyFeatures(enc: EnvEncoding, q: readonly number[], goal: readonly number[]): Float64Array {
  const x = new Float64Array(POLICY_IN);
  x.set(enc, 0);
  x.set(encodeAngles(q), ENC_DIM);
  x.set(encodeAngles(goal), ENC_DIM + 4);
  return x;
}

/** Wrapped displacement goal ⊖ q, each component in (−π, π]. */
export function torusDelta(q: readonly number[], to: readonly number[]): number[] {
  return [wrap(to[0] - q[0]), wrap(to[1] - q[1])];
}

export interface Hop {
  from: number[];
  to: number[];
  /** Did `Steer` certify the straight segment? */
  ok: boolean;
}

export interface Repair {
  /** Indices into `waypoints` the repair bridges. */
  from: number;
  to: number;
  path: number[][] | null;
  /** Both trees' nodes (for drawing the blue mini-tree). */
  treeNodes: { q: number[]; parent: number }[][];
  attempts: number;
}

export interface NeuralPlan {
  /** The policy's raw proposal, start first; ends at the goal if the policy got there. */
  waypoints: number[][];
  /** Jittered re-proposals that were needed because a hop landed in collision. */
  reproposed: number;
  hops: Hop[];
  reachedGoal: boolean;
  repairs: Repair[];
  /** Did the planner give up on local repairs and run RRT-Connect on the whole query? */
  fellBack: boolean;
  /** The verified path (every segment passed `Steer`), or null if the fallback also failed. */
  path: number[][] | null;
  /** Length of the final path contributed by the policy's own hops vs by repair trees. */
  byNet: number;
  byRepair: number;
  checks: number;
  inferenceMs: number;
}

export interface MpnetOptions {
  eta?: number;
  maxHops?: number;
  /** RRT-Connect iterations per local repair. */
  repairAttempts?: number;
  /** RRT-Connect iterations for the whole-query fallback. */
  fallbackAttempts?: number;
  steer?: Steer<number[]>;
}

export class MpnetLike {
  readonly eta: number;
  readonly maxHops: number;
  readonly repairAttempts: number;
  readonly fallbackAttempts: number;
  readonly steer: Steer<number[]>;

  constructor(
    readonly policy: Mlp,
    readonly wc: Pick<WorldCase, 'lab' | 'enc'>,
    opts: MpnetOptions = {},
  ) {
    this.eta = opts.eta ?? POLICY_ETA;
    this.maxHops = opts.maxHops ?? 40;
    this.repairAttempts = opts.repairAttempts ?? 400;
    this.fallbackAttempts = opts.fallbackAttempts ?? 3000;
    // The sound swept certificate by default: a net's hop may graze what a fixed step misses.
    this.steer = opts.steer ?? new SweptSteer(this.wc.lab);
  }

  /** One policy step: tanh outputs scaled to η, snapping to the goal when within reach. */
  hop(q: number[], goal: number[], jitter: Rng | null = null, sigma = 0.15): number[] {
    const space = this.wc.lab.space;
    if (space.dist(q, goal) <= this.eta) return [...goal];
    const y = this.policy.forward(policyFeatures(this.wc.enc, q, goal));
    let dx = Math.tanh(y[0]) * this.eta;
    let dy = Math.tanh(y[1]) * this.eta;
    if (jitter) {
      // MPNet keeps dropout on at inference so a re-proposal differs; Gaussian jitter plays that role here.
      dx += jitter.normal(0, sigma);
      dy += jitter.normal(0, sigma);
    }
    return space.wrap([q[0] + dx, q[1] + dy]);
  }

  /**
   * The raw waypoint proposal — D22.1's "proposal", before any verifier. A hop
   * whose *endpoint* is in collision is re-proposed with jitter up to
   * `reproposals` times (the node test is the cheapest question the verifier
   * answers, and every PRM asks it of every sample); the segment is still
   * unverified here.
   */
  propose(start: readonly number[], goalIn: readonly number[], rng: Rng | null = null, reproposals = 4): { waypoints: number[][]; reachedGoal: boolean; reproposed: number } {
    const { space, free } = this.wc.lab;
    const goal = [...goalIn];
    const waypoints: number[][] = [[...start]];
    let q = [...start];
    let reproposed = 0;
    for (let k = 0; k < this.maxHops; k++) {
      let next = this.hop(q, goal);
      if (rng && !free.isFree(next)) {
        for (let j = 0; j < reproposals; j++) {
          const alt = this.hop(q, goal, rng);
          reproposed++;
          if (free.isFree(alt)) {
            next = alt;
            break;
          }
        }
      }
      // A policy that stalls (tiny step) is not making progress; stop rather than loop.
      if (space.dist(q, next) < 1e-3) break;
      waypoints.push(next);
      q = next;
      if (space.equals(q, goal, 1e-9)) return { waypoints, reachedGoal: true, reproposed };
    }
    return { waypoints, reachedGoal: false, reproposed };
  }

  private connect(a: number[], b: number[], attempts: number, rng: Rng): { path: number[][] | null; trees: Repair['treeNodes'] } {
    const { space, free } = this.wc.lab;
    const rc = new RrtConnect<number[]>(space, free, a, b, { steer: this.steer, eta: this.eta, greed: ['extend', 'connect'] }, rng);
    const path = rc.merge(attempts);
    return { path, trees: rc.trees.map((t) => t.tree.nodes.map((n) => ({ q: n.q, parent: n.parent }))) };
  }

  /**
   * Propose, verify hop by hop, repair locally, fall back globally. With
   * `repair: false` the plan is the proposal and `path` is non-null only if
   * every hop passed and the goal was reached — the honest "net alone" number.
   */
  plan(start: readonly number[], goal: readonly number[], rng: Rng, opts: { repair?: boolean } = {}): NeuralPlan {
    const { space, free } = this.wc.lab;
    const repair = opts.repair ?? true;
    const checks0 = free.checks;
    const t0 = typeof performance !== 'undefined' ? performance.now() : Date.now();
    const { waypoints, reachedGoal, reproposed } = this.propose(start, goal, rng);
    const inferenceMs = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - t0;

    const hops: Hop[] = [];
    for (let i = 1; i < waypoints.length; i++) {
      const ok = this.steer.steer(space, free, waypoints[i - 1], waypoints[i]) !== null;
      hops.push({ from: waypoints[i - 1], to: waypoints[i], ok });
    }

    const repairs: Repair[] = [];
    let path: number[][] | null = null;
    let fellBack = false;
    let byNet = 0;
    let byRepair = 0;

    if (!repair) {
      if (reachedGoal && hops.every((h) => h.ok)) {
        path = waypoints.map((w) => [...w]);
        byNet = pathLength(space, path);
      }
      return { waypoints, reproposed, hops, reachedGoal, repairs, fellBack, path, byNet, byRepair, checks: free.checks - checks0, inferenceMs };
    }

    // Lazy repair: walk the proposal; splice an RRT-Connect bridge over every failed stretch.
    const out: number[][] = [[...start]];
    let i = 0;
    let failed = false;
    // If the policy stopped short, the goal is appended as one more (unverified) waypoint.
    const pts = reachedGoal ? waypoints : [...waypoints, [...goal]];
    const target = pts.length - 1;
    while (i < target) {
      const okHop = i < hops.length && hops[i].ok;
      if (okHop) {
        out.push(pts[i + 1]);
        byNet += space.dist(pts[i], pts[i + 1]);
        i++;
        continue;
      }
      // Find the next waypoint that is itself free; bridge to it (or to the goal).
      let j = i + 1;
      while (j < target && !free.isFree(pts[j])) j++;
      const { path: bridge, trees } = this.connect(pts[i], pts[j], this.repairAttempts, rng);
      repairs.push({ from: i, to: j, path: bridge, treeNodes: trees, attempts: this.repairAttempts });
      if (!bridge) {
        failed = true;
        break;
      }
      for (let k = 1; k < bridge.length; k++) out.push(bridge[k]);
      byRepair += pathLength(space, bridge);
      i = j;
    }

    if (!failed) path = out;
    else {
      // Derivation 4: the fallback runs on the *original* query, so the guarantee is RRT-Connect's.
      fellBack = true;
      const { path: whole, trees } = this.connect([...start], [...goal], this.fallbackAttempts, rng);
      repairs.push({ from: 0, to: target, path: whole, treeNodes: trees, attempts: this.fallbackAttempts });
      path = whole;
      byNet = 0;
      byRepair = whole ? pathLength(space, whole) : 0;
    }

    // Belt and braces: re-certify every segment of what we return.
    if (path && !path.every((q, k) => k === 0 || this.steer.steer(space, free, path![k - 1], q) !== null)) {
      path = null;
    }
    return { waypoints, reproposed, hops, reachedGoal, repairs, fellBack, path, byNet, byRepair, checks: free.checks - checks0, inferenceMs };
  }

  /** An independent dense recheck of a returned path (Chapter 12's `pathIsFree`). */
  verify(path: readonly number[][], step = 0.01): boolean {
    return pathIsFree(this.wc.lab.space, this.wc.lab.free, path, step);
  }
}
