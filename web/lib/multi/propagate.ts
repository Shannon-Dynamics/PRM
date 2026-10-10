/**
 * PREVIEW — forward propagation instead of steering (Choset §7.5.1).
 * Chapter 21 owns the full treatment: what Hitch's control function f is, why
 * it cannot be inverted into a `Steer`, controllability, Dubins curves as the
 * steer that makes this tree good. None of that is derived or inspected here.
 * Hitch's Chapter 2 `step(q, u, dt)` is used as a black box — "a well-behaved
 * control function f : 𝒬 × U → 𝒬 that describes a method for propagating a
 * robot state into the future" — and that is the whole point of this file:
 * a tree planner needs *a way to get new configurations from old ones*, and a
 * simulator is one such way.
 *
 * Chapter 12's RRT grew an edge by `extend`: pick q_rand, find q_near, steer a
 * distance η toward q_rand. A car cannot do that; the straight chart line from
 * q_near toward q_rand is not a motion the car can make. LaValle & Kuffner's
 * rule [C ref. 270–272] replaces the steer: sample a few controls, apply each
 * for Δt through f, keep the free result that lands closest to q_rand. The
 * relation q R q′ = "some u drives q to q′ in Δt collision-free" is asymmetric
 * and irreflexive — and Theorem 7.4.3 never used symmetry or reflexivity, so
 * probabilistic completeness carries over unchanged (Derivation 5).
 *
 * Everything imported: `Tree` from Chapter 12 (the node arena and kd-tree),
 * `FreeSpace` from Chapter 11, `Hitch` from Chapter 2.
 */

import type { Rng } from '../prob/rng';
import type { Pose2 } from '../geom/se2';
import type { Chart } from '../manifold/manifold';
import { Hitch, type CarCmd } from '../robots/hitch';
import type { FreeSpace } from '../sampling/free-space';
import { Tree, type NodeId } from '../sampling/tree';

/** Choset's control function f: one application of a control for a duration. */
export interface Propagate<P, U> {
  propagate(q: P, u: U, dt: number): P;
}

export interface KinoOptions<U> {
  /** Controls sampled per extension — the widget's headline. */
  controlsPerExtension: number;
  /** Δt each control is held, s. */
  dt: number;
  /** Collision checks along one propagation (the simulator is stepped this many times). */
  substeps: number;
  goalBias: number;
  /** Reaching within this distance (the chart's metric) of the goal ends the search. */
  goalRadius: number;
  sampleControl: (rng: Rng) => U;
}

export interface KinoExtension<P, U> {
  qRand: P;
  near: NodeId;
  /** Each propagated control with its endpoint and whether its whole segment was free. */
  tried: { u: U; q: P; free: boolean; dist: number }[];
  added: NodeId | null;
}

/**
 * The forward-propagated RRT. The tree is Chapter 12's `Tree` verbatim;
 * alongside it, per node, the control and the segment of simulator states that
 * produced it, so that a path is not a polyline to be interpolated but a list
 * of controls to be *re-executed* — `reintegrate` does exactly that and the
 * check demands bit-for-bit agreement.
 */
export class KinoRrtPreview<P, U> {
  readonly tree: Tree<P>;
  /** Control that produced node i (null at the root). */
  readonly control: (U | null)[] = [null];
  /** Simulator states along the edge into node i, substep by substep (root: []). */
  readonly segment: P[][] = [[]];
  iterations = 0;
  propagations = 0;
  /** Propagations whose segment hit something — the "wasted" count w14.3 shows. */
  wasted = 0;
  goalNode: NodeId | null = null;
  solvedAt: number | null = null;

  constructor(
    readonly space: Chart<P>,
    readonly free: FreeSpace<P>,
    readonly sim: Propagate<P, U>,
    root: P,
    readonly goal: P,
    readonly opts: KinoOptions<U>,
    readonly rng: Rng,
  ) {
    this.tree = new Tree(space, root, 'kd');
  }

  get solved(): boolean {
    return this.goalNode !== null;
  }

  /** Apply u for dt in `substeps` pieces, checking each intermediate state; null if any is in collision. */
  propagateChecked(q: P, u: U): P[] | null {
    this.propagations++;
    const h = this.opts.dt / this.opts.substeps;
    const out: P[] = [];
    let cur = q;
    for (let k = 0; k < this.opts.substeps; k++) {
      cur = this.sim.propagate(cur, u, h);
      if (!this.free.isFree(cur)) {
        this.wasted++;
        return null;
      }
      out.push(cur);
    }
    return out;
  }

  /** One iteration: sample, nearest, k controls through f, keep the free one closest to q_rand. */
  extendOnce(): KinoExtension<P, U> {
    this.iterations++;
    this.tree.clock = this.iterations;
    const qRand = this.rng.next() < this.opts.goalBias ? this.goal : this.space.sample(this.rng);
    const nb = this.tree.nearest(qRand);
    const qNear = this.tree.nodes[nb.idx].q;
    const tried: KinoExtension<P, U>['tried'] = [];
    let best: { u: U; seg: P[]; dist: number } | null = null;
    for (let k = 0; k < this.opts.controlsPerExtension; k++) {
      const u = this.opts.sampleControl(this.rng);
      const seg = this.propagateChecked(qNear, u);
      if (!seg) {
        tried.push({ u, q: this.sim.propagate(qNear, u, this.opts.dt), free: false, dist: Infinity });
        continue;
      }
      const qEnd = seg[seg.length - 1];
      const d = this.space.dist(qEnd, qRand);
      tried.push({ u, q: qEnd, free: true, dist: d });
      if (!best || d < best.dist) best = { u, seg, dist: d };
    }
    if (!best) return { qRand, near: nb.idx, tried, added: null };
    const id = this.tree.add(best.seg[best.seg.length - 1], nb.idx);
    this.control.push(best.u);
    this.segment.push(best.seg);
    if (this.goalNode === null && this.space.dist(this.tree.nodes[id].q, this.goal) <= this.opts.goalRadius) {
      this.goalNode = id;
      this.solvedAt = this.iterations;
    }
    return { qRand, near: nb.idx, tried, added: id };
  }

  plan(n: number): boolean {
    for (let i = 0; i < n && !this.solved; i++) this.extendOnce();
    return this.solved;
  }

  /** Root → goal as (state, control that led there). */
  path(): { q: P; u: U | null }[] | null {
    if (this.goalNode === null) return null;
    return this.tree.chainToRoot(this.goalNode).reverse().map((i) => ({ q: this.tree.nodes[i].q, u: this.control[i] }));
  }

  /** The dense state sequence along the solution: every simulator substep. */
  densePath(): P[] | null {
    if (this.goalNode === null) return null;
    const ids = this.tree.chainToRoot(this.goalNode).reverse();
    const out: P[] = [this.tree.root];
    for (const i of ids.slice(1)) out.push(...this.segment[i]);
    return out;
  }

  /**
   * Re-run the simulator from the root with the stored controls. A planner
   * that only *proposed* configurations could not pass this test; one that
   * produced them with f reproduces every node exactly.
   */
  reintegrate(): P[] | null {
    const p = this.path();
    if (!p) return null;
    const h = this.opts.dt / this.opts.substeps;
    const out: P[] = [p[0].q];
    let cur = p[0].q;
    for (let i = 1; i < p.length; i++) {
      for (let k = 0; k < this.opts.substeps; k++) cur = this.sim.propagate(cur, p[i].u!, h);
      out.push(cur);
    }
    return out;
  }
}

/* -------------------------------------------------------------------------- */
/* Hitch as a black box                                                        */
/* -------------------------------------------------------------------------- */

/** Chapter 2's kinematic update, wrapped and not looked into. SE(2) only: no trailer in the preview. */
export function hitchPropagator(car: Hitch = Hitch.car()): Propagate<Pose2, CarCmd> {
  return { propagate: (q, u, dt) => car.step({ body: q, trailer: null }, u, dt).body };
}

/** Controls: forward or reverse at one speed, steering uniform within the car's limit. */
export function hitchControls(car: Hitch = Hitch.car(), speed = 1.5, allowReverse = true): (rng: Rng) => CarCmd {
  return (rng) => ({ v: allowReverse && rng.next() < 0.35 ? -speed : speed, phi: rng.uniform(-car.p.maxSteer, car.p.maxSteer) });
}

/**
 * How undrivable a polyline of poses is. A car on a constant-curvature arc from
 * a to b has the chord's direction equal to the *mean* of its two headings, so
 * for every edge of a forward-propagated path |chord − (θ_a + θ_b)/2| is zero
 * (reversing counts as drivable too: the chord may point backward). Summed over
 * a Chapter 12 straight-line "RRT" path on SE(2), where θ is just a third
 * coordinate, the total is a sideways slide the car cannot make. The ghost
 * tree in w14.3 reports it.
 */
export function headingMismatch(path: readonly Pose2[]): number {
  let total = 0;
  const wrap = (x: number) => Math.atan2(Math.sin(x), Math.cos(x));
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1];
    const b = path[i];
    if (Math.hypot(b.x - a.x, b.y - a.y) < 1e-9) continue;
    const dir = Math.atan2(b.y - a.y, b.x - a.x);
    const mean = a.theta + 0.5 * wrap(b.theta - a.theta);
    total += Math.min(Math.abs(wrap(mean - dir)), Math.abs(wrap(mean - dir - Math.PI)));
  }
  return total;
}
