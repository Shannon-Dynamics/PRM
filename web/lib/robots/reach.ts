/**
 * Reach — the planar revolute arm, and its forward kinematics (Choset §3.8,
 * Example 3.8.1; FK only — the Jacobian and singularities are Chapter 4's).
 *
 * A configuration of an N-link arm is a point on the torus Tⁿ: N angles, each
 * on S¹, each wrapped to (−π, π]. In the Rust crate this is `Tn<const N>` and
 * handing a `HitchCfg` to `Reach::fk` is a compile error; here `Tn` is a
 * readonly number array and `HitchCfg` is an object, so the same mistake is a
 * type error in TypeScript too.
 *
 * Forward kinematics as a chain of SE(2) products: joint i sits at the end of
 * link i−1 and rotates everything after it by θ_i, so the absolute heading of
 * link k is θ₁ + … + θ_k and the joint positions accumulate. For N = 2 this is
 * exactly Choset's
 *
 *   φ(θ₁, θ₂) = ( L₁ cos θ₁ + L₂ cos(θ₁ + θ₂),  L₁ sin θ₁ + L₂ sin(θ₁ + θ₂) ).
 */

import { normalizeAngle } from '../geom/se2';
import type { Point2, Segment } from '../sim/world';

/** A point of Tⁿ: joint angles in (−π, π]. `T2` and `T3` are the 2R and 3R cases. */
export type Tn = readonly number[];
export type T2 = readonly [number, number];
export type T3 = readonly [number, number, number];

/** Wrap every angle to (−π, π] — the chart of the torus the book uses. */
export function wrapCfg(q: Tn): number[] {
  return q.map(normalizeAngle);
}

/** Construct a T² point with wrapping. */
export function t2(th1: number, th2: number): T2 {
  return [normalizeAngle(th1), normalizeAngle(th2)];
}

export function t3(th1: number, th2: number, th3: number): T3 {
  return [normalizeAngle(th1), normalizeAngle(th2), normalizeAngle(th3)];
}

/** The configuration printed *as its type*: `T2 { th1: 0.785, th2: 1.571 }`. */
export function fmtTn(q: Tn, digits = 3): string {
  const fields = q.map((v, i) => `th${i + 1}: ${v.toFixed(digits)}`).join(', ');
  return `T${q.length} { ${fields} }`;
}

/** Shortest-arc distance on Tⁿ (the ℓ¹ sum of per-joint wrapped differences). */
export function torusDistance(a: Tn, b: Tn): number {
  let d = 0;
  for (let i = 0; i < a.length; i++) d += Math.abs(normalizeAngle(b[i] - a[i]));
  return d;
}

export interface ReachParams {
  /** Where the first joint is pinned. */
  base: Point2;
  /** Link lengths L₁ … L_N. */
  lengths: readonly number[];
  /** Capsule radius of each link; 0 gives exact zero-thickness segments. */
  linkRadius: number;
}

export class Reach {
  readonly base: Point2;
  readonly lengths: readonly number[];
  readonly linkRadius: number;

  constructor(params: ReachParams) {
    this.base = params.base;
    this.lengths = params.lengths;
    this.linkRadius = params.linkRadius;
  }

  get n(): number {
    return this.lengths.length;
  }

  /** The 2R arm of Choset's Example 3.8.1: L₁ = L₂ = 1 at the origin, zero-thickness links. */
  static choset(): Reach {
    return new Reach({ base: { x: 0, y: 0 }, lengths: [1, 1], linkRadius: 0 });
  }

  /** The Workbench default: the same arm with 3 cm capsules. */
  static workbench(): Reach {
    return new Reach({ base: { x: 0, y: 0 }, lengths: [1, 1], linkRadius: 0.03 });
  }

  /** The redundant 3R variant used by Exercise 5 and Chapter 14. */
  static threeLink(): Reach {
    return new Reach({ base: { x: 0, y: 0 }, lengths: [0.6, 0.5, 0.4], linkRadius: 0.03 });
  }

  /** Same arm, different link lengths (the dashboard's disclosure control). */
  withLengths(lengths: readonly number[]): Reach {
    return new Reach({ base: this.base, lengths, linkRadius: this.linkRadius });
  }

  withLinkRadius(linkRadius: number): Reach {
    return new Reach({ base: this.base, lengths: this.lengths, linkRadius });
  }

  /**
   * φ as the chain T₀₁(θ₁) T₁₂(θ₂) ⋯: returns N + 1 points, base first, tip
   * last. Linear in N; nothing is memoized because it is cheaper than a lookup.
   */
  fk(q: Tn): Point2[] {
    if (q.length !== this.n) {
      throw new Error(`Reach.fk: expected T${this.n}, got a configuration with ${q.length} angles`);
    }
    const pts: Point2[] = [{ x: this.base.x, y: this.base.y }];
    let heading = 0;
    let x = this.base.x;
    let y = this.base.y;
    for (let i = 0; i < this.n; i++) {
      heading += q[i];
      x += this.lengths[i] * Math.cos(heading);
      y += this.lengths[i] * Math.sin(heading);
      pts.push({ x, y });
    }
    return pts;
  }

  /** φ(q): the tip position alone. */
  tip(q: Tn): Point2 {
    const pts = this.fk(q);
    return pts[pts.length - 1];
  }

  /** The tip's heading θ₁ + ⋯ + θ_N — the orientation half of the 3R end-effector frame (C problem 21). */
  tipHeading(q: Tn): number {
    return normalizeAngle(q.reduce((a, b) => a + b, 0));
  }

  /** R(q) as segments, one per link, for the collision checker. */
  linkSegments(q: Tn): Segment[] {
    const pts = this.fk(q);
    const segs: Segment[] = [];
    for (let i = 0; i < this.n; i++) {
      segs.push({ x1: pts[i].x, y1: pts[i].y, x2: pts[i + 1].x, y2: pts[i + 1].y });
    }
    return segs;
  }

  /** Total reach L₁ + ⋯ + L_N: the outer radius of the workspace annulus. */
  reach(): number {
    return this.lengths.reduce((a, b) => a + b, 0);
  }

  /** The inner radius |L₁ − L₂| of the 2R annulus (0 for N ≠ 2, where the origin is reachable). */
  innerRadius(): number {
    return this.n === 2 ? Math.abs(this.lengths[0] - this.lengths[1]) : 0;
  }

  /**
   * Closed-form 2R inverse kinematics (Choset problem 20): the law of cosines
   * gives cos θ₂ from ‖x‖², with two signs — elbow-down and elbow-up. Returns
   * `[null, null]` outside the annulus, and the two solutions coincide on its
   * boundary. Defined for N = 2 only.
   */
  ik(tip: Point2): [T2 | null, T2 | null] {
    if (this.n !== 2) throw new Error('Reach.ik: closed-form IK is defined for the 2R arm');
    const [L1, L2] = this.lengths;
    const dx = tip.x - this.base.x;
    const dy = tip.y - this.base.y;
    const r2 = dx * dx + dy * dy;
    const c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (c2 < -1 - 1e-12 || c2 > 1 + 1e-12) return [null, null];
    const cc = Math.max(-1, Math.min(1, c2));
    const s2 = Math.sqrt(Math.max(0, 1 - cc * cc));
    const solve = (sign: 1 | -1): T2 => {
      const th2 = Math.atan2(sign * s2, cc);
      const th1 = Math.atan2(dy, dx) - Math.atan2(L2 * Math.sin(th2), L1 + L2 * Math.cos(th2));
      return t2(th1, th2);
    };
    return [solve(1), solve(-1)];
  }

  /** Is the tip position inside the reachable annulus? */
  reachable(tip: Point2): boolean {
    const r = Math.hypot(tip.x - this.base.x, tip.y - this.base.y);
    return r <= this.reach() + 1e-12 && r >= this.innerRadius() - 1e-12;
  }
}

/** The book's Reach: the Workbench arm with 3 cm capsule links. */
export const REACH: Reach = Reach.workbench();

/** Choset's worked-example configuration q = (π/4, π/2). */
export const REACH_HOME: T2 = [Math.PI / 4, Math.PI / 2];
