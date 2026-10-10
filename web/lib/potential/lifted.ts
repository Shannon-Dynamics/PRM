/**
 * Potentials in non-Euclidean spaces — Choset §4.7, for Reach.
 *
 * Reach's configuration space is the torus T², which has no goal "point" a
 * workspace potential could be written at. Choset's answer is to write the
 * potential in the workspace — attractive terms at control points on the end
 * effector, repulsive terms at control points along the links — and lift each
 * workspace force to a joint torque with the transpose of that control point's
 * Jacobian (§4.7.1, virtual work: fᵀ J q̇ = uᵀ q̇ for all q̇, hence u = Jᵀ f).
 * The lifted torques are summed *in the configuration space* (eq. 4.26); summing
 * the workspace forces first would be wrong, because different control points
 * have different Jacobians.
 *
 * The tip's Jacobian is Chapter 4's `jacobian(reach, q)`; a point part-way along
 * link k is the tip of a shorter arm, so its Jacobian is the same formula with
 * the columns for joints beyond k zero and the last link shortened — written
 * out here as `controlPointJacobian`, with `jacobian` as its check at t = 1.
 *
 * `floating: true` adds Choset's r_float — the point on the arm closest to any
 * obstacle, recomputed at every q — because repelling the vertices alone does
 * not keep a link's interior out of a post (Fig. 4.20).
 */

import type { Scene } from '../collide/scene';
import { jacobian } from '../cspace/jacobian';
import type { Reach, Tn } from '../robots/reach';
import type { Point2 } from '../sim/world';
import { Attractive, Repulsive, type DistanceQuery, type ObstacleDistance, type Potential, type Vec } from './field';
import { PlanarObstacles } from './planar';

export type ControlRole =
  | { kind: 'attractive'; zeta: number; dGoalStar?: number }
  | { kind: 'repulsive'; eta: number; qStar: number };

export interface ControlPoint {
  /** Which link (0-based) the point rides on. */
  link: number;
  /** Fraction along the link, 0 at its proximal joint, 1 at its distal end. */
  t: number;
  role: ControlRole;
  label?: string;
}

/** r_j(q): the world position of a point at fraction t along link k. */
export function controlPointPosition(arm: Reach, q: Tn, link: number, t: number): Point2 {
  const pts = arm.fk(q);
  const a = pts[link];
  const b = pts[link + 1];
  return { x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) };
}

/**
 * J_j(q) for a point at fraction t along link k: joint i ≤ k swings every link
 * from i to k, the last one only for its first t·L_k. Columns for joints past k
 * are zero — turning them does not move the point. At t = 1 on the last link
 * this is Chapter 4's `jacobian`, and the chapter's check says so.
 */
export function controlPointJacobian(arm: Reach, q: Tn, link: number, t: number): [number[], number[]] {
  const n = arm.n;
  if (link === n - 1 && t === 1) return jacobian(arm, q);
  const row0 = new Array<number>(n).fill(0);
  const row1 = new Array<number>(n).fill(0);
  let h = 0;
  const headings: number[] = [];
  for (let k = 0; k < n; k++) {
    h += q[k];
    headings.push(h);
  }
  for (let i = 0; i <= link; i++) {
    for (let k = i; k <= link; k++) {
      const L = k === link ? t * arm.lengths[k] : arm.lengths[k];
      row0[i] += -L * Math.sin(headings[k]);
      row1[i] += L * Math.cos(headings[k]);
    }
  }
  return [row0, row1];
}

/** u = Jᵀ f for one control point — eq. (4.22) specialised to the arm. */
export function liftForce(J: [number[], number[]], f: Point2): Vec {
  return J[0].map((_, i) => J[0][i] * f.x + J[1][i] * f.y);
}

export interface LiftedTerm {
  point: ControlPoint;
  /** r_j(q) */
  r: Point2;
  /** The workspace force −∇U_j at r_j (what the arrows draw). */
  force: Point2;
  /** Its lift Jᵀ f. */
  torque: Vec;
  /** Potential value contributed. */
  value: number;
}

/**
 * The lifted potential for Reach. `value(q)` is Σ_j U_j(r_j(q)); `gradient(q)`
 * is Σ_j J_jᵀ ∇U_j(r_j(q)), which is the exact gradient of `value` by the chain
 * rule — including the floating point, by the envelope theorem (the minimiser
 * of a point-distance over the link may be held fixed when differentiating).
 * The chapter's check compares the two by finite differences.
 */
export class LiftedPotential implements Potential {
  readonly dim: number;
  readonly obstacles: PlanarObstacles;
  private readonly attractive: Map<number, Attractive> = new Map();
  private readonly repulsive: Map<number, Repulsive> = new Map();
  /** The last witness used for the floating control point, for drawing. */
  lastFloating: { link: number; t: number; r: Point2 } | null = null;

  constructor(
    readonly arm: Reach,
    readonly scene: Scene,
    readonly points: ControlPoint[],
    readonly goal: Tn,
    readonly floating = false,
    readonly floatingRole: { eta: number; qStar: number } = { eta: 1, qStar: 0.4 },
  ) {
    this.dim = arm.n;
    this.obstacles = PlanarObstacles.fromScene(scene);
    const goalPts = arm.fk(goal);
    points.forEach((cp, j) => {
      if (cp.role.kind === 'attractive') {
        const a = goalPts[cp.link];
        const b = goalPts[cp.link + 1];
        const rg: Vec = [a.x + cp.t * (b.x - a.x), a.y + cp.t * (b.y - a.y)];
        this.attractive.set(j, new Attractive(rg, cp.role.zeta, cp.role.dGoalStar ?? Infinity));
      } else {
        this.repulsive.set(j, new Repulsive(this.pointQuery(), cp.role.eta, cp.role.qStar));
      }
    });
  }

  /** A `DistanceQuery` that reads the point set by `setPoint` — so one `Repulsive` serves every control point. */
  private pointQuery(): DistanceQuery {
    const obs = this.obstacles;
    return { dim: 2, distances: (q: Vec): ObstacleDistance[] => obs.distances(q) };
  }

  /** r_float(q): the point on any link closest to any obstacle, as (link, t). */
  floatingPoint(q: Tn): { link: number; t: number; r: Point2; d: number } | null {
    const links = this.arm.linkSegments(q);
    let best: { link: number; t: number; r: Point2; d: number } | null = null;
    links.forEach((s, k) => {
      const w = this.scene.distanceSegment(s);
      if (!best || w.dist < best.d) {
        const len2 = (s.x2 - s.x1) ** 2 + (s.y2 - s.y1) ** 2 || 1;
        const t = ((w.onRobot.x - s.x1) * (s.x2 - s.x1) + (w.onRobot.y - s.y1) * (s.y2 - s.y1)) / len2;
        best = { link: k, t: Math.max(0, Math.min(1, t)), r: w.onRobot, d: w.dist };
      }
    });
    return best;
  }

  /** Every term with its force and lift — what w7.3 draws. */
  terms(q: Tn): LiftedTerm[] {
    const out: LiftedTerm[] = [];
    this.points.forEach((cp, j) => {
      const r = controlPointPosition(this.arm, q, cp.link, cp.t);
      const rv: Vec = [r.x, r.y];
      const U = cp.role.kind === 'attractive' ? this.attractive.get(j)! : this.repulsive.get(j)!;
      const g = U.gradient(rv);
      const J = controlPointJacobian(this.arm, q, cp.link, cp.t);
      out.push({ point: cp, r, force: { x: -g[0], y: -g[1] }, torque: liftForce(J, { x: g[0], y: g[1] }), value: U.value(rv) });
    });
    if (this.floating) {
      const fp = this.floatingPoint(q);
      this.lastFloating = fp;
      if (fp) {
        const U = new Repulsive(this.pointQuery(), this.floatingRole.eta, this.floatingRole.qStar);
        const rv: Vec = [fp.r.x, fp.r.y];
        const g = U.gradient(rv);
        const J = controlPointJacobian(this.arm, q, fp.link, fp.t);
        const cp: ControlPoint = { link: fp.link, t: fp.t, role: { kind: 'repulsive', ...this.floatingRole }, label: 'r_float' };
        out.push({ point: cp, r: fp.r, force: { x: -g[0], y: -g[1] }, torque: liftForce(J, { x: g[0], y: g[1] }), value: U.value(rv) });
      }
    }
    return out;
  }

  value(q: Vec): number {
    let u = 0;
    for (const t of this.terms(q)) u += t.value;
    return u;
  }

  /** u(q) = Σ_j J_jᵀ ∇U_j — eq. (4.26), summed in C-space. */
  gradient(q: Vec): Vec {
    const u = new Array<number>(this.dim).fill(0);
    for (const t of this.terms(q)) for (let i = 0; i < this.dim; i++) u[i] += t.torque[i];
    return u;
  }
}

/**
 * The Workbench default of w7.3: two attractive points on the end effector
 * (tip and mid-link-2, enough to pin a planar body) and repulsive points at
 * the quarter marks of each link.
 */
export function defaultControlPoints(arm: Reach, opts: { zeta?: number; eta?: number; qStar?: number; perLink?: number } = {}): ControlPoint[] {
  const { zeta = 1, eta = 0.05, qStar = 0.35, perLink = 2 } = opts;
  const pts: ControlPoint[] = [
    { link: arm.n - 1, t: 1, role: { kind: 'attractive', zeta }, label: 'tip' },
    { link: arm.n - 1, t: 0.5, role: { kind: 'attractive', zeta: zeta * 0.5 }, label: 'mid' },
  ];
  for (let k = 0; k < arm.n; k++) {
    for (let m = 1; m <= perLink; m++) {
      pts.push({ link: k, t: m / (perLink + 1), role: { kind: 'repulsive', eta, qStar }, label: `L${k + 1}·${m}` });
    }
  }
  return pts;
}
