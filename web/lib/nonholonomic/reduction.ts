/**
 * Kinematic reductions (Choset §12.4.2) and planning with decoupling vector
 * fields (§12.5.7): Reach's 3R arm with the third motor off, driven like a car.
 *
 * Theorem 12.4.7 (Bullo & Lynch): V is a decoupling vector field of
 * ∇_{q̇}q̇ = Σ Y_i u_i iff V ∈ span(Y) and ∇_V V ∈ span(Y). Along an integral
 * curve of such a V the arm can move at *any* speed profile w(t) — so a path
 * planned in Q from decoupling segments is lifted to a trajectory by Chapter
 * 18's time scaling, one segment at a time, stopping at every switch.
 *
 * For the 3R arm with u₃ = 0 the free third link is the planar body with
 * thrusters, the "thrusters" being the forces joints 1–2 apply at joint 3. Its
 * two decoupling fields are Choset's: translation along link 3, and rotation
 * of link 3 about its center of percussion with respect to joint 3, at
 * distance k = (I₃ + m₃r₃²)/(m₃r₃) from the joint. In joint coordinates both
 * are built through the 2R Jacobian of joint 3's position. The checks confirm
 * Theorem 12.4.7 numerically with Chapter 20's covariant derivative — and that
 * rotating about any *other* point, or mixing the two fields, is not decoupling
 * (the PBWT is not maximally reducible).
 */

import { inv, matVec, type Mat, type Vec } from '../prob/linalg';
import { normalizeAngle } from '../geom/se2';
import { PlanarArm, type LinkParams } from '../dynamics/planar-arm';
import type { Dynamics } from '../dynamics/model';
import { PathDynamics } from '../trajectory/constrained';
import type { Path } from '../trajectory/path';
import { symmetricLimits, type TorqueLimits } from '../trajectory/limits';
import { timeScale, type TimeScaling } from '../trajectory/time-scale';
import { KeyedHeap } from '../search/heap';
import { segmentIntersectsPolygon, type Polygon } from '../collide/polygon';
import { field, relativeResidual, type VectorField } from './field';
import { kineticConnection, selfCovariant, type Connection } from './mechanical';

/* -------------------------------------------------------------------------- */
/* Theorem 12.4.7                                                               */
/* -------------------------------------------------------------------------- */

export interface DecouplingTest {
  /** ‖V − proj_span(Y) V‖ / ‖V‖. */
  inSpan: number;
  /** ‖∇_V V − proj‖ / ‖∇_V V‖ (0 when ∇_V V = 0). */
  covariantInSpan: number;
  decoupling: boolean;
}

export function isDecoupling(conn: Connection, inputs: readonly VectorField[], V: VectorField, q: Vec, tol = 1e-6): DecouplingTest {
  const Y = inputs.map((y) => y.eval(q));
  const v = V.eval(q);
  const nab = selfCovariant(conn, V).eval(q);
  const inSpan = relativeResidual(v, Y);
  const nn = Math.hypot(...nab);
  const vn = Math.hypot(...v);
  const covariantInSpan = nn < 1e-10 * Math.max(1, vn * vn) ? 0 : relativeResidual(nab, Y);
  return { inSpan, covariantInSpan, decoupling: inSpan < tol && covariantInSpan < tol };
}

/* -------------------------------------------------------------------------- */
/* Choset's Table 12.1 robot                                                    */
/* -------------------------------------------------------------------------- */

/**
 * Table 12.1: L = (0.3, 0.3), r = (0.15, 0.15, 0.15), m = (2.0, 1.0, 0.5),
 * I = (0.02, 0.01, 0.004125), τ_max = (20, 10, 0). The table gives no L₃; we
 * draw link 3 as 0.3 m (its COM at r₃ = 0.15 m is then mid-link). Horizontal
 * plane: no gravity.
 */
export const TABLE_12_1: LinkParams[] = [
  { length: 0.3, com: 0.15, mass: 2.0, inertia: 0.02 },
  { length: 0.3, com: 0.15, mass: 1.0, inertia: 0.01 },
  { length: 0.3, com: 0.15, mass: 0.5, inertia: 0.004125 },
];
export const TABLE_12_1_LIMITS: TorqueLimits = symmetricLimits([20, 10]);

export function chosetDeadArm(): PlanarArm {
  return new PlanarArm(TABLE_12_1, 0);
}

/** Distance from joint 3 to link 3's center of percussion: (I₃ + m₃r₃²)/(m₃r₃). */
export function percussionDistance(arm: PlanarArm): number {
  const l = arm.links[2];
  return (l.inertia + l.mass * l.com * l.com) / (l.mass * l.com);
}

/** Inputs Y₁, Y₂ = columns of M⁻¹[e₁ e₂] (motors on joints 1 and 2 only). */
export function deadArmInputs(arm: Dynamics): VectorField[] {
  return [0, 1].map((k) =>
    field(`Y${k + 1}`, 3, (q) => {
      const Mi = inv(arm.inertia(q));
      return [Mi[0][k], Mi[1][k], Mi[2][k]];
    }),
  );
}

export interface Link3Pose {
  /** Joint 3's position. */
  x: number;
  y: number;
  /** Link 3's absolute heading q₁ + q₂ + q₃. */
  alpha: number;
}

export function link3Pose(arm: PlanarArm, q: Vec): Link3Pose {
  const p = arm.point(q, 1, 1);
  return { x: p.x, y: p.y, alpha: normalizeAngle(q[0] + q[1] + q[2]) };
}

/** 2R inverse kinematics for joint 3, then q₃ = α − q₁ − q₂; null outside the annulus or near the elbow singularity. */
export function ikLink3(arm: PlanarArm, pose: Link3Pose, elbow: 1 | -1, minSin = 0): Vec | null {
  const L1 = arm.links[0].length;
  const L2 = arm.links[1].length;
  const dx = pose.x - arm.reach.base.x;
  const dy = pose.y - arm.reach.base.y;
  const c2 = (dx * dx + dy * dy - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (c2 < -1 || c2 > 1) return null;
  const s2 = elbow * Math.sqrt(Math.max(0, 1 - c2 * c2));
  if (Math.abs(s2) < minSin) return null;
  const q2 = Math.atan2(s2, c2);
  const q1 = Math.atan2(dy, dx) - Math.atan2(L2 * s2, L1 + L2 * c2);
  return [q1, q2, normalizeAngle(pose.alpha - q1 - q2)];
}

/**
 * The two decoupling fields, and the family "rotate about a point at distance
 * c along link 3" for the negative check (c = k is the percussion point).
 */
export function link3Fields(arm: PlanarArm): { translate: VectorField; rotate: VectorField; rotateAbout: (c: number) => VectorField } {
  const jointRates = (q: Vec, vel: [number, number], alphaDot: number): Vec => {
    const J = arm.pointJacobian(q, 1, 1);
    const J2: Mat = [
      [J[0][0], J[0][1]],
      [J[1][0], J[1][1]],
    ];
    const r = matVec(inv(J2), vel);
    return [r[0], r[1], alphaDot - r[0] - r[1]];
  };
  const translate = field('V_trans', 3, (q) => {
    const a = q[0] + q[1] + q[2];
    return jointRates(q, [Math.cos(a), Math.sin(a)], 0);
  });
  const rotateAbout = (c: number) =>
    field(`V_rot(${c.toFixed(3)})`, 3, (q) => {
      const a = q[0] + q[1] + q[2];
      // Joint 3 circles the pivot P = J₃ + c(cos α, sin α) at unit angular rate.
      return jointRates(q, [c * Math.sin(a), -c * Math.cos(a)], 1);
    });
  return { translate, rotate: rotateAbout(percussionDistance(arm)), rotateAbout };
}

/* -------------------------------------------------------------------------- */
/* The switch-minimizing grid search (Alg. 22 with four primitives)             */
/* -------------------------------------------------------------------------- */

export type DriveAction = 'T+' | 'T-' | 'R+' | 'R-';
export const DRIVE_ACTIONS: readonly DriveAction[] = ['T+', 'T-', 'R+', 'R-'];

/** Apply a primitive to link 3's pose in closed form: translate along α, or rotate about the percussion point. */
export function applyDrive(pose: Link3Pose, act: DriveAction, amount: number, k: number): Link3Pose {
  if (act[0] === 'T') {
    const s = act[1] === '+' ? amount : -amount;
    return { x: pose.x + s * Math.cos(pose.alpha), y: pose.y + s * Math.sin(pose.alpha), alpha: pose.alpha };
  }
  const da = act[1] === '+' ? amount : -amount;
  const px = pose.x + k * Math.cos(pose.alpha);
  const py = pose.y + k * Math.sin(pose.alpha);
  const a = pose.alpha + da;
  return { x: px - k * Math.cos(a), y: py - k * Math.sin(a), alpha: normalizeAngle(a) };
}

export interface DriveOptions {
  arm: PlanarArm;
  start: Link3Pose;
  goal: Link3Pose;
  posTol: number;
  angTol: number;
  /** Translation step (m) and rotation step (rad) per primitive. */
  transStep: number;
  rotStep: number;
  /** Cells per dimension of (x, y, α). */
  d: number;
  elbow: 1 | -1;
  /** Minimum |sin q₂| — keep away from the singular q₂ ∈ {0, π}. */
  minSin: number;
  obstacles: readonly Polygon[];
  maxExpansions?: number;
}

export interface DriveSegment {
  act: DriveAction;
  /** Total translation (m) or rotation (rad), positive. */
  amount: number;
  from: Link3Pose;
  to: Link3Pose;
}

export interface DriveResult {
  found: boolean;
  segments: DriveSegment[];
  switches: number;
  expansions: number;
  expanded: Link3Pose[];
}

/** Is the arm free and away from the singularity at this link-3 pose? */
export function drivePoseValid(o: Pick<DriveOptions, 'arm' | 'elbow' | 'minSin' | 'obstacles'>, pose: Link3Pose): Vec | null {
  const q = ikLink3(o.arm, pose, o.elbow, o.minSin);
  if (!q) return null;
  const pts = o.arm.reach.fk(q);
  for (let i = 0; i < 3; i++) {
    const seg = { x1: pts[i].x, y1: pts[i].y, x2: pts[i + 1].x, y2: pts[i + 1].y };
    for (const poly of o.obstacles) if (segmentIntersectsPolygon(seg, poly)) return null;
  }
  return q;
}

/**
 * Best-first over link-3 poses with the four primitives; cost is the number of
 * switches between primitives (Choset's choice), ties broken by step count.
 * The occupancy grid and the pop-time goal test are Algorithm 22's.
 */
export function decouplingGridSearch(o: DriveOptions): DriveResult {
  const k = percussionDistance(o.arm);
  const R = o.arm.links[0].length + o.arm.links[1].length;
  const d = o.d;
  const cellOf = (p: Link3Pose) => {
    const ix = Math.max(0, Math.min(d - 1, Math.floor(((p.x + R) / (2 * R)) * d)));
    const iy = Math.max(0, Math.min(d - 1, Math.floor(((p.y + R) / (2 * R)) * d)));
    let a = p.alpha % (2 * Math.PI);
    if (a < 0) a += 2 * Math.PI;
    const ia = Math.min(d - 1, Math.floor((a / (2 * Math.PI)) * d));
    return (ia * d + iy) * d + ix;
  };
  interface N {
    p: Link3Pose;
    parent: number;
    act: DriveAction | null;
    sw: number;
    steps: number;
  }
  const nodes: N[] = [{ p: o.start, parent: -1, act: null, sw: 0, steps: 0 }];
  const occupied = new Uint8Array(d * d * d);
  const heap = new KeyedHeap<number>();
  heap.push([0, 0], 0);
  const expanded: Link3Pose[] = [];
  const maxExp = o.maxExpansions ?? 40000;
  let goal = -1;
  while (heap.size && expanded.length < maxExp) {
    const id = heap.pop()!.value;
    const n = nodes[id];
    if (Math.hypot(n.p.x - o.goal.x, n.p.y - o.goal.y) <= o.posTol && Math.abs(normalizeAngle(n.p.alpha - o.goal.alpha)) <= o.angTol) {
      goal = id;
      break;
    }
    const c = cellOf(n.p);
    if (occupied[c]) continue;
    occupied[c] = 1;
    expanded.push(n.p);
    for (const act of DRIVE_ACTIONS) {
      const amt = act[0] === 'T' ? o.transStep : o.rotStep;
      let ok = true;
      let p = n.p;
      for (let s = 1; s <= 4; s++) {
        p = applyDrive(n.p, act, (amt * s) / 4, k);
        if (!drivePoseValid(o, p)) {
          ok = false;
          break;
        }
      }
      if (!ok) continue;
      const sw = n.sw + (n.act !== null && n.act !== act ? 1 : 0);
      const nid = nodes.length;
      nodes.push({ p, parent: id, act, sw, steps: n.steps + 1 });
      heap.push([sw, n.steps + 1], nid);
    }
  }
  if (goal < 0) return { found: false, segments: [], switches: 0, expansions: expanded.length, expanded };
  const chain: N[] = [];
  for (let c = goal; c >= 0; c = nodes[c].parent) chain.push(nodes[c]);
  chain.reverse();
  const segments: DriveSegment[] = [];
  for (let i = 1; i < chain.length; i++) {
    const act = chain[i].act!;
    const amt = act[0] === 'T' ? o.transStep : o.rotStep;
    const last = segments[segments.length - 1];
    if (last && last.act === act) {
      last.amount += amt;
      last.to = chain[i].p;
    } else segments.push({ act, amount: amt, from: chain[i - 1].p, to: chain[i].p });
  }
  return { found: true, segments, switches: Math.max(0, segments.length - 1), expansions: expanded.length, expanded };
}

/* -------------------------------------------------------------------------- */
/* Time scaling each segment (Chapter 18), stopping at every switch             */
/* -------------------------------------------------------------------------- */

/** A drive segment as a Chapter 18 `Path` in joint space, s ∈ [0, 1]. */
export class DrivePath implements Path {
  readonly n = 3;
  private readonly k: number;
  constructor(
    readonly arm: PlanarArm,
    readonly seg: DriveSegment,
    readonly elbow: 1 | -1,
    private readonly h = 1e-4,
  ) {
    this.k = percussionDistance(arm);
  }
  pose(s: number): Link3Pose {
    return applyDrive(this.seg.from, this.seg.act, s * this.seg.amount, this.k);
  }
  q(s: number): Vec {
    const q = ikLink3(this.arm, this.pose(s), this.elbow)!;
    // Keep q₃ continuous with the segment start (IK returns it wrapped).
    const q0 = ikLink3(this.arm, this.seg.from, this.elbow)!;
    q[2] = q0[2] + normalizeAngle(q[2] - q0[2]);
    return q;
  }
  dq(s: number): Vec {
    const a = this.q(s + this.h);
    const b = this.q(s - this.h);
    return a.map((v, i) => (v - b[i]) / (2 * this.h));
  }
  ddq(s: number): Vec {
    const a = this.q(s + this.h);
    const m = this.q(s);
    const b = this.q(s - this.h);
    return a.map((v, i) => (v - 2 * m[i] + b[i]) / (this.h * this.h));
  }
}

/** Path dynamics restricted to the two actuated rows (joint 3's row is identically zero on a decoupling path). */
export class ActuatedPathDynamics extends PathDynamics {
  abc(s: number) {
    const r = super.abc(s);
    return { a: r.a.slice(0, 2), b: r.b.slice(0, 2), c: r.c.slice(0, 2) };
  }
}

export interface ScaledSegment {
  seg: DriveSegment;
  scaling: TimeScaling | null;
  tf: number;
  /** max over the grid of |a₃|, |b₃| — zero (to FD accuracy) on a decoupling path. */
  joint3Residual: number;
  /** Fraction of grid nodes where max_i |τ_i|/τ_max,i ≥ 0.99. */
  saturatedFraction: number;
}

export function timeScaleDrive(arm: PlanarArm, segments: readonly DriveSegment[], elbow: 1 | -1, lim: TorqueLimits = TABLE_12_1_LIMITS, N = 200): { parts: ScaledSegment[]; tf: number } {
  const parts: ScaledSegment[] = [];
  let total = 0;
  for (const seg of segments) {
    const path = new DrivePath(arm, seg, elbow);
    const full = new PathDynamics(arm, path);
    let res = 0;
    for (let i = 0; i <= 20; i++) {
      const abc = full.abc(i / 20);
      res = Math.max(res, Math.abs(abc.a[2]), Math.abs(abc.b[2]));
    }
    const pd = new ActuatedPathDynamics(arm, path);
    const r = timeScale(pd, lim, { N });
    if (!r.ok) {
      parts.push({ seg, scaling: null, tf: NaN, joint3Residual: res, saturatedFraction: 0 });
      continue;
    }
    const sc = r.scaling;
    let sat = 0;
    for (let i = 0; i < sc.s.length; i++) {
      const tau = pd.torque(sc.s[i], sc.sdot[i], sc.sddot[i]);
      const m = Math.max(...tau.map((t, j) => (t >= 0 ? t / lim.max[j] : t / lim.min[j])));
      if (m >= 0.99) sat++;
    }
    parts.push({ seg, scaling: sc, tf: sc.tf, joint3Residual: res, saturatedFraction: sat / sc.s.length });
    total += sc.tf;
  }
  return { parts, tf: total };
}

/** Kinetic-energy connection of the arm — re-exported for the checks and the widget. */
export function deadArmConnection(arm: PlanarArm): Connection {
  return kineticConnection(arm);
}

/** The decoupling-drive query of w21.6 and the checks: link 3 from q = (0.4, 1.6, −0.9), moved 0.19 m and turned 0.9 rad. */
export function deadArmQuery(): DriveOptions {
  const arm = chosetDeadArm();
  const start = link3Pose(arm, [0.4, 1.6, -0.9]);
  const goal = { x: start.x - 0.15, y: start.y + 0.12, alpha: start.alpha + 0.9 };
  return { arm, start, goal, posTol: 0.03, angTol: 0.08, transStep: 0.03, rotStep: 0.1, d: 40, elbow: 1, minSin: 0.2, obstacles: [] };
}
