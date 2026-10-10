/**
 * Hitch — the car-like robot, optionally pulling one trailer.
 *
 * The kinematic model is *stated* here as the simulator's physics and derived
 * in Chapter 20 (it is a Pfaffian constraint) and steered in Chapter 21:
 *
 *   ẍ = v cos θ,   ẏ = v sin θ,   θ̇ = (v / ℓ) tan φ,   ψ̇ = (v / d_h) sin(θ − ψ).
 *
 * (x, y) is the rear-axle midpoint, θ the heading, φ the steering angle (a
 * *control*, not part of the configuration), ψ the trailer heading, ℓ the
 * wheelbase and d_h the hitch length from the rear axle to the trailer axle.
 *
 * The configuration type is `HitchCfg = { body: Pose2, trailer: number | null }`
 * — SE(2) × S¹ with the trailer, SE(2) without. Rear-axle no-slip means the
 * body moves on a constant-curvature arc for constant (v, φ), so the body is
 * integrated *exactly* with the ported `boxplus`; the trailer, which does not
 * move on a circle, is integrated with a fixed-step RK4 inside the tick. Both
 * are pure functions of (q, u, dt), so a replay is bit-identical.
 */

import { boxplus, normalizeAngle, type Pose2 } from '../geom/se2';
import type { Point2 } from '../sim/world';
import { orientedRect, type Polygon } from '../collide/polygon';

export interface HitchCfg {
  /** Rear-axle pose in SE(2). */
  body: Pose2;
  /** Trailer heading ψ ∈ S¹, or null when no trailer is attached. */
  trailer: number | null;
}

export interface CarCmd {
  /** Forward speed, m/s (negative reverses). */
  v: number;
  /** Steering angle φ, rad; clamped to ±maxSteer. */
  phi: number;
}

export interface HitchParams {
  wheelbase: number;
  maxSteer: number;
  bodyWidth: number;
  /** Chassis extent behind the rear axle. */
  rearOverhang: number;
  /** Chassis extent ahead of the front axle. */
  frontOverhang: number;
  /** Rear axle → trailer axle, or null for no trailer. */
  hitchLen: number | null;
  trailerLength: number;
  trailerWidth: number;
}

export const HITCH_PARAMS: HitchParams = {
  wheelbase: 2.6,
  maxSteer: 0.6,
  bodyWidth: 1.8,
  rearOverhang: 0.6,
  frontOverhang: 0.8,
  hitchLen: null,
  trailerLength: 2.2,
  trailerWidth: 1.6,
};

export class Hitch {
  readonly p: HitchParams;

  constructor(params: HitchParams = HITCH_PARAMS) {
    this.p = params;
  }

  static car(): Hitch {
    return new Hitch(HITCH_PARAMS);
  }

  /** Same car, a trailer of hitch length `d` behind it (or none). */
  withTrailer(d: number | null): Hitch {
    return new Hitch({ ...this.p, hitchLen: d });
  }

  get hasTrailer(): boolean {
    return this.p.hitchLen !== null;
  }

  /** A configuration consistent with this car: trailer aligned behind the body. */
  cfg(x: number, y: number, theta: number, psi?: number): HitchCfg {
    return {
      body: { x, y, theta: normalizeAngle(theta) },
      trailer: this.hasTrailer ? normalizeAngle(psi ?? theta) : null,
    };
  }

  clampSteer(phi: number): number {
    return Math.max(-this.p.maxSteer, Math.min(this.p.maxSteer, phi));
  }

  /** Turning radius ℓ / tan φ (Infinity when driving straight). */
  turningRadius(phi: number): number {
    const t = Math.tan(this.clampSteer(phi));
    return Math.abs(t) < 1e-12 ? Infinity : this.p.wheelbase / t;
  }

  /**
   * One tick. The constraint is *in* the model: there is no input that
   * produces a lateral velocity, so no command sequence can slide the car
   * sideways — Chapter 20 proves the sideways *displacement* is nonetheless
   * reachable, by a Lie bracket.
   */
  step(q: HitchCfg, u: CarCmd, dt: number): HitchCfg {
    const phi = this.clampSteer(u.phi);
    const kappa = Math.tan(phi) / this.p.wheelbase; // curvature
    const ds = u.v * dt;
    const dtheta = kappa * ds;
    // Exact arc: the tangent vector (ds, 0, dθ) pushed through exp on SE(2).
    const body = boxplus(q.body, [ds, 0, dtheta]);

    if (q.trailer === null || this.p.hitchLen === null) return { body, trailer: null };

    // ψ̇ = (v / d_h) sin(θ(t) − ψ), with θ(t) = θ₀ + κ v t known in closed form,
    // integrated by RK4 over the tick in four substeps. Exact for the body,
    // fourth-order for the trailer — the test replays it bit for bit anyway.
    const dh = this.p.hitchLen;
    const v = u.v;
    const th0 = q.body.theta;
    const f = (t: number, psi: number) => (v / dh) * Math.sin(th0 + kappa * v * t - psi);
    const sub = 4;
    const h = dt / sub;
    let psi = q.trailer;
    let t = 0;
    for (let i = 0; i < sub; i++) {
      const k1 = f(t, psi);
      const k2 = f(t + h / 2, psi + (h / 2) * k1);
      const k3 = f(t + h / 2, psi + (h / 2) * k2);
      const k4 = f(t + h, psi + h * k3);
      psi += (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
      t += h;
    }
    return { body, trailer: normalizeAngle(psi) };
  }

  /** Front-axle midpoint, for drawing the steered wheels. */
  frontAxle(q: HitchCfg): Point2 {
    return {
      x: q.body.x + this.p.wheelbase * Math.cos(q.body.theta),
      y: q.body.y + this.p.wheelbase * Math.sin(q.body.theta),
    };
  }

  /** Trailer-axle midpoint, d_h behind the hitch point (the rear axle) along ψ. */
  trailerAxle(q: HitchCfg): Point2 | null {
    if (q.trailer === null || this.p.hitchLen === null) return null;
    return {
      x: q.body.x - this.p.hitchLen * Math.cos(q.trailer),
      y: q.body.y - this.p.hitchLen * Math.sin(q.trailer),
    };
  }

  /** Hitch angle θ − ψ, wrapped; large values are the jackknife Chapter 23 recovers from. */
  hitchAngle(q: HitchCfg): number | null {
    return q.trailer === null ? null : normalizeAngle(q.body.theta - q.trailer);
  }

  /**
   * R(q): the chassis rectangle, and the trailer rectangle when attached. Both
   * are oriented rectangles whose poses are functions of (x, y, θ, ψ) — which
   * is the whole content of Exercise 2.
   */
  footprint(q: HitchCfg): Polygon[] {
    const { wheelbase, rearOverhang, frontOverhang, bodyWidth } = this.p;
    const halfL = (rearOverhang + wheelbase + frontOverhang) / 2;
    const off = halfL - rearOverhang; // body center sits ahead of the rear axle
    const c = Math.cos(q.body.theta);
    const s = Math.sin(q.body.theta);
    const parts: Polygon[] = [
      orientedRect(q.body.x + off * c, q.body.y + off * s, halfL, bodyWidth / 2, q.body.theta),
    ];
    const axle = this.trailerAxle(q);
    if (axle && q.trailer !== null) {
      parts.push(orientedRect(axle.x, axle.y, this.p.trailerLength / 2, this.p.trailerWidth / 2, q.trailer));
    }
    return parts;
  }

  /** Circumscribed radius of the chassis about the rear axle — the sweep bound's Lipschitz constant. */
  bodyRadius(): number {
    const { wheelbase, rearOverhang, frontOverhang, bodyWidth } = this.p;
    const ahead = wheelbase + frontOverhang;
    return Math.hypot(Math.max(ahead, rearOverhang), bodyWidth / 2);
  }

  /** Circumscribed radius of the trailer about the hitch point. */
  trailerRadius(): number {
    if (this.p.hitchLen === null) return 0;
    return Math.hypot(this.p.hitchLen + this.p.trailerLength / 2, this.p.trailerWidth / 2);
  }

  /** The configuration printed as its type. */
  static fmt(q: HitchCfg, digits = 3): string {
    const b = `Pose2 { x: ${q.body.x.toFixed(digits)}, y: ${q.body.y.toFixed(digits)}, theta: ${q.body.theta.toFixed(digits)} }`;
    return q.trailer === null
      ? `HitchCfg { body: ${b}, trailer: None }`
      : `HitchCfg { body: ${b}, trailer: Some(${q.trailer.toFixed(digits)}) }`;
  }
}

export const HITCH: Hitch = Hitch.car();
export const HITCH_WITH_TRAILER: Hitch = Hitch.car().withTrailer(2.5);
