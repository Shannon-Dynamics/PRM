/**
 * Discrete-time models for iLQR and MPC: x_{k+1} = f(x_k, u_k) with Jacobians.
 *
 *   DoubleIntegrator  q̈ = u, zero-order hold — Choset's Example 11.3.1 made discrete
 *   Unicycle          Rusty: (x, y, θ), u = (v, ω), exact arc per step (the Chapter 2 model)
 *   KinematicCar      Hitch: (x, y, θ), u = (v, φ), θ̇ = (v/ℓ) tan φ, exact arc per step
 *   ReachDiscrete     (q, q̇) of Chapter 17's Reach2R, u = τ, one RK4 step per stage
 *
 * `jacobians` has analytic forms where they are two lines and central
 * differences otherwise; the check compares the two on every model.
 */

import type { Dynamics } from '../dynamics/model';
import { rk4StepDynamics } from '../dynamics/integrate';
import type { Mat, Vec } from '../prob/linalg';

export interface DiscreteDynamics {
  readonly nx: number;
  readonly nu: number;
  readonly dt: number;
  step(x: Vec, u: Vec): Vec;
  /** (f_x, f_u) at (x, u). */
  jacobians(x: Vec, u: Vec): { fx: Mat; fu: Mat };
}

/** Central-difference Jacobians of any step map — the default, and the oracle for the analytic ones. */
export function numericJacobians(f: (x: Vec, u: Vec) => Vec, x: Vec, u: Vec, h = 1e-6): { fx: Mat; fu: Mat } {
  const nx = x.length;
  const fx: Mat = Array.from({ length: nx }, () => new Array<number>(nx).fill(0));
  const fu: Mat = Array.from({ length: nx }, () => new Array<number>(u.length).fill(0));
  for (let j = 0; j < nx; j++) {
    const xp = x.slice();
    const xm = x.slice();
    xp[j] += h;
    xm[j] -= h;
    const fp = f(xp, u);
    const fm = f(xm, u);
    for (let i = 0; i < nx; i++) fx[i][j] = (fp[i] - fm[i]) / (2 * h);
  }
  for (let j = 0; j < u.length; j++) {
    const up = u.slice();
    const um = u.slice();
    up[j] += h;
    um[j] -= h;
    const fp = f(x, up);
    const fm = f(x, um);
    for (let i = 0; i < nx; i++) fu[i][j] = (fp[i] - fm[i]) / (2 * h);
  }
  return { fx, fu };
}

/** q̈ = u with zero-order hold: x⁺ = (q + q̇ Δt + ½ u Δt², q̇ + u Δt). Linear, so iLQR is exact in one pass. */
export class DoubleIntegrator implements DiscreteDynamics {
  readonly nx = 2;
  readonly nu = 1;
  constructor(readonly dt: number) {}
  step(x: Vec, u: Vec): Vec {
    return [x[0] + x[1] * this.dt + 0.5 * u[0] * this.dt * this.dt, x[1] + u[0] * this.dt];
  }
  jacobians(): { fx: Mat; fu: Mat } {
    return { fx: [[1, this.dt], [0, 1]], fu: [[0.5 * this.dt * this.dt], [this.dt]] };
  }
}

/** Rusty's unicycle, exact constant-twist arc per step. */
export class Unicycle implements DiscreteDynamics {
  readonly nx = 3;
  readonly nu = 2;
  constructor(readonly dt: number) {}
  step(x: Vec, u: Vec): Vec {
    const [v, w] = u;
    const th = x[2];
    if (Math.abs(w) < 1e-9) return [x[0] + v * this.dt * Math.cos(th), x[1] + v * this.dt * Math.sin(th), th];
    const th1 = th + w * this.dt;
    const r = v / w;
    return [x[0] + r * (Math.sin(th1) - Math.sin(th)), x[1] - r * (Math.cos(th1) - Math.cos(th)), th1];
  }
  jacobians(x: Vec, u: Vec): { fx: Mat; fu: Mat } {
    return numericJacobians((a, b) => this.step(a, b), x, u);
  }
}

/**
 * Hitch's kinematic car: u = (v, φ), curvature tan φ / ℓ, exact arc per step.
 * The steering clamp is opt-in (`maxSteer`): a clamp inside f makes the model
 * non-smooth and stalls iLQR at the saturation — a limit belongs in the cost.
 */
export class KinematicCar implements DiscreteDynamics {
  readonly nx = 3;
  readonly nu = 2;
  constructor(
    readonly dt: number,
    readonly wheelbase = 2.6,
    readonly maxSteer = Infinity,
  ) {}
  step(x: Vec, u: Vec): Vec {
    const v = u[0];
    const phi = Math.max(-this.maxSteer, Math.min(this.maxSteer, u[1]));
    const w = (v * Math.tan(phi)) / this.wheelbase;
    const th = x[2];
    if (Math.abs(w) < 1e-9) return [x[0] + v * this.dt * Math.cos(th), x[1] + v * this.dt * Math.sin(th), th];
    const th1 = th + w * this.dt;
    const r = v / w;
    return [x[0] + r * (Math.sin(th1) - Math.sin(th)), x[1] - r * (Math.cos(th1) - Math.cos(th)), th1];
  }
  jacobians(x: Vec, u: Vec): { fx: Mat; fu: Mat } {
    return numericJacobians((a, b) => this.step(a, b), x, u);
  }
}

/** Chapter 17's Reach as a discrete system: x = (q, q̇), u = τ, one RK4 step per stage. */
export class ReachDiscrete implements DiscreteDynamics {
  readonly nx: number;
  readonly nu: number;
  constructor(
    readonly dyn: Dynamics,
    readonly dt: number,
  ) {
    this.nx = 2 * dyn.n;
    this.nu = dyn.n;
  }
  step(x: Vec, u: Vec): Vec {
    const n = this.dyn.n;
    const s = rk4StepDynamics(this.dyn, { q: x.slice(0, n), qd: x.slice(n) }, u, this.dt);
    return [...s.q, ...s.qd];
  }
  jacobians(x: Vec, u: Vec): { fx: Mat; fu: Mat } {
    return numericJacobians((a, b) => this.step(a, b), x, u, 1e-5);
  }
}

/** Roll a control sequence out from x0. */
export function rollout(f: DiscreteDynamics, x0: Vec, us: readonly Vec[]): Vec[] {
  const xs = [x0.slice()];
  for (const u of us) xs.push(f.step(xs[xs.length - 1], u));
  return xs;
}
