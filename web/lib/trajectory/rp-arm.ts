/**
 * Choset's RP arm (Examples 10.1.2, 10.2.1, 11.2.1) as a `Dynamics`, kept here
 * only as the cross-check the design doc asks for: Choset reports a
 * minimum-time execution of "approximately 0.888 s" for the straight-line
 * Cartesian path x(s) = (2s − 1, 1) with m₁ = 5, I₁ = 0.1, r₁ = 0.2, m₂ = 3,
 * I₂ = 0.05, limits ±20 N·m and ±40 N, a_g = 9.8. The path has a zero-inertia
 * point at s = ½ (a₂(½) = 0 with b₂ ≡ 0), so it also exercises the §11.2.1 code.
 *
 *     u₁ = (I₁ + I₂ + m₁r₁² + m₂q₂²) q̈₁ + 2 m₂ q₂ q̇₁ q̇₂ + a_g (m₁r₁ + m₂q₂) cos q₁
 *     u₂ = m₂ q̈₂ − m₂ q₂ q̇₁² + a_g m₂ sin q₁
 */

import type { Mat, Vec } from '../prob/linalg';
import type { Christoffel } from '../dynamics/lagrange';
import { MechanicalSystem } from '../dynamics/model';
import type { Path } from './path';

export class RpArm extends MechanicalSystem {
  readonly n = 2;
  constructor(
    readonly m1 = 5,
    readonly i1 = 0.1,
    readonly r1 = 0.2,
    readonly m2 = 3,
    readonly i2 = 0.05,
    readonly aG = 9.8,
  ) {
    super();
  }
  inertia(q: Vec): Mat {
    return [
      [this.i1 + this.i2 + this.m1 * this.r1 * this.r1 + this.m2 * q[1] * q[1], 0],
      [0, this.m2],
    ];
  }
  /** Γ¹₁₂ = Γ¹₂₁ = m₂q₂, Γ²₁₁ = −m₂q₂ (Example 10.2.1). */
  christoffel(q: Vec): Christoffel {
    const k = this.m2 * q[1];
    return [
      [
        [0, k],
        [k, 0],
      ],
      [
        [-k, 0],
        [0, 0],
      ],
    ];
  }
  velocityProduct(q: Vec, qd: Vec): Vec {
    return [2 * this.m2 * q[1] * qd[0] * qd[1], -this.m2 * q[1] * qd[0] * qd[0]];
  }
  gravity(q: Vec): Vec {
    return [this.aG * (this.m1 * this.r1 + this.m2 * q[1]) * Math.cos(q[0]), this.aG * this.m2 * Math.sin(q[0])];
  }
  potentialEnergy(q: Vec): number {
    return this.aG * (this.m1 * this.r1 + this.m2 * q[1]) * Math.sin(q[0]);
  }
}

/** The straight-line path of Example 11.2.1 in joint coordinates (eqs. 11.15–11.17), with analytic derivatives. */
export class RpLinePath implements Path {
  readonly n = 2;
  q(s: number): Vec {
    return [Math.atan2(1, 2 * s - 1), Math.sqrt(4 * s * s - 4 * s + 2)];
  }
  dq(s: number): Vec {
    const d = 2 * s * s - 2 * s + 1;
    return [-1 / d, (4 * s - 2) / Math.sqrt(4 * s * s - 4 * s + 2)];
  }
  ddq(s: number): Vec {
    const d = 2 * s * s - 2 * s + 1;
    // d/ds(−1/d) = (4s − 2)/d²;  d/ds((4s−2)/√(2d)) = √2 / d^{3/2}  (eq. 11.17's ṡ² coefficients).
    return [(4 * s - 2) / (d * d), Math.SQRT2 / Math.pow(d, 1.5)];
  }
}
