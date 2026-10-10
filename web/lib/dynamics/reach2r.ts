/**
 * Reach as Choset's Problem 10.2 — the 2R arm in a vertical plane, derived by
 * hand (Derivation 3). Links of length L_i, mass m_i, inertia I_i about a
 * center of mass r_i from joint i; gravity a_g along −x₂:
 *
 *     M₁₁ = I₁ + I₂ + m₁r₁² + m₂(L₁² + r₂² + 2 L₁ r₂ cos q₂)
 *     M₁₂ = M₂₂ + m₂ L₁ r₂ cos q₂
 *     M₂₂ = I₂ + m₂ r₂²
 *     h   = m₂ L₁ r₂ sin q₂
 *     C q̇ = ( −h q̇₂² − 2h q̇₁ q̇₂ ,  h q̇₁² )ᵀ
 *     g₁  = (m₁r₁ + m₂L₁) a_g cos q₁ + m₂ r₂ a_g cos(q₁ + q₂),   g₂ = m₂ r₂ a_g cos(q₁ + q₂)
 *
 * and only four Christoffel symbols survive (Problem 10.3):
 * Γ¹₁₂ = Γ¹₂₁ = Γ¹₂₂ = −h and Γ²₁₁ = h. The closed form is what w17.2
 * eigen-decomposes per frame; `PlanarArm` with the same parameters must agree
 * with it everywhere, and the checks say so.
 */

import type { Mat, Vec } from '../prob/linalg';
import type { Christoffel } from './lagrange';
import { MechanicalSystem } from './model';
import { PlanarArm, type LinkParams } from './planar-arm';

export interface Reach2RParams {
  l1: number;
  r1: number;
  m1: number;
  i1: number;
  l2: number;
  r2: number;
  m2: number;
  i2: number;
  /** a_g (m/s²); 0 lays the Workbench flat. */
  aG: number;
}

export class Reach2R extends MechanicalSystem {
  readonly n = 2;
  p: Reach2RParams;

  constructor(p: Reach2RParams) {
    super();
    this.p = p;
  }

  /** Uniform rods: r_i = L_i/2, I_i = m_i L_i²/12. */
  static uniform(l1: number, m1: number, l2: number, m2: number, aG: number): Reach2R {
    return new Reach2R({
      l1, r1: l1 / 2, m1, i1: (m1 * l1 * l1) / 12,
      l2, r2: l2 / 2, m2, i2: (m2 * l2 * l2) / 12,
      aG,
    });
  }

  /** The chapter's micro-example: L = 1, 1; m = 2, 1 kg; a_g = 9.81. */
  static micro(aG = 9.81): Reach2R {
    return Reach2R.uniform(1, 2, 1, 1, aG);
  }

  /** The same arm assembled from COM Jacobians — the cross-check `PlanarArm` path. */
  asPlanarArm(): PlanarArm {
    const { l1, r1, m1, i1, l2, r2, m2, i2, aG } = this.p;
    const links: LinkParams[] = [
      { length: l1, com: r1, mass: m1, inertia: i1 },
      { length: l2, com: r2, mass: m2, inertia: i2 },
    ];
    return new PlanarArm(links, aG);
  }

  /** h(q) = m₂ L₁ r₂ sin q₂ — the single function every velocity-product term is built from. */
  h(q: Vec): number {
    return this.p.m2 * this.p.l1 * this.p.r2 * Math.sin(q[1]);
  }

  inertia(q: Vec): Mat {
    const { l1, r1, m1, i1, r2, m2, i2 } = this.p;
    const c2 = Math.cos(q[1]);
    const m22 = i2 + m2 * r2 * r2;
    const m12 = m22 + m2 * l1 * r2 * c2;
    const m11 = i1 + i2 + m1 * r1 * r1 + m2 * (l1 * l1 + r2 * r2 + 2 * l1 * r2 * c2);
    return [
      [m11, m12],
      [m12, m22],
    ];
  }

  /** Closed form: four nonzero symbols. */
  christoffel(q: Vec): Christoffel {
    const h = this.h(q);
    return [
      [
        [0, -h],
        [-h, -h],
      ],
      [
        [h, 0],
        [0, 0],
      ],
    ];
  }

  velocityProduct(_q: Vec, qd: Vec): Vec {
    const h = this.h(_q);
    return [-h * qd[1] * qd[1] - 2 * h * qd[0] * qd[1], h * qd[0] * qd[0]];
  }

  gravity(q: Vec): Vec {
    const { l1, r1, m1, r2, m2, aG } = this.p;
    const c1 = Math.cos(q[0]);
    const c12 = Math.cos(q[0] + q[1]);
    return [(m1 * r1 + m2 * l1) * aG * c1 + m2 * r2 * aG * c12, m2 * r2 * aG * c12];
  }

  potentialEnergy(q: Vec): number {
    const { l1, r1, m1, r2, m2, aG } = this.p;
    return m1 * aG * r1 * Math.sin(q[0]) + m2 * aG * (l1 * Math.sin(q[0]) + r2 * Math.sin(q[0] + q[1]));
  }

  /** Center-of-mass positions and the elbow/tip, for drawing. */
  points(q: Vec): { elbow: [number, number]; tip: [number, number]; com1: [number, number]; com2: [number, number] } {
    const { l1, r1, l2, r2 } = this.p;
    const c1 = Math.cos(q[0]);
    const s1 = Math.sin(q[0]);
    const c12 = Math.cos(q[0] + q[1]);
    const s12 = Math.sin(q[0] + q[1]);
    return {
      elbow: [l1 * c1, l1 * s1],
      tip: [l1 * c1 + l2 * c12, l1 * s1 + l2 * s12],
      com1: [r1 * c1, r1 * s1],
      com2: [l1 * c1 + r2 * c12, l1 * s1 + r2 * s12],
    };
  }
}

/** Choset's worked configuration, shared with Chapter 2's FK example: q = (π/4, π/2). */
export const MICRO_Q: Vec = [Math.PI / 4, Math.PI / 2];
