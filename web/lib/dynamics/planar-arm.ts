/**
 * The Lagrange recipe for an N-link planar revolute arm, assembled from
 * center-of-mass Jacobians (Choset §10.1, our Derivation 3 generalized):
 *
 *     K = Σ_i ½ ( m_i ‖J_ci q̇‖² + I_i ‖J_ωi q̇‖² )
 *     M(q) = Σ_i ( m_i J_ciᵀ J_ci + I_i J_ωiᵀ J_ωi )
 *     g(q) = ∂V/∂q = Σ_i m_i a_g · (row y of J_ci),   V = Σ_i m_i a_g y_ci
 *
 * This is the pattern every modern rigid-body library uses to build M, and
 * it is how `Reach3R` (and any longer chain) gets its dynamics without a
 * hand derivation. The forward kinematics are Chapter 2's `Reach.fk`: link k
 * has heading θ_k = q_1 + … + q_k, and joint k sits at the end of link k−1.
 * The center of mass of link i is r_i along it; a general body point at
 * fraction f along link i is what CHOMP (Ch. 19) needs, so `pointJacobian`
 * takes the fraction and the COM Jacobian is the special case f = r_i / L_i.
 *
 * Christoffel symbols are finite-differenced (`MechanicalSystem.christoffel`).
 * For the 2R case `Reach2R` has the closed form and the checks pin the two.
 */

import { Reach } from '../robots/reach';
import type { Point2 } from '../sim/world';
import type { Mat, Vec } from '../prob/linalg';
import { MechanicalSystem } from './model';

export interface LinkParams {
  /** Link length L_i (m). */
  length: number;
  /** Distance r_i from joint i to the link's center of mass (m). */
  com: number;
  /** Mass m_i (kg). */
  mass: number;
  /** Inertia I_i about the center of mass (kg·m²). */
  inertia: number;
}

/** A uniform rod of length L and mass m: r = L/2, I = mL²/12. */
export const uniformRod = (length: number, mass: number): LinkParams => ({
  length,
  com: length / 2,
  mass,
  inertia: (mass * length * length) / 12,
});

export class PlanarArm extends MechanicalSystem {
  readonly n: number;
  readonly links: readonly LinkParams[];
  /** Gravitational acceleration a_g along −y; 0 lays the Workbench flat. */
  gravityAccel: number;
  /** The Chapter 2 kinematic arm with the same link lengths, for drawing and collision. */
  readonly reach: Reach;

  constructor(links: readonly LinkParams[], gravityAccel: number, base: Point2 = { x: 0, y: 0 }) {
    super();
    this.links = links;
    this.n = links.length;
    this.gravityAccel = gravityAccel;
    this.reach = new Reach({ base, lengths: links.map((l) => l.length), linkRadius: 0.03 });
  }

  /** Reach's 3R variant (Chapter 2's `Reach.threeLink`) as uniform rods: L = 0.6, 0.5, 0.4 m; m = 1.2, 1.0, 0.8 kg. */
  static reach3R(gravityAccel = 9.81): PlanarArm {
    return new PlanarArm([uniformRod(0.6, 1.2), uniformRod(0.5, 1.0), uniformRod(0.4, 0.8)], gravityAccel);
  }

  /** Cumulative headings θ_k = q_1 + … + q_k. */
  headings(q: Vec): number[] {
    const th: number[] = [];
    let s = 0;
    for (let k = 0; k < this.n; k++) {
      s += q[k];
      th.push(s);
    }
    return th;
  }

  /** Position of the point at fraction `frac` ∈ [0, 1] along link `link`. */
  point(q: Vec, link: number, frac: number): Point2 {
    const th = this.headings(q);
    let x = this.reach.base.x;
    let y = this.reach.base.y;
    for (let k = 0; k < link; k++) {
      x += this.links[k].length * Math.cos(th[k]);
      y += this.links[k].length * Math.sin(th[k]);
    }
    const d = frac * this.links[link].length;
    return { x: x + d * Math.cos(th[link]), y: y + d * Math.sin(th[link]) };
  }

  /**
   * The 2×n Jacobian of that point: column j is ∂p/∂q_j, nonzero only for
   * j ≤ link, and equal to the sum of the perpendiculars of every link segment
   * between joint j and the point — the Chapter 4 Jacobian construction
   * applied at an interior point instead of the tip.
   */
  pointJacobian(q: Vec, link: number, frac: number): Mat {
    const th = this.headings(q);
    const J: Mat = [new Array<number>(this.n).fill(0), new Array<number>(this.n).fill(0)];
    for (let j = 0; j <= link; j++) {
      let jx = 0;
      let jy = 0;
      for (let k = j; k < link; k++) {
        jx += -this.links[k].length * Math.sin(th[k]);
        jy += this.links[k].length * Math.cos(th[k]);
      }
      const d = frac * this.links[link].length;
      jx += -d * Math.sin(th[link]);
      jy += d * Math.cos(th[link]);
      J[0][j] = jx;
      J[1][j] = jy;
    }
    return J;
  }

  comJacobian(q: Vec, link: number): Mat {
    return this.pointJacobian(q, link, this.links[link].com / this.links[link].length);
  }

  inertia(q: Vec): Mat {
    const n = this.n;
    const M: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
    for (let i = 0; i < n; i++) {
      const { mass, inertia } = this.links[i];
      const Jc = this.comJacobian(q, i);
      for (let a = 0; a < n; a++) {
        for (let b = 0; b < n; b++) {
          // m_i J_cᵀ J_c — the translational part.
          let s = mass * (Jc[0][a] * Jc[0][b] + Jc[1][a] * Jc[1][b]);
          // I_i J_ωᵀ J_ω: J_ω has ones in columns 0..i, so this is I_i for a, b ≤ i.
          if (a <= i && b <= i) s += inertia;
          M[a][b] += s;
        }
      }
    }
    return M;
  }

  potentialEnergy(q: Vec): number {
    let V = 0;
    for (let i = 0; i < this.n; i++) {
      V += this.links[i].mass * this.gravityAccel * this.point(q, i, this.links[i].com / this.links[i].length).y;
    }
    return V;
  }

  /** g_j = Σ_i m_i a_g ∂y_ci/∂q_j — gravity pulled into joint space through the COM Jacobians. */
  gravity(q: Vec): Vec {
    const g = new Array<number>(this.n).fill(0);
    for (let i = 0; i < this.n; i++) {
      const Jc = this.comJacobian(q, i);
      for (let j = 0; j < this.n; j++) g[j] += this.links[i].mass * this.gravityAccel * Jc[1][j];
    }
    return g;
  }
}
