/**
 * Kalman SLAM in its 2005 form — Choset §8.4, after Smith, Self & Cheeseman.
 *
 * Two instances:
 *
 *   §8.4.1  `simpleSlam2`  — an omnidirectional vehicle measuring the relative
 *           displacement to two uniquely identified landmarks. Linear, so it is
 *           a plain `Lti` with n = 2 + 2·2 = 6 and the sister's `Kf` runs it
 *           unchanged. Its observability matrix has rank 4: the *shape* of the
 *           map is learned, its placement in the world never is.
 *
 *   §8.4.2  `KalmanSlam`   — range–bearing SLAM for the unicycle, state
 *           [x_r, y_r, θ_r, x_ℓ1, y_ℓ1, …]. The sister's `Ekf` does the linear
 *           algebra; this class owns the sparse Jacobian (eq. 8.47), the
 *           χ² association with the new-landmark threshold (eq. 8.48), and the
 *           state augmentation Choset describes but does not write down.
 *
 * Cost per step is O(n_ℓ²) in the covariance update and the memory is
 * O(n_ℓ²); on long loops the linearization errors make the filter
 * inconsistent (sister Ch. 14). Both are why the sister book moves on to
 * factor graphs, and why this chapter stops here.
 */

import { Ekf } from '../filters/ekf';
import { angleDiff, normalizeAngle } from '../geom/se2';
import { eye, type Mat, type Vec } from '../prob/linalg';
import type { Landmark } from '../sim/world';
import { rangeBearingH } from './landmarks';
import type { Lti } from './lti';

/* -------------------------------------------------------------------------- */
/* §8.4.1 — simple SLAM, two landmarks                                        */
/* -------------------------------------------------------------------------- */

/**
 * State x = [x_r, y_r, x_ℓ1, y_ℓ1, x_ℓ2, y_ℓ2]ᵀ, input u = [u_x, u_y]ᵀ.
 *
 *   F = I₆,   G = [I₂; 0],   V = blkdiag(V_r, 0, 0),
 *   H_i = [−I₂ | … I₂ at landmark i …],   W = blkdiag(W₁, W₂).
 *
 * Landmarks do not move, so their rows of F are the identity and their rows
 * of V are zero: the filter is *told* the map is static, which is what lets
 * repeated sightings sharpen it.
 */
export function simpleSlam2(Vr: Mat, Wi: Mat): Lti {
  const n = 6;
  const G: Mat = Array.from({ length: n }, () => [0, 0]);
  G[0][0] = 1;
  G[1][1] = 1;
  const V: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
  for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) V[i][j] = Vr[i][j];
  const H: Mat = [];
  for (let i = 0; i < 2; i++) {
    const r0 = new Array<number>(n).fill(0);
    const r1 = new Array<number>(n).fill(0);
    r0[0] = -1;
    r0[2 + 2 * i] = 1;
    r1[1] = -1;
    r1[3 + 2 * i] = 1;
    H.push(r0, r1);
  }
  const W: Mat = Array.from({ length: 4 }, () => new Array<number>(4).fill(0));
  for (let i = 0; i < 2; i++) for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) W[2 * i + a][2 * i + b] = Wi[a][b];
  return { F: eye(n), G, H, V, W };
}

/**
 * The covariance of the *relative* vector x_ℓi − x_r, read off a joint
 * covariance: Var(ℓ − r) = P_ℓℓ + P_rr − P_ℓr − P_rℓ. This is the quantity
 * simple SLAM actually learns, and the one the check watches shrink while the
 * absolute entries grow.
 */
export function relativeCovariance(P: Mat, landmarkIndex: number, robotDim = 2): Mat {
  const l = robotDim + 2 * landmarkIndex;
  const out: Mat = [
    [0, 0],
    [0, 0],
  ];
  for (let a = 0; a < 2; a++) {
    for (let b = 0; b < 2; b++) {
      out[a][b] = P[l + a][l + b] + P[a][b] - P[l + a][b] - P[a][l + b];
    }
  }
  return out;
}

/* -------------------------------------------------------------------------- */
/* §8.4.2 — range–bearing SLAM                                                  */
/* -------------------------------------------------------------------------- */

export interface KalmanSlamOptions {
  /** Additive pose process covariance V_r (3×3). */
  V: Mat;
  sigmaRange: number;
  sigmaBearing: number;
  /** χ² below which a sighting is attributed to its nearest landmark. */
  chi2Match?: number;
  /** χ² above which a sighting founds a new landmark (eq. 8.48's "high threshold"). */
  chi2New?: number;
  /** Initial covariance given to a brand-new landmark, in excess of what the sighting implies. */
  newLandmarkInflate?: number;
}

export interface SlamSighting {
  /** Which state landmark the sighting was attributed to, or −1 if founded a new one, or −2 if ambiguous and dropped. */
  landmark: number;
  chi2: number;
}

/**
 * Range–bearing Kalman SLAM over the sister's `Ekf`.
 *
 * The state grows: every accepted new landmark appends two entries and the
 * `Ekf` is rebuilt around the enlarged (x, P). Rebuilding is O(n²) and happens
 * once per landmark, so it is never the bottleneck — the per-step covariance
 * update already is.
 */
export class KalmanSlam {
  ekf: Ekf;
  readonly V: Mat;
  readonly sigmaRange: number;
  readonly sigmaBearing: number;
  readonly chi2Match: number;
  readonly chi2New: number;
  readonly inflate: number;
  /** Signatures, when the simulator supplies them (never read by the filter). */
  ids: number[] = [];

  constructor(x0: Vec, P0: Mat, opts: KalmanSlamOptions) {
    this.ekf = new Ekf(x0, P0, { boxplus: slamBoxplus });
    this.V = opts.V;
    this.sigmaRange = opts.sigmaRange;
    this.sigmaBearing = opts.sigmaBearing;
    this.chi2Match = opts.chi2Match ?? 9.21;
    this.chi2New = opts.chi2New ?? 25;
    this.inflate = opts.newLandmarkInflate ?? 0;
  }

  get x(): Vec {
    return this.ekf.x;
  }
  get P(): Mat {
    return this.ekf.P;
  }
  get landmarkCount(): number {
    return (this.ekf.x.length - 3) / 2;
  }
  landmark(i: number): Landmark {
    return { x: this.ekf.x[3 + 2 * i], y: this.ekf.x[4 + 2 * i], id: this.ids[i] ?? i };
  }
  landmarkCov(i: number): Mat {
    const P = this.ekf.P;
    const l = 3 + 2 * i;
    return [
      [P[l][l], P[l][l + 1]],
      [P[l + 1][l], P[l + 1][l + 1]],
    ];
  }

  /** The unicycle moves; the map stays. F is the identity outside the pose block. */
  predict(u: [number, number]): void {
    const n = this.ekf.x.length;
    const Vfull: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) Vfull[i][j] = this.V[i][j];
    this.ekf.predict(
      (x) => {
        const out = x.slice();
        out[0] = x[0] + u[0] * Math.cos(x[2]);
        out[1] = x[1] + u[0] * Math.sin(x[2]);
        out[2] = normalizeAngle(x[2] + u[1]);
        return out;
      },
      (x) => {
        const F = eye(n);
        F[0][2] = -u[0] * Math.sin(x[2]);
        F[1][2] = u[0] * Math.cos(x[2]);
        return F;
      },
      Vfull,
    );
  }

  /**
   * Eq. 8.47: the Jacobian row pair of landmark i's range–bearing measurement
   * with respect to the full state. Dense in the first three columns, two
   * nonzero columns at the landmark, zero elsewhere.
   */
  private jacobianRows(x: Vec, i: number): Mat {
    const n = x.length;
    const l = 3 + 2 * i;
    const dx = x[0] - x[l];
    const dy = x[1] - x[l + 1];
    const q = Math.max(dx * dx + dy * dy, 1e-12);
    const rho = Math.sqrt(q);
    const r0 = new Array<number>(n).fill(0);
    const r1 = new Array<number>(n).fill(0);
    r0[0] = dx / rho;
    r0[1] = dy / rho;
    r0[l] = -dx / rho;
    r0[l + 1] = -dy / rho;
    r1[0] = -dy / q;
    r1[1] = dx / q;
    r1[2] = -1;
    r1[l] = dy / q;
    r1[l + 1] = -dx / q;
    return [r0, r1];
  }

  private W2(): Mat {
    return [
      [this.sigmaRange ** 2, 0],
      [0, this.sigmaBearing ** 2],
    ];
  }

  /** Eq. 8.48 against every landmark in the state. */
  associate(y: Vec): { landmark: number; chi2: number } {
    const x = this.ekf.x;
    const P = this.ekf.P;
    let best = -1;
    let bestChi2 = Infinity;
    for (let i = 0; i < this.landmarkCount; i++) {
      const h = rangeBearingH(x, this.landmark(i));
      const nu = [y[0] - h[0], angleDiff(y[1], h[1])];
      const H = this.jacobianRows(x, i);
      const W = this.W2();
      const S: Mat = [
        [W[0][0], 0],
        [0, W[1][1]],
      ];
      for (let a = 0; a < 2; a++)
        for (let b = 0; b < 2; b++)
          for (let p = 0; p < x.length; p++)
            for (let q = 0; q < x.length; q++) S[a][b] += H[a][p] * P[p][q] * H[b][q];
      const det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
      const chi2 = (S[1][1] * nu[0] * nu[0] - 2 * S[0][1] * nu[0] * nu[1] + S[0][0] * nu[1] * nu[1]) / det;
      if (chi2 < bestChi2) {
        bestChi2 = chi2;
        best = i;
      }
    }
    return { landmark: best, chi2: bestChi2 };
  }

  /**
   * Process one sighting: match → EKF update; novel → augment the state;
   * in-between → drop it (the honest thing to do with an ambiguous sighting).
   */
  observe(y: Vec, id?: number): SlamSighting {
    const { landmark, chi2 } = this.associate(y);
    if (landmark >= 0 && chi2 <= this.chi2Match) {
      this.ekf.update(
        y,
        (x) => rangeBearingH(x, { x: x[3 + 2 * landmark], y: x[4 + 2 * landmark], id: landmark }),
        (x) => this.jacobianRows(x, landmark),
        this.W2(),
        { residual: (z, zp) => [z[0] - zp[0], angleDiff(z[1], zp[1])] },
      );
      return { landmark, chi2 };
    }
    if (landmark < 0 || chi2 > this.chi2New) {
      this.augment(y, id);
      return { landmark: -1, chi2 };
    }
    return { landmark: -2, chi2 };
  }

  /**
   * Found a new landmark from a range–bearing sighting. Inverting Choset's
   * h: the landmark sits at distance ρ from the robot in direction
   * θ + φ + π (his bearing is measured *from* the landmark). Its covariance
   * is the pose covariance pushed through that inverse plus the sensor noise
   * pushed through the same map — the standard linearized initialization.
   */
  private augment(y: Vec, id?: number): void {
    const x = this.ekf.x;
    const P = this.ekf.P;
    const n = x.length;
    const [rho, phi] = y;
    const a = x[2] + phi + Math.PI;
    const lx = x[0] + rho * Math.cos(a);
    const ly = x[1] + rho * Math.sin(a);
    // ∂ℓ/∂(x_r, y_r, θ_r) and ∂ℓ/∂(ρ, φ)
    const Gx: Mat = [
      [1, 0, -rho * Math.sin(a)],
      [0, 1, rho * Math.cos(a)],
    ];
    const Gz: Mat = [
      [Math.cos(a), -rho * Math.sin(a)],
      [Math.sin(a), rho * Math.cos(a)],
    ];
    const W = this.W2();
    const xNew = [...x, lx, ly];
    const PNew: Mat = Array.from({ length: n + 2 }, () => new Array<number>(n + 2).fill(0));
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) PNew[i][j] = P[i][j];
    // Cross terms: Cov(ℓ, x) = Gx · P[0:3, :]
    for (let a2 = 0; a2 < 2; a2++) {
      for (let j = 0; j < n; j++) {
        let s = 0;
        for (let k = 0; k < 3; k++) s += Gx[a2][k] * P[k][j];
        PNew[n + a2][j] = s;
        PNew[j][n + a2] = s;
      }
    }
    // Own block: Gx P_xx Gxᵀ + Gz W Gzᵀ (+ optional inflation).
    for (let a2 = 0; a2 < 2; a2++) {
      for (let b = 0; b < 2; b++) {
        let s = 0;
        for (let p = 0; p < 3; p++) for (let q = 0; q < 3; q++) s += Gx[a2][p] * P[p][q] * Gx[b][q];
        for (let p = 0; p < 2; p++) s += Gz[a2][p] * W[p][p] * Gz[b][p];
        PNew[n + a2][n + b] = s + (a2 === b ? this.inflate : 0);
      }
    }
    this.ids.push(id ?? this.landmarkCount);
    this.ekf = new Ekf(xNew, PNew, { boxplus: slamBoxplus });
  }
}

/** Vector addition with the heading (index 2) wrapped. */
function slamBoxplus(x: Vec, dx: Vec): Vec {
  const out = x.map((v, i) => v + dx[i]);
  out[2] = normalizeAngle(out[2]);
  return out;
}
