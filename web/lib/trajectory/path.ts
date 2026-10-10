/**
 * Paths: twice-differentiable curves q : [0, 1] → Q (Choset §11.1).
 *
 * A path is geometry with no clock. Everything in this chapter needs q, q' and
 * q'' as functions of the path parameter s, because the path-constrained
 * dynamics a(s) s̈ + b(s) ṡ² + c(s) = u (eq. 11.6) are built from exactly those
 * three. `ddq` must be *continuous*: b(s) contains M q'', and a C¹ spline would
 * make the velocity-product term jump at every knot — the Rust `trait Path`
 * carries the same warning in its doc comment.
 *
 * Two implementations: a straight `Segment` (q'' = 0, the simplest honest path)
 * and a natural `CubicSpline` through waypoints, which is C² by construction.
 * The spline unwraps angles first — Reach lives on T², and a spline fitted to
 * raw chart coordinates would take the long way round the seam, or worse,
 * interpolate across a 2π discontinuity.
 */

import { normalizeAngle } from '../geom/se2';
import type { Vec } from '../prob/linalg';

export interface Path {
  /** Dimension of Q. */
  readonly n: number;
  q(s: number): Vec;
  /** dq/ds. */
  dq(s: number): Vec;
  /** d²q/ds². */
  ddq(s: number): Vec;
}

/** The straight line from a to b in the chart; ddq ≡ 0. */
export class Segment implements Path {
  readonly n: number;
  constructor(
    readonly a: Vec,
    readonly b: Vec,
  ) {
    this.n = a.length;
  }
  q(s: number): Vec {
    return this.a.map((ai, i) => ai + s * (this.b[i] - ai));
  }
  dq(): Vec {
    return this.a.map((ai, i) => this.b[i] - ai);
  }
  ddq(): Vec {
    return new Array<number>(this.n).fill(0);
  }
}

/**
 * Unwrap a sequence of configurations on Tⁿ so consecutive joint angles differ
 * by the shortest arc: the chart lift of a path through the seam. The first
 * waypoint is kept as given; later ones may leave (−π, π].
 */
export function unwrapWaypoints(pts: readonly Vec[], periodic: readonly boolean[] | boolean = true): Vec[] {
  if (pts.length === 0) return [];
  const n = pts[0].length;
  const per = typeof periodic === 'boolean' ? new Array<boolean>(n).fill(periodic) : periodic;
  const out: Vec[] = [pts[0].slice()];
  for (let k = 1; k < pts.length; k++) {
    const prev = out[k - 1];
    out.push(pts[k].map((v, i) => (per[i] ? prev[i] + normalizeAngle(v - prev[i]) : v)));
  }
  return out;
}

/**
 * Natural cubic spline through m ≥ 2 waypoints at uniform knots s_k = k/(m−1).
 *
 * Natural end conditions (q'' = 0 at both ends) are the right choice here: a
 * time scaling starts and ends at rest, and q'' = 0 at the ends means the
 * velocity-product term b(s) vanishes there too, so the ends of the phase plane
 * are governed by inertia and gravity alone. The second derivatives at the knots
 * come from the standard tridiagonal system, solved once per coordinate.
 */
export class CubicSpline implements Path {
  readonly n: number;
  readonly knots: number[];
  readonly pts: Vec[];
  /** Second derivatives M_k at the knots, per coordinate: m2[i][k]. */
  private readonly m2: number[][];

  constructor(waypoints: readonly Vec[], unwrap: readonly boolean[] | boolean = true) {
    if (waypoints.length < 2) throw new Error('CubicSpline: need at least two waypoints');
    this.pts = unwrap === false ? waypoints.map((p) => p.slice()) : unwrapWaypoints(waypoints, unwrap);
    this.n = this.pts[0].length;
    const m = this.pts.length;
    this.knots = Array.from({ length: m }, (_, k) => k / (m - 1));
    this.m2 = [];
    for (let i = 0; i < this.n; i++) this.m2.push(naturalSecondDerivatives(this.knots, this.pts.map((p) => p[i])));
  }

  /** Reach's T² convention: both joints periodic. */
  static onTorus(waypoints: readonly Vec[]): CubicSpline {
    return new CubicSpline(waypoints, true);
  }

  private locate(s: number): { k: number; h: number; t: number } {
    const m = this.knots.length;
    const sc = Math.min(1, Math.max(0, s));
    let k = Math.min(m - 2, Math.floor(sc * (m - 1)));
    if (k < 0) k = 0;
    const h = this.knots[k + 1] - this.knots[k];
    return { k, h, t: sc - this.knots[k] };
  }

  q(s: number): Vec {
    const { k, h, t } = this.locate(s);
    const out = new Array<number>(this.n);
    for (let i = 0; i < this.n; i++) {
      const y0 = this.pts[k][i];
      const y1 = this.pts[k + 1][i];
      const M0 = this.m2[i][k];
      const M1 = this.m2[i][k + 1];
      const u = h - t;
      // Standard natural-cubic piece on [s_k, s_k+1].
      out[i] = (M0 * u * u * u + M1 * t * t * t) / (6 * h) + (y0 / h - (M0 * h) / 6) * u + (y1 / h - (M1 * h) / 6) * t;
    }
    return out;
  }

  dq(s: number): Vec {
    const { k, h, t } = this.locate(s);
    const out = new Array<number>(this.n);
    for (let i = 0; i < this.n; i++) {
      const y0 = this.pts[k][i];
      const y1 = this.pts[k + 1][i];
      const M0 = this.m2[i][k];
      const M1 = this.m2[i][k + 1];
      const u = h - t;
      out[i] = (-M0 * u * u + M1 * t * t) / (2 * h) - (y0 / h - (M0 * h) / 6) + (y1 / h - (M1 * h) / 6);
    }
    return out;
  }

  ddq(s: number): Vec {
    const { k, h, t } = this.locate(s);
    const out = new Array<number>(this.n);
    for (let i = 0; i < this.n; i++) {
      const M0 = this.m2[i][k];
      const M1 = this.m2[i][k + 1];
      out[i] = (M0 * (h - t) + M1 * t) / h;
    }
    return out;
  }

  /** Length in the chart metric, by fine sampling — for the widgets' readouts. */
  chartLength(samples = 400): number {
    let L = 0;
    let prev = this.q(0);
    for (let k = 1; k <= samples; k++) {
      const cur = this.q(k / samples);
      L += Math.hypot(...cur.map((v, i) => v - prev[i]));
      prev = cur;
    }
    return L;
  }
}

/** Second derivatives of the natural cubic spline through (x_k, y_k): tridiagonal solve. */
function naturalSecondDerivatives(x: readonly number[], y: readonly number[]): number[] {
  const m = x.length;
  const M = new Array<number>(m).fill(0);
  if (m < 3) return M;
  // Interior equations: h_{k-1} M_{k-1} + 2(h_{k-1} + h_k) M_k + h_k M_{k+1} = 6 (d_k − d_{k-1}).
  const a = new Array<number>(m).fill(0);
  const b = new Array<number>(m).fill(0);
  const c = new Array<number>(m).fill(0);
  const r = new Array<number>(m).fill(0);
  for (let k = 1; k < m - 1; k++) {
    const h0 = x[k] - x[k - 1];
    const h1 = x[k + 1] - x[k];
    a[k] = h0;
    b[k] = 2 * (h0 + h1);
    c[k] = h1;
    r[k] = 6 * ((y[k + 1] - y[k]) / h1 - (y[k] - y[k - 1]) / h0);
  }
  // Thomas algorithm on the interior unknowns M_1 … M_{m−2}.
  for (let k = 2; k < m - 1; k++) {
    const w = a[k] / b[k - 1];
    b[k] -= w * c[k - 1];
    r[k] -= w * r[k - 1];
  }
  M[m - 2] = r[m - 2] / b[m - 2];
  for (let k = m - 3; k >= 1; k--) M[k] = (r[k] - c[k] * M[k + 1]) / b[k];
  return M;
}

/** Finite-difference derivatives of any path, to check an implementation's dq/ddq against its q. */
export function numericDerivatives(p: Path, s: number, h = 1e-4): { dq: Vec; ddq: Vec } {
  const sp = Math.min(1, s + h);
  const sm = Math.max(0, s - h);
  const qp = p.q(sp);
  const qm = p.q(sm);
  const q0 = p.q(s);
  const d = sp - sm;
  return {
    dq: qp.map((v, i) => (v - qm[i]) / d),
    ddq: qp.map((v, i) => (v - 2 * q0[i] + qm[i]) / ((d / 2) * (d / 2))),
  };
}
