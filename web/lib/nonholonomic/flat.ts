/**
 * Differential flatness (Choset §12.5.5, Example 12.5.1): pick a curve for the
 * flat outputs and read the whole state and control off its derivatives.
 *
 * The reduced car (and the unicycle) is flat with y = (x₁, x₂), the rear-axle
 * midpoint:
 *
 *   θ = atan2(ẏ₂, ẏ₁),   v = ±‖ẏ‖,   ω = (ẏ₁ÿ₂ − ÿ₁ẏ₂)/‖ẏ‖²            (12.32–12.33)
 *
 * and for the 4-state car the curvature κ = ω/v gives the steering angle,
 * tan φ = Lκ. With one trailer hitched at the rear axle the flat output moves
 * to the *trailer's* axle midpoint: the trailer heading ψ is that curve's
 * tangent, the car's rear axle is the trailer axle plus d_h along ψ (eqs.
 * 12.36–12.37 say the hitch point moves along the trailer's heading plus a
 * perpendicular swing), and the car's heading and steering come from
 * differentiating *that* curve once and twice more. Everything is singular
 * where the flat output stops (ẏ = 0) — the honesty item Choset prints under
 * (12.33).
 *
 * A `FlatCurve` is a pair of polynomials in t ∈ [0, t_f]; fitting one to the
 * boundary conditions is a linear solve (Hermite interpolation), so "planning"
 * here is algebra. What flatness does *not* give: control bounds (the curve may
 * ask for more curvature than 1/ρ) and obstacle avoidance — both need the extra
 * coefficients and a numerical optimizer, which Choset calls "a topic of
 * current research".
 */

import { normalizeAngle, type Pose2 } from '../geom/se2';

export interface FlatCurve {
  /** Polynomial coefficients, ascending powers of t, for y₁ and y₂. */
  cx: number[];
  cy: number[];
  tf: number;
  /** Drive the curve in reverse (heading = tangent + π, v < 0). */
  reverse: boolean;
}

/** p⁽ᵏ⁾(t) for ascending coefficients c. */
export function polyDeriv(c: readonly number[], t: number, k: number): number {
  let s = 0;
  for (let i = k; i < c.length; i++) {
    let f = 1;
    for (let j = 0; j < k; j++) f *= i - j;
    s += f * c[i] * t ** (i - k);
  }
  return s;
}

/** [y, ẏ, ÿ, y⃛, y⁗] of the flat outputs at t. */
export function flatJet(curve: FlatCurve, t: number): [number, number][] {
  return [0, 1, 2, 3, 4].map((k) => [polyDeriv(curve.cx, t, k), polyDeriv(curve.cy, t, k)] as [number, number]);
}

/**
 * Hermite fit of one coordinate: values p(0), p'(0), p''(0)…, p(T), p'(T), p''(T)…
 * with `m` conditions at each end (m = 2 → cubic, m = 3 → quintic). A small
 * dense solve — the degree is at most 7.
 */
export function hermite1d(start: readonly number[], end: readonly number[], T: number): number[] {
  const m = start.length;
  const n = 2 * m;
  const A: number[][] = [];
  const b: number[] = [];
  for (const [t, vals] of [
    [0, start],
    [T, end],
  ] as const) {
    for (let k = 0; k < m; k++) {
      const row = new Array<number>(n).fill(0);
      for (let i = k; i < n; i++) {
        let f = 1;
        for (let j = 0; j < k; j++) f *= i - j;
        row[i] = f * t ** (i - k);
      }
      A.push(row);
      b.push(vals[k]);
    }
  }
  return solveDense(A, b);
}

/** Gaussian elimination with partial pivoting. */
export function solveDense(Ain: readonly number[][], bin: readonly number[]): number[] {
  const n = bin.length;
  const A = Ain.map((r) => r.slice());
  const b = bin.slice();
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    [A[c], A[p]] = [A[p], A[c]];
    [b[c], b[p]] = [b[p], b[c]];
    const d = A[c][c];
    for (let r = c + 1; r < n; r++) {
      const f = A[r][c] / d;
      if (f === 0) continue;
      for (let k = c; k < n; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = b[r];
    for (let k = r + 1; k < n; k++) s -= A[r][k] * x[k];
    x[r] = s / A[r][r];
  }
  return x;
}

export interface FlatFitOptions {
  /** 3: position and tangent (Choset's cubic); 5: also zero acceleration, so curvature 0 at both ends. */
  degree: 3 | 5;
  tf: number;
  /** ‖ẏ‖ at both ends. Default: the straight-line distance over t_f (Example 12.5.1 uses 1 for a unit move). */
  speed?: number;
  reverse?: boolean;
}

/**
 * Fit the flat outputs between two poses of the *flat point* (rear axle for the
 * car, trailer axle with a trailer). Heading enters as the tangent direction;
 * degree 5 also pins ÿ = 0, i.e. zero curvature, so the steering wheel (car)
 * or the hitch angle (trailer) is straight at both ends.
 */
export function fitFlat(p0: Pose2, p1: Pose2, opts: FlatFitOptions): FlatCurve {
  const { degree, tf } = opts;
  const reverse = opts.reverse ?? false;
  const dist = Math.hypot(p1.x - p0.x, p1.y - p0.y);
  const speed = opts.speed ?? Math.max(dist, 1e-6) / tf;
  // The tangent points along the heading forward, against it in reverse.
  const sg = reverse ? -1 : 1;
  const t0 = [sg * speed * Math.cos(p0.theta), sg * speed * Math.sin(p0.theta)];
  const t1 = [sg * speed * Math.cos(p1.theta), sg * speed * Math.sin(p1.theta)];
  const cond = (pos: number, tan: number) => (degree === 3 ? [pos, tan] : [pos, tan, 0]);
  const cx = hermite1d(cond(p0.x, t0[0]), cond(p1.x, t1[0]), tf);
  const cy = hermite1d(cond(p0.y, t0[1]), cond(p1.y, t1[1]), tf);
  return { cx, cy, tf, reverse };
}

/** Choset's Example 12.5.1 curve: y₁ = t, y₂ = 3t² − 2t³ on [0, 1]. */
export const EXAMPLE_12_5_1: FlatCurve = { cx: [0, 1, 0, 0], cy: [0, 0, 3, -2], tf: 1, reverse: false };

/* -------------------------------------------------------------------------- */
/* Lifting: state and control from the flat outputs                             */
/* -------------------------------------------------------------------------- */

export interface CarState {
  pose: Pose2;
  /** Signed speed of the rear axle. */
  v: number;
  /** Body angular rate. */
  omega: number;
  /** Signed curvature ω/v. */
  kappa: number;
  /** Steering angle atan(Lκ) for wheelbase L. */
  phi: number;
}

/** (12.32)–(12.33) plus tan φ = Lκ: the reduced car / 4-state car from its rear-axle curve. */
export function liftCar(curve: FlatCurve, t: number, wheelbase: number): CarState {
  const [y, yd, ydd] = flatJet(curve, t);
  const sp2 = yd[0] * yd[0] + yd[1] * yd[1];
  const sp = Math.sqrt(sp2);
  const tangent = Math.atan2(yd[1], yd[0]);
  const theta = curve.reverse ? normalizeAngle(tangent + Math.PI) : tangent;
  const v = curve.reverse ? -sp : sp;
  const omega = (yd[0] * ydd[1] - ydd[0] * yd[1]) / sp2;
  const kappa = omega / v;
  return { pose: { x: y[0], y: y[1], theta }, v, omega, kappa, phi: Math.atan(wheelbase * kappa) };
}

export interface CarTrailerState {
  car: CarState;
  /** Trailer heading ψ (absolute, Chapter 2's `HitchCfg.trailer`). */
  psi: number;
  /** Trailer-axle midpoint — the flat output. */
  trailerAxle: { x: number; y: number };
  /** Hitch angle θ − ψ (Choset's θ₁). */
  hitch: number;
  /** ψ̇. */
  psiDot: number;
}

/**
 * Lift the car-with-trailer from the trailer axle's curve y(t).
 *
 *   ψ = atan2(ẏ),  ψ̇ = (ẏ₁ÿ₂ − ÿ₁ẏ₂)/‖ẏ‖²,
 *   p = y + d (cos ψ, sin ψ)                      (car rear axle)
 *   ṗ = ẏ + d ψ̇ (−sin ψ, cos ψ)
 *   p̈ = ÿ + d ψ̈ (−sin ψ, cos ψ) − d ψ̇² (cos ψ, sin ψ),
 *
 * with ψ̈ from the quotient rule (the ÿ₁ÿ₂ terms cancel in the numerator's
 * derivative, so ψ̈ needs y⃛). Then the car is lifted from p exactly as in
 * `liftCar`: θ = atan2(ṗ), κ = (ṗ × p̈)/‖ṗ‖³, φ = atan(Lκ). Four derivatives of
 * the flat output in total — "differentiate once more per body".
 */
export function liftTrailer(curve: FlatCurve, t: number, d: number, wheelbase: number): CarTrailerState {
  const [y, yd, ydd, yddd] = flatJet(curve, t);
  const D = yd[0] * yd[0] + yd[1] * yd[1];
  const N = yd[0] * ydd[1] - ydd[0] * yd[1];
  const Nd = yd[0] * yddd[1] - yddd[0] * yd[1];
  const Dd = 2 * (yd[0] * ydd[0] + yd[1] * ydd[1]);
  const tangent = Math.atan2(yd[1], yd[0]);
  const psi = curve.reverse ? normalizeAngle(tangent + Math.PI) : tangent;
  const psiDot = N / D;
  const psiDdot = (Nd * D - N * Dd) / (D * D);
  // Direction of the trailer *body* (ψ), not of its velocity: in reverse the car sits ahead along ψ still.
  const c = Math.cos(psi);
  const s = Math.sin(psi);
  const p = [y[0] + d * c, y[1] + d * s];
  const pd = [yd[0] - d * psiDot * s, yd[1] + d * psiDot * c];
  const pdd = [ydd[0] - d * psiDdot * s - d * psiDot * psiDot * c, ydd[1] + d * psiDdot * c - d * psiDot * psiDot * s];
  const sp2 = pd[0] * pd[0] + pd[1] * pd[1];
  const sp = Math.sqrt(sp2);
  const tangentCar = Math.atan2(pd[1], pd[0]);
  // The car drives the same way as the trailer (a trailer cannot push the car's velocity past 90°).
  const theta = curve.reverse ? normalizeAngle(tangentCar + Math.PI) : tangentCar;
  const v = curve.reverse ? -sp : sp;
  const omega = (pd[0] * pdd[1] - pdd[0] * pd[1]) / sp2;
  const kappa = omega / v;
  return {
    car: { pose: { x: p[0], y: p[1], theta }, v, omega, kappa, phi: Math.atan(wheelbase * kappa) },
    psi,
    trailerAxle: { x: y[0], y: y[1] },
    hitch: normalizeAngle(theta - psi),
    psiDot,
  };
}

/**
 * Plan a car-with-trailer move with both bodies aligned (θ = ψ) at both ends:
 * the flat point is the trailer axle, d behind the car's rear axle, and the
 * degree-5 fit pins zero curvature there (κ_trailer = tan(θ − ψ)/d = 0).
 */
export function fitFlatTrailer(car0: Pose2, car1: Pose2, d: number, opts: Omit<FlatFitOptions, 'degree'> & { degree?: 5 }): FlatCurve {
  const ax = (q: Pose2): Pose2 => ({ x: q.x - d * Math.cos(q.theta), y: q.y - d * Math.sin(q.theta), theta: q.theta });
  return fitFlat(ax(car0), ax(car1), { ...opts, degree: 5 });
}

export interface FlatSample {
  t: number;
  pose: Pose2;
  psi: number | null;
  v: number;
  phi: number;
  kappa: number;
}

/** Sample the lifted state along a curve (car alone, or with trailer when d is given). */
export function sampleFlat(curve: FlatCurve, n: number, wheelbase: number, d: number | null): FlatSample[] {
  const out: FlatSample[] = [];
  for (let i = 0; i <= n; i++) {
    const t = (curve.tf * i) / n;
    if (d === null) {
      const s = liftCar(curve, t, wheelbase);
      out.push({ t, pose: s.pose, psi: null, v: s.v, phi: s.phi, kappa: s.kappa });
    } else {
      const s = liftTrailer(curve, t, d, wheelbase);
      out.push({ t, pose: s.car.pose, psi: s.psi, v: s.car.v, phi: s.car.phi, kappa: s.car.kappa });
    }
  }
  return out;
}

/**
 * The "known trajectory" check: integrate the true car(-with-trailer) ODE with
 * the controls (v(t), φ(t)) that flatness produced, from the lifted initial
 * state, by RK4 — and compare with the lifted state at every step. Returns the
 * worst disagreement over position, heading and trailer heading.
 */
export function flatnessReplayError(curve: FlatCurve, wheelbase: number, d: number | null, steps = 2000): number {
  const lift = (t: number) => (d === null ? { s: liftCar(curve, t, wheelbase), psi: 0 } : (() => {
    const r = liftTrailer(curve, t, d, wheelbase);
    return { s: r.car, psi: r.psi };
  })());
  const f = (t: number, x: number[]): number[] => {
    const { s } = lift(t);
    const v = s.v;
    const k = Math.tan(s.phi) / wheelbase;
    const out = [v * Math.cos(x[2]), v * Math.sin(x[2]), v * k];
    if (d !== null) out.push((v / d) * Math.sin(x[2] - x[3]));
    return out;
  };
  const l0 = lift(0);
  let x = [l0.s.pose.x, l0.s.pose.y, l0.s.pose.theta];
  if (d !== null) x.push(l0.psi);
  const h = curve.tf / steps;
  let worst = 0;
  for (let i = 0; i < steps; i++) {
    const t = i * h;
    const k1 = f(t, x);
    const k2 = f(t + h / 2, x.map((xi, j) => xi + (h / 2) * k1[j]));
    const k3 = f(t + h / 2, x.map((xi, j) => xi + (h / 2) * k2[j]));
    const k4 = f(t + h, x.map((xi, j) => xi + h * k3[j]));
    x = x.map((xi, j) => xi + (h / 6) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]));
    const ref = lift(t + h);
    worst = Math.max(worst, Math.abs(x[0] - ref.s.pose.x), Math.abs(x[1] - ref.s.pose.y), Math.abs(normalizeAngle(x[2] - ref.s.pose.theta)));
    if (d !== null) worst = Math.max(worst, Math.abs(normalizeAngle(x[3] - ref.psi)));
  }
  return worst;
}
