/**
 * Exact steering without obstacles by closed loops in the shape variables:
 * Brockett's energy-optimal sinusoids (Choset §12.5.1) and the chained form
 * steered by sinusoids (§12.5.2, eqs. 12.28–12.31).
 *
 * Chained form, n states, two inputs:
 *
 *   ẋ₁ = u₁,  ẋ₂ = u₂,  ẋ₃ = x₂u₁,  ẋ₄ = x₃u₁,  …,  ẋₙ = xₙ₋₁u₁.
 *
 * The Lie brackets [g₁, [g₁, … [g₁, g₂]]] with k copies of g₁ equal (−1)ᵏ e_{k+2},
 * so the system is STLC everywhere (Chapter 20's LARC with the ladder spelled
 * out). Steering: drive x₁, x₂ straight to their goals; then for k = 1 … n−2 run
 * u₁ = a sin 2πt, u₂ = b cos 2kπt for one unit of time. That cycle returns
 * x₁ … x_{k+1} and moves x_{k+2} by exactly (a/4π)ᵏ b / k! (eq. 12.31) — the
 * higher states move too, which is why the ladder is climbed in ascending order.
 *
 * Two physical systems are put in chained form here: the unicycle / reduced car
 * through Choset's z = A(x)x, v = B(x)u, and the 4-state car (x, y, φ, θ)
 * through Murray & Sastry's coordinates z = (x, tan φ /(L cos³θ), tan θ, y),
 * written in a chart rotated so both end headings lie inside (−π/2, π/2).
 */

import { normalizeAngle, type Pose2 } from '../geom/se2';

/* -------------------------------------------------------------------------- */
/* Brockett's system (12.23) and its optimal sinusoids                          */
/* -------------------------------------------------------------------------- */

export interface BrockettSolution {
  /** λ₃ = 2kπ with k = ±1 chosen by the sign of the fiber goal. */
  k: number;
  u10: number;
  u20: number;
  /** J = ½(u₁(0)² + u₂(0)²). */
  cost: number;
}

/**
 * Pure fiber motion x = (0, 0, 0) → (0, 0, x₃,goal) with t_f = 1: λ₃ = 2kπ makes
 * ∫u₁ = ∫u₂ = 0, and x₃(1) = ∓(u₁(0)² + u₂(0)²)/4π for k = ±1. Any (u₁(0), u₂(0))
 * on the circle of radius √(4π|x₃|) is optimal; we return the one on the u₁-axis.
 */
export function steerBrockett(x3Goal: number): BrockettSolution {
  const r = Math.sqrt(4 * Math.PI * Math.abs(x3Goal));
  return { k: x3Goal >= 0 ? -1 : 1, u10: r, u20: 0, cost: 0.5 * r * r };
}

/** u(t) from (12.26)–(12.27) with λ₃ = 2kπ. */
export function brockettControl(sol: BrockettSolution, t: number): [number, number] {
  const l = 2 * Math.PI * sol.k;
  return [sol.u10 * Math.cos(l * t) - sol.u20 * Math.sin(l * t), sol.u10 * Math.sin(l * t) + sol.u20 * Math.cos(l * t)];
}

/** Integrate (12.23) from the origin under a control law; returns the trace. */
export function integrateBrockett(u: (t: number) => [number, number], steps = 4000): number[][] {
  const f = (t: number, x: number[]) => {
    const [u1, u2] = u(t);
    return [u1, u2, x[1] * u1];
  };
  return rk4Trace(f, [0, 0, 0], 0, 1, steps);
}

/* -------------------------------------------------------------------------- */
/* The generic chained form                                                    */
/* -------------------------------------------------------------------------- */

/** ẋ = u₁g₁(x) + u₂g₂(x) in chained form. */
export function chainedRhs(x: readonly number[], u1: number, u2: number): number[] {
  const n = x.length;
  const out = new Array<number>(n).fill(0);
  out[0] = u1;
  out[1] = u2;
  for (let i = 2; i < n; i++) out[i] = x[i - 1] * u1;
  return out;
}

/** g₁ and g₂ as columns, for the bracket ladder check against Chapter 20's `lieBracket`. */
export function chainedFields(n: number): { g1: (x: number[]) => number[]; g2: (x: number[]) => number[] } {
  return {
    g1: (x) => chainedRhs(x, 1, 0),
    g2: (x) => chainedRhs(x, 0, 1),
  };
}

export function rk4Trace(f: (t: number, x: number[]) => number[], x0: number[], t0: number, t1: number, steps: number): number[][] {
  const h = (t1 - t0) / steps;
  let x = x0.slice();
  const out = [x.slice()];
  for (let i = 0; i < steps; i++) {
    const t = t0 + i * h;
    const k1 = f(t, x);
    const k2 = f(t + h / 2, x.map((xi, j) => xi + (h / 2) * k1[j]));
    const k3 = f(t + h / 2, x.map((xi, j) => xi + (h / 2) * k2[j]));
    const k4 = f(t + h, x.map((xi, j) => xi + h * k3[j]));
    x = x.map((xi, j) => xi + (h / 6) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]));
    out.push(x.slice());
  }
  return out;
}

export interface SinusoidPhase {
  /** 0 = drive x₁, x₂ directly; k ≥ 1 = the k-th sinusoid cycle aimed at x_{k+2}. */
  k: number;
  a: number;
  b: number;
  /** Input functions on [0, 1]. */
  u: (t: number) => [number, number];
  /** The chained state at the dense steps (phase-local, including both ends). */
  trace: number[][];
  /** The value x_{k+2} was asked to move by (eq. 12.31), and by how much it actually moved. */
  target: number;
  achieved: number;
}

export interface SinusoidOptions {
  /** Cap on |a| — the amplitude of the u₁ sinusoid (the car's forward-backward swing). */
  ampCap?: number;
  stepsPerPhase?: number;
}

/**
 * Murray–Sastry steering (Choset's two-step recipe). Amplitudes: |a| = |b| =
 * (|Δ| k! (4π)ᵏ)^{1/(k+1)} balances the two inputs; if that exceeds the cap, a is
 * clamped and b grows to compensate — the same Δ, a different shape.
 */
export function steerSinusoids(xStart: readonly number[], xGoal: readonly number[], opts: SinusoidOptions = {}): { phases: SinusoidPhase[]; final: number[] } {
  const n = xStart.length;
  const steps = opts.stepsPerPhase ?? 400;
  const cap = opts.ampCap ?? Infinity;
  const phases: SinusoidPhase[] = [];
  let x = xStart.slice();
  // Phase 0: x₁, x₂ to their goals with constant inputs over unit time.
  {
    const d1 = xGoal[0] - x[0];
    const d2 = xGoal[1] - x[1];
    const u = (): [number, number] => [d1, d2];
    const trace = rk4Trace((_t, s) => chainedRhs(s, d1, d2), x, 0, 1, steps);
    phases.push({ k: 0, a: d1, b: d2, u, trace, target: d1, achieved: trace[trace.length - 1][0] - x[0] });
    x = trace[trace.length - 1];
  }
  for (let k = 1; k <= n - 2; k++) {
    const delta = xGoal[k + 1] - x[k + 1];
    let fact = 1;
    for (let i = 2; i <= k; i++) fact *= i;
    let a = Math.abs(delta) < 1e-15 ? 0 : Math.pow(Math.abs(delta) * fact * Math.pow(4 * Math.PI, k), 1 / (k + 1));
    let b: number;
    if (a === 0) b = 0;
    else {
      if (a > cap) a = cap;
      b = (delta * fact * Math.pow(4 * Math.PI / a, k));
    }
    const kk = k;
    const aa = a;
    const bb = b;
    const u = (t: number): [number, number] => [aa * Math.sin(2 * Math.PI * t), bb * Math.cos(2 * kk * Math.PI * t)];
    const trace = rk4Trace((t, s) => {
      const [u1, u2] = u(t);
      return chainedRhs(s, u1, u2);
    }, x, 0, 1, steps);
    const end = trace[trace.length - 1];
    phases.push({ k, a, b, u, trace, target: delta, achieved: end[k + 1] - x[k + 1] });
    x = end;
  }
  return { phases, final: x };
}

/** (a/4π)ᵏ b / k! — eq. (12.31). */
export function sinusoidGain(a: number, b: number, k: number): number {
  let fact = 1;
  for (let i = 2; i <= k; i++) fact *= i;
  return (Math.pow(a / (4 * Math.PI), k) * b) / fact;
}

/* -------------------------------------------------------------------------- */
/* Unicycle ↔ chained form (Choset's A(x), B(x))                                */
/* -------------------------------------------------------------------------- */

/** z = A(x)x: z₁ = x₃, z₂ = x₁cos x₃ + x₂ sin x₃, z₃ = x₁ sin x₃ − x₂ cos x₃. */
export function unicycleToChained(q: Pose2): number[] {
  const c = Math.cos(q.theta);
  const s = Math.sin(q.theta);
  return [q.theta, q.x * c + q.y * s, q.x * s - q.y * c];
}

export function chainedToUnicycle(z: readonly number[]): Pose2 {
  const c = Math.cos(z[0]);
  const s = Math.sin(z[0]);
  return { x: z[1] * c + z[2] * s, y: z[1] * s - z[2] * c, theta: z[0] };
}

/** v = B(x)u: v₁ = ω, v₂ = v + (x₂cos x₃ − x₁ sin x₃)ω; inverted: v = v₂ + z₃v₁, ω = v₁. */
export function unicycleInputs(z: readonly number[], v1: number, v2: number): { v: number; omega: number } {
  return { v: v2 + z[2] * v1, omega: v1 };
}

/* -------------------------------------------------------------------------- */
/* The 4-state car in chained form                                              */
/* -------------------------------------------------------------------------- */

export interface CarChart {
  /** Chart rotation β: z₁ is the position along (cos β, sin β). */
  beta: number;
  L: number;
}

/** (x, y, φ, θ) → z = (x′, tan φ/(L cos³θ′), tan θ′, y′) with primes in the chart rotated by β. */
export function carToChained(q: Pose2, phi: number, chart: CarChart): number[] {
  const cb = Math.cos(chart.beta);
  const sb = Math.sin(chart.beta);
  const xp = cb * q.x + sb * q.y;
  const yp = -sb * q.x + cb * q.y;
  const th = normalizeAngle(q.theta - chart.beta);
  return [xp, Math.tan(phi) / (chart.L * Math.cos(th) ** 3), Math.tan(th), yp];
}

export function chainedToCar(z: readonly number[], chart: CarChart): { pose: Pose2; phi: number } {
  const th = Math.atan(z[2]);
  const phi = Math.atan(z[1] * chart.L * Math.cos(th) ** 3);
  const cb = Math.cos(chart.beta);
  const sb = Math.sin(chart.beta);
  return { pose: { x: cb * z[0] - sb * z[3], y: sb * z[0] + cb * z[3], theta: normalizeAngle(th + chart.beta) }, phi };
}

export interface CarTraceSample {
  q: Pose2;
  phi: number;
  /** Signed arc length from the previous sample (v₁/cos θ′ · dt). */
  ds: number;
}

export interface CarSinusoidPlan {
  chart: CarChart;
  phases: SinusoidPhase[];
  samples: CarTraceSample[];
  /** Chained-coordinate error at the end (should be ~1e-9). */
  chainedError: number;
  /** World-pose error at the end. */
  poseError: number;
  length: number;
  cusps: number;
  maxSteer: number;
}

/**
 * Steer Hitch's 4-state car between two poses with straight wheels at both
 * ends, by sinusoids in chained coordinates. The chart axis is the bisector of
 * the two headings, so |θ′| < π/2 at both ends whenever the headings differ by
 * less than π; inside the motion θ′ = atan z₃ never reaches ±π/2 by
 * construction. What it does not respect: the steering limit (reported in
 * `maxSteer`) and obstacles.
 */
export function steerCarSinusoids(q0: Pose2, q1: Pose2, L: number, opts: SinusoidOptions = {}): CarSinusoidPlan {
  const beta = normalizeAngle(q0.theta + 0.5 * normalizeAngle(q1.theta - q0.theta));
  const chart: CarChart = { beta, L };
  const z0 = carToChained(q0, 0, chart);
  const z1 = carToChained(q1, 0, chart);
  const { phases, final } = steerSinusoids(z0, z1, opts);
  const samples: CarTraceSample[] = [];
  let length = 0;
  let cusps = 0;
  let prevSign = 0;
  let maxSteer = 0;
  for (const ph of phases) {
    const startIdx = samples.length === 0 ? 0 : 1;
    for (let i = startIdx; i < ph.trace.length; i++) {
      const { pose, phi } = chainedToCar(ph.trace[i], chart);
      let ds = 0;
      if (samples.length > 0) {
        const prev = samples[samples.length - 1].q;
        const step = Math.hypot(pose.x - prev.x, pose.y - prev.y);
        // Sign: forward if the displacement points along the heading.
        const dir = (pose.x - prev.x) * Math.cos(prev.theta) + (pose.y - prev.y) * Math.sin(prev.theta);
        ds = dir >= 0 ? step : -step;
        if (step > 1e-9) {
          const sg = Math.sign(ds);
          if (prevSign !== 0 && sg !== prevSign) cusps++;
          prevSign = sg;
        }
        length += step;
      }
      maxSteer = Math.max(maxSteer, Math.abs(phi));
      samples.push({ q: pose, phi, ds });
    }
  }
  const end = samples[samples.length - 1].q;
  return {
    chart,
    phases,
    samples,
    chainedError: Math.max(...final.map((v, i) => Math.abs(v - z1[i]))),
    poseError: Math.max(Math.abs(end.x - q1.x), Math.abs(end.y - q1.y), Math.abs(normalizeAngle(end.theta - q1.theta))),
    length,
    cusps,
    maxSteer,
  };
}
