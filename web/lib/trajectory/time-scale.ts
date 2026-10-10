/**
 * The Time-Scaling Algorithm (Choset §11.2, steps 1–5) with the zero-inertia
 * modification of §11.2.1 — Bobrow, Dubowsky & Gibson (1985) and Shin & McKay
 * (1985), as Choset presents them.
 *
 * The phase plane is integrated in x = ṡ² rather than ṡ: dx/ds = 2 s̈(s, √x) is
 * finite at ṡ = 0 (where dṡ/ds = s̈/ṡ blows up), and for a constant bound the
 * integral curve is a straight line, so the micro-example's switch lands on the
 * grid exactly. Every curve lives on the uniform grid s_k = k/N; crossings
 * between nodes are located by linear interpolation. Choset's "integrate until
 * … is penetrated" becomes "step until x_{k+1} > x_max(s_{k+1})" where the
 * limit at that node is of cone type (L = U). Where the limit is set by a
 * zero-inertia row or a joint-rate bound (L < U there — a *critical* point), the
 * step is clipped to the curve instead: that is s̈_max = min(s̈⁺_tangent, U) of
 * §11.2.1, and it is what lets the trajectory slide along a singular arc.
 *
 * Step 4 follows Choset's footnote 3: from the vertical line through s_lim,
 * bisect on ṡ′ for the highest min-acceleration curve that stays admissible,
 * and take its graze point as the tangent point. This is more robust than the
 * slope test dv/ds = U/v — it never selects a tangent point whose own backward
 * deceleration curve is doomed — and it makes the construction agree with
 * TOPP-RA's reachability sets to the grid's resolution, which the checks
 * assert on seeded gravity-loaded splines.
 *
 * Honesty items the prose must keep (and this code assumes): no inadmissible
 * islands (Choset footnote 2); the tangent point is a grid node; and where the
 * robot cannot hold a configuration statically the solver reports Choset's
 * Problems 11.3/11.4 (`StallsAtZero`, `NoSolution`) instead of guessing.
 */

import type { Vec } from '../prob/linalg';
import { rk4Step } from '../dynamics/integrate';
import type { Abc, PathDynamics } from './constrained';
import { accelBounds, limitPoint, type JointVelocityLimits, type LimitPoint, type TorqueLimits } from './limits';
import type { Trajectory, TrajectorySample } from './trajectory';

export type PhaseMode = 'max' | 'min' | 'slide';

export interface PhaseCurve {
  /** F: the backward min-acceleration curve; A: a forward/backward piece A_i of the construction. */
  kind: 'F' | 'A';
  mode: PhaseMode;
  /** Node indices [from, to] on which the curve is defined, and x = ṡ² there. */
  from: number;
  to: number;
  x: number[];
  /** Fractional point where this curve left the admissible region (drawn as the amber hit), if any. */
  hit?: { s: number; sdot: number };
}

export interface SwitchPoint {
  s: number;
  sdot: number;
  kind: 'max→min' | 'min→max';
  /** True when the switch is a tangent (or critical) point on the limit curve. */
  onLimit: boolean;
}

export interface TimeScaling {
  /** Choset's list 𝒮: s values of the switches, in order. */
  switches: number[];
  switchPoints: SwitchPoint[];
  /** The optimal phase-plane curve on the grid. */
  s: number[];
  sdot: number[];
  /** s̈ at each node: U or L at (s_k, ṡ_k) in max/min mode (an actuator is saturated there), the stage value on a slide. */
  sddot: number[];
  /** Stage accelerations (x_{k+1} − x_k)/(2Δ), k < N: x(s) is piecewise linear between nodes, so this is the s̈ a replay must apply. */
  stage: number[];
  mode: PhaseMode[];
  /** Time at each grid node; t[N] = t_f. */
  t: number[];
  tf: number;
  /** ṡ_max(s) with provenance, on the same grid. */
  limit: LimitPoint[];
  /** Construction curves (F, A_0, A_1, …) for drawing. */
  curves: PhaseCurve[];
  N: number;
}

export type TimeScaleError =
  | { kind: 'StallsAtZero'; s: number; detail: string }
  | { kind: 'NoSolution'; s: number; detail: string }
  | { kind: 'LimitIsland'; s: number; detail: string };

export type TimeScaleResult = { ok: true; scaling: TimeScaling } | { ok: false; error: TimeScaleError };

export interface TimeScaleOptions {
  /** Grid resolution: s_k = k/N. */
  N?: number;
  sdot0?: number;
  sdotF?: number;
  /** Joint-rate limits |q̇_i| ≤ v_i, as extra rows of the limit curve. */
  rateLimits?: JointVelocityLimits | null;
  maxSwitches?: number;
  /** Trace hook: one line per construction event (for debugging and the widget's log). */
  trace?: (msg: string) => void;
}

interface Tables {
  N: number;
  ds: number;
  /** abc at the half grid: fine[j] is s = j/(2N). */
  fine: Abc[];
  /** ṡ_max² at every half-grid node, for clamping RK4 stage states. */
  xmaxFine: number[];
  limit: LimitPoint[];
  xmax: number[];
  critical: boolean[];
}

function tabulate(pd: PathDynamics, lim: TorqueLimits, N: number, rate: JointVelocityLimits | null): Tables {
  const fine: Abc[] = [];
  const xmaxFine: number[] = [];
  const limit: LimitPoint[] = [];
  const xmax: number[] = [];
  const critical: boolean[] = [];
  for (let j = 0; j <= 2 * N; j++) {
    const s = j / (2 * N);
    const abc = pd.abc(s);
    fine.push(abc);
    const lp = limitPoint(s, abc, pd.path.dq(s), lim, rate);
    const xm = Number.isFinite(lp.sdotMax) ? lp.sdotMax * lp.sdotMax : Infinity;
    xmaxFine.push(xm);
    if (j % 2 === 0) {
      limit.push(lp);
      xmax.push(xm);
      critical.push(lp.kind === 'zip' || lp.kind === 'rate');
    }
  }
  return { N, ds: 1 / N, fine, xmaxFine, limit, xmax, critical };
}

/**
 * dx/ds at a half-grid node j for the given mode. The state is clamped to the
 * admissible region first: an RK4 stage that lands a hair above the limit
 * curve next to a zero-inertia point would otherwise read a bound n_i/a_i with
 * a_i ≈ 0 — an honest "inadmissible" signal, but one that drives the step to
 * ṡ² < 0. On the curve itself L = U and both are finite.
 */
function slopeAt(T: Tables, lim: TorqueLimits, j: number, x: number, mode: 'max' | 'min'): number {
  const xc = Math.min(Math.max(x, 0), T.xmaxFine[j]);
  const { L, U } = accelBounds(T.fine[j], lim, Math.sqrt(xc));
  return 2 * (mode === 'max' ? U : L);
}

/**
 * One RK4 step of dx/ds from node k to node k ± 1. The right-hand side is
 * evaluated at the three fine nodes the step spans (k, k ± ½, k ± 1).
 */
function stepX(T: Tables, lim: TorqueLimits, k: number, x: number, mode: 'max' | 'min', dir: 1 | -1): number {
  const h = dir * T.ds;
  const j0 = 2 * k;
  const jm = 2 * k + dir;
  const j1 = 2 * k + 2 * dir;
  // RK4 with a time-varying (s-varying) right-hand side; `rk4Step` takes an
  // autonomous f, so the stage index is threaded through a closure counter.
  let stage = 0;
  const f = (v: Vec): Vec => {
    const j = stage === 0 ? j0 : stage === 3 ? j1 : jm;
    stage++;
    return [slopeAt(T, lim, j, v[0], mode)];
  };
  return rk4Step(f, [x], h)[0];
}

const REL_TOL = 1e-7;

function above(x: number, xmax: number): boolean {
  if (!Number.isFinite(xmax)) return false;
  return x > xmax + REL_TOL * Math.max(1, xmax);
}

/** Slope of the limit curve in x at node k (central difference, one-sided at the ends). */
function limitSlope(T: Tables, k: number): number {
  const N = T.N;
  const xm = T.xmax;
  if (!Number.isFinite(xm[k])) return Infinity;
  if (k === 0) return Number.isFinite(xm[1]) ? (xm[1] - xm[0]) / T.ds : Infinity;
  if (k === N) return Number.isFinite(xm[N - 1]) ? (xm[N] - xm[N - 1]) / T.ds : -Infinity;
  const lo = Number.isFinite(xm[k - 1]) ? xm[k - 1] : xm[k];
  const hi = Number.isFinite(xm[k + 1]) ? xm[k + 1] : xm[k];
  return (hi - lo) / (2 * T.ds);
}

interface StepOut {
  x: number;
  /** The free integral curve would have left the admissible region, but an acceleration inside the cone lands the stage exactly on the limit curve instead. */
  slid: boolean;
  /** Even that is impossible: the stage penetrates the limit curve. `raw` is the unclipped value, for interpolating the hit. */
  penetrated: boolean;
  raw: number;
}

/**
 * One grid stage of the max- or min-acceleration curve, with the §11.2.1 rule.
 *
 * When the free integral curve would cross the limit curve between k and k±1,
 * ask whether the stage can land *on* the curve instead: the constant stage
 * acceleration that does so is u* = ±(x_max − x)/(2Δ), and it is feasible when
 * it lies inside the motion cone (checked at both ends of the stage). At a
 * critical point (L < U on the curve) this is Choset's s̈_max = min(s̈⁺_tangent, U)
 * / s̈_min = max(s̈⁻_tangent, L) — sliding along a singular arc. At an ordinary
 * point of v(s) the cone has collapsed, so the test only passes when the
 * curve is tangent to the integral curve up to the grid's resolution. When it
 * fails, the stage genuinely penetrates: that is Choset's (s_lim, ṡ_lim).
 */
function stepClipped(T: Tables, lim: TorqueLimits, k: number, x: number, mode: 'max' | 'min', dir: 1 | -1): StepOut {
  const raw = stepX(T, lim, k, x, mode, dir);
  const kn = k + dir;
  const xm = T.xmax[kn];
  if (!above(raw, xm)) return { x: raw, slid: false, penetrated: false, raw };
  const uStar = (dir * (xm - x)) / (2 * T.ds);
  const here = accelBounds(T.fine[2 * k], lim, Math.sqrt(Math.max(0, x)));
  const there = accelBounds(T.fine[2 * kn], lim, Math.sqrt(Math.max(0, xm)));
  const lo = Math.min(here.L, there.L);
  const hi = Math.max(here.U, there.U);
  const tol = 1e-9 * Math.max(1, Math.abs(lo), Math.abs(hi));
  if (uStar >= lo - tol && uStar <= hi + tol) return { x: xm, slid: true, penetrated: false, raw };
  return { x: raw, slid: false, penetrated: true, raw };
}

/** Fractional node where a curve going from x0 (admissible at k) to raw (above the curve at k ± 1) crosses the limit curve. */
function hitPoint(T: Tables, k: number, x0: number, raw: number, dir: 1 | -1): { s: number; sdot: number } {
  const kn = k + dir;
  const over = raw - T.xmax[kn];
  const under = Math.max(0, T.xmax[k] - x0);
  const frac = under + over > 0 ? under / (under + over) : 0;
  return { s: (k + dir * frac) / T.N, sdot: Math.sqrt(Math.max(0, x0 + frac * (raw - x0))) };
}

interface LCurve {
  /** x at nodes k0 … end (inclusive); NaN elsewhere. */
  x: number[];
  slid: boolean[];
  end: number;
  penetrated: boolean;
  crossedF: boolean;
  stalled: boolean;
}

/** The min-acceleration curve forward from (k0, x0): the object the footnote-3 bisection searches over. */
function integrateLForward(T: Tables, lim: TorqueLimits, k0: number, x0: number, F: readonly number[]): LCurve {
  const N = T.N;
  const x = new Array<number>(N + 1).fill(NaN);
  const slid = new Array<boolean>(N + 1).fill(false);
  x[k0] = x0;
  let end = k0;
  let penetrated = false;
  let crossedF = false;
  let stalled = false;
  for (let k = k0; k < N; k++) {
    const st = stepClipped(T, lim, k, x[k], 'min', 1);
    if (!Number.isNaN(F[k + 1]) && st.raw >= F[k + 1]) {
      crossedF = true;
      break;
    }
    if (st.penetrated) {
      penetrated = true;
      break;
    }
    if (st.x <= 0) {
      stalled = true;
      break;
    }
    x[k + 1] = st.x;
    slid[k + 1] = st.slid;
    end = k + 1;
  }
  return { x, slid, end, penetrated, crossedF, stalled };
}

/**
 * Steps 1–5 of Choset §11.2 with the §11.2.1 modification. Returns the switch
 * list, the optimal (s, ṡ) profile, t_f, the limit curve and the construction
 * curves — everything w18.1 draws.
 *
 * Step 4 uses Choset's footnote 3 rather than a slope test: from the vertical
 * line through s_lim, bisect on ṡ′ for the highest min-acceleration curve that
 * does not penetrate the limit curve; where that curve grazes the curve is the
 * tangent point. The bisection is robust where the slope test is not — it
 * never picks a tangent point whose own deceleration curve is doomed — and it
 * costs O(N log(1/tol)) stages per switch.
 */
export function timeScale(pd: PathDynamics, lim: TorqueLimits, opts: TimeScaleOptions = {}): TimeScaleResult {
  const N = opts.N ?? 400;
  const sdot0 = opts.sdot0 ?? 0;
  const sdotF = opts.sdotF ?? 0;
  const maxSwitches = opts.maxSwitches ?? 200;
  const T = tabulate(pd, lim, N, opts.rateLimits ?? null);
  const ds = T.ds;
  const trace = opts.trace ?? (() => undefined);

  const fail = (kind: TimeScaleError['kind'], k: number, detail: string): TimeScaleResult => ({
    ok: false,
    error: { kind, s: k / N, detail },
  });

  // ---- Step 1: the boundary states must be admissible. --------------------
  if (above(sdot0 * sdot0, T.xmax[0])) return fail('NoSolution', 0, `ṡ₀ = ${sdot0} exceeds the limit curve at s = 0`);
  if (above(sdotF * sdotF, T.xmax[N])) return fail('NoSolution', N, `ṡ_f = ${sdotF} exceeds the limit curve at s = 1`);
  for (let k = 0; k <= N; k++) {
    if (T.xmax[k] <= 0) return fail('NoSolution', k, `the path is inadmissible even at rest at s = ${(k / N).toFixed(3)} (the robot cannot hold this configuration)`);
  }

  const curves: PhaseCurve[] = [];
  const profile = new Array<number>(N + 1).fill(NaN);
  const mode = new Array<PhaseMode>(N + 1).fill('max');

  // ---- Step 2: F — integrate s̈ = L backward from (1, ṡ_f). ----------------
  const F = new Array<number>(N + 1).fill(NaN);
  const Fslid = new Array<boolean>(N + 1).fill(false);
  F[N] = sdotF * sdotF;
  let kF = N; // first node where F is defined
  let fHit: PhaseCurve['hit'];
  for (let k = N; k > 0; k--) {
    const st = stepClipped(T, lim, k, F[k], 'min', -1);
    if (st.penetrated) {
      fHit = hitPoint(T, k, F[k], st.raw, -1);
      break;
    }
    if (st.x <= 0) {
      return fail(
        'StallsAtZero',
        k - 1,
        `the minimum-acceleration curve F reaches ṡ = 0 at s ≈ ${((k - 1) / N).toFixed(3)} < 1: even full braking cannot bring the arm to rest at s = 1 (Choset Problem 11.3)`,
      );
    }
    F[k - 1] = st.x;
    Fslid[k - 1] = st.slid;
    kF = k - 1;
  }
  curves.push({ kind: 'F', mode: 'min', from: kF, to: N, x: F.slice(kF, N + 1), hit: fHit });

  // ---- Steps 3–5 ------------------------------------------------------------
  const switchPoints: SwitchPoint[] = [];
  let ki = 0;
  let xi = sdot0 * sdot0;
  profile[0] = xi;
  let solved = false;

  for (let iter = 0; iter <= N + 2 && !solved; iter++) {
    if (switchPoints.length > maxSwitches) {
      return fail('LimitIsland', ki, `more than ${maxSwitches} switches — the admissible region is probably not a single band below one limit curve (Choset footnote 2)`);
    }

    // Step 3: A_i forward with s̈ = U from (k_i, x_i).
    const A = new Array<number>(N + 1).fill(NaN);
    const Aslid = new Array<boolean>(N + 1).fill(false);
    A[ki] = xi;
    let kLim = -1;
    let aHit: PhaseCurve['hit'];
    let aRawNext = NaN;
    let crossNode = -1;
    let crossFrac = 0;
    for (let k = ki; k < N; k++) {
      const st = stepClipped(T, lim, k, A[k], 'max', 1);
      // Crossing F is tested on the free curve first: a stage that leaves the
      // admissible region above F has crossed F on the way.
      if (!Number.isNaN(F[k + 1]) && st.raw >= F[k + 1]) {
        const dPrev = !Number.isNaN(F[k]) ? F[k] - A[k] : Math.max(0, T.xmax[k] - A[k]);
        const dNext = st.raw - F[k + 1];
        crossFrac = dPrev + dNext > 0 ? dPrev / (dPrev + dNext) : 1;
        crossNode = k;
        A[k + 1] = st.raw;
        break;
      }
      if (st.penetrated) {
        kLim = k;
        aHit = hitPoint(T, k, A[k], st.raw, 1);
        aRawNext = st.raw;
        break;
      }
      if (st.x <= 0) return fail('NoSolution', k + 1, `the maximum-acceleration curve reaches ṡ = 0 at s ≈ ${((k + 1) / N).toFixed(3)}: gravity or the velocity-product term drives the arm backward along the path faster than the motors can push it forward (Choset Problem 11.4)`);
      A[k + 1] = st.x;
      Aslid[k + 1] = st.slid;
    }

    if (crossNode >= 0) {
      // Solved: A_i up to the crossing, F after it.
      const sSwitch = (crossNode + crossFrac) / N;
      const xSwitch = A[crossNode] + crossFrac * (A[crossNode + 1] - A[crossNode]);
      curves.push({ kind: 'A', mode: 'max', from: ki, to: crossNode + 1, x: A.slice(ki, crossNode + 2) });
      for (let m = ki; m <= crossNode; m++) {
        profile[m] = A[m];
        mode[m] = Aslid[m] ? 'slide' : 'max';
      }
      for (let m = crossNode + 1; m <= N; m++) {
        profile[m] = F[m];
        mode[m] = Fslid[m] ? 'slide' : 'min';
      }
      if (sSwitch > 0 && sSwitch < 1 && !(crossNode === ki && crossFrac === 0)) {
        switchPoints.push({ s: sSwitch, sdot: Math.sqrt(Math.max(0, xSwitch)), kind: 'max→min', onLimit: false });
      }
      solved = true;
      break;
    }

    if (kLim < 0) {
      // Reached s = 1 without crossing F: A_N < ṡ_f² — the final speed is out of reach.
      return fail('NoSolution', N, `the maximum-acceleration curve reaches s = 1 at ṡ = ${Math.sqrt(Math.max(0, A[N])).toFixed(3)} below ṡ_f = ${sdotF}`);
    }
    curves.push({ kind: 'A', mode: 'max', from: ki, to: kLim, x: A.slice(ki, kLim + 1), hit: aHit });
    trace(`A_${curves.length - 2}: from s = ${(ki / N).toFixed(4)} (ṡ = ${Math.sqrt(xi).toFixed(4)}) penetrates at s_lim = ${aHit?.s.toFixed(4)} (ṡ_lim = ${aHit?.sdot.toFixed(4)})`);

    // Step 4 (footnote 3): on the line s = s_lim, bisect for the highest
    // min-acceleration curve that stays admissible. Its graze point is the
    // tangent (or critical) point (s_tan, ṡ_tan).
    const pen = (xp: number) => integrateLForward(T, lim, kLim, xp, F);
    let lo = 0;
    let hi = A[kLim];
    let Lc = pen(hi);
    if (Lc.penetrated) {
      for (let it = 0; it < 48; it++) {
        const mid = 0.5 * (lo + hi);
        const trial = pen(mid);
        if (trial.penetrated) hi = mid;
        else {
          lo = mid;
          Lc = trial;
        }
      }
      if (Lc.penetrated) Lc = pen(lo);
      if (Lc.penetrated) return fail('NoSolution', kLim, `no deceleration curve from s = ${(kLim / N).toFixed(3)} stays admissible — the path is inadmissible at rest beyond it`);
    }
    const xPrime = Lc.x[kLim];
    // A braking curve that reaches ṡ = 0 is not an error: it is the low side of
    // the bisection, and its graze point (if any) lies before the stall.
    if (Lc.end <= kLim) {
      return fail('NoSolution', kLim, `the deceleration curve from s_lim = ${aHit?.s.toFixed(3)} cannot advance at all — every state on A_i is doomed (Choset figure 11.2)`);
    }

    if (Lc.crossedF) {
      // A_i → L-curve → F, with the L-curve merging into F (two min-acceleration curves cannot cross).
      const meet = meetBackward(T, lim, A, ki, kLim, xPrime, aRawNext);
      if (!meet.ok) return fail('NoSolution', meet.node, meet.detail);
      writeSegment(profile, mode, A, Aslid, ki, meet.node, 'max');
      writeSegment(profile, mode, meet.x, meet.slid, meet.node + 1, kLim, 'min');
      writeSegment(profile, mode, Lc.x, Lc.slid, kLim + 1, Lc.end, 'min');
      for (let m = Lc.end + 1; m <= N; m++) {
        profile[m] = F[m];
        mode[m] = Fslid[m] ? 'slide' : 'min';
      }
      curves.push({ kind: 'A', mode: 'min', from: meet.node, to: Lc.end, x: [...meet.x.slice(meet.node, kLim), ...Lc.x.slice(kLim, Lc.end + 1)] });
      pushMaxToMin(switchPoints, meet, N, ki);
      solved = true;
      break;
    }

    // Tangent point: the node where the L-curve comes closest to the limit curve.
    let kTan = -1;
    let gap = Infinity;
    for (let k = kLim + 1; k <= Lc.end; k++) {
      const g = T.xmax[k] - Lc.x[k];
      if (g < gap - 1e-12 * Math.max(1, Math.abs(g))) {
        gap = g;
        kTan = k;
      }
    }
    if (kTan < 0) return fail('NoSolution', kLim, 'the deceleration curve from s_lim has no admissible continuation');
    trace(`  bisection: ṡ′ = ${Math.sqrt(xPrime).toFixed(4)} at s_lim node ${(kLim / N).toFixed(4)}; L-curve ends at ${(Lc.end / N).toFixed(4)} (${Lc.penetrated ? 'penetrates' : Lc.stalled ? 'stalls' : 'runs out'}); tangent at s = ${(kTan / N).toFixed(4)}, ṡ = ${Math.sqrt(Lc.x[kTan]).toFixed(4)}, gap ${gap.toExponential(2)} (limit ${Math.sqrt(T.xmax[kTan]).toFixed(4)} ${T.limit[kTan].kind})`);

    // Back from (s_lim, ṡ′) with s̈ = L until the curve rises above A_i: that is the max→min switch.
    const meet = meetBackward(T, lim, A, ki, kLim, xPrime, aRawNext);
    if (!meet.ok) return fail('NoSolution', meet.node, meet.detail);
    writeSegment(profile, mode, A, Aslid, ki, meet.node, 'max');
    writeSegment(profile, mode, meet.x, meet.slid, meet.node + 1, kLim, 'min');
    writeSegment(profile, mode, Lc.x, Lc.slid, kLim + 1, kTan, 'min');
    curves.push({ kind: 'A', mode: 'min', from: meet.node, to: kTan, x: [...meet.x.slice(meet.node, kLim), ...Lc.x.slice(kLim, kTan + 1)] });
    pushMaxToMin(switchPoints, meet, N, ki);
    // Step 5: the tangent point is a min→max switch; continue from it.
    switchPoints.push({ s: kTan / N, sdot: Math.sqrt(Math.max(0, Lc.x[kTan])), kind: 'min→max', onLimit: true });
    ki = kTan;
    xi = Lc.x[kTan];
    profile[ki] = xi;
    if (ki >= N) {
      solved = Math.abs(xi - F[N]) <= REL_TOL * Math.max(1, F[N]);
      if (!solved) return fail('NoSolution', N, 'the tangent point is the final node but not at ṡ_f');
    }
  }
  if (!solved) return fail('LimitIsland', ki, 'the switching construction did not terminate');

  // Degenerate pairs: a min→max switch immediately followed by a max→min at the
  // same node is no switch at all (the curve slid along the limit).
  const cleaned: SwitchPoint[] = [];
  for (const sw of switchPoints) {
    const last = cleaned[cleaned.length - 1];
    if (last && Math.abs(last.s - sw.s) < 0.5 / N && last.kind !== sw.kind) cleaned.pop();
    else cleaned.push(sw);
  }

  // ---- Assemble the profile ----------------------------------------------------
  const s: number[] = [];
  const sdot: number[] = [];
  const sddot: number[] = [];
  const stage: number[] = [];
  const t: number[] = [0];
  for (let k = 0; k <= N; k++) {
    s.push(k / N);
    const x = Number.isNaN(profile[k]) ? 0 : Math.max(0, profile[k]);
    sdot.push(Math.sqrt(x));
  }
  for (let k = 0; k < N; k++) stage.push((sdot[k + 1] * sdot[k + 1] - sdot[k] * sdot[k]) / (2 * ds));
  for (let k = 0; k <= N; k++) {
    if (mode[k] === 'slide') {
      sddot.push(k < N ? stage[k] : stage[N - 1]);
    } else {
      const { L, U } = accelBounds(T.fine[2 * k], lim, sdot[k]);
      sddot.push(mode[k] === 'max' ? U : L);
    }
  }
  for (let k = 0; k < N; k++) {
    const denom = sdot[k] + sdot[k + 1];
    t.push(t[k] + (denom > 0 ? (2 * ds) / denom : 0));
  }

  return {
    ok: true,
    scaling: {
      switches: cleaned.map((sw) => sw.s),
      switchPoints: cleaned,
      s,
      sdot,
      sddot,
      stage,
      mode,
      t,
      tf: t[N],
      limit: T.limit,
      curves,
      N,
    },
  };
}

interface Meet {
  ok: boolean;
  /** Last node still on A_i; the switch lies in (node, node + 1]. */
  node: number;
  frac: number;
  /** The backward L-curve, defined on [node + 1, kLim]. */
  x: number[];
  slid: boolean[];
  detail: string;
}

/**
 * From (k_lim, x′) integrate s̈ = L backward until the curve rises above A_i.
 * A_i falls going backward (U > 0 along it) and the braking curve rises, so
 * they cross exactly once; if the braking curve is still below A_i at k_i, the
 * start state lies above every admissible deceleration curve and is doomed.
 */
function meetBackward(T: Tables, lim: TorqueLimits, A: readonly number[], ki: number, kLim: number, xPrime: number, aRawNext: number): Meet {
  const x = new Array<number>(T.N + 1).fill(NaN);
  const slid = new Array<boolean>(T.N + 1).fill(false);
  x[kLim] = xPrime;
  if (xPrime >= A[kLim] - 1e-12 * Math.max(1, A[kLim])) {
    // Braking from s_lim itself avoids the penetration: the switch is at k_lim.
    x[kLim] = A[kLim];
    return { ok: true, node: kLim, frac: 0, x, slid, detail: '' };
  }
  for (let m = kLim; m > ki; m--) {
    const st = stepClipped(T, lim, m, x[m], 'min', -1);
    if (st.penetrated) return { ok: false, node: m - 1, frac: 0, x, slid, detail: 'the deceleration curve penetrates the limit curve before meeting A_i' };
    if (st.x <= 0) return { ok: false, node: m - 1, frac: 0, x, slid, detail: `the deceleration curve reaches ṡ = 0 at s ≈ ${((m - 1) / T.N).toFixed(3)} before meeting A_i — the start state is doomed` };
    if (st.x >= A[m - 1]) {
      const dLo = st.x - A[m - 1];
      const aThere = Number.isNaN(A[m]) ? aRawNext : A[m];
      const dHi = Math.max(0, aThere - x[m]);
      const frac = dLo + dHi > 0 ? dLo / (dLo + dHi) : 0;
      return { ok: true, node: m - 1, frac, x, slid, detail: '' };
    }
    x[m - 1] = st.x;
    slid[m - 1] = st.slid;
  }
  return { ok: false, node: ki, frac: 0, x, slid, detail: 'the deceleration curve from the tangent point passes below the start state — the start is doomed (Choset figure 11.2)' };
}

function writeSegment(profile: number[], mode: PhaseMode[], x: readonly number[], slid: readonly boolean[], from: number, to: number, base: 'max' | 'min'): void {
  for (let m = from; m <= to; m++) {
    if (Number.isNaN(x[m])) continue;
    profile[m] = x[m];
    mode[m] = slid[m] ? 'slide' : base;
  }
}

function pushMaxToMin(list: SwitchPoint[], meet: Meet, N: number, ki: number): void {
  if (meet.node === ki && meet.frac === 0) {
    // No max segment at all: the previous min→max switch (if any) was spurious.
    const last = list[list.length - 1];
    if (last && last.kind === 'min→max' && Math.abs(last.s - ki / N) < 0.5 / N) list.pop();
    return;
  }
  const s = (meet.node + meet.frac) / N;
  const xA = meet.x[meet.node + 1];
  list.push({ s, sdot: Math.sqrt(Math.max(0, Number.isNaN(xA) ? 0 : xA)), kind: 'max→min', onLimit: false });
}


/** Linear interpolation of a tabulated function of s on the uniform grid. */
export function interpGrid(values: readonly number[], s: number): number {
  const N = values.length - 1;
  const u = Math.min(1, Math.max(0, s)) * N;
  const k = Math.min(N - 1, Math.floor(u));
  const f = u - k;
  return values[k] + f * (values[k + 1] - values[k]);
}

/**
 * The phase state (s, ṡ, s̈) at time t along a scaling, inverting t(s) exactly on
 * the piecewise-linear x(s). The acceleration returned is the *stage* value, so
 * that a replay applying u = a s̈ + b ṡ² + c reproduces exactly this profile.
 */
export function phaseAt(sc: TimeScaling, time: number): { s: number; sdot: number; sddot: number } {
  const N = sc.N;
  const tt = Math.min(sc.tf, Math.max(0, time));
  // Locate the grid interval by binary search on t.
  let lo = 0;
  let hi = N;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (sc.t[mid] <= tt) lo = mid;
    else hi = mid;
  }
  const k = lo;
  const ds = 1 / N;
  const x0 = sc.sdot[k] * sc.sdot[k];
  const x1 = sc.sdot[k + 1] * sc.sdot[k + 1];
  const tau = tt - sc.t[k];
  let s: number;
  let sd: number;
  if (Math.abs(x1 - x0) < 1e-15) {
    sd = Math.sqrt(x0);
    s = sc.s[k] + tau * sd;
  } else {
    sd = Math.sqrt(x0) + (tau * (x1 - x0)) / (2 * ds);
    const x = sd * sd;
    s = sc.s[k] + ((x - x0) * ds) / (x1 - x0);
  }
  s = Math.min(sc.s[k + 1], Math.max(sc.s[k], s));
  return { s, sdot: sd, sddot: sc.stage[k] };
}

/** A time-stamped sample of the arm along a time-scaled path, with the torque that produces it. */
export interface ScaledSample extends TrajectorySample<Vec> {
  s: number;
  sdot: number;
  sddot: number;
  qd: Vec;
  qdd: Vec;
  u: Vec;
}

/**
 * Compose the phase-plane profile with the path: (s(t), ṡ(t)) ↦ q(s(t)) sampled
 * every `dt`. Returns a `Trajectory` — the clocked object the rest of Part V
 * hands to controllers — whose samples also carry q̇, q̈ and the torque
 * u = a s̈ + b ṡ² + c, so a replay on the Chapter 17 integrator is one loop.
 */
export function intoTrajectory(sc: TimeScaling, pd: PathDynamics, dt: number): Trajectory<Vec> & { samples: ScaledSample[] } {
  const samples: ScaledSample[] = [];
  const steps = Math.max(1, Math.ceil(sc.tf / dt));
  for (let k = 0; k <= steps; k++) {
    const time = Math.min(sc.tf, k * dt);
    const ph = phaseAt(sc, time);
    const q = pd.path.q(ph.s);
    const dq = pd.path.dq(ph.s);
    const ddq = pd.path.ddq(ph.s);
    const qd = dq.map((v) => v * ph.sdot);
    const qdd = ddq.map((v, i) => v * ph.sdot * ph.sdot + dq[i] * ph.sddot);
    const u = pd.torque(ph.s, ph.sdot, ph.sddot);
    samples.push({ t: time, q, qd, qdd, u, s: ph.s, sdot: ph.sdot, sddot: ph.sddot });
  }
  return { samples, tf: sc.tf };
}

/**
 * The hook's other clock: execute the same path at constant ṡ. Returns the
 * torque profile and whether any sample violates its limit. There is no
 * optimization here — this is what "run the path in T seconds" means.
 */
export function constantSpeedExecution(pd: PathDynamics, lim: TorqueLimits, tf: number, N = 200): { s: number[]; u: Vec[]; violated: boolean; worst: number } {
  const sdot = 1 / tf;
  const s: number[] = [];
  const u: Vec[] = [];
  let violated = false;
  let worst = 0;
  for (let k = 0; k <= N; k++) {
    const sk = k / N;
    const tau = pd.torque(sk, sdot, 0);
    s.push(sk);
    u.push(tau);
    for (let i = 0; i < tau.length; i++) {
      const frac = tau[i] > 0 ? tau[i] / lim.max[i] : tau[i] / lim.min[i];
      worst = Math.max(worst, frac);
      if (tau[i] > lim.max[i] + 1e-12 || tau[i] < lim.min[i] - 1e-12) violated = true;
    }
  }
  return { s, u, violated, worst };
}
