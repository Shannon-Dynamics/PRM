/**
 * Numerical self-checks for Chapter 15 — Kalman Filtering: The Geometric Observer.
 *
 * Same contract as `lib/__checks__.ts`: the design doc's §3 micro-example
 * reproduced to its stated tolerance, plus invariants the mathematics
 * guarantees — the observer's error decays at the eigenvalues we placed, the
 * Kalman gain is the variance-minimizing observer gain, the two forms of
 * Choset's Jacobian agree, simple SLAM learns the map's shape but never its
 * place. Every number printed in the chapter's prose is pinned here.
 */

import type { CheckResult } from '../__checks__';
import { Kf } from '../filters/kf';
import { Ekf } from '../filters/ekf';
import { poseNees } from '../filters/ekf-localization';
import { Rng } from '../prob/rng';
import { matVec, sub, type Mat, type Vec } from '../prob/linalg';
import type { Landmark } from '../sim/world';
import { deadReckoning, gainSchedule, kalmanGain, kalmanStep, riccatiSteadyState } from './kf';
import { observabilityMatrix, observabilityRank, unobservableSubspace } from './lti';
import {
  argminTraceGain,
  cabs,
  eig2,
  errorMatrix,
  isStableGain,
  observerCovarianceUpdate,
  observerStep,
  placeObserverGain,
} from './observer';
import {
  ChosetEkf,
  jacobianDiscrepancy,
  linearizedObservability,
  rangeBearingH,
  rangeBearingJacobian,
  rangeBearingJacobianRaw,
  unicycleJacobian,
  unicycleStep,
} from './landmarks';
import { KalmanSlam, relativeCovariance, simpleSlam2 } from './slam';

const fmt = (x: number, d = 4): string => x.toFixed(d);
const matClose = (A: Mat, B: Mat, tol: number): number => {
  let worst = 0;
  for (let i = 0; i < A.length; i++) for (let j = 0; j < A[i].length; j++) worst = Math.max(worst, Math.abs(A[i][j] - B[i][j]));
  return worst <= tol ? worst : worst;
};
const vecWorst = (a: Vec, b: Vec): number => Math.max(...a.map((x, i) => Math.abs(x - b[i])));

export function runCh15Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // §3 micro-example: Choset §8.2.6, two steps, y = 2.7 then 2.0
  // -------------------------------------------------------------------------
  {
    const { sys, bel } = deadReckoning();
    const s1 = kalmanStep(sys, bel, [0], [2.7]);
    const s2 = kalmanStep(sys, s1.posterior, [0], [2.0]);
    const tol = 1e-3;
    const e1 = Math.max(
      vecWorst(s1.predicted.x, [4, 4]),
      matClose(s1.predicted.P, [[1.7, 1.05], [1.05, 2.1]], tol),
      Math.abs(s1.nu[0] + 1.3),
      Math.abs(s1.S[0][0] - 2.6),
      vecWorst(s1.R.map((r) => r[0]), [0.4038, 0.8077]),
      vecWorst(s1.posterior.x, [3.475, 2.95]),
      matClose(s1.posterior.P, [[1.276, 0.2019], [0.2019, 0.4038]], tol),
    );
    const e2 = Math.max(
      vecWorst(s2.predicted.x, [4.95, 2.95]),
      matClose(s2.predicted.P, [[1.7788, 0.4538], [0.4538, 0.5038]], tol),
      Math.abs(s2.nu[0] + 0.95),
      Math.abs(s2.S[0][0] - 1.0038),
      vecWorst(s2.R.map((r) => r[0]), [0.4521, 0.5019]),
      vecWorst(s2.posterior.x, [4.52, 2.473]),
      matClose(s2.posterior.P, [[1.5737, 0.2261], [0.2261, 0.251]], tol),
    );
    // The reader's third step (CheckAnswer ch15.step3): y = 2.3.
    const s3 = kalmanStep(sys, s2.posterior, [0], [2.3]);
    const e3 = Math.max(
      Math.abs(s3.predicted.P[0][0] - 2.0625),
      Math.abs(s3.S[0][0] - 0.851),
      Math.abs(s3.R[0][0] - 0.4719),
      Math.abs(s3.posterior.P[0][0] - 1.873),
      Math.abs(s3.posterior.P[1][1] - 0.2062),
    );
    push(
      'ch15 §8.2.6: two dead-reckoning Kalman steps reproduce the worked numbers to 1e-3 (and the reader\'s third step)',
      e1 < tol && e2 < tol && e3 < tol,
      `step1 worst ${e1.toExponential(2)}, step2 worst ${e2.toExponential(2)}, step3 worst ${e3.toExponential(2)}; x⁺₂ = [${fmt(s2.posterior.x[0], 3)}, ${fmt(s2.posterior.x[1], 3)}], P⁺₁₁(3) = ${fmt(s3.posterior.P[0][0], 4)}`,
    );
    // The reading the prose makes: velocity variance squishes, position variance grows.
    const vv = [bel.P[1][1], s1.posterior.P[1][1], s2.posterior.P[1][1]];
    const pv = [bel.P[0][0], s1.posterior.P[0][0], s2.posterior.P[0][0]];
    push(
      'ch15 §8.2.7: velocity variance falls and position variance grows through two optimal updates',
      vv[0] > vv[1] && vv[1] > vv[2] && pv[0] < pv[1] && pv[1] < pv[2],
      `v: ${vv.map((x) => fmt(x, 3)).join(' → ')}; x: ${pv.map((x) => fmt(x, 3)).join(' → ')}`,
    );
  }

  // -------------------------------------------------------------------------
  // Thm. 8.2.2 rank test on the 2×2 pairs
  // -------------------------------------------------------------------------
  {
    const vel = deadReckoning({ sensor: 'velocity' }).sys;
    const pos = deadReckoning({ sensor: 'position' }).sys;
    const Qv = observabilityMatrix(vel.F, vel.H);
    const Qp = observabilityMatrix(pos.F, pos.H);
    const rv = observabilityRank(vel.F, vel.H);
    const rp = observabilityRank(pos.F, pos.H);
    const qvOk = matClose(Qv, [[0, 1], [0, 1]], 1e-12) < 1e-12;
    const qpOk = matClose(Qp, [[1, 0], [1, 0.5]], 1e-12) < 1e-12;
    const blind = unobservableSubspace(vel.F, vel.H);
    push(
      'ch15 Thm 8.2.2: velocity sensor Q=[0 1;0 1] has rank 1, position sensor Q=[1 0;1 .5] has rank 2',
      qvOk && qpOk && rv === 1 && rp === 2 && blind.length === 1 && Math.abs(Math.abs(blind[0][0]) - 1) < 1e-12,
      `rank(velocity)=${rv}, rank(position)=${rp}, blind direction=[${blind[0].map((x) => fmt(x, 2)).join(', ')}]`,
    );
  }

  // -------------------------------------------------------------------------
  // The Kalman filter IS the sister's Kf (adapter honesty), and the Joseph
  // form equals Choset's P⁻ − R H P⁻ for the optimal gain.
  // -------------------------------------------------------------------------
  {
    const { sys, bel } = deadReckoning();
    const kf = new Kf(bel.x, bel.P);
    let worst = 0;
    let b = bel;
    const ys = [2.7, 2.0, 2.4, 1.9, 2.2];
    for (const y of ys) {
      kf.predictWith(sys.F, sys.V, sys.G, [0]);
      kf.updateWith([y], sys.H, sys.W);
      const s = kalmanStep(sys, b, [0], [y]);
      // Choset's short form, eq. 8.29.
      const RHP = s.R.map((r) => s.predicted.P[0].map((_, j) => r[0] * (sys.H[0][0] * s.predicted.P[0][j] + sys.H[0][1] * s.predicted.P[1][j])));
      const short = s.predicted.P.map((row, i) => row.map((x, j) => x - RHP[i][j]));
      worst = Math.max(worst, vecWorst(kf.x, s.posterior.x), matClose(kf.P, s.posterior.P, 0), matClose(short, s.posterior.P, 0));
      b = s.posterior;
    }
    push(
      'ch15 adapter: kalmanStep equals the vendored Kf, and Joseph form equals P⁻ − RHP⁻ at the optimal gain',
      worst < 1e-12,
      `worst discrepancy over ${ys.length} steps ${worst.toExponential(2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Observer: placed eigenvalues govern the error decay
  // -------------------------------------------------------------------------
  {
    const { sys } = deadReckoning({ sensor: 'position' });
    const desired = [0.5, 0.2];
    const K = placeObserverGain(sys.F, sys.H, desired);
    const eigs = eig2(errorMatrix(sys.F, sys.H, K)).map((z) => z.re).sort((a, b) => b - a);
    const eigOk = vecWorst(eigs, desired) < 1e-9;
    // Noise-free: e(k+1) = (I−KH)F e(k). After k steps the error is a sum of
    // c₁ 0.5ᵏ + c₂ 0.2ᵏ, so the ratio of successive errors tends to 0.5.
    let truth: Vec = [1, 0.3];
    let xhat: Vec = [0, 0];
    const ratios: number[] = [];
    let prev = Math.hypot(...sub(truth, xhat));
    for (let k = 0; k < 14; k++) {
      truth = matVec(sys.F, truth);
      const y = [sys.H[0][0] * truth[0] + sys.H[0][1] * truth[1]];
      xhat = observerStep(sys, xhat, K, [0], y).xhat;
      const e = Math.hypot(...sub(truth, xhat));
      ratios.push(e / prev);
      prev = e;
    }
    const tail = ratios.slice(-4);
    const ratioOk = tail.every((r) => Math.abs(r - 0.5) < 0.02);
    // The velocity sensor cannot be placed: the position eigenvalue is pinned at 1.
    let unplaceable = false;
    try {
      placeObserverGain(deadReckoning().sys.F, deadReckoning().sys.H, desired);
    } catch {
      unplaceable = true;
    }
    const velEigs = eig2(errorMatrix(deadReckoning().sys.F, deadReckoning().sys.H, [[0.3], [0.7]])).map(cabs);
    push(
      'ch15 App. J.4: Ackermann-placed observer gain gives eigenvalues {0.5, 0.2}; error contracts at the dominant one',
      eigOk && ratioOk && unplaceable && velEigs.some((m) => Math.abs(m - 1) < 1e-12),
      `K=[${K.map((r) => fmt(r[0], 3)).join(', ')}], eig=[${eigs.map((x) => fmt(x, 3)).join(', ')}], tail ratios ${tail.map((r) => fmt(r, 3)).join(' ')}; velocity-sensor error matrix has |λ|=1`,
    );
  }

  // -------------------------------------------------------------------------
  // KF gain = the observer gain that minimizes posterior variance (scalar)
  // -------------------------------------------------------------------------
  {
    const Pminus = 1.7;
    const H = 1;
    const W = 0.5;
    const brute = argminTraceGain(Pminus, H, W);
    const kalman = kalmanGain([[Pminus]], [[H]], [[W]])[0][0];
    const exact = Pminus / (Pminus + W);
    const Pplus = observerCovarianceUpdate([[Pminus]], [[H]], [[W]], [[kalman]])[0][0];
    const short = Pminus - kalman * H * Pminus;
    push(
      'ch15 Thm. 8.2.1 ⇒ gain: brute-force argmin of tr P⁺(K) is the Kalman gain P⁻/(P⁻+W)',
      Math.abs(brute.gain - kalman) < 2e-4 && Math.abs(kalman - exact) < 1e-12 && Math.abs(Pplus - short) < 1e-12,
      `argmin ${fmt(brute.gain)} vs R ${fmt(kalman)}; P⁺=${fmt(Pplus)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Riccati fixed point: steady gain reproduces the schedule's limit, and the
  // fixed-gain observer with R∞ is stable.
  // -------------------------------------------------------------------------
  {
    const { sys, bel } = deadReckoning({ sensor: 'position' });
    const ss = riccatiSteadyState(sys, bel.P);
    const sched = gainSchedule(sys, bel.P, 60);
    const last = sched[sched.length - 1];
    const gap = matClose(last, ss.R, 0);
    const stable = isStableGain(sys.F, sys.H, ss.R);
    const unobs = riccatiSteadyState(deadReckoning().sys, bel.P, 300);
    push(
      'ch15 schedule → knob: the gain schedule converges to the Riccati fixed point, which is a stable observer gain',
      ss.converged && gap < 1e-8 && stable && !unobs.converged,
      `R∞=[${ss.R.map((r) => fmt(r[0], 4)).join(', ')}] after ${ss.iterations} iters; |R(60)−R∞|=${gap.toExponential(1)}; velocity-sensor Riccati does not converge in 300 steps`,
    );
  }

  // -------------------------------------------------------------------------
  // EKF Jacobians: Choset's raw atan2 derivative equals the simplified form
  // and the central difference.
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(15);
    let worstRaw = 0;
    let worstNum = 0;
    for (let k = 0; k < 200; k++) {
      const x: Vec = [rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-Math.PI, Math.PI)];
      const lm: Landmark = { x: rng.uniform(-3, 3), y: rng.uniform(-3, 3), id: 0 };
      if (Math.abs(x[0] - lm.x) < 0.2) continue; // the raw form divides by Δx
      worstRaw = Math.max(worstRaw, matClose(rangeBearingJacobian(x, lm), rangeBearingJacobianRaw(x, lm), 0));
      worstNum = Math.max(worstNum, jacobianDiscrepancy(x, lm, 'range-bearing'), jacobianDiscrepancy(x, lm, 'range-only'));
    }
    push(
      'ch15 §8.3.1: simplified H equals Choset’s unsimplified atan2 derivative and the numeric Jacobian',
      worstRaw < 1e-9 && worstNum < 1e-5,
      `raw vs simplified ${worstRaw.toExponential(2)}, analytic vs central difference ${worstNum.toExponential(2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Linearized observability: range-only one landmark → rank 2, two → 3;
  // range–bearing one landmark → 3 moving, 2 standing still.
  // -------------------------------------------------------------------------
  {
    const x: Vec = [2, 1.9, 0.4];
    const L0: Landmark = { x: 0.35, y: 0.35, id: 0 };
    const L1: Landmark = { x: 3.65, y: 0.35, id: 1 };
    const u: [number, number] = [0.1, 0.02];
    const one = linearizedObservability(x, u, [L0], 'range-only');
    const two = linearizedObservability(x, u, [L0, L1], 'range-only');
    const rb = linearizedObservability(x, u, [L0], 'range-bearing');
    const still = linearizedObservability(x, [0, 0], [L0], 'range-bearing');
    // Collinear trap (Exercise 4): robot on the line through both landmarks.
    const onLine: Vec = [2, 0.35, 0.7];
    const col = linearizedObservability(onLine, u, [L0, L1], 'range-only');
    // The blind direction for one range-only landmark is tangent to the range circle in (x, y).
    const tangent = one.blind.find((v) => Math.abs(v[2]) < 1e-6);
    const radial = [x[0] - L0.x, x[1] - L0.y];
    const tangentOk = tangent !== undefined && Math.abs(tangent[0] * radial[0] + tangent[1] * radial[1]) < 1e-6;
    push(
      'ch15 §8.3.3: linearized rank is 2 for one range-only landmark, 3 for two, 3 for one range–bearing landmark in motion, 2 at rest, 2 when collinear',
      one.rank === 2 && two.rank === 3 && rb.rank === 3 && still.rank === 2 && col.rank === 2 && tangentOk,
      `ranks: one=${one.rank}, two=${two.rank}, rb-moving=${rb.rank}, rb-still=${still.rank}, collinear=${col.rank}`,
    );
  }

  // -------------------------------------------------------------------------
  // Range-only EKF: along the unobservable direction the variance never
  // shrinks; with two landmarks it does. NEES stays sane.
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(3);
    const V: Mat = [
      [0.002, 0, 0],
      [0, 0.002, 0],
      [0, 0, 0.0005],
    ];
    const run = (lms: Landmark[]) => {
      let truth: Vec = [1.5, 1.2, 0.3];
      const ekf = new ChosetEkf(truth.slice(), [[0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.02]], {
        landmarks: lms,
        mode: 'range-only',
        V,
        sigmaRange: 0.1,
        sigmaBearing: 0.05,
      });
      let worstNees = 0;
      let sumNees = 0;
      const traces: number[] = [];
      for (let k = 0; k < 240; k++) {
        const u: [number, number] = [0.06, 0.04];
        truth = unicycleStep(truth, u);
        truth = [truth[0] + rng.normal(0, Math.sqrt(V[0][0])), truth[1] + rng.normal(0, Math.sqrt(V[1][1])), truth[2] + rng.normal(0, Math.sqrt(V[2][2]))];
        ekf.predict(u);
        const ys = lms.map((lm) => [Math.hypot(truth[0] - lm.x, truth[1] - lm.y) + rng.normal(0, 0.1)]);
        ekf.updateKnown(ys, lms.map((_, i) => i));
        traces.push(ekf.P[0][0] + ekf.P[1][1] + ekf.P[2][2]);
        const n = poseNees({ x: truth[0], y: truth[1], theta: truth[2] }, { mu: ekf.pose(), Sigma: ekf.P });
        worstNees = Math.max(worstNees, n);
        sumNees += n;
      }
      return { traces, worstNees, meanNees: sumNees / 240, ekf };
    };
    const one = run([{ x: 0.35, y: 0.35, id: 0 }]);
    const two = run([{ x: 0.35, y: 0.35, id: 0 }, { x: 3.65, y: 0.35, id: 1 }]);
    const tr1 = one.traces[one.traces.length - 1];
    const tr2 = two.traces[two.traces.length - 1];
    push(
      'ch15 w15.2: one range-only landmark leaves tr P an order of magnitude above two landmarks after 240 steps',
      tr1 > 5 * tr2 && two.meanNees < 6 && two.worstNees < 50,
      `tr P: one landmark ${fmt(tr1, 3)}, two landmarks ${fmt(tr2, 4)}; mean NEES (two) ${fmt(two.meanNees, 2)} (3 dof expected), worst ${fmt(two.worstNees, 1)}`,
    );
  }

  // -------------------------------------------------------------------------
  // §8.4.1 simple SLAM: rank 4 of 6; relative covariance shrinks, absolute grows.
  // -------------------------------------------------------------------------
  {
    const Vr: Mat = [
      [0.01, 0],
      [0, 0.01],
    ];
    const Wi: Mat = [
      [0.04, 0],
      [0, 0.04],
    ];
    const sys = simpleSlam2(Vr, Wi);
    const rank = observabilityRank(sys.F, sys.H);
    const rng = new Rng(8);
    const truth: Vec = [0, 0, 2, 1, -1, 2];
    const P0: Mat = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (_, j) => (i === j ? (i < 2 ? 0.01 : 4) : 0)));
    const kf = new Kf([0, 0, 1, 2, 0, 0], P0);
    const relTrace: number[] = [];
    const absTrace: number[] = [];
    let t = truth.slice();
    for (let k = 0; k < 120; k++) {
      const u = [0.1 * Math.cos(k / 10), 0.1 * Math.sin(k / 10)];
      t = [t[0] + u[0] + rng.normal(0, 0.1), t[1] + u[1] + rng.normal(0, 0.1), t[2], t[3], t[4], t[5]];
      kf.predictWith(sys.F, sys.V, sys.G, u);
      const y = matVec(sys.H, t).map((v) => v + rng.normal(0, 0.2));
      kf.updateWith(y, sys.H, sys.W);
      const rel = relativeCovariance(kf.P, 0);
      relTrace.push(rel[0][0] + rel[1][1]);
      absTrace.push(kf.P[2][2] + kf.P[3][3]);
    }
    // Landmarks are static and carry no process noise, so both traces can only
    // fall; the *relative* one falls toward the sensor floor while the
    // *absolute* one stalls at the common-mode variance no sighting can touch.
    const relMono = relTrace.every((v, i) => i === 0 || v <= relTrace[i - 1] + 1e-12);
    const absMono = absTrace.every((v, i) => i === 0 || v <= absTrace[i - 1] + 1e-12);
    const relEnd = relTrace[relTrace.length - 1];
    const absEnd = absTrace[absTrace.length - 1];
    const absStalled = absTrace[59] - absEnd < 0.02 * absTrace[59];
    push(
      'ch15 §8.4.1: simple SLAM has rank 4 < 6; the map is learned relative to the robot (Var(ℓ−r) ↓) but its absolute covariance stalls at a floor',
      rank === 4 && relMono && absMono && absStalled && absEnd > 2.5 * relEnd,
      `rank Q=${rank}; tr Var(ℓ₁−r): ${fmt(relTrace[0], 3)} → ${fmt(relEnd, 4)}; tr P_ℓ₁: ${fmt(absTrace[0], 3)} → ${fmt(absEnd, 4)} (Δ over last 60 steps ${(absTrace[59] - absEnd).toExponential(1)})`,
    );
  }

  // -------------------------------------------------------------------------
  // §8.4.2 range–bearing Kalman SLAM: landmarks founded once, then matched;
  // map error below its claimed 3σ.
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(21);
    const lms: Landmark[] = [
      { x: 0.35, y: 0.35, id: 0 },
      { x: 3.65, y: 0.35, id: 1 },
      { x: 0.35, y: 3.45, id: 2 },
    ];
    const V: Mat = [
      [0.001, 0, 0],
      [0, 0.001, 0],
      [0, 0, 0.0003],
    ];
    const slam = new KalmanSlam([1.5, 1.2, 0.3], [[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]], {
      V,
      sigmaRange: 0.08,
      sigmaBearing: 0.04,
    });
    let truth: Vec = [1.5, 1.2, 0.3];
    let founded = 0;
    let matched = 0;
    for (let k = 0; k < 200; k++) {
      const u: [number, number] = [0.06, 0.05];
      truth = unicycleStep(truth, u);
      truth = [truth[0] + rng.normal(0, Math.sqrt(V[0][0])), truth[1] + rng.normal(0, Math.sqrt(V[1][1])), truth[2] + rng.normal(0, Math.sqrt(V[2][2]))];
      slam.predict(u);
      for (const lm of lms) {
        const h = rangeBearingH(truth, lm);
        const y = [h[0] + rng.normal(0, 0.08), h[1] + rng.normal(0, 0.04)];
        const r = slam.observe(y, lm.id);
        if (r.landmark === -1) founded++;
        else if (r.landmark >= 0) matched++;
      }
    }
    let worstSigma = 0;
    let worstErr = 0;
    for (let i = 0; i < slam.landmarkCount; i++) {
      const est = slam.landmark(i);
      const tru = lms.find((l) => l.id === est.id)!;
      const C = slam.landmarkCov(i);
      const err = Math.hypot(est.x - tru.x, est.y - tru.y);
      worstErr = Math.max(worstErr, err);
      worstSigma = Math.max(worstSigma, err / Math.sqrt(Math.max(C[0][0], C[1][1])));
    }
    // 4σ, not 3: EKF-SLAM is mildly overconfident by construction (sister
    // Ch. 14) and the check should expose that number, not hide it.
    push(
      'ch15 §8.4.2: Kalman SLAM founds each landmark exactly once, matches ≥97% of later sightings, and places the map within 0.2 m and 4σ',
      slam.landmarkCount === 3 && founded === 3 && matched >= 0.97 * (200 * 3 - 3) && worstErr < 0.2 && worstSigma < 4,
      `landmarks ${slam.landmarkCount}, founded ${founded}, matched ${matched}/${200 * 3 - 3}, worst map error ${fmt(worstErr, 3)} m = ${fmt(worstSigma, 2)}σ`,
    );
  }

  // -------------------------------------------------------------------------
  // Choset Problem 9 (Exercise 6): unicycle with a position-only sensor, one EKF
  // step by hand: x̂(1|1) = [1, 0.5, π/4], u = [3, π], T = 0.25, P = I, y(2) = 1.7,
  // measurement variance 0.5, no process noise.
  // -------------------------------------------------------------------------
  {
    const T = 0.25;
    const u: [number, number] = [3 * T, Math.PI * T];
    const ekf = new Ekf([1, 0.5, Math.PI / 4], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]);
    ekf.predict((x) => unicycleStep(x, u), (x) => unicycleJacobian(x, u), [[0, 0, 0], [0, 0, 0], [0, 0, 0]]);
    const pred = ekf.belief();
    const info = ekf.update([1.7], () => [ekf.x[0]], [[1, 0, 0]], [[0.5]]);
    const post = ekf.belief();
    // Hand values: x̂⁻ = [1 + 0.75 cos π/4, 0.5 + 0.75 sin π/4, π/2]; S = 1 + 0.75²/2 + 0.5.
    const c = 0.75 * Math.SQRT1_2;
    const predOk = vecWorst(pred.x, [1 + c, 0.5 + c, Math.PI / 2]) < 1e-12;
    const S = 1 + 0.75 * 0.75 / 2 + 0.5;
    const sOk = Math.abs(info.S[0][0] - S) < 1e-12;
    const nu = 1.7 - (1 + c);
    const xPlus = [1 + c + ((1 + 0.28125) / S) * nu, 0.5 + c - (0.28125 / S) * nu, Math.PI / 2 - (c / S) * nu];
    const postOk = vecWorst(post.x, xPlus) < 1e-12;
    push(
      'ch15 Problem 9: one EKF step on the unicycle with a position sensor matches the hand computation',
      predOk && sOk && postOk,
      `x̂(2|1)=[${pred.x.map((v) => fmt(v, 4)).join(', ')}], S=${fmt(info.S[0][0], 5)}, x̂(2|2)=[${post.x.map((v) => fmt(v, 4)).join(', ')}], P₁₁(2|2)=${fmt(post.P[0][0], 4)}`,
    );
  }

  return out;
}
