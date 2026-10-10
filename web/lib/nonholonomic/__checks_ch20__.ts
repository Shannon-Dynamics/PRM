/**
 * Numerical self-checks for Chapter 20 — `nonholonomic` (controllability).
 *
 * The design doc's micro-example (the unicycle bracket at x₃ = π/6, the exact
 * four-flow residue at ε = 0.1 against ε²[g₁, g₂], the car's ladder (2, 3, 4)
 * with det = −1, the trailer's (2, 3, 4, 5)), the bracket laws on seeded random
 * fields (antisymmetry, Jacobi, the linear case with Choset's sign), Example
 * 12.1.7's foliation, the §12.3 catalogue of verdicts (Chow, Sussmann, the
 * forward-only unicycle, the PBWT with one or two thrusters), the three
 * bracket-to-symmetric-product identities, Lewis–Murray on the PBWT, hopper,
 * knife-edge and Reach's dead third joint, and the sampled reachable sets.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { KnifeEdge } from '../dynamics/constraint';
import { PlanarArm } from '../dynamics/planar-arm';
import { bracketAt, detColumns, fmtVec, lieBracket, rankOf, type VectorField } from './field';
import { filtration, larc, sampleReachable, sussmannStlc, type ControlAffine } from './filtration';
import {
  HopperDynamics,
  PbwtDynamics,
  constrainedConnection,
  constrainedInputs,
  deadJointInputs,
  hopperInputs,
  kineticConnection,
  knifeEdgeExact,
  knifeEdgeInputs,
  lewisMurray,
  liftToStateSpace,
  pbwtInputs,
  pbwtSymExact,
  symmetricProduct,
} from './mechanical';
import { carMicro, carSystem, trailerMicro, unicycleMicro, unicycleSystem } from './micro';
import { car, doubleIntegrator, foliationExample, linearField, oneWayDrift, pbwt, randomQuadraticField, unicycle, wppsExample } from './systems';

const maxAbsDiff = (a: readonly number[], b: readonly number[]) => Math.max(...a.map((v, i) => Math.abs(v - b[i])));
const f6 = (x: number) => x.toFixed(6);

export function runCh20Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The micro-example: unicycle at x₃ = π/6.
  // -------------------------------------------------------------------------
  {
    const m = unicycleMicro();
    const ex3 = unicycleMicro(Math.PI / 6, 0.2); // Exercise 3's numbers
    const ex3ok = maxAbsDiff(ex3.residue, [0.02332, -0.032417, 0]) < 1e-5 && Math.abs(ex3.predicted[1] + 0.034641) < 1e-5 && ex3.residue[1] - ex3.predicted[1] > 0.002 && ex3.residue[1] - ex3.predicted[1] < 0.0025;
    const stated = [0.5, -0.866025, 0];
    const statedResidue = [0.005424, -0.008396, 0];
    const statedPredicted = [0.005, -0.00866, 0];
    const pass =
      maxAbsDiff(m.bracket, stated) < 1e-6 &&
      maxAbsDiff(m.bracket, m.bracketExact) < 1e-9 &&
      maxAbsDiff(m.residue, m.residueExact) < 1e-9 &&
      maxAbsDiff(m.residue, statedResidue) < 1e-6 &&
      maxAbsDiff(m.predicted, statedPredicted) < 1e-5 &&
      m.remainder > 1e-4 &&
      m.remainder < 5.1e-4 &&
      Math.abs(m.det - 1) < 1e-9 &&
      ex3ok;
    push(
      'nonholonomic: unicycle micro-example — [g₁, g₂] = [0.5, −0.866025, 0] at x₃ = π/6 (finite differences = hand formula), exact four-flow residue at ε = 0.1 is [0.005424, −0.008396, 0] against ε²[g₁, g₂] = [0.005, −0.00866, 0], remainder O(ε³) = 5.0×10⁻⁴, det[g₁ g₂ [g₁,g₂]] = 1; at ε = 0.2 the residue is [0.02332, −0.03242, 0] against [0.02, −0.03464, 0] (Exercise 3: the sideways error is +0.0022, the loop under-delivers)',
      pass,
      `bracket ${fmtVec(m.bracket)}; residue ${fmtVec(m.residue)} (closed form ${fmtVec(m.residueExact)}); predicted ${fmtVec(m.predicted)}; remainder ${m.remainder.toExponential(2)}; det = ${f6(m.det)}; ε = 0.2: residue ${fmtVec(ex3.residue)} predicted ${fmtVec(ex3.predicted)}`,
    );
  }

  // -------------------------------------------------------------------------
  // The ε² law: residue norm scales as ε² and its direction converges on the bracket.
  // -------------------------------------------------------------------------
  {
    const u = unicycle();
    const x = [0.3, -0.2, Math.PI / 6];
    const slopes: number[] = [];
    let angleShrinks = true;
    let prevAngle = Infinity;
    let prevNorm = 0;
    for (const eps of [0.4, 0.2, 0.1, 0.05, 0.025]) {
      const r = u.loopExact(x, eps);
      const nr = Math.hypot(r[0], r[1]);
      const br = u.bracketExact(x);
      const cosang = (r[0] * br[0] + r[1] * br[1]) / (nr * Math.hypot(br[0], br[1]));
      const ang = Math.acos(Math.min(1, cosang));
      if (ang > prevAngle + 1e-12) angleShrinks = false;
      prevAngle = ang;
      if (prevNorm > 0) slopes.push(Math.log(prevNorm / nr) / Math.log(2));
      prevNorm = nr;
    }
    const pass = slopes.every((s) => Math.abs(s - 2) < 0.15) && angleShrinks;
    push(
      'nonholonomic: four-flow residue scales as ε² (log–log slope 2.00 ± 0.15 over ε = 0.4 … 0.025) and its direction converges monotonically on the bracket',
      pass,
      `slopes ${slopes.map((s) => s.toFixed(3)).join(', ')}; final misalignment ${(prevAngle * 180 / Math.PI).toFixed(3)}°`,
    );
  }

  // -------------------------------------------------------------------------
  // Hitch's car: hand brackets, det = −sec⁴φ/L², the ladder (2, 3, 4), STLC.
  // -------------------------------------------------------------------------
  {
    const m = carMicro();
    const expected = [
      [1, 0, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, -1],
      [0, 1, 0, 0],
    ];
    const colsOk = m.columns.every((c, i) => maxAbsDiff(c, expected[i]) < 1e-9);
    // A generic configuration with nonzero steering and heading, L = 2.6 (Hitch's wheelbase).
    const c = car(2.6);
    const q = [1.0, -0.5, 0.35, 2.1];
    const g3 = lieBracket(c.drive, c.steer);
    const g4 = lieBracket(c.drive, g3);
    const e3 = maxAbsDiff(g3.eval(q), c.g3Exact(q));
    const e4 = maxAbsDiff(g4.eval(q), c.g4Exact(q));
    const det = detColumns([c.drive.eval(q), c.steer.eval(q), g3.eval(q), g4.eval(q)]);
    const pass =
      colsOk &&
      Math.abs(m.det + 1) < 1e-9 &&
      Math.abs(m.det - m.detExact) < 1e-9 &&
      m.dims.join(',') === '2,3,4' &&
      m.verdict.kind === 'stlc-symmetric' &&
      e3 < 1e-7 &&
      e4 < 1e-6 &&
      Math.abs(det - c.detExact(q)) < 1e-6;
    push(
      'nonholonomic: Hitch\'s car (eq. 12.34), L = 1 at q = 0 — columns e₁, e₃, −e₄, e₂, det[g₁ g₂ g₃ g₄] = −1, filtration (2, 3, 4), verdict STLC by symmetry; at a generic q with L = 2.6 the computed g₃, g₄ match the hand formulas and det = −sec⁴φ/L²',
      pass,
      `dims = [${m.dims.join(', ')}], det = ${f6(m.det)}, verdict ${m.verdict.kind}; |g₃ − hand| = ${e3.toExponential(1)}, |g₄ − hand| = ${e4.toExponential(1)}, det(q) = ${f6(det)} vs ${f6(c.detExact(q))}`,
    );
  }

  {
    const t0 = trailerMicro(1, 1, 0);
    const t1 = trailerMicro(1, 1, Math.PI / 2);
    const t2 = trailerMicro(2.6, 2.5, 0.8);
    const pass = t0.dims.join(',') === '2,3,4,5' && t1.dims.join(',') === '2,3,4,5' && t2.rank === 5;
    push(
      'nonholonomic: car with one trailer — the ladder is (2, 3, 4, 5) at q = 0 and still (2, 3, 4, 5) at a hitch angle of π/2 (the filtration is regular: the jackknife is not a loss of controllability, Chapter 21 locates the singularity in the chained-form and flat-output charts instead)',
      pass,
      `hitch 0: [${t0.dims.join(', ')}]; hitch π/2: [${t1.dims.join(', ')}]; Hitch\'s L = 2.6, d = 2.5 at hitch 0.8: rank ${t2.rank}`,
    );
  }

  // -------------------------------------------------------------------------
  // Bracket laws on seeded random quadratic fields; the linear case and its sign.
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(20);
    const draw = () => rng.uniform(-1, 1);
    let worstSkew = 0;
    let worstJacobi = 0;
    let worstLinear = 0;
    for (let trial = 0; trial < 10; trial++) {
      const n = 3;
      const f = randomQuadraticField(n, 'f', draw);
      const g = randomQuadraticField(n, 'g', draw);
      const h = randomQuadraticField(n, 'h', draw);
      const fg = lieBracket(f, g);
      const gh = lieBracket(g, h);
      const hf = lieBracket(h, f);
      for (let k = 0; k < 20; k++) {
        const x = [draw(), draw(), draw()];
        const a = bracketAt(f, g, x);
        const b = bracketAt(g, f, x);
        worstSkew = Math.max(worstSkew, maxAbsDiff(a, b.map((v) => -v)));
        const j1 = bracketAt(f, gh, x);
        const j2 = bracketAt(h, fg, x);
        const j3 = bracketAt(g, hf, x);
        const scale = Math.max(1, Math.hypot(...j1), Math.hypot(...j2), Math.hypot(...j3));
        worstJacobi = Math.max(worstJacobi, Math.hypot(...j1.map((v, i) => v + j2[i] + j3[i])) / scale);
      }
    }
    // Problem 8: A = [[0,1,0],[−1,0,0],[0,0,0]], B = [[0,0,0],[0,0,1],[0,−1,0]]; [Ax, Bx] = (BA − AB)x.
    const A = [
      [0, 1, 0],
      [-1, 0, 0],
      [0, 0, 0],
    ];
    const B = [
      [0, 0, 0],
      [0, 0, 1],
      [0, -1, 0],
    ];
    const gA = linearField(A, 'Ax');
    const gB = linearField(B, 'Bx');
    const BAmAB = A.map((_, i) => A.map((__, j) => B[i].reduce((s, b, k) => s + b * A[k][j], 0) - A[i].reduce((s, a, k) => s + a * B[k][j], 0)));
    for (let k = 0; k < 20; k++) {
      const x = [draw(), draw(), draw()];
      const lhs = bracketAt(gA, gB, x);
      const rhs = BAmAB.map((row) => row.reduce((s, v, j) => s + v * x[j], 0));
      worstLinear = Math.max(worstLinear, maxAbsDiff(lhs, rhs));
    }
    const pass = worstSkew < 1e-12 && worstJacobi < 1e-6 && worstLinear < 1e-12;
    push(
      'nonholonomic: bracket laws on 10 seeded random quadratic fields × 20 points — antisymmetry [f, g] = −[g, f] (exact), Jacobi identity [f,[g,h]] + [h,[f,g]] + [g,[h,f]] = 0 (one finite-difference level, < 10⁻⁶ relative), and Problem 8\'s linear case [Ax, Bx] = (BA − AB)x with Choset\'s sign',
      pass,
      `antisymmetry ${worstSkew.toExponential(1)}, Jacobi ${worstJacobi.toExponential(1)}, linear ${worstLinear.toExponential(1)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Example 12.1.7: a non-regular distribution and its nine leaves.
  // -------------------------------------------------------------------------
  {
    const ex = foliationExample();
    const rng = new Rng(7);
    let worstBr = 0;
    let worstDet = 0;
    for (let k = 0; k < 30; k++) {
      const x = [rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-3, 3)];
      const br = bracketAt(ex.g1, ex.g2, x);
      worstBr = Math.max(worstBr, maxAbsDiff(br, ex.bracketExact(x)));
      worstDet = Math.max(worstDet, Math.abs(detColumns([ex.g1.eval(x), ex.g2.eval(x), br]) - ex.detExact(x)));
    }
    const sys: ControlAffine = { name: 'ex12.1.7', n: 3, drift: null, controls: [ex.g1, ex.g2], controlSet: 'U±' };
    const interior = larc(sys, [0.7, -1.1, 0.4], 4);
    const halfPlane = larc(sys, [0, 1.3, 0.4], 5);
    const axis = larc(sys, [0, 0, 0.4], 5);
    const pass =
      worstBr < 1e-9 &&
      worstDet < 1e-9 &&
      interior.verdict.kind === 'stlc-symmetric' &&
      halfPlane.verdict.kind === 'confined' &&
      halfPlane.verdict.dim === 2 &&
      axis.verdict.kind === 'confined' &&
      axis.verdict.dim === 1 &&
      ex.leafDimension([0, 1.3, 0.4]) === 2;
    push(
      'nonholonomic: Example 12.1.7 — [g₁, g₂] = [x₁ sin x₃, −x₂ cos x₃, 0] and det = x₁x₂ at 30 random points; LARC gives STLC in a quadrant, a 2-dimensional leaf on a coordinate half-plane and a 1-dimensional leaf on the x₃-axis (rank is pointwise)',
      pass,
      `|bracket − hand| ${worstBr.toExponential(1)}, |det − x₁x₂| ${worstDet.toExponential(1)}; interior ${interior.verdict.kind}, half-plane ${halfPlane.verdict.kind} dims [${halfPlane.filtration.dims.join(',')}], axis ${axis.verdict.kind} dims [${axis.filtration.dims.join(',')}]`,
    );
  }

  // -------------------------------------------------------------------------
  // §12.3's catalogue of verdicts.
  // -------------------------------------------------------------------------
  {
    const uni = unicycleSystem('U±');
    const uniPlus = unicycleSystem('U+');
    const u = unicycle();
    const driveOnly: ControlAffine = { name: 'drive only', n: 3, drift: null, controls: [u.drive], controlSet: 'U±' };
    const x = [0.2, 0.1, 0.9];
    const a = larc(uni, x).verdict;
    const b = larc(uniPlus, x).verdict;
    const c = larc(driveOnly, x).verdict;
    const w = wppsExample();
    const wSys: ControlAffine = { name: 'wpps', n: 2, drift: w.drift, controls: [w.control], controlSet: 'U±' };
    const wBr = bracketAt(w.drift, w.control, [0.3, -0.7]);
    const wDet = detColumns([w.control.eval([0.3, -0.7]), wBr]);
    const wVerdict = larc(wSys, [0.3, -0.7]).verdict;
    const di = doubleIntegrator();
    const diSys: ControlAffine = { name: 'double integrator', n: 2, drift: di.drift, controls: [di.control], controlSet: 'U±' };
    const diRest = sussmannStlc(diSys, [0.5, 0]).verdict;
    const diMoving = sussmannStlc(diSys, [0.5, 0.3]).verdict;
    const ow = oneWayDrift();
    const owSys: ControlAffine = { name: 'x₂² drift', n: 2, drift: ow.drift, controls: [ow.control], controlSet: 'U±' };
    const owVerdict = sussmannStlc(owSys, [0, 0]);
    const pass =
      a.kind === 'stlc-symmetric' &&
      b.kind === 'stla' &&
      c.kind === 'confined' &&
      c.dim === 1 &&
      maxAbsDiff(wBr, w.bracketExact) < 1e-9 &&
      Math.abs(wDet + 0.5) < 1e-9 &&
      wVerdict.kind === 'stla' &&
      diRest.kind === 'stlc-sussmann' &&
      diMoving.kind === 'undecided' &&
      owVerdict.verdict.kind === 'stla' &&
      owVerdict.bad.some((bb) => bb.residual > 0.5);
    push(
      'nonholonomic: §12.3 catalogue — unicycle STLC for U±, only STLA for U⁺, confined to a 1-dimensional leaf with drive alone; Theorem 12.3.5\'s example has [g₀, g₁] = [0, −½], det = −½ (STLA by LARC); the double integrator is STLC at rest by Sussmann and undecided when moving; the x₂² drift is STLA with the bad bracket [g₁,[g₀,g₁]] = [2, 0] not neutralized',
      pass,
      `unicycle: ${a.kind} / ${b.kind} / ${c.kind}; wpps bracket ${fmtVec(wBr, 3)} det ${wDet.toFixed(3)} ${wVerdict.kind}; double integrator ${diRest.kind} / ${diMoving.kind}; x₂² drift ${owVerdict.verdict.kind}, bad residuals ${owVerdict.bad.map((bb) => `${bb.label}: ${bb.residual.toFixed(2)}`).join('; ')}`,
    );
  }

  // -------------------------------------------------------------------------
  // The PBWT: det = d⁴ with two thrusters (Example 12.1.8), Sussmann at rest, one thruster.
  // -------------------------------------------------------------------------
  {
    const d = 1.3;
    const p = pbwt(d, 0);
    const x = [0.4, -0.2, 0.8, 0.1, -0.3, 0.25];
    const g3 = lieBracket(p.drift, p.thrust1);
    const g4 = lieBracket(p.drift, p.thrust2);
    const g5 = lieBracket(p.thrust1, g4);
    const g6 = lieBracket(p.drift, g5);
    const computed = [p.thrust1.eval(x), p.thrust2.eval(x), g3.eval(x), g4.eval(x), g5.eval(x), g6.eval(x)];
    const exact = p.sixExact(x);
    const worst = Math.max(...computed.map((c, i) => maxAbsDiff(c, exact[i])));
    const det = detColumns(computed);
    const sys: ControlAffine = { name: 'pbwt', n: 6, drift: p.drift, controls: [p.thrust1, p.thrust2], controlSet: 'U±' };
    const rest = [0.4, -0.2, 0.8, 0, 0, 0];
    const s = sussmannStlc(sys, rest, 4);
    const badNeut = s.bad.find((b) => b.label === '[g2,[g0,g2]]');
    const one1: ControlAffine = { ...sys, controls: [p.thrust1] };
    const one2: ControlAffine = { ...sys, controls: [p.thrust2] };
    const r1 = larc(one1, rest, 5);
    const r2 = sussmannStlc(one2, rest, 6);
    // Choset's six columns for the single offset thruster: det = −16d⁸.
    const h02 = lieBracket(p.drift, p.thrust2);
    const h2 = lieBracket(p.thrust2, h02);
    const h3 = lieBracket(p.thrust2, lieBracket(p.drift, h02));
    const h4 = lieBracket(p.thrust2, h3);
    const h5 = lieBracket(p.drift, h4);
    const detSingle = detColumns([p.thrust2, h02, h2, h3, h4, h5].map((g) => g.eval(rest)));
    const grav = pbwt(d, 9.81);
    const gSys: ControlAffine = { name: 'pbwt+g', n: 6, drift: grav.drift, controls: [grav.thrust1, grav.thrust2], controlSet: 'U±' };
    const gVerdict = sussmannStlc(gSys, rest, 4).verdict;
    const pass =
      worst < 1e-6 &&
      Math.abs(det - d ** 4) < 1e-6 &&
      s.verdict.kind === 'stlc-sussmann' &&
      badNeut !== undefined &&
      badNeut.residual < 1e-6 &&
      r1.verdict.kind !== 'stla' &&
      r1.verdict.kind !== 'stlc-symmetric' &&
      r1.filtration.rank < 6 &&
      r2.verdict.kind === 'stla' &&
      r2.filtration.rank === 6 &&
      Math.abs(detSingle + 16 * d ** 8) < 1e-6 * 16 * d ** 8 &&
      gVerdict.kind === 'undecided';
    push(
      'nonholonomic: PBWT (d = 1.3, a_g = 0) — Choset\'s six fields g₁ … g₆ match the closed forms, det = d⁴; at rest Sussmann gives STLC with [g₂,[g₀,g₂]] = 2d g₁ neutralized; one thruster through the center of mass is not accessible (rank < 6), the offset thruster alone is STLA (full rank needs the degree-6 bracket [g₀,[g₂,[g₂,[g₀,[g₀,g₂]]]]]) but not STLC by this test; in gravity g₀ ≠ 0 and the theorem is silent',
      pass,
      `|fields − hand| ${worst.toExponential(1)}; det = ${f6(det)} (d⁴ = ${f6(d ** 4)}); two thrusters: ${s.verdict.kind}, bad ${s.bad.map((b) => `${b.label} ${b.residual.toExponential(1)}`).join(', ')}; u₁ only: rank ${r1.filtration.rank} (${r1.verdict.kind}); u₂ only: rank ${r2.filtration.rank} (${r2.verdict.kind}), det of Choset's six columns ${detSingle.toFixed(4)} = −16d⁸ = ${(-16 * d ** 8).toFixed(4)}; gravity: ${gVerdict.kind}`,
    );
  }

  // -------------------------------------------------------------------------
  // The three identities that make the symmetric product work (Choset p. 430).
  // -------------------------------------------------------------------------
  {
    const dyn = new PbwtDynamics(1, 1, 0);
    const Ys = pbwtInputs(1);
    const conn = kineticConnection(dyn);
    const lifted = liftToStateSpace(dyn, Ys);
    const q = [0.3, -0.4, 1.1];
    const rest = [...q, 0, 0, 0];
    const g0 = lifted.drift as VectorField;
    const [g1, g2] = lifted.controls;
    const b12 = bracketAt(g1, g2, rest);
    const b01 = bracketAt(g0, g1, rest);
    const b02 = bracketAt(g0, g2, rest);
    const b1_02 = bracketAt(g1, lieBracket(g0, g2), rest);
    const b2_02 = bracketAt(g2, lieBracket(g0, g2), rest);
    const y1 = Ys[0].eval(q);
    const y2 = Ys[1].eval(q);
    const sym12 = symmetricProduct(conn, Ys[0], Ys[1]).eval(q);
    const sym22 = symmetricProduct(conn, Ys[1], Ys[1]).eval(q);
    const e1 = Math.hypot(...b12);
    const e2 = maxAbsDiff(b01, [...y1.map((v) => -v), 0, 0, 0]);
    const e3 = maxAbsDiff(b02, [...y2.map((v) => -v), 0, 0, 0]);
    const e4 = maxAbsDiff(b1_02, [0, 0, 0, ...sym12]);
    const e5 = maxAbsDiff(b2_02, [0, 0, 0, ...sym22]);
    const e6 = maxAbsDiff(sym12, pbwtSymExact(q, 1));
    const e7 = maxAbsDiff(sym22, y1.map((v) => 2 * v));
    const pass = e1 < 1e-9 && e2 < 1e-6 && e3 < 1e-6 && e4 < 1e-5 && e5 < 1e-5 && e6 < 1e-6 && e7 < 1e-6;
    push(
      'nonholonomic: at rest on TQ, [g_i, g_j] = 0, [g₀, g_i] = [−Y_i; 0] and [g_i,[g₀, g_j]] = [0; ⟨Y_i : Y_j⟩] — the Lie-bracket route on the 6-dimensional PBWT agrees with the symmetric product on Q; ⟨Y₁ : Y₂⟩ = [d sin q₃, −d cos q₃, 0] and ⟨Y₂ : Y₂⟩ = 2d Y₁ (Choset prints d Y₁; the factor is immaterial to neutralization)',
      pass,
      `[g₁,g₂] ${e1.toExponential(1)}; [g₀,g_i] ${Math.max(e2, e3).toExponential(1)}; [g_i,[g₀,g_j]] vs ⟨:⟩ ${Math.max(e4, e5).toExponential(1)}; ⟨Y₁:Y₂⟩ vs hand ${e6.toExponential(1)}; ⟨Y₂:Y₂⟩ vs 2dY₁ ${e7.toExponential(1)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Lewis–Murray: PBWT (STLC), hopper (STLEC, not STLA, maximally reducible).
  // -------------------------------------------------------------------------
  {
    const pb = lewisMurray(kineticConnection(new PbwtDynamics()), pbwtInputs(1), [0.2, 0.5, -0.7]);
    const single = lewisMurray(kineticConnection(new PbwtDynamics()), [pbwtInputs(1)[1]], [0.2, 0.5, -0.7], { maxDeg: 3 });
    const hopConn = kineticConnection(new HopperDynamics(1, 1));
    const hy = hopperInputs(1, 1);
    const hq = [0.3, -0.2, 0.8];
    const hop = lewisMurray(hopConn, hy, hq, { maxDeg: 3, lieDeg: 2 });
    const hopSym11 = maxAbsDiff(symmetricProduct(hopConn, hy[0], hy[0]).eval(hq), [0, 0, -2 / hq[2] ** 3]);
    const hopBr = maxAbsDiff(bracketAt(hy[0], hy[1], hq), [0, -2 / hq[2] ** 3, 0]);
    const pass =
      hopSym11 < 1e-9 &&
      hopBr < 1e-9 &&
      pb.kind === 'stlc' &&
      pb.symDim === 3 &&
      pb.neutralized &&
      !pb.maximallyReducible &&
      single.kind === 'stla' &&
      single.symDim === 3 &&
      hop.kind === 'stlec' &&
      hop.symDim === 2 &&
      hop.lieSymDim === 3 &&
      hop.maximallyReducible &&
      hop.neutralized;
    push(
      'nonholonomic: Lewis–Murray — the PBWT is STLC from rest (Sym = T_qQ, ⟨Y₂:Y₂⟩ neutralized by Y₁, not maximally reducible); with the offset thruster alone rank(Y₂ ⟨Y₂:Y₂⟩ ⟨Y₂:⟨Y₂:Y₂⟩⟩) = 3 gives STLA only; the hopper has dim Sym = 2 (not STLA) but Lie(Sym) = T_qQ with ⟨Y₁:Y₁⟩ ∈ span(Y₂): STLEC and maximally reducible (Example 12.4.4, 12.4.6)',
      pass,
      `PBWT ${pb.kind} (Sym ${pb.sym.dims.join('→')}, bad ${pb.bad.map((b) => `${b.label} ${b.residual.toExponential(1)}`).join(', ')}); u₂ alone ${single.kind} (Sym ${single.sym.dims.join('→')}); hopper ${hop.kind} (Sym ${hop.sym.dims.join('→')}, Lie ${hop.lieDims.join('→')}, reducible ${hop.maximallyReducible}; ⟨Y₁:Y₁⟩ = [0,0,−2/(m²q₃³)] to ${hopSym11.toExponential(1)}, [Y₁,Y₂] to ${hopBr.toExponential(1)})`,
    );
  }

  // -------------------------------------------------------------------------
  // §12.4.3: the knife-edge through the projection P (Example 12.4.9).
  // -------------------------------------------------------------------------
  {
    const m = 2;
    const I = 0.5;
    const ke = new KnifeEdge(m, I);
    const conn = constrainedConnection(ke, ke.constraint());
    const Yt = constrainedInputs(ke, ke.constraint(), knifeEdgeInputs(m, I));
    const q = [0.4, -0.2, 0.7];
    const ex = knifeEdgeExact(q, m, I);
    const e1 = maxAbsDiff(Yt[0].eval(q), ex.Y1t);
    const e2 = maxAbsDiff(Yt[1].eval(q), ex.Y2t);
    const sym = symmetricProduct(conn, Yt[0], Yt[1]).eval(q);
    const e3 = maxAbsDiff(sym, ex.sym12);
    const br = lieBracket(Yt[0], Yt[1]);
    const br2 = lieBracket(Yt[1], br);
    const e4 = maxAbsDiff(br.eval(q), ex.br12);
    const e5 = maxAbsDiff(br2.eval(q), ex.br2_12);
    const det2 = detColumns([Yt[0].eval(q), Yt[1].eval(q), br.eval(q)]);
    const det3 = detColumns([Yt[1].eval(q), br.eval(q), br2.eval(q)]);
    const generic = lewisMurray(conn, Yt, q, { maxDeg: 3, lieDeg: 3, target: 2 });
    // The kinematic reduction {Ỹ₁, Ỹ₂} as a driftless system: its LARC at q₃ = π/2,
    // where Ỹ₁ vanishes, fails at degree 2 (det₂ = 0) and holds at degree 3.
    const side = [0.4, -0.2, Math.PI / 2];
    const kinSys: ControlAffine = { name: 'knife-edge reduction', n: 3, drift: null, controls: Yt, controlSet: 'U±' };
    const kin2 = larc(kinSys, side, 2);
    const kin3 = larc(kinSys, side, 3);
    const kinGeneric = larc(kinSys, q, 2);
    const det2side = detColumns([Yt[0].eval(side), Yt[1].eval(side), br.eval(side)]);
    const pass =
      e1 < 1e-9 &&
      e2 < 1e-9 &&
      e3 < 1e-7 &&
      e4 < 1e-7 &&
      e5 < 1e-5 &&
      Math.abs(det2 - ex.det2) < 1e-7 &&
      Math.abs(det3 - ex.det3) < 1e-5 &&
      generic.maximallyReducible &&
      generic.kind === 'stlec' &&
      kinGeneric.verdict.kind === 'stlc-symmetric' &&
      kin2.verdict.kind !== 'stlc-symmetric' &&
      kin2.filtration.rank === 2 &&
      Math.abs(det2side) < 1e-9 &&
      kin3.verdict.kind === 'stlc-symmetric';
    push(
      'nonholonomic: knife-edge with a forward force and a torque (m = 2, I = 0.5) — Ỹ₁ = PY₁, ⟨Ỹ₁:Ỹ₂⟩ = −(tan q₃/I) Ỹ₁, [Ỹ₁,Ỹ₂] and [Ỹ₂,[Ỹ₁,Ỹ₂]] match Choset; Sym(Ỹ) = span(Ỹ) (maximally reducible, STLEC); the kinematic reduction\'s LARC holds at degree 2 generically (det₂ = cos²q₃/(m²I²)) but not at q₃ = π/2, where det₂ = 0 and the degree-3 bracket with det₃ = 2/(m²I⁴) is needed',
      pass,
      `|Ỹ − hand| ${Math.max(e1, e2).toExponential(1)}, |⟨:⟩ − hand| ${e3.toExponential(1)}, |brackets − hand| ${Math.max(e4, e5).toExponential(1)}; det₂ ${f6(det2)} (${f6(ex.det2)}), det₃ ${f6(det3)} (${f6(ex.det3)}); Lewis–Murray ${generic.kind}, Lie ${generic.lieDims.join('→')}; reduction LARC at π/2: degree 2 rank ${kin2.filtration.rank} (${kin2.verdict.kind}, det₂ = ${det2side.toExponential(1)}), degree 3 ${kin3.verdict.kind} dims [${kin3.filtration.dims.join(',')}]`,
    );
  }

  // -------------------------------------------------------------------------
  // Reach's 3R with a dead third joint on the horizontal Workbench.
  // -------------------------------------------------------------------------
  {
    const arm = PlanarArm.reach3R(0);
    const conn = kineticConnection(arm);
    const Ys = deadJointInputs(arm);
    const qGeneric = [0.3, 1.1, -0.5];
    const v = lewisMurray(conn, Ys, qGeneric, { maxDeg: 3, lieDeg: 2, tol: 1e-6 });
    const qStraight = [0.3, 0, -0.5];
    const vs = lewisMurray(conn, Ys, qStraight, { maxDeg: 3, lieDeg: 2, tol: 1e-6 });
    const spanDim = rankOf(Ys.map((Y) => Y.eval(qGeneric)));
    const r11 = v.bad.find((b) => b.label === '<Y1:Y1>')?.residual ?? 0;
    const r22 = v.bad.find((b) => b.label === '<Y2:Y2>')?.residual ?? 0;
    const pass = v.kind === 'stla' && v.symDim === 3 && spanDim === 2 && !v.maximallyReducible && Math.abs(r11 - 0.105) < 0.003 && Math.abs(r22 - 0.048) < 0.003 && vs.symDim === 3 && vs.kind === 'stla';
    push(
      'nonholonomic: Reach\'s 3R with u₃ = 0 (Chapter 17\'s PlanarArm.reach3R, a_g = 0) — Y₁, Y₂ = columns of M⁻¹[e₁ e₂] give Sym(Y) = T_qQ (STLA from rest) but the bad products ⟨Y₁:Y₁⟩, ⟨Y₂:Y₂⟩ are NOT in span(Y) (relative residuals 0.105 and 0.048 at q = (0.3, 1.1, −0.5)): Lewis–Murray is silent on STLC — the arm\'s mass couples the link, unlike the free PBWT; Chapter 21 reaches equilibria anyway through decoupling fields (STLKC ⇒ STLEC)',
      pass,
      `generic q: ${v.kind}, Sym ${v.sym.dims.join('→')}, bad ${v.bad.map((b) => `${b.label} ${b.residual.toExponential(1)}`).join(', ')}; q₂ = 0: ${vs.kind}, Sym ${vs.sym.dims.join('→')}`,
    );
  }

  // -------------------------------------------------------------------------
  // Sampled reachable sets: a curve, a half-space, a blob.
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(2020);
    const draw = () => rng.next();
    const x0 = [0, 0, 0];
    const base = { T: 0.5, radius: 0.45, samples: 300, segments: 4 };
    const line = sampleReachable(unicycleSystem('U±'), x0, { ...base, active: [0] }, draw);
    const lineRank = rankOf(line.map((x) => x.map((v, i) => v - x0[i])), 1e-6);
    const both = sampleReachable(unicycleSystem('U±'), x0, base, draw);
    const bothRank = rankOf(both.map((x) => x.map((v, i) => v - x0[i])), 1e-6);
    const behind = both.filter((x) => x[0] < -0.05).length;
    const ahead = both.filter((x) => x[0] > 0.05).length;
    const fwd = sampleReachable(unicycleSystem('U+'), x0, base, draw);
    const fwdMinX = Math.min(...fwd.map((x) => x[0]));
    const fwdRank = rankOf(fwd.map((x) => x.map((v, i) => v - x0[i])), 1e-6);
    const pass = lineRank === 1 && bothRank === 3 && behind > 20 && ahead > 20 && fwdMinX >= -1e-9 && fwdRank === 3;
    push(
      'nonholonomic: sampled R^V(x, ≤ T) for the unicycle (300 histories, T = 0.5, |V| = 0.45) — drive alone fills a curve (rank 1), both inputs with U± fill a neighborhood (rank 3, points behind and ahead), forward-only U⁺ is full-dimensional yet never behind x (STLA, not STLC)',
      pass,
      `drive-only rank ${lineRank}; U± rank ${bothRank}, ${behind} behind / ${ahead} ahead of ${both.length}; U⁺ rank ${fwdRank}, min x = ${fwdMinX.toExponential(1)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Chow for the car at a configuration with the steering near its limit, and the symmetric filtration bookkeeping.
  // -------------------------------------------------------------------------
  {
    const sys = carSystem(2.6);
    const q = [3, 4, 0.58, -2.4];
    const r = larc(sys, q);
    const f = filtration([car(2.6).drive, car(2.6).steer], q, { maxDeg: 3 });
    const deg2 = f.steps[1].products.map((p) => p.field.name);
    const deg3 = f.steps[2].products.map((p) => p.field.name);
    const pass = r.verdict.kind === 'stlc-symmetric' && r.filtration.dims.join(',') === '2,3,4' && deg2.join(',') === '[g1,g2]' && deg3.includes('[g1,[g1,g2]]');
    push(
      'nonholonomic: Chow\'s recursion bookkeeping — degree 2 forms only [g₁,g₂], degree 3 forms [g₁,[g₁,g₂]] and [g₂,[g₁,g₂]]; Hitch\'s car at φ = 0.58 rad (near γ = 0.6) is still STLC with the ladder (2, 3, 4): the steering limit never costs controllability (§12.5.6)',
      pass,
      `degree 2: ${deg2.join(', ')}; degree 3: ${deg3.join(', ')}; dims [${r.filtration.dims.join(', ')}], ${r.verdict.kind}`,
    );
  }

  return out;
}
