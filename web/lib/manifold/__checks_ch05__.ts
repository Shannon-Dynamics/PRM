/**
 * Numerical self-checks for Chapter 5's manifold module.
 *
 * Same contract as `lib/__checks__.ts`: the design doc's micro-example to its
 * stated tolerance, plus the invariants the mathematics guarantees — metric
 * axioms on random triples, the geodesic property of `interpolate`, the
 * double cover of SO(3), gimbal lock as a detected failure, chart independence
 * of Dφ, and monotonicity of the component count under edge identification.
 *
 *     import { runCh05Checks } from './manifold/__checks_ch05__';
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { CHOSET_ARM, chainRuleExample, dphi, dphiChartShiftInvariance } from './dphi';
import { armRaster, componentReport, seamConnection, withJointLimits } from './lab';
import { Product, checkGeodesic, checkMetricAxioms, geodesicPath, type Manifold } from './manifold';
import { Rd } from './rd';
import { FOUR_CHART_ATLAS, S1, TWO_CHART_ATLAS, chartDistance, chartMidpoint, chartsContaining, s1Dist, s1Interpolate, transitionMap } from './s1';
import { CHOSET_FRAMES, SE2, frameWorked } from './se2';
import {
  HAAR_MEAN_ANGLE,
  SO3,
  axisAngle,
  eulerZYZ,
  eulerZYZToMatrix,
  matrixToQuat,
  quatFromAxisAngle,
  quatMul,
  quatToMatrix,
  rotationAngleBetween,
  so3Dist,
  so3Interpolate,
  type Quat,
} from './so3';
import { T2, labelComponents } from './torus';

const fmt = (x: number) => (Math.abs(x) < 1e-4 && x !== 0 ? x.toExponential(2) : x.toFixed(4));
const close = (a: number, b: number, tol: number) => Math.abs(a - b) <= tol;

export function runCh05Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The micro-example (design doc §3): seams on S¹ and T², the weighted SE(2) metric
  // -------------------------------------------------------------------------
  {
    const a = 0.1, b = 6.2;
    const d = s1Dist(a, b);
    const mid = s1Interpolate(a, b, 0.5);
    push(
      'manifold: S¹ dist(0.1, 6.2) = 0.1832, not the chart’s 6.1',
      close(d, 2 * Math.PI - 6.1, 1e-12) && close(d, 0.1832, 5e-5) && close(chartDistance(a, b), 6.1, 1e-12),
      `dist ${fmt(d)}, chart |a−b| ${fmt(chartDistance(a, b))}`,
    );
    push(
      'manifold: S¹ midpoint goes the short way — 0.0084, not the chart midpoint 3.15',
      close(mid, 0.1 - d / 2, 1e-12) && close(mid, 0.0084, 5e-5) && close(chartMidpoint(a, b), 3.15, 1e-12),
      `midpoint ${fmt(mid)} (chart ${fmt(chartMidpoint(a, b))})`,
    );
    const dT2 = T2.dist([0.1, 3.0], [6.2, 3.5]);
    push(
      'manifold: T² dist((0.1,3.0),(6.2,3.5)) = √(0.1832² + 0.5²) = 0.5325',
      close(dT2, Math.hypot(2 * Math.PI - 6.1, 0.5), 1e-12) && close(dT2, 0.5325, 5e-5),
      `dist ${fmt(dT2)}`,
    );
    const se2 = new SE2(0.5, { minX: -5, minY: -5, maxX: 5, maxY: 5 });
    const dSE2 = se2.dist({ x: 0, y: 0, theta: 0 }, { x: 1, y: 0, theta: Math.PI });
    push(
      'manifold: SE(2) with w = 0.5 m/rad: dist((0,0,0),(1,0,π)) = √(1 + 0.25π²) = 1.8621',
      close(dSE2, Math.sqrt(1 + 0.25 * Math.PI * Math.PI), 1e-12) && close(dSE2, 1.8621, 5e-5),
      `dist ${fmt(dSE2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Choset's frames (§3.5.1, figures 3.17–3.19)
  // -------------------------------------------------------------------------
  {
    const w = frameWorked(CHOSET_FRAMES);
    const ok =
      close(w.T_AC.x, 2, 1e-12) && close(w.T_AC.y, 1, 1e-12) && close(w.T_AC.theta, Math.PI / 2, 1e-12) &&
      close(w.w_B[0], -3, 1e-12) && close(w.w_B[1], 1, 1e-12) &&
      close(w.w_A[0], 1, 1e-12) && close(w.w_A[1], -1, 1e-12) &&
      close(w.w_displaced[0], -3, 1e-12) && close(w.w_displaced[1], 1, 1e-12) &&
      close(w.body.x, -2, 1e-12) && close(w.body.y, -1, 1e-12) && close(s1Dist(w.body.theta, 0), 0, 1e-12) &&
      close(w.world.x, 2, 1e-12) && close(w.world.y, 1, 1e-12) && close(s1Dist(w.world.theta, 0), 0, 1e-12) &&
      close(w.T_BC_recovered.x, -4, 1e-12) && close(w.T_BC_recovered.y, -1, 1e-12);
    push(
      'manifold: Choset’s frames — T_AC = (2, 1, π/2), w_B = (−3, 1), w_A = (1, −1), body (−2,−1,0) ≠ world (2,1,0)',
      ok,
      `T_AC (${fmt(w.T_AC.x)}, ${fmt(w.T_AC.y)}, ${fmt(w.T_AC.theta)}); w_B (${w.w_B.map(fmt)}); w_A (${w.w_A.map(fmt)}); body (${fmt(w.body.x)}, ${fmt(w.body.y)}); world (${fmt(w.world.x)}, ${fmt(w.world.y)})`,
    );
  }

  // -------------------------------------------------------------------------
  // Metric axioms and the geodesic property, on every space the chapter ships
  // -------------------------------------------------------------------------
  {
    const se2 = new SE2(0.7, { minX: -3, minY: -3, maxX: 3, maxY: 3 });
    const spaces: Manifold<unknown>[] = [
      S1 as Manifold<unknown>,
      T2 as Manifold<unknown>,
      new Rd(3, { lo: [-1, -1, -1], hi: [1, 1, 1] }) as Manifold<unknown>,
      se2 as Manifold<unknown>,
      SO3 as Manifold<unknown>,
      new Product(se2, S1, 0.4) as Manifold<unknown>,
    ];
    let worstTri = 0, worstSym = 0, worstDef = 0, degenerate = 0;
    let worstEnd = 0, worstProp = 0, worstAdd = 0;
    const names: string[] = [];
    for (const m of spaces) {
      const rng = new Rng(55);
      const r = checkMetricAxioms(m, rng, 2000);
      worstTri = Math.max(worstTri, r.triangle);
      worstSym = Math.max(worstSym, r.symmetry);
      worstDef = Math.max(worstDef, r.definiteness);
      degenerate += r.nondegeneracy;
      const g = checkGeodesic(m, new Rng(56), 400);
      worstEnd = Math.max(worstEnd, g.endpoint);
      worstProp = Math.max(worstProp, g.proportional);
      worstAdd = Math.max(worstAdd, g.additivity);
      names.push(m.name);
    }
    push(
      'manifold: metric axioms (Def. C.2.1) hold on 2000 seeded triples for S¹, T², ℝ³, SE(2), SO(3), SE(2)×S¹',
      worstTri < 1e-9 && worstSym < 1e-12 && worstDef < 1e-12 && degenerate === 0,
      `worst triangle violation ${worstTri.toExponential(2)}, symmetry ${worstSym.toExponential(2)}`,
    );
    push(
      'manifold: interpolate lands on both endpoints and dist(a, γ(t)) = t·dist(a, b) on every space',
      worstEnd < 1e-9 && worstProp < 1e-9 && worstAdd < 1e-9,
      `endpoint ${worstEnd.toExponential(2)}, |d(a,γ(t)) − t·d| ${worstProp.toExponential(2)}, over ${names.length} spaces`,
    );
  }

  // -------------------------------------------------------------------------
  // S¹: wrap-around shortest arc, and the atlas covers
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(9);
    let maxDist = 0;
    let maxStep = 0;
    let longWay = 0;
    for (let k = 0; k < 2000; k++) {
      const a = rng.uniform(-4 * Math.PI, 4 * Math.PI);
      const b = rng.uniform(-4 * Math.PI, 4 * Math.PI);
      const d = s1Dist(a, b);
      maxDist = Math.max(maxDist, d);
      const path = geodesicPath(S1, a, b, 16);
      for (let i = 1; i < path.length; i++) maxStep = Math.max(maxStep, s1Dist(path[i - 1], path[i]));
      // A geodesic never visits a point farther from a than its endpoint.
      for (const p of path) if (s1Dist(a, p) > d + 1e-12) longWay++;
    }
    // Specific seam crossing: from 3.0 to −3.0 the short way is through π, length 2π − 6.
    const seam = s1Interpolate(3.0, -3.0, 0.5);
    push(
      'manifold: S¹ geodesics take the short arc — dist ≤ π, the midpoint of 3.0 → −3.0 is π, never the long way',
      maxDist <= Math.PI + 1e-12 && maxStep <= Math.PI / 16 + 1e-12 && longWay === 0 && close(s1Dist(seam, Math.PI), 0, 1e-12),
      `max dist ${fmt(maxDist)}, midpoint(3.0, −3.0) = ${fmt(seam)}`,
    );

    let uncovered4 = 0, uncovered2 = 0, badTransition = 0, worstTrans = 0;
    for (let k = 0; k < 720; k++) {
      const th = -Math.PI + (k / 720) * 2 * Math.PI + 1e-3;
      if (chartsContaining(FOUR_CHART_ATLAS, th).length === 0) uncovered4++;
      if (chartsContaining(TWO_CHART_ATLAS, th).length === 0) uncovered2++;
    }
    // Choset's worked case: φ₁ ∘ φ₃⁻¹(z) = √(1 − z²) on (0, 1).
    for (let k = 1; k < 100; k++) {
      const z = k / 100;
      const t = transitionMap(FOUR_CHART_ATLAS, 0, 2, z);
      if (t === null) badTransition++;
      else worstTrans = Math.max(worstTrans, Math.abs(t - Math.sqrt(1 - z * z)));
    }
    push(
      'manifold: the four-chart and two-chart atlases cover S¹, and φ₁∘φ₃⁻¹(z) = √(1 − z²) on (0, 1)',
      uncovered4 === 0 && uncovered2 === 0 && badTransition === 0 && worstTrans < 1e-12,
      `uncovered points: ${uncovered4}/${uncovered2}; transition error ${worstTrans.toExponential(2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // SO(3): double cover, metric = axis–angle of RᵀR′, gimbal lock, Haar sampling
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(21);
    let worstCover = 0, worstAngle = 0, worstRound = 0, worstInterp = 0;
    for (let k = 0; k < 500; k++) {
      const q = SO3.sample(rng);
      const p = SO3.sample(rng);
      const neg: Quat = { w: -q.w, x: -q.x, y: -q.y, z: -q.z };
      worstCover = Math.max(worstCover, so3Dist(q, neg), so3Dist(q, so3Interpolate(q, neg, 0.37)));
      // dist equals the axis–angle θ of RᵀR′ computed from the matrices.
      worstAngle = Math.max(worstAngle, Math.abs(so3Dist(q, p) - rotationAngleBetween(quatToMatrix(q), quatToMatrix(p))));
      // matrix → quaternion → matrix round trip, through every branch of Shepperd's method.
      worstRound = Math.max(worstRound, so3Dist(q, matrixToQuat(quatToMatrix(q))));
      // Interpolation of q toward p·(−1) equals interpolation toward p: the sign is not a rotation.
      const pneg: Quat = { w: -p.w, x: -p.x, y: -p.y, z: -p.z };
      worstInterp = Math.max(worstInterp, so3Dist(so3Interpolate(q, p, 0.5), so3Interpolate(q, pneg, 0.5)));
    }
    push(
      'manifold: SO(3) double cover — dist(Q, −Q) = 0, slerp ignores the sign, dist = axis–angle of RᵀR′',
      worstCover < 1e-9 && worstAngle < 1e-7 && worstRound < 1e-7 && worstInterp < 1e-9,
      `dist(Q,−Q) ${worstCover.toExponential(2)}; |dist − θ(RᵀR′)| ${worstAngle.toExponential(2)}; round-trip ${worstRound.toExponential(2)}`,
    );

    // Composition: the quaternion product is the matrix product (E.4 and §3.5.1 agree).
    let worstComp = 0;
    for (let k = 0; k < 200; k++) {
      const a = SO3.sample(rng), b = SO3.sample(rng);
      const Ra = quatToMatrix(a), Rb = quatToMatrix(b), Rab = quatToMatrix(quatMul(a, b));
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
        let s = 0;
        for (let l = 0; l < 3; l++) s += Ra[i][l] * Rb[l][j];
        worstComp = Math.max(worstComp, Math.abs(s - Rab[i][j]));
      }
    }

    // Gimbal lock: at θ = 0 only φ + ψ survives (E.2, E.9); elsewhere the Z-Y-Z inverse round-trips.
    const locked = eulerZYZ(eulerZYZToMatrix(0.7, 0, 1.1));
    const lockOk = !locked.ok && close(s1Dist(locked.sum, 1.8), 0, 1e-9);
    let worstEuler = 0, lockedElsewhere = 0;
    for (let k = 0; k < 300; k++) {
      const phi = rng.uniform(-Math.PI, Math.PI), theta = rng.uniform(0.05, Math.PI - 0.05), psi = rng.uniform(-Math.PI, Math.PI);
      const r = eulerZYZ(eulerZYZToMatrix(phi, theta, psi));
      if (!r.ok) { lockedElsewhere++; continue; }
      worstEuler = Math.max(worstEuler, s1Dist(r.angles[0], phi), Math.abs(r.angles[1] - theta), s1Dist(r.angles[2], psi));
    }
    push(
      'manifold: Euler Z-Y-Z is a chart only off R₃₃ = ±1 — lock reported at θ = 0 with φ + ψ = 1.8 preserved; exact inverse elsewhere',
      lockOk && lockedElsewhere === 0 && worstEuler < 1e-9 && worstComp < 1e-12,
      `locked: ${!locked.ok}, φ+ψ ${locked.ok ? '—' : fmt(locked.sum)}; round-trip error ${worstEuler.toExponential(2)}; quat·quat = R·R to ${worstComp.toExponential(2)}`,
    );

    // Haar uniformity: the rotation angle of a uniform rotation has density (1 − cos θ)/π,
    // so its mean is π/2 + 2/π ≈ 2.2074 and only (0.5 − sin 0.5)/π = 0.65% of rotations lie
    // within 0.5 rad of the identity. Uniform Z-Y-Z Euler angles put three times that mass
    // near the identity (θ small is not rare for them) — the contrast Chapter 11's samplers need.
    const N = 20000;
    const rngH = new Rng(77);
    let sum = 0, nearH = 0;
    for (let k = 0; k < N; k++) {
      const ang = axisAngle(SO3.sample(rngH)).angle;
      sum += ang;
      if (ang < 0.5) nearH++;
    }
    const mean = sum / N;
    const rngE = new Rng(78);
    let nearE = 0;
    for (let k = 0; k < N; k++) {
      const R = eulerZYZToMatrix(rngE.uniform(-Math.PI, Math.PI), rngE.uniform(0, Math.PI), rngE.uniform(-Math.PI, Math.PI));
      if (axisAngle(matrixToQuat(R)).angle < 0.5) nearE++;
    }
    const expectedNear = (0.5 - Math.sin(0.5)) / Math.PI;
    push(
      'manifold: SO(3).sample is Haar-uniform — mean angle π/2 + 2/π = 2.2074, 0.65% within 0.5 rad of I (uniform Euler angles: ~2%)',
      close(mean, HAAR_MEAN_ANGLE, 0.03) && close(nearH / N, expectedNear, 0.003) && nearE / N > 2 * expectedNear,
      `mean angle ${fmt(mean)} vs ${fmt(HAAR_MEAN_ANGLE)}; near-identity fraction ${fmt(nearH / N)} (Haar, expected ${fmt(expectedNear)}) vs ${fmt(nearE / N)} (uniform Euler)`,
    );

    // A quaternion built from axis–angle has that axis–angle back (E.26 ↔ E.21).
    const qa = quatFromAxisAngle([0, 0, 1], 1.2);
    const aa = axisAngle(qa);
    push(
      'manifold: quaternion ↔ axis–angle round trip (E.26, E.21)',
      close(aa.angle, 1.2, 1e-12) && close(aa.axis[2], 1, 1e-12),
      `θ ${fmt(aa.angle)}, n = (${aa.axis.map(fmt).join(', ')})`,
    );
  }

  // -------------------------------------------------------------------------
  // Dφ is chart-independent (Derivation 5) — Choset’s numbers from Example 3.8.1
  // -------------------------------------------------------------------------
  {
    const q: [number, number] = [Math.PI / 4, Math.PI / 2];
    const xdot = dphi(CHOSET_ARM, q, [1, 0]);
    const shift = dphiChartShiftInvariance(CHOSET_ARM, q);
    const cr = chainRuleExample(0.3);
    push(
      'manifold: Dφ_q(1, 0) = (−√2, 0) at q = (π/4, π/2), unchanged under a 2π chart shift; Example C.5.1 chain rule agrees',
      close(xdot[0], -Math.SQRT2, 1e-12) && close(xdot[1], 0, 1e-12) && shift < 1e-12 && close(cr.chainRule, cr.direct, 1e-12),
      `ẋ = (${fmt(xdot[0])}, ${fmt(xdot[1])}), chart-shift change ${shift.toExponential(2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Integration lab: components on the square vs. the torus (§3.4.3)
  // -------------------------------------------------------------------------
  {
    const grid = armRaster(48);
    const rep = componentReport(grid);
    const link = seamConnection(grid, rep);
    // Monotonicity on random rasters: identification can only merge.
    const rng = new Rng(3);
    let violations = 0;
    for (let k = 0; k < 40; k++) {
      const n = 12;
      const occ = new Uint8Array(n * n);
      for (let c = 0; c < n * n; c++) if (rng.next() < 0.35) occ[c] = 1;
      const g = { n1: n, n2: n, occupied: occ };
      if (labelComponents(g, true).count > labelComponents(g, false).count) violations++;
    }
    const limited = componentReport(withJointLimits(grid, 3.0, 3.0));
    push(
      'manifold: Workbench raster on T² — identifying the edges can only merge components, and here it does',
      rep.onTorus <= rep.onSquare && violations === 0 && link !== null && link.dist < link.chartDist && limited.onTorus === limited.onSquare,
      `square ${rep.onSquare} → torus ${rep.onTorus} components; seam pair dist ${link ? fmt(link.dist) : '—'} vs chart ${link ? fmt(link.chartDist) : '—'}; with limits ${limited.onSquare}`,
    );
  }

  return out;
}
