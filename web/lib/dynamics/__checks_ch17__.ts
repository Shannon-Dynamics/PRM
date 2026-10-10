/**
 * Numerical self-checks for Chapter 17 — Robot Dynamics.
 *
 * The design doc's micro-example is the first pin: Reach with L = 1, 1 m,
 * m = 2, 1 kg, uniform rods, a_g = 9.81, at q = (π/4, π/2) has
 * M = [[2, 1/3], [1/3, 1/3]], g = (10.405, −3.468), C q̇ = (0, 0.5) for
 * q̇ = (1, 0), K = 1 J, V = 17.342 J, and M₁₁ sweeps 3 → 2 → 1 as q₂ goes
 * 0 → π/2 → π. Then the invariants the mathematics guarantees: closed-form
 * Christoffel symbols equal central differences; the Jacobian-assembled arm
 * equals the hand-derived one; Ṁ − 2C is skew; energy is conserved under RK4
 * and not under explicit Euler; inverse ∘ forward is the identity; M is
 * positive definite; the knife-edge KKT solve reproduces Choset's (10.25)–(10.28);
 * the torque-free brick conserves angular momentum and energy while its
 * intermediate-axis spin flips.
 *
 *   npx esbuild lib/dynamics/__checks_ch17__.ts --bundle --platform=node --format=esm \
 *     --outfile=.next/ck17.mjs && node -e "import('./.next/ck17.mjs').then(m=>m.runCh17Checks().forEach(r=>console.log(r.pass?'PASS':'FAIL',r.name,r.detail??'')))"
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import type { Vec } from '../prob/linalg';
import { quatToMatrix, QUAT_IDENTITY } from '../manifold/so3';
import { KnifeEdge, constrainedForward, matrixRank, projectionP, projectionPu } from './constraint';
import { energyDrift, simulate } from './integrate';
import { christoffelAsymmetry, christoffelFromInertia, skewSymmetryDefect, symmetricEigen2 } from './lagrange';
import { coriolis, forward, inverse, isPositiveDefinite, kineticEnergy, totalEnergy } from './model';
import { PlanarArm } from './planar-arm';
import { MICRO_Q, Reach2R } from './reach2r';
import {
  brickInertia,
  eulerBodyStep,
  intermediateAxis,
  rotationalKineticEnergy,
  spatialAngularMomentum,
  type RigidState,
  type Vec3,
} from './rigid';

const close = (a: number, b: number, tol: number) => Math.abs(a - b) <= tol;
const fmt = (x: number, d = 4) => x.toFixed(d);
const fmtV = (v: readonly number[], d = 4) => `[${v.map((x) => x.toFixed(d)).join(', ')}]`;

export function runCh17Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  const arm = Reach2R.micro();

  // -------------------------------------------------------------------------
  // Micro-example (design doc §3)
  // -------------------------------------------------------------------------
  {
    const M = arm.inertia(MICRO_Q);
    const g = arm.gravity(MICRO_Q);
    const qd: Vec = [1, 0];
    const c = arm.velocityProduct(MICRO_Q, qd);
    const K = kineticEnergy(arm, MICRO_Q, qd);
    const V = arm.potentialEnergy(MICRO_Q);
    const g1 = 1.5 * 9.81 * Math.SQRT1_2;
    const g2 = -0.5 * 9.81 * Math.SQRT1_2;
    const okM = close(M[0][0], 2, 1e-9) && close(M[0][1], 1 / 3, 1e-9) && close(M[1][0], 1 / 3, 1e-9) && close(M[1][1], 1 / 3, 1e-9);
    const okG = close(g[0], g1, 1e-9) && close(g[1], g2, 1e-9);
    const okC = close(c[0], 0, 1e-12) && close(c[1], 0.5, 1e-12);
    const okE = close(K, 1, 1e-12) && close(V, 17.3418, 5e-4);
    push(
      'dynamics: micro-example M = [[2, 1/3], [1/3, 1/3]], g = (10.4051, −3.4684), C q̇ = (0, 0.5), K = 1 J, V = 17.342 J at q = (π/4, π/2)',
      okM && okG && okC && okE,
      `M = ${fmtV(M[0])} ${fmtV(M[1])}; g = ${fmtV(g)}; Cq̇ = ${fmtV(c)}; K = ${fmt(K)}; V = ${fmt(V)}`,
    );

    const sweep = [0, Math.PI / 2, Math.PI].map((q2) => arm.inertia([Math.PI / 4, q2])[0][0]);
    const eig = symmetricEigen2(M);
    // Exercise 3's answer: the semi-axes 1/√λ at q₂ = π/2 are 0.696 and 1.928; at q₂ = π they are 0.981 and 1.844.
    const eigPi = symmetricEigen2(arm.inertia([Math.PI / 4, Math.PI]));
    const axesHalf = eig.values.map((l) => 1 / Math.sqrt(l));
    const axesPi = eigPi.values.map((l) => 1 / Math.sqrt(l));
    push(
      'dynamics: unit-energy ellipse semi-axes 1/√λ are (0.696, 1.928) at q₂ = π/2 and (0.981, 1.844) at q₂ = π',
      close(axesHalf[0], 0.696, 5e-4) && close(axesHalf[1], 1.928, 5e-4) && close(axesPi[0], 0.981, 5e-4) && close(axesPi[1], 1.844, 5e-4),
      `π/2: ${fmtV(axesHalf, 3)}; π: ${fmtV(axesPi, 3)}; M(π) = ${fmtV(arm.inertia([Math.PI / 4, Math.PI])[0])}`,
    );
    push(
      'dynamics: M₁₁ sweeps 3 → 2 → 1 as q₂ goes 0 → π/2 → π; det M = 5/9 with eigenvalues 2.0642 and 0.2691',
      close(sweep[0], 3, 1e-9) && close(sweep[1], 2, 1e-9) && close(sweep[2], 1, 1e-9) && close(eig.values[0], 2.0642, 5e-5) && close(eig.values[1], 0.2691, 5e-5),
      `M₁₁ = ${fmtV(sweep)}; λ = ${fmtV(eig.values)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Christoffel: closed form vs. central differences; symmetry; Ṁ − 2C skew
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(17);
    let worst = 0;
    let asym = 0;
    let skew = 0;
    for (let k = 0; k < 50; k++) {
      const q: Vec = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
      const qd: Vec = [rng.uniform(-3, 3), rng.uniform(-3, 3)];
      const closed = arm.christoffel(q);
      const fd = christoffelFromInertia((x) => arm.inertia(x), q);
      for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) for (let l = 0; l < 2; l++) worst = Math.max(worst, Math.abs(closed[i][j][l] - fd[i][j][l]));
      asym = Math.max(asym, christoffelAsymmetry(fd), christoffelAsymmetry(closed));
      skew = Math.max(skew, skewSymmetryDefect((x) => arm.inertia(x), q, qd));
      // The Christoffel C reproduces the velocity-product vector.
      const C = coriolis(arm, q, qd);
      const cv = arm.velocityProduct(q, qd);
      const Cqd = [C[0][0] * qd[0] + C[0][1] * qd[1], C[1][0] * qd[0] + C[1][1] * qd[1]];
      worst = Math.max(worst, Math.abs(Cqd[0] - cv[0]), Math.abs(Cqd[1] - cv[1]));
    }
    push(
      'dynamics: closed-form Christoffel symbols match central differences to 1e-7 and C q̇ reproduces the velocity-product vector',
      worst < 1e-7,
      `worst |Γ_closed − Γ_fd| ${worst.toExponential(2)}`,
    );
    push('dynamics: Γ^i_jk = Γ^i_kj and Ṁ − 2C is skew-symmetric (defect < 1e-7) over 50 seeded states', asym < 1e-12 && skew < 1e-7, `asymmetry ${asym.toExponential(2)}, skew defect ${skew.toExponential(2)}`);
  }

  // -------------------------------------------------------------------------
  // Jacobian-assembled arm equals the hand derivation
  // -------------------------------------------------------------------------
  {
    const assembled = arm.asPlanarArm();
    const rng = new Rng(27);
    let worst = 0;
    for (let k = 0; k < 40; k++) {
      const q: Vec = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
      const qd: Vec = [rng.uniform(-2, 2), rng.uniform(-2, 2)];
      const Ma = assembled.inertia(q);
      const Mb = arm.inertia(q);
      for (let i = 0; i < 2; i++) for (let j = 0; j < 2; j++) worst = Math.max(worst, Math.abs(Ma[i][j] - Mb[i][j]));
      const ga = assembled.gravity(q);
      const gb = arm.gravity(q);
      const ca = assembled.velocityProduct(q, qd);
      const cb = arm.velocityProduct(q, qd);
      for (let i = 0; i < 2; i++) worst = Math.max(worst, Math.abs(ga[i] - gb[i]), Math.abs(ca[i] - cb[i]) / 10);
      worst = Math.max(worst, Math.abs(assembled.potentialEnergy(q) - arm.potentialEnergy(q)));
    }
    push('dynamics: PlanarArm (COM Jacobians) agrees with Reach2R (hand-derived) in M, g, V exactly and in C q̇ to 1e-7', worst < 1e-8, `worst ${worst.toExponential(2)}`);
  }

  // -------------------------------------------------------------------------
  // Energy: RK4 conserves, explicit Euler does not
  // -------------------------------------------------------------------------
  {
    const zero = () => [0, 0];
    const rk4 = simulate(arm, { q: MICRO_Q.slice(), qd: [0, 0] }, zero, 1e-3, 10, 'rk4', 10);
    const euler = simulate(arm, { q: MICRO_Q.slice(), qd: [0, 0] }, zero, 1e-3, 10, 'explicit-euler', 10);
    const semi = simulate(arm, { q: MICRO_Q.slice(), qd: [0, 0] }, zero, 1e-3, 10, 'semi-implicit-euler', 10);
    const dRk4 = energyDrift(rk4);
    const dEuler = energyDrift(euler);
    const dSemi = energyDrift(semi);
    push(
      'dynamics: released from rest at the micro-example pose with τ = 0, E = 17.342 J stays within 1e-6 relative under RK4 (h = 1 ms, 10 s); explicit Euler drifts past 1e-3',
      close(rk4[0].energy, 17.3418, 5e-4) && dRk4 < 1e-6 && dEuler > 0.1 && dSemi > 0.05,
      `E₀ = ${fmt(rk4[0].energy)} J; drift rk4 ${dRk4.toExponential(2)}, explicit Euler ${dEuler.toExponential(2)}, semi-implicit Euler ${dSemi.toExponential(2)}`,
    );

    // Without gravity and damping on a *moving* arm the same holds — this is the Coriolis-off lie made measurable.
    const flat = Reach2R.micro(0);
    const moving = simulate(flat, { q: [0.3, 1.2], qd: [1.5, -2.0] }, zero, 1e-3, 5, 'rk4', 5);
    const liar = {
      ...flat,
      n: 2,
      inertia: (q: Vec) => flat.inertia(q),
      gravity: (q: Vec) => flat.gravity(q),
      dissipation: () => [0, 0],
      potentialEnergy: (q: Vec) => flat.potentialEnergy(q),
      christoffel: (q: Vec) => flat.christoffel(q),
      velocityProduct: () => [0, 0],
    };
    const lied = simulate(liar, { q: [0.3, 1.2], qd: [1.5, -2.0] }, zero, 1e-3, 5, 'rk4', 5);
    push(
      'dynamics: with a_g = 0 and no damping a swinging arm conserves E under RK4 (< 1e-6), and zeroing C q̇ breaks conservation by more than 1%',
      energyDrift(moving) < 1e-6 && energyDrift(lied) > 0.2,
      `honest drift ${energyDrift(moving).toExponential(2)}; Coriolis-off drift ${energyDrift(lied).toExponential(2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // inverse ∘ forward = id; M ≻ 0 on 2R and 3R
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(3);
    const three = PlanarArm.reach3R();
    let worst = 0;
    let pd = true;
    let skew3 = 0;
    for (let k = 0; k < 200; k++) {
      const q2: Vec = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
      const qd2: Vec = [rng.uniform(-3, 3), rng.uniform(-3, 3)];
      const tau: Vec = [rng.uniform(-20, 20), rng.uniform(-10, 10)];
      const back = inverse(arm, q2, qd2, forward(arm, q2, qd2, tau));
      worst = Math.max(worst, Math.abs(back[0] - tau[0]), Math.abs(back[1] - tau[1]));
      pd = pd && isPositiveDefinite(arm.inertia(q2));
      const q3: Vec = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
      const qd3: Vec = [rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3, 3)];
      pd = pd && isPositiveDefinite(three.inertia(q3));
      const tau3: Vec = [rng.uniform(-10, 10), rng.uniform(-10, 10), rng.uniform(-10, 10)];
      const back3 = inverse(three, q3, qd3, forward(three, q3, qd3, tau3));
      for (let i = 0; i < 3; i++) worst = Math.max(worst, Math.abs(back3[i] - tau3[i]));
      if (k < 20) skew3 = Math.max(skew3, skewSymmetryDefect((x) => three.inertia(x), q3, qd3));
    }
    push('dynamics: inverse(forward(q, q̇, τ)) returns τ to 1e-9 on 200 seeded states, for the 2R and the 3R arm', worst < 1e-9, `worst ${worst.toExponential(2)}`);
    push('dynamics: M(q) is positive definite at 200 seeded configurations of each arm; 3R Ṉ − 2C skew defect < 1e-6 (finite-difference Γ)', pd && skew3 < 1e-6, `3R skew defect ${skew3.toExponential(2)}`);
  }

  // -------------------------------------------------------------------------
  // Pfaffian constraints — Choset Example 10.3.1
  // -------------------------------------------------------------------------
  {
    const knife = new KnifeEdge(2, 0.5);
    const rng = new Rng(103);
    let worst = 0;
    let rankOk = true;
    let orthOk = 0;
    for (let k = 0; k < 30; k++) {
      const q: Vec = [rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-Math.PI, Math.PI)];
      // A velocity that satisfies the constraint: along the heading, plus spin.
      const v = rng.uniform(-2, 2);
      const qd: Vec = [v * Math.cos(q[2]), v * Math.sin(q[2]), rng.uniform(-2, 2)];
      const u: Vec = [rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-2, 2)];
      const kkt = constrainedForward(knife, knife.constraint(), q, qd, u);
      const ref = knife.closedForm(q, qd, u);
      for (let i = 0; i < 3; i++) worst = Math.max(worst, Math.abs(kkt.qdd[i] - ref.qdd[i]));
      worst = Math.max(worst, Math.abs(kkt.lambda[0] - ref.lambda));
      const M = knife.inertia();
      const A = knife.constraint().A(q);
      const P = projectionP(M, A);
      const Pu = projectionPu(M, A);
      rankOk = rankOk && matrixRank(P) === 2 && matrixRank(Pu) === 2;
      // P = Pᵀ_u, and the projected motion is M-orthogonal to the constrained one.
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) worst = Math.max(worst, Math.abs(P[i][j] - Pu[j][i]));
      const Pqd = [0, 1, 2].map((i) => P[i][0] * qd[0] + P[i][1] * qd[1] + P[i][2] * qd[2]);
      const rest = [0, 1, 2].map((i) => qd[i] - Pqd[i]);
      let dot = 0;
      for (let i = 0; i < 3; i++) dot += Pqd[i] * M[i][i] * rest[i];
      orthOk = Math.max(orthOk, Math.abs(dot));
      // Choset's P for the knife-edge: [[c², sc, 0], [sc, s², 0], [0, 0, 1]].
      const c = Math.cos(q[2]);
      const s = Math.sin(q[2]);
      worst = Math.max(worst, Math.abs(P[0][0] - c * c), Math.abs(P[0][1] - s * c), Math.abs(P[1][1] - s * s), Math.abs(P[2][2] - 1));
    }
    push(
      'dynamics: knife-edge KKT solve reproduces Choset (10.25)–(10.28) for q̈ and λ, and P = [[c², sc, 0], [sc, s², 0], [0, 0, 1]] has rank n − k = 2 with P = P_uᵀ',
      worst < 1e-9 && rankOk && orthOk < 1e-9,
      `worst ${worst.toExponential(2)}; M-orthogonality ${orthOk.toExponential(2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Euler's equation — torque-free brick
  // -------------------------------------------------------------------------
  {
    const I = brickInertia(1, 0.3, 0.2, 0.1); // Ixx = (0.04+0.01)/12, Iyy = (0.09+0.01)/12, Izz = (0.09+0.04)/12 → x minor, y intermediate, z major
    const mid = intermediateAxis(I);
    const run = (axis: number) => {
      const w0: Vec3 = [0, 0, 0];
      w0[axis] = 5;
      // A 1° perturbation off the axis.
      const other = (axis + 1) % 3;
      w0[other] = 5 * Math.tan(Math.PI / 180);
      let s: RigidState = { R: QUAT_IDENTITY, omega: w0 };
      const L0 = spatialAngularMomentum(s, I);
      const K0 = rotationalKineticEnergy(I, s.omega);
      let worstL = 0;
      let worstK = 0;
      let maxOff = 0;
      let worstOrth = 0;
      for (let k = 0; k < 4000; k++) {
        s = eulerBodyStep(s, [0, 0, 0], I, 2e-3);
        const L = spatialAngularMomentum(s, I);
        worstL = Math.max(worstL, Math.hypot(L[0] - L0[0], L[1] - L0[1], L[2] - L0[2]) / Math.hypot(...L0));
        worstK = Math.max(worstK, Math.abs(rotationalKineticEnergy(I, s.omega) - K0) / K0);
        const off = Math.hypot(...s.omega.map((w, i) => (i === axis ? 0 : w)));
        maxOff = Math.max(maxOff, off / 5);
        const R = quatToMatrix(s.R);
        const d = R[0][0] * R[0][1] + R[1][0] * R[1][1] + R[2][0] * R[2][1];
        worstOrth = Math.max(worstOrth, Math.abs(d));
      }
      return { worstL, worstK, maxOff, worstOrth };
    };
    const major = run(2);
    const inter = run(mid);
    push(
      'dynamics: torque-free brick conserves spatial angular momentum and kinetic energy to 1e-6 over 8 s of RK4 (h = 2 ms) and R stays orthonormal',
      major.worstL < 1e-6 && major.worstK < 1e-6 && inter.worstL < 1e-6 && inter.worstK < 1e-6 && inter.worstOrth < 1e-9,
      `|ΔL|/|L| ${Math.max(major.worstL, inter.worstL).toExponential(2)}, |ΔK|/K ${Math.max(major.worstK, inter.worstK).toExponential(2)}`,
    );
    push(
      'dynamics: a 1° perturbation of spin about the major axis stays below 2°, about the intermediate axis it grows past 45° — the flip',
      major.maxOff < Math.tan((2 * Math.PI) / 180) && inter.maxOff > Math.tan(Math.PI / 4),
      `major-axis max off-axis fraction ${major.maxOff.toFixed(4)}, intermediate ${inter.maxOff.toFixed(3)} (intermediate axis index ${mid})`,
    );
  }

  // Energy bookkeeping consistency: E = K + V equals the integrator's sample energy.
  {
    const e = totalEnergy(arm, MICRO_Q, [1, 0]);
    push('dynamics: E = K + V = 18.342 J at the micro-example state with q̇ = (1, 0)', close(e, 1 + 17.3418, 5e-4), `E = ${fmt(e)} J`);
  }

  return out;
}
