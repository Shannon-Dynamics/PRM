/**
 * Numerical self-checks for Chapter 7 — Potential Functions.
 *
 * Same contract as `lib/__checks__.ts`: the design doc's micro-example to its
 * stated tolerance, then invariants the mathematics guarantees — brushfire is
 * the exact grid metric, wave-front descent costs exactly the BFS distance,
 * the navigation function has one minimum at κ = 10, the lifted torque is the
 * finite-difference gradient of the lifted potential.
 */

import type { CheckResult } from '../__checks__';
import { bodyPointJacobian, jacobian } from '../cspace/jacobian';
import { Scene } from '../collide/scene';
import { edt2dSquared } from '../mapping/edt';
import { Rng } from '../prob/rng';
import { Reach } from '../robots/reach';
import { WORKBENCH } from '../robots/worlds';
import { bfs } from '../search/best-first';
import { GridGraph, cellFromColRow, cellIJ, randomGrid } from '../search/grid';
import { APARTMENT, distanceToSegment, distanceToWalls } from '../sim/world';
import { closestPointOnSegment } from '../collide/polygon';
import { rangeScan } from '../bugs/scan';
import { MICRO_7X3, brushfire, colRow, descendWavefront, doubleBackPointerCells, gridGradient, labelRows, wavefront } from './brushfire';
import { gradientDescent, oneStep } from './descent';
import { Additive, Attractive, Repulsive, finiteDifferenceGradient, vnorm, vsub, type Vec } from './field';
import { LiftedPotential, controlPointJacobian, defaultControlPoints, liftForce } from './lifted';
import {
  FIVE_DISC_GOAL,
  FIVE_DISC_WORLD,
  FIVE_STAR_WORLD,
  NavigationFunction,
  criticalPoints,
  hLambda,
  hLambdaJacobian,
  inFreeSpace,
  inStarFreeSpace,
  starBoundary,
  starNavigationFunction,
  suitableLambda,
} from './navigation';
import { PlanarObstacles, scanLocalMinima } from './planar';

const fmt = (x: number) => (Math.abs(x) < 1e-4 && x !== 0 ? x.toExponential(2) : x.toFixed(6));
const close = (a: number, b: number, tol: number) => Math.abs(a - b) <= tol;

export function runCh07Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // Part A — the 7 × 3 brushfire micro-grid
  // -------------------------------------------------------------------------
  {
    const dist = brushfire(MICRO_7X3, 4);
    const rows = labelRows(dist);
    const expected = [
      [2, 3, 4, 5, 4, 3, 2],
      [1, 2, 3, 4, 3, 2, 1],
      [2, 3, 4, 5, 4, 3, 2],
    ];
    const labelsOk = rows.every((r, j) => r.every((v, i) => v === expected[j][i]));
    const dbl = doubleBackPointerCells(dist)
      .map((c) => colRow(dist, c))
      .sort((a, b) => a[1] - b[1]);
    const dblOk = dbl.length === 3 && dbl.every(([c]) => c === 4);
    const g = gridGradient(dist, cellFromColRow(MICRO_7X3, 5, 3));
    const gradOk = g >= 0 && colRow(dist, g)[0] === 6 && colRow(dist, g)[1] === 3;
    push(
      'ch07 §3 Part A: brushfire labels 1 + Manhattan distance; column 4 has two back pointers; ∇ at (5,3) → (6,3)',
      labelsOk && dblOk && gradOk,
      `rows ${rows.map((r) => r.join(' ')).join(' | ')}; double pointers ${dbl.map(([c, r]) => `(${c},${r})`).join(' ')}`,
    );
  }

  // -------------------------------------------------------------------------
  // Part B — one potential evaluation and one descent step
  // -------------------------------------------------------------------------
  {
    const q: Vec = [0, 0];
    const att = new Attractive([4, 3], 1, 10);
    const rep = new Repulsive(new PlanarObstacles([{ kind: 'disc', c: { x: 0, y: -2 }, r: 1 }]), 1, 2);
    const U = new Additive([att, rep]);
    const ua = att.value(q);
    const ga = att.gradient(q);
    const ur = rep.value(q);
    const gr = rep.gradient(q);
    const u = U.value(q);
    const gu = U.gradient(q);
    const q1 = oneStep(U, q, 0.1);
    const tol = 1e-12;
    const pass =
      close(ua, 12.5, tol) && close(ga[0], -4, tol) && close(ga[1], -3, tol) &&
      close(ur, 0.125, tol) && close(gr[0], 0, tol) && close(gr[1], -0.5, tol) &&
      close(u, 12.625, tol) && close(gu[0], -4, tol) && close(gu[1], -3.5, tol) &&
      close(q1[0], 0.4, tol) && close(q1[1], 0.35, tol);
    push(
      'ch07 §3 Part B: U_att = 12.5, ∇U_att = (−4,−3); U_rep = 0.125, ∇U_rep = (0,−0.5); U = 12.625, ∇U = (−4,−3.5); q(1) = (0.4, 0.35)',
      pass,
      `U = ${u.toFixed(6)}, ∇U = (${gu[0].toFixed(6)}, ${gu[1].toFixed(6)}), q1 = (${q1[0].toFixed(6)}, ${q1[1].toFixed(6)})`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 1 — the blended attractive gradient is continuous and bounded
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(71);
    let worstJump = 0;
    let worstBound = 0;
    for (let k = 0; k < 200; k++) {
      const goal: Vec = [rng.uniform(-3, 3), rng.uniform(-3, 3)];
      const zeta = rng.uniform(0.2, 3);
      const ds = rng.uniform(0.5, 4);
      const att = new Attractive(goal, zeta, ds);
      const th = rng.uniform(0, 2 * Math.PI);
      const dir = [Math.cos(th), Math.sin(th)];
      const qIn: Vec = [goal[0] + dir[0] * ds * (1 - 1e-9), goal[1] + dir[1] * ds * (1 - 1e-9)];
      const qOut: Vec = [goal[0] + dir[0] * ds * (1 + 1e-9), goal[1] + dir[1] * ds * (1 + 1e-9)];
      worstJump = Math.max(worstJump, vnorm(vsub(att.gradient(qIn), att.gradient(qOut))));
      const far: Vec = [goal[0] + rng.uniform(-50, 50), goal[1] + rng.uniform(-50, 50)];
      worstBound = Math.max(worstBound, vnorm(att.gradient(far)) - zeta * ds);
    }
    push('ch07 Derivation 1: ∇U_att is continuous across d* and bounded by ζ d*', worstJump < 1e-6 && worstBound < 1e-9, `seam jump ${fmt(worstJump)}, bound excess ${fmt(worstBound)}`);
  }

  // -------------------------------------------------------------------------
  // Derivation 2 — ∇d_i = (q − c)/‖q − c‖: the analytic gradient of U_rep equals finite differences
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(72);
    const obs = new PlanarObstacles([
      { kind: 'disc', c: { x: 1, y: 1 }, r: 0.6 },
      { kind: 'polygon', poly: [{ x: -2, y: -1 }, { x: -0.5, y: -1.2 }, { x: -0.8, y: 0.4 }, { x: -2.2, y: 0.2 }] },
      { kind: 'segment', s: { x1: 0, y1: -2.5, x2: 2.5, y2: -1.5 } },
    ]);
    const rep = new Repulsive(obs, 0.7, 1.4);
    let worst = 0;
    let tested = 0;
    for (let k = 0; k < 400; k++) {
      const q: Vec = [rng.uniform(-3, 3), rng.uniform(-3, 3)];
      const near = obs.nearest(q);
      // Away from the obstacle boundary and off the Q* seam, where the FD stencil is valid.
      if (near.d < 0.05 || Math.abs(near.d - 1.4) < 0.01 || obs.inside({ x: q[0], y: q[1] })) continue;
      const fd = finiteDifferenceGradient((p) => rep.value(p), q, 1e-6);
      const an = rep.gradient(q);
      worst = Math.max(worst, vnorm(vsub(fd, an)) / Math.max(1, vnorm(an)));
      tested++;
    }
    push('ch07 Derivation 2: analytic ∇U_rep (via ∇d_i = (q − c)/‖q − c‖) matches finite differences', worst < 1e-5 && tested > 200, `${tested} points, worst rel. error ${fmt(worst)}`);
  }

  // -------------------------------------------------------------------------
  // Derivation 3 — two convex discs are enough for a non-goal minimum
  // -------------------------------------------------------------------------
  {
    const goal: Vec = [4, 0];
    const obs = new PlanarObstacles([
      { kind: 'disc', c: { x: 1, y: 0.8 }, r: 0.5 },
      { kind: 'disc', c: { x: 1, y: -0.8 }, r: 0.5 },
    ]);
    const U = new Additive([new Attractive(goal, 1, 10), new Repulsive(obs, 1, 1.5)]);
    const r = gradientDescent(U, [-2, 0.05], { alpha: 0.01, eps: 1e-7, maxSteps: 50000, goal, goalTol: 1e-3 });
    const q = r.path[r.path.length - 1];
    const ok =
      r.outcome.kind === 'localMinimum' && r.outcome.type === 'minimum' && Math.abs(q[1]) < 1e-4 && Math.abs(q[0] - 0.2409) < 1e-3 &&
      Math.abs(r.outcome.eigs[0] - 14.15) < 0.02 && Math.abs(r.outcome.eigs[1] - 16.16) < 0.02 && r.path.length - 1 === 160;
    push(
      'ch07 Derivation 3: descent between two convex discs stops at a non-goal minimum on the axis (Hessian positive definite)',
      ok,
      `${r.outcome.kind}${r.outcome.kind === 'localMinimum' ? ` (${r.outcome.type}, eigs ${r.outcome.eigs.map((e) => e.toFixed(3)).join(', ')})` : ''} at (${q[0].toFixed(4)}, ${q[1].toFixed(4)}) after ${r.path.length - 1} steps`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 4 — brushfire is the exact grid metric; sandwiched around the EDT
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(74);
    let metricOk = true;
    let sandwichOk = true;
    let worstVsEdt = 0;
    let withinOne = 0;
    let total = 0;
    for (let trial = 0; trial < 40; trial++) {
      const nx = 10 + Math.floor(rng.uniform(0, 12));
      const ny = 8 + Math.floor(rng.uniform(0, 10));
      const grid = randomGrid(nx, ny, rng.uniform(0.05, 0.2), rng);
      if (grid.occupied.every((v) => v === 0)) grid.occupied[0] = 1;
      const d4 = brushfire(grid, 4);
      const d8 = brushfire(grid, 8);
      const seed = new Float64Array(nx * ny).fill(1e20);
      for (let k = 0; k < nx * ny; k++) if (grid.occupied[k]) seed[k] = 0;
      const sq = edt2dSquared(seed, nx, ny);
      for (let k = 0; k < nx * ny; k++) {
        if (grid.occupied[k]) continue;
        const [i, j] = cellIJ(grid, k);
        let man = Infinity;
        let cheb = Infinity;
        for (let o = 0; o < nx * ny; o++) {
          if (!grid.occupied[o]) continue;
          const [oi, oj] = cellIJ(grid, o);
          man = Math.min(man, Math.abs(oi - i) + Math.abs(oj - j));
          cheb = Math.min(cheb, Math.max(Math.abs(oi - i), Math.abs(oj - j)));
        }
        if (d4.label[k] - 1 !== man || d8.label[k] - 1 !== cheb) metricOk = false;
        const e = Math.sqrt(sq[k]);
        if (!(cheb <= e + 1e-9 && e <= man + 1e-9)) sandwichOk = false;
        worstVsEdt = Math.max(worstVsEdt, Math.abs(e - cheb), Math.abs(man - e));
        if (Math.abs(e - cheb) < 1 || Math.abs(man - e) < 1) withinOne++;
        total++;
      }
    }
    push(
      'ch07 Derivation 4: brushfire = 1 + exact Manhattan (4-pt) / chessboard (8-pt) distance; chessboard ≤ EDT ≤ Manhattan',
      metricOk && sandwichOk && total === 7020 && Math.abs(worstVsEdt - 2.789) < 0.01 && withinOne / total > 0.99,
      `${total} cells; worst |grid − EDT| ${worstVsEdt.toFixed(3)} cells; ${((100 * withinOne) / total).toFixed(1)}% within one cell of the EDT`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 4 — wave-front descent path cost equals the BFS distance
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(75);
    let ok = true;
    let compared = 0;
    for (let trial = 0; trial < 60 && ok; trial++) {
      const nx = 12 + Math.floor(rng.uniform(0, 10));
      const ny = 9 + Math.floor(rng.uniform(0, 8));
      const conn = rng.next() < 0.5 ? 4 : 8;
      const start = Math.floor(rng.uniform(0, nx * ny));
      const goal = Math.floor(rng.uniform(0, nx * ny));
      if (start === goal) continue;
      const grid = randomGrid(nx, ny, 0.2, rng, [start, goal]);
      const wf = wavefront(grid, goal, conn);
      const graph = new GridGraph(grid, { connectivity: conn, cornerCutting: true });
      const t = bfs(graph, start, goal);
      if (t.status !== 'found') {
        if (wf.label[start] !== 0) ok = false;
        continue;
      }
      const path = descendWavefront(wf, start);
      const steps = path.length - 1;
      if (wf.label[start] - 2 !== t.path.length - 1 || steps !== t.path.length - 1) ok = false;
      for (let i = 1; i < path.length; i++) if (wf.label[path[i]] !== wf.label[path[i - 1]] - 1) ok = false;
      if (path[path.length - 1] !== goal) ok = false;
      compared++;
    }
    push('ch07 Derivation 4: wave-front label − 2 = BFS link length, and "point to a lowest neighbor" descends by exactly 1 per step to the goal', ok && compared > 30, `${compared} start/goal pairs compared against search.bfs`);
  }

  // -------------------------------------------------------------------------
  // §4.3.1 — sensor-based distance
  // -------------------------------------------------------------------------
  {
    // Every ray reads the exact distance along itself, so the smallest local
    // minimum of ρ is never below D(x). How far *above* D it can be depends on
    // what the nearest wall looks like from x. Broadside — the foot of the
    // perpendicular lies on the segment with margin D·tan(π/n) to both ends —
    // the ray nearest the perpendicular (within π/n of it) hits the wall at
    // D/cos δ ≤ D/cos(π/n): the ray quantisation bound. End-on, a zero-thickness
    // wall subtends no angle at all, and a fan of 360 rays can miss its end
    // entirely; then the nearest *visible* thing may be much farther, and no
    // sensor-only estimator can do better. The check asserts the bound where
    // it applies and counts the end-on poses it does not.
    const rng = new Rng(76);
    const nRays = 360;
    const bound = 1 / Math.cos(Math.PI / nRays) - 1;
    let worstBroadside = 0;
    let worstEndOn = 0;
    let neverBelow = true;
    let broadside = 0;
    let endOn = 0;
    let multi = 0;
    for (let k = 0; k < 80; k++) {
      const x = rng.uniform(0.3, 11.7);
      const y = rng.uniform(0.3, 8.7);
      const scan = rangeScan(APARTMENT, x, y, nRays, 6);
      const mins = scanLocalMinima(scan);
      if (mins.length === 0) continue;
      const D = Math.min(...mins.map((m) => m.d));
      const exact = distanceToWalls(APARTMENT, x, y);
      if (D < exact - 1e-9) neverBelow = false;
      if (mins.length >= 2) multi++;
      // Which wall is nearest, and where on it does the perpendicular land?
      let best = APARTMENT.walls[0];
      let bestD = Infinity;
      for (const w of APARTMENT.walls) {
        const d = distanceToSegment(x, y, w);
        if (d < bestD) {
          bestD = d;
          best = w;
        }
      }
      const c = closestPointOnSegment(x, y, best);
      const margin = exact * Math.tan(Math.PI / nRays);
      const toA = Math.hypot(c.x - best.x1, c.y - best.y1);
      const toB = Math.hypot(c.x - best.x2, c.y - best.y2);
      const rel = (D - exact) / Math.max(exact, 1e-9);
      if (toA > margin && toB > margin) {
        broadside++;
        worstBroadside = Math.max(worstBroadside, rel);
      } else {
        endOn++;
        worstEndOn = Math.max(worstEndOn, rel);
      }
    }
    push(
      'ch07 §4.3.1: the smallest local minimum of ρ is never below D(x), and broadside it overestimates D by at most the ray quantisation 1/cos(π/n) − 1',
      neverBelow && worstBroadside <= bound + 1e-12 && broadside === 70 && endOn === 10 && Math.abs(worstEndOn - 0.0828) < 1e-3,
      `${broadside} broadside poses, worst rel. overestimate ${worstBroadside.toExponential(2)} (bound ${bound.toExponential(2)}); ${endOn} end-on poses, worst ${worstEndOn.toFixed(4)} — a thin wall seen end-on is invisible to a finite fan; ${multi} poses saw ≥ 2 obstacles`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 5 — the navigation function: analytic gradient, κ sweep
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(77);
    const nf = new NavigationFunction(FIVE_DISC_WORLD, FIVE_DISC_GOAL, 5);
    let worst = 0;
    let tested = 0;
    for (let k = 0; k < 300; k++) {
      const q: Vec = [rng.uniform(-4.5, 4.5), rng.uniform(-4.5, 4.5)];
      if (!inFreeSpace(FIVE_DISC_WORLD, q)) continue;
      const an = nf.gradient(q);
      const fd = finiteDifferenceGradient((p) => nf.value(p), q, 1e-6);
      worst = Math.max(worst, vnorm(vsub(an, fd)) / Math.max(1e-6, vnorm(an)));
      tested++;
    }
    const range = (() => {
      let lo = Infinity;
      let hi = -Infinity;
      for (let k = 0; k < 2000; k++) {
        const q: Vec = [rng.uniform(-4.5, 4.5), rng.uniform(-4.5, 4.5)];
        if (!inFreeSpace(FIVE_DISC_WORLD, q)) continue;
        const v = nf.value(q);
        lo = Math.min(lo, v);
        hi = Math.max(hi, v);
      }
      return [lo, hi];
    })();
    push('ch07 Derivation 5: ∇φ (quotient rule) matches finite differences; φ ∈ [0, 1] on the free space', worst < 1e-4 && range[0] >= 0 && range[1] <= 1 && tested > 150, `${tested} points, worst rel. error ${fmt(worst)}, φ ∈ [${range[0].toFixed(4)}, ${range[1].toFixed(4)}]`);
  }
  {
    // The κ sweep the prose prints (Choset Figs. 4.15–4.16 for *our* five discs): spurious
    // minima at small κ, exactly one minimum from κ = 7 on. Saddles drop with them —
    // each spurious minimum takes a saddle along, as the Morse count on a disc with
    // five holes requires (minima − saddles = 1 − 5 = −4 throughout).
    const sweep = [2, 3, 4, 6, 7, 8, 10].map((k) => ({ k, ...criticalPoints(FIVE_DISC_WORLD, FIVE_DISC_GOAL, k, { seedsPerAxis: 28 }) }));
    const expectMinima: Record<number, number> = { 2: 3, 3: 2, 4: 2, 6: 2, 7: 1, 8: 1, 10: 1 };
    const ok = sweep.every((r) => r.minima === expectMinima[r.k] && r.maxima === 0 && r.minima - r.saddles === -4);
    push(
      'ch07 Derivation 5: κ sweep on the five-disc world — 3 minima at κ = 2, 2 at κ = 3…6, exactly one from κ = 7; minima − saddles = −4 (Euler characteristic of the disc with five holes)',
      ok,
      sweep.map((r) => `κ=${r.k}: minima=${r.minima} saddles=${r.saddles}`).join('; '),
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 6 — h_λ sends every star boundary onto its model circle, fixes the goal, and is a local diffeomorphism for a suitable λ
  // -------------------------------------------------------------------------
  {
    const kappa = 6;
    const lambda0 = suitableLambda(FIVE_STAR_WORLD, FIVE_DISC_GOAL, kappa);
    const h = hLambda(FIVE_STAR_WORLD, FIVE_DISC_GOAL, kappa, lambda0);
    let worst = 0;
    FIVE_STAR_WORLD.obstacles.forEach((o) => {
      for (const p of starBoundary(o, 64)) {
        const hp = h([p.x, p.y]);
        worst = Math.max(worst, Math.abs(Math.hypot(hp[0] - o.c.x, hp[1] - o.c.y) - o.r));
      }
    });
    const hg = h(FIVE_DISC_GOAL);
    const goalFixed = Math.hypot(hg[0] - FIVE_DISC_GOAL[0], hg[1] - FIVE_DISC_GOAL[1]);
    // det J_h > 0 on a lattice of free points for λ = λ₀ (orientation-preserving local
    // diffeomorphism), and < 0 somewhere for λ = 10⁻⁶ λ₀ — "suitable" is not decoration.
    let minDet = Infinity;
    let minDetTiny = Infinity;
    let lattice = 0;
    const hTiny = hLambda(FIVE_STAR_WORLD, FIVE_DISC_GOAL, kappa, lambda0 * 1e-6);
    const R = FIVE_STAR_WORLD.r0;
    const N = 48;
    for (let a = 0; a < N; a++) {
      for (let b = 0; b < N; b++) {
        const q: Vec = [-R + ((a + 0.5) / N) * 2 * R, -R + ((b + 0.5) / N) * 2 * R];
        if (!inStarFreeSpace(FIVE_STAR_WORLD, q)) continue;
        minDet = Math.min(minDet, hLambdaJacobian(h, q).det);
        minDetTiny = Math.min(minDetTiny, hLambdaJacobian(hTiny, q).det);
        lattice++;
      }
    }
    // The pulled-back gradient (chain rule) agrees with finite differences of φ∘h.
    const star = starNavigationFunction(FIVE_STAR_WORLD, FIVE_DISC_GOAL, kappa, lambda0);
    const rng = new Rng(78);
    let worstGrad = 0;
    let tested = 0;
    for (let k = 0; k < 200; k++) {
      const q: Vec = [rng.uniform(-4, 4), rng.uniform(-4, 4)];
      const b = FIVE_STAR_WORLD.obstacles.map((o) => Math.hypot(q[0] - o.c.x, q[1] - o.c.y) - o.r * (1 + 2.15 * o.amp));
      if (b.some((v) => v < 0.15) || Math.hypot(q[0], q[1]) > 4.2) continue;
      const an = star.gradient(q);
      const fd = finiteDifferenceGradient((p) => star.value(p), q, 1e-5);
      worstGrad = Math.max(worstGrad, vnorm(vsub(an, fd)) / Math.max(1e-6, vnorm(an)));
      tested++;
    }
    push(
      'ch07 Derivation 6: h_λ maps each star boundary onto the circle of radius r_i, fixes the goal, has det J_h > 0 on the free space at λ = λ₀ (and not at 10⁻⁶ λ₀); ∇(φ∘h) by the chain rule matches finite differences',
      worst < 1e-9 && goalFixed < 1e-12 && minDet > 0 && Math.abs(minDet - 0.984) < 0.005 && Math.abs(lambda0 / 2.058e18 - 1) < 0.01 && minDetTiny < 0 && lattice === 1450 && worstGrad < 1e-3 && tested > 60,
      `boundary error ${fmt(worst)}, |h(goal) − goal| ${fmt(goalFixed)}, min det J_h ${minDet.toFixed(3)} at λ₀ = ${lambda0.toExponential(2)} (${minDetTiny.toFixed(2)} at 10⁻⁶ λ₀) over ${lattice} lattice points, ${tested} gradient points worst rel. ${fmt(worstGrad)}`,
    );
  }

  // -------------------------------------------------------------------------
  // §4.7 — virtual work: Example 4.7.1 and the lifted torque on Reach
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(79);
    let worstTorque = 0;
    for (let k = 0; k < 100; k++) {
      const a = { x: rng.uniform(-1, 1), y: rng.uniform(-1, 1) };
      const th = rng.uniform(-Math.PI, Math.PI);
      const f = { x: rng.uniform(-2, 2), y: rng.uniform(-2, 2) };
      const J = bodyPointJacobian(a, th);
      const u = [J[0][0] * f.x + J[1][0] * f.y, J[0][1] * f.x + J[1][1] * f.y, J[0][2] * f.x + J[1][2] * f.y];
      const rx = a.x * Math.cos(th) - a.y * Math.sin(th);
      const ry = a.x * Math.sin(th) + a.y * Math.cos(th);
      const torque = rx * f.y - ry * f.x; // (r × f)_z
      worstTorque = Math.max(worstTorque, Math.abs(u[2] - torque), Math.abs(u[0] - f.x), Math.abs(u[1] - f.y));
    }
    const arm = Reach.workbench();
    let worstJ = 0;
    for (let k = 0; k < 50; k++) {
      const q = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
      const A = controlPointJacobian(arm, q, 1, 1);
      const B = jacobian(arm, q);
      for (let r = 0; r < 2; r++) for (let c = 0; c < 2; c++) worstJ = Math.max(worstJ, Math.abs(A[r][c] - B[r][c]));
    }
    push('ch07 Example 4.7.1: Jᵀf on a rigid body gives (f_x, f_y, (r × f)_z); the control-point Jacobian at the tip is Chapter 4’s jacobian()', worstTorque < 1e-12 && worstJ < 1e-12, `torque error ${fmt(worstTorque)}, Jacobian error ${fmt(worstJ)}`);
  }
  {
    const arm = Reach.workbench();
    const scene = Scene.fromWorld(WORKBENCH);
    const goal = arm.ik({ x: 1.3, y: 0.9 })[0] ?? [0.3, 0.8];
    const rng = new Rng(80);
    let worst = 0;
    let worstFloat = 0;
    let tested = 0;
    for (const floating of [false, true]) {
      const U = new LiftedPotential(arm, scene, defaultControlPoints(arm, { eta: 0.05, qStar: 0.35 }), goal, floating);
      for (let k = 0; k < 120; k++) {
        const q = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
        // Keep the FD stencil off the Q* seams and the collision boundary.
        const terms = U.terms(q);
        const nearSeam = terms.some((t) => {
          if (t.point.role.kind !== 'repulsive') return false;
          const d = U.obstacles.nearest([t.r.x, t.r.y]).d;
          return Math.abs(d - t.point.role.qStar) < 0.01 || d < 0.03;
        });
        if (nearSeam) continue;
        const an = U.gradient(q);
        const fd = finiteDifferenceGradient((p) => U.value(p), q, 1e-6);
        const err = vnorm(vsub(an, fd)) / Math.max(1, vnorm(an));
        if (floating) worstFloat = Math.max(worstFloat, err);
        else worst = Math.max(worst, err);
        tested++;
      }
    }
    // u_tip = Jᵀ f for a unit force at the tip equals Chapter 4's jointTorques — the design doc's reach_field example.
    const q0 = [Math.PI / 4, Math.PI / 2];
    const J = jacobian(arm, q0);
    const u = liftForce(J, { x: 1, y: 0 });
    const expected = [J[0][0], J[0][1]];
    const tipOk = Math.abs(u[0] - expected[0]) < 1e-12 && Math.abs(u[1] - expected[1]) < 1e-12;
    push('ch07 §4.7.3: Σ Jᵀf (lifted torque) equals the finite-difference gradient of the lifted potential, with and without r_float', worst < 1e-5 && worstFloat < 1e-4 && tipOk && tested > 120, `${tested} configurations; worst rel. error ${fmt(worst)} (fixed points), ${fmt(worstFloat)} (with r_float)`);
  }

  return out;
}
