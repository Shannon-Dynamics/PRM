/**
 * Numerical self-checks for Chapter 4 — Configuration Space I.
 *
 * Same contract as `lib/__checks__.ts`: invariants the mathematics guarantees
 * and the design doc's micro-example, never frozen outputs. Every number the
 * chapter prints as a worked example — the five hand-checked cells of Reach's
 * C-obstacle, the 29-cell row, the ±49° extent, arctan(1/4), the five
 * vertices of the triangle-vs-square Minkowski difference and its area 1.42,
 * the inflated square's 3.7854, Choset's Jacobian at (π/4, π/2) — is produced
 * here by the same code the widgets run.
 *
 * Run them with the esbuild one-liner in the brief, or through `npm run check`.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { Reach } from '../robots/reach';
import { convexPolygonsIntersectSAT, distancePolygonPolygon, polygonArea, rect } from '../collide/polygon';
import {
  DOORWAY_ROOM,
  LAB_GOAL,
  LAB_START,
  SLICE_HALF_WIDTH,
  THETA1_EXTENT,
  doorwayRaster,
  reachSquareChecker,
  reachSquareRaster,
  sliceHalfWidthBisection,
  workbenchLabRaster,
} from './examples';
import { Scene } from '../collide/scene';
import { ChainChecker } from '../collide/collision';
import { WORKBENCH } from '../robots/worlds';
import { rasterizeCobstacle, torusChart } from './raster';
import { gjkDistance } from './gjk';
import {
  CHOSET_JACOBIAN_EXAMPLE,
  bodyPointFk,
  bodyPointJacobian,
  detJacobian,
  finiteDifferenceJacobian,
  jacobian2,
  tipVelocity,
  velocityEllipse,
} from './jacobian';
import { applicableContacts, contactTable, polygonIntersectsByContacts, randomConvexPolygon, transformPolygon } from './polygon';
import { STAR_PRESET, inflateDisc, inflatedArea, minkowskiBruteForce, samePolygon, starAlgorithm } from './star';

const d2r = (d: number) => (d * Math.PI) / 180;
const fmt = (x: number, d = 4) => x.toFixed(d);
const exp = (x: number) => x.toExponential(2);

export function runCh04Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // §3.2.2 — the micro-example: Reach and the square [1.4, 1.6] × [−0.1, 0.1]
  // -------------------------------------------------------------------------

  {
    // (a)–(e) of the design doc, by hand and by the exact segment test.
    const c = reachSquareChecker();
    const a = !c.isFree([0, 0]);
    const b = !c.isFree([0, d2r(13)]);
    const cc = c.isFree([0, d2r(15)]);
    const d = !c.isFree([Math.PI / 4, -Math.PI / 2]);
    let e = true;
    for (let k = 0; k < 36; k++) if (!c.isFree([Math.PI / 2, -Math.PI + (k * 2 * Math.PI) / 36])) e = false;
    const tip = Reach.choset().tip([Math.PI / 4, -Math.PI / 2]);
    const nearest = Math.hypot(1.4 - 0, 0.1 - 1);
    push(
      'cspace: the five hand-checked cells of the Reach/square C-obstacle',
      a && b && cc && d && e && Math.abs(tip.x - Math.SQRT2) < 1e-12 && Math.abs(tip.y) < 1e-12 && nearest > 1,
      `(0°,0°) ${a ? 'collision' : 'free'}; (0°,13°) ${b ? 'collision' : 'free'}; (0°,15°) ${cc ? 'free' : 'collision'}; (45°,−90°) ${d ? 'collision' : 'free'} tip (${fmt(tip.x)}, ${fmt(tip.y)}); (90°,*) ${e ? 'free' : 'collision'} nearest ${fmt(nearest, 3)} > L₂`,
    );
  }

  const grid = reachSquareRaster(1);
  const row0 = Math.round(grid.n1 / 2); // θ₁ = 0° on the 360-cell torus chart

  {
    // The θ₁ = 0° row: 29 cells, exactly θ₂ ∈ {−14°, …, 14°}, and nothing else.
    const n = grid.rowCount(row0);
    let exact = true;
    for (let j = 0; j < grid.n2; j++) {
      const deg = j - row0;
      if (grid.get(row0, j) !== Math.abs(deg) <= 14) exact = false;
    }
    push('cspace: θ₁ = 0° row of the 1° raster has 2·14 + 1 = 29 occupied cells', n === 29 && exact, `${n} cells, θ₂ ∈ [−14°, 14°]: ${exact}`);
  }

  {
    // The slice half-width three ways: hand (arctan 1/4), raster (14° in, 15° out), bisection of the exact test.
    const bis = sliceHalfWidthBisection(60);
    const analytic = Math.atan(0.1 / 0.4);
    push(
      'cspace: slice half-width at θ₁ = 0 is arctan(1/4) = 0.2450 rad = 14.04°, by bisection of the exact test',
      Math.abs(bis - analytic) < 1e-9 && Math.abs(SLICE_HALF_WIDTH - analytic) < 1e-15 && d2r(14) < analytic && analytic < d2r(15),
      `bisection ${bis.toFixed(10)} vs ${analytic.toFixed(10)} (${fmt((analytic * 180) / Math.PI, 2)}°)`,
    );
  }

  {
    // θ₁ extent: rows |θ₁| ≤ 49° occupied, |θ₁| ≥ 50° empty; analytic bound 0.864 rad = 49.5°.
    let lo = Infinity;
    let hi = -Infinity;
    for (let i = 0; i < grid.n1; i++) {
      if (grid.rowCount(i) > 0) {
        lo = Math.min(lo, i - row0);
        hi = Math.max(hi, i - row0);
      }
    }
    const boundDeg = (THETA1_EXTENT * 180) / Math.PI;
    push(
      'cspace: θ₁ extent of the blob is rows −49°…49°, empty from ±50°; analytic bound 0.864 rad = 49.5°',
      lo === -49 && hi === 49 && Math.abs(THETA1_EXTENT - 0.8642) < 5e-4 && 49 < boundDeg && boundDeg < 50 && grid.count() === 2015,
      `rows ${lo}°…${hi}°, bound ${fmt(THETA1_EXTENT)} rad = ${fmt(boundDeg, 2)}°, ${grid.count()} cells of ${grid.n1 * grid.n2}`,
    );
  }

  {
    // Connectivity on the torus, measured before Chapter 5 states it: one blob, no wrap.
    const occ = grid.components(true, [true, true]);
    const free = grid.components(false, [true, true]);
    push('cspace: the C-obstacle is one connected region on T² that does not cross a seam', occ.count === 1 && free.count === 1 && !grid.touchesSeam(), `QO components ${occ.count}, Q_free components ${free.count}, touches seam: ${grid.touchesSeam()}`);
  }

  // -------------------------------------------------------------------------
  // App. F.3 — the star algorithm
  // -------------------------------------------------------------------------

  {
    // The merge-scan equals the hull of all pairwise differences oⱼ − rᵢ on seeded convex pairs.
    const rng = new Rng(4);
    let ok = 0;
    let worstArea = 0;
    const N = 500;
    for (let k = 0; k < N; k++) {
      const R = randomConvexPolygon(rng, 3 + Math.floor(rng.uniform(0, 6)), rng.uniform(0.2, 1));
      const W = randomConvexPolygon(rng, 3 + Math.floor(rng.uniform(0, 6)), rng.uniform(0.3, 1.5), { x: rng.uniform(-2, 2), y: rng.uniform(-2, 2) });
      const th = rng.uniform(-Math.PI, Math.PI);
      const star = starAlgorithm(R, th, W).polygon;
      const brute = minkowskiBruteForce(R, th, W);
      if (samePolygon(star, brute, 1e-9)) ok++;
      worstArea = Math.max(worstArea, Math.abs(polygonArea(star) - polygonArea(brute)));
    }
    push('star: merge-scan equals the brute-force Minkowski difference on 500 seeded convex pairs', ok === N, `${ok}/${N} identical, worst area gap ${exp(worstArea)}`);
  }

  {
    // The design doc's preset: triangle (0,0),(0.2,0),(0,0.2) vs the unit square [1,2]².
    const s = starAlgorithm(STAR_PRESET.robot, 0, STAR_PRESET.obstacle);
    const want = [
      { x: 1, y: 0.8 },
      { x: 2, y: 0.8 },
      { x: 2, y: 2 },
      { x: 0.8, y: 2 },
      { x: 0.8, y: 1 },
    ];
    const same = samePolygon(s.polygon, want, 1e-12);
    const area = polygonArea(s.polygon);
    const rotated = starAlgorithm(STAR_PRESET.robot, Math.PI / 6, STAR_PRESET.obstacle).polygon.length;
    push(
      'star: triangle vs unit square gives (1,0.8),(2,0.8),(2,2),(0.8,2),(0.8,1), area 1.42; seven vertices at θ = 30°',
      same && Math.abs(area - 1.42) < 1e-12 && s.polygon.length === 5 && rotated === 7 && s.steps.length === 7,
      `${s.polygon.length} vertices, area ${fmt(area, 2)}, ${s.steps.length} scan steps; at θ = 30°: ${rotated} vertices`,
    );
  }

  // -------------------------------------------------------------------------
  // §3.2.1 — the disc: inflation
  // -------------------------------------------------------------------------

  {
    // |WO ⊕ B_r| = |WO| + r p + π r² for [1,2]², r = 0.5: 1 + 2 + π/4 = 3.7854; chords converge from inside.
    const sq = rect(1, 1, 2, 2);
    const analytic = inflatedArea(sq, 0.5);
    const a32 = polygonArea(inflateDisc(sq, 0.5, 32));
    const a720 = polygonArea(inflateDisc(sq, 0.5, 720));
    push(
      'inflate: |[1,2]² ⊕ B₀.₅| = 1 + 0.5·4 + π·0.25 = 3.7854, and the chord polygon converges to it from inside',
      Math.abs(analytic - (3 + Math.PI / 4)) < 1e-12 && a32 < a720 && a720 < analytic && analytic - a720 < 1e-3,
      `analytic ${fmt(analytic)}, 32 chords ${fmt(a32)}, 720 chords ${fmt(a720)}`,
    );
  }

  // -------------------------------------------------------------------------
  // App. F.2 — the contact test is the black box, verified
  // -------------------------------------------------------------------------

  {
    // Choset's figure F.7 tables, with numbers: a triangle robot and a square have 3 Type A + 4 Type B
    // applicable contacts; in collision all seven half-plane inequalities hold, separated exactly one fails.
    const robot = [
      { x: 0, y: 0 },
      { x: 0.6, y: 0.1 },
      { x: 0.15, y: 0.5 },
    ];
    const obs = rect(1, 0, 2, 1);
    const ac = applicableContacts(robot, 0, obs);
    const nA = ac.filter((c) => c.kind === 'A').length;
    const nB = ac.filter((c) => c.kind === 'B').length;
    const inTable = contactTable(robot, { x: 0.8, y: 0.3, theta: 0 }, obs);
    const outTable = contactTable(robot, { x: 0.3, y: 0.3, theta: 0 }, obs);
    const failing = outTable.filter((r) => !r.satisfied);
    const f = failing[0];
    // The two tables the prose prints, to three decimals.
    const wantIn = [-0.658, -0.565, -1.236, -0.8, -1.2, -0.7, -0.4];
    const wantOut = [-0.575, -0.233, -1.715, -0.8, -1.7, -0.7, 0.1];
    const tablesMatch = inTable.every((r, k) => Math.abs(r.value - wantIn[k]) < 5e-4) && outTable.every((r, k) => Math.abs(r.value - wantOut[k]) < 5e-4);
    push(
      'contacts: figure F.7 with numbers — 3 Type A + 4 Type B contacts; all 7 hold in collision, exactly one fails apart',
      nA === 3 && nB === 4 && inTable.length === 7 && inTable.every((r) => r.satisfied) && failing.length === 1 && f.contact.kind === 'B' && f.contact.obstacleEdge === 3 && f.contact.robotVertex === 1 && Math.abs(f.value - 0.1) < 1e-12 && tablesMatch,
      `applicable ${nA}A + ${nB}B; at (0.8, 0.3): ${inTable.map((r) => fmt(r.value, 3)).join(', ')}; at (0.3, 0.3): ${outTable.map((r) => fmt(r.value, 3)).join(', ')} — E^W_4, r_2 fails`,
    );
  }

  {
    // black_box_agrees: the half-plane test and Chapter 2's separating-axis test give the same verdict.
    const rng = new Rng(21);
    let bad = 0;
    let hits = 0;
    const N = 5000;
    for (let k = 0; k < N; k++) {
      const R = randomConvexPolygon(rng, 3 + Math.floor(rng.uniform(0, 5)), rng.uniform(0.2, 0.8));
      const W = randomConvexPolygon(rng, 3 + Math.floor(rng.uniform(0, 5)), rng.uniform(0.3, 1.2), { x: rng.uniform(-1, 1), y: rng.uniform(-1, 1) });
      const q = { x: rng.uniform(-2, 2), y: rng.uniform(-2, 2), theta: rng.uniform(-Math.PI, Math.PI) };
      const byContacts = polygonIntersectsByContacts(R, q, W);
      const bySat = convexPolygonsIntersectSAT(transformPolygon(R, q), W);
      if (byContacts !== bySat) bad++;
      if (bySat) hits++;
    }
    push('contacts: half-plane test equals the Chapter 2 separating-axis box on 5,000 seeded poses', bad === 0, `${bad} disagreements, ${hits} collisions among ${N}`);
  }

  // -------------------------------------------------------------------------
  // App. F.5 — GJK
  // -------------------------------------------------------------------------

  {
    const rng = new Rng(9);
    let worst = 0;
    let worstWitness = 0;
    let verdictBad = 0;
    let maxIter = 0;
    let iterBoundOk = true;
    let intersecting = 0;
    const N = 500;
    for (let k = 0; k < N; k++) {
      const A = randomConvexPolygon(rng, 3 + Math.floor(rng.uniform(0, 6)), rng.uniform(0.2, 1));
      const B = randomConvexPolygon(rng, 3 + Math.floor(rng.uniform(0, 6)), rng.uniform(0.2, 1), { x: rng.uniform(-2.5, 2.5), y: rng.uniform(-2.5, 2.5) });
      const g = gjkDistance(A, B);
      const brute = distancePolygonPolygon(A, B);
      worst = Math.max(worst, Math.abs(g.dist - brute.dist));
      worstWitness = Math.max(worstWitness, Math.abs(Math.hypot(g.onA.x - g.onB.x, g.onA.y - g.onB.y) - g.dist));
      if (g.intersecting !== convexPolygonsIntersectSAT(A, B)) verdictBad++;
      if (g.intersecting) intersecting++;
      maxIter = Math.max(maxIter, g.iterations);
      // In exact arithmetic the simplex never revisits a vertex of Z, and Z has at most n_A + n_B vertices.
      if (g.iterations > A.length + B.length) iterBoundOk = false;
    }
    push(
      'gjk: distance equals brute-force polygon distance, witnesses realize it, verdict matches SAT, iterations ≤ n_A + n_B',
      worst < 1e-9 && worstWitness < 1e-9 && verdictBad === 0 && iterBoundOk,
      `worst |d − d_brute| ${exp(worst)}, witness gap ${exp(worstWitness)}, ${intersecting} intersecting pairs, max ${maxIter} iterations`,
    );
  }

  // -------------------------------------------------------------------------
  // §3.8 — the Jacobian and its singularities
  // -------------------------------------------------------------------------

  {
    // Choset's Example 3.8.1: L₁ = L₂ = 1, q = (π/4, π/2), q̇ = (1, 0) → J, det J = 1, ẋ = (−√2, 0).
    const arm = Reach.choset();
    const { q, J: want, xdot: wantX } = CHOSET_JACOBIAN_EXAMPLE;
    const J = jacobian2(arm, q);
    let err = 0;
    for (let r = 0; r < 2; r++) for (let c = 0; c < 2; c++) err = Math.max(err, Math.abs(J[r][c] - want[r][c]));
    const det = detJacobian(arm, q);
    const xdot = tipVelocity(arm, q, [1, 0]);
    push(
      'jacobian: Choset Example 3.8.1 — J = [[−√2, −√2/2],[0, −√2/2]], det J = 1, J·(1,0) = (−√2, 0)',
      err < 1e-12 && Math.abs(det - 1) < 1e-12 && Math.abs(xdot.x - wantX.x) < 1e-12 && Math.abs(xdot.y - wantX.y) < 1e-12,
      `J = [[${fmt(J[0][0])}, ${fmt(J[0][1])}], [${fmt(J[1][0])}, ${fmt(J[1][1])}]], det ${fmt(det)}, ẋ = (${fmt(xdot.x)}, ${fmt(xdot.y)})`,
    );
  }

  {
    // J differentiates φ (central differences), det J = L₁L₂ sin θ₂, and sin θ₂ = 0 is exactly singular.
    const rng = new Rng(11);
    let worstFd = 0;
    let worstDet = 0;
    let worstSing = 0;
    let worstBody = 0;
    for (let k = 0; k < 300; k++) {
      const L = [rng.uniform(0.3, 1.5), rng.uniform(0.3, 1.5)];
      const a = new Reach({ base: { x: rng.uniform(-1, 1), y: rng.uniform(-1, 1) }, lengths: L, linkRadius: 0 });
      const q = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)] as const;
      const Ja = jacobian2(a, q);
      const Jf = finiteDifferenceJacobian(a, q);
      for (let r = 0; r < 2; r++) for (let c = 0; c < 2; c++) worstFd = Math.max(worstFd, Math.abs(Ja[r][c] - Jf[r][c]));
      worstDet = Math.max(worstDet, Math.abs(detJacobian(a, q) - L[0] * L[1] * Math.sin(q[1])));
      worstSing = Math.max(worstSing, Math.abs(detJacobian(a, [q[0], 0])), Math.abs(detJacobian(a, [q[0], Math.PI])));
      // Example 3.8.2: the polygon's point Jacobian against central differences of its φ.
      const r = { x: rng.uniform(-1, 1), y: rng.uniform(-1, 1) };
      const pose = { x: rng.uniform(-1, 1), y: rng.uniform(-1, 1), theta: q[0] };
      const Jb = bodyPointJacobian(r, pose.theta);
      const h = 1e-6;
      const dth = bodyPointFk({ ...pose, theta: pose.theta + h }, r);
      const dthm = bodyPointFk({ ...pose, theta: pose.theta - h }, r);
      worstBody = Math.max(worstBody, Math.abs((dth.x - dthm.x) / (2 * h) - Jb[0][2]), Math.abs((dth.y - dthm.y) / (2 * h) - Jb[1][2]), Math.abs(Jb[0][0] - 1), Math.abs(Jb[1][1] - 1), Math.abs(Jb[0][1]), Math.abs(Jb[1][0]));
    }
    push(
      'jacobian: J equals central differences of φ; det J = L₁L₂ sin θ₂; θ₂ ∈ {0, π} is singular; Example 3.8.2 likewise',
      worstFd < 1e-7 && worstDet < 1e-12 && worstSing < 1e-12 && worstBody < 1e-7,
      `|J − FD| ${exp(worstFd)}, |det − L₁L₂ sin θ₂| ${exp(worstDet)}, |det| at sin θ₂ = 0: ${exp(worstSing)}, body-point ${exp(worstBody)}`,
    );
  }

  {
    // The velocity ellipse at θ₂ = 10°: σ₁σ₂ = |det J| = sin 10°, σ₁² + σ₂² = ‖J‖²_F = 3 + 2 cos 10°, ratio 28.58 (Exercise 4).
    const arm = Reach.choset();
    const e = velocityEllipse(arm, [0.3, d2r(10)]);
    const prod = e.rx * e.ry;
    const sum = e.rx * e.rx + e.ry * e.ry;
    const frob = 3 + 2 * Math.cos(d2r(10));
    const ratio = e.rx / e.ry;
    const e2 = velocityEllipse(arm, [-1.1, d2r(10)]);
    push(
      'jacobian: velocity ellipse at θ₂ = 10° has σ₁σ₂ = sin 10°, σ₁² + σ₂² = 3 + 2cos 10°, σ₁/σ₂ = 28.58, independent of θ₁',
      Math.abs(prod - Math.sin(d2r(10))) < 1e-9 && Math.abs(sum - frob) < 1e-9 && Math.abs(ratio - 28.58) < 0.01 && Math.abs(e2.rx / e2.ry - ratio) < 1e-9,
      `σ₁ ${fmt(e.rx)}, σ₂ ${fmt(e.ry)}, product ${fmt(prod)} vs ${fmt(Math.sin(d2r(10)))}, ratio ${fmt(ratio, 2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // §3.2.1 — w4.4: the doorway room, figure 3.5 with numbers
  // -------------------------------------------------------------------------

  {
    const cell = 0.05;
    const r0 = doorwayRaster(0, cell);
    const r11 = doorwayRaster(0.11, cell);
    const r35 = doorwayRaster(0.35, cell);
    const freeArea = (g: typeof r0) => (g.n1 * g.n2 - g.count()) * cell * cell;
    const c0 = r0.components(false, [false, false]).count;
    const c11 = r11.components(false, [false, false]).count;
    const c35 = r35.components(false, [false, false]).count;
    const crate = inflatedArea(DOORWAY_ROOM.obstacles[0].poly, 0.11);
    push(
      'doorway: Q_free is connected for r = 0 and r = 0.11 (Rusty) and splits in two for r = 0.35; point robot free area 19.60 m²',
      c0 === 1 && c11 === 1 && c35 === 2 && Math.abs(freeArea(r0) - 19.6) < 1e-9 && Math.abs(crate - (0.4 + 0.11 * 2.6 + Math.PI * 0.0121)) < 1e-12,
      `components ${c0}/${c11}/${c35}; free area ${fmt(freeArea(r0), 2)}/${fmt(freeArea(r11), 2)}/${fmt(freeArea(r35), 2)} m²; crate ⊕ B₀.₁₁ = ${fmt(crate, 4)} m²`,
    );
  }

  // -------------------------------------------------------------------------
  // The integration lab, and Exercise 3's band
  // -------------------------------------------------------------------------

  {
    // Two obstacles within link 1's reach cut two bands through every θ₂; two bands cut the torus
    // into two free components, and the lab's start and goal lie in different ones. Without the clamp, one.
    const chart = torusChart(360);
    const g = workbenchLabRaster(1, true);
    const free = g.components(false, chart.periodic);
    const occ = g.components(true, chart.periodic);
    const labelOf = (q: readonly [number, number]) => {
      const [i, j] = chart.cellOf(q[0], q[1]);
      return free.labels[i * chart.n2 + j];
    };
    const ls = labelOf(LAB_START);
    const lg = labelOf(LAB_GOAL);
    const bands: number[] = [];
    for (let i = 0; i < chart.n1; i++) if (g.rowCount(i) === chart.n2) bands.push(Math.round((chart.axis1(i) * 180) / Math.PI));
    const band1 = bands.filter((d) => d < 100);
    const band2 = bands.filter((d) => d >= 100);
    const g0 = workbenchLabRaster(1, false);
    const free0 = g0.components(false, chart.periodic).count;
    const pct = (100 * g.count()) / (chart.n1 * chart.n2);
    push(
      'lab: block + clamp within link 1 cut bands θ₁ ∈ [8°, 47°] and [118°, 152°]; Q_free has 2 components, start and goal in different ones; 1 without the clamp',
      free.count === 2 && occ.count === 3 && ls >= 0 && lg >= 0 && ls !== lg && band1[0] === 8 && band1[band1.length - 1] === 47 && band2[0] === 118 && band2[band2.length - 1] === 152 && free0 === 1 && Math.abs(pct - 33.51) < 0.05,
      `QO components ${occ.count}, Q_free ${free.count} (labels start ${ls}, goal ${lg}); bands ${band1[0]}°…${band1[band1.length - 1]}°, ${band2[0]}°…${band2[band2.length - 1]}°; ${pct.toFixed(2)} % occupied; without clamp Q_free ${free0}`,
    );
  }

  {
    // Exercise 3: the square slid to centre (0.95, 0). Link 1 alone hits it for |θ₁| ≤ arctan(0.1/0.85) = 6.71°,
    // so the 1° raster has 13 fully occupied rows (−6°…6°); 3 cm capsules widen the band to −8°…8°.
    const sq = rect(0.85, -0.1, 1.05, 0.1);
    const scene = Scene.ofWalls([], WORKBENCH.bounds).withObstacle('the square', sq);
    const chart = torusChart(360);
    const rows = (arm: Reach) => {
      const g = rasterizeCobstacle(new ChainChecker(scene, arm), chart);
      const full: number[] = [];
      for (let i = 0; i < chart.n1; i++) if (g.rowCount(i) === chart.n2) full.push(Math.round((chart.axis1(i) * 180) / Math.PI));
      return full;
    };
    const thin = rows(Reach.choset());
    const thick = rows(Reach.workbench());
    const half = (Math.atan(0.1 / 0.85) * 180) / Math.PI;
    const halfThick = half + (Math.asin(0.03 / Math.hypot(0.85, 0.1)) * 180) / Math.PI;
    push(
      'exercise 3: square at (0.95, 0) gives a band |θ₁| ≤ arctan(0.1/0.85) = 6.71° — 13 full rows at 1°, 17 with 3 cm links',
      thin.length === 13 && thin[0] === -6 && thin[12] === 6 && thick.length === 17 && thick[0] === -8 && thick[16] === 8 && 6 < half && half < 7 && 8 < halfThick && halfThick < 9,
      `thin rows ${thin[0]}°…${thin[thin.length - 1]}° (${thin.length}), analytic ${fmt(half, 2)}°; r = 0.03 rows ${thick[0]}°…${thick[thick.length - 1]}° (${thick.length}), analytic ${fmt(halfThick, 2)}°`,
    );
  }

  return out;
}
