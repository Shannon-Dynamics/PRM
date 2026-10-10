/**
 * Numerical self-checks for Chapter 9 — Roadmaps II.
 *
 * The design doc's micro-example (two point sites, Rusty at (2.5, 1), α = 1,
 * β = −1: G, DG, (DG)†, v, q̇, Γ and Γ̇ = −0.7918 to 10⁻⁴) and the Lyapunov
 * box's example (Γ never increases along the law and the robot lands on the
 * bisector); homing onto the slab's meet point; the GVG of a synthetic room
 * in R³ (every edge sample three-equidistant, every meet point
 * four-equidistant, every edge transversal); the HGVG repair of Fig. 5.19
 * (the GVG has two components, the sheet carries a period around the box,
 * the projected-gradient descent reaches the halo, and the halo is a GVG
 * cycle); the sensor-based exploration of the Apartment (every meet point
 * three-equidistant by exact geometry, every reachable exact meet point
 * found); Canny's silhouette (the sphere's is the equator to 10⁻⁶, the
 * torus's one slice curve becomes two at λ = R − r, and the silhouette
 * curves of the torus and the drilled ellipsoid are connected only through
 * the linking curves); the OPP (0 ∈ ∂D̃ at every freeway point, every
 * freeway point is a tie — Problem 12 — and bridges connect the freeways);
 * and the rod-GVG edge through a Workbench meet point closing over θ.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { APARTMENT } from '../sim/world';
import { WORKBENCH } from '../robots/worlds';
import { SLAB_CLEARANCE, SLAB_MEET, SLAB_SITES } from './examples';
import { exactGvd, siteDistances } from './gvd';
import { exploreGvg } from './explore';
import {
  MICRO_Q,
  MICRO_SITES,
  cannyBoundLog10,
  constraint,
  homeLaw,
  homeToMeetPoint,
  integrateTracer,
  lyapunovMicroExample,
  pointSitesQuery,
  traceLaw,
  transversalityMargin,
  vdist,
  type NearestQuery,
  type Vec,
} from './gvg';
import { FIG_5_19_BOX_CENTER, FIG_5_19_HALO_CLEARANCE, FIG_5_19_INDEX, FIG_5_19_SITES, buildFig519Hgvg, sites3Query } from './hgvg';
import { OPP_WORLD, oppRoadmap } from './opp';
import { rEdgePlacements, rodQuery, rodTieResidual, sampleRodEdge, type RodObstacle } from './rod';
import { ScanQuery, apartmentExactGvd, apartmentExactQuery } from './scan-query';
import { cannySweep, drilledEllipsoid, rankDeficiency, sliceCurves, sphere, torus } from './silhouette';

const fmt = (x: number, d = 4) => (Math.abs(x) < 1e-12 ? (0).toFixed(d) : x.toFixed(d));
const close = (a: number, b: number, tol: number) => Math.abs(a - b) <= tol;

export function runCh09Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // §3 micro-example — one Lyapunov tracing step
  // -------------------------------------------------------------------------
  {
    const e = lyapunovMicroExample(1, -1);
    const T = 1e-4;
    const ok =
      close(e.d1, 2.6926, T) && close(e.d2, 1.8028, T) && close(e.G, 0.8898, T) &&
      close(e.DG[0], 1.7605, T) && close(e.DG[1], -0.1833, T) &&
      close(e.DGdag[0], 0.5619, T) && close(e.DGdag[1], -0.0585, T) &&
      close(e.v[0], 0.1036, T) && close(e.v[1], 0.9946, T) &&
      close(e.correction[0], -0.5, T) && close(e.correction[1], 0.0521, T) &&
      close(e.qdot[0], -0.3964, T) && close(e.qdot[1], 1.0467, T) &&
      close(e.gamma, 0.3959, T) && close(e.dGamma, -0.7918, T) &&
      close(e.correction[0], -(MICRO_Q[0] - 2), 1e-12);
    push(
      'ch09 §3 micro-example: sites (0,0),(4,0), q = (2.5,1), α = 1, β = −1 → G = 0.8898, DG = (1.7605, −0.1833), DG† = (0.5619, −0.0585), v = (0.1036, 0.9946), q̇ = (−0.3964, 1.0467), Γ = 0.3959, Γ̇ = −0.7918 (all to 10⁻⁴); the correction\'s x-component is exactly −0.5, minus the offset from the bisector',
      ok,
      `G ${fmt(e.G)} DG (${fmt(e.DG[0])}, ${fmt(e.DG[1])}) DG† (${fmt(e.DGdag[0])}, ${fmt(e.DGdag[1])}) v (${fmt(e.v[0])}, ${fmt(e.v[1])}) corr (${fmt(e.correction[0])}, ${fmt(e.correction[1])}) q̇ (${fmt(e.qdot[0])}, ${fmt(e.qdot[1])}) Γ ${fmt(e.gamma)} Γ̇ ${fmt(e.dGamma)}`,
    );
  }

  // -------------------------------------------------------------------------
  // The Lyapunov box — energy decreases along the law; β > 0 peels away
  // -------------------------------------------------------------------------
  {
    const query = pointSitesQuery(MICRO_SITES);
    const run = integrateTracer(query, MICRO_Q, [0, 1], 1, -1, 0.01, 200);
    let monotone = true;
    for (let k = 1; k < run.gamma.length; k++) if (run.gamma[k] > run.gamma[k - 1] + 1e-12) monotone = false;
    // Γ̇ = β G² = 2βΓ, so Γ(t) = Γ₀ e^{2βt}: with β = −1, after t = 2 s, Γ ≈ Γ₀ e^{−4}.
    const predicted = run.gamma[0] * Math.exp(-4);
    const ratio = run.gamma[run.gamma.length - 1] / predicted;
    // Run long enough to land: 200 steps of 0.05 s (t = 10 s, e^{−10} ≈ 4.5·10⁻⁵ of Γ₀).
    const long = integrateTracer(query, MICRO_Q, [0, 1], 1, -1, 0.05, 200);
    let monotone2 = true;
    for (let k = 1; k < long.gamma.length; k++) if (long.gamma[k] > long.gamma[k - 1] + 1e-12) monotone2 = false;
    const end = long.path[long.path.length - 1];
    const bad = integrateTracer(query, MICRO_Q, [0, 1], 1, +1, 0.01, 60);
    const grows = bad.gamma[bad.gamma.length - 1] > 3 * bad.gamma[0]; // Γ₀ e^{2·0.6} ≈ 3.3 Γ₀
    // Mixed strategy: the law needs G and DG only; evaluate it from a scan-like query with no site identities.
    const anonymous: NearestQuery = (q) => query(q).map((n) => ({ ...n, index: -1 }));
    const st = traceLaw(anonymous(MICRO_Q), 1, -1, [0, 1]);
    push(
      'ch09 Lyapunov box: along q̇ = αv + β(DG)†G with β = −1, Γ = ½G² is monotone non-increasing (200 Euler steps of 0.01 s and of 0.05 s) and decays like e^{2βt} (ratio to e^{−4}Γ₀ within 3 % at t = 2 s); after 10 s the robot is on the bisector x = 2 to 10⁻³ and has advanced along it; β = +1 makes Γ grow; the law evaluates identically from anonymous (sensor-style) readings',
      monotone && monotone2 && Math.abs(end[0] - 2) < 1e-3 && end[1] > 5 && Math.abs(ratio - 1) < 0.03 && grows && close(st.dGamma, -0.7918, 1e-4),
      `Γ: ${fmt(run.gamma[0])} → ${run.gamma[run.gamma.length - 1].toExponential(3)} at t = 2 (ratio to e^{−4}Γ₀ ${fmt(ratio, 3)}); t = 10: Γ ${long.gamma[long.gamma.length - 1].toExponential(2)}, end (${fmt(end[0])}, ${fmt(end[1])}); β = +1: Γ ${fmt(bad.gamma[0])} → ${fmt(bad.gamma[bad.gamma.length - 1])}`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 4 — homing onto the slab's meet point from a Newton seed
  // -------------------------------------------------------------------------
  {
    const query: NearestQuery = (q) => siteDistances(SLAB_SITES, { x: q[0], y: q[1] }).map((s) => ({ d: s.d, grad: [s.grad.x, s.grad.y], index: s.index }));
    const seeds: Vec[] = [[2.4, 1.1], [1.6, 1.9], [2.2, 0.6]];
    let worst = 0;
    let allOk = true;
    const details: string[] = [];
    for (const s of seeds) {
      const h = homeToMeetPoint(query, s);
      if (!h) {
        allOk = false;
        continue;
      }
      worst = Math.max(worst, vdist(h.q, [SLAB_MEET.x, SLAB_MEET.y]), Math.abs(h.near[0].d - SLAB_CLEARANCE));
      details.push(`(${fmt(s[0], 2)}, ${fmt(s[1], 2)}) → (${fmt(h.q[0], 6)}, ${fmt(h.q[1], 6)})`);
    }
    // Choset's geometry: near the meet point the homing velocity points at the centre of the circle
    // through the three closest points. Linearizing d_i about q is exact for the wall and first-order
    // for the pegs, so the aim sharpens as q approaches the meet point.
    const aim = (q: Vec) => {
      const near = query(q);
      const st = homeLaw(near, -1, 2);
      const toCenter = [2 - q[0], 1.5 - q[1]];
      return (st.qdot[0] * toCenter[0] + st.qdot[1] * toCenter[1]) / (Math.hypot(st.qdot[0], st.qdot[1]) * Math.hypot(toCenter[0], toCenter[1]));
    };
    const far = aim([2.4, 1.1]);
    const nearAim = aim([2.04, 1.46]);
    push(
      'ch09 Derivation 4: homing with G = [d₁−d₂, d₁−d₃] converges from three seeds to the slab\'s meet point (2, 1.5), clearance 2.5, to 10⁻⁹; the Newton-like step aims at the meet point to within 45° from 0.57 m away and to within 3° from 0.06 m away (the linearization of the pegs\' distances sharpens as q approaches)',
      allOk && worst < 1e-9 && far > Math.cos((45 * Math.PI) / 180) && nearAim > Math.cos((3 * Math.PI) / 180),
      `${details.join('; ')}; worst error ${worst.toExponential(2)}; aim cos ${fmt(far, 4)} far, ${fmt(nearAim, 5)} near`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 1 — the GVG of a room in R³: 1-D edges, 4-equidistant meet points, transversal
  // -------------------------------------------------------------------------
  const hg = buildFig519Hgvg();
  {
    const q3 = sites3Query(FIG_5_19_SITES);
    const ex = hg.outer;
    let worstTie = 0;
    let worstMeet = 0;
    let minMargin = Infinity;
    let samples = 0;
    for (const e of ex.edges) {
      if (e.to < 0) continue;
      for (let k = 2; k + 2 < e.polyline.length; k++) {
        const near = q3(e.polyline[k]);
        const { G, DG } = constraint(near, 3);
        worstTie = Math.max(worstTie, ...G.map(Math.abs));
        minMargin = Math.min(minMargin, transversalityMargin(DG));
        samples++;
      }
    }
    const meets = ex.meetPoints;
    for (const m of meets) {
      const near = q3(m.q);
      worstMeet = Math.max(worstMeet, near[3].d - near[0].d);
    }
    const degrees = meets.map((m) => m.edges.length);
    const boundary = ex.nodes.filter((n) => n.kind === 'boundary').length;
    const expected = [
      [1.5, 1.5, 1.5],
      [8.5, 1.5, 1.5],
      [8.5, 6.5, 1.5],
      [1.5, 6.5, 1.5],
    ];
    const found = expected.every((p) => meets.some((m) => vdist(m.q, p) < 1e-6));
    push(
      'ch09 Derivation 1 (GVG in R³, 10×8×3 room): exploration from one start finds the 4 meet points (1.5,1.5,1.5), (8.5,1.5,1.5), (8.5,6.5,1.5), (1.5,6.5,1.5) — each four-equidistant to 10⁻⁹ and of degree 4 — and 8 corner spokes ending at boundary points; every edge sample is three-equidistant to 10⁻⁶ and the sheets meet transversally (smallest singular value of DG ≥ 0.87 = √(3 − √5), the rectangle edges\' value)',
      ex.phase === 'done' && meets.length === 4 && found && worstMeet < 1e-9 && degrees.every((d) => d === 4) && boundary === 8 && worstTie < 1e-6 && minMargin > 0.87,
      `${ex.phase}; ${meets.length} meet points (degrees ${degrees.join(',')}), ${boundary} boundary points, ${ex.edges.length} edges, ${samples} samples; worst tie ${worstTie.toExponential(2)}, worst 4-tie ${worstMeet.toExponential(2)}, min margin ${fmt(minMargin, 4)}; driven ${fmt(ex.distanceDriven, 1)} m`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 2 — the HGVG repair on Fig. 5.19
  // -------------------------------------------------------------------------
  {
    const q3 = sites3Query(FIG_5_19_SITES);
    // The outer network never reaches the halo: no node of it is 1.5 from the box.
    const touchesBox = hg.outer.nodes.some((n) => q3(n.q).slice(0, 3).some((x) => x.index === FIG_5_19_INDEX.box));
    const period = hg.period;
    const link = hg.link;
    const halo = hg.halo;
    let haloOk = false;
    let haloDetail = 'no halo';
    if (halo && link) {
      const end = link.path[link.path.length - 1];
      const near = q3(end);
      const trio = new Set(near.slice(0, 3).map((n) => n.index));
      const tied = trio.has(FIG_5_19_INDEX.floor) && trio.has(FIG_5_19_INDEX.ceiling) && trio.has(FIG_5_19_INDEX.box);
      const r = Math.hypot(end[0] - FIG_5_19_BOX_CENTER[0], end[1] - FIG_5_19_BOX_CENTER[1]);
      haloOk = link.stop === 'gvg' && tied && Math.abs(near[0].d - FIG_5_19_HALO_CLEARANCE) < 1e-6 && halo.closed && halo.worstTie < 1e-6 && Math.abs(end[2] - 1.5) < 1e-9;
      haloDetail = `descent ${link.stop} in ${link.path.length} steps to (${fmt(end[0], 3)}, ${fmt(end[1], 3)}, ${fmt(end[2], 3)}), d = ${fmt(near[0].d, 6)}, ${fmt(r, 3)} from the box axis; halo ${halo.closed ? 'closes' : 'open'} after ${fmt(halo.length, 2)} m, worst tie ${halo.worstTie.toExponential(2)}`;
    }
    push(
      'ch09 Derivation 2 (HGVG, Fig. 5.19): the outer GVG never touches the box; on the floor–ceiling sheet z = 1.5 the second-order edges whose tied pair includes the box form a closed period around it; descending −π∇d_box from the period reaches d_box = d_floor = d_ceiling = 1.5 — the halo — which traces as a closed GVG cycle, three-equidistant to 10⁻⁶',
      !touchesBox && period.closes && haloOk && hg.sheet.explorer.phase === 'done',
      `sheet: ${hg.sheet.explorer.phase}, ${hg.sheet.explorer.edges.length} second-order edges, ${hg.sheet.explorer.meetPoints.length} second-order meet points; period: ${period.edges.length} edges, largest angular gap ${fmt(period.maxGap, 3)} rad; ${haloDetail}`,
    );
  }

  // -------------------------------------------------------------------------
  // §5.3.3 on the Apartment — sensor-based exploration vs. the exact GVD
  // -------------------------------------------------------------------------
  {
    const sensor = new ScanQuery(APARTMENT, { nRays: 360, maxRange: 6 });
    const ex = exploreGvg(sensor.query, [1.0, 1.2], { alpha: 1, beta: -3, dt: 0.03, meetTol: 0.08, boundaryTol: 0.12, nodeTol: 0.015, maxTicks: 120000, homeTol: 1e-4 });
    const exact = apartmentExactQuery();
    let worstTie = 0;
    for (const m of ex.meetPoints) {
      const near = exact(m.q);
      worstTie = Math.max(worstTie, near[2].d - near[0].d);
    }
    const gvd = apartmentExactGvd(APARTMENT, 0.03, 36);
    const truth = gvd.meetPoints;
    const tol = 0.05;
    const matched = truth.filter((t) => ex.meetPoints.some((m) => Math.hypot(m.q[0] - t.q.x, m.q[1] - t.q.y) < tol));
    const extra = ex.meetPoints.filter((m) => !truth.some((t) => Math.hypot(m.q[0] - t.q.x, m.q[1] - t.q.y) < tol));
    const missed = truth.filter((t) => !matched.includes(t));
    push(
      'ch09 §5.3.3 (Apartment): Rusty\'s 360-ray ring (bearings refined between rays) builds the GVG from (1.0, 1.2) to completion; every meet point it homes onto is three-equidistant by exact wall geometry to 10⁻⁶; all 26 meet points of Chapter 8\'s exact GVD are found within 5 cm, and the two the exact tracer\'s seeding missed — (6.00, 4.30) in the corridor and (7.85, 6.79) in the bedroom — are genuine (same exact-geometry test)',
      ex.phase === 'done' && worstTie < 1e-6 && missed.length === 0 && truth.length === 26 && extra.length === 2 && extra.every((m) => Math.hypot(m.q[0] - 6, m.q[1] - 4.3) < 0.02 || Math.hypot(m.q[0] - 7.85, m.q[1] - 6.79) < 0.02),
      `${ex.phase} after ${ex.ticks} ticks, ${fmt(ex.distanceDriven, 1)} m driven; ${ex.meetPoints.length} meet points, ${ex.nodes.filter((n) => n.kind === 'boundary').length} boundary points, ${ex.edges.length} edges (${fmt(ex.roadmapLength, 1)} m); worst exact 3-tie ${worstTie.toExponential(2)}; exact GVD ${truth.length} meet points, ${matched.length} matched, ${missed.length} missed${missed.length ? ` (${missed.slice(0, 4).map((t) => `(${t.q.x.toFixed(2)},${t.q.y.toFixed(2)})`).join(' ')})` : ''}, ${extra.length} extra${extra.length ? ` (${extra.slice(0, 4).map((m) => `(${m.q[0].toFixed(2)},${m.q[1].toFixed(2)})`).join(' ')})` : ''}`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 5 — Canny's silhouette: sphere, torus, drilled ellipsoid
  // -------------------------------------------------------------------------
  {
    const sp = sphere(1);
    const sil = cannySweep(sp, 60, { step: 0.03, seeds: 10 });
    let worstQ3 = 0;
    let worstRank = 0;
    let n = 0;
    for (const c of sil.curves) for (const p of c.points) {
      worstQ3 = Math.max(worstQ3, Math.abs(p[2]));
      worstRank = Math.max(worstRank, rankDeficiency(sp, p));
      n++;
    }
    // Lemma 5.5.2 by hand: rows [2q₁, 2q₂, 2q₃], [1,0,0], [0,1,0] — |det| = 2|q₃|.
    const off = rankDeficiency(sp, [0, 0.6, 0.8]);
    push(
      'ch09 Derivation 5 (sphere): the silhouette of x² + y² + z² = 1 swept along q₁ is the equator — every silhouette sample has |q₃| < 10⁻⁶ and D(f, π₁₂) is rank-deficient there (|det| = 2|q₃| < 10⁻⁶), while off the equator |det| = 2|q₃| = 1.6 at (0, 0.6, 0.8); no critical slice besides birth and death',
      n > 50 && worstQ3 < 1e-6 && worstRank < 1e-6 && close(off, 1.6, 1e-12) && sil.critical.length === 2 && sil.curves.length === 2,
      `${sil.curves.length} silhouette curves, ${n} samples, worst |q₃| ${worstQ3.toExponential(2)}, worst |det| ${worstRank.toExponential(2)}; ${sil.critical.length} critical slices at λ = ${sil.critical.map((c) => fmt(c.lambda, 3)).join(', ')}`,
    );
  }
  {
    const R = 2;
    const r = 0.7;
    const to = torus(R, r);
    const before = sliceCurves(to, R - r - 0.05, { step: 0.03, seeds: 12 });
    const after = sliceCurves(to, R - r + 0.05, { step: 0.03, seeds: 12 });
    const sil = cannySweep(to, 90, { step: 0.03, seeds: 12 });
    const interior = sil.critical.filter((c) => Math.abs(c.lambda) < R);
    const atRminusR = interior.every((c) => Math.abs(Math.abs(c.lambda) - (R - r)) < 0.02);
    push(
      'ch09 Derivation 5 (torus R = 2, r = 0.7): the slice q₁ = λ meets the torus in two curves for |λ| < R − r and one for |λ| > R − r (C Fig. 5.31); the sweep finds the two interior critical slices at λ = ±(R − r) = ±1.3 to 2 cm and the birth/death slices at ±2.7; the silhouette is four arcs (upper and lower halves of the outer and inner equators) that only the critical slices\' curves connect: 4 components → 1',
      before.pieces.length === 2 && after.pieces.length === 1 && before.extrema.length === 4 && after.extrema.length === 2 && interior.length === 2 && atRminusR && sil.critical.length === 4 && sil.curves.length === 4 && sil.componentsWithoutLinks === 4 && sil.componentsWithLinks === 1,
      `pieces ${before.pieces.length} → ${after.pieces.length} (extrema ${before.extrema.length} → ${after.extrema.length}); critical λ: ${sil.critical.map((c) => fmt(c.lambda, 3)).join(', ')}; ${sil.curves.length} curves; components ${sil.componentsWithoutLinks} → ${sil.componentsWithLinks}`,
    );
  }
  {
    const de = drilledEllipsoid();
    const sil = cannySweep(de, 110, { step: 0.03, seeds: 14 });
    const interior = sil.critical.filter((c) => Math.abs(c.lambda) < 2.9);
    // Where the slice curve count changes: the hole punching through (1 → 2) and closing (2 → 1).
    const punch = interior.filter((c) => c.before[0] !== c.after[0]);
    const nearEnds = punch.every((c) => Math.abs(Math.abs(c.lambda - 0.6) - 0.7) < 0.03);
    const dimples = interior.filter((c) => c.before[0] === c.after[0]);
    const inHole = (sl: { pieces: unknown[]; extrema: unknown[] }) => sl.pieces.length === 2 && sl.extrema.length === 6;
    const mid = sliceCurves(de, 0.6, { step: 0.03, seeds: 14 });
    push(
      'ch09 Derivation 5 (ellipsoid with a bent hole): the slice curve count changes exactly twice, where the hole punches through at λ = 0.6 − 0.7 and closes at 0.6 + 0.7 (to 3 cm); midway through the hole the slice is two curves with six extrema — the outer extremum, the bent wall\'s extremum and the rim\'s two corners on one side, two on the other; the fillet adds a dimple critical slice just outside each punch-through; the silhouette curves are disconnected on their own and connected once the critical slices\' curves are added',
      punch.length === 2 && nearEnds && dimples.length <= 2 && dimples.every((c) => punch.some((p) => Math.abs(p.lambda - c.lambda) < 0.05)) && inHole(mid) && sil.componentsWithoutLinks > 1 && sil.componentsWithLinks === 1,
      `critical λ: ${sil.critical.map((c) => `${fmt(c.lambda, 3)} (${c.before.join('/')}→${c.after.join('/')})`).join(', ')}; ${sil.curves.length} silhouette curves; components ${sil.componentsWithoutLinks} → ${sil.componentsWithLinks}`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 6 — OPP: generalized gradient, Problem 12, bridges
  // -------------------------------------------------------------------------
  {
    const rm = oppRoadmap(OPP_WORLD, 100);
    let n = 0;
    let gz = 0;
    let ties = 0;
    for (const sl of rm.slices) for (const fp of sl) {
      n++;
      if (fp.generalizedGradientZero) gz++;
      if (fp.active.length >= 2) ties++;
    }
    push(
      'ch09 Derivation 6 (OPP): at every slice-restricted maximum of D̃ the generalized gradient contains 0 (the active slopes bracket zero); every maximum is a tie of two or more obstacles, so OPP ⊂ GVD (Problem 12, convex obstacles); the channel count changes at 4 interesting critical values, and the bridge curves there plus the bifurcation bridges where a freeway ends in free space connect the freeways into one roadmap',
      n > 150 && gz === n && ties === n && rm.critical.length === 4 && rm.componentsWithoutBridges > 1 && rm.componentsWithBridges === 1,
      `${n} freeway points, ${gz} with 0 ∈ ∂D̃, ${ties} ties; ${rm.freeways.length} freeways; critical λ: ${rm.critical.map((c) => `${fmt(c.lambda, 3)} (${c.before}→${c.after})`).join(', ')}; ${rm.bridges.length} bridges; components ${rm.componentsWithoutBridges} → ${rm.componentsWithBridges}`,
    );
  }

  // -------------------------------------------------------------------------
  // §5.4 — a rod-GVG edge through a Workbench meet point
  // -------------------------------------------------------------------------
  {
    const polys = WORKBENCH.obstacles.map((o) => o.poly);
    const walls = WORKBENCH.walls;
    const sites = [...polys.map((poly) => ({ kind: 'polygon' as const, poly })), ...walls.map((s) => ({ kind: 'segment' as const, s }))];
    const gvd = exactGvd(sites, WORKBENCH.bounds, { step: 0.03, seedsPerAxis: 20 });
    const obstacles: RodObstacle[] = [...polys.map((poly) => ({ kind: 'polygon' as const, poly })), ...walls.map((s) => ({ kind: 'segment' as const, s }))];
    const L = 0.3;
    const query = rodQuery(obstacles, L);
    // The meet point with the largest clearance: the rod has room to spin.
    const mp = [...gvd.meetPoints].sort((a, b) => b.clearance - a.clearance)[0];
    const samples = mp ? sampleRodEdge(query, mp.q, mp.sites, 36) : [];
    const worst = rodTieResidual(query, samples);
    const r = rEdgePlacements(gvd);
    push(
      'ch09 §5.4 (rod-HGVG sketch): a 0.3 m rod spun through 36 orientations about the Workbench\'s widest meet point stays three-way equidistant at every orientation (Newton in (x, y) at fixed θ converges to 10⁻¹⁰) — RF_ijk is a closed curve over θ, footnote 5 — and the R-edge placements sampled from the point-GVD number in the hundreds',
      !!mp && samples.length === 36 && worst < 1e-9 && r.length > 100,
      `meet point (${mp?.q.x.toFixed(3)}, ${mp?.q.y.toFixed(3)}) clearance ${mp?.clearance.toFixed(3)}; ${samples.length}/36 orientations, worst tie ${worst.toExponential(2)}; ${r.length} R-edge placements, ${gvd.edges.length} GVD edges`,
    );
  }

  // -------------------------------------------------------------------------
  // Complexity — Canny's bound as a table
  // -------------------------------------------------------------------------
  {
    const rows = [2, 3, 4, 6].map((n) => cannyBoundLog10(10, n, 2));
    push(
      'ch09 complexity: log₁₀ of Canny\'s bound p^n (log p) w^{n⁴} at p = 10, w = 2 is 7.2, 27.7, 81.5 and 396.4 for n = 2, 3, 4, 6 — singly exponential in the dimension',
      close(rows[0], 7.18, 0.05) && close(rows[1], 27.74, 0.05) && close(rows[2], 81.44, 0.05) && close(rows[3], 396.47, 0.1),
      rows.map((v) => fmt(v, 2)).join(', '),
    );
  }

  return out;
}
