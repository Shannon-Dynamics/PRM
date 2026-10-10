/**
 * Numerical self-checks for Chapter 8 — Roadmaps I.
 *
 * The design doc's micro-example (the slab's meet point at (2, 1.5) with
 * clearance 2.5) to its stated tolerance, Choset Table 5.1 reproduced row by
 * row, and the invariants the mathematics guarantees: the rotational sweep
 * equals the brute-force definition of visibility; the visibility graph's
 * shortest path is no longer than any sampled free path and is unchanged by
 * the reduction; gradient ascent lands on SS_ij to machine precision and the
 * landing points lie on the traced GVD; the Apartment's GVD is accessible,
 * connected and departable; loops survive the deformation retraction; and the
 * Morse box's torus example has Euler characteristic zero.
 */

import type { CheckResult } from '../__checks__';
import { randomConvexField } from '../bugs/geometry';
import { brushfire, rasterizeObstacles } from '../potential';
import { Rng } from '../prob/rng';
import { cellAt, cellCenter, cellIJ, freeCellNear, gridFromWorld } from '../search/grid';
import { APARTMENT, type Point2 } from '../sim/world';
import { ISLAND_CENTER, KITCHEN_ISLAND, SLAB_BOUNDS, SLAB_CLEARANCE, SLAB_MEET, SLAB_SITES, TABLE_5_1, apartmentDistanceQuery, apartmentVisibility, apartmentWithIslandQuery, fig58Scene, loopAround, slabObstacles } from './examples';
import { clearance, equidistanceField, exactGvd, siteDistances } from './gvd';
import { RidgeGraph, featureTransform, gridGvdQuery, ridgeMaskFromFeatures } from './grid';
import { circleLevelSet, eulerCharacteristicFromCriticalPoints, isRegularValueOnSamples, torusHeightMorsePoints } from './morse';
import { retract, retractToPoint, retractionHomotopy, windingNumber } from './retract';
import { minClearance, polylineLength } from './roadmap';
import { VisibilityScene, edgeSetName, properCross, strictlyInside } from './visibility';

const fmt = (x: number) => (Math.abs(x) < 1e-4 && x !== 0 ? x.toExponential(2) : x.toFixed(6));

export function runCh08Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // Choset Table 5.1 — the rotational sweep on Fig. 5.8
  // -------------------------------------------------------------------------
  {
    const sc = fig58Scene();
    const v = sc.nodes.findIndex((n) => n.name === 'v');
    const sw = sc.rotationalSweep(v);
    const names = (ids: readonly number[]) => ids.map((id) => sc.edges[id].name);
    const initOk = names(sw.initialS).join() === TABLE_5_1.initialS.join();
    const orderOk = sw.events.map((e) => sc.nodes[e.node].name).join() === TABLE_5_1.order.join();
    const sOk = sw.events.every((e, k) => names(e.S).join() === TABLE_5_1.S[k].join());
    const added = sw.visible.map((id) => sc.nodes[id].name);
    const addedOk = added.join() === TABLE_5_1.added.join();
    const brute = sc.visibleBruteForce(v).map((id) => sc.nodes[id].name).sort().join();
    push(
      'ch08 Table 5.1: initial S = {E4, E2, E8, E6}; events α3 α7 α4 α8 α1 α5 α2 α6; S after each row; edges added (v,v4), (v,v8), (v,v1) — and the brute force agrees',
      initOk && orderOk && sOk && addedOk && brute === [...added].sort().join(),
      `S₀ ${edgeSetName(sc, sw.initialS)}; rows ${sw.events.map((e) => `${sc.nodes[e.node].name}:${edgeSetName(sc, e.S)}`).join(' ')}; added ${added.join(', ')}`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivation 2 — sweep == brute force; Derivation 1 + reduced graph on seeded fields
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(81);
    let mismatches = 0;
    let nodes = 0;
    let fields = 0;
    let reducedDiffers = 0;
    let longerThanSample = 0;
    let samples = 0;
    let edgesFull = 0;
    let edgesReduced = 0;
    const bounds = { minX: 0, minY: 0, maxX: 10, maxY: 8 };
    const start = { x: 0.5, y: 0.5 };
    const goal = { x: 9.5, y: 7.5 };
    for (let t = 0; t < 50; t++) {
      const polys = randomConvexField(rng, bounds, { count: 5, radius: [0.5, 1.3], margin: 0.3, keepFree: [start, goal] }).map((p) => [...p.pts]);
      if (polys.length < 3) continue;
      const scene = new VisibilityScene(polys, [{ p: start, name: 'start' }, { p: goal, name: 'goal' }]);
      for (const n of scene.nodes) {
        const a = scene.rotationalSweep(n.id).visible.slice().sort((x, y) => x - y).join();
        const b = scene.visibleBruteForce(n.id).slice().sort((x, y) => x - y).join();
        if (a !== b) mismatches++;
        nodes++;
      }
      const s = scene.nodes.find((n) => n.name === 'start')!.id;
      const g = scene.nodes.find((n) => n.name === 'goal')!.id;
      const full = scene.graph();
      const red = scene.graph({ reduced: true });
      edgesFull += full.edgeCount;
      edgesReduced += red.edgeCount;
      const pf = full.shortestPath(s, g);
      const pr = red.shortestPath(s, g);
      if (!pf || !pr || Math.abs(pf.length - pr.length) > 1e-9) reducedDiffers++;
      // Random free polylines start → waypoints → goal; none may beat the visibility path.
      const free = (p: Point2) => polys.every((poly) => !strictlyInside(p, poly));
      const segFree = (a: Point2, b: Point2) => polys.every((poly) => poly.every((_, k) => !properCross(a, b, { x1: poly[k].x, y1: poly[k].y, x2: poly[(k + 1) % poly.length].x, y2: poly[(k + 1) % poly.length].y })));
      for (let k = 0; k < 40 && pf; k++) {
        const nWay = 1 + Math.floor(rng.uniform(0, 3));
        const pts: Point2[] = [start];
        for (let w = 0; w < nWay; w++) pts.push({ x: rng.uniform(0, 10), y: rng.uniform(0, 8) });
        pts.push(goal);
        let ok = pts.every(free);
        for (let i = 1; i < pts.length && ok; i++) ok = segFree(pts[i - 1], pts[i]) && free({ x: 0.5 * (pts[i - 1].x + pts[i].x), y: 0.5 * (pts[i - 1].y + pts[i].y) });
        if (!ok) continue;
        samples++;
        if (polylineLength(pts) < pf.length - 1e-9) longerThanSample++;
      }
      fields++;
    }
    push('ch08 Derivation 2: the rotational sweep returns exactly the brute-force visible set at every vertex of seeded convex fields', mismatches === 0 && nodes === 1598 && fields === 50, `${nodes} vertices over ${fields} fields, ${mismatches} mismatches`);
    push(
      'ch08 Derivation 1: the visibility-graph shortest path is no longer than any sampled free polyline, and the reduced graph has the same shortest-path length',
      longerThanSample === 0 && samples === 405 && reducedDiffers === 0 && edgesFull === 8386 && edgesReduced === 3875,
      `${samples} free polylines sampled, ${longerThanSample} shorter than A* on the graph; reduced graph differs on ${reducedDiffers} of ${fields} fields (${edgesReduced} vs ${edgesFull} edges in total)`,
    );
  }

  // -------------------------------------------------------------------------
  // §3 micro-example — the slab's exact GVD
  // -------------------------------------------------------------------------
  const gvd = exactGvd(SLAB_SITES, SLAB_BOUNDS, { step: 0.02, seedsPerAxis: 20 });
  {
    const m = gvd.meetPoints[0];
    const meetOk = gvd.meetPoints.length === 1 && Math.abs(m.q.x - SLAB_MEET.x) < 1e-9 && Math.abs(m.q.y - SLAB_MEET.y) < 1e-9 && Math.abs(m.clearance - SLAB_CLEARANCE) < 1e-9;
    const e01 = gvd.edges.find((e) => e.sites[0] === 0 && e.sites[1] === 1);
    const e02 = gvd.edges.find((e) => e.sites[0] === 0 && e.sites[1] === 2);
    const e12 = gvd.edges.find((e) => e.sites[0] === 1 && e.sites[1] === 2);
    let parab = 0;
    if (e02) for (const p of e02.points) parab = Math.max(parab, Math.abs(p.y - (2 - (p.x * p.x) / 8)));
    if (e12) for (const p of e12.points) parab = Math.max(parab, Math.abs(p.y - (2 - ((p.x - 4) * (p.x - 4)) / 8)));
    let vert = 0;
    let yMax = -Infinity;
    if (e01) for (const p of e01.points) {
      vert = Math.max(vert, Math.abs(p.x - 2));
      yMax = Math.max(yMax, p.y);
    }
    const xRange02 = e02 ? Math.max(...e02.points.map((p) => p.x)) : NaN;
    const xRange12 = e12 ? Math.min(...e12.points.map((p) => p.x)) : NaN;
    const { DG } = equidistanceField(SLAB_SITES, 0, 2);
    const dg = DG(SLAB_MEET);
    const slope = -dg.x / dg.y;
    const chordDot = 1 * 2 + slope * 4; // (1, slope) · (2, 4): the chord from P1 to the wall's foot (2, 4)
    const types = e02 && e01 && e12 ? e02.types.every((t) => t === 'parabola') && e12.types.every((t) => t === 'parabola') && e01.types.every((t) => t === 'line') : false;
    push(
      'ch08 §3: the slab\'s meet point is (2, 1.5) with clearance 2.5; P1|W and P2|W are the parabolas y = 2 − x²/8 and y = 2 − (x−4)²/8 (x ≤ 2, x ≥ 2); P1|P2 is the ray x = 2, y < 1.5; the tangent at the meet point has slope −0.5, perpendicular to the chord P1 → (2, 4)',
      meetOk && parab < 1e-9 && vert < 1e-9 && Math.abs(yMax - 1.5) < 1e-9 && xRange02 <= 2 + 1e-9 && xRange12 >= 2 - 1e-9 && Math.abs(slope + 0.5) < 1e-12 && Math.abs(chordDot) < 1e-12 && types && gvd.edges.length === 3,
      `meet (${m?.q.x.toFixed(9)}, ${m?.q.y.toFixed(9)}) D = ${m?.clearance.toFixed(9)}; parabola residual ${fmt(parab)}; |x − 2| on P1|P2 ${fmt(vert)}, top y ${yMax.toFixed(6)}; slope ${slope.toFixed(6)}, chord·tangent ${fmt(chordDot)}; ${gvd.edges.length} edges`,
    );
  }

  // -------------------------------------------------------------------------
  // §5.2.5 grid construction — brushfire ridges on the slab at 0.25 m
  // -------------------------------------------------------------------------
  {
    const obs = slabObstacles();
    const grid = rasterizeObstacles(obs, SLAB_BOUNDS, 0.25, 0.18);
    const meetY: Record<number, number[]> = {};
    let nearest: Record<number, number> = {};
    let sourcesOk = true;
    for (const conn of [4, 8] as const) {
      const d = brushfire(grid, conn);
      const ridge = new RidgeGraph(d);
      const nc = cellCenter(grid, ridge.nearest(cellAt(grid, SLAB_MEET.x, SLAB_MEET.y)));
      nearest[conn] = Math.hypot(nc.x - SLAB_MEET.x, nc.y - SLAB_MEET.y);
      meetY[conn] = [];
      for (const c of ridge.cells) {
        const [i, j] = cellIJ(grid, c);
        const srcs = new Set<number>();
        for (let dj = -1; dj <= 1; dj++) {
          for (let di = -1; di <= 1; di++) {
            const ii = i + di;
            const jj = j + dj;
            if (ii < 0 || jj < 0 || ii >= grid.nx || jj >= grid.ny) continue;
            const k = jj * grid.nx + ii;
            if (!grid.occupied[k] && d.source[k] >= 0) srcs.add(d.source[k]);
          }
        }
        if (srcs.size >= 3) {
          meetY[conn].push(cellCenter(grid, c).y);
          if (![0, 1, 2].every((s) => srcs.has(s))) sourcesOk = false;
        }
      }
      if (d.nSources !== 3) sourcesOk = false;
    }
    const y4 = meetY[4].reduce((a, b) => a + b, 0) / meetY[4].length;
    const y8 = meetY[8].reduce((a, b) => a + b, 0) / meetY[8].length;
    push(
      'ch08 §3 grid check: at 0.25 m the 8-point brushfire ridge passes within one cell of (2, 1.5) and the 4-point ridge within two; the grid meet cells (three sources among the 8-neighbours) sit at y ≈ 1 (Manhattan) and y ≈ 2 (chessboard), bracketing the Euclidean meet point',
      nearest[4] < 0.5 && nearest[8] < 0.25 * Math.SQRT2 && meetY[4].length > 0 && meetY[8].length > 0 && y4 < 1.5 && y8 > 1.5 && Math.abs(y4 - 1) < 0.3 && Math.abs(y8 - 2) < 0.3 && sourcesOk,
      `nearest ridge cell ${nearest[4].toFixed(3)} m (4-pt), ${nearest[8].toFixed(3)} m (8-pt); grid meet cells at y = ${y4.toFixed(3)} (4-pt, ${meetY[4].length} cells) and y = ${y8.toFixed(3)} (8-pt, ${meetY[8].length} cells)`,
    );
  }

  // -------------------------------------------------------------------------
  // Lemma 5.2.1 — retraction lands on SS_ij, on the traced GVD, and fixes it
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(83);
    const obs = slabObstacles();
    let landed = 0;
    let n = 0;
    let worstRes = 0;
    let worstToGvd = 0;
    let pairOk = true;
    for (let k = 0; k < 120; k++) {
      const q = { x: rng.uniform(-1.8, 5.8), y: rng.uniform(-0.8, 3.8) };
      if (clearance(SLAB_SITES, q) < 0.1) continue;
      const r = retract(obs, q, { step: 0.02, bounds: SLAB_BOUNDS });
      n++;
      if (!r.landed) {
        // The slab is open below the pegs: Lemma 5.2.1 needs a bounded environment, and an
        // ascent that leaves the window is the lemma's hypothesis failing, not its conclusion.
        if (r.stop !== 'bounds') pairOk = false;
        continue;
      }
      landed++;
      worstRes = Math.max(worstRes, r.residual);
      const land = r.path[r.path.length - 1];
      const ds = siteDistances(SLAB_SITES, land);
      if (!r.pair || !new Set(r.pair).has(ds[0].index) || !new Set(r.pair).has(ds[1].index)) pairOk = false;
      let toGvd = Infinity;
      for (const e of gvd.edges) for (const p of e.points) toGvd = Math.min(toGvd, Math.hypot(p.x - land.x, p.y - land.y));
      worstToGvd = Math.max(worstToGvd, toGvd);
    }
    // A point already on the GVD is fixed by the retraction.
    const onGvd = gvd.edges[0].points[Math.floor(gvd.edges[0].points.length / 2)];
    const fixed = retract(obs, onGvd, { step: 0.02, bounds: SLAB_BOUNDS });
    const fixedOk = fixed.landed && fixed.path.length === 1;
    push(
      'ch08 Lemma 5.2.1: gradient ascent of D from random slab points reaches a two-equidistant point (|d_i − d_j| ≈ 10⁻¹⁵) whose tied obstacles are the two nearest, within a trace step of the exact GVD, or leaves the (unbounded) window below the pegs; a point on the GVD is fixed',
      landed === 107 && n === 120 && worstRes < 1e-9 && pairOk && worstToGvd < 0.02 && fixedOk,
      `${landed}/${n} landed (the rest left the open window); worst residual ${fmt(worstRes)}; worst distance to the traced diagram ${fmt(worstToGvd)} m; RM(q) = q on the GVD: ${fixedOk}`,
    );
  }

  // -------------------------------------------------------------------------
  // Derivations 3–4 on the Apartment — accessibility, connectivity, departability
  // -------------------------------------------------------------------------
  // Inflation just over half a cell, so walls lying exactly on the window's edge (the shell) and on
  // cell boundaries (x = 4, x = 8, …) are rasterized; at 0.05 the shell would vanish and the GVD with it.
  const apt = gridFromWorld(APARTMENT, 0.1, 0.051);
  const ft = featureTransform(apt);
  const ridgeMask = ridgeMaskFromFeatures(apt, ft);
  const aptDist = brushfire(apt, 8);
  const ridge = new RidgeGraph(aptDist, ridgeMask);
  {
    const dq = apartmentDistanceQuery();
    const rng = new Rng(84);
    let landed = 0;
    let n = 0;
    let worstRes = 0;
    let within = 0;
    let worstGap = 0;
    for (let k = 0; k < 150; k++) {
      const q = { x: rng.uniform(0.2, 11.8), y: rng.uniform(0.2, 8.8) };
      if (dq.nearest([q.x, q.y]).d < 0.05) continue;
      const r = retract(dq, q, { step: 0.02, bounds: APARTMENT.bounds });
      n++;
      if (!r.landed) continue;
      landed++;
      worstRes = Math.max(worstRes, r.residual);
      const land = r.path[r.path.length - 1];
      const rc = cellCenter(apt, ridge.nearest(cellAt(apt, land.x, land.y)));
      const gap = Math.hypot(rc.x - land.x, rc.y - land.y);
      worstGap = Math.max(worstGap, gap);
      if (gap <= 0.2) within++;
    }
    const comps = ridge.components();
    push(
      'ch08 Derivation 3 (Apartment): gradient ascent from every seeded free point reaches SS_ij inside the walls (bounded environment), and lands within two cells of the 0.1 m grid GVD; the grid GVD is one connected component, like the free space',
      landed === n && n === 145 && worstRes < 1e-9 && within === n && worstGap < 0.15 && comps === 1 && ridge.cells.length === 1592,
      `${landed}/${n} landed, worst residual ${fmt(worstRes)}; ${within}/${n} within 0.2 m of a ridge cell (worst gap ${worstGap.toFixed(3)} m); ridge: ${ridge.cells.length} cells, ${comps} component(s)`,
    );
  }
  {
    // Fifty seeded queries answered on both roadmaps: the visibility path is never longer, the GVD path never less safe.
    const rng = new Rng(85);
    const dq = apartmentDistanceQuery();
    const D = (p: Point2) => dq.nearest([p.x, p.y]).d;
    let answered = 0;
    let tried = 0;
    let vgLonger = 0;
    let gvdLessClear = 0;
    let sumVg = 0;
    let sumGvd = 0;
    let sumVgClear = 0;
    let sumGvdClear = 0;
    for (let k = 0; k < 50; k++) {
      const s = { x: rng.uniform(0.4, 11.6), y: rng.uniform(0.4, 8.6) };
      const g = { x: rng.uniform(0.4, 11.6), y: rng.uniform(0.4, 8.6) };
      if (D(s) < 0.3 || D(g) < 0.3 || Math.hypot(s.x - g.x, s.y - g.y) < 2) continue;
      tried++;
      const q = gridGvdQuery(aptDist, ridge, freeCellNear(apt, s.x, s.y), freeCellNear(apt, g.x, g.y));
      const vs = apartmentVisibility(s, g, 0.22);
      const vgGraph = vs.graph();
      const sId = vs.nodes.find((n) => n.name === 'start')!.id;
      const gId = vs.nodes.find((n) => n.name === 'goal')!.id;
      const vp = vgGraph.shortestPath(sId, gId);
      if (!q || !vp) continue;
      answered++;
      const vgClear = minClearance(vp.points, D);
      const gvdClear = minClearance(q.points, D, 2);
      sumVg += vp.length;
      sumGvd += q.length;
      sumVgClear += vgClear;
      sumGvdClear += gvdClear;
      if (vp.length > q.length + 1e-9) vgLonger++;
      if (gvdClear < vgClear - 0.1) gvdLessClear++;
    }
    push(
      'ch08 Derivation 4 (Apartment): every seeded query is answered on both roadmaps — get on by retraction, ride by A*, get off — the visibility path is never longer than the GVD path, and the GVD path is never materially less clear',
      answered === tried && tried === 29 && vgLonger === 0 && gvdLessClear === 0 &&
        Math.abs(sumVg / answered - 6.68) < 0.02 && Math.abs(sumGvd / answered - 10.7) < 0.02 && Math.abs(sumVgClear / answered - 0.22) < 0.01 && Math.abs(sumGvdClear / answered - 0.48) < 0.01,
      `${answered}/${tried} queries; mean length VG ${(sumVg / answered).toFixed(2)} m vs GVD ${(sumGvd / answered).toFixed(2)} m; mean min clearance VG ${(sumVgClear / answered).toFixed(2)} m vs GVD ${(sumGvdClear / answered).toFixed(2)} m`,
    );
  }

  // -------------------------------------------------------------------------
  // §5.2.3 — a deformation retraction keeps the hole; a retraction to a point does not
  // -------------------------------------------------------------------------
  {
    const dq = apartmentWithIslandQuery();
    const around = loopAround(ISLAND_CENTER, 0.55, 48);
    const H = retractionHomotopy(dq, around, { step: 0.02, bounds: APARTMENT.bounds });
    const w0 = windingNumber(around, ISLAND_CENTER);
    const w1 = windingNumber(H.at(1), ISLAND_CENTER);
    const wHalf = windingNumber(H.at(0.5), ISLAND_CENTER);
    const allLanded = H.results.every((r) => r.landed);
    const openLoop = loopAround({ x: 2, y: 2 }, 0.5, 48); // room A, nothing inside
    const H2 = retractionHomotopy(dq, openLoop, { step: 0.02, bounds: APARTMENT.bounds });
    const w2 = windingNumber(H2.at(1), ISLAND_CENTER);
    const toPoint = retractToPoint(around, { x: 5.0, y: 3.0 }, (p) => !dq.inside(p) && dq.nearest([p.x, p.y]).d > 0);
    const islandInside = (p: Point2) => strictlyInside(p, KITCHEN_ISLAND);
    const h1 = H.at(1);
    const piercesIsland = h1.some(islandInside);
    push(
      'ch08 §5.2.3: a loop around the kitchen island keeps winding number 1 about it at t = 0, ½ and 1 of the gradient-ascent homotopy (every point lands on the GVD, none enters the island); a loop in an empty room has winding 0; the straight-line retraction to a point pierces the island',
      w0 === 1 && w1 === 1 && wHalf === 1 && allLanded && !piercesIsland && w2 === 0 && toPoint.pierces === 14,
      `winding ${w0} → ${wHalf} → ${w1}; ${H.results.filter((r) => r.landed).length}/${around.length} landed; empty-room loop winding ${w2}; retract-to-point pierces ${toPoint.pierces} of ${around.length} rays`,
    );
  }

  // -------------------------------------------------------------------------
  // The Morse box — torus height, Euler characteristic, the preimage warm-up
  // -------------------------------------------------------------------------
  {
    const pts = torusHeightMorsePoints();
    const idx = pts.map((p) => p.index);
    const chi = eulerCharacteristicFromCriticalPoints(pts);
    const circle = circleLevelSet(91538, 128);
    const reg = isRegularValueOnSamples(circle.DG, circle.samples);
    push(
      'ch08 Morse box: cos θ₁ + cos θ₂ on T² has critical points of index 2, 1, 1, 0 (one maximum, two saddles, one minimum), all nondegenerate, with Σ(−1)^index = 0 = χ(T²); 91,538 is a regular value of x² + y² (|Df| = 2√91538 on the circle)',
      idx.join() === '2,1,1,0' && pts.every((p) => p.nondegenerate) && chi === 0 && reg.regular && Math.abs(reg.worstGradient - 2 * Math.sqrt(91538)) < 1e-6,
      `indices ${idx.join(', ')}; χ = ${chi}; |Df| on the level set ${reg.worstGradient.toFixed(4)}`,
    );
  }

  return out;
}
