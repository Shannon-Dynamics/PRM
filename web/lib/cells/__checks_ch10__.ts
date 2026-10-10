/**
 * Numerical self-checks for Chapter 10 — Cell Decompositions and Coverage.
 *
 * The design doc's §3 micro-example (12 trapezoids, 13 adjacencies, Euler
 * faces 3; 6 critical points, 7 boustrophedon cells, a Reeb graph with 6
 * nodes and 7 edges; 4 laps in the first cell at 2r = 0.5), and the
 * invariants the mathematics guarantees: the sweep's list L equals a brute-
 * force re-sort at every event; cells tile Q_free exactly; Euler's count and
 * the closed form hold on 200 seeded worlds; midpoint–centroid paths are
 * collision-free; coverage visits every cell, stays in free space, misses no
 * more than its bound and is no longer than Choset's; the raster Morse
 * decomposition agrees with the exact sweep and obeys Euler for every slice
 * whose level sets pass both sides of each hole; the online cycle algorithm
 * re-discovers the sweep's critical points from range data alone; the
 * visibility polygon equals brute-force ray sampling; and the information-
 * graph search clears the two-pocket world with one pursuer, provably cannot
 * clear the three-pocket world with one, and clears it with a static second.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { coveragePath, coverageReport, lapXs, pathLengthBound, polylineInFreeSpace, uncoveredBound } from './coverage';
import { BENCH_FACTS, CELL_BENCH, LIVING_ROOM, LIVING_ROOM_START, WORKBENCH_START, WORKBENCH_TOP } from './examples';
import { LAB_RASTER, coverageLab, randomCellWorld, vertexCount } from './lab';
import { onlinePoints, runOnlineCoverage } from './online';
import { AdjacencyGraph, planOnCells } from './plan';
import {
  ContaminationField,
  PURSUER_START,
  STATIC_PURSUER,
  THREE_POCKETS,
  TWO_POCKETS,
  conservativeRegions,
  edgeCount,
  pocketTour,
  replayPursuit,
  solveInformationGraph,
  visibilityAgreement,
  visibilityPolygon,
  visibleMask,
  visibleMaskByRays,
} from './pursuit';
import { morseEulerCharacteristic, reebFromSweep } from './reeb';
import { freeRaster, rasterCounts, rasterMorse, type SliceKind } from './slices';
import { SlabLocator, adjacencyFaces, boustrophedon, bruteForceL, trapezoidCountConvex, trapezoidal } from './sweep';
import { asWorld, freeArea, isFreePoint, worldBounds, type CellWorld } from './world';
import type { Point2 } from '../sim/world';

/** Lab numbers the prose prints, pinned (length in m to ±0.05; coverage in % to ±0.05; N_cp/N_ce exact; bound to ±0.05). */
export const LAB_EXPECTED: Record<string, { length: number; covered: number; nCp: number; nCe: number; bound: number }> = {
  'living room/offline': { length: 45.86, covered: 99.87, nCp: 6, nCe: 7, bound: 308.9 },
  'living room/cycle': { length: 85.74, covered: 100.0, nCp: 6, nCe: 7, bound: 308.9 },
  'living room/raster': { length: 21.7, covered: 79.66, nCp: 2, nCe: 1, bound: 191.3 },
  'Workbench top/offline': { length: 170.21, covered: 99.98, nCp: 8, nCe: 10, bound: 631.2 },
  'Workbench top/cycle': { length: 264.41, covered: 100.0, nCp: 8, nCe: 10, bound: 631.2 },
  'Workbench top/raster': { length: 212.86, covered: 99.95, nCp: 4, nCe: 5, bound: 495.4 },
  'cell bench/offline': { length: 158.44, covered: 98.7, nCp: 6, nCe: 7, bound: 866.8 },
  'cell bench/cycle': { length: 272.4, covered: 100.0, nCp: 6, nCe: 7, bound: 866.8 },
  'cell bench/raster': { length: 190.09, covered: 86.36, nCp: 4, nCe: 5, bound: 771.5 },
};

const f3 = (x: number) => x.toFixed(3);

/** The f10.4 gallery: slice, centre, N_cp, N_ce at 8 cm pixels. */
export const GALLERY_EXPECTED: [SliceKind, Point2 | undefined, number, number][] = [
  ['vertical', undefined, 6, 7],
  ['radial', { x: 5, y: 3 }, 12, 13],
  ['spike', { x: -3, y: -2.5 }, 6, 7],
  ['squarel', { x: 5, y: 3 }, 15, 16],
  ['brushfire', undefined, 6, 5],
  ['wavefront', { x: 1.5, y: 0.8 }, 6, 7],
];

export function runCh10Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });
  const free = (w: CellWorld) => (p: Point2) => isFreePoint(w, p, 1e-7);

  // ---------------------------------------------------------------------------
  // 1. The §3 micro-example — trapezoidal sweep
  // ---------------------------------------------------------------------------
  {
    const t = trapezoidal(CELL_BENCH);
    let Lbad = 0;
    for (let i = 0; i + 1 < t.events.length; i++) {
      const x = (t.events[i].v.x + t.events[i + 1].v.x) / 2;
      if (bruteForceL(t, x).join() !== t.events[i].Lafter.join()) Lbad++;
    }
    const area = t.cells.reduce((s, c) => s + c.area, 0);
    const ok =
      t.cells.length === BENCH_FACTS.trapezoids &&
      t.adjacency.length === BENCH_FACTS.adjacencies &&
      adjacencyFaces(t) === BENCH_FACTS.eulerFaces &&
      trapezoidCountConvex(CELL_BENCH) === BENCH_FACTS.trapezoids &&
      Lbad === 0 &&
      Math.abs(area - freeArea(CELL_BENCH)) < 1e-9;
    push(
      'ch10 §3: trapezoidal sweep of the bench — 12 cells, 13 adjacencies, faces 3; L = brute force at every event; cells tile Q_free',
      ok,
      `${t.cells.length} cells, ${t.adjacency.length} edges, f = ${adjacencyFaces(t)}, closed form ${trapezoidCountConvex(CELL_BENCH)}, L mismatches ${Lbad}, Σ area ${area.toFixed(6)} vs ${freeArea(CELL_BENCH).toFixed(6)}`,
    );
  }

  // ---------------------------------------------------------------------------
  // 2. The §3 micro-example — boustrophedon, Reeb graph, Morse count, laps
  // ---------------------------------------------------------------------------
  {
    const b = boustrophedon(CELL_BENCH);
    const reeb = reebFromSweep(b);
    const xs = reeb.nodes.map((n) => n.q.x);
    const xsOk = xs.length === BENCH_FACTS.criticalXs.length && xs.every((x, i) => Math.abs(x - BENCH_FACTS.criticalXs[i]) < 1e-12);
    const chi = morseEulerCharacteristic(b);
    const cp = coveragePath(b, BENCH_FACTS.twoR);
    const first = b.cells.find((c) => c.xRange[0] === 0)!;
    const lapAbscissae = lapXs(0, 2, BENCH_FACTS.twoR);
    const xsLapOk = lapAbscissae.length === 4 && [0.25, 0.75, 1.25, 1.75].every((v, i) => Math.abs(lapAbscissae[i] - v) < 1e-12);
    const ok =
      b.cells.length === BENCH_FACTS.boustrophedonCells &&
      reeb.nCriticalPoints === BENCH_FACTS.reebNodes &&
      reeb.nCells === BENCH_FACTS.reebEdges &&
      reeb.eulerCheck(CELL_BENCH.obstacles.length) &&
      reeb.connected() &&
      xsOk &&
      chi === 1 - CELL_BENCH.obstacles.length &&
      first.xRange[1] === 2 &&
      cp.lapsPerCell[first.id] === BENCH_FACTS.firstCellLaps &&
      xsLapOk;
    push(
      'ch10 §3: boustrophedon — 6 critical points at x = 0, 2, 4, 6, 8.6, 10.4; 7 cells; Reeb 6/7 with N_ce = N_cp + N_ob − 1; Morse χ = −1; 4 laps in cell [0, 2]',
      ok,
      `${b.cells.length} cells, Reeb ${reeb.nCriticalPoints}/${reeb.nCells}, xs [${xs.join(', ')}], χ = ${chi}, laps in first cell ${cp.lapsPerCell[first.id]}`,
    );
  }

  // ---------------------------------------------------------------------------
  // 3. Euler's count and the closed form on 200 seeded worlds
  // ---------------------------------------------------------------------------
  {
    const rng = new Rng(10);
    let bad = 0;
    let worstArea = 0;
    for (let k = 0; k < 200; k++) {
      const w = randomCellWorld(rng);
      const t = trapezoidal(w);
      const b = boustrophedon(w);
      const reeb = reebFromSweep(b);
      if (t.cells.length !== trapezoidCountConvex(w)) bad++;
      if (!reeb.eulerCheck(w.obstacles.length) || !reeb.connected()) bad++;
      if (adjacencyFaces(t) !== w.obstacles.length + 1) bad++;
      if (morseEulerCharacteristic(b) !== 1 - w.obstacles.length) bad++;
      worstArea = Math.max(worstArea, Math.abs(b.cells.reduce((s, c) => s + c.area, 0) - freeArea(w)));
    }
    // Non-convex (star-shaped, notched) holes and boundaries: the closed form
    // survives as V + N_ob − 1 trapezoids and V + 2N_ob − 2 adjacencies.
    let starBad = 0;
    let reflexSeen = 0;
    for (let k = 0; k < 100; k++) {
      const w = randomCellWorld(rng, true);
      const t = trapezoidal(w);
      const b = boustrophedon(w);
      const reeb = reebFromSweep(b);
      const V = vertexCount(w);
      if (t.cells.length !== V + w.obstacles.length - 1 || t.adjacency.length !== V + 2 * w.obstacles.length - 2) starBad++;
      if (!reeb.eulerCheck(w.obstacles.length) || morseEulerCharacteristic(b) !== 1 - w.obstacles.length) starBad++;
      if (b.events.some((e) => e.slice === 'birth' && e.owner >= 0) || b.events.some((e) => e.slice === 'death' && e.owner >= 0)) reflexSeen++;
    }
    // Exercises 1 and 3: the notched K, the dragged triangle, the reflex K.
    const withK = (K: Point2[]) => ({ ...CELL_BENCH, obstacles: [CELL_BENCH.obstacles[0], K] });
    const notched = withK([{ x: 6, y: 1 }, { x: 8.6, y: 0.8 }, { x: 7.3, y: 2.4 }, { x: 8.8, y: 4 }, { x: 6.5, y: 4.4 }]);
    const nb = boustrophedon(notched);
    const nKinds = nb.events.filter((e) => e.owner === 1).map((e) => e.slice).join(',');
    const notchedOk = trapezoidal(notched).cells.length === 13 && nb.cells.length === 9 && reebFromSweep(nb).nCriticalPoints === 8 && nKinds === 'split,pass,birth,merge,merge';
    const dragT = { ...CELL_BENCH, obstacles: [[{ x: 2, y: 2 }, { x: 4, y: 1.5 }, { x: 3, y: 1.6 }], CELL_BENCH.obstacles[1]] };
    const reflexK = withK([{ x: 6, y: 1 }, { x: 8, y: 0.8 }, { x: 7.0, y: 2.6 }, { x: 6.5, y: 4.4 }]);
    const rkMerge = boustrophedon(reflexK).events.find((e) => e.owner === 1 && e.slice === 'merge');
    const dragOk =
      trapezoidal(dragT).cells.length === 12 && boustrophedon(dragT).cells.length === 7 &&
      trapezoidal(reflexK).cells.length === 12 && boustrophedon(reflexK).cells.length === 7 && rkMerge?.v.x === 8 && rkMerge?.v.y === 0.8;
    push(
      "ch10: Euler's count N_ce = N_cp + N_ob − 1, trapezoids = (m − 1) + Σ(k_i + 1), faces = N_ob + 1, χ = 1 − N_ob on 200 convex worlds; trapezoids = V + N_ob − 1 on 100 star-shaped worlds; Exercises 1 and 3",
      bad === 0 && worstArea < 1e-8 && starBad === 0 && reflexSeen === 69 && notchedOk && dragOk,
      `${bad} + ${starBad} violations; ${reflexSeen} star worlds have a notch tip that opens or closes a slice; worst |Σ cell area − free area| ${worstArea.toExponential(2)}; notched K ${notchedOk} (${nKinds}); dragged T and reflex K ${dragOk}`,
    );
  }

  // ---------------------------------------------------------------------------
  // 4. Midpoint–centroid paths are collision-free (Derivation 2)
  // ---------------------------------------------------------------------------
  {
    const rng = new Rng(4);
    let planned = 0, bad = 0, samples = 0;
    for (const w of [CELL_BENCH, LIVING_ROOM, WORKBENCH_TOP]) {
      const d = trapezoidal(w);
      const loc = new SlabLocator(d);
      const g = new AdjacencyGraph(d);
      const b = worldBounds(w);
      const draw = (): Point2 => {
        for (;;) {
          const p = { x: rng.uniform(b.minX, b.maxX), y: rng.uniform(b.minY, b.maxY) };
          if (isFreePoint(w, p)) return p;
        }
      };
      for (let k = 0; k < 40; k++) {
        const path = planOnCells(d, draw(), draw(), loc, g);
        if (!path) {
          bad++;
          continue;
        }
        planned++;
        const chk = polylineInFreeSpace(free(w), path.points, 0.01);
        samples += chk.samples;
        if (!chk.ok) bad++;
      }
    }
    push('ch10 Derivation 2: midpoint–centroid paths on trapezoids are collision-free (120 queries, 1 cm samples)', bad === 0, `${planned} planned, ${bad} failures, ${samples} samples`);
  }

  // ---------------------------------------------------------------------------
  // 5–6. Offline coverage: every cell, free space, missed area, length bound
  // ---------------------------------------------------------------------------
  {
    let ok = true;
    const rows: string[] = [];
    for (const [w, twoR, start] of [
      [CELL_BENCH, 0.5, undefined],
      [LIVING_ROOM, 0.44, LIVING_ROOM_START],
      [WORKBENCH_TOP, 0.2, WORKBENCH_START],
    ] as const) {
      for (const d of [boustrophedon(w), trapezoidal(w)]) {
        const cp = coveragePath(d, twoR, { start });
        const everyCell = new Set(cp.order).size === d.cells.length;
        const fr = polylineInFreeSpace(free(w), cp.points, 0.005);
        if (!everyCell || !fr.ok) ok = false;
        rows.push(`${w.name}/${d.mode}: ${d.cells.length} cells visited ${new Set(cp.order).size}, free ${fr.ok}`);
      }
    }
    push('ch10: offline coverage visits every free cell and stays in cl(Q_free) (dense 5 mm check), boustrophedon and trapezoidal', ok, rows.join('; '));
  }
  {
    let ok = true;
    const rows: string[] = [];
    // [world, 2r, start, r·ΣTV bound, boustrophedon length, trapezoidal length, Choset bound, Δ, P_total] — printed in the prose.
    const pins = [
      [CELL_BENCH, 0.5, { x: 1.5, y: 3 }, 6.15, 158.44, 174.4, 866.8, 11.908, 49.955],
      [LIVING_ROOM, 0.44, LIVING_ROOM_START, 2.35, 45.86, 79.13, 308.9, 4.899, 23.434],
      [WORKBENCH_TOP, 0.2, WORKBENCH_START, 1.148, 170.21, 201.64, 631.2, 6.793, 25.76],
    ] as const;
    for (const [w, twoR, start, ub, lb, lt, cb, delta, pTot] of pins) {
      const d = boustrophedon(w);
      const cp = coveragePath(d, twoR, { start });
      const tp = coveragePath(trapezoidal(w), twoR, { start });
      const rep = coverageReport(w, cp.points, twoR / 2, 0.02);
      const bound = uncoveredBound(d, twoR / 2);
      const L = pathLengthBound(w, twoR, d.cells.length);
      // Raster slack: a pixel is judged by its centre.
      const slack = 0.02 * L.pTotal;
      if (!(rep.uncoveredArea <= bound + slack) || !(cp.length <= L.bound) || !(cp.lapLength <= L.delta * Math.ceil(L.delta / twoR) + L.delta * d.cells.length)) ok = false;
      if (!(tp.length > cp.length)) ok = false;
      if (Math.abs(bound - ub) > 0.005 || Math.abs(cp.length - lb) > 0.05 || Math.abs(tp.length - lt) > 0.05 || Math.abs(L.bound - cb) > 0.05) ok = false;
      if (Math.abs(L.delta - delta) > 0.001 || Math.abs(L.pTotal - pTot) > 0.001) ok = false;
      if (w === CELL_BENCH && (cp.laps !== 32 || tp.laps !== 33 || L.maxLaps !== 31 || Math.abs(tp.transitLength - cp.transitLength - 13.87) > 0.05)) ok = false;
      if (w === LIVING_ROOM && tp.lapsPerCell.length !== 29) ok = false;
      if (w === WORKBENCH_TOP && tp.lapsPerCell.length !== 18) ok = false;
      rows.push(`${w.name}: Δ ${f3(L.delta)}, P ${f3(L.pTotal)}; missed ${f3(rep.uncoveredArea)} ≤ ${f3(bound)} m²; boustrophedon ${cp.length.toFixed(2)} < trapezoidal ${tp.length.toFixed(2)} ≤ ${L.bound.toFixed(1)} m`);
    }
    push('ch10: lawnmower missed area ≤ r·Σ TV(floor, ceiling); boustrophedon path shorter than trapezoidal; both ≤ Δ²/2r + 4ΔN_ce + 5P_total; lap length ≤ Δ⌈Δ/2r⌉ + ΔN_ce', ok, rows.join('; '));
  }

  // ---------------------------------------------------------------------------
  // 7. Morse decompositions for variable slices on a raster
  // ---------------------------------------------------------------------------
  {
    const v = rasterCounts(rasterMorse(CELL_BENCH, { kind: 'vertical' }, { cell: 0.05 }));
    const rows = [`h = x @5 cm: N_cp ${v.nCp}, N_ce ${v.nCe}`];
    let ok = v.nCp === 6 && v.nCe === 7;
    // The f10.4 gallery's slices at 8 cm, with the counts the figure and the prose print.
    for (const [kind, center, cp, ce] of GALLERY_EXPECTED) {
      const k = rasterCounts(rasterMorse(CELL_BENCH, { kind, center }, { cell: 0.08 }));
      const euler = kind === 'brushfire' ? k.nCe === k.nCp - 1 : k.nCe === k.nCp + k.nOb - 1;
      if (!euler || k.nCp !== cp || k.nCe !== ce) ok = false;
      rows.push(`${kind}: ${k.nCp}/${k.nCe}`);
    }
    push('ch10: raster Morse decomposition — h = x reproduces 6/7; radial, spike, squarel, wave-front obey N_ce = N_cp + N_ob − 1; brushfire is a tree (N_ce = N_cp − 1); gallery counts pinned', ok, rows.join('; '));
  }

  // ---------------------------------------------------------------------------
  // 8. Online: the cycle algorithm re-discovers the critical points from range data
  // ---------------------------------------------------------------------------
  {
    const res = runOnlineCoverage(CELL_BENCH, { twoR: 0.5, start: { x: 1.5, y: 3 } });
    const offline = reebFromSweep(boustrophedon(CELL_BENCH));
    const kinds = (ks: string[]) => [...ks].sort().join(',');
    let matched = 0;
    for (const n of offline.nodes) {
      if (res.nodes.some((m) => m.kind === n.kind && Math.hypot(m.q.x - n.q.x, m.q.y - n.q.y) < 0.15)) matched++;
    }
    // Convex critical points (split, merge): the ring's minimum ray is parallel to the sweep.
    let worstDeg = 0;
    for (const n of res.nodes) {
      if (n.kind !== 'split' && n.kind !== 'merge') continue;
      const a = Math.abs(Math.sin(n.minRayAngle));
      worstDeg = Math.max(worstDeg, (Math.asin(Math.min(1, a)) * 180) / Math.PI);
    }
    const lr = runOnlineCoverage(LIVING_ROOM, { twoR: 0.44, start: LIVING_ROOM_START });
    let lrDeg = 0;
    for (const n of lr.nodes) {
      if (n.kind !== 'split' && n.kind !== 'merge') continue;
      lrDeg = Math.max(lrDeg, (Math.asin(Math.min(1, Math.abs(Math.sin(n.minRayAngle)))) * 180) / Math.PI);
    }
    const ok =
      lrDeg <= 0.1 &&
      res.failures === 0 &&
      res.nodes.length === 6 &&
      res.reeb.nCells === 7 &&
      res.reeb.eulerCheck(2) &&
      kinds(res.nodes.map((n) => n.kind)) === kinds(offline.nodes.map((n) => n.kind)) &&
      matched === 6 &&
      worstDeg <= 0.01;
    push(
      'ch10 §6.2.4: online cycle algorithm finds the same 6 critical points (within 15 cm) and 7 cells from range data; minimum ray ∥ sweep within 0.01° at every split/merge (0.1° in the sheared living room)',
      ok,
      `${res.nodes.length} nodes (${res.nodes.map((n) => n.kind[0]).join('')}), ${res.reeb.nCells} cells, matched ${matched}/6, worst min-ray tilt ${worstDeg.toFixed(4)}° (living room ${lrDeg.toFixed(3)}°)`,
    );
  }

  // ---------------------------------------------------------------------------
  // 9. The lab: completeness, free space, bound; the raster's miss (the hook)
  // ---------------------------------------------------------------------------
  {
    const rows = coverageLab();
    let ok = true;
    const msg: string[] = [];
    for (const r of rows) {
      const key = `${r.world}/${r.method}`;
      if (r.method !== 'raster' && r.length > r.bound) ok = false;
      if (r.method === 'cycle' && r.covered < 0.9999) ok = false;
      const exp = LAB_EXPECTED[key];
      if (!exp || Math.abs(exp.length - r.length) > 0.05 || Math.abs(exp.covered - 100 * r.covered) > 0.05 || exp.nCp !== r.nCp || exp.nCe !== r.nCe || Math.abs(exp.bound - r.bound) > 0.05) ok = false;
      msg.push(`${key}: ${r.length.toFixed(2)} m, ${(100 * r.covered).toFixed(2)} %, missed ${r.missed.toFixed(2)} m², N_cp ${r.nCp}, N_ce ${r.nCe}, bound ${r.bound.toFixed(1)}`);
    }
    const lrRaster = rows.find((r) => r.world === LIVING_ROOM.name && r.method === 'raster')!;
    if (lrRaster.missed < 1.0) ok = false;
    // The online paths stay in the free space.
    for (const [w, twoR, start] of [
      [LIVING_ROOM, 0.44, LIVING_ROOM_START],
      [WORKBENCH_TOP, 0.2, WORKBENCH_START],
    ] as const) {
      const res = runOnlineCoverage(w, { twoR, start });
      const fr = polylineInFreeSpace(free(w), onlinePoints(res), 0.01);
      if (!fr.ok || res.failures) ok = false;
    }
    push(`ch10 lab: cycle coverage complete on a ${LAB_RASTER * 100} cm raster; offline and cycle within Choset's bound; numbers in the prose reproduced; online paths in free space; the raster scan leaves ≥ 1 m² of the living room bare`, ok, msg.join('; '));
  }

  // ---------------------------------------------------------------------------
  // 10. Visibility polygon = brute-force ray sampling
  // ---------------------------------------------------------------------------
  {
    let worst = 0, samples = 0, n = 0, maskBad = 0, maskPixels = 0;
    const rasters = new Map<CellWorld, ReturnType<typeof freeRaster>>();
    const rng = new Rng(61);
    for (const w of [TWO_POCKETS, THREE_POCKETS, LIVING_ROOM]) {
      const W = asWorld(w);
      const b = worldBounds(w);
      for (let k = 0; k < 8; k++) {
        let p: Point2;
        do p = { x: rng.uniform(b.minX, b.maxX), y: rng.uniform(b.minY, b.maxY) };
        while (!isFreePoint(w, p));
        const vp = visibilityPolygon(W, p);
        const ag = visibilityAgreement(W, vp, 1800);
        // The raster mask from V(x) agrees with one ray cast per pixel.
        const R = rasters.get(w) ?? freeRaster(w, 0.125);
        rasters.set(w, R);
        const fast = visibleMask(W, R, p, undefined, vp);
        const slow = visibleMaskByRays(W, R, p);
        for (let q = 0; q < fast.length; q++) if (R.free[q]) {
          maskPixels++;
          if (fast[q] !== slow[q]) maskBad++;
        }
        worst = Math.max(worst, ag.worst);
        samples += ag.samples;
        n++;
      }
    }
    push(
      'ch10 §6.3: visibility polygon by angular sweep equals brute-force ray sampling (24 points, 1800 bearings each, < 1e-13 m); its raster mask matches per-pixel ray casts',
      worst < 1e-13 && maskBad <= 0.001 * maskPixels,
      `${n} polygons, ${samples} bearings, worst |Δrange| ${worst.toExponential(2)}; mask disagreements ${maskBad} of ${maskPixels} pixels`,
    );
  }

  // ---------------------------------------------------------------------------
  // 11. Pursuit/evasion: one pursuer, two pockets; three pockets need two
  // ---------------------------------------------------------------------------
  {
    const two = solveInformationGraph(conservativeRegions(TWO_POCKETS, 0.125), PURSUER_START);
    const twoReplay = two.found ? replayPursuit(TWO_POCKETS, two.path, null).area : Infinity;
    const threeOne = solveInformationGraph(conservativeRegions(THREE_POCKETS, 0.125), PURSUER_START);
    const decS = conservativeRegions(THREE_POCKETS, 0.125, STATIC_PURSUER);
    const threeTwo = solveInformationGraph(decS, PURSUER_START);
    const threeReplay = threeTwo.found ? replayPursuit(THREE_POCKETS, threeTwo.path, STATIC_PURSUER).area : Infinity;
    // The naive patrol the widget replays when G_i has no solution.
    const f3p = new ContaminationField(THREE_POCKETS, 0.125);
    const naive = replayPursuit(THREE_POCKETS, pocketTour(THREE_POCKETS, f3p.raster, PURSUER_START), null).area;
    const pinned = two.statesExplored === 286 && threeOne.statesExplored === 693 && threeTwo.statesExplored === 2480 && edgeCount(TWO_POCKETS) === 20 && edgeCount(THREE_POCKETS) === 28;
    const ok = two.found && twoReplay === 0 && !threeOne.found && threeTwo.found && threeReplay === 0 && Math.abs(naive - 10.25) < 0.01 && pinned;
    push(
      'ch10 §6.3: information-graph search clears the two-pocket world with one pursuer; exhausts G_i without clearing three pockets; clears them with a static second pursuer (replays leave 0 m² contaminated)',
      ok,
      `two: ${two.found} (${two.statesExplored} states, replay ${twoReplay}); three/1: ${threeOne.found} (${threeOne.statesExplored} states exhausted); three/2: ${threeTwo.found} (${threeTwo.statesExplored} states, replay ${threeReplay}); naive patrol leaves ${naive.toFixed(2)} m²`,
    );
  }

  return out;
}
