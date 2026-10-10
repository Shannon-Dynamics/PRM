/**
 * The chapter's integration lab, as one function the prose, the widgets and
 * the checks all call: offline boustrophedon coverage (map known), online
 * sensor-based coverage with the cycle algorithm (map unknown), and the
 * conventional raster scan that misses closing critical points — in the
 * living room for disc-Rusty and on the Workbench top for Reach's pad.
 */

import type { Rng } from '../prob/rng';
import { randomConvexPolygon } from '../cspace/polygon';
import { randomConvexField } from '../bugs/geometry';
import type { Point2 } from '../sim/world';
import { coveragePath, coverageReport, pathLengthBound } from './coverage';
import { CELL_BENCH, LIVING_ROOM, LIVING_ROOM_START, RUSTY_RADIUS, WORKBENCH_START, WORKBENCH_TOP } from './examples';
import { onlinePoints, runOnlineCoverage, type Policy } from './online';
import { boustrophedon } from './sweep';
import { cellWorld, isValidWorld, type CellWorld } from './world';

export interface LabRow {
  world: string;
  method: 'offline' | 'cycle' | 'raster';
  twoR: number;
  length: number;
  covered: number;
  missed: number;
  nCp: number;
  nCe: number;
  laps: number;
  bound: number;
}

export interface LabCase {
  world: CellWorld;
  twoR: number;
  start: Point2;
}

/** Rusty's diameter for the living room; a 20 cm wiping pad on the Workbench; 2r = 0.5 on the bench. */
export const LAB_CASES: LabCase[] = [
  { world: LIVING_ROOM, twoR: 2 * RUSTY_RADIUS, start: LIVING_ROOM_START },
  { world: WORKBENCH_TOP, twoR: 0.2, start: WORKBENCH_START },
  { world: CELL_BENCH, twoR: 0.5, start: { x: 1.5, y: 3 } },
];

/** Raster pitch for "covered" — 3 cm, fine against every effector here. */
export const LAB_RASTER = 0.03;

export function runLabCase(c: LabCase, methods: readonly LabRow['method'][] = ['offline', 'cycle', 'raster']): LabRow[] {
  const out: LabRow[] = [];
  const r = c.twoR / 2;
  for (const method of methods) {
    if (method === 'offline') {
      const d = boustrophedon(c.world);
      const p = coveragePath(d, c.twoR, { start: c.start });
      const rep = coverageReport(c.world, p.points, r, LAB_RASTER);
      out.push({
        world: c.world.name,
        method,
        twoR: c.twoR,
        length: p.length,
        covered: rep.fraction,
        missed: rep.uncoveredArea,
        nCp: d.events.filter((e) => e.slice !== 'pass').length,
        nCe: d.cells.length,
        laps: p.laps,
        bound: pathLengthBound(c.world, c.twoR, d.cells.length).bound,
      });
    } else {
      const res = runOnlineCoverage(c.world, { twoR: c.twoR, start: c.start, policy: method as Policy });
      const rep = coverageReport(c.world, onlinePoints(res), r, LAB_RASTER);
      out.push({
        world: c.world.name,
        method,
        twoR: c.twoR,
        length: res.length,
        covered: rep.fraction,
        missed: rep.uncoveredArea,
        nCp: res.nodes.length,
        nCe: res.reeb.nCells,
        laps: res.laps,
        bound: pathLengthBound(c.world, c.twoR, res.reeb.nCells).bound,
      });
    }
  }
  return out;
}

export function coverageLab(): LabRow[] {
  return LAB_CASES.flatMap((c) => runLabCase(c));
}

/**
 * A seeded random world for the property checks: a convex m-gon boundary
 * around 2–4 disjoint convex holes, all in general position.
 */
export function randomCellWorld(rng: Rng, starShaped = false): CellWorld {
  for (let attempt = 0; attempt < 200; attempt++) {
    const m = 4 + Math.floor(rng.uniform(0, 5));
    const boundary = starShaped ? randomStarPolygon(rng, 7 + Math.floor(rng.uniform(0, 5)), 9.5, { x: 0, y: 0 }, 0.75) : randomConvexPolygon(rng, m, 9);
    if (boundary.length < 4) continue;
    const count = 2 + Math.floor(rng.uniform(0, 3));
    const field = randomConvexField(rng, { minX: -3, minY: -3, maxX: 3, maxY: 3 }, { count, radius: [0.5, 1.2], margin: 0.3, keepFree: [] });
    const holes = field.map((p) => {
      if (!starShaped) return [...p.pts];
      const c = p.pts.reduce((a, q) => ({ x: a.x + q.x / p.pts.length, y: a.y + q.y / p.pts.length }), { x: 0, y: 0 });
      const r = Math.min(...p.pts.map((q) => Math.hypot(q.x - c.x, q.y - c.y)), 1.1);
      return randomStarPolygon(rng, 5 + Math.floor(rng.uniform(0, 4)), r, c, 0.35);
    });
    const w = cellWorld('random', boundary, holes);
    if (w.obstacles.length >= 2 && isValidWorld(w, 0.2)) return w;
  }
  throw new Error('randomCellWorld: no valid world in 200 attempts');
}

/**
 * A seeded star-shaped polygon: n angles sorted, radii in [ρ·r, r]. Simple by
 * construction and, for small ρ, usually non-convex — notches included.
 */
export function randomStarPolygon(rng: Rng, n: number, r: number, c: Point2, rho: number): Point2[] {
  const angles = Array.from({ length: n }, () => rng.uniform(0, 2 * Math.PI)).sort((a, b) => a - b);
  return angles.map((a) => {
    const rr = r * rng.uniform(rho, 1);
    return { x: c.x + rr * Math.cos(a), y: c.y + rr * Math.sin(a) };
  });
}

/** Total vertex count V of a world (boundary plus holes). */
export const vertexCount = (w: CellWorld) => w.boundary.length + w.obstacles.reduce((s, o) => s + o.length, 0);
