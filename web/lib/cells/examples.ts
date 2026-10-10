/**
 * The chapter's worlds.
 *
 *  - CELL_BENCH        the design doc's §3 micro-example: a convex quadrilateral
 *                      plot with a triangle T and a quadrilateral K, all eleven
 *                      vertex x-coordinates distinct. 12 trapezoids, 13
 *                      adjacencies; 6 critical points, 7 boustrophedon cells.
 *  - LIVING_ROOM       Room C of the Apartment, furnished for this chapter with
 *                      a sofa against the north wall and a coffee table, as
 *                      disc-Rusty's C-space: the room eroded by Rusty's radius,
 *                      the furniture inflated by it (Chapter 4's `inflateDisc`).
 *  - WORKBENCH_TOP     the Workbench table as the free space of Reach's
 *                      end-effector pad, with the block, the post and the
 *                      shelf as holes (Chapter 2's objects).
 *
 * Rectilinear worlds go through `generalPosition`, which leans every vertical
 * wall by √2 milli-radians; the constants below are the *sheared* worlds.
 */

import { rect, type Polygon } from '../collide/polygon';
import { inflateDisc } from '../cspace/star';
import { WORKBENCH } from '../robots/worlds';
import type { Point2 } from '../sim/world';
import { cellWorld, generalPosition, type CellWorld } from './world';

/** Rusty's radius (Chapter 2) — the inflation of every Apartment C-obstacle. */
export const RUSTY_RADIUS = 0.22;

export const BENCH_BOUNDARY: Polygon = [
  { x: 0, y: 0.2 },
  { x: 10, y: 0 },
  { x: 10.4, y: 6 },
  { x: 0.3, y: 6.2 },
];
export const BENCH_T: Polygon = [
  { x: 2, y: 2 },
  { x: 4, y: 1.5 },
  { x: 3, y: 4 },
];
export const BENCH_K: Polygon = [
  { x: 6, y: 1 },
  { x: 8, y: 0.8 },
  { x: 8.6, y: 4 },
  { x: 6.5, y: 4.4 },
];

/** The §3 micro-world — already in general position, so no shear. */
export const CELL_BENCH: CellWorld = cellWorld('cell bench', BENCH_BOUNDARY, [BENCH_T, BENCH_K], ['T', 'K']);

/** The numbers the text promises for CELL_BENCH. */
export const BENCH_FACTS = {
  trapezoids: 12,
  adjacencies: 13,
  eulerFaces: 3,
  criticalPoints: 6,
  boustrophedonCells: 7,
  reebNodes: 6,
  reebEdges: 7,
  /** ⌈2 / 0.5⌉: laps in the first cell, x ∈ [0, 2], at 2r = 0.5. */
  firstCellLaps: 4,
  twoR: 0.5,
  criticalXs: [0, 2, 4, 6, 8.6, 10.4],
} as const;

/** Room C of the Apartment, eroded by Rusty's radius; doorway treated as closed. */
const ROOM_C = { minX: 8 + RUSTY_RADIUS, minY: 0 + RUSTY_RADIUS, maxX: 12 - RUSTY_RADIUS, maxY: 3.8 - RUSTY_RADIUS };

/** The sofa and the coffee table as workspace rectangles. */
export const SOFA: Polygon = rect(9.15, 2.45, 10.4, 3.05);
export const COFFEE_TABLE: Polygon = rect(9.8, 0.95, 10.9, 1.45);

/** Disc-Rusty's C-obstacle for a piece of furniture: Chapter 4's inflation, octagonal corners. */
export const inflateFurniture = (p: Polygon) => inflateDisc(p, RUSTY_RADIUS, 8);

/** The living room's C-space, before the general-position shear. */
export function livingRoomRaw(withTable = true): CellWorld {
  const obstacles = [inflateFurniture(SOFA)];
  const names = ['sofa'];
  if (withTable) {
    obstacles.push(inflateFurniture(COFFEE_TABLE));
    names.push('coffee table');
  }
  return cellWorld(withTable ? 'living room' : 'living room (sofa only)', rect(ROOM_C.minX, ROOM_C.minY, ROOM_C.maxX, ROOM_C.maxY), obstacles, names);
}

export const LIVING_ROOM: CellWorld = generalPosition(livingRoomRaw(true)).world;
export const LIVING_ROOM_SOFA_ONLY: CellWorld = generalPosition(livingRoomRaw(false)).world;

/** Where Rusty starts in the living room: its south-west corner, west of the coffee table. */
export const LIVING_ROOM_START: Point2 = { x: 9.0, y: 0.8 };

/**
 * The Workbench table top as the free space of Reach's end-effector: the
 * table square with the block, the post and the shelf as holes. Covering it is
 * wiping the table — the "end-effector footprint" of the integration lab.
 */
export function workbenchTopRaw(): CellWorld {
  const b = WORKBENCH.bounds;
  return cellWorld(
    'Workbench top',
    rect(b.minX, b.minY, b.maxX, b.maxY),
    WORKBENCH.obstacles.map((o) => o.poly),
    WORKBENCH.obstacles.map((o) => o.name),
  );
}

export const WORKBENCH_TOP: CellWorld = generalPosition(workbenchTopRaw()).world;
export const WORKBENCH_START: Point2 = { x: -2.0, y: 0.1 };
