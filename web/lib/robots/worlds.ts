/**
 * The three worlds of the lab: the Apartment (ported from the sister book),
 * the Workbench (Reach's table), and the Lot (Hitch's parking lot).
 *
 * All three share the ported `World` shape — a bounding box plus polyline
 * `walls` — so `drawWorld`, `rayCast`, `distanceToWalls` and the occupancy-grid
 * machinery work unchanged. The two new worlds add `obstacles`: *filled*
 * polygons with names. A wall is a curve the robot may not cross; an obstacle
 * is a region the robot may not enter. Reach's links and Hitch's chassis are
 * checked against both, and the names are the ones the prose uses for the
 * rest of the book ("the block", "the post", "the shelf", "bay 3", "the east
 * pillar"). A world is polylines and polygons with names — never a bitmap.
 */

import { APARTMENT, type Segment, type World } from '../sim/world';
import { rect, type Polygon } from '../collide/polygon';

export { APARTMENT };

export interface NamedPolygon {
  name: string;
  poly: Polygon;
}

export interface WorldLabel {
  x: number;
  y: number;
  text: string;
}

/** A world with filled polygonal obstacles on top of the ported polyline walls. */
export interface PolyWorld extends World {
  /** Solid regions. Collision is "footprint touches the polygon". */
  obstacles: NamedPolygon[];
  /** Painted lines (bay markings) — drawn, never collided with. */
  markings?: Segment[];
  /** Place names for the atlas. */
  labels?: WorldLabel[];
}

const boundaryWalls = (minX: number, minY: number, maxX: number, maxY: number): Segment[] => [
  { x1: minX, y1: minY, x2: maxX, y2: minY },
  { x1: maxX, y1: minY, x2: maxX, y2: maxY },
  { x1: maxX, y1: maxY, x2: minX, y2: maxY },
  { x1: minX, y1: maxY, x2: minX, y2: minY },
];

/**
 * The Workbench: a 4.8 m square table with Reach bolted to its center.
 *
 * Reach's default links are L₁ = L₂ = 1 m, so the arm can touch anything
 * within 2 m of the origin; the table edge at ±2.4 m is a wall it can never
 * reach. Three obstacles sit inside the annulus:
 *
 *   the block — the square [0.5, 1.0] × [1.0, 1.5]. This is the obstacle of the
 *               chapter's worked example (distance 1.5/√2 − 1 from the home
 *               configuration), so the number in the prose is a number about
 *               the default world, not about a toy.
 *   the post  — a 0.24 m square column at (−1.2, 0.8), in the arm's left-hand
 *               half; the swept-query example drives link 1 through it.
 *   the shelf — a long thin bar along the near edge, [−1.9, −0.3] × [−1.75, −1.55].
 */
export const WORKBENCH: PolyWorld = {
  name: 'Workbench',
  bounds: { minX: -2.4, minY: -2.4, maxX: 2.4, maxY: 2.4 },
  walls: boundaryWalls(-2.4, -2.4, 2.4, 2.4),
  obstacles: [
    { name: 'the block', poly: rect(0.5, 1.0, 1.0, 1.5) },
    { name: 'the post', poly: rect(-1.32, 0.68, -1.08, 0.92) },
    { name: 'the shelf', poly: rect(-1.9, -1.75, -0.3, -1.55) },
  ],
  labels: [
    { x: 0.75, y: 1.25, text: 'block' },
    { x: -1.2, y: 0.8, text: 'post' },
    { x: -1.1, y: -1.65, text: 'shelf' },
    { x: 0, y: -0.25, text: 'base' },
  ],
};

/**
 * The Lot: a 16 m × 10 m parking lot, curbed on all four sides.
 *
 * Four bays, each 2.5 m wide and 5 m deep, line the north curb; their painted
 * lines are `markings` (a car may drive over paint). Two 0.7 m square pillars
 * stand in the open floor so that Hitch has something to back around. The
 * curb is a wall; the pillars are obstacles; the bays are places with names.
 */
const BAY_X0 = 3.0;
const BAY_W = 2.5;
const BAY_DEPTH = 5.0;

export const LOT_BAYS: { name: string; minX: number; maxX: number; minY: number; maxY: number }[] = [
  1, 2, 3, 4,
].map((i) => ({
  name: `bay ${i}`,
  minX: BAY_X0 + (i - 1) * BAY_W,
  maxX: BAY_X0 + i * BAY_W,
  minY: 10 - BAY_DEPTH,
  maxY: 10,
}));

export const LOT: PolyWorld = {
  name: 'Lot',
  bounds: { minX: 0, minY: 0, maxX: 16, maxY: 10 },
  walls: boundaryWalls(0, 0, 16, 10),
  obstacles: [
    { name: 'the west pillar', poly: rect(3.65, 2.15, 4.35, 2.85) },
    { name: 'the east pillar', poly: rect(11.65, 2.15, 12.35, 2.85) },
  ],
  markings: [
    // Bay dividers: five lines from the north curb down to the bay mouths.
    ...[0, 1, 2, 3, 4].map((k) => ({
      x1: BAY_X0 + k * BAY_W,
      y1: 10 - BAY_DEPTH,
      x2: BAY_X0 + k * BAY_W,
      y2: 10,
    })),
  ],
  labels: [
    ...LOT_BAYS.map((b) => ({ x: (b.minX + b.maxX) / 2, y: (b.minY + b.maxY) / 2 + 1.3, text: b.name })),
    { x: 4.0, y: 1.6, text: 'west pillar' },
    { x: 12.0, y: 1.6, text: 'east pillar' },
  ],
};

/** The Apartment as a `PolyWorld` (no filled obstacles — its geometry is all walls). */
export const APARTMENT_SCENE: PolyWorld = {
  ...APARTMENT,
  obstacles: [],
  labels: [
    { x: 2.0, y: 1.9, text: 'A' },
    { x: 6.0, y: 2.6, text: 'B' },
    { x: 10.0, y: 1.9, text: 'C' },
    { x: 6.0, y: 4.4, text: 'corridor' },
    { x: 3.8, y: 7.0, text: 'D (study)' },
    { x: 8.0, y: 7.0, text: 'E (bedroom)' },
  ],
};

export const WORLDS: Record<'apartment' | 'workbench' | 'lot', PolyWorld> = {
  apartment: APARTMENT_SCENE,
  workbench: WORKBENCH,
  lot: LOT,
};

/** Every segment a ray can hit: walls plus the edges of each filled obstacle. */
export function allSegments(world: World | PolyWorld): Segment[] {
  const segs = [...world.walls];
  const obstacles = (world as PolyWorld).obstacles;
  if (obstacles) {
    for (const o of obstacles) {
      const n = o.poly.length;
      for (let i = 0; i < n; i++) {
        const a = o.poly[i];
        const b = o.poly[(i + 1) % n];
        segs.push({ x1: a.x, y1: a.y, x2: b.x, y2: b.y });
      }
    }
  }
  return segs;
}
