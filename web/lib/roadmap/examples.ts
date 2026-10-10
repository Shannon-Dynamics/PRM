/**
 * The chapter's fixed scenes: Choset Fig. 5.8 (two rectangles and a lighthouse
 * vertex v, placed so Table 5.1 reproduces exactly), the design doc's
 * three-obstacle slab (pegs at (0, 0) and (4, 0), a wall along y = 4), and the
 * Apartment seen two ways — as polygonal C-obstacles for disc-Rusty (the
 * visibility graph lives there) and as a distance query over its walls (where
 * the GVD lives; the GVD of inflated obstacles is the GVD of the originals,
 * restricted to clearance above the radius).
 */

import { Scene } from '../collide/scene';
import { rect, type Polygon } from '../collide/polygon';
import { inflateSegment } from '../cspace/star';
import { PlanarObstacles } from '../potential/planar';
import { APARTMENT, type Bounds, type Point2, type Segment, type World } from '../sim/world';
import type { Site } from './gvd';
import { VisibilityScene } from './visibility';

/* -------------------------------------------------------------------------- */
/* Choset Fig. 5.8 / Table 5.1                                                 */
/* -------------------------------------------------------------------------- */

/**
 * Rectangle 1: x ∈ [2, 4], y ∈ [−1, 1]; rectangle 2: x ∈ [6, 10], y ∈ [−2, 4];
 * v at the origin. Vertices are numbered counter-clockwise from the lower-left,
 * so edge E_k joins v_k to v_{k+1} as in the figure. The horizontal half-line
 * from v crosses E4, E2, E8, E6 in that order; the sweep pauses at
 * α3, α7, α4, α8, α1, α5, α2, α6 and adds (v, v4), (v, v8), (v, v1).
 */
export const FIG_5_8_RECT1: Polygon = rect(2, -1, 4, 1);
export const FIG_5_8_RECT2: Polygon = rect(6, -2, 10, 4);
export const FIG_5_8_V: Point2 = { x: 0, y: 0 };
export const FIG_5_8_BOUNDS: Bounds = { minX: -1, minY: -3, maxX: 11, maxY: 5 };

export function fig58Scene(extra: { p: Point2; name: string }[] = [{ p: FIG_5_8_V, name: 'v' }]): VisibilityScene {
  return new VisibilityScene([FIG_5_8_RECT1, FIG_5_8_RECT2], extra);
}

/** Table 5.1 as data: the vertex names in sweep order, S after each, and which were added. */
export const TABLE_5_1 = {
  initialS: ['E4', 'E2', 'E8', 'E6'],
  order: ['v3', 'v7', 'v4', 'v8', 'v1', 'v5', 'v2', 'v6'],
  S: [
    ['E4', 'E3', 'E8', 'E6'],
    ['E4', 'E3', 'E8', 'E7'],
    ['E8', 'E7'],
    [],
    // Choset prints {E1, E4} here and {E4, E1, E8, E5} at the next row; the two cannot both be
    // sorted by distance. Just past α1 the ray meets E4 (x = 2) before E1 (y = −1), so E4 is nearer.
    ['E4', 'E1'],
    ['E4', 'E1', 'E8', 'E5'],
    ['E4', 'E2', 'E8', 'E5'],
    ['E4', 'E2', 'E8', 'E6'],
  ],
  added: ['v4', 'v8', 'v1'],
};

/* -------------------------------------------------------------------------- */
/* The design doc's slab: two pegs and a wall                                  */
/* -------------------------------------------------------------------------- */

export const SLAB_BOUNDS: Bounds = { minX: -2, minY: -1, maxX: 6, maxY: 4 };
export const SLAB_WALL: Segment = { x1: -12, y1: 4, x2: 16, y2: 4 };
export const SLAB_SITES: Site[] = [
  { kind: 'point', p: { x: 0, y: 0 } },
  { kind: 'point', p: { x: 4, y: 0 } },
  { kind: 'segment', s: SLAB_WALL },
];
/** The meet point F_{P1 P2 W}: on x = 2, √(4 + y²) = 4 − y ⇒ y = 1.5; clearance 2.5. */
export const SLAB_MEET: Point2 = { x: 2, y: 1.5 };
export const SLAB_CLEARANCE = 2.5;

/** Sites for a movable version: pegs at p1, p2 and a wall along y = wallY. */
export function slabSites(p1: Point2, p2: Point2, wallY: number): Site[] {
  return [
    { kind: 'point', p: p1 },
    { kind: 'point', p: p2 },
    { kind: 'segment', s: { x1: -40, y1: wallY, x2: 40, y2: wallY } },
  ];
}

/** The slab as planar obstacles for brushfire: the pegs as zero-radius discs, the wall as a segment. */
export function slabObstacles(sites: readonly Site[] = SLAB_SITES): PlanarObstacles {
  return new PlanarObstacles(
    sites.map((s) => {
      if (s.kind === 'point') return { kind: 'disc' as const, c: s.p, r: 0 };
      if (s.kind === 'segment') return { kind: 'segment' as const, s: s.s };
      return { kind: 'polygon' as const, poly: s.poly };
    }),
  );
}

/* -------------------------------------------------------------------------- */
/* The Apartment, two ways                                                     */
/* -------------------------------------------------------------------------- */

/** Disc-Rusty's C-obstacles: every wall inflated by the radius into a capsule polygon (Chapter 4). */
export function apartmentCObstacles(radius = 0.22, arcSegments = 4, world: World = APARTMENT): Polygon[] {
  return world.walls.map((w) => inflateSegment(w, radius, arcSegments));
}

/** The walls as a distance query — one obstacle per segment — for the GVD and the retraction. */
export function apartmentDistanceQuery(world: World = APARTMENT): PlanarObstacles {
  return PlanarObstacles.fromScene(Scene.fromWorld(world));
}

/** Room B of the Apartment with its kitchen counter: the Voronoi Carver's second preset. */
export const ROOM_B_BOUNDS: Bounds = { minX: 4, minY: 0, maxX: 8, maxY: 3.8 };

/** A visibility scene for the Apartment at a given radius, with start and goal as extra nodes. */
export function apartmentVisibility(start: Point2, goal: Point2, radius = 0.22): VisibilityScene {
  return new VisibilityScene(apartmentCObstacles(radius), [
    { p: start, name: 'start' },
    { p: goal, name: 'goal' },
  ]);
}

/* -------------------------------------------------------------------------- */
/* The Apartment with a kitchen island (the Retract Candy's loop)              */
/* -------------------------------------------------------------------------- */

/** A free-standing island in room B, so the free space has a hole a loop can go around. */
export const KITCHEN_ISLAND: Polygon = rect(6.6, 1.8, 7.2, 2.4);
export const ISLAND_CENTER: Point2 = { x: 6.9, y: 2.1 };

/** The Apartment's walls plus the island, as one distance query (walls as segments, the island as a polygon). */
export function apartmentWithIslandQuery(world: World = APARTMENT): PlanarObstacles {
  return PlanarObstacles.fromScene(Scene.fromWorld(world).withObstacle('island', KITCHEN_ISLAND));
}

/** A closed loop of n points at radius r about c. */
export function loopAround(c: Point2, r: number, n = 48): Point2[] {
  const out: Point2[] = [];
  for (let k = 0; k < n; k++) out.push({ x: c.x + r * Math.cos((2 * Math.PI * k) / n), y: c.y + r * Math.sin((2 * Math.PI * k) / n) });
  return out;
}
