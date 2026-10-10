/**
 * The chapter's worked scenes — the numbers the prose prints come from here.
 *
 *   reachSquare   the micro-example of the design doc: Choset's arm (L₁ = L₂
 *                 = 1, zero thickness) and one small square [1.4, 1.6] ×
 *                 [−0.1, 0.1]. Link 1 cannot reach it, so QO is link 2's alone
 *                 and its θ₁ = 0 slice has an analytic half-width arctan(1/4).
 *   DOORWAY_ROOM  w4.4's room: a 4 m × 5 m room with a 0.6 m doorway in its
 *                 north wall and one convex crate — Choset's figure 3.5 with
 *                 numbers, so the inflation can be measured, not just seen.
 *   WORKBENCH_LAB the integration lab: the Workbench with its block slid to
 *                 within link 1's reach and a clamp added on the other side of
 *                 the base. Each near obstacle cuts a band through every θ₂;
 *                 two bands cut the torus into two free components, and no
 *                 planner in this book will connect a start in one to a goal
 *                 in the other. (The Workbench as Chapter 2 ships it keeps
 *                 every obstacle beyond link 1, and its Q_free stays connected.)
 */

import { Scene } from '../collide/scene';
import { ChainChecker, DiscChecker } from '../collide/collision';
import { rect, type Polygon } from '../collide/polygon';
import { Reach } from '../robots/reach';
import { WORKBENCH, type PolyWorld } from '../robots/worlds';
import type { Segment } from '../sim/world';
import { BitGrid, planeChart, rasterizeCobstacle, torusChart } from './raster';

// ---------------------------------------------------------------------------
// The micro-example: Reach and one small square
// ---------------------------------------------------------------------------

export const REACH_SQUARE: Polygon = rect(1.4, -0.1, 1.6, 0.1);

/** A scene with the square alone (no walls — the Workbench's table edge is out of reach anyway). */
export function reachSquareScene(square: Polygon = REACH_SQUARE): Scene {
  return Scene.ofWalls([], { minX: -2.4, minY: -2.4, maxX: 2.4, maxY: 2.4 }).withObstacle('the square', square);
}

export function reachSquareChecker(arm: Reach = Reach.choset(), square: Polygon = REACH_SQUARE): ChainChecker {
  return new ChainChecker(reachSquareScene(square), arm);
}

/** The 1° raster of the micro-example (360 × 360 cells centred at integer degrees). */
export function reachSquareRaster(degPerCell = 1, arm: Reach = Reach.choset()): BitGrid {
  const n = Math.round(360 / degPerCell);
  return rasterizeCobstacle(reachSquareChecker(arm), torusChart(n));
}

/** Analytic half-width of the θ₁ = 0 slice: the angle the near corner (1.4, 0.1) subtends from the elbow (1, 0). */
export const SLICE_HALF_WIDTH = Math.atan(0.1 / 0.4);

/**
 * Analytic θ₁ extent: the elbow (cos θ₁, sin θ₁) is within L₂ = 1 of the square
 * iff (1.4 − cos θ₁)² + (sin θ₁ − 0.1)² ≤ 1 (for θ₁ > 0; the square is
 * symmetric so the negative side mirrors it), i.e. 2.8 cos θ₁ + 0.2 sin θ₁ ≥
 * 1.97, i.e. |θ₁ − φ| ≤ arccos(1.97 / √7.88) with φ = atan2(0.2, 2.8).
 */
export const THETA1_EXTENT = Math.atan2(0.2, 2.8) + Math.acos(1.97 / Math.sqrt(2.8 * 2.8 + 0.2 * 0.2));

/**
 * Bisection of the exact segment test along the θ₁ = 0 row: the largest θ₂ ∈
 * [0, π/2] still in collision. Converges to arctan(1/4) because the boundary
 * configuration is the one where link 2 passes through the corner (1.4, 0.1).
 */
export function sliceHalfWidthBisection(iterations = 60): number {
  const c = reachSquareChecker();
  let lo = 0; // in collision
  let hi = Math.PI / 2; // free
  for (let k = 0; k < iterations; k++) {
    const mid = (lo + hi) / 2;
    if (c.isFree([0, mid])) hi = mid;
    else lo = mid;
  }
  return (lo + hi) / 2;
}

// ---------------------------------------------------------------------------
// w4.4 — the doorway room
// ---------------------------------------------------------------------------

const h = (y: number, x1: number, x2: number): Segment => ({ x1, y1: y, x2, y2: y });
const v = (x: number, y1: number, y2: number): Segment => ({ x1: x, y1, x2: x, y2 });

export const DOOR_X: [number, number] = [1.7, 2.3];
export const DOOR_Y = 3.6;

/**
 * A room with a doorway: 4 m wide, the dividing wall at y = 3.6 with a 0.6 m
 * gap, a convex crate [1.0, 1.8] × [1.2, 1.7] in the room, and a corridor
 * above. A disc of radius 0.11 (Rusty) passes the door with 0.19 m to spare on
 * each side; a disc of radius 0.35 has diameter 0.7 > 0.6 and cannot. Choset's
 * figure 3.5 is three such discs.
 */
export const DOORWAY_ROOM: PolyWorld = {
  name: 'Doorway room',
  bounds: { minX: 0, minY: 0, maxX: 4, maxY: 5 },
  walls: [
    h(0, 0, 4),
    h(5, 0, 4),
    v(0, 0, 5),
    v(4, 0, 5),
    h(DOOR_Y, 0, DOOR_X[0]),
    h(DOOR_Y, DOOR_X[1], 4),
  ],
  obstacles: [{ name: 'the crate', poly: rect(1.0, 1.2, 1.8, 1.7) }],
  labels: [
    { x: 1.4, y: 1.45, text: 'crate' },
    { x: 2.0, y: 4.3, text: 'corridor' },
  ],
};

export const DOORWAY_START = { x: 0.6, y: 0.6 };
export const DOORWAY_GOAL = { x: 3.4, y: 4.4 };

/** The disc's C-space picture: an R² raster of the room for a disc of radius r. */
export function doorwayRaster(radius: number, cell = 0.05): BitGrid {
  const scene = Scene.fromWorld(DOORWAY_ROOM);
  const b = DOORWAY_ROOM.bounds;
  const chart = planeChart(b, Math.round((b.maxX - b.minX) / cell), Math.round((b.maxY - b.minY) / cell));
  return rasterizeCobstacle(new DiscChecker(scene, radius), chart);
}

// ---------------------------------------------------------------------------
// The integration lab: a Workbench whose free space is in two pieces
// ---------------------------------------------------------------------------

/** The block slid toward the base: a 0.4 m square centred at (0.75, 0.35), nearest corner 0.58 m from the base. */
export const LAB_BLOCK: Polygon = rect(0.55, 0.15, 0.95, 0.55);
/** A clamp on the far side of the base: a 0.3 m square centred at (−0.55, 0.55), 0.57 m from the base. */
export const LAB_CLAMP: Polygon = rect(-0.7, 0.4, -0.4, 0.7);
/** Start and goal of the lab query, chosen in the two free components the bands cut. */
export const LAB_START: readonly [number, number] = [Math.PI / 2, 1.0];
export const LAB_GOAL: readonly [number, number] = [-Math.PI / 2, 1.0];

/**
 * The lab scene: the moved block, the Workbench's post and shelf, and (unless
 * `withClamp` is false) the clamp. Link 1 reaches the block for θ₁ ∈ [8°, 47°]
 * and the clamp for θ₁ ∈ [118°, 152°] — two vertical bands on the torus.
 */
export function workbenchLabScene(withClamp = true): Scene {
  let scene = Scene.ofWalls([], WORKBENCH.bounds)
    .withObstacle('the block', LAB_BLOCK)
    .withObstacle('the post', WORKBENCH.obstacles[1].poly)
    .withObstacle('the shelf', WORKBENCH.obstacles[2].poly);
  if (withClamp) scene = scene.withObstacle('the clamp', LAB_CLAMP);
  return scene;
}

/** The lab raster: the Workbench arm (3 cm capsules) on a `degPerCell` torus grid. */
export function workbenchLabRaster(degPerCell = 1, withClamp = true, arm: Reach = Reach.workbench()): BitGrid {
  const n = Math.round(360 / degPerCell);
  return rasterizeCobstacle(new ChainChecker(workbenchLabScene(withClamp), arm), torusChart(n));
}
