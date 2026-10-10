/**
 * The lab, wired for Part III: each of the three robots as a (Chart, FreeSpace)
 * pair, a few scratch worlds the chapters' widgets need, the torus C-obstacle
 * raster that the Reach tabs paint amber, and the lattice A* cost floor that
 * Chapter 13 draws as a dashed line under every cost curve.
 *
 * Nothing here is new geometry: the worlds are Chapter 2's, the checkers are
 * Chapter 2's, the manifolds are Chapter 5's, the grid search is Chapter 6's.
 * The C-obstacle raster is made by *asking the collision checker* on a grid
 * because this module may not own a C-space construction (Chapter 4 does).
 */

import { Scene } from '../collide/scene';
import type { Polygon } from '../collide/polygon';
import { rect } from '../collide/polygon';
import { APARTMENT, type World } from '../sim/world';
import { WORKBENCH, LOT, type PolyWorld } from '../robots/worlds';
import { Reach, REACH } from '../robots/reach';
import { Hitch } from '../robots/hitch';
import { RUSTY_RADIUS } from '../robots/rusty';
import { R2, Rd } from '../manifold/rd';
import { T2, Torus, type BitGrid, cellAngles } from '../manifold/torus';
import { SE2 } from '../manifold/se2';
import type { Pose2 } from '../geom/se2';
import { astar } from '../search/best-first';
import { GridGraph, cellCenter, freeCellNear, gridFromWorld, gridHeuristic } from '../search/grid';
import { BoxWorld, chainFreeSpace, discFreeSpace, footprintFreeSpace, type CountingFreeSpace } from './free-space';

/* -------------------------------------------------------------------------- */
/* Rusty as a disc in the Apartment: ℝ²                                        */
/* -------------------------------------------------------------------------- */

export interface DiscLab {
  world: World;
  scene: Scene;
  space: Rd;
  free: CountingFreeSpace<number[]>;
  radius: number;
  /** Lebesgue measure of the sampling box (μ(𝒬); the free part is a bit smaller). */
  muBox: number;
}

export function discLab(world: World = APARTMENT, radius = RUSTY_RADIUS * 2): DiscLab {
  const scene = Scene.fromWorld(world);
  const b = world.bounds;
  return {
    world,
    scene,
    space: R2([b.minX, b.minY], [b.maxX, b.maxY]),
    free: discFreeSpace(scene, radius),
    radius,
    muBox: (b.maxX - b.minX) * (b.maxY - b.minY),
  };
}

/** The Apartment query every Part III widget defaults to: room A to the bedroom. */
export const APARTMENT_START: number[] = [1.0, 1.0];
export const APARTMENT_GOAL: number[] = [9.0, 6.5];

/* -------------------------------------------------------------------------- */
/* Reach on the Workbench: T²                                                  */
/* -------------------------------------------------------------------------- */

export interface TorusLab {
  world: PolyWorld;
  scene: Scene;
  arm: Reach;
  space: Torus;
  free: CountingFreeSpace<number[]>;
  muBox: number;
}

export function torusLab(world: PolyWorld = WORKBENCH, arm: Reach = REACH): TorusLab {
  const scene = Scene.fromWorld(world);
  return { world, scene, arm, space: T2, free: chainFreeSpace(scene, arm), muBox: (2 * Math.PI) ** 2 };
}

/** The Workbench with a slot: two bars with a gap the elbow must thread (the Chapter 11 hook). */
export const WORKBENCH_SLOT: PolyWorld = {
  ...WORKBENCH,
  name: 'Workbench (slot)',
  obstacles: [
    ...WORKBENCH.obstacles,
    { name: 'the upper bar', poly: rect(1.2, -0.05, 2.3, 0.08) },
    { name: 'the lower bar', poly: rect(1.2, -0.88, 2.3, -0.55) },
  ],
};

/** The slot's mouth: where the hook asks the forearm's tip to go. */
export const SLOT_TARGET: [number, number] = [1.85, -0.3];

/** Reach's default torus query: from the home pose, around the block, to the far side. */
export const TORUS_START: number[] = [Math.PI / 4, Math.PI / 2];
export const TORUS_GOAL: number[] = [-2.2, -1.2];

/**
 * Raster of Reach's C-obstacle on the unwrapped square: 1 where the arm is in
 * collision. n × n cells, cell centers at `cellAngles`. This is sampling the
 * `Collision` checker directly — the chapter owns no C-space construction.
 */
export function rasterTorusCObstacle(lab: TorusLab, n = 72): BitGrid {
  const grid: BitGrid = { n1: n, n2: n, occupied: new Uint8Array(n * n) };
  const checker = lab.free;
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      const q = cellAngles(grid, i, j);
      if (!checker.isFree([q[0], q[1]])) grid.occupied[j * n + i] = 1;
    }
  }
  return grid;
}

/* -------------------------------------------------------------------------- */
/* Hitch in the Lot: SE(2), with a straight-line steer that ignores the car     */
/* -------------------------------------------------------------------------- */

export interface Se2Lab {
  world: PolyWorld;
  scene: Scene;
  car: Hitch;
  space: SE2;
  free: CountingFreeSpace<Pose2>;
}

/** w = 1 m/rad is a deliberate, arbitrary exchange rate; Chapter 11 says so. */
export function se2Lab(world: PolyWorld = LOT, car: Hitch = Hitch.car(), w = 1): Se2Lab {
  const scene = Scene.fromWorld(world);
  return { world, scene, car, space: new SE2(w, world.bounds), free: footprintFreeSpace(scene, car) };
}

export const LOT_START: Pose2 = { x: 2.0, y: 1.2, theta: 0 };
// Nose-in to the third bay: the rear axle at y = 6.4 puts the front overhang 0.2 m short of the north curb.
export const LOT_GOAL: Pose2 = { x: 9.25, y: 6.4, theta: Math.PI / 2 };

/* -------------------------------------------------------------------------- */
/* Scratch worlds                                                              */
/* -------------------------------------------------------------------------- */

/**
 * Two W × W rooms joined by a corridor of width w and length W (Choset figure
 * 7.19), as a point-robot BoxWorld. The corridor is centered vertically.
 */
export function corridorWorld(W: number, w: number, corridorLength = W): BoxWorld {
  const total = 2 * W + corridorLength;
  const yc = W / 2;
  const hw = Math.max(1e-3, w / 2);
  return new BoxWorld({ minX: 0, minY: 0, maxX: total, maxY: W }, [
    { minX: W, minY: 0, maxX: W + corridorLength, maxY: yc - hw },
    { minX: W, minY: yc + hw, maxX: W + corridorLength, maxY: W },
  ]);
}

/** A BoxWorld as a `Scene` drawn with the usual world painter (walls + filled boxes). */
export function boxWorldScene(bw: BoxWorld): Scene {
  return bw.toScene();
}

export function polygonsOf(bw: BoxWorld): Polygon[] {
  return bw.obstacles.map((o) => rect(o.minX, o.minY, o.maxX, o.maxY));
}

/* -------------------------------------------------------------------------- */
/* The lattice floor (Chapter 6 on the inflated Apartment)                      */
/* -------------------------------------------------------------------------- */

export interface LatticeFloor {
  cost: number;
  path: { x: number; y: number }[];
  cellSize: number;
}

/**
 * A* on the Chapter 6 grid with exact √2 diagonals and the disc's inflation —
 * the cost floor every Chapter 13 curve is measured against. It is a floor for
 * the *lattice*; a sampler placing nodes anywhere can beat it by the octile
 * detour, and the prose says so.
 */
export function latticeFloor(lab: DiscLab, start: readonly number[], goal: readonly number[], cellSize = 0.1): LatticeFloor | null {
  const grid = gridFromWorld(lab.world, cellSize, lab.radius);
  const graph = new GridGraph(grid, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
  const s = freeCellNear(grid, start[0], start[1]);
  const g = freeCellNear(grid, goal[0], goal[1]);
  const h = gridHeuristic(grid, g, 'octile', Math.SQRT2);
  const t = astar(graph, s, g, h);
  if (t.status !== 'found') return null;
  return { cost: t.cost, path: t.path.map((c) => cellCenter(grid, c)), cellSize };
}
