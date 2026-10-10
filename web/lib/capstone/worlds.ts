/**
 * The three missions' worlds — Chapter 2's Apartment, Workbench and Lot, with
 * what the capstone adds on top and nothing removed.
 *
 * The Apartment's floor plan is a tree: every room has exactly one door, so a
 * shut door would simply end Rusty's mission. The architect's drawing Rusty is
 * given therefore has one more opening than Chapter 2's — a connecting door
 * between the study (D) and the bedroom (E) — which is what makes "the door
 * turned out to be shut" a replanning problem rather than a dead end.
 *
 * What Rusty is *not* given: the furniture (a few seeded boxes the blueprint
 * does not show) and the state of the doors. Its map starts as the inflated
 * blueprint with every door assumed open — D* Lite's free-space assumption —
 * and is corrected only by what its range sensor confirms.
 */

import { APARTMENT, type Segment, type World } from '../sim/world';
import type { Pose2 } from '../geom/se2';
import { Rng } from '../prob/rng';
import type { Point2 } from '../sim/world';

/** The D–E party wall at x = 6 is split by a 0.9 m connecting door at y ∈ [6.3, 7.2]. */
export const CONNECTING_DOOR: { x: number; y0: number; y1: number } = { x: 6, y0: 6.3, y1: 7.2 };

/** The architect's drawing: Chapter 2's Apartment with the connecting door opened. */
export const BLUEPRINT: World = {
  name: 'Apartment (blueprint)',
  bounds: APARTMENT.bounds,
  walls: APARTMENT.walls.flatMap((w): Segment[] => {
    const isPartyWall = w.x1 === 6 && w.x2 === 6 && Math.min(w.y1, w.y2) === 5 && Math.max(w.y1, w.y2) === 9;
    if (!isPartyWall) return [w];
    return [
      { x1: 6, y1: 5, x2: 6, y2: CONNECTING_DOOR.y0 },
      { x1: 6, y1: CONNECTING_DOOR.y1, x2: 6, y2: 9 },
    ];
  }),
  landmarks: APARTMENT.landmarks,
};

/** The two doors into the bedroom: the connecting door from the study and the corridor door. */
export const DOORS: readonly { name: string; seg: Segment }[] = [
  { name: 'the study–bedroom door', seg: { x1: CONNECTING_DOOR.x, y1: CONNECTING_DOOR.y0, x2: CONNECTING_DOOR.x, y2: CONNECTING_DOOR.y1 } },
  { name: 'the bedroom’s corridor door', seg: { x1: 7.4, y1: 5, x2: 8.3, y2: 5 } },
];

export const RUSTY_START: Pose2 = { x: 1.0, y: 1.0, theta: Math.PI / 2 };
export const RUSTY_GOAL: Point2 = { x: 9.0, y: 6.5 };

export interface Furniture {
  name: string;
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

/**
 * Four curated placements, none of which closes a route on its own; a seed
 * picks two. Curated rather than uniform because a box dropped in a doorway
 * would make the mission infeasible, and the capstone's subject is replanning,
 * not infeasibility.
 */
export const FURNITURE_SLOTS: readonly Furniture[] = [
  { name: 'boxes in the corridor', x0: 4.6, y0: 4.65, x1: 5.3, y1: 4.95 },
  { name: 'a cart in the corridor', x0: 6.6, y0: 3.85, x1: 7.0, y1: 4.15 },
  { name: 'a basket in the bedroom', x0: 8.9, y0: 5.45, x1: 9.4, y1: 5.85 },
  { name: 'an armchair in the study', x0: 4.0, y0: 6.2, x1: 4.6, y1: 6.8 },
];

export function pickFurniture(seed: number): Furniture[] {
  const rng = new Rng(seed * 7919 + 13);
  const idx = [0, 1, 2, 3];
  // Fisher–Yates on the seeded stream, keep two.
  for (let i = idx.length - 1; i > 0; i--) {
    const j = Math.floor(rng.next() * (i + 1));
    [idx[i], idx[j]] = [idx[j], idx[i]];
  }
  return idx.slice(0, 2).sort((a, b) => a - b).map((i) => FURNITURE_SLOTS[i]);
}

export function boxSegments(f: Furniture): Segment[] {
  return [
    { x1: f.x0, y1: f.y0, x2: f.x1, y2: f.y0 },
    { x1: f.x1, y1: f.y0, x2: f.x1, y2: f.y1 },
    { x1: f.x1, y1: f.y1, x2: f.x0, y2: f.y1 },
    { x1: f.x0, y1: f.y1, x2: f.x0, y2: f.y0 },
  ];
}

/** The world as it really is: blueprint + furniture + (optionally) one shut door. */
export function trueApartment(furniture: readonly Furniture[], shut: Segment | null): World {
  return {
    name: 'Apartment (truth)',
    bounds: BLUEPRINT.bounds,
    walls: [...BLUEPRINT.walls, ...furniture.flatMap(boxSegments), ...(shut ? [shut] : [])],
  };
}

/* -------------------------------------------------------------------------- */
/* Reach's tasks on the Workbench                                             */
/* -------------------------------------------------------------------------- */

/** A pick: a tip target and the straight Cartesian approach that ends on it. */
export interface ReachTask {
  name: string;
  tip: Point2;
}

/** The three picks Reach cycles through, starting from Choset's home q = (π/4, π/2). */
export const REACH_TASKS: readonly ReachTask[] = [
  { name: 'left of the shelf', tip: { x: -1.5, y: -0.55 } },
  { name: 'right of the base', tip: { x: 1.55, y: -0.55 } },
  { name: 'above the post', tip: { x: -0.45, y: 1.6 } },
];

export const REACH_HOME: readonly [number, number] = [Math.PI / 4, Math.PI / 2];

/** The Cartesian approach length: the last 12 cm of every pick are a straight tip move. */
export const APPROACH_LEN = 0.12;

/** The *Singular Goal* chaos: the same pick pushed out radially to full stretch. */
export function fullStretch(tip: Point2, reach = 2.0): Point2 {
  const r = Math.hypot(tip.x, tip.y) || 1;
  return { x: (tip.x / r) * reach, y: (tip.y / r) * reach };
}

/* -------------------------------------------------------------------------- */
/* Hitch's query in the Lot                                                     */
/* -------------------------------------------------------------------------- */

/**
 * Hitch starts east of the bays facing the east curb, trailer aligned behind
 * it, and parks nose-in in bay 3 — the Chapter 13 goal pose. The only way in
 * is to back out first, which is exactly when the trailer is unstable (F4).
 */
export const HITCH_START: Pose2 = { x: 12.0, y: 4.0, theta: 0 };
export const HITCH_GOAL: Pose2 = { x: 9.25, y: 6.4, theta: Math.PI / 2 };
