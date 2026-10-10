/**
 * The chapter's worlds. Each is a `PolygonWorld` so Bug1/Bug2 can walk its
 * boundaries exactly, plus a query (start, goal).
 *
 *  - two-rectangles  the §3 micro-example: L_Bug1 = 34, L_Bug2 = 16, exactly
 *  - fig-2-1         two convex blobs in the spirit of Choset's figure 2.1
 *  - spiral          the figure 2.4 construction: Bug2 loses to Bug1
 *  - enclosed        the goal walled in: every Bug must say "unreachable"
 *  - random          a seeded field of disjoint convex polygons (arena re-roll)
 */

import { Rng } from '../prob/rng';
import type { Point2 } from '../sim/world';
import { polygonWorld, pt, randomConvexField, rectangle, spiralPolygon, type PolygonWorld } from './geometry';

export interface BugPreset {
  id: PresetId;
  label: string;
  world: PolygonWorld;
  start: Point2;
  goal: Point2;
  /** Sensible Tangent Bug range for this map. */
  range: number;
  /** One line for the arena's caption. */
  note: string;
}

export type PresetId = 'two-rectangles' | 'fig-2-1' | 'spiral' | 'enclosed' | 'fig-2-8' | 'random';

/** The §3 micro-example: WO₁ = [2,4]×[−1,1] (p₁ = 8), WO₂ = [6,7]×[−2,2] (p₂ = 10). */
export const TWO_RECTANGLES: BugPreset = {
  id: 'two-rectangles',
  label: 'Two rectangles (worked example)',
  world: polygonWorld(
    'two-rectangles',
    { minX: -1, minY: -4, maxX: 11, maxY: 4 },
    [rectangle(2, -1, 4, 1), rectangle(6, -2, 7, 2)],
  ),
  start: pt(0, 0),
  goal: pt(10, 0),
  range: 3,
  note: 'The worked example: Bug1 walks 34 against a bound of 37, Bug2 walks 16 against 28.',
};

/** Two convex blobs, start bottom-left, goal top-right — figure 2.1's situation. */
export const FIG_2_1: BugPreset = {
  id: 'fig-2-1',
  label: 'Two blobs (C fig. 2.1)',
  world: polygonWorld(
    'fig-2-1',
    { minX: 0, minY: 0, maxX: 12, maxY: 8 },
    [
      { pts: [pt(2.6, 1.2), pt(4.6, 0.9), pt(6.0, 2.2), pt(5.4, 4.0), pt(3.4, 4.3), pt(2.2, 3.0)] },
      { pts: [pt(6.8, 4.2), pt(8.9, 3.4), pt(10.3, 4.6), pt(9.9, 6.6), pt(7.9, 7.0), pt(6.5, 5.8)] },
    ],
  ),
  start: pt(0.8, 0.8),
  goal: pt(11.2, 7.2),
  range: 2.5,
  note: 'Simple convex obstacles: the greedy leave rule pays off and Bug2 is shortest.',
};

/**
 * A three-turn rectilinear spiral with the goal in its innermost channel. The
 * m-line from the start crosses the spiral's left arms six times. Turning right
 * at every hit, Bug2 leaves at the far side of each arm and walks nearly the
 * whole boundary it has already seen again for the next one; Bug1 pays the
 * perimeter once. This is Choset's figure 2.4 argument made concrete.
 */
export const SPIRAL: BugPreset = {
  id: 'spiral',
  label: 'Spiral (C fig. 2.4: Bug2 loses)',
  world: polygonWorld(
    'spiral',
    { minX: -1, minY: -4.5, maxX: 11, maxY: 4.5 },
    [spiralPolygon(pt(6.5, 0), 1.0, 0.3, 3)],
  ),
  start: pt(0, -0.5),
  goal: pt(6.5, -0.5),
  range: 2,
  note: 'The boundary crosses the m-line many times. Bug2 (turning right) walks it over and over; Bug1 walks it once.',
};

/** The goal inside a closed box: the one case every Bug must get right by giving up. */
export const ENCLOSED: BugPreset = {
  id: 'enclosed',
  label: 'Enclosed goal (unreachable)',
  world: polygonWorld(
    'enclosed',
    { minX: -1, minY: -4, maxX: 11, maxY: 4 },
    // The goal lies inside the second block — a walled room drawn solid,
    // Choset's figure 2.2. A polygon with a hole is not a simple polygon, so
    // the room is its own interior: unreachable by construction.
    [rectangle(2, -1, 4, 1), rectangle(6, -2, 9, 2)],
  ),
  start: pt(0, 0),
  goal: pt(7.8, 0.6),
  range: 3,
  note: 'The goal is inside WO₂. Bug1 halts after one loop; Bug2 when it re-encounters its hit point; Tangent Bug when it closes a cycle.',
};

/**
 * Choset's figure 2.8. From x the heuristic prefers the top corner of WO₁
 * (O₂ in the figure) because d(x, O) + d(O, goal) is smallest there — but the
 * segment from that corner to the goal cuts through WO₂. With R = 3 the robot
 * cannot see WO₂ and drives toward O₂; with R = 6 it can, assigns O₂ infinite
 * cost, and drives toward the bottom corner instead.
 */
export const FIG_2_8: BugPreset = {
  id: 'fig-2-8',
  label: 'The heuristic\'s blind spot (C fig. 2.8)',
  world: polygonWorld(
    'fig-2-8',
    { minX: 0, minY: -1, maxX: 10, maxY: 7 },
    [rectangle(4, 0, 5, 3), rectangle(5.5, 3.5, 6.5, 5)],
  ),
  start: pt(2, 1),
  goal: pt(8, 6),
  range: 3,
  note: 'At R = 3 the top corner of WO₁ looks best; at R = 6 the robot sees that WO₂ blocks the way from it and picks the bottom corner.',
};

export const PRESETS: readonly BugPreset[] = [TWO_RECTANGLES, FIG_2_1, SPIRAL, ENCLOSED];

export const presetById = (id: PresetId): BugPreset => PRESETS.find((p) => p.id === id) ?? TWO_RECTANGLES;

/** A seeded random field of 2–5 disjoint convex obstacles between a fixed start and goal. */
export function randomPreset(seed: number): BugPreset {
  const rng = new Rng(seed);
  const start = pt(0.7, 0.7);
  const goal = pt(11.3, 7.3);
  const bounds = { minX: 0, minY: 0, maxX: 12, maxY: 8 };
  const count = 2 + Math.floor(rng.uniform(0, 4));
  const obstacles = randomConvexField(rng, bounds, {
    count,
    radius: [0.6, 1.7],
    margin: 0.45,
    keepFree: [start, goal],
  });
  return {
    id: 'random',
    label: `Random field (seed ${seed})`,
    world: polygonWorld(`random-${seed}`, bounds, obstacles),
    start,
    goal,
    range: 2.5,
    note: 'A seeded field of disjoint convex obstacles. Both bounds hold on every seed.',
  };
}

/**
 * A smooth nonconvex obstacle for the Boundary Follower: a 96-gon sampled from
 * r(θ) = a − b·exp(−((θ − π)/σ)²), a disc with a Gaussian dent on its west
 * side. The dent's radius of curvature at its deepest point is small, so an
 * offset W* larger than it makes the offset set grow a cusp — the implicit
 * function theorem's hypothesis failing exactly where the widget can show it.
 */
export function kidneyPolygon(center: Point2, a = 1.5, b = 0.95, sigma = 0.55, n = 96): import('./geometry').Polygon {
  const pts: Point2[] = [];
  for (let i = 0; i < n; i++) {
    const th = (2 * Math.PI * i) / n;
    const d = Math.atan2(Math.sin(th - Math.PI), Math.cos(th - Math.PI));
    const r = a - b * Math.exp(-(d * d) / (sigma * sigma));
    pts.push(pt(center.x + r * Math.cos(th), center.y + r * Math.sin(th)));
  }
  return { pts };
}

export const KIDNEY: PolygonWorld = polygonWorld('kidney', { minX: -3, minY: -3, maxX: 3, maxY: 3 }, [kidneyPolygon(pt(0, 0))]);
