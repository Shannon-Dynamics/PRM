/**
 * Bug1 and Bug2 on a text world — Chapter 1's forty-line hook.
 *
 * DELIBERATELY MINIMAL. This is the 4-connected caricature of Choset's
 * Algorithms 1 and 2 that `crates/ch01_hello` prints, written so that the
 * number on the page (28 steps) and the animation in w1.1 are one computation.
 * Chapter 3 owns the real Bug algorithms — continuous boundary following,
 * Tangent Bug, the range sensor, the proofs — in `lib/bugs`. Nothing here is
 * meant to be reused by a later chapter; it exists so Chapter 1 can show a
 * complete planner before it has defined a configuration space.
 *
 * The world: '#' is obstacle, 'S' start, 'G' goal, anything else free. The
 * robot occupies one cell and moves to 4-neighbors. Its contact sensor
 * reports whether the next cell is blocked (or outside the bounded world).
 * The boundary it follows is the ring of free cells 8-adjacent to the
 * obstacle, kept on the robot's right hand.
 */

import type { Point2 } from '../sim/world';
import { polygonsIntersect, rect, type Polygon } from '../collide/polygon';

export interface Cell {
  x: number;
  y: number;
}

/** Headings in screen coordinates (y grows downward): E, S, W, N. Turning right adds one. */
export type Dir = 0 | 1 | 2 | 3;
const DX = [1, 0, -1, 0];
const DY = [0, 1, 0, -1];
const right = (d: Dir): Dir => ((d + 1) % 4) as Dir;
const left = (d: Dir): Dir => ((d + 3) % 4) as Dir;
const back = (d: Dir): Dir => ((d + 2) % 4) as Dir;

export const sameCell = (a: Cell, b: Cell): boolean => a.x === b.x && a.y === b.y;

/** Euclidean distance d(x, y) — Bug1 may measure it, by assumption (C §2.1). */
export function dist(a: Cell, b: Cell): number {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export class TextWorld {
  readonly width: number;
  readonly height: number;
  private readonly cells: Uint8Array;

  constructor(width: number, height: number, blocked?: Uint8Array) {
    this.width = width;
    this.height = height;
    this.cells = blocked ?? new Uint8Array(width * height);
  }

  /** Parse a text map; returns the world and the S and G cells. */
  static parse(s: string): { world: TextWorld; start: Cell; goal: Cell } {
    const rows = s
      .split('\n')
      .map((r) => r.trimEnd())
      .filter((r) => r.length > 0);
    const height = rows.length;
    const width = Math.max(...rows.map((r) => r.length));
    const world = new TextWorld(width, height);
    let start: Cell = { x: 0, y: 0 };
    let goal: Cell = { x: width - 1, y: 0 };
    rows.forEach((row, y) => {
      for (let x = 0; x < row.length; x++) {
        const ch = row[x];
        if (ch === '#') world.cells[y * width + x] = 1;
        else if (ch === 'S') start = { x, y };
        else if (ch === 'G') goal = { x, y };
      }
    });
    return { world, start, goal };
  }

  /** The contact sensor: blocked, or outside the bounded workspace. */
  blocked(c: Cell): boolean {
    if (c.x < 0 || c.y < 0 || c.x >= this.width || c.y >= this.height) return true;
    return this.cells[c.y * this.width + c.x] === 1;
  }

  setBlocked(c: Cell, v: boolean): void {
    if (c.x < 0 || c.y < 0 || c.x >= this.width || c.y >= this.height) return;
    this.cells[c.y * this.width + c.x] = v ? 1 : 0;
  }

  /** Every blocked cell, for drawing. */
  blockedCells(): Cell[] {
    const out: Cell[] = [];
    for (let y = 0; y < this.height; y++) {
      for (let x = 0; x < this.width; x++) if (this.cells[y * this.width + x]) out.push({ x, y });
    }
    return out;
  }

  /**
   * Rasterize filled polygons (in world metres) onto a grid of `cell` metres:
   * a grid cell is blocked when its square touches any polygon. Used by w1.1
   * to run the text-world Bug on a continuous-looking map.
   */
  static rasterize(
    bounds: { minX: number; minY: number; maxX: number; maxY: number },
    cell: number,
    polys: Polygon[],
  ): TextWorld {
    const width = Math.round((bounds.maxX - bounds.minX) / cell);
    const height = Math.round((bounds.maxY - bounds.minY) / cell);
    const w = new TextWorld(width, height);
    for (let j = 0; j < height; j++) {
      for (let i = 0; i < width; i++) {
        const x0 = bounds.minX + i * cell;
        // Row 0 is the *top* of the map, so grid y runs opposite to world y.
        const y0 = bounds.maxY - (j + 1) * cell;
        const sq = rect(x0, y0, x0 + cell, y0 + cell);
        if (polys.some((p) => polygonsIntersect(sq, p))) w.cells[j * width + i] = 1;
      }
    }
    return w;
  }
}

/** Cell ↔ world conversions for a rasterized map (cell centers). */
export function cellCenter(
  bounds: { minX: number; maxY: number },
  cell: number,
  c: Cell,
): Point2 {
  return { x: bounds.minX + (c.x + 0.5) * cell, y: bounds.maxY - (c.y + 0.5) * cell };
}

export function cellAt(bounds: { minX: number; maxY: number }, cell: number, p: Point2): Cell {
  return { x: Math.floor((p.x - bounds.minX) / cell), y: Math.floor((bounds.maxY - p.y) / cell) };
}

// ---------------------------------------------------------------------------
// The two behaviors
// ---------------------------------------------------------------------------

export type Step = { kind: 'move'; cell: Cell } | { kind: 'hit'; dir: Dir };

/**
 * motion-to-goal: the 4-neighbor that most reduces d(·, goal). When that
 * neighbor is blocked, the robot has hit the obstacle and reports the heading
 * it was pushing in. O(1).
 */
export function motionToGoal(w: TextWorld, x: Cell, goal: Cell): Step {
  let bestD = Infinity;
  let bestDir: Dir = 0;
  for (let d = 0; d < 4; d++) {
    const n = { x: x.x + DX[d], y: x.y + DY[d] };
    const dd = dist(n, goal);
    if (dd < bestD) {
      bestD = dd;
      bestDir = d as Dir;
    }
  }
  const n = { x: x.x + DX[bestDir], y: x.y + DY[bestDir] };
  return w.blocked(n) ? { kind: 'hit', dir: bestDir } : { kind: 'move', cell: n };
}

/**
 * One step of boundary following with the obstacle on the robot's right hand:
 * turn toward the wall if that cell is free (the wall turned away), else go
 * straight, else turn away, else reverse. O(1) sensing per step.
 */
export function followBoundary(w: TextWorld, x: Cell, heading: Dir): { cell: Cell; heading: Dir } {
  for (const h of [right(heading), heading, left(heading), back(heading)]) {
    const n = { x: x.x + DX[h], y: x.y + DY[h] };
    if (!w.blocked(n)) return { cell: n, heading: h };
  }
  return { cell: x, heading }; // boxed in on all four sides; cannot happen for a free start
}

/**
 * Circumnavigate: from the hit point, with the obstacle to the right, follow
 * the boundary until the hit point is re-encountered. Returns the cells
 * visited *after* the hit point, ending with the hit point itself — so the
 * array's length is the perimeter p_i as the robot measures it.
 */
export function circumnavigate(w: TextWorld, hit: Cell, dirToObstacle: Dir, maxSteps: number): Cell[] | null {
  const ring: Cell[] = [];
  let cell = hit;
  let heading = left(dirToObstacle);
  for (let k = 0; k < maxSteps; k++) {
    const next = followBoundary(w, cell, heading);
    cell = next.cell;
    heading = next.heading;
    ring.push(cell);
    if (sameCell(cell, hit)) return ring;
  }
  return null;
}

// ---------------------------------------------------------------------------
// Bug1 (C Algorithm 1) and Bug2 (C Algorithm 2)
// ---------------------------------------------------------------------------

export interface BugResult {
  outcome: 'reached' | 'unreachable';
  /** Every cell visited, start first. `path.length − 1` is the step count. */
  path: Cell[];
  hits: Cell[];
  leaves: Cell[];
  /** Perimeter of each obstacle met, in ring cells (Bug1 only — Bug2 never completes a loop). */
  perimeters: number[];
  /** d(q_start, q_goal), Euclidean. */
  straightLine: number;
  /**
   * |Δx| + |Δy|: the length of the grid's "straight line". On a 4-connected
   * grid motion-to-goal is a staircase, so this — not the Euclidean d — is what
   * the robot actually walks between obstacles. Equal to d on the worked example.
   */
  manhattan: number;
  /**
   * d₁ + 1.5 Σ p_i for Bug1 — C eq. (2.1) with the grid's straight-line length
   * d₁ in place of d (they coincide on the worked example). NaN for Bug2, whose
   * bound needs the m-line crossing counts Chapter 3 defines.
   */
  bound: number;
  /** Where the planner gave up, if it did. */
  stuckAt?: Cell;
}

export function bug1(w: TextWorld, start: Cell, goal: Cell): BugResult {
  const path: Cell[] = [start];
  const hits: Cell[] = [];
  const leaves: Cell[] = [];
  const perimeters: number[] = [];
  const straightLine = dist(start, goal);
  const manhattan = Math.abs(goal.x - start.x) + Math.abs(goal.y - start.y);
  const maxSteps = 8 * w.width * w.height + 16;
  let x = start;
  const finish = (outcome: BugResult['outcome'], stuckAt?: Cell): BugResult => ({
    outcome,
    path,
    hits,
    leaves,
    perimeters,
    straightLine,
    manhattan,
    bound: manhattan + 1.5 * perimeters.reduce((a, b) => a + b, 0),
    stuckAt,
  });

  let guard = 0;
  while (!sameCell(x, goal) && guard++ < maxSteps) {
    const step = motionToGoal(w, x, goal);
    if (step.kind === 'move') {
      x = step.cell;
      path.push(x);
      continue;
    }

    // Contact. Remember q^H_i, then walk the whole boundary.
    const hit = x;
    if (hits.some((h) => sameCell(h, hit))) return finish('unreachable', hit);
    hits.push(hit);
    const ring = circumnavigate(w, hit, step.dir, maxSteps);
    if (!ring) return finish('unreachable', hit);
    path.push(...ring);
    perimeters.push(ring.length);

    // The leave point: the perimeter cell closest to the goal (first one wins a tie).
    let bestI = ring.length - 1;
    let bestD = dist(hit, goal);
    for (let i = 0; i < ring.length; i++) {
      const d = dist(ring[i], goal);
      if (d < bestD - 1e-12) {
        bestD = d;
        bestI = i;
      }
    }
    const leave = ring[bestI];
    leaves.push(leave);

    // Go to q^L_i the shorter way round; a tie goes forward (clockwise).
    const forward = bestI + 1;
    const backward = ring.length - 1 - bestI;
    if (forward <= backward) {
      for (let i = 0; i <= bestI; i++) path.push(ring[i]);
    } else {
      for (let i = ring.length - 2; i >= bestI; i--) path.push(ring[i]);
    }
    x = leave;

    // If moving toward the goal would re-enter the obstacle, there is no path.
    const probe = motionToGoal(w, x, goal);
    if (probe.kind === 'hit') {
      let escape: Cell | null = null;
      for (let d = 0; d < 4; d++) {
        const n = { x: x.x + DX[d], y: x.y + DY[d] };
        if (!w.blocked(n) && dist(n, goal) < dist(x, goal)) escape = n;
      }
      if (!escape) return finish('unreachable', x);
      // A free neighbour still gets closer: the grid's version of a grazing exit.
      x = escape;
      path.push(x);
    }
  }
  return finish(sameCell(x, goal) ? 'reached' : 'unreachable', sameCell(x, goal) ? undefined : x);
}

/** Cells whose centers lie within half a cell of the segment start→goal: the m-line. */
export function onMLine(c: Cell, start: Cell, goal: Cell): boolean {
  const dx = goal.x - start.x;
  const dy = goal.y - start.y;
  const len2 = dx * dx + dy * dy;
  if (len2 === 0) return sameCell(c, start);
  const t = ((c.x - start.x) * dx + (c.y - start.y) * dy) / len2;
  if (t < -1e-9 || t > 1 + 1e-9) return false;
  const px = start.x + t * dx;
  const py = start.y + t * dy;
  return Math.hypot(c.x - px, c.y - py) <= 0.5 + 1e-9;
}

export function bug2(w: TextWorld, start: Cell, goal: Cell): BugResult {
  const path: Cell[] = [start];
  const hits: Cell[] = [];
  const leaves: Cell[] = [];
  const straightLine = dist(start, goal);
  const manhattan = Math.abs(goal.x - start.x) + Math.abs(goal.y - start.y);
  const maxSteps = 8 * w.width * w.height + 16;
  let x = start;
  const finish = (outcome: BugResult['outcome'], stuckAt?: Cell): BugResult => ({
    outcome,
    path,
    hits,
    leaves,
    perimeters: [],
    straightLine,
    manhattan,
    bound: NaN, // Bug2's bound needs n_i, the m-line crossings; Chapter 3 computes it.
    stuckAt,
  });

  let guard = 0;
  while (!sameCell(x, goal) && guard++ < maxSteps) {
    const step = motionToGoal(w, x, goal);
    if (step.kind === 'move') {
      x = step.cell;
      path.push(x);
      continue;
    }
    const hit = x;
    hits.push(hit);
    const dHit = dist(hit, goal);
    let cell = hit;
    let heading = left(step.dir);
    let left_ = false;
    for (let k = 0; k < maxSteps; k++) {
      const next = followBoundary(w, cell, heading);
      cell = next.cell;
      heading = next.heading;
      path.push(cell);
      if (sameCell(cell, goal)) {
        x = cell;
        left_ = true;
        break;
      }
      if (sameCell(cell, hit)) break; // back where we started: no path
      // Leave when back on the m-line, closer than the hit point, and able to step toward the goal.
      if (onMLine(cell, start, goal) && dist(cell, goal) < dHit - 1e-12) {
        const probe = motionToGoal(w, cell, goal);
        if (probe.kind === 'move') {
          leaves.push(cell);
          x = cell;
          left_ = true;
          break;
        }
      }
    }
    if (!left_) return finish('unreachable', hit);
  }
  return finish(sameCell(x, goal) ? 'reached' : 'unreachable', sameCell(x, goal) ? undefined : x);
}

/** Render the map with the path marked `·`, hit points `H`, leave points `L`. */
export function render(w: TextWorld, start: Cell, goal: Cell, r: BugResult): string {
  const rows: string[][] = [];
  for (let y = 0; y < w.height; y++) {
    const row: string[] = [];
    for (let x = 0; x < w.width; x++) row.push(w.blocked({ x, y }) ? '#' : '.');
    rows.push(row);
  }
  for (const c of r.path) rows[c.y][c.x] = '·';
  for (const c of r.hits) rows[c.y][c.x] = 'H';
  for (const c of r.leaves) rows[c.y][c.x] = 'L';
  rows[start.y][start.x] = 'S';
  rows[goal.y][goal.x] = 'G';
  return rows.map((r) => r.join('')).join('\n');
}

/** The chapter's 9 × 5 world, exactly as `crates/ch01_hello` has it. */
export const WORLD_9x5 = `
.........
...###...
S..###..G
...###...
.........
`;

/** The integration lab's edit: a 4 × 4 block in a 10 × 6 world. */
export const WORLD_4x4 = `
..........
...####...
S..####..G
...####...
...####...
..........
`;

/** The goal walled in — the unreachable test. */
export const WORLD_ENCLOSED = `
.........
.....###.
S....#G#.
.....###.
.........
`;

// ---------------------------------------------------------------------------
// Presets for w1.1 (polygons in metres on a 12 × 7.2 m board)
// ---------------------------------------------------------------------------

export const BUG_BOARD = { minX: 0, minY: 0, maxX: 12, maxY: 7.2 };
export const BUG_CELL = 0.15;

export type BugPreset = 'figure-2.1' | 'comb';

/**
 * Choset's figure 2.1: two blobs between start and goal. The seed nudges them.
 *
 * The comb: the world where Bug2 loses. A hook (a wall on the m-line with an
 * arm reaching east above it) followed by a row of tall teeth planted on the
 * m-line. Bug1 circumnavigates the hook once, finds that the perimeter point
 * nearest the goal is the *arm's tip*, high above the m-line, and leaves from
 * there on a fresh straight line that clears every tooth. Bug2 is welded to the
 * m-line: it exits the hook at m-line height and then must climb over each
 * tooth in turn. Greedy exits, paid for n times, cost more than one exhaustive
 * loop — which is C eq. (2.2)'s Σ nᵢpᵢ term made visible.
 */
export function bugPresetPolygons(preset: BugPreset, jitter: (lo: number, hi: number) => number): Polygon[] {
  if (preset === 'figure-2.1') {
    const ax = 3.6 + jitter(-0.3, 0.3);
    const ay = 3.9 + jitter(-0.3, 0.3);
    const bx = 7.6 + jitter(-0.3, 0.3);
    const by = 3.2 + jitter(-0.3, 0.3);
    return [
      [
        { x: ax - 1.0, y: ay - 1.4 },
        { x: ax + 1.1, y: ay - 1.0 },
        { x: ax + 1.3, y: ay + 0.6 },
        { x: ax + 0.2, y: ay + 1.3 },
        { x: ax - 1.2, y: ay + 0.5 },
      ],
      [
        { x: bx - 1.1, y: by - 0.9 },
        { x: bx + 0.9, y: by - 1.3 },
        { x: bx + 1.4, y: by + 0.4 },
        { x: bx + 0.3, y: by + 1.5 },
        { x: bx - 1.3, y: by + 0.9 },
      ],
    ];
  }
  // The hook: a wall through the m-line (y = 3.6) with an arm reaching east
  // above it. Its nearest point to the goal is the arm's tip.
  const hx = 3.0 + jitter(-0.3, 0.3);
  const hy = jitter(-0.2, 0.2);
  const hook: Polygon = [
    { x: hx, y: 2.8 + hy },
    { x: hx + 0.4, y: 2.8 + hy },
    { x: hx + 0.4, y: 5.2 + hy },
    { x: hx + 3.2, y: 5.2 + hy },
    { x: hx + 3.2, y: 5.6 + hy },
    { x: hx, y: 5.6 + hy },
  ];
  // Four teeth on the m-line, 1.0 m above it and 1.4 m below, 0.45 m apart.
  const teeth: Polygon[] = [];
  const tx0 = 6.9 + jitter(-0.15, 0.15);
  for (let k = 0; k < 4; k++) {
    const tx = tx0 + k * 0.75;
    teeth.push(rect(tx, 2.2, tx + 0.3, 4.6));
  }
  return [hook, ...teeth];
}

export const BUG_START: Point2 = { x: 0.9, y: 3.6 };
export const BUG_GOAL_DEFAULT: Point2 = { x: 11.1, y: 3.6 };
