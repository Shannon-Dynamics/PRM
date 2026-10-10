/**
 * Numerical self-checks for Chapter 1 — Robots That Plan.
 *
 * The chapter's one worked number is the 28-step Bug1 path on the 9 × 5 text
 * world; everything else is the invariant it illustrates (the bound of
 * C eq. 2.1) and the Piano Mover's narrow passage, pinned so that what the
 * reader watches is what the prose claims.
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { orientedRect, polygonsIntersectBrute, rect, segmentIntersectsPolygon } from '../collide/polygon';
import { Scene } from '../collide/scene';
import {
  BUG_BOARD,
  BUG_CELL,
  BUG_GOAL_DEFAULT,
  BUG_START,
  bug1,
  bug2,
  bugPresetPolygons,
  cellAt,
  TextWorld,
  WORLD_4x4,
  WORLD_9x5,
  WORLD_ENCLOSED,
} from './bug-preview';

export function runCh01Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The worked example: 28 steps, perimeter 16, bound 32
  // -------------------------------------------------------------------------
  const nine = TextWorld.parse(WORLD_9x5);
  const r1 = bug1(nine.world, nine.start, nine.goal);
  {
    const steps = r1.path.length - 1;
    const ok =
      r1.outcome === 'reached' &&
      steps === 28 &&
      r1.perimeters.length === 1 &&
      r1.perimeters[0] === 16 &&
      r1.hits.length === 1 &&
      r1.hits[0].x === 2 &&
      r1.hits[0].y === 2 &&
      r1.leaves[0].x === 6 &&
      r1.leaves[0].y === 2 &&
      Math.abs(r1.straightLine - 8) < 1e-12 &&
      Math.abs(r1.bound - 32) < 1e-12;
    push(
      'worked_example_ch01: Bug1 on the 9×5 world takes 28 steps, p₁ = 16, H = (2,2), L = (6,2), bound 32',
      ok,
      `steps ${steps}, perimeters [${r1.perimeters.join(', ')}], H (${r1.hits[0]?.x},${r1.hits[0]?.y}), L (${r1.leaves[0]?.x},${r1.leaves[0]?.y}), bound ${r1.bound}`,
    );
    push('bug1 bound holds on the worked example: 28 ≤ 8 + 1.5 · 16', steps <= r1.bound);
  }

  {
    const r2 = bug2(nine.world, nine.start, nine.goal);
    const steps = r2.path.length - 1;
    push(
      'bug2_text_world: Bug2 leaves at (6,2) after 8 boundary steps for 2 + 8 + 2 = 12',
      r2.outcome === 'reached' && steps === 12 && r2.leaves[0]?.x === 6 && r2.leaves[0]?.y === 2,
      `steps ${steps}`,
    );
  }

  {
    const enc = TextWorld.parse(WORLD_ENCLOSED);
    const r = bug1(enc.world, enc.start, enc.goal);
    const steps = r.path.length - 1;
    // One circumnavigation of the 3×3 enclosure is a ring of 16 cells; the
    // planner must give up within one loop plus the walk to the leave point.
    push(
      'unreachable_is_detected: a walled-in goal is reported Unreachable within one circumnavigation',
      r.outcome === 'unreachable' && r.perimeters.length === 1 && steps <= r.straightLine + 1.5 * r.perimeters[0] + 1,
      `${r.outcome} after ${steps} steps, perimeter ${r.perimeters[0]}, stuck at (${r.stuckAt?.x},${r.stuckAt?.y})`,
    );
  }

  {
    // The integration lab's edit: a 4×4 block in a 10×6 world. The ring has 20
    // cells and the shorter way round from H to L is 9 steps (the block is not
    // symmetric about the m-line), so the run is 2 + 20 + 9 + 2 = 33 ≤ 9 + 30.
    const four = TextWorld.parse(WORLD_4x4);
    const r = bug1(four.world, four.start, four.goal);
    const steps = r.path.length - 1;
    push(
      'integration lab: the 4×4 block gives 33 steps, perimeter 20, against a bound of 39',
      r.outcome === 'reached' && steps === 33 && r.perimeters[0] === 20 && Math.abs(r.bound - 39) < 1e-12,
      `steps ${steps}, perimeter ${r.perimeters[0]}, bound ${r.bound}`,
    );
  }

  // -------------------------------------------------------------------------
  // The bound is a theorem, not a coincidence: random single-block worlds
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(1);
    let ok = true;
    let runs = 0;
    let worstSlack = Infinity;
    for (let k = 0; k < 120; k++) {
      const W = 12 + Math.floor(rng.uniform(0, 8));
      const H = 7 + Math.floor(rng.uniform(0, 6));
      const w = new TextWorld(W, H);
      const bw = 1 + Math.floor(rng.uniform(0, 5));
      const bh = 1 + Math.floor(rng.uniform(0, 4));
      const bx = 3 + Math.floor(rng.uniform(0, W - bw - 6));
      const by = 1 + Math.floor(rng.uniform(0, H - bh - 2));
      for (let y = by; y < by + bh; y++) for (let x = bx; x < bx + bw; x++) w.setBlocked({ x, y }, true);
      const start = { x: 0, y: Math.floor(rng.uniform(0, H)) };
      const goal = { x: W - 1, y: Math.floor(rng.uniform(0, H)) };
      const r = bug1(w, start, goal);
      runs++;
      const steps = r.path.length - 1;
      if (r.outcome !== 'reached') ok = false;
      if (steps > r.bound + 1e-9) ok = false;
      // Every step must be a free 4-move.
      for (let i = 1; i < r.path.length; i++) {
        const a = r.path[i - 1], b = r.path[i];
        if (Math.abs(a.x - b.x) + Math.abs(a.y - b.y) !== 1 || w.blocked(b)) ok = false;
      }
      worstSlack = Math.min(worstSlack, r.bound - steps);
    }
    push(
      'bug1: on 120 seeded single-block worlds every run reaches the goal by free 4-moves within d + 1.5 Σ pᵢ',
      ok && runs === 120,
      `tightest slack ${worstSlack.toFixed(2)} steps`,
    );
  }

  // -------------------------------------------------------------------------
  // The presets w1.1 ships: Bug2 wins on figure 2.1 and loses on the comb
  // -------------------------------------------------------------------------
  {
    // One fresh stream per preset, exactly as w1.1 draws its default (seed 42).
    const run = (preset: 'figure-2.1' | 'comb') => {
      const rng = new Rng(42);
      const polys = bugPresetPolygons(preset, (lo, hi) => rng.uniform(lo, hi));
      const world = TextWorld.rasterize(BUG_BOARD, BUG_CELL, polys);
      const s = cellAt(BUG_BOARD, BUG_CELL, BUG_START);
      const g = cellAt(BUG_BOARD, BUG_CELL, BUG_GOAL_DEFAULT);
      const a = bug1(world, s, g);
      const b = bug2(world, s, g);
      return { a: a.path.length - 1, b: b.path.length - 1, ok: a.outcome === 'reached' && b.outcome === 'reached', bound: a.bound };
    };
    const fig = run('figure-2.1');
    const comb = run('comb');
    push(
      'presets: Bug2 beats Bug1 on figure 2.1 but loses to it on the comb; Bug1 stays under its bound on both',
      fig.ok && comb.ok && fig.b < fig.a && comb.b > comb.a && fig.a <= fig.bound && comb.a <= comb.bound,
      `figure 2.1: Bug1 ${fig.a} vs Bug2 ${fig.b} (bound ${fig.bound.toFixed(0)}); comb: Bug1 ${comb.a} vs Bug2 ${comb.b} (bound ${comb.bound.toFixed(0)})`,
    );
    // The counts the prose quotes for the default seed.
    push(
      'presets at seed 42: figure 2.1 is Bug1 262 / Bug2 114 steps; the comb is Bug1 186 / Bug2 198',
      fig.a === 262 && fig.b === 114 && comb.a === 186 && comb.b === 198,
      `${fig.a}/${fig.b}, ${comb.a}/${comb.b}`,
    );
  }

  // -------------------------------------------------------------------------
  // Piano Mover: the first narrow passage
  // -------------------------------------------------------------------------
  {
    // A room split by a wall at x = 4 with a 1.0 m doorway centered on y = 2.4;
    // a 1.6 × 0.6 sofa. Nose-first (θ = 0) the sofa's 0.6 m width fits the
    // doorway; broadside (θ = π/2) its 1.6 m length cannot.
    const doorW = 1.0;
    const walls = [
      { x1: 4, y1: 0, x2: 4, y2: 2.4 - doorW / 2 },
      { x1: 4, y1: 2.4 + doorW / 2, x2: 4, y2: 4.8 },
    ];
    const scene = Scene.ofWalls(walls, { minX: 0, minY: 0, maxX: 8, maxY: 4.8 });
    const sofa = (x: number, y: number, th: number) => orientedRect(x, y, 0.8, 0.3, th);
    const noseFirst = !scene.intersectsPolygon(sofa(4, 2.4, 0));
    let broadsideFree = false;
    for (let y = 0.9; y <= 3.9; y += 0.05) if (!scene.intersectsPolygon(sofa(4, y, Math.PI / 2))) broadsideFree = true;
    push(
      'piano mover: a 1.6 × 0.6 sofa passes a 1.0 m doorway nose-first (θ = 0) and at no height broadside (θ = π/2)',
      noseFirst && !broadsideFree,
    );

    // Polygon-vs-wall collision agrees with a dense sampling of the footprint's edges.
    const rng = new Rng(9);
    let agree = 0;
    const N = 300;
    for (let k = 0; k < N; k++) {
      const poly = sofa(rng.uniform(3, 5), rng.uniform(0.5, 4.3), rng.uniform(0, Math.PI));
      const fast = scene.intersectsPolygon(poly);
      let slow = false;
      for (const w of walls) {
        if (segmentIntersectsPolygon(w, poly)) slow = true;
      }
      // Brute force: the wall as a degenerate thin rectangle, through the general edge test.
      let slow2 = false;
      for (const w of walls) {
        const thin = rect(w.x1 - 1e-9, Math.min(w.y1, w.y2), w.x2 + 1e-9, Math.max(w.y1, w.y2));
        if (polygonsIntersectBrute(poly, thin)) slow2 = true;
      }
      if (fast === slow && slow === slow2) agree++;
    }
    push('piano mover: footprint-vs-wall collision agrees with two brute-force formulations on 300 poses', agree === N, `${agree}/${N}`);
  }

  return out;
}
