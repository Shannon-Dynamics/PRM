'use client';

import { useCallback, useEffect, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawPath, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import {
  BUG_BOARD,
  BUG_CELL,
  BUG_GOAL_DEFAULT,
  BUG_START,
  bug1,
  bug2,
  bugPresetPolygons,
  cellAt,
  cellCenter,
  TextWorld,
  type BugPreset,
  type BugResult,
} from '@/lib/robots/bug-preview';
import type { Polygon } from '@/lib/collide/polygon';
import type { Point2 } from '@/lib/sim/world';

/**
 * w1.1 — Bug Preview: a complete planner in forty lines, before any mathematics.
 *
 * A point robot with a contact sensor runs Bug1 (or Bug2) around two obstacles
 * toward a goal it cannot see past. The m-line is faint, hit and leave points
 * are labeled as in Choset's figure 2.1, and the counter under the canvas
 * never exceeds the bound d + 1.5 Σ pᵢ. The misconception killed: "a planner
 * without a map is a heuristic". It is a theorem with a contact sensor.
 *
 * This runs the deliberately minimal grid Bug of `lib/robots/bug-preview.ts`
 * — the same code that prints the chapter's 28-step text world. Chapter 3 owns
 * the real Bug algorithms; this is the hook.
 */

type Algo = 'bug1' | 'bug2';

interface State {
  polys: Polygon[];
  world: TextWorld;
  start: Point2;
  goal: Point2;
  run: BugResult;
  other: BugResult;
  cursor: number;
  hold: number;
}

const HOLD_TICKS = 50;

export function BugPreview() {
  const [goal, setGoal] = useState<Point2>(BUG_GOAL_DEFAULT);
  const [preset, setPreset] = useState<BugPreset>('figure-2.1');
  const [algo, setAlgo] = useState<Algo>('bug1');
  const [dragging, setDragging] = useState(false);

  const init = useCallback(
    (seed: number): State => {
      const rng = new Rng(seed);
      const polys = bugPresetPolygons(preset, (lo, hi) => rng.uniform(lo, hi));
      const world = TextWorld.rasterize(BUG_BOARD, BUG_CELL, polys);
      const s = cellAt(BUG_BOARD, BUG_CELL, BUG_START);
      let g = cellAt(BUG_BOARD, BUG_CELL, goal);
      // A goal dragged into an obstacle snaps to the nearest free cell on its row.
      if (world.blocked(g)) {
        for (let k = 1; k < world.width && world.blocked(g); k++) {
          const left = { x: g.x - k, y: g.y };
          const right = { x: g.x + k, y: g.y };
          if (!world.blocked(right)) g = right;
          else if (!world.blocked(left)) g = left;
        }
      }
      const a = bug1(world, s, g);
      const b = bug2(world, s, g);
      return {
        polys,
        world,
        start: cellCenter(BUG_BOARD, BUG_CELL, s),
        goal: cellCenter(BUG_BOARD, BUG_CELL, g),
        run: algo === 'bug1' ? a : b,
        other: algo === 'bug1' ? b : a,
        cursor: 0,
        hold: 0,
      };
    },
    [goal, preset, algo],
  );

  const step = useCallback((s: State): State => {
    if (s.cursor < s.run.path.length - 1) return { ...s, cursor: s.cursor + 1 };
    if (s.hold < HOLD_TICKS) return { ...s, hold: s.hold + 1 };
    return { ...s, cursor: 0, hold: 0 };
  }, []);

  const sim = useSimulation<State>({ init, step, fps: 40, initialSeed: 42 });
  const { reset } = sim;
  // A new goal, preset, or algorithm is a new planning problem: start over.
  useEffect(() => {
    reset();
  }, [reset]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') return setDragging(false);
      if (phase === 'down') {
        const d = Math.hypot(w[0] - goal.x, w[1] - goal.y);
        if (d > 0.6) return;
        setDragging(true);
      } else if (!dragging) return;
      setGoal({
        x: Math.max(BUG_BOARD.minX + 0.3, Math.min(BUG_BOARD.maxX - 0.3, w[0])),
        y: Math.max(BUG_BOARD.minY + 0.3, Math.min(BUG_BOARD.maxY - 0.3, w[1])),
      });
    },
    [dragging, goal],
  );

  const s = sim.state;
  const toWorld = (c: { x: number; y: number }) => cellCenter(BUG_BOARD, BUG_CELL, c);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);

      // Workspace obstacles, slate — drawn as the polygons they are, not as cells.
      ctx.save();
      ctx.fillStyle = p.obstacle;
      for (const poly of s.polys) {
        ctx.beginPath();
        poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
        ctx.closePath();
        ctx.fill();
      }
      ctx.restore();

      // The m-line, faint.
      ctx.save();
      ctx.strokeStyle = p.unknown;
      ctx.setLineDash([4, 5]);
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(sx(v, s.start.x), sy(v, s.start.y));
      ctx.lineTo(sx(v, s.goal.x), sy(v, s.goal.y));
      ctx.stroke();
      ctx.restore();

      // Executed path so far, in the robot's orange.
      const path = s.run.path.slice(0, s.cursor + 1).map(toWorld);
      drawPath(ctx, v, path, p.robot, { lineWidth: 2.2, alpha: 0.9 });

      // Hit and leave points, labeled as in C figure 2.1.
      s.run.hits.forEach((h, i) => {
        const w = toWorld(h);
        if (indexOfCell(s.run.path, h) > s.cursor) return;
        dot(ctx, sx(v, w.x), sy(v, w.y), 3.5, p.ink);
        label(ctx, `qH${i + 1}`, sx(v, w.x) - 6, sy(v, w.y) - 11, p.ink, { size: 10, weight: 600 });
      });
      s.run.leaves.forEach((l, i) => {
        const w = toWorld(l);
        if (indexOfCell(s.run.path, l) > s.cursor) return;
        dot(ctx, sx(v, w.x), sy(v, w.y), 3.5, p.ink);
        label(ctx, `qL${i + 1}`, sx(v, w.x) + 6, sy(v, w.y) + 12, p.ink, { size: 10, weight: 600 });
      });

      // Start, goal, robot.
      dot(ctx, sx(v, s.start.x), sy(v, s.start.y), 6, p.start);
      label(ctx, 'q_start', sx(v, s.start.x) - 8, sy(v, s.start.y) + 16, p.start, { size: 10, weight: 600 });
      dot(ctx, sx(v, s.goal.x), sy(v, s.goal.y), 7, p.goal);
      label(ctx, 'q_goal', sx(v, s.goal.x) - 8, sy(v, s.goal.y) + 16, p.goal, { size: 10, weight: 600 });
      const cur = toWorld(s.run.path[Math.min(s.cursor, s.run.path.length - 1)]);
      dot(ctx, sx(v, cur.x), sy(v, cur.y), Math.max(sl(v, BUG_CELL * 0.45), 4), p.robot);

      if (s.run.outcome === 'unreachable' && s.cursor >= s.run.path.length - 1) {
        label(ctx, 'goal unreachable — reported after one circumnavigation', 12, 16, p.goal, { size: 11, weight: 700 });
      }
    },
    [s],
  );

  const steps = Math.min(s.cursor, s.run.path.length - 1);
  const total = s.run.path.length - 1;
  const perim = s.run.perimeters.reduce((a, b) => a + b, 0);

  return (
    <WidgetFrame
      id="w1.1"
      title="Bug Preview"
      teaches="A contact sensor and two behaviors give a complete planner with a length bound — no map, no heuristic."
      colorKey={['start', 'goal', 'obstacle', 'robot']}
      caption={
        <>
          Bug1, Choset's Algorithm 1, on a point robot that can only feel what it touches: walk toward
          the goal; on contact, go all the way around, remembering the perimeter point closest to the
          goal; return to it and set off again. The counter tallies grid steps; the bound beside it is
          Choset's $d + 1.5\sum p_i$ with the grid's own straight-line length, and the counter never
          passes it. Drag the goal behind an obstacle: the circumnavigation gets longer, the guarantee
          does not budge. Switch to Bug2 (which leaves the first time it meets the m-line closer to
          the goal) and compare the two totals; then choose the <em>comb</em>, where Bug2's greed
          finally costs more than Bug1's patience.
        </>
      }
    >
      <SimCanvas
        world={BUG_BOARD}
        draw={draw}
        deps={[sim.tick, s]}
        aspect={12 / 7.2}
        padding={0.2}
        onPointer={onPointer}
        cursor={dragging ? 'grabbing' : 'grab'}
        ariaLabel="A point robot crawling around slate obstacles from a green start to a red goal, leaving an orange trail; hit and leave points are labeled."
      />

      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border text-center font-mono text-[0.72rem] tabular-nums sm:grid-cols-4">
        <Stat label={`${algo === 'bug1' ? 'Bug1' : 'Bug2'} steps`} value={`${steps} / ${total}`} />
        <Stat label="straight line d (grid d₁)" value={`${s.run.straightLine.toFixed(1)} (${s.run.manhattan})`} />
        <Stat label="bound d₁ + 1.5 Σpᵢ" value={algo === 'bug1' ? `${s.run.bound.toFixed(0)}  (Σpᵢ = ${perim})` : `${s.other.bound.toFixed(0)} for Bug1`} />
        <Stat label={algo === 'bug1' ? 'Bug2 on this world' : 'Bug1 on this world'} value={`${s.other.path.length - 1} steps`} />
      </div>

      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Advanced: algorithm, obstacle preset</summary>
        <ControlPanel columns={2}>
          <ButtonRow>
            <ActionButton emphasis={algo === 'bug1'} onClick={() => setAlgo('bug1')}>
              Bug1 (exhaustive)
            </ActionButton>
            <ActionButton emphasis={algo === 'bug2'} onClick={() => setAlgo('bug2')}>
              Bug2 (greedy)
            </ActionButton>
          </ButtonRow>
          <ButtonRow>
            <ActionButton emphasis={preset === 'figure-2.1'} onClick={() => setPreset('figure-2.1')}>
              Choset's figure 2.1
            </ActionButton>
            <ActionButton emphasis={preset === 'comb'} onClick={() => setPreset('comb')}>
              The comb (Bug2 loses)
            </ActionButton>
          </ButtonRow>
        </ControlPanel>
      </details>

      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => sim.reseed()}
        seed={sim.seed}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}

function Stat({ label: l, value }: { label: string; value: string }) {
  return (
    <div className="px-2 py-1.5">
      <div className="eyebrow">{l}</div>
      <div className="text-sm">{value}</div>
    </div>
  );
}

function dot(ctx: CanvasRenderingContext2D, x: number, y: number, r: number, color: string) {
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function indexOfCell(path: { x: number; y: number }[], c: { x: number; y: number }): number {
  for (let i = 0; i < path.length; i++) if (path[i].x === c.x && path[i].y === c.y) return i;
  return Infinity;
}
