'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawSegments, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { polygonArea } from '@/lib/collide/polygon';
import { Scene } from '@/lib/collide/scene';
import { DiscChecker } from '@/lib/collide/collision';
import { planeChart, rasterizeCobstacle, shortestCellPath } from '@/lib/cspace/raster';
import { inflateDisc, inflateSegment, inflatedArea } from '@/lib/cspace/star';
import { DOORWAY_GOAL, DOORWAY_ROOM, DOORWAY_START } from '@/lib/cspace/examples';
import { dot, drawRaster, fillPoly, fmt, hatchFillPoly, rasterImage } from './shared';

/**
 * w4.4 — Disc Inflation: Choset's figure 3.5 with numbers.
 *
 * Three discs — a point, Rusty (r = 0.11 m), and a large one you set — in one
 * room with a 0.6 m doorway and a convex crate. Each column shows the
 * workspace above and the configuration space below: the obstacles grown by
 * r (polygon edges offset, corners rounded; walls become capsules), hatched
 * amber, and the same start-to-goal query. The route is the shortest cell path
 * through the raster of a DiscChecker, so the picture and the query are the
 * same computation.
 *
 * The misconception addressed is the one that is *right*: "inflate by the
 * radius and plan for a point" is exactly correct — and correct only for
 * discs. A rotating body's C-obstacle is w4.2's.
 */

const CELL = 0.05;
const GAP = 5.6; // vertical offset of the C-space pane above the workspace
const WORLD = { minX: 0, minY: 0, maxX: 4, maxY: 5 + GAP };

interface Column {
  r: number;
  grid: ReturnType<typeof rasterizeCobstacle>;
  chart: ReturnType<typeof planeChart>;
  freeComps: number;
  freeArea: number;
  path: { x: number; y: number }[] | null;
  inflated: { x: number; y: number }[][];
  crateArea: number;
}

function buildColumn(r: number): Column {
  const scene = Scene.fromWorld(DOORWAY_ROOM);
  const b = DOORWAY_ROOM.bounds;
  const chart = planeChart(b, Math.round((b.maxX - b.minX) / CELL), Math.round((b.maxY - b.minY) / CELL));
  const grid = rasterizeCobstacle(new DiscChecker(scene, r), chart);
  const free = grid.components(false, chart.periodic);
  const cells = shortestCellPath(grid, chart.cellOf(DOORWAY_START.x, DOORWAY_START.y), chart.cellOf(DOORWAY_GOAL.x, DOORWAY_GOAL.y));
  const path = cells ? cells.map(([i, j]) => chart.point(i, j)) : null;
  const inflated = r > 0 ? [...DOORWAY_ROOM.obstacles.map((o) => inflateDisc(o.poly, r, 24)), ...DOORWAY_ROOM.walls.map((w) => inflateSegment(w, r, 16))] : [];
  return {
    r,
    grid,
    chart,
    freeComps: free.count,
    freeArea: (grid.n1 * grid.n2 - grid.count()) * CELL * CELL,
    path,
    inflated,
    crateArea: inflatedArea(DOORWAY_ROOM.obstacles[0].poly, r),
  };
}

export function DiscInflation() {
  const [r3, setR3] = useState(0.35);
  const columns = useMemo(() => [0, 0.11, r3].map(buildColumn), [r3]);

  // One clock: a phase along each column's path.
  const sim = useSimulation<{ phase: number }>({ init: () => ({ phase: 0 }), step: (s) => ({ phase: (s.phase + 0.006) % 1 }), fps: 30, initialSeed: 35 });
  const phase = sim.state.phase;

  const makeDraw = useCallback(
    (col: Column, img: HTMLCanvasElement | null) => (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      const pos = col.path && col.path.length > 1 ? col.path[Math.min(col.path.length - 1, Math.floor(phase * col.path.length))] : DOORWAY_START;

      // ---- workspace (bottom) ----
      drawSegments(ctx, v, DOORWAY_ROOM.walls, p.wall, 2.5);
      fillPoly(ctx, v, DOORWAY_ROOM.obstacles[0].poly, p.obstacle);
      dot(ctx, sx(v, DOORWAY_START.x), sy(v, DOORWAY_START.y), 5, p.start);
      dot(ctx, sx(v, DOORWAY_GOAL.x), sy(v, DOORWAY_GOAL.y), 5, p.goal);
      if (col.path) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 1.5;
        ctx.globalAlpha = 0.6;
        ctx.beginPath();
        col.path.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
        ctx.stroke();
        ctx.restore();
      }
      ctx.save();
      ctx.fillStyle = p.robot;
      ctx.globalAlpha = 0.9;
      ctx.beginPath();
      ctx.arc(sx(v, pos.x), sy(v, pos.y), Math.max(sl(v, col.r), 3.5), 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      label(ctx, 'workspace', sx(v, 0.08), sy(v, 4.8), p.ink, { size: 9, weight: 600 });

      // ---- configuration space (top), shifted up by GAP ----
      const shift: [number, number] = [0, GAP];
      if (img) drawRaster(ctx, v, img, col.chart, shift);
      for (const poly of col.inflated) hatchFillPoly(ctx, v, poly.map((q) => ({ x: q.x, y: q.y + GAP })), p.cobstacle, { alpha: 0.08, outline: true });
      drawSegments(ctx, v, DOORWAY_ROOM.walls.map((w) => ({ x1: w.x1, y1: w.y1 + GAP, x2: w.x2, y2: w.y2 + GAP })), p.wall, 1);
      dot(ctx, sx(v, DOORWAY_START.x), sy(v, DOORWAY_START.y + GAP), 5, p.start);
      dot(ctx, sx(v, DOORWAY_GOAL.x), sy(v, DOORWAY_GOAL.y + GAP), 5, p.goal);
      if (col.path) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 2;
        ctx.beginPath();
        col.path.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y + GAP)) : ctx.lineTo(sx(v, q.x), sy(v, q.y + GAP))));
        ctx.stroke();
        ctx.restore();
      }
      dot(ctx, sx(v, pos.x), sy(v, pos.y + GAP), 4.5, p.robot, p.bg);
      label(ctx, 'configuration space: a point among grown obstacles', sx(v, 0.08), sy(v, 4.8 + GAP), p.ink, { size: 9, weight: 600 });
      if (!col.path) label(ctx, 'Q_free has two components: no path', sx(v, 2), sy(v, 2.6 + GAP), p.goal, { size: 10, weight: 700, align: 'center' });
    },
    [phase],
  );

  return (
    <WidgetFrame
      id="w4.4"
      title="Disc Inflation"
      teaches="For a disc, growing the obstacles by r and planning for a point is exactly right — and right only for discs."
      colorKey={['start', 'goal', 'obstacle', 'cobstacle', 'path', 'robot']}
      wide
      caption={
        <>
          Three discs in one room, Choset's figure 3.5: a point, Rusty (r = 0.11 m), and a large disc
          you set with the slider. Each column shows the workspace below and the configuration space
          above, where the crate's edges are offset by r with rounded corners, the walls become
          capsules, and the doorway narrows by 2r. The orange point follows the shortest cell path of
          the raster through the inflated map; the same path, drawn in the workspace, keeps the disc
          clear of everything. Raise the third radius past 0.30 m and the doorway's two capsules
          touch: the free space falls into two components and the query has no answer. The free area
          shrinks by exactly <em>r·p + πr²</em> for the convex crate — the readout checks it.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border sm:grid-cols-3">
        {columns.map((col, idx) => (
          <ColumnView key={idx} col={col} idx={idx} draw={makeDraw} tick={sim.tick} />
        ))}
      </div>
      <ControlPanel columns={2} title="Headline: the third disc's radius">
        <Slider label="r₃" role="robot" value={r3} min={0.12} max={0.5} step={0.01} unit="m" onChange={setR3} help="The doorway is 0.6 m wide: a disc of diameter above 0.6 m cannot pass." />
        <div className="font-ui text-[0.72rem] text-fd-muted-foreground">
          crate |WO| = {fmt(Math.abs(polygonArea(DOORWAY_ROOM.obstacles[0].poly)), 2)} m², perimeter 2.6 m; grown by r₃: analytic |WO ⊕ B<sub>r</sub>| = {fmt(columns[2].crateArea, 3)} m² = |WO| + r·p + πr²
        </div>
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} />
    </WidgetFrame>
  );
}

function ColumnView({ col, idx, draw, tick }: { col: Column; idx: number; draw: (c: Column, img: HTMLCanvasElement | null) => (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => void; tick: number }) {
  // The raster image depends on the palette, which only the draw call knows; build it lazily per palette amber.
  const [img, setImg] = useState<{ color: string; key: number; img: HTMLCanvasElement | null } | null>(null);
  const key = col.r;
  const drawCol = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      let im = img;
      if (!im || im.color !== p.cobstacle || im.key !== key) {
        im = { color: p.cobstacle, key, img: rasterImage(col.grid, p.cobstacle, 4) };
        // Setting state inside draw is safe here: it schedules one more render with the cached image.
        setImg(im);
      }
      draw(col, im.img)(ctx, v, p);
    },
    [img, key, col, draw],
  );
  const names = ['a point robot', 'Rusty, r = 0.11 m', `r = ${col.r.toFixed(2)} m`];
  return (
    <div className="bg-fd-card">
      <div className="flex items-baseline justify-between px-2 py-1">
        <span className="eyebrow">{names[idx]}</span>
        <span className="font-mono text-[0.68rem] tabular-nums" style={col.path ? undefined : { color: 'var(--rm-goal)' }}>
          {col.path ? `path ${(col.path.length * CELL).toFixed(2)} m` : 'disconnected'}
        </span>
      </div>
      <SimCanvas world={WORLD} draw={drawCol} deps={[tick, col, img]} aspect={4 / 10.6} padding={0.15} ariaLabel={`Workspace and configuration space for a disc of radius ${col.r.toFixed(2)} metres in a room with a doorway`} />
      <div className="px-2 py-1.5 font-mono text-[0.68rem] text-fd-muted-foreground tabular-nums">
        free area {col.freeArea.toFixed(2)} m² · Q_free components {col.freeComps}
      </div>
    </div>
  );
}
