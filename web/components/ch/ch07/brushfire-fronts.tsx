'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT } from '@/lib/sim/world';
import { cellAt, cellCenter, cellIJ, cloneGrid, gridFromWorld, type Connectivity, type OccupancyGrid } from '@/lib/search/grid';
import { edt2dSquared } from '@/lib/mapping/edt';
import { MICRO_7X3, brushfire, ridgeCells } from '@/lib/potential/brushfire';
import { fmt } from './shared';

/**
 * w7.4 — Brushfire Fronts.
 *
 * The §3 micro-grid (7 × 3, obstacles at (1,2) and (7,2)) and then a room of
 * the Apartment: fronts grow from every obstacle pixel one label per tick, in
 * the graph colour's tints (an explored set). Pixels two fronts reach from
 * *different* obstacles light up as the ridge — Chapter 8's grid GVD. The
 * headline control is 4- versus 8-point connectivity: the same BFS grows
 * diamonds under one and squares under the other, which is the point —
 * brushfire computes a grid metric, not Euclidean distance. The exact EDT is
 * shown at the hovered cell for comparison.
 */

type Preset = 'micro' | 'room';

interface State {
  level: number;
}

const ROOM_BOUNDS = { minX: 4, minY: 0, maxX: 8, maxY: 3.8 };

export function BrushfireFronts() {
  const [preset, setPreset] = useState<Preset>('micro');
  const [conn, setConn] = useState<Connectivity>(4);
  const [paint, setPaint] = useState(false);
  const [hover, setHover] = useState<number | null>(null);
  const [edits, setEdits] = useState<OccupancyGrid | null>(null);
  const pressed = useRef(false);

  const base = useMemo<OccupancyGrid>(() => (preset === 'micro' ? MICRO_7X3 : gridFromWorld(APARTMENT, 0.2, 0.1, ROOM_BOUNDS)), [preset]);
  const grid = edits ?? base;
  const dist = useMemo(() => brushfire(grid, conn), [grid, conn]);
  const ridge = useMemo(() => new Set(ridgeCells(dist)), [dist]);
  // The exact Euclidean distance transform (sister book's edt), in cells, for the hover readout.
  const edt = useMemo(() => {
    const seed = new Float64Array(grid.nx * grid.ny).fill(1e20);
    for (let k = 0; k < seed.length; k++) if (grid.occupied[k]) seed[k] = 0;
    return edt2dSquared(seed, grid.nx, grid.ny);
  }, [grid]);

  const world = useMemo(() => ({ minX: grid.origin.x, minY: grid.origin.y, maxX: grid.origin.x + grid.nx * grid.cellSize, maxY: grid.origin.y + grid.ny * grid.cellSize }), [grid]);

  const init = useCallback((): State => ({ level: 1 }), []);
  const step = useCallback((s: State): State => ({ level: s.level >= dist.maxLabel + 6 ? 1 : s.level + 1 }), [dist]);
  const sim = useSimulation<State>({ init, step, fps: preset === 'micro' ? 1.5 : 6, initialSeed: 74 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [dist]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const c = cellAt(grid, w[0], w[1]);
      const inside = w[0] >= world.minX && w[0] <= world.maxX && w[1] >= world.minY && w[1] <= world.maxY;
      if (phase === 'move') setHover(inside ? c : null);
      if (!paint) return;
      if (phase === 'down') {
        pressed.current = true;
        if (inside) {
          const g = cloneGrid(grid);
          g.occupied[c] = g.occupied[c] ? 0 : 1;
          setEdits(g);
        }
      } else if (phase === 'up') pressed.current = false;
    },
    [grid, paint, world],
  );

  const level = sim.state.level;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const cs = grid.cellSize;
      const cw = Math.ceil(sl(v, cs)) + 1;
      const micro = preset === 'micro';
      for (let k = 0; k < grid.occupied.length; k++) {
        const { x, y } = cellCenter(grid, k);
        const px = sx(v, x - cs / 2);
        const py = sy(v, y + cs / 2);
        const l = dist.label[k];
        if (grid.occupied[k]) {
          ctx.fillStyle = p.obstacle;
          ctx.globalAlpha = 1;
          ctx.fillRect(px, py, cw, cw);
          if (micro) label(ctx, '1', sx(v, x), sy(v, y), p.bg, { size: 14, align: 'center', weight: 700 });
          continue;
        }
        const reached = l > 0 && l <= level;
        const onRidge = ridge.has(k) && reached;
        if (reached) {
          ctx.fillStyle = onRidge ? p.graph : p.graph;
          ctx.globalAlpha = onRidge ? 0.95 : 0.1 + 0.45 * (1 - (l - 2) / Math.max(1, dist.maxLabel - 2));
          ctx.fillRect(px, py, cw, cw);
          ctx.globalAlpha = 1;
          if (micro) label(ctx, String(l), sx(v, x), sy(v, y), onRidge ? p.bg : p.ink, { size: 14, align: 'center', weight: onRidge ? 700 : 500 });
          // The newest front, outlined.
          if (l === level) {
            ctx.strokeStyle = p.graph;
            ctx.lineWidth = 1.5;
            ctx.strokeRect(px + 0.75, py + 0.75, cw - 1.5, cw - 1.5);
          }
          // Back pointers on the micro grid: an arrow toward the labelling pixel.
          if (micro && dist.back[k] >= 0) {
            for (const b of [dist.back[k], dist.back2[k]]) {
              if (b < 0) continue;
              const bc = cellCenter(grid, b);
              const dx = bc.x - x;
              const dy = bc.y - y;
              const n = Math.hypot(dx, dy) || 1;
              ctx.save();
              ctx.strokeStyle = onRidge ? p.bg : p.ink;
              ctx.globalAlpha = 0.7;
              ctx.lineWidth = 1.2;
              ctx.beginPath();
              ctx.moveTo(sx(v, x + (dx / n) * cs * 0.22), sy(v, y + (dy / n) * cs * 0.22));
              ctx.lineTo(sx(v, x + (dx / n) * cs * 0.42), sy(v, y + (dy / n) * cs * 0.42));
              ctx.stroke();
              ctx.restore();
            }
          }
        } else if (l === 0) {
          ctx.fillStyle = p.unknown;
          ctx.globalAlpha = 0.25;
          ctx.fillRect(px, py, cw, cw);
          ctx.globalAlpha = 1;
        }
      }
      // Grid lines.
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let i = 0; i <= grid.nx; i++) {
        const x = sx(v, world.minX + i * cs);
        ctx.moveTo(x, sy(v, world.minY));
        ctx.lineTo(x, sy(v, world.maxY));
      }
      for (let j = 0; j <= grid.ny; j++) {
        const y = sy(v, world.minY + j * cs);
        ctx.moveTo(sx(v, world.minX), y);
        ctx.lineTo(sx(v, world.maxX), y);
      }
      ctx.stroke();
      ctx.restore();
      if (hover !== null) {
        const hc = cellCenter(grid, hover);
        ctx.strokeStyle = p.accent;
        ctx.lineWidth = 2;
        ctx.strokeRect(sx(v, hc.x - cs / 2), sy(v, hc.y + cs / 2), sl(v, cs), sl(v, cs));
      }
      const shown = Math.min(level, dist.maxLabel);
      label(ctx, `tick ${shown - 1}: front label ${shown} of ${dist.maxLabel}  ·  ${conn}-point  ·  ${ridge.size} ridge pixels`, 10, 14, p.ink, { size: 10.5, weight: 600 });
      if (micro) label(ctx, 'cells (col,row); column 4 is reached by both fronts — two back pointers', 10, v.height - 12, p.ink, { size: 9 });
    },
    [grid, dist, ridge, level, preset, conn, hover, world],
  );

  const hoverInfo = (() => {
    if (hover === null || grid.occupied[hover]) return null;
    const l = dist.label[hover];
    const [i, j] = cellIJ(grid, hover);
    return { col: i + 1, row: j + 1, label: l, gridDist: l > 0 ? l - 1 : NaN, euclid: Math.sqrt(edt[hover]), ridge: ridge.has(hover), two: dist.back2[hover] >= 0 };
  })();

  return (
    <WidgetFrame
      id="w7.4"
      title="Brushfire Fronts"
      teaches="Brushfire is breadth-first search from every obstacle pixel; it computes a grid metric, not Euclidean distance; where two fronts meet is the grid GVD."
      colorKey={['obstacle', 'graph']}
      caption={
        <>
          Obstacle pixels carry Choset&rsquo;s label 1; each tick the front advances one label, and a
          pixel&rsquo;s label is 1 + its link length to the nearest obstacle (Manhattan under 4-point,
          chessboard under 8-point). Pixels reached by fronts from <em>different</em> obstacles are
          drawn solid: on the micro-grid that is column 4, with two back pointers each; in the
          Apartment room it is the skeleton between the walls and the kitchen counter — Chapter
          8&rsquo;s grid GVD. Switch connectivity and watch diamonds become squares; hover a cell to
          compare its grid distance with the exact Euclidean distance transform. Enable painting and
          click to add or remove obstacle pixels.
        </>
      }
    >
      <SimCanvas world={world} draw={draw} deps={[sim.tick, level, dist, hover, preset, conn]} aspect={preset === 'micro' ? 7 / 3.4 : 4 / 3.8} padding={preset === 'micro' ? 0.3 : 0.15} onPointer={onPointer} cursor={paint ? 'crosshair' : 'default'} ariaLabel="A grid whose obstacle cells are slate and whose free cells fill with blue tints as brushfire fronts expand outward one label per tick; cells reached by two fronts are solid blue." />
      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border font-mono text-[0.7rem] tabular-nums sm:grid-cols-4">
        <Readout title="hovered cell" value={hoverInfo ? `(${hoverInfo.col}, ${hoverInfo.row})` : '—'} sub={hoverInfo?.ridge ? (hoverInfo.two ? 'ridge: two back pointers' : 'ridge: fronts met on an edge') : 'move the pointer over a cell'} />
        <Readout title="brushfire label" value={hoverInfo && hoverInfo.label > 0 ? String(hoverInfo.label) : '—'} sub={hoverInfo && hoverInfo.label > 0 ? `grid distance ${hoverInfo.gridDist} cells` : ''} />
        <Readout title="Euclidean distance (exact EDT)" value={hoverInfo ? `${fmt(hoverInfo.euclid, 3)} cells` : '—'} sub={hoverInfo && hoverInfo.label > 0 ? `grid − EDT = ${fmt(hoverInfo.gridDist - hoverInfo.euclid, 3)}` : ''} />
        <Readout title="metric" value={conn === 4 ? 'Manhattan' : 'chessboard'} sub={conn === 4 ? 'EDT ≤ grid distance' : 'grid distance ≤ EDT'} />
      </div>
      <ControlPanel columns={3} title="Headline: connectivity">
        <ButtonRow>
          <ActionButton emphasis={conn === 4} onClick={() => setConn(4)}>4-point (diamonds)</ActionButton>
          <ActionButton emphasis={conn === 8} onClick={() => setConn(8)}>8-point (squares)</ActionButton>
        </ButtonRow>
        <ButtonRow>
          <ActionButton emphasis={preset === 'micro'} onClick={() => { setPreset('micro'); setEdits(null); }}>7 × 3 micro-grid</ActionButton>
          <ActionButton emphasis={preset === 'room'} onClick={() => { setPreset('room'); setEdits(null); }}>Apartment room B</ActionButton>
        </ButtonRow>
        <div className="flex flex-col gap-1.5">
          <Toggle label="Paint obstacles (click cells)" role="obstacle" checked={paint} onChange={setPaint} />
          {edits ? <ActionButton onClick={() => setEdits(null)}>clear edits</ActionButton> : null}
        </div>
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={Math.min(level, dist.maxLabel) - 1} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}

function Readout({ title, value, sub }: { title: string; value: string; sub: string }) {
  return (
    <div className="px-3 py-2">
      <div className="eyebrow">{title}</div>
      <div className="mt-0.5 text-fd-foreground">{value}</div>
      <div className="mt-0.5 font-ui text-[0.68rem] text-fd-muted-foreground">{sub}</div>
    </div>
  );
}
