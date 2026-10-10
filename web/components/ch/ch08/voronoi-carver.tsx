'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT, type Bounds, type Point2, type Segment } from '@/lib/sim/world';
import { cellCenter, gridFromWorld, type Connectivity, type OccupancyGrid } from '@/lib/search/grid';
import { brushfire, rasterizeObstacles } from '@/lib/potential';
import { exactGvd, type Site } from '@/lib/roadmap/gvd';
import { featureTransform, ridgeMaskFromFeatures } from '@/lib/roadmap/grid';
import { ROOM_B_BOUNDS, SLAB_BOUNDS, slabObstacles, slabSites } from '@/lib/roadmap/examples';
import { fieldImage, drawField, marker } from '@/components/ch/ch07/shared';

/**
 * w8.2 — the Voronoi Carver.
 *
 * Brushfire fronts (Chapter 7) grow from the obstacles one label per tick; the
 * pixels where fronts meet light up as the grid GVD; the exact diagram —
 * traced with Chapter 3's predictor–corrector on d_i − d_j — overlays it. The
 * headline control is the raster resolution: at 1 m the ridge is a few jagged
 * cells, at 5 cm it hugs the parabolas. The slab preset is the design doc's
 * micro-example (meet point (2, 1.5), clearance 2.5); room B of the Apartment
 * has a kitchen counter whose GVD has a parabola at the counter's end.
 */

type Preset = 'slab' | 'room';
const RESOLUTIONS = [1, 0.5, 0.25, 0.1, 0.05];

interface State {
  level: number;
}

export function VoronoiCarver() {
  const [preset, setPreset] = useState<Preset>('slab');
  const [resIndex, setResIndex] = useState(2);
  const [conn, setConn] = useState<Connectivity>(8);
  const [showFronts, setShowFronts] = useState(true);
  const [showRidge, setShowRidge] = useState(true);
  const [showExact, setShowExact] = useState(true);
  const [pegs, setPegs] = useState<[Point2, Point2]>([{ x: 0, y: 0 }, { x: 4, y: 0 }]);
  const [wallY, setWallY] = useState(4);
  const dragging = useRef<0 | 1 | 'wall' | null>(null);
  const cell = RESOLUTIONS[resIndex];

  const bounds: Bounds = preset === 'slab' ? SLAB_BOUNDS : ROOM_B_BOUNDS;

  // Sites for the exact diagram and the raster for the fronts.
  const sites = useMemo<Site[]>(() => {
    if (preset === 'slab') return slabSites(pegs[0], pegs[1], wallY);
    // Room B: its four walls (with the door gap) and the counter, clipped to the room.
    const segs: Segment[] = [];
    for (const w of APARTMENT.walls) {
      const x1 = Math.max(ROOM_B_BOUNDS.minX, Math.min(w.x1, w.x2));
      const x2 = Math.min(ROOM_B_BOUNDS.maxX, Math.max(w.x1, w.x2));
      const y1 = Math.max(ROOM_B_BOUNDS.minY, Math.min(w.y1, w.y2));
      const y2 = Math.min(ROOM_B_BOUNDS.maxY, Math.max(w.y1, w.y2));
      if (x1 > x2 || y1 > y2) continue;
      if (x1 === x2 && y1 === y2) continue;
      segs.push({ x1, y1, x2, y2 });
    }
    return segs.map((s) => ({ kind: 'segment' as const, s }));
  }, [preset, pegs, wallY]);

  const grid = useMemo<OccupancyGrid>(() => {
    if (preset === 'slab') return rasterizeObstacles(slabObstacles(sites), bounds, cell, 0.51 * cell + 1e-9);
    return gridFromWorld(APARTMENT, cell, 0.51 * cell + 1e-9, ROOM_B_BOUNDS);
  }, [preset, sites, bounds, cell]);

  const dist = useMemo(() => brushfire(grid, conn), [grid, conn]);
  // Ridge: Chapter 7's component rule when the obstacles are separate components (the slab),
  // the feature-jump rule when the walls are one connected obstacle (the room).
  const ridge = useMemo(() => {
    if (dist.nSources >= 2) return dist.ridge;
    return ridgeMaskFromFeatures(grid, featureTransform(grid));
  }, [dist, grid]);
  const exact = useMemo(() => exactGvd(sites, bounds, { step: Math.min(0.02, cell / 2), seedsPerAxis: 22 }), [sites, bounds, cell]);

  const init = useCallback((): State => ({ level: 1 }), []);
  const step = useCallback((s: State): State => ({ level: s.level >= dist.maxLabel + 8 ? 1 : s.level + 1 }), [dist]);
  const sim = useSimulation<State>({ init, step, fps: cell >= 0.5 ? 2 : cell >= 0.25 ? 5 : 12, initialSeed: 82 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [dist]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (preset !== 'slab') return;
      if (phase === 'up') {
        dragging.current = null;
        return;
      }
      if (phase === 'down') {
        const d0 = Math.hypot(w[0] - pegs[0].x, w[1] - pegs[0].y);
        const d1 = Math.hypot(w[0] - pegs[1].x, w[1] - pegs[1].y);
        if (Math.min(d0, d1) < 0.4) dragging.current = d0 < d1 ? 0 : 1;
        else if (Math.abs(w[1] - wallY) < 0.35) dragging.current = 'wall';
        return;
      }
      if (dragging.current === 'wall') setWallY(Math.max(1.5, Math.min(6.5, w[1])));
      else if (dragging.current !== null) {
        const next: [Point2, Point2] = [pegs[0], pegs[1]];
        next[dragging.current] = { x: Math.max(-1.8, Math.min(5.8, w[0])), y: Math.max(-0.8, Math.min(wallY - 0.6, w[1])) };
        setPegs(next);
      }
    },
    [preset, pegs, wallY],
  );

  const imgCache = useRef<{ key: string; img: HTMLCanvasElement | null } | null>(null);
  const level = sim.state.level;
  const world = useMemo(() => ({ ...bounds }), [bounds]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const g = grid;
      // Fronts: labels ≤ level in graph tints (baked per level, since the image depends on it).
      if (showFronts) {
        const key = `${p.graph}:${level}:${cell}:${conn}:${preset}:${pegs[0].x},${pegs[0].y},${pegs[1].x},${pegs[1].y},${wallY}`;
        if (!imgCache.current || imgCache.current.key !== key) {
          imgCache.current = { key, img: fieldImage(dist.label, g.nx, g.ny, p.graph, (l) => (l <= 1 || l > level ? 0 : 0.1 + 0.45 * (1 - (l - 2) / Math.max(1, dist.maxLabel - 2)))) };
        }
        if (imgCache.current.img) drawField(ctx, v, imgCache.current.img, bounds, false);
      }
      const cw = Math.ceil(sl(v, cell)) + 1;
      // Obstacles and ridge cells.
      ctx.save();
      for (let k = 0; k < g.occupied.length; k++) {
        const { x, y } = cellCenter(g, k);
        if (g.occupied[k]) {
          ctx.fillStyle = p.obstacle;
          ctx.globalAlpha = 0.85;
          ctx.fillRect(sx(v, x - cell / 2), sy(v, y + cell / 2), cw, cw);
        } else if (showRidge && ridge[k] && dist.label[k] <= level) {
          ctx.fillStyle = p.graph;
          ctx.globalAlpha = 0.95;
          ctx.fillRect(sx(v, x - cell / 2), sy(v, y + cell / 2), cw, cw);
        }
      }
      ctx.restore();
      // The exact diagram and its meet points.
      if (showExact) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 2.2;
        for (const e of exact.edges) {
          ctx.beginPath();
          e.points.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
          ctx.stroke();
        }
        ctx.restore();
        for (const m of exact.meetPoints) {
          marker(ctx, sx(v, m.q.x), sy(v, m.q.y), 6, p.path, p.bg);
          label(ctx, `(${m.q.x.toFixed(2)}, ${m.q.y.toFixed(2)})  D = ${m.clearance.toFixed(2)}`, sx(v, m.q.x) + 9, sy(v, m.q.y) - 9, p.path, { size: 10, weight: 600 });
        }
        for (const b of exact.boundaryPoints) marker(ctx, sx(v, b.x), sy(v, b.y), 3, p.bg, p.path);
      }
      // Sites.
      if (preset === 'slab') {
        for (const pg of pegs) marker(ctx, sx(v, pg.x), sy(v, pg.y), 6, p.obstacle, p.bg);
        ctx.save();
        ctx.strokeStyle = p.obstacle;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(sx(v, bounds.minX), sy(v, wallY));
        ctx.lineTo(sx(v, bounds.maxX), sy(v, wallY));
        ctx.stroke();
        ctx.restore();
        label(ctx, 'drag the pegs or the wall', 10, v.height - 12, p.ink, { size: 9 });
      }
      const shown = Math.min(level, dist.maxLabel);
      const ridgeCount = Array.from(ridge).reduce((s, b, k) => s + (b && !g.occupied[k] ? 1 : 0), 0);
      label(ctx, `${cell} m cells · ${conn}-point · front label ${shown} / ${dist.maxLabel} · ${ridgeCount} ridge cells · exact: ${exact.edges.length} edges, ${exact.meetPoints.length} meet point(s)`, 10, 14, p.ink, { size: 10.5, weight: 600 });
    },
    [grid, dist, ridge, exact, level, cell, conn, preset, pegs, wallY, bounds, showFronts, showRidge, showExact],
  );

  return (
    <WidgetFrame
      id="w8.2"
      title="Voronoi Carver"
      teaches="Where brushfire fronts meet is the GVD — and between a vertex and an edge the exact diagram is a parabola, not a line. Grid ridges are jagged and converge as the cells shrink."
      colorKey={['obstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Brushfire fronts (blue tints) grow from the slate obstacles one label per tick and the
          pixels where fronts from different obstacles collide are drawn solid: the grid GVD. The
          purple curves are the exact diagram, traced as the zero set of d<sub>i</sub> − d<sub>j</sub>
          with Chapter 3&rsquo;s predictor–corrector; the ringed dot is the meet point, equidistant to
          three obstacles. On the slab that is (2, 1.5) with clearance 2.5: the vertical bisector of
          the pegs below it and two parabolas — focus a peg, directrix the wall — above. Coarsen the
          raster to 1 m and the ridge becomes a handful of cells whose meet point sits a cell and a
          half from the exact one (the chessboard metric meets at (2, 2), the Manhattan one at
          (2, 1)); refine to 5 cm and it hugs the curves. Drag a peg or the wall and the parabola
          follows its focus and directrix.
        </>
      }
    >
      <SimCanvas world={world} draw={draw} deps={[sim.tick, level, dist, ridge, exact, showFronts, showRidge, showExact, preset]} aspect={preset === 'slab' ? 8 / 5 : 4 / 3.8} padding={0.25} onPointer={onPointer} cursor={preset === 'slab' ? 'grab' : 'default'} ariaLabel="A rectangular slab with two peg obstacles and a wall; blue brushfire fronts grow from them, solid blue ridge cells mark where the fronts meet, and purple exact Voronoi curves with a marked meet point overlay the grid." />
      <ControlPanel columns={3} title="Headline: raster resolution">
        <Slider label="cell size" role="graph" value={resIndex} min={0} max={RESOLUTIONS.length - 1} step={1} format={(i) => `${RESOLUTIONS[Math.round(i)]} m`} onChange={(i) => setResIndex(Math.round(i))} />
        <ButtonRow>
          <ActionButton emphasis={preset === 'slab'} onClick={() => setPreset('slab')}>slab: two pegs and a wall</ActionButton>
          <ActionButton emphasis={preset === 'room'} onClick={() => setPreset('room')}>Apartment room B</ActionButton>
        </ButtonRow>
        <ButtonRow>
          <ActionButton emphasis={conn === 4} onClick={() => setConn(4)}>4-point</ActionButton>
          <ActionButton emphasis={conn === 8} onClick={() => setConn(8)}>8-point</ActionButton>
        </ButtonRow>
      </ControlPanel>
      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Layers · wall height · seed re-roll of the pegs</summary>
        <ControlPanel columns={3}>
          <div className="flex flex-col gap-1.5">
            <Toggle label="brushfire fronts" role="graph" checked={showFronts} onChange={setShowFronts} />
            <Toggle label="grid ridge" role="graph" checked={showRidge} onChange={setShowRidge} />
            <Toggle label="exact diagram" role="path" checked={showExact} onChange={setShowExact} />
          </div>
          <Slider label="wall height y" role="obstacle" value={wallY} min={1.5} max={6.5} step={0.1} unit="m" onChange={setWallY} />
          <ButtonRow>
            <ActionButton
              onClick={() => {
                const rng = new Rng(Math.floor(Math.random() * 1e5));
                setPegs([{ x: rng.uniform(-1.2, 2), y: rng.uniform(-0.6, 1.2) }, { x: rng.uniform(2.2, 5.4), y: rng.uniform(-0.6, 1.2) }]);
              }}
            >
              re-roll the pegs
            </ActionButton>
            <ActionButton onClick={() => { setPegs([{ x: 0, y: 0 }, { x: 4, y: 0 }]); setWallY(4); }}>design-doc slab</ActionButton>
          </ButtonRow>
        </ControlPanel>
      </details>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={Math.min(level, dist.maxLabel) - 1} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
