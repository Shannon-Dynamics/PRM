'use client';

import { useCallback, useMemo } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawRobot, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { LIVING_ROOM, LIVING_ROOM_START, RUSTY_RADIUS } from '@/lib/cells/examples';
import { arcLengths, coverageTimeline, pointAtArc } from '@/lib/cells/coverage';
import { onlinePoints, runOnlineCoverage, type OnlineResult } from '@/lib/cells/online';
import { freeRaster } from '@/lib/cells/slices';
import { LAB_RASTER } from '@/lib/cells/lab';
import { worldBounds } from '@/lib/cells/world';
import { blitRaster, drawCellWorld, hatchPixels, marker, polyline, rasterImage } from './shared';

/**
 * The chapter's hook (C Fig. 6.22, in the Apartment's living room): Rusty
 * covers the room twice from the same start with the same 44 cm effector.
 * Left, a conventional raster scan — lap, follow one lap width along whichever
 * boundary the lap ended on, lap back — senses critical points only on the
 * boundary it happens to follow, misses the ones in the ceiling, and declares
 * the room done with the strip behind the sofa and the floor south of the
 * coffee table still bare. Right, the cycle algorithm closes a loop around
 * every gap between laps, senses every critical point, and covers it all.
 */

const SPEED = 0.16;
const HOLD = 90;

interface Run {
  res: OnlineResult;
  pts: { x: number; y: number }[];
  cum: number[];
  timeline: Float32Array;
}

export function MissedSofaHook() {
  const raster = useMemo(() => freeRaster(LIVING_ROOM, LAB_RASTER), []);
  const runs = useMemo(() => {
    const make = (policy: 'raster' | 'cycle'): Run => {
      const res = runOnlineCoverage(LIVING_ROOM, { twoR: 2 * RUSTY_RADIUS, start: LIVING_ROOM_START, policy });
      const pts = onlinePoints(res);
      return { res, pts, cum: arcLengths(pts), timeline: coverageTimeline(raster, pts, RUSTY_RADIUS) };
    };
    return { raster: make('raster'), cycle: make('cycle') };
  }, [raster]);
  const total = Math.ceil(runs.cycle.res.length / SPEED) + HOLD;
  const init = useCallback(() => ({}), []);
  const step = useCallback((s: object) => s, []);
  const sim = useSimulation<object>({ init, step, fps: 30, initialSeed: 22, maxTicks: total, loop: true });
  const s = sim.tick * SPEED;
  const bounds = useMemo(() => worldBounds(LIVING_ROOM), []);

  const makeDraw = useCallback(
    (run: Run, title: string) => (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const L = run.res.length;
      const sNow = Math.min(s, L);
      const done = s >= L;
      blitRaster(ctx, v, raster, rasterImage(raster, p.path, (k) => (run.timeline[k] <= sNow ? 0.32 : 0)));
      if (done) hatchPixels(ctx, v, raster, (k) => raster.free[k] === 1 && !(run.timeline[k] <= L), p.cobstacle);
      drawCellWorld(ctx, v, p, LIVING_ROOM, { names: true });
      const at = pointAtArc(run.pts, run.cum, sNow);
      polyline(ctx, v, [...run.pts.slice(0, at.i + 1), at.p], p.path, 1.2, 0.85);
      for (const n of run.res.nodes) if (n.s <= sNow) marker(ctx, sx(v, n.q.x), sy(v, n.q.y), 4.5, p.graph, p.bg);
      const prev = run.pts[Math.max(0, at.i)];
      const th = Math.atan2(at.p.y - prev.y, at.p.x - prev.x);
      drawRobot(ctx, v, { x: at.p.x, y: at.p.y, theta: Number.isFinite(th) ? th : 0 }, p.robot, RUSTY_RADIUS);
      // Covered fraction so far.
      let free = 0, cov = 0;
      for (let k = 0; k < raster.free.length; k++) {
        if (!raster.free[k]) continue;
        free++;
        if (run.timeline[k] <= sNow) cov++;
      }
      label(ctx, title, 8, 13, p.ink, { size: 10.5, weight: 700 });
      label(ctx, `${sNow.toFixed(1)} m · ${((100 * cov) / free).toFixed(1)} % covered · ${run.res.nodes.filter((n) => n.s <= sNow).length} critical points${done ? ' · done' : ''}`, 8, v.height - 10, p.ink, { size: 9.5 });
    },
    [s, raster],
  );

  const drawRaster = useMemo(() => makeDraw(runs.raster, 'raster scan'), [makeDraw, runs]);
  const drawCycle = useMemo(() => makeDraw(runs.cycle, 'cycle algorithm'), [makeDraw, runs]);

  return (
    <div className="not-prose overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <div className="grid grid-cols-1 gap-px bg-fd-border sm:grid-cols-2">
        <div className="bg-fd-card">
          <SimCanvas world={bounds} draw={drawRaster} deps={[sim.tick]} aspect={1.05} padding={0.15} ariaLabel="Left: a robot mows a living room with a raster scan, leaving the strip behind the sofa and part of the floor uncovered." />
        </div>
        <div className="bg-fd-card">
          <SimCanvas world={bounds} draw={drawCycle} deps={[sim.tick]} aspect={1.05} padding={0.15} ariaLabel="Right: the same robot with the cycle algorithm covers the whole room." />
        </div>
      </div>
      <Transport playing={sim.playing} onToggle={sim.toggle} onReset={sim.reset} tick={sim.tick} />
    </div>
  );
}
