'use client';

import { useCallback } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawRobot, drawWorld, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT } from '@/lib/sim/world';
import { GvgExplorer } from '@/lib/roadmap/explore';
import { ScanQuery } from '@/lib/roadmap/scan-query';
import { drawExplorerGraph, polyline } from './shared';

/**
 * The chapter's hook: Rusty in an unknown Apartment with only its range
 * ring, and twenty seconds later the whole floor's graph exists, drawn by
 * the robot driving along it. A fast, control-free replay of w9.1.
 */

interface State {
  ex: GvgExplorer;
  sensor: ScanQuery;
}

export function UnknownApartmentHook() {
  const init = useCallback((): State => {
    const sensor = new ScanQuery(APARTMENT, { nRays: 360, maxRange: 6 });
    const ex = new GvgExplorer(sensor.query, [1.0, 1.2], { alpha: 1, beta: -3, dt: 0.03, meetTol: 0.08, boundaryTol: 0.12, nodeTol: 0.015, homeTol: 1e-4, maxTicks: 200000 });
    return { ex, sensor };
  }, []);
  const step = useCallback((s: State): State => {
    for (let k = 0; k < 10 && s.ex.phase !== 'done' && s.ex.phase !== 'stuck'; k++) s.ex.tick();
    return { ...s };
  }, []);
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 9, maxTicks: 720, loop: true });
  const { ex, sensor } = sim.state;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
      polyline(ctx, v, ex.trail, p.robot, 1, 0.3);
      drawExplorerGraph(ctx, v, p, ex, { labels: false });
      sensor.lastNearest.slice(0, 2).forEach((n) => {
        if (n.closest) polyline(ctx, v, [ex.q, n.closest], p.path, 1.8, 0.9);
      });
      drawRobot(ctx, v, { x: ex.q[0], y: ex.q[1], theta: Math.atan2(ex.heading[1], ex.heading[0]) }, p.robot, 0.3);
      const t = (ex.ticks * 0.03).toFixed(1);
      label(ctx, `t = ${t} s · ${ex.meetPoints.length} meet points · ${ex.edges.filter((e) => e.to >= 0).length} edges · ${ex.distanceDriven.toFixed(0)} m driven${ex.phase === 'done' ? ' · complete' : ''}`, 10, 14, p.ink, { size: 10.5, weight: 600 });
    },
    [ex, sensor],
  );

  return (
    <div className="not-prose overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas world={APARTMENT.bounds} draw={draw} deps={[sim.tick]} aspect={12 / 9} padding={0.3} ariaLabel="An orange robot drives through an apartment floorplan, drawing a blue graph of curves equidistant to the walls as it goes." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onReset={sim.reset} tick={ex.ticks} />
    </div>
  );
}
