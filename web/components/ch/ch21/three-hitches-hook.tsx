'use client';

import { useCallback, useMemo } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Hitch } from '@/lib/robots/hitch';
import { LOT } from '@/lib/robots/worlds';
import { Scene } from '@/lib/collide/scene';
import { planPark, type ParkMethod } from '@/lib/nonholonomic';
import { drawCarPath, drawHitch, drawLot, drawPoseMarker, insetBox } from './draw-lot';

/**
 * The chapter hook: three Hitches, one bay. Reeds–Shepp (crisp, a cusp),
 * a flat-output quintic (smooth, wide), and a forward-propagated search on the
 * dynamic car (jittery, collision-aware) — all from `planPark`, replayed side
 * by side with a scoreboard. Same destination, three fares.
 */

const WORLD = { minX: 0, minY: 0, maxX: 16, maxY: 10 };
const CAR = Hitch.car();
const SCENE = Scene.fromWorld(LOT);
const START = { x: 5.5, y: 1.2, theta: 0 };
const GOAL = { x: 9.25, y: 6.4, theta: Math.PI / 2 };
const METHODS: { m: ParkMethod; name: string; dash: number[] }[] = [
  { m: 'rs', name: 'Reeds–Shepp', dash: [2, 4] },
  { m: 'flat', name: 'flat-output quintic', dash: [] },
  { m: 'kino', name: 'forward-propagated RRT', dash: [7, 5] },
];

export function ThreeHitchesHook() {
  const init = useCallback(() => ({ k: 0 }), []);
  const step = useCallback((s: { k: number }) => ({ k: s.k + 1 }), []);
  const sim = useSimulation({ init, step, fps: 30, maxTicks: 200, loop: true, initialSeed: 21 });
  const results = useMemo(() => METHODS.map(({ m }) => planPark(m, { car: CAR, scene: SCENE, start: START, goal: GOAL }, { seed: sim.seed })), [sim.seed]);
  const frac = Math.min(1, sim.state.k / 150);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawLot(ctx, v, LOT, p);
      drawPoseMarker(ctx, v, START, p.start, 1.0);
      drawPoseMarker(ctx, v, GOAL, p.goal, 1.0);
      results.forEach((r, i) => {
        if (!r.ok) return;
        drawCarPath(ctx, v, r.samples, p.path, { lineWidth: 2, dash: METHODS[i].dash, alpha: 0.85 });
        const s = r.samples[Math.min(r.samples.length - 1, Math.floor(frac * (r.samples.length - 1)))];
        drawHitch(ctx, v, CAR, { body: s.q, trailer: null }, p.robot, { fill: true, alpha: 0.85 });
      });
      const x0 = v.width - 330;
      insetBox(ctx, x0, 8, 320, 70, p);
      label(ctx, 'method                 length  cusps  plan ms  clearance', x0 + 8, 18, p.ink, { size: 9, weight: 600 });
      results.forEach((r, i) => {
        const row = r.ok
          ? `${METHODS[i].name.padEnd(23)}${r.score.length.toFixed(1).padStart(6)}  ${String(r.score.cusps).padStart(5)}  ${r.ms.toFixed(1).padStart(7)}  ${r.score.collides ? 'collides' : r.score.clearance.toFixed(2).padStart(9)}`
          : `${METHODS[i].name.padEnd(23)}  failed`;
        label(ctx, row, x0 + 8, 34 + i * 14, p.ink, { size: 9 });
      });
    },
    [results, frac],
  );

  return (
    <div className="not-prose my-6 overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, results]} aspect={1.6} padding={0.2} ariaLabel="Three Hitches parking into the same bay of the Lot along a Reeds–Shepp path, a flat-output polynomial and a forward-propagated tree path, with a scoreboard." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} tick={sim.tick} seed={sim.seed} />
    </div>
  );
}
