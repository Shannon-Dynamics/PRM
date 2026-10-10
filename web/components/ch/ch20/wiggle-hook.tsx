'use client';

import { useCallback, useMemo } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Hitch } from '@/lib/robots/hitch';
import { fourFlowLoop, unicycle } from '@/lib/nonholonomic';
import type { Vec } from '@/lib/prob/linalg';
import { drawArrow, drawHitch, drawPolyline } from './draw-car';

/**
 * The chapter hook. Hitch is told "0.5 m to the left, same heading". A
 * holonomic ghost slides there in one second. Hitch, which cannot move
 * sideways, runs the four-flow loop spin → drive → unspin → undrive over and
 * over: each loop buys ε² of sideways motion (the Lie bracket of drive and
 * spin), so ε = 0.19 rad needs 14 loops for half a metre — and leaks ε³/2
 * forward per loop, the O(ε³) remainder the chapter measures.
 */

const WORLD = { minX: -3.2, minY: -1.6, maxX: 3.6, maxY: 2.2 };
const EPS = 0.19;
const TARGET = 0.5;
const TICKS_PER_LOOP = 24;

interface Sim {
  t: number;
}

interface Plan {
  loops: number;
  trace: Vec[];
  corners: Vec[];
  perLoop: Vec;
}

function buildPlan(): Plan {
  const u = unicycle();
  const loops = Math.ceil(TARGET / (EPS * Math.sin(EPS)));
  const trace: Vec[] = [[0, 0, 0]];
  const corners: Vec[] = [[0, 0, 0]];
  let x: Vec = [0, 0, 0];
  let perLoop: Vec = [0, 0, 0];
  for (let k = 0; k < loops; k++) {
    // spin first, then drive: the loop g₂ → g₁ → −g₂ → −g₁ moves by ε²[g₂, g₁] = −ε²[g₁, g₂] — to the LEFT.
    const loop = fourFlowLoop(u.spin, u.drive, x, EPS, TICKS_PER_LOOP / 4);
    trace.push(...loop.trace.slice(1));
    corners.push(...loop.corners.slice(1));
    x = loop.corners[4];
    perLoop = loop.residue;
  }
  return { loops, trace, corners, perLoop };
}

export function WiggleHook() {
  const plan = useMemo(buildPlan, []);
  const car = useMemo(() => Hitch.car(), []);
  const total = plan.trace.length + 40;
  const init = useCallback((): Sim => ({ t: 0 }), []);
  const step = useCallback((s: Sim): Sim => ({ t: s.t + 1 }), []);
  const sim = useSimulation<Sim>({ init, step, fps: 30, maxTicks: total, loop: true });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 0.5);
      const t = Math.min(sim.state.t, plan.trace.length - 1);
      const frac = Math.min(1, sim.state.t / 30);
      // Start and goal poses.
      drawHitch(ctx, v, car, car.cfg(0, 0, 0), p.start, { alpha: 0.6 });
      drawHitch(ctx, v, car, car.cfg(0, TARGET, 0), p.goal, { alpha: 0.7 });
      // Holonomic ghost: slides sideways in one second.
      drawHitch(ctx, v, car, car.cfg(0, TARGET * frac, 0), p.path, { alpha: 0.5, lineWidth: 1.2 });
      label(ctx, frac < 1 ? 'holonomic ghost: sliding…' : 'holonomic ghost: parked at t = 1 s', sx(v, -3.0), sy(v, 2.0), p.path, { size: 10 });
      // Hitch's wiggle trace so far.
      const done = plan.trace.slice(0, t + 1);
      drawPolyline(ctx, v, done.map((q) => ({ x: q[0], y: q[1] })), p.robot, { lineWidth: 1.2, alpha: 0.8 });
      const q = plan.trace[t];
      drawHitch(ctx, v, car, car.cfg(q[0], q[1], q[2]), p.robot, { fill: true });
      // Net sideways displacement so far and the bracket arrow.
      const loopsDone = Math.floor(t / TICKS_PER_LOOP);
      drawArrow(ctx, v, 0, 0, q[0], q[1], p.graph, { lineWidth: 2 });
      label(ctx, `loop ${Math.min(loopsDone, plan.loops)} of ${plan.loops}  ·  ε = ${EPS} rad  ·  Δy per loop = ε sin ε = ${(EPS * Math.sin(EPS)).toFixed(4)} ≈ ε²`, sx(v, -3.0), sy(v, -1.2), p.ink, { size: 10 });
      label(ctx, `net: Δy = ${q[1].toFixed(3)} m (left),  Δx = ${q[0].toFixed(3)} m (the O(ε³) leak, ε(1 − cos ε) = ${(EPS * (1 - Math.cos(EPS))).toFixed(4)} per loop)`, sx(v, -3.0), sy(v, -1.45), p.graph, { size: 10 });
    },
    [sim.state.t, plan, car],
  );

  return (
    <div className="not-prose overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick]} aspect={1.9} padding={0.2} ariaLabel="Hitch wiggling half a metre sideways by repeated spin–drive–unspin–undrive loops while a holonomic ghost slides there directly." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} />
    </div>
  );
}
