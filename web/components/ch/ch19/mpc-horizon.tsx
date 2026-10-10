'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawRobot, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Vec } from '@/lib/prob/linalg';
import { APARTMENT } from '@/lib/sim/world';
import { EdtField } from '@/lib/trajopt/field';
import { Unicycle } from '@/lib/trajopt/models';
import { GridCostToGo, Mpc, navigationCost } from '@/lib/trajopt/mpc';

/**
 * w19.3 — the MPC Horizon.
 *
 * Rusty in the Apartment under receding-horizon iLQR: each tick, a horizon-N_h
 * problem from the measured pose with the obstacle *where it is now* (a sensor
 * does not predict), apply u₀, discard the rest, warm-start the next solve by
 * shifting. A scripted disc slides in and out of the corridor through room B's
 * door. The terminal cost is the Chapter 6 cost-to-go (wave-front distance on
 * an occupancy grid); switch it off for a plain quadratic pull and every
 * horizon becomes myopic — Rusty parks against a wall it cannot see around.
 * With it, N_h buys compute, not safety: the plan is corrected every tick. The
 * open-loop ghost executes each plan to its end before replanning, and collides
 * where MPC waits.
 */

const DT = 0.1;
const START: Vec = [1.0, 1.0, 0];
const GOAL: Vec = [11.0, 6.4, 0];
const RUSTY_R = 0.22;

interface Params {
  horizon: number;
  obstacleSpeed: number;
  warm: boolean;
  ghost: boolean;
  costToGo: boolean;
  vMax: number;
}

interface Sim {
  x: Vec;
  t: number;
  plan: Vec[];
  trail: Vec[];
  solveMs: number;
  solveHist: number[];
  ghostX: Vec;
  ghostPlan: Vec[];
  ghostIndex: number;
  ghostTrail: Vec[];
  ghostHit: boolean;
  hit: boolean;
  reached: boolean;
  mpc: Mpc;
  ghostMpc: Mpc;
}

function obstacleAt(t: number, speed: number): { x: number; y: number; r: number } {
  // Slides in and out of the corridor through room B's door at x = 6.
  return { x: 6.0, y: 3.9 + 0.9 * Math.sin(speed * t), r: 0.35 };
}

export function MpcHorizon() {
  const [params, setParams] = useState<Params>({ horizon: 10, obstacleSpeed: 0.6, warm: true, ghost: true, costToGo: true, vMax: 1.0 });
  const f = useMemo(() => new Unicycle(DT), []);
  const walls = useMemo(() => EdtField.fromWorld(APARTMENT, 0.05), []);
  const ctg = useMemo(() => new GridCostToGo(APARTMENT, [GOAL[0], GOAL[1]], 0.1, 0.3), []);

  const makeMpc = useCallback(() => {
    const costAt = (t0: number) =>
      navigationCost({ goal: GOAL, costToGo: params.costToGo ? ctg : null, walls, radius: RUSTY_R, obstacle: obstacleAt(t0 * DT, params.obstacleSpeed), vMax: params.vMax, weights: { ctg: 12, obstacle: 1500, wall: 1500 } });
    return new Mpc(f, costAt, { horizon: params.horizon, warmStart: params.warm, ilqr: { maxIter: 8, mu0: 1e-2 } });
  }, [f, walls, ctg, params.horizon, params.warm, params.obstacleSpeed, params.costToGo, params.vMax]);

  const init = useCallback((): Sim => ({ x: START.slice(), t: 0, plan: [], trail: [START.slice()], solveMs: 0, solveHist: [], ghostX: START.slice(), ghostPlan: [], ghostIndex: 0, ghostTrail: [START.slice()], ghostHit: false, hit: false, reached: false, mpc: makeMpc(), ghostMpc: makeMpc() }), [makeMpc]);

  const step = useCallback(
    (s: Sim): Sim => {
      if (s.reached || s.t > 600) return s;
      const tick = s.mpc.tick(s.x, s.t);
      const x = f.step(s.x, tick.u);
      const o = obstacleAt((s.t + 1) * DT, params.obstacleSpeed);
      const hit = s.hit || Math.hypot(x[0] - o.x, x[1] - o.y) < o.r + RUSTY_R || walls.dist({ x: x[0], y: x[1] }) < RUSTY_R;
      const reached = Math.hypot(x[0] - GOAL[0], x[1] - GOAL[1]) < 0.2;
      // Ghost: executes each plan to the end, then replans.
      let { ghostX, ghostPlan, ghostIndex, ghostHit } = s;
      if (params.ghost) {
        if (ghostIndex >= ghostPlan.length) {
          const g = s.ghostMpc.tick(ghostX, s.t);
          ghostPlan = g.controls;
          ghostIndex = 0;
        }
        const gu = ghostPlan[ghostIndex] ?? [0, 0];
        ghostIndex++;
        if (Math.hypot(ghostX[0] - GOAL[0], ghostX[1] - GOAL[1]) > 0.2) ghostX = f.step(ghostX, gu);
        ghostHit = ghostHit || Math.hypot(ghostX[0] - o.x, ghostX[1] - o.y) < o.r + RUSTY_R;
      }
      return {
        ...s,
        x,
        t: s.t + 1,
        plan: tick.plan,
        trail: [...s.trail, x].slice(-600),
        solveMs: tick.solveMs,
        solveHist: [...s.solveHist, tick.solveMs].slice(-60),
        ghostX,
        ghostPlan,
        ghostIndex,
        ghostTrail: [...s.ghostTrail, ghostX].slice(-600),
        ghostHit,
        hit,
        reached,
      };
    },
    [f, walls, params.obstacleSpeed, params.ghost],
  );
  const sim = useSimulation<Sim>({ init, step, fps: 20 });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawWorld(ctx, v, APARTMENT, p);
      const s = sim.state;
      const o = obstacleAt(s.t * DT, params.obstacleSpeed);
      // obstacle
      ctx.fillStyle = p.obstacle;
      ctx.beginPath();
      ctx.arc(sx(v, o.x), sy(v, o.y), sl(v, o.r), 0, 2 * Math.PI);
      ctx.fill();
      ctx.strokeStyle = p.cobstacle;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.arc(sx(v, o.x), sy(v, o.y), sl(v, o.r + RUSTY_R + 0.15), 0, 2 * Math.PI);
      ctx.stroke();
      ctx.setLineDash([]);
      // goal
      ctx.fillStyle = p.goal;
      ctx.beginPath();
      ctx.arc(sx(v, GOAL[0]), sy(v, GOAL[1]), 6, 0, 2 * Math.PI);
      ctx.fill();
      ctx.fillStyle = p.start;
      ctx.beginPath();
      ctx.arc(sx(v, START[0]), sy(v, START[1]), 6, 0, 2 * Math.PI);
      ctx.fill();
      // ghost
      if (params.ghost) {
        ctx.strokeStyle = p.graph;
        ctx.globalAlpha = 0.5;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        s.ghostTrail.forEach((x, i) => (i === 0 ? ctx.moveTo(sx(v, x[0]), sy(v, x[1])) : ctx.lineTo(sx(v, x[0]), sy(v, x[1]))));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
        drawRobot(ctx, v, { x: s.ghostX[0], y: s.ghostX[1], theta: s.ghostX[2] }, s.ghostHit ? p.goal : p.graph, RUSTY_R, { filled: false });
      }
      // trail and plan
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 2;
      ctx.beginPath();
      s.trail.forEach((x, i) => (i === 0 ? ctx.moveTo(sx(v, x[0]), sy(v, x[1])) : ctx.lineTo(sx(v, x[0]), sy(v, x[1]))));
      ctx.stroke();
      if (s.plan.length) {
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        s.plan.forEach((x, i) => (i === 0 ? ctx.moveTo(sx(v, x[0]), sy(v, x[1])) : ctx.lineTo(sx(v, x[0]), sy(v, x[1]))));
        ctx.stroke();
        ctx.fillStyle = p.path;
        const end = s.plan[s.plan.length - 1];
        ctx.beginPath();
        ctx.arc(sx(v, end[0]), sy(v, end[1]), 3.5, 0, 2 * Math.PI);
        ctx.fill();
      }
      drawRobot(ctx, v, { x: s.x[0], y: s.x[1], theta: s.x[2] }, s.hit ? p.goal : p.robot, RUSTY_R);
      const avg = s.solveHist.length ? s.solveHist.reduce((a, b) => a + b, 0) / s.solveHist.length : 0;
      label(ctx, `t = ${(s.t * DT).toFixed(1)} s · N_h = ${params.horizon} (${(params.horizon * DT).toFixed(1)} s look-ahead) · solve ${avg.toFixed(1)} ms/tick${s.reached ? ' · GOAL' : ''}${s.hit ? ' · COLLISION' : ''}`, sx(v, 0.2), sy(v, 8.75), s.hit ? p.goal : p.ink, { size: 10, weight: 600 });
      if (params.ghost) label(ctx, `open-loop ghost (replans only when its plan runs out): ${s.ghostHit ? 'collided' : 'clear so far'}`, sx(v, 0.2), sy(v, 8.45), s.ghostHit ? p.goal : p.ink, { size: 9 });
      if (!s.reached && s.t > 350) label(ctx, params.costToGo ? 'stuck — try a longer horizon or a slower obstacle' : 'stuck — a myopic dead end: the quadratic pull cannot see around the wall; switch the cost-to-go terminal on', sx(v, 0.2), sy(v, 0.25), p.goal, { size: 10, weight: 600 });
      // the cost-to-go field, faint, when it is in use
      if (params.costToGo) {
        const g = ctg.grid;
        for (let j = 0; j < g.ny; j += 2) {
          for (let i = 0; i < g.nx; i += 2) {
            if (g.occupied[j * g.nx + i]) continue;
            const cx = g.origin.x + (i + 0.5) * g.cellSize;
            const cy = g.origin.y + (j + 0.5) * g.cellSize;
            const val = ctg.at(cx, cy);
            ctx.fillStyle = p.path;
            ctx.globalAlpha = Math.max(0, 0.16 * (1 - val / 14));
            ctx.fillRect(sx(v, cx - g.cellSize), sy(v, cy + g.cellSize), sl(v, 2 * g.cellSize), sl(v, 2 * g.cellSize));
          }
        }
        ctx.globalAlpha = 1;
      }
    },
    [sim.state, params, ctg],
  );

  return (
    <WidgetFrame
      id="w19.3"
      title="MPC Horizon"
      teaches="A longer horizon is not always better: past the point where the plan is corrected anyway, horizon buys compute, not safety — and too short a horizon is myopic."
      colorKey={['path', 'robot', 'graph', 'obstacle', 'cobstacle', 'start', 'goal']}
      caption={
        <>
          Rusty crosses the Apartment from room A to room E under receding-horizon iLQR, Δt = 0.1 s, re-solving every tick from its measured pose
          with the disc at room B&apos;s door frozen where it currently is. Purple: the current horizon plan; orange: what was actually driven; the faint
          purple wash is the Chapter 6 cost-to-go used as terminal cost. The blue ghost executes each plan to the end before replanning and
          collides where MPC waits. Switch the cost-to-go off and every horizon is myopic: the quadratic pull parks Rusty against the corridor wall.
          Switch it back on and lengthen N_h: the solve time grows while the driven path barely changes.
        </>
      }
      wide
    >
      <SimCanvas world={APARTMENT.bounds} draw={draw} deps={[sim.tick]} aspect={12 / 9} padding={0.3} ariaLabel="Rusty in the Apartment under receding-horizon control, with the current plan, the driven path, a moving obstacle and an open-loop ghost." />
      <ControlPanel columns={3}>
        <Slider label="horizon N_h (ticks)" role="path" value={params.horizon} min={4} max={40} step={1} onChange={(h) => setParams((p) => ({ ...p, horizon: Math.round(h) }))} />
        <Slider label="obstacle speed" role="obstacle" value={params.obstacleSpeed} min={0} max={2} step={0.1} onChange={(obstacleSpeed) => setParams((p) => ({ ...p, obstacleSpeed }))} />
        <Slider label="speed bound v_max" role="robot" value={params.vMax} min={0.4} max={2} step={0.1} unit="m/s" onChange={(vMax) => setParams((p) => ({ ...p, vMax }))} />
        <Toggle label="terminal cost-to-go from Chapter 6 (off: quadratic pull)" role="path" checked={params.costToGo} onChange={(costToGo) => setParams((p) => ({ ...p, costToGo }))} />
        <Toggle label="warm start (shift last plan)" checked={params.warm} onChange={(warm) => setParams((p) => ({ ...p, warm }))} />
        <Toggle label="open-loop ghost" role="graph" checked={params.ghost} onChange={(ghost) => setParams((p) => ({ ...p, ghost }))} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
