'use client';

import { useCallback } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawPath, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT } from '@/lib/sim/world';
import { discLab, latticeFloor } from '@/lib/sampling';
import { pathToTrajectory, trajectoriesConflict, trajectoryAt, type Trajectory } from '@/lib/multi';

/**
 * The hook — two Rustys in the Apartment, each planned alone (Chapter 6's
 * lattice A* on the inflated grid, the single-robot answer every planner in
 * Part II–III would give), both paths collision-free, executed together.
 * They meet head-on in the corridor. No randomness: the same two paths, the
 * same collision, every time; the transport only replays it.
 */

const SPEED = 0.6; // m/s
const DT = 0.1; // s per tick
const START_A = [1.0, 1.0];
const GOAL_A = [11.0, 1.5];

interface State {
  pathA: { x: number; y: number }[];
  pathB: { x: number; y: number }[];
  trA: Trajectory;
  trB: Trajectory;
  radius: number;
  t: number;
  tCollide: number | null;
  where: number[] | null;
}

export function CollideHook() {
  const init = useCallback((): State => {
    const lab = discLab();
    const a = latticeFloor(lab, START_A, GOAL_A);
    const b = latticeFloor(lab, GOAL_A, START_A);
    const pathA = a?.path ?? [{ x: START_A[0], y: START_A[1] }];
    const pathB = b?.path ?? [{ x: GOAL_A[0], y: GOAL_A[1] }];
    const trA = pathToTrajectory(pathA.map((q) => [q.x, q.y]), SPEED, DT);
    const trB = pathToTrajectory(pathB.map((q) => [q.x, q.y]), SPEED, DT);
    const c = trajectoriesConflict([trA, trB], lab.radius, DT);
    return { pathA, pathB, trA, trB, radius: lab.radius, t: 0, tCollide: c ? c.t : null, where: c ? trajectoryAt(trA, c.t) : null };
  }, []);

  const step = useCallback((s: State): State => {
    const end = s.tCollide !== null ? s.tCollide + 1.6 : Math.max(s.trA.q.length, s.trB.q.length) * DT;
    const t = s.t + DT;
    return { ...s, t: t > end ? 0 : t };
  }, []);

  const sim = useSimulation<State>({ init, step, fps: 20, initialSeed: 14 });
  const s = sim.state;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawWorld(ctx, v, APARTMENT, p);
      drawPath(ctx, v, s.pathA, p.robot, { lineWidth: 2, alpha: 0.6 });
      drawPath(ctx, v, s.pathB, p.robot, { lineWidth: 2, alpha: 0.6, dashed: true });
      const tShown = s.tCollide !== null ? Math.min(s.t, s.tCollide) : s.t;
      const qa = trajectoryAt(s.trA, tShown);
      const qb = trajectoryAt(s.trB, tShown);
      const disc = (q: number[], dashed: boolean, name: string) => {
        ctx.save();
        ctx.fillStyle = p.robot;
        ctx.strokeStyle = p.robot;
        ctx.globalAlpha = 0.9;
        if (dashed) ctx.setLineDash([4, 3]);
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(sx(v, q[0]), sy(v, q[1]), sl(v, s.radius), 0, Math.PI * 2);
        if (dashed) ctx.stroke();
        else ctx.fill();
        ctx.restore();
        label(ctx, name, sx(v, q[0]), sy(v, q[1]) - sl(v, s.radius) - 8, p.ink, { size: 10, align: 'center', weight: 600 });
      };
      disc(qa, false, 'Rusty 1');
      disc(qb, true, 'Rusty 2');
      const collided = s.tCollide !== null && s.t >= s.tCollide;
      if (collided && s.where) {
        const age = (s.t - s.tCollide!) / 1.6;
        ctx.save();
        ctx.strokeStyle = p.goal;
        ctx.lineWidth = 3;
        ctx.globalAlpha = Math.max(0, 1 - age);
        ctx.beginPath();
        ctx.arc(sx(v, s.where[0]), sy(v, s.where[1]), sl(v, s.radius * (2 + 3 * age)), 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
        label(ctx, `collision at t = ${s.tCollide!.toFixed(1)} s — each path was free alone`, v.width / 2, v.height - 6, p.goal, { size: 11, align: 'center', weight: 700 });
      } else {
        label(ctx, `t = ${tShown.toFixed(1)} s — two single-robot plans, both collision-free, executed together`, v.width / 2, v.height - 6, p.ink, { size: 10, align: 'center', weight: 600 });
      }
    },
    [s],
  );

  return (
    <WidgetFrame
      id="w14.0"
      title="Two Rustys collide"
      teaches="Two individually collision-free paths are not a collision-free plan: the second body is an obstacle that moves."
      colorKey={['obstacle', 'robot']}
      caption={
        <>
          Rusty 1 (solid) plans from room A to room C; Rusty 2 (dashed) plans the reverse. Each path is the
          shortest lattice path of Chapter 6 on the inflated Apartment, and each is free of every wall. Run
          together at 0.6 m/s they meet head-on in the corridor at the time the red ring marks. Nothing in a
          single-robot planner can see this: the collision is between two configurations that are never
          checked against each other.
        </>
      }
    >
      <SimCanvas world={APARTMENT.bounds} aspect={12 / 9} padding={0.3} draw={draw} deps={[sim.tick, s]} ariaLabel="Two disc robots in the Apartment following individually planned paths and colliding head-on in the corridor." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
