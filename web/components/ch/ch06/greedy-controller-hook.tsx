'use client';

import { useCallback } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawPath, drawRobot, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT, isFree } from '@/lib/sim/world';
import type { Pose2 } from '@/lib/geom/se2';

/**
 * The chapter hook (unnumbered in the manifest): Rusty in room A, goal in
 * room B, and the first controller anyone writes — drive toward the goal. It
 * wedges into the party wall at x = 4 and stays there. Something must look
 * ahead; the rest of the chapter is what that something is.
 */

const GOAL = { x: 6.0, y: 2.6 };
const START: Pose2 = { x: 1.0, y: 1.0, theta: 0 };
const RADIUS = 0.22;
const VIEW = { minX: 0, minY: 0, maxX: 8.5, maxY: 5.2 };

interface State {
  pose: Pose2;
  trail: { x: number; y: number }[];
  wedged: boolean;
  wedgedFor: number;
}

export function GreedyControllerHook() {
  const init = useCallback((): State => ({ pose: START, trail: [START], wedged: false, wedgedFor: 0 }), []);
  const step = useCallback((s: State): State => {
    if (s.wedged) return { ...s, wedgedFor: s.wedgedFor + 1 };
    const dx = GOAL.x - s.pose.x, dy = GOAL.y - s.pose.y;
    const d = Math.hypot(dx, dy);
    if (d < 0.1) return { ...s, wedged: true };
    const theta = Math.atan2(dy, dx);
    const nx = s.pose.x + 0.06 * Math.cos(theta), ny = s.pose.y + 0.06 * Math.sin(theta);
    if (!isFree(APARTMENT, nx, ny, RADIUS)) return { ...s, pose: { ...s.pose, theta }, wedged: true };
    const pose = { x: nx, y: ny, theta };
    return { ...s, pose, trail: [...s.trail, pose] };
  }, []);
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 60, maxTicks: 160, loop: true });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
      const s = sim.state;
      // The straight line the controller believes in.
      drawPath(ctx, v, [START, GOAL], p.grid, { dashed: true, lineWidth: 1.5 });
      drawPath(ctx, v, s.trail, p.robot, { lineWidth: 2.5 });
      ctx.save();
      ctx.fillStyle = p.goal;
      ctx.beginPath();
      ctx.arc(sx(v, GOAL.x), sy(v, GOAL.y), sl(v, 0.2), 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = p.start;
      ctx.beginPath();
      ctx.arc(sx(v, START.x), sy(v, START.y), sl(v, 0.14), 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      drawRobot(ctx, v, s.pose, p.robot, RADIUS * 1.6);
      label(ctx, 'room A', sx(v, 1.9), sy(v, 3.4), p.truth, { size: 11 });
      label(ctx, 'room B', sx(v, 6.0), sy(v, 3.4), p.truth, { size: 11 });
      label(ctx, 'corridor', sx(v, 4.2), sy(v, 4.6), p.truth, { size: 11 });
      if (s.wedged) label(ctx, 'wedged against the party wall — the goal is 2.2 m away through it', sx(v, s.pose.x) + 14, sy(v, s.pose.y) - 14, p.goal, { size: 11, weight: 700 });
    },
    [sim.state],
  );

  return (
    <div className="not-prose my-6 overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas
        world={VIEW}
        draw={draw}
        deps={[sim.tick, sim.state]}
        aspect={8.5 / 5.2}
        padding={0.15}
        ariaLabel="A floorplan with a robot in one room driving straight toward a goal in the next room and stopping against the wall between them."
      />
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} />
    </div>
  );
}
