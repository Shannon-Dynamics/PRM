'use client';

import { useCallback, useMemo } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Vec } from '@/lib/prob/linalg';
import { Reach2R } from '@/lib/dynamics/reach2r';
import { WORKBENCH } from '@/lib/robots/worlds';
import { Rrt } from '@/lib/sampling/tree';
import { StraightLine, shortcutGreedy } from '@/lib/sampling/steer';
import { torusLab, TORUS_GOAL, TORUS_START } from '@/lib/sampling/worlds';
import { PathDynamics } from '@/lib/trajectory/constrained';
import { symmetricLimits } from '@/lib/trajectory/limits';
import { CubicSpline, unwrapWaypoints } from '@/lib/trajectory/path';
import { phaseAt, timeScale, type TimeScaling } from '@/lib/trajectory/time-scale';
import { chomp, resamplePolyline } from '@/lib/trajopt/chomp';
import { reachWorkbench } from '@/lib/trajopt/micro';

/**
 * The chapter hook: a Chapter 12 RRT path for Reach (seeded), shortcut by
 * Chapter 13's greedy shortcutter, then thirty-odd CHOMP iterations — and both
 * handed to Chapter 18's time scaler on a horizontal Workbench. Same start, same
 * goal, same motors; the optimized path is smoother, keeps a chosen clearance
 * from the block, and executes faster because its q″ is smaller everywhere.
 */

const WORLD = { minX: 0, minY: 0, maxX: 10.4, maxY: 5.2 };
const SLOW = 0.4;

interface Lane {
  title: string;
  spline: CubicSpline;
  sc: TimeScaling | null;
  waypoints: Vec[];
  err?: string;
}

function buildLanes(seed: number): Lane[] {
  const lab = torusLab(WORKBENCH);
  const rng = new Rng(seed);
  const rrt = new Rrt<number[]>(lab.space, lab.free, TORUS_START, TORUS_GOAL, { steer: new StraightLine({ kind: 'subdivision', step: 0.05 }), eta: 0.4, goalBias: 0.1 }, rng);
  const raw = rrt.plan(4000) ?? [TORUS_START, TORUS_GOAL];
  const cut = shortcutGreedy(new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), lab.space, lab.free, raw);
  const jagged = unwrapWaypoints(raw.map((q) => [...q]));
  const base = unwrapWaypoints(cut.map((q) => [...q]));
  const scene = reachWorkbench(24, 0.05);
  const { q0, qEnd, xi } = resamplePolyline(base, 24);
  const eta = 8;
  const run = chomp({ q0, qEnd, field: scene.field, body: scene.body, eps: 0.2, lambdaObs: 3, eta, obstacleForm: 'arclength' }, xi, 40);
  const smooth = [q0, ...run.xi, qEnd];
  const arm = Reach2R.micro(0);
  const lim = symmetricLimits([20, 12]);
  const mk = (title: string, wps: Vec[]): Lane => {
    const spline = new CubicSpline(wps, false);
    const res = timeScale(new PathDynamics(arm, spline), lim, { N: 240 });
    return { title, spline, sc: res.ok ? res.scaling : null, waypoints: wps, err: res.ok ? undefined : res.error.kind };
  };
  return [mk(`RRT path, ${jagged.length} nodes, time-scaled`, jagged), mk(`after shortcut + ${40} CHOMP steps, time-scaled`, smooth)];
}

export function BeforeAfterHook() {
  const lanes = useMemo(() => buildLanes(19), []);
  const tfMax = Math.max(...lanes.map((l) => l.sc?.tf ?? 1), 0.5);
  const arm = useMemo(() => Reach2R.micro(0), []);
  const init = useCallback(() => ({ t: 0 }), []);
  const step = useCallback((s: { t: number }) => ({ t: s.t + SLOW / 30 }), []);
  const sim = useSimulation({ init, step, fps: 30, maxTicks: Math.round(((tfMax + 0.8) * 30) / SLOW), loop: true });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const t = sim.state.t;
      lanes.forEach((lane, li) => {
        const cx = 2.6 + li * 5.2;
        const cy = 2.6;
        const X = (x: number) => sx(v, cx + x);
        const Y = (y: number) => sy(v, cy + y);
        ctx.strokeStyle = p.wall;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(X(-2.4), Y(2.4), sl(v, 4.8), sl(v, 4.8));
        ctx.fillStyle = p.obstacle;
        ctx.globalAlpha = 0.5;
        for (const o of WORKBENCH.obstacles) {
          ctx.beginPath();
          o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(X(pt.x), Y(pt.y)) : ctx.lineTo(X(pt.x), Y(pt.y))));
          ctx.closePath();
          ctx.fill();
        }
        ctx.globalAlpha = 1;
        // tip trace of the path
        ctx.strokeStyle = li === 0 ? p.graph : p.path;
        ctx.lineWidth = 2;
        ctx.beginPath();
        for (let i = 0; i <= 160; i++) {
          const tip = arm.points(lane.spline.q(i / 160)).tip;
          if (i === 0) ctx.moveTo(X(tip[0]), Y(tip[1]));
          else ctx.lineTo(X(tip[0]), Y(tip[1]));
        }
        ctx.stroke();
        // waypoints
        ctx.fillStyle = li === 0 ? p.graph : p.path;
        for (const w of lane.waypoints) {
          const tip = arm.points(w).tip;
          ctx.beginPath();
          ctx.arc(X(tip[0]), Y(tip[1]), 2.5, 0, 2 * Math.PI);
          ctx.fill();
        }
        // the arm at time t
        let s = 1;
        if (lane.sc) s = t >= lane.sc.tf ? 1 : phaseAt(lane.sc, t).s;
        const q = lane.spline.q(s);
        const pts = arm.points(q);
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 5;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(X(0), Y(0));
        ctx.lineTo(X(pts.elbow[0]), Y(pts.elbow[1]));
        ctx.lineTo(X(pts.tip[0]), Y(pts.tip[1]));
        ctx.stroke();
        ctx.fillStyle = p.ink;
        ctx.beginPath();
        ctx.arc(X(0), Y(0), 3, 0, 2 * Math.PI);
        ctx.fill();
        label(ctx, lane.title, X(-2.35), sy(v, 5.05), p.ink, { size: 10, weight: 600 });
        label(ctx, lane.sc ? `t_f = ${lane.sc.tf.toFixed(3)} s${t >= lane.sc.tf ? ' — done' : ''}` : `no feasible clock (${lane.err})`, X(-2.35), sy(v, 0.1), lane.sc ? p.ink : p.goal, { size: 10 });
      });
      label(ctx, `t = ${Math.min(t, tfMax).toFixed(2)} s · horizontal Workbench, |τ| ≤ (20, 12) N·m`, sx(v, 3.6), sy(v, 5.05), p.ink, { size: 9 });
    },
    [lanes, sim.state, arm, tfMax],
  );

  return (
    <div className="not-prose overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick]} aspect={2} padding={0.1} ariaLabel="Reach executing a jagged RRT path and, beside it, the same query after shortcutting and CHOMP, both on their time-optimal clocks; the optimized path finishes first." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onReset={sim.reset} tick={sim.tick} />
    </div>
  );
}
