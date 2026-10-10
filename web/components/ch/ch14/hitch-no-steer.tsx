'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Pose2 } from '@/lib/geom/se2';
import { LOT } from '@/lib/robots/worlds';
import type { CarCmd } from '@/lib/robots/hitch';
import { LOT_GOAL, LOT_START, Rrt, StraightLine, paintDot, paintPolyWorld, rectMapper, se2Lab, tint, type Se2Lab } from '@/lib/sampling';
import { KinoRrtPreview, headingMismatch, hitchControls, hitchPropagator } from '@/lib/multi';
import { drawCar } from '@/components/ch/ch02/three-robot-dashboard';

/**
 * w14.3 — Hitch Without a Steer (PREVIEW; Chapter 21 owns the full treatment).
 *
 * Forward-propagated RRT for the car in the Lot. Every extension samples k
 * controls, runs each through Hitch's Chapter 2 kinematic update for Δt — a
 * black box here — and keeps the collision-free result closest to q_rand.
 * Chapter 12's straight-line RRT on SE(2) is ghosted in grey: its edges are
 * sideways slides no car can make, and the readout says by how much.
 */

const SPEED = 1.5;
const ITERS_PER_TICK = 12;
const BUDGET = 4000;

interface State {
  lab: Se2Lab;
  kin: KinoRrtPreview<Pose2, CarCmd>;
  ghost: Rrt<Pose2>;
  dense: Pose2[] | null;
  k: number;
  ghostMismatch: number | null;
}

export function HitchNoSteer() {
  const [controls, setControls] = useState(8);
  const [dt, setDt] = useState(0.5);
  const [goalBias, setGoalBias] = useState(0.1);

  const init = useCallback(
    (seed: number): State => {
      const lab = se2Lab();
      const kin = new KinoRrtPreview<Pose2, CarCmd>(lab.space, lab.free, hitchPropagator(lab.car), LOT_START, LOT_GOAL, { controlsPerExtension: controls, dt, substeps: 5, goalBias, goalRadius: 0.7, sampleControl: hitchControls(lab.car, SPEED) }, new Rng(seed));
      const ghostLab = se2Lab();
      const ghost = new Rrt<Pose2>(ghostLab.space, ghostLab.free, LOT_START, LOT_GOAL, { steer: new StraightLine<Pose2>({ kind: 'subdivision', step: 0.1 }), eta: SPEED * dt, goalBias, goalRadius: 0.7 }, new Rng(seed));
      return { lab, kin, ghost, dense: null, k: 0, ghostMismatch: null };
    },
    [controls, dt, goalBias],
  );

  const stepFn = useCallback((s: State): State => {
    const running = !s.kin.solved && s.kin.iterations < BUDGET;
    if (running) for (let i = 0; i < ITERS_PER_TICK && !s.kin.solved && s.kin.iterations < BUDGET; i++) s.kin.extendOnce();
    if (!s.ghost.solved && s.ghost.iterations < BUDGET) for (let i = 0; i < ITERS_PER_TICK && !s.ghost.solved && s.ghost.iterations < BUDGET; i++) s.ghost.extendOnce();
    if (s.kin.solved && !s.dense) s.dense = s.kin.densePath();
    if (s.ghost.solved && s.ghostMismatch === null) {
      const gp = s.ghost.path();
      if (gp) s.ghostMismatch = headingMismatch(gp);
    }
    if (s.dense) s.k = (s.k + 1) % (s.dense.length + 20);
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 14 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [controls, dt, goalBias]);

  const s = sim.state;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const m = rectMapper(v, { x0: LOT.bounds.minX, y0: LOT.bounds.minY, x1: LOT.bounds.maxX, y1: LOT.bounds.maxY }, LOT.bounds);
      paintPolyWorld(ctx, m, LOT, p);
      // Ghost: Chapter 12's straight-line tree, grey.
      ctx.save();
      ctx.strokeStyle = tint(p.ink, 72, p.bg);
      ctx.lineWidth = 0.8;
      ctx.globalAlpha = 0.8;
      ctx.beginPath();
      const gt = s.ghost.tree;
      for (let i = 1; i < gt.size; i++) {
        const q = gt.nodes[i].q;
        const pq = gt.nodes[gt.nodes[i].parent].q;
        ctx.moveTo(sx(v, pq.x), sy(v, pq.y));
        ctx.lineTo(sx(v, q.x), sy(v, q.y));
      }
      ctx.stroke();
      ctx.restore();
      // The propagated tree: every edge is the simulator's own arc.
      ctx.save();
      ctx.strokeStyle = p.graph;
      ctx.lineWidth = 1.1;
      ctx.globalAlpha = 0.85;
      ctx.beginPath();
      for (let i = 1; i < s.kin.tree.size; i++) {
        const pq = s.kin.tree.nodes[s.kin.tree.nodes[i].parent].q;
        ctx.moveTo(sx(v, pq.x), sy(v, pq.y));
        for (const q of s.kin.segment[i]) ctx.lineTo(sx(v, q.x), sy(v, q.y));
      }
      ctx.stroke();
      ctx.restore();
      if (s.dense) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        s.dense.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
        ctx.stroke();
        ctx.restore();
      }
      const gp = s.ghost.solved ? s.ghost.path() : null;
      if (gp) {
        ctx.save();
        ctx.strokeStyle = tint(p.ink, 45, p.bg);
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        gp.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
        ctx.stroke();
        ctx.restore();
      }
      drawCar(ctx, v, p, s.lab.car, { body: LOT_START, trailer: null }, -1, p.start);
      drawCar(ctx, v, p, s.lab.car, { body: LOT_GOAL, trailer: null }, -1, p.goal);
      paintDot(ctx, sx(v, LOT_GOAL.x), sy(v, LOT_GOAL.y), Math.max(3, (0.7 / (v.maxX - v.minX)) * v.width), p.goal, 0.25);
      if (s.dense) {
        const q = s.dense[Math.min(s.k, s.dense.length - 1)];
        drawCar(ctx, v, p, s.lab.car, { body: q, trailer: null }, -1, p.robot);
      }
      const status = s.kin.solved ? `goal region reached at iteration ${s.kin.solvedAt} — replaying the stored controls through Hitch.step` : s.kin.iterations >= BUDGET ? 'budget exhausted' : `growing… ${s.kin.iterations} iterations, ${s.kin.tree.size} nodes`;
      label(ctx, status, sx(v, 0.2), sy(v, 9.6), s.kin.solved ? p.path : p.ink, { size: 10.5, weight: 700 });
      label(ctx, 'blue: arcs produced by the simulator · grey: Chapter 12 straight-line RRT on SE(2), undrivable', sx(v, 0.2), sy(v, 0.35), p.ink, { size: 9.5 });
      label(ctx, 'preview — the simulator is a black box here; Chapter 21 opens it', sx(v, 15.8), sy(v, 0.35), p.ink, { size: 9.5, align: 'right', weight: 600 });
    },
    [s],
  );

  return (
    <WidgetFrame
      id="w14.3"
      title="Hitch Without a Steer"
      teaches="An RRT does not need a steering function: sample controls, integrate the simulator, keep the free result closest to q_rand — and every edge is drivable by construction."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Each extension samples the headline number of controls (speed ±1.5 m/s, steering within ±0.6 rad),
          integrates Hitch’s Chapter 2 kinematic update for Δt through five collision-checked substeps, and
          keeps the free endpoint closest to the random target. The blue tree is made of arcs the car can
          actually drive; the purple path is replayed by feeding the stored controls back into the same
          update, and lands on the same nodes exactly. The grey tree is Chapter 12’s RRT on SE(2) treating
          θ as a third coordinate — its path slides sideways by the number of radians in the readout. With
          one control per extension the tree wanders; with twenty, most propagations are discarded. Nothing
          about <em>why</em> the car moves this way is used here; that is Chapter 21.
        </>
      }
    >
      <SimCanvas world={LOT.bounds} aspect={16 / 10} padding={0.3} draw={draw} deps={[sim.tick, s]} ariaLabel="A parking lot with a tree of drivable arcs grown by forward propagation, a grey straight-line tree for comparison, and a car replaying the found path." />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="nodes" value={s.kin.tree.size} precision={0} role="graph" />
          <StatTile label="propagations · wasted" value={`${s.kin.propagations} · ${s.kin.wasted}`} />
          <StatTile label="solved at iteration" value={s.kin.solvedAt ?? (s.kin.iterations >= BUDGET ? 'never' : '—')} role="path" />
          <StatTile label="ghost path sideways slide" value={s.ghostMismatch !== null ? s.ghostMismatch : s.ghost.iterations >= BUDGET ? 'no ghost path' : '—'} precision={2} unit="rad" role="obstacle" />
        </Dashboard>
      </div>
      <ControlPanel columns={3} title="headline — controls sampled per extension">
        <Slider label="controls per extension" role="graph" value={controls} min={1} max={20} step={1} onChange={setControls} format={(x) => x.toFixed(0)} />
        <Slider label="Δt" value={dt} min={0.3} max={0.8} step={0.05} onChange={setDt} unit="s" />
        <Slider label="goal bias" role="goal" value={goalBias} min={0} max={0.3} step={0.01} onChange={setGoalBias} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
