'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawSegments, label, sl, sx, sy, readPalette, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Point2 } from '@/lib/sim/world';
import { Reach, t2, type T2 } from '@/lib/robots/reach';
import { WORKBENCH } from '@/lib/robots/worlds';
import { Scene } from '@/lib/collide/scene';
import { ChainChecker } from '@/lib/collide/collision';
import { RasterJob, torusChart } from '@/lib/cspace/raster';
import { Descender, type DescentOutcome } from '@/lib/potential/descent';
import { LiftedPotential, defaultControlPoints, type LiftedTerm } from '@/lib/potential/lifted';
import type { Vec } from '@/lib/potential/field';
import { drawRaster, drawReach, drawTorusAxes, fillPoly, rasterImage } from '@/components/ch/ch04/shared';
import { arrow, fmt, marker } from './shared';

/**
 * w7.3 — Arm in a Field.
 *
 * Reach on the Workbench under Choset §4.7's lifted potential: attractive
 * control points on the end effector pull it to the goal pose, repulsive
 * control points along the links push away from the block, the post and the
 * shelf, and every workspace force f_j is lifted to a joint torque J_jᵀ f_j
 * and *summed in configuration space* (eq. 4.26). The headline control is
 * r_float, the floating control point at the link point nearest any obstacle:
 * without it the fixed points on the links are repelled while the link's
 * interior sweeps straight through the post (Choset Fig. 4.20); with it the
 * same descent reaches the goal. The right pane is the unwrapped torus with
 * the Chapter 4 C-obstacle raster and the descent drawn on it.
 */

const BENCH = { minX: -2.4, minY: -2.4, maxX: 2.4, maxY: 2.4 };
const TORUS = { minX: -Math.PI, minY: -Math.PI, maxX: Math.PI, maxY: Math.PI };
const RASTER_DEG = 3;
const CELL_BUDGET = 1600;
type FloatMode = 'alternate' | 'on' | 'off';

interface State {
  descender: Descender;
  floating: boolean;
  trail: Vec[];
  outcome: DescentOutcome | null;
  idle: number;
  runs: number;
}

export function ArmInField() {
  const [mode, setMode] = useState<FloatMode>('alternate');
  const [zeta, setZeta] = useState(1);
  const [eta, setEta] = useState(0.05);
  const [qStar, setQStar] = useState(0.35);
  const [perLink, setPerLink] = useState(2);
  const [goalTip, setGoalTip] = useState<Point2>({ x: 1.3, y: 0.9 });
  const dragging = useRef(false);

  const arm = useMemo(() => Reach.workbench(), []);
  const scene = useMemo(() => Scene.fromWorld(WORKBENCH), []);
  const checker = useMemo(() => new ChainChecker(scene, arm), [scene, arm]);
  const goal = useMemo<T2>(() => {
    const [a, b] = arm.ik(goalTip);
    if (a && checker.isFree(a)) return a;
    if (b && checker.isFree(b)) return b;
    return a ?? b ?? [0, 1];
  }, [arm, goalTip, checker]);
  const points = useMemo(() => defaultControlPoints(arm, { zeta, eta, qStar, perLink }), [arm, zeta, eta, qStar, perLink]);
  const potentials = useMemo(
    () => ({
      off: new LiftedPotential(arm, scene, points, goal, false, { eta, qStar }),
      on: new LiftedPotential(arm, scene, points, goal, true, { eta, qStar }),
    }),
    [arm, scene, points, goal, eta, qStar],
  );

  // The torus raster of the Workbench C-obstacle, filled under a per-frame budget.
  const chart = useMemo(() => torusChart(Math.round(360 / RASTER_DEG)), []);
  const job = useRef<RasterJob<T2> | null>(null);
  const rasterImg = useRef<{ color: string; img: HTMLCanvasElement | null } | null>(null);
  const lastColor = useRef(readPalette().cobstacle);

  const makeDescender = useCallback(
    (U: LiftedPotential, start: Vec) =>
      new Descender(U, start, {
        alpha: 0.02,
        eps: 1e-4,
        maxSteps: 2500,
        goal: [goal[0], goal[1]],
        goalTol: 0.03,
        isFree: (q) => checker.isFree(q),
        maxStep: 0.05,
        stallWindow: 60,
        stallTol: 1e-4,
      }),
    [goal, checker],
  );

  const init = useCallback(
    (seed: number): State => {
      const rng = new Rng(seed);
      let start: Vec = [-2.75 + rng.uniform(-0.12, 0.12), 0.5 + rng.uniform(-0.2, 0.25)];
      for (let t = 0; t < 30 && !checker.isFree(start); t++) start = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
      const floating = mode === 'on';
      return { descender: makeDescender(floating ? potentials.on : potentials.off, start), floating, trail: [start], outcome: null, idle: 0, runs: 0 };
    },
    [checker, mode, potentials, makeDescender],
  );

  const step = useCallback(
    (s: State): State => {
      if (!job.current) job.current = new RasterJob<T2>(checker, chart);
      if (!job.current.done) job.current.advance(CELL_BUDGET);
      if (s.outcome) {
        if (s.idle < 36) return { ...s, idle: s.idle + 1 };
        // Alternate: rerun the same start with the other setting, so the comparison is exact.
        const floating = mode === 'alternate' ? !s.floating : mode === 'on';
        const start = s.descender.path[0];
        return { descender: makeDescender(floating ? potentials.on : potentials.off, start), floating, trail: [start], outcome: null, idle: 0, runs: s.runs + 1 };
      }
      let outcome: DescentOutcome | null = null;
      for (let k = 0; k < 2 && !outcome; k++) outcome = s.descender.step();
      return { ...s, trail: s.descender.path.slice(-2000), outcome };
    },
    [checker, chart, mode, potentials, makeDescender],
  );

  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 73 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [init]);

  const onBench = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') {
        dragging.current = false;
        return;
      }
      if (phase === 'down') {
        dragging.current = Math.hypot(w[0] - goalTip.x, w[1] - goalTip.y) < 0.3;
        return;
      }
      if (dragging.current) {
        const r = Math.hypot(w[0], w[1]);
        const lim = arm.reach() - 0.05;
        const k = r > lim ? lim / r : r < 0.1 ? 0.1 / (r || 1) : 1;
        const tip = { x: w[0] * k, y: w[1] * k };
        if (!scene.pointInObstacle(tip.x, tip.y)) setGoalTip(tip);
      }
    },
    [goalTip, arm, scene],
  );

  const s = sim.state;
  const q: Vec = s.trail[s.trail.length - 1];
  const U = s.floating ? potentials.on : potentials.off;
  const terms = useMemo(() => U.terms(q), [U, q]);
  const torque = useMemo(() => U.gradient(q), [U, q]);
  const inCollision = useMemo(() => !checker.isFree(q), [checker, q]);

  const drawBench = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      lastColor.current = p.cobstacle;
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 0.5);
      drawSegments(ctx, v, WORKBENCH.walls, p.wall, 2);
      for (const o of WORKBENCH.obstacles) fillPoly(ctx, v, o.poly, p.obstacle);
      // Goal pose as a dashed ghost.
      ctx.save();
      ctx.setLineDash([5, 4]);
      drawReach(ctx, v, p, arm, goal, p.goal, 0.55);
      ctx.restore();
      // Tip trail.
      ctx.save();
      ctx.strokeStyle = p.robot;
      ctx.globalAlpha = 0.5;
      ctx.lineWidth = 1.3;
      ctx.beginPath();
      s.trail.forEach((qq, i) => {
        const tip = arm.tip(qq);
        if (i === 0) ctx.moveTo(sx(v, tip.x), sy(v, tip.y));
        else ctx.lineTo(sx(v, tip.x), sy(v, tip.y));
      });
      ctx.stroke();
      ctx.restore();
      drawReach(ctx, v, p, arm, q, inCollision ? p.goal : p.robot);
      // Control points and their workspace forces (−∇U_j at r_j).
      const F = 0.35;
      for (const t of terms) {
        const isAtt = t.point.role.kind === 'attractive';
        const isFloat = t.point.label === 'r_float';
        const fn = Math.hypot(t.force.x, t.force.y);
        const L = Math.min(0.8, F * fn);
        if (fn > 1e-6) arrow(ctx, v, t.r.x, t.r.y, (t.force.x / fn) * L, (t.force.y / fn) * L, isAtt ? p.goal : isFloat ? p.robot : p.obstacle, isFloat ? 2.2 : 1.5);
        if (isFloat) marker(ctx, sx(v, t.r.x), sy(v, t.r.y), 6, p.bg, p.robot, 2.5);
        else marker(ctx, sx(v, t.r.x), sy(v, t.r.y), 3.5, isAtt ? p.goal : p.obstacle, p.bg);
      }
      marker(ctx, sx(v, goalTip.x), sy(v, goalTip.y), 6, p.goal, p.bg);
      const o = s.outcome;
      let text = `${s.floating ? 'r_float ON' : 'r_float OFF'} — descending, ‖u‖ = ${fmt(Math.hypot(torque[0], torque[1]), 3)}`;
      let color = p.ink;
      if (o?.kind === 'collision') {
        text = `${s.floating ? 'r_float ON' : 'r_float OFF'} — collision: a link's interior hit an obstacle between its control points (Fig. 4.20)`;
        color = p.goal;
      } else if (o?.kind === 'goal') text = `${s.floating ? 'r_float ON' : 'r_float OFF'} — reached the goal pose in ${o.steps} steps`;
      else if (o?.kind === 'localMinimum') {
        text = `${s.floating ? 'r_float ON' : 'r_float OFF'} — stuck in a local minimum of the lifted potential (eigs ${o.eigs.map((e) => fmt(e, 1)).join(', ')})`;
        color = p.goal;
      } else if (o?.kind === 'stalled') {
        text = `${s.floating ? 'r_float ON' : 'r_float OFF'} — stalled: chattering where the nearest obstacle switches`;
        color = p.goal;
      } else if (o?.kind === 'stepLimit') text = `${s.floating ? 'r_float ON' : 'r_float OFF'} — step limit`;
      label(ctx, text, 10, 14, color, { size: 10.5, weight: 600 });
      label(ctx, 'drag the red tip marker to move the goal', 10, v.height - 12, p.ink, { size: 9 });
    },
    [arm, goal, goalTip, s, q, terms, torque, inCollision],
  );

  const drawTorus = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const j = job.current;
      if (j?.done) {
        if (!rasterImg.current || rasterImg.current.color !== p.cobstacle) rasterImg.current = { color: p.cobstacle, img: rasterImage(j.grid, p.cobstacle) };
        if (rasterImg.current.img) drawRaster(ctx, v, rasterImg.current.img, chart);
      } else if (j) {
        const x = sx(v, -Math.PI + j.rowsDone * chart.d1);
        ctx.save();
        ctx.strokeStyle = p.cobstacle;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(x, sy(v, -Math.PI));
        ctx.lineTo(x, sy(v, Math.PI));
        ctx.stroke();
        ctx.restore();
      }
      drawTorusAxes(ctx, v, p);
      // The descent on the torus, broken at seams.
      ctx.save();
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      let prev: T2 | null = null;
      for (const qq of s.trail) {
        const w = t2(qq[0], qq[1]);
        if (!prev || Math.abs(w[0] - prev[0]) > Math.PI || Math.abs(w[1] - prev[1]) > Math.PI) ctx.moveTo(sx(v, w[0]), sy(v, w[1]));
        else ctx.lineTo(sx(v, w[0]), sy(v, w[1]));
        prev = w;
      }
      ctx.stroke();
      ctx.restore();
      const st = t2(s.trail[0][0], s.trail[0][1]);
      marker(ctx, sx(v, st[0]), sy(v, st[1]), 5, p.start, p.bg);
      marker(ctx, sx(v, goal[0]), sy(v, goal[1]), 6, p.goal, p.bg);
      const w = t2(q[0], q[1]);
      marker(ctx, sx(v, w[0]), sy(v, w[1]), 5.5, inCollision ? p.goal : p.robot, p.bg);
      // The lifted torque u as an arrow in joint space (descent goes along −u).
      const un = Math.hypot(torque[0], torque[1]);
      if (un > 1e-6 && !s.outcome) arrow(ctx, v, w[0], w[1], (-torque[0] / un) * 0.6, (-torque[1] / un) * 0.6, p.robot, 1.6);
      label(ctx, j?.done ? 'QO of the Workbench (Chapter 4 raster, 3° cells)' : `rasterizing QO … ${j ? (100 * j.progress).toFixed(0) : 0}%`, sx(v, -Math.PI), sy(v, Math.PI) - 12, p.ink, { size: 10 });
    },
    [s, q, goal, torque, inCollision, chart],
  );

  const maxBar = Math.max(0.05, ...terms.map((t) => Math.max(Math.abs(t.torque[0]), Math.abs(t.torque[1]))), Math.abs(torque[0]), Math.abs(torque[1]));

  return (
    <WidgetFrame
      id="w7.3"
      title="Arm in a Field"
      teaches="u = Jᵀ f: workspace forces at control points become joint torques summed in configuration space — and repelling a few fixed points does not keep the body safe."
      colorKey={['goal', 'obstacle', 'cobstacle', 'robot', 'start']}
      wide
      caption={
        <>
          Left, Reach on the Workbench with two attractive control points on the end effector (red
          arrows pull toward the dashed goal pose) and repulsive points at fixed fractions of each
          link (slate arrows push away from the block, the post and the shelf). The orange ring is
          r<sub>float</sub>, the link point nearest any obstacle, recomputed at every q. Right, the same
          descent on the unwrapped torus beside the Chapter 4 C-obstacle. The default alternates: the
          same start without r<sub>float</sub> drives link 1 through the post between its control
          points, then with it reaches the goal. The bars are each term&rsquo;s lifted torque
          J<sub>j</sub><sup>T</sup> f<sub>j</sub> on θ₁ and θ₂, and their sum — the gradient the arm descends.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border md:grid-cols-2">
        <div className="bg-fd-card">
          <div className="eyebrow px-2 py-1">workspace · the Workbench</div>
          <SimCanvas world={BENCH} draw={drawBench} deps={[sim.tick, s, goalTip]} aspect={1} padding={0.1} onPointer={onBench} ariaLabel="Reach, a two-link orange arm on a square table with three slate obstacles, with force arrows at control points along its links and a dashed red goal pose" />
        </div>
        <div className="bg-fd-card">
          <div className="eyebrow px-2 py-1">configuration space · T² with QO hatched</div>
          <SimCanvas world={TORUS} draw={drawTorus} deps={[sim.tick, s]} aspect={1} padding={0.45} ariaLabel="The unwrapped torus of joint angles with the C-obstacle hatched amber and the orange descent path from a green start to a red goal" />
        </div>
      </div>
      <div className="border-t border-fd-border px-3 py-2">
        <div className="eyebrow mb-1">lifted torques u = Σ<sub>j</sub> J<sub>j</sub><sup>T</sup> f<sub>j</sub> — per term, on (θ₁, θ₂)</div>
        <div className="grid grid-cols-1 gap-1 font-mono text-[0.68rem] tabular-nums sm:grid-cols-2">
          {terms.map((t, i) => (
            <TorqueBar key={i} term={t} max={maxBar} />
          ))}
          <div className="flex items-center gap-2 border-t border-fd-border pt-1 sm:col-span-2">
            <span className="w-16 shrink-0 font-semibold">Σ u</span>
            <Bars a={torque[0]} b={torque[1]} max={maxBar} color="var(--rm-robot)" />
            <span className="text-fd-muted-foreground">({fmt(torque[0], 3)}, {fmt(torque[1], 3)})</span>
          </div>
        </div>
      </div>
      <ControlPanel columns={3} title="Headline: the floating control point r_float">
        <ButtonRow>
          <ActionButton emphasis={mode === 'alternate'} onClick={() => setMode('alternate')}>alternate off / on</ActionButton>
          <ActionButton emphasis={mode === 'off'} onClick={() => setMode('off')}>r_float off</ActionButton>
          <ActionButton emphasis={mode === 'on'} onClick={() => setMode('on')}>r_float on</ActionButton>
        </ButtonRow>
        <Slider label="repulsive points per link" role="obstacle" value={perLink} min={1} max={5} step={1} onChange={(v) => setPerLink(Math.round(v))} />
        <Slider label="Q* at the links" role="obstacle" value={qStar} min={0.1} max={0.8} step={0.05} unit="m" onChange={setQStar} />
      </ControlPanel>
      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Advanced: gains</summary>
        <ControlPanel columns={2}>
          <Slider label="ζ — attractive gain at the end effector" role="goal" value={zeta} min={0.2} max={3} step={0.1} onChange={setZeta} />
          <Slider label="η — repulsive gain at the links" role="obstacle" value={eta} min={0.005} max={0.3} step={0.005} format={(v) => v.toFixed(3)} onChange={setEta} />
        </ControlPanel>
      </details>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}

function Bars({ a, b, max, color }: { a: number; b: number; max: number; color: string }) {
  const w = (x: number) => `${Math.min(100, (50 * Math.abs(x)) / max)}%`;
  const bar = (x: number) => (
    <div className="relative h-2 w-full overflow-hidden rounded-sm bg-fd-muted">
      <div className="absolute top-0 h-full" style={{ left: x >= 0 ? '50%' : `calc(50% - ${w(x)})`, width: w(x), background: color, opacity: 0.85 }} />
      <div className="absolute top-0 h-full w-px bg-fd-border" style={{ left: '50%' }} />
    </div>
  );
  return (
    <div className="flex w-full items-center gap-1.5">
      <span className="w-5 text-fd-muted-foreground">θ₁</span>
      {bar(a)}
      <span className="w-5 text-fd-muted-foreground">θ₂</span>
      {bar(b)}
    </div>
  );
}

function TorqueBar({ term, max }: { term: LiftedTerm; max: number }) {
  const isAtt = term.point.role.kind === 'attractive';
  const isFloat = term.point.label === 'r_float';
  const color = isAtt ? 'var(--rm-goal)' : isFloat ? 'var(--rm-robot)' : 'var(--rm-obstacle)';
  return (
    <div className="flex items-center gap-2">
      <span className="w-16 shrink-0" style={{ color }}>
        {term.point.label ?? `L${term.point.link + 1}`}
      </span>
      <Bars a={term.torque[0]} b={term.torque[1]} max={max} color={color} />
    </div>
  );
}
