'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Point2 } from '@/lib/sim/world';
import { cellAt, cellCenter, type Connectivity } from '@/lib/search/grid';
import { Additive, Attractive, Repulsive, type Vec } from '@/lib/potential/field';
import { Descender, type DescentOutcome } from '@/lib/potential/descent';
import { PlanarObstacles, horseshoe, horseshoePieces, rasterizeObstacles, type PlanarObstacle } from '@/lib/potential/planar';
import { descendWavefront, wavefront, type DistGrid } from '@/lib/potential/brushfire';
import { arrow, drawField, fieldImage, fmt, marker } from './shared';

/**
 * w7.1 — the Potential Well.
 *
 * A heightfield of U = U_att + Σ U_rep over a Workbench-sized square, a red
 * goal and a slate horseshoe (or two convex discs) you can drag, and a bead
 * that rolls down −∇U from a seeded start. With the horseshoe it stops in the
 * mouth; with two discs it stops between them — convexity is not the issue,
 * the *sum* is. Flip to wave-front mode and the heightfield becomes the BFS
 * cost-to-goal of Chapter 6, which has one minimum, and the bead escapes along
 * a grid-shortest path. The headline control is Q*, the repulsive radius.
 */

const BOUNDS = { minX: -3, minY: -3, maxX: 3, maxY: 3 };
const FIELD_N = 96;
const WF_CELL = 0.1;
type ObstacleSet = 'horseshoe' | 'discs' | 'both';

interface State {
  descender: Descender;
  trail: Vec[];
  outcome: DescentOutcome | null;
  wfPath: number[];
  wfIndex: number;
  start: Vec;
  idle: number;
}

export function PotentialWell() {
  const [qStar, setQStar] = useState(0.8);
  const [zeta, setZeta] = useState(1);
  const [eta, setEta] = useState(1);
  const [alpha, setAlpha] = useState(0.01);
  const [set, setSet] = useState<ObstacleSet>('horseshoe');
  const [convexPieces, setConvexPieces] = useState(true);
  const [waveMode, setWaveMode] = useState(false);
  const [conn, setConn] = useState<Connectivity>(4);
  const [goal, setGoal] = useState<Point2>({ x: 2.2, y: 0 });
  const [center, setCenter] = useState<Point2>({ x: 0.3, y: 0 });
  const dragging = useRef<'goal' | 'obstacle' | null>(null);

  // The obstacle set: the horseshoe as three convex rectangles (Choset's per-obstacle
  // sum keeps ∇U continuous) or as one nonconvex polygon (∇d jumps on its axis).
  const obstacles = useMemo(() => {
    const list: PlanarObstacle[] = [];
    if (set !== 'discs') {
      if (convexPieces) for (const poly of horseshoePieces(center)) list.push({ kind: 'polygon', poly });
      else list.push({ kind: 'polygon', poly: horseshoe(center) });
    }
    if (set !== 'horseshoe') {
      const dx = set === 'both' ? -1.6 : 0;
      list.push({ kind: 'disc', c: { x: center.x + dx, y: center.y + 0.7 }, r: 0.45 });
      list.push({ kind: 'disc', c: { x: center.x + dx, y: center.y - 0.7 }, r: 0.45 });
    }
    return new PlanarObstacles(list);
  }, [set, convexPieces, center]);

  const goalVec = useMemo<Vec>(() => [goal.x, goal.y], [goal]);
  const U = useMemo(() => new Additive([new Attractive(goalVec, zeta, 1.5), new Repulsive(obstacles, eta, qStar)]), [goalVec, zeta, eta, qStar, obstacles]);

  // The wave-front field (Derivation 4): BFS from the goal cell on a 0.1 m raster.
  const wf = useMemo<DistGrid | null>(() => {
    if (!waveMode) return null;
    const grid = rasterizeObstacles(obstacles, BOUNDS, WF_CELL, 0.06);
    const g = cellAt(grid, goal.x, goal.y);
    if (grid.occupied[g]) return null;
    return wavefront(grid, g, conn);
  }, [waveMode, obstacles, goal, conn]);

  // Heightfield samples of U (log-compressed so the repulsive spikes do not wash everything out).
  const field = useMemo(() => {
    const vals = new Float32Array(FIELD_N * FIELD_N);
    const cs = (BOUNDS.maxX - BOUNDS.minX) / FIELD_N;
    let max = 0;
    for (let j = 0; j < FIELD_N; j++) {
      for (let i = 0; i < FIELD_N; i++) {
        const x = BOUNDS.minX + (i + 0.5) * cs;
        const y = BOUNDS.minY + (j + 0.5) * cs;
        const inside = obstacles.inside({ x, y });
        const v = inside ? -1 : Math.min(U.value([x, y]), 60);
        vals[j * FIELD_N + i] = v;
        if (v > max) max = v;
      }
    }
    return { vals, max };
  }, [U, obstacles]);

  const imgCache = useRef<{ key: string; img: HTMLCanvasElement | null } | null>(null);

  const init = useCallback(
    (seed: number): State => {
      const rng = new Rng(seed);
      let start: Vec = [-2.4 + rng.uniform(-0.3, 0.2), rng.uniform(-0.6, 0.6)];
      for (let t = 0; t < 20 && obstacles.inside({ x: start[0], y: start[1] }); t++) start = [-2.4, rng.uniform(-1, 1)];
      const descender = new Descender(U, start, {
        alpha,
        eps: 5e-3,
        maxSteps: 6000,
        goal: goalVec,
        goalTol: 0.06,
        isFree: (q) => !obstacles.inside({ x: q[0], y: q[1] }) && Math.abs(q[0]) < 3.2 && Math.abs(q[1]) < 3.2,
        maxStep: 0.05,
        stallWindow: 40,
        stallTol: 1e-3,
      });
      let wfPath: number[] = [];
      if (wf) wfPath = descendWavefront(wf, cellAt(wf.grid, start[0], start[1]));
      return { descender, trail: [start], outcome: null, wfPath, wfIndex: 0, start, idle: 0 };
    },
    [U, alpha, goalVec, obstacles, wf],
  );

  const step = useCallback(
    (s: State): State => {
      if (wf) {
        if (s.wfIndex >= s.wfPath.length - 1) return s.idle < 30 ? { ...s, idle: s.idle + 1 } : init(Math.floor(Math.random() * 1e5));
        const i = Math.min(s.wfPath.length - 1, s.wfIndex + 1);
        const c = cellCenter(wf.grid, s.wfPath[i]);
        return { ...s, wfIndex: i, trail: [...s.trail, [c.x, c.y]] };
      }
      if (s.outcome) return s;
      let outcome: DescentOutcome | null = null;
      for (let k = 0; k < 3 && !outcome; k++) outcome = s.descender.step();
      return { ...s, trail: s.descender.path.slice(-900), outcome };
    },
    [wf, init],
  );

  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 71 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [init]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') {
        dragging.current = null;
        return;
      }
      if (phase === 'down') {
        if (Math.hypot(w[0] - goal.x, w[1] - goal.y) < 0.35) dragging.current = 'goal';
        else if (Math.abs(w[0] - center.x) < 0.9 && Math.abs(w[1] - center.y) < 1.0) dragging.current = 'obstacle';
        return;
      }
      const clamp = (v: number) => Math.max(-2.7, Math.min(2.7, v));
      if (dragging.current === 'goal') {
        const g = { x: clamp(w[0]), y: clamp(w[1]) };
        if (!obstacles.inside(g)) setGoal(g);
      } else if (dragging.current === 'obstacle') setCenter({ x: clamp(w[0]), y: clamp(w[1]) });
    },
    [goal, center, obstacles],
  );

  const s = sim.state;
  const q = s.trail[s.trail.length - 1];
  const grad = useMemo(() => U.gradient(q), [U, q]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      // Heightfield: U in ink tints, or the wave-front labels in graph tints.
      const key = wf ? `wf:${p.graph}:${wf.maxLabel}:${conn}:${goal.x},${goal.y}:${center.x},${center.y}:${set}` : `u:${p.ink}:${qStar}:${zeta}:${eta}:${goal.x},${goal.y}:${center.x},${center.y}:${set}:${convexPieces}`;
      if (!imgCache.current || imgCache.current.key !== key) {
        const img = wf
          ? fieldImage(wf.label, wf.grid.nx, wf.grid.ny, p.graph, (l) => (l <= 1 ? 0 : 0.08 + 0.55 * (1 - (l - 2) / Math.max(1, wf.maxLabel - 2))))
          : fieldImage(field.vals, FIELD_N, FIELD_N, p.ink, (u) => (u < 0 ? 0 : 0.04 + 0.5 * Math.log1p(u) / Math.log1p(field.max)));
        imgCache.current = { key, img };
      }
      if (imgCache.current.img) drawField(ctx, v, imgCache.current.img, BOUNDS, !wf);
      // Obstacles.
      ctx.save();
      ctx.fillStyle = p.obstacle;
      for (const o of obstacles.obstacles) {
        ctx.beginPath();
        if (o.kind === 'disc') ctx.arc(sx(v, o.c.x), sy(v, o.c.y), sl(v, o.r), 0, Math.PI * 2);
        else if (o.kind === 'polygon') o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
        ctx.closePath();
        ctx.fill();
      }
      // Q* ring around each disc, dashed: the domain of influence.
      ctx.setLineDash([3, 4]);
      ctx.strokeStyle = p.obstacle;
      ctx.globalAlpha = 0.5;
      for (const o of obstacles.obstacles) {
        if (o.kind !== 'disc') continue;
        ctx.beginPath();
        ctx.arc(sx(v, o.c.x), sy(v, o.c.y), sl(v, o.r + qStar), 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.restore();
      // Trail and bead.
      ctx.save();
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 2;
      ctx.globalAlpha = 0.85;
      ctx.beginPath();
      s.trail.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt[0]), sy(v, pt[1])) : ctx.lineTo(sx(v, pt[0]), sy(v, pt[1]))));
      ctx.stroke();
      ctx.restore();
      marker(ctx, sx(v, s.start[0]), sy(v, s.start[1]), 6, p.start, p.bg);
      marker(ctx, sx(v, goal.x), sy(v, goal.y), 7, p.goal, p.bg);
      marker(ctx, sx(v, q[0]), sy(v, q[1]), 6, p.robot, p.bg);
      if (!wf) {
        const gn = Math.hypot(grad[0], grad[1]) || 1;
        const L = Math.min(0.9, 0.25 * gn);
        arrow(ctx, v, q[0], q[1], (-grad[0] / gn) * L, (-grad[1] / gn) * L, p.robot, 1.6);
      }
      // Badge.
      const o = s.outcome;
      let text = wf ? `wave-front mode: ${s.wfPath.length ? `${s.wfPath.length - 1} grid steps to the goal` : 'start unreachable on the grid'}` : `rolling: ‖∇U‖ = ${fmt(Math.hypot(grad[0], grad[1]))}`;
      let color = p.ink;
      if (!wf && o) {
        if (o.kind === 'goal') text = `arrived at the goal after ${o.steps} steps`;
        else if (o.kind === 'localMinimum') {
          const signs = o.eigs.map((e) => (e > 1e-7 ? '+' : e < -1e-7 ? '−' : '0')).join(', ');
          text = `stuck: ‖∇U‖ = ${o.gradNorm.toExponential(1)} < ε, not at goal — Hessian eigenvalues (${signs}): a ${o.type}`;
          color = p.goal;
        } else if (o.kind === 'stalled') {
          text = `stalled: chattering with ‖∇U‖ = ${fmt(o.gradNorm)} — ∇d jumps where two closest points tie (nonconvex polygon)`;
          color = p.goal;
        } else if (o.kind === 'collision') {
          text = 'stepped into an obstacle (α too large for the repulsive spike)';
          color = p.goal;
        } else text = `step limit reached, ‖∇U‖ = ${fmt(o.gradNorm)}`;
      }
      label(ctx, text, 10, 14, color, { size: 10.5, weight: 600 });
      label(ctx, 'drag the goal or the obstacle', 10, v.height - 12, p.ink, { size: 9 });
    },
    [s, q, grad, obstacles, goal, center, set, convexPieces, qStar, zeta, eta, wf, conn, field],
  );

  return (
    <WidgetFrame
      id="w7.1"
      title="Potential Well"
      teaches="Local minima are a property of sums, not of concave obstacles — two convex discs suffice. A cost-to-go field has exactly one minimum."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'robot']}
      wide
      caption={
        <>
          The heightfield is U = U<sub>att</sub> + U<sub>rep</sub> (darker is higher); the orange bead rolls
          down −∇U at fixed α from a seeded green start. With the horseshoe it stops in the mouth,
          and the badge prints the Hessian signs at the stuck point: a genuine minimum, not a bug.
          Switch to the two convex discs — it still stops between them. Enable <em>wave-front
          mode</em>: the field becomes the BFS cost-to-goal of Chapter 6 (blue, lighter farther from
          the goal), and the bead escapes along a grid-shortest path. The Q* slider widens the
          repulsive domains of influence; too small and the bead clips a corner, too large and the
          mouth closes before the bead reaches it.
        </>
      }
    >
      <SimCanvas world={BOUNDS} draw={draw} deps={[sim.tick, s, goal, center, set, convexPieces, qStar, zeta, eta, waveMode, conn]} aspect={16 / 9} padding={0.2} onPointer={onPointer} ariaLabel="A square table with a slate horseshoe obstacle, a red goal and an orange bead rolling down a shaded potential heightfield; in wave-front mode the shading becomes blue grid cost-to-goal and the bead escapes." />
      <ControlPanel columns={3} title="Headline: Q*, the repulsive distance of influence">
        <Slider label="Q* — repulsive influence radius" role="obstacle" value={qStar} min={0.2} max={2} step={0.05} unit="m" onChange={setQStar} />
        <ButtonRow>
          <ActionButton emphasis={set === 'horseshoe'} onClick={() => setSet('horseshoe')}>horseshoe (Fig. 4.10)</ActionButton>
          <ActionButton emphasis={set === 'discs'} onClick={() => setSet('discs')}>two convex discs (Fig. 4.11)</ActionButton>
          <ActionButton emphasis={set === 'both'} onClick={() => setSet('both')}>both</ActionButton>
        </ButtonRow>
        <Toggle label="Wave-front mode (BFS cost-to-goal; Derivation 4)" role="graph" checked={waveMode} onChange={setWaveMode} />
      </ControlPanel>
      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Advanced: gains, step size, connectivity, the nonconvex polygon</summary>
        <ControlPanel columns={3}>
          <Slider label="ζ — attractive gain" role="goal" value={zeta} min={0.2} max={3} step={0.1} onChange={setZeta} />
          <Slider label="η — repulsive gain" role="obstacle" value={eta} min={0.1} max={3} step={0.1} onChange={setEta} />
          <Slider label="α — descent step" role="robot" value={alpha} min={0.002} max={0.04} step={0.002} format={(v) => v.toFixed(3)} onChange={setAlpha} />
          <ButtonRow>
            <ActionButton emphasis={conn === 4} onClick={() => setConn(4)}>4-point</ActionButton>
            <ActionButton emphasis={conn === 8} onClick={() => setConn(8)}>8-point</ActionButton>
          </ButtonRow>
          <Toggle label="Horseshoe as three convex pieces (per-obstacle U_rep; off = one nonconvex polygon, ∇d jumps)" role="obstacle" checked={convexPieces} onChange={setConvexPieces} />
        </ControlPanel>
      </details>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
