'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Hitch } from '@/lib/robots/hitch';
import { LOT } from '@/lib/robots/worlds';
import { Scene } from '@/lib/collide/scene';
import { FootprintChecker } from '@/lib/collide/collision';
import { Rng } from '@/lib/prob/rng';
import { normalizeAngle, type Pose2 } from '@/lib/geom/se2';
import { dynCarMetric, dynCarSystem, HITCH_DYNAMIC, KinoRrt, KinoRrtStar, reedsSheppSteer, turningRadius, type DynCar, type DynCarControl } from '@/lib/nonholonomic';
import { drawHitch, drawLot, drawPoseMarker, insetBox } from './draw-lot';

/**
 * w21.5 — Forward-Propagation Tree. Hitch's *dynamic* model (acceleration and
 * steering-rate inputs) grows a `KinoRrt` in the Lot: each extension samples
 * k controls, propagates each for Δt through the simulator, keeps the one
 * landing closest (under the weighted metric) to the random sample. No steering
 * function anywhere. Toggle Kinodynamic RRT* and the tree is rebuilt on the
 * reduced kinematic car with the exact Reeds–Shepp steer, rewiring as it goes —
 * the cost curve in the inset only ever descends.
 */

const WORLD = { minX: 0, minY: 0, maxX: 16, maxY: 10 };
const CAR = Hitch.car();
const SCENE = Scene.fromWorld(LOT);
const CHECK = new FootprintChecker(SCENE, CAR);
const START: Pose2 = { x: 5.5, y: 1.2, theta: 0 };
const GOAL: Pose2 = { x: 9.25, y: 6.4, theta: Math.PI / 2 };
const MAX_ITERS = 3000;

interface Params {
  k: number;
  dt: number;
  wTheta: number;
  star: boolean;
}

type Planner = { kind: 'rrt'; rrt: KinoRrt<DynCar, DynCarControl> } | { kind: 'star'; star: KinoRrtStar<Pose2>; edges: Map<number, Pose2[]> };

function makePlanner(params: Params, seed: number): Planner {
  const b = SCENE.bounds;
  const isFree = (s: Pose2) => CHECK.isFree({ body: s, trailer: null });
  if (params.star) {
    const star = new KinoRrtStar<Pose2>(
      { start: START, goal: GOAL, sample: (g) => ({ x: g.uniform(b.minX, b.maxX), y: g.uniform(b.minY, b.maxY), theta: g.uniform(-Math.PI, Math.PI) }), isFree },
      { steer: reedsSheppSteer(turningRadius(CAR)), eta: 5, gamma: 24, dim: 3, goalBias: 0.05, ds: 0.25 },
      new Rng(seed),
    );
    return { kind: 'star', star, edges: new Map() };
  }
  const rrt = new KinoRrt<DynCar, DynCarControl>(
    dynCarSystem(HITCH_DYNAMIC),
    {
      start: { ...START, v: 0, phi: 0 },
      isGoal: (s) => Math.hypot(s.x - GOAL.x, s.y - GOAL.y) < 0.8 && Math.abs(normalizeAngle(s.theta - GOAL.theta)) < 0.35,
      sample: (g) => ({ x: g.uniform(b.minX, b.maxX), y: g.uniform(b.minY, b.maxY), theta: g.uniform(-Math.PI, Math.PI), v: g.uniform(-2, 2), phi: 0 }),
      goalSample: () => ({ ...GOAL, v: 0, phi: 0 }),
      metric: dynCarMetric(params.wTheta, 0.3),
      isFree,
    },
    { controlsPerExtend: params.k, dt: params.dt, substeps: 4, goalBias: 0.15 },
    new Rng(seed),
  );
  return { kind: 'rrt', rrt };
}

export function ForwardPropagationTree() {
  const [params, setParams] = useState<Params>({ k: 8, dt: 0.8, wTheta: 1.0, star: false });
  const planner = useRef<Planner | null>(null);
  const init = useCallback(
    (seed: number) => {
      planner.current = makePlanner(params, seed);
      return { n: 0 };
    },
    [params],
  );
  const step = useCallback((s: { n: number }) => {
    const pl = planner.current;
    if (!pl) return s;
    if (pl.kind === 'rrt') {
      // Keep extending after a solution only briefly: the point is the first one.
      for (let i = 0; i < 12 && pl.rrt.iterations < MAX_ITERS && !pl.rrt.solved; i++) pl.rrt.extend();
    } else {
      for (let i = 0; i < 2 && pl.star.iterations < 900; i++) pl.star.iterate();
    }
    return { n: s.n + 1 };
  }, []);
  const sim = useSimulation({ init, step, fps: 30, initialSeed: 21 });
  // Rebuild when a parameter changes.
  const key = useMemo(() => JSON.stringify(params), [params]);
  const lastKey = useRef(key);
  if (lastKey.current !== key) {
    lastKey.current = key;
    planner.current = makePlanner(params, sim.seed);
  }

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawLot(ctx, v, LOT, p);
      drawPoseMarker(ctx, v, START, p.start, 1.0);
      drawPoseMarker(ctx, v, GOAL, p.goal, 1.0);
      const pl = planner.current;
      if (!pl) return;
      if (pl.kind === 'rrt') {
        const r = pl.rrt;
        ctx.save();
        ctx.lineWidth = 1;
        for (let i = 1; i < r.nodes.length; i++) {
          const n = r.nodes[i];
          const par = r.nodes[n.parent].s;
          const sp = Math.abs(n.s.v) / HITCH_DYNAMIC.maxSpeed;
          ctx.strokeStyle = n.s.v >= 0 ? p.graph : p.accent;
          ctx.globalAlpha = 0.25 + 0.6 * sp;
          ctx.beginPath();
          ctx.moveTo(sx(v, par.x), sy(v, par.y));
          for (const q of n.seg) ctx.lineTo(sx(v, q.x), sy(v, q.y));
          ctx.stroke();
        }
        ctx.restore();
        const path = r.densePath();
        if (path) {
          ctx.save();
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 3;
          ctx.beginPath();
          path.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
          ctx.stroke();
          ctx.restore();
          const end = path[path.length - 1];
          drawHitch(ctx, v, CAR, { body: end, trailer: null }, p.robot, { fill: true });
        }
        label(ctx, `forward propagation: best of ${params.k} sampled control${params.k > 1 ? 's' : ''}, Δt = ${params.dt.toFixed(2)} s`, 10, 14, p.ink, { size: 11, weight: 600 });
        label(ctx, `${r.nodes.length} nodes   ${r.propagations} propagations (${r.wasted} collided)   ${r.solved ? `solved at extension ${r.solvedAt}` : r.iterations >= MAX_ITERS ? 'budget spent' : `extension ${r.iterations}`}`, 10, 30, r.solved ? p.path : p.ink, { size: 10 });
        label(ctx, 'blue: forward edges   teal: reversing (opacity = speed)', 10, v.height - 12, p.ink, { size: 9 });
      } else {
        const st = pl.star;
        const steer = st.opts.steer;
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.globalAlpha = 0.4;
        ctx.lineWidth = 1;
        for (let i = 1; i < st.nodes.length; i++) {
          const n = st.nodes[i];
          // Edge cache keyed by child and parent: a rewire changes the parent and so the key.
          const k = i * 100000 + n.parent;
          let e = pl.edges.get(k);
          if (!e) {
            e = steer.trajectory(st.nodes[n.parent].s, n.s, 0.3);
            pl.edges.set(k, e);
          }
          ctx.beginPath();
          e.forEach((q, j) => (j === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
          ctx.stroke();
        }
        ctx.restore();
        const path = st.bestPath();
        if (path) {
          ctx.save();
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 3;
          ctx.beginPath();
          path.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
          ctx.stroke();
          ctx.restore();
        }
        label(ctx, 'Kinodynamic RRT*: exact Reeds–Shepp steer on the reduced car', 10, 14, p.ink, { size: 11, weight: 600 });
        label(ctx, `${st.nodes.length} nodes   ${st.rewires} rewires   best ${Number.isFinite(st.bestCost) ? st.bestCost.toFixed(2) + ' m' : '—'}`, 10, 30, st.bestNode !== null ? p.path : p.ink, { size: 10 });
        // Cost curve inset.
        const hist = st.costHistory;
        const ix = v.width - 200;
        const iy = v.height - 120;
        insetBox(ctx, ix, iy, 186, 106, p);
        label(ctx, 'best cost vs iteration', ix + 6, iy + 9, p.ink, { size: 9 });
        if (hist.length) {
          const cmax = hist[0].cost;
          const cmin = Math.min(...hist.map((h) => h.cost)) * 0.97;
          const X = (n: number) => ix + 10 + (n / Math.max(1, st.iterations)) * 166;
          const Y = (c: number) => iy + 96 - ((c - cmin) / Math.max(1e-9, cmax - cmin)) * 76;
          ctx.save();
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 2;
          ctx.beginPath();
          hist.forEach((h, i) => {
            if (i === 0) ctx.moveTo(X(h.n), Y(h.cost));
            else {
              ctx.lineTo(X(h.n), Y(hist[i - 1].cost));
              ctx.lineTo(X(h.n), Y(h.cost));
            }
          });
          ctx.lineTo(X(st.iterations), Y(hist[hist.length - 1].cost));
          ctx.stroke();
          ctx.restore();
          label(ctx, cmax.toFixed(1), ix + 160, iy + 18, p.ink, { size: 8 });
          label(ctx, hist[hist.length - 1].cost.toFixed(1), ix + 160, iy + 96, p.path, { size: 8 });
        }
      }
    },
    [params],
  );

  return (
    <WidgetFrame
      id="w21.5"
      title="Forward-Propagation Tree"
      teaches="An RRT does not need a steering function: a simulator and a metric are enough for completeness — but optimality (RRT*) needs the exact steer."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Each extension samples k controls (acceleration, steering rate), runs Hitch&apos;s dynamic model forward for Δt, and keeps the
          endpoint nearest the random sample under the metric √(Δx² + Δy² + (w_θΔθ)² + (0.3Δv)²). With k = 1 the tree wanders; with
          k = 8 it reaches. Drag w_θ to 3 and the tree stops exploring position to chase headings — the metric sensitivity Choset
          warns about. Toggle RRT*: rewiring needs the exact two-point optimal steer, which this widget has only for the kinematic
          car (Reeds–Shepp), so the tree switches models and its best cost descends.
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, params]} aspect={1.6} padding={0.2} ariaLabel="A forward-propagated tree of Hitch's dynamic model in the Lot, or a Kinodynamic RRT* with Reeds–Shepp edges and its cost curve." />
      <ControlPanel columns={3}>
        <Slider label="controls sampled per extension k" value={params.k} min={1} max={8} step={1} onChange={(k) => setParams((q) => ({ ...q, k }))} role="graph" />
        <div className="flex flex-col gap-1.5">
          <Slider label="Δt per propagation" value={params.dt} min={0.3} max={1.5} step={0.05} unit="s" onChange={(dt) => setParams((q) => ({ ...q, dt }))} format={(x) => x.toFixed(2)} />
          <Slider label="metric heading weight w_θ" value={params.wTheta} min={0.1} max={3} step={0.1} onChange={(wTheta) => setParams((q) => ({ ...q, wTheta }))} format={(x) => x.toFixed(1)} />
        </div>
        <Toggle label="Kinodynamic RRT* (exact RS steer)" checked={params.star} onChange={(star) => setParams((q) => ({ ...q, star }))} role="path" />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} tick={sim.tick} seed={sim.seed} />
    </WidgetFrame>
  );
}
