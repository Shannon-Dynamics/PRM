'use client';

import { useCallback, useMemo } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { BitGrid } from '@/lib/manifold/torus';
import {
  Prm,
  SLOT_TARGET,
  StraightLine,
  TORUS_START,
  Uniform,
  WORKBENCH_SLOT,
  chartPanel,
  paintArmMapped,
  paintChartPath,
  paintDot,
  paintGeodesic,
  paintPanelFrame,
  paintPolyWorld,
  paintRaster,
  rasterTorusCObstacle,
  rectMapper,
  torusLab,
  type TorusLab,
} from '@/lib/sampling';

/**
 * The Chapter 11 hook: Reach must put its forearm through a slot between two
 * bars on the Workbench. Chapter 8's visibility graph has no vertices to use
 * and Chapter 10's decomposition of the torus runs to thousands of cells; six
 * hundred random samples and a straight-line local planner solve it, and the
 * arm replays the roadmap path. Everything here is Chapter 11's `Prm` — the
 * same object w11.1 grows one node at a time.
 */

const ARENA = { minX: 0, minY: 0, maxX: 2.12, maxY: 1 };
const BENCH = { x0: 0, y0: 0, x1: 1, y1: 1 };
const TORUS = { x0: 1.12, y0: 0, x1: 2.12, y1: 1 };
const N = 600;
const K = 6;

/** Two-link inverse kinematics for the point (x, y): both elbow solutions. */
function ik2(x: number, y: number, l1: number, l2: number): number[][] {
  const r2 = x * x + y * y;
  const c2 = (r2 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
  if (Math.abs(c2) > 1) return [];
  const out: number[][] = [];
  for (const sgn of [1, -1]) {
    const t2 = sgn * Math.acos(c2);
    const t1 = Math.atan2(y, x) - Math.atan2(l2 * Math.sin(t2), l1 + l2 * Math.cos(t2));
    out.push([t1, t2]);
  }
  return out;
}

interface State {
  lab: TorusLab;
  prm: Prm<number[]>;
  goal: number[];
  path: number[][] | null;
  nUsed: number;
  seg: number[];
  L: number;
  t: number;
  hold: number;
}

function init(seed: number): State {
  const lab = torusLab(WORKBENCH_SLOT);
  // The goal: the forearm's tip inside the slot, whichever elbow solution is collision-free.
  const goal = ik2(SLOT_TARGET[0], SLOT_TARGET[1], lab.arm.lengths[0], lab.arm.lengths[1]).find((q) => lab.free.isFree(q)) ?? [-0.5, 0.75];
  const prm = new Prm<number[]>(lab.space, lab.free, { sampler: new Uniform<number[]>(), steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.2 }), connect: { kind: 'kNearest', k: K }, nn: 'kd' }, new Rng(seed));
  const a = prm.addNode([...TORUS_START]);
  prm.connectNode(a, prm.candidates(a));
  const b = prm.addNode(goal);
  prm.connectNode(b, prm.candidates(b));
  // Six hundred samples usually suffice; a stubborn seed gets a few hundred more rather than a blank screen.
  let nUsed = N;
  prm.grow(N + 2);
  while (!prm.roadmap.sameComponent(0, 1) && nUsed < 2000) {
    nUsed += 200;
    prm.grow(nUsed + 2);
  }
  let path: number[][] | null = null;
  if (prm.roadmap.sameComponent(0, 1)) {
    const sp = prm.roadmap.shortestPath(0, 1);
    if (sp) path = prm.roadmap.pathPoints(sp.path);
  }
  const seg = [0];
  let L = 0;
  if (path) {
    for (let i = 1; i < path.length; i++) {
      L += lab.space.dist(path[i - 1], path[i]);
      seg.push(L);
    }
  }
  return { lab, prm, goal, path, nUsed, seg, L, t: 0, hold: 0 };
}

function step(s: State): State {
  if (!s.path) return s;
  if (s.t >= s.L) {
    s.hold++;
    if (s.hold > 45) {
      s.t = 0;
      s.hold = 0;
    }
    return { ...s };
  }
  s.t = Math.min(s.L, s.t + 0.035);
  return { ...s };
}

export function SlotHook() {
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 6 });
  const s = sim.state;
  const raster = useMemo<BitGrid>(() => rasterTorusCObstacle(torusLab(WORKBENCH_SLOT), 72), []);

  const current = useMemo(() => {
    if (!s.path) return TORUS_START;
    let i = 1;
    while (i < s.seg.length - 1 && s.seg[i] < s.t) i++;
    return s.t >= s.L ? s.path[s.path.length - 1] : s.lab.space.interpolate(s.path[i - 1], s.path[i], (s.t - s.seg[i - 1]) / Math.max(1e-9, s.seg[i] - s.seg[i - 1]));
  }, [s]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const space = s.lab.space;
      // Left: the Workbench with the slot.
      const m = rectMapper(v, BENCH, WORKBENCH_SLOT.bounds);
      paintPolyWorld(ctx, m, WORKBENCH_SLOT, p);
      paintArmMapped(ctx, m, p, s.lab.arm, TORUS_START, p.start, 0.45);
      paintArmMapped(ctx, m, p, s.lab.arm, s.goal, p.goal, 0.45);
      // The tip's trace so far.
      if (s.path) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 1.5;
        ctx.globalAlpha = 0.8;
        ctx.beginPath();
        let first = true;
        for (let tt = 0; tt <= s.t; tt += 0.03) {
          let i = 1;
          while (i < s.seg.length - 1 && s.seg[i] < tt) i++;
          const q = space.interpolate(s.path[i - 1], s.path[i], (tt - s.seg[i - 1]) / Math.max(1e-9, s.seg[i] - s.seg[i - 1]));
          const pts = s.lab.arm.fk(q);
          const tip = pts[pts.length - 1];
          if (first) ctx.moveTo(m.X(tip.x), m.Y(tip.y));
          else ctx.lineTo(m.X(tip.x), m.Y(tip.y));
          first = false;
        }
        ctx.stroke();
        ctx.restore();
      }
      paintArmMapped(ctx, m, p, s.lab.arm, current, p.robot, 1);
      label(ctx, 'Workbench with a slot', m.X(WORKBENCH_SLOT.bounds.minX) + 4, m.Y(WORKBENCH_SLOT.bounds.maxY) + 10, p.ink, { size: 10 });

      // Right: the torus with the C-obstacle raster, the roadmap and the path.
      const panel = chartPanel(v, space, TORUS);
      paintRaster(ctx, panel, raster, p.cobstacle, 0.75);
      paintPanelFrame(ctx, panel, p, true);
      const nodes = s.prm.roadmap.nodes;
      const adj = s.prm.roadmap.adj;
      ctx.save();
      ctx.strokeStyle = p.graph;
      ctx.globalAlpha = 0.3;
      ctx.lineWidth = 0.8;
      ctx.beginPath();
      for (let a = 0; a < nodes.length; a++) for (const e of adj[a]) if (e.to > a) paintGeodesic(ctx, panel, space, nodes[a], nodes[e.to], 4);
      ctx.stroke();
      ctx.restore();
      for (let i = 2; i < nodes.length; i++) {
        const c = space.coords(nodes[i]);
        paintDot(ctx, panel.X(c[0]), panel.Y(c[1]), 1.6, p.graph, 0.8);
      }
      if (s.path) paintChartPath(ctx, panel, space, s.path, p.path, 3);
      const cs = space.coords(TORUS_START);
      const cg = space.coords(s.goal);
      const cc = space.coords(current);
      paintDot(ctx, panel.X(cs[0]), panel.Y(cs[1]), 6, p.start);
      paintDot(ctx, panel.X(cg[0]), panel.Y(cg[1]), 6, p.goal);
      paintDot(ctx, panel.X(cc[0]), panel.Y(cc[1]), 5, p.robot);
      label(ctx, s.path ? `${s.nUsed} samples, k = ${K}: ${s.prm.roadmap.components()} components, path of ${s.path.length - 1} edges` : `${s.nUsed} samples: start and goal not yet joined — re-roll`, panel.px + panel.w, panel.py + panel.h + 12, s.path ? p.path : p.goal, { size: 10, align: 'right', weight: 600 });
    },
    [s, current, raster],
  );

  return (
    <WidgetFrame
      id="w11.0"
      title="Reach through the slot"
      teaches="Explicit geometry fails where sampling succeeds: six hundred random configurations and a straight-line local planner thread an arm through a slot that no visibility vertex or cell boundary describes."
      colorKey={['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Reach (orange) must bring its forearm from the home pose (green) to a pose inside the slot
          between two bars (red). On the right, the same problem on the unwrapped torus: the amber
          cells are configurations in collision, sampled from the Chapter 2 checker; the blue dots are{' '}
          {s.nUsed} uniform samples that survived it, joined to their {K} nearest neighbors by
          straight-line local paths; the purple polyline is Dijkstra&rsquo;s path through the roadmap,
          which the arm replays. Nobody wrote down the free space. The planner only asked{' '}
          {s.lab.free.checks.toLocaleString()} times whether a configuration was free.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.12} padding={0.06} draw={draw} deps={[sim.tick, s]} ariaLabel="A two-link arm threading a slot between two bars, beside the torus chart of its configuration space with a roadmap and the found path." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
