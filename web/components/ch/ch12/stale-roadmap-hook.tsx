'use client';

import { useCallback, useMemo } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { WORKBENCH, type PolyWorld } from '@/lib/robots/worlds';
import { rect } from '@/lib/collide/polygon';
import {
  Prm,
  Rrt,
  StraightLine,
  TORUS_GOAL,
  TORUS_START,
  Uniform,
  chartPanel,
  paintChartPath,
  paintDot,
  paintGeodesic,
  paintPanelFrame,
  paintRaster,
  paintTreeEdges,
  rasterTorusCObstacle,
  torusLab,
  type TorusLab,
} from '@/lib/sampling';

/**
 * The Chapter 12 hook: a Chapter 11 roadmap for Reach, then the block on the
 * Workbench is moved. Every roadmap edge is re-checked against the new world;
 * the ones now in collision turn red, and the old query path runs straight
 * through them. Beside it, an RRT answers the single query on the moved world
 * from scratch — a few hundred iterations and a fraction of the roadmap's
 * collision-check bill.
 */

/** The Workbench with the block slid 0.45 m toward the base. */
export const WORKBENCH_MOVED: PolyWorld = {
  ...WORKBENCH,
  name: 'Workbench (block moved)',
  obstacles: WORKBENCH.obstacles.map((o) => (o.name === 'the block' ? { name: o.name, poly: rect(0.05, 0.95, 0.55, 1.45) } : o)),
};

const ARENA = { minX: 0, minY: 0, maxX: 2.12, maxY: 1 };
const LEFT = { x0: 0, y0: 0, x1: 1, y1: 1 };
const RIGHT = { x0: 1.12, y0: 0, x1: 2.12, y1: 1 };
const N = 500;
const K = 6;

interface State {
  oldLab: TorusLab;
  newLab: TorusLab;
  prm: Prm<number[]>;
  oldPath: number[][] | null;
  staleEdges: Set<string>;
  buildChecks: number;
  rrt: Rrt<number[]>;
}

function init(seed: number): State {
  const oldLab = torusLab(WORKBENCH);
  const newLab = torusLab(WORKBENCH_MOVED);
  const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.1 });
  const prm = new Prm<number[]>(oldLab.space, oldLab.free, { sampler: new Uniform<number[]>(), steer, connect: { kind: 'kNearest', k: K }, nn: 'kd' }, new Rng(seed));
  const a = prm.addNode([...TORUS_START]);
  prm.connectNode(a, prm.candidates(a));
  const b = prm.addNode([...TORUS_GOAL]);
  prm.connectNode(b, prm.candidates(b));
  prm.grow(N + 2);
  const buildChecks = oldLab.free.checks;
  const sp = prm.roadmap.sameComponent(0, 1) ? prm.roadmap.shortestPath(0, 1) : null;
  const oldPath = sp ? prm.roadmap.pathPoints(sp.path) : null;
  // Re-check every edge against the moved world: which ones are stale?
  const recheck = new StraightLine<number[]>({ kind: 'subdivision', step: 0.1 });
  const staleEdges = new Set<string>();
  const nodes = prm.roadmap.nodes;
  for (const e of prm.roadmap.edges()) {
    if (!recheck.steer(newLab.space, newLab.free, nodes[e.a], nodes[e.b])) staleEdges.add(`${e.a}-${e.b}`);
  }
  newLab.free.reset();
  const rrt = new Rrt(newLab.space, newLab.free, [...TORUS_START], [...TORUS_GOAL], { steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), eta: 0.4, goalBias: 0.05, goalRadius: 0.4 }, new Rng(seed + 1));
  return { oldLab, newLab, prm, oldPath, staleEdges, buildChecks, rrt };
}

function step(s: State): State {
  if (!s.rrt.solved && s.rrt.iterations < 6000) for (let k = 0; k < 3 && !s.rrt.solved; k++) s.rrt.extendOnce();
  return { ...s };
}

export function StaleRoadmapHook() {
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 12 });
  const s = sim.state;
  const rasterOld = useMemo(() => rasterTorusCObstacle(torusLab(WORKBENCH), 72), []);
  const rasterNew = useMemo(() => rasterTorusCObstacle(torusLab(WORKBENCH_MOVED), 72), []);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const space = s.oldLab.space;
      // Left: the old roadmap over the NEW C-obstacle, stale edges in red.
      const L = chartPanel(v, space, LEFT);
      paintRaster(ctx, L, rasterOld, p.cobstacle, 0.25);
      paintRaster(ctx, L, rasterNew, p.cobstacle, 0.8);
      paintPanelFrame(ctx, L, p, true);
      const nodes = s.prm.roadmap.nodes;
      for (const stale of [false, true]) {
        ctx.save();
        ctx.strokeStyle = stale ? p.goal : p.graph;
        ctx.lineWidth = stale ? 1.4 : 0.8;
        ctx.globalAlpha = stale ? 0.9 : 0.35;
        ctx.beginPath();
        for (const e of s.prm.roadmap.edges()) {
          if (s.staleEdges.has(`${e.a}-${e.b}`) !== stale) continue;
          paintGeodesic(ctx, L, space, nodes[e.a], nodes[e.b], 4);
        }
        ctx.stroke();
        ctx.restore();
      }
      if (s.oldPath) paintChartPath(ctx, L, space, s.oldPath, p.path, 2.5, 0.9);
      const cs = space.coords(TORUS_START);
      const cg = space.coords(TORUS_GOAL);
      paintDot(ctx, L.X(cs[0]), L.Y(cs[1]), 6, p.start);
      paintDot(ctx, L.X(cg[0]), L.Y(cg[1]), 6, p.goal);
      label(ctx, `Chapter 11 roadmap: ${N} nodes, ${s.prm.roadmap.edgeCount} edges, ${s.buildChecks.toLocaleString()} checks to build`, L.px, L.py - 8, p.ink, { size: 10, weight: 600 });
      label(ctx, `block moved → ${s.staleEdges.size} edges now in collision; the old path crosses them`, L.px, L.py + L.h + 12, p.goal, { size: 10, weight: 600 });

      // Right: a fresh RRT on the moved world.
      const R = chartPanel(v, space, RIGHT);
      paintRaster(ctx, R, rasterNew, p.cobstacle, 0.8);
      paintPanelFrame(ctx, R, p, true);
      paintTreeEdges(ctx, R, space, s.rrt.tree.nodes, p.graph, { lineWidth: 1, alpha: 0.85 });
      const path = s.rrt.path();
      if (path) paintChartPath(ctx, R, space, path, p.path, 3);
      paintDot(ctx, R.X(cs[0]), R.Y(cs[1]), 6, p.start);
      paintDot(ctx, R.X(cg[0]), R.Y(cg[1]), 6, p.goal);
      label(ctx, `fresh RRT on the moved world: ${s.rrt.iterations} iterations, ${s.rrt.tree.size} nodes`, R.px, R.py - 8, p.ink, { size: 10, weight: 600 });
      label(ctx, s.rrt.solved ? `solved at iteration ${s.rrt.solvedAt} for ${s.newLab.free.checks.toLocaleString()} checks` : `${s.newLab.free.checks.toLocaleString()} checks so far…`, R.px, R.py + R.h + 12, s.rrt.solved ? p.path : p.ink, { size: 10, weight: 600 });
    },
    [s, rasterOld, rasterNew],
  );

  return (
    <WidgetFrame
      id="w12.0"
      title="Stale roadmap, fresh tree"
      teaches="A roadmap answers many queries fast and one changed obstacle for nothing: a tree spends its budget on the one query it was asked, from scratch, and is done when start and goal meet."
      colorKey={['start', 'goal', 'cobstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Left: the Chapter 11 roadmap for Reach on the Workbench, drawn over the C-obstacle{' '}
          <em>after</em> the block is slid toward the base (the faint amber is where it used to be).
          Every edge was re-checked against the new world; the red ones are now in collision, and the
          purple path the roadmap would still confidently return runs through them. Nothing in the
          roadmap knows which edges died — it would have to re-check all of them. Right: an RRT
          grown on the moved world for this one query, stopping the instant it reaches the goal, for a
          fraction of the roadmap&rsquo;s collision-check bill.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.12} padding={0.08} draw={draw} deps={[sim.tick, s]} ariaLabel="Left, a stale roadmap on the torus with edges that now collide drawn in red; right, a fresh random tree solving the same query on the changed world." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
