'use client';

import { useCallback, useMemo } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { WORKBENCH } from '@/lib/robots/worlds';
import {
  RrtStar,
  StraightLine,
  TORUS_GOAL,
  TORUS_START,
  gammaStarSolovey2020,
  paintArmMapped,
  paintPolyWorld,
  rectMapper,
  torusLab,
  type TorusLab,
} from '@/lib/sampling';

/**
 * The Chapter 13 hook: Reach executes its Chapter 12 RRT path, needless
 * reversals and all; ten thousand more iterations do not change it. Beside it,
 * RRT* on the very same sample stream, whose path is the short way round.
 * Both planners are `RrtStar` — one with `rewire: false`, which is RRT.
 */

const ARENA = { minX: 0, minY: 0, maxX: 2.12, maxY: 1 };
const LEFT = { x0: 0, y0: 0, x1: 1, y1: 1 };
const RIGHT = { x0: 1.12, y0: 0, x1: 2.12, y1: 1 };
const ITERS = 2500;

interface Lane {
  lab: TorusLab;
  planner: RrtStar<number[]>;
  path: number[][] | null;
  seg: number[];
  L: number;
  costAtFirst: number;
}

interface State {
  lanes: [Lane, Lane];
  t: number;
  hold: number;
}

function makeLane(seed: number, rewire: boolean): Lane {
  const lab = torusLab();
  const planner = new RrtStar<number[]>(
    lab.space,
    lab.free,
    [...TORUS_START],
    [...TORUS_GOAL],
    { steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.04 }), eta: 0.4, goalBias: 0.05, goalRadius: 0.4, nbhd: { kind: 'radius', gamma: gammaStarSolovey2020(2, lab.muBox * 0.7) * 1.1 }, rewire },
    new Rng(seed),
  );
  let costAtFirst = Infinity;
  for (let i = 0; i < ITERS; i++) {
    planner.iterate();
    if (!Number.isFinite(costAtFirst) && planner.solved) costAtFirst = planner.bestCost;
  }
  const path = planner.bestPath();
  const seg = [0];
  let L = 0;
  if (path) for (let i = 1; i < path.length; i++) seg.push((L += lab.space.dist(path[i - 1], path[i])));
  return { lab, planner, path, seg, L, costAtFirst };
}

function init(seed: number): State {
  return { lanes: [makeLane(seed, false), makeLane(seed, true)], t: 0, hold: 0 };
}

function step(s: State): State {
  const L = Math.max(s.lanes[0].L, s.lanes[1].L);
  if (s.t >= L) {
    if (++s.hold > 40) {
      s.t = 0;
      s.hold = 0;
    }
    return { ...s };
  }
  s.t = Math.min(L, s.t + 0.03);
  return { ...s };
}

function configAt(lane: Lane, t: number): number[] {
  if (!lane.path) return TORUS_START;
  if (t >= lane.L) return lane.path[lane.path.length - 1];
  let i = 1;
  while (i < lane.seg.length - 1 && lane.seg[i] < t) i++;
  return lane.lab.space.interpolate(lane.path[i - 1], lane.path[i], (t - lane.seg[i - 1]) / Math.max(1e-9, lane.seg[i] - lane.seg[i - 1]));
}

export function WobbleHook() {
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 13 });
  const s = sim.state;
  const titles = useMemo(() => ['RRT — the first path, and the last', 'RRT* — same samples, allowed to change its mind'], []);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      s.lanes.forEach((lane, k) => {
        const m = rectMapper(v, k === 0 ? LEFT : RIGHT, WORKBENCH.bounds);
        paintPolyWorld(ctx, m, WORKBENCH, p);
        paintArmMapped(ctx, m, p, lane.lab.arm, TORUS_START, p.start, 0.35);
        paintArmMapped(ctx, m, p, lane.lab.arm, TORUS_GOAL, p.goal, 0.35);
        // The tip's trace so far.
        if (lane.path) {
          ctx.save();
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 1.5;
          ctx.globalAlpha = 0.85;
          ctx.beginPath();
          let first = true;
          for (let tt = 0; tt <= Math.min(s.t, lane.L); tt += 0.02) {
            const pts = lane.lab.arm.fk(configAt(lane, tt));
            const tip = pts[pts.length - 1];
            if (first) ctx.moveTo(m.X(tip.x), m.Y(tip.y));
            else ctx.lineTo(m.X(tip.x), m.Y(tip.y));
            first = false;
          }
          ctx.stroke();
          ctx.restore();
        }
        paintArmMapped(ctx, m, p, lane.lab.arm, configAt(lane, s.t), p.robot, 1);
        label(ctx, titles[k], m.X(WORKBENCH.bounds.minX), m.Y(WORKBENCH.bounds.maxY) - 8, p.ink, { size: 10, weight: 600 });
        const c = lane.planner.bestCost;
        label(
          ctx,
          Number.isFinite(c) ? `${ITERS} iterations · first path ${lane.costAtFirst.toFixed(2)} rad · now ${c.toFixed(2)} rad · ${lane.planner.rewires} rewires` : 'no path found — re-roll',
          m.X(WORKBENCH.bounds.minX),
          m.Y(WORKBENCH.bounds.minY) + 12,
          Number.isFinite(c) ? p.path : p.goal,
          { size: 10, weight: 600 },
        );
      });
    },
    [s, titles],
  );

  return (
    <WidgetFrame
      id="w13.0"
      title="Reach wobbles, then doesn't"
      teaches="Feasibility and quality are separate contracts: a tree planner's first path is an accident of sample order, and more sampling leaves it alone unless the planner is allowed to rewire."
      colorKey={['start', 'goal', 'obstacle', 'path', 'robot']}
      wide
      caption={
        <>
          Left: Reach executes the path a Chapter 12 RRT found after {ITERS} iterations on the
          Workbench — the purple trace is the fingertip, reversals and all. Its cost is the cost of the
          first path it found; the iterations after that changed nothing but the tree&rsquo;s size.
          Right: RRT* fed the <em>same</em> random samples in the same order. Every one of its rewires
          shortened the incumbent, and the arm goes the short way. Same world, same samples, same
          goal; one flag.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.12} padding={0.08} draw={draw} deps={[sim.tick, s]} ariaLabel="Two copies of the Workbench; a two-link arm traces a jagged fingertip path on the left and a smooth short one on the right." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
