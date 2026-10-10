'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawWorld, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import {
  APARTMENT_GOAL,
  APARTMENT_START,
  InformedRrtStar,
  InformedSampler,
  RrtStar,
  StraightLine,
  discLab,
  gammaStarSolovey2020,
  paintDot,
  paintEllipse,
  type DiscLab,
} from '@/lib/sampling';

/**
 * w13.2 — Ellipse Shrinker.
 *
 * Informed RRT* on the Apartment. After the first solution the only
 * configurations that can improve the path lie inside the prolate
 * hyperspheroid with foci q_start, q_goal and transverse diameter c_best — in
 * the plane, an ellipse — and the sampler draws from it directly. The headline
 * scrubs iterations after the first solution; the readout is μ(X_f̂ ∩ 𝒬)/μ(𝒬),
 * the share of the room still worth sampling. With informed sampling off, the
 * planner is plain RRT* and every sample that landed outside the ellipse is
 * drawn faintly as wasted.
 */

const ITERS_PER_TICK = 6;
const ETA = 0.8;

interface Sample {
  q: number[];
  inside: boolean;
}

interface State {
  lab: DiscLab;
  planner: RrtStar<number[]>;
  readout: InformedSampler<number[]>;
  samples: Sample[];
  start: number[];
  goal: number[];
}

function makePlanner(seed: number, start: number[], goal: number[], informed: boolean): State {
  const lab = discLab();
  const opts = {
    steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }),
    eta: ETA,
    goalBias: 0.05,
    goalRadius: ETA,
    nbhd: { kind: 'radius' as const, gamma: gammaStarSolovey2020(2, lab.muBox * 0.8) * 1.1 },
  };
  const planner = informed ? new InformedRrtStar<number[]>(lab.space, lab.free, [...start], [...goal], opts, new Rng(seed)) : new RrtStar<number[]>(lab.space, lab.free, [...start], [...goal], opts, new Rng(seed));
  return { lab, planner, readout: new InformedSampler(lab.space, [...start], [...goal]), samples: [], start, goal };
}

/** Run until `iterations` reaches `target`, recording post-solution samples. */
function advance(s: State, target: number, max: number): void {
  const p = s.planner;
  let n = 0;
  while (p.iterations < target && n++ < max) {
    const it = p.iterate();
    if (p.solved && it.qRand) {
      s.readout.update(p.bestCost);
      if (s.samples.length < 4000) s.samples.push({ q: it.qRand, inside: s.readout.contains(it.qRand) });
    }
  }
  if (p.solved) s.readout.update(p.bestCost);
}

export function EllipseShrinker() {
  const [budget, setBudget] = useState(1500);
  const [informed, setInformed] = useState(true);
  const [start, setStart] = useState<number[]>(APARTMENT_START);
  const [goal, setGoal] = useState<number[]>(APARTMENT_GOAL);
  const drag = useRef<'start' | 'goal' | null>(null);
  const budgetRef = useRef(budget);
  budgetRef.current = budget;

  const init = useCallback((seed: number): State => makePlanner(seed, start, goal, informed), [start, goal, informed]);

  const stepFn = useCallback((s: State): State => {
    const p = s.planner;
    const target = p.solved ? (p.solvedAt ?? 0) + budgetRef.current : p.iterations + ITERS_PER_TICK;
    if (p.iterations < target) advance(s, Math.min(target, p.iterations + ITERS_PER_TICK), ITERS_PER_TICK + 1);
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 13 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [start, goal, informed]);

  // Scrubbing backward replays: the run is deterministic in the seed, so rebuild and fast-forward.
  const s = sim.state;
  const seedRef = useRef(sim.seed);
  seedRef.current = sim.seed;
  useEffect(() => {
    const p = s.planner;
    if (p.solved && p.iterations > (p.solvedAt ?? 0) + budget) {
      sim.setState(() => {
        const fresh = makePlanner(seedRef.current, start, goal, informed);
        advance(fresh, 1, 1);
        while (!fresh.planner.solved && fresh.planner.iterations < 20000) advance(fresh, fresh.planner.iterations + 50, 50);
        advance(fresh, (fresh.planner.solvedAt ?? 0) + budget, budget + 1);
        return fresh;
      });
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [budget]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const near = (q: number[]) => Math.hypot(q[0] - w[0], q[1] - w[1]) < 0.4;
      if (phase === 'down') {
        if (near(start)) drag.current = 'start';
        else if (near(goal)) drag.current = 'goal';
      } else if (phase === 'up') {
        if (drag.current && s.lab.free.isFree([w[0], w[1]])) {
          if (drag.current === 'start') setStart([w[0], w[1]]);
          else setGoal([w[0], w[1]]);
        }
        drag.current = null;
      }
    },
    [start, goal, s.lab.free],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawWorld(ctx, v, APARTMENT, p);
      const tree = s.planner.tree;
      ctx.save();
      ctx.strokeStyle = p.graph;
      ctx.lineWidth = 0.8;
      ctx.globalAlpha = 0.45;
      ctx.beginPath();
      for (let i = 1; i < tree.size; i++) {
        const q = tree.q(i);
        const pq = tree.q(tree.parent(i));
        ctx.moveTo(sx(v, pq[0]), sy(v, pq[1]));
        ctx.lineTo(sx(v, q[0]), sy(v, q[1]));
      }
      ctx.stroke();
      ctx.restore();
      // Post-solution samples: inside the ellipse in ink, outside faint — the wasted ones.
      for (const smp of s.samples) paintDot(ctx, sx(v, smp.q[0]), sy(v, smp.q[1]), smp.inside ? 1.6 : 2.2, smp.inside ? p.path : p.obstacle, smp.inside ? 0.45 : 0.25);
      if (s.planner.solved) paintEllipse(ctx, v, s.start, s.goal, s.planner.bestCost, p.path, 1);
      const path = s.planner.bestPath();
      if (path) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        path.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q[0]), sy(v, q[1])) : ctx.lineTo(sx(v, q[0]), sy(v, q[1]))));
        ctx.stroke();
        ctx.restore();
      }
      paintDot(ctx, sx(v, s.start[0]), sy(v, s.start[1]), 7, p.start);
      paintDot(ctx, sx(v, s.goal[0]), sy(v, s.goal[1]), 7, p.goal);
      const after = s.planner.solved ? s.planner.iterations - (s.planner.solvedAt ?? 0) : 0;
      label(ctx, s.planner.solved ? `${informed ? 'Informed RRT*' : 'RRT*'} · ${after} iterations after the first solution · c_best = ${s.planner.bestCost.toFixed(3)} m` : `searching… iteration ${s.planner.iterations}`, sx(v, 0.2), sy(v, 8.7), p.ink, { size: 11, weight: 600 });
    },
    [s, informed],
  );

  const ratio = s.planner.solved ? s.readout.measureRatio(s.lab.muBox) : 1;
  const wasted = s.samples.filter((x) => !x.inside).length;

  return (
    <WidgetFrame
      id="w13.2"
      title="Ellipse Shrinker"
      teaches="After the first solution, not every sample is equally useful: only configurations inside the ellipse with foci at start and goal and string length c_best can lie on a shorter path, and the ellipse shrinks with every improvement."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Informed RRT* on the Apartment. The dashed purple ellipse is the informed set — every point{' '}
          <em>x</em> with dist(start, x) + dist(x, goal) ≤ <em>c_best</em>; by the triangle inequality
          nothing outside it can lie on a path shorter than the incumbent. As the cost drops, the
          string gets shorter and the ellipse tightens. The readout is the share of the room still
          worth sampling. Turn informed sampling off and the planner is plain RRT* on the same seed:
          the faint grey dots are the samples it wasted outside the ellipse. Drag the start and goal to
          move the foci; scrub the iterations after the first solution.
        </>
      }
    >
      <SimCanvas world={APARTMENT.bounds} aspect={12 / 9} draw={draw} deps={[sim.tick, s]} onPointer={onPointer} ariaLabel="An apartment floorplan with a tree, a purple path, and a dashed ellipse around start and goal that shrinks as the path improves." />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="μ(X_f̂ ∩ 𝒬) / μ(𝒬)" value={ratio} precision={3} role="path" />
          <StatTile label="c_best / c_min" value={s.planner.solved ? s.planner.bestCost / s.readout.cMin : '—'} precision={3} />
          <StatTile label="samples after first solution" value={s.samples.length} precision={0} role="graph" />
          <StatTile label="landed outside the ellipse" value={s.samples.length ? `${wasted} (${Math.round((100 * wasted) / s.samples.length)}%)` : '—'} role="obstacle" />
        </Dashboard>
      </div>
      <ControlPanel columns={2} title="headline — iterations after the first solution">
        <Slider label="iterations after first solution" role="path" value={budget} min={0} max={4000} step={50} onChange={setBudget} />
        <Toggle label="informed sampling (off = plain RRT*)" role="path" checked={informed} onChange={setInformed} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
