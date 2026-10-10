'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile, type BookRole } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawWorld, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import type { Chart } from '@/lib/manifold/manifold';
import {
  APARTMENT_GOAL,
  APARTMENT_START,
  Bit,
  Fmt,
  InformedRrtStar,
  RrtStar,
  StraightLine,
  TORUS_GOAL,
  TORUS_START,
  chartPanel,
  discLab,
  gammaStarFmt,
  gammaStarSolovey2020,
  latticeFloor,
  paintChartPath,
  paintDot,
  paintPanelFrame,
  paintRaster,
  rasterTorusCObstacle,
  torusLab,
  type Anytime,
  type CountingFreeSpace,
} from '@/lib/sampling';

/**
 * w13.3 — Batch Bench.
 *
 * RRT*, Informed RRT*, FMT* (one batch) and BIT* (batches) on one seed and
 * one world, each given the same wall-clock deadline of *compute* time —
 * measured by slicing: every frame each planner gets a few milliseconds and
 * its own clock advances only while it runs, so the page stays responsive and
 * the curves are honest about time. The anytime contract made literal: stop
 * at the deadline, and the path you would execute is the one drawn.
 */

type Tab = 'disc' | 'torus';
type Key = 'rrtstar' | 'informed' | 'fmt' | 'bit';

const PLANNERS: { key: Key; label: string; role: BookRole }[] = [
  { key: 'rrtstar', label: 'RRT*', role: 'graph' },
  { key: 'informed', label: 'Informed RRT*', role: 'path' },
  { key: 'fmt', label: 'FMT* (one batch)', role: 'cobstacle' },
  { key: 'bit', label: 'BIT* (batches)', role: 'robot' },
];

const SLICE_MS = 3;
const ARENA = { minX: 0, minY: 0, maxX: 1, maxY: 1 };

interface Lane {
  key: Key;
  free: CountingFreeSpace<number[]>;
  planner: Anytime<number[]> | null;
  acc: number;
  sliceStart: number;
  firstAt: number | null;
  make: (now: () => number) => Anytime<number[]>;
}

interface State {
  space: Chart<number[]>;
  lanes: Lane[];
  floor: number | null;
  start: number[];
  goal: number[];
}

export function BatchBench() {
  const [tab, setTab] = useState<Tab>('disc');
  const [deadline, setDeadline] = useState(1500);
  const [batch, setBatch] = useState(100);
  const [fmtN, setFmtN] = useState(1200);
  const deadlineRef = useRef(deadline);
  deadlineRef.current = deadline;

  const raster = useMemo(() => (tab === 'torus' ? rasterTorusCObstacle(torusLab(), 72) : null), [tab]);
  const floorMemo = useMemo(() => (tab === 'disc' ? latticeFloor(discLab(), APARTMENT_START, APARTMENT_GOAL) : null), [tab]);

  const init = useCallback(
    (seed: number): State => {
      const start = tab === 'disc' ? [...APARTMENT_START] : [...TORUS_START];
      const goal = tab === 'disc' ? [...APARTMENT_GOAL] : [...TORUS_GOAL];
      const eta = tab === 'disc' ? 0.8 : 0.4;
      const step = tab === 'disc' ? 0.05 : 0.04;
      const mkLab = () => (tab === 'disc' ? discLab() : torusLab());
      const probe = mkLab();
      const muFree = probe.muBox * (tab === 'disc' ? 0.8 : 0.7);
      const gamma = gammaStarSolovey2020(2, muFree) * 1.1;
      const steer = () => new StraightLine<number[]>({ kind: 'subdivision', step });
      const lanes: Lane[] = PLANNERS.map((p) => {
        const lab = mkLab();
        const base = { steer: steer(), eta, goalBias: 0.05, goalRadius: eta, nbhd: { kind: 'radius' as const, gamma } };
        const make = (now: () => number): Anytime<number[]> => {
          switch (p.key) {
            case 'rrtstar':
              return new RrtStar<number[]>(lab.space, lab.free, start, goal, { ...base, now }, new Rng(seed));
            case 'informed':
              return new InformedRrtStar<number[]>(lab.space, lab.free, start, goal, { ...base, now }, new Rng(seed));
            case 'fmt':
              return new Fmt<number[]>(lab.space, lab.free, start, goal, { steer: steer(), n: fmtN, gamma: gammaStarFmt(2, muFree) * 1.5, now }, new Rng(seed));
            case 'bit':
              return new Bit<number[]>(lab.space, lab.free, start, goal, { steer: steer(), batch, gamma: gamma * 1.3, now }, new Rng(seed));
          }
        };
        return { key: p.key, free: lab.free, planner: null, acc: 0, sliceStart: 0, firstAt: null, make };
      });
      return { space: probe.space, lanes, floor: floorMemo?.cost ?? null, start, goal };
    },
    [tab, batch, fmtN, floorMemo],
  );

  const stepFn = useCallback((s: State): State => {
    const limit = deadlineRef.current;
    for (const lane of s.lanes) {
      if (lane.acc >= limit) continue;
      lane.sliceStart = performance.now();
      const now = () => lane.acc + (performance.now() - lane.sliceStart);
      // Construction counts: FMT*'s whole batch is drawn in its constructor.
      if (!lane.planner) lane.planner = lane.make(now);
      const planner = lane.planner;
      let guard = 0;
      while (now() < limit && performance.now() - lane.sliceStart < SLICE_MS && guard++ < 5000) {
        const improved = planner.step();
        if (improved !== null && lane.firstAt === null) lane.firstAt = now();
        if (planner instanceof Fmt && planner.finished) break;
      }
      lane.acc = Math.min(limit, now());
      if (planner instanceof Fmt && planner.finished) lane.acc = Math.max(lane.acc, limit); // nothing more to do
    }
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 13 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [tab, batch, fmtN, deadline]);

  const s = sim.state;
  const done = s.lanes.every((l) => l.acc >= deadline);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      let X: (q: number[]) => number;
      let Y: (q: number[]) => number;
      let panel: ReturnType<typeof chartPanel> | null = null;
      if (tab === 'disc') {
        X = (q) => sx(v, q[0]);
        Y = (q) => sy(v, q[1]);
        drawWorld(ctx, v, APARTMENT, p);
      } else {
        panel = chartPanel(v, s.space, { x0: 0, y0: 0, x1: 1, y1: 1 });
        if (raster) paintRaster(ctx, panel, raster, p.cobstacle, 0.8);
        paintPanelFrame(ctx, panel, p, true);
        X = (q) => panel!.X(s.space.coords(q)[0]);
        Y = (q) => panel!.Y(s.space.coords(q)[1]);
      }
      // The path each planner would execute if stopped now.
      PLANNERS.forEach((pl, k) => {
        const lane = s.lanes[k];
        const best = lane.planner?.best() ?? null;
        if (!best) return;
        const color = p[pl.role];
        if (panel) paintChartPath(ctx, panel, s.space, best.path, color, 2.5, 0.9);
        else {
          ctx.save();
          ctx.strokeStyle = color;
          ctx.lineWidth = 2.5;
          ctx.globalAlpha = 0.9;
          ctx.lineJoin = 'round';
          ctx.beginPath();
          best.path.forEach((q, i) => (i === 0 ? ctx.moveTo(X(q), Y(q)) : ctx.lineTo(X(q), Y(q))));
          ctx.stroke();
          ctx.restore();
        }
      });
      paintDot(ctx, X(s.start), Y(s.start), 7, p.start);
      paintDot(ctx, X(s.goal), Y(s.goal), 7, p.goal);
      label(ctx, done ? `deadline ${deadline} ms reached — these are the paths you would execute` : `planning… ${Math.round(Math.max(...s.lanes.map((l) => l.acc)))} / ${deadline} ms of compute`, tab === 'disc' ? sx(v, 0.2) : 6, tab === 'disc' ? sy(v, 8.7) : 12, p.ink, { size: 11, weight: 600 });
    },
    [s, tab, raster, done, deadline],
  );

  const series = useMemo(
    () =>
      PLANNERS.map((pl, k) => {
        const lane = s.lanes[k];
        const hist = lane.planner?.costHistory ?? [];
        const data = hist.map((h) => ({ x: h.t, y: h.cost }));
        if (data.length) data.push({ x: lane.acc, y: data[data.length - 1].y });
        return { id: pl.label, role: pl.role, data };
      }),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [Math.floor(sim.tick / 3), s],
  );

  const unit = tab === 'disc' ? 'm' : 'rad';

  return (
    <WidgetFrame
      id="w13.3"
      title="Batch Bench"
      teaches="Asymptotically optimal does not mean better at every budget: FMT* has nothing until its batch is processed, BIT*'s first batch often beats RRT* early, and the curves cross — the deadline decides the winner."
      colorKey={['start', 'goal', 'graph', 'path', 'cobstacle', 'robot']}
      wide
      caption={
        <>
          Four asymptotically optimal planners on one seed, each given the same budget of compute
          time and stopped at the deadline. The chart is cost against planner time; the marks where
          each curve begins are the times to first solution. RRT* and Informed RRT* improve
          incrementally from the moment they first reach the goal. FMT* draws its whole batch first
          and then marches outward in cost order, so its curve is a single step — nothing, then an
          answer. BIT* draws a batch, searches it like A*, and repeats, pruning what cannot improve the
          incumbent. Slide the deadline and watch who is ahead when the clock stops. The drawn paths
          are the ones each would execute.
        </>
      }
    >
      <SimCanvas world={tab === 'disc' ? APARTMENT.bounds : ARENA} aspect={tab === 'disc' ? 12 / 9 : 1.2} padding={tab === 'disc' ? 0.4 : 0.08} draw={draw} deps={[sim.tick, s]} ariaLabel="A world with up to four overlaid solution paths in different colors, one per anytime planner, as they stand at the deadline." />
      <div className="border-t border-fd-border px-3 py-3">
        <LineChart
          series={series}
          xLabel="planner compute time (ms)"
          yLabel={`best cost (${unit})`}
          height={220}
          curve="stepAfter"
          markers={[...(s.floor !== null ? [{ axis: 'y' as const, value: s.floor, label: `lattice floor ${s.floor.toFixed(2)} m` }] : []), { axis: 'x' as const, value: deadline, label: 'deadline', role: 'goal' as const }]}
          ariaLabel="Best path cost against compute time for RRT*, Informed RRT*, FMT* and BIT*, with the deadline marked."
        />
        <div className="mt-3">
          <Dashboard columns={4}>
            {PLANNERS.map((pl, k) => {
              const lane = s.lanes[k];
              const best = lane.planner?.best() ?? null;
              return <StatTile key={pl.key} label={pl.label} value={best ? best.cost : '—'} precision={3} unit={best ? unit : undefined} role={pl.role} trend={lane.firstAt !== null ? Math.round(lane.firstAt) : undefined} trendLabel={lane.firstAt !== null ? 'ms to first solution' : undefined} />;
            })}
          </Dashboard>
        </div>
      </div>
      <ControlPanel columns={2} title="headline — the deadline">
        <Slider label="compute deadline" role="goal" value={deadline} min={50} max={5000} step={50} onChange={setDeadline} unit="ms" />
        <ButtonRow>
          <ActionButton emphasis={tab === 'disc'} onClick={() => setTab('disc')}>
            Rusty · Apartment
          </ActionButton>
          <ActionButton emphasis={tab === 'torus'} onClick={() => setTab('torus')}>
            Reach · Workbench
          </ActionButton>
        </ButtonRow>
      </ControlPanel>
      <ControlPanel columns={2} title="disclosure — batches">
        <Slider label="BIT* batch size" role="robot" value={batch} min={25} max={400} step={25} onChange={setBatch} />
        <Slider label="FMT* batch n" role="cobstacle" value={fmtN} min={300} max={3000} step={100} onChange={setFmtN} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
