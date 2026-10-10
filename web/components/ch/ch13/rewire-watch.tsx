'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { BarChart, Dashboard, LineChart, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import type { Chart } from '@/lib/manifold/manifold';
import {
  APARTMENT_GOAL,
  APARTMENT_START,
  RrtStar,
  StraightLine,
  TORUS_GOAL,
  TORUS_START,
  chartPanel,
  discLab,
  gammaStar,
  kStar,
  latticeFloor,
  paintChartPath,
  paintDot,
  paintGeodesic,
  paintPanelFrame,
  paintPolyWorld,
  paintRaster,
  rasterTorusCObstacle,
  rectMapper,
  tint,
  torusLab,
  type CountingFreeSpace,
  type Mapper,
  type RadiusConstant,
  type Rewire,
} from '@/lib/sampling';

/**
 * w13.1 — Rewire Watch.
 *
 * RRT (left) and RRT* (right) consuming the *same* seeded sample stream — the
 * same q_rand at every iteration, so the two trees have identical node sets
 * and differ only in their edges. The headline is γ/γ*, the radius multiplier
 * of r_n = γ (log n / n)^{1/d}; below 1 the optimality hypothesis fails and
 * the widget says so. Rewired edges flash; the cost curves descend (or not)
 * toward Chapter 6's lattice A* floor; a histogram of |Q_near| hovers near
 * log n.
 */

type Tab = 'disc' | 'torus';

const ARENA = { minX: 0, minY: 0, maxX: 2.12, maxY: 1 };
const LEFT = { x0: 0, y0: 0, x1: 1, y1: 1 };
const RIGHT = { x0: 1.12, y0: 0, x1: 2.12, y1: 1 };
const ITERS_PER_TICK = 6;
const MAX_ITERS = 6000;
const FLASH_TICKS = 18;

interface Flash extends Rewire {
  age: number;
}

interface State {
  space: Chart<number[]>;
  free: [CountingFreeSpace<number[]>, CountingFreeSpace<number[]>];
  plain: RrtStar<number[]>;
  star: RrtStar<number[]>;
  flashes: Flash[];
  costPlain: { x: number; y: number }[];
  costStar: { x: number; y: number }[];
  floor: number | null;
  muFree: number;
  rewired: boolean;
}

export function RewireWatch() {
  const [tab, setTab] = useState<Tab>('disc');
  const [mult, setMult] = useState(1.0);
  const [nbhd, setNbhd] = useState<'radius' | 'kNearest'>('radius');
  const [which, setWhich] = useState<RadiusConstant>('2020');
  const [eta, setEta] = useState(0.8);
  const [pauseOnRewire, setPauseOnRewire] = useState(false);

  const raster = useMemo(() => (tab === 'torus' ? rasterTorusCObstacle(torusLab(), 72) : null), [tab]);
  const floorMemo = useMemo(() => (tab === 'disc' ? latticeFloor(discLab(), APARTMENT_START, APARTMENT_GOAL) : null), [tab]);

  const init = useCallback(
    (seed: number): State => {
      const etaHere = tab === 'disc' ? eta : eta * 0.5;
      const labA = tab === 'disc' ? discLab() : torusLab();
      const labB = tab === 'disc' ? discLab() : torusLab();
      const start = tab === 'disc' ? APARTMENT_START : TORUS_START;
      const goal = tab === 'disc' ? APARTMENT_GOAL : TORUS_GOAL;
      // μ(𝒬_free): the box less the walls' share — about 80% of the Apartment, 70% of the torus.
      const muFree = labA.muBox * (tab === 'disc' ? 0.8 : 0.7);
      const gamma = gammaStar(2, muFree, which) * mult;
      const mk = (lab: typeof labA, rewire: boolean) =>
        new RrtStar<number[]>(
          lab.space,
          lab.free,
          [...start],
          [...goal],
          {
            steer: new StraightLine<number[]>({ kind: 'subdivision', step: tab === 'disc' ? 0.05 : 0.04 }),
            eta: etaHere,
            goalBias: 0.05,
            goalRadius: etaHere,
            nbhd: nbhd === 'radius' ? { kind: 'radius', gamma } : { kind: 'kNearest', kStar: kStar(2) * mult },
            rewire,
          },
          new Rng(seed),
        );
      return { space: labA.space, free: [labA.free, labB.free], plain: mk(labA, false), star: mk(labB, true), flashes: [], costPlain: [], costStar: [], floor: floorMemo?.cost ?? null, muFree, rewired: false };
    },
    [tab, mult, nbhd, which, eta, floorMemo],
  );

  const stepFn = useCallback((s: State): State => {
    s.rewired = false;
    if (s.star.iterations < MAX_ITERS) {
      for (let k = 0; k < ITERS_PER_TICK; k++) {
        s.plain.iterate();
        const r = s.star.iterate();
        for (const rw of r.rewires) s.flashes.push({ ...rw, age: 0 });
        if (r.rewires.length) s.rewired = true;
        if (s.star.iterations % 25 === 0) {
          if (Number.isFinite(s.plain.bestCost)) s.costPlain.push({ x: s.star.iterations, y: s.plain.bestCost });
          if (Number.isFinite(s.star.bestCost)) s.costStar.push({ x: s.star.iterations, y: s.star.bestCost });
        }
      }
    }
    for (const f of s.flashes) f.age++;
    s.flashes = s.flashes.filter((f) => f.age < FLASH_TICKS);
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 13 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [tab, mult, nbhd, which, eta]);

  const s = sim.state;
  const pauseRef = useRef(sim.pause);
  pauseRef.current = sim.pause;
  useEffect(() => {
    if (pauseOnRewire && s.rewired) pauseRef.current();
  }, [s, pauseOnRewire]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const panes: { rect: typeof LEFT; planner: RrtStar<number[]>; title: string }[] = [
        { rect: LEFT, planner: s.plain, title: 'RRT — same samples, parent := q_near, no rewire' },
        { rect: RIGHT, planner: s.star, title: `RRT* — choose-parent + rewire in ${nbhd === 'radius' ? 'r_n' : 'k_n'}` },
      ];
      for (const pane of panes) {
        const tree = pane.planner.tree;
        let X: (q: number[]) => number;
        let Y: (q: number[]) => number;
        let panel: ReturnType<typeof chartPanel> | null = null;
        let m: Mapper | null = null;
        if (tab === 'disc') {
          m = rectMapper(v, pane.rect, APARTMENT.bounds);
          paintPolyWorld(ctx, m, APARTMENT, p);
          X = (q) => m!.X(q[0]);
          Y = (q) => m!.Y(q[1]);
        } else {
          panel = chartPanel(v, s.space, pane.rect);
          if (raster) paintRaster(ctx, panel, raster, p.cobstacle, 0.8);
          paintPanelFrame(ctx, panel, p, true);
          X = (q) => panel!.X(s.space.coords(q)[0]);
          Y = (q) => panel!.Y(s.space.coords(q)[1]);
        }
        // Edges.
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.lineWidth = 0.9;
        ctx.globalAlpha = 0.7;
        ctx.beginPath();
        for (let i = 1; i < tree.size; i++) {
          const q = tree.q(i);
          const pq = tree.q(tree.parent(i));
          if (panel) paintGeodesic(ctx, panel, s.space, pq, q, 5);
          else {
            ctx.moveTo(X(pq), Y(pq));
            ctx.lineTo(X(q), Y(q));
          }
        }
        ctx.stroke();
        ctx.restore();
        // Rewire flashes on the RRT* pane: white-to-blue by age.
        if (pane.planner === s.star) {
          for (const f of s.flashes) {
            if (f.node >= tree.size) continue;
            const q = tree.q(f.node);
            const pq = tree.q(f.newParent);
            ctx.save();
            ctx.strokeStyle = tint(p.graph, Math.round(85 * (1 - f.age / FLASH_TICKS)), p.bg);
            ctx.lineWidth = 3 - (2 * f.age) / FLASH_TICKS;
            ctx.beginPath();
            if (panel) paintGeodesic(ctx, panel, s.space, pq, q, 5);
            else {
              ctx.moveTo(X(pq), Y(pq));
              ctx.lineTo(X(q), Y(q));
            }
            ctx.stroke();
            ctx.restore();
          }
        }
        const path = pane.planner.bestPath();
        if (path) {
          if (panel) paintChartPath(ctx, panel, s.space, path, p.path, 3);
          else {
            ctx.save();
            ctx.strokeStyle = p.path;
            ctx.lineWidth = 3;
            ctx.lineJoin = 'round';
            ctx.beginPath();
            path.forEach((q, i) => (i === 0 ? ctx.moveTo(X(q), Y(q)) : ctx.lineTo(X(q), Y(q))));
            ctx.stroke();
            ctx.restore();
          }
        }
        paintDot(ctx, X(pane.planner.start), Y(pane.planner.start), 6, p.start);
        paintDot(ctx, X(pane.planner.goal), Y(pane.planner.goal), 6, p.goal);
        const ox = panel ? panel.px : m!.X(APARTMENT.bounds.minX);
        const oy = panel ? panel.py - 8 : m!.Y(APARTMENT.bounds.maxY) - 8;
        label(ctx, pane.title, ox, oy, p.ink, { size: 10, weight: 600 });
        const c = pane.planner.bestCost;
        label(ctx, Number.isFinite(c) ? `cost ${c.toFixed(3)}${tab === 'disc' ? ' m' : ' rad'}` : 'no solution yet', ox, (panel ? panel.py + panel.h : m!.Y(APARTMENT.bounds.minY)) + 12, Number.isFinite(c) ? p.path : p.ink, { size: 10, weight: 700 });
      }
      if (mult < 1) label(ctx, `γ/γ* = ${mult.toFixed(2)} < 1: the optimality theorem's hypothesis fails — the ball shrinks too fast`, v.width / 2, v.height - 4, p.goal, { size: 10, weight: 700, align: 'center' });
    },
    [s, tab, nbhd, raster, mult],
  );

  const hist = useMemo(() => {
    const sizes = s.star.nearSizes.slice(-600);
    const bins = new Map<number, number>();
    for (const n of sizes) bins.set(n, (bins.get(n) ?? 0) + 1);
    const keys = [...bins.keys()].sort((a, b) => a - b);
    return keys.map((k) => ({ x: k, y: bins.get(k)! / Math.max(1, sizes.length) }));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [Math.floor(sim.tick / 8), s.star]);

  const logN = Math.log(Math.max(2, s.star.tree.size));
  const unit = tab === 'disc' ? 'm' : 'rad';

  return (
    <WidgetFrame
      id="w13.1"
      title="Rewire Watch"
      teaches="More samples do not make RRT's path better — its cost curve is flat forever. RRT* on the very same samples descends, because choose-parent and rewire let the tree change its mind inside a ball that shrinks just slowly enough."
      colorKey={tab === 'disc' ? ['start', 'goal', 'obstacle', 'graph', 'path'] : ['start', 'goal', 'cobstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Left and right consume the <em>same</em> seeded stream of random targets, so the two trees
          have identical nodes; only the edges differ. RRT attaches each new node to its nearest
          neighbor and never reconsiders. RRT* attaches it to the neighbor in the ball of radius{' '}
          <em>r_n</em> that minimizes cost-to-come, then rewires any neighbor that is now cheaper
          through the new node — the flashing edges — and propagates the saving down the subtree. The
          cost chart tells the story: the left curve changes only when a new node happens to enter the
          goal region more cheaply; the right one descends toward the dashed lattice floor. Slide γ/γ*
          below 1 and watch the right curve stall — the hypothesis of the theorem is real.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.12} padding={0.07} draw={draw} deps={[sim.tick, s]} ariaLabel="Two trees side by side from the same random samples: RRT on the left with a jagged path, RRT* on the right with rewired edges flashing and a straighter path." />
      <div className="border-t border-fd-border px-3 py-3">
        <div className="grid grid-cols-1 gap-3 lg:grid-cols-[1fr_minmax(0,18rem)]">
          <LineChart
            series={[
              { id: 'RRT', role: 'obstacle', data: s.costPlain },
              { id: 'RRT*', role: 'path', data: s.costStar },
            ]}
            xLabel="iterations"
            yLabel={`best cost (${unit})`}
            height={190}
            curve="stepAfter"
            markers={s.floor !== null ? [{ axis: 'y', value: s.floor, label: `lattice A* floor ${s.floor.toFixed(2)} m`, role: 'graph' }] : undefined}
            ariaLabel="Best path cost against iterations for RRT and RRT* on the same samples, with the lattice A* cost as a dashed floor."
          />
          <div className="flex flex-col gap-3">
            <BarChart series={[{ id: '|Q_near|', role: 'graph', data: hist }]} xLabel="|Q_near| at insertion" yLabel="fraction" height={140} legend={false} ariaLabel="Histogram of the neighborhood size at each insertion; it hovers near the logarithm of the tree size." />
            <Dashboard columns={2}>
              <StatTile label="log n" value={logN} precision={1} />
              <StatTile label="rewires · propagated" value={`${s.star.rewires} · ${s.star.propagated}`} role="graph" />
            </Dashboard>
          </div>
        </div>
        <div className="mt-3">
          <Dashboard columns={4}>
            <StatTile label="iterations" value={s.star.iterations} precision={0} />
            <StatTile label="nodes (both trees)" value={s.star.tree.size} precision={0} role="graph" />
            <StatTile label="RRT cost" value={Number.isFinite(s.plain.bestCost) ? s.plain.bestCost : '—'} precision={3} unit={unit} role="obstacle" />
            <StatTile label="RRT* cost" value={Number.isFinite(s.star.bestCost) ? s.star.bestCost : '—'} precision={3} unit={unit} role="path" trend={s.floor !== null && Number.isFinite(s.star.bestCost) ? Math.round(((s.star.bestCost - s.floor) / s.floor) * 1000) / 10 : undefined} trendLabel="% vs lattice floor" />
          </Dashboard>
        </div>
      </div>
      <ControlPanel columns={3} title="headline — the radius multiplier">
        <Slider label="γ / γ*" role="graph" value={mult} min={0.5} max={3} step={0.1} onChange={setMult} format={(x) => `${x.toFixed(1)}${x < 1 ? ' — below the bound' : ''}`} />
        <ButtonRow>
          <ActionButton emphasis={tab === 'disc'} onClick={() => setTab('disc')}>
            Rusty · Apartment
          </ActionButton>
          <ActionButton emphasis={tab === 'torus'} onClick={() => setTab('torus')}>
            Reach · torus
          </ActionButton>
        </ButtonRow>
        <Slider label="η cap" value={eta} min={0.3} max={1.5} step={0.1} onChange={setEta} unit={tab === 'disc' ? 'm' : '× 0.5 rad'} />
      </ControlPanel>
      <ControlPanel columns={3} title="disclosure — neighborhood rule">
        <ButtonRow>
          <ActionButton emphasis={nbhd === 'radius'} onClick={() => setNbhd('radius')}>
            r_n ball
          </ActionButton>
          <ActionButton emphasis={nbhd === 'kNearest'} onClick={() => setNbhd('kNearest')}>
            k_n nearest
          </ActionButton>
        </ButtonRow>
        <ButtonRow>
          <ActionButton emphasis={which === '2011'} onClick={() => setWhich('2011')}>
            γ* of 2011
          </ActionButton>
          <ActionButton emphasis={which === '2020'} onClick={() => setWhich('2020')}>
            γ* of 2020
          </ActionButton>
        </ButtonRow>
        <Toggle label="pause on rewire" checked={pauseOnRewire} onChange={setPauseOnRewire} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
