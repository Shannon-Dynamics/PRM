'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import { WORKBENCH } from '@/lib/robots/worlds';
import type { BitGrid } from '@/lib/manifold/torus';
import {
  APARTMENT_GOAL,
  APARTMENT_START,
  Prm,
  StraightLine,
  TORUS_GOAL,
  TORUS_START,
  UnionFind,
  chartPanel,
  checkSchedule,
  discLab,
  paintArmMapped,
  paintDot,
  paintGeodesic,
  paintPanelFrame,
  paintPolyWorld,
  paintRaster,
  rasterTorusCObstacle,
  rectMapper,
  strategySampler,
  tint,
  torusLab,
  type CountingFreeSpace,
  type DiscLab,
  type EdgeCheck,
  type Sampler,
  type StrategyName,
  type TorusLab,
} from '@/lib/sampling';

/**
 * w11.1 — Roadmap Rain.
 *
 * Choset's BASIC PRM (Algorithm 6) grown one node at a time, with q_start and
 * q_goal inserted as the roadmap's first two nodes (§7.2: "PRM itself can also
 * be used as a single-query planner"). Coarse rain: nodes fall and each joins
 * its k nearest neighbors that the straight-line local planner can reach.
 * Fine rain: every edge is a row of collision checks at `step`. The two rains
 * are metered separately — Δ calls and collision checks — and the component
 * count is the one curve the reader should watch: it collapses the same way on
 * every seed, and the frame where start and goal first share a component is
 * frozen as a marker on it.
 */

type Tab = 'disc' | 'torus';
type SamplerKind = StrategyName;

const N_MAX = 1200;
const BATCH = 8;
const TORUS_ARENA = { minX: 0, minY: 0, maxX: 2.12, maxY: 1 };
const TORUS_PANEL = { x0: 0, y0: 0, x1: 1, y1: 1 };
const BENCH_PANEL = { x0: 1.12, y0: 0, x1: 2.12, y1: 1 };

const SAMPLERS: { kind: SamplerKind; label: string }[] = [
  { kind: 'uniform', label: 'uniform' },
  { kind: 'gaussian', label: 'Gaussian' },
  { kind: 'bridge', label: 'bridge + uniform' },
  { kind: 'obprm', label: 'OBPRM' },
  { kind: 'medial-axis', label: 'medial axis' },
];

/** Scales in metres (disc) or radians (torus): about one robot diameter / one link width. */
const makeSampler = (kind: SamplerKind, tab: Tab, free: CountingFreeSpace<number[]>): Sampler<number[]> => strategySampler<number[]>(kind, tab === 'disc' ? 1 : 0.8, free.clearance);

interface State {
  prm: Prm<number[]>;
  steer: StraightLine<number[]>;
  free: CountingFreeSpace<number[]>;
  lab: DiscLab | TorusLab;
  joinedAt: number | null;
  cache: { shown: number; path: number[][]; cost: number } | null;
}

interface Params {
  tab: Tab;
  k: number;
  sampler: SamplerKind;
  check: EdgeCheck;
  start: number[];
  goal: number[];
}

function build(seed: number, prms: Params): State {
  const lab = prms.tab === 'disc' ? discLab() : torusLab();
  const steer = new StraightLine<number[]>(prms.check);
  const prm = new Prm<number[]>(
    lab.space,
    lab.free,
    { sampler: makeSampler(prms.sampler, prms.tab, lab.free), steer, connect: { kind: 'kNearest', k: prms.k }, nn: 'kd' },
    new Rng(seed),
  );
  // q_start and q_goal are nodes 0 and 1: "joined" is then one union–find lookup.
  const s = prm.addNode(prms.start);
  prm.connectNode(s, prm.candidates(s));
  const g = prm.addNode(prms.goal);
  prm.connectNode(g, prm.candidates(g));
  prm.componentHistory.push(prm.roadmap.components(), prm.roadmap.components());
  return { prm, steer, free: lab.free, lab, joinedAt: prm.roadmap.sameComponent(0, 1) ? 2 : null, cache: null };
}

/** Component label of every node among the first n, by a union–find over the visible edges. */
function componentLabels(prm: Prm<number[]>, n: number): { label: Int32Array; count: number } {
  const uf = new UnionFind();
  for (let i = 0; i < n; i++) uf.make();
  const adj = prm.roadmap.adj;
  for (let a = 0; a < n; a++) for (const e of adj[a]) if (e.to < n && a < e.to) uf.union(a, e.to);
  const out = new Int32Array(n);
  for (let i = 0; i < n; i++) out[i] = uf.find(i);
  return { label: out, count: uf.count };
}

export function RoadmapRain() {
  const [tab, setTab] = useState<Tab>('disc');
  const [nTarget, setNTarget] = useState(600);
  const [k, setK] = useState(6);
  const [samplerKind, setSamplerKind] = useState<SamplerKind>('uniform');
  const [checkKind, setCheckKind] = useState<EdgeCheck['kind']>('subdivision');
  const [step, setStep] = useState(0.25);
  const [start, setStart] = useState<number[]>(APARTMENT_START);
  const [goal, setGoal] = useState<number[]>(APARTMENT_GOAL);
  const [hover, setHover] = useState<[number, number] | null>(null);
  const drag = useRef<'start' | 'goal' | null>(null);

  const raster = useMemo<BitGrid | null>(() => (tab === 'torus' ? rasterTorusCObstacle(torusLab(), 72) : null), [tab]);

  const params = useMemo<Params>(
    () => ({ tab, k, sampler: samplerKind, check: { kind: checkKind, step: tab === 'disc' ? step : step * 0.8 }, start, goal }),
    [tab, k, samplerKind, checkKind, step, start, goal],
  );

  const init = useCallback((seed: number) => build(seed, params), [params]);
  const targetRef = useRef(nTarget);
  targetRef.current = nTarget;

  const stepFn = useCallback((s: State): State => {
    const want = Math.min(targetRef.current, N_MAX);
    if (s.prm.n >= want) return s;
    let added = 0;
    let attempts = 0;
    while (added < BATCH && s.prm.n < want && attempts++ < 400) {
      if (s.prm.addSample() !== null) {
        added++;
        if (s.joinedAt === null && s.prm.roadmap.sameComponent(0, 1)) s.joinedAt = s.prm.n;
      }
    }
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 11 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [params]);

  const switchTab = (t: Tab) => {
    if (t === tab) return;
    setTab(t);
    setStart(t === 'disc' ? APARTMENT_START : TORUS_START);
    setGoal(t === 'disc' ? APARTMENT_GOAL : TORUS_GOAL);
    setHover(null);
  };

  const s = sim.state;
  const shown = Math.min(s.prm.n, nTarget);
  const joined = s.joinedAt !== null && shown >= s.joinedAt;
  if (joined && (!s.cache || s.cache.shown !== shown)) {
    const sp = s.prm.roadmap.shortestPath(0, 1, shown);
    s.cache = sp ? { shown, path: s.prm.roadmap.pathPoints(sp.path), cost: sp.cost } : null;
  }
  const path = joined ? s.cache?.path ?? null : null;
  const labels = componentLabels(s.prm, shown);
  const space = s.lab.space;

  // Chart-space coordinates of a node for hover tests: world (disc) or the torus panel's unit square.
  const nodeXY = useCallback(
    (q: number[]): [number, number] => {
      if (tab === 'disc') return [q[0], q[1]];
      const c = space.coords(q);
      return [(c[0] + Math.PI) / (2 * Math.PI), (c[1] + Math.PI) / (2 * Math.PI)];
    },
    [tab, space],
  );

  const hoveredEdge = useMemo(() => {
    if (!hover) return null;
    const [hx, hy] = hover;
    let best: { a: number; b: number; d: number } | null = null;
    const adj = s.prm.roadmap.adj;
    const nodes = s.prm.roadmap.nodes;
    const thresh = tab === 'disc' ? 0.25 : 0.02;
    for (let a = 0; a < shown; a++) {
      const [ax, ay] = nodeXY(nodes[a]);
      for (const e of adj[a]) {
        if (e.to <= a || e.to >= shown) continue;
        const [bx, by] = nodeXY(nodes[e.to]);
        if (tab === 'torus' && (Math.abs(bx - ax) > 0.5 || Math.abs(by - ay) > 0.5)) continue; // seam edge
        const vx = bx - ax;
        const vy = by - ay;
        const L2 = vx * vx + vy * vy || 1e-12;
        const t = Math.max(0, Math.min(1, ((hx - ax) * vx + (hy - ay) * vy) / L2));
        const d = Math.hypot(ax + t * vx - hx, ay + t * vy - hy);
        if (d < thresh && (!best || d < best.d)) best = { a, b: e.to, d };
      }
    }
    return best;
  }, [hover, s.prm, shown, tab, nodeXY]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const toCfg = (): number[] | null => {
        if (tab === 'disc') return [w[0], w[1]];
        if (w[0] < 0 || w[0] > 1 || w[1] < 0 || w[1] > 1) return null;
        return [-Math.PI + w[0] * 2 * Math.PI, -Math.PI + w[1] * 2 * Math.PI];
      };
      const near = (q: number[]) => {
        const [x, y] = nodeXY(q);
        return Math.hypot(x - w[0], y - w[1]) < (tab === 'disc' ? 0.35 : 0.035);
      };
      if (phase === 'down') {
        if (near(start)) drag.current = 'start';
        else if (near(goal)) drag.current = 'goal';
      } else if (phase === 'move') {
        if (drag.current === null) {
          setHover(w);
          return;
        }
      } else {
        const q = toCfg();
        if (drag.current && q && s.free.isFree(q)) {
          if (drag.current === 'start') setStart(q);
          else setGoal(q);
        }
        drag.current = null;
      }
    },
    [tab, start, goal, nodeXY, s.free],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const nodes = s.prm.roadmap.nodes;
      const adj = s.prm.roadmap.adj;
      const colorOf = (i: number) => {
        if (labels.count <= 1) return p.graph;
        const root = labels.label[i];
        return tint(p.graph, ((root * 7919) % 5) * 11, p.accent);
      };

      // Position mapping for the two tabs.
      let X: (q: number[]) => number;
      let Y: (q: number[]) => number;
      let panel: ReturnType<typeof chartPanel> | null = null;
      if (tab === 'disc') {
        drawWorld(ctx, v, APARTMENT, p);
        X = (q) => sx(v, q[0]);
        Y = (q) => sy(v, q[1]);
      } else {
        panel = chartPanel(v, space, TORUS_PANEL);
        if (raster) paintRaster(ctx, panel, raster, p.cobstacle, 0.75);
        paintPanelFrame(ctx, panel, p, true);
        X = (q) => panel!.X(space.coords(q)[0]);
        Y = (q) => panel!.Y(space.coords(q)[1]);
        label(ctx, 'θ₁ →', panel.px + panel.w - 4, panel.py + panel.h + 12, p.ink, { size: 10, align: 'right' });
        label(ctx, '↑ θ₂', panel.px + 4, panel.py + 10, p.ink, { size: 10 });
        // The Workbench, with Reach at the start (green) and goal (red) and, once joined, walking the path.
        const lab = s.lab as TorusLab;
        const m = rectMapper(v, BENCH_PANEL, WORKBENCH.bounds);
        paintPolyWorld(ctx, m, WORKBENCH, p);
        paintArmMapped(ctx, m, p, lab.arm, start, p.start, 0.55);
        paintArmMapped(ctx, m, p, lab.arm, goal, p.goal, 0.55);
        if (path && path.length > 1) {
          let L = 0;
          const seg: number[] = [0];
          for (let i = 1; i < path.length; i++) {
            L += space.dist(path[i - 1], path[i]);
            seg.push(L);
          }
          const t = ((sim.tick * 0.02) % 1.2) * L;
          let i = 1;
          while (i < seg.length - 1 && seg[i] < t) i++;
          const q = t >= L ? path[path.length - 1] : space.interpolate(path[i - 1], path[i], (t - seg[i - 1]) / Math.max(1e-9, seg[i] - seg[i - 1]));
          paintArmMapped(ctx, m, p, lab.arm, q, p.robot, 1);
        }
        label(ctx, 'Workbench', m.X(WORKBENCH.bounds.minX) + 4, m.Y(WORKBENCH.bounds.maxY) + 10, p.ink, { size: 10 });
      }

      // Edges, tinted by the component they belong to.
      ctx.save();
      ctx.lineWidth = 1;
      ctx.globalAlpha = 0.65;
      const byColor = new Map<string, [number, number][]>();
      for (let a = 0; a < shown; a++) {
        for (const e of adj[a]) {
          if (e.to <= a || e.to >= shown) continue;
          const c = colorOf(a);
          const list = byColor.get(c) ?? [];
          list.push([a, e.to]);
          byColor.set(c, list);
        }
      }
      for (const [c, list] of byColor) {
        ctx.strokeStyle = c;
        ctx.beginPath();
        for (const [a, b] of list) {
          if (panel) paintGeodesic(ctx, panel, space, nodes[a], nodes[b], 6);
          else {
            ctx.moveTo(X(nodes[a]), Y(nodes[a]));
            ctx.lineTo(X(nodes[b]), Y(nodes[b]));
          }
        }
        ctx.stroke();
      }
      ctx.restore();
      // Nodes.
      const r = tab === 'disc' ? Math.max(1.5, sl(v, 0.06)) : 2;
      for (let i = 2; i < shown; i++) paintDot(ctx, X(nodes[i]), Y(nodes[i]), r, colorOf(i), 0.9);

      // The hovered edge: Choset figure 7.6's numbered checks, in the schedule's order.
      if (hoveredEdge) {
        const a = nodes[hoveredEdge.a];
        const b = nodes[hoveredEdge.b];
        ctx.save();
        ctx.strokeStyle = p.ink;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(X(a), Y(a));
        ctx.lineTo(X(b), Y(b));
        ctx.stroke();
        ctx.restore();
        const ts = checkSchedule(s.steer.check, space.dist(a, b));
        ts.slice(0, 24).forEach((t, i) => {
          const q = space.interpolate(a, b, t);
          paintDot(ctx, X(q), Y(q), 6, p.bg, 1);
          paintDot(ctx, X(q), Y(q), 5, i === 0 ? p.goal : p.ink, 1);
          label(ctx, String(i + 1), X(q), Y(q), p.bg, { size: 8, align: 'center', weight: 700 });
        });
        label(ctx, `${ts.length} interior check${ts.length === 1 ? '' : 's'} (${s.steer.check.kind})`, X(b) + 8, Y(b) - 10, p.ink, { size: 10, weight: 600 });
      }

      // Path, start, goal.
      if (path) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        for (let i = 1; i < path.length; i++) {
          if (panel) paintGeodesic(ctx, panel, space, path[i - 1], path[i], 8);
          else {
            ctx.moveTo(X(path[i - 1]), Y(path[i - 1]));
            ctx.lineTo(X(path[i]), Y(path[i]));
          }
        }
        ctx.stroke();
        ctx.restore();
      }
      paintDot(ctx, X(start), Y(start), 7, p.start);
      paintDot(ctx, X(goal), Y(goal), 7, p.goal);
      if (tab === 'disc') {
        label(ctx, 'q_start', X(start) + 9, Y(start), p.start, { size: 10, weight: 600 });
        label(ctx, 'q_goal', X(goal) + 9, Y(goal), p.goal, { size: 10, weight: 600 });
      }
      if (s.joinedAt !== null && shown >= s.joinedAt) {
        label(ctx, `joined at n = ${s.joinedAt}`, tab === 'disc' ? sx(v, 0.2) : 6, tab === 'disc' ? sy(v, 8.75) : v.height - 8, p.path, { size: 11, weight: 700 });
      }
    },
    [s, shown, labels, tab, space, raster, path, start, goal, hoveredEdge, sim.tick],
  );

  const history = useMemo(() => {
    const h = s.prm.componentHistory.slice(0, Math.max(2, shown));
    const stride = Math.max(1, Math.floor(h.length / 120));
    const data: { x: number; y: number }[] = [];
    for (let i = 0; i < h.length; i += stride) data.push({ x: i, y: h[i] });
    if ((h.length - 1) % stride !== 0) data.push({ x: h.length - 1, y: h[h.length - 1] });
    return data;
    // Redraw the chart every few ticks, not every frame.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [Math.floor(sim.tick / 6), shown, s.prm]);

  const unit = tab === 'disc' ? 'm' : 'rad';

  return (
    <WidgetFrame
      id="w11.1"
      title="Roadmap Rain"
      teaches="A PRM is a random graph, but whether it connects is not luck: the component count collapses the same way on every seed — luck lives in the constants, not the shape."
      colorKey={tab === 'disc' ? ['start', 'goal', 'obstacle', 'graph', 'path'] : ['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Choset&rsquo;s BASIC PRM, one node per drop. Each new sample joins its {k} nearest neighbors
          that the straight-line local planner can reach; the edge check is the fine rain, and
          hovering an edge shows its collision checks numbered in the order the {checkKind}{' '}
          schedule asks them. Nodes and edges wear one tint per component until the components merge.
          The chart beneath is the one to watch: the component count collapses on every seed, and the
          marker is the sample at which <span style={{ color: 'var(--rm-start)' }}>q_start</span> and{' '}
          <span style={{ color: 'var(--rm-goal)' }}>q_goal</span> first shared a component. Scrub{' '}
          <em>n</em> backward to replay the build (it is deterministic in the seed), switch to
          Reach&rsquo;s torus, change the sampler, drag the endpoints, re-roll.
        </>
      }
    >
      <SimCanvas
        world={tab === 'disc' ? APARTMENT.bounds : TORUS_ARENA}
        aspect={tab === 'disc' ? 12 / 9 : 2.12}
        padding={tab === 'disc' ? 0.4 : 0.06}
        draw={draw}
        deps={[sim.tick, s, shown, hoveredEdge, nTarget]}
        onPointer={onPointer}
        cursor="crosshair"
        ariaLabel={
          tab === 'disc'
            ? 'A probabilistic roadmap growing in the Apartment: blue nodes and edges tinted by connected component, a purple path once start and goal are joined.'
            : 'A probabilistic roadmap on the unwrapped torus of a two-link arm, amber C-obstacle cells behind it, and the Workbench beside it with the arm walking the found path.'
        }
      />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="nodes shown / built" value={`${shown} / ${s.prm.n}`} role="graph" />
          <StatTile label="components" value={labels.count} role="graph" sparkline={s.prm.componentHistory.slice(0, Math.max(2, shown))} precision={0} />
          <StatTile label="Δ calls (edges tried)" value={s.steer.calls} precision={0} role="path" />
          <StatTile label="collision checks" value={s.free.checks} precision={0} role="obstacle" trend={s.prm.rejected} trendLabel="rejected samples" />
        </Dashboard>
        <div className="mt-3">
          <LineChart
            series={[{ id: 'components', role: 'graph', data: history }]}
            xLabel="samples n"
            yLabel="components"
            height={170}
            legend={false}
            curve="stepAfter"
            yMin={0}
            markers={s.joinedAt !== null ? [{ axis: 'x', value: s.joinedAt, label: `joined, n = ${s.joinedAt}`, role: 'path' }] : undefined}
            ariaLabel="Number of connected components of the roadmap against the number of samples, with a marker where start and goal first joined."
          />
        </div>
      </div>
      <ControlPanel columns={3} title="headline — sample count">
        <Slider label="samples n" role="graph" value={nTarget} min={2} max={N_MAX} step={2} onChange={setNTarget} format={(x) => `${x.toFixed(0)}${x < s.prm.n ? ' (replay)' : ''}`} />
        <Slider label="neighbors k" value={k} min={1} max={12} step={1} onChange={setK} />
        <ButtonRow>
          <ActionButton emphasis={tab === 'disc'} onClick={() => switchTab('disc')}>
            Rusty · Apartment (ℝ²)
          </ActionButton>
          <ActionButton emphasis={tab === 'torus'} onClick={() => switchTab('torus')}>
            Reach · Workbench (T²)
          </ActionButton>
        </ButtonRow>
      </ControlPanel>
      <ControlPanel columns={3} title="disclosure — the two rains">
        <ButtonRow>
          {SAMPLERS.map((sm) => (
            <ActionButton key={sm.kind} emphasis={samplerKind === sm.kind} onClick={() => setSamplerKind(sm.kind)}>
              {sm.label}
            </ActionButton>
          ))}
        </ButtonRow>
        <ButtonRow>
          <ActionButton emphasis={checkKind === 'subdivision'} onClick={() => setCheckKind('subdivision')}>
            subdivision (midpoint first)
          </ActionButton>
          <ActionButton emphasis={checkKind === 'incremental'} onClick={() => setCheckKind('incremental')}>
            incremental
          </ActionButton>
        </ButtonRow>
        <Slider label={`edge check step (${unit})`} role="obstacle" value={step} min={0.05} max={0.6} step={0.05} onChange={setStep} format={(x) => (tab === 'disc' ? x : x * 0.8).toFixed(2)} help="A finite step can miss a collision thinner than the step; only a distance-based schedule cannot." />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
