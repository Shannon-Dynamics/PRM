'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile, type BookRole } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import {
  Prm,
  StraightLine,
  corridorExpansiveness,
  corridorWorld,
  expansiveSampleCount,
  fillPolygons,
  paintDot,
  polygonsOf,
  strategySampler,
  type BoxWorld,
  type Sampler,
  type StrategyName,
} from '@/lib/sampling';

/**
 * w11.2 — Narrow Passage.
 *
 * Choset figure 7.19: two W × W rooms joined by a corridor of width w. For
 * every sampler strategy the widget builds many seeded roadmaps at every
 * corridor width and plots the fraction that joined the rooms, live, while the
 * canvas shows *where* the active strategy drops its samples. The readout
 * beside the measured rate is figure 7.19's estimate ε ≈ α ≈ β ≈ w/W, and the
 * sample count Theorem 7.4.2 would demand for it — right shape, hopeless
 * constants.
 */

type Strategy = StrategyName;

const W = 4;
const WIDTHS = [0.15, 0.25, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0];
const SEEDS = 24;
const K = 6;
const JOBS_PER_TICK = 2;

const STRATEGIES: { key: Strategy; label: string; role: BookRole }[] = [
  { key: 'uniform', label: 'uniform', role: 'graph' },
  { key: 'gaussian', label: 'Gaussian', role: 'cobstacle' },
  { key: 'bridge', label: 'bridge + uniform', role: 'path' },
  { key: 'obprm', label: 'OBPRM', role: 'robot' },
  { key: 'medial-axis', label: 'medial axis', role: 'start' },
];

const makeSampler = (s: Strategy, world: BoxWorld): Sampler<number[]> => strategySampler<number[]>(s, 1, (q) => world.clearance(q));

// Off the corridor's axis, so no single straight line joins the rooms: the roadmap must find the passage.
const startOf = (bw: BoxWorld): number[] => [0.6, 0.7];
const goalOf = (bw: BoxWorld): number[] => [bw.bounds.maxX - 0.6, bw.bounds.maxY - 0.7];

/** One seeded roadmap; true if the two rooms ended in one component. */
function buildOnce(strategy: Strategy, w: number, n: number, seed: number): { prm: Prm<number[]>; world: BoxWorld; joined: boolean } {
  const world = corridorWorld(W, w);
  const prm = new Prm<number[]>(
    world.space,
    world.free,
    { sampler: makeSampler(strategy, world), steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.1 }), connect: { kind: 'kNearest', k: K }, nn: 'kd' },
    new Rng(seed),
  );
  const a = prm.addNode(startOf(world));
  prm.connectNode(a, prm.candidates(a));
  const b = prm.addNode(goalOf(world));
  prm.connectNode(b, prm.candidates(b));
  prm.grow(n + 2, 200 * n);
  return { prm, world, joined: prm.roadmap.sameComponent(0, 1) };
}

interface Tally {
  ok: number;
  total: number;
}

interface State {
  n: number;
  /** tally[strategy][widthIndex] */
  tally: Record<Strategy, Tally[]>;
  cursor: number;
  jobs: { s: Strategy; wi: number; seed: number }[];
}

function freshTally(): Record<Strategy, Tally[]> {
  const t = {} as Record<Strategy, Tally[]>;
  for (const s of STRATEGIES) t[s.key] = WIDTHS.map(() => ({ ok: 0, total: 0 }));
  return t;
}

export function NarrowPassage() {
  const [wIdx, setWIdx] = useState(3);
  const [n, setN] = useState(150);
  const [active, setActive] = useState<Strategy>('uniform');
  const [enabled, setEnabled] = useState<Record<Strategy, boolean>>({ uniform: true, gaussian: true, bridge: true, obprm: false, 'medial-axis': false });

  const init = useCallback(
    (seed: number): State => {
      // Round-robin over seeds so every curve fills in together.
      const jobs: State['jobs'] = [];
      for (let k = 0; k < SEEDS; k++) for (let wi = 0; wi < WIDTHS.length; wi++) for (const s of STRATEGIES) jobs.push({ s: s.key, wi, seed: seed * 1000 + k });
      return { n, tally: freshTally(), cursor: 0, jobs };
    },
    [n],
  );

  const enabledRef = useRef(enabled);
  enabledRef.current = enabled;

  const stepFn = useCallback((s: State): State => {
    let done = 0;
    while (done < JOBS_PER_TICK && s.cursor < s.jobs.length) {
      const job = s.jobs[s.cursor++];
      if (!enabledRef.current[job.s]) continue;
      const r = buildOnce(job.s, WIDTHS[job.wi], s.n, job.seed);
      const t = s.tally[job.s][job.wi];
      t.total++;
      if (r.joined) t.ok++;
      done++;
    }
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 7 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [n]);

  const st = sim.state;
  const w = WIDTHS[wIdx];
  // The cloud: one roadmap for the active strategy at the selected width, seed 0 of this run.
  const cloud = useMemo(() => buildOnce(active, w, n, sim.seed * 1000), [active, w, n, sim.seed]);
  const finished = st.cursor >= st.jobs.length;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const bw = cloud.world;
      fillPolygons(ctx, v, polygonsOf(bw), p.obstacle, 1);
      ctx.save();
      ctx.strokeStyle = p.wall;
      ctx.lineWidth = 2;
      ctx.strokeRect(sx(v, bw.bounds.minX), sy(v, bw.bounds.maxY), sl(v, bw.bounds.maxX - bw.bounds.minX), sl(v, bw.bounds.maxY - bw.bounds.minY));
      ctx.restore();
      const nodes = cloud.prm.roadmap.nodes;
      const adj = cloud.prm.roadmap.adj;
      ctx.save();
      ctx.strokeStyle = p.graph;
      ctx.globalAlpha = 0.35;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let a = 0; a < nodes.length; a++) {
        for (const e of adj[a]) {
          if (e.to <= a) continue;
          ctx.moveTo(sx(v, nodes[a][0]), sy(v, nodes[a][1]));
          ctx.lineTo(sx(v, nodes[e.to][0]), sy(v, nodes[e.to][1]));
        }
      }
      ctx.stroke();
      ctx.restore();
      for (let i = 2; i < nodes.length; i++) paintDot(ctx, sx(v, nodes[i][0]), sy(v, nodes[i][1]), 2.2, p.graph, 0.9);
      if (cloud.joined) {
        const sp = cloud.prm.roadmap.shortestPath(0, 1);
        if (sp) {
          const pts = cloud.prm.roadmap.pathPoints(sp.path);
          ctx.save();
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 3;
          ctx.lineJoin = 'round';
          ctx.beginPath();
          pts.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q[0]), sy(v, q[1])) : ctx.lineTo(sx(v, q[0]), sy(v, q[1]))));
          ctx.stroke();
          ctx.restore();
        }
      }
      const a = startOf(bw);
      const b = goalOf(bw);
      paintDot(ctx, sx(v, a[0]), sy(v, a[1]), 7, p.start);
      paintDot(ctx, sx(v, b[0]), sy(v, b[1]), 7, p.goal);
      const sm = cloud.prm.opts.sampler;
      label(
        ctx,
        `${STRATEGIES.find((x) => x.key === active)!.label}: ${nodes.length - 2} nodes from ${sm.attempts} attempts · w = ${w.toFixed(2)} m · ${cloud.joined ? 'joined' : 'not joined'}`,
        sx(v, 0.15),
        sy(v, bw.bounds.maxY) - 10,
        cloud.joined ? p.path : p.ink,
        { size: 11, weight: 600 },
      );
    },
    [cloud, active, w],
  );

  const series = useMemo(
    () =>
      STRATEGIES.filter((s) => enabled[s.key]).map((s) => ({
        id: s.label,
        role: s.role,
        data: WIDTHS.map((ww, i) => ({ ww, t: st.tally[s.key][i] }))
          .filter((d) => d.t.total > 0)
          .map((d) => ({ x: d.ww / W, y: d.t.ok / d.t.total })),
      })),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [enabled, Math.floor(sim.tick / 4), st.tally],
  );

  const tally = st.tally[active][wIdx];
  const eps = corridorExpansiveness(w, W);
  const need = expansiveSampleCount(eps, eps, eps, 0.05);

  return (
    <WidgetFrame
      id="w11.2"
      title="Narrow Passage"
      teaches="Uniform sampling does not see everything at practical sample counts — and biased sampling is not always better: the Gaussian sampler loses to uniform once the corridor is wide."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Two {W} m rooms and a corridor of width <em>w</em> (Choset figure 7.19). For each enabled
          strategy, {SEEDS} seeded roadmaps of {n} samples are built at every width and the fraction
          that joined the rooms is plotted against <em>w/W</em>, which figure 7.19 says is also the
          space&rsquo;s ε, α and β. The canvas shows the active strategy&rsquo;s sample cloud at the
          selected width: uniform rain falls everywhere; the Gaussian sampler hugs every wall,
          including the useless ones; the bridge test lands only where two obstacles are close —
          inside the corridor and nowhere else. Slide <em>w</em>, enable strategies, change{' '}
          <em>n</em>, re-roll.
        </>
      }
    >
      <SimCanvas
        world={cloud.world.bounds}
        aspect={3}
        padding={0.3}
        draw={draw}
        deps={[cloud, sim.tick]}
        ariaLabel="Two rooms joined by a narrow corridor with the sample cloud of one roadmap strategy, a path drawn when the rooms are connected."
      />
      <div className="border-t border-fd-border px-3 py-3">
        <div className="grid grid-cols-1 gap-3 lg:grid-cols-[1fr_minmax(0,20rem)]">
          <LineChart
            series={series}
            xLabel="corridor width w / W  (≈ ε, α, β)"
            yLabel="connect fraction"
            height={230}
            yMin={0}
            yMax={1}
            markers={[{ axis: 'x', value: w / W, label: `w = ${w.toFixed(2)} m` }]}
            ariaLabel="Fraction of seeded roadmaps that connect the two rooms against corridor width, one curve per sampling strategy."
          />
          <Dashboard columns={2}>
            <StatTile label="ε ≈ w / W" value={eps} precision={3} />
            <StatTile label={`measured rate · ${STRATEGIES.find((x) => x.key === active)!.label}`} value={tally.total ? tally.ok / tally.total : 0} precision={2} role={STRATEGIES.find((x) => x.key === active)!.role} trend={tally.total} trendLabel={`of ${SEEDS} seeds`} />
            <StatTile label="Theorem 7.4.2 asks (ε=α=β=w/W, δ=0.05)" value={need.samples} precision={0} unit="samples" />
            <StatTile label="sweep progress" value={`${Math.round((100 * st.cursor) / st.jobs.length)}%`} trend={finished ? 0 : undefined} trendLabel={finished ? 'done' : undefined} />
          </Dashboard>
        </div>
      </div>
      <ControlPanel columns={3} title="headline — corridor width">
        <Slider label="corridor width w" role="obstacle" value={wIdx} min={0} max={WIDTHS.length - 1} step={1} onChange={setWIdx} format={(i) => `${WIDTHS[i].toFixed(2)} m  (w/W = ${(WIDTHS[i] / W).toFixed(3)})`} />
        <Slider label="samples n per roadmap (restarts the sweep)" role="graph" value={n} min={50} max={600} step={50} onChange={setN} />
        <ButtonRow>
          {STRATEGIES.map((s) => (
            <ActionButton key={s.key} emphasis={active === s.key} onClick={() => setActive(s.key)}>
              cloud: {s.label}
            </ActionButton>
          ))}
        </ButtonRow>
      </ControlPanel>
      <ControlPanel columns={3} title="disclosure — strategies in the sweep">
        {STRATEGIES.map((s) => (
          <Toggle key={s.key} label={s.label} role={s.role} checked={enabled[s.key]} onChange={(on) => setEnabled((e) => ({ ...e, [s.key]: on }))} />
        ))}
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
