'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { BarChart, Dashboard, LineChart, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { BitGrid } from '@/lib/manifold/torus';
import { Prm, StraightLine, Uniform, chartPanel, paintArmMapped, paintChartPath, paintDot, paintGeodesic, paintPanelFrame, paintPolyWorld, paintRaster, rasterTorusCObstacle, rectMapper, torusLab, type ChartPanel, type Sampler, type TorusLab } from '@/lib/sampling';
import { CELLS_N, LearnedSampler, connectedQuery, loadModels, makeWorldCase, mixtureSampler, randomQuery, type WorldCase, type WorldGenerator } from '@/lib/learn';

/**
 * w22.1 — Learned Sampler vs Uniform.
 *
 * Two Chapter 11 PRMs grow side by side on the same randomized Workbench with
 * the same seed and the same query. Left: the uniform proposal. Middle: the
 * learned proposal p̂_θ(· | 𝓔, q_start, q_goal), drawn as a monochrome heat
 * layer under the amber C-obstacle raster, mixed with uniform at weight λ so
 * Derivation 1 still applies. Right: the Workbench, with Reach walking whichever
 * path is found first. The planner, the local planner and the collision
 * checker are identical on both sides — only the proposal differs — and the
 * success-at-budget curves below are accumulated over further seeds in the
 * background, one pair at a time. Budgets are in collision checks, the book's
 * cost unit: on Reach's torus a query joins within a handful of nodes, so the
 * node count hides what the proposal saves and the check count shows it.
 */

const N_MAX = 1500;
const BATCH = 4;
const ARENA = { minX: 0, minY: 0, maxX: 3.3, maxY: 1 };
const PANEL_U = { x0: 0, y0: 0, x1: 1, y1: 1 };
const PANEL_L = { x0: 1.12, y0: 0, x1: 2.12, y1: 1 };
const PANEL_W = { x0: 2.24, y0: 0, x1: 3.3, y1: 1 };
const CURVE_C = [25, 50, 100, 150, 200, 300, 400, 600, 800, 1200, 1600, 2400, 3200];
const EXTRA_SEEDS = 10;

interface Lane {
  prm: Prm<number[]>;
  lab: TorusLab;
  joinedAt: number | null;
  /** Collision checks spent when start and goal first shared a component. */
  joinedChecks: number | null;
  path: number[][] | null;
}

interface Pair {
  uniform: Lane;
  learned: Lane;
  learnedSampler: LearnedSampler;
}

interface State {
  wc: WorldCase;
  start: number[];
  goal: number[];
  raster: BitGrid;
  live: Pair;
  /** Background pairs on further seeds, for the success curves. */
  extra: Pair | null;
  extraSeed: number;
  extraDone: number;
  /** Checks at which each background pair's lanes joined (null: not within N_MAX nodes). */
  results: { u: number | null; l: number | null }[];
}

interface Params {
  gen: WorldGenerator;
  lambda: number;
}

function makeLane(wc: WorldCase, start: number[], goal: number[], sampler: Sampler<number[]>, seed: number): Lane {
  // Each lane gets its own lab so its collision-check counter is its own.
  const lab = torusLab(wc.world);
  const prm = new Prm<number[]>(lab.space, lab.free, { sampler, steer: new StraightLine<number[]>({ kind: 'subdivision', step: 0.2 }), connect: { kind: 'kNearest', k: 6 } }, new Rng(seed));
  const s = prm.addNode(start);
  prm.connectNode(s, prm.candidates(s));
  const g = prm.addNode(goal);
  prm.connectNode(g, prm.candidates(g));
  const joined = prm.roadmap.sameComponent(0, 1);
  return { prm, lab, joinedAt: joined ? 2 : null, joinedChecks: joined ? lab.free.checks : null, path: null };
}

function makePair(wc: WorldCase, start: number[], goal: number[], lambda: number, seed: number): Pair {
  const ls = new LearnedSampler(loadModels().sampler, wc.enc, start, goal);
  return {
    uniform: makeLane(wc, start, goal, new Uniform<number[]>(), seed),
    learned: makeLane(wc, start, goal, mixtureSampler(ls, lambda), seed),
    learnedSampler: ls,
  };
}

/** A blocked query whose endpoints share a component of the C-space raster, so the race has a finish line. */
function queryFor(wc: WorldCase, seed: number, raster: BitGrid): [number[], number[]] {
  const rng = new Rng(seed ^ 0x5bd1e995);
  return connectedQuery(wc.lab, raster, rng) ?? randomQuery(wc.lab, rng) ?? randomQuery(wc.lab, rng, 1.5, false) ?? [[0.8, 1.6], [-2.2, -1.2]];
}

function build(seed: number, prms: Params): State {
  const wc = makeWorldCase(seed, prms.gen);
  const raster = rasterTorusCObstacle(wc.lab, 48);
  const [start, goal] = queryFor(wc, seed, raster);
  return {
    wc,
    start,
    goal,
    raster,
    live: makePair(wc, start, goal, prms.lambda, seed),
    extra: null,
    extraSeed: seed + 1,
    extraDone: 0,
    results: [],
  };
}

/** Grow a lane by up to `batch` nodes toward `want`; record the join. */
function growLane(lane: Lane, want: number, batch: number): void {
  let added = 0;
  let attempts = 0;
  while (added < batch && lane.prm.n < want && attempts++ < 200) {
    if (lane.prm.addSample() !== null) {
      added++;
      if (lane.joinedAt === null && lane.prm.roadmap.sameComponent(0, 1)) {
        lane.joinedAt = lane.prm.n;
        lane.joinedChecks = lane.lab.free.checks;
      }
    }
  }
}

export function LearnedSamplerRace() {
  const [nTarget, setNTarget] = useState(600);
  const [lambda, setLambda] = useState(0.1);
  const [gen, setGen] = useState<WorldGenerator>('train');

  const params = useMemo<Params>(() => ({ gen, lambda }), [gen, lambda]);
  const init = useCallback((seed: number) => build(seed, params), [params]);
  const targetRef = useRef(nTarget);
  targetRef.current = nTarget;
  const lambdaRef = useRef(lambda);
  lambdaRef.current = lambda;

  const stepFn = useCallback((s: State): State => {
    const want = Math.min(targetRef.current, N_MAX);
    const liveBusy = s.live.uniform.prm.n < want || s.live.learned.prm.n < want;
    if (liveBusy) {
      growLane(s.live.uniform, want, BATCH);
      growLane(s.live.learned, want, BATCH);
      return { ...s };
    }
    // Background: further seeds, same world generator, for the success curves.
    if (s.extraDone >= EXTRA_SEEDS) return s;
    if (!s.extra) {
      const wc = makeWorldCase(s.extraSeed, s.wc.gen);
      const [a, b] = queryFor(wc, s.extraSeed, rasterTorusCObstacle(wc.lab, 48));
      s.extra = makePair(wc, a, b, lambdaRef.current, s.extraSeed);
    }
    const ex = s.extra;
    growLane(ex.uniform, N_MAX, BATCH * 3);
    growLane(ex.learned, N_MAX, BATCH * 3);
    const uDone = ex.uniform.joinedAt !== null || ex.uniform.prm.n >= N_MAX;
    const lDone = ex.learned.joinedAt !== null || ex.learned.prm.n >= N_MAX;
    if (uDone && lDone) {
      s.results.push({ u: ex.uniform.joinedChecks, l: ex.learned.joinedChecks });
      s.extra = null;
      s.extraSeed++;
      s.extraDone++;
    }
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 21 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [params]);

  const s = sim.state;
  const shownU = Math.min(s.live.uniform.prm.n, nTarget);
  const shownL = Math.min(s.live.learned.prm.n, nTarget);
  const space = s.wc.lab.space;

  const pathOf = (lane: Lane, shown: number): number[][] | null => {
    if (lane.joinedAt === null || shown < lane.joinedAt) return null;
    const sp = lane.prm.roadmap.shortestPath(0, 1, shown);
    return sp ? lane.prm.roadmap.pathPoints(sp.path) : null;
  };
  const pathU = useMemo(() => pathOf(s.live.uniform, shownU), [s.live.uniform, shownU, s.live.uniform.joinedAt]);
  const pathL = useMemo(() => pathOf(s.live.learned, shownL), [s.live.learned, shownL, s.live.learned.joinedAt]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const panelU = chartPanel(v, space, PANEL_U);
      const panelL = chartPanel(v, space, PANEL_L);

      const drawLane = (panel: ChartPanel, lane: Lane, shown: number, path: number[][] | null, title: string, heat: LearnedSampler | null) => {
        if (heat) {
          // Monochrome heat under everything: cell mass relative to the largest cell.
          const n = CELLS_N;
          const cw = panel.w / n;
          const ch = panel.h / n;
          let max = 0;
          for (let c = 0; c < heat.probs.length; c++) max = Math.max(max, heat.probs[c]);
          ctx.save();
          ctx.fillStyle = p.ink;
          for (let j = 0; j < n; j++) {
            for (let i = 0; i < n; i++) {
              const a = heat.probs[j * n + i] / max;
              if (a < 0.02) continue;
              ctx.globalAlpha = 0.04 + 0.38 * a;
              ctx.fillRect(panel.px + i * cw, panel.py + panel.h - (j + 1) * ch, cw + 0.5, ch + 0.5);
            }
          }
          ctx.restore();
        }
        paintRaster(ctx, panel, s.raster, p.cobstacle, 0.7);
        paintPanelFrame(ctx, panel, p, true);
        const nodes = lane.prm.roadmap.nodes;
        const adj = lane.prm.roadmap.adj;
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.lineWidth = 0.8;
        ctx.globalAlpha = 0.5;
        ctx.beginPath();
        for (let a = 0; a < shown; a++) for (const e of adj[a]) if (e.to > a && e.to < shown) paintGeodesic(ctx, panel, space, nodes[a], nodes[e.to], 3);
        ctx.stroke();
        ctx.restore();
        for (let i = 2; i < shown; i++) {
          const c = space.coords(nodes[i]);
          paintDot(ctx, panel.X(c[0]), panel.Y(c[1]), 1.6, p.graph, 0.9);
        }
        if (path) paintChartPath(ctx, panel, space, path, p.path, 3);
        const cs = space.coords(s.start);
        const cg = space.coords(s.goal);
        paintDot(ctx, panel.X(cs[0]), panel.Y(cs[1]), 6, p.start);
        paintDot(ctx, panel.X(cg[0]), panel.Y(cg[1]), 6, p.goal);
        label(ctx, title, panel.px + 4, panel.py - 9, p.ink, { size: 11, weight: 600 });
        label(ctx, `n = ${shown}`, panel.px + panel.w - 4, panel.py - 9, p.ink, { size: 10, align: 'right' });
        if (lane.joinedAt !== null && shown >= lane.joinedAt) label(ctx, `joined at n = ${lane.joinedAt} · ${lane.joinedChecks} checks`, panel.px + 4, panel.py + panel.h + 11, p.path, { size: 11, weight: 700 });
        else label(ctx, `${lane.lab.free.checks} checks`, panel.px + 4, panel.py + panel.h + 11, p.ink, { size: 10 });
      };

      drawLane(panelU, s.live.uniform, shownU, pathU, 'uniform proposal', null);
      drawLane(panelL, s.live.learned, shownL, pathL, `learned proposal, λ = ${lambda.toFixed(2)}`, s.live.learnedSampler);

      // The Workbench: start and goal ghosts, and the arm walking the first path found.
      const m = rectMapper(v, PANEL_W, s.wc.world.bounds);
      paintPolyWorld(ctx, m, s.wc.world, p);
      paintArmMapped(ctx, m, p, s.wc.lab.arm, s.start, p.start, 0.55);
      paintArmMapped(ctx, m, p, s.wc.lab.arm, s.goal, p.goal, 0.55);
      const walk = pathL ?? pathU;
      if (walk && walk.length > 1) {
        let L = 0;
        const seg: number[] = [0];
        for (let i = 1; i < walk.length; i++) {
          L += space.dist(walk[i - 1], walk[i]);
          seg.push(L);
        }
        const t = ((sim.tick * 0.02) % 1.2) * L;
        let i = 1;
        while (i < seg.length - 1 && seg[i] < t) i++;
        const q = t >= L ? walk[walk.length - 1] : space.interpolate(walk[i - 1], walk[i], (t - seg[i - 1]) / Math.max(1e-9, seg[i] - seg[i - 1]));
        paintArmMapped(ctx, m, p, s.wc.lab.arm, q, p.robot, 1);
      }
      label(ctx, s.wc.gen === 'train' ? 'Workbench · training generator' : 'Workbench · held-out generator', m.X(s.wc.world.bounds.minX) + 4, m.Y(s.wc.world.bounds.maxY) - 9, p.ink, { size: 10, weight: 600 });
    },
    [s, shownU, shownL, pathU, pathL, space, lambda, sim.tick],
  );

  // Success at budget (collision checks): live pair plus the finished background pairs.
  const curves = useMemo(() => {
    const all = [{ u: s.live.uniform.joinedChecks, l: s.live.learned.joinedChecks }, ...s.results];
    const frac = (key: 'u' | 'l') => CURVE_C.map((b) => ({ x: b, y: all.filter((r) => r[key] !== null && (r[key] as number) <= b).length / all.length }));
    return { uniform: frac('u'), learned: frac('l'), seeds: all.length };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [s.results.length, s.live.uniform.joinedChecks, s.live.learned.joinedChecks]);

  const quadrants = s.live.learnedSampler.quadrantMasses();
  const klCells = s.live.learnedSampler.klToUniform();
  const klQuad = quadrants.reduce((acc, q) => acc + (q > 0 ? q * Math.log(q * 4) : 0), 0);

  return (
    <WidgetFrame
      id="w22.1"
      title="Learned Sampler vs Uniform"
      teaches="A learned sampler changes where a PRM looks, not what it can find: with λ > 0 both panes connect every solvable query, the learner moves the knee of the success curve — and on a held-out world it moves it the wrong way."
      colorKey={['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Same randomized Workbench, same seed, same query, same local planner and collision checker; only the
          proposal differs. The grey heat under the middle pane is the learned categorical over 12 × 12 cells of
          Reach&rsquo;s torus, conditioned on the table&rsquo;s 8 × 8 occupancy raster and the query&rsquo;s endpoints;
          it is mixed with uniform at weight λ so that Derivation 1&rsquo;s bound still holds. Below: the four-quadrant
          coarsening with its KL divergence from uniform, and success-at-budget curves — the budget in collision checks,
          the book&rsquo;s cost unit — that fill in as further seeds run in the background. Queries are drawn so that start
          and goal share a component of the C-space raster: every one has an answer. Switch to the <em>held-out generator</em> — thin bars at arbitrary angles, a
          family the training set never contained — and watch the heat land where the training worlds had corridors.
        </>
      }
    >
      <SimCanvas
        world={ARENA}
        aspect={3.3}
        padding={0.08}
        draw={draw}
        deps={[sim.tick, s, shownU, shownL, nTarget]}
        ariaLabel="Two probabilistic roadmaps on the unwrapped torus of a two-link arm, left with uniform samples and right with a learned heat layer, and the Workbench beside them."
      />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="uniform: checks to join" value={s.live.uniform.joinedChecks ?? '—'} precision={0} role="graph" trend={s.live.uniform.joinedAt ?? undefined} trendLabel={s.live.uniform.joinedAt !== null ? 'samples n at the join' : undefined} />
          <StatTile label="learned: checks to join" value={s.live.learned.joinedChecks ?? '—'} precision={0} role="path" trend={s.live.learned.joinedAt ?? undefined} trendLabel={s.live.learned.joinedAt !== null ? 'samples n at the join' : undefined} />
          <StatTile label="D(p̂‖u), 144 cells" value={klCells} precision={3} unit="nats" role="robot" trend={Number((klCells / Math.LN2).toFixed(2))} trendLabel="bits" />
          <StatTile label="seeds in the curves" value={curves.seeds} precision={0} role="obstacle" trend={s.live.learnedSampler.inferenceMs} trendLabel="ms inference" />
        </Dashboard>
        <div className="mt-3 grid gap-3 md:grid-cols-2">
          <BarChart
            series={[
              { id: 'learned p̂', role: 'robot', data: quadrants.map((q, i) => ({ x: ['(−,−)', '(+,−)', '(−,+)', '(+,+)'][i], y: q })) },
              { id: 'uniform u', role: 'graph', data: [0, 1, 2, 3].map((i) => ({ x: ['(−,−)', '(+,−)', '(−,+)', '(+,+)'][i], y: 0.25 })) },
            ]}
            xLabel="quadrant of T² (θ₁ sign, θ₂ sign)"
            yLabel="mass"
            height={190}
            legend
            groupMode="grouped"
            valueFormat={(x) => x.toFixed(3)}
            caption={<>Four-cell coarsening of the learned proposal: D(p̂‖u) = {klQuad.toFixed(4)} nats on these 4 cells (the §3(a) computation, live).</>}
            ariaLabel="Bar chart of the learned sampler's mass in the four quadrants of the torus against the uniform quarter each."
          />
          <LineChart
            series={[
              { id: 'uniform', role: 'graph', data: curves.uniform },
              { id: `learned, λ = ${lambda.toFixed(2)}`, role: 'path', data: curves.learned },
            ]}
            xLabel="collision checks"
            yLabel="fraction of seeds joined"
            height={190}
            legend
            curve="stepAfter"
            yMin={0}
            yMax={1}
            caption={<>Success at a collision-check budget over {curves.seeds} seed{curves.seeds === 1 ? '' : 's'} on the {s.wc.gen === 'train' ? 'training' : 'held-out'} generator (more accumulate while the widget runs).</>}
            ariaLabel="Fraction of seeds whose roadmap joined start and goal, against the collision checks spent, for the uniform and the learned proposal."
          />
        </div>
      </div>
      <ControlPanel columns={3} title="headline — sample budget">
        <Slider label="samples n" role="graph" value={nTarget} min={50} max={N_MAX} step={10} onChange={setNTarget} />
        <Slider label="uniform floor λ" role="robot" value={lambda} min={0.02} max={0.5} step={0.02} onChange={setLambda} format={(x) => x.toFixed(2)} help="p_λ = (1 − λ) p̂ + λ u. Derivation 1 needs λ > 0; the slider cannot reach 0, and neither can the constructor." />
        <ButtonRow>
          <ActionButton emphasis={gen === 'train'} onClick={() => setGen('train')}>
            training generator (blocks)
          </ActionButton>
          <ActionButton emphasis={gen === 'heldout'} onClick={() => setGen('heldout')}>
            held-out generator (bars)
          </ActionButton>
        </ButtonRow>
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
