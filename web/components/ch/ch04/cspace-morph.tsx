'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawSegments, label, sl, sx, sy, type Palette, type Viewport, readPalette } from '@/lib/sim/draw';
import { normalizeAngle } from '@/lib/geom/se2';
import { Rng } from '@/lib/prob/rng';
import type { Point2 } from '@/lib/sim/world';
import { Reach, t2, type T2 } from '@/lib/robots/reach';
import { WORKBENCH } from '@/lib/robots/worlds';
import { rect, type Polygon } from '@/lib/collide/polygon';
import { Scene } from '@/lib/collide/scene';
import { ChainChecker } from '@/lib/collide/collision';
import { RasterJob, torusChart, type BitGrid, type Chart2 } from '@/lib/cspace/raster';
import { LAB_CLAMP, REACH_SQUARE } from '@/lib/cspace/examples';
import { deg, dot, drawRaster, drawReach, drawTorusAxes, fillPoly, fmt, rasterImage, strokePoly } from './shared';

/**
 * w4.1 — C-Space Morph: the book's signature figure.
 *
 * Left, the Workbench with Reach and one draggable slate block. Right, the
 * unwrapped torus [−π, π]² with the rasterized C-obstacle in hatched amber and
 * the current configuration as an orange dot. The block slides on a seeded
 * path until you grab it; the raster is re-evaluated row by row under a cell
 * budget so the picture keeps up without stalling the frame.
 *
 * The misconception killed: "the C-obstacle looks like the obstacle". A
 * square in the workspace is a blob on a torus; move the block toward the
 * base and the blob grows a vertical band (link 1 hits it for every θ₂) and
 * wraps across the seam; switch on the post and two blobs merge into a wall
 * across θ₁ — Q_free disconnects, Choset's figure 3.7. The micro-example
 * preset reproduces the 29-cell row the prose derives by hand.
 */

const TORUS = { minX: -Math.PI, minY: -Math.PI, maxX: Math.PI, maxY: Math.PI };
const CELL_BUDGET = 3000;
const TRAIL = 140;

interface MorphState {
  t: number;
  q: T2;
  block: Point2;
  trail: T2[];
  /** Seeded path parameters. */
  path: { bx: number; by: number; ax: number; ay: number; wx: number; wy: number; px: number; py: number; qa: number; qb: number; qw: number; qp: number };
}

interface Params {
  size: number;
  post: boolean;
  /** The integration lab's extra obstacles: the shelf and a clamp within link 1's reach. */
  lab: boolean;
  lengths: [number, number];
  linkRadius: number;
  degPerCell: number;
}

const MICRO: Params = { size: 0.2, post: false, lab: false, lengths: [1, 1], linkRadius: 0, degPerCell: 1 };
const BENCH: Params = { size: 0.5, post: true, lab: false, lengths: [1, 1], linkRadius: 0.03, degPerCell: 2 };
/** The integration lab: block slid to (0.75, 0.35), post, shelf and clamp — two bands cut the torus in two. */
const LAB: Params = { size: 0.4, post: true, lab: true, lengths: [1, 1], linkRadius: 0.03, degPerCell: 2 };
type Preset = 'bench' | 'micro' | 'lab' | null;

const blockPoly = (c: Point2, size: number): Polygon => rect(c.x - size / 2, c.y - size / 2, c.x + size / 2, c.y + size / 2);

export function CSpaceMorph() {
  const [params, setParams] = useState<Params>(BENCH);
  const [preset, setPreset] = useState<Preset>('bench');

  // Reader input lives in refs so the clock reads it without re-rendering per pointer event.
  const takenBlock = useRef(false);
  const takenQ = useRef(false);
  const dragging = useRef<'block' | 'q' | null>(null);
  const manualBlock = useRef<Point2 | null>(null);
  const manualQ = useRef<T2 | null>(null);

  const arm = useMemo(() => new Reach({ base: { x: 0, y: 0 }, lengths: params.lengths, linkRadius: params.linkRadius }), [params.lengths, params.linkRadius]);
  const chart = useMemo(() => torusChart(Math.round(360 / params.degPerCell)), [params.degPerCell]);

  // Raster bookkeeping: the job in flight, the last complete grid, and its image.
  const job = useRef<{ key: string; job: RasterJob<T2>; chart: Chart2<T2> } | null>(null);
  const shown = useRef<{ key: string; grid: BitGrid; img: HTMLCanvasElement | null; chart: Chart2<T2>; comps: number; freeComps: number; row0: number; wraps: boolean } | null>(null);

  const sceneFor = useCallback(
    (block: Point2) => {
      let scene = Scene.ofWalls([], WORKBENCH.bounds).withObstacle('the block', blockPoly(block, params.size));
      if (params.post) scene = scene.withObstacle('the post', WORKBENCH.obstacles[1].poly);
      if (params.lab) scene = scene.withObstacle('the shelf', WORKBENCH.obstacles[2].poly).withObstacle('the clamp', LAB_CLAMP);
      return scene;
    },
    [params.size, params.post, params.lab],
  );

  const init = useCallback((seed: number): MorphState => {
    const rng = new Rng(seed);
    takenBlock.current = false;
    takenQ.current = false;
    manualBlock.current = null;
    manualQ.current = null;
    const path = {
      bx: rng.uniform(0.6, 1.1),
      by: rng.uniform(0.4, 1.0),
      ax: rng.uniform(0.5, 0.8),
      ay: rng.uniform(0.5, 0.9),
      wx: rng.uniform(0.05, 0.09),
      wy: rng.uniform(0.04, 0.08),
      px: rng.uniform(0, 2 * Math.PI),
      py: rng.uniform(0, 2 * Math.PI),
      qa: rng.uniform(1.5, 2.8),
      qb: rng.uniform(1.0, 2.6),
      qw: rng.uniform(0.3, 0.6),
      qp: rng.uniform(0, 2 * Math.PI),
    };
    const block = { x: path.bx + path.ax * Math.cos(path.px), y: path.by + path.ay * Math.sin(path.py) };
    return { t: 0, q: t2(0.6, 1.2), block, trail: [], path };
  }, []);

  const step = useCallback(
    (s: MorphState): MorphState => {
      const t = s.t + 1 / 30;
      const P = s.path;
      const block = takenBlock.current && manualBlock.current ? manualBlock.current : { x: P.bx + P.ax * Math.cos(P.wx * t + P.px), y: P.by + P.ay * Math.sin(P.wy * t + P.py) };
      const q: T2 =
        takenQ.current && manualQ.current ? manualQ.current : t2(P.qa * Math.sin(P.qw * t + P.qp), P.qb * Math.sin(0.61 * P.qw * t + 1.3 * P.qp));
      const trail = [...s.trail, q];
      if (trail.length > TRAIL) trail.shift();

      // Keep the raster honest: a new scene or chart starts a new job; otherwise advance under budget.
      const key = `${block.x.toFixed(3)},${block.y.toFixed(3)},${params.size},${params.post},${params.lab},${params.lengths.join()},${params.linkRadius},${chart.n1}`;
      let j = job.current;
      if (!j || j.key !== key) {
        // ChainChecker is a Collision<Tn>; the torus chart hands it T² points, so pin Q = T2 explicitly.
        j = { key, job: new RasterJob<T2>(new ChainChecker(sceneFor(block), arm), chart), chart };
        job.current = j;
      }
      if (!j.job.done) j.job.advance(CELL_BUDGET);
      if (j.job.done && (!shown.current || shown.current.key !== j.key)) {
        const grid = j.job.grid;
        const occ = grid.components(true, chart.periodic);
        const free = grid.components(false, chart.periodic);
        shown.current = {
          key: j.key,
          grid,
          img: rasterImage(grid, lastColor.current),
          chart: j.chart,
          comps: occ.count,
          freeComps: free.count,
          row0: grid.rowCount(Math.round(chart.n1 / 2)),
          wraps: grid.touchesSeam(),
        };
      }
      return { t, q, block, trail, path: s.path };
    },
    [arm, chart, params, sceneFor],
  );

  const sim = useSimulation<MorphState>({ init, step, fps: 30, initialSeed: 4 });
  const { reset } = sim;
  useEffect(() => {
    reset();
    job.current = null;
    shown.current = null;
  }, [params, reset]);

  // The palette's amber, remembered so the raster image can be built off the clock.
  const lastColor = useRef(readPalette().cobstacle);

  const s = sim.state;
  const checker = useMemo(() => new ChainChecker(sceneFor(s.block), arm), [sceneFor, s.block, arm]);
  const inCollision = !checker.isFree(s.q);

  // ---- pointer ---------------------------------------------------------------
  const onWorkbench = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') {
        dragging.current = null;
        return;
      }
      if (phase === 'down') {
        const b = sim.state.block;
        if (Math.hypot(w[0] - b.x, w[1] - b.y) < params.size / 2 + 0.15) {
          dragging.current = 'block';
          takenBlock.current = true;
          manualBlock.current = b;
        }
        return;
      }
      if (dragging.current === 'block') {
        const r = Math.hypot(w[0], w[1]);
        const lim = arm.reach() + params.size;
        const k = r > lim ? lim / r : 1;
        manualBlock.current = { x: w[0] * k, y: w[1] * k };
      }
    },
    [sim.state.block, params.size, arm],
  );

  const onTorus = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') {
        dragging.current = null;
        return;
      }
      if (phase === 'down') {
        dragging.current = 'q';
        takenQ.current = true;
      }
      if (dragging.current === 'q') manualQ.current = t2(w[0], w[1]);
    },
    [],
  );

  // ---- drawing -----------------------------------------------------------------
  const drawBench = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      lastColor.current = p.cobstacle;
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 0.5);
      drawSegments(ctx, v, WORKBENCH.walls, p.wall, 2);
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.arc(sx(v, 0), sy(v, 0), sl(v, arm.reach()), 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
      if (params.post) fillPoly(ctx, v, WORKBENCH.obstacles[1].poly, p.obstacle);
      if (params.lab) {
        fillPoly(ctx, v, WORKBENCH.obstacles[2].poly, p.obstacle);
        fillPoly(ctx, v, LAB_CLAMP, p.obstacle);
        label(ctx, 'clamp', sx(v, -0.55), sy(v, 0.75) - 4, p.obstacle, { size: 9, align: 'center' });
      }
      const block = blockPoly(s.block, params.size);
      fillPoly(ctx, v, block, p.obstacle);
      strokePoly(ctx, v, block, p.obstacle, 2);
      // Tip trail.
      ctx.save();
      ctx.strokeStyle = p.robot;
      ctx.globalAlpha = 0.45;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      s.trail.forEach((q, i) => {
        const tip = arm.tip(q);
        if (i === 0) ctx.moveTo(sx(v, tip.x), sy(v, tip.y));
        else ctx.lineTo(sx(v, tip.x), sy(v, tip.y));
      });
      ctx.stroke();
      ctx.restore();
      drawReach(ctx, v, p, arm, s.q, inCollision ? p.goal : p.robot);
      label(ctx, 'drag the block', sx(v, s.block.x), sy(v, s.block.y + params.size / 2) - 8, p.obstacle, { size: 9, align: 'center' });
      label(ctx, `block centre (${fmt(s.block.x, 2)}, ${fmt(s.block.y, 2)})`, 10, 14, p.ink, { size: 10 });
    },
    [arm, params, s, inCollision],
  );

  const drawTorus = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const sh = shown.current;
      if (sh?.img) drawRaster(ctx, v, sh.img, sh.chart);
      drawTorusAxes(ctx, v, p);
      // Progress of the job in flight: a sweep line.
      const j = job.current;
      if (j && !j.job.done) {
        const x = sx(v, -Math.PI + j.job.rowsDone * j.chart.d1);
        ctx.save();
        ctx.strokeStyle = p.cobstacle;
        ctx.lineWidth = 1.5;
        ctx.globalAlpha = 0.8;
        ctx.beginPath();
        ctx.moveTo(x, sy(v, -Math.PI));
        ctx.lineTo(x, sy(v, Math.PI));
        ctx.stroke();
        ctx.restore();
      }
      // The trail on the torus, broken where it crosses a seam.
      ctx.save();
      ctx.strokeStyle = p.robot;
      ctx.globalAlpha = 0.5;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      let prev: T2 | null = null;
      for (const q of s.trail) {
        if (!prev || Math.abs(q[0] - prev[0]) > Math.PI || Math.abs(q[1] - prev[1]) > Math.PI) ctx.moveTo(sx(v, q[0]), sy(v, q[1]));
        else ctx.lineTo(sx(v, q[0]), sy(v, q[1]));
        prev = q;
      }
      ctx.stroke();
      ctx.restore();
      dot(ctx, sx(v, s.q[0]), sy(v, s.q[1]), 5.5, inCollision ? p.goal : p.robot, p.bg);
      label(ctx, `q = (${fmt(deg(s.q[0]), 1)}°, ${fmt(deg(s.q[1]), 1)}°)  ${inCollision ? 'in QO' : 'free'}`, sx(v, -Math.PI), sy(v, Math.PI) - 12, inCollision ? p.goal : p.ink, { size: 10, weight: 600 });
    },
    [s, inCollision],
  );

  const sh = shown.current;
  const total = chart.n1 * chart.n2;

  const apply = (next: Params, which: Preset) => {
    setParams(next);
    setPreset(which);
    takenBlock.current = true;
    manualBlock.current = which === 'micro' ? { x: 1.5, y: 0 } : which === 'bench' ? { x: 0.75, y: 1.25 } : which === 'lab' ? { x: 0.75, y: 0.35 } : manualBlock.current;
  };
  const change = (patch: Partial<Params>) => apply({ ...params, ...patch }, null);

  return (
    <WidgetFrame
      id="w4.1"
      title="C-Space Morph"
      teaches="A C-obstacle is not a picture of the obstacle: it is the set of configurations where R(q) meets it — a blob on a torus that moves the other way."
      colorKey={['obstacle', 'cobstacle', 'robot', 'goal']}
      wide
      caption={
        <>
          Left, the Workbench: Reach and one slate block sliding on a seeded path (grab it to take over).
          Right, the unwrapped torus with every configuration whose links touch the block hatched amber,
          re-rasterized row by row as the block moves; the sweep line is the row being evaluated. The
          orange dot is the current <em>q</em> in both panes — drag it on the torus and the arm follows.
          Slide the block toward the base and watch a vertical band appear (link 1 now hits it for every
          θ₂) and wrap across the seam; switch on the post and the two blobs fuse into a wall across θ₁,
          splitting the free space in two when a second band appears (the lab preset). The micro-example
          preset is the prose's square: its θ₁ = 0° row has exactly 29 occupied cells.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border md:grid-cols-2">
        <div className="bg-fd-card">
          <div className="eyebrow px-2 py-1">workspace · the Workbench</div>
          <SimCanvas world={{ minX: -2.4, minY: -2.4, maxX: 2.4, maxY: 2.4 }} draw={drawBench} deps={[sim.tick, params]} aspect={1} padding={0.1} onPointer={onWorkbench} cursor={dragging.current === 'block' ? 'grabbing' : 'grab'} ariaLabel="Reach, a two-link orange arm, on a square table with a draggable slate block" />
        </div>
        <div className="bg-fd-card">
          <div className="eyebrow px-2 py-1">configuration space · T² unwrapped</div>
          <SimCanvas world={TORUS} draw={drawTorus} deps={[sim.tick, params]} aspect={1} padding={0.45} onPointer={onTorus} cursor={dragging.current === 'q' ? 'grabbing' : 'crosshair'} ariaLabel="The unwrapped torus of joint angles with the C-obstacle hatched in amber and the current configuration as an orange dot" />
        </div>
      </div>

      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border font-mono text-[0.7rem] tabular-nums sm:grid-cols-4">
        <Readout title="occupied cells" value={sh ? `${sh.grid.count().toLocaleString()} / ${total.toLocaleString()}` : '…'} sub={sh ? `${((100 * sh.grid.count()) / total).toFixed(2)} % of T²` : 'rasterizing'} />
        <Readout title="θ₁ = 0° row" value={sh ? `${sh.row0} cells` : '…'} sub={`${params.degPerCell}° per cell`} />
        <Readout title="components" value={sh ? `QO: ${sh.comps}   Q_free: ${sh.freeComps}` : '…'} sub={sh ? (sh.wraps ? 'blob crosses a seam' : 'blob does not wrap') : ''} />
        <Readout title="raster" value={job.current && !job.current.job.done ? `${(100 * job.current.job.progress).toFixed(0)} %` : 'complete'} sub={`${CELL_BUDGET.toLocaleString()} queries per frame`} />
      </div>

      <ControlPanel columns={3} title="Headline: drag the block · presets">
        <ButtonRow>
          <ActionButton emphasis={preset === 'bench'} onClick={() => apply(BENCH, 'bench')}>
            Workbench block + post
          </ActionButton>
          <ActionButton emphasis={preset === 'micro'} onClick={() => apply(MICRO, 'micro')}>
            micro-example square
          </ActionButton>
          <ActionButton emphasis={preset === 'lab'} onClick={() => apply(LAB, 'lab')}>
            lab: two bands cut the torus
          </ActionButton>
        </ButtonRow>
        <Slider label="block side" role="obstacle" value={params.size} min={0.1} max={0.9} step={0.05} unit="m" onChange={(v) => change({ size: v })} />
        <div className="flex flex-col gap-2">
          <Toggle label="second obstacle: the post" role="obstacle" checked={params.post} onChange={(v) => change({ post: v })} />
          <Toggle label="lab obstacles: the shelf and a clamp near the base" role="obstacle" checked={params.lab} onChange={(v) => change({ lab: v })} />
        </div>
      </ControlPanel>

      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Advanced: link lengths, thickness, raster resolution</summary>
        <ControlPanel columns={2}>
          <Slider label="L₁" role="robot" value={params.lengths[0]} min={0.5} max={1.4} step={0.05} unit="m" onChange={(v) => change({ lengths: [v, params.lengths[1]] })} />
          <Slider label="L₂" role="robot" value={params.lengths[1]} min={0.5} max={1.4} step={0.05} unit="m" onChange={(v) => change({ lengths: [params.lengths[0], v] })} />
          <Slider label="link thickness (0 = exact segments)" role="robot" value={params.linkRadius} min={0} max={0.08} step={0.005} unit="m" onChange={(v) => change({ linkRadius: v })} />
          <Slider label="raster cell" role="cobstacle" value={params.degPerCell} min={0.5} max={4} step={0.5} unit="°" onChange={(v) => change({ degPerCell: v })} />
        </ControlPanel>
      </details>

      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}

function Readout({ title, value, sub }: { title: string; value: string; sub: string }) {
  return (
    <div className="px-3 py-2">
      <div className="eyebrow">{title}</div>
      <div className="mt-0.5 text-fd-foreground">{value}</div>
      <div className="mt-0.5 font-ui text-[0.68rem] text-fd-muted-foreground">{sub}</div>
    </div>
  );
}

export { REACH_SQUARE };
