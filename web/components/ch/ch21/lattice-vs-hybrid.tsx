'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Hitch } from '@/lib/robots/hitch';
import { LOT } from '@/lib/robots/worlds';
import { Scene } from '@/lib/collide/scene';
import { Rng } from '@/lib/prob/rng';
import { LOT_REVERSE_GOAL, LOT_REVERSE_START, hybridAStar, stateLatticeSearch, type CarSearchResult, type HeuristicKind, type PrimitiveSet } from '@/lib/nonholonomic';
import { drawCarPath, drawHitch, drawPoseMarker } from './draw-lot';
import { drawSegments, drawWorld } from '@/lib/sim/draw';

/**
 * w21.3 — Lattice vs Hybrid A*. The same Lot and query in two panels: a state
 * lattice (A* over lattice states with lattice-consistent primitives) and
 * hybrid A* (Alg. 22's continuous states, one per cell, plus Reeds–Shepp
 * shots and the dual heuristic). The headline slider sets the resolution; both
 * searches are re-run, then their expansions replayed in order.
 */

const GAP = 1.0;
const W = 16;
const WORLD = { minX: 0, minY: -1.2, maxX: 2 * W + GAP, maxY: 10 };
const CAR = Hitch.car();
const SCENE = Scene.fromWorld(LOT);
/** The seed picks hybrid A*'s heading resolution (72 = Dolgov et al.'s 5°, the checks' default). */
const HEADING_BINS = [72, 64, 80];

interface Params {
  k: number;
  set: PrimitiveSet;
  heuristic: HeuristicKind;
  shotEvery: number;
}

function drawPanel(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, ox: number, res: CarSearchResult, shown: number, title: string, done: boolean) {
  ctx.save();
  ctx.translate(sl(v, ox), 0);
  if (LOT.markings) drawSegments(ctx, v, LOT.markings, p.grid, 2);
  ctx.fillStyle = p.obstacle;
  for (const o of LOT.obstacles) {
    ctx.beginPath();
    o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
    ctx.closePath();
    ctx.fill();
  }
  drawWorld(ctx, v, LOT, p);
  // Expansions: the motion that reached each state (blue) and a dot for the state itself.
  const n = Math.min(shown, res.expanded.length);
  ctx.strokeStyle = p.graph;
  ctx.globalAlpha = 0.45;
  ctx.lineWidth = 1;
  for (let i = 0; i < n; i++) {
    const e = res.expanded[i];
    if (e.sweep.length < 2) continue;
    ctx.beginPath();
    e.sweep.forEach((q, j) => (j === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
    ctx.stroke();
  }
  ctx.globalAlpha = 0.9;
  ctx.fillStyle = p.graph;
  for (let i = 0; i < n; i++) {
    const q = res.expanded[i].q;
    ctx.beginPath();
    ctx.arc(sx(v, q.x), sy(v, q.y), 1.8, 0, 2 * Math.PI);
    ctx.fill();
  }
  ctx.globalAlpha = 1;
  // Shots attempted so far (hybrid only): failed faint, the successful one solid.
  const shotsShown = res.shots.filter((_s, i) => i < Math.ceil((n / Math.max(1, res.expanded.length)) * res.shots.length));
  for (const s of shotsShown.slice(-4)) {
    ctx.save();
    ctx.strokeStyle = p.path;
    ctx.globalAlpha = s.free ? 0.9 : 0.25;
    ctx.setLineDash(s.free ? [] : [3, 3]);
    ctx.beginPath();
    s.samples.forEach((q, j) => (j === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
    ctx.stroke();
    ctx.restore();
  }
  drawPoseMarker(ctx, v, LOT_REVERSE_START, p.start, 1.0);
  drawPoseMarker(ctx, v, LOT_REVERSE_GOAL, p.goal, 1.0);
  if (done && res.found) {
    drawCarPath(ctx, v, res.samples, p.path, { lineWidth: 3 });
    const end = res.samples[res.samples.length - 1].q;
    drawHitch(ctx, v, CAR, { body: end, trailer: null }, p.robot, { fill: true });
  }
  label(ctx, title, sx(v, 0.3), sy(v, -0.45), p.ink, { size: 11, weight: 600 });
  const stats = res.found ? `${n} expansions   length ${res.length.toFixed(2)} m   ${res.cusps} cusps${res.shots.length ? `   ${res.shots.length} RS shots` : ''}` : `${n} expansions — no path`;
  label(ctx, stats, sx(v, 0.3), sy(v, -0.95), done && res.found ? p.path : p.ink, { size: 10 });
  ctx.restore();
}

export function LatticeVsHybrid() {
  const [params, setParams] = useState<Params>({ k: 5, set: 'six', heuristic: 'dual', shotEvery: 10 });
  const [seed, setSeed] = useState(3);
  const cell = 2.5 / params.k;

  const { lat, hyb } = useMemo(() => {
    const q = { car: CAR, scene: SCENE, start: LOT_REVERSE_START, goal: LOT_REVERSE_GOAL, cell, heuristic: params.heuristic };
    return {
      lat: stateLatticeSearch({ ...q, set: params.set }),
      hyb: hybridAStar({ ...q, shotEvery: params.shotEvery, headingBins: HEADING_BINS[seed % 3] }),
    };
  }, [cell, params.heuristic, params.set, params.shotEvery, seed]);

  const total = Math.max(lat.expansions, hyb.expansions);
  const perTick = Math.max(1, Math.ceil(total / 150));
  const init = useCallback(() => ({ shown: 0 }), []);
  const step = useCallback((s: { shown: number }) => ({ shown: s.shown + perTick }), [perTick]);
  const sim = useSimulation({ init, step, fps: 30, maxTicks: Math.ceil(total / perTick) + 60, loop: true });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const s = sim.state.shown;
      drawPanel(ctx, v, p, 0, lat, s, `state lattice — ${params.set === 'six' ? '6' : '16'} primitives/heading`, s >= lat.expansions);
      drawPanel(ctx, v, p, W + GAP, hyb, s, `hybrid A* — RS shot every ${params.shotEvery}, ${HEADING_BINS[seed % 3]} heading bins`, s >= hyb.expansions);
    },
    [lat, hyb, sim.state.shown, params.set, params.shotEvery, seed],
  );

  return (
    <WidgetFrame
      id="w21.3"
      title="Lattice vs Hybrid A*"
      teaches="Hybrid A* is not a new algorithm: it is CAR GRID SEARCH with one continuous state per cell, an analytic Reeds–Shepp shot, and a better heuristic."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Reverse Hitch into bay 3. Left: A* over a state lattice (grid points × 8 headings) whose primitives were built to end on
          lattice states. Right: hybrid A* — the expansions are Alg. 22&apos;s six arcs from free-floating continuous states, and the
          dashed purple fans are Reeds–Shepp shots that hit something; the first free one ends the search exactly on the goal. Coarsen
          the grid and the lattice&apos;s detours grow while hybrid A* barely notices; set the heuristic to Euclidean and watch both
          panels flood. The seed re-rolls hybrid A*&apos;s heading resolution (64, 72 or 80 bins).
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, lat, hyb]} aspect={2.6} padding={0.15} ariaLabel="Two copies of the Lot: state-lattice expansions on the left, hybrid A* expansions and Reeds–Shepp shots on the right, with the final paths." />
      <ControlPanel columns={3}>
        <Slider label="resolution: cells per bay width" value={params.k} min={2} max={10} step={1} onChange={(k) => (setParams((q) => ({ ...q, k })), sim.reset())} format={(x) => `${x} (cell ${(2.5 / x).toFixed(3)} m)`} role="graph" />
        <div className="flex flex-col gap-1.5">
          <span className="font-ui text-[0.72rem] font-medium">lattice primitives / heuristic</span>
          <ButtonRow>
            {(['six', 'sixteen'] as const).map((s) => (
              <ActionButton key={s} emphasis={params.set === s} onClick={() => (setParams((q) => ({ ...q, set: s })), sim.reset())}>
                {s === 'six' ? '6' : '16'}
              </ActionButton>
            ))}
            {(['euclid', 'rs', 'dual'] as const).map((h) => (
              <ActionButton key={h} emphasis={params.heuristic === h} onClick={() => (setParams((q) => ({ ...q, heuristic: h })), sim.reset())}>
                {h === 'euclid' ? 'Euclid' : h === 'rs' ? 'RS' : 'max(RS, 2D)'}
              </ActionButton>
            ))}
          </ButtonRow>
        </div>
        <Slider label="RS shot every N expansions" value={params.shotEvery} min={1} max={50} step={1} onChange={(shotEvery) => (setParams((q) => ({ ...q, shotEvery })), sim.reset())} role="path" />
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => {
          setSeed((s) => new Rng(s).next() * 1000 | 0);
          sim.reset();
        }}
        tick={sim.tick}
        seed={seed}
      />
    </WidgetFrame>
  );
}
