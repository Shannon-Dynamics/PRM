'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawRobot, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Point2 } from '@/lib/sim/world';
import { CELL_BENCH, LIVING_ROOM, LIVING_ROOM_START } from '@/lib/cells/examples';
import { randomCellWorld } from '@/lib/cells/lab';
import { arcLengths, coveragePath, coverageTimeline, pathLengthBound, pointAtArc } from '@/lib/cells/coverage';
import { reebFromSweep } from '@/lib/cells/reeb';
import { freeRaster, rasterCounts, rasterMorse, type SliceKind } from '@/lib/cells/slices';
import { sweepDecompose, type DecompositionMode } from '@/lib/cells/sweep';
import { isFreePoint, isValidWorld, worldBounds, type CellWorld } from '@/lib/cells/world';
import { blitRaster, drawCellWorld, fillPoly, marker, polyline, rasterImage, segment, strokePoly } from './shared';

/**
 * w10.1 — the Boustrophedon Mower (Choset §6.2.1–6.2.5).
 *
 * A vertical slice sweeps the free space; at each critical point a cell wall
 * snaps in and the Reeb graph (docked right) gains a node. Then Rusty mows
 * cell by cell along the depth-first walk, lapping with spacing 2r and
 * hopping between cells along A* routes on the trapezoids. Switch to the
 * trapezoidal decomposition and the cell count jumps from 7 to 12 — and the
 * path gets longer, because every cell costs a turnaround. Euler's count
 * N_ce = N_cp + N_ob − 1 is printed live; drag a vertex and watch it hold.
 */

type WorldChoice = 'bench' | 'living' | 'random';
type SliceChoice = Extract<SliceKind, 'vertical' | 'radial' | 'squarel' | 'brushfire'>;

const SWEEP_TICKS = 110;
const MOW_TICKS = 560;
const HOLD_TICKS = 70;

interface State {
  t: number;
}

function centerFor(w: CellWorld): Point2 {
  const b = worldBounds(w);
  const c = { x: (b.minX + b.maxX) / 2, y: (b.minY + b.maxY) / 2 };
  if (isFreePoint(w, c)) return c;
  for (let r = 0.2; r < 10; r += 0.2) {
    for (let k = 0; k < 16; k++) {
      const p = { x: c.x + r * Math.cos((k * Math.PI) / 8), y: c.y + r * Math.sin((k * Math.PI) / 8) };
      if (isFreePoint(w, p) && w.obstacles.every((o) => o.every((q) => Math.hypot(q.x - p.x, q.y - p.y) > 0.3))) return p;
    }
  }
  return c;
}

function Segmented<T extends string>({ value, options, onChange, label: title }: { value: T; options: { v: T; label: string }[]; onChange: (v: T) => void; label: string }) {
  return (
    <div className="flex flex-col gap-1">
      <span className="font-ui text-[0.72rem] font-medium">{title}</span>
      <div className="flex flex-wrap gap-1">
        {options.map((o) => (
          <button
            key={o.v}
            type="button"
            onClick={() => onChange(o.v)}
            aria-pressed={value === o.v}
            className={`rounded-sm border border-fd-border px-2 py-0.5 font-ui text-[0.7rem] transition-colors ${value === o.v ? 'bg-fd-accent font-semibold text-fd-foreground' : 'bg-fd-card text-fd-muted-foreground hover:bg-fd-accent'}`}
          >
            {o.label}
          </button>
        ))}
      </div>
    </div>
  );
}

export function BoustrophedonMower() {
  const [choice, setChoice] = useState<WorldChoice>('bench');
  const [bench, setBench] = useState<CellWorld>(CELL_BENCH);
  const [benchSettled, setBenchSettled] = useState<CellWorld>(CELL_BENCH);
  const [mode, setMode] = useState<DecompositionMode>('boustrophedon');
  const [slice, setSlice] = useState<SliceChoice>('vertical');
  const [twoR, setTwoR] = useState(0.5);
  const drag = useRef<{ poly: number; idx: number } | null>(null);

  const init = useCallback((): State => ({ t: 0 }), []);
  const step = useCallback((s: State): State => ({ t: s.t + 1 }), []);
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 10 });

  const randomWorld = useMemo(() => randomCellWorld(new Rng(sim.seed)), [sim.seed]);
  const world = choice === 'bench' ? bench : choice === 'living' ? LIVING_ROOM : randomWorld;
  const settled = choice === 'bench' ? benchSettled : world;
  const dragging = choice === 'bench' && bench !== benchSettled;
  const bounds = useMemo(() => worldBounds(world), [world]);
  const start = choice === 'living' ? LIVING_ROOM_START : undefined;

  const dec = useMemo(() => sweepDecompose(world, mode), [world, mode]);
  const bdec = useMemo(() => (mode === 'boustrophedon' ? dec : sweepDecompose(world, 'boustrophedon')), [world, mode, dec]);
  const reeb = useMemo(() => reebFromSweep(bdec), [bdec]);
  const settledDec = useMemo(() => sweepDecompose(settled, mode), [settled, mode]);
  const cov = useMemo(() => {
    try {
      return coveragePath(settledDec, twoR, { start });
    } catch {
      return null;
    }
  }, [settledDec, twoR, start]);
  const res = choice === 'living' ? 0.03 : choice === 'bench' ? 0.06 : 0.15;
  const raster = useMemo(() => freeRaster(settled, res), [settled, res]);
  const timeline = useMemo(() => (cov ? coverageTimeline(raster, cov.points, twoR / 2) : null), [raster, cov, twoR]);
  const sortedFirst = useMemo(() => {
    if (!timeline) return null;
    const vals: number[] = [];
    let free = 0;
    for (let k = 0; k < raster.free.length; k++) {
      if (!raster.free[k]) continue;
      free++;
      if (Number.isFinite(timeline[k])) vals.push(timeline[k]);
    }
    return { vals: Float64Array.from(vals).sort(), free };
  }, [timeline, raster]);
  const cum = useMemo(() => (cov ? arcLengths(cov.points) : [0]), [cov]);
  const bound = useMemo(() => pathLengthBound(settled, twoR, settledDec.cells.length), [settled, twoR, settledDec]);
  const center = useMemo(() => centerFor(world), [world]);
  const morse = useMemo(
    () => (slice === 'vertical' ? null : rasterMorse(world, { kind: slice, center }, { cell: choice === 'random' ? 0.14 : choice === 'living' ? 0.05 : 0.08 })),
    [world, slice, center, choice],
  );
  const morseCounts = useMemo(() => (morse ? rasterCounts(morse) : null), [morse]);

  const L = cov ? cov.length : 0;
  const t = sim.state.t;
  const total = SWEEP_TICKS + (slice === 'vertical' ? MOW_TICKS : 0) + HOLD_TICKS;
  const tt = dragging ? SWEEP_TICKS : t % total;
  const sweepFrac = Math.min(1, tt / SWEEP_TICKS);
  const sliceX = bounds.minX + (bounds.maxX - bounds.minX) * sweepFrac;
  const mowS = slice !== 'vertical' || dragging ? 0 : tt <= SWEEP_TICKS ? 0 : Math.min(L, ((tt - SWEEP_TICKS) / MOW_TICKS) * L);
  const mowing = mowS > 0;
  const coveredFrac = useMemo(() => {
    if (!sortedFirst || !mowing) return 0;
    const a = sortedFirst.vals;
    let lo = 0, hi = a.length;
    while (lo < hi) {
      const m = (lo + hi) >> 1;
      if (a[m] <= mowS) lo = m + 1;
      else hi = m;
    }
    return lo / sortedFirst.free;
  }, [sortedFirst, mowS, mowing]);

  // Which cell is being mowed (for the docked graph).
  const currentCell = useMemo(() => {
    if (!cov || !mowing) return -1;
    let s = 0;
    for (const piece of cov.pieces) {
      let len = 0;
      for (let i = 1; i < piece.pts.length; i++) len += Math.hypot(piece.pts[i].x - piece.pts[i - 1].x, piece.pts[i].y - piece.pts[i - 1].y);
      if (mowS <= s + len) return piece.cell;
      s += len;
    }
    return -1;
  }, [cov, mowS, mowing]);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (choice !== 'bench') return;
      if (phase === 'down') {
        const polys = [bench.boundary, ...bench.obstacles];
        let best: { poly: number; idx: number } | null = null;
        let bd = 0.45;
        polys.forEach((poly, pi) =>
          poly.forEach((q, i) => {
            const dd = Math.hypot(q.x - w[0], q.y - w[1]);
            if (dd < bd) {
              bd = dd;
              best = { poly: pi, idx: i };
            }
          }),
        );
        drag.current = best;
      } else if (phase === 'move' && drag.current) {
        const { poly, idx } = drag.current;
        let x = Math.round(w[0] * 100) / 100;
        const y = Math.round(w[1] * 100) / 100;
        const polys = [bench.boundary, ...bench.obstacles].map((pp) => pp.slice());
        // General position, kept by a tiny deterministic jitter.
        const others = polys.flatMap((pp, pi) => pp.filter((_, i) => !(pi === poly && i === idx)).map((q) => q.x));
        while (others.some((ox) => Math.abs(ox - x) < 1e-6)) x += 1e-4;
        polys[poly][idx] = { x, y };
        const next: CellWorld = { ...bench, boundary: polys[0], obstacles: polys.slice(1) };
        if (isValidWorld(next, 0.05)) setBench(next);
      } else if (phase === 'up') {
        if (drag.current) {
          setBenchSettled(bench);
          sim.reset();
        }
        drag.current = null;
      }
    },
    [choice, bench, sim],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      if (morse && slice !== 'vertical') {
        // Variable slice: reveal the raster cells as the level λ rises.
        const maxBand = morse.bandOf.reduce((m, b) => Math.max(m, b), 0);
        const lam = sweepFrac * (maxBand + 1);
        const img = rasterImage(morse.raster, p.graph, (k) => {
          if (morse.bandOf[k] < 0 || morse.bandOf[k] > lam) return 0;
          if (morse.nodeOf[k] >= 0) return 0.75;
          const c = morse.cellOf[k];
          return c < 0 ? 0 : 0.08 + 0.09 * (c % 4);
        });
        blitRaster(ctx, v, morse.raster, img);
        drawCellWorld(ctx, v, p, world, { names: true });
        for (const n of morse.nodes) if (morse.bandOf.length && n.band <= lam) marker(ctx, sx(v, n.q.x), sy(v, n.q.y), 4.5, p.graph, p.bg);
        if (slice !== 'brushfire') marker(ctx, sx(v, center.x), sy(v, center.y), 3.5, p.accent, p.bg, 'diamond');
        const c = morseCounts!;
        const tree = slice === 'brushfire';
        label(
          ctx,
          `${slice} slice · N_cp = ${c.nCp} · N_ob = ${c.nOb} · N_ce = ${c.nCe}${tree ? ` = N_cp − 1 (each obstacle is a critical set)` : ` = ${c.nCp} + ${c.nOb} − 1 ${c.nCe === c.nCp + c.nOb - 1 ? '✓' : '✗'}`}`,
          10,
          14,
          p.ink,
          { size: 10.5, weight: 600 },
        );
        label(ctx, 'the mower runs for h = x; here the raster Morse decomposition (critical slices dark)', 10, v.height - 12, p.ink, { size: 9.5 });
        return;
      }
      // Cells revealed up to the slice.
      ctx.save();
      ctx.beginPath();
      ctx.rect(0, 0, sx(v, sliceX), v.height);
      ctx.clip();
      dec.cells.forEach((c, i) => {
        fillPoly(ctx, v, c.polygon, p.graph, c.id === currentCell ? 0.3 : 0.06 + 0.05 * (i % 3));
        strokePoly(ctx, v, c.polygon, p.graph, 0.6);
      });
      ctx.restore();
      // Coverage tint.
      if (mowing && timeline) {
        const img = rasterImage(raster, p.path, (k) => (timeline[k] <= mowS ? 0.3 : 0));
        blitRaster(ctx, v, raster, img);
      }
      drawCellWorld(ctx, v, p, world, { names: true });
      // Cell walls (extensions) and critical points up to the slice.
      for (const e of dec.events) {
        if (e.v.x > sliceX) continue;
        const crit = e.slice !== 'pass';
        if (e.extUp) segment(ctx, v, e.extUp.a, e.extUp.b, p.graph, crit ? 2.2 : 1.4);
        if (e.extDown) segment(ctx, v, e.extDown.a, e.extDown.b, p.graph, crit ? 2.2 : 1.4);
        if (crit) marker(ctx, sx(v, e.v.x), sy(v, e.v.y), 5, p.graph, p.bg);
      }
      if (!mowing) segment(ctx, v, { x: sliceX, y: bounds.minY - 0.3 }, { x: sliceX, y: bounds.maxY + 0.3 }, p.accent, 1.5, [6, 4]);
      // The path so far and Rusty.
      if (mowing && cov) {
        const at = pointAtArc(cov.points, cum, mowS);
        polyline(ctx, v, [...cov.points.slice(0, at.i + 1), at.p], p.path, 1.4, 0.9);
        const prev = cov.points[Math.max(0, at.i)];
        const heading = Math.atan2(at.p.y - prev.y, at.p.x - prev.x);
        // The effector's footprint, then Rusty.
        ctx.save();
        ctx.strokeStyle = p.robot;
        ctx.globalAlpha = 0.6;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.arc(sx(v, at.p.x), sy(v, at.p.y), Math.max(2, (twoR / 2) * (v.width / (v.maxX - v.minX))), 0, 2 * Math.PI);
        ctx.stroke();
        ctx.restore();
        drawRobot(ctx, v, { x: at.p.x, y: at.p.y, theta: Number.isFinite(heading) ? heading : 0 }, p.robot, Math.max(0.12, twoR / 2));
      }
      const nCp = reeb.nCriticalPoints;
      const nOb = world.obstacles.length;
      const head =
        mode === 'boustrophedon'
          ? `boustrophedon · N_cp = ${nCp} · N_ob = ${nOb} · N_ce = ${dec.cells.length} = ${nCp} + ${nOb} − 1 ${dec.cells.length === nCp + nOb - 1 ? '✓' : '✗'}`
          : `trapezoidal · ${dec.cells.length} cells, ${dec.adjacency.length} adjacencies (boustrophedon: ${bdec.cells.length})`;
      label(ctx, head, 10, 14, p.ink, { size: 10.5, weight: 600 });
      const foot = dragging
        ? 'release to re-plan the mower'
        : cov
          ? `path ${mowS.toFixed(1)} / ${L.toFixed(1)} m · bound ${bound.bound.toFixed(0)} m · covered ${(100 * coveredFrac).toFixed(1)} % · ${cov.laps} laps · transits ${cov.transitLength.toFixed(1)} m`
          : 'no coverage path';
      label(ctx, foot, 10, v.height - 12, p.ink, { size: 10 });
    },
    [morse, morseCounts, slice, sweepFrac, world, center, dec, bdec, currentCell, mowing, timeline, raster, mowS, sliceX, bounds, cov, cum, twoR, reeb, mode, dragging, L, bound, coveredFrac],
  );

  // The docked graph: Reeb graph (boustrophedon) or adjacency graph (trapezoidal).
  const drawGraph = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      strokePoly(ctx, v, world.boundary, p.obstacle, 1, true, [3, 3]);
      for (const o of world.obstacles) fillPoly(ctx, v, o, p.obstacle, 0.15);
      if (mode === 'boustrophedon') {
        for (const e of reeb.edges) {
          const a = reeb.nodes[e.from].q;
          const b = reeb.nodes[e.to].q;
          const c = bdec.cells[e.cell].centroid;
          const visible = b.x <= sliceX + 1e-9 || mowing;
          if (!visible) continue;
          const lit = e.cell === currentCell;
          polyline(ctx, v, [a, c, b], lit ? p.path : p.graph, lit ? 3 : 1.6);
          label(ctx, `${e.cell}`, sx(v, c.x) + 3, sy(v, c.y) - 6, lit ? p.path : p.ink, { size: 9 });
        }
        for (const n of reeb.nodes) if (n.q.x <= sliceX + 1e-9 || mowing) marker(ctx, sx(v, n.q.x), sy(v, n.q.y), 4.5, p.graph, p.bg);
        label(ctx, `Reeb graph: ${reeb.nCriticalPoints} nodes, ${reeb.nCells} edges`, 6, 12, p.ink, { size: 9.5, weight: 600 });
      } else {
        for (const a of dec.adjacency) {
          const ca = dec.cells[a.a].centroid, cb = dec.cells[a.b].centroid;
          if (Math.max(ca.x, cb.x) > sliceX && !mowing) continue;
          const lit = a.a === currentCell || a.b === currentCell;
          segment(ctx, v, ca, cb, lit ? p.path : p.graph, lit ? 2.4 : 1.3);
        }
        for (const c of dec.cells) if (c.xRange[1] <= sliceX || mowing) marker(ctx, sx(v, c.centroid.x), sy(v, c.centroid.y), c.id === currentCell ? 5 : 3.5, c.id === currentCell ? p.path : p.graph, p.bg);
        label(ctx, `adjacency graph: ${dec.cells.length} nodes, ${dec.adjacency.length} edges`, 6, 12, p.ink, { size: 9.5, weight: 600 });
      }
    },
    [world, mode, reeb, bdec, dec, sliceX, mowing, currentCell],
  );

  const reset = () => sim.reset();

  return (
    <WidgetFrame
      id="w10.1"
      title="Boustrophedon Mower"
      teaches="More cells is not finer coverage: cell walls belong only at the critical points, and every extra cell costs a turnaround and a transit — the trapezoidal mower is longer, not better."
      colorKey={['obstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          A vertical slice sweeps the free space; a cell wall (blue) snaps in only where the free
          slice splits or merges, and the Reeb graph on the right gains a node — 6 critical points,
          2 obstacles, 7 cells on the §3 bench, exactly N_ce = N_cp + N_ob − 1. Then Rusty mows the
          cells in depth-first order, lapping at spacing 2r (purple tint is covered ground) and
          hopping between cells along A* routes. Switch to trapezoidal and count 12 cells and a
          longer path; drag any vertex of the bench (a tiny jitter keeps vertices in general
          position) and the count still obeys Euler; try a curved slice (h = |q − c|, |x| + |y|, or
          the brushfire distance) to see the cells a different sweep induces; the shuffle button
          rolls a random world.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border lg:grid-cols-[3fr_1.3fr]">
        <div className="bg-fd-card">
          <SimCanvas
            world={bounds}
            draw={draw}
            deps={[t, mode, slice, twoR, world, cov]}
            aspect={16 / 10}
            padding={0.4}
            onPointer={onPointer}
            cursor={choice === 'bench' ? 'grab' : 'default'}
            ariaLabel="A polygonal free space with two obstacles. A dashed vertical line sweeps across; blue cell walls appear at critical points; then an orange robot mows back and forth while the covered area turns purple."
          />
        </div>
        <div className="bg-fd-card">
          <SimCanvas world={bounds} draw={drawGraph} deps={[t, mode, world]} aspect={4 / 3} padding={0.4} ariaLabel="The Reeb graph: one node per critical point, one edge per cell, drawn at the critical points' positions." />
        </div>
      </div>
      <ControlPanel columns={2}>
        <Slider
          label="Effector width 2r"
          role="path"
          value={twoR}
          min={0.2}
          max={1.0}
          step={0.05}
          unit="m"
          onChange={(x) => {
            setTwoR(x);
            reset();
          }}
        />
        <Segmented
          label="Decomposition"
          value={mode}
          options={[
            { v: 'boustrophedon', label: 'boustrophedon' },
            { v: 'trapezoidal', label: 'trapezoidal' },
          ]}
          onChange={(m) => {
            setMode(m);
            reset();
          }}
        />
        <Segmented
          label="World"
          value={choice}
          options={[
            { v: 'bench', label: '§3 bench (drag vertices)' },
            { v: 'living', label: 'living room (2r = 0.44)' },
            { v: 'random', label: 'random (seed)' },
          ]}
          onChange={(c) => {
            setChoice(c);
            if (c === 'living') setTwoR(0.44);
            else if (c === 'bench') setTwoR(0.5);
            reset();
          }}
        />
        <Segmented
          label="Slice function h"
          value={slice}
          options={[
            { v: 'vertical', label: 'x (mow)' },
            { v: 'radial', label: '|q − c|' },
            { v: 'squarel', label: '|x| + |y|' },
            { v: 'brushfire', label: 'brushfire D' },
          ]}
          onChange={(s) => {
            setSlice(s);
            reset();
          }}
        />
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={() => {
          if (choice === 'bench') {
            setBench(CELL_BENCH);
            setBenchSettled(CELL_BENCH);
          }
          sim.reset();
        }}
        onReseed={() => {
          setChoice('random');
          sim.reseed();
        }}
        seed={choice === 'random' ? sim.seed : undefined}
        tick={t}
      />
    </WidgetFrame>
  );
}
