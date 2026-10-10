'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { CELL_BENCH } from '@/lib/cells/examples';
import { trapezoidal, type Event, type EventKind } from '@/lib/cells/sweep';
import { worldBounds } from '@/lib/cells/world';
import { chip, drawCellWorld, fillPoly, marker, segment, strokePoly } from './shared';

/**
 * w10.3 — Trapezoid Sweep (Choset §6.1, Figs. 6.4–6.8).
 *
 * The §3 bench with the sweep line paused at each of its eleven events. The
 * strip beside the canvas is the list L — the edges the line crosses, bottom
 * to top — before and after the event, with the edges inserted and deleted
 * marked; the event's type is one of Choset's four, decided by which side of
 * the line the two incident edges lie on. Vertical extensions appear only
 * where free space lies above or below the vertex, so the totals end at 12
 * cells and 13 adjacencies — not two extensions per vertex.
 */

const KIND_TEXT: Record<EventKind, { short: string; action: string; shape: 'circle' | 'square' | 'diamond' | 'triangle'; role: 'start' | 'goal' | 'graph' | 'path' }> = {
  BothRight: { short: 'both edges right', action: 'insert e_lower and e_upper', shape: 'circle', role: 'start' },
  BothLeft: { short: 'both edges left', action: 'delete e_lower and e_upper', shape: 'square', role: 'goal' },
  LowerLeftUpperRight: { short: 'e_lower left, e_upper right', action: 'delete e_lower, insert e_upper', shape: 'diamond', role: 'graph' },
  LowerRightUpperLeft: { short: 'e_lower right, e_upper left', action: 'delete e_upper, insert e_lower', shape: 'triangle', role: 'path' },
};

const HOLD = 45;

interface State {
  k: number;
  hold: number;
}

export function TrapezoidSweep() {
  const d = useMemo(() => trapezoidal(CELL_BENCH), []);
  const bounds = useMemo(() => worldBounds(CELL_BENCH), []);
  const [scrub, setScrub] = useState<number | null>(null);
  const [typeColors, setTypeColors] = useState(true);
  const [adjacency, setAdjacency] = useState(false);
  const n = d.events.length;

  const init = useCallback((): State => ({ k: 0, hold: 0 }), []);
  const step = useCallback(
    (s: State): State => {
      if (s.hold + 1 < (s.k >= n ? 2 * HOLD : HOLD)) return { ...s, hold: s.hold + 1 };
      return { k: s.k >= n ? 0 : s.k + 1, hold: 0 };
    },
    [n],
  );
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 10 });
  // k = number of events processed (0 … n).
  const k = scrub ?? sim.state.k;
  const ev: Event | null = k > 0 ? d.events[k - 1] : null;
  const closedCells = useMemo(() => d.cells.filter((c) => c.closeEvent < k), [d, k]);
  const openedSoFar = d.events.slice(0, k).reduce((s, e) => s + e.opened.length, 0);
  const adjSoFar = d.adjacency.filter((a) => a.event < k);
  const edgeName = (id: number) => `e${id}`;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      // Closed cells, tinted.
      closedCells.forEach((c, i) => {
        fillPoly(ctx, v, c.polygon, p.graph, 0.07 + 0.06 * (i % 3));
        strokePoly(ctx, v, c.polygon, p.graph, 0.8);
      });
      drawCellWorld(ctx, v, p, CELL_BENCH);
      // Edge labels.
      for (const e of d.edges) {
        const m = { x: (e.a.x + e.b.x) / 2, y: (e.a.y + e.b.y) / 2 };
        const inL = ev ? ev.Lafter.includes(e.id) : false;
        chip(ctx, edgeName(e.id), sx(v, m.x), sy(v, m.y), inL ? p.accent : p.ink, p.bg, 9);
      }
      // Extensions so far.
      d.events.slice(0, k).forEach((e) => {
        const col = typeColors ? p[KIND_TEXT[e.kind].role] : p.graph;
        if (e.extUp) segment(ctx, v, e.extUp.a, e.extUp.b, col, e === ev ? 3 : 1.8);
        if (e.extDown) segment(ctx, v, e.extDown.a, e.extDown.b, col, e === ev ? 3 : 1.8);
      });
      // Adjacency overlay: centroid → extension midpoint → centroid.
      if (adjacency) {
        for (const a of adjSoFar) {
          const ca = d.cells[a.a]?.centroid;
          const cb = d.cells[a.b]?.centroid;
          const m = { x: (a.segment.a.x + a.segment.b.x) / 2, y: (a.segment.a.y + a.segment.b.y) / 2 };
          if (ca && a.event < k && d.cells[a.a].closeEvent < k) segment(ctx, v, ca, m, p.graph, 1.4, [4, 3]);
          if (cb && d.cells[a.b].closeEvent < k) segment(ctx, v, m, cb, p.graph, 1.4, [4, 3]);
        }
        for (const c of closedCells) marker(ctx, sx(v, c.centroid.x), sy(v, c.centroid.y), 3.5, p.graph, p.bg);
      }
      for (const c of closedCells) label(ctx, `c${c.id}`, sx(v, c.centroid.x) + 5, sy(v, c.centroid.y) - 7, p.ink, { size: 9 });
      // Sweep line.
      const xLine = ev ? ev.v.x : bounds.minX - 0.3;
      segment(ctx, v, { x: xLine, y: bounds.minY - 0.4 }, { x: xLine, y: bounds.maxY + 0.4 }, p.accent, 1.4, [6, 4]);
      // Event vertices processed so far.
      d.events.slice(0, k).forEach((e) => {
        const t = KIND_TEXT[e.kind];
        marker(ctx, sx(v, e.v.x), sy(v, e.v.y), e === ev ? 6 : 4, typeColors ? p[t.role] : p.ink, p.bg, t.shape);
      });
      label(ctx, ev ? `event ${k}/${n}: v = (${ev.v.x}, ${ev.v.y}) · ${KIND_TEXT[ev.kind].short}` : `before the first event · L = ∅`, 10, 14, p.ink, { size: 10.5, weight: 600 });
      label(ctx, `cells opened ${openedSoFar} · adjacencies ${adjSoFar.length}${k === n ? ' — complete: 12 trapezoids, 13 edges' : ''}`, 10, v.height - 12, p.ink, { size: 10 });
    },
    [d, k, ev, closedCells, typeColors, adjacency, adjSoFar, openedSoFar, n, bounds],
  );

  const Lstrip = (ids: readonly number[], other: readonly number[], mark: 'in' | 'out') => (
    <div className="flex flex-wrap gap-1">
      {ids.length === 0 ? <span className="text-fd-muted-foreground">∅</span> : null}
      {ids.map((id) => {
        const changed = !other.includes(id);
        return (
          <span
            key={id}
            className="rounded-sm border border-fd-border px-1.5 py-0.5"
            style={
              changed
                ? mark === 'in'
                  ? { borderColor: 'var(--rm-start)', color: 'var(--rm-start)', fontWeight: 700 }
                  : { borderColor: 'var(--rm-goal)', color: 'var(--rm-goal)', textDecoration: 'line-through' }
                : undefined
            }
          >
            {edgeName(id)}
          </span>
        );
      })}
    </div>
  );

  return (
    <WidgetFrame
      id="w10.3"
      title="Trapezoid Sweep"
      teaches="Not every vertex gets two extensions: only those with free space both above and below — the boustrophedon critical points — get both; the rest get one or none, and the totals land on 12 cells and 13 adjacencies."
      colorKey={['obstacle', 'graph']}
      caption={
        <>
          The §3 bench, swept left to right. The dashed line stops at each vertex in x-order; the
          strip lists L bottom to top before and after the event (inserted edges in green, deleted
          ones struck through in red), and the event type follows Choset Fig. 6.4 from which side of
          the line the two incident edges lie on. Colors and marker shapes: ● both right (insert),
          ■ both left (delete), ◆ e_lower left / e_upper right, ▲ e_lower right / e_upper left.
          Scrub the event index, or overlay the adjacency graph through the extension midpoints.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border lg:grid-cols-[3fr_2fr]">
        <div className="bg-fd-card">
          <SimCanvas world={bounds} draw={draw} deps={[k, typeColors, adjacency]} aspect={16 / 10} padding={0.5} ariaLabel="A polygonal plot with a triangle and a quadrilateral; a dashed vertical line steps through the vertices while blue vertical extensions and trapezoidal cells accumulate." />
        </div>
        <div className="bg-fd-card px-3 py-2 font-mono text-[0.7rem] tabular-nums">
          <div className="eyebrow mb-1">List L (bottom → top)</div>
          <div className="mb-1 text-fd-muted-foreground">before</div>
          {Lstrip(ev ? ev.Lbefore : [], ev ? ev.Lafter : [], 'out')}
          <div className="mb-1 mt-2 text-fd-muted-foreground">after</div>
          {Lstrip(ev ? ev.Lafter : [], ev ? ev.Lbefore : [], 'in')}
          {ev ? (
            <div className="mt-3 space-y-1 leading-snug">
              <div>
                <span className="text-fd-muted-foreground">type </span>
                {KIND_TEXT[ev.kind].short} — {KIND_TEXT[ev.kind].action}
              </div>
              <div>
                <span className="text-fd-muted-foreground">e_lower, e_upper </span>
                {edgeName(ev.eLower)}, {edgeName(ev.eUpper)} ·{' '}
                <span className="text-fd-muted-foreground">e_LOWER, e_UPPER </span>
                {ev.eLOWER === null ? '—' : edgeName(ev.eLOWER)}, {ev.eUPPER === null ? '—' : edgeName(ev.eUPPER)}
              </div>
              <div>
                <span className="text-fd-muted-foreground">extensions </span>
                {ev.extUp ? 'up ' : ''}
                {ev.extDown ? 'down ' : ''}
                {!ev.extUp && !ev.extDown ? 'none' : ''}
                <span className="text-fd-muted-foreground"> · slice </span>
                {ev.slice === 'pass' ? 'pass-through' : `${ev.slice} (critical)`}
              </div>
              <div>
                <span className="text-fd-muted-foreground">closes </span>
                {ev.closed.length ? ev.closed.map((c) => `c${c}`).join(', ') : '—'}
                <span className="text-fd-muted-foreground"> · opens </span>
                {ev.opened.length ? ev.opened.map((c) => `c${c}`).join(', ') : '—'}
              </div>
            </div>
          ) : (
            <div className="mt-3 text-fd-muted-foreground">Press play or drag the event slider.</div>
          )}
        </div>
      </div>
      <ControlPanel columns={3}>
        <Slider label="Event index" value={k} min={0} max={n} step={1} onChange={(x) => setScrub(x)} format={(x) => `${x} / ${n}`} />
        <Toggle label="Color by event type" checked={typeColors} onChange={setTypeColors} />
        <Toggle label="Adjacency overlay" checked={adjacency} onChange={setAdjacency} role="graph" />
      </ControlPanel>
      <Transport
        playing={sim.playing && scrub === null}
        onToggle={() => {
          setScrub(null);
          sim.toggle();
        }}
        onStep={() => {
          if (sim.playing) sim.toggle();
          setScrub(Math.min(n, k + 1));
        }}
        onReset={() => {
          setScrub(null);
          sim.reset();
        }}
        tick={k}
      />
    </WidgetFrame>
  );
}
