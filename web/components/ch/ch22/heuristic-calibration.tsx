'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { BestFirst } from '@/lib/search/best-first';
import { EPSILON_VALIDATION, HeuristicNet, TorusGridGraph, aStarEngine, aStarEps, costToGo, latticeOf, loadModels, makeWorldCase, randomFreeCell, torusOctile, type HeuristicKind, type SearchReport, type WorldCase } from '@/lib/learn';

/**
 * w22.2 — Heuristic Calibration.
 *
 * Chapter 6's A* on Reach's 24 × 24 torus lattice with a pluggable heuristic:
 * the admissible octile distance, the learned ĥ, or the learned ĥ deflated by
 * the ε measured on the validation suite. Left: the lattice with the explored
 * set (blue), the frontier (lighter), the path (purple), and every node whose
 * ĥ exceeds h* lit red. Right: each node at (h*(v), ĥ(v)) against the dashed
 * diagonal; the red dashed line is (1 + ε) h* for the ε measured on *this*
 * world. The headline knob scales ĥ: slide past 1 and watch expansions fall
 * while the cost ratio climbs — never past the F3 line.
 */

const ARENA = { minX: 0, minY: 0, maxX: 2.2, maxY: 1 };
const PANEL_MAP = { x0: 0, y0: 0, x1: 1, y1: 1 };
const PANEL_SCAT = { x0: 1.2, y0: 0, x1: 2.2, y1: 1 };
const EXPANSIONS_PER_TICK = 6;

interface State {
  wc: WorldCase;
  graph: TorusGridGraph;
  start: number;
  goal: number;
  hStar: Float64Array;
  net: HeuristicNet;
  /** ĥ for the chosen kind at the chosen scale, per cell (0 on obstacles). */
  h: Float64Array;
  engine: BestFirst<number>;
  final: SearchReport;
  dijkstra: SearchReport;
  octile: SearchReport;
  epsilon: number;
  violations: number;
  reachable: number;
}

interface Params {
  kind: HeuristicKind;
  scale: number;
}

function build(seed: number, prms: Params): State {
  const wc = makeWorldCase(seed, 'train');
  const graph = latticeOf(wc.lab);
  const rng = new Rng(seed ^ 0x9e3779b9);
  const goal = randomFreeCell(graph, rng);
  const hStar = costToGo(graph, goal);
  // The farthest of a few random reachable starts, so the search has somewhere to go.
  let start = -1;
  for (let t = 0; t < 12; t++) {
    const c = randomFreeCell(graph, rng);
    if (c >= 0 && Number.isFinite(hStar[c]) && (start < 0 || hStar[c] > hStar[start])) start = c;
  }
  if (start < 0) start = goal;
  const net = new HeuristicNet(loadModels().heuristic, wc.enc, graph, goal);
  const oct = torusOctile(graph, goal);
  const h = new Float64Array(graph.size);
  let epsilon = 0;
  let violations = 0;
  let reachable = 0;
  for (let c = 0; c < graph.size; c++) {
    if (!graph.isFree(c)) continue;
    const base = prms.kind === 'octile' ? oct(c) : prms.kind === 'learned' ? net.table[c] : net.table[c] / (1 + EPSILON_VALIDATION);
    h[c] = prms.scale * base;
    if (Number.isFinite(hStar[c]) && hStar[c] > 0) {
      reachable++;
      if (h[c] > hStar[c] + 1e-9) {
        violations++;
        epsilon = Math.max(epsilon, h[c] / hStar[c] - 1);
      }
    }
  }
  const hf = (c: number) => h[c];
  return {
    wc,
    graph,
    start,
    goal,
    hStar,
    net,
    h,
    engine: aStarEngine(graph, start, goal, hf),
    final: aStarEps(graph, start, goal, hf),
    dijkstra: aStarEps(graph, start, goal, () => 0),
    octile: aStarEps(graph, start, goal, oct),
    epsilon,
    violations,
    reachable,
  };
}

const KINDS: { kind: HeuristicKind; label: string }[] = [
  { kind: 'octile', label: 'octile (admissible)' },
  { kind: 'learned', label: 'learned ĥ' },
  { kind: 'learned-deflated', label: `learned ĥ / (1 + ε_val)` },
];

export function HeuristicCalibration() {
  const [scale, setScale] = useState(1.0);
  const [kind, setKind] = useState<HeuristicKind>('learned');
  const params = useMemo<Params>(() => ({ kind, scale }), [kind, scale]);
  const init = useCallback((seed: number) => build(seed, params), [params]);

  const stepFn = useCallback((s: State): State => {
    if (s.engine.status !== 'running') return s;
    for (let i = 0; i < EXPANSIONS_PER_TICK && s.engine.status === 'running'; i++) s.engine.step();
    return { ...s };
  }, []);

  // Seed 39: the learned ĥ expands fewer nodes than octile *and* returns a longer path — the misconception, on screen.
  const sim = useSimulation<State>({ init, step: stepFn, fps: 20, initialSeed: 39 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [params]);

  const s = sim.state;
  const cStar = s.dijkstra.cost;
  const bound = (1 + s.epsilon) * cStar;
  const ratio = s.final.found ? s.final.cost / cStar : NaN;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const g = s.graph;
      const n = g.n;
      // --- lattice panel
      const px = sx(v, PANEL_MAP.x0);
      const py = sy(v, PANEL_MAP.y1);
      const w = sx(v, PANEL_MAP.x1) - px;
      const hgt = sy(v, PANEL_MAP.y0) - py;
      const cw = w / n;
      const ch = hgt / n;
      const cellRect = (c: number): [number, number] => {
        const [i, j] = g.ij(c);
        return [px + i * cw, py + hgt - (j + 1) * ch];
      };
      ctx.save();
      for (let c = 0; c < g.size; c++) {
        const [x, y] = cellRect(c);
        if (!g.isFree(c)) {
          ctx.globalAlpha = 0.75;
          ctx.fillStyle = p.cobstacle;
          ctx.fillRect(x, y, cw + 0.5, ch + 0.5);
          continue;
        }
        const key = c;
        if (s.engine.closed.has(key)) {
          ctx.globalAlpha = 0.45;
          ctx.fillStyle = p.graph;
          ctx.fillRect(x, y, cw + 0.5, ch + 0.5);
        } else if (s.engine.open.has(key)) {
          ctx.globalAlpha = 0.2;
          ctx.fillStyle = p.graph;
          ctx.fillRect(x, y, cw + 0.5, ch + 0.5);
        }
        if (Number.isFinite(s.hStar[c]) && s.hStar[c] > 0 && s.h[c] > s.hStar[c] + 1e-9) {
          ctx.globalAlpha = 0.55;
          ctx.fillStyle = p.goal;
          ctx.fillRect(x + cw * 0.3, y + ch * 0.3, cw * 0.4, ch * 0.4);
        }
      }
      ctx.restore();
      // path so far / final
      const path = s.engine.status === 'found' ? s.engine.path() : [];
      if (path.length > 1) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 3;
        ctx.lineJoin = 'round';
        ctx.beginPath();
        let prev: [number, number] | null = null;
        for (const c of path) {
          const [x, y] = cellRect(c);
          const cx = x + cw / 2;
          const cy = y + ch / 2;
          if (prev && (Math.abs(cx - prev[0]) > w / 2 || Math.abs(cy - prev[1]) > hgt / 2)) ctx.moveTo(cx, cy);
          else if (prev) ctx.lineTo(cx, cy);
          else ctx.moveTo(cx, cy);
          prev = [cx, cy];
        }
        ctx.stroke();
        ctx.restore();
      }
      const [sxp, syp] = cellRect(s.start);
      const [gxp, gyp] = cellRect(s.goal);
      ctx.save();
      ctx.fillStyle = p.start;
      ctx.fillRect(sxp, syp, cw, ch);
      ctx.fillStyle = p.goal;
      ctx.fillRect(gxp, gyp, cw, ch);
      ctx.strokeStyle = p.accent;
      ctx.setLineDash([6, 5]);
      ctx.lineWidth = 1.5;
      ctx.strokeRect(px, py, w, hgt);
      ctx.restore();
      label(ctx, 'θ₁ →', px + w - 4, py + hgt + 12, p.ink, { size: 10, align: 'right' });
      label(ctx, '↑ θ₂', px + 4, py + 10, p.ink, { size: 10 });
      label(ctx, `A* with ${kind === 'octile' ? 'octile' : kind === 'learned' ? 'learned ĥ' : 'deflated ĥ'} × ${scale.toFixed(2)} — ${s.engine.pops.length} expansions${s.engine.status === 'found' ? `, cost ${s.engine.cost().toFixed(3)}` : ''}`, px + 4, py - 9, p.ink, { size: 11, weight: 600 });

      // --- scatter panel: (h*, ĥ)
      const qx = sx(v, PANEL_SCAT.x0);
      const qy = sy(v, PANEL_SCAT.y1);
      const qw = sx(v, PANEL_SCAT.x1) - qx;
      const qh = sy(v, PANEL_SCAT.y0) - qy;
      let max = 0;
      for (let c = 0; c < g.size; c++) if (Number.isFinite(s.hStar[c])) max = Math.max(max, s.hStar[c], s.h[c]);
      max *= 1.05;
      const X = (x: number) => qx + (x / max) * qw;
      const Y = (y: number) => qy + qh - (y / max) * qh;
      ctx.save();
      ctx.strokeStyle = p.wall;
      ctx.lineWidth = 1.5;
      ctx.strokeRect(qx, qy, qw, qh);
      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = p.ink;
      ctx.globalAlpha = 0.5;
      ctx.beginPath();
      ctx.moveTo(X(0), Y(0));
      ctx.lineTo(X(max), Y(max));
      ctx.stroke();
      if (s.epsilon > 0) {
        ctx.strokeStyle = p.goal;
        ctx.globalAlpha = 0.8;
        ctx.beginPath();
        ctx.moveTo(X(0), Y(0));
        const xe = Math.min(max, max / (1 + s.epsilon));
        ctx.lineTo(X(xe), Y(xe * (1 + s.epsilon)));
        ctx.stroke();
      }
      ctx.restore();
      for (let c = 0; c < g.size; c++) {
        if (!g.isFree(c) || !Number.isFinite(s.hStar[c]) || s.hStar[c] <= 0) continue;
        const over = s.h[c] > s.hStar[c] + 1e-9;
        const onPath = path.includes(c);
        ctx.save();
        ctx.globalAlpha = over ? 0.85 : 0.45;
        ctx.fillStyle = over ? p.goal : onPath ? p.path : p.graph;
        ctx.beginPath();
        ctx.arc(X(s.hStar[c]), Y(s.h[c]), onPath ? 3 : 2, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
      }
      label(ctx, 'h*(v) →', qx + qw - 4, qy + qh + 12, p.ink, { size: 10, align: 'right' });
      label(ctx, '↑ ĥ(v)', qx + 4, qy + 10, p.ink, { size: 10 });
      label(ctx, `${s.violations} of ${s.reachable} nodes above the diagonal · ε = ${s.epsilon.toFixed(3)}`, qx + 4, qy - 9, s.violations ? p.goal : p.ink, { size: 11, weight: 600 });
      label(ctx, s.epsilon > 0 ? `red line: (1 + ε) h*` : 'admissible: nothing above the diagonal', qx + qw - 4, qy + 24, p.ink, { size: 10, align: 'right' });
    },
    [s, kind, scale],
  );

  return (
    <WidgetFrame
      id="w22.2"
      title="Heuristic Calibration"
      teaches="A more informed heuristic is not a better one: the inadmissible cloud expands fewer nodes and returns a longer path — and the ε by which it overestimates is the exact bound on what you lose."
      colorKey={['start', 'goal', 'cobstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Chapter 6&rsquo;s A* (with reopening) on Reach&rsquo;s torus lattice, {s.graph.n} × {s.graph.n} cells of 15°. Left,
          the explored set grows a few expansions per frame; red marks the nodes where ĥ exceeds the true cost-to-go
          h*, which Dijkstra from the goal labelled exactly. Right, every reachable node at (h*, ĥ): the admissible
          octile heuristic never crosses the diagonal, the learned one does. The measured ε is the worst ratio on
          this world; Derivation 3 then promises the returned cost stays under (1 + ε)·C*, and the tile below shows
          both numbers. Slide the scale above 1 to buy expansions with path length; switch heuristics; re-roll for
          a new world and query. The deflated variant divides by (1 + ε_val) with ε_val = {EPSILON_VALIDATION.toFixed(3)}
          from the validation suite — admissible there, and not necessarily here.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.2} padding={0.08} draw={draw} deps={[sim.tick, s, kind, scale]} ariaLabel="A torus lattice with A*'s explored set and path, and beside it a scatter of learned heuristic values against true cost-to-go with the diagonal." />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="expansions (final)" value={s.final.expansions} precision={0} role="graph" trend={s.octile.expansions} trendLabel="with octile" />
          <StatTile label="cost ratio C / C*" value={Number.isFinite(ratio) ? ratio : '—'} precision={4} role="path" trend={s.dijkstra.expansions} trendLabel="Dijkstra expansions" />
          <StatTile label="measured ε (this world)" value={s.epsilon} precision={4} role="goal" trend={s.violations} trendLabel="nodes over" />
          <StatTile label="F3 bound (1 + ε) C*" value={bound} precision={3} unit="rad" role="robot" trend={s.final.found ? Number(s.final.cost.toFixed(3)) : undefined} trendLabel={s.final.found ? (s.final.cost <= bound + 1e-9 ? 'realized C ≤ bound ✓' : 'realized C > bound ✗') : undefined} />
        </Dashboard>
        <p className="mt-2 text-xs text-fd-muted-foreground">
          C* = {cStar.toFixed(3)} rad by Dijkstra; octile A* returned {s.octile.cost.toFixed(3)} in {s.octile.expansions} expansions; {s.final.reopened} node{s.final.reopened === 1 ? '' : 's'} reopened under the current heuristic.
        </p>
      </div>
      <ControlPanel columns={3} title="headline — heuristic scale">
        <Slider label="scale s on ĥ" role="goal" value={scale} min={0.5} max={1.5} step={0.05} onChange={setScale} format={(x) => `× ${x.toFixed(2)}`} help="s · ĥ. Above 1 even the octile heuristic becomes inadmissible — weighted A*." />
        <ButtonRow>
          {KINDS.map((k) => (
            <ActionButton key={k.kind} emphasis={kind === k.kind} onClick={() => setKind(k.kind)}>
              {k.label}
            </ActionButton>
          ))}
        </ButtonRow>
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
