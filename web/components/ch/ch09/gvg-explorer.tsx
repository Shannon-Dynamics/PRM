'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawRobot, drawWorld, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT, isFree, type Point2 } from '@/lib/sim/world';
import { Rng } from '@/lib/prob/rng';
import { GvgExplorer } from '@/lib/roadmap/explore';
import { ScanQuery, apartmentExactGvd } from '@/lib/roadmap/scan-query';
import { drawExplorerGraph, polyline } from './shared';
import { marker } from '@/components/ch/ch07/shared';

/**
 * w9.1 — the GVG Explorer.
 *
 * Rusty drops into the Apartment with nothing but its range ring and builds
 * the generalized Voronoi graph by driving it: gradient ascent to a
 * two-equidistant point, the Lyapunov law along each edge, homing onto
 * every meet point, depth-first search over the departure slots the robot
 * has not yet taken. The two (then three) shortest rays are highlighted,
 * Γ = ½ GᵀG runs in a sparkline, and the exact GVD of Chapter 8 is drawn
 * faintly underneath so the reader can see the sensor quantity land on the
 * geometric one. The headline is β: too gentle and the robot drifts off the
 * edge before being pulled back; too stiff and Euler's steps overshoot.
 */

interface State {
  ex: GvgExplorer;
  sensor: ScanQuery;
  events: string[];
}

const TICKS_PER_FRAME = 3;

function randomStart(rng: Rng): Point2 {
  for (let k = 0; k < 200; k++) {
    const x = rng.uniform(0.5, 11.5);
    const y = rng.uniform(0.5, 8.5);
    if (isFree(APARTMENT, x, y, 0.35)) return { x, y };
  }
  return { x: 1, y: 1.2 };
}

export function GvgExplorerWidget() {
  const [beta, setBeta] = useState(-3);
  const [alpha, setAlpha] = useState(1);
  const [sigma, setSigma] = useState(0);
  const [showExact, setShowExact] = useState(true);
  const [showRays, setShowRays] = useState(true);
  const [start, setStart] = useState<Point2 | null>(null);
  const params = useRef({ beta, alpha, sigma, start });
  params.current = { beta, alpha, sigma, start };

  const exact = useMemo(() => apartmentExactGvd(APARTMENT, 0.03, 36), []);

  const init = useCallback((seed: number): State => {
    const rng = new Rng(seed);
    const q0 = params.current.start ?? (seed === 9 ? { x: 1.0, y: 1.2 } : randomStart(rng));
    const sensor = new ScanQuery(APARTMENT, { nRays: 360, maxRange: 6, sigma: params.current.sigma }, rng.fork());
    const ex = new GvgExplorer(sensor.query, [q0.x, q0.y], {
      alpha: params.current.alpha,
      beta: params.current.beta,
      dt: 0.03,
      meetTol: 0.08,
      boundaryTol: 0.12,
      nodeTol: 0.015 + 2 * params.current.sigma,
      homeTol: Math.max(1e-4, 3 * params.current.sigma),
      maxTicks: 200000,
    });
    return { ex, sensor, events: [] };
  }, []);

  const step = useCallback((s: State): State => {
    // Gains are live: the reader changes β and the law changes under the robot.
    (s.ex.opts as { beta: number; alpha: number }).beta = params.current.beta;
    (s.ex.opts as { beta: number; alpha: number }).alpha = params.current.alpha;
    s.sensor.sigma = params.current.sigma;
    for (let k = 0; k < TICKS_PER_FRAME; k++) {
      const ev = s.ex.tick();
      if (ev.kind === 'meet') s.events.push(`meet point ${ev.node} at t=${s.ex.ticks}`);
      else if (ev.kind === 'boundary') s.events.push(`boundary point at t=${s.ex.ticks}`);
      else if (ev.kind === 'spurious') s.events.push(`spurious tie rejected at t=${s.ex.ticks}`);
      else if (ev.kind === 'done') s.events.push(`complete at t=${s.ex.ticks}`);
      if (s.events.length > 6) s.events.shift();
      if (s.ex.phase === 'done' || s.ex.phase === 'stuck') break;
    }
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 9 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [sigma, start]);

  const onPointer = useCallback((w: [number, number], phase: 'down' | 'move' | 'up') => {
    if (phase !== 'down') return;
    if (isFree(APARTMENT, w[0], w[1], 0.3)) setStart({ x: w[0], y: w[1] });
  }, []);

  const { ex, sensor } = sim.state;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      if (showExact) {
        for (const e of exact.edges) polyline(ctx, v, e.points, p.graph, 1, 0.18);
        for (const m of exact.meetPoints) marker(ctx, sx(v, m.q.x), sy(v, m.q.y), 2.5, p.graph);
      }
      drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
      // The driven trail.
      polyline(ctx, v, ex.trail, p.robot, 1, 0.35);
      drawExplorerGraph(ctx, v, p, ex);
      // The range ring: faint rays, the two (three while homing) shortest highlighted.
      const scan = sensor.lastScan;
      const q = ex.q;
      if (scan && showRays) {
        ctx.save();
        ctx.strokeStyle = p.robot;
        ctx.globalAlpha = 0.1;
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let k = 0; k < scan.ranges.length; k += 4) {
          const r = scan.ranges[k];
          if (!Number.isFinite(r)) continue;
          ctx.moveTo(sx(v, scan.x.x), sy(v, scan.x.y));
          ctx.lineTo(sx(v, scan.x.x + r * Math.cos(scan.angles[k])), sy(v, scan.x.y + r * Math.sin(scan.angles[k])));
        }
        ctx.stroke();
        ctx.restore();
      }
      const highlight = ex.phase === 'home' ? 3 : ex.phase === 'access' ? ex.accessTies : 2;
      sensor.lastNearest.slice(0, highlight).forEach((n, i) => {
        if (!n.closest) return;
        polyline(ctx, v, [q, n.closest], i === 2 ? p.goal : p.path, 2, 0.9);
        marker(ctx, sx(v, n.closest[0]), sy(v, n.closest[1]), 3, i === 2 ? p.goal : p.path);
      });
      drawRobot(ctx, v, { x: q[0], y: q[1], theta: Math.atan2(ex.heading[1], ex.heading[0]) }, p.robot, 0.3);
      const d = sensor.lastNearest;
      const readings = d.slice(0, 3).map((n) => n.d.toFixed(3)).join(' / ');
      label(
        ctx,
        `${ex.phase} · ${ex.meetPoints.length} meet points · ${ex.nodes.filter((n) => n.kind === 'boundary').length} boundary points · ${ex.edges.filter((e) => e.to >= 0).length} edges · frontier ${ex.frontier} · driven ${ex.distanceDriven.toFixed(1)} m`,
        10,
        14,
        p.ink,
        { size: 10.5, weight: 600 },
      );
      label(ctx, `d₁ / d₂ / d₃ = ${readings}${ex.last ? ` · G = ${ex.last.G.map((g) => g.toFixed(3)).join(', ')} · Γ = ${ex.last.gamma.toExponential(2)}` : ''}`, 10, 28, p.ink, { size: 10 });
      label(ctx, 'click an open spot to restart from there', 10, v.height - 10, p.ink, { size: 9 });
    },
    [ex, sensor, exact, showExact, showRays],
  );

  const drawSpark = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const g = ex.gammaHistory;
      const n = g.length;
      const maxG = Math.max(1e-6, ...g);
      ctx.save();
      ctx.strokeStyle = p.graph;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let k = 0; k < n; k++) {
        const x = (k / Math.max(1, 399)) * v.width;
        const y = v.height - 6 - (Math.log10(1 + g[k] / 1e-6) / Math.log10(1 + maxG / 1e-6)) * (v.height - 14);
        if (k === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.restore();
      label(ctx, `Γ = ½GᵀG along the last ${n} trace ticks (log scale) · max ${maxG.toExponential(2)}${ex.last ? ` · Γ̇ = βGᵀG = ${ex.last.dGamma.toExponential(2)}` : ''}`, 8, 11, p.ink, { size: 9.5 });
    },
    [ex],
  );

  return (
    <WidgetFrame
      id="w9.1"
      title="GVG Explorer"
      teaches="A roadmap is a sensor quantity: three ranges tie, and the robot is on it. Exploration is depth-first search over a graph the robot is still drawing."
      colorKey={['obstacle', 'graph', 'path', 'goal', 'robot']}
      wide
      caption={
        <>
          Rusty (orange) has no map. The purple rays are the two shortest returns of its range ring; the
          tracing law q̇ = αv + β(DG)†G keeps their difference at zero while advancing, so the robot
          drives along the GVG and draws it (blue). When a third return (red) ties, the homing law runs
          Newton onto the meet point; the number inside a hollow node is how many of its edges are
          still unexplored, and the robot backtracks along edges it already knows to reach them. Grey
          dots are boundary points, where an edge runs into a corner. The faint blue curves underneath
          are Chapter 8&rsquo;s exact GVD of the same walls, traced from geometry; the sensor&rsquo;s graph
          lands on them. Soften β toward −0.1 and watch the correction lag behind the curvature of the
          parabolic edges; raise the sensor noise and watch meet points be announced early, late or
          twice — the detector is a heuristic, and the text says so.
        </>
      }
    >
      <SimCanvas world={APARTMENT.bounds} draw={draw} deps={[sim.tick, showExact, showRays]} aspect={12 / 9} padding={0.3} onPointer={onPointer} cursor="pointer" ariaLabel="The Apartment floorplan; an orange robot drives along a growing blue graph of equidistant curves, with its two shortest range rays drawn in purple and a faint exact Voronoi diagram underneath." />
      <SimCanvas world={{ minX: 0, minY: 0, maxX: 1, maxY: 1 }} draw={drawSpark} deps={[sim.tick]} aspect={12} padding={0} ariaLabel="A sparkline of the Lyapunov function along the traced edge." />
      <ControlPanel columns={3} title="Headline: correction gain β">
        <Slider label="β (correction gain, 1/s)" role="graph" value={beta} min={-5} max={-0.1} step={0.1} onChange={setBeta} />
        <Slider label="α (speed along the edge, m/s)" role="robot" value={alpha} min={0.3} max={2} step={0.1} unit="m/s" onChange={setAlpha} />
        <Slider label="range noise σ" role="goal" value={sigma} min={0} max={0.03} step={0.001} unit="m" format={(x) => `${(x * 100).toFixed(1)} cm`} onChange={setSigma} />
      </ControlPanel>
      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Layers · start pose · event log</summary>
        <ControlPanel columns={3}>
          <div className="flex flex-col gap-1.5">
            <Toggle label="exact GVD (Chapter 8)" role="graph" checked={showExact} onChange={setShowExact} />
            <Toggle label="range rays" role="robot" checked={showRays} onChange={setShowRays} />
          </div>
          <ButtonRow>
            <ActionButton onClick={() => setStart({ x: 1.0, y: 1.2 })}>start in room A</ActionButton>
            <ActionButton onClick={() => setStart(null)}>random start (seed)</ActionButton>
          </ButtonRow>
          <div className="font-mono text-[0.68rem] text-fd-muted-foreground">
            {sim.state.events.length ? sim.state.events.map((e, i) => <div key={i}>{e}</div>) : 'no events yet'}
          </div>
        </ControlPanel>
      </details>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={ex.ticks} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
