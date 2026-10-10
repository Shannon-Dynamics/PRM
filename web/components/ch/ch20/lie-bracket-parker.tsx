'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Hitch, type HitchCfg } from '@/lib/robots/hitch';
import { Rng } from '@/lib/prob/rng';
import type { Vec } from '@/lib/prob/linalg';
import { bracketAt, car, carTrailer, fmtVec, fourFlowLoop, lieBracket, unicycle, type VectorField } from '@/lib/nonholonomic';
import { drawArrow, drawHitch, drawPolyline } from './draw-car';

/**
 * w20.1 — the Lie Bracket Parallel-Parker.
 *
 * Hitch in an empty bay follows the four-flow loop g₁ → g₂ → −g₁ → −g₂ and the
 * widget draws the four legs in orange, the residue x(4ε) − x₀ as a blue
 * arrow, and ε²[g₁, g₂](x₀) dashed beside it. The headline slider is ε: the
 * residue shrinks as ε² (the log–log inset has slope 2) and its direction
 * converges on the bracket. Field pairs: the unicycle's drive and spin on the
 * reduced car (the residue is the sideways [g₁, g₂]), or the 4-state car's g₁
 * and g₃ = [g₁, g₂], whose loop produces the sideways g₄ = [g₁, g₃]. With the
 * trailer on, the same loop runs on the 5-state system and the trailer is
 * drawn swinging. Every number comes from `fourFlowLoop` and `bracketAt`.
 */

const WORLD = { minX: -4.5, minY: -3.0, maxX: 6.5, maxY: 3.6 };
const L = 2.6;
const D_HITCH = 2.5;

type Pair = 'unicycle' | 'car';

interface Params {
  eps: number;
  pair: Pair;
  loops: number;
  overlay: boolean;
  trailer: boolean;
}

interface Plan {
  trace: Vec[];
  corners: Vec[];
  residue: Vec;
  predicted: Vec;
  remainder: number;
  bracket: Vec;
  x0: Vec;
  toCfg: (x: Vec) => HitchCfg;
  /** ‖residue‖ for the log–log inset at a ladder of ε values. */
  ladder: { eps: number; norm: number }[];
  pairLabel: string;
}

interface Sim {
  idx: number;
}

function fieldsFor(pair: Pair, trailer: boolean): { g1: VectorField; g2: VectorField; n: number; label: string } {
  if (pair === 'unicycle') {
    if (trailer) {
      // Reduced car + trailer: (x, y, θ, ψ) with drive and turn as inputs.
      const g1: VectorField = {
        name: 'g1',
        n: 4,
        eval: (x) => [Math.cos(x[2]), Math.sin(x[2]), 0, Math.sin(x[2] - x[3]) / D_HITCH],
      };
      const g2: VectorField = { name: 'g2', n: 4, eval: () => [0, 0, 1, 0] };
      return { g1, g2, n: 4, label: 'drive g₁ and turn g₂ (reduced car + trailer)' };
    }
    const u = unicycle();
    return { g1: u.drive, g2: u.spin, n: 3, label: 'drive g₁ and turn g₂ (reduced car)' };
  }
  if (trailer) {
    const t = carTrailer(L, D_HITCH);
    const g3 = lieBracket(t.drive, t.steer, 'g3');
    return { g1: t.drive, g2: g3, n: 5, label: 'drive g₁ and g₃ = [g₁, g₂] (car + trailer)' };
  }
  const c = car(L);
  const g3 = lieBracket(c.drive, c.steer, 'g3');
  return { g1: c.drive, g2: g3, n: 4, label: 'drive g₁ and g₃ = [g₁, g₂] (4-state car)' };
}

function buildPlan(params: Params, seed: number): Plan {
  const rng = new Rng(seed);
  const theta0 = rng.uniform(-0.6, 0.6);
  const { g1, g2, label: pairLabel } = fieldsFor(params.pair, params.trailer);
  // State layouts: unicycle (x, y, θ[, ψ]); car (x, y, φ, θ[, ψ]).
  const x0: Vec = params.pair === 'unicycle' ? (params.trailer ? [0, 0, theta0, theta0] : [0, 0, theta0]) : params.trailer ? [0, 0, 0, theta0, theta0] : [0, 0, 0, theta0];
  const toCfg = (x: Vec): HitchCfg => {
    if (params.pair === 'unicycle') return { body: { x: x[0], y: x[1], theta: x[2] }, trailer: params.trailer ? x[3] : null };
    return { body: { x: x[0], y: x[1], theta: x[3] }, trailer: params.trailer ? x[4] : null };
  };
  const trace: Vec[] = [x0.slice()];
  const corners: Vec[] = [x0.slice()];
  let x = x0.slice();
  let last = fourFlowLoop(g1, g2, x, params.eps, 10);
  for (let k = 0; k < params.loops; k++) {
    last = fourFlowLoop(g1, g2, x, params.eps, 10);
    trace.push(...last.trace.slice(1));
    corners.push(...last.corners.slice(1));
    x = last.corners[4];
  }
  const first = fourFlowLoop(g1, g2, x0, params.eps, 24);
  const ladder = [0.5, 0.35, 0.25, 0.18, 0.12, 0.08, 0.05, 0.03, 0.02].map((eps) => {
    const r = fourFlowLoop(g1, g2, x0, eps, 24).residue;
    return { eps, norm: Math.hypot(...r) };
  });
  return { trace, corners, residue: first.residue, predicted: first.predicted, remainder: first.remainder, bracket: bracketAt(g1, g2, x0), x0, toCfg, ladder, pairLabel };
}

export function LieBracketParker() {
  const [params, setParams] = useState<Params>({ eps: 0.3, pair: 'unicycle', loops: 1, overlay: true, trailer: false });
  const [seed, setSeed] = useState(42);
  const car0 = useMemo(() => (params.trailer ? Hitch.car().withTrailer(D_HITCH) : Hitch.car()), [params.trailer]);
  const plan = useMemo(() => buildPlan(params, seed), [params, seed]);
  const init = useCallback((): Sim => ({ idx: 0 }), []);
  const step = useCallback((s: Sim): Sim => ({ idx: s.idx + 1 }), []);
  const total = plan.trace.length + 30;
  const sim = useSimulation<Sim>({ init, step, fps: 24, maxTicks: total, loop: true });
  const idx = Math.min(sim.state.idx, plan.trace.length - 1);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      // The bay: two painted lines.
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(sx(v, -3.5), sy(v, 1.6));
      ctx.lineTo(sx(v, 3.5), sy(v, 1.6));
      ctx.moveTo(sx(v, -3.5), sy(v, -1.6));
      ctx.lineTo(sx(v, 3.5), sy(v, -1.6));
      ctx.stroke();
      ctx.restore();

      const pts = plan.trace.slice(0, idx + 1).map((q) => ({ x: q[0], y: q[1] }));
      // Start ghost.
      drawHitch(ctx, v, car0, plan.toCfg(plan.x0), p.start, { alpha: 0.55 });
      // The legs so far, with corner markers.
      drawPolyline(ctx, v, pts, p.robot, { lineWidth: 2 });
      for (let k = 0; k < plan.corners.length && k * 10 <= idx; k++) {
        const c = plan.corners[k];
        ctx.fillStyle = p.robot;
        ctx.beginPath();
        ctx.arc(sx(v, c[0]), sy(v, c[1]), 3, 0, 2 * Math.PI);
        ctx.fill();
      }
      const q = plan.trace[idx];
      drawHitch(ctx, v, car0, plan.toCfg(q), p.robot, { fill: true });

      // Residue and prediction arrows, magnified so a small ε stays visible.
      const mag = Math.max(1, Math.min(60, 0.9 / Math.max(1e-6, Math.hypot(plan.residue[0], plan.residue[1]))));
      const done = idx >= plan.trace.length - 1 || idx >= 40;
      if (done) {
        const end = plan.corners[Math.min(4, plan.corners.length - 1)];
        drawArrow(ctx, v, plan.x0[0], plan.x0[1], plan.x0[0] + mag * (end[0] - plan.x0[0]), plan.x0[1] + mag * (end[1] - plan.x0[1]), p.graph, { lineWidth: 2.5 });
        if (params.overlay) {
          drawArrow(ctx, v, plan.x0[0], plan.x0[1], plan.x0[0] + mag * plan.predicted[0], plan.x0[1] + mag * plan.predicted[1], p.graph, { lineWidth: 1.5, dashed: true });
        }
        label(ctx, `residue ×${mag.toFixed(0)}`, sx(v, plan.x0[0] + mag * (end[0] - plan.x0[0])) + 6, sy(v, plan.x0[1] + mag * (end[1] - plan.x0[1])), p.graph, { size: 10 });
      }

      // Readout.
      const y0 = 14;
      label(ctx, plan.pairLabel, 10, y0, p.ink, { size: 10, weight: 600 });
      label(ctx, `ε = ${params.eps.toFixed(3)}   [g₁,g₂](x₀) = ${fmtVec(plan.bracket, 3)}`, 10, y0 + 14, p.ink, { size: 10 });
      label(ctx, `residue  x(4ε) − x₀ = ${fmtVec(plan.residue, 4)}`, 10, y0 + 28, p.graph, { size: 10 });
      label(ctx, `ε²[g₁,g₂]           = ${fmtVec(plan.predicted, 4)}   gap ${plan.remainder.toExponential(2)} = O(ε³)`, 10, y0 + 42, p.graph, { size: 10 });

      // Log–log inset: ‖residue‖ vs ε with a slope-2 reference.
      const ix = v.width - 190;
      const iy = v.height - 130;
      const iw = 170;
      const ih = 110;
      ctx.save();
      ctx.fillStyle = p.bg;
      ctx.globalAlpha = 0.85;
      ctx.fillRect(ix, iy, iw, ih);
      ctx.globalAlpha = 1;
      ctx.strokeStyle = p.grid;
      ctx.strokeRect(ix, iy, iw, ih);
      const lx = (e: number) => ix + 12 + ((Math.log10(e) - Math.log10(0.02)) / (Math.log10(0.5) - Math.log10(0.02))) * (iw - 24);
      const norms = plan.ladder.map((l) => l.norm).filter((n) => n > 0);
      const lo = Math.log10(Math.min(...norms));
      const hi = Math.log10(Math.max(...norms));
      const ly = (n: number) => iy + ih - 14 - ((Math.log10(n) - lo) / Math.max(1e-9, hi - lo)) * (ih - 28);
      // slope-2 reference through the largest ε
      const ref0 = plan.ladder[0];
      ctx.strokeStyle = p.truth;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(lx(ref0.eps), ly(ref0.norm));
      ctx.lineTo(lx(0.02), ly(ref0.norm * (0.02 / ref0.eps) ** 2));
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.strokeStyle = p.graph;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      plan.ladder.forEach((l, i) => (i === 0 ? ctx.moveTo(lx(l.eps), ly(l.norm)) : ctx.lineTo(lx(l.eps), ly(l.norm))));
      ctx.stroke();
      ctx.fillStyle = p.robot;
      ctx.beginPath();
      ctx.arc(lx(params.eps), ly(Math.hypot(...plan.residue)), 4, 0, 2 * Math.PI);
      ctx.fill();
      label(ctx, 'log ‖residue‖ vs log ε  (dashed: slope 2)', ix + 6, iy + 8, p.ink, { size: 9 });
      label(ctx, '0.02', ix + 6, iy + ih - 5, p.ink, { size: 8 });
      label(ctx, '0.5', ix + iw - 20, iy + ih - 5, p.ink, { size: 8 });
      ctx.restore();
    },
    [plan, idx, params, car0],
  );

  return (
    <WidgetFrame
      id="w20.1"
      title="Lie Bracket Parallel-Parker"
      teaches="The net motion of drive–steer–undrive–unsteer is not zero: it is ε²[g₁, g₂] + O(ε³), and for a car that direction is sideways."
      colorKey={['start', 'robot', 'graph']}
      wide
      caption={
        <>
          Four legs in orange, the residue in blue (magnified so it stays visible), and ε²[g₁, g₂] dashed beside it. Shrink ε: the
          blue dot in the inset slides down a line of slope 2, and the two arrows align — the gap between them is the O(ε³)
          remainder, 5.0 × 10⁻⁴ at ε = 0.1 for the unicycle at x₃ = π/6. Switch to the 4-state car to see the loop of g₁ with g₃ = [g₁, g₂]
          produce the sideways g₄; add the trailer to watch a fifth coordinate ride along.
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, params]} aspect={1.75} padding={0.1} ariaLabel="Hitch following the four-flow loop in an empty bay, with the residue arrow, the bracket prediction and a log–log inset of residue norm against epsilon." />
      <ControlPanel columns={3}>
        <Slider label="ε (leg duration)" value={params.eps} min={0.02} max={0.5} step={0.005} onChange={(eps) => setParams((s) => ({ ...s, eps }))} role="graph" format={(x) => x.toFixed(3)} />
        <Slider label="loops" value={params.loops} min={1} max={12} step={1} onChange={(loops) => setParams((s) => ({ ...s, loops }))} />
        <div className="flex flex-col gap-1.5">
          <Toggle label="4-state car: pair g₁, g₃" checked={params.pair === 'car'} onChange={(c) => setParams((s) => ({ ...s, pair: c ? 'car' : 'unicycle' }))} />
          <Toggle label="ε²[g₁,g₂] overlay" checked={params.overlay} onChange={(overlay) => setParams((s) => ({ ...s, overlay }))} role="graph" />
          <Toggle label="trailer (fifth state)" checked={params.trailer} onChange={(trailer) => setParams((s) => ({ ...s, trailer }))} role="robot" />
        </div>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => {
          setSeed((s) => (s * 7919 + 13) % 100000);
          sim.reset();
        }}
        tick={sim.tick}
        seed={seed}
      />
    </WidgetFrame>
  );
}
