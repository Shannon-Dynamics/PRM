'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Vec } from '@/lib/prob/linalg';
import { bracketAt, detColumns, foliationExample, sampleReachable, type ControlAffine } from '@/lib/nonholonomic';
import { drawDot } from './draw-car';

/**
 * w20.3 — Foliation Leaves (Choset Example 12.1.7).
 *
 * g₁ = [x₁ cos x₃, x₂ sin x₃, 0]ᵀ, g₂ = ∂/∂x₃, det[g₁ g₂ [g₁, g₂]] = x₁x₂. Drag
 * the start point in the (x₁, x₂) plane (x₃ on the slider): random control
 * sequences shade the leaf the system is confined to — a line on the x₃-axis,
 * a half-plane when one coordinate is zero, a 3-dimensional quadrant otherwise.
 * The determinant is live, and the rank is a property of the *point*.
 */

const VIEW = { minX: 0, minY: 0, maxX: 10, maxY: 5 };
const HALF = 2.0;
const SNAP = 0.12;
const BATCH = 20;
const CAP = 6000;

interface Sim {
  pts: Vec[];
  rng: Rng;
  sig: string;
}

export function FoliationLeaves() {
  const ex = useMemo(() => foliationExample(), []);
  const sys: ControlAffine = useMemo(() => ({ name: 'ex 12.1.7', n: 3, drift: null, controls: [ex.g1, ex.g2], controlSet: 'U±' }), [ex]);
  const [start, setStart] = useState<Vec>([0.9, 0.7, 0.4]);
  const [dragging, setDragging] = useState(false);
  const sig = start.map((v) => v.toFixed(3)).join(',');

  const init = useCallback((seed: number): Sim => ({ pts: [], rng: new Rng(seed), sig }), [sig]);
  const step = useCallback(
    (s0: Sim): Sim => {
      const s = s0.sig === sig ? s0 : { pts: [], rng: s0.rng, sig };
      if (s.pts.length >= CAP) return s;
      const fresh = sampleReachable(sys, start, { T: 2.5, radius: 3.5, samples: BATCH, segments: 5, substeps: 4, keepAll: true }, () => s.rng.next());
      return { pts: s.pts.concat(fresh), rng: s.rng, sig };
    },
    [sys, start, sig],
  );
  const sim = useSimulation<Sim>({ init, step, fps: 20 });

  const det = detColumns([ex.g1.eval(start), ex.g2.eval(start), bracketAt(ex.g1, ex.g2, start)]);
  const leaf = ex.leafDimension(start);

  // Panel geometry: left (x₁, x₂), right (x₁, x₃).
  const panels = useMemo(
    () => [
      { cx: 2.6, cy: 2.5, title: '(x₁, x₂) — drag the start', pick: (q: Vec): [number, number] => [q[0], q[1]], xl: 'x₁ →', yl: '↑ x₂' },
      { cx: 7.4, cy: 2.5, title: '(x₁, x₃)', pick: (q: Vec): [number, number] => [q[0], q[2]], xl: 'x₁ →', yl: '↑ x₃' },
    ],
    [],
  );
  const S = HALF / 2.6; // world units per state unit (state box ±2.6)

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      for (const pn of panels) {
        const X = (a: number) => sx(v, pn.cx + a * S);
        const Y = (b: number) => sy(v, pn.cy + b * S);
        ctx.save();
        ctx.strokeStyle = p.grid;
        ctx.strokeRect(sx(v, pn.cx - HALF), sy(v, pn.cy + HALF), sl(v, 2 * HALF), sl(v, 2 * HALF));
        // Coordinate planes where the distribution loses rank.
        ctx.strokeStyle = p.obstacle;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(X(0), sy(v, pn.cy - HALF));
        ctx.lineTo(X(0), sy(v, pn.cy + HALF));
        if (pn.yl.includes('x₂')) {
          ctx.moveTo(sx(v, pn.cx - HALF), Y(0));
          ctx.lineTo(sx(v, pn.cx + HALF), Y(0));
        }
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = p.graph;
        ctx.globalAlpha = 0.3;
        for (const q of sim.state.pts) {
          const [a, b] = pn.pick(q);
          if (Math.abs(a) > 2.6 || Math.abs(b) > 2.6) continue;
          ctx.fillRect(X(a) - 1, Y(b) - 1, 2, 2);
        }
        ctx.globalAlpha = 1;
        ctx.restore();
        const [a0, b0] = pn.pick(start);
        drawDot(ctx, v, pn.cx + a0 * S, pn.cy + b0 * S, 0.07, p.start);
        label(ctx, pn.title, sx(v, pn.cx - HALF) + 4, sy(v, pn.cy + HALF) + 10, p.ink, { size: 10, weight: 600 });
        label(ctx, pn.xl, sx(v, pn.cx + HALF) - 4, sy(v, pn.cy - HALF) - 8, p.ink, { size: 9, align: 'right' });
        label(ctx, pn.yl, sx(v, pn.cx - HALF) + 4, sy(v, pn.cy - HALF) - 8, p.ink, { size: 9 });
      }
      const where = leaf === 3 ? 'a 3-dimensional quadrant: rank 3, STLC here' : leaf === 2 ? 'a coordinate half-plane: rank 2 — a 2-dimensional leaf' : 'the x₃-axis: rank 1 — a line';
      label(ctx, `start x = (${start.map((t) => t.toFixed(2)).join(', ')})   det[g₁ g₂ [g₁,g₂]] = x₁x₂ = ${det.toFixed(3)}   →  ${where}`, 10, v.height - 10, Math.abs(det) < 1e-9 ? p.goal : p.ink, { size: 10 });
    },
    [sim.state.pts, start, det, leaf, panels, S],
  );

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const pn = panels[0];
      const a = (w[0] - pn.cx) / S;
      const b = (w[1] - pn.cy) / S;
      if (phase === 'down') {
        if (Math.abs(a) <= 2.6 && Math.abs(b) <= 2.6) setDragging(true);
        else return;
      }
      if (phase === 'up') {
        setDragging(false);
        return;
      }
      if (!dragging && phase === 'move') return;
      const snap = (t: number) => (Math.abs(t) < SNAP ? 0 : Math.max(-2.5, Math.min(2.5, t)));
      setStart((s) => [snap(a), snap(b), s[2]]);
    },
    [dragging, panels, S],
  );

  return (
    <WidgetFrame
      id="w20.3"
      title="Foliation Leaves"
      teaches="A rank test at one configuration settles controllability only there: the same two fields give a line, a half-plane or a full quadrant depending on where you start."
      colorKey={['start', 'graph', 'obstacle']}
      caption={
        <>
          Choset&rsquo;s Example 12.1.7: g₁ = [x₁ cos x₃, x₂ sin x₃, 0]ᵀ and g₂ = ∂/∂x₃. Drag the green start onto a dashed
          coordinate plane (it snaps) and the shaded leaf collapses from a quadrant to a half-plane; onto the x₃-axis and it is a line.
          Nine leaves in all: one line, four half-planes, four quadrants. The determinant x₁x₂ drops to zero exactly where the rank does.
        </>
      }
    >
      <SimCanvas world={VIEW} draw={draw} deps={[sim.tick, start]} aspect={2} padding={0.1} ariaLabel="Two projections of the states reachable from a draggable start point under the two vector fields of Choset's Example 12.1.7, shading the leaf the system is confined to." onPointer={onPointer} />
      <ControlPanel columns={2}>
        <Slider label="x₃ (start)" value={start[2]} min={-Math.PI} max={Math.PI} step={0.01} onChange={(x3) => setStart((s) => [s[0], s[1], x3])} unit="rad" />
        <div className="font-ui text-[0.72rem] text-fd-muted-foreground">Presets:
          {[
            ['quadrant', [0.9, 0.7, 0.4]],
            ['half-plane x₁ = 0', [0, 0.9, 0.4]],
            ['axis', [0, 0, 0.4]],
          ].map(([name, q]) => (
            <button key={name as string} type="button" className="ml-2 rounded-sm border border-fd-border bg-fd-card px-2 py-0.5 text-xs hover:bg-fd-accent" onClick={() => setStart(q as Vec)}>
              {name as string}
            </button>
          ))}
        </div>
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={sim.reseed} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
