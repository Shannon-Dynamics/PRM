'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Point2 } from '@/lib/sim/world';
import { KIDNEY, polygonPerimeter, traceCurve, witness, type Trace } from '@/lib/bugs';
import { X, Y, drawArrow, drawPolygons, drawPolyline } from './draw-bugs';

/**
 * w3.2 — the Boundary Follower.
 *
 * Choset's predictor–corrector (§2.3.3, eq. 2.5, Appendix D) one hop at a time
 * on a smooth nonconvex obstacle: a tangent prediction Δλ along (∇D)^⊥, the
 * correcting line along ∇D, and each Newton iterate hopping back onto the
 * offset curve G = D − W* = 0 with residuals that square at every step.
 *
 * The misconception it kills: "following a wall is trivial". It is
 * root-finding, and its guarantee has hypotheses — into the dent, where the
 * offset set has a cusp, D_Y G collapses and the tracer stops and says so.
 */

const FRAME = { ox: 0, oy: 0 };
const A = 1.5;
const FINE = 0.02;

interface State {
  /** The trace the widget replays, one record per tick. */
  trace: Trace;
  k: number;
}

const PRESETS = [
  { id: 'smooth', label: 'W* = 0.5 — passes the dent', w: 0.5 },
  { id: 'cusp', label: 'W* = 0.25 — into the dent (cusp)', w: 0.25 },
] as const;

export function BoundaryFollower() {
  const [stepLen, setStepLen] = useState(0.2);
  const [wStar, setWStar] = useState(0.5);

  const G = useMemo(() => (p: Point2) => witness(KIDNEY, p).d - wStar, [wStar]);
  const DG = useMemo(() => (p: Point2) => witness(KIDNEY, p).grad, []);
  const start = useMemo<Point2>(() => ({ x: 0, y: -(A + wStar) }), [wStar]);

  /** The level set itself, traced finely, for the blue reference curve. */
  const reference = useMemo(() => traceCurve(G, DG, start, 'left', { step: FINE, maxSteps: 4000 }), [G, DG, start]);

  const init = useCallback(
    (): State => ({ trace: traceCurve(G, DG, start, 'left', { step: stepLen, maxSteps: 2000, newtonIters: 4 }), k: 0 }),
    [G, DG, start, stepLen],
  );
  const step = useCallback((s: State): State => (s.k < s.trace.records.length ? { ...s, k: s.k + 1 } : s), []);
  const sim = useSimulation<State>({ init, step, fps: 5, initialSeed: 3 });

  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [stepLen, wStar]);

  const perimeter = useMemo(() => polygonPerimeter(KIDNEY.obstacles[0]), []);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      const { trace, k } = sim.state;
      drawPolygons(ctx, v, FRAME, KIDNEY.obstacles, p);

      // The offset curve G⁻¹(0): a one-dimensional roadmap in disguise.
      drawPolyline(ctx, v, FRAME, reference.points, p.graph, { width: 1.2, alpha: 0.55, close: reference.stop === 'closed' });

      // Samples already produced.
      const done = trace.points.slice(0, Math.min(k + 1, trace.points.length));
      drawPolyline(ctx, v, FRAME, done, p.robot, { width: 1.6, alpha: 0.9 });
      ctx.save();
      ctx.fillStyle = p.robot;
      for (const q of done) {
        ctx.beginPath();
        ctx.arc(X(v, FRAME, q.x), Y(v, FRAME, q.y), 2, 0, 2 * Math.PI);
        ctx.fill();
      }
      ctx.restore();

      // The step being shown: prediction, correcting line, Newton hops.
      const rec = trace.records[Math.min(k, trace.records.length - 1)];
      if (rec && k < trace.records.length) {
        drawArrow(ctx, v, FRAME, rec.from, rec.predicted, p.robot, 1.6);
        const n = { x: rec.tangent.y, y: -rec.tangent.x }; // ±∇G: the correcting line
        const far = 1.2 * Math.max(stepLen, 0.3);
        drawPolyline(
          ctx,
          v,
          FRAME,
          [
            { x: rec.predicted.x - far * n.x, y: rec.predicted.y - far * n.y },
            { x: rec.predicted.x + far * n.x, y: rec.predicted.y + far * n.y },
          ],
          p.path,
          { dashed: true, width: 1, alpha: 0.8 },
        );
        ctx.save();
        let prev = rec.predicted;
        rec.corrections.forEach((c, i) => {
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 1.4;
          ctx.beginPath();
          ctx.moveTo(X(v, FRAME, prev.x), Y(v, FRAME, prev.y));
          ctx.lineTo(X(v, FRAME, c.x), Y(v, FRAME, c.y));
          ctx.stroke();
          ctx.fillStyle = p.path;
          ctx.beginPath();
          ctx.arc(X(v, FRAME, c.x), Y(v, FRAME, c.y), Math.max(1.5, 5 - 1.5 * i), 0, 2 * Math.PI);
          ctx.fill();
          prev = c;
        });
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.arc(X(v, FRAME, rec.predicted.x), Y(v, FRAME, rec.predicted.y), 4, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.restore();
        label(ctx, `y₀ = x + Δλ·t`, X(v, FRAME, rec.predicted.x) + 8, Y(v, FRAME, rec.predicted.y) - 8, p.robot, { size: 10 });
      }

      const status =
        k >= trace.records.length
          ? trace.stop === 'closed'
            ? `closed after ${trace.points.length - 1} steps · length ${trace.length.toFixed(3)}`
            : trace.stop === 'singular'
              ? `singular after ${trace.records.length} steps: |D_Y G| = ${Math.abs(trace.records[trace.records.length - 1]?.dyG ?? 0).toFixed(3)} — the correcting line is tangent to the level set`
              : trace.stop
          : `step ${k + 1} of ${trace.records.length}`;
      label(ctx, status, 8, v.height - 12, trace.stop === 'singular' && k >= trace.records.length ? p.goal : p.ink, { size: 11, weight: 600 });
      label(ctx, `W* = ${wStar.toFixed(2)}   Δλ = ${stepLen.toFixed(2)}   obstacle perimeter ${perimeter.toFixed(2)}`, 8, 14, p.ink, { size: 10 });
    },
    [sim.state, reference, stepLen, wStar, perimeter],
  );

  const s = sim.state;
  const shown = s.trace.records[Math.min(s.k, s.trace.records.length - 1)];

  return (
    <WidgetFrame
      id="w3.2"
      title="Boundary Follower"
      teaches="Following a wall is root-finding: predict along the tangent, correct with Newton, and respect the hypotheses of the theorem that makes it work."
      colorKey={['obstacle', 'graph', 'path', 'robot']}
      caption={
        <>
          The blue curve is the offset curve <em>G</em>(<em>x</em>) = <em>D</em>(<em>x</em>) − <em>W</em>* = 0 around a
          smooth obstacle with a dent on its west side. Each tick the tracer predicts a step Δλ along
          the tangent (∇<em>D</em>)<sup>⊥</sup> (orange arrow), lands off the curve, and hauls itself back along
          the correcting line (dashed purple) with Newton iterates (purple dots, shrinking). The
          residual strip prints |<em>G</em>(<em>y</em><sub>h</sub>)| per iterate: watch the exponents double —
          10<sup>−2</sup>, 10<sup>−4</sup>, 10<sup>−8</sup>, machine precision. Small Δλ barely needs
          correction; large Δλ falls off visibly and is caught in two iterations. Now lower{' '}
          <em>W</em>* to 0.25: the curve enters the dent, whose radius of curvature at the bottom is about
          0.05, so the offset set has a cusp there. ∇<em>D</em> swings through nearly 180° within one
          step, <em>D</em><sub>Y</sub><em>G</em> = ∇<em>G</em>·<em>n</em> collapses toward zero, and the tracer stops
          and reports <em>singular</em> rather than jumping to the other branch. That is Theorem D.1.1&rsquo;s
          hypothesis failing in public.
        </>
      }
    >
      <SimCanvas
        world={KIDNEY.bounds}
        draw={draw}
        deps={[sim.tick, sim.state, stepLen, wStar]}
        aspect={1}
        padding={0.2}
        ariaLabel="A kidney-shaped obstacle with its offset curve drawn around it. A tracer advances along the curve one predictor step at a time; each step shows the tangent prediction, the correcting line and the Newton iterates returning to the curve."
      />

      <div className="border-t border-fd-border px-3 py-2 font-mono text-[0.75rem] tabular-nums">
        <span className="eyebrow mr-3">residuals |G(y_h)|</span>
        {shown ? (
          shown.residuals.map((r, h) => (
            <span key={h} className="mr-3 inline-block">
              h={h}: {r === 0 ? '0' : r.toExponential(1)}
            </span>
          ))
        ) : (
          <span>—</span>
        )}
        {shown ? <span className="ml-2 text-fd-muted-foreground">D_Y G = {shown.dyG.toFixed(3)}</span> : null}
      </div>

      <ControlPanel columns={2}>
        <Slider
          label="Predictor step Δλ"
          role="robot"
          value={stepLen}
          min={0.05}
          max={0.8}
          step={0.01}
          onChange={setStepLen}
          help="The tangent prediction. Larger steps fall further off the curve and need more correction."
        />
        <Slider label="Offset W*" role="graph" value={wStar} min={0.15} max={0.9} step={0.01} onChange={setWStar} />
        <ButtonRow>
          {PRESETS.map((pr) => (
            <ActionButton key={pr.id} onClick={() => setWStar(pr.w)} emphasis={Math.abs(wStar - pr.w) < 1e-9}>
              {pr.label}
            </ActionButton>
          ))}
        </ButtonRow>
      </ControlPanel>

      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}
