'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Vec } from '@/lib/prob/linalg';
import { lieBracket, sampleReachable, unicycle, type ControlAffine, type ControlSetClass } from '@/lib/nonholonomic';
import { drawDot } from './draw-car';

/**
 * w20.2 — the Reachable Set Grower.
 *
 * R^V(x, ≤ T) for the reduced car, grown by sampling piecewise-constant controls
 * and keeping every state a trajectory visits before it leaves the ball V. Two
 * views: the (x, y) projection and the (x, θ) slice. Drive alone gives a curve
 * (a leaf); drive and spin with U± give a blob containing a neighborhood of x
 * (STLC); forward-only U⁺ gives a set with interior that hugs x's boundary
 * (STLA, controllable, not STLC — Dubins' lemma in Chapter 21); adding the
 * bracket [g₁, g₂] as a "macro" input makes sideways motion first-order.
 */

const VIEW = { minX: 0, minY: 0, maxX: 10, maxY: 5 };

type Inputs = 'drive' | 'spin' | 'both' | 'macro';

interface Params {
  inputs: Inputs;
  controlSet: ControlSetClass;
  T: number;
  radius: number;
}

interface Sim {
  pts: Vec[];
  histories: number;
  rng: Rng;
  /** Parameter signature the points were grown under; a change starts over. */
  sig: string;
}

const X0: Vec = [0, 0, 0];
const BATCH = 24;
const CAP = 9000;

function systemFor(inputs: Inputs, controlSet: ControlSetClass): ControlAffine {
  const u = unicycle();
  const controls = inputs === 'drive' ? [u.drive] : inputs === 'spin' ? [u.spin] : inputs === 'both' ? [u.drive, u.spin] : [u.drive, u.spin, lieBracket(u.drive, u.spin, 'g3')];
  return { name: 'reduced car', n: 3, drift: null, controls, controlSet };
}

export function ReachableSetGrower() {
  const [params, setParams] = useState<Params>({ inputs: 'both', controlSet: 'U±', T: 0.8, radius: 0.6 });
  const sys = useMemo(() => systemFor(params.inputs, params.controlSet), [params.inputs, params.controlSet]);

  const sig = `${params.inputs}|${params.controlSet}|${params.T}|${params.radius}`;
  const init = useCallback((seed: number): Sim => ({ pts: [], histories: 0, rng: new Rng(seed), sig }), [sig]);
  const step = useCallback(
    (s0: Sim): Sim => {
      const s = s0.sig === sig ? s0 : { pts: [], histories: 0, rng: s0.rng, sig };
      if (s.pts.length >= CAP) return s;
      const fresh = sampleReachable(sys, X0, { T: params.T, radius: params.radius, samples: BATCH, segments: 3, substeps: 3, keepAll: true }, () => s.rng.next());
      return { pts: s.pts.concat(fresh), histories: s.histories + BATCH, rng: s.rng, sig };
    },
    [sys, params.T, params.radius, sig],
  );
  const sim = useSimulation<Sim>({ init, step, fps: 20 });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const R = params.radius;
      // Two panels, each a square of side 2R·1.15 centred on x₀.
      const panel = (cx: number, cy: number, half: number, title: string, pick: (q: Vec) => [number, number], xlab: string, ylab: string) => {
        const S = half / (1.15 * R);
        const X = (a: number) => sx(v, cx + a * S);
        const Y = (b: number) => sy(v, cy + b * S);
        ctx.save();
        ctx.strokeStyle = p.grid;
        ctx.lineWidth = 1;
        ctx.strokeRect(sx(v, cx - half), sy(v, cy + half), sl(v, 2 * half), sl(v, 2 * half));
        // The neighborhood V.
        ctx.strokeStyle = p.obstacle;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.arc(X(0), Y(0), sl(v, R * S), 0, 2 * Math.PI);
        ctx.stroke();
        ctx.setLineDash([]);
        // Samples.
        ctx.fillStyle = p.graph;
        ctx.globalAlpha = 0.35;
        for (const q of sim.state.pts) {
          const [a, b] = pick(q);
          ctx.fillRect(X(a) - 1, Y(b) - 1, 2, 2);
        }
        ctx.globalAlpha = 1;
        ctx.restore();
        drawDot(ctx, v, cx, cy, 0.05, p.start);
        // heading tick at the start
        ctx.save();
        ctx.strokeStyle = p.start;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(X(0), Y(0));
        ctx.lineTo(X(0.25 * R), Y(0));
        ctx.stroke();
        ctx.restore();
        label(ctx, title, sx(v, cx - half) + 4, sy(v, cy + half) + 10, p.ink, { size: 10, weight: 600 });
        label(ctx, xlab, sx(v, cx + half) - 4, sy(v, cy - half) - 8, p.ink, { size: 9, align: 'right' });
        label(ctx, ylab, sx(v, cx - half) + 4, sy(v, cy - half) - 8, p.ink, { size: 9 });
      };
      panel(2.6, 2.5, 2.2, '(x, y) projection', (q) => [q[0], q[1]], 'x →', '↑ y');
      panel(7.4, 2.5, 2.2, '(x, θ) slice  (θ scaled to V)', (q) => [q[0], (q[2] * R) / Math.max(0.3, params.T)], 'x →', '↑ θ');

      // Verdict line: what the picture is a sample of.
      const name = params.inputs === 'drive' ? 'drive only: a curve — the leaf of span{g₁}; no controllability property holds' : params.inputs === 'spin' ? 'spin only: a line in θ — Choset\'s one-dimensional foliation' : params.inputs === 'both' ? (params.controlSet === 'U±' ? 'drive + spin, U±: a full neighborhood of x — STLC' : 'drive + spin, U⁺ (forward only, left only): interior yes, but nothing behind x — STLA and controllable, not STLC') : 'drive + spin + the bracket macro [g₁, g₂]: sideways is now first-order';
      label(ctx, name, 10, v.height - 10, p.ink, { size: 10 });
      label(ctx, `${sim.state.histories} control histories · ${sim.state.pts.length} states kept inside V · T = ${params.T.toFixed(2)}`, v.width - 10, v.height - 10, p.graph, { size: 10, align: 'right' });
    },
    [sim.state, params],
  );

  return (
    <WidgetFrame
      id="w20.2"
      title="Reachable Set Grower"
      teaches="Accessible, STLA, STLC and controllable are four different sets — a curve, a patch without x inside, a blob, and a blob you must leave V to fill."
      colorKey={['start', 'graph', 'obstacle']}
      caption={
        <>
          Blue states are reachable within time T without leaving the dashed ball V. Toggle the inputs: one input gives a leaf, two with
          U± give a neighborhood of the start (STLC), two with U⁺ give a set with interior that never gets behind the start — STLA and,
          in the open plane, controllable, but not STLC. The macro adds [g₁, g₂] as if it were a third input.
        </>
      }
    >
      <SimCanvas world={VIEW} draw={draw} deps={[sim.tick, params]} aspect={2} padding={0.1} ariaLabel="Two panels of sampled reachable states for the reduced car inside a neighborhood ball: the position projection and the x–heading slice." />
      <ControlPanel columns={3}>
        <div className="flex flex-col gap-1.5">
          <span className="font-ui text-[0.72rem] font-medium">available inputs</span>
          {(['drive', 'spin', 'both', 'macro'] as Inputs[]).map((k) => (
            <label key={k} className="flex cursor-pointer items-center gap-2 font-ui text-[0.72rem]">
              <input type="radio" name="w20-2-inputs" checked={params.inputs === k} onChange={() => setParams((s) => ({ ...s, inputs: k }))} className="size-3.5 accent-fd-primary" />
              {k === 'drive' ? 'drive g₁ only' : k === 'spin' ? 'spin g₂ only' : k === 'both' ? 'drive and spin' : 'both + bracket macro'}
            </label>
          ))}
        </div>
        <div className="flex flex-col gap-2">
          <Toggle label="forward-only control set U⁺ (else U±)" checked={params.controlSet === 'U+'} onChange={(c) => setParams((s) => ({ ...s, controlSet: c ? 'U+' : 'U±' }))} role="goal" />
          <Slider label="T (horizon)" value={params.T} min={0.1} max={2} step={0.05} onChange={(T) => setParams((s) => ({ ...s, T }))} />
        </div>
        <Slider label="radius of V" value={params.radius} min={0.2} max={1.5} step={0.05} onChange={(radius) => setParams((s) => ({ ...s, radius }))} role="obstacle" unit="m" />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={sim.reseed} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
