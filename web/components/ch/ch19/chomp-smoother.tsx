'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Vec } from '@/lib/prob/linalg';
import { WORKBENCH } from '@/lib/robots/worlds';
import { chompEvaluate, chompStableEta, chompStep, straightLine, POINT_BODY, type ChompProblem, type ChompStats } from '@/lib/trajopt/chomp';
import { DiscField, EdtField, type DistanceField2 } from '@/lib/trajopt/field';
import { microEquilibrium, reachWorkbench, type ReachScene } from '@/lib/trajopt/micro';
import { stompInit, stompStep, type StompState } from '@/lib/trajopt/stomp';

/**
 * w19.1 — the CHOMP Smoother.
 *
 * Left: Reach on the Workbench over the distance field, with the body points
 * CHOMP pulls on; right: the same trajectory on the unwrapped torus. One
 * covariant step per frame. The headline is the step 1/η: below the stability
 * bound η* = ½(1 + (λ/ε)/λ_min(A)) the rubber band oscillates, far above it
 * crawls. Pick the other initial path and the optimizer converges to a
 * different answer — it polishes the homotopy class it is handed and never
 * changes its mind; from the straight chart line it converges *in collision*
 * and the badge says so. STOMP mode replaces the gradient by K seeded rollouts.
 * The micro-example preset shows the clearance converging to D* = (λ − 1)/(2(1 + λ)).
 */

const ARENA = { minX: 0, minY: 0, maxX: 10.8, maxY: 5.4 };
const ARM_CX = 2.7;
const ARM_CY = 2.7;
const TX0 = 5.9;
const TX1 = 10.6;
const TY0 = 0.5;
const TY1 = 5.2;
const M = 24;

interface Params {
  logEta: number;
  lambda: number;
  eps: number;
  initial: 0 | 1;
  stomp: boolean;
  K: number;
  micro: boolean;
  exactField: boolean;
}

interface Sim {
  xi: Vec[];
  stats: ChompStats;
  iter: number;
  stompState: StompState | null;
  history: number[];
}

export function ChompSmoother() {
  const [params, setParams] = useState<Params>({ logEta: Math.log10(8), lambda: 3, eps: 0.25, initial: 1, stomp: false, K: 16, micro: false, exactField: false });
  const eta = 10 ** params.logEta;
  const scene: ReachScene = useMemo(() => reachWorkbench(M, 0.05), []);
  const exact = useMemo(() => EdtField.fromPolyWorld(WORKBENCH, 0.05), []);
  const field: DistanceField2 = params.exactField ? exact : scene.field;

  const problem: ChompProblem = useMemo(() => {
    if (params.micro) {
      return { q0: [0, 0], qEnd: [2, 0], field: new DiscField({ x: 1, y: 0 }, 0.5), body: POINT_BODY, eps: 0.5, lambdaObs: params.lambda, eta, obstacleForm: 'pointwise' };
    }
    return { q0: scene.q0, qEnd: scene.qEnd, field, body: scene.body, eps: params.eps, lambdaObs: params.lambda, eta, obstacleForm: 'arclength' };
  }, [params.micro, params.lambda, params.eps, eta, scene, field]);

  const initialXi = useCallback((): Vec[] => (params.micro ? [[1, 0.2]] : scene.initial[params.initial].xi.map((q) => q.slice())), [params.micro, params.initial, scene]);
  const etaStar = params.micro ? chompStableEta(1, params.lambda, 0.5) : chompStableEta(M, params.lambda, params.eps);
  const seedRef = useRef(42);

  const init = useCallback(
    (seed: number): Sim => {
      seedRef.current = seed;
      const xi = initialXi();
      const stats = chompEvaluate(problem, xi).stats;
      return { xi, stats, iter: 0, stompState: params.stomp ? stompInit(problem, xi, seed) : null, history: [stats.f] };
    },
    [initialXi, problem, params.stomp],
  );
  const step = useCallback(
    (s: Sim): Sim => {
      if (s.iter >= 600) return s;
      if (params.stomp) {
        const st = s.stompState ?? stompInit(problem, s.xi, seedRef.current);
        const stats = stompStep(problem, st, { K: params.K, noise: params.micro ? 0.15 : 0.12 });
        return { xi: st.xi, stats, iter: s.iter + 1, stompState: st, history: [...s.history, stats.f].slice(-300) };
      }
      const xi = s.xi.map((q) => q.slice());
      chompStep(problem, xi);
      const stats = chompEvaluate(problem, xi).stats;
      return { xi, stats, iter: s.iter + 1, stompState: null, history: [...s.history, stats.f].slice(-300) };
    },
    [problem, params.stomp, params.K, params.micro],
  );
  const sim = useSimulation<Sim>({ init, step, fps: 20, initialSeed: 42 });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const { xi, stats, iter } = sim.state;
      const full = [problem.q0, ...xi, problem.qEnd];
      const init0 = [problem.q0, ...initialXi(), problem.qEnd];

      if (params.micro) {
        // ---- the three-waypoint micro-example, in the plane --------------------------
        const PX = (x: number) => sx(v, 1.2 + x * 4.2);
        const PY = (y: number) => sy(v, 1.6 + y * 4.2);
        ctx.fillStyle = p.obstacle;
        ctx.globalAlpha = 0.55;
        ctx.beginPath();
        ctx.arc(PX(1), PY(0), sl(v, 0.5 * 4.2), 0, 2 * Math.PI);
        ctx.fill();
        ctx.globalAlpha = 0.15;
        ctx.fillStyle = p.cobstacle;
        ctx.beginPath();
        ctx.arc(PX(1), PY(0), sl(v, (0.5 + 0.5) * 4.2), 0, 2 * Math.PI);
        ctx.fill();
        ctx.globalAlpha = 1;
        const eq = microEquilibrium(params.lambda);
        ctx.strokeStyle = p.goal;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.moveTo(PX(0), PY(eq.y));
        ctx.lineTo(PX(2), PY(eq.y));
        ctx.stroke();
        ctx.setLineDash([]);
        label(ctx, `y* = λ/(1+λ) = ${eq.y.toFixed(3)}  (D* = ${eq.clearance.toFixed(3)})`, PX(0.02), PY(eq.y) - 5, p.goal, { size: 9 });
        const drawPoly = (pts: Vec[], col: string, w: number, alpha: number) => {
          ctx.strokeStyle = col;
          ctx.lineWidth = w;
          ctx.globalAlpha = alpha;
          ctx.beginPath();
          pts.forEach((q, i) => (i === 0 ? ctx.moveTo(PX(q[0]), PY(q[1])) : ctx.lineTo(PX(q[0]), PY(q[1]))));
          ctx.stroke();
          ctx.globalAlpha = 1;
        };
        drawPoly(init0, p.graph, 1.5, 0.7);
        drawPoly(full, p.path, 3, 1);
        for (const q of full) {
          ctx.fillStyle = p.path;
          ctx.beginPath();
          ctx.arc(PX(q[0]), PY(q[1]), 5, 0, 2 * Math.PI);
          ctx.fill();
        }
        ctx.fillStyle = p.start;
        ctx.beginPath();
        ctx.arc(PX(0), PY(0), 6, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillStyle = p.goal;
        ctx.beginPath();
        ctx.arc(PX(2), PY(0), 6, 0, 2 * Math.PI);
        ctx.fill();
        label(ctx, `q₁ = (${xi[0][0].toFixed(3)}, ${xi[0][1].toFixed(3)})   D = ${stats.minClearance.toFixed(4)}`, PX(0), sy(v, 0.9), p.ink, { size: 10 });
        label(ctx, `iteration ${iter} · stable iff η > (1+λ)/2 = ${etaStar.toFixed(2)} · η = ${eta.toFixed(2)} ${eta > etaStar ? '(stable)' : '(oscillates)'}`, PX(0), sy(v, 0.55), eta > etaStar ? p.ink : p.goal, { size: 9 });
      } else {
        // ---- Workbench with the field underlay ----------------------------------------
        const X = (x: number) => sx(v, ARM_CX + x);
        const Y = (y: number) => sy(v, ARM_CY + y);
        const cells = 44;
        const cw = 4.8 / cells;
        for (let j = 0; j < cells; j++) {
          for (let i = 0; i < cells; i++) {
            const x = -2.4 + (i + 0.5) * cw;
            const y = -2.4 + (j + 0.5) * cw;
            const d = field.dist({ x, y });
            const a = d <= 0 ? 0.55 : Math.max(0, 0.42 * (1 - d / 0.8));
            if (a < 0.02) continue;
            ctx.fillStyle = d <= 0 ? p.obstacle : p.cobstacle;
            ctx.globalAlpha = a;
            ctx.fillRect(X(x - cw / 2), Y(y + cw / 2), sl(v, cw) + 0.5, sl(v, cw) + 0.5);
          }
        }
        ctx.globalAlpha = 1;
        ctx.strokeStyle = p.wall;
        ctx.lineWidth = 2;
        ctx.strokeRect(X(-2.4), Y(2.4), sl(v, 4.8), sl(v, 4.8));
        // tip traces: initial (blue), current (purple)
        const tipTrace = (pts: Vec[], col: string, w: number, alpha: number) => {
          ctx.strokeStyle = col;
          ctx.lineWidth = w;
          ctx.globalAlpha = alpha;
          ctx.beginPath();
          pts.forEach((q, i) => {
            const tip = scene.arm.tip(q);
            if (i === 0) ctx.moveTo(X(tip.x), Y(tip.y));
            else ctx.lineTo(X(tip.x), Y(tip.y));
          });
          ctx.stroke();
          ctx.globalAlpha = 1;
        };
        tipTrace(init0, p.graph, 1.5, 0.7);
        tipTrace(full, p.path, 2.5, 1);
        // arm ghosts along the current path, with body points
        for (let k = 0; k < full.length; k += 4) {
          const pts = scene.arm.fk(full[k]);
          ctx.strokeStyle = p.robot;
          ctx.globalAlpha = k === 0 || k >= full.length - 4 ? 0.9 : 0.3;
          ctx.lineWidth = 3;
          ctx.beginPath();
          pts.forEach((pt, i) => (i === 0 ? ctx.moveTo(X(pt.x), Y(pt.y)) : ctx.lineTo(X(pt.x), Y(pt.y))));
          ctx.stroke();
          ctx.globalAlpha = 1;
        }
        // body points on the middle configuration, colored by clearance
        const qm = full[Math.floor(full.length / 2)];
        for (const b of scene.body) {
          const { x } = b(qm);
          const d = field.dist(x);
          ctx.fillStyle = d < 0 ? p.goal : d < problem.eps ? p.cobstacle : p.start;
          ctx.beginPath();
          ctx.arc(X(x.x), Y(x.y), 3, 0, 2 * Math.PI);
          ctx.fill();
        }
        label(ctx, `Workbench · ${params.exactField ? 'signed EDT' : 'brushfire (Ch. 7)'} field`, X(-2.35), Y(2.55), p.ink, { size: 10, weight: 600 });
        // ---- torus panel -------------------------------------------------------------
        const TXm = (q1: number) => sx(v, TX0 + ((q1 + Math.PI) / (2 * Math.PI)) * (TX1 - TX0));
        const TYm = (q2: number) => sy(v, TY0 + ((q2 + Math.PI) / (2 * Math.PI)) * (TY1 - TY0));
        ctx.strokeStyle = p.grid;
        ctx.strokeRect(TXm(-Math.PI), TYm(Math.PI), sl(v, TX1 - TX0), sl(v, TY1 - TY0));
        const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
        const torusTrace = (pts: Vec[], col: string, w: number, alpha: number) => {
          ctx.strokeStyle = col;
          ctx.lineWidth = w;
          ctx.globalAlpha = alpha;
          ctx.beginPath();
          let prev: Vec | null = null;
          for (const q of pts) {
            const a = wrap(q[0]);
            const b = wrap(q[1]);
            if (!prev || Math.abs(a - prev[0]) > Math.PI || Math.abs(b - prev[1]) > Math.PI) ctx.moveTo(TXm(a), TYm(b));
            else ctx.lineTo(TXm(a), TYm(b));
            prev = [a, b];
          }
          ctx.stroke();
          ctx.globalAlpha = 1;
        };
        torusTrace(init0, p.graph, 1.5, 0.7);
        torusTrace(full, p.path, 2.5, 1);
        ctx.fillStyle = p.start;
        ctx.beginPath();
        ctx.arc(TXm(wrap(problem.q0[0])), TYm(wrap(problem.q0[1])), 5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillStyle = p.goal;
        ctx.beginPath();
        ctx.arc(TXm(wrap(problem.qEnd[0])), TYm(wrap(problem.qEnd[1])), 5, 0, 2 * Math.PI);
        ctx.fill();
        label(ctx, 'T² (unwrapped chart)', TXm(-Math.PI), TYm(Math.PI) - 6, p.ink, { size: 10, weight: 600 });
        label(ctx, 'q₁', TXm(Math.PI) - 12, TYm(-Math.PI) + 12, p.ink, { size: 9 });
        label(ctx, 'q₂', TXm(-Math.PI) - 14, TYm(Math.PI) + 10, p.ink, { size: 9 });
        label(ctx, `iteration ${iter} · η = ${eta.toFixed(1)} · worst-case stability bound η* = ${etaStar.toFixed(0)}`, X(-2.35), sy(v, 0.1), p.ink, { size: 9 });
      }
      // ---- cost bars ---------------------------------------------------------------------
      const bx = params.micro ? 6.2 : 6.0;
      const by = 0.25;
      const bw = 4.2;
      const total = Math.max(stats.f, 1e-9);
      const fs = stats.fSmooth / total;
      ctx.fillStyle = p.grid;
      ctx.fillRect(sx(v, bx), sy(v, by + 0.3), sl(v, bw), sl(v, 0.3));
      ctx.fillStyle = p.path;
      ctx.fillRect(sx(v, bx), sy(v, by + 0.3), sl(v, bw * fs), sl(v, 0.3));
      ctx.fillStyle = p.cobstacle;
      ctx.fillRect(sx(v, bx + bw * fs), sy(v, by + 0.3), sl(v, bw * (1 - fs)), sl(v, 0.3));
      label(ctx, `F = ${stats.f.toFixed(3)} — smooth ${stats.fSmooth.toFixed(3)} + λ·obs ${(problem.lambdaObs * stats.fObs).toFixed(3)}`, sx(v, bx), sy(v, by + 0.42), p.ink, { size: 9 });
      const converged = stats.gradInf < 1e-3;
      if (stats.minClearance < 0 && converged) label(ctx, 'CONVERGED IN COLLISION', sx(v, bx), sy(v, by - 0.0), p.goal, { size: 11, weight: 700 });
      else label(ctx, `min clearance ${stats.minClearance.toFixed(3)} · |∇F|∞ ${stats.gradInf.toExponential(1)}${converged ? ' · converged' : ''}`, sx(v, bx), sy(v, by - 0.0), stats.minClearance < 0 ? p.goal : p.ink, { size: 9 });
    },
    [sim.state, problem, params, scene, field, initialXi, eta, etaStar],
  );

  return (
    <WidgetFrame
      id="w19.1"
      title="CHOMP Smoother"
      teaches="Optimization does not find the best path — it finds the nearest stationary point of the homotopy class it started in, and can converge in collision; switch the initial path and the answer changes."
      colorKey={['graph', 'path', 'robot', 'cobstacle', 'obstacle', 'start', 'goal']}
      caption={
        <>
          One covariant step per frame. Left: Reach on the Workbench over the distance field (amber fades with distance, slate is inside an
          obstacle); the body points CHOMP pulls on are marked on the mid-path configuration — red where in collision, amber inside the margin ε.
          Right: the initial (blue) and current (purple) trajectories on the torus. Bars: smoothness versus λ·obstacle cost. The headline slider
          is the step 1/η: below the worst-case bound η* the band oscillates, far above it crawls. Try the straight-line initial path: it converges
          with a body point still inside the block, and the badge says so. The micro-example preset shows the single waypoint settling at
          D* = (λ − 1)/(2(1 + λ)) — zero clearance at λ = 1.
        </>
      }
      wide
    >
      <SimCanvas world={ARENA} draw={draw} deps={[sim.tick, params]} aspect={2} padding={0.1} ariaLabel="Reach's trajectory on the Workbench and on the torus relaxing under CHOMP's covariant gradient, with cost bars and a collision badge." />
      <ControlPanel columns={3}>
        <Slider label="step size: log₁₀ η" role="path" value={params.logEta} min={-0.3} max={3} step={0.05} format={(x) => `η = ${(10 ** x).toFixed(2)}`} onChange={(logEta) => setParams((p) => ({ ...p, logEta }))} />
        <Slider label="λ_obs" role="cobstacle" value={params.lambda} min={0.25} max={8} step={0.25} onChange={(lambda) => setParams((p) => ({ ...p, lambda }))} />
        <Slider label="margin ε" role="cobstacle" value={params.eps} min={0.05} max={0.6} step={0.01} unit="m" onChange={(eps) => setParams((p) => ({ ...p, eps }))} />
        <Toggle label={`initial path: ${scene.initial[params.initial].name}`} role="graph" checked={params.initial === 1} onChange={(b) => setParams((p) => ({ ...p, initial: b ? 1 : 0 }))} />
        <Toggle label="STOMP mode (seeded rollouts, no gradient)" checked={params.stomp} onChange={(stomp) => setParams((p) => ({ ...p, stomp }))} />
        <Slider label="STOMP rollouts K" value={params.K} min={4} max={48} step={2} onChange={(K) => setParams((p) => ({ ...p, K }))} />
        <Toggle label="micro-example preset (point robot, one disc)" checked={params.micro} onChange={(micro) => setParams((p) => ({ ...p, micro, lambda: micro ? 1 : p.lambda, logEta: micro ? 0 : Math.log10(8) }))} />
        <Toggle label="exact signed EDT instead of brushfire" checked={params.exactField} onChange={(exactField) => setParams((p) => ({ ...p, exactField }))} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
