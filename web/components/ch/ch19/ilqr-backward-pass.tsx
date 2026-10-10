'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Mat, Vec } from '@/lib/prob/linalg';
import { LOT } from '@/lib/robots/worlds';
import { backwardPass, diagMat, forwardPass, quadraticCost, totalCost, type BackwardPass } from '@/lib/trajopt/ilqr';
import { KinematicCar, rollout } from '@/lib/trajopt/models';

/**
 * w19.2 — the iLQR Backward Pass.
 *
 * Hitch's kinematic car on a nominal trajectory in the Lot. Scrub k from N down
 * to 0 and watch the backward pass build its local value model: the V_xx
 * ellipse (position block) around x̄_k, the feedforward k_k as an arrow on the
 * control, and the feedback gain K_k as a vector field — what the policy would
 * do from a state displaced from the nominal. The forward pass then rolls out
 * that *policy*, closed loop, with backtracking on α; cost drops, the ellipses
 * tighten toward the goal, and the next backward pass starts from the new
 * nominal. iLQR is not gradient descent on the control sequence: it builds a
 * feedback law at every stage and the rollout obeys it.
 */

const DT = 0.25;
const WORLD = { minX: 0, minY: 0, maxX: 16, maxY: 10 };
const START: Vec = [1.5, 1.5, 0];
const GOAL: Vec = [6.75, 7.0, Math.PI / 2]; // bay 2, nose north

interface Params {
  N: number;
  mu: number;
  goalWeight: number;
  steerWeight: number;
  ddp: boolean;
}

interface Sim {
  xs: Vec[];
  us: Vec[];
  bp: BackwardPass;
  k: number;
  costs: number[];
  alphas: number[];
  iter: number;
}

function ellipsePoints(Vxx: Mat, cx: number, cy: number, level: number): [number, number][] {
  // Position block of V_xx: ½ δxᵀ P δx = level → an ellipse. Eigen-decompose 2×2.
  const a = Vxx[0][0];
  const b = Vxx[0][1];
  const d = Vxx[1][1];
  const tr = a + d;
  const det = a * d - b * b;
  const disc = Math.sqrt(Math.max(0, (tr * tr) / 4 - det));
  const l1 = tr / 2 + disc;
  const l2 = tr / 2 - disc;
  const ang = Math.abs(b) < 1e-12 ? (a >= d ? 0 : Math.PI / 2) : Math.atan2(l1 - a, b);
  const r1 = l1 > 1e-9 ? Math.sqrt((2 * level) / l1) : 3;
  const r2 = l2 > 1e-9 ? Math.sqrt((2 * level) / l2) : 3;
  const pts: [number, number][] = [];
  for (let i = 0; i <= 48; i++) {
    const t = (i / 48) * 2 * Math.PI;
    const ex = Math.min(r1, 3) * Math.cos(t);
    const ey = Math.min(r2, 3) * Math.sin(t);
    pts.push([cx + ex * Math.cos(ang) - ey * Math.sin(ang), cy + ex * Math.sin(ang) + ey * Math.cos(ang)]);
  }
  return pts;
}

export function IlqrBackwardPass() {
  const [params, setParams] = useState<Params>({ N: 28, mu: 1e-3, goalWeight: 40, steerWeight: 1.5, ddp: false });
  const car = useMemo(() => new KinematicCar(DT), []);
  const cost = useMemo(() => quadraticCost(diagMat([0.2, 0.2, 0.05]), diagMat([0.3, params.steerWeight]), diagMat([params.goalWeight, params.goalWeight, params.goalWeight * 0.6]), GOAL), [params.goalWeight, params.steerWeight]);

  const init = useCallback((): Sim => {
    const us = Array.from({ length: params.N }, () => [1.2, 0.0]);
    const xs = rollout(car, START, us);
    const bp = backwardPass(car, cost, xs, us, params.mu, params.ddp);
    return { xs, us, bp, k: params.N, costs: [totalCost(cost, xs, us)], alphas: [], iter: 0 };
  }, [params.N, params.mu, params.ddp, car, cost]);

  // Autoplay scrubs k from N down to 0, then holds.
  const step = useCallback((s: Sim): Sim => (s.k > 0 ? { ...s, k: s.k - 1 } : s), []);
  const sim = useSimulation<Sim>({ init, step, fps: 6 });

  const forward = useCallback(() => {
    sim.setState((s) => {
      const J = s.costs[s.costs.length - 1];
      for (const alpha of [1, 0.5, 0.25, 0.125, 0.0625, 0.03125]) {
        const trial = forwardPass(car, cost, s.xs, s.us, s.bp, alpha);
        if (trial.cost < J) {
          const bp = backwardPass(car, cost, trial.xs, trial.us, params.mu, params.ddp);
          return { xs: trial.xs, us: trial.us, bp, k: params.N, costs: [...s.costs, trial.cost], alphas: [...s.alphas, alpha], iter: s.iter + 1 };
        }
      }
      return { ...s, alphas: [...s.alphas, 0] };
    });
    sim.play();
  }, [sim, car, cost, params.mu, params.ddp, params.N]);

  const converge = useCallback(() => {
    sim.setState((s) => {
      let cur = s;
      for (let it = 0; it < 40; it++) {
        const J = cur.costs[cur.costs.length - 1];
        let moved = false;
        for (const alpha of [1, 0.5, 0.25, 0.125, 0.0625, 0.03125]) {
          const trial = forwardPass(car, cost, cur.xs, cur.us, cur.bp, alpha);
          if (trial.cost < J) {
            const bp = backwardPass(car, cost, trial.xs, trial.us, params.mu, params.ddp);
            cur = { xs: trial.xs, us: trial.us, bp, k: params.N, costs: [...cur.costs, trial.cost], alphas: [...cur.alphas, alpha], iter: cur.iter + 1 };
            moved = true;
            break;
          }
        }
        if (!moved) break;
        if (cur.costs.length >= 2 && Math.abs(cur.costs[cur.costs.length - 2] - J) < 1e-9 * Math.max(1, J)) break;
      }
      return cur;
    });
    sim.play();
  }, [sim, car, cost, params.mu, params.ddp, params.N]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const { xs, us, bp, k } = sim.state;
      // Lot
      ctx.strokeStyle = p.wall;
      ctx.lineWidth = 2;
      ctx.strokeRect(sx(v, 0), sy(v, 10), sl(v, 16), sl(v, 10));
      ctx.fillStyle = p.obstacle;
      ctx.globalAlpha = 0.6;
      for (const o of LOT.obstacles) {
        ctx.beginPath();
        o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
        ctx.closePath();
        ctx.fill();
      }
      ctx.globalAlpha = 0.35;
      ctx.strokeStyle = p.ink;
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      for (const m of LOT.markings ?? []) {
        ctx.beginPath();
        ctx.moveTo(sx(v, m.x1), sy(v, m.y1));
        ctx.lineTo(sx(v, m.x2), sy(v, m.y2));
        ctx.stroke();
      }
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
      // nominal trajectory
      ctx.strokeStyle = p.path;
      ctx.lineWidth = 2;
      ctx.beginPath();
      xs.forEach((x, i) => (i === 0 ? ctx.moveTo(sx(v, x[0]), sy(v, x[1])) : ctx.lineTo(sx(v, x[0]), sy(v, x[1]))));
      ctx.stroke();
      // value ellipses from k to N (already built), faint
      const level = 0.5 * Math.max(0.2, sim.state.costs[sim.state.costs.length - 1] * 0.02);
      for (let j = Math.max(k, 0); j <= params.N; j += Math.max(1, Math.floor(params.N / 7))) {
        const pts = ellipsePoints(bp.Vxx[j], xs[j][0], xs[j][1], level);
        ctx.strokeStyle = p.graph;
        ctx.globalAlpha = j === k ? 1 : 0.3;
        ctx.lineWidth = j === k ? 2 : 1;
        ctx.beginPath();
        pts.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt[0]), sy(v, pt[1])) : ctx.lineTo(sx(v, pt[0]), sy(v, pt[1]))));
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      // the car at k
      const drawCar = (x: Vec, col: string, alpha: number) => {
        ctx.save();
        ctx.globalAlpha = alpha;
        ctx.translate(sx(v, x[0]), sy(v, x[1]));
        ctx.rotate(-x[2]);
        ctx.strokeStyle = col;
        ctx.lineWidth = 2;
        const L = sl(v, 2.6);
        const W = sl(v, 1.8);
        ctx.strokeRect(-sl(v, 0.6), -W / 2, L + sl(v, 1.4), W);
        ctx.beginPath();
        ctx.moveTo(L + sl(v, 0.8), 0);
        ctx.lineTo(L + sl(v, 0.3), -W * 0.3);
        ctx.moveTo(L + sl(v, 0.8), 0);
        ctx.lineTo(L + sl(v, 0.3), W * 0.3);
        ctx.stroke();
        ctx.restore();
      };
      drawCar(xs[0], p.start, 0.8);
      drawCar(GOAL, p.goal, 0.8);
      drawCar(xs[k], p.robot, 1);
      if (k < params.N) {
        const x = xs[k];
        const kk = bp.k[k];
        const K = bp.K[k];
        // feedforward: δv along heading (scaled), δφ as a tilt
        const th = x[2];
        const u = us[k];
        const vNew = u[0] + kk[0];
        const phiNew = u[1] + kk[1];
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(sx(v, x[0]), sy(v, x[1]));
        ctx.lineTo(sx(v, x[0] + 1.2 * vNew * Math.cos(th + phiNew)), sy(v, x[1] + 1.2 * vNew * Math.sin(th + phiNew)));
        ctx.stroke();
        // feedback field: displaced positions around x̄_k, δθ = 0
        for (let i = -2; i <= 2; i++) {
          for (let j = -2; j <= 2; j++) {
            if (i === 0 && j === 0) continue;
            const dx = [i * 0.5, j * 0.5, 0];
            const du = [K[0][0] * dx[0] + K[0][1] * dx[1], K[1][0] * dx[0] + K[1][1] * dx[1]];
            const px = x[0] + dx[0];
            const py = x[1] + dx[1];
            const v2 = u[0] + kk[0] + du[0];
            const phi2 = u[1] + kk[1] + du[1];
            ctx.strokeStyle = p.graph;
            ctx.globalAlpha = 0.7;
            ctx.lineWidth = 1.2;
            ctx.beginPath();
            ctx.moveTo(sx(v, px), sy(v, py));
            ctx.lineTo(sx(v, px + 0.35 * v2 * Math.cos(th + phi2)), sy(v, py + 0.35 * v2 * Math.sin(th + phi2)));
            ctx.stroke();
            ctx.globalAlpha = 1;
          }
        }
        label(ctx, `k = ${k}:  k_k = (${kk[0].toFixed(2)}, ${kk[1].toFixed(2)})   K_k = [[${K[0].map((x) => x.toFixed(2)).join(', ')}], [${K[1].map((x) => x.toFixed(2)).join(', ')}]]`, sx(v, 0.3), sy(v, 9.6), p.ink, { size: 9 });
        label(ctx, `Q_uu = [[${bp.Quu[k][0].map((x) => x.toFixed(2)).join(', ')}], [${bp.Quu[k][1].map((x) => x.toFixed(2)).join(', ')}]]  + μ = ${params.mu}`, sx(v, 0.3), sy(v, 9.25), p.ink, { size: 9 });
      } else {
        label(ctx, `k = N = ${params.N}: V_N = ℓ_f, V_xx = Q_f — the recursion starts here`, sx(v, 0.3), sy(v, 9.6), p.ink, { size: 9 });
      }
      const J = sim.state.costs;
      label(ctx, `iteration ${sim.state.iter} · cost ${J[J.length - 1].toFixed(3)}${sim.state.alphas.length ? ` · last α = ${sim.state.alphas[sim.state.alphas.length - 1]}` : ''} · expected ΔV(1) = ${(-(bp.expectedDecrease[0] + bp.expectedDecrease[1])).toFixed(3)}${params.ddp ? ' · DDP' : ' · iLQR'}`, sx(v, 0.3), sy(v, 0.2), p.ink, { size: 9 });
      // cost history strip
      if (J.length > 1) {
        const x0 = 10.5;
        const w = 5.2;
        const y0 = 0.6;
        const h = 1.6;
        const mx = Math.max(...J);
        ctx.strokeStyle = p.grid;
        ctx.strokeRect(sx(v, x0), sy(v, y0 + h), sl(v, w), sl(v, h));
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        J.forEach((c, i) => {
          const px = x0 + (i / Math.max(1, J.length - 1)) * w;
          const py = y0 + (Math.log10(Math.max(c, 1e-3)) / Math.log10(Math.max(mx, 1.01))) * h;
          if (i === 0) ctx.moveTo(sx(v, px), sy(v, py));
          else ctx.lineTo(sx(v, px), sy(v, py));
        });
        ctx.stroke();
        label(ctx, 'cost per iteration (log)', sx(v, x0), sy(v, y0 + h + 0.1), p.ink, { size: 8 });
      }
    },
    [sim.state, params],
  );

  return (
    <WidgetFrame
      id="w19.2"
      title="iLQR Backward Pass"
      teaches="iLQR is not gradient descent on the control sequence: it builds a local value function and a feedback policy at every stage, and the forward rollout is closed-loop."
      colorKey={['path', 'graph', 'robot', 'start', 'goal', 'obstacle']}
      caption={
        <>
          Hitch parks from the south-west corner into bay 2. The purple curve is the nominal trajectory; the blue ellipses are the quadratic value
          model ½δxᵀV_xxδx at sampled stages, drawn as the backward pass reaches them (scrub k, or let it run). At the highlighted stage the orange
          arrow is the control after the feedforward k_k and the small blue arrows are what the feedback gain K_k would command from nearby
          displaced positions. Press <em>forward pass</em> to roll the policy out with backtracking on α and start the next backward pass from
          the new nominal; <em>converge</em> runs to a stationary point. DDP adds the second-order dynamics terms by differencing the Jacobians.
        </>
      }
      wide
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, sim.state, params]} aspect={1.7} padding={0.2} ariaLabel="Hitch's nominal parking trajectory in the Lot with the iLQR value ellipses, feedforward arrow and feedback vector field at the scrubbed stage." />
      <ControlPanel columns={3}>
        <Slider label="scrub stage k (N → 0)" role="graph" value={sim.state.k} min={0} max={params.N} step={1} onChange={(k) => { sim.pause(); sim.setState((s) => ({ ...s, k: Math.round(k) })); }} />
        <Slider label="horizon N" value={params.N} min={12} max={60} step={1} onChange={(N) => setParams((p) => ({ ...p, N: Math.round(N) }))} />
        <Slider label="regularization log₁₀ μ" value={Math.log10(params.mu)} min={-6} max={2} step={0.5} format={(x) => `μ = ${(10 ** x).toExponential(0)}`} onChange={(x) => setParams((p) => ({ ...p, mu: 10 ** x }))} />
        <Slider label="goal weight" role="goal" value={params.goalWeight} min={5} max={200} step={5} onChange={(goalWeight) => setParams((p) => ({ ...p, goalWeight }))} />
        <Slider label="steering cost" value={params.steerWeight} min={0.2} max={10} step={0.1} onChange={(steerWeight) => setParams((p) => ({ ...p, steerWeight }))} />
        <Toggle label="DDP (second-order dynamics terms)" checked={params.ddp} onChange={(ddp) => setParams((p) => ({ ...p, ddp }))} />
      </ControlPanel>
      <div className="border-t border-fd-border px-3 py-2">
        <ButtonRow>
          <ActionButton onClick={forward} emphasis>forward pass (one iteration)</ActionButton>
          <ActionButton onClick={converge}>converge</ActionButton>
        </ButtonRow>
      </div>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} />
    </WidgetFrame>
  );
}
