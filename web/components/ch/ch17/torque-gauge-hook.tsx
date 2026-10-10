'use client';

import { useCallback, useMemo } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Reach2R } from '@/lib/dynamics/reach2r';
import { inverse } from '@/lib/dynamics/model';

/**
 * The chapter hook: a planned path replayed on Reach at 1× and at 2× with a
 * torque gauge. A path is geometry; a trajectory is geometry plus a clock, and
 * the clock is what decides whether the motors can do it. Replayed twice as
 * fast, q̈ quadruples, C q̇ quadruples, and the gauge pegs red — the planner
 * never asked. Chapter 18 is the answer to the question this picture raises.
 */

const WORLD = { minX: 0, minY: 0, maxX: 10.4, maxY: 4.4 };
const LIMIT = [20, 12];
const T_PATH = 1.6; // seconds at 1×

/** A smooth planned path on T²: a quarter sweep of the shoulder while the elbow folds and unfolds. */
function pathQ(s: number): [number, number] {
  const u = 0.5 - 0.5 * Math.cos(Math.PI * s); // C² ease so q̈ is finite at the ends
  return [-Math.PI / 4 + (Math.PI / 2) * u, Math.PI / 2 + 0.9 * Math.sin(Math.PI * u)];
}

export function TorqueGaugeHook() {
  const arm = useMemo(() => Reach2R.micro(), []);
  const init = useCallback(() => ({ t: 0 }), []);
  const step = useCallback((s: { t: number }) => ({ t: s.t + 1 / 30 }), []);
  const sim = useSimulation({ init, step, fps: 30, maxTicks: Math.round(T_PATH * 30) + 20, loop: true });

  const evalAt = useCallback(
    (speed: number, t: number) => {
      const tf = T_PATH / speed;
      const s = Math.min(1, t / tf);
      const ds = 1e-3;
      const q = pathQ(s);
      const qp = pathQ(Math.min(1, s + ds));
      const qm = pathQ(Math.max(0, s - ds));
      const sdot = t < tf ? 1 / tf : 0;
      const dq = [(qp[0] - qm[0]) / (2 * ds), (qp[1] - qm[1]) / (2 * ds)];
      const ddq = [(qp[0] - 2 * q[0] + qm[0]) / (ds * ds), (qp[1] - 2 * q[1] + qm[1]) / (ds * ds)];
      const qd = dq.map((x) => x * sdot);
      const qdd = ddq.map((x) => x * sdot * sdot);
      const tau = inverse(arm, q, qd, qdd);
      return { q, tau };
    },
    [arm],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const t = sim.state.t;
      const lanes: [number, number, string][] = [[1, 2.6, '1× — the planner’s clock, stretched to 1.6 s'], [2, 7.8, '2× — the same path in 0.8 s']];
      for (const [speed, cx, title] of lanes) {
        const { q, tau } = evalAt(speed, t);
        const cy = 2.0;
        // Planned path as a faint purple trace
        ctx.strokeStyle = p.path;
        ctx.globalAlpha = 0.5;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        for (let i = 0; i <= 60; i++) {
          const pt = arm.points(pathQ(i / 60)).tip;
          if (i === 0) ctx.moveTo(sx(v, cx + pt[0]), sy(v, cy + pt[1]));
          else ctx.lineTo(sx(v, cx + pt[0]), sy(v, cy + pt[1]));
        }
        ctx.stroke();
        ctx.globalAlpha = 1;
        const pts = arm.points(q);
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 5;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(sx(v, cx), sy(v, cy));
        ctx.lineTo(sx(v, cx + pts.elbow[0]), sy(v, cy + pts.elbow[1]));
        ctx.lineTo(sx(v, cx + pts.tip[0]), sy(v, cy + pts.tip[1]));
        ctx.stroke();
        label(ctx, title, sx(v, cx - 2.3), sy(v, 4.2), p.ink, { size: 11, weight: 600 });
        // gauges
        for (let j = 0; j < 2; j++) {
          const gx = cx + 1.6 + j * 0.45;
          const gy0 = 0.3;
          const gh = 1.4;
          const frac = Math.min(1, Math.abs(tau[j]) / LIMIT[j]);
          ctx.fillStyle = p.grid;
          ctx.fillRect(sx(v, gx), sy(v, gy0 + gh), sl(v, 0.3), sl(v, gh));
          ctx.fillStyle = frac >= 0.999 ? p.goal : p.robot;
          ctx.fillRect(sx(v, gx), sy(v, gy0 + frac * gh), sl(v, 0.3), sl(v, frac * gh));
          label(ctx, `τ${j + 1}`, sx(v, gx + 0.02), sy(v, gy0 - 0.12), p.ink, { size: 9 });
          label(ctx, `${tau[j].toFixed(1)}`, sx(v, gx - 0.05), sy(v, gy0 + gh + 0.12), frac >= 0.999 ? p.goal : p.ink, { size: 9 });
        }
        if (Math.abs(tau[0]) > LIMIT[0] || Math.abs(tau[1]) > LIMIT[1]) {
          label(ctx, 'MOTOR LIMIT EXCEEDED', sx(v, cx - 2.3), sy(v, 0.25), p.goal, { size: 11, weight: 700 });
        }
      }
      label(ctx, `limits |τ₁| ≤ ${LIMIT[0]}, |τ₂| ≤ ${LIMIT[1]} N·m`, sx(v, 0.3), sy(v, 0.25), p.ink, { size: 9 });
    },
    [sim.state, arm, evalAt],
  );

  return (
    <div className="not-prose overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick]} aspect={2.4} padding={0.1} ariaLabel="The same planned arm path replayed at two speeds, with torque gauges that peg red at the faster speed." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onReset={sim.reset} tick={sim.tick} />
    </div>
  );
}
