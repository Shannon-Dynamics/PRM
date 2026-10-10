'use client';

import { useCallback, useMemo } from 'react';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Reach2R } from '@/lib/dynamics/reach2r';
import { racerProblem, trapezoidPhase } from '@/lib/trajectory/micro';
import { phaseAt, timeScale } from '@/lib/trajectory/time-scale';

/**
 * The chapter hook: one path, two clocks. The left lane runs the spline on a
 * trapezoidal clock that finishes in the same t_f the time scaler achieves;
 * the right lane runs the time scaler's clock. Same curve, same duration, same
 * motors — and only one of them is feasible, because the time scaler spends its
 * acceleration where the configuration can afford it and the uniform clock
 * spends it where the clock says.
 */

const WORLD = { minX: 0, minY: 0, maxX: 10.4, maxY: 4.6 };
const SLOW = 0.35; // playback factor: the motion is 0.6 s long

export function TwoClocksHook() {
  const problem = useMemo(() => {
    const { pd, lim } = racerProblem(9.81);
    const arm = Reach2R.micro(9.81);
    const res = timeScale(pd, lim, { N: 240 });
    return { pd, lim, arm, sc: res.ok ? res.scaling : null };
  }, []);
  const tf = problem.sc?.tf ?? 1;

  const init = useCallback(() => ({ t: 0 }), []);
  const step = useCallback((s: { t: number }) => ({ t: s.t + SLOW / 30 }), []);
  const sim = useSimulation({ init, step, fps: 30, maxTicks: Math.round(((tf + 0.5) * 30) / SLOW), loop: true });

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const { pd, lim, sc, arm } = problem;
      const t = Math.min(tf, sim.state.t);
      const lanes: { cx: number; title: string; phase: { s: number; sdot: number; sddot: number } }[] = [
        { cx: 2.6, title: `uniform clock — the path in ${tf.toFixed(2)} s at constant ṡ`, phase: trapezoidPhase(tf, t) },
        { cx: 7.8, title: `time-scaled clock — the same ${tf.toFixed(2)} s, bang-bang`, phase: sc ? phaseAt(sc, t) : { s: 0, sdot: 0, sddot: 0 } },
      ];
      for (const lane of lanes) {
        const cy = 2.5;
        const X = (x: number) => sx(v, lane.cx + x);
        const Y = (y: number) => sy(v, cy + y);
        // the planned path (purple)
        ctx.strokeStyle = p.path;
        ctx.globalAlpha = 0.55;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        for (let i = 0; i <= 80; i++) {
          const tip = arm.points(pd.path.q(i / 80)).tip;
          if (i === 0) ctx.moveTo(X(tip[0]), Y(tip[1]));
          else ctx.lineTo(X(tip[0]), Y(tip[1]));
        }
        ctx.stroke();
        ctx.globalAlpha = 1;
        const q = pd.path.q(lane.phase.s);
        const tau = pd.torque(lane.phase.s, lane.phase.sdot, lane.phase.sddot);
        const pts = arm.points(q);
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 5;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(X(0), Y(0));
        ctx.lineTo(X(pts.elbow[0]), Y(pts.elbow[1]));
        ctx.lineTo(X(pts.tip[0]), Y(pts.tip[1]));
        ctx.stroke();
        ctx.fillStyle = p.ink;
        ctx.beginPath();
        ctx.arc(X(0), Y(0), 3, 0, 2 * Math.PI);
        ctx.fill();
        label(ctx, lane.title, X(-2.4), sy(v, 4.4), p.ink, { size: 11, weight: 600 });
        // gauges
        let pegged = false;
        for (let j = 0; j < 2; j++) {
          const gx = lane.cx + 1.55 + j * 0.45;
          const gy0 = 0.35;
          const gh = 1.5;
          const frac = Math.min(1, Math.abs(tau[j]) / lim.max[j]);
          ctx.fillStyle = p.grid;
          ctx.fillRect(sx(v, gx), sy(v, gy0 + gh), sl(v, 0.3), sl(v, gh));
          const over = Math.abs(tau[j]) > lim.max[j] * 1.001;
          pegged = pegged || over;
          ctx.fillStyle = over ? p.goal : p.robot;
          ctx.fillRect(sx(v, gx), sy(v, gy0 + frac * gh), sl(v, 0.3), sl(v, frac * gh));
          label(ctx, `τ${j + 1}`, sx(v, gx + 0.03), sy(v, gy0 - 0.12), p.ink, { size: 9 });
          label(ctx, tau[j].toFixed(1), sx(v, gx - 0.06), sy(v, gy0 + gh + 0.12), over ? p.goal : p.ink, { size: 9 });
        }
        label(ctx, `s = ${lane.phase.s.toFixed(2)}   ṡ = ${lane.phase.sdot.toFixed(2)}`, X(-2.4), sy(v, 0.25), p.ink, { size: 9 });
        if (pegged) label(ctx, 'MOTOR LIMIT EXCEEDED', X(-2.4), sy(v, 0.55), p.goal, { size: 11, weight: 700 });
      }
      label(ctx, `|τ₁| ≤ ${lim.max[0]}, |τ₂| ≤ ${lim.max[1]} N·m · t = ${t.toFixed(2)} s`, sx(v, 4.2), sy(v, 0.25), p.ink, { size: 9 });
    },
    [problem, sim.state, tf],
  );

  return (
    <div className="not-prose overflow-hidden rounded-md border border-fd-border bg-fd-card">
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick]} aspect={2.3} padding={0.1} ariaLabel="The same arm path executed on a uniform clock and on the time-optimal clock, both finishing at the same time; only the time-scaled one keeps every torque inside its limit." />
      <Transport playing={sim.playing} onToggle={sim.toggle} onReset={sim.reset} tick={sim.tick} />
    </div>
  );
}
