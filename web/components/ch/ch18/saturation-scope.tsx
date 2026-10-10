'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle } from '@/components/sim/controls';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Reach2R } from '@/lib/dynamics/reach2r';
import { PathDynamics } from '@/lib/trajectory/constrained';
import { symmetricLimits } from '@/lib/trajectory/limits';
import { RACER_WAYPOINTS, trapezoidPhase } from '@/lib/trajectory/micro';
import { CubicSpline } from '@/lib/trajectory/path';
import { phaseAt, timeScale } from '@/lib/trajectory/time-scale';

/**
 * w18.3 — the Saturation Scope.
 *
 * Torque-versus-time strips for both joints under the w18.1 default solution,
 * with the limits drawn as rails. At every instant exactly one strip rides its
 * rail: that is what "bang-bang" means for an arm — not every motor at full
 * power, but one of them, in turn, with the other doing whatever the coupling
 * demands. The switch is where the saturated joint changes. Raise the limits
 * and a singular arc appears in which *neither* joint saturates: the profile
 * slides along the velocity-limit curve. The dashed overlay is the uniform
 * clock finishing at the same t_f — it leaves the rails.
 */

const ARENA = { minX: 0, minY: 0, maxX: 10.4, maxY: 5.2 };
const N = 240;

interface Params {
  scale: number;
  ghost: boolean;
}

export function SaturationScope() {
  const [params, setParams] = useState<Params>({ scale: 1, ghost: false });
  const [tFrac, setTFrac] = useState(0.35);

  const problem = useMemo(() => {
    const arm = Reach2R.micro(9.81);
    const pd = new PathDynamics(arm, CubicSpline.onTorus(RACER_WAYPOINTS.map((w) => [...w])));
    const lim = symmetricLimits([20 * params.scale, 12 * params.scale]);
    const res = timeScale(pd, lim, { N });
    const sc = res.ok ? res.scaling : null;
    const M = 300;
    const strips: { t: number; u: number[]; uGhost: number[]; sat: number; slide: boolean }[] = [];
    if (sc) {
      for (let k = 0; k <= M; k++) {
        const t = (k / M) * sc.tf;
        const ph = phaseAt(sc, t);
        const u = pd.torque(ph.s, ph.sdot, ph.sddot);
        const g = trapezoidPhase(sc.tf, t);
        const uGhost = pd.torque(g.s, g.sdot, g.sddot);
        let sat = -1;
        for (let i = 0; i < 2; i++) if (Math.abs(u[i]) >= lim.max[i] * 0.985) sat = i;
        const node = Math.min(sc.N, Math.round(ph.s * sc.N));
        strips.push({ t, u, uGhost, sat, slide: sc.mode[node] === 'slide' });
      }
    }
    return { arm, pd, lim, sc, strips };
  }, [params.scale]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const { arm, pd, lim, sc, strips } = problem;
      if (!sc) {
        label(ctx, 'no time scaling for these limits', sx(v, 0.4), sy(v, 2.6), p.goal, { size: 11 });
        return;
      }
      const t = tFrac * sc.tf;
      const ph = phaseAt(sc, t);
      const tau = pd.torque(ph.s, ph.sdot, ph.sddot);

      // ---- strips -------------------------------------------------------------
      const X0 = 0.6;
      const X1 = 7.0;
      const TX = (tt: number) => sx(v, X0 + (tt / sc.tf) * (X1 - X0));
      for (let j = 0; j < 2; j++) {
        const yc = j === 0 ? 3.9 : 1.6;
        const half = 0.95;
        const umax = lim.max[j];
        const range = umax * 1.35;
        const TY = (u: number) => sy(v, yc + (Math.max(-range, Math.min(range, u)) / range) * half);
        // rails
        ctx.strokeStyle = p.goal;
        ctx.lineWidth = 1.2;
        ctx.setLineDash([5, 3]);
        ctx.beginPath();
        ctx.moveTo(TX(0), TY(umax));
        ctx.lineTo(TX(sc.tf), TY(umax));
        ctx.moveTo(TX(0), TY(-umax));
        ctx.lineTo(TX(sc.tf), TY(-umax));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.strokeStyle = p.grid;
        ctx.beginPath();
        ctx.moveTo(TX(0), TY(0));
        ctx.lineTo(TX(sc.tf), TY(0));
        ctx.stroke();
        // ghost (uniform clock), clipped to the strip
        if (params.ghost) {
          ctx.save();
          ctx.beginPath();
          ctx.rect(TX(0), sy(v, yc + half + 0.05), TX(sc.tf) - TX(0), sl(v, 2 * half + 0.1));
          ctx.clip();
          ctx.strokeStyle = p.ink;
          ctx.globalAlpha = 0.6;
          ctx.setLineDash([4, 3]);
          ctx.lineWidth = 1.2;
          ctx.beginPath();
          strips.forEach((row, i) => (i === 0 ? ctx.moveTo(TX(row.t), TY(row.uGhost[j])) : ctx.lineTo(TX(row.t), TY(row.uGhost[j]))));
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.globalAlpha = 1;
          ctx.restore();
          // flags where the ghost exceeds the rail
          ctx.fillStyle = p.goal;
          strips.forEach((row, i) => {
            if (i % 6 === 0 && Math.abs(row.uGhost[j]) > umax) {
              ctx.beginPath();
              ctx.arc(TX(row.t), TY(Math.sign(row.uGhost[j]) * range), 2, 0, 2 * Math.PI);
              ctx.fill();
            }
          });
        }
        // the torque history: orange where saturated, ink otherwise, amber on a singular arc
        for (let i = 1; i < strips.length; i++) {
          const a = strips[i - 1];
          const b = strips[i];
          const satHere = b.sat === j;
          ctx.strokeStyle = b.slide ? p.cobstacle : satHere ? p.robot : p.ink;
          ctx.lineWidth = satHere ? 3.5 : 1.5;
          ctx.globalAlpha = satHere || b.slide ? 1 : 0.7;
          ctx.beginPath();
          ctx.moveTo(TX(a.t), TY(a.u[j]));
          ctx.lineTo(TX(b.t), TY(b.u[j]));
          ctx.stroke();
        }
        ctx.globalAlpha = 1;
        // the time cursor
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(TX(t), sy(v, yc + half + 0.05));
        ctx.lineTo(TX(t), sy(v, yc - half - 0.05));
        ctx.stroke();
        ctx.fillStyle = Math.abs(tau[j]) >= umax * 0.985 ? p.robot : p.ink;
        ctx.beginPath();
        ctx.arc(TX(t), TY(tau[j]), 4, 0, 2 * Math.PI);
        ctx.fill();
        label(ctx, `τ${j + 1}  (|τ| ≤ ${umax.toFixed(1)} N·m)`, TX(0), sy(v, yc + half + 0.12), p.ink, { size: 10, weight: 600 });
        label(ctx, `+${umax.toFixed(0)}`, TX(0) - 26, TY(umax) + 3, p.goal, { size: 8 });
        label(ctx, `−${umax.toFixed(0)}`, TX(0) - 26, TY(-umax) + 3, p.goal, { size: 8 });
      }
      // switch markers along the bottom
      for (const sw of sc.switchPoints) {
        const k = Math.min(sc.N, Math.round(sw.s * sc.N));
        const ts = sc.t[k];
        ctx.strokeStyle = p.ink;
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 2]);
        ctx.beginPath();
        ctx.moveTo(TX(ts), sy(v, 0.45));
        ctx.lineTo(TX(ts), sy(v, 4.95));
        ctx.stroke();
        ctx.setLineDash([]);
        label(ctx, `s=${sw.s.toFixed(2)}`, TX(ts) - 14, sy(v, 0.3), p.ink, { size: 8 });
      }
      label(ctx, `t = ${t.toFixed(3)} s of t_f = ${sc.tf.toFixed(3)} s · s = ${ph.s.toFixed(3)}, ṡ = ${ph.sdot.toFixed(2)}`, TX(0), sy(v, 0.1), p.ink, { size: 9 });

      // ---- the arm at time t ------------------------------------------------------
      const cx = 8.7;
      const cy = 2.9;
      const X = (x: number) => sx(v, cx + x);
      const Y = (y: number) => sy(v, cy + y);
      ctx.strokeStyle = p.path;
      ctx.lineWidth = 1.5;
      ctx.globalAlpha = 0.6;
      ctx.beginPath();
      for (let i = 0; i <= 80; i++) {
        const tip = arm.points(pd.path.q(i / 80)).tip;
        if (i === 0) ctx.moveTo(X(tip[0]), Y(tip[1]));
        else ctx.lineTo(X(tip[0]), Y(tip[1]));
      }
      ctx.stroke();
      ctx.globalAlpha = 1;
      const pts = arm.points(pd.path.q(ph.s));
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
      const node = Math.min(sc.N, Math.round(ph.s * sc.N));
      const satNow = sc.mode[node] === 'slide' ? 'singular arc — neither joint saturated' : Math.abs(tau[0]) >= lim.max[0] * 0.985 ? 'joint 1 saturated' : Math.abs(tau[1]) >= lim.max[1] * 0.985 ? 'joint 2 saturated' : 'between samples';
      label(ctx, satNow, X(-1.6), sy(v, 5.0), sc.mode[node] === 'slide' ? p.cobstacle : p.robot, { size: 10, weight: 600 });
      label(ctx, `mode: ${sc.mode[node] === 'max' ? 's̈ = U' : sc.mode[node] === 'min' ? 's̈ = L' : 'slide'}`, X(-1.6), sy(v, 4.75), p.ink, { size: 9 });
    },
    [problem, tFrac, params.ghost],
  );

  return (
    <WidgetFrame
      id="w18.3"
      title="Saturation Scope"
      teaches="Time-optimal does not mean every motor at full power: exactly one actuator is saturated at a time, the switch is where the saturated joint changes, and on a singular arc neither is."
      colorKey={['robot', 'goal', 'path', 'cobstacle']}
      caption={
        <>
          Torque histories of both joints under the w18.1 default solution, with the limits as red rails. The thick orange stretches are where
          that joint rides its rail; at every instant one of the two strips is orange. Scrub time to see which joint is saturated as the arm
          moves. Raise the limit scale to 1.5× or more and an amber stretch appears — a singular arc where the profile slides along the
          velocity-limit curve and neither motor is at its limit. The dashed overlay is the uniform clock finishing at the same t_f: red dots
          mark where it would need more torque than the motor has.
        </>
      }
    >
      <SimCanvas world={ARENA} draw={draw} deps={[tFrac, params, problem]} aspect={2} padding={0.1} ariaLabel="Torque-versus-time strips for both joints of Reach under the time-optimal scaling, showing one joint saturated at every instant." />
      <ControlPanel columns={3}>
        <Slider label="time scrub" role="path" value={tFrac} min={0} max={1} step={0.002} format={(f) => `${(f * (problem.sc?.tf ?? 1)).toFixed(3)} s`} onChange={setTFrac} />
        <Slider label="torque limit scale" role="goal" value={params.scale} min={0.8} max={3} step={0.05} unit="×" onChange={(scale) => setParams((p) => ({ ...p, scale }))} />
        <Toggle label="overlay the uniform clock" checked={params.ghost} onChange={(ghost) => setParams((p) => ({ ...p, ghost }))} />
      </ControlPanel>
    </WidgetFrame>
  );
}
