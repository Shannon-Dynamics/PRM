'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { phaseAt } from '@/lib/trajectory/time-scale';
import {
  ActuatedPathDynamics,
  DrivePath,
  TABLE_12_1_LIMITS,
  applyDrive,
  decouplingGridSearch,
  deadArmQuery,
  drivePoseValid,
  ikLink3,
  percussionDistance,
  timeScaleDrive,
  type DriveAction,
  type Link3Pose,
} from '@/lib/nonholonomic';
import { insetBox } from './draw-lot';

/**
 * w21.6 — Decoupling Drive. Choset's Table 12.1 arm on a horizontal table with
 * the third motor off. Link 3 is "driven like a car" with four primitives —
 * translate along itself, or rotate about its center of percussion — by hand
 * (the buttons) or by the switch-minimizing search, and the plan is executed
 * with Chapter 18's time-optimal scaling, segment by segment, stopping at each
 * switch. The bars show |τ₁|/20 and |τ₂|/10: one of them is always full.
 */

const WORLD = { minX: -0.72, minY: -0.35, maxX: 0.72, maxY: 0.72 };
const BASE_Q = deadArmQuery();
const ARM = BASE_Q.arm;
const K = percussionDistance(ARM);
const SLOW = 0.25;

function drawArmAt(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, q: number[], alpha = 1, color?: string) {
  const pts = ARM.reach.fk(q);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.lineCap = 'round';
  for (let i = 0; i < 3; i++) {
    ctx.strokeStyle = color ?? (i === 2 ? p.robot : p.wall);
    ctx.lineWidth = i === 2 ? 7 : 5;
    ctx.beginPath();
    ctx.moveTo(sx(v, pts[i].x), sy(v, pts[i].y));
    ctx.lineTo(sx(v, pts[i + 1].x), sy(v, pts[i + 1].y));
    ctx.stroke();
  }
  ctx.fillStyle = p.ink;
  for (let i = 0; i < 3; i++) {
    ctx.beginPath();
    ctx.arc(sx(v, pts[i].x), sy(v, pts[i].y), i === 2 ? 4 : 3, 0, 2 * Math.PI);
    ctx.fill();
  }
  ctx.restore();
}

interface Plan {
  segments: ReturnType<typeof decouplingGridSearch>['segments'];
  parts: ReturnType<typeof timeScaleDrive>['parts'];
  pds: ActuatedPathDynamics[];
  paths: DrivePath[];
  tf: number;
  switches: number;
  expansions: number;
}

export function DecouplingDrive() {
  const [seed, setSeed] = useState(1);
  const goal: Link3Pose = useMemo(() => {
    if (seed === 1) return BASE_Q.goal;
    const r = new Rng(seed);
    for (let i = 0; i < 200; i++) {
      const g = { x: BASE_Q.start.x + r.uniform(-0.2, 0.2), y: BASE_Q.start.y + r.uniform(-0.2, 0.2), alpha: BASE_Q.start.alpha + r.uniform(-1.2, 1.2) };
      if (drivePoseValid(BASE_Q, g)) return g;
    }
    return BASE_Q.goal;
  }, [seed]);
  const [manual, setManual] = useState<Link3Pose>(BASE_Q.start);
  const [planFrom, setPlanFrom] = useState<Link3Pose>(BASE_Q.start);
  const [mode, setMode] = useState<'manual' | 'replay'>('replay');

  const plan: Plan | null = useMemo(() => {
    const res = decouplingGridSearch({ ...BASE_Q, start: planFrom, goal });
    if (!res.found) return null;
    const ts = timeScaleDrive(ARM, res.segments, 1, TABLE_12_1_LIMITS, 160);
    const paths = res.segments.map((s) => new DrivePath(ARM, s, 1));
    return { segments: res.segments, parts: ts.parts, paths, pds: paths.map((pth) => new ActuatedPathDynamics(ARM, pth)), tf: ts.tf, switches: res.switches, expansions: res.expansions };
  }, [planFrom, goal]);

  const init = useCallback(() => ({ t: 0 }), []);
  const step = useCallback((s: { t: number }) => ({ t: s.t + SLOW / 30 }), []);
  const total = plan ? plan.tf + 0.15 : 1;
  const sim = useSimulation({ init, step, fps: 30, maxTicks: Math.ceil((total / SLOW) * 30) + 20, loop: true });

  /** Where the replay is: segment, phase state, configuration, torques. */
  const now = useMemo(() => {
    if (!plan || mode !== 'replay') return null;
    let t = Math.min(sim.state.t, plan.tf);
    for (let i = 0; i < plan.parts.length; i++) {
      const part = plan.parts[i];
      if (!part.scaling) continue;
      if (t <= part.tf || i === plan.parts.length - 1) {
        const ph = phaseAt(part.scaling, Math.min(t, part.tf));
        const q = plan.paths[i].q(ph.s);
        const tau = plan.pds[i].torque(ph.s, ph.sdot, ph.sddot);
        return { i, ph, q, tau };
      }
      t -= part.tf;
    }
    return null;
  }, [plan, sim.state.t, mode]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 0.1);
      // Reach annulus of joint 3.
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.arc(sx(v, 0), sy(v, 0), sl(v, 0.6), 0, 2 * Math.PI);
      ctx.stroke();
      ctx.restore();
      // Goal pose of link 3 (red ghost).
      const gq = ikLink3(ARM, goal, 1);
      if (gq) drawArmAt(ctx, v, p, gq, 0.35, p.goal);
      const sq = ikLink3(ARM, planFrom, 1);
      if (sq) drawArmAt(ctx, v, p, sq, 0.3, p.start);
      // Planned link-3 trace: joint 3 positions along each segment.
      if (plan && mode === 'replay') {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 2;
        for (const s of plan.segments) {
          ctx.beginPath();
          for (let k = 0; k <= 20; k++) {
            const ps = applyDrive(s.from, s.act, (k / 20) * s.amount, K);
            if (k === 0) ctx.moveTo(sx(v, ps.x), sy(v, ps.y));
            else ctx.lineTo(sx(v, ps.x), sy(v, ps.y));
          }
          ctx.stroke();
        }
        ctx.restore();
      }
      const q = now ? now.q : ikLink3(ARM, manual, 1);
      if (q) {
        drawArmAt(ctx, v, p, q);
        // Center of percussion of link 3.
        const pts = ARM.reach.fk(q);
        const a = q[0] + q[1] + q[2];
        const cx = pts[2].x + K * Math.cos(a);
        const cy = pts[2].y + K * Math.sin(a);
        ctx.save();
        ctx.fillStyle = p.accent;
        ctx.beginPath();
        ctx.arc(sx(v, cx), sy(v, cy), 5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
        label(ctx, 'center of percussion', sx(v, cx) + 8, sy(v, cy), p.accent, { size: 9 });
      }
      label(ctx, 'Choset Table 12.1 arm, joint 3 unactuated', 10, 14, p.ink, { size: 11, weight: 600 });
      if (mode === 'manual') label(ctx, 'manual driving — press "plan & replay" to search from here', 10, 30, p.ink, { size: 10 });
      else if (plan) {
        label(ctx, `plan: ${plan.segments.map((s) => `${s.act} ${s.amount.toFixed(2)}`).join(' | ')}   ${plan.switches} switches, ${plan.expansions} expansions`, 10, 30, p.path, { size: 10 });
        label(ctx, `time-optimal per segment, stop at each switch: t_f = ${plan.parts.map((x) => x.tf.toFixed(3)).join(' + ')} = ${plan.tf.toFixed(3)} s   (replay ×${SLOW})`, 10, 44, p.ink, { size: 10 });
      } else label(ctx, 'no switch-minimizing plan found from this pose', 10, 30, p.goal, { size: 10 });
      // Phase-plane inset and torque bars.
      if (plan && now) {
        const ix = v.width - 230;
        const iy = v.height - 128;
        insetBox(ctx, ix, iy, 218, 116, p);
        label(ctx, 'ṡ(s) per segment (rest to rest)', ix + 6, iy + 9, p.ink, { size: 9 });
        const segW = 150 / plan.parts.length;
        plan.parts.forEach((part, i) => {
          if (!part.scaling) return;
          const sc = part.scaling;
          const smax = Math.max(...sc.sdot, 1e-9);
          ctx.save();
          ctx.strokeStyle = i === now.i ? p.path : p.graph;
          ctx.lineWidth = i === now.i ? 2 : 1;
          ctx.beginPath();
          sc.s.forEach((s, j) => {
            const X = ix + 8 + i * segW + s * (segW - 4);
            const Y = iy + 100 - (sc.sdot[j] / smax) * 76;
            if (j === 0) ctx.moveTo(X, Y);
            else ctx.lineTo(X, Y);
          });
          ctx.stroke();
          if (i === now.i) {
            ctx.fillStyle = p.robot;
            ctx.beginPath();
            ctx.arc(ix + 8 + i * segW + now.ph.s * (segW - 4), iy + 100 - (now.ph.sdot / smax) * 76, 3.5, 0, 2 * Math.PI);
            ctx.fill();
          }
          ctx.restore();
        });
        // Torque bars.
        const lim = [20, 10];
        now.tau.forEach((tq, j) => {
          const frac = Math.min(1, Math.abs(tq) / lim[j]);
          const bx = ix + 166 + j * 24;
          ctx.save();
          ctx.strokeStyle = p.grid;
          ctx.strokeRect(bx, iy + 24, 14, 76);
          ctx.fillStyle = frac > 0.98 ? p.goal : p.robot;
          ctx.fillRect(bx, iy + 100 - 76 * frac, 14, 76 * frac);
          ctx.restore();
          label(ctx, `τ${j + 1}`, bx + 7, iy + 108, p.ink, { size: 9, align: 'center' });
        });
      }
    },
    [goal, planFrom, plan, now, manual, mode],
  );

  const drive = (act: DriveAction) => {
    const amt = act[0] === 'T' ? BASE_Q.transStep : BASE_Q.rotStep;
    const next = applyDrive(manual, act, amt, K);
    if (drivePoseValid(BASE_Q, next)) {
      setManual(next);
      setMode('manual');
      sim.pause();
    }
  };

  return (
    <WidgetFrame
      id="w21.6"
      title="Decoupling Drive"
      teaches="An unactuated joint does not make an arm unplannable: along decoupling vector fields the free link moves like a car with two actions, and any speed profile along them needs zero torque at the dead joint."
      colorKey={['start', 'goal', 'path', 'graph', 'robot']}
      caption={
        <>
          The orange third link has no motor. It can still translate along itself, or rotate about its center of percussion (teal
          dot, 0.205 m from the joint) — the two decoupling fields of Theorem 12.4.7 — and nothing else at arbitrary speed. Drive it
          with the buttons, then press &quot;plan &amp; replay&quot;: the switch-minimizing search returns a few segments, each
          time-scaled rest-to-rest by Chapter 18. In the inset one torque bar is always full: every segment runs at the edge of an
          actuator limit, and the arm stops dead at every switch.
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, plan, manual, goal]} aspect={1.35} padding={0.04} ariaLabel="A planar 3R arm whose third joint is unactuated, driven along translation and rotation-about-percussion-center primitives, with phase-plane and torque insets." />
      <ControlPanel columns={2}>
        <div className="flex flex-col gap-1.5">
          <span className="font-ui text-[0.72rem] font-medium">drive link 3 (one step each)</span>
          <ButtonRow>
            <ActionButton onClick={() => drive('T+')}>translate +</ActionButton>
            <ActionButton onClick={() => drive('T-')}>translate −</ActionButton>
            <ActionButton onClick={() => drive('R+')}>rotate +</ActionButton>
            <ActionButton onClick={() => drive('R-')}>rotate −</ActionButton>
          </ButtonRow>
        </div>
        <div className="flex flex-col gap-1.5">
          <span className="font-ui text-[0.72rem] font-medium">search and execute</span>
          <ButtonRow>
            <ActionButton
              emphasis
              onClick={() => {
                setPlanFrom(manual);
                setMode('replay');
                sim.reset();
                sim.play();
              }}
            >
              plan &amp; replay
            </ActionButton>
            <ActionButton
              onClick={() => {
                setManual(BASE_Q.start);
                setPlanFrom(BASE_Q.start);
                setMode('replay');
                sim.reset();
              }}
            >
              back to start
            </ActionButton>
          </ButtonRow>
        </div>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => {
          setSeed((s) => (s * 7919 + 13) % 100000);
          setMode('replay');
          sim.reset();
        }}
        tick={sim.tick}
        seed={seed}
      />
    </WidgetFrame>
  );
}
