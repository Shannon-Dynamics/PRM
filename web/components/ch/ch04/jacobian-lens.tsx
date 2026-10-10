'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawSegments, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Reach, t2, type T2 } from '@/lib/robots/reach';
import { WORKBENCH } from '@/lib/robots/worlds';
import { CHOSET_JACOBIAN_EXAMPLE, detJacobian, jacobian2, jointTorques, svd2, tipVelocity, velocityEllipse } from '@/lib/cspace/jacobian';
import { arrowPx, arrowWorld, deg, dot, drawReach, fmt } from './shared';

/**
 * w4.3 — Jacobian Lens: joint velocities seen through J(q).
 *
 * Left, Reach at a draggable configuration; at its tip, the ellipse that the
 * unit circle of joint velocities maps to under J(q), with the two columns of
 * J drawn as arrows (what each joint alone does to the tip). Right, the
 * joint-velocity plane with the unit circle, the rotating q̇ and the singular
 * directions. det J = L₁L₂ sin θ₂ is printed live.
 *
 * The misconception killed: "a singularity is where the arm breaks". As θ₂
 * → 0 or π the ellipse collapses to a segment *along the arm*: one tip
 * direction becomes instantaneously unavailable, and a planner that asks for
 * it asks for infinite joint speed. At Choset's q = (π/4, π/2), q̇ = (1, 0)
 * lands on ẋ = (−√2, 0), exactly as in his figure 3.21.
 */

const JOINT = { minX: -1.7, minY: -1.7, maxX: 1.7, maxY: 1.7 };
const BENCH = { minX: -2.4, minY: -2.4, maxX: 2.4, maxY: 2.4 };

export function JacobianLens() {
  const [q, setQ] = useState<T2>(CHOSET_JACOBIAN_EXAMPLE.q as T2);
  const [lengths, setLengths] = useState<[number, number]>([1, 1]);
  const [showTranspose, setShowTranspose] = useState(false);
  const drag = useRef<'elbow' | 'tip' | null>(null);

  const arm = useMemo(() => new Reach({ base: { x: 0, y: 0 }, lengths, linkRadius: 0 }), [lengths]);
  const J = useMemo(() => jacobian2(arm, q), [arm, q]);
  const det = useMemo(() => detJacobian(arm, q), [arm, q]);
  const ell = useMemo(() => velocityEllipse(arm, q), [arm, q]);
  const tip = useMemo(() => arm.tip(q), [arm, q]);

  // q̇ sweeps the unit circle; the Choset preset parks it at (1, 0).
  const sim = useSimulation<{ phase: number }>({ init: () => ({ phase: 0 }), step: (s) => ({ phase: (s.phase + 0.012) % 1 }), fps: 30, initialSeed: 3 });
  const qdot = useMemo(() => [Math.cos(2 * Math.PI * sim.state.phase), Math.sin(2 * Math.PI * sim.state.phase)], [sim.state.phase]);
  const xdot = useMemo(() => tipVelocity(arm, q, qdot), [arm, q, qdot]);
  const svdJ = useMemo(() => svd2(J), [J]);

  const choset = () => {
    setLengths([1, 1]);
    setQ(CHOSET_JACOBIAN_EXAMPLE.q as T2);
    sim.pause();
    sim.setState(() => ({ phase: 0 }));
  };

  // ---- drag the elbow or the tip -------------------------------------------------------
  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const pts = arm.fk(q);
      if (phase === 'up') {
        drag.current = null;
        return;
      }
      if (phase === 'down') {
        const dE = Math.hypot(pts[1].x - w[0], pts[1].y - w[1]);
        const dT = Math.hypot(pts[2].x - w[0], pts[2].y - w[1]);
        if (Math.min(dE, dT) > 0.35) return;
        drag.current = dE < dT ? 'elbow' : 'tip';
        return;
      }
      if (drag.current === 'elbow') setQ(t2(Math.atan2(w[1], w[0]), q[1]));
      else if (drag.current === 'tip') setQ(t2(q[0], Math.atan2(w[1] - pts[1].y, w[0] - pts[1].x) - q[0]));
    },
    [arm, q],
  );

  // ---- workspace pane -------------------------------------------------------------------
  const drawWork = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 0.5);
      drawSegments(ctx, v, WORKBENCH.walls, p.wall, 2);
      drawReach(ctx, v, p, arm, q);
      // The velocity ellipse J·S¹, scaled so σ = 1 reads as 0.5 m on the table.
      const K = 0.5;
      ctx.save();
      ctx.translate(sx(v, tip.x), sy(v, tip.y));
      ctx.rotate(-ell.angle);
      ctx.beginPath();
      ctx.ellipse(0, 0, Math.max(sl(v, K * ell.rx), 0.5), Math.max(sl(v, K * ell.ry), 0.5), 0, 0, Math.PI * 2);
      ctx.fillStyle = p.graph;
      ctx.globalAlpha = 0.14;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = p.graph;
      ctx.lineWidth = 1.8;
      ctx.stroke();
      ctx.restore();
      // Columns of J: what each joint does alone.
      arrowWorld(ctx, v, tip.x, tip.y, K * J[0][0], K * J[1][0], p.ink, 1.5);
      arrowWorld(ctx, v, tip.x, tip.y, K * J[0][1], K * J[1][1], p.ink, 1.5);
      label(ctx, '∂φ/∂θ₁', sx(v, tip.x + K * J[0][0]) + 6, sy(v, tip.y + K * J[1][0]), p.ink, { size: 9 });
      label(ctx, '∂φ/∂θ₂', sx(v, tip.x + K * J[0][1]) + 6, sy(v, tip.y + K * J[1][1]) + 10, p.ink, { size: 9 });
      // The current ẋ = J q̇.
      arrowWorld(ctx, v, tip.x, tip.y, K * xdot.x, K * xdot.y, p.path, 2.5);
      if (showTranspose) {
        // A unit force at the tip (goal red, the "pull"): its torques appear in the right pane.
        arrowWorld(ctx, v, tip.x, tip.y, -K * qdot[0], -K * qdot[1], p.goal, 2);
        label(ctx, 'f (unit force)', sx(v, tip.x - K * qdot[0]) - 6, sy(v, tip.y - K * qdot[1]) - 10, p.goal, { size: 9, align: 'right' });
      }
      const sing = Math.abs(Math.sin(q[1])) < 0.08;
      label(ctx, `det J = L₁L₂ sin θ₂ = ${fmt(det, 4)}   σ₁ = ${fmt(ell.rx, 3)}  σ₂ = ${fmt(ell.ry, 3)}`, 10, 14, sing ? p.goal : p.ink, { size: 11, weight: 600 });
      if (sing) label(ctx, 'near a singularity: the ellipse is a segment along the arm — one direction is unavailable', 10, 30, p.goal, { size: 10 });
      label(ctx, 'drag the elbow or the tip', sx(v, 0), sy(v, -2.2), p.ink, { size: 9, align: 'center' });
    },
    [arm, q, tip, ell, J, xdot, det, showTranspose, qdot],
  );

  // ---- joint-velocity pane ---------------------------------------------------------------
  const drawJoint = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 0.5);
      const o = { x: sx(v, 0), y: sy(v, 0) };
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.beginPath();
      ctx.moveTo(sx(v, -1.6), o.y);
      ctx.lineTo(sx(v, 1.6), o.y);
      ctx.moveTo(o.x, sy(v, -1.6));
      ctx.lineTo(o.x, sy(v, 1.6));
      ctx.stroke();
      // The unit circle of joint velocities.
      ctx.strokeStyle = p.graph;
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.arc(o.x, o.y, sl(v, 1), 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
      // Right singular vectors: the joint directions that map to the ellipse's axes.
      for (let i = 0; i < 2; i++) {
        const vec = svdJ.v[i];
        arrowWorld(ctx, v, 0, 0, vec.x, vec.y, p.graph, 1.5);
        label(ctx, `v${i + 1} → σ${i + 1} = ${fmt(svdJ.sigma[i], 2)}`, sx(v, vec.x) + 6, sy(v, vec.y) - 8, p.graph, { size: 9 });
      }
      if (showTranspose) {
        // Jᵀ maps the unit circle of tip forces to an ellipse of joint torques with the same σ's along v₁, v₂.
        ctx.save();
        ctx.translate(o.x, o.y);
        ctx.rotate(-Math.atan2(svdJ.v[0].y, svdJ.v[0].x));
        ctx.beginPath();
        ctx.ellipse(0, 0, Math.max(sl(v, svdJ.sigma[0]), 0.5), Math.max(sl(v, svdJ.sigma[1]), 0.5), 0, 0, Math.PI * 2);
        ctx.strokeStyle = p.goal;
        ctx.setLineDash([4, 3]);
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.restore();
        const tau = jointTorques(arm, q, { x: -qdot[0], y: -qdot[1] });
        arrowWorld(ctx, v, 0, 0, tau[0], tau[1], p.goal, 2.2);
        label(ctx, `τ = Jᵀf = (${fmt(tau[0], 2)}, ${fmt(tau[1], 2)})`, 10, 30, p.goal, { size: 10, weight: 600 });
      }
      arrowPx(ctx, o.x, o.y, sx(v, qdot[0]), sy(v, qdot[1]), p.path, 2.5);
      dot(ctx, sx(v, qdot[0]), sy(v, qdot[1]), 4, p.path, p.bg);
      label(ctx, `q̇ = (${fmt(qdot[0], 2)}, ${fmt(qdot[1], 2)})  →  ẋ = J q̇ = (${fmt(xdot.x, 3)}, ${fmt(xdot.y, 3)})`, 10, 14, p.ink, { size: 11, weight: 600 });
      label(ctx, 'θ̇₁', sx(v, 1.6) - 4, o.y - 10, p.ink, { size: 10, align: 'right' });
      label(ctx, 'θ̇₂', o.x + 8, sy(v, 1.6) + 4, p.ink, { size: 10 });
    },
    [svdJ, showTranspose, arm, q, qdot, xdot],
  );

  return (
    <WidgetFrame
      id="w4.3"
      title="Jacobian Lens"
      teaches="A singularity is not where the arm breaks: a direction of tip motion becomes unavailable, and asking for it asks for infinite joint speed."
      colorKey={['robot', 'graph', 'path', 'goal']}
      caption={
        <>
          Left: Reach with the unit circle of joint velocities mapped through <em>J(q)</em> to the blue
          ellipse at its tip (semi-axes σ₁, σ₂ are the singular values of <em>J</em>); the grey arrows
          are the columns of <em>J</em>, the tip velocity each joint produces alone; the purple arrow is
          <em> ẋ = J q̇</em> for the <em>q̇</em> circling in the right pane. Slide θ₂ toward 0 or π and
          watch the ellipse flatten into a segment along the arm as det <em>J</em> → 0. The Choset
          preset sets <em>q</em> = (π/4, π/2) and parks <em>q̇</em> at (1, 0): the tip moves at (−√2,
          0), his figure 3.21. The <em>J</em>ᵀ toggle shows the other direction of the same matrix: a unit
          force at the tip (red) becomes the joint torques τ = <em>J</em>ᵀ<em>f</em>, Chapter 7's lift.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border md:grid-cols-[1.25fr_1fr]">
        <div className="bg-fd-card">
          <div className="eyebrow px-2 py-1">workspace · tip velocities ẋ</div>
          <SimCanvas world={BENCH} draw={drawWork} deps={[sim.tick, q, lengths, showTranspose]} aspect={1.25} padding={0.1} onPointer={onPointer} cursor={drag.current ? 'grabbing' : 'grab'} ariaLabel="Reach with the velocity ellipse at its tip and the columns of the Jacobian as arrows" />
        </div>
        <div className="bg-fd-card">
          <div className="eyebrow px-2 py-1">joint-velocity plane · q̇</div>
          <SimCanvas world={JOINT} draw={drawJoint} deps={[sim.tick, q, lengths, showTranspose]} aspect={1} padding={0.1} ariaLabel="The unit circle of joint velocities with the current joint velocity and the singular directions" />
        </div>
      </div>

      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border font-mono text-[0.7rem] tabular-nums sm:grid-cols-4">
        <Readout title="q" value={`(${fmt(deg(q[0]), 1)}°, ${fmt(deg(q[1]), 1)}°)`} sub={`L₁ = ${lengths[0]}, L₂ = ${lengths[1]}`} />
        <Readout title="J(q)" value={`[${fmt(J[0][0])}  ${fmt(J[0][1])}]`} sub={`[${fmt(J[1][0])}  ${fmt(J[1][1])}]`} />
        <Readout title="det J" value={fmt(det, 4)} sub={`= ${lengths[0]}·${lengths[1]}·sin θ₂ = ${fmt(lengths[0] * lengths[1] * Math.sin(q[1]), 4)}`} />
        <Readout title="condition σ₁/σ₂" value={ell.ry > 1e-9 ? fmt(ell.rx / ell.ry, 2) : '∞'} sub={Math.abs(Math.sin(q[1])) < 1e-6 ? 'singular: rank 1' : 'rank 2'} />
      </div>

      <ControlPanel columns={3} title="Headline: θ₂">
        <Slider label="θ₂" role="robot" value={q[1]} min={-Math.PI} max={Math.PI} step={0.005} unit="rad" format={(x) => `${fmt(x, 3)} (${fmt(deg(x), 1)}°)`} onChange={(v) => setQ(t2(q[0], v))} />
        <ButtonRow>
          <ActionButton emphasis onClick={choset}>
            Choset: q = (π/4, π/2), q̇ = (1, 0)
          </ActionButton>
        </ButtonRow>
        <Toggle label="show Jᵀ: tip force → joint torques" role="goal" checked={showTranspose} onChange={setShowTranspose} />
      </ControlPanel>

      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Advanced: θ₁, link lengths</summary>
        <ControlPanel columns={3}>
          <Slider label="θ₁" role="robot" value={q[0]} min={-Math.PI} max={Math.PI} step={0.005} unit="rad" onChange={(v) => setQ(t2(v, q[1]))} />
          <Slider label="L₁" role="robot" value={lengths[0]} min={0.5} max={1.4} step={0.05} unit="m" onChange={(v) => setLengths([v, lengths[1]])} />
          <Slider label="L₂" role="robot" value={lengths[1]} min={0.5} max={1.4} step={0.05} unit="m" onChange={(v) => setLengths([lengths[0], v])} />
        </ControlPanel>
      </details>

      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} />
    </WidgetFrame>
  );
}

function Readout({ title, value, sub }: { title: string; value: string; sub: string }) {
  return (
    <div className="px-3 py-2">
      <div className="eyebrow">{title}</div>
      <div className="mt-0.5 break-words text-fd-foreground">{value}</div>
      <div className="mt-0.5 font-ui text-[0.68rem] text-fd-muted-foreground">{sub}</div>
    </div>
  );
}
