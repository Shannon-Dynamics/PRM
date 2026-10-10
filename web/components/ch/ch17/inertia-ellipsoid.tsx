'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Reach2R } from '@/lib/dynamics/reach2r';
import { symmetricEigen2 } from '@/lib/dynamics/lagrange';
import { PlanarArm } from '@/lib/dynamics/planar-arm';

/**
 * w17.2 — the Inertia Ellipsoid.
 *
 * Joint-velocity space with the unit-kinetic-energy ellipse {q̇ : q̇ᵀ M(q) q̇ = 1}.
 * Inertia is a *metric*, not a number: the ellipse is anisotropic, its axes
 * rotate with q₂, and M₁₁ reads 3 → 2 → 1 as the elbow unfolds. The ellipse
 * does not depend on q₁ at all — there is no q₁ in M — which is the question
 * the text asks the reader to answer. The Jacobian (manipulability) ellipse of
 * Chapter 4 can be overlaid for the same configuration, in the same units.
 */

const WORLD = { minX: -5.4, minY: -2.6, maxX: 5.4, maxY: 2.6 };

export function InertiaEllipsoid() {
  const [q2, setQ2] = useState(Math.PI / 2);
  const [m2, setM2] = useState(1);
  const [r2, setR2] = useState(0.5);
  const [overlayJ, setOverlayJ] = useState(true);
  const [sweeping, setSweeping] = useState(true);

  const arm = useMemo(() => {
    const a = Reach2R.uniform(1, 2, 1, m2, 9.81);
    a.p = { ...a.p, r2 };
    return a;
  }, [m2, r2]);
  const kin = useMemo(() => new PlanarArm([{ length: 1, com: 0.5, mass: 2, inertia: 1 / 6 }, { length: 1, com: r2, mass: m2, inertia: m2 / 12 }], 9.81), [m2, r2]);

  // Autoplay sweeps q₂ slowly through 0 → π → 0; touching the slider takes over.
  const init = useCallback(() => ({ phase: 0 }), []);
  const step = useCallback((s: { phase: number }) => ({ phase: s.phase + 0.012 }), []);
  const sim = useSimulation({ init, step, fps: 30 });
  const q2Live = sweeping && sim.playing ? (Math.PI / 2) * (1 - Math.cos(sim.state.phase)) : q2;

  const q = useMemo(() => [Math.PI / 4, q2Live], [q2Live]);
  const M = useMemo(() => arm.inertia(q), [arm, q]);
  const eig = useMemo(() => symmetricEigen2(M), [M]);
  const J = useMemo(() => kin.pointJacobian(q, 1, 1), [kin, q]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      // Left: the arm at q, so the reader sees which configuration the ellipse belongs to.
      const AX = -3.2;
      const AY = -0.2;
      ctx.strokeStyle = p.grid;
      ctx.beginPath();
      ctx.arc(sx(v, AX), sy(v, AY), sl(v, 2), 0, Math.PI * 2);
      ctx.stroke();
      const pts = arm.points(q);
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 6;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(sx(v, AX), sy(v, AY));
      ctx.lineTo(sx(v, AX + pts.elbow[0]), sy(v, AY + pts.elbow[1]));
      ctx.lineTo(sx(v, AX + pts.tip[0]), sy(v, AY + pts.tip[1]));
      ctx.stroke();
      ctx.lineWidth = 1;
      label(ctx, `q₂ = ${q[1].toFixed(2)} rad`, sx(v, AX - 1.9), sy(v, AY + 2.25), p.ink, { size: 11, weight: 600 });
      // Jacobian (manipulability) ellipse at the tip: image of the unit q̇ ball.
      if (overlayJ) {
        const JJt = [
          [J[0][0] * J[0][0] + J[0][1] * J[0][1], J[0][0] * J[1][0] + J[0][1] * J[1][1]],
          [J[0][0] * J[1][0] + J[0][1] * J[1][1], J[1][0] * J[1][0] + J[1][1] * J[1][1]],
        ];
        const e = symmetricEigen2(JJt);
        const ang = Math.atan2(e.vectors[0][1], e.vectors[0][0]);
        ctx.save();
        ctx.translate(sx(v, AX + pts.tip[0]), sy(v, AY + pts.tip[1]));
        ctx.rotate(-ang);
        ctx.strokeStyle = p.path;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.ellipse(0, 0, Math.max(sl(v, 0.35 * Math.sqrt(Math.max(e.values[0], 0))), 0.5), Math.max(sl(v, 0.35 * Math.sqrt(Math.max(e.values[1], 0))), 0.5), 0, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
        label(ctx, 'J ellipse (tip velocity for ‖q̇‖ = 1, ×0.35)', sx(v, AX - 1.9), sy(v, AY - 2.3), p.path, { size: 9 });
      }

      // Right: joint-velocity space with the unit-energy ellipse.
      const CX = 2.6;
      const CY = -0.1;
      const S = 0.95; // world units per rad/s
      ctx.strokeStyle = p.grid;
      ctx.beginPath();
      ctx.moveTo(sx(v, CX - 2.4), sy(v, CY));
      ctx.lineTo(sx(v, CX + 2.4), sy(v, CY));
      ctx.moveTo(sx(v, CX), sy(v, CY - 2.3));
      ctx.lineTo(sx(v, CX), sy(v, CY + 2.3));
      ctx.stroke();
      for (const t of [-2, -1, 1, 2]) {
        ctx.beginPath();
        ctx.moveTo(sx(v, CX + t * S), sy(v, CY - 0.05));
        ctx.lineTo(sx(v, CX + t * S), sy(v, CY + 0.05));
        ctx.moveTo(sx(v, CX - 0.05), sy(v, CY + t * S));
        ctx.lineTo(sx(v, CX + 0.05), sy(v, CY + t * S));
        ctx.stroke();
      }
      label(ctx, 'q̇₁ (rad/s)', sx(v, CX + 1.9), sy(v, CY - 0.2), p.ink, { size: 10 });
      label(ctx, 'q̇₂', sx(v, CX + 0.1), sy(v, CY + 2.15), p.ink, { size: 10 });
      // The ellipse: semi-axes 1/√λ along the eigenvectors.
      const a1 = 1 / Math.sqrt(eig.values[0]);
      const a2 = 1 / Math.sqrt(eig.values[1]);
      const ang = Math.atan2(eig.vectors[0][1], eig.vectors[0][0]);
      ctx.save();
      ctx.translate(sx(v, CX), sy(v, CY));
      ctx.rotate(-ang);
      ctx.fillStyle = p.robot;
      ctx.globalAlpha = 0.15;
      ctx.beginPath();
      ctx.ellipse(0, 0, sl(v, a1 * S), sl(v, a2 * S), 0, 0, Math.PI * 2);
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 2.5;
      ctx.stroke();
      // principal axes
      ctx.strokeStyle = p.ink;
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(-sl(v, a1 * S), 0);
      ctx.lineTo(sl(v, a1 * S), 0);
      ctx.moveTo(0, -sl(v, a2 * S));
      ctx.lineTo(0, sl(v, a2 * S));
      ctx.stroke();
      ctx.restore();
      // Unit circle for reference: what "inertia is a number" would predict.
      ctx.strokeStyle = p.truth;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.arc(sx(v, CX), sy(v, CY), sl(v, S / Math.sqrt(M[1][1])), 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
      label(ctx, 'q̇ᵀ M(q) q̇ = 1', sx(v, CX - 2.4), sy(v, CY + 2.3), p.robot, { size: 11, weight: 600 });
      label(ctx, `M₁₁ = ${M[0][0].toFixed(3)}   M₁₂ = ${M[0][1].toFixed(3)}   M₂₂ = ${M[1][1].toFixed(3)}`, sx(v, CX - 2.4), sy(v, CY - 2.15), p.ink, { size: 10 });
      label(ctx, `semi-axes 1/√λ: ${a1.toFixed(3)}, ${a2.toFixed(3)}   λ = ${eig.values[0].toFixed(3)}, ${eig.values[1].toFixed(3)}`, sx(v, CX - 2.4), sy(v, CY - 2.4), p.ink, { size: 10 });
    },
    [arm, q, M, eig, J, overlayJ],
  );

  return (
    <WidgetFrame
      id="w17.2"
      title="Inertia Ellipsoid"
      teaches="Inertia is not a number. M(q) is a configuration-dependent metric on joint-velocity space, and it is anisotropic."
      colorKey={['robot', 'path']}
      caption={
        <>
          Right: joint velocities q̇ that carry exactly one joule of kinetic energy, for Reach at
          the configuration on the left. The semi-axes are 1/√λ for the eigenvalues of M(q); the
          dashed circle is what a scalar inertia equal to M₂₂ would give. Sweep q₂ (it autoplays
          0 → π and back) and read M₁₁ going 3 → 2 → 1: the stretched arm is three times harder to
          swing about the shoulder than the folded one, and the long axis rotates. Nothing happens
          when q₁ changes — there is no q₁ in M. The dashed purple ellipse is Chapter 4&rsquo;s
          Jacobian ellipse at the tip for the same q.
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[q, M, eig, overlayJ]} aspect={2.1} padding={0.1} ariaLabel="A two-link arm on the left and, on the right, the ellipse of joint velocities with unit kinetic energy, with its principal axes and semi-axis lengths." />
      <ControlPanel columns={3}>
        <Slider
          label="q₂ (elbow)"
          role="robot"
          value={q2Live}
          min={0}
          max={Math.PI}
          step={0.01}
          unit="rad"
          onChange={(x) => {
            setSweeping(false);
            setQ2(x);
          }}
        />
        <Slider label="m₂ (kg)" value={m2} min={0.2} max={3} step={0.1} onChange={setM2} />
        <Slider label="r₂ (m, COM of link 2)" value={r2} min={0.1} max={1} step={0.05} onChange={setR2} />
        <Toggle label="overlay Jacobian ellipse" role="path" checked={overlayJ} onChange={setOverlayJ} />
        <Toggle label="auto-sweep q₂" checked={sweeping} onChange={setSweeping} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onReset={sim.reset} tick={sim.tick} />
    </WidgetFrame>
  );
}
