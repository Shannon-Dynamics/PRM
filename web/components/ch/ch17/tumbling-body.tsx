'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { quatToMatrix, QUAT_IDENTITY, type Mat3x3 } from '@/lib/manifold/so3';
import {
  brickInertia,
  eulerBodyStep,
  intermediateAxis,
  rotationalKineticEnergy,
  spatialAngularMomentum,
  type RigidState,
  type Vec3,
} from '@/lib/dynamics/rigid';

/**
 * w17.3 — the Tumbling Body.
 *
 * A uniform brick spinning torque-free under Euler's equation in the body
 * frame (Choset 10.43–10.44). Spin it about the major or minor axis with a 1°
 * nudge and the nudge stays a nudge; spin it about the intermediate axis and
 * the brick flips — ω̇ ≠ 0 with τ = 0, exactly as the equation says. The
 * polhode traces ω on the kinetic-energy ellipsoid; ‖𝓘_s ω_s‖ and K are
 * printed and do not move.
 */

const WORLD = { minX: -5.2, minY: -2.6, maxX: 5.2, maxY: 2.6 };
const H = 1 / 240;
const SUB = 8;

interface Sim {
  s: RigidState;
  polhode: Vec3[];
  t: number;
}

type Axis = 'major' | 'intermediate' | 'minor';

export function TumblingBody() {
  const [dims, setDims] = useState<[number, number, number]>([0.3, 0.2, 0.1]);
  const [axis, setAxis] = useState<Axis>('intermediate');
  const I = useMemo(() => brickInertia(1, dims[0], dims[1], dims[2]), [dims]);
  const axisIndex = useMemo(() => {
    const d = [I[0][0], I[1][1], I[2][2]];
    const order = [0, 1, 2].sort((a, b) => d[a] - d[b]);
    return axis === 'minor' ? order[0] : axis === 'major' ? order[2] : order[1];
  }, [I, axis]);

  const init = useCallback((): Sim => {
    const w: Vec3 = [0, 0, 0];
    w[axisIndex] = 6;
    w[(axisIndex + 1) % 3] = 6 * Math.tan(Math.PI / 180); // the 1° perturbation
    return { s: { R: QUAT_IDENTITY, omega: w }, polhode: [], t: 0 };
  }, [axisIndex]);

  const step = useCallback(
    (sim: Sim): Sim => {
      let s = sim.s;
      for (let i = 0; i < SUB; i++) s = eulerBodyStep(s, [0, 0, 0], I, H);
      return { s, polhode: [...sim.polhode, s.omega].slice(-900), t: sim.t + H * SUB };
    },
    [I],
  );

  const sim = useSimulation<Sim>({ init, step, fps: 30 });
  const K0 = useMemo(() => rotationalKineticEnergy(I, init().s.omega), [I, init]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const { s, polhode } = sim.state;
      const R: Mat3x3 = quatToMatrix(s.R);

      // --- Left: the brick, orthographic projection with a fixed tilt so three faces show.
      const CX = -2.6;
      const CY = 0;
      const tilt = (vec: Vec3): [number, number] => {
        // Rotate the spatial frame by a fixed viewing rotation (about x then y) and drop z.
        const a = -0.5;
        const b = 0.45;
        const y1 = vec[1] * Math.cos(a) - vec[2] * Math.sin(a);
        const z1 = vec[1] * Math.sin(a) + vec[2] * Math.cos(a);
        const x2 = vec[0] * Math.cos(b) + z1 * Math.sin(b);
        return [x2, y1];
      };
      const half = dims.map((d) => d * 4) as Vec3; // drawing scale
      const corners: Vec3[] = [];
      for (const sxn of [-1, 1]) for (const syn of [-1, 1]) for (const szn of [-1, 1]) corners.push([sxn * half[0], syn * half[1], szn * half[2]]);
      const proj = corners.map((c) => {
        const w: Vec3 = [
          R[0][0] * c[0] + R[0][1] * c[1] + R[0][2] * c[2],
          R[1][0] * c[0] + R[1][1] * c[1] + R[1][2] * c[2],
          R[2][0] * c[0] + R[2][1] * c[1] + R[2][2] * c[2],
        ];
        return tilt(w);
      });
      const edges: [number, number][] = [
        [0, 1], [0, 2], [0, 4], [1, 3], [1, 5], [2, 3], [2, 6], [3, 7], [4, 5], [4, 6], [5, 7], [6, 7],
      ];
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (const [a, b] of edges) {
        ctx.moveTo(sx(v, CX + proj[a][0]), sy(v, CY + proj[a][1]));
        ctx.lineTo(sx(v, CX + proj[b][0]), sy(v, CY + proj[b][1]));
      }
      ctx.stroke();
      // Body axes, colored by role: minor/intermediate/major.
      const axesWorld = [0, 1, 2].map((k) => {
        const e: Vec3 = [0, 0, 0];
        e[k] = 1.6;
        const w: Vec3 = [R[0][k] * e[k], R[1][k] * e[k], R[2][k] * e[k]];
        return tilt(w);
      });
      const mid = intermediateAxis(I);
      axesWorld.forEach((ax, k) => {
        ctx.strokeStyle = k === mid ? p.goal : p.graph;
        ctx.lineWidth = k === mid ? 2.5 : 1.5;
        ctx.beginPath();
        ctx.moveTo(sx(v, CX), sy(v, CY));
        ctx.lineTo(sx(v, CX + ax[0]), sy(v, CY + ax[1]));
        ctx.stroke();
        label(ctx, ['x', 'y', 'z'][k] + (k === mid ? ' (intermediate)' : ''), sx(v, CX + ax[0] * 1.08), sy(v, CY + ax[1] * 1.08), k === mid ? p.goal : p.graph, { size: 9 });
      });
      // Angular momentum, fixed in space.
      const L = spatialAngularMomentum(s, I);
      const Ln = Math.hypot(...L);
      const Lp = tilt([(L[0] / Ln) * 2.1, (L[1] / Ln) * 2.1, (L[2] / Ln) * 2.1]);
      ctx.strokeStyle = p.ink;
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.moveTo(sx(v, CX), sy(v, CY));
      ctx.lineTo(sx(v, CX + Lp[0]), sy(v, CY + Lp[1]));
      ctx.stroke();
      ctx.setLineDash([]);
      label(ctx, '𝓘ₛωₛ (fixed)', sx(v, CX + Lp[0] * 1.05), sy(v, CY + Lp[1] * 1.05), p.ink, { size: 9 });
      label(ctx, `t = ${sim.state.t.toFixed(1)} s`, sx(v, CX - 2.3), sy(v, 2.3), p.ink, { size: 10 });

      // --- Right: the polhode in body ω-space, projected onto two principal planes.
      const PX = 2.6;
      const wmax = Math.max(8, ...polhode.map((w) => Math.max(Math.abs(w[0]), Math.abs(w[1]), Math.abs(w[2])))) * 1.1;
      const S = 2.2 / wmax;
      ctx.strokeStyle = p.grid;
      ctx.beginPath();
      ctx.moveTo(sx(v, PX - 2.3), sy(v, 0));
      ctx.lineTo(sx(v, PX + 2.3), sy(v, 0));
      ctx.moveTo(sx(v, PX), sy(v, -2.3));
      ctx.lineTo(sx(v, PX), sy(v, 2.3));
      ctx.stroke();
      // The energy ellipsoid's section in this plane: Σ I_kk ω_k² = 2K, drawn in the (ω_a, ω_b) plane spanned by the spin axis and the perturbed axis.
      const a = axisIndex;
      const b = (axisIndex + 1) % 3;
      const c = (axisIndex + 2) % 3;
      ctx.strokeStyle = p.truth;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.ellipse(sx(v, PX), sy(v, 0), sl(v, Math.sqrt((2 * K0) / I[a][a]) * S), sl(v, Math.sqrt((2 * K0) / I[b][b]) * S), 0, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      polhode.forEach((w, i) => {
        const xx = sx(v, PX + w[a] * S);
        const yy = sy(v, w[b] * S);
        if (i === 0) ctx.moveTo(xx, yy);
        else ctx.lineTo(xx, yy);
      });
      ctx.stroke();
      // second projection, lighter: (ω_a, ω_c)
      ctx.strokeStyle = p.path;
      ctx.globalAlpha = 0.6;
      ctx.beginPath();
      polhode.forEach((w, i) => {
        const xx = sx(v, PX + w[a] * S);
        const yy = sy(v, w[c] * S);
        if (i === 0) ctx.moveTo(xx, yy);
        else ctx.lineTo(xx, yy);
      });
      ctx.stroke();
      ctx.globalAlpha = 1;
      const names = ['ωx', 'ωy', 'ωz'];
      label(ctx, `${names[a]} →`, sx(v, PX + 1.7), sy(v, -0.2), p.ink, { size: 10 });
      label(ctx, `${names[b]} ↑ (orange)   ${names[c]} ↑ (purple)`, sx(v, PX - 2.3), sy(v, 2.4), p.ink, { size: 10 });
      const K = rotationalKineticEnergy(I, s.omega);
      label(ctx, `‖𝓘ₛωₛ‖ = ${Ln.toFixed(5)}   K = ${K.toFixed(5)} J   ω = (${s.omega.map((w) => w.toFixed(2)).join(', ')})`, sx(v, PX - 2.3), sy(v, -2.45), p.ink, { size: 10 });
      ctx.fillStyle = p.robot;
      ctx.beginPath();
      ctx.arc(sx(v, PX + s.omega[a] * S), sy(v, s.omega[b] * S), 3.5, 0, Math.PI * 2);
      ctx.fill();
    },
    [sim.state, I, dims, axisIndex, K0],
  );

  return (
    <WidgetFrame
      id="w17.3"
      title="Tumbling Body"
      teaches="No torque does not mean constant angular velocity. Angular momentum and energy are conserved; ω is not — and about the intermediate axis the brick flips."
      colorKey={['robot', 'graph', 'goal', 'path']}
      caption={
        <>
          A uniform brick (a × b × c = {dims.map((d) => d.toFixed(2)).join(' × ')} m, 1 kg)
          spinning with no external torque, integrated by RK4 on Euler&rsquo;s body-frame
          equation. The dashed line is the spatial angular momentum, which never moves. Right:
          the body angular velocity ω tracing its polhode (orange, in the plane of the spin axis
          and the perturbed axis) against the kinetic-energy ellipse (dashed). Spin about the
          major or minor axis and the 1° perturbation circles the axis forever. Spin about the
          intermediate axis (red) and the polhode runs along the separatrix: the brick flips over
          and back, with ‖𝓘ₛωₛ‖ and K unchanged to five decimals.
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, sim.state]} aspect={2} padding={0.1} ariaLabel="A tumbling brick drawn in projection next to the curve its angular velocity traces in the body frame." />
      <ControlPanel columns={3}>
        <ButtonRow>
          <ActionButton emphasis={axis === 'major'} onClick={() => { setAxis('major'); sim.reset(); }}>spin major axis</ActionButton>
          <ActionButton emphasis={axis === 'intermediate'} onClick={() => { setAxis('intermediate'); sim.reset(); }}>intermediate</ActionButton>
          <ActionButton emphasis={axis === 'minor'} onClick={() => { setAxis('minor'); sim.reset(); }}>minor</ActionButton>
        </ButtonRow>
        <Slider label="a (m)" value={dims[0]} min={0.05} max={0.5} step={0.01} onChange={(a) => setDims(([, b, c]) => [a, b, c])} />
        <Slider label="b (m)" value={dims[1]} min={0.05} max={0.5} step={0.01} onChange={(b) => setDims(([a, , c]) => [a, b, c])} />
        <Slider label="c (m)" value={dims[2]} min={0.05} max={0.5} step={0.01} onChange={(c) => setDims(([a, b]) => [a, b, c])} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
