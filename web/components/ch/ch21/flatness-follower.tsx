'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Hitch, type HitchCfg } from '@/lib/robots/hitch';
import { Rng } from '@/lib/prob/rng';
import { flatJet, liftCar, liftTrailer, type FlatCurve } from '@/lib/nonholonomic';
import { drawArrow, drawHitch } from './draw-lot';

/**
 * w21.2 — Flatness Follower. A cubic Bézier is a cubic polynomial in t, i.e. a
 * `FlatCurve`; drag its four control points and the whole state is read off
 * the curve by `liftCar` (rear axle as flat output) or `liftTrailer` (trailer
 * axle as flat output, car position from the trailer's tangent). Where the
 * curvature the curve asks of the *car* exceeds 1/ρ the curve turns red: the
 * curve is fine, the car cannot follow it.
 */

const WORLD = { minX: -1, minY: -1, maxX: 15, maxY: 9 };
const N = 160;
const D_HITCH = 2.5;
const MAX_STEER = Hitch.car().p.maxSteer;

type P2 = { x: number; y: number };

/** Bernstein → power basis: B(t) = c₀ + c₁t + c₂t² + c₃t³. */
function bezierToFlat(P: P2[]): FlatCurve {
  const co = (k: 'x' | 'y') => [P[0][k], 3 * (P[1][k] - P[0][k]), 3 * (P[0][k] - 2 * P[1][k] + P[2][k]), -P[0][k] + 3 * P[1][k] - 3 * P[2][k] + P[3][k]];
  return { cx: co('x'), cy: co('y'), tf: 1, reverse: false };
}

const DEFAULT: P2[] = [
  { x: 1.5, y: 1.5 },
  { x: 6.5, y: 1.0 },
  { x: 8.0, y: 7.5 },
  { x: 12.5, y: 6.5 },
];

export function FlatnessFollower() {
  const [pts, setPts] = useState<P2[]>(DEFAULT);
  const [L, setL] = useState(2.6);
  const [trailer, setTrailer] = useState(false);
  const [vectors, setVectors] = useState(true);
  const [seed, setSeed] = useState(7);
  const dragIdx = useRef<number | null>(null);
  const curve = useMemo(() => bezierToFlat(pts), [pts]);
  const car = useMemo(() => (trailer ? Hitch.car().withTrailer(D_HITCH) : Hitch.car()), [trailer]);

  /** Lift the whole curve once per edit: states, car-path curvature, feasibility flags. */
  const lifted = useMemo(() => {
    const out: { t: number; cfg: HitchCfg; flat: P2; phi: number; kappa: number; v: number; omega: number; hitch: number; singular: boolean }[] = [];
    for (let i = 0; i <= N; i++) {
      const t = i / N;
      const [, yd] = flatJet(curve, t);
      const singular = Math.hypot(yd[0], yd[1]) < 1e-3;
      if (trailer) {
        const s = liftTrailer(curve, t, D_HITCH, L);
        out.push({ t, cfg: { body: s.car.pose, trailer: s.psi }, flat: s.trailerAxle, phi: s.car.phi, kappa: s.car.kappa, v: s.car.v, omega: s.car.omega, hitch: s.hitch, singular });
      } else {
        const s = liftCar(curve, t, L);
        out.push({ t, cfg: { body: s.pose, trailer: null }, flat: { x: s.pose.x, y: s.pose.y }, phi: s.phi, kappa: s.kappa, v: s.v, omega: s.omega, hitch: 0, singular });
      }
    }
    return out;
  }, [curve, trailer, L]);

  const rhoMin = L / Math.tan(MAX_STEER);
  const init = useCallback(() => ({ i: 0, dir: 1 }), []);
  const step = useCallback((s: { i: number; dir: number }) => {
    let i = s.i + s.dir;
    let dir = s.dir;
    if (i >= N) {
      i = N;
      dir = -1;
    } else if (i <= 0) {
      i = 0;
      dir = 1;
    }
    return { i, dir };
  }, []);
  const sim = useSimulation({ init, step, fps: 30 });
  const cur = lifted[Math.max(0, Math.min(N, sim.state.i))];

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      // Control polygon.
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      pts.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
      ctx.stroke();
      ctx.restore();
      // The flat-output curve, red where the car's curvature exceeds 1/ρ.
      ctx.save();
      ctx.lineWidth = 3;
      for (let i = 1; i <= N; i++) {
        const a = lifted[i - 1];
        const b = lifted[i];
        ctx.strokeStyle = Math.abs(b.kappa) * rhoMin > 1 || b.singular ? p.goal : p.path;
        ctx.beginPath();
        ctx.moveTo(sx(v, a.flat.x), sy(v, a.flat.y));
        ctx.lineTo(sx(v, b.flat.x), sy(v, b.flat.y));
        ctx.stroke();
      }
      ctx.restore();
      // With a trailer, the derived car path in faint orange.
      if (trailer) {
        ctx.save();
        ctx.strokeStyle = p.robot;
        ctx.globalAlpha = 0.45;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        lifted.forEach((s, i) => (i === 0 ? ctx.moveTo(sx(v, s.cfg.body.x), sy(v, s.cfg.body.y)) : ctx.lineTo(sx(v, s.cfg.body.x), sy(v, s.cfg.body.y))));
        ctx.stroke();
        ctx.restore();
      }
      // Control points.
      pts.forEach((q, i) => {
        ctx.save();
        ctx.fillStyle = i === 0 ? p.start : i === 3 ? p.goal : p.path;
        ctx.beginPath();
        ctx.arc(sx(v, q.x), sy(v, q.y), 7, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
      });
      drawHitch(ctx, v, car, cur.cfg, p.robot, { fill: true, lineWidth: 2 });
      if (vectors) {
        const [y, yd, ydd] = flatJet(curve, cur.t);
        drawArrow(ctx, v, y[0], y[1], y[0] + 0.35 * yd[0], y[1] + 0.35 * yd[1], p.graph, { lineWidth: 2 });
        drawArrow(ctx, v, y[0], y[1], y[0] + 0.08 * ydd[0], y[1] + 0.08 * ydd[1], p.accent, { lineWidth: 1.5, dashed: true });
      }
      const over = Math.abs(cur.kappa) * rhoMin > 1;
      label(ctx, trailer ? 'flat output: trailer-axle midpoint' : 'flat output: rear-axle midpoint', 10, 14, p.ink, { size: 11, weight: 600 });
      label(ctx, `t = ${cur.t.toFixed(3)}   θ = ${cur.cfg.body.theta.toFixed(3)}   v = ${cur.v.toFixed(3)}   ω = ${cur.omega.toFixed(3)}`, 10, 30, p.ink, { size: 10 });
      label(ctx, `κ = ${cur.kappa.toFixed(3)} (1/ρ = ${(1 / rhoMin).toFixed(3)})   φ = atan(Lκ) = ${cur.phi.toFixed(3)} rad${over ? '  > φ_max: infeasible' : ''}`, 10, 44, over ? p.goal : p.ink, { size: 10 });
      if (trailer) label(ctx, `ψ = ${(cur.cfg.trailer ?? 0).toFixed(3)}   hitch θ − ψ = ${cur.hitch.toFixed(3)}`, 10, 58, p.ink, { size: 10 });
      if (cur.singular) label(ctx, 'ẏ = 0: heading undefined (singular)', 10, 72, p.goal, { size: 10 });
      if (vectors) label(ctx, 'blue: ẏ (heading)   teal dashed: ÿ (curvature)', 10, v.height - 12, p.ink, { size: 9 });
    },
    [pts, lifted, rhoMin, trailer, car, cur, vectors, curve],
  );

  const onPointer = useCallback(([x, y]: [number, number], phase: 'down' | 'move' | 'up') => {
    if (phase === 'down') {
      let best = -1;
      let bd = 0.9;
      pts.forEach((q, i) => {
        const d = Math.hypot(q.x - x, q.y - y);
        if (d < bd) {
          bd = d;
          best = i;
        }
      });
      dragIdx.current = best >= 0 ? best : null;
    } else if (phase === 'move' && dragIdx.current !== null) {
      const k = dragIdx.current;
      setPts((ps) => ps.map((q, i) => (i === k ? { x: Math.max(WORLD.minX, Math.min(WORLD.maxX, x)), y: Math.max(WORLD.minY, Math.min(WORLD.maxY, y)) } : q)));
    } else if (phase === 'up') dragIdx.current = null;
  }, [pts]);

  return (
    <WidgetFrame
      id="w21.2"
      title="Flatness Follower"
      teaches="Planning a car does not need its equations integrated: pick a curve for the flat output and the heading, speed, turn rate and steering angle are read off its derivatives."
      colorKey={['start', 'goal', 'path', 'graph', 'robot']}
      caption={
        <>
          Drag the four control points of the purple flat-output curve. Hitch is placed by differentiation alone: heading from the
          tangent ẏ, steering from the curvature. Red stretches ask for more than 1/ρ — the curve is still flat, the car just cannot
          steer that hard; tighten a corner to make one appear. Turn the trailer on and the curve becomes the trailer axle&apos;s:
          the car is now one derivative further away (faint orange).
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, pts, lifted, vectors]} aspect={1.6} padding={0.2} onPointer={onPointer} cursor="grab" ariaLabel="A draggable cubic flat-output curve with Hitch placed along it by differentiation, and the derivative vectors." />
      <ControlPanel columns={3}>
        <Slider label="wheelbase L" value={L} min={1.5} max={4} step={0.05} unit="m" onChange={setL} format={(x) => x.toFixed(2)} role="robot" />
        <Toggle label="trailer (flat output moves to its axle)" checked={trailer} onChange={setTrailer} role="robot" />
        <Toggle label="derivative vectors ẏ, ÿ" checked={vectors} onChange={setVectors} role="graph" />
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={() => {
          setPts(DEFAULT);
          sim.reset();
        }}
        onReseed={() => {
          const ns = (seed * 7919 + 13) % 100000;
          setSeed(ns);
          const r = new Rng(ns);
          setPts([DEFAULT[0], { x: r.uniform(3, 9), y: r.uniform(0, 8) }, { x: r.uniform(5, 12), y: r.uniform(0, 8) }, DEFAULT[3]]);
          sim.reset();
        }}
        tick={sim.tick}
        seed={seed}
      />
    </WidgetFrame>
  );
}
