'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Vec } from '@/lib/prob/linalg';
import { Reach2R, MICRO_Q } from '@/lib/dynamics/reach2r';
import { kineticEnergy, type Dynamics } from '@/lib/dynamics/model';
import { stepDynamics, type Integrator, type State } from '@/lib/dynamics/integrate';
import { WORKBENCH } from '@/lib/robots/worlds';

/**
 * w17.1 — the Lagrangian Lab.
 *
 * Reach hangs on a vertical Workbench and swings under gravity from a release
 * configuration the reader drags into place. The energy bars K, V, E stay
 * level because the integrator runs the *honest* standard form; flip the
 * "Coriolis off" toggle and the same integrator is handed a model whose
 * velocity-product vector is zero. Energy then drifts within two swings — the
 * terms the reader was told are "a small correction" are what keeps the
 * physics from lying. The torque gauge shows g(q) and C q̇ as separate stacks,
 * so M q̈ and C q̇ can be watched trading places as the coordinates change.
 */

const TICK = 1 / 30;
const ARENA = { minX: 0, minY: 0, maxX: 10.4, maxY: 5.2 };
const ARM_CX = 2.6;
const ARM_CY = 2.6;
const HIST = 240;

interface Params {
  m2: number;
  aG: number;
  damping: number;
  integrator: Integrator;
  h: number;
  coriolisOff: boolean;
}

interface Sim {
  s: State;
  t: number;
  trace: [number, number][];
  energies: { k: number; v: number; e: number }[];
  phase: [number, number][];
  gauge: { g: Vec; c: Vec; mqdd: Vec };
  e0: number;
}

/** The same model with C q̇ zeroed — the lie the widget tells on request. */
function withoutCoriolis(arm: Reach2R): Dynamics {
  return {
    n: 2,
    inertia: (q) => arm.inertia(q),
    velocityProduct: () => [0, 0],
    gravity: (q) => arm.gravity(q),
    dissipation: (q, qd) => arm.dissipation(q, qd),
    potentialEnergy: (q) => arm.potentialEnergy(q),
    christoffel: (q) => arm.christoffel(q),
  };
}

export function LagrangianLab() {
  const [params, setParams] = useState<Params>({ m2: 1, aG: 9.81, damping: 0, integrator: 'rk4', h: 0.001, coriolisOff: false });
  const [release, setRelease] = useState<Vec>([MICRO_Q[0], MICRO_Q[1]]);
  const drag = useRef<'elbow' | 'tip' | null>(null);

  const arm = useMemo(() => {
    const a = Reach2R.uniform(1, 2, 1, params.m2, params.aG);
    a.damping = params.damping > 0 ? [params.damping, params.damping] : null;
    return a;
  }, [params.m2, params.aG, params.damping]);
  const model = useMemo(() => (params.coriolisOff ? withoutCoriolis(arm) : arm), [arm, params.coriolisOff]);

  const seedRef = useRef(42);
  const init = useCallback(
    (seed: number): Sim => {
      let q: Vec = release.slice();
      if (seed !== seedRef.current) {
        // Re-roll draws a random release configuration; the seed is shown in the transport bar.
        const rng = new Rng(seed);
        q = [rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI)];
        seedRef.current = seed;
        queueMicrotask(() => setRelease(q));
      }
      const s: State = { q, qd: [0, 0] };
      const k = kineticEnergy(arm, s.q, s.qd);
      const v = arm.potentialEnergy(s.q);
      return { s, t: 0, trace: [], energies: [{ k, v, e: k + v }], phase: [[q[0], 0]], gauge: { g: arm.gravity(q), c: [0, 0], mqdd: [0, 0] }, e0: k + v };
    },
    [arm, release],
  );

  const step = useCallback(
    (sim: Sim): Sim => {
      const sub = Math.max(1, Math.round(TICK / params.h));
      const h = TICK / sub;
      let s = sim.s;
      for (let i = 0; i < sub; i++) s = stepDynamics(model, s, [0, 0], h, params.integrator);
      const k = kineticEnergy(arm, s.q, s.qd);
      const v = arm.potentialEnergy(s.q);
      const g = arm.gravity(s.q);
      const c = arm.velocityProduct(s.q, s.qd);
      // M q̈ = −(C q̇ + g + b) under zero torque: read it off the honest model.
      const b = arm.dissipation(s.q, s.qd);
      const mqdd = [-(c[0] + g[0] + b[0]), -(c[1] + g[1] + b[1])];
      const tip = arm.points(s.q).tip;
      return {
        s,
        t: sim.t + TICK,
        trace: [...sim.trace, tip].slice(-HIST),
        energies: [...sim.energies, { k, v, e: k + v }].slice(-HIST),
        phase: [...sim.phase, [s.q[0], s.qd[0]] as [number, number]].slice(-HIST * 2),
        gauge: { g, c, mqdd },
        e0: sim.e0,
      };
    },
    [arm, model, params.h, params.integrator],
  );

  const sim = useSimulation<Sim>({ init, step, fps: 30, initialSeed: 42 });

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const x = w[0] - ARM_CX;
      const y = w[1] - ARM_CY;
      if (phase === 'down') {
        if (Math.hypot(x, y) > 2.3) return;
        const pts = arm.points(sim.state.s.q);
        const dElbow = Math.hypot(x - pts.elbow[0], y - pts.elbow[1]);
        const dTip = Math.hypot(x - pts.tip[0], y - pts.tip[1]);
        drag.current = dElbow < dTip ? 'elbow' : 'tip';
        sim.pause();
      } else if (phase === 'move' && drag.current) {
        const q = sim.state.s.q.slice();
        if (drag.current === 'elbow') {
          const th1 = Math.atan2(y, x);
          q[0] = th1;
        } else {
          const pts = arm.points(q);
          q[1] = Math.atan2(y - pts.elbow[1], x - pts.elbow[0]) - q[0];
        }
        const e0 = arm.potentialEnergy(q);
        sim.setState((st) => ({ ...st, s: { q, qd: [0, 0] }, t: 0, trace: [], phase: [[q[0], 0]], energies: [{ k: 0, v: e0, e: e0 }], gauge: { g: arm.gravity(q), c: [0, 0], mqdd: [0, 0] }, e0 }));
        setRelease(q);
      } else if (phase === 'up') {
        if (drag.current) {
          drag.current = null;
          sim.reset();
          sim.play();
        }
      }
    },
    [arm, sim],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const st = sim.state;
      const X = (x: number) => sx(v, ARM_CX + x);
      const Y = (y: number) => sy(v, ARM_CY + y);

      // --- Workbench panel: table edge, obstacles faint, the arm, its trace ---
      ctx.save();
      ctx.strokeStyle = p.wall;
      ctx.lineWidth = 2;
      ctx.strokeRect(X(-2.4), Y(2.4), sl(v, 4.8), sl(v, 4.8));
      ctx.fillStyle = p.obstacle;
      ctx.globalAlpha = 0.18;
      for (const o of WORKBENCH.obstacles) {
        ctx.beginPath();
        o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(X(pt.x), Y(pt.y)) : ctx.lineTo(X(pt.x), Y(pt.y))));
        ctx.closePath();
        ctx.fill();
      }
      ctx.globalAlpha = 1;
      // gravity arrow
      if (params.aG > 0) {
        ctx.strokeStyle = p.ink;
        ctx.globalAlpha = 0.5;
        ctx.beginPath();
        ctx.moveTo(X(2.1), Y(2.1));
        ctx.lineTo(X(2.1), Y(1.5));
        ctx.lineTo(X(2.0), Y(1.65));
        ctx.moveTo(X(2.1), Y(1.5));
        ctx.lineTo(X(2.2), Y(1.65));
        ctx.stroke();
        label(ctx, 'a_g', X(1.75), Y(1.8), p.ink, { size: 10 });
        ctx.globalAlpha = 1;
      }
      // tip trace
      if (st.trace.length > 1) {
        ctx.strokeStyle = p.robot;
        ctx.globalAlpha = 0.45;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        st.trace.forEach((pt, i) => (i === 0 ? ctx.moveTo(X(pt[0]), Y(pt[1])) : ctx.lineTo(X(pt[0]), Y(pt[1]))));
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      // the arm
      const pts = arm.points(st.s.q);
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 6;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(X(0), Y(0));
      ctx.lineTo(X(pts.elbow[0]), Y(pts.elbow[1]));
      ctx.lineTo(X(pts.tip[0]), Y(pts.tip[1]));
      ctx.stroke();
      ctx.fillStyle = p.bg;
      for (const j of [[0, 0], pts.elbow, pts.tip]) {
        ctx.beginPath();
        ctx.arc(X(j[0]), Y(j[1]), 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 2;
        ctx.stroke();
      }
      // centers of mass
      ctx.fillStyle = p.ink;
      for (const c of [pts.com1, pts.com2]) {
        ctx.beginPath();
        ctx.arc(X(c[0]), Y(c[1]), 3, 0, Math.PI * 2);
        ctx.fill();
      }
      label(ctx, `q = (${st.s.q[0].toFixed(2)}, ${st.s.q[1].toFixed(2)})  t = ${st.t.toFixed(1)} s`, X(-2.3), Y(-2.25), p.ink, { size: 10 });
      ctx.restore();

      // --- Right panel ---
      const PX = 5.6;
      const PW = 4.6;
      // Energy bars (top band y ∈ [3.5, 5.0])
      const last = st.energies[st.energies.length - 1] ?? { k: 0, v: 0, e: 0 };
      const scale = Math.max(Math.abs(st.e0) * 1.4, 1, Math.abs(last.e) * 1.2, Math.abs(last.v) * 1.2, last.k * 1.2);
      const bars: [string, number, string][] = [['K', last.k, p.accent], ['V', last.v, p.truth], ['E', last.e, p.ink]];
      const barBase = 3.55;
      const barH = 1.25;
      label(ctx, 'energy  (J)', sx(v, PX), sy(v, 5.05), p.ink, { size: 10, weight: 600 });
      bars.forEach(([name, val, color], i) => {
        const bx = PX + 0.2 + i * 0.9;
        const hgt = (Math.abs(val) / scale) * barH;
        ctx.fillStyle = color;
        ctx.globalAlpha = 0.85;
        if (val >= 0) ctx.fillRect(sx(v, bx), sy(v, barBase + hgt), sl(v, 0.55), sl(v, hgt));
        else ctx.fillRect(sx(v, bx), sy(v, barBase), sl(v, 0.55), sl(v, hgt));
        ctx.globalAlpha = 1;
        label(ctx, `${name} ${val.toFixed(2)}`, sx(v, bx), sy(v, barBase - 0.14), color, { size: 9 });
      });
      // E₀ reference line
      ctx.strokeStyle = p.goal;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      const eRefY = sy(v, barBase + (Math.abs(st.e0) / scale) * barH);
      ctx.moveTo(sx(v, PX + 0.1), eRefY);
      ctx.lineTo(sx(v, PX + 2.9), eRefY);
      ctx.stroke();
      ctx.setLineDash([]);
      const drift = st.e0 !== 0 ? (last.e - st.e0) / Math.abs(st.e0) : 0;
      label(ctx, `ΔE/E₀ = ${(drift * 100).toFixed(2)}%`, sx(v, PX + 3.0), eRefY, Math.abs(drift) > 0.01 ? p.goal : p.ink, { size: 10, weight: 600 });

      // Energy history sparkline (E over time)
      if (st.energies.length > 1) {
        ctx.strokeStyle = p.ink;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        st.energies.forEach((e, i) => {
          const xx = sx(v, PX + 3.0 + (i / HIST) * 1.5);
          const yy = sy(v, barBase + (Math.abs(e.e) / scale) * barH);
          if (i === 0) ctx.moveTo(xx, yy);
          else ctx.lineTo(xx, yy);
        });
        ctx.stroke();
      }

      // Phase portrait (q₁, q̇₁) in the middle band y ∈ [1.4, 3.3]
      const phX0 = PX;
      const phY0 = 1.45;
      const phW = 2.2;
      const phH = 1.75;
      ctx.strokeStyle = p.grid;
      ctx.strokeRect(sx(v, phX0), sy(v, phY0 + phH), sl(v, phW), sl(v, phH));
      label(ctx, 'phase (q₁, q̇₁)', sx(v, phX0), sy(v, phY0 + phH + 0.12), p.ink, { size: 10, weight: 600 });
      const qdMax = Math.max(4, ...st.phase.map((pp) => Math.abs(pp[1]))) * 1.1;
      const mapPh = (q1: number, qd1: number): [number, number] => [
        sx(v, phX0 + ((q1 + Math.PI) / (2 * Math.PI)) * phW),
        sy(v, phY0 + ((qd1 + qdMax) / (2 * qdMax)) * phH),
      ];
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      let pen = false;
      for (let i = 0; i < st.phase.length; i++) {
        const [q1, qd1] = st.phase[i];
        const [xx, yy] = mapPh(((q1 + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI, qd1);
        if (i > 0 && Math.abs(st.phase[i][0] - st.phase[i - 1][0]) > Math.PI) pen = false;
        if (!pen) {
          ctx.moveTo(xx, yy);
          pen = true;
        } else ctx.lineTo(xx, yy);
      }
      ctx.stroke();
      const lastPh = st.phase[st.phase.length - 1];
      if (lastPh) {
        const [xx, yy] = mapPh(((lastPh[0] + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI, lastPh[1]);
        ctx.fillStyle = p.robot;
        ctx.beginPath();
        ctx.arc(xx, yy, 3.5, 0, Math.PI * 2);
        ctx.fill();
      }

      // Torque gauge: g(q), C q̇, M q̈ per joint, in the right half of the middle band
      const gx = PX + 2.5;
      const gw = PW - 2.5;
      label(ctx, 'torques (N·m)', sx(v, gx), sy(v, phY0 + phH + 0.12), p.ink, { size: 10, weight: 600 });
      const gmax = Math.max(5, ...st.gauge.g.map(Math.abs), ...st.gauge.c.map(Math.abs), ...st.gauge.mqdd.map(Math.abs)) * 1.15;
      const mid = phY0 + phH / 2;
      ctx.strokeStyle = p.grid;
      ctx.beginPath();
      ctx.moveTo(sx(v, gx), sy(v, mid));
      ctx.lineTo(sx(v, gx + gw), sy(v, mid));
      ctx.stroke();
      const series: [string, Vec, string][] = [['g', st.gauge.g, p.truth], ['Cq̇', st.gauge.c, p.cobstacle], ['Mq̈', st.gauge.mqdd, p.robot]];
      for (let j = 0; j < 2; j++) {
        series.forEach(([name, vals, color], k) => {
          const bx = gx + 0.1 + j * (gw / 2) + k * 0.3;
          const val = vals[j];
          const hgt = (Math.abs(val) / gmax) * (phH / 2 - 0.1);
          ctx.fillStyle = color;
          ctx.globalAlpha = 0.85;
          ctx.fillRect(sx(v, bx), sy(v, val >= 0 ? mid + hgt : mid), sl(v, 0.22), sl(v, hgt));
          ctx.globalAlpha = 1;
          if (j === 0) label(ctx, name, sx(v, bx), sy(v, phY0 - 0.02), color, { size: 9 });
        });
        label(ctx, `joint ${j + 1}`, sx(v, gx + 0.1 + j * (gw / 2)), sy(v, phY0 + phH - 0.1), p.ink, { size: 9 });
      }

      // Status line
      const status = `${params.integrator}  h = ${(params.h * 1000).toFixed(1)} ms${params.coriolisOff ? '   CORIOLIS OFF — the model is lying' : ''}`;
      label(ctx, status, sx(v, PX), sy(v, 1.1), params.coriolisOff ? p.goal : p.ink, { size: 10, weight: params.coriolisOff ? 700 : 500 });
      label(ctx, 'drag the elbow or the tip to set the release pose', sx(v, PX), sy(v, 0.75), p.ink, { size: 9 });
    },
    [sim.state, arm, params],
  );

  return (
    <WidgetFrame
      id="w17.1"
      title="Lagrangian Lab"
      teaches="The velocity-product terms are not a small correction you can drop — zero them and energy appears from nowhere. And M q̈ is not a force on its own."
      colorKey={['robot', 'cobstacle']}
      wide
      caption={
        <>
          Reach swings freely under gravity from the configuration you drag it to (default
          q = (π/4, π/2), Choset&rsquo;s worked pose). Left: the arm (orange) and its tip trace.
          Right: the kinetic, potential and total energy bars with E₀ as a dashed line; the
          (q₁, q̇₁) phase portrait; and the torque gauge showing gravity g(q) (grey), the
          velocity-product vector C q̇ (amber) and M q̈ (orange) for each joint. E stays level
          under RK4. Switch <strong>Coriolis off</strong>: the same integrator, the same arm,
          and E drifts within two swings. Switch the integrator to explicit Euler to watch the
          numerical version of the same lie.
        </>
      }
    >
      <SimCanvas
        world={ARENA}
        draw={draw}
        deps={[sim.tick, sim.state, params]}
        aspect={2}
        padding={0.15}
        onPointer={onPointer}
        ariaLabel="A two-link arm swinging under gravity beside bars of kinetic, potential and total energy, a phase portrait, and a torque gauge per joint."
      />
      <ControlPanel columns={3}>
        <Slider label="m₂ (kg)" value={params.m2} min={0.2} max={3} step={0.1} onChange={(m2) => setParams((p) => ({ ...p, m2 }))} />
        <Slider label="a_g (m/s²) — 0 lays the bench flat" value={params.aG} min={0} max={15} step={0.01} onChange={(aG) => setParams((p) => ({ ...p, aG }))} />
        <Slider label="viscous damping" value={params.damping} min={0} max={1} step={0.01} onChange={(damping) => setParams((p) => ({ ...p, damping }))} />
        <Slider
          label="integrator step h"
          value={params.h}
          min={0.0005}
          max={0.02}
          step={0.0005}
          format={(x) => `${(x * 1000).toFixed(1)} ms`}
          onChange={(h) => setParams((p) => ({ ...p, h }))}
        />
        <ButtonRow>
          <ActionButton emphasis={params.integrator === 'rk4'} onClick={() => setParams((p) => ({ ...p, integrator: 'rk4' }))}>RK4</ActionButton>
          <ActionButton emphasis={params.integrator === 'semi-implicit-euler'} onClick={() => setParams((p) => ({ ...p, integrator: 'semi-implicit-euler' }))}>semi-implicit</ActionButton>
          <ActionButton emphasis={params.integrator === 'explicit-euler'} onClick={() => setParams((p) => ({ ...p, integrator: 'explicit-euler' }))}>explicit Euler</ActionButton>
        </ButtonRow>
        <Toggle label="Coriolis off (zero C q̇ in the model)" role="cobstacle" checked={params.coriolisOff} onChange={(coriolisOff) => setParams((p) => ({ ...p, coriolisOff }))} />
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => sim.reseed()}
        seed={sim.seed}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}
