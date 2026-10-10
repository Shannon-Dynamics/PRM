'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Vec } from '@/lib/prob/linalg';
import { Reach2R } from '@/lib/dynamics/reach2r';
import { Reach } from '@/lib/robots/reach';
import { WORKBENCH } from '@/lib/robots/worlds';
import { PathDynamics } from '@/lib/trajectory/constrained';
import { symmetricLimits } from '@/lib/trajectory/limits';
import { RACER_WAYPOINTS, microPath } from '@/lib/trajectory/micro';
import { CubicSpline, type Path } from '@/lib/trajectory/path';
import { phaseAt, timeScale, type TimeScaleResult } from '@/lib/trajectory/time-scale';

/**
 * w18.1 — the Phase-Plane Racer.
 *
 * Left: Reach on the Workbench following a natural cubic spline through
 * draggable waypoints. Right: the (s, ṡ) plane for that path — the inadmissible
 * region above ṡ_max(s) hatched amber, the construction curves F and A_i in
 * blue, the time-optimal profile in orange with its switches ticked. Every
 * redraw calls `timeScale`, so dragging a waypoint shows how the whole limit
 * curve, not one joint's speed limit, decides the clock. The grey dashed line
 * is the "constant speed" ghost: a uniform clock is a horizontal line in this
 * plane, and it crosses into amber wherever it is too fast for the coupling.
 */

const N = 240;
const ARENA = { minX: 0, minY: 0, maxX: 10.6, maxY: 5.2 };
const ARM_CX = 2.6;
const ARM_CY = 2.6;
const PX0 = 5.7;
const PX1 = 10.4;
const PY0 = 0.6;
const PY1 = 4.9;
const SLOW = 0.4;

interface Params {
  tau1: number;
  tau2: number;
  gravity: boolean;
  sdot0: number;
  sdotF: number;
  ghost: number;
  showGhost: boolean;
  /** The chapter's micro-example instead of the spline: joint 2 held at π/2, joint 1 sweeping a quarter turn. */
  micro: boolean;
}

interface Sim {
  t: number;
  trace: [number, number][];
}

const KIN = Reach.workbench();

function randomWaypoints(rng: Rng): Vec[] {
  // Hanging configurations: gravity torque stays inside the default limits, so most re-rolls are solvable.
  const m = 3 + (rng.next() < 0.4 ? 1 : 0);
  const out: Vec[] = [];
  for (let k = 0; k < m; k++) out.push([rng.uniform(-2.5, -0.6), rng.uniform(0.5, 2.5) * (rng.next() < 0.5 ? 1 : -1)]);
  return out;
}

export function PhasePlaneRacer() {
  const [params, setParams] = useState<Params>({ tau1: 20, tau2: 12, gravity: true, sdot0: 0, sdotF: 0, ghost: 1.6, showGhost: true, micro: false });
  const [waypoints, setWaypoints] = useState<Vec[]>(() => RACER_WAYPOINTS.map((w) => [...w]));
  const drag = useRef<number | null>(null);
  const seedRef = useRef(42);

  const arm = useMemo(() => Reach2R.micro(params.gravity ? 9.81 : 0), [params.gravity]);
  const path: Path = useMemo(() => (params.micro ? microPath() : CubicSpline.onTorus(waypoints)), [waypoints, params.micro]);
  const pd = useMemo(() => new PathDynamics(arm, path), [arm, path]);
  const lim = useMemo(() => symmetricLimits([params.tau1, params.tau2]), [params.tau1, params.tau2]);
  const result: TimeScaleResult = useMemo(() => timeScale(pd, lim, { N, sdot0: params.sdot0, sdotF: params.sdotF }), [pd, lim, params.sdot0, params.sdotF]);
  const sc = result.ok ? result.scaling : null;
  const tf = sc?.tf ?? 1.5;

  // Rows whose a_i changes sign along the path — the zero-inertia points flagged in the plane.
  const zeroInertia = useMemo(() => {
    const out: { s: number; row: number }[] = [];
    let prev = pd.abc(0).a;
    for (let k = 1; k <= 120; k++) {
      const a = pd.abc(k / 120).a;
      for (let i = 0; i < a.length; i++) if (Math.sign(a[i]) !== Math.sign(prev[i]) && Math.abs(a[i]) + Math.abs(prev[i]) > 1e-9) out.push({ s: (k - 0.5) / 120, row: i });
      prev = a;
    }
    return out;
  }, [pd]);

  const init = useCallback(
    (seed: number): Sim => {
      if (seed !== seedRef.current) {
        seedRef.current = seed;
        const wps = randomWaypoints(new Rng(seed));
        queueMicrotask(() => setWaypoints(wps));
      }
      return { t: 0, trace: [] };
    },
    [],
  );
  const step = useCallback(
    (s: Sim): Sim => {
      const t = s.t + SLOW / 30;
      if (!sc) return { t, trace: [] };
      const ph = phaseAt(sc, Math.min(sc.tf, t));
      const tip = arm.points(pd.path.q(ph.s)).tip;
      return { t, trace: [...s.trace, tip].slice(-200) };
    },
    [sc, arm, pd],
  );
  const sim = useSimulation<Sim>({ init, step, fps: 30, maxTicks: Math.round(((tf + 0.6) * 30) / SLOW), loop: true, initialSeed: 42 });

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const x = w[0] - ARM_CX;
      const y = w[1] - ARM_CY;
      if (params.micro) return;
      if (phase === 'down') {
        let best = -1;
        let bestD = 0.3;
        waypoints.forEach((q, k) => {
          const tip = arm.points(q).tip;
          const d = Math.hypot(tip[0] - x, tip[1] - y);
          if (d < bestD) {
            bestD = d;
            best = k;
          }
        });
        drag.current = best >= 0 ? best : null;
        if (best >= 0) sim.pause();
      } else if (phase === 'move' && drag.current !== null) {
        const k = drag.current;
        const r = Math.hypot(x, y);
        const rc = Math.min(1.97, Math.max(0.08, r));
        const target = { x: (x * rc) / (r || 1), y: (y * rc) / (r || 1) };
        const [a, b] = KIN.ik(target);
        if (!a || !b) return;
        const cur = waypoints[k];
        // Keep the elbow sign of the waypoint being dragged, and unwrap to the chart lift it already has.
        const unwrap = (q: readonly number[]) => q.map((v, i) => cur[i] + Math.atan2(Math.sin(v - cur[i]), Math.cos(v - cur[i])));
        const qa = unwrap(a);
        const qb = unwrap(b);
        const da = Math.hypot(qa[0] - cur[0], qa[1] - cur[1]);
        const db = Math.hypot(qb[0] - cur[0], qb[1] - cur[1]);
        const next = waypoints.map((q) => q.slice());
        next[k] = da <= db ? qa : qb;
        setWaypoints(next);
      } else if (phase === 'up' && drag.current !== null) {
        drag.current = null;
        sim.reset();
        sim.play();
      }
    },
    [waypoints, arm, sim, params.micro],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const X = (x: number) => sx(v, ARM_CX + x);
      const Y = (y: number) => sy(v, ARM_CY + y);
      const t = Math.min(tf, sim.state.t);
      const ph = sc ? phaseAt(sc, t) : { s: 0, sdot: 0, sddot: 0 };

      // ---- Workbench panel -------------------------------------------------
      ctx.save();
      ctx.strokeStyle = p.wall;
      ctx.lineWidth = 2;
      ctx.strokeRect(X(-2.4), Y(2.4), sl(v, 4.8), sl(v, 4.8));
      ctx.fillStyle = p.obstacle;
      ctx.globalAlpha = 0.16;
      for (const o of WORKBENCH.obstacles) {
        ctx.beginPath();
        o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(X(pt.x), Y(pt.y)) : ctx.lineTo(X(pt.x), Y(pt.y))));
        ctx.closePath();
        ctx.fill();
      }
      ctx.globalAlpha = 1;
      if (params.gravity) {
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
      // the path (purple): tip curve, elbow curve faint
      ctx.strokeStyle = p.path;
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i <= 120; i++) {
        const tip = arm.points(path.q(i / 120)).tip;
        if (i === 0) ctx.moveTo(X(tip[0]), Y(tip[1]));
        else ctx.lineTo(X(tip[0]), Y(tip[1]));
      }
      ctx.stroke();
      ctx.globalAlpha = 0.35;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let i = 0; i <= 120; i++) {
        const e = arm.points(path.q(i / 120)).elbow;
        if (i === 0) ctx.moveTo(X(e[0]), Y(e[1]));
        else ctx.lineTo(X(e[0]), Y(e[1]));
      }
      ctx.stroke();
      ctx.globalAlpha = 1;
      // executed trace (orange)
      if (sim.state.trace.length > 1) {
        ctx.strokeStyle = p.robot;
        ctx.globalAlpha = 0.5;
        ctx.lineWidth = 3;
        ctx.beginPath();
        sim.state.trace.forEach((pt, i) => (i === 0 ? ctx.moveTo(X(pt[0]), Y(pt[1])) : ctx.lineTo(X(pt[0]), Y(pt[1]))));
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
      // waypoints (handles) — the micro-example's ends when the preset is on
      const handles = params.micro ? [path.q(0), path.q(1)] : waypoints;
      handles.forEach((q, k) => {
        const tip = arm.points(q).tip;
        const col = k === 0 ? p.start : k === handles.length - 1 ? p.goal : p.path;
        ctx.fillStyle = col;
        ctx.beginPath();
        ctx.arc(X(tip[0]), Y(tip[1]), 6, 0, 2 * Math.PI);
        ctx.fill();
        ctx.strokeStyle = p.bg;
        ctx.lineWidth = 1.5;
        ctx.stroke();
      });
      // the arm
      const q = path.q(ph.s);
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
      // torque readout
      if (sc) {
        const tau = pd.torque(ph.s, ph.sdot, ph.sddot);
        for (let j = 0; j < 2; j++) {
          const gx = -2.25 + j * 0.3;
          const gy0 = -2.25;
          const gh = 1.2;
          const frac = Math.min(1, Math.abs(tau[j]) / lim.max[j]);
          ctx.fillStyle = p.grid;
          ctx.fillRect(X(gx), Y(gy0 + gh), sl(v, 0.2), sl(v, gh));
          ctx.fillStyle = frac > 0.995 ? p.goal : p.robot;
          ctx.fillRect(X(gx), Y(gy0 + frac * gh), sl(v, 0.2), sl(v, frac * gh));
          label(ctx, `τ${j + 1}`, X(gx), Y(gy0 - 0.08), p.ink, { size: 8 });
        }
      }
      label(ctx, params.micro ? 'Workbench · micro-example q(s) = (πs/2, π/2)' : 'Workbench · drag a waypoint', X(-2.35), Y(2.55), p.ink, { size: 10, weight: 600 });
      ctx.restore();

      // ---- Phase plane -------------------------------------------------------
      const limitMax = sc ? Math.max(...sc.limit.map((l) => (Number.isFinite(l.sdotMax) ? l.sdotMax : 0))) : 0;
      const peak = sc ? Math.max(...sc.sdot) : 1;
      const topRaw = Math.max(params.ghost * 1.1, peak * 1.15, Math.min(limitMax, peak * 2.5) * 1.05, 0.5);
      const top = Math.ceil(topRaw * 4) / 4;
      const PX = (s: number) => sx(v, PX0 + s * (PX1 - PX0));
      const PY = (sd: number) => sy(v, PY0 + (Math.min(sd, top) / top) * (PY1 - PY0));
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.strokeRect(PX(0), PY(top), sl(v, PX1 - PX0), sl(v, PY1 - PY0));
      // inadmissible region
      if (sc) {
        ctx.save();
        ctx.beginPath();
        ctx.moveTo(PX(0), PY(top));
        for (let k = 0; k <= sc.N; k++) ctx.lineTo(PX(sc.s[k]), PY(sc.limit[k].sdotMax));
        ctx.lineTo(PX(1), PY(top));
        ctx.closePath();
        ctx.fillStyle = p.cobstacle;
        ctx.globalAlpha = 0.18;
        ctx.fill();
        ctx.clip();
        ctx.globalAlpha = 0.45;
        ctx.strokeStyle = p.cobstacle;
        ctx.lineWidth = 1;
        ctx.beginPath();
        const x0 = PX(0);
        const x1 = PX(1);
        const yTop = PY(top);
        const yBot = PY(0);
        for (let x = x0 - (yBot - yTop); x < x1; x += 9) {
          ctx.moveTo(x, yBot);
          ctx.lineTo(x + (yBot - yTop), yTop);
        }
        ctx.stroke();
        ctx.restore();
        // the limit curve itself
        ctx.strokeStyle = p.cobstacle;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        for (let k = 0; k <= sc.N; k++) {
          const sd = sc.limit[k].sdotMax;
          if (!Number.isFinite(sd) || sd > top) {
            ctx.moveTo(PX(sc.s[k]), PY(top));
            continue;
          }
          ctx.lineTo(PX(sc.s[k]), PY(sd));
        }
        ctx.stroke();
        // construction curves (blue)
        ctx.strokeStyle = p.graph;
        ctx.lineWidth = 1;
        ctx.globalAlpha = 0.8;
        for (const c of sc.curves) {
          ctx.setLineDash(c.kind === 'F' ? [4, 3] : []);
          ctx.beginPath();
          c.x.forEach((x, i) => {
            const s = (c.from + i) / sc.N;
            const sd = Math.sqrt(Math.max(0, x));
            if (i === 0) ctx.moveTo(PX(s), PY(sd));
            else ctx.lineTo(PX(s), PY(sd));
          });
          ctx.stroke();
          if (c.hit) {
            ctx.fillStyle = p.cobstacle;
            ctx.beginPath();
            ctx.arc(PX(c.hit.s), PY(c.hit.sdot), 3, 0, 2 * Math.PI);
            ctx.fill();
          }
        }
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
        // the optimal profile (orange)
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        for (let k = 0; k <= sc.N; k++) {
          if (k === 0) ctx.moveTo(PX(sc.s[k]), PY(sc.sdot[k]));
          else ctx.lineTo(PX(sc.s[k]), PY(sc.sdot[k]));
        }
        ctx.stroke();
        // switches
        for (const sw of sc.switchPoints) {
          ctx.strokeStyle = p.ink;
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.moveTo(PX(sw.s), PY(0));
          ctx.lineTo(PX(sw.s), PY(0) - 8);
          ctx.stroke();
          ctx.fillStyle = sw.onLimit ? p.cobstacle : p.ink;
          ctx.beginPath();
          ctx.arc(PX(sw.s), PY(sw.sdot), 3.5, 0, 2 * Math.PI);
          ctx.fill();
        }
        // moving state
        ctx.fillStyle = p.robot;
        ctx.beginPath();
        ctx.arc(PX(ph.s), PY(ph.sdot), 5, 0, 2 * Math.PI);
        ctx.fill();
      }
      // zero-inertia flags
      for (const z of zeroInertia) {
        ctx.strokeStyle = p.cobstacle;
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 3]);
        ctx.beginPath();
        ctx.moveTo(PX(z.s), PY(0));
        ctx.lineTo(PX(z.s), PY(top));
        ctx.stroke();
        ctx.setLineDash([]);
        label(ctx, `a${z.row + 1}=0`, PX(z.s) + 3, PY(top) + 10, p.cobstacle, { size: 8 });
      }
      // constant-speed ghost
      if (params.showGhost) {
        ctx.strokeStyle = p.ink;
        ctx.globalAlpha = 0.6;
        ctx.setLineDash([6, 4]);
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(PX(0), PY(params.ghost));
        ctx.lineTo(PX(1), PY(params.ghost));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
        if (sc) {
          // where the ghost is inadmissible, overpaint in amber
          ctx.strokeStyle = p.cobstacle;
          ctx.lineWidth = 3;
          ctx.beginPath();
          let pen = false;
          for (let k = 0; k <= sc.N; k++) {
            const bad = params.ghost > sc.limit[k].sdotMax;
            if (bad && !pen) ctx.moveTo(PX(sc.s[k]), PY(params.ghost));
            else if (bad) ctx.lineTo(PX(sc.s[k]), PY(params.ghost));
            pen = bad;
          }
          ctx.stroke();
        }
        label(ctx, `ghost ṡ = ${params.ghost.toFixed(2)} (t = ${(1 / params.ghost).toFixed(2)} s)`, PX(0.02), PY(params.ghost) - 4, p.ink, { size: 8 });
      }
      // axes
      label(ctx, 's', PX(1) - 8, PY(0) + 14, p.ink, { size: 10 });
      label(ctx, 'ṡ', PX(0) - 14, PY(top) + 4, p.ink, { size: 10 });
      label(ctx, '0', PX(0) - 3, PY(0) + 12, p.ink, { size: 8 });
      label(ctx, '1', PX(1) - 3, PY(0) + 12, p.ink, { size: 8 });
      label(ctx, top.toFixed(2), PX(0) - 30, PY(top) + 4, p.ink, { size: 8 });
      label(ctx, 'phase plane (s, ṡ)', PX(0), PY(top) - 6, p.ink, { size: 10, weight: 600 });
      if (sc) {
        label(ctx, `t_f = ${sc.tf.toFixed(3)} s · switches ${sc.switches.map((s) => s.toFixed(2)).join(', ') || 'none'}`, PX(0), PY(0) + 28, p.ink, { size: 9 });
        const modes = sc.mode.some((m) => m === 'slide') ? 'singular arc present' : 'bang-bang throughout';
        label(ctx, `${modes} · t = ${t.toFixed(2)} s`, PX(0), PY(0) + 40, p.ink, { size: 9 });
      } else if (!result.ok) {
        label(ctx, `no solution: ${result.error.kind}`, PX(0), PY(0) + 28, p.goal, { size: 9, weight: 600 });
        const words = result.error.detail.split(' ');
        let line = '';
        let row = 0;
        for (const w of words) {
          if ((line + ' ' + w).length > 58) {
            label(ctx, line, PX(0), PY(0) + 40 + row * 11, p.goal, { size: 8 });
            row++;
            line = w;
          } else line = line ? line + ' ' + w : w;
        }
        label(ctx, line, PX(0), PY(0) + 40 + row * 11, p.goal, { size: 8 });
      }
      ctx.restore();
    },
    [sim.state, sc, result, tf, arm, path, pd, lim, params, waypoints, zeroInertia],
  );

  return (
    <WidgetFrame
      id="w18.1"
      title="Phase-Plane Racer"
      teaches="The speed limit along a path is a coupled function of configuration and speed, not the slowest joint's limit — and a greedy controller cannot pass a narrow waist."
      colorKey={['path', 'robot', 'cobstacle', 'graph', 'start', 'goal']}
      caption={
        <>
          Left: Reach follows a natural cubic spline through the waypoints (drag one). Right: the same motion in the (s, ṡ) plane. Amber is
          inadmissible — above the velocity-limit curve the motion cone is empty. Dashed blue is F, the deceleration curve integrated backward
          from the goal; solid blue are the Aᵢ curves; orange is the time-optimal profile with its switches ticked (amber dots are tangent
          points on the limit curve). The dashed grey ghost is a constant-speed clock; wherever it turns amber that clock is infeasible. Vertical
          dashed amber lines flag zero-inertia points where some aᵢ(s) changes sign.
        </>
      }
      wide
    >
      <SimCanvas world={ARENA} draw={draw} deps={[sim.tick, params, waypoints]} aspect={2.05} padding={0.1} ariaLabel="Reach following a spline on the Workbench beside the phase plane of the path, with the inadmissible region, the construction curves, the optimal profile and its switch points." onPointer={onPointer} />
      <ControlPanel columns={3}>
        <Slider label="τ₁ limit" role="robot" value={params.tau1} min={6} max={40} step={0.5} unit="N·m" onChange={(tau1) => setParams((p) => ({ ...p, tau1 }))} />
        <Slider label="τ₂ limit" role="robot" value={params.tau2} min={3} max={24} step={0.5} unit="N·m" onChange={(tau2) => setParams((p) => ({ ...p, tau2 }))} />
        <Toggle label="gravity (vertical Workbench)" checked={params.gravity} onChange={(gravity) => setParams((p) => ({ ...p, gravity }))} />
        <Slider label="ṡ₀ (start speed)" role="start" value={params.sdot0} min={0} max={2} step={0.05} onChange={(sdot0) => setParams((p) => ({ ...p, sdot0 }))} />
        <Slider label="ṡ_f (final speed)" role="goal" value={params.sdotF} min={0} max={2} step={0.05} onChange={(sdotF) => setParams((p) => ({ ...p, sdotF }))} />
        <Slider label="constant-speed ghost ṡ" value={params.ghost} min={0.2} max={4} step={0.05} onChange={(ghost) => setParams((p) => ({ ...p, ghost }))} />
        <Toggle label="show ghost" checked={params.showGhost} onChange={(showGhost) => setParams((p) => ({ ...p, showGhost }))} />
        <Toggle label="micro-example preset (straight quarter sweep)" role="path" checked={params.micro} onChange={(micro) => setParams((p) => ({ ...p, micro, gravity: micro ? false : p.gravity }))} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} />
    </WidgetFrame>
  );
}
