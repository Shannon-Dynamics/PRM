'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import {
  LatticeSearchStepper,
  bangBangTime1D,
  latticeToState,
  type LatticeParams,
  type LatticeResult,
  type LatticeState,
} from '@/lib/trajectory/lattice';

/**
 * w18.2 — the Lattice Hopper.
 *
 * Choset's GRID SEARCH (Algorithm 21) on the (q, q̇) lattice of a double
 * integrator, expanded a few nodes per frame. In 1-DOF mode the plane *is* the
 * state space: every reachable state sits on the grid of figure 11.9, the arcs
 * between them are the quadratics the controls trace, and the time-optimal
 * trajectory is the shortest level count. A planner that discretized q alone
 * could not draw this picture — the vertical axis is momentum, and it is where
 * time optimality lives. In 2-DOF mode the view is the (q₁, q₂) room with a disc
 * obstacle: the δ_v(c₀, c₁) margin grows with speed, so a larger c₁ forces the
 * detour of figure 11.11.
 */

const ARENA = { minX: 0, minY: 0, maxX: 10, maxY: 5.6 };
const D = 2; // travel distance along q₁
const MAX_EXPANSIONS = 60_000;

interface Params {
  h: number;
  aMax: number;
  vMax: number;
  c0: number;
  c1: number;
  astar: boolean;
  twoDof: boolean;
  perFrame: number;
}

interface Sim {
  stepper: LatticeSearchStepper;
  result: LatticeResult | null;
  budgetOut: boolean;
}

const WALL = 0.3; // 1-DOF corridor walls at q = −WALL and D + WALL
const DISC = { x: 1, y: 0.25, r: 0.3 };

function clearance1(q: number[]): number {
  return Math.min(q[0] + WALL, D + WALL - q[0]);
}
function clearance2(q: number[]): number {
  return Math.hypot(q[0] - DISC.x, q[1] - DISC.y) - DISC.r;
}

function makeProblem(prm: Params): { lattice: LatticeParams; start: LatticeState; goal: LatticeState } {
  const n = prm.twoDof ? 2 : 1;
  const dq = 0.5 * prm.aMax * prm.h * prm.h;
  const goalP = Math.round(D / dq);
  const lattice: LatticeParams = {
    aMax: prm.aMax,
    h: prm.h,
    vMax: prm.vMax,
    clearance: prm.twoDof ? clearance2 : clearance1,
    c0: prm.c0,
    c1: prm.c1,
    arcSamples: 4,
    // Choset's goal neighbourhood (5/2 a h², 2 a h) would be generous; one lattice cell in q, exact in q̇, keeps the hand trace exact.
    goalPos: dq * 0.5,
    goalVel: 0,
  };
  return {
    lattice,
    start: { p: new Array<number>(n).fill(0), m: new Array<number>(n).fill(0) },
    goal: { p: [goalP, ...new Array<number>(n - 1).fill(0)], m: new Array<number>(n).fill(0) },
  };
}

export function LatticeHopper() {
  const [params, setParams] = useState<Params>({ h: 0.25, aMax: 1, vMax: 2, c0: 0.05, c1: 0, astar: true, twoDof: false, perFrame: 2 });

  const init = useCallback((): Sim => {
    const { lattice, start, goal } = makeProblem(params);
    return { stepper: new LatticeSearchStepper(start, goal, lattice, params.astar ? 'astar' : 'bfs'), result: null, budgetOut: false };
  }, [params]);

  const step = useCallback(
    (s: Sim): Sim => {
      if (s.result || s.budgetOut) return s;
      const per = params.twoDof ? params.perFrame * 12 : params.perFrame;
      s.stepper.step(per);
      const r = s.stepper.result();
      if (s.stepper.status !== 'running') return { ...s, result: r };
      if (r.expansions >= MAX_EXPANSIONS) return { ...s, budgetOut: true, result: r };
      return { ...s };
    },
    [params],
  );
  const sim = useSimulation<Sim>({ init, step, fps: 30, loop: false });

  const bound = useMemo(() => bangBangTime1D(0, 0, D, 0, params.aMax), [params.aMax]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const { stepper, result, budgetOut } = sim.state;
      const prm = stepper.prm;
      const dq = 0.5 * prm.aMax * prm.h * prm.h;
      const dv = prm.aMax * prm.h;
      const search = stepper.search;
      const trace = search.trace();
      const found = trace.status === 'found';
      const pathStates = found ? trace.path : [];
      const pathKeys = new Set(pathStates.map((s) => stepper.graph.key(s)));

      if (!params.twoDof) {
        // ---- (q, q̇) plane ------------------------------------------------------
        const qMin = -WALL - 0.1;
        const qMax = D + WALL + 0.1;
        const vTop = prm.vMax * 1.15;
        const PX = (q: number) => sx(v, 0.8 + ((q - qMin) / (qMax - qMin)) * 8.6);
        const PY = (qd: number) => sy(v, 2.9 + (qd / vTop) * 2.3);
        // grid of reachable states (figure 11.9)
        ctx.strokeStyle = p.grid;
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let m = -Math.ceil(vTop / dv); m <= Math.ceil(vTop / dv); m++) {
          ctx.moveTo(PX(qMin), PY(m * dv));
          ctx.lineTo(PX(qMax), PY(m * dv));
        }
        const pMax = Math.ceil(qMax / dq);
        if (pMax - Math.floor(qMin / dq) < 400) {
          for (let pp = Math.floor(qMin / dq); pp <= pMax; pp++) {
            ctx.moveTo(PX(pp * dq), PY(-vTop));
            ctx.lineTo(PX(pp * dq), PY(vTop));
          }
        }
        ctx.stroke();
        // walls and velocity limit
        ctx.fillStyle = p.obstacle;
        ctx.globalAlpha = 0.25;
        ctx.fillRect(PX(qMin), PY(vTop), PX(-WALL) - PX(qMin), PY(-vTop) - PY(vTop));
        ctx.fillRect(PX(D + WALL), PY(vTop), PX(qMax) - PX(D + WALL), PY(-vTop) - PY(vTop));
        ctx.fillStyle = p.cobstacle;
        ctx.fillRect(PX(qMin), PY(vTop), PX(qMax) - PX(qMin), PY(prm.vMax) - PY(vTop));
        ctx.fillRect(PX(qMin), PY(-prm.vMax), PX(qMax) - PX(qMin), PY(-vTop) - PY(-prm.vMax));
        ctx.globalAlpha = 1;
        // safety margin as a function of speed: the corridor usable at |q̇| is [−WALL + c₀ + c₁|q̇|, …]
        if (prm.c1 || prm.c0) {
          ctx.strokeStyle = p.cobstacle;
          ctx.setLineDash([3, 3]);
          ctx.beginPath();
          for (const sign of [1, -1]) {
            ctx.moveTo(PX(-WALL + (prm.c0 ?? 0)), PY(0));
            ctx.lineTo(PX(-WALL + (prm.c0 ?? 0) + (prm.c1 ?? 0) * prm.vMax), PY(sign * prm.vMax));
            ctx.moveTo(PX(D + WALL - (prm.c0 ?? 0)), PY(0));
            ctx.lineTo(PX(D + WALL - (prm.c0 ?? 0) - (prm.c1 ?? 0) * prm.vMax), PY(sign * prm.vMax));
          }
          ctx.stroke();
          ctx.setLineDash([]);
        }
        // explored arcs: every closed node's parent link as the quadratic it traced
        const drawArc = (from: LatticeState, to: LatticeState, color: string, width: number, alpha: number) => {
          const a = latticeToState(from, prm);
          const c = (to.m[0] - from.m[0]) * prm.aMax;
          ctx.strokeStyle = color;
          ctx.lineWidth = width;
          ctx.globalAlpha = alpha;
          ctx.beginPath();
          for (let i = 0; i <= 8; i++) {
            const tau = (i / 8) * prm.h;
            const q = a.q[0] + a.qd[0] * tau + 0.5 * c * tau * tau;
            const qd = a.qd[0] + c * tau;
            if (i === 0) ctx.moveTo(PX(q), PY(qd));
            else ctx.lineTo(PX(q), PY(qd));
          }
          ctx.stroke();
          ctx.globalAlpha = 1;
        };
        for (const pop of trace.pops) {
          const k = stepper.graph.key(pop.node);
          const par = search.parent.get(k);
          if (par) drawArc(par, pop.node, p.graph, 1, 0.45);
        }
        for (const [k, node] of search.open) {
          const par = search.parent.get(k);
          if (par) drawArc(par, node, p.graph, 1, 0.18);
        }
        for (let i = 1; i < pathStates.length; i++) drawArc(pathStates[i - 1], pathStates[i], p.path, 3, 1);
        // nodes
        for (const pop of trace.pops) {
          const st = latticeToState(pop.node, prm);
          const onPath = pathKeys.has(stepper.graph.key(pop.node));
          ctx.fillStyle = onPath ? p.path : p.graph;
          ctx.beginPath();
          ctx.arc(PX(st.q[0]), PY(st.qd[0]), onPath ? 4 : 2.5, 0, 2 * Math.PI);
          ctx.fill();
        }
        ctx.fillStyle = p.start;
        ctx.beginPath();
        ctx.arc(PX(0), PY(0), 6, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillStyle = p.goal;
        ctx.beginPath();
        ctx.arc(PX(D), PY(0), 6, 0, 2 * Math.PI);
        ctx.fill();
        label(ctx, 'q', PX(qMax) + 6, PY(0) + 4, p.ink, { size: 10 });
        label(ctx, 'q̇', PX(qMin) - 4, PY(vTop) - 6, p.ink, { size: 10 });
        label(ctx, `v_max = ${prm.vMax}`, PX(qMin) + 4, PY(prm.vMax) - 4, p.cobstacle, { size: 8 });
        label(ctx, `grid: Δq = ½a·h² = ${dq.toFixed(4)}, Δq̇ = a·h = ${dv.toFixed(3)}`, PX(qMin), PY(-vTop) + 16, p.ink, { size: 9 });
      } else {
        // ---- (q₁, q₂) room ----------------------------------------------------------
        const xMin = -0.5;
        const xMax = D + 0.5;
        const yMin = -1.2;
        const yMax = 1.2;
        const scale = Math.min(8.6 / (xMax - xMin), 4.6 / (yMax - yMin));
        const PX = (x: number) => sx(v, 0.8 + (x - xMin) * scale);
        const PY = (y: number) => sy(v, 0.6 + (y - yMin) * scale);
        ctx.strokeStyle = p.grid;
        ctx.strokeRect(PX(xMin), PY(yMax), (xMax - xMin) * sl(v, scale), (yMax - yMin) * sl(v, scale));
        ctx.fillStyle = p.obstacle;
        ctx.globalAlpha = 0.6;
        ctx.beginPath();
        ctx.arc(PX(DISC.x), PY(DISC.y), DISC.r * sl(v, scale), 0, 2 * Math.PI);
        ctx.fill();
        ctx.globalAlpha = 0.18;
        ctx.fillStyle = p.cobstacle;
        ctx.beginPath();
        ctx.arc(PX(DISC.x), PY(DISC.y), (DISC.r + (prm.c0 ?? 0) + (prm.c1 ?? 0) * prm.vMax) * sl(v, scale), 0, 2 * Math.PI);
        ctx.fill();
        ctx.globalAlpha = 1;
        const drawArc2 = (from: LatticeState, to: LatticeState, color: string, width: number, alpha: number) => {
          const a = latticeToState(from, prm);
          const c = [(to.m[0] - from.m[0]) * prm.aMax, (to.m[1] - from.m[1]) * prm.aMax];
          ctx.strokeStyle = color;
          ctx.lineWidth = width;
          ctx.globalAlpha = alpha;
          ctx.beginPath();
          for (let i = 0; i <= 6; i++) {
            const tau = (i / 6) * prm.h;
            const x = a.q[0] + a.qd[0] * tau + 0.5 * c[0] * tau * tau;
            const y = a.q[1] + a.qd[1] * tau + 0.5 * c[1] * tau * tau;
            if (i === 0) ctx.moveTo(PX(x), PY(y));
            else ctx.lineTo(PX(x), PY(y));
          }
          ctx.stroke();
          ctx.globalAlpha = 1;
        };
        // Too many arcs to draw individually at fine h: subsample the closed set.
        const pops = trace.pops;
        const stride = Math.max(1, Math.floor(pops.length / 4000));
        for (let i = 0; i < pops.length; i += stride) {
          const k = stepper.graph.key(pops[i].node);
          const par = search.parent.get(k);
          if (par) drawArc2(par, pops[i].node, p.graph, 0.8, 0.3);
        }
        for (let i = 1; i < pathStates.length; i++) drawArc2(pathStates[i - 1], pathStates[i], p.path, 3, 1);
        for (const st of pathStates) {
          const q = latticeToState(st, prm).q;
          ctx.fillStyle = p.path;
          ctx.beginPath();
          ctx.arc(PX(q[0]), PY(q[1]), 3.5, 0, 2 * Math.PI);
          ctx.fill();
        }
        ctx.fillStyle = p.start;
        ctx.beginPath();
        ctx.arc(PX(0), PY(0), 6, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillStyle = p.goal;
        ctx.beginPath();
        ctx.arc(PX(D), PY(0), 6, 0, 2 * Math.PI);
        ctx.fill();
        label(ctx, 'q₁', PX(xMax) + 6, PY(0) + 4, p.ink, { size: 10 });
        label(ctx, 'q₂', PX(0) - 4, PY(yMax) - 6, p.ink, { size: 10 });
        label(ctx, `safety radius c₀ + c₁‖q̇‖ at v_max shown amber`, PX(xMin), PY(yMin) + 16, p.cobstacle, { size: 8 });
      }

      // ---- readouts ---------------------------------------------------------------
      const r = result ?? stepper.result();
      const status = found ? `found in ${r.levels} levels` : budgetOut ? `budget of ${MAX_EXPANSIONS} expansions exhausted` : trace.status === 'failed' ? 'FAILURE — no safe trajectory on this lattice' : 'searching…';
      label(ctx, `${params.astar ? 'A* (bang-bang heuristic)' : 'breadth-first (Alg. 21)'} · h = ${prm.h} · ${status}`, sx(v, 0.3), sy(v, 5.4), p.ink, { size: 10, weight: 600 });
      label(ctx, `expansions ${trace.expansions} · pruned ${r.pruned} · open ${search.open.size}`, sx(v, 0.3), sy(v, 5.15), p.ink, { size: 9 });
      if (found) {
        label(ctx, `lattice t_f = ${r.tf.toFixed(3)} s   vs   continuous bang-bang 2√(d/a) = ${bound.toFixed(3)} s`, sx(v, 0.3), sy(v, 0.22), p.ink, { size: 9 });
        label(ctx, `controls (units of a_max): ${r.controls.map((c) => (c.length === 1 ? (c[0] > 0 ? '+' : c[0] < 0 ? '−' : '0') : `(${c.map((x) => (x > 0 ? '+' : x < 0 ? '−' : '0')).join('')})`)).join(' ')}`.slice(0, 120), sx(v, 0.3), sy(v, 0.05), p.path, { size: 8 });
      }
    },
    [sim.state, params, bound],
  );

  return (
    <WidgetFrame
      id="w18.2"
      title="Lattice Hopper"
      teaches="Time-optimality lives in the state space (q, q̇): a grid in q alone cannot represent momentum, and the lattice is optimal only for its own discretization."
      colorKey={['graph', 'path', 'start', 'goal', 'cobstacle', 'obstacle']}
      caption={
        <>
          Choset&apos;s GRID SEARCH on a double integrator from rest at q = 0 to rest at q = 2. Blue: explored arcs (quadratics, not straight lines);
          purple: the returned piecewise-constant-acceleration trajectory. Coarsen h and the grid thins while the level count drops; the lattice
          time is never below the continuous bang-bang bound 2√(d/a). Switch to 2-DOF for the room with a disc, raise c₁, and watch the
          speed-dependent safety margin buy the slower detour of Choset&apos;s figure 11.11. Breadth-first mode is Algorithm 21 as printed; A* with
          the per-axis bang-bang time as heuristic returns the same level count with far fewer expansions.
        </>
      }
      wide
    >
      <SimCanvas world={ARENA} draw={draw} deps={[sim.tick, params]} aspect={1.8} padding={0.1} ariaLabel="The (q, q̇) lattice of a double integrator being expanded level by level, with the explored arcs in blue and the time-optimal lattice trajectory in purple." />
      <ControlPanel columns={3}>
        <Slider label="timestep h" value={params.h} min={0.125} max={1} step={0.125} unit="s" onChange={(h) => setParams((p) => ({ ...p, h }))} />
        <Slider label="a_max" value={params.aMax} min={0.5} max={2} step={0.25} onChange={(aMax) => setParams((p) => ({ ...p, aMax }))} />
        <Slider label="v_max" role="cobstacle" value={params.vMax} min={0.5} max={3} step={0.25} onChange={(vMax) => setParams((p) => ({ ...p, vMax }))} />
        <Slider label="safety c₀" role="cobstacle" value={params.c0} min={0} max={0.25} step={0.01} unit="m" onChange={(c0) => setParams((p) => ({ ...p, c0 }))} />
        <Slider label="safety c₁ (per unit speed)" role="cobstacle" value={params.c1} min={0} max={1} step={0.05} unit="s" onChange={(c1) => setParams((p) => ({ ...p, c1 }))} />
        <Slider label="expansions per frame" value={params.perFrame} min={1} max={40} step={1} onChange={(perFrame) => setParams((p) => ({ ...p, perFrame }))} />
        <Toggle label="A* with bang-bang heuristic (off: breadth-first)" role="graph" checked={params.astar} onChange={(astar) => setParams((p) => ({ ...p, astar }))} />
        <Toggle label="2-DOF room with a disc obstacle" checked={params.twoDof} onChange={(twoDof) => setParams((p) => ({ ...p, twoDof }))} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} />
    </WidgetFrame>
  );
}
