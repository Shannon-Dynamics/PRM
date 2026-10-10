'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { RrtConnect, StraightLine, paintDot, paintPolyWorld, rectMapper, tint, type Mapper } from '@/lib/sampling';
import {
  TimeRrt,
  compositePathToTrajectories,
  coordinationDiagram,
  corridor,
  corridorWorld,
  executeSchedule,
  monotoneSchedule,
  movingObstacle,
  planAlone,
  sliceRaster,
  trajectoryAt,
  trajectoryDuration,
  twoDiscLab,
  DEFAULT_TIME_RRT,
  type Corridor,
  type CoordinationDiagram,
  type MultiBody,
  type Schedule,
  type Trajectory,
} from '@/lib/multi';

/**
 * w14.1 — Two-Robot Composite C-space.
 *
 * Two discs of radius 0.5 in a corridor with one alcove; they must swap ends.
 * Three panels: the workspace; robot 2's 2-D slice of the 4-D composite
 * C-space, with the amber disc of radius r₁ + r₂ that follows robot 1 (a
 * C-obstacle that moves); and the coordination diagram of the two paths
 * planned alone, with its monotone schedule if one exists. The headline is the
 * corridor width in radii: below 4 the discs cannot pass, and only the coupled
 * planner — RRT-Connect on (ℝ²)² — finds the alcove detour.
 */

type Mode = 'decoupled' | 'prio12' | 'prio21' | 'coupled';
type Phase = 'planning' | 'executing' | 'done' | 'failed';

const R = 0.5;
const LENGTH = 6;
const ALCOVE = { x: 3, width: 1.6, depth: 1.2 };
const ARENA = { minX: 0, minY: 0, maxX: 3, maxY: 1.95 };
const WORK = { x0: 0, y0: 1.05, x1: 3, y1: 1.95 };
const SLICE = { x0: 0, y0: 0, x1: 1.9, y1: 0.95 };
const DIAG = { x0: 2.05, y0: 0, x1: 3, y1: 0.95 };
const ITERS_PER_TICK = 40;
const COUPLED_BUDGET = 8000;
const PRIO_BUDGET = 3000;
const DT = 0.05;
const VMAX = 1.0;

interface State {
  c: Corridor;
  free: MultiBody<number[]>;
  starts: number[][];
  goals: number[][];
  alone: [number[][] | null, number[][] | null];
  diagram: CoordinationDiagram | null;
  schedule: Schedule | null;
  mode: Mode;
  phase: Phase;
  coupled: RrtConnect<number[][]> | null;
  prio: { order: number[]; k: number; rrt: TimeRrt | null; trs: (Trajectory | null)[] } | null;
  trajectories: Trajectory[] | null;
  t: number;
  hold: number;
  /** Positions the panels draw (executed positions, or dragged ones). */
  pos: number[][];
  iterations: number;
  rng: Rng;
}

export function CompositeCSpace() {
  const [widthRadii, setWidthRadii] = useState(3.2);
  const [mode, setMode] = useState<Mode>('coupled');
  const [drag, setDrag] = useState<number | null>(null);

  const init = useCallback(
    (seed: number): State => {
      const h = widthRadii * R;
      const c = corridor(LENGTH, h, ALCOVE);
      const lab = twoDiscLab(c, R);
      const starts = [[0.8, h / 2], [LENGTH - 0.8, h / 2]];
      const goals = [[LENGTH - 0.8, h / 2], [0.8, h / 2]];
      const a = planAlone(c, R, starts[0], goals[0]);
      const b = planAlone(c, R, starts[1], goals[1]);
      const diagram = a && b ? coordinationDiagram(a, b, R, R, 48) : null;
      const schedule = diagram ? monotoneSchedule(diagram) : null;
      const rng = new Rng(seed);
      const s: State = { c, free: lab.free, starts, goals, alone: [a, b], diagram, schedule, mode, phase: 'planning', coupled: null, prio: null, trajectories: null, t: 0, hold: 0, pos: starts.map((q) => q.slice()), iterations: 0, rng };
      if (mode === 'coupled') {
        s.coupled = new RrtConnect<number[][]>(lab.space, lab.free, starts, goals, { steer: new StraightLine<number[][]>({ kind: 'subdivision', step: 0.1 }), eta: 0.6, greed: ['extend', 'connect'] }, rng.fork());
      } else if (mode === 'decoupled') {
        if (a && b && schedule) {
          const frames = executeSchedule(a, b, schedule, 240);
          s.trajectories = [
            { dt: DT, q: frames.map((f) => f.a) },
            { dt: DT, q: frames.map((f) => f.b) },
          ];
          s.phase = 'executing';
        } else s.phase = 'failed';
      } else {
        const order = mode === 'prio12' ? [0, 1] : [1, 0];
        s.prio = { order, k: 0, rrt: null, trs: [null, null] };
      }
      return s;
    },
    [widthRadii, mode],
  );

  const stepFn = useCallback((s: State): State => {
    if (s.phase === 'planning') {
      if (s.coupled) {
        for (let i = 0; i < ITERS_PER_TICK && !s.coupled.merged && s.coupled.attempts < COUPLED_BUDGET; i++) s.coupled.step();
        s.iterations = s.coupled.attempts;
        if (s.coupled.merged) {
          s.trajectories = compositePathToTrajectories(s.coupled.path()!, VMAX, DT);
          s.phase = 'executing';
        } else if (s.coupled.attempts >= COUPLED_BUDGET) s.phase = 'failed';
      } else if (s.prio) {
        const pr = s.prio;
        if (!pr.rrt) {
          const i = pr.order[pr.k];
          const moving = pr.trs.filter((tr): tr is Trajectory => tr !== null).map((tr) => movingObstacle(tr, R));
          pr.rrt = new TimeRrt(s.free.bodies[i], R, s.c.bounds, moving, s.starts[i], s.goals[i], DEFAULT_TIME_RRT, s.rng.fork());
        }
        for (let i = 0; i < ITERS_PER_TICK && !pr.rrt.solved && pr.rrt.iterations < PRIO_BUDGET; i++) pr.rrt.extendOnce();
        s.iterations += ITERS_PER_TICK;
        if (pr.rrt.solved) {
          pr.trs[pr.order[pr.k]] = pr.rrt.trajectory(DT);
          pr.k++;
          pr.rrt = null;
          if (pr.k === 2) {
            s.trajectories = pr.trs as Trajectory[];
            s.phase = 'executing';
          }
        } else if (pr.rrt.iterations >= PRIO_BUDGET) s.phase = 'failed';
      }
    } else if (s.phase === 'executing' && s.trajectories) {
      s.t += DT * 2;
      s.pos = s.trajectories.map((tr) => trajectoryAt(tr, s.t));
      if (s.t >= Math.max(...s.trajectories.map(trajectoryDuration))) s.phase = 'done';
    } else if (s.phase === 'done' || s.phase === 'failed') {
      s.hold++;
    }
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 14 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [widthRadii, mode]);

  const s = sim.state;

  const vRef = useRef<Viewport | null>(null);

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const v = vRef.current;
      if (!v) return;
      // The pointer arrives in arena coordinates; invert the workspace panel's affine mapper through pixels.
      const m = rectMapper(v, WORK, s.c.bounds);
      const b = s.c.bounds;
      const pxl = sx(v, w[0]);
      const pyl = sy(v, w[1]);
      const x = b.minX + ((pxl - m.X(b.minX)) / (m.X(b.maxX) - m.X(b.minX))) * (b.maxX - b.minX);
      const y = b.minY + ((pyl - m.Y(b.minY)) / (m.Y(b.maxY) - m.Y(b.minY))) * (b.maxY - b.minY);
      if (phase === 'down') {
        let best: number | null = null;
        let bd = Infinity;
        s.pos.forEach((q, i) => {
          const d = Math.hypot(q[0] - x, q[1] - y);
          if (d < R * 1.5 && d < bd) {
            bd = d;
            best = i;
          }
        });
        if (best !== null) {
          setDrag(best);
          sim.pause();
        }
        return;
      }
      if (phase === 'move' && drag !== null) {
        const i = drag;
        sim.setState((st) => {
          const pos = st.pos.map((q) => q.slice());
          pos[i] = [Math.max(b.minX, Math.min(b.maxX, x)), Math.max(b.minY, Math.min(b.maxY, y))];
          return { ...st, pos };
        });
      }
      if (phase === 'up') setDrag(null);
    },
    [s, drag, sim],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      vRef.current = v;
      clear(ctx, v, p);
      const world = corridorWorld(s.c);
      // --- workspace ---
      const m = rectMapper(v, WORK, s.c.bounds);
      paintPolyWorld(ctx, m, world, p);
      const drawPoly = (mm: Mapper, path: readonly number[][], color: string, dashed: boolean, width = 2, alpha = 0.7) => {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.globalAlpha = alpha;
        if (dashed) ctx.setLineDash([5, 4]);
        ctx.beginPath();
        path.forEach((q, i) => (i === 0 ? ctx.moveTo(mm.X(q[0]), mm.Y(q[1])) : ctx.lineTo(mm.X(q[0]), mm.Y(q[1]))));
        ctx.stroke();
        ctx.restore();
      };
      if (s.trajectories) {
        drawPoly(m, s.trajectories[0].q, p.path, false, 2.5, 0.8);
        drawPoly(m, s.trajectories[1].q, p.path, true, 2.5, 0.8);
      } else {
        if (s.alone[0]) drawPoly(m, s.alone[0], p.graph, false, 1.5, 0.5);
        if (s.alone[1]) drawPoly(m, s.alone[1], p.graph, true, 1.5, 0.5);
      }
      if (s.coupled && !s.coupled.merged) {
        // Project the composite tree onto each robot's workspace: thin blue hairs.
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.globalAlpha = 0.35;
        ctx.lineWidth = 0.8;
        ctx.beginPath();
        for (const t of s.coupled.trees) {
          for (let i = 1; i < t.tree.size; i++) {
            const q = t.tree.nodes[i].q;
            const pq = t.tree.nodes[t.tree.nodes[i].parent].q;
            for (let r = 0; r < 2; r++) {
              ctx.moveTo(m.X(pq[r][0]), m.Y(pq[r][1]));
              ctx.lineTo(m.X(q[r][0]), m.Y(q[r][1]));
            }
          }
        }
        ctx.stroke();
        ctx.restore();
      }
      if (s.prio?.rrt) {
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.globalAlpha = 0.35;
        ctx.lineWidth = 0.8;
        ctx.beginPath();
        for (const e of s.prio.rrt.edges()) {
          ctx.moveTo(m.X(e.a.q[0]), m.Y(e.a.q[1]));
          ctx.lineTo(m.X(e.b.q[0]), m.Y(e.b.q[1]));
        }
        ctx.stroke();
        ctx.restore();
      }
      const collidingPair = s.free.firstCollidingPair(s.pos);
      s.starts.forEach((q, i) => paintDot(ctx, m.X(q[0]), m.Y(q[1]), 5, p.start, i === 0 ? 1 : 0.6));
      s.goals.forEach((q, i) => paintDot(ctx, m.X(q[0]), m.Y(q[1]), 5, p.goal, i === 0 ? 1 : 0.6));
      s.pos.forEach((q, i) => {
        ctx.save();
        ctx.strokeStyle = collidingPair ? p.goal : p.robot;
        ctx.fillStyle = p.robot;
        ctx.lineWidth = 2;
        ctx.globalAlpha = 0.9;
        if (i === 1) ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.arc(m.X(q[0]), m.Y(q[1]), m.S(R), 0, Math.PI * 2);
        if (i === 0) ctx.fill();
        ctx.stroke();
        ctx.restore();
        label(ctx, `${i + 1}`, m.X(q[0]), m.Y(q[1]), i === 0 ? p.bg : p.robot, { size: 10, align: 'center', weight: 700 });
      });
      const title = s.mode === 'coupled' ? 'coupled: RRT-Connect on (ℝ²)²' : s.mode === 'decoupled' ? 'decoupled: paths alone + velocity schedule' : `prioritized ${s.mode === 'prio12' ? '1 → 2' : '2 → 1'}: RRT in 𝒬ᵢ × [0, T]`;
      const status = s.phase === 'planning' ? `planning… ${s.iterations} iterations` : s.phase === 'executing' ? `executing, t = ${s.t.toFixed(1)} s` : s.phase === 'done' ? 'done — conflict-free' : s.mode === 'decoupled' ? 'FAILED: no monotone schedule exists' : s.mode === 'coupled' ? 'FAILED within budget' : `FAILED: robot ${s.prio ? s.prio.order[s.prio.k] + 1 : ''} finds no trajectory`;
      label(ctx, title, m.X(s.c.bounds.minX), m.Y(s.c.bounds.maxY) - 10, p.ink, { size: 10, weight: 600 });
      label(ctx, status, m.X(s.c.bounds.maxX), m.Y(s.c.bounds.maxY) - 10, s.phase === 'failed' ? p.goal : s.phase === 'done' ? p.path : p.ink, { size: 10, weight: 700, align: 'right' });
      label(ctx, `corridor ${widthRadii.toFixed(1)} radii wide${widthRadii < 4 ? ' — the discs cannot pass; someone must step into the alcove' : ' — the discs can pass'}`, m.X(s.c.bounds.minX), m.Y(s.c.bounds.minY) + 10, p.ink, { size: 9.5 });

      // --- slice ---
      const ms = rectMapper(v, SLICE, s.c.bounds);
      const ras = sliceRaster(s.free, s.pos, 1, s.c.bounds, 72, 34);
      const cw = ms.S((s.c.bounds.maxX - s.c.bounds.minX) / ras.nx) + 0.6;
      const ch = ms.S((s.c.bounds.maxY - s.c.bounds.minY) / ras.ny) + 0.6;
      for (let j = 0; j < ras.ny; j++) {
        for (let i = 0; i < ras.nx; i++) {
          const val = ras.cells[j * ras.nx + i];
          if (!val) continue;
          const x = s.c.bounds.minX + (i / ras.nx) * (s.c.bounds.maxX - s.c.bounds.minX);
          const y = s.c.bounds.minY + ((j + 1) / ras.ny) * (s.c.bounds.maxY - s.c.bounds.minY);
          ctx.fillStyle = val === 1 ? tint(p.obstacle, 55, p.bg) : p.cobstacle;
          ctx.globalAlpha = val === 1 ? 0.6 : 0.75;
          ctx.fillRect(ms.X(x), ms.Y(y), cw, ch);
        }
      }
      ctx.globalAlpha = 1;
      paintPolyWorld(ctx, ms, world, p);
      if (s.alone[1]) drawPoly(ms, s.alone[1], p.graph, true, 1.2, 0.6);
      paintDot(ctx, ms.X(s.pos[1][0]), ms.Y(s.pos[1][1]), 4, p.robot);
      paintDot(ctx, ms.X(s.goals[1][0]), ms.Y(s.goals[1][1]), 4, p.goal, 0.8);
      label(ctx, 'robot 2’s slice of 𝒬 = (ℝ²)²: amber disc of radius r₁ + r₂ follows robot 1', ms.X(s.c.bounds.minX), ms.Y(s.c.bounds.maxY) - 10, p.ink, { size: 10, weight: 600 });

      // --- coordination diagram ---
      const md = rectMapper(v, DIAG, { minX: 0, minY: 0, maxX: 1, maxY: 1 });
      ctx.save();
      ctx.strokeStyle = p.ink;
      ctx.lineWidth = 1;
      ctx.strokeRect(md.X(0), md.Y(1), md.S(1), md.S(1));
      ctx.restore();
      if (s.diagram) {
        const n = s.diagram.n;
        const cs = md.S(1 / n) + 0.5;
        ctx.fillStyle = p.cobstacle;
        ctx.globalAlpha = 0.8;
        for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) if (s.diagram.occupied[j * n + i]) ctx.fillRect(md.X(i / n), md.Y((j + 1) / n), cs, cs);
        ctx.globalAlpha = 1;
        if (s.schedule) {
          ctx.save();
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 2.5;
          ctx.beginPath();
          s.schedule.params.forEach(([a, b], i) => (i === 0 ? ctx.moveTo(md.X(a), md.Y(b)) : ctx.lineTo(md.X(a), md.Y(b))));
          ctx.stroke();
          ctx.restore();
        } else {
          label(ctx, 'no monotone', md.X(0.5), md.Y(0.56), p.goal, { size: 10, align: 'center', weight: 700 });
          label(ctx, 'path (0,0)→(1,1)', md.X(0.5), md.Y(0.44), p.goal, { size: 10, align: 'center', weight: 700 });
        }
      }
      paintDot(ctx, md.X(0), md.Y(0), 4, p.start);
      paintDot(ctx, md.X(1), md.Y(1), 4, p.goal);
      label(ctx, 's₁', md.X(1), md.Y(0) + 11, p.ink, { size: 9.5, align: 'right' });
      label(ctx, 's₂', md.X(0) - 4, md.Y(1) + 4, p.ink, { size: 9.5, align: 'right' });
      label(ctx, 'coordination diagram', md.X(0.5), md.Y(1) - 10, p.ink, { size: 10, align: 'center', weight: 600 });
    },
    [s, widthRadii],
  );

  return (
    <WidgetFrame
      id="w14.1"
      title="Two-Robot Composite C-space"
      teaches="“Plan each robot separately and add a wait” fails on the swap problem: no wait works, and the alcove detour is only expressible in the composite space."
      colorKey={['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Top: two discs of radius 0.5 swapping ends in a corridor with one alcove. Bottom left: robot 2’s
          two-dimensional slice of the four-dimensional composite C-space — slate where robot 2 alone would
          hit a wall, amber where it would hit robot 1; drag either disc and the amber hole moves with it.
          Bottom right: the coordination diagram of the two paths planned alone, obstacle in amber; a monotone
          path from (0, 0) to (1, 1) is a velocity schedule, and below four radii of corridor width there is
          none. Switch the planner chips: decoupled fails, prioritized fails in both orders, and the coupled
          RRT-Connect on (ℝ²)² finds “robot 1 waits in the alcove” — at the price of four-dimensional samples.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={3 / 1.95} padding={0.05} draw={draw} deps={[sim.tick, s, drag]} ariaLabel="Workspace with two disc robots, robot 2's slice of the composite configuration space with a moving amber disc, and the coordination diagram with or without a monotone schedule." onPointer={onPointer} cursor={drag !== null ? 'grabbing' : 'grab'} />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="composite checks" value={s.free.checks} precision={0} />
          <StatTile label="rejected on the pair term only" value={s.free.pairOnlyRejections} precision={0} role="cobstacle" />
          <StatTile label="planner iterations" value={s.iterations} precision={0} role="graph" />
          <StatTile label="schedule from paths alone" value={s.schedule ? 'exists' : 'none'} role={s.schedule ? 'path' : 'goal'} />
        </Dashboard>
      </div>
      <ControlPanel columns={2} title="headline — corridor width">
        <Slider label="corridor width" role="obstacle" value={widthRadii} min={2.4} max={6} step={0.2} onChange={setWidthRadii} unit="radii" format={(x) => x.toFixed(1)} />
        <ButtonRow>
          <ActionButton emphasis={mode === 'decoupled'} onClick={() => setMode('decoupled')}>
            decoupled
          </ActionButton>
          <ActionButton emphasis={mode === 'prio12'} onClick={() => setMode('prio12')}>
            prioritized 1→2
          </ActionButton>
          <ActionButton emphasis={mode === 'prio21'} onClick={() => setMode('prio21')}>
            prioritized 2→1
          </ActionButton>
          <ActionButton emphasis={mode === 'coupled'} onClick={() => setMode('coupled')}>
            coupled (ℝ²)²
          </ActionButton>
        </ButtonRow>
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
