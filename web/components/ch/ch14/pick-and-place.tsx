'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { T2 } from '@/lib/manifold/torus';
import { paintArmMapped, paintDot, paintPolyWorld, rectMapper, tint } from '@/lib/sampling';
import { DEFAULT_MANIP, MANIP_WORLD, PUCK_RADIUS, manipScenario, rasterMode, type ManipPlan, type ManipulationPlanner, type Mode } from '@/lib/multi';
import type { Point2 } from '@/lib/sim/world';

/**
 * w14.2 — Pick-and-Place Modes.
 *
 * Reach moves a puck from A to B on the Workbench. Left: the workspace, the
 * puck slate at rest and orange when held. Right: the manipulation graph MG —
 * one column per placement, one row per grasp — with its edges colored by
 * verification status, and below it the two torus pictures: the transit
 * manifold's C-obstacle (the resting puck is an obstacle) and the transfer
 * manifold's (the held puck changes the robot's shape). The headline is |𝒢|.
 */

const ARENA = { minX: 0, minY: 0, maxX: 2.05, maxY: 1 };
const WORK = { x0: 0, y0: 0, x1: 1, y1: 1 };
const GRAPH = { x0: 1.1, y0: 0.5, x1: 2.05, y1: 1 };
const TOR1 = { x0: 1.1, y0: 0, x1: 1.55, y1: 0.45 };
const TOR2 = { x0: 1.6, y0: 0, x1: 2.05, y1: 0.45 };
const STEP = 0.05;
const RASTER_N = 48;

interface Frame {
  q: number[];
  puck: Point2;
  mode: Mode;
  seg: number;
}

interface State {
  planner: ManipulationPlanner;
  plan: ManipPlan | null;
  frames: Frame[];
  k: number;
  hold: number;
  rasters: Map<string, Uint8Array>;
}

function buildFrames(plan: ManipPlan): Frame[] {
  const frames: Frame[] = [];
  plan.segments.forEach((s, si) => {
    for (let i = 0; i + 1 < s.path.length; i++) {
      const a = [...s.path[i]];
      const b = [...s.path[i + 1]];
      const n = Math.max(1, Math.ceil(T2.dist(a, b) / STEP));
      for (let k = 0; k < n; k++) {
        const q = T2.interpolate(a, b, k / n);
        const puck = s.mode.kind === 'transit' ? s.puck[0] : { x: s.puck[i].x + (k / n) * (s.puck[i + 1].x - s.puck[i].x), y: s.puck[i].y + (k / n) * (s.puck[i + 1].y - s.puck[i].y) };
        frames.push({ q, puck, mode: s.mode, seg: si });
      }
    }
    // Pause at the mode switch so the color flip reads.
    const last = s.path[s.path.length - 1];
    for (let h = 0; h < 10; h++) frames.push({ q: [...last], puck: s.puck[s.puck.length - 1], mode: s.mode, seg: si });
  });
  return frames;
}

export function PickAndPlace() {
  const [grasps, setGrasps] = useState(2);
  const [withC, setWithC] = useState(true);
  const [eager, setEager] = useState(false);

  const init = useCallback(
    (seed: number): State => {
      const planner = manipScenario(grasps, withC, new Rng(seed), { ...DEFAULT_MANIP, eager });
      const plan = planner.plan();
      return { planner, plan, frames: plan ? buildFrames(plan) : [], k: 0, hold: 0, rasters: new Map() };
    },
    [grasps, withC, eager],
  );

  const stepFn = useCallback((s: State): State => {
    if (s.frames.length === 0) return s;
    s.k = (s.k + 1) % s.frames.length;
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 14 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [grasps, withC, eager]);

  const s = sim.state;
  const frame = s.frames.length ? s.frames[s.k] : null;
  const pa = s.planner.pa;

  const rasterFor = useCallback(
    (mode: Mode): Uint8Array => {
      const key = mode.kind === 'transit' ? `t${mode.placement}` : `g${mode.grasp}`;
      let r = s.rasters.get(key);
      if (!r) {
        r = rasterMode(s.planner.freeSpaceFor(mode), RASTER_N);
        s.rasters.set(key, r);
      }
      return r;
    },
    [s],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const m = rectMapper(v, WORK, MANIP_WORLD.bounds);
      paintPolyWorld(ctx, m, MANIP_WORLD, p);
      // Placements.
      s.planner.placements.forEach((pl, i) => {
        ctx.save();
        ctx.strokeStyle = i === s.planner.pStart ? p.start : i === s.planner.pGoal ? p.goal : p.ink;
        ctx.setLineDash([4, 3]);
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(m.X(pl.x), m.Y(pl.y), m.S(PUCK_RADIUS), 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
        label(ctx, pl.name, m.X(pl.x), m.Y(pl.y) - m.S(PUCK_RADIUS) - 8, p.ink, { size: 10, align: 'center', weight: 700 });
      });
      const q = frame ? frame.q : [...s.planner.qStart];
      const puck = frame ? frame.puck : s.planner.placements[s.planner.pStart];
      const held = frame ? frame.mode.kind === 'transfer' : false;
      // Puck: slate at rest, orange when held — the two modes as a color.
      ctx.save();
      ctx.fillStyle = held ? p.robot : p.obstacle;
      ctx.globalAlpha = 0.9;
      ctx.beginPath();
      ctx.arc(m.X(puck.x), m.Y(puck.y), m.S(PUCK_RADIUS), 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      paintArmMapped(ctx, m, p, pa.arm, q, p.robot);
      const modeText = frame ? (frame.mode.kind === 'transit' ? `transit — puck resting at ${s.planner.placements[frame.mode.placement].name}` : `transfer — holding with the ${s.planner.grasps[frame.mode.grasp].name} grasp`) : s.plan ? 'start' : 'no plan: MG is disconnected';
      label(ctx, modeText, m.X(MANIP_WORLD.bounds.minX) + 4, m.Y(MANIP_WORLD.bounds.maxY) + 12, s.plan ? (held ? p.robot : p.obstacle) : p.goal, { size: 10.5, weight: 700 });
      if (frame && s.plan) label(ctx, `segment ${frame.seg + 1} of ${s.plan.segments.length}`, m.X(MANIP_WORLD.bounds.maxX) - 4, m.Y(MANIP_WORLD.bounds.maxY) + 12, p.ink, { size: 10, align: 'right' });

      // --- MG ---
      const g = rectMapper(v, GRAPH, { minX: 0, minY: 0, maxX: 1, maxY: 1 });
      const P = s.planner.placements.length;
      const G = s.planner.grasps.length;
      const nodePos = (id: number): [number, number] => {
        const n = s.planner.nodes[id];
        if (n.g < 0) return [g.X(0.08), g.Y(0.5)];
        const col = 0.3 + (0.62 * n.p) / Math.max(1, P - 1);
        const row = G === 1 ? 0.5 : 0.12 + (0.76 * n.g) / (G - 1);
        return [g.X(col + (n.branch === 1 ? 0.05 : -0.05)), g.Y(1 - row)];
      };
      label(ctx, 'manipulation graph MG: nodes (p, g), transit edges within a column, transfer edges across', g.X(0), g.Y(1) - 8, p.ink, { size: 9.5, weight: 600 });
      const onPath = new Set<string>();
      if (s.plan) for (let i = 0; i + 1 < s.plan.nodes.length; i++) onPath.add(`${Math.min(s.plan.nodes[i], s.plan.nodes[i + 1])}-${Math.max(s.plan.nodes[i], s.plan.nodes[i + 1])}`);
      for (const e of s.planner.edges) {
        const [ax, ay] = nodePos(e.a);
        const [bx, by] = nodePos(e.b);
        const key = `${Math.min(e.a, e.b)}-${Math.max(e.a, e.b)}`;
        ctx.save();
        if (onPath.has(key)) {
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 3;
        } else if (e.status === 'verified') {
          ctx.strokeStyle = p.graph;
          ctx.lineWidth = 1.5;
        } else if (e.status === 'failed') {
          ctx.strokeStyle = p.goal;
          ctx.lineWidth = 1;
          ctx.setLineDash([3, 3]);
        } else {
          ctx.strokeStyle = tint(p.ink, 70, p.bg);
          ctx.lineWidth = 1;
          ctx.globalAlpha = 0.35 + 0.65 * e.belief;
          ctx.setLineDash([2, 3]);
        }
        ctx.beginPath();
        ctx.moveTo(ax, ay);
        ctx.lineTo(bx, by);
        ctx.stroke();
        ctx.restore();
      }
      for (const n of s.planner.nodes) {
        const [x, y] = nodePos(n.id);
        const on = s.plan?.nodes.includes(n.id);
        const current = frame && s.plan ? s.plan.nodes[frame.seg + (frame.mode.kind === 'transfer' ? 1 : 0)] === n.id : false;
        paintDot(ctx, x, y, current ? 6 : 4, n.g < 0 ? p.start : n.p === s.planner.pGoal ? p.goal : on ? p.path : p.graph, on || n.g < 0 || n.p === s.planner.pGoal ? 1 : 0.6);
      }
      s.planner.placements.forEach((pl, i) => label(ctx, pl.name, g.X(0.3 + (0.62 * i) / Math.max(1, P - 1)), g.Y(0.02), s.planner.unreachable.includes(i) ? p.goal : p.ink, { size: 10, align: 'center', weight: 700 }));
      s.planner.grasps.forEach((gr, i) => label(ctx, gr.name, g.X(0.19), g.Y(1 - (G === 1 ? 0.5 : 0.12 + (0.76 * i) / (G - 1))), p.ink, { size: 9, align: 'right' }));
      if (!s.plan) label(ctx, 'disconnected', g.X(0.62), g.Y(0.5), p.goal, { size: 12, align: 'center', weight: 700 });

      // --- the two tori ---
      const mode1: Mode = frame && frame.mode.kind === 'transit' ? frame.mode : { kind: 'transit', placement: frame ? (frame.mode.kind === 'transfer' ? s.planner.pStart : s.planner.pStart) : s.planner.pStart };
      const mode2: Mode = frame && frame.mode.kind === 'transfer' ? frame.mode : { kind: 'transfer', grasp: s.plan ? (s.plan.segments.find((sg) => sg.mode.kind === 'transfer')?.mode as { kind: 'transfer'; grasp: number } | undefined)?.grasp ?? 0 : 0 };
      const torus = (rect: typeof TOR1, mode: Mode, title: string) => {
        const t = rectMapper(v, rect, { minX: 0, minY: 0, maxX: 1, maxY: 1 });
        const ras = rasterFor(mode);
        const cs = t.S(1 / RASTER_N) + 0.5;
        ctx.save();
        ctx.fillStyle = p.cobstacle;
        ctx.globalAlpha = 0.75;
        for (let j = 0; j < RASTER_N; j++) for (let i = 0; i < RASTER_N; i++) if (ras[j * RASTER_N + i]) ctx.fillRect(t.X(i / RASTER_N), t.Y((j + 1) / RASTER_N), cs, cs);
        ctx.restore();
        ctx.save();
        ctx.strokeStyle = p.ink;
        ctx.setLineDash([3, 3]);
        ctx.strokeRect(t.X(0), t.Y(1), t.S(1), t.S(1));
        ctx.restore();
        const u = (q[0] + Math.PI) / (2 * Math.PI);
        const w = (q[1] + Math.PI) / (2 * Math.PI);
        paintDot(ctx, t.X(u), t.Y(w), 4, p.robot);
        label(ctx, title, t.X(0.5), t.Y(1) - 7, p.ink, { size: 9, align: 'center', weight: 600 });
      };
      torus(TOR1, mode1, `transit torus (puck at ${s.planner.placements[mode1.kind === 'transit' ? mode1.placement : 0].name})`);
      torus(TOR2, mode2, `transfer torus (${s.planner.grasps[mode2.kind === 'transfer' ? mode2.grasp : 0]?.name ?? ''} grasp)`);
    },
    [s, frame, pa, rasterFor],
  );

  return (
    <WidgetFrame
      id="w14.2"
      title="Pick-and-Place Modes"
      teaches="Manipulation planning is not path planning with a bigger robot: motion lives in a union of lower-dimensional manifolds, and switching happens only at their intersections — the grasps."
      colorKey={['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Reach carries the puck from A to B. Slate puck: at rest, an obstacle, the arm on a transit
          manifold. Orange puck: held, part of the robot, the arm on a transfer manifold — and the amber
          C-obstacle on the torus changes shape between the two. The manipulation graph on the right has one
          node per feasible (placement, grasp); grey dashed edges are believed free (FuzzyPRM’s prior), blue
          were planned for real, purple is the chosen path. With one grasp B is unreachable and MG is
          disconnected; with two grasps and no spare placement it is still disconnected; add C and the plan
          regrasps there: transit, transfer, transit, transfer.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.05} padding={0.04} draw={draw} deps={[sim.tick, s]} ariaLabel="Reach on the Workbench carrying a puck between placements, beside the manipulation graph and two torus pictures of the transit and transfer C-obstacles." />
      <div className="border-t border-fd-border px-3 py-3">
        <Dashboard columns={4}>
          <StatTile label="MG nodes · edges" value={`${s.planner.nodes.length} · ${s.planner.edges.length}`} role="graph" />
          <StatTile label="edges verified / failed" value={`${s.planner.edges.filter((e) => e.status === 'verified').length} / ${s.planner.edges.filter((e) => e.status === 'failed').length}`} role="path" />
          <StatTile label="collision checks (level two)" value={s.planner.edges.reduce((a, e) => a + e.checks, 0)} precision={0} />
          <StatTile label="plan" value={s.plan ? `${s.plan.segments.length} segments` : 'none — MG disconnected'} role={s.plan ? 'path' : 'goal'} />
        </Dashboard>
      </div>
      <ControlPanel columns={3} title="headline — the grasp set">
        <Slider label="|𝒢| grasps" role="robot" value={grasps} min={1} max={6} step={1} onChange={setGrasps} format={(x) => `${x.toFixed(0)}: ${['front', '+ left', '+ right', '+ front-left', '+ front-right', '+ back-left'].slice(0, x).join(', ')}`} />
        <Toggle label="third placement C (a place to regrasp)" checked={withC} onChange={setWithC} />
        <Toggle label="eager: verify every MG edge up front" checked={eager} onChange={setEager} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
