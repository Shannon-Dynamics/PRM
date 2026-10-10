'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { BitGrid } from '@/lib/manifold/torus';
import { chartPanel, paintArmMapped, paintChartPath, paintDot, paintGeodesic, paintPanelFrame, paintPolyWorld, paintRaster, paintTreeEdges, rasterTorusCObstacle, rectMapper } from '@/lib/sampling';
import { MpnetLike, loadModels, makeWorldCase, randomQuery, type NeuralPlan, type WorldCase, type WorldGenerator } from '@/lib/learn';

/**
 * w22.3 — Neural Planner Autopsy.
 *
 * An MPNet-style waypoint policy, trained by behavior cloning on Chapter 11
 * roadmap paths from the training generator, hops Reach from q_start toward
 * q_goal. Every hop is handed to the verifier (Chapter 2's sound swept check
 * through Chapter 11's `Steer` contract) and drawn blue if certified, red if
 * not; every failed stretch triggers a Chapter 12 RRT-Connect repair whose
 * mini-tree appears in thin blue; if the repairs run out, RRT-Connect runs on
 * the whole query. The step scrub replays the autopsy. The second tab is a
 * world from the held-out generator — the shape family the policy never saw.
 */

const ARENA = { minX: 0, minY: 0, maxX: 2.2, maxY: 1 };
const PANEL_T = { x0: 0, y0: 0, x1: 1, y1: 1 };
const PANEL_W = { x0: 1.14, y0: 0, x1: 2.2, y1: 1 };

interface State {
  wc: WorldCase;
  start: number[];
  goal: number[];
  raster: BitGrid;
  plan: NeuralPlan;
  raw: NeuralPlan;
  /** Total replay steps: one per hop, one per repair, one for the final path. */
  total: number;
  step: number;
}

interface Params {
  gen: WorldGenerator;
  repair: boolean;
}

function build(seed: number, prms: Params): State {
  const wc = makeWorldCase(seed, prms.gen);
  const rng = new Rng(seed ^ 0x5bd1e995);
  const q = randomQuery(wc.lab, rng) ?? randomQuery(wc.lab, rng, 1.5, false) ?? [[0.8, 1.6], [-2.2, -1.2]];
  const planner = new MpnetLike(loadModels().policy, wc, { repairAttempts: 300, fallbackAttempts: 1500 });
  const raw = planner.plan(q[0], q[1], new Rng(seed + 1), { repair: false });
  const plan = prms.repair ? planner.plan(q[0], q[1], new Rng(seed + 1)) : raw;
  return { wc, start: q[0], goal: q[1], raster: rasterTorusCObstacle(wc.lab, 48), plan, raw, total: plan.hops.length + plan.repairs.length + 1, step: 0 };
}

export function NeuralPlannerAutopsy() {
  const [gen, setGen] = useState<WorldGenerator>('train');
  const [repair, setRepair] = useState(true);
  const [scrub, setScrub] = useState<number | null>(null);
  const params = useMemo<Params>(() => ({ gen, repair }), [gen, repair]);
  const init = useCallback((seed: number) => build(seed, params), [params]);

  const stepFn = useCallback((s: State): State => {
    if (s.step >= s.total) return s;
    return { ...s, step: s.step + 1 };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 4, initialSeed: 3 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
    setScrub(null);
  }, [params]);

  const s = sim.state;
  const step = scrub ?? s.step;
  const space = s.wc.lab.space;
  const hopsShown = Math.min(step, s.plan.hops.length);
  const repairsShown = Math.max(0, Math.min(step - s.plan.hops.length, s.plan.repairs.length));
  const showFinal = step >= s.total;
  const current = s.plan.waypoints[Math.min(hopsShown, s.plan.waypoints.length - 1)];

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const panel = chartPanel(v, space, PANEL_T);
      paintRaster(ctx, panel, s.raster, p.cobstacle, 0.7);
      paintPanelFrame(ctx, panel, p, true);
      // Repair trees (thin blue) and their bridges.
      for (let r = 0; r < repairsShown; r++) {
        const rep = s.plan.repairs[r];
        for (const nodes of rep.treeNodes) paintTreeEdges(ctx, panel, space, nodes, p.graph, { lineWidth: 0.7, alpha: 0.45 });
        if (rep.path) paintChartPath(ctx, panel, space, rep.path, p.graph, 2, 0.9);
      }
      // Hops: certified blue, failed red dashed.
      for (let i = 0; i < hopsShown; i++) {
        const h = s.plan.hops[i];
        ctx.save();
        ctx.strokeStyle = h.ok ? p.graph : p.goal;
        ctx.lineWidth = h.ok ? 2.2 : 2;
        if (!h.ok) ctx.setLineDash([4, 3]);
        ctx.beginPath();
        paintGeodesic(ctx, panel, space, h.from, h.to, 8);
        ctx.stroke();
        ctx.restore();
        const c = space.coords(h.to);
        paintDot(ctx, panel.X(c[0]), panel.Y(c[1]), 2.5, h.ok ? p.graph : p.goal);
      }
      if (showFinal && s.plan.path) paintChartPath(ctx, panel, space, s.plan.path, p.path, 3.2);
      const cs = space.coords(s.start);
      const cg = space.coords(s.goal);
      paintDot(ctx, panel.X(cs[0]), panel.Y(cs[1]), 6, p.start);
      paintDot(ctx, panel.X(cg[0]), panel.Y(cg[1]), 6, p.goal);
      const cc = space.coords(current);
      paintDot(ctx, panel.X(cc[0]), panel.Y(cc[1]), 4.5, p.robot);
      label(ctx, 'θ₁ →', panel.px + panel.w - 4, panel.py + panel.h + 12, p.ink, { size: 10, align: 'right' });
      label(ctx, '↑ θ₂', panel.px + 4, panel.py + 10, p.ink, { size: 10 });
      const phase = step < s.plan.hops.length ? `hop ${step} of ${s.plan.hops.length}` : step < s.total ? `repair ${step - s.plan.hops.length + 1} of ${s.plan.repairs.length}` : s.plan.path ? (s.plan.fellBack ? 'fallback path (RRT-Connect on the whole query)' : 'verified path') : 'no path within the repair budget';
      label(ctx, phase, panel.px + 4, panel.py - 9, step >= s.total ? p.path : p.ink, { size: 11, weight: 600 });

      // Workbench with the arm at the current waypoint.
      const m = rectMapper(v, PANEL_W, s.wc.world.bounds);
      paintPolyWorld(ctx, m, s.wc.world, p);
      paintArmMapped(ctx, m, p, s.wc.lab.arm, s.start, p.start, 0.5);
      paintArmMapped(ctx, m, p, s.wc.lab.arm, s.goal, p.goal, 0.5);
      if (showFinal && s.plan.path && s.plan.path.length > 1) {
        const path = s.plan.path;
        let L = 0;
        const seg: number[] = [0];
        for (let i = 1; i < path.length; i++) {
          L += space.dist(path[i - 1], path[i]);
          seg.push(L);
        }
        const t = ((sim.tick * 0.08) % 1.2) * L;
        let i = 1;
        while (i < seg.length - 1 && seg[i] < t) i++;
        const q = t >= L ? path[path.length - 1] : space.interpolate(path[i - 1], path[i], (t - seg[i - 1]) / Math.max(1e-9, seg[i] - seg[i - 1]));
        paintArmMapped(ctx, m, p, s.wc.lab.arm, q, p.robot, 1);
      } else paintArmMapped(ctx, m, p, s.wc.lab.arm, current, p.robot, 1);
      label(ctx, s.wc.gen === 'train' ? 'Workbench · training generator' : 'Workbench · held-out generator', m.X(s.wc.world.bounds.minX) + 4, m.Y(s.wc.world.bounds.maxY) - 9, p.ink, { size: 10, weight: 600 });
    },
    [s, space, step, hopsShown, repairsShown, showFinal, current, sim.tick],
  );

  const okHops = s.plan.hops.filter((h) => h.ok).length;

  return (
    <WidgetFrame
      id="w22.3"
      title="Neural Planner Autopsy"
      teaches="The net did not learn to plan; it learned the training distribution. The repair planner behind it is the guarantee, and Derivation 4 is why the demo is complete at all."
      colorKey={['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Each hop of the waypoint policy (η = 0.35 rad) is a proposal; the swept collision certificate accepts it (blue) or
          rejects it (red, dashed). A rejected stretch is bridged by RRT-Connect — the thin blue mini-tree — and if the local
          repairs fail the whole query goes to RRT-Connect, which is where the planner&rsquo;s completeness actually comes from.
          The strip below is the hop record; the tiles split the final path into the part the net proposed and the part the
          repair planner supplied. On the held-out tab the policy keeps hopping toward corridors that are not there. Turn
          repair off to see what the net alone delivers.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.2} padding={0.08} draw={draw} deps={[sim.tick, s, step]} ariaLabel="A two-link arm's torus with a neural planner's hops drawn blue when certified and red when colliding, repair trees, and the Workbench beside it." />
      <div className="border-t border-fd-border px-3 py-3">
        <div className="mb-3 flex flex-wrap items-center gap-1" aria-label="hop pass/fail strip">
          <span className="mr-2 text-xs text-fd-muted-foreground">hops</span>
          {s.plan.hops.map((h, i) => (
            <span
              key={i}
              title={`hop ${i + 1}: ${h.ok ? 'certified' : 'collision'}`}
              className="inline-block h-3 w-3 rounded-sm"
              style={{ background: i < hopsShown ? (h.ok ? 'var(--rm-graph)' : 'var(--rm-goal)') : 'transparent', border: '1px solid var(--rm-graph)', opacity: i < hopsShown ? 1 : 0.35 }}
            />
          ))}
          <span className="ml-2 text-xs text-fd-muted-foreground">
            {okHops}/{s.plan.hops.length} certified · {s.plan.reachedGoal ? 'reached the goal' : 'stalled short of the goal'}
          </span>
        </div>
        <Dashboard columns={4}>
          <StatTile label="hops certified" value={`${okHops} / ${s.plan.hops.length}`} role="graph" />
          <StatTile label="repairs" value={s.plan.repairs.length} precision={0} role="path" trend={s.plan.fellBack ? 1 : 0} trendLabel={s.plan.fellBack ? 'fell back to whole-query RRT-Connect' : 'local repairs only'} />
          <StatTile label="path by net / by repair" value={s.plan.path ? `${s.plan.byNet.toFixed(2)} / ${s.plan.byRepair.toFixed(2)}` : '—'} unit="rad" role="robot" />
          <StatTile label="net alone (no repair)" value={s.raw.path ? 'solved' : 'failed'} role={s.raw.path ? 'start' : 'goal'} trend={s.plan.checks} trendLabel="collision checks, with repair" />
        </Dashboard>
      </div>
      <ControlPanel columns={3} title="headline — which world">
        <ButtonRow>
          <ActionButton emphasis={gen === 'train'} onClick={() => setGen('train')}>
            training generator
          </ActionButton>
          <ActionButton emphasis={gen === 'heldout'} onClick={() => setGen('heldout')}>
            held-out generator
          </ActionButton>
        </ButtonRow>
        <Slider label="replay step" role="graph" value={step} min={0} max={s.total} step={1} onChange={(x) => setScrub(Math.round(x))} />
        <Toggle label="repair with RRT-Connect" checked={repair} onChange={setRepair} role="path" />
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={() => {
          setScrub(null);
          sim.stepOnce();
        }}
        onReset={() => {
          setScrub(null);
          sim.reset();
        }}
        onReseed={() => {
          setScrub(null);
          sim.reseed();
        }}
        seed={sim.seed}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}
