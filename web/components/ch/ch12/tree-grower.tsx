'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile } from '@/components/viz';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { WORKBENCH } from '@/lib/robots/worlds';
import { cellOf, type BitGrid } from '@/lib/manifold/torus';
import {
  Est,
  Rrt,
  StraightLine,
  TORUS_GOAL,
  TORUS_START,
  chartPanel,
  paintArmMapped,
  paintChartPath,
  paintDot,
  paintPanelFrame,
  paintPolyWorld,
  paintRaster,
  paintTreeEdges,
  rasterTorusCObstacle,
  rectMapper,
  torusLab,
  voronoiOwners,
  type FreeSpace,
  type TorusLab,
  type VoronoiRaster,
} from '@/lib/sampling';

/**
 * w12.1 — Tree Grower.
 *
 * An RRT (Choset Algorithms 10–11) or an EST (Algorithms 8–9) grown on Reach's
 * unwrapped torus toward a goal configuration. The headline is η, the step
 * `extend` takes toward q_rand; the Voronoi overlay shades every node's cell by
 * its area, so the reader can watch the pull land, frame after frame, on the
 * nodes that own the most unexplored volume — the Voronoi-bias lemma made a
 * picture. At p = 1 the goal bias turns the planner into greedy descent of
 * dist(·, q_goal) and the "stuck" badge lights when it wedges.
 */

type Kind = 'rrt' | 'est';

const ARENA = { minX: 0, minY: 0, maxX: 2.12, maxY: 1 };
const TORUS = { x0: 0, y0: 0, x1: 1, y1: 1 };
const BENCH = { x0: 1.12, y0: 0, x1: 2.12, y1: 1 };
const ITERS_PER_TICK = 4;
const VOR_N = 44;
const MAX_ITERS = 6000;

interface State {
  lab: TorusLab;
  kind: Kind;
  rrt: Rrt<number[]> | null;
  est: Est<number[]> | null;
  vor: VoronoiRaster | null;
  lastRand: number[] | null;
  lastNear: number | null;
  walk: number;
}

/** A free-space oracle for the Voronoi raster that reads the C-obstacle mask instead of calling the checker. */
function maskFreeSpace(grid: BitGrid): FreeSpace<number[]> {
  return {
    isFree: (q) => {
      const [i, j] = cellOf(grid, [q[0], q[1]]);
      return grid.occupied[j * grid.n1 + i] === 0;
    },
    checks: 0,
  };
}

export function TreeGrower() {
  const [eta, setEta] = useState(0.35);
  const [bias, setBias] = useState(0.05);
  const [kind, setKind] = useState<Kind>('rrt');
  const [overlay, setOverlay] = useState(true);

  const raster = useMemo(() => rasterTorusCObstacle(torusLab(), 72), []);
  const mask = useMemo(() => maskFreeSpace(raster), [raster]);

  const init = useCallback(
    (seed: number): State => {
      const lab = torusLab();
      const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
      const rrt = kind === 'rrt' ? new Rrt(lab.space, lab.free, [...TORUS_START], [...TORUS_GOAL], { steer, eta, goalBias: bias, goalRadius: eta }, new Rng(seed)) : null;
      const est = kind === 'est' ? new Est(lab.space, lab.free, [...TORUS_START], [...TORUS_GOAL], { steer, radius: eta, goalRadius: eta }, new Rng(seed)) : null;
      return { lab, kind, rrt, est, vor: null, lastRand: null, lastNear: null, walk: 0 };
    },
    [kind, eta, bias],
  );

  const overlayRef = useRef(overlay);
  overlayRef.current = overlay;

  const stepFn = useCallback((s: State, tick: number): State => {
    const solved = s.rrt ? s.rrt.solved : s.est!.solved;
    const its = s.rrt ? s.rrt.iterations : s.est!.iterations;
    if (!solved && its < MAX_ITERS) {
      for (let k = 0; k < ITERS_PER_TICK; k++) {
        if (s.rrt) {
          const it = s.rrt.extendOnce();
          s.lastRand = it.qRand;
          s.lastNear = it.near;
          if (s.rrt.solved) break;
        } else {
          const it = s.est!.extendOnce();
          s.lastNear = it.from;
          s.lastRand = null;
          if (s.est!.solved) break;
        }
      }
      const tree = s.rrt ? s.rrt.tree : s.est!.tree;
      if (overlayRef.current && tick % 3 === 0) s.vor = voronoiOwners(tree, VOR_N, VOR_N, mask);
    } else {
      s.walk++;
    }
    return { ...s };
  }, [mask]);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 12 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [kind, eta, bias]);

  const s = sim.state;
  const tree = s.rrt ? s.rrt.tree : s.est!.tree;
  const solved = s.rrt ? s.rrt.solved : s.est!.solved;
  const solvedAt = s.rrt ? s.rrt.solvedAt : s.est!.solvedAt;
  const iterations = s.rrt ? s.rrt.iterations : s.est!.iterations;
  const trapped = s.rrt ? s.rrt.trapped : s.est!.rejected;
  const path = s.rrt ? s.rrt.path() : s.est!.path();
  const stuck = s.rrt !== null && bias >= 1 && s.rrt.trappedStreak > 40 && !solved;
  const nodeHistory = s.rrt ? s.rrt.nodeHistory : s.est!.nodeHistory;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const space = s.lab.space;
      const panel = chartPanel(v, space, TORUS);
      // Voronoi overlay: cells shaded by their owner's share, so the frontier glows.
      if (overlay && s.vor) {
        const vor = s.vor;
        const cw = panel.w / vor.n1;
        const ch = panel.h / vor.n2;
        let maxShare = 0;
        for (let k = 0; k < vor.share.length; k++) maxShare = Math.max(maxShare, vor.share[k]);
        ctx.save();
        ctx.fillStyle = p.graph;
        for (let j = 0; j < vor.n2; j++) {
          for (let i = 0; i < vor.n1; i++) {
            const o = vor.owner[j * vor.n1 + i];
            if (o < 0) continue;
            ctx.globalAlpha = 0.04 + 0.4 * Math.sqrt(vor.share[o] / Math.max(1e-9, maxShare));
            ctx.fillRect(panel.px + i * cw, panel.py + panel.h - (j + 1) * ch, cw + 0.6, ch + 0.6);
          }
        }
        ctx.restore();
      }
      paintRaster(ctx, panel, raster, p.cobstacle, 0.8);
      paintPanelFrame(ctx, panel, p, true);
      paintTreeEdges(ctx, panel, space, tree.nodes, p.graph, { lineWidth: 1, alpha: 0.8 });
      // EST: node halos sized by 1/w_T — the lonely nodes are the big ones.
      if (s.est) {
        for (let i = 0; i < tree.size; i++) {
          const c = space.coords(tree.nodes[i].q);
          const r = 2 + 10 / Math.sqrt(s.est.weight[i]);
          paintDot(ctx, panel.X(c[0]), panel.Y(c[1]), r, p.graph, 0.12);
        }
      }
      for (let i = 0; i < tree.size; i++) {
        const c = space.coords(tree.nodes[i].q);
        paintDot(ctx, panel.X(c[0]), panel.Y(c[1]), 1.8, p.graph, 0.9);
      }
      // The last q_rand and the node it pulled.
      if (s.lastRand && !solved) {
        const c = space.coords(s.lastRand);
        ctx.save();
        ctx.strokeStyle = p.ink;
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.globalAlpha = 0.7;
        if (s.lastNear !== null) {
          const n = space.coords(tree.nodes[s.lastNear].q);
          ctx.beginPath();
          ctx.moveTo(panel.X(n[0]), panel.Y(n[1]));
          ctx.lineTo(panel.X(c[0]), panel.Y(c[1]));
          ctx.stroke();
        }
        ctx.restore();
        paintDot(ctx, panel.X(c[0]), panel.Y(c[1]), 4, p.ink, 0.8);
        label(ctx, 'q_rand', panel.X(c[0]) + 6, panel.Y(c[1]) - 6, p.ink, { size: 9 });
      }
      if (s.lastNear !== null && !solved) {
        const n = space.coords(tree.nodes[s.lastNear].q);
        ctx.save();
        ctx.strokeStyle = p.robot;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(panel.X(n[0]), panel.Y(n[1]), 6, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
      }
      if (path) paintChartPath(ctx, panel, space, path, p.path, 3);
      const cs = space.coords(TORUS_START);
      const cg = space.coords(TORUS_GOAL);
      paintDot(ctx, panel.X(cs[0]), panel.Y(cs[1]), 6, p.start);
      paintDot(ctx, panel.X(cg[0]), panel.Y(cg[1]), 6, p.goal);
      ctx.save();
      ctx.strokeStyle = p.goal;
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.arc(panel.X(cg[0]), panel.Y(cg[1]), (eta / (2 * Math.PI)) * panel.w, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
      label(ctx, 'θ₁ →', panel.px + panel.w - 4, panel.py + panel.h + 12, p.ink, { size: 10, align: 'right' });
      if (stuck) label(ctx, 'STUCK — potential-field mode (p = 1)', panel.px + panel.w / 2, panel.py + 14, p.goal, { size: 11, weight: 700, align: 'center' });
      else if (solved) label(ctx, `solved at iteration ${solvedAt}`, panel.px + 6, panel.py + 14, p.path, { size: 11, weight: 700 });

      // The Workbench: Reach at the newest node, or walking the path once solved.
      const m = rectMapper(v, BENCH, WORKBENCH.bounds);
      paintPolyWorld(ctx, m, WORKBENCH, p);
      paintArmMapped(ctx, m, p, s.lab.arm, TORUS_START, p.start, 0.4);
      paintArmMapped(ctx, m, p, s.lab.arm, TORUS_GOAL, p.goal, 0.4);
      let q = tree.nodes[tree.size - 1].q;
      if (path && path.length > 1) {
        let L = 0;
        const seg = [0];
        for (let i = 1; i < path.length; i++) {
          L += space.dist(path[i - 1], path[i]);
          seg.push(L);
        }
        const t = ((s.walk * 0.025) % 1.25) * L;
        let i = 1;
        while (i < seg.length - 1 && seg[i] < t) i++;
        q = t >= L ? path[path.length - 1] : space.interpolate(path[i - 1], path[i], (t - seg[i - 1]) / Math.max(1e-9, seg[i] - seg[i - 1]));
      }
      paintArmMapped(ctx, m, p, s.lab.arm, q, p.robot, 1);
      label(ctx, 'Workbench', m.X(WORKBENCH.bounds.minX) + 4, m.Y(WORKBENCH.bounds.maxY) + 10, p.ink, { size: 10 });
    },
    [s, tree, solved, solvedAt, path, stuck, overlay, raster, eta],
  );

  const history = useMemo(() => {
    const stride = Math.max(1, Math.floor(nodeHistory.length / 150));
    const data: { x: number; y: number }[] = [{ x: 0, y: 1 }];
    for (let i = stride - 1; i < nodeHistory.length; i += stride) data.push({ x: i + 1, y: nodeHistory[i] });
    const diag = data.map((d) => ({ x: d.x, y: d.x }));
    return { data, diag };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [Math.floor(sim.tick / 5), nodeHistory.length]);

  return (
    <WidgetFrame
      id="w12.1"
      title="Tree Grower"
      teaches="RRT does not explore randomly: it explores where the Voronoi cells are large, a deterministic function of the tree — and the random target only decides which large cell goes next."
      colorKey={['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          A tree grown from Reach&rsquo;s home configuration (green) toward the goal (red) on the
          unwrapped torus; amber cells are configurations in collision. Each iteration draws a random
          target <em>q_rand</em> (black dot), finds the nearest tree node (orange ring) and moves η
          toward it. With the overlay on, every node&rsquo;s Voronoi cell is shaded by its area:
          the probability that the next iteration pulls that node. Watch the big cells get chosen and
          shrink. Slide η: small steps make a dense, short-edged tree; large ones flash{' '}
          <em>Trapped</em> at the amber band. Set the goal bias to 1 and the planner is Chapter
          7&rsquo;s gradient descent, wedging against the first wall. Switch to EST and the halos show{' '}
          <em>w_T</em> — the lonely nodes are pushed.
        </>
      }
    >
      <SimCanvas world={ARENA} aspect={2.12} padding={0.06} draw={draw} deps={[sim.tick, s, overlay]} ariaLabel="A rapidly-exploring random tree on the unwrapped torus of a two-link arm, Voronoi cells shaded by area, beside the Workbench where the arm follows the newest node." />
      <div className="border-t border-fd-border px-3 py-3">
        <div className="grid grid-cols-1 gap-3 lg:grid-cols-[minmax(0,22rem)_1fr]">
          <Dashboard columns={2}>
            <StatTile label="iterations" value={iterations} precision={0} />
            <StatTile label="nodes" value={tree.size} precision={0} role="graph" sparkline={nodeHistory.slice(-200)} />
            <StatTile label={s.rrt ? 'Trapped' : 'rejected'} value={trapped} precision={0} role="obstacle" trend={iterations ? Math.round((100 * trapped) / iterations) : 0} trendLabel="% of iterations" />
            <StatTile label="first solution" value={solvedAt ?? '—'} role="path" />
          </Dashboard>
          <LineChart
            series={[
              { id: 'nodes', role: 'graph', data: history.data },
              { id: 'iterations (no Trapped)', role: 'obstacle', data: history.diag },
            ]}
            xLabel="iterations"
            yLabel="nodes"
            height={170}
            ariaLabel="Tree size against iterations; the gap to the diagonal is the number of Trapped extensions."
          />
        </div>
      </div>
      <ControlPanel columns={3} title="headline — step size">
        <Slider label="step size η" role="graph" value={eta} min={0.1} max={1.2} step={0.05} onChange={setEta} unit="rad" />
        <Slider label="goal bias p" role="goal" value={bias} min={0} max={1} step={0.05} onChange={setBias} format={(x) => (x >= 1 ? '1 — potential field' : x.toFixed(2))} />
        <ButtonRow>
          <ActionButton emphasis={kind === 'rrt'} onClick={() => setKind('rrt')}>
            RRT (pull)
          </ActionButton>
          <ActionButton emphasis={kind === 'est'} onClick={() => setKind('est')}>
            EST (push)
          </ActionButton>
          <Toggle label="Voronoi overlay" role="graph" checked={overlay} onChange={setOverlay} />
        </ButtonRow>
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}


