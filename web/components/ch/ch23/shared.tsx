'use client';

import type { ReactNode } from 'react';
import { drawCovariance, drawPath, drawRobot, drawSegments, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { chartPanel, paintArmMapped, paintChartPath, paintPanelFrame, paintPolyWorld, paintRaster, paintTreeEdges, rectMapper, type ChartPanel, type Mapper } from '@/lib/sampling/paint';
import { T2, type BitGrid } from '@/lib/manifold/torus';
import { WORKBENCH, LOT } from '@/lib/robots/worlds';
import type { Point2 } from '@/lib/sim/world';
import type { Pose2 } from '@/lib/geom/se2';
import type { HitchCfg } from '@/lib/robots/hitch';
import { ellipse2 } from '@/lib/prob/linalg';
import { BLUEPRINT, boxSegments } from '@/lib/capstone/worlds';
import type { RustyStack } from '@/lib/capstone/rusty';
import type { ReachStack } from '@/lib/capstone/reach';
import type { HitchStack } from '@/lib/capstone/hitch';
import { PHI_MAX, PHI_WARN } from '@/lib/capstone/hitch';
import type { Mode } from '@/lib/capstone/supervisor';
import type { MissionEvent } from '@/lib/capstone/mission';
import type { OccupancyGrid } from '@/lib/search/grid';

/**
 * What Chapter 23's widgets share: painters for the three panels and a few
 * readout components. Painters draw what the stacks *published* (the believed
 * grid, the plan, the trajectory, the guard statistics); ground truth only on
 * request. Colors come from the palette the canvas hands in — planning roles
 * for planning objects, the estimation roles only for Rusty's belief.
 */

/* -------------------------------------------------------------------------- */
/* Small markup components                                                     */
/* -------------------------------------------------------------------------- */

export function Panel({ title, children }: { title: string; children: ReactNode }) {
  return (
    <div className="border-b border-fd-border px-3 py-2 last:border-b-0">
      <p className="eyebrow mb-1.5">{title}</p>
      {children}
    </div>
  );
}

export function Row({ k, v, color }: { k: string; v: string; color?: string }) {
  return (
    <div className="flex justify-between gap-3 font-mono text-[0.7rem] tabular-nums">
      <dt className="text-fd-muted-foreground">{k}</dt>
      <dd style={color ? { color } : undefined}>{v}</dd>
    </div>
  );
}

export function Bar({ frac, color, mark, warn = false }: { frac: number; color: string; mark?: number; warn?: boolean }) {
  const f = Math.max(0, Math.min(1, Number.isFinite(frac) ? frac : 1));
  return (
    <div className="relative h-1.5 w-full overflow-hidden rounded-full bg-fd-muted">
      <div className="h-full rounded-full" style={{ width: `${f * 100}%`, backgroundColor: warn ? 'var(--rm-goal)' : color }} />
      {mark !== undefined ? <div className="absolute top-0 h-full w-px" style={{ left: `${Math.min(1, Math.max(0, mark)) * 100}%`, backgroundColor: 'var(--pr-widget-ink)' }} /> : null}
    </div>
  );
}

/** The CSS color of a mode's dot: planning purple, executing orange, recovering amber, done green, failed red. */
export function modeVar(kind: Mode['kind']): string {
  switch (kind) {
    case 'Plan':
    case 'Replan':
      return 'var(--rm-path)';
    case 'Execute':
      return 'var(--rm-robot)';
    case 'Recover':
      return 'var(--rm-cobstacle)';
    case 'Done':
      return 'var(--rm-start)';
    case 'Failed':
      return 'var(--rm-goal)';
  }
}

export function ModeMachine({ name, mode, detail }: { name: string; mode: Mode; detail: string }) {
  const kinds: Mode['kind'][] = ['Plan', 'Execute', 'Replan', 'Recover', 'Done', 'Failed'];
  return (
    <div className="mb-2 last:mb-0">
      <div className="mb-1 flex items-baseline justify-between">
        <span className="font-ui text-[0.72rem] font-semibold">{name}</span>
        <span className="font-mono text-[0.62rem] text-fd-muted-foreground">{detail}</span>
      </div>
      <div className="flex flex-wrap gap-1">
        {kinds.map((k) => {
          const on = mode.kind === k;
          return (
            <span
              key={k}
              className="rounded-sm border px-1 py-px font-mono text-[0.6rem]"
              style={{
                borderColor: on ? modeVar(k) : 'var(--color-fd-border)',
                color: on ? modeVar(k) : 'var(--color-fd-muted-foreground)',
                opacity: on ? 1 : 0.55,
                fontWeight: on ? 700 : 400,
              }}
            >
              {k}
            </span>
          );
        })}
      </div>
    </div>
  );
}

/** Canvas color for an event pin. */
export function eventColor(p: Palette, kind: MissionEvent['kind']): string {
  switch (kind) {
    case 'chaos':
      return p.goal;
    case 'detect':
      return p.cobstacle;
    case 'done':
      return p.start;
    case 'fail':
      return p.goal;
    case 'mode':
      return p.path;
    default:
      return p.grid;
  }
}

/* -------------------------------------------------------------------------- */
/* Apartment                                                                   */
/* -------------------------------------------------------------------------- */

/** Occupied cells as run-length rectangles: a few hundred fillRects instead of ten thousand. */
export function fillGridRuns(ctx: CanvasRenderingContext2D, v: Viewport, g: OccupancyGrid, mask: (k: number) => boolean, color: string, alpha: number) {
  const cs = g.cellSize;
  ctx.save();
  ctx.fillStyle = color;
  ctx.globalAlpha = alpha;
  for (let j = 0; j < g.ny; j++) {
    let i = 0;
    while (i < g.nx) {
      if (!mask(j * g.nx + i)) {
        i++;
        continue;
      }
      const i0 = i;
      while (i < g.nx && mask(j * g.nx + i)) i++;
      const x0 = g.origin.x + i0 * cs;
      const y1 = g.origin.y + (j + 1) * cs;
      ctx.fillRect(sx(v, x0), sy(v, y1), sl(v, (i - i0) * cs) + 0.5, sl(v, cs) + 0.5);
    }
  }
  ctx.restore();
}

export interface ApartmentOptions {
  truth: boolean;
  scan: boolean;
  touched: boolean;
  particles: boolean;
}

export function paintApartment(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, r: RustyStack, o: ApartmentOptions) {
  ctx.clearRect(0, 0, v.width, v.height);
  ctx.fillStyle = p.bg;
  ctx.fillRect(0, 0, v.width, v.height);
  const believed = r.dstar.believed;
  // C-obstacles of the believed grid (amber), then the cells the map learned (deeper amber).
  fillGridRuns(ctx, v, believed, (k) => believed.occupied[k] === 1, p.cobstacle, 0.22);
  fillGridRuns(ctx, v, r.grid, (k) => r.confirmed.has(k), p.cobstacle, 0.9);
  if (o.touched) {
    const t = r.dstar.stats.touched;
    fillGridRuns(ctx, v, believed, (k) => t.has(k), p.graph, 0.45);
  }
  drawSegments(ctx, v, BLUEPRINT.walls, p.obstacle, 2.2);
  if (o.truth) {
    ctx.save();
    ctx.setLineDash([3, 3]);
    drawSegments(ctx, v, r.opts.furniture.flatMap(boxSegments), p.truth, 1.6);
    if (r.shutDoor) drawSegments(ctx, v, [r.shutDoor], p.truth, 3);
    ctx.restore();
  } else if (r.shutDoor) {
    // A shut door is a fact of the world, not of the map — drawn faint until the map learns it.
    ctx.save();
    ctx.globalAlpha = 0.35;
    drawSegments(ctx, v, [r.shutDoor], p.obstacle, 3);
    ctx.restore();
  }
  // Start and goal.
  ctx.save();
  ctx.fillStyle = p.start;
  ctx.beginPath();
  ctx.arc(sx(v, 1.0), sy(v, 1.0), 5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.fillStyle = p.goal;
  ctx.beginPath();
  ctx.arc(sx(v, r.goal.x), sy(v, r.goal.y), 6, 0, 2 * Math.PI);
  ctx.fill();
  ctx.restore();
  if (o.scan) {
    const scan = r.bus.latest<number[]>('rusty/scan');
    if (scan) {
      ctx.save();
      ctx.fillStyle = p.measurement;
      ctx.globalAlpha = 0.75;
      const pose = r.truth;
      for (let i = 0; i < scan.v.length; i++) {
        if (scan.v[i] >= 3.999) continue;
        const a = pose.theta + r.angles[i];
        ctx.fillRect(sx(v, pose.x + scan.v[i] * Math.cos(a)) - 1, sy(v, pose.y + scan.v[i] * Math.sin(a)) - 1, 2, 2);
      }
      ctx.restore();
    }
  }
  if (r.trail.length > 1) drawPath(ctx, v, r.trail, p.robot, { lineWidth: 1.5, alpha: 0.6 });
  if (r.path.length > 1) drawPath(ctx, v, r.path, p.path, { lineWidth: 2.4 });
  if (r.rollout.length > 1) drawPath(ctx, v, r.rollout, p.robot, { lineWidth: 2.6 });
  if (r.blocked) {
    ctx.save();
    ctx.strokeStyle = p.goal;
    ctx.lineWidth = 2;
    const b = r.blocked.point;
    ctx.beginPath();
    ctx.arc(sx(v, b.x), sy(v, b.y), 7, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.restore();
  }
  const bel = r.beliefMsg();
  if (o.particles) {
    ctx.save();
    ctx.fillStyle = p.posterior;
    ctx.globalAlpha = 0.5;
    for (const q of r.belief.pf.particles) ctx.fillRect(sx(v, q.state.x) - 1, sy(v, q.state.y) - 1, 2, 2);
    ctx.restore();
  }
  // The belief as an outline only (posterior purple would fight the path purple as a fill).
  const e = ellipse2(bel.cov, 3);
  drawCovariance(ctx, v, [bel.pose.x, bel.pose.y], { rx: Math.max(e.rx, 0.02), ry: Math.max(e.ry, 0.02), angle: e.angle }, p.posterior, { fill: false, lineWidth: 1.4 });
  if (o.truth) drawRobot(ctx, v, r.truth, p.truth, 0.3, { filled: false });
  drawRobot(ctx, v, bel.pose, p.robot, 0.3);
}

/* -------------------------------------------------------------------------- */
/* Workbench                                                                   */
/* -------------------------------------------------------------------------- */

/** The Workbench canvas is 2 × 1.36 world units: workspace square left, torus square right, phase strip below. */
export const BENCH_WORLD = { minX: 0, minY: 0, maxX: 2, maxY: 1.36 };

export interface BenchLayout {
  ws: Mapper;
  torus: ChartPanel;
  strip: { x0: number; y0: number; x1: number; y1: number };
}

export function benchLayout(v: Viewport): BenchLayout {
  const ws = rectMapper(v, { x0: 0.02, y0: 0.36, x1: 0.98, y1: 1.34 }, WORKBENCH.bounds);
  const torus = chartPanel(v, T2, { x0: 1.06, y0: 0.4, x1: 1.96, y1: 1.3 });
  return { ws, torus, strip: { x0: 0.04, y0: 0.03, x1: 1.96, y1: 0.3 } };
}

export function paintWorkbench(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, a: ReachStack, raster: BitGrid | null, showTree: boolean) {
  ctx.clearRect(0, 0, v.width, v.height);
  ctx.fillStyle = p.bg;
  ctx.fillRect(0, 0, v.width, v.height);
  const L = benchLayout(v);
  // Workspace: obstacles slate, the executed path's ghosts, the arm orange.
  paintPolyWorld(ctx, L.ws, WORKBENCH, p);
  const tip = a.tip;
  ctx.save();
  ctx.fillStyle = p.goal;
  ctx.beginPath();
  ctx.arc(L.ws.X(tip.x), L.ws.Y(tip.y), 4.5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.strokeStyle = p.grid;
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.arc(L.ws.X(0), L.ws.Y(0), L.ws.S(2), 0, 2 * Math.PI);
  ctx.stroke();
  ctx.restore();
  if (a.executing && a.executing.kind === 'Trajectory') {
    const s = a.executing.traj.samples;
    for (let i = 0; i < s.length; i += Math.max(1, Math.floor(s.length / 6))) paintArmMapped(ctx, L.ws, p, a.arm, s[i].q, p.path, 0.18);
  }
  paintArmMapped(ctx, L.ws, p, a.arm, a.q, p.robot, 1);
  // Torus: C-obstacle raster, tree, paths, the arm's configuration.
  if (raster) paintRaster(ctx, L.torus, raster, p.cobstacle, 0.55);
  if (showTree && a.rrt) paintTreeEdges(ctx, L.torus, T2, a.rrt.tree.inner.nodes, p.graph, { lineWidth: 0.8, alpha: 0.55 });
  if (a.shortPath) paintChartPath(ctx, L.torus, T2, a.shortPath, p.path, 2.2, 1);
  if (a.chompXi && a.chompProblemPoints()) paintChartPath(ctx, L.torus, T2, a.chompProblemPoints()!, p.path, 1.4, 0.8, true);
  paintPanelFrame(ctx, L.torus, p, true);
  const c = T2.coords(a.q);
  ctx.save();
  ctx.fillStyle = p.robot;
  ctx.beginPath();
  ctx.arc(L.torus.X(c[0]), L.torus.Y(c[1]), 4.5, 0, 2 * Math.PI);
  ctx.fill();
  if (a.qGoal) {
    const g = T2.coords(a.qGoal);
    ctx.fillStyle = p.goal;
    ctx.beginPath();
    ctx.arc(L.torus.X(g[0]), L.torus.Y(g[1]), 4, 0, 2 * Math.PI);
    ctx.fill();
  }
  ctx.restore();
  label(ctx, 'workspace', L.ws.X(-2.35), L.ws.Y(2.2), p.ink, { size: 9 });
  label(ctx, 'T² (θ₁, θ₂)', L.torus.px + 4, L.torus.py + 11, p.ink, { size: 9 });
  paintPhaseStrip(ctx, v, p, a, L.strip);
}

/** The (s, ṡ) phase plane of the trajectory being executed: limit curve and the profile. */
export function paintPhaseStrip(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, a: ReachStack, r: { x0: number; y0: number; x1: number; y1: number }) {
  const exec = a.executing && a.executing.kind === 'Trajectory' ? a.executing : null;
  const sc = exec?.scaling ?? null;
  const X = (s: number) => sx(v, r.x0 + s * (r.x1 - r.x0));
  ctx.save();
  ctx.strokeStyle = p.grid;
  ctx.strokeRect(sx(v, r.x0), sy(v, r.y1), sx(v, r.x1) - sx(v, r.x0), sy(v, r.y0) - sy(v, r.y1));
  ctx.restore();
  if (!sc) {
    label(ctx, 'phase plane (s, ṡ): waiting for a Trajectory', sx(v, r.x0) + 6, sy(v, r.y1) + 13, p.ink, { size: 9 });
    return;
  }
  let peak = 0;
  for (let k = 0; k <= sc.N; k++) peak = Math.max(peak, sc.sdot[k]);
  // Scale to the profile, letting the limit curve show up to three times its peak.
  let top = peak;
  for (let k = 0; k <= sc.N; k++) {
    const lim = sc.limit[k].sdotMax;
    if (Number.isFinite(lim)) top = Math.max(top, Math.min(lim, 3 * peak));
  }
  top = Math.max(top, 1e-6) * 1.08;
  const Y = (sd: number) => sy(v, r.y0 + (Math.min(sd, top) / top) * (r.y1 - r.y0));
  ctx.save();
  ctx.strokeStyle = p.cobstacle;
  ctx.lineWidth = 1.3;
  ctx.beginPath();
  let pen = false;
  for (let k = 0; k <= sc.N; k++) {
    const lim = sc.limit[k].sdotMax;
    if (!Number.isFinite(lim)) {
      pen = false;
      continue;
    }
    if (!pen) ctx.moveTo(X(sc.s[k]), Y(lim));
    else ctx.lineTo(X(sc.s[k]), Y(lim));
    pen = true;
  }
  ctx.stroke();
  ctx.strokeStyle = p.path;
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let k = 0; k <= sc.N; k++) (k === 0 ? ctx.moveTo : ctx.lineTo).call(ctx, X(sc.s[k]), Y(sc.sdot[k]));
  ctx.stroke();
  // Where the arm is now along s.
  const k = Math.min(a.execIndex, exec!.traj.samples.length - 1);
  const s = exec!.traj.samples[k];
  ctx.fillStyle = p.robot;
  ctx.beginPath();
  ctx.arc(X(s.s), Y(s.sdot), 3.5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.restore();
  label(ctx, `(s, ṡ) · t_f ${exec!.traj.tf.toFixed(2)} s · ${exec!.source}`, sx(v, r.x0) + 6, sy(v, r.y1) + 12, p.ink, { size: 9 });
}

/* -------------------------------------------------------------------------- */
/* Lot                                                                         */
/* -------------------------------------------------------------------------- */

function fillPoly(ctx: CanvasRenderingContext2D, v: Viewport, poly: readonly Point2[], color: string, alpha: number, stroke = false) {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.beginPath();
  poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
  ctx.closePath();
  if (stroke) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.stroke();
  } else {
    ctx.fillStyle = color;
    ctx.fill();
  }
  ctx.restore();
}

export function paintHitchCfg(ctx: CanvasRenderingContext2D, v: Viewport, h: HitchStack, q: HitchCfg, color: string, alpha: number, outline = false) {
  for (const poly of h.car.footprint(q)) fillPoly(ctx, v, poly, color, alpha, outline);
  const ax = h.car.trailerAxle(q);
  if (ax) {
    ctx.save();
    ctx.strokeStyle = color;
    ctx.globalAlpha = alpha;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(sx(v, q.body.x), sy(v, q.body.y));
    ctx.lineTo(sx(v, ax.x), sy(v, ax.y));
    ctx.stroke();
    ctx.restore();
  }
}

export function paintLot(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, h: HitchStack, showSearch: boolean) {
  ctx.clearRect(0, 0, v.width, v.height);
  ctx.fillStyle = p.bg;
  ctx.fillRect(0, 0, v.width, v.height);
  if (LOT.markings) drawSegments(ctx, v, LOT.markings, p.grid, 2);
  for (const o of LOT.obstacles) fillPoly(ctx, v, o.poly, p.obstacle, 1);
  drawSegments(ctx, v, LOT.walls, p.wall, 2.5);
  // Goal pose (red outline of the parked footprint).
  paintHitchCfg(ctx, v, h, h.car.cfg(h.goal.x, h.goal.y, h.goal.theta), p.goal, 0.9, true);
  if (showSearch && h.plan) {
    ctx.save();
    ctx.fillStyle = p.graph;
    ctx.globalAlpha = 0.7;
    for (const s of h.plan.searches) for (const e of s.expanded) ctx.fillRect(sx(v, e.q.x) - 1.5, sy(v, e.q.y) - 1.5, 3, 3);
    ctx.restore();
  }
  for (const rej of h.rejected) drawPath(ctx, v, rej.samples.map((s) => s.q), p.goal, { lineWidth: 1, alpha: 0.35, dashed: true });
  if (h.plan) {
    drawPath(ctx, v, h.plan.samples.map((s) => s.q), p.path, { lineWidth: 2.2 });
    ctx.save();
    ctx.strokeStyle = p.path;
    ctx.lineWidth = 1.5;
    for (const c of h.cuspPoints()) {
      ctx.beginPath();
      ctx.arc(sx(v, c.x), sy(v, c.y), 5, 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.restore();
  }
  const ex = h.executed;
  if (ex.length > 1) drawPath(ctx, v, ex.map((q) => q.body), p.robot, { lineWidth: 1.6, alpha: 0.7 });
  paintHitchCfg(ctx, v, h, h.q, p.robot, 0.85);
}

/** A semicircular hitch-angle gauge with the plan, warn and hard bounds. */
export function paintPhiGauge(ctx: CanvasRenderingContext2D, cx: number, cy: number, R: number, p: Palette, phi: number, phiPlan: number) {
  const ang = (x: number) => Math.PI * 1.5 + (Math.max(-1.6, Math.min(1.6, x)) / 1.6) * (Math.PI / 2);
  ctx.save();
  ctx.lineWidth = 6;
  ctx.strokeStyle = p.grid;
  ctx.beginPath();
  ctx.arc(cx, cy, R, Math.PI, 2 * Math.PI);
  ctx.stroke();
  for (const [b, c] of [[phiPlan, p.path], [PHI_WARN, p.cobstacle], [PHI_MAX, p.goal]] as const) {
    for (const sgn of [-1, 1]) {
      const a = ang(sgn * b);
      ctx.strokeStyle = c;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx + (R - 7) * Math.cos(a), cy + (R - 7) * Math.sin(a));
      ctx.lineTo(cx + (R + 7) * Math.cos(a), cy + (R + 7) * Math.sin(a));
      ctx.stroke();
    }
  }
  const a = ang(phi);
  ctx.strokeStyle = Math.abs(phi) > PHI_WARN ? p.goal : p.robot;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + R * Math.cos(a), cy + R * Math.sin(a));
  ctx.stroke();
  ctx.restore();
}

/** Heading arrow helper for pose markers. */
export function poseArrow(ctx: CanvasRenderingContext2D, v: Viewport, q: Pose2, color: string, len = 0.6) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(sx(v, q.x), sy(v, q.y));
  ctx.lineTo(sx(v, q.x + len * Math.cos(q.theta)), sy(v, q.y + len * Math.sin(q.theta)));
  ctx.stroke();
  ctx.restore();
}
