/**
 * Canvas painters shared by the Part III widgets. No React here — these are
 * the `lib/sim/draw.ts` primitives extended for what sampling planners draw:
 * filled polygon obstacles, Reach's arm, a torus C-obstacle raster on the
 * unwrapped square, tree edges (through the seam on T²), and the informed
 * ellipse. Colors always arrive from the palette the canvas hands to `draw`.
 */

import type { Palette, Viewport } from '../sim/draw';
import { sl, sx, sy } from '../sim/draw';
import type { Polygon } from '../collide/polygon';
import type { Reach } from '../robots/reach';
import type { PolyWorld } from '../robots/worlds';
import type { World } from '../sim/world';
import type { BitGrid } from '../manifold/torus';
import type { Chart } from '../manifold/manifold';
import { chartBox2 } from './tree';

/* -------------------------------------------------------------------------- */
/* A workspace drawn into a sub-rectangle of the canvas                         */
/* -------------------------------------------------------------------------- */

/** World → pixel for a workspace placed inside a rectangle of the canvas (uniform scale, centered). */
export interface Mapper {
  X: (x: number) => number;
  Y: (y: number) => number;
  /** Scalar length → pixels. */
  S: (d: number) => number;
}

/** Fit `bounds` into the viewport rectangle `rect` (given in the canvas's own world coordinates). */
export function rectMapper(v: Viewport, rect: { x0: number; y0: number; x1: number; y1: number }, bounds: { minX: number; minY: number; maxX: number; maxY: number }): Mapper {
  const px0 = sx(v, rect.x0);
  const py0 = sy(v, rect.y1);
  const pw = sx(v, rect.x1) - px0;
  const ph = sy(v, rect.y0) - py0;
  const bw = bounds.maxX - bounds.minX;
  const bh = bounds.maxY - bounds.minY;
  const s = Math.min(pw / bw, ph / bh);
  const ox = px0 + (pw - s * bw) / 2;
  const oy = py0 + (ph - s * bh) / 2;
  return {
    X: (x) => ox + (x - bounds.minX) * s,
    Y: (y) => oy + ph - (ph - s * bh) - (y - bounds.minY) * s,
    S: (d) => d * s,
  };
}

/** Walls as strokes and (for a PolyWorld) obstacles as filled slate polygons, through a mapper. */
export function paintPolyWorld(ctx: CanvasRenderingContext2D, m: Mapper, world: World | PolyWorld, p: Palette): void {
  ctx.save();
  ctx.strokeStyle = p.wall;
  ctx.lineWidth = 2;
  ctx.lineCap = 'round';
  ctx.beginPath();
  for (const w of world.walls) {
    ctx.moveTo(m.X(w.x1), m.Y(w.y1));
    ctx.lineTo(m.X(w.x2), m.Y(w.y2));
  }
  ctx.stroke();
  const obs = (world as PolyWorld).obstacles;
  if (obs) {
    ctx.fillStyle = p.obstacle;
    for (const o of obs) {
      ctx.beginPath();
      o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(m.X(pt.x), m.Y(pt.y)) : ctx.lineTo(m.X(pt.x), m.Y(pt.y))));
      ctx.closePath();
      ctx.fill();
    }
  }
  ctx.restore();
}

/** Reach's chain through a mapper (the side panel of every torus widget). */
export function paintArmMapped(ctx: CanvasRenderingContext2D, m: Mapper, p: Palette, arm: Reach, q: readonly number[], color: string, alpha = 1): void {
  const pts = arm.fk(q);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = Math.max(m.S(2 * arm.linkRadius), 4);
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  pts.forEach((pt, i) => (i === 0 ? ctx.moveTo(m.X(pt.x), m.Y(pt.y)) : ctx.lineTo(m.X(pt.x), m.Y(pt.y))));
  ctx.stroke();
  ctx.fillStyle = p.bg;
  ctx.lineWidth = 1.5;
  for (const pt of pts) {
    ctx.beginPath();
    ctx.arc(m.X(pt.x), m.Y(pt.y), 3, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

export function fillPolygons(ctx: CanvasRenderingContext2D, v: Viewport, polys: readonly Polygon[], color: string, alpha = 1): void {
  ctx.save();
  ctx.fillStyle = color;
  ctx.globalAlpha = alpha;
  for (const poly of polys) {
    ctx.beginPath();
    poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
}

/** Reach as a stroked chain with joint dots. */
export function paintArm(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, arm: Reach, q: readonly number[], color = p.robot, alpha = 1): void {
  const pts = arm.fk(q);
  const w = Math.max(sl(v, 2 * arm.linkRadius), 4);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = w;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  pts.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
  ctx.stroke();
  ctx.fillStyle = p.bg;
  ctx.lineWidth = 1.5;
  for (const pt of pts) {
    ctx.beginPath();
    ctx.arc(sx(v, pt.x), sy(v, pt.y), 3.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

/**
 * A square panel mapping a 2-D chart box onto a canvas rectangle, with the
 * seam drawn dashed when the axes are periodic. Returns the mapping functions
 * the caller uses to place nodes.
 */
export interface ChartPanel {
  X: (c: number) => number;
  Y: (c: number) => number;
  lo: [number, number];
  hi: [number, number];
  px: number;
  py: number;
  w: number;
  h: number;
}

export function chartPanel<P>(v: Viewport, space: Chart<P>, rect: { x0: number; y0: number; x1: number; y1: number }): ChartPanel {
  const [lo, hi] = chartBox2(space);
  const px = sx(v, rect.x0);
  const py = sy(v, rect.y1);
  const w = sx(v, rect.x1) - px;
  const h = sy(v, rect.y0) - py;
  return {
    X: (c) => px + ((c - lo[0]) / (hi[0] - lo[0])) * w,
    Y: (c) => py + h - ((c - lo[1]) / (hi[1] - lo[1])) * h,
    lo,
    hi,
    px,
    py,
    w,
    h,
  };
}

/** Amber C-obstacle raster on a chart panel (1 = occupied; row-major j·n1 + i). */
export function paintRaster(ctx: CanvasRenderingContext2D, panel: ChartPanel, grid: BitGrid, color: string, alpha = 0.8): void {
  const cw = panel.w / grid.n1;
  const ch = panel.h / grid.n2;
  ctx.save();
  ctx.fillStyle = color;
  ctx.globalAlpha = alpha;
  for (let j = 0; j < grid.n2; j++) {
    for (let i = 0; i < grid.n1; i++) {
      if (!grid.occupied[j * grid.n1 + i]) continue;
      ctx.fillRect(panel.px + i * cw, panel.py + panel.h - (j + 1) * ch, cw + 0.6, ch + 0.6);
    }
  }
  ctx.restore();
}

export function paintPanelFrame(ctx: CanvasRenderingContext2D, panel: ChartPanel, p: Palette, periodic: boolean): void {
  ctx.save();
  ctx.strokeStyle = periodic ? p.accent : p.wall;
  ctx.lineWidth = periodic ? 1.5 : 2;
  if (periodic) ctx.setLineDash([6, 5]);
  ctx.strokeRect(panel.px, panel.py, panel.w, panel.h);
  ctx.restore();
}

/**
 * A segment between two configurations on a chart panel, split where the
 * short-way geodesic crosses a seam. `n` interpolation points per segment.
 */
export function paintGeodesic<P>(
  ctx: CanvasRenderingContext2D,
  panel: ChartPanel,
  space: Chart<P>,
  a: P,
  b: P,
  n = 8,
): void {
  const span0 = panel.hi[0] - panel.lo[0];
  const span1 = panel.hi[1] - panel.lo[1];
  let prev: number[] | null = null;
  let pen = false;
  for (let k = 0; k <= n; k++) {
    const c = space.coords(space.interpolate(a, b, k / n));
    if (prev && (Math.abs(c[0] - prev[0]) > span0 / 2 || Math.abs(c[1] - prev[1]) > span1 / 2)) pen = false;
    if (!pen) {
      ctx.moveTo(panel.X(c[0]), panel.Y(c[1]));
      pen = true;
    } else ctx.lineTo(panel.X(c[0]), panel.Y(c[1]));
    prev = c;
  }
}

/** Tree edges on a chart panel, one path. */
export function paintTreeEdges<P>(
  ctx: CanvasRenderingContext2D,
  panel: ChartPanel,
  space: Chart<P>,
  nodes: readonly { q: P; parent: number }[],
  color: string,
  opts: { lineWidth?: number; alpha?: number; from?: number; to?: number } = {},
): void {
  const { lineWidth = 1, alpha = 0.7, from = 1, to = nodes.length } = opts;
  const periodic = space.periodic.some(Boolean);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.globalAlpha = alpha;
  ctx.beginPath();
  for (let i = Math.max(1, from); i < Math.min(to, nodes.length); i++) {
    const n = nodes[i];
    if (n.parent < 0) continue;
    const q = nodes[n.parent].q;
    if (periodic) paintGeodesic(ctx, panel, space, q, n.q, 6);
    else {
      const a = space.coords(q);
      const b = space.coords(n.q);
      ctx.moveTo(panel.X(a[0]), panel.Y(a[1]));
      ctx.lineTo(panel.X(b[0]), panel.Y(b[1]));
    }
  }
  ctx.stroke();
  ctx.restore();
}

/** A polyline of configurations on a chart panel (through seams). */
export function paintChartPath<P>(ctx: CanvasRenderingContext2D, panel: ChartPanel, space: Chart<P>, path: readonly P[], color: string, lineWidth = 2.5, alpha = 1, dashed = false): void {
  if (path.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.globalAlpha = alpha;
  ctx.lineJoin = 'round';
  if (dashed) ctx.setLineDash([5, 4]);
  ctx.beginPath();
  for (let i = 1; i < path.length; i++) paintGeodesic(ctx, panel, space, path[i - 1], path[i], 8);
  ctx.stroke();
  ctx.restore();
}

export function paintDot(ctx: CanvasRenderingContext2D, x: number, y: number, r: number, color: string, alpha = 1): void {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

/** The informed ellipse with foci a, b and transverse diameter cBest, in world coordinates. */
export function paintEllipse(ctx: CanvasRenderingContext2D, v: Viewport, a: readonly number[], b: readonly number[], cBest: number, color: string, alpha = 1): void {
  const cMin = Math.hypot(b[0] - a[0], b[1] - a[1]);
  if (!Number.isFinite(cBest) || cBest < cMin) return;
  const cx = (a[0] + b[0]) / 2;
  const cy = (a[1] + b[1]) / 2;
  const rx = sl(v, cBest / 2);
  const ry = sl(v, Math.sqrt(Math.max(0, cBest * cBest - cMin * cMin)) / 2);
  const angle = Math.atan2(b[1] - a[1], b[0] - a[0]);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.translate(sx(v, cx), sy(v, cy));
  ctx.rotate(-angle);
  ctx.beginPath();
  ctx.ellipse(0, 0, Math.max(rx, 0.5), Math.max(ry, 0.5), 0, 0, Math.PI * 2);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 4]);
  ctx.stroke();
  ctx.globalAlpha = alpha * 0.08;
  ctx.fillStyle = color;
  ctx.fill();
  ctx.restore();
}

/** Blend two palette colors by alpha stacking: returns a CSS color-mix() string (modern browsers). */
export function tint(color: string, pct: number, toward: string): string {
  return `color-mix(in srgb, ${color} ${Math.round(100 - pct)}%, ${toward} ${Math.round(pct)}%)`;
}
