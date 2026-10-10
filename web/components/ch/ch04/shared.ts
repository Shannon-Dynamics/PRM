/**
 * Drawing helpers shared by the Chapter 4 widgets: hatched amber fills (the
 * C-obstacle role is *hatched* amber by the book's color code), a cached
 * raster image for the pixel-method pictures, torus axes, arrows, and Reach
 * as capsule links. No widget ids live here.
 */

import { label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Polygon } from '@/lib/collide/polygon';
import type { Reach, Tn } from '@/lib/robots/reach';
import type { BitGrid, Chart2 } from '@/lib/cspace/raster';

export function pathPoly(ctx: CanvasRenderingContext2D, v: Viewport, poly: Polygon) {
  ctx.beginPath();
  poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
  ctx.closePath();
}

export function fillPoly(ctx: CanvasRenderingContext2D, v: Viewport, poly: Polygon, color: string, alpha = 1) {
  if (poly.length < 3) return;
  ctx.save();
  ctx.fillStyle = color;
  ctx.globalAlpha = alpha;
  pathPoly(ctx, v, poly);
  ctx.fill();
  ctx.restore();
}

export function strokePoly(ctx: CanvasRenderingContext2D, v: Viewport, poly: Polygon, color: string, lineWidth = 1.5, dashed = false, alpha = 1) {
  if (poly.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.globalAlpha = alpha;
  ctx.lineJoin = 'round';
  if (dashed) ctx.setLineDash([5, 4]);
  pathPoly(ctx, v, poly);
  ctx.stroke();
  ctx.restore();
}

/** Diagonal hatching clipped to a path already set on the context (pixel spacing). */
function hatchClipped(ctx: CanvasRenderingContext2D, w: number, h: number, color: string, spacing = 7, lineWidth = 1) {
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.beginPath();
  for (let d = -h; d < w + h; d += spacing) {
    ctx.moveTo(d, 0);
    ctx.lineTo(d + h, h);
  }
  ctx.stroke();
}

/** The C-obstacle fill: translucent amber with a diagonal hatch and a crisp outline. */
export function hatchFillPoly(ctx: CanvasRenderingContext2D, v: Viewport, poly: Polygon, color: string, opts: { alpha?: number; outline?: boolean } = {}) {
  if (poly.length < 3) return;
  const { alpha = 0.26, outline = true } = opts;
  ctx.save();
  pathPoly(ctx, v, poly);
  ctx.fillStyle = color;
  ctx.globalAlpha = alpha;
  ctx.fill();
  ctx.globalAlpha = 0.75;
  ctx.clip();
  hatchClipped(ctx, v.width, v.height, color);
  ctx.restore();
  if (outline) strokePoly(ctx, v, poly, color, 1.4);
}

/**
 * A raster rendered once into an offscreen canvas: amber where occupied, with
 * hatch stripes punched out so the picture reads as the hatched C-obstacle
 * role at any zoom. Axis 1 of the chart runs horizontally, axis 2 vertically
 * (upward). Occupied runs are filled as single rectangles.
 */
export function rasterImage(grid: BitGrid, color: string, pxPerCell?: number): HTMLCanvasElement | null {
  if (typeof document === 'undefined') return null;
  const k = pxPerCell ?? Math.max(1, Math.ceil(720 / Math.max(grid.n1, grid.n2)));
  const c = document.createElement('canvas');
  c.width = grid.n1 * k;
  c.height = grid.n2 * k;
  const g = c.getContext('2d');
  if (!g) return null;
  g.fillStyle = color;
  g.globalAlpha = 0.82;
  for (let i = 0; i < grid.n1; i++) {
    let j = 0;
    while (j < grid.n2) {
      if (!grid.get(i, j)) {
        j++;
        continue;
      }
      let j2 = j;
      while (j2 < grid.n2 && grid.get(i, j2)) j2++;
      // Column i, rows j..j2−1, with j increasing upward on screen.
      g.fillRect(i * k, (grid.n2 - j2) * k, k, (j2 - j) * k);
      j = j2;
    }
  }
  // Hatch: punch translucent stripes out of the amber.
  g.globalAlpha = 1;
  g.globalCompositeOperation = 'destination-out';
  g.strokeStyle = 'rgba(0,0,0,0.55)';
  g.lineWidth = Math.max(1, k * 0.9);
  const spacing = Math.max(6, k * 4);
  g.beginPath();
  for (let d = -c.height; d < c.width + c.height; d += spacing) {
    g.moveTo(d, 0);
    g.lineTo(d + c.height, c.height);
  }
  g.stroke();
  return c;
}

/** Blit a raster image over the chart's extent in world coordinates. */
export function drawRaster<Q>(ctx: CanvasRenderingContext2D, v: Viewport, img: HTMLCanvasElement, chart: Chart2<Q>, shift: [number, number] = [0, 0]) {
  const x0 = chart.min1 - chart.d1 / 2 + shift[0];
  const y0 = chart.min2 - chart.d2 / 2 + shift[1];
  const w = chart.n1 * chart.d1;
  const h = chart.n2 * chart.d2;
  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(img, sx(v, x0), sy(v, y0 + h), sl(v, w), sl(v, h));
  ctx.restore();
}

/** The unwrapped torus: the [−π, π]² square, ticks at quarter turns, matched seam marks. */
export function drawTorusAxes(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, names: [string, string] = ['θ₁', 'θ₂']) {
  const L = Math.PI;
  ctx.save();
  ctx.strokeStyle = p.ink;
  ctx.lineWidth = 1.2;
  ctx.strokeRect(sx(v, -L), sy(v, L), sl(v, 2 * L), sl(v, 2 * L));
  ctx.strokeStyle = p.grid;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (const t of [-L / 2, 0, L / 2]) {
    ctx.moveTo(sx(v, t), sy(v, -L));
    ctx.lineTo(sx(v, t), sy(v, L));
    ctx.moveTo(sx(v, -L), sy(v, t));
    ctx.lineTo(sx(v, L), sy(v, t));
  }
  ctx.stroke();
  // Seam marks: the left and right edges are the same circle, so are top and bottom.
  ctx.strokeStyle = p.ink;
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (const s of [-1, 1]) {
    const x = sx(v, s * L);
    ctx.moveTo(x - 5, sy(v, 0) - 5);
    ctx.lineTo(x + 5, sy(v, 0) + 5);
    const y = sy(v, s * L);
    ctx.moveTo(sx(v, 0) - 5, y - 5);
    ctx.lineTo(sx(v, 0) + 5, y + 5);
    ctx.moveTo(sx(v, 0) - 5, y + 5);
    ctx.lineTo(sx(v, 0) + 5, y - 5);
  }
  ctx.stroke();
  ctx.restore();
  const deg = ['−180°', '−90°', '0°', '90°', '180°'];
  [-L, -L / 2, 0, L / 2, L].forEach((t, k) => {
    label(ctx, deg[k], sx(v, t), sy(v, -L) + 10, p.ink, { size: 9, align: 'center' });
    label(ctx, deg[k], sx(v, -L) - 4, sy(v, t), p.ink, { size: 9, align: 'right' });
  });
  label(ctx, names[0], sx(v, L) + 6, sy(v, -L) + 10, p.ink, { size: 11, weight: 700 });
  label(ctx, names[1], sx(v, -L) - 4, sy(v, L) - 12, p.ink, { size: 11, weight: 700, align: 'right' });
}

/** A pixel-space arrow with a filled head. */
export function arrowPx(ctx: CanvasRenderingContext2D, x0: number, y0: number, x1: number, y1: number, color: string, lineWidth = 2, head = 7) {
  const dx = x1 - x0;
  const dy = y1 - y0;
  const len = Math.hypot(dx, dy);
  if (len < 1e-6) return;
  const ux = dx / len;
  const uy = dy / len;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1 - ux * head * 0.6, y1 - uy * head * 0.6);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - ux * head - uy * head * 0.5, y1 - uy * head + ux * head * 0.5);
  ctx.lineTo(x1 - ux * head + uy * head * 0.5, y1 - uy * head - ux * head * 0.5);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

/** A world-space arrow from a point along a vector. */
export function arrowWorld(ctx: CanvasRenderingContext2D, v: Viewport, x: number, y: number, dx: number, dy: number, color: string, lineWidth = 2) {
  arrowPx(ctx, sx(v, x), sy(v, y), sx(v, x + dx), sy(v, y + dy), color, lineWidth);
}

/** Reach as capsule links with joint dots. `color` defaults to the robot role. */
export function drawReach(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, arm: Reach, q: Tn, color: string = p.robot, alpha = 1) {
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
  ctx.lineWidth = 2;
  for (let i = 0; i < pts.length; i++) {
    ctx.beginPath();
    ctx.arc(sx(v, pts[i].x), sy(v, pts[i].y), i === 0 ? 5 : 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

export function dot(ctx: CanvasRenderingContext2D, x: number, y: number, r: number, color: string, ring?: string) {
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();
  if (ring) {
    ctx.strokeStyle = ring;
    ctx.lineWidth = 2;
    ctx.stroke();
  }
  ctx.restore();
}

export const fmt = (x: number, d = 3) => (Math.abs(x) < 5e-13 ? (0).toFixed(d) : x.toFixed(d));
export const deg = (rad: number) => (rad * 180) / Math.PI;
