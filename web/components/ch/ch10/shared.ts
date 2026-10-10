/**
 * Drawing helpers shared by the Chapter 10 widgets: polygonal free space,
 * cells, critical points, coverage tints and the hatched "contaminated" fill.
 * No widget ids live here. Colours always arrive from the palette.
 */

import { sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Point2 } from '@/lib/sim/world';
import type { CellWorld } from '@/lib/cells/world';
import type { FreeRaster } from '@/lib/cells/slices';
import { parseRgb } from '@/components/ch/ch07/shared';

export function tracePoly(ctx: CanvasRenderingContext2D, v: Viewport, pts: readonly Point2[], close = true) {
  pts.forEach((p, i) => (i === 0 ? ctx.moveTo(sx(v, p.x), sy(v, p.y)) : ctx.lineTo(sx(v, p.x), sy(v, p.y))));
  if (close) ctx.closePath();
}

export function fillPoly(ctx: CanvasRenderingContext2D, v: Viewport, pts: readonly Point2[], color: string, alpha: number) {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  ctx.beginPath();
  tracePoly(ctx, v, pts);
  ctx.fill();
  ctx.restore();
}

export function strokePoly(ctx: CanvasRenderingContext2D, v: Viewport, pts: readonly Point2[], color: string, width = 1.5, close = true, dash?: number[]) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  if (dash) ctx.setLineDash(dash);
  ctx.beginPath();
  tracePoly(ctx, v, pts, close);
  ctx.stroke();
  ctx.restore();
}

/** The free space: outside of the boundary shaded, holes filled solid. */
export function drawCellWorld(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, w: CellWorld, opts: { names?: boolean } = {}) {
  ctx.save();
  // Shade everything outside the boundary (even–odd: viewport rectangle minus boundary).
  ctx.fillStyle = p.obstacle;
  ctx.globalAlpha = 0.18;
  ctx.beginPath();
  ctx.rect(0, 0, v.width, v.height);
  tracePoly(ctx, v, w.boundary);
  ctx.fill('evenodd');
  ctx.restore();
  strokePoly(ctx, v, w.boundary, p.obstacle, 2);
  w.obstacles.forEach((o, i) => {
    fillPoly(ctx, v, o, p.obstacle, 0.85);
    strokePoly(ctx, v, o, p.obstacle, 1.5);
    if (opts.names && w.obstacleNames?.[i]) {
      const c = o.reduce((s, q) => ({ x: s.x + q.x / o.length, y: s.y + q.y / o.length }), { x: 0, y: 0 });
      ctx.save();
      ctx.fillStyle = p.bg;
      ctx.font = '600 10px ui-monospace, monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(w.obstacleNames[i], sx(v, c.x), sy(v, c.y));
      ctx.restore();
    }
  });
}

export function marker(ctx: CanvasRenderingContext2D, x: number, y: number, r: number, fill: string, ring: string, shape: 'circle' | 'square' | 'diamond' | 'triangle' = 'circle') {
  ctx.save();
  ctx.fillStyle = fill;
  ctx.strokeStyle = ring;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  if (shape === 'circle') ctx.arc(x, y, r, 0, 2 * Math.PI);
  else if (shape === 'square') ctx.rect(x - r, y - r, 2 * r, 2 * r);
  else if (shape === 'diamond') {
    ctx.moveTo(x, y - r * 1.25);
    ctx.lineTo(x + r * 1.25, y);
    ctx.lineTo(x, y + r * 1.25);
    ctx.lineTo(x - r * 1.25, y);
    ctx.closePath();
  } else {
    ctx.moveTo(x, y - r * 1.3);
    ctx.lineTo(x + r * 1.2, y + r * 0.9);
    ctx.lineTo(x - r * 1.2, y + r * 0.9);
    ctx.closePath();
  }
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

export function segment(ctx: CanvasRenderingContext2D, v: Viewport, a: Point2, b: Point2, color: string, width = 1.5, dash?: number[], alpha = 1) {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  if (dash) ctx.setLineDash(dash);
  ctx.beginPath();
  ctx.moveTo(sx(v, a.x), sy(v, a.y));
  ctx.lineTo(sx(v, b.x), sy(v, b.y));
  ctx.stroke();
  ctx.restore();
}

export function polyline(ctx: CanvasRenderingContext2D, v: Viewport, pts: readonly Point2[], color: string, width = 1.5, alpha = 1) {
  if (pts.length < 2) return;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.beginPath();
  tracePoly(ctx, v, pts, false);
  ctx.stroke();
  ctx.restore();
}

/**
 * Bake a raster mask into an offscreen canvas: pixel k gets `color` at
 * `alpha(k)`. Row j of the raster is drawn upright (y up).
 */
export function rasterImage(r: FreeRaster, color: string, alpha: (k: number) => number): HTMLCanvasElement | null {
  if (typeof document === 'undefined') return null;
  const c = document.createElement('canvas');
  c.width = r.nx;
  c.height = r.ny;
  const ctx = c.getContext('2d');
  if (!ctx) return null;
  const img = ctx.createImageData(r.nx, r.ny);
  const [R, G, B] = parseRgb(color);
  for (let j = 0; j < r.ny; j++) {
    for (let i = 0; i < r.nx; i++) {
      const k = j * r.nx + i;
      const a = alpha(k);
      if (a <= 0) continue;
      const o = ((r.ny - 1 - j) * r.nx + i) * 4;
      img.data[o] = R;
      img.data[o + 1] = G;
      img.data[o + 2] = B;
      img.data[o + 3] = Math.round(255 * Math.min(1, a));
    }
  }
  ctx.putImageData(img, 0, 0);
  return c;
}

/** Draw a baked raster image into world coordinates. */
export function blitRaster(ctx: CanvasRenderingContext2D, v: Viewport, r: FreeRaster, img: HTMLCanvasElement | null) {
  if (!img) return;
  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(img, sx(v, r.x0), sy(v, r.y0 + r.ny * r.cell), sl(v, r.nx * r.cell), sl(v, r.ny * r.cell));
  ctx.restore();
}

/**
 * Hatched fill of raster pixels (the "contaminated / unsafe to assume clear"
 * convention, in the C-obstacle colour): a tint plus diagonal hatching clipped
 * to the pixel set.
 */
export function hatchPixels(ctx: CanvasRenderingContext2D, v: Viewport, r: FreeRaster, on: (k: number) => boolean, color: string) {
  const path = new Path2D();
  const w = sl(v, r.cell) + 0.6;
  let any = false;
  for (let j = 0; j < r.ny; j++) {
    for (let i = 0; i < r.nx; i++) {
      const k = j * r.nx + i;
      if (!on(k)) continue;
      any = true;
      path.rect(sx(v, r.x0 + i * r.cell), sy(v, r.y0 + (j + 1) * r.cell), w, w);
    }
  }
  if (!any) return;
  ctx.save();
  ctx.fillStyle = color;
  ctx.globalAlpha = 0.22;
  ctx.fill(path);
  ctx.globalAlpha = 0.75;
  ctx.clip(path);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let d = -v.height; d < v.width; d += 7) {
    ctx.moveTo(d, v.height);
    ctx.lineTo(d + v.height, 0);
  }
  ctx.stroke();
  ctx.restore();
}

/** Text with a background chip, for labels over busy figures. */
export function chip(ctx: CanvasRenderingContext2D, text: string, x: number, y: number, fg: string, bg: string, size = 10) {
  ctx.save();
  ctx.font = `600 ${size}px ui-monospace, monospace`;
  const w = ctx.measureText(text).width + 6;
  ctx.globalAlpha = 0.85;
  ctx.fillStyle = bg;
  ctx.fillRect(x - w / 2, y - size / 2 - 3, w, size + 6);
  ctx.globalAlpha = 1;
  ctx.fillStyle = fg;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, x, y);
  ctx.restore();
}
