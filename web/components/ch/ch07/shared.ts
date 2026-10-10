/**
 * Drawing helpers shared by the Chapter 7 widgets: scalar fields baked into an
 * offscreen canvas (a heightfield is thousands of cells — far too many to
 * fillRect every frame), and small arrow/marker utilities. No widget ids live
 * here. Colors always arrive from the palette; nothing is hard-coded.
 */

import { sl, sx, sy, type Viewport } from '@/lib/sim/draw';
import type { Bounds } from '@/lib/sim/world';

/** Parse `#rgb`, `#rrggbb`, `rgb()` / `rgba()` into components; falls back to mid grey. */
export function parseRgb(c: string): [number, number, number] {
  const s = c.trim();
  if (s.startsWith('#')) {
    const hex = s.slice(1);
    if (hex.length === 3) return [parseInt(hex[0] + hex[0], 16), parseInt(hex[1] + hex[1], 16), parseInt(hex[2] + hex[2], 16)];
    if (hex.length >= 6) return [parseInt(hex.slice(0, 2), 16), parseInt(hex.slice(2, 4), 16), parseInt(hex.slice(4, 6), 16)];
  }
  const m = s.match(/rgba?\(([^)]+)\)/);
  if (m) {
    const parts = m[1].split(/[\s,/]+/).filter(Boolean).map(Number);
    if (parts.length >= 3 && !parts.some(Number.isNaN)) return [parts[0], parts[1], parts[2]];
  }
  return [120, 120, 120];
}

/**
 * Bake an nx × ny scalar field (row-major, j upward) into a canvas whose
 * pixels are `color` at the alpha `alpha(value, cell)` returns (0 → clear).
 * Cell (i, j) lands at pixel (i, ny − 1 − j) so the image is drawn upright.
 */
export function fieldImage(
  values: ArrayLike<number>,
  nx: number,
  ny: number,
  color: string,
  alpha: (v: number, k: number) => number,
): HTMLCanvasElement | null {
  if (typeof document === 'undefined') return null;
  const c = document.createElement('canvas');
  c.width = nx;
  c.height = ny;
  const ctx = c.getContext('2d');
  if (!ctx) return null;
  const img = ctx.createImageData(nx, ny);
  const [r, g, b] = parseRgb(color);
  for (let j = 0; j < ny; j++) {
    for (let i = 0; i < nx; i++) {
      const k = j * nx + i;
      const a = Math.max(0, Math.min(1, alpha(values[k], k)));
      const o = ((ny - 1 - j) * nx + i) * 4;
      img.data[o] = r;
      img.data[o + 1] = g;
      img.data[o + 2] = b;
      img.data[o + 3] = Math.round(255 * a);
    }
  }
  ctx.putImageData(img, 0, 0);
  return c;
}

/** Stretch a baked field over a world rectangle. `smooth` interpolates (heightfields); off for grids. */
export function drawField(ctx: CanvasRenderingContext2D, v: Viewport, img: HTMLCanvasElement, b: Bounds, smooth = true) {
  ctx.save();
  ctx.imageSmoothingEnabled = smooth;
  ctx.drawImage(img, sx(v, b.minX), sy(v, b.maxY), sl(v, b.maxX - b.minX), sl(v, b.maxY - b.minY));
  ctx.restore();
}

/** An arrow in world coordinates with a pixel-sized head. */
export function arrow(ctx: CanvasRenderingContext2D, v: Viewport, x: number, y: number, dx: number, dy: number, color: string, lineWidth = 1.6, head = 6) {
  const x0 = sx(v, x);
  const y0 = sy(v, y);
  const x1 = sx(v, x + dx);
  const y1 = sy(v, y + dy);
  const L = Math.hypot(x1 - x0, y1 - y0);
  if (L < 0.5) return;
  const ang = Math.atan2(y1 - y0, x1 - x0);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();
  const h = Math.min(head, L);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - h * Math.cos(ang - 0.45), y1 - h * Math.sin(ang - 0.45));
  ctx.lineTo(x1 - h * Math.cos(ang + 0.45), y1 - h * Math.sin(ang + 0.45));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

/** A filled disc marker with an optional ring, in pixels. */
export function marker(ctx: CanvasRenderingContext2D, px: number, py: number, r: number, fill: string, ring?: string, ringWidth = 2) {
  ctx.save();
  ctx.fillStyle = fill;
  ctx.beginPath();
  ctx.arc(px, py, r, 0, Math.PI * 2);
  ctx.fill();
  if (ring) {
    ctx.strokeStyle = ring;
    ctx.lineWidth = ringWidth;
    ctx.stroke();
  }
  ctx.restore();
}

/** A hollow diamond — the saddle glyph. */
export function diamond(ctx: CanvasRenderingContext2D, px: number, py: number, r: number, color: string) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(px, py - r);
  ctx.lineTo(px + r, py);
  ctx.lineTo(px, py + r);
  ctx.lineTo(px - r, py);
  ctx.closePath();
  ctx.stroke();
  ctx.restore();
}

export const fmt = (x: number, d = 2) => (Math.abs(x) < 5e-13 ? (0).toFixed(d) : x.toFixed(d));

/** A small two-line readout cell for the strip under a canvas. */
export const readoutClass = 'px-3 py-2';
