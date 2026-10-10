import type { Palette, Viewport } from '@/lib/sim/draw';
import { sl, sx, sy } from '@/lib/sim/draw';
import type { Hitch, HitchCfg } from '@/lib/robots/hitch';

/** Hitch's chassis (and trailer) as outlined rectangles, plus a heading tick — the one car drawing both chapters share. */
export function drawHitch(ctx: CanvasRenderingContext2D, v: Viewport, car: Hitch, q: HitchCfg, color: string, opts: { alpha?: number; fill?: boolean; lineWidth?: number } = {}) {
  const { alpha = 1, fill = false, lineWidth = 1.6 } = opts;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineJoin = 'round';
  for (const poly of car.footprint(q)) {
    ctx.beginPath();
    poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
    ctx.closePath();
    if (fill) {
      ctx.globalAlpha = alpha * 0.18;
      ctx.fill();
      ctx.globalAlpha = alpha;
    }
    ctx.stroke();
  }
  // Rear axle dot and heading tick.
  ctx.beginPath();
  ctx.arc(sx(v, q.body.x), sy(v, q.body.y), 3, 0, 2 * Math.PI);
  ctx.fill();
  const f = car.frontAxle(q);
  ctx.beginPath();
  ctx.moveTo(sx(v, q.body.x), sy(v, q.body.y));
  ctx.lineTo(sx(v, f.x), sy(v, f.y));
  ctx.stroke();
  if (q.trailer !== null) {
    const a = car.trailerAxle(q);
    if (a) {
      ctx.beginPath();
      ctx.moveTo(sx(v, q.body.x), sy(v, q.body.y));
      ctx.lineTo(sx(v, a.x), sy(v, a.y));
      ctx.stroke();
    }
  }
  ctx.restore();
}

/** An arrow from (x0, y0) to (x1, y1) in world units. */
export function drawArrow(ctx: CanvasRenderingContext2D, v: Viewport, x0: number, y0: number, x1: number, y1: number, color: string, opts: { lineWidth?: number; dashed?: boolean; head?: number } = {}) {
  const { lineWidth = 2, dashed = false, head = 7 } = opts;
  const X0 = sx(v, x0);
  const Y0 = sy(v, y0);
  const X1 = sx(v, x1);
  const Y1 = sy(v, y1);
  const len = Math.hypot(X1 - X0, Y1 - Y0);
  if (len < 1e-6) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = lineWidth;
  if (dashed) ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(X0, Y0);
  ctx.lineTo(X1, Y1);
  ctx.stroke();
  ctx.setLineDash([]);
  const ang = Math.atan2(Y1 - Y0, X1 - X0);
  ctx.beginPath();
  ctx.moveTo(X1, Y1);
  ctx.lineTo(X1 - head * Math.cos(ang - 0.4), Y1 - head * Math.sin(ang - 0.4));
  ctx.lineTo(X1 - head * Math.cos(ang + 0.4), Y1 - head * Math.sin(ang + 0.4));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

/** A polyline in world units. */
export function drawPolyline(ctx: CanvasRenderingContext2D, v: Viewport, pts: readonly { x: number; y: number }[], color: string, opts: { lineWidth?: number; dashed?: boolean; alpha?: number } = {}) {
  if (pts.length < 2) return;
  const { lineWidth = 1.5, dashed = false, alpha = 1 } = opts;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineJoin = 'round';
  if (dashed) ctx.setLineDash([4, 3]);
  ctx.beginPath();
  pts.forEach((p, i) => (i === 0 ? ctx.moveTo(sx(v, p.x), sy(v, p.y)) : ctx.lineTo(sx(v, p.x), sy(v, p.y))));
  ctx.stroke();
  ctx.restore();
}

/** A filled disc of world radius r. */
export function drawDot(ctx: CanvasRenderingContext2D, v: Viewport, x: number, y: number, r: number, color: string, alpha = 1) {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(sx(v, x), sy(v, y), Math.max(1, sl(v, r)), 0, 2 * Math.PI);
  ctx.fill();
  ctx.restore();
}
