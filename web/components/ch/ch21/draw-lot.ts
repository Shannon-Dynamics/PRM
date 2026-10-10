import type { Palette, Viewport } from '@/lib/sim/draw';
import { drawSegments, drawWorld, sl, sx, sy } from '@/lib/sim/draw';
import type { PolyWorld } from '@/lib/robots/worlds';
import type { Scene } from '@/lib/collide/scene';
import type { Pose2 } from '@/lib/geom/se2';
import type { CarSample } from '@/lib/nonholonomic';

export { drawArrow, drawDot, drawHitch, drawPolyline } from '@/components/ch/ch20/draw-car';

/** The Lot (or any PolyWorld): bay paint, curbs, filled pillars. */
export function drawLot(ctx: CanvasRenderingContext2D, v: Viewport, world: PolyWorld, p: Palette, extra: Scene | null = null) {
  if (world.markings) drawSegments(ctx, v, world.markings, p.grid, 2);
  ctx.save();
  ctx.fillStyle = p.obstacle;
  const polys = extra ? extra.obstacles.map((o) => o.poly) : world.obstacles.map((o) => o.poly);
  for (const poly of polys) {
    ctx.beginPath();
    poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
  drawWorld(ctx, v, world, p);
}

/** A dense car path; reversing stretches drawn with a different dash so cusps read at a glance. */
export function drawCarPath(ctx: CanvasRenderingContext2D, v: Viewport, samples: readonly CarSample[], color: string, opts: { lineWidth?: number; dash?: number[]; alpha?: number } = {}) {
  if (samples.length < 2) return;
  const { lineWidth = 2, dash = [], alpha = 1 } = opts;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.lineJoin = 'round';
  ctx.setLineDash(dash);
  ctx.beginPath();
  samples.forEach((s, i) => (i === 0 ? ctx.moveTo(sx(v, s.q.x), sy(v, s.q.y)) : ctx.lineTo(sx(v, s.q.x), sy(v, s.q.y))));
  ctx.stroke();
  ctx.setLineDash([]);
  // Cusps as small open circles.
  for (let i = 2; i < samples.length; i++) {
    const a = samples[i - 1].ds;
    const b = samples[i].ds;
    if (Math.abs(a) > 1e-9 && Math.abs(b) > 1e-9 && Math.sign(a) !== Math.sign(b)) {
      ctx.beginPath();
      ctx.arc(sx(v, samples[i - 1].q.x), sy(v, samples[i - 1].q.y), 4, 0, 2 * Math.PI);
      ctx.stroke();
    }
  }
  ctx.restore();
}

/** A pose marker: a short heading arrow from the rear axle. */
export function drawPoseMarker(ctx: CanvasRenderingContext2D, v: Viewport, q: Pose2, color: string, len = 1.2) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  const x0 = sx(v, q.x);
  const y0 = sy(v, q.y);
  const x1 = sx(v, q.x + len * Math.cos(q.theta));
  const y1 = sy(v, q.y + len * Math.sin(q.theta));
  ctx.beginPath();
  ctx.arc(x0, y0, Math.max(3, sl(v, 0.12)), 0, 2 * Math.PI);
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();
  const ang = Math.atan2(y1 - y0, x1 - x0);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - 7 * Math.cos(ang - 0.4), y1 - 7 * Math.sin(ang - 0.4));
  ctx.lineTo(x1 - 7 * Math.cos(ang + 0.4), y1 - 7 * Math.sin(ang + 0.4));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

/** Signed hitch-angle gauge: a half dial from −π/2 to π/2 whose needle reddens toward jackknife. */
export function drawHitchGauge(ctx: CanvasRenderingContext2D, cx: number, cy: number, r: number, angle: number, p: Palette) {
  ctx.save();
  ctx.lineWidth = 6;
  ctx.strokeStyle = p.grid;
  ctx.beginPath();
  ctx.arc(cx, cy, r, Math.PI, 2 * Math.PI);
  ctx.stroke();
  const frac = Math.min(1, Math.abs(angle) / (Math.PI / 2));
  ctx.strokeStyle = frac > 0.66 ? p.goal : p.robot;
  ctx.beginPath();
  const a0 = 1.5 * Math.PI;
  const a1 = a0 - Math.max(-Math.PI / 2, Math.min(Math.PI / 2, angle));
  ctx.arc(cx, cy, r, Math.min(a0, a1), Math.max(a0, a1));
  ctx.stroke();
  ctx.lineWidth = 2;
  ctx.strokeStyle = frac > 0.66 ? p.goal : p.ink;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + r * Math.cos(a1), cy + r * Math.sin(a1));
  ctx.stroke();
  ctx.restore();
}

/** Inset frame in pixel space. */
export function insetBox(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, p: Palette) {
  ctx.save();
  ctx.fillStyle = p.bg;
  ctx.globalAlpha = 0.88;
  ctx.fillRect(x, y, w, h);
  ctx.globalAlpha = 1;
  ctx.strokeStyle = p.grid;
  ctx.strokeRect(x, y, w, h);
  ctx.restore();
}
