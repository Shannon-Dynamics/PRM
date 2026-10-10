/**
 * Canvas helpers shared by the Chapter 3 widgets. World coordinates in, the
 * book palette only — no literal colours, so every figure follows the theme
 * and responds to a hovered equation term.
 */

import { label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Point2 } from '@/lib/sim/world';
import type { Polygon } from '@/lib/bugs';

/** A lane offset: everything drawn in this lane is shifted by (ox, oy) in world units. */
export interface Frame {
  ox: number;
  oy: number;
}

export const X = (v: Viewport, f: Frame, x: number) => sx(v, x + f.ox);
export const Y = (v: Viewport, f: Frame, y: number) => sy(v, y + f.oy);

/** Workspace obstacles: slate fill, hairline edge. */
export function drawPolygons(ctx: CanvasRenderingContext2D, v: Viewport, f: Frame, polys: readonly Polygon[], p: Palette) {
  ctx.save();
  for (const poly of polys) {
    ctx.beginPath();
    poly.pts.forEach((q, i) => (i === 0 ? ctx.moveTo(X(v, f, q.x), Y(v, f, q.y)) : ctx.lineTo(X(v, f, q.x), Y(v, f, q.y))));
    ctx.closePath();
    ctx.globalAlpha = 0.78;
    ctx.fillStyle = p.obstacle;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = p.obstacle;
    ctx.lineWidth = 1.2;
    ctx.stroke();
  }
  ctx.restore();
}

/** A polyline in world coordinates. */
export function drawPolyline(
  ctx: CanvasRenderingContext2D,
  v: Viewport,
  f: Frame,
  pts: readonly Point2[],
  color: string,
  opts: { width?: number; alpha?: number; dashed?: boolean; close?: boolean } = {},
) {
  if (pts.length < 2) return;
  const { width = 1.8, alpha = 1, dashed = false, close = false } = opts;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  if (dashed) ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.moveTo(X(v, f, pts[0].x), Y(v, f, pts[0].y));
  for (let i = 1; i < pts.length; i++) ctx.lineTo(X(v, f, pts[i].x), Y(v, f, pts[i].y));
  if (close) ctx.closePath();
  ctx.stroke();
  ctx.restore();
}

/** Start (green ring) or goal (red disc). */
export function drawEndpoint(ctx: CanvasRenderingContext2D, v: Viewport, f: Frame, q: Point2, kind: 'start' | 'goal', p: Palette) {
  ctx.save();
  const px = X(v, f, q.x);
  const py = Y(v, f, q.y);
  if (kind === 'start') {
    ctx.strokeStyle = p.start;
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    ctx.arc(px, py, 5.5, 0, 2 * Math.PI);
    ctx.stroke();
  } else {
    ctx.fillStyle = p.goal;
    ctx.beginPath();
    ctx.arc(px, py, 6, 0, 2 * Math.PI);
    ctx.fill();
  }
  ctx.restore();
}

/** Hit and leave points, labelled H₁, L₁, … in the robot colour. */
export function drawMarks(
  ctx: CanvasRenderingContext2D,
  v: Viewport,
  f: Frame,
  pts: readonly Point2[],
  prefix: string,
  color: string,
  opts: { filled?: boolean; dy?: number } = {},
) {
  const { filled = true, dy = -9 } = opts;
  ctx.save();
  pts.forEach((q, i) => {
    const px = X(v, f, q.x);
    const py = Y(v, f, q.y);
    ctx.beginPath();
    ctx.arc(px, py, 3.2, 0, 2 * Math.PI);
    if (filled) {
      ctx.fillStyle = color;
      ctx.fill();
    } else {
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.6;
      ctx.stroke();
    }
    label(ctx, `${prefix}${i + 1}`, px + 5, py + dy, color, { size: 10, weight: 600 });
  });
  ctx.restore();
}

/** The point robot, with a soft halo so it reads over a thick path. */
export function drawPointRobot(ctx: CanvasRenderingContext2D, v: Viewport, f: Frame, q: Point2, p: Palette, pulse = 0) {
  ctx.save();
  const px = X(v, f, q.x);
  const py = Y(v, f, q.y);
  ctx.fillStyle = p.robot;
  ctx.globalAlpha = 0.25;
  ctx.beginPath();
  ctx.arc(px, py, 9 + 2 * pulse, 0, 2 * Math.PI);
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.beginPath();
  ctx.arc(px, py, 4.5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.restore();
}

/** A circle in world units (the sensing range, the band W*). */
export function drawCircle(
  ctx: CanvasRenderingContext2D,
  v: Viewport,
  f: Frame,
  c: Point2,
  r: number,
  color: string,
  opts: { alpha?: number; dashed?: boolean; width?: number } = {},
) {
  const { alpha = 0.5, dashed = true, width = 1 } = opts;
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  if (dashed) ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.arc(X(v, f, c.x), Y(v, f, c.y), sl(v, r), 0, 2 * Math.PI);
  ctx.stroke();
  ctx.restore();
}

/** An arrow from a to b in world coordinates. */
export function drawArrow(ctx: CanvasRenderingContext2D, v: Viewport, f: Frame, a: Point2, b: Point2, color: string, width = 1.6) {
  const ax = X(v, f, a.x);
  const ay = Y(v, f, a.y);
  const bx = X(v, f, b.x);
  const by = Y(v, f, b.y);
  const ang = Math.atan2(by - ay, bx - ax);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(ax, ay);
  ctx.lineTo(bx, by);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(bx, by);
  ctx.lineTo(bx - 8 * Math.cos(ang - 0.4), by - 8 * Math.sin(ang - 0.4));
  ctx.lineTo(bx - 8 * Math.cos(ang + 0.4), by - 8 * Math.sin(ang + 0.4));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}
