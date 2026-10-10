/**
 * Drawing helpers shared by the Chapter 9 widgets: a tiny orthographic 3-D
 * projection for the silhouette sweep, polylines in world coordinates, and
 * the explorer's graph. No widget ids live here; colours arrive from the
 * palette.
 */

import { sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Vec } from '@/lib/roadmap/gvg';
import type { GvgExplorer } from '@/lib/roadmap/explore';
import { marker } from '@/components/ch/ch07/shared';

/** A 2-D polyline of world points. */
export function polyline(ctx: CanvasRenderingContext2D, v: Viewport, pts: readonly { x: number; y: number }[] | readonly Vec[], color: string, width = 1.6, alpha = 1, dashed = false) {
  if (pts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.globalAlpha = alpha;
  ctx.lineJoin = 'round';
  if (dashed) ctx.setLineDash([5, 4]);
  ctx.beginPath();
  pts.forEach((p, i) => {
    const x = Array.isArray(p) ? p[0] : p.x;
    const y = Array.isArray(p) ? p[1] : p.y;
    if (i === 0) ctx.moveTo(sx(v, x), sy(v, y));
    else ctx.lineTo(sx(v, x), sy(v, y));
  });
  ctx.stroke();
  ctx.restore();
}

/** The explorer's graph: finished edges, the edge under construction, nodes with their unexplored-slot counts. */
export function drawExplorerGraph(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, ex: GvgExplorer, opts: { labels?: boolean } = {}) {
  for (const e of ex.edges) polyline(ctx, v, e.polyline, p.graph, e.to >= 0 ? 2.2 : 1.4, e.to >= 0 ? 0.95 : 0.6);
  for (const n of ex.nodes) {
    const px = sx(v, n.q[0]);
    const py = sy(v, n.q[1]);
    if (n.kind === 'meet') {
      const open = n.explored.filter((e) => !e).length;
      marker(ctx, px, py, 5, open > 0 ? p.bg : p.graph, p.graph, 2);
      if (opts.labels !== false && open > 0) {
        ctx.save();
        ctx.fillStyle = p.graph;
        ctx.font = '600 9px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(String(open), px, py + 3);
        ctx.restore();
      }
    } else if (n.kind === 'boundary') marker(ctx, px, py, 2.5, p.obstacle);
    else marker(ctx, px, py, 3.5, p.bg, p.graph, 1.5);
  }
}

/* -------------------------------------------------------------------------- */
/* Orthographic 3-D                                                            */
/* -------------------------------------------------------------------------- */

export interface Camera {
  /** Yaw about the vertical axis and pitch above the horizontal plane, radians. */
  yaw: number;
  pitch: number;
  /** World units → pixels. */
  scale: number;
  /** Pixel centre. */
  cx: number;
  cy: number;
}

/** Project q = (q₁, q₂, q₃) with q₃ up: rotate by yaw about q₃, tilt by pitch, drop depth. Returns pixels and depth. */
export function project(cam: Camera, q: Vec): { x: number; y: number; z: number } {
  const cy = Math.cos(cam.yaw);
  const sy_ = Math.sin(cam.yaw);
  const x1 = cy * q[0] - sy_ * q[1];
  const y1 = sy_ * q[0] + cy * q[1];
  const cp = Math.cos(cam.pitch);
  const sp = Math.sin(cam.pitch);
  // Screen x = x1; screen up = q₃ cos p − y1 sin p; depth = y1 cos p + q₃ sin p.
  const up = q[2] * cp - y1 * sp;
  const depth = y1 * cp + q[2] * sp;
  return { x: cam.cx + cam.scale * x1, y: cam.cy - cam.scale * up, z: depth };
}

export function polyline3(ctx: CanvasRenderingContext2D, cam: Camera, pts: readonly Vec[], color: string, width = 1.4, alpha = 1, dashed = false) {
  if (pts.length < 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.globalAlpha = alpha;
  ctx.lineJoin = 'round';
  if (dashed) ctx.setLineDash([4, 4]);
  ctx.beginPath();
  pts.forEach((q, i) => {
    const s = project(cam, q);
    if (i === 0) ctx.moveTo(s.x, s.y);
    else ctx.lineTo(s.x, s.y);
  });
  ctx.stroke();
  ctx.restore();
}

/** A translucent quadrilateral in 3-D (the sweep plane). */
export function quad3(ctx: CanvasRenderingContext2D, cam: Camera, corners: readonly Vec[], fill: string, stroke: string, alpha = 0.12) {
  ctx.save();
  ctx.beginPath();
  corners.forEach((q, i) => {
    const s = project(cam, q);
    if (i === 0) ctx.moveTo(s.x, s.y);
    else ctx.lineTo(s.x, s.y);
  });
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.globalAlpha = alpha;
  ctx.fill();
  ctx.globalAlpha = Math.min(1, alpha * 5);
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 1.2;
  ctx.stroke();
  ctx.restore();
}

export function point3(ctx: CanvasRenderingContext2D, cam: Camera, q: Vec, r: number, fill: string, ring?: string) {
  const s = project(cam, q);
  marker(ctx, s.x, s.y, r, fill, ring);
}
