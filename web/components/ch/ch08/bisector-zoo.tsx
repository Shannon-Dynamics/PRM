import { bisectorLineLine, bisectorPointLine, bisectorPointPoint } from '@/lib/roadmap/gvd';
import type { Point2 } from '@/lib/sim/world';

/**
 * f8.4 — the bisector zoo (Choset Fig. 5.14): the three kinds of GVD edge in a
 * polygonal world, computed by the module's analytic bisectors and drawn as
 * static SVG. Point–point and line–line bisectors are lines; a point and a
 * line give a parabola — focus the vertex, directrix the edge.
 */

const W = 220;
const H = 180;
const toPx = (p: Point2, scale: number, ox: number, oy: number) => `${(ox + p.x * scale).toFixed(1)},${(oy - p.y * scale).toFixed(1)}`;

function Panel({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="flex flex-col items-center gap-1">
      <svg viewBox={`0 0 ${W} ${H}`} width="100%" role="img" aria-label={title} className="max-w-[260px]">
        <rect x={0} y={0} width={W} height={H} fill="var(--pr-canvas-bg, transparent)" />
        {children}
      </svg>
      <span className="font-ui text-[0.72rem] text-fd-muted-foreground">{title}</span>
    </div>
  );
}

export function BisectorZoo() {
  const scale = 28;
  const ox = W / 2;
  const oy = H / 2;
  // Point–point.
  const a = { x: -2, y: -1 };
  const b = { x: 2, y: 1 };
  const pp = bisectorPointPoint(a, b, 3.2);
  // Line–line: two edges meeting at a corner.
  const e1 = { x1: -3, y1: -2, x2: 2.5, y2: -2 };
  const e2 = { x1: -3, y1: -2, x2: -1, y2: 2.6 };
  const ll = bisectorLineLine(e1, e2, 3.5);
  // Point–line: a vertex above an edge.
  const focus = { x: 0.3, y: 1.6 };
  const directrix = { x1: -3.5, y1: -1.6, x2: 3.5, y2: -1.6 };
  const pl = bisectorPointLine(focus, directrix, [-3.4, 3.4], 48);
  const poly = (pts: Point2[]) => pts.map((p) => toPx(p, scale, ox, oy)).join(' ');
  return (
    <div className="not-prose grid grid-cols-1 gap-3 sm:grid-cols-3">
      <Panel title="vertex | vertex — a line">
        <polyline points={poly(pp)} fill="none" stroke="var(--rm-graph)" strokeWidth={2.2} />
        <line x1={ox + a.x * scale} y1={oy - a.y * scale} x2={ox + b.x * scale} y2={oy - b.y * scale} stroke="var(--pr-grid, rgba(128,128,128,0.4))" strokeDasharray="3 3" />
        <circle cx={ox + a.x * scale} cy={oy - a.y * scale} r={5} fill="var(--rm-obstacle)" />
        <circle cx={ox + b.x * scale} cy={oy - b.y * scale} r={5} fill="var(--rm-obstacle)" />
      </Panel>
      <Panel title="edge | edge — a line (the angle bisector)">
        <line x1={ox + e1.x1 * scale} y1={oy - e1.y1 * scale} x2={ox + e1.x2 * scale} y2={oy - e1.y2 * scale} stroke="var(--rm-obstacle)" strokeWidth={4} strokeLinecap="round" />
        <line x1={ox + e2.x1 * scale} y1={oy - e2.y1 * scale} x2={ox + e2.x2 * scale} y2={oy - e2.y2 * scale} stroke="var(--rm-obstacle)" strokeWidth={4} strokeLinecap="round" />
        <polyline points={poly(ll[0])} fill="none" stroke="var(--rm-graph)" strokeWidth={2.2} />
      </Panel>
      <Panel title="vertex | edge — a parabola">
        <line x1={ox + directrix.x1 * scale} y1={oy - directrix.y1 * scale} x2={ox + directrix.x2 * scale} y2={oy - directrix.y2 * scale} stroke="var(--rm-obstacle)" strokeWidth={4} strokeLinecap="round" />
        <circle cx={ox + focus.x * scale} cy={oy - focus.y * scale} r={5} fill="var(--rm-obstacle)" />
        <polyline points={poly(pl)} fill="none" stroke="var(--rm-graph)" strokeWidth={2.2} />
      </Panel>
    </div>
  );
}
