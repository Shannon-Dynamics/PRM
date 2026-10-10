'use client';

import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { Rng } from '@/lib/prob/rng';
import { T2 } from '@/lib/manifold/torus';
import { KdTree, LinearNn, type KdBox } from '@/lib/sampling';

/**
 * f11.3 — a kd-tree over the torus.
 *
 * Thirty seeded points on T², the splitting cells of the incremental kd-tree
 * drawn on the unwrapped square, and one query placed next to the seam whose
 * nearest neighbor lives on the *other* side of it. Hover a point to light up
 * its subtree; the bound on the far box is min(|c − s|, T − |c − s|), which is
 * why the query's search crosses the seam instead of pruning it away.
 */

const SIZE = 380;
const PAD = 22;
const N_PTS = 30;
const QUERY: number[] = [2.95, 0.35];

const px = (c: number) => PAD + ((c + Math.PI) / (2 * Math.PI)) * (SIZE - 2 * PAD);
const py = (c: number) => SIZE - PAD - ((c + Math.PI) / (2 * Math.PI)) * (SIZE - 2 * PAD);
const clamp = (x: number) => Math.max(-Math.PI, Math.min(Math.PI, x));

export function KdTreeFigure() {
  const [hover, setHover] = useState<number | null>(null);

  const model = useMemo(() => {
    const rng = new Rng(1103);
    const tree = new KdTree<number[]>(T2);
    const oracle = new LinearNn<number[]>(T2);
    const pts: number[][] = [];
    for (let i = 0; i < N_PTS; i++) {
      const q = T2.sample(rng);
      pts.push(q);
      tree.insert(i, q);
      oracle.insert(i, q);
    }
    const boxes = tree.boxes();
    const nn = tree.nearest(QUERY)!;
    const visited = tree.visited;
    const check = oracle.nearest(QUERY)!;
    const parent = new Map<number, number>(boxes.map((b) => [b.idx, b.parent]));
    return { pts, boxes, nn, visited, check, parent };
  }, []);

  const inSubtree = (idx: number, root: number): boolean => {
    for (let i: number = idx; i >= 0; i = model.parent.get(i) ?? -1) if (i === root) return true;
    return false;
  };

  const q = QUERY;
  const n = model.pts[model.nn.idx];
  // The seam crossing: the geodesic from q to its neighbor, drawn in two pieces when it wraps.
  const wraps = Math.abs(n[0] - q[0]) > Math.PI || Math.abs(n[1] - q[1]) > Math.PI;
  const pieces: [number[], number[]][] = [];
  if (!wraps) pieces.push([q, n]);
  else {
    const nn = [n[0] + (n[0] - q[0] > Math.PI ? -2 * Math.PI : n[0] - q[0] < -Math.PI ? 2 * Math.PI : 0), n[1] + (n[1] - q[1] > Math.PI ? -2 * Math.PI : n[1] - q[1] < -Math.PI ? 2 * Math.PI : 0)];
    // Split at the seam crossing along x (the only wrap in this figure's query).
    const t = nn[0] > Math.PI ? (Math.PI - q[0]) / (nn[0] - q[0]) : (-Math.PI - q[0]) / (nn[0] - q[0]);
    const seamY = q[1] + t * (nn[1] - q[1]);
    const side = nn[0] > Math.PI ? Math.PI : -Math.PI;
    pieces.push([q, [side, seamY]], [[-side, seamY], n]);
  }

  const hoverBox = hover === null ? null : model.boxes.find((b) => b.idx === hover) ?? null;

  return (
    <WidgetFrame
      id="f11.3"
      title="kd-tree over the torus"
      teaches="A kd-tree splits on chart coordinates but must measure with the manifold's metric: a neighbor across the seam is a real neighbor, and the periodic pruning bound is what keeps it from being thrown away."
      colorKey={['graph', 'path', 'cobstacle']}
      caption={
        <>
          {N_PTS} seeded points on the unwrapped torus and the cells of the incremental kd-tree built
          over them: the root splits on θ₁, its children on θ₂, and so on down. Hover a point to see
          the cell its subtree occupies. The amber query sits {(Math.PI - QUERY[0]).toFixed(2)} rad
          from the seam at θ₁ = π; its nearest neighbor under the torus metric is on the far side,
          reached by the dashed geodesic through the seam, at distance{' '}
          {model.nn.dist.toFixed(3)} rad. A flat kd-tree would have pruned that subtree — the gap from
          the query to the box&rsquo;s edge looks like {(Math.PI - QUERY[0]).toFixed(2)} rad on one side
          and {(2 * Math.PI - (Math.PI - QUERY[0])).toFixed(2)} rad on the other, and the periodic
          bound keeps the smaller. The search visited {model.visited} of {N_PTS} nodes and agrees with
          the brute-force oracle{model.check.idx === model.nn.idx ? '' : ' — it does not, which would be a bug'}.
        </>
      }
    >
      <div className="flex justify-center p-3">
        <svg width={SIZE} height={SIZE} viewBox={`0 0 ${SIZE} ${SIZE}`} role="img" aria-label="A square chart of the torus with kd-tree splitting lines, a hovered subtree highlighted, and a query whose nearest neighbor lies across the seam." className="max-w-full" onMouseLeave={() => setHover(null)}>
          {/* The chart's box, seam dashed. */}
          <rect x={PAD} y={PAD} width={SIZE - 2 * PAD} height={SIZE - 2 * PAD} fill="var(--color-fd-card)" stroke="var(--rm-cobstacle)" strokeDasharray="6 4" strokeWidth={1.5} />
          {/* The hovered subtree's cell. */}
          {hoverBox ? <rect x={px(clamp(hoverBox.lo[0]))} y={py(clamp(hoverBox.hi[1]))} width={px(clamp(hoverBox.hi[0])) - px(clamp(hoverBox.lo[0]))} height={py(clamp(hoverBox.lo[1])) - py(clamp(hoverBox.hi[1]))} fill="var(--rm-graph)" fillOpacity={0.12} stroke="var(--rm-graph)" strokeWidth={1.5} /> : null}
          {/* Split lines, faded with depth. */}
          {model.boxes.map((b: KdBox) => {
            const lit = hover !== null && inSubtree(b.idx, hover);
            const opacity = hover === null ? Math.max(0.15, 0.7 - 0.09 * b.depth) : lit ? 0.9 : 0.08;
            const lo = b.lo.map(clamp);
            const hi = b.hi.map(clamp);
            return b.axis === 0 ? (
              <line key={`s${b.idx}`} x1={px(b.split)} x2={px(b.split)} y1={py(lo[1])} y2={py(hi[1])} stroke="var(--rm-graph)" strokeOpacity={opacity} strokeWidth={lit ? 1.6 : 1} />
            ) : (
              <line key={`s${b.idx}`} x1={px(lo[0])} x2={px(hi[0])} y1={py(b.split)} y2={py(b.split)} stroke="var(--rm-graph)" strokeOpacity={opacity} strokeWidth={lit ? 1.6 : 1} />
            );
          })}
          {/* The seam-crossing geodesic to the nearest neighbor. */}
          {pieces.map(([a, b], i) => (
            <line key={`g${i}`} x1={px(a[0])} y1={py(a[1])} x2={px(b[0])} y2={py(b[1])} stroke="var(--rm-path)" strokeWidth={2} strokeDasharray="5 4" />
          ))}
          {/* Points. */}
          {model.pts.map((p, i) => {
            const lit = hover === null || inSubtree(i, hover);
            const isNn = i === model.nn.idx;
            return (
              <g key={i} onMouseEnter={() => setHover(i)} style={{ cursor: 'pointer' }}>
                <circle cx={px(p[0])} cy={py(p[1])} r={9} fill="transparent" />
                <circle cx={px(p[0])} cy={py(p[1])} r={isNn ? 5 : i === hover ? 5 : 3.5} fill={isNn ? 'var(--rm-path)' : 'var(--rm-graph)'} fillOpacity={lit ? 1 : 0.25} stroke="var(--color-fd-card)" strokeWidth={1} />
              </g>
            );
          })}
          <circle cx={px(q[0])} cy={py(q[1])} r={6} fill="var(--rm-cobstacle)" stroke="var(--color-fd-card)" strokeWidth={1.5} />
          <text x={px(q[0]) - 9} y={py(q[1]) - 9} fontSize={10} fontFamily="ui-monospace, monospace" textAnchor="end" fill="var(--rm-cobstacle)">
            query
          </text>
          <text x={px(n[0]) + 8} y={py(n[1]) + 4} fontSize={10} fontFamily="ui-monospace, monospace" fill="var(--rm-path)">
            nearest, {model.nn.dist.toFixed(2)} rad
          </text>
          <text x={SIZE / 2} y={SIZE - 5} fontSize={10} fontFamily="ui-monospace, monospace" textAnchor="middle" fill="currentColor">
            θ₁ ∈ [−π, π)
          </text>
          <text x={8} y={SIZE / 2} fontSize={10} fontFamily="ui-monospace, monospace" textAnchor="middle" fill="currentColor" transform={`rotate(-90 8 ${SIZE / 2})`}>
            θ₂ ∈ [−π, π)
          </text>
          {hoverBox ? (
            <text x={PAD + 4} y={PAD - 7} fontSize={10} fontFamily="ui-monospace, monospace" fill="var(--rm-graph)">
              node {hoverBox.idx}: depth {hoverBox.depth}, splits on θ{hoverBox.axis + 1} = {hoverBox.split.toFixed(2)}
            </text>
          ) : (
            <text x={PAD + 4} y={PAD - 7} fontSize={10} fontFamily="ui-monospace, monospace" fill="currentColor" opacity={0.7}>
              hover a point → its subtree
            </text>
          )}
        </svg>
      </div>
    </WidgetFrame>
  );
}
