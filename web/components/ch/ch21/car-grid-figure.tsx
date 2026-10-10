import { Hitch } from '@/lib/robots/hitch';
import { FIG1225_BLOCK, figure1225 } from '@/lib/nonholonomic';

/**
 * f21.7 — CAR GRID SEARCH tree and OPEN list (static SVG). Computed at render
 * from `figure1225`: forward actions L⁺, S⁺, R⁺ only, cost 1 per motion and 5
 * per steering change, wheels straight at the start, three pops. Expanded nodes
 * are filled blue, OPEN nodes hollow with their cost, motions that hit the
 * block are gray and never enter OPEN. Hover a node for its cost.
 */

const VB = { minX: 2.4, maxX: 8.8, minY: 2.3, maxY: 8.2 };
const W = 640;
const H = Math.round((W * (VB.maxY - VB.minY)) / (VB.maxX - VB.minX));
const X = (x: number) => ((x - VB.minX) / (VB.maxX - VB.minX)) * W;
const Y = (y: number) => H - ((y - VB.minY) / (VB.maxY - VB.minY)) * H;

export function CarGridFigure() {
  const fig = figure1225(Hitch.car());
  const openIds = new Set(fig.open.map((o) => o.id));
  const name = (id: number) => `q${id}`;
  const b = FIG1225_BLOCK;
  return (
    <figure id="f21.7" className="not-prose my-8">
      <div className="overflow-hidden rounded-md border border-fd-border bg-fd-card">
        <div className="flex flex-wrap items-baseline gap-x-3 border-b border-fd-border px-3 py-2">
          <span className="font-mono text-[0.7rem] font-medium" style={{ color: 'var(--pr-widget-ink)' }}>
            f21.7
          </span>
          <span className="font-display text-sm font-semibold">CAR GRID SEARCH tree and OPEN list</span>
        </div>
        <svg viewBox={`0 0 ${W} ${H + 70}`} className="block w-full" role="img" aria-label="Search tree of Algorithm 22 after three pops, with gray collided motions and the cost-ordered OPEN list.">
          <rect x={X(b.minX)} y={Y(b.maxY)} width={X(b.maxX) - X(b.minX)} height={Y(b.minY) - Y(b.maxY)} fill="var(--rm-obstacle)" opacity={0.85} />
          <text x={X(b.minX) + 8} y={Y(b.minY) - 8} fontSize={12} fill="var(--color-fd-background)">
            block
          </text>
          {fig.nodes.slice(1).map((n) => {
            const d = n.sweep.map((q, i) => `${i === 0 ? 'M' : 'L'}${X(q.x).toFixed(1)},${Y(q.y).toFixed(1)}`).join(' ');
            return (
              <path
                key={`e${n.id}`}
                d={d}
                fill="none"
                stroke={n.collided ? 'var(--color-fd-muted-foreground)' : 'var(--rm-graph)'}
                strokeWidth={n.collided ? 1.5 : 2.2}
                strokeDasharray={n.collided ? '4 4' : undefined}
                opacity={n.collided ? 0.7 : 1}
              />
            );
          })}
          {fig.nodes.map((n) => {
            const q = n.q.body;
            const isOpen = openIds.has(n.id);
            const fill = n.expanded ? 'var(--rm-graph)' : n.collided ? 'var(--color-fd-muted-foreground)' : 'var(--color-fd-card)';
            return (
              <g key={`n${n.id}`}>
                <title>{`${name(n.id)}: ${n.action ?? 'start'}, cost ${n.cost}${n.collided ? ' — motion collides, not in OPEN' : n.expanded ? ' — expanded' : isOpen ? ' — in OPEN' : ''}`}</title>
                <circle cx={X(q.x)} cy={Y(q.y)} r={7} fill={fill} stroke={n.collided ? 'var(--color-fd-muted-foreground)' : 'var(--rm-graph)'} strokeWidth={2} />
                <text x={X(q.x) + 9} y={Y(q.y) - 9} fontSize={12} fontFamily="ui-monospace, monospace" fill="var(--color-fd-foreground)">
                  {name(n.id)}
                  {n.id > 0 ? ` ${n.action}` : ''}
                  {isOpen ? ` (${n.cost})` : ''}
                </text>
              </g>
            );
          })}
          <text x={10} y={H + 22} fontSize={13} fontFamily="ui-monospace, monospace" fill="var(--color-fd-foreground)">
            {`popped: ${fig.popOrder.map(name).join(', ')}`}
          </text>
          <text x={10} y={H + 48} fontSize={13} fontFamily="ui-monospace, monospace" fill="var(--rm-graph)">
            {`OPEN = { ${fig.open.map((o) => `${name(o.id)} (${o.cost})`).join(', ')} }`}
          </text>
        </svg>
      </div>
      <figcaption className="pr-caption mt-2 font-ui text-[0.8rem] leading-relaxed text-fd-muted-foreground">
        Algorithm 22 on forward actions only, cost 1 per motion and 5 per steering change, wheels straight at q0. Filled nodes have
        been popped and expanded, hollow ones wait in OPEN with their cost, gray dashed motions hit the block and never enter OPEN.
        The next pop is the cheapest — three straight steps (cost 3) — even though a left turn reached farther: OPEN is sorted by
        cost, and with integer costs it is an array of FIFO buckets. Choset&apos;s Fig. 12.25 shows the same bookkeeping on a
        different tree (costs 3, 7, 7, 7, 8).
      </figcaption>
    </figure>
  );
}
