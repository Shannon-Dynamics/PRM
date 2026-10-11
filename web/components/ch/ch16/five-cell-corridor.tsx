'use client';

import { useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { FIVE_CELLS, fiveCellSenseMove } from '@/lib/estimate';

/**
 * f16.3 — The Five-Cell Corridor.
 *
 * Choset's Figure 9.1 shrunk to the chapter's §3 micro-example: five cells on a
 * loop, doors at 1 and 3, P(door | door cell) = 0.8, P(door | wall cell) = 0.4,
 * "right 1" succeeds with 0.8 and stays with 0.2. Four rows — prior, likelihood
 * strip, posterior after sensing a door, belief after moving right — with the
 * exact sevenths and entropies printed. Every number comes from
 * `fiveCellSenseMove()` in `lib/estimate/bayes.ts`, the same function the
 * check file pins to 1e-4; nothing here is typed in by hand.
 */

const STAGES = ['prior', 'likelihood', 'posterior', 'moved'] as const;
type Stage = (typeof STAGES)[number];

const SISTER_HALLWAY = 'https://prvr.shannon.id/chapters/ch05-bayes-filter';

const FRACTIONS: Record<Exclude<Stage, 'likelihood'>, string[]> = {
  prior: ['1/5', '1/5', '1/5', '1/5', '1/5'],
  posterior: ['1/7', '2/7', '1/7', '2/7', '1/7'],
  moved: ['1.0/7', '1.2/7', '1.8/7', '1.2/7', '1.8/7'],
};

export function FiveCellCorridor() {
  const trace = useMemo(() => fiveCellSenseMove(FIVE_CELLS), []);
  const [hover, setHover] = useState<Stage | null>(null);

  const rows: { stage: Stage; title: string; values: number[]; role: string; entropy?: number; note: string }[] = [
    { stage: 'prior', title: 'prior P(x(0)) — switched on, told nothing', values: trace.prior, role: 'var(--pr-prior)', entropy: trace.entropy.prior, note: 'uniform: 1/5 everywhere' },
    { stage: 'likelihood', title: 'likelihood P(y = door | x) — the green strip', values: trace.sense.likelihood, role: 'var(--pr-measurement)', note: '0.8 at a door cell, 0.4 at a wall cell; not a distribution over x' },
    { stage: 'posterior', title: 'posterior after sensing a door (9.4–9.6)', values: trace.sense.posterior, role: 'var(--pr-posterior)', entropy: trace.entropy.sensed, note: `unnormalized [0.08, 0.16, 0.08, 0.16, 0.08], η⁻¹ = ${trace.sense.evidence.toFixed(2)}` },
    { stage: 'moved', title: 'after moving right 1 (9.3): 0.8·bel(c−1) + 0.2·bel(c)', values: trace.moved, role: 'var(--pr-prediction)', entropy: trace.entropy.moved, note: 'sensing sharpened the belief; moving smeared it' },
  ];

  const W = 520;
  const cellW = 72;
  const left = 150;
  const rowH = 74;
  const barMax = 46;
  const H = rowH * rows.length + 36;

  return (
    <WidgetFrame
      id="f16.3"
      title="The Five-Cell Corridor"
      teaches="Sensing sharpens, moving smears — and both are the same equation applied to a table of five numbers."
      colorKey={['prior', 'measurement', 'posterior', 'prediction']}
      caption={
        <>
          Choset&rsquo;s Figure 9.1 on five cells with his problem 6–7 numbers. Hover a row for the exact fractions. The entropy
          falls from {trace.entropy.prior.toFixed(4)} to {trace.entropy.sensed.toFixed(4)} bits when the robot sees a door and climbs
          back to {trace.entropy.moved.toFixed(4)} bits when it moves: the move is a convolution with a stochastic kernel, and a
          convolution can only spread mass. The sister book&rsquo;s{' '}
          <a href={SISTER_HALLWAY} className="underline decoration-dotted" target="_blank" rel="noreferrer">
            Hallway Belief Machine
          </a>{' '}
          runs the same recursion on a continuous corridor with as many doors as you like.
        </>
      }
    >
      <div className="overflow-x-auto px-3 py-3">
        <svg viewBox={`0 0 ${W} ${H}`} width="100%" style={{ maxWidth: W, display: 'block', margin: '0 auto' }} role="img" aria-label="Four rows of five bars: a flat prior, a likelihood strip higher at the two door cells, a posterior with peaks at the door cells, and a smeared belief after moving one cell right. Entropies are printed beside the rows.">
          {/* cell headers */}
          {[0, 1, 2, 3, 4].map((c) => (
            <g key={c}>
              <text x={left + c * cellW + cellW / 2} y={14} textAnchor="middle" fontSize={11} fill="var(--pr-canvas-ink)" fontFamily="ui-monospace, monospace">
                cell {c}
              </text>
              {FIVE_CELLS.doors.includes(c) ? (
                <rect x={left + c * cellW + cellW / 2 - 9} y={19} width={18} height={6} rx={1} fill="var(--pr-measurement)" opacity={0.8} />
              ) : (
                <rect x={left + c * cellW + 4} y={22} width={cellW - 8} height={2} fill="var(--pr-wall)" opacity={0.6} />
              )}
            </g>
          ))}
          <text x={left + 5 * cellW + 6} y={14} fontSize={9} fill="var(--pr-truth)" fontFamily="ui-monospace, monospace">
            ▬ door
          </text>
          {rows.map((r, k) => {
            const y0 = 34 + k * rowH;
            const dim = hover !== null && hover !== r.stage;
            const vmax = r.stage === 'likelihood' ? 1 : 0.3;
            return (
              <g key={r.stage} opacity={dim ? 0.35 : 1} onPointerEnter={() => setHover(r.stage)} onPointerLeave={() => setHover(null)} style={{ cursor: 'help' }}>
                <rect x={0} y={y0} width={W} height={rowH - 4} fill={hover === r.stage ? 'var(--pr-grid)' : 'transparent'} />
                <text x={6} y={y0 + 16} fontSize={10.5} fill={r.role} fontWeight={600} fontFamily="ui-monospace, monospace">
                  {r.stage}
                </text>
                <text x={6} y={y0 + 30} fontSize={8.5} fill="var(--pr-truth)" fontFamily="ui-monospace, monospace">
                  {r.entropy !== undefined ? `H = ${r.entropy.toFixed(4)} bits` : 'P(y | x), not a belief'}
                </text>
                {r.stage !== 'likelihood' ? (
                  <text x={6} y={y0 + 43} fontSize={8.5} fill="var(--pr-truth)" fontFamily="ui-monospace, monospace">
                    Σ = {r.values.reduce((a, b) => a + b, 0).toFixed(2)}
                  </text>
                ) : null}
                {/* baseline */}
                <line x1={left} x2={left + 5 * cellW} y1={y0 + 8 + barMax} y2={y0 + 8 + barMax} stroke="var(--pr-wall)" strokeOpacity={0.5} />
                {r.values.map((v, c) => {
                  const h = Math.max(1.5, (v / vmax) * barMax);
                  const x = left + c * cellW + 10;
                  const yTop = y0 + 8 + barMax - h;
                  const labelText = hover === r.stage && r.stage !== 'likelihood' ? FRACTIONS[r.stage][c] : v.toFixed(r.stage === 'likelihood' ? 1 : 4);
                  return (
                    <g key={c}>
                      <rect x={x} y={yTop} width={cellW - 20} height={h} fill={r.role} opacity={0.85} rx={1} />
                      <text x={x + (cellW - 20) / 2} y={yTop - 3} textAnchor="middle" fontSize={8.5} fill="var(--pr-canvas-ink)" fontFamily="ui-monospace, monospace">
                        {labelText}
                      </text>
                    </g>
                  );
                })}
              </g>
            );
          })}
          <text x={left} y={H - 8} fontSize={8.5} fill="var(--pr-truth)" fontFamily="ui-monospace, monospace">
            {hover ? rows.find((r) => r.stage === hover)?.note : 'hover a row: exact fractions and what the step did'}
          </text>
        </svg>
      </div>
      <div className="border-t border-fd-border px-3 py-2 font-ui text-[0.72rem] text-fd-muted-foreground">
        {hover ? rows.find((r) => r.stage === hover)?.title : 'Rows: prior → likelihood of "door" → posterior → prediction after "right 1". Doors at cells 1 and 3; the loop wraps 4 → 0.'}
      </div>
    </WidgetFrame>
  );
}
