'use client';

import { useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { ActionButton, ButtonRow, ControlPanel } from '@/components/sim/controls';
import { BarChart, type BookRole } from '@/components/viz';

/**
 * f12.4 — Five planners × three worlds.
 *
 * Build-time data: medians over 20 seeds of the `tree_bench` run — RRT, EST,
 * RRT-Connect, SBL and KPIECE from the book's `sampling::tree` port, on the
 * Apartment (Rusty as a disc, ℝ²), the Workbench (Reach, T²) and the Lot
 * (Hitch, SE(2) with a straight-line steer that ignores the car). Budgets:
 * RRT 8 000 iterations, EST/SBL/KPIECE 10 000, RRT-Connect 6 000 attempts;
 * η = 0.6 m / 0.4 rad / 0.8 (SE(2) units); goal radius η; subdivision edge
 * checks at 0.05 m, 0.05 rad, 0.08. The Lot row is the deliberate wrong
 * answer: a feasible path in SE(2) that the car cannot drive.
 */

interface Row {
  planner: string;
  world: string;
  success: number;
  iterations: number;
  checks: number;
  length: number;
}

const SEEDS = 20;

const WORLDS = ['Apartment · Rusty (ℝ²)', 'Workbench · Reach (T²)', 'Lot · Hitch (SE(2), straight-line steer)'] as const;

const PLANNERS: { name: string; role: BookRole }[] = [
  { name: 'RRT', role: 'graph' },
  { name: 'RRT-Connect', role: 'path' },
  { name: 'EST', role: 'cobstacle' },
  { name: 'SBL', role: 'start' },
  { name: 'KPIECE', role: 'robot' },
];

/** Output of `tree_bench` (web port, 20 seeds), medians over the solved runs. */
const DATA: Row[] = [
  { planner: 'RRT', world: WORLDS[0], success: 20, iterations: 1382, checks: 5959, length: 14.03 },
  { planner: 'EST', world: WORLDS[0], success: 19, iterations: 5369, checks: 60666, length: 19.19 },
  { planner: 'RRT-Connect', world: WORLDS[0], success: 20, iterations: 1326, checks: 5075, length: 15.43 },
  { planner: 'SBL', world: WORLDS[0], success: 20, iterations: 1834, checks: 3939, length: 24.21 },
  { planner: 'KPIECE', world: WORLDS[0], success: 20, iterations: 2121, checks: 20800, length: 21.9 },
  { planner: 'RRT', world: WORLDS[1], success: 20, iterations: 88, checks: 784, length: 5.76 },
  { planner: 'EST', world: WORLDS[1], success: 20, iterations: 909, checks: 9902, length: 7.34 },
  { planner: 'RRT-Connect', world: WORLDS[1], success: 20, iterations: 4, checks: 191, length: 4.81 },
  { planner: 'SBL', world: WORLDS[1], success: 20, iterations: 269, checks: 615, length: 8.72 },
  { planner: 'KPIECE', world: WORLDS[1], success: 20, iterations: 544, checks: 4107, length: 12.5 },
  { planner: 'RRT', world: WORLDS[2], success: 20, iterations: 2003, checks: 3140, length: 11.89 },
  { planner: 'EST', world: WORLDS[2], success: 20, iterations: 1783, checks: 6895, length: 18.12 },
  { planner: 'RRT-Connect', world: WORLDS[2], success: 17, iterations: 2477, checks: 3780, length: 11.05 },
  { planner: 'SBL', world: WORLDS[2], success: 20, iterations: 1754, checks: 3979, length: 17.95 },
  { planner: 'KPIECE', world: WORLDS[2], success: 17, iterations: 1385, checks: 4841, length: 19.8 },
];

type Metric = 'checks' | 'iterations' | 'length';
const METRICS: { key: Metric; label: string; axis: string }[] = [
  { key: 'checks', label: 'collision checks', axis: 'median collision checks (log scale shown as bars)' },
  { key: 'iterations', label: 'iterations', axis: 'median iterations to first solution' },
  { key: 'length', label: 'path length', axis: 'median path length (m, rad, or SE(2) units)' },
];

const SHORT: Record<string, string> = { [WORLDS[0]]: 'Apartment (ℝ²)', [WORLDS[1]]: 'Workbench (T²)', [WORLDS[2]]: 'Lot (SE(2))' };

export function PlannerBench() {
  const [metric, setMetric] = useState<Metric>('checks');
  const m = METRICS.find((x) => x.key === metric)!;
  const series = PLANNERS.map((p) => ({
    id: p.name,
    role: p.role,
    data: WORLDS.map((w) => {
      const row = DATA.find((r) => r.planner === p.name && r.world === w)!;
      return { x: SHORT[w], y: metric === 'checks' ? Math.log10(row.checks) : row[metric] };
    }),
  }));

  return (
    <WidgetFrame
      id="f12.4"
      title="Five planners × three worlds"
      teaches="Single- and multi-query planners have different economics, lazy checking buys checks with path quality — and a feasible path for Hitch under a straight-line steer is a path the car cannot drive."
      colorKey={['graph', 'path', 'cobstacle', 'start', 'robot']}
      caption={
        <>
          Medians over {SEEDS} seeds from the book&rsquo;s `tree_bench`: RRT, RRT-Connect, EST, SBL and
          KPIECE, each with the same straight-line steer, on the three worlds. Collision checks are
          shown on a log₁₀ scale. On the Workbench, RRT-Connect merges in a handful of attempts for a
          few hundred checks — a hundredth of the Chapter 11 roadmap&rsquo;s build. SBL has the
          smallest check bill on the Apartment and the longest path: it verified one bridging path and
          nothing else. EST&rsquo;s push pays most. The Lot column is the deliberate wrong answer: every
          planner finds a path for Hitch, and none of them is drivable, because the steer interpolates
          heading as freely as position. Two planners solved 17 of 20 Lot seeds within budget; the
          medians are over the solved runs.
        </>
      }
    >
      <div className="px-3 pt-3">
        <BarChart
          series={series}
          xLabel="world"
          yLabel={metric === 'checks' ? 'log₁₀ collision checks' : m.axis}
          height={280}
          groupMode="grouped"
          valueFormat={(v) => (metric === 'checks' ? Math.round(Math.pow(10, v)).toLocaleString() : v.toFixed(metric === 'length' ? 2 : 0))}
          ariaLabel={`Grouped bars of median ${m.label} for five tree planners on three worlds.`}
        />
      </div>
      <div className="overflow-x-auto border-t border-fd-border">
        <table className="w-full min-w-[36rem] border-collapse text-left font-ui text-[0.78rem]">
          <thead>
            <tr className="border-b border-fd-border">
              <th className="px-3 py-1.5 font-medium">world</th>
              <th className="px-3 py-1.5 font-medium">planner</th>
              <th className="px-3 py-1.5 text-right font-medium">solved</th>
              <th className="px-3 py-1.5 text-right font-medium">iterations</th>
              <th className="px-3 py-1.5 text-right font-medium">collision checks</th>
              <th className="px-3 py-1.5 text-right font-medium">path length</th>
            </tr>
          </thead>
          <tbody className="font-mono tabular-nums">
            {DATA.map((r) => (
              <tr key={r.world + r.planner} className="border-b border-fd-border last:border-b-0">
                <td className="px-3 py-1 font-ui">{SHORT[r.world]}</td>
                <td className="px-3 py-1 font-ui" style={{ color: `var(--rm-${PLANNERS.find((p) => p.name === r.planner)!.role})` }}>
                  {r.planner}
                </td>
                <td className="px-3 py-1 text-right">
                  {r.success}/{SEEDS}
                </td>
                <td className="px-3 py-1 text-right">{r.iterations.toLocaleString()}</td>
                <td className="px-3 py-1 text-right">{r.checks.toLocaleString()}</td>
                <td className="px-3 py-1 text-right">{r.length.toFixed(2)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <ControlPanel columns={1}>
        <ButtonRow>
          {METRICS.map((x) => (
            <ActionButton key={x.key} emphasis={metric === x.key} onClick={() => setMetric(x.key)}>
              {x.label}
            </ActionButton>
          ))}
        </ButtonRow>
      </ControlPanel>
    </WidgetFrame>
  );
}
