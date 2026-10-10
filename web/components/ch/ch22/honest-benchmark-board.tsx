'use client';

import { Fragment, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { ActionButton, ButtonRow, ControlPanel, Slider } from '@/components/sim/controls';
import { Dashboard, LineChart, StatTile, type BookRole } from '@/components/viz';
import { BENCH, PLANNERS, TRAINING_META, amortizedMs, breakEvenQueries, costRatioCurve, successCurve, summarize, type BoardRow, type PlannerName } from '@/lib/learn';

/**
 * w22.5 — Honest Benchmark Board.
 *
 * The D22.6 table from recorded runs of `lib/learn/bench.ts`: seven planners on
 * two suites of 30 solvable queries each — randomized Workbenches from the
 * training generator and from the held-out one — at eight collision-check
 * budgets, with cost ratio to the incumbent, wall-clock without and with
 * inference, and training time amortized over a chosen number of queries.
 * Nothing here is recomputed in the browser; the numbers are the harness's,
 * the harness is in the library, and the checks assert its invariants.
 */

const ROLE: Record<PlannerName, BookRole> = {
  uniform: 'graph',
  gaussian: 'obstacle',
  bridge: 'cobstacle',
  obprm: 'start',
  learned: 'robot',
  'rrt-connect': 'path',
  mpnet: 'goal',
};

const LOG_MIN = 0;
const LOG_MAX = 4; // 10^0 … 10^4 queries

const pct = (x: number) => `${(100 * x).toFixed(0)}%`;
const ratio = (x: number) => (Number.isFinite(x) ? x.toFixed(2) : '—');
const ms = (x: number) => (Number.isFinite(x) ? (x >= 100 ? x.toFixed(0) : x.toFixed(1)) : '—');

function trainingMsFor(row: BoardRow): number {
  if (row.planner === 'learned') return BENCH.trainingMs.sampler;
  if (row.planner === 'mpnet') return BENCH.trainingMs.policy;
  return 0;
}

export function HonestBenchmarkBoard() {
  const [suite, setSuite] = useState<'train' | 'heldout'>('train');
  const [budgetIdx, setBudgetIdx] = useState(Math.min(3, BENCH.budgets.length - 1));
  const [logK, setLogK] = useState(2);
  const budget = BENCH.budgets[budgetIdx];
  const k = Math.round(Math.pow(10, logK));

  const rowsTrain = useMemo(() => summarize(BENCH.suites.train, budget), [budget]);
  const rowsHeld = useMemo(() => summarize(BENCH.suites.heldout, budget), [budget]);
  const active = suite === 'train' ? rowsTrain : rowsHeld;
  const activeSuite = suite === 'train' ? BENCH.suites.train : BENCH.suites.heldout;

  const learned = active.find((r) => r.planner === 'learned')!;
  const uniform = active.find((r) => r.planner === 'uniform')!;
  const bridge = active.find((r) => r.planner === 'bridge')!;
  const breakEven = breakEvenQueries(learned, uniform, BENCH.trainingMs.sampler);

  const successSeries = useMemo(() => PLANNERS.map((p) => ({ id: p.label, role: ROLE[p.name], data: successCurve(activeSuite, p.name, BENCH.budgets) })), [activeSuite]);
  const ratioSeries = useMemo(
    () => PLANNERS.map((p) => ({ id: p.label, role: ROLE[p.name], data: costRatioCurve(activeSuite, p.name, BENCH.budgets).filter((d) => Number.isFinite(d.y)) })),
    [activeSuite],
  );

  const learnedTint = 'color-mix(in srgb, var(--rm-robot) 12%, transparent)';
  const activeTint = 'color-mix(in srgb, var(--rm-graph) 8%, transparent)';

  return (
    <WidgetFrame
      id="w22.5"
      title="Honest Benchmark Board"
      teaches="A benchmark without inference time and without held-out worlds is not a benchmark: the learned rows win on one side of the table and lose on the other, and training is paid for only by enough queries on similar worlds."
      colorKey={['graph', 'cobstacle', 'robot', 'path', 'goal']}
      wide
      caption={
        <>
          Recorded runs of the chapter&rsquo;s harness: {BENCH.meta.worldsPerSuite} solvable queries per suite, every planner on the same world,
          seed and query, stopped at {BENCH.maxBudget.toLocaleString()} collision checks. Success is read off at the chosen budget; cost
          ratio is against the incumbent (the shortest path any method found, including a dense reference PRM); wall-clock is the
          planning loop&rsquo;s, with the learned rows&rsquo; forward passes shown separately and added. The amortization slider spreads the
          offline training time ({(BENCH.trainingMs.sampler / 1000).toFixed(0)} s for the sampler, {(BENCH.trainingMs.policy / 1000).toFixed(0)} s for the
          policy, including the PRM runs that made the data) over k queries. Learned rows are tinted orange; the active suite&rsquo;s columns are shaded.
        </>
      }
    >
      <div className="px-3 py-3">
        <div className="overflow-x-auto">
          <table className="w-full text-xs tabular-nums" aria-label="Benchmark table: seven planners by success, cost ratio and wall-clock on two suites">
            <thead>
              <tr className="text-left text-fd-muted-foreground">
                <th rowSpan={2} className="py-1 pr-2 align-bottom">
                  planner
                </th>
                <th colSpan={4} className="py-1 text-center" style={{ background: suite === 'train' ? activeTint : undefined }}>
                  in-distribution (blocks)
                </th>
                <th colSpan={4} className="py-1 text-center" style={{ background: suite === 'heldout' ? activeTint : undefined }}>
                  held-out (bars)
                </th>
              </tr>
              <tr className="text-left text-fd-muted-foreground">
                {[0, 1].map((i) => {
                  const bg = (i === 0) === (suite === 'train') ? activeTint : undefined;
                  return (
                    <Fragment key={i}>
                      <th className="py-1 px-1" style={{ background: bg }}>
                        success @ {budget}
                      </th>
                      <th className="py-1 px-1" style={{ background: bg }}>
                        cost / incumbent
                      </th>
                      <th className="py-1 px-1" style={{ background: bg }}>
                        ms (no inference)
                      </th>
                      <th className="py-1 px-1" style={{ background: bg }}>
                        ms incl. inference, amortized k = {k}
                      </th>
                    </Fragment>
                  );
                })}
              </tr>
            </thead>
            <tbody>
              {PLANNERS.map((p, i) => {
                const rt = rowsTrain[i];
                const rh = rowsHeld[i];
                const cell = (r: BoardRow, isActive: boolean) => (
                  <>
                    <td className="py-1 px-1" style={{ background: isActive ? activeTint : undefined }}>
                      {pct(r.success)} <span className="text-fd-muted-foreground">({r.solvedCount}/{r.total})</span>
                    </td>
                    <td className="py-1 px-1" style={{ background: isActive ? activeTint : undefined }}>
                      {ratio(r.costRatio)}
                    </td>
                    <td className="py-1 px-1" style={{ background: isActive ? activeTint : undefined }}>
                      {ms(r.ms - r.inferenceMs)}
                    </td>
                    <td className="py-1 px-1" style={{ background: isActive ? activeTint : undefined }}>
                      {ms(amortizedMs(r, trainingMsFor(r), k))}
                      {r.learned && <span className="text-fd-muted-foreground"> (+{ms(r.inferenceMs)} inf.)</span>}
                    </td>
                  </>
                );
                return (
                  <tr key={p.name} className="border-t border-fd-border" style={{ background: p.learned ? learnedTint : undefined }}>
                    <td className="py-1 pr-2 font-medium" style={{ color: `var(--rm-${ROLE[p.name]})` }}>
                      {p.label}
                    </td>
                    {cell(rt, suite === 'train')}
                    {cell(rh, suite === 'heldout')}
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
        <div className="mt-3">
          <Dashboard columns={4}>
            <StatTile label="learned vs uniform success" value={`${pct(learned.success)} vs ${pct(uniform.success)}`} role="robot" trend={Number((learned.costRatio - uniform.costRatio).toFixed(2))} trendLabel="cost-ratio difference" />
            <StatTile label="learned vs bridge success" value={`${pct(learned.success)} vs ${pct(bridge.success)}`} role="cobstacle" />
            <StatTile label="break-even k vs uniform" value={Number.isFinite(breakEven) ? breakEven : 'never'} precision={0} role="path" trend={Number((learned.ms - uniform.ms).toFixed(1))} trendLabel="ms per query, learned − uniform" />
            <StatTile label="training cost" value={(BENCH.trainingMs.sampler / 1000).toFixed(0)} unit="s" role="obstacle" trend={TRAINING_META.trainWorlds} trendLabel="training worlds" />
          </Dashboard>
        </div>
        <div className="mt-3 grid gap-3 md:grid-cols-2">
          <LineChart
            series={successSeries}
            xLabel="budget (collision checks)"
            yLabel="success"
            height={230}
            legend
            curve="monotoneX"
            yMin={0}
            yMax={1}
            markers={[{ axis: 'x', value: budget, label: `budget ${budget}`, role: 'graph' }]}
            ariaLabel="Success fraction against collision-check budget for seven planners on the active suite."
            caption={<>Success at budget, {suite === 'train' ? 'in-distribution' : 'held-out'} suite.</>}
          />
          <LineChart
            series={ratioSeries}
            xLabel="budget (collision checks)"
            yLabel="cost / incumbent"
            height={230}
            legend
            curve="monotoneX"
            yMin={1}
            markers={[{ axis: 'x', value: budget, label: `budget ${budget}`, role: 'graph' }]}
            ariaLabel="Mean cost ratio to the incumbent against budget for seven planners on the active suite."
            caption={<>Cost ratio of the first path found, over the queries solved within the budget.</>}
          />
        </div>
      </div>
      <ControlPanel columns={3} title="headline — which suite">
        <ButtonRow>
          <ActionButton emphasis={suite === 'train'} onClick={() => setSuite('train')}>
            in-distribution
          </ActionButton>
          <ActionButton emphasis={suite === 'heldout'} onClick={() => setSuite('heldout')}>
            held-out
          </ActionButton>
        </ButtonRow>
        <Slider label="budget (checks)" role="obstacle" value={budgetIdx} min={0} max={BENCH.budgets.length - 1} step={1} onChange={(x) => setBudgetIdx(Math.round(x))} format={() => String(budget)} />
        <Slider label="amortize training over k queries" role="robot" value={logK} min={LOG_MIN} max={LOG_MAX} step={0.1} onChange={setLogK} format={() => `k = ${k}`} />
      </ControlPanel>
      <p className="px-3 pb-3 text-xs text-fd-muted-foreground">
        Recorded {BENCH.meta.date} with seed {BENCH.meta.seed}; wall-clock is Node on the machine that ran the harness and is only comparable within the table.
      </p>
    </WidgetFrame>
  );
}
