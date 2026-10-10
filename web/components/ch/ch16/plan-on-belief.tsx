'use client';

import { useCallback, useEffect, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { StatTile } from '@/components/viz/stat-tile';
import { useSimulation } from '@/lib/sim/use-simulation';
import { useExplorableState } from '@/lib/explorable/store';
import { clear, drawParticles, drawPath, drawRobot, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT } from '@/lib/sim/world';
import { cellCenter, cellIJ } from '@/lib/search/grid';
import {
  ACTION_STEP,
  BeliefNavigator,
  defaultNavigator,
  replan3x3,
  type Action,
  type NavigatorSnapshot,
} from '@/lib/estimate';

/**
 * w16.2 — Plan-on-Belief, the chapter's owned artifact run live.
 *
 * Rusty is truly in room B, flush under the corridor wall. The belief says
 * room A (under A's open doorway) with weight 0.6 and room B with weight 0.4.
 * Each tick the navigator samples K cells from the cloud, scores every lattice
 * action by Σ_j ω_j [c(a) + J(f(x_j, a))] against the Chapter 6 cost-to-go
 * field J, executes the argmin, scans, and updates. From A "Up" is the door;
 * from B "Up" is a wall — so the mode says Up, the belief says sideways, and
 * which one wins depends on λ. The break-even is drawn on the slider.
 */

const CHAPTER_LAMBDA_KEY = 'ch16.lambda';
const LAMBDA_MIN = 1;
const LAMBDA_MAX = 30;
const REST_TICKS = 10;

interface Params {
  kSamples: number;
  betaInfo: number;
  planFromMode: boolean;
  freezeBelief: boolean;
}

interface State {
  seed: number;
  nav: BeliefNavigator;
  snap: NavigatorSnapshot;
  restTicks: number;
  runs: number;
}

const DEFAULT_PARAMS: Params = { kSamples: 24, betaInfo: 0, planFromMode: false, freezeBelief: false };

export function PlanOnBelief() {
  const [params, setParams] = useState<Params>(DEFAULT_PARAMS);
  // λ is shared with the prose: a <Scrub id="ch16.lambda"> and this slider are two views of one number.
  const [lambda, setLambda] = useExplorableState(CHAPTER_LAMBDA_KEY, 20);

  const init = useCallback(
    (seed: number): State => {
      const nav = defaultNavigator(APARTMENT, seed, {
        lambda,
        betaInfo: params.betaInfo,
        kSamples: params.kSamples,
        planFromMode: params.planFromMode,
        freezeBelief: params.freezeBelief,
      });
      return { seed, nav, snap: nav.snapshot(), restTicks: 0, runs: 0 };
    },
    // Only the seed re-creates the scene; every parameter is applied live in `step`.
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [],
  );

  const step = useCallback(
    (s: State): State => {
      const { nav } = s;
      // Live parameters: λ never reruns Dijkstra, the rest are plain fields of the loop.
      if (nav.cfg.lambda !== lambda) nav.setLambda(lambda);
      if (nav.cfg.betaInfo !== params.betaInfo) nav.cfg = { ...nav.cfg, betaInfo: params.betaInfo };
      nav.kSamples = params.kSamples;
      nav.planFromMode = params.planFromMode;
      nav.freezeBelief = params.freezeBelief;

      if (nav.reached) {
        // Hold the final frame for a moment, then run the same seed again.
        if (s.restTicks < REST_TICKS) return { ...s, restTicks: s.restTicks + 1 };
        const fresh = defaultNavigator(APARTMENT, s.seed + 101 * (s.runs + 1), {
          lambda,
          betaInfo: params.betaInfo,
          kSamples: params.kSamples,
          planFromMode: params.planFromMode,
          freezeBelief: params.freezeBelief,
        });
        return { seed: s.seed, nav: fresh, snap: fresh.snapshot(), restTicks: 0, runs: s.runs + 1 };
      }
      const snap = nav.step();
      return { ...s, snap };
    },
    [lambda, params],
  );

  const sim = useSimulation<State>({ init, step, fps: 3, initialSeed: 1616 });

  // Keep the inset honest while paused: a λ change re-scores the last sample set immediately.
  const [pausedTable, setPausedTable] = useState<NavigatorSnapshot | null>(null);
  useEffect(() => {
    const { nav } = sim.state;
    if (nav.cfg.lambda !== lambda) nav.setLambda(lambda);
    if (nav.lastSamples.length > 0) {
      const decision = nav.decide(nav.lastSamples);
      nav.lastDecision = decision;
      setPausedTable(nav.snapshot());
    } else setPausedTable(null);
  }, [lambda, sim.state]);

  const snap = pausedTable ?? sim.state.snap;
  const nav = sim.state.nav;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const g = nav.lattice;
      const cs = g.cellSize;
      const w = sl(v, cs);

      // The cost-to-go field J, shaded in the roadmap colour: dark near the goal, pale far away.
      let jmax = 0;
      for (let c = 0; c < g.occupied.length; c++) if (nav.J.isFree(c)) jmax = Math.max(jmax, nav.J.J(c));
      ctx.save();
      for (let c = 0; c < g.occupied.length; c++) {
        const { x, y } = cellCenter(g, c);
        const px = sx(v, x) - w / 2;
        const py = sy(v, y) - w / 2;
        if (g.occupied[c]) {
          ctx.fillStyle = p.obstacle;
          ctx.globalAlpha = 0.18;
          ctx.fillRect(px, py, w, w);
        } else if (nav.J.isFree(c)) {
          ctx.fillStyle = p.graph;
          ctx.globalAlpha = 0.04 + 0.4 * (1 - nav.J.J(c) / Math.max(jmax, 1e-9));
          ctx.fillRect(px, py, w, w);
        }
      }
      ctx.restore();
      drawWorld(ctx, v, APARTMENT, p);

      // The belief: the whole cloud as dots, the K scored samples as filled cells (never a stroke — the path owns strokes).
      drawParticles(ctx, v, nav.belief.pf.particles, p.posterior, { showHeading: false, maxRadius: 1.8 });
      ctx.save();
      ctx.fillStyle = p.posterior;
      const wmax = snap.samples.reduce((m, s) => Math.max(m, s.weight), 0) || 1;
      for (const s of snap.samples) {
        const { x, y } = cellCenter(g, s.cell);
        ctx.globalAlpha = 0.25 + 0.6 * (s.weight / wmax);
        ctx.fillRect(sx(v, x) - w / 2, sy(v, y) - w / 2, w, w);
      }
      ctx.restore();

      // The path the mode would follow, as a thick haloed stroke so purple-on-purple still reads.
      const path = nav.pathFromMode();
      if (path.length > 1) {
        drawPath(ctx, v, path.map((q) => ({ ...q, theta: 0 })), p.bg, { lineWidth: 7 });
        drawPath(ctx, v, path.map((q) => ({ ...q, theta: 0 })), p.path, { lineWidth: 3.2 });
      }

      // The chosen action, drawn from the mode cell.
      if (snap.decision) {
        const mode = cellCenter(g, nav.belief.mode(g));
        const [di, dj] = ACTION_STEP[snap.decision.action];
        const ax = sx(v, mode.x);
        const ay = sy(v, mode.y);
        const bx = sx(v, mode.x + di * cs * 1.4);
        const by = sy(v, mode.y + dj * cs * 1.4);
        ctx.save();
        ctx.strokeStyle = p.bg;
        ctx.lineWidth = 6;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(ax, ay);
        ctx.lineTo(bx, by);
        ctx.stroke();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(ax, ay);
        ctx.lineTo(bx, by);
        ctx.stroke();
        const ang = Math.atan2(by - ay, bx - ax);
        ctx.fillStyle = p.path;
        ctx.beginPath();
        ctx.moveTo(bx, by);
        ctx.lineTo(bx - 9 * Math.cos(ang - 0.4), by - 9 * Math.sin(ang - 0.4));
        ctx.lineTo(bx - 9 * Math.cos(ang + 0.4), by - 9 * Math.sin(ang + 0.4));
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }

      // Goal, executed trajectory, and Rusty where it really is.
      const goal = cellCenter(g, nav.goalCell);
      ctx.save();
      ctx.fillStyle = p.goal;
      ctx.beginPath();
      ctx.arc(sx(v, goal.x), sy(v, goal.y), Math.max(5, w * 0.35), 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      drawPath(ctx, v, [...nav.trail, nav.truth], p.robot, { lineWidth: 2 });
      drawRobot(ctx, v, nav.truth, p.robot, 0.26);

      const status = nav.reached ? `reached the goal in ${nav.steps} steps` : `step ${nav.steps}`;
      label(ctx, `${status}  ·  bumps ${nav.collisions}  ·  λ = ${lambda.toFixed(0)}${params.planFromMode ? '  ·  planning from the MODE' : ''}`, sx(v, 0.2), sy(v, 9) - 12, p.ink, { size: 11 });
    },
    [nav, snap, lambda, params.planFromMode],
  );

  const micro = useMemo(() => replan3x3(lambda), [lambda]);
  const flip = snap.flipLambda;
  const flipPct = flip !== null ? Math.max(0, Math.min(100, ((flip - LAMBDA_MIN) / (LAMBDA_MAX - LAMBDA_MIN)) * 100)) : null;

  return (
    <WidgetFrame
      id="w16.2"
      title="Plan-on-Belief"
      teaches="Planning on a belief does not mean planning from its mean or its mode. The mean can be in a wall and the mode can be wrong; expected cost-to-go over samples is the honest minimum short of a POMDP."
      colorKey={['posterior', 'graph', 'path', 'goal', 'robot', 'obstacle']}
      wide
      caption={
        <>
          Rusty (orange) is in room B; the cloud (purple dots) puts 0.6 of its mass in room A and 0.4 in room B. The blue field is
          the Chapter 6 cost-to-go J to the red goal, computed once. Each tick the planner scores every action over K sampled
          cells (filled purple squares) and executes the argmin (purple arrow); the haloed purple stroke is the route the mode
          alone would take. With the default λ the first moves are <em>sideways</em> — Up is a door under one hypothesis and a
          wall under the other — and the cloud collapses as corridor walls come into view. Tick <em>plan from the mode</em> and
          Rusty drives Up into the wall. Drag λ below the marked break-even and the belief planner gambles too.
        </>
      }
    >
      <SimCanvas
        world={APARTMENT.bounds}
        draw={draw}
        deps={[sim.tick, sim.state, snap, lambda, params.planFromMode]}
        aspect={12.8 / 9.8}
        padding={0.4}
        ariaLabel="The Apartment with a blue cost-to-go field shading free cells, a purple particle cloud split between two rooms, a thick purple path from the belief's mode to a red goal, and an orange robot that steps sideways first and then along the corridor."
      />
      <div className="grid grid-cols-1 gap-3 border-t border-fd-border px-3 py-3 lg:grid-cols-[1fr_auto]">
        <DecisionTable snap={snap} lambda={lambda} />
        <MicroExample micro={micro} lambda={lambda} />
      </div>
      <div className="grid grid-cols-2 gap-2 border-t border-fd-border px-3 py-2 sm:grid-cols-4">
        <StatTile label="belief entropy" value={snap.entropyBits} unit="bits" role="posterior" precision={2} sparkline={nav.entropyHistory.slice(-40)} />
        <StatTile label="break-even λ*" value={flip !== null ? flip : '—'} role="path" precision={1} />
        <StatTile label="bumps" value={nav.collisions} role="robot" precision={0} />
        <StatTile label="|mode − truth|" value={Math.hypot(nav.belief.modePose().x - nav.truth.x, nav.belief.modePose().y - nav.truth.y)} unit="m" role="truth" precision={2} />
      </div>
      <ControlPanel columns={1}>
        <div className="flex flex-col gap-1">
          <Slider label="Collision penalty λ (price of an action that ends in a wall)" role="path" value={lambda} min={LAMBDA_MIN} max={LAMBDA_MAX} step={0.5} format={(x) => x.toFixed(1)} onChange={setLambda} />
          <div className="relative h-3 w-full" aria-hidden="true">
            {flipPct !== null ? (
              <span className="absolute -translate-x-1/2 font-mono text-[0.62rem] leading-none" style={{ left: `${flipPct}%`, color: 'var(--rm-path)' }}>
                ▲ λ* = {flip!.toFixed(1)}
              </span>
            ) : (
              <span className="font-mono text-[0.62rem] leading-none text-fd-muted-foreground">no break-even: every action agrees under both hypotheses</span>
            )}
          </div>
        </div>
      </ControlPanel>
      <details className="border-t border-fd-border px-3 py-2">
        <summary className="cursor-pointer font-ui text-[0.72rem] font-medium text-fd-muted-foreground">More knobs: K, information bonus, plan from the mode, freeze the belief</summary>
        <div className="grid grid-cols-1 gap-x-5 gap-y-2.5 pt-2 sm:grid-cols-2">
          <Slider label="K samples per decision" role="posterior" value={params.kSamples} min={2} max={96} step={1} format={(k) => `${k}`} onChange={(k) => setParams((q) => ({ ...q, kSamples: k }))} />
          <Slider label="Information bonus β_info (0 disables)" role="measurement" value={params.betaInfo} min={0} max={3} step={0.1} format={(b) => b.toFixed(1)} onChange={(b) => setParams((q) => ({ ...q, betaInfo: b }))} />
          <Toggle label="Plan from the mode (certainty-equivalent)" role="path" checked={params.planFromMode} onChange={(m) => setParams((q) => ({ ...q, planFromMode: m }))} />
          <Toggle label="Freeze the belief (dead-reckon, no scan updates)" role="prediction" checked={params.freezeBelief} onChange={(f) => setParams((q) => ({ ...q, freezeBelief: f }))} />
        </div>
      </details>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => sim.reseed()}
        seed={sim.seed}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}

const ORDER: readonly Action[] = ['Up', 'UpRight', 'Right', 'DownRight', 'Down', 'DownLeft', 'Left', 'UpLeft'];

function DecisionTable({ snap, lambda }: { snap: NavigatorSnapshot; lambda: number }) {
  const rows = snap.decision ? [...snap.decision.table].sort((a, b) => ORDER.indexOf(a.action) - ORDER.indexOf(b.action)) : [];
  return (
    <div className="min-w-0">
      <p className="eyebrow mb-1">this tick: 𝒥_bel(a) = Σ ω_j [c(a) + J(f(x_j, a))] − β·ΔH_a at λ = {lambda.toFixed(1)}</p>
      <table className="w-full font-mono text-[0.7rem] tabular-nums">
        <thead className="text-fd-muted-foreground">
          <tr>
            <th className="text-left font-medium">action</th>
            <th className="text-right font-medium">expected</th>
            <th className="text-right font-medium" title="mass of samples for which the action ends in a wall or off the grid">collision mass</th>
            <th className="text-right font-medium">info bonus</th>
            <th className="text-right font-medium">total</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((r) => {
            const win = snap.decision?.action === r.action;
            return (
              <tr key={r.action} style={win ? { color: 'var(--rm-path)', fontWeight: 600 } : undefined}>
                <td className="text-left">{win ? '▶ ' : ''}{r.action}</td>
                <td className="text-right">{r.expected.toFixed(2)}</td>
                <td className="text-right">{r.collisionMass.toFixed(2)}</td>
                <td className="text-right">{r.infoBonus > 0 ? `−${r.infoBonus.toFixed(2)}` : '0'}</td>
                <td className="text-right">{r.total.toFixed(2)}</td>
              </tr>
            );
          })}
          {rows.length === 0 ? (
            <tr>
              <td colSpan={5} className="text-fd-muted-foreground">no decision yet — press play or step</td>
            </tr>
          ) : null}
        </tbody>
      </table>
    </div>
  );
}

function MicroExample({ micro, lambda }: { micro: ReturnType<typeof replan3x3>; lambda: number }) {
  const costs = Object.fromEntries(micro.decision.table.map((r) => [r.action, r.expected])) as Record<Action, number>;
  const win = micro.decision.action;
  const g = micro.lattice;
  return (
    <div className="font-mono text-[0.7rem] tabular-nums">
      <p className="eyebrow mb-1">the §3 3×3 example at this λ</p>
      <div className="flex items-start gap-3">
        <svg width={78} height={78} role="img" aria-label="A three-by-three grid: goal at the top middle, an obstacle at the middle left, cost-to-go numbers in each free cell, and two belief cells on the bottom row.">
          {Array.from({ length: 9 }, (_, c) => {
            const [i, j] = cellIJ(g, c);
            const row = g.ny - 1 - j;
            const x = i * 26;
            const y = row * 26;
            const occ = g.occupied[c] === 1;
            const isGoal = c === micro.J.goal;
            const sample = micro.samples.find((s) => s.cell === c);
            return (
              <g key={c}>
                <rect x={x} y={y} width={26} height={26} fill={occ ? 'var(--rm-obstacle)' : isGoal ? 'var(--rm-goal)' : sample ? 'var(--pr-posterior)' : 'var(--rm-graph)'} fillOpacity={occ ? 0.5 : isGoal ? 0.55 : sample ? 0.25 + 0.6 * sample.weight : 0.12} stroke="var(--pr-grid)" />
                <text x={x + 13} y={y + 17} textAnchor="middle" fontSize={10} fill="var(--pr-canvas-ink)">
                  {occ ? 'λ' : micro.Jtable[row][i].toFixed(0)}
                </text>
                {sample ? (
                  <text x={x + 13} y={y + 25} textAnchor="middle" fontSize={6.5} fill="var(--pr-posterior)">
                    ω={sample.weight.toFixed(1)}
                  </text>
                ) : null}
              </g>
            );
          })}
        </svg>
        <table>
          <tbody>
            {(['Up', 'Right', 'Left', 'Down'] as Action[]).map((a) => (
              <tr key={a} style={win === a ? { color: 'var(--rm-path)', fontWeight: 600 } : undefined}>
                <td className="pr-2">{win === a ? '▶ ' : ''}{a}</td>
                <td className="text-right">{costs[a].toFixed(2)}</td>
              </tr>
            ))}
            <tr className="text-fd-muted-foreground">
              <td className="pr-2">mode says</td>
              <td className="text-right">{micro.modeDecision.action}</td>
            </tr>
            <tr className="text-fd-muted-foreground">
              <td className="pr-2">λ*</td>
              <td className="text-right">{micro.breakEven?.toFixed(1) ?? '—'}</td>
            </tr>
          </tbody>
        </table>
      </div>
      <p className="mt-1 text-fd-muted-foreground">λ = {lambda.toFixed(1)} {lambda > 6 ? '> 6: sideways first' : lambda < 6 ? '< 6: gamble on the mode' : '= 6: tie'}</p>
    </div>
  );
}
