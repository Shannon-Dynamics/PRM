'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { label, type Palette, type Viewport } from '@/lib/sim/draw';
import { torusLab, rasterTorusCObstacle } from '@/lib/sampling/worlds';
import { APARTMENT } from '@/lib/sim/world';
import { LOT } from '@/lib/robots/worlds';
import { Mission } from '@/lib/capstone/mission';
import { GUARANTEE_NAMES } from '@/lib/capstone/stage';
import { budgetLine } from '@/lib/capstone/budget';
import { SIGMA_MAP } from '@/lib/capstone/rusty';
import { MU_MIN, QDOT_MAX } from '@/lib/capstone/reach';
import { PHI_MAX, PHI_OK, PHI_PLAN, PHI_WARN } from '@/lib/capstone/hitch';
import { armConditioning } from '@/lib/capstone/guards';
import { REACH_TASKS } from '@/lib/capstone/worlds';
import { BENCH_WORLD, Bar, ModeMachine, Panel, Row, eventColor, paintApartment, paintLot, paintPhiGauge, paintWorkbench } from './shared';

/**
 * w23.1 — The Grand Plan.
 *
 * Three stacks on one round-robin clock: Rusty replanning on a belief in the
 * Apartment, Reach planning, time-scaling and smoothing on the Workbench,
 * Hitch towing a trailer into bay 3 of the Lot. Nothing is recorded: every
 * frame advances the real `Mission` by about one 50 ms tick, and every number
 * in the inspectors is read out of the running stages. The chaos buttons are
 * the Failure Tour's three injections, live.
 */

const TABS = ['Belief', 'Search', 'Tree', 'Phase plane', 'Guards', 'Timing'] as const;
type Tab = (typeof TABS)[number];

interface State {
  mission: Mission;
}

export function GrandPlan() {
  const [truth, setTruth] = useState(false);
  const [learned, setLearned] = useState(false);
  const [tab, setTab] = useState<Tab>('Belief');

  const learnedRef = useRef(false);
  learnedRef.current = learned;
  const init = useCallback((seed: number): State => ({ mission: new Mission({ seed, tMaxMs: 180000 }) }), []);
  const step = useCallback((s: State): State => {
    // Chapter 22's toggle applies from the next pick the guard admits.
    if (s.mission.reach) s.mission.reach.learnedLambda = learnedRef.current ? 0.3 : null;
    if (!s.mission.finished) s.mission.step();
    return { ...s };
  }, []);
  const sim = useSimulation<State>({ init, step, fps: 20, initialSeed: 7, loop: false });
  const m = sim.state.mission;
  const r = m.rusty!;
  const a = m.reach!;
  const h = m.hitch!;

  // The torus C-obstacle is a property of the Workbench, not of the mission: compute it once.
  const raster = useMemo(() => rasterTorusCObstacle(torusLab(), 72), []);

  const drawApt = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => paintApartment(ctx, v, p, r, { truth, scan: true, touched: tab === 'Search', particles: tab === 'Belief' }),
    [r, truth, tab],
  );
  const drawBench = useCallback((ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => paintWorkbench(ctx, v, p, a, raster, tab === 'Tree' || a.phase === 'rrtstar'), [a, raster, tab]);
  const drawLot = useCallback((ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => paintLot(ctx, v, p, h, tab === 'Search' || tab === 'Tree'), [h, tab]);

  const events = m.events();
  const drawTimeline = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      ctx.clearRect(0, 0, v.width, v.height);
      ctx.fillStyle = p.bg;
      ctx.fillRect(0, 0, v.width, v.height);
      const tMax = Math.max(30000, m.now * 1.1);
      const X = (t: number) => 56 + (t / tMax) * (v.width - 64);
      const lanes = ['rusty', 'reach', 'hitch'] as const;
      const laneY = (i: number) => 12 + i * ((v.height - 18) / 3);
      lanes.forEach((name, i) => {
        label(ctx, name, 4, laneY(i) + 8, p.ink, { size: 9 });
        ctx.strokeStyle = p.grid;
        ctx.beginPath();
        ctx.moveTo(X(0), laneY(i) + 5);
        ctx.lineTo(X(tMax), laneY(i) + 5);
        ctx.stroke();
      });
      for (const e of events) {
        if (e.kind === 'info') continue;
        const i = lanes.indexOf(e.robot);
        ctx.fillStyle = eventColor(p, e.kind);
        ctx.fillRect(X(e.t) - 1.5, laneY(i), 3, 10);
      }
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(X(m.now), 2);
      ctx.lineTo(X(m.now), v.height - 2);
      ctx.stroke();
      label(ctx, `t = ${(m.now / 1000).toFixed(1)} s`, X(m.now) + 4, v.height - 4, p.ink, { size: 9 });
    },
    [m, events],
  );

  const now = m.now;
  const bel = r.beliefMsg();
  const err = Math.hypot(bel.pose.x - r.truth.x, bel.pose.y - r.truth.y);
  const plan = r.lastPlan;
  const recent = events.filter((e) => e.kind !== 'info').slice(-6).reverse();
  const cond = armConditioning(a.arm, a.q);
  const stats = m.rr.allStats();

  return (
    <WidgetFrame
      id="w23.1"
      title="The Grand Plan"
      teaches="An autonomy demo is not one clever algorithm: it is three chains of stated contracts, and every failure is a contract violation you can point at."
      colorKey={['start', 'goal', 'obstacle', 'cobstacle', 'graph', 'path', 'robot', 'posterior']}
      wide
      caption={
        <>
          Left, Rusty plans from its <em>belief</em> (the purple ellipse, 3σ) on a grid of amber C-obstacles that it
          grows only when two scans agree; the purple line is D* Lite&apos;s path and the orange stub is the MPC
          rollout. Centre, Reach&apos;s workspace and its torus T² side by side: the blue tree is Informed RRT*, the
          solid purple curve the shortcut path, the dashed one CHOMP mid-flight, and the strip beneath is the phase
          plane (s, ṡ) of the trajectory being executed against its amber velocity-limit curve. Right, Hitch and its
          trailer heading for the red outline in bay 3; blue dots are hybrid A* expansions, circles are cusps, dashed
          red curves are paths the trailer filter rejected. Press the three chaos buttons and watch the mode
          machines on the left: a door shuts in front of Rusty, Reach&apos;s next pick moves to full stretch, Hitch
          reverses at full lock. Each recovery is a statistic crossing a threshold — open the Guards tab to see which.
        </>
      }
    >
      <div className="grid grid-cols-1 border-b border-fd-border md:grid-cols-3">
        <div className="min-w-0 border-b border-fd-border md:border-b-0 md:border-r">
          <SimCanvas world={APARTMENT.bounds} draw={drawApt} deps={[sim.tick, truth, tab]} aspect={12 / 9} padding={0.2} ariaLabel="Rusty in the Apartment: blueprint walls, amber inflated cells, a purple D* Lite path, an orange rollout, and Rusty's belief ellipse." />
          <p className="px-2 pb-1 font-mono text-[0.62rem] text-fd-muted-foreground">Apartment · Rusty · D* Lite on a Ch. 16 belief</p>
        </div>
        <div className="min-w-0 border-b border-fd-border md:border-b-0 md:border-r">
          <SimCanvas world={BENCH_WORLD} draw={drawBench} deps={[sim.tick, tab]} aspect={2 / 1.36} padding={0} ariaLabel="Reach on the Workbench: the arm in its workspace, the torus of configurations with amber C-obstacles, a blue tree, purple paths, and a phase-plane strip." />
          <p className="px-2 pb-1 font-mono text-[0.62rem] text-fd-muted-foreground">Workbench · Reach · RRT* → time scaling → CHOMP → verify</p>
        </div>
        <div className="min-w-0">
          <SimCanvas world={LOT.bounds} draw={drawLot} deps={[sim.tick, tab]} aspect={16 / 10} padding={0.2} ariaLabel="Hitch and its trailer in the Lot: a purple hybrid A* path with cusps, blue expansions, and the goal footprint in bay 3." />
          <p className="px-2 pb-1 font-mono text-[0.62rem] text-fd-muted-foreground">Lot · Hitch + trailer · hybrid A* with a jackknife guard</p>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-[230px_minmax(0,1fr)]">
        <div className="border-b border-fd-border lg:border-b-0 lg:border-r">
          <Panel title="mode machines (D23.4)">
            <ModeMachine name="Rusty" mode={r.mode} detail={`v ${r.v.toFixed(2)} m/s`} />
            <ModeMachine name="Reach" mode={a.mode} detail={`pick ${Math.min(a.records.length + 1, a.totalTasks)}/${a.totalTasks} · ${a.phase}`} />
            <ModeMachine name="Hitch" mode={h.mode} detail={`φ ${h.phi.toFixed(2)} rad`} />
          </Panel>
          <Panel title="chaos">
            <ButtonRow>
              <ActionButton onClick={() => sim.setState((s) => (s.mission.rusty?.closeDoor(s.mission.now), { ...s }))}>Close Door</ActionButton>
              <ActionButton onClick={() => sim.setState((s) => (s.mission.reach?.pushSingular(s.mission.now), { ...s }))}>Singular Goal</ActionButton>
              <ActionButton onClick={() => sim.setState((s) => (s.mission.hitch?.reverseHard(s.mission.now), { ...s }))}>Reverse Hard</ActionButton>
            </ButtonRow>
          </Panel>
          <Panel title="toggles">
            <div className="space-y-1.5">
              <Toggle label="ground truth" checked={truth} onChange={setTruth} role="truth" />
              <Toggle label="Ch. 22 learned sampler (λ = 0.3)" checked={learned} onChange={setLearned} role="graph" />
            </div>
          </Panel>
          <Panel title="latest events">
            <ul className="space-y-0.5 font-mono text-[0.62rem] leading-snug">
              {recent.length === 0 ? <li className="text-fd-muted-foreground">nothing yet</li> : null}
              {recent.map((e, i) => (
                <li key={`${e.t}-${i}`}>
                  <span className="text-fd-muted-foreground">{(e.t / 1000).toFixed(1)}s {e.robot}</span> {e.text}
                </li>
              ))}
            </ul>
          </Panel>
        </div>

        <div className="min-w-0">
          <div className="flex flex-wrap border-b border-fd-border">
            {TABS.map((t) => (
              <button
                key={t}
                type="button"
                onClick={() => setTab(t)}
                aria-pressed={tab === t}
                className={`px-2.5 py-1.5 font-ui text-[0.7rem] font-medium transition-colors ${tab === t ? 'bg-fd-accent text-fd-foreground' : 'text-fd-muted-foreground hover:bg-fd-accent/50'}`}
              >
                {t}
              </button>
            ))}
          </div>

          {tab === 'Belief' ? (
            <div className="grid grid-cols-1 sm:grid-cols-2">
              <Panel title="Ch. 16 particle belief (map frame)">
                <dl className="space-y-1">
                  <Row k="particles" v={String(r.belief.size)} />
                  <Row k="σ_pose (√λ_max)" v={`${bel.sigma.toFixed(3)} m`} color="var(--pr-posterior)" />
                  <Row k="|belief − truth|" v={`${err.toFixed(3)} m`} color="var(--pr-truth)" />
                  <Row k="map trusts scans" v={bel.sigma <= SIGMA_MAP ? `yes (σ ≤ ${SIGMA_MAP})` : 'no — belief too loose'} />
                  <Row k="confirmed cells" v={String(r.confirmed.size)} />
                </dl>
              </Panel>
              <Panel title="replan_on_belief on D* Lite's field">
                {plan?.belief ? (
                  <ul className="space-y-1">
                    {plan.belief.table.map((row) => {
                      const best = Math.min(...plan.belief!.table.map((t) => t.total));
                      const worst = Math.max(...plan.belief!.table.map((t) => t.total));
                      const isMode = row.action === plan.modeAction;
                      return (
                        <li key={row.action} className="space-y-0.5">
                          <div className="flex justify-between font-mono text-[0.62rem] tabular-nums">
                            <span style={{ fontWeight: row.total === best ? 700 : 400 }}>
                              {row.total === best ? '★ ' : ''}
                              {row.action}
                              {isMode ? ' (D* step)' : ''}
                            </span>
                            <span className="opacity-70">
                              {row.expected.toFixed(2)} · hit {(row.collisionMass * 100).toFixed(0)}%
                            </span>
                          </div>
                          <Bar frac={(row.total - best) / Math.max(1e-9, worst - best)} color="var(--pr-posterior)" />
                        </li>
                      );
                    })}
                  </ul>
                ) : (
                  <p className="font-ui text-[0.7rem] text-fd-muted-foreground">Waiting for the first plan.</p>
                )}
              </Panel>
            </div>
          ) : null}

          {tab === 'Search' ? (
            <div className="grid grid-cols-1 sm:grid-cols-2">
              <Panel title="Ch. 6 D* Lite (blue = touched by the last repair)">
                <dl className="space-y-1">
                  <Row k="grid" v={`${r.grid.nx} × ${r.grid.ny} @ ${r.grid.cellSize} m`} />
                  <Row k="g(start)" v={plan ? `${plan.cost.toFixed(2)} m` : '—'} />
                  <Row k="last repair: touched" v={plan ? String(plan.touched) : '—'} color="var(--rm-graph)" />
                  <Row k="last repair: pops" v={plan ? String(plan.pops) : '—'} />
                  <Row k="repairs / full searches" v={`${r.repairs} / ${r.fullReplans}`} />
                  <Row k="path (shortcut)" v={`${r.path.length} vertices`} color="var(--rm-path)" />
                </dl>
              </Panel>
              <Panel title="Ch. 21 hybrid A* (Lot)">
                <dl className="space-y-1">
                  <Row k="plan" v={h.plan ? `${h.plan.samples.length} samples, ${h.plan.cusps} cusps` : '—'} color="var(--rm-path)" />
                  <Row k="expansions" v={h.plan ? String(h.plan.searches.reduce((n, s) => n + s.expansions, 0)) : '—'} color="var(--rm-graph)" />
                  <Row k="RS shots tried" v={h.plan ? String(h.plan.searches.reduce((n, s) => n + s.shots.length, 0)) : '—'} />
                  <Row k="ladder rung" v={h.plan ? `${h.plan.attempt + 1}${h.plan.via ? ' (staged)' : ''}` : '—'} />
                  <Row k="rejected by trailer filter" v={String(h.rejected.length)} color="var(--rm-goal)" />
                  <Row k="max |φ| on plan" v={h.plan ? `${h.plan.maxHitch.toFixed(2)} rad ≤ ${PHI_PLAN}` : '—'} />
                </dl>
              </Panel>
            </div>
          ) : null}

          {tab === 'Tree' ? (
            <div className="grid grid-cols-1 sm:grid-cols-2">
              <Panel title="Chs. 12–13 on T² (current pick)">
                <dl className="space-y-1">
                  <Row k="pick" v={REACH_TASKS[a.taskIndex % REACH_TASKS.length].name} />
                  <Row k="RRT-Connect" v={a.connect ? `${a.connect.attempts} attempts, ${a.connect.nodeCount} nodes${a.connect.solved ? ', merged' : ''}` : '—'} />
                  <Row k="Informed RRT*" v={a.rrt ? `${a.rrt.iterations} it, ${a.rrt.tree.size} nodes` : '—'} color="var(--rm-graph)" />
                  <Row k="best RRT* cost" v={a.rrt && a.rrt.solved ? `${a.rrt.bestCost.toFixed(2)} rad` : '—'} />
                  <Row k="after shortcut" v={a.shortPath ? `${(a.shortPath.length - 1)} edges` : '—'} color="var(--rm-path)" />
                  <Row k="sampler" v={a.learnedLambda !== null ? `learned, λ = ${a.learnedLambda}` : 'uniform'} />
                </dl>
              </Panel>
              <Panel title="picks so far">
                <ul className="space-y-1 font-mono text-[0.62rem] tabular-nums">
                  {a.records.length === 0 ? <li className="text-fd-muted-foreground">none yet</li> : null}
                  {a.records.map((rec) => (
                    <li key={rec.task}>
                      {rec.task + 1}. {rec.cost.toFixed(2)} rad · {rec.durationBefore.toFixed(2)} → {rec.durationAfter.toFixed(2)} s {rec.chompAccepted ? '(CHOMP)' : '(fallback)'}
                      {rec.guardFired ? ' · guard fired' : ''}
                    </li>
                  ))}
                </ul>
              </Panel>
            </div>
          ) : null}

          {tab === 'Phase plane' ? (
            <div className="grid grid-cols-1 sm:grid-cols-2">
              <Panel title="Ch. 18 time scaling (current pick)">
                <dl className="space-y-1">
                  <Row k="T₁ (shortcut)" v={a.traj1 ? `${a.traj1.tf.toFixed(3)} s` : '—'} />
                  <Row k="T₂ (after CHOMP)" v={a.traj2 ? `${a.traj2.tf.toFixed(3)} s` : '—'} />
                  <Row k="switches" v={a.scaling2 ? String(a.scaling2.switches.length) : a.scaling1 ? String(a.scaling1.switches.length) : '—'} />
                  <Row k="executing" v={a.executing ? (a.executing.kind === 'Trajectory' ? a.executing.source : 'Path (refused)') : '—'} />
                  <Row k="sample" v={a.executing && a.executing.kind === 'Trajectory' ? `${Math.min(a.execIndex, a.executing.traj.samples.length - 1)} / ${a.executing.traj.samples.length - 1}` : '—'} />
                </dl>
              </Panel>
              <Panel title="what the strip shows">
                <p className="font-ui text-[0.68rem] leading-snug text-fd-muted-foreground">
                  The amber curve is ṡ_max(s), the fastest the path can be traversed at each s without exceeding
                  |τ| ≤ (20, 12) N·m. The purple curve is the time-optimal profile: maximum acceleration until it
                  must switch to maximum deceleration to land on the next valley of the limit curve. The orange dot is
                  where the arm is now.
                </p>
              </Panel>
            </div>
          ) : null}

          {tab === 'Guards' ? (
            <div className="grid grid-cols-1 sm:grid-cols-3">
              <Panel title="F3 · singularity (Reach)">
                <dl className="space-y-1">
                  <Row k="μ(q) now" v={cond.mu.toFixed(3)} />
                  <Row k="μ(q_goal)" v={a.qGoal ? a.mu.toFixed(3) : '—'} />
                  <Row k="σ_min(J(q_goal))" v={a.qGoal ? a.sigmaMin.toFixed(3) : '—'} />
                </dl>
                <div className="mt-1.5">
                  <Bar frac={a.mu / 1} mark={MU_MIN} color="var(--rm-robot)" warn={a.mu < MU_MIN} />
                  <p className="mt-1 font-mono text-[0.6rem] text-fd-muted-foreground">μ_min = {MU_MIN}, |q̇| ≤ {QDOT_MAX} rad/s</p>
                </div>
              </Panel>
              <Panel title="F4 · jackknife (Hitch)">
                <SimCanvas
                  world={{ minX: 0, minY: 0, maxX: 1, maxY: 0.5 }}
                  draw={(ctx, v, p) => {
                    ctx.clearRect(0, 0, v.width, v.height);
                    paintPhiGauge(ctx, v.width / 2, v.height - 6, Math.min(v.width / 2 - 12, v.height - 14), p, h.phi, PHI_PLAN);
                  }}
                  deps={[sim.tick]}
                  aspect={2}
                  padding={0}
                  ariaLabel="Hitch-angle gauge with the plan, warn and hard bounds."
                />
                <dl className="space-y-1">
                  <Row k="φ = θ − ψ" v={`${h.phi.toFixed(3)} rad`} color="var(--rm-robot)" />
                  <Row k="φ_plan / φ_warn / φ_max" v={`${PHI_PLAN} / ${PHI_WARN} / ${PHI_MAX}`} />
                  <Row k="recover until |φ| <" v={String(PHI_OK)} />
                </dl>
              </Panel>
              <Panel title="D23.2 · validity (Rusty)">
                <dl className="space-y-1">
                  <Row k="L_valid" v={`${r.lValid.toFixed(2)} m`} />
                  <Row k="blocked ahead" v={r.blocked ? `${r.blocked.at.toFixed(2)} m` : 'clear'} color={r.blocked ? 'var(--rm-goal)' : undefined} />
                  <Row k="σ gate" v={`${bel.sigma.toFixed(3)} ≤ ${SIGMA_MAP}`} />
                  <Row k="replans" v={String(r.replans)} />
                </dl>
              </Panel>
            </div>
          ) : null}

          {tab === 'Timing' ? (
            <div className="grid grid-cols-1 sm:grid-cols-[minmax(0,1fr)_240px]">
              <Panel title="measured input staleness vs. period (one round-robin, 20 Hz base)">
                <ul className="space-y-1">
                  {m.stages().map((st) => {
                    const s = stats[st.id];
                    return (
                      <li key={st.id} className="grid grid-cols-[140px_minmax(0,1fr)_70px] items-center gap-2">
                        <span className="truncate font-mono text-[0.6rem]">{st.id}</span>
                        <Bar frac={s ? s.lastInputAge / Math.max(st.periodMs, 1) / 4 : 0} mark={0.25} color="var(--pr-widget-ink)" />
                        <span className="text-right font-mono text-[0.6rem] tabular-nums text-fd-muted-foreground">
                          {s ? `${s.lastInputAge} / ${st.periodMs} ms` : '—'}
                        </span>
                      </li>
                    );
                  })}
                </ul>
              </Panel>
              <Panel title="F2 budget and D23.3">
                <p className="mb-1.5 font-mono text-[0.6rem] leading-snug">{budgetLine(m.budget)}</p>
                <dl className="space-y-1">
                  <Row k="Rusty 𝒢" v={GUARANTEE_NAMES[m.composed('rusty')]} />
                  <Row k="Reach 𝒢" v={GUARANTEE_NAMES[m.composed('reach')]} />
                  <Row k="Hitch 𝒢" v={GUARANTEE_NAMES[m.composed('hitch')]} />
                  <Row k="sim time" v={`${(now / 1000).toFixed(2)} s`} />
                </dl>
              </Panel>
            </div>
          ) : null}
        </div>
      </div>

      <div className="border-t border-fd-border">
        <SimCanvas world={{ minX: 0, minY: 0, maxX: 1, maxY: 1 }} draw={drawTimeline} deps={[sim.tick]} aspect={9} padding={0} ariaLabel="Shared mission timeline with one lane per robot and colored pins for chaos, detections, recoveries, completions and failures." />
      </div>
      <Transport {...sim} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={sim.reseed} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
