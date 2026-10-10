'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { drawPath, drawRobot, drawSegments, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { paintArmMapped, paintPolyWorld, rectMapper } from '@/lib/sampling/paint';
import { APARTMENT } from '@/lib/sim/world';
import { LOT, WORKBENCH } from '@/lib/robots/worlds';
import { cellCenter } from '@/lib/search/grid';
import { DEFAULT_RATES } from '@/lib/capstone/budget';
import { MissionRecorder, type MissionCfg, type MissionEvent } from '@/lib/capstone/mission';
import { MU_MIN } from '@/lib/capstone/reach';
import { PHI_MAX, PHI_WARN, PHI_OK } from '@/lib/capstone/hitch';
import { BLUEPRINT } from '@/lib/capstone/worlds';
import { eventColor, paintHitchCfg } from './shared';

/**
 * w23.3 — Failure Tour.
 *
 * Three recorded missions, one per contract violation, each scrubbable:
 *
 *   Blocked Corridor  the door Rusty is heading for shuts; two scans confirm it,
 *                     the grid inflates the new cells, D* Lite repairs only
 *                     what changed, the MPC follows the new path. The variant
 *                     throttles the replanner to 0.5 Hz, and the D23.2 check
 *                     is what stops Rusty short.
 *   Singular Arm      the next pick moves to full stretch; μ(q_goal) dives
 *                     below μ_min, Recover(PullGoalInward) moves it in by δ,
 *                     the joint-space pipeline replans unchanged.
 *   Jackknife         full reverse at full lock; |φ| grows like e^{|v|t/d},
 *                     crosses φ_warn, Recover(PullForward) straightens it,
 *                     hybrid A* replans through a staging pose, Hitch parks.
 *
 * Recording is time-sliced (three ticks per frame) and playback can be scrubbed
 * over whatever has been recorded so far.
 */

type TabId = 'corridor' | 'arm' | 'jackknife';
const TABS: { id: TabId; name: string }[] = [
  { id: 'corridor', name: 'Blocked Corridor' },
  { id: 'arm', name: 'Singular Arm' },
  { id: 'jackknife', name: 'Jackknife' },
];

function cfgFor(tab: TabId, throttled: boolean): MissionCfg {
  switch (tab) {
    case 'corridor':
      return { seed: 7, robots: ['rusty'], chaos: [{ kind: 'close-door', t: 9000 }], rates: throttled ? { ...DEFAULT_RATES, planHz: 0.5 } : DEFAULT_RATES, tMaxMs: 45000 };
    case 'arm':
      return { seed: 7, robots: ['reach'], chaos: [{ kind: 'singular-goal', t: 4000 }], tMaxMs: 20000 };
    case 'jackknife':
      return { seed: 7, robots: ['hitch'], chaos: [{ kind: 'reverse-hard', t: 2500 }], tMaxMs: 30000 };
  }
}

interface State {
  rec: MissionRecorder;
  head: number;
}

export function FailureTour() {
  const [tab, setTab] = useState<TabId>('corridor');
  const [throttled, setThrottled] = useState(false);
  const init = useCallback((): State => ({ rec: new MissionRecorder(cfgFor(tab, throttled)), head: 0 }), [tab, throttled]);
  const step = useCallback((s: State): State => {
    if (!s.rec.done) s.rec.advance(3);
    return { rec: s.rec, head: Math.min(s.head + 1, s.rec.frames.length - 1) };
  }, []);
  const sim = useSimulation<State>({ init, step, fps: 20, initialSeed: 7, loop: false });
  const first = useRef(true);
  const { reset } = sim;
  useEffect(() => {
    if (first.current) {
      first.current = false;
      return;
    }
    reset();
  }, [reset]);
  const { rec, head } = sim.state;
  const frame = rec.frames[head];
  const events: MissionEvent[] = rec.mission.events().filter((e) => e.kind !== 'info' || /repaired|hybrid A\*|recovered/.test(e.text));

  const scrub = (k: number) => {
    sim.pause();
    sim.setState((s) => ({ rec: s.rec, head: Math.max(0, Math.min(k, s.rec.frames.length - 1)) }));
  };

  const drawWorld = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      ctx.clearRect(0, 0, v.width, v.height);
      ctx.fillStyle = p.bg;
      ctx.fillRect(0, 0, v.width, v.height);
      if (tab === 'corridor' && frame.rusty && rec.mission.rusty) {
        const r = rec.mission.rusty;
        const f = frame.rusty;
        const confirmed = [...r.confirmed].slice(0, f.confirmed);
        ctx.save();
        ctx.fillStyle = p.cobstacle;
        for (const c of confirmed) {
          const q = cellCenter(r.grid, c);
          ctx.fillRect(sx(v, q.x - 0.05), sy(v, q.y + 0.05), sl(v, 0.1) + 0.5, sl(v, 0.1) + 0.5);
        }
        ctx.restore();
        drawSegments(ctx, v, BLUEPRINT.walls, p.obstacle, 2);
        if (f.doorClosed && r.shutDoor) {
          ctx.save();
          ctx.setLineDash([3, 3]);
          drawSegments(ctx, v, [r.shutDoor], p.truth, 3);
          ctx.restore();
        }
        drawPath(ctx, v, r.trail.slice(0, f.trail), p.robot, { lineWidth: 1.6, alpha: 0.75 });
        if (f.path.length > 1) drawPath(ctx, v, f.path, p.path, { lineWidth: 2.2 });
        drawRobot(ctx, v, f.belief, p.robot, 0.35);
        ctx.fillStyle = p.goal;
        ctx.beginPath();
        ctx.arc(sx(v, r.goal.x), sy(v, r.goal.y), 5, 0, 2 * Math.PI);
        ctx.fill();
        label(ctx, f.mode, 8, 16, f.mode === 'Replan' ? p.cobstacle : p.ink, { size: 11, weight: 700 });
      } else if (tab === 'arm' && frame.reach && rec.mission.reach) {
        const a = rec.mission.reach;
        const f = frame.reach;
        const m = rectMapper(v, { x0: v.minX, y0: v.minY, x1: v.maxX, y1: v.maxY }, WORKBENCH.bounds);
        paintPolyWorld(ctx, m, WORKBENCH, p);
        ctx.save();
        ctx.strokeStyle = p.grid;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.arc(m.X(0), m.Y(0), m.S(2), 0, 2 * Math.PI);
        ctx.stroke();
        ctx.restore();
        ctx.fillStyle = p.goal;
        ctx.beginPath();
        ctx.arc(m.X(f.tip.x), m.Y(f.tip.y), 5, 0, 2 * Math.PI);
        ctx.fill();
        paintArmMapped(ctx, m, p, a.arm, f.q, p.robot, 1);
        label(ctx, `${f.mode} · ${f.phase}`, 8, 16, f.mode.startsWith('Recover') ? p.cobstacle : p.ink, { size: 11, weight: 700 });
      } else if (tab === 'jackknife' && frame.hitch && rec.mission.hitch) {
        const h = rec.mission.hitch;
        const f = frame.hitch;
        if (LOT.markings) drawSegments(ctx, v, LOT.markings, p.grid, 1.5);
        drawSegments(ctx, v, LOT.walls, p.wall, 2.2);
        ctx.fillStyle = p.obstacle;
        for (const o of LOT.obstacles) {
          ctx.beginPath();
          o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
          ctx.closePath();
          ctx.fill();
        }
        paintHitchCfg(ctx, v, h, h.car.cfg(h.goal.x, h.goal.y, h.goal.theta), p.goal, 0.9, true);
        if (f.plan.length > 1) drawPath(ctx, v, f.plan, p.path, { lineWidth: 2 });
        drawPath(ctx, v, h.executed.slice(0, f.executed).map((q) => q.body), p.robot, { lineWidth: 1.6, alpha: 0.75 });
        paintHitchCfg(ctx, v, h, f.q, Math.abs(f.phi) > PHI_WARN ? p.goal : p.robot, 0.85);
        label(ctx, f.mode, 8, 16, f.mode.startsWith('Recover') ? p.cobstacle : p.ink, { size: 11, weight: 700 });
      }
    },
    [tab, frame, rec],
  );

  /** The statistic that crosses a threshold, over the recorded frames, with the playhead. */
  const drawStat = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      ctx.clearRect(0, 0, v.width, v.height);
      ctx.fillStyle = p.bg;
      ctx.fillRect(0, 0, v.width, v.height);
      const frames = rec.frames;
      const tEnd = Math.max(frames[frames.length - 1].t, 10000);
      const X = (t: number) => 34 + (t / tEnd) * (v.width - 42);
      const pad = 14;
      let series: { y: (k: number) => number; color: string; max: number; name: string }[] = [];
      let thresholds: { value: number; color: string; name: string }[] = [];
      if (tab === 'corridor') {
        series = [
          { y: (k) => Math.abs(frames[k].rusty?.v ?? 0), color: p.robot, max: 0.7, name: '|v| (m/s)' },
          { y: (k) => (frames[k].rusty?.confirmed ?? 0) / 60, color: p.cobstacle, max: 0.7, name: 'confirmed cells / 60' },
        ];
      } else if (tab === 'arm') {
        series = [{ y: (k) => frames[k].reach?.mu ?? 0, color: p.robot, max: 1.05, name: 'μ(q_goal)' }];
        thresholds = [{ value: MU_MIN, color: p.goal, name: 'μ_min' }];
      } else {
        series = [{ y: (k) => Math.abs(frames[k].hitch?.phi ?? 0), color: p.robot, max: 1.4, name: '|φ| (rad)' }];
        thresholds = [
          { value: PHI_OK, color: p.start, name: 'φ_ok' },
          { value: PHI_WARN, color: p.cobstacle, name: 'φ_warn' },
          { value: PHI_MAX, color: p.goal, name: 'φ_max' },
        ];
      }
      const max = Math.max(...series.map((s) => s.max));
      const Y = (y: number) => v.height - pad - (Math.min(y, max) / max) * (v.height - 2 * pad);
      for (const th of thresholds) {
        ctx.save();
        ctx.strokeStyle = th.color;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.moveTo(X(0), Y(th.value));
        ctx.lineTo(X(tEnd), Y(th.value));
        ctx.stroke();
        ctx.restore();
        label(ctx, th.name, X(tEnd) - 40, Y(th.value) - 3, th.color, { size: 9 });
      }
      for (const e of rec.mission.events()) {
        if (e.kind === 'info' || e.kind === 'mode') continue;
        ctx.fillStyle = eventColor(p, e.kind);
        ctx.fillRect(X(e.t) - 1, 2, 2, 8);
      }
      series.forEach((s, si) => {
        ctx.save();
        ctx.strokeStyle = s.color;
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        for (let k = 0; k < frames.length; k++) (k === 0 ? ctx.moveTo : ctx.lineTo).call(ctx, X(frames[k].t), Y(s.y(k)));
        ctx.stroke();
        ctx.restore();
        label(ctx, s.name, 38 + si * 150, v.height - 2, s.color, { size: 9 });
      });
      ctx.strokeStyle = p.ink;
      ctx.beginPath();
      ctx.moveTo(X(frame.t), 0);
      ctx.lineTo(X(frame.t), v.height);
      ctx.stroke();
    },
    [tab, rec, frame],
  );

  const shown = events.filter((e) => e.t <= frame.t).slice(-7).reverse();

  return (
    <WidgetFrame
      id="w23.3"
      title="Failure Tour"
      teaches="Recovery is not magic: each one is a named statistic crossing a threshold, followed by a mode with a plan."
      colorKey={['path', 'robot', 'cobstacle', 'goal', 'start']}
      wide
      caption={
        <>
          Each tab is one recorded mission with one injection. Scrub the slider to the moment the statistic in the lower
          plot crosses its dashed threshold and read the mode the supervisor switched to. In <em>Blocked Corridor</em>,
          the amber cells are the only knowledge Rusty has of the shut door — two consecutive scans, projected through
          the belief — and the repair touches a few hundred vertices, not the grid. Tick <em>throttle replanner</em>
          to run D* Lite at 0.5 Hz: L_valid grows to 1.32 m and the D23.2 check, not the planner, is what brakes Rusty.
        </>
      }
    >
      <div className="flex flex-wrap items-center gap-1 border-b border-fd-border px-2 py-1.5">
        {TABS.map((t) => (
          <button
            key={t.id}
            type="button"
            onClick={() => setTab(t.id)}
            aria-pressed={tab === t.id}
            className={`rounded-sm px-2.5 py-1 font-ui text-[0.72rem] font-medium ${tab === t.id ? 'bg-fd-accent text-fd-foreground' : 'text-fd-muted-foreground hover:bg-fd-accent/50'}`}
          >
            {t.name}
          </button>
        ))}
        {tab === 'corridor' ? (
          <span className="ml-auto">
            <Toggle label="throttle replanner to 0.5 Hz" checked={throttled} onChange={setThrottled} role="path" />
          </span>
        ) : null}
      </div>
      <div className="grid grid-cols-1 md:grid-cols-[minmax(0,1fr)_260px]">
        <div className="min-w-0 border-b border-fd-border md:border-b-0 md:border-r">
          <SimCanvas
            world={tab === 'corridor' ? APARTMENT.bounds : tab === 'jackknife' ? LOT.bounds : { minX: 0, minY: 0, maxX: 1, maxY: 1 }}
            draw={drawWorld}
            deps={[head, tab, rec]}
            aspect={tab === 'corridor' ? 12 / 9 : tab === 'jackknife' ? 16 / 10 : 1.4}
            padding={0.2}
            ariaLabel="The recorded mission at the playhead."
          />
          <SimCanvas world={{ minX: 0, minY: 0, maxX: 1, maxY: 1 }} draw={drawStat} deps={[head, tab, rec]} aspect={5} padding={0} ariaLabel="The guard statistic over time against its threshold, with event pins and the playhead." />
          <div className="px-3 py-1.5">
            <input
              type="range"
              min={0}
              max={Math.max(0, rec.frames.length - 1)}
              value={head}
              onChange={(e) => scrub(Number(e.target.value))}
              className="h-1 w-full"
              aria-label="Scrub the recorded mission"
            />
            <div className="flex justify-between font-mono text-[0.62rem] text-fd-muted-foreground">
              <span>t = {(frame.t / 1000).toFixed(2)} s</span>
              <span>{rec.done ? `recorded ${(rec.frames[rec.frames.length - 1].t / 1000).toFixed(1)} s` : 'recording…'}</span>
            </div>
          </div>
        </div>
        <div className="px-3 py-2">
          <p className="eyebrow mb-1">events up to the playhead</p>
          <ul className="space-y-1 font-mono text-[0.64rem] leading-snug">
            {shown.length === 0 ? <li className="text-fd-muted-foreground">nothing yet</li> : null}
            {shown.map((e, i) => (
              <li key={`${e.t}-${i}`} style={{ opacity: i === 0 ? 1 : 0.75 }}>
                <span className="text-fd-muted-foreground">{(e.t / 1000).toFixed(2)}s</span> {e.text}
              </li>
            ))}
          </ul>
        </div>
      </div>
      <Transport {...sim} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
