'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import Link from 'next/link';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { drawPath, drawRobot, drawSegments, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { paintArmMapped, paintPolyWorld, rectMapper } from '@/lib/sampling/paint';
import { APARTMENT } from '@/lib/sim/world';
import { LOT, WORKBENCH } from '@/lib/robots/worlds';
import { MissionRecorder, SEED_7_CHAOS, effectiveStages, type MissionCfg, type MissionFrame } from '@/lib/capstone/mission';
import { GUARANTEE_NAMES, composedGuarantee, type StageInfo } from '@/lib/capstone/stage';
import { BLUEPRINT } from '@/lib/capstone/worlds';
import { paintHitchCfg } from './shared';

/**
 * w23.2 — Stack Anatomy.
 *
 * The §5 block diagram, driven by a recorded mission. Every arrow carries the
 * message rate measured over the last simulated second; every block is tinted
 * by how stale its newest input was when it last ran, relative to its period.
 * Click a block for its contract card; switch it off and the mission is
 * re-recorded with that stage dead from t = 3 s, so the degradation you watch
 * is caused, not described. Recording is time-sliced: a few ticks per frame,
 * playback never waits for the whole mission.
 */

const OFF_FROM = 3000;
const TRACES: { name: string; cfg: Omit<MissionCfg, 'disabled'> }[] = [
  { name: 'seed 7 · all three chaos events', cfg: { seed: 7, chaos: SEED_7_CHAOS, tMaxMs: 45000 } },
  { name: 'seed 7 · no chaos', cfg: { seed: 7, tMaxMs: 45000 } },
];

interface State {
  rec: MissionRecorder;
  head: number;
}

/** Diagram geometry: one row per robot, stages left to right in topological order. */
const ROW_Y: Record<StageInfo['robot'], number> = { rusty: 18, reach: 128, hitch: 238 };
const BOX_W = 118;
const BOX_H = 62;
const GAP = 18;

function heat(ageMs: number, periodMs: number): number {
  return Math.max(0, Math.min(1, ageMs / Math.max(1, 2 * periodMs)));
}

export function StackAnatomy() {
  const [trace, setTrace] = useState(0);
  const [off, setOff] = useState<string | null>(null);
  const [selected, setSelected] = useState<string>('rusty.plan');

  const init = useCallback(
    (): State => ({ rec: new MissionRecorder({ ...TRACES[trace].cfg, disabled: off ? [{ id: off, from: OFF_FROM }] : [] }), head: 0 }),
    [trace, off],
  );
  const step = useCallback((s: State): State => {
    // Record ahead of the playhead, a few ticks per frame: time-sliced, never blocking.
    if (!s.rec.done && s.rec.frames.length < s.head + 40) s.rec.advance(2);
    const head = Math.min(s.head + 1, s.rec.frames.length - 1);
    return { rec: s.rec, head };
  }, []);
  const sim = useSimulation<State>({ init, step, fps: 20, initialSeed: 7, loop: false });
  // A new trace or a switched-off stage is a new recording: restart from t = 0.
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
  const frame: MissionFrame = rec.frames[head];
  const prev: MissionFrame = rec.frames[Math.max(0, head - 20)];
  const infos = useMemo(() => rec.mission.stages(), [rec]);
  const win = Math.max(0.05, (frame.t - prev.t) / 1000);

  const rate = (id: string) => (frame.emitted[id] - (prev.emitted[id] ?? 0)) / win;
  const sel = infos.find((s) => s.id === selected) ?? infos[0];
  const rows: StageInfo['robot'][] = ['rusty', 'reach', 'hitch'];

  const restart = (nextOff: string | null, nextTrace = trace) => {
    setOff(nextOff);
    setTrace(nextTrace);
  };

  const drawApt = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      ctx.clearRect(0, 0, v.width, v.height);
      ctx.fillStyle = p.bg;
      ctx.fillRect(0, 0, v.width, v.height);
      const r = rec.mission.rusty;
      const f = frame.rusty;
      if (!r || !f) return;
      drawSegments(ctx, v, BLUEPRINT.walls, p.obstacle, 1.8);
      if (f.doorClosed && r.shutDoor) drawSegments(ctx, v, [r.shutDoor], p.goal, 3);
      drawPath(ctx, v, r.trail.slice(0, f.trail), p.robot, { lineWidth: 1.4, alpha: 0.7 });
      if (f.path.length > 1) drawPath(ctx, v, f.path, p.path, { lineWidth: 2 });
      drawRobot(ctx, v, f.truth, p.robot, 0.35);
      label(ctx, f.mode, 6, 14, f.mode.startsWith('Failed') ? p.goal : p.ink, { size: 10, weight: 600 });
    },
    [rec, frame],
  );
  const drawArm = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      ctx.clearRect(0, 0, v.width, v.height);
      ctx.fillStyle = p.bg;
      ctx.fillRect(0, 0, v.width, v.height);
      const a = rec.mission.reach;
      const f = frame.reach;
      if (!a || !f) return;
      const m = rectMapper(v, { x0: v.minX, y0: v.minY, x1: v.maxX, y1: v.maxY }, WORKBENCH.bounds);
      paintPolyWorld(ctx, m, WORKBENCH, p);
      ctx.save();
      ctx.fillStyle = p.goal;
      ctx.beginPath();
      ctx.arc(m.X(f.tip.x), m.Y(f.tip.y), 4, 0, 2 * Math.PI);
      ctx.fill();
      ctx.restore();
      paintArmMapped(ctx, m, p, a.arm, f.q, p.robot, 1);
      label(ctx, `${f.mode} · picks ${f.picks}`, 6, 14, f.mode.startsWith('Failed') ? p.goal : p.ink, { size: 10, weight: 600 });
    },
    [rec, frame],
  );
  const drawLot = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      ctx.clearRect(0, 0, v.width, v.height);
      ctx.fillStyle = p.bg;
      ctx.fillRect(0, 0, v.width, v.height);
      const h = rec.mission.hitch;
      const f = frame.hitch;
      if (!h || !f) return;
      if (LOT.markings) drawSegments(ctx, v, LOT.markings, p.grid, 1.5);
      drawSegments(ctx, v, LOT.walls, p.wall, 2);
      for (const o of LOT.obstacles) {
        ctx.fillStyle = p.obstacle;
        ctx.beginPath();
        o.poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
        ctx.closePath();
        ctx.fill();
      }
      if (f.plan.length > 1) drawPath(ctx, v, f.plan, p.path, { lineWidth: 1.8 });
      drawPath(ctx, v, h.executed.slice(0, f.executed).map((q) => q.body), p.robot, { lineWidth: 1.4, alpha: 0.7 });
      paintHitchCfg(ctx, v, h, f.q, p.robot, 0.85);
      label(ctx, `${f.mode} · φ ${f.phi.toFixed(2)}`, 6, 14, f.mode.startsWith('Failed') ? p.goal : p.ink, { size: 10, weight: 600 });
    },
    [rec, frame],
  );

  const failures = [frame.rusty, frame.reach, frame.hitch].filter((x) => x && x.detail).map((x) => x!.detail);

  return (
    <WidgetFrame
      id="w23.2"
      title="Stack Anatomy"
      teaches="An architecture diagram is not decoration: the arrows carry measured traffic, and removing a box has a visible, causal consequence."
      colorKey={['path', 'robot', 'cobstacle', 'goal']}
      wide
      caption={
        <>
          Fifteen stages on one 20 Hz round-robin. The number on each arrow is the rate its source actually emitted
          over the last simulated second — the scan at 10 Hz, D* Lite at 2 Hz, Reach&apos;s planner only while it
          has a query; the amber tint is the age of a block&apos;s newest input relative to its period. Click a block
          to read its contract, then switch it off: the mission re-records with that stage dead from t = 3 s. Try
          <em> D* Lite</em> (Rusty drives its stale path until the D23.2 check stops it short of the shut door, for
          good), <em>Time scaling</em> (the tracker is handed a Path and refuses it — in Rust that refusal is a
          compile error), and <em>Jackknife guard</em> (Reverse Hard folds the trailer past φ_max).
        </>
      }
    >
      <div className="overflow-x-auto px-2 pt-2">
        <svg viewBox="0 0 860 310" className="w-full min-w-[640px]" role="img" aria-label="Block diagram of the three pipelines with live message rates on the arrows and staleness tints on the blocks.">
          {rows.map((robot) => {
            const row = infos.filter((s) => s.robot === robot);
            return (
              <g key={robot}>
                <text x={4} y={ROW_Y[robot] + BOX_H / 2} fontSize={11} fontWeight={700} fill="var(--color-fd-foreground)" fontFamily="ui-monospace, monospace">
                  {robot}
                </text>
                <text x={4} y={ROW_Y[robot] + BOX_H + 14} fontSize={9.5} fill="var(--rm-path)" fontFamily="ui-monospace, monospace">
                  𝒢 = {GUARANTEE_NAMES[composedGuarantee(effectiveStages(row, off && frame.t >= OFF_FROM ? [off] : []))]}
                </text>
                {row.map((st, i) => {
                  const x = 52 + i * (BOX_W + GAP);
                  const y = ROW_Y[robot];
                  const isOff = off === st.id && frame.t >= OFF_FROM;
                  const isSel = st.id === selected;
                  const tint = heat(frame.age[st.id] ?? 0, st.periodMs);
                  const ran = frame.ran.includes(st.id);
                  return (
                    <g key={st.id} onClick={() => setSelected(st.id)} style={{ cursor: 'pointer' }}>
                      <rect x={x} y={y} width={BOX_W} height={BOX_H} rx={5} fill="var(--rm-cobstacle)" fillOpacity={isOff ? 0 : 0.08 + 0.5 * tint} stroke={isSel ? 'var(--pr-widget-ink)' : 'var(--color-fd-border)'} strokeWidth={isSel ? 2.2 : 1.2} strokeDasharray={isOff ? '5 4' : undefined} />
                      <text x={x + 8} y={y + 16} fontSize={11} fontWeight={600} fill={isOff ? 'var(--color-fd-muted-foreground)' : 'var(--color-fd-foreground)'}>
                        {st.label.length > 18 ? `${st.label.slice(0, 17)}…` : st.label}
                      </text>
                      <text x={x + 8} y={y + 31} fontSize={9.5} fill="var(--color-fd-muted-foreground)" fontFamily="ui-monospace, monospace">
                        Ch. {st.chapter} · {Math.round(1000 / st.periodMs)} Hz
                      </text>
                      <text x={x + 8} y={y + 46} fontSize={9.5} fill="var(--color-fd-muted-foreground)" fontFamily="ui-monospace, monospace">
                        {isOff ? 'OFF' : `ς ${frame.age[st.id] ?? 0} ms`}
                      </text>
                      <circle cx={x + BOX_W - 10} cy={y + 10} r={3.5} fill={ran && !isOff ? 'var(--rm-robot)' : 'var(--color-fd-border)'} />
                      {i < row.length - 1 ? (
                        <g>
                          <line x1={x + BOX_W} y1={y + BOX_H / 2} x2={x + BOX_W + GAP - 3} y2={y + BOX_H / 2} stroke="var(--rm-path)" strokeWidth={1.6} />
                          <polygon points={`${x + BOX_W + GAP - 3},${y + BOX_H / 2 - 3.5} ${x + BOX_W + GAP + 2},${y + BOX_H / 2} ${x + BOX_W + GAP - 3},${y + BOX_H / 2 + 3.5}`} fill="var(--rm-path)" />
                          <text x={x + BOX_W + GAP / 2} y={y + BOX_H / 2 + 15} fontSize={9} textAnchor="middle" fill="var(--rm-path)" fontFamily="ui-monospace, monospace">
                            {rate(st.id).toFixed(1)}
                          </text>
                        </g>
                      ) : null}
                    </g>
                  );
                })}
              </g>
            );
          })}
          <text x={852} y={304} fontSize={9} textAnchor="end" fill="var(--color-fd-muted-foreground)" fontFamily="ui-monospace, monospace">
            arrows: msgs/s over the last simulated second · t = {(frame.t / 1000).toFixed(1)} s{rec.done ? '' : ' (recording…)'}
          </text>
        </svg>
      </div>

      <div className="grid grid-cols-1 border-t border-fd-border md:grid-cols-[minmax(0,1fr)_300px]">
        <div className="grid grid-cols-3 gap-0 border-b border-fd-border md:border-b-0 md:border-r">
          <SimCanvas world={APARTMENT.bounds} draw={drawApt} deps={[head, rec]} aspect={12 / 9} padding={0.2} ariaLabel="Recorded Apartment panel at the playhead." />
          <SimCanvas world={{ minX: 0, minY: 0, maxX: 1, maxY: 1 }} draw={drawArm} deps={[head, rec]} aspect={1.33} padding={0} ariaLabel="Recorded Workbench panel at the playhead." />
          <SimCanvas world={LOT.bounds} draw={drawLot} deps={[head, rec]} aspect={16 / 12} padding={0.2} ariaLabel="Recorded Lot panel at the playhead." />
        </div>
        <div className="px-3 py-2">
          <p className="eyebrow mb-1">contract · {sel.id}</p>
          <dl className="space-y-0.5 font-mono text-[0.66rem]">
            <div className="flex justify-between gap-2"><dt className="text-fd-muted-foreground">chapter</dt><dd><Link href={`/chapters/${sel.slug}`} className="underline">Ch. {sel.chapter}</Link></dd></div>
            <div className="flex justify-between gap-2"><dt className="text-fd-muted-foreground">in</dt><dd className="text-right">{sel.input}</dd></div>
            <div className="flex justify-between gap-2"><dt className="text-fd-muted-foreground">out</dt><dd className="text-right">{sel.output}</dd></div>
            <div className="flex justify-between gap-2"><dt className="text-fd-muted-foreground">period</dt><dd>{sel.periodMs} ms</dd></div>
            <div className="flex justify-between gap-2"><dt className="text-fd-muted-foreground">role / 𝒢</dt><dd>{sel.role} · {GUARANTEE_NAMES[sel.guarantee]}</dd></div>
          </dl>
          <p className="mt-1.5 font-ui text-[0.68rem] leading-snug text-fd-muted-foreground">{sel.degradation}</p>
          <div className="mt-2">
            <ButtonRow>
              <ActionButton onClick={() => restart(off === sel.id ? null : sel.id)} emphasis={off !== sel.id}>
                {off === sel.id ? 'Switch back on' : 'Switch off at t = 3 s'}
              </ActionButton>
              <ActionButton onClick={() => restart(off, (trace + 1) % TRACES.length)}>Trace: {TRACES[trace].name}</ActionButton>
            </ButtonRow>
          </div>
          {failures.length ? (
            <pre className="mt-2 whitespace-pre-wrap rounded-sm border border-fd-border p-1.5 font-mono text-[0.62rem] leading-snug" style={{ color: 'var(--rm-goal)' }}>
              {failures.join('\n')}
            </pre>
          ) : null}
        </div>
      </div>
      <Transport {...sim} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
