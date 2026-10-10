'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import { Hitch, type HitchCfg } from '@/lib/robots/hitch';
import { LOT } from '@/lib/robots/worlds';
import { Scene } from '@/lib/collide/scene';
import { rect } from '@/lib/collide/polygon';
import { Rng } from '@/lib/prob/rng';
import type { Pose2 } from '@/lib/geom/se2';
import { PARK_LABELS, planPark, type ParkMethod, type ParkResult } from '@/lib/nonholonomic';
import { drawCarPath, drawHitch, drawHitchGauge, drawLot, drawPoseMarker } from './draw-lot';

/**
 * w21.1 — Trailer Parker. One parking query in the Lot, three steering
 * methods from the chapter, three purple paths told apart by dash pattern, the
 * selected one driven in orange with the trailer's hitch angle on a gauge, and
 * a scoreboard. Every path is `planPark` from `lib/nonholonomic/park.ts`; with
 * "obstacles" on, the third method switches from the obstacle-blind
 * Reeds–Shepp word to Alg. 22 CAR GRID SEARCH, which sees the pillars.
 */

const WORLD = { minX: 0, minY: 0, maxX: 16, maxY: 10 };
const D_HITCH = 2.5;
const BASE = Hitch.car();

type Slot = 'flat' | 'sinusoid' | 'third';
const SLOTS: Slot[] = ['flat', 'sinusoid', 'third'];
const DASH: Record<Slot, number[]> = { flat: [], sinusoid: [7, 5], third: [2, 4] };

interface Params {
  method: Slot;
  trailer: boolean;
  obstacles: boolean;
  flatSpeed: number;
  ampCap: number;
  cuspWeight: number;
  reverseIn: boolean;
}

function sceneFor(obstacles: boolean, seed: number): Scene {
  const base = Scene.fromWorld(LOT);
  if (!obstacles) return base;
  // A third pillar, placed by the seed in the open floor — the "randomized pillars" of the integration lab.
  const rng = new Rng(seed);
  const x = rng.uniform(6.5, 13);
  const y = rng.uniform(3.0, 4.2);
  return base.withObstacle('the seeded pillar', rect(x - 0.35, y - 0.35, x + 0.35, y + 0.35));
}

export function TrailerParker() {
  const [params, setParams] = useState<Params>({ method: 'flat', trailer: true, obstacles: false, flatSpeed: 1, ampCap: 12, cuspWeight: 3, reverseIn: false });
  const [seed, setSeed] = useState(21);
  const [start, setStart] = useState<Pose2>({ x: 5.5, y: 1.2, theta: 0 });
  const [bay, setBay] = useState<{ x: number; y: number }>({ x: 9.25, y: 6.4 });
  const drag = useRef<'start' | 'bay' | null>(null);

  const car = useMemo(() => (params.trailer ? BASE.withTrailer(D_HITCH) : BASE), [params.trailer]);
  const scene = useMemo(() => sceneFor(params.obstacles, seed), [params.obstacles, seed]);
  const goal: Pose2 = useMemo(() => (params.reverseIn ? { x: bay.x, y: Math.min(8.6, bay.y + 2.2), theta: -Math.PI / 2 } : { x: bay.x, y: bay.y, theta: Math.PI / 2 }), [bay, params.reverseIn]);

  const results = useMemo(() => {
    const q = { car, scene, start, goal };
    const out: Record<Slot, ParkResult> = {
      flat: planPark('flat', q, { flatSpeed: params.flatSpeed }),
      sinusoid: planPark('sinusoid', q, { sinusoidCap: params.ampCap }),
      third: planPark(params.obstacles ? 'grid' : 'rs', q, { gridCost: { a: 1, b: 0, c: params.cuspWeight }, seed }),
    };
    return out;
  }, [car, scene, start, goal, params.flatSpeed, params.ampCap, params.obstacles, params.cuspWeight, seed]);

  const sel = results[params.method];
  const init = useCallback(() => ({ i: 0 }), []);
  const step = useCallback((s: { i: number }) => ({ i: s.i + 1 }), []);
  const sim = useSimulation({ init, step, fps: 30, maxTicks: Math.max(2, sel.samples.length + 30), loop: true });
  const idx = Math.min(sim.state.i, Math.max(0, sel.samples.length - 1));

  const methodName = (s: Slot): string => (s === 'third' ? PARK_LABELS[params.obstacles ? 'grid' : 'rs'] : PARK_LABELS[s as ParkMethod]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawLot(ctx, v, LOT, p, scene);
      for (const s of SLOTS) {
        const r = results[s];
        if (!r.ok) continue;
        drawCarPath(ctx, v, r.samples, p.path, { lineWidth: s === params.method ? 3 : 1.5, dash: DASH[s], alpha: s === params.method ? 1 : 0.55 });
      }
      const startCfg: HitchCfg = { body: start, trailer: car.hasTrailer ? start.theta : null };
      drawHitch(ctx, v, car, startCfg, p.start, { alpha: 0.7 });
      drawHitch(ctx, v, car, { body: goal, trailer: car.hasTrailer ? goal.theta : null }, p.goal, { alpha: 0.7 });
      drawPoseMarker(ctx, v, start, p.start, 1.0);
      drawPoseMarker(ctx, v, goal, p.goal, 1.0);
      let hitch = 0;
      if (sel.ok && sel.samples.length) {
        const s = sel.samples[idx];
        const cfg: HitchCfg = { body: s.q, trailer: car.hasTrailer ? s.psi ?? s.q.theta : null };
        drawHitch(ctx, v, car, cfg, p.robot, { fill: true, lineWidth: 2 });
        hitch = car.hasTrailer ? Math.atan2(Math.sin(s.q.theta - (s.psi ?? s.q.theta)), Math.cos(s.q.theta - (s.psi ?? s.q.theta))) : 0;
      }
      label(ctx, `${methodName(params.method)}${sel.ok ? '' : ' — failed'}`, 10, 14, p.ink, { size: 11, weight: 600 });
      if (sel.ok) label(ctx, sel.note, 10, 30, p.ink, { size: 10 });
      if (car.hasTrailer) {
        drawHitchGauge(ctx, v.width - 60, 58, 36, hitch, p);
        label(ctx, `θ₁ = ${hitch.toFixed(2)} rad`, v.width - 60, 72, Math.abs(hitch) > 1.05 ? p.goal : p.ink, { size: 10, align: 'center' });
      }
    },
    [results, params.method, sel, idx, car, start, goal, scene, methodName],
  );

  const onPointer = useCallback(
    ([x, y]: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'down') {
        const ds = Math.hypot(x - start.x, y - start.y);
        const db = Math.hypot(x - goal.x, y - goal.y);
        drag.current = ds < 1.6 && ds <= db ? 'start' : db < 1.6 ? 'bay' : null;
      } else if (phase === 'move' && drag.current) {
        const cx = Math.max(1, Math.min(15, x));
        const cy = Math.max(1, Math.min(9, y));
        if (drag.current === 'start') setStart({ x: cx, y: cy, theta: 0 });
        else setBay({ x: cx, y: Math.min(6.6, cy) });
      } else if (phase === 'up') {
        drag.current = null;
        sim.reset();
      }
    },
    [start, goal, sim],
  );

  const fmt = (x: number, d = 2) => (Number.isFinite(x) ? x.toFixed(d) : '—');

  return (
    <WidgetFrame
      id="w21.1"
      title="Trailer Parker"
      teaches="There is no single 'path for a car': each steering method optimizes something different, and the trailer's swing is a cost none of them was asked about."
      colorKey={['start', 'goal', 'obstacle', 'path', 'robot']}
      wide
      caption={
        <>
          Three purple paths for one query: solid = flat-output quintic (the flat point is the trailer axle when hitched), long
          dashes = chained-form sinusoids, dots = the Reeds–Shepp word (Alg. 22 CAR GRID SEARCH once obstacles are on). Drag the green
          start or the red bay. Watch the gauge: the sinusoids drive the trailer far past the flat planner&apos;s swing, and both
          obstacle-blind methods can clip a pillar — the scoreboard says so instead of hiding it.
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, results, params]} aspect={1.6} padding={0.2} onPointer={onPointer} cursor="grab" ariaLabel="The Lot with Hitch and trailer: three planned paths into a bay, the selected one executed in orange, and a hitch-angle gauge." />
      <div className="overflow-x-auto border-t border-fd-border px-3 py-2">
        <table className="w-full font-mono text-[0.7rem]">
          <thead>
            <tr className="text-left text-fd-muted-foreground">
              <th className="pr-3 font-medium">method</th>
              <th className="pr-3 font-medium">length (m)</th>
              <th className="pr-3 font-medium">cusps</th>
              <th className="pr-3 font-medium">max |θ₁|</th>
              <th className="pr-3 font-medium">max |φ|</th>
              <th className="pr-3 font-medium">clearance</th>
              <th className="pr-3 font-medium">plan (ms)</th>
            </tr>
          </thead>
          <tbody>
            {SLOTS.map((s) => {
              const r = results[s];
              return (
                <tr key={s} style={s === params.method ? { color: 'var(--rm-path)' } : undefined}>
                  <td className="pr-3">{methodName(s)}</td>
                  <td className="pr-3">{r.ok ? fmt(r.score.length) : 'failed'}</td>
                  <td className="pr-3">{r.ok ? r.score.cusps : '—'}</td>
                  <td className="pr-3" style={r.score.maxHitch > 1.05 ? { color: 'var(--rm-goal)' } : undefined}>
                    {car.hasTrailer && r.ok ? fmt(r.score.maxHitch) : '—'}
                  </td>
                  <td className="pr-3" style={r.steerViolated ? { color: 'var(--rm-goal)' } : undefined}>
                    {r.ok ? `${fmt(r.maxSteer)}${r.steerViolated ? ' > 0.60' : ''}` : '—'}
                  </td>
                  <td className="pr-3" style={r.score.collides ? { color: 'var(--rm-goal)' } : undefined}>
                    {r.ok ? (r.score.collides ? 'collides' : fmt(r.score.clearance)) : '—'}
                  </td>
                  <td className="pr-3">{fmt(r.ms, 1)}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <ControlPanel columns={3}>
        <div className="flex flex-col gap-1.5">
          <span className="font-ui text-[0.72rem] font-medium">steering method</span>
          <ButtonRow>
            {SLOTS.map((s) => (
              <ActionButton key={s} emphasis={params.method === s} onClick={() => (setParams((q) => ({ ...q, method: s })), sim.reset())}>
                {s === 'flat' ? 'flat' : s === 'sinusoid' ? 'sinusoids' : params.obstacles ? 'Alg. 22' : 'Reeds–Shepp'}
              </ActionButton>
            ))}
          </ButtonRow>
        </div>
        <div className="flex flex-col gap-1.5">
          <Toggle label="trailer (d = 2.5 m)" checked={params.trailer} onChange={(trailer) => (setParams((q) => ({ ...q, trailer })), sim.reset())} role="robot" />
          <Toggle label="obstacles: seeded pillar, Alg. 22" checked={params.obstacles} onChange={(obstacles) => (setParams((q) => ({ ...q, obstacles })), sim.reset())} role="obstacle" />
          <Toggle label="reverse into the bay" checked={params.reverseIn} onChange={(reverseIn) => (setParams((q) => ({ ...q, reverseIn })), sim.reset())} role="goal" />
        </div>
        <div className="flex flex-col gap-1.5">
          <Slider label="flat end speed (× distance)" value={params.flatSpeed} min={0.3} max={2} step={0.05} onChange={(flatSpeed) => setParams((q) => ({ ...q, flatSpeed }))} format={(x) => x.toFixed(2)} />
          <Slider label="sinusoid amplitude cap |a|" value={params.ampCap} min={2} max={20} step={0.5} onChange={(ampCap) => setParams((q) => ({ ...q, ampCap }))} format={(x) => x.toFixed(1)} />
          <Slider label="Alg. 22 cusp weight c" value={params.cuspWeight} min={0} max={50} step={1} onChange={(cuspWeight) => setParams((q) => ({ ...q, cuspWeight }))} />
        </div>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => {
          setSeed((s) => (s * 7919 + 13) % 100000);
          sim.reset();
        }}
        tick={sim.tick}
        seed={seed}
      />
    </WidgetFrame>
  );
}
