'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import type { Pose2 } from '@/lib/geom/se2';
import { DUBINS_WORDS, RS_FAMILIES, dubinsAll, dubinsSample, rsAll, rsSample, type DubinsWord, type RsFamily } from '@/lib/nonholonomic';
import { drawPoseMarker } from './draw-lot';

/**
 * w21.4 — Reeds–Shepp Atlas. The start is fixed at the origin heading east;
 * the goal plane, for the goal heading on the slider, is tinted by which
 * family (or, with "Dubins only", which word) is shortest — the Souères–Laumond
 * synthesis emerging cell by cell from `rsAll` / `dubinsAll`. Drag the goal:
 * the word flips at region boundaries while the length readout moves
 * continuously.
 */

const WORLD = { minX: -8, minY: -5, maxX: 8, maxY: 5 };
const NX = 72;
const NY = 45;

type Key = RsFamily | DubinsWord;

function tintColor(p: Palette, k: Key): string {
  const map: Record<string, string> = {
    CSC: p.graph,
    'C|C|C': p.cobstacle,
    'CC|C': p.robot,
    'C|CC': p.start,
    'CC|CC': p.posterior,
    'C|CC|C': p.goal,
    'C|CSC': p.prior,
    'CSC|C': p.measurement,
    'C|CSC|C': p.truth,
    LSL: p.graph,
    RSR: p.prior,
    LSR: p.start,
    RSL: p.robot,
    RLR: p.cobstacle,
    LRL: p.goal,
  };
  return map[k] ?? p.ink;
}

export function ReedsSheppAtlas() {
  const [goal, setGoal] = useState<Pose2>({ x: 2, y: -1, theta: 0 });
  const [rho, setRho] = useState(1);
  const [dubins, setDubins] = useState(false);
  const [overlay, setOverlay] = useState(false);
  const [seed, setSeed] = useState(4);
  const dragging = useRef(false);
  const start: Pose2 = { x: 0, y: 0, theta: 0 };

  /** The synthesis for this goal heading and ρ: the optimal family per cell. */
  const tint = useMemo(() => {
    const keys: Key[] = new Array(NX * NY);
    for (let j = 0; j < NY; j++) {
      for (let i = 0; i < NX; i++) {
        const x = WORLD.minX + ((i + 0.5) / NX) * (WORLD.maxX - WORLD.minX);
        const y = WORLD.minY + ((j + 0.5) / NY) * (WORLD.maxY - WORLD.minY);
        const g = { x, y, theta: goal.theta };
        keys[j * NX + i] = dubins ? dubinsAll(start, g, rho)[0].word : rsAll(start, g, rho)[0].family;
      }
    }
    return keys;
    // start is constant
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [goal.theta, rho, dubins]);

  const best = useMemo(() => {
    if (dubins) {
      const all = dubinsAll(start, goal, rho);
      return { word: all[0].word, family: all[0].word as Key, length: all[0].length, path: dubinsSample(all[0], start, 0.05), others: all.slice(1).map((d) => dubinsSample(d, start, 0.08)) };
    }
    const all = rsAll(start, goal, rho);
    return { word: all[0].word, family: all[0].family as Key, length: all[0].length, path: rsSample(all[0], start, 0.05).map((s) => s.q), others: all.slice(1, 14).map((r) => rsSample(r, start, 0.08).map((s) => s.q)) };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [goal, rho, dubins]);

  const init = useCallback(() => ({ k: 0 }), []);
  const step = useCallback((s: { k: number }) => ({ k: s.k + 1 }), []);
  const sim = useSimulation({ init, step, fps: 30, maxTicks: 160, loop: true });
  const shown = Math.min(best.path.length, Math.ceil((sim.state.k / 100) * best.path.length));

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const cw = (sx(v, WORLD.maxX) - sx(v, WORLD.minX)) / NX;
      const ch = (sy(v, WORLD.minY) - sy(v, WORLD.maxY)) / NY;
      ctx.save();
      ctx.globalAlpha = 0.28;
      for (let j = 0; j < NY; j++) {
        for (let i = 0; i < NX; i++) {
          ctx.fillStyle = tintColor(p, tint[j * NX + i]);
          ctx.fillRect(sx(v, WORLD.minX) + i * cw, sy(v, WORLD.minY) - (j + 1) * ch, cw + 0.5, ch + 0.5);
        }
      }
      ctx.restore();
      if (overlay) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.globalAlpha = 0.18;
        ctx.lineWidth = 1;
        for (const o of best.others) {
          ctx.beginPath();
          o.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
          ctx.stroke();
        }
        ctx.restore();
      }
      ctx.save();
      ctx.strokeStyle = p.path;
      ctx.lineWidth = 3;
      ctx.beginPath();
      best.path.slice(0, shown).forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
      ctx.stroke();
      ctx.restore();
      if (shown > 0) drawPoseMarker(ctx, v, best.path[shown - 1], p.robot, 0.6 * rho);
      drawPoseMarker(ctx, v, start, p.start, 0.9 * rho);
      drawPoseMarker(ctx, v, goal, p.goal, 0.9 * rho);
      label(ctx, `${dubins ? 'Dubins' : 'Reeds–Shepp'}   word ${best.word}   family ${best.family}   length ${best.length.toFixed(3)} (ρ = ${rho.toFixed(2)})`, 10, 14, p.ink, { size: 11, weight: 600 });
      label(ctx, `goal (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}, ${goal.theta.toFixed(2)})`, 10, 30, p.ink, { size: 10 });
      // Legend.
      const keys: Key[] = dubins ? [...DUBINS_WORDS] : [...RS_FAMILIES];
      keys.forEach((k, i) => {
        const x = 10 + (i % 5) * 92;
        const y = v.height - 30 + Math.floor(i / 5) * 14;
        ctx.save();
        ctx.fillStyle = tintColor(p, k);
        ctx.globalAlpha = 0.6;
        ctx.fillRect(x, y - 5, 10, 10);
        ctx.restore();
        label(ctx, k, x + 14, y, p.ink, { size: 9 });
      });
    },
    [tint, best, shown, overlay, goal, rho, dubins],
  );

  const onPointer = useCallback(([x, y]: [number, number], phase: 'down' | 'move' | 'up') => {
    if (phase === 'down') dragging.current = Math.hypot(x - goal.x, y - goal.y) < 1.5;
    else if (phase === 'move' && dragging.current) setGoal((g) => ({ ...g, x: Math.max(-7.8, Math.min(7.8, x)), y: Math.max(-4.8, Math.min(4.8, y)) }));
    else if (phase === 'up') dragging.current = false;
  }, [goal]);

  return (
    <WidgetFrame
      id="w21.4"
      title="Reeds–Shepp Atlas"
      teaches="Reversing is not a Dubins word run backward: allowing cusps changes which word is shortest almost everywhere near the start, and the shortest length still varies continuously across the word boundaries."
      colorKey={['start', 'goal', 'path']}
      caption={
        <>
          Every pixel of the plane is a goal position (heading from the slider) tinted by its optimal word family — the synthesis
          Souères and Laumond drew by hand, recomputed here from the closed forms. Drag the red goal across a boundary: the word
          flips, the purple length does not jump. Toggle &quot;Dubins only&quot; to see how much of the plane the forward-only car
          pays for — and that RS is never longer. Try the goal (2, −1, 0) at ρ = 1 before reading Exercise 3.
        </>
      }
    >
      <SimCanvas world={WORLD} draw={draw} deps={[sim.tick, tint, best, overlay]} aspect={1.6} padding={0.1} onPointer={onPointer} cursor="grab" ariaLabel="The goal plane tinted by the optimal Reeds–Shepp family or Dubins word, with the optimal path to the draggable goal." />
      <ControlPanel columns={3}>
        <Slider label="goal heading θ" value={goal.theta} min={-Math.PI} max={Math.PI} step={0.01} onChange={(theta) => setGoal((g) => ({ ...g, theta }))} format={(x) => x.toFixed(2)} role="goal" />
        <Slider label="turning radius ρ" value={rho} min={0.5} max={2} step={0.05} onChange={setRho} format={(x) => x.toFixed(2)} />
        <div className="flex flex-col gap-1.5">
          <Toggle label="Dubins only (no reversing)" checked={dubins} onChange={setDubins} />
          <Toggle label="faint all-words overlay" checked={overlay} onChange={setOverlay} role="path" />
        </div>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={() => {
          setGoal({ x: 2, y: -1, theta: 0 });
          sim.reset();
        }}
        onReseed={() => {
          const ns = (seed * 7919 + 13) % 100000;
          setSeed(ns);
          const r = new Rng(ns);
          setGoal({ x: r.uniform(-6, 6), y: r.uniform(-4, 4), theta: r.uniform(-Math.PI, Math.PI) });
          sim.reset();
        }}
        tick={sim.tick}
        seed={seed}
      />
    </WidgetFrame>
  );
}
