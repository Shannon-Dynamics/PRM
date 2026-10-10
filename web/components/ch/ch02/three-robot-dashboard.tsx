'use client';

import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawPath, drawSegments, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Hitch } from '@/lib/robots/hitch';
import { Reach, fmtTn, t2, type Tn } from '@/lib/robots/reach';
import { APARTMENT, LOT, WORKBENCH } from '@/lib/robots/worlds';
import { RUSTY } from '@/lib/robots/rusty';
import {
  TOUR_HZ,
  tourCheckers,
  tourInit,
  tourStep,
  type TourOverrides,
  type TourState,
} from '@/lib/robots/tour';
import type { Polygon } from '@/lib/collide/polygon';

/**
 * w2.1 — the Three-Robot Dashboard: the lab's front door.
 *
 * Three panes, one clock. Rusty circles the Apartment, Reach sweeps the
 * Workbench, Hitch backs through the Lot, and each flashes on contact. Pick a
 * robot and the script hands you its controls: arrow keys drive Rusty (v, ω)
 * and Hitch (v, steering φ); Reach's joints are dragged by their handles.
 *
 * The misconception this kills is "driving the arm's tip is like driving the
 * rover". You drive *angles*: the tip goes where φ(θ₁, θ₂) sends it, and when
 * one joint moves the tip traces a circle whether you like it or not. Two
 * quieter lessons ride along: Hitch cannot slide sideways no matter what you
 * press, and Rusty stops a body radius short of the wall, not at it.
 *
 * Everything runs the real `tourStep` from the library — the same function the
 * determinism check replays bit for bit.
 */

type Active = 'rusty' | 'reach' | 'hitch';
const FLASH_TICKS = 6;

export function ThreeRobotDashboard() {
  const [active, setActive] = useState<Active>('rusty');
  const [lengths, setLengths] = useState<[number, number]>([1, 1]);
  const [trailer, setTrailer] = useState(false);
  const [hitchLen, setHitchLen] = useState(2.5);

  const reach = useMemo(() => Reach.workbench().withLengths(lengths), [lengths]);
  const hitch = useMemo(() => Hitch.car().withTrailer(trailer ? hitchLen : null), [trailer, hitchLen]);
  const checkers = useMemo(() => tourCheckers({ reach, hitch }), [reach, hitch]);

  // Reader input lives in refs: the simulation clock reads it every tick
  // without a re-render per key event.
  const keys = useRef<Set<string>>(new Set());
  const taken = useRef<Set<Active>>(new Set());
  const reachManual = useRef<Tn | null>(null);
  const steer = useRef(0);
  const drag = useRef<'elbow' | 'tip' | null>(null);

  const init = useCallback(
    (seed: number): TourState => {
      taken.current = new Set();
      reachManual.current = null;
      steer.current = 0;
      return tourInit(seed, { reach, hitch });
    },
    [reach, hitch],
  );

  const step = useCallback(
    (s: TourState, tick: number): TourState => {
      const o: TourOverrides = {};
      const k = keys.current;
      if (active === 'rusty' && (taken.current.has('rusty') || k.size > 0)) {
        taken.current.add('rusty');
        const v = (k.has('ArrowUp') ? 0.6 : 0) - (k.has('ArrowDown') ? 0.6 : 0);
        const omega = (k.has('ArrowLeft') ? 1.6 : 0) - (k.has('ArrowRight') ? 1.6 : 0);
        o.rusty = { v, omega };
      }
      if (active === 'hitch' && (taken.current.has('hitch') || k.size > 0)) {
        taken.current.add('hitch');
        if (k.has('ArrowLeft')) steer.current = Math.min(0.6, steer.current + 0.04);
        if (k.has('ArrowRight')) steer.current = Math.max(-0.6, steer.current - 0.04);
        const v = (k.has('ArrowUp') ? 1.2 : 0) - (k.has('ArrowDown') ? 1.2 : 0);
        o.hitch = { v, phi: steer.current };
      }
      if (reachManual.current) o.reach = reachManual.current;
      return tourStep(s, tick, checkers, o);
    },
    [active, checkers],
  );

  const sim = useSimulation<TourState>({ init, step, fps: TOUR_HZ, initialSeed: 2005 });
  const { reset } = sim;

  // A new arm or a new trailer is a new configuration type: restart the tour.
  useEffect(() => {
    reset();
  }, [reach, hitch, reset]);

  const choose = (r: Active) => {
    setActive(r);
    taken.current = new Set();
    reachManual.current = null;
  };

  // ---- keyboard --------------------------------------------------------------
  const onKey = (down: boolean) => (e: React.KeyboardEvent) => {
    if (!e.key.startsWith('Arrow')) return;
    e.preventDefault();
    if (down) keys.current.add(e.key);
    else keys.current.delete(e.key);
  };
  const hold = (key: string) => ({
    onPointerDown: () => keys.current.add(key),
    onPointerUp: () => keys.current.delete(key),
    onPointerLeave: () => keys.current.delete(key),
  });

  // ---- Reach joint handles ---------------------------------------------------
  const onReachPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const q = reachManual.current ?? sim.state.reach.q;
      const pts = reach.fk(q);
      if (phase === 'up') {
        drag.current = null;
        return;
      }
      if (phase === 'down') {
        const dElbow = Math.hypot(pts[1].x - w[0], pts[1].y - w[1]);
        const dTip = Math.hypot(pts[2].x - w[0], pts[2].y - w[1]);
        if (Math.min(dElbow, dTip) > 0.35) return;
        drag.current = dElbow < dTip ? 'elbow' : 'tip';
        choose('reach');
        reachManual.current = q;
        return;
      }
      if (!drag.current) return;
      const base = reach.base;
      if (drag.current === 'elbow') {
        // Dragging the elbow sets θ₁ alone; the forearm keeps its relative angle.
        const th1 = Math.atan2(w[1] - base.y, w[0] - base.x);
        reachManual.current = t2(th1, q[1]);
      } else {
        // Dragging the tip sets θ₂ given θ₁: the absolute forearm heading minus θ₁.
        const abs = Math.atan2(w[1] - pts[1].y, w[0] - pts[1].x);
        reachManual.current = t2(q[0], abs - q[0]);
      }
    },
    [reach, sim.state.reach.q],
  );

  // ---- drawing -----------------------------------------------------------------
  const s = sim.state;

  const drawApartment = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
      drawPath(ctx, v, s.rusty.trail, p.robot, { lineWidth: 1.5, alpha: 0.55 });
      const pose = s.rusty.q.pose;
      const r = Math.max(sl(v, RUSTY.bodyRadius), 4);
      ctx.save();
      ctx.fillStyle = p.robot;
      ctx.beginPath();
      ctx.arc(sx(v, pose.x), sy(v, pose.y), r, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = p.bg;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(sx(v, pose.x), sy(v, pose.y));
      ctx.lineTo(sx(v, pose.x + RUSTY.bodyRadius * 1.6 * Math.cos(pose.theta)), sy(v, pose.y + RUSTY.bodyRadius * 1.6 * Math.sin(pose.theta)));
      ctx.stroke();
      flashRing(ctx, p, s.rusty.flash, sx(v, pose.x), sy(v, pose.y), r + 4);
      ctx.restore();
    },
    [s],
  );

  const drawWorkbench = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 0.5);
      drawSegments(ctx, v, WORKBENCH.walls, p.wall, 2.5);
      fillPolygons(ctx, v, WORKBENCH.obstacles.map((o) => o.poly), p.obstacle);
      // The reachable annulus, faintly: the workspace is not the C-space.
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.arc(sx(v, 0), sy(v, 0), sl(v, reach.reach()), 0, Math.PI * 2);
      ctx.stroke();
      if (reach.innerRadius() > 0) {
        ctx.beginPath();
        ctx.arc(sx(v, 0), sy(v, 0), sl(v, reach.innerRadius()), 0, Math.PI * 2);
        ctx.stroke();
      }
      ctx.restore();
      drawPath(ctx, v, s.reach.trail, p.robot, { lineWidth: 1.2, alpha: 0.5 });
      const q = reachManual.current ?? s.reach.q;
      drawArm(ctx, v, p, reach, q, s.reach.flash, active === 'reach');
    },
    [s, reach, active],
  );

  const drawLot = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      drawSegments(ctx, v, LOT.walls, p.wall, 2.5);
      if (LOT.markings) drawSegments(ctx, v, LOT.markings, p.unknown, 1.5);
      fillPolygons(ctx, v, LOT.obstacles.map((o) => o.poly), p.obstacle);
      drawPath(ctx, v, s.hitch.trail, p.robot, { lineWidth: 1.5, alpha: 0.55 });
      drawCar(ctx, v, p, hitch, s.hitch.q, s.hitch.flash);
    },
    [s, hitch],
  );

  const reachQ = reachManual.current ?? s.reach.q;

  return (
    <WidgetFrame
      id="w2.1"
      title="Three-Robot Dashboard"
      teaches="A configuration is a few typed numbers and R(q) follows from them; you drive the angles, not the tip."
      colorKey={['robot', 'obstacle', 'goal']}
      wide
      caption={
        <>
          Three worlds, one clock. Each robot follows a seeded tour until you take over: pick it, then
          use the arrow keys (or the buttons) for Rusty and Hitch, or drag Reach's elbow and tip
          handles on the Workbench. A body outlined in red has just touched something — the only
          place in the book where red means "bad" — and the counters tally contacts. Watch Reach's
          tip when you drag only the elbow: it traces a circle, because you moved θ₁, not the tip.
          Try to make Hitch slide sideways into a bay; you cannot, and that is Chapter 20. Re-roll
          the seed to change the tours; the worlds never change.
        </>
      }
    >
      <div
        tabIndex={0}
        onKeyDown={onKey(true)}
        onKeyUp={onKey(false)}
        onBlur={() => keys.current.clear()}
        className="grid grid-cols-1 gap-px bg-fd-border outline-none focus-visible:ring-2 focus-visible:ring-fd-primary md:grid-cols-[1.3fr_1fr_1.5fr]"
        aria-label="Three simulation panes; focus here and use the arrow keys to drive the active robot"
      >
        <Pane name="Rusty · the Apartment" active={active === 'rusty'} onPick={() => choose('rusty')} count={s.rusty.collisions}>
          <SimCanvas world={APARTMENT.bounds} draw={drawApartment} deps={[sim.tick, active]} aspect={12 / 9} padding={0.3} ariaLabel="Rusty, an orange disc, driving through the Apartment floor plan" />
        </Pane>
        <Pane name="Reach · the Workbench" active={active === 'reach'} onPick={() => choose('reach')} count={s.reach.collisions}>
          <SimCanvas world={WORKBENCH.bounds} draw={drawWorkbench} deps={[sim.tick, active, reach]} aspect={1} padding={0.15} onPointer={onReachPointer} cursor={drag.current ? 'grabbing' : 'grab'} ariaLabel="Reach, a two-link orange arm on a square table with three slate obstacles" />
        </Pane>
        <Pane name="Hitch · the Lot" active={active === 'hitch'} onPick={() => choose('hitch')} count={s.hitch.collisions}>
          <SimCanvas world={LOT.bounds} draw={drawLot} deps={[sim.tick, active, hitch]} aspect={16 / 10} padding={0.3} ariaLabel="Hitch, an orange car-like robot, maneuvering in a parking lot with two pillars" />
        </Pane>
      </div>

      <div className="grid grid-cols-1 divide-y divide-fd-border border-t border-fd-border font-mono text-[0.7rem] tabular-nums sm:grid-cols-3 sm:divide-x sm:divide-y-0">
        <Readout
          title="Rusty"
          value={`Pose2 { x: ${s.rusty.q.pose.x.toFixed(3)}, y: ${s.rusty.q.pose.y.toFixed(3)}, theta: ${s.rusty.q.pose.theta.toFixed(3)} }`}
          sub={`R(q): disc r = ${RUSTY.bodyRadius} m`}
        />
        <Readout title="Reach" value={fmtTn(reachQ)} sub={`tip φ(q) = (${reach.tip(reachQ).x.toFixed(3)}, ${reach.tip(reachQ).y.toFixed(3)})`} />
        <Readout
          title="Hitch"
          value={Hitch.fmt(s.hitch.q)}
          sub={active === 'hitch' ? `steering φ = ${steer.current.toFixed(2)} rad (a control, not a coordinate)` : 'φ is a control, not a coordinate'}
        />
      </div>

      <ControlPanel columns={2} title="Active robot">
        <ButtonRow>
          {(['rusty', 'reach', 'hitch'] as Active[]).map((r) => (
            <ActionButton key={r} emphasis={active === r} onClick={() => choose(r)}>
              {r === 'rusty' ? 'Rusty' : r === 'reach' ? 'Reach' : 'Hitch'}
            </ActionButton>
          ))}
        </ButtonRow>
        <ButtonRow>
          <span className="font-ui text-[0.72rem] text-fd-muted-foreground">
            {active === 'reach' ? 'drag the elbow or tip handle' : active === 'rusty' ? 'arrows: ▲▼ speed, ◀▶ turn' : 'arrows: ▲▼ drive, ◀▶ steer'}
          </span>
          {active !== 'reach'
            ? ['ArrowUp', 'ArrowLeft', 'ArrowDown', 'ArrowRight'].map((k) => (
                <button
                  key={k}
                  type="button"
                  {...hold(k)}
                  aria-label={k.replace('Arrow', '')}
                  className="rounded-sm border border-fd-border bg-fd-card px-2 py-0.5 font-mono text-xs select-none hover:bg-fd-accent"
                >
                  {k === 'ArrowUp' ? '▲' : k === 'ArrowDown' ? '▼' : k === 'ArrowLeft' ? '◀' : '▶'}
                </button>
              ))
            : null}
        </ButtonRow>
      </ControlPanel>

      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Advanced: link lengths, trailer</summary>
        <ControlPanel columns={3}>
          <Slider label="Reach L₁" role="robot" value={lengths[0]} min={0.5} max={1.4} step={0.05} unit="m" onChange={(v) => setLengths([v, lengths[1]])} />
          <Slider label="Reach L₂" role="robot" value={lengths[1]} min={0.5} max={1.4} step={0.05} unit="m" onChange={(v) => setLengths([lengths[0], v])} />
          <div className="flex flex-col gap-2">
            <Toggle label="Hitch pulls a trailer (SE(2) × S¹)" role="robot" checked={trailer} onChange={setTrailer} />
            <Slider label="Hitch length d_h" value={hitchLen} min={2.0} max={3.5} step={0.1} unit="m" onChange={setHitchLen} />
          </div>
        </ControlPanel>
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

// ---------------------------------------------------------------------------
// Pieces
// ---------------------------------------------------------------------------

function Pane({ name, active, onPick, count, children }: { name: string; active: boolean; onPick: () => void; count: number; children: ReactNode }) {
  return (
    <div className="bg-fd-card" onPointerDown={onPick}>
      <div className="flex items-baseline justify-between px-2 py-1">
        <span className="eyebrow" style={active ? { color: 'var(--rm-robot)' } : undefined}>
          {name}
          {active ? ' · active' : ''}
        </span>
        <span className="font-mono text-[0.68rem] text-fd-muted-foreground tabular-nums">
          contacts <span style={count > 0 ? { color: 'var(--rm-goal)' } : undefined}>{count}</span>
        </span>
      </div>
      {children}
    </div>
  );
}

function Readout({ title, value, sub }: { title: string; value: string; sub: string }) {
  return (
    <div className="px-3 py-2">
      <div className="eyebrow">{title}</div>
      <div className="mt-0.5 break-words text-fd-foreground">{value}</div>
      <div className="mt-0.5 font-ui text-[0.68rem] text-fd-muted-foreground">{sub}</div>
    </div>
  );
}

export function fillPolygons(ctx: CanvasRenderingContext2D, v: Viewport, polys: Polygon[], color: string, alpha = 1) {
  ctx.save();
  ctx.fillStyle = color;
  ctx.globalAlpha = alpha;
  for (const poly of polys) {
    ctx.beginPath();
    poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();
}

export function strokePolygons(ctx: CanvasRenderingContext2D, v: Viewport, polys: Polygon[], color: string, lineWidth = 2, dashed = false) {
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  if (dashed) ctx.setLineDash([5, 4]);
  for (const poly of polys) {
    ctx.beginPath();
    poly.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
    ctx.closePath();
    ctx.stroke();
  }
  ctx.restore();
}

/** The red outline that marks a contact for a few frames. */
export function flashRing(ctx: CanvasRenderingContext2D, p: Palette, flash: number, cx: number, cy: number, r: number) {
  if (flash < 0 || flash > FLASH_TICKS) return;
  ctx.save();
  ctx.strokeStyle = p.goal;
  ctx.lineWidth = 3;
  ctx.globalAlpha = 1 - flash / (FLASH_TICKS + 1);
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();
}

/** Reach as capsule links with joint dots and optional drag handles. */
export function drawArm(
  ctx: CanvasRenderingContext2D,
  v: Viewport,
  p: Palette,
  arm: Reach,
  q: Tn,
  flash: number,
  handles: boolean,
  color: string = p.robot,
) {
  const pts = arm.fk(q);
  const w = Math.max(sl(v, 2 * arm.linkRadius), 5);
  ctx.save();
  if (flash >= 0 && flash <= FLASH_TICKS) {
    ctx.strokeStyle = p.goal;
    ctx.lineWidth = w + 6;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.globalAlpha = 1 - flash / (FLASH_TICKS + 1);
    ctx.beginPath();
    pts.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = w;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  pts.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
  ctx.stroke();
  // Joints: the base pin and every revolute joint.
  ctx.fillStyle = p.bg;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  for (let i = 0; i < pts.length; i++) {
    const r = i === 0 ? 5 : handles && i > 0 ? 7 : 4;
    ctx.beginPath();
    ctx.arc(sx(v, pts[i].x), sy(v, pts[i].y), r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

/** Hitch's chassis and trailer as filled rectangles, with the hitch tongue and steered wheels. */
export function drawCar(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, car: Hitch, q: { body: { x: number; y: number; theta: number }; trailer: number | null }, flash: number, color: string = p.robot) {
  const parts = car.footprint(q);
  fillPolygons(ctx, v, [parts[0]], color, 0.95);
  if (parts[1]) {
    fillPolygons(ctx, v, [parts[1]], color, 0.7);
    const axle = car.trailerAxle(q);
    if (axle) {
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(sx(v, q.body.x), sy(v, q.body.y));
      ctx.lineTo(sx(v, axle.x), sy(v, axle.y));
      ctx.stroke();
      ctx.restore();
    }
  }
  // Nose marker so the heading reads.
  const f = car.frontAxle(q);
  ctx.save();
  ctx.fillStyle = p.bg;
  ctx.beginPath();
  ctx.arc(sx(v, f.x), sy(v, f.y), 3, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
  if (flash >= 0 && flash <= FLASH_TICKS) {
    ctx.save();
    ctx.globalAlpha = 1 - flash / (FLASH_TICKS + 1);
    strokePolygons(ctx, v, parts, p.goal, 3);
    ctx.restore();
  }
}

export function drawWorldLabels(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, labels: { x: number; y: number; text: string }[]) {
  for (const l of labels) label(ctx, l.text, sx(v, l.x), sy(v, l.y), p.ink, { size: 10, align: 'center', weight: 600 });
}
