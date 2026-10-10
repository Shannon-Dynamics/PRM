'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT, type Point2 } from '@/lib/sim/world';
import { cellCenter, gridFromWorld } from '@/lib/search/grid';
import { featureTransform, ridgeMaskFromFeatures } from '@/lib/roadmap/grid';
import { alongPath, retract, retractToPoint, windingNumber, type RetractResult } from '@/lib/roadmap/retract';
import { ISLAND_CENTER, KITCHEN_ISLAND, apartmentWithIslandQuery, loopAround } from '@/lib/roadmap/examples';
import { fillPoly } from '@/components/ch/ch04/shared';
import { marker } from '@/components/ch/ch07/shared';

/**
 * w8.3 — Retract Candy.
 *
 * The Apartment's free space melts onto its GVD: every lattice point follows
 * its gradient-ascent path c_q(t), and the headline scrub is the homotopy time
 * t ∈ [0, 1] of H(q, t). Two loops ride along — one around the kitchen
 * island, one in an empty room — and their winding numbers about the island
 * are printed at t = 0 and at t: for a deformation retraction they never
 * disagree, so the hole survives. The "retract to a point" toggle replaces H
 * by straight lines to one point: still a retraction onto that point, still
 * continuous — and the loop around the island has to cross the island to get
 * there, which the red marks show.
 */

const SPACING = 0.3;
const POINT_TARGET: Point2 = { x: 6, y: 4.4 };

interface State {
  t: number;
  dir: 1 | -1;
  hold: number;
}

export function RetractCandy() {
  const [scrub, setScrub] = useState<number | null>(null);
  const [toPoint, setToPoint] = useState(false);
  const [showPaths, setShowPaths] = useState(false);
  const [showGvd, setShowGvd] = useState(true);
  const [loopCenter, setLoopCenter] = useState<Point2>({ x: 2, y: 2 });

  const dq = useMemo(() => apartmentWithIslandQuery(), []);
  const isFree = useCallback((p: Point2) => p.x > 0 && p.x < 12 && p.y > 0 && p.y < 9 && !dq.inside(p) && dq.nearest([p.x, p.y]).d > 0.04, [dq]);

  // The lattice and its ascent paths, once.
  const lattice = useMemo(() => {
    const pts: Point2[] = [];
    for (let y = SPACING / 2; y < 9; y += SPACING) for (let x = SPACING / 2; x < 12; x += SPACING) if (isFree({ x, y }) && dq.nearest([x, y]).d > 0.12) pts.push({ x, y });
    return pts;
  }, [dq, isFree]);
  const latticePaths = useMemo(() => lattice.map((p) => retract(dq, p, { step: 0.03, bounds: APARTMENT.bounds }).path), [lattice, dq]);
  const islandLoop = useMemo(() => loopAround(ISLAND_CENTER, 0.55, 48), []);
  const islandPaths = useMemo(() => islandLoop.map((p) => retract(dq, p, { step: 0.02, bounds: APARTMENT.bounds })), [islandLoop, dq]);
  const roomLoop = useMemo(() => loopAround(loopCenter, 0.45, 48).filter(isFree), [loopCenter, isFree]);
  const roomPaths = useMemo(() => roomLoop.map((p) => retract(dq, p, { step: 0.02, bounds: APARTMENT.bounds })), [roomLoop, dq]);
  const straight = useMemo(() => ({ island: retractToPoint(islandLoop, POINT_TARGET, isFree), room: retractToPoint(roomLoop, POINT_TARGET, isFree), lattice: retractToPoint(lattice, POINT_TARGET, isFree) }), [islandLoop, roomLoop, lattice, isFree]);

  // The GVD, for reference: the feature-jump ridge of a 0.1 m raster with the island added.
  const gvdCells = useMemo(() => {
    const world = { ...APARTMENT, walls: [...APARTMENT.walls, ...KITCHEN_ISLAND.map((p, i) => ({ x1: p.x, y1: p.y, x2: KITCHEN_ISLAND[(i + 1) % KITCHEN_ISLAND.length].x, y2: KITCHEN_ISLAND[(i + 1) % KITCHEN_ISLAND.length].y }))] };
    const g = gridFromWorld(world, 0.1, 0.051);
    const mask = ridgeMaskFromFeatures(g, featureTransform(g));
    const pts: Point2[] = [];
    for (let k = 0; k < mask.length; k++) if (mask[k] && !g.occupied[k]) pts.push(cellCenter(g, k));
    return pts;
  }, []);

  const init = useCallback((): State => ({ t: 0, dir: 1, hold: 0 }), []);
  const step = useCallback((s: State): State => {
    if (s.hold > 0) return { ...s, hold: s.hold - 1 };
    const t = s.t + s.dir * 0.012;
    if (t >= 1) return { t: 1, dir: -1, hold: 45 };
    if (t <= 0) return { t: 0, dir: 1, hold: 30 };
    return { ...s, t };
  }, []);
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 83 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [toPoint]);

  const t = scrub ?? sim.state.t;
  const at = (paths: RetractResult[] | Point2[][], mode: 'ascent' | 'point', pts: Point2[], sp: { at: (t: number) => Point2[] }) =>
    mode === 'point' ? sp.at(t) : (paths as (RetractResult | Point2[])[]).map((r) => alongPath(Array.isArray(r) ? r : r.path, t));
  const mode = toPoint ? 'point' : 'ascent';
  const latticeNow = useMemo(() => at(latticePaths, mode, lattice, straight.lattice), [latticePaths, mode, lattice, straight, t]);
  const islandNow = useMemo(() => at(islandPaths, mode, islandLoop, straight.island), [islandPaths, mode, islandLoop, straight, t]);
  const roomNow = useMemo(() => at(roomPaths, mode, roomLoop, straight.room), [roomPaths, mode, roomLoop, straight, t]);
  const w0 = windingNumber(islandLoop, ISLAND_CENTER);
  const wt = windingNumber(islandNow, ISLAND_CENTER);
  const r0 = windingNumber(roomLoop, ISLAND_CENTER);
  const rt = windingNumber(roomNow, ISLAND_CENTER);
  const pierced = useMemo(() => (toPoint ? islandNow.filter((p) => !isFree(p)).length + roomNow.filter((p) => !isFree(p)).length : 0), [toPoint, islandNow, roomNow, isFree]);

  const onPointer = useCallback((w: [number, number], phase: 'down' | 'move' | 'up') => {
    if (phase !== 'down') return;
    const c = { x: w[0], y: w[1] };
    if (isFree(c) && dq.nearest([c.x, c.y]).d > 0.5) setLoopCenter(c);
  }, [isFree, dq]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      if (showGvd) {
        ctx.save();
        ctx.fillStyle = p.graph;
        ctx.globalAlpha = 0.18;
        const cw = Math.max(2, sl(v, 0.1));
        for (const c of gvdCells) ctx.fillRect(sx(v, c.x - 0.05), sy(v, c.y + 0.05), cw, cw);
        ctx.restore();
      }
      drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
      fillPoly(ctx, v, KITCHEN_ISLAND, p.obstacle);
      if (showPaths && !toPoint) {
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.globalAlpha = 0.25;
        ctx.lineWidth = 1;
        for (const path of latticePaths) {
          ctx.beginPath();
          path.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
          ctx.stroke();
        }
        ctx.restore();
      }
      // The melting lattice.
      ctx.save();
      ctx.fillStyle = p.graph;
      ctx.globalAlpha = 0.75;
      for (const q of latticeNow) {
        ctx.beginPath();
        ctx.arc(sx(v, q.x), sy(v, q.y), 2, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
      // Loops: around the island (orange) and in a room (green).
      const drawLoop = (pts: Point2[], color: string) => {
        if (pts.length < 2) return;
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = 2.2;
        ctx.beginPath();
        pts.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q.x), sy(v, q.y)) : ctx.lineTo(sx(v, q.x), sy(v, q.y))));
        ctx.closePath();
        ctx.stroke();
        ctx.restore();
        if (toPoint) for (const q of pts) if (!isFree(q)) marker(ctx, sx(v, q.x), sy(v, q.y), 3.5, p.goal);
      };
      drawLoop(islandNow, p.robot);
      drawLoop(roomNow, p.start);
      if (toPoint) marker(ctx, sx(v, POINT_TARGET.x), sy(v, POINT_TARGET.y), 6, p.ink, p.bg);
      label(ctx, `${toPoint ? 'straight lines to one point' : 'gradient ascent of D'} · t = ${t.toFixed(2)} · winding about the island: orange ${w0} → ${wt}, green ${r0} → ${rt}${toPoint && pierced ? ` · ${pierced} loop points inside walls or the island` : ''}`, 10, 14, toPoint && pierced ? p.goal : p.ink, { size: 10.5, weight: 600 });
      label(ctx, 'click an open spot to drop the green loop there', 10, v.height - 12, p.ink, { size: 9 });
    },
    [latticeNow, islandNow, roomNow, latticePaths, gvdCells, showGvd, showPaths, toPoint, t, w0, wt, r0, rt, pierced, isFree],
  );

  return (
    <WidgetFrame
      id="w8.3"
      title="Retract Candy"
      teaches="A skeleton is not any thinner version of the space. Only a deformation retract keeps the holes — and gradient ascent onto the GVD is one."
      colorKey={['obstacle', 'graph', 'robot', 'start', 'goal']}
      wide
      caption={
        <>
          Every blue dot is a free point of the Apartment moving along its own gradient-ascent path
          c<sub>q</sub>(t) toward the GVD (the faint blue skeleton); the scrub is the homotopy time t
          of H(q, t). The orange loop circles a kitchen island, the green one sits in an empty room.
          Their winding numbers about the island are printed at t = 0 and at the current t: 1 → 1 and
          0 → 0, at every t — the deformation retraction never tears a loop across a hole. Switch to
          <em> retract to a point</em>: the map is still continuous and still fixes its image, but the
          orange loop&rsquo;s points cross the island on their way (red marks) — a retraction, not a
          deformation retraction, and the loop class is lost.
        </>
      }
    >
      <SimCanvas world={APARTMENT.bounds} draw={draw} deps={[sim.tick, t, toPoint, showPaths, showGvd, loopCenter]} aspect={12 / 9} padding={0.3} onPointer={onPointer} cursor="pointer" ariaLabel="The Apartment floorplan with a lattice of blue dots sliding toward the blue skeleton of the free space; an orange loop around a kitchen island and a green loop in a room deform with them." />
      <ControlPanel columns={3} title="Headline: homotopy time t">
        <Slider label="t in H(q, t) (scrub; play to resume)" role="graph" value={t} min={0} max={1} step={0.01} onChange={(v) => { sim.pause(); setScrub(v); }} />
        <Toggle label="Retract to a point instead (straight lines)" role="goal" checked={toPoint} onChange={(v) => { setToPoint(v); setScrub(null); }} />
        <div className="flex flex-col gap-1.5">
          <Toggle label="Show the GVD" role="graph" checked={showGvd} onChange={setShowGvd} />
          <Toggle label="Show every ascent path" role="graph" checked={showPaths} onChange={setShowPaths} />
          <ButtonRow>
            <ActionButton onClick={() => setLoopCenter({ x: 2, y: 2 })}>green loop back to room A</ActionButton>
          </ButtonRow>
        </div>
      </ControlPanel>
      <div className="border-t border-fd-border px-3 py-2 font-mono text-[0.7rem] text-fd-muted-foreground tabular-nums">
        {lattice.length} lattice points · island loop lands {islandPaths.filter((r) => r.landed).length}/{islandLoop.length} on the GVD · straight-line retraction pierces obstacles on {straight.island.pierces} of {islandLoop.length} island-loop rays
      </div>
      <Transport playing={sim.playing} onToggle={() => { setScrub(null); sim.toggle(); }} onStep={sim.stepOnce} onReset={() => { setScrub(null); sim.reset(); }} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
