'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawPath, drawSegments, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { APARTMENT, WORLDS, type PolyWorld } from '@/lib/robots/worlds';
import { RUSTY } from '@/lib/robots/rusty';
import type { Tn } from '@/lib/robots/reach';
import { TOUR_HZ, tourCheckers, tourInit, tourStep, type TourOverrides, type TourState } from '@/lib/robots/tour';
import { pointsBounds } from '@/lib/collide/collision';
import type { Point2 } from '@/lib/sim/world';
import { drawArm, drawCar, drawWorldLabels, fillPolygons, flashRing } from './three-robot-dashboard';

/**
 * w2.3 — the World Atlas: the three worlds as named places.
 *
 * The Apartment (rooms A–E and the corridor, inherited from the sister book),
 * the Workbench (the block, the post, the shelf), and the Lot (bays 1–4 and two
 * pillars), with the labels the prose uses for twenty-one chapters. The
 * resident robot runs its tour; click anywhere to teleport it there (the
 * checker refuses configurations in collision and flashes instead); hover to
 * read workspace coordinates, the footprint's bounding box, and D(x).
 *
 * The misconception killed: "a world is a bitmap". It is polylines and
 * polygons with names; the Chapter 7 grid is built *from* it, not the other
 * way round.
 */

type WorldKey = 'apartment' | 'workbench' | 'lot';
const NAMES: Record<WorldKey, string> = { apartment: 'The Apartment', workbench: 'The Workbench', lot: 'The Lot' };
const RESIDENT: Record<WorldKey, string> = { apartment: 'Rusty', workbench: 'Reach', lot: 'Hitch' };

export function WorldAtlas() {
  const [which, setWhich] = useState<WorldKey>('workbench');
  const [hover, setHover] = useState<Point2 | null>(null);
  const [refused, setRefused] = useState(0);
  const checkers = useMemo(() => tourCheckers(), []);
  const held = useRef<TourOverrides>({});

  const init = useCallback((seed: number): TourState => {
    held.current = {};
    return tourInit(seed);
  }, []);
  const step = useCallback((s: TourState, tick: number) => tourStep(s, tick, checkers, held.current), [checkers]);
  const sim = useSimulation<TourState>({ init, step, fps: TOUR_HZ, initialSeed: 7 });
  const s = sim.state;
  const world: PolyWorld = WORLDS[which];

  // ---- teleport on click, hover readout -------------------------------------------
  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const pt = { x: w[0], y: w[1] };
      if (phase === 'move') return setHover(pt);
      if (phase !== 'down') return;
      if (which === 'apartment') {
        if (!checkers.rustyChecker.isFree(pt)) return setRefused((n) => n + 1);
        sim.setState((st) => {
          st.rusty.q = { pose: { ...st.rusty.q.pose, x: pt.x, y: pt.y }, wheelAngles: st.rusty.q.wheelAngles };
          st.rusty.trail = [];
          return st;
        });
        held.current = { ...held.current, rusty: { v: 0, omega: 0 } };
      } else if (which === 'workbench') {
        // Teleport the *tip*: two IK solutions; take the first free one.
        const arm = checkers.reach;
        const sols = arm.ik(pt).filter((q): q is [number, number] => q !== null);
        const free = sols.find((q) => checkers.reachChecker.isFree(q));
        if (!free) return setRefused((n) => n + 1);
        held.current = { ...held.current, reach: free };
        sim.setState((st) => {
          st.reach.trail = [];
          return st;
        });
      } else {
        const q = checkers.hitch.cfg(pt.x, pt.y, s.hitch.q.body.theta);
        if (!checkers.hitchChecker.isFree(q)) return setRefused((n) => n + 1);
        sim.setState((st) => {
          st.hitch.q = q;
          st.hitch.trail = [];
          return st;
        });
        held.current = { ...held.current, hitch: { v: 0, phi: 0 } };
      }
    },
    [which, checkers, sim, s.hitch.q.body.theta],
  );

  // ---- hover readout ---------------------------------------------------------------------
  const readout = useMemo(() => {
    if (!hover) return null;
    const scene = which === 'apartment' ? checkers.rustyChecker.scene : which === 'workbench' ? checkers.reachChecker.scene : checkers.hitchChecker.scene;
    const D = scene.distancePoint(hover.x, hover.y).dist;
    let bbox: { minX: number; minY: number; maxX: number; maxY: number } | null = null;
    if (which === 'apartment') {
      const r = RUSTY.bodyRadius;
      bbox = { minX: hover.x - r, minY: hover.y - r, maxX: hover.x + r, maxY: hover.y + r };
    } else if (which === 'workbench') {
      const q: Tn = held.current.reach ?? s.reach.q;
      bbox = pointsBounds(checkers.reach.fk(q));
    } else {
      bbox = pointsBounds(checkers.hitch.footprint(s.hitch.q).flat());
    }
    return { D, bbox };
  }, [hover, which, checkers, s.reach.q, s.hitch.q]);

  // ---- drawing ------------------------------------------------------------------------------
  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, which === 'workbench' ? 0.5 : 1);
      if (which === 'apartment') {
        drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
        drawPath(ctx, v, s.rusty.trail, p.robot, { lineWidth: 1.5, alpha: 0.5 });
        const pose = s.rusty.q.pose;
        const r = Math.max(sl(v, RUSTY.bodyRadius), 4);
        ctx.save();
        ctx.fillStyle = p.robot;
        ctx.beginPath();
        ctx.arc(sx(v, pose.x), sy(v, pose.y), r, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
        flashRing(ctx, p, s.rusty.flash, sx(v, pose.x), sy(v, pose.y), r + 4);
      } else {
        drawSegments(ctx, v, world.walls, p.wall, 2.5);
        if (world.markings) drawSegments(ctx, v, world.markings, p.unknown, 1.5);
        fillPolygons(ctx, v, world.obstacles.map((o) => o.poly), p.obstacle);
        if (which === 'workbench') {
          drawPath(ctx, v, s.reach.trail, p.robot, { lineWidth: 1.2, alpha: 0.5 });
          drawArm(ctx, v, p, checkers.reach, held.current.reach ?? s.reach.q, s.reach.flash, false);
        } else {
          drawPath(ctx, v, s.hitch.trail, p.robot, { lineWidth: 1.5, alpha: 0.5 });
          drawCar(ctx, v, p, checkers.hitch, s.hitch.q, s.hitch.flash);
        }
      }
      if (world.labels) drawWorldLabels(ctx, v, p, world.labels);

      // Hover: coordinates, R(q)'s bounding box, D(x).
      if (hover && readout) {
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.setLineDash([3, 3]);
        ctx.lineWidth = 1;
        const b = readout.bbox;
        if (b) ctx.strokeRect(sx(v, b.minX), sy(v, b.maxY), sl(v, b.maxX - b.minX), sl(v, b.maxY - b.minY));
        ctx.setLineDash([]);
        ctx.fillStyle = p.graph;
        ctx.beginPath();
        ctx.arc(sx(v, hover.x), sy(v, hover.y), 3, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
        label(ctx, `x = (${hover.x.toFixed(2)}, ${hover.y.toFixed(2)})   D(x) = ${readout.D.toFixed(3)} m`, 12, v.height - 14, p.ink, { size: 11, weight: 600 });
      }
      label(ctx, `${NAMES[which]} · resident: ${RESIDENT[which]}`, 12, 16, p.ink, { size: 11, weight: 700 });
    },
    [which, world, s, hover, readout, checkers],
  );

  const aspect = which === 'apartment' ? 12 / 9 : which === 'workbench' ? 1.3 : 16 / 10;

  return (
    <WidgetFrame
      id="w2.3"
      title="World Atlas"
      teaches="A world is not a bitmap. It is polylines and polygons with names, and every grid in the book is built from it."
      colorKey={['robot', 'obstacle', 'graph']}
      caption={
        <>
          The three worlds with the names the rest of the book uses. The Apartment's rooms A–E and
          its long corridor are the sister book's; the Workbench holds <em>the block</em> (the
          worked example's square), <em>the post</em>, and <em>the shelf</em>; the Lot has bays 1–4
          along the north curb and two pillars. Click to teleport the resident robot — a click into
          an obstacle is refused ({refused} refused so far), because R(q) would intersect it. Hover
          to read coordinates, the footprint's bounding box, and the clearance D(x). Nothing here is
          a pixel: each wall is a segment, each obstacle a polygon, and that is why the geometry
          queries of this chapter are exact.
        </>
      }
    >
      <SimCanvas
        world={world.bounds}
        draw={draw}
        deps={[sim.tick, which, hover, readout]}
        aspect={aspect}
        padding={0.3}
        onPointer={onPointer}
        cursor="crosshair"
        ariaLabel={`${NAMES[which]} with its named places labeled and its resident robot ${RESIDENT[which]} moving through it.`}
      />
      <ControlPanel columns={1} title="World">
        <ButtonRow>
          {(['apartment', 'workbench', 'lot'] as WorldKey[]).map((k) => (
            <ActionButton key={k} emphasis={which === k} onClick={() => setWhich(k)}>
              {NAMES[k]} · {RESIDENT[k]}
            </ActionButton>
          ))}
          <span className="font-ui text-[0.72rem] text-fd-muted-foreground">
            {which === 'apartment' ? `${APARTMENT.walls.length} wall segments` : `${world.walls.length} walls, ${world.obstacles.length} polygons: ${world.obstacles.map((o) => o.name).join(', ')}`}
          </span>
        </ButtonRow>
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
