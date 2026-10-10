'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Point2 } from '@/lib/sim/world';
import {
  ContaminationField,
  PURSUER_START,
  STATIC_PURSUER,
  THREE_POCKETS,
  TWO_POCKETS,
  conservativeRegions,
  edgeCount,
  gapsOf,
  pocketTour,
  solveInformationGraph,
  visibilityPolygon,
  type PursuitSolution,
  type RegionDecomposition,
} from '@/lib/cells/pursuit';
import { pixelAt } from '@/lib/cells/slices';
import { isFreePoint, worldBounds } from '@/lib/cells/world';
import { chip, drawCellWorld, fillPoly, hatchPixels, marker, segment } from './shared';

/**
 * w10.2 — Pursuit-Evasion (Choset §6.3).
 *
 * A point pursuer in a hub with U-shaped pockets. Its visibility polygon V(x)
 * is shaded; everything the evader could still be in is hatched in the
 * C-obstacle amber ("unsafe to assume clear"); each gap edge of V(x) carries
 * its label, 0 clear or 1 contaminated, and B(x) is printed. Auto-solve runs
 * breadth-first search on the information graph G_i and replays the strategy
 * it finds. Two pockets: one pursuer suffices. Three pockets: G_i has no
 * all-zero node — the replay shows a naive patrol watching cleared pockets be
 * recontaminated — until a second, static pursuer holds the hub.
 */

type WorldKey = 'two' | 'three';

interface State {
  i: number;
}

export function PursuitEvasion() {
  const [worldKey, setWorldKey] = useState<WorldKey>('two');
  const [second, setSecond] = useState(false);
  const [showRegions, setShowRegions] = useState(false);
  const [manual, setManual] = useState(false);
  const [x, setX] = useState<Point2>(PURSUER_START);
  const [solution, setSolution] = useState<PursuitSolution | null>(null);
  const [solving, setSolving] = useState(false);
  const world = worldKey === 'two' ? TWO_POCKETS : THREE_POCKETS;
  const staticP = second ? STATIC_PURSUER : null;
  const bounds = useMemo(() => worldBounds(world), [world]);
  const field = useMemo(() => new ContaminationField(world, 0.125), [world]);
  const [, setVersion] = useState(0);
  const dec = useRef<RegionDecomposition | null>(null);

  // The path being replayed: the G_i solution if one exists, else a naive patrol.
  const route = useMemo(() => {
    if (solution?.found) return solution.path;
    return pocketTour(world, field.raster, PURSUER_START);
  }, [solution, world, field]);

  const pursuers = useCallback((p: Point2) => (staticP ? [p, staticP] : [p]), [staticP]);

  const restart = useCallback(() => {
    field.reset();
    field.observe(pursuers(PURSUER_START));
    setX(PURSUER_START);
    setVersion((n) => n + 1);
  }, [field, pursuers]);

  // Solve G_i whenever the world or the second pursuer changes (off the render path).
  useEffect(() => {
    let cancelled = false;
    setSolving(true);
    setSolution(null);
    const id = setTimeout(() => {
      const d = conservativeRegions(world, 0.125, staticP);
      dec.current = d;
      const sol = solveInformationGraph(d, PURSUER_START);
      if (!cancelled) {
        setSolution(sol);
        setSolving(false);
      }
    }, 30);
    return () => {
      cancelled = true;
      clearTimeout(id);
    };
  }, [world, staticP]);

  const init = useCallback((): State => ({ i: 0 }), []);
  const step = useCallback(
    (s: State): State => {
      if (manual || solving) return s;
      if (s.i + 1 >= route.length + 40) {
        restart();
        return { i: 0 };
      }
      if (s.i + 1 < route.length) {
        field.move(pursuers(route[s.i]), pursuers(route[s.i + 1]));
        setX(route[s.i + 1]);
      }
      return { i: s.i + 1 };
    },
    [manual, solving, route, field, pursuers, restart],
  );
  const sim = useSimulation<State>({ init, step, fps: 24, initialSeed: 6 });
  const simReset = sim.reset;
  useEffect(() => {
    restart();
    simReset();
  }, [route, restart, simReset]);

  const vp = useMemo(() => visibilityPolygon(field.world, x, field.vertices), [field, x]);
  const vpStatic = useMemo(() => (staticP ? visibilityPolygon(field.world, staticP, field.vertices) : null), [field, staticP]);
  const gaps = useMemo(() => gapsOf(vp), [vp]);

  const pressed = useRef(false);
  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') {
        pressed.current = false;
        return;
      }
      if (phase === 'down') pressed.current = true;
      if (!pressed.current) return; // hovering does not steer
      const p = { x: w[0], y: w[1] };
      if (!isFreePoint(world, p)) return;
      if (phase === 'down') setManual(true);
      field.move(pursuers(x), pursuers(p));
      setX(p);
      setVersion((n) => n + 1);
    },
    [world, field, pursuers, x],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const r = field.raster;
      if (vpStatic) fillPoly(ctx, v, vpStatic.pts, p.accent, 0.1);
      fillPoly(ctx, v, vp.pts, p.accent, 0.16);
      hatchPixels(ctx, v, r, (k) => field.contaminated[k] === 1, p.cobstacle);
      drawCellWorld(ctx, v, p, world);
      // Conservative regions and their adjacency (the cells of §6.3).
      const d = dec.current;
      if (showRegions && d) {
        const cur = d.regionOf[pixelAt(r, x)] ?? -1;
        for (const e of d.adjacency) segment(ctx, v, d.regions[e.A].centroid, d.regions[e.B].centroid, p.graph, 0.6, undefined, 0.35);
        for (const rg of d.regions) marker(ctx, sx(v, rg.centroid.x), sy(v, rg.centroid.y), rg.id === cur ? 4.5 : 1.8, rg.id === cur ? p.path : p.graph, p.bg);
      }
      // Gap edges and their labels: 1 if the shadow just behind the gap is contaminated.
      const labels = gaps.map((g) => field.gapLabel(g, field.lastVisible));
      gaps.forEach((g, gi) => {
        const lab = labels[gi];
        segment(ctx, v, g.near, g.far, lab ? p.cobstacle : p.accent, 2.4);
        const m = { x: (g.near.x + g.far.x) / 2, y: (g.near.y + g.far.y) / 2 };
        chip(ctx, `${lab}`, sx(v, m.x), sy(v, m.y), lab ? p.cobstacle : p.accent, p.bg, 11);
      });
      if (staticP) marker(ctx, sx(v, staticP.x), sy(v, staticP.y), 6, p.robot, p.bg, 'square');
      marker(ctx, sx(v, x.x), sy(v, x.y), 6.5, p.robot, p.bg);
      const bits = labels.join('');
      const area = field.contaminatedArea();
      label(ctx, `B(x) = {${bits || '∅'}} · contaminated ${area.toFixed(1)} m²${area === 0 ? ' — evader caught' : ''}`, 10, 14, p.ink, { size: 10.5, weight: 600 });
      const status = solving
        ? 'searching G_i…'
        : solution?.found
          ? `G_i: all-zero node found after ${solution.statesExplored} states · strategy crosses ${solution.regionPath.length - 1} regions${manual ? ' · manual' : ' · replaying'}`
          : `G_i: ${solution?.statesExplored ?? 0} states, no all-zero node — no single-pursuer strategy${manual ? ' · manual' : ' · replaying a naive patrol'}`;
      label(ctx, status, 10, v.height - 12, p.ink, { size: 10 });
    },
    [field, vp, vpStatic, world, showRegions, x, gaps, staticP, solving, solution, manual],
  );

  return (
    <WidgetFrame
      id="w10.2"
      title="Pursuit-Evasion"
      teaches="Seeing a region once does not clear it: information, not geometry, is the state. A pocket cleared and then left unwatched is recontaminated, and in three pockets no single pursuer's path ever drives every gap label to zero."
      colorKey={['obstacle', 'cobstacle', 'graph', 'robot']}
      caption={
        <>
          The orange pursuer's visibility polygon V(x) is shaded; hatched amber is everywhere the
          evader may still be. Each gap edge of V(x) is labelled 0 (clear behind) or 1
          (contaminated), and B(x) lists them. With two pockets, breadth-first search on the
          information graph finds a node with every label zero and the replay catches the evader.
          With three, G_i has no such node, and the naive patrol shown instead clears each pocket only
          to watch the hub recontaminate it; add the second (square) pursuer, static at the hub, and
          the search succeeds — Choset's static-pursuer argument for O(log n) pursuers. Drag in the
          free space to steer the pursuer by hand.
        </>
      }
    >
      <SimCanvas world={bounds} draw={draw} deps={[sim.tick, x, showRegions, solution, solving, manual]} aspect={4 / 3} padding={0.3} onPointer={onPointer} cursor="crosshair" ariaLabel="A square hub with U-shaped pockets; an orange dot's visibility region is shaded and hatched amber regions mark where an evader could hide." />
      <ControlPanel columns={2}>
        <ButtonRow>
          <ActionButton
            emphasis={worldKey === 'two'}
            onClick={() => {
              setWorldKey('two');
              setManual(false);
            }}
          >
            two pockets ({edgeCount(TWO_POCKETS)} edges)
          </ActionButton>
          <ActionButton
            emphasis={worldKey === 'three'}
            onClick={() => {
              setWorldKey('three');
              setManual(false);
            }}
          >
            three pockets ({edgeCount(THREE_POCKETS)} edges)
          </ActionButton>
        </ButtonRow>
        <ButtonRow>
          <Toggle
            label="Second pursuer (static, at the hub)"
            checked={second}
            onChange={(b) => {
              setSecond(b);
              setManual(false);
            }}
            role="robot"
          />
          <Toggle label="Conservative regions" checked={showRegions} onChange={setShowRegions} role="graph" />
          <ActionButton
            onClick={() => {
              setManual(false);
              restart();
              sim.reset();
            }}
          >
            auto-solve (replay)
          </ActionButton>
        </ButtonRow>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={() => {
          setManual(false);
          restart();
          sim.reset();
        }}
        tick={sim.state.i}
      />
    </WidgetFrame>
  );
}
