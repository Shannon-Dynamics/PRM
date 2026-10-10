'use client';

import { useCallback, useMemo, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle } from '@/components/sim/controls';
import { clear, drawGrid, drawSegments, drawWorld, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { normalizeAngle } from '@/lib/geom/se2';
import { Reach, fmtTn, type Tn } from '@/lib/robots/reach';
import { Hitch, type HitchCfg } from '@/lib/robots/hitch';
import { RUSTY } from '@/lib/robots/rusty';
import { APARTMENT, APARTMENT_SCENE, LOT, WORKBENCH, type PolyWorld } from '@/lib/robots/worlds';
import { Scene } from '@/lib/collide/scene';
import { ChainChecker, DiscChecker, FootprintChecker, type Collision, type R2, type Witness } from '@/lib/collide/collision';
import { drawArm, drawCar, fillPolygons, strokePolygons } from './three-robot-dashboard';

/**
 * w2.2 — Query Anatomy: what a collision checker actually returns.
 *
 * A frozen scene and one configuration. Four tabs ask the four questions a
 * planner may ask of geometry — intersects, distance, witness, swept — and
 * draw the answers. The misconception killed: "a collision checker returns a
 * boolean". It returns geometry: a distance (which hits zero exactly where the
 * fill turns red), a pair of nearest points (which jump between features as
 * the arm moves), and a verdict on a whole motion, not just its endpoints.
 *
 * The default scene is the Workbench with Choset's zero-thickness 2R arm, so
 * the numbers on screen at q = (π/4, π/2) are the chapter's worked numbers:
 * distance 0.0607 to the block, b* = (0.5, 1.0).
 */

type Tab = 'intersects' | 'distance' | 'witness' | 'swept';
type Robot = 'reach' | 'rusty' | 'hitch';

const TABS: { id: Tab; title: string }[] = [
  { id: 'intersects', title: 'intersects' },
  { id: 'distance', title: 'distance' },
  { id: 'witness', title: 'witness' },
  { id: 'swept', title: 'swept' },
];

interface Scenes {
  reach: { world: PolyWorld; scene: Scene };
  rusty: { world: PolyWorld; scene: Scene };
  hitch: { world: PolyWorld; scene: Scene };
}

export function QueryAnatomy() {
  const [tab, setTab] = useState<Tab>('distance');
  const [robot, setRobot] = useState<Robot>('reach');
  const [capsules, setCapsules] = useState(false);
  const [th, setTh] = useState<[number, number]>([Math.PI / 4, Math.PI / 2]);
  const [disc, setDisc] = useState<R2>({ x: 2.0, y: 2.0 });
  const [car, setCar] = useState<HitchCfg>({ body: { x: 8.0, y: 4.0, theta: 0.4 }, trailer: null });
  const [sweptStep, setSweptStep] = useState(0.9);
  const [drag, setDrag] = useState(false);

  const scenes = useMemo<Scenes>(
    () => ({
      reach: { world: WORKBENCH, scene: Scene.fromWorld(WORKBENCH) },
      rusty: { world: APARTMENT_SCENE, scene: Scene.fromWorld(APARTMENT_SCENE) },
      hitch: { world: LOT, scene: Scene.fromWorld(LOT) },
    }),
    [],
  );
  const arm = useMemo(() => (capsules ? Reach.workbench() : Reach.choset()), [capsules]);
  const hitch = useMemo(() => Hitch.car(), []);
  const reachChecker = useMemo(() => new ChainChecker(scenes.reach.scene, arm), [scenes, arm]);
  const discChecker = useMemo(() => new DiscChecker(scenes.rusty.scene, RUSTY.bodyRadius), [scenes]);
  const carChecker = useMemo(() => new FootprintChecker(scenes.hitch.scene, hitch), [scenes, hitch]);

  // The second configuration of the swept query: a straight chart-line step.
  const qb = useMemo(() => {
    if (robot === 'reach') return [th[0], normalizeAngle(th[1] + sweptStep)] as Tn;
    if (robot === 'rusty') return { x: disc.x + sweptStep * 1.4, y: disc.y } as R2;
    return { body: { x: car.body.x, y: car.body.y, theta: normalizeAngle(car.body.theta + sweptStep) }, trailer: null } as HitchCfg;
  }, [robot, th, disc, car, sweptStep]);

  // One generic evaluation for whichever robot is picked.
  const result = useMemo(() => {
    const run = <Q,>(c: Collision<Q>, a: Q, b: Q) => {
      const free = c.isFree(a);
      const w = c.witness(a);
      const d = c.distance(a);
      const endsFree = free && c.isFree(b);
      const swept = endsFree ? c.edgeFree(a, b) : false;
      // For the picture: sample the sweep and find the first colliding sample.
      const N = 24;
      let firstBad = -1;
      const samples: Q[] = [];
      for (let i = 0; i <= N; i++) {
        const q = (c as unknown as { interpolate: (a: Q, b: Q, s: number) => Q }).interpolate(a, b, i / N);
        samples.push(q);
        if (firstBad < 0 && !c.isFree(q)) firstBad = i;
      }
      return { free, w, d, endsFree, swept, samples, firstBad };
    };
    if (robot === 'reach') return run(reachChecker, th as Tn, qb as Tn);
    if (robot === 'rusty') return run(discChecker, disc, qb as R2);
    return run(carChecker, car, qb as HitchCfg);
  }, [robot, th, disc, car, qb, reachChecker, discChecker, carChecker]);

  const world = scenes[robot].world;

  // ---- pointer: drag the robot ------------------------------------------------
  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') return setDrag(false);
      if (phase === 'down') setDrag(true);
      else if (!drag) return;
      if (robot === 'reach') {
        // Drag the tip: closed-form IK, keeping the elbow solution nearest the current q.
        const target = { x: w[0], y: w[1] };
        const r = Math.hypot(target.x, target.y);
        const clamped = r > arm.reach() - 1e-6 ? { x: (target.x / r) * (arm.reach() - 1e-6), y: (target.y / r) * (arm.reach() - 1e-6) } : target;
        const sols = arm.ik(clamped).filter((s): s is [number, number] => s !== null);
        if (sols.length === 0) return;
        const cost = (s: Tn) => Math.abs(normalizeAngle(s[0] - th[0])) + Math.abs(normalizeAngle(s[1] - th[1]));
        sols.sort((a, b) => cost(a) - cost(b));
        setTh([sols[0][0], sols[0][1]]);
      } else if (robot === 'rusty') {
        setDisc({ x: Math.max(0.1, Math.min(11.9, w[0])), y: Math.max(0.1, Math.min(8.9, w[1])) });
      } else {
        setCar((c) => ({ ...c, body: { ...c.body, x: Math.max(0.5, Math.min(15.5, w[0])), y: Math.max(0.5, Math.min(9.5, w[1])) } }));
      }
    },
    [drag, robot, arm, th],
  );

  // ---- drawing -------------------------------------------------------------------
  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, robot === 'reach' ? 0.5 : 1);
      if (robot === 'rusty') drawWorld(ctx, v, { ...APARTMENT, landmarks: undefined }, p);
      else {
        drawSegments(ctx, v, world.walls, p.wall, 2.5);
        if (world.markings) drawSegments(ctx, v, world.markings, p.unknown, 1.5);
        fillPolygons(ctx, v, world.obstacles.map((o) => o.poly), p.obstacle);
      }

      const bodyColor = result.free ? p.robot : p.goal;

      // Swept tab: ghosts along the chart line, the first collision highlighted.
      if (tab === 'swept') {
        result.samples.forEach((q, i) => {
          const bad = result.firstBad >= 0 && i === result.firstBad;
          const alpha = bad ? 0.95 : 0.12;
          const color = bad ? p.goal : p.robot;
          if (i === 0 || i === result.samples.length - 1) return;
          ctx.save();
          ctx.globalAlpha = alpha;
          drawFootprint(ctx, v, p, robot, arm, hitch, q, color);
          ctx.restore();
        });
        ctx.save();
        ctx.globalAlpha = 0.45;
        drawFootprint(ctx, v, p, robot, arm, hitch, qb, result.endsFree ? p.robot : p.goal);
        ctx.restore();
      }

      // The configuration itself.
      drawFootprint(ctx, v, p, robot, arm, hitch, robot === 'reach' ? th : robot === 'rusty' ? disc : car, bodyColor, true);

      // Distance / witness overlays.
      const w = result.w;
      if (w && (tab === 'distance' || tab === 'witness') && w.dist > 0) {
        ctx.save();
        ctx.strokeStyle = p.graph;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.moveTo(sx(v, w.onRobot.x), sy(v, w.onRobot.y));
        ctx.lineTo(sx(v, w.onObstacle.x), sy(v, w.onObstacle.y));
        ctx.stroke();
        ctx.restore();
        if (tab === 'distance') {
          const mx = (sx(v, w.onRobot.x) + sx(v, w.onObstacle.x)) / 2;
          const my = (sy(v, w.onRobot.y) + sy(v, w.onObstacle.y)) / 2;
          label(ctx, `${w.dist.toFixed(4)} m`, mx + 8, my - 8, p.graph, { size: 11, weight: 700 });
        }
        if (tab === 'witness') drawWitness(ctx, v, p, w);
      } else if (w && tab === 'witness' && w.dist === 0) {
        label(ctx, 'intersecting: distance 0, witness = a contact point', sx(v, w.onRobot.x) + 10, sy(v, w.onRobot.y) - 10, p.goal, { size: 10, weight: 600 });
        drawWitness(ctx, v, p, w);
      }

      // Headline verdict.
      const verdict =
        tab === 'intersects'
          ? `intersects: ${!result.free}`
          : tab === 'distance'
            ? `distance: ${result.d.toFixed(4)} m`
            : tab === 'witness'
              ? w
                ? `a* = (${w.onRobot.x.toFixed(4)}, ${w.onRobot.y.toFixed(4)})   b* = (${w.onObstacle.x.toFixed(4)}, ${w.onObstacle.y.toFixed(4)})`
                : 'no witness'
              : result.endsFree
                ? `swept: ${result.swept}${!result.swept ? '   (both endpoints free — a collision strictly between them)' : ''}`
                : 'swept: false (an endpoint is already in collision)';
      label(ctx, verdict, 12, 16, tab === 'swept' && !result.swept ? p.goal : p.ink, { size: 12, weight: 700 });
    },
    [robot, world, result, tab, arm, hitch, th, disc, car, qb],
  );

  const aspect = robot === 'reach' ? 1.25 : robot === 'rusty' ? 12 / 9 : 16 / 10;

  return (
    <WidgetFrame
      id="w2.2"
      title="Query Anatomy"
      teaches="A collision checker does not return a boolean. It returns geometry: a distance, two witness points, and a verdict on a whole motion."
      colorKey={['robot', 'obstacle', 'graph', 'goal']}
      caption={
        <>
          One frozen configuration, four questions. <em>Intersects</em> fills the body red the instant
          it touches anything. <em>Distance</em> draws the shortest segment to the scene; slide θ₂ and
          watch the number fall to exactly zero where the fill turns red. <em>Witness</em> labels the
          nearest pair a*, b*: drag the tip and see b* jump from a corner to an edge when the nearest
          feature changes. <em>Swept</em> ghosts a second configuration one chart-step away and shades
          the motion between them; the red ghost is a collision strictly between two free endpoints,
          which is why no planner may check endpoints alone. At the default pose the distance is the
          chapter's 0.0607 m and b* is the block's corner (0.5, 1.0).
        </>
      }
    >
      <div className="flex flex-wrap gap-1 border-b border-fd-border px-3 py-2">
        {TABS.map((t) => (
          <button
            key={t.id}
            type="button"
            onClick={() => setTab(t.id)}
            className={`rounded-sm border px-2.5 py-1 font-mono text-xs transition-colors ${tab === t.id ? 'border-fd-primary bg-fd-primary text-fd-primary-foreground' : 'border-fd-border bg-fd-card hover:bg-fd-accent'}`}
            aria-pressed={tab === t.id}
          >
            {t.title}
          </button>
        ))}
        <span className="ml-auto self-center font-mono text-[0.7rem] text-fd-muted-foreground">
          {robot === 'reach' ? fmtTn(th) : robot === 'rusty' ? `R2 { x: ${disc.x.toFixed(3)}, y: ${disc.y.toFixed(3)} }` : Hitch.fmt(car, 2)}
        </span>
      </div>

      <SimCanvas
        world={world.bounds}
        draw={draw}
        deps={[result, tab, robot, th, disc, car, qb, arm]}
        aspect={aspect}
        padding={0.25}
        onPointer={onPointer}
        cursor={drag ? 'grabbing' : 'grab'}
        ariaLabel="A frozen scene with one robot configuration; the selected query's answer is drawn as a distance segment, witness points, or a shaded swept motion."
      />

      <ControlPanel columns={2}>
        {robot === 'reach' ? (
          <Slider label="θ₂ (headline)" role="robot" value={th[1]} min={-Math.PI} max={Math.PI} step={0.005} unit="rad" onChange={(v) => setTh([th[0], v])} />
        ) : robot === 'rusty' ? (
          <Slider label="x (headline)" role="robot" value={disc.x} min={0.2} max={11.8} step={0.01} unit="m" onChange={(v) => setDisc({ ...disc, x: v })} />
        ) : (
          <Slider label="heading θ (headline)" role="robot" value={car.body.theta} min={-Math.PI} max={Math.PI} step={0.005} unit="rad" onChange={(v) => setCar({ ...car, body: { ...car.body, theta: v } })} />
        )}
        <Slider label="swept step (chart distance)" role="graph" value={sweptStep} min={0.05} max={1.6} step={0.05} onChange={setSweptStep} help="How far the second configuration of the swept query is along the straight chart line." />
      </ControlPanel>

      <details className="border-t border-fd-border">
        <summary className="eyebrow cursor-pointer px-3 py-2 select-none">Advanced: robot, other coordinates, link thickness</summary>
        <ControlPanel columns={3}>
          <ButtonRow>
            {(['reach', 'rusty', 'hitch'] as Robot[]).map((r) => (
              <ActionButton key={r} emphasis={robot === r} onClick={() => setRobot(r)}>
                {r === 'reach' ? 'Reach · Workbench' : r === 'rusty' ? 'Rusty · Apartment' : 'Hitch · Lot'}
              </ActionButton>
            ))}
          </ButtonRow>
          {robot === 'reach' ? (
            <>
              <Slider label="θ₁" role="robot" value={th[0]} min={-Math.PI} max={Math.PI} step={0.005} unit="rad" onChange={(v) => setTh([v, th[1]])} />
              <Toggle label="3 cm capsule links (the Workbench default)" checked={capsules} onChange={setCapsules} />
            </>
          ) : robot === 'rusty' ? (
            <Slider label="y" role="robot" value={disc.y} min={0.2} max={8.8} step={0.01} unit="m" onChange={(v) => setDisc({ ...disc, y: v })} />
          ) : (
            <>
              <Slider label="x" role="robot" value={car.body.x} min={0.5} max={15.5} step={0.05} unit="m" onChange={(v) => setCar({ ...car, body: { ...car.body, x: v } })} />
              <Slider label="y" role="robot" value={car.body.y} min={0.5} max={9.5} step={0.05} unit="m" onChange={(v) => setCar({ ...car, body: { ...car.body, y: v } })} />
            </>
          )}
        </ControlPanel>
      </details>
    </WidgetFrame>
  );
}

function drawFootprint(
  ctx: CanvasRenderingContext2D,
  v: Viewport,
  p: Palette,
  robot: Robot,
  arm: Reach,
  hitch: Hitch,
  q: unknown,
  color: string,
  handles = false,
) {
  if (robot === 'reach') {
    drawArm(ctx, v, p, arm, q as Tn, -1, handles, color);
  } else if (robot === 'rusty') {
    const d = q as R2;
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(sx(v, d.x), sy(v, d.y), Math.max(sl(v, RUSTY.bodyRadius), 4), 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  } else {
    const c = q as HitchCfg;
    drawCar(ctx, v, p, hitch, c, -1, color);
    if (handles) strokePolygons(ctx, v, hitch.footprint(c), color, 1.5);
  }
}

function drawWitness(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, w: Witness) {
  ctx.save();
  for (const [pt, name] of [
    [w.onRobot, 'a*'],
    [w.onObstacle, 'b*'],
  ] as const) {
    ctx.fillStyle = p.graph;
    ctx.beginPath();
    ctx.arc(sx(v, pt.x), sy(v, pt.y), 4.5, 0, Math.PI * 2);
    ctx.fill();
    label(ctx, name, sx(v, pt.x) + 7, sy(v, pt.y) - 9, p.graph, { size: 11, weight: 700 });
  }
  ctx.restore();
}
