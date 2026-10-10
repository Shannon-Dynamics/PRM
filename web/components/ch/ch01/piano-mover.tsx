'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, drawSegments, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { normalizeAngle, type Pose2 } from '@/lib/geom/se2';
import { Rng } from '@/lib/prob/rng';
import { orientedRect } from '@/lib/collide/polygon';
import { Scene } from '@/lib/collide/scene';
import type { Segment } from '@/lib/sim/world';

/**
 * w1.2 — the Piano Mover (a sofa, in the plane).
 *
 * Left: a room with a doorway and a sofa the reader drags and rotates. Right:
 * the sofa's configuration (x, y, θ) as a dot in the θ-slice of SE(2), among
 * amber C-obstacles that redraw as θ changes. The sofa jams exactly when the
 * dot touches amber. The misconception killed: "the obstacle is the wall". The
 * obstacle the planner fights is the amber shape, and it depends on the robot.
 *
 * The slice is computed honestly, by asking the Chapter 2 polygon checker
 * whether R(x, y, θ) touches a wall at every cell of a grid over the room.
 * Chapter 4 replaces this brute force with the star algorithm; the picture
 * does not change, only the cost of drawing it.
 */

const ROOM = { minX: 0, minY: 0, maxX: 8, maxY: 4.8 };
const DIVIDER_X = 4;
const DOOR_Y = 2.4;
const SOFA_W = 0.6;
const CELL = 0.1;
const NX = Math.round((ROOM.maxX - ROOM.minX) / CELL);
const NY = Math.round((ROOM.maxY - ROOM.minY) / CELL);
const STACK = 8;

interface State {
  pose: Pose2;
  phase: [number, number, number];
  t: number;
  manual: boolean;
}

type Drag = 'body' | 'rotate' | 'dot' | null;

function roomWalls(doorW: number): Segment[] {
  const { minX, minY, maxX, maxY } = ROOM;
  return [
    { x1: minX, y1: minY, x2: maxX, y2: minY },
    { x1: maxX, y1: minY, x2: maxX, y2: maxY },
    { x1: maxX, y1: maxY, x2: minX, y2: maxY },
    { x1: minX, y1: maxY, x2: minX, y2: minY },
    { x1: DIVIDER_X, y1: minY, x2: DIVIDER_X, y2: DOOR_Y - doorW / 2 },
    { x1: DIVIDER_X, y1: DOOR_Y + doorW / 2, x2: DIVIDER_X, y2: maxY },
  ];
}

/** The θ-slice of the C-obstacle: one byte per cell, 1 where R(x, y, θ) touches a wall. */
function computeSlice(scene: Scene, halfL: number, theta: number): Uint8Array {
  const out = new Uint8Array(NX * NY);
  for (let j = 0; j < NY; j++) {
    const y = ROOM.minY + (j + 0.5) * CELL;
    for (let i = 0; i < NX; i++) {
      const x = ROOM.minX + (i + 0.5) * CELL;
      if (scene.intersectsPolygon(orientedRect(x, y, halfL, SOFA_W / 2, theta))) out[j * NX + i] = 1;
    }
  }
  return out;
}

export function PianoMover() {
  const [length, setLength] = useState(1.6);
  const [doorW, setDoorW] = useState(1.0);
  const [stacked, setStacked] = useState(false);
  const [drag, setDrag] = useState<Drag>(null);
  const grab = useRef<{ dx: number; dy: number }>({ dx: 0, dy: 0 });

  const walls = useMemo(() => roomWalls(doorW), [doorW]);
  const scene = useMemo(() => Scene.ofWalls(walls, ROOM), [walls]);
  const cache = useRef(new Map<string, Uint8Array>());
  // A new sofa or doorway invalidates every slice.
  const sliceKeyBase = `${length.toFixed(2)}|${doorW.toFixed(2)}`;
  const sliceFor = useCallback(
    (theta: number) => {
      const q = Math.round(theta / 0.02) * 0.02; // 0.02 rad quantization keeps the cache small
      const key = `${sliceKeyBase}|${q.toFixed(2)}`;
      let s = cache.current.get(key);
      if (!s) {
        if (cache.current.size > 400) cache.current.clear();
        s = computeSlice(scene, length / 2, q);
        cache.current.set(key, s);
      }
      return s;
    },
    [scene, length, sliceKeyBase],
  );

  // Autoplay: a seeded wiggle that carries the sofa back and forth through the
  // doorway while turning, so the reader sees the dot move and the amber breathe.
  const init = useCallback((seed: number): State => {
    const rng = new Rng(seed);
    return {
      pose: { x: 2.2, y: 2.4, theta: 0 },
      phase: [rng.uniform(0, 2 * Math.PI), rng.uniform(0, 2 * Math.PI), rng.uniform(0, 2 * Math.PI)],
      t: 0,
      manual: false,
    };
  }, []);
  const step = useCallback((s: State): State => {
    if (s.manual) return s;
    const t = s.t + 1 / 30;
    const [a, b, c] = s.phase;
    return {
      ...s,
      t,
      pose: {
        x: 4 + 1.9 * Math.sin(0.45 * t + a),
        y: 2.4 + 0.5 * Math.sin(0.31 * t + b),
        theta: normalizeAngle(0.55 * Math.sin(0.7 * t + c)),
      },
    };
  }, []);
  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 3 });
  const pose = sim.state.pose;
  const halfL = length / 2;
  const footprint = useMemo(() => orientedRect(pose.x, pose.y, halfL, SOFA_W / 2, pose.theta), [pose, halfL]);
  const colliding = useMemo(() => scene.intersectsPolygon(footprint), [scene, footprint]);
  const slice = useMemo(() => sliceFor(pose.theta), [sliceFor, pose.theta]);

  const handle = (p: Pose2) => ({ x: p.x + (halfL + 0.35) * Math.cos(p.theta), y: p.y + (halfL + 0.35) * Math.sin(p.theta) });

  // ---- left pane: drag the sofa --------------------------------------------------------
  const onRoomPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') return setDrag(null);
      if (phase === 'down') {
        const h = handle(pose);
        if (Math.hypot(w[0] - h.x, w[1] - h.y) < 0.3) setDrag('rotate');
        else if (Math.hypot(w[0] - pose.x, w[1] - pose.y) < halfL + 0.3) {
          grab.current = { dx: pose.x - w[0], dy: pose.y - w[1] };
          setDrag('body');
        } else return;
        sim.pause();
        sim.setState((s) => ({ ...s, manual: true }));
        return;
      }
      if (!drag) return;
      sim.setState((s) => {
        if (drag === 'rotate') {
          return { ...s, manual: true, pose: { ...s.pose, theta: Math.atan2(w[1] - s.pose.y, w[0] - s.pose.x) } };
        }
        return { ...s, manual: true, pose: { ...s.pose, x: w[0] + grab.current.dx, y: w[1] + grab.current.dy } };
      });
    },
    [drag, pose, halfL, sim],
  );

  // ---- right pane: drag the dot --------------------------------------------------------
  const onSlicePointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (phase === 'up') return setDrag(null);
      if (phase === 'down') {
        setDrag('dot');
        sim.pause();
      } else if (drag !== 'dot') return;
      sim.setState((s) => ({ ...s, manual: true, pose: { ...s.pose, x: w[0], y: w[1] } }));
    },
    [drag, sim],
  );

  // ---- drawing -----------------------------------------------------------------------------
  const drawRoom = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      drawSegments(ctx, v, walls, p.obstacle, 4);
      // The sofa: orange when free, red the instant it touches a wall.
      ctx.save();
      ctx.fillStyle = colliding ? p.goal : p.robot;
      ctx.beginPath();
      footprint.forEach((pt, i) => (i === 0 ? ctx.moveTo(sx(v, pt.x), sy(v, pt.y)) : ctx.lineTo(sx(v, pt.x), sy(v, pt.y))));
      ctx.closePath();
      ctx.fill();
      // Rotation handle on the nose.
      const h = handle(pose);
      ctx.strokeStyle = p.robot;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(sx(v, pose.x + halfL * Math.cos(pose.theta)), sy(v, pose.y + halfL * Math.sin(pose.theta)));
      ctx.lineTo(sx(v, h.x), sy(v, h.y));
      ctx.stroke();
      ctx.fillStyle = p.bg;
      ctx.beginPath();
      ctx.arc(sx(v, h.x), sy(v, h.y), 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.restore();
      label(ctx, 'workspace W — drag the sofa, turn it by the handle', 12, 16, p.ink, { size: 11, weight: 600 });
      label(ctx, `q = (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.theta.toFixed(2)})   ${colliding ? 'in collision' : 'free'}`, 12, v.height - 14, colliding ? p.goal : p.ink, { size: 11, weight: 600 });
    },
    [walls, footprint, colliding, pose, halfL],
  );

  const drawSlice = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 1);
      const cw = sl(v, CELL) + 0.5;

      const paint = (sl8: Uint8Array, yOff: number, alpha: number, hatch: boolean) => {
        ctx.save();
        ctx.globalAlpha = alpha;
        ctx.fillStyle = p.cobstacle;
        for (let j = 0; j < NY; j++) {
          for (let i = 0; i < NX; i++) {
            if (!sl8[j * NX + i]) continue;
            const x = ROOM.minX + i * CELL;
            const y = ROOM.minY + j * CELL + yOff;
            ctx.fillRect(sx(v, x), sy(v, y + CELL), cw, cw);
          }
        }
        if (hatch) {
          // Amber is hatched by convention: a C-obstacle is not a thing in the room.
          ctx.strokeStyle = p.cobstacle;
          ctx.lineWidth = 1;
          ctx.beginPath();
          for (let j = 0; j < NY; j++) {
            for (let i = 0; i < NX; i++) {
              if (!sl8[j * NX + i] || (i + j) % 3 !== 0) continue;
              const x = ROOM.minX + i * CELL;
              const y = ROOM.minY + j * CELL + yOff;
              ctx.moveTo(sx(v, x), sy(v, y));
              ctx.lineTo(sx(v, x + CELL), sy(v, y + CELL));
            }
          }
          ctx.stroke();
        }
        ctx.restore();
      };

      if (stacked) {
        // Slices at θₖ = kπ/8 stacked with a vertical offset — Choset's figure F.8 in translucent layers.
        for (let k = STACK - 1; k >= 0; k--) {
          const th = (k * Math.PI) / STACK;
          paint(sliceFor(th), k * 0.22, 0.18, false);
        }
      }
      paint(slice, 0, stacked ? 0.75 : 0.55, true);

      // The room's outline, for orientation: the slice shares the room's (x, y) axes.
      ctx.save();
      ctx.strokeStyle = p.grid;
      ctx.lineWidth = 1;
      ctx.strokeRect(sx(v, ROOM.minX), sy(v, ROOM.maxY), sl(v, ROOM.maxX - ROOM.minX), sl(v, ROOM.maxY - ROOM.minY));
      ctx.restore();

      // The configuration: one point.
      ctx.save();
      ctx.fillStyle = colliding ? p.goal : p.robot;
      ctx.beginPath();
      ctx.arc(sx(v, pose.x), sy(v, pose.y), 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      label(ctx, `θ-slice of SE(2) at θ = ${pose.theta.toFixed(2)} rad — the sofa is this dot`, 12, 16, p.ink, { size: 11, weight: 600 });
    },
    [slice, stacked, sliceFor, pose, colliding],
  );

  return (
    <WidgetFrame
      id="w1.2"
      title="Piano Mover"
      teaches="The obstacle the planner fights is not the wall. It is the amber shape in configuration space, and it depends on the robot."
      colorKey={['obstacle', 'cobstacle', 'robot', 'goal']}
      wide
      caption={
        <>
          Left, the room: drag the sofa, turn it by the handle on its nose, and try to get it through
          the doorway. Right, the same sofa as a single point in the θ-slice of its configuration
          space, with every (x, y) at which the sofa <em>at this heading</em> would touch a wall
          painted amber. The sofa jams exactly when the dot touches amber — not before, not after.
          Turn the sofa and watch the amber reshape: the doorway is a channel that exists at some
          headings and closes at others, your first narrow passage. Shorten the sofa until it fits
          nose-first; then stack the slices and see the channel twist through the third dimension.
        </>
      }
    >
      <div className="grid grid-cols-1 gap-px bg-fd-border md:grid-cols-[11fr_9fr]">
        <div className="bg-fd-card">
          <SimCanvas world={ROOM} draw={drawRoom} deps={[pose, footprint, colliding, walls]} aspect={8 / 4.8} padding={0.2} onPointer={onRoomPointer} cursor={drag ? 'grabbing' : 'grab'} ariaLabel="A room divided by a wall with a doorway; an orange sofa rectangle that can be dragged and rotated." />
        </div>
        <div className="bg-fd-card">
          <SimCanvas world={{ ...ROOM, maxY: stacked ? ROOM.maxY + 0.22 * (STACK - 1) : ROOM.maxY }} draw={drawSlice} deps={[slice, stacked, pose, colliding]} aspect={8 / 4.8} padding={0.2} onPointer={onSlicePointer} cursor="crosshair" ariaLabel="The configuration-space slice: amber hatched regions mark sofa positions that collide at the current heading; an orange dot marks the current configuration." />
        </div>
      </div>

      <ControlPanel columns={3}>
        <Slider label="Sofa length (headline)" role="robot" value={length} min={0.7} max={2.6} step={0.05} unit="m" onChange={setLength} help="The robot changes; the walls do not; the amber does." />
        <Slider label="Doorway width" role="obstacle" value={doorW} min={0.7} max={2.0} step={0.05} unit="m" onChange={setDoorW} />
        <Toggle label="Stack all θ-slices (C figure F.8)" role="cobstacle" checked={stacked} onChange={setStacked} />
      </ControlPanel>

      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onReset={() => {
          sim.reset();
        }}
        onReseed={() => sim.reseed()}
        seed={sim.seed}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}
