'use client';

import { useCallback, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawGrid, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { polygonArea, type Polygon } from '@/lib/collide/polygon';
import { convexHull, edgeNormals, randomConvexPolygon, rotatePolygon, toCcw } from '@/lib/cspace/polygon';
import { STAR_PRESET, minkowskiBruteForce, se2Slices, starAlgorithm } from '@/lib/cspace/star';
import { arrowWorld, deg, dot, fillPoly, fmt, hatchFillPoly, strokePoly } from './shared';

/**
 * w4.2 — Minkowski Studio: the star algorithm, one merged normal at a time.
 *
 * A convex robot (orange, reference point at the origin) and a convex
 * obstacle (slate). The dial shows the obstacle's edge normals and the
 * robot's *negated* normals on one circle; the scan walks them in angular
 * order and each step emits one vertex o_j − r_i of QO(θ), labelled A (a
 * robot edge sliding along an obstacle vertex) or B (an obstacle edge sliding
 * along a robot vertex). A brute-force ghost — the hull of every pairwise
 * difference — is drawn dashed and the two coincide exactly.
 *
 * The misconception killed: "Minkowski sums are expensive". For convex
 * polygons the sum is a merge of two sorted lists: n_R + n_W vertices,
 * computed in n_R + n_W steps. Rotating the robot re-pairs the normals and
 * can change the vertex count; "stack" sweeps θ and draws the SE(2)
 * C-obstacle as Choset's stacked slices (figure F.8).
 */

const N_SLICES = 24;

export function MinkowskiStudio() {
  const [robot, setRobot] = useState<Polygon>(STAR_PRESET.robot);
  const [obstacle, setObstacle] = useState<Polygon>(STAR_PRESET.obstacle);
  const [theta, setTheta] = useState(0);
  const [stack, setStack] = useState(false);
  const [edit, setEdit] = useState(false);
  const drag = useRef<{ which: 'robot' | 'obstacle'; index: number } | null>(null);

  const result = useMemo(() => starAlgorithm(robot, theta, obstacle), [robot, theta, obstacle]);
  const ghost = useMemo(() => minkowskiBruteForce(robot, theta, obstacle), [robot, theta, obstacle]);
  const slices = useMemo(() => (stack ? se2Slices(robot, obstacle, N_SLICES) : []), [stack, robot, obstacle]);

  // The scan advances one merged normal per tick, pauses on the finished polygon, and loops.
  const nSteps = result.steps.length;
  const sim = useSimulation<{ k: number }>({
    init: () => ({ k: 0 }),
    step: (s) => ({ k: (s.k + 1) % (nSteps + 4) }),
    fps: 1.6,
    initialSeed: 42,
  });
  const k = Math.min(sim.state.k, nSteps);
  const complete = k >= nSteps;

  // A stable world box around everything drawn, rounded so it does not jitter while dragging.
  const world = useMemo(() => {
    const pts = [...obstacle, ...ghost, ...result.robotAtTheta, { x: 0, y: 0 }];
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const p of pts) {
      minX = Math.min(minX, p.x);
      maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y);
      maxY = Math.max(maxY, p.y);
    }
    const r = (x: number) => Math.round(x * 2) / 2;
    return { minX: r(minX - 0.4), minY: r(minY - 0.4), maxX: r(maxX + 0.4), maxY: r(maxY + (stack ? 2.2 : 0.4)) };
  }, [obstacle, ghost, result.robotAtTheta, stack]);

  // ---- vertex editing ---------------------------------------------------------
  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      if (!edit || stack) return;
      if (phase === 'up') {
        drag.current = null;
        return;
      }
      if (phase === 'down') {
        let best: { which: 'robot' | 'obstacle'; index: number; d: number } | null = null;
        result.robotAtTheta.forEach((p, i) => {
          const d = Math.hypot(p.x - w[0], p.y - w[1]);
          if (d < 0.18 && (!best || d < best.d)) best = { which: 'robot', index: i, d };
        });
        obstacle.forEach((p, i) => {
          const d = Math.hypot(p.x - w[0], p.y - w[1]);
          if (d < 0.18 && (!best || d < best.d)) best = { which: 'obstacle', index: i, d };
        });
        drag.current = best;
        return;
      }
      const d = drag.current;
      if (!d) return;
      if (d.which === 'obstacle') {
        const next = obstacle.map((p, i) => (i === d.index ? { x: w[0], y: w[1] } : p));
        setObstacle(toCcw(convexHull(next)).length >= 3 ? toCcw(convexHull(next)) : obstacle);
      } else {
        // The handle is on the rotated robot; store the vertex in the body frame.
        const c = Math.cos(-theta);
        const s = Math.sin(-theta);
        const body = { x: c * w[0] - s * w[1], y: s * w[0] + c * w[1] };
        const next = robot.map((p, i) => (i === d.index ? body : p));
        const hull = toCcw(convexHull(next));
        if (hull.length >= 3) setRobot(hull);
      }
    },
    [edit, stack, result.robotAtTheta, obstacle, robot, theta],
  );

  // ---- drawing ----------------------------------------------------------------------
  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawGrid(ctx, v, p, 0.5);

      if (stack) {
        drawStack(ctx, v, p, slices, obstacle, theta);
        return;
      }

      const R = result.robotAtTheta;
      fillPoly(ctx, v, obstacle, p.obstacle, 0.9);
      // Obstacle normals and vertex labels.
      const nW = edgeNormals(obstacle);
      obstacle.forEach((o, j) => {
        const q = obstacle[(j + 1) % obstacle.length];
        const m = { x: (o.x + q.x) / 2, y: (o.y + q.y) / 2 };
        arrowWorld(ctx, v, m.x, m.y, 0.22 * nW[j].x, 0.22 * nW[j].y, p.obstacle, 1.5);
        label(ctx, `o${j + 1}`, sx(v, o.x) + 6, sy(v, o.y) - 8, p.obstacle, { size: 10, weight: 600 });
      });

      // Ghost: the brute-force hull, always.
      strokePoly(ctx, v, ghost, p.cobstacle, 1.2, true, 0.8);

      // Emitted vertices so far, and the polygon when complete.
      const emitted = result.vertices.slice(0, Math.min(result.vertices.length, k + 1));
      if (complete) hatchFillPoly(ctx, v, result.polygon, p.cobstacle);
      else if (emitted.length > 1) {
        ctx.save();
        ctx.strokeStyle = p.cobstacle;
        ctx.lineWidth = 2;
        ctx.beginPath();
        emitted.forEach((e, i) => (i === 0 ? ctx.moveTo(sx(v, e.point.x), sy(v, e.point.y)) : ctx.lineTo(sx(v, e.point.x), sy(v, e.point.y))));
        ctx.stroke();
        ctx.restore();
      }
      emitted.forEach((e, i) => {
        dot(ctx, sx(v, e.point.x), sy(v, e.point.y), 4, p.cobstacle, p.bg);
        if (i > 0 || complete) label(ctx, `${e.kind}: o${e.obstacleVertex + 1}−r${e.robotVertex + 1}`, sx(v, e.point.x) + 6, sy(v, e.point.y) + 9, p.cobstacle, { size: 9, weight: 600 });
      });

      // The current step: the robot translated to the contact configuration, sliding edge in bold.
      const stepNow = !complete && k > 0 ? result.steps[k - 1] : null;
      if (stepNow) {
        const at = stepNow.to;
        const ghostR = R.map((r) => ({ x: r.x + at.x, y: r.y + at.y }));
        fillPoly(ctx, v, ghostR, p.robot, 0.18);
        strokePoly(ctx, v, ghostR, p.robot, 1, false, 0.6);
        if (stepNow.kind === 'A') {
          const a = ghostR[stepNow.edge];
          const b = ghostR[(stepNow.edge + 1) % R.length];
          strokePoly(ctx, v, [a, b], p.robot, 4);
          const o = obstacle[stepNow.vertex];
          dot(ctx, sx(v, o.x), sy(v, o.y), 6, p.obstacle, p.bg);
        } else {
          const a = obstacle[stepNow.edge];
          const b = obstacle[(stepNow.edge + 1) % obstacle.length];
          strokePoly(ctx, v, [a, b], p.obstacle, 4);
          const r = ghostR[stepNow.vertex];
          dot(ctx, sx(v, r.x), sy(v, r.y), 6, p.robot, p.bg);
        }
      }

      // The robot at the origin with its negated normals.
      fillPoly(ctx, v, R, p.robot, 0.85);
      const nR = edgeNormals(R);
      R.forEach((r, i) => {
        const q = R[(i + 1) % R.length];
        const m = { x: (r.x + q.x) / 2, y: (r.y + q.y) / 2 };
        arrowWorld(ctx, v, m.x, m.y, -0.2 * nR[i].x, -0.2 * nR[i].y, p.robot, 1.5);
        label(ctx, `r${i + 1}`, sx(v, r.x) + 6, sy(v, r.y) - 8, p.robot, { size: 10, weight: 600 });
      });
      dot(ctx, sx(v, 0), sy(v, 0), 3.5, p.bg, p.robot);
      if (edit) {
        for (const h of [...R, ...obstacle]) dot(ctx, sx(v, h.x), sy(v, h.y), 7, 'rgba(0,0,0,0)', p.ink);
      }

      drawDial(ctx, v, p, result, k);
      const title = complete
        ? `QO(θ): ${result.polygon.length} vertices, area ${fmt(polygonArea(result.polygon), 3)}  (ghost: ${ghost.length}, ${fmt(polygonArea(ghost), 3)})`
        : k === 0
          ? `start: o${result.vertices[0].obstacleVertex + 1} − r${result.vertices[0].robotVertex + 1}, the +x-extreme pair`
          : describe(result, k - 1);
      label(ctx, title, 10, 14, p.ink, { size: 11, weight: 600 });
    },
    [stack, slices, obstacle, theta, result, ghost, k, complete, edit],
  );

  const reroll = (seed: number) => {
    const rng = new Rng(seed);
    setRobot(randomConvexPolygon(rng, 3 + Math.floor(rng.uniform(0, 4)), rng.uniform(0.25, 0.5)));
    setObstacle(randomConvexPolygon(rng, 3 + Math.floor(rng.uniform(0, 5)), rng.uniform(0.5, 0.9), { x: 1.5, y: 1.4 }));
    sim.reseed(seed);
  };

  return (
    <WidgetFrame
      id="w4.2"
      title="Minkowski Studio"
      teaches="For convex polygons the Minkowski difference is a merge of two sorted lists of edge normals — one vertex per step, n_R + n_W steps."
      colorKey={['robot', 'obstacle', 'cobstacle']}
      caption={
        <>
          The orange robot sits with its reference point at the origin; the slate obstacle is where it
          is. On the dial, slate ticks are the obstacle's edge normals and orange ticks the robot's
          <em> negated</em> normals; the pointer scans them in angular order. Each step slides one edge
          along one vertex — a robot edge along an obstacle vertex (Type A) or an obstacle edge along a
          robot vertex (Type B) — and emits the vertex <em>o<sub>j</sub> − r<sub>i</sub></em> where the
          slide ends; the faint orange copy is the robot at that contact. The dashed outline is the
          brute-force hull of every pairwise difference, and the finished amber polygon lands on it
          exactly. Turn θ and the pairing changes; with the triangle preset the vertex count goes from
          five to seven. "Stack" sweeps θ and piles the slices into the SE(2) C-obstacle.
        </>
      }
    >
      <SimCanvas world={world} draw={draw} deps={[sim.tick, result, ghost, stack, slices, edit, world]} aspect={stack ? 1.1 : 1.35} padding={0.1} onPointer={onPointer} cursor={edit && !stack ? 'grab' : 'default'} ariaLabel="A convex orange robot, a slate obstacle, and the amber C-obstacle being built vertex by vertex, with a dial of edge normals" />

      <ControlPanel columns={3} title="Headline: the robot's orientation θ">
        <Slider label="θ" role="robot" value={theta} min={-Math.PI} max={Math.PI} step={0.01} unit="rad" format={(x) => `${fmt(x, 2)} (${fmt(deg(x), 0)}°)`} onChange={setTheta} />
        <ButtonRow>
          <ActionButton
            onClick={() => {
              setRobot(STAR_PRESET.robot);
              setObstacle(STAR_PRESET.obstacle);
              setTheta(0);
              sim.reset();
            }}
          >
            triangle vs. unit square
          </ActionButton>
          <ActionButton onClick={() => reroll(Math.floor(Math.random() * 100000))}>random convex pair</ActionButton>
        </ButtonRow>
        <div className="flex flex-col gap-2">
          <Toggle label="stack θ-slices: QO ⊂ SE(2)" role="cobstacle" checked={stack} onChange={setStack} />
          <Toggle label="drag vertices (re-hulled to stay convex)" checked={edit} onChange={setEdit} />
        </div>
      </ControlPanel>

      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} tick={sim.tick} />
    </WidgetFrame>
  );
}

function describe(result: ReturnType<typeof starAlgorithm>, i: number): string {
  const s = result.steps[i];
  const name =
    s.kind === 'A'
      ? `Type A: robot edge E^R_${s.edge + 1} slides along obstacle vertex o${s.vertex + 1}`
      : `Type B: obstacle edge E^W_${s.edge + 1} slides along robot vertex r${s.vertex + 1}`;
  return `step ${i + 1}/${result.steps.length} — ${name} → (${fmt(s.to.x, 2)}, ${fmt(s.to.y, 2)})`;
}

/** The angle dial in the top-right corner: merged normals as ticks, the scan pointer on the current one. */
function drawDial(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, result: ReturnType<typeof starAlgorithm>, k: number) {
  const r = Math.min(46, v.width * 0.09);
  const cx = v.width - r - 14;
  const cy = r + 26;
  ctx.save();
  ctx.fillStyle = p.bg;
  ctx.globalAlpha = 0.85;
  ctx.beginPath();
  ctx.arc(cx, cy, r + 10, 0, Math.PI * 2);
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.strokeStyle = p.grid;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.stroke();
  result.merged.forEach((m, idx) => {
    const a = m.angle;
    const col = m.source === 'robot' ? p.robot : p.obstacle;
    const done = idx < k;
    ctx.strokeStyle = col;
    ctx.globalAlpha = done ? 1 : 0.45;
    ctx.lineWidth = idx === k - 1 ? 3 : 1.5;
    ctx.beginPath();
    ctx.moveTo(cx + (r - 8) * Math.cos(a), cy - (r - 8) * Math.sin(a));
    ctx.lineTo(cx + (r + 2) * Math.cos(a), cy - (r + 2) * Math.sin(a));
    ctx.stroke();
  });
  ctx.globalAlpha = 1;
  if (k > 0 && k <= result.merged.length) {
    const a = result.merged[k - 1].angle;
    ctx.strokeStyle = p.cobstacle;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + (r - 10) * Math.cos(a), cy - (r - 10) * Math.sin(a));
    ctx.stroke();
  }
  ctx.restore();
  label(ctx, 'normals: slate = obstacle, orange = −robot', cx, cy + r + 16, p.ink, { size: 8.5, align: 'center' });
}

/** Choset's figure F.8: QO(θ_k) slices stacked along a vertical θ axis in an oblique projection. */
function drawStack(ctx: CanvasRenderingContext2D, v: Viewport, p: Palette, slices: { theta: number; polygon: Polygon }[], obstacle: Polygon, thetaNow: number) {
  const H = 2.0; // world units of height for the full turn
  const shear = 0.25;
  const proj = (pt: { x: number; y: number }, z: number) => ({ x: pt.x + shear * z, y: pt.y + z });
  // Axis.
  const base = obstacle[0];
  ctx.save();
  ctx.strokeStyle = p.ink;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(sx(v, base.x - 0.9), sy(v, base.y - 0.3));
  ctx.lineTo(sx(v, base.x - 0.9 + shear * H), sy(v, base.y - 0.3 + H));
  ctx.stroke();
  ctx.restore();
  label(ctx, 'θ', sx(v, base.x - 0.9 + shear * H) - 10, sy(v, base.y - 0.3 + H) - 8, p.ink, { size: 11, weight: 700 });
  label(ctx, '−π', sx(v, base.x - 0.9) - 14, sy(v, base.y - 0.3), p.ink, { size: 9 });
  label(ctx, 'π', sx(v, base.x - 0.9 + shear * H) - 12, sy(v, base.y - 0.3 + H), p.ink, { size: 9 });
  fillPoly(ctx, v, obstacle, p.obstacle, 0.9);
  const kNow = Math.round(((thetaNow + Math.PI) / (2 * Math.PI)) * slices.length) % slices.length;
  slices.forEach((s, i) => {
    const z = ((s.theta + Math.PI) / (2 * Math.PI)) * H;
    const poly = s.polygon.map((pt) => proj(pt, z));
    const current = i === kNow;
    fillPoly(ctx, v, poly, p.cobstacle, current ? 0.45 : 0.1);
    strokePoly(ctx, v, poly, p.cobstacle, current ? 2.2 : 0.8, false, current ? 1 : 0.7);
  });
  label(ctx, `${slices.length} slices of QO(θ); bold = θ ≈ ${fmt(deg(thetaNow), 0)}°  — each slice is exact, the stack is a picture`, 10, 14, p.ink, { size: 11, weight: 600 });
  void sl;
}
