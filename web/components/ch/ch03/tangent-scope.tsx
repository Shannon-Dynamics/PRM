'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Point2 } from '@/lib/sim/world';
import {
  FIG_2_8,
  TWO_RECTANGLES,
  TangentBug,
  containingPolygon,
  dist,
  pointT,
  scanPoint,
  type BugPreset,
  type TangentSensing,
} from '@/lib/bugs';
import { X, Y, drawCircle, drawEndpoint, drawMarks, drawPointRobot, drawPolygons, drawPolyline } from './draw-bugs';

/**
 * w3.3 — the Tangent Scope.
 *
 * What a range sensor actually hands a planner: ρ_R(x, ·) as 360 thin rays,
 * the intervals of continuity drawn thick on the boundary, their endpoints
 * O_i, the point T on the sensing circle, and the heuristic d(x, n) + d(n, goal)
 * for every candidate, with the winner pulsing. The robot then walks Tangent
 * Bug's motion-to-goal so the picture moves; pause it and drag x or the goal.
 *
 * The misconception it kills: "a range sensor gives you the map". It gives
 * you a handful of discontinuities, and that is enough — until it is not
 * (figure 2.8: the heuristic prefers an O_i whose line to the goal is blocked by
 * an obstacle the robot cannot yet see).
 */

const FRAME = { ox: 0, oy: 0 };
const STEP = 0.05;
const CLEARANCE = 0.1;

interface State {
  tb: TangentBug;
  sensing: TangentSensing;
}

export function TangentScope() {
  const [presetId, setPresetId] = useState<'two-rectangles' | 'fig-2-8'>('fig-2-8');
  const preset: BugPreset = presetId === 'fig-2-8' ? FIG_2_8 : TWO_RECTANGLES;
  const [range, setRange] = useState(3);
  const [x, setX] = useState<Point2>(preset.start);
  const [goal, setGoal] = useState<Point2>(preset.goal);
  const dragging = useRef<'x' | 'goal' | null>(null);

  const choose = (id: 'two-rectangles' | 'fig-2-8') => {
    const pr = id === 'fig-2-8' ? FIG_2_8 : TWO_RECTANGLES;
    setPresetId(id);
    setX(pr.start);
    setGoal(pr.goal);
    setRange(pr.range);
  };

  const init = useCallback((): State => {
    const tb = new TangentBug(preset.world, x, goal, { step: STEP, turn: 'left', range, clearance: CLEARANCE, nRays: 360 });
    return { tb, sensing: tb.evaluate(x) };
  }, [preset, x, goal, range]);

  const step = useCallback((s: State): State => {
    const st = s.tb.state;
    if (st.done || st.failed || st.steps > 6000) return s;
    for (let i = 0; i < 2; i++) s.tb.step();
    return { tb: s.tb, sensing: s.tb.sensing ?? s.sensing };
  }, []);

  const sim = useSimulation<State>({ init, step, fps: 20, initialSeed: 1 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [preset, x, goal, range]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const { tb, sensing } = sim.state;
      const pos = tb.state.x;
      const pulse = 0.5 + 0.5 * Math.sin(sim.tick / 3);
      drawPolygons(ctx, v, FRAME, preset.world.obstacles, p);

      // ρ_R: thin rays; ∞ rays stop at the sensing circle.
      const sc = sensing.scan;
      ctx.save();
      ctx.strokeStyle = p.graph;
      ctx.globalAlpha = 0.16;
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let k = 0; k < sc.ranges.length; k++) {
        const r = Number.isFinite(sc.ranges[k]) ? sc.ranges[k] : Math.min(range, 60);
        ctx.moveTo(X(v, FRAME, sc.x.x), Y(v, FRAME, sc.x.y));
        ctx.lineTo(X(v, FRAME, sc.x.x + r * Math.cos(sc.angles[k])), Y(v, FRAME, sc.x.y + r * Math.sin(sc.angles[k])));
      }
      ctx.stroke();
      ctx.restore();
      drawCircle(ctx, v, FRAME, pos, Math.min(range, 60), p.graph, { alpha: 0.5 });

      // Intervals of continuity: thick on the boundary.
      for (const iv of sensing.intervals) {
        const pts: Point2[] = [];
        for (let i = 0; i < iv.count; i++) pts.push(scanPoint(sc, (iv.from + i) % sc.ranges.length));
        drawPolyline(ctx, v, FRAME, pts, p.graph, { width: 4, alpha: 0.9 });
      }

      // The segment to the goal, and whether it is blocked.
      drawPolyline(ctx, v, FRAME, [pos, goal], sensing.blocked ? p.obstacle : p.path, { dashed: true, width: 1.2, alpha: 0.8 });
      const T = pointT(pos, goal, range);
      if (dist(T, goal) > 1e-9) {
        ctx.save();
        ctx.fillStyle = p.path;
        ctx.beginPath();
        ctx.arc(X(v, FRAME, T.x), Y(v, FRAME, T.y), 3.5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.restore();
        label(ctx, 'T', X(v, FRAME, T.x) + 6, Y(v, FRAME, T.y) - 8, p.path, { size: 11, weight: 600 });
      }

      // The O_i, labelled, and the chosen one pulsing.
      drawMarks(ctx, v, FRAME, sensing.endpoints.map((e) => e.point), 'O', p.graph);
      if (sensing.chosen) {
        ctx.save();
        ctx.strokeStyle = p.path;
        ctx.lineWidth = 1.8;
        ctx.beginPath();
        ctx.arc(X(v, FRAME, sensing.chosen.point.x), Y(v, FRAME, sensing.chosen.point.y), 6 + 4 * pulse, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.restore();
      }

      drawPolyline(ctx, v, FRAME, tb.state.path, p.robot, { width: 1.8 });
      drawEndpoint(ctx, v, FRAME, goal, 'goal', p);
      drawPointRobot(ctx, v, FRAME, pos, p, pulse);
      label(ctx, 'x', X(v, FRAME, pos.x) + 8, Y(v, FRAME, pos.y) + 10, p.robot, { size: 11, weight: 600 });

      const st = tb.state;
      label(
        ctx,
        `R = ${range.toFixed(1)}   ${sensing.blocked ? 'goal blocked' : 'goal visible'}   ${st.done ? 'reached' : st.failed ? st.reason : st.reason}`,
        8,
        14,
        p.ink,
        { size: 10.5 },
      );
    },
    [sim.state, sim.tick, preset, goal, range],
  );

  const onPointer = useCallback(
    (world: [number, number], phase: 'down' | 'move' | 'up') => {
      const q: Point2 = { x: world[0], y: world[1] };
      if (phase === 'down') {
        if (dist(q, goal) < 0.4) dragging.current = 'goal';
        else if (dist(q, sim.state.tb.state.x) < 0.5) dragging.current = 'x';
        else dragging.current = null;
        if (dragging.current) sim.pause();
        return;
      }
      if (!dragging.current) return;
      if (phase === 'up') {
        const b = preset.world.bounds;
        const inBounds = q.x > b.minX + 0.1 && q.x < b.maxX - 0.1 && q.y > b.minY + 0.1 && q.y < b.maxY - 0.1;
        if (inBounds && containingPolygon(preset.world, q) < 0) {
          if (dragging.current === 'goal') setGoal(q);
          else setX(q);
        }
        dragging.current = null;
      }
    },
    [goal, preset, sim],
  );

  const sensing = sim.state.sensing;

  return (
    <WidgetFrame
      id="w3.3"
      title="Tangent Scope"
      teaches="A range sensor does not give you the map. It gives you a few discontinuities — and that is enough to plan with, until it is not."
      colorKey={['goal', 'obstacle', 'graph', 'path', 'robot']}
      caption={
        <>
          The thin blue rays are ρ<sub>R</sub>(<em>x</em>, θ); where they stop short of the dashed circle
          they have hit an obstacle, and the thick blue arcs on the boundaries are the intervals of
          continuity. Their endpoints O<sub>i</sub> arise where one obstacle occludes another (the ray is
          tangent) or where the boundary leaves sensing range; T is the point of the sensing circle on
          the line to the goal. The table prints every candidate&rsquo;s{' '}
          <em>d</em>(<em>x</em>, <em>n</em>) + <em>d</em>(<em>n</em>, <em>q</em><sub>goal</sub>); the winner pulses and the robot
          heads for it. The default is Choset&rsquo;s figure 2.8: with <em>R</em> = 3 the top corner of
          WO<sub>1</sub> looks best and the robot steers for it, not knowing WO<sub>2</sub> stands in the way
          beyond it. Press <em>R</em> = 6: now WO<sub>2</sub> is in view, the segment from that corner to
          the goal is seen to cut through it, its cost becomes ∞, and the robot steers for the near corner
          of WO<sub>2</sub> instead — and arrives by a shorter path, 8.13 against 8.46. Drag <em>x</em> and the goal; shrink <em>R</em> toward zero and watch the
          O<sub>i</sub> collapse onto the hit point until Tangent Bug is Bug2 in all but name.
        </>
      }
    >
      <SimCanvas
        world={preset.world.bounds}
        draw={draw}
        deps={[sim.tick, sim.state, range, goal]}
        aspect={(preset.world.bounds.maxX - preset.world.bounds.minX) / (preset.world.bounds.maxY - preset.world.bounds.minY)}
        padding={0.3}
        onPointer={onPointer}
        cursor="crosshair"
        ariaLabel="A point robot among rectangular obstacles casting 360 thin range rays. The visible stretches of boundary are drawn thick, their endpoints labelled O1, O2 and so on, and the endpoint minimizing the heuristic distance to the goal is highlighted."
      />

      <div className="overflow-x-auto border-t border-fd-border">
        <table className="w-full min-w-[26rem] border-collapse text-left font-ui text-[0.78rem]">
          <thead>
            <tr className="border-b border-fd-border">
              <th className="px-3 py-1.5 font-medium">candidate n</th>
              <th className="px-3 py-1.5 text-right font-medium">d(x, n)</th>
              <th className="px-3 py-1.5 text-right font-medium">d(n, q_goal)</th>
              <th className="px-3 py-1.5 text-right font-medium">heuristic</th>
              <th className="px-3 py-1.5 font-medium">why</th>
            </tr>
          </thead>
          <tbody className="font-mono tabular-nums">
            {sensing.candidates.map((c) => {
              const chosen = sensing.chosen === c;
              const e = sensing.endpoints.find((ep) => ep.point === c.point);
              return (
                <tr key={c.label} className={`border-b border-fd-border last:border-b-0 ${chosen ? 'font-semibold' : ''}`}>
                  <td className="px-3 py-1.5 font-ui">
                    {c.label}
                    {chosen ? ' ← chosen' : ''}
                  </td>
                  <td className="px-3 py-1.5 text-right">{dist(sim.state.tb.state.x, c.point).toFixed(3)}</td>
                  <td className="px-3 py-1.5 text-right">{dist(c.point, goal).toFixed(3)}</td>
                  <td className="px-3 py-1.5 text-right">{Number.isFinite(c.h) ? c.h.toFixed(3) : '∞'}</td>
                  <td className="px-3 py-1.5 font-ui text-fd-muted-foreground">
                    {!Number.isFinite(c.h)
                      ? 'the segment n → goal is seen to cut through another obstacle'
                      : e
                        ? e.kind === 'occlusion'
                          ? 'tangent ray: one obstacle occludes another'
                          : 'boundary leaves sensing range'
                        : 'the sensing circle on the line to the goal'}
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>

      <ControlPanel columns={2}>
        <Slider
          label="Sensing range R"
          role="graph"
          value={range}
          min={0.3}
          max={8}
          step={0.1}
          unit="m"
          onChange={setRange}
          help="Headline: how much of the world the robot can see. Small R collapses the O_i onto the hit point."
        />
        <ButtonRow>
          <ActionButton onClick={() => choose('fig-2-8')} emphasis={presetId === 'fig-2-8'}>
            C fig. 2.8 — the blind spot
          </ActionButton>
          <ActionButton onClick={() => choose('two-rectangles')} emphasis={presetId === 'two-rectangles'}>
            Worked example
          </ActionButton>
          <ActionButton onClick={() => setRange(6)}>R = 6</ActionButton>
        </ButtonRow>
      </ControlPanel>

      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}
