'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Toggle, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, type Palette, type Viewport } from '@/lib/sim/draw';
import type { Point2 } from '@/lib/sim/world';
import {
  Bug1,
  Bug2,
  TangentBug,
  PRESETS,
  bug1Bound,
  bug2Bound,
  containingPolygon,
  dist,
  lineCrossings,
  polygonContact,
  polygonPerimeter,
  presetById,
  randomPreset,
  type BugPlanner,
  type BugPreset,
  type PresetId,
  type Turn,
} from '@/lib/bugs';
import { X, Y, drawCircle, drawEndpoint, drawMarks, drawPointRobot, drawPolygons, drawPolyline } from './draw-bugs';

/**
 * w3.1 — the Bug Arena.
 *
 * Three lanes, one map, one query: Bug1, Bug2 and Tangent Bug run in lockstep
 * on the same obstacles toward the same goal, each with its hit and leave
 * points labelled as in Choset's figures 2.1–2.4 and a live length counter set
 * against its printed bound. It opens on the chapter's worked example — Bug2
 * wins — and, left alone, flips to the spiral of figure 2.4, where the greedy
 * leave rule walks the same boundary again and again and Bug2 loses badly.
 *
 * The misconception it kills: "greedy is faster". Greedy is faster on
 * average and unboundedly slower in the worst case; the counter shows both.
 */

const GAP = 0.9;
const STEP = 0.05;
const CLEARANCE = 0.1;

interface Lane {
  key: 'bug1' | 'bug2' | 'tangent';
  name: string;
  planner: BugPlanner;
  bound: number | null;
}

interface State {
  preset: BugPreset;
  lanes: Lane[];
  /** Ticks since every lane finished; drives the autoplay tour. */
  idle: number;
}

export function BugArena() {
  const [presetId, setPresetId] = useState<PresetId>('two-rectangles');
  const [turn, setTurn] = useState<Turn>('right');
  const [range, setRange] = useState(2.5);
  const [goalOverride, setGoalOverride] = useState<Point2 | null>(null);
  const [touched, setTouched] = useState(false);
  const dragging = useRef(false);

  const init = useCallback(
    (seed: number): State => {
      const base = presetId === 'random' ? randomPreset(seed) : presetById(presetId);
      const preset: BugPreset = goalOverride ? { ...base, goal: goalOverride } : base;
      const contact = polygonContact(preset.world);
      const perims = preset.world.obstacles.map(polygonPerimeter);
      const lanes: Lane[] = [
        {
          key: 'bug1',
          name: 'Bug1 (C Alg. 1)',
          planner: new Bug1(contact, preset.start, preset.goal, { step: STEP, turn }),
          bound: bug1Bound(preset.start, preset.goal, perims),
        },
        {
          key: 'bug2',
          name: 'Bug2 (C Alg. 2)',
          planner: new Bug2(contact, preset.start, preset.goal, { step: STEP, turn }),
          bound: bug2Bound(preset.start, preset.goal, preset.world.obstacles, perims),
        },
        {
          key: 'tangent',
          name: 'Tangent Bug (C Alg. 3)',
          planner: new TangentBug(preset.world, preset.start, preset.goal, {
            step: STEP,
            turn,
            range,
            clearance: CLEARANCE,
            nRays: 180,
          }),
          bound: null,
        },
      ];
      return { preset, lanes, idle: 0 };
    },
    [presetId, turn, range, goalOverride],
  );

  const step = useCallback((s: State): State => {
    let busy = false;
    for (const lane of s.lanes) {
      const st = lane.planner.state;
      if (st.done || st.failed) continue;
      busy = true;
      // Several motion steps per frame: 0.05 m × 4 × 30 fps = 6 m/s on screen.
      for (let i = 0; i < 4; i++) {
        lane.planner.step();
        if (lane.planner.state.steps > 60_000) {
          lane.planner.state.failed = true;
          lane.planner.state.mode = 'failed';
          lane.planner.state.reason = 'step budget exhausted';
          break;
        }
      }
    }
    return { ...s, idle: busy ? 0 : s.idle + 1 };
  }, []);

  const sim = useSimulation<State>({ init, step, fps: 30, initialSeed: 7 });

  // Parameter changes restart the race.
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [presetId, turn, range, goalOverride]);

  // The hook's second act: once the worked example has played out untouched,
  // flip to the spiral where Bug2 loses.
  useEffect(() => {
    if (!touched && presetId === 'two-rectangles' && sim.state.idle === 60) {
      setPresetId('spiral');
    }
  }, [sim.state.idle, presetId, touched]);

  const choosePreset = (id: PresetId) => {
    setTouched(true);
    setGoalOverride(null);
    setPresetId(id);
  };

  const { preset } = sim.state;
  const b = preset.world.bounds;
  const W = b.maxX - b.minX;
  const H = b.maxY - b.minY;
  const arena = useMemo(() => ({ minX: 0, minY: 0, maxX: 3 * W + 2 * GAP, maxY: H }), [W, H]);
  const frames = useMemo(() => [0, 1, 2].map((i) => ({ ox: i * (W + GAP) - b.minX, oy: -b.minY })), [W, b.minX, b.minY]);

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const s = sim.state;
      const pulse = 0.5 + 0.5 * Math.sin(sim.tick / 4);
      s.lanes.forEach((lane, i) => {
        const f = frames[i];
        const st = lane.planner.state;
        ctx.save();
        ctx.strokeStyle = p.grid;
        ctx.lineWidth = 1;
        ctx.strokeRect(X(v, f, b.minX), Y(v, f, b.maxY), sl(v, W), sl(v, H));
        ctx.restore();

        drawPolygons(ctx, v, f, s.preset.world.obstacles, p);

        // The m-line: Bug2's is fixed; Bug1's is redrawn from each leave point.
        if (lane.key === 'bug2') drawPolyline(ctx, v, f, [s.preset.start, s.preset.goal], p.path, { dashed: true, width: 1.2, alpha: 0.8 });
        if (lane.key === 'bug1') {
          const from = st.leaves.length > 0 ? st.leaves[st.leaves.length - 1] : s.preset.start;
          drawPolyline(ctx, v, f, [from, s.preset.goal], p.path, { dashed: true, width: 1.2, alpha: 0.8 });
        }

        // Tangent Bug's sensor: range circle, rays, and the chosen O_i.
        if (lane.key === 'tangent') {
          const tb = lane.planner as TangentBug;
          const sensed = tb.sensing;
          if (Number.isFinite(range)) drawCircle(ctx, v, f, st.x, range, p.graph, { alpha: 0.35 });
          if (sensed) {
            ctx.save();
            ctx.strokeStyle = p.graph;
            ctx.globalAlpha = 0.14;
            ctx.lineWidth = 1;
            ctx.beginPath();
            const sc = sensed.scan;
            for (let k = 0; k < sc.ranges.length; k += 2) {
              if (!Number.isFinite(sc.ranges[k])) continue;
              ctx.moveTo(X(v, f, sc.x.x), Y(v, f, sc.x.y));
              ctx.lineTo(X(v, f, sc.x.x + sc.ranges[k] * Math.cos(sc.angles[k])), Y(v, f, sc.x.y + sc.ranges[k] * Math.sin(sc.angles[k])));
            }
            ctx.stroke();
            ctx.globalAlpha = 0.9;
            ctx.fillStyle = p.graph;
            for (const e of sensed.endpoints) {
              ctx.beginPath();
              ctx.arc(X(v, f, e.point.x), Y(v, f, e.point.y), 2.4, 0, 2 * Math.PI);
              ctx.fill();
            }
            if (sensed.chosen) {
              ctx.strokeStyle = p.path;
              ctx.lineWidth = 1.6;
              ctx.beginPath();
              ctx.arc(X(v, f, sensed.chosen.point.x), Y(v, f, sensed.chosen.point.y), 5 + 3 * pulse, 0, 2 * Math.PI);
              ctx.stroke();
            }
            ctx.restore();
          }
        }

        drawPolyline(ctx, v, f, st.path, p.robot, { width: 2 });
        drawMarks(ctx, v, f, st.hits, lane.key === 'tangent' ? 'M' : 'H', p.robot);
        drawMarks(ctx, v, f, st.leaves, 'L', p.robot, { filled: false, dy: 9 });
        drawEndpoint(ctx, v, f, s.preset.start, 'start', p);
        drawEndpoint(ctx, v, f, s.preset.goal, 'goal', p);
        drawPointRobot(ctx, v, f, st.x, p, st.done || st.failed ? 0 : pulse);

        label(ctx, lane.name, X(v, f, b.minX) + 4, Y(v, f, b.maxY) - 9, p.ink, { size: 11, weight: 700 });
        const status = st.done ? 'reached' : st.failed ? 'unreachable' : st.mode;
        label(ctx, `${st.length.toFixed(2)} · ${status}`, X(v, f, b.maxX) - 4, Y(v, f, b.maxY) - 9, st.failed ? p.goal : p.robot, {
          size: 11,
          weight: 600,
          align: 'right',
        });
      });
    },
    [sim.state, sim.tick, frames, b, W, H, range],
  );

  // Drag the goal in any lane. Committed on release so the race restarts once.
  const onPointer = useCallback(
    (world: [number, number], phase: 'down' | 'move' | 'up') => {
      const laneW = W + GAP;
      const i = Math.max(0, Math.min(2, Math.floor(world[0] / laneW)));
      const f = frames[i];
      const q: Point2 = { x: world[0] - f.ox, y: world[1] - f.oy };
      const s = sim.state;
      if (phase === 'down') {
        if (dist(q, s.preset.goal) < 0.08 * W) dragging.current = true;
        return;
      }
      if (!dragging.current) return;
      if (phase === 'up') {
        dragging.current = false;
        const inBounds = q.x > b.minX + 0.2 && q.x < b.maxX - 0.2 && q.y > b.minY + 0.2 && q.y < b.maxY - 0.2;
        if (inBounds && containingPolygon(s.preset.world, q) < 0) {
          setTouched(true);
          setGoalOverride(q);
        }
      }
    },
    [sim.state, frames, W, b],
  );

  const s = sim.state;
  const d = dist(s.preset.start, s.preset.goal);
  const crossings = s.preset.world.obstacles.map((o) => lineCrossings(o, s.preset.start, s.preset.goal));

  return (
    <WidgetFrame
      id="w3.1"
      title="Bug Arena"
      teaches="Greedy is faster on average and unboundedly slower in the worst case — the switching rule is the whole difference between a heuristic and a theorem."
      colorKey={['start', 'goal', 'obstacle', 'path', 'robot', 'graph']}
      wide
      caption={
        <>
          Same map, same query, three switching rules. The dashed purple line is the m-line each Bug
          steers along; orange is the path walked, with hit points H<sub>i</sub> (filled) and leave
          points L<sub>i</sub> (hollow); Tangent Bug shows its range circle, its rays, the
          discontinuities O<sub>i</sub> in blue and the subgoal it is heading for pulsing in purple.
          The table beneath sets each length against its bound. On the worked example Bug2 wins
          16 to 34. Left alone, the arena switches to the spiral of Choset&rsquo;s figure 2.4: the
          m-line crosses the boundary ten times, Bug2 turning right leaves at the far side of each
          arm and re-walks the whole boundary it has already seen, and Bug1 — which walked it once
          — comes out ahead. Flip the turn direction and Bug2 wins again, by a mile. Drag the goal
          anywhere; every lane restarts. Re-roll for a random field and watch the bounds hold
          every time.
        </>
      }
    >
      <SimCanvas
        world={arena}
        draw={draw}
        deps={[sim.tick, sim.state, range]}
        aspect={arena.maxX / arena.maxY}
        padding={0.3}
        onPointer={onPointer}
        cursor="crosshair"
        ariaLabel="Three copies of the same obstacle map side by side. In each, a point robot walks from the green start to the red goal: Bug1 circumnavigating every obstacle it hits, Bug2 leaving at the first closer m-line crossing, and Tangent Bug steering by the discontinuities of its range scan."
      />

      <div className="overflow-x-auto border-t border-fd-border">
        <table className="w-full min-w-[30rem] border-collapse text-left font-ui text-[0.78rem]">
          <thead>
            <tr className="border-b border-fd-border">
              <th className="px-3 py-1.5 font-medium">planner</th>
              <th className="px-3 py-1.5 text-right font-medium">length</th>
              <th className="px-3 py-1.5 text-right font-medium">bound</th>
              <th className="px-3 py-1.5 text-right font-medium">hits / leaves</th>
              <th className="px-3 py-1.5 font-medium">status</th>
            </tr>
          </thead>
          <tbody className="font-mono tabular-nums">
            {s.lanes.map((lane) => {
              const st = lane.planner.state;
              return (
                <tr key={lane.key} className="border-b border-fd-border last:border-b-0">
                  <td className="px-3 py-1.5 font-ui">{lane.name}</td>
                  <td className="px-3 py-1.5 text-right">{st.length.toFixed(2)}</td>
                  <td className="px-3 py-1.5 text-right">
                    {lane.bound === null
                      ? '—'
                      : lane.key === 'bug1'
                        ? `${lane.bound.toFixed(2)} = ${d.toFixed(2)} + 1.5·Σp`
                        : `${lane.bound.toFixed(2)} = ${d.toFixed(2)} + ½·Σnᵢpᵢ (n = ${crossings.join(',')})`}
                  </td>
                  <td className="px-3 py-1.5 text-right">
                    {st.hits.length} / {st.leaves.length}
                  </td>
                  <td className="px-3 py-1.5 font-ui">{st.done ? 'reached the goal' : st.failed ? st.reason : st.reason}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>

      <ControlPanel columns={2} title="Map">
        <ButtonRow>
          {PRESETS.map((pr) => (
            <ActionButton key={pr.id} onClick={() => choosePreset(pr.id)} emphasis={presetId === pr.id}>
              {pr.label}
            </ActionButton>
          ))}
          <ActionButton onClick={() => choosePreset('random')} emphasis={presetId === 'random'}>
            Random field
          </ActionButton>
        </ButtonRow>
        <p className="font-ui text-[0.75rem] leading-snug text-fd-muted-foreground">{s.preset.note}</p>
      </ControlPanel>

      <ControlPanel columns={2}>
        <Slider
          label="Tangent Bug range R"
          role="graph"
          value={range}
          min={0.3}
          max={6}
          step={0.1}
          unit="m"
          onChange={(v) => {
            setTouched(true);
            setRange(v);
          }}
          help="R → 0 makes Tangent Bug behave like Bug2; large R straightens its path toward figure 2.13."
        />
        <Toggle
          label={`Turn ${turn} at a hit point`}
          role="robot"
          checked={turn === 'right'}
          onChange={(v) => {
            setTouched(true);
            setTurn(v ? 'right' : 'left');
          }}
        />
      </ControlPanel>

      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={() => {
          setTouched(true);
          setGoalOverride(null);
          setPresetId('random');
          sim.reseed();
        }}
        seed={sim.seed}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}
