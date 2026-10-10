'use client';

import { useCallback, useEffect, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, drawWorld, label, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import {
  APARTMENT_GOAL,
  APARTMENT_START,
  RrtConnect,
  Sbl,
  StraightLine,
  discLab,
  paintDot,
  shortcutGreedy,
  tint,
  type DiscLab,
  type Greed,
} from '@/lib/sampling';

/**
 * w12.2 — Bidirectional Connect.
 *
 * Four planners on one seed, in lockstep, across the Apartment's doorways:
 * Choset Algorithm 13 as written (Extend / Extend), Kuffner & LaValle's
 * RRT-Connect (Extend / Connect), the greediest setting (Connect / Connect),
 * and SBL — a bidirectional EST that checks an edge only when it lies on the
 * path that would join the trees. The table is the point: the same seed, the
 * same world, and the collision-check bill of each, plus the path before and
 * after Chapter 11's greedy shortcutting.
 */

type Mode = 'EE' | 'EC' | 'CC' | 'SBL';

const MODES: { key: Mode; label: string; greed?: [Greed, Greed] }[] = [
  { key: 'EE', label: 'Extend / Extend  (Alg. 13)', greed: ['extend', 'extend'] },
  { key: 'EC', label: 'Extend / Connect  (RRT-Connect)', greed: ['extend', 'connect'] },
  { key: 'CC', label: 'Connect / Connect', greed: ['connect', 'connect'] },
  { key: 'SBL', label: 'SBL  (lazy edges)' },
];

const ETA = 0.6;
const STEPS_PER_TICK = 3;

interface Lane {
  mode: Mode;
  lab: DiscLab;
  rc: RrtConnect<number[]> | null;
  sbl: Sbl<number[]> | null;
  path: number[][] | null;
  shortcut: number[][] | null;
  solvedAt: number | null;
  checksAtSolve: number | null;
}

interface State {
  lanes: Lane[];
}

const pathLength = (lab: DiscLab, path: readonly number[][]) => {
  let L = 0;
  for (let i = 1; i < path.length; i++) L += lab.space.dist(path[i - 1], path[i]);
  return L;
};

function makeLane(mode: Mode, seed: number): Lane {
  const lab = discLab();
  const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 });
  const m = MODES.find((x) => x.key === mode)!;
  if (mode === 'SBL') {
    return { mode, lab, rc: null, sbl: new Sbl(lab.space, lab.free, [...APARTMENT_START], [...APARTMENT_GOAL], { steer, radius: ETA * 1.5, cell: 0.6 }, new Rng(seed)), path: null, shortcut: null, solvedAt: null, checksAtSolve: null };
  }
  return { mode, lab, rc: new RrtConnect(lab.space, lab.free, [...APARTMENT_START], [...APARTMENT_GOAL], { steer, eta: ETA, greed: m.greed }, new Rng(seed)), sbl: null, path: null, shortcut: null, solvedAt: null, checksAtSolve: null };
}

export function BidirectionalConnect() {
  const [mode, setMode] = useState<Mode>('EC');
  const [attempts, setAttempts] = useState(4000);
  const attemptsRef = useRef(attempts);
  attemptsRef.current = attempts;

  const init = useCallback((seed: number): State => ({ lanes: MODES.map((m) => makeLane(m.key, seed)) }), []);

  const stepFn = useCallback((s: State): State => {
    for (const lane of s.lanes) {
      if (lane.path) continue;
      const its = lane.rc ? lane.rc.attempts : lane.sbl!.iterations;
      if (its >= attemptsRef.current) continue;
      for (let k = 0; k < STEPS_PER_TICK; k++) {
        if (lane.rc) lane.rc.step();
        else lane.sbl!.step();
        const path = lane.rc ? lane.rc.path() : lane.sbl!.path;
        if (path) {
          lane.path = path;
          lane.solvedAt = lane.rc ? lane.rc.mergedAt : lane.sbl!.solvedAt;
          lane.checksAtSolve = lane.lab.free.checks;
          // Shortcut against a fresh checker so the planner's bill stays the planner's.
          const fresh = discLab();
          lane.shortcut = shortcutGreedy(new StraightLine<number[]>({ kind: 'subdivision', step: 0.05 }), fresh.space, fresh.free, path);
          break;
        }
      }
    }
    return { ...s };
  }, []);

  const sim = useSimulation<State>({ init, step: stepFn, fps: 30, initialSeed: 12 });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [attempts]);

  const s = sim.state;
  const lane = s.lanes.find((l) => l.mode === mode)!;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      drawWorld(ctx, v, APARTMENT, p);
      const goalTint = tint(p.graph, 45, p.accent);
      if (lane.rc) {
        lane.rc.trees.forEach((t, ti) => {
          ctx.save();
          ctx.strokeStyle = ti === 0 ? p.graph : goalTint;
          ctx.lineWidth = 1;
          ctx.globalAlpha = 0.75;
          ctx.beginPath();
          for (let i = 1; i < t.tree.size; i++) {
            const n = t.tree.nodes[i];
            const q = t.tree.nodes[n.parent].q;
            ctx.moveTo(sx(v, q[0]), sy(v, q[1]));
            ctx.lineTo(sx(v, n.q[0]), sy(v, n.q[1]));
          }
          ctx.stroke();
          ctx.restore();
        });
      } else if (lane.sbl) {
        const nodes = lane.sbl.nodes;
        for (const checked of [false, true]) {
          ctx.save();
          ctx.lineWidth = checked ? 1.5 : 1;
          ctx.globalAlpha = checked ? 0.9 : 0.5;
          if (!checked) ctx.setLineDash([3, 3]);
          for (const treeIdx of [0, 1] as const) {
            ctx.strokeStyle = treeIdx === 0 ? p.graph : goalTint;
            ctx.beginPath();
            for (const e of lane.sbl.edges()) {
              if (e.checked !== checked || e.tree !== treeIdx) continue;
              ctx.moveTo(sx(v, nodes[e.a].q[0]), sy(v, nodes[e.a].q[1]));
              ctx.lineTo(sx(v, nodes[e.b].q[0]), sy(v, nodes[e.b].q[1]));
            }
            ctx.stroke();
          }
          ctx.restore();
        }
      }
      const poly = (path: readonly number[][], color: string, width: number, alpha: number, dashed = false) => {
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.globalAlpha = alpha;
        ctx.lineJoin = 'round';
        if (dashed) ctx.setLineDash([6, 4]);
        ctx.beginPath();
        path.forEach((q, i) => (i === 0 ? ctx.moveTo(sx(v, q[0]), sy(v, q[1])) : ctx.lineTo(sx(v, q[0]), sy(v, q[1]))));
        ctx.stroke();
        ctx.restore();
      };
      if (lane.path) poly(lane.path, p.path, 2, 0.55);
      if (lane.shortcut) poly(lane.shortcut, p.path, 3.5, 1, true);
      paintDot(ctx, sx(v, APARTMENT_START[0]), sy(v, APARTMENT_START[1]), 7, p.start);
      paintDot(ctx, sx(v, APARTMENT_GOAL[0]), sy(v, APARTMENT_GOAL[1]), 7, p.goal);
      const its = lane.rc ? lane.rc.attempts : lane.sbl!.iterations;
      const nodes = lane.rc ? lane.rc.nodeCount : lane.sbl!.nodes.length;
      label(ctx, `${MODES.find((m) => m.key === mode)!.label} · iteration ${its} · ${nodes} nodes · ${lane.lab.free.checks.toLocaleString()} checks${lane.path ? ` · merged at ${lane.solvedAt}` : its >= attempts ? ' · gave up' : ''}`, sx(v, 0.2), sy(v, 8.7), lane.path ? p.path : p.ink, { size: 11, weight: 600 });
    },
    [lane, mode, attempts],
  );

  return (
    <WidgetFrame
      id="w12.2"
      title="Bidirectional Connect"
      teaches="Greedier is not always faster: Connect/Connect wins in open rooms and loses in clutter, where long greedy runs end Trapped and every one of them was paid for in collision checks."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path']}
      wide
      caption={
        <>
          Two trees for Rusty-as-a-disc, one rooted at the start and one at the goal, grown toward
          each other across the Apartment&rsquo;s doorways. Four planners run on the <em>same</em>{' '}
          seed in lockstep; the canvas shows the one selected and the table shows all four. The
          goal tree wears the lighter blue. The dial is greediness: <em>Extend</em> moves η, <em>
          Connect</em> keeps going until it reaches or is trapped. In SBL mode the dashed edges have
          never been collision-checked — only the ones on a candidate bridging path were, and the
          check counter shows what that saves. The thin purple path is the raw one; the thick dashed
          one is after Chapter 11&rsquo;s greedy shortcutting.
        </>
      }
    >
      <SimCanvas world={APARTMENT.bounds} aspect={12 / 9} draw={draw} deps={[sim.tick, s, mode]} ariaLabel="Two trees growing toward each other through an apartment floorplan, one from the start and one from the goal, with the merged path drawn once they meet." />
      <div className="overflow-x-auto border-t border-fd-border">
        <table className="w-full min-w-[34rem] border-collapse text-left font-ui text-[0.78rem]">
          <thead>
            <tr className="border-b border-fd-border">
              <th className="px-3 py-1.5 font-medium">planner</th>
              <th className="px-3 py-1.5 text-right font-medium">merged at</th>
              <th className="px-3 py-1.5 text-right font-medium">nodes</th>
              <th className="px-3 py-1.5 text-right font-medium">collision checks</th>
              <th className="px-3 py-1.5 text-right font-medium">path (m)</th>
              <th className="px-3 py-1.5 text-right font-medium">after shortcut (m)</th>
            </tr>
          </thead>
          <tbody className="font-mono tabular-nums">
            {s.lanes.map((l) => {
              const its = l.rc ? l.rc.attempts : l.sbl!.iterations;
              const nodes = l.rc ? l.rc.nodeCount : l.sbl!.nodes.length;
              return (
                <tr key={l.mode} className={`border-b border-fd-border last:border-b-0 ${l.mode === mode ? 'bg-fd-accent/40' : ''}`}>
                  <td className="px-3 py-1 font-ui">
                    <button type="button" onClick={() => setMode(l.mode)} className="text-left hover:underline">
                      {MODES.find((m) => m.key === l.mode)!.label}
                    </button>
                  </td>
                  <td className="px-3 py-1 text-right">{l.solvedAt ?? (its >= attempts ? 'gave up' : `… ${its}`)}</td>
                  <td className="px-3 py-1 text-right">{nodes}</td>
                  <td className="px-3 py-1 text-right">{(l.checksAtSolve ?? l.lab.free.checks).toLocaleString()}</td>
                  <td className="px-3 py-1 text-right">{l.path ? pathLength(l.lab, l.path).toFixed(2) : '—'}</td>
                  <td className="px-3 py-1 text-right">{l.shortcut ? pathLength(l.lab, l.shortcut).toFixed(2) : '—'}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <ControlPanel columns={2} title="headline — the greediness dial">
        <ButtonRow>
          {MODES.map((m) => (
            <ActionButton key={m.key} emphasis={mode === m.key} onClick={() => setMode(m.key)}>
              {m.label}
            </ActionButton>
          ))}
        </ButtonRow>
        <Slider label="merge attempts ℓ (budget per planner; restarts)" value={attempts} min={500} max={8000} step={500} onChange={setAttempts} />
      </ControlPanel>
      <Transport playing={sim.playing} onToggle={sim.toggle} onStep={sim.stepOnce} onReset={sim.reset} onReseed={() => sim.reseed()} seed={sim.seed} tick={sim.tick} speed={sim.speed} onSpeed={sim.setSpeed} />
    </WidgetFrame>
  );
}
