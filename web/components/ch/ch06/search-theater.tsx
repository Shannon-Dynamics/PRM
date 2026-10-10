'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import {
  BestFirst,
  PRIORITY_BFS,
  PRIORITY_DIJKSTRA,
  PRIORITY_GREEDY,
  ZERO_HEURISTIC,
  weightedAStar,
  type Priority,
} from '@/lib/search/best-first';
import {
  CHOSET_DIAGONAL,
  GridGraph,
  cellIJ,
  cellIndex,
  cloneGrid,
  freeCellNear,
  gridFromWorld,
  gridHeuristic,
  type Connectivity,
  type OccupancyGrid,
} from '@/lib/search/grid';

/**
 * w6.1 — the Search Theater.
 *
 * Four lanes, one grid, one clock. Breadth-first, Dijkstra, greedy, and A*
 * are the same `BestFirst` engine with four values of the priority knob, and
 * they expand in lockstep so the reader can watch the explored sets grow
 * (blue: darker closed, lighter open) and the solutions appear (purple). The
 * headline slider is ε in f = g + ε·h: at 0 the A* lane *is* Dijkstra, at 1 it
 * is A*, and past 1 it buys expansions with path quality — the "optimality
 * lost" badge lights the first time its cost exceeds Dijkstra's.
 */

const W = 24;
const H = 16;
const CELL = 0.5;
const GAP = 1.5;
const BOUNDS = { minX: 0, minY: 0, maxX: W * CELL, maxY: H * CELL };
const ARENA = { minX: 0, minY: 0, maxX: 2 * W + GAP, maxY: 2 * H + GAP };

interface Lane {
  key: 'bfs' | 'dijkstra' | 'greedy' | 'astar';
  name: string;
  ox: number;
  oy: number;
}

const LANES: Lane[] = [
  { key: 'bfs', name: 'breadth-first (f = link length)', ox: 0, oy: H + GAP },
  { key: 'dijkstra', name: 'Dijkstra (f = g)', ox: W + GAP, oy: H + GAP },
  { key: 'greedy', name: 'greedy (f = h)', ox: 0, oy: 0 },
  { key: 'astar', name: 'A* (f = g + ε·h)', ox: W + GAP, oy: 0 },
];

interface State {
  searches: Record<Lane['key'], BestFirst<number>>;
  done: boolean;
}

type DragMode = 'none' | 'start' | 'goal' | 'paint';

/** The base layout: the Apartment's south rooms and corridor, rasterized for disc-Rusty, plus seeded clutter. */
function buildGrid(seed: number, clutter: number): OccupancyGrid {
  const g = gridFromWorld(APARTMENT, CELL, 0.22, BOUNDS);
  const rng = new Rng(seed);
  for (let k = 0; k < g.occupied.length; k++) if (!g.occupied[k] && rng.next() < clutter) g.occupied[k] = 1;
  return g;
}

export function SearchTheater() {
  const [eps, setEps] = useState(1);
  const [connectivity, setConnectivity] = useState<Connectivity>(8);
  const [clutter, setClutter] = useState(0.08);
  const [grid, setGrid] = useState<OccupancyGrid>(() => buildGrid(42, 0.08));
  const [start, setStart] = useState(() => freeCellNear(buildGrid(42, 0.08), 0.75, 0.75));
  const [goal, setGoal] = useState(() => freeCellNear(buildGrid(42, 0.08), 11.25, 7.25));
  const [lostAtEps, setLostAtEps] = useState<number | null>(null);
  const drag = useRef<DragMode>('none');
  const painted = useRef(0);
  const seedRef = useRef(42);

  const graph = useMemo(() => new GridGraph(grid, { connectivity, diagonal: CHOSET_DIAGONAL, cornerCutting: false }), [grid, connectivity]);
  const h = useMemo(() => gridHeuristic(grid, goal, connectivity === 8 ? 'octile' : 'manhattan', CHOSET_DIAGONAL), [grid, goal, connectivity]);

  const init = useCallback((seed: number): State => {
    // Re-roll rebuilds the clutter; keep start/goal where they are if still free.
    if (seed !== seedRef.current) {
      seedRef.current = seed;
      const g = buildGrid(seed, clutter);
      g.occupied[start] = 0;
      g.occupied[goal] = 0;
      // Defer the grid swap to an effect-free path: the search objects below use the fresh grid directly.
      queueMicrotask(() => setGrid(g));
      const gr = new GridGraph(g, { connectivity, diagonal: CHOSET_DIAGONAL, cornerCutting: false });
      const hh = gridHeuristic(g, goal, connectivity === 8 ? 'octile' : 'manhattan', CHOSET_DIAGONAL);
      return { searches: makeSearches(gr, start, goal, hh, eps), done: false };
    }
    return { searches: makeSearches(graph, start, goal, h, eps), done: false };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [graph, h, start, goal, eps, connectivity, clutter]);

  const step = useCallback((s: State): State => {
    if (s.done) return s;
    let running = false;
    for (const lane of LANES) {
      const bf = s.searches[lane.key];
      // Two expansions per tick keeps all four lanes legible yet finishes Dijkstra in seconds.
      for (let k = 0; k < 2; k++) if (bf.status === 'running') bf.step();
      if (bf.status === 'running') running = true;
    }
    return { ...s, done: !running };
  }, []);

  const sim = useSimulation<State>({ init, step, fps: 20, initialSeed: 42 });

  // Any change to the problem restarts all four lanes.
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [grid, start, goal, eps, connectivity]);

  const s = sim.state;
  const dijkstraCost = s.searches.dijkstra.cost();
  const astarCost = s.searches.astar.cost();
  const lost = Number.isFinite(dijkstraCost) && Number.isFinite(astarCost) && astarCost > dijkstraCost + 1e-9;
  useEffect(() => {
    if (lost && lostAtEps === null) setLostAtEps(eps);
    if (!lost && lostAtEps !== null && eps <= 1) setLostAtEps(null);
  }, [lost, eps, lostAtEps]);

  const laneAt = (wx: number, wy: number): { lane: Lane; cell: number } | null => {
    for (const lane of LANES) {
      const lx = wx - lane.ox, ly = wy - lane.oy;
      if (lx >= 0 && lx < W && ly >= 0 && ly < H) return { lane, cell: cellIndex(grid, Math.floor(lx), Math.floor(ly)) };
    }
    return null;
  };

  const onPointer = useCallback(
    (w: [number, number], phase: 'down' | 'move' | 'up') => {
      const hit = laneAt(w[0], w[1]);
      if (phase === 'down') {
        if (!hit) return;
        if (hit.cell === start) drag.current = 'start';
        else if (hit.cell === goal) drag.current = 'goal';
        else {
          drag.current = 'paint';
          painted.current = hit.cell;
          const g = cloneGrid(grid);
          g.occupied[hit.cell] = g.occupied[hit.cell] ? 0 : 1;
          setGrid(g);
        }
      } else if (phase === 'move') {
        if (!hit || drag.current === 'none') return;
        if (drag.current === 'start' && !grid.occupied[hit.cell] && hit.cell !== goal) setStart(hit.cell);
        else if (drag.current === 'goal' && !grid.occupied[hit.cell] && hit.cell !== start) setGoal(hit.cell);
        else if (drag.current === 'paint' && hit.cell !== painted.current && hit.cell !== start && hit.cell !== goal) {
          painted.current = hit.cell;
          const g = cloneGrid(grid);
          g.occupied[hit.cell] = 1;
          setGrid(g);
        }
      } else drag.current = 'none';
    },
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [grid, start, goal],
  );

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const cw = Math.ceil(sl(v, 1)) + 1;
      for (const lane of LANES) {
        const bf = s.searches[lane.key];
        const X = (x: number) => sx(v, x + lane.ox);
        const Y = (y: number) => sy(v, y + lane.oy);
        ctx.save();
        ctx.fillStyle = p.free;
        ctx.fillRect(X(0), Y(H), sl(v, W), sl(v, H));
        for (let k = 0; k < grid.occupied.length; k++) {
          const [i, j] = cellIJ(grid, k);
          if (grid.occupied[k]) {
            ctx.fillStyle = p.obstacle;
            ctx.globalAlpha = 1;
          } else if (bf.isClosed(k)) {
            ctx.fillStyle = p.graph;
            ctx.globalAlpha = 0.55;
          } else if (bf.isOpen(k)) {
            ctx.fillStyle = p.graph;
            ctx.globalAlpha = 0.22;
          } else continue;
          ctx.fillRect(X(i), Y(j + 1), cw, cw);
        }
        ctx.globalAlpha = 1;
        // Lane frame.
        ctx.strokeStyle = p.grid;
        ctx.lineWidth = 1;
        ctx.strokeRect(X(0), Y(H), sl(v, W), sl(v, H));
        // Solution path.
        const path = bf.path();
        if (path.length > 1) {
          ctx.strokeStyle = p.path;
          ctx.lineWidth = 3;
          ctx.lineJoin = 'round';
          ctx.beginPath();
          path.forEach((c, n) => {
            const [i, j] = cellIJ(grid, c);
            if (n === 0) ctx.moveTo(X(i + 0.5), Y(j + 0.5)); else ctx.lineTo(X(i + 0.5), Y(j + 0.5));
          });
          ctx.stroke();
        }
        // Start and goal.
        const [si, sj] = cellIJ(grid, start);
        const [gi, gj] = cellIJ(grid, goal);
        ctx.fillStyle = p.start;
        ctx.beginPath(); ctx.arc(X(si + 0.5), Y(sj + 0.5), sl(v, 0.42), 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = p.goal;
        ctx.beginPath(); ctx.arc(X(gi + 0.5), Y(gj + 0.5), sl(v, 0.42), 0, Math.PI * 2); ctx.fill();
        // Scoreboard line.
        const cost = bf.cost();
        label(ctx, lane.name, X(0.2), Y(H) - 9, p.ink, { size: 11, weight: 700 });
        label(
          ctx,
          `${bf.pops.length} exp · ${bf.status === 'found' ? `cost ${cost.toFixed(1)}` : bf.status === 'failed' ? 'no path' : 'searching…'}`,
          X(W - 0.2),
          Y(H) - 9,
          bf.status === 'found' ? p.path : p.ink,
          { size: 11, weight: 600, align: 'right' },
        );
        if (lane.key === 'astar' && lost) label(ctx, 'OPTIMALITY LOST', X(W / 2), Y(-0.9), p.goal, { size: 10, weight: 700, align: 'center' });
        ctx.restore();
      }
    },
    [s, grid, start, goal, lost],
  );

  return (
    <WidgetFrame
      id="w6.1"
      title="Search Theater"
      teaches="A stronger heuristic means fewer expansions only while it stays optimistic. Past that, it buys speed with path quality — a better heuristic does not always mean a better path."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path']}
      wide
      caption={
        <>
          One grid — the Apartment&rsquo;s south rooms and corridor at 0.5 m, with seeded clutter —
          searched four ways in lockstep. Blue is what each search has looked at: closed cells
          darker, the open frontier lighter. Purple is the path each returns. Breadth-first and
          Dijkstra flood; greedy darts and often arrives with a bad path; A* with ε = 1 floods only
          where it must and matches Dijkstra&rsquo;s cost exactly. Now slide ε. Below 1 nothing
          changes but the expansion count. Above 1 the A* lane keeps getting cheaper to run while
          its cost drifts above Dijkstra&rsquo;s — the badge marks the moment. Drag the start and
          goal, click to paint walls, switch to 4-connectivity, re-roll the clutter.
        </>
      }
    >
      <SimCanvas
        world={ARENA}
        draw={draw}
        deps={[sim.tick, s, grid, start, goal, lost]}
        aspect={ARENA.maxX / ARENA.maxY}
        padding={0.6}
        onPointer={onPointer}
        ariaLabel="Four copies of the same occupancy grid, each being searched by a different algorithm: breadth-first, Dijkstra, greedy, and A*. Explored cells are blue, the found path purple, start green, goal red."
      />
      <div className="overflow-x-auto border-t border-fd-border">
        <table className="w-full min-w-[28rem] border-collapse text-left font-ui text-[0.78rem]">
          <thead>
            <tr className="border-b border-fd-border">
              <th className="px-3 py-1.5 font-medium">search</th>
              <th className="px-3 py-1.5 text-right font-medium">expansions</th>
              <th className="px-3 py-1.5 text-right font-medium">path cost</th>
              <th className="px-3 py-1.5 text-right font-medium">vs. Dijkstra</th>
              <th className="px-3 py-1.5 text-right font-medium">reopened</th>
            </tr>
          </thead>
          <tbody className="font-mono tabular-nums">
            {LANES.map((lane) => {
              const bf = s.searches[lane.key];
              const c = bf.cost();
              const ratio = Number.isFinite(c) && Number.isFinite(dijkstraCost) ? c / dijkstraCost : NaN;
              return (
                <tr key={lane.key} className="border-b border-fd-border last:border-b-0">
                  <td className="px-3 py-1 font-ui">
                    {lane.name}
                    {lane.key === 'astar' && lost ? (
                      <span className="ml-2 rounded-sm border border-fd-border px-1.5 py-0.5 font-mono text-[0.65rem]" style={{ color: 'var(--rm-goal)' }}>
                        optimality lost{lostAtEps !== null ? ` at ε = ${lostAtEps.toFixed(1)}` : ''}
                      </span>
                    ) : null}
                  </td>
                  <td className="px-3 py-1 text-right">{bf.pops.length}</td>
                  <td className="px-3 py-1 text-right">{Number.isFinite(c) ? c.toFixed(1) : bf.status === 'failed' ? 'none' : '…'}</td>
                  <td className="px-3 py-1 text-right">{Number.isFinite(ratio) ? `×${ratio.toFixed(3)}` : '—'}</td>
                  <td className="px-3 py-1 text-right">{bf.reopened}</td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
      <ControlPanel columns={3}>
        <Slider
          label="Heuristic weight ε (A* lane)"
          role="graph"
          value={eps}
          min={0}
          max={3}
          step={0.1}
          onChange={setEps}
          format={(val) => (val === 0 ? '0 — Dijkstra' : val === 1 ? '1 — A*' : val > 1 ? `${val.toFixed(1)} — weighted` : val.toFixed(1))}
          help="f = g + ε·h. Admissible for ε ≤ 1; past 1 the path may be up to ε times longer than optimal."
        />
        <ButtonRow>
          <ActionButton emphasis={connectivity === 8} onClick={() => setConnectivity(8)}>8-connected (1 / 1.4)</ActionButton>
          <ActionButton emphasis={connectivity === 4} onClick={() => setConnectivity(4)}>4-connected</ActionButton>
        </ButtonRow>
        <Slider
          label="Clutter density (re-roll applies it)"
          role="obstacle"
          value={clutter}
          min={0}
          max={0.25}
          step={0.01}
          onChange={setClutter}
          format={(val) => `${(val * 100).toFixed(0)}%`}
        />
      </ControlPanel>
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

function makeSearches(graph: GridGraph, start: number, goal: number, h: (n: number) => number, eps: number): State['searches'] {
  const mk = (prio: Priority, heuristic: (n: number) => number) => new BestFirst(graph, start, goal, heuristic, prio);
  return {
    bfs: mk(PRIORITY_BFS, ZERO_HEURISTIC),
    dijkstra: mk(PRIORITY_DIJKSTRA, ZERO_HEURISTIC),
    greedy: mk(PRIORITY_GREEDY, h),
    astar: mk(weightedAStar(eps), h),
  };
}
