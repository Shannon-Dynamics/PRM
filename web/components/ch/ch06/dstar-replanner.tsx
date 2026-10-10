'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { WidgetFrame } from '@/components/sim/widget-frame';
import { SimCanvas } from '@/components/sim/sim-canvas';
import { ActionButton, ButtonRow, ControlPanel, Slider, Transport } from '@/components/sim/controls';
import { useSimulation } from '@/lib/sim/use-simulation';
import { clear, label, sl, sx, sy, type Palette, type Viewport } from '@/lib/sim/draw';
import { Rng } from '@/lib/prob/rng';
import { APARTMENT } from '@/lib/sim/world';
import { astar } from '@/lib/search/best-first';
import { DStar, chosetGateScenario, type GateScenario } from '@/lib/search/dstar';
import { DStarLite } from '@/lib/search/dstar-lite';
import {
  CHOSET_DIAGONAL,
  GridGraph,
  cellIJ,
  cloneGrid,
  colRowOf,
  freeCellNear,
  gridFromWorld,
  gridHeuristic,
  type OccupancyGrid,
} from '@/lib/search/grid';

/**
 * w6.2 — the D* Replanner.
 *
 * Choset's gate scenario (§H.3, figures H.21–H.26) and then the Apartment:
 * Rusty follows the planned path, and at a chosen step a gate cell flips to
 * obstacle. D* and D* Lite repair only the states whose cost-to-goal actually
 * depended on the gate; a from-scratch A* floods again. The widget counts the
 * cells each touches and paints D*'s RAISE states (orange outline, k < h) and
 * LOWER states (blue outline) — the two kinds of news that incremental search
 * is made of. Replanning does not mean searching again.
 */

type Algo = 'dstar' | 'dstarlite' | 'astar';
type Scenario = 'gate' | 'apartment';

interface Repair {
  atStep: number;
  touched: number[];
  raised: number[];
  lowered: number[];
  freshExpansions: number;
  freshClosed: number[];
}

interface State {
  sc: GateScenario;
  truth: OccupancyGrid;
  believedPathFull: number[];
  robot: number;
  trail: number[];
  path: number[];
  dstar: DStar | null;
  lite: DStarLite | null;
  stepsTaken: number;
  gateClosed: boolean;
  repair: Repair | null;
  finished: boolean;
  noPath: boolean;
}

function apartmentScenario(seed: number): GateScenario {
  const grid = gridFromWorld(APARTMENT, 0.5, 0.22);
  const rng = new Rng(seed);
  const start = freeCellNear(grid, 0.75 + rng.uniform(0, 1.5), 0.75 + rng.uniform(0, 1.5));
  const goal = freeCellNear(grid, 10.25 + rng.uniform(0, 1.2), 7.25 + rng.uniform(0, 1.2));
  // The gate: a cell on the initial optimal path, about 55% of the way along — in the
  // Apartment that is almost always a doorway or the corridor.
  const graph = new GridGraph(grid, { connectivity: 8, diagonal: CHOSET_DIAGONAL, cornerCutting: false });
  const t = astar(graph, start, goal, gridHeuristic(grid, goal, 'octile', CHOSET_DIAGONAL));
  const gate = t.path.length > 4 ? t.path[Math.floor(t.path.length * 0.55)] : start;
  return { grid, start, goal, gate };
}

export function DStarReplanner() {
  const [scenario, setScenario] = useState<Scenario>('gate');
  const [algo, setAlgo] = useState<Algo>('dstar');
  const [closeStep, setCloseStep] = useState(1);

  const seedRef = useRef(6);
  const base = useMemo(() => (scenario === 'gate' ? chosetGateScenario() : apartmentScenario(seedRef.current)), [scenario]);
  const opts = useMemo(
    () => (scenario === 'gate' ? { connectivity: 8 as const, diagonal: CHOSET_DIAGONAL, cornerCutting: true } : { connectivity: 8 as const, diagonal: CHOSET_DIAGONAL, cornerCutting: false }),
    [scenario],
  );

  const init = useCallback(
    (seed: number): State => {
      const sc = scenario === 'gate' ? base : seed === seedRef.current ? base : apartmentScenario(seed);
      seedRef.current = seed;
      const truth = cloneGrid(sc.grid);
      let dstar: DStar | null = null;
      let lite: DStarLite | null = null;
      let path: number[] = [];
      if (algo === 'dstar') {
        dstar = new DStar(sc.grid, sc.goal, opts);
        dstar.planInitial(sc.start);
        path = dstar.backpointerPath(sc.start) ?? [];
      } else if (algo === 'dstarlite') {
        lite = new DStarLite(sc.grid, sc.start, sc.goal, opts);
        path = lite.pathFromStart() ?? [];
      } else {
        const t = astar(new GridGraph(sc.grid, opts), sc.start, sc.goal, gridHeuristic(sc.grid, sc.goal, 'octile', CHOSET_DIAGONAL));
        path = t.path;
      }
      return {
        sc,
        truth,
        believedPathFull: path,
        robot: sc.start,
        trail: [sc.start],
        path,
        dstar,
        lite,
        stepsTaken: 0,
        gateClosed: false,
        repair: null,
        finished: false,
        noPath: path.length === 0,
      };
    },
    [scenario, base, algo, opts],
  );

  const step = useCallback(
    (s: State): State => {
      if (s.finished || s.noPath) return s;
      const goal = s.sc.goal;
      const gate = s.sc.gate;
      // Close the gate at the chosen step (if the robot has not already passed it).
      let truth = s.truth;
      let gateClosed = s.gateClosed;
      if (!gateClosed && s.stepsTaken === closeStep && s.robot !== gate) {
        truth = cloneGrid(s.truth);
        truth.occupied[gate] = 1;
        gateClosed = true;
      }
      // Sense: a discrepancy within one cell of the robot triggers a repair.
      let repair = s.repair;
      let path = s.path;
      let dstar = s.dstar;
      let lite = s.lite;
      const nextOnPath = path.length > 1 ? path[1] : null;
      const aboutToHit = nextOnPath !== null && truth.occupied[nextOnPath] === 1;
      const adjacentToGate = gateClosed && s.robot !== gate && Math.max(...cellIJ(truth, s.robot).map((c, k) => Math.abs(c - cellIJ(truth, gate)[k]))) <= 1;
      if ((aboutToHit || adjacentToGate) && repair === null) {
        const fresh = astar(new GridGraph(truth, opts), s.robot, goal, gridHeuristic(truth, goal, 'octile', CHOSET_DIAGONAL));
        const freshClosed: number[] = fresh.pops.map((pp) => pp.node);
        if (algo === 'dstar' && dstar) {
          const st = dstar.senseAndRepair(s.robot, truth);
          repair = { atStep: s.stepsTaken, touched: [...new Set([...st.popped, ...st.inserted])], raised: st.raised, lowered: st.lowered, freshExpansions: fresh.expansions, freshClosed };
          path = dstar.backpointerPath(s.robot) ?? [];
          if (path.length && dstar.pathCost(path) >= 10000) path = [];
        } else if (algo === 'dstarlite' && lite) {
          const changes = [{ cell: gate, occupied: true }];
          lite.updateCells(changes);
          repair = { atStep: s.stepsTaken, touched: [...lite.stats.touched], raised: [], lowered: [], freshExpansions: fresh.expansions, freshClosed };
          path = lite.pathFromStart() ?? [];
        } else {
          repair = { atStep: s.stepsTaken, touched: freshClosed, raised: [], lowered: [], freshExpansions: fresh.expansions, freshClosed };
          path = fresh.path;
        }
        if (path.length === 0) return { ...s, truth, gateClosed, repair, path, noPath: true };
      }
      // Take one step along the current plan.
      if (s.robot === goal) return { ...s, truth, gateClosed, finished: true };
      let next: number | null = null;
      if (algo === 'dstar' && dstar) {
        const bp = dstar.backpointerPath(s.robot);
        next = bp && bp.length > 1 ? bp[1] : null;
        path = bp ?? [];
      } else if (algo === 'dstarlite' && lite) {
        next = lite.nextStep();
        path = lite.pathFromStart() ?? [];
      } else {
        const idx = path.indexOf(s.robot);
        next = idx >= 0 && idx + 1 < path.length ? path[idx + 1] : null;
      }
      if (next === null || truth.occupied[next]) return { ...s, truth, gateClosed, repair, path, noPath: next === null };
      const robot = next;
      return {
        ...s,
        truth,
        gateClosed,
        repair,
        path: algo === 'astar' ? path : path.length && path[0] === robot ? path : path.slice(Math.max(0, path.indexOf(robot))),
        robot,
        trail: [...s.trail, robot],
        stepsTaken: s.stepsTaken + 1,
        finished: robot === goal,
      };
    },
    [algo, closeStep, opts],
  );

  const sim = useSimulation<State>({ init, step, fps: 3, initialSeed: 6, maxTicks: 80, loop: true });
  const resetRef = useRef(sim.reset);
  resetRef.current = sim.reset;
  useEffect(() => {
    resetRef.current();
  }, [scenario, algo, closeStep]);

  const s = sim.state;
  const grid = s.truth;

  const draw = useCallback(
    (ctx: CanvasRenderingContext2D, v: Viewport, p: Palette) => {
      clear(ctx, v, p);
      const cw = Math.ceil(sl(v, 1)) + 1;
      const X = (x: number) => sx(v, x), Y = (y: number) => sy(v, y);
      ctx.save();
      ctx.fillStyle = p.free;
      ctx.fillRect(X(0), Y(grid.ny), sl(v, grid.nx), sl(v, grid.ny));
      // Repair footprint first, so obstacles paint over it.
      if (s.repair) {
        const set = new Set(s.repair.touched);
        ctx.fillStyle = p.graph;
        ctx.globalAlpha = algo === 'astar' ? 0.35 : 0.3;
        for (const c of set) {
          const [i, j] = cellIJ(grid, c);
          ctx.fillRect(X(i), Y(j + 1), cw, cw);
        }
        ctx.globalAlpha = 1;
      }
      for (let k = 0; k < grid.occupied.length; k++) {
        if (!grid.occupied[k]) continue;
        const [i, j] = cellIJ(grid, k);
        ctx.fillStyle = p.obstacle;
        ctx.fillRect(X(i), Y(j + 1), cw, cw);
      }
      // The gate.
      {
        const [i, j] = cellIJ(grid, s.sc.gate);
        ctx.strokeStyle = p.goal;
        ctx.lineWidth = 2;
        ctx.setLineDash(s.gateClosed ? [] : [4, 3]);
        ctx.strokeRect(X(i) + 2, Y(j + 1) + 2, sl(v, 1) - 4, sl(v, 1) - 4);
        ctx.setLineDash([]);
        if (scenario === 'gate') label(ctx, s.gateClosed ? 'gate closed' : 'gate (open)', X(i + 0.5), Y(j + 1) - 8, p.goal, { size: 9.5, align: 'center', weight: 600 });
      }
      // Raised / lowered outlines (D* only).
      if (s.repair && algo === 'dstar') {
        ctx.lineWidth = 2;
        for (const c of s.repair.lowered) {
          const [i, j] = cellIJ(grid, c);
          ctx.strokeStyle = p.graph;
          ctx.strokeRect(X(i) + 1.5, Y(j + 1) + 1.5, sl(v, 1) - 3, sl(v, 1) - 3);
        }
        for (const c of s.repair.raised) {
          const [i, j] = cellIJ(grid, c);
          ctx.strokeStyle = p.robot;
          ctx.strokeRect(X(i) + 1.5, Y(j + 1) + 1.5, sl(v, 1) - 3, sl(v, 1) - 3);
        }
      }
      // Grid lines for the small scenario.
      if (scenario === 'gate') {
        ctx.strokeStyle = p.grid;
        ctx.lineWidth = 1;
        for (let i = 0; i <= grid.nx; i++) { ctx.beginPath(); ctx.moveTo(X(i), Y(0)); ctx.lineTo(X(i), Y(grid.ny)); ctx.stroke(); }
        for (let j = 0; j <= grid.ny; j++) { ctx.beginPath(); ctx.moveTo(X(0), Y(j)); ctx.lineTo(X(grid.nx), Y(j)); ctx.stroke(); }
        // Choset's (col,row) labels and the D* h values.
        for (let k = 0; k < grid.occupied.length; k++) {
          const [i, j] = cellIJ(grid, k);
          const [c, r] = colRowOf(grid, k);
          label(ctx, `(${c},${r})`, X(i) + 3, Y(j + 1) + 8, p.truth, { size: 8 });
          if (s.dstar && !grid.occupied[k]) {
            const hval = s.dstar.costToGo[k];
            if (Number.isFinite(hval) && hval < 10000) label(ctx, `h ${hval.toFixed(1)}`, X(i + 0.5), Y(j + 0.5) + 4, p.ink, { size: 9, align: 'center', weight: 600 });
            if (s.dstar.isRaised(k)) label(ctx, `k ${s.dstar.kmin[k].toFixed(1)}`, X(i + 0.5), Y(j + 0.5) + 14, p.robot, { size: 8, align: 'center' });
          }
        }
      }
      // Planned path, trail, robot, start, goal.
      const poly = (cells: number[], color: string, width: number, dashed = false, alpha = 1) => {
        if (cells.length < 2) return;
        ctx.save();
        ctx.globalAlpha = alpha;
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineJoin = 'round';
        ctx.setLineDash(dashed ? [5, 4] : []);
        ctx.beginPath();
        cells.forEach((c, n) => {
          const [i, j] = cellIJ(grid, c);
          if (n === 0) ctx.moveTo(X(i + 0.5), Y(j + 0.5)); else ctx.lineTo(X(i + 0.5), Y(j + 0.5));
        });
        ctx.stroke();
        ctx.restore();
      };
      poly(s.believedPathFull, p.path, 1.5, true, 0.4);
      poly(s.path, p.path, 3);
      poly(s.trail, p.robot, 3.5, false, 0.85);
      const disc = (c: number, color: string, r: number) => {
        const [i, j] = cellIJ(grid, c);
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(X(i + 0.5), Y(j + 0.5), sl(v, r), 0, Math.PI * 2);
        ctx.fill();
      };
      disc(s.sc.start, p.start, 0.3);
      disc(s.sc.goal, p.goal, 0.3);
      disc(s.robot, p.robot, 0.36);
      ctx.restore();
    },
    [s, grid, algo, scenario],
  );

  const maxStep = Math.max(1, s.believedPathFull.length - 2);
  const algoName = algo === 'dstar' ? 'D*' : algo === 'dstarlite' ? 'D* Lite' : 'from-scratch A*';

  return (
    <WidgetFrame
      id="w6.2"
      title="D* Replanner"
      teaches="Replanning does not mean searching again. Most of the old search is still correct after the world changes, and D* proves which part — then fixes only the rest."
      colorKey={['start', 'goal', 'obstacle', 'graph', 'path', 'robot']}
      wide
      caption={
        <>
          Choset&rsquo;s gate (figures H.21–H.26): Rusty plans from (2,1) to (7,6) with the gate
          (4,3) open and starts along the dashed diagonal. At the chosen step the gate closes.
          When Rusty is adjacent it notices, and the planner repairs: D* re-inserts the changed
          cell, pops (3,2) as a RAISE state (orange outline, k &lt; h), and lets the LOWER
          neighbour (4,1) — whose own path never used the gate — rescue it at h = 6.2 + 1.4 =
          7.6. The blue footprint is every cell the repair touched; compare its count with the
          expansions a from-scratch A* needs from the same spot. Switch to D* Lite, or to A* to
          see the flood; then try the Apartment, where a door closes while Rusty is en route.
        </>
      }
    >
      <SimCanvas
        world={{ minX: 0, minY: 0, maxX: grid.nx, maxY: grid.ny }}
        draw={draw}
        deps={[sim.tick, s, algo, scenario]}
        aspect={grid.nx / grid.ny}
        padding={scenario === 'gate' ? 0.4 : 0.5}
        ariaLabel="A grid with obstacles, a gate cell, a robot following a planned path toward a goal, and the cells touched by the replanner shaded blue after the gate closes."
      />
      <div className="grid grid-cols-2 divide-x divide-fd-border border-t border-fd-border text-center sm:grid-cols-4">
        <Stat label="algorithm" value={algoName} />
        <Stat label={algo === 'astar' ? 'cells expanded by replan' : 'cells touched by repair'} value={s.repair ? String(s.repair.touched.length) : s.gateClosed ? 'not yet sensed' : '—'} />
        <Stat label="from-scratch A* expansions" value={s.repair ? String(s.repair.freshExpansions) : '—'} />
        <Stat
          label={algo === 'dstar' ? 'raised / lowered states' : 'status'}
          value={
            algo === 'dstar' && s.repair
              ? `${s.repair.raised.length} / ${s.repair.lowered.length}`
              : s.noPath
                ? 'no path'
                : s.finished
                  ? 'arrived'
                  : s.repair
                    ? 'repaired'
                    : 'driving'
          }
        />
      </div>
      <ControlPanel columns={3}>
        <Slider
          label="Step at which the gate closes"
          role="goal"
          value={Math.min(closeStep, maxStep)}
          min={0}
          max={maxStep}
          step={1}
          onChange={setCloseStep}
          format={(val) => `step ${val.toFixed(0)}`}
          help="Rusty takes one cell per step along its plan; the gate flips to obstacle at this step."
        />
        <ButtonRow>
          <ActionButton emphasis={algo === 'dstar'} onClick={() => setAlgo('dstar')}>D*</ActionButton>
          <ActionButton emphasis={algo === 'dstarlite'} onClick={() => setAlgo('dstarlite')}>D* Lite</ActionButton>
          <ActionButton emphasis={algo === 'astar'} onClick={() => setAlgo('astar')}>A* from scratch</ActionButton>
        </ButtonRow>
        <ButtonRow>
          <ActionButton emphasis={scenario === 'gate'} onClick={() => setScenario('gate')}>Choset&rsquo;s gate 7×6</ActionButton>
          <ActionButton emphasis={scenario === 'apartment'} onClick={() => setScenario('apartment')}>the Apartment</ActionButton>
        </ButtonRow>
      </ControlPanel>
      <Transport
        playing={sim.playing}
        onToggle={sim.toggle}
        onStep={sim.stepOnce}
        onReset={sim.reset}
        onReseed={scenario === 'apartment' ? () => sim.reseed() : undefined}
        seed={sim.seed}
        tick={sim.tick}
        speed={sim.speed}
        onSpeed={sim.setSpeed}
      />
    </WidgetFrame>
  );
}

function Stat({ label: l, value }: { label: string; value: string }) {
  return (
    <div className="px-2 py-1.5">
      <div className="eyebrow">{l}</div>
      <div className="font-mono text-sm tabular-nums">{value}</div>
    </div>
  );
}
