/**
 * Numerical self-checks for Chapter 6's search module.
 *
 * The design doc's hand trace is the first pin: five A* pops with their exact
 * g/h/f triples, cost 5.2, ten Dijkstra expansions. Then the invariants: A*
 * with an admissible heuristic matches Dijkstra's cost on random grids; the
 * non-optimistic counterexample returns 8 where 5 exists; a consistent
 * heuristic never reopens; BFS is link-optimal; D* repairs Choset's gate to
 * k(3,2) = 7.6 via (4,1); D* Lite equals a fresh A* after every change; and a
 * universal plan is a Bellman fixed point.
 *
 *     import { runCh06Checks } from './search/__checks_ch06__';
 */

import type { CheckResult } from '../__checks__';
import { Rng } from '../prob/rng';
import { APARTMENT } from '../sim/world';
import { astar, bestFirst, bfs, dfs, dijkstra, greedy, pathCost, weightedAStar, ZERO_HEURISTIC } from './best-first';
import { DStar, OBSTACLE_COST, chosetGateScenario } from './dstar';
import { DStarLite } from './dstar-lite';
import { NONOPTIMISTIC, OPTIMISTIC_TWIN } from './graph';
import {
  CHOSET_DIAGONAL,
  GridGraph,
  MICRO_GOAL,
  MICRO_GRID,
  MICRO_START,
  cellFromColRow,
  colRowOf,
  freeCellNear,
  gridFromWorld,
  gridHeuristic,
  randomGrid,
} from './grid';
import { bellmanResidual, followGridPlan, followPlan, universalPlan, universalPlanGrid } from './policy';

const close = (a: number, b: number, tol = 1e-9) => Math.abs(a - b) <= tol;
const fmt = (x: number) => (Number.isFinite(x) ? x.toFixed(x === Math.round(x) ? 0 : 2) : '∞');

export function runCh06Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // The hand trace (design doc §3): 4 × 3 grid, obstacles (2,1), (2,2)
  // -------------------------------------------------------------------------
  const micro = new GridGraph(MICRO_GRID, { connectivity: 8, diagonal: CHOSET_DIAGONAL });
  const hMicro = gridHeuristic(MICRO_GRID, MICRO_GOAL, 'octile', CHOSET_DIAGONAL);
  {
    const t = astar(micro, MICRO_START, MICRO_GOAL, hMicro);
    const expected: [number, number, number, number, number][] = [
      [1, 1, 0, 3.0, 3.0],
      [1, 2, 1.0, 3.4, 4.4],
      [2, 3, 2.4, 2.8, 5.2],
      [3, 2, 3.8, 1.4, 5.2],
      [4, 1, 5.2, 0, 5.2],
    ];
    let ok = t.status === 'found' && t.pops.length === 5;
    const rows: string[] = [];
    expected.forEach(([c, r, g, h, f], i) => {
      const p = t.pops[i];
      if (!p) { ok = false; return; }
      const [pc, pr] = colRowOf(MICRO_GRID, p.node);
      rows.push(`(${pc},${pr}) g=${p.g.toFixed(1)} h=${p.h.toFixed(1)} f=${p.f.toFixed(1)}`);
      if (pc !== c || pr !== r || !close(p.g, g) || !close(p.h, h) || !close(p.f, f)) ok = false;
    });
    const path = t.path.map((n) => colRowOf(MICRO_GRID, n));
    const pathOk = JSON.stringify(path) === JSON.stringify([[1, 1], [1, 2], [2, 3], [3, 2], [4, 1]]);
    push(
      'search: A* on the 4×3 micro-grid pops (1,1) (1,2) (2,3) (3,2) (4,1) with the printed g/h/f, cost 5.2 in five expansions',
      ok && pathOk && close(t.cost, 5.2),
      `${rows.join(' · ')}; cost ${t.cost.toFixed(1)}`,
    );

    const d = dijkstra(micro, MICRO_START, MICRO_GOAL);
    push(
      'search: Dijkstra (h ≡ 0) on the same grid expands all ten free cells and returns the same cost 5.2',
      d.status === 'found' && d.expansions === 10 && close(d.cost, 5.2),
      `${d.expansions} expansions, cost ${d.cost.toFixed(1)}`,
    );

    const gr = greedy(micro, MICRO_START, MICRO_GOAL, hMicro);
    push(
      'search: greedy (f = h) happens to find the same 5.2 path here — in five expansions, with no guarantee',
      gr.status === 'found' && close(gr.cost, 5.2) && gr.expansions === 5,
      `${gr.expansions} expansions, cost ${gr.cost.toFixed(1)}`,
    );
  }

  // -------------------------------------------------------------------------
  // The non-optimistic counterexample (§H.2.5, figure H.20)
  // -------------------------------------------------------------------------
  {
    const { graph, start, goal, h } = NONOPTIMISTIC;
    const a = astar(graph, start, goal, (n) => h[n]);
    const d = dijkstra(graph, start, goal);
    const twin = astar(OPTIMISTIC_TWIN.graph, start, goal, (n) => OPTIMISTIC_TWIN.h[n]);
    push(
      'search: non-optimistic h(B) = 10 makes A* return S, A, G at cost 8 — the optimum S, B, G costs 5; fixing h(B) restores it',
      a.status === 'found' && close(a.cost, 8) && a.path.join('') === 'SAG' && close(d.cost, 5) && d.path.join('') === 'SBG' && close(twin.cost, 5),
      `A*: ${a.path.join(',')} cost ${fmt(a.cost)}; Dijkstra: ${d.path.join(',')} cost ${fmt(d.cost)}; admissible twin cost ${fmt(twin.cost)}`,
    );
  }

  // -------------------------------------------------------------------------
  // A* = Dijkstra on random grids; consistency ⇒ no reopening; weighted bound
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(606);
    let trials = 0, mismatches = 0, reopened = 0, fewer = 0, bfsLinkFail = 0, weightedViolations = 0, dfsFail = 0;
    for (let k = 0; k < 200; k++) {
      const nx = 12 + Math.floor(rng.uniform(0, 10));
      const ny = 8 + Math.floor(rng.uniform(0, 8));
      const start = 0;
      const goal = nx * ny - 1;
      const grid = randomGrid(nx, ny, rng.uniform(0.15, 0.35), rng, [start, goal]);
      const diag = rng.next() < 0.5 ? CHOSET_DIAGONAL : Math.SQRT2;
      const graph = new GridGraph(grid, { connectivity: 8, diagonal: diag });
      const h = gridHeuristic(grid, goal, 'octile', diag);
      const a = astar(graph, start, goal, h);
      const d = dijkstra(graph, start, goal);
      if (a.status !== d.status) mismatches++;
      if (a.status !== 'found') continue;
      trials++;
      if (!close(a.cost, d.cost, 1e-9)) mismatches++;
      if (a.reopened > 0) reopened++;
      if (a.expansions <= d.expansions) fewer++;
      // Weighted A*: cost ≤ ε·c*.
      const eps = 2.5;
      const w = bestFirst(graph, start, goal, h, weightedAStar(eps));
      if (w.status !== 'found' || w.cost > eps * d.cost + 1e-9) weightedViolations++;
      // BFS minimizes link length: equals Dijkstra's depth on the unit-weight (4-connected) version.
      const g4 = new GridGraph(grid, { connectivity: 4 });
      const b = bfs(g4, start, goal);
      const d4 = dijkstra(g4, start, goal);
      if (b.status !== d4.status || (b.status === 'found' && !close(b.path.length - 1, d4.cost))) bfsLinkFail++;
      // DFS is complete: it finds a path whenever one exists, and the path is valid.
      const f = dfs(graph, start, goal);
      if (f.status !== 'found' || !Number.isFinite(pathCost(graph, f.path))) dfsFail++;
    }
    push(
      'search: A* with the octile heuristic returns Dijkstra’s cost on 200 seeded random grids, never reopens, never expands more',
      trials > 100 && mismatches === 0 && reopened === 0 && fewer === trials,
      `${trials} solvable grids; mismatches ${mismatches}; reopenings ${reopened}; A* ≤ Dijkstra expansions in ${fewer}/${trials}`,
    );
    push(
      'search: weighted A* (ε = 2.5) stays within ε of optimal; BFS is link-optimal (4-connected); DFS is complete',
      weightedViolations === 0 && bfsLinkFail === 0 && dfsFail === 0,
      `weighted violations ${weightedViolations}; BFS/Dijkstra link mismatches ${bfsLinkFail}; DFS failures ${dfsFail}`,
    );
  }

  // -------------------------------------------------------------------------
  // Choset's gate (§H.3): initial diagonal path, then k(3,2) = 7.6 via (4,1)
  // -------------------------------------------------------------------------
  {
    const sc = chosetGateScenario();
    const ds = new DStar(sc.grid, sc.goal, { connectivity: 8, diagonal: CHOSET_DIAGONAL });
    const planned = ds.planInitial(sc.start);
    const initial = ds.backpointerPath(sc.start) ?? [];
    const initialCR = initial.map((c) => colRowOf(sc.grid, c));
    const initialOk =
      planned &&
      JSON.stringify(initialCR) === JSON.stringify([[2, 1], [3, 2], [4, 3], [5, 4], [6, 5], [7, 6]]) &&
      close(ds.costToGo[sc.start], 7.0) &&
      close(ds.costToGo[cellFromColRow(sc.grid, 4, 1)], 6.2);
    // The robot moves to (3,2) and finds the gate closed.
    const Xc = cellFromColRow(sc.grid, 3, 2);
    const truth = { ...sc.grid, occupied: Uint8Array.from(sc.grid.occupied) };
    truth.occupied[sc.gate] = 1;
    const stats = ds.senseAndRepair(Xc, truth);
    const repaired = ds.backpointerPath(Xc) ?? [];
    const repairedCR = repaired.map((c) => colRowOf(sc.grid, c));
    const via41 = ds.back[Xc] === cellFromColRow(sc.grid, 4, 1);
    const repairOk =
      via41 && close(ds.kmin[Xc], 7.6) && close(ds.costToGo[Xc], 7.6) && close(ds.pathCost(repaired), 7.6) && repaired.indexOf(sc.gate) < 0;
    // A from-scratch A* on the changed grid agrees on the cost; redoing D*'s own
    // backward Dijkstra from scratch (what "replan" means without repair) costs
    // more pops than the local repair did.
    const fresh = astar(new GridGraph(truth, { connectivity: 8, diagonal: CHOSET_DIAGONAL }), Xc, sc.goal, gridHeuristic(truth, sc.goal, 'octile', CHOSET_DIAGONAL));
    const redo = new DStar(truth, sc.goal, { connectivity: 8, diagonal: CHOSET_DIAGONAL });
    redo.planInitial(Xc);
    push(
      'search: D* on Choset’s gate — initial path (2,1)(3,2)(4,3)(5,4)(6,5)(7,6) cost 7.0; gate closes; (3,2) repaired to k = h = 7.6 via (4,1)',
      // The work counts are implementation-specific (both endpoints of a changed arc are
      // re-inserted), so they are regression-locked here because the chapter prints them.
      initialOk && repairOk && close(fresh.cost, 7.6) && stats.popped.length < redo.expansions &&
        stats.popped.length === 15 && stats.raised.length === 5 && redo.expansions === 32 && fresh.expansions === 7,
      `initial ${initialCR.map((p) => `(${p})`).join('')}; repaired ${repairedCR.map((p) => `(${p})`).join('')} cost ${fmt(ds.pathCost(repaired))}; repair popped ${stats.popped.length} (raised ${stats.raised.length}) vs ${redo.expansions} for a from-scratch backward Dijkstra (fresh A*: ${fresh.expansions})`,
    );
    push(
      'search: while raised, h(3,2) exceeds Choset’s obstacle cost — the back pointer through the closed gate is flagged before it is fixed',
      stats.raised.includes(Xc) && OBSTACLE_COST === 10000,
      `raised states: ${stats.raised.map((c) => `(${colRowOf(sc.grid, c)})`).join(' ')}`,
    );
  }

  // -------------------------------------------------------------------------
  // D* Lite equals a fresh A* after every change, on random grids
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(77);
    let trials = 0, mismatches = 0, changesApplied = 0, pathMismatch = 0;
    for (let k = 0; k < 40; k++) {
      const nx = 14, ny = 10;
      const start = 0, goal = nx * ny - 1;
      const grid = randomGrid(nx, ny, 0.2, rng, [start, goal]);
      const opts = { connectivity: 8 as const, diagonal: Math.SQRT2 };
      const lite = new DStarLite(grid, start, goal, opts);
      let cur = start;
      for (let step = 0; step < 25; step++) {
        // Flip a few random cells (never the robot's or the goal's) and let both planners see them.
        const changes: { cell: number; occupied: boolean }[] = [];
        for (let c = 0; c < 3; c++) {
          const cell = Math.floor(rng.uniform(0, nx * ny));
          if (cell === cur || cell === goal) continue;
          changes.push({ cell, occupied: grid.occupied[cell] === 0 });
          grid.occupied[cell] = grid.occupied[cell] ? 0 : 1;
        }
        changesApplied += changes.length;
        lite.updateCells(changes);
        trials++;
        const fresh = astar(new GridGraph(grid, opts), cur, goal, gridHeuristic(grid, goal, 'octile', Math.SQRT2));
        const liteCost = lite.costFromStart();
        if (fresh.status === 'found' ? !close(liteCost, fresh.cost, 1e-9) : Number.isFinite(liteCost)) mismatches++;
        // The greedy-descent path D* Lite follows really has that cost.
        const p = lite.pathFromStart();
        if (fresh.status === 'found' && (!p || !close(pathCost(new GridGraph(grid, opts), p), fresh.cost, 1e-9))) pathMismatch++;
        // Move one step.
        const next = lite.nextStep();
        if (next === null) break;
        cur = next;
        if (cur === goal) break;
      }
    }
    push(
      'search: D* Lite’s cost-to-goal equals a fresh A* after every map change and every robot step (40 grids, ~1000 changes)',
      trials > 300 && mismatches === 0 && pathMismatch === 0,
      `${trials} comparisons after ${changesApplied} cell flips; mismatches ${mismatches}; path-cost mismatches ${pathMismatch}`,
    );
  }

  // -------------------------------------------------------------------------
  // Universal plans (§H.4): the Bellman fixed point, on the Apartment
  // -------------------------------------------------------------------------
  {
    const grid = gridFromWorld(APARTMENT, 0.25, 0.22);
    const graph = new GridGraph(grid, { connectivity: 8, diagonal: Math.SQRT2, cornerCutting: false });
    const goal = freeCellNear(grid, 11.0, 8.0);
    const plan = universalPlanGrid(graph, goal);
    const generic = universalPlan(graph, goal);
    const residual = bellmanResidual(graph, generic);
    const rng = new Rng(3);
    let reached = 0, tried = 0, costMismatch = 0, agree = 0;
    let free = 0;
    for (let c = 0; c < grid.occupied.length; c++) if (!grid.occupied[c]) free++;
    for (let k = 0; k < 60; k++) {
      const s = freeCellNear(grid, rng.uniform(0.3, 11.7), rng.uniform(0.3, 8.7));
      if (!Number.isFinite(plan.costToGo[s])) continue;
      tried++;
      const path = followGridPlan(plan, s);
      if (path.length && path[path.length - 1] === goal) reached++;
      if (!close(pathCost(graph, path), plan.costToGo[s], 1e-9)) costMismatch++;
      // The policy's cost is Dijkstra's cost from s — the plan answers every query optimally.
      const d = dijkstra(graph, s, goal);
      if (close(d.cost, plan.costToGo[s], 1e-9)) agree++;
      if (followPlan(graph, generic, s).length !== path.length) costMismatch++;
    }
    const labelled = [...plan.costToGo].filter(Number.isFinite).length;
    push(
      'search: a universal plan on the Apartment labels every free cell, is a Bellman fixed point, and following it from 60 random cells reaches the goal at Dijkstra’s cost',
      residual < 1e-9 && tried > 40 && reached === tried && costMismatch === 0 && agree === tried && labelled === free,
      `residual ${residual.toExponential(2)}; ${reached}/${tried} reached; ${labelled}/${free} free cells labelled`,
    );
  }

  // Sanity: greedy with a zero heuristic degenerates to insertion order but still finds a path.
  {
    const t = bestFirst(micro, MICRO_START, MICRO_GOAL, ZERO_HEURISTIC, { kind: 'heuristicOnly' });
    push('search: every priority rule is complete on the micro-grid (h ≡ 0 greedy still terminates with a path)', t.status === 'found', `cost ${fmt(t.cost)}`);
  }

  return out;
}
