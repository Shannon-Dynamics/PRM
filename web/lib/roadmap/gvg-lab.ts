/**
 * Chapter 9's integration lab: the roadmap Rusty drew with its range ring,
 * used as a roadmap. Get on by Chapter 8's gradient ascent (the same access
 * the explorer used), ride by Chapter 6's A* on the explored graph, get off by
 * the goal's ascent reversed — and compare the length with the Euclidean
 * shortest path for disc-Rusty from Chapter 8's visibility graph.
 *
 * The explored edges are polylines sampled every α·dt along the drive. A
 * gradient-ascent landing point lies on the GVD but generally between two
 * samples of some edge, so the roadmap here is the *chain* graph: every
 * sample is a node and consecutive samples are joined. That costs a few
 * thousand nodes for the Apartment, which A* does not notice, and it lets
 * "get on" be a hop of a centimetre instead of a split of an edge.
 */

import { Rng } from '../prob/rng';
import type { Point2 } from '../sim/world';
import { apartmentDistanceQuery, apartmentVisibility } from './examples';
import type { GvgExplorer } from './explore';
import { retract } from './retract';
import { Roadmap, minClearance, polylineLength } from './roadmap';

/** The explored GVG as a chain graph: one node per polyline sample, meet points shared between edges. */
export function explorerChainRoadmap(ex: GvgExplorer, mergeTol = 1e-6): Roadmap {
  const rm = new Roadmap();
  const nodeOf = new Map<number, number>();
  for (const n of ex.nodes) nodeOf.set(n.id, rm.addNode({ x: n.q[0], y: n.q[1] }));
  for (const e of ex.edges) {
    if (e.to < 0 || e.polyline.length < 2) continue;
    let prev = nodeOf.get(e.from)!;
    const last = e.polyline.length - 1;
    for (let k = 1; k < last; k++) {
      const p = e.polyline[k];
      const prevP = rm.nodes[prev];
      if (Math.hypot(p[0] - prevP.x, p[1] - prevP.y) < mergeTol) continue;
      const id = rm.addNode({ x: p[0], y: p[1] });
      rm.addEdge(prev, id);
      prev = id;
    }
    rm.addEdge(prev, nodeOf.get(e.to)!);
  }
  return rm;
}

export interface LabQuery {
  start: Point2;
  goal: Point2;
  gvgLength: number;
  shortestLength: number;
  gvgClearance: number;
  shortestClearance: number;
  gvgPath: Point2[];
}

export interface LabResult {
  tried: number;
  answered: number;
  queries: LabQuery[];
  meanGvg: number;
  meanShortest: number;
  /** Mean and worst of (GVG length) / (shortest length). */
  meanRatio: number;
  worstRatio: number;
  meanGvgClearance: number;
  meanShortestClearance: number;
  /** Largest hop from an ascent landing point to the nearest chain node — how well the explored graph covers the GVD. */
  worstHop: number;
  components: number;
}

/**
 * Fifty seeded Apartment queries (the same generator and filter as Chapter
 * 8's lab: both ends at least 0.3 m from a wall and 2 m apart) answered on
 * the explored GVG and on the visibility graph for a 0.22 m disc.
 */
export function gvgVersusShortest(ex: GvgExplorer, n = 50, seed = 85, radius = 0.22): LabResult {
  const rm = explorerChainRoadmap(ex);
  const dq = apartmentDistanceQuery();
  const D = (p: Point2) => dq.nearest([p.x, p.y]).d;
  const rng = new Rng(seed);
  const queries: LabQuery[] = [];
  let tried = 0;
  let worstHop = 0;
  const access = (p: Point2) => {
    const r = retract(dq, p, { step: 0.02 });
    if (!r.landed) return null;
    const land = r.path[r.path.length - 1];
    const node = rm.nearestNode(land);
    const hop = Math.hypot(rm.nodes[node].x - land.x, rm.nodes[node].y - land.y);
    worstHop = Math.max(worstHop, hop);
    return { node, path: [...r.path, rm.nodes[node]] };
  };
  for (let k = 0; k < n; k++) {
    const s = { x: rng.uniform(0.4, 11.6), y: rng.uniform(0.4, 8.6) };
    const g = { x: rng.uniform(0.4, 11.6), y: rng.uniform(0.4, 8.6) };
    if (D(s) < 0.3 || D(g) < 0.3 || Math.hypot(s.x - g.x, s.y - g.y) < 2) continue;
    tried++;
    const on = access(s);
    const off = access(g);
    if (!on || !off) continue;
    const ride = rm.shortestPath(on.node, off.node);
    const vs = apartmentVisibility(s, g, radius);
    const sId = vs.nodes.find((x) => x.name === 'start')!.id;
    const gId = vs.nodes.find((x) => x.name === 'goal')!.id;
    const vp = vs.graph().shortestPath(sId, gId);
    if (!ride || !vp) continue;
    const gvgPath = [...on.path, ...ride.points.slice(1), ...off.path.slice().reverse().slice(1)];
    queries.push({
      start: s,
      goal: g,
      gvgLength: polylineLength(gvgPath),
      shortestLength: vp.length,
      gvgClearance: minClearance(gvgPath, D, 2),
      shortestClearance: minClearance(vp.points, D),
      gvgPath,
    });
  }
  const m = (f: (q: LabQuery) => number) => queries.reduce((a, q) => a + f(q), 0) / Math.max(1, queries.length);
  return {
    tried,
    answered: queries.length,
    queries,
    meanGvg: m((q) => q.gvgLength),
    meanShortest: m((q) => q.shortestLength),
    meanRatio: m((q) => q.gvgLength / q.shortestLength),
    worstRatio: Math.max(...queries.map((q) => q.gvgLength / q.shortestLength)),
    meanGvgClearance: m((q) => q.gvgClearance),
    meanShortestClearance: m((q) => q.shortestClearance),
    worstHop,
    components: rm.components().count,
  };
}
