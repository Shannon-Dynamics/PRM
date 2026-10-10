/**
 * Planning on a cell decomposition — Choset Ch. 6 intro and §6.1, Derivation 2.
 *
 * The adjacency graph is a Chapter 6 `Graph`: a node per cell, an edge per
 * shared vertical extension, costed by the length of the hop centroid →
 * extension midpoint → centroid. Chapter 6's A* searches it with the
 * straight-line distance between centroids as heuristic (admissible by the
 * triangle inequality). The node path is then made into a path in the plane
 * by Choset's midpoint–centroid rule: q_start → midpoint of the first
 * extension → centroid of the next cell → midpoint of the next extension →
 * … → q_goal. Every piece joins two points of one *convex* cell, so it stays
 * in the cell, so the path is collision-free (Derivation 2). For the non-convex
 * cells of a boustrophedon decomposition that argument fails, which is why
 * coverage routes its transits through the trapezoids, which refine them.
 */

import { astar } from '../search/best-first';
import type { Edge, Graph } from '../search/graph';
import type { Point2 } from '../sim/world';
import { SlabLocator, type Adjacency, type Decomposition } from './sweep';

const dist = (a: Point2, b: Point2) => Math.hypot(a.x - b.x, a.y - b.y);
const mid = (a: Adjacency): Point2 => ({ x: (a.segment.a.x + a.segment.b.x) / 2, y: (a.segment.a.y + a.segment.b.y) / 2 });

/** The adjacency graph as a Chapter 6 search graph. */
export class AdjacencyGraph implements Graph<number> {
  private readonly star: Edge<number>[][];
  /** For each ordered pair (a, b), the adjacency record the cheapest edge uses. */
  private readonly via = new Map<string, Adjacency>();

  constructor(readonly d: Decomposition) {
    this.star = d.cells.map(() => []);
    for (const a of d.adjacency) {
      const m = mid(a);
      const cost = dist(d.cells[a.a].centroid, m) + dist(m, d.cells[a.b].centroid);
      const k1 = `${a.a}:${a.b}`;
      const k2 = `${a.b}:${a.a}`;
      const old = this.via.get(k1);
      if (!old || cost < dist(d.cells[a.a].centroid, mid(old)) + dist(mid(old), d.cells[a.b].centroid)) {
        this.via.set(k1, a);
        this.via.set(k2, a);
      }
      this.star[a.a].push({ to: a.b, cost });
      this.star[a.b].push({ to: a.a, cost });
    }
  }

  neighbors(n: number): readonly Edge<number>[] {
    return this.star[n];
  }

  key(n: number): number {
    return n;
  }

  order(n: number): number {
    return n;
  }

  /** The extension two adjacent cells share (the cheapest, if several). */
  extension(a: number, b: number): Adjacency | undefined {
    return this.via.get(`${a}:${b}`);
  }

  degree(n: number): number {
    return this.star[n].length;
  }
}

export interface CellPath {
  cells: number[];
  points: Point2[];
  length: number;
}

/** A* on the adjacency graph between two cells (Chapter 6). */
export function cellRoute(g: AdjacencyGraph, from: number, to: number): number[] | null {
  if (from === to) return [from];
  const goal = g.d.cells[to].centroid;
  const tr = astar(g, from, to, (n) => dist(g.d.cells[n].centroid, goal));
  return tr.status === 'found' ? tr.path : null;
}

/** Midpoint–centroid chaining of a cell route from p (in the first cell) to q (in the last). */
export function chainRoute(g: AdjacencyGraph, route: readonly number[], p: Point2, q: Point2): Point2[] {
  const pts: Point2[] = [p];
  for (let i = 0; i + 1 < route.length; i++) {
    const ext = g.extension(route[i], route[i + 1])!;
    if (i > 0) pts.push(g.d.cells[route[i]].centroid);
    pts.push(mid(ext));
  }
  pts.push(q);
  return pts;
}

/**
 * Plan from q_s to q_g on a (convex-celled) decomposition: locate both,
 * search the adjacency graph, chain midpoints and centroids. Null when either
 * point is outside Q_free or they lie in different components.
 */
export function planOnCells(d: Decomposition, qs: Point2, qg: Point2, locator = new SlabLocator(d), graph = new AdjacencyGraph(d)): CellPath | null {
  const cs = locator.locate(qs);
  const cg = locator.locate(qg);
  if (cs === null || cg === null) return null;
  const route = cellRoute(graph, cs, cg);
  if (!route) return null;
  const points = chainRoute(graph, route, qs, qg);
  let length = 0;
  for (let i = 1; i < points.length; i++) length += dist(points[i - 1], points[i]);
  return { cells: route, points, length };
}
