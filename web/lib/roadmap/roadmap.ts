/**
 * The roadmap itself — Choset Def. 5.0.2 as a data structure.
 *
 * A roadmap is a union of one-dimensional curves embedded in Q_free, used the
 * way people use a highway system: get on, ride, get off. Here it is a graph
 * whose nodes are points of the plane and whose edges carry a sampled polyline
 * (a straight segment for the visibility graph, a traced curve for the GVD)
 * and its length as the cost. It implements Chapter 6's `Graph<number>`, so
 * "ride" is A* from `search/best-first` with the Euclidean heuristic and
 * nothing is re-implemented.
 *
 * The three roadmap properties are not fields of this class; they are
 * theorems about how the nodes and edges were built (visibility.ts,
 * retract.ts, gvd.ts) and the checks assert them.
 */

import { bestFirst, PRIORITY_ASTAR } from '../search/best-first';
import type { Edge, Graph } from '../search/graph';
import type { Point2 } from '../sim/world';

export interface RoadmapPath {
  /** Node ids from start to goal. */
  nodes: number[];
  /** The embedded curve: every polyline sample along the way. */
  points: Point2[];
  length: number;
  /** A* expansions — the cost of the search, for the dashboards. */
  expansions: number;
}

export function polylineLength(pts: readonly Point2[]): number {
  let L = 0;
  for (let i = 1; i < pts.length; i++) L += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  return L;
}

export class Roadmap implements Graph<number> {
  readonly nodes: Point2[] = [];
  private readonly adj: Edge<number>[][] = [];
  private readonly polylines = new Map<string, Point2[]>();

  private static edgeKey(a: number, b: number): string {
    return a < b ? `${a}-${b}` : `${b}-${a}`;
  }

  addNode(p: Point2): number {
    this.nodes.push({ x: p.x, y: p.y });
    this.adj.push([]);
    return this.nodes.length - 1;
  }

  /** An undirected edge; the cost is the polyline's length (the chord when none is given). */
  addEdge(a: number, b: number, polyline?: Point2[]): void {
    if (a === b) return;
    const key = Roadmap.edgeKey(a, b);
    if (this.polylines.has(key)) return;
    const pts = polyline ?? [this.nodes[a], this.nodes[b]];
    // Store the polyline oriented from the smaller id so `edgePolyline` can flip it.
    const oriented = a < b ? pts : [...pts].reverse();
    this.polylines.set(key, oriented);
    const cost = polylineLength(pts);
    this.adj[a].push({ to: b, cost });
    this.adj[b].push({ to: a, cost });
  }

  hasEdge(a: number, b: number): boolean {
    return this.polylines.has(Roadmap.edgeKey(a, b));
  }

  /** The polyline from a to b, in that direction. */
  edgePolyline(a: number, b: number): Point2[] {
    const pts = this.polylines.get(Roadmap.edgeKey(a, b)) ?? [this.nodes[a], this.nodes[b]];
    return a < b ? pts : [...pts].reverse();
  }

  edges(): { a: number; b: number; cost: number; points: Point2[] }[] {
    const out: { a: number; b: number; cost: number; points: Point2[] }[] = [];
    for (const [key, pts] of this.polylines) {
      const [a, b] = key.split('-').map(Number);
      out.push({ a, b, cost: polylineLength(pts), points: pts });
    }
    return out;
  }

  get edgeCount(): number {
    return this.polylines.size;
  }

  neighbors(n: number): readonly Edge<number>[] {
    return this.adj[n] ?? [];
  }

  key(n: number): number {
    return n;
  }

  degree(n: number): number {
    return this.adj[n]?.length ?? 0;
  }

  nearestNode(p: Point2): number {
    let best = -1;
    let bd = Infinity;
    this.nodes.forEach((q, i) => {
      const d = Math.hypot(q.x - p.x, q.y - p.y);
      if (d < bd) {
        bd = d;
        best = i;
      }
    });
    return best;
  }

  /** Connected components by node (−1 for isolated nodes is not used: every node gets a label). */
  components(): { count: number; label: Int32Array } {
    const label = new Int32Array(this.nodes.length).fill(-1);
    let count = 0;
    for (let s = 0; s < this.nodes.length; s++) {
      if (label[s] >= 0) continue;
      const stack = [s];
      label[s] = count;
      while (stack.length) {
        const n = stack.pop()!;
        for (const { to } of this.adj[n]) {
          if (label[to] < 0) {
            label[to] = count;
            stack.push(to);
          }
        }
      }
      count++;
    }
    return { count, label };
  }

  /** "Ride": A* on the roadmap with the Euclidean heuristic — Chapter 6's engine, unchanged. */
  shortestPath(a: number, b: number): RoadmapPath | null {
    const goal = this.nodes[b];
    const t = bestFirst(this, a, b, (n) => Math.hypot(this.nodes[n].x - goal.x, this.nodes[n].y - goal.y), PRIORITY_ASTAR);
    if (t.status !== 'found') return null;
    const points: Point2[] = [];
    for (let i = 0; i < t.path.length; i++) {
      if (i === 0) points.push(this.nodes[t.path[0]]);
      else {
        const pl = this.edgePolyline(t.path[i - 1], t.path[i]);
        for (let k = 1; k < pl.length; k++) points.push(pl[k]);
      }
    }
    return { nodes: t.path, points, length: t.cost, expansions: t.expansions };
  }
}

/** Minimum clearance of a sampled path under a distance function — the dashboards' second column. */
export function minClearance(points: readonly Point2[], D: (p: Point2) => number, samplesPerSegment = 8): number {
  let m = Infinity;
  for (let i = 0; i < points.length; i++) {
    m = Math.min(m, D(points[i]));
    if (i + 1 < points.length) {
      for (let k = 1; k < samplesPerSegment; k++) {
        const t = k / samplesPerSegment;
        m = Math.min(m, D({ x: points[i].x + t * (points[i + 1].x - points[i].x), y: points[i].y + t * (points[i + 1].y - points[i].y) }));
      }
    }
  }
  return m;
}
