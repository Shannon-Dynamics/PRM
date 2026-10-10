/**
 * The Reeb graph — Choset §6.2.4 [ref. 154] — and Euler's count, §6.2.5.
 *
 * The Reeb graph is the adjacency graph's dual: its nodes are the critical
 * points, its edges are the cells, each cell joining the critical point that
 * opened it to the one that closed it. It is connected and planar, its faces
 * are the obstacles plus the outside, so Euler's v − e + f = 2 with
 * v = N_cp, e = N_ce, f = N_ob + 1 gives N_ce = N_cp + N_ob − 1.
 *
 * The same graph is built two ways in this module's callers: offline from the
 * sweep (`reebFromSweep`) and online, one node at a time, by the sensor-based
 * coverer (`online.ts`), which is the point of §6.2.4.
 *
 * Morse vocabulary comes from Chapter 8's box (`roadmap/morse.ts`): a
 * critical point of h restricted to the boundary is a `MorsePoint` with an
 * index (0 for a local minimum of h on ∂Q_free, 1 for a local maximum), and
 * the Morse count for a manifold with boundary — sum (−1)^index over the
 * boundary critical points where ∇h points *into* Q_free — must equal the
 * Euler characteristic χ(Q_free) = 1 − N_ob. That is the same fact as
 * Euler's count, seen from the other side.
 */

import { eulerCharacteristicFromCriticalPoints, type MorsePoint } from '../roadmap/morse';
import type { Point2 } from '../sim/world';
import type { Decomposition, SliceEvent } from './sweep';

export type CriticalKind = Exclude<SliceEvent, 'pass'>;

export interface ReebNode {
  id: number;
  q: Point2;
  kind: CriticalKind;
  /** Sweep event index (offline) or discovery order (online). */
  tag: number;
}

export interface ReebEdge {
  /** The cell this edge stands for. */
  cell: number;
  from: number;
  to: number;
}

export class ReebGraph {
  nodes: ReebNode[] = [];
  edges: ReebEdge[] = [];

  addNode(q: Point2, kind: CriticalKind, tag: number): ReebNode {
    const n = { id: this.nodes.length, q, kind, tag };
    this.nodes.push(n);
    return n;
  }

  addEdge(cell: number, from: number, to: number): ReebEdge {
    const e = { cell, from, to };
    this.edges.push(e);
    return e;
  }

  get nCriticalPoints(): number {
    return this.nodes.length;
  }

  get nCells(): number {
    return this.edges.length;
  }

  /** Euler's count for a boustrophedon decomposition: e = v + N_ob − 1. */
  eulerCheck(nObstacles: number): boolean {
    return this.edges.length === this.nodes.length + nObstacles - 1;
  }

  /** Is the graph connected (as an undirected multigraph)? */
  connected(): boolean {
    if (this.nodes.length === 0) return true;
    const adj = this.nodes.map(() => [] as number[]);
    for (const e of this.edges) {
      adj[e.from].push(e.to);
      adj[e.to].push(e.from);
    }
    const seen = new Set<number>([0]);
    const stack = [0];
    while (stack.length) {
      const n = stack.pop()!;
      for (const m of adj[n]) if (!seen.has(m)) {
        seen.add(m);
        stack.push(m);
      }
    }
    return seen.size === this.nodes.length;
  }

  degree(id: number): number {
    return this.edges.filter((e) => e.from === id || e.to === id).length;
  }
}

/** The Reeb graph of a boustrophedon sweep: one node per critical event, one edge per cell. */
export function reebFromSweep(d: Decomposition): ReebGraph {
  const g = new ReebGraph();
  const nodeOf = new Map<number, number>();
  for (const ev of d.events) {
    if (ev.slice === 'pass') continue;
    nodeOf.set(ev.index, g.addNode(ev.v, ev.slice, ev.index).id);
  }
  for (const c of d.cells) {
    const a = nodeOf.get(c.openEvent);
    const b = nodeOf.get(c.closeEvent);
    if (a === undefined || b === undefined) {
      throw new Error('reebFromSweep: a cell opens or closes at a non-critical event — not a boustrophedon decomposition');
    }
    g.addEdge(c.id, a, b);
  }
  return g;
}

/**
 * The boundary critical points of h(x, y) = x as Chapter 8 `MorsePoint`s, with
 * a flag for whether ∇h = (1, 0) points into Q_free there. Births (the
 * boundary's leftmost points) and merges (a hole's rightmost points) have the
 * gradient pointing inward; splits and deaths have it pointing out.
 */
export function boundaryMorsePoints(d: Decomposition): { point: MorsePoint; inward: boolean; kind: CriticalKind }[] {
  return d.events
    .filter((e) => e.slice !== 'pass')
    .map((e) => {
      const kind = e.slice as CriticalKind;
      const isMin = kind === 'birth' || kind === 'split';
      const point: MorsePoint = {
        q: [e.v.x, e.v.y],
        value: e.v.x,
        eigs: [],
        index: isMin ? 0 : 1,
        type: isMin ? 'minimum' : 'maximum',
        nondegenerate: true,
      };
      return { point, inward: kind === 'birth' || kind === 'merge', kind };
    });
}

/** χ(Q_free) from the Morse count with boundary: Σ (−1)^index over inward-gradient boundary critical points. */
export function morseEulerCharacteristic(d: Decomposition): number {
  return eulerCharacteristicFromCriticalPoints(boundaryMorsePoints(d).filter((m) => m.inward).map((m) => m.point));
}
