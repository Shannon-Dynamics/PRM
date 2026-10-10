/**
 * Nearest neighbors over a `Chart` — the data structure behind N_q.
 *
 * Choset §7.1.2 recommends the kd-tree of de Berg, van Kreveld & Overmars
 * [C ref. 124]: O(dn log n) to build, O(n^{1−1/d} + m) per range query. The
 * one thing a textbook kd-tree cannot do is a torus. Reach's configuration
 * space wraps, and a nearest neighbor across the seam at θ = ±π is a real
 * neighbor. So this tree splits on *chart coordinates* but measures with the
 * *manifold metric*, and its pruning bound knows which axes are periodic: the
 * distance from a query to a box along a periodic axis is the circular
 * distance to the nearer box edge, min(|c − s|, T − |c − s|).
 *
 * The bound must never exceed the true metric, or pruning would discard real
 * neighbors. For ℝᵈ and Tⁿ (ℓ² of per-axis distances) the coordinate gaps are
 * exact lower bounds; for SE(2) with weight w the θ-axis gap must be scaled by
 * w, and for a `Product` by its exchange rate. `axisWeights` derives those
 * scales from the chart; anything it does not recognize gets weight 1 on
 * non-periodic axes and is assumed ℓ²-compatible.
 *
 * Insertion is incremental (depth-cycled split axis), which is what every
 * planner in Part III needs: a roadmap or tree grows one node at a time.
 * `LinearNn` is the brute-force oracle the checks compare it against.
 */

import type { Chart } from '../manifold/manifold';
import { Product } from '../manifold/manifold';
import { SE2 } from '../manifold/se2';

export interface Neighbor {
  idx: number;
  dist: number;
}

export interface NearestNeighbors<P> {
  readonly size: number;
  insert(idx: number, q: P): void;
  nearest(q: P, exclude?: number): Neighbor | null;
  /** The k closest, sorted by distance. */
  kNearest(q: P, k: number, exclude?: number): Neighbor[];
  /** Everything within r (inclusive), sorted by distance — Chapter 13's Q_near. */
  within(q: P, r: number, exclude?: number): Neighbor[];
}

/** Per-axis scale factors such that dist(a, b) ≥ √Σ (wᵢ · gapᵢ)². */
export function axisWeights<P>(space: Chart<P>): number[] {
  if (space instanceof SE2) return [1, 1, space.w];
  if (space instanceof Product) {
    const a = axisWeights(space.first as Chart<unknown>);
    const b = axisWeights(space.second as Chart<unknown>).map((x) => x * space.weight);
    return [...a, ...b];
  }
  return new Array<number>(space.dim).fill(1);
}

export interface KdBox {
  idx: number;
  depth: number;
  axis: number;
  split: number;
  /** Chart coordinates of the point stored at this node. */
  c: number[];
  lo: number[];
  hi: number[];
  parent: number;
}

interface KdNode {
  idx: number;
  c: number[];
  axis: number;
  left: KdNode | null;
  right: KdNode | null;
}

export class KdTree<P> implements NearestNeighbors<P> {
  private root: KdNode | null = null;
  private readonly pts = new Map<number, P>();
  private readonly weights: number[];
  private readonly D: number;
  size = 0;
  /** Nodes visited by the last query — the widget's "work" readout and the check's pruning evidence. */
  visited = 0;

  constructor(readonly space: Chart<P>) {
    this.D = space.dim;
    this.weights = axisWeights(space);
  }

  insert(idx: number, q: P): void {
    const c = this.space.coords(q);
    this.pts.set(idx, q);
    this.size++;
    const node: KdNode = { idx, c, axis: 0, left: null, right: null };
    if (!this.root) {
      this.root = node;
      return;
    }
    let cur = this.root;
    let depth = 0;
    for (;;) {
      const a = cur.axis;
      const goLeft = c[a] < cur.c[a];
      const next = goLeft ? cur.left : cur.right;
      depth++;
      if (!next) {
        node.axis = depth % this.D;
        if (goLeft) cur.left = node;
        else cur.right = node;
        return;
      }
      cur = next;
    }
  }

  /**
   * Every node's splitting box — the cells of figure f11.3. Non-periodic root
   * extents are ±∞; the figure clamps them to the chart's box.
   */
  boxes(): KdBox[] {
    const out: KdBox[] = [];
    const walk = (node: KdNode | null, lo: number[], hi: number[], depth: number, parent: number) => {
      if (!node) return;
      out.push({ idx: node.idx, depth, axis: node.axis, split: node.c[node.axis], c: node.c.slice(), lo: lo.slice(), hi: hi.slice(), parent });
      const a = node.axis;
      const lhi = hi.slice();
      lhi[a] = node.c[a];
      const rlo = lo.slice();
      rlo[a] = node.c[a];
      walk(node.left, lo, lhi, depth + 1, node.idx);
      walk(node.right, rlo, hi, depth + 1, node.idx);
    };
    walk(this.root, this.rootLo(), this.rootHi(), 0, -1);
    return out;
  }

  point(idx: number): P {
    const p = this.pts.get(idx);
    if (p === undefined) throw new Error(`KdTree: no point with index ${idx}`);
    return p;
  }

  nearest(q: P, exclude?: number): Neighbor | null {
    const r = this.kNearest(q, 1, exclude);
    return r.length ? r[0] : null;
  }

  kNearest(q: P, k: number, exclude?: number): Neighbor[] {
    if (!this.root || k <= 0) return [];
    const qc = this.space.coords(q);
    const best: Neighbor[] = [];
    this.visited = 0;
    const lo = this.rootLo();
    const hi = this.rootHi();
    const worst = () => (best.length < k ? Infinity : best[best.length - 1].dist);
    const consider = (idx: number, d: number) => {
      if (best.length < k || d < worst()) {
        let i = best.length;
        best.push({ idx, dist: d });
        while (i > 0 && best[i - 1].dist > d) {
          best[i] = best[i - 1];
          i--;
        }
        best[i] = { idx, dist: d };
        if (best.length > k) best.pop();
      }
    };
    this.search(this.root, q, qc, lo, hi, exclude, worst, consider);
    return best;
  }

  within(q: P, r: number, exclude?: number): Neighbor[] {
    if (!this.root) return [];
    const qc = this.space.coords(q);
    const out: Neighbor[] = [];
    this.visited = 0;
    this.search(
      this.root,
      q,
      qc,
      this.rootLo(),
      this.rootHi(),
      exclude,
      () => r,
      (idx, d) => {
        if (d <= r) out.push({ idx, dist: d });
      },
    );
    out.sort((a, b) => a.dist - b.dist || a.idx - b.idx);
    return out;
  }

  private rootLo(): number[] {
    return Array.from({ length: this.D }, (_, i) => (this.space.periodic[i] ? -this.space.period / 2 : -Infinity));
  }

  private rootHi(): number[] {
    return Array.from({ length: this.D }, (_, i) => (this.space.periodic[i] ? this.space.period / 2 : Infinity));
  }

  /** √Σ (wᵢ gapᵢ)² — the distance from q to the box, never more than the true metric. */
  lowerBound(qc: number[], lo: number[], hi: number[]): number {
    let s = 0;
    for (let i = 0; i < this.D; i++) {
      const c = qc[i];
      let gap = 0;
      if (c < lo[i] || c > hi[i]) {
        if (this.space.periodic[i]) {
          const T = this.space.period;
          const dl = Math.abs(c - lo[i]);
          const dh = Math.abs(c - hi[i]);
          gap = Math.min(dl, T - dl, dh, T - dh);
        } else {
          gap = c < lo[i] ? lo[i] - c : c - hi[i];
        }
      }
      const g = gap * this.weights[i];
      s += g * g;
    }
    return Math.sqrt(s);
  }

  private search(
    node: KdNode,
    q: P,
    qc: number[],
    lo: number[],
    hi: number[],
    exclude: number | undefined,
    bound: () => number,
    consider: (idx: number, d: number) => void,
  ): void {
    this.visited++;
    if (node.idx !== exclude) consider(node.idx, this.space.dist(q, this.pts.get(node.idx)!));
    const a = node.axis;
    const s = node.c[a];
    const nearLeft = qc[a] < s;
    const first = nearLeft ? node.left : node.right;
    const second = nearLeft ? node.right : node.left;
    if (first) {
      const nlo = lo.slice();
      const nhi = hi.slice();
      if (nearLeft) nhi[a] = s;
      else nlo[a] = s;
      this.search(first, q, qc, nlo, nhi, exclude, bound, consider);
    }
    if (second) {
      const nlo = lo.slice();
      const nhi = hi.slice();
      if (nearLeft) nlo[a] = s;
      else nhi[a] = s;
      // Prune: nothing in the far box can beat the current bound.
      if (this.lowerBound(qc, nlo, nhi) <= bound()) {
        this.search(second, q, qc, nlo, nhi, exclude, bound, consider);
      }
    }
  }
}

/** The oracle: every distance computed, no pruning. */
export class LinearNn<P> implements NearestNeighbors<P> {
  private readonly pts: { idx: number; q: P }[] = [];
  size = 0;

  constructor(readonly space: Chart<P>) {}

  insert(idx: number, q: P): void {
    this.pts.push({ idx, q });
    this.size++;
  }

  nearest(q: P, exclude?: number): Neighbor | null {
    const r = this.kNearest(q, 1, exclude);
    return r.length ? r[0] : null;
  }

  kNearest(q: P, k: number, exclude?: number): Neighbor[] {
    return this.all(q, exclude)
      .sort((a, b) => a.dist - b.dist || a.idx - b.idx)
      .slice(0, k);
  }

  within(q: P, r: number, exclude?: number): Neighbor[] {
    return this.all(q, exclude)
      .filter((n) => n.dist <= r)
      .sort((a, b) => a.dist - b.dist || a.idx - b.idx);
  }

  private all(q: P, exclude?: number): Neighbor[] {
    const out: Neighbor[] = [];
    for (const p of this.pts) if (p.idx !== exclude) out.push({ idx: p.idx, dist: this.space.dist(q, p.q) });
    return out;
  }
}

export function makeNn<P>(space: Chart<P>, kind: 'kd' | 'linear' = 'kd'): NearestNeighbors<P> {
  return kind === 'kd' ? new KdTree(space) : new LinearNn(space);
}
