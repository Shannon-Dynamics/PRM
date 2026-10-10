/**
 * The visibility graph — Choset §5.1.
 *
 * Nodes are the start, the goal and every vertex of the polygonal C-obstacles;
 * two nodes share an edge iff the segment between them lies in cl(Q_free)
 * (§5.1.1). Two constructions of the same graph live here, and the chapter's
 * check insists they agree on seeded worlds:
 *
 *   brute force      O(n³): every candidate segment against every edge.
 *   rotational sweep Choset Alg. 5, O(n² log n) in the text's accounting: a
 *                    half-line turns about v, pausing at every vertex angle,
 *                    and keeps the set S of edges it crosses sorted by
 *                    distance; v_i is visible iff v·v_i reaches it before the
 *                    nearest crossed edge and the sweep line does not enter
 *                    the obstacle at v. The port keeps S as a sorted array
 *                    rather than a balanced tree — O(n) per update instead of
 *                    O(log n), which the prose says honestly; the *point* of
 *                    the algorithm, that only the nearest crossed edge can
 *                    occlude, is intact.
 *
 * "Visible" is defined once, by `VisibilityScene.visible`, with the same
 * primitives both constructions use: a segment is blocked by a polygon edge
 * not incident to either endpoint iff the two *properly* cross, and a
 * segment whose midpoint lies strictly inside a polygon is blocked (that is
 * what catches a chord of a convex obstacle). General position — no three
 * vertices collinear — is assumed, as Choset assumes it; the seeded checks
 * satisfy it almost surely and Fig. 5.8's rectangles were placed to.
 *
 * The *reduced* visibility graph keeps only edges on supporting or separating
 * lines (§5.1.1): at a polygon vertex the edge's line must be tangent in the
 * nonsmooth sense — both neighbouring vertices on one side — which at a
 * reflex vertex of a nonconvex obstacle never holds (local convexity).
 */

import { polygonEdges, type Polygon } from '../collide/polygon';
import { distanceToSegment, type Point2, type Segment } from '../sim/world';
import { Roadmap } from './roadmap';

export interface VisNode {
  id: number;
  p: Point2;
  /** Polygon index, or −1 for a free node (start, goal, the lighthouse v). */
  poly: number;
  /** Vertex index within its polygon (−1 for free nodes). */
  vertex: number;
  /** Not strictly inside any *other* polygon; only free nodes join the graph. */
  free: boolean;
  /** Choset-style name: v1, v2, … for vertices; the caller's for extras. */
  name: string;
}

export interface VisEdge {
  id: number;
  poly: number;
  /** Edge k of its polygon joins vertex k to vertex k + 1. */
  k: number;
  a: number;
  b: number;
  seg: Segment;
  name: string;
}

const EPS = 1e-12;

/** Orientation sign of (a → b → c): +1 left turn, −1 right turn, 0 collinear (within EPS). */
function orient(a: Point2, b: Point2, c: Point2): number {
  const v = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return Math.abs(v) < EPS ? 0 : v > 0 ? 1 : -1;
}

/** Proper crossing: the segments intersect at a single interior point of both. */
export function properCross(p: Point2, q: Point2, s: Segment): boolean {
  const a = { x: s.x1, y: s.y1 };
  const b = { x: s.x2, y: s.y2 };
  const o1 = orient(p, q, a);
  const o2 = orient(p, q, b);
  const o3 = orient(a, b, p);
  const o4 = orient(a, b, q);
  return o1 !== 0 && o2 !== 0 && o3 !== 0 && o4 !== 0 && o1 !== o2 && o3 !== o4;
}

/** Inside and not on the boundary. */
export function strictlyInside(p: Point2, poly: Polygon, tol = 1e-9): boolean {
  // Even–odd rule, then exclude the boundary.
  let inside = false;
  const n = poly.length;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const a = poly[i];
    const b = poly[j];
    if (a.y > p.y !== b.y > p.y) {
      const xAt = ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x;
      if (p.x < xAt) inside = !inside;
    }
  }
  if (!inside) return false;
  for (const e of polygonEdges(poly)) if (distanceToSegment(p.x, p.y, e) <= tol) return false;
  return true;
}

export interface SweepEvent {
  /** The vertex the sweep line paused at. */
  node: number;
  angle: number;
  visible: boolean;
  /** Edge ids inserted into / deleted from S at this event. */
  inserted: number[];
  deleted: number[];
  /** S after the event, nearest first. */
  S: number[];
}

export interface SweepResult {
  v: number;
  visible: number[];
  initialS: number[];
  events: SweepEvent[];
}

export class VisibilityScene {
  readonly nodes: VisNode[] = [];
  readonly edges: VisEdge[] = [];
  readonly polys: Polygon[];
  private readonly incident: number[][] = [];
  private readonly scale: number;

  constructor(polys: Polygon[], extra: { p: Point2; name: string }[] = []) {
    this.polys = polys;
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    polys.forEach((poly, pi) => {
      const base = this.nodes.length;
      poly.forEach((p, vi) => {
        this.nodes.push({ id: base + vi, p, poly: pi, vertex: vi, free: true, name: `v${base + vi + 1}` });
        this.incident.push([]);
        minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x); minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
      });
      poly.forEach((p, vi) => {
        const q = poly[(vi + 1) % poly.length];
        const id = this.edges.length;
        const a = base + vi;
        const b = base + ((vi + 1) % poly.length);
        this.edges.push({ id, poly: pi, k: vi, a, b, seg: { x1: p.x, y1: p.y, x2: q.x, y2: q.y }, name: `E${id + 1}` });
        this.incident[a].push(id);
        this.incident[b].push(id);
      });
    });
    for (const e of extra) {
      this.nodes.push({ id: this.nodes.length, p: e.p, poly: -1, vertex: -1, free: true, name: e.name });
      this.incident.push([]);
      minX = Math.min(minX, e.p.x); maxX = Math.max(maxX, e.p.x); minY = Math.min(minY, e.p.y); maxY = Math.max(maxY, e.p.y);
    }
    this.scale = Math.max(maxX - minX, maxY - minY, 1e-6);
    // A vertex strictly inside another polygon (overlapping C-obstacles) is not in cl(Q_free).
    for (const n of this.nodes) {
      for (let pi = 0; pi < polys.length; pi++) {
        if (pi === n.poly) continue;
        if (strictlyInside(n.p, polys[pi])) {
          n.free = false;
          break;
        }
      }
    }
  }

  incidentEdges(node: number): readonly number[] {
    return this.incident[node];
  }

  /** The neighbours of a polygon vertex along its polygon (prev, next); null for free nodes. */
  polygonNeighbours(node: number): [Point2, Point2] | null {
    const n = this.nodes[node];
    if (n.poly < 0) return null;
    const poly = this.polys[n.poly];
    const m = poly.length;
    return [poly[(n.vertex - 1 + m) % m], poly[(n.vertex + 1) % m]];
  }

  /** Does edge e block the segment a→b? Edges incident to a or b never do. */
  private blocks(e: VisEdge, a: number, b: number): boolean {
    if (e.a === a || e.b === a || e.a === b || e.b === b) return false;
    return properCross(this.nodes[a].p, this.nodes[b].p, e.seg);
  }

  /** Does the direction `dir` from polygon vertex `node` point into its own polygon's interior? */
  entersObstacleAt(node: number, dir: Point2): boolean {
    const n = this.nodes[node];
    if (n.poly < 0) return false;
    const eps = 1e-6 * this.scale;
    const L = Math.hypot(dir.x, dir.y) || 1;
    return strictlyInside({ x: n.p.x + (eps * dir.x) / L, y: n.p.y + (eps * dir.y) / L }, this.polys[n.poly], 1e-3 * eps);
  }

  /** Midpoint strictly inside any polygon? (Catches chords of a convex obstacle.) */
  private midpointBuried(a: number, b: number): boolean {
    const p = this.nodes[a].p;
    const q = this.nodes[b].p;
    const m = { x: 0.5 * (p.x + q.x), y: 0.5 * (p.y + q.y) };
    for (const poly of this.polys) if (strictlyInside(m, poly)) return true;
    return false;
  }

  /** The definition (§5.1.1): s·v_a + (1 − s)·v_b ∈ cl(Q_free) for all s. Brute force, O(#edges). */
  visible(a: number, b: number): boolean {
    if (a === b) return false;
    if (!this.nodes[a].free || !this.nodes[b].free) return false;
    for (const e of this.edges) if (this.blocks(e, a, b)) return false;
    return !this.midpointBuried(a, b);
  }

  /** All nodes visible from v, by the definition — the O(n²)-per-vertex reference. */
  visibleBruteForce(v: number): number[] {
    const out: number[] = [];
    for (const n of this.nodes) if (n.id !== v && this.visible(v, n.id)) out.push(n.id);
    return out;
  }

  /** Distance from `from` along the ray at `angle` to edge e, or +∞ if the ray misses it. */
  private rayDistance(from: Point2, angle: number, e: VisEdge): number {
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);
    const sx = e.seg.x2 - e.seg.x1;
    const sy = e.seg.y2 - e.seg.y1;
    const denom = dx * sy - dy * sx;
    if (Math.abs(denom) < EPS) return Infinity;
    const ex = e.seg.x1 - from.x;
    const ey = e.seg.y1 - from.y;
    const t = (ex * sy - ey * sx) / denom;
    const u = (ex * dy - ey * dx) / denom;
    if (t < -1e-9 || u < -1e-9 || u > 1 + 1e-9) return Infinity;
    return Math.max(0, t);
  }

  /**
   * Choset Algorithm 5 for one vertex. Returns the visible set and the full
   * event log — Table 5.1 is `events` printed.
   */
  rotationalSweep(v: number): SweepResult {
    const vp = this.nodes[v].p;
    const vIncident = new Set(this.incident[v]);
    // Step 1–2: angles α_i and the sorted vertex list E (general position: distinct angles).
    const events = this.nodes
      .filter((n) => n.id !== v && n.free)
      .map((n) => {
        let a = Math.atan2(n.p.y - vp.y, n.p.x - vp.x);
        if (a < 0) a += 2 * Math.PI;
        return { node: n.id, angle: a, dist: Math.hypot(n.p.x - vp.x, n.p.y - vp.y) };
      })
      .sort((p, q) => p.angle - q.angle || p.dist - q.dist);
    // Step 3: S = edges crossing the horizontal half-line from v, nearest first.
    const S: number[] = [];
    const dist0 = new Map<number, number>();
    for (const e of this.edges) {
      if (vIncident.has(e.id)) continue;
      const ya = e.seg.y1 - vp.y;
      const yb = e.seg.y2 - vp.y;
      if (!(ya > 0 !== yb > 0)) continue; // must straddle the half-line's line…
      const t = this.rayDistance(vp, 0, e);
      if (!Number.isFinite(t) || t <= 1e-12) continue; // …on the +x side
      dist0.set(e.id, t);
      S.push(e.id);
    }
    S.sort((a, b) => dist0.get(a)! - dist0.get(b)!);
    const initialS = S.slice();
    const visible: number[] = [];
    const log: SweepEvent[] = [];
    for (const ev of events) {
      const n = this.nodes[ev.node];
      const nIncident = new Set(this.incident[ev.node]);
      // Visibility: nearest crossed edge not incident to v or v_i must lie beyond v_i,
      // and the sweep line must not enter the obstacle at v (nor the chord be buried).
      let nearest = Infinity;
      for (const id of S) {
        if (nIncident.has(id)) continue;
        nearest = Math.min(nearest, this.rayDistance(vp, ev.angle, this.edges[id]));
      }
      const dir = { x: n.p.x - vp.x, y: n.p.y - vp.y };
      const vis = ev.dist < nearest - 1e-9 && !this.entersObstacleAt(v, dir) && !this.midpointBuried(v, ev.node);
      if (vis) visible.push(ev.node);
      // Steps 8–13: an incident edge already in S ends here (delete); otherwise it begins (insert).
      const inserted: number[] = [];
      const deleted: number[] = [];
      for (const id of this.incident[ev.node]) {
        if (vIncident.has(id)) continue;
        const at = S.indexOf(id);
        if (at >= 0) {
          S.splice(at, 1);
          deleted.push(id);
        } else {
          // Insert by distance along the ray just *past* this angle: two edges that begin at the
          // same vertex tie exactly at it, and the one the sweep meets first thereafter is nearer.
          // Edges in S never cross, so the order persists until one of them is deleted.
          const ahead = ev.angle + 1e-7;
          const mine = this.rayDistance(vp, ahead, this.edges[id]);
          let pos = 0;
          while (pos < S.length && this.rayDistance(vp, ahead, this.edges[S[pos]]) < mine) pos++;
          S.splice(pos, 0, id);
          inserted.push(id);
        }
      }
      log.push({ node: ev.node, angle: ev.angle, visible: vis, inserted, deleted, S: S.slice() });
    }
    return { v, visible, initialS, events: log };
  }

  /**
   * Is the line through node `a` toward `b` tangent at `a` in the nonsmooth
   * sense — both polygon neighbours of `a` on one side (or on the line)? Free
   * nodes impose nothing. A supporting or separating line is tangent at both
   * ends, so an edge survives the reduction iff this holds at both.
   */
  tangentAt(a: number, b: number): boolean {
    const nb = this.polygonNeighbours(a);
    if (!nb) return true;
    const p = this.nodes[a].p;
    const q = this.nodes[b].p;
    const s1 = orient(p, q, nb[0]);
    const s2 = orient(p, q, nb[1]);
    return s1 === 0 || s2 === 0 || s1 === s2;
  }

  /**
   * The visibility graph as a `Roadmap`, built by the sweep (or brute force);
   * `reduced` keeps only supporting/separating edges. Node ids coincide with
   * `VisNode.id`, so a start or goal passed as an extra node is addressable.
   */
  graph(opts: { method?: 'sweep' | 'brute'; reduced?: boolean } = {}): Roadmap {
    const { method = 'sweep', reduced = false } = opts;
    const rm = new Roadmap();
    for (const n of this.nodes) rm.addNode(n.p);
    for (const n of this.nodes) {
      if (!n.free) continue;
      const vis = method === 'sweep' ? this.rotationalSweep(n.id).visible : this.visibleBruteForce(n.id);
      for (const m of vis) {
        if (m < n.id) continue;
        if (reduced && !(this.tangentAt(n.id, m) && this.tangentAt(m, n.id))) continue;
        rm.addEdge(n.id, m);
      }
    }
    return rm;
  }
}

/** Names of a list of edge ids, Choset style: "{E4, E2, E8, E6}". */
export function edgeSetName(scene: VisibilityScene, ids: readonly number[]): string {
  return `{${ids.map((id) => scene.edges[id].name).join(', ')}}`;
}
