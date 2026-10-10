/**
 * Sensor-based coverage — Choset §6.2.4, the cycle algorithm [ref. 11].
 *
 * The robot knows nothing about the world. It has Chapter 3's equipment: a
 * contact sensor for "move until D = δ" and a boundary follower that keeps a
 * standoff δ by tracing the offset curve G = D − δ = 0 (`offsetContact`, the
 * predictor–corrector of Appendix D), plus a range ring (`rangeScan`). It
 * covers the free space cell by cell and builds the Reeb graph as it goes.
 *
 * Critical-point sensing (Derivation 4). While following a boundary in the
 * sweep direction ±x, the robot is at a critical point of h = x on the
 * boundary exactly when its motion stops progressing in x and turns back: the
 * tangent is vertical, so the surface normal — the range ring's minimum ray —
 * is parallel to the sweep. The port watches the follower's x-progress and
 * declares an event when it has fallen back by `turnBack` after a maximum;
 * the event sits at the middle of the stretch where the maximum was attained.
 * On a smooth convex extremity that stretch is a short arc and its middle is
 * the point where the minimum ray is exactly horizontal; on an axis-aligned
 * wall — a degenerate critical point, which Choset's sensing cannot see — the
 * stretch is the whole wall and the two degenerate critical points at its
 * ends are reported once, at its middle. That is this port's merged-event
 * rule; it is beyond Choset. The range ring then classifies the event: if the
 * minimum ray points back against the direction of travel the boundary bulged
 * toward the robot (a convex extremity: a merge when moving +x, a split when
 * moving −x); otherwise the robot is in a corner that pinches the slice off (a
 * death, or a birth).
 *
 * Finding every critical point (the cycle algorithm). A raster scan — lap,
 * follow the boundary one lap width, lap back, follow the *other* boundary
 * one lap width, … — follows each boundary only on every other gap, so a
 * critical point on the boundary it is not following goes unseen (Choset Fig.
 * 6.22). The cycle algorithm closes a loop around every gap between two laps:
 *
 *   forward  lap; follow the boundary forward one lap width (or to an event)
 *   reverse  lap back; follow the other boundary backward one lap width
 *   closing  if that did not return to where the first lap started, an
 *            obstacle lies between: lap toward the start and follow forward
 *            until the critical point that explains it
 *   undo     retrace the reverse follow, and start the next gap
 *
 * The loop is the boundary of the gap region, so any critical point in the gap
 * lies on it. Each cell ends at its *closing* critical point; new cells hang
 * off that node as unexplored Reeb edges; a depth-first search over the Reeb
 * graph picks the next one, and the robot travels there over ground it has
 * already covered — its own trail plus range-ring line-of-sight shortcuts,
 * searched by Chapter 6's A*.
 */

import { offsetContact, type Contact } from '../bugs/follow';
import { witness } from '../bugs/geometry';
import { gradientFromScan, rangeScan } from '../bugs/scan';
import { tangentFromGradient } from '../bugs/trace';
import { astar } from '../search/best-first';
import type { Edge, Graph } from '../search/graph';
import { rayCast, type Point2, type World } from '../sim/world';
import { ReebGraph, type CriticalKind } from './reeb';
import { asWorld, type CellWorld } from './world';

const dist = (a: Point2, b: Point2) => Math.hypot(a.x - b.x, a.y - b.y);

export type Phase = 'seek' | 'entry' | 'lap' | 'forward' | 'reverse' | 'closing' | 'undo' | 'travel';
export type Policy = 'cycle' | 'raster';
export type Side = 'L' | 'LU' | 'LD' | 'R' | 'RU' | 'RD';

export const SIDES: Record<CriticalKind, Side[]> = {
  birth: ['R'],
  death: ['L'],
  split: ['L', 'RU', 'RD'],
  merge: ['LU', 'LD', 'R'],
};

export interface OnlineOptions {
  /** Effector width 2r = interlap spacing. */
  twoR: number;
  /** Standoff δ the follower keeps from every boundary. */
  standoff?: number;
  policy?: Policy;
  /** Range ring: rays and range. */
  nRays?: number;
  maxRange?: number;
  /** Where the robot is put down. */
  start: Point2;
}

export interface PathSample {
  p: Point2;
  phase: Phase;
  /** Arc length from the start. */
  s: number;
}

export interface OnlineNode {
  id: number;
  kind: CriticalKind;
  /** Where the robot was when it sensed the critical point (the anchor). */
  q: Point2;
  /** The obstacle point the range ring's minimum ray hit there. */
  obstaclePoint: Point2;
  /** Bearing of the minimum ray, radians (0 = +x). */
  minRayAngle: number;
  sides: Partial<Record<Side, number | null>>;
  /** Arc length at which it was discovered. */
  s: number;
}

export interface OnlineCell {
  id: number;
  dir: 1 | -1;
  from: number;
  fromSide: Side;
  to: number | null;
  toSide: Side | null;
  laps: number;
  /** Arc-length span during which this cell was being covered. */
  s0: number;
  s1: number;
}

export interface OnlineEvent {
  s: number;
  node: number;
  cell: number | null;
  phase: Phase;
  matched: boolean;
}

export interface OnlineResult {
  policy: Policy;
  path: PathSample[];
  nodes: OnlineNode[];
  cells: OnlineCell[];
  events: OnlineEvent[];
  reeb: ReebGraph;
  length: number;
  byPhase: Record<Phase, number>;
  laps: number;
  /** Cells abandoned because a follow ran out of budget (should be 0). */
  failures: number;
}

interface FollowEvent {
  anchor: Point2;
  kind: CriticalKind;
  obstaclePoint: Point2;
  minRayAngle: number;
}

interface FollowResult {
  end: Point2;
  event: FollowEvent | null;
  /** The obstacle being followed lay below the robot (the robot followed a floor). */
  obstacleBelow: boolean;
  /** Points traversed (including the start), for the undo. */
  pts: Point2[];
  stalled: boolean;
}

/** A Chapter 6 graph over the robot's own trail, for travelling through covered ground. */
class TrailGraph implements Graph<number> {
  pts: Point2[] = [];
  adj: { to: number; cost: number; via: Point2[] | null }[][] = [];
  private buckets = new Map<string, number[]>();
  private sinceLast = 0;
  private lastKept = -1;

  constructor(
    private readonly world: World,
    private readonly spacing = 0.12,
    private readonly reach = 1.0,
  ) {}

  private bucketKey(p: Point2) {
    return `${Math.floor(p.x / this.reach)},${Math.floor(p.y / this.reach)}`;
  }

  /** Feed a segment of the executed path; keeps points every `spacing` metres. */
  feed(pts: readonly Point2[], forceKeepLast = false) {
    let pending: Point2[] = this.lastKept >= 0 ? [this.pts[this.lastKept]] : [];
    for (let i = 0; i < pts.length; i++) {
      const p = pts[i];
      if (this.lastKept < 0) {
        this.keep(p, null);
        pending = [p];
        continue;
      }
      const prev = pending[pending.length - 1];
      this.sinceLast += dist(prev, p);
      pending.push(p);
      if (this.sinceLast >= this.spacing || (forceKeepLast && i === pts.length - 1)) {
        this.keep(p, pending);
        pending = [p];
        this.sinceLast = 0;
      }
    }
  }

  private keep(p: Point2, via: Point2[] | null) {
    const id = this.pts.length;
    this.pts.push(p);
    this.adj.push([]);
    if (this.lastKept >= 0 && via) {
      let L = 0;
      for (let i = 1; i < via.length; i++) L += dist(via[i - 1], via[i]);
      this.adj[this.lastKept].push({ to: id, cost: L, via });
      this.adj[id].push({ to: this.lastKept, cost: L, via: [...via].reverse() });
    }
    // Line-of-sight shortcuts the range ring can vouch for.
    const bx = Math.floor(p.x / this.reach), by = Math.floor(p.y / this.reach);
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        for (const j of this.buckets.get(`${bx + dx},${by + dy}`) ?? []) {
          const q = this.pts[j];
          const d = dist(p, q);
          if (d > this.reach || d < 1e-9 || j === this.lastKept) continue;
          const ang = Math.atan2(q.y - p.y, q.x - p.x);
          if (rayCast(this.world, p.x, p.y, ang, d + 1e-6) < d - 1e-9) continue;
          if (rayCast(this.world, q.x, q.y, ang + Math.PI, d + 1e-6) < d - 1e-9) continue;
          this.adj[id].push({ to: j, cost: d, via: null });
          this.adj[j].push({ to: id, cost: d, via: null });
        }
      }
    }
    const k = this.bucketKey(p);
    if (!this.buckets.has(k)) this.buckets.set(k, []);
    this.buckets.get(k)!.push(id);
    this.lastKept = id;
  }

  neighbors(n: number): readonly Edge<number>[] {
    return this.adj[n];
  }

  key(n: number) {
    return n;
  }

  nearest(p: Point2): number {
    let best = 0, bd = Infinity;
    this.pts.forEach((q, i) => {
      const d = dist(p, q);
      if (d < bd) {
        bd = d;
        best = i;
      }
    });
    return best;
  }

  route(a: number, b: number): Point2[] | null {
    const goal = this.pts[b];
    const tr = astar(this, a, b, (n) => dist(this.pts[n], goal));
    if (tr.status !== 'found') return null;
    const out: Point2[] = [this.pts[a]];
    for (let i = 0; i + 1 < tr.path.length; i++) {
      const e = this.adj[tr.path[i]].filter((x) => x.to === tr.path[i + 1]).sort((u, v) => u.cost - v.cost)[0];
      if (e.via) out.push(...e.via.slice(1));
      else out.push(this.pts[tr.path[i + 1]]);
    }
    return out;
  }
}

/**
 * Run sensor-based boustrophedon coverage to completion. Deterministic: the
 * only inputs are the world, the start and the options.
 */
export function runOnlineCoverage(cw: CellWorld, opts: OnlineOptions): OnlineResult {
  const world = asWorld(cw);
  const w = opts.twoR;
  const r = w / 2;
  const delta = opts.standoff ?? Math.min(0.05, r / 3);
  const policy = opts.policy ?? 'cycle';
  const nRays = opts.nRays ?? 360;
  const maxRange = opts.maxRange ?? 4;
  const contact: Contact = offsetContact(world, { clearance: delta, maxHop: delta / 2 });
  const turnBack = 0.3 * delta;
  const nodeTol = Math.max(5 * delta, 0.6 * r);
  const perimeterBudget = 4 * world.walls.reduce((s, q) => s + Math.hypot(q.x2 - q.x1, q.y2 - q.y1), 0);

  const path: PathSample[] = [];
  let s = 0;
  let x: Point2 = opts.start;
  const trail = new TrailGraph(world);
  const byPhase = { seek: 0, entry: 0, lap: 0, forward: 0, reverse: 0, closing: 0, undo: 0, travel: 0 } as Record<Phase, number>;
  path.push({ p: x, phase: 'seek', s: 0 });
  trail.feed([x]);

  const record = (pts: readonly Point2[], phase: Phase, keepLast = false) => {
    for (const p of pts) {
      const d = dist(x, p);
      if (d < 1e-12) continue;
      s += d;
      byPhase[phase] += d;
      x = p;
      path.push({ p, phase, s });
    }
    trail.feed(pts, keepLast);
  };

  const DG = (p: Point2) => witness(world, p).grad;

  /** Move straight toward `target` until contact (D = δ) or arrival. */
  const moveTo = (target: Point2, phase: Phase): { hit: boolean } => {
    const res = contact.moveToward(x, target, dist(x, target));
    record([res.x], phase, true);
    return { hit: res.hit };
  };

  /** A lap: straight up (+1) or down (−1) until contact. */
  const lap = (dirY: 1 | -1, phase: Phase = 'lap'): Point2 => {
    moveTo({ x: x.x, y: x.y + dirY * 1e3 }, phase);
    return x;
  };

  /** Read the range ring at p and classify a turn-back seen while moving `movingDx`. */
  const sense = (p: Point2, movingDx: 1 | -1): FollowEvent => {
    const scan = rangeScan(world, p.x, p.y, nRays, maxRange);
    const g = gradientFromScan(scan, { world });
    const d = witness(world, p).d;
    const convex = g.x * movingDx > 0.3;
    const kind: CriticalKind = movingDx > 0 ? (convex ? 'merge' : 'death') : convex ? 'split' : 'birth';
    return { anchor: p, kind, obstaclePoint: { x: p.x - d * g.x, y: p.y - d * g.y }, minRayAngle: Math.atan2(-g.y, -g.x) };
  };

  /**
   * Follow the boundary the robot is touching, preferring initial direction
   * `prefer`, measuring progress along `wantDx`; stop after `lateral` of
   * progress, or at a turn-back event when `detect` is on.
   */
  const follow = (wantDx: 1 | -1, lateral: number, prefer: Point2, detect: boolean, phase: Phase): FollowResult => {
    const g0 = DG(x);
    const heading = { x: -g0.x, y: -g0.y };
    const tR = tangentFromGradient(g0, 'right');
    const leftTurn = heading.x * tR.y - heading.y * tR.x > 0 ? 'right' : 'left';
    const tSide = (side: 'left' | 'right') => tangentFromGradient(g0, side);
    const dot = (a: Point2, b: Point2) => a.x * b.x + a.y * b.y;
    const wantSide: 'left' | 'right' = dot(tSide('left'), prefer) >= dot(tSide('right'), prefer) ? 'left' : 'right';
    const turn = wantSide === leftTurn ? 'left' : 'right';
    const f = contact.follow(x, heading, turn);
    const obstacleBelow = g0.y > 0;
    const x0 = x.x;
    const pts: Point2[] = [x];
    const prog: number[] = [0];
    let best = 0;
    let walked = 0;
    const hop = delta / 2;
    while (walked < perimeterBudget) {
      const m = f.advance(hop);
      walked += hop;
      const p = m.x;
      pts.push(p);
      const pr = (p.x - x0) * wantDx;
      prog.push(pr);
      if (pr > best) best = pr;
      if (pr >= lateral) {
        record(pts.slice(1), phase, true);
        return { end: x, event: null, obstacleBelow, pts, stalled: false };
      }
      if (detect && pr < best - turnBack) {
        // The stretch where the maximum was attained; the event sits at its middle.
        let hi = prog.length - 1;
        while (hi > 0 && prog[hi] < best - 0.25 * turnBack) hi--;
        let lo = hi;
        while (lo > 0 && prog[lo - 1] >= best - 0.25 * turnBack) lo--;
        // Middle by arc length.
        let total = 0;
        for (let i = lo + 1; i <= hi; i++) total += dist(pts[i - 1], pts[i]);
        let acc = 0;
        let mid = lo;
        for (let i = lo + 1; i <= hi; i++) {
          acc += dist(pts[i - 1], pts[i]);
          if (acc >= total / 2) {
            mid = i;
            break;
          }
        }
        const kept = pts.slice(0, mid + 1);
        record(kept.slice(1), phase, true);
        let ev = sense(x, wantDx);
        if (ev.kind === 'merge' || ev.kind === 'split') {
          // A convex extremity: the robot rounds it on an arc of radius δ about
          // the obstacle point c, and the critical configuration is the arc's
          // point level with c — where the minimum ray is exactly horizontal.
          // Step to it along the arc's chord (outside the obstacle) and re-read.
          // Iterate: if the stretch's middle sat on a straight run beside the
          // corner, the first snap lands on the arc and the second is exact.
          for (let it = 0; it < 4; it++) {
            const snap = { x: ev.obstaclePoint.x + wantDx * delta, y: ev.obstaclePoint.y };
            if (dist(snap, x) < 1e-9) break;
            if (dist(snap, x) > 3 * delta || Math.abs(witness(world, snap).d - delta) > 0.1 * delta) break;
            record([snap], phase, true);
            kept.push(snap);
            const again = sense(x, wantDx);
            if (again.kind !== ev.kind) break;
            ev = again;
          }
        }
        return { end: x, event: ev, obstacleBelow, pts: kept, stalled: false };
      }
    }
    record(pts.slice(1), phase, true);
    return { end: x, event: null, obstacleBelow, pts, stalled: true };
  };

  // ---------------------------------------------------------------------------
  // The Reeb graph, grown online
  // ---------------------------------------------------------------------------

  const nodes: OnlineNode[] = [];
  const cells: OnlineCell[] = [];
  const events: OnlineEvent[] = [];
  let failures = 0;
  let laps = 0;

  const nodeFor = (ev: FollowEvent, cell: number | null, arrival: Side | null, phase: Phase): OnlineNode => {
    let n = nodes.find((m) => m.kind === ev.kind && dist(m.q, ev.anchor) < nodeTol);
    const matched = !!n;
    if (!n) {
      n = {
        id: nodes.length,
        kind: ev.kind,
        q: ev.anchor,
        obstaclePoint: ev.obstaclePoint,
        minRayAngle: ev.minRayAngle,
        sides: Object.fromEntries(SIDES[ev.kind].map((sd) => [sd, null])) as Partial<Record<Side, number | null>>,
        s,
      };
      nodes.push(n);
    }
    if (cell !== null && arrival && n.sides[arrival] === null) n.sides[arrival] = cell;
    else if (cell !== null && arrival && n.sides[arrival] !== cell) {
      // The side was taken: put the cell on any free side of the same direction.
      const alt = SIDES[n.kind].find((sd) => sd[0] === arrival[0] && n!.sides[sd] === null);
      if (alt) n.sides[alt] = cell;
    }
    events.push({ s, node: n.id, cell, phase, matched });
    return n;
  };

  /** Which side of the closing node a cell arrives on. */
  const arrivalSide = (kind: CriticalKind, dir: 1 | -1, cellAbove: boolean | null): Side => {
    if (dir === 1) {
      if (kind === 'merge') return cellAbove ? 'LU' : 'LD';
      return 'L';
    }
    if (kind === 'split') return cellAbove ? 'RU' : 'RD';
    return 'R';
  };

  // ---------------------------------------------------------------------------
  // Covering one cell
  // ---------------------------------------------------------------------------

  type SideName = 'floor' | 'ceil';
  const flip = (sd: SideName): SideName => (sd === 'floor' ? 'ceil' : 'floor');

  const closeWith = (c: OnlineCell, ev: FollowEvent, cellAbove: boolean | null, phase: Phase, followedFloor?: boolean, gapStart?: Point2): OnlineNode => {
    const kind = ev.kind;
    const arr = arrivalSide(kind, c.dir, cellAbove);
    const n = nodeFor(ev, c.id, SIDES[kind].includes(arr) ? arr : null, phase);
    c.to = n.id;
    c.toSide = arr;
    if (policy === 'cycle') {
      if (kind === 'split' || kind === 'merge') {
        // The closing lap: one full slice through the critical point, so the
        // strip between the last lap and the critical slice is not left bare.
        // Step off the obstacle's extremity first: a lap at exactly the
        // standoff would graze a leaning wall and stop.
        const away = ev.obstaclePoint.x > x.x ? -1 : 1;
        moveTo({ x: x.x + away * delta, y: x.y }, 'lap');
        lap(1);
        lap(-1);
        laps++;
        c.laps++;
      } else if (followedFloor !== undefined && gapStart) {
        // A pinch: walk the other boundary back over the last gap.
        follow((-c.dir) as 1 | -1, Math.abs(x.x - gapStart.x), { x: -c.dir, y: followedFloor ? 1 : -1 }, false, 'closing');
      }
    }
    c.s1 = s;
    return n;
  };

  /** Cover cell c starting from the boundary `side` the robot stands on. Returns the closing node. */
  const coverCell = (c: OnlineCell, startSide: SideName): OnlineNode | null => {
    const dir = c.dir;
    let side = startSide;
    let Tk = x; // the far end of the previous lap, on the side opposite the robot
    lap(side === 'floor' ? 1 : -1);
    laps++;
    c.laps++;
    side = flip(side);
    for (let guard = 0; guard < 400; guard++) {
      // Forward phase: follow the current boundary one lap width.
      const xStart = x;
      // Aim a hop short of a full width: the follower stops on the first hop past it.
      const r1 = follow(dir, w - delta / 2, { x: dir, y: 0 }, true, 'forward');
      if (r1.event) return closeWith(c, r1.event, r1.obstacleBelow, 'forward', r1.obstacleBelow, xStart);
      if (r1.stalled) return null;
      const fwdEnd = x;
      // Lap to the other boundary.
      lap(side === 'floor' ? 1 : -1);
      laps++;
      c.laps++;
      const side2 = flip(side);
      if (policy === 'raster') {
        Tk = fwdEnd;
        side = side2;
        continue;
      }
      const lapEnd = x;
      // Reverse phase: follow the other boundary back to the previous lap.
      const back = Math.abs(fwdEnd.x - xStart.x);
      const r2 = follow((-dir) as 1 | -1, back, { x: -dir, y: 0 }, true, 'reverse');
      if (r2.event) return closeWith(c, r2.event, null, 'reverse');
      if (r2.stalled) return null;
      if (dist(x, Tk) > Math.max(4 * delta, 0.25 * w)) {
        // Closing phase: something separates us from where the gap began.
        moveTo({ x: x.x, y: Tk.y }, 'closing');
        const contactPt = x;
        if (dist(x, Tk) > 2 * delta) {
          const r3 = follow(dir, 3 * w, { x: dir, y: 0 }, true, 'closing');
          if (r3.event) return closeWith(c, r3.event, Tk.y > contactPt.y, 'closing');
        }
        // No critical point explains it (should not happen): go back to the lap's end and carry on.
        const a = trail.nearest(x);
        const b = trail.nearest(lapEnd);
        const route = trail.route(a, b);
        if (route) record(route, 'travel', true);
      } else {
        // Undo the reverse follow: retrace it exactly.
        record([...r2.pts].reverse().slice(1), 'undo', true);
      }
      Tk = fwdEnd;
      side = side2;
    }
    return null;
  };

  // ---------------------------------------------------------------------------
  // Main loop: seek a first critical point, then DFS over the Reeb graph
  // ---------------------------------------------------------------------------

  moveTo({ x: x.x - 1e3, y: x.y }, 'seek');
  const seek = follow(-1, Infinity, { x: -1, y: -0.2 }, true, 'seek');
  if (!seek.event) {
    failures++;
    return finish();
  }
  const root = nodeFor(seek.event, null, null, 'seek');
  const stack: number[] = [root.id];

  for (let guard = 0; guard < 200 && stack.length; guard++) {
    let n: OnlineNode | null = null;
    let sd: Side | null = null;
    while (stack.length) {
      const top = nodes[stack[stack.length - 1]];
      const free = SIDES[top.kind].find((k) => top.sides[k] === null);
      if (free) {
        n = top;
        sd = free;
        break;
      }
      stack.pop();
    }
    if (!n || !sd) break;
    // Travel to the node over covered ground.
    if (dist(x, n.q) > 1e-6) {
      const route = trail.route(trail.nearest(x), trail.nearest(n.q));
      if (route) record([...route, n.q], 'travel', true);
      else moveTo(n.q, 'travel');
    }
    const dir: 1 | -1 = sd[0] === 'R' ? 1 : -1;
    const c: OnlineCell = { id: cells.length, dir, from: n.id, fromSide: sd, to: null, toSide: null, laps: 0, s0: s, s1: s };
    cells.push(c);
    n.sides[sd] = c.id;

    // Entry maneuver.
    let startSide: 'floor' | 'ceil';
    let closed: OnlineNode | null = null;
    if (sd === 'RU' || sd === 'LU' || sd === 'RD' || sd === 'LD') {
      const up = sd.endsWith('U');
      const r0 = follow(dir, r - delta, { x: 0, y: up ? 1 : -1 }, true, 'entry');
      if (r0.event) closed = closeWith(c, r0.event, up, 'entry');
      startSide = up ? 'floor' : 'ceil';
    } else {
      // The anchor is already δ off the boundary: the first lap goes r − δ further.
      moveTo({ x: x.x + dir * (r - delta), y: x.y }, 'entry');
      lap(-1, 'entry');
      startSide = 'floor';
    }
    if (!closed) closed = coverCell(c, startSide);
    if (!closed) {
      failures++;
      c.s1 = s;
      continue;
    }
    stack.push(closed.id);
  }
  return finish();

  function finish(): OnlineResult {
    const reeb = new ReebGraph();
    for (const n of nodes) reeb.addNode(n.q, n.kind, n.id);
    for (const c of cells) if (c.to !== null) reeb.addEdge(c.id, c.from, c.to);
    return { policy, path, nodes, cells, events, reeb, length: s, byPhase, laps, failures };
  }
}

/** The points of an online run, for coverage measurement. */
export const onlinePoints = (res: OnlineResult): Point2[] => res.path.map((q) => q.p);
