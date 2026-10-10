/**
 * Sensor-based incremental construction of the GVG — Choset §5.2.5 and
 * §5.3.3's five steps: (1) access, (2) trace edges, (3) locate nodes,
 * (4) explore the branches leaving each node, (5) terminate.
 *
 * The robot is a point driven by a velocity; its only sense is a
 * `NearestQuery` (a range ring's local minima, or exact geometry in the
 * checks). The construction is a depth-first search over a graph the robot
 * is still drawing: every meet point is a node with m + 1 departure slots,
 * an edge is traced with the Lyapunov law until a further obstacle ties
 * (a meet point: home onto it) or the clearance vanishes (a boundary point:
 * turn back), and when a node has no unexplored slot the robot drives back
 * along known edges to the nearest node that has one. When none is left the
 * roadmap is complete — for whatever part of the free space the robot could
 * reach by tracing, which in R³ is not all of it (§5.3.2).
 *
 * Meet-point detection is the heuristic Choset states: "a sudden change in
 * one of the two closest obstacles", read here as the (m + 1)-th nearest
 * reading coming within `meetTol` of the nearest. It is a heuristic, and
 * with noisy ranges it fires early or late; the widget shows both.
 */

import { degenerateDepartures2D, departureDirections, homeLaw, homeToMeetPoint, nullSpaceDirection, projectToGvg, pseudoInverse, traceLaw, vadd, vdist, vdot, vnorm, vscale, vsub, vunit, matVec, constraint, type NearestQuery, type Vec } from './gvg';

export type Phase = 'access' | 'depart' | 'trace' | 'home' | 'backtrack' | 'done' | 'stuck';

export type NodeKind = 'meet' | 'access' | 'boundary';

export interface GvgNode {
  id: number;
  q: Vec;
  kind: NodeKind;
  /** D at the node. */
  clearance: number;
  /** Unit departure directions (m + 1 at a meet point, 2 at the access point, 0 at a boundary point). */
  departures: Vec[];
  explored: boolean[];
  /** Ids of incident edges. */
  edges: number[];
}

export interface GvgEdge {
  id: number;
  from: number;
  /** −1 while the edge is still being traced. */
  to: number;
  polyline: Vec[];
  length: number;
}

export interface ExplorerOptions {
  /** Speed along the edge, α (m/s). */
  alpha?: number;
  /** Correction gain, β < 0 (1/s). */
  beta?: number;
  /** Integration step (s). */
  dt?: number;
  /** A further obstacle this close to the nearest means a meet point is near. */
  meetTol?: number;
  /** Two readings this close count as tied during access. */
  accessTol?: number;
  /** Clearance below which an edge has run into the boundary. */
  boundaryTol?: number;
  /** Two nodes closer than this are the same node. */
  nodeTol?: number;
  /** Distance to step off a node before tracing starts. */
  departStep?: number;
  /** Give up after this many ticks. */
  maxTicks?: number;
  /** Ticks to ignore meet detection after a rejected homing. */
  cooldownTicks?: number;
  /** Keep at most this many trail points (for drawing). */
  trailCap?: number;
  /** Accept a homed meet point whose ‖G‖ stalls above the Newton tolerance but below this (quantized or noisy readings). */
  homeTol?: number;
  /**
   * A further reading within this of the tie at a meet point makes the node degenerate (two meet
   * points closer than the robot can tell apart): in the plane it then gets one departure per
   * adjacent pair of closest points instead of three. Defaults to 3·nodeTol + departStep.
   */
  degenerateTol?: number;
}

const DEFAULTS: Required<ExplorerOptions> = {
  alpha: 1,
  beta: -3,
  dt: 0.03,
  meetTol: 0.08,
  accessTol: 0.01,
  boundaryTol: 0.12,
  nodeTol: 0.02,
  departStep: 0.02,
  maxTicks: 60000,
  cooldownTicks: 25,
  trailCap: 6000,
  homeTol: 1e-6,
  degenerateTol: -1,
};

export interface ExplorerEvent {
  kind: 'none' | 'accessed' | 'meet' | 'boundary' | 'revisit' | 'spurious' | 'done' | 'stuck';
  node?: number;
}

/**
 * The explorer. Call `tick()` once per simulation step; read `phase`, `q`,
 * `nodes`, `edges`, `gammaHistory` for drawing; `graph()` for a Roadmap-like
 * summary. The same class serves the plane (dim 2, from a scan) and space
 * (dim 3, from analytic sites); only the query differs.
 */
export class GvgExplorer {
  readonly opts: Required<ExplorerOptions>;
  readonly dim: number;
  phase: Phase = 'access';
  q: Vec;
  heading: Vec;
  nodes: GvgNode[] = [];
  edges: GvgEdge[] = [];
  /** Γ at each trace tick (the sparkline). */
  gammaHistory: number[] = [];
  /** Everything the robot has driven, for the orange trail. */
  trail: Vec[] = [];
  distanceDriven = 0;
  ticks = 0;
  /** Number of readings currently tied (during access). */
  accessTies = 1;
  /** The last law evaluation, for the widget's readout. */
  last: { G: Vec; gamma: number; dGamma: number; margin: number; nearest: number[] } | null = null;

  private currentEdge: GvgEdge | null = null;
  private currentNode = -1;
  private cooldown = 0;
  private backtrackPath: Vec[] = [];
  private backtrackIndex = 0;
  private backtrackTarget = -1;
  private homeStart: Vec | null = null;
  private homeIters = 0;

  constructor(
    readonly query: NearestQuery,
    q0: Vec,
    opts: ExplorerOptions = {},
    heading0?: Vec,
  ) {
    this.opts = { ...DEFAULTS, ...opts };
    this.dim = q0.length;
    this.q = q0.slice();
    this.heading = heading0 ? vunit(heading0) : [1, ...new Array<number>(this.dim - 1).fill(0)];
    this.trail.push(this.q.slice());
  }

  /** Unexplored departure slots over the whole graph — zero means the construction is complete. */
  get frontier(): number {
    return this.nodes.reduce((s, n) => s + n.explored.filter((e) => !e).length, 0);
  }

  get meetPoints(): GvgNode[] {
    return this.nodes.filter((n) => n.kind === 'meet');
  }

  private move(to: Vec) {
    this.distanceDriven += vdist(this.q, to);
    this.q = to;
    this.trail.push(to.slice());
    if (this.trail.length > this.opts.trailCap) this.trail.splice(0, this.trail.length - this.opts.trailCap);
  }

  private addNode(q: Vec, kind: NodeKind, clearance: number, departures: Vec[]): GvgNode {
    const node: GvgNode = { id: this.nodes.length, q: q.slice(), kind, clearance, departures, explored: departures.map(() => false), edges: [] };
    this.nodes.push(node);
    return node;
  }

  private nodeNear(q: Vec, kind?: NodeKind): GvgNode | null {
    let best: GvgNode | null = null;
    let bd = this.opts.nodeTol;
    for (const n of this.nodes) {
      if (kind && n.kind !== kind) continue;
      const d = vdist(n.q, q);
      if (d < bd) {
        bd = d;
        best = n;
      }
    }
    return best;
  }

  /** Mark the departure slot of `node` that the robot just arrived along (the one pointing back at it). */
  private markArrival(node: GvgNode, arrivalHeading: Vec) {
    let best = -1;
    let bd = -Infinity;
    node.departures.forEach((d, i) => {
      const s = -vdot(d, arrivalHeading);
      if (s > bd) {
        bd = s;
        best = i;
      }
    });
    if (best >= 0) node.explored[best] = true;
  }

  private finishEdge(node: GvgNode) {
    const e = this.currentEdge;
    if (!e) return;
    e.to = node.id;
    e.polyline.push(node.q.slice());
    e.length = 0;
    for (let i = 1; i < e.polyline.length; i++) e.length += vdist(e.polyline[i - 1], e.polyline[i]);
    node.edges.push(e.id);
    this.currentEdge = null;
  }

  /** Pick the next unexplored slot at `node` or start backtracking. */
  private continueFrom(node: GvgNode): void {
    this.currentNode = node.id;
    if (node.explored.some((e) => !e)) {
      this.phase = 'depart';
      return;
    }
    this.planBacktrack();
  }

  private planBacktrack(): void {
    // BFS over nodes by edge hops to the nearest node with an unexplored slot.
    const start = this.currentNode;
    const prev = new Map<number, { node: number; edge: number }>();
    const seen = new Set<number>([start]);
    const queue = [start];
    let target = -1;
    while (queue.length) {
      const n = queue.shift()!;
      if (n !== start && this.nodes[n].explored.some((e) => !e)) {
        target = n;
        break;
      }
      for (const eid of this.nodes[n].edges) {
        const e = this.edges[eid];
        if (e.to < 0) continue;
        const other = e.from === n ? e.to : e.from;
        if (seen.has(other)) continue;
        seen.add(other);
        prev.set(other, { node: n, edge: eid });
        queue.push(other);
      }
    }
    if (target < 0) {
      this.phase = this.frontier === 0 ? 'done' : 'stuck';
      return;
    }
    // Assemble the polyline from start to target.
    const hops: { node: number; edge: number }[] = [];
    let cur = target;
    while (cur !== start) {
      const p = prev.get(cur)!;
      hops.push({ node: cur, edge: p.edge });
      cur = p.node;
    }
    hops.reverse();
    const path: Vec[] = [];
    let at = start;
    for (const h of hops) {
      const e = this.edges[h.edge];
      const pl = e.from === at ? e.polyline : [...e.polyline].reverse();
      for (const p of pl) path.push(p);
      at = h.node;
    }
    this.backtrackPath = path;
    this.backtrackIndex = 0;
    this.backtrackTarget = target;
    this.phase = 'backtrack';
  }

  /** One simulation step. */
  tick(): ExplorerEvent {
    if (this.phase === 'done' || this.phase === 'stuck') return { kind: this.phase };
    this.ticks++;
    if (this.ticks > this.opts.maxTicks) {
      this.phase = 'stuck';
      return { kind: 'stuck' };
    }
    if (this.cooldown > 0) this.cooldown--;
    switch (this.phase) {
      case 'access':
        return this.tickAccess();
      case 'depart':
        return this.tickDepart();
      case 'trace':
        return this.tickTrace();
      case 'home':
        return this.tickHome();
      case 'backtrack':
        return this.tickBacktrack();
      default:
        return { kind: 'none' };
    }
  }

  /**
   * Access (Lemma 5.2.1, then §5.3.3): ascend ∇D until two readings tie,
   * then ascend along the two-equidistant sheet until a third ties, … until
   * m readings tie — the robot is on a GVG edge. Each stage is the law with
   * k − 1 constraint rows and v the projected gradient of d₁ (the direction
   * that increases the common distance while keeping the ties).
   */
  private tickAccess(): ExplorerEvent {
    const { alpha, beta, dt, meetTol } = this.opts;
    // A tie is detected between steps, so the tolerance must exceed what one step can change.
    const accessTol = Math.max(this.opts.accessTol, 1.5 * alpha * dt);
    const m = this.dim;
    const near = this.query(this.q);
    if (near.length < m) {
      this.phase = 'stuck';
      return { kind: 'stuck' };
    }
    // How many readings are tied with the nearest?
    let k = 1;
    while (k < near.length && near[k].d - near[0].d < accessTol) k++;
    this.accessTies = Math.min(k, m);
    if (k >= m) {
      // On an edge (to accessTol): project exactly onto it.
      const on = projectToGvg(this.query, this.q);
      if (on) this.move(on);
      const n2 = this.query(this.q);
      if (n2.length > m && n2[m].d - n2[0].d < meetTol) {
        // Landed next to a meet point: home onto it and start from there (no arrival edge).
        this.currentEdge = null;
        this.phase = 'home';
        this.homeStart = this.q.slice();
        this.homeIters = 0;
        return { kind: 'none' };
      }
      const t = nullSpaceDirection(constraint(n2, m).DG, this.heading);
      if (vnorm(t) < 1e-9) {
        // Non-transversal tie (parallel gradients): not SS_ij — keep climbing.
        this.move(vadd(this.q, vscale(near[0].grad, alpha * dt)));
        return { kind: 'none' };
      }
      const node = this.addNode(this.q, 'access', n2[0].d, [t, vscale(t, -1)]);
      this.last = { G: constraint(n2, m).G, gamma: 0, dGamma: 0, margin: 0, nearest: n2.slice(0, m).map((n) => n.d) };
      this.continueFrom(node);
      return { kind: 'accessed', node: node.id };
    }
    if (k === 1) {
      // Plain gradient ascent of D.
      this.move(vadd(this.q, vscale(near[0].grad, alpha * dt)));
      return { kind: 'none' };
    }
    // k ties (1 < k < m): keep them with the correction term, climb along the projected ∇d₁.
    const { G, DG } = constraint(near, k);
    const dag = pseudoInverse(DG);
    if (!dag) {
      this.move(vadd(this.q, vscale(near[0].grad, alpha * dt)));
      return { kind: 'none' };
    }
    // Projection onto Null(DG): P g = g − DG† DG g.
    const g = near[0].grad;
    const proj = vsub(g, matVec(dag, matVec(DG, g)));
    const v = vunit(proj);
    const correction = vscale(matVec(dag, G), beta);
    this.move(vadd(this.q, vscale(vadd(vscale(v, alpha), correction), dt)));
    return { kind: 'none' };
  }

  private tickDepart(): ExplorerEvent {
    const node = this.nodes[this.currentNode];
    const slot = node.explored.findIndex((e) => !e);
    if (slot < 0) {
      this.continueFrom(node);
      return { kind: 'none' };
    }
    node.explored[slot] = true;
    const dir = node.departures[slot];
    this.heading = dir.slice();
    const start = vadd(node.q, vscale(dir, this.opts.departStep));
    const edge: GvgEdge = { id: this.edges.length, from: node.id, to: -1, polyline: [node.q.slice(), start.slice()], length: 0 };
    this.edges.push(edge);
    node.edges.push(edge.id);
    this.currentEdge = edge;
    this.move(start);
    this.cooldown = Math.max(this.cooldown, 3);
    this.phase = 'trace';
    return { kind: 'none' };
  }

  private tickTrace(): ExplorerEvent {
    const { alpha, beta, dt, meetTol, boundaryTol, nodeTol } = this.opts;
    const m = this.dim;
    const near = this.query(this.q);
    const edge = this.currentEdge!;
    if (near.length < m || near[0].d < boundaryTol) {
      // The edge has run into the boundary (D → 0): a boundary point. Turn back.
      const node = this.addNode(this.q, 'boundary', near.length ? near[0].d : 0, []);
      this.finishEdge(node);
      this.last = null;
      this.continueFrom(this.nodes[edge.from]);
      return { kind: 'boundary', node: node.id };
    }
    const st = traceLaw(near, alpha, beta, this.heading);
    this.gammaHistory.push(st.gamma);
    if (this.gammaHistory.length > 400) this.gammaHistory.shift();
    this.last = { G: st.G, gamma: st.gamma, dGamma: st.dGamma, margin: st.margin, nearest: near.slice(0, m + 1).map((n) => n.d) };
    // Meet detection: the (m + 1)-th reading has caught up with the nearest. Decide with a virtual
    // homing (Newton from here, without moving): the node we just left does not count unless we
    // have gone round a cycle, and a near-tie that homes nowhere is spurious.
    const from = this.nodes[edge.from];
    if (this.cooldown === 0 && near.length > m && near[m].d - near[0].d < meetTol) {
      const h = homeToMeetPoint(this.query, this.q, 50, 1e-11, this.opts.homeTol);
      const backHome = h && vdist(h.q, from.q) < nodeTol && edge.polyline.length < 40;
      if (h && !backHome) {
        this.phase = 'home';
        this.homeStart = this.q.slice();
        this.homeIters = 0;
        return { kind: 'none' };
      }
      if (!h) this.cooldown = this.opts.cooldownTicks;
    }
    // Passing through an access node (a degree-two node in the middle of an edge, known from the
    // start of the exploration): the edge ends there, and the access node's slot pointing back at
    // us is explored.
    const acc = this.nodeNear(this.q, 'access');
    if (acc && acc.id !== from.id) {
      this.markArrival(acc, this.heading);
      this.finishEdge(acc);
      this.continueFrom(acc);
      return { kind: 'revisit', node: acc.id };
    }
    // Closing a cycle back onto the node we left (the GVG cycle of §5.3.2 has no meet point).
    if (edge.polyline.length > 10 && vdist(this.q, from.q) < nodeTol && from.kind !== 'boundary') {
      this.markArrival(from, this.heading);
      this.finishEdge(from);
      this.continueFrom(from);
      return { kind: 'revisit', node: from.id };
    }
    if (st.singular) {
      // Parallel gradients: not SS_ij. Nudge along the heading and hope the tie resolves.
      this.move(vadd(this.q, vscale(this.heading, alpha * dt)));
      return { kind: 'none' };
    }
    const next = vadd(this.q, vscale(st.qdot, dt));
    const t = vunit(st.tangent);
    if (vnorm(t) > 0) this.heading = t;
    this.move(next);
    edge.polyline.push(next.slice());
    return { kind: 'none' };
  }

  /**
   * Homing (§5.3.3): the law with m + 1 readings, whose null space is empty,
   * run as Newton steps (β dt = −1) until ‖G‖ vanishes. The fixed point is
   * the centre of the circle (sphere) through the m + 1 closest points.
   */
  private tickHome(): ExplorerEvent {
    const m = this.dim;
    const near = this.query(this.q);
    const edge = this.currentEdge;
    const reject = (): ExplorerEvent => {
      // Spurious: a near-tie that is not a meet point. Resume tracing (or climbing) with a cooldown.
      this.phase = edge ? 'trace' : 'access';
      this.cooldown = this.opts.cooldownTicks;
      return { kind: 'spurious' };
    };
    if (near.length < m + 1) return reject();
    const st = homeLaw(near, -1, m);
    this.homeIters++;
    this.last = { G: st.G, gamma: st.gamma, dGamma: st.dGamma, margin: st.margin, nearest: near.slice(0, m + 1).map((n) => n.d) };
    if (st.singular || this.homeIters > 12 || (this.homeStart && vdist(this.q, this.homeStart) > 6 * this.opts.meetTol + 0.3)) return reject();
    const gnorm = vnorm(st.G);
    if (gnorm > 1e-9) {
      // One Newton step, capped by the clearance (distance functions are 1-Lipschitz).
      const dq = st.qdot; // β = −1: −DG⁻¹ G
      const n = vnorm(dq);
      const cap = Math.max(0.5 * near[0].d, 1e-3);
      this.move(vadd(this.q, n > cap ? vscale(dq, cap / n) : dq));
      if (edge) edge.polyline.push(this.q.slice());
      return { kind: 'none' };
    }
    // Converged. Polish, then validate: the m + 1 nearest tie, and nothing else does within tolerance.
    const polished = homeToMeetPoint(this.query, this.q, 50, 1e-11, this.opts.homeTol);
    if (!polished) return reject();
    const nr = polished.near;
    this.move(polished.q);
    const existing = this.nodeNear(polished.q, 'meet');
    if (existing) {
      if (edge) this.markArrival(existing, this.heading);
      this.finishEdge(existing);
      this.continueFrom(existing);
      return { kind: 'revisit', node: existing.id };
    }
    const degTol = this.opts.degenerateTol >= 0 ? this.opts.degenerateTol : 3 * this.opts.nodeTol + this.opts.departStep;
    let k = m + 1;
    while (k < nr.length && nr[k].d - nr[0].d < degTol) k++;
    const deps = m === 2 && k > m + 1 ? degenerateDepartures2D(nr, k) : departureDirections(nr, m);
    const node = this.addNode(polished.q, 'meet', nr[0].d, deps);
    if (edge) this.markArrival(node, this.heading);
    this.finishEdge(node);
    this.continueFrom(node);
    return { kind: 'meet', node: node.id };
  }

  private tickBacktrack(): ExplorerEvent {
    const { alpha, dt } = this.opts;
    let budget = alpha * dt;
    while (budget > 0 && this.backtrackIndex < this.backtrackPath.length) {
      const target = this.backtrackPath[this.backtrackIndex];
      const d = vdist(this.q, target);
      if (d <= budget) {
        this.move(target.slice());
        budget -= d;
        this.backtrackIndex++;
      } else {
        this.move(vadd(this.q, vscale(vunit(vsub(target, this.q)), budget)));
        budget = 0;
      }
    }
    if (this.backtrackIndex >= this.backtrackPath.length) {
      const node = this.nodes[this.backtrackTarget];
      this.move(node.q.slice());
      this.currentNode = node.id;
      this.phase = 'depart';
    }
    return { kind: 'none' };
  }

  /** Run to completion (or budget). */
  run(maxTicks = this.opts.maxTicks): this {
    while (this.phase !== 'done' && this.phase !== 'stuck' && this.ticks < maxTicks) this.tick();
    return this;
  }

  /** Total length of the finished edges. */
  get roadmapLength(): number {
    return this.edges.filter((e) => e.to >= 0).reduce((s, e) => s + e.length, 0);
  }
}

/** Convenience: explore from q0 and return the explorer in its final state. */
export function exploreGvg(query: NearestQuery, q0: Vec, opts: ExplorerOptions = {}, heading0?: Vec): GvgExplorer {
  return new GvgExplorer(query, q0, opts, heading0).run();
}
