/**
 * Manipulation planning as transit and transfer modes — Choset §7.5.3, the
 * two-level PRM of Nielsen & Kavraki [C ref. 335].
 *
 * Reach moves a puck on the Workbench. The composite configuration space is
 * T² × ℝ² (arm × puck), but almost none of it is reachable: an unheld puck
 * rests at a stable placement and cannot move, so the arm moves on a
 * *transit* manifold ℳ_transit(p) = T² × {p}; a held puck is a rigid transform
 * of the hand, so the system moves on a *transfer* manifold ℳ_transfer(g) =
 * {(q, c_g(q))} — a graph over T² for each grasp g. Both have the dimension of
 * the arm alone, and the only way from one to another is through their
 * intersection: a grasp configuration, the arm holding the puck *at* a
 * placement. Those intersections (p, g) are the nodes of the manipulation
 * graph MG; transit paths (regrasps at one placement) and transfer paths (one
 * grasp, two placements) are its edges.
 *
 * Why a puck and not a block: with a 2R arm the hand's heading is fixed by
 * (θ₁, θ₂), so a grasp that also prescribed the object's orientation would be
 * three constraints on two unknowns. A rotationally symmetric object needs
 * only its center placed, and then a grasp g — the puck's bearing α_g in the
 * hand frame — turns the arm-plus-puck into a 2R arm with a longer, bent last
 * link, whose closed-form IK is Chapter 2's `Reach.ik` on a virtual arm. The
 * design doc's block is this puck; the mode structure is identical.
 *
 * Lazy verification, FuzzyPRM-style: every MG edge first gets a *probability*
 * of being free from a handful of interpolated configurations; the upper level
 * searches MG by Dijkstra on −log p; only the edges on the candidate path are
 * planned for real (Chapter 12's RRT-Connect in that mode's free space); a
 * failed edge is deleted and the search repeats. The counter of verified vs
 * total edges is the number Exercise 4 asks about.
 */

import type { Rng } from '../prob/rng';
import { normalizeAngle } from '../geom/se2';
import type { Point2 } from '../sim/world';
import { Scene } from '../collide/scene';
import { ChainChecker } from '../collide/collision';
import { closestPointOnSegment, orientedRect } from '../collide/polygon';
import { Reach, REACH, type Tn } from '../robots/reach';
import { WORKBENCH, type PolyWorld } from '../robots/worlds';
import { T2 } from '../manifold/torus';
import { dijkstra } from '../search/best-first';
import type { Edge, Graph } from '../search/graph';
import { CountingFreeSpace, type FreeSpace } from '../sampling/free-space';
import { StraightLine } from '../sampling/steer';
import { RrtConnect } from '../sampling/tree';

/* -------------------------------------------------------------------------- */
/* The scene                                                                   */
/* -------------------------------------------------------------------------- */

/** The Workbench with the fixed block removed — the puck is the movable body now. */
const deg = (d: number) => (d * Math.PI) / 180;

/**
 * The bar: a 0.8 m rail across the puck's goal placement B, on the side facing
 * the base. The arm can only reach B around the bar's ends — tangentially — so
 * a grasp that holds the puck straight ahead of the forearm is geometrically
 * impossible there, while a side grasp is not. That asymmetry, not any rule in
 * the planner, is what makes regrasping necessary.
 */
export const MANIP_WORLD: PolyWorld = {
  ...WORKBENCH,
  name: 'Workbench (manipulation)',
  obstacles: [...WORKBENCH.obstacles.filter((o) => o.name !== 'the block'), { name: 'the bar', poly: orientedRect(1.05 * Math.cos(deg(100)), 1.05 * Math.sin(deg(100)), 0.4, 0.05, deg(190)) }],
  labels: [...(WORKBENCH.labels?.filter((l) => l.text !== 'block') ?? []), { x: 0.1, y: 0.95, text: 'bar' }],
};

export const PUCK_RADIUS = 0.15;

/** A grasp: the puck's bearing in the hand frame (0 = straight ahead of the tip). */
export interface Grasp {
  alpha: number;
  name: string;
}

/**
 * The grasp catalogue, in the order the widget's |𝒢| slider adds them: straight
 * ahead first, then the two side grasps, then the diagonals, then one reaching
 * back. Bearings beyond about ±116° put the puck onto the forearm itself
 * (self-collision: 0.2·sin|α| < r + link radius), so the catalogue stops there.
 */
export const GRASP_BEARINGS_DEG: readonly number[] = [0, 90, -90, 45, -45, 110];
const GRASP_NAMES: readonly string[] = ['front', 'left', 'right', 'front-left', 'front-right', 'back-left'];

/** The first k grasps of the catalogue. */
export function graspSet(k: number): Grasp[] {
  const n = Math.max(1, Math.min(GRASP_BEARINGS_DEG.length, k));
  return GRASP_BEARINGS_DEG.slice(0, n).map((d, i) => ({ alpha: (d * Math.PI) / 180, name: GRASP_NAMES[i] }));
}

/** A stable placement: the puck's center at rest on the table. */
export interface Placement {
  x: number;
  y: number;
  name: string;
}

/* -------------------------------------------------------------------------- */
/* The hand–puck geometry                                                      */
/* -------------------------------------------------------------------------- */

export class PuckArm {
  /** Hand-frame offset from the tip to the held puck's center: r + link radius + a little air. */
  readonly holdDistance: number;

  constructor(
    readonly arm: Reach = REACH,
    readonly puckRadius: number = PUCK_RADIUS,
    readonly scene: Scene = Scene.fromWorld(MANIP_WORLD),
  ) {
    this.holdDistance = puckRadius + arm.linkRadius + 0.02;
  }

  /** c_g(q): where a puck held with grasp g is when the arm is at q — a rigid transform of the hand. */
  heldCenter(q: Tn, g: Grasp): Point2 {
    const tip = this.arm.tip(q);
    const heading = this.arm.tipHeading(q) + g.alpha;
    return { x: tip.x + this.holdDistance * Math.cos(heading), y: tip.y + this.holdDistance * Math.sin(heading) };
  }

  /**
   * The grasp configurations at (p, g): the arm's two IK branches for a virtual
   * 2R arm whose last link is link 2 extended by the hold offset — the vector
   * w = (L₂, 0) + d_g (cos α, sin α) in the forearm frame — so that the virtual
   * tip is the puck's center. θ₂ = θ₂′ − atan2(w).
   */
  graspConfigs(p: Placement, g: Grasp): Tn[] {
    const [L1, L2] = this.arm.lengths;
    const wx = L2 + this.holdDistance * Math.cos(g.alpha);
    const wy = this.holdDistance * Math.sin(g.alpha);
    const virt = this.arm.withLengths([L1, Math.hypot(wx, wy)]);
    const beta = Math.atan2(wy, wx);
    const sols = virt.ik({ x: p.x, y: p.y });
    const out: Tn[] = [];
    for (const s of sols) if (s) out.push([s[0], normalizeAngle(s[1] - beta)]);
    return out;
  }

  /** Smallest distance from a point to any link's axis (the puck must clear the arm's capsules). */
  private linkClearance(q: Tn, c: Point2, skipLast = false): number {
    let best = Infinity;
    const segs = this.arm.linkSegments(q);
    for (let i = 0; i < segs.length - (skipLast ? 1 : 0); i++) {
      const cp = closestPointOnSegment(c.x, c.y, segs[i]);
      best = Math.min(best, Math.hypot(cp.x - c.x, cp.y - c.y));
    }
    return best;
  }

  /** Is a puck at rest at `c` a legal placement: on the table, clear of the obstacles? */
  placementFree(c: Point2): boolean {
    return this.scene.inBounds(c) && this.scene.distancePoint(c.x, c.y).dist > this.puckRadius;
  }

  /**
   * ℳ_transit(p): the arm must clear the walls, the fixed obstacles and the
   * resting puck. The puck is an obstacle like any other here — the amber
   * C-obstacle on the transit torus is the Chapter 4 picture with one more body.
   */
  transitFree(p: Placement): CountingFreeSpace<number[]> {
    const chain = new ChainChecker(this.scene, this.arm);
    const clear = this.puckRadius + this.arm.linkRadius;
    return new CountingFreeSpace<number[]>((q) => chain.isFree(q) && this.linkClearance(q, p) > clear);
  }

  /**
   * ℳ_transfer(g): the arm clears the scene as before, *and* the held puck
   * clears the scene and the arm's own links. The robot's shape has changed —
   * the C-obstacle on the transfer torus is a different amber region.
   */
  transferFree(g: Grasp): CountingFreeSpace<number[]> {
    const chain = new ChainChecker(this.scene, this.arm);
    const clear = this.puckRadius + this.arm.linkRadius;
    return new CountingFreeSpace<number[]>((q) => {
      if (!chain.isFree(q)) return false;
      const c = this.heldCenter(q, g);
      if (!this.placementFree(c)) return false;
      // The puck hangs d_g off the tip, so link 2's clearance is d_g by construction unless α ≈ π; check every link anyway.
      return this.linkClearance(q, c) > clear - 1e-9;
    });
  }

  /** Both modes at once: the configuration is a legal node of MG only if it is free in each. */
  graspFeasible(q: Tn, p: Placement, g: Grasp): boolean {
    return this.transitFree(p).isFree([...q]) && this.transferFree(g).isFree([...q]);
  }
}

/* -------------------------------------------------------------------------- */
/* The manipulation graph                                                      */
/* -------------------------------------------------------------------------- */

export type Mode = { kind: 'transit'; placement: number } | { kind: 'transfer'; grasp: number };

export interface MgNode {
  id: number;
  /** Placement index. */
  p: number;
  /** Grasp index, or −1 for the arm's free start configuration (puck at rest, nothing held). */
  g: number;
  /** IK branch (0 elbow-down, 1 elbow-up), or −1 for the start node. */
  branch: number;
  q: Tn;
  label: string;
}

export interface MgEdge {
  id: number;
  a: number;
  b: number;
  mode: Mode;
  /** FuzzyPRM's belief that the edge is free, before verification. */
  belief: number;
  status: 'unverified' | 'verified' | 'failed';
  /** The motion plan once verified: configurations from a to b. */
  path: Tn[] | null;
  /** Collision checks the verification spent. */
  checks: number;
}

export interface ManipSegment {
  mode: Mode;
  from: number;
  to: number;
  path: Tn[];
  /** The puck's center along the path: constant on a transit edge, c_g(q) on a transfer edge. */
  puck: Point2[];
}

export interface ManipPlan {
  segments: ManipSegment[];
  nodes: number[];
  /** Edges planned for real / edges that failed / edges in MG. */
  verified: number;
  failed: number;
  totalEdges: number;
  /** Collision checks spent by edge verification. */
  checks: number;
  /** Upper-level searches run before a fully verified path appeared. */
  rounds: number;
}

export interface ManipOptions {
  /** RRT-Connect attempts per edge. */
  attempts: number;
  /** Interpolated configurations used for the prior belief. */
  beliefSamples: number;
  /** Verify every edge first (eager) instead of only the ones on candidate paths (lazy). */
  eager: boolean;
  eta: number;
  checkStep: number;
}

export const DEFAULT_MANIP: ManipOptions = { attempts: 1500, beliefSamples: 12, eager: false, eta: 0.35, checkStep: 0.04 };

/** MG as a Chapter 6 graph with −log(belief) weights; failed edges are gone, verified ones nearly free. */
class MgGraph implements Graph<number> {
  constructor(
    readonly adj: Map<number, MgEdge[]>,
    readonly edges: MgEdge[],
  ) {}

  neighbors(n: number): Edge<number>[] {
    const out: Edge<number>[] = [];
    for (const e of this.adj.get(n) ?? []) {
      if (e.status === 'failed') continue;
      const other = e.a === n ? e.b : e.a;
      // A verified edge still costs one step, so the path prefers fewer mode switches; an unverified one costs its doubt too.
      const w = 1 + (e.status === 'verified' ? 0 : -Math.log(Math.max(1e-6, e.belief)));
      out.push({ to: other, cost: w });
    }
    return out;
  }

  key(n: number): number {
    return n;
  }
}

/**
 * The two-level planner. Level one: MG over placements × grasps × IK branches,
 * every edge believed free with the FuzzyPRM prior. Level two: Chapter 12's
 * RRT-Connect on T² in the edge's own mode, run lazily.
 */
export class ManipulationPlanner {
  readonly nodes: MgNode[] = [];
  readonly edges: MgEdge[] = [];
  private readonly adj = new Map<number, MgEdge[]>();
  readonly startNode: number;
  readonly goalNodes: number[] = [];
  /** Placements that produced no feasible grasp configuration at all. */
  readonly unreachable: number[] = [];

  constructor(
    readonly pa: PuckArm,
    readonly placements: readonly Placement[],
    readonly grasps: readonly Grasp[],
    readonly qStart: Tn,
    readonly pStart: number,
    readonly pGoal: number,
    readonly rng: Rng,
    readonly opts: ManipOptions = DEFAULT_MANIP,
  ) {
    // Nodes: the start, then every feasible (p, g, branch).
    this.startNode = this.addNode({ p: pStart, g: -1, branch: -1, q: qStart, label: 'start (free hand)' });
    for (let pi = 0; pi < placements.length; pi++) {
      let any = false;
      for (let gi = 0; gi < grasps.length; gi++) {
        const cfgs = pa.graspConfigs(placements[pi], grasps[gi]);
        cfgs.forEach((q, branch) => {
          if (!pa.graspFeasible(q, placements[pi], grasps[gi])) return;
          any = true;
          const id = this.addNode({ p: pi, g: gi, branch, q, label: `(${placements[pi].name}, ${grasps[gi].name}${cfgs.length > 1 ? (branch === 0 ? ', elbow-down' : ', elbow-up') : ''})` });
          if (pi === pGoal) this.goalNodes.push(id);
        });
      }
      if (!any) this.unreachable.push(pi);
    }
    // Edges: transit between nodes sharing a placement (including the start node at p_start); transfer between nodes sharing a grasp.
    for (let i = 0; i < this.nodes.length; i++) {
      for (let j = i + 1; j < this.nodes.length; j++) {
        const a = this.nodes[i];
        const b = this.nodes[j];
        if (a.p === b.p) this.addEdge(i, j, { kind: 'transit', placement: a.p });
        if (a.g >= 0 && a.g === b.g && a.p !== b.p) this.addEdge(i, j, { kind: 'transfer', grasp: a.g });
      }
    }
    if (opts.eager) for (const e of this.edges) this.verify(e);
  }

  private addNode(n: Omit<MgNode, 'id'>): number {
    const id = this.nodes.length;
    this.nodes.push({ id, ...n });
    this.adj.set(id, []);
    return id;
  }

  private addEdge(a: number, b: number, mode: Mode): void {
    const e: MgEdge = { id: this.edges.length, a, b, mode, belief: this.prior(a, b, mode), status: 'unverified', path: null, checks: 0 };
    this.edges.push(e);
    this.adj.get(a)!.push(e);
    this.adj.get(b)!.push(e);
  }

  freeSpaceFor(mode: Mode): CountingFreeSpace<number[]> {
    return mode.kind === 'transit' ? this.pa.transitFree(this.placements[mode.placement]) : this.pa.transferFree(this.grasps[mode.grasp]);
  }

  /** FuzzyPRM's prior: the share of a few geodesic samples that are free, Laplace-smoothed so nothing is ever ruled out unseen. */
  private prior(a: number, b: number, mode: Mode): number {
    const free = this.freeSpaceFor(mode);
    const qa = [...this.nodes[a].q];
    const qb = [...this.nodes[b].q];
    const n = this.opts.beliefSamples;
    let ok = 0;
    for (let k = 1; k <= n; k++) if (free.isFree(T2.interpolate(qa, qb, k / (n + 1)))) ok++;
    return (ok + 1) / (n + 2);
  }

  /** Level two: plan the edge for real in its mode's free space. */
  verify(e: MgEdge): boolean {
    if (e.status !== 'unverified') return e.status === 'verified';
    const free = this.freeSpaceFor(e.mode);
    const qa = [...this.nodes[e.a].q];
    const qb = [...this.nodes[e.b].q];
    const before = free.checks;
    const planner = new RrtConnect<number[]>(T2, free, qa, qb, { steer: new StraightLine<number[]>({ kind: 'subdivision', step: this.opts.checkStep }), eta: this.opts.eta, greed: ['extend', 'connect'] }, this.rng.fork());
    const path = planner.merge(this.opts.attempts);
    e.checks = free.checks - before;
    if (path) {
      e.status = 'verified';
      e.belief = 1;
      e.path = path;
      return true;
    }
    e.status = 'failed';
    e.belief = 0;
    return false;
  }

  /** The current best path in MG from the start to any goal node, with unverified edges allowed. */
  search(): number[] | null {
    const g = new MgGraph(this.adj, this.edges);
    let best: { path: number[]; cost: number } | null = null;
    for (const goal of this.goalNodes) {
      const t = dijkstra(g, this.startNode, goal);
      if (t.status === 'found' && (!best || t.cost < best.cost)) best = { path: t.path, cost: t.cost };
    }
    return best?.path ?? null;
  }

  private edgeBetween(a: number, b: number): MgEdge | undefined {
    return (this.adj.get(a) ?? []).find((e) => (e.a === a && e.b === b) || (e.a === b && e.b === a));
  }

  /**
   * Search, verify along the path, delete what fails, repeat. Returns null
   * when MG has no start–goal path left — with one grasp and the wrong
   * geometry, that is the disconnected manipulation graph of Exercise 4.
   */
  plan(maxRounds = 50): ManipPlan | null {
    let rounds = 0;
    while (rounds < maxRounds) {
      rounds++;
      const path = this.search();
      if (!path) return null;
      let allOk = true;
      for (let k = 0; k + 1 < path.length; k++) {
        const e = this.edgeBetween(path[k], path[k + 1])!;
        if (!this.verify(e)) {
          allOk = false;
          break;
        }
      }
      if (!allOk) continue;
      const segments: ManipSegment[] = [];
      for (let k = 0; k + 1 < path.length; k++) {
        const e = this.edgeBetween(path[k], path[k + 1])!;
        let qs = e.path!;
        if (e.a !== path[k]) qs = qs.slice().reverse();
        const mode = e.mode;
        const puck = qs.map((q) => (mode.kind === 'transit' ? { x: this.placements[mode.placement].x, y: this.placements[mode.placement].y } : this.pa.heldCenter(q, this.grasps[mode.grasp])));
        segments.push({ mode, from: path[k], to: path[k + 1], path: qs, puck });
      }
      return { segments, nodes: path, verified: this.edges.filter((e) => e.status === 'verified').length, failed: this.edges.filter((e) => e.status === 'failed').length, totalEdges: this.edges.length, checks: this.edges.reduce((s, e) => s + e.checks, 0), rounds };
    }
    return null;
  }

  /** Is any goal node reachable from the start through edges not yet failed? (Connectivity of MG, before any planning.) */
  connected(): boolean {
    return this.search() !== null;
  }
}

/* -------------------------------------------------------------------------- */
/* The chapter's instance                                                      */
/* -------------------------------------------------------------------------- */

/**
 * Placements on the Workbench. A sits 2.1 m from the base, at the very edge of
 * what the arm can reach — only by holding the puck *ahead* of the forearm
 * (reach 1 + 1.2) does the arm get there; B sits behind the bar, where only a
 * side grasp fits; C is open table in between, where the puck can be set down
 * and picked up again with a different grasp. A → B therefore needs a regrasp,
 * and with the front grasp alone the manipulation graph is disconnected.
 */
export const PLACEMENTS: Placement[] = [
  { x: 2.1 * Math.cos(deg(-10)), y: 2.1 * Math.sin(deg(-10)), name: 'A' },
  { x: 1.4 * Math.cos(deg(100)), y: 1.4 * Math.sin(deg(100)), name: 'B' },
  { x: 0.9, y: 1.1, name: 'C' },
];

/** Reach's resting configuration before the task. */
export const MANIP_START: Tn = [-Math.PI / 2, Math.PI / 3];

/**
 * The scenario the widget and the checks run: puck from A to B, |𝒢| grasps,
 * with or without the regrasp placement C.
 */
export function manipScenario(k: number, withC: boolean, rng: Rng, opts: ManipOptions = DEFAULT_MANIP): ManipulationPlanner {
  const placements = withC ? PLACEMENTS : PLACEMENTS.slice(0, 2);
  return new ManipulationPlanner(new PuckArm(), placements, graspSet(k), MANIP_START, 0, 1, rng, opts);
}

/** Does a plan alternate modes, keep the grasp on every transfer, and keep the puck still on every transit? */
export function planIsConsistent(plan: ManipPlan, pa: PuckArm, placements: readonly Placement[], grasps: readonly Grasp[], tol = 1e-9): { alternates: boolean; rigid: boolean; still: boolean; switchesAtPlacements: boolean } {
  let alternates = true;
  let rigid = true;
  let still = true;
  let switchesAtPlacements = true;
  for (let k = 0; k < plan.segments.length; k++) {
    const s = plan.segments[k];
    if (k > 0 && plan.segments[k - 1].mode.kind === s.mode.kind) alternates = false;
    if (s.mode.kind === 'transfer') {
      const g = grasps[s.mode.grasp];
      s.path.forEach((q, i) => {
        const c = pa.heldCenter(q, g);
        if (Math.hypot(c.x - s.puck[i].x, c.y - s.puck[i].y) > tol) rigid = false;
      });
      // A transfer starts and ends with the puck exactly at a placement.
      const first = s.puck[0];
      const last = s.puck[s.puck.length - 1];
      const onP = (c: Point2) => placements.some((p) => Math.hypot(p.x - c.x, p.y - c.y) < 1e-6);
      if (!onP(first) || !onP(last)) switchesAtPlacements = false;
    } else {
      const p = placements[s.mode.placement];
      for (const c of s.puck) if (Math.hypot(c.x - p.x, c.y - p.y) > tol) still = false;
    }
  }
  return { alternates, rigid, still, switchesAtPlacements };
}

/** Re-check every waypoint and the geodesic between consecutive ones in the segment's own mode. */
export function planIsFree(plan: ManipPlan, planner: ManipulationPlanner, step = 0.01): boolean {
  for (const s of plan.segments) {
    const free: FreeSpace<number[]> = planner.freeSpaceFor(s.mode);
    for (let i = 0; i + 1 < s.path.length; i++) {
      const a = [...s.path[i]];
      const b = [...s.path[i + 1]];
      const n = Math.max(1, Math.ceil(T2.dist(a, b) / step));
      for (let k = 0; k <= n; k++) if (!free.isFree(T2.interpolate(a, b, k / n))) return false;
    }
  }
  return true;
}

/** The torus C-obstacle of a mode, rasterized by asking its checker — for the widget's two torus pictures. */
export function rasterMode(free: FreeSpace<number[]>, n = 64): Uint8Array {
  const out = new Uint8Array(n * n);
  for (let j = 0; j < n; j++) {
    for (let i = 0; i < n; i++) {
      const th1 = -Math.PI + ((i + 0.5) / n) * 2 * Math.PI;
      const th2 = -Math.PI + ((j + 0.5) / n) * 2 * Math.PI;
      if (!free.isFree([th1, th2])) out[j * n + i] = 1;
    }
  }
  return out;
}

export { Reach };
