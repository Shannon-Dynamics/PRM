/**
 * Planning with a simulator: Choset §7.5.1's forward propagation, in full, and
 * Kinodynamic RRT* as RRT* with an exact optimal-control steer.
 *
 * `KinoRrt` is LaValle & Kuffner's tree [C refs. 270–272]: sample a state,
 * find the nearest node under a *metric* (the planner's only notion of
 * "close", and the knob it is most sensitive to), apply k sampled controls for
 * Δt through prop(x, u, Δt), keep the free result closest to the sample. It
 * never steers *to* anything: k = 1 is the classic single random control,
 * k = 8 is best-of-8. Chapter 14's `KinoRrtPreview` was a one-page sketch on
 * the kinematic car; this file owns the `Propagate`-with-controls interface,
 * the dynamic car (acceleration and steering-rate inputs, so velocity and
 * steering angle are *state*), and the reintegration test.
 *
 * `KinoRrtStar` is the other half of the honesty item "optimality needs
 * steering". RRT*'s rewiring compares cost-to-come through different parents,
 * which needs the exact optimal cost and trajectory between two states — a
 * two-point boundary-value problem. Two exact steers are provided:
 *
 *   • Reeds–Shepp for the reduced kinematic car — the shortest path *is* the
 *     optimal control for |v| ≤ 1, |ω| ≤ |v|/ρ (Karaman & Frazzoli used Dubins
 *     the same way);
 *   • Webb & van den Berg's fixed-final-state, free-final-time LQR edge for the
 *     double integrator, cost ∫(1 + uᵀRu)dt, in closed form per axis.
 *
 * Neither is available for Hitch's *dynamic* model; linearizing it about rest
 * and using the LQR edge is Exercise 6.
 */

import { angleDiff, normalizeAngle, type Pose2 } from '../geom/se2';
import type { Rng } from '../prob/rng';
import type { Propagate } from '../multi/propagate';
import { reedsSheppShortestPath, rsSample, rsTruncate, rsEndpoint } from './reeds-shepp';

/* -------------------------------------------------------------------------- */
/* Systems                                                                      */
/* -------------------------------------------------------------------------- */

/** Chapter 14's `Propagate` plus the control sampler forward propagation needs. */
export interface KinoSystem<S, U> extends Propagate<S, U> {
  sampleControl(rng: Rng): U;
}

/** Hitch's dynamic model: v and φ are state; inputs are acceleration and steering rate. */
export interface DynCar {
  x: number;
  y: number;
  theta: number;
  v: number;
  phi: number;
}

export interface DynCarControl {
  acc: number;
  steerRate: number;
}

export interface DynCarParams {
  wheelbase: number;
  maxSteer: number;
  maxSpeed: number;
  maxAcc: number;
  maxSteerRate: number;
  /** Allow reverse speeds. */
  reverse: boolean;
}

export const HITCH_DYNAMIC: DynCarParams = { wheelbase: 2.6, maxSteer: 0.6, maxSpeed: 2.0, maxAcc: 1.5, maxSteerRate: 0.8, reverse: true };

/**
 * ẋ = v cos θ, ẏ = v sin θ, θ̇ = v tan φ / L, v̇ = a, φ̇ = σ — Chapter 2's car with
 * a dynamic extension (Choset §12.5.6: "dynamic extensions of these models,
 * where the inputs are accelerations"). RK4, then speed and steering clamped to
 * their limits (saturating actuators), which keeps prop Lipschitz.
 */
export function dynCarSystem(p: DynCarParams = HITCH_DYNAMIC, substeps = 4): KinoSystem<DynCar, DynCarControl> {
  const f = (s: number[], u: DynCarControl) => [s[3] * Math.cos(s[2]), s[3] * Math.sin(s[2]), (s[3] * Math.tan(s[4])) / p.wheelbase, u.acc, u.steerRate];
  const vmin = p.reverse ? -p.maxSpeed : 0;
  return {
    propagate(q, u, dt) {
      let s = [q.x, q.y, q.theta, q.v, q.phi];
      const h = dt / substeps;
      for (let i = 0; i < substeps; i++) {
        const k1 = f(s, u);
        const k2 = f(s.map((x, j) => x + (h / 2) * k1[j]), u);
        const k3 = f(s.map((x, j) => x + (h / 2) * k2[j]), u);
        const k4 = f(s.map((x, j) => x + h * k3[j]), u);
        s = s.map((x, j) => x + (h / 6) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]));
        s[3] = Math.max(vmin, Math.min(p.maxSpeed, s[3]));
        s[4] = Math.max(-p.maxSteer, Math.min(p.maxSteer, s[4]));
      }
      return { x: s[0], y: s[1], theta: normalizeAngle(s[2]), v: s[3], phi: s[4] };
    },
    sampleControl(rng) {
      return { acc: rng.uniform(-p.maxAcc, p.maxAcc), steerRate: rng.uniform(-p.maxSteerRate, p.maxSteerRate) };
    },
  };
}

/** The weighted metric √(Δx² + Δy² + (w_θΔθ)² + (w_vΔv)²) — the knob w21.5 exposes. */
export function dynCarMetric(wTheta: number, wV: number): (a: DynCar, b: DynCar) => number {
  return (a, b) => Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (wTheta * angleDiff(a.theta, b.theta)) ** 2 + (wV * (a.v - b.v)) ** 2);
}

/* -------------------------------------------------------------------------- */
/* Forward-propagation RRT                                                      */
/* -------------------------------------------------------------------------- */

export interface KinoProblem<S> {
  start: S;
  isGoal(s: S): boolean;
  /** A random state (the RRT's q_rand); goal bias is applied by the planner. */
  sample(rng: Rng): S;
  goalSample(rng: Rng): S;
  metric(a: S, b: S): number;
  isFree(s: S): boolean;
}

export interface KinoNode<S, U> {
  s: S;
  parent: number;
  u: U | null;
  /** States along the edge into this node, substep by substep (root: []). */
  seg: S[];
}

export interface KinoRrtOptions {
  controlsPerExtend: number;
  dt: number;
  /** Collision checks per propagation. */
  substeps: number;
  goalBias: number;
}

export class KinoRrt<S, U> {
  readonly nodes: KinoNode<S, U>[];
  iterations = 0;
  propagations = 0;
  wasted = 0;
  goalNode: number | null = null;
  solvedAt: number | null = null;

  constructor(
    readonly sys: KinoSystem<S, U>,
    readonly prob: KinoProblem<S>,
    readonly opts: KinoRrtOptions,
    readonly rng: Rng,
  ) {
    this.nodes = [{ s: prob.start, parent: -1, u: null, seg: [] }];
  }

  get solved(): boolean {
    return this.goalNode !== null;
  }

  nearest(q: S): number {
    let best = 0;
    let bd = Infinity;
    for (let i = 0; i < this.nodes.length; i++) {
      const d = this.prob.metric(this.nodes[i].s, q);
      if (d < bd) {
        bd = d;
        best = i;
      }
    }
    return best;
  }

  /** Apply u for Δt in `substeps` pieces, checking each; null on collision. */
  propagateChecked(s: S, u: U): S[] | null {
    this.propagations++;
    const h = this.opts.dt / this.opts.substeps;
    const out: S[] = [];
    let cur = s;
    for (let k = 0; k < this.opts.substeps; k++) {
      cur = this.sys.propagate(cur, u, h);
      if (!this.prob.isFree(cur)) {
        this.wasted++;
        return null;
      }
      out.push(cur);
    }
    return out;
  }

  /** One extension: sample, nearest under the metric, best of k propagated controls. */
  extend(): { rand: S; near: number; added: number | null } {
    this.iterations++;
    const rand = this.rng.next() < this.opts.goalBias ? this.prob.goalSample(this.rng) : this.prob.sample(this.rng);
    const near = this.nearest(rand);
    let best: { u: U; seg: S[]; d: number } | null = null;
    for (let k = 0; k < this.opts.controlsPerExtend; k++) {
      const u = this.sys.sampleControl(this.rng);
      const seg = this.propagateChecked(this.nodes[near].s, u);
      if (!seg) continue;
      const d = this.prob.metric(seg[seg.length - 1], rand);
      if (!best || d < best.d) best = { u, seg, d };
    }
    if (!best) return { rand, near, added: null };
    const id = this.nodes.length;
    this.nodes.push({ s: best.seg[best.seg.length - 1], parent: near, u: best.u, seg: best.seg });
    if (this.goalNode === null) {
      for (const s of best.seg) {
        if (this.prob.isGoal(s)) {
          this.goalNode = id;
          this.solvedAt = this.iterations;
          break;
        }
      }
    }
    return { rand, near, added: id };
  }

  plan(n: number): boolean {
    for (let i = 0; i < n && !this.solved; i++) this.extend();
    return this.solved;
  }

  chain(id: number): number[] {
    const out: number[] = [];
    for (let c = id; c >= 0; c = this.nodes[c].parent) out.push(c);
    return out.reverse();
  }

  /** Dense states along the solution. */
  densePath(): S[] | null {
    if (this.goalNode === null) return null;
    const out: S[] = [this.prob.start];
    for (const i of this.chain(this.goalNode).slice(1)) out.push(...this.nodes[i].seg);
    return out;
  }

  /** Re-run the simulator from the root with the stored controls (must agree bit for bit). */
  reintegrate(): S[] | null {
    if (this.goalNode === null) return null;
    const h = this.opts.dt / this.opts.substeps;
    const out: S[] = [this.prob.start];
    let cur = this.prob.start;
    for (const i of this.chain(this.goalNode).slice(1)) {
      for (let k = 0; k < this.opts.substeps; k++) {
        cur = this.sys.propagate(cur, this.nodes[i].u!, h);
        out.push(cur);
      }
    }
    return out;
  }
}

/* -------------------------------------------------------------------------- */
/* Exact steers                                                                 */
/* -------------------------------------------------------------------------- */

export interface OptimalSteer<S> {
  /** Exact optimal cost of the two-point problem a → b. */
  cost(a: S, b: S): number;
  /** A cheap lower bound on cost (for pruning the near set). */
  lowerBound(a: S, b: S): number;
  /** States along the optimal trajectory at resolution ~ds. */
  trajectory(a: S, b: S, ds: number): S[];
  /** The state reached after spending `budget` of cost toward b (the RRT "extend"). */
  toward(a: S, b: S, budget: number): S;
}

/** Reeds–Shepp as the exact steer of the reduced car (cost = path length). */
export function reedsSheppSteer(rho: number): OptimalSteer<Pose2> {
  return {
    cost: (a, b) => reedsSheppShortestPath(a, b, rho)?.length ?? Infinity,
    lowerBound: (a, b) => Math.hypot(a.x - b.x, a.y - b.y),
    trajectory: (a, b, ds) => {
      const p = reedsSheppShortestPath(a, b, rho);
      return p ? rsSample(p, a, ds).map((s) => s.q) : [a];
    },
    toward: (a, b, budget) => {
      const p = reedsSheppShortestPath(a, b, rho);
      if (!p) return a;
      if (p.length <= budget) return b;
      return rsEndpoint(rsTruncate(p, budget), a);
    },
  };
}

/** Double-integrator state in the plane: position and velocity. */
export type DiState = [number, number, number, number];

/**
 * Webb & van den Berg's closed form for ẍ = u, cost c[τ] = ∫₀^τ (1 + r‖u‖²) dt:
 * for a fixed arrival time τ the minimum-energy cost is
 *   c(τ) = τ + dᵀG(τ)⁻¹d,  d = x₁ − e^{Aτ}x₀,  G(τ) = (1/r)[[τ³/3, τ²/2], [τ²/2, τ]]
 * per axis, i.e. τ + r Σ (12 Δp²/τ³ − 12 Δp Δv/τ² + 4 Δv²/τ) with Δp = p₁ − p₀ − v₀τ,
 * Δv = v₁ − v₀; the free final time τ* minimizes it (a 1-D search here).
 */
export function diCostAt(a: DiState, b: DiState, tau: number, r: number): number {
  let c = tau;
  for (let ax = 0; ax < 2; ax++) {
    const dp = b[ax] - a[ax] - a[ax + 2] * tau;
    const dv = b[ax + 2] - a[ax + 2];
    c += r * ((12 * dp * dp) / tau ** 3 - (12 * dp * dv) / tau ** 2 + (4 * dv * dv) / tau);
  }
  return c;
}

export function diOptimalTime(a: DiState, b: DiState, r: number): { tau: number; cost: number } {
  // Coarse geometric scan, then golden section on the bracketing interval.
  let bestT = 1;
  let bestC = Infinity;
  const grid: number[] = [];
  for (let k = 0; k <= 120; k++) grid.push(1e-2 * Math.pow(1e4, k / 120));
  for (const t of grid) {
    const c = diCostAt(a, b, t, r);
    if (c < bestC) {
      bestC = c;
      bestT = t;
    }
  }
  let lo = bestT / Math.pow(1e4, 1 / 120);
  let hi = bestT * Math.pow(1e4, 1 / 120);
  const g = (Math.sqrt(5) - 1) / 2;
  let x1 = hi - g * (hi - lo);
  let x2 = lo + g * (hi - lo);
  let f1 = diCostAt(a, b, x1, r);
  let f2 = diCostAt(a, b, x2, r);
  for (let i = 0; i < 80; i++) {
    if (f1 < f2) {
      hi = x2;
      x2 = x1;
      f2 = f1;
      x1 = hi - g * (hi - lo);
      f1 = diCostAt(a, b, x1, r);
    } else {
      lo = x1;
      x1 = x2;
      f1 = f2;
      x2 = lo + g * (hi - lo);
      f2 = diCostAt(a, b, x2, r);
    }
  }
  const tau = 0.5 * (lo + hi);
  return { tau, cost: diCostAt(a, b, tau, r) };
}

/** The optimal control u(t) = R⁻¹Bᵀe^{Aᵀ(τ−t)}G(τ)⁻¹d for the fixed-τ problem. */
export function diControl(a: DiState, b: DiState, tau: number, r: number): (t: number) => [number, number] {
  const lam: [number, number][] = [];
  for (let ax = 0; ax < 2; ax++) {
    const dp = b[ax] - a[ax] - a[ax + 2] * tau;
    const dv = b[ax + 2] - a[ax + 2];
    lam.push([r * ((12 * dp) / tau ** 3 - (6 * dv) / tau ** 2), r * ((-6 * dp) / tau ** 2 + (4 * dv) / tau)]);
  }
  return (t) => [((tau - t) * lam[0][0] + lam[0][1]) / r, ((tau - t) * lam[1][0] + lam[1][1]) / r];
}

/** Integrate ẍ = u(t) by RK4 — the check that the closed form lands on b. */
export function diIntegrate(a: DiState, u: (t: number) => [number, number], tau: number, steps = 400): DiState[] {
  let s = a.slice() as number[];
  const out: DiState[] = [s.slice() as DiState];
  const h = tau / steps;
  const f = (t: number, x: number[]) => {
    const [ux, uy] = u(t);
    return [x[2], x[3], ux, uy];
  };
  for (let i = 0; i < steps; i++) {
    const t = i * h;
    const k1 = f(t, s);
    const k2 = f(t + h / 2, s.map((x, j) => x + (h / 2) * k1[j]));
    const k3 = f(t + h / 2, s.map((x, j) => x + (h / 2) * k2[j]));
    const k4 = f(t + h, s.map((x, j) => x + h * k3[j]));
    s = s.map((x, j) => x + (h / 6) * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]));
    out.push(s.slice() as DiState);
  }
  return out;
}

export function doubleIntegratorSteer(r: number): OptimalSteer<DiState> {
  return {
    cost: (a, b) => diOptimalTime(a, b, r).cost,
    // No cheap bound is used: every candidate gets the exact 1-D minimization (fine for the checks' tree sizes).
    lowerBound: () => 0,
    trajectory: (a, b, ds) => {
      const { tau } = diOptimalTime(a, b, r);
      const n = Math.max(2, Math.ceil(tau / Math.max(1e-3, ds)));
      return diIntegrate(a, diControl(a, b, tau, r), tau, n);
    },
    toward: (a, b) => b,
  };
}

/* -------------------------------------------------------------------------- */
/* Kinodynamic RRT*                                                             */
/* -------------------------------------------------------------------------- */

export interface KinoStarOptions<S> {
  steer: OptimalSteer<S>;
  /** Longest edge (cost units); also the extend budget. */
  eta: number;
  /** γ in r_n = γ (log n / n)^{1/d}. */
  gamma: number;
  dim: number;
  goalBias: number;
  /** Collision resolution along edges. */
  ds: number;
}

export interface KinoStarNode<S> {
  s: S;
  parent: number;
  cost: number;
  children: number[];
}

export class KinoRrtStar<S> {
  readonly nodes: KinoStarNode<S>[];
  iterations = 0;
  rewires = 0;
  bestCost = Infinity;
  bestNode: number | null = null;
  /** (iteration, best cost) whenever the incumbent improves — w21.5's descending curve. */
  readonly costHistory: { n: number; cost: number }[] = [];
  private readonly goalEdge = new Map<number, number>();

  constructor(
    readonly prob: { start: S; goal: S; sample(rng: Rng): S; isFree(s: S): boolean },
    readonly opts: KinoStarOptions<S>,
    readonly rng: Rng,
  ) {
    this.nodes = [{ s: prob.start, parent: -1, cost: 0, children: [] }];
  }

  private edgeFree(a: S, b: S): boolean {
    for (const s of this.opts.steer.trajectory(a, b, this.opts.ds)) if (!this.prob.isFree(s)) return false;
    return true;
  }

  private radius(): number {
    const n = this.nodes.length + 1;
    return Math.min(this.opts.eta, this.opts.gamma * Math.pow(Math.log(n) / n, 1 / this.opts.dim));
  }

  iterate(): void {
    this.iterations++;
    const st = this.opts.steer;
    const rand = this.rng.next() < this.opts.goalBias ? this.prob.goal : this.prob.sample(this.rng);
    // Nearest by exact cost (lower bound prunes).
    let near = 0;
    let nd = Infinity;
    for (let i = 0; i < this.nodes.length; i++) {
      if (st.lowerBound(this.nodes[i].s, rand) >= nd) continue;
      const c = st.cost(this.nodes[i].s, rand);
      if (c < nd) {
        nd = c;
        near = i;
      }
    }
    const xNew = st.toward(this.nodes[near].s, rand, this.opts.eta);
    if (!this.prob.isFree(xNew)) return;
    const r = Math.max(this.radius(), 1e-9);
    // Near set: nodes whose exact cost *into* x_new is within r.
    const cand: { i: number; c: number }[] = [];
    for (let i = 0; i < this.nodes.length; i++) {
      if (st.lowerBound(this.nodes[i].s, xNew) > r && i !== near) continue;
      const c = st.cost(this.nodes[i].s, xNew);
      if (c <= r || i === near) cand.push({ i, c });
    }
    cand.sort((a, b) => this.nodes[a.i].cost + a.c - (this.nodes[b.i].cost + b.c));
    let parent = -1;
    let edge = 0;
    for (const c of cand) {
      if (this.edgeFree(this.nodes[c.i].s, xNew)) {
        parent = c.i;
        edge = c.c;
        break;
      }
    }
    if (parent < 0) return;
    const id = this.nodes.length;
    this.nodes.push({ s: xNew, parent, cost: this.nodes[parent].cost + edge, children: [] });
    this.nodes[parent].children.push(id);
    // Rewire: cost *out of* x_new (the steer is asymmetric in general).
    for (const c of cand) {
      if (c.i === parent || c.i === 0) continue;
      const out = st.cost(xNew, this.nodes[c.i].s);
      const through = this.nodes[id].cost + out;
      if (through < this.nodes[c.i].cost - 1e-9 && this.edgeFree(xNew, this.nodes[c.i].s)) {
        this.reparent(c.i, id, out);
        this.rewires++;
      }
    }
    // Goal connection.
    const cg = st.cost(xNew, this.prob.goal);
    if (cg <= this.opts.eta && this.edgeFree(xNew, this.prob.goal)) this.goalEdge.set(id, cg);
    this.refreshBest();
  }

  private reparent(v: number, p: number, edge: number): void {
    const old = this.nodes[v].parent;
    this.nodes[old].children = this.nodes[old].children.filter((c) => c !== v);
    this.nodes[v].parent = p;
    this.nodes[p].children.push(v);
    const delta = this.nodes[p].cost + edge - this.nodes[v].cost;
    const stack = [v];
    while (stack.length) {
      const u = stack.pop()!;
      this.nodes[u].cost += delta;
      stack.push(...this.nodes[u].children);
    }
  }

  private refreshBest(): void {
    let best = Infinity;
    let node: number | null = null;
    for (const [v, cg] of this.goalEdge) {
      const c = this.nodes[v].cost + cg;
      if (c < best) {
        best = c;
        node = v;
      }
    }
    if (node !== null && best < this.bestCost - 1e-9) {
      this.bestCost = best;
      this.bestNode = node;
      this.costHistory.push({ n: this.iterations, cost: best });
    }
  }

  /** The incumbent as a dense state sequence. */
  bestPath(): S[] | null {
    if (this.bestNode === null) return null;
    const ids: number[] = [];
    for (let c = this.bestNode; c >= 0; c = this.nodes[c].parent) ids.push(c);
    ids.reverse();
    const out: S[] = [];
    for (let k = 1; k < ids.length; k++) out.push(...this.opts.steer.trajectory(this.nodes[ids[k - 1]].s, this.nodes[ids[k]].s, this.opts.ds));
    out.push(...this.opts.steer.trajectory(this.nodes[this.bestNode].s, this.prob.goal, this.opts.ds));
    return out;
  }
}
