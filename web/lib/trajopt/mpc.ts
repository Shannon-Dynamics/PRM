/**
 * Receding-horizon control around iLQR: each tick, solve a horizon-N_h problem
 * from the *measured* state, apply the first control, discard the rest, and
 * warm-start the next solve by shifting the previous plan one stage. The loop
 * is the whole content of "MPC"; what varies between papers is the solver
 * inside it and how the terminal cost is chosen. Stability via a terminal cost
 * that is a Lyapunov function is a pointer here, not a theorem proved.
 */

import type { Mat, Vec } from '../prob/linalg';
import { wavefront } from '../potential/brushfire';
import { cellAt, gridFromWorld, type OccupancyGrid } from '../search/grid';
import type { World } from '../sim/world';
import { ilqr, type IlqrOptions, type RunningDerivs, type StageCost, type TerminalDerivs } from './ilqr';
import type { DiscreteDynamics } from './models';
import type { DistanceField2 } from './field';

export interface MpcOptions {
  horizon: number;
  warmStart?: boolean;
  ilqr?: IlqrOptions;
}

export interface MpcTick {
  /** The control applied this tick. */
  u: Vec;
  /** The planned horizon, from the measured state. */
  plan: Vec[];
  controls: Vec[];
  cost: number;
  iterations: number;
  /** Wall-clock milliseconds of the solve. */
  solveMs: number;
}

export class Mpc {
  private warm: Vec[] | null = null;
  constructor(
    readonly f: DiscreteDynamics,
    /** The cost for a horizon starting at absolute tick `t0` (moving obstacles are time-indexed). */
    readonly costAt: (t0: number) => StageCost,
    readonly opts: MpcOptions,
  ) {}

  reset(): void {
    this.warm = null;
  }

  /** One receding-horizon step from the measured state. */
  tick(xMeasured: Vec, t0: number): MpcTick {
    const N = this.opts.horizon;
    const nu = this.f.nu;
    let uInit: Vec[];
    if (this.warm && (this.opts.warmStart ?? true)) {
      uInit = [...this.warm.slice(1), this.warm[this.warm.length - 1].slice()];
    } else {
      uInit = Array.from({ length: N }, () => new Array<number>(nu).fill(0));
    }
    const t = performance.now();
    const res = ilqr(this.f, this.costAt(t0), xMeasured, uInit, { maxIter: 6, mu0: 1e-3, ...this.opts.ilqr });
    const solveMs = performance.now() - t;
    this.warm = res.us;
    return { u: res.us[0], plan: res.xs, controls: res.us, cost: res.costs[res.costs.length - 1], iterations: res.iterations, solveMs };
  }
}

/** Convenience: a quadratic weight matrix from a diagonal. */
export const diag = (d: Vec): Mat => d.map((v, i) => d.map((_, j) => (i === j ? v : 0)));

/**
 * A cost-to-go from Chapter 6: the wave-front (breadth-first) distance to the
 * goal on an occupancy grid of the world, 8-connected, read back as metres by
 * bilinear interpolation. This is the terminal cost that lets a short-horizon
 * MPC see around a corner — Exercise 4's claim, and the standard remedy for
 * receding-horizon myopia. Unreachable cells carry a large constant.
 */
export class GridCostToGo {
  readonly grid: OccupancyGrid;
  private readonly value: Float64Array;
  constructor(world: World, goal: [number, number], cellSize = 0.1, inflation = 0.3) {
    this.grid = gridFromWorld(world, cellSize, inflation);
    const g = this.grid;
    let goalCell = cellAt(g, goal[0], goal[1]);
    if (g.occupied[goalCell]) {
      // Nearest free cell to the goal.
      let best = Infinity;
      for (let k = 0; k < g.occupied.length; k++) {
        if (g.occupied[k]) continue;
        const i = k % g.nx;
        const j = Math.floor(k / g.nx);
        const d = Math.hypot(g.origin.x + (i + 0.5) * cellSize - goal[0], g.origin.y + (j + 0.5) * cellSize - goal[1]);
        if (d < best) {
          best = d;
          goalCell = k;
        }
      }
    }
    const wf = wavefront(g, goalCell, 8);
    this.value = new Float64Array(g.nx * g.ny);
    const far = (g.nx + g.ny) * cellSize;
    for (let k = 0; k < this.value.length; k++) this.value[k] = wf.label[k] >= 2 ? (wf.label[k] - 2) * cellSize : far;
  }

  /** Cost-to-go in metres (chessboard), bilinear. */
  at(x: number, y: number): number {
    const g = this.grid;
    const u = (x - g.origin.x) / g.cellSize - 0.5;
    const w = (y - g.origin.y) / g.cellSize - 0.5;
    const i0 = Math.min(g.nx - 2, Math.max(0, Math.floor(u)));
    const j0 = Math.min(g.ny - 2, Math.max(0, Math.floor(w)));
    const fu = Math.min(1, Math.max(0, u - i0));
    const fv = Math.min(1, Math.max(0, w - j0));
    const v = (i: number, j: number) => this.value[j * g.nx + i];
    return (1 - fu) * (1 - fv) * v(i0, j0) + fu * (1 - fv) * v(i0 + 1, j0) + (1 - fu) * fv * v(i0, j0 + 1) + fu * fv * v(i0 + 1, j0 + 1);
  }
}

export interface NavigationCostOptions {
  goal: Vec;
  /** Terminal cost-to-go; null for a plain quadratic pull to the goal. */
  costToGo: GridCostToGo | null;
  /** Wall distance field and the robot radius for the wall penalty. */
  walls: DistanceField2;
  radius: number;
  /** Moving obstacle at the planning time (frozen over the horizon), or null. */
  obstacle: { x: number; y: number; r: number } | null;
  vMax: number;
  weights?: { goal?: number; terminal?: number; ctg?: number; obstacle?: number; wall?: number; effort?: [number, number]; speed?: number };
}

/**
 * Rusty's receding-horizon cost: effort, a soft speed bound (|v| − v_max)⁺², a
 * hinge-squared penalty inside the obstacle and wall margins, a small quadratic
 * pull to the goal, and — the part that matters — a terminal cost that is the
 * Chapter 6 cost-to-go squared when one is supplied. State derivatives are by
 * central differences (the fields are piecewise linear); control derivatives
 * are analytic.
 */
export function navigationCost(o: NavigationCostOptions): StageCost {
  const w = { goal: 0.02, terminal: 20, ctg: 12, obstacle: 600, wall: 600, effort: [0.1, 0.05] as [number, number], speed: 60, ...o.weights };
  const stateCost = (x: Vec): number => {
    let c = 0.5 * w.goal * ((x[0] - o.goal[0]) ** 2 + (x[1] - o.goal[1]) ** 2);
    const dWall = o.walls.dist({ x: x[0], y: x[1] }) - o.radius - 0.1;
    if (dWall < 0) c += w.wall * dWall * dWall;
    if (o.obstacle) {
      const d = Math.hypot(x[0] - o.obstacle.x, x[1] - o.obstacle.y) - o.obstacle.r - o.radius - 0.15;
      if (d < 0) c += w.obstacle * d * d;
    }
    return c;
  };
  const terminalCost = (x: Vec): number => {
    if (o.costToGo) {
      // Linear in the cost-to-go: a constant pull of w.ctg per metre, so the obstacle and wall hinges (quadratic, steep) always win inside their margins.
      return w.ctg * o.costToGo.at(x[0], x[1]) + stateCost(x);
    }
    return 0.5 * w.terminal * ((x[0] - o.goal[0]) ** 2 + (x[1] - o.goal[1]) ** 2) + stateCost(x);
  };
  const fd = (f: (x: Vec) => number, x: Vec): { l: number; lx: Vec; lxx: Mat } => {
    const h = 2e-3;
    const n = x.length;
    const l0 = f(x);
    const lx = new Array<number>(n).fill(0);
    const lxx: Mat = Array.from({ length: n }, () => new Array<number>(n).fill(0));
    const fp: number[] = [];
    for (let i = 0; i < n; i++) {
      const xp = x.slice();
      const xm = x.slice();
      xp[i] += h;
      xm[i] -= h;
      const a = f(xp);
      const b = f(xm);
      fp.push(a);
      lx[i] = (a - b) / (2 * h);
      lxx[i][i] = Math.max(0, (a - 2 * l0 + b) / (h * h));
    }
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const xpp = x.slice();
        xpp[i] += h;
        xpp[j] += h;
        const v = (f(xpp) - fp[i] - fp[j] + l0) / (h * h);
        lxx[i][j] = v;
        lxx[j][i] = v;
      }
    }
    return { l: l0, lx, lxx };
  };
  return {
    running(x: Vec, u: Vec): RunningDerivs {
      const s = fd(stateCost, x);
      const over = Math.max(0, Math.abs(u[0]) - o.vMax);
      const sgn = u[0] >= 0 ? 1 : -1;
      const l = s.l + 0.5 * (w.effort[0] * u[0] * u[0] + w.effort[1] * u[1] * u[1]) + w.speed * over * over;
      const lu = [w.effort[0] * u[0] + 2 * w.speed * over * sgn, w.effort[1] * u[1]];
      const luu = [
        [w.effort[0] + (over > 0 ? 2 * w.speed : 0), 0],
        [0, w.effort[1]],
      ];
      return { l, lx: s.lx, lu, lxx: s.lxx, luu, lux: [[0, 0, 0], [0, 0, 0]] };
    },
    terminal(x: Vec): TerminalDerivs {
      return fd(terminalCost, x);
    },
  };
}
