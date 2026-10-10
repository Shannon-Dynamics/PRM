/**
 * Gradient descent — Choset Algorithm 4, with the sign the text intends.
 *
 * Line 4 of Algorithm 4 as printed reads q(i+1) = q(i) + α(i)∇U(q(i)); the
 * surrounding text says "take a small step in the direction opposite the
 * gradient" and ċ = −∇U. We follow the text: q(i+1) = q(i) − α(i)∇U(q(i)).
 *
 * The termination test is Choset's forgiving one, ‖∇U(q(i))‖ < ε, and the
 * outcome says *why* the loop stopped: at the goal, at a local minimum that is
 * not the goal (with the Hessian's eigenvalues, so a widget can print their
 * signs), or because the step budget ran out. A descent that cannot tell these
 * apart is the one that reports "success" from the bottom of a cup.
 */

import { classifyCritical, eigenSym, hessian, vdist, vnorm, vscale, vsub, vzero, type CriticalType, type Potential, type Vec } from './field';

export interface DescentOptions {
  /** α(i): a constant or a schedule. Choset: "often made on an ad hoc or empirical basis". */
  alpha: number | ((i: number) => number);
  /** ε in ‖∇U(q(i))‖ < ε. */
  eps: number;
  maxSteps: number;
  /** When given, arriving within `goalTol` of this point counts as success. */
  goal?: Vec;
  goalTol?: number;
  /** Optional free-space test; a step into collision stops the descent. */
  isFree?: (q: Vec) => boolean;
  /** Cap on the per-step displacement, so a repulsive spike cannot fling the bead across the map. */
  maxStep?: number;
  /**
   * Stall detection: if the iterate's mean position over the last `stallWindow`
   * steps moved less than `stallTol`, stop. Choset's ‖∇U‖ < ε never fires where
   * ∇d_i is discontinuous (two closest points tie on a nonconvex obstacle): the
   * bead chatters across the seam with a large gradient and goes nowhere.
   */
  stallWindow?: number;
  stallTol?: number;
}

export type DescentOutcome =
  | { kind: 'goal'; steps: number }
  | { kind: 'localMinimum'; steps: number; gradNorm: number; eigs: number[]; type: CriticalType }
  | { kind: 'stepLimit'; steps: number; gradNorm: number }
  | { kind: 'collision'; steps: number }
  /** Chattering in place with ‖∇U‖ ≥ ε: the nonsmooth seam of a nonconvex obstacle (§4.1's oscillation). */
  | { kind: 'stalled'; steps: number; gradNorm: number };

export interface DescentResult {
  path: Vec[];
  outcome: DescentOutcome;
}

/** One step of Algorithm 4 (with the minus sign). */
export function descentStep(U: Potential, q: Vec, alpha: number, maxStep = Infinity): { q: Vec; grad: Vec } {
  const grad = U.gradient(q);
  let step = vscale(grad, -alpha);
  const n = vnorm(step);
  if (n > maxStep) step = vscale(step, maxStep / n);
  return { q: q.map((v, i) => v + step[i]), grad };
}

/**
 * Classify where a descent stopped. `atGoal` short-circuits the Hessian: a
 * vanishing gradient at the goal is the minimum we wanted.
 */
export function stuckReport(U: Potential, q: Vec, eps: number, goal?: Vec, goalTol = 1e-3): DescentOutcome | null {
  const grad = U.gradient(q);
  const gn = vnorm(grad);
  if (goal && vdist(q, goal) <= goalTol) return { kind: 'goal', steps: 0 };
  if (gn >= eps) return null;
  const eigs = eigenSym(hessian(U, q));
  return { kind: 'localMinimum', steps: 0, gradNorm: gn, eigs, type: classifyCritical(eigs, 1e-7) };
}

/**
 * Algorithm 4 run to completion. Returns the iterates {q(0), …, q(i)} and why
 * it stopped.
 */
export function gradientDescent(U: Potential, qStart: Vec, opts: DescentOptions): DescentResult {
  const d = new Descender(U, qStart, opts);
  while (!d.done) d.step();
  return { path: d.path, outcome: d.outcome! };
}

/** The same algorithm as a stepper, so a widget can advance it a few iterations per frame. */
export class Descender {
  readonly path: Vec[];
  outcome: DescentOutcome | null = null;
  private i = 0;

  constructor(
    readonly U: Potential,
    qStart: Vec,
    readonly opts: DescentOptions,
  ) {
    this.path = [qStart.slice()];
  }

  get q(): Vec {
    return this.path[this.path.length - 1];
  }

  get done(): boolean {
    return this.outcome !== null;
  }

  private alpha(i: number): number {
    const a = this.opts.alpha;
    return typeof a === 'number' ? a : a(i);
  }

  step(): DescentOutcome | null {
    if (this.outcome) return this.outcome;
    const { eps, maxSteps, goal, goalTol = 1e-3, isFree, maxStep = Infinity, stallWindow = 0, stallTol = 0 } = this.opts;
    const q = this.q;
    if (stallWindow > 0 && this.path.length > 2 * stallWindow) {
      const n = this.path.length;
      const mean = (from: number, to: number): Vec => {
        const m = vzero(q.length);
        for (let k = from; k < to; k++) for (let i = 0; i < q.length; i++) m[i] += this.path[k][i] / (to - from);
        return m;
      };
      const recent = mean(n - stallWindow, n);
      const before = mean(n - 2 * stallWindow, n - stallWindow);
      if (vdist(recent, before) < stallTol) {
        this.outcome = { kind: 'stalled', steps: this.i, gradNorm: vnorm(this.U.gradient(q)) };
        return this.outcome;
      }
    }
    if (goal && vdist(q, goal) <= goalTol) {
      this.outcome = { kind: 'goal', steps: this.i };
      return this.outcome;
    }
    const grad = this.U.gradient(q);
    const gn = vnorm(grad);
    if (gn < eps) {
      const eigs = eigenSym(hessian(this.U, q));
      this.outcome = { kind: 'localMinimum', steps: this.i, gradNorm: gn, eigs, type: classifyCritical(eigs, 1e-7) };
      return this.outcome;
    }
    if (this.i >= maxSteps) {
      this.outcome = { kind: 'stepLimit', steps: this.i, gradNorm: gn };
      return this.outcome;
    }
    let step = vscale(grad, -this.alpha(this.i));
    const sn = vnorm(step);
    if (sn > maxStep) step = vscale(step, maxStep / sn);
    const next = q.map((v, k) => v + step[k]);
    if (isFree && !isFree(next)) {
      this.outcome = { kind: 'collision', steps: this.i };
      return this.outcome;
    }
    // A step that overshoots the goal and would start oscillating: snap.
    if (goal && vdist(next, goal) <= goalTol) {
      this.path.push(goal.slice());
      this.i++;
      this.outcome = { kind: 'goal', steps: this.i };
      return this.outcome;
    }
    this.path.push(next);
    this.i++;
    return null;
  }
}

/** The design doc's Part B, as a function of its parameters — pinned by the chapter's check. */
export function oneStep(U: Potential, q: Vec, alpha: number): Vec {
  return vsub(q, vscale(U.gradient(q), alpha));
}
