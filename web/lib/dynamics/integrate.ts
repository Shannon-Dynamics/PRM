/**
 * Integrators for the first-order state x = (q, q̇), ẋ = (q̇, forward(q, q̇, u)).
 *
 * Classical RK4 is the book's default: local error O(h⁵), and on the
 * conservative Reach an energy error that stays below 1e-6 relative over the
 * 10 s horizons the widgets run (Derivation 6 — an empirical bound, not a
 * symplectic guarantee, and the text says so). Explicit Euler is kept as the
 * honest bad example: its energy drifts linearly and the check asserts it
 * does. Semi-implicit (symplectic) Euler is included because it is what game
 * engines run: cheap, bounded energy error, but a first-order one.
 */

import type { Vec } from '../prob/linalg';
import { forward, totalEnergy, type Dynamics } from './model';

export type Integrator = 'rk4' | 'semi-implicit-euler' | 'explicit-euler';

export interface State {
  q: Vec;
  qd: Vec;
}

/** One classical RK4 step of ẋ = f(x) on a flat state vector. */
export function rk4Step(f: (x: Vec) => Vec, x: Vec, h: number): Vec {
  const k1 = f(x);
  const k2 = f(x.map((v, i) => v + (h / 2) * k1[i]));
  const k3 = f(x.map((v, i) => v + (h / 2) * k2[i]));
  const k4 = f(x.map((v, i) => v + h * k3[i]));
  return x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

/** RK4 on (q, q̇) with torque u held over the step. */
export function rk4StepDynamics(dyn: Dynamics, s: State, u: Vec, h: number): State {
  const n = dyn.n;
  const f = (x: Vec): Vec => {
    const q = x.slice(0, n);
    const qd = x.slice(n);
    return [...qd, ...forward(dyn, q, qd, u)];
  };
  const x = rk4Step(f, [...s.q, ...s.qd], h);
  return { q: x.slice(0, n), qd: x.slice(n) };
}

/** Symplectic Euler: update q̇ first, then q with the *new* velocity. */
export function semiImplicitEulerStep(dyn: Dynamics, s: State, u: Vec, h: number): State {
  const qdd = forward(dyn, s.q, s.qd, u);
  const qd = s.qd.map((v, i) => v + h * qdd[i]);
  const q = s.q.map((v, i) => v + h * qd[i]);
  return { q, qd };
}

/** Explicit Euler: both updated from the old state. Drifts; shown as the warning. */
export function explicitEulerStep(dyn: Dynamics, s: State, u: Vec, h: number): State {
  const qdd = forward(dyn, s.q, s.qd, u);
  return { q: s.q.map((v, i) => v + h * s.qd[i]), qd: s.qd.map((v, i) => v + h * qdd[i]) };
}

export function stepDynamics(dyn: Dynamics, s: State, u: Vec, h: number, method: Integrator): State {
  switch (method) {
    case 'rk4':
      return rk4StepDynamics(dyn, s, u, h);
    case 'semi-implicit-euler':
      return semiImplicitEulerStep(dyn, s, u, h);
    case 'explicit-euler':
      return explicitEulerStep(dyn, s, u, h);
  }
}

export interface Sample {
  t: number;
  q: Vec;
  qd: Vec;
  energy: number;
}

/** Integrate for `T` seconds with step `h` under a torque policy; sample every `every` steps. */
export function simulate(
  dyn: Dynamics,
  s0: State,
  u: (t: number, s: State) => Vec,
  h: number,
  T: number,
  method: Integrator = 'rk4',
  every = 1,
): Sample[] {
  const out: Sample[] = [{ t: 0, q: s0.q.slice(), qd: s0.qd.slice(), energy: totalEnergy(dyn, s0.q, s0.qd) }];
  let s = s0;
  const steps = Math.round(T / h);
  for (let k = 1; k <= steps; k++) {
    const t = (k - 1) * h;
    s = stepDynamics(dyn, s, u(t, s), h, method);
    if (k % every === 0) out.push({ t: k * h, q: s.q.slice(), qd: s.qd.slice(), energy: totalEnergy(dyn, s.q, s.qd) });
  }
  return out;
}

/** max_t |E(t) − E(0)| / |E(0)| — the number the drift table reports. */
export function energyDrift(samples: readonly Sample[]): number {
  const e0 = samples[0].energy;
  const scale = Math.max(Math.abs(e0), 1e-12);
  let worst = 0;
  for (const s of samples) worst = Math.max(worst, Math.abs(s.energy - e0) / scale);
  return worst;
}
