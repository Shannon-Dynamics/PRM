/**
 * The common currency of every car planner in Chapter 21: a path is a list of
 * full-lock-or-straight segments with *signed* arc lengths (Reeds–Shepp words,
 * CAR GRID SEARCH actions, lattice primitives and hybrid-A* expansions all
 * reduce to this), or — for the smooth planners (flatness, sinusoids) — a dense
 * list of poses with signed steps. Either way the scoreboard asks the same
 * questions: how long, how many cusps, how far did the trailer swing, how close
 * to the pillars, and can Chapter 2's simulator drive it back to the same
 * endpoint (`replaySegments`).
 */

import { boxplus, normalizeAngle, type Pose2 } from '../geom/se2';
import type { Hitch, HitchCfg } from '../robots/hitch';
import type { Scene } from '../collide/scene';
import { FootprintChecker } from '../collide/collision';
import type { RsPath, RsSteer } from './reeds-shepp';

export interface CarSeg {
  steer: RsSteer;
  /** Signed arc length in metres; negative = reverse. */
  len: number;
}

export interface CarSample {
  q: Pose2;
  /** Signed arc length from the previous sample. */
  ds: number;
  /** Trailer heading ψ when a trailer is simulated along. */
  psi?: number;
}

/** Minimum turning radius ρ = L / tan φ_max (Choset's r_min). */
export function turningRadius(car: Hitch): number {
  return car.p.wheelbase / Math.tan(car.p.maxSteer);
}

/** A Reeds–Shepp (or Dubins) word as signed metric segments. */
export function rsToSegments(p: RsPath): CarSeg[] {
  return p.segments.map((g) => ({ steer: g.steer, len: g.len * p.rho }));
}

/** One segment from q with turning radius ρ. */
export function segEnd(q: Pose2, steer: RsSteer, len: number, rho: number): Pose2 {
  if (steer === 'S') return boxplus(q, [len, 0, 0]);
  const k = (steer === 'L' ? 1 : -1) / rho;
  return boxplus(q, [len, 0, k * len]);
}

/** Dense samples along a segment list at spacing ≤ ds. */
export function sampleSegments(q0: Pose2, segs: readonly CarSeg[], rho: number, ds: number): CarSample[] {
  const out: CarSample[] = [{ q: q0, ds: 0 }];
  let q = q0;
  for (const g of segs) {
    const L = Math.abs(g.len);
    if (L < 1e-12) continue;
    const n = Math.max(1, Math.ceil(L / ds));
    for (let k = 1; k <= n; k++) out.push({ q: segEnd(q, g.steer, (g.len * k) / n, rho), ds: g.len / n });
    q = segEnd(q, g.steer, g.len, rho);
  }
  return out;
}

export function segmentsEnd(q0: Pose2, segs: readonly CarSeg[], rho: number): Pose2 {
  let q = q0;
  for (const g of segs) q = segEnd(q, g.steer, g.len, rho);
  return q;
}

/**
 * Re-integrate a segment list with Chapter 2's `Hitch.step` (φ = ±φ_max or 0,
 * v = ±1, Δt = |len| in one exact tick): the "is it drivable?" test. The body
 * update is an exact arc, so agreement is to round-off.
 */
export function replaySegments(car: Hitch, q0: HitchCfg, segs: readonly CarSeg[]): HitchCfg {
  let q = q0;
  for (const g of segs) {
    const phi = g.steer === 'L' ? car.p.maxSteer : g.steer === 'R' ? -car.p.maxSteer : 0;
    // The trailer's RK4 is per tick; split long segments so it stays accurate.
    const n = Math.max(1, Math.ceil(Math.abs(g.len) / 0.05));
    for (let i = 0; i < n; i++) q = car.step(q, { v: Math.sign(g.len), phi }, Math.abs(g.len) / n);
  }
  return q;
}

/**
 * Integrate the passive trailer ψ' = (1/d) sin(θ − ψ) (per unit signed arc
 * length) along a dense body path, RK4 with θ interpolated linearly between
 * samples — exact for arcs and straights, which is all the planners produce.
 */
export function trailerAlong(samples: readonly CarSample[], d: number, psi0: number): CarSample[] {
  const out: CarSample[] = [{ ...samples[0], psi: psi0 }];
  let psi = psi0;
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1].q.theta;
    const dth = normalizeAngle(samples[i].q.theta - a);
    const ds = samples[i].ds;
    const f = (u: number, p: number) => Math.sin(a + dth * u - p) / d;
    const k1 = f(0, psi);
    const k2 = f(0.5, psi + 0.5 * ds * k1);
    const k3 = f(0.5, psi + 0.5 * ds * k2);
    const k4 = f(1, psi + ds * k3);
    psi = normalizeAngle(psi + (ds / 6) * (k1 + 2 * k2 + 2 * k3 + k4));
    out.push({ ...samples[i], psi });
  }
  return out;
}

export interface PathScore {
  length: number;
  cusps: number;
  /** max |θ − ψ| along the path (0 without trailer). */
  maxHitch: number;
  /** Minimum footprint distance to the scene, metres (0 = touching or colliding). */
  clearance: number;
  collides: boolean;
}

export function scorePath(samples: readonly CarSample[], car: Hitch, scene: Scene | null, clearanceEvery = 3): PathScore {
  let length = 0;
  let cusps = 0;
  let prev = 0;
  let maxHitch = 0;
  for (let i = 1; i < samples.length; i++) {
    const ds = samples[i].ds;
    length += Math.abs(ds);
    if (Math.abs(ds) > 1e-9) {
      const sg = Math.sign(ds);
      if (prev !== 0 && sg !== prev) cusps++;
      prev = sg;
    }
  }
  for (const s of samples) if (s.psi !== undefined) maxHitch = Math.max(maxHitch, Math.abs(normalizeAngle(s.q.theta - s.psi)));
  let clearance = Infinity;
  let collides = false;
  if (scene) {
    const checker = new FootprintChecker(scene, car);
    for (let i = 0; i < samples.length; i++) {
      const cfg: HitchCfg = { body: samples[i].q, trailer: car.hasTrailer ? samples[i].psi ?? samples[i].q.theta : null };
      if (!checker.isFree(cfg)) {
        collides = true;
        clearance = 0;
        break;
      }
      if (i % clearanceEvery === 0 || i === samples.length - 1) clearance = Math.min(clearance, checker.distance(cfg));
    }
  }
  return { length, cusps, maxHitch, clearance: scene ? clearance : Infinity, collides };
}

/** Is every sample's footprint free? (The trailer, if any, at its simulated heading.) */
export function samplesFree(samples: readonly CarSample[], checker: FootprintChecker, withTrailer: boolean): boolean {
  for (const s of samples) {
    if (!checker.isFree({ body: s.q, trailer: withTrailer ? s.psi ?? s.q.theta : null })) return false;
  }
  return true;
}
