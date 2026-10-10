/**
 * "Same destination, six fares": one parking query in the Lot, every steering
 * method of the chapter, one scoreboard. Used by w21.1 (Trailer Parker), the
 * hook, and the integration lab.
 *
 *   rs        — the Reeds–Shepp shortest path (exact, obstacle-blind)
 *   flat      — a quintic in the flat output (rear axle, or trailer axle with a trailer)
 *   sinusoid  — chained form + sinusoids on the 4-state car (obstacle-blind, limit-blind)
 *   grid      — Alg. 22 CAR GRID SEARCH (obstacles, trailer extension when hitched)
 *   hybrid    — hybrid A* with RS shots (obstacles; car only, trailer scored passively)
 *   kino      — forward-propagation RRT on the dynamic car (obstacles; goal *region*)
 *
 * Every method returns dense samples with signed steps; the trailer heading
 * is simulated along them (or produced by the method itself: flat with a
 * trailer, grid with a trailer) and the same `scorePath` grades all six.
 */

import { normalizeAngle, type Pose2 } from '../geom/se2';
import { Rng } from '../prob/rng';
import type { Hitch, HitchCfg } from '../robots/hitch';
import type { Scene } from '../collide/scene';
import { FootprintChecker } from '../collide/collision';
import { reedsSheppShortestPath } from './reeds-shepp';
import { fitFlat, fitFlatTrailer, liftCar, liftTrailer } from './flat';
import { steerCarSinusoids } from './chained';
import { CarGridSearch, type GridSearchCost } from './car-grid';
import { hybridAStar } from './lattice';
import { dynCarMetric, dynCarSystem, HITCH_DYNAMIC, KinoRrt, type DynCar } from './kinodynamic';
import { rsToSegments, sampleSegments, scorePath, trailerAlong, turningRadius, type CarSample, type PathScore } from './car-path';

export type ParkMethod = 'rs' | 'flat' | 'sinusoid' | 'grid' | 'hybrid' | 'kino';
export const PARK_METHODS: readonly ParkMethod[] = ['rs', 'flat', 'sinusoid', 'grid', 'hybrid', 'kino'];
export const PARK_LABELS: Record<ParkMethod, string> = {
  rs: 'Reeds–Shepp',
  flat: 'flat-output quintic',
  sinusoid: 'chained-form sinusoids',
  grid: 'CAR GRID SEARCH (Alg. 22)',
  hybrid: 'hybrid A*',
  kino: 'forward-propagation RRT',
};

export interface ParkQuery {
  car: Hitch;
  scene: Scene;
  start: Pose2;
  goal: Pose2;
}

export interface ParkOptions {
  seed?: number;
  gridCost?: GridSearchCost;
  /** Alg. 22 resolution d and step. */
  gridD?: number;
  gridStep?: number;
  /** Flat planner: endpoint speed as a multiple of the straight-line distance. */
  flatSpeed?: number;
  sinusoidCap?: number;
  kinoIters?: number;
}

export interface ParkResult {
  method: ParkMethod;
  ok: boolean;
  samples: CarSample[];
  score: PathScore;
  ms: number;
  /** Largest |φ| the path asks for, and whether it exceeds the car's limit. */
  maxSteer: number;
  steerViolated: boolean;
  /** Distance and heading error at the end (zero for exact methods). */
  endError: number;
  note: string;
}

const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

function steerDemand(samples: readonly CarSample[]): number {
  // Curvature from consecutive heading changes: |Δθ / ds|, then φ = atan(Lκ) — uniform across methods.
  return Math.max(0, ...samples.slice(1).map((s, i) => (Math.abs(s.ds) > 1e-6 ? Math.abs(normalizeAngle(s.q.theta - samples[i].q.theta) / s.ds) : 0)));
}

export function planPark(method: ParkMethod, query: ParkQuery, opts: ParkOptions = {}): ParkResult {
  const { car, scene, start, goal } = query;
  const rho = turningRadius(car);
  const d = car.p.hitchLen;
  const t0 = now();
  let samples: CarSample[] = [];
  let ok = true;
  let note = '';
  let maxPhi = 0;
  const withTrailer = (s: CarSample[]) => (d !== null ? trailerAlong(s, d, s[0].q.theta) : s);

  switch (method) {
    case 'rs': {
      const p = reedsSheppShortestPath(start, goal, rho)!;
      samples = withTrailer(sampleSegments(start, rsToSegments(p), rho, 0.1));
      note = `${p.word}, ${p.cusps} cusp${p.cusps === 1 ? '' : 's'}`;
      break;
    }
    case 'flat': {
      const dist = Math.hypot(goal.x - start.x, goal.y - start.y);
      const speed = (opts.flatSpeed ?? 1) * dist;
      const n = 160;
      if (d === null) {
        const c = fitFlat(start, goal, { degree: 5, tf: 1, speed });
        for (let i = 0; i <= n; i++) {
          const s = liftCar(c, i / n, car.p.wheelbase);
          samples.push({ q: s.pose, ds: i === 0 ? 0 : 0 });
          maxPhi = Math.max(maxPhi, Math.abs(s.phi));
        }
      } else {
        const c = fitFlatTrailer(start, goal, d, { tf: 1, speed });
        for (let i = 0; i <= n; i++) {
          const s = liftTrailer(c, i / n, d, car.p.wheelbase);
          samples.push({ q: s.car.pose, ds: 0, psi: s.psi });
          maxPhi = Math.max(maxPhi, Math.abs(s.car.phi));
        }
      }
      for (let i = 1; i < samples.length; i++) {
        const a = samples[i - 1].q;
        const b = samples[i].q;
        samples[i].ds = Math.hypot(b.x - a.x, b.y - a.y);
      }
      note = d === null ? 'flat output: rear axle' : 'flat output: trailer axle';
      break;
    }
    case 'sinusoid': {
      const plan = steerCarSinusoids(start, goal, car.p.wheelbase, { ampCap: opts.sinusoidCap ?? Infinity, stepsPerPhase: 240 });
      samples = withTrailer(plan.samples.map((s) => ({ q: s.q, ds: s.ds })));
      maxPhi = plan.maxSteer;
      note = `${plan.phases.length - 1} sinusoid cycles`;
      break;
    }
    case 'grid': {
      const startCfg: HitchCfg = { body: start, trailer: d !== null ? start.theta : null };
      const s = new CarGridSearch({
        car,
        scene,
        start: startCfg,
        goal: { pose: goal, posTol: 0.5, headTol: 0.2 },
        step: opts.gridStep ?? 0.8,
        d: opts.gridD ?? 40,
        cost: opts.gridCost ?? { a: 1, b: 0, c: 3 },
        maxTree: 150000,
        jackknife: d !== null ? 1.4 : undefined,
      });
      s.run(200000);
      const path = s.path();
      if (!path) {
        ok = false;
        note = 'no path within MAXTREESIZE';
        break;
      }
      // Dense: the stored sweeps, trailer from the simulator itself.
      let cur = startCfg;
      samples.push({ q: cur.body, ds: 0, psi: cur.trailer ?? undefined });
      for (const n of path.slice(1)) {
        const act = n.action!;
        const phi = act[0] === 'L' ? car.p.maxSteer : act[0] === 'R' ? -car.p.maxSteer : 0;
        const v = act[1] === '+' ? 1 : -1;
        const sub = 4;
        for (let k = 0; k < sub; k++) {
          cur = car.step(cur, { v, phi }, (opts.gridStep ?? 0.8) / sub);
          samples.push({ q: cur.body, ds: (v * (opts.gridStep ?? 0.8)) / sub, psi: cur.trailer ?? undefined });
        }
      }
      maxPhi = car.p.maxSteer;
      note = `${s.expansions} expansions, goal region ±0.5 m / ±0.2 rad`;
      break;
    }
    case 'hybrid': {
      const r = hybridAStar({ car: car.withTrailer(null), scene, start, goal, cell: 0.5, heuristic: 'dual', shotEvery: 5 });
      if (!r.found) {
        ok = false;
        note = 'hybrid A* failed';
        break;
      }
      samples = withTrailer(r.samples);
      maxPhi = car.p.maxSteer;
      note = `${r.expansions} expansions, ${r.shots.length} RS shots`;
      break;
    }
    case 'kino': {
      const sys = dynCarSystem(HITCH_DYNAMIC);
      const checker = new FootprintChecker(scene, car.withTrailer(null));
      const b = scene.bounds;
      const rng = new Rng(opts.seed ?? 21);
      const rrt = new KinoRrt<DynCar, { acc: number; steerRate: number }>(
        sys,
        {
          start: { ...start, v: 0, phi: 0 },
          isGoal: (s) => Math.hypot(s.x - goal.x, s.y - goal.y) < 0.8 && Math.abs(normalizeAngle(s.theta - goal.theta)) < 0.35,
          sample: (g) => ({ x: g.uniform(b.minX, b.maxX), y: g.uniform(b.minY, b.maxY), theta: g.uniform(-Math.PI, Math.PI), v: g.uniform(-2, 2), phi: 0 }),
          goalSample: () => ({ ...goal, v: 0, phi: 0 }),
          metric: dynCarMetric(1.0, 0.3),
          isFree: (s) => checker.isFree({ body: s, trailer: null }),
        },
        { controlsPerExtend: 8, dt: 0.8, substeps: 4, goalBias: 0.15 },
        rng,
      );
      rrt.plan(opts.kinoIters ?? 3000);
      const dense = rrt.densePath();
      if (!dense) {
        ok = false;
        note = `no solution in ${rrt.iterations} extensions`;
        break;
      }
      samples = [{ q: { x: dense[0].x, y: dense[0].y, theta: dense[0].theta }, ds: 0 }];
      for (let i = 1; i < dense.length; i++) {
        const a = dense[i - 1];
        const s = dense[i];
        const step = Math.hypot(s.x - a.x, s.y - a.y);
        samples.push({ q: { x: s.x, y: s.y, theta: s.theta }, ds: s.v >= 0 ? step : -step });
        maxPhi = Math.max(maxPhi, Math.abs(s.phi));
      }
      samples = withTrailer(samples);
      note = `${rrt.nodes.length} nodes, ${rrt.propagations} propagations`;
      break;
    }
  }
  const ms = now() - t0;
  if (!ok || samples.length === 0) {
    return { method, ok: false, samples: [], score: { length: 0, cusps: 0, maxHitch: 0, clearance: 0, collides: false }, ms, maxSteer: 0, steerViolated: false, endError: Infinity, note };
  }
  // Methods that do not report a steering angle: estimate it from the path's curvature.
  if (method === 'rs' || method === 'hybrid' || method === 'grid') maxPhi = Math.atan(car.p.wheelbase * Math.min(1 / rho, steerDemand(samples)));
  const score = scorePath(samples, car, scene);
  const end = samples[samples.length - 1].q;
  const endError = Math.max(Math.hypot(end.x - goal.x, end.y - goal.y), Math.abs(normalizeAngle(end.theta - goal.theta)));
  return { method, ok, samples, score, ms, maxSteer: maxPhi, steerViolated: maxPhi > car.p.maxSteer + 1e-6, endError, note };
}
