/**
 * The three-robot tour: the chapter's integration lab and the engine behind
 * the dashboard (w2.1).
 *
 * Rusty circles the Apartment's corridor, Reach sweeps the Workbench, Hitch
 * backs through the Lot. Each is driven by a *script* whose free parameters
 * are drawn once from a seeded `Rng` (re-rolling the seed changes the tours,
 * never the worlds), each is collision-checked every tick through its own
 * `Collision` implementation, and each flashes and counts on contact.
 *
 * The whole thing is a pure function of (seed, tick): `recordTour(seed, n)`
 * produces a `Log` of frames, and running it twice produces the same bytes.
 * That is the determinism contract every later chapter inherits, and the
 * check `tour_replays_bitwise` holds the book to it.
 */

import { normalizeAngle, type Pose2 } from '../geom/se2';
import { Rng } from '../prob/rng';
import { diffDriveSlipStep, pursuePoint, RUSTY, type RustyState, type Twist } from '../sim/rusty';
import { Scene } from '../collide/scene';
import { ChainChecker, DiscChecker, FootprintChecker } from '../collide/collision';
import { Hitch, type CarCmd, type HitchCfg } from './hitch';
import { Reach, type Tn } from './reach';
import { APARTMENT_SCENE, LOT, WORKBENCH } from './worlds';

export const TOUR_HZ = 20;
export const TOUR_DT = 1 / TOUR_HZ;
/** The book's fixed seed, so the printed final configurations are reproducible. */
export const BOOK_SEED = 2005;

export interface TourOptions {
  reach?: Reach;
  hitch?: Hitch;
}

/** Scripted tour parameters, drawn once per seed. */
export interface TourScripts {
  rusty: { waypoints: { x: number; y: number }[]; speed: number };
  reach: { amp: [number, number]; omega: [number, number]; phase: [number, number]; center: [number, number] };
  hitch: { legs: { u: CarCmd; ticks: number }[] };
}

export interface RobotPane<Q> {
  q: Q;
  /** Frames since the last collision onset; −1 when never. Drives the flash. */
  flash: number;
  /** Number of contact *events* (entries into collision). */
  collisions: number;
  inCollision: boolean;
}

export interface TourState {
  seed: number;
  scripts: TourScripts;
  rusty: RobotPane<RustyState> & { waypoint: number; trail: Pose2[] };
  reach: RobotPane<Tn> & { trail: { x: number; y: number }[] };
  hitch: RobotPane<HitchCfg> & { trail: Pose2[] };
  rng: Rng;
}

export interface Frame {
  tick: number;
  rusty: Pose2;
  reach: number[];
  hitch: HitchCfg;
  collisions: [number, number, number];
}

export interface Log {
  seed: number;
  frames: Frame[];
}

/** The checkers each pane uses — built once, shared by the dashboard and the checks. */
export function tourCheckers(opts: TourOptions = {}) {
  const reach = opts.reach ?? Reach.workbench();
  const hitch = opts.hitch ?? Hitch.car();
  return {
    reach,
    hitch,
    rustyChecker: new DiscChecker(Scene.fromWorld(APARTMENT_SCENE), RUSTY.bodyRadius),
    reachChecker: new ChainChecker(Scene.fromWorld(WORKBENCH), reach),
    hitchChecker: new FootprintChecker(Scene.fromWorld(LOT), hitch),
  };
}

/** Draw the tour's script parameters from the seed. */
export function drawScripts(rng: Rng): TourScripts {
  // Rusty: a loop out of room A, down the corridor, into room C and back; the
  // seed jitters the waypoints so the executed path differs run to run.
  const j = () => rng.uniform(-0.12, 0.12);
  const waypoints = [
    { x: 2.0 + j(), y: 2.0 + j() },
    { x: 2.05, y: 4.4 + j() },
    { x: 9.9 + j(), y: 4.4 + j() },
    { x: 9.95, y: 2.2 + j() },
    { x: 9.9, y: 4.4 + j() },
    { x: 2.05 + j(), y: 4.4 + j() },
  ];
  const speed = rng.uniform(0.5, 0.7);

  // Reach: two incommensurate sinusoids so the tip traces a Lissajous figure
  // through the block's neighbourhood.
  const reach: TourScripts['reach'] = {
    amp: [rng.uniform(0.7, 1.1), rng.uniform(0.9, 1.4)],
    omega: [rng.uniform(0.25, 0.4), rng.uniform(0.45, 0.7)],
    phase: [rng.uniform(0, 2 * Math.PI), rng.uniform(0, 2 * Math.PI)],
    center: [rng.uniform(0.6, 1.0), rng.uniform(1.2, 1.8)],
  };

  // Hitch: a reversing maneuver — pull forward, swing, back toward the bays —
  // with the steering angles randomized.
  const legs: TourScripts['hitch']['legs'] = [
    { u: { v: 1.2, phi: 0 }, ticks: 50 },
    { u: { v: 1.0, phi: rng.uniform(0.3, 0.55) }, ticks: 60 },
    { u: { v: -1.0, phi: -rng.uniform(0.3, 0.55) }, ticks: 70 },
    { u: { v: -0.9, phi: rng.uniform(-0.15, 0.15) }, ticks: 50 },
    { u: { v: 1.1, phi: rng.uniform(-0.5, 0.5) }, ticks: 60 },
    { u: { v: -1.0, phi: rng.uniform(-0.5, 0.5) }, ticks: 60 },
  ];
  return { rusty: { waypoints, speed }, reach, hitch: { legs } };
}

export function tourInit(seed: number, opts: TourOptions = {}): TourState {
  const rng = new Rng(seed);
  const scripts = drawScripts(rng);
  const hitch = opts.hitch ?? Hitch.car();
  const pane = <Q,>(q: Q): RobotPane<Q> => ({ q, flash: -1, collisions: 0, inCollision: false });
  const start: Pose2 = { x: 2.0, y: 2.0, theta: Math.PI / 2 };
  return {
    seed,
    scripts,
    rng,
    rusty: { ...pane<RustyState>({ pose: start, wheelAngles: [0, 0] }), waypoint: 1, trail: [start] },
    reach: { ...pane<Tn>([scripts.reach.center[0], scripts.reach.center[1]]), trail: [] },
    hitch: { ...pane<HitchCfg>(hitch.cfg(8.0, 2.0, 0)), trail: [] },
  };
}

/** Which script leg is active at this tick (loops). */
export function hitchCommandAt(scripts: TourScripts, tick: number): CarCmd {
  const total = scripts.hitch.legs.reduce((a, l) => a + l.ticks, 0);
  let k = tick % total;
  for (const leg of scripts.hitch.legs) {
    if (k < leg.ticks) return leg.u;
    k -= leg.ticks;
  }
  return { v: 0, phi: 0 };
}

export function reachScriptAt(scripts: TourScripts, t: number): Tn {
  const s = scripts.reach;
  return [
    normalizeAngle(s.center[0] + s.amp[0] * Math.sin(s.omega[0] * t + s.phase[0])),
    normalizeAngle(s.center[1] + s.amp[1] * Math.sin(s.omega[1] * t + s.phase[1])),
  ];
}

/** Update the flash/counter bookkeeping of a pane from this tick's collision status. */
function mark<Q>(pane: RobotPane<Q>, colliding: boolean): void {
  if (colliding && !pane.inCollision) {
    pane.collisions += 1;
    pane.flash = 0;
  } else if (pane.flash >= 0) {
    pane.flash += 1;
  }
  pane.inCollision = colliding;
}

export interface TourOverrides {
  /** When set, this twist drives Rusty instead of the script. */
  rusty?: Twist;
  /** When set, Reach holds this configuration instead of following the script. */
  reach?: Tn;
  /** When set, this command drives Hitch instead of the script. */
  hitch?: CarCmd;
}

const MAX_TRAIL = 400;

/**
 * One 1/20 s tick of all three robots. Mutates and returns the state (the
 * animation hook allows that); `overrides` let the reader take over one pane.
 */
export function tourStep(
  s: TourState,
  tick: number,
  checkers: ReturnType<typeof tourCheckers>,
  overrides: TourOverrides = {},
): TourState {
  const t = tick * TOUR_DT;

  // --- Rusty -----------------------------------------------------------------
  {
    const r = s.rusty;
    let u: Twist;
    if (overrides.rusty) {
      u = overrides.rusty;
    } else {
      const wps = s.scripts.rusty.waypoints;
      const target = wps[r.waypoint];
      if (Math.hypot(target.x - r.q.pose.x, target.y - r.q.pose.y) < 0.3) {
        r.waypoint = (r.waypoint + 1) % wps.length;
      }
      u = pursuePoint(r.q.pose, wps[r.waypoint], { speed: s.scripts.rusty.speed });
    }
    // Slip is the one stochastic subsystem; it owns the tour's rng stream.
    const out = diffDriveSlipStep(r.q, u, TOUR_DT, null, RUSTY, s.rng);
    const free = checkers.rustyChecker.isFree({ x: out.pose.x, y: out.pose.y });
    if (free) {
      r.q = { pose: out.pose, wheelAngles: out.wheelAngles };
      r.trail.push(out.pose);
      if (r.trail.length > MAX_TRAIL) r.trail.shift();
    } else {
      // Held at the wall, wheels still turning — the stuck-robot encoders.
      r.q = { pose: r.q.pose, wheelAngles: out.wheelAngles };
    }
    mark(r, !free);
  }

  // --- Reach -----------------------------------------------------------------
  {
    const a = s.reach;
    a.q = overrides.reach ?? reachScriptAt(s.scripts, t);
    const free = checkers.reachChecker.isFree(a.q);
    a.trail.push(checkers.reach.tip(a.q));
    if (a.trail.length > MAX_TRAIL) a.trail.shift();
    mark(a, !free);
  }

  // --- Hitch -----------------------------------------------------------------
  {
    const h = s.hitch;
    const u = overrides.hitch ?? hitchCommandAt(s.scripts, tick);
    const next = checkers.hitch.step(h.q, u, TOUR_DT);
    const free = checkers.hitchChecker.isFree(next);
    if (free) {
      h.q = next;
      h.trail.push(next.body);
      if (h.trail.length > MAX_TRAIL) h.trail.shift();
    }
    mark(h, !free);
  }

  return s;
}

/** Snapshot the three configurations and counters. */
export function frameOf(s: TourState, tick: number): Frame {
  return {
    tick,
    rusty: { ...s.rusty.q.pose },
    reach: [...s.reach.q],
    hitch: { body: { ...s.hitch.q.body }, trailer: s.hitch.q.trailer },
    collisions: [s.rusty.collisions, s.reach.collisions, s.hitch.collisions],
  };
}

/** `Run::record` — the whole tour as a log of frames. */
export function recordTour(seed: number, ticks: number, opts: TourOptions = {}): Log {
  const checkers = tourCheckers(opts);
  const s = tourInit(seed, opts);
  const frames: Frame[] = [];
  for (let k = 0; k < ticks; k++) {
    tourStep(s, k, checkers);
    frames.push(frameOf(s, k));
  }
  return { seed, frames };
}

/** Bit-for-bit equality of two logs — not "close", *equal*. */
export function logsIdentical(a: Log, b: Log): boolean {
  if (a.seed !== b.seed || a.frames.length !== b.frames.length) return false;
  for (let i = 0; i < a.frames.length; i++) {
    const p = a.frames[i];
    const q = b.frames[i];
    if (p.rusty.x !== q.rusty.x || p.rusty.y !== q.rusty.y || p.rusty.theta !== q.rusty.theta) return false;
    if (p.reach.length !== q.reach.length) return false;
    for (let j = 0; j < p.reach.length; j++) if (p.reach[j] !== q.reach[j]) return false;
    if (p.hitch.body.x !== q.hitch.body.x || p.hitch.body.y !== q.hitch.body.y) return false;
    if (p.hitch.body.theta !== q.hitch.body.theta || p.hitch.trailer !== q.hitch.trailer) return false;
    for (let j = 0; j < 3; j++) if (p.collisions[j] !== q.collisions[j]) return false;
  }
  return true;
}
