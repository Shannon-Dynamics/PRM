/**
 * Numerical self-checks for Chapter 2 — The Lab.
 *
 * Same contract as `lib/__checks__.ts`: the worked numbers of the design doc
 * reproduced to their stated tolerance, plus invariants the geometry
 * guarantees (brute-force agreement, round-trips, bit-exact replay). Run with
 *
 *   npx esbuild lib/robots/__checks_ch02__.ts --bundle --platform=node --format=esm \
 *     --outfile=.next/ck02.mjs && node -e "import('./.next/ck02.mjs').then(m=>m.runCh02Checks().forEach(r=>console.log(r.pass?'PASS':'FAIL',r.name,r.detail??'')))"
 */

import type { CheckResult } from '../__checks__';
import { normalizeAngle } from '../geom/se2';
import { Rng } from '../prob/rng';
import { APARTMENT, distanceToWalls } from '../sim/world';
import { RUSTY } from '../sim/rusty';
import {
  convexPolygonsIntersectSAT,
  polygonsIntersectBrute,
  rect,
  type Polygon,
} from '../collide/polygon';
import { Scene } from '../collide/scene';
import { ChainChecker, DiscChecker, FootprintChecker } from '../collide/collision';
import { nearestReturn, scan } from '../collide/sensor';
import { Reach, REACH_HOME, t2, type Tn } from './reach';
import { Hitch, type HitchCfg } from './hitch';
import { APARTMENT_SCENE, LOT, WORKBENCH } from './worlds';
import { BOOK_SEED, logsIdentical, recordTour } from './tour';

const fmt = (x: number, d = 4): string => x.toFixed(d);

export function runCh02Checks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  // -------------------------------------------------------------------------
  // Reach's forward kinematics — Choset Example 3.8.1
  // -------------------------------------------------------------------------
  const arm = Reach.choset();
  {
    const pts = arm.fk(REACH_HOME);
    const elbow = pts[1];
    const tip = pts[2];
    const ok =
      Math.abs(elbow.x - Math.SQRT1_2) < 1e-12 &&
      Math.abs(elbow.y - Math.SQRT1_2) < 1e-12 &&
      Math.abs(tip.x) < 1e-12 &&
      Math.abs(tip.y - Math.SQRT2) < 1e-12;
    push(
      'fk_matches_choset_example: q = (π/4, π/2) gives elbow (0.7071, 0.7071), tip (0, 1.4142)',
      ok,
      `elbow (${fmt(elbow.x)}, ${fmt(elbow.y)})  tip (${fmt(tip.x)}, ${fmt(tip.y)})`,
    );
  }

  {
    // The chart wraps: adding 2π to any joint is the same point of T².
    const q: Tn = [0.4, -2.9];
    const a = arm.tip(q);
    const b = arm.tip(t2(q[0] + 2 * Math.PI, q[1] - 4 * Math.PI));
    push(
      'T² chart: fk(q) == fk(q + 2πk) and t2() wraps into (−π, π]',
      Math.hypot(a.x - b.x, a.y - b.y) < 1e-12 && t2(4, -4).every((v) => v > -Math.PI && v <= Math.PI),
      `|Δtip| = ${Math.hypot(a.x - b.x, a.y - b.y).toExponential(2)}`,
    );
  }

  // -------------------------------------------------------------------------
  // The worked distance query: link 2 against the square [0.5, 1] × [1, 1.5]
  // -------------------------------------------------------------------------
  {
    const scratch = Scene.ofWalls([], WORKBENCH.bounds).withObstacle('the block', rect(0.5, 1.0, 1.0, 1.5));
    const checker = new ChainChecker(scratch, arm);
    const expected = 1.5 / Math.SQRT2 - 1;
    const w = checker.witness(REACH_HOME);
    const d = checker.distance(REACH_HOME);
    const aStar = { x: 0.5 - expected * Math.SQRT1_2, y: 1.0 - expected * Math.SQRT1_2 };
    const ok =
      w !== null &&
      Math.abs(d - expected) < 1e-9 &&
      Math.abs(w.onObstacle.x - 0.5) < 1e-6 &&
      Math.abs(w.onObstacle.y - 1.0) < 1e-6 &&
      Math.abs(w.onRobot.x - aStar.x) < 1e-6 &&
      Math.abs(w.onRobot.y - aStar.y) < 1e-6 &&
      checker.isFree(REACH_HOME);
    push(
      'reach_distance_to_square: dist = 1.5/√2 − 1 ≈ 0.0607, a* ≈ (0.4571, 0.9571), b* = (0.5, 1.0), intersects false',
      ok,
      w
        ? `dist ${fmt(d)}  a*=(${fmt(w.onRobot.x)}, ${fmt(w.onRobot.y)})  b*=(${fmt(w.onObstacle.x)}, ${fmt(w.onObstacle.y)})`
        : 'no witness',
    );

    // The default Workbench arm has 3 cm capsules: the same query reads 0.0307.
    const thick = new ChainChecker(scratch, Reach.workbench());
    push(
      'capsule links: the 3 cm Workbench arm sees 0.0607 − 0.03 = 0.0307 to the block',
      Math.abs(thick.distance(REACH_HOME) - (expected - 0.03)) < 1e-9,
      `dist ${fmt(thick.distance(REACH_HOME))}`,
    );

    // The block *is* the default Workbench's block, so the world agrees with the scratch scene.
    const inWorld = new ChainChecker(Scene.fromWorld(WORKBENCH), arm).distance(REACH_HOME);
    push(
      'the Workbench block is the worked-example square: same distance from the default world',
      Math.abs(inWorld - expected) < 1e-9,
      `dist ${fmt(inWorld)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Rusty against one wall: a disc query is a point query minus r
  // -------------------------------------------------------------------------
  {
    const scene = Scene.ofWalls([{ x1: 3, y1: 0, x2: 3, y2: 3 }], { minX: 0, minY: 0, maxX: 6, maxY: 3 });
    const disc = new DiscChecker(scene, RUSTY.bodyRadius);
    const d = disc.distance({ x: 2.0, y: 1.0 });
    const w = disc.witness({ x: 2.0, y: 1.0 });
    push(
      'rusty_wall_distance: center (2, 1), wall x = 3, r = 0.11 → distance 0.89, witness (3, 1)',
      Math.abs(d - 0.89) < 1e-12 && w !== null && Math.abs(w.onObstacle.x - 3) < 1e-12 && Math.abs(w.onObstacle.y - 1) < 1e-12,
      `distance ${fmt(d)}  b*=(${w ? fmt(w.onObstacle.x) : '?'}, ${w ? fmt(w.onObstacle.y) : '?'})`,
    );

    // And the 360-ray ideal scan from the same point has a ray straight at the wall.
    const ranges = scan({ nRays: 360, maxRange: 4 }, scene, { x: 2.0, y: 1.0 });
    const rho = nearestReturn(ranges);
    push(
      'scan_is_rho_R: min over a 360-ray scan equals the point distance 1.0 to the wall',
      Math.abs(rho - 1.0) < 1e-9,
      `min ρ_R = ${fmt(rho, 6)}`,
    );
  }

  {
    // Disc checker agrees with the ported `distanceToWalls` − r on the Apartment,
    // and the ideal scan never undercuts D(x).
    const scene = Scene.fromWorld(APARTMENT_SCENE);
    const disc = new DiscChecker(scene, RUSTY.bodyRadius);
    const rng = new Rng(21);
    let worst = 0;
    let undercut = 0;
    for (let k = 0; k < 200; k++) {
      const x = rng.uniform(0.2, 11.8);
      const y = rng.uniform(0.2, 8.8);
      const expect = Math.max(0, distanceToWalls(APARTMENT, x, y) - RUSTY.bodyRadius);
      worst = Math.max(worst, Math.abs(disc.distance({ x, y }) - expect));
      const rho = nearestReturn(scan({ nRays: 360, maxRange: 6 }, scene, { x, y }));
      if (rho < distanceToWalls(APARTMENT, x, y) - 1e-9) undercut++;
    }
    push(
      'disc checker = ported wall distance − r on the Apartment; ρ_R ≥ D(x) on every scan',
      worst < 1e-12 && undercut === 0,
      `worst |Δ| = ${worst.toExponential(2)}, ${undercut} undercuts`,
    );
  }

  // -------------------------------------------------------------------------
  // Inverse kinematics round-trip
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(3);
    let worst = 0;
    let both = 0;
    for (let k = 0; k < 1000; k++) {
      const q = t2(rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI));
      const x = arm.tip(q);
      const sols = arm.ik(x);
      for (const s of sols) {
        if (!s) continue;
        both++;
        const y = arm.tip(s);
        worst = Math.max(worst, Math.hypot(x.x - y.x, x.y - y.y));
      }
    }
    push(
      'ik_roundtrip: fk(ik(fk(q))) == fk(q) for 1000 seeded q, both elbow solutions',
      worst < 1e-9 && both === 2000,
      `worst tip error ${worst.toExponential(2)} over ${both} solutions`,
    );
  }

  // -------------------------------------------------------------------------
  // The swept query catches what endpoint checks miss
  // -------------------------------------------------------------------------
  {
    const checker = new ChainChecker(Scene.fromWorld(WORKBENCH), Reach.workbench());
    const a: Tn = [2.0, 0.0];
    const b: Tn = [3.1, 0.0];
    const freeEnds = checker.isFree(a) && checker.isFree(b);
    const swept = checker.edgeFree(a, b);
    push(
      'swept_catches_midpoint: two free Reach configurations whose straight sweep crosses the post',
      freeEnds && !swept,
      `endpoints free: ${freeEnds}, edgeFree: ${swept}`,
    );
  }

  {
    // Sphere marching agrees with dense sampling for all three robots.
    const rng = new Rng(8);
    const reach = new ChainChecker(Scene.fromWorld(WORKBENCH), Reach.workbench());
    const disc = new DiscChecker(Scene.fromWorld(APARTMENT_SCENE), RUSTY.bodyRadius);
    const hitch = Hitch.car().withTrailer(2.5);
    const car = new FootprintChecker(Scene.fromWorld(LOT), hitch);
    let agree = 0, total = 0, blocked = 0;
    for (let k = 0; k < 150; k++) {
      const qa = t2(rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI));
      const qb = t2(rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI));
      if (!reach.isFree(qa) || !reach.isFree(qb)) continue;
      const m = reach.edgeFree(qa, qb);
      const d = reach.edgeFreeDense(qa, qb, 0.002);
      total++;
      if (m === d) agree++;
      if (!m) blocked++;
    }
    for (let k = 0; k < 150; k++) {
      const pa = { x: rng.uniform(0.3, 11.7), y: rng.uniform(0.3, 8.7) };
      const pb = { x: rng.uniform(0.3, 11.7), y: rng.uniform(0.3, 8.7) };
      if (!disc.isFree(pa) || !disc.isFree(pb)) continue;
      const m = disc.edgeFree(pa, pb);
      const d = disc.edgeFreeDense(pa, pb, 0.002);
      total++;
      if (m === d) agree++;
      if (!m) blocked++;
    }
    for (let k = 0; k < 100; k++) {
      const qa: HitchCfg = hitch.cfg(rng.uniform(2, 14), rng.uniform(2, 8), rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI));
      const qb: HitchCfg = hitch.cfg(rng.uniform(2, 14), rng.uniform(2, 8), rng.uniform(-Math.PI, Math.PI), rng.uniform(-Math.PI, Math.PI));
      if (!car.isFree(qa) || !car.isFree(qb)) continue;
      const m = car.edgeFree(qa, qb);
      const d = car.edgeFreeDense(qa, qb, 0.002);
      total++;
      if (m === d) agree++;
      if (!m) blocked++;
    }
    push(
      'swept: conservative sphere marching agrees with dense sampling (disc, chain, footprint)',
      total > 150 && agree === total,
      `${agree}/${total} edges agree, ${blocked} blocked`,
    );
  }

  // -------------------------------------------------------------------------
  // Polygon intersection: SAT vs. brute force on random convex pairs
  // -------------------------------------------------------------------------
  {
    const rng = new Rng(5);
    const randomConvex = (): Polygon => {
      const cx = rng.uniform(-2, 2), cy = rng.uniform(-2, 2);
      const n = 3 + Math.floor(rng.uniform(0, 5));
      const angles = Array.from({ length: n }, () => rng.uniform(0, 2 * Math.PI)).sort((a, b) => a - b);
      const r = rng.uniform(0.3, 1.2);
      return angles.map((a) => ({ x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) }));
    };
    let agree = 0, hits = 0;
    const N = 400;
    for (let k = 0; k < N; k++) {
      const a = randomConvex();
      const b = randomConvex();
      const s = convexPolygonsIntersectSAT(a, b);
      const bf = polygonsIntersectBrute(a, b);
      if (s === bf) agree++;
      if (bf) hits++;
    }
    push(
      'polygons: separating-axis test equals edge-and-containment brute force on 400 convex pairs',
      agree === N && hits > 50 && hits < N - 50,
      `${agree}/${N} agree, ${hits} intersecting`,
    );
  }

  // -------------------------------------------------------------------------
  // Hitch cannot slide: every displacement is a chord of its own heading arc
  // -------------------------------------------------------------------------
  {
    const car = Hitch.car().withTrailer(2.5);
    const rng = new Rng(13);
    let q = car.cfg(8, 5, 0.3);
    let worst = 0;
    let trailerOk = true;
    for (let k = 0; k < 500; k++) {
      const u = { v: rng.uniform(-1.5, 1.5), phi: rng.uniform(-0.9, 0.9) };
      const n = car.step(q, u, 0.05);
      const dx = n.body.x - q.body.x;
      const dy = n.body.y - q.body.y;
      if (Math.hypot(dx, dy) > 1e-9) {
        // Exact-arc chord: its bearing is the mid-heading, so the lateral
        // component in the mid-heading frame is zero.
        const mid = q.body.theta + normalizeAngle(n.body.theta - q.body.theta) / 2;
        const lateral = -Math.sin(mid) * dx + Math.cos(mid) * dy;
        worst = Math.max(worst, Math.abs(lateral));
      }
      if (n.trailer === null || Math.abs(n.trailer) > Math.PI) trailerOk = false;
      q = n;
    }
    const clamped = car.step(car.cfg(0, 0, 0), { v: 1, phi: 5 }, 1);
    const expectTheta = Math.tan(0.6) / 2.6; // steering clamped to ±maxSteer
    push(
      'hitch: zero lateral velocity in the body frame on every tick; steering clamps to ±max_steer',
      worst < 1e-9 && trailerOk && Math.abs(clamped.body.theta - expectTheta) < 1e-12,
      `worst lateral ${worst.toExponential(2)}, θ after clamped step ${fmt(clamped.body.theta)}`,
    );
  }

  // -------------------------------------------------------------------------
  // Exercise 3: Reach at (0, π/2 + 0.3) against the 0.2 m square centred on (1, 1)
  // -------------------------------------------------------------------------
  {
    const scratch = Scene.ofWalls([], WORKBENCH.bounds).withObstacle('square', rect(0.9, 0.9, 1.1, 1.1));
    const checker = new ChainChecker(scratch, arm);
    const inside = checker.isFree([0, Math.PI / 2]);
    const w = checker.witness([0, Math.PI / 2 + 0.3]);
    const d = checker.distance([0, Math.PI / 2 + 0.3]);
    push(
      'exercise 3: at (0, π/2) the tip sits at the centre of the square (intersecting); at (0, π/2 + 0.3) the distance is 0.1704 to the corner (0.9, 0.9)',
      !inside && w !== null && Math.abs(d - 0.1704) < 5e-4 && Math.abs(w.onObstacle.x - 0.9) < 1e-9 && Math.abs(w.onObstacle.y - 0.9) < 1e-9,
      `intersects at (0, π/2): ${!inside}; dist ${fmt(d)} b*=(${w ? fmt(w.onObstacle.x) : '?'}, ${w ? fmt(w.onObstacle.y) : '?'})`,
    );
  }

  // -------------------------------------------------------------------------
  // Determinism: the tour replays bit for bit
  // -------------------------------------------------------------------------
  {
    const ticks = 20 * 60; // 60 s at 20 Hz
    const a = recordTour(BOOK_SEED, ticks);
    const b = recordTour(BOOK_SEED, ticks);
    const c = recordTour(BOOK_SEED + 1, ticks);
    const last = a.frames[a.frames.length - 1];
    push(
      'tour_replays_bitwise: 1200 frames at BOOK_SEED are identical across two runs, and differ for seed + 1',
      logsIdentical(a, b) && !logsIdentical(a, c) && last.collisions.some((n) => n > 0),
      `final Rusty (${fmt(last.rusty.x, 3)}, ${fmt(last.rusty.y, 3)}), Reach [${last.reach.map((v) => fmt(v, 3)).join(', ')}], Hitch (${fmt(last.hitch.body.x, 3)}, ${fmt(last.hitch.body.y, 3)}, ${fmt(last.hitch.body.theta, 3)}); collisions ${last.collisions.join('/')}`,
    );
    // The final configurations the prose prints for the 60 s tour at BOOK_SEED = 2005.
    const close = (x: number, y: number) => Math.abs(x - y) < 5e-4;
    push(
      'tour at BOOK_SEED after 60 s: Rusty (6.212, 4.466), Reach T2 { 0.186, 1.324 }, Hitch (6.245, 1.351, 0.272), contacts 0/9/12',
      close(last.rusty.x, 6.212) && close(last.rusty.y, 4.466) && close(last.reach[0], 0.186) && close(last.reach[1], 1.324) &&
        close(last.hitch.body.x, 6.245) && close(last.hitch.body.y, 1.351) && close(last.hitch.body.theta, 0.272) &&
        last.collisions[0] === 0 && last.collisions[1] === 9 && last.collisions[2] === 12,
    );
  }

  return out;
}
