/**
 * Numerical self-checks — the aggregator.
 *
 * The simulations in this book *are* these algorithms: a reader who does the
 * algebra by hand should get the number the widget shows. Each chapter's module
 * ships its own `__checks_chNN__.ts` asserting invariants the mathematics
 * guarantees (round-trips, agreement of two derivations, equality against a
 * brute-force computation) rather than frozen outputs, and registers it in the
 * list below. `npm run check` runs them all.
 */

import { angleDiff, boxminus, boxplus, se2Exp, se2Log, type Pose2, type Twist2 } from './geom/se2';
import { bruteForceEdtSquared, edt2dSquared } from './mapping/edt';
import { Rng } from './prob/rng';
import { APARTMENT, distanceToWalls, rayCast } from './sim/world';
// @chapter-check-imports-start
import { runCh01Checks } from './robots/__checks_ch01__';
import { runCh02Checks } from './robots/__checks_ch02__';
import { runCh03Checks } from './bugs/__checks_ch03__';
import { runCh04Checks } from './cspace/__checks_ch04__';
import { runCh05Checks } from './manifold/__checks_ch05__';
import { runCh06Checks } from './search/__checks_ch06__';
import { runCh07Checks } from './potential/__checks_ch07__';
import { runCh08Checks } from './roadmap/__checks_ch08__';
import { runCh09Checks } from './roadmap/__checks_ch09__';
import { runCh10Checks } from './cells/__checks_ch10__';
import { runCh11Checks } from './sampling/__checks_ch11__';
import { runCh12Checks } from './sampling/__checks_ch12__';
import { runCh13Checks } from './sampling/__checks_ch13__';
import { runCh14Checks } from './multi/__checks_ch14__';
import { runCh15Checks } from './estimate/__checks_ch15__';
import { runCh16Checks } from './estimate/__checks_ch16__';
import { runCh17Checks } from './dynamics/__checks_ch17__';
import { runCh18Checks } from './trajectory/__checks_ch18__';
import { runCh19Checks } from './trajopt/__checks_ch19__';
import { runCh20Checks } from './nonholonomic/__checks_ch20__';
import { runCh21Checks } from './nonholonomic/__checks_ch21__';
import { runCh22Checks } from './learn/__checks_ch22__';
import { runCh23Checks } from './capstone/__checks_ch23__';
// @chapter-check-imports-end

export interface CheckResult {
  name: string;
  pass: boolean;
  detail?: string;
}

const close = (a: number, b: number, tol = 1e-9) => Math.abs(a - b) <= tol;

/** Checks on the ported foundation (sister-book code we depend on but do not re-teach). */
function portedChecks(): CheckResult[] {
  const out: CheckResult[] = [];
  const push = (name: string, pass: boolean, detail?: string) => out.push({ name, pass, detail });

  {
    // SE(2): exp∘log is the identity on poses, and ⊞/⊟ invert each other.
    const rng = new Rng(2);
    let worst = 0;
    for (let k = 0; k < 200; k++) {
      const p: Pose2 = { x: rng.uniform(-5, 5), y: rng.uniform(-5, 5), theta: rng.uniform(-Math.PI, Math.PI) };
      const tau: Twist2 = [rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1)];
      const q = boxplus(p, tau);
      const back = boxminus(q, p);
      worst = Math.max(worst, Math.abs(back[0] - tau[0]), Math.abs(back[1] - tau[1]), Math.abs(angleDiff(back[2], tau[2])));
      const l = se2Log(se2Exp(tau));
      worst = Math.max(worst, Math.abs(l[0] - tau[0]), Math.abs(l[1] - tau[1]), Math.abs(angleDiff(l[2], tau[2])));
    }
    push('ported geom: SE(2) exp/log and ⊞/⊟ round-trip', worst < 1e-8, `worst ${worst.toExponential(2)}`);
  }
  {
    // Seeded RNG is deterministic — the property every demo in the book rests on.
    const a = new Rng(42), b = new Rng(42);
    let same = true;
    for (let i = 0; i < 1000; i++) if (a.next() !== b.next()) same = false;
    push('ported prob: identical seeds give identical streams', same);
  }
  {
    // Exact distance transform agrees with brute force on a random seed grid.
    const nx = 23, ny = 17, rng = new Rng(7);
    const seed = new Float64Array(nx * ny).fill(1e20);
    for (let k = 0; k < 15; k++) seed[Math.floor(rng.uniform(0, nx * ny))] = 0;
    const fast = edt2dSquared(seed, nx, ny), slow = bruteForceEdtSquared(seed, nx, ny);
    let worst = 0;
    for (let i = 0; i < fast.length; i++) worst = Math.max(worst, Math.abs(fast[i] - slow[i]));
    push('ported edt: Felzenszwalb EDT equals brute force', worst < 1e-9, `worst ${worst.toExponential(2)}`);
  }
  {
    // A ray cast toward a wall returns a hit no farther than the point distance to the walls.
    const d = distanceToWalls(APARTMENT, 2.0, 2.0);
    let minRay = Infinity;
    for (let k = 0; k < 64; k++) minRay = Math.min(minRay, rayCast(APARTMENT, 2.0, 2.0, (k / 64) * 2 * Math.PI, 50));
    push('ported sim: nearest-wall distance lower-bounds every ray cast', minRay + 1e-9 >= d, `d=${d.toFixed(3)} min ray=${minRay.toFixed(3)}`);
  }
  return out;
}

/** Chapter check modules register here (one import + one entry each). */
const CHAPTER_CHECKS: Array<() => CheckResult[]> = [
  runCh01Checks, // Chapter 1
  runCh02Checks, // Chapter 2
  runCh03Checks, // Chapter 3
  runCh04Checks, // Chapter 4
  runCh05Checks, // Chapter 5
  runCh06Checks, // Chapter 6
  runCh07Checks, // Chapter 7
  runCh08Checks, // Chapter 8
  runCh09Checks, // Chapter 9
  runCh10Checks, // Chapter 10
  runCh11Checks, // Chapter 11
  runCh12Checks, // Chapter 12
  runCh13Checks, // Chapter 13
  runCh14Checks, // Chapter 14
  runCh15Checks, // Chapter 15
  runCh16Checks, // Chapter 16
  runCh17Checks, // Chapter 17
  runCh18Checks, // Chapter 18
  runCh19Checks, // Chapter 19
  runCh20Checks, // Chapter 20
  runCh21Checks, // Chapter 21
  runCh22Checks, // Chapter 22
  runCh23Checks, // Chapter 23
];

export function runSelfChecks(): CheckResult[] {
  const results: CheckResult[] = [...portedChecks()];
  for (const run of CHAPTER_CHECKS) results.push(...run());
  return results;
}

export { close as approxEqual };
