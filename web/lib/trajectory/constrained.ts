/**
 * The Chapter 17 model restricted to a path (Choset eqs. 11.2–11.6).
 *
 * Substitute q̇ = q' ṡ and q̈ = q'' ṡ² + q' s̈ into M q̈ + q̇ᵀΓq̇ + g = u and
 * group by s̈, ṡ² and constants:
 *
 *     a(s) = M(q) q',   b(s) = M(q) q'' + q'ᵀ Γ(q) q',   c(s) = g(q),
 *     a(s) s̈ + b(s) ṡ² + c(s) = u.
 *
 * The velocity-product piece of b is exactly `velocityProduct(q, q')` — the
 * Chapter 17 vector evaluated at q̇ = q'. No new physics is introduced here;
 * this file is the one place where an n-DOF arm becomes a 1-DOF system in s.
 */

import { matVec, type Vec } from '../prob/linalg';
import type { Dynamics } from '../dynamics/model';
import type { Path } from './path';

export interface Abc {
  a: Vec;
  b: Vec;
  c: Vec;
}

export class PathDynamics {
  readonly n: number;
  constructor(
    readonly dynamics: Dynamics,
    readonly path: Path,
  ) {
    if (dynamics.n !== path.n) throw new Error(`PathDynamics: dynamics has ${dynamics.n} DOF, path has ${path.n}`);
    this.n = dynamics.n;
  }

  /** (a, b, c)(s) — eq. (11.6). */
  abc(s: number): Abc {
    const q = this.path.q(s);
    const dq = this.path.dq(s);
    const ddq = this.path.ddq(s);
    const M = this.dynamics.inertia(q);
    const a = matVec(M, dq);
    const Mddq = matVec(M, ddq);
    const vp = this.dynamics.velocityProduct(q, dq);
    const b = Mddq.map((v, i) => v + vp[i]);
    const c = this.dynamics.gravity(q);
    return { a, b, c };
  }

  /** The torque that executes the path at phase state (s, ṡ) with path acceleration s̈. */
  torque(s: number, sdot: number, sddot: number): Vec {
    const { a, b, c } = this.abc(s);
    return a.map((ai, i) => ai * sddot + b[i] * sdot * sdot + c[i]);
  }

  /** Tabulate (a, b, c) on a uniform grid of N+1 points — what every algorithm here consumes. */
  tabulate(N: number): Abc[] {
    const out: Abc[] = [];
    for (let k = 0; k <= N; k++) out.push(this.abc(k / N));
    return out;
  }
}
