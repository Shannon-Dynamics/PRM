/**
 * TOPP-RA — time-optimal path parameterization by reachability analysis
 * (Pham & Pham, IEEE T-RO 34(3), 2018).
 *
 * The reformulation that replaces switch-point hunting with interval
 * propagation. With x = ṡ² and u = s̈ held constant over a stage [s_k, s_{k+1}]
 * of length Δ_k, the kinematics are *linear*: x_{k+1} = x_k + 2 u Δ_k. Each
 * actuator row u_min,i ≤ a_i u + b_i x + c_i ≤ u_max,i is linear in (x, u) at a
 * fixed s, so the per-stage feasible set is a convex polygon and the set of
 * x_k from which some admissible u lands x_{k+1} in a target interval is itself
 * an interval — its ends are the optima of two two-variable linear programs.
 *
 *   backward:  K_N = [x_f, x_f];  K_k = { x ≥ 0 : ∃u feasible at k with x + 2uΔ ∈ K_{k+1} }
 *   forward:   x_0 = ṡ_0²;  x_{k+1} = max { x_k + 2uΔ : u feasible at k, x_{k+1} ∈ K_{k+1} }
 *
 * Greedy-forward is optimal because a larger x_k never shrinks the reachable
 * set downstream (controllability is monotone in x). The two-variable LPs are
 * solved exactly by vertex enumeration — seven half-planes, at most 21 corners.
 */

import type { PathDynamics } from './constrained';
import { ZERO_INERTIA_EPS, type TorqueLimits } from './limits';

const BIG = 1e9;

interface HalfPlane {
  /** p·x + r·u ≤ q */
  p: number;
  r: number;
  q: number;
}

/** min and max of x over the polygon ∩{half-planes}; null when infeasible. */
function extremeX(hp: HalfPlane[]): { lo: number; hi: number } | null {
  let lo = Infinity;
  let hi = -Infinity;
  const feasible = (x: number, u: number) => hp.every((h) => h.p * x + h.r * u <= h.q + 1e-7 * Math.max(1, Math.abs(h.q)));
  for (let i = 0; i < hp.length; i++) {
    for (let j = i + 1; j < hp.length; j++) {
      const A = hp[i];
      const B = hp[j];
      const det = A.p * B.r - A.r * B.p;
      if (Math.abs(det) < 1e-14) continue;
      const x = (A.q * B.r - A.r * B.q) / det;
      const u = (A.p * B.q - A.q * B.p) / det;
      if (!Number.isFinite(x) || !Number.isFinite(u)) continue;
      if (feasible(x, u)) {
        lo = Math.min(lo, x);
        hi = Math.max(hi, x);
      }
    }
  }
  return lo <= hi ? { lo, hi } : null;
}

export interface ToppraResult {
  ok: boolean;
  detail?: string;
  s: number[];
  sdot: number[];
  sddot: number[];
  t: number[];
  tf: number;
  /** Controllable sets K_k = [lo, hi] in ṡ (√x), on the grid. */
  controllable: { lo: number; hi: number }[];
}

export interface ToppraOptions {
  N?: number;
  sdot0?: number;
  sdotF?: number;
}

export function toppra(pd: PathDynamics, lim: TorqueLimits, opts: ToppraOptions = {}): ToppraResult {
  const N = opts.N ?? 200;
  const sdot0 = opts.sdot0 ?? 0;
  const sdotF = opts.sdotF ?? 0;
  const ds = 1 / N;
  const abc = pd.tabulate(N);
  const s = Array.from({ length: N + 1 }, (_, k) => k / N);

  const rowsAt = (k: number): HalfPlane[] => {
    const { a, b, c } = abc[k];
    const hp: HalfPlane[] = [];
    for (let i = 0; i < a.length; i++) {
      const ai = Math.abs(a[i]) < ZERO_INERTIA_EPS ? 0 : a[i];
      hp.push({ p: b[i], r: ai, q: lim.max[i] - c[i] });
      hp.push({ p: -b[i], r: -ai, q: -(lim.min[i] - c[i]) });
    }
    hp.push({ p: -1, r: 0, q: 0 }); // x ≥ 0
    hp.push({ p: 1, r: 0, q: BIG });
    hp.push({ p: 0, r: 1, q: BIG });
    hp.push({ p: 0, r: -1, q: BIG });
    return hp;
  };

  // ---- Backward pass: controllable intervals in x. -------------------------
  const K: { lo: number; hi: number }[] = new Array(N + 1);
  K[N] = { lo: sdotF * sdotF, hi: sdotF * sdotF };
  let ok = true;
  let detail: string | undefined;
  for (let k = N - 1; k >= 0; k--) {
    const hp = rowsAt(k);
    const next = K[k + 1];
    // x + 2Δu ∈ [lo, hi]
    hp.push({ p: 1, r: 2 * ds, q: next.hi });
    hp.push({ p: -1, r: -2 * ds, q: -next.lo });
    const ext = extremeX(hp);
    if (!ext) {
      ok = false;
      detail = `controllable set empty at s = ${s[k].toFixed(3)}`;
      for (let m = k; m >= 0; m--) K[m] = { lo: 0, hi: 0 };
      break;
    }
    K[k] = { lo: Math.max(0, ext.lo), hi: ext.hi };
  }

  // ---- Forward greedy pass. ---------------------------------------------------
  const x = new Array<number>(N + 1).fill(0);
  const sddot = new Array<number>(N + 1).fill(0);
  x[0] = sdot0 * sdot0;
  if (ok && (x[0] < K[0].lo - 1e-9 || x[0] > K[0].hi + 1e-9)) {
    ok = false;
    detail = `ṡ₀ = ${sdot0} is not controllable (K₀ = [${Math.sqrt(K[0].lo).toFixed(3)}, ${Math.sqrt(K[0].hi).toFixed(3)}])`;
  }
  if (ok) {
    for (let k = 0; k < N; k++) {
      const { a, b, c } = abc[k];
      let uLo = -BIG;
      let uHi = BIG;
      for (let i = 0; i < a.length; i++) {
        const rhsHi = lim.max[i] - c[i] - b[i] * x[k];
        const rhsLo = lim.min[i] - c[i] - b[i] * x[k];
        if (Math.abs(a[i]) < ZERO_INERTIA_EPS) continue;
        const u1 = rhsHi / a[i];
        const u2 = rhsLo / a[i];
        uHi = Math.min(uHi, Math.max(u1, u2));
        uLo = Math.max(uLo, Math.min(u1, u2));
      }
      // Land inside K_{k+1}.
      uHi = Math.min(uHi, (K[k + 1].hi - x[k]) / (2 * ds));
      uLo = Math.max(uLo, (K[k + 1].lo - x[k]) / (2 * ds));
      if (uLo > uHi + 1e-7) {
        // Numerical slack at the boundary of K: take the closest admissible control.
        uHi = uLo;
      }
      const u = uHi;
      sddot[k] = u;
      x[k + 1] = Math.max(0, x[k] + 2 * u * ds);
    }
    sddot[N] = sddot[N - 1];
  }

  const sdot = x.map((v) => Math.sqrt(Math.max(0, v)));
  const t = [0];
  for (let k = 0; k < N; k++) {
    const d = sdot[k] + sdot[k + 1];
    t.push(t[k] + (d > 0 ? (2 * ds) / d : 0));
  }
  return {
    ok,
    detail,
    s,
    sdot,
    sddot,
    t,
    tf: ok ? t[N] : Infinity,
    controllable: K.map((kk) => ({ lo: Math.sqrt(Math.max(0, kk.lo)), hi: Math.sqrt(Math.max(0, kk.hi)) })),
  };
}
