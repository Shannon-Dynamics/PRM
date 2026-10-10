/**
 * Dubins' shortest paths for the forward-only reduced car (Choset §12.5.6,
 * eq. 12.35 with v ≡ 1, |ω| ≤ 1/ρ).
 *
 * Pontryagin makes the Hamiltonian linear in ω, so ω is bang-bang or
 * singular: every segment is a full-lock arc (L, R) or a straight (S), and
 * without cusps the only candidates are CSC and CCC — the six words
 * LSL, RSR, LSR, RSL, RLR, LRL. Each has a closed form in the normalized
 * frame (start at the origin heading along x, distances in units of ρ). The
 * formulas below are the standard ones (Shkel & Lumelsky's parameterization);
 * every word the module returns is checked by *integrating* it — a word that
 * does not land on its goal is a bug, and the chapter's check integrates 400
 * random pairs to 10⁻⁹.
 *
 * Normalized segment lengths: arcs in radians, the straight in units of ρ, so
 * `length = ρ · (t + p + q)`.
 */

import { boxplus, normalizeAngle, type Pose2 } from '../geom/se2';

export type DubinsWord = 'LSL' | 'RSR' | 'LSR' | 'RSL' | 'RLR' | 'LRL';
export const DUBINS_WORDS: readonly DubinsWord[] = ['LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL'];

export interface DubinsPath {
  word: DubinsWord;
  /** Normalized segment lengths (t, p, q): arcs in radians, straight in units of ρ. */
  segs: [number, number, number];
  rho: number;
  /** ρ·(t + p + q). */
  length: number;
}

const TWO_PI = 2 * Math.PI;
/** Wrap to [0, 2π). */
const mod2pi = (a: number): number => {
  let r = a % TWO_PI;
  if (r < 0) r += TWO_PI;
  return r;
};

interface Normalized {
  d: number;
  alpha: number;
  beta: number;
}

/** Start at the origin, bearing to the goal along x: d = ‖Δp‖/ρ, α = θ₀ − bearing, β = θ₁ − bearing. */
export function dubinsNormalize(q0: Pose2, q1: Pose2, rho: number): Normalized {
  const dx = q1.x - q0.x;
  const dy = q1.y - q0.y;
  const D = Math.hypot(dx, dy);
  const bearing = D < 1e-12 ? q0.theta : Math.atan2(dy, dx);
  return { d: D / rho, alpha: mod2pi(q0.theta - bearing), beta: mod2pi(q1.theta - bearing) };
}

/** One word's closed form, or null if infeasible. */
export function dubinsWord(word: DubinsWord, n: Normalized): [number, number, number] | null {
  const { d, alpha, beta } = n;
  const sa = Math.sin(alpha);
  const sb = Math.sin(beta);
  const ca = Math.cos(alpha);
  const cb = Math.cos(beta);
  const cab = Math.cos(alpha - beta);
  switch (word) {
    case 'LSL': {
      // The two left circles and their external tangent — Derivation 5(v).
      const tmp = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
      if (tmp < 0) return null;
      const p = Math.sqrt(tmp);
      const th = Math.atan2(cb - ca, d + sa - sb);
      return [mod2pi(-alpha + th), p, mod2pi(beta - th)];
    }
    case 'RSR': {
      const tmp = 2 + d * d - 2 * cab + 2 * d * (sb - sa);
      if (tmp < 0) return null;
      const p = Math.sqrt(tmp);
      const th = Math.atan2(ca - cb, d - sa + sb);
      return [mod2pi(alpha - th), p, mod2pi(-beta + th)];
    }
    case 'LSR': {
      const tmp = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
      if (tmp < 0) return null;
      const p = Math.sqrt(tmp);
      const th = Math.atan2(-ca - cb, d + sa + sb) - Math.atan2(-2, p);
      return [mod2pi(-alpha + th), p, mod2pi(-mod2pi(beta) + th)];
    }
    case 'RSL': {
      const tmp = d * d - 2 + 2 * cab - 2 * d * (sa + sb);
      if (tmp < 0) return null;
      const p = Math.sqrt(tmp);
      const th = Math.atan2(ca + cb, d - sa - sb) - Math.atan2(2, p);
      return [mod2pi(alpha - th), p, mod2pi(beta - th)];
    }
    case 'RLR': {
      const tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8;
      if (Math.abs(tmp) > 1) return null;
      const p = mod2pi(TWO_PI - Math.acos(tmp));
      const t = mod2pi(alpha - Math.atan2(ca - cb, d - sa + sb) + p / 2);
      return [t, p, mod2pi(alpha - beta - t + p)];
    }
    case 'LRL': {
      const tmp = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8;
      if (Math.abs(tmp) > 1) return null;
      const p = mod2pi(TWO_PI - Math.acos(tmp));
      const t = mod2pi(-alpha + Math.atan2(-ca + cb, d + sa - sb) + p / 2);
      return [t, p, mod2pi(mod2pi(beta) - alpha - t + p)];
    }
  }
}

/** Every feasible word with its length, sorted shortest first. */
export function dubinsAll(q0: Pose2, q1: Pose2, rho: number): DubinsPath[] {
  const n = dubinsNormalize(q0, q1, rho);
  const out: DubinsPath[] = [];
  for (const word of DUBINS_WORDS) {
    const segs = dubinsWord(word, n);
    if (!segs) continue;
    out.push({ word, segs, rho, length: rho * (segs[0] + segs[1] + segs[2]) });
  }
  return out.sort((a, b) => a.length - b.length);
}

/** The shortest Dubins path, or null (never, for ρ > 0 and distinct poses — LSL or RSR always exists). */
export function dubinsShortestPath(q0: Pose2, q1: Pose2, rho: number): DubinsPath | null {
  const all = dubinsAll(q0, q1, rho);
  return all.length ? all[0] : null;
}

/** Follow one segment of a word: an arc of angle `len` (L or R) or a straight of ρ·len. */
export function dubinsSegment(q: Pose2, kind: 'L' | 'S' | 'R', len: number, rho: number): Pose2 {
  if (kind === 'S') return boxplus(q, [rho * len, 0, 0]);
  const sgn = kind === 'L' ? 1 : -1;
  return boxplus(q, [rho * len, 0, sgn * len]);
}

/** Integrate the word from q0: the endpoint, for the cross-check. */
export function dubinsEndpoint(path: DubinsPath, q0: Pose2): Pose2 {
  let q = q0;
  for (let i = 0; i < 3; i++) q = dubinsSegment(q, path.word[i] as 'L' | 'S' | 'R', path.segs[i], path.rho);
  return q;
}

/** Poses along the path at arc-length spacing ds (endpoints included), for drawing and collision checking. */
export function dubinsSample(path: DubinsPath, q0: Pose2, ds: number): Pose2[] {
  const out: Pose2[] = [q0];
  let q = q0;
  for (let i = 0; i < 3; i++) {
    const kind = path.word[i] as 'L' | 'S' | 'R';
    const total = path.segs[i];
    const n = Math.max(1, Math.ceil((path.rho * total) / ds));
    for (let k = 1; k <= n; k++) {
      out.push(dubinsSegment(q, kind, (total * k) / n, path.rho));
    }
    q = dubinsSegment(q, kind, total, path.rho);
  }
  return out;
}

/** The pose at arc length s ∈ [0, length] along the path. */
export function dubinsAt(path: DubinsPath, q0: Pose2, s: number): Pose2 {
  let q = q0;
  let rem = Math.max(0, Math.min(path.length, s)) / path.rho;
  for (let i = 0; i < 3; i++) {
    const kind = path.word[i] as 'L' | 'S' | 'R';
    const len = path.segs[i];
    if (rem <= len) return dubinsSegment(q, kind, rem, path.rho);
    q = dubinsSegment(q, kind, len, path.rho);
    rem -= len;
  }
  return q;
}

/** Pose error (position, wrapped heading) between two poses — the integration cross-check's metric. */
export function poseError(a: Pose2, b: Pose2): number {
  return Math.max(Math.abs(a.x - b.x), Math.abs(a.y - b.y), Math.abs(normalizeAngle(a.theta - b.theta)));
}
