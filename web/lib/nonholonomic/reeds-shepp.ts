/**
 * Reeds–Shepp shortest paths for the reduced car that may reverse (Choset
 * §12.5.6, eq. 12.35 with |v| ≤ 1, |ω| ≤ |v|/ρ).
 *
 * Reeds and Shepp proved the shortest path lies in Choset's nine families
 *
 *   C|C|C, CC|C, C|CC, CC_a|C_aC, C|C_aC_a|C, C|C_{π/2}SC, CSC_{π/2}|C,
 *   C|C_{π/2}SC_{π/2}|C, CSC,
 *
 * where C is a full-lock arc, S a straight and | a cusp. Writing a word with
 * signs (L⁺ forward-left, R⁻ reverse-right, …) gives Reeds–Shepp's list of
 * forty-eight words. Nobody codes forty-eight formulas. Reeds and Shepp's own
 * §8 gives a handful of *base* formulas (LSL, LSR, L|R|L, L|R⁺L⁺|R, L|R⁻L⁻|R,
 * L|R_{π/2}SL, L|R_{π/2}SR, L|R_{π/2}SL_{π/2}|R) and three symmetries generate
 * the rest:
 *
 *   time-flip   (x, y, φ) → (−x, y, −φ)   reverse every segment (L⁺ ↔ L⁻),
 *   reflect     (x, y, φ) → (x, −y, −φ)   swap left and right   (L ↔ R),
 *   backwards   (x, y, φ) → (x cos φ + y sin φ, x sin φ − y cos φ, φ)
 *                                        read the word right to left.
 *
 * The table below is that construction exactly as OMPL's
 * `ReedsSheppStateSpace` implements it: eleven base formulas (four of them
 * the "backwards" readings), each evaluated under the four flip/reflect
 * combinations — forty-four evaluations. Several base formulas return a last
 * segment whose sign is free (L⁺R⁻L⁺ and L⁺R⁻L⁻ come out of one formula), so
 * the forty-four evaluations cover the forty-eight words. What this module
 * does *not* claim: it does not prove completeness itself. The chapter's checks
 * test the two consequences that would fail if a family were missing or a
 * formula mistyped — every returned word integrates to its goal, and the
 * Reeds–Shepp length never exceeds the Dubins length — on 400 seeded pairs.
 *
 * Units: segment lengths are in units of ρ (arcs in radians), *signed*:
 * negative means driven in reverse.
 */

import { boxplus, normalizeAngle, type Pose2 } from '../geom/se2';

export type RsSteer = 'L' | 'S' | 'R';

export interface RsSegment {
  steer: RsSteer;
  /** Signed normalized length: radians for arcs, units of ρ for S; < 0 is reverse. */
  len: number;
}

/** The nine families of Choset's list, for the atlas tint. */
export type RsFamily = 'C|C|C' | 'CC|C' | 'C|CC' | 'CC|CC' | 'C|CC|C' | 'C|CSC' | 'CSC|C' | 'C|CSC|C' | 'CSC';
export const RS_FAMILIES: readonly RsFamily[] = ['CSC', 'C|C|C', 'CC|C', 'C|CC', 'CC|CC', 'C|CC|C', 'C|CSC', 'CSC|C', 'C|CSC|C'];

export interface RsPath {
  segments: RsSegment[];
  rho: number;
  /** ρ·Σ|len|. */
  length: number;
  /** e.g. "L+R-L+": steering letter and direction sign per segment. */
  word: string;
  family: RsFamily;
  /** Number of direction reversals. */
  cusps: number;
  /** Which base formula and symmetry produced it ("LpSpLp/timeflip", …), for the curious. */
  source: string;
}

const PI = Math.PI;
const TWO_PI = 2 * PI;
const ZERO = 10 * Number.EPSILON;

/** OMPL's mod2pi: wrap to (−π, π]. */
function mod2pi(x: number): number {
  let v = x % TWO_PI;
  if (v < -PI) v += TWO_PI;
  else if (v > PI) v -= TWO_PI;
  return v;
}

function polar(x: number, y: number): [number, number] {
  return [Math.sqrt(x * x + y * y), Math.atan2(y, x)];
}

function tauOmega(u: number, v: number, xi: number, eta: number, phi: number): [number, number] {
  const delta = mod2pi(u - v);
  const A = Math.sin(u) - Math.sin(delta);
  const B = Math.cos(u) - Math.cos(delta) - 1;
  const t1 = Math.atan2(eta * A - xi * B, xi * A + eta * B);
  const t2 = 2 * (Math.cos(delta) - Math.cos(v) - Math.cos(u)) + 3;
  const tau = t2 < 0 ? mod2pi(t1 + PI) : mod2pi(t1);
  const omega = mod2pi(tau - u + v - phi);
  return [tau, omega];
}

type Tuv = [number, number, number] | null;

/* ---- the base formulas (Reeds & Shepp 1990, §8; numbering as theirs) ---- */

/** 8.1: L⁺S⁺L⁺ — the two left circles and their external tangent. */
function LpSpLp(x: number, y: number, phi: number): Tuv {
  const [u, t] = polar(x - Math.sin(phi), y - 1 + Math.cos(phi));
  if (t >= -ZERO) {
    const v = mod2pi(phi - t);
    if (v >= -ZERO) return [t, u, v];
  }
  return null;
}

/** 8.2: L⁺S⁺R⁺ — internal tangent. */
function LpSpRp(x: number, y: number, phi: number): Tuv {
  const [u1raw, t1] = polar(x + Math.sin(phi), y - 1 - Math.cos(phi));
  const u1 = u1raw * u1raw;
  if (u1 >= 4) {
    const u = Math.sqrt(u1 - 4);
    const theta = Math.atan2(2, u);
    const t = mod2pi(t1 + theta);
    const v = mod2pi(t - phi);
    if (t >= -ZERO && v >= -ZERO) return [t, u, v];
  }
  return null;
}

/** 8.3/8.4: L⁺R⁻L — three arcs with a cusp after the first. */
function LpRmL(x: number, y: number, phi: number): Tuv {
  const xi = x - Math.sin(phi);
  const eta = y - 1 + Math.cos(phi);
  const [u1, theta] = polar(xi, eta);
  if (u1 <= 4) {
    const u = -2 * Math.asin(0.25 * u1);
    const t = mod2pi(theta + 0.5 * u + PI);
    const v = mod2pi(phi - t + u);
    if (t >= -ZERO && u <= ZERO) return [t, u, v];
  }
  return null;
}

/** 8.7: L⁺R⁺_a L⁻_a R⁻. */
function LpRupLumRm(x: number, y: number, phi: number): Tuv {
  const xi = x + Math.sin(phi);
  const eta = y - 1 - Math.cos(phi);
  const rho = 0.25 * (2 + Math.sqrt(xi * xi + eta * eta));
  if (rho <= 1) {
    const u = Math.acos(rho);
    const [t, v] = tauOmega(u, -u, xi, eta, phi);
    if (t >= -ZERO && v <= ZERO) return [t, u, v];
  }
  return null;
}

/** 8.8: L⁺R⁻_a L⁻_a R⁺. */
function LpRumLumRp(x: number, y: number, phi: number): Tuv {
  const xi = x + Math.sin(phi);
  const eta = y - 1 - Math.cos(phi);
  const rho = (20 - xi * xi - eta * eta) / 16;
  if (rho >= 0 && rho <= 1) {
    const u = -Math.acos(rho);
    if (u >= -0.5 * PI) {
      const [t, v] = tauOmega(u, u, xi, eta, phi);
      if (t >= -ZERO && v >= -ZERO) return [t, u, v];
    }
  }
  return null;
}

/** 8.9: L⁺R⁻_{π/2} S⁻ L⁻. */
function LpRmSmLm(x: number, y: number, phi: number): Tuv {
  const xi = x - Math.sin(phi);
  const eta = y - 1 + Math.cos(phi);
  const [rho, theta] = polar(xi, eta);
  if (rho >= 2) {
    const r = Math.sqrt(rho * rho - 4);
    const u = 2 - r;
    const t = mod2pi(theta + Math.atan2(r, -2));
    const v = mod2pi(phi - 0.5 * PI - t);
    if (t >= -ZERO && u <= ZERO && v <= ZERO) return [t, u, v];
  }
  return null;
}

/** 8.10: L⁺R⁻_{π/2} S⁻ R⁻. */
function LpRmSmRm(x: number, y: number, phi: number): Tuv {
  const xi = x + Math.sin(phi);
  const eta = y - 1 - Math.cos(phi);
  const [rho, theta] = polar(-eta, xi);
  if (rho >= 2) {
    const t = theta;
    const u = 2 - rho;
    const v = mod2pi(t + 0.5 * PI - phi);
    if (t >= -ZERO && u <= ZERO && v <= ZERO) return [t, u, v];
  }
  return null;
}

/** 8.11: L⁺R⁻_{π/2} S⁻ L⁻_{π/2} R⁺. */
function LpRmSLmRp(x: number, y: number, phi: number): Tuv {
  const xi = x + Math.sin(phi);
  const eta = y - 1 - Math.cos(phi);
  const [rho] = polar(xi, eta);
  if (rho >= 2) {
    const u = 4 - Math.sqrt(rho * rho - 4);
    if (u <= ZERO) {
      const t = mod2pi(Math.atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
      const v = mod2pi(t - phi);
      if (t >= -ZERO && v >= -ZERO) return [t, u, v];
    }
  }
  return null;
}

interface Raw {
  steers: RsSteer[];
  lens: number[];
  source: string;
}

const H = 0.5 * PI;

/**
 * Every candidate word for the normalized goal (x, y, φ) — start at the origin
 * heading along x, ρ = 1. Not just the shortest: the atlas draws all of them.
 */
export function rsCandidatesNormalized(x: number, y: number, phi: number): Raw[] {
  const out: Raw[] = [];
  const add = (steers: string, lens: number[], source: string) => out.push({ steers: steers.split('') as RsSteer[], lens, source });
  const sym = [
    { name: 'base', sx: 1, sy: 1, sp: 1, flip: false, refl: false },
    { name: 'timeflip', sx: -1, sy: 1, sp: -1, flip: true, refl: false },
    { name: 'reflect', sx: 1, sy: -1, sp: -1, flip: false, refl: true },
    { name: 'timeflip+reflect', sx: -1, sy: -1, sp: 1, flip: true, refl: true },
  ];
  const swapLR = (w: string) => w.replace(/[LR]/g, (c) => (c === 'L' ? 'R' : 'L'));
  const xb = x * Math.cos(phi) + y * Math.sin(phi);
  const yb = x * Math.sin(phi) - y * Math.cos(phi);
  for (const s of sym) {
    const sg = s.flip ? -1 : 1;
    const word = (w: string) => (s.refl ? swapLR(w) : w);
    // CSC
    let r = LpSpLp(s.sx * x, s.sy * y, s.sp * phi);
    if (r) add(word('LSL'), r.map((l) => sg * l), `LpSpLp/${s.name}`);
    r = LpSpRp(s.sx * x, s.sy * y, s.sp * phi);
    if (r) add(word('LSR'), r.map((l) => sg * l), `LpSpRp/${s.name}`);
    // CCC and its backwards reading
    r = LpRmL(s.sx * x, s.sy * y, s.sp * phi);
    if (r) add(word('LRL'), r.map((l) => sg * l), `LpRmL/${s.name}`);
    r = LpRmL(s.sx * xb, s.sy * yb, s.sp * phi);
    if (r) add(word('LRL'), [r[2], r[1], r[0]].map((l) => sg * l), `LpRmL-backwards/${s.name}`);
    // CCCC
    r = LpRupLumRm(s.sx * x, s.sy * y, s.sp * phi);
    if (r) add(word('LRLR'), [r[0], r[1], -r[1], r[2]].map((l) => sg * l), `LpRupLumRm/${s.name}`);
    r = LpRumLumRp(s.sx * x, s.sy * y, s.sp * phi);
    if (r) add(word('LRLR'), [r[0], r[1], r[1], r[2]].map((l) => sg * l), `LpRumLumRp/${s.name}`);
    // CCSC and its backwards reading
    r = LpRmSmLm(s.sx * x, s.sy * y, s.sp * phi);
    if (r) add(word('LRSL'), [r[0], -H, r[1], r[2]].map((l) => sg * l), `LpRmSmLm/${s.name}`);
    r = LpRmSmRm(s.sx * x, s.sy * y, s.sp * phi);
    if (r) add(word('LRSR'), [r[0], -H, r[1], r[2]].map((l) => sg * l), `LpRmSmRm/${s.name}`);
    r = LpRmSmLm(s.sx * xb, s.sy * yb, s.sp * phi);
    if (r) add(word('LSRL'), [r[2], r[1], -H, r[0]].map((l) => sg * l), `LpRmSmLm-backwards/${s.name}`);
    r = LpRmSmRm(s.sx * xb, s.sy * yb, s.sp * phi);
    if (r) add(word('RSRL'), [r[2], r[1], -H, r[0]].map((l) => sg * l), `LpRmSmRm-backwards/${s.name}`);
    // CCSCC
    r = LpRmSLmRp(s.sx * x, s.sy * y, s.sp * phi);
    if (r) add(word('LRSLR'), [r[0], -H, r[1], -H, r[2]].map((l) => sg * l), `LpRmSLmRp/${s.name}`);
  }
  return out;
}

/* -------------------------------------------------------------------------- */
/* Words, families, integration                                                 */
/* -------------------------------------------------------------------------- */

function wordOf(segs: readonly RsSegment[]): string {
  return segs.map((s) => `${s.steer}${s.len >= 0 ? '+' : '-'}`).join('');
}

function cuspsOf(segs: readonly RsSegment[]): number {
  let c = 0;
  let prev = 0;
  for (const s of segs) {
    if (Math.abs(s.len) < 1e-10) continue;
    const sg = Math.sign(s.len);
    if (prev !== 0 && sg !== prev) c++;
    prev = sg;
  }
  return c;
}

/** Choset's family label, read off the letters and the cusp positions. */
export function rsFamily(segs: readonly RsSegment[]): RsFamily {
  const letters = segs.map((s) => (s.steer === 'S' ? 'S' : 'C')).join('');
  const signs = segs.map((s) => Math.sign(s.len) || 1);
  let w = '';
  for (let i = 0; i < letters.length; i++) {
    if (i > 0 && signs[i] !== signs[i - 1]) w += '|';
    w += letters[i];
  }
  // Collapse onto the nine families (C_a, C_{π/2} subscripts dropped).
  switch (w) {
    case 'CSC':
      return 'CSC';
    case 'C|C|C':
      return 'C|C|C';
    case 'CC|C':
      return 'CC|C';
    case 'C|CC':
      return 'C|CC';
    case 'CC|CC':
      return 'CC|CC';
    case 'C|CC|C':
      return 'C|CC|C';
    case 'C|CSC':
      return 'C|CSC';
    case 'CSC|C':
      return 'CSC|C';
    case 'C|CSC|C':
      return 'C|CSC|C';
    case 'CCC':
      return 'C|CC'; // a zero-length cusp: L+R-L+ with u → 0 is degenerate
    default:
      // Degenerate words (a segment of length ~0 merging two families): fall back by cusp count.
      return letters.includes('S') ? (w.startsWith('C|') ? 'C|CSC' : 'CSC|C') : 'C|C|C';
  }
}

/** Every candidate as a path in world units for the query q0 → q1. */
export function rsAll(q0: Pose2, q1: Pose2, rho: number): RsPath[] {
  const dx = q1.x - q0.x;
  const dy = q1.y - q0.y;
  const c = Math.cos(q0.theta);
  const s = Math.sin(q0.theta);
  const x = (c * dx + s * dy) / rho;
  const y = (-s * dx + c * dy) / rho;
  const phi = normalizeAngle(q1.theta - q0.theta);
  const raws = rsCandidatesNormalized(x, y, phi);
  return raws
    .map((r) => {
      // Drop degenerate zero-length segments so the word and family read correctly.
      const segments = r.steers.map((steer, i) => ({ steer, len: r.lens[i] })).filter((g) => Math.abs(g.len) > 1e-10);
      const length = rho * segments.reduce((a, g) => a + Math.abs(g.len), 0);
      return { segments, rho, length, word: wordOf(segments), family: rsFamily(segments), cusps: cuspsOf(segments), source: r.source };
    })
    .sort((a, b) => a.length - b.length);
}

/** The shortest Reeds–Shepp path (never null for ρ > 0: CSC words always cover some reading). */
export function reedsSheppShortestPath(q0: Pose2, q1: Pose2, rho: number): RsPath | null {
  const all = rsAll(q0, q1, rho);
  return all.length ? all[0] : null;
}

/** Length only — the hybrid-A* heuristic and the RRT* edge cost. */
export function reedsSheppLength(q0: Pose2, q1: Pose2, rho: number): number {
  const p = reedsSheppShortestPath(q0, q1, rho);
  return p ? p.length : Infinity;
}

/** One segment from q: an arc of signed angle `len` (L or R) or a straight of signed ρ·len. */
export function rsSegmentEnd(q: Pose2, steer: RsSteer, len: number, rho: number): Pose2 {
  if (steer === 'S') return boxplus(q, [rho * len, 0, 0]);
  return boxplus(q, [rho * len, 0, steer === 'L' ? len : -len]);
}

/** Integrate the word from q0 — the cross-check's endpoint. */
export function rsEndpoint(path: RsPath, q0: Pose2): Pose2 {
  let q = q0;
  for (const g of path.segments) q = rsSegmentEnd(q, g.steer, g.len, path.rho);
  return q;
}

export interface RsSample {
  q: Pose2;
  /** Signed arc length from the previous sample (negative: reversing). */
  ds: number;
  steer: RsSteer;
}

/** Poses along the path at arc-length spacing ≤ ds, with the signed step that reached each. */
export function rsSample(path: RsPath, q0: Pose2, ds: number): RsSample[] {
  const out: RsSample[] = [{ q: q0, ds: 0, steer: 'S' }];
  let q = q0;
  for (const g of path.segments) {
    const L = Math.abs(g.len) * path.rho;
    if (L < 1e-12) continue;
    const n = Math.max(1, Math.ceil(L / ds));
    for (let k = 1; k <= n; k++) {
      out.push({ q: rsSegmentEnd(q, g.steer, (g.len * k) / n, path.rho), ds: (Math.sign(g.len) * L) / n, steer: g.steer });
    }
    q = rsSegmentEnd(q, g.steer, g.len, path.rho);
  }
  return out;
}

/** The pose at arc length s along the path. */
export function rsAt(path: RsPath, q0: Pose2, s: number): Pose2 {
  let q = q0;
  let rem = Math.max(0, Math.min(path.length, s)) / path.rho;
  for (const g of path.segments) {
    const a = Math.abs(g.len);
    if (rem <= a) return rsSegmentEnd(q, g.steer, Math.sign(g.len) * rem, path.rho);
    q = rsSegmentEnd(q, g.steer, g.len, path.rho);
    rem -= a;
  }
  return q;
}

/** A straight-forward truncation: the first `maxLen` of the path as its own RsPath (prefixes are feasible). */
export function rsTruncate(path: RsPath, maxLen: number): RsPath {
  if (maxLen >= path.length) return path;
  let rem = maxLen / path.rho;
  const segments: RsSegment[] = [];
  for (const g of path.segments) {
    const a = Math.abs(g.len);
    if (rem <= 0) break;
    const take = Math.min(a, rem);
    segments.push({ steer: g.steer, len: Math.sign(g.len) * take });
    rem -= take;
  }
  const length = path.rho * segments.reduce((s, g) => s + Math.abs(g.len), 0);
  return { ...path, segments, length, word: wordOf(segments), cusps: cuspsOf(segments) };
}
