/**
 * Guards: a statistic, a threshold, and a typed error (F3, F4, D23.2).
 *
 * A recovery in this stack is never magic. Each one starts with a number the
 * book already derived crossing a threshold someone has to defend:
 *
 *   singularity_guard  μ(q) = |det J(q)| = l₁ l₂ |sin θ₂|   (Chapter 4's Jacobian)
 *   jackknife_guard    φ = θ_car − ψ_trailer while reversing (Chapter 20's constraint)
 *   corridor_blocked   the first occupied cell within L_valid along the path (D23.2)
 *
 * They return `Result`-shaped values rather than booleans so that the caller
 * — the supervisor — must say what it does with the error, which is the whole
 * argument of D23.4.
 */

import { detJacobian, jacobian2, manipulability, svd2 } from '../cspace/jacobian';
import type { Reach, Tn } from '../robots/reach';
import { cellAt, type OccupancyGrid } from '../search/grid';
import { carTrailer } from '../nonholonomic/systems';
import type { Point2 } from '../sim/world';

export type GuardResult<E> = { ok: true } | { ok: false; err: E };

export interface Singular {
  kind: 'Singular';
  mu: number;
  /** σ_min(J): the joint-speed amplification bound is 1/σ_min (F3). */
  sigmaMin: number;
}

export interface Jackknife {
  kind: 'Jackknife';
  phi: number;
}

/** μ(q) and σ_min(J(q)) for Reach — Yoshikawa's measure and the F3 bound's denominator. */
export function armConditioning(arm: Reach, q: Tn): { mu: number; sigmaMin: number; det: number } {
  const svd = svd2(jacobian2(arm, q));
  return { mu: manipulability(arm, q), sigmaMin: svd.sigma[1], det: detJacobian(arm, q) };
}

/** F3: reject a goal whose manipulability is below μ_min. */
export function singularityGuard(arm: Reach, q: Tn, muMin: number): GuardResult<Singular> {
  const c = armConditioning(arm, q);
  return c.mu < muMin ? { ok: false, err: { kind: 'Singular', mu: c.mu, sigmaMin: c.sigmaMin } } : { ok: true };
}

/**
 * F4: fire while reversing (v < 0, where φ̇ ≈ −(v/d) φ is unstable) with
 * |φ| past φ_warn. Driving forward is the recovery, so a large φ with v > 0
 * is not an error — it is already decaying.
 */
export function jackknifeGuard(phi: number, v: number, phiWarn: number): GuardResult<Jackknife> {
  return v < 0 && Math.abs(phi) > phiWarn ? { ok: false, err: { kind: 'Jackknife', phi } } : { ok: true };
}

/**
 * F4's hitch-angle rate from Chapter 20's car-with-trailer drive field
 * g₁ = [cos θ, sin θ, 0, tan δ / L, sin(θ − ψ)/d]: with speed v,
 * φ̇ = v (g₁[θ] − g₁[ψ]) = (v/L) tan δ − (v/d) sin φ.
 */
export function hitchRate(phi: number, v: number, steer: number, wheelbase: number, d: number): number {
  const g = carTrailer(wheelbase, d).drive.eval([0, 0, steer, phi, 0]);
  return v * (g[3] - g[4]);
}

/**
 * The linearized hitch-angle rate of F4: φ̇ ≈ −(v/d) φ about φ = 0 with the
 * steering straight. The e-folding distance of a reversing trailer is d.
 */
export function hitchRateLinear(phi: number, v: number, d: number): number {
  return -(v / d) * phi;
}

export interface Blocked {
  kind: 'Blocked';
  cell: number;
  /** Arc length from the robot to the blocked cell along the path. */
  at: number;
  point: Point2;
}

/** Arc-length position of the point of `path` closest to `p` (projection onto the polyline). */
export function projectOnPath(path: readonly Point2[], p: Point2): { s: number; seg: number; t: number; d: number } {
  let best = { s: 0, seg: 0, t: 0, d: Infinity };
  let acc = 0;
  for (let i = 0; i + 1 < path.length; i++) {
    const a = path[i];
    const b = path[i + 1];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const L2 = dx * dx + dy * dy;
    const L = Math.sqrt(L2);
    const t = L2 > 0 ? Math.min(1, Math.max(0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2)) : 0;
    const d = Math.hypot(a.x + t * dx - p.x, a.y + t * dy - p.y);
    if (d < best.d) best = { s: acc + t * L, seg: i, t, d };
    acc += L;
  }
  return best;
}

/** The point at arc length s along a polyline (clamped to its ends). */
export function pointAtArc(path: readonly Point2[], s: number): Point2 {
  if (path.length === 0) return { x: 0, y: 0 };
  let acc = 0;
  for (let i = 0; i + 1 < path.length; i++) {
    const a = path[i];
    const b = path[i + 1];
    const L = Math.hypot(b.x - a.x, b.y - a.y);
    if (acc + L >= s) {
      const t = L > 0 ? (s - acc) / L : 0;
      return { x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y) };
    }
    acc += L;
  }
  return path[path.length - 1];
}

export function polylineLength(path: readonly Point2[]): number {
  let L = 0;
  for (let i = 1; i < path.length; i++) L += Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  return L;
}

/**
 * `corridor_blocked` (D23.2): walk the path from the robot's projection for
 * L_valid metres at half-cell spacing and report the first sample in an
 * occupied cell of the *newest* map — which may be newer than the map the
 * path was planned on. O(L_valid / Δ).
 */
export function corridorBlocked(path: readonly Point2[], grid: OccupancyGrid, from: Point2, lValid: number, ignoreNear = 0): GuardResult<Blocked> {
  if (path.length < 2) return { ok: true };
  const s0 = projectOnPath(path, from).s;
  const step = grid.cellSize / 2;
  const total = polylineLength(path);
  for (let s = s0 + step; s <= Math.min(total, s0 + lValid) + 1e-9; s += step) {
    const p = pointAtArc(path, s);
    // Cells the robot is already standing in were inflated around it, not across its path.
    if (Math.hypot(p.x - from.x, p.y - from.y) < ignoreNear) continue;
    const cell = cellAt(grid, p.x, p.y);
    if (grid.occupied[cell]) return { ok: false, err: { kind: 'Blocked', cell, at: s - s0, point: p } };
  }
  return { ok: true };
}
