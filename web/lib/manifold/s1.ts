/**
 * S¹ — the circle, stored as a wrapped angle.
 *
 * A point of S¹ is not a number; a number is its coordinate in *some* chart
 * (Choset §3.4.2). We store the representative in (−π, π] and the type's
 * operations never let the reader see the seam: `dist` is the quotient metric
 * of ℝ by 2πℤ, `interpolate` goes the short way, `sample` is uniform on the
 * circle. The chapter's micro-example is the whole argument in two numbers —
 * d(0.1, 6.2) is 0.1832, not 6.1, and the midpoint is 0.0084, not 3.15.
 *
 * The second half of the file is Choset's four-chart atlas (figure 3.14) plus
 * the two-chart angle atlas of problem 3.9 and the one-chart "attempt" with its
 * seam, as data for the Atlas of the Circle widget.
 */

import { normalizeAngle } from '../geom/se2';
import type { Rng } from '../prob/rng';
import type { Chart } from './manifold';

export const TAU = 2 * Math.PI;

/** Canonical representative in (−π, π]. */
export const wrapAngle = normalizeAngle;

/**
 * d_{S¹}(a, b) = min(|a − b|, 2π − |a − b|). Writing it as |wrap(a − b)| makes
 * the quotient-metric reading literal: the distance between two angles is the
 * length of the shortest representative of their difference.
 */
export function s1Dist(a: number, b: number): number {
  return Math.abs(wrapAngle(a - b));
}

/**
 * Geodesic on S¹: travel the short way. The signed step wrap(b − a) ∈ (−π, π]
 * is unique except at the antipode, where |b − a| = π exactly; there the
 * representative is +π by the convention of `normalizeAngle`, so antipodal
 * interpolation deterministically goes counter-clockwise. Documented, not hidden.
 */
export function s1Interpolate(a: number, b: number, t: number): number {
  return wrapAngle(a + t * wrapAngle(b - a));
}

/** What the chart says if you forget the seam — the wrong answers the chapter prints beside the right ones. */
export const chartDistance = (a: number, b: number) => Math.abs(a - b);
export const chartMidpoint = (a: number, b: number) => (a + b) / 2;

export const S1: Chart<number> = {
  name: 'S¹',
  dim: 1,
  periodic: [true],
  period: TAU,
  dist: s1Dist,
  interpolate: s1Interpolate,
  sample: (rng: Rng) => rng.uniform(-Math.PI, Math.PI),
  wrap: wrapAngle,
  equals: (a, b, tol = 1e-9) => s1Dist(a, b) <= tol,
  coords: (p) => [wrapAngle(p)],
  fromCoords: (c) => wrapAngle(c[0]),
};

/* -------------------------------------------------------------------------- */
/* Atlases                                                                     */
/* -------------------------------------------------------------------------- */

/** A point of S¹ embedded in ℝ², as Choset writes it: x = (x₁, x₂) with x₁² + x₂² = 1. */
export const embedS1 = (theta: number): [number, number] => [Math.cos(theta), Math.sin(theta)];

export interface S1ChartDef {
  id: string;
  /** TeX-ish label for the UI, e.g. "U₁ = {x₂ > 0}". */
  label: string;
  /** Is the point inside the (open) domain? */
  contains(theta: number): boolean;
  /** The chart coordinate φᵢ(x), or null outside Uᵢ. */
  coord(theta: number): number | null;
  /** The parameterization φᵢ⁻¹(z), as an angle, for z in the chart's image. */
  inverse(z: number): number;
  /** The image of the chart in ℝ, for drawing the number line. */
  range: [number, number];
}

/**
 * Choset's four charts (§3.4.2): the open half-circles, each coordinatized by
 * the coordinate that varies monotonically on it. U₁ ∩ U₂ = U₃ ∩ U₄ = ∅, and on
 * the four remaining overlaps the transition maps are ±√(1 − z²), smooth on
 * open subintervals of (−1, 1) — which is all an atlas requires.
 */
export const FOUR_CHART_ATLAS: readonly S1ChartDef[] = [
  {
    id: 'U1',
    label: 'U₁ = {x₂ > 0}, φ₁ = x₁',
    contains: (th) => Math.sin(th) > 0,
    coord: (th) => (Math.sin(th) > 0 ? Math.cos(th) : null),
    inverse: (z) => Math.atan2(Math.sqrt(Math.max(0, 1 - z * z)), z),
    range: [-1, 1],
  },
  {
    id: 'U2',
    label: 'U₂ = {x₂ < 0}, φ₂ = x₁',
    contains: (th) => Math.sin(th) < 0,
    coord: (th) => (Math.sin(th) < 0 ? Math.cos(th) : null),
    inverse: (z) => Math.atan2(-Math.sqrt(Math.max(0, 1 - z * z)), z),
    range: [-1, 1],
  },
  {
    id: 'U3',
    label: 'U₃ = {x₁ > 0}, φ₃ = x₂',
    contains: (th) => Math.cos(th) > 0,
    coord: (th) => (Math.cos(th) > 0 ? Math.sin(th) : null),
    inverse: (z) => Math.atan2(z, Math.sqrt(Math.max(0, 1 - z * z))),
    range: [-1, 1],
  },
  {
    id: 'U4',
    label: 'U₄ = {x₁ < 0}, φ₄ = x₂',
    contains: (th) => Math.cos(th) < 0,
    coord: (th) => (Math.cos(th) < 0 ? Math.sin(th) : null),
    inverse: (z) => Math.atan2(z, -Math.sqrt(Math.max(0, 1 - z * z))),
    range: [-1, 1],
  },
];

/**
 * The two-chart angle atlas (Choset problem 3.9) that `S1` itself uses: the
 * angle measured from +x on the circle minus the point (−1, 0), and the angle
 * measured from −x on the circle minus (1, 0). Their transition map is a
 * translation by π on each component of the overlap — derivative 1, which is
 * why Derivation 5 can say the torus's Jacobian is the same in every chart.
 */
export const TWO_CHART_ATLAS: readonly S1ChartDef[] = [
  {
    id: 'V1',
    label: 'V₁ = S¹ \\ {(−1, 0)}, ψ₁ = angle from +x in (−π, π)',
    contains: (th) => Math.abs(wrapAngle(th)) < Math.PI - 1e-12,
    coord: (th) => (Math.abs(wrapAngle(th)) < Math.PI - 1e-12 ? wrapAngle(th) : null),
    inverse: (z) => z,
    range: [-Math.PI, Math.PI],
  },
  {
    id: 'V2',
    label: 'V₂ = S¹ \\ {(1, 0)}, ψ₂ = angle from −x in (−π, π)',
    contains: (th) => Math.abs(wrapAngle(th - Math.PI)) < Math.PI - 1e-12,
    coord: (th) => (Math.abs(wrapAngle(th - Math.PI)) < Math.PI - 1e-12 ? wrapAngle(th - Math.PI) : null),
    inverse: (z) => wrapAngle(z + Math.PI),
    range: [-Math.PI, Math.PI],
  },
];

/**
 * The single-chart attempt: θ ∈ [0, 2π). It covers every point, but its domain
 * is not open on the circle — the point (1, 0) has no neighbourhood on which
 * the map is continuous. Approaching the seam from below reads 6.28, from above
 * reads 0.00. A chart in name only; the widget shows the tear.
 */
export const SINGLE_CHART_ATTEMPT: S1ChartDef = {
  id: 'W',
  label: 'W = S¹, θ ∈ [0, 2π) — not a chart: the domain is not open',
  contains: () => true,
  coord: (th) => {
    const w = wrapAngle(th);
    return w < 0 ? w + TAU : w;
  },
  inverse: (z) => wrapAngle(z),
  range: [0, TAU],
};

/**
 * The transition map φᵢ ∘ φⱼ⁻¹ evaluated at z, or null if φⱼ⁻¹(z) ∉ Uᵢ. For the
 * four-chart atlas every value is ±√(1 − z²) on its domain — e.g.
 * φ₁ ∘ φ₃⁻¹(z) = √(1 − z²) on (0, 1), exactly Choset's worked case.
 */
export function transitionMap(atlas: readonly S1ChartDef[], i: number, j: number, z: number): number | null {
  const theta = atlas[j].inverse(z);
  if (!atlas[i].contains(theta)) return null;
  return atlas[i].coord(theta);
}

/** Which charts of an atlas contain a point — the cover property, as a list. */
export function chartsContaining(atlas: readonly S1ChartDef[], theta: number): S1ChartDef[] {
  return atlas.filter((c) => c.contains(theta));
}
