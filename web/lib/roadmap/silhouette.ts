/**
 * Canny's roadmap on an implicit surface in R³ — Choset §5.5.1.
 *
 * The configuration space is a surface S = f⁻¹(0). Sweep the slice
 * Q_λ = π₁⁻¹(λ) along q₁; on each slice the surface is a set of closed
 * curves, and the *extrema of π₂* on them are the silhouette points. By
 * Lemma 5.5.2 these are where D(f, π₁₂) loses rank — rows ∇f, e₁, e₂ — i.e.
 * where ∂f/∂q₃ = 0 on the slice curve: the curve's tangent has no q₂
 * component. The slice lemma says the union over λ is exactly the critical
 * set Σ(π₁₂|S): the silhouette.
 *
 * The silhouette is not connected in general. Where the number of slice
 * curves or of extrema changes — a *critical slice*, where some silhouette
 * curve is tangent to the slice — the algorithm recurses on the slice with
 * one dimension fewer. The slice here is two-dimensional and the surface
 * meets it in curves, so the recursion's roadmap is those curves themselves
 * ("the roadmap of a one-dimensional set is the set itself"): the *linking
 * curves*. Silhouette plus linking curves is Canny's roadmap.
 *
 * The slice curves are traced with Chapter 3's predictor–corrector on
 * G(p) = f(λ, p), p = (q₂, q₃): nothing about tracing is new. Everything
 * here is for two bodies — a torus (C Figs. 5.31–5.32) and an ellipsoid
 * with a bent hole (C Fig. 5.25) — and for the sphere whose silhouette the
 * check pins to the equator.
 */

import { traceCurve, correctGradient } from '../bugs/trace';
import type { Point2 } from '../sim/world';
import type { Vec } from './gvg';

export interface ImplicitSurface {
  name: string;
  /** f(q) < 0 inside the body, = 0 on the surface. */
  f: (q: Vec) => number;
  grad: (q: Vec) => Vec;
  /** A box containing the body, per axis. */
  bounds: [number, number][];
}

/* -------------------------------------------------------------------------- */
/* Bodies                                                                      */
/* -------------------------------------------------------------------------- */

export function sphere(r = 1): ImplicitSurface {
  return {
    name: 'sphere',
    f: (q) => q[0] * q[0] + q[1] * q[1] + q[2] * q[2] - r * r,
    grad: (q) => [2 * q[0], 2 * q[1], 2 * q[2]],
    bounds: [[-r, r], [-r, r], [-r, r]],
  };
}

/** A torus with axis q₃, major radius R, tube radius r. */
export function torus(R = 2, r = 0.7): ImplicitSurface {
  return {
    name: 'torus',
    f: (q) => {
      const rho = Math.hypot(q[0], q[1]);
      return (rho - R) * (rho - R) + q[2] * q[2] - r * r;
    },
    grad: (q) => {
      const rho = Math.hypot(q[0], q[1]) || 1e-12;
      const k = (2 * (rho - R)) / rho;
      return [k * q[0], k * q[1], 2 * q[2]];
    },
    bounds: [[-(R + r), R + r], [-(R + r), R + r], [-r, r]],
  };
}

/**
 * An ellipsoid with a hole drilled along q₃ whose axis bends with height,
 * y_c(z) = κ z² — C Fig. 5.25's body, simplified. The bend is not
 * decoration: with a straight hole the tube's wall is a cylinder, π₂ is
 * constant along its vertical lines, and the extrema are degenerate (not
 * Morse). Bending it makes each slice's hole-wall curve a parabola with one
 * nondegenerate extremum. Solid = ellipsoid ∩ (outside the tube), written
 * with a smooth maximum so that f is C^∞ and Newton has a gradient to use.
 */
export function drilledEllipsoid(a = 3, b = 2, c = 1.5, holeX = 0.6, holeR = 0.7, bend = 0.5, rounding = 0.25): ImplicitSurface {
  const E = (q: Vec) => (q[0] / a) ** 2 + (q[1] / b) ** 2 + (q[2] / c) ** 2 - 1;
  const gE = (q: Vec): Vec => [(2 * q[0]) / (a * a), (2 * q[1]) / (b * b), (2 * q[2]) / (c * c)];
  const yc = (z: number) => bend * z * z;
  const T = (q: Vec) => ((q[0] - holeX) / holeR) ** 2 + ((q[1] - yc(q[2])) / holeR) ** 2 - 1;
  const gT = (q: Vec): Vec => {
    const u = (q[1] - yc(q[2])) / holeR;
    return [(2 * (q[0] - holeX)) / (holeR * holeR), (2 * u) / holeR, (2 * u * -2 * bend * q[2]) / holeR];
  };
  // smooth max(A, B) = (A + B + sqrt((A − B)² + ρ²)) / 2 with A = E, B = −T.
  return {
    name: 'drilled ellipsoid',
    f: (q) => {
      const A = E(q);
      const B = -T(q);
      return 0.5 * (A + B + Math.sqrt((A - B) * (A - B) + rounding * rounding));
    },
    grad: (q) => {
      const A = E(q);
      const B = -T(q);
      const s = Math.sqrt((A - B) * (A - B) + rounding * rounding);
      const wA = 0.5 * (1 + (A - B) / s);
      const wB = 0.5 * (1 - (A - B) / s);
      const ga = gE(q);
      const gb = gT(q).map((v) => -v);
      return [wA * ga[0] + wB * gb[0], wA * ga[1] + wB * gb[1], wA * ga[2] + wB * gb[2]];
    },
    bounds: [[-a, a], [-b, b], [-c, c]],
  };
}

/** The same body with its axes permuted, so the sweep direction can be changed without touching the sweep. */
export function permuteAxes(s: ImplicitSurface, perm: [number, number, number]): ImplicitSurface {
  // q' = P q with q'_i = q_{perm[i]}; f'(q') = f(q) where q_{perm[i]} = q'_i.
  const toOrig = (qp: Vec): Vec => {
    const q = [0, 0, 0];
    for (let i = 0; i < 3; i++) q[perm[i]] = qp[i];
    return q;
  };
  return {
    name: `${s.name} (axes ${perm.join('')})`,
    f: (qp) => s.f(toOrig(qp)),
    grad: (qp) => {
      const g = s.grad(toOrig(qp));
      return [g[perm[0]], g[perm[1]], g[perm[2]]];
    },
    bounds: [s.bounds[perm[0]], s.bounds[perm[1]], s.bounds[perm[2]]],
  };
}

/* -------------------------------------------------------------------------- */
/* One slice: its curves and their π₂-extrema                                   */
/* -------------------------------------------------------------------------- */

export interface SlicePiece {
  /** Samples of the closed slice curve, in slice coordinates p = (q₂, q₃). */
  points: Point2[];
  closed: boolean;
}

export interface SliceResult {
  lambda: number;
  pieces: SlicePiece[];
  /** Extrema of q₂ on the slice curves, as 3-D points, with the piece they belong to and whether they are maxima. */
  extrema: { q: Vec; piece: number; max: boolean }[];
}

export interface SweepOptions {
  /** Tracing step in the slice plane. */
  step?: number;
  /** Seeds per axis in the slice plane. */
  seeds?: number;
}

/**
 * Trace every closed curve of S ∩ Q_λ with Chapter 3's tracer on
 * G(p) = f(λ, p), then find the extrema of q₂ on each as the sign changes of
 * ∂f/∂q₃ along the curve (the tangent (−f₃, f₂) has zero q₂-component there),
 * refined by bisection on the curve's parameter.
 */
export function sliceCurves(s: ImplicitSurface, lambda: number, opts: SweepOptions = {}): SliceResult {
  const { step = 0.03, seeds = 14 } = opts;
  const [b2, b3] = [s.bounds[1], s.bounds[2]];
  const G = (p: Point2) => s.f([lambda, p.x, p.y]);
  const DG = (p: Point2): Point2 => {
    const g = s.grad([lambda, p.x, p.y]);
    return { x: g[1], y: g[2] };
  };
  const pieces: SlicePiece[] = [];
  const nearExisting = (p: Point2) => pieces.some((pc) => pc.points.some((q) => Math.hypot(q.x - p.x, q.y - p.y) < 2 * step));
  const inBox = (p: Point2) => p.x >= b2[0] - 1 && p.x <= b2[1] + 1 && p.y >= b3[0] - 1 && p.y <= b3[1] + 1;
  for (let i = 0; i < seeds; i++) {
    for (let j = 0; j < seeds; j++) {
      const seed = { x: b2[0] + ((i + 0.5) / seeds) * (b2[1] - b2[0]), y: b3[0] + ((j + 0.5) / seeds) * (b3[1] - b3[0]) };
      const g0 = DG(seed);
      if (Math.hypot(g0.x, g0.y) < 1e-9) continue;
      const on = correctGradient(G, DG, seed, 40, 1e-12);
      if (Math.abs(G(on)) > 1e-8 || !inBox(on) || nearExisting(on)) continue;
      const gOn = DG(on);
      if (Math.hypot(gOn.x, gOn.y) < 1e-6) continue; // a singular point of the slice curve (the critical slice itself)
      // Trace; where the corrector fails (a tight fillet) retry with a finer step, and as a last
      // resort trace both ways and join.
      let tr = traceCurve(G, DG, on, 'left', { step, newtonIters: 4, tol: 1e-12, singularTol: 1e-4, maxSteps: 6000 });
      if (tr.stop !== 'closed') tr = traceCurve(G, DG, on, 'left', { step: step / 4, newtonIters: 5, tol: 1e-12, singularTol: 1e-5, maxSteps: 24000 });
      let points = tr.points;
      let closed = tr.stop === 'closed';
      if (!closed) {
        const back = traceCurve(G, DG, on, 'right', { step: step / 4, newtonIters: 5, tol: 1e-12, singularTol: 1e-5, maxSteps: 24000 });
        points = [...back.points.slice().reverse(), ...tr.points.slice(1)];
      }
      if (points.length < 4) continue;
      // A duplicate of a piece already found (seeded elsewhere on the same curve) is dropped.
      const overlap = pieces.some((pc) => {
        let hits = 0;
        for (let k = 0; k < points.length; k += 4) if (pc.points.some((q) => Math.hypot(q.x - points[k].x, q.y - points[k].y) < 2 * step)) hits++;
        return hits > 0.5 * Math.ceil(points.length / 4);
      });
      if (overlap) continue;
      pieces.push({ points, closed });
    }
  }
  // Extrema of q₂ along each piece: sign changes of f₃ = ∂f/∂q₃.
  const extrema: SliceResult['extrema'] = [];
  pieces.forEach((pc, pi) => {
    const pts = pc.points;
    const n = pc.closed ? pts.length - 1 : pts.length;
    const f3 = (p: Point2) => s.grad([lambda, p.x, p.y])[2];
    for (let k = 0; k < n - (pc.closed ? 0 : 1); k++) {
      const a = pts[k];
      const b = pts[(k + 1) % n];
      const fa = f3(a);
      const fb = f3(b);
      if (fa === 0 || fa * fb > 0) continue;
      // Bisect along the chord, projecting back onto the curve each time.
      let lo = 0;
      let hi = 1;
      let p = a;
      for (let it = 0; it < 40; it++) {
        const mid = 0.5 * (lo + hi);
        p = correctGradient(G, DG, { x: a.x + mid * (b.x - a.x), y: a.y + mid * (b.y - a.y) }, 6, 1e-13);
        if (f3(p) * fa > 0) lo = mid;
        else hi = mid;
      }
      // Maximum or minimum of q₂: compare with the neighbouring samples (the outward normal alone
      // cannot tell, since the slice curves are not convex).
      extrema.push({ q: [lambda, p.x, p.y], piece: pi, max: p.x >= a.x && p.x >= b.x });
    }
  });
  return { lambda, pieces, extrema };
}

/* -------------------------------------------------------------------------- */
/* The sweep, critical slices, silhouette curves, linking curves               */
/* -------------------------------------------------------------------------- */

export interface CriticalSlice {
  lambda: number;
  /** (pieces, extrema) just before and just after. */
  before: [number, number];
  after: [number, number];
  /** Where the topology changed: the mean of the extrema that appear or vanish. */
  point: Vec;
  /** The slice curves on either side of λ*, as 3-D polylines: the recursion's roadmap. */
  linking: Vec[][];
}

export interface SilhouetteCurve {
  points: Vec[];
  /** Whether it is a locus of maxima (true) or minima of q₂. */
  max: boolean;
}

export interface Silhouette {
  surface: ImplicitSurface;
  lambdas: number[];
  slices: SliceResult[];
  critical: CriticalSlice[];
  curves: SilhouetteCurve[];
  /** Connected components of silhouette curves alone, and with linking curves. */
  componentsWithoutLinks: number;
  componentsWithLinks: number;
}

const signature = (sl: SliceResult): [number, number] => [sl.pieces.length, sl.extrema.length];

/**
 * Sweep the slice over [λ_min, λ_max] in `n` steps, locate every critical
 * value by bisection between slices whose signature differs, chain the
 * extrema into silhouette curves, attach the linking curves at critical
 * slices, and count connected components with and without them.
 */
export function cannySweep(s: ImplicitSurface, n = 120, opts: SweepOptions = {}): Silhouette {
  const [l0, l1] = s.bounds[0];
  const pad = 1e-3 * (l1 - l0);
  const lambdas: number[] = [];
  for (let k = 0; k <= n; k++) lambdas.push(l0 - pad + ((l1 - l0 + 2 * pad) * k) / n);
  const slices = lambdas.map((lam) => sliceCurves(s, lam, opts));
  const dl = lambdas[1] - lambdas[0];

  // Critical slices by bisection on the signature.
  const critical: CriticalSlice[] = [];
  for (let k = 1; k < slices.length; k++) {
    const sa = signature(slices[k - 1]);
    const sb = signature(slices[k]);
    if (sa[0] === sb[0] && sa[1] === sb[1]) continue;
    let lo = lambdas[k - 1];
    let hi = lambdas[k];
    let before = slices[k - 1];
    let after = slices[k];
    for (let it = 0; it < 8; it++) {
      const mid = 0.5 * (lo + hi);
      const sm = sliceCurves(s, mid, opts);
      const sig = signature(sm);
      if (sig[0] === sa[0] && sig[1] === sa[1]) {
        lo = mid;
        before = sm;
      } else {
        hi = mid;
        after = sm;
      }
    }
    const lam = 0.5 * (lo + hi);
    // The changing extrema: those on the richer side with no close partner on the poorer side.
    const rich = after.extrema.length >= before.extrema.length ? after : before;
    const poor = rich === after ? before : after;
    const changed = rich.extrema.filter((e) => !poor.extrema.some((o) => Math.hypot(e.q[1] - o.q[1], e.q[2] - o.q[2]) < 0.25));
    const point: Vec = changed.length ? [lam, changed.reduce((a, e) => a + e.q[1], 0) / changed.length, changed.reduce((a, e) => a + e.q[2], 0) / changed.length] : [lam, 0, 0];
    // The recursion's roadmap: the slice curves half a step on either side of λ* (at λ* itself the
    // curve is pinched and the tracer rightly refuses it).
    const linking: Vec[][] = [];
    for (const lamSide of [lam - 0.5 * dl, lam + 0.5 * dl]) {
      const side = sliceCurves(s, lamSide, opts);
      for (const pc of side.pieces) linking.push(pc.points.map((p) => [lamSide, p.x, p.y]));
    }
    critical.push({ lambda: lam, before: sa, after: sb, point, linking });
  }

  // Chain extrema across slices into curves (nearest neighbour within a jump bound).
  const curves: SilhouetteCurve[] = [];
  const open: { curve: SilhouetteCurve; last: Vec }[] = [];
  const jump = 0.6 * Math.max(dl * 8, 0.4);
  for (const sl of slices) {
    const used = new Set<number>();
    const stillOpen: typeof open = [];
    for (const o of open) {
      let best = -1;
      let bd = jump;
      sl.extrema.forEach((e, i) => {
        if (used.has(i) || e.max !== o.curve.max) return;
        const d = Math.hypot(e.q[1] - o.last[1], e.q[2] - o.last[2]);
        if (d < bd) {
          bd = d;
          best = i;
        }
      });
      if (best >= 0) {
        used.add(best);
        o.curve.points.push(sl.extrema[best].q);
        o.last = sl.extrema[best].q;
        stillOpen.push(o);
      }
    }
    sl.extrema.forEach((e, i) => {
      if (used.has(i)) return;
      const curve: SilhouetteCurve = { points: [e.q], max: e.max };
      curves.push(curve);
      stillOpen.push({ curve, last: e.q });
    });
    open.length = 0;
    open.push(...stillOpen);
  }
  const curvesKept = curves.filter((c) => c.points.length >= 2);

  // Connectivity: curves touching (endpoints within a jump) alone; then with linking curves.
  const near = (a: Vec, b: Vec, tol: number) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < tol;
  const touches = (c1: Vec[], c2: Vec[], tol: number) => {
    const ends = (c: Vec[]) => [c[0], c[c.length - 1]];
    for (const a of ends(c1)) for (const b of c2) if (near(a, b, tol)) return true;
    for (const a of ends(c2)) for (const b of c1) if (near(a, b, tol)) return true;
    return false;
  };
  const components = (items: Vec[][], tol: number) => {
    const n = items.length;
    const label = new Array<number>(n).fill(-1);
    let count = 0;
    for (let i = 0; i < n; i++) {
      if (label[i] >= 0) continue;
      const stack = [i];
      label[i] = count;
      while (stack.length) {
        const a = stack.pop()!;
        for (let b = 0; b < n; b++) if (label[b] < 0 && touches(items[a], items[b], tol)) {
          label[b] = count;
          stack.push(b);
        }
      }
      count++;
    }
    return count;
  };
  const tol = 2 * dl + 2 * (opts.step ?? 0.03);
  const curvePts = curvesKept.map((c) => c.points);
  const without = components(curvePts, tol);
  const links = critical.flatMap((c) => c.linking);
  // Curves attach to a linking curve when any of their samples near λ* lies on it.
  const all = [...curvePts, ...links];
  const touchesLink = (c: Vec[], L: Vec[], lam: number) => c.some((p) => Math.abs(p[0] - lam) <= 2.5 * dl && L.some((q) => Math.hypot(p[1] - q[1], p[2] - q[2]) < tol));
  const nAll = all.length;
  const label = new Array<number>(nAll).fill(-1);
  let count = 0;
  const linkLambda = (idx: number) => {
    let k = idx - curvePts.length;
    for (const c of critical) {
      if (k < c.linking.length) return c.lambda;
      k -= c.linking.length;
    }
    return NaN;
  };
  const adjacent = (a: number, b: number) => {
    const aCurve = a < curvePts.length;
    const bCurve = b < curvePts.length;
    if (aCurve && bCurve) return touches(all[a], all[b], tol);
    if (!aCurve && !bCurve) return Math.abs(linkLambda(a) - linkLambda(b)) < 1e-9 && touches(all[a], all[b], tol);
    const [c, L, lam] = aCurve ? [all[a], all[b], linkLambda(b)] : [all[b], all[a], linkLambda(a)];
    return touchesLink(c, L, lam);
  };
  for (let i = 0; i < nAll; i++) {
    if (label[i] >= 0) continue;
    const stack = [i];
    label[i] = count;
    while (stack.length) {
      const a = stack.pop()!;
      for (let b = 0; b < nAll; b++) if (label[b] < 0 && adjacent(a, b)) {
        label[b] = count;
        stack.push(b);
      }
    }
    count++;
  }
  // Only components that contain at least one silhouette curve count.
  const withLinks = new Set(curvePts.map((_, i) => label[i])).size;

  return { surface: s, lambdas, slices, critical, curves: curvesKept, componentsWithoutLinks: without, componentsWithLinks: withLinks };
}

/* -------------------------------------------------------------------------- */
/* Lemma 5.5.2 on the sphere: D(f, π₁₂) and its rank                            */
/* -------------------------------------------------------------------------- */

/** The stacked differential D(f, h) for h = π₁₂ at q: rows ∇f, e₁, e₂. Its rank drops iff ∂f/∂q₃ = 0. */
export function stackedDifferential(s: ImplicitSurface, q: Vec): number[][] {
  return [s.grad(q), [1, 0, 0], [0, 1, 0]];
}

/** |det D(f, π₁₂)| = |∂f/∂q₃|: zero exactly on the silhouette. */
export function rankDeficiency(s: ImplicitSurface, q: Vec): number {
  const M = stackedDifferential(s, q);
  return Math.abs(M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]));
}
