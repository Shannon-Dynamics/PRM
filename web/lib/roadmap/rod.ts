/**
 * The rod-HGVG, sketched — Choset §5.4.
 *
 * The robot is a line segment (a rod) of length L in the plane; Q = SE(2) is
 * three-dimensional, so "equidistant to three obstacles" is again a curve —
 * but distance is measured in the *workspace*, d_i(q) = min_{r ∈ R(q), c ∈
 * WO_i} d(r, c), which is what a sensor on the rod can measure. The rod
 * two-equidistant face RF_ij = {d_i = d_j, ∇d_i ≠ ∇d_j} and the rod-GVG edge
 * RF_ijk = RF_ij ∩ RF_ik ∩ RF_jk follow the same preimage-theorem argument
 * as the point GVG, with the gradient taken in (x, y, θ). For a rod that is
 * small next to the obstacles each RF_ijk is one closed curve: the rod can
 * spin a full turn while staying three-way equidistant (footnote 5). The
 * R-edges — rod placements tangent to the planar point-GVD — link the
 * rod-GVG edges into a roadmap; here they are sampled from Chapter 8's GVD.
 *
 * This file is the stretch exercise's scaffold and the figure's data; it
 * does not implement the junction-region bookkeeping of [C ref. 107].
 */

import { distanceSegmentPolygon, distanceSegmentSegment, type Polygon } from '../collide/polygon';
import type { Point2, Segment } from '../sim/world';
import { constraint, solveSquare, type Nearest, type NearestQuery, type Vec } from './gvg';
import type { Gvd } from './gvd';

export type RodObstacle = { kind: 'polygon'; poly: Polygon } | { kind: 'segment'; s: Segment };

/** The rod's endpoints at configuration q = (x, y, θ). */
export function rodSegment(q: Vec, L: number): Segment {
  const c = Math.cos(q[2]);
  const s = Math.sin(q[2]);
  return { x1: q[0] - (L / 2) * c, y1: q[1] - (L / 2) * s, x2: q[0] + (L / 2) * c, y2: q[1] + (L / 2) * s };
}

/**
 * d_i(q) and its gradient in (x, y, θ). With r the closest rod point and c
 * the closest obstacle point, ∂d/∂(x, y) = (r − c)/d, and since r = centre
 * + s·u(θ) with s the signed offset along the rod, ∂d/∂θ = (r − c)/d · s u'(θ).
 */
export function rodDistance(q: Vec, L: number, obstacle: RodObstacle, index: number): Nearest {
  const seg = rodSegment(q, L);
  const w = obstacle.kind === 'polygon' ? distanceSegmentPolygon(seg, obstacle.poly) : distanceSegmentSegment(seg, obstacle.s);
  const d = w.dist;
  if (d < 1e-15) return { d: 0, grad: [0, 0, 0], closest: [w.onObstacle.x, w.onObstacle.y], index };
  const gx = (w.onRobot.x - w.onObstacle.x) / d;
  const gy = (w.onRobot.y - w.onObstacle.y) / d;
  const u = [Math.cos(q[2]), Math.sin(q[2])];
  const s = (w.onRobot.x - q[0]) * u[0] + (w.onRobot.y - q[1]) * u[1];
  const gth = gx * (-s * u[1]) + gy * (s * u[0]);
  return { d, grad: [gx, gy, gth], closest: [w.onObstacle.x, w.onObstacle.y], index };
}

/** The rod's nearest-obstacle query over SE(2). */
export function rodQuery(obstacles: readonly RodObstacle[], L: number): NearestQuery {
  return (q) => obstacles.map((o, i) => rodDistance(q, L, o, i)).sort((a, b) => a.d - b.d);
}

export interface RodPlacement {
  q: Vec;
  /** The three tied obstacles. */
  triple: [number, number, number];
  /** Their common distance. */
  d: number;
}

/**
 * Sample the rod-GVG edge RF_ijk through a point-GVD meet point: for each θ
 * on a grid, Newton in (x, y) on G = [d_i − d_j, d_i − d_k] with θ fixed,
 * seeded at the meet point, accepted when the triple is the three nearest.
 * For a small rod this parametrizes the whole S¹ of RF_ijk by θ.
 */
export function sampleRodEdge(query: NearestQuery, seed: Point2, triple: [number, number, number], nTheta = 36, iters = 40): RodPlacement[] {
  const out: RodPlacement[] = [];
  let x = seed.x;
  let y = seed.y;
  for (let k = 0; k < nTheta; k++) {
    const th = (2 * Math.PI * k) / nTheta;
    let ok = false;
    let q: Vec = [x, y, th];
    for (let it = 0; it < iters; it++) {
      const near = query(q);
      const byIdx = (i: number) => near.find((n) => n.index === i)!;
      const a = byIdx(triple[0]);
      const b = byIdx(triple[1]);
      const c = byIdx(triple[2]);
      const G = [a.d - b.d, a.d - c.d];
      if (Math.hypot(G[0], G[1]) < 1e-11) {
        ok = true;
        break;
      }
      const J = [
        [a.grad[0] - b.grad[0], a.grad[1] - b.grad[1]],
        [a.grad[0] - c.grad[0], a.grad[1] - c.grad[1]],
      ];
      const dq = solveSquare(J, [-G[0], -G[1]], 1e-12);
      if (!dq) break;
      const n = Math.hypot(dq[0], dq[1]);
      const cap = Math.max(0.5 * a.d, 1e-3);
      const f = n > cap ? cap / n : 1;
      q = [q[0] + f * dq[0], q[1] + f * dq[1], th];
    }
    if (!ok) continue;
    const near = query(q);
    const three = new Set([near[0].index, near[1].index, near[2].index]);
    if (!triple.every((i) => three.has(i))) continue;
    out.push({ q, triple, d: near[0].d });
    // Warm-start the next θ from this solution.
    x = q[0];
    y = q[1];
  }
  return out;
}

/** R-edges: rod placements centred on the planar point-GVD with the rod along the edge's tangent. */
export function rEdgePlacements(gvd: Gvd, every = 6): Vec[] {
  const out: Vec[] = [];
  for (const e of gvd.edges) {
    for (let k = 1; k + 1 < e.points.length; k += every) {
      const a = e.points[k - 1];
      const b = e.points[k + 1];
      out.push([e.points[k].x, e.points[k].y, Math.atan2(b.y - a.y, b.x - a.x)]);
    }
  }
  return out;
}

/** Worst |d_a − d_b| over the triple at each placement — the figure's honesty check. */
export function rodTieResidual(query: NearestQuery, placements: readonly RodPlacement[]): number {
  let worst = 0;
  for (const p of placements) {
    const near = query(p.q);
    const { G } = constraint(near, 3);
    worst = Math.max(worst, ...G.map(Math.abs));
  }
  return worst;
}
