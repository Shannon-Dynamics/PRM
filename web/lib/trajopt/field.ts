/**
 * Distance fields for CHOMP and the sequential-convex recipe: D(x) and ∇D(x)
 * for a point in the plane, *signed* where the geometry allows it (negative
 * inside an obstacle, so the obstacle cost keeps pushing).
 *
 *   DiscField     one analytic disc — the micro-example, so its numbers are exact.
 *   PlanarField   Chapter 7's exact planar query (`PlanarObstacles`): discs,
 *                 polygons, wall segments with witnesses. Inside a polygon the
 *                 distance is −(depth to the boundary), which Chapter 7's
 *                 unsigned query does not report; it is computed here from the
 *                 boundary witness.
 *   GridField     Chapter 7's brushfire grid, bilinearly interpolated so D is
 *                 continuous and ∇D piecewise constant — the field w19.1 draws
 *                 and descends on the Workbench.
 *   EdtField      the ported exact Euclidean distance transform of the Apartment
 *                 walls (mapping/edt), the same interpolation; used by the MPC
 *                 widget because the Apartment is all walls.
 */

import { pointInPolygon, polygonEdges } from '../collide/polygon';
import { distanceAt, exactDistanceField, type ExactDistanceField } from '../mapping/edt';
import { brushfire, type DistGrid } from '../potential/brushfire';
import type { OccupancyGrid } from '../search/grid';
import { PlanarObstacles, rasterizeObstacles } from '../potential/planar';
import type { Scene } from '../collide/scene';
import type { Bounds, Point2, World } from '../sim/world';
import type { PolyWorld } from '../robots/worlds';

export interface DistanceField2 {
  /** Signed distance to the nearest obstacle at a workspace point. */
  dist(x: Point2): number;
  /** ∇D, a unit vector pointing away from the nearest obstacle (zero where undefined). */
  grad(x: Point2): [number, number];
}

/** An analytic disc: D = ‖x − c‖ − r, ∇D radial. */
export class DiscField implements DistanceField2 {
  constructor(
    readonly c: Point2,
    readonly r: number,
  ) {}
  dist(x: Point2): number {
    return Math.hypot(x.x - this.c.x, x.y - this.c.y) - this.r;
  }
  grad(x: Point2): [number, number] {
    const dx = x.x - this.c.x;
    const dy = x.y - this.c.y;
    const n = Math.hypot(dx, dy);
    return n < 1e-12 ? [0, 1] : [dx / n, dy / n];
  }
}

/** The minimum over several fields — D(x) = min_i D_i(x), with the gradient of the minimizer. */
export class UnionField implements DistanceField2 {
  constructor(readonly parts: DistanceField2[]) {}
  private best(x: Point2): DistanceField2 | null {
    let best: DistanceField2 | null = null;
    let d = Infinity;
    for (const p of this.parts) {
      const di = p.dist(x);
      if (di < d) {
        d = di;
        best = p;
      }
    }
    return best;
  }
  dist(x: Point2): number {
    return Math.min(...this.parts.map((p) => p.dist(x)));
  }
  grad(x: Point2): [number, number] {
    const b = this.best(x);
    return b ? b.grad(x) : [0, 0];
  }
}

/** Chapter 7's exact planar query as a signed field. */
export class PlanarField implements DistanceField2 {
  constructor(readonly obstacles: PlanarObstacles) {}

  static fromScene(scene: Scene): PlanarField {
    return new PlanarField(PlanarObstacles.fromScene(scene));
  }

  private query(x: Point2): { d: number; grad: [number, number] } {
    let d = Infinity;
    let grad: [number, number] = [0, 0];
    this.obstacles.obstacles.forEach((o, index) => {
      if (o.kind === 'polygon' && pointInPolygon(x.x, x.y, o.poly)) {
        // Penetration depth: the boundary witness, with the gradient pointing *out* through it.
        const bw = boundaryDistance(x, o.poly);
        const di = -bw.d;
        if (di < d) {
          d = di;
          grad = bw.grad;
        }
        return;
      }
      const r = this.obstacles.distances([x.x, x.y])[index];
      if (r.d < d) {
        d = r.d;
        grad = [r.grad[0], r.grad[1]];
      }
    });
    return { d, grad };
  }

  dist(x: Point2): number {
    return this.query(x).d;
  }
  grad(x: Point2): [number, number] {
    return this.query(x).grad;
  }
}

/** Distance from an interior point to the polygon boundary, and the outward direction through the nearest edge. */
function boundaryDistance(x: Point2, poly: Point2[]): { d: number; grad: [number, number] } {
  let best = Infinity;
  let grad: [number, number] = [0, 0];
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    const ex = b.x - a.x;
    const ey = b.y - a.y;
    const L2 = ex * ex + ey * ey || 1e-18;
    const t = Math.max(0, Math.min(1, ((x.x - a.x) * ex + (x.y - a.y) * ey) / L2));
    const cx = a.x + t * ex;
    const cy = a.y + t * ey;
    const d = Math.hypot(x.x - cx, x.y - cy);
    if (d < best) {
      best = d;
      grad = d > 1e-12 ? [(cx - x.x) / d, (cy - x.y) / d] : [0, 0];
    }
  }
  return { d: best, grad };
}

/** Bilinear interpolation on a cell-centred scalar grid, with the central-difference gradient. */
function bilinear(
  nx: number,
  ny: number,
  cell: number,
  ox: number,
  oy: number,
  at: (i: number, j: number) => number,
  x: Point2,
): { d: number; grad: [number, number] } {
  const u = (x.x - ox) / cell - 0.5;
  const v = (x.y - oy) / cell - 0.5;
  const i0 = Math.min(nx - 2, Math.max(0, Math.floor(u)));
  const j0 = Math.min(ny - 2, Math.max(0, Math.floor(v)));
  const fu = Math.min(1, Math.max(0, u - i0));
  const fv = Math.min(1, Math.max(0, v - j0));
  const d00 = at(i0, j0);
  const d10 = at(i0 + 1, j0);
  const d01 = at(i0, j0 + 1);
  const d11 = at(i0 + 1, j0 + 1);
  const d = (1 - fu) * (1 - fv) * d00 + fu * (1 - fv) * d10 + (1 - fu) * fv * d01 + fu * fv * d11;
  const gx = ((1 - fv) * (d10 - d00) + fv * (d11 - d01)) / cell;
  const gy = ((1 - fu) * (d01 - d00) + fu * (d11 - d10)) / cell;
  const n = Math.hypot(gx, gy);
  return { d, grad: n < 1e-12 ? [0, 0] : [gx / n, gy / n] };
}

/**
 * Chapter 7's brushfire as a continuous *signed* field. Two brushfires: one
 * from the obstacle cells into free space and one from the free cells into
 * the obstacles, so that
 *     D = +(label − 1.5)·cell on free cells,   D = −(label_in − 1.5)·cell on occupied cells,
 * which puts the zero crossing on the cell boundary between them. Bilinear in
 * between, gradient by central differences of the interpolant. Without the
 * inside half the field would be flat zero inside an obstacle and CHOMP could
 * not push a body point out; Choset's §4.3.2 only needed the outside half.
 * With 8-connectivity the labels are the chessboard distance, which lies
 * between d/√2 and the Euclidean d — the check brackets it.
 */
export class GridField implements DistanceField2 {
  readonly outside: DistGrid;
  readonly inside: DistGrid;
  constructor(grid: OccupancyGrid) {
    this.outside = brushfire(grid, 8);
    const inv: OccupancyGrid = { ...grid, occupied: grid.occupied.map((v) => (v ? 0 : 1)) as Uint8Array };
    // If there are no obstacles at all the inverted grid is all occupied and brushfire has no seeds; that is fine (labels stay 0).
    this.inside = brushfire(inv, 8);
  }

  /**
   * Rasterize a polygon world (Workbench, Lot) at `cellSize`: a cell is occupied
   * when its centre lies inside a polygon or a wall segment passes through it
   * (zero-thickness walls would otherwise never own a cell centre).
   */
  static fromPolyWorld(world: PolyWorld, cellSize = 0.05): GridField {
    const obs = new PlanarObstacles(world.obstacles.map((o) => ({ kind: 'polygon' as const, poly: o.poly })));
    const grid = rasterizeObstacles(obs, world.bounds, cellSize, 0);
    const walls = [...world.walls, ...(world.obstacles.length ? [] : [])];
    for (const w of walls) {
      const len = Math.hypot(w.x2 - w.x1, w.y2 - w.y1);
      const steps = Math.max(1, Math.ceil((len / cellSize) * 3));
      for (let n = 0; n <= steps; n++) {
        const t = n / steps;
        const x = w.x1 + t * (w.x2 - w.x1);
        const y = w.y1 + t * (w.y2 - w.y1);
        const i = Math.min(grid.nx - 1, Math.max(0, Math.floor((x - grid.origin.x) / cellSize)));
        const j = Math.min(grid.ny - 1, Math.max(0, Math.floor((y - grid.origin.y) / cellSize)));
        grid.occupied[j * grid.nx + i] = 1;
      }
    }
    return new GridField(grid);
  }

  get grid(): OccupancyGrid {
    return this.outside.grid;
  }

  private at = (i: number, j: number): number => {
    const g = this.outside.grid;
    const k = j * g.nx + i;
    if (g.occupied[k]) {
      const lab = this.inside.label[k];
      return lab <= 1 ? -0.5 * g.cellSize : -(lab - 1.5) * g.cellSize;
    }
    const lab = this.outside.label[k];
    return lab <= 1 ? 0.5 * g.cellSize : (lab - 1.5) * g.cellSize;
  };

  dist(x: Point2): number {
    const g = this.outside.grid;
    return bilinear(g.nx, g.ny, g.cellSize, g.origin.x, g.origin.y, this.at, x).d;
  }
  grad(x: Point2): [number, number] {
    const g = this.outside.grid;
    return bilinear(g.nx, g.ny, g.cellSize, g.origin.x, g.origin.y, this.at, x).grad;
  }
}

/**
 * The ported exact Euclidean distance transform, bilinearly interpolated. For a
 * wall world (the Apartment) it is unsigned. For a polygon world the polygon
 * edges are rasterized as walls and the sign is flipped inside a polygon, which
 * gives a signed Euclidean field — the exact alternative to the brushfire's
 * chessboard metric, offered side by side in w19.1.
 */
export class EdtField implements DistanceField2 {
  constructor(
    readonly field: ExactDistanceField,
    readonly insidePolys: readonly Point2[][] = [],
  ) {}

  static fromWorld(world: World, cellSize = 0.05, bounds: Bounds = world.bounds): EdtField {
    return new EdtField(exactDistanceField(world, cellSize, bounds));
  }

  static fromPolyWorld(world: PolyWorld, cellSize = 0.05): EdtField {
    const walls = [...world.walls, ...world.obstacles.flatMap((o) => polygonEdges(o.poly))];
    const field = exactDistanceField({ name: `${world.name} edges`, bounds: world.bounds, walls }, cellSize, world.bounds);
    return new EdtField(field, world.obstacles.map((o) => o.poly));
  }

  private sign(x: Point2): number {
    for (const poly of this.insidePolys) if (pointInPolygon(x.x, x.y, poly)) return -1;
    return 1;
  }

  private at = (i: number, j: number): number => this.field.data[j * this.field.nx + i];

  dist(x: Point2): number {
    const f = this.field;
    return this.sign(x) * bilinear(f.nx, f.ny, f.cellSize, f.bounds.minX, f.bounds.minY, this.at, x).d;
  }
  grad(x: Point2): [number, number] {
    const f = this.field;
    const g = bilinear(f.nx, f.ny, f.cellSize, f.bounds.minX, f.bounds.minY, this.at, x).grad;
    const s = this.sign(x);
    return [s * g[0], s * g[1]];
  }
  /** Nearest-cell lookup, for comparison with the interpolant. */
  nearest(x: Point2): number {
    return distanceAt(this.field, x.x, x.y);
  }
}
