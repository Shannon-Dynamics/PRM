/**
 * The data factory: randomized Workbenches, their coarse encodings, and seeded
 * queries for Reach.
 *
 * Two generators, on purpose. `train` scatters axis-aligned blocks the way the
 * real Workbench does (Chapter 2's block, post and shelf are all rectangles);
 * `heldout` lays down long thin bars at arbitrary angles — a family of shapes
 * the training generator never produces. Everything the chapter says about
 * distribution shift is measured between these two.
 *
 * The environment encoding 𝓔 is the smallest thing a net can condition on: an
 * 8 × 8 occupancy raster of the table, each cell the fraction of four probe
 * points inside an obstacle. It is a *workspace* raster — the net has to learn
 * what Chapter 4 computes, the map from workspace obstacles to C-obstacles on
 * T². That gap is the whole reason the sampler can be wrong.
 */

import { Rng } from '../prob/rng';
import { orientedRect, rect, type Polygon } from '../collide/polygon';
import { Scene } from '../collide/scene';
import { WORKBENCH, type NamedPolygon, type PolyWorld } from '../robots/worlds';
import { torusLab, type TorusLab } from '../sampling/worlds';
import { StraightLine } from '../sampling/steer';

export type WorldGenerator = 'train' | 'heldout';

/** Side of the square encoding raster; the net sees ENC_N² numbers. */
export const ENC_N = 8;
export const ENC_DIM = ENC_N * ENC_N;

export type EnvEncoding = Float64Array;

const BASE_CLEARANCE = 0.32;
const REACH_RADIUS = 1.95;

/** Does the polygon come within `BASE_CLEARANCE` of the arm's base? Reject those — Reach must be able to move at all. */
function nearBase(poly: Polygon): boolean {
  for (const v of poly) if (Math.hypot(v.x, v.y) < BASE_CLEARANCE) return true;
  // The base could sit inside a large polygon with distant vertices; test the centroid distance too.
  const cx = poly.reduce((s, v) => s + v.x, 0) / poly.length;
  const cy = poly.reduce((s, v) => s + v.y, 0) / poly.length;
  return Math.hypot(cx, cy) < BASE_CLEARANCE;
}

/** Keep polygons inside the table so a wall never slices one. */
function insideTable(poly: Polygon): boolean {
  const b = WORKBENCH.bounds;
  return poly.every((v) => v.x > b.minX + 0.05 && v.x < b.maxX - 0.05 && v.y > b.minY + 0.05 && v.y < b.maxY - 0.05);
}

function trainObstacles(rng: Rng): NamedPolygon[] {
  const n = 4 + Math.floor(rng.next() * 3); // 4–6 blocks
  const out: NamedPolygon[] = [];
  let guard = 0;
  while (out.length < n && guard++ < 300) {
    const r = rng.uniform(0.55, REACH_RADIUS);
    const phi = rng.uniform(-Math.PI, Math.PI);
    const cx = r * Math.cos(phi);
    const cy = r * Math.sin(phi);
    const w = rng.uniform(0.25, 0.7);
    const h = rng.uniform(0.25, 0.7);
    const poly = rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2);
    if (nearBase(poly) || !insideTable(poly)) continue;
    out.push({ name: `block ${out.length + 1}`, poly });
  }
  return out;
}

function heldoutObstacles(rng: Rng): NamedPolygon[] {
  const n = 3 + Math.floor(rng.next() * 2); // 3–4 bars
  const out: NamedPolygon[] = [];
  let guard = 0;
  while (out.length < n && guard++ < 300) {
    const r = rng.uniform(0.7, REACH_RADIUS);
    const phi = rng.uniform(-Math.PI, Math.PI);
    const cx = r * Math.cos(phi);
    const cy = r * Math.sin(phi);
    const halfLength = rng.uniform(0.55, 0.9);
    const halfWidth = 0.07;
    const theta = rng.uniform(-Math.PI, Math.PI);
    const poly = orientedRect(cx, cy, halfLength, halfWidth, theta);
    if (nearBase(poly) || !insideTable(poly)) continue;
    out.push({ name: `bar ${out.length + 1}`, poly });
  }
  return out;
}

/** A seeded Workbench from one of the two generators. */
export function randomWorkbench(rng: Rng, gen: WorldGenerator): PolyWorld {
  const obstacles = gen === 'train' ? trainObstacles(rng) : heldoutObstacles(rng);
  return {
    ...WORKBENCH,
    name: gen === 'train' ? 'Workbench (random blocks)' : 'Workbench (held-out bars)',
    obstacles,
    labels: [],
  };
}

/** The 8 × 8 occupancy fraction raster of a Workbench. */
export function encodeWorld(world: PolyWorld): EnvEncoding {
  const scene = Scene.fromWorld(world);
  const b = world.bounds;
  const cw = (b.maxX - b.minX) / ENC_N;
  const ch = (b.maxY - b.minY) / ENC_N;
  const enc = new Float64Array(ENC_DIM);
  for (let j = 0; j < ENC_N; j++) {
    for (let i = 0; i < ENC_N; i++) {
      let hit = 0;
      for (const [u, v] of [
        [0.25, 0.25],
        [0.75, 0.25],
        [0.25, 0.75],
        [0.75, 0.75],
      ]) {
        if (scene.pointInObstacle(b.minX + (i + u) * cw, b.minY + (j + v) * ch)) hit++;
      }
      enc[j * ENC_N + i] = hit / 4;
    }
  }
  return enc;
}

/** cos/sin of each joint angle: the wrap-aware encoding of a point of T². */
export function encodeAngles(q: readonly number[]): number[] {
  const out: number[] = [];
  for (const a of q) out.push(Math.cos(a), Math.sin(a));
  return out;
}

/** A free configuration drawn uniformly, or null after `maxTries`. */
export function randomFreeConfig(lab: TorusLab, rng: Rng, maxTries = 500): number[] | null {
  for (let t = 0; t < maxTries; t++) {
    const q = lab.space.sample(rng);
    if (lab.free.isFree(q)) return q;
  }
  return null;
}

/**
 * A start/goal pair at least `minDist` apart on T² (both free) whose straight
 * geodesic is *blocked* — a query that needs a planner. Pairs the local planner
 * joins directly say nothing about any sampler, so they are rejected here.
 */
export function randomQuery(lab: TorusLab, rng: Rng, minDist = 2.0, requireBlocked = true): [number[], number[]] | null {
  const steer = new StraightLine<number[]>({ kind: 'subdivision', step: 0.1 });
  for (let t = 0; t < 400; t++) {
    const s = randomFreeConfig(lab, rng);
    const g = randomFreeConfig(lab, rng);
    if (!s || !g) return null;
    if (lab.space.dist(s, g) < minDist) continue;
    if (requireBlocked && steer.steer(lab.space, lab.free, s, g) !== null) continue;
    return [s, g];
  }
  return null;
}

/** A world, its lab and its encoding, together: what every learner consumes. */
export interface WorldCase {
  world: PolyWorld;
  lab: TorusLab;
  enc: EnvEncoding;
  gen: WorldGenerator;
  seed: number;
}

export function makeWorldCase(seed: number, gen: WorldGenerator): WorldCase {
  const rng = new Rng(seed);
  const world = randomWorkbench(rng, gen);
  return { world, lab: torusLab(world), enc: encodeWorld(world), gen, seed };
}
