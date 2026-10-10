/**
 * The planner-side range sensor: Choset's saturated raw distance function
 *
 *   ρ_R(x, s) = ρ(x, s)  if ρ(x, s) < R,   ∞ otherwise          (C §2.2)
 *
 * sampled at n equally spaced bearings by exact ray casting with *no noise*.
 * This is deliberately not the sister book's LiDAR (`sim/rusty.ts`, with
 * Gaussian range noise and dropouts): the Bug guarantees of Chapter 3 assume
 * an ideal sensor, and the text says so. Chapter 3 owns the theory of
 * ρ, D(x) and ∇D built on top of this; this module owns only the geometry.
 */

import type { Point2 } from '../sim/world';
import type { Scene } from './scene';

export interface RangeSensor {
  /** Number of equally spaced rays over the full circle. 360 by default. */
  nRays: number;
  /** Saturation range R. Rays that travel farther report Infinity. */
  maxRange: number;
}

export const DEFAULT_RANGE_SENSOR: RangeSensor = { nRays: 360, maxRange: 4.0 };

/** Bearing of ray k: 2πk/n, so ray 0 looks along +x. */
export function rayBearings(sensor: RangeSensor): number[] {
  return Array.from({ length: sensor.nRays }, (_, k) => (2 * Math.PI * k) / sensor.nRays);
}

/** One scan: ρ_R(x, s_k) for every bearing; Infinity beyond R. */
export function scan(sensor: RangeSensor, scene: Scene, x: Point2): number[] {
  const out = new Array<number>(sensor.nRays);
  const bearings = rayBearings(sensor);
  for (let k = 0; k < sensor.nRays; k++) {
    const r = scene.raycast(x.x, x.y, bearings[k], sensor.maxRange);
    out[k] = r < sensor.maxRange ? r : Infinity;
  }
  return out;
}

/** The scan's endpoints as workspace points (finite returns only). */
export function scanPoints(sensor: RangeSensor, x: Point2, ranges: number[]): Point2[] {
  const bearings = rayBearings(sensor);
  const pts: Point2[] = [];
  for (let k = 0; k < ranges.length; k++) {
    if (!Number.isFinite(ranges[k])) continue;
    pts.push({ x: x.x + ranges[k] * Math.cos(bearings[k]), y: x.y + ranges[k] * Math.sin(bearings[k]) });
  }
  return pts;
}

/** min_k ρ_R(x, s_k) — the scan's estimate of D(x). Infinity when nothing is in range. */
export function nearestReturn(ranges: number[]): number {
  let best = Infinity;
  for (const r of ranges) if (r < best) best = r;
  return best;
}
