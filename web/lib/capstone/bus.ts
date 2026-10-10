/**
 * The capstone's wire format (D23.1): nothing crosses a stage boundary without
 * a time and a frame on it.
 *
 * On native Rust each stage owns a thread and the bus is a set of bounded
 * `crossbeam-channel`s; in the browser — and in the WASM build of the Rust —
 * there are no threads, so the bus is a table of latest-value slots that a
 * deterministic round-robin scheduler (`stage.ts`) reads and writes inside one
 * frame. The *types* are the same either way, which is the point: a stage
 * cannot tell which runtime it is in.
 *
 * Time is integer milliseconds of simulated time. Floating-point seconds would
 * accumulate 0.05 + 0.05 + … into 0.30000000000000004 and two stages with the
 * same nominal period would drift out of phase; integer ticks keep the
 * schedule — and therefore the mission — bit-identical across runs.
 */

/** Simulated time, integer milliseconds since mission start. */
export type SimTime = number;

/**
 * The frame a value is expressed in. Rust makes this a newtype
 * (`FrameId(&'static str)`) so a planner `q` in `map` cannot be handed to the
 * `odom`-frame belief by accident; here a string-literal union does the same
 * job for the type checker.
 *
 *   map    — the Apartment's fixed frame (blueprint coordinates)
 *   odom   — Rusty's dead-reckoned frame (drifts from map; the belief corrects it)
 *   base   — a robot's own body frame (scan bearings)
 *   torus  — Reach's joint space T²
 *   bench  — the Workbench frame (tip targets)
 *   lot    — the Lot frame (Hitch and trailer)
 */
export type FrameId = 'map' | 'odom' | 'base' | 'torus' | 'bench' | 'lot';

/** D23.1: a value, the time it describes, and the frame it lives in. */
export interface Stamped<T> {
  t: SimTime;
  frame: FrameId;
  v: T;
}

export function stamp<T>(t: SimTime, frame: FrameId, v: T): Stamped<T> {
  return { t, frame, v };
}

/** Age of a message at time `now` — the staleness ς a consumer actually sees. */
export function ageOf(msg: Stamped<unknown> | null, now: SimTime): number {
  return msg === null ? Infinity : now - msg.t;
}

/**
 * Latest-value topics plus a message counter per topic. A consumer always
 * reads the newest message (what a `try_iter().last()` on a channel gives the
 * Rust side); the counters are what w23.2 turns into live message rates.
 */
export class Bus {
  private readonly slots = new Map<string, Stamped<unknown>>();
  private readonly counts = new Map<string, number>();

  publish<T>(topic: string, msg: Stamped<T>): void {
    this.slots.set(topic, msg);
    this.counts.set(topic, (this.counts.get(topic) ?? 0) + 1);
  }

  latest<T>(topic: string): Stamped<T> | null {
    return (this.slots.get(topic) as Stamped<T> | undefined) ?? null;
  }

  /** Messages published on `topic` since the mission began. */
  count(topic: string): number {
    return this.counts.get(topic) ?? 0;
  }

  topics(): string[] {
    return [...this.counts.keys()];
  }
}
