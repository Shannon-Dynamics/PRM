/**
 * `Trajectory<M>` — a path with a clock (Choset §1.3, §11.1), made a type.
 *
 * Owned by this chapter and imported by Chapters 19, 21 and 23: a tracker or
 * an integrator accepts a `Trajectory`, never a `Path`, so handing a geometric
 * curve to a controller is a type error rather than a runtime surprise. The
 * Rust side is `Trajectory<M: Manifold>`; here `M` is whatever the chart point
 * type is (`number[]` for Reach, `Pose2` for Rusty and Hitch).
 */

export interface TrajectorySample<M> {
  /** Time stamp, seconds from the start. */
  t: number;
  q: M;
}

export interface Trajectory<M> {
  /** Time-ordered samples; samples[0].t = 0 and the last is at t_f. */
  samples: TrajectorySample<M>[];
  tf: number;
}

/** Build a trajectory from a uniform time grid and a sampler. */
export function sampleTrajectory<M>(tf: number, dt: number, q: (t: number) => M): Trajectory<M> {
  const samples: TrajectorySample<M>[] = [];
  const steps = Math.max(1, Math.ceil(tf / dt));
  for (let k = 0; k <= steps; k++) {
    const t = Math.min(tf, k * dt);
    samples.push({ t, q: q(t) });
  }
  return { samples, tf };
}

/** Index of the last sample with t ≤ time (binary search). */
export function sampleIndexAt<M>(traj: Trajectory<M>, time: number): number {
  const s = traj.samples;
  let lo = 0;
  let hi = s.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (s[mid].t <= time) lo = mid;
    else hi = mid;
  }
  return lo;
}

/** Linear interpolation for vector-valued trajectories (chart coordinates, no wrapping). */
export function interpolateVec(traj: Trajectory<number[]>, time: number): number[] {
  const s = traj.samples;
  if (time <= s[0].t) return s[0].q.slice();
  if (time >= s[s.length - 1].t) return s[s.length - 1].q.slice();
  const k = sampleIndexAt(traj, time);
  const a = s[k];
  const b = s[Math.min(k + 1, s.length - 1)];
  const f = b.t > a.t ? (time - a.t) / (b.t - a.t) : 0;
  return a.q.map((v, i) => v + f * (b.q[i] - v));
}

/** A uniform time scaling s(t) = t / t_f applied to any path sampler — the "constant clock" of the hook. */
export function uniformTimeScaling<M>(tf: number, dt: number, q: (s: number) => M): Trajectory<M> {
  return sampleTrajectory(tf, dt, (t) => q(Math.min(1, t / tf)));
}
